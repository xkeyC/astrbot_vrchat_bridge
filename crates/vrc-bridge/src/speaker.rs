//! Who is speaking, from where the voice comes and whose nameplate lights up
//! (no voiceprints yet). The contract with the plugin is
//! `speaker-contract.md` §1; how it works and how to calibrate it,
//! `docs/full-vr/speaker.md`.
//!
//! - **Audio** (a thread of its own, fed every 20 ms by the capture task
//!   with each block's place on the client's clock and when it was read):
//!   both ears go through
//!   `vrc-audio` (speech detector, segments on the client's sample clock,
//!   per-bin votes on the head-relative azimuth). The votes are turned by
//!   where the head looked then into a fixed frame (the tracking space's
//!   yaw), so a segment's votes add up even while the head turns, and the
//!   front/back mirror smears while the true direction stays put.
//! - **Vision** (a thread of its own, its own eye tap): players' nameplates
//!   are placed in 3D (OCR, and the panorama's depth under their rays, as
//!   the follower does; the follower's and the surveys' sightings come in
//!   too), and each plate's outline is
//!   measured against the plate's own quiet look: the ring VRChat lights
//!   round a speaking player's plate (`glow_stats`). The user camera's
//!   orbit (`orbit`) adds plates read all round the bot: a bearing alone
//!   (no distance) and the ring as that view saw it.
//! - **Fusion** per segment: every placed player is a candidate, with the
//!   segment's votes near its bearing, its plate's ring (lit as the segment
//!   began, or lit through it and not as an earlier speech's tail), and how
//!   recently it was placed; "unknown" is a candidate too. The
//!   result goes to the client as `speaker` events (every ~250 ms while
//!   the segment runs, then once more `final`).
//! - **Attending** (`POST /v1/vr/attend`): turn to the latest speech,
//!   measure again, confirm by the lit plate in the middle of the view.
//!
//! While the bot's own voice plays (and `--echo-tail-ms` after), what is
//! heard may be the bot played back by another player's open speakers, and
//! their plate glows with it: those hops' votes and those looks at the
//! plates count for nothing, and a segment heard mostly then is pinned on
//! nobody (`bot_echo`).

use std::collections::{BTreeMap, VecDeque};
use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use serde_json::{json, Value};
use vrc_audio::{mirror_deg, wrap_deg, Block, Front, FrontParams, HrirTable, Hrtf, MixTarget, MonoMix, Ring, SegEvent, SphericalHead};
use vrc_players::{OcrClient, Sighting};
use vrc_vr::tap::{format, EyeFrame, EyeTap};
use vrc_vr::Pose;

use crate::bridge::Bridge;
use crate::vr::VrCore;
use crate::{Args, Lock};

/// A segment's state is sent this often while it runs (hops of audio).
const EMIT_EVERY_HOPS: u32 = 25;
/// Kept for attending and for recordings: audio, per-hop votes.
const KEEP_AUDIO: Duration = Duration::from_secs(8);
/// Finished segments kept (`GET /v1/speakers`, attending).
const KEEP_SEGMENTS: usize = 64;
/// Votes within this of a bearing are that bearing's (degrees).
const NEAR_DEG: f32 = 8.0;
/// Bins a segment needs before its votes say anything, and before the
/// user camera's lens turns to it (a first look, checked by the plates).
const MIN_BINS: usize = 60;
const ONSET_BINS: usize = 20;
/// The lens turns to someone placed whose side (front and back folded
/// together) is within this of the voice's peak (degrees).
const SIDE_MATCH_DEG: f32 = 35.0;
/// A player placed longer ago than this, or before the bot walked
/// STALE_WALK_M (the tracking space slides past when it walks), is no
/// candidate.
const SEEN_FOR: Duration = Duration::from_secs(60);
const STALE_WALK_M: f32 = 2.0;
/// A placed plate is measured where it should be in a new frame (no OCR)
/// only this soon after it was placed.
const PROJECT_FOR: Duration = Duration::from_secs(2);
/// The watcher's frames: while someone speaks (or a recording runs), and
/// otherwise (plates' quiet looks), when other players are in the room.
const LOOK_HOT: Duration = Duration::from_millis(250);
const LOOK_QUIET: Duration = Duration::from_secs(10);
/// OCR while someone speaks: at most this often, and not when the follower
/// read the plates this recently.
const OCR_HOT: Duration = Duration::from_secs(1);
const OCR_FRESH: Duration = Duration::from_millis(600);
/// Speech ended this recently still counts as hot (the glow lags).
const HOT_AFTER: Duration = Duration::from_secs(1);
/// The ring (measured 2026-10-08, `speaker.md` 2.3): it comes on this long
/// before the voice is heard in the capture, and its onset is matched to a
/// segment's start within GLOW_ALIGN (plus half the gap between the looks
/// either side of it, itself at most GLOW_ALIGN). It stays lit through
/// pauses shorter than about a second and goes off `--glow-release-ms`
/// (0.9 s) after the speech: lit looks in that tail are no speech.
const GLOW_LEAD: Duration = Duration::from_millis(100);
const GLOW_ALIGN: Duration = Duration::from_millis(300);
/// An onset is known when an unlit look came at most this long before the
/// first lit one.
const ONSET_GAP: Duration = Duration::from_millis(1200);
/// A plate's quiet look rises toward what is seen this slowly (seconds;
/// it falls at once).
const BASELINE_RISE_S: f32 = 20.0;
/// A plate's quiet score before its first unlit look (an unlit plate
/// scores 0.1-0.2: the share of its outline a saturated background
/// touches).
const QUIET_PRIOR: f32 = 0.2;
/// A plate lit this long without a break is taken for a changed view.
const STUCK_LIT: Duration = Duration::from_secs(30);
/// Where a plate read with a bearing alone (the orbit's view) is put for
/// the candidates: this far from the head (tracking metres; only its
/// bearing counts).
const BEARING_ONLY_M: f32 = 2.5;
/// A plate placed by the depth keeps its place when a bearing-only read agrees
/// within this (degrees).
const BEARING_AGREES_DEG: f32 = 20.0;
/// A placed player's plate text when no frame said how big (tracking
/// metres: width, height).
const TEXT_SIZE: [f32; 2] = [0.25, 0.05];
/// Attending: up to this off the body the head turns alone; the turn's
/// pace; how long to listen again after.
const HEAD_ONLY_DEG: f32 = 35.0;
const TURN_S_BASE: f32 = 0.35;
const TURN_DEG_PER_S: f32 = 160.0;
const RELISTEN: Duration = Duration::from_millis(1500);
/// A plate this near the middle of the view (degrees) is the one faced.
const CENTRE_DEG: f32 = 12.0;
/// Front and back sound alike (a voice ahead often peaks right behind): a
/// candidate's votes are those near its bearing or, at this weight, near
/// its mirror (the true side, where turning keeps the votes, a little
/// likelier).
const MIRROR_WEIGHT: f32 = 0.8;
/// The event's `candidates`: at most this many, each at least this likely.
const LISTED_MAX: usize = 3;
const LISTED_P: f32 = 0.1;

/// The ring detector: lit at once (one look) when the score is `on` over
/// the plate's quiet look, unlit below `off`; the ring's tail after the
/// speech, `release_s`.
#[derive(Clone, Copy, Debug)]
pub struct GlowParams {
    pub on: f32,
    pub off: f32,
    pub release_s: f32,
}

/// Who looked at a plate: the bot's eyes, or the user camera's orbit (each
/// with its own quiet look: the views differ).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LookFrom {
    Eye = 0,
    Orbit = 1,
}

pub struct Speakers {
    pub enabled: bool,
    /// The HRTF the templates came from.
    pub hrtf: String,
    pub glow: GlowParams,
    /// From the capture to the ears' pose then (the game's audio path).
    pub latency: Duration,
    /// The bot's voice may come back through another player's speakers
    /// this long after it played out (their network and device; the
    /// playout time already has the bot's own latency).
    pub echo_tail: Duration,
    /// A candidate only the voice's direction speaks for (their plate not
    /// seen lit) is at most this likely (`--doa-only-cap`).
    pub doa_only_cap: f32,
    front: Mutex<Front>,
    /// The templates' directions (the votes' length).
    directions: usize,
    /// The mono mix, on the capture task, the way the analysis last chose.
    mixer: Mutex<MonoMix>,
    target: Mutex<MixTarget>,
    state: Mutex<State>,
    recording: Mutex<Option<Recording>>,
    /// Where recordings go (`speakers/` next to the token).
    dir: PathBuf,
}

#[derive(Default)]
struct State {
    /// The client whose clock the segments are on (0: none).
    client: u64,
    /// The head as sent (when, yaw, position), the last few seconds.
    heads: VecDeque<(Instant, f32, [f32; 3])>,
    hops: VecDeque<HopRec>,
    /// Both ears, interleaved, the last KEEP_AUDIO.
    audio: VecDeque<f32>,
    open: Option<Seg>,
    segments: VecDeque<Seg>,
    players: BTreeMap<String, Seen>,
    plates: BTreeMap<String, Plate>,
    /// The other players in the room: (id, name).
    room: Vec<(String, String)>,
    /// How far the bot walked (world metres, its own speed integrated).
    walked: f32,
    /// When plates were last read (by anyone).
    read_at: Option<Instant>,
    bot_until: Option<Instant>,
}

/// One analysed hop.
struct HopRec {
    t: Instant,
    voiced: bool,
    /// Heard while the bot's voice may be playing back.
    echo: bool,
    /// Head-relative votes (voiced hops).
    votes: Option<Vec<f32>>,
    bins: usize,
    yaw: f32,
}

#[derive(Clone)]
struct Seg {
    id: u64,
    client: u64,
    start: u64,
    end: u64,
    t0: Instant,
    t1: Instant,
    /// Votes in the fixed frame (tracking-space yaw).
    world: Ring,
    bins: usize,
    /// Where the head looked as it began.
    yaw0: f32,
    /// Voiced hops, and of them those heard while the bot's voice may be
    /// playing back (their votes are left out); mostly those: `bot_echo`.
    voiced: u32,
    echoed: u32,
    bot_echo: bool,
    /// Hops of audio since it opened, and at its last event.
    hops: u32,
    emitted: Option<u32>,
    sent: bool,
    done: bool,
    who: Option<Attribution>,
}

#[derive(Clone, Debug)]
struct Attribution {
    name: Option<String>,
    user_id: Option<String>,
    /// The source's yaw in the fixed frame, and relative to the head when
    /// attributed.
    world_yaw: Option<f32>,
    bearing: Option<f32>,
    confidence: f32,
    cues: Value,
    candidates: Value,
    /// The event's `candidates`: (name, or None for someone not
    /// recognised; how likely), likeliest first.
    ranked: Vec<(Option<String>, f32)>,
}

/// A player placed by their nameplate.
#[derive(Clone, Copy, Debug)]
struct Seen {
    tag: [f32; 3],
    at: Instant,
    walked: f32,
    /// The plate's text, tracking metres (width, height).
    size: [f32; 2],
    /// Read with a bearing alone (the orbit's view): `tag` is put
    /// BEARING_ONLY_M away that way, and is not projected into frames.
    bearing_only: bool,
}

#[derive(Default)]
struct Plate {
    /// The quiet look, per who looks (`LookFrom`).
    base: [Option<f32>; 2],
    /// The last look's score over its quiet look.
    over: f32,
    on: bool,
    /// Since when it has been lit without a break.
    on_since: Option<Instant>,
    /// Looks, oldest first (the last minute).
    history: VecDeque<Look>,
}

/// One look at a plate.
#[derive(Clone, Copy, Debug)]
struct Look {
    t: Instant,
    on: bool,
    /// While the bot's voice may be playing back.
    echo: bool,
    from: LookFrom,
}

/// A plate's ring for one segment.
#[derive(Clone, Copy, Debug, PartialEq)]
struct GlowCue {
    /// It came on as the segment began (GLOW_LEAD before, within GLOW_ALIGN).
    onset: bool,
    /// The share of looks during the segment that saw it lit, the ring's
    /// tail after an earlier speech left out.
    share: f32,
    /// Of the lit looks that count (the onset's, those during), one was the
    /// bot's own eyes' (else all were the user camera's lens).
    eye: bool,
}

impl GlowCue {
    /// The ring speaks for its player: lit as the segment began, or through
    /// it.
    fn lit(&self) -> bool {
        self.onset || self.share > 0.0
    }
}

/// A candidate for one segment.
#[derive(Clone, Debug)]
struct Cand {
    name: String,
    user_id: Option<String>,
    world_yaw: f32,
    age_s: f32,
    /// The plate's ring during the segment (None: not seen then).
    glow: Option<GlowCue>,
}

/// What a nameplate's outline looks like in one frame (`glow_stats`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GlowStats {
    /// The share of the outline's samples whose band (just outside the
    /// pill's dark fill) holds a strongly saturated, bright pixel of the
    /// outline's most common hue: the ring, whatever its colour.
    pub ring: f32,
    /// Means over the samples of the band's most saturated pixel: chroma
    /// (max - min) and brightness (max), 0..1.
    pub chroma: f32,
    pub value: f32,
    /// The share of strongly saturated pixels of the ring's hue beside the
    /// pill's ends: the ripple arcs (0 without a ring hue).
    pub ripple: f32,
    /// The ring's hue (degrees), when some samples are strong.
    pub hue: Option<f32>,
    /// The share of samples where the pill's edge was found.
    pub edges: f32,
}

impl GlowStats {
    /// The score compared with the plate's quiet look (unlit 0.1-0.2, lit
    /// about 1 on the 2026-10-08 recording).
    pub fn score(&self) -> f32 {
        self.ring + 0.5 * self.chroma + 0.25 * self.ripple
    }

    fn json(&self) -> Value {
        json!({"ring": r3(self.ring), "chroma": r3(self.chroma), "value": r3(self.value), "ripple": r3(self.ripple), "hue": self.hue.map(r1), "edges": r2(self.edges), "score": r3(self.score())})
    }
}

/// A speech for the user camera's lens to turn to (`Speakers::voice`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Voice {
    pub segment: u64,
    /// Going on, and when it began and was last heard.
    pub open: bool,
    pub t0: Instant,
    pub t1: Instant,
    /// Where it comes from (the tracking space's yaw), and its front/back
    /// mirror when the votes there are about as many (look there next).
    pub yaw: f32,
    pub mirror: Option<f32>,
    /// `candidate` (pinned on a placed player) or `direction` (the votes).
    pub from: &'static str,
}

/// What `/v1/vr/attend` turns to.
#[derive(Clone, Debug)]
pub struct Source {
    pub name: Option<String>,
    pub user_id: Option<String>,
    pub world_yaw: f32,
    pub from: &'static str,
    pub segment: u64,
    /// Which speech it was: `named` (pinned on the name asked for),
    /// `ended` (the latest that ended) or `running` (going on).
    pub picked: &'static str,
}

impl Speakers {
    pub fn new(args: &Args) -> Speakers {
        let head = load_head(&args.hrtf_table);
        let hrtf = head.name();
        let mut front = Front::new(head.as_ref(), FrontParams::default());
        front.measure = args.speakers;
        if args.speakers {
            tracing::info!("speakers: direction by {hrtf}");
        }
        let dir = crate::game::expand(&args.token_file).parent().map(|p| p.join("speakers")).unwrap_or_else(|| "speakers".into());
        Speakers {
            enabled: args.speakers,
            hrtf,
            glow: GlowParams { on: args.glow_on, off: args.glow_off, release_s: args.glow_release_ms as f32 / 1000.0 },
            latency: Duration::from_millis(args.audio_latency_ms),
            echo_tail: Duration::from_millis(args.echo_tail_ms),
            doa_only_cap: args.doa_only_cap.clamp(0.0, 1.0),
            directions: front.doa.templates.len(),
            target: Mutex::new(front.policy.target()),
            mixer: Mutex::new(MonoMix::default()),
            front: Mutex::new(front),
            state: Mutex::new(State::default()),
            recording: Mutex::new(None),
            dir,
        }
    }

    // -- audio ------------------------------------------------------------------

    /// A block of both ears as 16-bit mono for the client (on the capture
    /// task: the mix alone, the way the analysis of the blocks before chose).
    pub fn mix(&self, left: &[f32], right: &[f32]) -> Vec<i16> {
        let target = *self.target.lk();
        self.mixer.lk().process(left, right, target)
    }

    /// Analyses a block (on the analysis thread, after its mix was sent or
    /// dropped): it starts at `clock` on client `client`'s sample clock and
    /// reached it if `counted`; it was read at `now`. `head`: the head as
    /// last sent; `bot_until`: when the bot's voice plays out. The
    /// `speaker` events for the client.
    #[allow(clippy::too_many_arguments)]
    pub fn analyse(
        &self,
        left: &[f32],
        right: &[f32],
        clock: u64,
        counted: bool,
        client: u64,
        head: Option<(Instant, Pose)>,
        bot_until: Option<Instant>,
        now: Instant,
    ) -> Vec<Value> {
        {
            let mut st = self.state.lk();
            st.bot_until = bot_until;
            if let Some((at, pose)) = head {
                if st.heads.back().is_none_or(|h| h.0 != at) {
                    st.heads.push_back((at, pose.yaw_pitch().0, pose.position));
                }
            }
            while st.heads.len() > 2 && st.heads.front().is_some_and(|h| now.saturating_duration_since(h.0) > KEEP_AUDIO) {
                st.heads.pop_front();
            }
            for (l, r) in left.iter().zip(right) {
                st.audio.push_back(*l);
                st.audio.push_back(*r);
            }
            let keep = 2 * (vrc_audio::RATE as usize) * KEEP_AUDIO.as_secs() as usize;
            while st.audio.len() > keep {
                st.audio.pop_front();
            }
            if client != st.client {
                // A new client: its clock starts here (at 0).
                st.client = client;
                self.front.lk().segmenter.restart_clock(clock);
                if let Some(o) = st.open.as_mut() {
                    (o.client, o.start, o.end, o.sent) = (client, clock, clock, false);
                }
            }
        }
        // The analysis itself, with the state free for the others.
        let hops = {
            let mut front = self.front.lk();
            let hops = front.analyse(&Block { left, right }, clock, counted);
            *self.target.lk() = front.policy.target();
            hops
        };
        let mut st = self.state.lk();
        let n = hops.len();
        let mut out = Vec::new();
        for (i, h) in hops.into_iter().enumerate() {
            // What this hop was rendered for: the head then.
            let t = now.checked_sub(self.latency + Duration::from_millis(10 * (n - 1 - i) as u64)).unwrap_or(now);
            let (yaw, _) = head_at(&st.heads, t);
            let echo = bot_until.is_some_and(|b| t < b + self.echo_tail);
            for ev in &h.events {
                match *ev {
                    SegEvent::Opened { id, start } => {
                        st.open = Some(Seg {
                            id,
                            client,
                            start,
                            end: h.end,
                            t0: t,
                            t1: t,
                            world: Ring::zeros(self.directions),
                            bins: 0,
                            yaw0: yaw,
                            voiced: 0,
                            echoed: 0,
                            bot_echo: false,
                            hops: 0,
                            emitted: None,
                            sent: false,
                            done: false,
                            who: None,
                        });
                    }
                    SegEvent::Closed { id, start, end } => {
                        if let Some(mut seg) = st.open.take().filter(|s| s.id == id) {
                            (seg.start, seg.end, seg.t1, seg.done) = (start, end, t, true);
                            let who = self.attribute(&st, &seg, now);
                            if client != 0 && seg.client == client {
                                out.push(event(&seg, &who, true));
                            }
                            seg.who = Some(who);
                            st.segments.push_back(seg);
                            while st.segments.len() > KEEP_SEGMENTS {
                                st.segments.pop_front();
                            }
                        }
                    }
                    SegEvent::Dropped { id } => {
                        if let Some(seg) = st.open.take().filter(|s| s.id == id) {
                            if seg.sent && client != 0 && seg.client == client {
                                let none = Attribution::unknown(json!({"dropped": true}), 0.0);
                                out.push(event(&seg, &none, true));
                            }
                        }
                    }
                }
            }
            if let Some(seg) = st.open.as_mut() {
                seg.end = h.end;
                seg.t1 = t;
                seg.hops += 1;
                if h.level.voiced {
                    seg.voiced += 1;
                    seg.echoed += echo as u32;
                }
                // The bot's own voice played back: mostly that, the whole
                // segment is nobody's; else only those hops count for nothing.
                seg.bot_echo = 2 * seg.echoed > seg.voiced;
                if let (Some(d), false) = (&h.doa, echo) {
                    seg.world.add_turned(&d.votes, yaw, 1.0);
                    seg.bins += d.bins;
                }
            }
            if let Some(rec) = self.recording.lk().as_mut() {
                rec.hop(&h, t, yaw, echo, st.open.as_ref().map(|s| s.id));
            }
            st.hops.push_back(HopRec { t, voiced: h.level.voiced, echo, bins: h.doa.as_ref().map_or(0, |d| d.bins), votes: h.doa.map(|d| d.votes), yaw });
        }
        while st.hops.front().is_some_and(|h| now.saturating_duration_since(h.t) > KEEP_AUDIO) {
            st.hops.pop_front();
        }
        // The open segment, now and then.
        let due = st.open.as_ref().is_some_and(|s| s.hops >= EMIT_EVERY_HOPS && s.emitted.is_none_or(|e| s.hops - e >= EMIT_EVERY_HOPS));
        if due && self.enabled {
            let seg = st.open.clone().unwrap();
            let who = self.attribute(&st, &seg, now);
            if client != 0 && seg.client == client {
                out.push(event(&seg, &who, false));
            }
            let o = st.open.as_mut().unwrap();
            (o.emitted, o.sent, o.who) = (Some(o.hops), client != 0, Some(who));
        }
        drop(st);
        let done = self.recording.lk().as_mut().is_some_and(|r| {
            r.audio(left, right);
            now >= r.until
        });
        if done {
            self.finish_recording();
        }
        if self.enabled {
            out
        } else {
            Vec::new()
        }
    }

    // -- fusion -------------------------------------------------------------------

    /// Who said `seg`, as far as can be told now.
    fn attribute(&self, st: &State, seg: &Seg, now: Instant) -> Attribution {
        if seg.bot_echo {
            return Attribution::unknown(json!({"bot_echo": true}), 0.0);
        }
        let (head_yaw, head_pos) = head_at(&st.heads, now);
        let ring = (seg.bins >= MIN_BINS).then_some(&seg.world);
        let mut cands = candidates(st, now, head_pos);
        for c in cands.iter_mut() {
            c.glow = st.plates.get(&c.name).and_then(|p| p.cue(seg.t0, seg.t1, &self.glow));
        }
        let unplaced = st.room.iter().filter(|(_, n)| !cands.iter().any(|c| &c.name == n)).count();
        let fused = fuse(ring, &cands, unplaced, seg.yaw0, self.doa_only_cap);
        let peak = ring.and_then(|r| r.peaks(NEAR_DEG, 20.0, 1).first().copied());
        let best_mass = ring.map(|r| r.best_mass(NEAR_DEG).1).unwrap_or(0.0);
        let doa = |c: &Cand| ring.map(|r| r2(doa_mass(r, c.world_yaw, seg.yaw0) / best_mass.max(1e-6)));
        let candidates: Vec<Value> = cands
            .iter()
            .zip(&fused.posteriors)
            .map(|(c, p)| {
                json!({
                    "name": c.name,
                    "bearing_deg": r1(wrap_deg(c.world_yaw - head_yaw)),
                    "doa": doa(c),
                    "glow": c.glow.map(|g| r2(g.share)),
                    "glow_onset": c.glow.map(|g| g.onset),
                    "evidence": evidence(c, ring.is_some()),
                    "age_s": r1(c.age_s),
                    "p": r2(*p),
                })
            })
            .collect();
        let candidates = json!({"list": candidates, "unknown_p": r2(fused.unknown), "unplaced": unplaced, "doa_peak_deg": peak.map(|p| r1(wrap_deg(p.0 - head_yaw))), "bins": seg.bins});
        let ranked = fused.ranked(&cands);
        match fused.best {
            Some(i) => {
                let c = &cands[i];
                Attribution {
                    name: Some(c.name.clone()),
                    user_id: c.user_id.clone(),
                    world_yaw: Some(c.world_yaw),
                    bearing: Some(wrap_deg(c.world_yaw - head_yaw)),
                    confidence: fused.posteriors[i],
                    cues: json!({"doa": doa(c), "glow": c.glow.map(|g| r2(g.share)), "glow_onset": c.glow.map(|g| g.onset), "evidence": evidence(c, ring.is_some())}),
                    candidates,
                    ranked,
                }
            }
            None => Attribution {
                name: None,
                user_id: None,
                world_yaw: peak.map(|p| p.0),
                bearing: peak.map(|p| wrap_deg(p.0 - head_yaw)),
                confidence: fused.unknown,
                cues: json!({"doa": peak.map(|p| r2(p.1 / best_mass.max(1e-6))), "evidence": if ring.is_some() { "direction" } else { "none" }}),
                candidates,
                ranked,
            },
        }
    }

    /// Whether someone is speaking now (a segment open).
    pub fn speaking(&self) -> bool {
        self.state.lk().open.is_some()
    }

    /// The latest speech that is not the bot's own echo, going on or ended
    /// within `since`, once it has a bearing: its player's when it is
    /// pinned on one, else its votes' peak (ONSET_BINS are enough to aim a
    /// look; of a peak and its mirror much alike, the side with a placed
    /// player first, the other as `mirror`).
    pub fn voice(&self, since: Duration) -> Option<Voice> {
        let now = Instant::now();
        let st = self.state.lk();
        let seg = st.open.as_ref().or_else(|| st.segments.back().filter(|s| now.saturating_duration_since(s.t1) <= since))?;
        if seg.bot_echo {
            return None;
        }
        let (_, head_pos) = head_at(&st.heads, now);
        let cands = candidates(&st, now, head_pos);
        let voice = |yaw: f32, mirror: Option<f32>, from| Voice { segment: seg.id, open: !seg.done, t0: seg.t0, t1: seg.t1, yaw: wrap_deg(yaw), mirror: mirror.map(wrap_deg), from };
        if let Some(name) = seg.who.as_ref().and_then(|w| w.name.as_ref()) {
            if let Some(c) = cands.iter().find(|c| &c.name == name) {
                return Some(voice(c.world_yaw, None, "candidate"));
            }
        }
        if seg.bins < ONSET_BINS {
            return None;
        }
        let peak = seg.world.peaks(NEAR_DEG, 20.0, 1).first().copied()?;
        // Front and back sound alike (measured in game: a voice ahead often
        // peaks right behind): someone placed whose side matches the peak's
        // is likelier than the peak itself, so look where they are.
        if let Some(c) = side_match(&cands, peak.0, seg.yaw0) {
            return Some(voice(c.world_yaw, None, "candidate side"));
        }
        let mirror = seg.yaw0 + mirror_deg(peak.0 - seg.yaw0);
        let alike = wrap_deg(mirror - peak.0).abs() > 2.0 * NEAR_DEG && seg.world.mass_near(mirror, NEAR_DEG) > 0.6 * peak.1;
        if !alike {
            return Some(voice(peak.0, None, "direction"));
        }
        let near = |yaw: f32| cands.iter().any(|c| wrap_deg(c.world_yaw - yaw).abs() < 20.0);
        Some(if !near(peak.0) && near(mirror) { voice(mirror, Some(peak.0), "direction") } else { voice(peak.0, Some(mirror), "direction") })
    }

    /// A plate's ring measured again where it was read just before (the
    /// lens turned to a voice holds still: every frame between reads).
    pub fn saw_glow(&self, name: &str, at: Instant, g: GlowStats) {
        if !self.enabled {
            return;
        }
        let mut st = self.state.lk();
        let echo = st.bot_until.is_some_and(|b| at < b + self.echo_tail);
        self.plate_seen(&mut st, name, g, LookFrom::Orbit, at, echo, None, "lens");
    }

    /// Whether `name`'s plate was seen lit at its last look, within `within`.
    pub fn plate_lit(&self, name: &str, within: Duration) -> bool {
        let now = Instant::now();
        let st = self.state.lk();
        st.plates.get(name).and_then(|p| p.history.back()).is_some_and(|h| h.on && !h.echo && now.saturating_duration_since(h.t) <= within)
    }

    /// Whether `name` was heard speaking just now (a segment running or
    /// ended within 4 s pinned on them).
    pub fn talking(&self, name: &str) -> bool {
        let st = self.state.lk();
        let now = Instant::now();
        let pinned = |s: &Seg| s.who.as_ref().is_some_and(|w| w.name.as_deref() == Some(name));
        st.open.as_ref().is_some_and(pinned) || st.segments.iter().any(|s| pinned(s) && now.duration_since(s.t1) < Duration::from_secs(4))
    }

    // -- vision ---------------------------------------------------------------------

    /// Players placed by someone's look (the follower, a survey, the
    /// watcher): where their plates are; with the frame they were read
    /// from, their plates' glow too.
    pub fn saw(&self, seen: &[Sighting], frame: Option<&EyeFrame>) {
        if !self.enabled || seen.is_empty() {
            return;
        }
        let now = Instant::now();
        let mut st = self.state.lk();
        st.read_at = Some(now);
        let echo = st.bot_until.is_some_and(|b| now < b + self.echo_tail);
        for s in seen {
            let size = frame.and_then(|f| text_size(f, s.tag, s.bbox)).unwrap_or(TEXT_SIZE);
            let walked = st.walked;
            st.players.insert(s.name.clone(), Seen { tag: s.tag, at: now, walked, size, bearing_only: false });
            if let Some(f) = frame {
                if let Some(g) = View::eye(f).and_then(|v| glow_stats(&v, s.bbox)) {
                    self.plate_seen(&mut st, &s.name, g, LookFrom::Eye, now, echo, View::eye(f).map(|v| (v, s.bbox)), "ocr");
                }
            }
        }
    }

    /// A player's plate read with a bearing alone (the user camera's orbit:
    /// no distance), `yaw` in the tracking space's frame, at `at`; with the
    /// plate's ring as that view saw it. A plate placed by the depth whose
    /// bearing agrees keeps its place (and counts as seen now); else the
    /// player is put BEARING_ONLY_M away that way.
    pub fn saw_bearing(&self, name: &str, yaw: f32, at: Instant, glow: Option<GlowStats>) {
        if !self.enabled {
            return;
        }
        let mut st = self.state.lk();
        let (_, head) = head_at(&st.heads, at);
        let walked = st.walked;
        let agrees = st
            .players
            .get(name)
            .is_some_and(|s| !s.bearing_only && at.saturating_duration_since(s.at) <= SEEN_FOR && wrap_deg(yaw_to(head, s.tag) - yaw).abs() <= BEARING_AGREES_DEG);
        if agrees {
            let s = st.players.get_mut(name).expect("checked");
            s.at = s.at.max(at);
            s.walked = walked;
        } else {
            let (sn, cs) = yaw.to_radians().sin_cos();
            let tag = [head[0] + BEARING_ONLY_M * sn, head[1] + 0.3, head[2] - BEARING_ONLY_M * cs];
            st.players.insert(name.to_string(), Seen { tag, at, walked, size: TEXT_SIZE, bearing_only: true });
        }
        if let Some(g) = glow {
            let echo = st.bot_until.is_some_and(|b| at < b + self.echo_tail);
            self.plate_seen(&mut st, name, g, LookFrom::Orbit, at, echo, None, "orbit");
        }
    }

    /// A plate measured: its ring detector moves on, and a recording keeps it.
    #[allow(clippy::too_many_arguments)]
    fn plate_seen(&self, st: &mut State, name: &str, g: GlowStats, from: LookFrom, now: Instant, echo: bool, crop: Option<(View, [f32; 4])>, how: &str) {
        let p = st.plates.entry(name.to_string()).or_default();
        p.update(now, g.score(), from, &self.glow, echo);
        if let Some(rec) = self.recording.lk().as_mut() {
            let (base, over, on) = (p.base[from as usize], p.over, p.on);
            rec.glow(now, name, how, &g, base, over, on, echo, crop);
        }
    }

    /// The watcher (a thread): frames of its own while someone speaks (and
    /// now and then otherwise), the plates placed and measured.
    pub fn watch(self: Arc<Self>, b: Arc<Bridge>) {
        let mut tap = EyeTap::open(&b.args.tap);
        let ocr = OcrClient::new(&b.args.ocr_url, &b.args.ocr_model).map_err(|e| tracing::warn!("speakers: no OCR: {e:#}")).ok();
        let mut last_tick = Instant::now();
        let mut looked: Option<Instant> = None;
        let mut read: Option<Instant> = None;
        loop {
            std::thread::sleep(Duration::from_millis(100));
            let now = Instant::now();
            let dt = now.duration_since(last_tick).as_secs_f32();
            last_tick = now;
            let speed = b.anim.live.lk()["speed"].as_f64().unwrap_or(0.0) as f32;
            let (running, room) = {
                let g = b.game.lk();
                (g.running, g.others())
            };
            let recording = self.recording.lk().is_some();
            let (hot, fresh, echo) = {
                let mut st = self.state.lk();
                st.walked += speed.abs() * dt;
                st.room = room.clone();
                let hot = recording || st.open.is_some() || st.segments.back().is_some_and(|s| now.duration_since(s.t1) < HOT_AFTER);
                (hot, st.read_at.is_some_and(|t| now.duration_since(t) < OCR_FRESH), st.bot_until.is_some_and(|t| now < t + self.echo_tail))
            };
            if !running || (room.is_empty() && !recording) {
                continue;
            }
            if looked.is_some_and(|t| now.duration_since(t) < if hot { LOOK_HOT } else { LOOK_QUIET }) {
                continue;
            }
            looked = Some(now);
            // Asking Monado to tap costs it a copy of every frame: only while
            // someone speaks, or when nobody else (the follower) has it
            // tapping already.
            let frame = if hot || !tap.tapping() { tap.read() } else { tap.read_unasked() };
            let frame = match frame {
                Ok(Some(f)) => f,
                _ => continue,
            };
            // The plates are read off a pano frame only (the ones over the
            // eyes and the lens's, placed in its depth; decision D42): the
            // usual view (leased for a menu) has no depth to place them by.
            let due = !hot || read.is_none_or(|t| now.duration_since(t) >= OCR_HOT);
            if let (Some(ocr), true, false, true) = (&ocr, due, fresh, vrc_pano::classify(&frame).is_pano()) {
                read = Some(now);
                match self.read_plates_pano(&b, ocr) {
                    Ok(()) => continue,
                    Err(e) => tracing::debug!("speakers: reading plates failed: {e:#}"),
                }
            }
            // No OCR this time: the plates placed just now, where they
            // should be in this frame.
            let mut st = self.state.lk();
            let placed: Vec<(String, Seen)> =
                st.players.iter().filter(|(_, s)| !s.bearing_only && now.saturating_duration_since(s.at) < PROJECT_FOR).map(|(n, s)| (n.clone(), *s)).collect();
            let Some(view) = View::eye(&frame) else { continue };
            for (name, s) in placed {
                if let Some(bbox) = project(&frame, s.tag, s.size) {
                    if let Some(g) = glow_stats(&view, bbox) {
                        self.plate_seen(&mut st, &name, g, LookFrom::Eye, now, echo, Some((view, bbox)), "projected");
                    }
                }
            }
        }
    }

    /// Reads the nameplates with the panorama on: the ones VRChat draws over
    /// the eyes (measured there, `saw` with the frame) and the lens's of
    /// the last second, placed in the panorama's depth (`panolook`).
    fn read_plates_pano(&self, b: &Bridge, ocr: &OcrClient) -> Result<()> {
        let room = crate::panolook::room(b);
        let o = crate::panolook::LookOptions { lens_within: Duration::from_secs(1), ..Default::default() };
        let l = crate::panolook::look(b, &room, Some(ocr), &o)?;
        l.to_speakers(b, &b.social.whitelist_names());
        Ok(())
    }

    // -- attending --------------------------------------------------------------------

    /// What to turn to, of the speech with votes enough that is not the
    /// bot's own echo:
    /// - with `name` (who the transcript says spoke): the latest pinned on
    ///   them, going on or ended within `since`;
    /// - else, or none pinned on them, with `ended_first` (a transcript:
    ///   it comes after its speech, and what goes on now may be someone
    ///   else's) the latest that ended within `since`, else the speech
    ///   going on;
    /// - else (a wake word: heard while its speech goes on) the speech
    ///   going on, else the latest that ended within `since`.
    ///
    /// Then its player's bearing now if it was pinned on one; else someone
    /// placed (by a look, the lens, the idle sweep) on the voice's side,
    /// front and back folded (`name` first: who the transcript says spoke);
    /// else its votes' peak (of a peak and its front/back mirror much alike,
    /// the side with a player).
    pub fn source(&self, since: Duration, name: Option<&str>, ended_first: bool) -> Option<Source> {
        let now = Instant::now();
        let st = self.state.lk();
        let usable = |s: &&Seg| s.bins >= MIN_BINS && !s.bot_echo;
        let running = st.open.as_ref().filter(usable);
        let ended = || st.segments.iter().rev().filter(|s| now.saturating_duration_since(s.t1) <= since).find(usable);
        let named = name.and_then(|n| {
            let pinned = |s: &&Seg| s.who.as_ref().is_some_and(|w| w.name.as_deref() == Some(n));
            running.filter(pinned).or_else(|| st.segments.iter().rev().filter(|s| now.saturating_duration_since(s.t1) <= since).filter(usable).find(pinned))
        });
        let (seg, picked) = match named {
            Some(s) => (s, "named"),
            None if ended_first => ended().map(|s| (s, "ended")).or_else(|| running.map(|s| (s, "running")))?,
            None => running.map(|s| (s, "running")).or_else(|| ended().map(|s| (s, "ended")))?,
        };
        let (_, head_pos) = head_at(&st.heads, now);
        let cands = candidates(&st, now, head_pos);
        let who = seg.who.clone().unwrap_or_else(|| self.attribute(&st, seg, now));
        if let Some(name) = &who.name {
            if let Some(c) = cands.iter().find(|c| &c.name == name) {
                return Some(Source { name: Some(name.clone()), user_id: who.user_id.clone(), world_yaw: c.world_yaw, from: "candidate", segment: seg.id, picked });
            }
        }
        let peak = seg.world.peaks(NEAR_DEG, 20.0, 1).first().copied()?;
        // Someone placed (by a look, the lens, the idle sweep) on the
        // voice's side, front and back folded: the one the transcript names
        // first, else the nearest.
        let named = name.and_then(|n| cands.iter().find(|c| c.name == n)).filter(|c| side_match(std::slice::from_ref(*c), peak.0, seg.yaw0).is_some());
        if let Some(c) = named {
            return Some(Source { name: Some(c.name.clone()), user_id: c.user_id.clone(), world_yaw: c.world_yaw, from: "candidate side", segment: seg.id, picked });
        }
        if let Some(c) = side_match(&cands, peak.0, seg.yaw0) {
            return Some(Source { name: None, user_id: None, world_yaw: c.world_yaw, from: "candidate side", segment: seg.id, picked });
        }
        // The mirror about the ears' axis as the head was then.
        let mirror = seg.yaw0 + mirror_deg(peak.0 - seg.yaw0);
        let near = |yaw: f32| cands.iter().any(|c| wrap_deg(c.world_yaw - yaw).abs() < 20.0);
        let alike = seg.world.mass_near(mirror, NEAR_DEG) > 0.6 * peak.1;
        let world_yaw = if alike && !near(peak.0) && near(mirror) { wrap_deg(mirror) } else { peak.0 };
        Some(Source { name: None, user_id: None, world_yaw, from: "direction", segment: seg.id, picked })
    }

    /// Head-relative votes of the voiced hops after `from` heard with the
    /// head within 4 degrees of `yaw`: (votes, bins).
    fn votes_since(&self, from: Instant, yaw: f32) -> (Ring, usize) {
        let st = self.state.lk();
        let n = self.directions;
        let mut ring = Ring::zeros(n);
        let mut bins = 0;
        for h in st.hops.iter().filter(|h| h.t >= from && h.voiced && !h.echo && wrap_deg(h.yaw - yaw).abs() < 4.0) {
            if let Some(v) = &h.votes {
                ring.add_turned(v, 0.0, 1.0);
                bins += h.bins;
            }
        }
        (ring, bins)
    }

    /// Whether `name`'s plate is lit now: `g` over its quiet look, or lit in
    /// the last moments.
    fn lit(&self, name: &str, g: Option<GlowStats>) -> bool {
        let now = Instant::now();
        let st = self.state.lk();
        let Some(p) = st.plates.get(name) else { return false };
        let over = g.is_some_and(|g| g.score() - p.base[LookFrom::Eye as usize].unwrap_or(QUIET_PRIOR) > self.glow.on);
        over || p.history.iter().any(|h| h.on && !h.echo && now.saturating_duration_since(h.t) < Duration::from_millis(1500))
    }

    /// Turns to `src` (the head alone when near the body's facing, else the
    /// whole bot, at an even pace), listens again: the voice should be
    /// straight ahead now (behind: the first turn went for the mirror, so
    /// turn round; off to a side: correct once), then looks for a lit plate
    /// in the middle of the view.
    pub fn attend(&self, vr: &mut VrCore, src: &Source) -> Result<Value> {
        let before = vr.yaw;
        let bearing = wrap_deg(src.world_yaw - before);
        turn_to(vr, src.world_yaw)?;
        let mut facing = src.world_yaw;
        let mut residual = None;
        let mut fixed = None;
        let since = Instant::now() + self.latency;
        std::thread::sleep(Duration::from_millis(250));
        let until = Instant::now() + RELISTEN;
        while Instant::now() < until {
            std::thread::sleep(Duration::from_millis(100));
            let (ring, bins) = self.votes_since(since, facing);
            if bins < MIN_BINS {
                continue;
            }
            let (ahead, behind) = (ring.mass_near(0.0, NEAR_DEG + 2.0), ring.mass_near(180.0, NEAR_DEG + 2.0));
            let top = ring.peaks(NEAR_DEG, 20.0, 1).first().map(|p| p.0).unwrap_or(0.0);
            residual = Some(top);
            if behind > 1.3 * ahead && behind > 0.1 {
                fixed = Some("mirror");
                facing = wrap_deg(facing + 180.0);
            } else if top.abs() > 10.0 && top.abs() < 150.0 {
                fixed = Some("corrected");
                facing = wrap_deg(facing + top);
            }
            break;
        }
        if fixed.is_some() {
            turn_to(vr, facing)?;
        }
        // The plate in the middle of the view, and whether it is lit: only a
        // confirmation, so a failure here is logged and confirms nothing
        // (the turn is done; the headset is fine).
        let centred = self.centred_plate(vr).unwrap_or_else(|e| {
            tracing::info!("speakers: attend: no plate to confirm by: {e:#}");
            None
        });
        let ahead = residual.is_none_or(|r: f32| r.abs() <= CENTRE_DEG || fixed.is_some());
        let confirmed = ahead && centred.as_ref().is_some_and(|c| c.1 && src.name.as_ref().is_none_or(|n| n == &c.0));
        let name = src.name.clone().or_else(|| centred.as_ref().filter(|c| c.1).map(|c| c.0.clone()));
        Ok(json!({
            "ok": true,
            "name": name,
            "user_id": src.user_id,
            "bearing_deg": r1(bearing),
            "confirmed": confirmed,
            "from": src.from,
            "picked": src.picked,
            "segment": src.segment,
            "residual_deg": residual.map(r1),
            "fixed": fixed,
            "plate": centred.map(|c| json!({"name": c.0, "lit": c.1})),
        }))
    }

    /// The nameplate read in the middle of the view now, and whether it is
    /// lit (of several, a lit one).
    fn centred_plate(&self, vr: &mut VrCore) -> Result<Option<(String, bool)>> {
        let frame = vr.frame()?;
        let names: Vec<String> = self.state.lk().room.iter().map(|(_, n)| n.clone()).collect();
        let Some(ocr) = vr.rig(&[])?.ocr.clone() else { return Ok(None) };
        let [fx, _, cx, _] = frame.views[0].fov.intrinsics(frame.width, frame.height);
        let lines = ocr.lines_rgb(&frame.eye_rgb8(0)?, frame.width as u16, frame.height as u16)?;
        let view = View::eye(&frame).context("a frame without its pixels")?;
        let mut centred: Option<(String, bool)> = None;
        for l in &lines {
            let Some((i, _)) = vrc_players::names::best_match(&l.text, &names) else { continue };
            let off = ((l.bbox[0] + l.bbox[2] / 2.0 - cx) / fx).atan().to_degrees();
            if off.abs() > CENTRE_DEG {
                continue;
            }
            let lit = self.lit(&names[i], glow_stats(&view, l.bbox));
            if centred.as_ref().is_none_or(|c| !c.1 && lit) {
                centred = Some((names[i].clone(), lit));
            }
        }
        Ok(centred)
    }

    // -- recordings --------------------------------------------------------------

    /// Records `seconds` from now (and what the ring holds before) for
    /// tuning: both ears, every hop's detector and votes, every plate
    /// measured (and a crop of it), and the segments. The recording's own
    /// thread encodes and writes it.
    pub fn record(&self, seconds: f32) -> Result<Value> {
        anyhow::ensure!(self.enabled, "the speaker tracker is off (--speakers false)");
        anyhow::ensure!((1.0..=600.0).contains(&seconds), "seconds is 1-600");
        // What the ring holds goes first (the state before the recording's
        // lock: the audio path takes them the other way round).
        let preroll: Vec<f32> = self.state.lk().audio.iter().copied().collect();
        let (step, el) = {
            let f = self.front.lk();
            (f.doa.templates.step, f.doa.templates.el)
        };
        let mut rec = self.recording.lk();
        anyhow::ensure!(rec.is_none(), "a recording is running");
        let stamp = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_millis());
        let dir = self.dir.join(format!("rec-{stamp}"));
        let r = Recording::start(
            &dir,
            seconds,
            preroll,
            self.directions,
            json!({
                "hrtf": self.hrtf,
                "glow": {"on": self.glow.on, "off": self.glow.off, "release_s": self.glow.release_s},
                "latency_ms": self.latency.as_millis() as u64,
                "echo_tail_ms": self.echo_tail.as_millis() as u64,
                "templates": {"step_deg": step, "elevation_deg": el},
            }),
        )?;
        let v = json!({"ok": true, "dir": dir, "seconds": seconds, "preroll_s": r.preroll_s});
        *rec = Some(r);
        Ok(v)
    }

    /// Ends the running recording now (`{"stop": true}`).
    pub fn stop_recording(&self) -> Result<Value> {
        let dir = {
            let mut rec = self.recording.lk();
            let r = rec.as_mut().context("no recording is running")?;
            r.until = r.until.min(Instant::now());
            r.dir.clone()
        };
        self.finish_recording();
        Ok(json!({"ok": true, "dir": dir}))
    }

    fn finish_recording(&self) {
        let Some(rec) = self.recording.lk().take() else { return };
        let st = self.state.lk();
        let segments: Vec<Value> = st.segments.iter().filter(|s| s.t1 >= rec.started).map(|s| segment_json(s, rec.started, &st)).collect();
        drop(st);
        rec.finish(segments);
    }

    // -- status -------------------------------------------------------------------

    /// `GET /v1/speakers`: settings, the players placed, the recent
    /// segments (newest first) with their attribution and why.
    pub fn status(&self) -> Value {
        let now = Instant::now();
        let st = self.state.lk();
        let (head_yaw, head_pos) = head_at(&st.heads, now);
        let mix = match *self.target.lk() {
            MixTarget::Aligned(d) => format!("aligned ({d} samples, + left lags)"),
            MixTarget::Ear(true) => "left ear".into(),
            MixTarget::Ear(false) => "right ear".into(),
        };
        let players: Vec<Value> = st
            .players
            .iter()
            .map(|(name, s)| {
                let p = st.plates.get(name);
                json!({
                    "name": name,
                    "bearing_deg": r1(wrap_deg(yaw_to(head_pos, s.tag) - head_yaw)),
                    // The plate over the bot's eyes and how far (tracking metres):
                    // the voice anchor's calibration compares the voice with these.
                    "rise_m": r2(s.tag[1] - head_pos[1]),
                    "distance_m": r2((s.tag[0] - head_pos[0]).hypot(s.tag[2] - head_pos[2])),
                    "age_s": r1(now.saturating_duration_since(s.at).as_secs_f32()),
                    "stale": now.saturating_duration_since(s.at) > SEEN_FOR || st.walked - s.walked > STALE_WALK_M,
                    "placed_by": if s.bearing_only { "bearing" } else { "depth" },
                    "glow": p.map(|p| json!({"on": p.on, "over": r3(p.over), "quiet": {"eye": p.base[0].map(r3), "orbit": p.base[1].map(r3)}})),
                })
            })
            .collect();
        let mut segs: Vec<Value> = st.open.iter().chain(st.segments.iter().rev()).take(20).map(|s| segment_json(s, now, &st)).collect();
        for s in segs.iter_mut() {
            if let Some(t) = s["t_s"].as_f64() {
                s["ago_s"] = json!(r1(-t as f32));
            }
        }
        json!({
            "enabled": self.enabled,
            "hrtf": self.hrtf,
            "glow": {"on": self.glow.on, "off": self.glow.off, "release_s": self.glow.release_s},
            "latency_ms": self.latency.as_millis() as u64,
            "echo_tail_ms": self.echo_tail.as_millis() as u64,
            "mix": mix,
            "head_yaw": r1(head_yaw),
            "speaking": st.open.is_some(),
            "players": players,
            "segments": segs,
            "recording": self.recording.lk().as_ref().map(|r| r.dir.clone()),
        })
    }

    /// For tests: a player placed at `tag` now.
    #[cfg(test)]
    fn place(&self, name: &str, tag: [f32; 3]) {
        let mut st = self.state.lk();
        let walked = st.walked;
        st.players.insert(name.to_string(), Seen { tag, at: Instant::now(), walked, size: TEXT_SIZE, bearing_only: false });
        st.room.push((format!("usr_{name}"), name.to_string()));
    }
}

/// The HRTF to build templates from: `sphere`, a table's path, or `auto`
/// (the repo's Steam Audio table near the binary or here, else the sphere).
fn load_head(spec: &str) -> Box<dyn Hrtf> {
    const ASSET: &str = "assets/hrtf/steam-default-48k.bin";
    let paths: Vec<PathBuf> = match spec {
        "sphere" | "" => Vec::new(),
        "auto" => {
            let mut v = vec![PathBuf::from(ASSET)];
            if let Some(exe) = std::env::current_exe().ok().and_then(|e| e.parent().map(Path::to_path_buf)) {
                // <repo>/target/release/vrc-bridge
                v.insert(0, exe.join("../..").join(ASSET));
            }
            v
        }
        path => vec![crate::game::expand(path)],
    };
    for p in &paths {
        match HrirTable::read(p) {
            Ok(t) => return Box::new(t),
            Err(e) if spec != "auto" => tracing::warn!("speakers: {e:#}; using the spherical head"),
            Err(_) => {}
        }
    }
    Box::new(SphericalHead::default())
}

/// Turns to `yaw` (tracking space): the head alone when it stays within
/// HEAD_ONLY_DEG of where the body faces, else the whole bot; at an even,
/// unhurried pace.
fn turn_to(vr: &mut VrCore, yaw: f32) -> Result<()> {
    let d = wrap_deg(yaw - vr.yaw);
    let secs = (TURN_S_BASE + d.abs() / TURN_DEG_PER_S).min(1.6);
    if wrap_deg(yaw - vr.body_yaw()?).abs() <= HEAD_ONLY_DEG {
        vr.turn_head_gently(yaw, 0.0, secs)
    } else {
        vr.turn_gently(yaw, 0.0, secs, 0.0, 0.0)?;
        vr.face(yaw, 0.0)
    }
}

/// The candidates: players placed recently enough (and not before the bot
/// walked far), still in the room (when the room is known).
fn candidates(st: &State, now: Instant, head_pos: [f32; 3]) -> Vec<Cand> {
    st.players
        .iter()
        .filter(|(_, s)| now.saturating_duration_since(s.at) <= SEEN_FOR && st.walked - s.walked <= STALE_WALK_M)
        .filter(|(n, _)| st.room.is_empty() || st.room.iter().any(|(_, r)| r == *n))
        .map(|(n, s)| Cand {
            name: n.clone(),
            user_id: st.room.iter().find(|(_, r)| r == n).map(|(id, _)| id.clone()),
            world_yaw: yaw_to(head_pos, s.tag),
            age_s: now.saturating_duration_since(s.at).as_secs_f32(),
            glow: None,
        })
        .collect()
}

struct Fused {
    best: Option<usize>,
    posteriors: Vec<f32>,
    unknown: f32,
}

impl Fused {
    /// The event's `candidates`: the candidates and unknown (None),
    /// likeliest first, at most LISTED_MAX, each at least LISTED_P.
    fn ranked(&self, cands: &[Cand]) -> Vec<(Option<String>, f32)> {
        let mut all: Vec<(Option<String>, f32)> = cands.iter().zip(&self.posteriors).map(|(c, p)| (Some(c.name.clone()), *p)).collect();
        all.push((None, self.unknown));
        all.sort_by(|a, b| b.1.total_cmp(&a.1));
        all.retain(|(_, p)| *p >= LISTED_P);
        all.truncate(LISTED_MAX);
        all
    }
}

/// The share of `ring`'s votes near `yaw`, or (front and back folded, at
/// MIRROR_WEIGHT) near its mirror across the ears as the head looked at
/// `yaw0`.
fn doa_mass(ring: &Ring, yaw: f32, yaw0: f32) -> f32 {
    let mirror = yaw0 + mirror_deg(yaw - yaw0);
    ring.mass_near(yaw, NEAR_DEG).max(MIRROR_WEIGHT * ring.mass_near(mirror, NEAR_DEG))
}

/// What speaks for a candidate (the event's `cues.evidence`): its plate
/// lit, seen so by the bot's eyes (`glow`) or only by the user camera's
/// lens (`lens`); else the voice's direction alone (`direction`), or not
/// even that (`none`: too few votes, how recently they were placed).
fn evidence(c: &Cand, votes: bool) -> &'static str {
    match c.glow {
        Some(g) if g.lit() && g.eye => "glow",
        Some(g) if g.lit() => "lens",
        _ if votes => "direction",
        _ => "none",
    }
}

/// Weighs the candidates (and "unknown") for one segment: a prior from how
/// recently each was placed, times how much more of the votes lie near its
/// bearing (or its front/back mirror, `doa_mass`: the head looked at `yaw0`
/// as the segment began) than chance would put there, times its plate's
/// ring (came on as the segment began: strongly for; lit through it: for;
/// seen and dark all through, or lit only as an earlier speech's tail:
/// against; unseen: neither). Unknown takes chance's votes, and more prior
/// the more players in the room are not placed.
///
/// A candidate whose plate was not seen lit (only the direction speaks for
/// them) is at most `cap` likely, the rest going to unknown: a guess by
/// direction stays a guess (`[Bob 55% / someone 45%]`).
fn fuse(ring: Option<&Ring>, cands: &[Cand], unplaced: usize, yaw0: f32, cap: f32) -> Fused {
    let chance = (2.0 * NEAR_DEG / 360.0).min(1.0);
    let weights: Vec<f32> = cands
        .iter()
        .map(|c| {
            let prior = (-c.age_s / 30.0).exp().max(0.1);
            let doa = ring.map_or(1.0, |r| ((doa_mass(r, c.world_yaw, yaw0) + 0.02) / (chance + 0.02)).clamp(0.05, 20.0));
            let glow = match c.glow {
                Some(g) if g.onset => 9.0,
                Some(g) if g.share > 0.0 => 1.0 + 6.0 * g.share,
                Some(_) => 0.35,
                None => 1.0,
            };
            prior * doa * glow
        })
        .collect();
    let unknown = 0.25 + 0.15 * unplaced as f32;
    let total = weights.iter().sum::<f32>() + unknown;
    let mut unknown = unknown / total;
    let posteriors: Vec<f32> = weights
        .iter()
        .zip(cands)
        .map(|(w, c)| {
            let p = w / total;
            if p <= cap || c.glow.is_some_and(|g| g.lit()) {
                p
            } else {
                unknown += p - cap;
                cap
            }
        })
        .collect();
    let best = (0..cands.len()).max_by(|&a, &b| posteriors[a].total_cmp(&posteriors[b])).filter(|&i| posteriors[i] > unknown && posteriors[i] >= 0.4);
    Fused { best, posteriors, unknown }
}

impl Attribution {
    fn unknown(cues: Value, confidence: f32) -> Attribution {
        Attribution { name: None, user_id: None, world_yaw: None, bearing: None, confidence, cues, candidates: Value::Null, ranked: vec![(None, 1.0)] }
    }
}

/// The contract's `speaker` event.
fn event(seg: &Seg, who: &Attribution, last: bool) -> Value {
    json!({
        "type": "speaker",
        "start": seg.start,
        "end": seg.end,
        "final": last,
        "name": who.name,
        "user_id": who.user_id,
        "bearing_deg": who.bearing.map(r1),
        "confidence": r2(who.confidence),
        "cues": who.cues,
        "candidates": who.ranked.iter().map(|(name, p)| json!({"name": name, "p": r2(*p)})).collect::<Vec<_>>(),
    })
}

fn segment_json(s: &Seg, t0: Instant, st: &State) -> Value {
    let (head_yaw, _) = head_at(&st.heads, Instant::now());
    let peaks: Vec<Value> =
        if s.bins >= MIN_BINS { s.world.peaks(NEAR_DEG, 20.0, 3).iter().map(|p| json!([r1(wrap_deg(p.0 - head_yaw)), r2(p.1)])).collect() } else { Vec::new() };
    let who = s.who.as_ref();
    json!({
        "id": s.id,
        "start": s.start,
        "end": s.end,
        "final": s.done,
        "t_s": r2(signed_secs(s.t0, t0)),
        "duration_s": r2(s.t1.duration_since(s.t0).as_secs_f32()),
        "bot_echo": s.bot_echo,
        "voiced_hops": s.voiced,
        "echo_hops": s.echoed,
        "name": who.and_then(|w| w.name.clone()),
        "user_id": who.and_then(|w| w.user_id.clone()),
        "bearing_deg": who.and_then(|w| w.bearing).map(r1),
        "world_yaw": who.and_then(|w| w.world_yaw).map(r1),
        "confidence": who.map(|w| r2(w.confidence)),
        "cues": who.map(|w| w.cues.clone()),
        "doa_peaks_deg_share": peaks,
        "candidates": who.map(|w| w.candidates.clone()),
    })
}

impl Plate {
    /// One look from `from` at `now`: the ring is on or off at once (the
    /// ring itself is on/off, lit from its first frame), with hysteresis
    /// between `on` and `off` over the quiet look of that view; the quiet
    /// look follows (down at once, up slowly) while the plate is unlit and
    /// the bot's own voice is not playing back. Looks may come late (the
    /// orbit's OCR): kept in time order.
    fn update(&mut self, now: Instant, score: f32, from: LookFrom, g: &GlowParams, echo: bool) {
        let k = from as usize;
        let last = self.history.iter().rev().find(|h| h.t <= now).map(|h| h.t);
        let dt = last.map_or(0.0, |l| now.saturating_duration_since(l).as_secs_f32());
        let over = score - self.base[k].unwrap_or(QUIET_PRIOR);
        self.over = over;
        self.on = if self.on { over > g.off } else { over > g.on };
        if !self.on {
            self.on_since = None;
        } else if self.on_since.is_none() {
            self.on_since = Some(now);
        }
        // Lit for longer than anyone speaks: the view changed (a saturated
        // background); the quiet look learns it.
        let stuck = self.on_since.is_some_and(|s| now.saturating_duration_since(s) > STUCK_LIT);
        if !echo && (!self.on || stuck) {
            self.base[k] = Some(match self.base[k] {
                None => score.min(QUIET_PRIOR),
                Some(b) if score < b => b + (score - b) * 0.5,
                Some(b) => b + (score - b) * (dt / BASELINE_RISE_S).min(1.0),
            });
        }
        let look = Look { t: now, on: self.on, echo, from };
        let at = self.history.iter().rposition(|h| h.t <= now).map_or(0, |i| i + 1);
        self.history.insert(at, look);
        let newest = self.history.back().map_or(now, |h| h.t);
        while self.history.front().is_some_and(|h| newest.saturating_duration_since(h.t) > Duration::from_secs(60)) {
            self.history.pop_front();
        }
    }

    /// The ring for a segment heard from `t0` to `t1` (None: no looks then),
    /// leaving out looks while the bot's voice may have been playing back
    /// (the player's plate lights with it):
    /// - `onset`: a lit spell began GLOW_LEAD before `t0`, within
    ///   GLOW_ALIGN and half the gap between the looks either side of it;
    /// - `share`: of the looks from GLOW_LEAD before `t0` to `t1`, those
    ///   lit and not in the last `release_s` of a spell that ended (that
    ///   tail is after the speech: someone who stopped as this began).
    fn cue(&self, t0: Instant, t1: Instant, g: &GlowParams) -> Option<GlowCue> {
        let looks: Vec<&Look> = self.history.iter().filter(|h| !h.echo).collect();
        let expected = t0.checked_sub(GLOW_LEAD).unwrap_or(t0);
        let tail = Duration::from_secs_f32(g.release_s.max(0.0));
        // Spells of lit looks: (index of the first, of the last).
        let mut spells: Vec<(usize, usize)> = Vec::new();
        for (i, h) in looks.iter().enumerate() {
            if h.on {
                match spells.last_mut() {
                    Some(s) if s.1 + 1 == i => s.1 = i,
                    _ => spells.push((i, i)),
                }
            }
        }
        let mut onset = false;
        let mut eye = false;
        let mut voiced = vec![false; looks.len()];
        for &(a, b) in &spells {
            // The onset: between the unlit look before and the first lit one.
            if a > 0 {
                let (before, first) = (looks[a - 1].t, looks[a].t);
                let gap = first.saturating_duration_since(before);
                if gap <= ONSET_GAP {
                    let at = before + gap / 2;
                    let slack = GLOW_ALIGN + (gap / 2).min(GLOW_ALIGN);
                    let off = if at > expected { at - expected } else { expected - at };
                    if off <= slack {
                        onset = true;
                        eye |= looks[a].from == LookFrom::Eye;
                    }
                }
            }
            // The tail: lit looks within `release_s` of the spell's end (at
            // the latest the unlit look after it).
            let end = looks.get(b + 1).map(|after| after.t);
            for (i, v) in voiced.iter_mut().enumerate().take(b + 1).skip(a) {
                *v = end.is_none_or(|e| looks[i].t + tail <= e);
            }
        }
        let during: Vec<usize> = (0..looks.len()).filter(|&i| looks[i].t >= expected && looks[i].t <= t1).collect();
        if during.is_empty() && !onset {
            return None;
        }
        let lit: Vec<usize> = during.iter().copied().filter(|&i| voiced[i]).collect();
        eye |= lit.iter().any(|&i| looks[i].from == LookFrom::Eye);
        let share = if during.is_empty() { 0.0 } else { lit.len() as f32 / during.len() as f32 };
        Some(GlowCue { onset, share, eye })
    }
}

/// The head's yaw and position at `t` (the nearest kept; none kept:
/// straight ahead at the origin).
fn head_at(heads: &VecDeque<(Instant, f32, [f32; 3])>, t: Instant) -> (f32, [f32; 3]) {
    heads.iter().min_by_key(|h| if h.0 > t { h.0 - t } else { t - h.0 }).map(|h| (h.1, h.2)).unwrap_or((0.0, [0.0; 3]))
}

/// The yaw (tracking space, degrees, + right of -z) from `from` to `to`.
fn yaw_to(from: [f32; 3], to: [f32; 3]) -> f32 {
    (to[0] - from[0]).atan2(-(to[2] - from[2])).to_degrees()
}

/// The text box of a plate whose tag is at `tag`, `size` tracking metres,
/// in `frame`'s left eye; None when behind or out of the view.
fn project(frame: &EyeFrame, tag: [f32; 3], size: [f32; 2]) -> Option<[f32; 4]> {
    let view = frame.views[0];
    let d = [tag[0] - view.pose.position[0], tag[1] - view.pose.position[1], tag[2] - view.pose.position[2]];
    let v = view.pose.unrotate(d);
    if v[2] > -0.2 {
        return None;
    }
    let [fx, fy, cx, cy] = view.fov.intrinsics(frame.width, frame.height);
    let z = -v[2];
    let (u, w) = (cx + fx * v[0] / z, fx * size[0] / z);
    let (y, h) = (cy - fy * v[1] / z, fy * size[1] / z);
    let b = [u - w / 2.0, y - h / 2.0, w, h];
    let inside = b[0] - 3.0 * h >= 0.0 && b[1] - h >= 0.0 && b[0] + w + h < frame.width as f32 && b[1] + 2.0 * h < frame.height as f32;
    (inside && h >= 4.0).then_some(b)
}

/// The size (tracking metres) of the text box `bbox` whose middle is at
/// `tag`.
fn text_size(frame: &EyeFrame, tag: [f32; 3], bbox: [f32; 4]) -> Option<[f32; 2]> {
    let view = frame.views[0];
    let d = [tag[0] - view.pose.position[0], tag[1] - view.pose.position[1], tag[2] - view.pose.position[2]];
    let z = -view.pose.unrotate(d)[2];
    let [fx, fy, _, _] = view.fov.intrinsics(frame.width, frame.height);
    (z > 0.2).then(|| [bbox[2] * z / fx, bbox[3] * z / fy])
}

/// The nameplate's pill (the 2026 plate, measured 2026-10-08 at 2-3 m) in
/// heights `h` of the name's text box (x, y, w, h): from `x - 3.0 h` (the
/// avatar icon is inside, on the left) to `x + w + 0.85 h`, `3.0 h` high
/// round the text's middle, the ends round. The edge is looked for along
/// each sample's normal from PILL_INSIDE h inside to PILL_OUTSIDE h outside
/// this model (OCR boxes are a little loose or tight).
const PILL_LEFT: f32 = 3.0;
const PILL_RIGHT: f32 = 0.85;
const PILL_HALF: f32 = 1.5;
const PILL_INSIDE: f32 = 0.5;
const PILL_OUTSIDE: f32 = 0.5;
/// Samples round the outline.
const PILL_SAMPLES: usize = 48;
/// The ring's band just outside the pill's dark fill: about 0.12 h, 2-6 px.
const RING_BAND: f32 = 0.12;
/// A ring pixel: saturated (chroma, max - min) and bright (max), 0..1;
/// "the same hue" within this (degrees).
const RING_CHROMA: f32 = 0.30;
const RING_VALUE: f32 = 0.45;
const RING_HUE_DEG: f32 = 30.0;
/// The ripple arcs: beside the pill's ends, from the band out to this many
/// text heights.
const RIPPLE_OUT: f32 = 0.8;

/// Pixels a plate is measured in: a frame's left eye, or an RGB image (the
/// user camera's view).
#[derive(Clone, Copy)]
pub struct View<'a> {
    px: &'a [u8],
    pub width: usize,
    pub height: usize,
    bpp: usize,
    bgr: bool,
}

impl<'a> View<'a> {
    /// The left eye of `frame` (None: a frame without its pixels).
    pub fn eye(frame: &'a EyeFrame) -> Option<View<'a>> {
        (frame.pixels.len() >= frame.eye_bytes()).then(|| View {
            px: frame.eye(0),
            width: frame.width as usize,
            height: frame.height as usize,
            bpp: frame.bytes_per_pixel as usize,
            bgr: matches!(frame.format, format::B8G8R8A8_UNORM | format::B8G8R8A8_SRGB),
        })
    }

    /// RGB8, `width` x `height`.
    pub fn rgb(px: &'a [u8], width: usize, height: usize) -> View<'a> {
        assert!(px.len() >= width * height * 3, "an RGB image smaller than its size");
        View { px, width, height, bpp: 3, bgr: false }
    }

    /// The pixel at (x, y) as RGB 0..1; None outside.
    fn at(&self, x: i64, y: i64) -> Option<[f32; 3]> {
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
            return None;
        }
        let i = (y as usize * self.width + x as usize) * self.bpp;
        let p = &self.px[i..i + 3];
        let (r, g, b) = if self.bgr { (p[2], p[1], p[0]) } else { (p[0], p[1], p[2]) };
        Some([r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0])
    }

    fn rgb8(&self, x: usize, y: usize) -> [u8; 3] {
        let i = (y * self.width + x) * self.bpp;
        let p = &self.px[i..i + 3];
        if self.bgr {
            [p[2], p[1], p[0]]
        } else {
            [p[0], p[1], p[2]]
        }
    }
}

/// Points round the pill's model (`PILL_*`) for the text box `bbox`, with
/// their outward normals.
fn pill_outline(bbox: [f32; 4]) -> Vec<([f32; 2], [f32; 2])> {
    let [x, y, w, h] = bbox;
    let (cy, r) = (y + h / 2.0, PILL_HALF * h);
    let (cl, cr) = (x - PILL_LEFT * h + r, (x + w + PILL_RIGHT * h - r).max(x - PILL_LEFT * h + r));
    let straight = cr - cl;
    let arc = std::f32::consts::PI * r;
    let total = 2.0 * straight + 2.0 * arc;
    (0..PILL_SAMPLES)
        .map(|i| {
            let s = (i as f32 + 0.5) / PILL_SAMPLES as f32 * total;
            if s < straight {
                ([cl + s, cy - r], [0.0, -1.0])
            } else if s < straight + arc {
                let a = -std::f32::consts::FRAC_PI_2 + (s - straight) / r;
                ([cr + r * a.cos(), cy + r * a.sin()], [a.cos(), a.sin()])
            } else if s < 2.0 * straight + arc {
                ([cr - (s - straight - arc), cy + r], [0.0, 1.0])
            } else {
                let a = std::f32::consts::FRAC_PI_2 + (s - 2.0 * straight - arc) / r;
                ([cl + r * a.cos(), cy + r * a.sin()], [a.cos(), a.sin()])
            }
        })
        .collect()
}

fn chroma_value(p: [f32; 3]) -> (f32, f32) {
    let max = p[0].max(p[1]).max(p[2]);
    (max - p[0].min(p[1]).min(p[2]), max)
}

fn hue_apart(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(360.0);
    d.min(360.0 - d)
}

/// The ring round a nameplate whose name's text box is `bbox` in `view`
/// (`speaker.md` 2.3): VRChat draws a thin outline hugging the dark pill
/// (RGB about 30, 30, 35) while the player speaks, in the plate's accent
/// colour (gold for a friend, cyan for others), and faint ripple arcs
/// beside it. Colour-agnostic: for each sample round the pill's model the
/// pill's edge is found (the first pixel past its dark fill, walking out),
/// and the band just outside it gives its most saturated pixel; the ring
/// is the share of samples whose pixel is strongly saturated and bright,
/// of their most common hue. None when the box is too small, the text box
/// shows no dark fill, or most of the outline is out of the view.
pub fn glow_stats(view: &View, bbox: [f32; 4]) -> Option<GlowStats> {
    let [x, y, w, h] = bbox;
    if h < 3.0 || w <= 0.0 || !(x.is_finite() && y.is_finite()) {
        return None;
    }
    // The fill: the dark pixels in the text box (between the glyphs).
    let mut dark: Vec<[f32; 3]> = Vec::new();
    let step = ((h / 10.0) as i64).max(1);
    let (mut yy, y1) = (y.floor() as i64, (y + h).ceil() as i64);
    while yy < y1 {
        let mut xx = x.floor() as i64;
        while xx < (x + w).ceil() as i64 {
            if let Some(p) = view.at(xx, yy) {
                if chroma_value(p).1 < 0.3 {
                    dark.push(p);
                }
            }
            xx += step;
        }
        yy += step;
    }
    if dark.len() < 8 {
        return None;
    }
    let median = |k: usize| {
        let mut v: Vec<f32> = dark.iter().map(|p| p[k]).collect();
        v.sort_by(f32::total_cmp);
        v[v.len() / 2]
    };
    let fill = [median(0), median(1), median(2)];
    let off_fill = |p: [f32; 3]| (p[0] - fill[0]).abs().max((p[1] - fill[1]).abs()).max((p[2] - fill[2]).abs());
    let mut spread: Vec<f32> = dark.iter().map(|p| off_fill(*p)).collect();
    spread.sort_by(f32::total_cmp);
    let tol = (3.0 * spread[spread.len() / 2] + 4.0 / 255.0).clamp(4.0 / 255.0, 12.0 / 255.0);
    let is_fill = |p: [f32; 3]| off_fill(p) <= tol;
    let band = (RING_BAND * h).round().clamp(2.0, 6.0) as i64;
    let (inside, outside) = ((PILL_INSIDE * h).round() as i64, (PILL_OUTSIDE * h).round() as i64);
    let ripple_out = (RIPPLE_OUT * h).round() as i64;
    let px = |p: [f32; 2], n: [f32; 2], k: i64| view.at((p[0] + n[0] * k as f32).round() as i64, (p[1] + n[1] * k as f32).round() as i64);
    // Per sample: (chroma, value, hue) of the band's most saturated pixel,
    // and where the band ended (for the ripple), when the edge was found.
    type Sample = ((f32, f32, f32), i64);
    let mut samples: Vec<Option<Sample>> = Vec::new();
    let outline = pill_outline(bbox);
    for &(p, n) in &outline {
        if px(p, n, 0).is_none() {
            continue; // out of the view: not a sample
        }
        let mut run = 0;
        let mut edge = None;
        for k in -inside..=outside {
            let Some(c) = px(p, n, k) else { break };
            if is_fill(c) {
                run += 1;
            } else if run >= 2 {
                edge = Some(k);
                break;
            } else {
                run = 0;
            }
        }
        let Some(edge) = edge else {
            samples.push(None);
            continue;
        };
        let mut best: Option<(f32, f32, f32)> = None;
        for k in edge..edge + band {
            if let Some(c) = px(p, n, k) {
                let (chroma, value) = chroma_value(c);
                if best.is_none_or(|b| chroma > b.0) {
                    best = Some((chroma, value, hue(c[0], c[1], c[2])));
                }
            }
        }
        samples.push(best.map(|b| (b, edge + band)));
    }
    if samples.len() < PILL_SAMPLES / 2 {
        return None;
    }
    let n = samples.len() as f32;
    let strong: Vec<f32> = samples.iter().flatten().filter(|(b, _)| b.0 >= RING_CHROMA && b.1 >= RING_VALUE).map(|(b, _)| b.2).collect();
    // The most common hue of the strong ones, and how many share it.
    let (hue_of, count) = strong
        .iter()
        .map(|&h| (h, strong.iter().filter(|&&o| hue_apart(o, h) <= RING_HUE_DEG).count()))
        .max_by_key(|&(_, c)| c)
        .map_or((None, 0), |(h, c)| (Some(h), c));
    let mut ripple = (0f32, 0f32);
    if let Some(ring_hue) = hue_of {
        for (&(p, nrm), s) in outline.iter().zip(samples.iter()) {
            let Some((_, from)) = s else { continue };
            if nrm[0].abs() < 0.7 {
                continue; // the ends only
            }
            for k in from + 1..=from + ripple_out {
                if let Some(c) = px(p, nrm, k) {
                    let (chroma, value) = chroma_value(c);
                    ripple.1 += 1.0;
                    if chroma >= RING_CHROMA && value >= RING_VALUE && hue_apart(hue(c[0], c[1], c[2]), ring_hue) <= RING_HUE_DEG {
                        ripple.0 += 1.0;
                    }
                }
            }
        }
    }
    let sum = |f: &dyn Fn(&(f32, f32, f32)) -> f32| samples.iter().map(|s| s.as_ref().map_or(0.0, |(b, _)| f(b))).sum::<f32>() / n;
    Some(GlowStats {
        ring: count as f32 / n,
        chroma: sum(&|b| b.0),
        value: sum(&|b| b.1),
        ripple: if ripple.1 > 0.0 { ripple.0 / ripple.1 } else { 0.0 },
        hue: hue_of,
        edges: samples.iter().filter(|s| s.is_some()).count() as f32 / n,
    })
}

/// Hue in degrees.
fn hue(r: f32, g: f32, b: f32) -> f32 {
    let max = r.max(g).max(b);
    let d = max - r.min(g).min(b);
    if d <= 0.0 {
        return 0.0;
    }
    let h = if max == r {
        ((g - b) / d).rem_euclid(6.0)
    } else if max == g {
        (b - r) / d + 2.0
    } else {
        (r - g) / d + 4.0
    };
    60.0 * h
}

// -- recordings ---------------------------------------------------------------------

/// A calibration recording (`POST /v1/speakers/record`), in its own
/// directory: `audio.wav` (both ears, float, with what the ring held
/// before as a pre-roll), `hops.jsonl` (every 10 ms: the detector, the
/// head, the votes' peaks), `votes.f32` (every hop's head-relative votes,
/// the templates' azimuths), `glow.jsonl` and `crops/` (every plate
/// measured), `segments.json`, `meta.json`.
///
/// The callers (the analysis, the watcher, under the state's lock) only
/// hand over what to write; the recording's own thread encodes the crops
/// and writes the files.
struct Recording {
    dir: PathBuf,
    started: Instant,
    until: Instant,
    preroll_s: f32,
    votes_len: usize,
    meta: Value,
    crops: usize,
    tx: mpsc::Sender<RecWrite>,
}

/// What the recording's thread writes.
enum RecWrite {
    /// Both ears, interleaved.
    Audio(Vec<f32>),
    /// A line of `hops.jsonl` and the hop's row of `votes.f32`.
    Hop(String, Vec<f32>),
    /// A line of `glow.jsonl`, and a crop to encode into a file.
    Glow(String, Option<(PathBuf, Crop)>),
    Finish { segments: Vec<Value>, meta: Value },
}

/// A plate's crop, RGB8.
struct Crop {
    rgb: Vec<u8>,
    width: u16,
    height: u16,
}

/// The recording's files, on its thread.
struct RecFiles {
    dir: PathBuf,
    wav: BufWriter<File>,
    samples: u64,
    hops: BufWriter<File>,
    votes: BufWriter<File>,
    glow: BufWriter<File>,
}

const MAX_CROPS: usize = 20_000;

impl Recording {
    fn start(dir: &Path, seconds: f32, preroll: Vec<f32>, votes_len: usize, meta: Value) -> Result<Recording> {
        std::fs::create_dir_all(dir.join("crops")).with_context(|| format!("creating {}", dir.display()))?;
        let mut files = RecFiles {
            dir: dir.to_path_buf(),
            wav: BufWriter::new(File::create(dir.join("audio.wav"))?),
            samples: 0,
            hops: BufWriter::new(File::create(dir.join("hops.jsonl"))?),
            votes: BufWriter::new(File::create(dir.join("votes.f32"))?),
            glow: BufWriter::new(File::create(dir.join("glow.jsonl"))?),
        };
        wav_header(&mut files.wav, 0)?;
        let preroll_s = preroll.len() as f32 / 2.0 / vrc_audio::RATE as f32;
        let (tx, rx) = mpsc::channel();
        let _ = tx.send(RecWrite::Audio(preroll));
        std::thread::Builder::new().name("speakers-rec".into()).spawn(move || files.run(rx))?;
        let now = Instant::now();
        Ok(Recording { dir: dir.to_path_buf(), started: now, until: now + Duration::from_secs_f32(seconds), preroll_s, votes_len, meta, crops: 0, tx })
    }

    fn audio(&mut self, left: &[f32], right: &[f32]) {
        let both = left.iter().zip(right).flat_map(|(l, r)| [*l, *r]).collect();
        let _ = self.tx.send(RecWrite::Audio(both));
    }

    fn hop(&mut self, h: &vrc_audio::HopOut, t: Instant, yaw: f32, echo: bool, segment: Option<u64>) {
        let peaks = h.doa.as_ref().map(|d| {
            let mut r = Ring::zeros(d.votes.len());
            r.add_turned(&d.votes, 0.0, 1.0);
            r.peaks(NEAR_DEG, 20.0, 2).iter().map(|p| [r1(p.0), r2(p.1)]).collect::<Vec<_>>()
        });
        let line = json!({
            "t": r3(signed_secs(t, self.started)),
            "clock": [h.start, h.end],
            "db": r1(h.level.db), "floor_db": r1(h.level.floor_db), "speech_share": r2(h.level.speech_share),
            "voiced": h.level.voiced, "active": h.level.active,
            "bot_echo": echo,
            "head_yaw": r1(yaw),
            "lag": h.lag.map(|l| [r2(l.0), r2(l.1)]),
            "bins": h.doa.as_ref().map_or(0, |d| d.bins),
            "peaks": peaks,
            "segment": segment,
        });
        // A row of the templates' directions a hop (zeros when not voiced).
        let votes = h.doa.as_ref().map_or_else(|| vec![0f32; self.votes_len], |d| d.votes.clone());
        let _ = self.tx.send(RecWrite::Hop(line.to_string(), votes));
    }

    #[allow(clippy::too_many_arguments)]
    fn glow(
        &mut self,
        t: Instant,
        name: &str,
        how: &str,
        g: &GlowStats,
        base: Option<f32>,
        over: f32,
        on: bool,
        echo: bool,
        crop: Option<(View, [f32; 4])>,
    ) {
        let ms = (signed_secs(t, self.started) * 1000.0).round() as i64;
        let cut = crop.filter(|_| self.crops < MAX_CROPS).and_then(|(f, b)| {
            let safe: String = name.chars().map(|c| if c.is_alphanumeric() { c } else { '_' }).collect();
            let file = format!("crops/{ms}_{safe}.jpg");
            crop_rgb(&f, b).ok().map(|c| (file, c))
        });
        if cut.is_some() {
            self.crops += 1;
        }
        let line = json!({
            "t": r3(ms as f32 / 1000.0), "name": name, "how": how, "stats": g.json(),
            "quiet": base.map(r3), "over": r3(over), "on": on, "bot_echo": echo,
            "bbox": crop.map(|c| c.1.map(|v| v.round())), "crop": cut.as_ref().map(|c| c.0.clone()),
        });
        let _ = self.tx.send(RecWrite::Glow(line.to_string(), cut.map(|(file, c)| (self.dir.join(file), c))));
    }

    /// Hands the end to the recording's thread (it writes the rest and
    /// says when it is done).
    fn finish(self, segments: Vec<Value>) {
        let mut meta = self.meta;
        meta["preroll_s"] = json!(r3(self.preroll_s));
        meta["seconds"] = json!(r2(self.until.saturating_duration_since(self.started).as_secs_f32()));
        meta["crops"] = json!(self.crops);
        meta["rate"] = json!(vrc_audio::RATE);
        meta["hop_s"] = json!(vrc_audio::HOP as f32 / vrc_audio::RATE as f32);
        meta["votes_per_hop"] = json!(self.votes_len);
        let _ = self.tx.send(RecWrite::Finish { segments, meta });
    }
}

impl RecFiles {
    /// The recording's thread: writes what comes until the end (or until
    /// the recording is dropped unfinished).
    fn run(mut self, rx: mpsc::Receiver<RecWrite>) {
        for w in rx {
            match w {
                RecWrite::Audio(both) => {
                    for x in &both {
                        let _ = self.wav.write_all(&x.to_le_bytes());
                    }
                    self.samples += both.len() as u64 / 2;
                }
                RecWrite::Hop(line, votes) => {
                    let _ = writeln!(self.hops, "{line}");
                    for x in &votes {
                        let _ = self.votes.write_all(&x.to_le_bytes());
                    }
                }
                RecWrite::Glow(line, crop) => {
                    if let Some((path, c)) = crop {
                        if let Err(e) = write_jpeg(&path, &c) {
                            tracing::debug!("speakers: a crop not written: {e:#}");
                        }
                    }
                    let _ = writeln!(self.glow, "{line}");
                }
                RecWrite::Finish { segments, meta } => {
                    match self.finish(&segments, &meta) {
                        Ok(()) => tracing::info!("speakers: recording done in {}", self.dir.display()),
                        Err(e) => tracing::warn!("speakers: recording failed: {e:#}"),
                    }
                    return;
                }
            }
        }
    }

    fn finish(&mut self, segments: &[Value], meta: &Value) -> Result<()> {
        self.wav.flush()?;
        let f = self.wav.get_mut();
        f.seek(SeekFrom::Start(0))?;
        wav_header(f, self.samples)?;
        self.hops.flush()?;
        self.votes.flush()?;
        self.glow.flush()?;
        std::fs::write(self.dir.join("segments.json"), serde_json::to_vec_pretty(segments)?)?;
        std::fs::write(self.dir.join("meta.json"), serde_json::to_vec_pretty(meta)?)?;
        Ok(())
    }
}

// A float32 stereo WAV header for `frames` frames.
fn wav_header(w: &mut impl Write, frames: u64) -> Result<()> {
    let data = (frames * 8).min(u32::MAX as u64 - 64) as u32;
    let rate = vrc_audio::RATE;
    w.write_all(b"RIFF")?;
    w.write_all(&(36 + data).to_le_bytes())?;
    w.write_all(b"WAVEfmt ")?;
    w.write_all(&16u32.to_le_bytes())?;
    w.write_all(&3u16.to_le_bytes())?; // IEEE float
    w.write_all(&2u16.to_le_bytes())?;
    w.write_all(&rate.to_le_bytes())?;
    w.write_all(&(rate * 8).to_le_bytes())?;
    w.write_all(&8u16.to_le_bytes())?;
    w.write_all(&32u16.to_le_bytes())?;
    w.write_all(b"data")?;
    w.write_all(&data.to_le_bytes())?;
    Ok(())
}

/// The plate and round it (the pill and its ripple arcs, with a margin),
/// copied out of the view.
fn crop_rgb(view: &View, bbox: [f32; 4]) -> Result<Crop> {
    let [x, y, w, h] = bbox;
    let (cy, half) = (y + h / 2.0, (PILL_HALF + 1.0) * h);
    let x0 = (x - (PILL_LEFT + 1.5) * h).max(0.0) as usize;
    let y0 = (cy - half).max(0.0) as usize;
    let x1 = ((x + w + (PILL_RIGHT + 1.5) * h).ceil().max(0.0) as usize).min(view.width);
    let y1 = ((cy + half).ceil().max(0.0) as usize).min(view.height);
    anyhow::ensure!(x1 > x0 + 2 && y1 > y0 + 2, "an empty crop");
    let mut rgb = Vec::with_capacity((x1 - x0) * (y1 - y0) * 3);
    for yy in y0..y1 {
        for xx in x0..x1 {
            rgb.extend(view.rgb8(xx, yy));
        }
    }
    Ok(Crop { rgb, width: (x1 - x0) as u16, height: (y1 - y0) as u16 })
}

/// A crop as JPEG.
fn write_jpeg(path: &Path, c: &Crop) -> Result<()> {
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 90).encode(&c.rgb, c.width, c.height, jpeg_encoder::ColorType::Rgb)?;
    std::fs::write(path, jpeg)?;
    Ok(())
}

// Seconds from `from` to `t` (negative when `t` is earlier).
fn signed_secs(t: Instant, from: Instant) -> f32 {
    if t >= from {
        t.duration_since(from).as_secs_f32()
    } else {
        -from.duration_since(t).as_secs_f32()
    }
}

fn r1(v: f32) -> f64 {
    (v as f64 * 10.0).round() / 10.0
}

fn r2(v: f32) -> f64 {
    (v as f64 * 100.0).round() / 100.0
}

fn r3(v: f32) -> f64 {
    (v as f64 * 1000.0).round() / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;
    use vrc_audio::hrtf::render;

    fn args(extra: &[&str]) -> Args {
        let dir = std::env::temp_dir().join(format!("vrc-speaker-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let token = dir.join("token");
        let mut a = vec!["vrc-bridge", "--token-file", token.to_str().unwrap(), "--hrtf-table", "sphere"];
        a.extend_from_slice(extra);
        Args::parse_from(a)
    }

    /// A ring with all its votes at `deg`.
    fn ring_at(deg: f32) -> Ring {
        let mut r = Ring::zeros(180);
        let mut v = vec![0.0; 180];
        v[((deg + 180.0) / 2.0).round() as usize % 180] = 100.0;
        r.add_turned(&v, 0.0, 1.0);
        r
    }

    fn cand(name: &str, yaw: f32, glow: Option<f32>) -> Cand {
        Cand { name: name.into(), user_id: None, world_yaw: yaw, age_s: 1.0, glow: glow.map(|share| GlowCue { onset: false, share, eye: true }) }
    }

    /// No cap: what the cues alone say.
    const NO_CAP: f32 = 1.0;
    const CAP: f32 = 0.55;

    #[test]
    fn the_voice_picks_the_player_it_comes_from() {
        let ring = ring_at(40.0);
        let ahead = fuse(Some(&ring), &[cand("a", 41.0, None), cand("b", -60.0, None)], 0, 0.0, NO_CAP);
        assert_eq!(ahead.best, Some(0));
        assert!(ahead.posteriors[0] > 0.8, "{:?}", ahead.posteriors);
        // Nobody where the voice is: unknown.
        let f = fuse(Some(&ring), &[cand("b", -60.0, None)], 1, 0.0, NO_CAP);
        assert_eq!(f.best, None);
        assert!(f.unknown > 0.5);
        // Someone right behind, where a voice ahead often peaks too: theirs,
        // a little less surely than straight from them.
        let behind = fuse(Some(&ring), &[cand("a", 140.0, None), cand("b", -60.0, None)], 0, 0.0, NO_CAP);
        assert_eq!(behind.best, Some(0));
        assert!(behind.posteriors[0] > 0.6 && behind.posteriors[0] < ahead.posteriors[0], "{:?}", behind.posteriors);
    }

    #[test]
    fn the_direction_alone_is_a_capped_guess() {
        // Bob placed where the voice comes from (the sweep's cache), his
        // plate unseen or seen dark: named, but no more than the cap, the
        // rest someone not recognised.
        let ring = ring_at(40.0);
        for glow in [None, Some(0.0)] {
            let cands = [cand("Bob", 41.0, glow), cand("Ann", -60.0, None)];
            let f = fuse(Some(&ring), &cands, 0, 0.0, CAP);
            assert_eq!(f.best, Some(0), "{glow:?}");
            assert!((f.posteriors[0] - CAP).abs() < 1e-6, "{:?}", f.posteriors);
            assert!(f.unknown > 0.35, "{}", f.unknown);
            assert_eq!(evidence(&cands[0], true), "direction");
            let ranked = f.ranked(&cands);
            assert_eq!(ranked[0], (Some("Bob".into()), CAP));
            assert_eq!(ranked[1].0, None);
            assert!(ranked.iter().all(|(_, p)| *p >= LISTED_P) && ranked.len() <= LISTED_MAX);
        }
        // Front and back folded: someone placed right behind gets the
        // voice from ahead, still capped.
        let f = fuse(Some(&ring), &[cand("Bob", 140.0, None)], 0, 0.0, CAP);
        assert_eq!(f.best, Some(0));
        assert!((f.posteriors[0] - CAP).abs() < 1e-6);
        // No votes either (too short): how recently placed alone, capped.
        let f = fuse(None, &[cand("Bob", 41.0, None)], 0, 0.0, CAP);
        assert!(f.posteriors[0] <= CAP + 1e-6);
        assert_eq!(evidence(&cand("Bob", 41.0, None), false), "none");
    }

    #[test]
    fn the_direction_and_a_lit_plate_are_sure() {
        let ring = ring_at(40.0);
        let cands = [cand("Bob", 41.0, Some(0.9)), cand("Ann", -60.0, Some(0.0))];
        let f = fuse(Some(&ring), &cands, 0, 0.0, CAP);
        assert_eq!(f.best, Some(0));
        assert!(f.posteriors[0] > 0.9, "{:?}", f.posteriors);
        assert_eq!(evidence(&cands[0], true), "glow");
        let ranked = f.ranked(&cands);
        assert_eq!(ranked.len(), 1, "{ranked:?}");
        // Lit as only the user camera's lens saw it.
        let lens = Cand { glow: Some(GlowCue { onset: true, share: 0.5, eye: false }), ..cand("Bob", 41.0, None) };
        assert_eq!(evidence(&lens, true), "lens");
        let f = fuse(Some(&ring), &[lens], 0, 0.0, CAP);
        assert!(f.posteriors[0] > 0.9, "{:?}", f.posteriors);
    }

    #[test]
    fn the_glow_settles_a_voice_between_two_players() {
        // Votes split between the two (a front/back mirror, say).
        let mut ring = ring_at(30.0);
        ring.add_turned(&ring_at(150.0).p, 0.0, 1.0);
        let both = [cand("a", 30.0, Some(0.0)), cand("b", 150.0, Some(0.8))];
        assert_eq!(fuse(Some(&ring), &both, 0, 0.0, CAP).best, Some(1));
        // No votes at all (too short), one plate lit: that one.
        let f = fuse(None, &[cand("a", 30.0, Some(0.0)), cand("b", 150.0, Some(1.0))], 0, 0.0, CAP);
        assert_eq!(f.best, Some(1));
    }

    /// A segment `id` ended (or still going, `done` false) at `t1`, pinned
    /// on `name`, with plenty of votes.
    fn seg(id: u64, name: Option<&str>, t1: Instant, done: bool) -> Seg {
        Seg {
            id,
            client: 1,
            start: id * 100_000,
            end: id * 100_000 + 48_000,
            t0: t1 - Duration::from_secs(1),
            t1,
            world: ring_at(0.0),
            bins: 500,
            yaw0: 0.0,
            voiced: 100,
            echoed: 0,
            bot_echo: false,
            hops: 100,
            emitted: Some(100),
            sent: true,
            done,
            who: Some(Attribution {
                name: name.map(String::from),
                user_id: name.map(|n| format!("usr_{n}")),
                world_yaw: None,
                bearing: None,
                confidence: 0.8,
                cues: Value::Null,
                candidates: Value::Null,
                ranked: vec![(name.map(String::from), 0.8)],
            }),
        }
    }

    #[test]
    fn events_have_the_contracts_shape() {
        let mut seg = seg(3, None, Instant::now(), false);
        (seg.start, seg.end) = (123_456, 140_000);
        let who = Attribution {
            name: Some("xkeyC".into()),
            user_id: Some("usr_1".into()),
            world_yaw: Some(10.0),
            bearing: Some(-32.54),
            confidence: 0.8213,
            cues: json!({"doa": 0.9, "glow": 0.7, "evidence": "glow"}),
            candidates: Value::Null,
            ranked: vec![(Some("xkeyC".into()), 0.8213), (None, 0.1234)],
        };
        let v = event(&seg, &who, false);
        assert_eq!(
            v,
            json!({"type": "speaker", "start": 123456, "end": 140000, "final": false, "name": "xkeyC", "user_id": "usr_1",
                   "bearing_deg": -32.5, "confidence": 0.82, "cues": {"doa": 0.9, "glow": 0.7, "evidence": "glow"},
                   "candidates": [{"name": "xkeyC", "p": 0.82}, {"name": null, "p": 0.12}]})
        );
        let echo = event(&seg, &Attribution::unknown(json!({"bot_echo": true}), 0.0), true);
        assert_eq!(echo["name"], Value::Null);
        assert_eq!(echo["user_id"], Value::Null);
        assert_eq!(echo["final"], true);
        assert_eq!(echo["cues"], json!({"bot_echo": true}));
        assert_eq!(echo["candidates"], json!([{"name": null, "p": 1.0}]));
    }

    /// Speech from `az` (head straight ahead at the origin) through the
    /// whole audio path, the blocks read 20 ms apart from `start` on: the
    /// events the client would get. The speech is 0.5-2.0 s after `start`.
    fn hear(s: &Speakers, az: f32, start: Instant, bot_until: Option<Instant>) -> Vec<Value> {
        let head = SphericalHead::default();
        let mut mono = vec![0.0f32; 24_000];
        // A voice-like buzz: harmonics of a pitch gliding round 150 Hz.
        let mut phase = 0.0f32;
        mono.extend((0..72_000).map(|i| {
            let t = i as f32 / 48_000.0;
            phase += 2.0 * std::f32::consts::PI * (150.0 + 40.0 * (2.0 * std::f32::consts::PI * 3.0 * t).sin()) / 48_000.0;
            (1..=25).map(|h| (phase * h as f32).sin() / h as f32).sum::<f32>() * 0.03
        }));
        mono.extend(vec![0.0; 33_600]);
        let (l, r) = render(&head, az, 0.0, &mono);
        let mut events = Vec::new();
        let mut clock = 0;
        for (i, (lb, rb)) in l.chunks_exact(960).zip(r.chunks_exact(960)).enumerate() {
            s.mix(lb, rb);
            let at = start + Duration::from_millis(20 * (i as u64 + 1));
            events.extend(s.analyse(lb, rb, clock, true, 1, None, bot_until, at));
            clock += 960;
        }
        events
    }

    #[test]
    fn a_voice_from_a_placed_player_is_pinned_on_them() {
        let s = Speakers::new(&args(&[]));
        // Bob 40 degrees to the right, 2 m away; Ann to the left.
        let (sn, cs) = 40f32.to_radians().sin_cos();
        s.place("Bob", [2.0 * sn, 0.0, -2.0 * cs]);
        s.place("Ann", [-2.0, 0.0, 0.0]);
        let events = hear(&s, 40.0, Instant::now(), None);
        let finals: Vec<&Value> = events.iter().filter(|e| e["final"] == true).collect();
        assert_eq!(finals.len(), 1, "{events:?}");
        let f = finals[0];
        assert_eq!(f["name"], "Bob", "{f}");
        assert_eq!(f["user_id"], "usr_Bob");
        assert!((f["bearing_deg"].as_f64().unwrap() - 40.0).abs() < 1.0, "{f}");
        // On the clock: speech from 24000 for 72000 samples.
        assert!((f["start"].as_i64().unwrap() - 24_000).abs() <= 960, "{f}");
        assert!((f["end"].as_i64().unwrap() - 96_000).abs() <= 3_400, "{f}");
        // Sent along the way too, every ~250 ms, all for the same start.
        let running: Vec<&Value> = events.iter().filter(|e| e["final"] == false).collect();
        assert!((5..=7).contains(&running.len()), "{}", running.len());
        assert!(running.iter().all(|e| e["start"] == f["start"]));
        assert_eq!(s.source(Duration::from_secs(2), None, false).map(|src| src.name), Some(Some("Bob".to_string())));
        assert!(s.talking("Bob") && !s.talking("Ann"));
    }

    #[test]
    fn the_voice_for_the_lens_has_a_bearing() {
        // Nobody placed: the votes' peak.
        let s = Speakers::new(&args(&[]));
        hear(&s, 40.0, Instant::now(), None);
        let v = s.voice(Duration::from_secs(5)).unwrap();
        assert!(!v.open && v.from == "direction" && (v.yaw - 40.0).abs() < 3.0, "{v:?}");
        // Pinned on a placed player: their way.
        let s = Speakers::new(&args(&[]));
        let (sn, cs) = 40f32.to_radians().sin_cos();
        s.place("Bob", [2.0 * sn, 0.0, -2.0 * cs]);
        hear(&s, 40.0, Instant::now(), None);
        let v = s.voice(Duration::from_secs(5)).unwrap();
        assert!(v.from == "candidate" && (v.yaw - 40.0).abs() < 0.5, "{v:?}");
        // The bot's own echo: none.
        let s = Speakers::new(&args(&[]));
        let start = Instant::now();
        hear(&s, 40.0, start, Some(start + Duration::from_millis(2500)));
        assert!(s.voice(Duration::from_secs(5)).is_none());
    }

    #[test]
    fn nothing_is_pinned_while_the_bot_speaks() {
        let s = Speakers::new(&args(&[]));
        s.place("Bob", [1.0, 0.0, -1.0]);
        let start = Instant::now();
        let events = hear(&s, 45.0, start, Some(start + Duration::from_millis(2500)));
        let f = events.iter().find(|e| e["final"] == true).unwrap();
        assert_eq!(f["name"], Value::Null);
        assert_eq!(f["cues"]["bot_echo"], true);
        assert!(s.source(Duration::from_secs(2), None, false).is_none());
    }

    #[test]
    fn a_reply_right_after_the_bot_is_still_pinned() {
        // The bot's voice plays out 0.4 s in, the echo tail runs to 0.8 s:
        // the first ~0.35 s of the 1.5 s reply are left out, not all of it.
        let s = Speakers::new(&args(&[]));
        let (sn, cs) = 40f32.to_radians().sin_cos();
        s.place("Bob", [2.0 * sn, 0.0, -2.0 * cs]);
        let start = Instant::now();
        let events = hear(&s, 40.0, start, Some(start + Duration::from_millis(400)));
        let f = events.iter().find(|e| e["final"] == true).unwrap();
        assert_eq!(f["name"], "Bob", "{f}");
        assert!((f["bearing_deg"].as_f64().unwrap() - 40.0).abs() < 1.0, "{f}");
        let seg = s.state.lk().segments.back().cloned().unwrap();
        assert!(!seg.bot_echo && seg.echoed > 10 && seg.echoed < seg.voiced / 2, "{} of {}", seg.echoed, seg.voiced);
        assert!(s.source(Duration::from_secs(2), None, true).is_some());
        // A longer tail covers most of it: nobody's.
        let s = Speakers::new(&args(&["--echo-tail-ms", "1500"]));
        s.place("Bob", [2.0 * sn, 0.0, -2.0 * cs]);
        let start = Instant::now();
        let events = hear(&s, 40.0, start, Some(start + Duration::from_millis(400)));
        let f = events.iter().find(|e| e["final"] == true).unwrap();
        assert_eq!(f["name"], Value::Null, "{f}");
        assert_eq!(f["cues"]["bot_echo"], true);
    }

    #[test]
    fn attending_picks_the_named_the_ended_or_the_running_speech() {
        let s = Speakers::new(&args(&[]));
        s.place("Ann", [-2.0, 0.0, 0.0]);
        s.place("Bob", [2.0, 0.0, 0.0]);
        let now = Instant::now();
        {
            let mut st = s.state.lk();
            st.segments.push_back(seg(1, Some("Ann"), now - Duration::from_millis(800), true));
            st.segments.push_back(seg(2, None, now - Duration::from_millis(500), true));
            st.open = Some(seg(3, Some("Bob"), now, false));
        }
        let pick = |since_ms: u64, name: Option<&str>, ended_first: bool| {
            let src = s.source(Duration::from_millis(since_ms), name, ended_first).unwrap();
            (src.segment, src.picked, src.name)
        };
        // A wake word: the speech going on.
        assert_eq!(pick(2000, None, false), (3, "running", Some("Bob".into())));
        // A transcript: the latest that ended, before what goes on now.
        assert_eq!(pick(4000, None, true), (2, "ended", None));
        // A transcript with its speaker: theirs.
        assert_eq!(pick(4000, Some("Ann"), true), (1, "named", Some("Ann".into())));
        assert_eq!(pick(4000, Some("Bob"), true), (3, "named", Some("Bob".into())));
        // Nothing of theirs within reach: as without a name.
        assert_eq!(pick(600, Some("Ann"), true), (2, "ended", None));
        assert_eq!(pick(300, Some("Cid"), true), (3, "running", Some("Bob".into())));
    }

    #[test]
    fn attending_an_unpinned_voice_turns_to_whoever_is_on_its_side() {
        // The votes peak 30 degrees right; Bob stands at 150 (its front/back
        // mirror, where a voice from behind peaks ahead), Ann on the left.
        let s = Speakers::new(&args(&[]));
        let (sn, cs) = 150f32.to_radians().sin_cos();
        s.place("Bob", [2.0 * sn, 0.0, -2.0 * cs]);
        s.place("Ann", [-2.0, 0.0, 0.0]);
        let now = Instant::now();
        {
            let mut st = s.state.lk();
            let mut seg = seg(1, None, now - Duration::from_millis(300), true);
            seg.world = ring_at(30.0);
            st.segments.push_back(seg);
        }
        let src = s.source(Duration::from_secs(2), None, true).unwrap();
        assert_eq!((src.from, src.name.as_deref()), ("candidate side", None));
        assert!((src.world_yaw - 150.0).abs() < 1.0, "{}", src.world_yaw);
        // The transcript names him: his name goes with it.
        let src = s.source(Duration::from_secs(2), Some("Bob"), true).unwrap();
        assert_eq!((src.from, src.name.as_deref()), ("candidate side", Some("Bob")));
        // Named someone on the other side: still the one on the voice's side.
        let src = s.source(Duration::from_secs(2), Some("Ann"), true).unwrap();
        assert_eq!(src.name, None);
        assert!((src.world_yaw - 150.0).abs() < 1.0);
    }

    #[test]
    fn the_bot_turns_to_a_voice_only_when_idle() {
        assert_eq!(attend_busy(false, false, false), None);
        assert_eq!(attend_busy(true, false, false), Some("following"));
        assert_eq!(attend_busy(true, true, true), Some("following"));
        assert_eq!(attend_busy(false, true, false), Some("moving"));
        assert_eq!(attend_busy(false, false, true), Some("menu"));
    }

    #[test]
    fn a_recording_is_written_on_its_own_thread_and_stops_when_told() {
        let s = Speakers::new(&args(&[]));
        let v = s.record(60.0).unwrap();
        let dir = PathBuf::from(v["dir"].as_str().unwrap());
        hear(&s, 30.0, Instant::now(), None);
        let stopped = s.stop_recording().unwrap();
        assert_eq!(stopped["dir"], v["dir"]);
        assert!(s.stop_recording().is_err());
        let meta = dir.join("meta.json");
        let since = Instant::now();
        while !meta.exists() && since.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(20));
        }
        let meta: Value = serde_json::from_slice(&std::fs::read(meta).unwrap()).unwrap();
        assert!(meta["seconds"].as_f64().unwrap() < 10.0, "{meta}");
        let segments: Value = serde_json::from_slice(&std::fs::read(dir.join("segments.json")).unwrap()).unwrap();
        assert_eq!(segments.as_array().unwrap().len(), 1);
        // 2.7 s heard: the WAV's frames and the hops' lines.
        let wav = std::fs::read(dir.join("audio.wav")).unwrap();
        let frames = u32::from_le_bytes(wav[40..44].try_into().unwrap()) as usize / 8;
        assert_eq!(frames, (wav.len() - 44) / 8);
        assert_eq!(frames, 129_600);
        let hops = std::fs::read_to_string(dir.join("hops.jsonl")).unwrap();
        assert_eq!(hops.lines().count(), 270);
        assert_eq!(std::fs::metadata(dir.join("votes.f32")).unwrap().len(), 270 * 180 * 4);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_plate_lights_at_once_over_its_own_quiet_look() {
        let g = GlowParams { on: 0.30, off: 0.15, release_s: 0.9 };
        let mut p = Plate::default();
        let t = Instant::now();
        let at = |s: f32| t + Duration::from_secs_f32(s);
        for i in 0..10 {
            p.update(at(i as f32 * 0.25), 0.15, LookFrom::Eye, &g, false);
        }
        assert!(!p.on);
        assert!((p.base[0].unwrap() - 0.15).abs() < 0.01);
        // One lit look is enough; a dip under `on` but over `off` keeps it.
        p.update(at(2.5), 1.0, LookFrom::Eye, &g, false);
        assert!(p.on);
        p.update(at(2.75), 0.35, LookFrom::Eye, &g, false);
        assert!(p.on);
        p.update(at(3.0), 0.2, LookFrom::Eye, &g, false);
        assert!(!p.on);
        // The orbit's view has its own quiet look (a busier background).
        p.update(at(3.1), 0.4, LookFrom::Orbit, &g, false);
        assert!(!p.on, "the orbit's first look is over the prior, not over the eye's quiet look");
        assert!(p.base[1].is_some());
        // While the bot's voice may be playing back, the quiet look stays,
        // and a ring then counts for nobody.
        p.update(at(4.0), 0.05, LookFrom::Eye, &g, true);
        assert!((p.base[0].unwrap() - 0.15).abs() < 0.03);
        p.update(at(4.25), 1.0, LookFrom::Eye, &g, true);
        assert!(p.on);
        assert_eq!(p.cue(at(4.2), at(4.3), &g), None);
        // Looks may come late (the orbit's OCR): kept in time order.
        p.update(at(4.1), 0.1, LookFrom::Orbit, &g, false);
        assert!(p.history.iter().zip(p.history.iter().skip(1)).all(|(a, b)| a.t <= b.t));
    }

    #[test]
    fn a_plate_lit_for_long_learns_its_new_quiet_look() {
        let g = GlowParams { on: 0.30, off: 0.15, release_s: 0.9 };
        let mut p = Plate::default();
        let t = Instant::now();
        for i in 0..400 {
            p.update(t + Duration::from_millis(250 * i), 0.7, LookFrom::Orbit, &g, false);
        }
        assert!(!p.on, "{:?}", p.base);
    }

    /// Looks at a plate every `every` s from `from` to `to` (s), lit within
    /// `lit` (s).
    fn looks(g: &GlowParams, t: Instant, every: f32, from: f32, to: f32, lit: &[(f32, f32)]) -> Plate {
        let mut p = Plate::default();
        let mut s = from;
        while s <= to + 1e-4 {
            let on = lit.iter().any(|&(a, b)| s >= a && s <= b);
            p.update(t + Duration::from_secs_f32(s), if on { 1.0 } else { 0.15 }, LookFrom::Eye, g, false);
            s += every;
        }
        p
    }

    #[test]
    fn the_rings_onset_is_matched_to_the_speech_start() {
        let g = GlowParams { on: 0.30, off: 0.15, release_s: 0.9 };
        let t = Instant::now() + Duration::from_secs(1);
        let at = |s: f32| t + Duration::from_secs_f32(s);
        // Speech heard from 5.0 to 6.0 s: the ring on from 4.9 to 6.9 s
        // (looked at every 0.25 s).
        let speaker = looks(&g, t, 0.25, 0.0, 10.0, &[(4.9, 6.9)]);
        let cue = speaker.cue(at(5.0), at(6.0), &g).unwrap();
        assert!(cue.onset && cue.share > 0.9, "{cue:?}");
        // Someone who stopped at 4.6 s: their ring stays lit to 5.5 s, over
        // the new speech's start, but that is its tail: no onset, no share.
        let before = looks(&g, t, 0.25, 0.0, 10.0, &[(2.0, 5.5)]);
        let cue = before.cue(at(5.0), at(6.0), &g).unwrap();
        assert!(!cue.onset && cue.share == 0.0, "{cue:?}");
        // The same person going on after a short pause: lit through, no new
        // onset, and the looks during it count.
        let going_on = looks(&g, t, 0.25, 0.0, 10.0, &[(2.0, 6.9)]);
        let cue = going_on.cue(at(5.0), at(6.0), &g).unwrap();
        assert!(!cue.onset && cue.share > 0.9, "{cue:?}");
        // An onset a second late is someone else's.
        let late = looks(&g, t, 0.25, 0.0, 10.0, &[(6.0, 8.0)]);
        assert!(!late.cue(at(5.0), at(6.5), &g).unwrap().onset);
        // Seen dark all through.
        let dark = looks(&g, t, 0.25, 0.0, 10.0, &[]);
        assert_eq!(dark.cue(at(5.0), at(6.0), &g), Some(GlowCue { onset: false, share: 0.0, eye: false }));
        // Looked at only every second (the orbit): the onset still within
        // the slack, half the gap.
        let sparse = looks(&g, t, 1.0, 0.0, 10.0, &[(4.9, 6.9)]);
        assert!(sparse.cue(at(5.0), at(6.0), &g).unwrap().onset);
        // In fusion an aligned onset settles it against a voice in between.
        let mut ring = ring_at(30.0);
        ring.add_turned(&ring_at(150.0).p, 0.0, 1.0);
        let cands = [
            Cand { glow: Some(GlowCue { onset: false, share: 0.0, eye: true }), ..cand("a", 30.0, None) },
            Cand { glow: Some(GlowCue { onset: true, share: 0.5, eye: true }), ..cand("b", 150.0, None) },
        ];
        assert_eq!(fuse(Some(&ring), &cands, 0, 0.0, CAP).best, Some(1));
        // The ring the eyes saw lit, and the lens's alone.
        assert!(speaker.cue(at(5.0), at(6.0), &g).unwrap().eye);
        let mut lens = Plate::default();
        for i in 0..40 {
            let s = i as f32 * 0.25;
            let on = (4.9..=6.9).contains(&s);
            lens.update(t + Duration::from_secs_f32(s), if on { 1.0 } else { 0.15 }, LookFrom::Orbit, &g, false);
        }
        let cue = lens.cue(at(5.0), at(6.0), &g).unwrap();
        assert!(cue.lit() && !cue.eye, "{cue:?}");
    }

    #[test]
    fn a_bearing_alone_places_a_player_that_way() {
        let s = Speakers::new(&args(&[]));
        let now = Instant::now();
        // Read by the orbit 90 degrees to the right (tracking frame).
        s.saw_bearing("Cid", 90.0, now, None);
        {
            let st = s.state.lk();
            let c = st.players.get("Cid").unwrap();
            assert!(c.bearing_only);
            assert!((yaw_to([0.0; 3], c.tag) - 90.0).abs() < 0.1, "{:?}", c.tag);
        }
        // A depth place that agrees is kept; one that does not is replaced.
        s.place("Bob", [2.0, 0.0, -2.0]); // 45 degrees right
        s.saw_bearing("Bob", 50.0, now, None);
        assert!(!s.state.lk().players["Bob"].bearing_only);
        s.saw_bearing("Bob", -60.0, now, None);
        let st = s.state.lk();
        assert!(st.players["Bob"].bearing_only);
        let cands = candidates(&st, now, [0.0; 3]);
        let bob = cands.iter().find(|c| c.name == "Bob").unwrap();
        assert!((bob.world_yaw + 60.0).abs() < 0.1);
    }

    /// A plate as VRChat draws it: the dark pill round a text box `bbox`
    /// (the model's proportions, scaled by `scale` round its middle), an
    /// icon on the left, yellow glyphs, on a navy background with a purple
    /// patch at the right; lit, a 3 px ring of `ring` and ripple arcs.
    fn plate_image(bbox: [f32; 4], scale: f32, ring: Option<[u8; 3]>) -> (usize, usize, Vec<u8>) {
        let (w, h) = (420usize, 220usize);
        let [x, y, bw, bh] = bbox;
        let (cy, r) = (y + bh / 2.0, PILL_HALF * bh * scale);
        let mid = x + bw / 2.0;
        let (left, right) = (mid - (mid - (x - PILL_LEFT * bh)) * scale, mid + (x + bw + PILL_RIGHT * bh - mid) * scale);
        let (cl, cr) = (left + r, right - r);
        let dist = |px: f32, py: f32| {
            if px >= cl && px <= cr {
                (py - cy).abs() - r
            } else {
                let c = if px < cl { cl } else { cr };
                (px - c).hypot(py - cy) - r
            }
        };
        let mut img = vec![0u8; w * h * 3];
        for yy in 0..h {
            for xx in 0..w {
                let (px, py) = (xx as f32 + 0.5, yy as f32 + 0.5);
                let d = dist(px, py);
                let mut c: [u8; 3] = if xx > 330 && yy > 90 { [69, 27, 119] } else { [16, 16, 28] };
                if d <= 0.0 {
                    c = [30, 30, 35];
                    if (px - (cl + 0.1 * r)).hypot(py - cy) < 0.75 * r {
                        c = [180, 150, 130]; // the icon
                    }
                    let in_text = px >= x && px <= x + bw && py >= y + 0.15 * bh && py <= y + 0.85 * bh;
                    if in_text && ((px - x) / (0.16 * bh)) as i32 % 2 == 0 {
                        c = [255, 204, 0];
                    }
                } else if let Some(ring) = ring {
                    if d <= 3.0 {
                        c = ring;
                    } else if (8.0..=10.0).contains(&d) && (px < cl - 0.6 * r || px > cr + 0.6 * r) {
                        c = ring.map(|v| (v as f32 * 0.8) as u8); // a ripple arc
                    }
                }
                img[(yy * w + xx) * 3..(yy * w + xx) * 3 + 3].copy_from_slice(&c);
            }
        }
        (w, h, img)
    }

    const GOLD: [u8; 3] = [242, 190, 50];
    const CYAN: [u8; 3] = [90, 230, 255];

    #[test]
    fn the_ring_is_found_whatever_its_colour() {
        let bbox = [200.0, 95.0, 80.0, 30.0];
        let stats = |ring: Option<[u8; 3]>, scale: f32| {
            let (w, h, img) = plate_image(bbox, scale, ring);
            glow_stats(&View::rgb(&img, w, h), bbox).unwrap()
        };
        let unlit = stats(None, 1.0);
        assert!(unlit.ring < 0.2 && unlit.score() < 0.3, "{unlit:?}");
        for (colour, hue_deg) in [(GOLD, 44.0), (CYAN, 194.0)] {
            let lit = stats(Some(colour), 1.0);
            assert!(lit.ring > 0.7 && lit.score() - unlit.score() > 0.6, "{colour:?}: {lit:?}");
            assert!(hue_apart(lit.hue.unwrap(), hue_deg) < 5.0, "{lit:?}");
            assert!(lit.ripple > 0.05, "{lit:?}");
            // An OCR box a little loose or tight round the text.
            for scale in [0.88, 1.12] {
                let off = stats(Some(colour), scale);
                assert!(off.ring > 0.6, "scale {scale}: {off:?}");
                assert!(stats(None, scale).ring < 0.2);
            }
        }
        // Too small, or no dark fill in the text box: nothing to say.
        let (w, h, img) = plate_image(bbox, 1.0, None);
        assert!(glow_stats(&View::rgb(&img, w, h), [200.0, 95.0, 80.0, 2.0]).is_none());
        assert!(glow_stats(&View::rgb(&img, w, h), [20.0, 5.0, 40.0, 10.0]).is_none());
    }

    fn fixture(name: &str) -> (usize, usize, Vec<u8>) {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
        let mut reader = png::Decoder::new(File::open(path).unwrap()).read_info().unwrap();
        let mut buf = vec![0; reader.output_buffer_size()];
        let info = reader.next_frame(&mut buf).unwrap();
        assert_eq!(info.color_type, png::ColorType::Rgb);
        buf.truncate(info.buffer_size());
        (info.width as usize, info.height as usize, buf)
    }

    #[test]
    fn a_friends_plate_measured_on_2026_10_08() {
        // Crops of the left eye, a friend's plate 2-3 m away: dark (the 60th
        // frame) and lit, the gold ring (the 300th); the name's text box as
        // OCR gives it, a little loose.
        let (w, h, dark) = fixture("plate_unlit.png");
        let unlit = glow_stats(&View::rgb(&dark, w, h), [126.0, 59.0, 78.0, 38.0]).unwrap();
        let (w, h, lit) = fixture("plate_lit_friend.png");
        let gold = glow_stats(&View::rgb(&lit, w, h), [147.0, 63.0, 78.0, 38.0]).unwrap();
        assert!(unlit.ring < 0.2 && unlit.score() < 0.3, "{unlit:?}");
        assert!(gold.ring > 0.6 && gold.score() - unlit.score() > 0.5, "{gold:?}");
        assert!(hue_apart(gold.hue.unwrap(), 43.0) < 8.0, "{gold:?}");
        // The default thresholds over a quiet look of the dark plate.
        let g = GlowParams { on: 0.30, off: 0.15, release_s: 0.9 };
        let mut p = Plate::default();
        let t = Instant::now();
        p.update(t, unlit.score(), LookFrom::Eye, &g, false);
        assert!(!p.on);
        p.update(t + Duration::from_millis(33), gold.score(), LookFrom::Eye, &g, false);
        assert!(p.on);
    }

    /// The whole 2026-10-08 recording when it is here (`VRC_GLOW_RECORDING`:
    /// a directory with `big.raw`, 1801 crops 700x400 RGB at 30 fps, and
    /// `series.csv`, per crop `t,gold,ripple,rms,x,y`: the reference ring
    /// measure and where the plate is); skipped otherwise. Every frame the
    /// reference calls lit (gold > 0.05) must score over the default `on`
    /// above the dark frames' median, and every dark one under `off`.
    #[test]
    fn the_recorded_series_when_it_is_here() {
        let Some(dir) = std::env::var_os("VRC_GLOW_RECORDING").map(PathBuf::from) else { return };
        let csv = std::fs::read_to_string(dir.join("series.csv")).unwrap();
        let raw = std::fs::read(dir.join("big.raw")).unwrap();
        let (w, h) = (700usize, 400usize);
        let mut lit = Vec::new();
        let mut dark = Vec::new();
        for (i, line) in csv.lines().skip(1).enumerate() {
            let v: Vec<f32> = line.split(',').map(|x| x.parse().unwrap()).collect();
            let frame = &raw[i * w * h * 3..(i + 1) * w * h * 3];
            // The text box from the plate's place (as on the fixtures).
            let bbox = [v[4] + 61.0, v[5] + 14.0, 78.0, 38.0];
            let g = glow_stats(&View::rgb(frame, w, h), bbox).unwrap();
            // Leave out the reference's own edges (a frame either side).
            if v[1] > 0.05 {
                lit.push(g.score());
            } else if v[1] == 0.0 {
                dark.push(g.score());
            }
        }
        dark.sort_by(f32::total_cmp);
        lit.sort_by(f32::total_cmp);
        let quiet = dark[dark.len() / 2];
        eprintln!("dark {} {:.3}..{:.3} (median {quiet:.3}), lit {} {:.3}..{:.3}", dark.len(), dark[0], dark[dark.len() - 1], lit.len(), lit[0], lit[lit.len() - 1]);
        assert!(dark[dark.len() - 1] - quiet < 0.15);
        assert!(lit[0] - quiet > 0.30);
    }
}

/// Of `cands`, the one whose side (front and back folded, the head having
/// looked at `yaw0`) is nearest the votes' peak `peak`, within
/// SIDE_MATCH_DEG (all yaws in the fixed frame).
fn side_match(cands: &[Cand], peak: f32, yaw0: f32) -> Option<&Cand> {
    let side = |yaw: f32| fold_front_back(wrap_deg(yaw - yaw0));
    cands
        .iter()
        .map(|c| (c, (side(c.world_yaw) - side(peak)).abs()))
        .filter(|(_, off)| *off <= SIDE_MATCH_DEG)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(c, _)| c)
}

/// Why the bot may not turn to a voice now (`POST /v1/vr/attend`), or None
/// when it may: only when idle. Following (or a follow paused for a move),
/// moving, or the usual view leased (the VR menu): it stays as it is.
pub fn attend_busy(following: bool, moving: bool, leased: bool) -> Option<&'static str> {
    if following {
        Some("following")
    } else if moving {
        Some("moving")
    } else if leased {
        Some("menu")
    } else {
        None
    }
}

/// A bearing (degrees, head-relative) with front and back folded together:
/// -90..90, how far to the right; what a voice's direction tells surely.
fn fold_front_back(rel: f32) -> f32 {
    let rel = wrap_deg(rel);
    if rel > 90.0 {
        180.0 - rel
    } else if rel < -90.0 {
        -180.0 - rel
    } else {
        rel
    }
}

#[cfg(test)]
mod side_tests {
    use super::fold_front_back;

    #[test]
    fn front_and_back_fold_together() {
        // Ahead and right behind are the same side; so are 30 and 150.
        assert!((fold_front_back(0.4) - fold_front_back(-174.9)).abs() < 6.0);
        assert!((fold_front_back(30.0) - fold_front_back(150.0)).abs() < 1e-3);
        assert!((fold_front_back(-30.0) - fold_front_back(-150.0)).abs() < 1e-3);
        // Left and right stay apart.
        assert!((fold_front_back(60.0) - fold_front_back(-60.0)).abs() > 100.0);
        assert_eq!(fold_front_back(90.0), 90.0);
    }
}
