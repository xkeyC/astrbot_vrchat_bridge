//! The avatar's panorama rig in the bridge (`vrc-pano`, docs/full-vr/
//! avatar-panorama.md): the avatar parameter `Pano` (OSC) turns six
//! local-only cameras on, and then every frame of the eyes is the whole
//! sphere: colour in the left eye, depth in the right, a code (0x5B) saying
//! where the cameras are. Here: turning it on and off, what the eyes show
//! now (a panorama, the usual view, or neither yet), the latest pano frame
//! decoded (once per tapped frame), and short leases of the usual view.
//!
//! - `--pano auto` (the default, decision D36): on while the game is in a
//!   world; an avatar that shows no code within AUTO_ANSWER of being asked
//!   (another avatar, no rig, still loading) is let be (`fallback`: the
//!   eyes' usual view, and nothing sees depth: looks, the follower and
//!   the speaker tracker's vision wait) and asked again after RETRY, and at
//!   once on joining a world.
//! - `--pano on`: on from the start, and kept on: VRChat resets the
//!   parameter (unsaved) with a new world or avatar, so it is sent again
//!   every RESEND while wanted.
//! - `--pano off`: the bridge turns it on only when asked (`POST
//!   /v1/vr/pano`).
//!
//! What reads the eyes (looks, the follower, sightings, the speaker
//! tracker's vision) needs the panorama (`usable`): it is the only way the
//! bot sees depth (decision D42; the head scan with stereo is gone). A
//! screenshot without it shows the usual view.
//!
//! A lease of the usual view (`normal_view`: the VR menu, calibration,
//! opening the user camera, a screenshot asked `normal=1`) turns the rig
//! off, waits for a frame of the usual view, and gives the rig back LINGER
//! after the last lease ends (a menu worked in several steps keeps its
//! view). Menu work over HTTP (`/v1/vr/hand`, `/v1/vr/input`) holds the
//! usual view a while (`hold_normal`) without waiting for it.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use serde_json::{json, Value};
use vrc_pano::{classify, PanoCode, PanoFrame, PanoParams, Seen};
use vrc_vr::osc::{Arg, Osc};
use vrc_vr::tap::{EyeFrame, EyeTap};

use crate::Lock;

/// The avatar parameter.
pub const PARAM: &str = "/avatar/parameters/Pano";
/// A rig asked on shows its code within this (it draws from the next
/// frame; age 2 a frame later).
const ANSWER: Duration = Duration::from_millis(500);
/// While wanted on, the parameter is sent again this often.
const RESEND: Duration = Duration::from_secs(5);
/// `auto`: an avatar shows its code within this (a few seconds: it may be
/// loading), or it is let be and asked again after RETRY.
const AUTO_ANSWER: Duration = Duration::from_secs(3);
const RETRY: Duration = Duration::from_secs(60);
/// The rig comes back this long after the last lease of the usual view.
const LINGER: Duration = Duration::from_secs(3);
/// A frame of the usual view counts this long after the rig was asked off
/// (VRChat applies OSC within a frame or two; the tap may be a frame late).
const SETTLE: Duration = Duration::from_millis(120);
/// The watcher's beat; frames looked at this often while waiting for an
/// answer (on or off), and every OBSERVE_EVERY otherwise while wanted on.
const TICK: Duration = Duration::from_millis(100);
const OBSERVE_EVERY: Duration = Duration::from_secs(1);

#[derive(clap::ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Setting {
    Off,
    On,
    Auto,
}

impl Setting {
    fn name(self) -> &'static str {
        match self {
            Setting::Off => "off",
            Setting::On => "on",
            Setting::Auto => "auto",
        }
    }
}

/// What a frame looked at showed.
#[derive(Clone, Copy, Debug)]
pub struct Observed {
    pub seen: Seen,
    pub at: Instant,
    pub tap_seq: u64,
}

/// The rig's wanted and observed state, without the I/O (tested alone).
#[derive(Debug)]
pub struct State {
    pub setting: Setting,
    /// On wanted (the setting, or the debug route).
    pub wanted: bool,
    /// Leases of the usual view held, and when the last ended.
    pub leases: u32,
    pub released: Option<Instant>,
    /// What was sent last, and when.
    pub sent: Option<(bool, Instant)>,
    /// When it was asked on last; whether a code showed since.
    pub asked: Option<Instant>,
    pub answered: bool,
    /// Asked on, no code showed within ANSWER: not this avatar's (auto:
    /// left off till RETRY).
    pub fallback: bool,
    /// Asked on at some time (else off needs no looking at).
    pub ever_on: bool,
    pub observed: Option<Observed>,
    /// The last code read, with the head then.
    pub last_code: Option<(PanoCode, vrc_pano::HeadPose, Instant)>,
    /// The game in a world (`auto` turns it on only then).
    pub in_world: bool,
    /// The usual view held for menu work over HTTP until then.
    pub hold_until: Option<Instant>,
}

impl State {
    pub fn new(setting: Setting) -> State {
        State {
            setting,
            wanted: setting != Setting::Off,
            leases: 0,
            released: None,
            sent: None,
            asked: None,
            answered: false,
            fallback: false,
            ever_on: false,
            observed: None,
            last_code: None,
            in_world: true,
            hold_until: None,
        }
    }

    /// How long an answer may take.
    fn answer(&self) -> Duration {
        if self.setting == Setting::Auto {
            AUTO_ANSWER
        } else {
            ANSWER
        }
    }

    /// Whether the rig should be on now.
    pub fn effective(&self, now: Instant) -> bool {
        let lingering = self.released.is_some_and(|t| now - t < LINGER);
        let auto = self.setting == Setting::Auto;
        let given_up = auto && self.fallback && self.asked.is_some_and(|t| now - t < RETRY);
        let held = self.hold_until.is_some_and(|t| now < t);
        self.wanted && self.leases == 0 && !lingering && !given_up && !held && (!auto || self.in_world)
    }

    /// The game is (or is not) in a world now. Joining one, `auto` asks the
    /// avatar afresh (the parameter was reset; a fallback while it loaded
    /// is forgotten).
    pub fn set_in_world(&mut self, in_world: bool) {
        if in_world && !self.in_world && self.setting == Setting::Auto {
            self.fallback = false;
            self.asked = None;
            if self.sent.is_some_and(|(on, _)| on) {
                self.sent = None;
            }
        }
        self.in_world = in_world;
    }

    /// What to send now, if anything: a change, or on again every RESEND.
    pub fn tick(&mut self, now: Instant) -> Option<bool> {
        // Asked on and nothing showed: not this avatar's.
        let asking = self.sent.is_some_and(|(on, _)| on);
        if asking && !self.answered && !self.fallback && self.asked.is_some_and(|t| now - t >= self.answer()) {
            self.fallback = true;
            tracing::warn!("pano: no 0x5B code within {} ms of asking: the avatar has no rig (fallback)", self.answer().as_millis());
        }
        let on = self.effective(now);
        let send = match self.sent {
            Some((was, at)) => was != on || (on && now - at >= RESEND),
            // At start: off once too (a bridge before may have left it on).
            None => true,
        };
        if !send {
            return None;
        }
        if on && !asking {
            // A new ask: the answer is awaited afresh.
            self.asked = Some(now);
            self.answered = false;
            self.fallback = false;
            self.ever_on = true;
        }
        self.sent = Some((on, now));
        Some(on)
    }

    pub fn observe(&mut self, seen: Seen, tap_seq: u64, now: Instant) {
        if let Seen::Pano { code, head } = seen {
            self.last_code = Some((code, head, now));
        }
        if !matches!(seen, Seen::Normal) {
            self.answered = true;
            self.fallback = false;
        }
        self.observed = Some(Observed { seen, at: now, tap_seq });
    }

    /// The eyes may not show the usual view: the rig was asked on, or a
    /// frame looked at did not.
    fn maybe_not_normal(&self) -> bool {
        self.sent.is_some_and(|(on, _)| on) || self.observed.is_some_and(|o| !matches!(o.seen, Seen::Normal))
    }

    /// Whether the watcher should look at a frame now.
    fn wants_frame(&self, now: Instant) -> bool {
        let Some((on, sent)) = self.sent else { return false };
        let last = self.observed.map(|o| o.at);
        let since_sent = last.is_none_or(|t| t < sent);
        if on {
            // Waiting for the answer, then now and then.
            (since_sent && now - sent < 2 * self.answer()) || last.is_none_or(|t| now - t >= OBSERVE_EVERY)
        } else {
            // Turned off after being on: until the usual view is back.
            (self.ever_on && since_sent) || self.observed.is_some_and(|o| !matches!(o.seen, Seen::Normal) && now - o.at >= TICK)
        }
    }
}

/// The rig, its frames and the leases.
pub struct Pano {
    pub state: Mutex<State>,
    osc: Osc,
    tap_path: String,
    tap: Mutex<Option<EyeTap>>,
    /// The latest pano frame decoded (by its tap seq), with the tapped
    /// frame (the eyes' tracking poses; the UI over the eyes).
    cache: Mutex<Option<(Arc<PanoFrame>, Arc<EyeFrame>)>>,
    params: PanoParams,
}

/// The usual view, held: the rig comes back LINGER after the last one
/// is dropped.
pub struct NormalView {
    pano: Arc<Pano>,
}

impl Drop for NormalView {
    fn drop(&mut self) {
        let mut s = self.pano.state.lk();
        s.leases = s.leases.saturating_sub(1);
        s.released = Some(Instant::now());
    }
}

impl Pano {
    pub fn new(args: &crate::Args) -> Arc<Pano> {
        Arc::new(Pano {
            state: Mutex::new(State::new(args.pano)),
            osc: Osc::with_ports(&format!("127.0.0.1:{}", args.osc_port), 0).expect("an OSC socket"),
            tap_path: args.tap.clone(),
            tap: Mutex::new(None),
            cache: Mutex::new(None),
            params: PanoParams::default(),
        })
    }

    /// Wants the rig on or off (the setting stays; a lease still wins).
    pub fn want_pano(&self, on: bool) {
        let mut s = self.state.lk();
        if s.wanted != on {
            tracing::info!(on, "pano: wanted");
        }
        s.wanted = on;
        if on && s.fallback {
            // Asked by hand: ask the avatar again now.
            s.fallback = false;
            s.sent = None;
        }
        drop(s);
        self.step(Instant::now());
    }

    /// Sends what the state says (if anything).
    fn step(&self, now: Instant) {
        let send = self.state.lk().tick(now);
        if let Some(on) = send {
            if let Err(e) = self.osc.send(PARAM, &[Arg::Bool(on)]) {
                tracing::warn!("pano: OSC {PARAM} {on}: {e:#}");
            }
        }
    }

    /// The latest frame of the eyes, from the rig's own tap (asking Monado
    /// only when nobody else does).
    fn read(&self) -> Result<EyeFrame> {
        let mut tap = self.tap.lk();
        let tap = tap.get_or_insert_with(|| EyeTap::open(&self.tap_path));
        let frame = if tap.tapping() { tap.read_unasked()? } else { tap.read()? };
        frame.context("no frame of the eyes yet (is the game in VR mode?)")
    }

    /// Looks at the latest frame: what it shows (noted in the state).
    pub fn observe_now(&self) -> Result<(EyeFrame, Seen)> {
        let frame = self.read()?;
        let seen = classify(&frame);
        self.state.lk().observe(seen, frame.seq, Instant::now());
        Ok((frame, seen))
    }

    /// The latest pano frame, decoded (once per tapped frame); an error
    /// says why there is none.
    pub fn frame(&self) -> Result<Arc<PanoFrame>> {
        self.frame_and_eyes().map(|(p, _)| p)
    }

    /// Whether a pano frame is to be had now, without reading one: the rig
    /// wanted on and in effect, and the last frame looked at (within
    /// FRESH) a panorama.
    pub fn usable(&self) -> bool {
        const FRESH: Duration = Duration::from_secs(3);
        let s = self.state.lk();
        let now = Instant::now();
        s.effective(now) && s.observed.is_some_and(|o| o.seen.is_pano() && now - o.at < FRESH)
    }

    /// The latest pano frame and the tapped frame it came from.
    pub fn frame_and_eyes(&self) -> Result<(Arc<PanoFrame>, Arc<EyeFrame>)> {
        let (frame, seen) = self.observe_now()?;
        let Seen::Pano { code, head } = seen else {
            let s = self.state.lk();
            let why = match seen {
                Seen::Unusable(why) => why.to_string(),
                _ if !s.wanted => "the rig is off (POST /v1/vr/pano {\"on\": true})".into(),
                _ if s.fallback => "no 0x5B code: the avatar has no panorama rig".into(),
                _ if s.leases > 0 => "the usual view is leased (the VR menu, calibration)".into(),
                _ => "the eyes show the usual view still".into(),
            };
            bail!("no pano frame: {why}");
        };
        if let Some(c) = self.cache.lk().as_ref().filter(|c| c.0.tap_seq == frame.seq) {
            return Ok(c.clone());
        }
        let p = Arc::new(vrc_pano::decode_as(&frame, code, head, &self.params)?);
        let c = (p, Arc::new(frame));
        *self.cache.lk() = Some(c.clone());
        Ok(c)
    }

    /// The usual view, for the VR menu or a calibration: the rig off, and a
    /// frame of the usual view seen (at most `wait`). Free when the rig was
    /// never on.
    pub fn normal_view(self: &Arc<Self>, wait: Duration) -> Result<NormalView> {
        let lease = {
            let mut s = self.state.lk();
            s.leases += 1;
            NormalView { pano: self.clone() }
        };
        if !self.state.lk().maybe_not_normal() {
            return Ok(lease);
        }
        self.step(Instant::now());
        // When it was asked off: VRChat takes a frame or two to apply it, and
        // the latest frame may be older still.
        let off_at = self.state.lk().sent.map_or_else(Instant::now, |(_, at)| at);
        let since = Instant::now();
        while since.elapsed() < wait {
            if let Ok((_, Seen::Normal)) = self.observe_now() {
                if off_at.elapsed() >= SETTLE {
                    return Ok(lease);
                }
            }
            std::thread::sleep(Duration::from_millis(30));
        }
        bail!("the eyes still do not show the usual view {} ms after turning the panorama off", wait.as_millis())
    }

    /// The usual view held for menu work (`for_` from now; longer holds
    /// stay): the rig off meanwhile, back LINGER after. No waiting for it
    /// (a lease does that).
    pub fn hold_normal(&self, for_: Duration) {
        let now = Instant::now();
        {
            let mut s = self.state.lk();
            let until = now + for_;
            s.hold_until = Some(s.hold_until.map_or(until, |t| t.max(until)));
            s.released = Some(until);
        }
        self.step(now);
    }

    /// Keeps the parameter as the state wants it, and looks at frames
    /// while an answer is due (a thread of its own); `in_world` says
    /// whether the game is in a world.
    pub fn run(self: Arc<Self>, in_world: impl Fn() -> bool) {
        loop {
            let now = Instant::now();
            let w = in_world();
            self.state.lk().set_in_world(w);
            self.step(now);
            if self.state.lk().wants_frame(now) {
                if let Err(e) = self.observe_now() {
                    tracing::debug!("pano: no frame to look at: {e:#}");
                }
                // An answer looked at now settles the fallback at once.
                self.step(Instant::now());
            }
            std::thread::sleep(TICK);
        }
    }

    pub fn status(&self) -> Value {
        let now = Instant::now();
        let s = self.state.lk();
        let ms = |t: Instant| (now - t).as_millis() as u64;
        let observed = s.observed.map(|o| {
            json!({
                "mode": o.seen.name(),
                "why": match o.seen { Seen::Unusable(why) => Value::from(why), _ => Value::Null },
                "ms_ago": ms(o.at),
                "tap_seq": o.tap_seq,
            })
        });
        let code = s.last_code.map(|(c, head, at)| {
            json!({
                "position": c.position,
                "rig_yaw": r2(c.rig_yaw),
                "seq": c.seq,
                "age": c.age,
                "layout": c.layout,
                "route": format!("{:?}", c.route),
                "depth_code": c.depth_code,
                "range_m": [c.zmin, c.zmax],
                "head": {"position": head.position, "yaw": r2(head.yaw), "pitch": r2(head.pitch)},
                "ms_ago": ms(at),
            })
        });
        let frame = self.cache.lk().as_ref().map(|(p, _)| {
            json!({
                "tap_seq": p.tap_seq,
                "decode_ms": r2(p.decode_ms),
                "calibration_residual": r2(p.calibration.residual),
                "calibration_cells_out": {"covered": p.calibration.covered, "dropped": p.calibration.dropped},
                "vignette": {"a": r2(p.calibration.channels[0].a), "b": r2(p.calibration.channels[0].b)},
                "check": {"rg_p50_p99": p.check.rg, "b_p50_p99": p.check.b, "overlay": r2(p.check.overlay)},
                "depth_ordinal": p.depth_ordinal,
                "scaled": p.scaled,
                "eye_size": p.eye_size,
            })
        });
        json!({
            "setting": s.setting.name(),
            "wanted": s.wanted,
            "effective": s.effective(now),
            "leases": s.leases,
            "sent": s.sent.map(|(on, at)| json!({"on": on, "ms_ago": ms(at)})),
            "asked_ms_ago": s.asked.map(ms),
            "fallback": s.fallback,
            "in_world": s.in_world,
            "held_normal_ms": s.hold_until.filter(|t| *t > now).map(|t| (t - now).as_millis() as u64),
            "observed": observed,
            "last_code": code,
            "frame": frame,
        })
    }
}

fn r2(v: f32) -> f64 {
    (v as f64 * 100.0).round() / 100.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pano_seen() -> Seen {
        let code = PanoCode {
            position: [1.0, 1.2, 3.0],
            rig_yaw: 140.0,
            seq: 1,
            age: 15,
            layout: 1,
            route: vrc_pano::Route::D1,
            depth_code: vrc_pano::E1C,
            zmin: 0.25,
            zmax: 64.0,
        };
        let head = vrc_pano::HeadPose { position: [1.0, 1.2, 3.0], yaw: 140.0, pitch: 0.0, seq: 1, eyes: 2 };
        Seen::Pano { code, head }
    }

    #[test]
    fn off_sends_off_once_and_stays_quiet() {
        let t0 = Instant::now();
        let mut s = State::new(Setting::Off);
        assert_eq!(s.tick(t0), Some(false));
        assert_eq!(s.tick(t0 + Duration::from_secs(30)), None);
        // Never on: no frame looked at, a lease is free.
        assert!(!s.wants_frame(t0 + Duration::from_secs(30)));
        assert!(!s.maybe_not_normal());
    }

    #[test]
    fn on_is_sent_again_and_answered() {
        let t0 = Instant::now();
        let mut s = State::new(Setting::On);
        assert_eq!(s.tick(t0), Some(true));
        assert!(s.wants_frame(t0 + TICK));
        s.observe(pano_seen(), 2, t0 + Duration::from_millis(100));
        assert_eq!(s.tick(t0 + Duration::from_millis(200)), None);
        assert!(!s.fallback && s.last_code.is_some());
        // Kept on: sent again.
        assert_eq!(s.tick(t0 + RESEND), Some(true));
    }

    #[test]
    fn no_code_is_a_fallback_and_auto_gives_up_till_retry() {
        let t0 = Instant::now();
        for setting in [Setting::On, Setting::Auto] {
            let mut s = State::new(setting);
            assert_eq!(s.tick(t0), Some(true));
            s.observe(Seen::Normal, 2, t0 + Duration::from_millis(300));
            // Not yet: `auto` gives the avatar a few seconds (it may load).
            if setting == Setting::Auto {
                assert_eq!(s.tick(t0 + ANSWER + Duration::from_millis(10)), None);
                assert!(!s.fallback);
            }
            let t = t0 + s.answer() + Duration::from_millis(10);
            let sent = s.tick(t);
            assert!(s.fallback);
            match setting {
                // `on` keeps asking.
                Setting::On => assert_eq!(sent, None),
                // `auto` lets the avatar be...
                _ => {
                    assert_eq!(sent, Some(false));
                    assert!(!s.effective(t + Duration::from_secs(10)));
                    // ... and asks again after RETRY.
                    assert_eq!(s.tick(t0 + RETRY + Duration::from_millis(10)), Some(true));
                    assert!(s.asked.is_some_and(|a| a > t));
                    s.observe(pano_seen(), 9, t0 + RETRY + Duration::from_millis(200));
                    assert!(!s.fallback && s.effective(t0 + RETRY + Duration::from_secs(1)));
                }
            }
        }
    }

    #[test]
    fn auto_is_on_in_a_world_and_asks_afresh_on_joining_one() {
        let t0 = Instant::now();
        let mut s = State::new(Setting::Auto);
        s.set_in_world(false);
        // Between worlds: off.
        assert_eq!(s.tick(t0), Some(false));
        assert!(!s.effective(t0));
        // In a world: on, asked.
        s.set_in_world(true);
        assert_eq!(s.tick(t0 + TICK), Some(true));
        // No code (the avatar loading): let be...
        let t = t0 + TICK + AUTO_ANSWER + Duration::from_millis(10);
        assert_eq!(s.tick(t), Some(false));
        assert!(s.fallback);
        // ... until the next world: asked again at once.
        s.set_in_world(false);
        s.tick(t + TICK);
        s.set_in_world(true);
        assert!(!s.fallback);
        assert_eq!(s.tick(t + 2 * TICK), Some(true));
        s.observe(pano_seen(), 3, t + 3 * TICK);
        assert!(s.effective(t + 4 * TICK));
        // `on` does not care about worlds.
        let mut on = State::new(Setting::On);
        on.set_in_world(false);
        assert_eq!(on.tick(t0), Some(true));
    }

    #[test]
    fn menu_work_holds_the_usual_view_a_while() {
        let t0 = Instant::now();
        let mut s = State::new(Setting::On);
        s.tick(t0);
        s.observe(pano_seen(), 2, t0 + Duration::from_millis(100));
        s.hold_until = Some(t0 + Duration::from_secs(20));
        s.released = s.hold_until;
        assert_eq!(s.tick(t0 + Duration::from_secs(1)), Some(false));
        assert!(!s.effective(t0 + Duration::from_secs(19)));
        // Back after the hold and the linger.
        assert!(!s.effective(t0 + Duration::from_secs(21)));
        assert!(s.effective(t0 + Duration::from_secs(20) + LINGER));
    }

    #[test]
    fn a_lease_turns_it_off_and_it_comes_back_after_the_linger() {
        let t0 = Instant::now();
        let mut s = State::new(Setting::On);
        s.tick(t0);
        s.observe(pano_seen(), 2, t0 + Duration::from_millis(100));
        s.leases += 1;
        assert!(s.maybe_not_normal());
        assert_eq!(s.tick(t0 + Duration::from_secs(1)), Some(false));
        assert!(s.wants_frame(t0 + Duration::from_secs(1)));
        s.observe(Seen::Normal, 5, t0 + Duration::from_millis(1100));
        // Released: still off for the linger, then on (an answer awaited).
        s.leases -= 1;
        let end = t0 + Duration::from_secs(2);
        s.released = Some(end);
        assert_eq!(s.tick(end + LINGER / 2), None);
        assert_eq!(s.tick(end + LINGER), Some(true));
        assert!(!s.answered && s.asked == Some(end + LINGER));
    }
}
