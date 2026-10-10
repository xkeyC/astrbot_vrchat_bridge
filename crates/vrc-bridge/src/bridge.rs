//! The bridge's shared state and its background work: the stream client,
//! audio both ways, the chatbox queue, the log tail and the watchdog.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::extract::ws::Message;
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::process::{Child, Command};
use tokio::sync::mpsc;
use vrc_vr::osc::{Arg, Osc};

use crate::anim::Anim;
use crate::follow::Follower;
use crate::game::{self, GameState, LogTail};
use crate::mapping::Mapping;
use crate::sightings::Sightings;
use crate::social::Social;
use crate::speaker::Speakers;
use crate::vr::VrCore;
use crate::Args;
use crate::Lock;

pub const SAMPLE_RATE: u32 = 48_000;
/// 20 ms: what goes to the client as 16-bit mono.
pub const FRAME_SAMPLES: usize = 960;
/// 20 ms of the capture: float32 stereo.
const CAPTURE_BYTES: usize = FRAME_SAMPLES * 2 * 4;
/// Push-to-talk stays held this long after the bot's audio has played out,
/// so the end of a sentence is not cut by the release.
const PTT_TAIL: Duration = Duration::from_millis(300);
/// Latency of the playback stream into the microphone sink.
const PLAYBACK_LATENCY_MS: u64 = 60;
/// Outbound frames queued for a slow client before new ones are dropped.
const CLIENT_QUEUE: usize = 200;
/// Blocks waiting for the speaker tracker's analysis (1 s) before new ones
/// go unanalysed (the client still gets them).
const ANALYSIS_QUEUE: usize = 50;
pub const CHATBOX_LIMIT: usize = 144;
/// VRChat throttles chatbox spam; messages are paced at least this far apart.
const CHATBOX_INTERVAL: Duration = Duration::from_millis(1600);
const WATCHDOG_INTERVAL: Duration = Duration::from_secs(5);
/// A follow the model's own moves paused resumes this long after its last
/// move (a step, a walk, a look around).
pub const TAKEOVER_IDLE: Duration = Duration::from_secs(10);

/// The stream client: its id, its outbound queue, and its sample clock (the
/// audio samples actually queued to it since it attached; a frame a full
/// queue dropped does not count: `docs/full-vr/speaker.md`).
struct Client {
    id: u64,
    tx: mpsc::Sender<Message>,
    clock: u64,
}

/// A block of the capture for the speaker tracker's analysis: where it
/// landed on the client's clock (and whether it reached the client), the
/// head and the bot's voice then, and when it was read.
struct Captured {
    left: Vec<f32>,
    right: Vec<f32>,
    clock: u64,
    counted: bool,
    client: u64,
    head: Option<(Instant, vrc_vr::Pose)>,
    bot_until: Option<Instant>,
    at: Instant,
}

/// A follow paused while the model moves the avatar itself.
#[derive(Clone, Debug)]
pub struct Takeover {
    pub target: String,
    /// Its standing distance (world metres).
    pub distance: Option<f32>,
}

pub struct Bridge {
    pub args: Args,
    pub token: String,
    pub osc: Osc,
    pub game: Mutex<GameState>,
    /// The stream client (one at a time).
    client: Mutex<Option<Client>>,
    client_ids: AtomicU64,
    player: tokio::sync::Mutex<Player>,
    chatbox: mpsc::Sender<(String, bool)>,
    /// One game stop/start at a time (joins, /v1/game/start).
    pub game_lock: tokio::sync::Mutex<()>,
    /// Leaving an instance the bot must not be in.
    leaving: AtomicBool,
    pub social: Arc<Social>,
    pub vr: Arc<Mutex<VrCore>>,
    pub follower: Arc<Follower>,
    pub sightings: Arc<Sightings>,
    /// The lasting map of the world the bot is in.
    pub mapping: Arc<Mapping>,
    pub anim: Arc<Anim>,
    /// The async runtime: tasks started from plain threads (the animation's,
    /// a calibration's) go to it.
    pub rt: tokio::runtime::Handle,
    /// Motion clips (`motions/` next to the token).
    pub motions: Arc<crate::motion::Library>,
    /// The follow the model's moves paused, given back TAKEOVER_IDLE after
    /// its last (take_over, idle_later).
    takeover: Mutex<Option<Takeover>>,
    /// Counts the model's moves: a resume scheduled before the latest is off.
    takeover_moves: AtomicU64,
    /// Who is speaking: direction and nameplates (`speaker`).
    pub speaker: Arc<Speakers>,
    /// When the bot's own voice has played out (what other players' open
    /// speakers may play back for a while after).
    pub bot_voice_until: Mutex<Option<Instant>>,
    /// VRChat's user camera as a remote eye (`usercam`).
    pub usercam: crate::usercam::UserCam,
    /// Its lens round the bot, and the names it reads (`orbit`).
    pub orbit: Arc<crate::orbit::Orbit>,
    /// The idle patrol: the head and the lens look different ways
    /// (`patrol`, decision D45).
    pub patrol: Arc<crate::patrol::Patrol>,
}

#[derive(Default)]
struct Player {
    child: Option<Child>,
    ptt_held: bool,
    /// When the bot's queued audio has played out.
    speech_until: Option<Instant>,
    release_pending: bool,
    /// The odd byte of a frame, ahead of the next one.
    odd_byte: Option<u8>,
}

impl Bridge {
    pub fn new(args: Args, token: String) -> (Arc<Bridge>, mpsc::Receiver<(String, bool)>) {
        let osc = Osc::with_ports(&format!("127.0.0.1:{}", args.osc_port), 0).expect("an OSC socket");
        let (chat_tx, chat_rx) = mpsc::channel(20);
        let config_dir = game::expand(&args.token_file).parent().map(|p| p.to_path_buf()).unwrap_or_default();
        let social = Arc::new(Social::new(config_dir.join("social.json"), config_dir.join("cookies.json")));
        let anim = Arc::new(Anim::new(config_dir.join("anim.json")));
        let motions = Arc::new(crate::motion::Library::new(config_dir.join("motions")));
        let mapping = Mapping::new(config_dir.join("maps"));
        let speaker = Arc::new(Speakers::new(&args));
        let orbit = Arc::new(crate::orbit::Orbit::new(Osc::with_ports_from(&osc).expect("an OSC socket"), anim.clone()));
        let mut core = VrCore::new(&args);
        core.head_height = anim.params().head_height;
        core.map = Some(mapping.nav.clone());
        let vr = Arc::new(Mutex::new(core));
        let bridge = Arc::new(Bridge {
            token,
            osc,
            game: Mutex::new(GameState::default()),
            client: Mutex::new(None),
            client_ids: AtomicU64::new(0),
            player: tokio::sync::Mutex::new(Player::default()),
            chatbox: chat_tx,
            game_lock: tokio::sync::Mutex::new(()),
            leaving: AtomicBool::new(false),
            social,
            vr,
            follower: Arc::new(Follower::default()),
            sightings: Arc::new(Sightings::default()),
            mapping,
            anim,
            rt: tokio::runtime::Handle::current(),
            motions,
            args,
            takeover: Mutex::new(None),
            takeover_moves: AtomicU64::new(0),
            speaker,
            bot_voice_until: Mutex::new(None),
            usercam: crate::usercam::UserCam::new(config_dir.join("usercam.json")),
            orbit,
            patrol: Arc::new(crate::patrol::Patrol::default()),
        });
        (bridge, chat_rx)
    }

    // -- events ---------------------------------------------------------------

    /// Sends a text event to the stream client, if any.
    pub fn send_event(&self, data: Value) {
        let follow = data["type"] == "follow";
        if let Some(c) = self.client.lk().as_ref() {
            let _ = c.tx.try_send(Message::Text(data.to_string().into()));
        }
        if follow {
            self.notify_state(); // whom it follows, and how, is room state
        }
    }

    /// The game state, each other player marked as a friend (VRChat's friend
    /// list) and by their whitelist rank (1 first, 0 not on it), and whom the
    /// avatar follows (null when nobody).
    pub fn room_state(&self) -> Value {
        let mut snap = self.game.lk().snapshot();
        let ranks = self.social.whitelist_ids();
        let friends = self.social.friend_ids();
        if let Some(players) = snap["players"].as_array_mut() {
            for p in players {
                let id = p["id"].as_str().unwrap_or("").to_string();
                p["friend"] = json!(friends.contains(&id));
                p["whitelist"] = json!(ranks.iter().position(|r| *r == id).map_or(0, |i| i + 1));
            }
        }
        let f = self.follower.status();
        snap["follow"] = if f["state"] == "idle" { Value::Null } else { json!({"target": f["target"], "state": f["state"]}) };
        snap["takeover"] = match self.takeover.lk().as_ref() {
            Some(t) => json!({"target": t.target, "distance": t.distance}),
            None => Value::Null,
        };
        snap
    }

    /// Pushes the room state (and guards the instance).
    pub fn notify_state(&self) {
        if let Some(c) = self.client.lk().as_ref() {
            let _ = c.tx.try_send(Message::Text(json!({"type": "state", "state": self.room_state()}).to_string().into()));
        }
    }

    // -- the model's moves pause following ------------------------------------

    /// The model moves the avatar: a follow is paused, resumed by itself
    /// once the model is done (idle_later). A move while one is paused
    /// keeps it paused.
    pub fn take_over(&self) {
        self.takeover_moves.fetch_add(1, Ordering::SeqCst);
        if self.follower.is_idle() {
            return;
        }
        let s = self.follower.status();
        if let Some(target) = s["target"].as_str() {
            *self.takeover.lk() = Some(Takeover {
                target: target.to_string(),
                distance: s["distance"].as_f64().map(|d| d as f32),
            });
            tracing::info!(target, "takeover: following paused while the model moves");
        }
        self.follower.stop();
        self.notify_state();
    }

    /// After a move: unless the model moves again within TAKEOVER_IDLE, the
    /// follow it paused resumes.
    pub fn idle_later(self: &Arc<Self>) {
        self.idle_later_for(TAKEOVER_IDLE);
    }

    /// [`Bridge::idle_later`] after `idle` instead (turning to a speaker
    /// pauses the follow for as long as asked).
    pub fn idle_later_for(self: &Arc<Self>, idle: Duration) {
        let moves = self.takeover_moves.fetch_add(1, Ordering::SeqCst) + 1;
        if self.takeover.lk().is_none() {
            return;
        }
        let me = self.clone();
        self.rt.spawn(async move {
            tokio::time::sleep(idle).await;
            if me.takeover_moves.load(Ordering::SeqCst) == moves {
                me.resume_following();
            }
        });
    }

    fn resume_following(self: &Arc<Self>) {
        let Some(t) = self.takeover.lk().take() else { return };
        if !self.follower.is_idle() || self.require_game().is_err() {
            self.notify_state();
            return;
        }
        tracing::info!(target = t.target, "takeover over: following again");
        self.follower.start(self, &t.target, t.distance);
        self.send_event(json!({"type": "follow", "state": "resumed", "target": t.target}));
    }

    /// Drops a paused follow without resuming it (a stop, a new follow).
    /// Whether a follow is paused while the model moves.
    pub fn follow_paused(&self) -> bool {
        self.takeover.lk().is_some()
    }

    pub fn end_takeover(&self) {
        self.takeover_moves.fetch_add(1, Ordering::SeqCst);
        if self.takeover.lk().take().is_some() {
            self.notify_state();
        }
    }

    pub fn state_changed(self: &Arc<Self>) {
        self.guard_instance();
        self.notify_state();
    }

    /// Leaves an instance the bot must not be in (public, group, ...): only
    /// joins it starts itself are checked, yet a portal or a followed player
    /// can take it anywhere.
    fn guard_instance(self: &Arc<Self>) {
        let (running, instance) = {
            let g = self.game.lk();
            (g.running, g.instance.clone())
        };
        if !running || instance.is_empty() || crate::api::joinable(&instance) || self.leaving.swap(true, Ordering::SeqCst) {
            return;
        }
        let kind = crate::api::instance_kind(&instance).unwrap_or("unknown");
        tracing::error!("in a {kind} instance ({instance}): leaving");
        self.follower.stop();
        self.send_event(json!({"type": "alert", "reason": "not_joinable_instance", "kind": kind}));
        let me = self.clone();
        self.rt.spawn(async move {
            let _ = game::stop_game().await;
            me.leaving.store(false, Ordering::SeqCst);
        });
    }

    pub fn require_game(&self) -> anyhow::Result<()> {
        if !self.game.lk().running {
            anyhow::bail!("VRChat is not running");
        }
        Ok(())
    }

    /// An OSC handle that can also query OSCQuery (on the port the log names).
    pub fn osc_query(&self) -> anyhow::Result<Osc> {
        let port = self.game.lk().oscquery_port;
        if port == 0 {
            anyhow::bail!("the game is not running");
        }
        Osc::with_ports(&format!("127.0.0.1:{}", self.args.osc_port), port)
    }

    // -- the stream client ------------------------------------------------------

    /// A new stream client: the old one is dropped; its outbound queue. Its
    /// sample clock starts at 0.
    pub fn attach_client(&self) -> (u64, mpsc::Receiver<Message>) {
        let id = self.client_ids.fetch_add(1, Ordering::SeqCst) + 1;
        let (tx, rx) = mpsc::channel(CLIENT_QUEUE);
        let _ = tx.try_send(Message::Text(json!({"type": "state", "state": self.room_state()}).to_string().into()));
        if let Some(old) = self.client.lk().replace(Client { id, tx, clock: 0 }) {
            let _ = old.tx.try_send(Message::Close(None));
        }
        (id, rx)
    }

    pub fn detach_client(&self, id: u64) {
        let mut c = self.client.lk();
        if c.as_ref().is_some_and(|c| c.id == id) {
            *c = None;
        }
    }

    /// Queues a text event to client `id` if it is still the one attached;
    /// whether it was queued.
    fn send_to(&self, id: u64, data: &Value) -> bool {
        match self.client.lk().as_ref() {
            Some(c) if c.id == id => c.tx.try_send(Message::Text(data.to_string().into())).is_ok(),
            _ => false,
        }
    }

    // -- audio ------------------------------------------------------------------

    /// Reads what the game plays (both ears) and hands the client a mono mix
    /// of it, if there is a client; the speaker tracker hears both ears (on
    /// a thread of its own) and sends its events in the same queue, after
    /// the audio they are about.
    pub async fn capture(self: Arc<Self>) {
        let analysis = self.clone().analysis();
        let (mut unanalysed, mut warned) = (0u64, None::<Instant>);
        loop {
            let child = Command::new("parec")
                .args([
                    &format!("--device={}", self.args.capture_device),
                    "--format=float32le",
                    &format!("--rate={SAMPLE_RATE}"),
                    "--channels=2",
                    "--channel-map=front-left,front-right",
                    "--latency-msec=20",
                    "--raw",
                ])
                .stdout(std::process::Stdio::piped())
                .kill_on_drop(true)
                .spawn();
            match child {
                Ok(mut child) => {
                    tracing::info!("capturing {} (stereo)", self.args.capture_device);
                    let mut out = child.stdout.take().unwrap();
                    let mut buf = vec![0u8; CAPTURE_BYTES];
                    let (mut left, mut right) = (vec![0f32; FRAME_SAMPLES], vec![0f32; FRAME_SAMPLES]);
                    while out.read_exact(&mut buf).await.is_ok() {
                        for (i, f) in buf.chunks_exact(8).enumerate() {
                            let (l, r) = (f32::from_le_bytes(f[..4].try_into().unwrap()), f32::from_le_bytes(f[4..].try_into().unwrap()));
                            (left[i], right[i]) = if self.args.swap_ears { (r, l) } else { (l, r) };
                        }
                        let mono = self.speaker.mix(&left, &right);
                        let bytes: Vec<u8> = mono.iter().flat_map(|s| s.to_le_bytes()).collect();
                        // Counted only if queued (a full queue drops it).
                        let (id, clock, counted) = match self.client.lk().as_mut() {
                            Some(c) => {
                                let before = c.clock;
                                let queued = c.tx.try_send(Message::Binary(bytes.into())).is_ok();
                                if queued {
                                    c.clock += FRAME_SAMPLES as u64;
                                }
                                (c.id, before, queued)
                            }
                            None => (0, 0, false),
                        };
                        let block = Captured {
                            left: left.clone(),
                            right: right.clone(),
                            clock,
                            counted,
                            client: id,
                            head: *self.anim.head.lk(),
                            bot_until: *self.bot_voice_until.lk(),
                            at: Instant::now(),
                        };
                        if analysis.try_send(block).is_err() {
                            unanalysed += 1;
                            if warned.is_none_or(|t| t.elapsed() > Duration::from_secs(10)) {
                                tracing::warn!("speakers: the analysis falls behind ({unanalysed} blocks left out so far)");
                                warned = Some(Instant::now());
                            }
                        }
                    }
                    let _ = child.kill().await;
                }
                Err(e) => tracing::warn!("capture failed: {e}"),
            }
            tracing::warn!("capture ended, restarting");
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }

    /// The speaker tracker's analysis, on a thread of its own: the blocks in
    /// order, each with its own clock and time; its `speaker` events into
    /// the client's queue (finals a full queue refused tried again with the
    /// next blocks, for the client they were for, a while).
    fn analysis(self: Arc<Self>) -> std::sync::mpsc::SyncSender<Captured> {
        let (tx, rx) = std::sync::mpsc::sync_channel::<Captured>(ANALYSIS_QUEUE);
        std::thread::Builder::new()
            .name("speakers-audio".into())
            .spawn(move || {
                let mut retry: Vec<(u64, Value, u32)> = Vec::new();
                for c in rx {
                    let events = self.speaker.analyse(&c.left, &c.right, c.clock, c.counted, c.client, c.head, c.bot_until, c.at);
                    retry.retain_mut(|(to, ev, tries)| {
                        *tries += 1;
                        *to == c.client && *tries < 50 && !self.send_to(*to, ev)
                    });
                    for ev in events {
                        if !self.send_to(c.client, &ev) && ev["final"] == true {
                            retry.push((c.client, ev, 0));
                        }
                    }
                }
            })
            .expect("a thread for the speaker tracker");
        tx
    }

    /// Plays the bot's voice into the microphone, holding push-to-talk.
    pub async fn play(self: &Arc<Self>, pcm: &[u8]) {
        let mut p = self.player.lock().await;
        // Whole samples only: an odd byte waits for the next frame.
        let mut joined = Vec::with_capacity(pcm.len() + 1);
        joined.extend(p.odd_byte.take());
        joined.extend_from_slice(pcm);
        if joined.len() % 2 == 1 {
            p.odd_byte = joined.pop();
        }
        let pcm = joined.as_slice();
        let dead = match &mut p.child {
            Some(c) => c.try_wait().map(|s| s.is_some()).unwrap_or(true),
            None => true,
        };
        if dead {
            p.child = Command::new("pacat")
                .args([
                    "--playback",
                    &format!("--device={}", self.args.playback_device),
                    "--format=s16le",
                    &format!("--rate={SAMPLE_RATE}"),
                    "--channels=1",
                    &format!("--latency-msec={PLAYBACK_LATENCY_MS}"),
                    "--raw",
                ])
                .stdin(std::process::Stdio::piped())
                .kill_on_drop(true)
                .spawn()
                .map_err(|e| tracing::warn!("playback failed: {e}"))
                .ok();
        }
        if !p.ptt_held {
            let _ = self.osc.send_i32("/input/Voice", 1);
            p.ptt_held = true;
            tracing::info!("timing: the bot speaks (push-to-talk on)");
        }
        let now = Instant::now();
        let len = Duration::from_secs_f64(pcm.len() as f64 / (SAMPLE_RATE as f64 * 2.0));
        let starts = p.speech_until.filter(|t| *t > now).unwrap_or(now);
        self.anim.heard_bot(pcm, starts + Duration::from_millis(PLAYBACK_LATENCY_MS));
        p.speech_until = Some(p.speech_until.filter(|t| *t > now).unwrap_or(now) + len);
        *self.bot_voice_until.lk() = p.speech_until.map(|t| t + Duration::from_millis(PLAYBACK_LATENCY_MS));
        if !p.release_pending {
            p.release_pending = true;
            let me = self.clone();
            tokio::spawn(async move { me.release_ptt().await });
        }
        if let Some(stdin) = p.child.as_mut().and_then(|c| c.stdin.as_mut()) {
            if stdin.write_all(pcm).await.is_err() {
                p.child = None;
            }
        }
    }

    async fn release_ptt(self: Arc<Self>) {
        let tail = PTT_TAIL + Duration::from_millis(PLAYBACK_LATENCY_MS);
        // Checked again under the lock it lets go under: more speech may
        // have come while it waited for it.
        let mut p = loop {
            let p = self.player.lock().await;
            let wait = p.speech_until.map(|u| (u + tail).saturating_duration_since(Instant::now())).unwrap_or_default();
            if wait.is_zero() {
                break p;
            }
            drop(p);
            tokio::time::sleep(wait).await;
        };
        let _ = self.osc.send_i32("/input/Voice", 0);
        p.ptt_held = false;
        p.release_pending = false;
        tracing::info!("timing: the bot stops speaking (push-to-talk off)");
    }

    /// Lets go of every input VRChat keeps the last value of (the stick,
    /// jump, push-to-talk): at start, and on the way out.
    pub fn let_go(&self) {
        for axis in ["/input/Vertical", "/input/Horizontal"] {
            let _ = self.osc.send_f32(axis, 0.0);
        }
        let _ = self.osc.send_i32("/input/Jump", 0);
        let _ = self.osc.send_i32("/input/Voice", 0);
    }

    pub async fn release_voice(&self) {
        let mut p = self.player.lock().await;
        if p.ptt_held {
            let _ = self.osc.send_i32("/input/Voice", 0);
            p.ptt_held = false;
        }
    }

    // -- chatbox ------------------------------------------------------------------

    /// Queues `text` for the chatbox, split into 144-character parts.
    pub fn chatbox(&self, text: &str, notify: bool) -> usize {
        let parts = split_chatbox(text);
        for part in &parts {
            let _ = self.chatbox.try_send((part.clone(), notify)); // a full queue drops it
        }
        parts.len()
    }

    pub async fn chatbox_sender(self: Arc<Self>, mut rx: mpsc::Receiver<(String, bool)>) {
        while let Some((text, notify)) = rx.recv().await {
            let _ = self.osc.send("/chatbox/input", &[Arg::Str(text), Arg::Bool(true), Arg::Bool(notify)]);
            tokio::time::sleep(CHATBOX_INTERVAL).await;
        }
    }

    // -- the game -----------------------------------------------------------------

    /// Follows the client's output log.
    pub async fn log_tail(self: Arc<Self>) {
        let mut tail = LogTail::new(game::expand(&self.args.log_dir));
        loop {
            let changed = {
                let mut g = self.game.lk();
                tail.poll(&mut g).unwrap_or_else(|e| {
                    tracing::warn!("log tail failed: {e}");
                    false
                })
            };
            if changed {
                self.state_changed();
            }
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }

    /// Tracks whether the game runs and its VRAM; stops it above the hard limit.
    pub async fn watchdog(self: Arc<Self>) {
        let mut first = true;
        loop {
            if !first {
                tokio::time::sleep(WATCHDOG_INTERVAL).await;
            }
            first = false;
            let pid = game::game_pid().await;
            let running = pid.is_some();
            let (vram, total) = match pid {
                Some(pid) => game::vram(pid).await,
                None => (0, 0),
            };
            let changed = {
                let mut g = self.game.lk();
                let mut changed = running != g.running;
                g.running = running;
                g.vram_mib = vram;
                if !running && (!g.instance.is_empty() || !g.players.is_empty() || !g.world_name.is_empty()) {
                    // No game, no instance: not even one an old log replayed.
                    g.world_name.clear();
                    g.instance.clear();
                    g.players.clear();
                    changed = true;
                }
                changed
            };
            if changed && !running {
                self.follower.stop(); // nothing to read or steer
            }
            if changed {
                self.state_changed();
            }
            let a = &self.args;
            if running && (vram > a.vram_hard_mib || total > a.gpu_hard_mib) {
                tracing::error!("VRAM over the limit (game {vram} MiB, GPU {total} MiB): stopping the game");
                self.send_event(json!({"type": "alert", "reason": "vram", "game_mib": vram, "gpu_mib": total, "action": "stopped"}));
                let _ = game::stop_game().await;
            } else if running && vram > a.vram_soft_mib {
                self.send_event(json!({"type": "alert", "reason": "vram", "game_mib": vram, "gpu_mib": total, "action": "none"}));
            }
        }
    }
}

/// Splits `text` into chatbox messages of at most 144 characters, preferring
/// sentence ends, then spaces.
pub fn split_chatbox(text: &str) -> Vec<String> {
    let mut text: Vec<char> = text.split_whitespace().collect::<Vec<_>>().join(" ").chars().collect();
    let mut parts = Vec::new();
    while text.len() > CHATBOX_LIMIT {
        let window = &text[..CHATBOX_LIMIT];
        let cut = window.iter().rposition(|c| "。！？!?；;，,. ".contains(*c));
        let cut = match cut {
            Some(c) if c >= CHATBOX_LIMIT / 2 => c + 1,
            _ => CHATBOX_LIMIT,
        };
        parts.push(text[..cut].iter().collect::<String>().trim().to_string());
        text = text[cut..].iter().collect::<String>().trim().chars().collect();
    }
    if !text.is_empty() {
        parts.push(text.into_iter().collect());
    }
    parts
}

/// A bridge for tests: no game, no headset, its files in a temporary
/// directory.
#[cfg(test)]
pub(crate) fn test_bridge() -> Arc<Bridge> {
    use clap::Parser;
    let dir = std::env::temp_dir().join(format!("vrc-bridge-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let token = dir.join("token");
    let args = Args::parse_from(["vrc-bridge", "--token-file", token.to_str().unwrap()]);
    Bridge::new(args, "t".to_string()).0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bridge() -> Arc<Bridge> {
        test_bridge()
    }

    fn paused(b: &Bridge) {
        *b.takeover.lk() = Some(Takeover { target: "xkeyC".to_string(), distance: Some(1.5) });
    }

    #[tokio::test(start_paused = true)]
    async fn a_paused_follow_resumes_after_the_models_last_move() {
        let b = bridge();
        paused(&b);
        b.idle_later();
        tokio::time::sleep(TAKEOVER_IDLE / 2).await;
        b.take_over(); // another move: the first resume is off
        b.idle_later();
        tokio::time::sleep(TAKEOVER_IDLE - Duration::from_secs(1)).await;
        assert!(b.takeover.lk().is_some());
        assert_eq!(b.room_state()["takeover"], json!({"target": "xkeyC", "distance": 1.5}));
        tokio::time::sleep(Duration::from_secs(2)).await;
        // Handed back (with no game running, not followed).
        assert!(b.takeover.lk().is_none());
        assert!(b.follower.is_idle());
    }

    #[tokio::test(start_paused = true)]
    async fn a_stop_drops_a_paused_follow() {
        let b = bridge();
        paused(&b);
        b.idle_later();
        b.end_takeover();
        assert!(b.takeover.lk().is_none());
        assert_eq!(b.room_state()["takeover"], Value::Null);
        // A move after it pauses nothing: nobody is followed.
        b.take_over();
        b.idle_later();
        tokio::time::sleep(TAKEOVER_IDLE * 2).await;
        assert!(b.takeover.lk().is_none());
    }

    #[test]
    fn chatbox_parts() {
        assert_eq!(split_chatbox("  hello   world "), vec!["hello world".to_string()]);
        let long = "这是一句话。".repeat(40);
        let parts = split_chatbox(&long);
        assert!(parts.iter().all(|p| p.chars().count() <= CHATBOX_LIMIT));
        assert!(parts[0].ends_with('。'));
        assert_eq!(parts.concat(), long);
    }
}
