//! The avatar's procedural motion (`vrc_vr::anim`), run 45 times a second
//! on the headset connection's animator handle: the owner (surveys, walks,
//! the follower) keeps the head and the body's facing; this adds the hands'
//! and the head's motion on top, and stands aside while a scan holds still.
//!
//! Inputs: the avatar's speed (OSCQuery `VelocityZ`), the loudness of the
//! bot's own voice as it plays (`heard_bot`), and whether anything else is
//! driving the head (glances only when not). Parameters are tunable at run
//! time (`/v1/anim`) and kept in `anim.json` next to the token.
//!
//! The same tick sends VRChat's OSC trackers when they are on
//! (`/v1/vr/trackers`, trying out full body: a body standing under the
//! head), and stands aside while the hands are set by hand (`/v1/vr/hand`);
//! every few seconds it checks whether VRChat lost them, to calibrate again
//! (`crate::calibrate::Watch`).

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;
use vrc_vr::anim::{AnimInput, AnimParams, Animator};
use vrc_vr::osc::Osc;
use vrc_vr::remote::{HmdLink, FLOOR_Y};
use vrc_vr::trackers::{self, Part};

use crate::bridge::{Bridge, SAMPLE_RATE};
use crate::Lock;

const TICK: Duration = Duration::from_millis(22);
/// A headset not there is tried again this often.
const RECONNECT_EVERY: Duration = Duration::from_secs(5);
/// The speed is read every few ticks (an HTTP request each).
const SPEED_EVERY: u32 = 3;
/// Whether the body needs a calibration is checked every few seconds.
const CALIBRATION_EVERY: u32 = 230;
/// Loudness is measured over windows this long.
const WINDOW: Duration = Duration::from_millis(30);

/// The OSC trackers (`/v1/vr/trackers`).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TrackerSettings {
    pub on: bool,
    /// `hip`, `feet`, `chest`, `knees`.
    pub parts: Vec<String>,
    /// The head's alignment: `once` (its rotation once, then its position
    /// each tick), `always` (both each tick), `off`.
    pub head: String,
    /// Moves trackers off the standing pose (metres: right, up, ahead), by
    /// `hip`, `left_foot`, ...: trying out that the body follows.
    pub shift: std::collections::BTreeMap<String, [f32; 3]>,
    /// Multiplies the trackers' positions (None: 1). VRChat takes them in
    /// the tracking space's metres, as the headset's (checked against the
    /// calibration mirror: at 1 the markers sit in the avatar's hips and
    /// ankles, at 0.8 the feet ride on the shins); for trying others.
    pub scale: Option<f32>,
    /// Calibrate by itself when VRChat tracks head and hands only though
    /// the trackers are on (after the game started), while nothing else
    /// moves the bot (`crate::calibrate::watch`).
    pub auto_calibrate: bool,
}

impl Default for TrackerSettings {
    fn default() -> Self {
        TrackerSettings { on: false, parts: vec!["hip".into(), "feet".into()], head: "once".into(), shift: Default::default(), scale: None, auto_calibrate: true }
    }
}

impl TrackerSettings {
    pub fn parts(&self) -> anyhow::Result<Vec<Part>> {
        let mut out = Vec::new();
        for name in &self.parts {
            let parts = Part::named(name).ok_or_else(|| anyhow::anyhow!("no tracker part {name} (hip, feet, chest, knees)"))?;
            out.extend(parts.iter().filter(|p| !out.contains(*p)).copied().collect::<Vec<_>>());
        }
        anyhow::ensure!(["once", "always", "off"].contains(&self.head.as_str()), "head is once, always or off");
        for (key, v) in &self.shift {
            anyhow::ensure!(Part::ALL.iter().any(|p| p.key() == key), "no tracker {key} to shift");
            anyhow::ensure!(v.iter().all(|x| x.is_finite() && x.abs() <= 1.0), "shifts within 1 m");
        }
        if let Some(k) = self.scale {
            anyhow::ensure!(k.is_finite() && (0.1..=3.0).contains(&k), "scale is 0.1-3");
        }
        Ok(out)
    }
}

pub struct Anim {
    params: Mutex<AnimParams>,
    path: PathBuf,
    /// The bot's voice: (when it plays, loudness 0..1) per window.
    voice: Mutex<VecDeque<(Instant, f32)>>,
    /// What the last tick saw (speed, talking, holding still), for `/v1/anim`.
    pub live: Mutex<Value>,
    pub trackers: Mutex<TrackerSettings>,
    /// `trackers.json` next to `anim.json`: the trackers stay as set across
    /// restarts (VRChat keeps its calibration while the game runs).
    trackers_path: PathBuf,
    /// Since when the trackers have been sent without a break.
    pub trackers_on_since: Mutex<Option<Instant>>,
    /// The head's rotation is to be sent (once: on the next tick).
    pub head_rotation_due: AtomicBool,
    /// The hands are set by hand: no overlay.
    pub manual_hands: AtomicBool,
    /// How far the head is leaned off where the body stands (tracking
    /// space, `/v1/vr/head`): the trackers stay under the body.
    pub head_lean: Mutex<[f32; 3]>,
}

/// What the trackers send: each tracker, the head, and the scale applied.
pub struct TrackerFrame {
    pub trackers: Vec<(Part, vrc_vr::Pose)>,
    pub head: vrc_vr::Pose,
    pub scale: f32,
}

impl Anim {
    pub fn new(path: PathBuf) -> Anim {
        let params = std::fs::read(&path)
            .ok()
            .and_then(|b| serde_json::from_slice::<AnimParams>(&b).map_err(|e| tracing::warn!("{}: {e}", path.display())).ok())
            .and_then(|p| p.check().map(|()| p).map_err(|e| tracing::warn!("{}: {e}", path.display())).ok())
            .unwrap_or_default();
        let trackers_path = path.with_file_name("trackers.json");
        let trackers = std::fs::read(&trackers_path)
            .ok()
            .and_then(|b| serde_json::from_slice::<TrackerSettings>(&b).map_err(|e| tracing::warn!("{}: {e}", trackers_path.display())).ok())
            .filter(|t| t.parts().map_err(|e| tracing::warn!("{}: {e}", trackers_path.display())).is_ok())
            .unwrap_or_default();
        Anim {
            params: Mutex::new(params),
            path,
            trackers_path,
            voice: Mutex::new(VecDeque::new()),
            live: Mutex::new(Value::Null),
            trackers: Mutex::new(trackers),
            trackers_on_since: Mutex::new(None),
            head_rotation_due: AtomicBool::new(true),
            manual_hands: AtomicBool::new(false),
            head_lean: Mutex::new([0.0; 3]),
        }
    }

    pub fn params(&self) -> AnimParams {
        self.params.lk().clone()
    }

    /// Merges `patch` into the parameters (and keeps them).
    pub fn tune(&self, patch: &Value) -> anyhow::Result<AnimParams> {
        let mut p = self.params.lk();
        let mut v = serde_json::to_value(&*p)?;
        let (Some(dst), Some(src)) = (v.as_object_mut(), patch.as_object()) else {
            anyhow::bail!("expected a JSON object of parameters");
        };
        if patch.get("reset").and_then(Value::as_bool) == Some(true) {
            *p = AnimParams::default();
        } else {
            for (k, val) in src {
                if !dst.contains_key(k) {
                    anyhow::bail!("no parameter {k}");
                }
                dst.insert(k.clone(), val.clone());
            }
            let tuned: AnimParams = serde_json::from_value(v)?;
            tuned.check().map_err(anyhow::Error::msg)?;
            *p = tuned;
        }
        std::fs::write(&self.path, serde_json::to_vec_pretty(&*p)?)?;
        Ok(p.clone())
    }

    /// The bot's voice `pcm` (s16le mono), playing from `at`.
    pub fn heard_bot(&self, pcm: &[u8], at: Instant) {
        let per_window = (SAMPLE_RATE as f64 * WINDOW.as_secs_f64()) as usize * 2;
        let mut voice = self.voice.lk();
        for (i, chunk) in pcm.chunks(per_window).enumerate() {
            let n = chunk.len() / 2;
            if n == 0 {
                continue;
            }
            let sum: f64 = chunk.chunks_exact(2).map(|s| (i16::from_le_bytes([s[0], s[1]]) as f64).powi(2)).sum();
            let rms = (sum / n as f64).sqrt() / 32768.0;
            // Speech runs 0.05-0.2 RMS: about 0.2-0.8.
            voice.push_back((at + WINDOW * i as u32, (rms * 4.0).min(1.0) as f32));
        }
        // Long queues of speech are fine; a stale tail is not.
        while voice.len() > 4000 {
            voice.pop_front();
        }
    }

    /// The bot's voice now: `None` while silent.
    fn voice_now(&self, now: Instant) -> Option<f32> {
        let mut voice = self.voice.lk();
        while voice.len() > 1 && voice[1].0 <= now {
            voice.pop_front();
        }
        match voice.front() {
            Some(&(at, level)) if at <= now && now < at + WINDOW => Some(level),
            Some(&(at, _)) if at + WINDOW <= now => {
                voice.pop_front();
                None
            }
            _ => None,
        }
    }

    /// The trackers of a body standing under the owner's head (where it
    /// stands when the head leans).
    pub fn tracker_frame(&self, settings: &TrackerSettings, state: &vrc_vr::remote::State) -> anyhow::Result<TrackerFrame> {
        let parts = settings.parts()?;
        let lean = *self.head_lean.lk();
        let standing_head = [state.head.position[0] - lean[0], state.head.position[1] - lean[1], state.head.position[2] - lean[2]];
        let mut poses = trackers::standing(&parts, standing_head, state.body_yaw, FLOOR_Y);
        trackers::shifted(&mut poses, state.body_yaw, |p| settings.shift.get(p.key()).copied());
        let scale = settings.scale.unwrap_or(1.0);
        let mut head = state.head;
        head.position = head.position.map(|x| x * scale);
        for (_, pose) in &mut poses {
            pose.position = pose.position.map(|x| x * scale);
        }
        Ok(TrackerFrame { trackers: poses, head, scale })
    }

    /// Sets the trackers (and keeps them).
    pub fn set_trackers(&self, settings: TrackerSettings) -> anyhow::Result<()> {
        settings.parts()?;
        std::fs::write(&self.trackers_path, serde_json::to_vec_pretty(&settings)?)?;
        *self.trackers.lk() = settings;
        self.head_rotation_due.store(true, Ordering::Relaxed);
        Ok(())
    }

    /// How long the trackers have been sent without a break.
    pub fn trackers_on_for(&self) -> Duration {
        self.trackers_on_since.lk().map(|t| t.elapsed()).unwrap_or_default()
    }

    /// Sends the trackers.
    fn send_trackers(&self, bridge: &Bridge, state: &vrc_vr::remote::State) {
        let settings = self.trackers.lk().clone();
        if !settings.on {
            *self.trackers_on_since.lk() = None;
            return;
        }
        self.trackers_on_since.lk().get_or_insert_with(Instant::now);
        let frame = match self.tracker_frame(&settings, state) {
            Ok(f) => f,
            Err(e) => {
                tracing::debug!("trackers: {e:#}");
                return;
            }
        };
        let rotation = match settings.head.as_str() {
            "always" => true,
            "once" => self.head_rotation_due.swap(false, Ordering::Relaxed),
            _ => false,
        };
        let head = (settings.head != "off").then_some(&frame.head);
        for m in trackers::messages(&frame.trackers, head, rotation) {
            if let Err(e) = bridge.osc.send_raw(&m) {
                tracing::debug!("trackers: {e:#}");
                break;
            }
        }
    }

    pub fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(1);
        let mut animator = Animator::new(self.params(), seed);
        let mut watch = crate::calibrate::Watch::new();
        let mut link: Option<HmdLink> = None;
        let mut osc: Option<Osc> = None;
        let mut speed = 0.0f32;
        let mut tick = 0u32;
        let mut last = Instant::now();
        let mut off_sent = false;
        let mut tried = Instant::now().checked_sub(RECONNECT_EVERY).unwrap_or_else(Instant::now);
        loop {
            std::thread::sleep(TICK);
            tick = tick.wrapping_add(1);
            let now = Instant::now();
            let dt = (now - last).as_secs_f32();
            last = now;
            // The headset connection: the rig's; none yet (nothing used the
            // headset since the bridge started) or a dead one (Monado
            // restarted): the animation connects it, or the avatar stands
            // stiff until something else does.
            let owner = link.as_ref().and_then(HmdLink::owner);
            let Some(owner) = owner else {
                off_sent = false;
                if let Some(mut vr) = bridge.vr.try_lk() {
                    let dead = vr.link().as_ref().and_then(HmdLink::owner).is_none();
                    if dead && tried.elapsed() >= RECONNECT_EVERY {
                        tried = Instant::now();
                        vr.reset();
                        match vr.rig(&[]) {
                            Ok(_) => tracing::info!("animation: headset connected"),
                            Err(e) => tracing::debug!("animation: no headset yet: {e:#}"),
                        }
                    }
                    link = vr.link();
                }
                continue;
            };
            if tick.is_multiple_of(SPEED_EVERY) {
                if osc.is_none() {
                    osc = bridge.osc_query().ok();
                }
                match osc.as_ref().map(|o| o.query("/avatar/parameters/VelocityZ")) {
                    Some(Ok(v)) => speed = v as f32,
                    _ => {
                        osc = None;
                        speed = 0.0;
                    }
                }
            }
            animator.params = self.params();
            let enabled = animator.params.enabled;
            // Glances only when nobody else is using the head.
            let idle = bridge.follower.is_idle() && bridge.vr.try_lk().is_some();
            let voice = self.voice_now(now);
            let overlay = animator.update(&AnimInput { dt, owner: owner.state, speed, voice, idle });
            if tick.is_multiple_of(SPEED_EVERY) {
                *self.live.lk() = serde_json::json!({"speed": speed, "talking": voice.is_some(), "idle": idle, "still": owner.still});
            }
            self.send_trackers(&bridge, &owner.state);
            if tick.is_multiple_of(CALIBRATION_EVERY) {
                watch.check(&bridge);
            }
            let link = link.as_ref().unwrap();
            let sent = if enabled && !self.manual_hands.load(Ordering::Relaxed) {
                off_sent = false;
                link.set_overlay(Some(overlay))
            } else if !off_sent {
                off_sent = true;
                link.set_overlay(None)
            } else {
                Ok(())
            };
            if let Err(e) = sent {
                tracing::debug!("animation: {e:#}");
            }
        }
    }
}
