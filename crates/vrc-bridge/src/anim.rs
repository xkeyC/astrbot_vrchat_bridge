//! The avatar's procedural motion (`vrc_vr::anim`), run 45 times a second
//! on the headset connection's animator handle: the owner (surveys, walks,
//! the follower) keeps the head and the body's facing; this adds the hands'
//! and the head's motion on top, and stands aside while a scan holds still.
//!
//! Inputs: the avatar's speed over the ground (OSCQuery `VelocityZ` and
//! `VelocityX`: a step aside walks too, `ground_speed`), the loudness of the
//! bot's own voice as it plays (`heard_bot`), and whether anything else is
//! driving the head (glances only when not). Parameters are tunable at run
//! time (`/v1/anim`) and kept in `anim.json` next to the token.
//!
//! The same tick sends VRChat's OSC trackers when they are on
//! (`/v1/vr/trackers`, trying out full body: a body standing under the
//! head), and stands aside while the hands are set by hand (`/v1/vr/hand`);
//! every few seconds it checks whether VRChat lost them, to calibrate again
//! (`crate::calibrate::Watch`), and whether the user camera closed though
//! it should stay open (`crate::usercam::Watch`).

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
/// The avatar's size (eye height) is read this often (ticks).
const SIZE_EVERY: u32 = 90;
/// Moving faster than this (world m/s), the legs walk; from RUN_FROM they
/// run (fully by RUN_FULL).
const GAIT_FROM: f32 = 0.25;
const RUN_FROM: f32 = 1.1;
const RUN_FULL: f32 = 1.4;
/// How much of the gait cycles' motion plays (strides, knee lift, the hips'
/// bob): a little tamer than the capture; the hips' own sway and twist much
/// tamer (the headset does not sway with them).
const GAIT_AMPLITUDE: f32 = 1.0;
const HIPS_AMPLITUDE: f32 = 0.45;
/// Seconds for the legs to start and stop walking, and to change pace.
const GAIT_FADE: f32 = 0.35;
/// Seconds for the legs to tuck up off the ground (`jump_air`), and to come
/// down again on landing.
const AIR_IN: f32 = 0.12;
const AIR_OUT: f32 = 0.25;

/// The legs walking or running in place while the bot moves (the stick
/// moves it): gait cycles (`walk_cycle`, `run_cycle`) at a phase that
/// advances with the avatar's speed, so a foot on the ground stays put.
#[derive(Default)]
struct Gait {
    /// Into the cycle (0..1, from a left heel strike).
    phase: f32,
    /// How much the legs walk (0: standing).
    weight: f32,
    /// How much of that is running.
    run: f32,
    /// How much the legs are tucked up in the air.
    air: f32,
}

impl Gait {
    /// The legs' pose now (walking, running, in the air), or None while
    /// standing still on the ground.
    fn update(&mut self, motions: &crate::motion::Library, speed: f32, stature: f32, dt: f32, airborne: bool) -> Option<vrc_vr::motion::Body> {
        let walk = motions.get("walk_cycle")?;
        let run = motions.get("run_cycle")?;
        let (ws, rs) = (walk.speed?, run.speed?);
        let pace = speed.abs();
        let step = (dt / GAIT_FADE).min(1.0);
        let towards = |x: f32, to: f32| x + (to - x) * step;
        self.weight = towards(self.weight, if pace > GAIT_FROM && !airborne { 1.0 } else { 0.0 });
        let run_to = ((pace - RUN_FROM) / (RUN_FULL - RUN_FROM)).clamp(0.0, 1.0);
        self.run = towards(self.run, run_to);
        self.air = if airborne { (self.air + dt / AIR_IN).min(1.0) } else { (self.air - dt / AIR_OUT).max(0.0) };
        let air = motions.get("jump_air");
        let tuck = |body: vrc_vr::motion::Body| match &air {
            Some(a) if self.air > 0.0 => vrc_vr::motion::blend(&body, &a.frames[0], ease(self.air)),
            _ => body,
        };
        if self.weight < 0.01 {
            self.weight = 0.0;
            self.phase = 0.0;
            if self.air > 0.0 {
                return Some(tuck(walk.standing));
            }
            return None;
        }
        // A cycle covers its speed times its length (statures): advance by
        // the floor covered.
        let cycle = |c: &vrc_vr::motion::Clip, s: f32| s * c.duration() * stature;
        let dist = (cycle(&walk, ws) * (1.0 - self.run) + cycle(&run, rs) * self.run) * GAIT_AMPLITUDE;
        if dist > 1e-3 {
            self.phase = (self.phase + speed * dt / dist).rem_euclid(1.0);
        }
        let (walk, run) = (walk.in_place(), run.in_place());
        let w = walk.sample(self.phase * walk.duration());
        let r = run.sample(self.phase * run.duration());
        let legs = vrc_vr::motion::blend(&w, &r, self.run);
        let mut body = vrc_vr::motion::blend(&walk.standing, &legs, self.weight * GAIT_AMPLITUDE);
        body[0] = vrc_vr::motion::blend(&walk.standing, &legs, self.weight * HIPS_AMPLITUDE)[0];
        Some(tuck(body))
    }
}

/// A foot steps to where the body would stand it once it is this far off
/// (degrees of turn, or metres), one foot at a time.
const STEP_TURN_DEG: f32 = 15.0;
const STEP_OFF_M: f32 = 0.06;
/// A step: how long, how high the foot lifts (metres) and how far it turns
/// at most (degrees; a bigger turn takes more steps).
const STEP_S: f32 = 0.2;
const STEP_LIFT_M: f32 = 0.05;
const STEP_MAX_DEG: f32 = 60.0;
/// Far behind the body (more than FAR_DEG to turn yet), the steps are
/// quicker and bigger.
const FAR_DEG: f32 = 90.0;
const STEP_FAR_S: f32 = 0.15;
const STEP_FAR_MAX_DEG: f32 = 75.0;
/// The hips lead the feet by at most this much (degrees): the body turns
/// at once (a turn round on the spot, a new way round by the map), the feet
/// take a step at a time; hips that turned with the body left the feet on
/// the wrong sides of them, the legs crossed and twisted (user: "有时候 ai
/// 会以奇怪的姿势扭曲脚"). The rest of a quick turn is the upper body's.
const HIPS_AHEAD_FEET_DEG: f32 = 45.0;
/// A foot on the ground pivots (turns where it stands) to stay within this
/// of where the hips face (degrees): the body turns with the head, the feet
/// twist and step after it. Turning its toes in toward the other foot, less
/// (they would cross).
const PIVOT_DEG: f32 = 40.0;
const PIVOT_IN_DEG: f32 = 15.0;
/// A step lands no nearer the other foot than this share of their standing
/// distance.
const STEP_KEEP_APART: f32 = 0.7;

/// The hips turn with the body, behind it at most this much (degrees): a
/// quick turn of the head takes the body round with it.
const HIPS_BEHIND_DEG: f32 = 15.0;

/// The feet stay where they stand while the body turns above them, then
/// step after it one at a time, round the body's centre (the head's plumb
/// line), the foot on the side it turns to first: turning on the spot, the
/// feet tread round instead of sliding round, and never through each other.
#[derive(Default)]
struct Feet {
    /// Where each foot stands (left, right), once known.
    planted: [Option<vrc_vr::Pose>; 2],
    /// The foot stepping now: which, from, to, the turn it makes round the
    /// centre (degrees), how far through (0..1), how long it takes.
    step: Option<(usize, vrc_vr::Pose, vrc_vr::Pose, f32, f32, f32)>,
    /// Where the hips face (degrees), close behind the body.
    hips: Option<f32>,
}

fn yaw_of(p: &vrc_vr::Pose) -> f32 {
    p.yaw_pitch().0
}

fn turn_between(a: f32, b: f32) -> f32 {
    (b - a + 540.0).rem_euclid(360.0) - 180.0
}

/// `p` turned `deg` (+ right) round the vertical through `centre`.
fn round(p: [f32; 3], centre: [f32; 3], deg: f32) -> [f32; 3] {
    let r = vrc_vr::Pose::looking(deg, 0.0, [0.0; 3]).rotate([p[0] - centre[0], 0.0, p[2] - centre[2]]);
    [centre[0] + r[0], p[1], centre[2] + r[2]]
}

impl Feet {
    /// Lets go: something else moves the feet (a gait, a program).
    fn reset(&mut self) {
        *self = Feet::default();
    }

    /// The feet now (left, right) and the yaw for the hips, given where the
    /// body would stand the feet (`wanted`), its centre and its facing.
    fn update(&mut self, wanted: [vrc_vr::Pose; 2], centre: [f32; 3], body_yaw: f32, dt: f32) -> ([vrc_vr::Pose; 2], f32) {
        for i in 0..2 {
            if self.planted[i].is_none() {
                self.planted[i] = Some(wanted[i]);
            }
        }
        let mut now = [self.planted[0].unwrap(), self.planted[1].unwrap()];
        if let Some((i, from, to, turn, t, secs)) = self.step.as_mut() {
            *t = (*t + dt / *secs).min(1.0);
            let w = ease(*t);
            // Round the centre, then the rest of the way straight.
            let swung = round(from.position, centre, *turn * w);
            let full = round(from.position, centre, *turn);
            let mut p = [0, 1, 2].map(|k| swung[k] + (to.position[k] - full[k]) * w);
            p[1] = from.position[1] + (to.position[1] - from.position[1]) * w + STEP_LIFT_M * (std::f32::consts::PI * *t).sin();
            let yaw = yaw_of(from) + turn_between(yaw_of(from), yaw_of(to)) * w;
            now[*i] = vrc_vr::Pose::looking(yaw, 0.0, p);
            if *t >= 1.0 {
                self.planted[*i] = Some(*to);
                now[*i] = *to;
                self.step = None;
            }
        } else {
            let off = |i: usize| {
                let turn = turn_between(yaw_of(&now[i]), yaw_of(&wanted[i]));
                let p = &now[i].position;
                let dist = ((p[0] - wanted[i].position[0]).powi(2) + (p[2] - wanted[i].position[2]).powi(2)).sqrt();
                (turn, dist)
            };
            let need = |i: usize| {
                let (turn, dist) = off(i);
                turn.abs() / STEP_TURN_DEG + dist / STEP_OFF_M
            };
            // The foot on the side of the turn first (right turn: the right
            // foot), unless the other is much further off.
            let (turn0, _) = off(0);
            let lead = if turn0 > 0.0 { 1 } else { 0 };
            let i = if need(1 - lead) > 1.5 * need(lead) { 1 - lead } else { lead };
            let (turn, dist) = off(i);
            if turn.abs() > STEP_TURN_DEG || dist > STEP_OFF_M {
                let far = turn.abs() > FAR_DEG;
                let most = if far { STEP_FAR_MAX_DEG } else { STEP_MAX_DEG };
                let k = (most / turn.abs().max(1e-3)).min(1.0);
                let from = now[i];
                let swing = turn * k;
                let mut to = if k >= 1.0 {
                    wanted[i]
                } else {
                    vrc_vr::Pose::looking(yaw_of(&from) + swing, 0.0, round(from.position, centre, swing))
                };
                // Not onto the other foot: no nearer it than most of their
                // standing distance.
                let other = now[1 - i].position;
                let apart = |a: [f32; 3], b: [f32; 3]| ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                let keep = STEP_KEEP_APART * apart(wanted[0].position, wanted[1].position);
                let d = apart(to.position, other);
                if d < keep && d > 1e-4 {
                    let s = keep / d;
                    to.position[0] = other[0] + (to.position[0] - other[0]) * s;
                    to.position[2] = other[2] + (to.position[2] - other[2]) * s;
                }
                self.step = Some((i, from, to, swing, 0.0, if far { STEP_FAR_S } else { STEP_S }));
            }
        }
        // The hips with the body (a little behind it at most); a foot on the
        // ground pivots where it stands to keep within PIVOT_DEG of them.
        self.hips = match self.hips {
            Some(h) => {
                let d = turn_between(h, body_yaw);
                Some(body_yaw - d.clamp(-HIPS_BEHIND_DEG, HIPS_BEHIND_DEG) * (1.0 - (dt / 0.15).min(1.0)))
            }
            None => Some(body_yaw),
        };
        // Not further round than the feet allow: the way they stand, from
        // where they are (left to right is the stance's right), not how they
        // point (a planted foot pivots where it stands: by its pointing the
        // hips went round past it and the feet stood in a line across them).
        let (l, r) = (now[0].position, now[1].position);
        let feet_yaw = (r[2] - l[2]).atan2(r[0] - l[0]).to_degrees();
        let lead = turn_between(feet_yaw, self.hips.unwrap());
        if lead.abs() > HIPS_AHEAD_FEET_DEG {
            self.hips = Some(feet_yaw + lead.clamp(-HIPS_AHEAD_FEET_DEG, HIPS_AHEAD_FEET_DEG));
        }
        let hips = self.hips.unwrap();
        let stepping = self.step.as_ref().map(|s| s.0);
        for i in 0..2 {
            if stepping == Some(i) {
                continue;
            }
            // The foot behind the hips by `off`: it pivots to within its
            // limit. Toes turning in (the left foot turning right, the right
            // foot left) go less far.
            let off = turn_between(hips, yaw_of(&now[i]));
            let turning_in = if i == 0 { off < 0.0 } else { off > 0.0 };
            let limit = if turning_in { PIVOT_IN_DEG } else { PIVOT_DEG };
            if off.abs() > limit {
                let yaw = hips + off.clamp(-limit, limit);
                now[i] = vrc_vr::Pose::looking(yaw, 0.0, now[i].position);
                self.planted[i] = Some(now[i]);
            }
        }
        (now, hips)
    }
}

/// Between two poses (0: a, 1: b).
fn lerp_pose(a: &vrc_vr::Pose, b: &vrc_vr::Pose, w: f32) -> vrc_vr::Pose {
    let position = [0, 1, 2].map(|k| a.position[k] + (b.position[k] - a.position[k]) * w);
    let dot: f32 = (0..4).map(|i| a.orientation[i] * b.orientation[i]).sum();
    let s = if dot < 0.0 { -1.0 } else { 1.0 };
    let q: [f32; 4] = std::array::from_fn(|i| a.orientation[i] * (1.0 - w) + s * b.orientation[i] * w);
    let n = q.iter().map(|x| x * x).sum::<f32>().sqrt().max(1e-6);
    vrc_vr::Pose { orientation: q.map(|x| x / n), position }
}

/// Eased in and out (0..1 to 0..1).
fn ease(w: f32) -> f32 {
    w * w * (3.0 - 2.0 * w)
}

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
    /// The legs (and arms) walk and run with the bot's moves (gait cycles);
    /// off: they stand.
    pub gait: bool,
    /// Calibrate by itself when VRChat tracks head and hands only though
    /// the trackers are on (after the game started), while nothing else
    /// moves the bot (`crate::calibrate::watch`).
    pub auto_calibrate: bool,
}

impl Default for TrackerSettings {
    fn default() -> Self {
        TrackerSettings { on: false, parts: vec!["hip".into(), "feet".into()], head: "once".into(), shift: Default::default(), scale: None, gait: true, auto_calibrate: true }
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
    /// The head as last sent to the headset, the animation's on the
    /// owner's, and when (what the game's listener hears from: the speaker
    /// tracker turns head-relative directions with it).
    pub head: Mutex<Option<(Instant, vrc_vr::Pose)>>,
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
    /// The headset's owner holds the hands where it put them (a little back
    /// while a follow looks about): the animation moves the rest.
    pub owner_hands: AtomicBool,
    /// The motion program playing (`crate::motion`): it moves the trackers,
    /// the headset and the hands instead of all the above.
    pub motion: Mutex<Option<crate::motion::Program>>,
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
            head: Mutex::new(None),
            trackers: Mutex::new(trackers),
            trackers_on_since: Mutex::new(None),
            head_rotation_due: AtomicBool::new(true),
            manual_hands: AtomicBool::new(false),
            owner_hands: AtomicBool::new(false),
            motion: Mutex::new(None),
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

    /// Sends the trackers (`legs`: where a gait puts them instead of
    /// standing), with the headset where it is now (`hmd`, the animation on
    /// it): VRChat aligns the trackers to the head every frame, so a head
    /// that moves without the trackers knowing would drag the feet along.
    fn send_trackers(&self, bridge: &Bridge, state: &vrc_vr::remote::State, hmd: vrc_vr::Pose, legs: Option<Vec<(Part, vrc_vr::Pose)>>) {
        let settings = self.trackers.lk().clone();
        if !settings.on {
            *self.trackers_on_since.lk() = None;
            return;
        }
        self.trackers_on_since.lk().get_or_insert_with(Instant::now);
        let frame = match self.tracker_frame(&settings, state) {
            Ok(mut f) => {
                f.head = vrc_vr::Pose { orientation: hmd.orientation, position: hmd.position.map(|x| x * f.scale) };
                if let Some(legs) = legs {
                    f.trackers = legs;
                }
                f
            }
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

    /// Sends the trackers a motion program placed (when they are on).
    fn send_placed_trackers(&self, bridge: &Bridge, placed: &vrc_vr::motion::Placed) {
        let settings = self.trackers.lk().clone();
        if !settings.on {
            *self.trackers_on_since.lk() = None;
            return;
        }
        self.trackers_on_since.lk().get_or_insert_with(Instant::now);
        let rotation = match settings.head.as_str() {
            "always" => true,
            "once" => self.head_rotation_due.swap(false, Ordering::Relaxed),
            _ => false,
        };
        let head = (settings.head != "off").then_some(&placed.head);
        for m in trackers::messages(&placed.trackers, head, rotation) {
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
        let mut usercam = crate::usercam::Watch::new();
        let mut link: Option<HmdLink> = None;
        let mut osc: Option<Osc> = None;
        let mut speed = 0.0f32;
        let mut eye_height = 0.0f32;
        let mut airborne = false;
        let mut gait = Gait::default();
        let mut feet = Feet::default();
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
                let velocity = osc.as_ref().map(|o| (o.query("/avatar/parameters/VelocityZ"), o.query("/avatar/parameters/VelocityX")));
                match velocity {
                    Some((Ok(z), x)) => speed = ground_speed(z as f32, x.map_or(0.0, |x| x as f32)),
                    _ => {
                        osc = None;
                        speed = 0.0;
                    }
                }
                // Off the ground (a jump, a fall): the legs tuck up.
                airborne = matches!(osc.as_ref().map(|o| o.query("/avatar/parameters/Grounded")), Some(Ok(g)) if g == 0.0);
                if tick.is_multiple_of(SIZE_EVERY) || eye_height == 0.0 {
                    if let Some(Ok(h)) = osc.as_ref().map(|o| o.eye_height()) {
                        eye_height = h as f32;
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
            if tick.is_multiple_of(CALIBRATION_EVERY) {
                watch.check(&bridge);
                usercam.check(&bridge);
            }
            let link = link.as_ref().unwrap();
            // A motion program plays on everything.
            let played = self.motion.lk().as_mut().map(|m| {
                let placed = m.tick(dt);
                let o = vrc_vr::remote::Overlay {
                    left: m.hand(0, placed.left),
                    right: m.hand(1, placed.right),
                    rest: overlay.rest,
                    head: Default::default(),
                    head_pose: Some(placed.head),
                };
                (placed, o)
            });
            if let Some((placed, o)) = played {
                *self.head.lk() = Some((now, placed.head));
                self.send_placed_trackers(&bridge, &placed);
                off_sent = false;
                if let Err(e) = link.set_overlay(Some(o)) {
                    tracing::debug!("animation: {e:#}");
                }
                continue;
            }
            // The legs walk while the bot moves (not while a scan holds
            // still or the hands are set by hand).
            let manual = self.manual_hands.load(Ordering::Relaxed);
            let stature = if eye_height > 0.0 { eye_height / 0.936 } else { 1.6 };
            let gait_on = self.trackers.lk().gait;
            let walking = if owner.still || manual || !gait_on { None } else { gait.update(&bridge.motions, speed, stature, dt, airborne) }.map(|body| {
                let stand = vrc_vr::motion::Stand {
                    eyes: owner.state.head.position,
                    yaw_deg: owner.state.body_yaw,
                    floor_y: FLOOR_Y,
                };
                let parts = self.trackers.lk().parts().unwrap_or_default();
                let attached = crate::motion::attachments(&stand, &parts, &animator.params);
                let standing = bridge.motions.get("walk_cycle").map(|c| c.standing).unwrap_or(body);
                vrc_vr::motion::place(&stand, &standing, &body, crate::motion::EYES_AHEAD, &attached)
            });
            // The arms swing with the legs (the same cycle, the same phase),
            // in place of the animation's own swing; the head stays its own.
            let mut overlay = overlay;
            if let Some(placed) = &walking {
                let w = gait.weight.min(1.0);
                overlay.left.pose = lerp_pose(&overlay.left.pose, &placed.left, w);
                overlay.right.pose = lerp_pose(&overlay.right.pose, &placed.right, w);
            }
            if self.owner_hands.load(Ordering::Relaxed) {
                overlay.left = owner.state.left;
                overlay.right = owner.state.right;
                overlay.rest = [owner.state.left, owner.state.right];
            }
            let mut legs = walking.map(|p| p.trackers);
            // Standing (no gait): the feet stay put and step after a turn
            // (a scan's too).
            if legs.is_some() || manual {
                feet.reset();
            } else if let Ok(parts) = self.trackers.lk().parts() {
                let lean = *self.head_lean.lk();
                let head = owner.state.head.position;
                let standing_head = [head[0] - lean[0], head[1] - lean[1], head[2] - lean[2]];
                let wanted = trackers::standing(&parts, standing_head, owner.state.body_yaw, FLOOR_Y);
                let pick = |p: Part| wanted.iter().find(|(q, _)| *q == p).map(|(_, x)| *x);
                if let (Some(l), Some(r)) = (pick(Part::LeftFoot), pick(Part::RightFoot)) {
                    let ([l, r], hips_yaw) = feet.update([l, r], standing_head, owner.state.body_yaw, dt);
                    // The rest (hips, chest, knees) turned as far as the hips.
                    let mut stood = trackers::standing(&parts, standing_head, hips_yaw, FLOOR_Y);
                    for (part, pose) in stood.iter_mut() {
                        match part {
                            Part::LeftFoot => *pose = l,
                            Part::RightFoot => *pose = r,
                            _ => {}
                        }
                    }
                    legs = Some(stood);
                }
            }
            // Where the headset is: the animation's head on the owner's,
            // unless it is off (or holding still: then the owner's alone).
            let hmd = if enabled && !manual && !owner.still { overlay.head.apply(owner.state.head) } else { owner.state.head };
            *self.head.lk() = Some((now, hmd));
            self.send_trackers(&bridge, &owner.state, hmd, legs);
            let sent = if enabled && !manual {
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

/// The gait's speed from the avatar's velocity (its own axes: `z` ahead,
/// `x` right): over the ground, backwards when it goes back. A step aside
/// walks the legs too (it slid with `VelocityZ` alone: the last metre onto a
/// named place goes aside, `VrCore::settle`).
fn ground_speed(z: f32, x: f32) -> f32 {
    let over = z.hypot(x);
    if z < 0.0 && -z >= x.abs() {
        -over
    } else {
        over
    }
}

#[cfg(test)]
mod feet_tests {
    use super::*;

    /// The feet where the body would stand them, facing `yaw` (0.2 m apart).
    fn wanted(yaw: f32) -> [vrc_vr::Pose; 2] {
        let f = vrc_vr::Pose::looking(yaw, 0.0, [0.0; 3]);
        let r = f.rotate([1.0, 0.0, 0.0]);
        [-0.1f32, 0.1].map(|side| vrc_vr::Pose::looking(yaw, 0.0, [r[0] * side, 0.0, r[2] * side]))
    }

    /// Where the left foot is from the right, across the hips (+: to the
    /// left of it, as it should be).
    fn apart_across(feet: &[vrc_vr::Pose; 2], hips: f32) -> f32 {
        let r = vrc_vr::Pose::looking(hips, 0.0, [0.0; 3]).rotate([1.0, 0.0, 0.0]);
        let d = [feet[1].position[0] - feet[0].position[0], feet[1].position[2] - feet[0].position[2]];
        d[0] * r[0] + d[1] * r[2]
    }

    #[test]
    fn a_step_aside_walks_the_legs() {
        assert_eq!(ground_speed(1.0, 0.0), 1.0);
        assert!((ground_speed(0.0, 0.5) - 0.5).abs() < 1e-6, "aside: walking");
        assert!((ground_speed(0.0, -0.5) - 0.5).abs() < 1e-6);
        assert!((ground_speed(-1.0, 0.2) + 1.0198).abs() < 1e-3, "back: backwards");
    }

    #[test]
    fn a_turn_round_at_once_never_crosses_the_legs() {
        for turn in [180.0f32, -170.0, 120.0, -90.0] {
            let mut feet = Feet::default();
            let centre = [0.0, 1.5, 0.0];
            let dt = 0.04;
            for _ in 0..10 {
                feet.update(wanted(0.0), centre, 0.0, dt);
            }
            let mut last = None;
            let mut done = None;
            for k in 0..100 {
                let (now, hips) = feet.update(wanted(turn), centre, turn, dt);
                let across = apart_across(&now, hips);
                assert!(across > 0.05, "turn {turn}, tick {k}: the legs cross ({across:.3} m, hips {hips:.0})");
                for f in &now {
                    let twist = turn_between(hips, yaw_of(f)).abs();
                    assert!(twist <= PIVOT_DEG + HIPS_AHEAD_FEET_DEG + 1.0, "turn {turn}, tick {k}: a foot {twist:.0} deg off the hips");
                }
                if done.is_none() && turn_between(hips, turn).abs() < 2.0 && now.iter().all(|f| turn_between(yaw_of(f), turn).abs() < STEP_TURN_DEG + 1.0) {
                    done = Some(k as f32 * dt);
                }
                last = Some((now, hips));
            }
            // About a second for a turn round (six steps).
            let took = done.expect("round in the end");
            assert!(took <= 1.6, "turn {turn}: {took:.2} s");
            eprintln!("turn {turn}: round in {took:.2} s");
            // Round in the end: the hips and both feet facing the new way.
            let (now, hips) = last.unwrap();
            assert!(turn_between(hips, turn).abs() < 2.0, "turn {turn}: hips at {hips}");
            for f in &now {
                assert!(turn_between(yaw_of(f), turn).abs() < STEP_TURN_DEG + 1.0, "turn {turn}: a foot at {}", yaw_of(f));
            }
        }
    }
}
