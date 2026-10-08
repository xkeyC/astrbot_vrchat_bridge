//! The user camera's lens round the bot (`docs/full-vr/agent-vr-use.md`
//! section 8, decision D35), and the nameplates it reads.
//!
//! The avatar's own cameras never draw nameplates, and with the panorama
//! always on (the target) the eyes are panorama tiles: the user camera's
//! stream view, the game's desktop window, is where the names are read.
//!
//! - **Standing: the front lens** (`idle: "front"`, the default since the
//!   user found the orbit drew too much attention). The lens rests where
//!   the travel lens would be (above and a little behind the head, looking
//!   where the head looks, a little down) and is aimed again only when the
//!   heading drifts `travel.reaim_deg` off it (at most every
//!   `travel.reaim_every_ms`): one Pose, flying off 150 ms after, no
//!   stream of Poses. A name in another direction is read by turning the
//!   lens there a moment (`Orbit::look_at`, `Orbit::name_toward`,
//!   `POST /v1/vr/usercam/look`), then it comes back.
//! - **Standing: the orbit** (`idle: "orbit"`). The lens goes round the head (`radius_m` out,
//!   about head height, a gentle bob), looking outward and a little down,
//!   one turn every `period_s`: the stream view sweeps all round the bot
//!   and keeps seeing the nameplates every way. Other players see the lens
//!   anyway (its visibility cannot be hidden from them), so it might as
//!   well move.
//! - **Moving: the travel lens.** Every `/usercamera/Pose` turns the
//!   camera's flying on, and while it is on the movement inputs fly the
//!   camera, not the bot. All movement goes through one choke point, the
//!   OSC move gate (`vrc_vr::osc::set_move_gate`: the follower's legs,
//!   `/v1/step`, `goto`, the stick, a jump). A push from standing (a walk,
//!   each leg of one) stops the orbit and puts the lens above and a little
//!   behind the head, looking the way the push goes and a little down
//!   (`travel`), sends `Flying` false 150 ms after that Pose (sent
//!   together, the Pose's flying wins) and lets the push go `settle_ms`
//!   after that. With the camera's follow mode 玩家位置 (set once in the
//!   game, measured 2026-10-08) the lens then rides along with the bot's
//!   position, but keeps its world yaw: when the way the bot goes drifts
//!   more than `reaim_deg` off it (at most every `reaim_every_ms`), the
//!   orbit's thread lets go of the axes, aims the lens again, turns flying
//!   off and pushes them again: the walk stalls about 200 ms.
//! - **A voice: the lens turns to it** (`attend_onset`, off by default
//!   since 2026-10-09: too abrupt; the bot itself turns when called). When the speaker
//!   tracker opens a new speech (not the bot's echo) and has a bearing for
//!   it (its player's, or its votes' peak), the orbit stops and the lens
//!   goes 0.3 m out from the head that way, looking along it, a little
//!   down. The frames then are read faster (`attend.ocr_hz`) and every
//!   frame between reads measures the plates read last again: the ring's
//!   onset, and a named bearing for the tracker. No lit plate within
//!   `mirror_after_ms`, and the votes' front/back mirror about as strong:
//!   it looks at the mirror. It holds until the speech ended `hold_s`
//!   ago (the ring's tail is 0.9 s), or another speech comes from a way
//!   more than `reaim_deg` off (at most every `reaim_every_ms`). Moving,
//!   the travel lens gives way only to a voice more than `travel_off_deg`
//!   off the way the bot goes (the walk stalls as for a re-aim).
//! - The orbit comes back once every movement input has been let go and
//!   the bot has stood still (its own speed too) `resume_after_s`. The gate
//!   holds no lock while it waits and never touches the follower; no push
//!   goes out while flying may be on.
//! - **The quick sweep** (`Orbit::sweep`, decision D41): the follower
//!   looking for someone lost, the idle sweep. `snap.views` fixed views
//!   (the look lens's place), one Pose each; a view is taken from the
//!   first frame that shows it (`snap.min_ms` after its Pose and unlike
//!   the frame before it, or `snap.sure_ms` old) and the next view's Pose
//!   goes at once, the reads (OCR) on threads of their own meanwhile (up
//!   to `snap.in_flight`). From where someone was last seen the views go
//!   out either way by turns; their name read in one, the sweep ends and
//!   the lens turns to them (`look_at`). About a second round.
//! - **Explicit placements** (`shot`, `sweep`, opening the camera) hold
//!   `usercam::RUNNING`: the orbit leaves the camera to them and takes it
//!   back after.
//! - **Closed camera**: the orbit does nothing (it looks every 2 s); it
//!   comes back after the camera is opened again (by hand or `keep_open`).
//! - **Name sightings** (`sightings`, a thread of its own): while the lens
//!   orbits or travels, the desktop window is grabbed (an ffmpeg x11grab
//!   stream, `grab_fps`), each frame tagged with the lens's pose `lag_ms`
//!   before it (the game applies a Pose 1-3 frames late; orbit poses are
//!   interpolated). `ocr_hz` times a second (`ocr_hz_hot` while someone
//!   speaks) the latest frame is read by the infra OCR, the names matched
//!   to the room's players, and each read given a bearing from the head
//!   (the lens's offset from the head and yaw, the box's place, the
//!   camera's vertical field of view from `/usercamera/Zoom` unless
//!   `fov_deg` says) and the plate's ring measured there. They go to the
//!   speaker tracker (a bearing alone: `Speakers::saw_bearing`) and are
//!   kept for any reader (`Orbit::names_since`: the follower and the
//!   surveys, once ported to the panorama).

use std::collections::VecDeque;
use std::io::Read;
use std::process::{Child, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use vrc_audio::wrap_deg;
use vrc_players::OcrClient;
use vrc_vr::osc::{encode, Arg, Osc, MOVE_INPUTS};
use vrc_vr::tap::EyeTap;

use crate::anim::Anim;
use crate::bridge::Bridge;
use crate::speaker::{glow_stats, View, Voice};
use crate::usercam::{self, CamPose, MODE_CLOSED};
use crate::Lock;

/// After a Pose: flying off sent sooner is undone (measured 2026-10-08).
const POSE_TAKES: Duration = Duration::from_millis(150);
/// The camera's mode (and its zoom) is read this often.
const CAMERA_EVERY: Duration = Duration::from_secs(2);
/// The head (the position beacon) is read this often standing, and this
/// often moving (the travel lens is placed from it).
const HEAD_EVERY: Duration = Duration::from_secs(1);
const HEAD_MOVING: Duration = Duration::from_millis(100);
/// A head older than this places nothing.
const HEAD_FRESH: Duration = Duration::from_secs(5);
/// The heads read lately, kept this long: where the bot was when a Pose
/// went and when a frame was grabbed (the lens rides along with it).
const KEEP_HEADS: Duration = Duration::from_secs(6);
/// The head moved on from the last two reads at most this far ahead.
const HEAD_AHEAD: Duration = Duration::from_millis(500);
/// A lens frame's thumbnail (grey, this many cells across and down).
const THUMB: (usize, usize) = (32, 18);
/// The head's yaw as last sent to the headset is used this long; older,
/// the beacon's.
const ANIM_HEAD_FRESH: Duration = Duration::from_secs(2);
/// Poses kept for tagging frames.
const KEEP_POSES: Duration = Duration::from_secs(5);
/// A flying off is checked this long after it was sent (OSCQuery), and
/// sent again up to this many times.
const VERIFY_AFTER: Duration = Duration::from_millis(300);
const VERIFY_TRIES: u8 = 3;
/// Movement inputs held this long with the bot not moving at all are taken
/// for a sender that never let go (a task that died): the orbit comes back.
const STALE_PUSH: Duration = Duration::from_secs(30);
/// The camera's vertical field of view when neither `fov_deg` nor
/// `/usercamera/Zoom` says (degrees): 47, measured 2026-10-09 against the
/// eye with the lens at the eye (Zoom reads NaN); 60 had every lens ray
/// too steep, and no body was found under a plate.
const DEFAULT_FOV_DEG: f32 = 47.0;
/// A plate read with a bearing alone is taken this far from the head to
/// turn the lens's bearing into the head's (metres; only the parallax of
/// the lens's offset depends on it).
const PLATE_RANGE_M: f32 = 2.5;
/// Name sightings kept for readers.
const KEEP_NAMES: Duration = Duration::from_secs(60);
const KEEP_NAMES_MAX: usize = 1000;
/// Reads kept for the status.
const KEEP_READS: usize = 12;
/// The speech the lens turns to: going on, or ended this recently.
const VOICE_WITHIN: Duration = Duration::from_secs(2);
/// A plate counts as lit for the lens's turn when its last look, this
/// recent, was.
const LIT_WITHIN: Duration = Duration::from_secs(1);

// -- settings ---------------------------------------------------------------------

/// How the lens goes (`usercam.json` under `orbit`; `POST /v1/vr/usercam
/// {"orbit": {...}}`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct OrbitSettings {
    /// The lens at all: `idle` while the bot stands still, the travel lens
    /// while it moves.
    pub on: bool,
    /// Standing still: the steady front lens (the default; the user found
    /// the orbit drew too much attention) or the orbit.
    pub idle: Idle,
    /// From the eyes, horizontally (metres; outside the head).
    pub radius_m: f32,
    /// One turn (seconds).
    pub period_s: f32,
    /// Over the eyes (metres), and the bob's height (two a turn).
    pub height_m: f32,
    pub bob_m: f32,
    /// The lens looks this far down (degrees).
    pub pitch_deg: f32,
    /// Poses a second.
    pub rate_hz: f32,
    /// Standing still this long brings the orbit back (seconds).
    pub resume_after_s: f32,
    /// The lens while the bot moves.
    pub travel: Travel,
    /// Nameplates read in the lens's view (the speaker tracker, readers).
    pub sightings: bool,
    /// The desktop grabbed this often while sighting (frames a second).
    pub grab_fps: f32,
    /// OCR this often (reads a second), and while someone speaks.
    pub ocr_hz: f32,
    pub ocr_hz_hot: f32,
    /// The camera's vertical field of view (degrees); null: from
    /// `/usercamera/Zoom`.
    pub fov_deg: Option<f32>,
    /// A frame shows the pose sent this long before it (ms).
    pub lag_ms: u64,
    /// Standing idle, the lens goes once round this often (seconds; 0:
    /// never), names read all round placed by the panorama (`people`).
    pub idle_sweep_s: f32,
    /// A held lens (not the orbit) is read only once its Pose was the same
    /// this long by the frame's grab (ms), and the frame looks like the one
    /// before it under that Pose (mean grey difference at most
    /// `settle_diff`, 0..255; 0: not checked). A Pose `settle_sure_ms`
    /// old has taken whatever the frames do (the lens riding along with a
    /// walk, an animated sign, the bot's own head swaying at the edge): no
    /// likeness asked then (decision D40).
    pub settle_ms: u64,
    pub settle_diff: f32,
    pub settle_sure_ms: u64,
    /// Following: the front lens faces the target (its bearing, as the
    /// follower last placed them) rather than the heading, and a lens left
    /// unplaced this long is placed again (seconds; D40).
    pub follow_stale_s: f32,
    /// The lens turned a moment to a bearing (`look_at`).
    pub look: LookLens,
    /// The quick sweep (`Orbit::sweep`, decision D41).
    pub snap: SnapSettings,
    /// The lens turns to a voice as it starts (off by default: too
    /// abrupt; the plates over the pano eyes name who is in front, and an
    /// idle bot called by name turns itself, `POST /v1/vr/attend`).
    pub attend_onset: bool,
    pub attend: AttendSettings,
}

/// What the lens does while the bot stands still.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Idle {
    /// At rest where the travel lens would be (above and a little behind
    /// the head, looking where the head looks, a little down), aimed again
    /// only when the heading drifts `travel.reaim_deg` off it: no stream
    /// of Poses, nothing to notice.
    #[default]
    Front,
    /// Round the head, one turn every `period_s`.
    Orbit,
}

/// The lens turned to a voice: `out_m` from the eyes that way, `height_m`
/// over them, `pitch_deg` down.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AttendSettings {
    pub out_m: f32,
    pub height_m: f32,
    pub pitch_deg: f32,
    /// Held until the speech ended this long ago (seconds).
    pub hold_s: f32,
    /// No lit plate this long after aiming: the votes' mirror next (ms).
    pub mirror_after_ms: u64,
    /// A new speech this far off (degrees) aims again, at most every
    /// `reaim_every_ms`.
    pub reaim_deg: f32,
    pub reaim_every_ms: u64,
    /// Moving: only a voice this far off the way the bot goes (degrees).
    pub travel_off_deg: f32,
    /// OCR this often while turned to a voice (reads a second).
    pub ocr_hz: f32,
}

impl Default for AttendSettings {
    fn default() -> Self {
        AttendSettings {
            out_m: 0.3,
            height_m: 0.0,
            pitch_deg: 5.0,
            hold_s: 1.0,
            mirror_after_ms: 400,
            reaim_deg: 30.0,
            reaim_every_ms: 500,
            travel_off_deg: 60.0,
            ocr_hz: 5.0,
        }
    }
}

impl Default for OrbitSettings {
    fn default() -> Self {
        OrbitSettings {
            on: true,
            idle: Idle::Front,
            radius_m: 0.7,
            period_s: 3.0,
            height_m: 0.0,
            bob_m: 0.04,
            pitch_deg: 8.0,
            rate_hz: 30.0,
            resume_after_s: 1.5,
            travel: Travel::default(),
            sightings: true,
            grab_fps: 15.0,
            ocr_hz: 4.0,
            ocr_hz_hot: 5.0,
            fov_deg: None,
            lag_ms: 70,
            idle_sweep_s: 60.0,
            settle_ms: 150,
            settle_diff: 8.0,
            settle_sure_ms: 600,
            follow_stale_s: 20.0,
            look: LookLens::default(),
            snap: SnapSettings::default(),
            attend_onset: false,
            attend: AttendSettings::default(),
        }
    }
}

/// The lens turned a moment to a bearing (`look_at`): `back_m` behind the
/// eyes (away from that way), `up_m` over them, `pitch_deg` down. Near
/// the head and over it: turned to someone close (behind the bot), it
/// is not in their face, and their plate is in the middle of the view.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LookLens {
    pub back_m: f32,
    pub up_m: f32,
    pub pitch_deg: f32,
}

impl Default for LookLens {
    fn default() -> Self {
        LookLens { back_m: 0.15, up_m: 0.25, pitch_deg: 2.0 }
    }
}

/// The quick sweep (`Orbit::sweep`, decision D41): `views` fixed views
/// round the head, the look lens's place (`look`) each, one Pose a view.
/// A view is read from the first frame that shows it: grabbed at least
/// `min_ms` after its Pose and unlike the frame before the Pose (the
/// mean grey difference at least `diff`), or any frame of it once
/// `sure_ms` old. The next view's Pose goes as soon as one was taken (or
/// after `max_ms` without): the reads (OCR) go on meanwhile, up to
/// `in_flight` at once.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SnapSettings {
    pub views: u32,
    pub min_ms: u64,
    pub diff: f32,
    pub sure_ms: u64,
    pub max_ms: u64,
    pub in_flight: u32,
}

impl Default for SnapSettings {
    fn default() -> Self {
        SnapSettings { views: 6, min_ms: 100, diff: 6.0, sure_ms: 350, max_ms: 700, in_flight: 3 }
    }
}

/// The travel lens: `back_m` behind the eyes (negative: ahead), `up_m`
/// above them, looking the way the bot goes and `pitch_deg` down. Aimed
/// again at the start of every push from standing when the way is more
/// than `leg_deg` off its yaw, and while moving when the way drifts more
/// than `reaim_deg` off (at most every `reaim_every_ms`). A push waits
/// `settle_ms` after the flying off.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Travel {
    pub on: bool,
    pub back_m: f32,
    pub up_m: f32,
    pub pitch_deg: f32,
    pub leg_deg: f32,
    pub reaim_deg: f32,
    pub reaim_every_ms: u64,
    pub settle_ms: u64,
}

impl Default for Travel {
    fn default() -> Self {
        Travel { on: true, back_m: 0.35, up_m: 0.35, pitch_deg: 12.0, leg_deg: 10.0, reaim_deg: 30.0, reaim_every_ms: 1000, settle_ms: 50 }
    }
}

impl OrbitSettings {
    /// Within sane bounds, or why not.
    pub fn checked(self) -> Result<OrbitSettings> {
        let within = |v: f32, lo: f32, hi: f32, what: &str| -> Result<()> {
            anyhow::ensure!(v.is_finite() && (lo..=hi).contains(&v), "{what} is {lo}-{hi}");
            Ok(())
        };
        within(self.radius_m, 0.3, 3.0, "radius_m")?;
        within(self.period_s, 2.0, 60.0, "period_s")?;
        within(self.height_m, -1.0, 2.0, "height_m")?;
        within(self.bob_m, 0.0, 0.3, "bob_m")?;
        within(self.pitch_deg, -60.0, 60.0, "pitch_deg")?;
        within(self.rate_hz, 5.0, 60.0, "rate_hz")?;
        within(self.resume_after_s, 0.5, 10.0, "resume_after_s")?;
        within(self.grab_fps, 1.0, 15.0, "grab_fps")?;
        within(self.ocr_hz, 0.2, 5.0, "ocr_hz")?;
        within(self.ocr_hz_hot, 0.2, 10.0, "ocr_hz_hot")?;
        if let Some(f) = self.fov_deg {
            within(f, 10.0, 150.0, "fov_deg")?;
        }
        anyhow::ensure!(self.lag_ms <= 500, "lag_ms is 0-500");
        anyhow::ensure!(self.settle_ms <= 1000, "settle_ms is 0-1000");
        anyhow::ensure!(self.idle_sweep_s == 0.0 || (10.0..=3600.0).contains(&self.idle_sweep_s), "idle_sweep_s is 0 (never) or 10-3600");
        within(self.settle_diff, 0.0, 255.0, "settle_diff")?;
        anyhow::ensure!(self.settle_sure_ms <= 5000, "settle_sure_ms is 0-5000");
        within(self.follow_stale_s, 5.0, 300.0, "follow_stale_s")?;
        within(self.look.back_m, -1.0, 1.0, "look.back_m")?;
        within(self.look.up_m, -1.0, 1.0, "look.up_m")?;
        within(self.look.pitch_deg, -60.0, 60.0, "look.pitch_deg")?;
        let n = &self.snap;
        anyhow::ensure!((3..=12).contains(&n.views), "snap.views is 3-12");
        anyhow::ensure!(n.min_ms <= 1000, "snap.min_ms is 0-1000");
        within(n.diff, 0.0, 255.0, "snap.diff")?;
        anyhow::ensure!((n.min_ms..=3000).contains(&n.sure_ms), "snap.sure_ms is snap.min_ms-3000");
        anyhow::ensure!((100..=5000).contains(&n.max_ms), "snap.max_ms is 100-5000");
        anyhow::ensure!((1..=4).contains(&n.in_flight), "snap.in_flight is 1-4");
        let t = &self.travel;
        within(t.back_m, -1.0, 2.0, "travel.back_m")?;
        within(t.up_m, -1.0, 2.0, "travel.up_m")?;
        within(t.pitch_deg, -60.0, 60.0, "travel.pitch_deg")?;
        within(t.leg_deg, 0.0, 180.0, "travel.leg_deg")?;
        within(t.reaim_deg, 5.0, 180.0, "travel.reaim_deg")?;
        anyhow::ensure!((200..=10_000).contains(&t.reaim_every_ms), "travel.reaim_every_ms is 200-10000");
        anyhow::ensure!(t.settle_ms <= 500, "travel.settle_ms is 0-500");
        let a = &self.attend;
        within(a.out_m, 0.0, 2.0, "attend.out_m")?;
        within(a.height_m, -1.0, 2.0, "attend.height_m")?;
        within(a.pitch_deg, -60.0, 60.0, "attend.pitch_deg")?;
        within(a.hold_s, 0.0, 10.0, "attend.hold_s")?;
        within(a.reaim_deg, 5.0, 180.0, "attend.reaim_deg")?;
        within(a.travel_off_deg, 0.0, 180.0, "attend.travel_off_deg")?;
        within(a.ocr_hz, 0.2, 10.0, "attend.ocr_hz")?;
        anyhow::ensure!(a.mirror_after_ms <= 5000, "attend.mirror_after_ms is 0-5000");
        anyhow::ensure!((100..=10_000).contains(&a.reaim_every_ms), "attend.reaim_every_ms is 100-10000");
        Ok(self)
    }

    /// These settings changed by a request's `orbit`: a boolean (on or
    /// off) or an object of the fields to change.
    pub fn merged(&self, change: &Value) -> Result<OrbitSettings> {
        if let Some(on) = change.as_bool() {
            return Ok(OrbitSettings { on, ..self.clone() });
        }
        anyhow::ensure!(change.is_object(), "orbit is true, false or an object of its settings");
        let mut v = serde_json::to_value(self)?;
        merge(&mut v, change);
        serde_json::from_value::<OrbitSettings>(v).context("orbit's settings")?.checked()
    }
}

fn merge(into: &mut Value, change: &Value) {
    match (into, change) {
        (Value::Object(a), Value::Object(b)) => {
            for (k, v) in b {
                merge(a.entry(k.clone()).or_insert(Value::Null), v);
            }
        }
        (a, b) => *a = b.clone(),
    }
}

// -- geometry ------------------------------------------------------------------------

/// The lens on the orbit at `angle` (degrees clockwise from +z seen from
/// above, as the beacon's yaw) round the eyes at `eye`: looking outward
/// that way, `pitch_deg` down, bobbing twice a turn.
pub fn orbit_pose(eye: [f32; 3], angle: f32, s: &OrbitSettings) -> CamPose {
    let bob = s.bob_m * (2.0 * angle.to_radians()).sin();
    usercam::around(eye, 0.0, angle, s.radius_m, s.height_m + bob, usercam::Look::Out, -s.pitch_deg)
}

/// The travel lens for the bot at `eye` going `way` (world yaw): above and
/// behind, looking that way.
pub fn travel_pose(eye: [f32; 3], way: f32, t: &Travel) -> CamPose {
    let (s, c) = way.to_radians().sin_cos();
    let position = [eye[0] - t.back_m * s, eye[1] + t.up_m, eye[2] - t.back_m * c];
    CamPose { position, pitch: t.pitch_deg, yaw: way.rem_euclid(360.0), roll: 0.0 }
}

/// The lens turned a moment to `way` (world yaw) from the eyes at `eye`
/// (`look_at`): behind and over the head, looking that way.
pub fn look_pose(eye: [f32; 3], way: f32, l: &LookLens) -> CamPose {
    travel_pose(eye, way, &Travel { back_m: l.back_m, up_m: l.up_m, pitch_deg: l.pitch_deg, ..Travel::default() })
}

/// The lens turned to a voice `way` (world yaw) from the eyes at `eye`.
pub fn attend_pose(eye: [f32; 3], way: f32, a: &AttendSettings) -> CamPose {
    usercam::around(eye, 0.0, way, a.out_m, a.height_m, usercam::Look::Out, -a.pitch_deg)
}

/// The world way (the beacon's yaw) a push goes: the head's yaw and the
/// axes (`Vertical` ahead, `Horizontal` right).
pub fn push_way(head_yaw: f32, axes: [f32; 2]) -> f32 {
    let off = if axes[0].abs() < 1e-3 && axes[1].abs() < 1e-3 { 0.0 } else { axes[1].atan2(axes[0]).to_degrees() };
    (head_yaw + off).rem_euclid(360.0)
}

/// The world direction (Unity: x right, y up, z ahead at yaw 0) through
/// pixel (u, v) of a `w` x `h` view looking `yaw`, `pitch` (+ down),
/// `fov_v` degrees high.
pub fn pixel_ray(yaw: f32, pitch: f32, fov_v: f32, w: f32, h: f32, u: f32, v: f32) -> [f32; 3] {
    let fy = (h / 2.0) / (fov_v.to_radians() / 2.0).tan();
    let (a, b) = ((u - w / 2.0) / fy, -(v - h / 2.0) / fy);
    let (sy, cy) = yaw.to_radians().sin_cos();
    let (sp, cp) = pitch.to_radians().sin_cos();
    let fwd = [sy * cp, -sp, cy * cp];
    let right = [cy, 0.0, -sy];
    let up = [sy * sp, cp, cy * sp];
    [0, 1, 2].map(|k| fwd[k] + a * right[k] + b * up[k])
}

/// The yaw (the beacon's convention) from the eyes to a plate seen from a
/// lens `rel` off them along `ray`, taken `range` metres from the eyes
/// (the parallax of the lens's offset).
pub fn yaw_from_head(rel: [f32; 3], ray: [f32; 3], range: f32) -> f32 {
    let n = ray[0].hypot(ray[2]).max(1e-6);
    let d = [ray[0] / n, ray[2] / n];
    let f = [rel[0], rel[2]];
    let fd = f[0] * d[0] + f[1] * d[1];
    let disc = fd * fd - (f[0] * f[0] + f[1] * f[1]) + range * range;
    let s = (if disc > 0.0 { -fd + disc.sqrt() } else { -fd }).max(0.0);
    let p = [f[0] + s * d[0], f[1] + s * d[1]];
    if p[0].hypot(p[1]) < 1e-3 {
        return d[0].atan2(d[1]).to_degrees();
    }
    p[0].atan2(p[1]).to_degrees()
}

// -- the state ---------------------------------------------------------------------

/// The head as the position beacon last said.
#[derive(Clone, Copy, Debug)]
pub struct Head {
    pub at: Instant,
    /// The eyes (world, between the two) and the head's yaw (the beacon's).
    pub eye: [f32; 3],
    pub yaw: f32,
    /// The beacon's yaw less the tracking space's (both clockwise from
    /// above): a world yaw less this is the tracking yaw the speaker
    /// tracker uses.
    pub offset: f32,
}

/// Who sent a Pose.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Lens {
    Orbit,
    Travel,
    /// At rest ahead (`Idle::Front`): the travel lens's place, the head's
    /// heading.
    Front,
    /// Turned to a voice.
    Attend,
    /// Turned a moment to a bearing to read a name there (`Orbit::look_at`).
    Look,
    /// One of the quick sweep's views (`Orbit::sweep`).
    Snap,
    /// An explicit placement (a shot, a sweep, the check on opening).
    Placed,
}

/// The lens turned to a voice.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Attending {
    /// The speech (the latest of those from that way).
    segment: u64,
    /// Where the lens looks (world yaw), and the votes' mirror not looked
    /// at yet.
    world_yaw: f32,
    mirror: Option<f32>,
    from: &'static str,
    since: Instant,
    aimed_at: Instant,
    /// When it lets go (the speech ended): none while it goes on.
    until: Option<Instant>,
}

/// What the lens does about the voices: the speech `voice` (its yaws
/// turned into the world by `offset`), with a plate seen lit since the
/// last aim or not. Answers the turn (none: the orbit), the latest speech
/// dealt with, and whether to aim now.
fn decide(prev: Option<Attending>, handled: u64, voice: Option<Voice>, offset: f32, lit_since_aim: bool, now: Instant, s: &AttendSettings) -> (Option<Attending>, u64, bool) {
    let mut a = prev;
    let mut handled = handled;
    let mut aim = false;
    let every = Duration::from_millis(s.reaim_every_ms);
    let hold = Duration::from_secs_f32(s.hold_s);
    let world = |yaw: f32| (yaw + offset).rem_euclid(360.0);
    match voice {
        Some(v) if v.segment > handled && v.open => {
            let way = world(v.yaw);
            match a.as_mut() {
                // From the way it looks already: hold on for this one too.
                Some(cur) if wrap_deg(way - cur.world_yaw).abs() <= s.reaim_deg => {
                    cur.segment = v.segment;
                    cur.until = None;
                    handled = v.segment;
                }
                // Another way: aim again, but not too often.
                Some(cur) if now.saturating_duration_since(cur.aimed_at) < every => {}
                _ => {
                    a = Some(Attending { segment: v.segment, world_yaw: way, mirror: v.mirror.map(world), from: v.from, since: now, aimed_at: now, until: None });
                    handled = v.segment;
                    aim = true;
                }
            }
        }
        Some(v) if a.is_some_and(|c| c.segment == v.segment) => {
            let cur = a.as_mut().expect("checked");
            if v.open {
                cur.until = None;
                // Pinned on a placed player since: their way.
                let way = world(v.yaw);
                if v.from == "candidate" && cur.from != "candidate" && wrap_deg(way - cur.world_yaw).abs() > s.reaim_deg && now.saturating_duration_since(cur.aimed_at) >= every {
                    (cur.world_yaw, cur.mirror, cur.from, cur.aimed_at) = (way, None, "candidate", now);
                    aim = true;
                }
            } else if cur.until.is_none() {
                cur.until = Some(v.t1 + hold);
            }
        }
        _ => {
            if let Some(cur) = a.as_mut() {
                cur.until.get_or_insert(now + hold);
            }
        }
    }
    if let Some(cur) = a.as_mut() {
        // Nobody lit that way: the votes' mirror, once.
        if !aim && !lit_since_aim && now.saturating_duration_since(cur.aimed_at) >= Duration::from_millis(s.mirror_after_ms) {
            if let Some(m) = cur.mirror.take() {
                (cur.world_yaw, cur.aimed_at) = (m, now);
                aim = true;
            }
        }
        if cur.until.is_some_and(|u| now >= u) {
            a = None;
            aim = false;
        }
    }
    (a, handled, aim)
}

/// A Pose sent: when, the world pose, by whom, and the lens's offset from
/// the eyes then (with the follow mode 玩家位置 it rides along with the
/// bot: the offset stays while the bot walks).
#[derive(Clone, Copy, Debug)]
struct Sent {
    t: Instant,
    pose: CamPose,
    lens: Lens,
    rel: Option<[f32; 3]>,
}

/// The lens as a frame saw it: its yaw and pitch (world) and its offset
/// from the eyes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tagged {
    pub yaw: f32,
    pub pitch: f32,
    pub rel: [f32; 3],
    /// Where the Pose put it (world): it rides along with the bot since.
    pub position: [f32; 3],
    pub lens: Lens,
    /// When the Pose it follows was sent (frames after the same steady
    /// pose have the same).
    pub posed: Instant,
}

/// A nameplate read in the lens's view.
#[derive(Clone, Debug, Serialize)]
pub struct NameSighting {
    #[serde(skip)]
    pub at: Instant,
    pub name: String,
    /// From the head: the world yaw (the beacon's), the tracking space's,
    /// and off the head's yaw then (+ right).
    pub world_yaw: f32,
    pub tracking_yaw: f32,
    pub bearing_deg: f32,
    /// Up from the lens (degrees).
    pub elevation_deg: f32,
    pub lens: Lens,
    /// The plate's ring score (`GlowStats::score`), and its text box.
    pub glow: Option<f32>,
    pub bbox: [f32; 4],
    /// Where the depth found them under the plate (the panorama: the
    /// feet, world) and how far from the head (metres); none: the bearing
    /// is taken 2.5 m out along the ray.
    pub feet: Option<[f32; 3]>,
    pub distance_m: Option<f32>,
    /// The lens (world) and the ray from it through the plate (world,
    /// unit): the person stands under it (`vrc_pano::people`).
    pub ray_from: [f32; 3],
    pub ray_dir: [f32; 3],
}

#[derive(Default)]
struct State {
    settings: OrbitSettings,
    /// The axes as last asked for (`/input/Vertical`, `/input/Horizontal`)
    /// and the other movement inputs held (bits by MOVE_INPUTS).
    axes: [f32; 2],
    held: u8,
    /// The latest push (a movement input not 0), and when they were all let
    /// go (none: held now, or never pushed).
    last_push: Option<Instant>,
    released_at: Option<Instant>,
    /// Pushes wait for flying to go off until then.
    ready_at: Option<Instant>,
    /// The bot's own speed said it moved, last.
    moved_at: Option<Instant>,
    /// A Pose was sent since the last flying off.
    flying_maybe_on: bool,
    verify_at: Option<Instant>,
    verify_tries: u8,
    /// The Poses sent lately (by anyone).
    sent: VecDeque<Sent>,
    /// The orbit's angle, and whether it sends now.
    angle: Option<f32>,
    active: bool,
    idle: &'static str,
    head: Option<Head>,
    /// The heads read lately (when, the eyes): `eye_at`.
    heads: VecDeque<(Instant, [f32; 3])>,
    /// The head's tracking yaw as last sent to the headset.
    anim_yaw: Option<(Instant, f32)>,
    /// The camera's mode as last read, and its zoom.
    camera: Option<(Instant, Option<i32>)>,
    zoom: Option<f32>,
    last_reaim: Option<Instant>,
    /// The lens turned to a voice; the latest speech dealt with; when a
    /// plate in its view was last seen lit.
    attending: Option<Attending>,
    handled: u64,
    lit_at: Option<Instant>,
    attends: u64,
    /// Turned a moment to a bearing (`Orbit::look_at`): its world yaw and
    /// until when it holds.
    looking: Option<(f32, Instant)>,
    looks: u64,
    /// The quick sweep asked for (`Orbit::sweep`), whose name it looks
    /// for, and how many were.
    sweep: Option<Sweep>,
    seek: Option<String>,
    sweeps: u64,
    snap_poses: u64,
    /// How the last sweep ended ("done", "found", "move", "voice",
    /// "not begun"), and when.
    sweep_end: Option<(&'static str, Instant)>,
    /// The travel lens aimed again less often until then (`calm_travel`:
    /// the follower going round something, its way swinging about).
    calm_until: Option<Instant>,
    /// A follow runs (`set_following`), and where its target was last
    /// placed (world yaw from the head) and when (`aim_at`): the front
    /// lens faces them (decision D40).
    following: bool,
    aim: Option<(f32, Instant)>,
    target_poses: u64,
    stale_poses: u64,
    front_poses: u64,
    /// Counters for the status.
    orbit_poses: u64,
    parks: u64,
    travel_poses: u64,
    reaims: u64,
    flying_offs: u64,
}

impl State {
    fn released(&self) -> bool {
        self.axes == [0.0; 2] && self.held == 0
    }

    /// Whether the bot moves (or just did, or is about to): no orbit.
    fn moving(&self, now: Instant) -> bool {
        // A sweep goes as soon as every input is let go (and flying off has
        // taken): the bot's own coasting is ridden along with (玩家位置).
        if self.sweeping(now) && self.released() && self.ready_at.is_none_or(|r| r <= now) {
            return false;
        }
        let resume = Duration::from_secs_f32(self.settings.resume_after_s);
        let within = |t: Option<Instant>| t.is_some_and(|t| now.saturating_duration_since(t) < resume);
        let stale = self.last_push.is_some_and(|t| now.saturating_duration_since(t) > STALE_PUSH) && !within(self.moved_at);
        (!self.released() && !stale) || self.ready_at.is_some_and(|r| r > now) || within(self.released_at) || within(self.last_push) || within(self.moved_at)
    }

    /// Whether the standing lens may be placed now: standing, or (the
    /// front lens) every input let go though the bot has not stood
    /// `resume_after_s` yet. Before, the lens stayed wherever a sweep or a
    /// look left it, or on a way long gone, until the bot had stood a
    /// while (decision D40); the orbit still waits.
    fn may_front(&self, now: Instant) -> bool {
        !self.moving(now) || (self.settings.idle == Idle::Front && self.released())
    }

    fn camera_open(&self) -> bool {
        self.camera.is_some_and(|(_, m)| m.is_some_and(|m| m != MODE_CLOSED))
    }

    /// A head read: kept for `eye_at` too.
    fn note_head(&mut self, head: Head) {
        self.heads.push_back((head.at, head.eye));
        while self.heads.front().is_some_and(|h| head.at.saturating_duration_since(h.0) > KEEP_HEADS) {
            self.heads.pop_front();
        }
        self.head = Some(head);
    }

    /// Where the eyes were at `t` (world): between the reads about it,
    /// else moved on from the last two (at most HEAD_AHEAD), else the
    /// nearest read.
    fn eye_at(&self, t: Instant) -> Option<[f32; 3]> {
        let lerp = |a: &(Instant, [f32; 3]), b: &(Instant, [f32; 3]), t: Instant| {
            let span = b.0.saturating_duration_since(a.0).as_secs_f32();
            if span <= 1e-4 {
                return b.1;
            }
            let k = (t.saturating_duration_since(a.0).as_secs_f32() - a.0.saturating_duration_since(t).as_secs_f32()) / span;
            [0, 1, 2].map(|i| a.1[i] + (b.1[i] - a.1[i]) * k)
        };
        let i = self.heads.partition_point(|h| h.0 <= t);
        match (i.checked_sub(1).and_then(|j| self.heads.get(j)), self.heads.get(i)) {
            (Some(a), Some(b)) => Some(lerp(a, b, t)),
            (Some(b), None) => match i.checked_sub(2).and_then(|j| self.heads.get(j)) {
                Some(a) => Some(lerp(a, b, t.min(b.0 + HEAD_AHEAD))),
                None => Some(b.1),
            },
            (None, Some(b)) => Some(b.1),
            (None, None) => None,
        }
    }

    /// The lens a frame grabbed at `frame_t` saw (`tag`), placed in the
    /// world: where its Pose put it, moved on as far as the bot moved since
    /// (the camera's follow mode 玩家位置 keeps its offset from the player,
    /// not its turn); and the eyes then. None without heads.
    fn lens_in_world(&self, tag: &Tagged, frame_t: Instant) -> Option<([f32; 3], [f32; 3])> {
        let eye = self.eye_at(frame_t)?;
        let then = self.eye_at(tag.posed)?;
        Some(([0, 1, 2].map(|k| tag.position[k] + eye[k] - then[k]), eye))
    }

    fn fresh_head(&self, now: Instant) -> Option<Head> {
        self.head.filter(|h| now.saturating_duration_since(h.at) <= HEAD_FRESH)
    }

    /// The head's world yaw now: as last sent to the headset (turned into
    /// the world by the beacon), else the beacon's.
    fn head_yaw(&self, now: Instant) -> Option<f32> {
        let head = self.fresh_head(now)?;
        Some(match self.anim_yaw.filter(|(t, _)| now.saturating_duration_since(*t) <= ANIM_HEAD_FRESH) {
            Some((_, y)) => (y + head.offset).rem_euclid(360.0),
            None => head.yaw,
        })
    }

    fn last(&self) -> Option<&Sent> {
        self.sent.back()
    }

    /// How far the travel lens looks off `way` (180 when the lens is not the
    /// travel lens).
    fn lens_off(&self, way: f32) -> f32 {
        match self.last() {
            Some(s) if matches!(s.lens, Lens::Travel | Lens::Front) => wrap_deg(way - s.pose.yaw).abs(),
            _ => 180.0,
        }
    }

    fn record(&mut self, now: Instant, pose: CamPose, lens: Lens, eye: Option<[f32; 3]>) {
        let rel = eye.map(|e| [0, 1, 2].map(|k| pose.position[k] - e[k]));
        self.sent.push_back(Sent { t: now, pose, lens, rel });
        while self.sent.front().is_some_and(|p| now.saturating_duration_since(p.t) > KEEP_POSES) {
            self.sent.pop_front();
        }
        self.flying_maybe_on = true;
        self.verify_at = None;
    }

    /// The travel lens for a push going `way`, when it should be aimed
    /// (`min_off`: degrees off its yaw that call for it).
    fn travel_due(&self, now: Instant, min_off: f32) -> Option<(CamPose, [f32; 3], f32)> {
        let t = &self.settings.travel;
        if !(self.settings.on && t.on && self.camera_open()) {
            return None;
        }
        let head = self.fresh_head(now)?;
        let way = push_way(self.head_yaw(now)?, self.axes);
        // Turned to a voice far off the way: that stays.
        if let (Some(a), Some(Lens::Attend)) = (self.attending, self.last().map(|s| s.lens)) {
            if wrap_deg(a.world_yaw - way).abs() > self.settings.attend.travel_off_deg {
                return None;
            }
        }
        (self.lens_off(way) > min_off).then(|| (travel_pose(head.eye, way, t), head.eye, way))
    }

    /// Whether the axes may be let go a moment to place the lens (pushed,
    /// nothing else held, flying off, no other placement under way).
    fn may_pause(&self, now: Instant) -> bool {
        !self.released() && self.held == 0 && !self.flying_maybe_on && self.ready_at.is_none_or(|r| r <= now)
    }

    fn lit_since(&self, t: Instant) -> bool {
        self.lit_at.is_some_and(|l| l >= t)
    }

    /// Turned to a bearing by `look_at`, still.
    /// The sweep ends (`why`), if there is one.
    fn end_sweep(&mut self, why: &'static str, now: Instant) {
        if self.sweep.take().is_some() {
            self.sweep_end = Some((why, now));
        }
        self.seek = None;
    }

    /// A quick sweep under way: begun and views left (each at most
    /// `max`, and a moment), or asked for within SWEEP_START_WITHIN.
    fn sweeping(&self, now: Instant) -> bool {
        self.sweep.is_some_and(|s| match s.started {
            Some(t) => s.at < s.views && now.saturating_duration_since(t) < s.max * s.views + Duration::from_secs(1),
            None => now.saturating_duration_since(s.asked) < SWEEP_START_WITHIN,
        })
    }

    /// The quick sweep's next Pose for the head at `head`, when one is
    /// due: the first view at once, the next as soon as the view now was
    /// taken (or `max` went by without); none while a view waits for its
    /// frame. The views all gone: the sweep is over ("done").
    fn snap_next(&mut self, head: &Head, now: Instant) -> Option<CamPose> {
        let look = self.settings.look.clone();
        let s = self.sweep.as_mut()?;
        if let Some(p) = s.posed {
            if !s.taken && now.saturating_duration_since(p) < s.max {
                return None;
            }
            s.at += 1;
        }
        if s.at >= s.views {
            self.end_sweep("done", now);
            return None;
        }
        let first = *s.from.get_or_insert(head.yaw);
        s.started.get_or_insert(now);
        (s.posed, s.taken) = (Some(now), false);
        Some(look_pose(head.eye, ring_yaw(first, s.at, s.views, s.out), &look))
    }

    /// A frame of the view posed at `posed` taken (the sightings): the
    /// next view may go. Whether it was the view now.
    fn snap_taken(&mut self, posed: Instant) -> bool {
        match self.sweep.as_mut() {
            Some(s) if s.posed == Some(posed) && !s.taken => {
                s.taken = true;
                true
            }
            _ => false,
        }
    }

    fn looking(&self, now: Instant) -> bool {
        self.looking.is_some_and(|(_, until)| now < until)
    }

    /// Standing, `Idle::Front`: the front lens for the head at `head`
    /// looking `heading` (world yaw), when it should be placed: not the
    /// lens now (after the orbit, a voice, a look, a shot), or the heading
    /// drifted `travel.reaim_deg` off it (and it is `reaim_every_ms` old).
    fn front_due(&self, now: Instant, head: &Head, heading: f32) -> Option<CamPose> {
        self.front_why(now, head, heading).map(|(pose, _)| pose)
    }

    /// `front_due`, and why: "front" (not the lens now, or drifted off),
    /// "target" (following: drifted off the target's way), "stale"
    /// (following: no Pose for `follow_stale_s`). Following, the lens faces
    /// the target while the follower places them (`aim_at`, AIM_FRESH),
    /// else the heading (decision D40).
    fn front_why(&self, now: Instant, head: &Head, heading: f32) -> Option<(CamPose, &'static str)> {
        let t = &self.settings.travel;
        let aim = self.aim_now(now);
        let way = aim.unwrap_or(heading);
        let why = if aim.is_some() { "target" } else { "front" };
        let due = match self.last() {
            Some(s) if matches!(s.lens, Lens::Front | Lens::Travel) => {
                let old = now.saturating_duration_since(s.t);
                if self.following && old >= Duration::from_secs_f32(self.settings.follow_stale_s) {
                    Some("stale")
                } else {
                    (wrap_deg(way - s.pose.yaw).abs() > t.reaim_deg && old >= Duration::from_millis(t.reaim_every_ms)).then_some(why)
                }
            }
            _ => Some(why),
        };
        due.map(|why| (travel_pose(head.eye, way, t), why))
    }

    /// Following: the target's world yaw from the head, while the follower
    /// placed them within AIM_FRESH.
    fn aim_now(&self, now: Instant) -> Option<f32> {
        self.aim.filter(|(_, t)| self.following && now.saturating_duration_since(*t) <= AIM_FRESH).map(|(y, _)| y)
    }
}

/// The quick sweep (`Orbit::sweep`, decision D41): `views` fixed views
/// of the lens round the head, one after the other as each is taken
/// (none begun yet: it waits for the bot to stand, up to
/// SWEEP_START_WITHIN after `asked`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Sweep {
    /// Who asked: "follow" (someone lost), "idle" (the idle sweep).
    why: &'static str,
    asked: Instant,
    /// The first view's world yaw (none: the head's as it begins), and
    /// whether the views go out from it either way by turns (where
    /// someone was last seen first) or round.
    from: Option<f32>,
    out: bool,
    views: u32,
    /// The view now, when its Pose went, and whether a frame of it was
    /// taken (`State::snap_taken`).
    at: u32,
    posed: Option<Instant>,
    taken: bool,
    started: Option<Instant>,
    /// The longest a view waits for its frame (`snap.max_ms`).
    max: Duration,
}

/// The `k`th of `n` views from `first` (world yaws): round, or out from
/// it either way by turns (0, +60, -60, +120, -120, 180 for six).
fn ring_yaw(first: f32, k: u32, n: u32, out: bool) -> f32 {
    let step = 360.0 / n.max(1) as f32;
    let off = if out {
        let m = k.div_ceil(2) as f32 * step;
        if k % 2 == 1 { m } else { -m }
    } else {
        k as f32 * step
    };
    (first + off).rem_euclid(360.0)
}

/// Whether a frame of a quick sweep's view is to be taken: grabbed `age`
/// after its Pose (at least `min_ms`), and unlike the frame before the
/// Pose (`changed`: the mean grey difference) or `sure_ms` old.
fn snap_ready(age: Duration, changed: Option<f32>, s: &SnapSettings) -> bool {
    age >= Duration::from_millis(s.min_ms) && (changed.is_some_and(|d| d >= s.diff) || age >= Duration::from_millis(s.sure_ms))
}

/// The quick sweep found whom it looked for: the lens turned to them
/// (`look_at`) this long.
const SNAP_HIT_HOLD: Duration = Duration::from_millis(2500);
/// A name read is the one sought from this score (`match_score`).
const SEEK_SCORE: f32 = 0.6;

/// Calm (`Orbit::calm_travel`): the travel lens aimed again only this far
/// off the way, at most this often.
const CALM_REAIM_DEG: f32 = 60.0;
const CALM_REAIM_EVERY_MS: u64 = 3000;

/// A sweep not begun this long after it was asked for is dropped.
const SWEEP_START_WITHIN: Duration = Duration::from_secs(2);

/// Following: the target's way (`aim_at`) is faced while this fresh.
const AIM_FRESH: Duration = Duration::from_secs(3);
/// The lens's reads kept for `reads_since` (the follower's confirming look).
const KEEP_READ_LOG: Duration = Duration::from_secs(20);

/// What a push does after the gate looked.
enum Plan {
    Go,
    Wait(Instant),
    Park { off_at: Instant, ready: Instant },
}

/// The lens round the bot: its state, the move gate, and its two threads.
pub struct Orbit {
    osc: Osc,
    anim: Arc<Anim>,
    state: Mutex<State>,
    sights: Mutex<Sights>,
    names: Mutex<VecDeque<NameSighting>>,
    /// The quick sweep's reads (OCR) under way.
    reading: AtomicUsize,
}

#[derive(Default)]
struct Sights {
    grabbing: bool,
    frames: u64,
    reads: u64,
    /// Rings measured between reads (turned to a voice).
    glows: u64,
    untagged: u64,
    /// Frames under a held lens not yet settled (not read).
    unsettled: u64,
    error: Option<String>,
    last: VecDeque<Value>,
    /// The reads lately: when, under which lens, and when its Pose went.
    log: VecDeque<(Instant, Lens, Instant)>,
}

fn sleep_until(t: Instant) {
    let now = Instant::now();
    if t > now {
        std::thread::sleep(t - now);
    }
}

impl Orbit {
    pub fn new(osc: Osc, anim: Arc<Anim>) -> Orbit {
        let state = State { idle: "starting", ..State::default() };
        Orbit { osc, anim, state: Mutex::new(state), sights: Mutex::new(Sights::default()), names: Mutex::new(VecDeque::new()), reading: AtomicUsize::new(0) }
    }

    fn flying_off(&self) {
        let _ = self.osc.send("/usercamera/Flying", &[Arg::Bool(false)]);
        let mut st = self.state.lk();
        st.flying_offs += 1;
        st.verify_at = Some(Instant::now() + VERIFY_AFTER);
        st.verify_tries = VERIFY_TRIES;
    }

    /// The head's yaw as last sent to the headset (tracking space).
    fn anim_yaw(&self) -> Option<(Instant, f32)> {
        self.anim.head.lk().map(|(t, p)| (t, p.yaw_pitch().0))
    }

    /// The move gate (`vrc_vr::osc::set_move_gate`): every movement input
    /// comes here before it is sent. A push from standing aims the travel
    /// lens the way it goes (when it is more than `leg_deg` off); a push
    /// while the lens may be flying turns flying off first; either way the
    /// push goes once that took. Holds the state's lock only to look and to
    /// send one Pose; waits without it.
    pub fn gate(&self, address: &str, value: f32) {
        let anim_yaw = self.anim_yaw();
        let now = Instant::now();
        let plan = {
            let mut st = self.state.lk();
            st.anim_yaw = anim_yaw.or(st.anim_yaw);
            let was_still = st.released();
            match address {
                "/input/Vertical" => st.axes[0] = value,
                "/input/Horizontal" => st.axes[1] = value,
                a => {
                    if let Some(i) = MOVE_INPUTS.iter().position(|m| *m == a) {
                        let bit = 1u8 << i.min(7);
                        if value != 0.0 {
                            st.held |= bit;
                        } else {
                            st.held &= !bit;
                        }
                    }
                }
            }
            if value == 0.0 {
                if !was_still && st.released() {
                    st.released_at = Some(now);
                }
                return;
            }
            st.last_push = Some(now);
            st.released_at = None;
            st.active = false;
            st.end_sweep("move", now);
            if let Some(r) = st.ready_at.filter(|r| *r > now) {
                Plan::Wait(r)
            } else {
                // A push from standing (a walk, a leg of one) aims the lens;
                // while moving, the orbit's thread keeps it aimed.
                let min_off = if was_still { st.settings.travel.leg_deg } else { 180.0 };
                // Flying may be on after the orbit's (or a voice's, a look's)
                // Pose: the travel lens replaces it. After the front lens
                // (or a travel one) looking about the way, it stays.
                let resting = st.last().is_some_and(|s| matches!(s.lens, Lens::Front | Lens::Travel));
                if let Some((pose, eye, _)) = st.travel_due(now, min_off).or_else(|| if st.flying_maybe_on && !resting { st.travel_due(now, -1.0) } else { None }) {
                    let _ = self.osc.send_raw(&pose.message());
                    st.record(now, pose, Lens::Travel, Some(eye));
                    st.travel_poses += 1;
                }
                if !st.flying_maybe_on {
                    Plan::Go
                } else {
                    let posed = st.last().map_or(now, |p| p.t);
                    let off_at = (posed + POSE_TAKES).max(now);
                    let ready = off_at + Duration::from_millis(st.settings.travel.settle_ms);
                    st.ready_at = Some(ready);
                    st.flying_maybe_on = false;
                    st.parks += 1;
                    Plan::Park { off_at, ready }
                }
            }
        };
        match plan {
            Plan::Go => {}
            Plan::Wait(r) => sleep_until(r),
            Plan::Park { off_at, ready } => {
                sleep_until(off_at);
                self.flying_off();
                sleep_until(ready);
            }
        }
    }

    /// While moving: the way the bot goes drifted `reaim_deg` off the
    /// travel lens (and the last aim is `reaim_every_ms` old): the axes let
    /// go (not through the gate: what is asked for stays), the lens aimed
    /// again, flying off, the axes pushed again as asked for by then.
    /// Not with other movement inputs held (a button's press cannot be
    /// repeated).
    fn reaim(&self, now: Instant) -> bool {
        let anim_yaw = self.anim_yaw();
        {
            let mut st = self.state.lk();
            st.anim_yaw = anim_yaw.or(st.anim_yaw);
            let mut t = st.settings.travel.clone();
            if st.calm_until.is_some_and(|c| now < c) {
                t.reaim_deg = t.reaim_deg.max(CALM_REAIM_DEG);
                t.reaim_every_ms = t.reaim_every_ms.max(CALM_REAIM_EVERY_MS);
            }
            let due = st.may_pause(now)
                && st.last_reaim.is_none_or(|r| now.saturating_duration_since(r) >= Duration::from_millis(t.reaim_every_ms))
                && st.last().is_none_or(|s| now.saturating_duration_since(s.t) >= Duration::from_millis(t.reaim_every_ms));
            if !due {
                return false;
            }
            // Following, a lens left alone `follow_stale_s` is placed again
            // whatever its drift (decision D40).
            let stale = st.following && st.last().is_none_or(|s| now.saturating_duration_since(s.t) >= Duration::from_secs_f32(st.settings.follow_stale_s));
            let Some((pose, eye, _)) = st.travel_due(now, if stale { -1.0 } else { t.reaim_deg }) else { return false };
            if stale {
                st.stale_poses += 1;
            }
            st.last_reaim = Some(now);
            st.reaims += 1;
            st.travel_poses += 1;
            self.paused_place(st, now, pose, Lens::Travel, eye);
        }
        true
    }

    /// While moving: the axes let go (not through the gate: what is asked
    /// for stays), `pose` placed, flying off, the axes pushed again as
    /// asked for by then (a push meanwhile waited for this). Takes the
    /// state's lock and lets it go before it waits.
    fn paused_place(&self, mut st: std::sync::MutexGuard<'_, State>, now: Instant, pose: CamPose, lens: Lens, eye: [f32; 3]) {
        let off_at = now + POSE_TAKES;
        let ready = off_at + Duration::from_millis(st.settings.travel.settle_ms);
        st.ready_at = Some(ready);
        for a in ["/input/Vertical", "/input/Horizontal"] {
            let _ = self.osc.send_raw(&encode(a, &[Arg::Float(0.0)]));
        }
        let _ = self.osc.send_raw(&pose.message());
        st.record(now, pose, lens, Some(eye));
        st.flying_maybe_on = false;
        drop(st);
        sleep_until(off_at);
        self.flying_off();
        sleep_until(ready);
        let st = self.state.lk();
        for (a, v) in ["/input/Vertical", "/input/Horizontal"].iter().zip(st.axes) {
            let _ = self.osc.send_raw(&encode(a, &[Arg::Float(v)]));
        }
    }

    /// The voices, as the orbit's thread sees them (`decide`): None when the
    /// lens is not turned to one; else whether it should be aimed now, and
    /// where (world yaw).
    fn attend_tick(&self, voice: Option<Voice>, now: Instant, moving: bool) -> Option<(bool, f32)> {
        let mut st = self.state.lk();
        if !st.settings.attend_onset {
            st.attending = None;
            return None;
        }
        let head = st.fresh_head(now)?;
        let lit = st.attending.is_some_and(|a| st.lit_since(a.aimed_at));
        let (mut a, handled, mut aim) = decide(st.attending, st.handled, voice, head.offset, lit, now, &st.settings.attend);
        st.handled = handled;
        if moving {
            // The travel lens gives way only to a voice far off the way.
            let way = st.head_yaw(now).map(|y| push_way(y, st.axes));
            if let (Some(cur), Some(way)) = (a, way) {
                if wrap_deg(cur.world_yaw - way).abs() <= st.settings.attend.travel_off_deg {
                    (a, aim) = (None, false);
                }
            }
        }
        if aim {
            st.attends += 1;
        }
        // A voice owns the lens: a sweep gives way at once.
        if a.is_some() {
            st.end_sweep("voice", now);
        }
        st.attending = a;
        a.map(|a| (aim, a.world_yaw))
    }

    /// A plate in the lens's view seen lit at `t` (the sightings).
    fn note_lit(&self, t: Instant) {
        let mut st = self.state.lk();
        st.lit_at = Some(st.lit_at.map_or(t, |l| l.max(t)));
    }

    /// A Pose sent by an explicit placement (`usercam::place`).
    pub fn note_pose(&self, pose: CamPose) {
        self.state.lk().record(Instant::now(), pose, Lens::Placed, None);
    }

    /// Flying turned off by an explicit placement.
    pub fn note_flying_off(&self) {
        let mut st = self.state.lk();
        st.flying_maybe_on = false;
        st.verify_at = None;
    }

    /// The lens a frame saw at `t` (its time less the lag): between two
    /// orbit poses, interpolated; after an orbit pose, it while the orbit
    /// goes on; after a travel pose, it until the next. None at a gap, or
    /// after an explicit placement.
    pub fn lens_at(&self, t: Instant) -> Option<Tagged> {
        let st = self.state.lk();
        let gap = Duration::from_secs_f32(2.5 / st.settings.rate_hz.max(1.0));
        let i = st.sent.partition_point(|p| p.t <= t);
        let a = st.sent.get(i.checked_sub(1)?)?;
        let rel = a.rel?;
        let tag = |yaw: f32, pitch: f32, rel: [f32; 3], position: [f32; 3]| Tagged { yaw, pitch, rel, position, lens: a.lens, posed: a.t };
        match (a.lens, st.sent.get(i)) {
            (Lens::Orbit, Some(b)) if b.lens == Lens::Orbit && b.t.saturating_duration_since(a.t) <= gap => {
                let k = (t.saturating_duration_since(a.t).as_secs_f32() / b.t.saturating_duration_since(a.t).as_secs_f32().max(1e-6)).clamp(0.0, 1.0);
                let brel = b.rel?;
                Some(tag(
                    (a.pose.yaw + wrap_deg(b.pose.yaw - a.pose.yaw) * k).rem_euclid(360.0),
                    a.pose.pitch + (b.pose.pitch - a.pose.pitch) * k,
                    [0, 1, 2].map(|j| rel[j] + (brel[j] - rel[j]) * k),
                    [0, 1, 2].map(|j| a.pose.position[j] + (b.pose.position[j] - a.pose.position[j]) * k),
                ))
            }
            (Lens::Orbit, None) if st.active && t.saturating_duration_since(a.t) <= gap => Some(tag(a.pose.yaw, a.pose.pitch, rel, a.pose.position)),
            (Lens::Travel | Lens::Front | Lens::Attend | Lens::Look | Lens::Snap, _) => Some(tag(a.pose.yaw, a.pose.pitch, rel, a.pose.position)),
            _ => None,
        }
    }

    pub fn head(&self) -> Option<Head> {
        self.state.lk().fresh_head(Instant::now())
    }

    /// The names read in the lens's view since `since`, oldest first.
    pub fn names_since(&self, since: Instant) -> Vec<NameSighting> {
        self.names.lk().iter().filter(|n| n.at >= since).cloned().collect()
    }

    pub fn status(&self) -> Value {
        let now = Instant::now();
        let st = self.state.lk();
        let ago = |t: Option<Instant>| t.map(|t| now.saturating_duration_since(t).as_millis() as u64);
        let sights = self.sights.lk();
        json!({
            "settings": st.settings,
            "active": st.active,
            "idle": if st.active { None } else { Some(st.idle) },
            "angle_deg": st.angle.map(|a| (a * 10.0).round() / 10.0),
            "moving": st.moving(now),
            "axes": st.axes,
            "push_ago_ms": ago(st.last_push),
            "released_ago_ms": ago(st.released_at),
            "moved_ago_ms": ago(st.moved_at),
            "flying_maybe_on": st.flying_maybe_on,
            "camera": st.camera.map(|(_, m)| usercam::mode_name(m)),
            "zoom": st.zoom,
            "head": st.head.map(|h| json!({"eye": h.eye, "yaw": (h.yaw * 10.0).round() / 10.0, "age_ms": ago(Some(h.at))})),
            "last_pose": st.last().map(|s| json!({"pose": s.pose, "lens": s.lens, "ago_ms": ago(Some(s.t))})),
            "idle_lens": st.settings.idle,
            "counts": {"snap_poses": st.snap_poses, "orbit_poses": st.orbit_poses, "front_poses": st.front_poses, "target_poses": st.target_poses, "stale_poses": st.stale_poses, "parks": st.parks, "travel_poses": st.travel_poses, "reaims": st.reaims, "attends": st.attends, "looks": st.looks, "sweeps": st.sweeps, "flying_offs": st.flying_offs},
            "following": st.following,
            "aim": st.aim.filter(|_| st.following).map(|(y, t)| json!({
                "world_yaw": r1(y),
                "bearing_deg": st.head.map(|h| r1(wrap_deg(y - h.yaw))),
                "age_ms": now.saturating_duration_since(t).as_millis() as u64,
                "faced": st.aim_now(now).is_some(),
            })),
            "sweeping": st.sweep.filter(|_| st.sweeping(now)).map(|s| json!({
                "why": s.why,
                "seek": st.seek,
                "from": s.from.map(r1),
                "out": s.out,
                "view": s.at,
                "views": s.views,
                "started_ms": s.started.map(|t| now.saturating_duration_since(t).as_millis() as u64),
            })),
            "reading": self.reading(),
            "sweep_end": st.sweep_end.map(|(why, t)| json!({"how": why, "ago_ms": now.saturating_duration_since(t).as_millis() as u64})),
            "calm_ms": st.calm_until.filter(|c| now < *c).map(|c| c.saturating_duration_since(now).as_millis() as u64),
            "looking": st.looking.filter(|(_, u)| now < *u).map(|(y, u)| json!({
                "world_yaw": r1(y),
                "bearing_deg": st.head.map(|h| r1(wrap_deg(y - h.yaw))),
                "until_ms": u.saturating_duration_since(now).as_millis() as u64,
            })),
            "attending": st.attending.map(|a| json!({
                "segment": a.segment,
                "world_yaw": r1(a.world_yaw),
                "bearing_deg": st.head.map(|h| r1(wrap_deg(a.world_yaw - h.yaw))),
                "why": a.from,
                "mirror_left": a.mirror.map(r1),
                "since_ms": ago(Some(a.since)),
                "aimed_ms": ago(Some(a.aimed_at)),
                "lit": st.lit_since(a.aimed_at),
                "until_ms": a.until.map(|u| u.saturating_duration_since(now).as_millis() as u64),
            })),
            "sightings": {
                "grabbing": sights.grabbing,
                "frames": sights.frames,
                "reads": sights.reads,
                "glows": sights.glows,
                "untagged": sights.untagged,
                "unsettled": sights.unsettled,
                "error": sights.error,
                "last": sights.last,
            },
        })
    }

    // -- the orbit's thread ------------------------------------------------------

    /// The lens (a thread of its own): a Pose every 1/`rate_hz` while the
    /// bot stands still and the camera is open; while it moves, the travel
    /// lens kept aimed; otherwise flying off once.
    pub fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        let mut tap = EyeTap::open(&bridge.args.tap);
        let mut head_read: Option<Instant> = None;
        loop {
            let settings = bridge.usercam.settings.lk().orbit.clone();
            let tick = Duration::from_secs_f32(1.0 / settings.rate_hz.clamp(1.0, 60.0));
            std::thread::sleep(tick);
            let now = Instant::now();
            let idle = self.tick(&bridge, &mut tap, &mut head_read, settings, now);
            let mut st = self.state.lk();
            if let Some(why) = idle {
                st.active = false;
                st.idle = why;
            }
        }
    }

    /// One tick: None when an orbit Pose went out, else why not.
    fn tick(&self, bridge: &Bridge, tap: &mut EyeTap, head_read: &mut Option<Instant>, settings: OrbitSettings, now: Instant) -> Option<&'static str> {
        let resume = Duration::from_secs_f32(settings.resume_after_s);
        let moved = bridge.mapping.moved_within(resume);
        let (due_camera, moving, coasting) = {
            let mut st = self.state.lk();
            st.settings = settings.clone();
            if moved {
                st.moved_at = Some(now);
            }
            let moving = st.moving(now);
            let coasting = moving && st.may_front(now);
            (st.camera.is_none_or(|(t, _)| now.saturating_duration_since(t) >= CAMERA_EVERY), moving, coasting)
        };
        let (running, in_world) = {
            let g = bridge.game.lk();
            (g.running, !g.instance.is_empty())
        };
        if !running || !in_world {
            self.state.lk().camera = None;
            return Some("not in a world");
        }
        if due_camera {
            let osc = bridge.osc_query().ok();
            let mode = osc.as_ref().and_then(usercam::mode);
            let zoom = osc.as_ref().and_then(|o| o.query("/usercamera/Zoom").ok()).map(|z| z as f32);
            let mut st = self.state.lk();
            st.camera = Some((now, mode));
            st.zoom = zoom;
        }
        if !self.state.lk().camera_open() {
            self.housekeep(bridge, now);
            return Some("camera closed");
        }
        if !settings.on {
            self.housekeep(bridge, now);
            return Some("off");
        }
        // The head: only while the lens is wanted (reading it asks Monado to
        // copy frames when nobody else does).
        if head_read.is_none_or(|t| now.saturating_duration_since(t) >= if moving { HEAD_MOVING } else { HEAD_EVERY }) {
            *head_read = Some(now);
            // Monado taps for whoever asks: ask only when nobody does.
            let frame = if tap.tapping() { tap.read_unasked() } else { tap.read() };
            if let Ok(Some(frame)) = frame {
                let read: Vec<_> = (0..2).filter_map(|i| vrc_vr::beacon::read(&frame, i)).collect();
                if let Some(first) = read.first() {
                    let n = read.len() as f32;
                    let eye = [0, 1, 2].map(|k| read.iter().map(|b| b.position[k]).sum::<f32>() / n);
                    let tracking = frame.views[0].pose.yaw_pitch().0;
                    self.state.lk().note_head(Head { at: now, eye, yaw: first.yaw, offset: wrap_deg(first.yaw - tracking) });
                }
            }
        }
        let voice = if settings.attend_onset { bridge.speaker.voice(VOICE_WITHIN) } else { None };
        // Someone speaks: the idle sweep gives way at once (the follower's
        // goes on looking for whom it lost).
        if bridge.speaker.speaking() {
            let mut st = self.state.lk();
            if st.sweep.is_some_and(|s| s.why == "idle") {
                st.end_sweep("voice", now);
            }
        }
        if moving && !coasting {
            if let Some((aim, way)) = self.attend_tick(voice, now, true) {
                // A voice far off the way the bot goes: the lens turns to it
                // (the walk stalls a moment), and stays (the travel lens
                // gives way meanwhile).
                if aim {
                    let st = self.state.lk();
                    if let (true, Some(head)) = (st.may_pause(now), st.fresh_head(now)) {
                        let pose = attend_pose(head.eye, way, &settings.attend);
                        self.paused_place(st, now, pose, Lens::Attend, head.eye);
                    }
                } else {
                    self.housekeep(bridge, now);
                }
                return Some("attending");
            }
            if self.state.lk().looking(now) {
                self.housekeep(bridge, now);
                return Some("looking");
            }
            if !self.reaim(now) {
                self.housekeep(bridge, now);
            }
            return Some("moving");
        }
        // Explicit placements hold the camera meanwhile.
        let Some(_camera) = usercam::RUNNING.try_lk() else {
            return Some("busy");
        };
        // A voice: the lens turned to it, until it is over.
        if let Some((aim, way)) = self.attend_tick(voice, now, false) {
            if aim {
                let mut st = self.state.lk();
                if let (false, Some(head)) = (st.moving(now), st.fresh_head(now)) {
                    let pose = attend_pose(head.eye, way, &settings.attend);
                    if self.osc.send_raw(&pose.message()).is_ok() {
                        st.record(now, pose, Lens::Attend, Some(head.eye));
                    }
                }
            }
            drop(_camera);
            // Flying off 150 ms after the turn: a walk may start any time.
            self.housekeep(bridge, now);
            return Some("attending");
        }
        let mut st = self.state.lk();
        // (Checked again under the lock the front's Pose goes out with: a
        // push since then waits at the gate for flying off.)
        if !st.may_front(now) {
            return Some("moving");
        }
        // Turned to a bearing a moment (`look_at`): held till it is over.
        if st.looking(now) {
            drop(st);
            drop(_camera);
            self.housekeep(bridge, now);
            return Some("looking");
        }
        let Some(head) = st.fresh_head(now) else {
            drop(st);
            self.housekeep(bridge, now);
            return Some("no head (the position beacon)");
        };
        // The quick sweep: its views one by one, each as soon as the one
        // before was taken (decision D41).
        if st.sweeping(now) {
            st.active = true;
            if let Some(pose) = st.snap_next(&head, now) {
                if self.osc.send_raw(&pose.message()).is_err() {
                    return Some("OSC send failed");
                }
                st.record(now, pose, Lens::Snap, Some(head.eye));
                st.snap_poses += 1;
            }
            if st.sweep.is_some() {
                return None;
            }
        }
        // A sweep over: the idle lens again.
        let why = if st.sweep.is_some_and(|s| s.started.is_none()) { "not begun" } else { "done" };
        st.end_sweep(why, now);
        if settings.idle == Idle::Front {
            let placed = self.front(&mut st, &head, now);
            drop(st);
            drop(_camera);
            if !placed {
                self.housekeep(bridge, now);
            }
            return None;
        }
        // Going on from where it was (a resume continues the turn).
        let step = match st.last() {
            Some(s) if s.lens == Lens::Orbit && st.active => 360.0 * now.saturating_duration_since(s.t).as_secs_f32().min(0.2) / settings.period_s,
            _ => 0.0,
        };
        let angle = (st.angle.unwrap_or(head.yaw) + step).rem_euclid(360.0);
        let pose = orbit_pose(head.eye, angle, &settings);
        if self.osc.send_raw(&pose.message()).is_err() {
            return Some("OSC send failed");
        }
        st.record(now, pose, Lens::Orbit, Some(head.eye));
        st.angle = Some(angle);
        st.active = true;
        st.orbit_poses += 1;
        None
    }

    /// Standing, `Idle::Front`: the front lens placed when due (`front_due`),
    /// and it rests (counted active). Whether a Pose went out.
    fn front(&self, st: &mut State, head: &Head, now: Instant) -> bool {
        st.looking = None;
        st.active = true;
        let heading = st.head_yaw(now).unwrap_or(head.yaw);
        let Some((pose, why)) = st.front_why(now, head, heading) else { return false };
        if self.osc.send_raw(&pose.message()).is_err() {
            return false;
        }
        st.record(now, pose, Lens::Front, Some(head.eye));
        st.front_poses += 1;
        match why {
            "target" => st.target_poses += 1,
            "stale" => st.stale_poses += 1,
            _ => {}
        }
        true
    }

    /// A follow starts (true) or ends: following, the front lens faces the
    /// target (`aim_at`) and is placed again when left alone
    /// `follow_stale_s` (decision D40).
    pub fn set_following(&self, on: bool) {
        let mut st = self.state.lk();
        st.following = on;
        st.aim = None;
    }

    /// The follower placed its target `world_yaw` from the head (the
    /// beacon's convention).
    pub fn aim_at(&self, world_yaw: f32) {
        let mut st = self.state.lk();
        if st.following {
            st.aim = Some((world_yaw.rem_euclid(360.0), Instant::now()));
        }
    }

    /// How many times the lens read (OCR) under `lens` placed at or after
    /// `since`: a look that read and found no name there saw nobody's.
    pub fn reads_since(&self, since: Instant, lens: Lens) -> u32 {
        self.sights.lk().log.iter().filter(|(_, l, posed)| *l == lens && *posed >= since).count() as u32
    }

    /// Turns the lens a moment to `world_yaw` (the beacon's convention) to
    /// read the name there: 0.3 m out from the head that way (the attend
    /// lens), held `hold`, then back to the idle lens. One Pose, flying off
    /// 150 ms after (the orbit's thread, or the move gate before any push).
    /// While the bot walks the axes are let go a moment, as for a re-aim.
    /// Waits only for that pause, holding nothing; not to be called with
    /// the follower's lock held.
    pub fn look_at(&self, world_yaw: f32, hold: Duration) -> Result<()> {
        // A shot or an opening holds the camera for long: a moment's wait.
        let camera = {
            let until = Instant::now() + Duration::from_millis(100);
            loop {
                if let Some(g) = usercam::RUNNING.try_lk() {
                    break g;
                }
                anyhow::ensure!(Instant::now() < until, "the user camera is busy (a shot, a sweep, opening)");
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        let now = Instant::now();
        let mut st = self.state.lk();
        anyhow::ensure!(st.settings.on, "the lens is off (orbit.on)");
        anyhow::ensure!(st.camera_open(), "the user camera is not open");
        let head = st.fresh_head(now).context("no head (the position beacon)")?;
        let pose = look_pose(head.eye, world_yaw, &st.settings.look);
        if !st.released() {
            anyhow::ensure!(st.may_pause(now), "moving, and the lens cannot be placed just now");
        }
        st.looking = Some((world_yaw.rem_euclid(360.0), now + hold));
        st.looks += 1;
        st.active = false;
        if !st.released() {
            drop(camera);
            self.paused_place(st, now, pose, Lens::Look, head.eye);
            return Ok(());
        }
        self.osc.send_raw(&pose.message())?;
        st.record(now, pose, Lens::Look, Some(head.eye));
        Ok(())
    }

    /// The name read toward `world_yaw` (within `tol_deg`): one read within
    /// `fresh` before the call, or else the lens turned there (`look_at`)
    /// and up to `wait` for a read. Blocks; holds nothing while it waits.
    pub fn name_toward(&self, world_yaw: f32, tol_deg: f32, fresh: Duration, wait: Duration) -> Result<Option<NameSighting>> {
        let t0 = Instant::now();
        let near = |n: &NameSighting| wrap_deg(n.world_yaw - world_yaw).abs() <= tol_deg;
        let best = |list: Vec<NameSighting>| list.into_iter().filter(near).last();
        if let Some(n) = best(self.names_since(t0.checked_sub(fresh).unwrap_or(t0))) {
            return Ok(Some(n));
        }
        self.look_at(world_yaw, wait + POSE_TAKES)?;
        let deadline = t0 + wait + POSE_TAKES;
        while Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(30));
            if let Some(n) = best(self.names_since(t0)) {
                self.state.lk().looking = None;
                return Ok(Some(n));
            }
        }
        Ok(None)
    }

    /// A quick look all round (decision D41), whatever `idle` says:
    /// `snap.views` fixed views of the lens round the head (the look
    /// lens's place), each Pose as soon as the view before was taken (its
    /// first frame that shows it), the reads (OCR) going on meanwhile.
    /// From `from` (a world yaw: where someone was last seen) the views go
    /// out either way by turns (0, +60, -60, +120, -120, 180); none: round
    /// from the head's yaw. `seek`: a name; read in a view, the sweep ends
    /// there ("found") and the lens turns to them (`look_at`). Only with
    /// every movement input let go (the orbit's thread begins it on its
    /// next tick; a push meanwhile ends it). Holds nothing; not to be
    /// called with the follower's lock held.
    pub fn sweep(&self, why: &'static str, from: Option<f32>, seek: Option<&str>) -> Result<()> {
        let now = Instant::now();
        let mut st = self.state.lk();
        anyhow::ensure!(st.settings.on, "the lens is off (orbit.on)");
        anyhow::ensure!(st.camera_open(), "the user camera is not open");
        anyhow::ensure!(st.fresh_head(now).is_some(), "no head (the position beacon)");
        anyhow::ensure!(st.released(), "moving: no sweep");
        anyhow::ensure!(st.attending.is_none(), "turned to a voice");
        let s = &st.settings.snap;
        let (views, max) = (s.views, Duration::from_millis(s.max_ms));
        st.sweep = Some(Sweep { why, asked: now, from: from.map(|y| y.rem_euclid(360.0)), out: from.is_some(), views, at: 0, posed: None, taken: false, started: None, max });
        st.seek = seek.map(str::to_string);
        st.looking = None;
        st.sweeps += 1;
        Ok(())
    }

    /// How long a quick sweep takes at most (every view waiting its
    /// longest, and its start).
    pub fn sweep_budget(&self) -> Duration {
        let s = self.state.lk().settings.snap.clone();
        Duration::from_millis(s.max_ms) * s.views + SWEEP_START_WITHIN
    }

    /// Reads (OCR) of the quick sweep's views under way.
    pub fn reading(&self) -> usize {
        self.reading.load(Ordering::SeqCst)
    }

    /// The names a quick sweep's view read: the one it looks for among
    /// them ends it ("found"), and the lens turns to them.
    fn snap_hit(&self, seen: &[PlateRead]) {
        let way = {
            let st = self.state.lk();
            let Some(want) = st.seek.as_deref().filter(|_| st.sweep.is_some()) else { return };
            seen.iter().find(|s| vrc_players::names::match_score(&s.name, want) >= SEEK_SCORE).map(|s| s.sighting.world_yaw)
        };
        let Some(way) = way else { return };
        self.state.lk().end_sweep("found", Instant::now());
        tracing::info!(world_yaw = way.round(), "usercam lens: the sweep read whom it looked for: the lens to them");
        if let Err(e) = self.look_at(way, SNAP_HIT_HOLD) {
            tracing::debug!("usercam lens: no look after the sweep: {e:#}");
        }
    }

    /// The travel lens aimed again less often for `hold` (60 degrees off
    /// the way, every 3 s at most): going round something the way swings
    /// about, and every re-aim stalls the walk; the names elsewhere come
    /// from the panorama and a look now and then.
    pub fn calm_travel(&self, hold: Duration) {
        self.state.lk().calm_until = Some(Instant::now() + hold);
    }

    /// Whether a sweep is asked for or going round.
    pub fn sweeping(&self) -> bool {
        self.state.lk().sweeping(Instant::now())
    }

    /// The sweep ended early: "found" (the name it looked for was read),
    /// "stopped" (whoever asked gave up). The next tick puts the idle lens
    /// back (the front faces the target while following).
    pub fn end_sweep(&self, why: &'static str) {
        self.state.lk().end_sweep(why, Instant::now());
    }

    /// How the last sweep ended, and when.
    pub fn sweep_end(&self) -> Option<(&'static str, Instant)> {
        self.state.lk().sweep_end
    }

    /// Not orbiting: flying off once a Pose's own has taken (the orbit's
    /// last, a shot's), and checked after.
    fn housekeep(&self, bridge: &Bridge, now: Instant) {
        let (off, verify) = {
            let st = self.state.lk();
            let off = st.flying_maybe_on && st.ready_at.is_none_or(|r| r <= now) && st.last().is_none_or(|p| now >= p.t + POSE_TAKES);
            let verify = !st.flying_maybe_on && st.verify_at.is_some_and(|v| v <= now);
            (off, verify)
        };
        if off {
            self.state.lk().flying_maybe_on = false;
            self.flying_off();
        } else if verify {
            let on = bridge.osc_query().ok().and_then(|o| o.query("/usercamera/Flying").ok()).map(|v| v != 0.0);
            let mut st = self.state.lk();
            if st.flying_maybe_on {
                return; // a Pose since: its own flying off comes
            }
            match on {
                Some(true) if st.verify_tries > 0 => {
                    st.verify_tries -= 1;
                    st.verify_at = Some(now + VERIFY_AFTER);
                    drop(st);
                    let _ = self.osc.send("/usercamera/Flying", &[Arg::Bool(false)]);
                }
                _ => st.verify_at = None,
            }
        }
    }

    // -- name sightings ----------------------------------------------------------------

    /// The name sightings (a thread of its own): while the lens orbits or
    /// travels and others are in the room, the desktop streamed and read
    /// now and then. A quick sweep's view is read from the first frame
    /// that shows it, its read (OCR) on a thread of its own so the next
    /// view goes at once (decision D41).
    pub fn sightings(self: Arc<Self>, bridge: Arc<Bridge>) {
        let mut grab: Option<Grabber> = None;
        let mut ocr: Option<OcrClient> = None;
        let mut not_before: Option<Instant> = None;
        let mut read_at: Option<Instant> = None;
        // The Pose read under last (a held lens's first frame is read at
        // once, whatever the pace).
        let mut read_posed: Option<Instant> = None;
        let mut last_seq = 0u64;
        // The plates of the last read, and the pose it was read under.
        let mut last_read: Option<(Instant, Vec<(String, [f32; 4])>)> = None;
        // The last frame's thumbnail, and its Pose; the one before a new
        // Pose's first frame (a quick sweep's view must differ from it).
        let mut last_thumb: Option<(Instant, Vec<u8>)> = None;
        let mut before_pose: Option<(Instant, Vec<u8>)> = None;
        // The quick sweep's view last taken (its Pose).
        let mut snapped: Option<Instant> = None;
        loop {
            std::thread::sleep(Duration::from_millis(10));
            let now = Instant::now();
            let (settings, lens) = {
                let st = self.state.lk();
                let lens = st.camera_open()
                    && st.last().is_some_and(|s| match s.lens {
                        Lens::Orbit => st.active && now.saturating_duration_since(s.t) < Duration::from_millis(500),
                        Lens::Travel | Lens::Front | Lens::Attend | Lens::Look | Lens::Snap => true,
                        Lens::Placed => false,
                    });
                (st.settings.clone(), lens)
            };
            let (running, room) = {
                let g = bridge.game.lk();
                (g.running, g.others())
            };
            let want = settings.on && settings.sightings && running && !room.is_empty() && lens;
            if grab.as_mut().is_some_and(|g| !g.alive()) {
                self.sights.lk().error = Some("ffmpeg stopped".into());
                grab = None;
                not_before = Some(now + Duration::from_secs(10));
            }
            if !want || not_before.is_some_and(|t| now < t) {
                if grab.take().is_some() {
                    self.sights.lk().grabbing = false;
                }
                continue;
            }
            if grab.as_ref().is_some_and(|g| (g.fps - settings.grab_fps).abs() > 0.01) {
                grab = None;
            }
            let g = match grab.as_mut() {
                Some(g) => g,
                None => match Grabber::start(&bridge.args.display, &bridge.args.desktop_grab, settings.grab_fps) {
                    Ok(g) => {
                        let mut s = self.sights.lk();
                        (s.grabbing, s.error) = (true, None);
                        grab.insert(g)
                    }
                    Err(e) => {
                        tracing::warn!("usercam lens: no desktop stream: {e:#}");
                        self.sights.lk().error = Some(format!("{e:#}"));
                        not_before = Some(now + Duration::from_secs(30));
                        continue;
                    }
                },
            };
            let attending = self.state.lk().attending.is_some();
            let hz = if attending {
                settings.attend.ocr_hz
            } else if bridge.speaker.speaking() {
                settings.ocr_hz_hot
            } else {
                settings.ocr_hz
            };
            let Some((seq, t, img)) = g.latest() else { continue };
            if seq == last_seq {
                continue;
            }
            last_seq = seq;
            self.sights.lk().frames = seq;
            let shown = t.checked_sub(Duration::from_millis(settings.lag_ms)).unwrap_or(t);
            let Some(tag) = self.lens_at(shown) else {
                self.sights.lk().untagged += 1;
                continue;
            };
            let (w, h) = (g.width, g.height);
            let thumb = thumbnail(&img, w, h);
            let like = match last_thumb.replace((tag.posed, thumb.clone())) {
                Some((posed, before)) if posed == tag.posed => Some(thumb_diff(&before, &thumb)),
                Some((_, before)) => {
                    before_pose = Some((tag.posed, before));
                    None
                }
                None => None,
            };
            let same = self.lens_at(t).is_some_and(|n| n.posed == tag.posed);
            let age = t.saturating_duration_since(tag.posed);
            match tag.lens {
                Lens::Orbit => {}
                // A quick sweep's view: its first frame that shows it, once,
                // with a read free.
                Lens::Snap => {
                    if snapped == Some(tag.posed) {
                        continue;
                    }
                    let changed = before_pose.as_ref().filter(|(posed, _)| *posed == tag.posed).map(|(_, before)| thumb_diff(before, &thumb));
                    if !same || !snap_ready(age, changed, &settings.snap) {
                        self.sights.lk().unsettled += 1;
                        continue;
                    }
                    if self.reading() >= settings.snap.in_flight as usize {
                        continue;
                    }
                }
                // A held lens: only once it has settled (its Pose the same by
                // the grab, settle_ms old, the frame like the one before).
                _ => {
                    if !settled(same, age, like, &settings) {
                        self.sights.lk().unsettled += 1;
                        continue;
                    }
                }
            }
            // Where the lens was (the bot moved since its Pose: the lens
            // with it), and the eyes then.
            let Some((lens, eye)) = self.state.lk().lens_in_world(&tag, shown) else {
                self.sights.lk().untagged += 1;
                continue;
            };
            let tag = Tagged { rel: [0, 1, 2].map(|k| lens[k] - eye[k]), ..tag };
            let paced = read_at.is_some_and(|r| now.saturating_duration_since(r).as_secs_f32() < 1.0 / hz);
            let first = tag.lens != Lens::Orbit && read_posed != Some(tag.posed);
            if tag.lens != Lens::Snap && paced && !first {
                // Turned to a voice and holding still: the plates read last
                // measured again in every frame between reads (the ring's
                // onset to a frame).
                if let (Lens::Attend, Some((posed, plates))) = (tag.lens, &last_read) {
                    if *posed == tag.posed {
                        let view = View::rgb(&img, w, h);
                        for (name, bbox) in plates {
                            if let Some(gs) = glow_stats(&view, *bbox) {
                                bridge.speaker.saw_glow(name, t, gs);
                                if bridge.speaker.plate_lit(name, LIT_WITHIN) {
                                    self.note_lit(t);
                                }
                            }
                        }
                        self.sights.lk().glows += plates.len() as u64;
                    }
                }
                continue;
            }
            let Some(head) = self.head().map(|h| Head { eye, ..h }) else { continue };
            if ocr.is_none() {
                match OcrClient::new(&bridge.args.ocr_url, &bridge.args.ocr_model) {
                    Ok(c) => ocr = Some(c),
                    Err(e) => {
                        self.sights.lk().error = Some(format!("no OCR: {e:#}"));
                        not_before = Some(now + Duration::from_secs(30));
                        continue;
                    }
                }
            }
            let client = ocr.clone().expect("made above");
            let names: Vec<String> = room.iter().map(|(_, n)| n.clone()).collect();
            if tag.lens == Lens::Snap {
                // Taken: the next view goes; the read on its own thread.
                snapped = Some(tag.posed);
                self.state.lk().snap_taken(tag.posed);
                self.reading.fetch_add(1, Ordering::SeqCst);
                let (me, bridge) = (self.clone(), bridge.clone());
                let spawned = std::thread::Builder::new().name("usercam-snap".into()).spawn(move || {
                    if let Err(e) = me.read_frame(&bridge, &client, &img, (w, h), &tag, &head, t, &names, &settings) {
                        me.sights.lk().error = Some(format!("OCR: {e:#}"));
                    }
                    me.reading.fetch_sub(1, Ordering::SeqCst);
                });
                if spawned.is_err() {
                    self.reading.fetch_sub(1, Ordering::SeqCst);
                }
                continue;
            }
            (read_at, read_posed) = (Some(now), Some(tag.posed));
            match self.read_frame(&bridge, &client, &img, (w, h), &tag, &head, t, &names, &settings) {
                Ok(plates) => last_read = Some((tag.posed, plates)),
                Err(e) => self.sights.lk().error = Some(format!("OCR: {e:#}")),
            }
        }
    }

    /// One frame read (OCR) under the lens `tag`, the head at `head`: the
    /// room's `names` read placed (the panorama's depth along their rays),
    /// told to the speaker tracker, logged and kept; a quick sweep looking
    /// for one of them ends there and the lens turns to them. The plates
    /// read (their boxes).
    #[allow(clippy::too_many_arguments)]
    fn read_frame(&self, bridge: &Bridge, ocr: &OcrClient, img: &[u8], (w, h): (usize, usize), tag: &Tagged, head: &Head, t: Instant, names: &[String], settings: &OrbitSettings) -> Result<Vec<(String, [f32; 4])>> {
        let lines = ocr.lines_rgb(img, w as u16, h as u16)?;
        let fov = settings.fov_deg.or_else(|| self.state.lk().zoom.filter(|z| (10.0..=150.0).contains(z))).unwrap_or(DEFAULT_FOV_DEG);
        // Names read: the panorama's depth places them (the bearing and
        // distance from the head, not 2.5 m out along the ray).
        let any = lines.iter().any(|l| vrc_players::names::best_match(&l.text, names).is_some());
        let depth = if any && bridge.pano.usable() { bridge.pano.frame().ok().map(|f| vrc_pano::Cloud::new(&f, DEPTH_STEP)) } else { None };
        let seen = read_names(&View::rgb(img, w, h), &lines, names, tag, head, fov, t, depth.as_ref());
        for s in &seen {
            bridge.speaker.saw_bearing(&s.name, s.tracking_yaw, t, s.glow_stats);
            if s.glow_stats.is_some() && bridge.speaker.plate_lit(&s.name, LIT_WITHIN) {
                self.note_lit(t);
            }
        }
        let plates = seen.iter().map(|s| (s.name.clone(), s.sighting.bbox)).collect();
        let now = Instant::now();
        let mut sights = self.sights.lk();
        sights.reads += 1;
        sights.error = None;
        sights.log.push_back((now, tag.lens, tag.posed));
        while sights.log.front().is_some_and(|r| now.saturating_duration_since(r.0) > KEEP_READ_LOG) {
            sights.log.pop_front();
        }
        if !seen.is_empty() {
            let list: Vec<Value> = seen
                .iter()
                .map(|s| json!({"name": s.sighting.name, "bearing_deg": r1(s.sighting.bearing_deg), "distance_m": s.sighting.distance_m.map(r2), "glow": s.sighting.glow.map(r2)}))
                .collect();
            sights.last.push_back(json!({"lens": tag.lens, "lens_yaw": r1(tag.yaw), "fov_deg": fov, "seen": list}));
            while sights.last.len() > KEEP_READS {
                sights.last.pop_front();
            }
        }
        drop(sights);
        // The lens turned to whom a sweep looks for before their name is
        // kept: whoever waits on the name finds the lens on them.
        self.snap_hit(&seen);
        let mut kept = self.names.lk();
        kept.extend(seen.into_iter().map(|s| s.sighting));
        while kept.front().is_some_and(|n| now.saturating_duration_since(n.at) > KEEP_NAMES) || kept.len() > KEEP_NAMES_MAX {
            kept.pop_front();
        }
        Ok(plates)
    }
}

/// One plate read in a frame, with its ring.
struct PlateRead {
    name: String,
    tracking_yaw: f32,
    glow_stats: Option<crate::speaker::GlowStats>,
    sighting: NameSighting,
}

/// The panorama's depth for placing lens names: every this-th pixel.
const DEPTH_STEP: u32 = 4;

/// Whether a frame under a held lens is to be read: its Pose the same at
/// the grab (`same`), `age` old by then (at least `settle_ms`), and, when
/// checked, like the frame before under it (`like`: the mean grey
/// difference) until the Pose is `settle_sure_ms` old: frames that keep
/// changing under a lens long placed (riding along with a walk, a sign's
/// video, the bot's own head swaying at the edge) were never read before
/// (decision D40).
fn settled(same: bool, age: Duration, like: Option<f32>, s: &OrbitSettings) -> bool {
    let sure = age >= Duration::from_millis(s.settle_sure_ms.max(s.settle_ms));
    same && age >= Duration::from_millis(s.settle_ms) && (s.settle_diff <= 0.0 || sure || like.is_some_and(|d| d <= s.settle_diff))
}

/// A grey thumbnail of an RGB frame (THUMB cells, each one pixel's).
fn thumbnail(rgb: &[u8], w: usize, h: usize) -> Vec<u8> {
    let (cw, ch) = THUMB;
    let mut out = Vec::with_capacity(cw * ch);
    for j in 0..ch {
        for i in 0..cw {
            let (x, y) = ((i * 2 + 1) * w / (cw * 2), (j * 2 + 1) * h / (ch * 2));
            let k = (y * w + x) * 3;
            out.push(rgb.get(k..k + 3).map_or(0, |p| ((p[0] as u32 + 2 * p[1] as u32 + p[2] as u32) / 4) as u8));
        }
    }
    out
}

fn thumb_diff(a: &[u8], b: &[u8]) -> f32 {
    if a.len() != b.len() || a.is_empty() {
        return f32::INFINITY;
    }
    a.iter().zip(b).map(|(x, y)| (*x as f32 - *y as f32).abs()).sum::<f32>() / a.len() as f32
}

/// Where a plate seen along `dir` from the lens at `from` is, from the
/// eyes at `eye`: the person the panorama's depth has under it (the world
/// yaw from the eyes, the horizontal distance, the feet), else None.
fn on_depth(depth: &vrc_pano::Cloud, from: [f32; 3], dir: [f32; 3], eye: [f32; 3]) -> Option<(f32, f32, [f32; 3])> {
    let b = depth.person_along(from, dir, &vrc_pano::PeopleParams::default())?;
    let (dx, dz) = (b.feet[0] - eye[0], b.feet[2] - eye[2]);
    Some((dx.atan2(dz).to_degrees().rem_euclid(360.0), dx.hypot(dz), b.feet))
}

/// The room's players' names read in a lens frame (OCR `lines`), each with
/// its bearing from the head and its ring; placed by the panorama's
/// `depth` when there is one (else the bearing is taken 2.5 m out).
#[allow(clippy::too_many_arguments)]
fn read_names(view: &View, lines: &[vrc_players::ocr::OcrLine], names: &[String], tag: &Tagged, head: &Head, fov: f32, at: Instant, depth: Option<&vrc_pano::Cloud>) -> Vec<PlateRead> {
    let (w, h) = (view.width as f32, view.height as f32);
    lines
        .iter()
        .filter_map(|l| {
            let (i, _) = vrc_players::names::best_match(&l.text, names)?;
            let [x, y, bw, bh] = l.bbox;
            let ray = pixel_ray(tag.yaw, tag.pitch, fov, w, h, x + bw / 2.0, y + bh / 2.0);
            let n = (ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]).sqrt().max(1e-6);
            let from = [0, 1, 2].map(|k| head.eye[k] + tag.rel[k]);
            let placed = depth.and_then(|d| on_depth(d, from, ray.map(|v| v / n), head.eye));
            let world = placed.map_or_else(|| yaw_from_head(tag.rel, ray, PLATE_RANGE_M).rem_euclid(360.0), |p| p.0);
            let elevation = ray[1].atan2(ray[0].hypot(ray[2])).to_degrees();
            let tracking = wrap_deg(world - head.offset);
            let glow = glow_stats(view, l.bbox);
            Some(PlateRead {
                name: names[i].clone(),
                tracking_yaw: tracking,
                glow_stats: glow,
                sighting: NameSighting {
                    at,
                    name: names[i].clone(),
                    world_yaw: world,
                    tracking_yaw: tracking,
                    bearing_deg: wrap_deg(world - head.yaw),
                    elevation_deg: elevation,
                    lens: tag.lens,
                    glow: glow.map(|g| g.score()),
                    bbox: l.bbox,
                    feet: placed.map(|p| p.2),
                    distance_m: placed.map(|p| p.1),
                    ray_from: from,
                    ray_dir: ray.map(|v| v / n),
                },
            })
        })
        .collect()
}

fn r1(v: f32) -> f64 {
    (v as f64 * 10.0).round() / 10.0
}

fn r2(v: f32) -> f64 {
    (v as f64 * 100.0).round() / 100.0
}

// -- the desktop stream ------------------------------------------------------------

/// The latest frame of a stream: (its number, when it came, RGB).
type Latest = Arc<Mutex<Option<(u64, Instant, Arc<Vec<u8>>)>>>;

/// The desktop window streamed by ffmpeg (x11grab, raw RGB): a reader
/// thread keeps the latest frame. Dropped, ffmpeg is killed.
struct Grabber {
    child: Child,
    latest: Latest,
    width: usize,
    height: usize,
    fps: f32,
}

impl Grabber {
    fn start(display: &str, spec: &str, fps: f32) -> Result<Grabber> {
        let (mut cmd, width, height) = usercam::x11grab(display, spec, &["-framerate", &format!("{fps}")], None)?;
        cmd.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::null());
        let mut child = cmd.spawn().context("running ffmpeg")?;
        let mut out = child.stdout.take().context("ffmpeg's output")?;
        let latest: Latest = Arc::new(Mutex::new(None));
        let to = latest.clone();
        std::thread::Builder::new().name("usercam-grab".into()).spawn(move || {
            let mut seq = 0u64;
            loop {
                let mut buf = vec![0u8; width * height * 3];
                if out.read_exact(&mut buf).is_err() {
                    return;
                }
                seq += 1;
                *to.lk() = Some((seq, Instant::now(), Arc::new(buf)));
            }
        })?;
        Ok(Grabber { child, latest, width, height, fps })
    }

    fn alive(&mut self) -> bool {
        matches!(self.child.try_wait(), Ok(None))
    }

    fn latest(&self) -> Option<(u64, Instant, Arc<Vec<u8>>)> {
        self.latest.lk().clone()
    }
}

impl Drop for Grabber {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    fn orbit() -> Orbit {
        let anim = Arc::new(Anim::new(std::env::temp_dir().join(format!("vrc-orbit-test-{}", std::process::id())).join("anim.json")));
        Orbit::new(Osc::with_ports("127.0.0.1:9", 0).unwrap(), anim)
    }

    /// An orbit with the camera open, the head at `eye` facing yaw 0, and
    /// the lens orbiting.
    fn orbiting(eye: [f32; 3]) -> Orbit {
        let o = orbit();
        let now = Instant::now();
        {
            let mut st = o.state.lk();
            st.camera = Some((now, Some(usercam::MODE_STREAM)));
            st.head = Some(Head { at: now, eye, yaw: 0.0, offset: 0.0 });
            st.record(now, orbit_pose(eye, 0.0, &OrbitSettings::default()), Lens::Orbit, Some(eye));
            st.active = true;
        }
        o
    }

    #[test]
    fn the_lens_goes_round_the_head_looking_out() {
        let s = OrbitSettings { bob_m: 0.0, ..OrbitSettings::default() };
        let eye = [1.0, 1.2, 2.0];
        for angle in [0.0f32, 90.0, 200.0] {
            let p = orbit_pose(eye, angle, &s);
            let (sn, cs) = angle.to_radians().sin_cos();
            assert!(close(p.position[0], 1.0 + 0.7 * sn, 1e-4) && close(p.position[2], 2.0 + 0.7 * cs, 1e-4), "{p:?}");
            assert!(close(p.position[1], 1.2, 1e-4));
            // Outward, a little down (VRChat: + down).
            assert!(close(p.yaw, angle, 1e-3) && close(p.pitch, 8.0, 1e-4), "{p:?}");
        }
        // The bob: twice a turn, within its height.
        let s = OrbitSettings::default();
        assert!(close(orbit_pose(eye, 45.0, &s).position[1], 1.2 + 0.04, 1e-4));
        assert!(close(orbit_pose(eye, 135.0, &s).position[1], 1.2 - 0.04, 1e-4));
    }

    #[test]
    fn the_travel_lens_is_above_and_behind_looking_the_way() {
        let p = travel_pose([0.0, 1.5, 0.0], 90.0, &Travel::default());
        // Going +x: behind is -x.
        assert!(close(p.position[0], -0.35, 1e-4) && close(p.position[2], 0.0, 1e-4) && close(p.position[1], 1.85, 1e-4), "{p:?}");
        assert!(close(p.yaw, 90.0, 1e-3) && close(p.pitch, 12.0, 1e-4));
        // The way a push goes: the head's yaw and the axes.
        assert!(close(push_way(10.0, [0.6, 0.0]), 10.0, 1e-3));
        assert!(close(push_way(10.0, [0.0, 0.5]), 100.0, 1e-3));
        assert!(close(push_way(10.0, [-0.6, 0.0]), 190.0, 1e-3));
    }

    #[test]
    fn a_pixel_has_a_bearing_from_the_head() {
        // The lens 0.7 m out ahead (+z), looking out and 8 degrees down.
        let rel = [0.0, 0.0, 0.7];
        let mid = pixel_ray(0.0, 8.0, 60.0, 1280.0, 720.0, 640.0, 360.0);
        assert!(close(mid[0], 0.0, 1e-5) && mid[1] < 0.0 && mid[2] > 0.0);
        assert!(close(yaw_from_head(rel, mid, 2.5), 0.0, 1e-3));
        // At the right edge of a 60-degree-high 16:9 view: the horizontal
        // half field of view off the lens's axis.
        let edge = pixel_ray(0.0, 0.0, 60.0, 1280.0, 720.0, 1280.0, 360.0);
        let off = edge[0].atan2(edge[2]).to_degrees();
        assert!(close(off, (16.0 / 9.0 * 30f32.to_radians().tan()).atan().to_degrees(), 0.01), "{off}");
        // From the head, the point on that ray 2.5 m from it: less off.
        let from_head = yaw_from_head(rel, edge, 2.5);
        let (sn, cs) = off.to_radians().sin_cos();
        let s = -0.7 * cs + ((0.7 * cs).powi(2) - 0.49 + 6.25).sqrt();
        let want = (s * sn).atan2(0.7 + s * cs).to_degrees();
        assert!(close(from_head, want, 0.01) && from_head < off, "{from_head} {want} {off}");
        // The lens behind the head looking ahead (travel): more off instead.
        let behind = yaw_from_head([0.0, 0.35, -0.35], edge, 2.5);
        assert!(behind > off, "{behind} {off}");
        // Looking the other way round the bot.
        let ray = pixel_ray(270.0, 8.0, 60.0, 1280.0, 720.0, 640.0, 360.0);
        assert!(close(wrap_deg(yaw_from_head([-0.7, 0.0, 0.0], ray, 2.5)), -90.0, 1e-3));
    }

    #[test]
    fn frames_are_tagged_with_the_lens_then() {
        let o = orbit();
        let t = Instant::now();
        let s = OrbitSettings { bob_m: 0.0, ..OrbitSettings::default() };
        {
            let mut st = o.state.lk();
            st.settings = s.clone();
            for i in 0..10u64 {
                st.record(t + Duration::from_millis(50 * i), orbit_pose([0.0; 3], 3.0 * i as f32, &s), Lens::Orbit, Some([0.0; 3]));
            }
            st.active = true;
        }
        // Halfway between the 4th and 5th: 10.5 degrees.
        let p = o.lens_at(t + Duration::from_millis(175)).unwrap();
        assert!(close(p.yaw, 10.5, 1e-3) && p.lens == Lens::Orbit, "{p:?}");
        let (sn, cs) = 10.5f32.to_radians().sin_cos();
        assert!(close(p.rel[0], 0.7 * sn, 2e-3) && close(p.rel[2], 0.7 * cs, 2e-3), "{p:?}");
        assert!(o.lens_at(t - Duration::from_millis(10)).is_none());
        // After the last, while the orbit goes on: the last.
        assert!(close(o.lens_at(t + Duration::from_millis(470)).unwrap().yaw, 27.0, 1e-3));
        assert!(o.lens_at(t + Duration::from_millis(800)).is_none());
        // A shot: none after it.
        o.state.lk().record(t + Duration::from_millis(500), CamPose { position: [0.0; 3], pitch: 0.0, yaw: 180.0, roll: 0.0 }, Lens::Placed, None);
        assert!(o.lens_at(t + Duration::from_millis(520)).is_none());
        // The travel lens: steady until the next pose.
        o.state.lk().record(t + Duration::from_millis(600), travel_pose([0.0, 1.5, 0.0], 90.0, &Travel::default()), Lens::Travel, Some([0.0, 1.5, 0.0]));
        let tr = o.lens_at(t + Duration::from_secs(3)).unwrap();
        assert!(tr.lens == Lens::Travel && close(tr.yaw, 90.0, 1e-3) && close(tr.rel[0], -0.35, 1e-4) && close(tr.rel[1], 0.35, 1e-4));
    }

    #[test]
    fn a_push_aims_the_travel_lens_and_waits_for_flying_off() {
        let o = orbiting([0.0, 1.5, 0.0]);
        // The first push: the travel lens the way it goes, flying off 150 ms
        // after it, the push let go `settle_ms` after that.
        let t0 = Instant::now();
        o.gate("/input/Vertical", 0.6);
        let took = t0.elapsed();
        assert!(took >= Duration::from_millis(195) && took < Duration::from_millis(500), "{took:?}");
        {
            let st = o.state.lk();
            assert!(!st.active && !st.flying_maybe_on && st.travel_poses == 1 && st.parks == 1 && st.flying_offs == 1);
            assert!(st.moving(Instant::now()));
            let last = st.last().unwrap();
            assert_eq!(last.lens, Lens::Travel);
            assert!(close(last.pose.position[1], 1.85, 1e-4) && close(last.pose.position[2], -0.35, 1e-4) && close(last.pose.yaw, 0.0, 1e-3), "{last:?}");
        }
        // Pushes while walking pass at once (the thread keeps it aimed).
        let t1 = Instant::now();
        o.gate("/input/Vertical", 0.4);
        o.gate("/input/Horizontal", 0.2);
        assert!(t1.elapsed() < Duration::from_millis(50));
        // Let go: still "moving" for the resume time, then not.
        o.gate("/input/Horizontal", 0.0);
        o.gate("/input/Vertical", 0.0);
        {
            let mut st = o.state.lk();
            assert!(st.moving(Instant::now()));
            st.settings.resume_after_s = 0.5;
            assert!(!st.moving(Instant::now() + Duration::from_millis(600)));
        }
        // The next leg goes the same way: the lens stays, the push goes.
        let t2 = Instant::now();
        o.gate("/input/Vertical", 0.6);
        assert!(t2.elapsed() < Duration::from_millis(50));
        o.gate("/input/Vertical", 0.0);
        // A leg to the right (stepping aside): aimed again that way.
        let t3 = Instant::now();
        o.gate("/input/Horizontal", 0.5);
        assert!(t3.elapsed() >= Duration::from_millis(195));
        {
            let st = o.state.lk();
            assert_eq!(st.travel_poses, 2);
            assert!(close(st.last().unwrap().pose.yaw, 90.0, 1e-3));
        }
        o.gate("/input/Horizontal", 0.0);
        // A jump is a push too, and its release lets go.
        o.gate("/input/Jump", 1.0);
        assert!(!o.state.lk().released());
        o.gate("/input/Jump", 0.0);
        assert!(o.state.lk().released());
    }

    #[test]
    fn a_drift_while_walking_aims_again_with_the_axes_let_go() {
        let o = orbiting([0.0, 1.5, 0.0]);
        o.gate("/input/Vertical", 0.6);
        let reaim_every = Duration::from_millis(o.state.lk().settings.travel.reaim_every_ms);
        // Too soon, or not far enough off: nothing.
        assert!(!o.reaim(Instant::now()));
        let later = Instant::now() + reaim_every;
        assert!(!o.reaim(later));
        // The head turned 40 degrees (the beacon's, no headset pose here).
        o.state.lk().head = Some(Head { at: later, eye: [0.0, 1.5, 0.0], yaw: 40.0, offset: 0.0 });
        let t = Instant::now();
        assert!(o.reaim(later));
        assert!(t.elapsed() >= Duration::from_millis(150));
        let st = o.state.lk();
        assert_eq!(st.reaims, 1);
        assert!(close(st.last().unwrap().pose.yaw, 40.0, 1e-3));
        // What is asked for stays: the axes go out again as they were.
        assert!(close(st.axes[0], 0.6, 1e-6) && !st.flying_maybe_on);
    }

    #[test]
    fn with_the_camera_closed_a_push_goes_at_once() {
        let o = orbit();
        let t = Instant::now();
        o.gate("/input/Vertical", 0.6);
        assert!(t.elapsed() < Duration::from_millis(50));
        assert_eq!(o.state.lk().parks, 0);
    }

    #[test]
    fn names_read_in_a_lens_frame_get_bearings_from_the_head() {
        // A frame from the lens orbiting at 90 degrees (out to +x), the head
        // facing 0: a plate in the middle is 90 degrees right of the head;
        // the tracking frame is 30 degrees off the world's.
        let head = Head { at: Instant::now(), eye: [0.0, 1.5, 0.0], yaw: 0.0, offset: 30.0 };
        let tag = Tagged { yaw: 90.0, pitch: 8.0, rel: [0.7, 0.0, 0.0], position: [0.7, 1.5, 0.0], lens: Lens::Orbit, posed: Instant::now() };
        let img = vec![40u8; 1280 * 720 * 3];
        let line = |text: &str, x: f32| vrc_players::ocr::OcrLine { text: text.into(), bbox: [x - 40.0, 340.0, 80.0, 20.0], confidence: 0.9 };
        let lines = [line("xkeyC", 640.0), line("nobody here", 300.0)];
        let names = ["xkeyC".to_string(), "Ann".to_string()];
        let read = read_names(&View::rgb(&img, 1280, 720), &lines, &names, &tag, &head, 60.0, Instant::now(), None);
        assert_eq!(read.len(), 1);
        let s = &read[0].sighting;
        assert_eq!(s.name, "xkeyC");
        assert!(close(s.world_yaw, 90.0, 0.01) && close(s.bearing_deg, 90.0, 0.01) && close(s.tracking_yaw, 60.0, 0.01), "{s:?}");
        assert!(s.elevation_deg < 0.0 && s.lens == Lens::Orbit);
    }

    /// A person 1 m from the head, at world yaw 150 (behind and right), a
    /// cylinder of depth points (the panorama's).
    fn someone_behind() -> (vrc_pano::Cloud, [f32; 3]) {
        let (sn, cs) = 150f32.to_radians().sin_cos();
        let feet = [sn, 0.0, cs];
        let mut points = Vec::new();
        for k in 0..36 {
            let (a, b) = (k as f32 * 10.0).to_radians().sin_cos();
            for j in 2..=32 {
                points.push([feet[0] + 0.15 * a, 0.05 * j as f32, feet[2] + 0.15 * b]);
            }
        }
        let fg = vec![true; points.len()];
        (vrc_pano::Cloud { points, fg, centre: [0.0, 1.5, 0.0], floor: Some(0.0) }, feet)
    }

    #[test]
    fn a_name_read_by_the_lens_is_placed_by_the_depth_not_2_5_m_out() {
        // The lens orbiting at 90 degrees (0.7 m out to +x), someone 1 m
        // away behind the bot (world yaw 150), their plate 0.35 m over
        // their head: the ray from the lens through it.
        let (depth, feet) = someone_behind();
        let head = Head { at: Instant::now(), eye: [0.0, 1.5, 0.0], yaw: 0.0, offset: 0.0 };
        let lens = [0.7, 1.5, 0.0];
        let plate = [feet[0], 1.6 + 0.35, feet[2]];
        let d = [0, 1, 2].map(|k| plate[k] - lens[k]);
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let dir = d.map(|v| v / n);
        // 2.5 m out along the ray, the parallax of the lens's offset puts
        // them nearly behind (about 177 degrees): wrong by over 20.
        let guess = yaw_from_head([0.7, 0.0, 0.0], dir, PLATE_RANGE_M).rem_euclid(360.0);
        assert!(wrap_deg(guess - 150.0).abs() > 20.0, "{guess}");
        // The depth: where they stand, from the head.
        let (yaw, distance, at) = on_depth(&depth, lens, dir, head.eye).expect("placed");
        assert!(wrap_deg(yaw - 150.0).abs() < 8.0 && (distance - 1.0).abs() < 0.2, "{yaw} {distance} {at:?}");
        // As read_names has it: the sighting carries the distance and the feet.
        let yaw_px = dir[0].atan2(dir[2]).to_degrees();
        let pitch = -(dir[1].atan2(dir[0].hypot(dir[2])).to_degrees());
        let tag = Tagged { yaw: yaw_px, pitch, rel: [0.7, 0.0, 0.0], position: lens, lens: Lens::Look, posed: Instant::now() };
        let img = vec![40u8; 1280 * 720 * 3];
        let lines = [vrc_players::ocr::OcrLine { text: "Ann".into(), bbox: [600.0, 350.0, 80.0, 20.0], confidence: 0.9 }];
        let read = read_names(&View::rgb(&img, 1280, 720), &lines, &["Ann".to_string()], &tag, &head, 60.0, Instant::now(), Some(&depth));
        let s = &read[0].sighting;
        assert!(wrap_deg(s.world_yaw - 150.0).abs() < 8.0 && s.distance_m.is_some_and(|d| (d - 1.0).abs() < 0.2) && s.feet.is_some(), "{s:?}");
        // No depth there: the 2.5 m guess, no distance.
        let none = read_names(&View::rgb(&img, 1280, 720), &lines, &["Ann".to_string()], &tag, &head, 60.0, Instant::now(), None);
        assert!(none[0].sighting.distance_m.is_none());
    }

    #[test]
    fn the_lens_rides_along_with_the_bot_since_its_pose() {
        // The front lens placed at t0 with the eyes at the origin; the bot
        // then walks 1 m/s along +z (heads read every 100 ms).
        let o = standing([0.0, 1.5, 0.0]);
        let t0 = Instant::now();
        let mut st = o.state.lk();
        st.heads.clear();
        for i in 0..=6u64 {
            let eye = [0.0, 1.5, 0.1 * i as f32];
            st.note_head(Head { at: t0 + Duration::from_millis(100 * i), eye, yaw: 0.0, offset: 0.0 });
        }
        let pose = travel_pose([0.0, 1.5, 0.0], 0.0, &Travel::default());
        st.record(t0, pose, Lens::Front, Some([0.0, 1.5, 0.0]));
        // Between reads, and ahead of the last (at most HEAD_AHEAD).
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        assert!(close(st.eye_at(at(250)).unwrap()[2], 0.25, 1e-3));
        assert!(close(st.eye_at(at(700)).unwrap()[2], 0.7, 1e-3));
        assert!(close(st.eye_at(at(5000)).unwrap()[2], 0.6 + 0.5, 1e-3));
        drop(st);
        // A frame grabbed at 450 ms: the lens is where the Pose put it, 0.45
        // m on (rotation as sent), not where the Pose said.
        let tag = o.lens_at(at(450)).unwrap();
        let (lens, eye) = o.state.lk().lens_in_world(&tag, at(450)).unwrap();
        assert!(close(lens[2], pose.position[2] + 0.45, 1e-3) && close(lens[0], pose.position[0], 1e-4) && close(eye[2], 0.45, 1e-3), "{lens:?}");
        assert!(close(tag.yaw, 0.0, 1e-4));
    }

    #[test]
    fn a_held_lens_is_read_only_once_settled() {
        let s = OrbitSettings::default();
        let ms = Duration::from_millis;
        // Settled: the same Pose at the grab, 150 ms old, like the frame
        // before.
        assert!(settled(true, ms(200), Some(2.0), &s));
        // Too young, the Pose changed by the grab, nothing to compare yet,
        // or the view still moving.
        assert!(!settled(true, ms(100), Some(2.0), &s));
        assert!(!settled(false, ms(400), Some(2.0), &s));
        assert!(!settled(true, ms(400), None, &s));
        assert!(!settled(true, ms(400), Some(30.0), &s));
        // The image check off: age and the Pose only.
        let off = OrbitSettings { settle_diff: 0.0, ..OrbitSettings::default() };
        assert!(settled(true, ms(200), None, &off));
        // A Pose settle_sure_ms old has taken, however the frames change
        // (riding along with a walk, a sign's video): read (D40).
        assert!(settled(true, ms(600), Some(30.0), &s) && settled(true, ms(2000), None, &s));
        assert!(!settled(false, ms(2000), Some(30.0), &s));
        // Thumbnails: alike frames differ little, a turned view much.
        let (w, h) = (320, 180);
        let a: Vec<u8> = (0..w * h * 3).map(|k| ((k / 3) % w * 255 / w) as u8).collect();
        let mut b = a.clone();
        b[0] = 255;
        let c: Vec<u8> = (0..w * h * 3).map(|k| (255 - (k / 3) % w * 255 / w) as u8).collect();
        let (ta, tb, tc) = (thumbnail(&a, w, h), thumbnail(&b, w, h), thumbnail(&c, w, h));
        assert!(thumb_diff(&ta, &tb) < 1.0 && thumb_diff(&ta, &tc) > 30.0);
    }

    #[test]
    fn the_default_orbit_steps_small() {
        // 3 s a turn at 30 poses a second: 4 degrees (5 cm at 0.7 m) a pose,
        // about a game frame each.
        let s = OrbitSettings::default();
        let step = 360.0 / s.period_s / s.rate_hz;
        assert!(close(step, 4.0, 1e-4));
        assert!(step.to_radians() * s.radius_m < 0.06);
        // With a 60-degree-high 16:9 view (91 degrees wide), a plate stays
        // in view about 0.76 s a turn: three reads at 4 a second.
        let wide = 2.0 * (16.0 / 9.0 * 30f32.to_radians().tan()).atan().to_degrees();
        let in_view = wide / 360.0 * s.period_s;
        assert!(close(wide, 91.5, 0.1) && in_view * s.ocr_hz >= 3.0, "{wide} {in_view}");
    }

    fn voice(segment: u64, yaw: f32, open: bool, t1: Instant) -> Voice {
        Voice { segment, open, t0: t1, t1, yaw, mirror: None, from: "direction" }
    }

    #[test]
    fn the_lens_turns_to_a_voice_and_holds_till_it_is_over() {
        let s = AttendSettings::default();
        let t = Instant::now();
        let at = |ms: u64| t + Duration::from_millis(ms);
        // A speech starts 40 degrees right (tracking), the tracking frame 10
        // degrees off the world's: the lens turns to 50 (world).
        let (a, handled, aim) = decide(None, 0, Some(voice(1, 40.0, true, t)), 10.0, false, t, &s);
        let a1 = a.unwrap();
        assert!(aim && handled == 1 && close(a1.world_yaw, 50.0, 1e-3) && a1.until.is_none());
        // Going on: held, not aimed again.
        let (a, _, aim) = decide(a, 1, Some(voice(1, 41.0, true, at(200))), 10.0, true, at(200), &s);
        assert!(!aim && a.is_some());
        // Another speech from about the same way: held for it too.
        let (a, handled, aim) = decide(a, 1, Some(voice(2, 45.0, true, at(300))), 10.0, true, at(300), &s);
        assert!(!aim && handled == 2 && a.unwrap().segment == 2);
        // One from far off, too soon after the aim: later; then aimed.
        let (a, handled, aim) = decide(a, 2, Some(voice(3, -90.0, true, at(400))), 10.0, true, at(400), &s);
        assert!(!aim && handled == 2);
        let (a, handled, aim) = decide(a, handled, Some(voice(3, -90.0, true, at(600))), 10.0, true, at(600), &s);
        assert!(aim && handled == 3 && close(a.unwrap().world_yaw, 280.0, 1e-3));
        // It ends at 2.0 s: held until a second after.
        let (a, _, _) = decide(a, 3, Some(voice(3, -90.0, false, at(2000))), 10.0, true, at(2100), &s);
        assert_eq!(a.unwrap().until, Some(at(3000)));
        let (a, _, _) = decide(a, 3, Some(voice(3, -90.0, false, at(2000))), 10.0, true, at(2900), &s);
        assert!(a.is_some());
        let (a, _, aim) = decide(a, 3, Some(voice(3, -90.0, false, at(2000))), 10.0, true, at(3000), &s);
        assert!(a.is_none() && !aim);
        // The bot's own echo (no voice at all): nothing to turn to.
        assert_eq!(decide(None, 3, None, 0.0, false, at(3100), &s), (None, 3, false));
        // Disabled turns are not the decider's: a speech already dealt
        // with is not turned to again.
        assert!(decide(None, 3, Some(voice(3, 0.0, true, at(3200))), 0.0, false, at(3200), &s).0.is_none());
    }

    #[test]
    fn no_lit_plate_that_way_turns_to_the_mirror() {
        let s = AttendSettings::default();
        let t = Instant::now();
        let at = |ms: u64| t + Duration::from_millis(ms);
        let v = Voice { mirror: Some(150.0), ..voice(1, 30.0, true, t) };
        let (a, _, _) = decide(None, 0, Some(v), 0.0, false, t, &s);
        // Lit within 0.4 s: it stays.
        let (lit, _, aim) = decide(a, 1, Some(v), 0.0, true, at(500), &s);
        assert!(!aim && close(lit.unwrap().world_yaw, 30.0, 1e-3));
        // Nothing lit: the mirror, once.
        let (m, _, aim) = decide(a, 1, Some(v), 0.0, false, at(300), &s);
        assert!(!aim && m == a);
        let (m, _, aim) = decide(a, 1, Some(v), 0.0, false, at(450), &s);
        let m1 = m.unwrap();
        assert!(aim && close(m1.world_yaw, 150.0, 1e-3) && m1.mirror.is_none() && m1.aimed_at == at(450));
        let (_, _, aim) = decide(m, 1, Some(v), 0.0, false, at(1000), &s);
        assert!(!aim);
        // Pinned on a placed player since, elsewhere: their way.
        let pinned = Voice { from: "candidate", mirror: None, ..voice(1, -100.0, true, at(1200)) };
        let (p, _, aim) = decide(m, 1, Some(pinned), 0.0, false, at(1200), &s);
        assert!(aim && close(p.unwrap().world_yaw, 260.0, 1e-3) && p.unwrap().from == "candidate");
    }

    #[test]
    fn walking_the_travel_lens_gives_way_only_to_a_voice_far_off() {
        let o = orbiting([0.0, 1.5, 0.0]);
        // Off by default: the lens leaves voices be.
        let t = Instant::now();
        assert!(o.attend_tick(Some(voice(9, 90.0, true, t)), t, false).is_none());
        o.state.lk().settings.attend_onset = true;
        o.gate("/input/Vertical", 0.6); // going yaw 0
        // 40 degrees off the way: the travel lens stays.
        assert!(o.attend_tick(Some(voice(1, 40.0, true, t)), t, true).is_none());
        // 90 off: turned to it.
        let (aim, way) = o.attend_tick(Some(voice(2, 90.0, true, t)), t, true).unwrap();
        assert!(aim && close(way, 90.0, 1e-3));
        // Once it is the lens, the push's travel lens leaves it be.
        let eye = [0.0, 1.5, 0.0];
        let mut st = o.state.lk();
        st.record(t, attend_pose(eye, 90.0, &AttendSettings::default()), Lens::Attend, Some(eye));
        assert!(st.travel_due(t, -1.0).is_none());
        st.attending = None;
        assert!(st.travel_due(t, -1.0).is_some());
    }

    #[test]
    fn the_lens_turned_to_a_voice_is_near_the_head() {
        let p = attend_pose([1.0, 1.5, 2.0], 90.0, &AttendSettings::default());
        assert!(close(p.position[0], 1.3, 1e-4) && close(p.position[1], 1.5, 1e-4) && close(p.position[2], 2.0, 1e-4), "{p:?}");
        assert!(close(p.yaw, 90.0, 1e-3) && close(p.pitch, 5.0, 1e-4));
    }

    /// The camera open, the head at `eye` facing yaw 0, no lens yet.
    fn standing(eye: [f32; 3]) -> Orbit {
        let o = orbit();
        let now = Instant::now();
        let mut st = o.state.lk();
        st.camera = Some((now, Some(usercam::MODE_STREAM)));
        st.head = Some(Head { at: now, eye, yaw: 0.0, offset: 0.0 });
        drop(st);
        o
    }

    #[test]
    fn a_sweep_waits_for_the_bot_to_stand_and_gives_way_to_a_move() {
        let eye = [0.0, 1.5, 0.0];
        let o = standing(eye);
        let t = Instant::now();
        {
            let mut st = o.state.lk();
            // Just stopped: the orbit waits resume_after_s, a sweep not.
            st.released_at = Some(t);
            assert!(st.moving(t));
        }
        o.sweep("follow", Some(90.0), Some("Ann")).unwrap();
        {
            let st = o.state.lk();
            assert!(st.sweeping(t) && !st.moving(t));
            assert_eq!(st.seek.as_deref(), Some("Ann"));
            // Not begun within SWEEP_START_WITHIN: dropped.
            assert!(!st.sweeping(t + SWEEP_START_WITHIN + Duration::from_millis(10)));
        }
        // A push ends it at once (the gate), and says so.
        o.gate("/input/Vertical", 0.5);
        assert!(!o.sweeping());
        assert_eq!(o.sweep_end().unwrap().0, "move");
        assert!(o.state.lk().seek.is_none());
        // Moving: no sweep.
        assert!(o.sweep("idle", None, None).is_err());
        o.gate("/input/Vertical", 0.0);
        o.sweep("idle", None, None).unwrap();
        o.end_sweep("found");
        assert_eq!(o.sweep_end().unwrap().0, "found");
        // A voice turned to: no sweep (the attend lens owns the lens).
        o.state.lk().attending = Some(Attending { segment: 1, world_yaw: 90.0, mirror: None, from: "direction", since: t, aimed_at: t, until: None });
        assert!(o.sweep("idle", None, None).is_err());
    }

    /// Decision D41: the views are fixed; from where someone was last seen
    /// they go out either way by turns, else round.
    #[test]
    fn the_sweep_views_go_out_from_where_they_were() {
        let out: Vec<f32> = (0..6).map(|k| ring_yaw(90.0, k, 6, true)).collect();
        for (got, want) in out.iter().zip([90.0, 150.0, 30.0, 210.0, 330.0, 270.0]) {
            assert!(close(*got, want, 1e-3), "{out:?}");
        }
        let round: Vec<f32> = (0..6).map(|k| ring_yaw(350.0, k, 6, false)).collect();
        for (got, want) in round.iter().zip([350.0, 50.0, 110.0, 170.0, 230.0, 290.0]) {
            assert!(close(*got, want, 1e-3), "{round:?}");
        }
        // Six views of the lens (47 degrees high, 16:9: about 75 wide) 60
        // apart: nothing between them unseen.
        let wide = 2.0 * ((DEFAULT_FOV_DEG / 2.0).to_radians().tan() * 16.0 / 9.0).atan().to_degrees();
        assert!(wide > 360.0 / SnapSettings::default().views as f32 + 10.0, "{wide}");
    }

    /// Decision D41: a view's Pose at once, the next as soon as a frame of
    /// it was taken (or after `max_ms`), and over after the last.
    #[test]
    fn the_sweep_goes_view_by_view_as_each_is_taken() {
        let eye = [0.0, 1.5, 0.0];
        let o = standing(eye);
        let s = SnapSettings::default();
        let t = Instant::now();
        let ms = |m: u64| t + Duration::from_millis(m);
        o.sweep("follow", Some(90.0), Some("Ann")).unwrap();
        let mut st = o.state.lk();
        let head = st.head.unwrap();
        // The first view on the first tick: where they were.
        let first = st.snap_next(&head, t).expect("the first view at once");
        assert!(close(first.yaw, 90.0, 1e-3) && first == look_pose(eye, 90.0, &st.settings.look));
        st.record(t, first, Lens::Snap, Some(eye));
        // Waiting for its frame: nothing new.
        assert!(st.snap_next(&head, ms(50)).is_none());
        // Taken (a frame of an older Pose is not it): the next at once.
        assert!(!st.snap_taken(ms(1)));
        assert!(st.snap_taken(t));
        let second = st.snap_next(&head, ms(160)).expect("the next view once taken");
        assert!(close(second.yaw, 150.0, 1e-3));
        // Never taken: on after max_ms.
        assert!(st.snap_next(&head, ms(160 + s.max_ms - 10)).is_none());
        let third = st.snap_next(&head, ms(160 + s.max_ms)).expect("on after max_ms");
        assert!(close(third.yaw, 30.0, 1e-3));
        // The rest, each taken at once: then over ("done"), the idle lens.
        let mut now = 160 + s.max_ms;
        for _ in 3..s.views {
            let posed = st.sweep.unwrap().posed.unwrap();
            assert!(st.snap_taken(posed));
            now += 150;
            assert!(st.snap_next(&head, ms(now)).is_some());
        }
        assert!(st.sweeping(ms(now)));
        let posed = st.sweep.unwrap().posed.unwrap();
        st.snap_taken(posed);
        assert!(st.snap_next(&head, ms(now + 150)).is_none());
        assert!(st.sweep.is_none() && !st.sweeping(ms(now + 150)));
        assert_eq!(st.sweep_end.unwrap().0, "done");
        // The whole sweep at about 150 ms a view: about a second.
        assert!(now + 150 < 2000, "{now}");
    }

    /// Decision D41: a view is read from its first frame that shows it.
    #[test]
    fn a_sweep_view_is_taken_from_its_first_changed_frame() {
        let s = SnapSettings::default();
        let ms = Duration::from_millis;
        // Too young, whatever it shows.
        assert!(!snap_ready(ms(60), Some(40.0), &s));
        // Changed from the frame before the Pose: at once.
        assert!(snap_ready(ms(s.min_ms), Some(40.0), &s));
        // Like it still (the old view, or a blank wall): only once sure.
        assert!(!snap_ready(ms(200), Some(1.0), &s) && !snap_ready(ms(200), None, &s));
        assert!(snap_ready(ms(s.sure_ms), Some(1.0), &s) && snap_ready(ms(s.sure_ms), None, &s));
    }

    #[test]
    fn a_sweep_frame_is_tagged_with_its_view() {
        let eye = [0.0, 1.5, 0.0];
        let o = standing(eye);
        let t = Instant::now();
        let pose = look_pose(eye, 200.0, &LookLens::default());
        o.state.lk().record(t, pose, Lens::Snap, Some(eye));
        let tag = o.lens_at(t + Duration::from_millis(400)).expect("a snap view is held");
        assert!(tag.lens == Lens::Snap && tag.posed == t && close(tag.yaw, 200.0, 1e-3));
    }

    #[test]
    fn going_round_the_travel_lens_is_aimed_again_less_often() {
        let o = standing([0.0, 1.5, 0.0]);
        o.calm_travel(Duration::from_secs(2));
        let st = o.state.lk();
        assert!(st.calm_until.is_some_and(|c| c > Instant::now()));
        assert!(CALM_REAIM_DEG > Travel::default().reaim_deg && CALM_REAIM_EVERY_MS > Travel::default().reaim_every_ms);
    }

    #[test]
    fn standing_the_front_lens_rests_and_aims_again_only_on_a_drift() {
        assert_eq!(OrbitSettings::default().idle, Idle::Front);
        let eye = [0.0, 1.5, 0.0];
        let o = standing(eye);
        let t = Instant::now();
        let head = o.state.lk().head.unwrap();
        // The first tick places it: the travel lens's place, the heading.
        assert!(o.front(&mut o.state.lk(), &head, t));
        {
            let st = o.state.lk();
            let last = st.last().unwrap();
            assert_eq!(last.lens, Lens::Front);
            assert_eq!(last.pose, travel_pose(eye, 0.0, &Travel::default()));
            assert!(st.active && st.flying_maybe_on && st.front_poses == 1);
        }
        // Then nothing: no stream of Poses.
        for ms in [40u64, 500, 3000] {
            assert!(!o.front(&mut o.state.lk(), &head, t + Duration::from_millis(ms)));
        }
        // The head turned 20 degrees: within reaim_deg, it stays.
        let later = t + Duration::from_secs(2);
        let turned = |yaw: f32| Head { at: later, yaw, ..head };
        o.state.lk().head = Some(turned(20.0));
        assert!(!o.front(&mut o.state.lk(), &turned(20.0), later));
        // 40 degrees: aimed again, that way.
        o.state.lk().head = Some(turned(40.0));
        assert!(o.front(&mut o.state.lk(), &turned(40.0), later));
        assert!(close(o.state.lk().last().unwrap().pose.yaw, 40.0, 1e-3));
        // ... but not again within reaim_every_ms.
        o.state.lk().head = Some(turned(100.0));
        assert!(!o.front(&mut o.state.lk(), &turned(100.0), later + Duration::from_millis(300)));
        // After a voice or a shot, it comes back at once.
        let now = later + Duration::from_millis(400);
        o.state.lk().record(now, attend_pose(eye, 200.0, &AttendSettings::default()), Lens::Attend, Some(eye));
        assert!(o.front(&mut o.state.lk(), &turned(100.0), now));
        assert_eq!(o.state.lk().last().unwrap().lens, Lens::Front);
        // Frames under it are tagged (the names are read from it).
        assert_eq!(o.lens_at(now + Duration::from_secs(5)).unwrap().lens, Lens::Front);
    }

    #[test]
    fn a_push_ahead_from_the_front_lens_only_waits_for_flying_off() {
        let o = standing([0.0, 1.5, 0.0]);
        let head = o.state.lk().head.unwrap();
        assert!(o.front(&mut o.state.lk(), &head, Instant::now()));
        // Ahead: the lens already looks that way, so no new Pose; flying
        // may be on (the Pose just went out), so the push waits for it off.
        let t = Instant::now();
        o.gate("/input/Vertical", 0.6);
        assert!(t.elapsed() >= Duration::from_millis(150));
        let st = o.state.lk();
        assert!(st.travel_poses == 0 && st.parks == 1 && !st.flying_maybe_on);
        drop(st);
        o.gate("/input/Vertical", 0.0);
        // Flying off already: a push to the side aims the travel lens.
        let t = Instant::now();
        o.gate("/input/Horizontal", 0.5);
        assert!(t.elapsed() >= Duration::from_millis(150));
        assert_eq!(o.state.lk().travel_poses, 1);
    }

    #[test]
    fn a_look_turns_the_lens_a_moment_then_the_front_comes_back() {
        let eye = [0.0, 1.5, 0.0];
        let o = standing(eye);
        o.look_at(90.0, Duration::from_millis(800)).unwrap();
        let now = Instant::now();
        {
            let st = o.state.lk();
            let last = st.last().unwrap();
            assert_eq!(last.lens, Lens::Look);
            // Behind and over the head (not out in the face of someone
            // close), looking that way.
            let pose = look_pose(eye, 90.0, &LookLens::default());
            assert_eq!(last.pose, pose);
            assert!(close(pose.position[0], -0.15, 1e-4) && close(pose.position[1], 1.75, 1e-4) && close(pose.yaw, 90.0, 1e-3) && close(pose.pitch, 2.0, 1e-4), "{pose:?}");
            assert!(st.looking(now) && !st.looking(now + Duration::from_secs(1)) && st.looks == 1);
        }
        // Frames under it are tagged: a name read there has a bearing.
        assert_eq!(o.lens_at(now).unwrap().lens, Lens::Look);
        // Over: the front lens is due again.
        let head = o.state.lk().head.unwrap();
        assert!(o.state.lk().front_due(now + Duration::from_secs(1), &head, 0.0).is_some());
        // Closed camera, or the lens off: no look.
        o.state.lk().camera = Some((now, Some(usercam::MODE_CLOSED)));
        assert!(o.look_at(0.0, Duration::from_secs(1)).is_err());
    }

    #[test]
    fn a_name_already_read_that_way_needs_no_look() {
        let o = standing([0.0, 1.5, 0.0]);
        let seen = |name: &str, yaw: f32| NameSighting {
            at: Instant::now(),
            name: name.into(),
            world_yaw: yaw,
            tracking_yaw: yaw,
            bearing_deg: yaw,
            elevation_deg: 5.0,
            lens: Lens::Front,
            glow: None,
            bbox: [0.0; 4],
            feet: None,
            distance_m: None,
            ray_from: [0.0; 3],
            ray_dir: [0.0, 0.0, 1.0],
        };
        o.names.lk().extend([seen("Ann", 10.0), seen("Bob", 350.0)]);
        let n = o.name_toward(355.0, 15.0, Duration::from_secs(1), Duration::from_millis(200)).unwrap().unwrap();
        assert_eq!(n.name, "Bob");
        assert_eq!(o.state.lk().looks, 0);
        // Nobody read that way: the lens looks, nothing read in time.
        assert!(o.name_toward(180.0, 15.0, Duration::from_secs(1), Duration::from_millis(200)).unwrap().is_none());
        assert_eq!(o.state.lk().looks, 1);
    }

    /// Decision D40: following and standing (or just stopped), the front
    /// lens faces the target, is aimed again when they drift off it, and
    /// is placed again when left alone `follow_stale_s`.
    #[test]
    fn following_and_standing_the_lens_faces_the_target_and_is_aimed_again() {
        let eye = [0.0, 1.5, 0.0];
        let o = standing(eye);
        let t = Instant::now();
        let head = o.state.lk().head.unwrap();
        let every = Duration::from_millis(Travel::default().reaim_every_ms);
        // Not following: the target's way is not taken.
        o.aim_at(60.0);
        assert!(o.front(&mut o.state.lk(), &head, t));
        assert!(close(o.state.lk().last().unwrap().pose.yaw, 0.0, 1e-3));
        // Following, the target 60 degrees right: the lens there at once.
        o.set_following(true);
        o.aim_at(60.0);
        let t1 = t + every;
        assert!(o.front(&mut o.state.lk(), &head, t1));
        {
            let st = o.state.lk();
            let last = st.last().unwrap();
            assert!(last.lens == Lens::Front && close(last.pose.yaw, 60.0, 1e-3) && st.target_poses == 1, "{last:?}");
        }
        // They stepped 10 degrees: it stays; 40: aimed again (not sooner
        // than reaim_every_ms).
        o.aim_at(70.0);
        assert!(!o.front(&mut o.state.lk(), &head, t1 + every));
        o.aim_at(100.0);
        assert!(!o.front(&mut o.state.lk(), &head, t1 + Duration::from_millis(200)));
        let t2 = t1 + every + Duration::from_millis(10);
        o.state.lk().aim = Some((100.0, t2));
        assert!(o.front(&mut o.state.lk(), &head, t2));
        assert!(close(o.state.lk().last().unwrap().pose.yaw, 100.0, 1e-3));
        // Nobody placed them for a while: the heading again.
        let t3 = t2 + AIM_FRESH + every + Duration::from_millis(10);
        assert!(o.front(&mut o.state.lk(), &head, t3));
        assert!(close(o.state.lk().last().unwrap().pose.yaw, 0.0, 1e-3));
        // Left alone follow_stale_s, the same way: placed again all the same.
        let stale = Duration::from_secs_f32(OrbitSettings::default().follow_stale_s);
        assert!(!o.front(&mut o.state.lk(), &head, t3 + stale - Duration::from_millis(100)));
        assert!(o.front(&mut o.state.lk(), &head, t3 + stale));
        assert_eq!(o.state.lk().stale_poses, 1);
        // The follow over: the target forgotten.
        o.set_following(false);
        assert!(o.state.lk().aim.is_none() && !o.state.lk().following);
    }

    /// Decision D40: every input let go, the bot not stood long yet (the
    /// follower's legs holding): the front lens may be placed; pushing, not.
    #[test]
    fn just_stopped_the_front_lens_may_be_placed_the_orbit_waits() {
        let o = standing([0.0, 1.5, 0.0]);
        o.gate("/input/Vertical", 0.6);
        let now = Instant::now();
        assert!(!o.state.lk().may_front(now), "pushing");
        o.gate("/input/Vertical", 0.0);
        let st = o.state.lk();
        assert!(st.moving(now) && st.may_front(now), "let go, coasting");
        drop(st);
        o.state.lk().settings.idle = Idle::Orbit;
        assert!(!o.state.lk().may_front(Instant::now()), "the orbit waits resume_after_s");
    }

    /// Decision D40: however a sweep ends, the lens goes back to the front
    /// (or the travel lens): a push, the finder giving up, its turn done.
    #[test]
    fn a_sweep_abort_returns_the_lens() {
        let eye = [0.0, 1.5, 0.0];
        for how in ["move", "stopped", "found", "done"] {
            let o = standing(eye);
            o.sweep("follow", Some(135.0), Some("Ann")).unwrap();
            let t = Instant::now();
            {
                // The sweep under way: its first view.
                let mut st = o.state.lk();
                let head = st.head.unwrap();
                let pose = st.snap_next(&head, t).unwrap();
                st.record(t, pose, Lens::Snap, Some(eye));
                st.active = true;
            }
            let after = match how {
                "move" => {
                    o.gate("/input/Vertical", 0.5);
                    let st = o.state.lk();
                    assert_eq!(st.last().unwrap().lens, Lens::Travel, "a push: the travel lens");
                    assert!(close(st.last().unwrap().pose.yaw, 0.0, 1e-3));
                    drop(st);
                    o.gate("/input/Vertical", 0.0);
                    Instant::now()
                }
                "done" => t + o.sweep_budget() + Duration::from_secs(5),
                why => {
                    o.end_sweep(why);
                    Instant::now()
                }
            };
            let st = o.state.lk();
            assert!(!st.sweeping(after), "{how}");
            assert!(st.may_front(after), "{how}: the standing lens may go");
            let head = st.head.unwrap();
            // Front due (after the sweep's view), or the travel lens
            // already there.
            assert!(st.front_due(after, &head, 0.0).is_some() || st.last().unwrap().lens == Lens::Travel, "{how}");
            if how != "done" {
                assert_eq!(st.sweep_end.unwrap().0, how);
            }
        }
    }

    #[test]
    fn reads_under_a_look_are_counted() {
        let o = standing([0.0, 1.5, 0.0]);
        let t = Instant::now();
        {
            let mut s = o.sights.lk();
            s.log.push_back((t, Lens::Front, t - Duration::from_secs(9)));
            s.log.push_back((t, Lens::Look, t + Duration::from_millis(10)));
            s.log.push_back((t, Lens::Look, t + Duration::from_millis(10)));
            s.log.push_back((t, Lens::Look, t - Duration::from_secs(5)));
        }
        assert_eq!(o.reads_since(t, Lens::Look), 2);
        assert_eq!(o.reads_since(t, Lens::Front), 0);
    }

    #[test]
    fn settings_change_by_parts_and_are_checked() {
        let s = OrbitSettings::default();
        let m = s.merged(&json!({"radius_m": 1.0, "travel": {"reaim_deg": 45}})).unwrap();
        assert!(close(m.radius_m, 1.0, 1e-6) && close(m.travel.reaim_deg, 45.0, 1e-6) && close(m.travel.up_m, 0.35, 1e-6) && m.on);
        assert!(!s.merged(&json!(false)).unwrap().on);
        assert!(s.merged(&json!({"radius_m": 0.1})).is_err());
        assert!(s.merged(&json!({"rate_hz": 100})).is_err());
        assert!(s.merged(&json!("fast")).is_err());
        // Old files without `orbit` read as the defaults.
        let old: crate::usercam::Settings = serde_json::from_str(r#"{"keep_open": true, "stow": true}"#).unwrap();
        assert_eq!(old.orbit, OrbitSettings::default());
        // The idle lens: front by default, the orbit when asked.
        assert_eq!(s.merged(&json!({"idle": "orbit"})).unwrap().idle, Idle::Orbit);
        assert!(s.merged(&json!({"idle": "spin"})).is_err());
        assert_eq!(s.merged(&json!({"snap": {"views": 8}})).unwrap().snap.views, 8);
        assert!(s.merged(&json!({"snap": {"views": 1}})).is_err());
        assert!(s.merged(&json!({"snap": {"sure_ms": 50}})).is_err());
        let older: crate::usercam::Settings = serde_json::from_str(r#"{"orbit": {"on": true, "radius_m": 0.9}}"#).unwrap();
        assert_eq!(older.orbit.idle, Idle::Front);
    }
}
