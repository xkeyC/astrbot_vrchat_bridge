//! Motion programs on the full body (`/v1/motion`): clips (`vrc_vr::motion`,
//! made by `tools/motion`) played one after another, each mirrored, faster
//! or slower, looped for a while, in place or moving the body.
//!
//! A program runs on a thread of its own that holds the headset (nothing
//! else turns the head or walks meanwhile); the animation thread plays it
//! each tick (`Program::tick`) on the trackers, the headset and the hands.
//! When it ends, the body stays where the program left it: the owner's head
//! and facing are moved there. A program whose last clip holds a posture
//! (lying, sitting) stays in it until stopped (it gets up first: the
//! clip's exit) or until the next program goes on from it.
//!
//! Clips live in `motions/` next to the token (`~/.config/vrc-bridge`).

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use serde::Deserialize;
use serde_json::{json, Value};
use vrc_vr::anim::{self, AnimParams};
use vrc_vr::motion::{blend, place, Attached, Body, Clip, Placed, Root, Stand};
use vrc_vr::pose::Pose;
use vrc_vr::remote::FLOOR_Y;
use vrc_vr::trackers::{self, Part};

use crate::bridge::Bridge;
use crate::Lock;

/// The eyes stand this far (stature units) ahead of the hips' plumb line
/// (the canonical skeleton's, `tools/motion/skeleton.py`).
pub const EYES_AHEAD: f32 = 0.045;
/// From one step into the next, unless a step says (seconds).
const STEP_FADE: f32 = 0.6;
/// Back to standing at the end of a program (seconds).
const OUT_FADE: f32 = 3.0;
/// Into a posture's exit (rolling onto the back, getting up).
const EXIT_FADE: f32 = 1.5;
/// A looped clip without a length plays this long (seconds).
const LOOP_SECONDS: f32 = 5.0;
/// A program at most (seconds).
const MAX_SECONDS: f32 = 600.0;

/// The clips on disk.
pub struct Library {
    dir: PathBuf,
    clips: Mutex<BTreeMap<String, Arc<Clip>>>,
}

impl Library {
    pub fn new(dir: PathBuf) -> Library {
        let lib = Library { dir, clips: Mutex::new(BTreeMap::new()) };
        if let Err(e) = lib.reload() {
            tracing::warn!("motions: {e:#}");
        }
        lib
    }

    /// Reads every `*.json` of the directory again.
    pub fn reload(&self) -> Result<Vec<String>> {
        let mut clips = BTreeMap::new();
        let entries = std::fs::read_dir(&self.dir).with_context(|| format!("no motions at {}", self.dir.display()))?;
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|e| e == "json") {
                match Clip::load(&path) {
                    Ok(c) => {
                        let name = path.file_stem().unwrap().to_string_lossy().into_owned();
                        clips.insert(name, Arc::new(c));
                    }
                    Err(e) => tracing::warn!("motions: {e:#}"),
                }
            }
        }
        let names = clips.keys().cloned().collect();
        *self.clips.lk() = clips;
        Ok(names)
    }

    pub fn get(&self, name: &str) -> Option<Arc<Clip>> {
        self.clips.lk().get(name).cloned()
    }

    pub fn list(&self) -> Value {
        let clips = self.clips.lk();
        clips
            .iter()
            .map(|(name, c)| json!({"name": name, "seconds": (c.duration() * 10.0).round() / 10.0, "loop": c.looping}))
            .collect()
    }
}

/// One step of a program.
#[derive(Clone, Debug, Deserialize)]
pub struct Step {
    pub clip: String,
    /// The other side (a wave with the left hand).
    #[serde(default)]
    pub mirror: bool,
    #[serde(default = "one")]
    pub speed: f32,
    /// A looped clip: how long; a one-shot: cut short at.
    pub seconds: Option<f32>,
    /// Play where the body stands (its travel on the floor taken out),
    /// whatever the clip says.
    #[serde(default)]
    pub in_place: bool,
    /// Seconds to blend in from the pose before (rolling over from one way
    /// of lying to another wants long).
    pub fade: Option<f32>,
}

fn one() -> f32 {
    1.0
}

struct Prepared {
    clip: Clip,
    seconds: f32,
    speed: f32,
    root: Root,
    fade: f32,
}

/// Where a program left the body in a posture it holds (lying), for the
/// next one to go on from.
#[derive(Clone)]
pub struct Handover {
    stand: Stand,
    last: Body,
    curls: [f32; 10],
    exit: Option<String>,
    pub posture: Option<String>,
}

/// The way out of a held posture: its exit clip, and that one's while it
/// holds a posture too (lying on a side: onto the back, then getting up),
/// a held one only long enough to blend into.
fn exits(library: &Library, first: Option<String>) -> Vec<Prepared> {
    let mut out = Vec::new();
    let mut next = first;
    while let Some(name) = next.take() {
        if out.len() >= 4 {
            break;
        }
        let exit = Step { clip: name, mirror: false, speed: 1.0, seconds: None, in_place: false, fade: Some(EXIT_FADE) };
        let Ok(mut step) = prepare(library, &exit) else { break };
        if step.root == Root::Hold {
            step.seconds = EXIT_FADE + 0.4;
            next = step.clip.exit.clone();
        }
        out.push(step);
    }
    out
}

/// The program for a posture (`/v1/motion` with `posture`): standing
/// (none: stop), sitting, or lying `way` (back, left, right, front), from
/// the posture held now (`holding`).
pub fn posture_steps(posture: &str, way: Option<&str>, holding: Option<&str>) -> Result<Option<Vec<Step>>> {
    let step = |clip: &str, fade: Option<f32>| Step { clip: clip.into(), mirror: false, speed: 1.0, seconds: None, in_place: false, fade };
    Ok(match posture {
        "stand" => None,
        "sit" if holding == Some("sitting") => Some(vec![step("sitting", None)]),
        "sit" => Some(vec![step("sit_down", None), step("sitting", Some(1.2))]),
        "lie" => {
            let way = way.unwrap_or("back");
            if !["back", "left", "right", "front"].contains(&way) {
                bail!("way is back, left, right or front");
            }
            let lying = format!("lying_{way}");
            if holding == Some("lying") {
                Some(vec![step(&lying, Some(1.8))])
            } else {
                Some(vec![step("lie_down", None), step(&lying, Some(1.5))])
            }
        }
        _ => bail!("posture is stand, sit or lie"),
    })
}

/// A program being played.
pub struct Program {
    steps: Vec<Prepared>,
    index: usize,
    /// Seconds into the current step.
    t: f32,
    stand: Stand,
    attached: Attached,
    /// The pose the current step fades in from, and the last one played
    /// (with its finger curl changes).
    from: Body,
    last: Body,
    from_curls: [f32; 10],
    pub curls: [f32; 10],
    last_curls: [f32; 10],
    standing: Body,
    /// Fading out (to standing) since this many seconds.
    out: Option<f32>,
    /// The last step holds its posture until another program or a stop.
    pub holding: bool,
    pub done: bool,
    parts: Vec<Part>,
    params: AnimParams,
}

/// Eased in and out (0..1 to 0..1).
fn ease(w: f32) -> f32 {
    w * w * (3.0 - 2.0 * w)
}

/// What is attached to a body standing at `stand`.
pub fn attachments(stand: &Stand, parts: &[Part], params: &AnimParams) -> Attached {
    let hand = |side: f32| anim::hand_pose(params, stand.eyes, stand.yaw_deg, side, [0.0; 3], [0.0, 0.0, 0.0, 1.0]);
    Attached {
        trackers: trackers::standing(parts, stand.eyes, stand.yaw_deg, stand.floor_y),
        head: Pose::looking(stand.yaw_deg, 0.0, stand.eyes),
        left: hand(-1.0),
        right: hand(1.0),
    }
}

fn prepare(library: &Library, s: &Step) -> Result<Prepared> {
    let mut clip = (*library.get(&s.clip).with_context(|| format!("no motion {}", s.clip))?).clone();
    if s.mirror {
        clip = clip.mirrored();
    }
    if !(s.speed.is_finite() && (0.25..=4.0).contains(&s.speed)) {
        bail!("speed is 0.25-4");
    }
    let root = if s.in_place { Root::InPlace } else { clip.root };
    if root == Root::InPlace {
        clip = clip.in_place();
    }
    let natural = if clip.looping { LOOP_SECONDS } else { clip.duration() / s.speed };
    let seconds = s.seconds.filter(|x| x.is_finite() && *x > 0.0).unwrap_or(natural);
    let seconds = if clip.looping { seconds } else { seconds.min(natural) };
    let fade = s.fade.filter(|f| f.is_finite()).unwrap_or(STEP_FADE).clamp(0.0, 5.0);
    Ok(Prepared { clip, seconds, speed: s.speed, root, fade })
}

impl Program {
    /// The steps against `library`, for a body standing at `stand`, or going
    /// on from where a holding program left it (its posture's exit clip
    /// first, unless the steps stay in a held posture themselves).
    pub fn new(
        library: &Library,
        steps: &[Step],
        stand: Stand,
        parts: Vec<Part>,
        params: AnimParams,
        from: Option<Handover>,
    ) -> Result<Program> {
        if steps.is_empty() {
            bail!("no steps");
        }
        let mut prepared = Vec::new();
        if let Some(h) = &from {
            // Another clip of the same posture goes on from it; anything else
            // leaves it by its exits first.
            let first = library.get(&steps[0].clip);
            let stays = h.posture.is_some() && first.as_ref().is_some_and(|c| c.posture == h.posture);
            if !stays {
                prepared.extend(exits(library, h.exit.clone()));
            }
        }
        for s in steps {
            prepared.push(prepare(library, s)?);
        }
        let total: f32 = prepared.iter().map(|p| p.seconds).sum();
        if total > MAX_SECONDS {
            bail!("a program of at most {MAX_SECONDS} s");
        }
        let standing = prepared[0].clip.standing;
        let (stand, last, curls) = match &from {
            Some(h) => (h.stand, h.last, h.curls),
            None => (stand, standing, [0.0; 10]),
        };
        Ok(Program {
            attached: attachments(&stand, &parts, &params),
            steps: prepared,
            index: 0,
            t: 0.0,
            stand,
            from: last,
            last,
            from_curls: curls,
            curls,
            last_curls: curls,
            standing,
            out: None,
            holding: false,
            done: false,
            parts,
            params,
        })
    }

    /// In the last step, one that holds a posture (lying, sitting): the
    /// next program goes on from here (it need not have settled).
    pub fn holds(&self) -> bool {
        self.out.is_none() && !self.done && self.index + 1 == self.steps.len() && self.steps[self.index].root == Root::Hold
    }

    /// What a program holding a posture hands to the next one.
    pub fn handover(&self) -> Option<Handover> {
        self.holds().then(|| Handover {
            stand: self.stand,
            last: self.last,
            curls: self.curls,
            exit: self.steps.last().and_then(|s| s.clip.exit.clone()),
            posture: self.steps.last().and_then(|s| s.clip.posture.clone()),
        })
    }

    /// Ends the program: a held posture is left by its exit clip (getting
    /// up), anything else fades to standing where the body is now.
    pub fn stop(&mut self, library: &Library) {
        if self.out.is_some() || self.done {
            return;
        }
        let exit = self.steps.get(self.index).filter(|s| s.root == Root::Hold).and_then(|s| s.clip.exit.clone());
        let way_out = exits(library, exit);
        if !way_out.is_empty() {
            // Cut the program after the current step; the exits go on from
            // the pose now.
            self.steps.truncate(self.index + 1);
            self.steps.extend(way_out);
            self.from = self.last;
            self.from_curls = self.curls;
            self.index += 1;
            self.t = 0.0;
            self.holding = false;
            return;
        }
        self.out = Some(0.0);
    }

    /// Where the body stands (as the program has moved it).
    pub fn stand(&self) -> Stand {
        self.stand
    }

    pub fn status(&self) -> Value {
        let step = self.steps.get(self.index);
        json!({
            "step": self.index + 1,
            "of": self.steps.len(),
            "clip": step.map(|s| s.clip.name.clone()),
            "t_s": (self.t * 10.0).round() / 10.0,
            "holding": self.holding,
            "stopping": self.out.is_some(),
            "done": self.done,
        })
    }

    /// Advances `dt` seconds: what to send now.
    pub fn tick(&mut self, dt: f32) -> Placed {
        if self.done {
            return place(&self.stand, &self.standing, &self.standing, EYES_AHEAD, &self.attached);
        }
        if let Some(out) = self.out.as_mut() {
            *out += dt;
            let w = ease((*out / OUT_FADE).min(1.0));
            let body = blend(&self.last, &self.standing, w);
            let ending = *out >= OUT_FADE;
            // The curls fade from where the fade began (kept in `last_curls`).
            let curls = self.last_curls.map(|c| c * (1.0 - w));
            if ending {
                self.done = true;
            }
            self.curls = curls;
            return place(&self.stand, &self.standing, &body, EYES_AHEAD, &self.attached);
        }
        self.t += dt;
        let step = &self.steps[self.index];
        let last_step = self.index + 1 >= self.steps.len();
        if self.t >= step.seconds && !(last_step && step.root == Root::Hold) {
            // The step is over: the body stays where a travelling one left
            // it, and the next starts from there.
            if step.root == Root::Travel {
                let (end, turned) = step.clip.end_root();
                self.stand = moved(&self.stand, end, turned);
                self.attached = attachments(&self.stand, &self.parts, &self.params);
                self.last = self.standing;
            }
            let travelled = step.root == Root::Travel;
            self.from = self.last;
            self.from_curls = self.curls;
            self.index += 1;
            self.t = 0.0;
            if self.index >= self.steps.len() {
                // Out of steps: back to standing (instantly after a
                // travelling one, which ends standing anyway).
                self.index = self.steps.len() - 1;
                self.out = Some(if travelled { OUT_FADE } else { 0.0 });
                return self.tick(0.0);
            }
        }
        let step = &self.steps[self.index];
        if last_step && step.root == Root::Hold && self.t >= step.seconds {
            // Held: a loop goes on, a one-shot stays on its last frame.
            self.holding = true;
        }
        let mut body = step.clip.sample(self.t * step.speed);
        let mut curls = step.clip.sample_curls(self.t * step.speed);
        let w = if step.fade > 0.0 { ease((self.t / step.fade).min(1.0)) } else { 1.0 };
        if w < 1.0 {
            body = blend(&self.from, &body, w);
            curls = std::array::from_fn(|i| self.from_curls[i] + (curls[i] - self.from_curls[i]) * w);
        }
        self.last = body;
        self.curls = curls;
        self.last_curls = curls;
        place(&self.stand, &self.standing, &body, EYES_AHEAD, &self.attached)
    }

    /// A hand's controller at `pose` (`side`: 0 left, 1 right), its fingers
    /// relaxed and changed as the clip curls them.
    pub fn hand(&self, side: usize, pose: Pose) -> vrc_vr::remote::Controller {
        let curl = std::array::from_fn(|i| self.params.curl[i] + self.curls[side * 5 + i]);
        anim::hand(&self.params, pose, curl)
    }
}

/// Stops the program playing (`/v1/motion/stop`, or anything else that
/// moves the body).
pub static STOP: AtomicBool = AtomicBool::new(false);
/// A new program takes over from one holding a posture.
static HANDOVER: AtomicBool = AtomicBool::new(false);
/// One program at a time.
static RUNNING: Mutex<()> = Mutex::new(());

/// Stops the program playing and waits until it has let go of the headset
/// (a held posture gets up first).
/// The posture the program playing holds (lying, sitting), if it holds one.
pub fn holding(bridge: &Bridge) -> Option<String> {
    bridge.anim.motion.lk().as_ref().and_then(Program::handover).and_then(|h| h.posture)
}

pub fn stop_and_wait(bridge: &Bridge) {
    if bridge.anim.motion.lk().is_none() {
        return;
    }
    STOP.store(true, Ordering::SeqCst);
    let _wait = RUNNING.lk();
}

/// Plays `steps` (blocking until they are over, stopped, or hold a posture
/// another program takes over): holds the headset, plays through the
/// animation thread, then leaves the body where the program did.
pub fn play(bridge: &Arc<Bridge>, steps: Vec<Step>) -> Result<Value> {
    // A program holding a posture hands it over; any other is stopped.
    let holding = bridge.anim.motion.lk().as_ref().is_some_and(Program::holds);
    if holding {
        HANDOVER.store(true, Ordering::SeqCst);
    } else {
        STOP.store(true, Ordering::SeqCst);
    }
    let _one = RUNNING.lk();
    STOP.store(false, Ordering::SeqCst);
    HANDOVER.store(false, Ordering::SeqCst);
    let from = bridge.anim.motion.lk().as_ref().and_then(Program::handover);
    bridge.take_over();
    let result = {
        let mut vr = bridge.vr.lk();
        let r = run(bridge, &mut vr, &steps, from);
        if r.is_err() {
            vr.reset();
        }
        r
    };
    // Handed over: the next program has taken it on.
    if !matches!(&result, Ok(v) if v["handed_over"] == true) {
        *bridge.anim.motion.lk() = None;
    }
    bridge.idle_later();
    result
}

fn run(bridge: &Arc<Bridge>, vr: &mut crate::vr::VrCore, steps: &[Step], from: Option<Handover>) -> Result<Value> {
    let started = Instant::now();
    let stand = match &from {
        Some(h) => h.stand,
        None => {
            // Level, the body under the head.
            let yaw = vr.yaw;
            vr.face(yaw, 0.0)?;
            let eyes = vr.rig(&[])?.hmd.state.head.position;
            Stand { eyes, yaw_deg: yaw, floor_y: FLOOR_Y }
        }
    };
    let parts = bridge.anim.trackers.lk().parts().unwrap_or_default();
    let program = Program::new(&bridge.motions, steps, stand, parts, bridge.anim.params(), from)?;
    // Replaced in one go: the animation never sees no program between two.
    *bridge.anim.motion.lk() = Some(program);
    let mut stopped = false;
    loop {
        std::thread::sleep(Duration::from_millis(50));
        let mut m = bridge.anim.motion.lk();
        let Some(p) = m.as_mut() else { bail!("the program vanished") };
        if STOP.load(Ordering::SeqCst) && !stopped {
            stopped = true;
            p.stop(&bridge.motions);
        }
        if p.holds() && HANDOVER.load(Ordering::SeqCst) {
            // The next program goes on from here; nothing to commit.
            return Ok(json!({"ok": true, "handed_over": true, "took_s": secs(started)}));
        }
        if p.done {
            break;
        }
        if p.holding && !stopped {
            // Holding a posture: the program stays (the headset with it)
            // until stopped or handed over.
            continue;
        }
    }
    // The body stays where the program left it.
    let end = bridge.anim.motion.lk().as_ref().map(Program::stand).unwrap_or(stand);
    let rig = vr.rig(&[])?;
    rig.hmd.state.head.position = end.eyes;
    vr.face(end.yaw_deg, 0.0)?;
    vr.forget_places();
    Ok(json!({
        "ok": true,
        "stopped": stopped,
        "took_s": secs(started),
        "turned_deg": ((end.yaw_deg - stand.yaw_deg + 540.0).rem_euclid(360.0) - 180.0).round(),
    }))
}

fn secs(since: Instant) -> f64 {
    (since.elapsed().as_secs_f64() * 10.0).round() / 10.0
}

/// Where a body standing at `stand` stands after its hips travelled
/// `end` (body frame: x left, z ahead; stature units) and it turned
/// `turned` degrees to its right.
fn moved(stand: &Stand, end: [f32; 2], turned: f32) -> Stand {
    let s = stand.stature();
    let facing = Pose::looking(stand.yaw_deg, 0.0, [0.0; 3]);
    // The hips' plumb line now, from where the eyes were.
    let hips0 = facing.rotate([0.0, 0.0, EYES_AHEAD * s]);
    let travel = facing.rotate([-end[0] * s, 0.0, -end[1] * s]);
    let yaw = stand.yaw_deg + turned;
    let ahead = Pose::looking(yaw, 0.0, [0.0; 3]).rotate([0.0, 0.0, -EYES_AHEAD * s]);
    Stand {
        eyes: [
            stand.eyes[0] + hips0[0] + travel[0] + ahead[0],
            stand.eyes[1],
            stand.eyes[2] + hips0[2] + travel[2] + ahead[2],
        ],
        yaw_deg: (yaw + 540.0).rem_euclid(360.0) - 180.0,
        floor_y: stand.floor_y,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standing_still_moves_nowhere() {
        let s = Stand { eyes: [0.2, 1.56, -0.4], yaw_deg: 30.0, floor_y: -0.32 };
        let m = moved(&s, [0.0, 0.0], 0.0);
        assert!((0..3).all(|i| (m.eyes[i] - s.eyes[i]).abs() < 1e-5) && m.yaw_deg == 30.0);
    }

    #[test]
    fn a_half_turn_in_place_swaps_the_eyes_round_the_hips() {
        let s = Stand { eyes: [0.0, 1.56, 0.0], yaw_deg: 0.0, floor_y: -0.32 };
        let m = moved(&s, [0.0, 0.0], 180.0);
        let k = EYES_AHEAD * s.stature();
        // The hips are k behind the eyes (+Z); turned round, the eyes are k
        // further on, at +2k.
        assert!((m.eyes[2] - 2.0 * k).abs() < 1e-4 && m.eyes[0].abs() < 1e-4, "{:?}", m.eyes);
        assert!((m.yaw_deg.abs() - 180.0).abs() < 1e-3);
    }

    #[test]
    fn a_step_ahead_moves_the_eyes_ahead() {
        let s = Stand { eyes: [0.0, 1.56, 0.0], yaw_deg: 90.0, floor_y: -0.32 };
        // 0.5 statures ahead, facing +X.
        let m = moved(&s, [0.0, 0.5], 0.0);
        assert!((m.eyes[0] - 0.5 * s.stature()).abs() < 1e-4 && m.eyes[2].abs() < 1e-4, "{:?}", m.eyes);
    }
}
