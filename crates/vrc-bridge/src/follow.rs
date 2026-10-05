//! Following a player in the room (VR): their name tag says who and where.
//!
//! Two loops. The eyes (this thread): take the newest frame (looking ahead,
//! a little down), read the name tags (OCR) and place them by stereo
//! (`vrc-players`, the same logic as the model's look around); each sighting
//! of the target is a fix of where they stand. The legs (a thread of their
//! own, 25 times a second): integrate the avatar's own velocity (OSCQuery)
//! into an odometry, so a fix half a second old still says how far the
//! target is *now*; turn towards them (walking follows the head, the body
//! follows) and set the thumbstick from the distance left to the standing
//! distance, braking smoothly as it shrinks: a round of the eyes takes
//! 0.3-0.8 s, and at full stick the avatar runs 4 m/s.
//!
//! In the way (each view's points, along the way to the target, measured
//! from the ground just before it): whatever does not reach the eyes is
//! jumped first, with a run-up (user's rule). What reaches them, or what a
//! jump did not get past, is walked round by following it (a "bug"
//! algorithm, user's rule): keep it on one side (the side away from the
//! target's way round), each view taking the free heading nearest that
//! side, round corners, until the straight way to the target is open (no
//! wall up to the eyes; user's rule: keep checking while going round): out
//! of an enclosure or into one alike. Every GLANCE_EVERY the head turns to
//! the target's way for a view of it, as the walk along the wall faces
//! elsewhere. While following a wall the target may be out of sight: the
//! bot keeps going for where they were. Pushing without moving (stuck) jumps once,
//! then backs off and turns aside.
//!
//! Glances turn only the head: the stick is split into ahead and aside
//! (VRChat walks relative to the head), so the walk keeps its way.
//!
//! Lost, the bot stands and the eyes look around one view at a time,
//! a full turn one way, starting where the target was last seen; the first
//! view that finds them ends the search and the legs go that way. A turn
//! that finds nobody walks a while toward where they were, then turns
//! again.
//! Following ends only when told to stop or when they leave the room.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_players::names::match_score;
use vrc_stereo::{SgmParams, Stereo};
use vrc_vr::osc::Osc;
use vrc_vr::remote::FLOOR_Y;
use vrc_vr::scan::{self, angle_diff};
use vrc_vr::tap::EyeFrame;

use crate::bridge::Bridge;
use crate::Lock;

/// Standing distance (world metres) by default, and its limits.
const STAND_M: f32 = 1.5;
const MIN_STAND_M: f32 = 0.8;
const MAX_STAND_M: f32 = 7.0;
const CLOSER: f32 = 0.7;
const FARTHER: f32 = 1.4;
/// Walking starts this far past the standing distance (and goes on until
/// it is reached); backing off starts this far inside it, and goes on
/// until BACK_UNTIL inside.
const WALK_MARGIN: f32 = 0.3;
const BACK_MARGIN: f32 = 0.6;
const BACK_UNTIL: f32 = 0.15;
/// Something within this (world metres) straight ahead, nearer than the
/// target, stops the walk.
const OBSTACLE_M: f32 = 0.6;
/// Not seen for this long: stand and search.
const LOST_AFTER: Duration = Duration::from_millis(1500);
/// Out of the room for this long: gone.
const GONE_AFTER: Duration = Duration::from_secs(20);
const PITCH: f32 = -10.0;
/// Search views: offsets (degrees) from where the target was last seen.
const SEARCH: [f32; 6] = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0];
/// A search that found nobody: walk toward where they were this long.
const SEEK_FOR: Duration = Duration::from_secs(4);
/// Following a wall without seeing the target this long: stop and search.
const WALL_UNSEEN: Duration = Duration::from_secs(10);
/// A whole search found nothing: wait this long before the next.
const SEARCH_PAUSE: Duration = Duration::from_secs(2);

// The legs, measured on VRChat (VR mode, thumbstick forward): 0.3 runs
// 0.9 m/s, 0.6 runs 2.2, 1.0 runs 4.0; about 0.3 s to speed up or stop.
const TICK: Duration = Duration::from_millis(40);
const AXIS_DEAD: f32 = 0.1;
const SPEED_PER_AXIS: f32 = 4.44;
const MAX_SPEED: f32 = 3.0;
/// Deceleration planned for (m/s²): gentle, well inside what VRChat does.
const BRAKE: f32 = 1.5;
/// Input to motion, and the stop itself, as seconds of the current speed.
const LAG_S: f32 = 0.25;
/// Slower than this is not worth a step.
const MIN_SPEED: f32 = 0.3;
const BACK_AXIS: f32 = -0.3;
/// The head turns to the target at most this fast (degrees per second).
const TURN_RATE: f32 = 200.0;
/// Fixes predict the target's motion at most this far ahead.
const PREDICT_S: f32 = 1.0;
const ODOMETRY_KEPT: Duration = Duration::from_secs(4);
/// Obstacles: standing more than STEP_M over the ground before them (lower
/// ones are walked up), with BIN_POINTS points in a 10 cm bin and
/// OBSTACLE_POINTS in it and the next two, spanning MIN_SPAN_M of height
/// (NEAR_POINTS nearer than 0.9 m): stray points of stereo come alone, a
/// wall met at a slant spreads thin over several bins (25 points a bin
/// missed such walls); reaching
/// within EYE_MARGIN_M of the eyes, walked round, else jumped first, this
/// far before them, after a run-up at least RUN_UP_SPEED fast.
const STEP_M: f32 = 0.3;
const BIN_POINTS: usize = 3;
const OBSTACLE_POINTS: usize = 12;
const NEAR_POINTS: usize = 12;
const MIN_SPAN_M: f32 = 0.05;
const EYE_MARGIN_M: f32 = 0.25;
const JUMP_AT_M: f32 = 0.5;
/// A jump that left the same obstacle (within this) in the way failed:
/// walk round it for a while.
const SAME_PLACE_M: f32 = 0.7;
const NO_JUMP_FOR: Duration = Duration::from_secs(10);
const RUN_UP_SPEED: f32 = 1.6;
const JUMP_EVERY: Duration = Duration::from_millis(1500);
/// After a jump the run goes on this long (in the air, over it).
const JUMP_CARRY: Duration = Duration::from_millis(900);
/// A heading chosen by a view is followed this long after it.
const DETOUR_FOR: Duration = Duration::from_millis(1200);
/// Stuck: pushing (stick past STUCK_AXIS) yet slower than STUCK_SPEED for STUCK_FOR.
const STUCK_AXIS: f32 = 0.25;
const STUCK_SPEED: f32 = 0.15;
const STUCK_FOR: Duration = Duration::from_millis(600);
const ESCAPE_FOR: Duration = Duration::from_millis(500);
/// Views judge only ways within this of where they look (degrees).
const VIEW_HALF_DEG: f32 = 40.0;
const STEREO_THREADS: usize = 6;
/// Following a wall: headings tried this far apart (degrees), each side of
/// where it looks; a heading is free when nothing stands within FREE_M
/// along it. The wall is left once, along the way to the target, nothing up
/// to the eyes stands in it; given up after WALL_FOR. A
/// wall met again within SAME_SIDE_FOR of leaving one is followed on the
/// same side.
const WALL_STEP_DEG: f32 = 15.0;
const WALL_STEPS: i32 = 3;
const FREE_M: f32 = 1.2;
const WALL_FOR: Duration = Duration::from_secs(40);
const WALL_SPEED: f32 = 1.4;
const GLANCE_EVERY: Duration = Duration::from_millis(1500);
const SAME_SIDE_FOR: Duration = Duration::from_secs(8);

#[derive(Default)]
pub struct Follower {
    inner: Mutex<State>,
    /// The running follow's stop flag (a new one per follow).
    stop: Mutex<Arc<AtomicBool>>,
    /// The running follow's thread: the next waits for it to end.
    runner: Mutex<Option<std::thread::JoinHandle<()>>>,
}

#[derive(Default, Clone)]
struct State {
    target: String,
    state: &'static str,
    last_seen: Option<Instant>,
    distance: f32,
    obstacle: f32,
    stand: f32,
    hold: bool,
    moving: f32,
    running: bool,
    /// What the way ahead calls for: "", "detour", "jump", "blocked", "stuck".
    avoiding: &'static str,
    jumps: u32,
}

/// Where the target stood, in the odometry's frame (world metres, axes of
/// the tracking space: +x right, +z back).
#[derive(Clone, Copy)]
struct Fix {
    at: Instant,
    pos: [f32; 2],
    /// Their velocity (m/s), from the fixes before.
    vel: [f32; 2],
}

/// What the eyes and the legs share.
#[derive(Default)]
struct Track {
    /// The avatar's position (odometry) and its recent history.
    pos: [f32; 2],
    history: VecDeque<(Instant, [f32; 2])>,
    /// Where the head (and so the walk) points, degrees.
    facing: f32,
    target: Option<Fix>,
    /// Something in the way to the target, as the last view saw it.
    obstacle: Option<Obstacle>,
    /// A way round it: (until when, heading).
    detour: Option<(Instant, f32)>,
    /// The last jump: when, and where the obstacle stood (odometry).
    jumped: Option<(Instant, [f32; 2])>,
    /// Obstacles a jump did not get past (odometry), until when.
    no_jump: Vec<([f32; 2], Instant)>,
    /// Following a wall round to the target.
    wall: Option<Wall>,
    /// The side of the last wall followed, and when it was left.
    last_side: Option<(f32, Instant)>,
    /// Walking toward where the target was (not seen), until when.
    seek: Option<Instant>,
}

/// Following a wall: on which side it is kept (-1 left, +1 right), and
/// since when.
#[derive(Clone, Copy, Debug)]
struct Wall {
    side: f32,
    since: Instant,
}

/// Something in the way (world metres).
#[derive(Clone, Copy, Debug)]
struct Obstacle {
    /// The heading it lies along, and the odometry then.
    yaw: f32,
    then: [f32; 2],
    /// How far along the heading.
    distance: f32,
    /// To be jumped (else walked round).
    jump: bool,
}

impl Obstacle {
    /// How far ahead it is now, walking along `facing` from `pos`; `None`
    /// when walking another way.
    fn ahead(&self, pos: [f32; 2], facing: f32) -> Option<f32> {
        if angle_diff(facing, self.yaw).abs() > 25.0 {
            return None;
        }
        let (s, c) = self.yaw.to_radians().sin_cos();
        Some(self.distance - ((pos[0] - self.then[0]) * s - (pos[1] - self.then[1]) * c))
    }

    /// Where it stands (odometry).
    fn place(&self) -> [f32; 2] {
        let (s, c) = self.yaw.to_radians().sin_cos();
        [self.then[0] + s * self.distance, self.then[1] - c * self.distance]
    }
}

/// What stands in the way, from the view's points (world metres).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Blocker {
    /// How far ahead its front is.
    distance: f32,
    /// Its top over the ground before it.
    top: f32,
    /// It reaches (nearly) up to the eyes: no jumping that.
    tall: bool,
}

/// The first thing in the way along `yaw` from `eye`, in a corridor half a
/// metre wide out to 4 m; `None` when the way is clear. The ground is
/// followed along the corridor (slopes, steps), so a raised floor is not in
/// the way; something stands in the way when it rises more than a step
/// over the ground before it. Nearer than 0.9 m the bot's own body is in
/// view too (arms swinging forward, a gesture, props such as a weapon on the
/// back that reaches over the shoulder, all below about 1.1 m): there only
/// points from NEAR_UP of the eye height up count, so a wall (which reaches
/// that high) is seen however near; a low thing that near was seen from
/// farther, and the odometry counts down to it.
fn corridor(points: &[[f32; 3]], eye: [f32; 3], yaw: f32, metres: f32, floor: f32) -> Option<Blocker> {
    const BIN: f32 = 0.1;
    const FROM: f32 = 0.9;
    const NEAR_FROM: f32 = 0.35;
    const NEAR_UP: f32 = 0.85;
    const BINS: usize = 31;
    let (s, c) = yaw.to_radians().sin_cos();
    let eye_m = (eye[1] - floor) * metres;
    // Per 10 cm along the way: the heights of the points in it.
    let mut bins: Vec<Vec<(f32, f32)>> = vec![Vec::new(); BINS];
    for p in points {
        let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
        let (ahead, side, up) = ((dx * s - dz * c) * metres, (dx * c + dz * s) * metres, (p[1] - floor) * metres);
        if ahead > FROM && side.abs() < 0.25 && up < eye_m + 0.3 {
            let i = ((ahead - FROM) / BIN) as usize;
            if i < BINS {
                bins[i].push((ahead, up));
            }
        }
    }
    // Near: only what reaches up toward the eyes (a wall), not the body.
    let near: Vec<f32> = points
        .iter()
        .filter_map(|p| {
            let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
            let (ahead, side, up) = ((dx * s - dz * c) * metres, (dx * c + dz * s) * metres, (p[1] - floor) * metres);
            (ahead > NEAR_FROM && ahead <= FROM && side.abs() < 0.25 && up > NEAR_UP * eye_m && up < eye_m + 0.3).then_some(ahead)
        })
        .collect();
    if near.len() >= NEAR_POINTS {
        let distance = near.iter().copied().fold(f32::INFINITY, f32::min);
        return Some(Blocker { distance, top: eye_m, tall: true });
    }
    let mut ground = 0.0f32;
    for (i, bin) in bins.iter().enumerate() {
        if bin.is_empty() {
            continue;
        }
        let above: Vec<(f32, f32)> = bin.iter().copied().filter(|&(_, up)| up > ground + STEP_M).collect();
        // A wall met at a slant spreads over several bins, a few points
        // each; stray points come alone: count this bin and the next two,
        // and want some height to them.
        let window: Vec<f32> = bins[i..(i + 3).min(BINS)].iter().flatten().filter(|a| a.1 > ground + STEP_M).map(|a| a.1).collect();
        let span = window.iter().copied().fold(f32::NEG_INFINITY, f32::max) - window.iter().copied().fold(f32::INFINITY, f32::min);
        if above.len() >= BIN_POINTS && window.len() >= OBSTACLE_POINTS && span >= MIN_SPAN_M {
            let distance = above.iter().map(|a| a.0).fold(f32::INFINITY, f32::min);
            // Its top: the highest point within half a metre past the front.
            let upto = (i + 6).min(BINS);
            let top = bins[i..upto].iter().flatten().filter(|a| a.0 < distance + 0.5).map(|a| a.1).fold(0.0f32, f32::max);
            return Some(Blocker { distance, top: top - ground, tall: top >= eye_m - EYE_MARGIN_M });
        }
        // Ground: follow it up and down by a step at most (a drop is not followed).
        let low = bin.iter().map(|a| a.1).fold(f32::INFINITY, f32::min);
        if (low - ground).abs() <= STEP_M {
            ground = low;
        }
    }
    None
}

impl Track {
    /// The odometry at `t` (interpolated from the history).
    fn pos_at(&self, t: Instant) -> [f32; 2] {
        let mut before: Option<&(Instant, [f32; 2])> = None;
        for h in &self.history {
            if h.0 >= t {
                return match before {
                    Some(b) if h.0 > b.0 => {
                        let f = (t - b.0).as_secs_f32() / (h.0 - b.0).as_secs_f32();
                        [b.1[0] + (h.1[0] - b.1[0]) * f, b.1[1] + (h.1[1] - b.1[1]) * f]
                    }
                    _ => h.1,
                };
            }
            before = Some(h);
        }
        self.pos
    }

    /// The target's position now (predicted from the last fix).
    fn target_now(&self) -> Option<[f32; 2]> {
        let f = self.target?;
        let ahead = f.at.elapsed().as_secs_f32().min(PREDICT_S);
        Some([f.pos[0] + f.vel[0] * ahead, f.pos[1] + f.vel[1] * ahead])
    }

    fn add_fix(&mut self, at: Instant, pos: [f32; 2]) {
        let vel = match self.target {
            Some(prev) if at > prev.at && (at - prev.at) < Duration::from_secs(2) && (at - prev.at) > Duration::from_millis(200) => {
                let dt = (at - prev.at).as_secs_f32();
                let v = [(pos[0] - prev.pos[0]) / dt, (pos[1] - prev.pos[1]) / dt];
                let v = [0.5 * prev.vel[0] + 0.5 * v[0], 0.5 * prev.vel[1] + 0.5 * v[1]];
                let speed = v[0].hypot(v[1]);
                // Standing still, give or take the noise of stereo.
                if speed < 0.3 {
                    [0.0, 0.0]
                } else if speed > 3.0 {
                    [v[0] * 3.0 / speed, v[1] * 3.0 / speed]
                } else {
                    v
                }
            }
            _ => [0.0, 0.0],
        };
        self.target = Some(Fix { at, pos, vel });
    }
}

impl Follower {
    pub fn status(&self) -> Value {
        let s = self.inner.lk();
        json!({
            "target": s.target,
            "state": if s.state.is_empty() { "idle" } else { s.state },
            "last_seen_s": s.last_seen.map(|t| (t.elapsed().as_secs_f64() * 10.0).round() / 10.0),
            "distance_m": (s.distance as f64 * 100.0).round() / 100.0,
            "obstacle_m": if s.obstacle.is_finite() { json!((s.obstacle as f64 * 100.0).round() / 100.0) } else { Value::Null },
            "distance": (if s.stand > 0.0 { s.stand } else { STAND_M } as f64 * 10.0).round() / 10.0,
            "hold": s.hold,
            "move": s.moving,
            "avoiding": s.avoiding,
            "jumps": s.jumps,
        })
    }

    pub fn is_idle(&self) -> bool {
        !self.inner.lk().running
    }

    /// Follows `name`, standing `distance` world metres away.
    pub fn start(self: &Arc<Self>, bridge: &Arc<Bridge>, name: &str, distance: Option<f32>) {
        let stop = Arc::new(AtomicBool::new(false));
        // Swapped under the lock: a start at the same moment cannot leave
        // a flag nobody sets.
        std::mem::replace(&mut *self.stop.lk(), stop.clone()).store(true, Ordering::SeqCst);
        {
            let mut s = self.inner.lk();
            *s = State {
                target: name.to_string(),
                state: "following",
                last_seen: None,
                stand: distance.unwrap_or(STAND_M).clamp(MIN_STAND_M, MAX_STAND_M),
                obstacle: f32::INFINITY,
                running: true,
                ..Default::default()
            };
        }
        // After the follow before has let go of the stick and the headset.
        let mut runner = self.runner.lk();
        let before = runner.take();
        let (me, b) = (self.clone(), bridge.clone());
        *runner = Some(std::thread::spawn(move || {
            if let Some(before) = before {
                let _ = before.join();
            }
            if !stop.load(Ordering::SeqCst) {
                me.run(b, stop);
            }
        }));
        drop(runner);
        bridge.notify_state();
    }

    /// The follow's state to write, while `stop` is the running follow's
    /// (a stopped one, still finishing a look, writes nothing).
    fn inner_for(&self, stop: &AtomicBool) -> Option<std::sync::MutexGuard<'_, State>> {
        let s = self.inner.lk();
        (!stop.load(Ordering::SeqCst)).then_some(s)
    }

    pub fn stop(&self) {
        self.stop.lk().store(true, Ordering::SeqCst);
        let mut s = self.inner.lk();
        s.running = false;
        s.state = "idle";
        s.moving = 0.0;
    }

    /// closer / farther / stay / resume.
    pub fn adjust(&self, change: &str) -> anyhow::Result<()> {
        let mut s = self.inner.lk();
        if !s.running {
            anyhow::bail!("not following anyone");
        }
        match change {
            "closer" => (s.stand, s.hold) = ((s.stand * CLOSER).clamp(MIN_STAND_M, MAX_STAND_M), false),
            "farther" => (s.stand, s.hold) = ((s.stand * FARTHER).clamp(MIN_STAND_M, MAX_STAND_M), false),
            "stay" => s.hold = true,
            "resume" => s.hold = false,
            _ => anyhow::bail!("change must be closer, farther, stay or resume"),
        }
        Ok(())
    }

    fn run(self: Arc<Self>, bridge: Arc<Bridge>, stop: Arc<AtomicBool>) {
        let target = self.inner.lk().target.clone();
        let facing = {
            let mut vr = bridge.vr.lk();
            if stop.load(Ordering::SeqCst) {
                return; // stopped while it waited for the headset
            }
            vr.forget_places(); // following moves the bot
            vr.yaw
        };
        let track = Arc::new(Mutex::new(Track { facing, ..Default::default() }));
        let legs = {
            let (me, b, t, s) = (self.clone(), bridge.clone(), track.clone(), stop.clone());
            std::thread::spawn(move || me.legs(&b, &t, &s))
        };
        // However the follow ends (a panic too): the legs stop, the stick
        // is let go, and the state says so if no other follow took over.
        let _done = Done { me: self.clone(), bridge: bridge.clone(), stop: stop.clone(), legs: Some(legs) };
        let mut last_here = Instant::now();
        let mut searching = false;
        let mut next_search = Instant::now();
        let mut last_glance = Instant::now();
        let mut osc: Option<Osc> = None;
        while !stop.load(Ordering::SeqCst) {
            let (running, here, room) = {
                let g = bridge.game.lk();
                let room: Vec<String> = g.others().into_iter().map(|(_, n)| n).collect();
                (g.running, room.iter().any(|n| match_score(n, &target) >= 0.8), room)
            };
            if !running {
                break;
            }
            if here || room.is_empty() {
                last_here = Instant::now();
            } else if last_here.elapsed() > GONE_AFTER {
                if !stop.load(Ordering::SeqCst) {
                    bridge.send_event(json!({"type": "follow", "state": "gone", "target": target}));
                }
                break;
            }
            if osc.is_none() {
                osc = bridge.osc_query().ok();
            }
            let metres = match osc.as_ref().map(|o| o.eye_height()) {
                Some(Ok(h)) if h > 0.0 => h as f32 / (bridge.anim.params().head_height - FLOOR_Y),
                _ => {
                    osc = None;
                    1.0
                }
            };
            // Following a wall, the target may be out of sight a while.
            let lost = {
                let mut t = track.lk();
                let unseen = t.target.map_or(Duration::MAX, |f| f.at.elapsed());
                if t.wall.is_some() && unseen > WALL_UNSEEN {
                    t.wall = None; // round a wall long out of sight: look for them
                    t.detour = None;
                }
                let seeking = t.seek.is_some_and(|until| Instant::now() < until);
                t.wall.is_none() && !seeking && unseen > LOST_AFTER
            };
            // Along a wall, now and then a view the target's way.
            let glance = {
                let t = track.lk();
                match (t.wall, t.target_now()) {
                    (Some(_), Some(goal)) if last_glance.elapsed() > GLANCE_EVERY => {
                        let to = bearing(t.pos, goal);
                        (angle_diff(to, t.facing).abs() > VIEW_HALF_DEG).then_some(to)
                    }
                    _ => None,
                }
            };
            let result = if let Some(to) = glance {
                last_glance = Instant::now();
                self.look(&bridge, &track, &target, &room, metres, Some(to), &stop).map(|_| ())
            } else if !lost {
                self.look(&bridge, &track, &target, &room, metres, None, &stop).map(|_| ())
            } else if Instant::now() >= next_search {
                if !searching {
                    searching = true;
                    // The event after the lock: sending it reads the follow's
                    // state (status), and the lock is not reentrant.
                    let current = self.inner_for(&stop).map(|mut s| s.state = "searching").is_some();
                    if current {
                        bridge.send_event(json!({"type": "follow", "state": "searching", "target": target}));
                    }
                }
                let found = self.search(&bridge, &track, &target, &room, metres, &stop);
                if !matches!(found, Ok(true)) {
                    // Nobody all round: walk toward where they were a while.
                    let mut t = track.lk();
                    let far = t.target_now().is_some_and(|g| (g[0] - t.pos[0]).hypot(g[1] - t.pos[1]) > self.inner.lk().stand + 0.5);
                    if far {
                        t.seek = Some(Instant::now() + SEEK_FOR);
                        next_search = Instant::now() + SEEK_FOR;
                    } else {
                        next_search = Instant::now() + SEARCH_PAUSE;
                    }
                }
                found.map(|_| ())
            } else {
                std::thread::sleep(Duration::from_millis(100));
                Ok(())
            };
            if let Err(e) = result {
                tracing::warn!("follow round failed: {e:#}");
                bridge.vr.lk().reset();
                std::thread::sleep(Duration::from_millis(500));
            }
            let seen = track.lk().target.is_some_and(|f| f.at.elapsed() < LOST_AFTER);
            if seen && searching {
                searching = false;
                let current = self.inner_for(&stop).map(|mut s| s.state = "following").is_some();
                if current {
                    bridge.send_event(json!({"type": "follow", "state": "found", "target": target}));
                }
            }
        }
    }

    /// The search: one view at a time, from where the target was last seen
    /// outwards; true as soon as a view finds them.
    fn search(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, stop: &AtomicBool) -> anyhow::Result<bool> {
        let from = {
            let t = track.lk();
            match t.target_now() {
                Some(p) => bearing(t.pos, p),
                None => t.facing,
            }
        };
        for offset in SEARCH {
            if stop.load(Ordering::SeqCst) {
                return Ok(false);
            }
            let yaw = wrap(from + offset);
            if self.look(bridge, track, target, room, metres, Some(yaw), stop)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// One look: the newest frame (or, with `aim`, the first one looking
    /// that way, the whole bot turned there), its name tags placed; whether
    /// the target was among them.
    #[allow(clippy::too_many_arguments)]
    fn look(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, aim: Option<f32>, stop: &AtomicBool) -> anyhow::Result<bool> {
        // Along a wall a look elsewhere is a glance: the head alone.
        let glance = aim.is_some() && track.lk().wall.is_some();
        let whitelist = bridge.social.whitelist_names();
        let (frame, ocr) = {
            let mut vr = bridge.vr.lk();
            let frame = match aim {
                Some(yaw) => {
                    // Exactly that way: the animation's sway would miss it.
                    vr.rig(&whitelist)?.hmd.hold_still(true)?;
                    let turned = if glance { vr.aim(yaw, PITCH) } else { vr.face(yaw, PITCH) };
                    let frame = turned.and_then(|()| scan::rendered_at(&mut vr.rig(&whitelist)?.tap, yaw, PITCH, Duration::from_secs(1)));
                    // The head back the way the walk goes: walking follows
                    // the head, and the next view judges from it.
                    let back = if glance { vr.aim(track.lk().facing, PITCH) } else { Ok(()) };
                    vr.rig(&whitelist)?.hmd.hold_still(false)?;
                    back?;
                    if !glance {
                        track.lk().facing = yaw;
                    }
                    frame?
                }
                None => vr.rig(&whitelist)?.tap.read()?.ok_or_else(|| anyhow::anyhow!("no frame yet"))?,
            };
            let rig = vr.rig(&whitelist)?;
            let ocr = rig.ocr.clone().ok_or_else(|| anyhow::anyhow!("following needs OCR"))?;
            (frame, ocr)
        };
        // The frame is at most a frame old: as good as now for the odometry.
        let at = Instant::now();
        let stereo = Stereo::from_frame(&frame, 2).ok_or_else(|| anyhow::anyhow!("not an 8-bit frame"))?;
        let disp = stereo_pool().install(|| stereo.disparity(&SgmParams::default()));
        let eye = eye_of(&frame);
        let (yaw, _) = frame.views[0].pose.yaw_pitch();
        // The floor, in stereo units (the tracking space's).
        let floor = FLOOR_Y;
        let lines = ocr.lines_rgb(&frame.eye_rgb8(0)?, frame.width as u16, frame.height as u16)?;
        let mut names: Vec<String> = room.to_vec();
        if !names.iter().any(|n| n == target) {
            names.push(target.to_string());
        }
        let seen = vrc_players::sightings(&frame, &stereo, &disp, &lines, &names, &whitelist, floor);
        // Whitelisted friends in sight: sightings, for "when did you last see".
        for s in &seen {
            if s.whitelist_rank.is_some() {
                if let Ok(jpeg) = crate::vr::eye_jpeg(&frame, 640) {
                    bridge.sightings.saw(&s.name, &bridge.game.lk().world_name, jpeg);
                }
            }
        }
        let points: Vec<[f32; 3]> = stereo.points(&disp, 2).into_iter().map(|(p, _)| p).collect();
        let hit = seen.iter().filter(|s| match_score(&s.name, target) >= 0.6).max_by(|a, b| a.score.total_cmp(&b.score));
        let mut t = track.lk();
        let then = t.pos_at(at);
        let found = if let Some(hit) = hit {
            let rel = [(hit.feet[0] - eye[0]) * metres, (hit.feet[2] - eye[2]) * metres];
            t.add_fix(at, [then[0] + rel[0], then[1] + rel[1]]);
            t.seek = None;
            if let Some(mut s) = self.inner_for(stop) {
                s.last_seen = Some(at);
                s.distance = rel[0].hypot(rel[1]);
            }
            true
        } else {
            false
        };
        // The way to them, as far as this view shows it (following a wall,
        // where they were).
        let goal = if t.wall.is_some() { t.target_now() } else { t.target.filter(|f| f.at.elapsed() < LOST_AFTER).and_then(|_| t.target_now()) };
        let Some(goal) = goal else {
            return Ok(found);
        };
        let direct = bearing(then, goal);
        let gap = (goal[0] - then[0]).hypot(goal[1] - then[1]);
        let in_view = angle_diff(direct, yaw).abs() <= VIEW_HALF_DEG;
        // Their own body (within half a metre of their feet) is not in the way.
        let blocked = if in_view { corridor(&points, eye, direct, metres, floor).filter(|b| b.distance < gap - 0.5 && b.distance < 3.0) } else { None };
        let free = |h: f32| corridor(&points, eye, h, metres, floor).is_none_or(|b| b.distance > FREE_M);
        let mut guard = self.inner_for(stop);
        let mut scratch = State::default(); // a stopped follow's: not kept
        let s: &mut State = match guard.as_deref_mut() {
            Some(s) => s,
            None => &mut scratch,
        };
        s.obstacle = blocked.map_or(f32::INFINITY, |b| b.distance);
        if let Some(w) = t.wall {
            // Nothing up to the eyes the straight way, nor a low thing a
            // jump did not clear (one not yet tried is jumped, below):
            // leave the wall for them.
            let (ds, dc) = direct.to_radians().sin_cos();
            let failed = |b: &Blocker| {
                let q = [then[0] + ds * b.distance, then[1] - dc * b.distance];
                t.no_jump.iter().any(|&(n, until)| at < until && (n[0] - q[0]).hypot(n[1] - q[1]) < SAME_PLACE_M)
            };
            let open = in_view && blocked.is_none_or(|b| !b.tall && !failed(&b));
            if open || at - w.since > WALL_FOR {
                t.wall = None;
                t.detour = None;
                t.last_side = Some((w.side, at));
                s.avoiding = "";
            } else if glance {
                return Ok(found); // a glance their way: the walk along the wall goes on
            } else {
                // A hand on the wall: turn toward it only where it opens up
                // (a corner: 30 degrees or more its way free), keep the way
                // while it is free, else the free heading nearest the wall.
                let at_side = |k: i32| wrap(yaw + w.side * k as f32 * WALL_STEP_DEG);
                let opening = (2..=WALL_STEPS).rev().map(at_side).find(|&h| free(h));
                let keep = Some(yaw).filter(|&h| free(h));
                let away = (-WALL_STEPS..=1).rev().map(at_side).find(|&h| free(h));
                let heading = opening.or(keep).or(away).unwrap_or_else(|| wrap(yaw - w.side * 70.0)); // boxed in: turn away
                t.detour = Some((at + DETOUR_FOR, heading));
                s.avoiding = "wall";
                return Ok(found);
            }
        }
        if !in_view {
            return Ok(found); // not in view: the turn goes on
        }
        let Some(b) = blocked else {
            t.obstacle = None;
            t.detour = None;
            s.avoiding = "";
            return Ok(found);
        };
        let mut obstacle = Obstacle { yaw: direct, then, distance: b.distance, jump: false };
        let place = obstacle.place();
        let near = |q: [f32; 2]| (q[0] - place[0]).hypot(q[1] - place[1]) < SAME_PLACE_M;
        // Still there after a jump at it: that jump failed.
        if let Some((when, q)) = t.jumped {
            let since = at.saturating_duration_since(when);
            if since > Duration::from_millis(600) && since < Duration::from_secs(4) && near(q) {
                t.no_jump.push((q, at + NO_JUMP_FOR));
                t.jumped = None;
            }
        }
        t.no_jump.retain(|&(_, until)| at < until);
        // Not up to the eyes, and not failed yet: over it, first.
        obstacle.jump = !b.tall && !t.no_jump.iter().any(|&(q, _)| near(q));
        if obstacle.jump {
            t.obstacle = Some(obstacle);
            t.detour = None;
            s.avoiding = "jump";
            return Ok(found);
        }
        // Else round it, following it: go the side with a free heading
        // nearest the way to them, keeping the wall on the other side.
        t.obstacle = None;
        let again = t.last_side.filter(|&(_, when)| at - when < SAME_SIDE_FOR).map(|(side, _)| side);
        let side = again.unwrap_or_else(|| (1..=4)
            .find_map(|k| {
                let (r, l) = (wrap(direct + k as f32 * WALL_STEP_DEG), wrap(direct - k as f32 * WALL_STEP_DEG));
                if free(r) {
                    Some(-1.0) // round to the right: the wall on the left
                } else if free(l) {
                    Some(1.0)
                } else {
                    None
                }
            })
            .unwrap_or(1.0));
        t.wall = Some(Wall { side, since: at });
        t.detour = None;
        s.avoiding = "wall";
        Ok(found)
    }

    /// The legs: odometry, turning to the target, the thumbstick.
    fn legs(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, stop: &AtomicBool) {
        let mut osc: Option<Osc> = None;
        let mut last = Instant::now();
        let mut axis = 0.0f32;
        let mut aimed = f32::NAN;
        let mut last_jump: Option<Instant> = None;
        let mut slow_since: Option<Instant> = None;
        let mut pushed_since: Option<Instant> = None;
        let mut escape: Option<Instant> = None;
        let mut aside = 1.0f32;
        let mut link: Option<vrc_vr::remote::HmdLink> = None;
        let mut strafe = 0.0f32;
        while !stop.load(Ordering::SeqCst) {
            std::thread::sleep(TICK);
            if osc.is_none() {
                osc = bridge.osc_query().ok();
            }
            let velocity = osc.as_ref().and_then(|o| {
                Some((o.query("/avatar/parameters/VelocityX").ok()? as f32, o.query("/avatar/parameters/VelocityZ").ok()? as f32))
            });
            let Some((vx, vz)) = velocity else {
                osc = None;
                self.set_axis(bridge, &mut axis, 0.0);
                if strafe != 0.0 {
                    strafe = 0.0;
                    let _ = bridge.osc.send_f32("/input/Horizontal", 0.0);
                }
                continue;
            };
            let now = Instant::now();
            let dt = (now - last).as_secs_f32().min(0.2);
            last = now;
            let (stand, hold) = {
                let s = self.inner.lk();
                (s.stand, s.hold)
            };
            // The way the head looks (walking follows it): from the headset
            // itself, as walks of a way round turn it too.
            if link.is_none() {
                link = bridge.vr.try_lk().and_then(|vr| vr.link());
            }
            let owner = link.as_ref().and_then(|l| l.owner());
            if owner.is_none() {
                link = None; // the headset connected again since: its new link next tick
            }
            let heading = owner.map(|o| o.state.head.yaw_pitch().0);
            let mut t = track.lk();
            // Odometry: the avatar's velocity is its own (z ahead, x right).
            let (fs, fc) = heading.unwrap_or(t.facing).to_radians().sin_cos();
            let (ahead, right) = ([fs, -fc], [fc, fs]);
            for k in 0..2 {
                t.pos[k] += (vz * ahead[k] + vx * right[k]) * dt;
            }
            let pos = t.pos;
            t.history.push_back((now, pos));
            while t.history.front().is_some_and(|h| now - h.0 > ODOMETRY_KEPT) {
                t.history.pop_front();
            }
            let seeking = t.seek.is_some_and(|until| now < until);
            let fresh = t.wall.is_some() || seeking || t.target.is_some_and(|f| f.at.elapsed() < LOST_AFTER);
            let walling = t.wall.is_some();
            let mut jump = false;
            let want = match t.target_now() {
                _ if escape.is_some_and(|e| now < e) => BACK_AXIS,
                Some(goal) if fresh && !hold => {
                    let (gx, gz) = (goal[0] - pos[0], goal[1] - pos[1]);
                    let gap = gx.hypot(gz);
                    // Round something, or straight to them.
                    // Along a wall the last way a view chose holds until the
                    // next view (one may take longer than DETOUR_FOR): never
                    // straight at them through the wall meanwhile.
                    let detour = t.detour.filter(|d| now < d.0 || walling).map(|d| d.1);
                    if gap > 0.3 || detour.is_some() {
                        let to = detour.unwrap_or_else(|| bearing(pos, goal));
                        let step = angle_diff(to, t.facing).clamp(-TURN_RATE * dt, TURN_RATE * dt);
                        t.facing = wrap(t.facing + step);
                    }
                    let speed = vz.max(0.0);
                    let left = gap - stand - speed * LAG_S;
                    // Something in the way, nearer than they are?
                    let facing = t.facing;
                    if t.obstacle.and_then(|o| o.ahead(pos, facing)).is_some_and(|d| d < -0.3) {
                        t.obstacle = None; // over it, past it
                    }
                    let ob = t.obstacle.and_then(|o| o.ahead(pos, facing).map(|d| (o, d))).filter(|&(_, d)| d < gap - 0.3);
                    // In the air after a jump at it, the run goes on over it.
                    let airborne = last_jump.is_some_and(|j| now - j < JUMP_CARRY);
                    let over = ob.filter(|(o, d)| o.jump && *d < 2.0 && (airborne || last_jump.is_none_or(|j| now - j > JUMP_EVERY)));
                    let blocked = ob.is_some_and(|(_, d)| d < OBSTACLE_M) && over.is_none();
                    if let Some((o, d)) = over.filter(|_| !airborne) {
                        jump = d < JUMP_AT_M + speed * 0.1;
                        if jump {
                            t.jumped = Some((now, o.place()));
                        }
                    }
                    let start = if axis > 0.0 { 0.05 } else { WALK_MARGIN };
                    if gap - stand > start && !blocked {
                        let mut v = (2.0 * BRAKE * left.max(0.0)).sqrt().min(MAX_SPEED);
                        if over.is_some() {
                            v = v.max(RUN_UP_SPEED); // a run-up
                        }
                        if walling {
                            v = v.clamp(MIN_SPEED * 2.0, WALL_SPEED); // along it, out of reach or not
                        }
                        if v >= MIN_SPEED { AXIS_DEAD + v / SPEED_PER_AXIS } else { 0.0 }
                    } else if gap < stand - BACK_MARGIN || (axis < 0.0 && gap < stand - BACK_UNTIL) {
                        BACK_AXIS
                    } else {
                        0.0
                    }
                }
                _ => 0.0,
            };
            // Stuck: pushing, not moving. Jump once; then back off and turn aside.
            if axis > STUCK_AXIS {
                let pushed = *pushed_since.get_or_insert(now);
                // The way it moves (the body may lag the head: sideways too).
                if now - pushed > Duration::from_millis(500) && vx.hypot(vz) < STUCK_SPEED {
                    if now - *slow_since.get_or_insert(now) > STUCK_FOR {
                        slow_since = None;
                        if last_jump.is_none_or(|j| now - j > Duration::from_secs(3)) {
                            jump = true;
                        } else {
                            escape = Some(now + ESCAPE_FOR);
                            aside = -aside;
                            let yaw = wrap(t.facing + aside * 60.0);
                            t.detour = Some((now + ESCAPE_FOR + DETOUR_FOR, yaw));
                            if let Some(mut s) = self.inner_for(stop) {
                                s.avoiding = "stuck";
                            }
                        }
                    }
                } else {
                    slow_since = None;
                }
            } else {
                pushed_since = None;
                slow_since = None;
            }
            let facing = t.facing;
            drop(t);
            // Walking goes where the head looks: split the stick so the walk
            // keeps to the facing while a glance turns the head.
            let off = heading.map_or(0.0, |h| angle_diff(facing, h));
            let (ahead_axis, aside_axis) = if want > 0.0 {
                if off.abs() > 100.0 {
                    (0.0, 0.0)
                } else {
                    let (s, c) = off.to_radians().sin_cos();
                    (want * c, want * s)
                }
            } else {
                (want, 0.0)
            };
            if (ahead_axis - axis).abs() > 0.02 || (ahead_axis == 0.0 && axis != 0.0) {
                self.set_axis(bridge, &mut axis, ahead_axis.clamp(-1.0, 1.0));
            }
            if (aside_axis - strafe).abs() > 0.03 || (aside_axis == 0.0 && strafe != 0.0) {
                strafe = aside_axis.clamp(-1.0, 1.0);
                let _ = bridge.osc.send_f32("/input/Horizontal", strafe);
            }
            if jump {
                last_jump = Some(now);
                let _ = bridge.osc.send_i32("/input/Jump", 1);
                let b = bridge.clone();
                std::thread::spawn(move || {
                    std::thread::sleep(Duration::from_millis(100));
                    let _ = b.osc.send_i32("/input/Jump", 0);
                });
                if let Some(mut s) = self.inner_for(stop) {
                    s.jumps += 1;
                }
            }
            if fresh && (aimed.is_nan() || angle_diff(facing, aimed).abs() >= 1.0) {
                // The eyes may hold the headset a moment: next tick then.
                if let Some(mut vr) = bridge.vr.try_lk() {
                    if vr.face(facing, PITCH).is_ok() {
                        aimed = facing;
                    }
                }
            }
        }
        self.set_axis(bridge, &mut axis, 0.0);
        let _ = bridge.osc.send_f32("/input/Horizontal", 0.0);
    }

    fn set_axis(&self, bridge: &Arc<Bridge>, axis: &mut f32, value: f32) {
        if *axis != value {
            let _ = bridge.osc.send_f32("/input/Vertical", value);
            *axis = value;
            self.inner.lk().moving = (value * 100.0).round() / 100.0;
        }
    }
}

/// The end of a follow (see `Follower::run`).
struct Done {
    me: Arc<Follower>,
    bridge: Arc<Bridge>,
    stop: Arc<AtomicBool>,
    legs: Option<std::thread::JoinHandle<()>>,
}

impl Drop for Done {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(legs) = self.legs.take() {
            let _ = legs.join();
        }
        // The legs let go when they end; a panic in them may not have.
        for axis in ["/input/Vertical", "/input/Horizontal"] {
            let _ = self.bridge.osc.send_f32(axis, 0.0);
        }
        let current = Arc::ptr_eq(&self.me.stop.lk(), &self.stop);
        if current {
            let mut s = self.me.inner.lk();
            s.running = false;
            s.state = "idle";
            s.moving = 0.0;
        }
        self.bridge.notify_state();
    }
}

/// Following looks again and again: its stereo gets a few cores, not all
/// (all of them made a follow cost about six cores' time, beside the game).
fn stereo_pool() -> &'static rayon::ThreadPool {
    static POOL: std::sync::OnceLock<rayon::ThreadPool> = std::sync::OnceLock::new();
    POOL.get_or_init(|| rayon::ThreadPoolBuilder::new().num_threads(STEREO_THREADS).build().expect("a thread pool"))
}

/// What the corridor along `yaw` holds, for tuning (`/v1/vr/corridor`):
/// per 10 cm from 0.3 m out, the points' count and lowest and highest
/// height over the floor (world metres), and what `corridor` makes of it.
pub fn corridor_report(frame: &EyeFrame, yaw: Option<f32>, metres: f32) -> anyhow::Result<Value> {
    let stereo = Stereo::from_frame(frame, 2).ok_or_else(|| anyhow::anyhow!("not an 8-bit frame"))?;
    let disp = stereo_pool().install(|| stereo.disparity(&SgmParams::default()));
    let points: Vec<[f32; 3]> = stereo.points(&disp, 2).into_iter().map(|(p, _)| p).collect();
    let eye = eye_of(frame);
    let yaw = yaw.unwrap_or_else(|| frame.views[0].pose.yaw_pitch().0);
    let (s, c) = yaw.to_radians().sin_cos();
    let mut bins: Vec<(usize, f32, f32)> = vec![(0, f32::INFINITY, f32::NEG_INFINITY); 40];
    for p in &points {
        let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
        let (ahead, side, up) = ((dx * s - dz * c) * metres, (dx * c + dz * s) * metres, (p[1] - FLOOR_Y) * metres);
        if ahead > 0.3 && side.abs() < 0.25 {
            let i = ((ahead - 0.3) / 0.1) as usize;
            if let Some(b) = bins.get_mut(i) {
                (b.0, b.1, b.2) = (b.0 + 1, b.1.min(up), b.2.max(up));
            }
        }
    }
    let r = |v: f32| (v as f64 * 100.0).round() / 100.0;
    Ok(json!({
        "yaw": yaw.round(),
        "eye_m": r((eye[1] - FLOOR_Y) * metres),
        "points": points.len(),
        "blocker": corridor(&points, eye, yaw, metres, FLOOR_Y).map(|b| json!({"distance": r(b.distance), "top": r(b.top), "tall": b.tall})),
        "bins": bins.iter().enumerate().filter(|(_, b)| b.0 > 0).map(|(i, b)| json!([r(0.3 + 0.1 * i as f32), b.0, r(b.1), r(b.2)])).collect::<Vec<_>>(),
    }))
}

/// The middle of the eyes of `frame` (tracking space, stereo units).
fn eye_of(frame: &EyeFrame) -> [f32; 3] {
    let (a, b) = (frame.views[0].pose.position, frame.views[1].pose.position);
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0]
}

/// The yaw (degrees, + right of -z) from `from` to `to`.
fn bearing(from: [f32; 2], to: [f32; 2]) -> f32 {
    (to[0] - from[0]).atan2(-(to[1] - from[1])).to_degrees()
}

fn wrap(deg: f32) -> f32 {
    (deg + 540.0).rem_euclid(360.0) - 180.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odometry_interpolates() {
        let t0 = Instant::now();
        let mut t = Track::default();
        t.history.push_back((t0, [0.0, 0.0]));
        t.history.push_back((t0 + Duration::from_millis(100), [1.0, -2.0]));
        t.pos = [1.0, -2.0];
        let mid = t.pos_at(t0 + Duration::from_millis(50));
        assert!((mid[0] - 0.5).abs() < 1e-4 && (mid[1] + 1.0).abs() < 1e-4);
        assert_eq!(t.pos_at(t0 + Duration::from_secs(1)), [1.0, -2.0]);
    }

    #[test]
    fn fixes_estimate_motion() {
        let t0 = Instant::now() - Duration::from_secs(1);
        let mut t = Track::default();
        t.add_fix(t0, [0.0, -3.0]);
        t.add_fix(t0 + Duration::from_millis(500), [0.0, -3.5]);
        let v = t.target.unwrap().vel;
        // Half of 1 m/s (smoothed from rest), away along -z.
        assert!((v[1] + 0.5).abs() < 1e-4, "{v:?}");
        // Jitter of a few centimetres is standing still.
        t.add_fix(t0 + Duration::from_millis(900), [0.02, -3.5]);
        assert!(t.target.unwrap().vel[0].abs() < 0.3);
    }

    #[test]
    fn corridors_find_and_measure_obstacles() {
        let eye = [0.0, 1.6, 0.0];
        let floor_to = |points: &mut Vec<[f32; 3]>, from: f32, to: f32, y: f32| {
            let mut z = from;
            while z < to {
                for i in 0..5 {
                    points.push([-0.2 + 0.1 * i as f32, y, -z]);
                }
                z += 0.05;
            }
        };
        // Floor, then a box 0.4 m high from 1.0 m ahead (-z).
        let mut points = Vec::new();
        floor_to(&mut points, 0.4, 1.0, 0.0);
        // Its face, as densely as stereo samples it (2 cm).
        for j in 0..21 {
            for i in 0..20 {
                points.push([-0.2 + 0.02 * i as f32, 0.02 * j as f32, -1.0]);
            }
        }
        let b = corridor(&points, eye, 0.0, 1.0, 0.0).unwrap();
        assert!((b.distance - 1.0).abs() < 0.05 && (b.top - 0.4).abs() < 0.05 && !b.tall, "{b:?}");
        // A wall up past the eyes, 1.5 m to the right: tall.
        let mut wall = Vec::new();
        floor_to(&mut wall, 0.4, 1.5, 0.0);
        let wall: Vec<[f32; 3]> = wall.iter().map(|p| [-p[2], p[1], p[0]]).collect::<Vec<_>>();
        let mut wall = wall;
        for j in 0..40 {
            for i in 0..5 {
                wall.push([1.5, 0.05 * j as f32, -0.2 + 0.1 * i as f32]);
            }
        }
        let b = corridor(&wall, eye, 90.0, 1.0, 0.0).unwrap();
        assert!((b.distance - 1.5).abs() < 0.05 && b.tall, "{b:?}");
        assert!(corridor(&wall, eye, -90.0, 1.0, 0.0).is_none());
        // A ramp rising 0.15 m per 0.3 m: followed, not in the way.
        let mut ramp = Vec::new();
        let mut z = 0.4;
        while z < 3.0 {
            for i in 0..5 {
                ramp.push([-0.2 + 0.1 * i as f32, (z - 0.4) * 0.5, -z]);
            }
            z += 0.05;
        }
        assert!(corridor(&ramp, eye, 0.0, 1.0, 0.0).is_none());
        // Near: the bot's own weapon at 0.6 m (up to 1.1 m high) is not in
        // the way; a wall there (up past the eyes) is.
        let mut prop = Vec::new();
        for j in 0..10 {
            for i in 0..5 {
                prop.push([-0.2 + 0.1 * i as f32, 0.6 + 0.05 * j as f32, -0.6]);
            }
        }
        assert!(corridor(&prop, eye, 0.0, 1.0, 0.0).is_none());
        for j in 0..34 {
            for i in 0..10 {
                prop.push([-0.2 + 0.05 * i as f32, 0.05 * j as f32, -0.7]);
            }
        }
        let b = corridor(&prop, eye, 0.0, 1.0, 0.0).unwrap();
        assert!((b.distance - 0.7).abs() < 0.05 && b.tall, "{b:?}");
        // Walking up to an obstacle along its heading brings it nearer; another way, it is not ahead.
        let o = Obstacle { yaw: 0.0, then: [0.0, 0.0], distance: 1.0, jump: true };
        assert!((o.ahead([0.0, -0.4], 0.0).unwrap() - 0.6).abs() < 1e-4);
        assert!(o.ahead([0.0, 0.0], 60.0).is_none());
        assert_eq!(o.place(), [0.0, -1.0]);
    }

    #[test]
    fn bearings() {
        assert!((bearing([0.0, 0.0], [0.0, -1.0])).abs() < 1e-4);
        assert!((bearing([0.0, 0.0], [1.0, 0.0]) - 90.0).abs() < 1e-4);
        assert!((wrap(190.0) + 170.0).abs() < 1e-4);
    }
}
