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
//! Lost, the bot stands and the eyes look around one view at a time,
//! starting where the target was last seen and widening both ways; the
//! first view that finds them ends the search and the legs go that way.
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
const SEARCH: [f32; 6] = [0.0, 55.0, -55.0, 115.0, -115.0, 180.0];
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

#[derive(Default)]
pub struct Follower {
    inner: Mutex<State>,
    /// The running follow's stop flag (a new one per follow).
    stop: Mutex<Arc<AtomicBool>>,
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
    /// Something ahead: (when, how far ahead along `facing` then, odometry then).
    obstacle: Option<(Instant, f32, [f32; 2])>,
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
        let s = self.inner.lock().unwrap();
        json!({
            "target": s.target,
            "state": if s.state.is_empty() { "idle" } else { s.state },
            "last_seen_s": s.last_seen.map(|t| (t.elapsed().as_secs_f64() * 10.0).round() / 10.0),
            "distance_m": (s.distance as f64 * 100.0).round() / 100.0,
            "obstacle_m": if s.obstacle.is_finite() { json!((s.obstacle as f64 * 100.0).round() / 100.0) } else { Value::Null },
            "distance": (if s.stand > 0.0 { s.stand } else { STAND_M } as f64 * 10.0).round() / 10.0,
            "hold": s.hold,
            "move": s.moving,
        })
    }

    pub fn is_idle(&self) -> bool {
        !self.inner.lock().unwrap().running
    }

    /// Follows `name`, standing `distance` world metres away.
    pub fn start(self: &Arc<Self>, bridge: &Arc<Bridge>, name: &str, distance: Option<f32>) {
        self.stop();
        let stop = Arc::new(AtomicBool::new(false));
        *self.stop.lock().unwrap() = stop.clone();
        {
            let mut s = self.inner.lock().unwrap();
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
        let (me, b) = (self.clone(), bridge.clone());
        std::thread::spawn(move || me.run(b, stop));
        bridge.notify_state();
    }

    pub fn stop(&self) {
        self.stop.lock().unwrap().store(true, Ordering::SeqCst);
        let mut s = self.inner.lock().unwrap();
        s.running = false;
        s.state = "idle";
        s.moving = 0.0;
    }

    /// closer / farther / stay / resume.
    pub fn adjust(&self, change: &str) -> anyhow::Result<()> {
        let mut s = self.inner.lock().unwrap();
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
        let target = self.inner.lock().unwrap().target.clone();
        let track = Arc::new(Mutex::new(Track { facing: bridge.vr.lock().unwrap().yaw, ..Default::default() }));
        let legs = {
            let (me, b, t, s) = (self.clone(), bridge.clone(), track.clone(), stop.clone());
            std::thread::spawn(move || me.legs(&b, &t, &s))
        };
        let mut last_here = Instant::now();
        let mut searching = false;
        let mut next_search = Instant::now();
        let mut osc: Option<Osc> = None;
        while !stop.load(Ordering::SeqCst) {
            let (running, here, room) = {
                let g = bridge.game.lock().unwrap();
                let room: Vec<String> = g.others().into_iter().map(|(_, n)| n).collect();
                (g.running, room.iter().any(|n| match_score(n, &target) >= 0.8), room)
            };
            if !running {
                break;
            }
            if here || room.is_empty() {
                last_here = Instant::now();
            } else if last_here.elapsed() > GONE_AFTER {
                bridge.send_event(json!({"type": "follow", "state": "gone", "target": target}));
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
            let lost = track.lock().unwrap().target.is_none_or(|f| f.at.elapsed() > LOST_AFTER);
            let result = if !lost {
                self.look(&bridge, &track, &target, &room, metres, None).map(|_| ())
            } else if Instant::now() >= next_search {
                if !searching {
                    searching = true;
                    self.inner.lock().unwrap().state = "searching";
                    bridge.send_event(json!({"type": "follow", "state": "searching", "target": target}));
                }
                let found = self.search(&bridge, &track, &target, &room, metres, &stop);
                if !matches!(found, Ok(true)) {
                    next_search = Instant::now() + SEARCH_PAUSE;
                }
                found.map(|_| ())
            } else {
                std::thread::sleep(Duration::from_millis(100));
                Ok(())
            };
            if let Err(e) = result {
                tracing::warn!("follow round failed: {e:#}");
                bridge.vr.lock().unwrap().reset();
                std::thread::sleep(Duration::from_millis(500));
            }
            let seen = track.lock().unwrap().target.is_some_and(|f| f.at.elapsed() < LOST_AFTER);
            if seen && searching {
                searching = false;
                self.inner.lock().unwrap().state = "following";
                bridge.send_event(json!({"type": "follow", "state": "found", "target": target}));
            }
        }
        stop.store(true, Ordering::SeqCst);
        let _ = legs.join();
        let mut s = self.inner.lock().unwrap();
        s.running = false;
        s.state = "idle";
        s.moving = 0.0;
        drop(s);
        bridge.notify_state();
    }

    /// The search: one view at a time, from where the target was last seen
    /// outwards; true as soon as a view finds them.
    fn search(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, stop: &AtomicBool) -> anyhow::Result<bool> {
        let from = {
            let t = track.lock().unwrap();
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
            if self.look(bridge, track, target, room, metres, Some(yaw))? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// One look: the newest frame (or, with `aim`, the first one looking
    /// that way, the whole bot turned there), its name tags placed; whether
    /// the target was among them.
    fn look(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, aim: Option<f32>) -> anyhow::Result<bool> {
        let whitelist = bridge.social.whitelist_names();
        let (frame, ocr) = {
            let mut vr = bridge.vr.lock().unwrap();
            let frame = match aim {
                Some(yaw) => {
                    // Exactly that way: the animation's sway would miss it.
                    vr.rig(&whitelist)?.hmd.hold_still(true)?;
                    let frame = vr.face(yaw, PITCH).and_then(|()| scan::rendered_at(&mut vr.rig(&whitelist)?.tap, yaw, PITCH, Duration::from_secs(1)));
                    vr.rig(&whitelist)?.hmd.hold_still(false)?;
                    track.lock().unwrap().facing = yaw;
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
        let disp = stereo.disparity(&SgmParams::default());
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
                    bridge.sightings.saw(&s.name, &bridge.game.lock().unwrap().world_name, jpeg);
                }
            }
        }
        // Anything close straight ahead (a corridor half a metre wide, from
        // knee height to the eyes)?
        let mut nearest = f32::INFINITY;
        let (fs, fc) = yaw.to_radians().sin_cos();
        for (p, _) in stereo.points(&disp, 2) {
            let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
            let ahead = dx * fs - dz * fc;
            let side = dx * fc + dz * fs;
            // Nearer than 0.35 m is the bot's own arm (a gesture).
            if ahead * metres > 0.35 && side.abs() * metres < 0.25 && p[1] > floor + 0.4 / metres && p[1] < eye[1] {
                nearest = nearest.min(ahead * metres);
            }
        }
        let hit = seen.iter().filter(|s| match_score(&s.name, target) >= 0.6).max_by(|a, b| a.score.total_cmp(&b.score));
        let mut t = track.lock().unwrap();
        let then = t.pos_at(at);
        t.obstacle = Some((at, nearest, then));
        let mut s = self.inner.lock().unwrap();
        s.obstacle = nearest;
        let Some(hit) = hit else { return Ok(false) };
        let rel = [(hit.feet[0] - eye[0]) * metres, (hit.feet[2] - eye[2]) * metres];
        t.add_fix(at, [then[0] + rel[0], then[1] + rel[1]]);
        // Their own body is not in the way.
        if nearest > rel[0].hypot(rel[1]) - 0.5 {
            t.obstacle = None;
        }
        s.last_seen = Some(at);
        s.distance = rel[0].hypot(rel[1]);
        Ok(true)
    }

    /// The legs: odometry, turning to the target, the thumbstick.
    fn legs(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, stop: &AtomicBool) {
        let mut osc: Option<Osc> = None;
        let mut last = Instant::now();
        let mut axis = 0.0f32;
        let mut aimed = f32::NAN;
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
                continue;
            };
            let now = Instant::now();
            let dt = (now - last).as_secs_f32().min(0.2);
            last = now;
            let (stand, hold) = {
                let s = self.inner.lock().unwrap();
                (s.stand, s.hold)
            };
            let mut t = track.lock().unwrap();
            // Odometry: the avatar's velocity is its own (z ahead, x right).
            let (fs, fc) = t.facing.to_radians().sin_cos();
            let (ahead, right) = ([fs, -fc], [fc, fs]);
            for k in 0..2 {
                t.pos[k] += (vz * ahead[k] + vx * right[k]) * dt;
            }
            let pos = t.pos;
            t.history.push_back((now, pos));
            while t.history.front().is_some_and(|h| now - h.0 > ODOMETRY_KEPT) {
                t.history.pop_front();
            }
            let fresh = t.target.is_some_and(|f| f.at.elapsed() < LOST_AFTER);
            let want = match t.target_now() {
                Some(goal) if fresh && !hold => {
                    let (gx, gz) = (goal[0] - pos[0], goal[1] - pos[1]);
                    let gap = gx.hypot(gz);
                    if gap > 0.3 {
                        // Turn towards them, smoothly.
                        let to = bearing(pos, goal);
                        let step = angle_diff(to, t.facing).clamp(-TURN_RATE * dt, TURN_RATE * dt);
                        t.facing = wrap(t.facing + step);
                    }
                    let speed = vz.max(0.0);
                    let left = gap - stand - speed * LAG_S;
                    // Something in the way, nearer than they are?
                    let blocked = t.obstacle.is_some_and(|(_, d, then)| {
                        let walked = (pos[0] - then[0]) * ahead[0] + (pos[1] - then[1]) * ahead[1];
                        d - walked < OBSTACLE_M && d - walked < gap - 0.3
                    });
                    let start = if axis > 0.0 { 0.05 } else { WALK_MARGIN };
                    if gap - stand > start && !blocked {
                        let v = (2.0 * BRAKE * left.max(0.0)).sqrt().min(MAX_SPEED);
                        if v >= MIN_SPEED { AXIS_DEAD + v / SPEED_PER_AXIS } else { 0.0 }
                    } else if gap < stand - BACK_MARGIN || (axis < 0.0 && gap < stand - BACK_UNTIL) {
                        BACK_AXIS
                    } else {
                        0.0
                    }
                }
                _ => 0.0,
            };
            let facing = t.facing;
            drop(t);
            if (want - axis).abs() > 0.02 || (want == 0.0 && axis != 0.0) {
                self.set_axis(bridge, &mut axis, want.clamp(-1.0, 1.0));
            }
            if fresh && (aimed.is_nan() || angle_diff(facing, aimed).abs() >= 1.0) {
                // The eyes may hold the headset a moment: next tick then.
                if let Ok(mut vr) = bridge.vr.try_lock() {
                    if vr.face(facing, PITCH).is_ok() {
                        aimed = facing;
                    }
                }
            }
        }
        self.set_axis(bridge, &mut axis, 0.0);
    }

    fn set_axis(&self, bridge: &Arc<Bridge>, axis: &mut f32, value: f32) {
        if *axis != value {
            let _ = bridge.osc.send_f32("/input/Vertical", value);
            *axis = value;
            self.inner.lock().unwrap().moving = (value * 100.0).round() / 100.0;
        }
    }
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
    fn bearings() {
        assert!((bearing([0.0, 0.0], [0.0, -1.0])).abs() < 1e-4);
        assert!((bearing([0.0, 0.0], [1.0, 0.0]) - 90.0).abs() < 1e-4);
        assert!((wrap(190.0) + 170.0).abs() < 1e-4);
    }
}
