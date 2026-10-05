//! Following a player in the room (VR): their name tag says who and where.
//!
//! Each round takes the newest frame of the eyes (looking ahead, a little
//! down), reads the name tags (OCR) and places them by stereo
//! (`vrc-players`, the same logic as the model's look around). Seen, the
//! head turns to them (walking follows the head) and the thumbstick keeps
//! the standing distance: faster the farther, backing off when too close,
//! stopping for something just ahead. Lost, the bot stands, then looks all
//! around (a survey reads every name tag) and turns to them; following ends
//! only when told to stop or when they leave the room.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_players::names::match_score;
use vrc_stereo::{SgmParams, Stereo};
use vrc_vr::scan;

use crate::bridge::Bridge;

/// Standing distance (world metres) by default, and its limits.
const STAND_M: f32 = 1.5;
const MIN_STAND_M: f32 = 0.8;
const MAX_STAND_M: f32 = 7.0;
const CLOSER: f32 = 0.7;
const FARTHER: f32 = 1.4;
/// Walking starts this far past the standing distance, backing off this
/// far inside it.
const WALK_MARGIN: f32 = 0.3;
const BACK_MARGIN: f32 = 0.6;
/// Something within this (world metres) straight ahead stops the walk.
const OBSTACLE_M: f32 = 0.7;
/// Not seen for this long: stand; for LOOK_AFTER: look all around.
const LOST_AFTER: Duration = Duration::from_millis(1200);
const LOOK_AFTER: Duration = Duration::from_millis(2500);
/// Out of the room for this long: gone.
const GONE_AFTER: Duration = Duration::from_secs(20);
const PITCH: f32 = -10.0;

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

impl Follower {
    pub fn status(&self) -> Value {
        let s = self.inner.lock().unwrap();
        json!({
            "target": s.target,
            "state": if s.state.is_empty() { "idle" } else { s.state },
            "last_seen_s": s.last_seen.map(|t| (t.elapsed().as_secs_f64() * 10.0).round() / 10.0),
            "distance_m": (s.distance as f64 * 100.0).round() / 100.0,
            "obstacle_m": (s.obstacle as f64 * 100.0).round() / 100.0,
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
                last_seen: Some(Instant::now()),
                stand: distance.unwrap_or(STAND_M).clamp(MIN_STAND_M, MAX_STAND_M),
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
        let mut last_here = Instant::now();
        let mut last_look: Option<Instant> = None;
        let mut announced_search = false;
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
            let seen = match self.round(&bridge, &target, &room) {
                Ok(seen) => seen,
                Err(e) => {
                    tracing::warn!("follow round failed: {e:#}");
                    bridge.vr.lock().unwrap().reset();
                    std::thread::sleep(Duration::from_millis(500));
                    false
                }
            };
            let since = self.inner.lock().unwrap().last_seen.map(|t| t.elapsed()).unwrap_or_default();
            if seen {
                if announced_search {
                    bridge.send_event(json!({"type": "follow", "state": "found", "target": target}));
                    announced_search = false;
                }
                continue;
            }
            if since > LOST_AFTER {
                self.walk(&bridge, 0.0);
            }
            if since > LOOK_AFTER && last_look.is_none_or(|t| t.elapsed() > Duration::from_secs(3)) {
                if !announced_search {
                    self.inner.lock().unwrap().state = "searching";
                    bridge.send_event(json!({"type": "follow", "state": "searching", "target": target}));
                    announced_search = true;
                }
                last_look = Some(Instant::now());
                if let Err(e) = self.look_around(&bridge, &target) {
                    tracing::warn!("follow search failed: {e:#}");
                }
            }
        }
        self.walk(&bridge, 0.0);
        let mut s = self.inner.lock().unwrap();
        if !stop.load(Ordering::SeqCst) {
            s.running = false;
            s.state = "idle";
        }
        drop(s);
        bridge.notify_state();
    }

    /// One look ahead: whether the target was seen (and steered to).
    fn round(&self, bridge: &Arc<Bridge>, target: &str, room: &[String]) -> anyhow::Result<bool> {
        let whitelist = bridge.social.whitelist_names();
        let mut vr = bridge.vr.lock().unwrap();
        let yaw = vr.yaw;
        if (vr.pitch - PITCH).abs() > 0.5 {
            vr.aim(yaw, PITCH)?;
        }
        let rig = vr.rig(&whitelist)?;
        let frame = scan::rendered_at(&mut rig.tap, yaw, PITCH, Duration::from_secs(1))?;
        let stereo = Stereo::from_frame(&frame, 2).ok_or_else(|| anyhow::anyhow!("not an 8-bit frame"))?;
        let disp = stereo.disparity(&SgmParams::default());
        let eye = rig.hmd.state.head.position;
        // The floor: from the last survey, else the usual eye height in stereo units.
        let floor = eye[1] - 1.93;
        let metres = match rig.osc.as_ref().map(|o| o.eye_height()) {
            Some(Ok(h)) if h > 0.0 => h as f32 / 1.93,
            _ => 1.0,
        };
        let Some(ocr) = rig.ocr.as_ref() else { anyhow::bail!("following needs OCR") };
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
        // above the floor to the eyes)?
        let mut nearest = f32::INFINITY;
        let (fs, fc) = yaw.to_radians().sin_cos();
        for (p, _) in stereo.points(&disp, 2) {
            let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
            let ahead = dx * fs - dz * fc;
            let side = dx * fc + dz * fs;
            if ahead > 0.0 && side.abs() * metres < 0.3 && p[1] > floor + 0.3 / metres && p[1] < eye[1] {
                nearest = nearest.min(ahead * metres);
            }
        }
        let Some(t) = seen.iter().filter(|s| match_score(&s.name, target) >= 0.6).max_by(|a, b| a.score.total_cmp(&b.score)) else {
            self.inner.lock().unwrap().obstacle = nearest;
            return Ok(false);
        };
        let (dx, dz) = (t.feet[0] - eye[0], t.feet[2] - eye[2]);
        let distance = dx.hypot(dz) * metres;
        let to = dx.atan2(-dz).to_degrees();
        vr.aim(to, PITCH)?;
        drop(vr);
        let (stand, hold) = {
            let mut s = self.inner.lock().unwrap();
            s.state = "following";
            s.last_seen = Some(Instant::now());
            s.distance = distance;
            s.obstacle = nearest;
            (s.stand, s.hold)
        };
        let blocked = nearest < OBSTACLE_M && nearest < distance;
        let axis = if hold {
            0.0
        } else if distance > stand + WALK_MARGIN && !blocked {
            (0.35 + 0.25 * (distance - stand)).clamp(0.35, 1.0)
        } else if distance < stand - BACK_MARGIN {
            -0.4
        } else {
            0.0
        };
        self.walk(bridge, axis);
        Ok(true)
    }

    /// Looks all around for the target; turns to them if found.
    fn look_around(&self, bridge: &Arc<Bridge>, target: &str) -> anyhow::Result<()> {
        let whitelist = bridge.social.whitelist_names();
        let mut vr = bridge.vr.lock().unwrap();
        vr.survey(&whitelist, true)?;
        let found = vr.survey.as_ref().and_then(|s| {
            s.players
                .iter()
                .filter(|p| match_score(&p.name, target) >= 0.6)
                .max_by(|a, b| a.score.total_cmp(&b.score))
                .map(|p| {
                    let (dx, dz) = (p.feet[0] - s.eye[0], p.feet[2] - s.eye[2]);
                    dx.atan2(-dz).to_degrees()
                })
        });
        if let Some(to) = found {
            vr.aim(to, PITCH)?;
        }
        Ok(())
    }

    fn walk(&self, bridge: &Arc<Bridge>, axis: f32) {
        let mut s = self.inner.lock().unwrap();
        if s.moving != axis {
            let _ = bridge.osc.send_f32("/input/Vertical", axis);
            s.moving = axis;
        }
    }
}
