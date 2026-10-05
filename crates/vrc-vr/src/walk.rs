//! Walking a measured leg: face a way (by turning the head; the playspace
//! never turns, so the tracking space stays aligned with the world), push
//! the thumbstick, and integrate the avatar's own speed (OSCQuery
//! `VelocityX/Z`, world m/s) until the leg is walked, or the avatar stops
//! moving while pushed (something in the way: the collision sensor).

use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::osc::Osc;
use crate::pose::Pose;
use crate::remote::RemoteHmd;

/// How a leg went.
#[derive(Clone, Debug)]
pub struct Leg {
    /// World metres walked (speed integrated).
    pub walked: f32,
    /// Stopped by something while pushing.
    pub blocked: bool,
    pub took: Duration,
    /// (seconds, VelocityX, VelocityZ) samples, for tuning.
    pub samples: Vec<(f32, f32, f32)>,
}

/// Leg settings.
#[derive(Clone, Copy, Debug)]
pub struct WalkParams {
    /// Forward thumbstick 0..1.
    pub axis: f32,
    /// Speed (world m/s) under which a pushed avatar counts as stopped...
    pub stuck_speed: f32,
    /// ...for this long (after the start-up).
    pub stuck_for: Duration,
    /// Time to get up to speed, not judged.
    pub start_up: Duration,
    /// Seconds of motion the avatar keeps after letting go (stop early by it).
    pub coast: f32,
}

impl Default for WalkParams {
    fn default() -> Self {
        WalkParams {
            axis: 0.6,
            stuck_speed: 0.15,
            stuck_for: Duration::from_millis(400),
            start_up: Duration::from_millis(350),
            coast: 0.08,
        }
    }
}

/// Faces `yaw_deg` (head, level) and walks `metres` (world) forward.
pub fn leg(hmd: &mut RemoteHmd, osc: &Osc, yaw_deg: f32, metres: f32, p: &WalkParams) -> Result<Leg> {
    let head = hmd.state.head.position;
    // The whole body faces the way: the hands too.
    hmd.state.hands_at_rest(head, yaw_deg);
    hmd.set_head(Pose::looking(yaw_deg, 0.0, head))?;
    sleep(Duration::from_millis(60)); // a frame or two for the body to follow
    let started = Instant::now();
    let mut last = started;
    let mut walked = 0.0f32;
    let mut slow_since: Option<Instant> = None;
    let mut samples = Vec::new();
    let mut blocked = false;
    let result = (|| -> Result<()> {
        osc.send_f32("/input/Vertical", p.axis)?;
        loop {
            sleep(Duration::from_millis(40));
            let now = Instant::now();
            let vx = osc.query("/avatar/parameters/VelocityX").unwrap_or(0.0) as f32;
            let vz = osc.query("/avatar/parameters/VelocityZ").unwrap_or(0.0) as f32;
            let speed = vx.hypot(vz);
            walked += speed * (now - last).as_secs_f32();
            last = now;
            samples.push(((now - started).as_secs_f32(), vx, vz));
            if walked + speed * p.coast >= metres {
                return Ok(());
            }
            if now - started > p.start_up && speed < p.stuck_speed {
                let since = *slow_since.get_or_insert(now);
                if now - since >= p.stuck_for {
                    blocked = true;
                    return Ok(());
                }
            } else {
                slow_since = None;
            }
            if now - started > Duration::from_secs_f32(4.0 + metres * 2.0) {
                blocked = true; // far too slow: something is wrong
                return Ok(());
            }
        }
    })();
    osc.send_f32("/input/Vertical", 0.0)?;
    result?;
    // The coast after letting go counts too.
    let stop = Instant::now();
    while stop.elapsed() < Duration::from_millis(400) {
        sleep(Duration::from_millis(40));
        let now = Instant::now();
        let vx = osc.query("/avatar/parameters/VelocityX").unwrap_or(0.0) as f32;
        let vz = osc.query("/avatar/parameters/VelocityZ").unwrap_or(0.0) as f32;
        let speed = vx.hypot(vz);
        walked += speed * (now - last).as_secs_f32();
        last = now;
        samples.push(((now - started).as_secs_f32(), vx, vz));
        if speed < 0.05 {
            break;
        }
    }
    Ok(Leg { walked, blocked, took: started.elapsed(), samples })
}
