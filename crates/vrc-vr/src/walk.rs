//! Walking a measured leg: face a way (by turning the head; the playspace
//! never turns, so the tracking space stays aligned with the world), push
//! the thumbstick, and integrate the avatar's own speed (OSCQuery
//! `VelocityX/Z`, world m/s) until the leg is walked, or the avatar stops
//! moving while pushed (something in the way: the collision sensor).

use std::sync::atomic::{AtomicU64, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::osc::Osc;
use crate::pose::Pose;
use crate::remote::RemoteHmd;

/// Stops so far: a walk started before the latest one ends.
static STOPS: AtomicU64 = AtomicU64::new(0);

/// Stops every walk going on (its leg lets go of the stick at once; a walk
/// of several legs takes no further one).
pub fn stop_all() {
    STOPS.fetch_add(1, Ordering::SeqCst);
}

/// Where the stops are now: give it to [`stopped_since`].
pub fn stops() -> u64 {
    STOPS.load(Ordering::SeqCst)
}

/// Whether a stop came after `stops` was taken.
pub fn stopped_since(stops: u64) -> bool {
    STOPS.load(Ordering::SeqCst) != stops
}

/// How a leg went.
#[derive(Clone, Debug)]
pub struct Leg {
    /// World metres walked (speed integrated).
    pub walked: f32,
    /// Stopped by something while pushing.
    pub blocked: bool,
    /// Stopped by [`stop_all`].
    pub stopped: bool,
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
    leg_since(hmd, osc, yaw_deg, metres, p, stops())
}

/// [`leg`], stopped by any stop after `begun` ([`stops`] when the walk was
/// asked for: a stop while it waited counts).
pub fn leg_since(hmd: &mut RemoteHmd, osc: &Osc, yaw_deg: f32, metres: f32, p: &WalkParams, begun: u64) -> Result<Leg> {
    leg_facing(hmd, osc, yaw_deg, 0.0, metres, p, begun)
}

/// [`leg_since`] facing `yaw_deg` but walking `way_deg` off it (+ right):
/// 180 steps back, 90 to the right, the head and body still facing
/// `yaw_deg` (the thumbstick pushed that way).
pub fn leg_facing(hmd: &mut RemoteHmd, osc: &Osc, yaw_deg: f32, way_deg: f32, metres: f32, p: &WalkParams, begun: u64) -> Result<Leg> {
    anyhow::ensure!(yaw_deg.is_finite() && way_deg.is_finite() && metres.is_finite(), "a walk needs a finite way and length");
    let head = hmd.state.head.position;
    // The whole body faces the way: the hands too.
    hmd.state.hands_at_rest(head, yaw_deg);
    hmd.set_head(Pose::looking(yaw_deg, 0.0, head))?;
    let (side, ahead) = way_deg.to_radians().sin_cos();
    let (vertical, horizontal) = (p.axis * ahead, p.axis * side);
    sleep(Duration::from_millis(60)); // a frame or two for the body to follow
    let started = Instant::now();
    let mut last = started;
    let mut walked = 0.0f32;
    let mut slow_since: Option<Instant> = None;
    let mut samples = Vec::new();
    let mut blocked = false;
    let mut stopped = false;
    let result = (|| -> Result<()> {
        if stopped_since(begun) {
            stopped = true;
            return Ok(());
        }
        osc.send_f32("/input/Vertical", vertical)?;
        if horizontal.abs() > 1e-3 {
            osc.send_f32("/input/Horizontal", horizontal)?;
        }
        loop {
            sleep(Duration::from_millis(40));
            if stopped_since(begun) {
                stopped = true;
                return Ok(());
            }
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
    let released = osc.send_f32("/input/Vertical", 0.0).and(osc.send_f32("/input/Horizontal", 0.0));
    result?;
    released?;
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
    Ok(Leg { walked, blocked, stopped, took: started.elapsed(), samples })
}
