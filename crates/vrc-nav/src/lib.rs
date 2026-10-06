//! The bot's capabilities around moving, for the bridge to offer (the
//! AstrBot plugin turns them into the model's tools; the Codex agent in
//! AstrBot decides):
//!
//! - [`survey`]: look all around by turning the head, and know what is
//!   there: a panorama, a height map from stereo, the room's players by
//!   their name tags (whitelisted friends marked), and numbered candidates
//!   (places and players) to pick from.
//! - [`goto`]: walk to a point of a survey: short legs along a planned
//!   path, each measured by the avatar's own speed, a fresh survey and plan
//!   after each, a blocked leg remembered as an obstacle.
//!
//! Coordinates are the tracking space's (the head turns, the playspace
//! never does, so its axes stay fixed to the world; walking moves the world
//! past the head). Distances inside are stereo units; `Survey::metres`
//! turns them into world metres.

pub mod render;

use std::path::PathBuf;
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use vrc_players::{OcrClient, Sighting};
use vrc_scene::candidates::{paths, stopped, walkable};
use vrc_scene::{candidates, Candidate, CandidateParams, HeightMap, MapParams, Person};
use vrc_stereo::{fit_floor, Disparity, Floor, SgmParams, Stereo};
use vrc_vr::osc::Osc;
use vrc_vr::remote::RemoteHmd;
use vrc_vr::scan::{self, Shot};
use vrc_vr::tap::EyeTap;
use vrc_vr::walk::{self, WalkParams};

/// What the capabilities drive: the virtual headset, its eyes, VRChat's
/// OSC, and (optionally) OCR for name tags.
pub struct Rig {
    pub hmd: RemoteHmd,
    pub tap: EyeTap,
    pub osc: Option<Osc>,
    pub ocr: Option<OcrClient>,
    /// Display names in priority order.
    pub whitelist: Vec<String>,
    /// VRChat's log directory (the room's players).
    pub log_dir: PathBuf,
}

/// How long the avatar's arms take to follow the hands (VRChat's IK).
const ARMS_SETTLE: Duration = Duration::from_millis(150);

/// Room kept from obstacles, world metres (half a body's width and a bit).
pub const CLEARANCE_M: f32 = 0.25;

/// The map's thresholds in world metres: VRChat's default walking (steps
/// of about 0.3 m, jumps of about 0.5 m), the avatar's body around the eyes.
pub fn world_params() -> MapParams {
    MapParams { step: 0.3, drop: 0.8, jump: 0.5, self_radius: 0.8, ..Default::default() }
}

#[derive(Clone, Debug)]
pub struct SurveyOptions {
    /// Views in the ring.
    pub count: usize,
    /// Only the one view ahead (and the look down), not the ring.
    pub ahead: bool,
    pub pitch: f32,
    /// Also look down at the feet.
    pub down: bool,
    /// Read name tags (needs the rig's OCR).
    pub players: bool,
    /// Match the stereo at 1/scale of the eye size (0: about
    /// `vrc_stereo::MATCH_WIDTH` wide, whatever the eye size).
    pub stereo_scale: usize,
}

impl Default for SurveyOptions {
    fn default() -> Self {
        SurveyOptions { count: 5, ahead: false, pitch: -10.0, down: true, players: true, stereo_scale: 0 }
    }
}

/// What a survey found.
pub struct Survey {
    pub shots: Vec<Shot>,
    pub pairs: Vec<(Stereo, Disparity)>,
    /// The eyes' centre (head position) and heading.
    pub eye: [f32; 3],
    pub yaw: f32,
    pub floor: Floor,
    pub map: HeightMap,
    pub players: Vec<Sighting>,
    pub candidates: Vec<Candidate>,
    /// World metres per stereo unit (1.0 when VRChat did not say).
    pub metres: f32,
    pub room: Vec<String>,
    pub timings: Timings,
}

#[derive(Clone, Debug, Default)]
pub struct Timings {
    pub scan: Duration,
    pub stereo: Duration,
    pub ocr: Duration,
    pub map: Duration,
}

/// Looks all around and works out what is there. `blocked` are points
/// (tracking space) known to be in the way though not seen (a walk was
/// stopped there).
pub fn survey(rig: &mut Rig, opts: &SurveyOptions, blocked: &[[f32; 2]]) -> Result<Survey> {
    let head = rig.hmd.state.head;
    let (yaw, _) = head.yaw_pitch();
    let mut views = if opts.ahead { vec![(yaw, opts.pitch)] } else { scan::ring(opts.count, opts.pitch) };
    if opts.down {
        views.push((yaw, -80.0));
    }
    // Arms down at the sides, out of the views.
    rig.hmd.state.hands_at_rest(head.position, yaw);
    rig.hmd.send()?;
    sleep(ARMS_SETTLE);
    let t = Instant::now();
    let shots = scan::scan(&mut rig.hmd, &mut rig.tap, &views, Duration::from_secs(2))?;
    let mut timings = Timings { scan: t.elapsed(), ..Default::default() };

    let t = Instant::now();
    let mut points = Vec::new();
    let mut pairs = Vec::new();
    for s in &shots {
        let scale = if opts.stereo_scale == 0 { vrc_stereo::match_scale(s.frame.width) } else { opts.stereo_scale };
        let stereo = Stereo::from_frame(&s.frame, scale).context("not an 8-bit frame")?;
        let disp = stereo.disparity(&SgmParams::default());
        points.extend(stereo.points(&disp, 1).into_iter().map(|(p, _)| p));
        pairs.push((stereo, disp));
    }
    timings.stereo = t.elapsed();
    let eye = head.position;
    let floor = fit_floor(&points, eye, 0.5).context("no floor in sight")?;

    let t = Instant::now();
    let mut room = Vec::new();
    let mut players = Vec::new();
    if opts.players {
        if let Some(ocr) = &rig.ocr {
            room = vrc_players::room::players(&rig.log_dir).unwrap_or_default();
            if !room.is_empty() {
                let mut all = Vec::new();
                for (s, (stereo, disp)) in shots.iter().zip(&pairs) {
                    let rgb = s.frame.eye_rgb8(0)?;
                    let lines = ocr.lines_rgb(&rgb, s.frame.width as u16, s.frame.height as u16)?;
                    all.extend(vrc_players::sightings(&s.frame, stereo, disp, &lines, &room, &rig.whitelist, floor.height));
                }
                players = vrc_players::merge(all);
            }
        }
    }
    timings.ocr = t.elapsed();

    // World metres per stereo unit; a floor found nonsense (at or over
    // the eyes) or an eye height not read leaves it 1.
    let metres = match rig.osc.as_ref().map(|o| o.eye_height()) {
        Some(Ok(h)) if h > 0.0 && eye[1] - floor.height > 0.1 => h as f32 / (eye[1] - floor.height),
        _ => 1.0,
    }
    .clamp(0.1, 10.0);

    let t = Instant::now();
    let mut map = HeightMap::new(world_params().in_units(metres), [eye[0], eye[2]], floor.height);
    map.add(&points, eye);
    for &[x, z] in blocked {
        map.mark_blocked(x, z, 0.15 / metres);
    }
    let people: Vec<Person> = players
        .iter()
        .map(|s| Person { name: s.name.clone(), whitelist_rank: s.whitelist_rank, feet: s.feet })
        .collect();
    let cparams = CandidateParams { clearance: CLEARANCE_M / metres, min_distance: 0.8 / metres, min_gap: 0.8 / metres, ..Default::default() };
    let candidates = candidates(&map, eye, yaw, &people, &cparams);
    timings.map = t.elapsed();
    Ok(Survey { shots, pairs, eye, yaw, floor, map, players, candidates, metres, room, timings })
}

impl Survey {
    /// The candidates as the model sees them (world metres, degrees).
    pub fn candidates_json(&self) -> serde_json::Value {
        let m = self.metres as f64;
        let r2 = |v: f32| (v as f64 * m * 100.0).round() / 100.0;
        serde_json::Value::Array(
            self.candidates
                .iter()
                .map(|c| {
                    serde_json::json!({
                        "id": c.id,
                        "kind": c.kind.name(),
                        "name": c.name,
                        "whitelist_rank": c.whitelist_rank,
                        "distance_m": r2(c.distance),
                        "bearing_deg": c.bearing.round() as f64,
                        "walk_m": if c.path.is_finite() { serde_json::json!(r2(c.path)) } else { serde_json::Value::Null },
                        "height_m": r2(c.position[1] - self.floor.height),
                        "rise_m": c.rise.map(r2),
                    })
                })
                .collect(),
        )
    }
}

#[derive(Clone, Debug)]
pub struct GotoOptions {
    /// Close enough, world metres.
    pub arrive: f32,
    /// Longest leg between surveys, world metres.
    pub leg: f32,
    pub max_legs: usize,
    pub walk: WalkParams,
    /// [`walk::stops`] when the walk was asked for (a stop since ends it);
    /// `None`: when `goto` starts.
    pub since: Option<u64>,
}

impl Default for GotoOptions {
    fn default() -> Self {
        GotoOptions { arrive: 0.6, leg: 1.6, max_legs: 12, walk: WalkParams::default(), since: None }
    }
}

/// How a walk to a point went.
#[derive(Clone, Debug)]
pub struct GotoReport {
    pub arrived: bool,
    /// Left to go (world metres), straight line.
    pub remaining: f32,
    pub legs: Vec<LegReport>,
    pub took: Duration,
    /// Why it stopped short, if it did.
    pub reason: Option<String>,
}

#[derive(Clone, Debug)]
pub struct LegReport {
    pub yaw: f32,
    /// World metres planned and walked.
    pub planned: f32,
    pub walked: f32,
    pub blocked: bool,
    /// Straight distance left before the leg (world metres).
    pub before: f32,
}

/// Walks to `target` (x, z in the tracking space as of `first`, the survey
/// it was picked from).
pub fn goto(rig: &mut Rig, first: Survey, target: [f32; 2], opts: &GotoOptions) -> Result<GotoReport> {
    let osc = rig.osc.as_ref().context("walking needs VRChat's OSC")?;
    let osc = Osc::with_ports_from(osc)?;
    let started = Instant::now();
    let begun = opts.since.unwrap_or_else(walk::stops);
    let mut target = target;
    let mut blocked: Vec<[f32; 2]> = Vec::new();
    let mut legs = Vec::new();
    let mut s = first;
    let survey_opts = SurveyOptions { players: false, ..Default::default() };
    for _ in 0..opts.max_legs {
        let eye = s.eye;
        let m = s.metres;
        let left = (target[0] - eye[0]).hypot(target[1] - eye[2]) * m;
        if left <= opts.arrive {
            return Ok(GotoReport { arrived: true, remaining: left, legs, took: started.elapsed(), reason: None });
        }
        let Some((yaw, wp_dist)) = next_waypoint(&s.map, eye, target, CLEARANCE_M / m, opts.leg / m) else {
            return Ok(GotoReport {
                arrived: false,
                remaining: left,
                legs,
                took: started.elapsed(),
                reason: Some("no way there from here".into()),
            });
        };
        let planned = (wp_dist * m).min(opts.leg);
        if walk::stopped_since(begun) {
            return Ok(GotoReport { arrived: false, remaining: left, legs, took: started.elapsed(), reason: Some("stopped".into()) });
        }
        let leg = walk::leg_since(&mut rig.hmd, &osc, yaw, planned, &opts.walk, begun)?;
        // The world moved past the head: so did the target and what blocked us.
        let d = leg.walked / m;
        let (sy, cy) = yaw.to_radians().sin_cos();
        let (dx, dz) = (sy * d, -cy * d);
        target = [target[0] - dx, target[1] - dz];
        for b in &mut blocked {
            *b = [b[0] - dx, b[1] - dz];
        }
        if leg.blocked {
            // Something we could not see is just ahead: a wall across the
            // way (a mirror, glass), not a post to walk round.
            for k in -4..=4 {
                let (ahead, across) = (0.4 / m, k as f32 * 0.15 / m);
                blocked.push([eye[0] + sy * ahead + cy * across, eye[2] - cy * ahead + sy * across]);
            }
        }
        legs.push(LegReport { yaw, planned, walked: leg.walked, blocked: leg.blocked, before: left });
        if leg.stopped {
            let remaining = (left - leg.walked).max(0.0);
            return Ok(GotoReport { arrived: false, remaining, legs, took: started.elapsed(), reason: Some("stopped".into()) });
        }
        s = survey(rig, &survey_opts, &blocked)?;
    }
    let left = (target[0] - s.eye[0]).hypot(target[1] - s.eye[2]) * s.metres;
    Ok(GotoReport {
        arrived: left <= opts.arrive,
        remaining: left,
        legs,
        took: started.elapsed(),
        reason: (left > opts.arrive).then(|| "too many legs".into()),
    })
}

/// Heading (degrees) and distance (stereo units) of the next leg toward
/// `target`: the farthest point of the planned path, within `max_leg`, that
/// a straight walk reaches over walkable cells. Toward the nearest
/// reachable cell when the target itself is not reachable.
pub fn next_waypoint(map: &HeightMap, eye: [f32; 3], target: [f32; 2], clearance: f32, max_leg: f32) -> Option<(f32, f32)> {
    let n = map.size;
    let (dist, prev) = paths(map, eye, clearance);
    let free = walkable(map, clearance);
    let halt = stopped(map);
    let ground = map.grounds();
    let centre = |i: usize| {
        let half = n as f32 / 2.0;
        [
            map.origin[0] + ((i % n) as f32 + 0.5 - half) * map.params.cell,
            map.origin[1] + ((i / n) as f32 + 0.5 - half) * map.params.cell,
        ]
    };
    // The goal: the target's cell, or the reachable cell nearest it.
    let goal = map
        .index(target[0], target[1])
        .filter(|&i| dist[i].is_some())
        .or_else(|| {
            (0..n * n).filter(|&i| dist[i].is_some()).min_by(|&a, &b| {
                let (pa, pb) = (centre(a), centre(b));
                ((pa[0] - target[0]).hypot(pa[1] - target[1])).total_cmp(&(pb[0] - target[0]).hypot(pb[1] - target[1]))
            })
        })?;
    // The path back from the goal, the way the walk distances came.
    let mut path = vec![goal];
    let mut at = goal;
    while prev[at] != usize::MAX && path.len() <= n * n {
        at = prev[at];
        path.push(at);
    }
    path.reverse(); // from the start ring to the goal
    // The farthest path cell within reach and in a straight line of sight:
    // over walkable cells, no step up or down on the way more than the
    // walk takes, and not through where a walk was stopped.
    let (step, drop) = (map.params.step, map.params.drop);
    let line_clear = |to: [f32; 2]| {
        let (dx, dz) = (to[0] - eye[0], to[1] - eye[2]);
        let steps = ((dx.hypot(dz)) / (map.params.cell * 0.5)).ceil() as usize;
        let mut last = map.floor;
        (0..=steps).all(|k| {
            let t = k as f32 / steps.max(1) as f32;
            let (x, z) = (eye[0] + dx * t, eye[2] + dz * t);
            let Some(i) = map.index(x, z) else { return false };
            if halt[i] {
                return false;
            }
            let near = (x - eye[0]).hypot(z - eye[2]) < map.params.self_radius + 2.0 * map.params.cell;
            if near {
                return true;
            }
            let Some(h) = ground[i].filter(|_| free[i]) else { return false };
            let rise = h - last;
            last = h;
            rise <= step && -rise <= drop
        })
    };
    let mut pick = None;
    for &i in &path {
        let p = centre(i);
        let d = (p[0] - eye[0]).hypot(p[1] - eye[2]);
        if d > max_leg {
            break;
        }
        if line_clear(p) {
            pick = Some(p);
        }
    }
    let p = pick.or_else(|| path.first().map(|&i| centre(i)))?;
    let (dx, dz) = (p[0] - eye[0], p[1] - eye[2]);
    let d = dx.hypot(dz);
    if d < 1e-3 {
        return None;
    }
    Some((dx.atan2(-dz).to_degrees(), d))
}

impl Rig {
    /// The whitelist, the OCR and the log directory from the usual places
    /// for the bot user (`$HOME`).
    pub fn connect(remote: &str, tap: &str, ocr_url: Option<&str>, ocr_model: &str, whitelist: Vec<String>) -> Result<Rig> {
        let home = std::env::var("HOME").context("no HOME")?;
        let osc = Osc::connect().ok();
        let ocr = match ocr_url {
            Some(url) if !url.is_empty() => Some(OcrClient::new(url, ocr_model)?),
            _ => None,
        };
        if remote.is_empty() {
            bail!("no remote driver address");
        }
        Ok(Rig {
            hmd: RemoteHmd::connect(remote)?,
            tap: EyeTap::open(tap),
            osc,
            ocr,
            whitelist,
            log_dir: PathBuf::from(home).join(vrc_vr::osc::LOG_DIR),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Flat floor all round, 3 m each way (stereo units = metres).
    fn floor() -> (HeightMap, [f32; 3]) {
        let eye = [0.0, 1.6, 0.0];
        let mut m = HeightMap::new(MapParams { radius: 4.0, ..world_params() }, [0.0, 0.0], 0.0);
        let mut pts = Vec::new();
        for i in 0..240 {
            for j in 0..240 {
                pts.push([i as f32 / 40.0 - 3.0 + 0.01, 0.0, j as f32 / 40.0 - 3.0 + 0.01]);
            }
        }
        m.add(&pts, eye);
        (m, eye)
    }

    #[test]
    fn straight_ahead_on_open_floor() {
        let (m, eye) = floor();
        let (yaw, d) = next_waypoint(&m, eye, [0.0, -2.5], CLEARANCE_M, 1.6).unwrap();
        assert!(yaw.abs() < 5.0 && d > 1.2, "{yaw} {d}");
    }

    #[test]
    fn a_stopped_walk_turns_the_next_one_aside() {
        let (mut m, eye) = floor();
        // As goto marks a leg blocked 0.4 ahead (glass across the way).
        for k in -4..=4 {
            m.mark_blocked(k as f32 * 0.15, -0.4, 0.15);
        }
        let (yaw, _) = next_waypoint(&m, eye, [0.0, -2.5], CLEARANCE_M, 1.6).unwrap();
        assert!(yaw.abs() > 20.0, "walks into the glass again: {yaw}");
    }
}
