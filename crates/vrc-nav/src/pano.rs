//! A survey from one pano frame (the avatar's six cameras: the whole
//! sphere, in colour and metric depth, in every frame; decision D36):
//! no head scan, no stereo. The height map and the candidates come from
//! the depth's points, in the tracking space as the head scan's do, so
//! everything after (candidates, `goto`, the lasting map, the pictures) is
//! the same.
//!
//! Who is who is the caller's (the bridge reads the names: the user
//! camera's lens, the plates over the eyes; and places them in the depth,
//! `vrc_pano::people`): named people, names with a bearing alone (nobody
//! found under the plate).

use std::sync::Arc;
use std::time::Instant;

use anyhow::Result;
use vrc_pano::{Body, Cloud, PanoFrame, Tracking};
use vrc_players::{DetectClient, ObjectSighting, Sighting};
use vrc_scene::{candidates, CandidateParams, HeightMap, Person};
use vrc_stereo::Floor;
use vrc_vr::remote::FLOOR_Y;
use vrc_vr::tap::EyeFrame;

use crate::{world_params, Survey, SurveyOptions, Timings, CLEARANCE_M};

/// What a pano survey is made from.
pub struct PanoInput {
    pub frame: Arc<PanoFrame>,
    /// The tapped frame it was decoded from (the eyes' tracking poses).
    pub eyes: Arc<EyeFrame>,
    pub cloud: Cloud,
    /// People placed under their plates (world).
    pub named: Vec<(String, Body)>,
    /// Names read with nobody found under the plate: (name, world yaw from
    /// the head).
    pub bearings: Vec<(String, f32)>,
    /// The room's players (VRChat's log).
    pub room: Vec<String>,
}

/// What a pano survey keeps of its frame.
pub struct PanoLook {
    pub frame: Arc<PanoFrame>,
    pub tracking: Tracking,
    /// The depth's points (tracking space).
    pub points: Vec<[f32; 3]>,
    /// Names with a bearing alone: (name, tracking yaw).
    pub bearings: Vec<(String, f32)>,
}

/// Points of every this-th pixel (both ways) go into the height map (the
/// floor near the bot is in the down face, at 3 pixels a degree).
const MAP_STEP: u32 = 2;
/// Objects are placed by the depth inside the middle of their boxes, the
/// nearest part (a sofa's box holds the wall over its back).
const MID_X: (f32, f32) = (0.25, 0.75);
const MID_Y: (f32, f32) = (0.2, 0.85);
const OBJECT_NEAR_M: f32 = 0.35;
const OBJECT_POINTS: usize = 20;

/// The survey of a pano frame. `blocked`: points (tracking space) a walk
/// was stopped at.
/// `whitelist` in priority order; `detect`: the things in view (the
/// lasting map's), with `opts.objects`.
pub fn survey_pano(input: PanoInput, opts: &SurveyOptions, blocked: &[[f32; 2]], whitelist: &[String], detect: Option<&DetectClient>) -> Result<Survey> {
    let t = Instant::now();
    let tracking = Tracking::new(&input.frame, &input.eyes, input.cloud.floor, FLOOR_Y);
    let metres = tracking.metres;
    let eye = tracking.head_track;
    let yaw = input.eyes.views[0].pose.yaw_pitch().0;
    let points: Vec<[f32; 3]> = input.frame.points(MAP_STEP).into_iter().map(|p| tracking.point(p.world)).collect();
    // The floor: the depth's own under the bot, else the headset's.
    let floor_h = input.cloud.floor.map_or(FLOOR_Y, |f| tracking.point([tracking.head_world[0], f, tracking.head_world[2]])[1]);
    let floor = Floor { height: floor_h, tilt_deg: 0.0, inliers: points.len() };
    let mut map = HeightMap::new(world_params().in_units(metres), [eye[0], eye[2]], floor_h);
    map.add(&points, eye);
    for &[x, z] in blocked {
        map.mark_blocked(x, z, 0.15 / metres);
    }
    let map_time = t.elapsed();
    let players: Vec<Sighting> = input
        .named
        .iter()
        .map(|(name, b)| Sighting {
            name: name.clone(),
            text: name.clone(),
            score: 1.0,
            whitelist_rank: whitelist.iter().position(|n| n == name).map(|i| i + 1),
            tag: tracking.point(b.plate()),
            feet: tracking.point(b.feet),
            bbox: [0.0; 4],
            seen_ns: input.frame.capture_ns,
        })
        .collect();
    let t = Instant::now();
    let objects = match detect {
        Some(d) if opts.objects => place_objects(d, &input.frame, &tracking),
        _ => Vec::new(),
    };
    let detect = t.elapsed();
    let t = Instant::now();
    let people: Vec<Person> = players.iter().map(|s| Person { name: s.name.clone(), whitelist_rank: s.whitelist_rank, feet: s.feet }).collect();
    let cparams = CandidateParams { clearance: CLEARANCE_M / metres, min_distance: 0.8 / metres, min_gap: 0.8 / metres, ..Default::default() };
    let candidates = candidates(&map, eye, yaw, &people, &cparams);
    let timings = Timings { map: map_time + t.elapsed(), detect, ..Default::default() };
    let look = PanoLook {
        bearings: input.bearings.iter().map(|(n, y)| (n.clone(), tracking.yaw(*y))).collect(),
        frame: input.frame,
        tracking,
        points,
    };
    Ok(Survey {
        shots: Vec::new(),
        pairs: Vec::new(),
        eye,
        yaw,
        floor,
        map,
        players,
        objects,
        candidates,
        metres,
        room: input.room,
        timings,
        pano: Some(look),
    })
}

/// The detector over the four level faces (perspective views: what it was
/// trained on, unlike the panorama), each thing placed by the depth in its
/// box (tracking space).
fn place_objects(detect: &DetectClient, frame: &PanoFrame, tracking: &Tracking) -> Vec<ObjectSighting> {
    let mut out = Vec::new();
    for (k, v) in frame.views.iter().enumerate().filter(|(_, v)| v.face.yaw().is_some()) {
        let found = match detect.detect_rgb(&v.rgb, v.width, v.height) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("detection failed: {e:#}");
                break;
            }
        };
        for d in found {
            let [x, y, w, h] = d.bbox;
            let (i0, i1) = ((x + MID_X.0 * w) as u32, ((x + MID_X.1 * w) as u32).min(v.width));
            let (j0, j1) = ((y + MID_Y.0 * h) as u32, ((y + MID_Y.1 * h) as u32).min(v.height));
            let mut pts: Vec<(f32, [f32; 3])> = Vec::new();
            for j in (j0..j1).step_by(2) {
                for i in (i0..i1).step_by(2) {
                    if let (Some(z), Some(p)) = (v.depth_at(i, j), frame.point(k, i, j)) {
                        pts.push((z, p));
                    }
                }
            }
            if pts.len() < OBJECT_POINTS {
                continue;
            }
            pts.sort_by(|a, b| a.0.total_cmp(&b.0));
            // The nearest part: within OBJECT_NEAR_M of the 20th percentile.
            let front = pts[pts.len() / 5].0;
            let near: Vec<[f32; 3]> = pts.iter().filter(|q| q.0 <= front + OBJECT_NEAR_M).map(|q| q.1).collect();
            let n = near.len() as f32;
            let centre = [near.iter().map(|p| p[0]).sum::<f32>() / n, 0.0, near.iter().map(|p| p[2]).sum::<f32>() / n];
            let foot = near.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
            let [fx, ..] = v.intrinsics;
            let size = [w / fx * front, h / fx * front];
            out.push(ObjectSighting {
                label: d.label,
                score: d.confidence,
                at: tracking.point([centre[0], foot, centre[2]]),
                size: size.map(|s| s / tracking.metres),
                bbox: d.bbox,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use vrc_pano::synth::{person, Scene};
    use vrc_pano::{decode, PanoParams, PeopleParams};
    use vrc_scene::{Cell, Kind};
    use vrc_vr::Pose;

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    /// The synthetic room with someone (Ann, 1.6 m tall) 2.5 m ahead of the
    /// head and a table; the eyes 1.6 m up in the tracking space, looking
    /// along its -z.
    fn room() -> (Scene, PanoInput, f32) {
        let mut s = Scene::room();
        let ahead = s.head_yaw;
        let (sn, cs) = ahead.to_radians().sin_cos();
        let (x, z) = (s.position[0] + 2.5 * sn, s.position[2] + 2.5 * cs);
        s.boxes.push(person(&s, x, z, 1.6));
        let y = s.room_min[1];
        s.boxes.push(([4.0, y, -1.0], [5.0, y + 0.75, -0.2], [120, 90, 60]));
        let mut eyes = s.render();
        for v in eyes.views.iter_mut() {
            v.pose = Pose::looking(0.0, 0.0, [0.0, 1.6, 0.0]);
        }
        let frame = Arc::new(decode(&eyes, &PanoParams::default()).unwrap());
        let cloud = Cloud::new(&frame, 4);
        let head = frame.head.position;
        let plate = [x, y + 1.6 + 0.4, z];
        let dir = [plate[0] - head[0], plate[1] - head[1], plate[2] - head[2]];
        let body = cloud.person_along(head, dir, &PeopleParams::default()).expect("Ann under her plate");
        let input = PanoInput {
            frame,
            eyes: Arc::new(eyes),
            cloud,
            named: vec![("Ann".into(), body)],
            bearings: vec![("Bob".into(), (s.head_yaw + 90.0).rem_euclid(360.0))],
            room: vec!["Ann".into(), "Bob".into()],
        };
        (s, input, ahead)
    }

    #[test]
    fn a_survey_of_one_pano_frame() {
        let (s, input, _) = room();
        let opts = SurveyOptions { objects: false, ..Default::default() };
        let v = survey_pano(input, &opts, &[], &["Ann".to_string()], None).unwrap();
        // World metres a tracking unit: the eyes 1.19 m over the floor in
        // the world, 1.92 in the tracking space.
        let eye_world = s.position[1] - 0.01 - s.room_min[1];
        assert!(close(v.metres, eye_world / (1.6 - FLOOR_Y), 0.01), "{}", v.metres);
        // The floor where the headset's is (1.19 m under the eyes).
        assert!(close((v.eye[1] - v.floor.height) * v.metres, eye_world, 0.03), "{:?}", v.floor);
        assert!(close(v.yaw, 0.0, 0.01));
        // Ann: straight ahead, 2.5 m; a numbered player place.
        let ann = &v.players[0];
        assert_eq!((ann.name.as_str(), ann.whitelist_rank), ("Ann", Some(1)));
        let (dx, dz) = (ann.feet[0] - v.eye[0], ann.feet[2] - v.eye[2]);
        assert!(close(dx.hypot(dz) * v.metres, 2.5, 0.15) && close(dx.atan2(-dz).to_degrees(), 0.0, 4.0), "{ann:?}");
        let c = v.candidates.iter().find(|c| c.kind == Kind::Player).expect("Ann's place");
        assert_eq!(c.name.as_deref(), Some("Ann"));
        assert!(close(c.distance * v.metres, 2.5, 0.2) && c.bearing.abs() < 5.0, "{c:?}");
        // Bob read along a bearing alone: 90 degrees right.
        let look = v.pano.as_ref().unwrap();
        assert!(close(look.bearings[0].1, 90.0, 0.01), "{:?}", look.bearings);
        // The height map: floor ahead, the room's +z wall (4.5 m off along
        // world yaw 0; the map reaches 8 tracking units, 5 m here) an
        // obstacle.
        let at = |world_yaw: f32, d: f32| {
            let t = look.tracking.yaw(world_yaw).to_radians();
            let u = d / v.metres;
            v.map.index(v.eye[0] + u * t.sin(), v.eye[2] - u * t.cos()).unwrap()
        };
        assert_eq!(v.map.cell(at(s.head_yaw + 30.0, 1.5)).0, Cell::Floor);
        assert_eq!(v.map.cell(at(0.0, 4.45)).0, Cell::Obstacle);
        let [unknown, floor, obstacle] = v.map.census();
        assert!(floor > 1000 && obstacle > 100, "unknown {unknown}, floor {floor}, obstacle {obstacle}");
        // The panorama: the whole sphere; the lasting map takes one look.
        assert!(v.panorama(512).coverage > 0.99);
        assert_eq!(crate::observations(&v).len(), 1);
    }
}
