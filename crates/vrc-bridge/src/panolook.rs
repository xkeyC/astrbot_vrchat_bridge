//! What the panorama shows, with names on it (decision D36): one pano frame
//! (the whole sphere, colour and metric depth), the people in its depth
//! (`vrc_pano::people`) and who they are. The avatar's cameras never draw
//! nameplates, so names come from elsewhere, each a ray through the plate:
//!
//! - the user camera's lens (`orbit`: the front lens, a voice turned to, a
//!   look): its name sightings, kept 60 s;
//! - the plates VRChat draws over the eyes (its UI is drawn after the
//!   avatar's HUD, at the place a plate has in the usual 100° view): read
//!   by OCR in the left eye, told from text in the tiles by the right eye's
//!   UI mask, placed through the eye's own projection;
//! - asking: the lens turned a moment to someone nobody named yet
//!   (`Orbit::name_toward`).
//!
//! A name's ray finds the person under the plate in the depth; a ray that
//! finds nobody is a name with a bearing alone. Person-shaped things with
//! no name are nobody (as "maybe someone" they were all furniture, D36).
//! Used by the surveys, the follower, the
//! sightings and the speaker tracker alike.

use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use vrc_nav::{PanoInput, Rig, Survey, SurveyOptions};
use vrc_pano::{Body, Cloud, PanoFrame, PeopleParams, Tracking, MASK_OVERLAY};
use vrc_players::{OcrClient, Sighting};
use vrc_vr::remote::FLOOR_Y;
use vrc_vr::tap::EyeFrame;

use crate::bridge::Bridge;
use crate::orbit::NameSighting;
use crate::Lock;

/// The depth's points: every this-th pixel.
const CLOUD_STEP: u32 = 4;
/// A name's person and a person-shaped thing this near are one.
const SAME_BODY_M: f32 = 0.6;
/// The UI's largest stereo disparity (pixels at 1920): its copy in the
/// left eye is up to this right of the right eye's.
const DISPARITY_PX: f32 = 50.0;

/// A name read, as a ray through its plate.
#[derive(Clone, Debug)]
pub struct PlateRay {
    pub name: String,
    /// Where it was seen from, and the way (world, unit).
    pub from: [f32; 3],
    pub dir: [f32; 3],
    pub at: Instant,
    /// "overlay" (the eyes), "lens", "asked".
    pub source: &'static str,
    /// The plate's text box in the left eye (overlay plates).
    pub bbox: Option<[f32; 4]>,
    /// The plate's world yaw from the head (the lens's sightings: taken
    /// 2.5 m out, the lens's offset from the head allowed for).
    pub head_yaw: f32,
}

impl PlateRay {
    /// From a lens sighting.
    pub fn lens(n: &NameSighting, source: &'static str) -> PlateRay {
        PlateRay { name: n.name.clone(), from: n.ray_from, dir: n.ray_dir, at: n.at, source, bbox: None, head_yaw: n.world_yaw }
    }

    /// The world yaw it points along from the head.
    pub fn yaw(&self) -> f32 {
        self.head_yaw
    }
}

/// Someone in the depth, named or not.
#[derive(Clone, Debug)]
pub struct Person {
    pub body: Body,
    pub name: Option<String>,
    /// How the name came: "overlay", "lens", "asked"; none unnamed.
    pub how: Option<&'static str>,
    /// The ray that named them (overlay ones carry the plate's box).
    pub ray: Option<PlateRay>,
}

/// One look at the panorama.
pub struct Look {
    /// When the frame was taken (read off the tap): the pose then is the
    /// look's (the follower walks on while it is worked out).
    pub taken: Instant,
    pub frame: Arc<PanoFrame>,
    pub eyes: Arc<EyeFrame>,
    pub cloud: Cloud,
    pub tracking: Tracking,
    pub people: Vec<Person>,
    /// Names read with nobody found under them: (name, world yaw from the
    /// head).
    pub bearings: Vec<(String, f32)>,
}

#[derive(Clone, Debug)]
pub struct LookOptions {
    /// Read names at all.
    pub names: bool,
    /// The plates over the eyes (an OCR of the left eye).
    pub overlay: bool,
    /// The lens's sightings this recent.
    pub lens_within: Duration,
    /// How the people near the bot (`people`) say their names came (the
    /// idle sweep's: "sweep"); none: as each was read.
    pub source: Option<&'static str>,
}

impl Default for LookOptions {
    fn default() -> Self {
        LookOptions { names: true, overlay: true, lens_within: Duration::from_secs(8), source: None }
    }
}

/// The plates over the eyes in `eyes` (OCR `lines`), as rays: the room's
/// players' names whose text lies on the UI (the right eye's mask, up to
/// the disparity left of it), through the left eye's projection.
pub fn overlay_rays(frame: &PanoFrame, eyes: &EyeFrame, tracking: &Tracking, lines: &[vrc_players::OcrLine], names: &[String], at: Instant) -> Vec<PlateRay> {
    let view = eyes.views[0];
    let [fx, fy, cx, cy] = view.fov.intrinsics(eyes.width, eyes.height);
    let scale = eyes.width as f32 / 1920.0;
    let from = tracking.to_world(view.pose.position);
    let mut out: Vec<PlateRay> = Vec::new();
    for l in lines {
        let Some((i, _)) = vrc_players::names::best_match(&l.text, names) else { continue };
        let [x, y, w, h] = l.bbox;
        if !on_the_ui(frame, [x, y, w, h], DISPARITY_PX * scale) {
            continue; // text in the world (a tile), not a plate
        }
        let (u, v) = (x + w / 2.0, y + h / 2.0);
        let d = view.pose.rotate([(u - cx) / fx, (cy - v) / fy, -1.0]);
        let d = tracking.dir_to_world(d);
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1e-6);
        let head_yaw = d[0].atan2(d[2]).to_degrees().rem_euclid(360.0);
        let ray = PlateRay { name: names[i].clone(), from, dir: d.map(|c| c / n), at, source: "overlay", bbox: Some(l.bbox), head_yaw };
        match out.iter_mut().find(|r| r.name == ray.name) {
            Some(_) => {}
            None => out.push(ray),
        }
    }
    out
}

/// Whether a text box of the left eye lies on VRChat's UI: the right eye's
/// UI mask (`MASK_OVERLAY`) under it, shifted 0..`disparity` left.
pub fn on_the_ui(frame: &PanoFrame, bbox: [f32; 4], disparity: f32) -> bool {
    let [x, y, w, h] = bbox;
    let rows = [y + 0.3 * h, y + 0.5 * h, y + 0.7 * h];
    let mut hits = 0;
    let mut tries = 0;
    for &row in &rows {
        for k in 0..=10 {
            let shift = disparity * k as f32 / 10.0;
            for col in [x + 0.25 * w, x + 0.5 * w, x + 0.75 * w] {
                let (px, py) = (col - shift, row);
                if px < 0.0 || py < 0.0 {
                    continue;
                }
                tries += 1;
                if let Some((v, i, j)) = frame.view_at(px as u32, py as u32) {
                    let m = frame.views[v].mask[(j * frame.views[v].width + i) as usize];
                    hits += (m & MASK_OVERLAY != 0) as usize;
                }
            }
        }
    }
    tries > 0 && hits * 4 >= tries
}

/// The newest of the lens's sightings per name since `since`.
pub fn lens_rays(seen: &[NameSighting]) -> Vec<PlateRay> {
    let mut out: Vec<PlateRay> = Vec::new();
    for n in seen.iter().rev() {
        if !out.iter().any(|r| r.name == n.name) {
            out.push(PlateRay::lens(n, "lens"));
        }
    }
    out
}

/// Names put on people: each ray's person (the depth under the plate; a
/// person-shaped thing there is that person), else a bearing. Overlay rays
/// first (seen with this very frame), then the lens's, newest first.
pub fn name_people(cloud: &Cloud, bodies: &[Body], rays: &[PlateRay], p: &PeopleParams) -> (Vec<Person>, Vec<(String, f32)>) {
    let mut people: Vec<Person> = bodies.iter().map(|b| Person { body: *b, name: None, how: None, ray: None }).collect();
    let mut bearings: Vec<(String, f32)> = Vec::new();
    let mut order: Vec<&PlateRay> = rays.iter().collect();
    order.sort_by_key(|r| (r.source != "overlay", std::cmp::Reverse(r.at)));
    for r in order {
        if people.iter().any(|q| q.name.as_deref() == Some(&r.name)) || bearings.iter().any(|b| b.0 == r.name) {
            continue; // named already, by a fresher ray
        }
        match cloud.person_along(r.from, r.dir, p) {
            Some(b) => {
                let named = Person { body: b, name: Some(r.name.clone()), how: Some(r.source), ray: Some(r.clone()) };
                // A person-shaped thing there is them (keep the depth's own
                // place for it, as the follower tracks that).
                match people.iter_mut().filter(|q| q.name.is_none()).find(|q| (q.body.feet[0] - b.feet[0]).hypot(q.body.feet[2] - b.feet[2]) < SAME_BODY_M) {
                    Some(q) => {
                        q.name = named.name;
                        q.how = named.how;
                        q.ray = named.ray;
                    }
                    None => people.push(named),
                }
            }
            None => bearings.push((r.name.clone(), r.yaw())),
        }
    }
    (people, bearings)
}

/// One look: the latest pano frame, its people, their names (`o`).
pub fn look(b: &Bridge, room: &[String], ocr: Option<&OcrClient>, o: &LookOptions) -> Result<Look> {
    let taken = Instant::now();
    let (frame, eyes) = b.pano.frame_and_eyes()?;
    let at = Instant::now();
    let cloud = Cloud::new(&frame, CLOUD_STEP);
    let tracking = Tracking::new(&frame, &eyes, cloud.floor, FLOOR_Y);
    let p = PeopleParams::default();
    let bodies = cloud.bodies(&p);
    let mut rays = Vec::new();
    if o.names && !room.is_empty() {
        rays.extend(lens_rays(&b.orbit.names_since(at.checked_sub(o.lens_within).unwrap_or(at))));
        if let (true, Some(ocr)) = (o.overlay, ocr) {
            match eyes.eye_rgb8(0).and_then(|rgb| ocr.lines_rgb(&rgb, eyes.width as u16, eyes.height as u16)) {
                Ok(lines) => rays.extend(overlay_rays(&frame, &eyes, &tracking, &lines, room, at)),
                Err(e) => tracing::debug!("pano look: no OCR of the eyes: {e:#}"),
            }
        }
    }
    let (mut people, bearings) = name_people(&cloud, &bodies, &rays, &p);
    for q in people.iter_mut().filter(|q| q.name.is_none()) {
        q.how = None;
    }
    let look = Look { taken, frame, eyes, cloud, tracking, people, bearings };
    look.to_people(b, o.source, at);
    Ok(look)
}

impl Look {
    /// The named people as the follower and the surveys place players
    /// (tracking space); overlay plates with their text box.
    pub fn sightings(&self, whitelist: &[String]) -> Vec<Sighting> {
        self.people
            .iter()
            .filter_map(|q| {
                let name = q.name.clone()?;
                Some(Sighting {
                    whitelist_rank: whitelist.iter().position(|n| *n == name).map(|i| i + 1),
                    text: name.clone(),
                    name,
                    score: 1.0,
                    tag: self.tracking.point(q.body.plate()),
                    feet: self.tracking.point(q.body.feet),
                    bbox: q.ray.as_ref().and_then(|r| r.bbox).unwrap_or([0.0; 4]),
                    seen_ns: self.frame.capture_ns,
                })
            })
            .collect()
    }

    /// The players to the speaker tracker: the plates over the eyes with
    /// the frame (their rings measured there), the rest placed alone.
    pub fn to_speakers(&self, b: &Bridge, whitelist: &[String]) {
        let (over, rest): (Vec<Sighting>, Vec<Sighting>) = self.sightings(whitelist).into_iter().partition(|s| s.bbox[2] > 0.0);
        b.speaker.saw(&over, Some(&self.eyes));
        b.speaker.saw(&rest, None);
    }

    /// The people near the bot (`people`) updated from this look; those
    /// followed by continuity (no name read this look) go to the speaker
    /// tracker placed (the named ones go with `to_speakers`).
    fn to_people(&self, b: &Bridge, source: Option<&'static str>, at: Instant) {
        let tr = &self.tracking;
        let to_tracking = |p: [f32; 3]| tr.point(p);
        let head = crate::people::HeadThen { eye: tr.head_world, yaw: tr.world_yaw(self.eyes.views[0].pose.yaw_pitch().0), to_tracking: &to_tracking };
        let seen: Vec<crate::people::Seen> = self.people.iter().map(|q| crate::people::Seen { name: q.name.as_deref(), body: q.body, how: q.how }).collect();
        let kept = b.people.update(&seen, &head, source, at);
        let placed: Vec<Sighting> = kept
            .iter()
            .map(|n| Sighting {
                whitelist_rank: None,
                text: n.name.clone(),
                name: n.name.clone(),
                score: 1.0,
                tag: tr.point(n.body.plate()),
                feet: tr.point(n.feet),
                bbox: [0.0; 4],
                seen_ns: self.frame.capture_ns,
            })
            .collect();
        b.speaker.saw(&placed, None);
    }

    /// What a survey is made from.
    pub fn input(&self, room: Vec<String>) -> PanoInput {
        PanoInput {
            frame: self.frame.clone(),
            eyes: self.eyes.clone(),
            cloud: self.cloud.clone(),
            named: self.people.iter().filter_map(|q| Some((q.name.clone()?, q.body))).collect(),
            bearings: self.bearings.clone(),
            room,
            taken: self.taken,
        }
    }
}

/// The latest pano frame's depth as the follower sees it: the points
/// (tracking space), the head, world metres per unit, the floor's height
/// and the head's tracking yaw (the corridor's tuning route).
pub fn depth_now(b: &Bridge) -> Result<(Vec<[f32; 3]>, [f32; 3], f32, f32, f32)> {
    let (frame, eyes) = b.pano.frame_and_eyes()?;
    let cloud = Cloud::new(&frame, CLOUD_STEP);
    let tr = Tracking::new(&frame, &eyes, cloud.floor, FLOOR_Y);
    let floor = cloud.floor.map_or(FLOOR_Y, |f| tr.point([tr.head_world[0], f, tr.head_world[2]])[1]);
    let points = cloud.points.iter().map(|&p| tr.point(p)).collect();
    Ok((points, tr.head_track, tr.metres, floor, eyes.views[0].pose.yaw_pitch().0))
}

/// The room's other players' names.
pub fn room(b: &Bridge) -> Vec<String> {
    b.game.lk().others().into_iter().map(|(_, n)| n).collect()
}

/// How surveys look: a pano frame (names read when `players`). The
/// panorama is the only way the bot sees depth (decision D42): without it
/// (the avatar has none, or the usual view is leased) a survey fails.
pub fn surveyor(b: &Arc<Bridge>) -> impl FnMut(&mut Rig, &SurveyOptions, &[[f32; 2]]) -> Result<Survey> + '_ {
    move |rig: &mut Rig, opts: &SurveyOptions, blocked: &[[f32; 2]]| {
        anyhow::ensure!(b.pano.usable(), "no panorama: the avatar's pano cameras are needed to look around (`GET /v1/vr/pano`)");
        let room = room(b);
        let o = LookOptions { names: opts.players, overlay: opts.players, ..Default::default() };
        let l = look(b, &room, rig.ocr.as_ref(), &o)?;
        if opts.players {
            l.to_speakers(b, &rig.whitelist);
        }
        vrc_nav::survey_pano(l.input(room), opts, blocked, &rig.whitelist, rig.detect.as_ref())
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use vrc_pano::synth::{person, Scene};
    use vrc_pano::{decode, PanoParams};
    use vrc_vr::{Fov, Pose};

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    /// A rendered scene with the eyes' tracking poses: 1.6 m up, looking
    /// along the tracking space's -z, 100 degrees square.
    pub(crate) fn eyes_of(s: &Scene) -> EyeFrame {
        let mut eyes = s.render();
        let half = 50f32.to_radians();
        for (k, v) in eyes.views.iter_mut().enumerate() {
            v.pose = Pose::looking(0.0, 0.0, [if k == 0 { -0.0315 } else { 0.0315 }, 1.6, 0.0]);
            v.fov = Fov { left: -half, right: half, up: half, down: -half };
        }
        eyes
    }

    /// `d` metres from the rig along world yaw `yaw` (x, z).
    pub(crate) fn out(s: &Scene, yaw: f32, d: f32) -> (f32, f32) {
        let (sn, cs) = yaw.to_radians().sin_cos();
        (s.position[0] + d * sn, s.position[2] + d * cs)
    }

    fn ray(name: &str, from: [f32; 3], to: [f32; 3], source: &'static str) -> PlateRay {
        let d = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        PlateRay { name: name.into(), from, dir: d.map(|c| c / n), at: Instant::now(), source, bbox: None, head_yaw: d[0].atan2(d[2]).to_degrees().rem_euclid(360.0) }
    }

    #[test]
    fn names_go_on_the_people_under_their_plates() {
        let mut s = Scene::room();
        let y = s.room_min[1];
        // Ann 2 m ahead of the head, someone else 3 m to its right.
        let (ax, az) = out(&s, s.head_yaw, 2.0);
        let (bx, bz) = out(&s, s.head_yaw + 90.0, 3.0);
        s.boxes.push(person(&s, ax, az, 1.6));
        s.boxes.push(person(&s, bx, bz, 1.7));
        let frame = decode(&eyes_of(&s), &PanoParams::default()).unwrap();
        let cloud = Cloud::new(&frame, 4);
        let p = PeopleParams::default();
        let bodies = cloud.bodies(&p);
        assert_eq!(bodies.len(), 2, "{bodies:?}");
        let head = frame.head.position;
        // Ann read by the lens (from behind the head); Bob read toward a
        // wall where nobody stands; an older ray of Ann's loses.
        let rays = vec![
            ray("Ann", [head[0], head[1] + 0.35, head[2] + 0.2], [ax, y + 2.0, az], "lens"),
            ray("Bob", head, { let (x, z) = out(&s, s.head_yaw - 90.0, 3.0); [x, y + 1.9, z] }, "lens"),
            PlateRay { at: Instant::now() - Duration::from_secs(5), ..ray("Ann", head, [bx, y + 2.1, bz], "lens") },
        ];
        let (people, bearings) = name_people(&cloud, &bodies, &rays, &p);
        let ann: Vec<&Person> = people.iter().filter(|q| q.name.as_deref() == Some("Ann")).collect();
        assert_eq!(ann.len(), 1);
        assert!((ann[0].body.feet[0] - ax).hypot(ann[0].body.feet[2] - az) < 0.25, "{:?}", ann[0]);
        // The other person-shaped one is nobody's yet.
        assert_eq!(people.iter().filter(|q| q.name.is_none()).count(), 1);
        assert_eq!(bearings.len(), 1);
        assert!(bearings[0].0 == "Bob" && close(vrc_pano::frame::wrap(bearings[0].1 - (s.head_yaw - 90.0)), 0.0, 3.0), "{bearings:?}");
    }

    #[test]
    fn a_plate_over_the_eyes_is_a_ray_through_the_eye() {
        let mut s = Scene::room();
        let (ax, az) = out(&s, s.head_yaw, 2.0);
        s.boxes.push(person(&s, ax, az, 1.6));
        let mut eyes = eyes_of(&s);
        // VRChat's plate over the middle of the eyes (the right eye's copy
        // 40 px left of the left eye's): a dark pill with white text.
        let n = eyes.eye_bytes();
        let (w, cx, cy) = (1920usize, 960usize, 900usize);
        for (eye, shift) in [(0usize, 0usize), (1, 40)] {
            for y in cy - 20..cy + 20 {
                for x in cx - 100 - shift..cx + 100 - shift {
                    let k = eye * n + (y * w + x) * 4;
                    let white = (y as i64 - cy as i64).abs() < 6 && (x / 7) % 2 == 0;
                    eyes.pixels[k..k + 3].copy_from_slice(if white { &[255, 255, 255] } else { &[30, 30, 35] });
                }
            }
        }
        let frame = decode(&eyes, &PanoParams::default()).unwrap();
        let tracking = Tracking::new(&frame, &eyes, Cloud::new(&frame, 4).floor, FLOOR_Y);
        let line = |text: &str, x: f32, y: f32| vrc_players::OcrLine { text: text.into(), bbox: [x - 40.0, y - 10.0, 80.0, 20.0], confidence: 0.9 };
        let names = vec!["Ann".to_string(), "Bob".to_string()];
        // The plate's text (on the UI), and Bob's name on a poster in a
        // tile (not on the UI).
        let lines = vec![line("Ann", cx as f32, cy as f32), line("Bob", 400.0, 300.0)];
        let rays = overlay_rays(&frame, &eyes, &tracking, &lines, &names, Instant::now());
        assert_eq!(rays.len(), 1, "{rays:?}");
        let r = &rays[0];
        assert_eq!((r.name.as_str(), r.source, r.bbox.is_some()), ("Ann", "overlay", true));
        // Through the middle of the left eye: where the head looks (a hair
        // below: the pixel's middle), from the left eye.
        assert!(close(vrc_pano::frame::wrap(r.yaw() - s.head_yaw), 0.0, 1.0), "{} vs {}", r.yaw(), s.head_yaw);
        assert!((r.from[0] - frame.head.position[0]).hypot(r.from[2] - frame.head.position[2]) < 0.1, "{:?}", r.from);
    }

    /// Frame `b` of 2026-10-09 (E1C, VRChat Home) as `vrc-pano`'s test cuts
    /// have it, with the eyes' poses then (tracking space).
    fn real_b() -> EyeFrame {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../vrc-pano/tests/data");
        let png = |name: &str| {
            let dec = png::Decoder::new(std::fs::File::open(dir.join(name)).unwrap());
            let mut r = dec.read_info().unwrap();
            let mut buf = vec![0; r.output_buffer_size()];
            let info = r.next_frame(&mut buf).unwrap();
            (info.width as usize, info.height as usize, buf)
        };
        let n = 1920 * 1920 * 4;
        let mut pixels = vec![255u8; 2 * n];
        for (k, px) in pixels.chunks_exact_mut(4).enumerate() {
            px[..3].copy_from_slice(if k < 1920 * 1920 { &[90, 90, 90] } else { &[255, 0, 255] });
        }
        let mut paste = |eye: usize, x: usize, y: usize, (w, h, rgb): (usize, usize, Vec<u8>), rows: std::ops::Range<usize>| {
            for (j, src) in rows.enumerate() {
                for i in 0..w {
                    let d = eye * n + ((y + j) * 1920 + x + i) * 4;
                    pixels[d..d + 3].copy_from_slice(&rgb[(src * w + i) * 3..(src * w + i) * 3 + 3]);
                }
            }
            let _ = h;
        };
        paste(0, 0, 1440, png("e1c_reserved.png"), 0..480);
        paste(1, 0, 1440, png("e1c_reserved.png"), 480..960);
        paste(1, 1300, 200, png("e1c_front.png"), 0..420);
        paste(1, 1440, 1440, png("e1c_down.png"), 0..480);
        paste(1, 940, 720, png("e1c_plate.png"), 0..220);
        let half = 0.8726646f32;
        let ori = [0.009096330031752586f32, 0.3675105571746826, -0.013836679980158806, 0.9298719763755798];
        let pose = |p: [f32; 3]| vrc_vr::tap::EyeView { fov: Fov { left: -half, right: half, up: half, down: -half }, pose: Pose { orientation: ori, position: p } };
        EyeFrame {
            seq: 2,
            frame_id: 1,
            display_time_ns: 0,
            capture_ns: 0,
            width: 1920,
            height: 1920,
            format: vrc_vr::tap::format::R8G8B8A8_SRGB,
            bytes_per_pixel: 4,
            views: [pose([0.43833449, 1.56121182, -1.81110727]), pose([0.48429229, 1.56001186, -1.85418212])],
            pixels,
        }
    }

    #[test]
    fn a_real_plate_over_the_eyes_names_the_person_under_it() {
        let eyes = real_b();
        let frame = decode(&eyes, &PanoParams::default()).unwrap();
        let cloud = Cloud::new(&frame, 4);
        let tracking = Tracking::new(&frame, &eyes, cloud.floor, FLOOR_Y);
        // "xkeyC" in the plate's pill as the left eye shows it (the OCR's
        // box; the service is not here), and "dokidoki" over it, not a
        // player's name.
        let lines = vec![
            vrc_players::OcrLine { text: "xkeyC".into(), bbox: [1085.0, 833.0, 86.0, 36.0], confidence: 0.95 },
            vrc_players::OcrLine { text: "dokidoki".into(), bbox: [1050.0, 775.0, 100.0, 24.0], confidence: 0.9 },
        ];
        let rays = overlay_rays(&frame, &eyes, &tracking, &lines, &["xkeyC".to_string()], Instant::now());
        assert_eq!(rays.len(), 1, "{rays:?}");
        let p = PeopleParams::default();
        let (people, bearings) = name_people(&cloud, &cloud.bodies(&p), &rays, &p);
        assert!(bearings.is_empty(), "{bearings:?}");
        let user = people.iter().find(|q| q.name.as_deref() == Some("xkeyC")).expect("named");
        let b = user.body;
        assert_eq!(user.how, Some("overlay"));
        assert!((b.distance - 2.92).abs() < 0.2 && (b.yaw_from(frame.head.position) - 147.3).abs() < 3.0, "{b:?}");
        // In the tracking space: ahead-ish of the head, about 2.9 m (scale 1:
        // the avatar's eyes 1.18 m, the headset's 1.88 m over its floor).
        let f = tracking.point(b.feet);
        let d = (f[0] - tracking.head_track[0]).hypot(f[2] - tracking.head_track[2]) * tracking.metres;
        assert!((d - 2.92).abs() < 0.2, "{d}");
    }
}
