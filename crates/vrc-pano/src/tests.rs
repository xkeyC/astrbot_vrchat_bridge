//! End to end on synthetic frames (`synth`): drawn as the avatar draws
//! them, decoded, and checked against the room they show.

use crate::code::{classify, Route, Seen, E1C};
use crate::frame::{decode, wrap, PanoParams, MASK_BODY, MASK_COLOUR, MASK_OVERLAY};
use crate::layout::{self, Face};
use crate::synth::{self, Scene};

fn close(a: f32, b: f32, tol: f32) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn the_code_reads_and_every_check_holds() {
    let s = Scene::room();
    let frame = s.render();
    let Seen::Pano { code, head } = classify(&frame) else { panic!("{:?}", classify(&frame)) };
    assert_eq!(code.position, s.position);
    assert!(close(code.rig_yaw, 140.26, 0.01), "{}", code.rig_yaw);
    assert_eq!((code.seq, code.age, code.layout, code.route, code.depth_code), (40, 15, 1, Route::D1, E1C));
    assert_eq!((code.zmin, code.zmax), (0.25, 64.0));
    assert_eq!((head.seq, head.eyes), (40, 2));
    assert!(close(head.yaw, 231.78, 0.01) && close(head.pitch, 0.5, 0.01), "{head:?}");
    assert!(close(head.position[0], s.position[0], 1e-4) && close(head.position[1], s.position[1] - 0.01, 1e-4));

    // Another depth encoding (an older rig): turned down, saying so.
    for depth_code in [0u8, 1, 3] {
        let old = Scene { depth_code, ..Scene::room() }.render();
        let Seen::Unusable(why) = classify(&old) else { panic!("{depth_code}: {:?}", classify(&old)) };
        assert!(why.contains("E1C"), "{why}");
    }
    // Just turned on: the faces may be the last time's.
    let young = Scene { age: 1, ..Scene::room() }.render();
    assert!(matches!(classify(&young), Seen::Unusable(_)));
    // The beacon's clock elsewhere: another frame's code.
    let mut other = Scene { seq: 41, ..Scene::room() }.render();
    let code = Scene::room().code_bits();
    for eye in 0..2 {
        let n = other.eye_bytes();
        synth::draw_grid(&mut other.pixels[eye * n..(eye + 1) * n], 1920, crate::code::ORIGIN, &code);
    }
    assert_eq!(classify(&other), Seen::Unusable("the beacon's seq differs"));
    // One eye's code gone: no guessing.
    let mut one = s.render();
    let n = one.eye_bytes();
    synth::fill_ndc(&mut one.pixels[n..], 1920, 1920, [-0.97, -0.85, -0.73, -0.73], [0, 0, 0]);
    assert!(matches!(classify(&one), Seen::Unusable(_)));
    // Neither: the usual view.
    synth::fill_ndc(&mut one.pixels[..n], 1920, 1920, [-0.97, -0.85, -0.73, -0.73], [0, 0, 0]);
    assert_eq!(classify(&one), Seen::Normal);
    assert!(decode(&one, &PanoParams::default()).is_err());
}

/// How far a world point is off the room's nearest wall.
fn off_the_walls(s: &Scene, p: [f32; 3]) -> f32 {
    (0..3).map(|a| (p[a] - s.room_min[a]).abs().min((p[a] - s.room_max[a]).abs())).fold(f32::INFINITY, f32::min)
}

#[test]
fn depth_lands_on_the_walls_and_the_floor() {
    let s = Scene::room();
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    assert!(!p.depth_ordinal && p.calibration.residual < 0.5, "{:?}", p.calibration);
    let points = p.points(4);
    assert!(points.len() > 150_000, "{}", points.len());
    let mut worst = 0.0f32;
    for q in &points {
        let d = ((q.world[0] - s.position[0]).powi(2) + (q.world[1] - s.position[1]).powi(2) + (q.world[2] - s.position[2]).powi(2)).sqrt();
        // E1: half a level is 1.09 % of the distance along the axis.
        worst = worst.max(off_the_walls(&s, q.world) / d);
    }
    assert!(worst < 0.013, "{worst}");
    // The floor below: the down face's middle.
    let down = Face::ALL.iter().position(|&f| f == Face::Down).unwrap();
    let floor = p.point(down, 240, 240).unwrap();
    assert!(close(floor[1], s.room_min[1], 0.02) && close(floor[0], s.position[0], 0.01), "{floor:?}");
    assert_eq!(p.map_point(down, 240, 240).unwrap()[2], -floor[2]);
    // The corners of the room are no depth edges: next to nothing dropped.
    let dropped: usize = p.views.iter().map(|v| v.depth.iter().filter(|z| z.is_nan()).count()).sum();
    let all: usize = p.views.iter().map(|v| v.depth.len()).sum();
    assert!((dropped as f32) < 0.002 * all as f32, "{dropped} of {all}");
}

#[test]
fn bearings_are_from_the_head() {
    let s = Scene::room();
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    for (k, v) in p.views.iter().enumerate() {
        let ray = p.ray(k, v.width as f32 / 2.0, v.height as f32 / 2.0);
        match v.face.yaw() {
            Some(yaw) => {
                assert!(close(wrap(ray.bearing - wrap(s.rig_yaw + yaw - s.head_yaw)), 0.0, 0.01), "{:?}: {ray:?}", v.face);
                assert!(close(ray.elevation, 0.0, 0.01));
            }
            None => assert!(close(ray.elevation.abs(), 90.0, 0.01), "{:?}: {ray:?}", v.face),
        }
    }
    // A pixel right of the 90 face's middle is further right.
    let k = Face::ALL.iter().position(|&f| f == Face::F90).unwrap();
    let (a, b) = (p.ray(k, 480.0, 360.0), p.ray(k, 600.0, 300.0));
    assert!(wrap(b.bearing - a.bearing) > 5.0 && b.elevation > 5.0);
    // The world ray points where the room's colour says.
    let (u, v) = (700.0, 200.0);
    let ray = p.ray(k, u, v);
    let px = p.views[k].rgb_at(u as u32, v as u32);
    let want = s.hit(ray.world).1;
    assert!(px.iter().zip(want).all(|(a, b)| (*a as i32 - b as i32).abs() <= 4), "{px:?} {want:?}");
}

/// The middle of the red pixels of an equirect, as (bearing, elevation).
fn red_centre(e: &crate::render::Equirect, near: (f32, f32)) -> Option<(f32, f32)> {
    let (w, h) = (e.pano.width, e.pano.height);
    let (nc, nr) = e.pixel_of(near.0, near.1);
    let (mut sx, mut sy, mut sz, mut n) = (0.0f32, 0.0f32, 0.0f32, 0);
    for row in 0..h {
        for col in 0..w {
            let k = (row * w + col) * 3;
            let dc = ((col as f32 - nc).abs()).min(w as f32 - (col as f32 - nc).abs());
            if e.pano.rgb[k..k + 3] == [255, 0, 0] && dc < w as f32 / 8.0 && (row as f32 - nr).abs() < h as f32 / 6.0 {
                // Averaged as directions (the poles have no one column).
                let b = ((col as f32 + 0.5) / w as f32 * 360.0 - 180.0).to_radians();
                let el = (90.0 - (row as f32 + 0.5) / h as f32 * 180.0).to_radians();
                sx += el.cos() * b.sin();
                sy += el.sin();
                sz += el.cos() * b.cos();
                n += 1;
            }
        }
    }
    (n > 0).then(|| (sx.atan2(sz).to_degrees(), sy.atan2(sx.hypot(sz)).to_degrees()))
}

#[test]
fn a_marker_in_each_face_lands_at_its_bearing() {
    let mut s = Scene::room();
    // (rig azimuth, elevation): inside each face, off its middle.
    let marks = [(20.0f32, 10.0f32), (110.0, -15.0), (200.0, 20.0), (290.0, -5.0), (60.0, 70.0), (-150.0, -65.0)];
    for &(az, el) in &marks {
        let (az, el) = ((az + s.rig_yaw).to_radians(), el.to_radians());
        s.markers.push(([az.sin() * el.cos(), el.sin(), az.cos() * el.cos()], 2.0));
    }
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    // Each mark is where the most central view sees it.
    for (k, &(az, el)) in marks.iter().enumerate() {
        let face = p.views[p.project(s.markers[k].0).unwrap().0].face;
        let want = [Face::F0, Face::F90, Face::F180, Face::F270, Face::Up, Face::Down][k];
        assert_eq!(face, want, "mark {k} at {az}, {el}");
    }
    let e = p.heading_equirect(1440);
    assert!(e.pano.coverage > 0.999, "{}", e.pano.coverage);
    for &(az, el) in &marks {
        let bearing = wrap(az + s.rig_yaw - s.head_yaw);
        let (b, e2) = red_centre(&e, (bearing, el)).unwrap_or_else(|| panic!("no mark near {bearing}, {el}"));
        assert!(close(wrap(b - bearing), 0.0, 0.6) && close(e2, el, 0.6), "mark at {bearing}, {el}: found {b}, {e2}");
    }
}

#[test]
fn the_equirect_is_the_room_seams_and_all() {
    let s = Scene::room();
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    let e = p.equirect(1024, 0.0);
    let (w, h) = (e.pano.width, e.pano.height);
    let (mut bad, mut range_bad) = (0, 0);
    for row in 0..h {
        let lat = std::f32::consts::FRAC_PI_2 - (row as f32 + 0.5) / h as f32 * std::f32::consts::PI;
        for col in 0..w {
            let lon = (col as f32 + 0.5) / w as f32 * std::f32::consts::TAU - std::f32::consts::PI;
            let d = [lon.sin() * lat.cos(), lat.sin(), lon.cos() * lat.cos()];
            let (t, want) = s.hit(d);
            let k = (row * w + col) * 3;
            if e.pano.rgb[k..k + 3].iter().zip(want).any(|(a, b)| (*a as i32 - b as i32).abs() > 6) {
                bad += 1;
            }
            let r = e.range[row * w + col];
            if !(r.is_finite() && (r - t).abs() < 0.015 * t) {
                range_bad += 1;
            }
            // Next to its neighbour (and across the wrap): no seam.
            let n = (row * w + (col + 1) % w) * 3;
            assert!(
                e.pano.rgb[k..k + 3].iter().zip(&e.pano.rgb[n..n + 3]).all(|(a, b)| (*a as i32 - *b as i32).abs() <= 24),
                "a jump at row {row}, col {col}"
            );
        }
    }
    assert!(bad < w * h / 2000, "{bad} pixels off the room's colour");
    assert!(range_bad < w * h / 500, "{range_bad} pixels off its distance");
    // Centred on 0, vrc-scene's pixel_of places map points on it.
    let ahead = [s.position[0], s.position[1], -(s.position[2] + 2.0)];
    let (col, row) = e.pano.pixel_of(crate::frame::to_map(s.position), ahead);
    assert!(close(col, w as f32 / 2.0, 0.5) && close(row, h as f32 / 2.0, 0.5), "{col}, {row}");
}

/// How far the points of every 4th pixel are off the walls, over their
/// distance, sorted.
fn errors(s: &Scene, p: &crate::frame::PanoFrame) -> Vec<f32> {
    let mut e: Vec<f32> = p
        .points(4)
        .iter()
        .map(|q| {
            let d = ((q.world[0] - s.position[0]).powi(2) + (q.world[1] - s.position[1]).powi(2) + (q.world[2] - s.position[2]).powi(2)).sqrt();
            off_the_walls(s, q.world) / d
        })
        .collect();
    e.sort_by(f32::total_cmp);
    e
}

#[test]
fn post_processing_is_undone() {
    let s = Scene { post: Some(synth::graded), ..Scene::room() };
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    assert!(p.calibration.fitted && p.calibration.residual < 2.0 && !p.depth_ordinal, "{:?}", p.calibration);
    // The vignette's shape over the faces: 1 - 0.1 r², relative to r = 1,
    // in every channel.
    for c in &p.calibration.channels {
        for r2 in [0.2f32, 0.5, 1.0, 1.4, 1.8] {
            let g = c.gain(r2);
            assert!(close(g, (1.0 - 0.1 * r2) / 0.9, 0.03), "gain {g} at r² {r2}: a {} b {}", c.a, c.b);
        }
    }
    // Graded and dithered per channel, the depth still passes the check.
    // Measured: 99 % within 3 and 6.25 (bytes' worth); 0.06 % turned down.
    assert!(p.check.overlay < 0.002 && p.check.rg[1] <= 4.0 && p.check.b[1] <= 7.0, "{:?}", p.check);
    eprintln!("graded: the check's residuals {:?}", p.check);
    // No bias left; the dither is (a byte: about a level and a half here,
    // 3 %, on top of half a level of rounding).
    let e = errors(&s, &p);
    let n = e.len();
    assert!(e[n / 2] < 0.012 && e[n * 99 / 100] < 0.06, "median {}, 99 % {}", e[n / 2], e[n * 99 / 100]);
}

/// Paints `rgb` into eye `eye` over the rectangle (x0, y0) .. (x1, y1).
fn paint(frame: &mut vrc_vr::tap::EyeFrame, eye: usize, [x0, y0, x1, y1]: [usize; 4], rgb: [u8; 3]) {
    let (w, n) = (frame.width as usize, frame.eye_bytes());
    for y in y0..y1 {
        for x in x0..x1 {
            let k = eye * n + (y * w + x) * 4;
            frame.pixels[k..k + 3].copy_from_slice(&rgb);
        }
    }
}

#[test]
fn the_ui_over_the_depth_fails_the_check() {
    // VRChat's UI over the front face (the right eye; its left-eye copy
    // 40 px to the right): a plate's dark pill and white text, the
    // viewfinder's black bar, mid grey, yellow and green text, rings in
    // cyan, and pure cyan (exactly q 0's code).
    let s = Scene { post: Some(synth::graded), ..Scene::room() };
    let clean = decode(&s.render(), &PanoParams::default()).unwrap();
    let ui: [[u8; 3]; 12] = [
        [30, 30, 35],
        [60, 60, 66],
        [255, 255, 255],
        [0, 0, 0],
        [128, 128, 128],
        [255, 220, 0],
        [255, 255, 0],
        [0, 255, 0],
        [60, 200, 80],
        [0, 200, 255],
        [64, 224, 255],
        [0, 255, 255],
    ];
    let mut frame = s.render();
    // Each colour a 30 x 30 square along a row of the front face (eye x
    // 1000.., y 300..330).
    for (k, &rgb) in ui.iter().enumerate() {
        let x = 1000 + 60 * k;
        paint(&mut frame, 1, [x, 300, x + 30, 330], rgb);
        paint(&mut frame, 0, [x + 40, 300, x + 70, 330], rgb);
    }
    let p = decode(&frame, &PanoParams::default()).unwrap();
    let f0 = p.view(Face::F0);
    let at = |x: usize, y: usize| (y * f0.width as usize + x - 960) as usize;
    for (k, rgb) in ui.iter().enumerate() {
        let x = 1000 + 60 * k;
        // Every pixel of the square and 2 px round it: no depth.
        for y in 298..332 {
            for xx in x - 2..x + 32 {
                let i = at(xx, y);
                assert!(f0.mask[i] & MASK_OVERLAY != 0 && f0.depth[i].is_nan(), "{rgb:?} at ({xx}, {y})");
            }
        }
        // The left eye's copy, up to the disparity to the right: no colour.
        for xx in x..x + 70 {
            assert!(f0.mask[at(xx, 315)] & MASK_COLOUR != 0, "{rgb:?}: colour at {xx}");
            assert!(f0.colour_at((xx - 960) as u32, 315).is_none());
        }
    }
    // Away from it the depth is as before.
    let i = at(1000, 400);
    assert!(f0.mask[i] == 0 && close(f0.depth[i], clean.view(Face::F0).depth[i], 1e-6));
    assert!(p.check.overlay > 0.0 && p.check.overlay < 0.01, "{:?}", p.check);
    // The equirect: no colour there (none black), the room's elsewhere.
    let e = p.heading_equirect(1024);
    assert!(e.pano.coverage > 0.999);
    // A nameplate box found by OCR in the left eye: its colour masked.
    let mut p = p;
    p.mask_colour(1200, 500, 1300, 540);
    assert!(p.view(Face::F0).colour_at(1250 - 960, 520).is_none() && p.view(Face::F0).colour_at(1250 - 960, 560).is_some());
}

#[test]
fn clean_depth_passes_the_check() {
    let p = decode(&Scene::room().render(), &PanoParams::default()).unwrap();
    // No post-processing: the channels agree to the rounding.
    assert_eq!(p.check.overlay, 0.0);
    assert!(p.check.rg[1] <= 0.5 && p.check.b[1] <= 1.0, "{:?}", p.check);
}

#[test]
fn the_own_body_is_masked() {
    let s = Scene { body: Some(0.35), ..Scene::room() };
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    let v = p.view(Face::Down);
    assert!(v.depth_at(240, 240).is_none() && v.mask[240 * 480 + 240] & MASK_BODY != 0);
    // The floor round it stays.
    assert!(v.depth_at(20, 20).is_some_and(|z| close(z, s.position[1] - s.room_min[1], 0.03)));
    let p = decode(&s.render(), &PanoParams { body_near_m: 0.3, ..Default::default() }).unwrap();
    assert!(p.view(Face::Down).depth_at(240, 240).is_some());
}

#[test]
fn smaller_eyes_are_scaled() {
    let s = Scene { size: 1280, ..Scene::room() };
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    assert!(p.scaled);
    let f0 = p.view(Face::F0);
    assert_eq!((f0.width, f0.height, f0.block), (640, 480, [640, 0, 640, 480]));
    assert_eq!(f0.intrinsics, [320.0, 320.0, 320.0, 240.0]);
    let down = Face::ALL.iter().position(|&f| f == Face::Down).unwrap();
    let floor = p.point(down, 160, 160).unwrap();
    assert!(close(floor[1], s.room_min[1], 0.03), "{floor:?}");
    let mut worst = 0.0f32;
    for q in p.points(6) {
        let d = ((q.world[0] - s.position[0]).powi(2) + (q.world[1] - s.position[1]).powi(2) + (q.world[2] - s.position[2]).powi(2)).sqrt();
        worst = worst.max(off_the_walls(&s, q.world) / d);
    }
    // Resampled: the pixel's middle is up to half a face texel off.
    assert!(worst < 0.03, "{worst}");
    assert_eq!(p.view_at(700, 10).map(|(k, i, j)| (p.views[k].face, i, j)), Some((Face::F0, 60, 10)));
    assert_eq!(layout::reserved_rect(1280, 1280), [0, 960, 640, 1280]);
}

#[test]
fn decoding_is_quick() {
    let frame = Scene::room().render();
    let params = PanoParams::default();
    decode(&frame, &params).unwrap();
    let t = std::time::Instant::now();
    let n = 5;
    for _ in 0..n {
        decode(&frame, &params).unwrap();
    }
    let ms = t.elapsed().as_secs_f32() * 1e3 / n as f32;
    let e = std::time::Instant::now();
    let p = decode(&frame, &params).unwrap();
    let _ = p.heading_equirect(2048);
    eprintln!("decode {ms:.1} ms a frame; equirect 2048: {:.1} ms", e.elapsed().as_secs_f32() * 1e3 - p.decode_ms);
}

#[test]
fn a_view_out_of_the_panorama_looks_where_asked() {
    // A red disc 30 degrees right of the rig's ahead, 10 degrees up.
    let (yaw, pitch) = (Scene::room().rig_yaw + 30.0, 10.0f32);
    let (sy, cy) = yaw.to_radians().sin_cos();
    let (sp, cp) = pitch.to_radians().sin_cos();
    let s = Scene { markers: vec![([sy * cp, sp, cy * cp], 2.0)], ..Scene::room() };
    let p = decode(&s.render(), &PanoParams::default()).unwrap();
    let w = 201;
    let at = |rgb: &[u8], i: usize, j: usize| [rgb[(j * w + i) * 3], rgb[(j * w + i) * 3 + 1], rgb[(j * w + i) * 3 + 2]];
    // Looking at it: in the middle.
    let v = p.perspective(yaw, pitch, 100.0, w, w);
    assert_eq!(at(&v, 100, 100), [255, 0, 0]);
    // 20 degrees to its left: right of the middle, by f tan 20.
    let v = p.perspective(yaw - 20.0, pitch, 100.0, w, w);
    let f = 100.5 / 50f32.to_radians().tan();
    let i = (100.5 + f * 20f32.to_radians().tan()) as usize;
    assert_eq!(at(&v, i, 100), [255, 0, 0]);
    assert_ne!(at(&v, 100, 100), [255, 0, 0]);
}
