//! Real frames, cut small into `tests/data/` and pasted into otherwise empty
//! eyes:
//!
//! - 2026-10-08, VRChat Home: the code region of a frame with the rig off.
//! - 2026-10-09, VRChat Home, E1C (frame `b`): the user stands 2.9 m ahead
//!   of the bot (about 6 degrees right of the front face's middle), 0.26 m
//!   before a panel; their nameplate, drawn by VRChat over the eyes, lies on
//!   the right eye's tiles. Cuts: the codes and the calibration cells, the
//!   right eye's front round the user, its down face (the floor), and its
//!   plate.

use std::path::Path;

use vrc_pano::{classify, decode, Cloud, Face, PanoParams, PeopleParams, Seen, E1C, MASK_OVERLAY};
use vrc_vr::tap::{format, EyeFrame};

const EYE: usize = 1920;

fn png(name: &str) -> (usize, usize, Vec<u8>) {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    let dec = png::Decoder::new(std::fs::File::open(&path).unwrap());
    let mut reader = dec.read_info().unwrap();
    let mut buf = vec![0; reader.output_buffer_size()];
    let info = reader.next_frame(&mut buf).unwrap();
    assert_eq!(info.color_type, png::ColorType::Rgb);
    buf.truncate(info.buffer_size());
    (info.width as usize, info.height as usize, buf)
}

/// Both eyes, 1920², the left grey and the right "nothing drawn" (E1C).
fn empty_frame() -> EyeFrame {
    let mut pixels = vec![255u8; 2 * EYE * EYE * 4];
    for px in pixels[..EYE * EYE * 4].chunks_exact_mut(4) {
        px[..3].copy_from_slice(&[90, 90, 90]);
    }
    for px in pixels[EYE * EYE * 4..].chunks_exact_mut(4) {
        px[..3].copy_from_slice(&[255, 0, 255]);
    }
    EyeFrame {
        seq: 2,
        frame_id: 1,
        display_time_ns: 0,
        capture_ns: 0,
        width: EYE as u32,
        height: EYE as u32,
        format: format::R8G8B8A8_SRGB,
        bytes_per_pixel: 4,
        views: Default::default(),
        pixels,
    }
}

/// Pastes `w` x `h` RGB from `src` (rows from `sy`) at (`x`, `y`) of eye `eye`.
#[allow(clippy::too_many_arguments)]
fn paste(frame: &mut EyeFrame, eye: usize, x: usize, y: usize, (w, h): (usize, usize), src: &[u8], src_w: usize, sx: usize, sy: usize) {
    let base = eye * EYE * EYE * 4;
    for j in 0..h {
        for i in 0..w {
            let s = ((sy + j) * src_w + sx + i) * 3;
            let d = base + ((y + j) * EYE + x + i) * 4;
            frame.pixels[d..d + 3].copy_from_slice(&src[s..s + 3]);
        }
    }
}

#[test]
fn a_real_frame_with_the_rig_off_is_the_usual_view() {
    let mut f = empty_frame();
    let (w, _, rgb) = png("t1_off_codes.png");
    for eye in 0..2 {
        paste(&mut f, eye, 0, 1648, (272, 256), &rgb, w, 0, eye * 256);
    }
    assert_eq!(classify(&f), Seen::Normal);
    // Its beacon still reads.
    assert_eq!(vrc_vr::beacon::read(&f, 0).map(|b| b.seq), Some(74));
}

/// Frame `b` of 2026-10-09 as far as the cuts have it.
fn e1c_b() -> EyeFrame {
    let mut f = empty_frame();
    let (w, _, rgb) = png("e1c_reserved.png");
    for eye in 0..2 {
        paste(&mut f, eye, 0, 1440, (960, 480), &rgb, w, 0, eye * 480);
    }
    for (name, x, y) in [("e1c_front.png", 1300, 200), ("e1c_down.png", 1440, 1440), ("e1c_plate.png", 940, 720)] {
        let (w, h, rgb) = png(name);
        paste(&mut f, 1, x, y, (w, h), &rgb, w, 0, 0);
    }
    f
}

#[test]
fn a_real_e1c_frame() {
    let f = e1c_b();
    let Seen::Pano { code, head } = classify(&f) else { panic!("{:?}", classify(&f)) };
    assert_eq!((code.depth_code, code.age, code.layout), (E1C, 15, 1));
    assert!((code.rig_yaw - 138.58).abs() < 0.05 && (head.yaw - 138.39).abs() < 0.05, "{code:?} {head:?}");
    let p = decode(&f, &PanoParams::default()).unwrap();
    // VRChat Home: no post-processing; the cells read as drawn.
    let c = &p.calibration;
    assert!(c.fitted && c.residual < 1.0 && c.covered == 0, "residual {} covered {}", c.residual, c.covered);
    assert!(!p.depth_ordinal);
    // The check on the depth (the coordinator measured p99 2 and 3 levels
    // on the whole frame).
    assert!(p.check.rg[1] <= 2.5 && p.check.b[1] <= 3.5, "{:?}", p.check);
    // The plate over the eyes (the right eye's, about x 975-1175, y
    // 790-890): the UI, all of it; the depth beside it passes.
    let back = p.view(Face::F180);
    let ui = |x: u32, y: u32| back.mask[((y - 720) * back.width + x - 960) as usize] & MASK_OVERLAY != 0;
    let (mut on, mut all) = (0, 0);
    for y in (800..880).step_by(4) {
        for x in (1000..1160).step_by(4) {
            on += ui(x, y) as usize;
            all += 1;
        }
    }
    assert!(on as f32 > 0.9 * all as f32, "{on} of {all}");
    assert!(!ui(1230, 930) && back.depth_at(1230 - 960, 930 - 720).is_some());
    // The floor 1.18 m under the eyes.
    let cloud = Cloud::new(&p, 4);
    let floor = cloud.floor.unwrap();
    assert!((head.position[1] - floor - 1.176).abs() < 0.02, "{floor}");
    // The user: one person-shaped thing, 2.9 m out, the front face's way.
    let bodies = cloud.bodies(&PeopleParams::default());
    assert_eq!(bodies.len(), 1, "{bodies:?}");
    let b = bodies[0];
    let yaw = b.yaw_from(head.position);
    assert!((b.distance - 2.92).abs() < 0.15 && (yaw - 147.3).abs() < 3.0, "{b:?} at {yaw}");
}
