//! Synthetic pano frames for the tests: a box room around the rig, drawn
//! the way the avatar's shader draws it (faces resampled into their
//! blocks, E1C depth, both codes, the calibration cells, an optional
//! post-processing of the right eye, per channel).

use vrc_vr::beacon::{self, crc16, COLS, ROWS};
use vrc_vr::tap::{format, EyeFrame};

use crate::calib::{self, e1c, srgb_to_linear};
use crate::code::{E1C, ORIGIN};
use crate::layout::{self, Face};

pub struct Scene {
    pub size: u32,
    /// The cameras' centre (Unity's world).
    pub position: [f32; 3],
    pub rig_yaw: f32,
    pub head_yaw: f32,
    pub head_pitch: f32,
    pub seq: u8,
    pub age: u8,
    pub room_min: [f32; 3],
    pub room_max: [f32; 3],
    /// Red discs: a world direction (any length), its radius (degrees).
    pub markers: Vec<([f32; 3], f32)>,
    /// Boxes in the room (Unity's world: min, max corners) and their
    /// colour: people, furniture.
    pub boxes: Vec<([f32; 3], [f32; 3], [u8; 3])>,
    /// The bot's body in the down face (|x|, |y| < 0.35 of its plane):
    /// this near (metres).
    pub body: Option<f32>,
    /// The right eye's post-processing: the byte of (channel, code, r²).
    pub post: Option<fn(usize, f32, f32) -> u8>,
    /// The code's depth field (E1C; anything else is turned down).
    pub depth_code: u8,
}

impl Scene {
    pub fn room() -> Scene {
        Scene {
            size: 1920,
            position: [2.0, 1.4, -3.0],
            rig_yaw: 140.26,
            head_yaw: 231.78,
            head_pitch: 0.5,
            seq: 40,
            age: 15,
            room_min: [-2.5, 0.2, -7.0],
            room_max: [7.0, 3.6, 1.5],
            markers: Vec::new(),
            boxes: Vec::new(),
            body: None,
            post: None,
            depth_code: E1C,
        }
    }

    /// What the room shows along world direction `d` from the centre: the
    /// distance in units of `d`, and the colour.
    pub fn hit(&self, d: [f32; 3]) -> (f32, [u8; 3]) {
        let p = self.position;
        let mut t = f32::INFINITY;
        for a in 0..3 {
            if d[a] > 1e-9 {
                t = t.min((self.room_max[a] - p[a]) / d[a]);
            } else if d[a] < -1e-9 {
                t = t.min((self.room_min[a] - p[a]) / d[a]);
            }
        }
        let mut colour = None;
        for &(lo, hi, c) in &self.boxes {
            // Slabs: the ray's entry into the box, if in front.
            let (mut t0, mut t1) = (0.0f32, f32::INFINITY);
            for a in 0..3 {
                if d[a].abs() < 1e-9 {
                    if p[a] < lo[a] || p[a] > hi[a] {
                        t0 = f32::INFINITY;
                    }
                    continue;
                }
                let (ta, tb) = ((lo[a] - p[a]) / d[a], (hi[a] - p[a]) / d[a]);
                t0 = t0.max(ta.min(tb));
                t1 = t1.min(ta.max(tb));
            }
            if t0 <= t1 && t0 > 0.0 && t0 < t {
                t = t0;
                colour = Some(c);
            }
        }
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if let Some(c) = colour {
            return (t, c);
        }
        for &(m, radius) in &self.markers {
            let mn = (m[0] * m[0] + m[1] * m[1] + m[2] * m[2]).sqrt();
            let cos = (d[0] * m[0] + d[1] * m[1] + d[2] * m[2]) / (n * mn);
            if cos > radius.to_radians().cos() {
                return (t, [255, 0, 0]);
            }
        }
        (t, colour_at([p[0] + t * d[0], p[1] + t * d[1], p[2] + t * d[2]]))
    }

    /// The right eye's byte of channel `ch` for a code at NDC radius² `r2`.
    fn post(&self, ch: usize, code: f32, r2: f32) -> u8 {
        match self.post {
            Some(f) => f(ch, code, r2),
            None => code.round() as u8,
        }
    }

    pub fn render(&self) -> EyeFrame {
        let (w, h) = (self.size as usize, self.size as usize);
        let mut left = vec![0u8; w * h * 4];
        let mut right = vec![0u8; w * h * 4];
        for px in left.chunks_exact_mut(4).chain(right.chunks_exact_mut(4)) {
            px[3] = 255;
        }
        let (zmin, zmax) = (0.25f32, 64.0f32);
        for face in Face::ALL {
            let [bx, by, bw, bh] = face.block(self.size, self.size).map(|v| v as usize);
            let ([cw, ch], f, [cx, cy]) = face.camera();
            let rot = face.rotation();
            for j in 0..bh {
                // The shader's nearest texel of the face.
                let tj = (((j as f32 + 0.5) / bh as f32 * ch as f32) as u32).min(ch - 1);
                for i in 0..bw {
                    let ti = (((i as f32 + 0.5) / bw as f32 * cw as f32) as u32).min(cw - 1);
                    let x = (ti as f32 + 0.5 - cx) / f;
                    let y = (cy - (tj as f32 + 0.5)) / f;
                    let world = layout::turn(layout::mul(&rot, [x, y, 1.0]), self.rig_yaw);
                    let (mut z, mut rgb) = self.hit(world);
                    if let (Face::Down, Some(near)) = (face, self.body) {
                        if x.abs() < 0.35 && y.abs() < 0.35 {
                            (z, rgb) = (near, [60, 60, 60]);
                        }
                    }
                    let (ex, ey) = (bx + i, by + j);
                    let k = (ey * w + ex) * 4;
                    left[k..k + 3].copy_from_slice(&rgb);
                    let nx = (ex as f32 + 0.5) / w as f32 * 2.0 - 1.0;
                    let ny = 1.0 - (ey as f32 + 0.5) / h as f32 * 2.0;
                    let r2 = nx * nx + ny * ny;
                    let t = ((z / zmin).ln() / (zmax / zmin).ln()).clamp(0.0, 1.0);
                    let q = if z.is_finite() { (254.0 * t).round() } else { 255.0 };
                    right[k..k + 3].copy_from_slice(&[0, 1, 2].map(|ch| self.post(ch, e1c(q, ch), r2)));
                }
            }
        }
        // The head's beacons: each eye 3.15 cm aside.
        let side = layout::turn([1.0, 0.0, 0.0], self.head_yaw);
        for (eye, px) in [(-1.0f32, &mut left), (1.0, &mut right)] {
            let at = [self.position[0] + eye * 0.0315 * side[0], self.position[1] - 0.01, self.position[2] + eye * 0.0315 * side[2]];
            let bits = beacon_bits(at, self.head_yaw, self.head_pitch, self.seq);
            draw_grid(px, self.size, [-1.0 + beacon::MARGIN_NDC; 2], &bits);
            draw_grid(px, self.size, ORIGIN, &self.code_bits());
        }
        calib::draw_cells(&mut right, self.size, self.size, &|ch, code, r2| self.post(ch, code, r2));
        let mut pixels = left;
        pixels.extend_from_slice(&right);
        EyeFrame {
            seq: 2,
            frame_id: 1,
            display_time_ns: 0,
            capture_ns: 0,
            width: self.size,
            height: self.size,
            format: format::R8G8B8A8_SRGB,
            bytes_per_pixel: 4,
            views: Default::default(),
            pixels,
        }
    }

    /// The rig's code: layout 1, D1, the scene's depth code, 0.25..64 m.
    pub fn code_bits(&self) -> Vec<u8> {
        let word = ((self.seq as u32) << 16) | ((self.age as u32) << 12) | (1 << 8) | (1 << 6) | ((self.depth_code as u32 & 3) << 4) | (2 << 2) | 1;
        let mut bits = Vec::new();
        put(&mut bits, 0x5B, 8);
        for v in self.position {
            put(&mut bits, v.to_bits(), 32);
        }
        put(&mut bits, ((self.rig_yaw / 360.0 * 65536.0).round() as u32) & 0xFFFF, 16);
        put(&mut bits, word, 24);
        let crc = crc16(&bits);
        put(&mut bits, crc, 16);
        bits
    }
}

/// A smooth colour over the room's walls (continuous across their edges).
pub fn colour_at(p: [f32; 3]) -> [u8; 3] {
    let c = |v: f32| (128.0 + 90.0 * v.sin()).round() as u8;
    [c(1.1 * p[0] + 0.3), c(0.9 * p[1] + 1.0), c(1.3 * p[2] + 2.0)]
}

fn put(bits: &mut Vec<u8>, v: u32, n: usize) {
    for i in (0..n).rev() {
        bits.push(((v >> i) & 1) as u8);
    }
}

fn beacon_bits(at: [f32; 3], yaw: f32, pitch: f32, seq: u8) -> Vec<u8> {
    let mut bits = Vec::new();
    put(&mut bits, 0x5A, 8);
    for v in at {
        put(&mut bits, v.to_bits(), 32);
    }
    put(&mut bits, ((yaw / 360.0 * 65536.0).round() as u32) & 0xFFFF, 16);
    put(&mut bits, ((pitch * 100.0).round() as i32 as u32) & 0xFFFF, 16);
    put(&mut bits, seq as u32, 8);
    let crc = crc16(&bits);
    put(&mut bits, crc, 16);
    bits
}

/// A 22 x 10 grid with its bottom-left at `origin` (NDC), in an eye
/// `size`², its 160 bits as given.
pub fn draw_grid(px: &mut [u8], size: u32, origin: [f32; 2], bits: &[u8]) {
    for row in 0..ROWS {
        for col in 0..COLS {
            let white = if row == 0 {
                col % 2 == 0
            } else if row == ROWS - 1 || col == 0 || col == COLS - 1 {
                true
            } else {
                bits[(row - 1) * (COLS - 2) + (col - 1)] == 1
            };
            let x = origin[0] + col as f32 * beacon::BLOCK_NDC;
            let y = origin[1] + (ROWS - 1 - row) as f32 * beacon::BLOCK_NDC;
            let v = if white { 255 } else { 0 };
            fill_ndc(px, size, size, [x, y, x + beacon::BLOCK_NDC, y + beacon::BLOCK_NDC], [v; 3]);
        }
    }
}

/// Fills the pixels whose middles are in an NDC rectangle (x0, y0, x1, y1).
pub fn fill_ndc(px: &mut [u8], w: u32, h: u32, r: [f32; 4], rgb: [u8; 3]) {
    fill_ndc_with(px, w, h, r, &|_| rgb);
}

/// The same, each pixel's colour from its NDC radius².
pub fn fill_ndc_with(px: &mut [u8], w: u32, h: u32, r: [f32; 4], rgb_at: &dyn Fn(f32) -> [u8; 3]) {
    let to_x = |x: f32| (x + 1.0) / 2.0 * w as f32;
    let to_y = |y: f32| (1.0 - y) / 2.0 * h as f32;
    let (xa, xb) = ((to_x(r[0]) - 0.5).ceil().max(0.0) as u32, ((to_x(r[2]) - 0.5).ceil().max(0.0) as u32).min(w));
    let (ya, yb) = ((to_y(r[3]) - 0.5).ceil().max(0.0) as u32, ((to_y(r[1]) - 0.5).ceil().max(0.0) as u32).min(h));
    for y in ya..yb {
        for x in xa..xb {
            let i = ((y * w + x) * 4) as usize;
            let (nx, ny) = ((x as f32 + 0.5) / w as f32 * 2.0 - 1.0, 1.0 - (y as f32 + 0.5) / h as f32 * 2.0);
            px[i..i + 3].copy_from_slice(&rgb_at(nx * nx + ny * ny));
        }
    }
}

/// A person-sized box standing on the room's floor at (`x`, `z`) (Unity's
/// world): 0.5 m wide, 0.3 m deep, `height` tall.
pub fn person(s: &Scene, x: f32, z: f32, height: f32) -> ([f32; 3], [f32; 3], [u8; 3]) {
    let y = s.room_min[1];
    ([x - 0.25, y, z - 0.15], [x + 0.25, y + height, z + 0.15], [200, 160, 140])
}

/// A post-processing for the tests: darker corners, a gamma-ish curve
/// with lifted blacks, a byte of dither; the gamma a little different per
/// channel (grading).
pub fn graded(ch: usize, code: f32, r2: f32) -> u8 {
    let l = (1.0 - 0.1 * r2) * srgb_to_linear(code);
    let dither = ((code * 7.3 + r2 * 131.0 + ch as f32 * 1.7).sin() * 0.6).round();
    let gamma = [0.9, 0.95, 0.85][ch];
    (calib::linear_to_srgb(0.004 + 0.96 * l.clamp(0.0, 1.0).powf(gamma)) + dither).round().clamp(0.0, 255.0) as u8
}
