//! Layout 1 (avatar-panorama.md 3.1, 3.2): where the six faces sit in an
//! eye, and how each face's camera looks from the rig.
//!
//! The rig's frame is Unity's, level: +x right, +y up, +z the rig's ahead.
//! A face's camera sees (x, y, 1) at its pixel (x right, y up, both over
//! the focal length); its rotation takes that into the rig's frame.

/// The layout this crate reads (the code's `layout`).
pub const LAYOUT: u8 = 1;
/// The eye size the layout is drawn for: its blocks are the faces' own
/// pixels 1:1 there, and scaled (nearest pixel) at other sizes.
pub const EYE: u32 = 1920;

/// The six cameras, in the code's order: four level ones (named by where
/// they face from the rig's ahead, clockwise seen from above), up, down.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Face {
    F0,
    F90,
    F180,
    F270,
    Up,
    Down,
}

impl Face {
    pub const ALL: [Face; 6] = [Face::F0, Face::F90, Face::F180, Face::F270, Face::Up, Face::Down];

    pub fn name(self) -> &'static str {
        match self {
            Face::F0 => "0",
            Face::F90 => "90",
            Face::F180 => "180",
            Face::F270 => "270",
            Face::Up => "up",
            Face::Down => "down",
        }
    }

    /// Where a level face looks, degrees clockwise from the rig's ahead.
    pub fn yaw(self) -> Option<f32> {
        match self {
            Face::F0 => Some(0.0),
            Face::F90 => Some(90.0),
            Face::F180 => Some(180.0),
            Face::F270 => Some(270.0),
            Face::Up | Face::Down => None,
        }
    }

    /// The camera's own image: width, height, focal length, centre
    /// (pixels; a pixel i covers [i, i + 1)).
    pub fn camera(self) -> ([u32; 2], f32, [f32; 2]) {
        match self {
            Face::Up | Face::Down => ([480, 480], 240.0 / 55f32.to_radians().tan(), [240.0, 240.0]),
            _ => ([960, 720], 480.0, [480.0, 360.0]),
        }
    }

    /// The face's block in a 1920² eye: x, y, width, height (top-left origin).
    pub fn block_1920(self) -> [u32; 4] {
        match self {
            Face::F270 => [0, 0, 960, 720],
            Face::F0 => [960, 0, 960, 720],
            Face::F90 => [0, 720, 960, 720],
            Face::F180 => [960, 720, 960, 720],
            Face::Up => [960, 1440, 480, 480],
            Face::Down => [1440, 1440, 480, 480],
        }
    }

    /// The face's block in an eye `w` x `h`: the 1920² one scaled, edges
    /// rounded (the shader cuts at fractions of the eye: x 1/2 and 3/4,
    /// y 3/8 and 3/4).
    pub fn block(self, w: u32, h: u32) -> [u32; 4] {
        let [x, y, bw, bh] = self.block_1920();
        let sx = |v: u32| (v as f64 * w as f64 / EYE as f64).round() as u32;
        let sy = |v: u32| (v as f64 * h as f64 / EYE as f64).round() as u32;
        [sx(x), sy(y), sx(x + bw) - sx(x), sy(y + bh) - sy(y)]
    }

    /// fx, fy, cx, cy of the face as it lands in a block `bw` x `bh`
    /// (`Fov::intrinsics`' order; y down): the camera's own, scaled.
    pub fn intrinsics(self, bw: u32, bh: u32) -> [f32; 4] {
        let ([w, h], f, [cx, cy]) = self.camera();
        let (sx, sy) = (bw as f32 / w as f32, bh as f32 / h as f32);
        [f * sx, f * sy, cx * sx, cy * sy]
    }

    /// Rig <- camera: rows x, y, z of the rig; columns the camera's right,
    /// up and ahead. `R · (x, y, 1)` is the formula of avatar-panorama.md
    /// 3.2 (the up and down faces both have the rig's ahead at their top).
    pub fn rotation(self) -> [[f32; 3]; 3] {
        match self {
            Face::Up => [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
            Face::Down => [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
            f => {
                let (s, c) = f.yaw().unwrap().to_radians().sin_cos();
                [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
            }
        }
    }
}

/// The reserved block (PosBeacon, the code, the calibration cells) in an
/// eye `w` x `h`: x0, y0, x1, y1. Nothing a face shows is in it; whatever
/// reads the whole eye leaves it out.
pub fn reserved_rect(w: u32, h: u32) -> [u32; 4] {
    let [_, y, ..] = Face::F90.block(w, h);
    let [_, _, _, bh] = Face::F90.block(w, h);
    [0, y + bh, Face::Up.block(w, h)[0], h]
}

/// `m · v`.
pub fn mul(m: &[[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// `mᵀ · v` (the inverse, for a rotation).
pub fn mul_t(m: &[[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

/// `v` turned `yaw_deg` clockwise seen from above (Unity's axes: +z to +x).
pub fn turn(v: [f32; 3], yaw_deg: f32) -> [f32; 3] {
    let (s, c) = yaw_deg.to_radians().sin_cos();
    [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f32; 3], b: [f32; 3]) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < 1e-5)
    }

    #[test]
    fn rotations_are_the_docs_formulas() {
        let (x, y) = (0.3f32, -0.2f32);
        for f in [Face::F0, Face::F90, Face::F180, Face::F270] {
            let (s, c) = f.yaw().unwrap().to_radians().sin_cos();
            assert!(close(mul(&f.rotation(), [x, y, 1.0]), [x * c + s, y, -x * s + c]), "{f:?}");
        }
        assert!(close(mul(&Face::Up.rotation(), [x, y, 1.0]), [-x, 1.0, y]));
        assert!(close(mul(&Face::Down.rotation(), [x, y, 1.0]), [x, -1.0, y]));
        // Ahead, right, behind, left: the level faces' axes.
        assert!(close(mul(&Face::F90.rotation(), [0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]));
        assert!(close(mul(&Face::F270.rotation(), [0.0, 0.0, 1.0]), [-1.0, 0.0, 0.0]));
        for f in Face::ALL {
            let r = f.rotation();
            assert!(close(mul_t(&r, mul(&r, [x, y, 1.0])), [x, y, 1.0]));
        }
        assert!(close(turn([0.0, 0.0, 1.0], 90.0), [1.0, 0.0, 0.0]));
    }

    #[test]
    fn blocks_tile_the_eye() {
        for size in [1920, 1280, 1440] {
            let mut covered = vec![0u8; (size * size) as usize];
            let r = reserved_rect(size, size);
            for y in r[1]..r[3] {
                for x in r[0]..r[2] {
                    covered[(y * size + x) as usize] += 1;
                }
            }
            for f in Face::ALL {
                let [x0, y0, w, h] = f.block(size, size);
                for y in y0..y0 + h {
                    for x in x0..x0 + w {
                        covered[(y * size + x) as usize] += 1;
                    }
                }
            }
            assert!(covered.iter().all(|&c| c == 1), "{size}");
        }
        assert_eq!(Face::F0.intrinsics(960, 720), [480.0, 480.0, 480.0, 360.0]);
        assert!((Face::Down.intrinsics(480, 480)[0] - 168.05).abs() < 0.01);
        assert_eq!(reserved_rect(1920, 1920), [0, 1440, 960, 1920]);
    }
}
