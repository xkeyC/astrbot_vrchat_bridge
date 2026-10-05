//! Equirectangular panoramas from head-scan frames.
//!
//! Longitude 0 is the tracking space's -Z (ahead at yaw 0), growing to the
//! right; latitude grows up. Each output pixel takes the left-eye frame that
//! sees its direction closest to that frame's centre. The eyes turn about
//! the head centre, 3 cm off: parallax only matters closer than a metre.

use rayon::prelude::*;
use vrc_vr::tap::{format, EyeFrame};

/// An RGB8 equirectangular image, `width` = 2 x `height`.
pub struct Panorama {
    pub width: usize,
    pub height: usize,
    pub rgb: Vec<u8>,
    /// Fraction of pixels some frame saw.
    pub coverage: f32,
}

impl Panorama {
    /// Pixel (column, row) showing the direction from `eye` to `point`
    /// (tracking space).
    pub fn pixel_of(&self, eye: [f32; 3], point: [f32; 3]) -> (f32, f32) {
        let (dx, dy, dz) = (point[0] - eye[0], point[1] - eye[1], point[2] - eye[2]);
        let lon = dx.atan2(-dz);
        let lat = dy.atan2(dx.hypot(dz));
        let col = (lon + std::f32::consts::PI) / std::f32::consts::TAU * self.width as f32;
        let row = (std::f32::consts::FRAC_PI_2 - lat) / std::f32::consts::PI * self.height as f32;
        (col, row)
    }

    /// Stitches the left eyes of `frames` into a `width` x `width/2` panorama.
    pub fn stitch(frames: &[&EyeFrame], width: usize) -> Panorama {
        let height = width / 2;
        let views: Vec<View> = frames.iter().filter_map(|f| View::new(f)).collect();
        let mut rgb = vec![0u8; width * height * 3];
        let seen: usize = rgb
            .par_chunks_mut(width * 3)
            .enumerate()
            .map(|(row, out)| {
                let lat = std::f32::consts::FRAC_PI_2 - (row as f32 + 0.5) / height as f32 * std::f32::consts::PI;
                let mut seen = 0;
                for col in 0..width {
                    let lon = (col as f32 + 0.5) / width as f32 * std::f32::consts::TAU - std::f32::consts::PI;
                    let dir = [lon.sin() * lat.cos(), lat.sin(), -lon.cos() * lat.cos()];
                    let best = views
                        .iter()
                        .filter_map(|v| v.sample(dir))
                        .max_by(|a, b| a.0.total_cmp(&b.0));
                    if let Some((_, px)) = best {
                        out[col * 3..col * 3 + 3].copy_from_slice(&px);
                        seen += 1;
                    }
                }
                seen
            })
            .sum();
        Panorama { width, height, rgb, coverage: seen as f32 / (width * height) as f32 }
    }
}

struct View<'a> {
    frame: &'a EyeFrame,
    intrinsics: [f32; 4],
    bgr: bool,
}

impl<'a> View<'a> {
    fn new(frame: &'a EyeFrame) -> Option<View<'a>> {
        let bgr = match frame.format {
            format::R8G8B8A8_UNORM | format::R8G8B8A8_SRGB => false,
            format::B8G8R8A8_UNORM | format::B8G8R8A8_SRGB => true,
            _ => return None,
        };
        Some(View { frame, intrinsics: frame.views[0].fov.intrinsics(frame.width, frame.height), bgr })
    }

    /// How centrally this view sees `dir` (cosine to its axis) and the pixel there.
    fn sample(&self, dir: [f32; 3]) -> Option<(f32, [u8; 3])> {
        let v = self.frame.views[0].pose.unrotate(dir);
        if v[2] >= -1e-3 {
            return None; // behind
        }
        let [fx, fy, cx, cy] = self.intrinsics;
        let z = -v[2];
        let u = cx + fx * v[0] / z;
        let w = cy - fy * v[1] / z;
        let (width, height) = (self.frame.width as f32, self.frame.height as f32);
        if !(0.0..width).contains(&u) || !(0.0..height).contains(&w) {
            return None;
        }
        let i = (w as usize * self.frame.width as usize + u as usize) * 4;
        let p = &self.frame.eye(0)[i..i + 4];
        let px = if self.bgr { [p[2], p[1], p[0]] } else { [p[0], p[1], p[2]] };
        Some((z / (v[0] * v[0] + v[1] * v[1] + z * z).sqrt(), px))
    }
}
