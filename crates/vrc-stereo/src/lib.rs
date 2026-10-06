//! Stereo depth from the bot's two rendered eyes.
//!
//! The tap gives both eyes with the field of view and pose each was rendered
//! with ([`vrc_vr::tap`]): same intrinsics, parallel axes, a known baseline,
//! so the pair is already rectified. [`sgm`] matches them; [`Stereo`] turns
//! disparities into metric depth and points in the tracking space.
//!
//! Units are the tracking space's metres. VRChat scales the world to the
//! avatar (a small avatar sees a big world), so these are "the bot's metres":
//! consistent with its own eye height above the floor, not necessarily the
//! world's. See docs/full-vr/decisions.md.

#[cfg(feature = "cuda")]
pub mod gpu;
pub mod sgm;

use rayon::prelude::*;
use vrc_vr::tap::{format, EyeFrame};
use vrc_vr::Pose;

pub use sgm::{Disparity, Gray, SgmParams};

/// A stereo pair ready to match, at `scale` times fewer pixels a side.
pub struct Stereo {
    pub left: Gray,
    pub right: Gray,
    /// Intrinsics of the matched (scaled) images: fx, fy, cx, cy.
    pub intrinsics: [f32; 4],
    pub baseline: f32,
    /// The left eye, as rendered.
    pub left_pose: Pose,
}

impl Stereo {
    /// The pair of a tapped frame (8-bit RGBA/BGRA formats).
    pub fn from_frame(frame: &EyeFrame, scale: usize) -> Option<Stereo> {
        let bgr = match frame.format {
            format::R8G8B8A8_UNORM | format::R8G8B8A8_SRGB => false,
            format::B8G8R8A8_UNORM | format::B8G8R8A8_SRGB => true,
            _ => return None,
        };
        let (w, h) = (frame.width as usize, frame.height as usize);
        let [fx, fy, cx, cy] = frame.views[0].fov.intrinsics(frame.width, frame.height);
        let s = scale.max(1) as f32;
        Some(Stereo {
            left: Gray::from_rgba(frame.eye(0), w, h, bgr, scale),
            right: Gray::from_rgba(frame.eye(1), w, h, bgr, scale),
            intrinsics: [fx / s, fy / s, cx / s, cy / s],
            baseline: frame.baseline(),
            left_pose: frame.views[0].pose,
        })
    }

    /// The disparities, on the GPU when there is one (the same, bit for
    /// bit), else on the CPU.
    pub fn disparity(&self, params: &SgmParams) -> Disparity {
        #[cfg(feature = "cuda")]
        if let Some(d) = gpu::sgm(&self.left, &self.right, params) {
            return d;
        }
        sgm::sgm(&self.left, &self.right, params)
    }

    /// Distance along the view axis (metres) of each pixel; NaN without one.
    pub fn depth(&self, disp: &Disparity) -> Vec<f32> {
        let fb = self.intrinsics[0] * self.baseline;
        disp.data.iter().map(|&d| if d > 0.25 { fb / d } else { f32::NAN }).collect()
    }

    /// The tracking-space point at (`x`, `y`) of the matched (scaled) left
    /// image with disparity `d`; pixel coordinates are continuous (a pixel's
    /// centre is at +0.5).
    pub fn point(&self, x: f32, y: f32, d: f32) -> [f32; 3] {
        let [fx, fy, cx, cy] = self.intrinsics;
        let z = fx * self.baseline / d;
        // OpenXR view space: x right, y up, -z ahead; pixel rows go down.
        let v = [(x - cx) / fx * z, -(y - cy) / fy * z, -z];
        let r = self.left_pose.rotate(v);
        let t = self.left_pose.position;
        [r[0] + t[0], r[1] + t[1], r[2] + t[2]]
    }

    /// Points in the tracking space (every `step`th pixel each way), and the
    /// pixel each came from.
    pub fn points(&self, disp: &Disparity, step: usize) -> Vec<([f32; 3], (u32, u32))> {
        let [fx, fy, cx, cy] = self.intrinsics;
        let fb = fx * self.baseline;
        let step = step.max(1);
        (0..disp.height)
            .into_par_iter()
            .step_by(step)
            .flat_map_iter(|y| {
                (0..disp.width).step_by(step).filter_map(move |x| {
                    let d = disp.at(x, y);
                    if !(d > 0.25) {
                        return None;
                    }
                    let z = fb / d;
                    // OpenXR view space: x right, y up, -z ahead; pixel rows go down.
                    let v = [(x as f32 + 0.5 - cx) / fx * z, -(y as f32 + 0.5 - cy) / fy * z, -z];
                    let r = self.left_pose.rotate(v);
                    let t = self.left_pose.position;
                    Some(([r[0] + t[0], r[1] + t[1], r[2] + t[2]], (x as u32, y as u32)))
                })
            })
            .collect()
    }
}

/// The floor under the eyes, from points in the tracking space.
#[derive(Clone, Copy, Debug)]
pub struct Floor {
    /// Height (y) of the floor right under the eye.
    pub height: f32,
    /// Its slope, degrees from level.
    pub tilt_deg: f32,
    pub inliers: usize,
}

/// The most populated horizontal layer at least `min_below` under the eye,
/// refined by a least-squares plane `y = a x + b z + c` through it.
pub fn fit_floor(points: &[[f32; 3]], eye: [f32; 3], min_below: f32) -> Option<Floor> {
    const BIN: f32 = 0.02;
    let below: Vec<&[f32; 3]> = points.iter().filter(|p| p[1] < eye[1] - min_below).collect();
    if below.len() < 50 {
        return None;
    }
    let lo = below.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
    let bins = ((eye[1] - lo) / BIN) as usize + 1;
    let mut hist = vec![0usize; bins];
    for p in &below {
        hist[((p[1] - lo) / BIN) as usize] += 1;
    }
    let (peak, _) = hist.iter().enumerate().max_by_key(|(_, &n)| n)?;
    let level = lo + (peak as f32 + 0.5) * BIN;
    let near: Vec<&&[f32; 3]> = below.iter().filter(|p| (p[1] - level).abs() < 0.05).collect();
    // Normal equations of y = a x + b z + c, around the eye for conditioning.
    let mut m = [[0f64; 3]; 3];
    let mut v = [0f64; 3];
    for p in &near {
        let row = [(p[0] - eye[0]) as f64, (p[2] - eye[2]) as f64, 1.0];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += row[i] * row[j];
            }
            v[i] += row[i] * p[1] as f64;
        }
    }
    let [a, b, c] = solve3(m, v)?;
    Some(Floor {
        height: c as f32,
        tilt_deg: (a.hypot(b)).atan().to_degrees() as f32,
        inliers: near.len(),
    })
}

fn solve3(mut m: [[f64; 3]; 3], mut v: [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let pivot = (col..3).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
        if m[pivot][col].abs() < 1e-9 {
            return None;
        }
        m.swap(col, pivot);
        v.swap(col, pivot);
        for row in 0..3 {
            if row != col {
                let f = m[row][col] / m[col][col];
                for k in 0..3 {
                    m[row][k] -= f * m[col][k];
                }
                v[row] -= f * v[col];
            }
        }
    }
    Some([v[0] / m[0][0], v[1] / m[1][1], v[2] / m[2][2]])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_of_a_tilted_plane_with_clutter() {
        let mut pts = Vec::new();
        for i in 0..40 {
            for j in 0..40 {
                let (x, z) = (i as f32 * 0.1 - 2.0, -(j as f32) * 0.1);
                pts.push([x, 0.02 + 0.01 * x, z]); // 0.57 degrees
            }
        }
        for i in 0..300 {
            pts.push([0.5, 0.4 + (i % 30) as f32 * 0.02, -1.0]); // a box
        }
        let f = fit_floor(&pts, [0.0, 1.6, 0.0], 0.5).unwrap();
        assert!((f.height - 0.02).abs() < 1e-3, "{f:?}");
        assert!((f.tilt_deg - 0.573).abs() < 0.05, "{f:?}");
    }
}
