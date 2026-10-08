//! Pictures of a pano frame: the equirectangular panorama (colour and
//! depth) and the raw tiles.
//!
//! The equirect follows `vrc-scene`'s `Panorama`: `width` x `width / 2`,
//! longitude growing to the right, latitude up; each pixel from the view
//! that sees its direction most centrally (`Panorama::stitch`'s rule). Its
//! middle column looks along `centre_yaw` (Unity's world yaw, clockwise
//! from +z: the bot's heading convention). Centred on the head's yaw it is
//! what `vrc-nav`'s `marked_panorama` shows (ahead in the middle, right to
//! the right); centred on 0, `Panorama::pixel_of` places map points on it
//! (map -z is Unity's +z).

use rayon::prelude::*;
use vrc_scene::Panorama;

use crate::frame::PanoFrame;

pub struct Equirect {
    /// The colour (coverage: the fraction some view saw: 1 for a full rig).
    pub pano: Panorama,
    /// Metres along each pixel's ray (not the camera's axis); NaN: none.
    pub range: Vec<f32>,
    /// The middle column's world yaw (degrees).
    pub centre_yaw: f32,
}

impl Equirect {
    /// Column and row of a bearing from the middle (+ right) and an
    /// elevation (degrees).
    pub fn pixel_of(&self, bearing: f32, elevation: f32) -> (f32, f32) {
        let (w, h) = (self.pano.width as f32, self.pano.height as f32);
        ((crate::frame::wrap(bearing) + 180.0) / 360.0 * w, (90.0 - elevation) / 180.0 * h)
    }
}

impl PanoFrame {
    /// The `width` x `width / 2` panorama, its middle along `centre_yaw`.
    pub fn equirect(&self, width: usize, centre_yaw: f32) -> Equirect {
        let width = width.max(2) & !1;
        let height = width / 2;
        let mut rgb = vec![0u8; width * height * 3];
        let mut range = vec![f32::NAN; width * height];
        // Rig azimuth = world yaw - rig yaw.
        let offset = (centre_yaw - self.code.rig_yaw).to_radians();
        // Per column: the rig azimuth's sine and cosine, and the level face
        // nearest it (`Face::ALL`'s order).
        let columns: Vec<(f32, f32, usize)> = (0..width)
            .map(|col| {
                let az = offset + (col as f32 + 0.5) / width as f32 * std::f32::consts::TAU - std::f32::consts::PI;
                let (sa, ca) = az.sin_cos();
                (sa, ca, ((az / std::f32::consts::FRAC_PI_2).round() as i32).rem_euclid(4) as usize)
            })
            .collect();
        let seen: usize = rgb
            .par_chunks_mut(width * 3)
            .zip(range.par_chunks_mut(width))
            .enumerate()
            .map(|(row, (out, dist))| {
                let lat = std::f32::consts::FRAC_PI_2 - (row as f32 + 0.5) / height as f32 * std::f32::consts::PI;
                let (sl, cl) = lat.sin_cos();
                let vertical = if sl >= 0.0 { 4 } else { 5 };
                let mut seen = 0;
                for (col, &(sa, ca, level)) in columns.iter().enumerate() {
                    let rig = [sa * cl, sl, ca * cl];
                    // The level face nearest the azimuth, and up or down:
                    // the two that see it most centrally; all six only if
                    // neither has it. The UI's colour is none: another view
                    // seeing the same direction gives it, else it stays
                    // black.
                    let mut best: Option<(&crate::PanoView, f32, f32, f32)> = None;
                    for k in [level, vertical] {
                        if let Some((u, w, c)) = self.views[k].project(rig) {
                            if best.is_none_or(|b| c > b.3) {
                                best = Some((&self.views[k], u, w, c));
                            }
                        }
                    }
                    if best.is_none() {
                        best = self.views.iter().filter_map(|v| v.project(rig).map(|(u, w, c)| (v, u, w, c))).max_by(|a, b| a.3.total_cmp(&b.3));
                    }
                    if let Some((v, u, w, _)) = best {
                        let (i, j) = (u as u32, w as u32);
                        let colour = v.colour_at(i, j).or_else(|| {
                            self.views.iter().filter(|o| !std::ptr::eq(*o, v)).find_map(|o| o.project(rig).and_then(|(u, w, _)| o.colour_at(u as u32, w as u32)))
                        });
                        if let Some(rgb) = colour {
                            out[col * 3..col * 3 + 3].copy_from_slice(&rgb);
                        }
                        if let Some(z) = v.depth_at(i, j) {
                            let c = v.camera_dir(u, w);
                            dist[col] = z * (c[0] * c[0] + c[1] * c[1] + 1.0).sqrt();
                        }
                        seen += 1;
                    }
                }
                seen
            })
            .sum();
        let pano = Panorama { width, height, rgb, coverage: seen as f32 / (width * height) as f32 };
        Equirect { pano, range, centre_yaw }
    }

    /// A perspective view `width` x `height` (RGB8) out of the panorama:
    /// looking along world yaw `yaw` (clockwise from +z, the beacon's
    /// convention) and `pitch` (+ up), `fov` degrees wide; each pixel from
    /// the face that sees it most centrally (the UI's colour taken from
    /// another face, else black).
    pub fn perspective(&self, yaw: f32, pitch: f32, fov: f32, width: usize, height: usize) -> Vec<u8> {
        let (sy, cy) = yaw.to_radians().sin_cos();
        let (sp, cp) = pitch.to_radians().sin_cos();
        let fwd = [sy * cp, sp, cy * cp];
        let right = [cy, 0.0, -sy];
        let up = [-sy * sp, cp, -cy * sp];
        let f = (width as f32 / 2.0) / (fov.clamp(10.0, 170.0).to_radians() / 2.0).tan();
        let mut rgb = vec![0u8; width * height * 3];
        rgb.par_chunks_mut(width * 3).enumerate().for_each(|(j, row)| {
            let y = (height as f32 / 2.0 - (j as f32 + 0.5)) / f;
            for i in 0..width {
                let x = (i as f32 + 0.5 - width as f32 / 2.0) / f;
                let d = [0, 1, 2].map(|k| fwd[k] + x * right[k] + y * up[k]);
                let rig = self.world_to_rig(d);
                let mut seen: Vec<(&crate::PanoView, f32, f32, f32)> = self.views.iter().filter_map(|v| v.project(rig).map(|(u, w, c)| (v, u, w, c))).collect();
                seen.sort_by(|a, b| b.3.total_cmp(&a.3));
                if let Some(c) = seen.iter().find_map(|(v, u, w, _)| v.colour_at(*u as u32, *w as u32)) {
                    row[i * 3..i * 3 + 3].copy_from_slice(&c);
                }
            }
        });
        rgb
    }

    /// The panorama with where the head looks in the middle.
    pub fn heading_equirect(&self, width: usize) -> Equirect {
        self.equirect(width, self.head.yaw)
    }

    /// The tiles as the eyes hold them: the colour (left eye) and the depth
    /// decoded (`depth_rgb`; black: none), side by side; the reserved block
    /// dark. (`2 x eye width`, eye height, RGB8.)
    pub fn tiles_rgb(&self) -> (usize, usize, Vec<u8>) {
        let [w, h] = self.eye_size.map(|v| v as usize);
        let mut out = vec![24u8; 2 * w * h * 3];
        for v in &self.views {
            let [bx, by, bw, bh] = v.block.map(|v| v as usize);
            let depth = depth_rgb(&v.depth, self.code.zmin, self.code.zmax);
            for j in 0..bh {
                let row = (by + j) * 2 * w;
                out[(row + bx) * 3..(row + bx + bw) * 3].copy_from_slice(&v.rgb[j * bw * 3..(j + 1) * bw * 3]);
                out[(row + w + bx) * 3..(row + w + bx + bw) * 3].copy_from_slice(&depth[j * bw * 3..(j + 1) * bw * 3]);
            }
        }
        (2 * w, h, out)
    }
}

/// Depth (metres; NaN none) as colours over the log range: near red, far
/// blue (turbo, its darkest end left out so that far is not black); none
/// black.
pub fn depth_rgb(depth: &[f32], zmin: f32, zmax: f32) -> Vec<u8> {
    let ln_range = (zmax / zmin).ln();
    let mut out = vec![0u8; depth.len() * 3];
    out.par_chunks_mut(3 * 4096).zip(depth.par_chunks(4096)).for_each(|(o, d)| {
        for (px, &z) in o.chunks_exact_mut(3).zip(d) {
            if z.is_finite() {
                px.copy_from_slice(&turbo(1.0 - 0.9 * ((z / zmin).ln() / ln_range).clamp(0.0, 1.0)));
            }
        }
    });
    out
}

/// Google's turbo colour map (polynomial fit), `t` 0..1: blue to red.
fn turbo(t: f32) -> [u8; 3] {
    let r = 34.61 + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05))));
    let g = 23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56))));
    let b = 27.2 + t * (3211.1 - t * (15327.97 - t * (27814.0 - t * (22569.18 - t * 6838.66))));
    [r, g, b].map(|v| v.clamp(0.0, 255.0) as u8)
}

/// `rgb` (`w` x `h`) box-averaged down to `out_w` wide (as is when not
/// narrower): (width, height, RGB8).
pub fn downscale(rgb: &[u8], w: usize, h: usize, out_w: usize) -> (usize, usize, Vec<u8>) {
    if out_w == 0 || out_w >= w {
        return (w, h, rgb.to_vec());
    }
    let out_h = (h * out_w / w).max(1);
    let mut out = vec![0u8; out_w * out_h * 3];
    out.par_chunks_mut(out_w * 3).enumerate().for_each(|(y, row)| {
        let (y0, y1) = (y * h / out_h, ((y + 1) * h / out_h).max(y * h / out_h + 1));
        for x in 0..out_w {
            let (x0, x1) = (x * w / out_w, ((x + 1) * w / out_w).max(x * w / out_w + 1));
            let mut sum = [0u32; 3];
            for yy in y0..y1 {
                for xx in x0..x1 {
                    for k in 0..3 {
                        sum[k] += rgb[(yy * w + xx) * 3 + k] as u32;
                    }
                }
            }
            let n = ((x1 - x0) * (y1 - y0)) as u32;
            for k in 0..3 {
                row[x * 3 + k] = (sum[k] / n) as u8;
            }
        }
    });
    (out_w, out_h, out)
}
