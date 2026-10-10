//! Semi-global matching (Hirschmüller 2008) on census costs, for a rectified
//! pair: a point at column `x` of the left image is at `x - d` in the right.
//!
//! The bot's eyes are rendered, not photographed: same intrinsics, parallel
//! axes, no noise or exposure differences. Census (7x7) still beats raw
//! intensity on the large flat surfaces games are full of.

use rayon::prelude::*;

/// An 8-bit grey image.
#[derive(Clone, Debug)]
pub struct Gray {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl Gray {
    /// Luma of 8-bit RGBA (or BGRA) pixels, each output pixel the mean of a
    /// `scale` x `scale` block.
    pub fn from_rgba(pixels: &[u8], width: usize, height: usize, bgr: bool, scale: usize) -> Gray {
        let scale = scale.max(1);
        let (w, h) = (width / scale, height / scale);
        let (ri, bi) = if bgr { (2, 0) } else { (0, 2) };
        let mut data = vec![0u8; w * h];
        data.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
            for (x, out) in row.iter_mut().enumerate() {
                let mut sum = 0u32;
                for sy in 0..scale {
                    let base = ((y * scale + sy) * width + x * scale) * 4;
                    for sx in 0..scale {
                        let p = &pixels[base + sx * 4..base + sx * 4 + 4];
                        sum += 54 * p[ri] as u32 + 183 * p[1] as u32 + 19 * p[bi] as u32;
                    }
                }
                *out = (sum / (256 * (scale * scale) as u32)) as u8;
            }
        });
        Gray { width: w, height: h, data }
    }
}

/// Matching parameters.
#[derive(Clone, Copy, Debug)]
pub struct SgmParams {
    /// Disparities searched: 0..max_disparity.
    pub max_disparity: usize,
    /// Penalty of a 1 pixel disparity change between neighbours.
    pub p1: u16,
    /// Penalty of a larger change (a depth edge).
    pub p2: u16,
    /// A match is kept only if every non-neighbouring disparity costs at
    /// least this much more (fraction).
    pub uniqueness: f32,
    /// Largest left/right disagreement (pixels) of a kept match.
    pub lr_tolerance: f32,
}

impl Default for SgmParams {
    fn default() -> Self {
        // Census 7x7 costs are 0..=48.
        SgmParams { max_disparity: 64, p1: 8, p2: 96, uniqueness: 0.05, lr_tolerance: 1.0 }
    }
}

/// Disparities of the left image, pixels; NaN where no match was kept.
#[derive(Clone, Debug)]
pub struct Disparity {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f32>,
}

impl Disparity {
    pub fn at(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.width + x]
    }

    /// Fraction of pixels with a disparity.
    pub fn density(&self) -> f32 {
        self.data.iter().filter(|d| d.is_finite()).count() as f32 / self.data.len() as f32
    }
}

const CENSUS_R: isize = 3;

/// 7x7 census: one bit per neighbour, set when it is darker than the centre
/// (out of the image counts as the edge pixel).
pub fn census(img: &Gray) -> Vec<u64> {
    let (w, h) = (img.width as isize, img.height as isize);
    let mut out = vec![0u64; img.data.len()];
    out.par_chunks_mut(img.width).enumerate().for_each(|(y, row)| {
        let y = y as isize;
        for (x, bits) in row.iter_mut().enumerate() {
            let x = x as isize;
            let c = img.data[(y * w + x) as usize];
            let mut v = 0u64;
            for dy in -CENSUS_R..=CENSUS_R {
                let yy = (y + dy).clamp(0, h - 1);
                for dx in -CENSUS_R..=CENSUS_R {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let xx = (x + dx).clamp(0, w - 1);
                    v = (v << 1) | (img.data[(yy * w + xx) as usize] < c) as u64;
                }
            }
            *bits = v;
        }
    });
    out
}

/// Disparity of `left` against `right` (same size).
pub fn sgm(left: &Gray, right: &Gray, p: &SgmParams) -> Disparity {
    assert_eq!((left.width, left.height), (right.width, right.height));
    let (w, h, nd) = (left.width, left.height, p.max_disparity);
    let (cl, cr) = (census(left), census(right));

    // Matching costs, (y, x, d); out of the right image costs the most.
    const OUTSIDE: u8 = 48;
    let mut cost = vec![0u8; w * h * nd];
    cost.par_chunks_mut(w * nd).enumerate().for_each(|(y, row)| {
        for x in 0..w {
            let l = cl[y * w + x];
            for d in 0..nd {
                row[x * nd + d] =
                    if x >= d { (l ^ cr[y * w + x - d]).count_ones() as u8 } else { OUTSIDE };
            }
        }
    });

    // Aggregated costs over 8 paths.
    let mut sum = vec![0u16; w * h * nd];
    for dx in [-1isize, 1] {
        aggregate_row_paths(&cost, &mut sum, w, nd, dx, p);
    }
    for dy in [-1isize, 1] {
        for dx in [-1isize, 0, 1] {
            aggregate_column_paths(&cost, &mut sum, w, h, nd, dx, dy, p);
        }
    }

    // Right image disparities (for the left/right check): the best d of the
    // left pixel x + d that would show this right pixel.
    let mut right_best = vec![u16::MAX; w * h];
    right_best.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        for (xr, best) in row.iter_mut().enumerate() {
            let mut min = u16::MAX;
            for d in 0..nd.min(w - xr) {
                let s = sum[(y * w + xr + d) * nd + d];
                if s < min {
                    min = s;
                    *best = d as u16;
                }
            }
        }
    });

    let mut data = vec![f32::NAN; w * h];
    data.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        for (x, out) in row.iter_mut().enumerate() {
            let s = &sum[(y * w + x) * nd..(y * w + x + 1) * nd];
            let n = nd.min(x + 1);
            let (mut best, mut best_d) = (u16::MAX, 0usize);
            for (d, &v) in s[..n].iter().enumerate() {
                if v < best {
                    best = v;
                    best_d = d;
                }
            }
            let other = s[..n]
                .iter()
                .enumerate()
                .filter(|(d, _)| d.abs_diff(best_d) > 1)
                .map(|(_, &v)| v)
                .min()
                .unwrap_or(u16::MAX);
            if (best as f32) * (1.0 + p.uniqueness) > other as f32 {
                continue;
            }
            let mut disp = best_d as f32;
            if best_d > 0 && best_d + 1 < n {
                let (a, b, c) = (s[best_d - 1] as f32, best as f32, s[best_d + 1] as f32);
                let denom = a - 2.0 * b + c;
                if denom > 0.0 {
                    disp += (a - c) / (2.0 * denom);
                }
            }
            let xr = (x as f32 - disp).round();
            if xr < 0.0 {
                continue;
            }
            let back = right_best[y * w + xr as usize];
            if back != u16::MAX && (back as f32 - disp).abs() <= p.lr_tolerance {
                *out = disp;
            }
        }
    });
    Disparity { width: w, height: h, data }
}

/// One step of a path: `out[d] = c[d] + min(prev[d], prev[d +- 1] + p1,
/// min(prev) + p2) - min(prev)`; returns min(out).
#[inline]
fn step(c: &[u8], prev: &[u16], prev_min: u16, out: &mut [u16], p: &SgmParams) -> u16 {
    let nd = c.len();
    let jump = prev_min + p.p2;
    let mut min = u16::MAX;
    for d in 0..nd {
        let mut m = prev[d].min(jump);
        if d > 0 {
            m = m.min(prev[d - 1] + p.p1);
        }
        if d + 1 < nd {
            m = m.min(prev[d + 1] + p.p1);
        }
        let v = c[d] as u16 + m - prev_min;
        out[d] = v;
        min = min.min(v);
    }
    min
}

/// Paths along rows (left to right for `dx` 1, right to left for -1).
fn aggregate_row_paths(cost: &[u8], sum: &mut [u16], w: usize, nd: usize, dx: isize, p: &SgmParams) {
    sum.par_chunks_mut(w * nd).enumerate().for_each(|(y, srow)| {
        let crow = &cost[y * w * nd..(y + 1) * w * nd];
        let mut prev = vec![0u16; nd];
        let mut cur = vec![0u16; nd];
        let mut prev_min = 0u16;
        for i in 0..w {
            let x = if dx > 0 { i } else { w - 1 - i };
            let c = &crow[x * nd..(x + 1) * nd];
            if i == 0 {
                for d in 0..nd {
                    cur[d] = c[d] as u16;
                }
                prev_min = *cur.iter().min().unwrap();
            } else {
                prev_min = step(c, &prev, prev_min, &mut cur, p);
            }
            for d in 0..nd {
                srow[x * nd + d] += cur[d];
            }
            std::mem::swap(&mut prev, &mut cur);
        }
    });
}

/// Paths coming from the row above (`dy` 1) or below (-1), straight (`dx` 0)
/// or diagonal: a pixel follows the one at (x - dx, y - dy).
#[allow(clippy::too_many_arguments)]
fn aggregate_column_paths(
    cost: &[u8],
    sum: &mut [u16],
    w: usize,
    h: usize,
    nd: usize,
    dx: isize,
    dy: isize,
    p: &SgmParams,
) {
    let mut prev = vec![0u16; w * nd];
    let mut prev_min = vec![0u16; w];
    let mut cur = vec![0u16; w * nd];
    let mut cur_min = vec![0u16; w];
    for i in 0..h {
        let y = if dy > 0 { i } else { h - 1 - i };
        let crow = &cost[y * w * nd..(y + 1) * w * nd];
        let srow = &mut sum[y * w * nd..(y + 1) * w * nd];
        cur.par_chunks_mut(nd)
            .zip(cur_min.par_iter_mut())
            .zip(srow.par_chunks_mut(nd))
            .enumerate()
            .with_min_len(64)
            .for_each(|(x, ((out, out_min), s))| {
                let c = &crow[x * nd..(x + 1) * nd];
                let px = x as isize - dx;
                if i == 0 || px < 0 || px >= w as isize {
                    for d in 0..nd {
                        out[d] = c[d] as u16;
                    }
                    *out_min = *out.iter().min().unwrap();
                } else {
                    let px = px as usize;
                    *out_min = step(c, &prev[px * nd..(px + 1) * nd], prev_min[px], out, p);
                }
                for d in 0..nd {
                    s[d] += out[d];
                }
            });
        std::mem::swap(&mut prev, &mut cur);
        std::mem::swap(&mut prev_min, &mut cur_min);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A textured image and its view shifted by `shift` pixels (rows
    /// `0..h/2`) and `shift2` (the rest): a two-plane scene.
    fn pair(w: usize, h: usize, shift: usize, shift2: usize) -> (Gray, Gray) {
        let mut seed = 12345u32;
        let mut noise = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed % 256) as u8
        };
        let src_w = w + 64;
        let src: Vec<u8> = (0..src_w * h).map(|_| noise()).collect();
        let mut l = vec![0u8; w * h];
        let mut r = vec![0u8; w * h];
        for y in 0..h {
            let s = if y < h / 2 { shift } else { shift2 };
            for x in 0..w {
                l[y * w + x] = src[y * src_w + x + 32];
                r[y * w + x] = src[y * src_w + x + 32 + s];
            }
        }
        (Gray { width: w, height: h, data: l }, Gray { width: w, height: h, data: r })
    }

    #[test]
    fn finds_two_planes() {
        let (l, r) = pair(160, 80, 7, 21);
        let disp = sgm(&l, &r, &SgmParams { max_disparity: 32, ..Default::default() });
        let mut good = 0;
        let mut total = 0;
        for y in 4..76 {
            if (38..42).contains(&y) {
                continue; // the depth edge
            }
            let want = if y < 40 { 7.0 } else { 21.0 };
            for x in 30..156 {
                total += 1;
                let d = disp.at(x, y);
                if d.is_finite() && (d - want).abs() < 0.5 {
                    good += 1;
                }
            }
        }
        assert!(good as f32 > 0.95 * total as f32, "{good}/{total}");
    }

    #[test]
    fn grey_of_rgba_blocks() {
        let px = [255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255];
        let g = Gray::from_rgba(&px, 2, 2, false, 2);
        assert_eq!((g.width, g.height), (1, 1));
        assert_eq!(g.data[0], (255 * 54 / 256) as u8);
        let g = Gray::from_rgba(&px, 2, 2, true, 1);
        assert_eq!(g.data[0], (255 * 19 / 256) as u8);
    }
}
