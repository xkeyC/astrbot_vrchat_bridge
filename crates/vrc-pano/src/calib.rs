//! The right eye's calibration cells: the world's post-processing (dither,
//! tone mapping, grading, vignetting) runs over the depth too, so each
//! frame's bytes are mapped back to the code values the shader wrote before
//! they are decoded.
//!
//! The depth is E1C (`pano-depth-check-spec.md`): R = q, G = 255 - q,
//! B = |2q - 255|, q the 8-bit log depth. The cells are drawn the same way
//! for their nominal q, and each channel is fitted on its own (grading may
//! treat R, G and B differently).
//!
//! Model, per channel: observed = F(g(r) · L(code)), L the code's linear
//! light (sRGB), g the vignetting at NDC radius r, F monotone. The strip
//! (16 levels, q = 0, 17, .., 255, at r 0.89-1.19) gives F's knots; the
//! four q = 64 | 192 pairs (r 1.14, 1.01, 0.68, 0.59) give g. Both come out
//! of one linear least squares: F⁻¹ is piecewise linear through the
//! strip's observed bytes, g(r) = 1 + a (r² - 1) + b (r² - 1)².
//!
//! VRChat's own UI (nameplates, the user camera's viewfinder) is drawn
//! over the eyes after the avatar's HUD and may cover cells (measured
//! 2026-10-08: the viewfinder over three of the four pairs). A cell that is
//! not even, or whose three channels do not tell one q, is left out; and
//! the cell a channel's fit misses most is dropped and the fit run again
//! while it leaves more than `ROBUST_RESIDUAL` levels.

/// A rectangle in NDC (y up): x0, y0, x1, y1.
type Ndc = [f32; 4];

/// The strip's cell `k` (q = 17k).
fn strip_cell(k: usize) -> Ndc {
    let x = -0.96 + 0.025 * k as f32;
    [x, -0.69, x + 0.025, -0.665]
}

/// The vignetting pairs' bottom-left corners; each a q = 64 cell and a
/// q = 192 cell to its right, 0.025 square.
const PAIRS: [[f32; 2]; 4] = [[-0.66, -0.96], [-0.36, -0.96], [-0.36, -0.60], [-0.10, -0.60]];

/// The calibration cells: (q, rectangle).
pub fn cells() -> Vec<(u8, Ndc)> {
    let mut out: Vec<(u8, Ndc)> = (0..16).map(|k| ((17 * k) as u8, strip_cell(k))).collect();
    for [x, y] in PAIRS {
        out.push((64, [x, y, x + 0.025, y + 0.025]));
        out.push((192, [x + 0.025, y, x + 0.05, y + 0.025]));
    }
    out
}

/// E1C: channel `ch`'s code for level `q` (0..255): q, 255 - q, |2q - 255|.
pub fn e1c(q: f32, ch: usize) -> f32 {
    match ch {
        0 => q,
        1 => 255.0 - q,
        _ => (2.0 * q - 255.0).abs().min(255.0),
    }
}

/// sRGB byte value (0..255, fractional) to linear light.
pub fn srgb_to_linear(c: f32) -> f32 {
    let c = c / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// Linear light to an sRGB byte value (fractional, 0..255).
pub fn linear_to_srgb(l: f32) -> f32 {
    let l = l.clamp(0.0, 1.0);
    255.0 * if l <= 0.003_130_8 { l * 12.92 } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 }
}

/// Entries of the linear -> code table (over 0..1).
const TO_CODE: usize = 4096;
/// A cell is not even (UI over part of it) when a channel's bytes spread
/// more than this over its middle (dither is a level or two).
const MAX_SPREAD: f32 = 12.0;
/// A cell's three channels, mapped back, must tell one q within these
/// (levels: |qR - qG|, |tri(qR) - B|); twice the pixels' tolerances.
const CELL_RG: f32 = 8.0;
const CELL_B: f32 = 12.0;
/// A channel's fit is run again without its worst cell while it leaves
/// more than this (code levels), up to `MAX_DROPS` times.
const ROBUST_RESIDUAL: f32 = 2.0;
const MAX_DROPS: usize = 4;

/// One channel's mapping back: observed byte to code value.
#[derive(Clone, Debug)]
pub struct Channel {
    /// Observed byte -> linear light before the vignetting (F⁻¹).
    finv: Vec<f32>,
    /// g(r) = 1 + a (r² - 1) + b (r² - 1)².
    pub a: f32,
    pub b: f32,
    /// The worst cell after the fit, in code levels.
    pub residual: f32,
    /// The fit ran (the strip read as a scale); else the identity.
    pub fitted: bool,
    /// Cells dropped by the robust refit.
    pub dropped: u8,
    to_code: Vec<f32>,
}

impl Channel {
    /// No post-processing: bytes are the codes.
    pub fn identity() -> Channel {
        Channel::with((0..256).map(|y| srgb_to_linear(y as f32)).collect(), 0.0, 0.0, 0.0, false)
    }

    fn with(finv: Vec<f32>, a: f32, b: f32, residual: f32, fitted: bool) -> Channel {
        // Linear -> code, interpolated (exact on sRGB's linear segment).
        let to_code = (0..=TO_CODE).map(|i| linear_to_srgb(i as f32 / TO_CODE as f32)).collect();
        Channel { finv, a, b, residual, fitted, dropped: 0, to_code }
    }

    /// The vignetting's gain at NDC radius² `r2`.
    pub fn gain(&self, r2: f32) -> f32 {
        let s = r2 - 1.0;
        (1.0 + self.a * s + self.b * s * s).max(0.05)
    }

    /// The code value (0..255, fractional) a `byte` at NDC radius² `r2`
    /// stands for.
    #[inline]
    pub fn code(&self, byte: u8, r2: f32) -> f32 {
        self.code_of_linear(self.finv[byte as usize] / self.gain(r2))
    }

    /// Code levels a byte is worth there (one byte of dither moves the
    /// code this much): 1 without grading, more where a curve compresses.
    pub fn slope(&self, byte: u8, r2: f32) -> f32 {
        let (lo, hi) = (byte.saturating_sub(1), byte.saturating_add(1));
        ((self.code(hi, r2) - self.code(lo, r2)) / (hi - lo).max(1) as f32).abs().max(0.25)
    }

    /// The same for a fractional byte (a cell's mean).
    fn code_at(&self, y: f32, r2: f32) -> f32 {
        let (lo, f) = (y.floor().clamp(0.0, 254.0), y - y.floor().clamp(0.0, 254.0));
        let l = self.finv[lo as usize] * (1.0 - f) + self.finv[lo as usize + 1] * f;
        self.code_of_linear(l / self.gain(r2))
    }

    #[inline]
    fn code_of_linear(&self, l: f32) -> f32 {
        let x = l.clamp(0.0, 1.0) * TO_CODE as f32;
        let i = (x as usize).min(TO_CODE - 1);
        let f = x - i as f32;
        self.to_code[i] * (1.0 - f) + self.to_code[i + 1] * f
    }
}

/// The three channels' mapping back.
#[derive(Clone, Debug)]
pub struct Calibration {
    /// R, G, B.
    pub channels: [Channel; 3],
    /// The worst cell of R and G (the depth's), code levels.
    pub residual: f32,
    /// Every channel fitted.
    pub fitted: bool,
    /// Cells left out: not even, or their channels not one q (UI over
    /// them); of 24.
    pub covered: u8,
    /// Cells dropped by the robust refits (the most of any channel).
    pub dropped: u8,
}

/// A cell as read in one channel: its code, mean byte, NDC radius², and
/// whether it is of the strip.
#[derive(Clone, Copy, Debug)]
struct Seen {
    code: f32,
    y: f32,
    r2: f32,
    strip: bool,
}

/// A cell as read: q, NDC radius², per channel its mean and spread.
#[derive(Clone, Copy, Debug)]
struct Cell {
    q: f32,
    r2: f32,
    strip: bool,
    mean: [f32; 3],
    spread: [f32; 3],
}

impl Calibration {
    /// No post-processing.
    pub fn identity() -> Calibration {
        Calibration { channels: [Channel::identity(), Channel::identity(), Channel::identity()], residual: 0.0, fitted: false, covered: 0, dropped: 0 }
    }

    /// `rgb`'s q (R's and G's, averaged) and the check's two residuals
    /// (|qR - qG|, |tri(qR) - B|) in units of what a byte is worth there
    /// (`check_scale`), at NDC radius² `r2`.
    pub fn read(&self, rgb: [u8; 3], r2: f32) -> (f32, f32, f32) {
        let [r, g, b] = &self.channels;
        let (qr, qg, qb) = (r.code(rgb[0], r2), 255.0 - g.code(rgb[1], r2), b.code(rgb[2], r2));
        let (sr, sb) = check_scale([0, 1, 2].map(|ch| self.channels[ch].slope(rgb[ch], r2)));
        ((qr + qg) / 2.0, (qr - qg).abs() / sr, (e1c(qr, 2) - qb).abs() / sb)
    }

    /// Fits the cells of a right eye (tightly packed 4-byte pixels, `w` x
    /// `h`). A channel whose strip does not read as a scale stays the
    /// identity (`fitted` false).
    pub fn fit(px: &[u8], w: u32, h: u32) -> Calibration {
        let all: Vec<Cell> = cells()
            .into_iter()
            .enumerate()
            .map(|(k, (q, r))| {
                let (mean, spread) = sample(px, w, h, r);
                Cell { q: q as f32, r2: radius2(r), strip: k < 16, mean, spread }
            })
            .collect();
        let even: Vec<bool> = all.iter().map(|c| c.spread.iter().all(|&s| s <= MAX_SPREAD)).collect();
        // Cells whose channels do not tell one q are UI over them. Judged
        // first by the strip's fit alone (the pairs, more often covered,
        // cannot pull it), then by the fit on the cells that agree, twice.
        let strip_only: Vec<bool> = all.iter().zip(&even).map(|(c, &e)| e && c.strip).collect();
        let mut keep = even.clone();
        let mut cal = fit_channels(&all, &strip_only);
        if !cal.fitted {
            cal = fit_channels(&all, &keep);
        }
        for _ in 0..3 {
            let ok: Vec<bool> = all.iter().zip(&even).map(|(c, &e)| e && agrees(&cal, c)).collect();
            if ok.iter().filter(|&&k| k).count() < 12 {
                break;
            }
            let same = ok == keep;
            keep = ok;
            cal = fit_channels(&all, &keep);
            if same {
                break;
            }
        }
        cal.covered = keep.iter().filter(|&&k| !k).count() as u8;
        cal
    }
}

/// The check's two residuals are compared in bytes' worth: `s` the levels
/// a byte is worth in R, G, B there; |qR - qG| over (sR + sG) / 2, and
/// |tri(qR) - B| (tri doubles R's error) over (2 sR + sB) / 3. Both 1
/// without grading: the tolerances are then levels.
#[inline]
pub fn check_scale(s: [f32; 3]) -> (f32, f32) {
    ((s[0] + s[1]) / 2.0, (2.0 * s[0] + s[2]) / 3.0)
}

/// Whether a cell's channels, mapped back, tell its q.
fn agrees(cal: &Calibration, c: &Cell) -> bool {
    let [r, g, b] = &cal.channels;
    let (qr, qg, qb) = (r.code_at(c.mean[0], c.r2), 255.0 - g.code_at(c.mean[1], c.r2), b.code_at(c.mean[2], c.r2));
    (qr - qg).abs() <= CELL_RG && (e1c(qr, 2) - qb).abs() <= CELL_B && ((qr + qg) / 2.0 - c.q).abs() <= CELL_RG
}

/// The channels fitted together on the cells kept (each its F, one g:
/// PPv2's vignette scales the light before the grading), then the cell the
/// fit misses most (in any channel) dropped and fitted again while it
/// misses by more than `ROBUST_RESIDUAL`.
fn fit_channels(all: &[Cell], keep: &[bool]) -> Calibration {
    let mut idx: Vec<usize> = (0..all.len()).filter(|&i| keep[i]).collect();
    let worst = |c: &Calibration, idx: &[usize]| -> (usize, f32) {
        idx.iter()
            .map(|&i| (i, (0..3).map(|ch| cell_error(&c.channels[ch], &seen(&all[i], ch))).fold(0.0, f32::max)))
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, 0.0))
    };
    let mut dropped = 0;
    let mut best = fit_joint(all, &idx);
    while let Some(c) = &best {
        let err = worst(c, &idx).1;
        if err <= ROBUST_RESIDUAL || dropped >= MAX_DROPS || idx.len() <= 12 {
            break;
        }
        // Least squares spreads a bad cell's error over the others: each
        // cell is tried out, the drop that leaves the least kept.
        let tried = idx.iter().filter_map(|&w| {
            let fewer: Vec<usize> = idx.iter().copied().filter(|&i| i != w).collect();
            let next = fit_joint(all, &fewer)?;
            let e = worst(&next, &fewer).1;
            Some((fewer, next, e))
        });
        match tried.min_by(|a, b| a.2.total_cmp(&b.2)) {
            Some((fewer, next, e)) if e < err => {
                idx = fewer;
                dropped += 1;
                best = Some(next);
            }
            _ => break,
        }
    }
    match best {
        Some(mut c) => {
            c.dropped = dropped as u8;
            c
        }
        None => Calibration::identity(),
    }
}

/// A cell as channel `ch` read it.
fn seen(c: &Cell, ch: usize) -> Seen {
    Seen { code: e1c(c.q, ch), y: c.mean[ch], r2: c.r2, strip: c.strip }
}

/// The squared NDC radius of a cell's middle.
fn radius2(r: Ndc) -> f32 {
    let (x, y) = ((r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0);
    x * x + y * y
}

/// A cell's middle half (both ways): per channel its mean byte (dither
/// averaged out) and spread (max - min).
fn sample(px: &[u8], w: u32, h: u32, r: Ndc) -> ([f32; 3], [f32; 3]) {
    let to_x = |x: f32| (x + 1.0) / 2.0 * w as f32;
    let to_y = |y: f32| (1.0 - y) / 2.0 * h as f32;
    let (x0, x1) = (to_x(r[0]), to_x(r[2]));
    let (y0, y1) = (to_y(r[3]), to_y(r[1]));
    let (qx, qy) = ((x1 - x0) / 4.0, (y1 - y0) / 4.0);
    let (xa, xb) = ((x0 + qx).floor() as u32, ((x1 - qx).ceil() as u32).min(w));
    let (ya, yb) = ((y0 + qy).floor() as u32, ((y1 - qy).ceil() as u32).min(h));
    let (mut sum, mut n) = ([0u64; 3], 0u64);
    let (mut lo, mut hi) = ([255u8; 3], [0u8; 3]);
    for y in ya..yb.max(ya + 1).min(h) {
        for x in xa..xb.max(xa + 1).min(w) {
            let i = (y as usize * w as usize + x as usize) * 4;
            for ch in 0..3 {
                let v = px[i + ch];
                sum[ch] += v as u64;
                (lo[ch], hi[ch]) = (lo[ch].min(v), hi[ch].max(v));
            }
            n += 1;
        }
    }
    if n == 0 {
        return ([0.0; 3], [255.0; 3]);
    }
    (std::array::from_fn(|ch| sum[ch] as f32 / n as f32), std::array::from_fn(|ch| hi[ch].saturating_sub(lo[ch]) as f32))
}

/// F⁻¹ through (`knots`, `v`) over the bytes' own linear light (so F =
/// identity is exact, not only at the knots): a cubic between knots
/// (non-uniform Catmull-Rom, its slopes from the neighbours: a tone curve
/// is smooth, and B has knots only every 34 levels), linear past the ends
/// (the strip sees the brightest codes darkened by its own vignetting;
/// nearer the middle they come out brighter). The weights of the knots at
/// `y` (the interpolation is linear in `v`).
fn weights(knots: &[f32], y: f32) -> Vec<(usize, f32)> {
    let n = knots.len();
    let j = knots.partition_point(|&k| k <= y).clamp(1, n - 1) - 1;
    let x = |i: usize| srgb_to_linear(knots[i]);
    let (x1, x2) = (x(j), x(j + 1));
    let d = x2 - x1;
    let s = (srgb_to_linear(y) - x1) / d;
    if !(0.0..=1.0).contains(&s) {
        return vec![(j, 1.0 - s), (j + 1, s)];
    }
    let (s2, s3) = (s * s, s * s * s);
    let (h00, h10, h01, h11) = (2.0 * s3 - 3.0 * s2 + 1.0, s3 - 2.0 * s2 + s, -2.0 * s3 + 3.0 * s2, s3 - s2);
    let mut w = vec![(j, h00), (j + 1, h01)];
    // The slope at x1: (v2 - v0) / (x2 - x0), or the segment's own.
    if j > 0 {
        let k = h10 * d / (x2 - x(j - 1));
        w.push((j - 1, -k));
        w[1].1 += k;
    } else {
        w[0].1 -= h10;
        w[1].1 += h10;
    }
    // At x2: (v3 - v1) / (x3 - x1), or the segment's own.
    if j + 2 < n {
        let k = h11 * d / (x(j + 2) - x1);
        w.push((j + 2, k));
        w[0].1 -= k;
    } else {
        w[0].1 -= h11;
        w[1].1 += h11;
    }
    w
}

/// The three channels' fit over cells `idx` of `all`: per channel F⁻¹'s
/// knots, one a and b.
fn fit_joint(all: &[Cell], idx: &[usize]) -> Option<Calibration> {
    // Knots per channel: the strip's observed bytes in code order, rising
    // by at least half a level (a cell that does not rise adds no knot;
    // still an equation). G falls with q and B folds, so the strip is
    // sorted; B has 8 distinct strip codes (17 .. 255 every 34).
    let mut knots: [Vec<f32>; 3] = Default::default();
    for (ch, k) in knots.iter_mut().enumerate() {
        let mut strip: Vec<Seen> = idx.iter().map(|&i| seen(&all[i], ch)).filter(|c| c.strip).collect();
        strip.sort_by(|a, b| a.code.total_cmp(&b.code).then(a.y.total_cmp(&b.y)));
        for c in &strip {
            if k.last().is_none_or(|&last| c.y > last + 0.5) {
                k.push(c.y);
            }
        }
        if k.len() < 6 || strip.last()?.y - strip.first()?.y < 64.0 {
            return None;
        }
    }
    let base = [0, knots[0].len(), knots[0].len() + knots[1].len()];
    let n = base[2] + knots[2].len() + 2;
    let (ia, ib) = (n - 2, n - 1);
    // Normal equations of: F⁻¹(y) - L(c) (a s + b s²) = L(c), s = r² - 1.
    let mut ata = vec![vec![0.0f64; n]; n];
    let mut atb = vec![0.0f64; n];
    let mut add = |row: &[(usize, f64)], rhs: f64| {
        for &(i, vi) in row {
            for &(j, vj) in row {
                ata[i][j] += vi * vj;
            }
            atb[i] += vi * rhs;
        }
    };
    for &i in idx {
        for ch in 0..3 {
            let Seen { code, y, r2, .. } = seen(&all[i], ch);
            let l = srgb_to_linear(code) as f64;
            let s = (r2 - 1.0) as f64;
            let mut row: Vec<(usize, f64)> = weights(&knots[ch], y).into_iter().map(|(k, w)| (base[ch] + k, w as f64)).collect();
            row.push((ia, -l * s));
            row.push((ib, -l * s * s));
            add(&row, l);
        }
    }
    // A little ridge on a, more on b (the cells span r 0.59-1.19 only; the
    // faces reach 1.41): without vignetting they stay 0. Without a pair
    // (the strip alone, r 0.89-1.19) the vignette is not to be told from
    // the curves: held at none.
    let pairs = idx.iter().any(|&i| !all[i].strip);
    let ridge = if pairs { [0.01, 0.1] } else { [1e4, 1e4] };
    add(&[(ia, ridge[0])], 0.0);
    add(&[(ib, ridge[1])], 0.0);
    let x = solve(ata, atb)?;
    let (a, b) = (x[ia] as f32, x[ib] as f32);
    if !a.is_finite() || !b.is_finite() {
        return None;
    }
    let mut channels: Vec<Channel> = Vec::with_capacity(3);
    for ch in 0..3 {
        let nk = knots[ch].len();
        // F⁻¹ rising, over every byte (a cubic may dip a hair between knots).
        let mut v: Vec<f32> = x[base[ch]..base[ch] + nk].iter().map(|&v| (v as f32).max(0.0)).collect();
        for i in 1..v.len() {
            v[i] = v[i].max(v[i - 1]);
        }
        let mut finv: Vec<f32> = (0..256).map(|y| weights(&knots[ch], y as f32).into_iter().map(|(i, w)| v[i] * w).sum::<f32>().max(0.0)).collect();
        for i in 1..256 {
            finv[i] = finv[i].max(finv[i - 1]);
        }
        if finv.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let mut c = Channel::with(finv, a, b, 0.0, true);
        c.residual = idx.iter().map(|&i| cell_error(&c, &seen(&all[i], ch))).fold(0.0, f32::max);
        channels.push(c);
    }
    let channels: [Channel; 3] = channels.try_into().ok()?;
    let residual = channels[0].residual.max(channels[1].residual);
    Some(Calibration { channels, residual, fitted: true, covered: 0, dropped: 0 })
}

/// A cell's error after `c` (code levels): its byte at the cell's radius
/// back to a code, against the code drawn.
fn cell_error(c: &Channel, o: &Seen) -> f32 {
    (c.code_at(o.y, o.r2) - o.code).abs()
}

/// Gaussian elimination with partial pivoting.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let p = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[p][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, p);
        b.swap(col, p);
        for row in col + 1..n {
            let f = a[row][col] / a[col][col];
            if f != 0.0 {
                for k in col..n {
                    a[row][k] -= f * a[col][k];
                }
                b[row] -= f * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let s: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    Some(x)
}

/// Draws the cells as a post-processed right eye shows them (tests, the
/// synthetic frames). `post(channel, code, r2)` gives the byte, at each
/// pixel's own radius (its vignette, and a dither that varies).
#[cfg(any(test, feature = "synth"))]
pub(crate) fn draw_cells(px: &mut [u8], w: u32, h: u32, post: &dyn Fn(usize, f32, f32) -> u8) {
    for (q, r) in cells() {
        crate::synth::fill_ndc_with(px, w, h, r, &|r2| std::array::from_fn(|ch| post(ch, e1c(q as f32, ch), r2)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eye_with(post: &dyn Fn(usize, f32, f32) -> u8) -> Vec<u8> {
        let (w, h) = (1920u32, 1920u32);
        let mut px = vec![0u8; (w * h * 4) as usize];
        draw_cells(&mut px, w, h, post);
        px
    }

    /// The pixel a cell's middle is at.
    fn at(r: Ndc) -> (usize, usize) {
        let (x, y) = ((r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0);
        (((x + 1.0) / 2.0 * 1920.0) as usize, ((1.0 - y) / 2.0 * 1920.0) as usize)
    }

    /// A vignette (PPv2's: on the light, before the grading), per channel
    /// a different tone curve, lifted blacks, a level of dither.
    fn graded(ch: usize, code: f32, r2: f32) -> u8 {
        let g = 1.0 - 0.1 * r2;
        let gamma = [0.85, 0.95, 0.9][ch];
        let l = g * srgb_to_linear(code);
        let dither = ((code * 7.3 + r2 * 131.0 + ch as f32 * 2.1).sin() * 0.6).round();
        (linear_to_srgb(0.004 + 0.96 * l.clamp(0.0, 1.0).powf(gamma)) + dither).round().clamp(0.0, 255.0) as u8
    }

    #[test]
    fn no_post_processing_is_the_identity() {
        let c = Calibration::fit(&eye_with(&|_, code, _| code.round() as u8), 1920, 1920);
        assert!(c.fitted && c.residual < 0.3 && c.covered == 0, "{c:?}");
        for ch in &c.channels {
            assert!(ch.a.abs() < 0.01 && ch.b.abs() < 0.01);
        }
        for q in [0u8, 1, 17, 64, 100, 128, 200, 254] {
            for r2 in [0.1, 1.0, 1.9] {
                let rgb = [0, 1, 2].map(|ch| e1c(q as f32, ch) as u8);
                let (back, rg, b) = c.read(rgb, r2);
                assert!((back - q as f32).abs() < 0.6 && rg < 1.1 && b < 1.1, "{q} at {r2}: {back} {rg} {b}");
            }
        }
    }

    #[test]
    fn each_channel_undoes_its_own_tone_curve_and_vignette() {
        let c = Calibration::fit(&eye_with(&graded), 1920, 1920);
        assert!(c.fitted && c.residual < 2.0 && c.covered == 0, "fitted {} covered {} residual {}", c.fitted, c.covered, c.residual);
        let mut worst = [0.0f32; 3];
        for q in (2..=252).step_by(5) {
            for r2 in [0.3, 0.6, 1.0, 1.4] {
                let rgb = [0, 1, 2].map(|ch| graded(ch, e1c(q as f32, ch), r2));
                let (back, rg, b) = c.read(rgb, r2);
                worst = [worst[0].max((back - q as f32).abs()), worst[1].max(rg), worst[2].max(b)];
            }
        }
        // The pixels' check (6 and 10, in bytes' worth: the defaults)
        // passes graded depth. Measured: q within 2.4 levels, |qR - qG| 4.3,
        // |tri - B| 7.7 (the worst near tri 0, below B's lowest strip knot,
        // 17: a toe there is extrapolated).
        assert!(worst[0] < 3.0 && worst[1] < 6.0 && worst[2] < 10.0, "{worst:?}");
    }

    #[test]
    fn cells_under_the_ui_are_left_out() {
        // The vignette and grading, and UI over three of the pairs (the
        // viewfinder's colours, measured) and an even grey over a strip cell.
        let mut px = eye_with(&graded);
        let cs = cells();
        let mut paint = |k: usize, colour: [u8; 3]| {
            let (cx, cy) = at(cs[k].1);
            for y in cy - 12..cy + 12 {
                for x in cx - 12..cx + 12 {
                    px[(y * 1920 + x) * 4..(y * 1920 + x) * 4 + 3].copy_from_slice(&colour);
                }
            }
        };
        for (k, colour) in [(18usize, [136u8, 124, 111]), (19, [141, 129, 114]), (20, [21, 31, 16]), (21, [120, 100, 84]), (22, [51, 43, 33]), (23, [45, 39, 30])] {
            paint(k, colour);
        }
        paint(9, [60; 3]);
        let c = Calibration::fit(&px, 1920, 1920);
        assert!(c.fitted && c.covered == 7 && c.residual < 2.0, "fitted {} covered {} residual {}", c.fitted, c.covered, c.residual);
        for q in [40.0f32, 120.0, 200.0] {
            for r2 in [0.4, 1.0] {
                let rgb = [0, 1, 2].map(|ch| graded(ch, e1c(q, ch), r2));
                let (back, rg, _) = c.read(rgb, r2);
                assert!((back - q).abs() < 3.0 && rg < 4.0, "{q} at r² {r2}: {back} {rg}");
            }
        }
    }

    #[test]
    fn a_strip_that_does_not_rise_is_not_fitted() {
        let c = Calibration::fit(&eye_with(&|_, _, _| 0), 1920, 1920);
        assert!(!c.fitted);
    }
}
