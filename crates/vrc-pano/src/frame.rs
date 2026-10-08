//! A pano frame decoded: six views, each a perspective camera at the
//! code's position with its colour (the left eye's block, copied as is),
//! its metric depth (the right eye's, mapped back through the calibration
//! and decoded) and where it looks; and the geometry on them: pixel to ray
//! (rig, world, bearing from the head), pixel to world point.
//!
//! Depth is z along the face camera's axis, not the ray's length: a point
//! is `position + turn(R_face · z (x, y, 1), rig_yaw)`.
//!
//! The right eye is E1C (`pano-depth-check-spec.md`): R = q, G = 255 - q,
//! B = |2q - 255|. VRChat's UI drawn over the HUD (nameplates, the user
//! camera's viewfinder) breaks that relation, so a pixel whose channels,
//! mapped back, do not tell one q is the UI: no depth (`MASK_OVERLAY`,
//! grown a little), and the left eye's colour there and up to the UI's
//! stereo disparity to its right is the UI too (`MASK_COLOUR`).

use std::time::Instant;

use anyhow::{bail, Result};
use rayon::prelude::*;
use vrc_vr::tap::{format, EyeFrame};

use crate::calib::{check_scale, Calibration};
use crate::code::{classify, HeadPose, PanoCode, Route, Seen};
use crate::layout::{self, Face};

/// Mask bits: the bot's own body (the down face, too near).
pub const MASK_BODY: u8 = 1;
/// A depth edge (a jump too big within 3 x 3): bloom and the eyes'
/// anti-aliasing smear edges, so its depth is dropped.
pub const MASK_EDGE: u8 = 2;
/// VRChat's UI over the depth (the E1C check failed here or within
/// `overlay_margin`): no depth.
pub const MASK_OVERLAY: u8 = 4;
/// VRChat's UI over the colour (the left eye: the right eye's UI shifted
/// by up to its disparity; nameplate boxes found by OCR): no colour.
pub const MASK_COLOUR: u8 = 8;

#[derive(Clone, Debug)]
pub struct PanoParams {
    /// In the down face, depth nearer than this (metres) is the bot's own
    /// body (D2 shows it; D1 should not).
    pub body_near_m: f32,
    /// Within a pixel's 3 x 3, the far over the near (no depth: farther
    /// than anything) above this: an edge, the 3 x 3 median taken...
    pub edge_soft: f32,
    /// ... above this: dropped.
    pub edge_drop: f32,
    /// The calibration's residual (code levels) above which the depth only
    /// orders near and far (`PanoFrame::depth_ordinal`).
    pub max_residual: f32,
    /// The E1C check, after the calibration: |qR - qG| and |tri(qR) - B|
    /// at most these, or the pixel is the UI. In levels without grading;
    /// where a curve compresses a channel, in what a byte is worth there
    /// (`calib::check_scale`: a byte of dither is more levels).
    pub check_rg: f32,
    pub check_b: f32,
    /// The UI's mask grown by this (pixels at 1920; anti-aliased borders).
    pub overlay_margin: u32,
    /// The UI's largest stereo disparity (pixels at 1920): fx 0.063 m over
    /// 1 m, the nearest a nameplate is drawn.
    pub overlay_disparity: u32,
}

impl Default for PanoParams {
    fn default() -> Self {
        PanoParams {
            body_near_m: 0.5,
            // edge_soft: 3 levels (dither is 1); edge_drop: a quarter.
            edge_soft: 1.07,
            edge_drop: 1.25,
            max_residual: 3.0,
            // The spec's starting point was 4 and 6; a graded world (a toe,
            // per-channel curves, a byte of dither) reaches about 5 and 9
            // on the synthetic frames, while the UI's colours miss by tens
            // of levels or more. To be tuned on real E1C frames
            // (`PanoFrame::check`).
            check_rg: 6.0,
            check_b: 10.0,
            overlay_margin: 2,
            overlay_disparity: 50,
        }
    }
}

/// One face as decoded.
#[derive(Clone, Debug)]
pub struct PanoView {
    pub face: Face,
    pub width: u32,
    pub height: u32,
    /// RGB8, rows top first (the left eye's block; `MASK_COLOUR`: the UI).
    pub rgb: Vec<u8>,
    /// Metres along the camera's axis; NaN: none (nothing drawn, masked,
    /// an edge, no depth route).
    pub depth: Vec<f32>,
    /// `MASK_*` bits.
    pub mask: Vec<u8>,
    /// fx, fy, cx, cy (pixels, y down; `Fov::intrinsics`' order): fx = fy
    /// in a square eye.
    pub intrinsics: [f32; 4],
    /// Rig <- camera (`Face::rotation`).
    pub rotation: [[f32; 3]; 3],
    /// Where the block is in the eye: x, y, width, height.
    pub block: [u32; 4],
}

impl PanoView {
    /// The camera's (x, y, 1) at a point of the image (pixel `i`'s middle
    /// is `i + 0.5`).
    pub fn camera_dir(&self, u: f32, v: f32) -> [f32; 3] {
        let [fx, fy, cx, cy] = self.intrinsics;
        [(u - cx) / fx, (cy - v) / fy, 1.0]
    }

    /// The rig's direction there (camera axis component 1, not unit).
    pub fn rig_dir(&self, u: f32, v: f32) -> [f32; 3] {
        layout::mul(&self.rotation, self.camera_dir(u, v))
    }

    /// Where a rig direction lands, `(u, v, cos)` (cos: to the axis, how
    /// centrally it is seen); `None` behind or outside.
    pub fn project(&self, rig: [f32; 3]) -> Option<(f32, f32, f32)> {
        let c = layout::mul_t(&self.rotation, rig);
        if c[2] <= 1e-6 {
            return None;
        }
        let [fx, fy, cx, cy] = self.intrinsics;
        let (u, v) = (cx + fx * c[0] / c[2], cy - fy * c[1] / c[2]);
        if !(0.0..self.width as f32).contains(&u) || !(0.0..self.height as f32).contains(&v) {
            return None;
        }
        Some((u, v, c[2] / (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt()))
    }

    pub fn depth_at(&self, i: u32, j: u32) -> Option<f32> {
        if i >= self.width || j >= self.height {
            return None;
        }
        let z = self.depth[(j * self.width + i) as usize];
        z.is_finite().then_some(z)
    }

    pub fn rgb_at(&self, i: u32, j: u32) -> [u8; 3] {
        let k = (j * self.width + i) as usize * 3;
        [self.rgb[k], self.rgb[k + 1], self.rgb[k + 2]]
    }

    /// The colour there, unless it is the UI's.
    pub fn colour_at(&self, i: u32, j: u32) -> Option<[u8; 3]> {
        (self.mask[(j * self.width + i) as usize] & MASK_COLOUR == 0).then(|| self.rgb_at(i, j))
    }
}

/// A ray through a pixel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    /// Unit, the rig's frame.
    pub rig: [f32; 3],
    /// Unit, Unity's world.
    pub world: [f32; 3],
    /// From where the head looks (beacon), + right, degrees (-180, 180].
    pub bearing: f32,
    /// + up, degrees.
    pub elevation: f32,
}

/// A point of the depth, for checking and mapping.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PanoPoint {
    /// Unity's world (metres).
    pub world: [f32; 3],
    /// None where the colour is the UI's.
    pub rgb: Option<[u8; 3]>,
    pub face: Face,
}

impl PanoPoint {
    /// In the bot's map axes: x, y, -z.
    pub fn map(&self) -> [f32; 3] {
        to_map(self.world)
    }
}

/// Unity's world to the bot's map axes (x, y, -z).
pub fn to_map(p: [f32; 3]) -> [f32; 3] {
    [p[0], p[1], -p[2]]
}

/// How well the E1C check held on the depth (pixels not the UI): the
/// 50th and 99th percentiles of |qR - qG| and |tri(qR) - B| (levels), for
/// tuning `check_rg` and `check_b` on real frames.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CheckStats {
    pub rg: [f32; 2],
    pub b: [f32; 2],
    /// The fraction of the faces' pixels found to be the UI.
    pub overlay: f32,
}

#[derive(Clone, Debug)]
pub struct PanoFrame {
    pub code: PanoCode,
    pub head: HeadPose,
    /// In `Face::ALL`'s order.
    pub views: Vec<PanoView>,
    pub calibration: Calibration,
    /// The calibration left more than `max_residual` levels: the depth
    /// orders near and far, its metres are not to be trusted.
    pub depth_ordinal: bool,
    pub check: CheckStats,
    /// The tap's seq and capture time of the frame.
    pub tap_seq: u64,
    pub capture_ns: i64,
    pub eye_size: [u32; 2],
    /// Not 1920²: the faces were resampled into their blocks (detail lost).
    pub scaled: bool,
    pub decode_ms: f32,
}

/// Decodes a tapped frame; an error says why it is not a pano frame.
pub fn decode(frame: &EyeFrame, params: &PanoParams) -> Result<PanoFrame> {
    match classify(frame) {
        Seen::Pano { code, head } => decode_as(frame, code, head, params),
        Seen::Unusable(why) => bail!("not a usable pano frame: {why}"),
        Seen::Normal => {
            bail!("not a pano frame (no 0x5B code: the rig is off, or the avatar has none)")
        }
    }
}

/// Decodes a frame [`classify`] already found to be a panorama.
pub fn decode_as(frame: &EyeFrame, code: PanoCode, head: HeadPose, params: &PanoParams) -> Result<PanoFrame> {
    let t0 = Instant::now();
    let bgr = match frame.format {
        format::R8G8B8A8_UNORM | format::R8G8B8A8_SRGB => false,
        format::B8G8R8A8_UNORM | format::B8G8R8A8_SRGB => true,
        other => bail!("no pano decoding for VkFormat {other}"),
    };
    let calibration = if code.route == Route::None { Calibration::identity() } else { Calibration::fit(&rgb_order(frame.eye(1), bgr), frame.width, frame.height) };
    let depth_ordinal = code.route != Route::None && calibration.residual > params.max_residual;
    let tables = Tables::new(&calibration, &code);
    let scale = frame.width as f32 / layout::EYE as f32;
    let margin = (params.overlay_margin as f32 * scale).round() as usize;
    let decoded: Vec<(PanoView, Hist)> = Face::ALL.par_iter().map(|&face| decode_view(frame, face, &code, &tables, params, bgr, margin)).collect();
    let mut hist = Hist::default();
    let mut views = Vec::with_capacity(6);
    for (v, h) in decoded {
        hist.add(&h);
        views.push(v);
    }
    let disparity = (params.overlay_disparity as f32 * scale).round() as usize;
    mask_colour(&mut views, frame.width as usize, frame.height as usize, disparity);
    let all: usize = views.iter().map(|v| v.mask.len()).sum();
    let check = CheckStats { rg: [hist.rg.pct(0.5), hist.rg.pct(0.99)], b: [hist.b.pct(0.5), hist.b.pct(0.99)], overlay: hist.overlay as f32 / all.max(1) as f32 };
    Ok(PanoFrame {
        code,
        head,
        views,
        calibration,
        depth_ordinal,
        check,
        tap_seq: frame.seq,
        capture_ns: frame.capture_ns,
        eye_size: [frame.width, frame.height],
        scaled: frame.width != layout::EYE || frame.height != layout::EYE,
        decode_ms: t0.elapsed().as_secs_f32() * 1e3,
    })
}

/// The calibration reads RGB: a BGR eye's bytes swapped (the cells only
/// are read, but the copy is cheap next to the decoding).
fn rgb_order(px: &[u8], bgr: bool) -> std::borrow::Cow<'_, [u8]> {
    if !bgr {
        return std::borrow::Cow::Borrowed(px);
    }
    let mut v = px.to_vec();
    v.par_chunks_exact_mut(4).for_each(|p| p.swap(0, 2));
    std::borrow::Cow::Owned(v)
}

/// No depth, while filtering: farther than anything.
const FAR: f32 = 2.0;
/// The code tables' rings of NDC radius² (0..2, the eye's corners).
const RINGS_PER_R2: f32 = 64.0;
const RINGS: usize = 129;
/// Entries of the log depth -> metres table.
const EXPS: usize = 4096;
/// The check's histograms: bins a quarter level, up to 32 levels.
const BINS: usize = 128;
const BIN: f32 = 0.25;

/// A frame's lookups: per channel the code a byte stands for in each ring
/// (the calibration's F⁻¹ and vignetting), and metres of the log depth.
struct Tables {
    /// `[channel][ring * 256 + byte]`: the code (0..255, fractional) and
    /// the levels a byte is worth there (`Channel::slope`).
    codes: [Vec<[f32; 2]>; 3],
    /// Metres at t = k / EXPS.
    metres: Vec<f32>,
    /// Every ring within a quarter level of the first (no vignetting to
    /// speak of: the dither alone is a level): ring 0 alone.
    flat: bool,
}

impl Tables {
    fn new(calib: &Calibration, code: &PanoCode) -> Tables {
        // Per ring: each byte's code, and what a byte is worth there (the
        // codes either side, as `Channel::slope`).
        let codes: [Vec<[f32; 2]>; 3] = std::array::from_fn(|ch| {
            let c = &calib.channels[ch];
            let mut out = vec![[0.0f32; 2]; RINGS * 256];
            out.par_chunks_mut(256).enumerate().for_each(|(ring, row)| {
                let r2 = (ring as f32 + 0.5) / RINGS_PER_R2;
                let code: Vec<f32> = (0..256).map(|b| c.code(b as u8, r2)).collect();
                for (b, o) in row.iter_mut().enumerate() {
                    let (lo, hi) = (b.saturating_sub(1), (b + 1).min(255));
                    *o = [code[b], ((code[hi] - code[lo]) / (hi - lo) as f32).abs().max(0.25)];
                }
            });
            out
        });
        let ln_range = (code.zmax / code.zmin).ln();
        let metres = (0..=EXPS).map(|k| code.zmin * (k as f32 / EXPS as f32 * ln_range).exp()).collect();
        let flat = codes.iter().all(|c| (1..RINGS).all(|ring| (0..256).all(|b| (c[ring * 256 + b][0] - c[b][0]).abs() < 0.25)));
        Tables { codes, metres, flat }
    }

    /// Metres at log depth `t` (0..1), interpolated.
    #[inline]
    fn metres(&self, t: f32) -> f32 {
        let x = t * EXPS as f32;
        let i = (x as usize).min(EXPS - 1);
        let f = x - i as f32;
        self.metres[i] + (self.metres[i + 1] - self.metres[i]) * f
    }
}

/// A histogram of check residuals.
#[derive(Clone)]
struct Bins([u32; BINS]);

impl Default for Bins {
    fn default() -> Self {
        Bins([0; BINS])
    }
}

impl Bins {
    fn put(&mut self, v: f32) {
        self.0[((v / BIN) as usize).min(BINS - 1)] += 1;
    }

    /// The `p` quantile (levels; the bin's upper edge).
    fn pct(&self, p: f32) -> f32 {
        let n: u32 = self.0.iter().sum();
        if n == 0 {
            return 0.0;
        }
        let want = (p * n as f32).ceil() as u32;
        let mut seen = 0;
        for (k, &c) in self.0.iter().enumerate() {
            seen += c;
            if seen >= want {
                return (k + 1) as f32 * BIN;
            }
        }
        BINS as f32 * BIN
    }
}

/// The check's residuals over the depth (pixels that passed).
#[derive(Clone, Default)]
struct Hist {
    rg: Bins,
    b: Bins,
    /// Pixels found to be the UI (grown).
    overlay: usize,
}

impl Hist {
    fn add(&mut self, o: &Hist) {
        self.overlay += o.overlay;
        for k in 0..BINS {
            self.rg.0[k] += o.rg.0[k];
            self.b.0[k] += o.b.0[k];
        }
    }
}

fn decode_view(frame: &EyeFrame, face: Face, code: &PanoCode, tables: &Tables, params: &PanoParams, bgr: bool, margin: usize) -> (PanoView, Hist) {
    let (w, h) = (frame.width as usize, frame.height as usize);
    let block = face.block(frame.width, frame.height);
    let [bx, by, bw, bh] = block.map(|v| v as usize);
    let (left, right) = (frame.eye(0), frame.eye(1));
    let ch = if bgr { [2, 1, 0] } else { [0, 1, 2] };
    let mut rgb = vec![0u8; bw * bh * 3];
    rgb.par_chunks_mut(bw * 3).enumerate().for_each(|(j, out)| {
        let src = &left[((by + j) * w + bx) * 4..((by + j) * w + bx + bw) * 4];
        for (o, p) in out.chunks_exact_mut(3).zip(src.chunks_exact(4)) {
            (o[0], o[1], o[2]) = (p[ch[0]], p[ch[1]], p[ch[2]]);
        }
    });
    let intrinsics = face.intrinsics(bw as u32, bh as u32);
    let view = |depth: Vec<f32>, mask: Vec<u8>| PanoView { face, width: bw as u32, height: bh as u32, rgb: rgb.clone(), depth, mask, intrinsics, rotation: face.rotation(), block };
    if code.route == Route::None {
        return (view(vec![f32::NAN; bw * bh], vec![0; bw * bh]), Hist::default());
    }
    // Per pixel: the log depth (0..1; FAR none), and whether the check
    // failed (the UI); the residuals of those that passed.
    let mut t = vec![FAR; bw * bh];
    let mut bad = vec![false; bw * bh];
    let hist = t
        .par_chunks_mut(bw)
        .zip(bad.par_chunks_mut(bw))
        .enumerate()
        .map(|(j, (trow, brow))| {
            let mut hist = Hist::default();
            let y = by + j;
            let ny = 1.0 - (y as f32 + 0.5) / h as f32 * 2.0;
            let dx = 2.0 / w as f32;
            let src = &right[(y * w + bx) * 4..(y * w + bx + bw) * 4];
            for (i, p) in src.chunks_exact(4).enumerate() {
                let ring = if tables.flat {
                    0
                } else {
                    let nx = (bx + i) as f32 * dx + dx / 2.0 - 1.0;
                    (((nx * nx + ny * ny) * RINGS_PER_R2) as usize).min(RINGS - 1) * 256
                };
                let [qr, sr] = tables.codes[0][ring + p[ch[0]] as usize];
                let [qg, sg] = tables.codes[1][ring + p[ch[1]] as usize];
                let [qb, sb] = tables.codes[2][ring + p[ch[2]] as usize];
                let qg = 255.0 - qg;
                let (scale_rg, scale_b) = check_scale([sr, sg, sb]);
                let (rg, b) = ((qr - qg).abs(), ((2.0 * qr - 255.0).abs() - qb).abs());
                if rg > params.check_rg * scale_rg || b > params.check_b * scale_b {
                    brow[i] = true;
                    continue;
                }
                if i % 4 == 0 {
                    hist.rg.put(rg / scale_rg);
                    hist.b.put(b / scale_b);
                }
                let q = (qr + qg) / 2.0;
                // q 0 (nearer than the range) is pure cyan, a colour of
                // the UI (a ring): unknown, as the UI.
                if q < 0.5 {
                    brow[i] = true;
                    continue;
                }
                if q < 254.5 {
                    trow[i] = (q / 254.0).clamp(0.0, 1.0);
                }
            }
            hist
        })
        .reduce(Hist::default, |mut a, b| {
            a.add(&b);
            a
        });
    // The UI grown by the margin: no depth, and its neighbours see "none"
    // (an edge there is dropped, not smeared).
    let over = grow(&bad, bw, bh, margin);
    let mut hist = hist;
    hist.overlay = over.par_iter().filter(|&&o| o).count();
    for (tv, &o) in t.iter_mut().zip(&over) {
        if o {
            *tv = FAR;
        }
    }
    let ln_range = (code.zmax / code.zmin).ln();
    let (soft, drop) = (params.edge_soft.ln() / ln_range, params.edge_drop.ln() / ln_range);
    let body = if face == Face::Down { params.body_near_m } else { 0.0 };
    // Each pixel's nearest and farthest of its 3 across (the block's edge
    // repeats), then of the 3 such down: the 3 x 3's.
    let mut lo3 = vec![0.0f32; bw * bh];
    let mut hi3 = vec![0.0f32; bw * bh];
    lo3.par_chunks_mut(bw).zip(hi3.par_chunks_mut(bw)).enumerate().for_each(|(j, (l, u))| {
        let row = &t[j * bw..(j + 1) * bw];
        for i in 0..bw {
            let (a, b, c) = (row[i.saturating_sub(1)], row[i], row[(i + 1).min(bw - 1)]);
            (l[i], u[i]) = (a.min(b).min(c), a.max(b).max(c));
        }
    });
    let mut depth = vec![f32::NAN; bw * bh];
    let mut mask = vec![0u8; bw * bh];
    depth.par_chunks_mut(bw).zip(mask.par_chunks_mut(bw)).enumerate().for_each(|(j, (drow, mrow))| {
        let rows = [j.saturating_sub(1), j, (j + 1).min(bh - 1)].map(|jj| jj * bw);
        for i in 0..bw {
            let k = rows[1] + i;
            if over[k] {
                mrow[i] |= MASK_OVERLAY;
                continue;
            }
            let tk = t[k];
            if tk >= FAR {
                continue;
            }
            let lo = lo3[rows[0] + i].min(lo3[k]).min(lo3[rows[2] + i]);
            let hi = hi3[rows[0] + i].max(hi3[k]).max(hi3[rows[2] + i]);
            let jump = hi - lo;
            let z = if jump <= soft {
                tables.metres(tk)
            } else if jump > drop {
                mrow[i] |= MASK_EDGE;
                continue;
            } else {
                // Rare: the median of the 3 x 3.
                let mut win = [FAR; 9];
                let mut n = 0;
                for row in rows {
                    for ii in i.saturating_sub(1)..=(i + 1).min(bw - 1) {
                        win[n] = t[row + ii];
                        n += 1;
                    }
                }
                let win = &mut win[..n];
                win.sort_unstable_by(f32::total_cmp);
                if win[n / 2] >= FAR {
                    mrow[i] |= MASK_EDGE;
                    continue;
                }
                tables.metres(win[n / 2])
            };
            if z < body {
                mrow[i] |= MASK_BODY;
                continue;
            }
            drow[i] = z;
        }
    });
    (view(depth, mask), hist)
}

/// `m` (`w` x `h`, rows) grown by `margin` cells every way (a box).
fn grow(m: &[bool], w: usize, h: usize, margin: usize) -> Vec<bool> {
    if margin == 0 || !m.iter().any(|&v| v) {
        return m.to_vec();
    }
    let mut a = m.to_vec();
    rows_grow(&mut a, w, margin);
    let mut t = transpose(&a, w, h);
    rows_grow(&mut t, h, margin);
    transpose(&t, h, w)
}

/// Each row (`w` long) grown by `margin` cells each way.
fn rows_grow(m: &mut [bool], w: usize, margin: usize) {
    m.par_chunks_mut(w).for_each_init(Vec::new, |prefix: &mut Vec<u32>, row| {
        prefix.clear();
        prefix.push(0);
        for &v in row.iter() {
            prefix.push(prefix.last().unwrap() + v as u32);
        }
        let n = row.len();
        for (k, o) in row.iter_mut().enumerate() {
            *o = prefix[(k + margin + 1).min(n)] > prefix[k.saturating_sub(margin)];
        }
    });
}

/// `w` x `h` (rows) to `h` x `w`.
fn transpose(m: &[bool], w: usize, h: usize) -> Vec<bool> {
    let mut t = vec![false; w * h];
    t.par_chunks_mut(h).enumerate().for_each(|(i, col)| {
        for (j, o) in col.iter_mut().enumerate() {
            *o = m[j * w + i];
        }
    });
    t
}

/// The left eye's UI: VRChat draws it in both eyes with a stereo
/// disparity, so the left eye's copy lies 0..`disparity` pixels right of
/// the right eye's (measured: a plate about 48 px at 1.1 m). Every view's
/// colour within that of an overlay pixel (in eye coordinates: the UI
/// crosses blocks) is masked.
fn mask_colour(views: &mut [PanoView], w: usize, h: usize, disparity: usize) {
    if !views.par_iter().any(|v| v.mask.iter().any(|m| m & MASK_OVERLAY != 0)) {
        return;
    }
    let mut eye = vec![false; w * h];
    for v in views.iter() {
        let [bx, by, bw, _] = v.block.map(|x| x as usize);
        for (k, m) in v.mask.iter().enumerate() {
            if m & MASK_OVERLAY != 0 {
                eye[(by + k / bw) * w + bx + k % bw] = true;
            }
        }
    }
    // Per eye row: overlay pixels counted, prefix sums.
    for v in views.iter_mut() {
        let [bx, by, bw, _] = v.block.map(|x| x as usize);
        v.mask.par_chunks_mut(bw).enumerate().for_each(|(j, mrow)| {
            let row = &eye[(by + j) * w..(by + j + 1) * w];
            let start = bx.saturating_sub(disparity);
            let mut prefix = Vec::with_capacity(bx + bw - start + 1);
            prefix.push(0u32);
            for &o in &row[start..bx + bw] {
                prefix.push(prefix.last().unwrap() + o as u32);
            }
            for (i, m) in mrow.iter_mut().enumerate() {
                // Overlay within [x - disparity, x] (eye x = bx + i).
                let x = bx + i - start;
                if prefix[x + 1] > prefix[x.saturating_sub(disparity)] {
                    *m |= MASK_COLOUR;
                }
            }
        });
    }
}

/// Degrees into (-180, 180].
pub fn wrap(deg: f32) -> f32 {
    let d = deg.rem_euclid(360.0);
    if d > 180.0 {
        d - 360.0
    } else {
        d
    }
}

impl PanoFrame {
    pub fn view(&self, face: Face) -> &PanoView {
        &self.views[Face::ALL.iter().position(|&f| f == face).unwrap()]
    }

    /// A rig direction in Unity's world.
    pub fn rig_to_world(&self, d: [f32; 3]) -> [f32; 3] {
        layout::turn(d, self.code.rig_yaw)
    }

    pub fn world_to_rig(&self, d: [f32; 3]) -> [f32; 3] {
        layout::turn(d, -self.code.rig_yaw)
    }

    /// A world direction's bearing from where the head looks (+ right,
    /// degrees, (-180, 180]) and elevation (+ up).
    pub fn bearing(&self, world: [f32; 3]) -> (f32, f32) {
        let yaw = world[0].atan2(world[2]).to_degrees();
        (wrap(yaw - self.head.yaw), world[1].atan2(world[0].hypot(world[2])).to_degrees())
    }

    /// The ray through point (`u`, `v`) of view `view` (pixel `i`'s middle
    /// is `i + 0.5`).
    pub fn ray(&self, view: usize, u: f32, v: f32) -> Ray {
        let d = self.views[view].rig_dir(u, v);
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let rig = d.map(|c| c / n);
        let world = self.rig_to_world(rig);
        let (bearing, elevation) = self.bearing(world);
        Ray { rig, world, bearing, elevation }
    }

    /// Pixel (`i`, `j`) of view `view` in Unity's world, by its depth.
    pub fn point(&self, view: usize, i: u32, j: u32) -> Option<[f32; 3]> {
        let v = &self.views[view];
        let z = v.depth_at(i, j)?;
        let d = self.rig_to_world(v.rig_dir(i as f32 + 0.5, j as f32 + 0.5));
        let p = self.code.position;
        Some([p[0] + z * d[0], p[1] + z * d[1], p[2] + z * d[2]])
    }

    /// The same in the bot's map axes (x, y, -z).
    pub fn map_point(&self, view: usize, i: u32, j: u32) -> Option<[f32; 3]> {
        self.point(view, i, j).map(to_map)
    }

    /// Every `step`-th pixel (both ways) of every view with depth.
    pub fn points(&self, step: u32) -> Vec<PanoPoint> {
        let step = step.max(1);
        let mut out = Vec::new();
        for (k, v) in self.views.iter().enumerate() {
            for j in (step / 2..v.height).step_by(step as usize) {
                for i in (step / 2..v.width).step_by(step as usize) {
                    if let Some(world) = self.point(k, i, j) {
                        out.push(PanoPoint { world, rgb: v.colour_at(i, j), face: v.face });
                    }
                }
            }
        }
        out
    }

    /// The view seeing a world direction most centrally: (view, u, v).
    pub fn project(&self, world: [f32; 3]) -> Option<(usize, f32, f32)> {
        let rig = self.world_to_rig(world);
        self.views
            .iter()
            .enumerate()
            .filter_map(|(k, v)| v.project(rig).map(|(u, w, c)| (k, u, w, c)))
            .max_by(|a, b| a.3.total_cmp(&b.3))
            .map(|(k, u, w, _)| (k, u, w))
    }

    /// The colour in an eye rectangle (`x0`, `y0`) .. (`x1`, `y1`) marked
    /// the UI's (`MASK_COLOUR`): a nameplate's pill found by OCR in the
    /// left eye.
    pub fn mask_colour(&mut self, x0: u32, y0: u32, x1: u32, y1: u32) {
        for v in &mut self.views {
            let [bx, by, bw, bh] = v.block;
            let (i0, i1) = (x0.max(bx), x1.min(bx + bw));
            let (j0, j1) = (y0.max(by), y1.min(by + bh));
            for j in j0..j1 {
                for i in i0..i1 {
                    v.mask[((j - by) * bw + i - bx) as usize] |= MASK_COLOUR;
                }
            }
        }
    }

    /// Which view an eye pixel (`x`, `y`) is in, and where: (view, i, j).
    pub fn view_at(&self, x: u32, y: u32) -> Option<(usize, u32, u32)> {
        self.views.iter().enumerate().find_map(|(k, v)| {
            let [bx, by, bw, bh] = v.block;
            (x >= bx && x < bx + bw && y >= by && y < by + bh).then(|| (k, x - bx, y - by))
        })
    }
}
