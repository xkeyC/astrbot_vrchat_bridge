//! Mirrors, found by moving: stereo sees the world *in* a mirror (it reads
//! as open space behind the wall), so the bot holds its arms out and looks
//! for what changes with them.
//!
//! Three frames per view at the same head pose: arms down (`rest`), arms
//! out (`out`), arms down again (`back`). A block of the image is mirror if
//! it changed rest -> out and out -> back but not rest -> back (things that
//! change by themselves, like video screens, change rest -> back too). The
//! arms stay out of a forward view, so only reflections of them change.

use vrc_stereo::{Disparity, Stereo};
use vrc_vr::tap::{format, EyeFrame};

/// Image blocks (left eye) that are mirror.
#[derive(Clone, Debug)]
pub struct MirrorMask {
    /// Block side, pixels of the frame.
    pub block: usize,
    pub cols: usize,
    pub rows: usize,
    pub mirror: Vec<bool>,
}

impl MirrorMask {
    /// Whether frame pixel (x, y) is mirror.
    pub fn at(&self, x: f32, y: f32) -> bool {
        let (c, r) = ((x as usize) / self.block, (y as usize) / self.block);
        c < self.cols && r < self.rows && self.mirror[r * self.cols + c]
    }

    pub fn count(&self) -> usize {
        self.mirror.iter().filter(|&&m| m).count()
    }
}

/// Mean luma per `block` x `block` block of the left eye.
fn block_luma(f: &EyeFrame, block: usize) -> (usize, usize, Vec<f32>) {
    let (w, h) = (f.width as usize, f.height as usize);
    let (cols, rows) = (w / block, h / block);
    let bgr = matches!(f.format, format::B8G8R8A8_UNORM | format::B8G8R8A8_SRGB);
    let eye = f.eye(0);
    let mut out = vec![0f32; cols * rows];
    for r in 0..rows {
        for c in 0..cols {
            let mut sum = 0u32;
            for y in r * block..(r + 1) * block {
                for x in c * block..(c + 1) * block {
                    let p = &eye[(y * w + x) * 4..(y * w + x) * 4 + 4];
                    let (rr, bb) = if bgr { (p[2], p[0]) } else { (p[0], p[2]) };
                    sum += 54 * rr as u32 + 183 * p[1] as u32 + 19 * bb as u32;
                }
            }
            out[r * cols + c] = sum as f32 / (256 * block * block) as f32;
        }
    }
    (cols, rows, out)
}

/// Mirror blocks from the three frames of one view. `threshold` is the
/// change in mean luma (0..255) that counts; groups smaller than
/// `min_blocks` are dropped.
pub fn find(rest: &EyeFrame, out: &EyeFrame, back: &EyeFrame, block: usize, threshold: f32, min_blocks: usize) -> MirrorMask {
    let (cols, rows, a) = block_luma(rest, block);
    let (_, _, b) = block_luma(out, block);
    let (_, _, c) = block_luma(back, block);
    let changed = |x: &[f32], y: &[f32], i: usize| (x[i] - y[i]).abs() > threshold;
    let raw: Vec<bool> =
        (0..cols * rows).map(|i| changed(&a, &b, i) && changed(&b, &c, i) && !changed(&a, &c, i)).collect();
    // Keep groups of at least `min_blocks` (8-connected).
    let mut mirror = vec![false; cols * rows];
    let mut seen = vec![false; cols * rows];
    for s in 0..cols * rows {
        if !raw[s] || seen[s] {
            continue;
        }
        let mut group = vec![s];
        seen[s] = true;
        let mut k = 0;
        while k < group.len() {
            let i = group[k];
            k += 1;
            let (r, c) = ((i / cols) as isize, (i % cols) as isize);
            for dr in -1..=1 {
                for dc in -1..=1 {
                    let (rr, cc) = (r + dr, c + dc);
                    if rr >= 0 && cc >= 0 && (rr as usize) < rows && (cc as usize) < cols {
                        let j = rr as usize * cols + cc as usize;
                        if raw[j] && !seen[j] {
                            seen[j] = true;
                            group.push(j);
                        }
                    }
                }
            }
        }
        if group.len() >= min_blocks {
            for i in group {
                mirror[i] = true;
            }
        }
    }
    // A reflection shows only part of the mirror (the arms): fill each row
    // between its leftmost and rightmost mirror block, and each column the
    // same, so the whole glass between them counts.
    let mut filled = mirror.clone();
    for r in 0..rows {
        let row: Vec<usize> = (0..cols).filter(|&c| mirror[r * cols + c]).collect();
        if let (Some(&c0), Some(&c1)) = (row.first(), row.last()) {
            (c0..=c1).for_each(|c| filled[r * cols + c] = true);
        }
    }
    MirrorMask { block, cols, rows, mirror: filled }
}

/// Points standing for the glass of a mirror: every mirror pixel (every
/// `step`th) placed at the distance of the frame around it, the median
/// depth of the non-mirror pixels just outside the mask. Empty without a
/// mirror or a frame to measure.
pub fn surface(mask: &MirrorMask, stereo: &Stereo, disp: &Disparity, frame_width: u32, step: usize) -> Vec<[f32; 3]> {
    if mask.count() == 0 {
        return Vec::new();
    }
    let scale = frame_width as f32 / disp.width as f32;
    // Disparities on the frame: non-mirror pixels next to mirror blocks.
    let mut border = Vec::new();
    for r in 0..mask.rows {
        for c in 0..mask.cols {
            if !mask.mirror[r * mask.cols + c] {
                continue;
            }
            for (dr, dc) in [(-1isize, 0isize), (1, 0), (0, -1), (0, 1)] {
                let (rr, cc) = (r as isize + dr, c as isize + dc);
                if rr < 0 || cc < 0 || rr as usize >= mask.rows || cc as usize >= mask.cols {
                    continue;
                }
                if mask.mirror[rr as usize * mask.cols + cc as usize] {
                    continue;
                }
                let (x, y) = ((cc as usize * mask.block + mask.block / 2) as f32 / scale, (rr as usize * mask.block + mask.block / 2) as f32 / scale);
                let d = disp.at(x as usize, y as usize);
                if d.is_finite() && d > 0.25 {
                    border.push(d);
                }
            }
        }
    }
    if border.len() < 4 {
        return Vec::new();
    }
    border.sort_by(f32::total_cmp);
    let d = border[border.len() / 2];
    let mut out = Vec::new();
    for y in (0..disp.height).step_by(step.max(1)) {
        for x in (0..disp.width).step_by(step.max(1)) {
            let (fx, fy) = (x as f32 * scale, y as f32 * scale);
            if mask.at(fx, fy) {
                out.push(stereo.point(x as f32 + 0.5, y as f32 + 0.5, d));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use vrc_vr::tap::EyeView;

    fn frame(fill: impl Fn(usize, usize) -> u8) -> EyeFrame {
        let (w, h) = (64usize, 64usize);
        let mut pixels = vec![0u8; w * h * 4 * 2];
        for y in 0..h {
            for x in 0..w {
                let v = fill(x, y);
                pixels[(y * w + x) * 4..(y * w + x) * 4 + 4].copy_from_slice(&[v, v, v, 255]);
            }
        }
        EyeFrame {
            seq: 2,
            frame_id: 1,
            display_time_ns: 0,
            capture_ns: 0,
            width: w as u32,
            height: h as u32,
            format: format::R8G8B8A8_SRGB,
            bytes_per_pixel: 4,
            views: [EyeView::default(); 2],
            pixels,
        }
    }

    #[test]
    fn a_reflection_moves_a_screen_flickers() {
        // A mirror at x 8..24 shows the arms (bright) only when they are out;
        // a screen at x 40..56 flickers on its own.
        let rest = frame(|x, _| if (40..56).contains(&x) { 200 } else { 50 });
        let out = frame(|x, y| if (8..24).contains(&x) && (16..32).contains(&y) { 230 } else if (40..56).contains(&x) { 20 } else { 50 });
        let back = frame(|x, _| if (40..56).contains(&x) { 120 } else { 50 });
        let m = find(&rest, &out, &back, 8, 20.0, 2);
        assert!(m.at(12.0, 20.0) && m.at(20.0, 28.0));
        assert!(!m.at(44.0, 20.0), "the screen is not a mirror");
        assert!(!m.at(30.0, 50.0));
    }
}
