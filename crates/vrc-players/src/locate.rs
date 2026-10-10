//! Name tags placed in the tracking space.
//!
//! An OCR line counts as a name tag only if it matches a player in the room
//! (posters and signs are ignored) and floats: a name written on a wall (a
//! world's board of who is in the room) has the wall round it at its depth,
//! a tag has what is behind the player there (`on_a_surface`). The stereo
//! disparity inside its box places the tag, and the player stands on the
//! floor under it.

use vrc_stereo::{Disparity, Stereo};
use vrc_vr::tap::EyeFrame;

use crate::names::best_match;
use crate::ocr::OcrLine;

/// A player seen in a frame.
#[derive(Clone, Debug)]
pub struct Sighting {
    /// The display name it matched (from the room's players).
    pub name: String,
    /// What OCR read.
    pub text: String,
    pub score: f32,
    /// 1-based priority on the whitelist, if whitelisted.
    pub whitelist_rank: Option<usize>,
    /// The name tag, tracking space.
    pub tag: [f32; 3],
    /// Where the player stands: under the tag, on the floor.
    pub feet: [f32; 3],
    /// The tag's box in the frame (left eye, frame pixels).
    pub bbox: [f32; 4],
    /// When the frame was captured (CLOCK_MONOTONIC ns).
    pub seen_ns: i64,
}

/// The players of `room` whose tags are among `lines` (read off the left
/// eye of `frame`), placed with `disp` (the frame's disparities, matched at
/// `stereo`'s scale); `whitelist` in priority order; `floor` its height.
pub fn sightings(
    frame: &EyeFrame,
    stereo: &Stereo,
    disp: &Disparity,
    lines: &[OcrLine],
    room: &[String],
    whitelist: &[String],
    floor: f32,
) -> Vec<Sighting> {
    let scale = frame.width as f32 / disp.width as f32;
    let mut out: Vec<Sighting> = Vec::new();
    for line in lines {
        let Some((who, score)) = best_match(&line.text, room) else { continue };
        let [x, y, w, h] = line.bbox;
        // Disparities inside the box (the text has texture; the plate around it may not).
        let mut ds: Vec<f32> = Vec::new();
        let (x0, x1) = ((x / scale) as usize, ((x + w) / scale).ceil() as usize);
        let (y0, y1) = ((y / scale) as usize, ((y + h) / scale).ceil() as usize);
        for yy in y0..y1.min(disp.height) {
            for xx in x0..x1.min(disp.width) {
                let d = disp.at(xx, yy);
                if d.is_finite() && d > 0.25 {
                    ds.push(d);
                }
            }
        }
        if ds.len() < 3 {
            continue;
        }
        ds.sort_by(f32::total_cmp);
        let d = ds[ds.len() / 2];
        if on_a_surface(disp, (x0, x1), (y0, y1), d) {
            continue; // a name on a wall, not over a player
        }
        let tag = stereo.point((x + w / 2.0) / scale, (y + h / 2.0) / scale, d);
        let name = room[who].clone();
        let whitelist_rank = whitelist.iter().position(|n| *n == name).map(|i| i + 1);
        let s = Sighting {
            name,
            text: line.text.clone(),
            score,
            whitelist_rank,
            tag,
            feet: [tag[0], floor, tag[2]],
            bbox: line.bbox,
            seen_ns: frame.capture_ns,
        };
        match out.iter_mut().find(|o| o.name == s.name) {
            Some(o) if o.score < s.score => *o = s,
            Some(_) => {}
            None => out.push(s),
        }
    }
    out
}

/// Beside a tag's text: past the plate's end on the right, and past the
/// avatar's picture on the left (box heights out from the text).
const RIGHT_BAND: (f32, f32) = (0.6, 2.6);
const LEFT_BAND: (f32, f32) = (2.0, 4.0);
/// A band is the same surface as the text when this share of its
/// disparities are within SAME_SHARE_D of the text's (and at least
/// SAME_MIN_PX pixels); a side with fewer disparities than BAND_POINTS says
/// nothing (a flat colour).
const SAME_SHARE: f32 = 0.6;
const SURE_SHARE: f32 = 0.8;
const SAME_SHARE_D: f32 = 0.08;
const SAME_MIN_PX: f32 = 0.3;
const BAND_POINTS: usize = 12;

/// Whether the text in the box (`x`, `y`: matched pixels, start and end)
/// at disparity `d` is written on a surface: both sides of it at its depth
/// (or one surely, the other unseen).
pub fn on_a_surface(disp: &Disparity, x: (usize, usize), y: (usize, usize), d: f32) -> bool {
    let h = (y.1.saturating_sub(y.0)).max(1) as f32;
    let rows = ((y.0 as f32 - 0.5 * h).max(0.0) as usize, ((y.1 as f32 + 0.5 * h) as usize).min(disp.height));
    // (valid, same) disparities in columns from..to.
    let band = |from: f32, to: f32| {
        let (from, to) = (from.max(0.0) as usize, (to.max(0.0) as usize).min(disp.width));
        let (mut valid, mut same) = (0usize, 0usize);
        for yy in rows.0..rows.1 {
            for xx in from..to {
                let v = disp.at(xx, yy);
                if v.is_finite() && v > 0.25 {
                    valid += 1;
                    same += ((v - d).abs() <= (SAME_SHARE_D * d).max(SAME_MIN_PX)) as usize;
                }
            }
        }
        (valid >= BAND_POINTS).then(|| same as f32 / valid as f32)
    };
    let right = band(x.1 as f32 + RIGHT_BAND.0 * h, x.1 as f32 + RIGHT_BAND.1 * h);
    let left = band(x.0 as f32 - LEFT_BAND.1 * h, x.0 as f32 - LEFT_BAND.0 * h);
    match (left, right) {
        (Some(l), Some(r)) => l >= SAME_SHARE && r >= SAME_SHARE,
        (Some(s), None) | (None, Some(s)) => s >= SURE_SHARE,
        (None, None) => false,
    }
}

/// One sighting per player across several frames (a scan): the best read.
pub fn merge(all: impl IntoIterator<Item = Sighting>) -> Vec<Sighting> {
    let mut out: Vec<Sighting> = Vec::new();
    for s in all {
        match out.iter_mut().find(|o| o.name == s.name) {
            Some(o) if o.score < s.score => *o = s,
            Some(_) => {}
            None => out.push(s),
        }
    }
    out.sort_by_key(|s| (s.whitelist_rank.unwrap_or(usize::MAX), s.name.clone()));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A disparity map `w` x `h` of `back` everywhere, `near` in the box.
    fn map(w: usize, h: usize, back: f32, near: f32, bx: (usize, usize), by: (usize, usize)) -> Disparity {
        let mut data = vec![back; w * h];
        for y in by.0..by.1 {
            for x in bx.0..bx.1 {
                data[y * w + x] = near;
            }
        }
        Disparity { width: w, height: h, data }
    }

    #[test]
    fn a_name_on_a_wall_is_not_a_tag() {
        // Text 40 x 8 px at disparity 10.
        let (bx, by) = ((100, 140), (50, 58));
        // On a wall: all round at 10 (a little noise).
        let mut wall = map(320, 120, 10.2, 10.0, bx, by);
        assert!(on_a_surface(&wall, bx, by, 10.0));
        // Over a player: the room behind at disparity 4 (the plate a
        // little wider than the text, at 10).
        let floating = map(320, 120, 4.0, 10.0, (bx.0 - 14, bx.1 + 3), by);
        assert!(!on_a_surface(&floating, bx, by, 10.0));
        // A player in front of a wall 0.5 m behind (at 3 m: disparity 10 vs 8.6).
        let near_wall = map(320, 120, 8.6, 10.0, (bx.0 - 14, bx.1 + 3), by);
        assert!(!on_a_surface(&near_wall, bx, by, 10.0));
        // A flat wall with no texture beside (no disparities): not judged.
        for v in wall.data.iter_mut() {
            *v = f32::NAN;
        }
        assert!(!on_a_surface(&wall, bx, by, 10.0));
    }
}
