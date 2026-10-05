//! Name tags placed in the tracking space.
//!
//! An OCR line counts as a name tag only if it matches a player in the room
//! (posters and signs are ignored); the stereo disparity inside its box
//! places the tag, and the player stands on the floor under it.

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
