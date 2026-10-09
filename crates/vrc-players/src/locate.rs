//! A player seen, placed in the tracking space (the bridge names the
//! people in the panorama's depth: `panolook`).

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
    /// The tag's box in the frame (left eye, frame pixels; zero when it was
    /// not read off the eyes).
    pub bbox: [f32; 4],
    /// When the frame was captured (CLOCK_MONOTONIC ns).
    pub seen_ns: i64,
}
