//! What the bot knows of its surroundings.
//!
//! - [`panorama`]: an equirectangular picture stitched from the frames of a
//!   head scan, each placed by the exact pose it was rendered with.
//! - [`heightmap`]: a 2.5D grid over the floor from stereo points: the
//!   floor's height, what stands on it, and what is still unseen.

pub mod heightmap;
pub mod panorama;

pub use heightmap::{Cell, HeightMap, MapParams};
pub use panorama::Panorama;
