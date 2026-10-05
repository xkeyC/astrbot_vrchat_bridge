//! What the bot knows of its surroundings.
//!
//! - [`panorama`]: an equirectangular picture stitched from the frames of a
//!   head scan, each placed by the exact pose it was rendered with.
//! - [`heightmap`]: a 2.5D grid over the floor from stereo points: the
//!   floor's height, what stands on it, and what is still unseen.
//! - [`candidates`]: places worth going to, numbered for a model to pick;
//!   [`draw`] marks them on the panorama and the map.

pub mod candidates;
pub mod draw;
pub mod heightmap;
pub mod mirror;
pub mod panorama;

pub use candidates::{candidates, Candidate, CandidateParams, Kind, Person};
pub use heightmap::{Cell, HeightMap, MapParams};
pub use panorama::Panorama;
