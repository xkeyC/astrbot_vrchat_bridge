//! Who is around and where. One logic for everything that looks for
//! players: the follower, the model's tools, and the candidates it picks
//! from.
//!
//! - [`ocr`]: text lines of a frame from local-multimodal-infra
//!   (`POST /v1/ocr/lines`, PP-OCRv5).
//! - [`room`]: the players in the room, from VRChat's log (joins and
//!   leaves since the last room entered).
//! - [`names`]: how well an OCR line matches a display name.
//! - [`objects`]: things in a frame (`POST /v1/detect/objects`), for the
//!   lasting map (placed by the panorama's depth, `vrc_nav::pano`).
//! - [`locate`]: a player seen and where (the bridge names the people in
//!   the panorama's depth), whitelisted friends marked.

pub mod locate;
pub mod names;
pub mod objects;
pub mod ocr;
pub mod room;

pub use locate::Sighting;
pub use objects::{DetectClient, Detection, ObjectSighting};
pub use ocr::{OcrClient, OcrLine};
