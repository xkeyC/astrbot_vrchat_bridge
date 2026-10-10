//! Who is around and where. One logic for everything that looks for
//! players: the follower, the model's tools, and the candidates it picks
//! from.
//!
//! - [`ocr`]: text lines of a frame from local-multimodal-infra
//!   (`POST /v1/ocr/lines`, PP-OCRv5).
//! - [`room`]: the players in the room, from VRChat's log (joins and
//!   leaves since the last room entered).
//! - [`names`]: how well an OCR line matches a display name.
//! - [`objects`]: things in a frame (`POST /v1/detect/objects`), placed by
//!   stereo for the lasting map.
//! - [`locate`]: name tags matched to the room's players (other text, like
//!   posters, is ignored), placed in the tracking space by the stereo
//!   disparity under them, whitelisted friends marked.

pub mod locate;
pub mod names;
pub mod objects;
pub mod ocr;
pub mod room;

pub use locate::{merge, sightings, Sighting};
pub use objects::{DetectClient, Detection, ObjectSighting};
pub use ocr::{OcrClient, OcrLine};
