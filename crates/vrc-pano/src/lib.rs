//! The avatar's panorama rig in the bot's eyes (docs/full-vr/
//! avatar-panorama.md): with the avatar parameter `Pano` on, six
//! local-only cameras on the avatar (four level, up, down) are copied into
//! the eyes, the left eye their colour and the right eye, pixel for pixel,
//! their depth; a code (magic 0x5B, a PosBeacon grid above the beacon)
//! says where the cameras are and how the depth is written. One tapped
//! frame is the whole sphere, in colour and in metres.
//!
//! - [`code`]: reading the code, and what a frame is ([`classify`]: a
//!   panorama, the usual view, or neither).
//! - [`layout`]: where the faces are in an eye and how each camera looks.
//! - [`calib`]: the right eye's calibration cells, undoing the world's
//!   post-processing before the depth is decoded.
//! - [`frame`]: [`decode`] into a [`PanoFrame`]: per face its colour,
//!   metric depth, intrinsics and rotation; rays, bearings from the head,
//!   world points.
//! - [`people`]: people by the depth: under a nameplate's ray, or
//!   person-shaped and unnamed; world to the tracking space.
//! - [`render`]: the equirectangular panorama (colour, depth) with the
//!   head's heading in the middle, as `vrc-scene`'s `Panorama`, and the raw
//!   tiles.
//!
//! The decoder assumes nothing of where the rig is or faces: the code's
//! position and yaw place it (the rig may follow the body or the head).

pub mod calib;
pub mod code;
pub mod frame;
pub mod layout;
pub mod people;
pub mod render;
#[cfg(any(test, feature = "synth"))]
pub mod synth;
#[cfg(test)]
mod tests;

pub use calib::Calibration;
pub use code::{classify, HeadPose, PanoCode, Route, Seen, E1C};
pub use frame::{decode, decode_as, to_map, CheckStats, PanoFrame, PanoParams, PanoPoint, PanoView, Ray, MASK_BODY, MASK_COLOUR, MASK_EDGE, MASK_OVERLAY};
pub use layout::{reserved_rect, Face};
pub use people::{Body, Cloud, PeopleParams, Tracking};
pub use render::{depth_rgb, downscale, Equirect};
