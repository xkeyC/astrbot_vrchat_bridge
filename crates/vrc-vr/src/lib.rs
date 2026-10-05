//! The virtual headset of the VRChat bot.
//!
//! VRChat runs in VR mode against Monado (OpenVR -> xrizer -> OpenXR -> Monado)
//! with no headset attached:
//!
//! - [`remote`] sets where the head, each eye and both controllers are, through
//!   Monado's remote driver (TCP).
//! - [`tap`] reads the eyes VRChat rendered, with the exact pose and field of
//!   view of each, from the frame tap our Monado patch adds to its null
//!   compositor (shared memory).
//!
//! See `docs/full-vr/` for how the pieces fit and why.

pub mod pose;
pub mod remote;
pub mod scan;
pub mod tap;

pub use pose::{Fov, Pose};
