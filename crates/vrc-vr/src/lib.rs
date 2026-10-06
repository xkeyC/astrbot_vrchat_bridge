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
//! - [`anim`] gives the hands (and a little of the head) life: idle sway,
//!   arm swing when walking, gestures when talking.
//! - [`trackers`] sends VRChat's OSC trackers (hip, feet, ...): full body.
//!
//! See `docs/full-vr/` for how the pieces fit and why.

pub mod anim;
pub mod fps;
pub mod osc;
pub mod pose;
pub mod remote;
pub mod scan;
pub mod tap;
pub mod trackers;
pub mod walk;

pub use pose::{Fov, Pose};
