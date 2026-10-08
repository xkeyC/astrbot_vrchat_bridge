//! Where a voice comes from, from the game's own binaural output.
//!
//! VRChat spatializes every voice with Steam Audio (its default HRTF, no
//! reflections, no occlusion) and the bot hears the result as the 2-channel
//! game output. That is a noise-free dummy head with a known HRTF: the
//! interaural phase and level of each time-frequency bin say the direction.
//!
//! - [`stft`]: 20 ms Hann windows every 10 ms of both ears.
//! - [`activity`]: speech or not, per frame (energy over a tracked floor and
//!   the share in the speech band, with hysteresis; no model).
//! - [`segment`]: speech segments, in the client's sample clock.
//! - [`hrtf`]: how a head hears a direction: a spherical head (Woodworth's
//!   ITD and a simple head shadow) or a table rendered from Steam Audio's
//!   default HRTF ([`table`], `tools/hrtf-render`).
//! - [`doa`]: per bin, the likelihood of each head-relative azimuth from the
//!   templates; summed over bins and frames it is a histogram over the whole
//!   circle (front and back are left ambiguous for the caller).
//! - [`mix`]: the mono mix sent on: both ears aligned on the dominant delay
//!   (a plain sum notches side voices), or the louder ear.
//! - [`front`]: all of it on 20 ms blocks of the capture.
//!
//! Angles are degrees, 0 ahead, + right; elevation + up. Directions use
//! Steam Audio's (and OpenXR's) axes: +x right, +y up, -z ahead.

pub mod activity;
pub mod doa;
pub mod front;
pub mod hrtf;
pub mod mix;
pub mod segment;
pub mod stft;
pub mod table;

pub use activity::{Activity, ActivityParams, Level};
pub use doa::{Doa, DoaParams, FrameDoa, Ring};
pub use front::{Block, Front, FrontParams, HopOut};
pub use hrtf::{Hrtf, SphericalHead, Templates};
pub use mix::{MixPolicy, MixTarget, MonoMix};
pub use segment::{SegEvent, Segmenter};
pub use stft::{Spectra, Stft};
pub use table::HrirTable;

/// The game's output rate.
pub const RATE: u32 = 48_000;
/// One analysis window (20 ms) and its hop (10 ms).
pub const WINDOW: usize = 960;
pub const HOP: usize = 480;
/// One block of the capture as sent on (20 ms): two hops.
pub const BLOCK: usize = 960;

/// The frequency of bin `k` of a `WINDOW` long transform.
pub fn bin_hz(k: usize) -> f32 {
    k as f32 * RATE as f32 / WINDOW as f32
}

/// The first bin at or above `hz`.
pub fn hz_bin(hz: f32) -> usize {
    (hz * WINDOW as f32 / RATE as f32).ceil().max(0.0) as usize
}

/// `deg` wrapped to -180..180.
pub fn wrap_deg(deg: f32) -> f32 {
    (deg + 540.0).rem_euclid(360.0) - 180.0
}

/// `rad` wrapped to -pi..pi.
pub fn wrap_rad(rad: f32) -> f32 {
    use std::f32::consts::PI;
    (rad + 3.0 * PI).rem_euclid(2.0 * PI) - PI
}

/// The unit vector toward azimuth `az`, elevation `el` (degrees).
pub fn direction(az: f32, el: f32) -> [f32; 3] {
    let (sa, ca) = az.to_radians().sin_cos();
    let (se, ce) = el.to_radians().sin_cos();
    [sa * ce, se, -ca * ce]
}

/// The head-relative azimuth with the same interaural cues as `az` from
/// behind (or in front): the cone of confusion's mirror.
pub fn mirror_deg(az: f32) -> f32 {
    wrap_deg(180.0 - az)
}
