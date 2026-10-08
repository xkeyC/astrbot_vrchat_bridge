//! The rig's code (magic 0x5B, avatar-panorama.md 3.6) and what a tapped
//! frame is: a panorama to decode, the eyes' usual view, or neither.
//!
//! The code is a PosBeacon grid (22 x 10 blocks, 160 bits, CRC-16) right
//! above the beacon, the same in both eyes: the camera centre's world
//! position, the rig's yaw, seq (the beacon's clock), age (1/30 s since
//! `Pano` came on, at most 15), layout, the depth's route and code, and its
//! log range. A frame is a panorama only when all of it holds (4.2):
//! never a guess.

use vrc_vr::beacon::{self, field, Beacon};
use vrc_vr::tap::EyeFrame;

use crate::layout::LAYOUT;

pub const MAGIC: u32 = 0x5B;
/// The code's bottom-left corner (NDC, y up): above the beacon.
pub const ORIGIN: [f32; 2] = [-0.96, -0.84];
/// Frames younger than this (1/30 s) may still show the faces the rig
/// left when it was last turned off.
pub const MIN_AGE: u8 = 2;

/// Where the depth comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Route {
    /// No depth.
    None,
    /// Depth-only cameras.
    D1,
    /// A depth light and an encoder quad.
    D2,
}

/// The depth's code field (bits 138-139, high bit first, as every field)
/// for E1C, the one encoding read (avatar-panorama.md 3.3): R = q,
/// G = 255 - q, B = |2q - 255|.
pub const E1C: u8 = 2;

/// What the code says.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PanoCode {
    /// The cameras' centre, Unity's world (metres).
    pub position: [f32; 3],
    /// Where the rig's ahead points: clockwise from +z seen from above
    /// (degrees, 0..360), as the beacon's yaw.
    pub rig_yaw: f32,
    pub seq: u8,
    /// 1/30 s since the rig came on (15: half a second or more).
    pub age: u8,
    pub layout: u8,
    pub route: Route,
    /// The depth's code field: `E1C`, or a frame `classify` turns down.
    pub depth_code: u8,
    /// The depth's log range (metres).
    pub zmin: f32,
    pub zmax: f32,
}

impl PanoCode {
    /// The 160 bits read (and CRC-checked) off the grid; `None`: not a pano
    /// code, or fields out of range.
    pub fn parse(bits: &[u8; 160]) -> Option<PanoCode> {
        if field(bits, 0, 8) != MAGIC {
            return None;
        }
        let position = [f32::from_bits(field(bits, 8, 32)), f32::from_bits(field(bits, 40, 32)), f32::from_bits(field(bits, 72, 32))];
        if !position.iter().all(|v| v.is_finite()) {
            return None;
        }
        let route = match field(bits, 136, 2) {
            0 => Route::None,
            1 => Route::D1,
            2 => Route::D2,
            _ => return None,
        };
        let depth_code = field(bits, 138, 2) as u8;
        Some(PanoCode {
            position,
            rig_yaw: field(bits, 104, 16) as f32 / 65536.0 * 360.0,
            seq: field(bits, 120, 8) as u8,
            age: field(bits, 128, 4) as u8,
            layout: field(bits, 132, 4) as u8,
            route,
            depth_code,
            zmin: 2f32.powi(field(bits, 140, 2) as i32 - 4),
            zmax: 2f32.powi(field(bits, 142, 2) as i32 + 5),
        })
    }

    /// The position in the bot's map axes (right-handed, -z ahead).
    pub fn position_map(&self) -> [f32; 3] {
        [self.position[0], self.position[1], -self.position[2]]
    }
}

/// The head as the beacon has it in the same frame: both eyes' middle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadPose {
    /// Unity's world (metres).
    pub position: [f32; 3],
    /// Where the head looks: clockwise from +z (degrees, 0..360).
    pub yaw: f32,
    /// Up positive (degrees).
    pub pitch: f32,
    pub seq: u8,
    /// Eyes read (1 or 2).
    pub eyes: u8,
}

impl HeadPose {
    /// From the beacons read in either eye; `None` when neither was, or
    /// they disagree on seq.
    pub fn from_beacons(left: Option<Beacon>, right: Option<Beacon>) -> Option<HeadPose> {
        let read: Vec<Beacon> = [left, right].into_iter().flatten().collect();
        let first = *read.first()?;
        if read.iter().any(|b| b.seq != first.seq) {
            return None;
        }
        let n = read.len() as f32;
        let mut position = [0.0f32; 3];
        for b in &read {
            for (p, v) in position.iter_mut().zip(b.position) {
                *p += v / n;
            }
        }
        // The yaws' circular mean.
        let (s, c) = read.iter().fold((0.0f32, 0.0f32), |(s, c), b| (s + b.yaw.to_radians().sin(), c + b.yaw.to_radians().cos()));
        Some(HeadPose {
            position,
            yaw: s.atan2(c).to_degrees().rem_euclid(360.0),
            pitch: read.iter().map(|b| b.pitch).sum::<f32>() / n,
            seq: first.seq,
            eyes: read.len() as u8,
        })
    }
}

/// What a tapped frame is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seen {
    /// A panorama to decode: every check held.
    Pano { code: PanoCode, head: HeadPose },
    /// A pano code reads, but the frame cannot be used (why): starting up,
    /// the eyes or the beacon out of step, a layout this crate does not
    /// know. The eyes are not the usual view either.
    Unusable(&'static str),
    /// No pano code in either eye: the eyes' usual view.
    Normal,
}

impl Seen {
    pub fn is_pano(&self) -> bool {
        matches!(self, Seen::Pano { .. })
    }

    /// For status: "pano", "unusable", "normal".
    pub fn name(&self) -> &'static str {
        match self {
            Seen::Pano { .. } => "pano",
            Seen::Unusable(_) => "unusable",
            Seen::Normal => "normal",
        }
    }
}

/// The code's bits in each eye (CRC-checked), the usual way up only: the
/// layout is drawn that way (verified in game).
fn code_bits(frame: &EyeFrame, eye: usize) -> Option<[u8; 160]> {
    beacon::read_grid(frame.eye(eye), frame.width, frame.height, ORIGIN, false)
}

/// Classifies a tapped frame (8-bit, 4 bytes a pixel; anything else is
/// `Normal`: no rig draws into it). Cheap: two grids and two beacons.
pub fn classify(frame: &EyeFrame) -> Seen {
    if frame.bytes_per_pixel != 4 || frame.pixels.len() < 2 * frame.eye_bytes() || frame.width < 64 || frame.height < 64 {
        return Seen::Normal;
    }
    let (left, right) = (code_bits(frame, 0), code_bits(frame, 1));
    let bits = match (left, right) {
        (None, None) => return Seen::Normal,
        (Some(l), Some(r)) if l == r => l,
        (Some(_), Some(_)) => return Seen::Unusable("the eyes' codes differ"),
        _ => return Seen::Unusable("the code reads in one eye only"),
    };
    let Some(code) = PanoCode::parse(&bits) else {
        return Seen::Unusable("not a pano code (magic or fields)");
    };
    if code.layout != LAYOUT {
        return Seen::Unusable("a layout this bridge does not know");
    }
    if code.depth_code != E1C {
        return Seen::Unusable("the depth is not E1C (an older pano rig on the avatar: upload it again)");
    }
    if code.age < MIN_AGE {
        return Seen::Unusable("just turned on (age < 2)");
    }
    let Some(head) = HeadPose::from_beacons(beacon::read(frame, 0), beacon::read(frame, 1)) else {
        return Seen::Unusable("no position beacon");
    };
    if head.seq != code.seq {
        return Seen::Unusable("the beacon's seq differs");
    }
    Seen::Pano { code, head }
}
