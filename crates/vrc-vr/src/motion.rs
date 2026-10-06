//! Motion clips played on the full body: the trackers, the headset and both
//! hands moved as one skeleton (`tools/motion`, `docs/full-vr/motion.md`).
//!
//! A clip is the canonical skeleton's tracked joints at a fixed rate, in
//! the body's frame (x its left, y up, z ahead; units of its stature; the
//! origin on the floor under where its hips started), each joint's rotation
//! relative to the calibration pose. Played, each joint moves what is
//! attached to it as it was attached when standing for the calibration:
//!
//! - the trackers (hip, feet, ...) where `trackers::standing` puts them;
//! - the headset (the eyes) on the head;
//! - each hand's grip where the animation rests it (`anim::hand_pose`).
//!
//! So a clip standing still reproduces exactly what the body does at rest,
//! and the clip's proportions are the calibration's whatever the avatar.

use std::path::Path;

use anyhow::{bail, Context, Result};
use serde::Deserialize;

use crate::pose::{quat_mul, Pose};
use crate::trackers::Part;

/// The joints a clip holds, in its order.
pub const JOINTS: [&str; 11] =
    ["hips", "chest", "head", "l_leg", "r_leg", "l_foot", "r_foot", "l_forearm", "r_forearm", "l_hand", "r_hand"];
const HIPS: usize = 0;
const CHEST: usize = 1;
const HEAD: usize = 2;
const L_KNEE: usize = 3;
const R_KNEE: usize = 4;
const L_FOOT: usize = 5;
const R_FOOT: usize = 6;
const L_HAND: usize = 9;
const R_HAND: usize = 10;

/// A joint of a clip: position (body frame, stature units) and rotation
/// relative to the calibration pose (body frame, x y z w).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint {
    pub pos: [f32; 3],
    pub rot: [f32; 4],
}

/// A whole pose.
pub type Body = [Joint; 11];

#[derive(Deserialize)]
struct File {
    name: String,
    fps: f32,
    #[serde(default)]
    looping: Option<bool>,
    #[serde(default, rename = "loop")]
    loop_: bool,
    joints: Vec<String>,
    standing: std::collections::HashMap<String, [f32; 3]>,
    frames: Vec<Vec<f32>>,
    #[serde(default)]
    curls: Option<Vec<Vec<f32>>>,
    #[serde(default)]
    root: Option<String>,
    #[serde(default)]
    exit: Option<String>,
    #[serde(default)]
    posture: Option<String>,
    #[serde(default)]
    speed: Option<f32>,
}

/// How the body moves with a clip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Root {
    /// It moves with the clip and stays where it ends (standing).
    Travel,
    /// Its travel on the floor is taken out.
    InPlace,
    /// As is, and the next clip goes on from where this one leaves the
    /// body (lying down, then lying): nothing is committed.
    Hold,
}

impl Root {
    pub fn parse(s: &str) -> Option<Root> {
        Some(match s {
            "travel" => Root::Travel,
            "in_place" => Root::InPlace,
            "hold" => Root::Hold,
            _ => return None,
        })
    }
}

/// A clip.
#[derive(Clone, Debug)]
pub struct Clip {
    pub name: String,
    pub fps: f32,
    pub looping: bool,
    pub frames: Vec<Body>,
    /// The joints in the calibration pose.
    pub standing: Body,
    /// Finger curls per frame as changes from the relaxed hand: left then
    /// right, each little, ring, middle, index, thumb (empty: none).
    pub curls: Vec<[f32; 10]>,
    pub root: Root,
    /// The clip that ends this posture (lying: getting up), played when a
    /// program that stays in it is stopped or followed by another.
    pub exit: Option<String>,
    /// The posture a held clip keeps (lying, sitting): another clip of it
    /// goes on from it directly.
    pub posture: Option<String>,
    /// A gait cycle's speed over the floor (statures a second), played in
    /// place at the body's own speed.
    pub speed: Option<f32>,
}

impl Clip {
    pub fn load(path: &Path) -> Result<Clip> {
        let text = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        Clip::parse(&text)
    }

    pub fn parse(json: &[u8]) -> Result<Clip> {
        let f: File = serde_json::from_slice(json)?;
        if f.joints.iter().map(String::as_str).ne(JOINTS) {
            bail!("clip {}: joints {:?}, expected {:?}", f.name, f.joints, JOINTS);
        }
        if !(f.fps > 0.0) || f.frames.is_empty() {
            bail!("clip {}: no frames", f.name);
        }
        let identity = [0.0, 0.0, 0.0, 1.0];
        let mut standing = [Joint { pos: [0.0; 3], rot: identity }; 11];
        for (i, name) in JOINTS.iter().enumerate() {
            standing[i].pos = *f.standing.get(*name).with_context(|| format!("clip {}: no standing {name}", f.name))?;
        }
        let mut frames = Vec::with_capacity(f.frames.len());
        for (n, row) in f.frames.iter().enumerate() {
            if row.len() != 77 || row.iter().any(|x| !x.is_finite()) {
                bail!("clip {}: frame {n} is not 11 joints of 7 numbers", f.name);
            }
            let mut body = standing;
            for (i, j) in body.iter_mut().enumerate() {
                let r = &row[i * 7..i * 7 + 7];
                j.pos = [r[0], r[1], r[2]];
                j.rot = normalize([r[3], r[4], r[5], r[6]]);
            }
            frames.push(body);
        }
        let mut curls = Vec::new();
        if let Some(rows) = f.curls {
            if rows.len() != frames.len() {
                bail!("clip {}: {} curl rows for {} frames", f.name, rows.len(), frames.len());
            }
            for row in rows {
                if row.len() != 10 || row.iter().any(|x| !x.is_finite()) {
                    bail!("clip {}: a curl row is not 10 numbers", f.name);
                }
                curls.push(std::array::from_fn(|i| row[i].clamp(-1.0, 1.0)));
            }
        }
        let looping = f.looping.unwrap_or(f.loop_);
        let root = match f.root.as_deref() {
            Some(r) => Root::parse(r).with_context(|| format!("clip {}: root {r} (travel, in_place, hold)", f.name))?,
            None if looping => Root::InPlace,
            None => Root::Travel,
        };
        Ok(Clip { name: f.name, fps: f.fps, looping, frames, standing, curls, root, exit: f.exit, posture: f.posture,
            speed: f.speed.filter(|v| v.is_finite() && *v > 0.0) })
    }

    pub fn duration(&self) -> f32 {
        (self.frames.len().saturating_sub(1)) as f32 / self.fps
    }

    /// The pose `t` seconds in (wrapped when looping, held at the ends
    /// otherwise), interpolated between frames.
    pub fn sample(&self, t: f32) -> Body {
        let d = self.duration();
        let t = if self.looping && d > 0.0 { t.rem_euclid(d) } else { t.clamp(0.0, d) };
        let x = t * self.fps;
        let i = (x.floor() as usize).min(self.frames.len() - 1);
        let k = (i + 1).min(self.frames.len() - 1);
        blend(&self.frames[i], &self.frames[k], x - i as f32)
    }

    /// The fingers' curl changes `t` seconds in (both hands; zero without).
    pub fn sample_curls(&self, t: f32) -> [f32; 10] {
        if self.curls.is_empty() {
            return [0.0; 10];
        }
        let d = self.duration();
        let t = if self.looping && d > 0.0 { t.rem_euclid(d) } else { t.clamp(0.0, d) };
        let x = t * self.fps;
        let i = (x.floor() as usize).min(self.curls.len() - 1);
        let k = (i + 1).min(self.curls.len() - 1);
        let w = x - i as f32;
        std::array::from_fn(|n| self.curls[i][n] + (self.curls[k][n] - self.curls[i][n]) * w)
    }

    /// The same clip with left and right swapped (a wave with the other
    /// hand).
    pub fn mirrored(&self) -> Clip {
        let swap = |b: &Body| {
            let mut out = *b;
            for (a, c) in [(3, 4), (5, 6), (7, 8), (9, 10)] {
                out[a] = b[c];
                out[c] = b[a];
            }
            for j in &mut out {
                j.pos[0] = -j.pos[0];
                // Mirrored in x: x stays, y and z turn the other way.
                j.rot = [j.rot[0], -j.rot[1], -j.rot[2], j.rot[3]];
            }
            out
        };
        Clip {
            name: format!("{} (mirrored)", self.name),
            fps: self.fps,
            looping: self.looping,
            frames: self.frames.iter().map(swap).collect(),
            standing: swap(&self.standing),
            curls: self.curls.iter().map(|c| std::array::from_fn(|n| c[(n + 5) % 10])).collect(),
            root: self.root,
            exit: self.exit.clone(),
            posture: self.posture.clone(),
            speed: self.speed,
        }
    }

    /// Where the hips end, on the floor (body frame: x left, z ahead), and
    /// how far the body has turned (degrees, + to its right).
    pub fn end_root(&self) -> ([f32; 2], f32) {
        let last = self.frames.last().unwrap();
        let s = &self.standing[HIPS];
        let h = last[HIPS];
        let ahead = rotate(h.rot, [0.0, 0.0, 1.0]);
        // + to the right: the body's right is -x.
        let yaw = (-ahead[0]).atan2(ahead[2]).to_degrees();
        ([h.pos[0] - s.pos[0], h.pos[2] - s.pos[2]], yaw)
    }

    /// The clip with the hips' travel on the floor taken out (it plays where
    /// it stands); turns stay.
    pub fn in_place(&self) -> Clip {
        let s = self.standing[HIPS].pos;
        let frames = self
            .frames
            .iter()
            .map(|b| {
                let (dx, dz) = (b[HIPS].pos[0] - s[0], b[HIPS].pos[2] - s[2]);
                let mut out = *b;
                for j in &mut out {
                    j.pos[0] -= dx;
                    j.pos[2] -= dz;
                }
                out
            })
            .collect();
        Clip { frames, ..self.clone() }
    }
}

/// Joint by joint between `a` and `b` (0: a, 1: b).
pub fn blend(a: &Body, b: &Body, w: f32) -> Body {
    let mut out = *a;
    for i in 0..out.len() {
        out[i].pos = [0, 1, 2].map(|k| a[i].pos[k] + (b[i].pos[k] - a[i].pos[k]) * w);
        out[i].rot = nlerp(a[i].rot, b[i].rot, w);
    }
    out
}

fn normalize(q: [f32; 4]) -> [f32; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-6 {
        [0.0, 0.0, 0.0, 1.0]
    } else {
        q.map(|x| x / n)
    }
}

fn nlerp(a: [f32; 4], b: [f32; 4], w: f32) -> [f32; 4] {
    let dot: f32 = (0..4).map(|i| a[i] * b[i]).sum();
    let s = if dot < 0.0 { -1.0 } else { 1.0 };
    normalize([0, 1, 2, 3].map(|i| a[i] * (1.0 - w) + s * b[i] * w))
}

fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    Pose { orientation: q, position: [0.0; 3] }.rotate(v)
}

/// Where the body stands in the tracking space: its eyes when standing
/// (the headset's position), its facing, its floor and its stature.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stand {
    pub eyes: [f32; 3],
    pub yaw_deg: f32,
    pub floor_y: f32,
}

impl Stand {
    pub fn stature(&self) -> f32 {
        (self.eyes[1] - self.floor_y) / 0.936
    }

    /// A body-frame point (stature units) in the tracking space. The eyes
    /// stand `eyes_ahead` (stature units) ahead of the body frame's origin.
    fn point(&self, p: [f32; 3], eyes_ahead: f32) -> [f32; 3] {
        let s = self.stature();
        let facing = Pose::looking(self.yaw_deg, 0.0, [0.0; 3]);
        // Body axes in the tracking space: x left = -right, z ahead.
        let local = [-p[0] * s, 0.0, (p[2] - eyes_ahead) * s];
        let d = facing.rotate([local[0], 0.0, -local[2]]);
        [self.eyes[0] + d[0], self.floor_y + p[1] * s, self.eyes[2] + d[2]]
    }

    /// A body-frame rotation in the tracking space.
    fn rotation(&self, q: [f32; 4]) -> [f32; 4] {
        // Body axes to OpenXR facing -Z: a half turn about y; then the yaw.
        let c = [-q[0], q[1], -q[2], q[3]];
        let facing = Pose::looking(self.yaw_deg, 0.0, [0.0; 3]).orientation;
        let inv = [-facing[0], -facing[1], -facing[2], facing[3]];
        quat_mul(quat_mul(facing, c), inv)
    }
}

/// What a pose moves: the trackers, the headset and the hands' grips.
#[derive(Clone, Debug, PartialEq)]
pub struct Placed {
    pub trackers: Vec<(Part, Pose)>,
    pub head: Pose,
    pub left: Pose,
    pub right: Pose,
}

/// What is attached to the body when it stands at `stand` (tracking
/// space): the trackers, the headset (looking level ahead) and the grips.
pub struct Attached {
    pub trackers: Vec<(Part, Pose)>,
    pub head: Pose,
    pub left: Pose,
    pub right: Pose,
}

/// Each of `attached` moved with its joint from `standing` to `body`.
pub fn place(stand: &Stand, standing: &Body, body: &Body, eyes_ahead: f32, attached: &Attached) -> Placed {
    let carry = |j: usize, x: &Pose| -> Pose {
        let from = stand.point(standing[j].pos, eyes_ahead);
        let to = stand.point(body[j].pos, eyes_ahead);
        let r = stand.rotation(body[j].rot);
        let rel = [x.position[0] - from[0], x.position[1] - from[1], x.position[2] - from[2]];
        let d = Pose { orientation: r, position: [0.0; 3] }.rotate(rel);
        Pose { orientation: normalize(quat_mul(r, x.orientation)), position: [to[0] + d[0], to[1] + d[1], to[2] + d[2]] }
    };
    let joint_of = |part: Part| match part {
        Part::Hip => HIPS,
        Part::Chest => CHEST,
        Part::LeftFoot => L_FOOT,
        Part::RightFoot => R_FOOT,
        Part::LeftKnee => L_KNEE,
        Part::RightKnee => R_KNEE,
    };
    Placed {
        trackers: attached.trackers.iter().map(|(p, x)| (*p, carry(joint_of(*p), x))).collect(),
        head: carry(HEAD, &attached.head),
        left: carry(L_HAND, &attached.left),
        right: carry(R_HAND, &attached.right),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clip() -> Clip {
        // Two frames: standing, then the hips 0.1 ahead and turned 90 degrees
        // to the body's right (about y by -90 in the body frame).
        let standing = "{\"hips\":[0,0.53,0],\"chest\":[0,0.72,0],\"head\":[0,0.868,0],\"l_leg\":[0.0955,0.285,0],\"r_leg\":[-0.0955,0.285,0],\"l_foot\":[0.0955,0.039,0],\"r_foot\":[-0.0955,0.039,0],\"l_forearm\":[0.16,0.6,0],\"r_forearm\":[-0.16,0.6,0],\"l_hand\":[0.18,0.45,0],\"r_hand\":[-0.18,0.45,0]}";
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let mut f0 = Vec::new();
        let mut f1 = Vec::new();
        let st: std::collections::HashMap<String, [f32; 3]> = serde_json::from_str(standing).unwrap();
        for j in JOINTS {
            let p = st[j];
            f0.extend([p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0]);
            f1.extend([p[0], p[1], p[2] + 0.1, 0.0, -s, 0.0, s]);
        }
        let json = serde_json::json!({"name": "t", "fps": 1.0, "loop": false, "joints": JOINTS, "standing": st, "frames": [f0, f1]});
        Clip::parse(json.to_string().as_bytes()).unwrap()
    }

    fn attached(stand: &Stand) -> Attached {
        let t = crate::trackers::standing(&[Part::Hip], stand.eyes, stand.yaw_deg, stand.floor_y);
        let head = Pose::looking(stand.yaw_deg, 0.0, stand.eyes);
        Attached { trackers: t, head, left: head, right: head }
    }

    #[test]
    fn standing_moves_nothing() {
        let c = clip();
        let stand = Stand { eyes: [0.3, 1.56, -0.2], yaw_deg: 40.0, floor_y: -0.32 };
        let a = attached(&stand);
        let p = place(&stand, &c.standing, &c.frames[0], 0.045, &a);
        assert_eq!(p.trackers[0].1.position.map(|x| (x * 1e4).round()), a.trackers[0].1.position.map(|x| (x * 1e4).round()));
        assert!((0..3).all(|i| (p.head.position[i] - a.head.position[i]).abs() < 1e-4));
    }

    #[test]
    fn ahead_and_a_right_turn_follow_the_facing() {
        let c = clip();
        // Facing -Z (yaw 0): ahead is -Z.
        let stand = Stand { eyes: [0.0, 1.56, 0.0], yaw_deg: 0.0, floor_y: -0.32 };
        let a = attached(&stand);
        let p = place(&stand, &c.standing, &c.frames[1], 0.045, &a);
        let moved = p.head.position[2] - a.head.position[2];
        assert!(moved < -0.1, "the head went ahead (-Z): {moved}");
        // Turned 90 degrees to the right: it now looks along +X.
        let (yaw, _) = p.head.yaw_pitch();
        assert!((yaw - 90.0).abs() < 0.5, "{yaw}");
        let (end, turned) = c.end_root();
        assert!((end[1] - 0.1).abs() < 1e-4 && (turned - 90.0).abs() < 0.5, "{end:?} {turned}");
    }

    #[test]
    fn mirroring_swaps_sides_and_turns() {
        let m = clip().mirrored();
        let (_, turned) = m.end_root();
        assert!((turned + 90.0).abs() < 0.5, "{turned}");
        // The left knee's slot holds the right knee mirrored: still on the left.
        assert!(m.frames[0][3].pos[0] > 0.0);
    }

    #[test]
    fn sampling_interpolates_and_holds() {
        let c = clip();
        let mid = c.sample(0.5);
        assert!((mid[HIPS].pos[2] - 0.05).abs() < 1e-4);
        assert_eq!(c.sample(5.0)[HIPS].pos, c.frames[1][HIPS].pos);
    }
}
