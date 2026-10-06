//! VRChat's OSC trackers: up to eight body trackers (hip, chest, feet,
//! knees, elbows) sent as OSC (`/tracking/trackers/1..8/position` and
//! `/rotation`), aligned to the avatar by the head's
//! (`/tracking/trackers/head/...`). VRChat calibrates them like other full
//! body trackers (Quick Menu, then both triggers) and assigns body parts by
//! where they are then: the slot numbers mean nothing to it.
//!
//! Their space is Unity's: metres, left-handed, +Y up, +Z ahead; rotations
//! as Euler angles in degrees applied Z, then X, then Y. We keep the
//! tracking space (OpenXR: -Z ahead) and convert here. The playspace never
//! turns (`walk`), so sending the head's pose from the same space aligns
//! the two once and for all.

use crate::osc::{encode, Arg};
use crate::pose::{quat_axis, Pose};

/// Body parts with a tracker, and their slots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    Hip,
    LeftFoot,
    RightFoot,
    Chest,
    LeftKnee,
    RightKnee,
}

impl Part {
    pub const ALL: [Part; 6] = [Part::Hip, Part::LeftFoot, Part::RightFoot, Part::Chest, Part::LeftKnee, Part::RightKnee];

    pub fn slot(self) -> u8 {
        self as u8 + 1
    }

    /// `hip`, `feet`, `chest`, `knees`: the parts a name stands for.
    pub fn named(name: &str) -> Option<&'static [Part]> {
        Some(match name {
            "hip" => &[Part::Hip],
            "feet" => &[Part::LeftFoot, Part::RightFoot],
            "chest" => &[Part::Chest],
            "knees" => &[Part::LeftKnee, Part::RightKnee],
            _ => return None,
        })
    }
}

/// Heights and offsets as fractions of the stature (Drillis & Contini): the
/// eyes at 0.936; (up, out to the side, ahead of the eyes' plumb line).
fn place(part: Part) -> [f32; 3] {
    match part {
        // A belt tracker: a little above the hip joints (0.530).
        Part::Hip => [0.56, 0.0, -0.04],
        Part::Chest => [0.72, 0.0, -0.01],
        // Between the instep and the ankle, a foot's width apart, behind
        // the eyes' plumb line: where the calibration mirror (trackers as
        // axes) showed them inside the boots; at [0.045, 0.06, 0.0] they
        // sat on the toes, at the sole.
        Part::LeftFoot => [0.085, -0.06, -0.05],
        Part::RightFoot => [0.085, 0.06, -0.05],
        // On the kneecaps (the knee joints at 0.285).
        Part::LeftKnee => [0.29, -0.055, 0.0],
        Part::RightKnee => [0.29, 0.055, 0.0],
    }
}

/// The trackers of a body standing straight under `head` (tracking space,
/// the eyes), facing `body_yaw_deg`, on a floor at `floor_y`.
pub fn standing(parts: &[Part], head: [f32; 3], body_yaw_deg: f32, floor_y: f32) -> Vec<(Part, Pose)> {
    let stature = (head[1] - floor_y) / 0.936;
    let facing = Pose::looking(body_yaw_deg, 0.0, [0.0; 3]);
    let right = facing.rotate([1.0, 0.0, 0.0]);
    let ahead = facing.rotate([0.0, 0.0, -1.0]);
    parts
        .iter()
        .map(|&part| {
            let [up, side, fwd] = place(part);
            let position = [
                head[0] + (right[0] * side + ahead[0] * fwd) * stature,
                floor_y + up * stature,
                head[2] + (right[2] * side + ahead[2] * fwd) * stature,
            ];
            (part, Pose { orientation: facing.orientation, position })
        })
        .collect()
}

impl Part {
    /// `hip`, `left_foot`, ... (snake case of the variant).
    pub fn key(self) -> &'static str {
        match self {
            Part::Hip => "hip",
            Part::LeftFoot => "left_foot",
            Part::RightFoot => "right_foot",
            Part::Chest => "chest",
            Part::LeftKnee => "left_knee",
            Part::RightKnee => "right_knee",
        }
    }
}

/// Moves each tracker by its `shift` (metres: right, up, ahead of where the
/// body faces `body_yaw_deg`).
pub fn shifted(trackers: &mut [(Part, Pose)], body_yaw_deg: f32, shift: impl Fn(Part) -> Option<[f32; 3]>) {
    let facing = Pose::looking(body_yaw_deg, 0.0, [0.0; 3]);
    let (right, ahead) = (facing.rotate([1.0, 0.0, 0.0]), facing.rotate([0.0, 0.0, -1.0]));
    for (part, pose) in trackers {
        if let Some([r, u, a]) = shift(*part) {
            pose.position[0] += right[0] * r + ahead[0] * a;
            pose.position[1] += u;
            pose.position[2] += right[2] * r + ahead[2] * a;
        }
    }
}

/// A tracking-space pose in Unity's terms: position, Euler angles (degrees,
/// Z then X then Y).
pub fn to_unity(p: &Pose) -> ([f32; 3], [f32; 3]) {
    let [x, y, z] = p.position;
    let [qx, qy, qz, qw] = p.orientation;
    ([x, y, -z], euler_zxy_deg([-qx, -qy, qz, qw]))
}

/// Unity's Euler angles of a quaternion (`Quaternion.Euler(x, y, z)` is
/// the turn about Y after X after Z).
pub fn euler_zxy_deg(q: [f32; 4]) -> [f32; 3] {
    let [x, y, z, w] = q;
    let sx = (2.0 * (w * x - y * z)).clamp(-1.0, 1.0);
    let ex = sx.asin();
    let (ey, ez) = if sx.abs() > 0.9999 {
        // Gimbal lock: all the turn about Y, none about Z.
        ((2.0 * (w * y - x * z)).atan2(1.0 - 2.0 * (y * y + z * z)), 0.0)
    } else {
        ((2.0 * (w * y + x * z)).atan2(1.0 - 2.0 * (x * x + y * y)), (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (x * x + z * z)))
    };
    [ex.to_degrees(), ey.to_degrees(), ez.to_degrees()]
}

/// Unity's `Quaternion.Euler` (for tests and checks).
pub fn unity_euler_quat(e: [f32; 3]) -> [f32; 4] {
    use crate::pose::quat_mul;
    let [x, y, z] = e.map(f32::to_radians);
    quat_mul(quat_axis([0.0, 1.0, 0.0], y), quat_mul(quat_axis([1.0, 0.0, 0.0], x), quat_axis([0.0, 0.0, 1.0], z)))
}

fn vec3(address: &str, v: [f32; 3]) -> Vec<u8> {
    encode(address, &v.map(Arg::Float))
}

/// The OSC messages for `trackers`, and the head's (its position; its
/// rotation too when `head_rotation`).
pub fn messages(trackers: &[(Part, Pose)], head: Option<&Pose>, head_rotation: bool) -> Vec<Vec<u8>> {
    let mut out = Vec::with_capacity(trackers.len() * 2 + 2);
    for (part, pose) in trackers {
        let (p, r) = to_unity(pose);
        let slot = part.slot();
        out.push(vec3(&format!("/tracking/trackers/{slot}/position"), p));
        out.push(vec3(&format!("/tracking/trackers/{slot}/rotation"), r));
    }
    if let Some(head) = head {
        let (p, r) = to_unity(head);
        out.push(vec3("/tracking/trackers/head/position", p));
        if head_rotation {
            out.push(vec3("/tracking/trackers/head/rotation", r));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn same_turn(a: [f32; 4], b: [f32; 4]) -> bool {
        let dot: f32 = (0..4).map(|i| a[i] * b[i]).sum();
        dot.abs() > 0.9999
    }

    #[test]
    fn euler_round_trips() {
        for e in [[0.0, 0.0, 0.0], [10.0, 20.0, 30.0], [-35.0, 120.0, 5.0], [80.0, -60.0, -45.0], [0.0, 179.0, 0.0]] {
            let q = unity_euler_quat(e);
            let back = euler_zxy_deg(q);
            assert!(same_turn(unity_euler_quat(back), q), "{e:?} -> {back:?}");
        }
    }

    #[test]
    fn a_right_turn_is_a_positive_unity_yaw() {
        // OpenXR: looking 30 degrees right; Unity: +30 about Y.
        let (_, e) = to_unity(&Pose::looking(30.0, 0.0, [0.0; 3]));
        assert!((e[1] - 30.0).abs() < 1e-3 && e[0].abs() < 1e-3 && e[2].abs() < 1e-3, "{e:?}");
        // Looking up is a negative turn about Unity's X.
        let (_, e) = to_unity(&Pose::looking(0.0, 20.0, [0.0; 3]));
        assert!((e[0] + 20.0).abs() < 1e-3, "{e:?}");
    }

    #[test]
    fn ahead_is_plus_z_in_unity() {
        let (p, _) = to_unity(&Pose { orientation: [0.0, 0.0, 0.0, 1.0], position: [0.1, 1.5, -2.0] });
        assert_eq!(p, [0.1, 1.5, 2.0]);
    }

    #[test]
    fn a_standing_body_is_under_the_head() {
        let head = [0.5, 1.56, -1.0];
        let t = standing(&Part::ALL, head, 90.0, -0.32);
        let get = |part| t.iter().find(|(p, _)| *p == part).unwrap().1.position;
        let (hip, lf, rf) = (get(Part::Hip), get(Part::LeftFoot), get(Part::RightFoot));
        // Facing +X (90 degrees right of -Z): the hip a little behind (-X),
        // the left foot at -Z (left of +X is -Z), the right at +Z.
        assert!(hip[0] < head[0] && (hip[2] - head[2]).abs() < 1e-4);
        assert!(lf[2] < head[2] && rf[2] > head[2]);
        assert!(lf[1] < -0.1 && hip[1] > 0.7 && hip[1] < 0.9, "{lf:?} {hip:?}");
    }

    #[test]
    fn messages_carry_three_floats() {
        let t = standing(Part::named("feet").unwrap(), [0.0, 1.56, 0.0], 0.0, -0.32);
        let m = messages(&t, Some(&Pose::IDENTITY), true);
        assert_eq!(m.len(), 6);
        assert!(m[0].starts_with(b"/tracking/trackers/2/position\0"));
        assert!(m[5].starts_with(b"/tracking/trackers/head/rotation\0"));
        // address (32) + ",fff" (8) + 3 floats.
        assert_eq!(m[0].len(), 32 + 8 + 12);
    }
}
