//! Poses and fields of view, in OpenXR conventions: metres, right-handed,
//! +Y up, -Z ahead; quaternions as x, y, z, w.

/// A rigid transform: rotation, then position.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Quaternion x, y, z, w.
    pub orientation: [f32; 4],
    pub position: [f32; 3],
}

impl Pose {
    pub const IDENTITY: Pose = Pose { orientation: [0.0, 0.0, 0.0, 1.0], position: [0.0; 3] };

    /// Looking `yaw_deg` to the right (negative: left) and `pitch_deg` up
    /// (negative: down), from `position`.
    pub fn looking(yaw_deg: f32, pitch_deg: f32, position: [f32; 3]) -> Pose {
        let yaw = quat_axis([0.0, 1.0, 0.0], -yaw_deg.to_radians());
        let pitch = quat_axis([1.0, 0.0, 0.0], pitch_deg.to_radians());
        Pose { orientation: quat_mul(yaw, pitch), position }
    }

    /// Yaw (+ right) and pitch (+ up) in degrees of where this pose looks.
    pub fn yaw_pitch(&self) -> (f32, f32) {
        let ahead = self.rotate([0.0, 0.0, -1.0]);
        let yaw = ahead[0].atan2(-ahead[2]).to_degrees();
        let pitch = ahead[1].clamp(-1.0, 1.0).asin().to_degrees();
        (yaw, pitch)
    }

    /// `v` rotated by this pose's orientation.
    pub fn rotate(&self, v: [f32; 3]) -> [f32; 3] {
        let [x, y, z, w] = self.orientation;
        // v + 2w (q x v) + 2 q x (q x v)
        let t = [2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])];
        [
            v[0] + w * t[0] + (y * t[2] - z * t[1]),
            v[1] + w * t[1] + (z * t[0] - x * t[2]),
            v[2] + w * t[2] + (x * t[1] - y * t[0]),
        ]
    }
}

impl Default for Pose {
    fn default() -> Self {
        Pose::IDENTITY
    }
}

/// Angles of the four sides of a view, radians (left and down are negative).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Fov {
    pub left: f32,
    pub right: f32,
    pub up: f32,
    pub down: f32,
}

impl Fov {
    /// Focal lengths and principal point (pixels) of a `width` x `height`
    /// image rendered with this field of view: fx, fy, cx, cy, with y down.
    pub fn intrinsics(&self, width: u32, height: u32) -> [f32; 4] {
        let (tl, tr) = (self.left.tan(), self.right.tan());
        let (tu, td) = (self.up.tan(), self.down.tan());
        let fx = width as f32 / (tr - tl);
        let fy = height as f32 / (tu - td);
        [fx, fy, -tl * fx, tu * fy]
    }
}

fn quat_axis(axis: [f32; 3], angle: f32) -> [f32; 4] {
    let (s, c) = (angle / 2.0).sin_cos();
    [axis[0] * s, axis[1] * s, axis[2] * s, c]
}

fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-3
    }

    #[test]
    fn looking_round_trips() {
        for (yaw, pitch) in [(0.0, 0.0), (60.0, 0.0), (0.0, -35.0), (-90.0, 10.0), (135.0, 40.0)] {
            let (y, p) = Pose::looking(yaw, pitch, [0.0; 3]).yaw_pitch();
            assert!(close(y, yaw) && close(p, pitch), "{yaw},{pitch} -> {y},{p}");
        }
    }

    #[test]
    fn yaw_right_matches_what_monado_reported() {
        // The tap showed this orientation for a head turned 60 degrees right.
        let q = Pose::looking(60.0, 0.0, [0.0; 3]).orientation;
        assert!(close(q[1], -0.5) && close(q[3], 0.866), "{q:?}");
    }

    #[test]
    fn intrinsics_of_a_symmetric_view() {
        let a = 42.5f32.to_radians();
        let fov = Fov { left: -a, right: a, up: a, down: -a };
        let [fx, fy, cx, cy] = fov.intrinsics(960, 960);
        assert!(close(fx, fy) && close(cx, 480.0) && close(cy, 480.0));
        assert!(close(fx, 480.0 / a.tan()));
    }
}
