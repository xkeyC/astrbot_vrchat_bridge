//! Looking around by turning only the head: the headset is virtual, so a
//! new head pose takes effect on the next rendered frame. A scan sets each
//! pose in turn and keeps the first frame rendered with it.
//!
//! Others in the room see the avatar's head snap around: keep scans short
//! and rare.

use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{bail, Result};

use crate::pose::Pose;
use crate::remote::RemoteHmd;
use crate::tap::{EyeFrame, EyeTap};

/// Degrees within which a rendered frame counts as looking where asked.
const AIM_TOLERANCE_DEG: f32 = 0.3;

/// One frame of a scan and how long it took to get it.
pub struct Shot {
    pub yaw: f32,
    pub pitch: f32,
    pub frame: EyeFrame,
    pub waited: Duration,
}

/// Head yaw and pitch (degrees) of a ring of `count` views around, at `pitch`.
pub fn ring(count: usize, pitch: f32) -> Vec<(f32, f32)> {
    (0..count).map(|i| (i as f32 * 360.0 / count as f32, pitch)).collect()
}

/// Turns the head through `views` (yaw, pitch in degrees, relative to the
/// tracking space) and returns the first frame rendered looking at each; the
/// head goes back to where it was at the end.
pub fn scan(hmd: &mut RemoteHmd, tap: &mut EyeTap, views: &[(f32, f32)], timeout: Duration) -> Result<Vec<Shot>> {
    let home = hmd.state.head;
    let mut shots = Vec::with_capacity(views.len());
    let result = (|| {
        for &(yaw, pitch) in views {
            let started = Instant::now();
            hmd.set_head(Pose::looking(yaw, pitch, home.position))?;
            let frame = rendered_at(tap, yaw, pitch, timeout)?;
            shots.push(Shot { yaw, pitch, frame, waited: started.elapsed() });
        }
        Ok(())
    })();
    hmd.set_head(home)?;
    result.map(|()| shots)
}

/// The first frame rendered looking at `yaw`, `pitch` (polls the tap's header).
pub fn rendered_at(tap: &mut EyeTap, yaw: f32, pitch: f32, timeout: Duration) -> Result<EyeFrame> {
    let started = Instant::now();
    let mut seen = None;
    while started.elapsed() < timeout {
        if let Some(head) = tap.peek()? {
            if Some(head.seq) != seen {
                seen = Some(head.seq);
                let (y, p) = head.views[0].pose.yaw_pitch();
                if angle_diff(y, yaw).abs() < AIM_TOLERANCE_DEG && (p - pitch).abs() < AIM_TOLERANCE_DEG {
                    if let Some(frame) = tap.read()? {
                        if frame.seq == head.seq {
                            return Ok(frame);
                        }
                        // Overwritten meanwhile: the next one is just as good if it still looks there.
                        let (y, p) = frame.views[0].pose.yaw_pitch();
                        if angle_diff(y, yaw).abs() < AIM_TOLERANCE_DEG && (p - pitch).abs() < AIM_TOLERANCE_DEG {
                            return Ok(frame);
                        }
                    }
                }
            }
        }
        sleep(Duration::from_millis(2));
    }
    bail!("no frame rendered looking at yaw {yaw}, pitch {pitch} within {timeout:?}")
}

/// `a - b` wrapped to -180..180 degrees.
pub fn angle_diff(a: f32, b: f32) -> f32 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wraps_angles() {
        assert!((angle_diff(179.0, -179.0) + 2.0).abs() < 1e-4);
        assert!((angle_diff(-90.0, 270.0)).abs() < 1e-4);
        assert_eq!(ring(4, -10.0), vec![(0.0, -10.0), (90.0, -10.0), (180.0, -10.0), (270.0, -10.0)]);
    }
}
