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
/// head goes back to where it was at the end. The hands stay where the
/// caller put them: down at the sides ([`crate::remote::State::hands_at_rest`])
/// keeps the arms out of the views.
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

/// Like [`scan`], but without waiting for each view's frame before the next:
/// the head moves on every `hold`, and frames are claimed by the pose they
/// were rendered with as they come back (the render pipeline is ~40 ms deep,
/// so waiting view by view leaves the GPU idle). Views no frame came back for
/// are then taken one by one.
pub fn scan_pipelined(
    hmd: &mut RemoteHmd,
    tap: &mut EyeTap,
    views: &[(f32, f32)],
    hold: Duration,
    timeout: Duration,
) -> Result<Vec<Shot>> {
    let home = hmd.state.head;
    let started = Instant::now();
    let mut got: Vec<Option<Shot>> = views.iter().map(|_| None).collect();
    let result = (|| {
        let mut next = 0usize;
        let mut switch_at = started;
        let mut written = tap.written()?;
        let mut checked = 2 * written; // frames up to this seq are older than the scan
        let deadline = started + hold * views.len() as u32 + Duration::from_millis(400);
        while Instant::now() < deadline && got.iter().any(Option::is_none) {
            let now = Instant::now();
            if next < views.len() && now >= switch_at {
                let (yaw, pitch) = views[next];
                hmd.set_head(Pose::looking(yaw, pitch, home.position))?;
                next += 1;
                switch_at = now + hold;
            }
            let n = tap.written()?;
            if n != written {
                written = n;
                // The ring keeps the last few frames: claim each new one by its pose.
                for head in tap.peek_all()? {
                    if head.seq <= checked {
                        continue;
                    }
                    checked = head.seq;
                    let (y, p) = head.views[0].pose.yaw_pitch();
                    let hit = views.iter().enumerate().position(|(i, &(yaw, pitch))| {
                        got[i].is_none()
                            && angle_diff(y, yaw).abs() < AIM_TOLERANCE_DEG
                            && (p - pitch).abs() < AIM_TOLERANCE_DEG
                    });
                    if let Some(i) = hit {
                        if let Some(frame) = tap.read_seq(head.seq)? {
                            let (yaw, pitch) = views[i];
                            got[i] = Some(Shot { yaw, pitch, frame, waited: started.elapsed() });
                        }
                    }
                }
            }
            sleep(Duration::from_micros(500));
        }
        // Whatever the stream missed, one by one.
        for (i, &(yaw, pitch)) in views.iter().enumerate() {
            if got[i].is_none() {
                hmd.set_head(Pose::looking(yaw, pitch, home.position))?;
                let frame = rendered_at(tap, yaw, pitch, timeout)?;
                got[i] = Some(Shot { yaw, pitch, frame, waited: started.elapsed() });
            }
        }
        Ok(())
    })();
    hmd.set_head(home)?;
    result.map(|()| got.into_iter().flatten().collect())
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

/// The first frame newer than `after_seq` rendered looking at `yaw`, `pitch`
/// (for "the same view, after something changed").
pub fn rendered_after(tap: &mut EyeTap, after_seq: u64, yaw: f32, pitch: f32, timeout: Duration) -> Result<EyeFrame> {
    let started = Instant::now();
    while started.elapsed() < timeout {
        if let Some(head) = tap.peek()? {
            let (y, p) = head.views[0].pose.yaw_pitch();
            if head.seq > after_seq && angle_diff(y, yaw).abs() < AIM_TOLERANCE_DEG && (p - pitch).abs() < AIM_TOLERANCE_DEG {
                if let Some(frame) = tap.read_seq(head.seq)? {
                    return Ok(frame);
                }
            }
        }
        sleep(Duration::from_millis(2));
    }
    bail!("no new frame looking at yaw {yaw}, pitch {pitch} within {timeout:?}")
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
