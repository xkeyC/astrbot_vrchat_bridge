//! Reader of the eye tap our Monado patch adds to its null compositor
//! (`third_party/monado/patches/0001-null-compositor-frame-tap.patch`).
//!
//! Monado writes, at most `XRT_NULL_TAP_FPS` times a second, the first
//! projection layer of the frame being committed to the file `XRT_NULL_TAP`
//! (e.g. `/dev/shm/vrc-eyes`): a 512 byte header, then both eyes' pixels
//! (left first, rows tightly packed, in the app's swapchain format). The
//! header's `seq` is odd while a frame is being written.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::atomic::{fence, AtomicU64, Ordering};
use std::thread::sleep;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use memmap2::Mmap;

use crate::pose::{Fov, Pose};

const HEADER_SIZE: usize = 512;
const MAGIC: &[u8; 8] = b"MNDTAP01";

/// Vulkan formats the tap writes (VkFormat values).
pub mod format {
    pub const R8G8B8A8_UNORM: u32 = 37;
    pub const R8G8B8A8_SRGB: u32 = 43;
    pub const B8G8R8A8_UNORM: u32 = 44;
    pub const B8G8R8A8_SRGB: u32 = 50;
}

/// How one eye was rendered: the pose and field of view the app used, in
/// the app's tracking space.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EyeView {
    pub fov: Fov,
    pub pose: Pose,
}

/// Both eyes of one frame.
#[derive(Clone, Debug)]
pub struct EyeFrame {
    pub seq: u64,
    pub frame_id: i64,
    /// The time the app rendered for.
    pub display_time_ns: i64,
    /// When the tap copied it out (CLOCK_MONOTONIC).
    pub capture_ns: i64,
    /// Of one eye.
    pub width: u32,
    pub height: u32,
    /// A VkFormat, see [`format`].
    pub format: u32,
    pub bytes_per_pixel: u32,
    pub views: [EyeView; 2],
    /// Left eye, then right eye.
    pub pixels: Vec<u8>,
}

impl EyeFrame {
    /// The pixels of eye 0 (left) or 1 (right).
    pub fn eye(&self, i: usize) -> &[u8] {
        let n = (self.width * self.height * self.bytes_per_pixel) as usize;
        &self.pixels[i * n..(i + 1) * n]
    }

    /// Eye `i` as tightly packed RGB8, for 8-bit formats.
    pub fn eye_rgb8(&self, i: usize) -> Result<Vec<u8>> {
        let bgr = match self.format {
            format::R8G8B8A8_UNORM | format::R8G8B8A8_SRGB => false,
            format::B8G8R8A8_UNORM | format::B8G8R8A8_SRGB => true,
            other => bail!("no RGB8 conversion for VkFormat {other}"),
        };
        Ok(self
            .eye(i)
            .chunks_exact(4)
            .flat_map(|p| if bgr { [p[2], p[1], p[0]] } else { [p[0], p[1], p[2]] })
            .collect())
    }

    /// Distance between the two eyes, metres (the stereo baseline).
    pub fn baseline(&self) -> f32 {
        let [a, b] = [self.views[0].pose.position, self.views[1].pose.position];
        ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
    }
}

/// The tap file, mapped.
pub struct EyeTap {
    path: PathBuf,
    map: Option<Mmap>,
}

impl EyeTap {
    pub fn open(path: impl AsRef<Path>) -> EyeTap {
        EyeTap { path: path.as_ref().to_owned(), map: None }
    }

    /// The latest frame; `None` until Monado wrote one. Waits out a frame
    /// being written (up to about a second).
    pub fn read(&mut self) -> Result<Option<EyeFrame>> {
        for _ in 0..200 {
            let len = std::fs::metadata(&self.path)
                .with_context(|| format!("no tap at {}", self.path.display()))?
                .len() as usize;
            if len < HEADER_SIZE {
                return Ok(None);
            }
            // The tap resizes the file when the eyes change size: map again.
            if self.map.as_ref().map(|m| m.len()) != Some(len) {
                let file = File::open(&self.path)?;
                // SAFETY: Monado only writes inside the seqlock, which the
                // copy below checks; the file is never shrunk under a reader
                // that mapped its current size, as sizes only change with
                // the eyes (a remap above).
                self.map = Some(unsafe { Mmap::map(&file)? });
            }
            let map = self.map.as_ref().unwrap();
            if &map[..8] != MAGIC {
                return Ok(None);
            }
            // SAFETY: offset 8 of a page-aligned mapping is 8-aligned.
            let seq_cell = unsafe { &*(map.as_ptr().add(8) as *const AtomicU64) };
            let seq = seq_cell.load(Ordering::Acquire);
            if seq == 0 || seq % 2 == 1 {
                sleep(Duration::from_millis(5));
                continue;
            }
            let header: [u8; HEADER_SIZE] = map[..HEADER_SIZE].try_into().unwrap();
            let pixels = map[HEADER_SIZE..].to_vec();
            fence(Ordering::Acquire);
            if seq_cell.load(Ordering::Relaxed) != seq {
                continue;
            }
            return parse(seq, &header, pixels).map(Some);
        }
        bail!("the tap kept being written")
    }
}

fn parse(seq: u64, h: &[u8; HEADER_SIZE], pixels: Vec<u8>) -> Result<EyeFrame> {
    let i64_at = |at: usize| i64::from_le_bytes(h[at..at + 8].try_into().unwrap());
    let u32_at = |at: usize| u32::from_le_bytes(h[at..at + 4].try_into().unwrap());
    let f32_at = |at: usize| f32::from_le_bytes(h[at..at + 4].try_into().unwrap());
    let view = |at: usize| {
        let f = |i: usize| f32_at(at + 4 * i);
        EyeView {
            fov: Fov { left: f(0), right: f(1), up: f(2), down: f(3) },
            pose: Pose { orientation: [f(4), f(5), f(6), f(7)], position: [f(8), f(9), f(10)] },
        }
    };
    let frame = EyeFrame {
        seq,
        frame_id: i64_at(16),
        display_time_ns: i64_at(24),
        capture_ns: i64_at(32),
        width: u32_at(40),
        height: u32_at(44),
        format: u32_at(48),
        bytes_per_pixel: u32_at(52),
        views: [view(56), view(100)],
        pixels,
    };
    let need = 2 * (frame.width * frame.height * frame.bytes_per_pixel) as usize;
    if frame.pixels.len() < need {
        bail!("tap holds {} bytes of pixels, the header says {need}", frame.pixels.len());
    }
    Ok(frame)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_what_the_tap_writes() {
        let mut h = [0u8; HEADER_SIZE];
        h[..8].copy_from_slice(MAGIC);
        h[16..24].copy_from_slice(&7i64.to_le_bytes());
        for (at, v) in [(40, 2u32), (44, 1), (48, format::B8G8R8A8_SRGB), (52, 4)] {
            h[at..at + 4].copy_from_slice(&v.to_le_bytes());
        }
        // right eye position x at 100 + 4 * 8
        h[132..136].copy_from_slice(&0.0315f32.to_le_bytes());
        h[88..92].copy_from_slice(&(-0.0315f32).to_le_bytes());
        let pixels = vec![1, 2, 3, 255, 4, 5, 6, 255, 7, 8, 9, 255, 10, 11, 12, 255];
        let f = parse(2, &h, pixels).unwrap();
        assert_eq!((f.frame_id, f.width, f.height), (7, 2, 1));
        assert_eq!(f.eye_rgb8(1).unwrap(), vec![9, 8, 7, 12, 11, 10]);
        assert!((f.baseline() - 0.063).abs() < 1e-6);
    }
}
