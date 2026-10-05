//! Reader of the eye tap our Monado patch adds to its null compositor
//! (`third_party/monado/patches/0001-null-compositor-frame-tap.patch`).
//!
//! Monado writes the first projection layer of each committed frame (at
//! most `XRT_NULL_TAP_FPS` a second, 0: all) to a ring of
//! `XRT_NULL_TAP_SLOTS` frames in the file `XRT_NULL_TAP` (e.g.
//! `/dev/shm/vrc-eyes`):
//!
//! - file header (512 bytes): `"MNDTAP02"`, u32 slots @8, u64 slot size @16,
//!   u64 frames written @24 (the latest is in slot `(n - 1) % slots`);
//! - each slot: a 512 byte header (`"MNDSLOT1"`, seq, frame id, times, eye
//!   size and format, per-eye FOV and pose) and both eyes' pixels, left
//!   first, rows tightly packed, in the app's swapchain format.
//!
//! A slot's `seq` is `2n` while frame `n` (from 1) is in it and odd while it
//! is being written: a seqlock, and a frame's identity across slots.

use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::atomic::{fence, AtomicU64, Ordering};
use std::thread::sleep;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use memmap2::Mmap;

use crate::pose::{Fov, Pose};

const HEADER_SIZE: usize = 512;
const MAGIC: &[u8; 8] = b"MNDTAP02";

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
    /// 2n for the tap's n-th frame.
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
    /// Left eye, then right eye (empty from [`EyeTap::peek`]).
    pub pixels: Vec<u8>,
}

impl EyeFrame {
    /// Bytes of one eye's pixels.
    pub fn eye_bytes(&self) -> usize {
        (self.width * self.height * self.bytes_per_pixel) as usize
    }

    /// The pixels of eye 0 (left) or 1 (right).
    pub fn eye(&self, i: usize) -> &[u8] {
        let n = self.eye_bytes();
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

    /// Frames written so far (0 before the first, or while there is no tap).
    pub fn written(&mut self) -> Result<u64> {
        Ok(match self.ring()? {
            Some(ring) => ring.written(),
            None => 0,
        })
    }

    /// The latest frame; `None` until Monado wrote one.
    pub fn read(&mut self) -> Result<Option<EyeFrame>> {
        self.latest(true)
    }

    /// The latest frame without its pixels (`pixels` empty): cheap enough
    /// to poll for a frame rendered at some pose.
    pub fn peek(&mut self) -> Result<Option<EyeFrame>> {
        self.latest(false)
    }

    /// Every frame still in the ring, without pixels, oldest first.
    pub fn peek_all(&mut self) -> Result<Vec<EyeFrame>> {
        let Some(ring) = self.ring()? else { return Ok(Vec::new()) };
        let mut frames: Vec<EyeFrame> =
            (0..ring.slots).filter_map(|i| ring.slot(i, false).ok().flatten()).collect();
        frames.sort_by_key(|f| f.seq);
        Ok(frames)
    }

    /// The frame with this `seq`, if it is still in the ring.
    pub fn read_seq(&mut self, seq: u64) -> Result<Option<EyeFrame>> {
        let Some(ring) = self.ring()? else { return Ok(None) };
        if seq < 2 {
            return Ok(None);
        }
        let slot = ((seq / 2 - 1) % ring.slots as u64) as usize;
        Ok(ring.slot(slot, true)?.filter(|f| f.seq == seq))
    }

    fn latest(&mut self, pixels: bool) -> Result<Option<EyeFrame>> {
        for _ in 0..100 {
            let Some(ring) = self.ring()? else { return Ok(None) };
            let n = ring.written();
            if n == 0 {
                return Ok(None);
            }
            let slot = ((n - 1) % ring.slots as u64) as usize;
            match ring.slot(slot, pixels)? {
                Some(frame) if frame.seq == 2 * n => return Ok(Some(frame)),
                Some(_) => {}
                // Being written, or the slots just laid out anew: a moment.
                None => std::thread::sleep(std::time::Duration::from_millis(2)),
            }
            // Overwritten while reading, or a newer frame meanwhile: again.
        }
        bail!("the tap kept being written")
    }

    /// The ring as mapped now (remapped when Monado resized it).
    fn ring(&mut self) -> Result<Option<Ring<'_>>> {
        let len = std::fs::metadata(&self.path)
            .with_context(|| format!("no tap at {}", self.path.display()))?
            .len() as usize;
        if len < HEADER_SIZE {
            return Ok(None);
        }
        if self.map.as_ref().map(|m| m.len()) != Some(len) {
            let file = File::open(&self.path)?;
            // SAFETY: Monado only writes a slot inside its seqlock, which
            // the reads check; it lays the slots out anew only when the eyes
            // change size, and then only grows the file (patch 0001), so a
            // mapping of the old size never reaches past its end.
            self.map = Some(unsafe { Mmap::map(&file)? });
        }
        let map = self.map.as_ref().unwrap();
        if &map[..8] != MAGIC {
            return Ok(None);
        }
        let slots = u32::from_le_bytes(map[8..12].try_into().unwrap()) as usize;
        let slot_size = u64::from_le_bytes(map[16..24].try_into().unwrap()) as usize;
        if slots == 0 || HEADER_SIZE + slots * slot_size > map.len() {
            return Ok(None);
        }
        Ok(Some(Ring { map, slots, slot_size }))
    }
}

struct Ring<'a> {
    map: &'a Mmap,
    slots: usize,
    slot_size: usize,
}

impl Ring<'_> {
    fn written(&self) -> u64 {
        // SAFETY: offset 24 of a page-aligned mapping is 8-aligned.
        unsafe { &*(self.map.as_ptr().add(24) as *const AtomicU64) }.load(Ordering::Acquire)
    }

    /// Slot `i` if it holds a whole frame (waits out a write in progress).
    fn slot(&self, i: usize, pixels: bool) -> Result<Option<EyeFrame>> {
        let base = HEADER_SIZE + i * self.slot_size;
        let bytes = &self.map[base..base + self.slot_size];
        // SAFETY: slots start 512 + k * slot size in, and slot sizes (512 +
        // whole 4-byte pixels for two eyes) are multiples of 8, so +8 is 8-aligned.
        let seq_cell = unsafe { &*(bytes.as_ptr().add(8) as *const AtomicU64) };
        for _ in 0..200 {
            let seq = seq_cell.load(Ordering::Acquire);
            if seq == 0 {
                return Ok(None);
            }
            if seq % 2 == 1 {
                sleep(Duration::from_micros(500));
                continue;
            }
            let header: [u8; HEADER_SIZE] = bytes[..HEADER_SIZE].try_into().unwrap();
            let data = if pixels { bytes[HEADER_SIZE..].to_vec() } else { Vec::new() };
            fence(Ordering::Acquire);
            if seq_cell.load(Ordering::Relaxed) != seq {
                continue;
            }
            let frame = parse(seq, &header, data);
            if pixels && frame.pixels.len() < 2 * frame.eye_bytes() {
                bail!("a tap slot holds {} bytes, the frame needs {}", frame.pixels.len(), 2 * frame.eye_bytes());
            }
            return Ok(Some(frame));
        }
        bail!("tap slot {i} kept being written")
    }
}

fn parse(seq: u64, h: &[u8; HEADER_SIZE], pixels: Vec<u8>) -> EyeFrame {
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
    EyeFrame {
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    /// A ring file as Monado writes it: 3 slots, 2x1 eyes, `frames` written.
    fn ring_file(frames: u64) -> PathBuf {
        let slot_size = HEADER_SIZE + 2 * 2 * 4;
        let mut buf = vec![0u8; HEADER_SIZE + 3 * slot_size];
        buf[..8].copy_from_slice(MAGIC);
        buf[8..12].copy_from_slice(&3u32.to_le_bytes());
        buf[16..24].copy_from_slice(&(slot_size as u64).to_le_bytes());
        buf[24..32].copy_from_slice(&frames.to_le_bytes());
        for n in frames.saturating_sub(2).max(1)..=frames {
            let s = HEADER_SIZE + ((n - 1) % 3) as usize * slot_size;
            buf[s..s + 8].copy_from_slice(b"MNDSLOT1");
            buf[s + 8..s + 16].copy_from_slice(&(2 * n).to_le_bytes());
            buf[s + 16..s + 24].copy_from_slice(&(n as i64).to_le_bytes());
            for (at, v) in [(40, 2u32), (44, 1), (48, format::B8G8R8A8_SRGB), (52, 4)] {
                buf[s + at..s + at + 4].copy_from_slice(&v.to_le_bytes());
            }
            buf[s + 88..s + 92].copy_from_slice(&(-0.0315f32).to_le_bytes());
            buf[s + 132..s + 136].copy_from_slice(&0.0315f32.to_le_bytes());
            let px = [1, 2, 3, 255, 4, 5, 6, 255, 7, 8, 9, 255, 10, 11, n as u8, 255];
            buf[s + HEADER_SIZE..s + HEADER_SIZE + 16].copy_from_slice(&px);
        }
        let path = std::env::temp_dir().join(format!("vrc-tap-test-{frames}-{}", std::process::id()));
        File::create(&path).unwrap().write_all(&buf).unwrap();
        path
    }

    #[test]
    fn reads_the_ring() {
        let path = ring_file(5);
        let mut tap = EyeTap::open(&path);
        assert_eq!(tap.written().unwrap(), 5);
        let f = tap.read().unwrap().unwrap();
        assert_eq!((f.seq, f.frame_id, f.width, f.height), (10, 5, 2, 1));
        assert_eq!(f.eye_rgb8(1).unwrap(), vec![9, 8, 7, 5, 11, 10]);
        assert!((f.baseline() - 0.063).abs() < 1e-6);
        let all: Vec<i64> = tap.peek_all().unwrap().iter().map(|f| f.frame_id).collect();
        assert_eq!(all, vec![3, 4, 5]);
        assert_eq!(tap.read_seq(8).unwrap().unwrap().frame_id, 4);
        assert!(tap.read_seq(4).unwrap().is_none()); // frame 2: overwritten by frame 5
        std::fs::remove_file(path).ok();
    }
}
