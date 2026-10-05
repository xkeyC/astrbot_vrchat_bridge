//! The frame rate of Monado's null compositor, changed while it runs (our
//! patch 0003, `XRT_NULL_FPS_FILE`): VRChat renders at whatever rate the
//! compositor paces it at, so a head scan can go fast for a moment and the
//! GPU rest the rest of the time.
//!
//! The file holds two little-endian u32: the rate asked for (0: Monado's
//! default, `XRT_COMPOSITOR_NULL_FPS`) and the rate in use.

use std::fs::OpenOptions;
use std::path::Path;
use std::sync::atomic::{AtomicU32, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use memmap2::MmapMut;

pub struct FpsControl {
    map: MmapMut,
}

impl FpsControl {
    pub fn open(path: impl AsRef<Path>) -> Result<FpsControl> {
        let path = path.as_ref();
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .with_context(|| format!("no frame rate control at {}", path.display()))?;
        // SAFETY: Monado maps the same 8 bytes; both sides only do atomic u32 accesses.
        let map = unsafe { MmapMut::map_mut(&file)? };
        if map.len() < 8 {
            bail!("{} is not a frame rate control", path.display());
        }
        Ok(FpsControl { map })
    }

    fn cell(&self, i: usize) -> &AtomicU32 {
        // SAFETY: in bounds (checked at open) and 4-aligned (page-aligned map).
        unsafe { &*(self.map.as_ptr().add(4 * i) as *const AtomicU32) }
    }

    /// Asks for `fps` (0: Monado's default).
    pub fn request(&self, fps: u32) {
        self.cell(0).store(fps, Ordering::Relaxed);
    }

    /// The rate in use.
    pub fn current(&self) -> u32 {
        self.cell(1).load(Ordering::Relaxed)
    }

    /// Asks for `fps` and waits (up to `timeout`) until Monado paces at it;
    /// returns the rate in use.
    pub fn set(&self, fps: u32, timeout: Duration) -> u32 {
        self.request(fps);
        let started = Instant::now();
        while started.elapsed() < timeout {
            let now = self.current();
            if fps == 0 || now == fps {
                return now;
            }
            sleep(Duration::from_millis(1));
        }
        self.current()
    }
}
