//! A table of head-related impulse responses (HRIRs) on a grid of
//! directions: what `tools/hrtf-render` writes from Steam Audio's default
//! HRTF, and what the templates are built from.
//!
//! The file (`VRCHRTF1`, little-endian):
//!
//! | at | what |
//! |---|---|
//! | 0 | `"VRCHRTF1"` |
//! | 8 | u32 sample rate |
//! | 12 | u32 taps (samples of each HRIR) |
//! | 16 | u32 elevations, u32 azimuths |
//! | 24 | f32 first elevation, its step; f32 first azimuth, its step (degrees) |
//! | 40 | u32 interpolation it was rendered with (0 nearest, 1 bilinear) |
//! | 44 | f32 scale: a sample is its i16 times this |
//! | 48 | u32 Steam Audio version (major << 16 \| minor << 8 \| patch) |
//! | 52 | zeros up to 64 |
//! | 64 | per direction, elevation-major: f32 peak delay left, right (seconds, as the API gives them); i16 left HRIR, i16 right HRIR |
//!
//! Azimuth 0 is ahead, + right; elevation + up.

use std::path::Path;

use anyhow::{bail, ensure, Context, Result};
use realfft::num_complex::Complex32;

use crate::hrtf::Hrtf;
use crate::wrap_deg;

const MAGIC: &[u8; 8] = b"VRCHRTF1";
const HEADER: usize = 64;

#[derive(Clone, Debug, PartialEq)]
pub struct HrirTable {
    pub rate: u32,
    pub taps: usize,
    pub elevations: Vec<f32>,
    pub azimuths: Vec<f32>,
    /// 0 nearest, 1 bilinear (Steam Audio's `IPLHRTFInterpolation`).
    pub interpolation: u32,
    pub version: u32,
    /// Per direction (elevation-major): the API's peak delays (seconds).
    pub peak_delays: Vec<[f32; 2]>,
    /// Per direction: left taps, then right taps.
    pub hrirs: Vec<f32>,
}

impl HrirTable {
    pub fn directions(&self) -> usize {
        self.elevations.len() * self.azimuths.len()
    }

    pub fn index(&self, el: usize, az: usize) -> usize {
        el * self.azimuths.len() + az
    }

    /// The left and right HRIR of direction `i`.
    pub fn hrir(&self, i: usize) -> (&[f32], &[f32]) {
        let at = i * 2 * self.taps;
        (&self.hrirs[at..at + self.taps], &self.hrirs[at + self.taps..at + 2 * self.taps])
    }

    /// The grid point nearest (`az`, `el`).
    pub fn nearest(&self, az: f32, el: f32) -> usize {
        let e = nearest_of(&self.elevations, el, |a, b| a - b);
        let a = nearest_of(&self.azimuths, az, |a, b| wrap_deg(a - b));
        self.index(e, a)
    }

    pub fn read(path: &Path) -> Result<HrirTable> {
        let bytes = std::fs::read(path).with_context(|| format!("no HRIR table at {}", path.display()))?;
        HrirTable::parse(&bytes).with_context(|| format!("HRIR table {}", path.display()))
    }

    pub fn parse(b: &[u8]) -> Result<HrirTable> {
        ensure!(b.len() >= HEADER && &b[..8] == MAGIC, "not a VRCHRTF1 file");
        let u = |at: usize| u32::from_le_bytes(b[at..at + 4].try_into().unwrap());
        let f = |at: usize| f32::from_le_bytes(b[at..at + 4].try_into().unwrap());
        let (rate, taps, n_el, n_az) = (u(8), u(12) as usize, u(16) as usize, u(20) as usize);
        let (el0, el_step, az0, az_step) = (f(24), f(28), f(32), f(36));
        let (interpolation, scale, version) = (u(40), f(44), u(48));
        ensure!(taps > 0 && taps <= 8192 && n_el > 0 && n_az > 0, "a bad header");
        let per = 8 + 4 * taps;
        let n = n_el * n_az;
        if b.len() != HEADER + n * per {
            bail!("{} bytes, the header says {}", b.len(), HEADER + n * per);
        }
        let mut peak_delays = Vec::with_capacity(n);
        let mut hrirs = Vec::with_capacity(n * 2 * taps);
        for d in 0..n {
            let at = HEADER + d * per;
            peak_delays.push([f(at), f(at + 4)]);
            hrirs.extend(b[at + 8..at + per].chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 * scale));
        }
        Ok(HrirTable {
            rate,
            taps,
            elevations: (0..n_el).map(|i| el0 + el_step * i as f32).collect(),
            azimuths: (0..n_az).map(|i| az0 + az_step * i as f32).collect(),
            interpolation,
            version,
            peak_delays,
            hrirs,
        })
    }

    /// The file's bytes. Grids must be evenly spaced.
    pub fn to_bytes(&self) -> Vec<u8> {
        let step = |v: &[f32]| if v.len() > 1 { v[1] - v[0] } else { 0.0 };
        let peak = self.hrirs.iter().fold(0f32, |m, x| m.max(x.abs())).max(1e-9);
        let scale = peak / 32767.0;
        let mut out = Vec::with_capacity(HEADER + self.directions() * (8 + 4 * self.taps));
        out.extend_from_slice(MAGIC);
        for v in [self.rate, self.taps as u32, self.elevations.len() as u32, self.azimuths.len() as u32] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for v in [self.elevations[0], step(&self.elevations), self.azimuths[0], step(&self.azimuths)] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&self.interpolation.to_le_bytes());
        out.extend_from_slice(&scale.to_le_bytes());
        out.extend_from_slice(&self.version.to_le_bytes());
        out.resize(HEADER, 0);
        for d in 0..self.directions() {
            out.extend_from_slice(&self.peak_delays[d][0].to_le_bytes());
            out.extend_from_slice(&self.peak_delays[d][1].to_le_bytes());
            let (l, r) = self.hrir(d);
            for x in l.iter().chain(r) {
                out.extend_from_slice(&((x / scale).round().clamp(-32768.0, 32767.0) as i16).to_le_bytes());
            }
        }
        out
    }

    pub fn write(&self, path: &Path) -> Result<()> {
        std::fs::write(path, self.to_bytes()).with_context(|| format!("writing {}", path.display()))
    }
}

fn nearest_of(grid: &[f32], v: f32, diff: impl Fn(f32, f32) -> f32) -> usize {
    (0..grid.len()).min_by(|&a, &b| diff(grid[a], v).abs().total_cmp(&diff(grid[b], v).abs())).unwrap_or(0)
}

impl Hrtf for HrirTable {
    fn name(&self) -> String {
        let interp = if self.interpolation == 1 { "bilinear" } else { "nearest" };
        let v = self.version;
        format!("steam-audio {}.{}.{} default HRTF ({interp}, {} taps)", v >> 16, (v >> 8) & 0xff, v & 0xff, self.taps)
    }

    /// The nearest grid point's HRIRs, transformed at `freqs`.
    fn response(&self, az: f32, el: f32, freqs: &[f32]) -> Vec<(Complex32, Complex32)> {
        let (l, r) = self.hrir(self.nearest(az, el));
        freqs.iter().map(|&f| (dft(l, f, self.rate), dft(r, f, self.rate))).collect()
    }

    /// From the API's peak delays: left minus right, + when the left ear
    /// lags (a source on the right).
    fn itd_s(&self, az: f32, el: f32) -> f32 {
        let [l, r] = self.peak_delays[self.nearest(az, el)];
        l - r
    }
}

/// `h` transformed at `hz`.
fn dft(h: &[f32], hz: f32, rate: u32) -> Complex32 {
    let w = -2.0 * std::f32::consts::PI * hz / rate as f32;
    h.iter().enumerate().fold(Complex32::new(0.0, 0.0), |acc, (n, &x)| acc + Complex32::from_polar(x, w * n as f32))
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn tiny() -> HrirTable {
        let (elevations, azimuths) = (vec![0.0, 10.0], vec![-180.0, -90.0, 0.0, 90.0]);
        let taps = 8;
        let n = elevations.len() * azimuths.len();
        let mut hrirs = Vec::new();
        for d in 0..n {
            for i in 0..2 * taps {
                hrirs.push(((d * 31 + i * 7) % 13) as f32 / 13.0 - 0.5);
            }
        }
        HrirTable {
            rate: 48_000,
            taps,
            elevations,
            azimuths,
            interpolation: 1,
            version: 0x040801,
            peak_delays: (0..n).map(|d| [d as f32, 1.0]).collect(),
            hrirs,
        }
    }

    #[test]
    fn round_trips_through_the_file_format() {
        let t = tiny();
        let back = HrirTable::parse(&t.to_bytes()).unwrap();
        assert_eq!((back.rate, back.taps, back.interpolation, back.version), (t.rate, t.taps, 1, 0x040801));
        assert_eq!(back.azimuths, t.azimuths);
        assert_eq!(back.elevations, t.elevations);
        assert_eq!(back.peak_delays, t.peak_delays);
        for (a, b) in back.hrirs.iter().zip(&t.hrirs) {
            assert!((a - b).abs() < 1e-4);
        }
        assert_eq!(back.nearest(170.0, 2.0), back.index(0, 0)); // wraps round to -180
        assert_eq!(back.nearest(80.0, 7.0), back.index(1, 3));
        assert!(HrirTable::parse(&t.to_bytes()[..100]).is_err());
    }
}
