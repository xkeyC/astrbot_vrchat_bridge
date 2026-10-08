//! Short-time spectra of both ears: a Hann window of `WINDOW` samples moved
//! on by `HOP`.

use std::sync::Arc;

use realfft::num_complex::Complex32;
use realfft::{RealFftPlanner, RealToComplex};

use crate::{HOP, WINDOW};

/// One frame's spectra (`WINDOW / 2 + 1` bins each).
#[derive(Clone, Debug)]
pub struct Spectra {
    pub left: Vec<Complex32>,
    pub right: Vec<Complex32>,
}

pub struct Stft {
    fft: Arc<dyn RealToComplex<f32>>,
    window: Vec<f32>,
    /// The last WINDOW samples of each ear.
    left: Vec<f32>,
    right: Vec<f32>,
    input: Vec<f32>,
    scratch: Vec<Complex32>,
}

impl Default for Stft {
    fn default() -> Self {
        Stft::new()
    }
}

impl Stft {
    pub fn new() -> Stft {
        let fft = RealFftPlanner::<f32>::new().plan_fft_forward(WINDOW);
        let window = (0..WINDOW).map(|i| 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / WINDOW as f32).cos()).collect();
        let scratch = fft.make_scratch_vec();
        Stft { fft, window, left: vec![0.0; WINDOW], right: vec![0.0; WINDOW], input: vec![0.0; WINDOW], scratch }
    }

    /// Takes one hop of each ear (`HOP` samples); the spectra of the window
    /// that ends with it.
    pub fn push(&mut self, left: &[f32], right: &[f32]) -> Spectra {
        debug_assert!(left.len() == HOP && right.len() == HOP);
        for (buf, new) in [(&mut self.left, left), (&mut self.right, right)] {
            buf.copy_within(HOP.., 0);
            buf[WINDOW - HOP..].copy_from_slice(new);
        }
        Spectra { left: self.transform(true), right: self.transform(false) }
    }

    fn transform(&mut self, left: bool) -> Vec<Complex32> {
        let src = if left { &self.left } else { &self.right };
        for ((x, s), w) in self.input.iter_mut().zip(src).zip(&self.window) {
            *x = s * w;
        }
        let mut out = self.fft.make_output_vec();
        // Lengths are the plan's own: it cannot fail.
        let _ = self.fft.process_with_scratch(&mut self.input, &mut out, &mut self.scratch);
        out
    }
}

/// The power a full-scale sine gives in its bin (Hann window): levels are
/// relative to it.
pub fn full_scale_power() -> f32 {
    let a = WINDOW as f32 / 4.0;
    a * a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bin_hz, hz_bin};

    #[test]
    fn a_sine_lands_in_its_bin() {
        let mut stft = Stft::new();
        let k = hz_bin(1000.0);
        assert_eq!(bin_hz(k), 1000.0);
        let sine: Vec<f32> = (0..WINDOW).map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin()).collect();
        stft.push(&sine[..HOP], &sine[..HOP]);
        let s = stft.push(&sine[HOP..], &sine[HOP..]);
        let peak = (0..s.left.len()).max_by(|&a, &b| s.left[a].norm().total_cmp(&s.left[b].norm())).unwrap();
        assert_eq!(peak, k);
        let p = s.left[k].norm_sqr() / full_scale_power();
        assert!((p - 1.0).abs() < 0.05, "{p}");
    }
}
