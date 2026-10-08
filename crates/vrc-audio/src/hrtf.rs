//! How a head hears a direction, and the templates the direction finder
//! compares each bin with.
//!
//! Two heads: [`SphericalHead`] (a model, always there) and
//! [`crate::HrirTable`] (Steam Audio's default HRTF, rendered offline by
//! `tools/hrtf-render`). The game renders voices with the second, so it is
//! the one to use when the table is there; the sphere has no pinnae and so
//! no front/back difference at all.

use std::ops::Range;

use realfft::num_complex::Complex32;
use realfft::RealFftPlanner;

use crate::{bin_hz, wrap_deg, RATE};

pub trait Hrtf: Send + Sync {
    fn name(&self) -> String;
    /// The left and right ear's response at each of `freqs` (Hz) for a
    /// source at azimuth `az`, elevation `el` (degrees).
    fn response(&self, az: f32, el: f32, freqs: &[f32]) -> Vec<(Complex32, Complex32)>;
    /// The interaural time difference (seconds, + when the left ear lags).
    fn itd_s(&self, az: f32, el: f32) -> f32;
}

/// A rigid sphere: Woodworth's ITD, and a head shadow that grows with the
/// frequency and the lateral angle. Front and back mirror each other
/// exactly.
#[derive(Clone, Copy, Debug)]
pub struct SphericalHead {
    pub radius_m: f32,
    /// The level difference at 90 degrees, high up.
    pub ild_max_db: f32,
    /// Where the shadow is half grown (Hz).
    pub shadow_hz: f32,
}

impl Default for SphericalHead {
    fn default() -> Self {
        SphericalHead { radius_m: 0.0875, ild_max_db: 18.0, shadow_hz: 1800.0 }
    }
}

const SOUND_M_S: f32 = 343.0;

impl SphericalHead {
    /// The lateral angle (radians, + right): what the cone of confusion keeps.
    fn lateral(az: f32, el: f32) -> f32 {
        (az.to_radians().sin() * el.to_radians().cos()).clamp(-1.0, 1.0).asin()
    }

    /// The right ear's level over the left's at `hz` (dB).
    pub fn ild_db(&self, az: f32, el: f32, hz: f32) -> f32 {
        let f2 = hz * hz;
        Self::lateral(az, el).sin() * self.ild_max_db * f2 / (f2 + self.shadow_hz * self.shadow_hz)
    }
}

impl Hrtf for SphericalHead {
    fn name(&self) -> String {
        format!("spherical head (r {} m)", self.radius_m)
    }

    fn response(&self, az: f32, el: f32, freqs: &[f32]) -> Vec<(Complex32, Complex32)> {
        let itd = self.itd_s(az, el);
        freqs
            .iter()
            .map(|&f| {
                let ild = self.ild_db(az, el, f);
                let w = 2.0 * std::f32::consts::PI * f;
                let l = Complex32::from_polar(10f32.powf(-ild / 40.0), -w * itd / 2.0);
                let r = Complex32::from_polar(10f32.powf(ild / 40.0), w * itd / 2.0);
                (l, r)
            })
            .collect()
    }

    fn itd_s(&self, az: f32, el: f32) -> f32 {
        let phi = Self::lateral(az, el);
        self.radius_m / SOUND_M_S * (phi + phi.sin())
    }
}

/// The interaural phase and level of every direction on an azimuth ring,
/// at the bins the direction finder uses.
pub struct Templates {
    pub name: String,
    /// Head-relative azimuths (degrees), evenly spaced from -180.
    pub az: Vec<f32>,
    pub step: f32,
    pub el: f32,
    /// The bins (of a `WINDOW` long transform).
    pub bins: Range<usize>,
    /// Per direction, per bin: arg(L conj R) (radians) and the left's level
    /// over the right's (dB).
    ipd: Vec<f32>,
    ild: Vec<f32>,
    /// Per direction (seconds, + when the left ear lags).
    pub itd: Vec<f32>,
}

impl Templates {
    /// `head`'s templates every `step` degrees of azimuth at elevation `el`.
    pub fn build(head: &dyn Hrtf, step: f32, el: f32, bins: Range<usize>) -> Templates {
        let n = (360.0 / step).round() as usize;
        let az: Vec<f32> = (0..n).map(|i| -180.0 + i as f32 * step).collect();
        let freqs: Vec<f32> = bins.clone().map(bin_hz).collect();
        let nb = freqs.len();
        let (mut ipd, mut ild, mut itd) = (Vec::with_capacity(n * nb), Vec::with_capacity(n * nb), Vec::with_capacity(n));
        for &a in &az {
            for (l, r) in head.response(a, el, &freqs) {
                let x = l * r.conj();
                ipd.push(x.arg());
                ild.push(10.0 * (l.norm_sqr().max(1e-20) / r.norm_sqr().max(1e-20)).log10());
            }
            itd.push(head.itd_s(a, el));
        }
        Templates { name: head.name(), az, step, el, bins, ipd, ild, itd }
    }

    pub fn len(&self) -> usize {
        self.az.len()
    }

    pub fn is_empty(&self) -> bool {
        self.az.is_empty()
    }

    /// Bins per direction.
    pub fn width(&self) -> usize {
        self.bins.len()
    }

    /// Direction `d`'s phases and levels over the bins.
    pub fn of(&self, d: usize) -> (&[f32], &[f32]) {
        let nb = self.width();
        (&self.ipd[d * nb..(d + 1) * nb], &self.ild[d * nb..(d + 1) * nb])
    }

    /// The index of the direction nearest `az`.
    pub fn index(&self, az: f32) -> usize {
        ((wrap_deg(az) + 180.0) / self.step).round() as usize % self.len()
    }
}

/// `mono` as `head` hears it from (`az`, `el`): (left, right), the same
/// length (a circular convolution on a padded buffer; for tests and tools).
pub fn render(head: &dyn Hrtf, az: f32, el: f32, mono: &[f32]) -> (Vec<f32>, Vec<f32>) {
    let n = (mono.len() + 4096).next_power_of_two();
    let mut planner = RealFftPlanner::<f32>::new();
    let (fwd, inv) = (planner.plan_fft_forward(n), planner.plan_fft_inverse(n));
    let mut buf = mono.to_vec();
    buf.resize(n, 0.0);
    let mut spec = fwd.make_output_vec();
    let _ = fwd.process(&mut buf, &mut spec);
    let freqs: Vec<f32> = (0..spec.len()).map(|k| k as f32 * RATE as f32 / n as f32).collect();
    let resp = head.response(az, el, &freqs);
    let ear = |left: bool| {
        let mut s: Vec<Complex32> = spec.iter().zip(&resp).map(|(x, (l, r))| x * if left { *l } else { *r } / n as f32).collect();
        // The DC and Nyquist bins of a real signal are real.
        let last = s.len() - 1;
        s[0].im = 0.0;
        s[last].im = 0.0;
        let mut out = inv.make_output_vec();
        let _ = inv.process(&mut s, &mut out);
        out.truncate(mono.len());
        out
    };
    (ear(true), ear(false))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hz_bin;

    #[test]
    fn a_sphere_lags_the_far_ear() {
        let h = SphericalHead::default();
        assert!(h.itd_s(0.0, 0.0).abs() < 1e-9);
        let side = h.itd_s(90.0, 0.0);
        assert!((side - 0.000656).abs() < 2e-5, "{side}");
        assert!((h.itd_s(-90.0, 0.0) + side).abs() < 1e-9);
        // Front and back alike.
        assert!((h.itd_s(30.0, 0.0) - h.itd_s(150.0, 0.0)).abs() < 1e-9);
        let t = Templates::build(&h, 2.0, 0.0, hz_bin(200.0)..hz_bin(5000.0));
        assert_eq!(t.len(), 180);
        assert_eq!(t.index(179.5), 0);
        assert_eq!(t.az[t.index(30.6)], 30.0);
        // A source on the right: the right ear louder up high.
        let (_, ild) = t.of(t.index(90.0));
        assert!(ild[ild.len() - 1] < -10.0);
    }

    #[test]
    fn rendering_delays_the_far_ear() {
        let h = SphericalHead::default();
        let mut click = vec![0.0f32; 2000];
        click[1000] = 1.0;
        let (l, r) = render(&h, 60.0, 0.0, &click);
        let peak = |x: &[f32]| (0..x.len()).max_by(|&a, &b| x[a].abs().total_cmp(&x[b].abs())).unwrap() as f32;
        let lag = peak(&l) - peak(&r);
        let want = h.itd_s(60.0, 0.0) * RATE as f32;
        assert!((lag - want).abs() <= 1.5, "{lag} vs {want}");
    }
}
