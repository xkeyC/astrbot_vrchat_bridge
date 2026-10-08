//! Direction of arrival by matching each time-frequency bin against the
//! templates of a known HRTF.
//!
//! Per frame, a bin is kept when both ears agree in it (coherence over the
//! last few frames: one source dominates it, the W-disjoint orthogonality
//! of speech) and it stands out of its own noise floor. A kept bin's phase
//! difference and level difference give a likelihood for every template
//! direction; normalised over the directions it is one vote, spread where
//! the bin points. Votes add up over bins and frames into a histogram over
//! the whole circle: one peak per talker (DUET-style), and for a lone talker
//! two, mirrored front to back when the HRTF cannot tell them apart.
//!
//! The same smoothed cross-spectrum gives the dominant delay between the
//! ears (GCC-PHAT), for the mono mix.

use std::f32::consts::{FRAC_1_SQRT_2, PI, TAU};

use realfft::num_complex::Complex32;

use crate::hrtf::Templates;
use crate::mix::MAX_LAG;
use crate::stft::Spectra;

#[derive(Clone, Copy, Debug)]
pub struct DoaParams {
    /// Coherence a bin needs (0..1).
    pub coherence: f32,
    /// How far a bin may stand from a template and still count (radians of
    /// phase, dB of level).
    pub sigma_ipd: f32,
    pub sigma_ild_db: f32,
    /// A bin counts this far over its own noise floor (dB).
    pub snr_db: f32,
    /// The cross-spectra's smoothing pole per frame (0: none).
    pub smooth: f32,
    /// The floor rises this much a frame (a factor) and falls at once.
    pub floor_rise: f32,
}

impl Default for DoaParams {
    fn default() -> Self {
        DoaParams { coherence: 0.85, sigma_ipd: 0.6, sigma_ild_db: 3.0, snr_db: 6.0, smooth: 0.6, floor_rise: 1.005 }
    }
}

/// One frame's votes over the templates' directions (unnormalised: a vote
/// per bin kept).
#[derive(Clone, Debug, Default)]
pub struct FrameDoa {
    pub votes: Vec<f32>,
    pub bins: usize,
}

pub struct Doa {
    pub templates: Templates,
    pub params: DoaParams,
    s_ll: Vec<f32>,
    s_rr: Vec<f32>,
    s_lr: Vec<Complex32>,
    floor: Vec<f32>,
    /// A frame's distances to (then likelihoods of) each direction.
    like: Vec<f32>,
    /// The templates bin by bin (all directions of a bin side by side):
    /// phase (radians) and level (dB).
    ipd: Vec<f32>,
    ild: Vec<f32>,
    /// e^(i w lag) per whole lag in -(MAX_LAG + 1)..=MAX_LAG + 1, per bin
    /// (lag-major): GCC-PHAT without a sine and a cosine per lag and bin.
    phasors: Vec<Complex32>,
}

/// A direction this much further (in the likelihood's exponent) than the
/// nearest gets no share of the vote: e^-30 of the nearest's is nothing.
const FAR: f32 = 30.0;
/// A bin whose nearest direction is this far (e^-69 ~ 1e-30) is no vote.
const NOWHERE: f32 = 69.0;

impl Doa {
    pub fn new(templates: Templates, params: DoaParams) -> Doa {
        let nb = templates.width();
        let nd = templates.len();
        let (mut ipd, mut ild) = (vec![0.0; nb * nd], vec![0.0; nb * nd]);
        for d in 0..nd {
            let (p, l) = templates.of(d);
            for b in 0..nb {
                (ipd[b * nd + d], ild[b * nd + d]) = (p[b], l[b]);
            }
        }
        let phasors = (-(MAX_LAG + 1)..=MAX_LAG + 1)
            .flat_map(|lag| templates.bins.clone().map(move |k| Complex32::from_polar(1.0, bin_w(k) * lag as f32)))
            .collect();
        Doa {
            templates,
            params,
            s_ll: vec![0.0; nb],
            s_rr: vec![0.0; nb],
            s_lr: vec![Complex32::new(0.0, 0.0); nb],
            floor: vec![f32::MAX; nb],
            like: vec![0.0; nd],
            ipd,
            ild,
            phasors,
        }
    }

    /// Takes a frame: the statistics always, and with `measure` its votes.
    pub fn frame(&mut self, s: &Spectra, measure: bool) -> Option<FrameDoa> {
        let p = self.params;
        let bins = self.templates.bins.clone();
        let a = p.smooth;
        let snr = 10f32.powf(p.snr_db / 10.0);
        let mut votes = if measure { vec![0.0f32; self.templates.len()] } else { Vec::new() };
        let mut kept = 0;
        for (b, k) in bins.enumerate() {
            let (l, r) = (s.left[k], s.right[k]);
            let (pl, pr) = (l.norm_sqr(), r.norm_sqr());
            self.s_ll[b] = a * self.s_ll[b] + (1.0 - a) * pl;
            self.s_rr[b] = a * self.s_rr[b] + (1.0 - a) * pr;
            self.s_lr[b] = self.s_lr[b] * a + l * r.conj() * (1.0 - a);
            let power = pl + pr;
            let loud = power > self.floor[b] * snr;
            self.floor[b] = if power < self.floor[b] { power.max(1e-20) } else { self.floor[b] * p.floor_rise };
            if !measure || !loud || pl <= 0.0 || pr <= 0.0 {
                continue;
            }
            let coherence = self.s_lr[b].norm_sqr() / (self.s_ll[b] * self.s_rr[b]).max(1e-30);
            if coherence < p.coherence {
                continue;
            }
            let phase = (l * r.conj()).arg();
            let level = 10.0 * (pl / pr).log10();
            // exp(-0.5 ((dphase / sigma)^2 + (dlevel / sigma)^2)), the
            // halves and the sigmas folded into two factors.
            let (kp, kl) = (FRAC_1_SQRT_2 / p.sigma_ipd, FRAC_1_SQRT_2 / p.sigma_ild_db);
            let nd = self.like.len();
            let (ipd, ild) = (&self.ipd[b * nd..(b + 1) * nd], &self.ild[b * nd..(b + 1) * nd]);
            let mut nearest = f32::MAX;
            for ((e, &tp), &tl) in self.like.iter_mut().zip(ipd).zip(ild) {
                // Both phases are within -pi..pi: one turn wraps the difference.
                let mut dp = phase - tp;
                if dp >= PI {
                    dp -= TAU;
                } else if dp < -PI {
                    dp += TAU;
                }
                let (x, y) = (dp * kp, (level - tl) * kl);
                *e = x * x + y * y;
                nearest = nearest.min(*e);
            }
            if nearest > NOWHERE {
                continue;
            }
            // Relative to the nearest (the share is the same): the far
            // directions need no exponential.
            let mut sum = 0.0;
            for e in self.like.iter_mut() {
                let d = *e - nearest;
                *e = if d < FAR { (-d).exp() } else { 0.0 };
                sum += *e;
            }
            let share = 1.0 / sum;
            for (v, x) in votes.iter_mut().zip(&self.like) {
                *v += x * share;
            }
            kept += 1;
        }
        measure.then_some(FrameDoa { votes, bins: kept })
    }

    /// The delay between the ears of whatever dominates now (samples, +
    /// when the left ear lags), and how sure (0..1): GCC-PHAT over the bins
    /// standing out of their floor, refined between samples.
    pub fn lag(&self, max_lag: i32) -> Option<(f32, f32)> {
        let snr = 10f32.powf(self.params.snr_db / 10.0);
        let used: Vec<(usize, Complex32)> = (0..self.templates.width())
            .filter(|&b| self.s_ll[b] + self.s_rr[b] > self.floor[b] * snr && self.s_lr[b].norm() > 0.0)
            .map(|b| (b, self.s_lr[b] / self.s_lr[b].norm()))
            .collect();
        if used.len() < 8 {
            return None;
        }
        let (nb, first) = (self.templates.width(), self.templates.bins.start);
        let at = |lag: i32| {
            let sum: f32 = if lag.abs() <= MAX_LAG + 1 {
                let row = &self.phasors[(lag + MAX_LAG + 1) as usize * nb..][..nb];
                used.iter().map(|&(b, x)| x.re * row[b].re - x.im * row[b].im).sum()
            } else {
                used.iter().map(|&(b, x)| (x * Complex32::from_polar(1.0, bin_w(first + b) * lag as f32)).re).sum()
            };
            sum / used.len() as f32
        };
        let (best, peak) = (-max_lag..=max_lag).map(|l| (l, at(l))).max_by(|a, b| a.1.total_cmp(&b.1))?;
        let (lo, hi) = (at(best - 1), at(best + 1));
        let den = lo - 2.0 * peak + hi;
        let shift = if den < 0.0 { (0.5 * (lo - hi) / den).clamp(-0.5, 0.5) } else { 0.0 };
        Some((best as f32 + shift, peak.max(0.0)))
    }
}

/// Bin `k`'s angular frequency (radians a sample).
fn bin_w(k: usize) -> f32 {
    2.0 * PI * k as f32 / crate::WINDOW as f32
}

/// A histogram over the templates' azimuths (or anything on an evenly
/// spaced ring), normalised to sum 1.
#[derive(Clone, Debug, PartialEq)]
pub struct Ring {
    /// Bin `i` is centred on `-180 + i * step`.
    pub p: Vec<f32>,
    pub step: f32,
}

impl Ring {
    pub fn zeros(n: usize) -> Ring {
        Ring { p: vec![0.0; n], step: 360.0 / n as f32 }
    }

    pub fn total(&self) -> f32 {
        self.p.iter().sum()
    }

    /// Adds `votes` (on the same ring) turned by `turn` degrees (a head
    /// turned that way: head-relative votes into a fixed frame), shared
    /// between the two nearest bins.
    pub fn add_turned(&mut self, votes: &[f32], turn: f32, weight: f32) {
        let n = self.p.len();
        debug_assert_eq!(votes.len(), n);
        let shift = turn / self.step;
        let (whole, frac) = (shift.floor(), shift - shift.floor());
        let whole = whole as i64;
        for (i, &v) in votes.iter().enumerate() {
            let j = (i as i64 + whole).rem_euclid(n as i64) as usize;
            self.p[j] += v * weight * (1.0 - frac);
            self.p[(j + 1) % n] += v * weight * frac;
        }
    }

    pub fn normalised(&self) -> Ring {
        let t = self.total();
        Ring { p: self.p.iter().map(|x| if t > 0.0 { x / t } else { 0.0 }).collect(), step: self.step }
    }

    pub fn angle(&self, i: usize) -> f32 {
        -180.0 + i as f32 * self.step
    }

    /// The share within `half` degrees of `deg`.
    pub fn mass_near(&self, deg: f32, half: f32) -> f32 {
        let t = self.total();
        if t <= 0.0 {
            return 0.0;
        }
        (0..self.p.len()).filter(|&i| crate::wrap_deg(self.angle(i) - deg).abs() <= half + 1e-3).map(|i| self.p[i]).sum::<f32>() / t
    }

    /// The best `mass_near` anywhere (and where).
    pub fn best_mass(&self, half: f32) -> (f32, f32) {
        (0..self.p.len()).map(|i| (self.angle(i), self.mass_near(self.angle(i), half))).max_by(|a, b| a.1.total_cmp(&b.1)).unwrap_or((0.0, 0.0))
    }

    /// Local maxima, highest first: (angle refined between bins, share
    /// within `half` degrees), at most `max`, each at least `apart` from
    /// a higher one.
    pub fn peaks(&self, half: f32, apart: f32, max: usize) -> Vec<(f32, f32)> {
        let n = self.p.len();
        let mut found: Vec<(f32, f32)> = Vec::new();
        let mut order: Vec<usize> = (0..n).filter(|&i| self.p[i] > 0.0 && self.p[i] >= self.p[(i + n - 1) % n] && self.p[i] >= self.p[(i + 1) % n]).collect();
        order.sort_by(|&a, &b| self.p[b].total_cmp(&self.p[a]));
        for i in order {
            let (lo, mid, hi) = (self.p[(i + n - 1) % n], self.p[i], self.p[(i + 1) % n]);
            let den = lo - 2.0 * mid + hi;
            let shift = if den < 0.0 { (0.5 * (lo - hi) / den).clamp(-0.5, 0.5) } else { 0.0 };
            let at = crate::wrap_deg(self.angle(i) + shift * self.step);
            if found.iter().any(|f| crate::wrap_deg(f.0 - at).abs() < apart) {
                continue;
            }
            found.push((at, self.mass_near(at, half)));
            if found.len() == max {
                break;
            }
        }
        found
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hrtf::{render, Hrtf, SphericalHead};
    use crate::stft::Stft;
    use crate::{hz_bin, mirror_deg, wrap_deg, HOP};

    /// Speech-like noise: white noise shaped by a slow random envelope.
    fn babble(n: usize, seed: u32) -> Vec<f32> {
        let mut s = seed;
        let mut rnd = move || {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (s >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
        };
        let mut env = 0.0f32;
        (0..n)
            .map(|i| {
                if i % 2400 == 0 {
                    env = 0.2 + 0.8 * rnd().abs();
                }
                rnd() * env * 0.1
            })
            .collect()
    }

    fn histogram(head: &dyn Hrtf, ears: &(Vec<f32>, Vec<f32>)) -> Ring {
        let t = Templates::build(head, 2.0, 0.0, hz_bin(200.0)..hz_bin(6000.0));
        let mut doa = Doa::new(t, DoaParams::default());
        let mut stft = Stft::new();
        let mut ring = Ring::zeros(doa.templates.len());
        for (l, r) in ears.0.chunks_exact(HOP).zip(ears.1.chunks_exact(HOP)) {
            if let Some(f) = doa.frame(&stft.push(l, r), true) {
                ring.add_turned(&f.votes, 0.0, 1.0);
            }
        }
        ring
    }

    fn mix(a: &(Vec<f32>, Vec<f32>), b: &(Vec<f32>, Vec<f32>)) -> (Vec<f32>, Vec<f32>) {
        (a.0.iter().zip(&b.0).map(|(x, y)| x + y).collect(), a.1.iter().zip(&b.1).map(|(x, y)| x + y).collect())
    }

    /// The nearer of `az` and its mirror to the found peak, and how far.
    fn off(found: f32, az: f32) -> f32 {
        wrap_deg(found - az).abs().min(wrap_deg(found - mirror_deg(az)).abs())
    }

    #[test]
    fn one_talker_is_found_within_a_few_degrees() {
        let head = SphericalHead::default();
        let voice = babble(48_000, 7);
        for az in [-70.0f32, -24.0, 0.0, 16.0, 50.0, 130.0] {
            let ring = histogram(&head, &render(&head, az, 0.0, &voice));
            let peaks = ring.peaks(6.0, 20.0, 2);
            assert!(off(peaks[0].0, az) <= 3.0, "az {az}: {peaks:?}");
            // The sphere cannot tell front from back: both carry the vote.
            if (wrap_deg(az).abs() - 90.0).abs() > 30.0 {
                assert!(off(peaks[1].0, az) <= 3.0, "az {az}: {peaks:?}");
            }
        }
    }

    /// The table rendered from Steam Audio (the repo's asset), if there.
    fn steam() -> Option<crate::HrirTable> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/hrtf/steam-default-48k.bin");
        crate::HrirTable::read(&path).ok()
    }

    #[test]
    fn steam_audios_head_finds_one_talker_and_mostly_its_side() {
        let Some(head) = steam() else {
            eprintln!("no assets/hrtf/steam-default-48k.bin: skipped");
            return;
        };
        let voice = babble(48_000, 9);
        for az in [-150.0f32, -100.0, -45.0, -10.0, 0.0, 20.0, 60.0, 120.0, 170.0] {
            let ring = histogram(&head, &render(&head, az, 0.0, &voice));
            let peaks = ring.peaks(6.0, 20.0, 2);
            assert!(off(peaks[0].0, az) <= 3.0, "az {az}: {peaks:?}");
            // Its pinnae favour the true side over the mirror (measured:
            // 1.7 to 6 times the votes on clean speech-like noise).
            let (here, there) = (ring.mass_near(az, 6.0), ring.mass_near(mirror_deg(az), 6.0));
            assert!(here >= there * 1.5, "az {az}: {here} vs mirror {there}");
        }
    }

    #[test]
    fn two_talkers_give_two_peaks() {
        let head = SphericalHead::default();
        let a = render(&head, -40.0, 0.0, &babble(48_000, 1));
        let b = render(&head, 35.0, 0.0, &babble(48_000, 2));
        let ring = histogram(&head, &mix(&a, &b));
        let peaks = ring.peaks(6.0, 15.0, 4);
        for az in [-40.0, 35.0] {
            assert!(peaks.iter().any(|p| off(p.0, az) <= 4.0), "{az}: {peaks:?}");
        }
    }

    #[test]
    fn noise_in_each_ear_does_not_move_the_peak() {
        let head = SphericalHead::default();
        let ears = render(&head, 28.0, 0.0, &babble(48_000, 3));
        let noise = (babble(48_000, 11).iter().map(|x| x * 0.3).collect(), babble(48_000, 12).iter().map(|x| x * 0.3).collect());
        let ring = histogram(&head, &mix(&ears, &noise));
        let p = ring.peaks(6.0, 20.0, 1)[0];
        assert!(off(p.0, 28.0) <= 4.0, "{p:?}");
    }

    #[test]
    fn the_lag_is_the_itd() {
        let head = SphericalHead::default();
        let t = Templates::build(&head, 2.0, 0.0, hz_bin(200.0)..hz_bin(6000.0));
        let mut doa = Doa::new(t, DoaParams::default());
        let mut stft = Stft::new();
        let (l, r) = render(&head, 60.0, 0.0, &babble(24_000, 5));
        for (l, r) in l.chunks_exact(HOP).zip(r.chunks_exact(HOP)) {
            doa.frame(&stft.push(l, r), false);
        }
        let (lag, sure) = doa.lag(48).unwrap();
        let want = head.itd_s(60.0, 0.0) * 48_000.0;
        assert!((lag - want).abs() < 1.0 && sure > 0.5, "{lag} vs {want} ({sure})");
    }

    #[test]
    fn a_turned_ring_moves_its_votes() {
        let mut r = Ring::zeros(180);
        let mut v = vec![0.0; 180];
        v[90] = 1.0; // 0 degrees
        r.add_turned(&v, 31.0, 1.0);
        let p = r.peaks(1.0, 1.0, 1)[0];
        assert!((p.0 - 31.0).abs() < 1.1, "{p:?}");
        assert!((r.mass_near(31.0, 2.0) - 1.0).abs() < 1e-5);
    }
}
