//! The mono mix sent on to the client (ASR and speaker embeddings).
//!
//! A plain sum of the ears notches a side voice: its ears differ by up to
//! ~0.7 ms, so L + R cancels at 1/(2 ITD) (~700 Hz) and its odd multiples.
//! So the ears are aligned on the dominant talker's delay first (the lagging
//! ear taken as is, the leading one delayed), and summed. With no clear
//! delay (several talkers, diffuse sound) for a while, the louder ear alone.
//! A change of either crossfades over one block: no clicks.

use std::collections::VecDeque;

/// Delays beyond this (samples) are not a head's (1 ms at 48 kHz).
pub const MAX_LAG: i32 = 48;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MixTarget {
    /// Both ears, the leading one delayed by the lag (samples, + when the
    /// left ear lags).
    Aligned(i32),
    /// One ear alone (true: left).
    Ear(bool),
}

pub struct MonoMix {
    /// The last MAX_LAG samples of each ear.
    hist_l: Vec<f32>,
    hist_r: Vec<f32>,
    current: MixTarget,
}

impl Default for MonoMix {
    fn default() -> Self {
        MonoMix { hist_l: vec![0.0; MAX_LAG as usize], hist_r: vec![0.0; MAX_LAG as usize], current: MixTarget::Aligned(0) }
    }
}

impl MonoMix {
    pub fn current(&self) -> MixTarget {
        self.current
    }

    /// One block of both ears into 16-bit mono, mixed as `target` asks
    /// (crossfaded from the last block's way).
    pub fn process(&mut self, left: &[f32], right: &[f32], target: MixTarget) -> Vec<i16> {
        let m = MAX_LAG as usize;
        let ext = |hist: &[f32], new: &[f32]| hist.iter().chain(new).copied().collect::<Vec<f32>>();
        let (el, er) = (ext(&self.hist_l, left), ext(&self.hist_r, right));
        let sample = |t: MixTarget, n: usize| match t {
            MixTarget::Aligned(lag) => {
                let lag = lag.clamp(-MAX_LAG, MAX_LAG);
                let (dl, dr) = (lag.min(0).unsigned_abs() as usize, lag.max(0) as usize);
                0.5 * (el[m + n - dl] + er[m + n - dr])
            }
            MixTarget::Ear(true) => el[m + n],
            MixTarget::Ear(false) => er[m + n],
        };
        let len = left.len();
        let out = (0..len)
            .map(|n| {
                let x = if target == self.current {
                    sample(target, n)
                } else {
                    let w = 0.5 - 0.5 * (std::f32::consts::PI * (n as f32 + 0.5) / len as f32).cos();
                    (1.0 - w) * sample(self.current, n) + w * sample(target, n)
                };
                (x * 32767.0).round().clamp(-32768.0, 32767.0) as i16
            })
            .collect();
        self.hist_l = el[el.len() - m..].to_vec();
        self.hist_r = er[er.len() - m..].to_vec();
        self.current = target;
        out
    }
}

/// Which way to mix, from each frame's delay estimate.
pub struct MixPolicy {
    /// GCC-PHAT strength a delay needs to be believed.
    pub min_strength: f32,
    /// Voiced frames in a row with no believable delay before the louder
    /// ear is used alone.
    pub ear_after: u32,
    /// The other ear must be louder by this much (dB) to switch ears.
    pub ear_switch_db: f32,
    lags: VecDeque<f32>,
    unsure: u32,
    target: MixTarget,
}

impl Default for MixPolicy {
    fn default() -> Self {
        MixPolicy { min_strength: 0.4, ear_after: 50, ear_switch_db: 3.0, lags: VecDeque::new(), unsure: 0, target: MixTarget::Aligned(0) }
    }
}

impl MixPolicy {
    pub fn target(&self) -> MixTarget {
        self.target
    }

    /// One frame: whether it is voiced, the delay estimate (samples,
    /// strength) and the ears' recent power. Silence keeps the last way.
    pub fn update(&mut self, voiced: bool, lag: Option<(f32, f32)>, power_l: f32, power_r: f32) -> MixTarget {
        if !voiced {
            return self.target;
        }
        match lag.filter(|l| l.1 >= self.min_strength && l.0.abs() <= MAX_LAG as f32) {
            Some((lag, _)) => {
                self.unsure = 0;
                self.lags.push_back(lag);
                if self.lags.len() > 5 {
                    self.lags.pop_front();
                }
                let mut sorted: Vec<f32> = self.lags.iter().copied().collect();
                sorted.sort_by(f32::total_cmp);
                let median = sorted[sorted.len() / 2];
                let keep = matches!(self.target, MixTarget::Aligned(d) if (median - d as f32).abs() < 1.0);
                if !keep {
                    self.target = MixTarget::Aligned(median.round() as i32);
                }
            }
            None => {
                self.unsure += 1;
                if self.unsure > self.ear_after {
                    let db = 10.0 * (power_l.max(1e-20) / power_r.max(1e-20)).log10();
                    self.target = match self.target {
                        MixTarget::Ear(true) if db > -self.ear_switch_db => MixTarget::Ear(true),
                        MixTarget::Ear(false) if db < self.ear_switch_db => MixTarget::Ear(false),
                        _ => MixTarget::Ear(db >= 0.0),
                    };
                }
            }
        }
        self.target
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(n: usize, hz: f32, delay: f32) -> Vec<f32> {
        (0..n).map(|i| 0.5 * (2.0 * std::f32::consts::PI * hz * (i as f32 - delay) / 48_000.0).sin()).collect()
    }

    fn rms(x: &[i16]) -> f32 {
        (x.iter().map(|&v| (v as f32 / 32767.0).powi(2)).sum::<f32>() / x.len() as f32).sqrt()
    }

    #[test]
    fn aligning_undoes_the_comb_notch() {
        // A side voice: the left ear 31 samples late; 774 Hz is the notch.
        let (l, r) = (tone(4800, 774.0, 31.0), tone(4800, 774.0, 0.0));
        let mut plain = MonoMix::default();
        let mut aligned = MonoMix::default();
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for (lb, rb) in l.chunks(960).zip(r.chunks(960)) {
            a.extend(plain.process(lb, rb, MixTarget::Aligned(0)));
            b.extend(aligned.process(lb, rb, MixTarget::Aligned(31)));
        }
        assert!(rms(&a[960..]) < 0.05, "{}", rms(&a[960..]));
        assert!((rms(&b[960..]) - 0.354).abs() < 0.01, "{}", rms(&b[960..]));
    }

    #[test]
    fn switching_crossfades_without_a_click() {
        let (l, r) = (tone(1920, 300.0, 10.0), tone(1920, 300.0, 0.0));
        let mut m = MonoMix::default();
        let mut out = m.process(&l[..960], &r[..960], MixTarget::Aligned(10));
        out.extend(m.process(&l[960..], &r[960..], MixTarget::Ear(false)));
        // The biggest step between samples stays a sine's.
        let step = out.windows(2).map(|w| (w[1] as i32 - w[0] as i32).abs()).max().unwrap();
        let sine_step = (0.5 * 32767.0 * 2.0 * std::f32::consts::PI * 300.0 / 48_000.0) as i32;
        assert!(step <= sine_step + 30, "{step} vs {sine_step}");
    }

    #[test]
    fn the_policy_follows_believable_delays_and_falls_back_to_an_ear() {
        let mut p = MixPolicy::default();
        for _ in 0..5 {
            p.update(true, Some((20.2, 0.9)), 1.0, 1.0);
        }
        assert_eq!(p.target(), MixTarget::Aligned(20));
        p.update(true, Some((20.6, 0.9)), 1.0, 1.0);
        assert_eq!(p.target(), MixTarget::Aligned(20)); // within a sample: kept
        p.update(false, None, 1.0, 1.0);
        for _ in 0..60 {
            p.update(true, Some((3.0, 0.1)), 4.0, 1.0);
        }
        assert_eq!(p.target(), MixTarget::Ear(true));
        p.update(true, None, 1.0, 1.5); // not louder by 3 dB: stays
        assert_eq!(p.target(), MixTarget::Ear(true));
    }
}
