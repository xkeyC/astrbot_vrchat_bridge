//! Speech or not, per frame, without a model: the frame's level over a
//! tracked noise floor, and how much of it lies in the speech band, with
//! hysteresis (a few frames to start, a hangover to end).

use crate::hz_bin;
use crate::stft::{full_scale_power, Spectra};

#[derive(Clone, Copy, Debug)]
pub struct ActivityParams {
    /// Above the floor by this much (dB) a frame is voiced...
    pub on_db: f32,
    /// ...and at least this loud (dB re full scale), with at least this
    /// share of its energy (50-12000 Hz) in the voice's 150-4000 Hz (white
    /// noise has a third there, a voice nearly all).
    pub min_db: f32,
    pub speech_share: f32,
    /// Voiced frames in a row to start; unvoiced frames in a row to end.
    pub start_frames: u32,
    pub hangover_frames: u32,
    /// How fast the floor rises toward the level (dB a frame) while quiet,
    /// and while voiced; it falls at once.
    pub rise_quiet_db: f32,
    pub rise_voiced_db: f32,
}

impl Default for ActivityParams {
    fn default() -> Self {
        ActivityParams { on_db: 9.0, min_db: -60.0, speech_share: 0.6, start_frames: 3, hangover_frames: 25, rise_quiet_db: 0.02, rise_voiced_db: 0.003 }
    }
}

/// One frame as the detector saw it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Level {
    /// Both ears, 50-12000 Hz (dB re a full-scale sine).
    pub db: f32,
    pub floor_db: f32,
    /// The share in 150-4000 Hz.
    pub speech_share: f32,
    /// This frame alone looks like speech.
    pub voiced: bool,
    /// Speech is on (started, not yet ended).
    pub active: bool,
}

pub struct Activity {
    pub params: ActivityParams,
    floor_db: f32,
    on: bool,
    run: u32,
    quiet: u32,
}

impl Activity {
    pub fn new(params: ActivityParams) -> Activity {
        Activity { params, floor_db: params.min_db - params.on_db, on: false, run: 0, quiet: 0 }
    }

    pub fn update(&mut self, s: &Spectra) -> Level {
        let p = self.params;
        let power = |k: usize| s.left[k].norm_sqr() + s.right[k].norm_sqr();
        let band = |lo: f32, hi: f32| (hz_bin(lo)..hz_bin(hi).min(s.left.len())).map(power).sum::<f32>();
        let all = band(50.0, 12000.0);
        let speech = band(150.0, 4000.0);
        let db = 10.0 * (all / full_scale_power()).max(1e-12).log10();
        let share = if all > 0.0 { speech / all } else { 0.0 };
        let voiced = db > self.floor_db + p.on_db && db > p.min_db && share >= p.speech_share;
        // The floor: down at once, up slowly (slower still under speech).
        if db < self.floor_db {
            self.floor_db = db.max(-120.0);
        } else {
            let rise = if voiced { p.rise_voiced_db } else { p.rise_quiet_db };
            self.floor_db = (self.floor_db + rise).min(db);
        }
        if voiced {
            self.run += 1;
            self.quiet = 0;
            if self.run >= p.start_frames {
                self.on = true;
            }
        } else {
            self.run = 0;
            self.quiet += 1;
            if self.quiet > p.hangover_frames {
                self.on = false;
            }
        }
        Level { db, floor_db: self.floor_db, speech_share: share, voiced, active: self.on }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stft::Stft;
    use crate::HOP;

    /// A vowel-like buzz: harmonics of 150 Hz up to 3 kHz.
    pub(crate) fn buzz(n: usize, amp: f32) -> Vec<f32> {
        (0..n)
            .map(|i| {
                let t = i as f32 / 48_000.0;
                (1..=20).map(|h| (2.0 * std::f32::consts::PI * 150.0 * h as f32 * t).sin() / h as f32).sum::<f32>() * amp
            })
            .collect()
    }

    #[test]
    fn speech_turns_it_on_and_quiet_off() {
        let mut a = Activity::new(ActivityParams::default());
        let mut stft = Stft::new();
        let mut noise = 12345u32;
        let mut signal: Vec<f32> = (0..48_000)
            .map(|_| {
                noise = noise.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (noise >> 8) as f32 / (1u32 << 24) as f32 * 2e-3 - 1e-3
            })
            .collect();
        let speech = buzz(24_000, 0.1);
        for (x, s) in signal[12_000..36_000].iter_mut().zip(speech) {
            *x += s;
        }
        let mut on_frames = Vec::new();
        for (i, hop) in signal.chunks_exact(HOP).enumerate() {
            let l = a.update(&stft.push(hop, hop));
            if l.active {
                on_frames.push(i);
            }
        }
        // Speech is hops 25..75; on from a few hops in, off a hangover after.
        let (first, last) = (on_frames[0], *on_frames.last().unwrap());
        assert!((26..=29).contains(&first), "{first}");
        assert!((75 + 24..=75 + 28).contains(&last), "{last}");
    }
}
