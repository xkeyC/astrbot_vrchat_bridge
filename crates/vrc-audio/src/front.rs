//! The whole front end on the capture's 20 ms blocks: the mono mix to send
//! on, then per 10 ms hop the spectra, the speech detector, the votes on
//! the direction and the segments on the client's clock.
//!
//! A block is mixed first (its way chosen from the hops before it), sent or
//! dropped by the caller, then analysed with where it landed on the clock.

use crate::activity::{Activity, ActivityParams, Level};
use crate::doa::{Doa, DoaParams, FrameDoa};
use crate::hrtf::{Hrtf, Templates};
use crate::mix::{MixPolicy, MonoMix, MAX_LAG};
use crate::segment::{SegEvent, Segmenter};
use crate::stft::Stft;
use crate::{hz_bin, HOP};

#[derive(Clone, Copy, Debug)]
pub struct FrontParams {
    pub activity: ActivityParams,
    pub doa: DoaParams,
    /// The bins the direction finder uses.
    pub lo_hz: f32,
    pub hi_hz: f32,
    /// The templates' azimuth step and elevation (degrees).
    pub step: f32,
    pub elevation: f32,
}

impl Default for FrontParams {
    fn default() -> Self {
        FrontParams { activity: ActivityParams::default(), doa: DoaParams::default(), lo_hz: 200.0, hi_hz: 6000.0, step: 2.0, elevation: 0.0 }
    }
}

/// A block of both ears (`BLOCK` samples each).
pub struct Block<'a> {
    pub left: &'a [f32],
    pub right: &'a [f32],
}

/// One hop, analysed.
#[derive(Clone, Debug)]
pub struct HopOut {
    pub level: Level,
    /// The votes on the direction (voiced hops, when measuring).
    pub doa: Option<FrameDoa>,
    /// Where the hop lies on the client's clock.
    pub start: u64,
    pub end: u64,
    pub events: Vec<SegEvent>,
    /// The ears' dominant delay now (samples, + left lags; strength).
    pub lag: Option<(f32, f32)>,
}

pub struct Front {
    stft: Stft,
    pub activity: Activity,
    pub doa: Doa,
    pub segmenter: Segmenter,
    mix: MonoMix,
    pub policy: MixPolicy,
    power: [f32; 2],
    /// Vote on directions (off: the detector and the mix only).
    pub measure: bool,
}

impl Front {
    pub fn new(head: &dyn Hrtf, params: FrontParams) -> Front {
        let templates = Templates::build(head, params.step, params.elevation, hz_bin(params.lo_hz)..hz_bin(params.hi_hz));
        Front {
            stft: Stft::new(),
            activity: Activity::new(params.activity),
            doa: Doa::new(templates, params.doa),
            segmenter: Segmenter::new(params.activity.start_frames),
            mix: MonoMix::default(),
            policy: MixPolicy::default(),
            power: [0.0; 2],
            measure: true,
        }
    }

    /// The block as 16-bit mono, the way the hops so far chose.
    pub fn mix(&mut self, b: &Block) -> Vec<i16> {
        let target = self.policy.target();
        self.mix.process(b.left, b.right, target)
    }

    /// Analyses the block: it starts at `clock` on the client's clock, and
    /// `counted` says whether it reached the client (else it takes no time).
    pub fn analyse(&mut self, b: &Block, clock: u64, counted: bool) -> Vec<HopOut> {
        let mut out = Vec::with_capacity(2);
        for (i, (l, r)) in b.left.chunks_exact(HOP).zip(b.right.chunks_exact(HOP)).enumerate() {
            let s = self.stft.push(l, r);
            let level = self.activity.update(&s);
            let doa = self.doa.frame(&s, self.measure && level.voiced);
            let pl: f32 = l.iter().map(|x| x * x).sum();
            let pr: f32 = r.iter().map(|x| x * x).sum();
            self.power = [0.9 * self.power[0] + 0.1 * pl, 0.9 * self.power[1] + 0.1 * pr];
            let lag = if level.voiced { self.doa.lag(MAX_LAG) } else { None };
            self.policy.update(level.voiced, lag, self.power[0], self.power[1]);
            let (start, end) = if counted { (clock + (i * HOP) as u64, clock + ((i + 1) * HOP) as u64) } else { (clock, clock) };
            let events = self.segmenter.push(&level, start, end);
            out.push(HopOut { level, doa, start, end, events, lag });
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hrtf::{render, SphericalHead};
    use crate::mix::MixTarget;
    use crate::BLOCK;

    #[test]
    fn a_side_talker_is_segmented_and_mixed_aligned() {
        let head = SphericalHead::default();
        let mut front = Front::new(&head, FrontParams::default());
        // 0.5 s quiet, 1 s of a buzz from 70 degrees right, 0.6 s quiet.
        let mut mono = vec![0.0f32; 24_000];
        mono.extend((0..48_000).map(|i| {
            let t = i as f32 / 48_000.0;
            (1..=20).map(|h| (2.0 * std::f32::consts::PI * 140.0 * h as f32 * t).sin() / h as f32).sum::<f32>() * 0.05
        }));
        mono.extend(vec![0.0; 28_800]);
        let (l, r) = render(&head, 70.0, 0.0, &mono);
        let mut events = Vec::new();
        let mut clock = 0u64;
        for (k, (lb, rb)) in l.chunks_exact(BLOCK).zip(r.chunks_exact(BLOCK)).enumerate() {
            let b = Block { left: lb, right: rb };
            front.mix(&b);
            // Blocks 10..15 are dropped by a full queue.
            let counted = !(10..15).contains(&k);
            for h in front.analyse(&b, clock, counted) {
                events.extend(h.events);
            }
            if counted {
                clock += BLOCK as u64;
            }
        }
        let want_lag = (head.itd_s(70.0, 0.0) * 48_000.0).round() as i32;
        assert!(matches!(front.policy.target(), MixTarget::Aligned(d) if (d - want_lag).abs() <= 1), "{:?} vs {want_lag}", front.policy.target());
        // Speech from 24000 (the 5 dropped blocks were before it: -4800).
        let closed: Vec<_> = events.iter().filter_map(|e| if let SegEvent::Closed { start, end, .. } = e { Some((*start, *end)) } else { None }).collect();
        assert_eq!(closed.len(), 1, "{events:?}");
        let (start, end) = closed[0];
        assert!((start as i64 - 19_200).abs() <= 960, "{start}");
        assert!((end as i64 - (19_200 + 48_000)).abs() <= 3_400, "{end}");
    }

    /// A rough timing of the analysis on speech (Steam Audio's table when
    /// the repo's asset is there): `cargo test --release -p vrc-audio --
    /// --ignored --nocapture timing`.
    #[test]
    #[ignore]
    fn timing() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/hrtf/steam-default-48k.bin");
        let head: Box<dyn Hrtf> = match crate::HrirTable::read(&path) {
            Ok(t) => Box::new(t),
            Err(_) => Box::new(SphericalHead::default()),
        };
        let mut front = Front::new(head.as_ref(), FrontParams::default());
        let mono: Vec<f32> = (0..48_000 * 5)
            .map(|i| {
                let t = i as f32 / 48_000.0;
                (1..=20).map(|h| (2.0 * std::f32::consts::PI * 140.0 * h as f32 * t).sin() / h as f32).sum::<f32>() * 0.05
            })
            .collect();
        let (l, r) = render(head.as_ref(), 40.0, 0.0, &mono);
        let (mut blocks, mut voiced) = (0u32, 0u32);
        let t = std::time::Instant::now();
        for (lb, rb) in l.chunks_exact(BLOCK).zip(r.chunks_exact(BLOCK)) {
            let b = Block { left: lb, right: rb };
            front.mix(&b);
            voiced += front.analyse(&b, 0, true).iter().filter(|h| h.doa.is_some()).count() as u32;
            blocks += 1;
        }
        let per = t.elapsed().as_secs_f64() * 1e6 / blocks as f64;
        eprintln!("{}: {per:.0} us a 20 ms block ({voiced} of {} hops voted)", head.name(), 2 * blocks);
    }
}
