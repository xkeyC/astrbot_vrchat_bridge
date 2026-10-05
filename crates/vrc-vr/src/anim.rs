//! Procedural life for the avatar: VRChat's IK puts the arms where the
//! controllers are and plays the legs when the thumbstick walks, nothing
//! more; left alone the bot stands frozen. Each tick the [`Animator`] works
//! out both hands (and a little of the head) as an [`Overlay`] on top of
//! where its owner points the head and faces the body:
//!
//! - **idle**: breathing, a slow sway of head and hands (smooth noise), a
//!   shift of weight every 8-20 s, a glance aside every 4-10 s when nothing
//!   else drives the head;
//! - **walking**: fitted to CMU motion capture (181 strides of walks and
//!   runs, scaled to a 1.6 m eye height; D25): the arms swing opposite each
//!   other once a stride, about a mean a little ahead, the hand tipping
//!   forward and turning palm-back as it comes forward, rising with it;
//!   running bends the elbows (hands up, fingers ahead) and closes the
//!   fists. The head bobs twice a stride (lowest just after each heel
//!   strike), sways to the standing foot, turns and rolls a little with the
//!   chest, and nods;
//! - **talking**: driven by the loudness of the bot's own voice: the right
//!   hand comes up in front, beats down on accents (a jump in loudness),
//!   the head nods on some of them.
//!
//! Numbers from gait studies and motion capture (see `docs/full-vr` D23);
//! distances are for a 1.6 m eye height and scale with
//! [`crate::remote::EYE_HEIGHT`]. Everything here is pure: the caller
//! supplies time, speed and loudness, and sends the overlay.

use std::f32::consts::PI;

use serde::{Deserialize, Serialize};

use crate::pose::{quat_axis, quat_mul, Pose};
use crate::remote::{Controller, HeadOffset, Overlay, State, EYE_HEIGHT};


/// Tunable at run time (the bridge's `/v1/anim`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AnimParams {
    /// Animate at all (else the owner's rest pose alone).
    pub enabled: bool,
    /// The hands at rest, in eye heights in the body's frame: out to the
    /// side, up, back.
    pub rest: [f32; 3],
    /// The grip's turn at rest (degrees): tipped down (about x), turned
    /// in (about y, mirrored for the left hand), rolled (about the grip's
    /// z, mirrored). Monado's hand simulation runs the fingers along the
    /// grip's -z (not round it, as a real hand on a controller): fingers
    /// down is -z down, and the roll turns the palms in. Tuned by eye in
    /// VRChat (D23).
    pub grip: [f32; 3],
    /// Send finger curls as hand tracking (xrizer passes them on as the
    /// skeleton); else the hands are what the controllers' buttons say.
    pub hand_tracking: bool,
    /// Finger curls at rest: little, ring, middle, index, thumb (0..1).
    pub curl: [f32; 5],
    /// Breaths per second, and the head's rise (m) with each.
    pub breath_hz: f32,
    pub breath_m: f32,
    /// Scale of the idle sway and weight shifts (0: none).
    pub sway: f32,
    /// The arms' own slow swing at rest (m, front to back; scaled by `sway`).
    pub arm_sway_m: f32,
    /// Glances aside when nothing else drives the head.
    pub glances: bool,
    /// Scale of the arm swing when walking (0: none), and the head's bob
    /// (m, half its rise and fall each step, walking; running bobs 3.3
    /// times that).
    pub swing: f32,
    pub bob_m: f32,
    /// Leaning into the walk: the head ahead of the hands (m) and tipped
    /// down (degrees) at a full walk; twice that running. With only head
    /// and hands, VRChat's IK reads the torso's tilt from them.
    pub lean_m: f32,
    pub lean_deg: f32,
    /// The head's nod with each step (degrees either way; up just after
    /// each heel strike). Motion capture has 0.5; more reads better.
    pub nod_deg: f32,
    /// Gestures and nods while talking.
    pub talk: bool,
    /// The head's height above the tracking floor (m): where VRChat's
    /// calibration expects it (higher, the avatar stands on tiptoe; lower,
    /// it bends its knees; 1.56 by eye, D23). Not animation: the owner's,
    /// kept here to be tuned and kept with the rest.
    pub head_height: f32,
}

impl Default for AnimParams {
    fn default() -> Self {
        AnimParams {
            enabled: true,
            rest: [0.155, -0.48, 0.0],
            grip: [85.0, 0.0, -90.0],
            hand_tracking: true,
            curl: [0.40, 0.35, 0.30, 0.25, 0.25],
            breath_hz: 0.24,
            breath_m: 0.004,
            sway: 1.5,
            arm_sway_m: 0.02,
            glances: true,
            swing: 1.0,
            bob_m: 0.018,
            lean_m: 0.03,
            lean_deg: 3.0,
            nod_deg: 1.5,
            talk: true,
            head_height: 1.56,
        }
    }
}

/// The hand on `side` (-1 left, +1 right) at rest under `head` for a body
/// facing `body_yaw` (degrees), moved by `offset` (body frame, tracking
/// metres) and turned by `turn` (body frame, after the rest turn).
pub fn hand_pose(p: &AnimParams, head: [f32; 3], body_yaw: f32, side: f32, offset: [f32; 3], turn: [f32; 4]) -> Pose {
    let body = Pose::looking(body_yaw, 0.0, head);
    let [tip, inward, roll] = p.grip.map(f32::to_radians);
    let rest = quat_mul(
        quat_mul(quat_axis([0.0, 1.0, 0.0], side * inward), quat_axis([1.0, 0.0, 0.0], -tip)),
        quat_axis([0.0, 0.0, 1.0], side * roll),
    );
    let at = [
        side * p.rest[0] * EYE_HEIGHT + offset[0],
        p.rest[1] * EYE_HEIGHT + offset[1],
        p.rest[2] * EYE_HEIGHT + offset[2],
    ];
    let d = body.rotate(at);
    Pose {
        orientation: quat_mul(body.orientation, quat_mul(turn, rest)),
        position: [head[0] + d[0], head[1] + d[1], head[2] + d[2]],
    }
}

/// A controller at `pose` with `curl`ed fingers.
pub fn hand(p: &AnimParams, pose: Pose, curl: [f32; 5]) -> Controller {
    Controller {
        active: true,
        pose,
        hand_tracking_active: p.hand_tracking,
        hand_curl: curl.map(|c| c.clamp(0.0, 1.0)),
        ..Default::default()
    }
}

/// What drives a tick.
#[derive(Clone, Copy, Debug)]
pub struct AnimInput {
    /// Seconds since the last tick.
    pub dt: f32,
    /// The owner's state: the head, where the body faces.
    pub owner: State,
    /// The avatar's speed ahead (m/s; negative backing).
    pub speed: f32,
    /// The bot's voice now (0..1), `None` while silent.
    pub voice: Option<f32>,
    /// Nothing else drives the head: glances are welcome.
    pub idle: bool,
}

/// The state of the motion; one per avatar.
pub struct Animator {
    pub params: AnimParams,
    t: f32,
    rng: u64,
    wobble: [Wobble; 14],
    breath: f32,
    breath_rate: f32,
    shift: Ease,
    shift_next: f32,
    glance_yaw: Ease,
    glance_pitch: Ease,
    glance_next: f32,
    glance_back: Option<f32>,
    speed: f32,
    gait: f32,
    talk: f32,
    loud_avg: f32,
    last_voice: f32,
    beats: Vec<(f32, bool)>,
}

/// Indices into [`Animator::wobble`].
const W_HEAD: usize = 0; // x, z, yaw, pitch, roll: 0..5
const W_HAND: usize = 5; // left x, y, z, right x, y, z: 5..11
const W_TURN: usize = 11; // left, right
const W_CURL: usize = 13;

impl Animator {
    pub fn new(params: AnimParams, seed: u64) -> Animator {
        let mut rng = seed | 1;
        let wobble = std::array::from_fn(|i| {
            // Heads sway slower than hands.
            let (lo, hi) = if i < W_HAND { (0.05, 0.3) } else { (0.1, 0.4) };
            Wobble::new(&mut rng, lo, hi)
        });
        let mut a = Animator {
            params,
            t: 0.0,
            rng,
            wobble,
            breath: 0.0,
            breath_rate: 1.0,
            shift: Ease::default(),
            shift_next: 0.0,
            glance_yaw: Ease::default(),
            glance_pitch: Ease::default(),
            glance_next: 0.0,
            glance_back: None,
            speed: 0.0,
            gait: 0.0,
            talk: 0.0,
            loud_avg: 0.0,
            last_voice: -10.0,
            beats: Vec::new(),
        };
        a.shift_next = a.uniform(8.0, 20.0);
        a.glance_next = a.uniform(4.0, 10.0);
        a
    }

    fn uniform(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * next(&mut self.rng)
    }

    /// One tick: the hands and the head's offset now; `rest` is the pose
    /// without motion, for when the owner holds still.
    pub fn update(&mut self, input: &AnimInput) -> Overlay {
        let p = self.params.clone();
        let dt = input.dt.clamp(0.0, 0.2);
        self.t += dt;
        let t = self.t;
        let k = EYE_HEIGHT / 1.6;
        let head = input.owner.head.position;
        let body_yaw = input.owner.body_yaw;

        // -- breathing: inhale 40 % of the cycle, each a little different.
        self.breath += p.breath_hz * self.breath_rate * dt;
        if self.breath >= 1.0 {
            self.breath -= 1.0;
            self.breath_rate = self.uniform(0.85, 1.15);
        }
        let warped = if self.breath < 0.4 { self.breath / 0.4 * 0.5 } else { 0.5 + (self.breath - 0.4) / 0.6 * 0.5 };
        let breath = 0.5 - 0.5 * (2.0 * PI * warped).cos();

        // -- talking: loudness, accents, how far into a gesture.
        let talking = input.voice.is_some();
        if let Some(level) = input.voice {
            self.last_voice = t;
            let accent = level > 0.05 && level > 2.0 * self.loud_avg && self.beats.last().is_none_or(|b| t - b.0 >= 0.25);
            if accent && p.talk {
                let nod = next(&mut self.rng) < 0.4;
                self.beats.push((t, nod));
            }
            self.loud_avg += (level - self.loud_avg) * (dt / 1.5).min(1.0);
        }
        self.beats.retain(|b| t - b.0 < 0.6);
        let gesturing = p.talk && t - self.last_voice < 1.2;
        self.talk += ((if gesturing { 1.0 } else { 0.0 }) - self.talk) * (dt / 0.35).min(1.0);
        let beat = self.beats.iter().map(|b| bump(t - b.0, 0.25)).fold(0.0f32, f32::max);
        let nod = self.beats.iter().filter(|b| b.1).map(|b| bump(t - b.0, 0.4)).fold(0.0f32, f32::max);

        // -- walking: speed eased in and out; the stride's phase (0: the
        // left heel strikes, 0.5: the right). Steps per second from motion
        // capture: walking 0.83 + 0.75 v, running 1.49 + 0.355 v.
        let v = input.speed.abs();
        self.speed += (v - self.speed) * (dt / 0.3).min(1.0);
        let v = self.speed;
        let walking = smoothstep(0.15, 0.5, v);
        let run = smoothstep(2.0, 2.8, v);
        let mix = |walk: f32, running: f32| walk + (running - walk) * run;
        let cadence = mix(0.83 + 0.75 * v.min(2.4), 1.49 + 0.355 * v);
        self.gait = (self.gait + cadence / 2.0 * dt).fract();
        let g = self.gait;
        // Once a stride, twice a stride: largest at phase `at`.
        let once = |at: f32| (2.0 * PI * (g - at)).cos();
        let twice = |at: f32| (4.0 * PI * (g - at)).cos();
        let amp = swing_amplitude(v) * p.swing * k;

        // -- weight shifts and glances.
        if t >= self.shift_next {
            let to = if self.shift.to > 0.0 { -1.0 } else { 1.0 } * self.uniform(0.03, 0.05);
            self.shift.go(t, to, 1.8);
            self.shift_next = t + self.uniform(8.0, 20.0);
        }
        let can_glance = p.glances && input.idle && !talking && v < 0.1;
        if !can_glance {
            if self.glance_yaw.to != 0.0 || self.glance_pitch.to != 0.0 {
                self.glance_yaw.go(t, 0.0, 0.4);
                self.glance_pitch.go(t, 0.0, 0.4);
            }
            self.glance_back = None;
            self.glance_next = self.glance_next.max(t + 3.0);
        } else if let Some(back) = self.glance_back {
            if t >= back {
                self.glance_yaw.go(t, 0.0, 0.5);
                self.glance_pitch.go(t, 0.0, 0.5);
                self.glance_back = None;
                self.glance_next = t + self.uniform(4.0, 10.0);
            }
        } else if t >= self.glance_next {
            let side = if next(&mut self.rng) < 0.5 { -1.0 } else { 1.0 };
            let (yaw, pitch) = (side * self.uniform(15.0, 35.0), self.uniform(-8.0, 6.0));
            let dur = self.uniform(0.3, 0.6);
            self.glance_yaw.go(t, yaw, dur);
            self.glance_pitch.go(t, pitch, dur);
            self.glance_back = Some(t + dur + self.uniform(1.0, 3.0));
        }

        // -- the head.
        let w = |a: &Wobble| a.at(t);
        let still = (1.0 - walking) * p.sway * if talking { 2.0 } else { 1.0 };
        let shift = self.shift.at(t) * p.sway * k;
        let (bs, bc) = body_yaw.to_radians().sin_cos();
        let (right, ahead) = ([bc, 0.0, bs], [bs, 0.0, -bc]);
        // Over the standing foot: furthest right at 0.81 (right stance).
        let sway = mix(0.024, 0.017) * k * once(mix(0.81, 0.74));
        let side_m = w(&self.wobble[W_HEAD]) * 0.012 * k * still + shift + walking * sway;
        let lean = smoothstep(0.3, 1.5, v) + run;
        let ahead_m = w(&self.wobble[W_HEAD + 1]) * 0.008 * k * still + p.lean_m * k * lean;
        // Lowest just after each heel strike, highest mid-stance.
        let bob = p.bob_m * mix(1.0, 3.3) * k * twice(mix(0.31, 0.41));
        let up_m = breath * p.breath_m * k + walking * bob;
        let head_offset = HeadOffset {
            // Turned and rolled with the chest (left at the left heel strike).
            yaw: w(&self.wobble[W_HEAD + 2]) * 2.5 * still + self.glance_yaw.at(t) - walking * mix(2.0, 6.4) * once(mix(0.05, 0.98)),
            pitch: w(&self.wobble[W_HEAD + 3]) * 1.5 * still - 0.3 * breath + self.glance_pitch.at(t) - 6.0 * nod - p.lean_deg * lean
                + walking * p.nod_deg * twice(0.16),
            roll: w(&self.wobble[W_HEAD + 4]) * 1.0 * still + shift / (0.04 * k) * 1.5 - walking * mix(1.5, 0.3) * once(0.04),
            position: std::array::from_fn(|i| right[i] * side_m + ahead[i] * ahead_m + if i == 1 { up_m } else { 0.0 }),
        };

        // -- the hands.
        let mut hands = [Controller::default(); 2];
        let mut rest = [Controller::default(); 2];
        for (n, side) in [(0usize, -1.0f32), (1, 1.0)] {
            let hw = |i: usize| w(&self.wobble[W_HAND + 3 * n + i]);
            // Opposite arms: the left furthest forward just after the right
            // heel strikes (0.56), the right half a stride on.
            let peak = mix(0.56, 0.38) - if side > 0.0 { 0.5 } else { 0.0 };
            let swing = amp * once(peak); // ahead of the mean
            let phase = swing / (amp + 1e-4); // -1..1
            let mean = walking * mix(0.038, 0.045) * k;
            // Rising as it comes forward; running, at both ends too (U).
            let rise = (1.0 - run) * 0.20 * swing + run * (0.46 * swing + 9.1 * swing * swing / k) + run * 0.20 * k;
            // Body frame offsets (x right, y up, z back), tracking metres.
            let mut off = [
                hw(0) * (0.007 + 0.4 * p.arm_sway_m) * k * still - side * run * 0.06 * k,
                hw(1) * 0.007 * k * still + breath * p.breath_m * k + walking * rise,
                hw(2) * (0.007 + p.arm_sway_m) * k * still - walking * (mean + swing),
            ];
            // Fingers tip forward (pitch, degrees) and the palm turns back
            // (twist) as the arm comes forward; running holds the hand out.
            let pitch = walking * (mix(23.0, 83.0) + mix(20.0, 24.5) * phase);
            let twist = walking * (mix(26.0, 11.0) + (1.0 - run) * 10.6 * phase);
            // The wrist follows the arm's own sway at rest too.
            let mut tilt = (-hw(2) * p.arm_sway_m * k * still / (0.6 * k)).atan() + pitch.to_radians();
            let mut curl = p.curl.map(|c| c + run * (0.75 - c) + w(&self.wobble[W_CURL]) * 0.05 * still);
            if side > 0.0 && self.talk > 0.0 {
                // The talking hand: up in front, beating down.
                let g = self.talk;
                off[0] += g * -0.05 * k;
                off[1] += g * (0.30 * k - 0.045 * k * beat);
                off[2] += g * (-0.22 * k - 0.02 * k * beat);
                tilt += g * 0.5;
                curl = curl.map(|c| c - g * 0.15 * beat);
            }
            let turn = quat_mul(
                quat_axis([1.0, 0.0, 0.0], tilt),
                quat_axis([0.0, 1.0, 0.0], side * (w(&self.wobble[W_TURN + n]) * 3.0 * still + twist).to_radians()),
            );
            hands[n] = hand(&p, hand_pose(&p, head, body_yaw, side, off, turn), curl);
            rest[n] = hand(&p, hand_pose(&p, head, body_yaw, side, [0.0; 3], [0.0, 0.0, 0.0, 1.0]), p.curl);
        }
        if !p.enabled {
            return Overlay { left: rest[0], right: rest[1], rest, head: HeadOffset::default() };
        }
        Overlay { left: hands[0], right: hands[1], rest, head: head_offset }
    }
}

/// The front-to-back half swing of the wrist (m, 1.6 m eye height) at
/// `v` m/s, from motion capture: 15 cm at 1.2 m/s, 23 at 1.65; running
/// (3.35) bends the elbows and swings 13.
fn swing_amplitude(v: f32) -> f32 {
    const TABLE: [(f32, f32); 8] = [(0.0, 0.0), (0.7, 0.10), (1.0, 0.14), (1.3, 0.16), (1.7, 0.21), (2.2, 0.20), (3.0, 0.14), (4.0, 0.13)];
    if v < 0.3 {
        return 0.0;
    }
    for w in TABLE.windows(2) {
        let ((v0, a0), (v1, a1)) = (w[0], w[1]);
        if v <= v1 {
            return a0 + (a1 - a0) * (v - v0) / (v1 - v0);
        }
    }
    TABLE[TABLE.len() - 1].1
}

fn smoothstep(lo: f32, hi: f32, x: f32) -> f32 {
    let t = ((x - lo) / (hi - lo)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A half sine over `len` seconds from 0: 0 before, 1 at the middle.
fn bump(t: f32, len: f32) -> f32 {
    if (0.0..len).contains(&t) {
        (PI * t / len).sin()
    } else {
        0.0
    }
}

/// xorshift64*: 0..1.
fn next(s: &mut u64) -> f32 {
    *s ^= *s >> 12;
    *s ^= *s << 25;
    *s ^= *s >> 27;
    ((s.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40) as f32) / (1u64 << 24) as f32
}

/// Smooth noise in about -1..1: three sines of unrelated frequencies.
#[derive(Clone, Copy)]
struct Wobble([(f32, f32, f32); 3]);

impl Wobble {
    fn new(rng: &mut u64, lo: f32, hi: f32) -> Wobble {
        Wobble(std::array::from_fn(|i| {
            let f = lo + (hi - lo) * next(rng);
            let phase = 2.0 * PI * next(rng);
            // The octaves: half as loud each.
            (f * (1.0 + i as f32 * 0.7), phase, 0.57 / (1 << i) as f32)
        }))
    }

    fn at(&self, t: f32) -> f32 {
        self.0.iter().map(|&(f, ph, a)| a * (2.0 * PI * f * t + ph).sin()).sum()
    }
}

/// A value moving to a target along a minimum-jerk curve.
#[derive(Clone, Copy, Default)]
struct Ease {
    from: f32,
    to: f32,
    start: f32,
    len: f32,
}

impl Ease {
    fn at(&self, t: f32) -> f32 {
        if self.len <= 0.0 {
            return self.to;
        }
        let s = ((t - self.start) / self.len).clamp(0.0, 1.0);
        let m = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
        self.from + (self.to - self.from) * m
    }

    fn go(&mut self, t: f32, to: f32, len: f32) {
        *self = Ease { from: self.at(t), to, start: t, len };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(speed: f32, voice: Option<f32>) -> AnimInput {
        let owner = State { head: Pose::looking(0.0, 0.0, [0.0, EYE_HEIGHT, 0.0]), ..Default::default() };
        AnimInput { dt: 1.0 / 45.0, owner, speed, voice, idle: true }
    }

    #[test]
    fn rest_matches_the_owner_rest() {
        let mut s = State::default();
        s.hands_at_rest([0.0, EYE_HEIGHT, 0.0], 0.0);
        let mut a = Animator::new(AnimParams::default(), 7);
        let o = a.update(&input(0.0, None));
        assert_eq!(o.rest[1].pose, s.right.pose);
        assert_eq!(o.rest[0].pose, s.left.pose);
    }

    #[test]
    fn idle_moves_a_little() {
        let mut a = Animator::new(AnimParams::default(), 7);
        let mut most = 0.0f32;
        for _ in 0..45 * 30 {
            let o = a.update(&input(0.0, None));
            let d: f32 = (0..3).map(|i| (o.right.pose.position[i] - o.rest[1].pose.position[i]).powi(2)).sum::<f32>().sqrt();
            most = most.max(d);
            assert!(o.head.yaw.abs() < 40.0 && o.head.pitch.abs() < 15.0);
        }
        assert!(most > 0.01 && most < 0.09, "{most}");
    }

    #[test]
    fn walking_swings_the_arms_opposite() {
        let mut a = Animator::new(AnimParams { sway: 0.0, ..Default::default() }, 7);
        let (mut fwd_r, mut fwd_l, mut opposite) = (0.0f32, 0.0f32, 0);
        for i in 0..45 * 4 {
            let o = a.update(&input(1.4, None));
            if i < 45 {
                continue; // up to speed
            }
            // Ahead is -z (body faces 0 degrees).
            let r = o.rest[1].pose.position[2] - o.right.pose.position[2];
            let l = o.rest[0].pose.position[2] - o.left.pose.position[2];
            fwd_r = fwd_r.max(r);
            fwd_l = fwd_l.max(l);
            if r * l < 0.0 {
                opposite += 1;
            }
        }
        let k = EYE_HEIGHT / 1.6;
        // Motion capture: the wrist reaches about 19-27 cm ahead of the head walking.
        assert!(fwd_r > 0.15 * k && fwd_r < 0.27 * k, "{fwd_r}");
        assert!(fwd_l > 0.10 * k, "{fwd_l}");
        assert!(opposite > 100, "{opposite}");
    }

    #[test]
    fn talking_raises_the_right_hand() {
        let mut a = Animator::new(AnimParams::default(), 7);
        let mut o = a.update(&input(0.0, None));
        for i in 0..45 * 2 {
            let level = if i % 15 < 3 { 0.6 } else { 0.1 };
            o = a.update(&input(0.0, Some(level)));
        }
        assert!(o.right.pose.position[1] > o.rest[1].pose.position[1] + 0.2);
        assert!((o.left.pose.position[1] - o.rest[0].pose.position[1]).abs() < 0.05);
        // Silence: back down.
        for _ in 0..45 * 3 {
            o = a.update(&input(0.0, None));
        }
        assert!(o.right.pose.position[1] < o.rest[1].pose.position[1] + 0.05);
    }

    #[test]
    fn noise_is_bounded() {
        let mut rng = 3u64;
        let w = Wobble::new(&mut rng, 0.1, 0.4);
        for i in 0..1000 {
            assert!(w.at(i as f32 * 0.1).abs() <= 1.0);
        }
        assert!((0..1000).all(|_| (0.0..1.0).contains(&next(&mut rng))));
    }
}
