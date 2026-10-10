//! The avatar's procedural motion (`vrc_vr::anim`), run 45 times a second
//! on the headset connection's animator handle: the owner (surveys, walks,
//! the follower) keeps the head and the body's facing; this adds the hands'
//! and the head's motion on top, and stands aside while a scan holds still.
//!
//! Inputs: the avatar's speed over the ground (OSCQuery `VelocityZ` and
//! `VelocityX`: a step aside walks too, `ground_speed`), the loudness of the
//! bot's own voice as it plays (`heard_bot`), and whether anything else is
//! driving the head (glances only when not). Parameters are tunable at run
//! time (`/v1/anim`) and kept in `anim.json` next to the token.
//!
//! The same tick sends VRChat's OSC trackers when they are on
//! (`/v1/vr/trackers`, trying out full body: a body standing under the
//! head), and stands aside while the hands are set by hand (`/v1/vr/hand`);
//! every few seconds it checks whether VRChat lost them, to calibrate again
//! (`crate::calibrate::Watch`), and whether the user camera closed though
//! it should stay open (`crate::usercam::Watch`).

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;
use vrc_vr::anim::{AnimInput, AnimParams, Animator};
use vrc_vr::osc::Osc;
use vrc_vr::remote::{HmdLink, FLOOR_Y};
use vrc_vr::trackers::{self, Part};

use crate::bridge::{Bridge, SAMPLE_RATE};
use crate::Lock;

const TICK: Duration = Duration::from_millis(22);
/// A headset not there is tried again this often.
const RECONNECT_EVERY: Duration = Duration::from_secs(5);
/// The speed is read every few ticks (an HTTP request each).
const SPEED_EVERY: u32 = 3;
/// The avatar's size (eye height) is read this often (ticks).
const SIZE_EVERY: u32 = 90;
/// Moving faster than this (world m/s), the legs walk.
const GAIT_FROM: f32 = 0.25;
/// From walking to running by the Froude number (v² over g times the legs'
/// length, the hips' height): people change over at about 0.5; a 1.6 m
/// avatar runs from 1.8 m/s, fully by 2.2 (the follower's 1.8 still walks).
const RUN_FROM_FROUDE: f32 = 0.4;
const RUN_FULL_FROUDE: f32 = 0.6;
/// Seconds for the legs to start and stop walking, and to change pace.
const GAIT_FADE: f32 = 0.35;
/// Seconds for the legs to tuck up off the ground (`jump_air`), and to come
/// down again on landing.
const AIR_IN: f32 = 0.12;
const AIR_OUT: f32 = 0.25;

/// The clips' joints (`vrc_vr::motion::JOINTS`).
const J_HIPS: usize = 0;
const J_KNEES: [usize; 2] = [3, 4];
const J_FEET: [usize; 2] = [5, 6];

/// The share of a cycle a foot is on the ground at `v` statures a second,
/// shorter the faster (user, 2026-10-11: the time a foot rests on the
/// ground by the speed), as people's: walking 67% at 0.5 m/s, 62% at 1.2,
/// 58% at 1.8; running 40% at 2.2 m/s, 35% at 3, 30% at 4 (1.6 m tall).
fn duty(v: f32, run: f32) -> f32 {
    let walk = (0.70 - 0.11 * v).clamp(0.56, 0.68);
    walk + ((0.52 - 0.09 * v).clamp(0.25, 0.42) - walk) * run
}
/// Where a foot lands, as a share of its sweep under the hips ahead of
/// them (the rest it goes behind, the heel coming up to reach): a foot far
/// ahead pulled the hips down at each step, a heavy tread.
const LAND_AHEAD: f32 = 0.4;
/// A foot on the ground sweeps at most this far under the hips (statures):
/// faster, the steps come quicker instead.
const MAX_SWEEP: f32 = 0.385;
/// The steps this much shorter than people's, and quicker by as much (user,
/// 2026-10-11: "步子迈太开了，减少 30%，交替速度增加"; then 10% longer
/// again, 0.7 was a little short).
const STRIDE: f32 = 0.77;
/// The heel comes up over this last share of the time on the ground, this
/// high (statures), the foot pitched toes down this much (degrees); it
/// lands toes up this much and is flat again FLAT_BY into the next.
const HEEL_OFF: [f32; 2] = [0.25, 0.4];
const HEEL_UP: [f32; 2] = [0.015, 0.025];
const TOES_DOWN_DEG: [f32; 2] = [18.0, 26.0];
const TOES_UP_DEG: [f32; 2] = [10.0, 4.0];
const FLAT_BY: f32 = 0.15;
/// The knees a little bent all the way (statures the hips come down), and
/// running bent more under the body's weight mid-stance; the hips come down
/// this share of what straight legs would need to reach the feet (the
/// ankles and the hips' twist give the rest) and at most MAX_DROP in all:
/// all of it bobbed the body down hard at every step (user: "步子迈太重了").
const CROUCH: [f32; 2] = [0.005, 0.012];
const RUN_GIVE: f32 = 0.015;
const REACH_DROP: f32 = 0.6;
const MAX_DROP: f32 = 0.045;
/// The hips twist toward the leg ahead (degrees).
const HIPS_TWIST_DEG: f32 = 4.0;
/// Running, the feet come in toward the midline, this far off it
/// (statures) at most.
const RUN_FEET_X: f32 = 0.08;

/// Cycles (two steps) a second at `v` statures a second: as people walk
/// (1.75 steps a second at 1 m/s, 2.1 at 1.8; 1.7 m tall) and run (2.7 at
/// 3 m/s, 2.9 at 4), quickened for strides STRIDE as long.
fn cadence(v: f32, run: f32) -> f32 {
    let walk = 0.62 + 0.4 * v;
    (walk + (1.22 + 0.09 * v - walk) * run) / STRIDE
}

/// How high a swinging foot lifts (statures): 6-10 cm walking (1.6 m
/// tall), running 14-21 cm with the heel kicked up behind (higher stepped
/// heavily, user 2026-10-11).
fn lift(v: f32, run: f32) -> f32 {
    let walk = 0.035 + 0.025 * (v / 1.2).min(1.0);
    walk + (0.09 + 0.04 * ((v - 1.2) / 1.5).clamp(0.0, 1.0) - walk) * run
}

/// The legs walking or running in place while the bot moves (the stick
/// moves it), stepped here: played from the captured cycles at the bot's
/// 2-4 m/s, a cycle's stride (1.1 m running) took seven steps a second,
/// short shuffles that slid and hardly lifted the feet, and the captured
/// run curved 17° a cycle, swinging back each loop (user, 2026-10-11:
/// "左右脚不协调，右脚有种在打滑的感觉，缺少抬腿的感觉"). A foot on the ground
/// moves back exactly as fast as the floor goes by (it stays put), the
/// stride grows with the speed (cadence as people's), the heel comes up
/// before the foot lifts and the hips come down as far as the legs need to
/// reach. The arms and the chest swing as captured (`walk_cycle`,
/// `run_cycle`, their turn taken out), at the same phase.
#[derive(Default)]
struct Gait {
    /// Into the cycle (0..1, from a left heel strike).
    phase: f32,
    /// How much the legs walk (0: standing).
    weight: f32,
    /// How much of that is running.
    run: f32,
    /// How much the legs are tucked up in the air.
    air: f32,
    /// The feet (left, right) while walking.
    feet: Option<[Stride; 2]>,
}

/// A foot in the gait: how far ahead of the hips (statures), whether on the
/// ground, and where it lifted off.
#[derive(Clone, Copy, Debug, Default)]
struct Stride {
    z: f32,
    down: bool,
    lift_z: f32,
}

impl Gait {
    /// The legs' pose now (walking, running, in the air), or None while
    /// standing still on the ground.
    fn update(&mut self, motions: &crate::motion::Library, speed: f32, stature: f32, dt: f32, airborne: bool) -> Option<vrc_vr::motion::Body> {
        let walk = motions.get("walk_cycle")?;
        let run = motions.get("run_cycle")?;
        let air = motions.get("jump_air");
        self.step(&walk, &run, air.as_deref(), speed, stature, dt, airborne)
    }

    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        walk: &vrc_vr::motion::Clip,
        run: &vrc_vr::motion::Clip,
        air: Option<&vrc_vr::motion::Clip>,
        speed: f32,
        stature: f32,
        dt: f32,
        airborne: bool,
    ) -> Option<vrc_vr::motion::Body> {
        let pace = speed.abs();
        let fade = (dt / GAIT_FADE).min(1.0);
        let towards = |x: f32, to: f32| x + (to - x) * fade;
        self.weight = towards(self.weight, if pace > GAIT_FROM && !airborne { 1.0 } else { 0.0 });
        let froude = pace * pace / (9.81 * walk.standing[J_HIPS].pos[1] * stature);
        let run_to = ((froude - RUN_FROM_FROUDE) / (RUN_FULL_FROUDE - RUN_FROM_FROUDE)).clamp(0.0, 1.0);
        self.run = towards(self.run, run_to);
        self.air = if airborne { (self.air + dt / AIR_IN).min(1.0) } else { (self.air - dt / AIR_OUT).max(0.0) };
        let tucked = self.air;
        let tuck = |body: vrc_vr::motion::Body| match air {
            Some(a) if tucked > 0.0 && !a.frames.is_empty() => vrc_vr::motion::blend(&body, &a.frames[0], ease(tucked)),
            _ => body,
        };
        if self.weight < 0.01 {
            self.weight = 0.0;
            self.phase = 0.0;
            self.feet = None;
            if self.air > 0.0 {
                return Some(tuck(walk.standing));
            }
            return None;
        }
        let legs = self.legs(&walk.standing, speed / stature, dt);
        // The arms and the chest as captured, at the same phase.
        let mut body = vrc_vr::motion::blend(&cycle_pose(walk, self.phase), &cycle_pose(run, self.phase), self.run);
        for j in [J_HIPS, J_KNEES[0], J_KNEES[1], J_FEET[0], J_FEET[1]] {
            body[j] = legs[j];
        }
        Some(tuck(vrc_vr::motion::blend(&walk.standing, &body, self.weight)))
    }

    /// The hips, knees and feet (body frame) a tick on at `v` statures a
    /// second (negative: backing).
    fn legs(&mut self, standing: &vrc_vr::motion::Body, v: f32, dt: f32) -> vrc_vr::motion::Body {
        let run = self.run;
        let mix = |a: [f32; 2]| a[0] + (a[1] - a[0]) * run;
        let pace = v.abs();
        let duty = duty(pace, run);
        let f = cadence(pace, run).max(pace * duty / MAX_SWEEP);
        // On the ground a foot goes back `sweep` under the hips: it lands
        // LAND_AHEAD of that ahead.
        let sweep = v * duty / f;
        let swing_s = (1.0 - duty) / f;
        let first = self.feet.is_none();
        if !first {
            self.phase = (self.phase + f * dt).rem_euclid(1.0);
        }
        // Backing, the toes land first and the heel lifts last.
        let toes = if v < 0.0 { -1.0 } else { 1.0 };
        let hips_y = standing[J_HIPS].pos[1];
        let mut body = *standing;
        let mut drop = 0.0f32;
        let feet = self.feet.get_or_insert([Stride::default(); 2]);
        for (i, foot) in feet.iter_mut().enumerate() {
            let local = (self.phase + 0.5 * i as f32).rem_euclid(1.0);
            let down = local < duty;
            if down && (first || !foot.down) {
                // Down: where it landed, the floor gone by since.
                foot.z = sweep * LAND_AHEAD - v * local / f;
            } else if down {
                foot.z -= v * dt;
            } else if first || foot.down {
                foot.lift_z = if first { sweep * (LAND_AHEAD - 1.0) } else { foot.z };
            }
            foot.down = down;
            foot.z = foot.z.clamp(-MAX_SWEEP, MAX_SWEEP);
            let at = standing[J_FEET[i]].pos;
            let side = at[0].signum();
            let x = at[0].abs() + (at[0].abs().min(RUN_FEET_X) - at[0].abs()) * run;
            let (z, up, pitch, grounded) = if down {
                let s = local / duty;
                let heel = ease(((s - (1.0 - mix(HEEL_OFF))) / mix(HEEL_OFF)).clamp(0.0, 1.0));
                let flat = 1.0 - ease((s / FLAT_BY).min(1.0));
                let give = RUN_GIVE * run * (std::f32::consts::PI * s).sin();
                drop = drop.max(give);
                (foot.z, mix(HEEL_UP) * heel, mix(TOES_DOWN_DEG) * heel - mix(TOES_UP_DEG) * flat, 1.0 - heel)
            } else {
                let u = ((local - duty) / (1.0 - duty)).clamp(0.0, 1.0);
                // Leaving and landing at the floor's speed (still on it).
                let m = -v * swing_s;
                let z = hermite(foot.lift_z, sweep * LAND_AHEAD, m, m, u);
                // Set down softly (slowing to the floor), not stamped.
                let k = 0.9 - 0.3 * run;
                let up = mix(HEEL_UP) * (1.0 - u) * (1.0 - u) + lift(pace, run) * (std::f32::consts::PI * u.powf(k)).sin().powf(1.5);
                let pitch = mix(TOES_DOWN_DEG) + (-mix(TOES_UP_DEG) - mix(TOES_DOWN_DEG)) * ease(u);
                (z, up, pitch, ease(((u - 0.8) / 0.2).clamp(0.0, 1.0)))
            };
            // A straight leg reaches that far ahead or behind only with the
            // hips lower.
            let leg = hips_y - at[1];
            let reach = leg - (leg * leg - z.abs().min(leg * 0.95).powi(2)).sqrt();
            drop = drop.max(reach * grounded * REACH_DROP);
            let half = (toes * pitch).to_radians() / 2.0;
            body[J_FEET[i]].pos = [side * x, at[1] + up, z];
            body[J_FEET[i]].rot = [half.sin(), 0.0, 0.0, half.cos()];
        }
        let drop = (drop + mix(CROUCH)).min(MAX_DROP);
        let hips = &mut body[J_HIPS];
        hips.pos[1] = hips_y - drop;
        // Twisted toward the leg ahead (the left at the cycle's start; + to
        // the body's right is about -y).
        let half = -(HIPS_TWIST_DEG * toes * (std::f32::consts::TAU * self.phase).cos()).to_radians() / 2.0;
        hips.rot = [0.0, half.sin(), 0.0, half.cos()];
        let hips_at = hips.pos;
        for i in 0..2 {
            body[J_KNEES[i]].pos = knee(hips_at, standing[J_KNEES[i]].pos, standing[J_FEET[i]].pos, body[J_FEET[i]].pos);
        }
        body
    }
}

/// Cubic Hermite from `a` to `b` (tangents `ma`, `mb` per unit `u`).
fn hermite(a: f32, b: f32, ma: f32, mb: f32, u: f32) -> f32 {
    let (u2, u3) = (u * u, u * u * u);
    (2.0 * u3 - 3.0 * u2 + 1.0) * a + (u3 - 2.0 * u2 + u) * ma + (-2.0 * u3 + 3.0 * u2) * b + (u3 - u2) * mb
}

/// Where a knee goes (body frame) for the hips at `hips` and the ankle at
/// `foot`, bending forward: the thigh and the shin as long as standing
/// (`knee0`, `foot0`; the hip joint level with the hips).
fn knee(hips: [f32; 3], knee0: [f32; 3], foot0: [f32; 3], foot: [f32; 3]) -> [f32; 3] {
    let thigh = (hips[1] - knee0[1]).max(1e-3);
    let shin = (knee0[1] - foot0[1]).max(1e-3);
    let (dz, dy) = (foot[2] - hips[2], foot[1] - hips[1]);
    let d = dz.hypot(dy).clamp(1e-3, (thigh + shin) * 0.999);
    let a = ((thigh * thigh + d * d - shin * shin) / (2.0 * thigh * d)).clamp(-1.0, 1.0).acos();
    let (uz, uy) = (dz / d, dy / d);
    // Turned toward ahead (+z) by `a` from the line to the ankle.
    let (kz, ky) = (uz * a.cos() - uy * a.sin(), uz * a.sin() + uy * a.cos());
    [knee0[0] + (foot[0] - foot0[0]) * 0.5, hips[1] + ky * thigh, hips[2] + kz * thigh]
}

/// A gait cycle's pose `phase` (0..1) through, in place and with its
/// path's curve taken out (looped, the body swung back each cycle). Its
/// frames are one cycle without the next one's first (`tools/motion`): the
/// last goes on into the first.
fn cycle_pose(c: &vrc_vr::motion::Clip, phase: f32) -> vrc_vr::motion::Body {
    let n = c.frames.len();
    if n == 0 {
        return c.standing;
    }
    // Where the hips face (radians, + toward the body's left, +x).
    let yaw = |b: &vrc_vr::motion::Body| {
        let a = vrc_vr::Pose { orientation: b[J_HIPS].rot, position: [0.0; 3] }.rotate([0.0, 0.0, 1.0]);
        a[0].atan2(a[2])
    };
    let wrap = |a: f32| (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
    let per = if n > 1 { wrap(yaw(&c.frames[n - 1]) - yaw(&c.frames[0])) / (n - 1) as f32 } else { 0.0 };
    let mean = c.frames.iter().enumerate().map(|(i, b)| wrap(yaw(b) - per * i as f32 - yaw(&c.frames[0]))).sum::<f32>() / n as f32 + yaw(&c.frames[0]);
    let s = c.standing[J_HIPS].pos;
    let straight = |i: usize| {
        let b = &c.frames[i];
        let turn = -(per * i as f32 + mean);
        let q = [0.0, (turn / 2.0).sin(), 0.0, (turn / 2.0).cos()];
        let r = vrc_vr::Pose { orientation: q, position: [0.0; 3] };
        let (dx, dz) = (b[J_HIPS].pos[0] - s[0], b[J_HIPS].pos[2] - s[2]);
        let mut out = *b;
        for j in out.iter_mut() {
            let p = r.rotate([j.pos[0] - dx - s[0], 0.0, j.pos[2] - dz - s[2]]);
            j.pos = [s[0] + p[0], j.pos[1], s[2] + p[2]];
            j.rot = vrc_vr::pose::quat_mul(q, j.rot);
        }
        out
    };
    let x = phase.rem_euclid(1.0) * n as f32;
    let i = (x.floor() as usize).min(n - 1);
    vrc_vr::motion::blend(&straight(i), &straight((i + 1) % n), x - i as f32)
}

/// A foot steps to where the body would stand it once it is this far off
/// (degrees of turn, or metres), one foot at a time.
const STEP_TURN_DEG: f32 = 15.0;
const STEP_OFF_M: f32 = 0.06;
/// A step: how long, how high the foot lifts (metres) and how far it turns
/// at most (degrees; a bigger turn takes more steps).
const STEP_S: f32 = 0.2;
const STEP_LIFT_M: f32 = 0.05;
const STEP_MAX_DEG: f32 = 60.0;
/// Far behind the body (more than FAR_DEG to turn yet), the steps are
/// quicker and bigger.
const FAR_DEG: f32 = 90.0;
const STEP_FAR_S: f32 = 0.15;
const STEP_FAR_MAX_DEG: f32 = 75.0;
/// The hips lead the feet by at most this much (degrees): the body turns
/// at once (a turn round on the spot, a new way round by the map), the feet
/// take a step at a time; hips that turned with the body left the feet on
/// the wrong sides of them, the legs crossed and twisted (user: "有时候 ai
/// 会以奇怪的姿势扭曲脚"). The rest of a quick turn is the upper body's.
const HIPS_AHEAD_FEET_DEG: f32 = 45.0;
/// A foot on the ground pivots (turns where it stands) to stay within this
/// of where the hips face (degrees): the body turns with the head, the feet
/// twist and step after it. Turning its toes in toward the other foot, less
/// (they would cross).
const PIVOT_DEG: f32 = 40.0;
const PIVOT_IN_DEG: f32 = 15.0;
/// A step lands no nearer the other foot than this share of their standing
/// distance.
const STEP_KEEP_APART: f32 = 0.7;

/// The hips turn with the body, behind it at most this much (degrees): a
/// quick turn of the head takes the body round with it.
const HIPS_BEHIND_DEG: f32 = 15.0;

/// The feet stay where they stand while the body turns above them, then
/// step after it one at a time, round the body's centre (the head's plumb
/// line), the foot on the side it turns to first: turning on the spot, the
/// feet tread round instead of sliding round, and never through each other.
#[derive(Default)]
struct Feet {
    /// Where each foot stands (left, right), once known.
    planted: [Option<vrc_vr::Pose>; 2],
    /// The foot stepping now: which, from, to, the turn it makes round the
    /// centre (degrees), how far through (0..1), how long it takes.
    step: Option<(usize, vrc_vr::Pose, vrc_vr::Pose, f32, f32, f32)>,
    /// Where the hips face (degrees), close behind the body.
    hips: Option<f32>,
}

fn yaw_of(p: &vrc_vr::Pose) -> f32 {
    p.yaw_pitch().0
}

fn turn_between(a: f32, b: f32) -> f32 {
    (b - a + 540.0).rem_euclid(360.0) - 180.0
}

/// `p` turned `deg` (+ right) round the vertical through `centre`.
fn round(p: [f32; 3], centre: [f32; 3], deg: f32) -> [f32; 3] {
    let r = vrc_vr::Pose::looking(deg, 0.0, [0.0; 3]).rotate([p[0] - centre[0], 0.0, p[2] - centre[2]]);
    [centre[0] + r[0], p[1], centre[2] + r[2]]
}

impl Feet {
    /// Lets go: something else moves the feet (a gait, a program).
    fn reset(&mut self) {
        *self = Feet::default();
    }

    /// The feet now (left, right) and the yaw for the hips, given where the
    /// body would stand the feet (`wanted`), its centre and its facing.
    fn update(&mut self, wanted: [vrc_vr::Pose; 2], centre: [f32; 3], body_yaw: f32, dt: f32) -> ([vrc_vr::Pose; 2], f32) {
        for i in 0..2 {
            if self.planted[i].is_none() {
                self.planted[i] = Some(wanted[i]);
            }
        }
        let mut now = [self.planted[0].unwrap(), self.planted[1].unwrap()];
        if let Some((i, from, to, turn, t, secs)) = self.step.as_mut() {
            *t = (*t + dt / *secs).min(1.0);
            let w = ease(*t);
            // Round the centre, then the rest of the way straight.
            let swung = round(from.position, centre, *turn * w);
            let full = round(from.position, centre, *turn);
            let mut p = [0, 1, 2].map(|k| swung[k] + (to.position[k] - full[k]) * w);
            p[1] = from.position[1] + (to.position[1] - from.position[1]) * w + STEP_LIFT_M * (std::f32::consts::PI * *t).sin();
            let yaw = yaw_of(from) + turn_between(yaw_of(from), yaw_of(to)) * w;
            now[*i] = vrc_vr::Pose::looking(yaw, 0.0, p);
            if *t >= 1.0 {
                self.planted[*i] = Some(*to);
                now[*i] = *to;
                self.step = None;
            }
        } else {
            let off = |i: usize| {
                let turn = turn_between(yaw_of(&now[i]), yaw_of(&wanted[i]));
                let p = &now[i].position;
                let dist = ((p[0] - wanted[i].position[0]).powi(2) + (p[2] - wanted[i].position[2]).powi(2)).sqrt();
                (turn, dist)
            };
            let need = |i: usize| {
                let (turn, dist) = off(i);
                turn.abs() / STEP_TURN_DEG + dist / STEP_OFF_M
            };
            // The foot on the side of the turn first (right turn: the right
            // foot), unless the other is much further off.
            let (turn0, _) = off(0);
            let lead = if turn0 > 0.0 { 1 } else { 0 };
            let i = if need(1 - lead) > 1.5 * need(lead) { 1 - lead } else { lead };
            let (turn, dist) = off(i);
            if turn.abs() > STEP_TURN_DEG || dist > STEP_OFF_M {
                let far = turn.abs() > FAR_DEG;
                let most = if far { STEP_FAR_MAX_DEG } else { STEP_MAX_DEG };
                let k = (most / turn.abs().max(1e-3)).min(1.0);
                let from = now[i];
                let swing = turn * k;
                let mut to = if k >= 1.0 {
                    wanted[i]
                } else {
                    vrc_vr::Pose::looking(yaw_of(&from) + swing, 0.0, round(from.position, centre, swing))
                };
                // Not onto the other foot: no nearer it than most of their
                // standing distance.
                let other = now[1 - i].position;
                let apart = |a: [f32; 3], b: [f32; 3]| ((a[0] - b[0]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                let keep = STEP_KEEP_APART * apart(wanted[0].position, wanted[1].position);
                let d = apart(to.position, other);
                if d < keep && d > 1e-4 {
                    let s = keep / d;
                    to.position[0] = other[0] + (to.position[0] - other[0]) * s;
                    to.position[2] = other[2] + (to.position[2] - other[2]) * s;
                }
                self.step = Some((i, from, to, swing, 0.0, if far { STEP_FAR_S } else { STEP_S }));
            }
        }
        // The hips with the body (a little behind it at most); a foot on the
        // ground pivots where it stands to keep within PIVOT_DEG of them.
        self.hips = match self.hips {
            Some(h) => {
                let d = turn_between(h, body_yaw);
                Some(body_yaw - d.clamp(-HIPS_BEHIND_DEG, HIPS_BEHIND_DEG) * (1.0 - (dt / 0.15).min(1.0)))
            }
            None => Some(body_yaw),
        };
        // Not further round than the feet allow: the way they stand, from
        // where they are (left to right is the stance's right), not how they
        // point (a planted foot pivots where it stands: by its pointing the
        // hips went round past it and the feet stood in a line across them).
        let (l, r) = (now[0].position, now[1].position);
        let feet_yaw = (r[2] - l[2]).atan2(r[0] - l[0]).to_degrees();
        let lead = turn_between(feet_yaw, self.hips.unwrap());
        if lead.abs() > HIPS_AHEAD_FEET_DEG {
            self.hips = Some(feet_yaw + lead.clamp(-HIPS_AHEAD_FEET_DEG, HIPS_AHEAD_FEET_DEG));
        }
        let hips = self.hips.unwrap();
        let stepping = self.step.as_ref().map(|s| s.0);
        for i in 0..2 {
            if stepping == Some(i) {
                continue;
            }
            // The foot behind the hips by `off`: it pivots to within its
            // limit. Toes turning in (the left foot turning right, the right
            // foot left) go less far.
            let off = turn_between(hips, yaw_of(&now[i]));
            let turning_in = if i == 0 { off < 0.0 } else { off > 0.0 };
            let limit = if turning_in { PIVOT_IN_DEG } else { PIVOT_DEG };
            if off.abs() > limit {
                let yaw = hips + off.clamp(-limit, limit);
                now[i] = vrc_vr::Pose::looking(yaw, 0.0, now[i].position);
                self.planted[i] = Some(now[i]);
            }
        }
        (now, hips)
    }
}

/// Between two poses (0: a, 1: b).
fn lerp_pose(a: &vrc_vr::Pose, b: &vrc_vr::Pose, w: f32) -> vrc_vr::Pose {
    let position = [0, 1, 2].map(|k| a.position[k] + (b.position[k] - a.position[k]) * w);
    let dot: f32 = (0..4).map(|i| a.orientation[i] * b.orientation[i]).sum();
    let s = if dot < 0.0 { -1.0 } else { 1.0 };
    let q: [f32; 4] = std::array::from_fn(|i| a.orientation[i] * (1.0 - w) + s * b.orientation[i] * w);
    let n = q.iter().map(|x| x * x).sum::<f32>().sqrt().max(1e-6);
    vrc_vr::Pose { orientation: q.map(|x| x / n), position }
}

/// Eased in and out (0..1 to 0..1).
fn ease(w: f32) -> f32 {
    w * w * (3.0 - 2.0 * w)
}

/// Whether the body needs a calibration is checked every few seconds.
const CALIBRATION_EVERY: u32 = 230;
/// Loudness is measured over windows this long.
const WINDOW: Duration = Duration::from_millis(30);

/// The OSC trackers (`/v1/vr/trackers`).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TrackerSettings {
    pub on: bool,
    /// `hip`, `feet`, `chest`, `knees`.
    pub parts: Vec<String>,
    /// The head's alignment: `once` (its rotation once, then its position
    /// each tick), `always` (both each tick), `off`.
    pub head: String,
    /// Moves trackers off the standing pose (metres: right, up, ahead), by
    /// `hip`, `left_foot`, ...: trying out that the body follows.
    pub shift: std::collections::BTreeMap<String, [f32; 3]>,
    /// Multiplies the trackers' positions (None: 1). VRChat takes them in
    /// the tracking space's metres, as the headset's (checked against the
    /// calibration mirror: at 1 the markers sit in the avatar's hips and
    /// ankles, at 0.8 the feet ride on the shins); for trying others.
    pub scale: Option<f32>,
    /// The legs (and arms) walk and run with the bot's moves (gait cycles);
    /// off: they stand.
    pub gait: bool,
    /// Calibrate by itself when VRChat tracks head and hands only though
    /// the trackers are on (after the game started), while nothing else
    /// moves the bot (`crate::calibrate::watch`).
    pub auto_calibrate: bool,
}

impl Default for TrackerSettings {
    fn default() -> Self {
        TrackerSettings { on: false, parts: vec!["hip".into(), "feet".into()], head: "once".into(), shift: Default::default(), scale: None, gait: true, auto_calibrate: true }
    }
}

impl TrackerSettings {
    pub fn parts(&self) -> anyhow::Result<Vec<Part>> {
        let mut out = Vec::new();
        for name in &self.parts {
            let parts = Part::named(name).ok_or_else(|| anyhow::anyhow!("no tracker part {name} (hip, feet, chest, knees)"))?;
            out.extend(parts.iter().filter(|p| !out.contains(*p)).copied().collect::<Vec<_>>());
        }
        anyhow::ensure!(["once", "always", "off"].contains(&self.head.as_str()), "head is once, always or off");
        for (key, v) in &self.shift {
            anyhow::ensure!(Part::ALL.iter().any(|p| p.key() == key), "no tracker {key} to shift");
            anyhow::ensure!(v.iter().all(|x| x.is_finite() && x.abs() <= 1.0), "shifts within 1 m");
        }
        if let Some(k) = self.scale {
            anyhow::ensure!(k.is_finite() && (0.1..=3.0).contains(&k), "scale is 0.1-3");
        }
        Ok(out)
    }
}

pub struct Anim {
    params: Mutex<AnimParams>,
    path: PathBuf,
    /// The bot's voice: (when it plays, loudness 0..1) per window.
    voice: Mutex<VecDeque<(Instant, f32)>>,
    /// What the last tick saw (speed, talking, holding still), for `/v1/anim`.
    pub live: Mutex<Value>,
    /// The head as last sent to the headset, the animation's on the
    /// owner's, and when (what the game's listener hears from: the speaker
    /// tracker turns head-relative directions with it).
    pub head: Mutex<Option<(Instant, vrc_vr::Pose)>>,
    pub trackers: Mutex<TrackerSettings>,
    /// `trackers.json` next to `anim.json`: the trackers stay as set across
    /// restarts (VRChat keeps its calibration while the game runs).
    trackers_path: PathBuf,
    /// Since when the trackers have been sent without a break.
    pub trackers_on_since: Mutex<Option<Instant>>,
    /// The head's rotation is to be sent (once: on the next tick).
    pub head_rotation_due: AtomicBool,
    /// The hands are set by hand: no overlay.
    pub manual_hands: AtomicBool,
    /// The headset's owner holds the hands where it put them (a little back
    /// while a follow looks about): the animation moves the rest.
    pub owner_hands: AtomicBool,
    /// The motion program playing (`crate::motion`): it moves the trackers,
    /// the headset and the hands instead of all the above.
    pub motion: Mutex<Option<crate::motion::Program>>,
    /// How far the head is leaned off where the body stands (tracking
    /// space, `/v1/vr/head`): the trackers stay under the body.
    pub head_lean: Mutex<[f32; 3]>,
}

/// What the trackers send: each tracker, the head, and the scale applied.
pub struct TrackerFrame {
    pub trackers: Vec<(Part, vrc_vr::Pose)>,
    pub head: vrc_vr::Pose,
    pub scale: f32,
}

impl Anim {
    pub fn new(path: PathBuf) -> Anim {
        let params = std::fs::read(&path)
            .ok()
            .and_then(|b| serde_json::from_slice::<AnimParams>(&b).map_err(|e| tracing::warn!("{}: {e}", path.display())).ok())
            .and_then(|p| p.check().map(|()| p).map_err(|e| tracing::warn!("{}: {e}", path.display())).ok())
            .unwrap_or_default();
        let trackers_path = path.with_file_name("trackers.json");
        let trackers = std::fs::read(&trackers_path)
            .ok()
            .and_then(|b| serde_json::from_slice::<TrackerSettings>(&b).map_err(|e| tracing::warn!("{}: {e}", trackers_path.display())).ok())
            .filter(|t| t.parts().map_err(|e| tracing::warn!("{}: {e}", trackers_path.display())).is_ok())
            .unwrap_or_default();
        Anim {
            params: Mutex::new(params),
            path,
            trackers_path,
            voice: Mutex::new(VecDeque::new()),
            live: Mutex::new(Value::Null),
            head: Mutex::new(None),
            trackers: Mutex::new(trackers),
            trackers_on_since: Mutex::new(None),
            head_rotation_due: AtomicBool::new(true),
            manual_hands: AtomicBool::new(false),
            owner_hands: AtomicBool::new(false),
            motion: Mutex::new(None),
            head_lean: Mutex::new([0.0; 3]),
        }
    }

    pub fn params(&self) -> AnimParams {
        self.params.lk().clone()
    }

    /// Merges `patch` into the parameters (and keeps them).
    pub fn tune(&self, patch: &Value) -> anyhow::Result<AnimParams> {
        let mut p = self.params.lk();
        let mut v = serde_json::to_value(&*p)?;
        let (Some(dst), Some(src)) = (v.as_object_mut(), patch.as_object()) else {
            anyhow::bail!("expected a JSON object of parameters");
        };
        if patch.get("reset").and_then(Value::as_bool) == Some(true) {
            *p = AnimParams::default();
        } else {
            for (k, val) in src {
                if !dst.contains_key(k) {
                    anyhow::bail!("no parameter {k}");
                }
                dst.insert(k.clone(), val.clone());
            }
            let tuned: AnimParams = serde_json::from_value(v)?;
            tuned.check().map_err(anyhow::Error::msg)?;
            *p = tuned;
        }
        std::fs::write(&self.path, serde_json::to_vec_pretty(&*p)?)?;
        Ok(p.clone())
    }

    /// The bot's voice `pcm` (s16le mono), playing from `at`.
    pub fn heard_bot(&self, pcm: &[u8], at: Instant) {
        let per_window = (SAMPLE_RATE as f64 * WINDOW.as_secs_f64()) as usize * 2;
        let mut voice = self.voice.lk();
        for (i, chunk) in pcm.chunks(per_window).enumerate() {
            let n = chunk.len() / 2;
            if n == 0 {
                continue;
            }
            let sum: f64 = chunk.chunks_exact(2).map(|s| (i16::from_le_bytes([s[0], s[1]]) as f64).powi(2)).sum();
            let rms = (sum / n as f64).sqrt() / 32768.0;
            // Speech runs 0.05-0.2 RMS: about 0.2-0.8.
            voice.push_back((at + WINDOW * i as u32, (rms * 4.0).min(1.0) as f32));
        }
        // Long queues of speech are fine; a stale tail is not.
        while voice.len() > 4000 {
            voice.pop_front();
        }
    }

    /// The bot's voice now: `None` while silent.
    fn voice_now(&self, now: Instant) -> Option<f32> {
        let mut voice = self.voice.lk();
        while voice.len() > 1 && voice[1].0 <= now {
            voice.pop_front();
        }
        match voice.front() {
            Some(&(at, level)) if at <= now && now < at + WINDOW => Some(level),
            Some(&(at, _)) if at + WINDOW <= now => {
                voice.pop_front();
                None
            }
            _ => None,
        }
    }

    /// The trackers of a body standing under the owner's head (where it
    /// stands when the head leans).
    pub fn tracker_frame(&self, settings: &TrackerSettings, state: &vrc_vr::remote::State) -> anyhow::Result<TrackerFrame> {
        let parts = settings.parts()?;
        let lean = *self.head_lean.lk();
        let standing_head = [state.head.position[0] - lean[0], state.head.position[1] - lean[1], state.head.position[2] - lean[2]];
        let mut poses = trackers::standing(&parts, standing_head, state.body_yaw, FLOOR_Y);
        trackers::shifted(&mut poses, state.body_yaw, |p| settings.shift.get(p.key()).copied());
        let scale = settings.scale.unwrap_or(1.0);
        let mut head = state.head;
        head.position = head.position.map(|x| x * scale);
        for (_, pose) in &mut poses {
            pose.position = pose.position.map(|x| x * scale);
        }
        Ok(TrackerFrame { trackers: poses, head, scale })
    }

    /// Sets the trackers (and keeps them).
    pub fn set_trackers(&self, settings: TrackerSettings) -> anyhow::Result<()> {
        settings.parts()?;
        std::fs::write(&self.trackers_path, serde_json::to_vec_pretty(&settings)?)?;
        *self.trackers.lk() = settings;
        self.head_rotation_due.store(true, Ordering::Relaxed);
        Ok(())
    }

    /// How long the trackers have been sent without a break.
    pub fn trackers_on_for(&self) -> Duration {
        self.trackers_on_since.lk().map(|t| t.elapsed()).unwrap_or_default()
    }

    /// Sends the trackers (`legs`: where a gait puts them instead of
    /// standing), with the headset where it is now (`hmd`, the animation on
    /// it): VRChat aligns the trackers to the head every frame, so a head
    /// that moves without the trackers knowing would drag the feet along.
    fn send_trackers(&self, bridge: &Bridge, state: &vrc_vr::remote::State, hmd: vrc_vr::Pose, legs: Option<Vec<(Part, vrc_vr::Pose)>>) {
        let settings = self.trackers.lk().clone();
        if !settings.on {
            *self.trackers_on_since.lk() = None;
            return;
        }
        self.trackers_on_since.lk().get_or_insert_with(Instant::now);
        let frame = match self.tracker_frame(&settings, state) {
            Ok(mut f) => {
                f.head = vrc_vr::Pose { orientation: hmd.orientation, position: hmd.position.map(|x| x * f.scale) };
                if let Some(legs) = legs {
                    f.trackers = legs;
                }
                f
            }
            Err(e) => {
                tracing::debug!("trackers: {e:#}");
                return;
            }
        };
        let rotation = match settings.head.as_str() {
            "always" => true,
            "once" => self.head_rotation_due.swap(false, Ordering::Relaxed),
            _ => false,
        };
        let head = (settings.head != "off").then_some(&frame.head);
        for m in trackers::messages(&frame.trackers, head, rotation) {
            if let Err(e) = bridge.osc.send_raw(&m) {
                tracing::debug!("trackers: {e:#}");
                break;
            }
        }
    }

    /// Sends the trackers a motion program placed (when they are on).
    fn send_placed_trackers(&self, bridge: &Bridge, placed: &vrc_vr::motion::Placed) {
        let settings = self.trackers.lk().clone();
        if !settings.on {
            *self.trackers_on_since.lk() = None;
            return;
        }
        self.trackers_on_since.lk().get_or_insert_with(Instant::now);
        let rotation = match settings.head.as_str() {
            "always" => true,
            "once" => self.head_rotation_due.swap(false, Ordering::Relaxed),
            _ => false,
        };
        let head = (settings.head != "off").then_some(&placed.head);
        for m in trackers::messages(&placed.trackers, head, rotation) {
            if let Err(e) = bridge.osc.send_raw(&m) {
                tracing::debug!("trackers: {e:#}");
                break;
            }
        }
    }

    pub fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(1);
        let mut animator = Animator::new(self.params(), seed);
        let mut watch = crate::calibrate::Watch::new();
        let mut usercam = crate::usercam::Watch::new();
        let mut link: Option<HmdLink> = None;
        let mut osc: Option<Osc> = None;
        let mut speed = 0.0f32;
        // Off the body's facing the walk goes (degrees, + right): the legs
        // step that way (`ground_motion`).
        let mut walk_off = 0.0f32;
        let mut eye_height = 0.0f32;
        let mut airborne = false;
        let mut gait = Gait::default();
        let mut feet = Feet::default();
        let mut tick = 0u32;
        let mut last = Instant::now();
        let mut off_sent = false;
        let mut tried = Instant::now().checked_sub(RECONNECT_EVERY).unwrap_or_else(Instant::now);
        loop {
            std::thread::sleep(TICK);
            tick = tick.wrapping_add(1);
            let now = Instant::now();
            let dt = (now - last).as_secs_f32();
            last = now;
            // The headset connection: the rig's; none yet (nothing used the
            // headset since the bridge started) or a dead one (Monado
            // restarted): the animation connects it, or the avatar stands
            // stiff until something else does.
            let owner = link.as_ref().and_then(HmdLink::owner);
            let Some(owner) = owner else {
                off_sent = false;
                if let Some(mut vr) = bridge.vr.try_lk() {
                    let dead = vr.link().as_ref().and_then(HmdLink::owner).is_none();
                    if dead && tried.elapsed() >= RECONNECT_EVERY {
                        tried = Instant::now();
                        vr.reset();
                        match vr.rig(&[]) {
                            Ok(_) => tracing::info!("animation: headset connected"),
                            Err(e) => tracing::debug!("animation: no headset yet: {e:#}"),
                        }
                    }
                    link = vr.link();
                }
                continue;
            };
            if tick.is_multiple_of(SPEED_EVERY) {
                if osc.is_none() {
                    osc = bridge.osc_query().ok();
                }
                let velocity = osc.as_ref().map(|o| (o.query("/avatar/parameters/VelocityZ"), o.query("/avatar/parameters/VelocityX")));
                match velocity {
                    Some((Ok(z), x)) => (speed, walk_off) = ground_motion(z as f32, x.map_or(0.0, |x| x as f32)),
                    _ => {
                        osc = None;
                        (speed, walk_off) = (0.0, 0.0);
                    }
                }
                // Off the ground (a jump, a fall): the legs tuck up.
                airborne = matches!(osc.as_ref().map(|o| o.query("/avatar/parameters/Grounded")), Some(Ok(g)) if g == 0.0);
                if tick.is_multiple_of(SIZE_EVERY) || eye_height == 0.0 {
                    if let Some(Ok(h)) = osc.as_ref().map(|o| o.eye_height()) {
                        eye_height = h as f32;
                    }
                }
            }
            animator.params = self.params();
            let enabled = animator.params.enabled;
            // Glances only when nobody else is using the head.
            let idle = bridge.follower.is_idle() && bridge.vr.try_lk().is_some();
            let voice = self.voice_now(now);
            let overlay = animator.update(&AnimInput { dt, owner: owner.state, speed, voice, idle });
            if tick.is_multiple_of(SPEED_EVERY) {
                *self.live.lk() = serde_json::json!({"speed": speed, "talking": voice.is_some(), "idle": idle, "still": owner.still});
            }
            if tick.is_multiple_of(CALIBRATION_EVERY) {
                watch.check(&bridge);
                usercam.check(&bridge);
            }
            let link = link.as_ref().unwrap();
            // A motion program plays on everything.
            let played = self.motion.lk().as_mut().map(|m| {
                let placed = m.tick(dt);
                let o = vrc_vr::remote::Overlay {
                    left: m.hand(0, placed.left),
                    right: m.hand(1, placed.right),
                    rest: overlay.rest,
                    head: Default::default(),
                    head_pose: Some(placed.head),
                };
                (placed, o)
            });
            if let Some((placed, o)) = played {
                *self.head.lk() = Some((now, placed.head));
                self.send_placed_trackers(&bridge, &placed);
                off_sent = false;
                if let Err(e) = link.set_overlay(Some(o)) {
                    tracing::debug!("animation: {e:#}");
                }
                continue;
            }
            // The legs walk while the bot moves (not while a scan holds
            // still or the hands are set by hand).
            let manual = self.manual_hands.load(Ordering::Relaxed);
            let stature = if eye_height > 0.0 { eye_height / 0.936 } else { 1.6 };
            let gait_on = self.trackers.lk().gait;
            let walking = if owner.still || manual || !gait_on { None } else { gait.update(&bridge.motions, speed, stature, dt, airborne) }.map(|body| {
                // The legs step the way the bot goes (a walk at a slant, a
                // step aside): stepping along the body's facing instead,
                // the feet crossed.
                let stand = vrc_vr::motion::Stand {
                    eyes: owner.state.head.position,
                    yaw_deg: owner.state.body_yaw + walk_off,
                    floor_y: FLOOR_Y,
                };
                let parts = self.trackers.lk().parts().unwrap_or_default();
                let attached = crate::motion::attachments(&stand, &parts, &animator.params);
                let standing = bridge.motions.get("walk_cycle").map(|c| c.standing).unwrap_or(body);
                vrc_vr::motion::place(&stand, &standing, &body, crate::motion::EYES_AHEAD, &attached)
            });
            // The arms swing with the legs (the same cycle, the same phase),
            // in place of the animation's own swing; the head stays its own.
            let mut overlay = overlay;
            if let Some(placed) = &walking {
                let w = gait.weight.min(1.0);
                overlay.left.pose = lerp_pose(&overlay.left.pose, &placed.left, w);
                overlay.right.pose = lerp_pose(&overlay.right.pose, &placed.right, w);
            }
            if self.owner_hands.load(Ordering::Relaxed) {
                overlay.left = owner.state.left;
                overlay.right = owner.state.right;
                overlay.rest = [owner.state.left, owner.state.right];
            }
            let mut legs = walking.map(|p| p.trackers);
            // Standing (no gait): the feet stay put and step after a turn
            // (a scan's too).
            if legs.is_some() || manual {
                feet.reset();
            } else if let Ok(parts) = self.trackers.lk().parts() {
                let lean = *self.head_lean.lk();
                let head = owner.state.head.position;
                let standing_head = [head[0] - lean[0], head[1] - lean[1], head[2] - lean[2]];
                let wanted = trackers::standing(&parts, standing_head, owner.state.body_yaw, FLOOR_Y);
                let pick = |p: Part| wanted.iter().find(|(q, _)| *q == p).map(|(_, x)| *x);
                if let (Some(l), Some(r)) = (pick(Part::LeftFoot), pick(Part::RightFoot)) {
                    let ([l, r], hips_yaw) = feet.update([l, r], standing_head, owner.state.body_yaw, dt);
                    // The rest (hips, chest, knees) turned as far as the hips.
                    let mut stood = trackers::standing(&parts, standing_head, hips_yaw, FLOOR_Y);
                    for (part, pose) in stood.iter_mut() {
                        match part {
                            Part::LeftFoot => *pose = l,
                            Part::RightFoot => *pose = r,
                            _ => {}
                        }
                    }
                    legs = Some(stood);
                }
            }
            // Where the headset is: the animation's head on the owner's,
            // unless it is off (or holding still: then the owner's alone).
            let hmd = if enabled && !manual && !owner.still { overlay.head.apply(owner.state.head) } else { owner.state.head };
            *self.head.lk() = Some((now, hmd));
            self.send_trackers(&bridge, &owner.state, hmd, legs);
            let sent = if enabled && !manual {
                off_sent = false;
                link.set_overlay(Some(overlay))
            } else if !off_sent {
                off_sent = true;
                link.set_overlay(None)
            } else {
                Ok(())
            };
            if let Err(e) = sent {
                tracing::debug!("animation: {e:#}");
            }
        }
    }
}

/// The gait from the avatar's velocity (its own axes: `z` ahead, `x`
/// right): the speed over the ground (backwards when it goes back) and the
/// way off the body's facing the legs step (degrees, + right; at most
/// WALK_OFF_MAX_DEG, a step aside walking at that slant). A step aside walks
/// the legs too (it slid with `VelocityZ` alone, D44), and they step the
/// way it goes (stepping ahead while going aside, the feet crossed, D46).
fn ground_motion(z: f32, x: f32) -> (f32, f32) {
    let over = z.hypot(x);
    if over < 1e-3 {
        return (0.0, 0.0);
    }
    let way = x.atan2(z).to_degrees();
    if way.abs() <= 100.0 {
        (over, way.clamp(-WALK_OFF_MAX_DEG, WALK_OFF_MAX_DEG))
    } else {
        // Going back: the cycle backwards, along the way back.
        let back = (way + 360.0) % 360.0 - 180.0;
        (-over, back.clamp(-WALK_OFF_MAX_DEG, WALK_OFF_MAX_DEG))
    }
}

/// How far off the body's facing the legs step at most (degrees).
const WALK_OFF_MAX_DEG: f32 = 60.0;

#[cfg(test)]
mod feet_tests {
    use super::*;

    /// The feet where the body would stand them, facing `yaw` (0.2 m apart).
    fn wanted(yaw: f32) -> [vrc_vr::Pose; 2] {
        let f = vrc_vr::Pose::looking(yaw, 0.0, [0.0; 3]);
        let r = f.rotate([1.0, 0.0, 0.0]);
        [-0.1f32, 0.1].map(|side| vrc_vr::Pose::looking(yaw, 0.0, [r[0] * side, 0.0, r[2] * side]))
    }

    /// Where the left foot is from the right, across the hips (+: to the
    /// left of it, as it should be).
    fn apart_across(feet: &[vrc_vr::Pose; 2], hips: f32) -> f32 {
        let r = vrc_vr::Pose::looking(hips, 0.0, [0.0; 3]).rotate([1.0, 0.0, 0.0]);
        let d = [feet[1].position[0] - feet[0].position[0], feet[1].position[2] - feet[0].position[2]];
        d[0] * r[0] + d[1] * r[2]
    }

    #[test]
    fn a_step_aside_walks_the_legs_the_way_it_goes() {
        assert_eq!(ground_motion(1.0, 0.0), (1.0, 0.0));
        let (s, off) = ground_motion(1.0, 1.0);
        assert!((s - 2f32.sqrt()).abs() < 1e-5 && (off - 45.0).abs() < 1e-3, "a slant: the legs that way");
        let (s, off) = ground_motion(0.0, 0.5);
        assert!((s - 0.5).abs() < 1e-6 && (off - WALK_OFF_MAX_DEG).abs() < 1e-3, "aside: at most the slant");
        let (s, off) = ground_motion(0.0, -0.5);
        assert!((s - 0.5).abs() < 1e-6 && (off + WALK_OFF_MAX_DEG).abs() < 1e-3);
        let (s, off) = ground_motion(-1.0, 0.2);
        assert!((s + 1.0198).abs() < 1e-3 && (off + 11.31).abs() < 0.05, "back: backwards, slanting the same way as the step: {off}");
        assert_eq!(ground_motion(0.0, 0.0), (0.0, 0.0));
    }

    #[test]
    fn a_turn_round_at_once_never_crosses_the_legs() {
        for turn in [180.0f32, -170.0, 120.0, -90.0] {
            let mut feet = Feet::default();
            let centre = [0.0, 1.5, 0.0];
            let dt = 0.04;
            for _ in 0..10 {
                feet.update(wanted(0.0), centre, 0.0, dt);
            }
            let mut last = None;
            let mut done = None;
            for k in 0..100 {
                let (now, hips) = feet.update(wanted(turn), centre, turn, dt);
                let across = apart_across(&now, hips);
                assert!(across > 0.05, "turn {turn}, tick {k}: the legs cross ({across:.3} m, hips {hips:.0})");
                for f in &now {
                    let twist = turn_between(hips, yaw_of(f)).abs();
                    assert!(twist <= PIVOT_DEG + HIPS_AHEAD_FEET_DEG + 1.0, "turn {turn}, tick {k}: a foot {twist:.0} deg off the hips");
                }
                if done.is_none() && turn_between(hips, turn).abs() < 2.0 && now.iter().all(|f| turn_between(yaw_of(f), turn).abs() < STEP_TURN_DEG + 1.0) {
                    done = Some(k as f32 * dt);
                }
                last = Some((now, hips));
            }
            // About a second for a turn round (six steps).
            let took = done.expect("round in the end");
            assert!(took <= 1.6, "turn {turn}: {took:.2} s");
            eprintln!("turn {turn}: round in {took:.2} s");
            // Round in the end: the hips and both feet facing the new way.
            let (now, hips) = last.unwrap();
            assert!(turn_between(hips, turn).abs() < 2.0, "turn {turn}: hips at {hips}");
            for f in &now {
                assert!(turn_between(yaw_of(f), turn).abs() < STEP_TURN_DEG + 1.0, "turn {turn}: a foot at {}", yaw_of(f));
            }
        }
    }
}

#[cfg(test)]
mod gait_tests {
    use super::*;

    /// A cycle of `n` frames standing, the hips turning `turn_deg` over it
    /// (+ left).
    fn clip(n: usize, turn_deg: f32) -> vrc_vr::motion::Clip {
        let standing = serde_json::json!({"hips":[0,0.53,0],"chest":[0,0.72,0],"head":[0,0.868,0],"l_leg":[0.0955,0.285,0],"r_leg":[-0.0955,0.285,0],"l_foot":[0.0955,0.039,0],"r_foot":[-0.0955,0.039,0],"l_forearm":[0.16,0.6,0],"r_forearm":[-0.16,0.6,0],"l_hand":[0.18,0.45,0],"r_hand":[-0.18,0.45,0]});
        let frames: Vec<Vec<f32>> = (0..n)
            .map(|i| {
                let a = (turn_deg * i as f32 / n as f32).to_radians() / 2.0;
                let mut row = Vec::new();
                for (k, j) in vrc_vr::motion::JOINTS.iter().enumerate() {
                    let p = standing[j].as_array().unwrap().iter().map(|x| x.as_f64().unwrap() as f32).collect::<Vec<_>>();
                    let rot = if k == J_HIPS { [0.0, a.sin(), 0.0, a.cos()] } else { [0.0, 0.0, 0.0, 1.0] };
                    row.extend([p[0], p[1], p[2] + 0.03 * i as f32]);
                    row.extend(rot);
                }
                row
            })
            .collect();
        let json = serde_json::json!({"name": "cycle", "fps": 30.0, "loop": true, "root": "in_place", "joints": vrc_vr::motion::JOINTS, "standing": standing, "frames": frames, "speed": 0.7});
        vrc_vr::motion::Clip::parse(json.to_string().as_bytes()).unwrap()
    }

    /// The legs walking `secs` at `speed` (m/s, 1.6 m tall): each tick's
    /// body.
    fn walk(speed: f32, secs: f32) -> Vec<vrc_vr::motion::Body> {
        let c = clip(32, 0.0);
        let mut g = Gait::default();
        (0..(secs / 0.022) as usize).filter_map(|_| g.step(&c, &c, None, speed, 1.6, 0.022, false)).collect()
    }

    #[test]
    fn a_foot_on_the_ground_stays_put() {
        for speed in [1.0, 2.2, 4.0] {
            let bodies = walk(speed, 5.0);
            let floor = 0.039;
            let mut held = 0;
            for w in bodies[150..].windows(2) {
                for f in J_FEET {
                    let (a, b) = (w[0][f].pos, w[1][f].pos);
                    // Flat on the floor both ticks: it moved back as fast as
                    // the floor went by.
                    if a[1] < floor + 1e-4 && b[1] < floor + 1e-4 {
                        let moved = b[2] - a[2];
                        assert!((moved + speed / 1.6 * 0.022).abs() < 1e-3, "{speed} m/s: moved {moved}");
                        held += 1;
                    }
                }
            }
            assert!(held > 20, "{speed} m/s: {held}");
        }
    }

    #[test]
    fn the_feet_lift_take_turns_and_step_as_people_do() {
        for (speed, high, most) in [(1.2, 0.045, 1.6), (4.0, 0.08, 2.4)] {
            let bodies = walk(speed, 6.0);
            let ups: Vec<[f32; 2]> = bodies[150..].iter().map(|b| J_FEET.map(|f| b[f].pos[1] - 0.039)).collect();
            for side in 0..2 {
                let top = ups.iter().map(|u| u[side]).fold(0.0, f32::max);
                assert!(top > high, "{speed} m/s, foot {side}: lifts {top}");
            }
            if speed < 2.0 {
                // Walking, one foot is always on the floor.
                assert!(ups.iter().all(|u| u[0].min(u[1]) < 0.03), "{speed} m/s: both feet up");
            }
            // Cycles a second: the left foot's lifts.
            let lifts = ups.windows(2).filter(|w| w[0][0] < 0.03 && w[1][0] >= 0.03).count() as f32;
            let secs = ups.len() as f32 * 0.022;
            assert!(lifts / secs > 1.0 && lifts / secs < most, "{speed} m/s: {} cycles a second", lifts / secs);
        }
    }

    #[test]
    fn the_toes_point_down_as_the_foot_leaves_and_up_as_it_lands() {
        let bodies = walk(1.5, 4.0);
        let toes = |b: &vrc_vr::motion::Body, f: usize| vrc_vr::Pose { orientation: b[f].rot, position: [0.0; 3] }.rotate([0.0, 0.0, 1.0])[1];
        let mut seen = (false, false);
        for w in bodies[100..].windows(2) {
            let (a, b) = (w[0][J_FEET[0]].pos[1], w[1][J_FEET[0]].pos[1]);
            if a < 0.039 + 0.02 && b >= 0.039 + 0.02 {
                assert!(toes(&w[1], J_FEET[0]) < -0.15, "leaving: {}", toes(&w[1], J_FEET[0]));
                seen.0 = true;
            }
            if a > 0.039 + 1e-4 && b <= 0.039 + 1e-4 {
                assert!(toes(&w[1], J_FEET[0]) > 0.1, "landing: {}", toes(&w[1], J_FEET[0]));
                seen.1 = true;
            }
        }
        assert!(seen.0 && seen.1, "{seen:?}");
    }

    #[test]
    fn the_knees_bend_forward_and_the_legs_reach() {
        for b in &walk(2.0, 4.0)[100..] {
            for i in 0..2 {
                let (hips, knee, foot) = (b[J_HIPS].pos, b[J_KNEES[i]].pos, b[J_FEET[i]].pos);
                // Ahead of the line from the hip to the ankle.
                let t = (knee[1] - hips[1]) / (foot[1] - hips[1]);
                assert!(knee[2] >= hips[2] + (foot[2] - hips[2]) * t - 1e-3, "{knee:?} {hips:?} {foot:?}");
                let reach = (foot[2] - hips[2]).hypot(foot[1] - hips[1]);
                assert!(reach < 0.53 - 0.039 + 0.05, "{reach}");
            }
        }
    }

    #[test]
    fn the_feet_rest_on_the_ground_shorter_the_faster() {
        let c = clip(32, 0.0);
        let mut last = 1.0;
        for speed in [0.6, 1.2, 1.8, 3.0, 4.0] {
            let mut g = Gait::default();
            let mut down = 0;
            for t in 0..400 {
                g.step(&c, &c, None, speed, 1.6, 0.022, false);
                if t >= 150 && g.feet.unwrap()[0].down {
                    down += 1;
                }
            }
            let share = down as f32 / 250.0;
            assert!(share < last, "{speed} m/s: {share} on the ground, slower {last}");
            last = share;
        }
        assert!(last < 0.36, "{last}");
    }

    #[test]
    fn the_captured_turn_is_taken_out() {
        let c = clip(24, 17.0);
        let yaw = |b: &vrc_vr::motion::Body| {
            let a = vrc_vr::Pose { orientation: b[J_HIPS].rot, position: [0.0; 3] }.rotate([0.0, 0.0, 1.0]);
            a[0].atan2(a[2]).to_degrees()
        };
        for k in 0..50 {
            let b = cycle_pose(&c, k as f32 / 50.0);
            assert!(yaw(&b).abs() < 0.5, "{k}: {}", yaw(&b));
            // In place: the hips where they stand.
            assert!(b[J_HIPS].pos[2].abs() < 1e-4 && b[J_HIPS].pos[0].abs() < 1e-4);
        }
    }
}
