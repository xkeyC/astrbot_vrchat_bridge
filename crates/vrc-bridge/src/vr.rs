//! The virtual headset's capabilities for the HTTP API: surveys and walks
//! (`vrc-nav`), turning and tilting the head, short measured steps, frames
//! of the eyes. Everything here blocks (the head moves, frames are waited
//! for): callers run it on a blocking thread, one at a time.

use std::time::Duration;

use anyhow::{bail, Context, Result};
use serde_json::{json, Value};
use vrc_nav::{GotoOptions, Rig, Survey, SurveyOptions};
use vrc_vr::osc::Osc;
use vrc_vr::scan;
use vrc_vr::tap::EyeFrame;
use vrc_vr::walk::{self, WalkParams};
use vrc_vr::Pose;

use crate::Args;

pub const STEP_MAX_M: f32 = 5.0;

/// The forward stick for a pace: `walk` (about 0.9 m/s) or `run` (about
/// 1.35 m/s): near the gait cycles' own speeds, so the steps keep their
/// natural rate; none: the walks' default.
pub fn pace_axis(pace: Option<&str>) -> Result<Option<f32>> {
    Ok(match pace {
        None => None,
        Some("walk") => Some(0.3),
        Some("run") => Some(0.4),
        Some(p) => bail!("pace is walk or run, not {p}"),
    })
}
pub use vrc_vr::anim::{MAX_HEAD_HEIGHT, MIN_HEAD_HEIGHT};

pub struct VrCore {
    remote: String,
    tap: String,
    ocr_url: String,
    ocr_model: String,
    rig: Option<Rig>,
    pub survey: Option<Survey>,
    pub serial: u64,
    /// Where the head looks (degrees): turns and tilts keep it.
    pub yaw: f32,
    pub pitch: f32,
    /// The head's height above the tracking space's floor (metres): where
    /// VRChat's calibration expects it, else the avatar stands on tiptoe
    /// (too high) or crouches.
    pub head_height: f32,
    /// The head leaned off where it stands (`lean_head`).
    pub lean: [f32; 3],
}

impl VrCore {
    pub fn new(args: &Args) -> VrCore {
        VrCore {
            remote: args.remote.clone(),
            tap: args.tap.clone(),
            ocr_url: args.ocr_url.clone(),
            ocr_model: args.ocr_model.clone(),
            rig: None,
            survey: None,
            serial: 0,
            yaw: 0.0,
            pitch: 0.0,
            head_height: vrc_vr::anim::AnimParams::default().head_height,
            lean: [0.0; 3],
        }
    }

    /// The rig, connected on first use (and again after a failure: `reset`).
    pub fn rig(&mut self, whitelist: &[String]) -> Result<&mut Rig> {
        if self.rig.is_none() {
            let mut rig = Rig::connect(&self.remote, &self.tap, Some(self.ocr_url.as_str()), &self.ocr_model, Vec::new())?;
            let (yaw, pitch) = rig.hmd.state.head.yaw_pitch();
            self.yaw = yaw;
            self.pitch = pitch;
            rig.hmd.state.head.position[1] = self.head_height;
            self.lean = [0.0; 3];
            let head = rig.hmd.state.head.position;
            rig.hmd.state.hands_at_rest(head, yaw);
            rig.hmd.send()?;
            self.rig = Some(rig);
        }
        let rig = self.rig.as_mut().unwrap();
        rig.whitelist = whitelist.to_vec();
        // OSCQuery may have come up (or moved) since: VRChat restarts.
        if rig.osc.as_ref().is_none_or(|o| o.eye_height().is_err()) {
            rig.osc = Osc::connect().ok();
        }
        Ok(rig)
    }

    /// Moves the head to `metres` above the floor (the hands with it).
    pub fn set_head_height(&mut self, metres: f32) -> Result<()> {
        self.head_height = metres;
        if let Some(rig) = self.rig.as_mut() {
            rig.hmd.state.head.position[1] = metres;
            let (head, body) = (rig.hmd.state.head.position, rig.hmd.state.body_yaw);
            rig.hmd.state.hands_at_rest(head, body);
            rig.hmd.send()?;
        }
        Ok(())
    }

    /// The headset connection's animator handle, once the rig is up.
    pub fn link(&self) -> Option<vrc_vr::remote::HmdLink> {
        self.rig.as_ref().map(|r| r.hmd.link())
    }

    /// Drops the rig: the next use connects again.
    pub fn reset(&mut self) {
        self.rig = None;
    }

    /// Points the head (degrees; pitch clamped to +-80).
    pub fn aim(&mut self, yaw: f32, pitch: f32) -> Result<()> {
        anyhow::ensure!(yaw.is_finite() && pitch.is_finite(), "angles must be finite numbers");
        let rig = self.rig(&[])?;
        let head = rig.hmd.state.head.position;
        let pitch = pitch.clamp(-80.0, 80.0);
        rig.hmd.set_head(Pose::looking(yaw, pitch, head))?;
        self.yaw = (yaw + 540.0).rem_euclid(360.0) - 180.0;
        self.pitch = pitch;
        Ok(())
    }

    /// Turns the whole bot round to `yaw` (degrees) gently, in `secs`: head
    /// and body together, eased in and out, the head tilting `tilt` degrees
    /// aside on the way and level again at the end (a look about for
    /// someone), and raising it `nod` degrees on the way (and down again);
    /// the hands a little back of rest (out of the view). The feet step after the body (the
    /// animation's).
    pub fn turn_gently(&mut self, yaw: f32, pitch: f32, secs: f32, tilt: f32, nod: f32) -> Result<()> {
        let (y0, p0) = (self.yaw, self.pitch);
        let d = vrc_vr::scan::angle_diff(yaw, y0);
        let steps = (secs.max(0.1) / 0.022).ceil() as usize;
        for i in 1..=steps {
            let w = i as f32 / steps as f32;
            let e = w * w * (3.0 - 2.0 * w);
            let (y, p) = (y0 + d * e, p0 + (pitch - p0) * e);
            let bump = (std::f32::consts::PI * w).sin();
            let roll = tilt * bump;
            let p = p + nod * bump;
            let rig = self.rig(&[])?;
            let at = rig.hmd.state.head.position;
            rig.hmd.state.hands_back_a_little(at, y);
            let look = Pose::looking(y, p, at);
            let tilted = vrc_vr::pose::quat_mul(look.orientation, vrc_vr::pose::quat_axis([0.0, 0.0, 1.0], -roll.to_radians()));
            rig.hmd.set_head(Pose { orientation: tilted, position: at })?;
            std::thread::sleep(Duration::from_millis(22));
        }
        self.yaw = (yaw + 540.0).rem_euclid(360.0) - 180.0;
        self.pitch = pitch;
        Ok(())
    }

    /// Turns the whole bot: the head (degrees; pitch clamped to +-80) and
    /// the body under it (the hands at its sides).
    /// The body facing `body_yaw` (the hands at its sides, the feet
    /// stepping after it), the head looking along `yaw`, `pitch` (degrees).
    pub fn face_and_look(&mut self, body_yaw: f32, yaw: f32, pitch: f32) -> Result<()> {
        let rig = self.rig(&[])?;
        let head = rig.hmd.state.head.position;
        rig.hmd.state.hands_at_rest(head, body_yaw);
        self.aim(yaw, pitch)
    }

    pub fn face(&mut self, yaw: f32, pitch: f32) -> Result<()> {
        let rig = self.rig(&[])?;
        let head = rig.hmd.state.head.position;
        rig.hmd.state.hands_at_rest(head, yaw);
        self.aim(yaw, pitch)
    }

    /// Sets a hand by hand (`/v1/vr/hand`): its grip `offset` from the eyes
    /// (metres: right, up, ahead of where the body faces), turned `turn`
    /// (degrees: yaw, pitch, roll from the body's facing; the grip's -Z is
    /// where it points), the trigger pulled `trigger` (0..1) and `buttons`
    /// held. Returns the grip pose sent.
    pub fn set_hand(&mut self, hand: &str, offset: [f32; 3], turn: [f32; 3], trigger: f32, buttons: &[String]) -> Result<Pose> {
        for name in buttons {
            anyhow::ensure!(["a", "b", "system", "thumbstick"].contains(&name.as_str()), "buttons are a, b, system, thumbstick");
        }
        let rig = self.rig(&[])?;
        let state = &mut rig.hmd.state;
        let (head, body) = (state.head.position, state.body_yaw);
        let facing = Pose::looking(body, 0.0, [0.0; 3]);
        let (right, ahead) = (facing.rotate([1.0, 0.0, 0.0]), facing.rotate([0.0, 0.0, -1.0]));
        let position = [
            head[0] + right[0] * offset[0] + ahead[0] * offset[2],
            head[1] + offset[1],
            head[2] + right[2] * offset[0] + ahead[2] * offset[2],
        ];
        let look = Pose::looking(body + turn[0], turn[1], position);
        let roll = vrc_vr::pose::quat_axis([0.0, 0.0, 1.0], -turn[2].to_radians());
        let pose = Pose { orientation: vrc_vr::pose::quat_mul(look.orientation, roll), position };
        let c = match hand {
            "left" => &mut state.left,
            "right" => &mut state.right,
            _ => bail!("hand is left or right"),
        };
        c.active = true;
        c.pose = pose;
        c.trigger = trigger.clamp(0.0, 1.0);
        c.trigger_touch = trigger > 0.0;
        c.trigger_click = trigger >= 0.95;
        let held = |name: &str| buttons.iter().any(|b| b == name);
        (c.a_click, c.a_touch) = (held("a"), held("a"));
        (c.b_click, c.b_touch) = (held("b"), held("b"));
        (c.system_click, c.system_touch) = (held("system"), held("system"));
        (c.thumbstick_click, c.thumbstick_touch) = (held("thumbstick"), held("thumbstick"));
        rig.hmd.send()?;
        Ok(pose)
    }

    /// Leans the head `lean` (metres: right, up, ahead of where the body
    /// faces) off where it stands (`/v1/vr/head`: looking past what the
    /// avatar wears to its own feet); `[0, 0, 0]` stands straight again.
    /// Returns the head, and how far it is off (tracking space).
    pub fn lean_head(&mut self, lean: [f32; 3]) -> Result<([f32; 3], [f32; 3])> {
        let old = self.lean;
        let rig = self.rig(&[])?;
        let state = &mut rig.hmd.state;
        let facing = Pose::looking(state.body_yaw, 0.0, [0.0; 3]);
        let (right, ahead) = (facing.rotate([1.0, 0.0, 0.0]), facing.rotate([0.0, 0.0, -1.0]));
        let offset = |l: [f32; 3]| [right[0] * l[0] + ahead[0] * l[2], l[1], right[2] * l[0] + ahead[2] * l[2]];
        let (was, now) = (offset(old), offset(lean));
        for i in 0..3 {
            state.head.position[i] += now[i] - was[i];
        }
        let head = state.head.position;
        rig.hmd.send()?;
        self.lean = lean;
        Ok((head, now))
    }

    /// Bends forward at the hips by `deg` (0: straight), hands clasped
    /// behind the back (out of the view): the head goes ahead and down as
    /// a body's does, about the hip joints (Drillis & Contini: 0.530 of
    /// the stature, the eyes at 0.936), so it neither floats nor stretches.
    /// Returns the head, and how far it is off where it stands.
    pub fn bend_over(&mut self, deg: f32) -> Result<([f32; 3], [f32; 3])> {
        let stature = (self.head_height - vrc_vr::remote::FLOOR_Y) / 0.936;
        let (s, c) = deg.to_radians().sin_cos();
        // (up, ahead) from the hip joints, turned forward about them.
        let bend = |up: f32, ahead: f32| (up * c - ahead * s, ahead * c + up * s);
        let eyes = ((0.936 - 0.530) * stature, 0.02 * stature);
        let (up, ahead) = bend(eyes.0, eyes.1);
        let out = self.lean_head([0.0, up - eyes.0, ahead - eyes.1])?;
        if deg == 0.0 {
            return Ok(out);
        }
        // The hands at the small of the back, a little apart.
        let (hand_up, hand_ahead) = bend(0.06 * stature, -0.11 * stature);
        let rig = self.rig(&[])?;
        let state = &mut rig.hmd.state;
        let body = state.body_yaw;
        let facing = Pose::looking(body, 0.0, [0.0; 3]);
        let (right, fwd) = (facing.rotate([1.0, 0.0, 0.0]), facing.rotate([0.0, 0.0, -1.0]));
        // The hip joints under the standing head.
        let head = [state.head.position[0] - out.1[0], state.head.position[1] - out.1[1], state.head.position[2] - out.1[2]];
        let hip = [head[0] - fwd[0] * eyes.1, head[1] - eyes.0, head[2] - fwd[2] * eyes.1];
        for (hand, side) in [(&mut state.left, -1.0f32), (&mut state.right, 1.0)] {
            let r = side * 0.05 * stature;
            hand.active = true;
            hand.pose = Pose::looking(
                body + side * 90.0,
                -60.0,
                [hip[0] + right[0] * r + fwd[0] * hand_ahead, hip[1] + hand_up, hip[2] + right[2] * r + fwd[2] * hand_ahead],
            );
        }
        rig.hmd.send()?;
        Ok(out)
    }

    /// Sets a hand's grip (tracking space) and its trigger (0..1).
    pub fn set_hand_pose(&mut self, hand: &str, pose: Pose, trigger: f32) -> Result<()> {
        let rig = self.rig(&[])?;
        let c = match hand {
            "left" => &mut rig.hmd.state.left,
            "right" => &mut rig.hmd.state.right,
            _ => bail!("hand is left or right"),
        };
        c.active = true;
        c.pose = pose;
        c.trigger = trigger.clamp(0.0, 1.0);
        c.trigger_touch = trigger > 0.0;
        c.trigger_click = trigger >= 0.95;
        rig.hmd.send()
    }

    /// Lets go of both hands' triggers and buttons, keeping where they are.
    pub fn release_hands(&mut self) -> Result<()> {
        let rig = self.rig(&[])?;
        for c in [&mut rig.hmd.state.left, &mut rig.hmd.state.right] {
            (c.trigger, c.trigger_touch, c.trigger_click) = (0.0, false, false);
            (c.a_click, c.a_touch, c.b_click, c.b_touch) = (false, false, false, false);
            (c.system_click, c.system_touch, c.thumbstick_click, c.thumbstick_touch) = (false, false, false, false);
        }
        rig.hmd.send()
    }

    // -- surveys and walks --------------------------------------------------------

    /// Looks: all around (`around`), else only ahead (and down at the feet).
    pub fn survey(&mut self, whitelist: &[String], players: bool, around: bool) -> Result<Value> {
        let opts = SurveyOptions { players, ahead: !around, ..Default::default() };
        let s = vrc_nav::survey(self.rig(whitelist)?, &opts, &[])?;
        self.serial += 1;
        self.yaw = s.yaw;
        let v = survey_json(self.serial, &s);
        self.survey = Some(s);
        Ok(v)
    }

    /// Walks to a place of the last look around or a bearing; `since`: the
    /// stops when it was asked for ([`walk::stops`]). Then looks ahead (all
    /// around with `around`).
    pub fn goto(&mut self, whitelist: &[String], input: &Value, since: u64) -> Result<Value> {
        // A place number of a look around the caller did not see (another
        // caller looked since) would lead elsewhere.
        if input["candidate"].is_u64() {
            if let Some(seen) = input["survey"].as_u64() {
                anyhow::ensure!(seen == self.serial, "the numbered places no longer hold (you moved, or looked again since): look again");
            }
        }
        // A fresh survey to plan from (people move, and so may the bot).
        let s = vrc_nav::survey(self.rig(whitelist)?, &SurveyOptions { players: false, ..Default::default() }, &[])?;
        let target = if let Some(id) = input["candidate"].as_u64() {
            let seen = self.survey.as_ref().context("no view to pick a place from: look first")?;
            let c = seen.candidates.iter().find(|c| c.id as u64 == id).context("no such place in the last view")?;
            [c.position[0], c.position[2]]
        } else {
            let bearing = input["bearing"].as_f64().context("a candidate, or a bearing and a distance")? as f32;
            let distance = input["distance"].as_f64().unwrap_or(2.0) as f32;
            anyhow::ensure!(bearing.is_finite() && distance.is_finite() && (0.0..=30.0).contains(&distance), "a bearing, and a distance of at most 30 m");
            let yaw = (s.yaw + bearing).to_radians();
            let d = distance / s.metres;
            [s.eye[0] + yaw.sin() * d, s.eye[2] - yaw.cos() * d]
        };
        // Moving: the places' numbers stop holding (here and on failure). A
        // stop since it was asked for ends it before the first leg.
        self.forget_places();
        let mut opts = GotoOptions { since: Some(since), ..Default::default() };
        if let Some(a) = pace_axis(input["pace"].as_str())? {
            opts.walk.axis = a;
        }
        let report = vrc_nav::goto(self.rig(whitelist)?, s, target, &opts)?;
        let around = input["around"].as_bool().unwrap_or(false);
        let after = vrc_nav::survey(self.rig(whitelist)?, &SurveyOptions { ahead: !around, ..Default::default() }, &[])?;
        self.serial += 1;
        self.yaw = after.yaw;
        let mut v = json!({
            "arrived": report.arrived,
            "remaining_m": r2(report.remaining),
            "took_s": (report.took.as_secs_f64() * 10.0).round() / 10.0,
            "reason": report.reason,
            "legs": report.legs.iter().map(|l| json!({
                "heading_deg": l.yaw.round() as f64,
                "planned_m": r2(l.planned),
                "walked_m": r2(l.walked),
                "blocked": l.blocked,
            })).collect::<Vec<_>>(),
        });
        v["after"] = survey_json(self.serial, &after);
        self.survey = Some(after);
        Ok(v)
    }

    // -- small moves ------------------------------------------------------------------

    /// The numbered places of the last look around no longer hold (the bot
    /// moved, or the headset was reset): a walk to a number needs a new
    /// look around.
    pub fn forget_places(&mut self) {
        self.serial += 1;
        self.survey = None;
    }

    /// /v1/step: turns by `turn` degrees, then walks `meters` (world) a way
    /// (forward, back, left, right of where it then faces; still facing
    /// there: back steps back, left and right step aside), jumping as it
    /// starts if asked. `since`: the
    /// stops when it was asked for ([`walk::stops`]).
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, osc_jump: impl Fn(), turn: f32, direction: &str, meters: f32, jump: bool, since: u64, axis: Option<f32>) -> Result<Value> {
        let offset = match direction {
            "forward" => 0.0,
            "back" => 180.0,
            "left" => -90.0,
            "right" => 90.0,
            _ => bail!("direction must be forward, back, left or right"),
        };
        let facing = self.yaw + turn;
        if turn != 0.0 || meters == 0.0 {
            self.face(facing, 0.0)?;
        }
        if jump {
            osc_jump();
        }
        if meters <= 0.0 {
            return Ok(json!({"ok": true, "turned": turn, "blocked": false, "moved": {"ahead_m": 0.0, "right_m": 0.0}}));
        }
        let rig = self.rig(&[])?;
        let osc = rig.osc.as_ref().context("walking needs VRChat's OSC")?;
        let osc = Osc::with_ports_from(osc)?;
        let mut params = WalkParams::default();
        if let Some(a) = axis {
            params.axis = a;
        }
        let leg = walk::leg_facing(&mut rig.hmd, &osc, facing, offset, meters.min(STEP_MAX_M), &params, since);
        self.forget_places();
        let leg = leg?;
        self.yaw = (facing + 540.0).rem_euclid(360.0) - 180.0;
        self.pitch = 0.0;
        let (s, c) = offset.to_radians().sin_cos();
        Ok(json!({
            "ok": true, "turned": turn, "blocked": leg.blocked, "stopped": leg.stopped,
            "moved": {"ahead_m": r1(leg.walked * c), "right_m": r1(leg.walked * s)},
        }))
    }

    /// The first frame with the head tilted to `pitch` (degrees, + up).
    pub fn frame_looking(&mut self, pitch: f32) -> Result<EyeFrame> {
        let yaw = self.yaw;
        self.rig(&[])?.hmd.hold_still(true)?;
        let frame = self.aim(yaw, pitch).and_then(|()| {
            let pitch = self.pitch;
            scan::rendered_at(&mut self.rig(&[])?.tap, yaw, pitch, Duration::from_secs(1))
        });
        self.rig(&[])?.hmd.hold_still(false)?;
        frame
    }

    /// The latest frame of the eyes.
    pub fn frame(&mut self) -> Result<EyeFrame> {
        let rig = self.rig(&[])?;
        rig.tap.read()?.context("no frame yet (is the game in VR mode?)")
    }
}

/// The left eye of `frame` as JPEG, `width` pixels wide (0: as is).
pub fn eye_jpeg(frame: &EyeFrame, width: u32) -> Result<Vec<u8>> {
    let rgb = frame.eye_rgb8(0)?;
    let (w, h) = (frame.width as usize, frame.height as usize);
    let (out_w, out_h, data) = if width > 0 && (width as usize) < w {
        let ow = width as usize;
        let oh = h * ow / w;
        let mut out = vec![0u8; ow * oh * 3];
        for y in 0..oh {
            for x in 0..ow {
                // Box average of the source pixels this one covers.
                let (x0, x1) = (x * w / ow, ((x + 1) * w / ow).max(x * w / ow + 1));
                let (y0, y1) = (y * h / oh, ((y + 1) * h / oh).max(y * h / oh + 1));
                let mut sum = [0u32; 3];
                for yy in y0..y1 {
                    for xx in x0..x1 {
                        for k in 0..3 {
                            sum[k] += rgb[(yy * w + xx) * 3 + k] as u32;
                        }
                    }
                }
                let n = ((x1 - x0) * (y1 - y0)) as u32;
                for k in 0..3 {
                    out[(y * ow + x) * 3 + k] = (sum[k] / n) as u8;
                }
            }
        }
        (ow, oh, out)
    } else {
        (w, h, rgb)
    };
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 85).encode(&data, out_w as u16, out_h as u16, jpeg_encoder::ColorType::Rgb)?;
    Ok(jpeg)
}

fn r1(v: f32) -> f64 {
    (v as f64 * 10.0).round() / 10.0
}

fn r2(v: f32) -> f64 {
    (v as f64 * 100.0).round() / 100.0
}

pub fn survey_json(serial: u64, s: &Survey) -> Value {
    let ms = |d: Duration| (d.as_secs_f64() * 1e3).round();
    json!({
        "survey": serial,
        "candidates": s.candidates_json(),
        "players": s.players.iter().map(|p| json!({
            "name": p.name,
            "whitelist_rank": p.whitelist_rank,
            "ocr": p.text,
        })).collect::<Vec<_>>(),
        "room": s.room,
        "metres_per_unit": s.metres,
        "timings_ms": {
            "scan": ms(s.timings.scan), "stereo": ms(s.timings.stereo),
            "ocr": ms(s.timings.ocr), "map": ms(s.timings.map),
        },
    })
}

/// The latest survey's panorama as JPEG, cropped to the rows and columns
/// some frame saw (a look ahead: that one view).
pub fn pano_jpeg(s: &Survey) -> Result<Vec<u8>> {
    let p = s.marked_panorama(2048);
    // What the frames saw, from the panorama before the marks (a place's
    // number may lie outside them), turned like the marked one.
    let plain = s.panorama(p.width);
    let shift = ((s.yaw / 360.0) * p.width as f32).round() as i64;
    let px = |r: usize, c: usize| {
        let c = (c as i64 + shift).rem_euclid(p.width as i64) as usize;
        &plain.rgb[(r * p.width + c) * 3..(r * p.width + c + 1) * 3]
    };
    let row_seen = |r: usize| (0..p.width).any(|c| px(r, c).iter().any(|&b| b != 0));
    let col_seen = |c: usize| (0..p.height).any(|r| px(r, c).iter().any(|&b| b != 0));
    let first = (0..p.height).find(|&r| row_seen(r)).unwrap_or(0);
    let last = (0..p.height).rev().find(|&r| row_seen(r)).unwrap_or(p.height - 1);
    let left = (0..p.width).find(|&c| col_seen(c)).unwrap_or(0);
    let right = (0..p.width).rev().find(|&c| col_seen(c)).unwrap_or(p.width - 1);
    let mut rgb = Vec::with_capacity((last + 1 - first) * (right + 1 - left) * 3);
    for r in first..=last {
        rgb.extend_from_slice(&p.rgb[(r * p.width + left) * 3..(r * p.width + right + 1) * 3]);
    }
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 85).encode(&rgb, (right + 1 - left) as u16, (last + 1 - first) as u16, jpeg_encoder::ColorType::Rgb)?;
    Ok(jpeg)
}

/// The latest survey's map as PNG.
pub fn map_png(s: &Survey) -> Result<Vec<u8>> {
    let (side, rgb) = s.marked_map(4);
    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, side as u32, side as u32);
        enc.set_color(png::ColorType::Rgb);
        enc.set_depth(png::BitDepth::Eight);
        enc.write_header()?.write_image_data(&rgb)?;
    }
    Ok(out)
}
