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
        }
    }

    /// The rig, connected on first use (and again after a failure: `reset`).
    pub fn rig(&mut self, whitelist: &[String]) -> Result<&mut Rig> {
        if self.rig.is_none() {
            let mut rig = Rig::connect(&self.remote, &self.tap, Some(self.ocr_url.as_str()), &self.ocr_model, Vec::new())?;
            let (yaw, pitch) = rig.hmd.state.head.yaw_pitch();
            self.yaw = yaw;
            self.pitch = pitch;
            let head = rig.hmd.state.head.position;
            rig.hmd.state.hands_at_rest(head, yaw);
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

    /// Drops the rig: the next use connects again.
    pub fn reset(&mut self) {
        self.rig = None;
    }

    /// Points the head (degrees; pitch clamped to +-80).
    pub fn aim(&mut self, yaw: f32, pitch: f32) -> Result<()> {
        let rig = self.rig(&[])?;
        let head = rig.hmd.state.head.position;
        let pitch = pitch.clamp(-80.0, 80.0);
        rig.hmd.set_head(Pose::looking(yaw, pitch, head))?;
        self.yaw = (yaw + 540.0).rem_euclid(360.0) - 180.0;
        self.pitch = pitch;
        Ok(())
    }

    /// Turns the whole bot: the head (degrees; pitch clamped to +-80) and
    /// the body under it (the hands at its sides).
    pub fn face(&mut self, yaw: f32, pitch: f32) -> Result<()> {
        let rig = self.rig(&[])?;
        let head = rig.hmd.state.head.position;
        rig.hmd.state.hands_at_rest(head, yaw);
        self.aim(yaw, pitch)
    }

    // -- surveys and walks --------------------------------------------------------

    pub fn survey(&mut self, whitelist: &[String], players: bool) -> Result<Value> {
        let opts = SurveyOptions { players, ..Default::default() };
        let s = vrc_nav::survey(self.rig(whitelist)?, &opts, &[])?;
        self.serial += 1;
        self.yaw = s.yaw;
        let v = survey_json(self.serial, &s);
        self.survey = Some(s);
        Ok(v)
    }

    pub fn goto(&mut self, whitelist: &[String], input: &Value) -> Result<Value> {
        // A fresh survey to plan from (people move, and so may the bot).
        let s = vrc_nav::survey(self.rig(whitelist)?, &SurveyOptions { players: false, ..Default::default() }, &[])?;
        let target = if let Some(id) = input["candidate"].as_u64() {
            let seen = self.survey.as_ref().context("no survey to pick a candidate from: look around first")?;
            let c = seen.candidates.iter().find(|c| c.id as u64 == id).context("no such place in the last look around")?;
            [c.position[0], c.position[2]]
        } else {
            let bearing = input["bearing"].as_f64().context("a candidate, or a bearing and a distance")? as f32;
            let distance = input["distance"].as_f64().unwrap_or(2.0) as f32;
            let yaw = (s.yaw + bearing).to_radians();
            let d = distance / s.metres;
            [s.eye[0] + yaw.sin() * d, s.eye[2] - yaw.cos() * d]
        };
        let report = vrc_nav::goto(self.rig(whitelist)?, s, target, &GotoOptions::default())?;
        let after = vrc_nav::survey(self.rig(whitelist)?, &SurveyOptions::default(), &[])?;
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

    /// /v1/step: turns by `turn` degrees, then walks `meters` (world) a way
    /// (forward, back, left, right of where it then faces; the head turns
    /// to the way walked), jumping as it starts if asked.
    pub fn step(&mut self, osc_jump: impl Fn(), turn: f32, direction: &str, meters: f32, jump: bool) -> Result<Value> {
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
        let way = facing + offset;
        let rig = self.rig(&[])?;
        let osc = rig.osc.as_ref().context("walking needs VRChat's OSC")?;
        let osc = Osc::with_ports_from(osc)?;
        let leg = walk::leg(&mut rig.hmd, &osc, way, meters.min(STEP_MAX_M), &WalkParams::default())?;
        self.yaw = (way + 540.0).rem_euclid(360.0) - 180.0;
        self.pitch = 0.0;
        let (s, c) = offset.to_radians().sin_cos();
        Ok(json!({
            "ok": true, "turned": turn, "blocked": leg.blocked,
            "moved": {"ahead_m": r1(leg.walked * c), "right_m": r1(leg.walked * s)},
        }))
    }

    /// The first frame with the head tilted to `pitch` (degrees, + up).
    pub fn frame_looking(&mut self, pitch: f32) -> Result<EyeFrame> {
        let yaw = self.yaw;
        self.aim(yaw, pitch)?;
        let pitch = self.pitch;
        let rig = self.rig(&[])?;
        scan::rendered_at(&mut rig.tap, yaw, pitch, Duration::from_secs(1))
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

/// The latest survey's panorama as JPEG, cropped to the rows some frame saw.
pub fn pano_jpeg(s: &Survey) -> Result<Vec<u8>> {
    let p = s.marked_panorama(2048);
    let seen = |r: usize| p.rgb[r * p.width * 3..(r + 1) * p.width * 3].iter().any(|&b| b != 0);
    let first = (0..p.height).find(|&r| seen(r)).unwrap_or(0);
    let last = (0..p.height).rev().find(|&r| seen(r)).unwrap_or(p.height - 1);
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 85).encode(
        &p.rgb[first * p.width * 3..(last + 1) * p.width * 3],
        p.width as u16,
        (last + 1 - first) as u16,
        jpeg_encoder::ColorType::Rgb,
    )?;
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
