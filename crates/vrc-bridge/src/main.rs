//! vrc-bridge: the bot's VR capabilities over HTTP, for the AstrBot plugin
//! (which turns them into the model's tools; the Codex agent in AstrBot
//! decides what to do with them).
//!
//!     GET  /v1/health
//!     POST /v1/vr/survey   {"players": true}
//!          -> {"survey": n, "candidates": [...], "players": [...], "metres_per_unit", "timings_ms"}
//!     GET  /v1/vr/survey/pano.jpg   the latest survey's panorama, candidates numbered
//!     GET  /v1/vr/survey/map.png    its top-down map, the same numbers
//!     POST /v1/vr/goto     {"candidate": 3} or {"bearing": 40, "distance": 2.5}
//!          -> {"arrived", "remaining_m", "legs": [...], "took_s", "reason", and the new survey}
//!
//! Every request needs `Authorization: Bearer <token>` (the token file the
//! desktop bridge uses, `~/.config/vrc-bridge/token`). One request runs at a
//! time: they all move the same head.

use std::path::PathBuf;
use std::sync::Mutex;

use anyhow::{bail, Context, Result};
use clap::Parser;
use serde_json::{json, Value};
use tiny_http::{Header, Method, Request, Response, Server};
use vrc_nav::{GotoOptions, Rig, Survey, SurveyOptions};

#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "127.0.0.1")]
    listen: String,
    #[arg(long, default_value_t = 6121)]
    port: u16,
    /// The bearer token clients must send (a file holding it).
    #[arg(long, default_value = "~/.config/vrc-bridge/token")]
    token_file: String,
    /// Monado's remote driver.
    #[arg(long, default_value = "127.0.0.1:4242")]
    remote: String,
    /// The null compositor's eye tap.
    #[arg(long, default_value = "/dev/shm/vrc-eyes")]
    tap: String,
    /// local-multimodal-infra's OCR endpoint ("" for none).
    #[arg(long, default_value = "http://127.0.0.1:17890/v1/ocr/lines")]
    ocr_url: String,
    #[arg(long, default_value = "ppocrv5-mobile-onnx")]
    ocr_model: String,
    /// Whitelisted display names, in priority order (comma-separated).
    #[arg(long, value_delimiter = ',')]
    whitelist: Vec<String>,
}

struct State {
    args: Args,
    token: String,
    rig: Option<Rig>,
    survey: Option<Survey>,
    serial: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let token_path = expand(&args.token_file);
    let token = std::fs::read_to_string(&token_path)
        .with_context(|| format!("no token at {}", token_path.display()))?
        .trim()
        .to_string();
    if token.is_empty() {
        bail!("the token file {} is empty", token_path.display());
    }
    let addr = format!("{}:{}", args.listen, args.port);
    let server = Server::http(&addr).map_err(|e| anyhow::anyhow!("listen on {addr}: {e}"))?;
    eprintln!("vrc-bridge listening on {addr}");
    let state = Mutex::new(State { args, token, rig: None, survey: None, serial: 0 });
    for request in server.incoming_requests() {
        let mut st = state.lock().unwrap();
        handle(&mut st, request);
    }
    Ok(())
}

fn handle(st: &mut State, mut req: Request) {
    let authorized = req
        .headers()
        .iter()
        .any(|h| h.field.equiv("Authorization") && h.value.as_str() == format!("Bearer {}", st.token));
    let url = req.url().split('?').next().unwrap_or("").to_string();
    let method = req.method().clone();
    if url == "/v1/health" {
        respond(req, 200, json!({"ok": true, "survey": st.serial}));
        return;
    }
    if !authorized {
        respond(req, 401, json!({"error": "unauthorized"}));
        return;
    }
    let mut body = String::new();
    let _ = req.as_reader().read_to_string(&mut body);
    let input: Value = if body.trim().is_empty() { json!({}) } else { serde_json::from_str(&body).unwrap_or(json!({})) };
    let result = match (method, url.as_str()) {
        (Method::Post, "/v1/vr/survey") => survey(st, &input).map(Reply::Json),
        (Method::Post, "/v1/vr/goto") => goto(st, &input).map(Reply::Json),
        (Method::Get, "/v1/vr/survey/pano.jpg") => pano(st),
        (Method::Get, "/v1/vr/survey/map.png") => map(st),
        _ => Ok(Reply::Status(404, json!({"error": "no such endpoint"}))),
    };
    match result {
        Ok(Reply::Json(v)) => respond(req, 200, v),
        Ok(Reply::Status(code, v)) => respond(req, code, v),
        Ok(Reply::Bytes(mime, bytes)) => {
            let header = Header::from_bytes("Content-Type", mime).unwrap();
            let _ = req.respond(Response::from_data(bytes).with_header(header));
        }
        Err(e) => {
            // A failed capability may leave the rig in a bad state: reconnect next time.
            st.rig = None;
            respond(req, 500, json!({"error": format!("{e:#}")}));
        }
    }
}

enum Reply {
    Json(Value),
    Status(u16, Value),
    Bytes(&'static str, Vec<u8>),
}

fn respond(req: Request, code: u16, v: Value) {
    let header = Header::from_bytes("Content-Type", "application/json").unwrap();
    let _ = req.respond(Response::from_string(v.to_string()).with_status_code(code).with_header(header));
}

fn rig(st: &mut State) -> Result<&mut Rig> {
    if st.rig.is_none() {
        let a = &st.args;
        st.rig = Some(Rig::connect(&a.remote, &a.tap, Some(a.ocr_url.as_str()), &a.ocr_model, a.whitelist.clone())?);
    }
    Ok(st.rig.as_mut().unwrap())
}

fn survey_json(serial: u64, s: &Survey) -> Value {
    let ms = |d: std::time::Duration| (d.as_secs_f64() * 1e3).round();
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

fn survey(st: &mut State, input: &Value) -> Result<Value> {
    let players = input["players"].as_bool().unwrap_or(true);
    let opts = SurveyOptions { players, ..Default::default() };
    let s = vrc_nav::survey(rig(st)?, &opts, &[])?;
    st.serial += 1;
    let v = survey_json(st.serial, &s);
    st.survey = Some(s);
    Ok(v)
}

fn goto(st: &mut State, input: &Value) -> Result<Value> {
    // A fresh survey to plan from (the latest may be stale: people move,
    // and so may the bot by other means).
    let s = vrc_nav::survey(rig(st)?, &SurveyOptions::default(), &[])?;
    let target = if let Some(id) = input["candidate"].as_u64() {
        // The candidate as numbered in the survey the model saw: matched by
        // what it is and where it was, in this fresh survey's terms.
        let seen = st.survey.as_ref().context("no survey to pick a candidate from")?;
        let c = seen.candidates.iter().find(|c| c.id as u64 == id).context("no such candidate")?;
        [c.position[0], c.position[2]]
    } else {
        let bearing = input["bearing"].as_f64().context("a candidate, or a bearing and a distance")? as f32;
        let distance = input["distance"].as_f64().unwrap_or(2.0) as f32;
        let yaw = (s.yaw + bearing).to_radians();
        let d = distance / s.metres;
        [s.eye[0] + yaw.sin() * d, s.eye[2] - yaw.cos() * d]
    };
    let report = vrc_nav::goto(rig(st)?, s, target, &GotoOptions::default())?;
    let after = vrc_nav::survey(rig(st)?, &SurveyOptions::default(), &[])?;
    st.serial += 1;
    let mut v = json!({
        "arrived": report.arrived,
        "remaining_m": (report.remaining * 100.0).round() / 100.0,
        "took_s": (report.took.as_secs_f64() * 10.0).round() / 10.0,
        "reason": report.reason,
        "legs": report.legs.iter().map(|l| json!({
            "heading_deg": l.yaw.round(),
            "planned_m": (l.planned * 100.0).round() / 100.0,
            "walked_m": (l.walked * 100.0).round() / 100.0,
            "blocked": l.blocked,
        })).collect::<Vec<_>>(),
    });
    v["after"] = survey_json(st.serial, &after);
    st.survey = Some(after);
    Ok(v)
}

fn pano(st: &mut State) -> Result<Reply> {
    let Some(s) = &st.survey else { return Ok(Reply::Status(404, json!({"error": "no survey yet"}))) };
    let p = s.marked_panorama(2048);
    // Cropped to the rows some frame saw (above and below the ring: nothing).
    let seen = |r: usize| p.rgb[r * p.width * 3..(r + 1) * p.width * 3].iter().any(|&b| b != 0);
    let first = (0..p.height).find(|&r| seen(r)).unwrap_or(0);
    let last = (0..p.height).rev().find(|&r| seen(r)).unwrap_or(p.height - 1);
    let rows = last + 1 - first;
    let mut jpeg = Vec::new();
    jpeg_encoder::Encoder::new(&mut jpeg, 85).encode(
        &p.rgb[first * p.width * 3..(last + 1) * p.width * 3],
        p.width as u16,
        rows as u16,
        jpeg_encoder::ColorType::Rgb,
    )?;
    Ok(Reply::Bytes("image/jpeg", jpeg))
}

fn map(st: &mut State) -> Result<Reply> {
    let Some(s) = &st.survey else { return Ok(Reply::Status(404, json!({"error": "no survey yet"}))) };
    let (side, rgb) = s.marked_map(4);
    let mut out = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut out, side as u32, side as u32);
        enc.set_color(png::ColorType::Rgb);
        enc.set_depth(png::BitDepth::Eight);
        enc.write_header()?.write_image_data(&rgb)?;
    }
    Ok(Reply::Bytes("image/png", out))
}

fn expand(path: &str) -> PathBuf {
    match path.strip_prefix("~/") {
        Some(rest) => PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(rest),
        None => PathBuf::from(path),
    }
}
