//! vrc-bridge: the bot's VRChat client over HTTP/WebSocket, for the AstrBot
//! plugin (which turns it into a platform and the model's tools; the Codex
//! agent in AstrBot decides what to do). Runs as the bot's Linux user next
//! to VRChat (Proton, VR mode on the virtual headset: `docs/full-vr/`).
//!
//! * `/v1/stream` (WebSocket, one client at a time): binary frames carry
//!   audio, 16-bit mono PCM at 48 kHz, both ways: in, what the game plays
//!   (the capture sink's monitor: other players' voices); out, the bot's
//!   voice, played into the microphone sink while push-to-talk is held.
//!   Text frames carry the room state and events.
//! * HTTP (bearer token): status, chatbox, steps, jumps, emotes, stopping,
//!   following, the Web API side (whitelist, invites, following friends
//!   across instances), sightings, a frame of the eyes, starting and stopping
//!   the game, and looking and walking: `/v1/vr/survey` (look around:
//!   numbered places and players) and `/v1/vr/goto` (walk there).
//!
//! The desktop bridge's mouse-and-screenshot endpoints (move, turn, look,
//! drive, nav, goto, look_around, map, note, autopilot, camera_y) are gone:
//! the head's look around and the walks replace them.

mod api;
mod bridge;
mod follow;
mod game;
mod sightings;
mod social;
mod vr;

use std::sync::Arc;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use axum::body::Bytes;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Query, Request, State};
use axum::http::{HeaderMap, HeaderValue, StatusCode};
use axum::middleware::{self, Next};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::{Parser, Subcommand};
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};

use crate::bridge::Bridge;

const LOG_DIR: &str =
    "~/.local/share/Steam/steamapps/compatdata/438100/pfx/drive_c/users/steamuser/AppData/LocalLow/VRChat/VRChat";
/// VRCEmote values of VRChat's default emote menu.
const EMOTES: [(&str, i32); 8] =
    [("wave", 1), ("clap", 2), ("point", 3), ("cheer", 4), ("dance", 5), ("backflip", 6), ("sadness", 7), ("die", 8)];

#[derive(Parser, Clone, Debug)]
#[command(about = "The bot's VRChat client over HTTP/WebSocket for AstrBot")]
pub struct Args {
    #[command(subcommand)]
    command: Option<Cmd>,
    #[arg(long, default_value = "127.0.0.1", global = true)]
    pub listen: String,
    #[arg(long, default_value_t = 6120, global = true)]
    pub port: u16,
    #[arg(long, default_value = "~/.config/vrc-bridge/token", global = true)]
    pub token_file: String,
    #[arg(long, default_value = LOG_DIR)]
    pub log_dir: String,
    #[arg(long, default_value_t = 9000)]
    pub osc_port: u16,
    #[arg(long, default_value = ":1")]
    pub display: String,
    #[arg(long, default_value = "vrc_out.monitor")]
    pub capture_device: String,
    #[arg(long, default_value = "vrc_mic_in")]
    pub playback_device: String,
    #[arg(long, default_value_t = 4500)]
    pub vram_soft_mib: u64,
    #[arg(long, default_value_t = 6000)]
    pub vram_hard_mib: u64,
    #[arg(long, default_value_t = 11000)]
    pub gpu_hard_mib: u64,
    /// local-multimodal-infra's text lines endpoint (name tags).
    #[arg(long, default_value = "http://127.0.0.1:17890/v1/ocr/lines")]
    pub ocr_url: String,
    #[arg(long, default_value = "ppocrv5-mobile-onnx")]
    pub ocr_model: String,
    /// Monado's remote driver (the virtual headset).
    #[arg(long, default_value = "127.0.0.1:4242")]
    pub remote: String,
    /// The null compositor's eye tap.
    #[arg(long, default_value = "/dev/shm/vrc-eyes")]
    pub tap: String,
}

#[derive(Subcommand, Clone, Debug)]
enum Cmd {
    /// Serve (the default).
    Serve,
    /// Log the bot account in to the VRChat Web API (interactive; only the
    /// cookies are kept).
    Login,
}

type App = Arc<Bridge>;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()))
        .init();
    let args = Args::parse();
    let token_path = game::expand(&args.token_file);
    if matches!(args.command, Some(Cmd::Login)) {
        let cookies = token_path.parent().context("no config directory")?.join("cookies.json");
        return api::login_main(&cookies).await;
    }
    let token = std::fs::read_to_string(&token_path)
        .with_context(|| format!("no token at {}", token_path.display()))?
        .trim()
        .to_string();
    if token.len() < 16 {
        bail!("the token file must hold a token of 16+ characters");
    }
    let addr = format!("{}:{}", args.listen, args.port);
    let (bridge, chat_rx) = Bridge::new(args, token);
    tokio::spawn(bridge.clone().capture());
    tokio::spawn(bridge.clone().log_tail());
    tokio::spawn(bridge.clone().watchdog());
    tokio::spawn(bridge.clone().chatbox_sender(chat_rx));
    tokio::spawn(bridge.social.clone().run(bridge.clone()));
    tokio::spawn(bridge.sightings.clone().run(bridge.clone()));

    let app = Router::new()
        .route("/v1/stream", get(stream))
        .route("/v1/status", get(status))
        .route("/v1/chatbox", post(chatbox))
        .route("/v1/stop", post(stop))
        .route("/v1/jump", post(jump))
        .route("/v1/step", post(step))
        .route("/v1/emote", post(emote))
        .route("/v1/follow", get(follow_status).post(follow))
        .route("/v1/social", get(social_status))
        .route("/v1/social/config", post(social_config))
        .route("/v1/screenshot", get(screenshot))
        .route("/v1/sightings", get(sightings_list))
        .route("/v1/sightings/image", get(sighting_image))
        .route("/v1/game/start", post(game_start))
        .route("/v1/game/stop", post(game_stop))
        .route("/v1/vr/survey", post(vr_survey))
        .route("/v1/vr/goto", post(vr_goto))
        .route("/v1/vr/survey/pano.jpg", get(vr_pano))
        .route("/v1/vr/survey/map.png", get(vr_map))
        .route_layer(middleware::from_fn_with_state(bridge.clone(), auth))
        .with_state(bridge.clone());
    let listener = tokio::net::TcpListener::bind(&addr).await.with_context(|| format!("listen on {addr}"))?;
    tracing::info!("vrc-bridge listening on {addr}");
    let shutdown = bridge.clone();
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            let _ = tokio::signal::ctrl_c().await;
            shutdown.follower.stop();
            shutdown.release_voice().await;
        })
        .await?;
    Ok(())
}

// -- plumbing -------------------------------------------------------------------

async fn auth(State(b): State<App>, req: Request, next: Next) -> Response {
    let given = req
        .headers()
        .get("Authorization")
        .and_then(|v| v.to_str().ok())
        .map(|v| v.trim_start_matches("Bearer ").trim().to_string())
        .unwrap_or_default();
    // Compared in full, whatever differs first.
    let ok = given.len() == b.token.len() && given.bytes().zip(b.token.bytes()).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0;
    if !ok {
        return (StatusCode::UNAUTHORIZED, Json(json!({"error": "unauthorized"}))).into_response();
    }
    next.run(req).await
}

/// An error for the client: 409 (the action cannot be done now) with its
/// message, as the desktop bridge answered.
struct Fail(anyhow::Error);

impl<E: Into<anyhow::Error>> From<E> for Fail {
    fn from(e: E) -> Self {
        Fail(e.into())
    }
}

impl IntoResponse for Fail {
    fn into_response(self) -> Response {
        (StatusCode::CONFLICT, Json(json!({"error": format!("{:#}", self.0)}))).into_response()
    }
}

type Reply = std::result::Result<Json<Value>, Fail>;

/// A JSON body, whatever its Content-Type says (the desktop bridge took
/// any); empty is `{}`.
struct Body(Value);

impl<S: Send + Sync> axum::extract::FromRequest<S> for Body {
    type Rejection = Response;

    async fn from_request(req: Request, state: &S) -> std::result::Result<Self, Self::Rejection> {
        let bytes = Bytes::from_request(req, state).await.map_err(IntoResponse::into_response)?;
        if bytes.iter().all(u8::is_ascii_whitespace) {
            return Ok(Body(json!({})));
        }
        serde_json::from_slice(&bytes)
            .map(Body)
            .map_err(|e| (StatusCode::BAD_REQUEST, Json(json!({"error": format!("bad request: {e}")}))).into_response())
    }
}

fn num(v: &Value, key: &str, default: f64) -> f64 {
    v[key].as_f64().unwrap_or(default)
}

/// Runs `f` on the headset on a blocking thread.
async fn on_headset<T: Send + 'static>(b: &App, f: impl FnOnce(&mut vr::VrCore, &App) -> Result<T> + Send + 'static) -> Result<T> {
    let b = b.clone();
    tokio::task::spawn_blocking(move || {
        let mut core = b.vr.lock().unwrap();
        let r = f(&mut core, &b);
        if r.is_err() {
            core.reset(); // a failure may leave the connection in a bad state
        }
        r
    })
    .await?
}

// -- stream ---------------------------------------------------------------------

async fn stream(State(b): State<App>, ws: WebSocketUpgrade) -> Response {
    ws.max_message_size(1 << 20).on_upgrade(move |socket| client(b, socket))
}

async fn client(b: App, socket: WebSocket) {
    let (id, mut out) = b.attach_client();
    tracing::info!("stream client connected");
    let (mut tx, mut rx) = socket.split();
    let sender = tokio::spawn(async move {
        while let Some(msg) = out.recv().await {
            let close = matches!(msg, Message::Close(_));
            if tx.send(msg).await.is_err() || close {
                break;
            }
        }
    });
    while let Some(Ok(msg)) = rx.next().await {
        match msg {
            Message::Binary(pcm) => b.play(&pcm).await,
            Message::Close(_) => break,
            _ => {}
        }
    }
    sender.abort();
    b.detach_client(id);
    tracing::info!("stream client gone");
}

// -- status, chatbox, moves -------------------------------------------------------

async fn status(State(b): State<App>) -> Json<Value> {
    let mut s = b.room_state();
    s["follow"] = b.follower.status();
    Json(s)
}

async fn chatbox(State(b): State<App>, Body(body): Body) -> Reply {
    let text = body["text"].as_str().context("text is required")?;
    let parts = b.chatbox(text, body["notify"].as_bool().unwrap_or(false));
    Ok(Json(json!({"queued": parts})))
}

async fn stop(State(b): State<App>) -> Json<Value> {
    b.follower.stop();
    for axis in ["/input/Vertical", "/input/Horizontal"] {
        let _ = b.osc.send_f32(axis, 0.0);
    }
    b.notify_state();
    Json(json!({"ok": true}))
}

async fn jump(State(b): State<App>) -> Reply {
    b.require_game()?;
    let _ = b.osc.send_i32("/input/Jump", 1);
    tokio::time::sleep(Duration::from_millis(100)).await;
    let _ = b.osc.send_i32("/input/Jump", 0);
    Ok(Json(json!({"ok": true})))
}

async fn step(State(b): State<App>, Body(body): Body) -> Reply {
    b.require_game()?;
    let turn = num(&body, "turn", 0.0).clamp(-180.0, 180.0) as f32;
    let meters = num(&body, "meters", 0.0).clamp(0.0, vr::STEP_MAX_M as f64) as f32;
    let direction = body["direction"].as_str().unwrap_or("forward").to_string();
    let jump = body["jump"].as_bool().unwrap_or(false);
    b.follower.stop();
    let v = on_headset(&b, move |vr, b| {
        let osc = &b.osc;
        vr.step(
            || {
                let _ = osc.send_i32("/input/Jump", 1);
                std::thread::sleep(Duration::from_millis(100));
                let _ = osc.send_i32("/input/Jump", 0);
            },
            turn,
            &direction,
            meters,
            jump,
        )
    })
    .await?;
    Ok(Json(v))
}

async fn emote(State(b): State<App>, Body(body): Body) -> Reply {
    b.require_game()?;
    let name = body["name"].as_str().context("name is required")?;
    let value = EMOTES
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, v)| *v)
        .with_context(|| format!("emote must be one of {}", EMOTES.map(|e| e.0).join(", ")))?;
    let seconds = num(&body, "seconds", 2.0).clamp(0.5, 10.0);
    let osc = b.osc_query()?;
    // Answered at once: the caller (a voice model) should not wait for the
    // emote to play out before it speaks.
    tokio::task::spawn_blocking(move || {
        if osc.query("/avatar/parameters/VRCEmote").is_err() {
            tracing::warn!("emote failed: the current avatar has no emotes");
            return;
        }
        let _ = osc.send_i32("/avatar/parameters/VRCEmote", value);
        std::thread::sleep(Duration::from_secs_f64(seconds));
        let _ = osc.send_i32("/avatar/parameters/VRCEmote", 0);
    });
    Ok(Json(json!({"ok": true})))
}

// -- following ----------------------------------------------------------------------

async fn follow_status(State(b): State<App>) -> Json<Value> {
    Json(b.follower.status())
}

async fn follow(State(b): State<App>, Body(body): Body) -> Reply {
    if body["stop"].as_bool().unwrap_or(false) {
        b.follower.stop();
        b.notify_state();
        return Ok(Json(b.follower.status()));
    }
    if let Some(change) = body["adjust"].as_str() {
        if change == "stay" && b.follower.is_idle() {
            for axis in ["/input/Vertical", "/input/Horizontal"] {
                let _ = b.osc.send_f32(axis, 0.0);
            }
        } else {
            b.follower.adjust(change)?;
        }
        b.notify_state();
        return Ok(Json(b.follower.status()));
    }
    b.require_game()?;
    let mut name = body["name"].as_str().unwrap_or("").trim().to_string();
    if name.is_empty() {
        // The whitelisted player in the room with the highest priority.
        let here = b.game.lock().unwrap().others();
        name = b
            .social
            .whitelist_ids()
            .iter()
            .find_map(|id| here.iter().find(|(pid, _)| pid == id).map(|(_, n)| n.clone()))
            .unwrap_or_default();
    }
    if name.is_empty() {
        bail_fail("nobody to follow: name someone, or a whitelisted player must be here")?;
    }
    let distance = body["distance"].as_f64().map(|d| d as f32);
    b.follower.start(&b, &name, distance);
    Ok(Json(b.follower.status()))
}

fn bail_fail(msg: &str) -> std::result::Result<(), Fail> {
    Err(Fail(anyhow::anyhow!(msg.to_string())))
}

// -- social, sightings, screenshots ----------------------------------------------------

async fn social_status(State(b): State<App>) -> Json<Value> {
    Json(b.social.status())
}

async fn social_config(State(b): State<App>, Body(body): Body) -> Reply {
    b.social.set_config(&b, &body)?;
    Ok(Json(b.social.status()))
}

async fn screenshot(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> std::result::Result<Response, Fail> {
    b.require_game()?;
    let width: u32 = q.get("width").and_then(|w| w.parse().ok()).unwrap_or(0).min(3840);
    let jpeg = on_headset(&b, move |vr, _| {
        let frame = vr.frame()?;
        vr::eye_jpeg(&frame, width)
    })
    .await?;
    Ok(([("Content-Type", "image/jpeg")], jpeg).into_response())
}

async fn sightings_list(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> Json<Value> {
    let listing = b.sightings.listing();
    let mut body = json!({"sightings": listing});
    if let Some(name) = q.get("name") {
        let found = b.sightings.latest(name);
        body["match"] = found
            .and_then(|(n, _)| body["sightings"].as_array().and_then(|a| a.iter().find(|s| s["name"] == n.as_str()).cloned()))
            .unwrap_or(Value::Null);
    }
    Json(body)
}

async fn sighting_image(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> std::result::Result<Response, Fail> {
    let name = q.get("name").cloned().unwrap_or_default();
    let Some((who, s)) = b.sightings.latest(&name) else {
        return Err(Fail(anyhow::anyhow!("no whitelisted friend has been seen yet")));
    };
    let mut headers = HeaderMap::new();
    headers.insert("Content-Type", HeaderValue::from_static("image/jpeg"));
    headers.insert("X-Sighting-Name", HeaderValue::from_str(&api::percent_encode(&who))?);
    headers.insert("X-Sighting-At", HeaderValue::from_str(&s.at.to_string())?);
    headers.insert("X-Sighting-World", HeaderValue::from_str(&api::percent_encode(&s.world))?);
    Ok((headers, s.jpeg).into_response())
}

// -- the game ---------------------------------------------------------------------------

async fn game_start(State(b): State<App>, Body(body): Body) -> Reply {
    let url = body["url"].as_str().unwrap_or("").trim().to_string();
    let location = if url.is_empty() { String::new() } else { api::launch_location(&url)? };
    if body["whitelisted_only"].as_bool().unwrap_or(false) {
        let there = b.social.whitelist_ids().iter().any(|id| b.social.friend_location(id).as_deref() == Some(location.as_str()));
        if location.is_empty() || !there {
            bail_fail("only the instance a whitelisted friend is in now")?;
        }
    }
    // Rebuilt from the checked instance: nothing else reaches the game.
    let url = if location.is_empty() { String::new() } else { api::launch_url(&location)? };
    let _guard = b.game_lock.lock().await; // not across a join's restart
    if body["restart"].as_bool().unwrap_or(false) {
        game::stop_game().await?;
    }
    game::start_game(&b.args.display, &url).await?;
    Ok(Json(json!({"ok": true})))
}

async fn game_stop(State(_b): State<App>) -> Reply {
    game::stop_game().await?;
    Ok(Json(json!({"ok": true, "running": game::game_pid().await.is_some()})))
}

// -- VR ---------------------------------------------------------------------------------

async fn vr_survey(State(b): State<App>, Body(body): Body) -> Reply {
    let players = body["players"].as_bool().unwrap_or(true);
    let whitelist = b.social.whitelist_names();
    Ok(Json(on_headset(&b, move |vr, _| vr.survey(&whitelist, players)).await?))
}

async fn vr_goto(State(b): State<App>, Body(body): Body) -> Reply {
    b.require_game()?;
    b.follower.stop();
    let whitelist = b.social.whitelist_names();
    Ok(Json(on_headset(&b, move |vr, _| vr.goto(&whitelist, &body)).await?))
}

async fn vr_pano(State(b): State<App>) -> std::result::Result<Response, Fail> {
    let jpeg = on_headset(&b, |vr, _| vr::pano_jpeg(vr.survey.as_ref().context("no survey yet")?)).await?;
    Ok(([("Content-Type", "image/jpeg")], jpeg).into_response())
}

async fn vr_map(State(b): State<App>) -> std::result::Result<Response, Fail> {
    let png = on_headset(&b, |vr, _| vr::map_png(vr.survey.as_ref().context("no survey yet")?)).await?;
    Ok(([("Content-Type", "image/png")], png).into_response())
}
