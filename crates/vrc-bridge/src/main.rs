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
//!   The capture is stereo: the mix sent is mono, and who is speaking
//!   (`speaker`: direction and nameplates) goes out as `speaker` events in
//!   the same queue, on the client's sample clock.
//! * HTTP (bearer token): status, chatbox, steps, jumps, emotes, stopping,
//!   following, the Web API side (whitelist, invites, following friends
//!   across instances), sightings, a frame of the eyes, starting and stopping
//!   the game, and looking and walking: `/v1/vr/survey` (look around:
//!   numbered places and players) and `/v1/vr/goto` (walk there); who spoke
//!   (`/v1/speakers`) and turning to them (`/v1/vr/attend`).
//!
//! The desktop bridge's mouse-and-screenshot endpoints (move, turn, look,
//! drive, nav, goto, look_around, map, note, autopilot, camera_y) are gone:
//! the head's look around and the walks replace them.

mod anim;
mod api;
mod bridge;
mod calibrate;
mod follow;
mod mapping;
mod motion;
mod game;
mod orbit;
mod panolook;
mod people;
mod pano;
mod sightings;
mod social;
mod speaker;
mod usercam;
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

/// A std mutex taken whatever a panicking holder left: the state behind
/// these locks stays usable, and one panic must not turn every later
/// request into an error (or kill the animation and the follower).
pub trait Lock<T> {
    fn lk(&self) -> std::sync::MutexGuard<'_, T>;
    /// Taken if free now.
    fn try_lk(&self) -> Option<std::sync::MutexGuard<'_, T>>;
}

impl<T> Lock<T> for std::sync::Mutex<T> {
    fn lk(&self) -> std::sync::MutexGuard<'_, T> {
        self.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    fn try_lk(&self) -> Option<std::sync::MutexGuard<'_, T>> {
        match self.try_lock() {
            Ok(g) => Some(g),
            Err(std::sync::TryLockError::Poisoned(e)) => Some(e.into_inner()),
            Err(std::sync::TryLockError::WouldBlock) => None,
        }
    }
}

/// The view ahead out of the panorama (degrees across): the eyes' own.
const FORWARD_FOV_DEG: f32 = 100.0;
/// Menu work over HTTP (hands, buttons, a screenshot of the usual view)
/// holds the usual view this long after each call.
const MENU_HOLD: Duration = Duration::from_secs(20);

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
    /// Monado's `monado-ctl`, for recentering on `/v1/vr/reset` (none: skip).
    #[arg(long, default_value = "")]
    pub monado_ctl: String,
    /// Who is speaking: the voice's direction and the nameplates' glow
    /// (`docs/full-vr/speaker.md`). Off: the mono mix only.
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    pub speakers: bool,
    /// The HRTF the direction is matched against: `auto` (Steam Audio's
    /// default, `assets/hrtf/steam-default-48k.bin` from the checkout the
    /// binary is in, or from here; else the sphere), `sphere` (a spherical
    /// head), or a table's path (`tools/hrtf-render`).
    #[arg(long, default_value = "auto")]
    pub hrtf_table: String,
    /// The capture's ears are the other way round (the calibration says).
    #[arg(long, default_value_t = false, action = clap::ArgAction::Set)]
    pub swap_ears: bool,
    /// From the head pose the game renders audio for to the capture (ms).
    #[arg(long, default_value_t = 60)]
    pub audio_latency_ms: u64,
    /// The bot's voice may come back through another player's open
    /// speakers this long after it played out (ms; the playout already
    /// counts the bot's own latency): those hops and looks count for nothing.
    #[arg(long, default_value_t = 400)]
    pub echo_tail_ms: u64,
    /// The nameplate's ring (speaker.md 2.3): lit at once above this score
    /// over the plate's quiet look, unlit below `glow_off`; the ring stays
    /// lit `glow_release_ms` after the speech (measured 2026-10-08: unlit
    /// about 0.15, lit about 1; 0.9 s).
    #[arg(long, default_value_t = 0.30)]
    pub glow_on: f32,
    #[arg(long, default_value_t = 0.15)]
    pub glow_off: f32,
    #[arg(long, default_value_t = 900)]
    pub glow_release_ms: u64,
    /// A speaker only the voice's direction speaks for (their nameplate not
    /// seen lit, by the eyes or the lens) is at most this likely; the rest
    /// is "someone not recognised" (speaker.md 3).
    #[arg(long, default_value_t = 0.55)]
    pub doa_only_cap: f32,
    /// The game's desktop window on `--display` (`WxH+X,Y`): in stream
    /// mode it shows the user camera's view (`/v1/vr/usercam`).
    #[arg(long, default_value = "1280x720+0,0")]
    pub desktop_grab: String,
    /// The avatar's panorama rig (OSC `Pano`, docs/full-vr/avatar-panorama.md,
    /// decision D36): `auto` (the default: on while in a world, unless the
    /// avatar shows no rig within a few seconds: asked again every minute
    /// and on each world joined), `on` (on from the start, kept on), `off`
    /// (the bridge turns it on only when asked: `POST /v1/vr/pano`).
    #[arg(long, value_enum, default_value = "auto")]
    pub pano: pano::Setting,
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
    // A bridge before this one may have died walking or talking.
    bridge.let_go();
    tokio::spawn(bridge.clone().capture());
    tokio::spawn(bridge.clone().log_tail());
    tokio::spawn(bridge.clone().watchdog());
    tokio::spawn(bridge.clone().chatbox_sender(chat_rx));
    tokio::spawn(bridge.social.clone().run(bridge.clone()));
    tokio::spawn(bridge.sightings.clone().run(bridge.clone()));
    bridge.mapping.start(bridge.clone());
    if bridge.speaker.enabled {
        let (speakers, b) = (bridge.speaker.clone(), bridge.clone());
        std::thread::spawn(move || speakers.watch(b));
    }
    {
        let (anim, b) = (bridge.anim.clone(), bridge.clone());
        std::thread::spawn(move || anim.run(b));
    }
    {
        let (pano, b) = (bridge.pano.clone(), bridge.clone());
        std::thread::Builder::new().name("pano".into()).spawn(move || {
            pano.run(move || {
                let g = b.game.lk();
                g.running && !g.instance.is_empty()
            })
        })?;
    }
    {
        // Every movement input goes through the user camera's lens first
        // (its flying would take the walk): the one choke point.
        let orbit = bridge.orbit.clone();
        vrc_vr::osc::set_move_gate(Some(Arc::new(move |address: &str, value: f32| orbit.gate(address, value))));
        let (orbit, b) = (bridge.orbit.clone(), bridge.clone());
        std::thread::Builder::new().name("usercam-orbit".into()).spawn(move || orbit.run(b))?;
        let (orbit, b) = (bridge.orbit.clone(), bridge.clone());
        std::thread::Builder::new().name("usercam-names".into()).spawn(move || orbit.sightings(b))?;
        let (people, b) = (bridge.people.clone(), bridge.clone());
        std::thread::Builder::new().name("idle-sweep".into()).spawn(move || people.run(b))?;
    }

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
        .route("/v1/anim", get(anim_params).post(anim_tune))
        .route("/v1/sightings", get(sightings_list))
        .route("/v1/sightings/image", get(sighting_image))
        .route("/v1/game/start", post(game_start))
        .route("/v1/game/stop", post(game_stop))
        .route("/v1/vr/survey", post(vr_survey))
        .route("/v1/vr/goto", post(vr_goto))
        .route("/v1/vr/height", get(vr_height).post(vr_set_height))
        .route("/v1/vr/reset", post(vr_reset))
        .route("/v1/vr/corridor", get(vr_corridor))
        .route("/v1/vr/trackers", get(vr_trackers).post(vr_set_trackers))
        .route("/v1/vr/hand", post(vr_hand))
        .route("/v1/vr/input", post(vr_input))
        .route("/v1/vr/head", post(vr_head))
        .route("/v1/vr/calibrate", get(vr_calibrate_status).post(vr_calibrate))
        .route("/v1/vr/attend", post(vr_attend))
        .route("/v1/vr/usercam", get(vr_usercam_status).post(vr_usercam))
        .route("/v1/vr/usercam/shot", post(vr_usercam_shot))
        .route("/v1/vr/usercam/sweep", post(vr_usercam_sweep))
        .route("/v1/vr/usercam/names", get(vr_usercam_names))
        .route("/v1/vr/usercam/look", post(vr_usercam_look))
        .route("/v1/vr/people", get(vr_people))
        .route("/v1/speakers", get(speakers_status))
        .route("/v1/speakers/record", post(speakers_record))
        .route("/v1/motion", get(motion_status).post(motion_play))
        .route("/v1/motion/stop", post(motion_stop))
        .route("/v1/motion/reload", post(motion_reload))
        .route("/v1/vr/survey/pano.jpg", get(vr_pano))
        .route("/v1/vr/survey/map.png", get(vr_map))
        .route("/v1/vr/beacon", get(vr_beacon))
        .route("/v1/vr/pano", get(vr_pano_status).post(vr_pano_set))
        .route("/v1/vr/pano.jpg", get(vr_pano_jpg))
        .route("/v1/vr/pano/points", get(vr_pano_points))
        .route("/v1/vr/detect", post(vr_detect))
        .route("/v1/map", get(map_status))
        .route("/v1/map.png", get(map_png))
        .route("/v1/map/save", post(map_save))
        .route("/v1/map/forget", post(map_forget))
        .route("/v1/map/place", post(map_place))
        .route_layer(middleware::from_fn_with_state(bridge.clone(), auth))
        .with_state(bridge.clone());
    let listener = tokio::net::TcpListener::bind(&addr).await.with_context(|| format!("listen on {addr}"))?;
    tracing::info!("vrc-bridge listening on {addr}");
    let shutdown = bridge.clone();
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            stopping().await;
            shutdown.follower.stop();
            vrc_vr::walk::stop_all();
            shutdown.release_voice().await;
            // A tick for the legs to see the stop, then nothing pushed.
            tokio::time::sleep(Duration::from_millis(100)).await;
            shutdown.let_go();
        })
        .await?;
    Ok(())
}

// -- plumbing -------------------------------------------------------------------

/// Ctrl-C, or systemd stopping the unit (SIGTERM).
async fn stopping() {
    #[cfg(unix)]
    {
        let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()).expect("a SIGTERM handler");
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {}
            _ = term.recv() => {}
        }
    }
    #[cfg(not(unix))]
    let _ = tokio::signal::ctrl_c().await;
}

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
        let mut core = b.vr.lk();
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
    b.end_takeover(); // stopping means staying stopped
    b.follower.stop();
    vrc_vr::walk::stop_all();
    for axis in ["/input/Vertical", "/input/Horizontal"] {
        let _ = b.osc.send_f32(axis, 0.0);
    }
    b.notify_state();
    Json(json!({"ok": true}))
}

async fn jump(State(b): State<App>) -> Reply {
    b.require_game()?;
    // The move gate may hold the push a moment (the user camera's lens).
    tokio::task::spawn_blocking(move || {
        let _ = b.osc.send_i32("/input/Jump", 1);
        std::thread::sleep(Duration::from_millis(100));
        let _ = b.osc.send_i32("/input/Jump", 0);
    })
    .await?;
    Ok(Json(json!({"ok": true})))
}

async fn step(State(b): State<App>, Body(body): Body) -> Reply {
    let_go_of_motion(&b).await;
    b.require_game()?;
    let turn = num(&body, "turn", 0.0).clamp(-180.0, 180.0) as f32;
    let meters = num(&body, "meters", 0.0).clamp(0.0, vr::STEP_MAX_M as f64) as f32;
    let direction = body["direction"].as_str().unwrap_or("forward").to_string();
    let jump = body["jump"].as_bool().unwrap_or(false);
    let axis = vr::pace_axis(body["pace"].as_str())?;
    b.take_over();
    let since = vrc_vr::walk::stops();
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
            since,
            axis,
        )
    })
    .await;
    b.idle_later();
    Ok(Json(v?))
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
    // The follow's settings (`{"settings": {"kept_s": 7}}`): changed, the
    // follow (if any) goes on.
    if let Some(change) = body.get("settings") {
        b.follower.set(change)?;
        return Ok(Json(b.follower.status()));
    }
    if !body["stop"].as_bool().unwrap_or(false) {
        let_go_of_motion(&b).await;
    }
    if body["stop"].as_bool().unwrap_or(false) {
        b.end_takeover();
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
        let here = b.game.lk().others();
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
    b.end_takeover();
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

async fn anim_params(State(b): State<App>) -> Json<Value> {
    let mut v = serde_json::to_value(b.anim.params()).unwrap_or_default();
    v["live"] = b.anim.live.lk().clone();
    Json(v)
}

async fn anim_tune(State(b): State<App>, Body(body): Body) -> Reply {
    let p = b.anim.tune(&body)?;
    if body.get("head_height").is_some() {
        let h = p.head_height;
        on_headset(&b, move |vr, _| vr.set_head_height(h)).await?;
    }
    Ok(Json(serde_json::to_value(p)?))
}

/// What the bot sees ahead (JPEG): `width`, `pitch` (+ up). With the
/// panorama on: the view ahead out of it (100 degrees, square, as the
/// eyes'), the head left alone; `normal=1`: the eyes' usual view, the
/// panorama put aside a moment (a lease).
async fn screenshot(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> std::result::Result<Response, Fail> {
    b.require_game()?;
    let width: u32 = q.get("width").and_then(|w| w.parse().ok()).unwrap_or(0).min(3840);
    let pitch: Option<f32> = q.get("pitch").and_then(|p| p.parse().ok()).filter(|p: &f32| p.is_finite());
    let normal = q.get("normal").is_some_and(|v| v == "1" || v == "true");
    if !normal && b.pano.usable() {
        let pano = b.pano.clone();
        let made = tokio::task::spawn_blocking(move || -> anyhow::Result<Vec<u8>> {
            let f = pano.frame()?;
            vr::view_jpeg(&f, f.head.yaw, pitch.unwrap_or(0.0), FORWARD_FOV_DEG, if width == 0 { 960 } else { width })
        })
        .await?;
        match made {
            Ok(jpeg) => return Ok(([("Content-Type", "image/jpeg")], jpeg).into_response()),
            Err(e) => tracing::info!("screenshot: no view out of the panorama ({e:#}): the eyes"),
        }
    }
    if normal {
        b.pano.hold_normal(MENU_HOLD);
    }
    let jpeg = on_headset(&b, move |vr, b| {
        // The usual view asked for: the panorama off while the frame is taken.
        let _lease = if normal { Some(b.pano.normal_view(Duration::from_secs(2))?) } else { None };
        let frame = match pitch {
            Some(pitch) => vr.frame_looking(pitch)?,
            None => vr.frame()?,
        };
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
    let_go_of_motion(&b).await;
    let players = body["players"].as_bool().unwrap_or(true);
    // All around (the default here), or only ahead.
    let around = body["around"].as_bool().unwrap_or(true);
    // The scan turns the head, and a follow walks where the head looks.
    b.take_over();
    let whitelist = b.social.whitelist_names();
    let result = on_headset(&b, move |vr, b| {
        let v = vr.survey(&whitelist, players, around, &mut panolook::surveyor(b))?;
        survey_to_speakers(vr, b);
        Ok(v)
    })
    .await;
    b.idle_later();
    let mut v = result?;
    // Players heard speaking just now.
    for list in ["candidates", "players"] {
        for p in v[list].as_array_mut().into_iter().flatten() {
            if let Some(name) = p["name"].as_str().map(str::to_string) {
                p["talking"] = json!(b.speaker.talking(&name));
            }
        }
    }
    Ok(Json(v))
}

/// The players a survey placed, for the speaker tracker (with the frame
/// each was read from: their plates' glow).
fn survey_to_speakers(vr: &vr::VrCore, b: &App) {
    let Some(s) = vr.survey.as_ref() else { return };
    if s.pano.is_some() {
        return; // told as it looked (`panolook::Look::to_speakers`)
    }
    for p in &s.players {
        let frame = s.shots.iter().find(|shot| shot.frame.capture_ns == p.seen_ns).map(|shot| &shot.frame);
        b.speaker.saw(std::slice::from_ref(p), frame);
    }
}

async fn vr_goto(State(b): State<App>, Body(body): Body) -> Reply {
    let_go_of_motion(&b).await;
    b.require_game()?;
    b.take_over();
    let whitelist = b.social.whitelist_names();
    let since = vrc_vr::walk::stops();
    let result = on_headset(&b, move |vr, b| {
        let v = vr.goto(&whitelist, &body, since, &mut panolook::surveyor(b))?;
        survey_to_speakers(vr, b);
        Ok(v)
    })
    .await;
    b.idle_later();
    Ok(Json(result?))
}

// -- who is speaking ------------------------------------------------------------------

/// The speaker tracker: settings, the players placed, the recent segments
/// with who they were pinned on and why.
async fn speakers_status(State(b): State<App>) -> Json<Value> {
    Json(b.speaker.status())
}

/// `{"seconds": N}`: records N seconds (and the few before) for tuning,
/// into `speakers/rec-<unix ms>/` next to the token; `{"stop": true}` ends
/// the running one now.
async fn speakers_record(State(b): State<App>, Body(body): Body) -> Reply {
    if body["stop"].as_bool() == Some(true) {
        return Ok(Json(b.speaker.stop_recording()?));
    }
    let seconds = num(&body, "seconds", 30.0) as f32;
    Ok(Json(b.speaker.record(seconds)?))
}

/// Turns to whoever is speaking, or spoke last, pausing the follow
/// `pause_s` (default 8): `{"pause_s": 8, "since_ms": 4000, "name": "xkeyC"}`.
/// `since_ms` (a transcript's call, heard after its speech): the latest
/// speech that ended within it comes before the speech going on (without:
/// the speech going on, else what ended within 2 s). `name` (who the
/// transcript says spoke): their latest speech comes first. Only an idle
/// bot turns: following (or a follow paused for a move), moving, or with
/// the VR menu's usual view leased, it stays as it is and answers
/// `{"ok": false, "reason": "busy", "busy": "following" | "moving" | "menu"}`.
async fn vr_attend(State(b): State<App>, Body(body): Body) -> Reply {
    let pause = Duration::from_secs_f64(num(&body, "pause_s", 8.0).clamp(1.0, 120.0));
    let ended_first = body["since_ms"].is_number();
    let since = Duration::from_millis(num(&body, "since_ms", 2000.0).clamp(0.0, 8000.0) as u64);
    let name = body["name"].as_str().map(str::trim).filter(|n| !n.is_empty());
    if !b.speaker.enabled {
        return Ok(Json(json!({"ok": false, "reason": "the speaker tracker is off"})));
    }
    let following = !b.follower.is_idle() || !b.room_state()["takeover"].is_null();
    let moving = b.mapping.moved_within(Duration::from_secs(1));
    let leased = b.pano.status()["leases"].as_u64().unwrap_or(0) > 0;
    if let Some(busy) = speaker::attend_busy(following, moving, leased) {
        return Ok(Json(json!({"ok": false, "reason": "busy", "busy": busy})));
    }
    let Some(src) = b.speaker.source(since, name, ended_first) else {
        return Ok(Json(json!({"ok": false, "reason": "no recent speech"})));
    };
    let_go_of_motion(&b).await;
    b.require_game()?;
    b.take_over();
    let result = on_headset(&b, move |vr, b| b.speaker.attend(vr, &src)).await;
    b.idle_later_for(pause);
    Ok(Json(result?))
}

// -- the lasting map -----------------------------------------------------------------

/// Things in the latest frame of the eyes; `{"save": true}` keeps the
/// frame and the answer (`detect/` next to the token) to judge the detector by.
async fn vr_detect(State(b): State<App>, Body(body): Body) -> Reply {
    let save = body["save"].as_bool().unwrap_or(false).then(|| {
        game::expand(&b.args.token_file).parent().map(|p| p.join("detect")).unwrap_or_else(|| "detect".into())
    });
    // The detector and the stereo want the usual view (a tuning route).
    Ok(Json(
        on_headset(&b, move |vr, b| {
            let _lease = b.pano.normal_view(Duration::from_secs(2))?;
            vr.detect(save)
        })
        .await?,
    ))
}

/// The position beacon in the latest frame of the eyes.
async fn vr_beacon(State(b): State<App>) -> Reply {
    Ok(Json(on_headset(&b, |vr, _| vr.beacon()).await?))
}

// -- the panorama rig -----------------------------------------------------------------

/// The rig: wanted, sent, what the eyes showed last, the last code, the
/// last frame decoded. Looks at the latest frame first (`?peek=1`: not).
async fn vr_pano_status(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> Reply {
    if !q.get("peek").is_some_and(|v| v == "1" || v == "true") {
        let p = b.pano.clone();
        let looked = tokio::task::spawn_blocking(move || p.observe_now().map(|_| ())).await?;
        if let Err(e) = looked {
            let mut v = b.pano.status();
            v["error"] = json!(format!("{e:#}"));
            return Ok(Json(v));
        }
    }
    Ok(Json(b.pano.status()))
}

/// `{"on": true}`: wants the rig on (off: `false`), then waits (1.5 s at
/// most) for the eyes to show it so.
async fn vr_pano_set(State(b): State<App>, Body(body): Body) -> Reply {
    let on = body["on"].as_bool().context("on: true or false")?;
    b.pano.want_pano(on);
    let p = b.pano.clone();
    tokio::task::spawn_blocking(move || {
        let since = std::time::Instant::now();
        while since.elapsed() < Duration::from_millis(1500) {
            if let Ok((_, seen)) = p.observe_now() {
                if seen.is_pano() == on && since.elapsed() >= Duration::from_millis(150) {
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    })
    .await?;
    Ok(Json(b.pano.status()))
}

/// The latest pano frame as JPEG: `kind` `color` (default: the
/// equirectangular panorama, the head's heading in the middle), `depth`
/// (the same, the depth coloured: near red, far blue, none black) or
/// `tiles` (the six faces as the eyes hold them, colour left of depth);
/// `width` (default 2048; tiles: 1920).
async fn vr_pano_jpg(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> std::result::Result<Response, Fail> {
    let kind = q.get("kind").map_or("color", |k| k.as_str()).to_string();
    let width: usize = q.get("width").and_then(|w| w.parse().ok()).unwrap_or(0);
    let p = b.pano.clone();
    let jpeg = tokio::task::spawn_blocking(move || -> Result<Vec<u8>> {
        let f = p.frame()?;
        let (w, h, rgb) = match kind.as_str() {
            "color" | "colour" | "depth" => {
                let e = f.heading_equirect(if width == 0 { 2048 } else { width.clamp(256, 4096) });
                let rgb = if kind == "depth" { vrc_pano::depth_rgb(&e.range, f.code.zmin, f.code.zmax) } else { e.pano.rgb };
                (e.pano.width, e.pano.height, rgb)
            }
            "tiles" => {
                let (w, h, rgb) = f.tiles_rgb();
                vrc_pano::downscale(&rgb, w, h, if width == 0 { 1920 } else { width.clamp(256, w) })
            }
            other => bail!("kind is color, depth or tiles, not {other}"),
        };
        let mut jpeg = Vec::new();
        jpeg_encoder::Encoder::new(&mut jpeg, 85).encode(&rgb, w as u16, h as u16, jpeg_encoder::ColorType::Rgb)?;
        Ok(jpeg)
    })
    .await??;
    Ok(([("Content-Type", "image/jpeg")], jpeg).into_response())
}

/// World points of the latest pano frame, every `step`-th pixel (default
/// 16) of every face: JSON (`{"points": [[x, y, z, r, g, b], ...]}`, the
/// map's axes: x, y, -z of Unity's) or `format=ply` (ASCII, Unity's axes
/// as they are). At most 200k points.
async fn vr_pano_points(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> std::result::Result<Response, Fail> {
    let step: u32 = q.get("step").and_then(|v| v.parse().ok()).unwrap_or(16).clamp(1, 256);
    let ply = q.get("format").is_some_and(|f| f == "ply");
    let p = b.pano.clone();
    let (f, pts) = tokio::task::spawn_blocking(move || -> Result<_> {
        let f = p.frame()?;
        let mut pts = f.points(step);
        pts.truncate(200_000);
        Ok((f, pts))
    })
    .await??;
    if ply {
        let mut out = format!(
            "ply\nformat ascii 1.0\ncomment vrc-pano: Unity's world, metres; rig yaw {:.2}\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
            f.code.rig_yaw,
            pts.len()
        );
        for q in &pts {
            // The UI's colour (unknown): magenta.
            let c = q.rgb.unwrap_or([255, 0, 255]);
            out.push_str(&format!("{:.3} {:.3} {:.3} {} {} {}\n", q.world[0], q.world[1], q.world[2], c[0], c[1], c[2]));
        }
        return Ok(([("Content-Type", "text/plain")], out).into_response());
    }
    let round = |v: f32| (v as f64 * 1000.0).round() / 1000.0;
    let list: Vec<Value> = pts
        .iter()
        .map(|q| {
            let m = q.map();
            match q.rgb {
                Some(c) => json!([round(m[0]), round(m[1]), round(m[2]), c[0], c[1], c[2]]),
                // The UI's colour: unknown.
                None => json!([round(m[0]), round(m[1]), round(m[2]), null, null, null]),
            }
        })
        .collect();
    Ok(Json(json!({
        "tap_seq": f.tap_seq,
        "position": f.code.position_map(),
        "rig_yaw": f.code.rig_yaw,
        "head_yaw": f.head.yaw,
        "step": step,
        "axes": "map: x, y, -z of Unity's world (metres)",
        "points": list,
    }))
    .into_response())
}

async fn map_status(State(b): State<App>) -> Json<Value> {
    Json(b.mapping.status())
}

/// `radius` (metres, default 8), `px` (pixels a 10 cm column, default 3),
/// `up` (1: the way the bot faces up), `to` (`x,z` on the map: a way there drawn).
async fn map_png(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> std::result::Result<Response, Fail> {
    let radius = q.get("radius").and_then(|v| v.parse::<f32>().ok()).unwrap_or(8.0).clamp(2.0, 40.0);
    let px = q.get("px").and_then(|v| v.parse::<usize>().ok()).unwrap_or(3).clamp(1, 8);
    let up = q.get("up").is_some_and(|v| v == "1" || v == "true");
    let to = q.get("to").and_then(|v| {
        let (x, z) = v.split_once(',')?;
        Some([x.trim().parse::<f32>().ok()?, z.trim().parse::<f32>().ok()?])
    });
    let m = b.mapping.clone();
    let png = tokio::task::spawn_blocking(move || m.png(radius, px, up, to)).await.map_err(|e| Fail(e.into()))??;
    Ok(([(axum::http::header::CONTENT_TYPE, "image/png")], png).into_response())
}

async fn map_save(State(b): State<App>) -> Json<Value> {
    let m = b.mapping.clone();
    let _ = tokio::task::spawn_blocking(move || m.save()).await;
    Json(b.mapping.status())
}

/// Starts this world's map anew (the old file kept aside).
async fn map_forget(State(b): State<App>) -> Reply {
    let m = b.mapping.clone();
    tokio::task::spawn_blocking(move || m.forget()).await.map_err(|e| Fail(e.into()))??;
    Ok(Json(b.mapping.status()))
}

/// Names where the bot stands: `{"name": "..."}`.
async fn map_place(State(b): State<App>, Body(body): Body) -> Reply {
    let name = body["name"].as_str().map(str::trim).filter(|n| !n.is_empty()).context("name is required")?;
    // Standing as usual: sitting, lying or in a motion the head is not over
    // the feet, and the place would be off (and so would the way it faces).
    if let Some(posture) = motion::holding(&b) {
        bail_fail(&format!("you are {posture} now: stand up first (vrchat_posture stand), then remember the place"))?;
    }
    if b.anim.motion.lk().is_some() {
        bail_fail("a motion is playing: wait for it to end (or stop it), standing as usual, then remember the place")?;
    }
    if b.vr.try_lk().is_some_and(|vr| vr.lean != [0.0; 3]) {
        bail_fail("your head is leaned or bent: stand straight first, then remember the place")?;
    }
    let at = b.mapping.name_place(name)?;
    Ok(Json(json!({"ok": true, "name": name, "at": at})))
}

async fn vr_height(State(b): State<App>) -> Json<Value> {
    Json(json!({"head_height": cm(b.anim.params().head_height)}))
}

/// Metres to the centimetre.
fn cm(metres: f32) -> f64 {
    (metres as f64 * 100.0).round() / 100.0
}

/// The headset's height: `metres` (1.2-1.9), or `change_cm` from now.
async fn vr_set_height(State(b): State<App>, Body(body): Body) -> Reply {
    let now = b.anim.params().head_height;
    let h = match (body["metres"].as_f64(), body["change_cm"].as_f64()) {
        (Some(m), _) => m as f32,
        (None, Some(cm)) => now + cm as f32 / 100.0,
        _ => return Err(anyhow::anyhow!("metres, or change_cm").into()),
    };
    let h = (h.clamp(vr::MIN_HEAD_HEIGHT, vr::MAX_HEAD_HEIGHT) * 100.0).round() / 100.0;
    b.anim.tune(&json!({"head_height": h}))?;
    on_headset(&b, move |vr, _| vr.set_head_height(h)).await?;
    Ok(Json(json!({"head_height": cm(h), "was": cm(now)})))
}

/// Like SteamVR's reset: stops moving, connects the headset again, looks
/// level ahead with the hands at rest, and recenters Monado's local spaces.
async fn vr_reset(State(b): State<App>) -> Reply {
    let_go_of_motion(&b).await;
    b.take_over();
    vrc_vr::walk::stop_all();
    for axis in ["/input/Vertical", "/input/Horizontal"] {
        let _ = b.osc.send_f32(axis, 0.0);
    }
    let yaw = on_headset(&b, |vr, b| {
        let yaw = vr.yaw;
        vr.forget_places();
        vr.reset();
        vr.face(yaw, 0.0)?;
        *b.anim.head_lean.lk() = [0.0; 3];
        Ok(yaw)
    })
    .await;
    b.idle_later();
    let yaw = yaw?;
    let recentered = if b.args.monado_ctl.is_empty() {
        None
    } else {
        let out = tokio::process::Command::new(&b.args.monado_ctl).arg("-c").output().await;
        Some(matches!(out, Ok(ref o) if o.status.success()))
    };
    b.notify_state();
    Ok(Json(json!({"ok": true, "facing_deg": yaw.round(), "head_height": cm(b.anim.params().head_height), "recentered": recentered})))
}

/// The corridor ahead as the follower sees it (tuning): `yaw` to look along
/// another way than the head's.
async fn vr_corridor(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> Reply {
    let yaw: Option<f32> = q.get("yaw").and_then(|v| v.parse().ok()).filter(|y: &f32| y.is_finite());
    let height = b.osc_query().and_then(|o| o.eye_height()).unwrap_or(0.0) as f32;
    let metres = if height > 0.0 { height / (b.anim.params().head_height - vrc_vr::remote::FLOOR_Y) } else { 1.0 };
    // The stereo wants the usual view (a tuning route).
    let frame = on_headset(&b, |vr, b| {
        let _lease = b.pano.normal_view(Duration::from_secs(2))?;
        vr.frame()
    })
    .await?;
    Ok(Json(tokio::task::spawn_blocking(move || follow::corridor_report(&frame, yaw, metres)).await??))
}

async fn vr_pano(State(b): State<App>) -> std::result::Result<Response, Fail> {
    let jpeg = on_headset(&b, |vr, _| vr::pano_jpeg(vr.survey.as_ref().context("no survey yet")?)).await?;
    Ok(([("Content-Type", "image/jpeg")], jpeg).into_response())
}

async fn vr_map(State(b): State<App>) -> std::result::Result<Response, Fail> {
    let png = on_headset(&b, |vr, _| vr::map_png(vr.survey.as_ref().context("no survey yet")?)).await?;
    Ok(([("Content-Type", "image/png")], png).into_response())
}

// -- full body (trying it out) ----------------------------------------------------------

/// The OSC trackers as set, and what they would send now (Unity's terms).
fn trackers_json(b: &App) -> Result<Value> {
    use vrc_vr::trackers;
    let settings = b.anim.trackers.lk().clone();
    settings.parts()?;
    let state = b.vr.try_lk().and_then(|vr| vr.link()).and_then(|l| l.owner()).map(|o| o.state);
    let round = |v: [f32; 3]| v.map(|x| (x as f64 * 1000.0).round() / 1000.0);
    let mut v = serde_json::to_value(&settings)?;
    match state.map(|s| b.anim.tracker_frame(&settings, &s)) {
        Some(Ok(frame)) => {
            v["applied_scale"] = json!(frame.scale);
            v["sending"] = frame
                .trackers
                .iter()
                .map(|(part, pose)| {
                    let (p, r) = trackers::to_unity(pose);
                    json!({"part": format!("{part:?}"), "slot": part.slot(), "position": round(p), "rotation": round(r)})
                })
                .collect();
            let (p, r) = trackers::to_unity(&frame.head);
            v["head"] = json!({"position": round(p), "rotation": round(r)});
        }
        Some(Err(e)) => v["error"] = json!(format!("{e:#}")),
        None => v["error"] = json!("no headset yet"),
    }
    Ok(v)
}

async fn vr_trackers(State(b): State<App>) -> Reply {
    Ok(Json(trackers_json(&b)?))
}

/// `{"on": true, "parts": ["hip", "feet"], "head": "once"}` (any of them):
/// sets the OSC trackers; the head's rotation is sent again.
async fn vr_set_trackers(State(b): State<App>, Body(body): Body) -> Reply {
    let mut v = serde_json::to_value(b.anim.trackers.lk().clone())?;
    for (k, val) in body.as_object().into_iter().flatten() {
        if v.get(k).is_none() {
            return Err(anyhow::anyhow!("no setting {k} (on, parts, head, shift, scale, gait, auto_calibrate)").into());
        }
        v[k] = val.clone();
    }
    let settings: anim::TrackerSettings = serde_json::from_value(v)?;
    b.anim.set_trackers(settings)?;
    Ok(Json(trackers_json(&b)?))
}

/// Sets a hand by hand: `{"hand": "right", "offset": [right, up, ahead],
/// "turn": [yaw, pitch, roll], "trigger": 0..1, "buttons": ["b"],
/// "squeeze": 0..1, "press_ms": 150}` (a press: let go after);
/// `{"release": true}` gives the hands back to the animation, at rest.
/// A squeeze (the grip: VRChat grabs pickups with it) holds across calls
/// until `"squeeze": 0`, a press's let go, or a release: a pickup is
/// dragged by moving the hand in small steps while it holds.
async fn vr_hand(State(b): State<App>, Body(body): Body) -> Reply {
    let_go_of_motion(&b).await;
    // Hands at work: the VR menu, read in the usual view (several calls:
    // held a while).
    b.pano.hold_normal(MENU_HOLD);
    use std::sync::atomic::Ordering;
    if body["release"].as_bool() == Some(true) {
        b.anim.manual_hands.store(false, Ordering::Relaxed);
        on_headset(&b, |vr, _| {
            let yaw = vr.yaw;
            vr.release_hands()?;
            vr.face(yaw, 0.0)
        })
        .await?;
        return Ok(Json(json!({"ok": true, "hands": "animation"})));
    }
    let hand = body["hand"].as_str().unwrap_or("right").to_string();
    let triple = |key: &str| -> Result<[f32; 3]> {
        match body.get(key) {
            None => Ok([0.0; 3]),
            Some(v) => {
                let a: Vec<f32> = serde_json::from_value(v.clone())?;
                anyhow::ensure!(a.len() == 3 && a.iter().all(|x| x.is_finite()), "{key} is three numbers");
                Ok([a[0], a[1], a[2]])
            }
        }
    };
    let offset = triple("offset")?;
    if offset.iter().any(|x| x.abs() > 1.5) {
        return Err(anyhow::anyhow!("offset within 1.5 m").into());
    }
    let turn = triple("turn")?;
    let trigger = num(&body, "trigger", 0.0) as f32;
    let buttons: Vec<String> = serde_json::from_value(body.get("buttons").cloned().unwrap_or(json!([])))?;
    let press = Duration::from_millis(num(&body, "press_ms", 0.0).clamp(0.0, 5000.0) as u64);
    let squeeze = match body.get("squeeze") {
        None => None,
        Some(v) => Some(v.as_f64().filter(|x| (0.0..=1.0).contains(x)).context("squeeze is 0..1")? as f32),
    };
    b.anim.manual_hands.store(true, Ordering::Relaxed);
    let pose = on_headset(&b, move |vr, _| {
        let pose = vr.set_hand(&hand, offset, turn, trigger, &buttons)?;
        if let Some(s) = squeeze {
            vr.set_squeeze(&hand, s)?;
        }
        if !press.is_zero() {
            std::thread::sleep(press);
            vr.release_hands()?;
        }
        Ok(pose)
    })
    .await?;
    Ok(Json(json!({"ok": true, "grip": {"position": pose.position, "orientation": pose.orientation}})))
}

/// Presses one of VRChat's OSC inputs (`/input/<name>`, a button: 1, then
/// 0 after `press_ms`), e.g. `QuickMenuToggleLeft`.
async fn vr_input(State(b): State<App>, Body(body): Body) -> Reply {
    let name = body["name"].as_str().unwrap_or("").to_string();
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric()) {
        return Err(anyhow::anyhow!("name: an /input/ button").into());
    }
    let address = format!("/input/{name}");
    if name.contains("Menu") {
        // The VR menu: read in the usual view.
        b.pano.hold_normal(MENU_HOLD);
    }
    let press = Duration::from_millis(num(&body, "press_ms", 150.0).clamp(20.0, 5000.0) as u64);
    // A movement input waits at the move gate a moment (the user camera's lens).
    let sent = address.clone();
    tokio::task::spawn_blocking(move || -> Result<()> {
        b.osc.send_i32(&sent, 1)?;
        std::thread::sleep(press);
        b.osc.send_i32(&sent, 0)
    })
    .await??;
    Ok(Json(json!({"ok": true, "input": address})))
}

/// The head off where it stands: `{"bend": degrees}` bends forward at
/// the hips, hands behind the back (0: straight again, the hands back to
/// the animation), e.g. to look past what the avatar wears at its own feet
/// (then `/v1/screenshot?pitch=-80`); or `{"lean": [right, up, ahead]}`
/// moves the head alone (metres).
async fn vr_head(State(b): State<App>, Body(body): Body) -> Reply {
    let_go_of_motion(&b).await;
    use std::sync::atomic::Ordering;
    let (head, off) = if let Some(deg) = body["bend"].as_f64() {
        if !(0.0..=75.0).contains(&deg) {
            return Err(anyhow::anyhow!("bend is 0-75 degrees").into());
        }
        let deg = deg as f32;
        if deg > 0.0 {
            b.anim.manual_hands.store(true, Ordering::Relaxed);
        }
        let out = on_headset(&b, move |vr, _| {
            let out = vr.bend_over(deg)?;
            if deg == 0.0 {
                let yaw = vr.yaw;
                vr.face(yaw, 0.0)?;
            }
            Ok(out)
        })
        .await?;
        if deg == 0.0 {
            b.anim.manual_hands.store(false, Ordering::Relaxed);
        }
        out
    } else {
        let lean: Vec<f32> = serde_json::from_value(body.get("lean").cloned().unwrap_or(json!([0, 0, 0])))?;
        if lean.len() != 3 || lean.iter().any(|x| !x.is_finite() || x.abs() > 0.8) {
            return Err(anyhow::anyhow!("lean is three numbers within 0.8 m").into());
        }
        on_headset(&b, move |vr, _| vr.lean_head([lean[0], lean[1], lean[2]])).await?
    };
    // The trackers stay where the body stands.
    *b.anim.head_lean.lk() = off;
    Ok(Json(json!({"ok": true, "head": head})))
}

/// Whether VRChat tracks the full body now (its TrackingType: 6 with hip
/// and feet trackers calibrated, 3 head and hands).
async fn vr_calibrate_status(State(b): State<App>) -> Reply {
    let tt = tokio::task::spawn_blocking({
        let b = b.clone();
        move || calibrate::tracking_type(&b)
    })
    .await?;
    Ok(Json(json!({"tracking_type": tt, "full_body": tt == Some(calibrate::FULL_BODY), "trackers_on": b.anim.trackers.lk().on})))
}

/// Calibrates full body by itself (`calibrate`): turns the trackers on if
/// they are not, then the Quick Menu, 校准, both triggers. A body already
/// tracked in full is left alone unless `{"force": true}`.
async fn vr_calibrate(State(b): State<App>, Body(body): Body) -> Reply {
    let force = body["force"].as_bool().unwrap_or(false);
    b.require_game()?;
    let report = tokio::task::spawn_blocking(move || calibrate::now(&b, force)).await??;
    Ok(Json(report))
}

// -- the user camera ----------------------------------------------------------------

/// The user camera as VRChat reports it (mode, flying, the UI mask) and
/// the bridge's settings, the last opening's report and pose.
async fn vr_usercam_status(State(b): State<App>) -> Reply {
    Ok(Json(tokio::task::spawn_blocking(move || usercam::status(&b)).await?))
}

/// `{"open": true}` opens VRChat's user camera and sets it up as a remote
/// eye (`usercam`: the Quick Menu, a double-click on its camera icon, the
/// viewfinder dragged into the body, stream mode and the UI mask, flying
/// off, the desktop checked); `"stow": false` leaves the viewfinder,
/// `"check": false` skips the desktop check, `"keep_open"` (default true
/// with `open`) opens it again by itself after the game starts or a world
/// is joined. `{"close": true}` closes it (and stops keeping it open).
/// `"orbit"`: true, false or the lens's settings to change (`orbit`: the
/// lens while the bot stands, the travel lens while it moves, the name
/// sightings), kept in `usercam.json`; alone it changes them and answers
/// the settings. `{"stow": true}` without `open` stows the viewfinder of
/// an open camera (`usercam::restow`: closed and opened again, the
/// viewfinder dragged into the body as it spawns).
async fn vr_usercam(State(b): State<App>, Body(body): Body) -> Reply {
    b.require_game()?;
    let mut settings = b.usercam.settings.lk().clone();
    let restow = body["stow"].as_bool() == Some(true) && body["open"].as_bool() != Some(true) && body["close"].as_bool() != Some(true);
    if let Some(s) = body["stow"].as_bool() {
        settings.stow = s;
    }
    if let Some(o) = body.get("orbit") {
        settings.orbit = settings.orbit.merged(o)?;
    }
    if body["close"].as_bool() == Some(true) {
        settings.keep_open = body["keep_open"].as_bool().unwrap_or(false);
        b.usercam.set(settings)?;
        let v = tokio::task::spawn_blocking(move || usercam::close(&b)).await??;
        return Ok(Json(v));
    }
    if body["open"].as_bool() != Some(true) && !restow {
        if let Some(k) = body["keep_open"].as_bool() {
            settings.keep_open = k;
        }
        b.usercam.set(settings.clone())?;
        return Ok(Json(json!({"ok": true, "settings": settings})));
    }
    settings.keep_open = body["keep_open"].as_bool().unwrap_or(!restow || settings.keep_open);
    b.usercam.set(settings.clone())?;
    let delta = match body.get("stow_delta") {
        None => usercam::STOW_DELTA,
        Some(v) => {
            let a: Vec<f32> = serde_json::from_value(v.clone())?;
            if a.len() != 3 || a.iter().any(|x| !x.is_finite() || x.abs() > 1.0) {
                return Err(anyhow::anyhow!("stow_delta is three numbers within 1 m (right, up, ahead)").into());
            }
            [a[0], a[1], a[2]]
        }
    };
    let opts = usercam::OpenOptions { stow: settings.stow, stow_delta: delta, check: body["check"].as_bool().unwrap_or(true) };
    let report = if restow {
        tokio::task::spawn_blocking(move || usercam::restow(&b, &opts)).await??
    } else {
        tokio::task::spawn_blocking(move || usercam::open(&b, &opts)).await??
    };
    Ok(Json(report))
}

/// The names the user camera's lens read lately (`orbit`), oldest first:
/// `?since_ms=10000` (at most 60 s): each with its bearing from the head
/// (world, tracking and off the head's yaw), the lens (orbit or travel)
/// and the plate's ring score.
/// The people near the bot (`people`, D39): name, place (world and
/// tracking), bearing and distance from the head, when last placed and
/// named, how; and the idle sweep's schedule.
async fn vr_people(State(b): State<App>) -> Json<Value> {
    let every = b.usercam.settings.lk().orbit.idle_sweep_s;
    Json(b.people.status(every))
}

async fn vr_usercam_names(State(b): State<App>, Query(q): Query<std::collections::HashMap<String, String>>) -> Json<Value> {
    let since_ms = q.get("since_ms").and_then(|v| v.parse::<u64>().ok()).unwrap_or(10_000).min(60_000);
    let now = std::time::Instant::now();
    let since = now.checked_sub(Duration::from_millis(since_ms)).unwrap_or(now);
    let names: Vec<Value> = b
        .orbit
        .names_since(since)
        .into_iter()
        .map(|n| {
            let mut v = serde_json::to_value(&n).unwrap_or_default();
            v["ago_ms"] = json!(now.saturating_duration_since(n.at).as_millis() as u64);
            v
        })
        .collect();
    Json(json!({"names": names}))
}

/// Turns the lens a moment toward a bearing to read the name there
/// (`Orbit::name_toward`): `{"bearing_deg": 40}` (from where the head looks,
/// + right), `"tol_deg"` (default 15), `"wait_ms"` (default 1500, at most
/// 5000). A name read that way in the last second answers at once.
/// Answers the sighting, or `"name": null`.
async fn vr_usercam_look(State(b): State<App>, Body(body): Body) -> Reply {
    b.require_game()?;
    let bearing = body["bearing_deg"].as_f64().filter(|v| v.is_finite()).ok_or_else(|| anyhow::anyhow!("bearing_deg is a number"))? as f32;
    let tol = body["tol_deg"].as_f64().unwrap_or(15.0).clamp(1.0, 90.0) as f32;
    let wait = Duration::from_millis(body["wait_ms"].as_u64().unwrap_or(1500).min(5000));
    let head = b.orbit.head().ok_or_else(|| anyhow::anyhow!("no head (the position beacon) yet"))?;
    let world = (head.yaw + bearing).rem_euclid(360.0);
    let orbit = b.orbit.clone();
    let seen = tokio::task::spawn_blocking(move || orbit.name_toward(world, tol, Duration::from_secs(1), wait)).await??;
    Ok(Json(json!({"ok": true, "world_yaw": world, "name": seen.as_ref().map(|n| n.name.clone()), "sighting": seen})))
}

/// Places the user camera and returns what it sees (JPEG; the pose in
/// `X-Usercam-Pose`: x,y,z,pitch,yaw,roll as `/usercamera/Pose` takes
/// them): `{"pose": [x, y, z, pitch, yaw, roll]}` (world), or round the
/// head: `{"bearing_deg": 0, "distance_m": 0, "height_m": 0.3, "look":
/// "out"|"back", "pitch_deg": 0}`.
async fn vr_usercam_shot(State(b): State<App>, Body(body): Body) -> std::result::Result<Response, Fail> {
    b.require_game()?;
    let aim = usercam::aim_from(&body)?;
    let (pose, rgb) = tokio::task::spawn_blocking(move || usercam::shot(&b, aim)).await??;
    let jpeg = rgb.jpeg()?;
    let a = pose.args();
    let header = format!("{:.3},{:.3},{:.3},{:.2},{:.2},{:.2}", a[0], a[1], a[2], a[3], a[4], a[5]);
    Ok(([("Content-Type", "image/jpeg".to_string()), ("X-Usercam-Pose", header)], jpeg).into_response())
}

/// A ring of shots round the head: `{"views": 6, "distance_m": 0,
/// "height_m": 0.3, "look": "out"|"back", "pitch_deg": 0}`; one JPEG, the
/// views three to a row (640x360 each), clockwise from straight ahead (the
/// bearings in `X-Usercam-Bearings`).
async fn vr_usercam_sweep(State(b): State<App>, Body(body): Body) -> std::result::Result<Response, Fail> {
    b.require_game()?;
    let views = num(&body, "views", 6.0).clamp(2.0, 12.0) as usize;
    let usercam::Aim::Around { distance, height, look, pitch_up, .. } = usercam::aim_from(&body)? else {
        return Err(anyhow::anyhow!("a sweep goes round the head: no pose").into());
    };
    let (bearings, rgb) = tokio::task::spawn_blocking(move || usercam::sweep(&b, views, distance, height, look, pitch_up)).await??;
    let jpeg = rgb.jpeg()?;
    let list = bearings.iter().map(|v| format!("{v:.0}")).collect::<Vec<_>>().join(",");
    Ok(([("Content-Type", "image/jpeg".to_string()), ("X-Usercam-Bearings", list)], jpeg).into_response())
}

// -- motion programs ----------------------------------------------------------------

/// The clips, and the program playing.
async fn motion_status(State(b): State<App>) -> Json<Value> {
    let playing = b.anim.motion.lk().as_ref().map(motion::Program::status);
    Json(json!({"clips": b.motions.list(), "playing": playing}))
}

/// Plays a program: `{"steps": [{"clip": "wave", "mirror": false, "speed":
/// 1, "seconds": 5, "in_place": false, "fade": 0.6}, ...]}`, one clip as
/// `{"clip": "wave", ...}`, or a posture: `{"posture": "lie", "way":
/// "left"}` (stand, sit, lie: back, left, right, front). Answers at once unless `{"wait": true}`; a new
/// program stops the one playing.
async fn motion_play(State(b): State<App>, Body(body): Body) -> Reply {
    b.require_game()?;
    let steps: Vec<motion::Step> = if let Some(posture) = body["posture"].as_str() {
        let held = motion::holding(&b);
        match motion::posture_steps(posture, body["way"].as_str(), held.as_deref())? {
            Some(steps) => steps,
            None => {
                // Standing: whatever holds a posture gets up.
                motion::STOP.store(true, std::sync::atomic::Ordering::SeqCst);
                return Ok(Json(json!({"ok": true, "standing_up": held.is_some()})));
            }
        }
    } else {
        match body.get("steps") {
            Some(s) => serde_json::from_value(s.clone())?,
            None => vec![serde_json::from_value(body.clone())?],
        }
    };
    for s in &steps {
        if b.motions.get(&s.clip).is_none() {
            return Err(anyhow::anyhow!("no motion {} (GET /v1/motion lists them)", s.clip).into());
        }
    }
    let wait = body["wait"].as_bool().unwrap_or(false);
    let task = tokio::task::spawn_blocking({
        let b = b.clone();
        move || motion::play(&b, steps)
    });
    if wait {
        return Ok(Json(task.await??));
    }
    tokio::spawn(async move {
        match task.await {
            Ok(Ok(v)) => tracing::info!("motion: {v}"),
            Ok(Err(e)) => tracing::warn!("motion: {e:#}"),
            Err(e) => tracing::warn!("motion: {e}"),
        }
    });
    Ok(Json(json!({"ok": true, "playing": true})))
}

/// Stops the program playing (a held posture gets up first).
async fn motion_stop(State(_b): State<App>) -> Json<Value> {
    motion::STOP.store(true, std::sync::atomic::Ordering::SeqCst);
    Json(json!({"ok": true}))
}

async fn motion_reload(State(b): State<App>) -> Reply {
    let names = b.motions.reload()?;
    Ok(Json(json!({"clips": names})))
}

/// Stops a motion program (a held posture gets up first) and waits until it
/// lets go of the headset: before anything else moves the body.
async fn let_go_of_motion(b: &App) {
    let b = b.clone();
    let _ = tokio::task::spawn_blocking(move || motion::stop_and_wait(&b)).await;
}
