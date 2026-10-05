//! The bridge's shared state and its background work: the stream client,
//! audio both ways, the chatbox queue, the log tail and the watchdog.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::extract::ws::Message;
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::process::{Child, Command};
use tokio::sync::mpsc;
use vrc_vr::osc::{Arg, Osc};

use crate::anim::Anim;
use crate::follow::Follower;
use crate::game::{self, GameState, LogTail};
use crate::sightings::Sightings;
use crate::social::Social;
use crate::vr::VrCore;
use crate::Args;

pub const SAMPLE_RATE: u32 = 48_000;
/// 20 ms of 16-bit mono.
pub const FRAME_BYTES: usize = 960 * 2;
/// Push-to-talk stays held this long after the bot's audio has played out,
/// so the end of a sentence is not cut by the release.
const PTT_TAIL: Duration = Duration::from_millis(300);
/// Latency of the playback stream into the microphone sink.
const PLAYBACK_LATENCY_MS: u64 = 60;
/// Outbound frames queued for a slow client before new ones are dropped.
const CLIENT_QUEUE: usize = 200;
pub const CHATBOX_LIMIT: usize = 144;
/// VRChat throttles chatbox spam; messages are paced at least this far apart.
const CHATBOX_INTERVAL: Duration = Duration::from_millis(1600);
const WATCHDOG_INTERVAL: Duration = Duration::from_secs(5);

pub struct Bridge {
    pub args: Args,
    pub token: String,
    pub osc: Osc,
    pub game: Mutex<GameState>,
    /// The stream client (one at a time): an id and its outbound queue.
    client: Mutex<Option<(u64, mpsc::Sender<Message>)>>,
    client_ids: AtomicU64,
    player: tokio::sync::Mutex<Player>,
    chatbox: mpsc::Sender<(String, bool)>,
    /// One game stop/start at a time (joins, /v1/game/start).
    pub game_lock: tokio::sync::Mutex<()>,
    /// Leaving an instance the bot must not be in.
    leaving: AtomicBool,
    pub social: Arc<Social>,
    pub vr: Arc<Mutex<VrCore>>,
    pub follower: Arc<Follower>,
    pub sightings: Arc<Sightings>,
    pub anim: Arc<Anim>,
}

#[derive(Default)]
struct Player {
    child: Option<Child>,
    ptt_held: bool,
    /// When the bot's queued audio has played out.
    speech_until: Option<Instant>,
    release_pending: bool,
}

impl Bridge {
    pub fn new(args: Args, token: String) -> (Arc<Bridge>, mpsc::Receiver<(String, bool)>) {
        let osc = Osc::with_ports(&format!("127.0.0.1:{}", args.osc_port), 0).expect("an OSC socket");
        let (chat_tx, chat_rx) = mpsc::channel(20);
        let config_dir = game::expand(&args.token_file).parent().map(|p| p.to_path_buf()).unwrap_or_default();
        let social = Arc::new(Social::new(config_dir.join("social.json"), config_dir.join("cookies.json")));
        let vr = Arc::new(Mutex::new(VrCore::new(&args)));
        let bridge = Arc::new(Bridge {
            token,
            osc,
            game: Mutex::new(GameState::default()),
            client: Mutex::new(None),
            client_ids: AtomicU64::new(0),
            player: tokio::sync::Mutex::new(Player::default()),
            chatbox: chat_tx,
            game_lock: tokio::sync::Mutex::new(()),
            leaving: AtomicBool::new(false),
            social,
            vr,
            follower: Arc::new(Follower::default()),
            sightings: Arc::new(Sightings::default()),
            anim: Arc::new(Anim::new(config_dir.join("anim.json"))),
            args,
        });
        (bridge, chat_rx)
    }

    // -- events ---------------------------------------------------------------

    /// Sends a text event to the stream client, if any.
    pub fn send_event(&self, data: Value) {
        let follow = data["type"] == "follow";
        if let Some((_, tx)) = self.client.lock().unwrap().as_ref() {
            let _ = tx.try_send(Message::Text(data.to_string().into()));
        }
        if follow {
            self.notify_state(); // whom it follows, and how, is room state
        }
    }

    /// The game state, each other player marked as a friend (VRChat's friend
    /// list) and by their whitelist rank (1 first, 0 not on it), and whom the
    /// avatar follows (null when nobody).
    pub fn room_state(&self) -> Value {
        let mut snap = self.game.lock().unwrap().snapshot();
        let ranks = self.social.whitelist_ids();
        let friends = self.social.friend_ids();
        if let Some(players) = snap["players"].as_array_mut() {
            for p in players {
                let id = p["id"].as_str().unwrap_or("").to_string();
                p["friend"] = json!(friends.contains(&id));
                p["whitelist"] = json!(ranks.iter().position(|r| *r == id).map_or(0, |i| i + 1));
            }
        }
        let f = self.follower.status();
        snap["follow"] = if f["state"] == "idle" { Value::Null } else { json!({"target": f["target"], "state": f["state"]}) };
        snap["takeover"] = Value::Null;
        snap
    }

    /// Pushes the room state (and guards the instance).
    pub fn notify_state(&self) {
        if let Some((_, tx)) = self.client.lock().unwrap().as_ref() {
            let _ = tx.try_send(Message::Text(json!({"type": "state", "state": self.room_state()}).to_string().into()));
        }
    }

    pub fn state_changed(self: &Arc<Self>) {
        self.guard_instance();
        self.notify_state();
    }

    /// Leaves an instance the bot must not be in (public, group, ...): only
    /// joins it starts itself are checked, yet a portal or a followed player
    /// can take it anywhere.
    fn guard_instance(self: &Arc<Self>) {
        let (running, instance) = {
            let g = self.game.lock().unwrap();
            (g.running, g.instance.clone())
        };
        if !running || instance.is_empty() || crate::api::joinable(&instance) || self.leaving.swap(true, Ordering::SeqCst) {
            return;
        }
        let kind = crate::api::instance_kind(&instance).unwrap_or("unknown");
        tracing::error!("in a {kind} instance ({instance}): leaving");
        self.follower.stop();
        self.send_event(json!({"type": "alert", "reason": "not_joinable_instance", "kind": kind}));
        let me = self.clone();
        tokio::spawn(async move {
            let _ = game::stop_game().await;
            me.leaving.store(false, Ordering::SeqCst);
        });
    }

    pub fn require_game(&self) -> anyhow::Result<()> {
        if !self.game.lock().unwrap().running {
            anyhow::bail!("VRChat is not running");
        }
        Ok(())
    }

    /// An OSC handle that can also query OSCQuery (on the port the log names).
    pub fn osc_query(&self) -> anyhow::Result<Osc> {
        let port = self.game.lock().unwrap().oscquery_port;
        if port == 0 {
            anyhow::bail!("the game is not running");
        }
        Osc::with_ports(&format!("127.0.0.1:{}", self.args.osc_port), port)
    }

    // -- the stream client ------------------------------------------------------

    /// A new stream client: the old one is dropped; its outbound queue.
    pub fn attach_client(&self) -> (u64, mpsc::Receiver<Message>) {
        let id = self.client_ids.fetch_add(1, Ordering::SeqCst) + 1;
        let (tx, rx) = mpsc::channel(CLIENT_QUEUE);
        let _ = tx.try_send(Message::Text(json!({"type": "state", "state": self.room_state()}).to_string().into()));
        if let Some((_, old)) = self.client.lock().unwrap().replace((id, tx)) {
            let _ = old.try_send(Message::Close(None));
        }
        (id, rx)
    }

    pub fn detach_client(&self, id: u64) {
        let mut c = self.client.lock().unwrap();
        if c.as_ref().is_some_and(|(cid, _)| *cid == id) {
            *c = None;
        }
    }

    // -- audio ------------------------------------------------------------------

    /// Reads what the game plays and hands it to the client, if any.
    pub async fn capture(self: Arc<Self>) {
        loop {
            let child = Command::new("parec")
                .args([
                    &format!("--device={}", self.args.capture_device),
                    "--format=s16le",
                    &format!("--rate={SAMPLE_RATE}"),
                    "--channels=1",
                    "--latency-msec=20",
                    "--raw",
                ])
                .stdout(std::process::Stdio::piped())
                .kill_on_drop(true)
                .spawn();
            match child {
                Ok(mut child) => {
                    tracing::info!("capturing {}", self.args.capture_device);
                    let mut out = child.stdout.take().unwrap();
                    let mut buf = vec![0u8; FRAME_BYTES];
                    while out.read_exact(&mut buf).await.is_ok() {
                        if let Some((_, tx)) = self.client.lock().unwrap().as_ref() {
                            let _ = tx.try_send(Message::Binary(buf.clone().into())); // a full queue drops it
                        }
                    }
                    let _ = child.kill().await;
                }
                Err(e) => tracing::warn!("capture failed: {e}"),
            }
            tracing::warn!("capture ended, restarting");
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }

    /// Plays the bot's voice into the microphone, holding push-to-talk.
    pub async fn play(self: &Arc<Self>, pcm: &[u8]) {
        let mut p = self.player.lock().await;
        let dead = match &mut p.child {
            Some(c) => c.try_wait().map(|s| s.is_some()).unwrap_or(true),
            None => true,
        };
        if dead {
            p.child = Command::new("pacat")
                .args([
                    "--playback",
                    &format!("--device={}", self.args.playback_device),
                    "--format=s16le",
                    &format!("--rate={SAMPLE_RATE}"),
                    "--channels=1",
                    &format!("--latency-msec={PLAYBACK_LATENCY_MS}"),
                    "--raw",
                ])
                .stdin(std::process::Stdio::piped())
                .kill_on_drop(true)
                .spawn()
                .map_err(|e| tracing::warn!("playback failed: {e}"))
                .ok();
        }
        if !p.ptt_held {
            let _ = self.osc.send_i32("/input/Voice", 1);
            p.ptt_held = true;
        }
        let now = Instant::now();
        let len = Duration::from_secs_f64(pcm.len() as f64 / (SAMPLE_RATE as f64 * 2.0));
        let starts = p.speech_until.filter(|t| *t > now).unwrap_or(now);
        self.anim.heard_bot(pcm, starts + Duration::from_millis(PLAYBACK_LATENCY_MS));
        p.speech_until = Some(p.speech_until.filter(|t| *t > now).unwrap_or(now) + len);
        if !p.release_pending {
            p.release_pending = true;
            let me = self.clone();
            tokio::spawn(async move { me.release_ptt().await });
        }
        if let Some(stdin) = p.child.as_mut().and_then(|c| c.stdin.as_mut()) {
            if stdin.write_all(pcm).await.is_err() {
                p.child = None;
            }
        }
    }

    async fn release_ptt(self: Arc<Self>) {
        let tail = PTT_TAIL + Duration::from_millis(PLAYBACK_LATENCY_MS);
        loop {
            let until = self.player.lock().await.speech_until;
            let wait = until.map(|u| (u + tail).saturating_duration_since(Instant::now())).unwrap_or_default();
            if wait.is_zero() {
                break;
            }
            tokio::time::sleep(wait).await;
        }
        let mut p = self.player.lock().await;
        let _ = self.osc.send_i32("/input/Voice", 0);
        p.ptt_held = false;
        p.release_pending = false;
    }

    pub async fn release_voice(&self) {
        let mut p = self.player.lock().await;
        if p.ptt_held {
            let _ = self.osc.send_i32("/input/Voice", 0);
            p.ptt_held = false;
        }
    }

    // -- chatbox ------------------------------------------------------------------

    /// Queues `text` for the chatbox, split into 144-character parts.
    pub fn chatbox(&self, text: &str, notify: bool) -> usize {
        let parts = split_chatbox(text);
        for part in &parts {
            let _ = self.chatbox.try_send((part.clone(), notify)); // a full queue drops it
        }
        parts.len()
    }

    pub async fn chatbox_sender(self: Arc<Self>, mut rx: mpsc::Receiver<(String, bool)>) {
        while let Some((text, notify)) = rx.recv().await {
            let _ = self.osc.send("/chatbox/input", &[Arg::Str(text), Arg::Bool(true), Arg::Bool(notify)]);
            tokio::time::sleep(CHATBOX_INTERVAL).await;
        }
    }

    // -- the game -----------------------------------------------------------------

    /// Follows the client's output log.
    pub async fn log_tail(self: Arc<Self>) {
        let mut tail = LogTail::new(game::expand(&self.args.log_dir));
        loop {
            let changed = {
                let mut g = self.game.lock().unwrap();
                tail.poll(&mut g).unwrap_or_else(|e| {
                    tracing::warn!("log tail failed: {e}");
                    false
                })
            };
            if changed {
                self.state_changed();
            }
            tokio::time::sleep(Duration::from_secs(1)).await;
        }
    }

    /// Tracks whether the game runs and its VRAM; stops it above the hard limit.
    pub async fn watchdog(self: Arc<Self>) {
        let mut first = true;
        loop {
            if !first {
                tokio::time::sleep(WATCHDOG_INTERVAL).await;
            }
            first = false;
            let pid = game::game_pid().await;
            let running = pid.is_some();
            let (vram, total) = match pid {
                Some(pid) => game::vram(pid).await,
                None => (0, 0),
            };
            let changed = {
                let mut g = self.game.lock().unwrap();
                let mut changed = running != g.running;
                g.running = running;
                g.vram_mib = vram;
                if !running && (!g.instance.is_empty() || !g.players.is_empty() || !g.world_name.is_empty()) {
                    // No game, no instance: not even one an old log replayed.
                    g.world_name.clear();
                    g.instance.clear();
                    g.players.clear();
                    changed = true;
                }
                changed
            };
            if changed && !running {
                self.follower.stop(); // nothing to read or steer
            }
            if changed {
                self.state_changed();
            }
            let a = &self.args;
            if running && (vram > a.vram_hard_mib || total > a.gpu_hard_mib) {
                tracing::error!("VRAM over the limit (game {vram} MiB, GPU {total} MiB): stopping the game");
                self.send_event(json!({"type": "alert", "reason": "vram", "game_mib": vram, "gpu_mib": total, "action": "stopped"}));
                let _ = game::stop_game().await;
            } else if running && vram > a.vram_soft_mib {
                self.send_event(json!({"type": "alert", "reason": "vram", "game_mib": vram, "gpu_mib": total, "action": "none"}));
            }
        }
    }
}

/// Splits `text` into chatbox messages of at most 144 characters, preferring
/// sentence ends, then spaces.
pub fn split_chatbox(text: &str) -> Vec<String> {
    let mut text: Vec<char> = text.split_whitespace().collect::<Vec<_>>().join(" ").chars().collect();
    let mut parts = Vec::new();
    while text.len() > CHATBOX_LIMIT {
        let window = &text[..CHATBOX_LIMIT];
        let cut = window.iter().rposition(|c| "。！？!?；;，,. ".contains(*c));
        let cut = match cut {
            Some(c) if c >= CHATBOX_LIMIT / 2 => c + 1,
            _ => CHATBOX_LIMIT,
        };
        parts.push(text[..cut].iter().collect::<String>().trim().to_string());
        text = text[cut..].iter().collect::<String>().trim().chars().collect();
    }
    if !text.is_empty() {
        parts.push(text.into_iter().collect());
    }
    parts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chatbox_parts() {
        assert_eq!(split_chatbox("  hello   world "), vec!["hello world".to_string()]);
        let long = "这是一句话。".repeat(40);
        let parts = split_chatbox(&long);
        assert!(parts.iter().all(|p| p.chars().count() <= CHATBOX_LIMIT));
        assert!(parts[0].ends_with('。'));
        assert_eq!(parts.concat(), long);
    }
}
