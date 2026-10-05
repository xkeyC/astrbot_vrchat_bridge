//! Whitelist, invites and following across instances (VRChat Web API).
//!
//! The whitelist is an ordered list of friends (display names or `usr_`
//! ids); its order is the priority. Invites from whitelisted friends are
//! accepted, and with following on the bot goes wherever the highest-priority
//! whitelisted friend in a joinable instance is. Public instances (and public
//! group instances) are never joined. Joining restarts the game into the
//! instance: a running client cannot be sent elsewhere.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};

use crate::api::{instance_kind, joinable, launch_url, redact, AuthRequired, VrcApi};
use crate::bridge::Bridge;
use crate::game;
use crate::Lock;

/// A friend's new location is acted on once it held this long.
const FOLLOW_SETTLE: Duration = Duration::from_secs(8);
/// Joins are at least this far apart (each one restarts the game).
const JOIN_COOLDOWN: Duration = Duration::from_secs(45);
const FRIENDS_REFRESH: Duration = Duration::from_secs(300);
const RETRY: Duration = Duration::from_secs(60);
/// Seconds a join waits for the game it started to run.
const GAME_START_WAIT: u64 = 90;

#[derive(Clone, Debug)]
pub struct Friend {
    pub name: String,
    pub location: String,
}

pub struct Social {
    config_file: PathBuf,
    pub api: tokio::sync::Mutex<VrcApi>,
    inner: Mutex<Inner>,
}

struct Inner {
    whitelist: Vec<String>,
    auto_accept: bool,
    follow: bool,
    logged_in: bool,
    me: Value,
    friends: BTreeMap<String, Friend>,
    /// The planned join: its location and a generation to cancel it.
    pending: Option<(String, u64)>,
    generation: u64,
    last_join: Option<Instant>,
}

fn same_instance(a: &str, b: &str) -> bool {
    !a.is_empty() && a.split('~').next() == b.split('~').next()
}

impl Social {
    pub fn new(config_file: PathBuf, cookie_file: PathBuf) -> Social {
        let mut inner = Inner {
            whitelist: Vec::new(),
            auto_accept: true,
            follow: false,
            logged_in: false,
            me: json!({}),
            friends: BTreeMap::new(),
            pending: None,
            generation: 0,
            last_join: None,
        };
        if let Ok(v) = std::fs::read_to_string(&config_file).map(|s| serde_json::from_str::<Value>(&s).unwrap_or_default()) {
            if let Some(w) = v["whitelist"].as_array() {
                inner.whitelist = w.iter().filter_map(|e| e.as_str().map(String::from)).collect();
            }
            inner.auto_accept = v["auto_accept"].as_bool().unwrap_or(true);
            inner.follow = v["follow"].as_bool().unwrap_or(false);
        }
        Social { config_file, api: tokio::sync::Mutex::new(VrcApi::new(cookie_file)), inner: Mutex::new(inner) }
    }

    // -- config / status ---------------------------------------------------------

    pub fn set_config(&self, bridge: &Arc<Bridge>, update: &Value) -> anyhow::Result<()> {
        {
            let mut s = self.inner.lk();
            if let Some(w) = update.get("whitelist") {
                let entries: Vec<String> = match w {
                    Value::String(text) => text.replace('，', ",").replace('\n', ",").split(',').map(String::from).collect(),
                    Value::Array(a) => a.iter().map(|e| e.as_str().map(String::from).unwrap_or_else(|| e.to_string())).collect(),
                    _ => Vec::new(),
                };
                s.whitelist = entries.into_iter().map(|e| e.trim().to_string()).filter(|e| !e.is_empty()).collect();
            }
            if let Some(b) = update.get("auto_accept").and_then(Value::as_bool) {
                s.auto_accept = b;
            }
            if let Some(b) = update.get("follow").and_then(Value::as_bool) {
                s.follow = b;
            }
            if let Some(dir) = self.config_file.parent() {
                std::fs::create_dir_all(dir)?;
            }
            std::fs::write(
                &self.config_file,
                json!({"whitelist": s.whitelist, "auto_accept": s.auto_accept, "follow": s.follow}).to_string(),
            )?;
        }
        self.evaluate(bridge);
        bridge.notify_state(); // whitelist ranks are room context
        Ok(())
    }

    /// The whitelist as user ids, in priority order (unknown names left out).
    pub fn whitelist_ids(&self) -> Vec<String> {
        let s = self.inner.lk();
        ids_of(&s)
    }

    /// The whitelist as display names, in priority order (ids resolved through
    /// the friend list).
    pub fn whitelist_names(&self) -> Vec<String> {
        let s = self.inner.lk();
        s.whitelist
            .iter()
            .map(|e| if e.starts_with("usr_") { s.friends.get(e).map(|f| f.name.clone()).unwrap_or_default() } else { e.clone() })
            .filter(|n| !n.is_empty())
            .collect()
    }

    pub fn friend_ids(&self) -> Vec<String> {
        self.inner.lk().friends.keys().cloned().collect()
    }

    pub fn friend_location(&self, id: &str) -> Option<String> {
        self.inner.lk().friends.get(id).map(|f| f.location.clone())
    }

    pub fn status(&self) -> Value {
        let s = self.inner.lk();
        let by_name: BTreeMap<&str, &str> = s.friends.iter().map(|(id, f)| (f.name.as_str(), id.as_str())).collect();
        let entries: Vec<Value> = s
            .whitelist
            .iter()
            .map(|entry| {
                let uid = if entry.starts_with("usr_") { entry.clone() } else { by_name.get(entry.as_str()).map(|s| s.to_string()).unwrap_or_default() };
                let friend = s.friends.get(&uid);
                let location = friend.map(|f| f.location.clone()).unwrap_or_default();
                json!({
                    "entry": entry, "id": uid, "name": friend.map(|f| f.name.clone()).unwrap_or_default(),
                    "friend": friend.is_some(), "location": location,
                    "kind": instance_kind(&location), "joinable": joinable(&location),
                })
            })
            .collect();
        let target = follow_target(&s);
        json!({
            "logged_in": s.logged_in,
            "me": {"id": s.me["id"].as_str().unwrap_or(""), "name": s.me["displayName"].as_str().unwrap_or("")},
            "auto_accept": s.auto_accept,
            "follow": s.follow,
            "follow_target": target.and_then(|t| s.friends.get(&t).map(|f| f.name.clone())).unwrap_or_default(),
            "whitelist": entries,
        })
    }

    // -- running -----------------------------------------------------------------

    pub async fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        let mut refresher: Option<tokio::task::JoinHandle<()>> = None;
        loop {
            let result: anyhow::Result<()> = async {
                {
                    let mut api = self.api.lock().await;
                    if !api.load() {
                        return Err(AuthRequired("not logged in".into()).into());
                    }
                }
                let me = self.api.lock().await.me().await?;
                {
                    let mut s = self.inner.lk();
                    if !s.logged_in {
                        tracing::info!("logged in as {}", me["displayName"].as_str().unwrap_or("?"));
                    }
                    s.logged_in = true;
                    s.me = me;
                }
                self.refresh(&bridge).await?;
                if refresher.as_ref().is_none_or(|h| h.is_finished()) {
                    let (me2, b2) = (self.clone(), bridge.clone());
                    refresher = Some(tokio::spawn(async move {
                        loop {
                            tokio::time::sleep(FRIENDS_REFRESH).await;
                            if let Err(e) = me2.refresh(&b2).await {
                                tracing::warn!("friends refresh failed: {}", redact(&e.to_string()));
                            }
                        }
                    }));
                }
                let auth = self.api.lock().await.auth()?;
                let (me2, b2) = (self.clone(), bridge.clone());
                crate::api::pipeline(&auth, move |kind, content| {
                    let (me3, b3) = (me2.clone(), b2.clone());
                    async move { me3.on_event(&b3, &kind, &content).await }
                })
                .await?;
                tracing::warn!("pipeline closed, reconnecting");
                tokio::time::sleep(Duration::from_secs(5)).await;
                Ok(())
            }
            .await;
            if let Err(e) = result {
                if let Some(auth) = e.downcast_ref::<AuthRequired>() {
                    let was = {
                        let s = self.inner.lk();
                        s.logged_in || s.me.get("id").is_none()
                    };
                    if was {
                        tracing::warn!("VRChat login required: {}", auth.0);
                        bridge.send_event(json!({"type": "auth_required", "reason": auth.0}));
                    }
                    self.inner.lk().logged_in = false;
                } else {
                    tracing::warn!("VRChat API failed: {}", redact(&format!("{e:#}")));
                }
                tokio::time::sleep(RETRY).await;
            }
        }
    }

    async fn refresh(&self, bridge: &Arc<Bridge>) -> anyhow::Result<()> {
        let friends = self.api.lock().await.friends().await?;
        {
            let mut s = self.inner.lk();
            s.friends = friends
                .iter()
                .filter_map(|f| {
                    Some((
                        f["id"].as_str()?.to_string(),
                        Friend {
                            name: f["displayName"].as_str().unwrap_or("").to_string(),
                            location: f["location"].as_str().unwrap_or("").to_string(),
                        },
                    ))
                })
                .collect();
        }
        self.evaluate(bridge);
        bridge.notify_state(); // who is a friend is room context
        Ok(())
    }

    async fn on_event(self: &Arc<Self>, bridge: &Arc<Bridge>, kind: &str, content: &Value) {
        if kind == "notification" && content["type"] == "invite" {
            let sender = content["senderUserId"].as_str().unwrap_or("").to_string();
            let location = content["details"]["worldId"].as_str().unwrap_or("").to_string();
            let name = content["senderUsername"].as_str().unwrap_or("").to_string();
            self.on_invite(bridge, &sender, &location, &name);
            return;
        }
        if kind == "notification" && content["type"] == "requestInvite" {
            let sender = content["senderUserId"].as_str().unwrap_or("").to_string();
            let name = content["senderUsername"].as_str().unwrap_or("").to_string();
            self.on_request_invite(bridge, &sender, &name).await;
            return;
        }
        let uid = content["userId"].as_str().unwrap_or("").to_string();
        match kind {
            "friend-location" | "friend-online" | "friend-active" | "friend-update" => {
                {
                    let mut s = self.inner.lk();
                    let f = s.friends.entry(uid).or_insert(Friend { name: String::new(), location: String::new() });
                    if let Some(n) = content["user"]["displayName"].as_str() {
                        f.name = n.to_string();
                    }
                    let mut location = content["location"].as_str().map(String::from);
                    if location.as_deref() == Some("traveling") {
                        location = content["travelingToLocation"].as_str().map(String::from).or(location);
                    }
                    if let Some(l) = location {
                        f.location = l;
                    }
                }
                self.evaluate(bridge);
            }
            "friend-offline" => {
                if let Some(f) = self.inner.lk().friends.get_mut(&uid) {
                    f.location = "offline".into();
                }
            }
            "friend-add" => {
                let _ = self.refresh(bridge).await;
            }
            _ => {}
        }
    }

    fn on_invite(self: &Arc<Self>, bridge: &Arc<Bridge>, sender: &str, location: &str, sender_name: &str) {
        let verdict = {
            let s = self.inner.lk();
            let ids = ids_of(&s);
            if !ids.iter().any(|i| i == sender) {
                "ignored: not whitelisted".to_string()
            } else if !s.auto_accept {
                "ignored: auto-accept is off".to_string()
            } else if !joinable(location) {
                format!("refused: {} instance", instance_kind(location).unwrap_or("unknown"))
            } else {
                let target = if s.follow { follow_target(&s) } else { None };
                let pos = |id: &str| ids.iter().position(|i| i == id).unwrap_or(usize::MAX);
                match target {
                    Some(t) if pos(&t) < pos(sender) && !same_instance(&s.friends[&t].location, location) => {
                        "ignored: following a higher-priority friend".to_string()
                    }
                    _ => "accepted".to_string(),
                }
            }
        };
        if verdict == "accepted" {
            self.schedule(bridge, location.to_string(), format!("invite from {sender_name}"), Duration::ZERO);
        }
        tracing::info!("invite from {sender_name} to {location}: {verdict}");
        bridge.send_event(json!({"type": "invite", "from": sender_name, "location": location, "verdict": verdict}));
    }

    /// A whitelisted friend asks to join: invite them to the bot's instance
    /// (as VRCX's auto-accept of invite requests does).
    async fn on_request_invite(&self, bridge: &Arc<Bridge>, sender: &str, sender_name: &str) {
        let (here, running) = {
            let g = bridge.game.lk();
            (g.instance.clone(), g.running)
        };
        let (listed, auto) = {
            let s = self.inner.lk();
            (ids_of(&s).iter().any(|i| i == sender), s.auto_accept)
        };
        let verdict = if !listed {
            "ignored: not whitelisted".to_string()
        } else if !auto {
            "ignored: auto-accept is off".to_string()
        } else if !running || !joinable(&here) {
            "ignored: the bot is in no joinable instance".to_string()
        } else {
            match self.api.lock().await.invite(sender, &here).await {
                Ok(_) => "invited".to_string(),
                Err(e) => format!("invite failed: {e}"),
            }
        };
        tracing::info!("invite request from {sender_name}: {verdict}");
        bridge.send_event(json!({"type": "request_invite", "from": sender_name, "verdict": verdict}));
    }

    /// Follows the follow target if it is elsewhere.
    fn evaluate(self: &Social, bridge: &Arc<Bridge>) {
        let plan = {
            let s = self.inner.lk();
            if !s.follow {
                return;
            }
            let Some(target) = follow_target(&s) else { return };
            let f = &s.friends[&target];
            (f.location.clone(), f.name.clone())
        };
        let here = bridge.game.lk().instance.clone();
        if same_instance(&plan.0, &here) {
            let mut s = self.inner.lk();
            s.pending = None;
            s.generation += 1;
            return;
        }
        // Scheduling needs an Arc of self: through the bridge.
        let social = bridge.social.clone();
        social.schedule(bridge, plan.0, format!("following {}", plan.1), FOLLOW_SETTLE);
    }

    fn schedule(self: &Arc<Self>, bridge: &Arc<Bridge>, location: String, reason: String, settle: Duration) {
        let generation = {
            let mut s = self.inner.lk();
            if s.pending.as_ref().is_some_and(|(l, _)| *l == location) {
                return; // already planned
            }
            s.generation += 1;
            s.pending = Some((location.clone(), s.generation));
            s.generation
        };
        let (me, b) = (self.clone(), bridge.clone());
        tokio::spawn(async move { me.join(b, location, reason, settle, generation).await });
    }

    async fn join(self: Arc<Self>, bridge: Arc<Bridge>, location: String, reason: String, settle: Duration, generation: u64) {
        let cooldown = {
            let s = self.inner.lk();
            s.last_join.map(|t| (t + JOIN_COOLDOWN).saturating_duration_since(Instant::now())).unwrap_or_default()
        };
        tokio::time::sleep(settle.max(cooldown)).await;
        {
            let mut s = self.inner.lk();
            if s.generation != generation {
                return; // replaced or cancelled
            }
            s.pending = None;
        }
        let here = bridge.game.lk().instance.clone();
        if same_instance(&location, &here) || !joinable(&location) {
            return;
        }
        self.inner.lk().last_join = Some(Instant::now());
        tracing::info!("joining {location} ({reason})");
        bridge.send_event(json!({"type": "joining", "location": location, "reason": reason}));
        // The restart finishes once begun, whatever is planned meanwhile.
        let _guard = bridge.game_lock.lock().await;
        let result = async {
            game::stop_game().await?;
            game::start_game(&bridge.args.display, &launch_url(&location)?).await?;
            for _ in 0..GAME_START_WAIT {
                if game::game_pid().await.is_some() {
                    break;
                }
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
            anyhow::Ok(())
        }
        .await;
        if let Err(e) = result {
            tracing::error!("join failed: {e:#}");
            bridge.send_event(json!({"type": "join_failed", "location": location, "error": e.to_string()}));
        }
    }
}

fn ids_of(s: &Inner) -> Vec<String> {
    let by_name: BTreeMap<&str, &str> = s.friends.iter().map(|(id, f)| (f.name.as_str(), id.as_str())).collect();
    let mut ids: Vec<String> = Vec::new();
    for e in &s.whitelist {
        let uid = if e.starts_with("usr_") { Some(e.clone()) } else { by_name.get(e.as_str()).map(|s| s.to_string()) };
        if let Some(u) = uid {
            if !ids.contains(&u) {
                ids.push(u);
            }
        }
    }
    ids
}

/// The highest-priority whitelisted friend in a joinable instance.
fn follow_target(s: &Inner) -> Option<String> {
    ids_of(s).into_iter().find(|id| s.friends.get(id).is_some_and(|f| joinable(&f.location)))
}
