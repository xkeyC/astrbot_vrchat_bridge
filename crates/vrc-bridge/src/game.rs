//! The game: its state from the client's output log (world, instance,
//! players, the OSCQuery port), whether it runs and how much VRAM it takes,
//! and starting / stopping it.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{bail, Result};
use serde_json::{json, Value};
use tokio::process::Command;

pub const STEAM_APP_ID: &str = "438100";

#[derive(Clone, Debug, Default)]
pub struct GameState {
    pub running: bool,
    pub self_name: String,
    pub self_id: String,
    pub world_name: String,
    /// wrld_...:id~...
    pub instance: String,
    /// usr id -> display name
    pub players: BTreeMap<String, String>,
    pub oscquery_port: u16,
    pub vram_mib: u64,
}

impl GameState {
    pub fn snapshot(&self) -> Value {
        let others: Vec<Value> = self
            .players
            .iter()
            .filter(|(id, _)| **id != self.self_id)
            .map(|(id, name)| json!({"id": id, "name": name}))
            .collect();
        json!({
            "running": self.running,
            "self": {"name": self.self_name, "id": self.self_id},
            "world": self.world_name,
            "instance": self.instance,
            "players": others,
            "vram_mib": self.vram_mib,
        })
    }

    /// Display names of the other players.
    pub fn others(&self) -> Vec<(String, String)> {
        self.players
            .iter()
            .filter(|(id, _)| **id != self.self_id)
            .map(|(id, n)| (id.clone(), n.clone()))
            .collect()
    }

    /// Applies one log line; whether the room state changed.
    pub fn apply(&mut self, line: &str) -> bool {
        if let Some((name, id)) = player_event(line, "[Behaviour] OnPlayerJoined ") {
            self.players.insert(id, name);
        } else if let Some((_, id)) = player_event(line, "[Behaviour] OnPlayerLeft ") {
            self.players.remove(&id);
        } else if let Some(at) = line.find("[Behaviour] Entering Room: ") {
            self.world_name = line[at + 27..].trim().to_string();
            self.players.clear();
        } else if let Some(at) = line.find("[Behaviour] Joining wrld_") {
            let rest = &line[at + 20..];
            self.instance = rest.split_whitespace().next().unwrap_or("").to_string();
        } else if line.contains("[Behaviour] OnLeftRoom") {
            self.world_name.clear();
            self.instance.clear();
            self.players.clear();
        } else if let Some((name, id)) = player_event(line, "User Authenticated: ") {
            self.self_name = name;
            self.self_id = id;
        } else if let Some(at) = line.find("of type OSCQuery on ") {
            let digits: String = line[at + 20..].chars().take_while(char::is_ascii_digit).collect();
            self.oscquery_port = digits.parse().unwrap_or(0);
            return false;
        } else {
            return false;
        }
        true
    }
}

/// `<marker><name> (usr_...)` in a line: (name, id).
fn player_event(line: &str, marker: &str) -> Option<(String, String)> {
    let at = line.find(marker)? + marker.len();
    let rest = line[at..].trim_end();
    let open = rest.rfind(" (usr_")?;
    let id = rest[open + 2..].strip_suffix(')')?;
    if !id.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
        return None;
    }
    Some((rest[..open].to_string(), id.to_string()))
}

/// Follows the newest VRChat output log.
pub struct LogTail {
    pub dir: PathBuf,
    path: Option<PathBuf>,
    pos: u64,
    partial: String,
}

impl LogTail {
    pub fn new(dir: PathBuf) -> LogTail {
        LogTail { dir, path: None, pos: 0, partial: String::new() }
    }

    fn newest(&self) -> Option<PathBuf> {
        std::fs::read_dir(&self.dir)
            .ok()?
            .filter_map(|e| e.ok())
            .filter(|e| e.file_name().to_string_lossy().starts_with("output_log_"))
            .max_by_key(|e| e.metadata().and_then(|m| m.modified()).ok())
            .map(|e| e.path())
    }

    /// Reads what was added; whether the room state changed.
    pub fn poll(&mut self, state: &mut GameState) -> Result<bool> {
        use std::io::{Read, Seek, SeekFrom};
        let newest = self.newest();
        let mut changed = false;
        if newest != self.path {
            // A new client run: its log starts from scratch.
            self.path = newest;
            self.pos = 0;
            self.partial.clear();
            state.world_name.clear();
            state.instance.clear();
            state.players.clear();
            state.oscquery_port = 0;
            changed = true;
        }
        let Some(path) = &self.path else { return Ok(changed) };
        let mut f = std::fs::File::open(path)?;
        f.seek(SeekFrom::Start(self.pos))?;
        let mut buf = Vec::new();
        f.read_to_end(&mut buf)?;
        self.pos += buf.len() as u64;
        if buf.is_empty() {
            return Ok(changed);
        }
        let text = self.partial.clone() + &String::from_utf8_lossy(&buf);
        let mut lines: Vec<&str> = text.split('\n').collect();
        self.partial = lines.pop().unwrap_or("").to_string();
        for line in lines {
            changed |= state.apply(line);
        }
        Ok(changed)
    }
}

/// The game's process id (by name: Proton's wrappers carry VRChat.exe in
/// their arguments too).
pub async fn game_pid() -> Option<u32> {
    let uid = unsafe_uid();
    let out = Command::new("pgrep").args(["-u", &uid, "-x", "VRChat.exe"]).output().await.ok()?;
    String::from_utf8_lossy(&out.stdout).split_whitespace().filter_map(|p| p.parse().ok()).min()
}

fn unsafe_uid() -> String {
    std::env::var("UID").ok().or_else(|| {
        std::process::Command::new("id").arg("-u").output().ok().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    })
    .unwrap_or_else(|| "0".into())
}

/// (the game's VRAM, the whole GPU's), MiB.
pub async fn vram(pid: u32) -> (u64, u64) {
    let mut game = 0;
    if let Ok(out) = Command::new("nvidia-smi")
        .args(["--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"])
        .output()
        .await
    {
        for line in String::from_utf8_lossy(&out.stdout).lines() {
            let parts: Vec<&str> = line.split(',').map(str::trim).collect();
            if parts.len() == 2 && parts[0] == pid.to_string() {
                game = parts[1].parse().unwrap_or(0);
            }
        }
    }
    let total = Command::new("nvidia-smi")
        .args(["--query-gpu=memory.used", "--format=csv,noheader,nounits"])
        .output()
        .await
        .ok()
        .and_then(|o| String::from_utf8_lossy(&o.stdout).split_whitespace().next().and_then(|v| v.parse().ok()))
        .unwrap_or(0);
    (game, total)
}

async fn run(cmd: &str, args: &[&str]) -> Result<bool> {
    Ok(Command::new(cmd).args(args).status().await?.success())
}

/// Starts Steam (if needed) and VRChat; a `vrchat://launch` URL picks the
/// instance (checked by the caller).
pub async fn start_game(display: &str, url: &str) -> Result<()> {
    if game_pid().await.is_some() {
        bail!("the game is already running");
    }
    let xauth = format!("--setenv=XAUTHORITY={}", home().join(".Xauthority").display());
    let disp = format!("--setenv=DISPLAY={display}");
    if !run("pgrep", &["-u", &unsafe_uid(), "-x", "steam"]).await.unwrap_or(false) {
        let _ = run("systemctl", &["--user", "reset-failed", "vrc-steam"]).await;
        run("systemd-run", &["--user", "--unit=vrc-steam", &disp, &xauth, "/usr/bin/steam", "-silent"]).await?;
        tokio::time::sleep(Duration::from_secs(25)).await; // login and IPC take a while
    }
    let mut args = vec!["--user", "--quiet", "--collect", &disp, &xauth, "/usr/bin/steam", "-applaunch", STEAM_APP_ID];
    if !url.is_empty() {
        args.push(url);
    }
    run("systemd-run", &args).await?;
    Ok(())
}

pub async fn stop_game() -> Result<()> {
    let _ = run("pkill", &["-TERM", "-u", &unsafe_uid(), "-f", "VRChat.exe"]).await;
    for _ in 0..60 {
        if game_pid().await.is_none() {
            break;
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
    Ok(())
}

pub fn home() -> PathBuf {
    PathBuf::from(std::env::var("HOME").unwrap_or_default())
}

/// `~/...` expanded.
pub fn expand(path: &str) -> PathBuf {
    match path.strip_prefix("~/") {
        Some(rest) => home().join(rest),
        None => Path::new(path).to_path_buf(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn follows_the_log() {
        let mut s = GameState::default();
        let lines = [
            "2026.10.05 19:55:01 Log - User Authenticated: Mon3tr (usr_aaa-1)",
            "2026.10.05 19:55:10 Log - [Behaviour] Joining wrld_4432ea9b-729c:12345~friends(usr_x) ",
            "2026.10.05 19:55:10 Log - [Behaviour] Entering Room: VRChat Home",
            "2026.10.05 19:55:11 Log - [Behaviour] OnPlayerJoined Mon3tr (usr_aaa-1)",
            "2026.10.05 19:55:12 Log - [Behaviour] OnPlayerJoined x key (C) (usr_bbb)",
            "2026.10.05 19:55:13 Log - [Behaviour] OnPlayerJoined Bob (usr_ccc)",
            "2026.10.05 19:55:14 Log - [Behaviour] OnPlayerLeft Bob (usr_ccc)",
            "2026.10.05 19:55:15 Log - OSC Service of type OSCQuery on 46737",
        ];
        for l in lines {
            s.apply(l);
        }
        assert_eq!(s.self_name, "Mon3tr");
        assert_eq!(s.world_name, "VRChat Home");
        assert_eq!(s.instance, "wrld_4432ea9b-729c:12345~friends(usr_x)");
        assert_eq!(s.others(), vec![("usr_bbb".to_string(), "x key (C)".to_string())]);
        assert_eq!(s.oscquery_port, 46737);
        s.apply("x [Behaviour] OnLeftRoom");
        assert!(s.players.is_empty() && s.instance.is_empty());
    }
}
