//! The players in the room, from VRChat's log: who joined and has not left
//! since the last room was entered, without the bot itself.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

/// Display names in the room, as of the newest log in `dir`.
pub fn players(dir: &Path) -> Result<Vec<String>> {
    let newest: PathBuf = std::fs::read_dir(dir)
        .with_context(|| format!("no VRChat logs in {}", dir.display()))?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_string_lossy().starts_with("output_log_"))
        .max_by_key(|e| e.metadata().and_then(|m| m.modified()).ok())
        .map(|e| e.path())
        .context("no VRChat log")?;
    Ok(from_log(&String::from_utf8_lossy(&std::fs::read(&newest)?)))
}

/// Display names in the room at the end of `log`.
pub fn from_log(log: &str) -> Vec<String> {
    let mut me = None;
    let mut here: Vec<String> = Vec::new();
    for line in log.lines() {
        if let Some(name) = between(line, "User Authenticated: ", " (usr_") {
            me = Some(name.to_string());
        } else if line.contains("[Behaviour] Entering Room: ") {
            here.clear();
        } else if let Some(name) = between(line, "[Behaviour] OnPlayerJoined ", " (usr_") {
            if !here.iter().any(|n| n == name) {
                here.push(name.to_string());
            }
        } else if let Some(name) = between(line, "[Behaviour] OnPlayerLeft ", " (usr_") {
            here.retain(|n| n != name);
        }
    }
    here.retain(|n| Some(n) != me.as_ref());
    here
}

fn between<'a>(line: &'a str, start: &str, end: &str) -> Option<&'a str> {
    let at = line.find(start)? + start.len();
    let len = line[at..].rfind(end)?;
    Some(&line[at..at + len])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn follows_joins_and_leaves() {
        let log = "\
2026.10.05 19:55:01 Log - User Authenticated: Mon3tr (usr_aaa)
2026.10.05 19:55:10 Log - [Behaviour] Entering Room: Old Room
2026.10.05 19:55:11 Log - [Behaviour] OnPlayerJoined Ghost (usr_ggg)
2026.10.05 19:56:00 Log - [Behaviour] Entering Room: VRChat Home
2026.10.05 19:56:01 Log - [Behaviour] OnPlayerJoined Mon3tr (usr_aaa)
2026.10.05 19:56:30 Log - [Behaviour] OnPlayerJoined x key (C) (usr_bbb)
2026.10.05 19:56:40 Log - [Behaviour] OnPlayerJoined Bob (usr_ccc)
2026.10.05 19:57:00 Log - [Behaviour] OnPlayerLeft Bob (usr_ccc)
";
        assert_eq!(from_log(log), vec!["x key (C)".to_string()]);
    }
}
