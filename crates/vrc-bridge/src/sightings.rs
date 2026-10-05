//! When each whitelisted friend was last seen, and the view then: for "when
//! did you last see X" and "show me". While nobody is followed, a frame of
//! the eyes is read every WATCH_INTERVAL when a whitelisted friend is in the
//! room; following records sightings itself.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde_json::{json, Value};
use vrc_players::names::{match_score, MATCH_RATIO};

use crate::bridge::Bridge;

const WATCH_INTERVAL: Duration = Duration::from_secs(10);

#[derive(Clone)]
pub struct Sighting {
    pub at: f64,
    pub world: String,
    pub jpeg: Vec<u8>,
}

#[derive(Default)]
pub struct Sightings {
    last: Mutex<BTreeMap<String, Sighting>>,
}

fn now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

impl Sightings {
    pub fn saw(&self, name: &str, world: &str, jpeg: Vec<u8>) {
        self.last.lock().unwrap().insert(name.to_string(), Sighting { at: now(), world: world.to_string(), jpeg });
    }

    pub fn listing(&self) -> Vec<Value> {
        let t = now();
        let mut out: Vec<(f64, Value)> = self
            .last
            .lock()
            .unwrap()
            .iter()
            .map(|(n, s)| (s.at, json!({"name": n, "at": s.at, "age_s": ((t - s.at) * 10.0).round() / 10.0, "world": s.world})))
            .collect();
        out.sort_by(|a, b| b.0.total_cmp(&a.0));
        out.into_iter().map(|(_, v)| v).collect()
    }

    /// The newest sighting of `name` (best match), or of anyone.
    pub fn latest(&self, name: &str) -> Option<(String, Sighting)> {
        let last = self.last.lock().unwrap();
        if name.is_empty() {
            return last.iter().max_by(|a, b| a.1.at.total_cmp(&b.1.at)).map(|(n, s)| (n.clone(), s.clone()));
        }
        let (best, s) = last.iter().max_by(|a, b| match_score(a.0, name).total_cmp(&match_score(b.0, name)))?;
        (match_score(best, name) >= MATCH_RATIO).then(|| (best.clone(), s.clone()))
    }

    /// Watches for whitelisted friends while nobody is followed.
    pub async fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        loop {
            tokio::time::sleep(WATCH_INTERVAL).await;
            let (running, room, world) = {
                let g = bridge.game.lock().unwrap();
                (g.running, g.others().into_iter().map(|(_, n)| n).collect::<Vec<_>>(), g.world_name.clone())
            };
            let whitelist = bridge.social.whitelist_names();
            let here: Vec<String> = whitelist.iter().filter(|w| room.contains(w)).cloned().collect();
            if !running || !bridge.follower.is_idle() || here.is_empty() {
                continue;
            }
            let (me, b) = (self.clone(), bridge.clone());
            let result = tokio::task::spawn_blocking(move || -> anyhow::Result<()> {
                let Ok(mut vr) = b.vr.try_lock() else { return Ok(()) }; // busy moving: next round
                let frame = vr.frame()?;
                let rig = vr.rig(&whitelist)?;
                let Some(ocr) = rig.ocr.as_ref() else { return Ok(()) };
                let lines = ocr.lines_rgb(&frame.eye_rgb8(0)?, frame.width as u16, frame.height as u16)?;
                let mut jpeg = None;
                for name in &here {
                    if lines.iter().any(|l| match_score(&l.text, name) >= MATCH_RATIO) {
                        if jpeg.is_none() {
                            jpeg = Some(crate::vr::eye_jpeg(&frame, 640)?);
                        }
                        me.saw(name, &world, jpeg.clone().unwrap());
                    }
                }
                Ok(())
            })
            .await;
            if let Ok(Err(e)) = result {
                tracing::debug!("watch failed: {e:#}");
            }
        }
    }
}
