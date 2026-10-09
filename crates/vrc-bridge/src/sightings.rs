//! When each whitelisted friend was last seen, and the view then: for "when
//! did you last see X" and "show me". While nobody is followed, a frame of
//! the eyes is read every WATCH_INTERVAL when a whitelisted friend is in the
//! room; following records sightings itself.
//!
//! With the panorama (D36): the names come from the user camera's lens and
//! the plates over the eyes, the places from the panorama's depth
//! (`panolook`); the view kept is the panorama's, looking their way (a
//! name with no one found under it: that way still).

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde_json::{json, Value};
use vrc_players::names::{match_score, MATCH_RATIO};
use vrc_players::OcrClient;

use crate::bridge::Bridge;
use crate::Lock;

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
        self.last.lk().insert(name.to_string(), Sighting { at: now(), world: world.to_string(), jpeg });
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
        let last = self.last.lk();
        if name.is_empty() {
            return last.iter().max_by(|a, b| a.1.at.total_cmp(&b.1.at)).map(|(n, s)| (n.clone(), s.clone()));
        }
        let (best, s) = last.iter().max_by(|a, b| match_score(a.0, name).total_cmp(&match_score(b.0, name)))?;
        (match_score(best, name) >= MATCH_RATIO).then(|| (best.clone(), s.clone()))
    }

    /// One round of the watch with the panorama: the friends `here` named
    /// in it (placed, or by their plate's way alone), each with the view
    /// their way.
    fn watch_pano(&self, b: &Bridge, here: &[String], world: &str) -> anyhow::Result<()> {
        let room = crate::panolook::room(b);
        let ocr = OcrClient::new(&b.args.ocr_url, &b.args.ocr_model).ok();
        let o = crate::panolook::LookOptions { lens_within: WATCH_INTERVAL, ..Default::default() };
        let l = crate::panolook::look(b, &room, ocr.as_ref(), &o)?;
        let head = l.frame.head.position;
        let ways = l.people.iter().filter_map(|q| Some((q.name.clone()?, q.body.yaw_from(head)))).chain(l.bearings.iter().cloned());
        for (name, yaw) in ways {
            if here.contains(&name) {
                self.saw(&name, world, crate::vr::view_jpeg(&l.frame, yaw, 0.0, 100.0, 640)?);
            }
        }
        l.to_speakers(b, &b.social.whitelist_names());
        Ok(())
    }

    /// Watches for whitelisted friends while nobody is followed.
    pub async fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        loop {
            tokio::time::sleep(WATCH_INTERVAL).await;
            let (running, room, world) = {
                let g = bridge.game.lk();
                (g.running, g.others().into_iter().map(|(_, n)| n).collect::<Vec<_>>(), g.world_name.clone())
            };
            let whitelist = bridge.social.whitelist_names();
            let here: Vec<String> = whitelist.iter().filter(|w| room.contains(w)).cloned().collect();
            if !running || !bridge.follower.is_idle() || here.is_empty() {
                continue;
            }
            let (me, b) = (self.clone(), bridge.clone());
            let result = tokio::task::spawn_blocking(move || -> anyhow::Result<()> {
                // The panorama only (decision D42): without it, nothing seen.
                if !b.pano.usable() {
                    return Ok(());
                }
                me.watch_pano(&b, &here, &world)
            })
            .await;
            if let Ok(Err(e)) = result {
                tracing::debug!("watch failed: {e:#}");
            }
        }
    }
}
