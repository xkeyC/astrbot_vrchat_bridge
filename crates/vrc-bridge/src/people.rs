//! The people near the bot (decision D39): who, where (world and tracking),
//! which way and how far from the head, when last placed and named, and
//! how. Every look at the panorama (`panolook::look`) updates it: the
//! names read (the lens, the plates over the eyes) placed by the depth
//! under their plates, and those not named this look followed by
//! continuity (the person-shaped thing nearest where they were, within a
//! gate that grows as they may walk). Entries are kept KEEP_FOR.
//!
//! **The idle sweep**: while the bot is idle (not following, no follow
//! paused for a move, not moving, nobody speaking, no menu holding the
//! usual view, the panorama and the camera on), the lens goes once round
//! the head every `orbit.idle_sweep_s` (60 s; 0: never), the quick sweep
//! (`orbit.snap`: six views, each as soon as the one before was taken,
//! decision D41), reading names all round; then a look at the
//! panorama places them (their lens rays through the depth) and the
//! speaker tracker gets them placed. A move or a voice ends the sweep at
//! once (the orbit's move gate and its turn to a voice); the lens goes
//! back to the front after.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_pano::Body;

use crate::bridge::Bridge;
use crate::panolook::{self, LookOptions};
use crate::Lock;

/// Entries not placed this long are gone.
const KEEP_FOR: Duration = Duration::from_secs(150);
/// Someone not named in a look is followed by position while their last
/// place is this recent: the nearest person-shaped thing within
/// TRACK_GATE_M plus TRACK_SPEED m/s since (at most TRACK_GATE_MAX_M),
/// not where someone named stands.
const TRACK_FOR: Duration = Duration::from_secs(8);
/// ... and while a name said it was them this recent: kept fixes renewed
/// the place for ever, and a sign board stood "xkeyC" for two minutes
/// (decision D40).
const TRACK_UNNAMED_FOR: Duration = Duration::from_secs(15);
const TRACK_GATE_M: f32 = 0.6;
const TRACK_SPEED: f32 = 1.5;
const TRACK_GATE_MAX_M: f32 = 3.0;
const SAME_BODY_M: f32 = 0.6;
/// The idle sweep's scheduler looks this often; the sweep's last reads
/// (OCR) are waited for this long at most.
const READS_WAIT: Duration = Duration::from_millis(1500);
const TICK: Duration = Duration::from_secs(1);
/// Moved this recently: not idle.
const STILL_FOR: Duration = Duration::from_secs(3);

/// Someone near the bot.
#[derive(Clone, Debug)]
pub struct Near {
    pub name: String,
    /// Where they stand (world: Unity's), and in the tracking space as of
    /// that look.
    pub feet: [f32; 3],
    pub tracking: [f32; 3],
    /// From the head then: the world yaw, off the head's heading (+ right),
    /// the distance (metres).
    pub world_yaw: f32,
    pub bearing_deg: f32,
    pub distance_m: f32,
    /// Their body (the plate floats over it).
    pub body: Body,
    /// When last placed, and when a name last said it was them.
    pub seen: Instant,
    pub named: Instant,
    /// How the last place came: "overlay", "lens", "asked", "sweep" (named),
    /// "kept" (by continuity).
    pub source: &'static str,
}

/// One look's people as the cache takes them: name (if read), body, how
/// the name came.
pub struct Seen<'a> {
    pub name: Option<&'a str>,
    pub body: Body,
    pub how: Option<&'static str>,
}

/// Where the head was for a look: the eyes (world), its world yaw, and
/// world to tracking.
pub struct HeadThen<'a> {
    pub eye: [f32; 3],
    pub yaw: f32,
    pub to_tracking: &'a dyn Fn([f32; 3]) -> [f32; 3],
}

#[derive(Default)]
struct Sweeps {
    last: Option<Instant>,
    count: u64,
    /// How the last ended ("done", "move", "voice", ...), and how many names
    /// it read.
    last_end: Option<&'static str>,
    last_names: usize,
    /// Why the one due did not go yet.
    skipped: Option<String>,
}

#[derive(Default)]
pub struct People {
    near: Mutex<Vec<Near>>,
    sweeps: Mutex<Sweeps>,
}

impl People {
    /// A look's people: the named placed (and named now), the rest of the
    /// cache followed by continuity. Answers those kept by continuity.
    pub fn update(&self, seen: &[Seen], head: &HeadThen, override_how: Option<&'static str>, now: Instant) -> Vec<Near> {
        let mut near = self.near.lk();
        update(&mut near, seen, head, override_how, now)
    }

    /// Everyone near, latest first.
    pub fn list(&self) -> Vec<Near> {
        let mut v = self.near.lk().clone();
        v.sort_by(|a, b| b.seen.cmp(&a.seen));
        v
    }

    pub fn status(&self, every_s: f32) -> Value {
        let now = Instant::now();
        let s1 = |d: Duration| (d.as_secs_f64() * 10.0).round() / 10.0;
        let r2 = |v: f32| (v as f64 * 100.0).round() / 100.0;
        let sw = self.sweeps.lk();
        let people: Vec<Value> = self
            .list()
            .into_iter()
            .map(|n| {
                json!({
                    "name": n.name,
                    "feet": n.feet.map(r2),
                    "tracking": n.tracking.map(r2),
                    "world_yaw": (n.world_yaw as f64 * 10.0).round() / 10.0,
                    "bearing_deg": (n.bearing_deg as f64 * 10.0).round() / 10.0,
                    "distance_m": r2(n.distance_m),
                    "last_seen_s": s1(now.saturating_duration_since(n.seen)),
                    "named_s": s1(now.saturating_duration_since(n.named)),
                    "source": n.source,
                })
            })
            .collect();
        json!({
            "people": people,
            "idle_sweep": {
                "every_s": every_s,
                "sweeps": sw.count,
                "last_ago_s": sw.last.map(|t| s1(now.saturating_duration_since(t))),
                "next_in_s": sw.last.filter(|_| every_s > 0.0).map(|t| s1((t + Duration::from_secs_f32(every_s)).saturating_duration_since(now))),
                "last_end": sw.last_end,
                "last_names": sw.last_names,
                "skipped": sw.skipped,
            },
        })
    }

    /// The idle sweep's scheduler (a thread of its own).
    pub fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        self.sweeps.lk().last = Some(Instant::now());
        loop {
            std::thread::sleep(TICK);
            let every = bridge.usercam.settings.lk().orbit.idle_sweep_s;
            let due = every > 0.0 && self.sweeps.lk().last.is_none_or(|t| t.elapsed().as_secs_f32() >= every);
            if !due {
                continue;
            }
            if let Some(why) = idle_busy(&bridge) {
                self.sweeps.lk().skipped = Some(why.to_string());
                continue;
            }
            self.sweep(&bridge);
        }
    }

    /// One idle sweep: the lens once round, then a look that places the
    /// names it read.
    fn sweep(&self, bridge: &Arc<Bridge>) {
        let t0 = Instant::now();
        if let Err(e) = bridge.orbit.sweep("idle", None, None) {
            self.sweeps.lk().skipped = Some(format!("{e:#}"));
            return;
        }
        {
            let mut sw = self.sweeps.lk();
            sw.last = Some(t0);
            sw.count += 1;
            sw.skipped = None;
        }
        let deadline = t0 + bridge.orbit.sweep_budget() + Duration::from_secs(1);
        while bridge.orbit.sweeping() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        // The last reads in flight.
        let reads = Instant::now() + READS_WAIT;
        while bridge.orbit.reading() > 0 && Instant::now() < reads {
            std::thread::sleep(Duration::from_millis(30));
        }
        let end = bridge.orbit.sweep_end().map(|e| e.0);
        let room = panolook::room(bridge);
        let names = bridge.orbit.names_since(t0).len();
        {
            let mut sw = self.sweeps.lk();
            sw.last_end = end;
            sw.last_names = names;
        }
        if names == 0 || !bridge.pano.usable() {
            return;
        }
        let o = LookOptions { names: true, overlay: false, lens_within: t0.elapsed() + Duration::from_millis(200), source: Some("sweep") };
        match panolook::look(bridge, &room, None, &o) {
            Ok(l) => l.to_speakers(bridge, &bridge.social.whitelist_names()),
            Err(e) => tracing::info!("idle sweep: no pano look: {e:#}"),
        }
    }
}

/// Why the bot is not idle for a sweep now, if it is not.
fn idle_busy(b: &Bridge) -> Option<&'static str> {
    let (running, in_world, others) = {
        let g = b.game.lk();
        (g.running, !g.instance.is_empty(), !g.others().is_empty())
    };
    if !running || !in_world {
        return Some("not in a world");
    }
    if !others {
        return Some("nobody else in the room");
    }
    if !b.follower.is_idle() || b.follow_paused() {
        return Some("following");
    }
    if b.mapping.moved_within(STILL_FOR) {
        return Some("moving");
    }
    if b.speaker.speaking() {
        return Some("someone is speaking");
    }
    if !b.pano.usable() {
        return Some("no panorama (off, or the usual view leased)");
    }
    if b.usercam.settings.lk().orbit.idle == crate::orbit::Idle::Orbit {
        return Some("the lens orbits anyway");
    }
    None
}

/// The cache's update (see `People::update`).
fn update(near: &mut Vec<Near>, seen: &[Seen], head: &HeadThen, override_how: Option<&'static str>, now: Instant) -> Vec<Near> {
    let place = |n: &mut Near, body: Body| {
        let (dx, dz) = (body.feet[0] - head.eye[0], body.feet[2] - head.eye[2]);
        let yaw = dx.atan2(dz).to_degrees().rem_euclid(360.0);
        n.feet = body.feet;
        n.tracking = (head.to_tracking)(body.feet);
        n.world_yaw = yaw;
        n.bearing_deg = vrc_audio::wrap_deg(yaw - head.yaw);
        n.distance_m = dx.hypot(dz);
        n.body = body;
        n.seen = now;
    };
    // The named: theirs, wherever.
    let mut named: Vec<String> = Vec::new();
    for s in seen {
        let Some(name) = s.name else { continue };
        let how = override_how.or(s.how).unwrap_or("named");
        match near.iter_mut().find(|n| n.name == name) {
            Some(n) => {
                place(n, s.body);
                n.named = now;
                n.source = how;
            }
            None => {
                let mut n = Near {
                    name: name.to_string(),
                    feet: [0.0; 3],
                    tracking: [0.0; 3],
                    world_yaw: 0.0,
                    bearing_deg: 0.0,
                    distance_m: 0.0,
                    body: s.body,
                    seen: now,
                    named: now,
                    source: how,
                };
                place(&mut n, s.body);
                near.push(n);
            }
        }
        named.push(name.to_string());
    }
    // The rest: by continuity, latest placed first, each body once, not
    // where someone named stands.
    let mut free: Vec<Body> = seen
        .iter()
        .filter(|s| s.name.is_none())
        .map(|s| s.body)
        .filter(|b| !seen.iter().any(|o| o.name.is_some() && (o.body.feet[0] - b.feet[0]).hypot(o.body.feet[2] - b.feet[2]) < SAME_BODY_M))
        .collect();
    let mut order: Vec<usize> = (0..near.len()).filter(|&i| !named.contains(&near[i].name)).collect();
    order.sort_by(|&a, &b| near[b].seen.cmp(&near[a].seen));
    let mut kept = Vec::new();
    for i in order {
        let since = now.saturating_duration_since(near[i].seen);
        if since > TRACK_FOR || now.saturating_duration_since(near[i].named) > TRACK_UNNAMED_FOR {
            continue;
        }
        let gate = (TRACK_GATE_M + TRACK_SPEED * since.as_secs_f32()).min(TRACK_GATE_MAX_M);
        let at = near[i].feet;
        let best = free
            .iter()
            .enumerate()
            .map(|(k, b)| (k, (b.feet[0] - at[0]).hypot(b.feet[2] - at[2])))
            .filter(|(_, d)| *d <= gate)
            .min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((k, _)) = best {
            let body = free.remove(k);
            place(&mut near[i], body);
            near[i].source = "kept";
            kept.push(near[i].clone());
        }
    }
    near.retain(|n| now.saturating_duration_since(n.seen) <= KEEP_FOR);
    kept
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(x: f32, z: f32) -> Body {
        Body { feet: [x, 0.0, z], top: 1.6, low: 0.1, distance: x.hypot(z), points: 100 }
    }

    #[test]
    fn the_idle_sweep_waits_for_an_idle_bot_and_its_setting_is_checked() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _rt = rt.enter();
        let b = crate::bridge::test_bridge();
        // No game: not idle for a sweep (nothing to read, nobody to move).
        assert_eq!(idle_busy(&b), Some("not in a world"));
        let s = crate::orbit::OrbitSettings::default();
        assert_eq!(s.idle_sweep_s, 60.0);
        assert!(s.merged(&json!({"idle_sweep_s": 0})).is_ok());
        assert!(s.merged(&json!({"idle_sweep_s": 5})).is_err());
        assert!(s.merged(&json!({"idle_sweep_s": 120})).is_ok());
    }

    #[test]
    fn named_people_are_placed_and_followed_between_names() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let id = |p: [f32; 3]| p;
        let head = HeadThen { eye: [0.0, 1.5, 0.0], yaw: 0.0, to_tracking: &id };
        let mut near = Vec::new();
        // A sweep read Ann ahead-right (2 m at 45 degrees) and Bob behind;
        // a person-shaped thing nobody named stands to the left.
        let s = 2f32.sqrt();
        let first = [
            Seen { name: Some("Ann"), body: body(s, s), how: Some("lens") },
            Seen { name: Some("Bob"), body: body(0.0, -3.0), how: Some("lens") },
            Seen { name: None, body: body(-2.0, 0.0), how: None },
        ];
        assert!(update(&mut near, &first, &head, Some("sweep"), at(0)).is_empty());
        assert_eq!(near.len(), 2, "nobody unnamed is anybody");
        let ann = near.iter().find(|n| n.name == "Ann").unwrap();
        assert!((ann.world_yaw - 45.0).abs() < 0.1 && (ann.distance_m - 2.0).abs() < 0.01 && ann.source == "sweep");
        let bob = near.iter().find(|n| n.name == "Bob").unwrap();
        assert!((bob.bearing_deg.abs() - 180.0).abs() < 0.1);
        // Half a second on, no names: Ann stepped 0.4 m, kept; Bob's place
        // has nobody near (the thing at the left is 3.6 m off): not moved.
        let next = [Seen { name: None, body: body(s + 0.4, s), how: None }, Seen { name: None, body: body(-2.0, 0.0), how: None }];
        let kept = update(&mut near, &next, &head, None, at(500));
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].name, "Ann");
        let ann = near.iter().find(|n| n.name == "Ann").unwrap();
        assert!((ann.feet[0] - (s + 0.4)).abs() < 1e-4 && ann.source == "kept" && ann.named == at(0) && ann.seen == at(500));
        assert_eq!(near.iter().find(|n| n.name == "Bob").unwrap().seen, at(0));
        // A name read on someone wins over continuity: Bob named where the
        // thing at the left stood (now he is there).
        let named = [Seen { name: Some("Bob"), body: body(-2.0, 0.0), how: Some("overlay") }, Seen { name: None, body: body(-2.1, 0.0), how: None }];
        let kept = update(&mut near, &named, &head, None, at(1000));
        assert!(kept.iter().all(|k| k.name != "Bob"));
        let bob = near.iter().find(|n| n.name == "Bob").unwrap();
        assert!((bob.feet[0] + 2.0).abs() < 1e-4 && bob.source == "overlay");
        // Long unseen: no continuity (the gate is not stretched for ever),
        // and gone after KEEP_FOR.
        let far_later = at(1000) + TRACK_FOR + Duration::from_secs(1);
        let kept = update(&mut near, &[Seen { name: None, body: body(s + 0.4, s), how: None }], &head, None, far_later);
        assert!(kept.is_empty());
        // Kept look after look, never named again: not past TRACK_UNNAMED_FOR.
        let mut ms = 1000;
        let mut last_kept = 0;
        while ms < 1000 + 2 * TRACK_UNNAMED_FOR.as_millis() as u64 {
            ms += 500;
            if !update(&mut near, &[Seen { name: None, body: body(-2.0, 0.0), how: None }], &head, None, at(ms)).is_empty() {
                last_kept = ms;
            }
        }
        assert!(last_kept > 1000 && Duration::from_millis(last_kept - 1000) <= TRACK_UNNAMED_FOR, "{last_kept}");
        update(&mut near, &[], &head, None, at(ms) + KEEP_FOR + Duration::from_secs(1));
        assert!(near.is_empty());
    }
}
