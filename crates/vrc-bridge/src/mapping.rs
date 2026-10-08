//! The lasting map (`vrc-map`) in the bridge: a map per world, kept as the
//! bot walks, on disk next to the token (`maps/`).
//!
//! - The odometry (a thread of its own, 25 times a second): the avatar's
//!   velocity (OSCQuery `VelocityX/Y/Z`, `Grounded`) turned by where the
//!   head looks, into the feet's position; the feet's trail marks the map.
//!   It also watches the world: a new visit (another world, or the same one
//!   joined again) saves the map and loads the next one (placed on it by
//!   its first looks, see `vrc_map::nav`).
//! - Looks: the follower's go to a thread of their own (a look dropped when
//!   it is busy: the follow never waits); surveys go on at once (a walk
//!   plans on them next).
//! - Saved every minute when changed, and when the world changes.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_map::nav::Begun;
use vrc_map::{store, Belief, MarkKind, Nav, Observation, Shared};
use vrc_vr::osc::Osc;

use crate::bridge::Bridge;
use crate::Lock;

const TICK: Duration = Duration::from_millis(40);
/// `Grounded` is read every this many ticks.
const GROUNDED_EVERY: u32 = 4;
const SAVE_EVERY: Duration = Duration::from_secs(60);
const WORLD_EVERY: Duration = Duration::from_secs(1);
/// Looks waiting for the map thread.
const QUEUE: usize = 2;
/// The beacon is read off the eyes this often moving, and standing (user:
/// "给了 shader 坐标就尽可能用起来"); between, the odometry.
const BEACON_MOVING: Duration = Duration::from_millis(400);
const BEACON_STILL: Duration = Duration::from_secs(5);

pub struct Mapping {
    pub nav: Shared,
    dir: PathBuf,
    tx: SyncSender<Observation>,
    rx: Mutex<Option<Receiver<Observation>>>,
    /// Looks dropped (the map thread was busy).
    dropped: AtomicU64,
    /// The head's heading (session) as last read.
    heading: Mutex<f32>,
    /// When the odometry last moved the feet (for the status).
    moving: Mutex<Option<Instant>>,
}

impl Mapping {
    pub fn new(dir: PathBuf) -> Arc<Mapping> {
        let (tx, rx) = sync_channel(QUEUE);
        Arc::new(Mapping {
            nav: Arc::new(Mutex::new(Nav::new("", vrc_map::WorldMap::default()))),
            dir,
            tx,
            rx: Mutex::new(Some(rx)),
            dropped: AtomicU64::new(0),
            heading: Mutex::new(0.0),
            moving: Mutex::new(None),
        })
    }

    /// Starts the odometry and the map threads.
    pub fn start(self: &Arc<Self>, bridge: Arc<Bridge>) {
        if let Some(rx) = self.rx.lk().take() {
            let me = self.clone();
            std::thread::spawn(move || {
                while let Ok(obs) = rx.recv() {
                    let mut nav = me.nav.lk();
                    if !nav.world.is_empty() {
                        nav.observe(&obs);
                    }
                }
            });
        }
        let me = self.clone();
        std::thread::spawn(move || me.odometry(&bridge));
    }

    /// Whether the odometry moved the feet within `d` (the avatar's own
    /// speed: walking, falling, pushed).
    pub fn moved_within(&self, d: Duration) -> bool {
        self.moving.lk().is_some_and(|t| t.elapsed() < d)
    }

    /// A look for the map, when the map thread is free.
    pub fn observe(&self, obs: Observation) {
        if let Err(TrySendError::Full(_)) = self.tx.try_send(obs) {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// A walk going `heading` (session) was stopped: shut there.
    pub fn stopped(&self, heading: f32) {
        let mut nav = self.nav.lk();
        if !nav.world.is_empty() {
            nav.stopped(heading, 0.4, MarkKind::Blocked, vrc_map::unix_now());
        }
    }

    fn odometry(&self, bridge: &Arc<Bridge>) {
        let mut osc: Option<Osc> = None;
        let mut link: Option<vrc_vr::remote::HmdLink> = None;
        let mut last = Instant::now();
        let mut checked = Instant::now() - WORLD_EVERY;
        let mut saved = Instant::now();
        let mut grounded = true;
        let mut tick = 0u32;
        let mut beacon_read = Instant::now();
        // The eyes' tap of its own: a motion or a look holding the headset
        // (sitting a minute) does not keep the beacon from being read.
        let mut tap = vrc_vr::tap::EyeTap::open(&bridge.args.tap);
        let mut eyes_m: Option<(f32, Instant)> = None;
        loop {
            std::thread::sleep(TICK);
            tick = tick.wrapping_add(1);
            let now = Instant::now();
            if now - checked >= WORLD_EVERY {
                checked = now;
                self.watch_world(bridge);
            }
            if now - saved >= SAVE_EVERY {
                saved = now;
                self.save();
            }
            if osc.is_none() {
                osc = bridge.osc_query().ok();
            }
            if link.is_none() {
                link = bridge.vr.try_lk().and_then(|vr| vr.link());
            }
            let owner = link.as_ref().and_then(|l| l.owner());
            if owner.is_none() {
                link = None;
            }
            let heading = owner.map(|o| o.state.head.yaw_pitch().0);
            if let Some(h) = heading {
                *self.heading.lk() = h;
            }
            let read = |o: &Osc, p: &str| o.query(p).ok().map(|v| v as f32);
            let velocity = osc.as_ref().and_then(|o| {
                Some((read(o, "/avatar/parameters/VelocityX")?, read(o, "/avatar/parameters/VelocityY")?, read(o, "/avatar/parameters/VelocityZ")?))
            });
            let dt = (now - last).as_secs_f32().min(0.2);
            last = now;
            let Some((vx, vy, vz)) = velocity else {
                osc = None;
                continue;
            };
            if tick % GROUNDED_EVERY == 0 {
                if let Some(g) = osc.as_ref().and_then(|o| read(o, "/avatar/parameters/Grounded")) {
                    grounded = g != 0.0;
                }
            }
            // The avatar's velocity is its own (z ahead, x right), as the
            // head looks.
            let (s, c) = heading.unwrap_or(*self.heading.lk()).to_radians().sin_cos();
            let delta = [(vz * s + vx * c) * dt, vy * dt, (-vz * c + vx * s) * dt];
            if delta[0].hypot(delta[2]) > 1e-4 {
                *self.moving.lk() = Some(now);
            }
            {
                let mut nav = self.nav.lk();
                if !nav.world.is_empty() {
                    nav.advance(now, delta, grounded, vrc_map::unix_now());
                }
            }
            // The beacon: where the bot really is.
            let moving = self.moving.lk().is_some_and(|t| t.elapsed() < Duration::from_millis(500));
            let every = if moving { BEACON_MOVING } else { BEACON_STILL };
            if now - beacon_read >= every {
                beacon_read = now;
                if eyes_m.is_none_or(|(_, at)| at.elapsed() > Duration::from_secs(5)) {
                    eyes_m = osc.as_ref().and_then(|o| o.eye_height().ok()).filter(|h| *h > 0.0).map(|h| (h as f32, now));
                }
                let frame = tap.read().ok().flatten();
                if let (Some(frame), Some((h, _))) = (frame, eyes_m) {
                    vrc_nav::beacon_fix_as_is(&self.nav, &frame, h, bridge.anim.params().head_height, now);
                }
            }
        }
    }

    /// A new visit: the map saved, the next loaded.
    fn watch_world(&self, bridge: &Arc<Bridge>) {
        let (world, session) = {
            let g = bridge.game.lk();
            (g.world_id().to_string(), g.session())
        };
        if world.is_empty() {
            return; // between worlds: the map stays till the next
        }
        {
            let nav = self.nav.lk();
            if nav.world == world && nav.session == session {
                return;
            }
        }
        self.save();
        let file = store::path(&self.dir, &world);
        let stored = match store::load(&file) {
            Ok(m) => Some(m),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
            Err(e) => {
                tracing::warn!("map of {world} unreadable ({e}): starting anew, the file kept aside");
                let _ = std::fs::rename(&file, file.with_extension(format!("vrcmap.{}.bad", vrc_map::unix_now())));
                None
            }
        };
        let head = *self.heading.lk();
        let begun = self.nav.lk().begin(&world, &session, stored, head);
        tracing::info!(world, session, ?begun, "map: a visit begins");
        if begun == Begun::Fresh {
            // Saved at once: the next visit knows where this one began.
            self.save();
        }
    }

    /// Writes the map (when changed and placed); a map this visit replaced
    /// is kept aside first.
    pub fn save(&self) {
        let (file, meta, map, replaced) = {
            let mut nav = self.nav.lk();
            // Only maps in the world's own frame (the beacon's) are kept.
            if nav.world.is_empty() || nav.placing.is_some() || !nav.world_frame || (!nav.map.dirty && !nav.replaced) {
                return;
            }
            nav.map.dirty = false;
            let replaced = std::mem::take(&mut nav.replaced);
            (store::path(&self.dir, &nav.world), nav.meta(vrc_map::unix_now()), nav.map.clone(), replaced)
        };
        if replaced && file.exists() {
            let aside = file.with_extension(format!("vrcmap.{}.old", vrc_map::unix_now()));
            if let Err(e) = std::fs::rename(&file, &aside) {
                tracing::warn!("could not keep the old map aside: {e}");
            }
        }
        if let Err(e) = store::save(&file, &meta, &map) {
            tracing::warn!("map not saved: {e}");
            self.nav.lk().map.dirty = true;
        }
    }

    /// Forgets this world's map (kept aside on disk) and starts it anew here.
    pub fn forget(&self) -> anyhow::Result<()> {
        let (world, session) = {
            let nav = self.nav.lk();
            anyhow::ensure!(!nav.world.is_empty(), "not in a world");
            (nav.world.clone(), nav.session.clone())
        };
        let file = store::path(&self.dir, &world);
        if file.exists() {
            std::fs::rename(&file, file.with_extension(format!("vrcmap.{}.old", vrc_map::unix_now())))?;
        }
        let head = *self.heading.lk();
        self.nav.lk().begin(&world, &session, None, head);
        self.nav.lk().replaced = false;
        self.save();
        Ok(())
    }

    /// Names where the bot stands.
    pub fn name_place(&self, name: &str) -> anyhow::Result<[f32; 3]> {
        let mut nav = self.nav.lk();
        anyhow::ensure!(nav.ready(), "the map works only where the avatar's position beacon is read (none now)");
        let at = nav.pose;
        let heading = Some(nav.map_heading(*self.heading.lk()));
        nav.map.places.retain(|p| p.name != name);
        nav.map.places.push(vrc_map::Place { name: name.to_string(), at, heading });
        nav.map.dirty = true;
        Ok(at)
    }

    pub fn status(&self) -> Value {
        let nav = self.nav.lk();
        let now = vrc_map::unix_now();
        let r2 = |v: f32| (v as f64 * 100.0).round() / 100.0;
        let (columns, filled, walked) = nav.map.census();
        let shut = nav.map.marks.iter().filter(|m| m.belief(now) == Belief::Shut).count();
        let doubtful = nav.map.marks.iter().filter(|m| m.belief(now) == Belief::Doubtful).count();
        let heading = nav.map_heading(*self.heading.lk());
        json!({
            "world": nav.world,
            "session": nav.session,
            "source": if nav.fixed() { "beacon" } else { "odometry" },
            "world_frame": nav.world_frame,
            "ready": nav.ready(),
            "writable": nav.writable(),
            "fixes": nav.fixes,
            "placing": nav.placing.as_ref().map(|p| json!({"tries": p.tries})),
            "pose": nav.pose.map(r2),
            "heading_deg": heading.round(),
            "yaw_deg": r2(nav.yaw),
            "moving": self.moving.lk().is_some_and(|t| t.elapsed() < Duration::from_millis(500)),
            "looks": nav.looks,
            "fits": nav.fits,
            "dropped": self.dropped.load(Ordering::Relaxed),
            "walked_m": r2(nav.walked),
            "corrected_m": r2(nav.corrected),
            "last_fit": nav.last_fit.map(|f| json!({
                "shift": f.shift.map(r2), "overlap": r2(f.overlap), "used": f.used,
                "across": f.across, "up": f.up, "flat_deg": f.flat.map(|d| d.round()), "why": f.why,
            })),
            "columns": columns,
            "filled_voxels": filled,
            "walked_columns": walked,
            "marks": {"shut": shut, "doubtful": doubtful},
            "objects": nav.map.objects.iter().map(|o| json!({
                "label": o.label, "at": o.at.map(r2), "seen": o.seen, "score": r2(o.score),
                "confirmed": o.confirmed(), "spread_m": r2(o.spread()), "size_m": o.size.map(r2),
                "kinds": o.kinds.iter().map(|k| json!([k.0, k.1])).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
            "places": nav.map.places.iter().map(|p| json!({"name": p.name, "at": p.at.map(r2)})).collect::<Vec<_>>(),
            "spawns": nav.spawns.len(),
        })
    }

    /// The map from above round the bot, as PNG: `radius` metres, the way
    /// it faces up (`up`) or the map's -z.
    pub fn png(&self, radius: f32, px: usize, up: bool, to: Option<[f32; 2]>) -> anyhow::Result<Vec<u8>> {
        let nav = self.nav.lk();
        anyhow::ensure!(!nav.world.is_empty(), "no map yet (not in a world)");
        let heading = nav.map_heading(*self.heading.lk());
        let p = vrc_nav::plan_params(1.1);
        // A way to `to` (map x, z), drawn.
        let path = to
            .and_then(|goal| vrc_map::plan::Planner::new(&nav.map, p, nav.pose).plan(goal, None))
            .map(|p| p.points)
            .unwrap_or_default();
        let pic = vrc_map::render::render(&nav.map, nav.pose, heading, radius, px, up.then_some(heading), &path, &p);
        drop(nav);
        let mut out = Vec::new();
        {
            let mut enc = png::Encoder::new(&mut out, pic.side as u32, pic.side as u32);
            enc.set_color(png::ColorType::Rgb);
            enc.set_depth(png::BitDepth::Eight);
            enc.write_header()?.write_image_data(&pic.rgb)?;
        }
        Ok(out)
    }
}
