//! Where the bot is on its map: an odometry (the avatar's own velocity,
//! integrated), kept on the map by fitting each look to it
//! ([`crate::register`]), or set outright where the world's own
//! coordinates are read (an avatar shader writing them into a corner of
//! the eyes: then nothing drifts and a world's map holds from one visit to
//! the next).
//!
//! Frames: the **session**'s axes are the tracking space's (fixed to the
//! world while the bot is in it: the playspace never turns); the **map**'s
//! are the session's turned `yaw` degrees and shifted (another visit, its
//! own session). Positions are of the feet: where the avatar stands on its
//! floor.
//!
//! A visit (a session) begins where the world spawns the bot. The first
//! visit's map takes the session's axes. On a later visit the bot stands at
//! one of the spawns the map knows, facing that spawn's way (VRChat turns
//! the arriving player to it): each is a guess of how the session lies on
//! the map, tried (a little turned either way, then all round) by fitting
//! the first looks to the map. Until one fits, the looks go into a map of
//! their own; once one does, they are put on the world's map and the bot
//! carries on there. None fitting after a few looks, the visit's own map
//! becomes the world's (the old one is kept aside by the caller).

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::register::{register, Fit};
use crate::store::Meta;
use crate::{cell_of, turn, wrap, MarkKind, Observation, WorldMap};

/// Poses kept this long back (a look is placed by the pose when taken).
const HISTORY: Duration = Duration::from_secs(5);
/// Feet on the ground are pulled this share of the way to a seen surface
/// within SNAP_M of them, each step of the odometry.
const SNAP: f32 = 0.2;
const SNAP_M: f32 = 0.35;
/// The feet's trail is marked every this far.
const TRAIL_M: f32 = 0.1;
/// A step of the trail longer than this is no walk but a jump of the pose
/// (a respawn or teleport, a misread beacon, a fix putting the odometry
/// right): it clears no mark (glass marks went, the bot never through
/// them) and marks nothing walked between. Walking (4 m/s at most, the
/// odometry at 25 Hz, the beacon every few tenths) steps far less.
const JUMP_M: f32 = 0.6;
/// Placing a visit: looks kept to try (those with walls enough), the
/// share of their wall points that must fall on the map's walls, the turns
/// tried round a spawn's way (and all round, this far apart).
const PLACE_LOOKS: usize = 4;
const PLACE_WALLS: usize = 300;
const PLACE_OVERLAP: f32 = 0.55;
const PLACE_NEAR_DEG: [f32; 9] = [0.0, -2.0, 2.0, -4.0, 4.0, -7.0, 7.0, -10.0, 10.0];
const PLACE_ALL_ROUND_DEG: f32 = 5.0;
/// A spawn within this of a known one is that one.
const SAME_SPAWN_M: f32 = 1.5;
/// A look is fitted only after the feet moved this far since the last one
/// (standing, the odometry is right: a fit would only follow the depth's error,
/// as first measured: 0.84 m of "corrections" in a look round on the spot),
/// and by at most FIT_BASE plus FIT_SHARE of that way across (FIT_UP_SHARE
/// up).
const FIT_AFTER_M: f32 = 0.1;
const FIT_BASE: f32 = 0.05;
const FIT_SHARE: f32 = 0.25;
const FIT_UP_SHARE: f32 = 0.2;
/// A walk stopped by something unseen is stopped by a pane (glass, an
/// invisible wall), and panes span between solid things (a frame, a
/// pillar, a wall): the mark runs across the way, every PANE_STEP, out to
/// the first thing seen standing there (knee to head high) each side, at
/// most PANE_MAX; with nothing seen that side, PANE_SHORT. (Marked where it
/// was bumped only, the way round went past the mark's end into the same
/// pane, again and again.)
pub const PANE_STEP: f32 = 0.15;
const PANE_MAX: f32 = 3.0;
const PANE_SHORT: f32 = 0.75;

/// Not moved further than this since the beacon's last reading: the pose
/// is still its (the bot sat on a sofa a minute: "记住这里" failed there).
const STILL_SINCE_FIX_M: f32 = 0.05;

/// The beacon's word holds this long: after it, looks are fitted again.
const FIX_HOLDS: Duration = Duration::from_secs(10);
/// The map is written (looks, the trail, marks, things) only this soon after
/// the beacon said where the bot is (user: "无坐标时直接不建地图": no world
/// coordinates, no map; between readings the odometry carries the pose).
const FIX_WRITES: Duration = Duration::from_secs(3);

pub type Shared = Arc<Mutex<Nav>>;

impl std::fmt::Debug for Nav {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Nav").field("world", &self.world).field("pose", &self.pose).field("yaw", &self.yaw).finish_non_exhaustive()
    }
}

/// How the pose is kept.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// Odometry, fitted to the map.
    Odometry,
    /// The world's own coordinates.
    World,
}

pub struct Nav {
    pub map: WorldMap,
    /// The world (`wrld_...`) the map is of.
    pub world: String,
    /// The feet (map frame).
    pub pose: [f32; 3],
    /// The session's axes turned this far are the map's (degrees).
    pub yaw: f32,
    pub source: Source,
    history: VecDeque<(Instant, [f32; 3])>,
    trail: Option<[f32; 3]>,
    pub last_fit: Option<Fit>,
    /// Looks added, and those whose shift across was taken.
    pub looks: u32,
    pub fits: u32,
    /// Metres walked by the odometry, and corrected by fits (across).
    pub walked: f32,
    pub corrected: f32,
    /// Walked since the last look.
    unfitted: f32,
    /// Moved since the beacon's last reading.
    since_fix: f32,
    /// The map is in the world's own frame (the position beacon's), and
    /// when the beacon last said where the bot is.
    pub world_frame: bool,
    pub last_fix: Option<Instant>,
    pub fixes: u32,
    /// The map only with world coordinates (the default): without the
    /// beacon nothing is written or walked by. Off, the odometry fitted to
    /// the map carries it (kept for worlds and avatars without one).
    pub beacon_only: bool,
    /// The visit (the game's log and its joins): a bridge started again
    /// in the same one carries on where it was.
    pub session: String,
    /// Where visits began (map frame: feet, heading).
    pub spawns: Vec<[f32; 4]>,
    /// Not yet placed on the world's map (a later visit).
    pub placing: Option<Placing>,
    /// The world's map was not found again: this visit's replaced it (the
    /// caller keeps the old file aside).
    pub replaced: bool,
}

/// A later visit, not yet placed.
pub struct Placing {
    /// The world's map, as loaded.
    stored: WorldMap,
    /// The session's heading at the spawn.
    head: f32,
    /// The world's map is in the world's own frame.
    stored_world: bool,
    /// Looks to fit, with the feet then (session frame).
    kept: Vec<(Observation, [f32; 3])>,
    pub tries: u32,
}

/// How a visit began.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Begun {
    /// The same visit as the map was saved in: carried on.
    Resumed,
    /// A new world (no map yet), or one whose map has no spawn to start from.
    Fresh,
    /// A known world: being placed on its map.
    Placing,
}

impl Nav {
    pub fn new(world: &str, map: WorldMap) -> Nav {
        Nav {
            map,
            world: world.to_string(),
            pose: [0.0; 3],
            yaw: 0.0,
            source: Source::Odometry,
            history: VecDeque::new(),
            trail: None,
            last_fit: None,
            looks: 0,
            fits: 0,
            walked: 0.0,
            corrected: 0.0,
            unfitted: 0.0,
            since_fix: 0.0,
            world_frame: false,
            last_fix: None,
            fixes: 0,
            beacon_only: true,
            session: String::new(),
            spawns: Vec::new(),
            placing: None,
            replaced: false,
        }
    }

    /// A visit to `world` begins (or the bridge started during one):
    /// `stored`, the world's map from disk if any; `head`, the head's heading
    /// now (session).
    pub fn begin(&mut self, world: &str, session: &str, stored: Option<(Meta, WorldMap)>, head: f32) -> Begun {
        let beacon_only = self.beacon_only;
        let begun = self.begin_anew(world, session, stored, head);
        self.beacon_only = beacon_only;
        begun
    }

    fn begin_anew(&mut self, world: &str, session: &str, stored: Option<(Meta, WorldMap)>, head: f32) -> Begun {
        match stored {
            Some((meta, map)) if meta.session == session => {
                *self = Nav::new(world, map);
                self.session = session.to_string();
                self.pose = meta.pose;
                self.yaw = meta.yaw;
                self.spawns = meta.spawns;
                self.world_frame = meta.world_frame;
                Begun::Resumed
            }
            Some((meta, map)) if !meta.spawns.is_empty() => {
                *self = Nav::new(world, WorldMap::default());
                self.session = session.to_string();
                self.spawns = meta.spawns;
                self.placing = Some(Placing { stored: map, head, stored_world: meta.world_frame, kept: Vec::new(), tries: 0 });
                Begun::Placing
            }
            stored => {
                *self = Nav::new(world, WorldMap::default());
                self.session = session.to_string();
                self.spawns = vec![[0.0, 0.0, 0.0, head]];
                self.replaced = stored.is_some();
                Begun::Fresh
            }
        }
    }

    /// What to save beside the map.
    pub fn meta(&self, saved: u64) -> Meta {
        Meta {
            world: self.world.clone(),
            session: self.session.clone(),
            pose: self.pose,
            yaw: self.yaw,
            saved,
            spawns: self.spawns.clone(),
            world_frame: self.world_frame,
        }
    }

    /// Whether the beacon said where the bot is lately.
    pub fn fixed(&self) -> bool {
        self.last_fix.is_some_and(|t| t.elapsed() < FIX_HOLDS) || self.still_since_fix()
    }

    /// Read by the beacon once, and not moved since: the pose is still its.
    fn still_since_fix(&self) -> bool {
        self.last_fix.is_some() && self.since_fix < STILL_SINCE_FIX_M
    }

    /// The map can be walked by: in the world's frame, the beacon read
    /// lately (without `beacon_only`: placed on it, as the odometry has it).
    pub fn ready(&self) -> bool {
        let placed = !self.world.is_empty() && self.placing.is_none();
        placed && (!self.beacon_only || (self.world_frame && self.fixed()))
    }

    /// The map can be written: ready, the beacon read just now.
    pub fn writable(&self) -> bool {
        if self.beacon_only {
            self.ready() && (self.last_fix.is_some_and(|t| t.elapsed() < FIX_WRITES) || self.still_since_fix())
        } else {
            !self.world.is_empty() // a visit being placed writes its own map
        }
    }

    /// The position beacon: at `at` the feet were at `feet` and the head
    /// faced `heading` (the world's frame, the bot's axes), the head facing
    /// `head` in the session. The first time, the map goes over to the
    /// world's frame (the world's map, if it is in it already: the looks
    /// of this visit put on it).
    pub fn fix(&mut self, at: Instant, feet: [f32; 3], heading: f32, head: f32) {
        let yaw = wrap(heading - head);
        if !self.world_frame {
            let from = self.pose_at(at);
            let turned = wrap(yaw - self.yaw);
            let go = |p: [f32; 3]| {
                let q = turn([p[0] - from[0], p[1] - from[1], p[2] - from[2]], turned);
                [q[0] + feet[0], q[1] + feet[1], q[2] + feet[2]]
            };
            match self.placing.take() {
                Some(pl) if pl.stored_world => {
                    let mut map = pl.stored;
                    for (obs, p) in &pl.kept {
                        map.integrate(obs, go(*p), yaw);
                    }
                    self.map = map;
                }
                other => {
                    self.map = self.map.transformed(from, feet, turned);
                    self.spawns = self.spawns.iter().map(|s| {
                        let p = go([s[0], s[1], s[2]]);
                        [p[0], p[1], p[2], wrap(s[3] + turned)]
                    }).collect();
                    // A map of another frame the world's file held: kept aside.
                    self.replaced |= other.is_some();
                }
            }
            self.pose = go(self.pose);
            for h in &mut self.history {
                h.1 = go(h.1);
            }
            self.trail = None;
            self.world_frame = true;
            self.map.dirty = true;
        }
        self.yaw = yaw;
        self.source = Source::World;
        // The feet now: where the beacon put them, and what the odometry
        // walked since.
        let then = self.pose_at(at);
        let s = [feet[0] - then[0], feet[1] - then[1], feet[2] - then[2]];
        for k in 0..3 {
            self.pose[k] += s[k];
        }
        for h in &mut self.history {
            for k in 0..3 {
                h.1[k] += s[k];
            }
        }
        self.unfitted = 0.0;
        self.last_fix = Some(at);
        self.since_fix = 0.0;
        self.fixes += 1;
    }

    /// Tries to place the visit on the world's map from the looks kept.
    fn try_place(&mut self) {
        let Some(pl) = &mut self.placing else { return };
        if pl.kept.is_empty() {
            return;
        }
        pl.tries += 1;
        let pl: &Placing = pl;
        // How the looks fit with the session turned `yaw` and its origin at
        // `origin` (map): their mean overlap and the origin put right.
        let fits = |yaw: f32, origin: [f32; 3]| -> Option<(f32, [f32; 3])> {
            let mut sum = 0.0;
            let mut shift = [0.0f32; 3];
            for (obs, p) in &pl.kept {
                let q = turn(*p, yaw);
                let predicted = [origin[0] + q[0], origin[1] + q[1], origin[2] + q[2]];
                let f = register(&pl.stored, obs, predicted, yaw);
                if !f.across {
                    return None;
                }
                sum += f.overlap;
                for k in 0..3 {
                    shift[k] += f.shift[k] / pl.kept.len() as f32;
                }
            }
            Some((sum / pl.kept.len() as f32, [origin[0] + shift[0], origin[1] + shift[1], origin[2] + shift[2]]))
        };
        // (mean overlap, yaw, the session's origin on the map)
        let mut best: Option<(f32, f32, [f32; 3])> = None;
        for s in &self.spawns {
            let origin = [s[0], s[1], s[2]];
            let base = s[3] - pl.head;
            for pass in 0..2 {
                let turns: Vec<f32> = if pass == 0 {
                    PLACE_NEAR_DEG.to_vec()
                } else if best.is_some_and(|b| b.0 >= PLACE_OVERLAP) {
                    break;
                } else {
                    (0..(360.0 / PLACE_ALL_ROUND_DEG) as i32).map(|k| k as f32 * PLACE_ALL_ROUND_DEG).collect()
                };
                for d in turns {
                    let yaw = wrap(base + d);
                    if let Some((overlap, at)) = fits(yaw, origin) {
                        if best.is_none_or(|b| overlap > b.0) {
                            best = Some((overlap, yaw, at));
                        }
                    }
                }
            }
        }
        let enough = pl.kept.len() >= PLACE_LOOKS;
        match best {
            Some((overlap, yaw, origin)) if overlap >= PLACE_OVERLAP => {
                let pl = self.placing.take().unwrap();
                let mut map = pl.stored;
                let to_map = |p: [f32; 3]| {
                    let q = turn(p, yaw);
                    [origin[0] + q[0], origin[1] + q[1], origin[2] + q[2]]
                };
                for (obs, p) in &pl.kept {
                    map.integrate(obs, to_map(*p), yaw);
                }
                self.map = map;
                self.pose = to_map(self.pose);
                for h in &mut self.history {
                    h.1 = to_map(h.1);
                }
                self.trail = None;
                self.yaw = yaw;
                self.world_frame = pl.stored_world;
                self.add_spawn([origin[0], origin[1], origin[2], wrap(pl.head + yaw)]);
            }
            _ if enough => {
                // Not found: this visit's map becomes the world's.
                let pl = self.placing.take().unwrap();
                self.spawns = vec![[0.0, 0.0, 0.0, pl.head]];
                self.replaced = true;
            }
            _ => {}
        }
    }

    fn add_spawn(&mut self, s: [f32; 4]) {
        if !self.spawns.iter().any(|k| (k[0] - s[0]).hypot(k[2] - s[2]) < SAME_SPAWN_M && (k[1] - s[1]).abs() < 1.0) {
            self.spawns.push(s);
        }
    }

    /// The odometry moved the feet `delta` (session axes, world metres)
    /// by `at`; `grounded`: on the ground (not in the air).
    pub fn advance(&mut self, at: Instant, delta: [f32; 3], grounded: bool, now: u64) {
        let d = turn(delta, self.yaw);
        for k in 0..3 {
            self.pose[k] += d[k];
        }
        self.walked += d[0].hypot(d[2]);
        self.since_fix += d[0].hypot(d[2]) + d[1].abs();
        self.unfitted += d[0].hypot(d[2]);
        if grounded {
            self.snap();
        }
        self.moved(at, grounded, now);
    }

    /// The world's own coordinates say the feet are at `pose` (map frame).
    pub fn set(&mut self, at: Instant, pose: [f32; 3], grounded: bool, now: u64) {
        self.source = Source::World;
        self.pose = pose;
        self.moved(at, grounded, now);
    }

    fn moved(&mut self, at: Instant, grounded: bool, now: u64) {
        self.history.push_back((at, self.pose));
        while self.history.front().is_some_and(|h| at.saturating_duration_since(h.0) > HISTORY) {
            self.history.pop_front();
        }
        if !self.writable() {
            self.trail = None;
            return;
        }
        // The trail: on the ground only (not a jump's arc).
        if !grounded {
            self.trail = None;
            return;
        }
        let p = self.pose;
        match self.trail {
            Some(t) if (t[0] - p[0]).hypot(t[2] - p[2]) < TRAIL_M => {}
            last => {
                if let Some(t) = last.filter(|t| (t[0] - p[0]).hypot(t[2] - p[2]) <= JUMP_M) {
                    self.map.walked_through(t, p);
                }
                self.map.walked(p, true, now);
                self.trail = Some(p);
            }
        }
    }

    /// Feet on the ground stand on what was seen there.
    fn snap(&mut self) {
        let surfaces = self.map.surfaces(cell_of(self.pose[0], self.pose[2]));
        let y = self.pose[1];
        if let Some(s) = surfaces.iter().filter(|s| s.seen && (s.h - y).abs() < SNAP_M).min_by(|a, b| (a.h - y).abs().total_cmp(&(b.h - y).abs())) {
            self.pose[1] += (s.h - y) * SNAP;
        }
    }

    /// The feet at `at` (from the history; now if newer or older).
    pub fn pose_at(&self, at: Instant) -> [f32; 3] {
        let mut before: Option<&(Instant, [f32; 3])> = None;
        for h in &self.history {
            if h.0 >= at {
                return match before {
                    Some(b) if h.0 > b.0 => {
                        let f = (at - b.0).as_secs_f32() / (h.0 - b.0).as_secs_f32();
                        [0, 1, 2].map(|k| b.1[k] + (h.1[k] - b.1[k]) * f)
                    }
                    _ => h.1,
                };
            }
            before = Some(h);
        }
        self.pose
    }

    /// Adds a look: fitted to the map first (the odometry put right by
    /// it), then put on it.
    pub fn observe(&mut self, obs: &Observation) -> Fit {
        if !self.writable() {
            return Fit { why: "no world coordinates (the avatar's beacon)", ..Default::default() };
        }
        let then = self.pose_at(obs.at);
        let moved = std::mem::take(&mut self.unfitted);
        let fit = if self.fixed() {
            Fit { why: "world coordinates", ..Default::default() }
        } else if moved < FIT_AFTER_M {
            Fit { why: "standing still", ..Default::default() }
        } else {
            let mut f = register(&self.map, obs, then, self.yaw);
            if f.shift[0].hypot(f.shift[2]) > FIT_BASE + FIT_SHARE * moved {
                f = Fit { why: "more than the walk explains", used: f.used, overlap: f.overlap, ..Default::default() };
            } else if f.shift[1].abs() > FIT_BASE + FIT_UP_SHARE * moved {
                f.shift[1] = 0.0;
                f.up = false;
            }
            f
        };
        let s = fit.shift;
        if s != [0.0; 3] {
            for k in 0..3 {
                self.pose[k] += s[k];
            }
            for h in &mut self.history {
                for k in 0..3 {
                    h.1[k] += s[k];
                }
            }
            if let Some(t) = &mut self.trail {
                for k in 0..3 {
                    t[k] += s[k];
                }
            }
            self.corrected += s[0].hypot(s[2]);
        }
        let then = [then[0] + s[0], then[1] + s[1], then[2] + s[2]];
        self.map.integrate(obs, then, self.yaw);
        self.looks += 1;
        self.fits += fit.across as u32;
        self.last_fit = Some(fit);
        if let Some(pl) = &mut self.placing {
            let walls = obs.points.iter().filter(|p| p[1] > 0.4 && p[1] < obs.eye[1] + 0.3 && obs.keeps(**p)).count();
            if walls >= PLACE_WALLS && pl.kept.len() < PLACE_LOOKS {
                pl.kept.push((obs.clone(), then));
                self.try_place();
            }
        }
        fit
    }

    /// A point of a look taken at `at` (its frame) on the map.
    pub fn place(&self, rel: [f32; 3], at: Instant) -> [f32; 3] {
        let p = self.pose_at(at);
        let q = turn(rel, self.yaw);
        [p[0] + q[0], p[1] + q[1], p[2] + q[2]]
    }

    /// A heading of the session (the head's yaw) on the map, and back.
    pub fn map_heading(&self, session: f32) -> f32 {
        wrap(session + self.yaw)
    }

    pub fn session_heading(&self, map: f32) -> f32 {
        wrap(map - self.yaw)
    }

    /// A walk going `heading` (session) was stopped: the way is shut
    /// `ahead` metres on.
    pub fn stopped(&mut self, heading: f32, ahead: f32, kind: MarkKind, now: u64) {
        if !self.writable() {
            return;
        }
        let yaw = self.map_heading(heading);
        let (s, c) = yaw.to_radians().sin_cos();
        let mid = [self.pose[0] + s * ahead, self.pose[1], self.pose[2] - c * ahead];
        let at = |k: f32| [mid[0] + c * k, mid[1], mid[2] + s * k];
        // A jump that fell short: that ledge, where it was tried.
        if kind != MarkKind::Blocked {
            self.map.mark(mid, yaw, kind, now);
            return;
        }
        // Across the way, to the pane's frame each side.
        let feet = self.pose[1];
        let reach = |side: f32| {
            let mut k = PANE_STEP;
            while k <= PANE_MAX {
                let p = at(side * k);
                if self.map.column(cell_of(p[0], p[2])).is_some_and(|col| col.filled_between(feet + 0.5, feet + 1.6)) {
                    return k - PANE_STEP;
                }
                k += PANE_STEP;
            }
            PANE_SHORT
        };
        let (left, right) = (reach(-1.0), reach(1.0));
        let mut k = -left;
        while k <= right + 1e-3 {
            self.map.mark(at(k), yaw, kind, now);
            k += PANE_STEP;
        }
    }

    /// The places named and the things confirmed on the map, nearest
    /// first: (name, kind, where). Things of a kind are named by it, the
    /// nearest first ("couch", "couch 2"...).
    pub fn landmarks(&self) -> Vec<(String, &'static str, [f32; 3])> {
        let p = self.pose;
        let dist = |a: &[f32; 3]| (a[0] - p[0]).hypot(a[2] - p[2]);
        let mut out: Vec<(String, &'static str, [f32; 3])> = self.map.places.iter().map(|pl| (pl.name.clone(), "place", pl.at)).collect();
        let mut things: Vec<&crate::Object> = self.map.objects.iter().filter(|o| o.confirmed()).collect();
        things.sort_by(|a, b| dist(&a.at).total_cmp(&dist(&b.at)));
        let mut counts: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
        for o in things {
            let n = counts.entry(o.label.as_str()).or_insert(0);
            *n += 1;
            let name = if *n == 1 { o.label.clone() } else { format!("{} {}", o.label, n) };
            out.push((name, "thing", o.at));
        }
        out.sort_by(|a, b| dist(&a.2).total_cmp(&dist(&b.2)));
        out
    }

    /// A landmark by name ([`Nav::landmarks`]): a place's name, a thing's
    /// ("couch", "couch 2"), or a thing's kind in Chinese ("沙发": the
    /// nearest).
    pub fn find(&self, name: &str) -> Option<(String, [f32; 3], Option<f32>)> {
        let want = name.trim().to_lowercase();
        // A place named: where, and the way the bot faced there.
        if let Some(p) = self.map.places.iter().find(|p| p.name.to_lowercase() == want) {
            return Some((p.name.clone(), p.at, p.heading));
        }
        let marks = self.landmarks();
        if let Some(l) = marks.iter().find(|l| l.0.to_lowercase() == want) {
            return Some((l.0.clone(), l.2, None));
        }
        let (kind, nth) = match want.rsplit_once(' ') {
            Some((k, n)) if n.parse::<usize>().is_ok() => (k.to_string(), n.parse::<usize>().unwrap()),
            _ => (want.clone(), 1),
        };
        let label = crate::label_of(&kind)?;
        let found = if nth == 1 { label.to_string() } else { format!("{label} {nth}") };
        marks.iter().find(|l| l.0 == found).map(|l| (l.0.clone(), l.2, None))
    }

    /// Starts over in another world (or this one, anew): `map` as loaded.
    pub fn reset(&mut self, world: &str, map: WorldMap) {
        *self = Nav::new(world, map);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{floor, look, wall_x, wall_z};

    #[test]
    fn odometry_trail_and_a_look_put_right() {
        let mut pts = Vec::new();
        floor(&mut pts, (-4.0, 4.0), (-4.0, 4.0), 0.0);
        wall_x(&mut pts, (-4.0, 4.0), -4.0, 0.0, 2.5);
        wall_x(&mut pts, (-4.0, 4.0), 3.0, 0.0, 2.5);
        wall_z(&mut pts, (-4.0, 3.0), 3.5, 0.0, 2.5);
        wall_z(&mut pts, (-4.0, 3.0), -3.5, 0.0, 2.5);
        let mut nav = Nav { beacon_only: false, ..Nav::new("wrld_test", WorldMap::default()) };
        let t0 = Instant::now();
        nav.observe(&Observation { at: t0, ..look(&pts, [0.0; 3], 1.5) });
        // Walk 1 m right; the odometry says 1.2.
        for i in 1..=12 {
            nav.advance(t0 + Duration::from_millis(100 * i), [0.1, 0.0, 0.0], true, 0);
        }
        assert!((nav.pose[0] - 1.2).abs() < 1e-4);
        let at = t0 + Duration::from_millis(1300);
        nav.advance(at, [0.0; 3], true, 0);
        let fit = nav.observe(&Observation { at, ..look(&pts, [1.0, 0.0, 0.0], 1.5) });
        assert!(fit.across, "{fit:?}");
        assert!((nav.pose[0] - 1.0).abs() < 0.04, "{:?} {fit:?}", nav.pose);
        // The trail: about every 10 cm along the way.
        let (_, _, walked) = nav.map.census();
        assert!(walked >= 10, "{walked}");
    }

    #[test]
    fn a_later_visit_is_placed_on_the_worlds_map() {
        // An L-shaped room (not the same turned any way).
        let mut pts = Vec::new();
        floor(&mut pts, (-4.0, 6.0), (-4.0, 4.0), 0.0);
        wall_x(&mut pts, (-4.0, 6.0), -4.0, 0.0, 2.5);
        wall_x(&mut pts, (-4.0, 2.0), 3.0, 0.0, 2.5);
        wall_z(&mut pts, (-4.0, 3.0), -3.5, 0.0, 2.5);
        wall_z(&mut pts, (-4.0, 0.0), 5.5, 0.0, 2.5);
        wall_x(&mut pts, (2.0, 5.5), 0.0, 0.0, 2.5);
        // First visit: spawned at (1, 0, 1) facing -z; the session's axes
        // are the map's, its origin the spawn.
        let spawn = [1.0f32, 0.0, 1.0];
        let local: Vec<[f32; 3]> = pts.iter().map(|p| [p[0] - spawn[0], p[1], p[2] - spawn[2]]).collect();
        let mut first = Nav { beacon_only: false, ..Nav::new("w", WorldMap::default()) };
        assert_eq!(first.begin("w", "s1", None, 0.0), Begun::Fresh);
        first.observe(&look(&local, [0.0; 3], 1.5));
        let saved = (first.meta(1), first.map.clone());
        // Second visit: the same spawn, the session's axes turned (the head
        // faced the session's 90 as it arrived, the spawn's way being the
        // map's 0): a map heading is the session's less 90.
        let turned: Vec<[f32; 3]> = local.iter().map(|p| turn(*p, 90.0)).collect();
        let mut second = Nav { beacon_only: false, ..Nav::new("w", WorldMap::default()) };
        assert_eq!(second.begin("w", "s2", Some(saved.clone()), 90.0), Begun::Placing);
        second.observe(&look(&turned, [0.0; 3], 1.5));
        assert!(second.placing.is_none(), "placed after one look");
        assert!(wrap(second.yaw + 90.0).abs() < 3.0, "{}", second.yaw);
        assert!(second.pose[0].abs() < 0.1 && second.pose[2].abs() < 0.1, "{:?}", second.pose);
        // The same visit again (the bridge restarted): carried on.
        let mut again = Nav::new("w", WorldMap::default());
        assert_eq!(again.begin("w", "s1", Some(saved), 0.0), Begun::Resumed);
    }

    #[test]
    fn the_beacon_takes_the_map_to_the_worlds_frame() {
        let mut pts = Vec::new();
        floor(&mut pts, (-3.0, 3.0), (-3.0, 3.0), 0.0);
        wall_x(&mut pts, (-3.0, 3.0), -3.0, 0.0, 2.5);
        let mut nav = Nav::new("w", WorldMap::default());
        nav.begin("w", "s1", None, 0.0);
        let t0 = Instant::now();
        // No world coordinates yet: nothing is written.
        let fit = nav.observe(&Observation { at: t0, ..look(&pts, [0.0; 3], 1.5) });
        assert!(nav.map.chunks.is_empty() && !nav.ready(), "{fit:?}");
        // The world: the bot stands at (10, 2, 20), and its session's ahead
        // (-z) is the world's +x (heading 90).
        nav.fix(t0, [10.0, 2.0, 20.0], 90.0, 0.0);
        assert!(nav.ready() && nav.writable());
        nav.observe(&Observation { at: t0, ..look(&pts, [0.0; 3], 1.5) });
        assert!(nav.world_frame && nav.fixed() && nav.yaw == 90.0);
        assert!((nav.pose[0] - 10.0).abs() < 1e-3 && (nav.pose[2] - 20.0).abs() < 1e-3, "{:?}", nav.pose);
        // The wall 3 m ahead in the session is 3 m along +x in the world.
        let s = nav.map.surfaces(cell_of(13.05, 20.05));
        assert!(s.iter().any(|s| (s.h - 4.5).abs() < 0.1), "{s:?}");
        // The odometry walks on from there; a later fix puts it right.
        nav.advance(t0 + Duration::from_millis(100), [0.0, 0.0, -1.0], true, 0);
        assert!((nav.pose[0] - 11.0).abs() < 1e-3, "{:?}", nav.pose);
        nav.fix(t0 + Duration::from_millis(100), [10.9, 2.0, 20.0], 90.0, 0.0);
        assert!((nav.pose[0] - 10.9).abs() < 1e-3);
        // Long after the reading, standing still since: still usable;
        // having walked on, not.
        nav.last_fix = Some(t0 - Duration::from_secs(60));
        assert!(nav.ready() && nav.writable());
        nav.advance(t0 + Duration::from_millis(200), [0.0, 0.0, -0.3], true, 0);
        assert!(!nav.ready());
        nav.fix(t0 + Duration::from_millis(300), [11.2, 2.0, 20.0], 90.0, 0.0);
        // Saved and loaded in another visit: the beacon puts it straight on.
        let saved = (nav.meta(1), nav.map.clone());
        let mut next = Nav::new("w", WorldMap::default());
        assert_eq!(next.begin("w", "s2", Some(saved), 45.0), Begun::Placing);
        next.fix(Instant::now(), [5.0, 2.0, 20.0], 90.0, 45.0);
        assert!(next.placing.is_none() && next.world_frame && !next.replaced && next.yaw == 45.0);
        assert!(next.map.surfaces(cell_of(13.05, 20.05)).iter().any(|s| (s.h - 4.5).abs() < 0.1));
    }

    #[test]
    fn landmarks_by_name_and_glass_across_the_way() {
        let mut nav = Nav { beacon_only: false, ..Nav::new("w", WorldMap::default()) };
        nav.map.places.push(crate::Place { name: "entrance".into(), at: [5.0, 0.0, 0.0], heading: Some(90.0) });
        nav.map.saw_object("couch", [2.0, 0.0, 0.0], [1.6, 0.8], 0.8, 1);
        nav.map.saw_object("couch", [-3.0, 0.0, 0.0], [1.6, 0.8], 0.8, 1);
        nav.map.saw_object("clock", [1.0, 2.0, 0.0], [0.3, 0.3], 0.3, 1); // not confirmed
        let names: Vec<String> = nav.landmarks().into_iter().map(|l| l.0).collect();
        assert_eq!(names, ["couch", "couch 2", "entrance"]);
        assert_eq!(nav.find("沙发").unwrap().1, [2.0, 0.0, 0.0]);
        assert_eq!(nav.find("沙发 2").unwrap().1, [-3.0, 0.0, 0.0]);
        assert_eq!(nav.find("Entrance").unwrap().0, "entrance");
        assert_eq!(nav.find("entrance").unwrap().2, Some(90.0));
        assert!(nav.find("clock").is_none());
        // Stopped going ahead (-z): a pane across the way is shut, straight
        // through it and a little aside; round its end is not.
        nav.stopped(0.0, 0.4, MarkKind::Blocked, 100);
        assert_eq!(nav.map.marks.len(), 11, "nothing seen either side: 0.75 m each way");
        assert!(nav.map.crosses_shut([0.0, 0.0, 0.0], [0.0, 0.0, -2.0], 100));
        assert!(nav.map.crosses_shut([0.3, 0.0, 0.0], [0.3, 0.0, -2.0], 100));
        assert!(!nav.map.crosses_shut([1.5, 0.0, 0.0], [1.5, 0.0, -2.0], 100));
        assert!(nav.map.crosses_shut([0.7, 0.0, 0.0], [0.7, 0.0, -2.0], 100));
        assert!(nav.map.crosses_shut([0.0, 0.0, -2.0], [0.0, 0.0, 0.0], 100), "glass both ways");
        assert!(nav.map.crosses_shut([1.0, 0.0, 0.3], [-0.5, 0.0, -1.2], 100), "and at a slant");
    }

    /// The pose jumping across a pane (a respawn, a misread beacon, a fix
    /// putting the odometry right) is no walk through it: its marks stay.
    /// Walking through it clears them.
    #[test]
    fn a_jump_of_the_pose_across_glass_keeps_its_marks() {
        let mut nav = Nav { beacon_only: false, ..Nav::new("w", WorldMap::default()) };
        let t0 = Instant::now();
        nav.advance(t0, [0.0; 3], true, 100);
        nav.stopped(0.0, 0.4, MarkKind::Blocked, 100);
        let marks = nav.map.marks.len();
        assert!(marks > 0);
        // Two metres through the glass at once.
        nav.set(t0 + Duration::from_millis(100), [0.0, 0.0, -2.0], true, 100);
        assert_eq!(nav.map.marks.len(), marks, "a jump is no walk");
        // Back again, step by step: walked through.
        for i in 1..=20 {
            nav.advance(t0 + Duration::from_millis(100 + 40 * i), [0.0, 0.0, 0.1], true, 100);
        }
        assert!(nav.pose[2].abs() < 1e-3, "{:?}", nav.pose);
        assert!(nav.map.marks.len() < marks, "walked through: cleared");
    }

    #[test]
    fn a_pane_is_marked_out_to_its_frames() {
        // Glass 0.45 m ahead (-z) between two pillars at x = -2 and x = 1.2.
        let mut pts = Vec::new();
        floor(&mut pts, (-3.0, 3.0), (-3.0, 1.0), 0.0);
        for x in [-2.0f32, 1.2] {
            wall_x(&mut pts, (x - 0.1, x + 0.1), -0.45, 0.0, 2.5);
        }
        let mut nav = Nav { beacon_only: false, ..Nav::new("w", WorldMap::default()) };
        // (No lines of sight: these points do not hide what is behind them.)
        let obs = Observation { rays_every: 0, ..look(&pts, [0.0, 0.0, 2.0], 1.5) };
        nav.map.integrate(&obs, [0.0, 0.0, 2.0], 0.0);
        nav.stopped(0.0, 0.45, MarkKind::Blocked, 100);
        let xs: Vec<f32> = nav.map.marks.iter().map(|m| m.at[0]).collect();
        let (lo, hi) = (xs.iter().copied().fold(f32::INFINITY, f32::min), xs.iter().copied().fold(f32::NEG_INFINITY, f32::max));
        assert!(lo < -1.6 && lo > -2.0 && hi > 0.8 && hi < 1.2, "{lo} {hi}");
        // The way through anywhere along it is shut; round a pillar it is not.
        assert!(nav.map.crosses_shut([-1.5, 0.0, 0.5], [-1.5, 0.0, -1.5], 100));
        assert!(nav.map.crosses_shut([0.9, 0.0, 0.5], [0.9, 0.0, -1.5], 100));
        assert!(!nav.map.crosses_shut([2.0, 0.0, 0.5], [2.0, 0.0, -1.5], 100));
    }

    #[test]
    fn headings_and_marks_in_a_turned_session() {
        let mut nav = Nav { beacon_only: false, ..Nav::new("w", WorldMap::default()) };
        nav.yaw = 90.0;
        assert_eq!(nav.map_heading(0.0), 90.0);
        assert_eq!(nav.session_heading(90.0), 0.0);
        // Ahead in the session is +x on the map.
        nav.stopped(0.0, 0.4, MarkKind::Blocked, 0);
        let m = nav.map.marks[5];
        assert!((m.at[0] - 0.4).abs() < 1e-4 && m.at[2].abs() < 1e-4 && m.yaw == 90.0, "{m:?}");
    }
}
