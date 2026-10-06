//! The bot's lasting map of a world, built as it walks.
//!
//! Two layers, in world metres (the map's own frame, see [`nav`]):
//!
//! - **Seen**: stereo points fused into columns of voxels (10 cm across,
//!   5 cm high), each with how many points hit it (weighted by how near
//!   they were seen: stereo's error grows with the square of the distance)
//!   and how often a line of sight passed through it (a player who walked
//!   off, a stray match: cleared). A column's runs of filled voxels give its
//!   **surfaces**, each one a height something stands on with how much room
//!   is over it: a floor, a stair's tread, a table top, the floor upstairs
//!   and the floor under it. Stairs, ramps and floors over floors are
//!   surfaces of neighbouring columns at heights a step apart, not a special
//!   case.
//! - **Learned** (walking says it, sight does not): where the bot's feet
//!   went (a way that goes: walked again first), where players were seen
//!   standing (a weaker yes), and **marks** where a walk that looked open was
//!   stopped (glass, an invisible wall: that way, there, is shut) or a jump
//!   fell short. Marks fade: shut for a while, then doubtful (tried again
//!   when the way round is long), then gone; walking through one clears it.
//!
//! Also **objects** (a sofa, a door...: a detector's boxes placed by
//! stereo), merged by kind and place, and named **places**.
//!
//! [`plan`] walks the surfaces (and what was walked) by A*, [`register`]
//! keeps the odometry on the map, [`store`] keeps a map per world on disk.

pub mod nav;
pub mod plan;
pub mod register;
pub mod render;
pub mod store;

use std::collections::HashMap;
use std::time::Instant;

pub use nav::{Nav, Shared};
pub use plan::{Path, PlanParams};
pub use register::Fit;

/// A column's side (world metres).
pub const CELL: f32 = 0.1;
/// A voxel's height.
pub const VOXEL: f32 = 0.05;
/// Columns a chunk side.
const CHUNK: i32 = 16;
/// A voxel is filled with this many points at least...
const MIN_POINTS: u16 = 3;
/// ...and this share of hits among hits and passes.
const FILLED_SHARE: f32 = 0.4;
/// Runs of filled voxels this many voxels apart (or nearer) are one.
const RUN_GAP: i16 = 2;
/// Runs weaker than this share of their column's strongest are strays.
const RUN_SHARE: f32 = 0.25;
/// Nearer than this (horizontally, world metres) to the eyes is the bot's
/// own body and what it carries: left out (seen from farther, it is in).
pub const SELF_RADIUS: f32 = 0.9;
/// Points farther than this are too coarse to place (stereo's error grows
/// with the square of the distance: some 0.5 m at 5 m, as first measured;
/// nearer looks fill in what lies farther as the bot walks).
pub const MAX_RANGE: f32 = 5.0;
/// Points this far over the eyes are left out (a ceiling far up); nearer,
/// they are what is over a floor (the underside of the stairs, a low
/// ceiling).
const OVER_EYES: f32 = 1.0;
/// A point's weight is (WEIGHT_NEAR / range)², within WEIGHT_MIN..1.
const WEIGHT_NEAR: f32 = 2.0;
const WEIGHT_MIN: f32 = 0.1;
/// A line of sight clears voxels it passes up to this far short of its end
/// (and this share of its length): grazing the floor, it is close to it.
const CLEAR_SHORT: f32 = 0.5;
const CLEAR_SHORT_SHARE: f32 = 0.1;
/// Steps along a line of sight.
const RAY_STEP: f32 = 0.1;
/// Players' bodies: within this of their feet, up to PERSON_TALL.
const PERSON_RADIUS: f32 = 0.45;
const PERSON_TALL: f32 = 2.5;

/// Seconds since the Unix epoch (marks and walks keep when).
pub fn unix_now() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

/// A vector of the session's axes turned into the map's: `yaw` degrees,
/// the bot's convention (+ right of -z, as headings go).
pub fn turn(v: [f32; 3], yaw: f32) -> [f32; 3] {
    let (s, c) = yaw.to_radians().sin_cos();
    [v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c]
}

/// Degrees wrapped to -180..180.
pub fn wrap(deg: f32) -> f32 {
    (deg + 540.0).rem_euclid(360.0) - 180.0
}

/// The heading (degrees, + right of -z) from `a` to `b` (x, z).
pub fn heading(a: [f32; 2], b: [f32; 2]) -> f32 {
    (b[0] - a[0]).atan2(-(b[1] - a[1])).to_degrees()
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Voxel {
    /// Height index: the voxel spans iy * VOXEL ..+ VOXEL.
    pub iy: i16,
    /// Points that hit it.
    pub n: u16,
    /// Their weight, and the weighted sums of where they were (across:
    /// where in the column a wall stands, finer than the column).
    pub hit: f32,
    pub sum_x: f32,
    pub sum_y: f32,
    pub sum_z: f32,
    /// Weight of the lines of sight that passed through it.
    pub miss: f32,
}

impl Voxel {
    pub fn filled(&self) -> bool {
        self.n >= MIN_POINTS && self.hit >= FILLED_SHARE * (self.hit + self.miss)
    }

    /// The height of what is in it (the mean of its points).
    pub fn y(&self) -> f32 {
        if self.hit > 0.0 {
            self.sum_y / self.hit
        } else {
            (self.iy as f32 + 0.5) * VOXEL
        }
    }
}

/// Feet seen at a height in a column: the bot's own (it walked there) or
/// players'.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Walk {
    pub iy: i16,
    pub own: u16,
    pub others: u16,
    /// When last (Unix seconds).
    pub last: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Column {
    /// By height, lowest first.
    pub voxels: Vec<Voxel>,
    pub walks: Vec<Walk>,
}

/// A run of filled voxels in a column.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Run {
    /// Its bottom (the lowest voxel's floor), and its top (the highest
    /// voxel's points' mean height).
    pub bottom: f32,
    pub top: f32,
    /// Its points' weight.
    pub weight: f32,
    /// Its first and last voxel (indices into the column's).
    pub first: usize,
    pub last: usize,
}

/// A height something stands on, in a column.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Surface {
    pub h: f32,
    /// Free height over it (to the next filled voxel; infinite if none).
    pub room: f32,
    /// Seen (a run of filled voxels); else only walked.
    pub seen: bool,
    pub own: u16,
    pub others: u16,
}

impl Column {
    fn voxel_mut(&mut self, iy: i16) -> &mut Voxel {
        let k = match self.voxels.binary_search_by_key(&iy, |v| v.iy) {
            Ok(k) => k,
            Err(k) => {
                self.voxels.insert(k, Voxel { iy, ..Default::default() });
                k
            }
        };
        &mut self.voxels[k]
    }

    /// Its runs of filled voxels (near enough to be one thing: a floor, a
    /// wall, a table's top), lowest first; runs far weaker than the
    /// column's strongest are stereo's strays (a floor's points scatter a
    /// little up and down, and a few land together now and then), left out.
    pub fn runs(&self) -> Vec<Run> {
        let mut runs: Vec<Run> = Vec::new();
        for (i, v) in self.voxels.iter().enumerate() {
            if !v.filled() {
                continue;
            }
            match runs.last_mut() {
                Some(r) if v.iy - self.voxels[r.last].iy <= RUN_GAP => {
                    r.last = i;
                    r.top = v.y();
                    r.weight += v.hit;
                }
                _ => runs.push(Run { bottom: v.iy as f32 * VOXEL, top: v.y(), weight: v.hit, first: i, last: i }),
            }
        }
        let strongest = runs.iter().map(|r| r.weight).fold(0.0f32, f32::max);
        runs.retain(|r| r.weight >= RUN_SHARE * strongest);
        runs
    }

    /// The filled voxels of its runs between heights `lo` and `hi`.
    fn voxels_between(&self, lo: f32, hi: f32) -> impl Iterator<Item = &Voxel> + '_ {
        let runs = self.runs();
        runs.into_iter()
            .flat_map(move |r| self.voxels[r.first..=r.last].iter())
            .filter(move |v| v.filled() && v.y() > lo && v.y() < hi)
    }

    /// Where (x, z) what lies between heights `lo` and `hi` is, on the
    /// whole (its points' weighted mean); `None` without anything.
    pub fn mean_between(&self, lo: f32, hi: f32) -> Option<[f32; 2]> {
        let (mut w, mut x, mut z) = (0.0f32, 0.0f32, 0.0f32);
        for v in self.voxels_between(lo, hi) {
            (w, x, z) = (w + v.hit, x + v.sum_x, z + v.sum_z);
        }
        (w > 0.0).then(|| [x / w, z / w])
    }

    /// Whether something lies between heights `lo` and `hi`.
    pub fn filled_between(&self, lo: f32, hi: f32) -> bool {
        self.voxels_between(lo, hi).next().is_some()
    }

    /// Its surfaces, lowest first: the tops of its runs, and heights only
    /// walked (glass, a floor never seen: feet went there).
    pub fn surfaces(&self) -> Vec<Surface> {
        let runs = self.runs();
        let mut out: Vec<Surface> = Vec::with_capacity(runs.len());
        for (k, r) in runs.iter().enumerate() {
            let h = r.top;
            let room = runs.get(k + 1).map_or(f32::INFINITY, |next| next.bottom - h);
            out.push(Surface { h, room, seen: true, own: 0, others: 0 });
        }
        for w in &self.walks {
            let h = w.iy as f32 * VOXEL;
            match out.iter_mut().filter(|s| (s.h - h).abs() <= 0.2).min_by(|a, b| (a.h - h).abs().total_cmp(&(b.h - h).abs())) {
                Some(s) => {
                    s.own = s.own.saturating_add(w.own);
                    s.others = s.others.saturating_add(w.others);
                }
                None => out.push(Surface { h, room: f32::INFINITY, seen: false, own: w.own, others: w.others }),
            }
        }
        out.sort_by(|a, b| a.h.total_cmp(&b.h));
        out
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Chunk {
    pub cols: Vec<Column>,
}

/// What a mark says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarkKind {
    /// A walk was stopped here, going this way: shut, though it looked open.
    Blocked,
    /// A jump up here, this way, fell short.
    JumpFailed,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mark {
    /// Where the way is shut (the feet's height).
    pub at: [f32; 3],
    /// The way that was tried (degrees, map axes).
    pub yaw: f32,
    pub kind: MarkKind,
    /// Times it happened, and when last (Unix seconds).
    pub count: u16,
    pub last: u64,
}

/// How much a mark still says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Belief {
    Shut,
    Doubtful,
    Gone,
}

/// Shut for a day for each time it happened (at most a fortnight), then
/// doubtful for a month.
const SHUT_FOR: u64 = 24 * 3600;
const SHUT_MAX: u64 = 14 * 24 * 3600;
const DOUBT_FOR: u64 = 30 * 24 * 3600;

impl Mark {
    pub fn belief(&self, now: u64) -> Belief {
        let age = now.saturating_sub(self.last);
        let shut = (SHUT_FOR * self.count.max(1) as u64).min(SHUT_MAX);
        if age < shut {
            Belief::Shut
        } else if age < shut + DOUBT_FOR {
            Belief::Doubtful
        } else {
            Belief::Gone
        }
    }

    /// Whether going from `a` to `b` (feet heights) goes through it.
    ///
    /// A stop (`Blocked`) is a piece of a pane: a short stretch of wall
    /// across the way that was tried (PANE_HALF each side of `at`; a row of
    /// them, PANE_STEP apart, is one pane), shut both ways and from any
    /// angle (glass is; the user: "眼前的坐标已经被标记为墙，为什么还会往前走"
    /// when the marks shut only ways within 70 degrees of the one tried).
    /// Along it, not through it, is open. A jump that fell short shuts that
    /// way up there (near it, in about its direction).
    pub fn shuts(&self, a: [f32; 3], b: [f32; 3]) -> bool {
        // Another floor (over or under it), not this one. Wide: a mark is
        // at the feet's height as the odometry or the beacon had it, a way
        // at the floor's as seen (they differed by 0.3-0.4 m: the planner
        // took a way the legs' own check called shut, and the bot stood).
        if (a[1] - self.at[1]).abs() >= MARK_LEVEL {
            return false;
        }
        if self.kind == MarkKind::JumpFailed {
            let mid = [(a[0] + b[0]) / 2.0, (a[2] + b[2]) / 2.0];
            let near = (mid[0] - self.at[0]).hypot(mid[1] - self.at[2]) < MARK_RADIUS;
            return near && wrap(heading([a[0], a[2]], [b[0], b[2]]) - self.yaw).abs() < MARK_SPREAD_DEG;
        }
        let (s, c) = self.yaw.to_radians().sin_cos();
        // Ahead (the way tried) and across (along the pane).
        let ahead = |p: [f32; 3]| (p[0] - self.at[0]) * s - (p[2] - self.at[2]) * c;
        let across = |p: [f32; 3]| (p[0] - self.at[0]) * c + (p[2] - self.at[2]) * s;
        let (da, db) = (ahead(a), ahead(b));
        if (da > 0.0 && db > 0.0) || (da < 0.0 && db < 0.0) || (da == db) {
            return false; // one side of it, or along it
        }
        let t = da / (da - db);
        let p = [a[0] + (b[0] - a[0]) * t, a[1], a[2] + (b[2] - a[2]) * t];
        across(p).abs() <= PANE_HALF
    }
}

/// A stop's piece of pane reaches this far each side of it (a row PANE_STEP
/// apart joins up, with a little to spare).
pub const PANE_HALF: f32 = 0.1;
/// A mark is on the floor within this height of it.
pub const MARK_LEVEL: f32 = 1.0;

/// A jump's mark shuts ways within this of it (world metres), within this
/// of its direction (degrees).
pub const MARK_RADIUS: f32 = 0.35;
pub const MARK_SPREAD_DEG: f32 = 70.0;
/// A mark this near one of its kind (and way) is that one again: under the
/// 0.15 m a row across a pane is marked at.
const SAME_MARK_M: f32 = 0.12;

/// Something a detector found, placed.
#[derive(Clone, Debug, PartialEq)]
pub struct Object {
    pub label: String,
    /// Where (its middle at the floor, map frame), the mean of its sightings.
    pub at: [f32; 3],
    /// Its size across and up (world metres), the largest seen.
    pub size: [f32; 2],
    pub seen: u16,
    pub score: f32,
    pub last: u64,
    /// The sum of squared distances (across) of its sightings from their
    /// running mean (Welford): how well it is placed.
    pub m2: f32,
    /// Its sightings by kind (a group's kinds: a sofa seen as a bed now and
    /// then); `label` is the one seen most.
    pub kinds: Vec<(String, u16)>,
}

impl Object {
    /// Seen twice or more, or once and surely: not a stray detection.
    pub fn confirmed(&self) -> bool {
        self.seen >= 2 || self.score >= 0.5
    }

    /// How far its sightings scatter round where it is (metres, across).
    pub fn spread(&self) -> f32 {
        if self.seen > 1 {
            (self.m2 / self.seen as f32).sqrt()
        } else {
            0.0
        }
    }
}

/// Sightings of one kind nearer than this (or within the object's size)
/// are the same object.
const SAME_OBJECT_M: f32 = 0.8;
/// Kinds a detector mixes up for one thing (a sofa seen as a bed from one
/// side, a bench from another): sightings of a group nearer than this are
/// the same object, its kind the one seen most.
const SAME_GROUP_M: f32 = 0.6;
const GROUPS: &[&[&str]] = &[&["couch", "bed", "bench", "chair"], &["dining table", "desk", "coffee table"], &["tv", "laptop"]];

/// What a thing's kind (COCO's) is called in Chinese, for names given that
/// way ("去沙发那").
const CHINESE: &[(&str, &[&str])] = &[
    ("couch", &["沙发"]),
    ("chair", &["椅子", "凳子"]),
    ("bed", &["床"]),
    ("dining table", &["桌子", "餐桌", "茶几", "桌"]),
    ("tv", &["电视", "电视机", "屏幕"]),
    ("potted plant", &["盆栽", "植物", "花"]),
    ("clock", &["钟", "时钟", "挂钟"]),
    ("vase", &["花瓶"]),
    ("refrigerator", &["冰箱"]),
    ("sink", &["水槽", "洗手池"]),
    ("toilet", &["马桶", "厕所"]),
    ("bench", &["长凳", "长椅"]),
    ("laptop", &["电脑", "笔记本"]),
    ("microwave", &["微波炉"]),
    ("oven", &["烤箱"]),
];

/// The kind a name means: itself (an English kind), or by its Chinese.
pub fn label_of(name: &str) -> Option<&'static str> {
    let name = name.trim();
    CHINESE.iter().find(|(label, zh)| *label == name || zh.contains(&name)).map(|(label, _)| *label)
}

fn group(label: &str) -> Option<usize> {
    GROUPS.iter().position(|g| g.contains(&label))
}

/// A place the model named ("the sofa", "the stage").
#[derive(Clone, Debug, PartialEq)]
pub struct Place {
    pub name: String,
    pub at: [f32; 3],
    /// The way the bot faced there (map heading), to face again.
    pub heading: Option<f32>,
}

/// What one look saw, in the session's axes, world metres, from the feet
/// (the avatar's position on its floor) when it was taken.
#[derive(Clone, Debug)]
pub struct Observation {
    pub points: Vec<[f32; 3]>,
    pub eye: [f32; 3],
    /// Players' feet: their bodies are left out.
    pub people: Vec<[f32; 3]>,
    /// One line of sight cleared for every this many points (0: none).
    pub rays_every: usize,
    /// When it was taken (its pose: the odometry then).
    pub at: Instant,
}

impl Observation {
    /// From tracking-space points (stereo units) of a frame: `eye` the
    /// eyes' middle, `floor` the floor's height there, `metres` world metres
    /// per unit.
    pub fn from_tracking(points: &[[f32; 3]], eye: [f32; 3], floor: f32, metres: f32, people: &[[f32; 3]], at: Instant) -> Observation {
        let rel = |p: &[f32; 3]| [(p[0] - eye[0]) * metres, (p[1] - floor) * metres, (p[2] - eye[2]) * metres];
        Observation {
            points: points.iter().map(rel).collect(),
            eye: [0.0, (eye[1] - floor) * metres, 0.0],
            people: people.iter().map(rel).collect(),
            rays_every: 8,
            at,
        }
    }

    /// Whether `p` (this frame) is the bot's own body, a player's or out of reach.
    fn keeps(&self, p: [f32; 3]) -> bool {
        let r = p[0].hypot(p[2]);
        if r < SELF_RADIUS || r > MAX_RANGE || p[1] > self.eye[1] + OVER_EYES {
            return false;
        }
        !self.people.iter().any(|f| (p[0] - f[0]).hypot(p[2] - f[2]) < PERSON_RADIUS && p[1] > f[1] - 0.2 && p[1] < f[1] + PERSON_TALL)
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct WorldMap {
    pub chunks: HashMap<(i32, i32), Chunk>,
    pub marks: Vec<Mark>,
    pub objects: Vec<Object>,
    pub places: Vec<Place>,
    /// Changed since saved.
    pub dirty: bool,
}

/// The column holding (x, z).
pub fn cell_of(x: f32, z: f32) -> (i32, i32) {
    ((x / CELL).floor() as i32, (z / CELL).floor() as i32)
}

/// The middle of a column.
pub fn centre(c: (i32, i32)) -> [f32; 2] {
    [(c.0 as f32 + 0.5) * CELL, (c.1 as f32 + 0.5) * CELL]
}

fn split(c: (i32, i32)) -> ((i32, i32), usize) {
    let key = (c.0.div_euclid(CHUNK), c.1.div_euclid(CHUNK));
    (key, (c.1.rem_euclid(CHUNK) * CHUNK + c.0.rem_euclid(CHUNK)) as usize)
}

impl WorldMap {
    pub fn column(&self, c: (i32, i32)) -> Option<&Column> {
        let (key, k) = split(c);
        self.chunks.get(&key).map(|ch| &ch.cols[k])
    }

    pub fn column_mut(&mut self, c: (i32, i32)) -> &mut Column {
        let (key, k) = split(c);
        let ch = self.chunks.entry(key).or_insert_with(|| Chunk { cols: vec![Column::default(); (CHUNK * CHUNK) as usize] });
        &mut ch.cols[k]
    }

    /// Every column with something in it.
    pub fn columns(&self) -> impl Iterator<Item = ((i32, i32), &Column)> {
        self.chunks.iter().flat_map(|(key, ch)| {
            ch.cols.iter().enumerate().filter(|(_, c)| !c.voxels.is_empty() || !c.walks.is_empty()).map(move |(k, c)| {
                let k = k as i32;
                ((key.0 * CHUNK + k % CHUNK, key.1 * CHUNK + k / CHUNK), c)
            })
        })
    }

    pub fn surfaces(&self, c: (i32, i32)) -> Vec<Surface> {
        self.column(c).map(Column::surfaces).unwrap_or_default()
    }

    /// Adds a look taken with the feet at `pose` (map frame), the session's
    /// axes turned `yaw` degrees to the map's.
    pub fn integrate(&mut self, obs: &Observation, pose: [f32; 3], yaw: f32) {
        let to_map = |p: [f32; 3]| {
            let q = turn(p, yaw);
            [q[0] + pose[0], q[1] + pose[1], q[2] + pose[2]]
        };
        let eye = to_map(obs.eye);
        let mut kept = 0usize;
        for &p in &obs.points {
            if !obs.keeps(p) {
                continue;
            }
            let r = (p[0] - obs.eye[0]).hypot(p[2] - obs.eye[2]).hypot(p[1] - obs.eye[1]);
            let w = (WEIGHT_NEAR / r.max(0.1)).powi(2).clamp(WEIGHT_MIN, 1.0);
            let q = to_map(p);
            let v = self.column_mut(cell_of(q[0], q[2])).voxel_mut((q[1] / VOXEL).floor() as i16);
            v.n = v.n.saturating_add(1);
            v.hit += w;
            v.sum_x += w * q[0];
            v.sum_y += w * q[1];
            v.sum_z += w * q[2];
            kept += 1;
            if obs.rays_every > 0 && kept % obs.rays_every == 0 {
                self.clear_ray(eye, q, w);
            }
        }
        self.dirty = true;
    }

    /// The line of sight from `eye` to `to` passed through what lies on it
    /// (short of its end).
    fn clear_ray(&mut self, eye: [f32; 3], to: [f32; 3], w: f32) {
        let d = [to[0] - eye[0], to[1] - eye[1], to[2] - eye[2]];
        let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let upto = len - CLEAR_SHORT.max(CLEAR_SHORT_SHARE * len);
        let mut t = SELF_RADIUS;
        let mut last = None;
        while t < upto {
            let f = t / len;
            let p = [eye[0] + d[0] * f, eye[1] + d[1] * f, eye[2] + d[2] * f];
            let (c, iy) = (cell_of(p[0], p[2]), (p[1] / VOXEL).floor() as i16);
            if last != Some((c, iy)) {
                last = Some((c, iy));
                let (key, k) = split(c);
                if let Some(ch) = self.chunks.get_mut(&key) {
                    if let Ok(i) = ch.cols[k].voxels.binary_search_by_key(&iy, |v| v.iy) {
                        ch.cols[k].voxels[i].miss += w;
                    }
                }
            }
            t += RAY_STEP;
        }
    }

    /// Feet at `at`: the bot's own (`own`) or a player's.
    pub fn walked(&mut self, at: [f32; 3], own: bool, now: u64) {
        let iy = (at[1] / VOXEL).round() as i16;
        let col = self.column_mut(cell_of(at[0], at[2]));
        let w = match col.walks.iter_mut().find(|w| (w.iy - iy).abs() <= 4) {
            Some(w) => w,
            None => {
                col.walks.push(Walk { iy, ..Default::default() });
                col.walks.last_mut().unwrap()
            }
        };
        if own {
            w.own = w.own.saturating_add(1);
            // Feet keep to where they really are.
            w.iy = iy;
        } else {
            w.others = w.others.saturating_add(1);
        }
        w.last = now;
        self.dirty = true;
    }

    /// The bot walked from `from` to `to` (feet): a mark across that way
    /// there no longer holds.
    pub fn walked_through(&mut self, from: [f32; 3], to: [f32; 3]) -> usize {
        if (to[0] - from[0]).hypot(to[2] - from[2]) < 0.05 {
            return 0;
        }
        let before = self.marks.len();
        self.marks.retain(|m| !m.shuts(from, to));
        let gone = before - self.marks.len();
        if gone > 0 {
            self.dirty = true;
        }
        gone
    }

    /// Whether going straight from `a` to `b` (feet) runs into a way shut
    /// (a mark believed shut: glass, an invisible wall).
    pub fn crosses_shut(&self, a: [f32; 3], b: [f32; 3], now: u64) -> bool {
        self.shut_along(a, b, now).is_some()
    }

    /// How far along the straight way from `a` to `b` (feet) it first runs
    /// into a way shut, if it does.
    pub fn shut_along(&self, a: [f32; 3], b: [f32; 3], now: u64) -> Option<f32> {
        let len = (b[0] - a[0]).hypot(b[2] - a[2]);
        let steps = (len / CELL).ceil().max(1.0) as usize;
        let at = |t: f32| [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
        let shut: Vec<&Mark> = self.marks.iter().filter(|m| m.kind == MarkKind::Blocked && m.belief(now) == Belief::Shut).collect();
        if shut.is_empty() {
            return None;
        }
        (0..steps).find_map(|i| {
            let (p, q) = (at(i as f32 / steps as f32), at((i + 1) as f32 / steps as f32));
            shut.iter().any(|m| m.shuts(p, q)).then_some(len * i as f32 / steps as f32)
        })
    }

    /// Something stopped a walk at `at` (feet) going `yaw` (map axes).
    pub fn mark(&mut self, at: [f32; 3], yaw: f32, kind: MarkKind, now: u64) {
        match self.marks.iter_mut().find(|m| {
            m.kind == kind && (m.at[0] - at[0]).hypot(m.at[2] - at[2]) < SAME_MARK_M && (m.at[1] - at[1]).abs() < 0.3 && wrap(m.yaw - yaw).abs() < 45.0
        }) {
            Some(m) => {
                m.count = m.count.saturating_add(1);
                m.last = now;
            }
            None => self.marks.push(Mark { at, yaw, kind, count: 1, last: now }),
        }
        self.dirty = true;
    }

    /// A detector's sighting of `label` at `at` (map frame, its foot),
    /// `size` across and up.
    pub fn saw_object(&mut self, label: &str, at: [f32; 3], size: [f32; 2], score: f32, now: u64) {
        let g = group(label);
        let same = self.objects.iter_mut().find(|o| {
            let d = (o.at[0] - at[0]).hypot(o.at[2] - at[2]);
            let level = (o.at[1] - at[1]).abs() < 1.0;
            let kin = o.label == label || o.kinds.iter().any(|k| k.0 == label);
            let reach = SAME_OBJECT_M.max(0.5 * o.size[0].max(size[0]));
            level && ((kin && d < reach) || (g.is_some() && group(&o.label) == g && d < SAME_GROUP_M))
        });
        match same {
            Some(o) => {
                let n = o.seen as f32;
                let before = (at[0] - o.at[0]).hypot(at[2] - o.at[2]);
                for k in 0..3 {
                    o.at[k] = (o.at[k] * n + at[k]) / (n + 1.0);
                }
                o.m2 += before * (at[0] - o.at[0]).hypot(at[2] - o.at[2]);
                o.size = [o.size[0].max(size[0]), o.size[1].max(size[1])];
                o.score = o.score.max(score);
                o.seen = o.seen.saturating_add(1);
                o.last = now;
                match o.kinds.iter_mut().find(|k| k.0 == label) {
                    Some(k) => k.1 = k.1.saturating_add(1),
                    None => o.kinds.push((label.to_string(), 1)),
                }
                if let Some(top) = o.kinds.iter().max_by_key(|k| k.1) {
                    o.label = top.0.clone();
                }
            }
            None => self.objects.push(Object { label: label.to_string(), at, size, seen: 1, score, last: now, m2: 0.0, kinds: vec![(label.to_string(), 1)] }),
        }
        self.dirty = true;
    }

    /// The map moved: what was at `from` is at `to`, turned `turned` degrees
    /// round it (the world's own frame found: everything goes over to it).
    pub fn transformed(&self, from: [f32; 3], to: [f32; 3], turned: f32) -> WorldMap {
        let go = |p: [f32; 3]| {
            let q = turn([p[0] - from[0], p[1] - from[1], p[2] - from[2]], turned);
            [q[0] + to[0], q[1] + to[1], q[2] + to[2]]
        };
        let mut out = WorldMap { dirty: true, ..Default::default() };
        for (c, col) in self.columns() {
            let middle = centre(c);
            for v in &col.voxels {
                let (x, z) = if v.hit > 0.0 { (v.sum_x / v.hit, v.sum_z / v.hit) } else { (middle[0], middle[1]) };
                let p = go([x, v.y(), z]);
                let w = out.column_mut(cell_of(p[0], p[2])).voxel_mut((p[1] / VOXEL).floor() as i16);
                w.n = w.n.saturating_add(v.n);
                w.hit += v.hit;
                w.miss += v.miss;
                w.sum_x += v.hit * p[0];
                w.sum_y += v.hit * p[1];
                w.sum_z += v.hit * p[2];
            }
            for k in &col.walks {
                let p = go([middle[0], k.iy as f32 * VOXEL, middle[1]]);
                let iy = (p[1] / VOXEL).round() as i16;
                let dst = out.column_mut(cell_of(p[0], p[2]));
                match dst.walks.iter_mut().find(|w| (w.iy - iy).abs() <= 4) {
                    Some(w) => {
                        w.own = w.own.saturating_add(k.own);
                        w.others = w.others.saturating_add(k.others);
                        w.last = w.last.max(k.last);
                    }
                    None => dst.walks.push(Walk { iy, ..*k }),
                }
            }
        }
        out.marks = self.marks.iter().map(|m| Mark { at: go(m.at), yaw: wrap(m.yaw + turned), ..*m }).collect();
        out.objects = self.objects.iter().map(|o| Object { at: go(o.at), ..o.clone() }).collect();
        out.places = self.places.iter().map(|p| Place { at: go(p.at), heading: p.heading.map(|h| wrap(h + turned)), ..p.clone() }).collect();
        out
    }

    /// How much is in it: columns, filled voxels, walked columns.
    pub fn census(&self) -> (usize, usize, usize) {
        let (mut cols, mut filled, mut walked) = (0, 0, 0);
        for (_, c) in self.columns() {
            cols += 1;
            filled += c.voxels.iter().filter(|v| v.filled()).count();
            walked += (!c.walks.is_empty()) as usize;
        }
        (cols, filled, walked)
    }
}

/// Distance from `p` to the segment `a`-`b` (x, z).
pub fn point_segment(p: [f32; 2], a: [f32; 2], b: [f32; 2]) -> f32 {
    let (dx, dz) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dz * dz;
    let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dz) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dz)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Points of a scene (map frame) seen from `eye`, as an observation
    /// taken with the feet at `feet` (session axes = map axes).
    pub fn look(points: &[[f32; 3]], feet: [f32; 3], eye_h: f32) -> Observation {
        Observation {
            points: points.iter().map(|p| [p[0] - feet[0], p[1] - feet[1], p[2] - feet[2]]).collect(),
            eye: [0.0, eye_h, 0.0],
            people: Vec::new(),
            rays_every: 4,
            at: Instant::now(),
        }
    }

    /// A floor at height `y` over x0..x1, z0..z1, 4 points a column.
    pub fn floor(pts: &mut Vec<[f32; 3]>, x: (f32, f32), z: (f32, f32), y: f32) {
        let mut a = x.0 + 0.025;
        while a < x.1 {
            let mut b = z.0 + 0.025;
            while b < z.1 {
                pts.push([a, y, b]);
                b += 0.05;
            }
            a += 0.05;
        }
    }

    /// A wall along x at `z`, from x0 to x1, from `base` up to `top`
    /// (2.5 cm apart, as densely as stereo sees a wall a few metres off).
    pub fn wall_x(pts: &mut Vec<[f32; 3]>, x: (f32, f32), z: f32, base: f32, top: f32) {
        let mut a = x.0 + 0.0125;
        while a < x.1 {
            let mut y = base + 0.0125;
            while y < top {
                pts.push([a, y, z]);
                y += 0.025;
            }
            a += 0.025;
        }
    }

    /// A wall along z at `x`.
    pub fn wall_z(pts: &mut Vec<[f32; 3]>, z: (f32, f32), x: f32, base: f32, top: f32) {
        let mut w = Vec::new();
        wall_x(&mut w, z, 0.0, base, top);
        pts.extend(w.into_iter().map(|p| [x, p[1], p[0]]));
    }

    #[test]
    fn surfaces_of_floor_table_and_upstairs() {
        let mut m = WorldMap::default();
        let mut pts = Vec::new();
        floor(&mut pts, (1.0, 3.0), (-1.0, 1.0), 0.0);
        floor(&mut pts, (1.0, 3.0), (-1.0, 1.0), 0.0); // seen twice
        floor(&mut pts, (2.0, 2.5), (-0.2, 0.2), 0.75); // a table top over it
        m.integrate(&look(&pts, [0.0; 3], 1.6), [0.0; 3], 0.0);
        // The floor upstairs, seen from up there.
        let mut up = Vec::new();
        floor(&mut up, (2.5, 3.0), (-0.2, 0.2), 3.0);
        m.integrate(&look(&up, [1.0, 3.0, 0.0], 1.6), [1.0, 3.0, 0.0], 0.0);
        let s = m.surfaces(cell_of(1.55, 0.05));
        assert_eq!(s.len(), 1);
        assert!(s[0].h.abs() < 0.03 && s[0].room.is_infinite(), "{s:?}");
        let s = m.surfaces(cell_of(2.25, 0.05));
        assert_eq!(s.len(), 2, "{s:?}");
        assert!((s[0].room - 0.75).abs() < 0.06 && (s[1].h - 0.75).abs() < 0.03, "{s:?}");
        let s = m.surfaces(cell_of(2.75, 0.05));
        assert!(s.len() == 2 && (s[1].h - 3.0).abs() < 0.03 && s[0].room > 2.9, "{s:?}");
        // Own body: not mapped.
        assert!(m.column(cell_of(0.3, 0.0)).is_none_or(|c| c.voxels.is_empty()));
    }

    #[test]
    fn lines_of_sight_clear_what_left() {
        let mut m = WorldMap::default();
        // Someone stood 2 m ahead (a post of points), then left: the floor
        // beyond is seen through where they were.
        let mut post = Vec::new();
        let mut y = 0.025;
        while y < 1.6 {
            for _ in 0..4 {
                post.push([0.05, y, -2.05]);
            }
            y += 0.05;
        }
        m.integrate(&look(&post, [0.0; 3], 1.6), [0.0; 3], 0.0);
        // (The lines to the floor beyond pass it 0.75-0.95 m up.)
        let filled = |m: &WorldMap| m.column(cell_of(0.05, -2.05)).unwrap().voxels.iter().filter(|v| v.filled() && v.y() > 0.8 && v.y() < 0.9).count();
        assert!(filled(&m) >= 2);
        let mut beyond = Vec::new();
        floor(&mut beyond, (-0.3, 0.4), (-6.0, -4.0), 0.0);
        for _ in 0..6 {
            let mut o = look(&beyond, [0.0; 3], 1.6);
            o.rays_every = 1;
            m.integrate(&o, [0.0; 3], 0.0);
        }
        assert_eq!(filled(&m), 0, "the lines of sight should have cleared it");
    }

    #[test]
    fn walks_and_marks() {
        let mut m = WorldMap::default();
        let now = 1_000_000;
        m.walked([1.0, 0.0, 1.0], true, now);
        m.walked([1.0, 0.02, 1.0], true, now);
        let s = m.surfaces(cell_of(1.0, 1.0));
        assert_eq!(s.len(), 1);
        assert!(!s[0].seen && s[0].own == 2, "{s:?}");
        // Glass 0.35 m ahead, going -z.
        m.mark([0.0, 0.0, -0.35], 0.0, MarkKind::Blocked, now);
        let m0 = m.marks[0];
        assert!(m0.shuts([0.0, 0.0, -0.2], [0.0, 0.0, -0.5]));
        assert!(m0.shuts([0.0, 0.0, -0.5], [0.0, 0.0, -0.2]), "glass is glass both ways");
        assert!(m0.shuts([0.3, 0.0, -0.1], [-0.3, 0.0, -0.6]), "at a slant too");
        assert!(!m0.shuts([-0.3, 0.0, -0.35], [0.3, 0.0, -0.36]), "along it is open");
        assert!(!m0.shuts([1.0, 0.0, -0.2], [1.0, 0.0, -0.5]), "elsewhere");
        assert_eq!(m0.belief(now + 3600), Belief::Shut);
        assert_eq!(m0.belief(now + 3 * 24 * 3600), Belief::Doubtful);
        assert_eq!(m0.belief(now + 60 * 24 * 3600), Belief::Gone);
        // Again there: shut longer.
        m.mark([0.05, 0.0, -0.35], 5.0, MarkKind::Blocked, now);
        assert_eq!(m.marks.len(), 1);
        assert_eq!(m.marks[0].belief(now + 36 * 3600), Belief::Shut);
        // Walking along beside it clears nothing; through it, it goes.
        assert_eq!(m.walked_through([0.5, 0.0, 0.0], [0.5, 0.0, -1.0]), 0);
        assert_eq!(m.walked_through([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]), 1);
    }

    #[test]
    fn sightings_of_one_object_merge() {
        let mut m = WorldMap::default();
        m.saw_object("couch", [2.0, 0.0, 1.0], [1.8, 0.8], 0.7, 1);
        m.saw_object("couch", [2.3, 0.0, 1.2], [1.6, 0.8], 0.8, 2);
        m.saw_object("couch", [6.0, 0.0, 1.0], [1.6, 0.8], 0.8, 3);
        m.saw_object("vase", [2.1, 0.0, 1.0], [0.2, 0.3], 0.6, 3);
        assert_eq!(m.objects.len(), 3);
        let o = &m.objects[0];
        assert!(o.seen == 2 && (o.at[0] - 2.15).abs() < 1e-4 && o.score == 0.8, "{o:?}");
        // Two sightings 0.36 m apart: each 0.18 from the middle.
        assert!((o.spread() - 0.18).abs() < 0.01, "{}", o.spread());
        // A sofa seen as a bed now and then is one thing, a sofa.
        m.saw_object("bed", [2.2, 0.0, 1.1], [1.6, 0.8], 0.5, 4);
        m.saw_object("couch", [2.1, 0.0, 1.1], [1.6, 0.8], 0.8, 5);
        assert_eq!(m.objects.len(), 3, "{:?}", m.objects);
        let o = &m.objects[0];
        assert!(o.label == "couch" && o.seen == 4 && o.kinds.len() == 2, "{o:?}");
    }

    #[test]
    fn turns_follow_headings() {
        // Ahead (-z) turned 90 degrees is to the right (+x), as headings go.
        let v = turn([0.0, 0.0, -1.0], 90.0);
        assert!((v[0] - 1.0).abs() < 1e-5 && v[2].abs() < 1e-5, "{v:?}");
        assert!((heading([0.0, 0.0], [v[0], v[2]]) - 90.0).abs() < 1e-3);
    }
}
