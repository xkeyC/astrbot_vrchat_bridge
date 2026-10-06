//! Ways over the map: A* over its surfaces (a column's floor, a stair's
//! tread, the floor upstairs), from one to a neighbouring column's when the
//! height between them is a step up, a drop down, or (when jumps are
//! allowed) a jump up.
//!
//! A surface is stood on when the bot fits: room over it for its body, and
//! nothing from knee height up to its head within its half width (a wall, a
//! table's top over the floor: not a stair's next treads, they are lower).
//! What walking taught overrules sight: feet went there, so it goes (a sofa
//! without collision, a glass floor); a mark shuts the way it was stopped
//! going, there. Ways walked before cost less: the bot keeps to them.

use std::collections::{BinaryHeap, HashMap};

use crate::{cell_of, centre, heading, Belief, MarkKind, Surface, WorldMap, CELL, SELF_RADIUS};

#[derive(Clone, Copy, Debug)]
pub struct PlanParams {
    /// Highest step up, deepest drop down, highest jump up (world metres).
    pub step: f32,
    pub drop: f32,
    pub jump: f32,
    /// Jumps allowed.
    pub jumps: bool,
    /// Half the body's width, and its height.
    pub radius: f32,
    pub body: f32,
    /// Round the start, the bot's own unseen disc: stood on at its height.
    pub start_radius: f32,
    pub max_expand: usize,
    /// Unix seconds (marks fade).
    pub now: u64,
}

impl Default for PlanParams {
    fn default() -> Self {
        PlanParams {
            step: 0.3,
            drop: 0.8,
            jump: 0.5,
            jumps: false,
            radius: 0.25,
            body: 1.5,
            start_radius: SELF_RADIUS + 0.1,
            max_expand: 400_000,
            now: crate::unix_now(),
        }
    }
}

/// Over the feet, from this height up, something within the body's half
/// width is in the way (lower: a stair's next tread, a kerb).
const KNEE: f32 = 0.5;
/// Cost of a metre: over ground walked before, players walked, seen only.
/// (User: "走通过就知道该怎么走": a way walked once is the way.) Right on
/// the feet's trail, beside it (within TRAIL_NEAR), where players stood.
const OWN_COST: f32 = 0.35;
const NEAR_OWN_COST: f32 = 0.6;
const OTHERS_COST: f32 = 0.85;
/// Beside the trail: within this many columns of it, at about its height.
const TRAIL_NEAR: i32 = 3;
/// Feet went here: within this many columns the body went too (stood on,
/// whatever sight says: a sofa without collision, a doorway's edge).
const TRAIL_BODY: i32 = 1;
const TRAIL_LEVEL: f32 = 0.3;
/// A doubtful mark: its way costs this many times more.
const DOUBT_COST: f32 = 4.0;
/// A jump costs as much as this many metres more.
const JUMP_COST: f32 = 1.5;
/// Searched no farther than this from the start.
const REACH: f32 = 40.0;

/// A way found.
#[derive(Clone, Debug, Default)]
pub struct Path {
    /// From the start (the feet) to the goal (or as near as it goes):
    /// column middles at their surfaces' heights.
    pub points: Vec<[f32; 3]>,
    /// Whether the way into each point is a jump up.
    pub jumps: Vec<bool>,
    /// Its length in metres.
    pub length: f32,
    /// It reaches the goal's column (else it ends as near as it goes).
    pub reached: bool,
    pub expanded: usize,
}

/// A surface: its column and its index there (`VIRTUAL`: the start's
/// unseen disc at the feet's height).
type Key = (i32, i32, u8);
const VIRTUAL: u8 = u8::MAX;

pub struct Planner<'a> {
    pub map: &'a WorldMap,
    pub p: PlanParams,
    start: [f32; 3],
    surfaces: HashMap<(i32, i32), Vec<Surface>>,
    stands: HashMap<Key, bool>,
    /// Trails near columns: (column, reach, height band) -> whether.
    trails: HashMap<(i32, i32, i32, i32), bool>,
}

impl<'a> Planner<'a> {
    pub fn new(map: &'a WorldMap, p: PlanParams, start: [f32; 3]) -> Planner<'a> {
        Planner { map, p, start, surfaces: HashMap::new(), stands: HashMap::new(), trails: HashMap::new() }
    }

    fn surfaces(&mut self, c: (i32, i32)) -> &Vec<Surface> {
        let map = self.map;
        self.surfaces.entry(c).or_insert_with(|| map.surfaces(c))
    }

    fn in_start(&self, c: (i32, i32)) -> bool {
        let m = centre(c);
        (m[0] - self.start[0]).hypot(m[1] - self.start[2]) <= self.p.start_radius
    }

    /// The column's surfaces to stand on, with their keys and heights; in
    /// the start's disc, one at the feet's height unless one is near it.
    fn nodes(&mut self, c: (i32, i32)) -> Vec<(Key, f32, Surface)> {
        let all = self.surfaces(c).clone();
        let mut out = Vec::new();
        for (k, s) in all.iter().enumerate().take(VIRTUAL as usize) {
            if self.stands(c, k as u8, s) {
                out.push(((c.0, c.1, k as u8), s.h, *s));
            }
        }
        if self.in_start(c) && !out.iter().any(|n| (n.1 - self.start[1]).abs() <= self.p.step) {
            let s = Surface { h: self.start[1], room: f32::INFINITY, seen: false, own: 1, others: 0 };
            out.push(((c.0, c.1, VIRTUAL), self.start[1], s));
        }
        out
    }

    /// Whether the bot stands on surface `k` (`s`) of column `c`.
    fn stands(&mut self, c: (i32, i32), k: u8, s: &Surface) -> bool {
        if let Some(&v) = self.stands.get(&(c.0, c.1, k)) {
            return v;
        }
        let v = if s.own > 0 || s.others > 0 || self.trail_within(c, s.h, TRAIL_BODY) {
            true // feet were there (or right beside)
        } else if !s.seen || s.room < self.p.body {
            false
        } else {
            let r = self.p.radius / CELL;
            let k = r.ceil() as i32;
            let (lo, hi) = (s.h + KNEE, s.h + self.p.body);
            !(-k..=k).any(|dz| {
                (-k..=k).any(|dx| {
                    ((dx * dx + dz * dz) as f32) <= r * r
                        && self.map.column((c.0 + dx, c.1 + dz)).is_some_and(|col| col.filled_between(lo, hi))
                })
            })
        };
        self.stands.insert((c.0, c.1, k), v);
        v
    }

    /// Whether the bot's feet went within `k` columns of `c`, at about
    /// height `h`.
    fn trail_within(&mut self, c: (i32, i32), h: f32, k: i32) -> bool {
        let key = (c.0, c.1, k, (h / TRAIL_LEVEL).round() as i32);
        if let Some(&v) = self.trails.get(&key) {
            return v;
        }
        let v = self.trail_near(c, h, k);
        self.trails.insert(key, v);
        v
    }

    fn trail_near(&mut self, c: (i32, i32), h: f32, k: i32) -> bool {
        for dz in -k..=k {
            for dx in -k..=k {
                let n = (c.0 + dx, c.1 + dz);
                if self.surfaces(n).iter().any(|s| s.own > 0 && (s.h - h).abs() < TRAIL_LEVEL) {
                    return true;
                }
            }
        }
        false
    }

    /// What a metre over surface `s` of column `c` costs: least on the
    /// trail, less beside it, less where players stood.
    fn ground_cost(&mut self, c: (i32, i32), s: &Surface) -> f32 {
        if s.own > 0 {
            OWN_COST
        } else if self.trail_within(c, s.h, TRAIL_NEAR) {
            NEAR_OWN_COST
        } else if s.others > 0 {
            OTHERS_COST
        } else {
            1.0
        }
    }

    /// How much more a way from `a` to `b` (feet) costs for the marks
    /// there: `None` when one shuts it.
    fn marked(&self, a: [f32; 3], b: [f32; 3], jump: bool) -> Option<f32> {
        let mut f = 1.0;
        for m in &self.map.marks {
            if (m.kind == MarkKind::JumpFailed && !jump) || !m.shuts(a, b) {
                continue;
            }
            match m.belief(self.p.now) {
                Belief::Shut => return None,
                Belief::Doubtful => f *= DOUBT_COST,
                Belief::Gone => {}
            }
        }
        Some(f)
    }

    /// The way from the start to `goal` (x, z; with a height, the surface
    /// nearest it there).
    pub fn plan(&mut self, goal: [f32; 2], goal_h: Option<f32>) -> Option<Path> {
        let goal_c = cell_of(goal[0], goal[1]);
        let start2 = [self.start[0], self.start[2]];
        let h_of = |d: f32| OWN_COST * d;
        let mut g: HashMap<Key, f32> = HashMap::new();
        let mut came: HashMap<Key, (Key, bool)> = HashMap::new();
        let mut at: HashMap<Key, [f32; 3]> = HashMap::new();
        let mut open = BinaryHeap::new();
        let push = |open: &mut BinaryHeap<Item>, f: f32, k: Key| open.push(Item(f, k));

        // The start: the disc's columns, at the feet's height (or a surface
        // a step from it), reached straight from the feet unless a mark
        // shuts that.
        let k = (self.p.start_radius / CELL).ceil() as i32;
        let sc = cell_of(self.start[0], self.start[2]);
        for dz in -k..=k {
            for dx in -k..=k {
                let c = (sc.0 + dx, sc.1 + dz);
                if !self.in_start(c) {
                    continue;
                }
                let m = centre(c);
                for (key, h, _) in self.nodes(c) {
                    if (h - self.start[1]).abs() > self.p.step {
                        continue;
                    }
                    let p = [m[0], h, m[1]];
                    let Some(f) = self.marked(self.start, p, false) else { continue };
                    let d = (m[0] - start2[0]).hypot(m[1] - start2[1]) * f;
                    if g.get(&key).is_none_or(|&b| d < b) {
                        g.insert(key, d);
                        at.insert(key, p);
                        push(&mut open, d + h_of((m[0] - goal[0]).hypot(m[1] - goal[1])), key);
                    }
                }
            }
        }

        let mut best: Option<(f32, Key)> = None;
        let mut expanded = 0;
        let mut reached = None;
        while let Some(Item(_, key)) = open.pop() {
            let here = at[&key];
            let gk = g[&key];
            let c = (key.0, key.1);
            if c == goal_c && goal_h.is_none_or(|h| (here[1] - h).abs() < 1.0) {
                reached = Some(key);
                break;
            }
            expanded += 1;
            if expanded > self.p.max_expand {
                break;
            }
            let left = (here[0] - goal[0]).hypot(here[2] - goal[1]);
            if best.is_none_or(|b| left < b.0) {
                best = Some((left, key));
            }
            for (dx, dz) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
                let n = (c.0 + dx, c.1 + dz);
                let m = centre(n);
                if (m[0] - start2[0]).hypot(m[1] - start2[1]) > REACH {
                    continue;
                }
                let len = if dx != 0 && dz != 0 { CELL * std::f32::consts::SQRT_2 } else { CELL };
                for (nk, h, s) in self.nodes(n) {
                    let rise = h - here[1];
                    let walk = rise <= self.p.step && -rise <= self.p.drop;
                    let jump = !walk && self.p.jumps && rise > self.p.step && rise <= self.p.jump;
                    if !walk && !jump {
                        continue;
                    }
                    let p = [m[0], h, m[1]];
                    let Some(f) = self.marked(here, p, jump) else { continue };
                    let ground = self.ground_cost(n, &s);
                    let cost = gk + len * ground * f + if jump { JUMP_COST } else { 0.0 };
                    if g.get(&nk).is_none_or(|&b| cost < b - 1e-6) {
                        g.insert(nk, cost);
                        came.insert(nk, (key, jump));
                        at.insert(nk, p);
                        push(&mut open, cost + h_of((m[0] - goal[0]).hypot(m[1] - goal[1])), nk);
                    }
                }
            }
        }
        let end = match reached {
            Some(k) => k,
            None => {
                let (left, k) = best?;
                // No nearer than the start: no way at all.
                if left >= (start2[0] - goal[0]).hypot(start2[1] - goal[1]) - CELL {
                    return None;
                }
                k
            }
        };
        let mut keys = vec![(end, false)];
        let mut k = end;
        while let Some(&(prev, jump)) = came.get(&k) {
            keys.last_mut().unwrap().1 = jump;
            keys.push((prev, false));
            k = prev;
        }
        keys.reverse();
        let mut points = vec![self.start];
        let mut jumps = vec![false];
        for (k, jump) in keys {
            points.push(at[&k]);
            jumps.push(jump);
        }
        let length = points.windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][2] - w[0][2])).sum();
        Some(Path { points, jumps, length, reached: reached.is_some(), expanded })
    }

    /// Whether a straight walk from `a` to `b` (feet) keeps on surfaces the
    /// bot stands on, a step at a time, through no shut mark.
    pub fn straight(&mut self, a: [f32; 3], b: [f32; 3]) -> bool {
        let len = (b[0] - a[0]).hypot(b[2] - a[2]);
        let steps = (len / (CELL * 0.5)).ceil().max(1.0) as usize;
        let mut h = a[1];
        let mut last = a;
        for i in 1..=steps {
            let t = i as f32 / steps as f32;
            let (x, z) = (a[0] + (b[0] - a[0]) * t, a[2] + (b[2] - a[2]) * t);
            let c = cell_of(x, z);
            let step = self.p.step;
            let drop = self.p.drop;
            let Some(next) = self
                .nodes(c)
                .into_iter()
                .map(|n| n.1)
                .filter(|&nh| nh - h <= step && h - nh <= drop)
                .min_by(|p, q| (p - h).abs().total_cmp(&(q - h).abs()))
            else {
                return false;
            };
            let p = [x, next, z];
            if self.marked(last, p, false).is_none() {
                return false;
            }
            h = next;
            last = p;
        }
        true
    }

    /// The next leg of `path`: the farthest point within `max_leg` reached
    /// by a straight walk (before any jump): (heading, distance, point).
    pub fn leg(&mut self, path: &Path, max_leg: f32) -> Option<(f32, f32, [f32; 3])> {
        let from = *path.points.first()?;
        let mut pick = None;
        for (i, &p) in path.points.iter().enumerate().skip(1) {
            if path.jumps[i] {
                break;
            }
            let d = (p[0] - from[0]).hypot(p[2] - from[2]);
            if d > max_leg {
                break;
            }
            if self.straight(from, p) {
                pick = Some(p);
            }
        }
        // Nothing straight (a turn right at the start): the first point.
        let p = pick.or_else(|| path.points.get(1).copied())?;
        let d = (p[0] - from[0]).hypot(p[2] - from[2]);
        (d > 1e-3).then(|| (heading([from[0], from[2]], [p[0], p[2]]), d, p))
    }

    /// Whether surface `s` of column `c` is stood on (for pictures).
    pub fn stood_on(&mut self, c: (i32, i32), k: usize, s: &Surface) -> bool {
        self.stands(c, k as u8, s)
    }
}

/// A key and its priority (least first).
struct Item(f32, Key);

impl PartialEq for Item {
    fn eq(&self, o: &Self) -> bool {
        self.0 == o.0
    }
}
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.0.total_cmp(&self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{floor, look, wall_x};
    use crate::{MarkKind, WorldMap};

    fn mapped(pts: &[[f32; 3]]) -> WorldMap {
        let mut m = WorldMap::default();
        // Seen from a few places, as walking would.
        for feet in [[0.0, 0.0, 0.0], [2.0, 0.0, -2.0], [-2.0, 0.0, -2.0], [0.0, 0.0, -4.0]] {
            m.integrate(&look(pts, feet, 1.5), feet, 0.0);
        }
        m
    }

    #[test]
    fn round_a_wall_through_its_door() {
        let mut pts = Vec::new();
        floor(&mut pts, (-4.0, 4.0), (-6.0, 2.0), 0.0);
        wall_x(&mut pts, (-4.0, 1.0), -2.0, 0.0, 2.5);
        wall_x(&mut pts, (2.0, 4.0), -2.0, 0.0, 2.5); // a door at x 1..2
        let m = mapped(&pts);
        let p = PlanParams { now: 0, ..Default::default() };
        let path = Planner::new(&m, p, [0.0, 0.0, 0.0]).plan([0.0, -4.0], None).expect("a way");
        assert!(path.reached, "{path:?}");
        // Through the door.
        let crossing = path.points.iter().find(|q| (q[2] + 2.0).abs() < 0.1).expect("crosses the wall's line");
        assert!((1.0..2.0).contains(&crossing[0]), "{crossing:?}");
        assert!(path.length > 4.5, "round, not through: {}", path.length);
    }

    #[test]
    fn up_the_stairs_to_the_floor_above() {
        // Floor, then 10 steps (0.18 up, 0.3 deep) to a landing 1.8 up.
        let mut pts = Vec::new();
        floor(&mut pts, (-1.0, 1.0), (-1.0, 1.0), 0.0);
        floor(&mut pts, (-1.0, 1.0), (-1.5, -1.0), 0.0);
        for k in 0..10 {
            let z0 = -1.5 - 0.3 * k as f32;
            let y = 0.18 * (k + 1) as f32;
            floor(&mut pts, (-1.0, 1.0), (z0 - 0.3, z0), y);
            // The riser.
            wall_x(&mut pts, (-1.0, 1.0), z0, y - 0.18, y);
        }
        floor(&mut pts, (-1.0, 1.0), (-7.5, -4.5), 1.8);
        // Walls beside the stairs, up past a body.
        for x in [-1.05f32, 1.05] {
            let mut z = -7.5;
            while z < 1.0 {
                let mut y = 0.0;
                while y < 4.0 {
                    pts.push([x, y, z]);
                    y += 0.05;
                }
                z += 0.05;
            }
        }
        let mut m = WorldMap::default();
        for feet in [[0.0, 0.0, 0.0], [0.0, 0.9, -3.0], [0.0, 1.8, -6.0]] {
            m.integrate(&look(&pts, feet, 1.5), feet, 0.0);
        }
        let p = PlanParams { now: 0, body: 1.2, ..Default::default() };
        let path = Planner::new(&m, p, [0.0, 0.0, 0.0]).plan([0.0, -6.5], Some(1.8)).expect("up the stairs");
        assert!(path.reached, "{path:?}");
        let top = path.points.last().unwrap();
        assert!((top[1] - 1.8).abs() < 0.05, "{top:?}");
        // Every move a step or less.
        assert!(path.points.windows(2).all(|w| (w[1][1] - w[0][1]).abs() <= 0.3), "{path:?}");
    }

    #[test]
    fn a_mark_shuts_the_way_it_was_stopped_and_walking_reopens_it() {
        let mut pts = Vec::new();
        floor(&mut pts, (-3.0, 3.0), (-5.0, 1.0), 0.0);
        let mut m = mapped(&pts);
        let p = PlanParams { now: 100, ..Default::default() };
        let straight = Planner::new(&m, p, [0.0, 0.0, 0.0]).plan([0.0, -4.0], None).unwrap();
        assert!(straight.length < 4.3, "{}", straight.length);
        // Glass across the way at z = -2 (x -1..1): walks stopped there.
        for k in -6..=6 {
            m.mark([k as f32 * crate::nav::PANE_STEP, 0.0, -2.0], 0.0, MarkKind::Blocked, 100);
        }
        let round = Planner::new(&m, p, [0.0, 0.0, 0.0]).plan([0.0, -4.0], None).unwrap();
        assert!(round.reached && round.length > straight.length + 0.3, "{round:?}");
        assert!(round.points.iter().filter(|q| (q[2] + 2.0).abs() < 0.1).all(|q| q[0].abs() > 0.9), "{round:?}");
        // Much later the marks are doubtful: a long way round costs more than trying.
        let later = PlanParams { now: 100 + 3 * 24 * 3600, ..Default::default() };
        let tried = Planner::new(&m, later, [0.0, 0.0, 0.0]).plan([0.0, -4.0], None).unwrap();
        assert!(tried.reached);
    }

    #[test]
    fn ways_walked_are_kept_to_and_feet_overrule_sight() {
        // A sofa (seen, so in the way) between the bot and the goal; the bot
        // once walked through it (no collision).
        let mut pts = Vec::new();
        floor(&mut pts, (-3.0, 3.0), (-5.0, 1.0), 0.0);
        let mut sofa = Vec::new();
        floor(&mut sofa, (-2.0, 2.0), (-2.5, -1.7), 0.45);
        wall_x(&mut sofa, (-2.0, 2.0), -2.5, 0.0, 0.9);
        pts.extend(sofa);
        let mut m = mapped(&pts);
        let p = PlanParams { now: 0, ..Default::default() };
        let round = Planner::new(&m, p, [0.0, 0.0, 0.0]).plan([0.0, -4.0], None).unwrap();
        assert!(round.points.iter().filter(|q| (-2.5..-1.7).contains(&q[2])).all(|q| q[0].abs() > 1.9), "{round:?}");
        let mut z = -0.9;
        while z > -3.2 {
            m.walked([0.0, 0.0, z], true, 0);
            z -= 0.1;
        }
        let through = Planner::new(&m, p, [0.0, 0.0, 0.0]).plan([0.0, -4.0], None).unwrap();
        assert!(through.length < 4.5, "{through:?}");
        // Beside the trail (a few columns off): still drawn to it.
        let mut pl = Planner::new(&m, p, [0.25, 0.0, 0.0]);
        let off = pl.plan([0.0, -4.0], None).unwrap();
        assert!(off.points.iter().filter(|q| (-2.5..-1.7).contains(&q[2])).all(|q| q[0].abs() < 0.35), "{off:?}");
        // And it walks it straight.
        let mut pl = Planner::new(&m, p, [0.0, 0.0, 0.0]);
        let (yaw, d, _) = pl.leg(&through, 3.0).unwrap();
        assert!(yaw.abs() < 5.0 && d > 2.5, "{yaw} {d}");
    }
}
