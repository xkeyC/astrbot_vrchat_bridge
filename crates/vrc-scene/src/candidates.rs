//! Places worth going to, from a height map: what the cloud model picks
//! from when it decides where the bot walks ("go to 3").
//!
//! - Reachable floor: a flood fill over floor cells from around the eyes,
//!   keeping a body's width off obstacles (unknown cells are not walked on,
//!   but not kept off: what is not seen of a floor is mostly more floor).
//! - Frontiers: reachable cells next to the unknown, grouped; one point per
//!   group (where looking further would show more).
//! - Open: per sector around the bot, the farthest reachable cell (where
//!   walking takes it far).
//! - Platforms: raised tops next to reachable floor (a jump up).
//! - Players: people the caller found (`vrc-players`: name tags matched to
//!   the room, whitelisted friends first), always kept.
//!
//! Points closer than `min_gap` to a better one are dropped, and the rest
//! numbered left to right as seen from where the bot faces.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::heightmap::{Cell, HeightMap};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Frontier,
    Open,
    Platform,
    Player,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::Frontier => "frontier",
            Kind::Open => "open",
            Kind::Platform => "platform",
            Kind::Player => "player",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Candidate {
    /// 1-based, left to right from the heading.
    pub id: usize,
    pub kind: Kind,
    /// x, z in the tracking space; y of the floor (or platform top) there.
    pub position: [f32; 3],
    /// Straight-line distance from the eyes, along the floor.
    pub distance: f32,
    /// Walking distance over reachable cells (to the cell next to a platform).
    pub path: f32,
    /// Degrees from the heading, + right.
    pub bearing: f32,
    /// A player's display name.
    pub name: Option<String>,
    /// A player's priority on the whitelist (1 first).
    pub whitelist_rank: Option<usize>,
    /// A platform's height over the ground it is jumped onto from.
    pub rise: Option<f32>,
}

/// A player to offer as a candidate.
#[derive(Clone, Debug)]
pub struct Person {
    pub name: String,
    pub whitelist_rank: Option<usize>,
    /// Where they stand (tracking space, y on the floor).
    pub feet: [f32; 3],
}

#[derive(Clone, Copy, Debug)]
pub struct CandidateParams {
    /// Clearance kept from obstacles and the unknown (a body's half width).
    pub clearance: f32,
    /// Sectors around the bot for the open points.
    pub sectors: usize,
    /// Open points nearer than this are not worth a step.
    pub min_distance: f32,
    /// Points closer to each other than this collapse into one.
    pub min_gap: f32,
    /// Frontier groups smaller than this many cells are noise.
    pub min_frontier_cells: usize,
    pub max_candidates: usize,
}

impl Default for CandidateParams {
    fn default() -> Self {
        CandidateParams {
            clearance: 0.3,
            sectors: 8,
            min_distance: 1.0,
            min_gap: 1.0,
            min_frontier_cells: 4,
            max_candidates: 10,
        }
    }
}

/// Cells a body fits on: floor at least `clearance` from obstacles.
pub fn walkable(map: &HeightMap, clearance: f32) -> Vec<bool> {
    let n = map.size;
    let cells = map.classify();
    let blocked = inflate(map, &cells, clearance);
    (0..n * n).map(|i| cells[i] == Cell::Floor && !blocked[i]).collect()
}

/// Cells a walk was stopped at ([`HeightMap::mark_blocked`]): not even
/// the cells around the bot, which count as its floor, are walked through
/// there. (Not grown: the bot stands close to them.)
pub fn stopped(map: &HeightMap) -> Vec<bool> {
    (0..map.size * map.size).map(|i| map.is_blocked(i)).collect()
}

fn inflate(map: &HeightMap, cells: &[Cell], clearance: f32) -> Vec<bool> {
    let obstacles: Vec<bool> = cells.iter().map(|c| *c == Cell::Obstacle).collect();
    grow(map, &obstacles, clearance)
}

/// `cells` grown by `clearance`.
fn grow(map: &HeightMap, cells: &[bool], clearance: f32) -> Vec<bool> {
    let n = map.size;
    let r = (clearance / map.params.cell).ceil() as isize;
    let mut blocked = vec![false; n * n];
    for (i, c) in cells.iter().enumerate() {
        if *c {
            let (row, col) = ((i / n) as isize, (i % n) as isize);
            for dr in -r..=r {
                for dc in -r..=r {
                    let (rr, cc) = (row + dr, col + dc);
                    if dr * dr + dc * dc <= r * r && rr >= 0 && cc >= 0 && (rr as usize) < n && (cc as usize) < n {
                        blocked[rr as usize * n + cc as usize] = true;
                    }
                }
            }
        }
    }
    blocked
}

/// Reachable cells with their walking distance in tenths of a cell (a
/// straight step 10, a diagonal 14), from around `eye`: over walkable
/// cells, stepping up at most `step` and down at most `drop` between
/// neighbours (the map's parameters).
pub fn reachable(map: &HeightMap, eye: [f32; 3], clearance: f32) -> Vec<Option<u32>> {
    paths(map, eye, clearance).0
}

/// [`reachable`], and each reached cell's previous cell on its shortest
/// walk (`usize::MAX` for the start).
pub fn paths(map: &HeightMap, eye: [f32; 3], clearance: f32) -> (Vec<Option<u32>>, Vec<usize>) {
    let n = map.size;
    let free = walkable(map, clearance);
    let halt = stopped(map);
    let ground = map.grounds();
    let (step, drop) = (map.params.step, map.params.drop);

    // Start from the cells around the eyes: the bot stands there, so they
    // are its floor whatever the depth made of them (its own body, a coat or a
    // tail reaching past `self_radius`, reads as raised ground).
    let mut dist: Vec<Option<u32>> = vec![None; n * n];
    let mut prev = vec![usize::MAX; n * n];
    let mut height: Vec<f32> = ground.iter().map(|g| g.unwrap_or(f32::NAN)).collect();
    let mut queue = BinaryHeap::new();
    let start_r = map.params.self_radius + 2.0 * map.params.cell;
    let k = (start_r / map.params.cell).ceil() as isize;
    if let Some(centre) = map.index(eye[0], eye[2]) {
        let (row, col) = ((centre / n) as isize, (centre % n) as isize);
        for dr in -k..=k {
            for dc in -k..=k {
                let (rr, cc) = (row + dr, col + dc);
                if rr < 0 || cc < 0 || rr as usize >= n || cc as usize >= n {
                    continue;
                }
                let i = rr as usize * n + cc as usize;
                let inside = (dr * dr + dc * dc) as f32 * map.params.cell * map.params.cell <= start_r * start_r;
                // Not past where a walk was stopped (a mirror, glass).
                let to = map.centre(i);
                if inside && !crosses(map, &halt, [eye[0], eye[2]], to) {
                    height[i] = map.floor;
                    dist[i] = Some(0);
                    queue.push(Reverse((0u32, i)));
                }
            }
        }
    }
    while let Some(Reverse((d, i))) = queue.pop() {
        if dist[i].is_some_and(|best| d > best) {
            continue;
        }
        let (row, col) = (i / n, i % n);
        for (dr, dc) in [(-1isize, 0isize), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)] {
            let (rr, cc) = (row as isize + dr, col as isize + dc);
            if rr < 0 || cc < 0 || rr as usize >= n || cc as usize >= n {
                continue;
            }
            let j = rr as usize * n + cc as usize;
            if !free[j] {
                continue;
            }
            let rise = height[j] - height[i];
            if !(rise <= step && -rise <= drop) {
                continue;
            }
            let nd = d + if dr != 0 && dc != 0 { 14 } else { 10 };
            if dist[j].is_none_or(|best| nd < best) {
                dist[j] = Some(nd);
                prev[j] = i;
                queue.push(Reverse((nd, j)));
            }
        }
    }
    (dist, prev)
}

/// Whether the straight line `from`-`to` (x, z) passes a cell of `cells`.
pub fn crosses(map: &HeightMap, cells: &[bool], from: [f32; 2], to: [f32; 2]) -> bool {
    let (dx, dz) = (to[0] - from[0], to[1] - from[1]);
    let steps = (dx.hypot(dz) / (map.params.cell * 0.5)).ceil().max(1.0) as usize;
    (0..=steps).any(|k| {
        let t = k as f32 / steps as f32;
        map.index(from[0] + dx * t, from[1] + dz * t).is_some_and(|i| cells[i])
    })
}

/// Candidates around `eye`, numbered from the heading `yaw_deg`: `people`
/// first (whitelisted before others), then places.
pub fn candidates(
    map: &HeightMap,
    eye: [f32; 3],
    yaw_deg: f32,
    people: &[Person],
    p: &CandidateParams,
) -> Vec<Candidate> {
    let n = map.size;
    let cell = map.params.cell;
    let cells = map.classify();
    let dist = reachable(map, eye, p.clearance);
    let centre = |i: usize| {
        let half = n as f32 / 2.0;
        [
            map.origin[0] + ((i % n) as f32 + 0.5 - half) * cell,
            map.origin[1] + ((i / n) as f32 + 0.5 - half) * cell,
        ]
    };
    let ground = map.grounds();
    let floor_at = |i: usize| ground[i].unwrap_or(map.floor);
    let make = |i: usize, kind: Kind, path_cells: u32| {
        let [x, z] = centre(i);
        let (dx, dz) = (x - eye[0], z - eye[2]);
        let bearing = super_angle(dx.atan2(-dz).to_degrees() - yaw_deg);
        Candidate {
            id: 0,
            kind,
            position: [x, floor_at(i), z],
            distance: dx.hypot(dz),
            path: path_cells as f32 * cell / 10.0,
            bearing,
            name: None,
            whitelist_rank: None,
            rise: None,
        }
    };
    let neighbours = |i: usize| {
        let (row, col) = ((i / n) as isize, (i % n) as isize);
        [(-1isize, 0isize), (1, 0), (0, -1), (0, 1)].into_iter().filter_map(move |(dr, dc)| {
            let (rr, cc) = (row + dr, col + dc);
            (rr >= 0 && cc >= 0 && (rr as usize) < n && (cc as usize) < n).then(|| rr as usize * n + cc as usize)
        })
    };

    let mut found: Vec<(f32, Candidate)> = Vec::new(); // (score, candidate)

    // People outrank every place; whitelisted friends outrank strangers.
    for person in people {
        let [x, y, z] = person.feet;
        let (dx, dz) = (x - eye[0], z - eye[2]);
        let walk = map.index(x, z).and_then(|i| dist[i]).map(|d| d as f32 * cell / 10.0).unwrap_or(f32::NAN);
        let rank = person.whitelist_rank.map_or(1000.0, |r| r as f32);
        found.push((
            1e6 - rank,
            Candidate {
                id: 0,
                kind: Kind::Player,
                position: [x, y, z],
                distance: dx.hypot(dz),
                path: walk,
                bearing: super_angle(dx.atan2(-dz).to_degrees() - yaw_deg),
                name: Some(person.name.clone()),
                whitelist_rank: person.whitelist_rank,
                rise: None,
            },
        ));
    }

    // Frontiers: reachable cells touching the unknown, grouped 8-connected
    // (not the ring the bot stands in, next to its own unseen disc).
    let is_frontier: Vec<bool> = (0..n * n)
        .map(|i| dist[i].is_some_and(|d| d > 0) && neighbours(i).any(|j| cells[j] == Cell::Unknown))
        .collect();
    let mut seen = vec![false; n * n];
    for s in 0..n * n {
        if !is_frontier[s] || seen[s] {
            continue;
        }
        let mut group = vec![s];
        seen[s] = true;
        let mut k = 0;
        while k < group.len() {
            let i = group[k];
            k += 1;
            let (row, col) = ((i / n) as isize, (i % n) as isize);
            for dr in -1..=1 {
                for dc in -1..=1 {
                    let (rr, cc) = (row + dr, col + dc);
                    if rr < 0 || cc < 0 || rr as usize >= n || cc as usize >= n {
                        continue;
                    }
                    let j = rr as usize * n + cc as usize;
                    if is_frontier[j] && !seen[j] {
                        seen[j] = true;
                        group.push(j);
                    }
                }
            }
        }
        if group.len() < p.min_frontier_cells {
            continue;
        }
        // The group's cell nearest its middle.
        let (sx, sz) = group.iter().fold((0.0, 0.0), |(a, b), &i| {
            let [x, z] = centre(i);
            (a + x, b + z)
        });
        let (mx, mz) = (sx / group.len() as f32, sz / group.len() as f32);
        let best = *group
            .iter()
            .min_by(|&&a, &&b| {
                let (pa, pb) = (centre(a), centre(b));
                ((pa[0] - mx).hypot(pa[1] - mz)).total_cmp(&(pb[0] - mx).hypot(pb[1] - mz))
            })
            .unwrap();
        let c = make(best, Kind::Frontier, dist[best].unwrap());
        if c.distance >= p.min_distance {
            found.push((group.len() as f32 * cell, c));
        }
    }

    // Open: the farthest reachable cell per sector.
    let mut far: Vec<Option<(f32, usize)>> = vec![None; p.sectors];
    for i in 0..n * n {
        if dist[i].is_none() {
            continue;
        }
        let c = make(i, Kind::Open, 0);
        if c.distance < p.min_distance {
            continue;
        }
        let s = (((c.bearing + 180.0) / 360.0 * p.sectors as f32) as usize).min(p.sectors - 1);
        if far[s].is_none_or(|(d, _)| c.distance > d) {
            far[s] = Some((c.distance, i));
        }
    }
    for (d, i) in far.into_iter().flatten() {
        found.push((d, make(i, Kind::Open, dist[i].unwrap())));
    }

    // Platforms: ground out of walking reach that is higher than a
    // reachable neighbour by more than a step and at most a jump.
    let (step, jump) = (map.params.step, map.params.jump);
    let free = walkable(map, 0.0);
    let is_top = |i: usize| -> Option<(u32, f32)> {
        let h = ground[i]?;
        if dist[i].is_some() || !free[i] {
            return None;
        }
        let (row, col) = ((i / n) as isize, (i % n) as isize);
        let mut best = None;
        let reach = (p.clearance / cell).ceil() as isize + 1;
        for dr in -reach..=reach {
            for dc in -reach..=reach {
                let (rr, cc) = (row + dr, col + dc);
                if rr < 0 || cc < 0 || rr as usize >= n || cc as usize >= n {
                    continue;
                }
                let j = rr as usize * n + cc as usize;
                if let (Some(d), Some(g)) = (dist[j], ground[j]) {
                    let rise = h - g;
                    if rise > step && rise <= jump && best.is_none_or(|(b, _)| d < b) {
                        best = Some((d, rise));
                    }
                }
            }
        }
        best
    };
    let mut top_seen = vec![false; n * n];
    for i in 0..n * n {
        if top_seen[i] || is_top(i).is_none() {
            continue;
        }
        // The top as a group of neighbouring ground at about its height.
        let h0 = ground[i].unwrap();
        let mut group = vec![i];
        top_seen[i] = true;
        let mut k = 0;
        while k < group.len() {
            let a = group[k];
            k += 1;
            for b in neighbours(a) {
                if !top_seen[b] && dist[b].is_none() && ground[b].is_some_and(|g| (g - h0).abs() <= step) {
                    top_seen[b] = true;
                    group.push(b);
                }
            }
        }
        if group.len() < 3 {
            continue;
        }
        // Its edge cell nearest by walk.
        let best = group.iter().filter_map(|&a| is_top(a).map(|(d, rise)| (d, a, rise))).min_by_key(|t| (t.0, t.1));
        if let Some((d, a, rise)) = best {
            let mut c = make(a, Kind::Platform, d);
            c.rise = Some(rise);
            found.push((group.len() as f32 * cell * 2.0, c));
        }
    }

    // Best first, then drop points too close to a kept one.
    found.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut kept: Vec<Candidate> = Vec::new();
    for (_, c) in found {
        if kept.len() >= p.max_candidates {
            break;
        }
        let close = c.kind != Kind::Player
            && kept.iter().any(|k| (k.position[0] - c.position[0]).hypot(k.position[2] - c.position[2]) < p.min_gap);
        if !close {
            kept.push(c);
        }
    }
    kept.sort_by(|a, b| a.bearing.total_cmp(&b.bearing));
    for (i, c) in kept.iter_mut().enumerate() {
        c.id = i + 1;
    }
    kept
}

/// Degrees wrapped to -180..180.
fn super_angle(a: f32) -> f32 {
    (a + 540.0).rem_euclid(360.0) - 180.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::heightmap::MapParams;

    /// A room 6 x 6 around the bot, a wall across the far half with a door,
    /// a table to the right, unknown beyond the door.
    fn room() -> (HeightMap, [f32; 3]) {
        let mut m = HeightMap::new(MapParams { radius: 5.0, ..Default::default() }, [0.0, 0.0], 0.0);
        let eye = [0.0, 1.6, 0.0];
        let mut pts = Vec::new();
        for i in 0..180 {
            for j in 0..180 {
                let (x, z) = (i as f32 / 30.0 - 3.0, j as f32 / 30.0 - 3.0);
                let table = (1.5..2.2).contains(&x) && (-0.5..0.5).contains(&z);
                if !table {
                    pts.push([x, 0.0, z]);
                }
                // A wall at z = -2 with a door at x in -0.5..0.5.
                if (-2.05..-1.95).contains(&z) && !(-0.5..0.5).contains(&x) {
                    for k in 0..10 {
                        pts.push([x, 0.2 * k as f32, z]);
                    }
                }
                if table {
                    pts.push([x, 0.75, z]);
                }
            }
        }
        // A corridor seen through the door.
        for i in 0..30 {
            for j in 0..30 {
                pts.push([i as f32 / 30.0 - 0.5, 0.0, -2.0 - j as f32 / 30.0]);
            }
        }
        m.add(&pts, eye);
        (m, eye)
    }

    #[test]
    fn finds_the_door_the_room_and_the_table() {
        let (m, eye) = room();
        let friend = Person { name: "xkeyC".into(), whitelist_rank: Some(1), feet: [-1.0, 0.0, -1.0] };
        let cs = candidates(&m, eye, 0.0, std::slice::from_ref(&friend), &CandidateParams::default());
        // The friend, to the front left, named.
        let f = cs.iter().find(|c| c.kind == Kind::Player).unwrap();
        assert_eq!((f.name.as_deref(), f.whitelist_rank), (Some("xkeyC"), Some(1)));
        assert!((f.bearing + 45.0).abs() < 1.0, "{f:?}");
        assert!(!cs.is_empty());
        // Ids go left to right.
        assert!(cs.windows(2).all(|w| w[0].bearing <= w[1].bearing));
        // Something through the door, ahead.
        assert!(cs.iter().any(|c| c.position[2] < -2.2 && c.bearing.abs() < 30.0), "{cs:#?}");
        // The table, to the right.
        assert!(cs.iter().any(|c| c.kind == Kind::Platform && c.bearing > 45.0), "{cs:#?}");
        // Behind the wall is reached only through the door: the walk is
        // longer than the straight line.
        let dist = reachable(&m, eye, 0.3);
        let behind_wall = m.index(-2.0, -2.5).unwrap();
        let walk = dist[behind_wall].unwrap() as f32 * m.params.cell;
        // (The walk starts at the ring around the body, self_radius + 2 cells out.)
        let walk = walk / 10.0;
        let straight_from_ring = 2.0f32.hypot(2.5) - (m.params.self_radius + 0.2);
        assert!(walk > straight_from_ring + 0.5, "{walk}");
        // Beyond what was seen, nothing.
        assert!(dist[m.index(-4.0, 0.0).unwrap()].is_none());
    }
}

