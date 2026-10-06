//! Keeping the odometry on the map. The playspace never turns (the bot
//! turns its head, not the world), so a look is off the map by a shift
//! only: across, from where its walls fall on the map's walls; up, from
//! where its floors fall on the map's floors.
//!
//! Across: the map's walls near the bot (what stands between a knee and a
//! bit over the eyes over the bot's floor) as a distance field; the look's
//! wall points shifted over a grid of shifts, the best by the mean squared
//! distance (truncated: what the map has not seen costs the same anywhere),
//! with a little cost on the shift itself (the odometry is a fair guess).
//! Along a corridor's walls any shift along it fits as well: that way is
//! left to the odometry. Up: the median gap from the look's floor points to
//! the map's surfaces in the same columns.

use crate::{cell_of, turn, Observation, WorldMap, CELL};

/// Wall points: higher than this over the feet, up to a bit over the eyes.
const WALL_FROM: f32 = 0.4;
const WALL_OVER_EYES: f32 = 0.3;
/// Points used: from this far to this far (stereo is fine near).
const NEAR: f32 = 1.0;
const FAR: f32 = 4.0;
const FLOOR_FAR: f32 = 3.5;
const SAMPLE: usize = 3000;
/// The map's walls this far round the bot.
const WINDOW: f32 = 7.0;
/// Distances count up to this.
const TRUNC: f32 = 0.5;
/// Shifts tried: within this, this far apart.
const SEARCH: f32 = 0.5;
const STEP: f32 = 0.05;
/// Cost of a shift per square metre.
const PRIOR: f32 = 0.01;
const MIN_POINTS: usize = 150;
const MIN_MAP_CELLS: usize = 80;
/// After the shift, this share of the points on (within 15 cm of) a wall.
const MIN_OVERLAP: f32 = 0.35;
/// A direction is undecided when it curves less than this share of the
/// best one.
const FLAT_SHARE: f32 = 0.15;
const UP_POINTS: usize = 40;
const UP_MAX: f32 = 0.25;

/// How a look fitted the map.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Fit {
    /// To add to the odometry (map frame).
    pub shift: [f32; 3],
    /// Wall points used, and the share of them on the map's walls after.
    pub used: usize,
    pub overlap: f32,
    /// Mean squared distance before and after (m²).
    pub before: f32,
    pub after: f32,
    /// The shift across was taken.
    pub across: bool,
    /// The shift up was taken (floor points matched).
    pub up: bool,
    /// One direction (degrees, map axes) was left to the odometry.
    pub flat: Option<f32>,
    /// Why not (empty when taken).
    pub why: &'static str,
}

struct Field {
    origin: [f32; 2],
    n: usize,
    dist: Vec<f32>,
}

impl Field {
    /// The map's walls round `at` (x, z), between `lo` and `hi`: each wall
    /// column's points where they are in it (not its middle: a wall on a
    /// column's edge is there), the distance to the nearest at each
    /// column's middle.
    fn walls(map: &WorldMap, at: [f32; 2], lo: f32, hi: f32) -> (Field, usize) {
        let n = (2.0 * WINDOW / CELL) as usize;
        let c0 = cell_of(at[0] - WINDOW, at[1] - WINDOW);
        let mut wall: Vec<Option<[f32; 2]>> = vec![None; n * n];
        let mut count = 0;
        for r in 0..n {
            for c in 0..n {
                if let Some(col) = map.column((c0.0 + c as i32, c0.1 + r as i32)) {
                    wall[r * n + c] = col.mean_between(lo, hi);
                    count += wall[r * n + c].is_some() as usize;
                }
            }
        }
        let origin = [c0.0 as f32 * CELL, c0.1 as f32 * CELL];
        let k = (TRUNC / CELL).ceil() as isize + 1;
        let mut dist = vec![TRUNC; n * n];
        for (i, w) in wall.iter().enumerate() {
            let Some(p) = w else { continue };
            let (r0, c0) = ((i / n) as isize, (i % n) as isize);
            for dr in -k..=k {
                for dc in -k..=k {
                    let (r, c) = (r0 + dr, c0 + dc);
                    if r < 0 || c < 0 || r as usize >= n || c as usize >= n {
                        continue;
                    }
                    let j = r as usize * n + c as usize;
                    let m = [origin[0] + (c as f32 + 0.5) * CELL, origin[1] + (r as f32 + 0.5) * CELL];
                    let d = (m[0] - p[0]).hypot(m[1] - p[1]);
                    if d < dist[j] {
                        dist[j] = d;
                    }
                }
            }
        }
        (Field { origin, n, dist }, count)
    }

    /// The distance at (x, z), bilinear between cell middles.
    fn at(&self, x: f32, z: f32) -> f32 {
        let gx = (x - self.origin[0]) / CELL - 0.5;
        let gz = (z - self.origin[1]) / CELL - 0.5;
        let (c, r) = (gx.floor(), gz.floor());
        if c < 0.0 || r < 0.0 || c as usize + 1 >= self.n || r as usize + 1 >= self.n {
            return TRUNC;
        }
        let (fx, fz) = (gx - c, gz - r);
        let (c, r) = (c as usize, r as usize);
        let v = |r: usize, c: usize| self.dist[r * self.n + c];
        let top = v(r, c) * (1.0 - fx) + v(r, c + 1) * fx;
        let bottom = v(r + 1, c) * (1.0 - fx) + v(r + 1, c + 1) * fx;
        top * (1.0 - fz) + bottom * fz
    }
}

/// How `obs`, taken with the feet at `pose` (map frame, the session's axes
/// turned `yaw`), fits the map.
pub fn register(map: &WorldMap, obs: &Observation, pose: [f32; 3], yaw: f32) -> Fit {
    let mut fit = Fit::default();
    let to_map = |p: [f32; 3]| {
        let q = turn(p, yaw);
        [q[0] + pose[0], q[1] + pose[1], q[2] + pose[2]]
    };
    let wall_hi = obs.eye[1] + WALL_OVER_EYES;
    let range = |p: &[f32; 3]| p[0].hypot(p[2]);
    let mut walls: Vec<[f32; 3]> = obs
        .points
        .iter()
        .filter(|p| p[1] > WALL_FROM && p[1] < wall_hi && (NEAR..FAR).contains(&range(p)) && obs.keeps(**p))
        .map(|&p| to_map(p))
        .collect();
    thin(&mut walls, SAMPLE);
    fit.used = walls.len();

    // Across.
    let (field, cells) = Field::walls(map, [pose[0], pose[2]], pose[1] + WALL_FROM, pose[1] + wall_hi);
    let cost = |t: [f32; 2]| {
        let sum: f32 = walls.iter().map(|p| field.at(p[0] + t[0], p[2] + t[1]).powi(2)).sum();
        sum / walls.len().max(1) as f32
    };
    let mut shift = [0.0f32; 2];
    if walls.len() < MIN_POINTS {
        fit.why = "too few wall points";
    } else if cells < MIN_MAP_CELLS {
        fit.why = "too little map";
    } else {
        fit.before = cost([0.0, 0.0]);
        let k = (SEARCH / STEP).round() as i32;
        let mut best = (f32::INFINITY, [0.0f32; 2]);
        for i in -k..=k {
            for j in -k..=k {
                let t = [i as f32 * STEP, j as f32 * STEP];
                let c = cost(t) + PRIOR * (t[0] * t[0] + t[1] * t[1]);
                if c < best.0 {
                    best = (c, t);
                }
            }
        }
        let t = best.1;
        let total = |t: [f32; 2]| cost(t) + PRIOR * (t[0] * t[0] + t[1] * t[1]);
        let c0 = total(t);
        // Curvature at the best (finite differences), for each direction.
        let h = STEP;
        let hxx = (total([t[0] + h, t[1]]) + total([t[0] - h, t[1]]) - 2.0 * c0) / (h * h);
        let hzz = (total([t[0], t[1] + h]) + total([t[0], t[1] - h]) - 2.0 * c0) / (h * h);
        let hxz = (total([t[0] + h, t[1] + h]) - total([t[0] + h, t[1] - h]) - total([t[0] - h, t[1] + h]) + total([t[0] - h, t[1] - h])) / (4.0 * h * h);
        // Sub-step: the parabola's bottom each way.
        let sub = |a: f32, b: f32, c: f32| if a + b - 2.0 * c > 1e-9 { (0.5 * (a - b) / (a + b - 2.0 * c)).clamp(-0.5, 0.5) } else { 0.0 };
        let t = [
            t[0] + h * sub(total([t[0] - h, t[1]]), total([t[0] + h, t[1]]), c0),
            t[1] + h * sub(total([t[0], t[1] - h]), total([t[0], t[1] + h]), c0),
        ];
        // The eigen directions of the curvature.
        let (tr, det) = (hxx + hzz, hxx * hzz - hxz * hxz);
        let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
        let (big, small) = (tr / 2.0 + disc, tr / 2.0 - disc);
        let dir_big = if hxz.abs() > 1e-9 { [big - hzz, hxz] } else if hxx >= hzz { [1.0, 0.0] } else { [0.0, 1.0] };
        let norm = dir_big[0].hypot(dir_big[1]);
        let e = [dir_big[0] / norm, dir_big[1] / norm];
        shift = t;
        if small < FLAT_SHARE * big {
            // Only the well-decided direction: along e.
            let along = t[0] * e[0] + t[1] * e[1];
            shift = [e[0] * along, e[1] * along];
            fit.flat = Some(crate::heading([0.0, 0.0], [-e[1], e[0]]));
        }
        fit.after = cost(shift);
        let on = walls.iter().filter(|p| field.at(p[0] + shift[0], p[2] + shift[1]) < 0.15).count();
        fit.overlap = on as f32 / walls.len() as f32;
        let edge = shift[0].abs().max(shift[1].abs()) > SEARCH - STEP;
        if big <= 0.0 {
            fit.why = "no shape to fit";
        } else if fit.overlap < MIN_OVERLAP {
            fit.why = "does not overlap the map";
        } else if edge {
            fit.why = "off by more than the search";
        } else {
            fit.across = true;
        }
        if !fit.across {
            shift = [0.0, 0.0];
        }
    }

    // Up: floor points (near, not over the eyes) on the map's surfaces.
    let mut gaps: Vec<f32> = obs
        .points
        .iter()
        .filter(|p| p[1] < obs.eye[1] - 0.3 && (NEAR..FLOOR_FAR).contains(&range(p)) && obs.keeps(**p))
        .step_by(4)
        .filter_map(|&p| {
            let q = to_map(p);
            let q = [q[0] + shift[0], q[1], q[2] + shift[1]];
            let col = map.column(cell_of(q[0], q[2]))?;
            col.surfaces()
                .into_iter()
                .filter(|s| s.seen && (s.h - q[1]).abs() < UP_MAX + 0.05)
                .map(|s| s.h - q[1])
                .min_by(|a, b| a.abs().total_cmp(&b.abs()))
        })
        .collect();
    let mut up = 0.0;
    if gaps.len() >= UP_POINTS {
        gaps.sort_by(f32::total_cmp);
        up = gaps[gaps.len() / 2].clamp(-UP_MAX, UP_MAX);
        fit.up = true;
    }
    fit.shift = [shift[0], up, shift[1]];
    if !fit.across && fit.why.is_empty() {
        fit.why = "not taken";
    }
    fit
}

/// Keeps about `n` of `v`, evenly.
fn thin<T: Copy>(v: &mut Vec<T>, n: usize) {
    if v.len() > n {
        let every = v.len() as f32 / n as f32;
        *v = (0..n).map(|i| v[(i as f32 * every) as usize]).collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{floor, look, wall_x, wall_z};

    /// A room 8 x 8 with a few walls and a pillar; the bot in it.
    fn room() -> Vec<[f32; 3]> {
        let mut pts = Vec::new();
        floor(&mut pts, (-4.0, 4.0), (-4.0, 4.0), 0.0);
        wall_x(&mut pts, (-4.0, 4.0), -4.0, 0.0, 2.5);
        wall_x(&mut pts, (-4.0, 1.0), 3.0, 0.0, 2.5);
        wall_z(&mut pts, (-4.0, 1.0), 3.0, 0.0, 2.5);
        // A pillar.
        for k in 0..20 {
            for a in 0..12 {
                let t = a as f32 * std::f32::consts::PI / 6.0;
                pts.push([-1.5 + 0.2 * t.cos(), 0.1 * k as f32, -1.5 + 0.2 * t.sin()]);
            }
        }
        pts
    }

    #[test]
    fn finds_the_drift() {
        let pts = room();
        let mut m = WorldMap::default();
        // Mapped from the middle.
        m.integrate(&look(&pts, [0.0; 3], 1.5), [0.0; 3], 0.0);
        // Then seen from (0.5, 0, 0.5), but the odometry says (0.3, 0.1, 0.6).
        let truth = [0.5, 0.0, 0.5];
        let obs = look(&pts, truth, 1.5);
        let fit = register(&m, &obs, [0.3, 0.1, 0.6], 0.0);
        assert!(fit.across && fit.up, "{fit:?}");
        assert!((fit.shift[0] - 0.2).abs() < 0.03 && (fit.shift[2] + 0.1).abs() < 0.03 && (fit.shift[1] + 0.1).abs() < 0.03, "{fit:?}");
        assert!(fit.flat.is_none(), "{fit:?}");
    }

    #[test]
    fn a_corridor_leaves_its_length_to_the_odometry() {
        // Two long walls along z (x = -1, x = 1): any shift along z fits.
        let mut pts = Vec::new();
        floor(&mut pts, (-1.0, 1.0), (-8.0, 8.0), 0.0);
        for x in [-1.0f32, 1.0] {
            wall_z(&mut pts, (-8.0, 8.0), x, 0.0, 2.5);
        }
        let mut m = WorldMap::default();
        m.integrate(&look(&pts, [0.0; 3], 1.5), [0.0; 3], 0.0);
        let obs = look(&pts, [0.0, 0.0, -0.5], 1.5);
        let fit = register(&m, &obs, [0.2, 0.0, -0.3], 0.0);
        assert!(fit.across, "{fit:?}");
        // Across the corridor, put right; along it, left as it was.
        assert!((fit.shift[0] + 0.2).abs() < 0.03 && fit.shift[2].abs() < 0.05, "{fit:?}");
        assert!(fit.flat.is_some_and(|f| f.abs() < 10.0 || (f.abs() - 180.0).abs() < 10.0), "{fit:?}");
    }

    #[test]
    fn nothing_mapped_yet_takes_nothing() {
        let pts = room();
        let m = WorldMap::default();
        let fit = register(&m, &look(&pts, [0.0; 3], 1.5), [0.0; 3], 0.0);
        assert!(!fit.across && !fit.up && fit.shift == [0.0; 3], "{fit:?}");
    }
}
