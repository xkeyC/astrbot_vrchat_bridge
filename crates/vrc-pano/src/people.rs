//! People in a pano frame, by its depth (no detector: decided 2026-10-09,
//! a detector is unreliable on VRChat's avatars).
//!
//! - **Foreground**: a person stands out of what is behind: the depth 0.45 m
//!   to either side of them (measured round the eyes, across the faces) is
//!   farther by a clear margin. Walls, floors and the sides of furniture
//!   never are; someone a quarter of a metre in front of a wall is (on a
//!   real frame, 2026-10-09: 2.88 m against 3.14 m; E1C's levels, 6.6 cm
//!   there, fill such a gap, so depth alone does not split them).
//! - **Named**: a nameplate read somewhere (the user camera's lens, or the
//!   eyes' overlay) gives a ray from where it was seen, through the plate.
//!   The person stands under the plate: [`Cloud::person_along`] looks for
//!   the nearest foreground thing above the floor in a narrow band along
//!   that ray, below it, its top a little under the plate. Its distance and
//!   the floor put the feet in the world.
//! - **Maybe someone**: [`Cloud::bodies`]: upright clusters of foreground
//!   above the floor, person-sized (a footprint up to about a metre,
//!   0.9-2.3 m tall, down to near the floor). Never named without a plate
//!   read.
//! - **Kept between reads**: [`nearest`] by position (the follower).
//!
//! Positions are Unity's world (metres). [`Tracking`] turns them into the
//! bot's tracking space (the follower's and the surveys' frame).

use rayon::prelude::*;
use vrc_vr::tap::EyeFrame;

use crate::frame::PanoFrame;
use crate::layout::Face;

/// The depth's points, for looking for people.
#[derive(Clone, Debug)]
pub struct Cloud {
    /// Unity's world (metres): every `step`-th pixel of every face with depth.
    pub points: Vec<[f32; 3]>,
    /// Each point standing out of what is behind it (level faces only).
    pub fg: Vec<bool>,
    /// The cameras' centre (the eyes).
    pub centre: [f32; 3],
    /// The floor under the bot (world y): the median of the down face's
    /// points within FLOOR_RADIUS_M round under it; none without.
    pub floor: Option<f32>,
}

/// Round under the bot, the floor's points.
const FLOOR_RADIUS_M: f32 = 1.2;
/// Round where someone stands, their ground's points (and how many).
const GROUND_RADIUS_M: f32 = 0.6;
const GROUND_MIN_POINTS: usize = 8;
/// Foreground: the depth this far to either side (metres, round the eyes)
/// is farther by at least FG_MARGIN_M or FG_MARGIN of the range.
const FG_SIDE_M: f32 = 0.45;
const FG_MARGIN_M: f32 = 0.15;
const FG_MARGIN: f32 = 0.05;
const FG_LOWER_M: f32 = 0.15;
/// The front of a body is seen; its middle is this much farther.
const BODY_HALF_DEPTH_M: f32 = 0.12;

/// How people are told in the depth.
#[derive(Clone, Debug)]
pub struct PeopleParams {
    /// Above the floor by less than this: the floor (metres).
    pub min_up: f32,
    /// Off the plate's ray sideways by at most this (half a body's width
    /// and a little).
    pub lateral: f32,
    /// The plate floats this far over the top of the head (metres; VRChat:
    /// about 0.3-0.5 m).
    pub plate_over: (f32, f32),
    /// Upright: at least this tall (metres).
    pub min_height: f32,
    /// Points along the ray closer than this to each other (metres) are one
    /// thing.
    pub gap: f32,
    /// At least this many points (of every 4th pixel) to be a body.
    pub min_points: usize,
    /// Out to this far (metres).
    pub max_range: f32,
    /// Maybe someone: footprint at most this across, this tall at least
    /// and at most, its lowest point at most this over the floor.
    pub body_across: f32,
    pub body_height: (f32, f32),
    pub body_low: f32,
    /// ... and its foreground at least this wide across the line of sight.
    pub body_width: f32,
}

impl Default for PeopleParams {
    fn default() -> Self {
        PeopleParams {
            min_up: 0.2,
            lateral: 0.3,
            plate_over: (0.05, 0.9),
            min_height: 0.6,
            gap: 0.25,
            min_points: 12,
            max_range: 12.0,
            body_across: 1.1,
            body_height: (0.9, 2.3),
            body_low: 0.6,
            body_width: 0.18,
        }
    }
}

/// Someone (or something person-shaped) in the depth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body {
    /// Where they stand: on the floor under them (Unity's world).
    pub feet: [f32; 3],
    /// The top of the head (world y), and their lowest point seen (their
    /// feet, or what hides them: up on something, it is over the floor).
    pub top: f32,
    pub low: f32,
    /// Horizontally from the eyes (metres).
    pub distance: f32,
    /// The depth's points on them.
    pub points: usize,
}

impl Body {
    /// Where the head looks to them: the world yaw (clockwise from +z, the
    /// beacon's convention) from `from`.
    pub fn yaw_from(&self, from: [f32; 3]) -> f32 {
        (self.feet[0] - from[0]).atan2(self.feet[2] - from[2]).to_degrees().rem_euclid(360.0)
    }

    /// Their plate, as VRChat floats it (0.35 m over the head).
    pub fn plate(&self) -> [f32; 3] {
        [self.feet[0], self.top + 0.35, self.feet[2]]
    }
}

impl Cloud {
    /// The points of every `step`-th pixel (both ways) of the faces.
    pub fn new(frame: &PanoFrame, step: u32) -> Cloud {
        let all = frame.points(step);
        let centre = frame.code.position;
        let mut under: Vec<f32> =
            all.iter().filter(|p| p.face == Face::Down && (p.world[0] - centre[0]).hypot(p.world[2] - centre[2]) < FLOOR_RADIUS_M).map(|p| p.world[1]).collect();
        let fg: Vec<bool> = all.par_iter().map(|p| p.face.yaw().is_some() && foreground(frame, centre, p.world)).collect();
        let points: Vec<[f32; 3]> = all.into_iter().map(|p| p.world).collect();
        under.sort_by(f32::total_cmp);
        let floor = (under.len() >= 20).then(|| under[under.len() / 2]);
        Cloud { points, fg, centre, floor }
    }

    /// The ground round (`x`, `z`) (world y): the lowest point within
    /// GROUND_RADIUS_M of it and below `under`, when enough points are
    /// there (someone higher or lower than the bot stands on their own).
    fn ground_at(&self, x: f32, z: f32, under: f32) -> Option<f32> {
        let mut ys: Vec<f32> = self
            .points
            .iter()
            .filter(|q| q[1] < under - 0.5 && (q[0] - x).hypot(q[2] - z) <= GROUND_RADIUS_M)
            .map(|q| q[1])
            .collect();
        if ys.len() < GROUND_MIN_POINTS {
            return None;
        }
        ys.sort_by(|a, b| a.total_cmp(b));
        // A low percentile, not the very lowest: stray points under it.
        Some(ys[ys.len() / 10])
    }

    /// The person under a plate seen along `dir` from `origin` (world: the
    /// lens, or the eyes), the nearest along it.
    pub fn person_along(&self, origin: [f32; 3], dir: [f32; 3], p: &PeopleParams) -> Option<Body> {
        let h = dir[0].hypot(dir[2]);
        if h < 0.05 {
            return None;
        }
        let (ux, uz, slope) = (dir[0] / h, dir[2] / h, dir[1] / h);
        // Points in the band along the ray, under it, over the floor:
        // (along, lateral, point).
        let mut band: Vec<(f32, f32, [f32; 3])> = Vec::new();
        for (&q, _) in self.points.iter().zip(&self.fg).filter(|(_, f)| **f) {
            let (dx, dz) = (q[0] - origin[0], q[2] - origin[2]);
            let along = dx * ux + dz * uz;
            if along < 0.3 || along > p.max_range {
                continue;
            }
            let lateral = -dx * uz + dz * ux;
            let ray_y = origin[1] + along * slope;
            // Under the ray only: someone may stand higher or lower than the
            // bot, so their own ground is found per thing, not the bot's.
            if q[1] > ray_y + 0.05 {
                continue;
            }
            if lateral.abs() <= p.lateral {
                band.push((along, lateral, q));
            }
        }
        band.sort_by(|a, b| a.0.total_cmp(&b.0));
        // Things along the ray, nearest first: runs of points with no gap
        // over `gap`.
        let mut start = 0;
        while start < band.len() {
            let mut end = start + 1;
            while end < band.len() && band[end].0 - band[end - 1].0 <= p.gap {
                end += 1;
            }
            let run = &band[start..end];
            start = end;
            if run.len() < p.min_points {
                continue;
            }
            let mid = run[run.len() / 2].0;
            let top = run.iter().map(|r| r.2[1]).fold(f32::NEG_INFINITY, f32::max);
            let ray_y = origin[1] + mid * slope;
            // The front of them is seen: the middle is a little farther.
            let n = run.len() as f32;
            let (mx, mz) = (run.iter().map(|r| r.2[0]).sum::<f32>() / n + BODY_HALF_DEPTH_M * ux, run.iter().map(|r| r.2[2]).sum::<f32>() / n + BODY_HALF_DEPTH_M * uz);
            // Their own ground: the lowest point round where they stand (the
            // bot's floor when nothing lower is seen there).
            let ground = self.ground_at(mx, mz, top).or(self.floor);
            let Some(ground) = ground else { continue };
            let low = run.iter().map(|r| r.2[1]).fold(f32::INFINITY, f32::min);
            // The plate floats over the head, the thing is upright and
            // reaches down near its ground.
            if !(p.plate_over.0..=p.plate_over.1).contains(&(ray_y - top)) || top - ground < p.min_height || low - ground > p.min_up + 0.5 {
                continue;
            }
            return Some(Body { feet: [mx, ground, mz], top, low, distance: (mx - self.centre[0]).hypot(mz - self.centre[2]), points: run.len() });
        }
        None
    }

    /// Upright, person-sized clusters above the floor ("maybe someone").
    pub fn bodies(&self, p: &PeopleParams) -> Vec<Body> {
        const CELL: f32 = 0.1;
        let Some(floor) = self.floor else { return Vec::new() };
        let span = p.max_range + 1.0;
        let n = (2.0 * span / CELL) as usize;
        let index = |x: f32, z: f32| -> Option<usize> {
            let (i, j) = (((x - self.centre[0] + span) / CELL) as isize, ((z - self.centre[2] + span) / CELL) as isize);
            (i >= 0 && j >= 0 && (i as usize) < n && (j as usize) < n).then(|| j as usize * n + i as usize)
        };
        // Per cell: points, their sums, the highest and lowest, which.
        let mut cells: std::collections::HashMap<usize, (usize, f32, f32, f32, f32)> = std::collections::HashMap::new();
        let mut members_of: std::collections::HashMap<usize, Vec<[f32; 3]>> = std::collections::HashMap::new();
        for (&q, _) in self.points.iter().zip(&self.fg).filter(|(_, f)| **f) {
            let up = q[1] - floor;
            let d = (q[0] - self.centre[0]).hypot(q[2] - self.centre[2]);
            if !(0.3..=p.body_height.1 + 0.1).contains(&up) || !(0.4..=p.max_range).contains(&d) {
                continue;
            }
            let Some(k) = index(q[0], q[2]) else { continue };
            let c = cells.entry(k).or_insert((0, 0.0, 0.0, f32::NEG_INFINITY, f32::INFINITY));
            (c.0, c.1, c.2, c.3, c.4) = (c.0 + 1, c.1 + q[0], c.2 + q[2], c.3.max(q[1]), c.4.min(q[1]));
            members_of.entry(k).or_default().push(q);
        }
        cells.retain(|_, c| c.0 >= 3);
        // Connected (8 ways), one cluster each.
        let mut seen: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut out = Vec::new();
        let keys: Vec<usize> = cells.keys().copied().collect();
        for k in keys {
            if !seen.insert(k) {
                continue;
            }
            let mut stack = vec![k];
            let mut members = Vec::new();
            while let Some(c) = stack.pop() {
                members.push(c);
                let (i, j) = ((c % n) as isize, (c / n) as isize);
                for (di, dj) in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)] {
                    let (a, b) = (i + di, j + dj);
                    if a < 0 || b < 0 || a as usize >= n || b as usize >= n {
                        continue;
                    }
                    let m = b as usize * n + a as usize;
                    if cells.contains_key(&m) && seen.insert(m) {
                        stack.push(m);
                    }
                }
            }
            let (mut count, mut sx, mut sz, mut top, mut low) = (0usize, 0.0f32, 0.0f32, f32::NEG_INFINITY, f32::INFINITY);
            let (mut i0, mut i1, mut j0, mut j1) = (usize::MAX, 0, usize::MAX, 0);
            for &m in &members {
                let c = cells[&m];
                (count, sx, sz, top, low) = (count + c.0, sx + c.1, sz + c.2, top.max(c.3), low.min(c.4));
                let (i, j) = (m % n, m / n);
                (i0, i1, j0, j1) = (i0.min(i), i1.max(i), j0.min(j), j1.max(j));
            }
            let across = (((i1 - i0 + 1) as f32).hypot((j1 - j0 + 1) as f32)) * CELL;
            let tall = top - floor;
            if count < p.min_points * 2 || across > p.body_across || !(p.body_height.0..=p.body_height.1).contains(&tall) || low - floor > p.body_low {
                continue;
            }
            let (mx, mz) = (sx / count as f32, sz / count as f32);
            // Across the line of sight, a person's foreground is a body's
            // width; an object's end seen at a slant is a sliver.
            let (dx, dz) = (mx - self.centre[0], mz - self.centre[2]);
            let n_ = dx.hypot(dz).max(1e-3);
            let (lo_l, hi_l) = members.iter().flat_map(|m| members_of[m].iter()).map(|q| (-(q[0] - self.centre[0]) * dz + (q[2] - self.centre[2]) * dx) / n_).fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), l| (a.min(l), b.max(l)));
            if hi_l - lo_l < p.body_width {
                continue;
            }
            // The front of them is seen: the middle is a little farther.
            let (mx, mz) = (mx + BODY_HALF_DEPTH_M * dx / n_, mz + BODY_HALF_DEPTH_M * dz / n_);
            out.push(Body { feet: [mx, floor, mz], top, low, distance: (mx - self.centre[0]).hypot(mz - self.centre[2]), points: count });
        }
        out.sort_by(|a, b| a.distance.total_cmp(&b.distance));
        out
    }
}

/// Whether world point `p` stands out of what is behind it: the depth
/// FG_SIDE_M to either side of it, round `centre`, at its height and
/// FG_LOWER_M lower, is farther by a clear margin (none there, the sky:
/// farther; the UI: unknown, not farther).
fn foreground(frame: &PanoFrame, centre: [f32; 3], p: [f32; 3]) -> bool {
    let d = [p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]];
    let h = d[0].hypot(d[2]);
    if h < 0.3 {
        return false;
    }
    let r = (h * h + d[1] * d[1]).sqrt();
    let need = FG_MARGIN_M.max(FG_MARGIN * r);
    let a = FG_SIDE_M.atan2(h);
    // At its height and a little lower: a horizontal edge (a cabinet's top
    // seen against the wall) has the wall beside it at its height only.
    [(-a, 0.0), (a, 0.0), (-a, FG_LOWER_M), (a, FG_LOWER_M)].iter().all(|&(a, lower)| {
        let (sn, cs) = a.sin_cos();
        let dir = [d[0] * cs + d[2] * sn, d[1] - lower, -d[0] * sn + d[2] * cs];
        let Some((k, u, v)) = frame.project(dir) else { return true };
        match frame.point(k, u as u32, v as u32) {
            Some(q) => ((q[0] - centre[0]).powi(2) + (q[1] - centre[1]).powi(2) + (q[2] - centre[2]).powi(2)).sqrt() > r + need,
            None => frame.views[k].mask[(v as u32 * frame.views[k].width + u as u32) as usize] & crate::frame::MASK_OVERLAY == 0,
        }
    })
}

/// The body nearest `at` (world feet), within `gate` metres.
pub fn nearest(bodies: &[Body], at: [f32; 3], gate: f32) -> Option<usize> {
    bodies
        .iter()
        .enumerate()
        .map(|(i, b)| (i, (b.feet[0] - at[0]).hypot(b.feet[2] - at[2])))
        .filter(|(_, d)| *d <= gate)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

/// Unity's world to the bot's tracking space as one frame shows both: the
/// head (the beacon, world) and the eyes' poses (tracking). The tracking
/// space turns with the playspace (`offset`: the beacon's yaw less the
/// tracking yaw of the same look) and is scaled with the avatar
/// (`metres`: world metres a tracking unit).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tracking {
    pub head_world: [f32; 3],
    pub head_track: [f32; 3],
    /// Degrees.
    pub offset: f32,
    pub metres: f32,
}

impl Tracking {
    /// From a frame: `pano`'s head (world) and `eyes`' poses (tracking),
    /// the floor in both (world: the depth's; tracking: where the headset's
    /// floor is) for the scale; 1 when either is missing or odd.
    pub fn new(pano: &PanoFrame, eyes: &EyeFrame, floor_world: Option<f32>, floor_track: f32) -> Tracking {
        let (a, b) = (eyes.views[0].pose.position, eyes.views[1].pose.position);
        let head_track = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
        let track_yaw = eyes.views[0].pose.yaw_pitch().0;
        let head_world = pano.head.position;
        let metres = match floor_world {
            Some(f) if head_track[1] - floor_track > 0.1 && head_world[1] - f > 0.1 => (head_world[1] - f) / (head_track[1] - floor_track),
            _ => 1.0,
        };
        Tracking { head_world, head_track, offset: crate::frame::wrap(pano.head.yaw - track_yaw), metres: metres.clamp(0.1, 10.0) }
    }

    /// A world point in the tracking space.
    pub fn point(&self, w: [f32; 3]) -> [f32; 3] {
        // Map axes (x, y, -z: right-handed, -z ahead, as the tracking
        // space), turned by -offset.
        let (dx, dy, dz) = (w[0] - self.head_world[0], w[1] - self.head_world[1], -(w[2] - self.head_world[2]));
        let (s, c) = (-self.offset).to_radians().sin_cos();
        let (tx, tz) = (dx * c - dz * s, dx * s + dz * c);
        let m = self.metres;
        [self.head_track[0] + tx / m, self.head_track[1] + dy / m, self.head_track[2] + tz / m]
    }

    /// A tracking point in the world.
    pub fn to_world(&self, p: [f32; 3]) -> [f32; 3] {
        let m = self.metres;
        let d = [(p[0] - self.head_track[0]) * m, (p[1] - self.head_track[1]) * m, (p[2] - self.head_track[2]) * m];
        let w = self.dir_to_world(d);
        [self.head_world[0] + w[0], self.head_world[1] + w[1], self.head_world[2] + w[2]]
    }

    /// A tracking direction (or offset, unscaled) in the world's axes.
    pub fn dir_to_world(&self, d: [f32; 3]) -> [f32; 3] {
        let (s, c) = self.offset.to_radians().sin_cos();
        let (mx, mz) = (d[0] * c - d[2] * s, d[0] * s + d[2] * c);
        [mx, d[1], -mz]
    }

    /// A world yaw (the beacon's) as a tracking yaw (+ right of -z).
    pub fn yaw(&self, world_yaw: f32) -> f32 {
        crate::frame::wrap(world_yaw - self.offset)
    }

    /// A tracking yaw as a world yaw (0..360).
    pub fn world_yaw(&self, track_yaw: f32) -> f32 {
        (track_yaw + self.offset).rem_euclid(360.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{decode, PanoParams};
    use crate::synth::{person, Scene};

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    /// The room with `boxes`, decoded; its cloud.
    fn cloud(s: &Scene) -> Cloud {
        Cloud::new(&decode(&s.render(), &PanoParams::default()).unwrap(), 4)
    }

    /// `d` metres from the rig along world yaw `yaw`.
    fn out(s: &Scene, yaw: f32, d: f32) -> (f32, f32) {
        let (sn, cs) = yaw.to_radians().sin_cos();
        (s.position[0] + d * sn, s.position[2] + d * cs)
    }

    #[test]
    fn the_floor_is_under_the_bot() {
        let s = Scene::room();
        let c = cloud(&s);
        assert!(close(c.floor.unwrap(), s.room_min[1], 0.02), "{:?}", c.floor);
        // Nobody there: no person-shaped thing (the walls are too wide).
        assert!(c.bodies(&PeopleParams::default()).is_empty());
    }

    #[test]
    fn a_plate_ray_finds_the_person_under_it() {
        let mut s = Scene::room();
        // Someone 2.2 m out, 40 degrees right of the rig's ahead; 1.6 m tall.
        let yaw = s.rig_yaw + 40.0;
        let (x, z) = out(&s, yaw, 2.2);
        s.boxes.push(person(&s, x, z, 1.6));
        let c = cloud(&s);
        let p = PeopleParams::default();
        let plate = [x, s.room_min[1] + 1.6 + 0.4, z];
        // From the eyes...
        let from_eyes = |o: [f32; 3]| [plate[0] - o[0], plate[1] - o[1], plate[2] - o[2]];
        let b = c.person_along(s.position, from_eyes(s.position), &p).expect("the person");
        assert!(close(b.feet[0], x, 0.12) && close(b.feet[2], z, 0.12), "{b:?} vs ({x}, {z})");
        assert!(close(b.feet[1], s.room_min[1], 0.02) && close(b.top, s.room_min[1] + 1.6, 0.06), "{b:?}");
        assert!(close(b.distance, 2.2, 0.15) && close(crate::frame::wrap(b.yaw_from(s.position) - yaw), 0.0, 3.0), "{b:?}");
        // ... and from a lens behind and above the head: the same person.
        let (bx, bz) = out(&s, s.rig_yaw + 180.0, 0.35);
        let lens = [bx, s.position[1] + 0.35, bz];
        let b2 = c.person_along(lens, from_eyes(lens), &p).expect("from the lens");
        assert!((b2.feet[0] - b.feet[0]).hypot(b2.feet[2] - b.feet[2]) < 0.1, "{b2:?} {b:?}");
        // As someone maybe there, unnamed.
        let all = c.bodies(&p);
        assert_eq!(all.len(), 1, "{all:?}");
        assert!((all[0].feet[0] - x).hypot(all[0].feet[2] - z) < 0.2, "{all:?}");
        // Kept by position: a little off is them, far off is not.
        assert_eq!(nearest(&all, [x + 0.3, 0.0, z], 0.6), Some(0));
        assert_eq!(nearest(&all, [x + 2.0, 0.0, z], 0.6), None);
    }

    #[test]
    fn a_plate_ray_with_nobody_under_it_finds_none() {
        let mut s = Scene::room();
        let (x, z) = out(&s, s.rig_yaw, 2.5);
        s.boxes.push(person(&s, x, z, 1.6));
        let c = cloud(&s);
        // Toward a wall, 90 degrees off the person.
        let (wx, wz) = out(&s, s.rig_yaw + 90.0, 3.0);
        let dir = [wx - s.position[0], 0.3, wz - s.position[2]];
        assert!(c.person_along(s.position, dir, &PeopleParams::default()).is_none());
        // Up at the ceiling: none.
        assert!(c.person_along(s.position, [0.0, 1.0, 0.01], &PeopleParams::default()).is_none());
    }

    #[test]
    fn someone_by_a_wall_is_found_and_low_things_are_not_people() {
        let mut s = Scene::room();
        // The room's +z wall is at 1.5; the rig at z -3, x 2: someone 0.5 m
        // from it straight across, a table (0.75 m) and a cabinet (2.5 m
        // wide) elsewhere.
        s.boxes.push(person(&s, 2.0, 0.85, 1.7));
        let y = s.room_min[1];
        s.boxes.push(([3.5, y, -5.0], [4.5, y + 0.75, -4.2], [120, 90, 60]));
        s.boxes.push(([-1.0, y, -6.9], [1.5, y + 1.8, -6.4], [120, 90, 60]));
        let c = cloud(&s);
        let p = PeopleParams::default();
        let plate = [2.0, y + 1.7 + 0.35, 0.85];
        let o = s.position;
        let b = c.person_along(o, [plate[0] - o[0], plate[1] - o[1], plate[2] - o[2]], &p).expect("by the wall");
        assert!(close(b.feet[2], 0.85, 0.15) && close(b.feet[0], 2.0, 0.15), "{b:?}");
        let all = c.bodies(&p);
        assert_eq!(all.len(), 1, "{all:?}");
        assert!(close(all[0].feet[2], 0.85, 0.2));
    }

    #[test]
    fn someone_lower_than_the_bot_is_found_on_their_own_ground() {
        let mut s = Scene::room();
        let y = s.room_min[1];
        // The bot on a 1.2 m platform; someone on the room's floor 3 m out
        // (seen live: their body under the bot's floor was never looked at,
        // and a plant by the bot was taken for them).
        let up = 1.2;
        let (px, pz) = (s.position[0], s.position[2]);
        s.boxes.push(([px - 0.8, y, pz - 0.8], [px + 0.8, y + up, pz + 0.8], [90, 90, 90]));
        s.position[1] += up;
        let yaw = s.rig_yaw + 20.0;
        let (x, z) = out(&s, yaw, 3.0);
        s.boxes.push(person(&s, x, z, 1.6));
        let c = cloud(&s);
        assert!(close(c.floor.unwrap(), y + up, 0.05), "{:?}", c.floor);
        let plate = [x, y + 1.6 + 0.4, z];
        let o = s.position;
        let b = c.person_along(o, [plate[0] - o[0], plate[1] - o[1], plate[2] - o[2]], &PeopleParams::default()).expect("below the bot");
        assert!(close(b.feet[1], y, 0.1), "{b:?}");
        assert!(close(b.feet[0], x, 0.2) && close(b.feet[2], z, 0.2), "{b:?} vs ({x}, {z})");
        assert!(close(b.distance, 3.0, 0.3), "{b:?}");
    }

    #[test]
    fn world_to_tracking() {
        // The tracking space turned 90 degrees from the world (the beacon's
        // yaw 90 where the headset's is 0), half the world's scale.
        let t = Tracking { head_world: [10.0, 1.5, 20.0], head_track: [0.0, 1.6, 0.0], offset: 90.0, metres: 2.0 };
        // 2 m along world +x (world yaw 90): ahead in the tracking space
        // (-z), 1 unit.
        let p = t.point([12.0, 1.5, 20.0]);
        assert!(close(p[0], 0.0, 1e-5) && close(p[2], -1.0, 1e-5) && close(p[1], 1.6, 1e-5), "{p:?}");
        // World +z (yaw 0) is the tracking space's yaw -90: to the left.
        let q = t.point([10.0, 2.5, 22.0]);
        assert!(close(q[0], -1.0, 1e-5) && close(q[2], 0.0, 1e-5) && close(q[1], 2.1, 1e-5), "{q:?}");
        assert!(close(t.yaw(90.0), 0.0, 1e-4) && close(t.yaw(0.0), -90.0, 1e-4) && close(t.world_yaw(-90.0), 0.0, 1e-4));
        // And back.
        for w in [[12.0f32, 1.5, 20.0], [9.0, 0.3, 24.0]] {
            let b = t.to_world(t.point(w));
            assert!((0..3).all(|k| close(b[k], w[k], 1e-4)), "{b:?} {w:?}");
        }
        // Ahead in the tracking space is world +x (yaw 90).
        let d = t.dir_to_world([0.0, 0.0, -1.0]);
        assert!(close(d[0], 1.0, 1e-5) && close(d[2], 0.0, 1e-5), "{d:?}");
    }
}
