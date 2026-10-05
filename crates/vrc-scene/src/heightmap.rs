//! A 2.5D grid over the ground around the bot, from stereo points in the
//! tracking space. Each cell keeps a histogram of the heights of its points
//! (up to a bit over the eyes; ceilings are left out), and from it:
//!
//! - its ground: the lowest height enough of its points rest on (a floor,
//!   a step, a sunken lounge, a table top whose legs were not seen);
//! - whether something stands on that ground taller than a step (a wall,
//!   a pillar, a person): then it is an obstacle.
//!
//! Grounds of different heights are what stairs, steps and platforms are:
//! walking between neighbouring cells is decided by their height difference
//! (see `candidates::reachable`), not by one floor for the whole map.
//!
//! Distances are the stereo's units (see `vrc_stereo`: world metres times a
//! constant per avatar height); `MapParams::in_units` turns world thresholds
//! into them. Stray matches are common far away and on flat colours, so a
//! cell needs several points, and a share of them, to call anything.

/// What a cell holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    /// Nothing (or too little) seen there.
    Unknown,
    /// Ground to stand on (at the height `HeightMap::cell` gives).
    Floor,
    /// Something standing on the ground, taller than a step.
    Obstacle,
}

/// Grid size and the heights that decide the cells.
#[derive(Clone, Copy, Debug)]
pub struct MapParams {
    /// Cell side.
    pub cell: f32,
    /// The grid spans this far each way from its centre.
    pub radius: f32,
    /// Highest step up the avatar walks.
    pub step: f32,
    /// Deepest step down it walks off.
    pub drop: f32,
    /// Highest ledge it can jump onto.
    pub jump: f32,
    /// Points above the eyes by more than this are ceiling, left out.
    pub above_eye: f32,
    /// Points closer than this (horizontally) to the eyes are the bot's own
    /// body, left out.
    pub self_radius: f32,
    /// Points farther than this (horizontally) are too coarse to place.
    pub max_range: f32,
    /// Points a cell needs to be anything but unknown.
    pub min_points: u32,
    /// Share of a cell's points standing more than a step over its ground
    /// that makes it an obstacle.
    pub min_above_share: f32,
}

impl Default for MapParams {
    fn default() -> Self {
        MapParams {
            cell: 0.1,
            radius: 8.0,
            step: 0.3,
            drop: 1.0,
            jump: 0.9,
            above_eye: 0.3,
            self_radius: 0.9,
            max_range: 8.0,
            min_points: 4,
            min_above_share: 0.2,
        }
    }
}

impl MapParams {
    /// These parameters with the world-metre ones (`step`, `drop`, `jump`,
    /// `self_radius`) given in world metres, turned into stereo units at
    /// `metres` world metres per unit.
    pub fn in_units(self, metres: f32) -> MapParams {
        MapParams {
            step: self.step / metres,
            drop: self.drop / metres,
            jump: self.jump / metres,
            self_radius: self.self_radius / metres,
            ..self
        }
    }
}

/// Height histogram bin.
const BIN: f32 = 0.05;
/// The histogram reaches this far under the bot's floor (and 3 over it).
const DEPTH: f32 = 3.0;

pub struct HeightMap {
    pub params: MapParams,
    /// x, z of the grid's centre.
    pub origin: [f32; 2],
    /// Height of the floor under the bot (the start of every walk).
    pub floor: f32,
    /// Cells a side.
    pub size: usize,
    /// Height of the histograms' first bin.
    base: f32,
    bins: usize,
    hist: Vec<u16>,
    /// Points in each cell.
    pub count: Vec<u32>,
    blocked: Vec<bool>,
}

impl HeightMap {
    /// An empty map centred on `origin` (x, z), the bot's floor at `floor`.
    pub fn new(params: MapParams, origin: [f32; 2], floor: f32) -> HeightMap {
        let size = (2.0 * params.radius / params.cell).ceil() as usize;
        let base = floor - DEPTH;
        let bins = ((DEPTH + 3.0) / BIN).ceil() as usize;
        HeightMap {
            params,
            origin,
            floor,
            size,
            base,
            bins,
            hist: vec![0; size * size * bins],
            count: vec![0; size * size],
            blocked: vec![false; size * size],
        }
    }

    /// Index of the cell holding (x, z), if on the grid. Row 0 is the far
    /// -Z side (ahead at yaw 0), column 0 the -X side.
    pub fn index(&self, x: f32, z: f32) -> Option<usize> {
        let half = self.size as f32 / 2.0;
        let c = ((x - self.origin[0]) / self.params.cell + half).floor();
        let r = ((z - self.origin[1]) / self.params.cell + half).floor();
        (c >= 0.0 && r >= 0.0 && (c as usize) < self.size && (r as usize) < self.size)
            .then(|| r as usize * self.size + c as usize)
    }

    /// The centre (x, z) of cell `i`.
    pub fn centre(&self, i: usize) -> [f32; 2] {
        let half = self.size as f32 / 2.0;
        [
            self.origin[0] + ((i % self.size) as f32 + 0.5 - half) * self.params.cell,
            self.origin[1] + ((i / self.size) as f32 + 0.5 - half) * self.params.cell,
        ]
    }

    /// Adds points seen from `eye`.
    pub fn add(&mut self, points: &[[f32; 3]], eye: [f32; 3]) {
        let p = self.params;
        for pt in points {
            if pt[1] > eye[1] + p.above_eye {
                continue; // ceiling
            }
            let (dx, dz) = (pt[0] - eye[0], pt[2] - eye[2]);
            let d2 = dx * dx + dz * dz;
            if d2 < p.self_radius * p.self_radius || d2 > p.max_range * p.max_range {
                continue;
            }
            let b = ((pt[1] - self.base) / BIN).floor();
            if b < 0.0 || b as usize >= self.bins {
                continue;
            }
            if let Some(i) = self.index(pt[0], pt[2]) {
                let h = &mut self.hist[i * self.bins + b as usize];
                *h = h.saturating_add(1);
                self.count[i] += 1;
            }
        }
    }

    /// Marks the cells within `radius` of (x, z) as obstacles: something
    /// stopped a walk there that stereo did not see (glass, an invisible wall).
    pub fn mark_blocked(&mut self, x: f32, z: f32, radius: f32) {
        let k = (radius / self.params.cell).ceil() as i32;
        for dr in -k..=k {
            for dc in -k..=k {
                let (px, pz) = (x + dc as f32 * self.params.cell, z + dr as f32 * self.params.cell);
                if (px - x).hypot(pz - z) <= radius {
                    if let Some(i) = self.index(px, pz) {
                        self.blocked[i] = true;
                    }
                }
            }
        }
    }

    /// Whether a walk was stopped at cell `i` ([`HeightMap::mark_blocked`]).
    pub fn is_blocked(&self, i: usize) -> bool {
        self.blocked[i]
    }

    /// Cell `i`: what it is, and its ground height if it has one.
    pub fn cell(&self, i: usize) -> (Cell, Option<f32>) {
        let p = self.params;
        if self.blocked[i] {
            return (Cell::Obstacle, None);
        }
        let n = self.count[i];
        if n < p.min_points {
            return (Cell::Unknown, None);
        }
        let hist = &self.hist[i * self.bins..(i + 1) * self.bins];
        // The ground: the lowest bin holding a fair share of the points.
        let enough = ((n as f32 * 0.15).ceil() as u16).max(2);
        let Some(g) = hist.iter().position(|&c| c >= enough) else {
            // Points spread up and down with no ground among them: a wall
            // seen at a glance (its foot hidden), when most of them stand
            // over the bot's floor by more than a step.
            let over = ((self.floor + p.step - self.base) / BIN).ceil().max(0.0) as usize;
            let high: u32 = hist[over.min(self.bins)..].iter().map(|&c| c as u32).sum();
            if n >= 3 * p.min_points && high as f32 >= 0.6 * n as f32 {
                return (Cell::Obstacle, None);
            }
            return (Cell::Unknown, None);
        };
        let step_bins = (p.step / BIN).ceil() as usize;
        let top = (g + step_bins).min(self.bins);
        // Its height: the mean of the points from it up a step.
        let (mut sum, mut cnt) = (0.0f32, 0u32);
        for (k, &c) in hist[g..top].iter().enumerate() {
            sum += (self.base + (g + k) as f32 * BIN + BIN / 2.0) * c as f32;
            cnt += c as u32;
        }
        let ground = sum / cnt.max(1) as f32;
        let above: u32 = hist[top..].iter().map(|&c| c as u32).sum();
        if above >= 3 && above as f32 >= p.min_above_share * n as f32 {
            (Cell::Obstacle, Some(ground))
        } else {
            (Cell::Floor, Some(ground))
        }
    }

    /// Every cell.
    pub fn classify(&self) -> Vec<Cell> {
        (0..self.size * self.size).map(|i| self.cell(i).0).collect()
    }

    /// Every cell's ground height (floor cells only).
    pub fn grounds(&self) -> Vec<Option<f32>> {
        (0..self.size * self.size)
            .map(|i| match self.cell(i) {
                (Cell::Floor, h) => h,
                _ => None,
            })
            .collect()
    }

    /// A top-down RGB picture (`size` x `size`, ahead at yaw 0 up): floor
    /// green, bluer higher and darker lower than the bot's floor, obstacles
    /// red, unknown dark; the eye in white, looking along `yaw_deg`.
    pub fn render(&self, eye: [f32; 3], yaw_deg: f32) -> Vec<u8> {
        let mut rgb = vec![0u8; self.size * self.size * 3];
        for i in 0..self.size * self.size {
            let px = match self.cell(i) {
                (Cell::Unknown, _) => [40, 40, 48],
                (Cell::Floor, h) => {
                    let t = ((h.unwrap_or(self.floor) - self.floor) / (2.0 * self.params.step)).clamp(-1.0, 1.0);
                    let g = (170.0 + 70.0 * t) as u8;
                    [g / 2, g, (g as f32 * 0.5 + 100.0 * t.max(0.0)) as u8]
                }
                (Cell::Obstacle, _) => [220, 50, 50],
            };
            rgb[i * 3..i * 3 + 3].copy_from_slice(&px);
        }
        let (s, c) = yaw_deg.to_radians().sin_cos();
        for k in 0..(1.0 / self.params.cell) as i32 {
            let d = k as f32 * self.params.cell;
            if let Some(i) = self.index(eye[0] + s * d, eye[2] - c * d) {
                rgb[i * 3..i * 3 + 3].copy_from_slice(&[255, 255, 255]);
            }
        }
        rgb
    }

    /// How many cells of each kind: unknown, floor, obstacle.
    pub fn census(&self) -> [usize; 3] {
        let mut n = [0; 3];
        for c in self.classify() {
            n[c as usize] += 1;
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floor_pillar_table_step_and_stray_points() {
        let mut m = HeightMap::new(MapParams { radius: 3.0, ..Default::default() }, [0.0, 0.0], 0.0);
        let eye = [0.0, 1.6, 0.0];
        let mut pts = Vec::new();
        // Floor, 9 points a cell; none under the table (its legs unseen);
        // a sunken area (x > 2) 0.2 lower.
        for i in 0..180 {
            for j in 0..180 {
                let (x, z) = (i as f32 / 30.0 - 3.0 + 0.016, j as f32 / 30.0 - 3.0 + 0.016);
                if !((-1.1..-1.0).contains(&x) && (-1.1..-1.0).contains(&z)) {
                    pts.push([x, if x > 2.0 { -0.2 } else { 0.0 }, z]);
                }
            }
        }
        for k in 0..10 {
            pts.push([1.05, 0.1 * k as f32, -1.05]); // a pillar standing on the floor
            pts.push([-1.05, 0.75, -1.05]); // a table top
        }
        pts.push([1.55, 1.0, 1.55]); // one stray match over the floor
        m.add(&pts, eye);
        let at = |x: f32, z: f32| m.cell(m.index(x, z).unwrap());
        assert_eq!(at(1.05, -1.05).0, Cell::Obstacle);
        let (table, h) = at(-1.05, -1.05);
        assert_eq!(table, Cell::Floor);
        assert!((h.unwrap() - 0.75).abs() < 0.05, "{h:?}");
        assert_eq!(at(1.55, 1.55).0, Cell::Floor);
        let (sunk, h) = at(2.5, 0.0);
        assert_eq!(sunk, Cell::Floor);
        assert!((h.unwrap() + 0.2).abs() < 0.05, "{h:?}");
        assert_eq!(at(0.1, 0.1).0, Cell::Unknown); // own feet
    }

    #[test]
    fn a_wall_without_its_foot_is_an_obstacle() {
        let mut m = HeightMap::new(MapParams { radius: 3.0, ..Default::default() }, [0.0, 0.0], 0.0);
        let eye = [0.0, 1.6, 0.0];
        // Points all the way up a wall at z = -2.05, a few on its foot.
        let mut pts = Vec::new();
        for k in 0..40 {
            pts.push([0.05, 0.05 * k as f32, -2.05]);
        }
        pts.push([0.05, 0.0, -2.05]);
        // And a few stray points up in the air elsewhere: still unknown.
        for k in 0..4 {
            pts.push([1.05, 0.5 * k as f32, -1.05]);
        }
        m.add(&pts, eye);
        assert_eq!(m.cell(m.index(0.05, -2.05).unwrap()).0, Cell::Obstacle);
        assert_eq!(m.cell(m.index(1.05, -1.05).unwrap()).0, Cell::Unknown);
    }
}
