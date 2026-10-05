//! A 2.5D grid over the floor around the bot, from stereo points in the
//! tracking space. Each cell counts the points near the floor and the
//! points standing above it (up to a bit over the eyes; ceilings are left
//! out), and keeps the lowest and highest of them.
//!
//! Distances are the stereo's units (see `vrc_stereo`: world metres times a
//! constant per avatar height); the thresholds are given in the same units.
//! Single stray matches are common far away and on flat colours, so a cell
//! needs several points, and a share of them, to call something there.

/// What a cell holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    /// Nothing (or too little) seen there.
    Unknown,
    /// Floor, or something low enough to walk over.
    Floor,
    /// A raised flat top whose base was not seen (a table, a platform).
    Raised,
    /// Something standing on the floor, taller than a step.
    Obstacle,
}

/// Grid size and the heights that decide the cells.
#[derive(Clone, Copy, Debug)]
pub struct MapParams {
    /// Cell side.
    pub cell: f32,
    /// The grid spans this far each way from its centre.
    pub radius: f32,
    /// Highest bump that still counts as floor.
    pub step: f32,
    /// Points above the eyes by more than this are ceiling, left out.
    pub above_eye: f32,
    /// Points closer than this (horizontally) to the eyes are the bot's own
    /// body, left out.
    pub self_radius: f32,
    /// Points farther than this (horizontally) are too coarse to place.
    pub max_range: f32,
    /// Points a cell needs to be anything but unknown.
    pub min_points: u32,
    /// Share of a cell's points that must stand above the floor for an
    /// obstacle (or a raised top) there.
    pub min_above_share: f32,
}

impl Default for MapParams {
    fn default() -> Self {
        MapParams {
            cell: 0.1,
            radius: 8.0,
            step: 0.3,
            above_eye: 0.3,
            self_radius: 0.5,
            max_range: 8.0,
            min_points: 4,
            min_above_share: 0.2,
        }
    }
}

pub struct HeightMap {
    pub params: MapParams,
    /// x, z of the grid's centre.
    pub origin: [f32; 2],
    /// Height of the floor the cells are judged against.
    pub floor: f32,
    /// Cells a side.
    pub size: usize,
    pub low: Vec<f32>,
    pub high: Vec<f32>,
    /// Points in each cell.
    pub count: Vec<u32>,
    /// Of them, standing more than a step above the floor.
    pub above: Vec<u32>,
}

impl HeightMap {
    /// An empty map centred on `origin` (x, z), over a floor at `floor`.
    pub fn new(params: MapParams, origin: [f32; 2], floor: f32) -> HeightMap {
        let size = (2.0 * params.radius / params.cell).ceil() as usize;
        HeightMap {
            params,
            origin,
            floor,
            size,
            low: vec![f32::INFINITY; size * size],
            high: vec![f32::NEG_INFINITY; size * size],
            count: vec![0; size * size],
            above: vec![0; size * size],
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

    /// Adds points seen from `eye`.
    pub fn add(&mut self, points: &[[f32; 3]], eye: [f32; 3]) {
        let p = self.params;
        for pt in points {
            if pt[1] > eye[1] + p.above_eye || pt[1] < self.floor - p.step {
                continue; // ceiling, or under the floor (a mirror, a mismatch)
            }
            let (dx, dz) = (pt[0] - eye[0], pt[2] - eye[2]);
            let d2 = dx * dx + dz * dz;
            if d2 < p.self_radius * p.self_radius || d2 > p.max_range * p.max_range {
                continue;
            }
            if let Some(i) = self.index(pt[0], pt[2]) {
                self.low[i] = self.low[i].min(pt[1]);
                self.high[i] = self.high[i].max(pt[1]);
                self.count[i] += 1;
                if pt[1] > self.floor + p.step {
                    self.above[i] += 1;
                }
            }
        }
    }

    /// Every cell.
    pub fn classify(&self) -> Vec<Cell> {
        let p = self.params;
        (0..self.size * self.size)
            .map(|i| {
                let (n, up) = (self.count[i], self.above[i]);
                if n < p.min_points {
                    Cell::Unknown
                } else if (up as f32) < p.min_above_share * n as f32 || up < p.min_points.min(3) {
                    Cell::Floor
                } else if up == n && self.high[i] - self.low[i] < p.step {
                    Cell::Raised // only a flat top was seen
                } else {
                    Cell::Obstacle
                }
            })
            .collect()
    }

    /// A top-down RGB picture (`size` x `size`, ahead at yaw 0 up): floor
    /// green, obstacles red, raised tops orange, unknown dark; the eye in
    /// white, looking along `yaw_deg`.
    pub fn render(&self, eye: [f32; 3], yaw_deg: f32) -> Vec<u8> {
        let cells = self.classify();
        let mut rgb = vec![0u8; self.size * self.size * 3];
        for (i, cell) in cells.iter().enumerate() {
            let px = match cell {
                Cell::Unknown => [40, 40, 48],
                Cell::Floor => [90, 190, 90],
                Cell::Raised => [240, 160, 40],
                Cell::Obstacle => [220, 50, 50],
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

    /// How many cells of each kind: unknown, floor, raised, obstacle.
    pub fn census(&self) -> [usize; 4] {
        let mut n = [0; 4];
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
    fn floor_pillar_table_and_stray_points() {
        let mut m = HeightMap::new(MapParams { radius: 3.0, ..Default::default() }, [0.0, 0.0], 0.0);
        let eye = [0.0, 1.6, 0.0];
        let mut pts = Vec::new();
        // Floor, 9 points a cell; none under the table (its base unseen).
        for i in 0..180 {
            for j in 0..180 {
                let (x, z) = (i as f32 / 30.0 - 3.0 + 0.016, j as f32 / 30.0 - 3.0 + 0.016);
                if !((-1.1..-1.0).contains(&x) && (-1.1..-1.0).contains(&z)) {
                    pts.push([x, 0.0, z]);
                }
            }
        }
        for k in 0..10 {
            pts.push([1.05, 0.1 * k as f32, -1.05]); // a pillar standing on the floor
            pts.push([-1.05, 0.75, -1.05]); // a table top
        }
        pts.push([2.05, 1.0, 2.05]); // one stray match over the floor
        m.add(&pts, eye);
        let cells = m.classify();
        assert_eq!(cells[m.index(1.05, -1.05).unwrap()], Cell::Obstacle);
        assert_eq!(cells[m.index(-1.05, -1.05).unwrap()], Cell::Raised);
        assert_eq!(cells[m.index(2.05, 2.05).unwrap()], Cell::Floor);
        assert_eq!(cells[m.index(0.1, 0.1).unwrap()], Cell::Unknown); // own feet
    }
}
