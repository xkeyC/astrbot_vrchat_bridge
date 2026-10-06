//! The map from above, round the bot: what it stands on green (lighter
//! higher, darker lower than its feet), walked ways blue (by players:
//! teal), what is in the way red, unknown dark; marks magenta, objects
//! yellow, the planned way white, the bot a white dot with its heading.
//! Of several floors, the one nearest the bot's feet.

use crate::plan::{PlanParams, Planner};
use crate::{cell_of, Belief, WorldMap, CELL};

pub struct Picture {
    pub side: usize,
    pub rgb: Vec<u8>,
}

/// `radius` metres round `pose` (feet), `px` pixels a column; up is the
/// map's -z, or `up` (a heading, degrees) when given; `path` drawn.
pub fn render(map: &WorldMap, pose: [f32; 3], heading: f32, radius: f32, px: usize, up: Option<f32>, path: &[[f32; 3]], p: &PlanParams) -> Picture {
    let cells = (2.0 * radius / CELL).round() as usize;
    let side = cells * px;
    let mut rgb = vec![0u8; side * side * 3];
    let mut planner = Planner::new(map, *p, pose);
    let rot = up.unwrap_or(0.0).to_radians();
    let (s, c) = rot.sin_cos();
    // Picture (right, down from the middle, metres) to map (x, z): turned
    // so `up` points up.
    let to_map = |u: f32, v: f32| [pose[0] + u * c - v * s, pose[2] + u * s + v * c];
    let to_pic = |x: f32, z: f32| {
        let (dx, dz) = (x - pose[0], z - pose[2]);
        let (u, v) = (dx * c + dz * s, -dx * s + dz * c);
        ((u + radius) / CELL * px as f32, (v + radius) / CELL * px as f32)
    };
    let mut cache: std::collections::HashMap<(i32, i32), [u8; 3]> = std::collections::HashMap::new();
    for y in 0..side {
        for x in 0..side {
            let (u, v) = ((x as f32 + 0.5) / px as f32 * CELL - radius, (y as f32 + 0.5) / px as f32 * CELL - radius);
            let [mx, mz] = to_map(u, v);
            let cell = cell_of(mx, mz);
            let colour = *cache.entry(cell).or_insert_with(|| {
                let surfaces = map.surfaces(cell);
                let near = surfaces.iter().enumerate().filter(|(_, s)| (s.h - pose[1]).abs() < 2.0).min_by(|a, b| (a.1.h - pose[1]).abs().total_cmp(&(b.1.h - pose[1]).abs()));
                match near {
                    None if map.column(cell).is_some_and(|c| c.filled_between(pose[1] + 0.3, pose[1] + 2.0)) => [200, 60, 60],
                    None => [36, 36, 44],
                    Some((k, s)) => {
                        if !planner.stood_on(cell, k, s) {
                            [200, 60, 60]
                        } else if s.own > 0 {
                            [70, 120, 235]
                        } else if s.others > 0 {
                            [60, 170, 170]
                        } else {
                            let t = ((s.h - pose[1]) / 0.6).clamp(-1.0, 1.0);
                            let g = (165.0 + 70.0 * t) as u8;
                            [g / 3, g, g / 3]
                        }
                    }
                }
            });
            rgb[(y * side + x) * 3..(y * side + x) * 3 + 3].copy_from_slice(&colour);
        }
    }
    let mut dot = |x: f32, z: f32, r: i32, colour: [u8; 3]| {
        let (cx, cy) = to_pic(x, z);
        for dy in -r..=r {
            for dx in -r..=r {
                let (px_, py_) = (cx as i32 + dx, cy as i32 + dy);
                if px_ >= 0 && py_ >= 0 && (px_ as usize) < side && (py_ as usize) < side {
                    rgb[(py_ as usize * side + px_ as usize) * 3..(py_ as usize * side + px_ as usize) * 3 + 3].copy_from_slice(&colour);
                }
            }
        }
    };
    for w in path.windows(2) {
        let n = ((w[1][0] - w[0][0]).hypot(w[1][2] - w[0][2]) / (CELL / 2.0)).ceil().max(1.0) as usize;
        for i in 0..=n {
            let t = i as f32 / n as f32;
            dot(w[0][0] + (w[1][0] - w[0][0]) * t, w[0][2] + (w[1][2] - w[0][2]) * t, 0, [255, 255, 255]);
        }
    }
    for m in &map.marks {
        if m.belief(p.now) != Belief::Gone && (m.at[1] - pose[1]).abs() < 2.0 {
            let colour = if m.belief(p.now) == Belief::Shut { [255, 0, 255] } else { [150, 60, 150] };
            dot(m.at[0], m.at[2], px as i32, colour);
        }
    }
    for o in &map.objects {
        if o.confirmed() && (o.at[1] - pose[1]).abs() < 2.0 {
            dot(o.at[0], o.at[2], px as i32 + 1, [250, 220, 40]);
        }
    }
    let (hs, hc) = heading.to_radians().sin_cos();
    for k in 0..(0.8 / (CELL / 2.0)) as i32 {
        let d = k as f32 * CELL / 2.0;
        dot(pose[0] + hs * d, pose[2] - hc * d, 0, [255, 255, 255]);
    }
    dot(pose[0], pose[2], px as i32, [255, 255, 255]);
    Picture { side, rgb }
}
