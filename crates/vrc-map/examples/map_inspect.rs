//! Why a saved map's columns are stood on or not, round where the bot was:
//!   cargo run --release -p vrc-map --example map_inspect -- FILE [BODY] [OUT.ppm]
//! Colours: green stood on; red too little room over it (its own column);
//! orange something within the body's half width (a neighbour); purple no
//! surface near the feet's height but filled voxels over it; dark unknown.
//! Prints the counts and the heights (over the feet) of what blocks.
use std::collections::BTreeMap;

use vrc_map::{cell_of, store, CELL};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (meta, map) = store::load(std::path::Path::new(&args[1])).expect("a map file");
    let body: f32 = args.get(2).and_then(|v| v.parse().ok()).unwrap_or(1.25);
    let out = args.get(3).cloned().unwrap_or_else(|| "inspect.ppm".into());
    let pose = meta.pose;
    let radius = 7.0f32;
    let n = (2.0 * radius / CELL) as i32;
    let c0 = cell_of(pose[0] - radius, pose[2] - radius);
    let mut rgb = vec![0u8; (n * n * 3) as usize];
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut blocking: BTreeMap<i32, usize> = BTreeMap::new();
    let mut ratio: BTreeMap<i32, usize> = BTreeMap::new();
    let mut pts: BTreeMap<i32, usize> = BTreeMap::new();
    for r in 0..n {
        for c in 0..n {
            let cell = (c0.0 + c, c0.1 + r);
            let surfaces = map.surfaces(cell);
            let near = surfaces.iter().filter(|s| (s.h - pose[1]).abs() < 0.5).min_by(|a, b| (a.h - pose[1]).abs().total_cmp(&(b.h - pose[1]).abs()));
            let (why, colour) = match near {
                None if map.column(cell).is_some_and(|col| col.filled_between(pose[1] + 0.3, pose[1] + 2.0)) => ("over, no floor", [160, 60, 200]),
                None => ("unknown", [36, 36, 44]),
                Some(s) if s.room < body => {
                    *blocking.entry(((s.h + s.room - pose[1]) * 10.0).round() as i32).or_default() += 1;
                    // The run over it: its points and weight, against the surface's voxel.
                    if let Some(col) = map.column(cell) {
                        let top = col.voxels.iter().filter(|v| v.filled() && (v.y() - s.h).abs() < 0.03).map(|v| (v.n, v.hit)).next().unwrap_or((0, 0.0));
                        let over: Vec<_> = col.voxels.iter().filter(|v| v.filled() && v.y() > s.h + 0.1 && v.y() < s.h + body).map(|v| (v.n, v.hit, v.miss)).collect();
                        let (n, w): (u32, f32) = over.iter().fold((0, 0.0), |a, v| (a.0 + v.0 as u32, a.1 + v.1));
                        *ratio.entry(((w / top.1.max(0.01)) * 10.0).round().min(50.0) as i32).or_default() += 1;
                        *pts.entry((n as i32).min(60) / 5 * 5).or_default() += 1;
                    }
                    ("room", [210, 50, 50])
                }
                Some(s) => {
                    let k = (0.25 / CELL).ceil() as i32;
                    let r2 = (0.25 / CELL) * (0.25 / CELL);
                    let mut hit = false;
                    for dz in -k..=k {
                        for dx in -k..=k {
                            if ((dx * dx + dz * dz) as f32) <= r2 {
                                if let Some(col) = map.column((cell.0 + dx, cell.1 + dz)) {
                                    if col.filled_between(s.h + 0.5, s.h + body) {
                                        hit = true;
                                    }
                                }
                            }
                        }
                    }
                    if hit { ("beside", [240, 150, 30]) } else { ("stood on", [60, 180, 60]) }
                }
            };
            *counts.entry(why).or_default() += 1;
            let i = ((r * n + c) * 3) as usize;
            rgb[i..i + 3].copy_from_slice(&colour);
        }
    }
    let mid = ((n / 2) * n + n / 2) as usize * 3;
    rgb[mid..mid + 3].copy_from_slice(&[255, 255, 255]);
    let mut f = format!("P6 {n} {n} 255\n").into_bytes();
    f.extend_from_slice(&rgb);
    std::fs::write(&out, f).unwrap();
    println!("pose {pose:?} body {body}");
    println!("{counts:?}");
    println!("room's top over the feet (dm: columns): {blocking:?}");
    println!("weight over / the surface's voxel (x10: columns): {ratio:?}");
    println!("points over (by 5: columns): {pts:?}");
}
