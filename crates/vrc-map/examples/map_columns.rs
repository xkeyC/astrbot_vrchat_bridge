//! A saved map's columns along a heading from where the bot stood:
//!   cargo run --release -p vrc-map --example map_columns -- FILE HEADING [STEP_M] [COUNT]
use vrc_map::{cell_of, store};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (meta, map) = store::load(std::path::Path::new(&a[1])).expect("a map file");
    let heading: f32 = a[2].parse().unwrap();
    let step: f32 = a.get(3).and_then(|v| v.parse().ok()).unwrap_or(0.3);
    let count: usize = a.get(4).and_then(|v| v.parse().ok()).unwrap_or(15);
    let p = meta.pose;
    let (s, c) = heading.to_radians().sin_cos();
    println!("feet {:?}", p);
    for k in 1..=count {
        let d = k as f32 * step;
        let (x, z) = (p[0] + s * d, p[2] - c * d);
        let col = map.column(cell_of(x, z));
        let vox: Vec<String> = col
            .map(|col| {
                col.voxels
                    .iter()
                    .map(|v| format!("{:+.2}:{}/{:.1}/{:.1}{}", v.y() - p[1], v.n, v.hit, v.miss, if v.filled() { "*" } else { "" }))
                    .collect()
            })
            .unwrap_or_default();
        let surf: Vec<String> = map.surfaces(cell_of(x, z)).iter().map(|s| format!("{:+.2}(room {:.2})", s.h - p[1], s.room)).collect();
        println!("{d:.1} m: surfaces {surf:?}\n      voxels {}", vox.join(" "));
    }
}
