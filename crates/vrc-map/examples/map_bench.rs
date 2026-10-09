//! How long a survey's looks take to go onto the map (synthetic room).
use std::time::Instant;

use vrc_map::{Nav, Observation, WorldMap};

fn main() {
    // A room 12 x 12 with walls, as ~70k points a look (depth, every 2nd
    // pixel), six looks round.
    let mut pts = Vec::new();
    let mut x = -6.0f32;
    while x < 6.0 {
        let mut z = -6.0f32;
        while z < 6.0 {
            pts.push([x, 0.0, z]);
            z += 0.06;
        }
        x += 0.06;
    }
    for k in 0..4 {
        let mut a = -6.0f32;
        while a < 6.0 {
            let mut y = 0.0;
            while y < 2.5 {
                let p = match k { 0 => [a, y, -6.0], 1 => [a, y, 6.0], 2 => [-6.0, y, a], _ => [6.0, y, a] };
                pts.push(p);
                y += 0.04;
            }
            a += 0.04;
        }
    }
    println!("{} points", pts.len());
    let mut nav = Nav::new("bench", WorldMap::default());
    nav.begin("bench", "s", None, 0.0);
    let t = Instant::now();
    for _ in 0..6 {
        let obs = Observation { points: pts.clone(), eye: [0.0, 1.2, 0.0], people: vec![], rays_every: 8, at: Instant::now() };
        let t1 = Instant::now();
        let fit = nav.observe(&obs);
        println!("look: {:?} fit {} {:.2} {}", t1.elapsed(), fit.across, fit.overlap, fit.why);
    }
    println!("six looks: {:?}; {:?}", t.elapsed(), nav.map.census());
    let t = Instant::now();
    let mut pl = vrc_map::plan::Planner::new(&nav.map, Default::default(), nav.pose);
    let path = pl.plan([4.0, -4.0], None);
    println!("plan: {:?} {:?}", t.elapsed(), path.map(|p| (p.reached, p.length, p.expanded)));
    let t = Instant::now();
    let pic = vrc_map::render::render(&nav.map, nav.pose, 0.0, 8.0, 3, None, &[], &Default::default());
    println!("render: {:?} {}", t.elapsed(), pic.side);
}
