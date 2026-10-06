//! A way on a saved map from where the bot stood to a point:
//!   cargo run --release -p vrc-map --example map_route -- FILE X Z [START_RADIUS] [OUT.ppm]
//! Prints the way (and where it runs into a shut mark); draws it.
use vrc_map::plan::{PlanParams, Planner};
use vrc_map::{store, unix_now};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (meta, map) = store::load(std::path::Path::new(&a[1])).expect("a map file");
    let goal = [a[2].parse::<f32>().unwrap(), a[3].parse::<f32>().unwrap()];
    let start_radius: f32 = a.get(4).and_then(|v| v.parse().ok()).unwrap_or(1.0);
    let now = unix_now();
    let p = PlanParams { body: 1.25, start_radius, now, ..Default::default() };
    let pose = meta.pose;
    println!("feet {pose:?} -> {goal:?}, straight shut at {:?}", map.shut_along(pose, [goal[0], pose[1], goal[1]], now));
    let mut pl = Planner::new(&map, p, pose);
    let t = std::time::Instant::now();
    let planned = pl.plan(goal, None);
    println!("planned in {:?}", t.elapsed());
    match planned {
        None => println!("no way"),
        Some(path) => {
            println!("way {:.2} m, reached {}, {} points", path.length, path.reached, path.points.len());
            let through: Vec<usize> = (1..path.points.len()).filter(|&i| map.crosses_shut(path.points[i - 1], path.points[i], now)).collect();
            println!("steps through a shut mark: {through:?}");
            println!("first leg: {:?}", pl.leg(&path, 2.0));
            let pts: Vec<String> = path.points.iter().step_by(4).map(|q| format!("({:.1},{:.1})", q[0], q[2])).collect();
            println!("{}", pts.join(" "));
            if let Some(out) = a.get(5) {
                let pic = vrc_map::render::render(&map, pose, 0.0, 4.0, 6, None, &path.points, &p);
                let mut f = format!("P6 {0} {0} 255\n", pic.side).into_bytes();
                f.extend_from_slice(&pic.rgb);
                std::fs::write(out, f).unwrap();
            }
        }
    }
}
