//! What map files hold: `cargo run -p vrc-map --example map_info -- <file>...`
//! (the world, the session, places, objects, marks, how much was seen).

use vrc_map::store;

fn main() {
    for arg in std::env::args().skip(1) {
        match store::load(std::path::Path::new(&arg)) {
            Ok((meta, map)) => {
                println!("{arg}");
                println!(
                    "  world {} world_frame {} session {} saved {} pose {:?} spawns {}",
                    meta.world,
                    meta.world_frame,
                    meta.session,
                    meta.saved,
                    meta.pose.map(|v| (v * 100.0).round() / 100.0),
                    meta.spawns.len()
                );
                println!("  places {}", map.places.len());
                for p in &map.places {
                    println!("    {} at {:?}", p.name, p.at.map(|v| (v * 100.0).round() / 100.0));
                }
                let mut labels: Vec<&str> = map.objects.iter().map(|o| o.label.as_str()).collect();
                labels.sort_unstable();
                println!("  objects {} {:?}", map.objects.len(), labels);
                println!("  marks {}", map.marks.len());
                for m in &map.marks {
                    println!("    {:?} at {:?} yaw {:.0} count {} last {}", m.kind, m.at.map(|v| (v * 100.0).round() / 100.0), m.yaw, m.count, m.last);
                }
            }
            Err(e) => println!("{arg}: {e}"),
        }
    }
}
