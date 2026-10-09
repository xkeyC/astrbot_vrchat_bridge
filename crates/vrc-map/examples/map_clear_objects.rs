//! Clears the things (objects) of map files, keeping everything else
//! (places, marks, what was seen and walked): `cargo run -p vrc-map
//! --example map_clear_objects -- <file>...`. Not while the bridge has the
//! map loaded (it would save its own over it).

use vrc_map::store;

fn main() {
    for arg in std::env::args().skip(1) {
        let file = std::path::Path::new(&arg);
        match store::load(file) {
            Ok((meta, mut map)) => {
                let (objects, places, marks) = (map.objects.len(), map.places.len(), map.marks.len());
                map.objects.clear();
                match store::save(file, &meta, &map) {
                    Ok(()) => println!("{arg}: {objects} objects cleared; {places} places, {marks} marks kept"),
                    Err(e) => println!("{arg}: not saved: {e}"),
                }
            }
            Err(e) => println!("{arg}: {e}"),
        }
    }
}
