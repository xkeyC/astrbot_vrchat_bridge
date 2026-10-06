//! A world's map on disk: one file per world (`<dir>/<wrld_...>.vrcmap`),
//! little-endian, written to a temporary file and renamed over the old.
//!
//! Besides the map: where the bot last stood and how the session's axes
//! lay on the map's, with the session they belong to (the game's instance
//! and start), so a bridge started again in the same session carries on.

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use crate::{Chunk, Column, Mark, MarkKind, Object, Place, Voxel, Walk, WorldMap};

const MAGIC: &[u8; 8] = b"VRCMAP04";
/// Files of the versions before load too: 02 (no objects' spread and
/// kinds), 03 (no places' headings).
const MAGIC_02: &[u8; 8] = b"VRCMAP02";
const MAGIC_03: &[u8; 8] = b"VRCMAP03";

/// What a map file says besides the map.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Meta {
    pub world: String,
    /// The session it was saved in, and its pose then (map frame) and
    /// the session's axes' turn.
    pub session: String,
    pub pose: [f32; 3],
    pub yaw: f32,
    pub saved: u64,
    /// Where sessions began (spawn points): feet and heading (map frame).
    pub spawns: Vec<[f32; 4]>,
    /// The map is in the world's own frame (the position beacon's).
    pub world_frame: bool,
}

/// The file of `world`'s map in `dir`.
pub fn path(dir: &Path, world: &str) -> PathBuf {
    let safe: String = world.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
    dir.join(format!("{safe}.vrcmap"))
}

pub fn save(file: &Path, meta: &Meta, map: &WorldMap) -> std::io::Result<()> {
    if let Some(dir) = file.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = file.with_extension("vrcmap.tmp");
    {
        let mut w = BufWriter::new(File::create(&tmp)?);
        w.write_all(MAGIC)?;
        put_str(&mut w, &meta.world)?;
        put_str(&mut w, &meta.session)?;
        for v in meta.pose {
            put_f32(&mut w, v)?;
        }
        put_f32(&mut w, meta.yaw)?;
        put_u64(&mut w, meta.saved)?;
        put_u32(&mut w, meta.spawns.len() as u32)?;
        for s in &meta.spawns {
            for v in s {
                put_f32(&mut w, *v)?;
            }
        }
        w.write_all(&[meta.world_frame as u8])?;
        put_u32(&mut w, map.chunks.len() as u32)?;
        let mut keys: Vec<_> = map.chunks.keys().copied().collect();
        keys.sort();
        for key in keys {
            put_i32(&mut w, key.0)?;
            put_i32(&mut w, key.1)?;
            for col in &map.chunks[&key].cols {
                put_u16(&mut w, col.voxels.len() as u16)?;
                for v in &col.voxels {
                    put_i16(&mut w, v.iy)?;
                    put_u16(&mut w, v.n)?;
                    put_f32(&mut w, v.hit)?;
                    put_f32(&mut w, v.sum_x)?;
                    put_f32(&mut w, v.sum_y)?;
                    put_f32(&mut w, v.sum_z)?;
                    put_f32(&mut w, v.miss)?;
                }
                put_u16(&mut w, col.walks.len() as u16)?;
                for k in &col.walks {
                    put_i16(&mut w, k.iy)?;
                    put_u16(&mut w, k.own)?;
                    put_u16(&mut w, k.others)?;
                    put_u64(&mut w, k.last)?;
                }
            }
        }
        put_u32(&mut w, map.marks.len() as u32)?;
        for m in &map.marks {
            for v in m.at {
                put_f32(&mut w, v)?;
            }
            put_f32(&mut w, m.yaw)?;
            w.write_all(&[match m.kind {
                MarkKind::Blocked => 0,
                MarkKind::JumpFailed => 1,
            }])?;
            put_u16(&mut w, m.count)?;
            put_u64(&mut w, m.last)?;
        }
        put_u32(&mut w, map.objects.len() as u32)?;
        for o in &map.objects {
            put_str(&mut w, &o.label)?;
            for v in o.at {
                put_f32(&mut w, v)?;
            }
            for v in o.size {
                put_f32(&mut w, v)?;
            }
            put_u16(&mut w, o.seen)?;
            put_f32(&mut w, o.score)?;
            put_u64(&mut w, o.last)?;
            put_f32(&mut w, o.m2)?;
            put_u16(&mut w, o.kinds.len() as u16)?;
            for (k, n) in &o.kinds {
                put_str(&mut w, k)?;
                put_u16(&mut w, *n)?;
            }
        }
        put_u32(&mut w, map.places.len() as u32)?;
        for p in &map.places {
            put_str(&mut w, &p.name)?;
            for v in p.at {
                put_f32(&mut w, v)?;
            }
            put_f32(&mut w, p.heading.unwrap_or(f32::NAN))?;
        }
        w.flush()?;
    }
    std::fs::rename(&tmp, file)
}

pub fn load(file: &Path) -> std::io::Result<(Meta, WorldMap)> {
    let mut r = BufReader::new(File::open(file)?);
    let mut magic = [0u8; 8];
    r.read_exact(&mut magic)?;
    let v2 = &magic == MAGIC_02;
    let v3 = &magic == MAGIC_03;
    if &magic != MAGIC && !v2 && !v3 {
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "not a map file"));
    }
    let mut meta = Meta { world: get_str(&mut r)?, session: get_str(&mut r)?, ..Default::default() };
    for k in 0..3 {
        meta.pose[k] = get_f32(&mut r)?;
    }
    meta.yaw = get_f32(&mut r)?;
    meta.saved = get_u64(&mut r)?;
    for _ in 0..get_u32(&mut r)? {
        meta.spawns.push([get_f32(&mut r)?, get_f32(&mut r)?, get_f32(&mut r)?, get_f32(&mut r)?]);
    }
    let mut flag = [0u8];
    r.read_exact(&mut flag)?;
    meta.world_frame = flag[0] != 0;
    let mut map = WorldMap::default();
    for _ in 0..get_u32(&mut r)? {
        let key = (get_i32(&mut r)?, get_i32(&mut r)?);
        let mut cols = Vec::with_capacity(256);
        for _ in 0..256 {
            let mut col = Column::default();
            for _ in 0..get_u16(&mut r)? {
                col.voxels.push(Voxel { iy: get_i16(&mut r)?, n: get_u16(&mut r)?, hit: get_f32(&mut r)?, sum_x: get_f32(&mut r)?, sum_y: get_f32(&mut r)?, sum_z: get_f32(&mut r)?, miss: get_f32(&mut r)? });
            }
            for _ in 0..get_u16(&mut r)? {
                col.walks.push(Walk { iy: get_i16(&mut r)?, own: get_u16(&mut r)?, others: get_u16(&mut r)?, last: get_u64(&mut r)? });
            }
            cols.push(col);
        }
        map.chunks.insert(key, Chunk { cols });
    }
    for _ in 0..get_u32(&mut r)? {
        let at = [get_f32(&mut r)?, get_f32(&mut r)?, get_f32(&mut r)?];
        let yaw = get_f32(&mut r)?;
        let mut kind = [0u8];
        r.read_exact(&mut kind)?;
        let kind = if kind[0] == 1 { MarkKind::JumpFailed } else { MarkKind::Blocked };
        map.marks.push(Mark { at, yaw, kind, count: get_u16(&mut r)?, last: get_u64(&mut r)? });
    }
    for _ in 0..get_u32(&mut r)? {
        let label = get_str(&mut r)?;
        let at = [get_f32(&mut r)?, get_f32(&mut r)?, get_f32(&mut r)?];
        let size = [get_f32(&mut r)?, get_f32(&mut r)?];
        let (seen, score, last) = (get_u16(&mut r)?, get_f32(&mut r)?, get_u64(&mut r)?);
        let m2 = if v2 { 0.0 } else { get_f32(&mut r)? };
        let mut kinds = Vec::new();
        if !v2 {
            for _ in 0..get_u16(&mut r)? {
                kinds.push((get_str(&mut r)?, get_u16(&mut r)?));
            }
        }
        if kinds.is_empty() {
            kinds.push((label.clone(), seen));
        }
        map.objects.push(Object { label, at, size, seen, score, last, m2, kinds });
    }
    for _ in 0..get_u32(&mut r)? {
        let name = get_str(&mut r)?;
        let at = [get_f32(&mut r)?, get_f32(&mut r)?, get_f32(&mut r)?];
        let heading = if v2 || v3 { None } else { Some(get_f32(&mut r)?).filter(|h| h.is_finite()) };
        map.places.push(Place { name, at, heading });
    }
    Ok((meta, map))
}

fn put_u16(w: &mut impl Write, v: u16) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn put_i16(w: &mut impl Write, v: i16) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn put_u32(w: &mut impl Write, v: u32) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn put_i32(w: &mut impl Write, v: i32) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn put_u64(w: &mut impl Write, v: u64) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn put_f32(w: &mut impl Write, v: f32) -> std::io::Result<()> {
    w.write_all(&v.to_le_bytes())
}
fn put_str(w: &mut impl Write, s: &str) -> std::io::Result<()> {
    put_u16(w, s.len().min(u16::MAX as usize) as u16)?;
    w.write_all(&s.as_bytes()[..s.len().min(u16::MAX as usize)])
}

fn get<const N: usize>(r: &mut impl Read) -> std::io::Result<[u8; N]> {
    let mut b = [0u8; N];
    r.read_exact(&mut b)?;
    Ok(b)
}
fn get_u16(r: &mut impl Read) -> std::io::Result<u16> {
    Ok(u16::from_le_bytes(get(r)?))
}
fn get_i16(r: &mut impl Read) -> std::io::Result<i16> {
    Ok(i16::from_le_bytes(get(r)?))
}
fn get_u32(r: &mut impl Read) -> std::io::Result<u32> {
    Ok(u32::from_le_bytes(get(r)?))
}
fn get_i32(r: &mut impl Read) -> std::io::Result<i32> {
    Ok(i32::from_le_bytes(get(r)?))
}
fn get_u64(r: &mut impl Read) -> std::io::Result<u64> {
    Ok(u64::from_le_bytes(get(r)?))
}
fn get_f32(r: &mut impl Read) -> std::io::Result<f32> {
    Ok(f32::from_le_bytes(get(r)?))
}
fn get_str(r: &mut impl Read) -> std::io::Result<String> {
    let n = get_u16(r)? as usize;
    let mut b = vec![0u8; n];
    r.read_exact(&mut b)?;
    String::from_utf8(b).map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::{floor, look};

    #[test]
    fn a_map_comes_back_as_it_went() {
        let mut pts = Vec::new();
        floor(&mut pts, (1.0, 3.0), (-1.0, 1.0), 0.0);
        let mut m = WorldMap::default();
        m.integrate(&look(&pts, [0.0; 3], 1.5), [0.0; 3], 0.0);
        m.walked([0.0, 0.0, 0.0], true, 7);
        m.mark([0.0, 0.0, -0.4], 10.0, MarkKind::Blocked, 9);
        m.saw_object("sofa", [2.0, 0.0, 0.0], [1.5, 0.8], 0.7, 9);
        m.places.push(Place { name: "沙发".into(), at: [2.0, 0.0, 0.5], heading: Some(-30.0) });
        m.places.push(Place { name: "门口".into(), at: [0.0, 0.0, 3.0], heading: None });
        m.dirty = false;
        let dir = std::env::temp_dir().join(format!("vrc-map-test-{}", std::process::id()));
        let file = path(&dir, "wrld_a:b/c");
        assert!(file.file_name().unwrap().to_str().unwrap() == "wrld_a_b_c.vrcmap");
        let meta = Meta { world: "wrld_a".into(), session: "s1".into(), pose: [1.0, 2.0, 3.0], yaw: 30.0, saved: 5, spawns: vec![[0.0, 0.0, 0.0, 12.0]], world_frame: true };
        save(&file, &meta, &m).unwrap();
        let (meta2, m2) = load(&file).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(meta, meta2);
        assert_eq!(m, m2);
    }
}
