//! Pictures of a survey for the model: the panorama and the top-down map,
//! both turned so that the bot's heading is ahead (the panorama's middle,
//! the map's up), with the candidates' numbers on both. Bearings in the
//! candidates are from the same heading (+ right).

use vrc_scene::draw::Canvas;
use vrc_scene::{Kind, Panorama};

use crate::Survey;

pub fn colour(k: Kind) -> [u8; 3] {
    match k {
        Kind::Frontier => [90, 220, 255],
        Kind::Open => [255, 235, 60],
        Kind::Platform => [255, 150, 40],
        Kind::Player => [255, 110, 220],
    }
}

impl Survey {
    /// The equirectangular panorama of the pano frame, `width` x `width /
    /// 2`, its middle the tracking space's -Z.
    pub fn panorama(&self, width: usize) -> Panorama {
        let look = &self.pano;
        look.frame.equirect(width, look.tracking.world_yaw(0.0)).pano
    }

    /// The panorama turned to the heading (ahead in the middle, behind at
    /// both edges), with the candidates' numbers.
    pub fn marked_panorama(&self, width: usize) -> Panorama {
        let mut pano = self.panorama(width);
        let r = (width / 90).max(8) as i64;
        let at: Vec<(f32, f32)> = self.candidates.iter().map(|c| pano.pixel_of(self.eye, c.position)).collect();
        {
            let mut canvas = Canvas { rgb: &mut pano.rgb, width: pano.width, height: pano.height };
            for (c, (x, y)) in self.candidates.iter().zip(at) {
                canvas.mark(x as i64, y as i64, c.id, r, colour(c.kind));
            }
        }
        // Roll the columns so the heading is in the middle.
        let shift = ((self.yaw / 360.0) * width as f32).round() as i64;
        let mut rolled = vec![0u8; pano.rgb.len()];
        for row in 0..pano.height {
            for col in 0..width {
                let src = (col as i64 + shift).rem_euclid(width as i64) as usize;
                let (d, s) = ((row * width + col) * 3, (row * width + src) * 3);
                rolled[d..d + 3].copy_from_slice(&pano.rgb[s..s + 3]);
            }
        }
        pano.rgb = rolled;
        pano
    }

    /// The height map, `up` times enlarged and turned so the heading points
    /// up (the bot in the middle), with the candidates' numbers: (side, RGB8).
    pub fn marked_map(&self, up: usize) -> (usize, Vec<u8>) {
        let map = &self.map;
        let small = map.render(self.eye, self.yaw);
        let size = map.size * up;
        let half = map.size as f32 / 2.0;
        // Output pixel -> map cell: rotate by the heading about the middle.
        let (s, c) = self.yaw.to_radians().sin_cos();
        let mut big = vec![0u8; size * size * 3];
        for (i, px) in big.chunks_mut(3).enumerate() {
            // Output coordinates in cells from the middle: x right, y down.
            let x = (i % size) as f32 / up as f32 - half;
            let y = (i / size) as f32 / up as f32 - half;
            // Heading-up frame to the map's: right = (c, s), ahead (up) = (s, -c).
            let mx = x * c - y * s;
            let mz = x * s + y * c;
            let (col, row) = ((mx + half).floor(), (mz + half).floor());
            if col >= 0.0 && row >= 0.0 && (col as usize) < map.size && (row as usize) < map.size {
                let k = (row as usize * map.size + col as usize) * 3;
                px.copy_from_slice(&small[k..k + 3]);
            } else {
                px.copy_from_slice(&[40, 40, 48]);
            }
        }
        let mut canvas = Canvas { rgb: &mut big, width: size, height: size };
        for cand in &self.candidates {
            // Map offsets in cells, then into the heading-up frame.
            let dx = (cand.position[0] - map.origin[0]) / map.params.cell;
            let dz = (cand.position[2] - map.origin[1]) / map.params.cell;
            let x = dx * c + dz * s;
            let y = -dx * s + dz * c;
            canvas.mark(((x + half) * up as f32) as i64, ((y + half) * up as f32) as i64, cand.id, (3 * up) as i64 + 2, colour(cand.kind));
        }
        (size, big)
    }
}
