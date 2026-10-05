//! Pictures of a survey for the model: the panorama and the top-down map,
//! with the candidates' numbers on both.

use vrc_scene::draw::Canvas;
use vrc_scene::{Kind, Panorama};
use vrc_vr::tap::EyeFrame;

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
    /// The equirectangular panorama of the scan, `width` x `width / 2`.
    pub fn panorama(&self, width: usize) -> Panorama {
        let frames: Vec<&EyeFrame> = self.shots.iter().map(|s| &s.frame).collect();
        Panorama::stitch(&frames, width)
    }

    /// The panorama with the candidates' numbers.
    pub fn marked_panorama(&self, width: usize) -> Panorama {
        let mut pano = self.panorama(width);
        let r = (width / 90).max(8) as i64;
        let mut canvas = Canvas { rgb: &mut pano.rgb, width: pano.width, height: pano.height };
        for c in &self.candidates {
            let (x, y) = {
                let (dx, dy, dz) = (c.position[0] - self.eye[0], c.position[1] - self.eye[1], c.position[2] - self.eye[2]);
                let lon = dx.atan2(-dz);
                let lat = dy.atan2(dx.hypot(dz));
                (
                    (lon + std::f32::consts::PI) / std::f32::consts::TAU * width as f32,
                    (std::f32::consts::FRAC_PI_2 - lat) / std::f32::consts::PI * (width / 2) as f32,
                )
            };
            canvas.mark(x as i64, y as i64, c.id, r, colour(c.kind));
        }
        pano
    }

    /// The height map, `up` times enlarged, with the candidates' numbers:
    /// (side, RGB8).
    pub fn marked_map(&self, up: usize) -> (usize, Vec<u8>) {
        let map = &self.map;
        let small = map.render(self.eye, self.yaw);
        let size = map.size * up;
        let mut big = vec![0u8; size * size * 3];
        for (i, px) in big.chunks_mut(3).enumerate() {
            let (r, c) = (i / size / up, i % size / up);
            px.copy_from_slice(&small[(r * map.size + c) * 3..(r * map.size + c) * 3 + 3]);
        }
        let mut canvas = Canvas { rgb: &mut big, width: size, height: size };
        let half = map.size as f32 / 2.0;
        for c in &self.candidates {
            let col = ((c.position[0] - map.origin[0]) / map.params.cell + half) * up as f32;
            let row = ((c.position[2] - map.origin[1]) / map.params.cell + half) * up as f32;
            canvas.mark(col as i64, row as i64, c.id, (3 * up) as i64 + 2, colour(c.kind));
        }
        (size, big)
    }
}
