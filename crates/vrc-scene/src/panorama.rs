//! Equirectangular panoramas (made from the avatar's pano frame,
//! `vrc_pano::PanoFrame::equirect`).
//!
//! Longitude 0 is the tracking space's -Z (ahead at yaw 0), growing to the
//! right; latitude grows up.

/// An RGB8 equirectangular image, `width` = 2 x `height`.
pub struct Panorama {
    pub width: usize,
    pub height: usize,
    pub rgb: Vec<u8>,
    /// Fraction of pixels some view saw.
    pub coverage: f32,
}

impl Panorama {
    /// Pixel (column, row) showing the direction from `eye` to `point`
    /// (tracking space).
    pub fn pixel_of(&self, eye: [f32; 3], point: [f32; 3]) -> (f32, f32) {
        let (dx, dy, dz) = (point[0] - eye[0], point[1] - eye[1], point[2] - eye[2]);
        let lon = dx.atan2(-dz);
        let lat = dy.atan2(dx.hypot(dz));
        let col = (lon + std::f32::consts::PI) / std::f32::consts::TAU * self.width as f32;
        let row = (std::f32::consts::FRAC_PI_2 - lat) / std::f32::consts::PI * self.height as f32;
        (col, row)
    }
}
