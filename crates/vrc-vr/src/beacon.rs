//! The position beacon: an avatar shader draws, in a corner of the bot's
//! own eyes only, the eye camera's world position and heading as black and
//! white blocks (docs/full-vr/avatar-position-beacon.md). Read here from a
//! tapped frame.
//!
//! The marker: 22 x 10 blocks, each 0.01 of NDC, 0.04 in from the eye's
//! bottom-left corner (top-left if the image came out flipped); row 0
//! white, black, white... (the levels), the rest of the border white, rows
//! 1-8 x columns 1-20 the 160 bits: magic 0x5A, x, y, z (f32 bits), yaw
//! (u16 of a turn), pitch (i16, centidegrees), seq (u8), CRC-16/CCITT-FALSE
//! of the 144 before it. Unity's world: +x right, +y up, +z ahead.
//!
//! The grid's reading ([`read_grid`]) is shared: the avatar's panorama rig
//! draws its own code (magic 0x5B) the same way, right above (`vrc-pano`).

use crate::tap::EyeFrame;

pub const COLS: usize = 22;
pub const ROWS: usize = 10;
pub const BLOCK_NDC: f32 = 0.01;
pub const MARGIN_NDC: f32 = 0.04;
const MAGIC: u32 = 0x5A;
/// White and black must differ by this much (0-255) to be read.
const MIN_CONTRAST: f32 = 80.0;

/// What one eye's marker says.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Beacon {
    /// The eye camera's position, Unity's world (metres).
    pub position: [f32; 3],
    /// Clockwise from +z seen from above (degrees, 0..360).
    pub yaw: f32,
    /// Up positive (degrees).
    pub pitch: f32,
    pub seq: u8,
}

impl Beacon {
    /// The position in the bot's axes (right-handed, -z ahead): z flipped.
    /// Headings need no change: Unity's yaw is the bot's (+ right of ahead).
    pub fn position_bot(&self) -> [f32; 3] {
        [self.position[0], self.position[1], -self.position[2]]
    }
}

/// The marker's pixel rectangle in an eye `w` x `h`: the middle of block
/// (`col`, `row`, row 0 at the marker's top) when drawn the usual way up
/// (`flipped`: upside down at the top-left).
pub fn block_centre(w: u32, h: u32, col: usize, row: usize, flipped: bool) -> (f32, f32) {
    block_centre_at(w, h, [-1.0 + MARGIN_NDC, -1.0 + MARGIN_NDC], col, row, flipped)
}

/// The middle of block (`col`, `row`) of a 22 x 10 grid whose bottom-left
/// corner is `origin` (NDC, y up) in an eye `w` x `h` (pixels, top-left
/// origin); `flipped`: the image upside down. The pano rig's code (magic
/// 0x5B) is such a grid too, above the beacon (avatar-panorama.md 3.5).
pub fn block_centre_at(w: u32, h: u32, origin: [f32; 2], col: usize, row: usize, flipped: bool) -> (f32, f32) {
    let x = origin[0] + (col as f32 + 0.5) * BLOCK_NDC;
    let y = origin[1] + ROWS as f32 * BLOCK_NDC - (row as f32 + 0.5) * BLOCK_NDC;
    let px = (x + 1.0) / 2.0 * w as f32;
    let py = if flipped { (1.0 + y) / 2.0 * h as f32 } else { (1.0 - y) / 2.0 * h as f32 };
    (px, py)
}

/// The marker's rectangle in pixels (x0, y0, x1, y1), both ways up: to be
/// left out of what else reads the eyes.
pub fn rects(w: u32, h: u32) -> [[u32; 4]; 2] {
    let x0 = ((MARGIN_NDC / 2.0) * w as f32).floor() as u32;
    let x1 = (((MARGIN_NDC + COLS as f32 * BLOCK_NDC) / 2.0) * w as f32).ceil() as u32 + 1;
    let ylo = ((MARGIN_NDC / 2.0) * h as f32).floor() as u32;
    let yhi = (((MARGIN_NDC + ROWS as f32 * BLOCK_NDC) / 2.0) * h as f32).ceil() as u32 + 1;
    [[x0, h.saturating_sub(yhi), x1, h - ylo], [x0, ylo, x1, yhi]]
}

/// Reads eye `eye` (0 left, 1 right) of an 8-bit RGBA/BGRA frame.
pub fn read(frame: &EyeFrame, eye: usize) -> Option<Beacon> {
    if frame.bytes_per_pixel != 4 || frame.pixels.is_empty() {
        return None;
    }
    read_pixels(frame.eye(eye), frame.width, frame.height)
}

/// Reads tightly packed 4-byte pixels (any channel order: the levels are
/// the mean of the first three), `w` x `h`, rows top first.
pub fn read_pixels(px: &[u8], w: u32, h: u32) -> Option<Beacon> {
    [false, true].into_iter().find_map(|flipped| read_way(px, w, h, flipped))
}

fn read_way(px: &[u8], w: u32, h: u32, flipped: bool) -> Option<Beacon> {
    let bits = read_grid(px, w, h, [-1.0 + MARGIN_NDC, -1.0 + MARGIN_NDC], flipped)?;
    if field(&bits, 0, 8) != MAGIC {
        return None;
    }
    let position = [f32::from_bits(field(&bits, 8, 32)), f32::from_bits(field(&bits, 40, 32)), f32::from_bits(field(&bits, 72, 32))];
    if !position.iter().all(|v| v.is_finite()) {
        return None;
    }
    Some(Beacon {
        position,
        yaw: field(&bits, 104, 16) as f32 / 65536.0 * 360.0,
        pitch: field(&bits, 120, 16) as u16 as i16 as f32 / 100.0,
        seq: field(&bits, 136, 8) as u8,
    })
}

/// The 160 bits of a 22 x 10 grid with its bottom-left corner at `origin`
/// (NDC), when its levels, border and CRC (over the first 144, in the last
/// 16) hold; the magic is the caller's to check. Tightly packed 4-byte
/// pixels, any channel order.
pub fn read_grid(px: &[u8], w: u32, h: u32, origin: [f32; 2], flipped: bool) -> Option<[u8; 160]> {
    if px.len() < (w as usize) * (h as usize) * 4 {
        return None;
    }
    let level = |col: usize, row: usize| -> f32 {
        let (cx, cy) = block_centre_at(w, h, origin, col, row, flipped);
        let (cx, cy) = (cx as i64, cy as i64);
        let mut sum = 0u32;
        let mut n = 0u32;
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (x, y) = (cx + dx, cy + dy);
                if x >= 0 && y >= 0 && (x as u32) < w && (y as u32) < h {
                    let i = (y as usize * w as usize + x as usize) * 4;
                    sum += px[i] as u32 + px[i + 1] as u32 + px[i + 2] as u32;
                    n += 3;
                }
            }
        }
        if n == 0 {
            0.0
        } else {
            sum as f32 / n as f32
        }
    };
    let (mut white, mut black) = (0.0f32, 0.0f32);
    for col in 0..COLS {
        if col % 2 == 0 {
            white += level(col, 0);
        } else {
            black += level(col, 0);
        }
    }
    let (white, black) = (white / (COLS / 2) as f32, black / (COLS / 2) as f32);
    if white - black < MIN_CONTRAST {
        return None;
    }
    let mid = (white + black) / 2.0;
    // The rest of the border is white.
    let border_off = (0..COLS).filter(|&c| level(c, ROWS - 1) < mid).count()
        + (1..ROWS - 1).filter(|&r| level(0, r) < mid || level(COLS - 1, r) < mid).count();
    if border_off > 2 {
        return None;
    }
    let mut bits = [0u8; 160];
    for (k, b) in bits.iter_mut().enumerate() {
        let (row, col) = (1 + k / (COLS - 2), 1 + k % (COLS - 2));
        *b = (level(col, row) >= mid) as u8;
    }
    (field(&bits, 144, 16) == crc16(&bits[..144])).then_some(bits)
}

/// Bits `from..from + len` (at most 32) as a number, the first the highest.
pub fn field(bits: &[u8], from: usize, len: usize) -> u32 {
    bits[from..from + len].iter().fold(0u32, |a, &b| (a << 1) | b as u32)
}

/// CRC-16/CCITT-FALSE over bits (most significant first).
pub fn crc16(bits: &[u8]) -> u32 {
    let mut crc = 0xFFFFu32;
    for &b in bits {
        let top = (crc >> 15) & 1;
        crc = (crc << 1) & 0xFFFF;
        if top ^ b as u32 != 0 {
            crc ^= 0x1021;
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 160 bits a shader would draw for `b`.
    fn encode(b: &Beacon) -> Vec<u8> {
        let mut bits = Vec::new();
        let mut put = |v: u32, n: usize| {
            for i in (0..n).rev() {
                bits.push(((v >> i) & 1) as u8);
            }
        };
        put(MAGIC, 8);
        for v in b.position {
            put(v.to_bits(), 32);
        }
        put(((b.yaw / 360.0 * 65536.0).round() as u32) & 0xFFFF, 16);
        put(((b.pitch * 100.0).round() as i32 as u32) & 0xFFFF, 16);
        put(b.seq as u32, 8);
        let crc = crc16(&bits);
        let mut out = bits;
        for i in (0..16).rev() {
            out.push(((crc >> i) & 1) as u8);
        }
        out
    }

    /// An eye `w` x `h` (grey, some light bloom) with the marker drawn.
    fn eye(b: &Beacon, w: u32, h: u32, flipped: bool) -> Vec<u8> {
        let bits = encode(b);
        let mut px = vec![90u8; (w * h * 4) as usize];
        let bw = BLOCK_NDC / 2.0 * w as f32;
        let bh = BLOCK_NDC / 2.0 * h as f32;
        for row in 0..ROWS {
            for col in 0..COLS {
                let white = if row == 0 {
                    col % 2 == 0
                } else if row == ROWS - 1 || col == 0 || col == COLS - 1 {
                    true
                } else {
                    bits[(row - 1) * (COLS - 2) + (col - 1)] == 1
                };
                let (cx, cy) = block_centre(w, h, col, row, flipped);
                let (x0, y0) = ((cx - bw / 2.0).round() as u32, (cy - bh / 2.0).round() as u32);
                for y in y0..(y0 + bh.round() as u32) {
                    for x in x0..(x0 + bw.round() as u32) {
                        let i = ((y * w + x) * 4) as usize;
                        // Post-processing: white a little dim, black a little lifted.
                        let v = if white { 230 } else { 30 };
                        px[i..i + 4].copy_from_slice(&[v, v, v, 255]);
                    }
                }
            }
        }
        px
    }

    #[test]
    fn reads_what_the_shader_draws_either_way_up() {
        let b = Beacon { position: [12.345, 1.234, -56.789], yaw: 300.5, pitch: -12.34, seq: 77 };
        for (w, flipped) in [(1920, false), (1280, false), (1920, true)] {
            let got = read_pixels(&eye(&b, w, w, flipped), w, w).expect("a beacon");
            assert_eq!(got.position, b.position);
            assert!((got.yaw - 300.5).abs() < 0.01 && (got.pitch + 12.34).abs() < 0.01 && got.seq == 77, "{got:?}");
        }
        assert_eq!(b.position_bot(), [12.345, 1.234, 56.789]);
    }

    #[test]
    fn nothing_drawn_or_a_bit_wrong_reads_nothing() {
        let w = 1920;
        assert!(read_pixels(&vec![90u8; (w * w * 4) as usize], w, w).is_none());
        let b = Beacon { position: [1.0, 2.0, 3.0], yaw: 10.0, pitch: 0.0, seq: 1 };
        let mut px = eye(&b, w, w, false);
        // One data block flipped: the CRC says no.
        let (cx, cy) = block_centre(w, w, 5, 3, false);
        for dy in -3i32..=3 {
            for dx in -3i32..=3 {
                let i = (((cy as i32 + dy) as u32 * w + (cx as i32 + dx) as u32) * 4) as usize;
                px[i] = 255 - px[i];
                px[i + 1] = 255 - px[i + 1];
                px[i + 2] = 255 - px[i + 2];
            }
        }
        assert!(read_pixels(&px, w, w).is_none());
    }

    #[test]
    fn rects_cover_the_marker() {
        let [up, flipped] = rects(1920, 1920);
        for (col, row) in [(0, 0), (COLS - 1, ROWS - 1)] {
            let (x, y) = block_centre(1920, 1920, col, row, false);
            assert!(x as u32 >= up[0] && (x as u32) < up[2] && y as u32 >= up[1] && (y as u32) < up[3]);
            let (x, y) = block_centre(1920, 1920, col, row, true);
            assert!(x as u32 >= flipped[0] && (x as u32) < flipped[2] && y as u32 >= flipped[1] && (y as u32) < flipped[3]);
        }
    }
}
