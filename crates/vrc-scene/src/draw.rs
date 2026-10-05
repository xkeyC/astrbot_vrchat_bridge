//! Numbered marks on RGB8 images, for showing candidates to a model: a
//! filled disc with a dark rim and the number in it, readable on any
//! background.

/// 3x5 digit glyphs, rows top to bottom, bit 2 = left column.
const DIGITS: [[u8; 5]; 10] = [
    [0b111, 0b101, 0b101, 0b101, 0b111],
    [0b010, 0b110, 0b010, 0b010, 0b111],
    [0b111, 0b001, 0b111, 0b100, 0b111],
    [0b111, 0b001, 0b111, 0b001, 0b111],
    [0b101, 0b101, 0b111, 0b001, 0b001],
    [0b111, 0b100, 0b111, 0b001, 0b111],
    [0b111, 0b100, 0b111, 0b101, 0b111],
    [0b111, 0b001, 0b010, 0b010, 0b010],
    [0b111, 0b101, 0b111, 0b101, 0b111],
    [0b111, 0b101, 0b111, 0b001, 0b111],
];

pub struct Canvas<'a> {
    pub rgb: &'a mut [u8],
    pub width: usize,
    pub height: usize,
}

impl Canvas<'_> {
    pub fn put(&mut self, x: i64, y: i64, c: [u8; 3]) {
        if x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height {
            let i = (y as usize * self.width + x as usize) * 3;
            self.rgb[i..i + 3].copy_from_slice(&c);
        }
    }

    pub fn disc(&mut self, cx: i64, cy: i64, r: i64, c: [u8; 3]) {
        for y in -r..=r {
            for x in -r..=r {
                if x * x + y * y <= r * r {
                    self.put(cx + x, cy + y, c);
                }
            }
        }
    }

    /// Digits of `text` (others skipped), centred on (cx, cy), `scale` px a dot.
    pub fn number(&mut self, cx: i64, cy: i64, text: &str, scale: i64, c: [u8; 3]) {
        let digits: Vec<usize> = text.chars().filter_map(|ch| ch.to_digit(10)).map(|d| d as usize).collect();
        let w = digits.len() as i64 * 4 * scale - scale;
        let (x0, y0) = (cx - w / 2, cy - 5 * scale / 2);
        for (k, &d) in digits.iter().enumerate() {
            for (row, bits) in DIGITS[d].iter().enumerate() {
                for col in 0..3 {
                    if bits & (0b100 >> col) != 0 {
                        for sy in 0..scale {
                            for sx in 0..scale {
                                self.put(
                                    x0 + k as i64 * 4 * scale + col as i64 * scale + sx,
                                    y0 + row as i64 * scale + sy,
                                    c,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    /// A numbered mark: dark rim, coloured disc, dark number.
    pub fn mark(&mut self, cx: i64, cy: i64, id: usize, radius: i64, fill: [u8; 3]) {
        self.disc(cx, cy, radius + radius / 5 + 1, [0, 0, 0]);
        self.disc(cx, cy, radius, fill);
        let text = id.to_string();
        let scale = (radius * 2 / (5 * text.len() as i64).max(6)).max(1);
        self.number(cx, cy, &text, scale, [0, 0, 0]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn draws_inside_and_clips_outside() {
        let mut rgb = vec![255u8; 20 * 20 * 3];
        let mut c = Canvas { rgb: &mut rgb, width: 20, height: 20 };
        c.mark(10, 10, 7, 8, [255, 255, 0]);
        c.mark(-50, -50, 1, 8, [255, 0, 0]); // off the canvas: nothing
        // The centre column of a 7 is ink.
        assert!(rgb.chunks(3).any(|p| p == [0, 0, 0]));
        assert!(rgb.chunks(3).any(|p| p == [255, 255, 0]));
        assert!(!rgb.chunks(3).any(|p| p == [255, 0, 0]));
    }
}
