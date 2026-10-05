//! Matching OCR text to display names. Same rule as the desktop bridge
//! (its Python `follow.py`, in the git history): case and spaces ignored; a name inside a
//! longer line is a full match; otherwise a similarity ratio, and 0.6 or
//! more counts.

/// Score from which a line is taken to be a name.
pub const MATCH_RATIO: f32 = 0.6;

pub fn normalized(text: &str) -> Vec<char> {
    text.chars().filter(|c| !c.is_whitespace()).flat_map(char::to_lowercase).collect()
}

/// How well an OCR `line` matches the display name `name` (0..1).
pub fn match_score(line: &str, name: &str) -> f32 {
    let (a, b) = (normalized(line), normalized(name));
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    if a.windows(b.len()).any(|w| w == b.as_slice()) {
        return 1.0;
    }
    2.0 * lcs(&a, &b) as f32 / (a.len() + b.len()) as f32
}

/// Length of the longest common subsequence.
fn lcs(a: &[char], b: &[char]) -> usize {
    let mut row = vec![0usize; b.len() + 1];
    for &ca in a {
        let mut diag = 0;
        for (j, &cb) in b.iter().enumerate() {
            let up = row[j + 1];
            row[j + 1] = if ca == cb { diag + 1 } else { row[j + 1].max(row[j]) };
            diag = up;
        }
    }
    row[b.len()]
}

/// The best matching name of `names` for `line`, if it matches well enough:
/// (index, score).
pub fn best_match(line: &str, names: &[String]) -> Option<(usize, f32)> {
    names
        .iter()
        .enumerate()
        .map(|(i, n)| (i, match_score(line, n)))
        .filter(|&(_, s)| s >= MATCH_RATIO)
        .max_by(|a, b| a.1.total_cmp(&b.1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scores() {
        assert_eq!(match_score("xkeyC", "xkeyc"), 1.0);
        assert_eq!(match_score("[VIP] x key C", "xkeyC"), 1.0);
        assert!(match_score("xkeyG", "xkeyC") >= MATCH_RATIO); // one misread letter
        assert!(match_score("YOZORA PROGRAM", "xkeyC") < MATCH_RATIO);
        let names = vec!["Alice".to_string(), "xkeyC".to_string()];
        assert_eq!(best_match("xkeyc", &names).map(|m| m.0), Some(1));
        assert_eq!(best_match("bob", &names), None);
    }
}
