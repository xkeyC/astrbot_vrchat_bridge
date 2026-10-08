//! Speech segments from the detector's frames, in the client's sample clock
//! (samples actually handed to the stream client: a frame that was dropped
//! takes no time on it).
//!
//! A segment starts where the frames that started the detector began (it
//! needs a few voiced frames in a row) and ends a short tail after the last
//! voiced frame (not where the hangover ran out). Segments with too little
//! voice are dropped; very long ones are cut.

use std::collections::VecDeque;

use crate::activity::Level;
use crate::HOP;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SegEvent {
    Opened {
        id: u64,
        start: u64,
    },
    Closed {
        id: u64,
        start: u64,
        end: u64,
    },
    /// Opened, but too little voice in it: forget it.
    Dropped {
        id: u64,
    },
}

#[derive(Clone, Copy, Debug)]
struct Open {
    id: u64,
    start: u64,
    /// The end of the last voiced frame, and of the last frame.
    voiced_end: u64,
    end: u64,
    /// Voice in it (samples heard, whether or not they reached the client).
    voiced: u64,
}

pub struct Segmenter {
    /// Frames the detector needs to start: the start goes back as far.
    back: usize,
    /// Kept after the last voiced frame (samples).
    pub tail: u64,
    /// Less voice than this (samples) and a segment is dropped.
    pub min_voiced: u64,
    /// Longer than this and a segment is cut (and goes on as a new one).
    pub max_len: u64,
    starts: VecDeque<u64>,
    open: Option<Open>,
    next_id: u64,
}

impl Segmenter {
    /// `back`: the detector's `start_frames`.
    pub fn new(back: u32) -> Segmenter {
        Segmenter { back: back.max(1) as usize, tail: 2_400, min_voiced: 7_200, max_len: 20 * 48_000, starts: VecDeque::new(), open: None, next_id: 1 }
    }

    /// The open segment: (id, start, end so far).
    pub fn current(&self) -> Option<(u64, u64, u64)> {
        self.open.map(|o| (o.id, o.start, o.end))
    }

    /// The clock started again (a new client): an open segment now starts
    /// at its beginning.
    pub fn restart_clock(&mut self, at: u64) {
        self.starts.clear();
        if let Some(o) = self.open.as_mut() {
            (o.start, o.voiced_end, o.end) = (at, at, at);
        }
    }

    /// One frame: its level and where it lies on the clock (`start..end`).
    pub fn push(&mut self, level: &Level, start: u64, end: u64) -> Vec<SegEvent> {
        let mut out = Vec::new();
        self.starts.push_back(start);
        while self.starts.len() > self.back {
            self.starts.pop_front();
        }
        match self.open.as_mut() {
            None if level.active => {
                let begin = self.starts.front().copied().unwrap_or(start);
                let id = self.next_id;
                self.next_id += 1;
                self.open = Some(Open { id, start: begin, voiced_end: end, end, voiced: (self.starts.len() * HOP) as u64 });
                out.push(SegEvent::Opened { id, start: begin });
            }
            None => {}
            Some(o) if level.active => {
                o.end = end;
                if level.voiced {
                    o.voiced += HOP as u64;
                    o.voiced_end = end;
                }
                if o.end - o.start >= self.max_len {
                    // Cut here, and go on as a new segment.
                    let o = *o;
                    out.push(SegEvent::Closed { id: o.id, start: o.start, end: o.end });
                    let id = self.next_id;
                    self.next_id += 1;
                    self.open = Some(Open { id, start: o.end, voiced_end: o.end, end: o.end, voiced: 0 });
                    out.push(SegEvent::Opened { id, start: o.end });
                }
            }
            Some(o) => {
                let o = *o;
                self.open = None;
                if o.voiced < self.min_voiced {
                    out.push(SegEvent::Dropped { id: o.id });
                } else {
                    out.push(SegEvent::Closed { id: o.id, start: o.start, end: (o.voiced_end + self.tail).min(end) });
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(voiced: bool, active: bool) -> Level {
        Level { voiced, active, ..Default::default() }
    }

    /// Runs `voiced` through a detector-like on/off (3 to start, 5 to
    /// end), the clock advancing a hop a frame except over `dropped`.
    fn run(voiced: &[bool], dropped: std::ops::Range<usize>) -> Vec<SegEvent> {
        let mut s = Segmenter::new(3);
        s.min_voiced = 4 * HOP as u64;
        let (mut run, mut quiet, mut on, mut clock) = (0, 0, false, 0u64);
        let mut out = Vec::new();
        for (i, &v) in voiced.iter().enumerate() {
            if v {
                run += 1;
                quiet = 0;
                on |= run >= 3;
            } else {
                run = 0;
                quiet += 1;
                on &= quiet <= 5;
            }
            let start = clock;
            if !dropped.contains(&i) {
                clock += HOP as u64;
            }
            out.extend(s.push(&frame(v, on), start, clock));
        }
        out
    }

    #[test]
    fn segments_start_where_the_voice_did_and_end_a_tail_after_it() {
        let mut v = vec![false; 10];
        v.extend([true; 20]);
        v.extend([false; 20]);
        let ev = run(&v, 0..0);
        let h = HOP as u64;
        assert_eq!(ev, vec![SegEvent::Opened { id: 1, start: 10 * h }, SegEvent::Closed { id: 1, start: 10 * h, end: 30 * h + 2_400 }]);
    }

    #[test]
    fn dropped_frames_take_no_time_and_blips_are_dropped() {
        let mut v = vec![false; 10];
        v.extend([true; 20]);
        v.extend([false; 10]);
        v.extend([true; 3]);
        v.extend([false; 10]);
        // Frames 15..20 never reached the client.
        let ev = run(&v, 15..20);
        let h = HOP as u64;
        assert_eq!(ev[1], SegEvent::Closed { id: 1, start: 10 * h, end: 25 * h + 2_400 });
        assert_eq!(ev[2], SegEvent::Opened { id: 2, start: 35 * h });
        assert_eq!(ev[3], SegEvent::Dropped { id: 2 });
    }

    #[test]
    fn long_speech_is_cut() {
        let mut s = Segmenter::new(1);
        s.max_len = 10 * HOP as u64;
        let mut ev = Vec::new();
        for i in 0..25u64 {
            ev.extend(s.push(&frame(true, true), i * HOP as u64, (i + 1) * HOP as u64));
        }
        let closed: Vec<_> = ev.iter().filter(|e| matches!(e, SegEvent::Closed { .. })).collect();
        assert_eq!(closed.len(), 2);
        assert_eq!(*closed[0], SegEvent::Closed { id: 1, start: 0, end: 10 * HOP as u64 });
        assert_eq!(s.current(), Some((3, 20 * HOP as u64, 25 * HOP as u64)));
    }
}
