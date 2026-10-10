//! The idle patrol (decision D45): while the bot stands idle (not
//! following, not moving, nobody speaking, others in the room), every
//! `orbit.idle_sweep_s` (60 s; 0: never) the head looks round the front
//! half (`vrc_nav::survey` with `front`: three views, the eyes' stereo
//! placing the name tags) while the user camera's lens snaps the other half
//! (`Orbit::sweep_views` from the way opposite the head): who is where, all
//! round, for the speaker tracker (the eyes' players placed; the lens's
//! names by their bearing) and the lasting map. A move or a voice ends the
//! lens's part (the orbit's move gate, its idle-sweep rule); the head's
//! look is short and leaves it facing as it did.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_nav::SurveyOptions;

use crate::bridge::Bridge;
use crate::Lock;

/// The scheduler looks this often.
const TICK: Duration = Duration::from_secs(1);
/// Moved this recently: not idle.
const STILL_FOR: Duration = Duration::from_secs(3);
/// The lens's views round the half behind (as many as the head's).
const LENS_VIEWS: u32 = 3;
/// The lens's sweep and its last reads (OCR) are waited for this long at
/// most after the head's look.
const LENS_WAIT: Duration = Duration::from_secs(3);

#[derive(Default)]
pub struct Patrol {
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    last: Option<Instant>,
    count: u64,
    /// The names the last patrol read: the eyes' (placed), the lens's.
    eyes: Vec<String>,
    lens: Vec<String>,
    took_ms: u64,
    skipped: Option<String>,
}

impl Patrol {
    pub fn status(&self) -> Value {
        let st = self.state.lk();
        json!({
            "count": st.count,
            "last_ago_s": st.last.map(|t| (t.elapsed().as_secs_f64() * 10.0).round() / 10.0),
            "took_ms": st.took_ms,
            "eyes": st.eyes,
            "lens": st.lens,
            "skipped": st.skipped,
        })
    }

    /// The scheduler (a thread of its own).
    pub fn run(self: Arc<Self>, bridge: Arc<Bridge>) {
        self.state.lk().last = Some(Instant::now());
        loop {
            std::thread::sleep(TICK);
            let every = bridge.usercam.settings.lk().orbit.idle_sweep_s;
            let due = every > 0.0 && self.state.lk().last.is_none_or(|t| t.elapsed().as_secs_f32() >= every);
            if !due {
                continue;
            }
            if let Some(why) = idle_busy(&bridge) {
                self.state.lk().skipped = Some(why.to_string());
                continue;
            }
            if let Err(e) = self.patrol(&bridge) {
                tracing::info!("idle patrol: {e:#}");
                self.state.lk().skipped = Some(format!("{e:#}"));
            }
        }
    }

    /// One patrol: the lens behind, the head ahead, at once.
    fn patrol(&self, b: &Arc<Bridge>) -> anyhow::Result<()> {
        let t0 = Instant::now();
        {
            let mut st = self.state.lk();
            st.last = Some(t0);
            st.skipped = None;
        }
        // The lens: the half behind the head (its sweep goes on while the
        // head looks; nothing sought).
        if let Some(head) = b.orbit.head() {
            if let Err(e) = b.orbit.sweep_views("idle", Some((head.yaw + 180.0).rem_euclid(360.0)), None, LENS_VIEWS) {
                tracing::debug!("idle patrol: no lens sweep: {e:#}");
            }
        }
        // The eyes: the front half (the head alone; the scan turns it back).
        let whitelist = b.social.whitelist_names();
        let opts = SurveyOptions { front: true, down: false, objects: false, ..Default::default() };
        let s = {
            let Some(mut vr) = b.vr.try_lk() else { anyhow::bail!("the headset is busy") };
            vrc_nav::survey(vr.rig(&whitelist)?, &opts, &[])?
        };
        for p in &s.players {
            let frame = s.shots.iter().find(|shot| shot.frame.capture_ns == p.seen_ns).map(|shot| &shot.frame);
            b.speaker.saw(std::slice::from_ref(p), frame);
        }
        vrc_nav::observe(&b.mapping.nav, &s);
        // The lens's sweep, and its reads in flight.
        let until = Instant::now() + LENS_WAIT;
        while (b.orbit.sweeping() || b.orbit.reading() > 0) && Instant::now() < until {
            std::thread::sleep(Duration::from_millis(50));
        }
        let mut lens: Vec<String> = b.orbit.names_since(t0).into_iter().map(|n| n.name).collect();
        lens.sort();
        lens.dedup();
        let mut st = self.state.lk();
        st.count += 1;
        st.eyes = s.players.iter().map(|p| p.name.clone()).collect();
        st.lens = lens;
        st.took_ms = t0.elapsed().as_millis() as u64;
        Ok(())
    }
}

/// Why the bot is not idle for a patrol now, if it is not.
fn idle_busy(b: &Bridge) -> Option<&'static str> {
    let (running, in_world, others) = {
        let g = b.game.lk();
        (g.running, !g.instance.is_empty(), !g.others().is_empty())
    };
    if !running || !in_world {
        return Some("not in a world");
    }
    if !others {
        return Some("nobody else in the room");
    }
    if !b.follower.is_idle() || b.follow_paused() {
        return Some("following");
    }
    if b.mapping.moved_within(STILL_FOR) {
        return Some("moving");
    }
    if b.speaker.speaking() {
        return Some("someone is speaking");
    }
    // A motion playing, or a posture held (sitting, lying): the head is
    // the program's.
    if b.anim.motion.lk().is_some() {
        return Some("a motion plays");
    }
    None
}
