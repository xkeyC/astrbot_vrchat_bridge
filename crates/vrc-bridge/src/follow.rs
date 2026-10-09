//! Following a player in the room (VR): their name says who, the
//! panorama's depth where.
//!
//! Two loops. The eyes (this thread, `look_pano`, decision D36): each look
//! is the latest pano frame, all round at once (the avatar's six cameras,
//! colour and metric depth; the only way the bot sees, decision D42): the
//! people in its depth, the target among them named (a name read on
//! someone: the user camera's lens, the plates over the eyes) or kept by
//! position (the person-shaped one nearest where they were, within a gate
//! that grows as they may walk; never someone named otherwise); each is a
//! fix of where they stand. The legs (a thread of their own, 25 times a
//! second): integrate the avatar's own velocity (OSCQuery) into an
//! odometry, so a fix half a second old still says how far the target is
//! *now*; turn towards them (walking follows the head, the body follows)
//! and set the thumbstick from the distance left to the standing distance,
//! braking smoothly as it shrinks: at full stick the avatar runs 4 m/s.
//! Without a pano frame (the avatar has none, the usual view leased for a
//! menu) nothing is looked at: the target goes unseen and the legs stand.
//!
//! In the way (each look's points, along the way to the target, measured
//! from the ground just before it, `steer`): whatever does not reach the
//! eyes is jumped first, with a run-up (user's rule). What reaches them,
//! or what a jump did not get past, is walked round by following it (a
//! "bug" algorithm, user's rule): keep it on one side (the side away from
//! the target's way round), each look taking the free heading nearest that
//! side, round corners, until the straight way to the target is open (no
//! wall up to the eyes; user's rule: keep checking while going round): out
//! of an enclosure or into one alike. Every way is in view: no glances.
//! While following a wall the target may be out of sight: the bot keeps
//! going for where they were. Pushing without moving (stuck) jumps once,
//! then backs off and turns aside.
//!
//! Heights go by the avatar's eyes (its OSCQuery eye height), not by fixed
//! numbers: a wall reaches over them; whatever is lower is jumped first.
//! What the avatar carries (a bag, a cup at its side) is in view near it:
//! nearer than 1.1 m a low thing counts only across the way (a ledge, a
//! sofa), not at one side (a prop).
//!
//! Lost (unseen LOST_AFTER, a second, walking or standing): the bot stands
//! and the lens's quick sweep goes first, six views from where they were
//! last seen out either way by turns (decision D41); their name read in
//! one: the lens turns there and the depth places them. Only then the bot
//! turns straight to where their name was last read, then to where they
//! were last placed, the lens asked each way (`lost_ways`); from the second
//! search on, round in views. A search that finds nobody walks a while
//! toward where they were, then searches again. Following ends only when
//! told to stop or when they leave the room.
//!
//! **Kept by position, briefly** (decision D40, `Track::place`, `Confirm`):
//! only while a name said it was them `kept_s` ago at most; from
//! CONFIRM_AFTER the lens looks at the one kept, and another name or no
//! name there drops it (lost at once: the search, the lens first, at where
//! their name last put them). A lens read of their name anywhere places
//! them at once (`seen_by: "lens"`), by the depth or by its bearing. The
//! front lens faces the target while following (`Orbit::aim_at`).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_players::names::match_score;
use vrc_vr::osc::Osc;
use vrc_vr::remote::FLOOR_Y;
use vrc_vr::scan::angle_diff;

use crate::bridge::Bridge;
use crate::orbit::{Lens, NameSighting};
use crate::panolook::{self, LookOptions};
use crate::Lock;
use vrc_players::OcrClient;

/// Standing distance (world metres) by default, and its limits.
const STAND_M: f32 = 1.5;
const MIN_STAND_M: f32 = 0.8;
const MAX_STAND_M: f32 = 7.0;
const CLOSER: f32 = 0.7;
const FARTHER: f32 = 1.4;
/// Walking starts this far past the standing distance (and goes on until
/// it is reached); backing off starts this far inside it, and goes on
/// until BACK_UNTIL inside.
const WALK_MARGIN: f32 = 0.3;
const BACK_MARGIN: f32 = 0.6;
const BACK_UNTIL: f32 = 0.15;
/// Something within this (world metres) straight ahead, nearer than the
/// target, stops the walk.
const OBSTACLE_M: f32 = 0.6;
/// The bot turning to look for someone (`turn_to`): SEARCH_STEP_S for
/// every SEARCH_STEP_DEG. Which way they went (logged when lost): the side
/// their last movement went, across the bot's line of sight to them
/// (moving within SEARCH_MOVED_FOR of being lost); standing, the side of
/// the bot's facing they were on.
const SEARCH_STEP_DEG: f32 = 60.0;
const SEARCH_STEP_S: f32 = 0.35;
const SEARCH_MOVED_FOR: Duration = Duration::from_secs(5);
/// Up or down from the bot by more than a step: it goes up to them (as near
/// as this), the stairs are no stopping place.
const OTHER_FLOOR_STAND_M: f32 = 0.8;
/// Not seen for this long, walking or standing: stand and search (the
/// user, 2026-10-09: lost over a second, the lens starts looking where they
/// were last seen, decision D41).
const LOST_AFTER: Duration = Duration::from_millis(1000);
/// Out of the room for this long: gone.
const GONE_AFTER: Duration = Duration::from_secs(20);
const PITCH: f32 = -10.0;
/// Search views: offsets (degrees) from where the target was last seen.
/// A search that found nobody: walk toward where they were this long.
const SEEK_FOR: Duration = Duration::from_secs(4);
/// A whole search found nothing: wait this long before the next.
const SEARCH_PAUSE: Duration = Duration::from_secs(2);
/// The panorama: a name read on someone counts this long; the plates over
/// the eyes are read (OCR) at most this often (more often while someone
/// speaks); the person kept by position is the one within KEEP_GATE_M (plus
/// KEEP_SPEED m/s since the last fix) of where they were, for KEEP_FOR.
const NAME_FRESH: Duration = Duration::from_secs(2);
const OVERLAY_EVERY: Duration = Duration::from_millis(800);
const OVERLAY_HOT: Duration = Duration::from_millis(400);
const KEEP_GATE_M: f32 = 0.6;
const KEEP_SPEED: f32 = 1.5;
const KEEP_FOR: Duration = Duration::from_secs(4);
/// Lost: their name's last sighting this recent is where to turn first;
/// two ways nearer than MERGE_DEG are one; each way the lens looks a while.
const NAME_SEEN_FOR: Duration = Duration::from_secs(30);
const MERGE_DEG: f32 = 20.0;
const ASK_WAIT: Duration = Duration::from_millis(1200);

// The legs, measured on VRChat (VR mode, thumbstick forward): 0.3 runs
// 0.9 m/s, 0.6 runs 2.2, 1.0 runs 4.0; about 0.3 s to speed up or stop.
const TICK: Duration = Duration::from_millis(40);
const AXIS_DEAD: f32 = 0.1;
const SPEED_PER_AXIS: f32 = 4.44;
const MAX_SPEED: f32 = 1.8;
/// Deceleration planned for (m/s²): gentle, well inside what VRChat does.
const BRAKE: f32 = 1.5;
/// Input to motion, and the stop itself, as seconds of the current speed.
const LAG_S: f32 = 0.25;
/// Slower than this is not worth a step.
const MIN_SPEED: f32 = 0.3;
const BACK_AXIS: f32 = -0.3;
/// The body (and the walk) turns to the target at most this fast (degrees
/// per second).
const TURN_RATE: f32 = 200.0;
/// Standing, the body squares up to the target once they are more than
/// SQUARE_FROM_DEG off, until within SQUARE_TO_DEG; nearer the middle the
/// head alone follows them (at most HEAD_MAX_DEG off the body, HEAD_RATE
/// degrees a second), up and down too: at the face under their name tag
/// (TAG_OVER_FACE_M) so the tag stays in view close up, the pitch within
/// HEAD_PITCH_MIN..HEAD_PITCH_MAX.
const SQUARE_FROM_DEG: f32 = 20.0;
const SQUARE_TO_DEG: f32 = 3.0;
const HEAD_MAX_DEG: f32 = 35.0;
const HEAD_RATE: f32 = 150.0;
const TAG_OVER_FACE_M: f32 = 0.3;
const HEAD_PITCH_MIN: f32 = -30.0;
const HEAD_PITCH_MAX: f32 = 30.0;
/// Fixes predict the target's motion at most this far ahead.
const PREDICT_S: f32 = 1.0;
const ODOMETRY_KEPT: Duration = Duration::from_secs(4);
/// Obstacles: standing more than STEP_M over the ground before them (lower
/// ones are walked up), with BIN_POINTS points in a 10 cm bin and
/// OBSTACLE_POINTS in it and the next two, spanning MIN_SPAN_M of height
/// (NEAR_POINTS nearer than 1.1 m): stray depth points come alone, a
/// wall met at a slant spreads thin over several bins (25 points a bin
/// missed such walls); reaching
/// the eyes (within EYE_MARGIN_M: the line of sight, whatever the avatar's
/// height), walked round, else jumped first (a ledge, a platform), this
/// far before them, after a run-up at least RUN_UP_SPEED fast.
const STEP_M: f32 = 0.3;
/// The ground along the way is followed up (and down) by at most STAIR_M a
/// 10 cm bin (a riser), and points as high as STAIRS_SLOPE times the way
/// out over the eyes may be stairs' (followed, not in the way).
const STAIR_M: f32 = 0.22;
const STAIRS_SLOPE: f32 = 1.0;
const BIN_POINTS: usize = 3;
const OBSTACLE_POINTS: usize = 12;
const NEAR_POINTS: usize = 12;
const MIN_SPAN_M: f32 = 0.05;
const EYE_MARGIN_M: f32 = 0.05;
const JUMP_AT_M: f32 = 0.5;
/// A jump that left the same obstacle (within this) in the way failed:
/// walk round it for a while.
const SAME_PLACE_M: f32 = 0.7;
const NO_JUMP_FOR: Duration = Duration::from_secs(10);
const RUN_UP_SPEED: f32 = 1.6;
const JUMP_EVERY: Duration = Duration::from_millis(1500);
/// After a jump the run goes on this long (in the air, over it).
const JUMP_CARRY: Duration = Duration::from_millis(900);
/// A heading chosen by a view is followed this long after it.
const DETOUR_FOR: Duration = Duration::from_millis(1200);
/// Stuck: pushing (stick past STUCK_AXIS) yet slower than STUCK_SPEED for STUCK_FOR.
const STUCK_AXIS: f32 = 0.25;
const STUCK_SPEED: f32 = 0.15;
const STUCK_FOR: Duration = Duration::from_millis(600);
const ESCAPE_FOR: Duration = Duration::from_millis(500);
/// Views judge only ways within this of where they look (degrees).
const VIEW_HALF_DEG: f32 = 40.0;
/// Following a wall: headings tried this far apart (degrees), each side of
/// where it looks; a heading is free when nothing stands within FREE_M
/// along it. The wall is left once, along the way to the target, nothing up
/// to the eyes stands in it (given up by the episode's watch, `Avoid`). A
/// wall met again within SAME_SIDE_FOR of leaving one is followed on the
/// same side.
const WALL_STEP_DEG: f32 = 15.0;
const WALL_STEPS: i32 = 3;
const FREE_M: f32 = 1.2;
const WALL_SPEED: f32 = 1.4;
const SAME_SIDE_FOR: Duration = Duration::from_secs(8);
/// A way round by the lasting map: legs this long at most, each held this
/// long (the next look plans again).
const MAP_LEG_M: f32 = 2.0;
const MAP_LEG_FOR: Duration = Duration::from_millis(1500);
/// Going round by the map holds this long after the last look that chose
/// it: meanwhile, like along a wall, the target may be out of sight.
const ROUTE_HOLD: Duration = Duration::from_secs(3);
/// The legs do not push on toward a pane on the map nearer than this
/// (between looks too: a way round held 1.5 s, then straight at the
/// target, into the glass).
const PANE_AHEAD_M: f32 = 0.7;

/// Going round something (a wall, the map's way, a jump, a way out of
/// being stuck) is one episode while its breaks are shorter than
/// AVOID_GRACE (decision D37). Every look (the panorama sees all round)
/// keeps the target's place; the way round is planned again for where
/// they are now when they moved REPLAN_M since it was last, or every
/// REPLAN_EVERY. Re-located (the panorama and the lens look for them, the
/// map plans afresh, the wall is tried the other way round) when the
/// episode got no nearer by PROGRESS_M for AVOID_STALL, or lasted
/// AVOID_FOR; not seen for AVOID_UNSEEN of it, they are lost (the search:
/// the lens first). After AVOID_ROUNDS re-locations with no AVOID_CLEAR
/// of free walking between, the bot stands REST_FOR facing them: never
/// round the same thing for ever.
const AVOID_GRACE: Duration = Duration::from_secs(2);
const REPLAN_M: f32 = 0.75;
const REPLAN_EVERY: Duration = Duration::from_millis(1500);
const PROGRESS_M: f32 = 0.5;
const AVOID_STALL: Duration = Duration::from_secs(15);
const AVOID_FOR: Duration = Duration::from_secs(30);
const AVOID_UNSEEN: Duration = Duration::from_secs(6);
const AVOID_ROUNDS: u32 = 3;
const AVOID_CLEAR: Duration = Duration::from_secs(5);
const REST_FOR: Duration = Duration::from_secs(8);
/// Going round, no name read on them this long: the lens looks their way
/// (at most every AVOID_LOOK_EVERY; the travel lens stays on the way
/// otherwise, re-aimed less often while going round).
const AVOID_NAME_STALE: Duration = Duration::from_secs(3);
const AVOID_LOOK_EVERY: Duration = Duration::from_secs(4);
const AVOID_CALM: Duration = Duration::from_secs(2);
/// Re-locating: the legs stand this long at most.
const RELOCATE_HOLD: Duration = Duration::from_secs(3);
/// Lost (the panorama): the lens's quick sweep begins where a name of
/// theirs read this recent put them (else where they were last placed);
/// only then the bot turns. With the panorama the whole sphere is in
/// every look: standing, unseen LOST_AFTER is lost too.
const LENS_LOOK_FRESH: Duration = Duration::from_secs(5);
/// Kept by position with no name read on them (decision D40): from
/// CONFIRM_AFTER the lens looks at the one kept (at most every
/// CONFIRM_EVERY, held CONFIRM_HOLD; walking, only as the move gate lets
/// it pause). Their name read: them. Another name read that way (within
/// CONFIRM_TOL_DEG, as far as CONFIRM_SAME_M), or CONFIRM_READS reads of
/// nobody's while their plate should show (nearer than CONFIRM_RANGE_M):
/// not them, dropped, lost at once. No answer by CONFIRM_HOLD plus
/// CONFIRM_GRACE: asked again later. And no name for `kept_s`: kept by
/// position no longer (dropped, lost).
const CONFIRM_AFTER: Duration = Duration::from_millis(1000);
const CONFIRM_EVERY: Duration = Duration::from_secs(3);
const CONFIRM_HOLD: Duration = Duration::from_millis(1500);
const CONFIRM_GRACE: Duration = Duration::from_millis(1000);
const CONFIRM_READS: u32 = 3;
const CONFIRM_TOL_DEG: f32 = 20.0;
const CONFIRM_SAME_M: f32 = 1.0;
const CONFIRM_RANGE_M: f32 = 8.0;
const KEPT_S: f32 = 7.0;
/// A name of theirs read with nobody under its ray this look (the lens's
/// bearing): a body this near its bearing (degrees) and, with a distance
/// known, this near it (metres) is them; else the bearing at the last
/// distance (DEFAULT_GAP_M with none).
const LENS_BODY_DEG: f32 = 8.0;
const LENS_BODY_M: f32 = 1.2;
const DEFAULT_GAP_M: f32 = 2.0;
/// The head's pitch at them settles over this (seconds), and moves only
/// when it would by more than PITCH_DEADBAND (degrees).
const PITCH_SMOOTH_S: f32 = 0.4;
const PITCH_DEADBAND: f32 = 1.5;
/// The quick sweep may wait this long for the legs to let go (they stop
/// as the follower does, lost).
const SWEEP_ASK_FOR: Duration = Duration::from_millis(500);
/// Its last reads (OCR) are waited for this long at most.
const SWEEP_READS_WAIT: Duration = Duration::from_millis(1500);
/// The last resort (from the second search on): the bot turns round this
/// many views, the plates over the eyes read at each.
const SCAN_VIEWS: i32 = 4;

/// A way round by the map is taken when no longer than this many times
/// the straight way, and this much (else round the wall in sight).
const MAP_DETOUR_MAX: f32 = 3.0;
const MAP_DETOUR_SLACK: f32 = 4.0;

/// What the lasting map says of the straight way to the target.
enum MapWay {
    /// Nothing shut on it (or no map yet).
    Clear,
    /// It runs into a way a walk was stopped going (glass, an invisible
    /// wall: the target seen through it), and the map has a way round: this
    /// heading first (session), and how long the way is (world metres).
    Round(f32, f32),
    /// Shut that far along, and no way round on the map.
    Shut(f32),
}

/// The lasting map on the straight way from `from` to `to` (the follow's
/// frame: session axes, world metres, the bot then at `from`), the target
/// `up` over the bot's floor.
#[allow(clippy::too_many_arguments)]
fn map_way(me: &Follower, bridge: &Bridge, from: [f32; 2], to: [f32; 2], up: f32, at: Instant, eyes_m: f32, wall_at: Option<f32>) -> MapWay {
    use vrc_nav::vrc_map::plan::{PlanParams, Planner};
    let nav = bridge.mapping.nav.lk();
    if !nav.ready() {
        return MapWay::Clear;
    }
    let start = nav.pose_at(at);
    let goal = nav.place([to[0] - from[0], up, to[1] - from[1]], at);
    if let Some(mut s) = me.inner.try_lock().ok() {
        s.goal_map = Some(goal);
        (s.map_way, s.route_m) = ("clear", 0.0);
    }
    let now = vrc_nav::vrc_map::unix_now();
    // Shut on the map, or a wall in sight (the map may know the way round:
    // ways walked before first).
    let Some(along) = nav.map.shut_along(start, goal, now).or(wall_at) else { return MapWay::Clear };
    let straight = (goal[0] - start[0]).hypot(goal[2] - start[2]);
    let p = PlanParams { max_expand: 150_000, now, ..vrc_nav::plan_params(eyes_m) };
    let mut planner = Planner::new(&nav.map, p, start);
    let leg = planner
        .plan([goal[0], goal[2]], Some(goal[1]))
        .filter(|p| p.reached && p.length <= MAP_DETOUR_MAX * straight + MAP_DETOUR_SLACK)
        .and_then(|path| planner.leg(&path, MAP_LEG_M).map(|l| (l.0, path.length)));
    let way = match leg {
        Some((heading, length)) => MapWay::Round(nav.session_heading(heading), length),
        None => MapWay::Shut(along),
    };
    if let Some(mut s) = me.inner.try_lock().ok() {
        (s.map_way, s.route_m) = match way {
            MapWay::Round(_, length) => ("round", length),
            MapWay::Shut(_) => ("shut", 0.0),
            MapWay::Clear => ("clear", 0.0),
        };
    }
    way
}

/// Whether walking `heading` (session) runs into a way shut on the lasting
/// map within `metres` (the map busy: no).
fn pane_ahead(bridge: &Bridge, heading: f32, metres: f32, wait: bool) -> bool {
    let nav = if wait { bridge.mapping.nav.lk() } else {
        match bridge.mapping.nav.try_lk() {
            Some(n) => n,
            None => return false,
        }
    };
    if !nav.ready() {
        return false;
    }
    let p = nav.pose;
    let (s, c) = nav.map_heading(heading).to_radians().sin_cos();
    nav.map.crosses_shut(p, [p[0] + s * metres, p[1], p[2] - c * metres], vrc_nav::vrc_map::unix_now())
}

/// The follow's settings (`POST /v1/follow {"settings": {...}}`; the
/// bridge's lifetime, not saved).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FollowSettings {
    /// Kept by position at most this long after their name was last read
    /// (seconds, 3-30; decision D40).
    pub kept_s: f32,
}

impl Default for FollowSettings {
    fn default() -> Self {
        FollowSettings { kept_s: KEPT_S }
    }
}

#[derive(Default)]
pub struct Follower {
    inner: Mutex<State>,
    settings: Mutex<FollowSettings>,
    /// The running follow's stop flag (a new one per follow).
    stop: Mutex<Arc<AtomicBool>>,
    /// The running follow's thread: the next waits for it to end.
    runner: Mutex<Option<std::thread::JoinHandle<()>>>,
}

#[derive(Default, Clone)]
struct State {
    target: String,
    state: &'static str,
    last_seen: Option<Instant>,
    distance: f32,
    obstacle: f32,
    stand: f32,
    hold: bool,
    moving: f32,
    running: bool,
    /// What the way ahead calls for: "", "detour", "jump", "blocked", "stuck".
    avoiding: &'static str,
    jumps: u32,
    /// The target on the lasting map (map frame), and what the map said of
    /// the way to them: "clear", "round" (and how long), "shut".
    goal_map: Option<[f32; 3]>,
    map_way: &'static str,
    route_m: f32,
    /// Where the target stands over the bot's floor (world metres).
    target_up: f32,
    /// How the last fix was made: "named" (a name read on them), "lens"
    /// (the lens read their name, placed by its ray), "kept" (by position).
    seen_by: &'static str,
    /// The going round's watch, as the last look left it.
    avoid: Avoid,
    /// Lost: the search's stage ("lens_ring", "body_turn", "scan"); empty
    /// otherwise.
    search_stage: &'static str,
    /// When a name last said it was them; the confirming of one kept by
    /// position, as the last look left it; the head's pitch as sent.
    named_at: Option<Instant>,
    confirm: Confirm,
    head_pitch: f32,
}

/// Confirming the one kept by position (decision D40, see CONFIRM_AFTER).
#[derive(Clone, Debug, Default)]
struct Confirm {
    /// The lens's look under way: since when, the world yaw it looks along
    /// (the one kept, from the head), and how far they were (metres).
    asked: Option<(Instant, f32, f32)>,
    last_ask: Option<Instant>,
    looks: u32,
    confirmed: u32,
    drops: u32,
    /// How the last ended ("named", "other name", "no name", "expired",
    /// "unanswered"), and when.
    last: &'static str,
    last_at: Option<Instant>,
}

/// What the confirming says after a look.
#[derive(Clone, Copy, Debug, PartialEq)]
enum ConfirmAct {
    Nothing,
    /// The lens to look along this world yaw.
    Look(f32),
    /// Not them: the one kept dropped, lost at once.
    Drop(&'static str),
}

/// A name read by the lens since the confirming look began: the name, its
/// world yaw from the head, how far (when the depth placed it).
struct Read<'a> {
    name: &'a str,
    world_yaw: f32,
    distance: Option<f32>,
}

impl Confirm {
    /// One look's confirming at `now`: their name last read at `named_at`;
    /// `kept` (world yaw from the head, metres) while they are kept by
    /// position (none: named this look, or not placed); what the lens read
    /// since the look began (`seen`) and how many reads it made under it
    /// (`reads`); `kept_for`, how long a kept one may go unnamed.
    #[allow(clippy::too_many_arguments)]
    fn tick(&mut self, now: Instant, named_at: Option<Instant>, kept: Option<(f32, f32)>, target: &str, seen: &[Read], reads: u32, kept_for: Duration) -> ConfirmAct {
        let unnamed = named_at.map_or(Duration::MAX, |n| now.saturating_duration_since(n));
        if let Some((asked, yaw, gap)) = self.asked {
            // Their name read since the look began (there or anywhere: the
            // follower took that place): them.
            // (Not kept any more: a name placed them, maybe a read from
            // before the look that the follower took only now.)
            if kept.is_none() || named_at.is_some_and(|n| n >= asked) || seen.iter().any(|r| match_score(r.name, target) >= 0.6) {
                self.end("named", now);
                self.confirmed += 1;
                return ConfirmAct::Nothing;
            }
            let other = seen.iter().any(|r| {
                match_score(r.name, target) < 0.6 && angle_diff(r.world_yaw, yaw).abs() <= CONFIRM_TOL_DEG && r.distance.is_none_or(|d| (d - gap).abs() <= CONFIRM_SAME_M)
            });
            if other {
                return self.dropped("other name", now);
            }
            if reads >= CONFIRM_READS && gap <= CONFIRM_RANGE_M {
                return self.dropped("no name", now);
            }
            if now.saturating_duration_since(asked) > CONFIRM_HOLD + CONFIRM_GRACE {
                self.end("unanswered", now);
            } else {
                return ConfirmAct::Nothing;
            }
        }
        let Some((yaw, gap)) = kept else { return ConfirmAct::Nothing };
        if unnamed > kept_for {
            return self.dropped("expired", now);
        }
        if unnamed >= CONFIRM_AFTER && self.last_ask.is_none_or(|l| now.saturating_duration_since(l) >= CONFIRM_EVERY) {
            self.asked = Some((now, yaw, gap));
            self.last_ask = Some(now);
            self.looks += 1;
            return ConfirmAct::Look(yaw);
        }
        ConfirmAct::Nothing
    }

    fn end(&mut self, why: &'static str, now: Instant) {
        self.asked = None;
        self.last = why;
        self.last_at = Some(now);
    }

    fn dropped(&mut self, why: &'static str, now: Instant) -> ConfirmAct {
        self.end(why, now);
        self.drops += 1;
        ConfirmAct::Drop(why)
    }

    fn status(&self, named_at: Option<Instant>) -> Value {
        let now = Instant::now();
        let s = |d: Duration| (d.as_secs_f64() * 10.0).round() / 10.0;
        json!({
            "unnamed_s": named_at.map(|n| s(now.saturating_duration_since(n))),
            "asking": self.asked.map(|(t, yaw, gap)| json!({"world_yaw": (yaw as f64 * 10.0).round() / 10.0, "distance_m": (gap as f64 * 100.0).round() / 100.0, "age_s": s(now.saturating_duration_since(t))})),
            "looks": self.looks,
            "confirmed": self.confirmed,
            "drops": self.drops,
            "last": if self.last.is_empty() { Value::Null } else { json!(self.last) },
            "last_ago_s": self.last_at.map(|t| s(now.saturating_duration_since(t))),
        })
    }
}

/// The watch over going round something (see AVOID_GRACE): one episode,
/// its counts, the re-locations.
#[derive(Clone, Debug, Default)]
struct Avoid {
    /// What the way calls for now ("wall", "map", "jump", "detour"), and
    /// since when the episode goes on; none: not going round.
    why: &'static str,
    since: Option<Instant>,
    /// Since when nothing is gone round (the episode ends AVOID_GRACE
    /// after; the re-locations in a row are forgotten AVOID_CLEAR after).
    clear_since: Option<Instant>,
    /// The target's place the way round was last planned for, and when.
    planned: Option<([f32; 2], Instant)>,
    /// The nearest the bot came to them in the episode, and when.
    best: Option<(f32, Instant)>,
    replans: u32,
    relocates: u32,
    in_row: u32,
    last_relocate: &'static str,
    rest_until: Option<Instant>,
    /// Looks of the lens their way while going round.
    lens_looks: u32,
}

/// What the watch says after a look.
#[derive(Clone, Copy, Debug, PartialEq)]
enum AvoidAct {
    Go,
    /// Plan the way round again, for where they are now.
    Replan,
    /// Stop going round; look for them (the panorama, the lens), then the
    /// map's way from scratch, the wall the other way round.
    Relocate(&'static str),
    /// Not seen going round this long: lost (the search).
    Lost,
    /// Too many re-locations in a row: stand, facing them.
    Rest,
}

impl Avoid {
    /// One look's watch at `now`: going round for `why` (none: not), the
    /// target at `goal`, `gap` from the bot, last placed at `seen`.
    fn tick(&mut self, now: Instant, why: Option<&'static str>, goal: [f32; 2], gap: f32, seen: Option<Instant>) -> AvoidAct {
        if let Some(r) = self.rest_until {
            if now < r {
                return AvoidAct::Rest;
            }
            self.rest_until = None;
            self.in_row = 0;
        }
        let Some(why) = why else {
            let clear = *self.clear_since.get_or_insert(now);
            let clear_for = now.saturating_duration_since(clear);
            if clear_for > AVOID_GRACE {
                self.end();
            }
            if clear_for > AVOID_CLEAR {
                self.in_row = 0;
            }
            return AvoidAct::Go;
        };
        self.clear_since = None;
        self.why = why;
        let Some(since) = self.since else {
            self.since = Some(now);
            self.planned = Some((goal, now));
            self.best = Some((gap, now));
            return AvoidAct::Go;
        };
        let best = match self.best {
            Some(b) if gap >= b.0 - PROGRESS_M => b,
            _ => (gap, now),
        };
        self.best = Some(best);
        let unseen = seen.map_or(Duration::MAX, |s| now.saturating_duration_since(s));
        if unseen.min(now.saturating_duration_since(since)) > AVOID_UNSEEN {
            self.relocated(now, "lost");
            return AvoidAct::Lost;
        }
        let why = if now.saturating_duration_since(best.1) > AVOID_STALL {
            "stalled"
        } else if now.saturating_duration_since(since) > AVOID_FOR {
            "timeout"
        } else {
            ""
        };
        if !why.is_empty() {
            return if self.relocated(now, why) { AvoidAct::Rest } else { AvoidAct::Relocate(why) };
        }
        let (at, when) = self.planned.unwrap_or((goal, now));
        if (goal[0] - at[0]).hypot(goal[1] - at[1]) > REPLAN_M || now.saturating_duration_since(when) >= REPLAN_EVERY {
            self.planned = Some((goal, now));
            self.replans += 1;
            return AvoidAct::Replan;
        }
        AvoidAct::Go
    }

    /// The episode ends in a re-location (`why`); whether that is one too
    /// many in a row (then the bot rests).
    fn relocated(&mut self, now: Instant, why: &'static str) -> bool {
        self.relocates += 1;
        self.in_row += 1;
        self.last_relocate = why;
        self.end();
        if why != "lost" && self.in_row >= AVOID_ROUNDS {
            self.rest_until = Some(now + REST_FOR);
            return true;
        }
        false
    }

    fn end(&mut self) {
        self.since = None;
        self.planned = None;
        self.best = None;
    }

    fn status(&self) -> Value {
        let now = Instant::now();
        let s = |d: Duration| (d.as_secs_f64() * 10.0).round() / 10.0;
        json!({
            "why": if self.since.is_some() { json!(self.why) } else { Value::Null },
            "age_s": self.since.map(|t| s(now.saturating_duration_since(t))),
            "progress_age_s": self.best.map(|b| s(now.saturating_duration_since(b.1))),
            "timeout_s": AVOID_FOR.as_secs(),
            "stall_s": AVOID_STALL.as_secs(),
            "unseen_s": AVOID_UNSEEN.as_secs(),
            "replans": self.replans,
            "relocates": self.relocates,
            "in_row": self.in_row,
            "last_relocate": if self.last_relocate.is_empty() { Value::Null } else { json!(self.last_relocate) },
            "resting_s": self.rest_until.filter(|r| now < *r).map(|r| s(r.saturating_duration_since(now))),
            "lens_looks": self.lens_looks,
        })
    }
}

/// Where the target stood, in the odometry's frame (world metres, axes of
/// the tracking space: +x right, +z back).
#[derive(Clone, Copy)]
struct Fix {
    at: Instant,
    pos: [f32; 2],
    /// Their velocity (m/s), from the fixes before.
    vel: [f32; 2],
    /// Where their feet are over the bot's floor (world metres): up on
    /// something, the way there is a jump up, not round the map.
    up: f32,
    /// Their name tag over the bot's eyes (world metres).
    tag_rise: f32,
}

/// What the eyes and the legs share.
#[derive(Default)]
struct Track {
    /// The avatar's position (odometry) and its recent history.
    pos: [f32; 2],
    history: VecDeque<(Instant, [f32; 2])>,
    /// Where the head (and so the walk) points, degrees.
    facing: f32,
    target: Option<Fix>,
    /// Something in the way to the target, as the last view saw it.
    obstacle: Option<Obstacle>,
    /// A way round it: (until when, heading).
    detour: Option<(Instant, f32)>,
    /// The last jump: when, and where the obstacle stood (odometry).
    jumped: Option<(Instant, [f32; 2])>,
    /// Obstacles a jump did not get past (odometry), until when.
    no_jump: Vec<([f32; 2], Instant)>,
    /// Following a wall round to the target.
    wall: Option<Wall>,
    /// The side of the last wall followed, and when it was left.
    last_side: Option<(f32, Instant)>,
    /// Walking toward where the target was (not seen), until when.
    seek: Option<Instant>,
    /// The way the target last moved (heading, degrees) and when.
    moved: Option<(f32, Instant)>,
    /// Views in a row that should have shown the target (in view, near)
    /// and did not read them.
    misses: u32,
    /// When a look last chose a way round by the lasting map.
    routed: Option<Instant>,
    /// That way's length (world metres) from where the bot then was
    /// (odometry): near them through glass is not there yet.
    route: Option<([f32; 2], f32)>,
    /// The panorama: where the target's feet were at the last fix (world),
    /// and when a name last said it was them.
    world: Option<([f32; 3], Instant)>,
    named_at: Option<Instant>,
    /// When the plates over the eyes were last read.
    overlay_read: Option<Instant>,
    /// The going round's watch, the re-location it asked for, and the
    /// legs standing until then (re-locating, resting).
    avoid: Avoid,
    relocate: Option<&'static str>,
    hold_until: Option<Instant>,
    /// The last thing in the way on the straight way (world metres).
    blocked_m: Option<f32>,
    /// When the lens last looked their way while going round.
    avoid_look: Option<Instant>,
    /// The fix the last name made (where the search looks first), the one
    /// kept by position now (world yaw from the head, metres; none when
    /// named), the confirming of it, and when one kept was dropped (lost
    /// at once, until a name is read again). Decision D40.
    named_fix: Option<Fix>,
    kept: Option<(f32, f32)>,
    confirm: Confirm,
    dropped: Option<Instant>,
}

/// What one look saw, for steering (`Follower::steer`): when, the
/// odometry then, whether the target was among it, where the head looked
/// (degrees), a glance (the head alone, along a wall), the name tags read,
/// the eyes and the floor (tracking space), world metres a tracking unit,
/// the points in view (tracking space), and whether the view is all round
/// (a panorama: every way is in view).
struct View {
    at: Instant,
    then: [f32; 2],
    found: bool,
    yaw: f32,
    glance: bool,
    tags: usize,
    eye: [f32; 3],
    floor: f32,
    metres: f32,
    points: Vec<[f32; 3]>,
    all_round: bool,
}

/// Following a wall: on which side it is kept (-1 left, +1 right), and
/// since when.
#[derive(Clone, Copy, Debug)]
struct Wall {
    side: f32,
    since: Instant,
}

/// Something in the way (world metres).
#[derive(Clone, Copy, Debug)]
struct Obstacle {
    /// The heading it lies along, and the odometry then.
    yaw: f32,
    then: [f32; 2],
    /// How far along the heading.
    distance: f32,
    /// To be jumped (else walked round).
    jump: bool,
}

impl Obstacle {
    /// How far ahead it is now, walking along `facing` from `pos`; `None`
    /// when walking another way.
    fn ahead(&self, pos: [f32; 2], facing: f32) -> Option<f32> {
        if angle_diff(facing, self.yaw).abs() > 25.0 {
            return None;
        }
        let (s, c) = self.yaw.to_radians().sin_cos();
        Some(self.distance - ((pos[0] - self.then[0]) * s - (pos[1] - self.then[1]) * c))
    }

    /// Where it stands (odometry).
    fn place(&self) -> [f32; 2] {
        let (s, c) = self.yaw.to_radians().sin_cos();
        [self.then[0] + s * self.distance, self.then[1] - c * self.distance]
    }
}

/// What stands in the way, from the view's points (world metres).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Blocker {
    /// How far ahead its front is.
    distance: f32,
    /// Its top over the ground before it.
    top: f32,
    /// It reaches (nearly) up to the eyes: no jumping that.
    tall: bool,
}

/// Where everything over a step counts (nearer, what reaches over the eyes,
/// or a low thing across the way): past where props reach.
const FROM_ANYWAY: f32 = 1.1;

/// The first thing in the way along `yaw` from `eye`, in a corridor half a
/// metre wide out to 4 m; `None` when the way is clear. The ground is
/// followed along the corridor (slopes, stairs), so a raised floor is not in
/// the way; something stands in the way when it rises more than a step
/// over the ground there. Nearer than `from` the bot's own body is in
/// view too (arms swinging forward, a gesture, props it carries: a weapon on
/// the back, a cup at its side reaching 1 m out), whatever the avatar: there
/// only points over the eyes count, so a wall (which reaches that high) is
/// seen however near, and a body, a prop or a ledge (jumped) is not; a low
/// thing that near was seen from farther, and the odometry counts down to
/// it.
fn corridor(points: &[[f32; 3]], eye: [f32; 3], yaw: f32, metres: f32, floor: f32, from: f32) -> Option<Blocker> {
    const BIN: f32 = 0.1;
    // Nearer, the head's own (hair, ears) is in view.
    const NEAR_FROM: f32 = 0.5;
    // Off the middle by this much counts as a side.
    const SIDE: f32 = 0.08;
    let from = from.clamp(NEAR_FROM + BIN, FROM_ANYWAY);
    // From NEAR_FROM out to 4 m; nearer than `from` a low thing counts only
    // across the way (both sides: a ledge, not a prop at one side), and the
    // ground is followed (a slope already rising there).
    const BINS: usize = 36;
    let first = ((from - NEAR_FROM) / BIN).round() as usize;
    let (s, c) = yaw.to_radians().sin_cos();
    let eye_m = (eye[1] - floor) * metres;
    // Per 10 cm along the way: the points in it (ahead, height, side), up
    // to as high as stairs rising from here could go.
    let mut bins: Vec<Vec<(f32, f32, f32)>> = vec![Vec::new(); BINS];
    for p in points {
        let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
        let (ahead, side, up) = ((dx * s - dz * c) * metres, (dx * c + dz * s) * metres, (p[1] - floor) * metres);
        if ahead > NEAR_FROM && side.abs() < 0.25 && up < eye_m + 0.3 + ahead * STAIRS_SLOPE {
            let i = ((ahead - NEAR_FROM) / BIN) as usize;
            if i < BINS {
                bins[i].push((ahead, up, side));
            }
        }
    }
    // The ground under each bin, as it is reached from here: followed up
    // and down by STAIR_M a bin at most (a slope, stairs: each bin's lowest
    // point), not up a ledge (and a drop is not followed).
    let mut ground = vec![0.0f32; BINS];
    let mut g = 0.0f32;
    for (i, bin) in bins.iter().enumerate() {
        ground[i] = g;
        let low = bin.iter().map(|a| a.1).fold(f32::INFINITY, f32::min);
        if (low - g).abs() <= STAIR_M {
            g = low;
        }
    }
    // Over the ground there (up to a little over the eyes: not a ceiling
    // over stairs).
    let (bins, ground) = (&bins, &ground);
    let over = |i: usize| bins[i].iter().map(move |a| (a.0, a.1 - ground[i], a.2)).filter(move |a| a.1 < eye_m + 0.3);
    // Near: only what reaches up toward the eyes (a wall), not the body.
    let near: Vec<f32> = (0..BINS).flat_map(over).filter(|a| a.0 <= from && a.1 > eye_m).map(|a| a.0).collect();
    if near.len() >= NEAR_POINTS {
        let distance = near.iter().copied().fold(f32::INFINITY, f32::min);
        return Some(Blocker { distance, top: eye_m, tall: true });
    }
    for i in 0..BINS {
        if bins[i].is_empty() {
            continue;
        }
        let above: Vec<(f32, f32, f32)> = over(i).filter(|a| a.1 > STEP_M).collect();
        // A wall met at a slant spreads over several bins, a few points
        // each; stray points come alone: count this bin and the next two,
        // and want some height to them.
        let window: Vec<(f32, f32, f32)> = (i..(i + 3).min(BINS)).flat_map(over).filter(|a| a.1 > STEP_M).collect();
        let span = window.iter().map(|a| a.1).fold(f32::NEG_INFINITY, f32::max) - window.iter().map(|a| a.1).fold(f32::INFINITY, f32::min);
        let across = || {
            let left = window.iter().filter(|a| a.2 < -SIDE).count();
            let right = window.iter().filter(|a| a.2 > SIDE).count();
            left >= OBSTACLE_POINTS / 2 && right >= OBSTACLE_POINTS / 2
        };
        let stands = above.len() >= BIN_POINTS && window.len() >= OBSTACLE_POINTS && span >= MIN_SPAN_M;
        if stands && (i >= first || across()) {
            let distance = above.iter().map(|a| a.0).fold(f32::INFINITY, f32::min);
            // Its top: the highest point within half a metre past the front.
            let top = (i..(i + 6).min(BINS)).flat_map(over).filter(|a| a.0 < distance + 0.5).map(|a| a.1).fold(0.0f32, f32::max);
            return Some(Blocker { distance, top, tall: top >= eye_m - EYE_MARGIN_M });
        }
    }
    None
}

impl Track {
    /// Going round by the lasting map (the target may be out of sight).
    fn routing(&self) -> bool {
        self.routing_at(Instant::now())
    }

    fn routing_at(&self, now: Instant) -> bool {
        self.routed.is_some_and(|r| now.saturating_duration_since(r) < ROUTE_HOLD)
    }

    /// Going round something (a wall, or by the map): the target may be out
    /// of sight, and the walk goes on for where they were.
    fn rounding(&self) -> bool {
        self.rounding_at(Instant::now())
    }

    fn rounding_at(&self, now: Instant) -> bool {
        self.wall.is_some() || self.routing_at(now)
    }

    /// What going round calls for at `now`, if anything: a wall followed,
    /// the map's way, a jump coming, a heading held (out of being stuck).
    fn avoiding_at(&self, now: Instant) -> Option<&'static str> {
        if self.wall.is_some() {
            Some("wall")
        } else if self.routing_at(now) {
            Some("map")
        } else if self.obstacle.is_some_and(|o| o.jump) {
            Some("jump")
        } else if self.detour.is_some_and(|d| now < d.0) {
            Some("detour")
        } else {
            None
        }
    }

    /// Stops going round: the wall, the map's way, the heading held; with
    /// `flip` (a re-location) the next wall is followed the other way round.
    fn drop_round(&mut self, now: Instant, flip: bool) {
        let side = self.wall.map(|w| w.side).or(self.last_side.map(|l| l.0));
        if let (true, Some(side)) = (flip, side) {
            self.last_side = Some((-side, now));
        }
        self.wall = None;
        self.routed = None;
        self.route = None;
        self.detour = None;
        self.obstacle = None;
    }

    /// The odometry at `t` (interpolated from the history).
    fn pos_at(&self, t: Instant) -> [f32; 2] {
        let mut before: Option<&(Instant, [f32; 2])> = None;
        for h in &self.history {
            if h.0 >= t {
                return match before {
                    Some(b) if h.0 > b.0 => {
                        let f = (t - b.0).as_secs_f32() / (h.0 - b.0).as_secs_f32();
                        [b.1[0] + (h.1[0] - b.1[0]) * f, b.1[1] + (h.1[1] - b.1[1]) * f]
                    }
                    _ => h.1,
                };
            }
            before = Some(h);
        }
        self.pos
    }

    /// The target's position now (predicted from the last fix).
    fn target_now(&self) -> Option<[f32; 2]> {
        self.target_at(Instant::now())
    }

    fn target_at(&self, now: Instant) -> Option<[f32; 2]> {
        let f = self.target?;
        let ahead = now.saturating_duration_since(f.at).as_secs_f32().min(PREDICT_S);
        Some([f.pos[0] + f.vel[0] * ahead, f.pos[1] + f.vel[1] * ahead])
    }

    /// The head's pitch (degrees) to their face under the name tag, from
    /// here: the tag stays in view close up and up on something.
    fn tag_pitch(&self) -> Option<f32> {
        let (f, goal) = (self.target?, self.target_now()?);
        let gap = (goal[0] - self.pos[0]).hypot(goal[1] - self.pos[1]);
        let pitch = (f.tag_rise - TAG_OVER_FACE_M).atan2(gap.max(0.3)).to_degrees();
        Some(pitch.clamp(HEAD_PITCH_MIN, HEAD_PITCH_MAX))
    }

    fn add_fix(&mut self, at: Instant, pos: [f32; 2], up: f32, tag_rise: f32) {
        let vel = match self.target {
            Some(prev) if at > prev.at && (at - prev.at) < Duration::from_secs(2) && (at - prev.at) > Duration::from_millis(200) => {
                let dt = (at - prev.at).as_secs_f32();
                let v = [(pos[0] - prev.pos[0]) / dt, (pos[1] - prev.pos[1]) / dt];
                let v = [0.5 * prev.vel[0] + 0.5 * v[0], 0.5 * prev.vel[1] + 0.5 * v[1]];
                let speed = v[0].hypot(v[1]);
                // Standing still, give or take the noise of the depth.
                if speed < 0.3 {
                    [0.0, 0.0]
                } else if speed > 3.0 {
                    [v[0] * 3.0 / speed, v[1] * 3.0 / speed]
                } else {
                    v
                }
            }
            _ => [0.0, 0.0],
        };
        if vel != [0.0, 0.0] {
            self.moved = Some((bearing([0.0, 0.0], vel), at));
        }
        self.target = Some(Fix { at, pos, vel, up, tag_rise });
    }

    /// A name said it was them (the fix just made, at `at`): kept no more,
    /// nothing dropped, the confirming over.
    fn named(&mut self, at: Instant) {
        self.named_at = Some(self.named_at.map_or(at, |n| n.max(at)));
        self.named_fix = self.target;
        self.kept = None;
        self.dropped = None;
    }

    /// The one kept by position was not them (`why`, decision D40): kept no
    /// more, going round stops, and the target is where their name last
    /// put them, unseen since: lost at once (the search, the legs
    /// standing).
    fn drop_kept(&mut self, now: Instant, why: &'static str) {
        tracing::info!(why, unnamed_s = self.named_at.map(|n| now.saturating_duration_since(n).as_secs_f32()), "follow: the one kept by position is not them: lost");
        self.world = None;
        self.kept = None;
        self.dropped = Some(now);
        self.seek = None;
        self.drop_round(now, false);
        self.target = self.named_fix.map(|f| Fix { vel: [0.0, 0.0], ..f }).or(self.target.map(|f| Fix { at: f.at.min(now.checked_sub(KEEP_FOR).unwrap_or(f.at)), ..f }));
    }

    /// One panorama look's placing of the target (decision D36, D40):
    /// `people` (name, how it came, body), the lens's reads of their name
    /// lately (`heard`, newest first), the head per `tr`, at `at`. Named
    /// (the plates, the lens) wherever; else the one kept by position, only
    /// while a name said it was them `kept_for` ago at most (else dropped:
    /// lost); a lens read since the last name that found nobody under its
    /// ray this look, or found them elsewhere than the one kept, wins (the
    /// depth's body near it, or its bearing at the last distance). The fix
    /// is made; what it was, if any.
    fn place(&mut self, people: &[(Option<String>, Option<&'static str>, vrc_pano::Body)], target: &str, heard: &[NameSighting], tr: &vrc_pano::Tracking, at: Instant, kept_for: Duration) -> Option<Placed> {
        let (eye, metres) = (tr.head_track, tr.metres);
        let fresh_name = self.named_at.is_some_and(|n| at.saturating_duration_since(n) <= kept_for);
        // Kept fixes renewed the place look after look: a sign board was
        // "them" for two minutes (decision D40).
        if self.world.is_some() && !fresh_name {
            if self.kept.is_some() && self.dropped.is_none() {
                self.drop_kept(at, "expired");
                let _ = self.confirm.dropped("expired", at);
            } else {
                self.world = None; // unseen since their name: nothing to keep
            }
        }
        let last = self.world.filter(|_| fresh_name && self.dropped.is_none());
        let picked = pick(people, target, last, at);
        let gap = self.target_at(at).map(|g| {
            let then = self.pos_at(at);
            (g[0] - then[0]).hypot(g[1] - then[1])
        });
        let lens = heard
            .iter()
            .find(|n| match_score(&n.name, target) >= 0.6)
            .filter(|n| self.named_at.is_none_or(|named| n.at > named))
            .filter(|_| picked.is_none_or(|(_, how)| how == "kept"))
            .map(|n| lens_place(n, people, tr, gap));
        // (when, feet (world), plate (world y), how)
        let (when, feet, plate_y, how) = match (lens, picked) {
            (Some(LensPlace::Body(k)), _) => (at, people[k].2.feet, people[k].2.plate()[1], "lens"),
            (Some(LensPlace::Point { feet, plate_y, at: when }), _) => (when, feet, plate_y, "lens"),
            (None, Some((k, how))) => (at, people[k].2.feet, people[k].2.plate()[1], how),
            (None, None) => return None,
        };
        let body = people.iter().find(|q| (q.2.feet[0] - feet[0]).hypot(q.2.feet[2] - feet[2]) < 0.05).map(|q| q.2);
        let f = tr.point(feet);
        let rel = [(f[0] - eye[0]) * metres, (f[2] - eye[2]) * metres];
        let then = self.pos_at(when);
        // Up on something: their lowest point well over the floor.
        let up = match body {
            Some(b) if b.low - b.feet[1] > 0.45 => b.low - b.feet[1],
            Some(_) => 0.0,
            None => self.target.map_or(0.0, |f| f.up),
        };
        self.add_fix(when, [then[0] + rel[0], then[1] + rel[1]], up, plate_y - tr.head_world[1]);
        self.world = Some((feet, when));
        let world_yaw = (feet[0] - tr.head_world[0]).atan2(feet[2] - tr.head_world[2]).to_degrees().rem_euclid(360.0);
        let gap = rel[0].hypot(rel[1]);
        if how == "kept" {
            self.kept = Some((world_yaw, gap));
        } else {
            self.named(when);
            if how == "lens" {
                tracing::info!(world_yaw = world_yaw.round(), gap, body = body.is_some(), "follow: the lens read their name: there");
            }
        }
        self.misses = 0;
        self.seek = None;
        Some(Placed { when, how, world_yaw, gap, up })
    }

    /// The tracking yaw from the bot to where their name last put them.
    fn name_way(&self) -> Option<f32> {
        self.named_fix.filter(|f| f.at.elapsed() < NAME_SEEN_FOR).map(|f| bearing(self.pos, f.pos))
    }

    /// The search's way round from `from` (where they were last seen, a
    /// bearing): +1 right, -1 left.
    fn search_way(&self, from: f32) -> f32 {
        let side = match self.moved {
            // Going right across the line of sight to them: round to the right.
            Some((way, when)) if when.elapsed() < SEARCH_MOVED_FOR => angle_diff(way, from),
            _ => angle_diff(from, self.facing),
        };
        if side < 0.0 {
            -1.0
        } else {
            1.0
        }
    }
}

impl Follower {
    pub fn status(&self) -> Value {
        let s = self.inner.lk();
        json!({
            "target": s.target,
            "state": if s.state.is_empty() { "idle" } else { s.state },
            "last_seen_s": s.last_seen.map(|t| (t.elapsed().as_secs_f64() * 10.0).round() / 10.0),
            "distance_m": (s.distance as f64 * 100.0).round() / 100.0,
            "obstacle_m": if s.obstacle.is_finite() { json!((s.obstacle as f64 * 100.0).round() / 100.0) } else { Value::Null },
            "distance": (if s.stand > 0.0 { s.stand } else { STAND_M } as f64 * 10.0).round() / 10.0,
            "hold": s.hold,
            "move": s.moving,
            "avoiding": s.avoiding,
            "jumps": s.jumps,
            "target_up_m": (s.target_up as f64 * 100.0).round() / 100.0,
            "goal_map": s.goal_map.map(|g| g.map(|v| (v as f64 * 100.0).round() / 100.0)),
            "map_way": s.map_way,
            "route_m": (s.route_m as f64 * 100.0).round() / 100.0,
            "seen_by": if s.seen_by.is_empty() { Value::Null } else { json!(s.seen_by) },
            "avoid": s.avoid.status(),
            "search_stage": if s.search_stage.is_empty() { Value::Null } else { json!(s.search_stage) },
            "confirm": s.confirm.status(s.named_at),
            "head_pitch_deg": (s.head_pitch as f64 * 10.0).round() / 10.0,
            "settings": {"kept_s": self.settings.lk().kept_s},
        })
    }

    /// Changes the settings (`{"kept_s": 7}`), checked.
    pub fn set(&self, change: &Value) -> anyhow::Result<()> {
        anyhow::ensure!(change.is_object(), "settings is an object: {{\"kept_s\": 7}}");
        let mut s = *self.settings.lk();
        if let Some(v) = change.get("kept_s") {
            let k = v.as_f64().ok_or_else(|| anyhow::anyhow!("kept_s is a number"))? as f32;
            anyhow::ensure!((3.0..=30.0).contains(&k), "kept_s is 3-30");
            s.kept_s = k;
        }
        *self.settings.lk() = s;
        Ok(())
    }

    fn kept_for(&self) -> Duration {
        Duration::from_secs_f32(self.settings.lk().kept_s)
    }

    pub fn is_idle(&self) -> bool {
        !self.inner.lk().running
    }

    /// Follows `name`, standing `distance` world metres away.
    pub fn start(self: &Arc<Self>, bridge: &Arc<Bridge>, name: &str, distance: Option<f32>) {
        let stop = Arc::new(AtomicBool::new(false));
        // Swapped under the lock: a start at the same moment cannot leave
        // a flag nobody sets.
        std::mem::replace(&mut *self.stop.lk(), stop.clone()).store(true, Ordering::SeqCst);
        {
            let mut s = self.inner.lk();
            *s = State {
                target: name.to_string(),
                state: "following",
                last_seen: None,
                stand: distance.unwrap_or(STAND_M).clamp(MIN_STAND_M, MAX_STAND_M),
                obstacle: f32::INFINITY,
                running: true,
                ..Default::default()
            };
        }
        // After the follow before has let go of the stick and the headset.
        let mut runner = self.runner.lk();
        let before = runner.take();
        let (me, b) = (self.clone(), bridge.clone());
        *runner = Some(std::thread::spawn(move || {
            if let Some(before) = before {
                let _ = before.join();
            }
            if !stop.load(Ordering::SeqCst) {
                me.run(b, stop);
            }
        }));
        drop(runner);
        bridge.notify_state();
    }

    /// The follow's state to write, while `stop` is the running follow's
    /// (a stopped one, still finishing a look, writes nothing).
    fn inner_for(&self, stop: &AtomicBool) -> Option<std::sync::MutexGuard<'_, State>> {
        let s = self.inner.lk();
        (!stop.load(Ordering::SeqCst)).then_some(s)
    }

    pub fn stop(&self) {
        self.stop.lk().store(true, Ordering::SeqCst);
        let mut s = self.inner.lk();
        s.running = false;
        s.state = "idle";
        s.moving = 0.0;
    }

    /// closer / farther / stay / resume.
    pub fn adjust(&self, change: &str) -> anyhow::Result<()> {
        let mut s = self.inner.lk();
        if !s.running {
            anyhow::bail!("not following anyone");
        }
        match change {
            "closer" => (s.stand, s.hold) = ((s.stand * CLOSER).clamp(MIN_STAND_M, MAX_STAND_M), false),
            "farther" => (s.stand, s.hold) = ((s.stand * FARTHER).clamp(MIN_STAND_M, MAX_STAND_M), false),
            "stay" => s.hold = true,
            "resume" => s.hold = false,
            _ => anyhow::bail!("change must be closer, farther, stay or resume"),
        }
        Ok(())
    }

    fn run(self: Arc<Self>, bridge: Arc<Bridge>, stop: Arc<AtomicBool>) {
        let target = self.inner.lk().target.clone();
        let facing = {
            let mut vr = bridge.vr.lk();
            if stop.load(Ordering::SeqCst) {
                return; // stopped while it waited for the headset
            }
            vr.forget_places(); // following moves the bot
            vr.yaw
        };
        let track = Arc::new(Mutex::new(Track { facing, ..Default::default() }));
        // The front lens faces the target while they are placed (D40).
        bridge.orbit.set_following(true);
        let legs = {
            let (me, b, t, s) = (self.clone(), bridge.clone(), track.clone(), stop.clone());
            std::thread::spawn(move || me.legs(&b, &t, &s))
        };
        // However the follow ends (a panic too): the legs stop, the stick
        // is let go, and the state says so if no other follow took over.
        let _done = Done { me: self.clone(), bridge: bridge.clone(), stop: stop.clone(), legs: Some(legs) };
        let mut last_here = Instant::now();
        let mut searching = false;
        // Search rounds since they were lost (`search_stages`).
        let mut rounds = 0u32;
        let mut next_search = Instant::now();
        // Since when no panorama was usable.
        let mut no_pano: Option<Instant> = None;
        while !stop.load(Ordering::SeqCst) {
            let (running, here, room) = {
                let g = bridge.game.lk();
                let room: Vec<String> = g.others().into_iter().map(|(_, n)| n).collect();
                (g.running, room.iter().any(|n| match_score(n, &target) >= 0.8), room)
            };
            if !running {
                break;
            }
            if here || room.is_empty() {
                last_here = Instant::now();
            } else if last_here.elapsed() > GONE_AFTER {
                if !stop.load(Ordering::SeqCst) {
                    bridge.send_event(json!({"type": "follow", "state": "gone", "target": target}));
                }
                break;
            }
            let standing = self.inner.lk().moving.abs() < 0.05;
            // The panorama: all round in one frame, the only way the bot
            // sees (decision D42). Without it (the avatar has none, the
            // usual view leased for a menu) nothing is looked at: the
            // target goes unseen and the legs stand.
            if !bridge.pano.usable() {
                if no_pano.is_none() {
                    no_pano = Some(Instant::now());
                    tracing::info!("follow: no panorama: waiting for it");
                }
                std::thread::sleep(Duration::from_millis(200));
                continue;
            }
            no_pano = None;
            // (Going round a wall long out of sight of them ends by the
            // episode's watch, `Avoid`: lost, the search.)
            // The one kept by position dropped (not them, decision D40):
            // lost at once, the lens looking first.
            let (lost, unseen) = {
                let t = track.lk();
                let unseen = t.target.map_or(Duration::MAX, |f| f.at.elapsed());
                let seeking = t.seek.is_some_and(|until| Instant::now() < until);
                (t.dropped.is_some() || (!t.rounding() && !seeking && unseen > LOST_AFTER), unseen)
            };
            let result = if !lost {
                self.look_pano(&bridge, &track, &target, &room, false, &stop).map(|_| ())
            } else if Instant::now() >= next_search {
                if !searching {
                    searching = true;
                    rounds = 0;
                    {
                        let t = track.lk();
                        let (gap, off, up) = match (t.target, t.target_now()) {
                            (Some(f), Some(g)) => ((g[0] - t.pos[0]).hypot(g[1] - t.pos[1]), angle_diff(bearing(t.pos, g), t.facing).round(), f.up),
                            _ => (f32::NAN, f32::NAN, f32::NAN),
                        };
                        let from = t.target_now().map_or(t.facing, |p| bearing(t.pos, p));
                        let going = t.moved.filter(|m| m.1.elapsed() < SEARCH_MOVED_FOR).map(|m| m.0.round());
                        tracing::info!(unseen_s = unseen.as_secs_f32(), gap, off, up, standing, misses = t.misses, ?going, way = t.search_way(from), "follow: lost them, searching");
                    }
                    // The event after the lock: sending it reads the follow's
                    // state (status), and the lock is not reentrant.
                    let current = self.inner_for(&stop).map(|mut s| s.state = "searching").is_some();
                    if current {
                        bridge.send_event(json!({"type": "follow", "state": "searching", "target": target}));
                    }
                }
                let found = self.find_pano(&bridge, &track, &target, &room, &mut rounds, &stop);
                if !matches!(found, Ok(true)) {
                    // Nobody all round: walk toward where they were a while.
                    let mut t = track.lk();
                    let far = t.target_now().is_some_and(|g| (g[0] - t.pos[0]).hypot(g[1] - t.pos[1]) > self.inner.lk().stand + 0.5);
                    if far {
                        t.seek = Some(Instant::now() + SEEK_FOR);
                        next_search = Instant::now() + SEEK_FOR;
                    } else {
                        next_search = Instant::now() + SEARCH_PAUSE;
                    }
                }
                found.map(|_| ())
            } else {
                std::thread::sleep(Duration::from_millis(100));
                Ok(())
            };
            if let Err(e) = result {
                tracing::warn!("follow round failed: {e:#}");
                bridge.vr.lk().reset();
                std::thread::sleep(Duration::from_millis(500));
            }
            // Going round got nowhere: re-located (the watch asked for it);
            // else, going round, the lens their way now and then.
            let ask = track.lk().relocate.take();
            if let Some(why) = ask {
                if let Err(e) = self.relocate(&bridge, &track, &target, &room, why, &stop) {
                    tracing::warn!("follow: re-locating failed: {e:#}");
                }
                let mut t = track.lk();
                if t.avoid.rest_until.is_none() {
                    t.hold_until = None;
                }
            } else if !lost {
                // Kept by position, no name a while: the lens looks at the
                // one kept (D40); else, going round, the lens their way.
                if !self.confirm_kept(&bridge, &track, &target, &stop) {
                    self.watch_round(&bridge, &track);
                }
            }
            let seen = track.lk().target.is_some_and(|f| f.at.elapsed() < LOST_AFTER);
            if seen && searching {
                searching = false;
                let current = self
                    .inner_for(&stop)
                    .map(|mut s| {
                        s.state = "following";
                        s.search_stage = "";
                    })
                    .is_some();
                if current {
                    bridge.send_event(json!({"type": "follow", "state": "found", "target": target}));
                }
            }
        }
    }

    /// The way to the target as one view shows it (`steer_way`), and the
    /// watch over going round (`supervise`). With the track held (`t`).
    fn steer(&self, bridge: &Arc<Bridge>, t: &mut Track, view: &View, stop: &AtomicBool) -> anyhow::Result<bool> {
        let found = self.steer_way(bridge, t, view, stop)?;
        self.supervise(bridge, t, view, stop);
        Ok(found)
    }

    /// The watch over going round something (`Avoid`, decision D37), after
    /// a look placed the target (or did not): the way round planned again
    /// for where they are now, a re-location asked for (the run's loop
    /// does it, holding nothing), lost, or a rest. With the track held.
    fn supervise(&self, bridge: &Arc<Bridge>, t: &mut Track, view: &View, stop: &AtomicBool) {
        let at = view.at;
        let Some(goal) = t.target_at(at) else { return };
        let gap = (goal[0] - view.then[0]).hypot(goal[1] - view.then[1]);
        let why = t.avoiding_at(at);
        // Progress: round by the map, the way's length (it may lead away
        // from them first); else the straight gap.
        let left = t.route.filter(|_| t.routing_at(at)).map_or(gap, |r| r.1);
        let act = t.avoid.tick(at, why, goal, left, t.target.map(|f| f.at));
        let avoiding = match act {
            AvoidAct::Go => None,
            AvoidAct::Replan => {
                // Along a wall: the map's way for where they are now, if it
                // knows one by now (the way round by the map is planned
                // afresh every look anyway).
                tracing::debug!(goal = ?goal.map(|v| (v * 10.0).round() / 10.0), why, "follow: going round: planned again for where they are");
                if t.wall.is_some() && !view.glance {
                    let up = t.target.map_or(0.0, |f| f.up);
                    let eyes_m = (view.eye[1] - view.floor) * view.metres;
                    if let MapWay::Round(heading, length) = map_way(self, bridge, view.then, goal, up, at, eyes_m, Some(t.blocked_m.unwrap_or(gap))) {
                        t.wall = None;
                        t.routed = Some(at);
                        t.route = Some((view.then, length));
                        t.detour = Some((at + MAP_LEG_FOR, heading));
                        t.obstacle = None;
                        tracing::info!(heading = heading.round(), length, "follow: going round: the map's way for where they are now");
                        Some("map")
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            AvoidAct::Relocate(why) => {
                tracing::info!(why, relocates = t.avoid.relocates, in_row = t.avoid.in_row, gap, "follow: going round got nowhere: re-locating");
                t.drop_round(at, true);
                t.relocate = Some(why);
                t.hold_until = Some(at + RELOCATE_HOLD);
                Some("relocate")
            }
            AvoidAct::Lost => {
                tracing::info!(relocates = t.avoid.relocates, "follow: going round, not seen: lost");
                t.drop_round(at, true);
                Some("")
            }
            AvoidAct::Rest => {
                if t.hold_until != t.avoid.rest_until {
                    tracing::info!(in_row = t.avoid.in_row, "follow: going round again and again: standing a while");
                    t.drop_round(at, true);
                }
                t.drop_round(at, false);
                t.hold_until = t.avoid.rest_until;
                Some("rest")
            }
        };
        if let Some(mut s) = self.inner_for(stop) {
            if let Some(a) = avoiding {
                s.avoiding = a;
            }
            s.avoid = t.avoid.clone();
        }
    }

    /// The way to the target as one view shows it (following a wall, where
    /// they were): the lasting map's way round, what is in the corridor,
    /// a wall to follow, a jump. With the track held (`t`).
    fn steer_way(&self, bridge: &Arc<Bridge>, t: &mut Track, view: &View, stop: &AtomicBool) -> anyhow::Result<bool> {
        let View { at, then, found, yaw, glance, tags, eye, floor, metres, ref points, all_round } = *view;
        // The way to them, as far as this view shows it (following a wall,
        // or seeking them: where they were).
        let seeking = t.seek.is_some_and(|until| at < until);
        let goal = if t.rounding_at(at) || seeking {
            t.target_at(at)
        } else {
            t.target.filter(|f| at.saturating_duration_since(f.at) < LOST_AFTER).and_then(|_| t.target_at(at))
        };
        let Some(goal) = goal else {
            return Ok(found);
        };
        let direct = bearing(then, goal);
        let gap = (goal[0] - then[0]).hypot(goal[1] - then[1]);
        let in_view = all_round || angle_diff(direct, yaw).abs() <= VIEW_HALF_DEG;
        if !found && in_view && gap < 8.0 && !glance {
            t.misses += 1;
            if t.misses == 2 || t.misses % 6 == 0 {
                let unseen = t.target.map_or(f32::NAN, |f| at.saturating_duration_since(f.at).as_secs_f32());
                tracing::info!(misses = t.misses, unseen_s = unseen, gap, off = angle_diff(direct, yaw).round(), tags, "follow: in view, not read");
            }
        }
        // The lasting map: a way shut on the straight way to them (glass
        // between: seen through it, never walked through): round by the map.
        let target_up = t.target.map_or(0.0, |f| f.up);
        let by_map = if glance { MapWay::Clear } else { map_way(self, bridge, then, goal, target_up, at, (eye[1] - floor) * metres, None) };
        if let MapWay::Round(heading, length) = by_map {
            t.routed = Some(at);
            t.route = Some((then, length));
            t.detour = Some((at + MAP_LEG_FOR, heading));
            t.wall = None;
            t.obstacle = None;
            if let Some(mut s) = self.inner_for(stop) {
                s.avoiding = "map";
            }
            tracing::info!(heading = heading.round(), direct = direct.round(), "follow: round by the map");
            return Ok(found);
        }
        // Their own body (within half a metre of their feet) is not in the way.
        let blocked = if in_view { corridor(points, eye, direct, metres, floor, FROM_ANYWAY).filter(|b| b.distance < gap - 0.5 && b.distance < 3.0) } else { None };
        // They stand up on something: its edge (up to their feet and a bit)
        // is jumped onto, however high, not walked round the whole map.
        let up = t.target.map_or(0.0, |f| f.up);
        let blocked = blocked.map(|mut b| {
            if up > STEP_M && b.top <= up + 0.3 {
                b.tall = false;
            }
            b
        });
        // A wall in sight on the way (up to the eyes: lower things are
        // jumped first): the map's way round it, if it knows one (ways
        // walked before first), before feeling along it.
        if let (MapWay::Clear, Some(b)) = (&by_map, blocked.filter(|b| b.tall)) {
            if !glance && t.wall.is_none() {
                if let MapWay::Round(heading, length) = map_way(self, bridge, then, goal, target_up, at, (eye[1] - floor) * metres, Some(b.distance)) {
                    t.routed = Some(at);
                    t.route = Some((then, length));
                    t.detour = Some((at + MAP_LEG_FOR, heading));
                    t.obstacle = None;
                    if let Some(mut s) = self.inner_for(stop) {
                        s.avoiding = "map";
                    }
                    tracing::info!(heading = heading.round(), direct = direct.round(), length, "follow: round a wall by the map");
                    return Ok(found);
                }
            }
        }
        // Shut on the map and no way round on it: a wall there, whatever
        // the eyes say (followed round as any wall).
        let blocked = match by_map {
            MapWay::Shut(d) if blocked.is_none_or(|b| b.distance > d) => Some(Blocker { distance: d, top: (eye[1] - floor) * metres, tall: true }),
            _ => blocked,
        };
        if in_view {
            t.blocked_m = blocked.map(|b| b.distance);
        }
        // Free: nothing in sight, and no pane on the map (glass is not in
        // sight: feeling along it, the way picked went into it, the legs
        // stopped short of it, and the bot stood).
        let free = |h: f32| corridor(points, eye, h, metres, floor, FROM_ANYWAY).is_none_or(|b| b.distance > FREE_M) && !pane_ahead(bridge, h, FREE_M, true);
        let mut guard = self.inner_for(stop);
        let mut scratch = State::default(); // a stopped follow's: not kept
        let s: &mut State = match guard.as_deref_mut() {
            Some(s) => s,
            None => &mut scratch,
        };
        s.obstacle = blocked.map_or(f32::INFINITY, |b| b.distance);
        if let Some(w) = t.wall {
            // Nothing up to the eyes the straight way, nor a low thing a
            // jump did not clear (one not yet tried is jumped, below):
            // leave the wall for them.
            let (ds, dc) = direct.to_radians().sin_cos();
            let failed = |b: &Blocker| {
                let q = [then[0] + ds * b.distance, then[1] - dc * b.distance];
                t.no_jump.iter().any(|&(n, until)| at < until && (n[0] - q[0]).hypot(n[1] - q[1]) < SAME_PLACE_M)
            };
            let open = in_view && blocked.is_none_or(|b| !b.tall && !failed(&b));
            if open {
                t.wall = None;
                t.detour = None;
                t.last_side = Some((w.side, at));
                s.avoiding = "";
            } else if glance {
                return Ok(found); // a glance their way: the walk along the wall goes on
            } else {
                // A hand on the wall: turn toward it only where it opens up
                // (a corner: 30 degrees or more its way free), keep the way
                // while it is free, else the free heading nearest the wall.
                let at_side = |k: i32| wrap(yaw + w.side * k as f32 * WALL_STEP_DEG);
                // An opening is wide (30 and 45 degrees its way both free):
                // one free heading between blocked ones is a gap or noise,
                // and turning into it only turns back next view (swaying).
                let opening = Some(at_side(2)).filter(|_| (2..=WALL_STEPS).all(|k| free(at_side(k))));
                let keep = Some(yaw).filter(|&h| free(h));
                let away = (-WALL_STEPS..=1).rev().map(at_side).find(|&h| free(h));
                let heading = opening.or(keep).or(away).unwrap_or_else(|| wrap(yaw - w.side * 70.0)); // boxed in: turn away
                t.detour = Some((at + DETOUR_FOR, heading));
                tracing::info!(
                    yaw = yaw.round(),
                    side = w.side,
                    for_s = at.saturating_duration_since(w.since).as_secs_f32().round(),
                    heading = heading.round(),
                    pick = if opening.is_some() { "opening" } else if keep.is_some() { "keep" } else if away.is_some() { "away" } else { "boxed" },
                    free = %(-3..=3).map(|k| if free(at_side(k)) { '.' } else { '#' }).collect::<String>(),
                    "follow: wall"
                );
                s.avoiding = "wall";
                return Ok(found);
            }
        }
        if !in_view {
            return Ok(found); // not in view: the turn goes on
        }
        let Some(b) = blocked else {
            t.obstacle = None;
            t.detour = None;
            s.avoiding = "";
            return Ok(found);
        };
        let mut obstacle = Obstacle { yaw: direct, then, distance: b.distance, jump: false };
        let place = obstacle.place();
        let near = |q: [f32; 2]| (q[0] - place[0]).hypot(q[1] - place[1]) < SAME_PLACE_M;
        // Still there after a jump at it: that jump failed.
        if let Some((when, q)) = t.jumped {
            let since = at.saturating_duration_since(when);
            if since > Duration::from_millis(600) && since < Duration::from_secs(4) && near(q) {
                t.no_jump.push((q, at + NO_JUMP_FOR));
                t.jumped = None;
            }
        }
        t.no_jump.retain(|&(_, until)| at < until);
        // Not up to the eyes, and not failed yet: over it, first.
        obstacle.jump = !b.tall && !t.no_jump.iter().any(|&(q, _)| near(q));
        if obstacle.jump {
            t.obstacle = Some(obstacle);
            t.detour = None;
            s.avoiding = "jump";
            return Ok(found);
        }
        // Else round it, following it: go the side with a free heading
        // nearest the way to them, keeping the wall on the other side.
        t.obstacle = None;
        let again = t.last_side.filter(|&(_, when)| at - when < SAME_SIDE_FOR).map(|(side, _)| side);
        let side = again.unwrap_or_else(|| (1..=4)
            .find_map(|k| {
                let (r, l) = (wrap(direct + k as f32 * WALL_STEP_DEG), wrap(direct - k as f32 * WALL_STEP_DEG));
                if free(r) {
                    Some(-1.0) // round to the right: the wall on the left
                } else if free(l) {
                    Some(1.0)
                } else {
                    None
                }
            })
            .unwrap_or(1.0));
        t.wall = Some(Wall { side, since: at });
        t.detour = None;
        s.avoiding = "wall";
        Ok(found)
    }

    /// One look at the panorama: its people, the target among them (named,
    /// or kept by position), the way to them (`steer`). Whether the target
    /// was found.
    fn look_pano(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], read_plates: bool, stop: &AtomicBool) -> anyhow::Result<bool> {
        let whitelist = bridge.social.whitelist_names();
        let mut names: Vec<String> = room.to_vec();
        if !names.iter().any(|n| n == target) {
            names.push(target.to_string());
        }
        // The plates over the eyes: an OCR, now and then.
        let every = if bridge.speaker.speaking() { OVERLAY_HOT } else { OVERLAY_EVERY };
        let overlay = {
            let mut t = track.lk();
            let due = read_plates || t.overlay_read.is_none_or(|r| r.elapsed() >= every);
            if due {
                t.overlay_read = Some(Instant::now());
            }
            due
        };
        let ocr = if overlay { ocr_client(bridge) } else { None };
        let o = LookOptions { names: true, overlay: ocr.is_some(), lens_within: NAME_FRESH, ..Default::default() };
        let l = panolook::look(bridge, &names, ocr.as_ref(), &o)?;
        // Where things were when the frame was taken (decode and OCR take
        // a few hundred ms, walking on): the beacon's fix, the look on the
        // lasting map and the target placed by the odometry then.
        let at = l.taken;
        let tr = l.tracking;
        let (eye, metres) = (tr.head_track, tr.metres);
        let floor = l.cloud.floor.map_or(FLOOR_Y, |f| tr.point([tr.head_world[0], f, tr.head_world[2]])[1]);
        let yaw = l.eyes.views[0].pose.yaw_pitch().0;
        // Who speaks (their plates' rings), and friends seen, as before.
        l.to_speakers(bridge, &whitelist);
        let seen = l.sightings(&whitelist);
        for s in seen.iter().filter(|s| s.whitelist_rank.is_some()) {
            if let Ok(jpeg) = crate::vr::view_jpeg(&l.frame, tr.world_yaw(bearing_of(eye, s.feet)), 0.0, FORWARD_FOV, 640) {
                bridge.sightings.saw(&s.name, &bridge.game.lk().world_name, jpeg);
            }
        }
        let points: Vec<[f32; 3]> = l.cloud.points.iter().map(|&p| tr.point(p)).collect();
        vrc_nav::beacon_fix(&bridge.mapping.nav, &l.eyes, (eye[1] - floor) * metres, at);
        let feet: Vec<[f32; 3]> = seen.iter().map(|s| s.feet).collect();
        bridge.mapping.observe(vrc_nav::vrc_map::Observation::from_tracking(&points, eye, floor, metres, &feet, at));
        let people: Vec<(Option<String>, Option<&'static str>, vrc_pano::Body)> = l.people.iter().map(|q| (q.name.clone(), q.how, q.body)).collect();
        // Their name read by the lens lately (any lens: the front, the
        // travel lens, a look, a sweep), newest first.
        let heard: Vec<NameSighting> = bridge
            .orbit
            .names_since(at.checked_sub(NAME_FRESH).unwrap_or(at))
            .into_iter()
            .rev()
            .filter(|n| match_score(&n.name, target) >= 0.6)
            .collect();
        let mut t = track.lk();
        let then = t.pos_at(at);
        let placed = t.place(&people, target, &heard, &tr, at, self.kept_for());
        if let Some(p) = &placed {
            // The front lens faces them (standing, following).
            bridge.orbit.aim_at(p.world_yaw);
            if let Some(mut s) = self.inner_for(stop) {
                s.last_seen = Some(p.when);
                s.distance = p.gap;
                s.target_up = p.up;
                s.seen_by = p.how;
            }
        }
        if let Some(mut s) = self.inner_for(stop) {
            s.named_at = t.named_at;
            s.confirm = t.confirm.clone();
        }
        let found = placed.is_some();
        let view = View { at, then, found, yaw, glance: false, tags: seen.len(), eye, floor, metres, points, all_round: true };
        self.steer(bridge, &mut t, &view, stop)
    }

    /// Lost, with the panorama (decisions D37, D41): the lens first, the
    /// bot standing (it is quick, and nothing turns): the quick sweep
    /// (`lens_ring`), its first view where their name read moments ago
    /// put them, else where they were last placed, the rest out either way
    /// by turns; their name read: the lens turns there and the depth
    /// places them along its ray (the next look). Only then the bot turns:
    /// to where their name was last read, then to where they were last
    /// placed, the lens asked each way (`body_turn`); from the second
    /// search on, round in SCAN_VIEWS views, the plates over the eyes read
    /// at each (`scan`). True as soon as a look finds them.
    fn find_pano(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], rounds: &mut u32, stop: &AtomicBool) -> anyhow::Result<bool> {
        *rounds += 1;
        let found = self.find_stages(bridge, track, target, room, *rounds, stop);
        if matches!(found, Ok(true)) {
            self.stage(stop, "");
        }
        found
    }

    fn stage(&self, stop: &AtomicBool, stage: &'static str) {
        if let Some(mut s) = self.inner_for(stop) {
            s.search_stage = stage;
        }
    }

    fn find_stages(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], rounds: u32, stop: &AtomicBool) -> anyhow::Result<bool> {
        let theirs = |within: Duration| {
            bridge.orbit.names_since(Instant::now().checked_sub(within).unwrap_or_else(Instant::now)).into_iter().rev().find(|n| match_score(&n.name, target) >= 0.6)
        };
        let stopped = || stop.load(Ordering::SeqCst);
        // Where their name was last read (the plates, the lens: the follower
        // took it), from here; else the lens's read moments ago.
        let name_way = {
            let t = track.lk();
            t.name_way()
        };
        let fresh = theirs(LENS_LOOK_FRESH);
        let head = bridge.orbit.head();
        // Where they were last placed (tracking), into the world.
        let placed_way = {
            let t = track.lk();
            t.target_now().map(|p| bearing(t.pos, p))
        };
        let to_world = |y: f32| head.map(|h| (y + h.offset).rem_euclid(360.0));
        let look_way = name_way.and_then(to_world).or(fresh.as_ref().map(|n| n.world_yaw)).or(placed_way.and_then(to_world));
        let mut read: Option<NameSighting> = None;
        for stage in search_stages(rounds) {
            if stopped() {
                return Ok(false);
            }
            self.stage(stop, stage);
            match stage {
                // The lens's quick sweep, the bot standing: from where
                // they were out either way.
                "lens_ring" => {
                    tracing::info!(world_yaw = look_way.map(f32::round), "follow: lost them: the lens's quick sweep");
                    read = sweep_for(bridge, target, look_way, stop);
                    if let Some(n) = &read {
                        tracing::info!(world_yaw = n.world_yaw.round(), "follow: lost them: the sweep read their name");
                        if self.look_pano(bridge, track, target, room, false, stop)? {
                            return Ok(true);
                        }
                    }
                }
                // The bot turns: to their name (the sweep's first), then
                // to where they were.
                "body_turn" => {
                    let named = read.clone().or_else(|| theirs(NAME_SEEN_FOR));
                    let last = {
                        let t = track.lk();
                        t.target_now().map(|p| bearing(t.pos, p))
                    };
                    let ways = lost_ways(named.as_ref().map(|n| n.tracking_yaw), last);
                    tracing::info!(?ways, "follow: lost them (panorama)");
                    for (yaw, why) in ways {
                        if stopped() {
                            return Ok(false);
                        }
                        self.turn_to(bridge, track, yaw)?;
                        // The lens that way: a name read there names someone there.
                        let world = bridge.orbit.head().map(|h| (yaw + h.offset).rem_euclid(360.0));
                        if let Some(w) = world {
                            if let Err(e) = bridge.orbit.name_toward(w, 25.0, Duration::ZERO, ASK_WAIT) {
                                tracing::debug!("follow: the lens: {e:#}");
                            }
                        }
                        tracing::info!(yaw = yaw.round(), why, "follow: looking for them");
                        if self.look_pano(bridge, track, target, room, false, stop)? {
                            return Ok(true);
                        }
                    }
                }
                // The last resort: round, the plates over the eyes read
                // every view.
                _ => {
                    let from = track.lk().facing;
                    for k in 1..=SCAN_VIEWS {
                        if stopped() {
                            return Ok(false);
                        }
                        let yaw = wrap(from + 360.0 * k as f32 / SCAN_VIEWS as f32);
                        self.turn_to(bridge, track, yaw)?;
                        tracing::info!(yaw = yaw.round(), "follow: looking for them (round)");
                        if self.look_pano(bridge, track, target, room, true, stop)? {
                            return Ok(true);
                        }
                    }
                }
            }
        }
        Ok(false)
    }

    /// The bot turned (gently, the head held still) to `yaw` (tracking).
    fn turn_to(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, yaw: f32) -> anyhow::Result<()> {
        let whitelist = bridge.social.whitelist_names();
        let mut vr = bridge.vr.lk();
        let secs = (angle_diff(yaw, vr.yaw).abs() / SEARCH_STEP_DEG * SEARCH_STEP_S).max(SEARCH_STEP_S * 0.5);
        vr.rig(&whitelist)?.hmd.hold_still(true)?;
        let turned = vr.turn_gently(yaw, PITCH, secs, 0.0, 0.0);
        vr.rig(&whitelist)?.hmd.hold_still(false)?;
        turned?;
        track.lk().facing = yaw;
        Ok(())
    }

    /// Re-locating them after going round got nowhere (`why`; the legs
    /// stand meanwhile, `hold_until`): with the panorama a look all round
    /// (the plates over the eyes read), then the lens to where their name
    /// was last read and to where they were last placed, a look after
    /// each; without, a look their way. Whether a look found them. The
    /// next look plans the way afresh for where they are (the map, the
    /// wall the other way round).
    fn relocate(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], why: &'static str, stop: &AtomicBool) -> anyhow::Result<bool> {
        tracing::info!(why, "follow: re-locating them");
        let last = {
            let t = track.lk();
            t.target_now().map(|p| bearing(t.pos, p))
        };
        let named_now = |track: &Arc<Mutex<Track>>| track.lk().named_at.is_some_and(|n| n.elapsed() < NAME_FRESH);
        let found = self.look_pano(bridge, track, target, room, true, stop)?;
        if found && named_now(track) {
            return Ok(true);
        }
        let named = bridge
            .orbit
            .names_since(Instant::now().checked_sub(NAME_SEEN_FOR).unwrap_or_else(Instant::now))
            .into_iter()
            .rev()
            .find(|n| match_score(&n.name, target) >= 0.6);
        let ways = lost_ways(named.map(|n| n.tracking_yaw), last);
        for (yaw, way) in ways {
            if stop.load(Ordering::SeqCst) {
                break;
            }
            let Some(head) = bridge.orbit.head() else { break };
            tracing::info!(yaw = yaw.round(), way, "follow: re-locating: the lens");
            if let Err(e) = bridge.orbit.name_toward((yaw + head.offset).rem_euclid(360.0), 25.0, Duration::ZERO, ASK_WAIT) {
                tracing::debug!("follow: the lens: {e:#}");
            }
            if self.look_pano(bridge, track, target, room, false, stop)? && named_now(track) {
                return Ok(true);
            }
        }
        Ok(found || track.lk().target.is_some_and(|f| f.at.elapsed() < LOST_AFTER))
    }

    /// Going round (an episode of the watch): the travel lens kept calm,
    /// and, no name read on them for AVOID_NAME_STALE, the lens their way
    /// a moment (at most every AVOID_LOOK_EVERY; the walk stalls as for a
    /// re-aim): the panorama sees all round, the names come from the lens.
    /// Holds nothing while the lens moves.
    fn watch_round(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>) {
        let now = Instant::now();
        let look = {
            let mut t = track.lk();
            if t.avoid.since.is_none() {
                return;
            }
            let due = t.named_at.is_none_or(|n| now.saturating_duration_since(n) > AVOID_NAME_STALE)
                && t.avoid_look.is_none_or(|l| now.saturating_duration_since(l) > AVOID_LOOK_EVERY);
            match (due, t.target_now()) {
                (true, Some(goal)) => {
                    t.avoid_look = Some(now);
                    Some(bearing(t.pos, goal))
                }
                _ => None,
            }
        };
        bridge.orbit.calm_travel(AVOID_CALM);
        if let (Some(yaw), Some(head)) = (look, bridge.orbit.head()) {
            match bridge.orbit.look_at((yaw + head.offset).rem_euclid(360.0), ASK_WAIT) {
                Ok(()) => track.lk().avoid.lens_looks += 1,
                Err(e) => tracing::debug!("follow: going round, the lens: {e:#}"),
            }
        }
    }

    /// Kept by position with no name a while (decision D40, `Confirm`):
    /// the lens looks at the one kept (rate-limited; walking, only as the
    /// move gate lets it pause), and what it reads there confirms them or
    /// drops the one kept (lost at once: the search). Whether the lens was
    /// asked to look. Holds nothing while the lens moves.
    fn confirm_kept(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, stop: &AtomicBool) -> bool {
        let now = Instant::now();
        let asked = track.lk().confirm.asked.map(|a| a.0);
        let (heard, reads) = match asked {
            Some(a) => (bridge.orbit.names_since(a), bridge.orbit.reads_since(a, Lens::Look)),
            None => (Vec::new(), 0),
        };
        // Under the look (theirs: from any lens).
        let seen: Vec<Read> = heard
            .iter()
            .filter(|n| n.lens == Lens::Look || match_score(&n.name, target) >= 0.6)
            .map(|n| Read { name: &n.name, world_yaw: n.world_yaw, distance: n.distance_m })
            .collect();
        let act = {
            let mut t = track.lk();
            let (named_at, kept) = (t.named_at, t.kept);
            let act = t.confirm.tick(now, named_at, kept, target, &seen, reads, self.kept_for());
            if let ConfirmAct::Drop(why) = act {
                t.drop_kept(now, why);
            }
            if let Some(mut s) = self.inner_for(stop) {
                s.confirm = t.confirm.clone();
            }
            act
        };
        let ConfirmAct::Look(yaw) = act else { return false };
        tracing::info!(world_yaw = yaw.round(), "follow: kept by position, no name a while: the lens looks");
        match bridge.orbit.look_at(yaw, CONFIRM_HOLD) {
            Ok(()) => true,
            Err(e) => {
                // Not now (moving and no pause, the camera busy or closed):
                // asked again later; `kept_s` ends it meanwhile.
                tracing::debug!("follow: the lens could not look: {e:#}");
                let mut t = track.lk();
                t.confirm.asked = None;
                t.confirm.looks = t.confirm.looks.saturating_sub(1);
                false
            }
        }
    }

    /// The legs: odometry, turning to the target, the thumbstick.
    fn legs(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, stop: &AtomicBool) {
        let mut osc: Option<Osc> = None;
        let mut last = Instant::now();
        let mut axis = 0.0f32;
        // What was last sent: the body's yaw, the head's yaw and pitch.
        let mut aimed = [f32::NAN; 3];
        let (mut head_yaw, mut head_pitch) = (f32::NAN, PITCH);
        // The pitch at them, smoothed (D40: the plate's height comes from
        // the depth's body or the lens's read, each look a little off).
        let mut smooth_pitch = PITCH;
        let mut shown_pitch = f32::NAN;
        let mut squaring = false;
        let mut last_jump: Option<Instant> = None;
        let mut slow_since: Option<Instant> = None;
        let mut pushed_since: Option<Instant> = None;
        let mut escape: Option<Instant> = None;
        let mut aside = 1.0f32;
        let mut link: Option<vrc_vr::remote::HmdLink> = None;
        let mut strafe = 0.0f32;
        while !stop.load(Ordering::SeqCst) {
            std::thread::sleep(TICK);
            if osc.is_none() {
                osc = bridge.osc_query().ok();
            }
            let velocity = osc.as_ref().and_then(|o| {
                Some((o.query("/avatar/parameters/VelocityX").ok()? as f32, o.query("/avatar/parameters/VelocityZ").ok()? as f32))
            });
            let Some((vx, vz)) = velocity else {
                osc = None;
                self.set_axis(bridge, &mut axis, 0.0);
                if strafe != 0.0 {
                    strafe = 0.0;
                    let _ = bridge.osc.send_f32("/input/Horizontal", 0.0);
                }
                continue;
            };
            let now = Instant::now();
            let dt = (now - last).as_secs_f32().min(0.2);
            last = now;
            let (stand, hold) = {
                let s = self.inner.lk();
                (s.stand, s.hold)
            };
            // The way the head looks (walking follows it): from the headset
            // itself, as walks of a way round turn it too.
            if link.is_none() {
                link = bridge.vr.try_lk().and_then(|vr| vr.link());
            }
            let owner = link.as_ref().and_then(|l| l.owner());
            if owner.is_none() {
                link = None; // the headset connected again since: its new link next tick
            }
            let heading = owner.map(|o| o.state.head.yaw_pitch().0);
            let mut t = track.lk();
            // Odometry: the avatar's velocity is its own (z ahead, x right).
            let (fs, fc) = heading.unwrap_or(t.facing).to_radians().sin_cos();
            let (ahead, right) = ([fs, -fc], [fc, fs]);
            for k in 0..2 {
                t.pos[k] += (vz * ahead[k] + vx * right[k]) * dt;
            }
            let pos = t.pos;
            // Up or down stairs from the bot: right up to them.
            let stand = if t.target.is_some_and(|f| f.up.abs() > STEP_M) { stand.min(OTHER_FLOOR_STAND_M) } else { stand };
            t.history.push_back((now, pos));
            while t.history.front().is_some_and(|h| now - h.0 > ODOMETRY_KEPT) {
                t.history.pop_front();
            }
            let seeking = t.seek.is_some_and(|until| now < until);
            let fresh = t.rounding() || seeking || t.target.is_some_and(|f| f.at.elapsed() < LOST_AFTER);
            // Re-locating or resting (the watch): standing, facing them.
            let hold = hold || t.hold_until.is_some_and(|h| now < h);
            let walling = t.wall.is_some();
            let mut jump = false;
            let want = match t.target_now() {
                _ if escape.is_some_and(|e| now < e) => BACK_AXIS,
                Some(goal) if fresh && !hold => {
                    let (gx, gz) = (goal[0] - pos[0], goal[1] - pos[1]);
                    let gap = gx.hypot(gz);
                    // Round by the map, what is left of the way round, not
                    // the straight line (they may be just beyond the glass).
                    let gap = match t.route.filter(|_| t.routing()) {
                        Some((from, length)) => gap.max(length - (pos[0] - from[0]).hypot(pos[1] - from[1])),
                        None => gap,
                    };
                    // Round something, or straight to them.
                    // Along a wall the last way a view chose holds until the
                    // next view (one may take longer than DETOUR_FOR): never
                    // straight at them through the wall meanwhile.
                    let detour = t.detour.filter(|d| now < d.0 || walling).map(|d| d.1);
                    // Walking, the body turns the way it goes; standing, it
                    // squares up to them once they are well off the middle
                    // (the head alone follows them nearer it).
                    let walks = detour.is_some() || axis > 0.0 || gap - stand > WALK_MARGIN;
                    if gap > 0.3 || detour.is_some() {
                        let to = detour.unwrap_or_else(|| bearing(pos, goal));
                        let off = angle_diff(to, t.facing);
                        if walks || squaring || off.abs() > SQUARE_FROM_DEG {
                            squaring = !walks && off.abs() > SQUARE_TO_DEG;
                            let step = off.clamp(-TURN_RATE * dt, TURN_RATE * dt);
                            t.facing = wrap(t.facing + step);
                        }
                    } else {
                        squaring = false;
                    }
                    let speed = vz.max(0.0);
                    let left = gap - stand - speed * LAG_S;
                    // Something in the way, nearer than they are?
                    let facing = t.facing;
                    if t.obstacle.and_then(|o| o.ahead(pos, facing)).is_some_and(|d| d < -0.3) {
                        t.obstacle = None; // over it, past it
                    }
                    let ob = t.obstacle.and_then(|o| o.ahead(pos, facing).map(|d| (o, d))).filter(|&(_, d)| d < gap - 0.3);
                    // In the air after a jump at it, the run goes on over it.
                    let airborne = last_jump.is_some_and(|j| now - j < JUMP_CARRY);
                    let over = ob.filter(|(o, d)| o.jump && *d < 2.0 && (airborne || last_jump.is_none_or(|j| now - j > JUMP_EVERY)));
                    let blocked = ob.is_some_and(|(_, d)| d < OBSTACLE_M) && over.is_none();
                    if let Some((o, d)) = over.filter(|_| !airborne) {
                        jump = d < JUMP_AT_M + speed * 0.1;
                        if jump {
                            t.jumped = Some((now, o.place()));
                        }
                    }
                    let start = if axis > 0.0 { 0.05 } else { WALK_MARGIN };
                    if gap - stand > start && !blocked {
                        let mut v = (2.0 * BRAKE * left.max(0.0)).sqrt().min(MAX_SPEED);
                        if over.is_some() {
                            v = v.max(RUN_UP_SPEED); // a run-up
                        }
                        if walling {
                            v = v.clamp(MIN_SPEED * 2.0, WALL_SPEED); // along it, out of reach or not
                        }
                        if v >= MIN_SPEED { AXIS_DEAD + v / SPEED_PER_AXIS } else { 0.0 }
                    } else if gap < stand - BACK_MARGIN || (axis < 0.0 && gap < stand - BACK_UNTIL) {
                        BACK_AXIS
                    } else {
                        0.0
                    }
                }
                _ => 0.0,
            };
            // A pane on the map just ahead (glass): turn, but do not push on;
            // the next look finds the way round.
            let want = if want > 0.0 && pane_ahead(bridge, t.facing, PANE_AHEAD_M, false) {
                if let Some(mut s) = self.inner_for(stop) {
                    s.avoiding = "pane";
                }
                0.0
            } else {
                want
            };
            // Stuck: pushing, not moving. Jump once; then back off and turn aside.
            if axis > STUCK_AXIS {
                let pushed = *pushed_since.get_or_insert(now);
                // The way it moves (the body may lag the head: sideways too).
                if now - pushed > Duration::from_millis(500) && vx.hypot(vz) < STUCK_SPEED {
                    if now - *slow_since.get_or_insert(now) > STUCK_FOR {
                        slow_since = None;
                        tracing::info!(facing = t.facing.round(), wall = t.wall.is_some(), "follow: stuck");
                        bridge.mapping.stopped(t.facing);
                        if last_jump.is_none_or(|j| now - j > Duration::from_secs(3)) {
                            jump = true;
                        } else {
                            escape = Some(now + ESCAPE_FOR);
                            aside = -aside;
                            let yaw = wrap(t.facing + aside * 60.0);
                            t.detour = Some((now + ESCAPE_FOR + DETOUR_FOR, yaw));
                            if let Some(mut s) = self.inner_for(stop) {
                                s.avoiding = "stuck";
                            }
                        }
                    }
                } else {
                    slow_since = None;
                }
            } else {
                pushed_since = None;
                slow_since = None;
            }
            let facing = t.facing;
            // The head: at them (their face, under the tag), within reach of
            // the body's facing; along a wall or round something, the way it
            // walks.
            if !fresh {
                (head_yaw, head_pitch, smooth_pitch) = (f32::NAN, PITCH, PITCH);
            }
            let want_pitch = t.tag_pitch().filter(|_| fresh).unwrap_or(PITCH);
            smooth_pitch = smooth_pitch_step(smooth_pitch, want_pitch, dt);
            let want_yaw = match t.target_now() {
                Some(goal) if fresh && !walling && !t.detour.is_some_and(|d| now < d.0) => {
                    let gap = (goal[0] - pos[0]).hypot(goal[1] - pos[1]);
                    if gap > 0.3 { wrap(facing + angle_diff(bearing(pos, goal), facing).clamp(-HEAD_MAX_DEG, HEAD_MAX_DEG)) } else { facing }
                }
                _ => facing,
            };
            drop(t);
            if head_yaw.is_nan() {
                head_yaw = facing;
            }
            let reach = HEAD_RATE * dt;
            head_yaw = wrap(head_yaw + angle_diff(want_yaw, head_yaw).clamp(-reach, reach));
            // Never further off the body than the head turns.
            head_yaw = wrap(facing + angle_diff(head_yaw, facing).clamp(-HEAD_MAX_DEG, HEAD_MAX_DEG));
            if (smooth_pitch - head_pitch).abs() > PITCH_DEADBAND {
                head_pitch += (smooth_pitch - head_pitch).clamp(-reach, reach);
            }
            if (head_pitch - shown_pitch).abs() >= 0.5 || shown_pitch.is_nan() {
                shown_pitch = head_pitch;
                if let Some(mut s) = self.inner_for(stop) {
                    s.head_pitch = head_pitch;
                }
            }
            // Walking goes where the head looks: split the stick so the walk
            // keeps to the facing while a glance turns the head.
            let off = heading.map_or(0.0, |h| angle_diff(facing, h));
            let (ahead_axis, aside_axis) = if want > 0.0 {
                if off.abs() > 100.0 {
                    (0.0, 0.0)
                } else {
                    let (s, c) = off.to_radians().sin_cos();
                    (want * c, want * s)
                }
            } else {
                (want, 0.0)
            };
            if (ahead_axis - axis).abs() > 0.02 || (ahead_axis == 0.0 && axis != 0.0) {
                self.set_axis(bridge, &mut axis, ahead_axis.clamp(-1.0, 1.0));
            }
            if (aside_axis - strafe).abs() > 0.03 || (aside_axis == 0.0 && strafe != 0.0) {
                strafe = aside_axis.clamp(-1.0, 1.0);
                let _ = bridge.osc.send_f32("/input/Horizontal", strafe);
            }
            if jump {
                last_jump = Some(now);
                let _ = bridge.osc.send_i32("/input/Jump", 1);
                let b = bridge.clone();
                std::thread::spawn(move || {
                    std::thread::sleep(Duration::from_millis(100));
                    let _ = b.osc.send_i32("/input/Jump", 0);
                });
                if let Some(mut s) = self.inner_for(stop) {
                    s.jumps += 1;
                }
            }
            let now_aim = [facing, head_yaw, head_pitch];
            let moved = aimed.iter().zip(now_aim).any(|(a, b)| a.is_nan() || angle_diff(b, *a).abs() >= 1.0);
            if fresh {
                // The eyes may hold the headset a moment: next tick then.
                if let Some(mut vr) = bridge.vr.try_lk() {
                    // Something else turned the head meanwhile (a glance): again.
                    let elsewhere = angle_diff(vr.yaw, aimed[1]).abs() >= 1.0 || (vr.pitch - aimed[2]).abs() >= 1.0;
                    if (moved || elsewhere) && vr.face_and_look(facing, head_yaw, head_pitch).is_ok() {
                        aimed = now_aim;
                    }
                }
            }
        }
        self.set_axis(bridge, &mut axis, 0.0);
        let _ = bridge.osc.send_f32("/input/Horizontal", 0.0);
    }

    fn set_axis(&self, bridge: &Arc<Bridge>, axis: &mut f32, value: f32) {
        if *axis != value {
            let _ = bridge.osc.send_f32("/input/Vertical", value);
            *axis = value;
            self.inner.lk().moving = (value * 100.0).round() / 100.0;
        }
    }
}

/// The end of a follow (see `Follower::run`).
struct Done {
    me: Arc<Follower>,
    bridge: Arc<Bridge>,
    stop: Arc<AtomicBool>,
    legs: Option<std::thread::JoinHandle<()>>,
}

impl Drop for Done {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(legs) = self.legs.take() {
            let _ = legs.join();
        }
        // The legs let go when they end; a panic in them may not have.
        for axis in ["/input/Vertical", "/input/Horizontal"] {
            let _ = self.bridge.osc.send_f32(axis, 0.0);
        }
        // The front lens faces the heading again (a follow after this one
        // turns it back on when it runs).
        self.bridge.orbit.set_following(false);
        let current = Arc::ptr_eq(&self.me.stop.lk(), &self.stop);
        if current {
            let mut s = self.me.inner.lk();
            s.running = false;
            s.state = "idle";
            s.moving = 0.0;
        }
        self.bridge.notify_state();
    }
}

/// The head's pitch (degrees) a tick of `dt` seconds on from `pitch`
/// toward `want` (at them, clamped): settling over PITCH_SMOOTH_S.
fn smooth_pitch_step(pitch: f32, want: f32, dt: f32) -> f32 {
    let want = want.clamp(HEAD_PITCH_MIN, HEAD_PITCH_MAX);
    pitch + (want - pitch) * (1.0 - (-dt.max(0.0) / PITCH_SMOOTH_S).exp())
}

/// The search's stages (the panorama, decisions D37, D41), in order: the
/// lens's quick sweep, the bot turning to their name and where they were;
/// from the second search on (`rounds`), the bot round in views.
fn search_stages(rounds: u32) -> Vec<&'static str> {
    let mut stages = vec!["lens_ring", "body_turn"];
    if rounds >= 2 {
        stages.push("scan");
    }
    stages
}


/// The lens's quick sweep (`Orbit::sweep`), the bot standing, from `from`
/// (a world yaw: where they were) out either way: the target's name if
/// it was read (the sweep ends there and the lens turns to them). Waits
/// for the sweep and its reads in flight; holds nothing.
fn sweep_for(bridge: &Bridge, target: &str, from: Option<f32>, stop: &AtomicBool) -> Option<NameSighting> {
    let t0 = Instant::now();
    // The legs stop as the follower does: a moment for them to let go.
    loop {
        match bridge.orbit.sweep("follow", from, Some(target)) {
            Ok(()) => break,
            Err(e) if t0.elapsed() < SWEEP_ASK_FOR && !stop.load(Ordering::SeqCst) => {
                tracing::trace!("follow: no lens sweep yet: {e:#}");
                std::thread::sleep(Duration::from_millis(30));
            }
            Err(e) => {
                tracing::info!("follow: no lens sweep: {e:#}");
                return None;
            }
        }
    }
    let theirs = || bridge.orbit.names_since(t0).into_iter().rev().find(|n| match_score(&n.name, target) >= 0.6);
    let deadline = Instant::now() + bridge.orbit.sweep_budget() + Duration::from_millis(500);
    while bridge.orbit.sweeping() && Instant::now() < deadline {
        if stop.load(Ordering::SeqCst) {
            bridge.orbit.end_sweep("stopped");
            return None;
        }
        // (The lens is turned to them before their name is kept.)
        if let Some(n) = theirs() {
            return Some(n);
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    // Over (every view taken, or found): the reads still in flight.
    let reads = Instant::now() + SWEEP_READS_WAIT;
    loop {
        if let Some(n) = theirs() {
            return Some(n);
        }
        if bridge.orbit.reading() == 0 || Instant::now() > reads || stop.load(Ordering::SeqCst) {
            bridge.orbit.end_sweep("stopped");
            return None;
        }
        std::thread::sleep(Duration::from_millis(30));
    }
}

/// A forward view's width (degrees): the friends' sightings show that.
const FORWARD_FOV: f32 = 100.0;

/// The OCR client (the plates over the eyes), from the headset's rig when
/// it is free, else made.
fn ocr_client(bridge: &Bridge) -> Option<OcrClient> {
    if let Some(mut vr) = bridge.vr.try_lk() {
        if let Some(o) = vr.rig(&[]).ok().and_then(|r| r.ocr.clone()) {
            return Some(o);
        }
    }
    OcrClient::new(&bridge.args.ocr_url, &bridge.args.ocr_model).ok()
}

/// The yaw (tracking, degrees) from `eye` to `p`.
fn bearing_of(eye: [f32; 3], p: [f32; 3]) -> f32 {
    bearing([eye[0], eye[2]], [p[0], p[2]])
}

/// The target among a panorama's people (name, how it came, body):
/// someone their name was read on; else the person-shaped one nearest
/// where they were last placed (`last`: world feet, when), within
/// KEEP_GATE_M plus KEEP_SPEED a second since, for KEEP_FOR, and not named
/// as someone else (the caller gives `last` only while a name said it was
/// them `kept_s` ago at most). The index and how: "named" (the plates
/// over the eyes), "lens" (the lens's read), or "kept".
fn pick(people: &[(Option<String>, Option<&'static str>, vrc_pano::Body)], target: &str, last: Option<([f32; 3], Instant)>, now: Instant) -> Option<(usize, &'static str)> {
    let named = people
        .iter()
        .enumerate()
        .filter(|(_, (n, _, _))| n.as_deref().is_some_and(|n| match_score(n, target) >= 0.6))
        .min_by(|a, b| a.1 .2.distance.total_cmp(&b.1 .2.distance));
    if let Some((k, (_, how, _))) = named {
        return Some((k, if matches!(how, Some("lens" | "asked" | "sweep")) { "lens" } else { "named" }));
    }
    let (at, when) = last?;
    let since = now.saturating_duration_since(when);
    if since > KEEP_FOR {
        return None;
    }
    let gate = KEEP_GATE_M + KEEP_SPEED * since.as_secs_f32();
    people
        .iter()
        .enumerate()
        .filter(|(_, (n, _, _))| n.is_none())
        .map(|(k, (_, _, b))| (k, (b.feet[0] - at[0]).hypot(b.feet[2] - at[2])))
        .filter(|(_, d)| *d <= gate)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(k, _)| (k, "kept"))
}

/// The target placed by a look (`Track::place`): when (a lens read's
/// time), how ("named", "lens", "kept"), the world yaw from the head, how
/// far (world metres), how far up.
#[derive(Clone, Copy, Debug)]
struct Placed {
    when: Instant,
    how: &'static str,
    world_yaw: f32,
    gap: f32,
    up: f32,
}

/// Where a lens read of their name puts them, when this look found nobody
/// under its ray (decision D40).
#[derive(Clone, Copy, Debug, PartialEq)]
enum LensPlace {
    /// A body of this look (an index into its people) is them.
    Body(usize),
    /// No body: their feet (world), their plate's height (world y), as of
    /// the read.
    Point { feet: [f32; 3], plate_y: f32, at: Instant },
}

/// A lens read with no depth under it is taken at least this far (metres).
const BEARING_ONLY_MIN_GAP_M: f32 = 2.5;

/// Where the lens's read `n` puts them among this look's `people` (name,
/// how, body), the head per `tr`, `gap` the distance the follower last had
/// (world metres): where the depth placed the read (`feet`), the body
/// standing there if any; else along its bearing, a body near it (within
/// LENS_BODY_DEG, and LENS_BODY_M of the distance known) or the last
/// distance. The plate's height from the read's ray (its elevation) at
/// that distance: their head's pitch follows it.
fn lens_place(n: &NameSighting, people: &[(Option<String>, Option<&'static str>, vrc_pano::Body)], tr: &vrc_pano::Tracking, gap: Option<f32>) -> LensPlace {
    let head = tr.head_world;
    let free = |q: &(Option<String>, Option<&'static str>, vrc_pano::Body)| q.0.as_deref().is_none_or(|name| name == n.name);
    if let Some(f) = n.feet {
        if let Some((k, _)) = people
            .iter()
            .enumerate()
            .filter(|(_, q)| free(q))
            .map(|(k, q)| (k, (q.2.feet[0] - f[0]).hypot(q.2.feet[2] - f[2])))
            .filter(|(_, d)| *d < 0.6)
            .min_by(|a, b| a.1.total_cmp(&b.1))
        {
            return LensPlace::Body(k);
        }
    }
    // With no depth under the plate, the last gap is a guess that may be
    // long stale (seen live: 1.1 m kept while they stood 5 m away, below a
    // platform; the bot thought itself there and stood still): never nearer
    // than BEARING_ONLY_MIN_GAP_M, so it goes their way till they are placed.
    let distance = n.distance_m.or(gap.map(|g| g.max(BEARING_ONLY_MIN_GAP_M))).unwrap_or(DEFAULT_GAP_M);
    let along = people
        .iter()
        .enumerate()
        .filter(|(_, q)| free(q))
        .filter_map(|(k, q)| {
            let yaw = q.2.yaw_from(head);
            let d = (q.2.feet[0] - head[0]).hypot(q.2.feet[2] - head[2]);
            let off = angle_diff(yaw, n.world_yaw).abs();
            (off <= LENS_BODY_DEG && (d - distance).abs() <= LENS_BODY_M).then_some((k, off))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1));
    if n.feet.is_none() {
        if let Some((k, _)) = along {
            return LensPlace::Body(k);
        }
    }
    let feet = n.feet.unwrap_or_else(|| {
        let (s, c) = n.world_yaw.to_radians().sin_cos();
        [head[0] + distance * s, head[1] - 1.6, head[2] + distance * c]
    });
    // The plate on the read's ray, as far out as their feet.
    let h = (feet[0] - n.ray_from[0]).hypot(feet[2] - n.ray_from[2]);
    let flat = n.ray_dir[0].hypot(n.ray_dir[2]).max(1e-3);
    let plate_y = n.ray_from[1] + n.ray_dir[1] / flat * h;
    LensPlace::Point { feet, plate_y, at: n.at }
}

/// Where a lost target is looked for (tracking yaws), in order: where
/// their name was last read, then where they were last placed (one way
/// when within MERGE_DEG of each other).
fn lost_ways(named: Option<f32>, last: Option<f32>) -> Vec<(f32, &'static str)> {
    let mut ways = Vec::new();
    if let Some(n) = named {
        ways.push((wrap(n), "their name"));
    }
    if let Some(l) = last {
        if ways.iter().all(|(w, _)| angle_diff(l, *w).abs() > MERGE_DEG) {
            ways.push((wrap(l), "where they were"));
        }
    }
    ways
}

/// What the corridor along `yaw` holds, for tuning (`/v1/vr/corridor`):
/// per 10 cm from 0.3 m out, the points' count and lowest and highest
/// height over the floor (world metres), and what `corridor` makes of it.
/// From the pano depth's `points` (tracking space), the head at `eye`, the
/// floor at `floor`.
pub fn corridor_report(points: &[[f32; 3]], eye: [f32; 3], yaw: f32, metres: f32, floor: f32) -> Value {
    let (s, c) = yaw.to_radians().sin_cos();
    let mut bins: Vec<(usize, f32, f32)> = vec![(0, f32::INFINITY, f32::NEG_INFINITY); 40];
    for p in points {
        let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
        let (ahead, side, up) = ((dx * s - dz * c) * metres, (dx * c + dz * s) * metres, (p[1] - floor) * metres);
        if ahead > 0.3 && side.abs() < 0.25 {
            let i = ((ahead - 0.3) / 0.1) as usize;
            if let Some(b) = bins.get_mut(i) {
                (b.0, b.1, b.2) = (b.0 + 1, b.1.min(up), b.2.max(up));
            }
        }
    }
    let r = |v: f32| (v as f64 * 100.0).round() / 100.0;
    json!({
        "yaw": yaw.round(),
        "eye_m": r((eye[1] - floor) * metres),
        "points": points.len(),
        "blocker": corridor(points, eye, yaw, metres, floor, FROM_ANYWAY).map(|b| json!({"distance": r(b.distance), "top": r(b.top), "tall": b.tall})),
        "bins": bins.iter().enumerate().filter(|(_, b)| b.0 > 0).map(|(i, b)| json!([r(0.3 + 0.1 * i as f32), b.0, r(b.1), r(b.2)])).collect::<Vec<_>>(),
    })
}

/// The yaw (degrees, + right of -z) from `from` to `to`.
fn bearing(from: [f32; 2], to: [f32; 2]) -> f32 {
    (to[0] - from[0]).atan2(-(to[1] - from[1])).to_degrees()
}

fn wrap(deg: f32) -> f32 {
    (deg + 540.0).rem_euclid(360.0) - 180.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odometry_interpolates() {
        let t0 = Instant::now();
        let mut t = Track::default();
        t.history.push_back((t0, [0.0, 0.0]));
        t.history.push_back((t0 + Duration::from_millis(100), [1.0, -2.0]));
        t.pos = [1.0, -2.0];
        let mid = t.pos_at(t0 + Duration::from_millis(50));
        assert!((mid[0] - 0.5).abs() < 1e-4 && (mid[1] + 1.0).abs() < 1e-4);
        assert_eq!(t.pos_at(t0 + Duration::from_secs(1)), [1.0, -2.0]);
    }

    #[test]
    fn fixes_estimate_motion() {
        let t0 = Instant::now() - Duration::from_secs(1);
        let mut t = Track::default();
        t.add_fix(t0, [0.0, -3.0], 0.0, 0.5);
        t.add_fix(t0 + Duration::from_millis(500), [0.0, -3.5], 0.0, 0.5);
        let v = t.target.unwrap().vel;
        // Half of 1 m/s (smoothed from rest), away along -z.
        assert!((v[1] + 0.5).abs() < 1e-4, "{v:?}");
        // Jitter of a few centimetres is standing still.
        t.add_fix(t0 + Duration::from_millis(900), [0.02, -3.5], 0.0, 0.5);
        assert!(t.target.unwrap().vel[0].abs() < 0.3);
    }

    #[test]
    fn a_ledge_lower_than_the_eyes_is_jumped_however_tall_the_avatar() {
        // A short avatar (eyes 1.17 m) before a 1 m ledge 1.5 m ahead.
        let eye = [0.0, 1.17, 0.0];
        let mut pts = Vec::new();
        for i in 0..30 {
            for z in -4..=4 {
                let x = 1.5 + i as f32 * 0.05;
                for y in 0..=10 {
                    if i == 0 || y == 10 {
                        pts.push([x, y as f32 * 0.1, z as f32 * 0.05]);
                    }
                }
            }
        }
        let b = corridor(&pts, eye, 90.0, 1.0, 0.0, FROM_ANYWAY).expect("the ledge");
        assert!(!b.tall, "{b:?}");
        // Pressed up to it (0.7 m): it is across the way, so seen.
        let near: Vec<[f32; 3]> = pts.iter().map(|p| [p[0] - 0.8, p[1], p[2]]).collect();
        let b = corridor(&near, eye, 90.0, 1.0, 0.0, FROM_ANYWAY).expect("the ledge, near");
        assert!(!b.tall && (b.distance - 0.7).abs() < 0.05, "{b:?}");
    }

    #[test]
    fn corridors_find_and_measure_obstacles() {
        let eye = [0.0, 1.6, 0.0];
        let floor_to = |points: &mut Vec<[f32; 3]>, from: f32, to: f32, y: f32| {
            let mut z = from;
            while z < to {
                for i in 0..5 {
                    points.push([-0.2 + 0.1 * i as f32, y, -z]);
                }
                z += 0.05;
            }
        };
        // Floor, then a box 0.4 m high from 1.3 m ahead (-z).
        let mut points = Vec::new();
        floor_to(&mut points, 0.4, 1.3, 0.0);
        // Its face, as densely as stereo samples it (2 cm).
        for j in 0..21 {
            for i in 0..20 {
                points.push([-0.2 + 0.02 * i as f32, 0.02 * j as f32, -1.3]);
            }
        }
        let b = corridor(&points, eye, 0.0, 1.0, 0.0, FROM_ANYWAY).unwrap();
        assert!((b.distance - 1.3).abs() < 0.05 && (b.top - 0.4).abs() < 0.05 && !b.tall, "{b:?}");
        // A wall up past the eyes, 1.5 m to the right: tall.
        let mut wall = Vec::new();
        floor_to(&mut wall, 0.4, 1.5, 0.0);
        let wall: Vec<[f32; 3]> = wall.iter().map(|p| [-p[2], p[1], p[0]]).collect::<Vec<_>>();
        let mut wall = wall;
        for j in 0..40 {
            for i in 0..5 {
                wall.push([1.5, 0.05 * j as f32, -0.2 + 0.1 * i as f32]);
            }
        }
        let b = corridor(&wall, eye, 90.0, 1.0, 0.0, FROM_ANYWAY).unwrap();
        assert!((b.distance - 1.5).abs() < 0.05 && b.tall, "{b:?}");
        assert!(corridor(&wall, eye, -90.0, 1.0, 0.0, FROM_ANYWAY).is_none());
        // A ramp rising 0.15 m per 0.3 m: followed, not in the way.
        let mut ramp = Vec::new();
        let mut z = 0.4;
        while z < 3.0 {
            for i in 0..5 {
                ramp.push([-0.2 + 0.1 * i as f32, (z - 0.4) * 0.5, -z]);
            }
            z += 0.05;
        }
        assert!(corridor(&ramp, eye, 0.0, 1.0, 0.0, FROM_ANYWAY).is_none());
        // Stairs from 1 m out (0.18 m risers, 0.25 m treads, up to 2.5 m
        // high), seen from a short avatar: climbed, not in the way; a wall
        // behind the landing (up past the eyes over it) is.
        let eye_short = [0.0, 1.18, 0.0];
        let mut stairs = Vec::new();
        floor_to(&mut stairs, 0.4, 1.0, 0.0);
        for k in 0..14 {
            let (z0, y) = (1.0 + 0.25 * k as f32, 0.18 * (k + 1) as f32);
            for j in 0..9 {
                for i in 0..5 {
                    stairs.push([-0.2 + 0.1 * i as f32, y - 0.18 + 0.02 * j as f32, -z0]);
                }
            }
            floor_to(&mut stairs, z0, z0 + 0.25, y);
        }
        assert!(corridor(&stairs, eye_short, 0.0, 1.0, 0.0, FROM_ANYWAY).is_none());
        let mut landing = Vec::new();
        floor_to(&mut landing, 0.4, 1.0, 0.0);
        for k in 0..4 {
            let (z0, y) = (1.0 + 0.25 * k as f32, 0.18 * (k + 1) as f32);
            for j in 0..9 {
                for i in 0..5 {
                    landing.push([-0.2 + 0.1 * i as f32, y - 0.18 + 0.02 * j as f32, -z0]);
                }
            }
            floor_to(&mut landing, z0, z0 + 0.25, y);
        }
        floor_to(&mut landing, 2.0, 2.5, 0.72);
        for j in 0..40 {
            for i in 0..5 {
                landing.push([-0.2 + 0.1 * i as f32, 0.72 + 0.05 * j as f32, -2.5]);
            }
        }
        let b = corridor(&landing, eye_short, 0.0, 1.0, 0.0, FROM_ANYWAY).expect("the wall over the landing");
        assert!((b.distance - 2.5).abs() < 0.05 && b.tall, "{b:?}");
        // Near: a prop at the bot's side at 0.6 m (up to 1.1 m high) is not
        // in the way; a ledge across it is; a wall there (up past the
        // eyes) is.
        let mut prop = Vec::new();
        for j in 0..10 {
            for i in 0..5 {
                prop.push([0.12 + 0.03 * i as f32, 0.6 + 0.05 * j as f32, -0.6]);
            }
        }
        assert!(corridor(&prop, eye, 0.0, 1.0, 0.0, FROM_ANYWAY).is_none());
        let mut ledge = Vec::new();
        for j in 0..12 {
            for i in 0..9 {
                ledge.push([-0.2 + 0.05 * i as f32, 0.05 * j as f32, -0.7]);
            }
        }
        let b = corridor(&ledge, eye, 0.0, 1.0, 0.0, FROM_ANYWAY).expect("a ledge across the way");
        assert!((b.distance - 0.7).abs() < 0.05 && !b.tall, "{b:?}");
        for j in 0..38 {
            for i in 0..10 {
                prop.push([-0.2 + 0.05 * i as f32, 0.05 * j as f32, -0.7]);
            }
        }
        let b = corridor(&prop, eye, 0.0, 1.0, 0.0, FROM_ANYWAY).unwrap();
        assert!((b.distance - 0.7).abs() < 0.05 && b.tall, "{b:?}");
        // Walking up to an obstacle along its heading brings it nearer; another way, it is not ahead.
        let o = Obstacle { yaw: 0.0, then: [0.0, 0.0], distance: 1.0, jump: true };
        assert!((o.ahead([0.0, -0.4], 0.0).unwrap() - 0.6).abs() < 1e-4);
        assert!(o.ahead([0.0, 0.0], 60.0).is_none());
        assert_eq!(o.place(), [0.0, -1.0]);
    }

    #[test]
    fn the_search_turns_the_way_they_went() {
        let t0 = Instant::now() - Duration::from_secs(1);
        let mut t = Track::default();
        // Bot at the origin facing -z; they stood 3 m ahead, then walked
        // off to the bot's right (+x).
        t.add_fix(t0, [0.0, -3.0], 0.0, 0.5);
        t.add_fix(t0 + Duration::from_millis(400), [0.6, -3.0], 0.0, 0.5);
        t.add_fix(t0 + Duration::from_millis(800), [1.2, -3.0], 0.0, 0.5);
        assert_eq!(t.search_way(0.0), 1.0);
        // To the left: round to the left.
        let mut l = Track::default();
        l.add_fix(t0, [0.0, -3.0], 0.0, 0.5);
        l.add_fix(t0 + Duration::from_millis(400), [-0.6, -3.0], 0.0, 0.5);
        l.add_fix(t0 + Duration::from_millis(800), [-1.2, -3.0], 0.0, 0.5);
        assert_eq!(l.search_way(0.0), -1.0);
        // Never moving, last seen to the left of where the bot faces.
        let mut s = Track { facing: 30.0, ..Default::default() };
        s.add_fix(t0, [0.0, -3.0], 0.0, 0.5);
        assert_eq!(s.search_way(0.0), -1.0);
    }

    fn body(x: f32, z: f32) -> vrc_pano::Body {
        vrc_pano::Body { feet: [x, 0.0, z], top: 1.6, low: 0.25, distance: x.hypot(z), points: 100 }
    }

    #[test]
    fn the_target_is_named_or_kept_by_position() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let ann = Some("Ann".to_string());
        // Named: theirs, wherever.
        let people = vec![(None, None, body(1.0, 1.0)), (ann.clone(), Some("overlay"), body(3.0, 0.0))];
        assert_eq!(pick(&people, "Ann", None, at(0)), Some((1, "named")));
        // Named by the lens: seen by the lens.
        let by_lens = vec![(None, None, body(1.0, 1.0)), (ann.clone(), Some("lens"), body(3.0, 0.0))];
        assert_eq!(pick(&by_lens, "Ann", None, at(0)), Some((1, "lens")));
        // No name read: the one nearest where they were (0.3 m on), not the
        // other (2 m off), nor the one named Bob though nearer still.
        let last = Some(([3.0, 0.0, 0.0], at(0)));
        let people = vec![(None, None, body(3.3, 0.0)), (None, None, body(5.0, 0.0)), (Some("Bob".to_string()), Some("overlay"), body(3.1, 0.0))];
        assert_eq!(pick(&people, "Ann", last, at(300)), Some((0, "kept")));
        // Walked 1.5 m in a second: within the gate then, not at once.
        let far = vec![(None, None, body(4.5, 0.0))];
        assert_eq!(pick(&far, "Ann", last, at(1000)), Some((0, "kept")));
        assert_eq!(pick(&far, "Ann", last, at(100)), None);
        // Kept too long without a name or a fix: not theirs any more.
        assert_eq!(pick(&people, "Ann", last, at(5000)), None);
        // A name read on someone else's place wins over the position.
        let both = vec![(None, None, body(3.1, 0.0)), (ann, Some("overlay"), body(1.0, 2.0))];
        assert_eq!(pick(&both, "Ann", last, at(300)), Some((1, "named")));
    }

    #[test]
    fn lost_turns_to_their_name_then_where_they_were() {
        assert_eq!(lost_ways(Some(100.0), Some(-30.0)), vec![(100.0, "their name"), (-30.0, "where they were")]);
        // The same way: one turn.
        assert_eq!(lost_ways(Some(100.0), Some(110.0)), vec![(100.0, "their name")]);
        assert_eq!(lost_ways(None, Some(190.0)), vec![(-170.0, "where they were")]);
        assert!(lost_ways(None, None).is_empty());
    }

    #[test]
    fn following_through_pano_frames_named_then_kept() {
        use crate::panolook::tests::{eyes_of, out};
        use crate::panolook::{name_people, PlateRay};
        use vrc_pano::synth::{person, Scene};
        use vrc_pano::{decode, Cloud, PanoParams, PeopleParams};
        let p = PeopleParams::default();
        let t0 = Instant::now();
        // Frame 1: Ann 2 m ahead of the head, read by the lens; Bob 3 m to
        // the right, nobody's name read on him.
        let mut s = Scene::room();
        let y = s.room_min[1];
        let (ax, az) = out(&s, s.head_yaw, 2.0);
        let (bx, bz) = out(&s, s.head_yaw + 90.0, 3.0);
        s.boxes = vec![person(&s, ax, az, 1.6), person(&s, bx, bz, 1.7)];
        let frame = decode(&eyes_of(&s), &PanoParams::default()).unwrap();
        let cloud = Cloud::new(&frame, 4);
        let head = frame.head.position;
        let d = [ax - head[0], y + 2.0 - head[1], az - head[2]];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let ray = PlateRay { name: "Ann".into(), from: head, dir: d.map(|c| c / n), at: t0, source: "lens", bbox: None, head_yaw: s.head_yaw };
        let (people, _) = name_people(&cloud, &cloud.bodies(&p), &[ray], &p);
        let people: Vec<_> = people.into_iter().map(|q| (q.name, q.how, q.body)).collect();
        let (k, how) = pick(&people, "Ann", None, t0).unwrap();
        assert_eq!(how, "lens");
        let first = people[k].2;
        assert!((first.feet[0] - ax).hypot(first.feet[2] - az) < 0.25);
        // Frame 2, half a second on: Ann stepped 0.4 m aside; no name read.
        let (ax2, az2) = out(&s, s.head_yaw + 12.0, 2.0);
        s.boxes = vec![person(&s, ax2, az2, 1.6), person(&s, bx, bz, 1.7)];
        let frame = decode(&eyes_of(&s), &PanoParams::default()).unwrap();
        let cloud = Cloud::new(&frame, 4);
        let (people, _) = name_people(&cloud, &cloud.bodies(&p), &[], &p);
        let people: Vec<_> = people.into_iter().map(|q| (q.name, q.how, q.body)).collect();
        let then = t0 + Duration::from_millis(500);
        let (k, how) = pick(&people, "Ann", Some((first.feet, t0)), then).expect("kept by position");
        assert_eq!(how, "kept");
        assert!((people[k].2.feet[0] - ax2).hypot(people[k].2.feet[2] - az2) < 0.25, "{:?}", people[k]);
        // The fix it makes: in the follow's frame, 2 m out the way the head
        // looks, rising as their plate does.
        let tr = vrc_pano::Tracking::new(&frame, &eyes_of(&s), cloud.floor, FLOOR_Y);
        let f = tr.point(people[k].2.feet);
        let rel = [(f[0] - tr.head_track[0]) * tr.metres, (f[2] - tr.head_track[2]) * tr.metres];
        assert!((rel[0].hypot(rel[1]) - 2.0).abs() < 0.2 && (bearing([0.0, 0.0], rel) - 12.0).abs() < 5.0, "{rel:?}");
    }

    #[test]
    fn the_watch_over_going_round_replans_relocates_and_rests() {
        use AvoidAct::*;
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut a = Avoid::default();
        let wall = Some("wall");
        // Not going round: nothing.
        assert_eq!(a.tick(at(0), None, [0.0, -4.0], 4.0, Some(at(0))), Go);
        assert!(a.since.is_none());
        // Going round: the episode starts, planned for where they are.
        assert_eq!(a.tick(at(100), wall, [0.0, -4.0], 4.0, Some(at(100))), Go);
        assert_eq!(a.since, Some(at(100)));
        // They walked a metre: planned again at once, for there.
        assert_eq!(a.tick(at(300), wall, [1.0, -4.0], 4.1, Some(at(300))), Replan);
        assert_eq!(a.planned.unwrap().0, [1.0, -4.0]);
        // Standing: again every REPLAN_EVERY.
        assert_eq!(a.tick(at(1000), wall, [1.0, -4.0], 4.1, Some(at(1000))), Go);
        assert_eq!(a.tick(at(1900), wall, [1.0, -4.0], 4.1, Some(at(1900))), Replan);
        assert_eq!(a.replans, 2);
        // A break shorter than AVOID_GRACE is the same episode.
        assert_eq!(a.tick(at(2000), None, [1.0, -4.0], 4.1, Some(at(2000))), Go);
        a.tick(at(2600), Some("map"), [1.0, -4.0], 4.1, Some(at(2600)));
        assert_eq!((a.since, a.why), (Some(at(100)), "map"));
        // Getting nearer 0.7 m every 10 s: never stalled, but after
        // AVOID_FOR the episode is re-located ("timeout").
        let mut first = None;
        for ms in (2800..40_000).step_by(200) {
            let gap = 4.0 - 0.7 * ((ms - 2800) / 10_000) as f32;
            match a.tick(at(ms), wall, [1.0, -4.0], gap, Some(at(ms))) {
                Relocate(why) => {
                    first = Some((ms, why));
                    break;
                }
                Go | Replan => {}
                other => panic!("{other:?} at {ms}"),
            }
        }
        let (ms, why) = first.expect("re-located");
        assert_eq!(why, "timeout");
        let age = Duration::from_millis(ms - 100);
        assert!(age > AVOID_FOR && age <= AVOID_FOR + Duration::from_millis(400), "{ms}");
        assert!(a.since.is_none() && a.relocates == 1 && a.in_row == 1);
        // Pinned (no nearer at all): stalled after AVOID_STALL.
        let mut t = ms;
        let mut acts = Vec::new();
        while acts.len() < 2 {
            t += 200;
            match a.tick(at(t), wall, [1.0, -4.0], 4.0, Some(at(t))) {
                Relocate(why) => acts.push((t, why, false)),
                Rest => acts.push((t, "rest", true)),
                _ => {}
            }
        }
        assert_eq!(acts[0].1, "stalled");
        // The third in a row: the bot rests, nothing else meanwhile.
        assert!(acts[1].2 && a.in_row == AVOID_ROUNDS, "{acts:?}");
        let rest_from = acts[1].0;
        for k in 1..(REST_FOR.as_millis() as u64 / 200) {
            let ms = rest_from + 200 * k;
            assert_eq!(a.tick(at(ms), wall, [1.0, -4.0], 4.0, Some(at(ms))), Rest);
        }
        // Rested: the count starts again, a fresh episode.
        let after = rest_from + REST_FOR.as_millis() as u64 + 200;
        assert_eq!(a.tick(at(after), wall, [1.0, -4.0], 4.0, Some(at(after))), Go);
        assert!(a.in_row == 0 && a.since == Some(at(after)));
        // Not seen going round for AVOID_UNSEEN: lost (no rest for that).
        let seen = at(after);
        let mut lost = None;
        for k in 1..60 {
            let ms = after + 200 * k;
            if a.tick(at(ms), wall, [1.0, -4.0], 4.0, Some(seen)) == Lost {
                lost = Some(ms);
                break;
            }
        }
        let lost = lost.expect("lost");
        assert!(Duration::from_millis(lost - after) > AVOID_UNSEEN && a.last_relocate == "lost" && a.rest_until.is_none());
        // Walking free AVOID_CLEAR: the re-locations in a row are forgotten.
        assert_eq!(a.in_row, 1);
        a.tick(at(lost + 200), None, [1.0, -4.0], 3.0, Some(at(lost + 200)));
        a.tick(at(lost + 300 + AVOID_CLEAR.as_millis() as u64), None, [1.0, -4.0], 3.0, Some(at(lost)));
        assert_eq!(a.in_row, 0);
    }

    /// Look by look, with synthetic panorama observations (all round,
    /// metric): a wall between the bot and the target, the bot pinned
    /// before it (the test does not walk). The target walks along behind
    /// it, seen every look: the way round is planned for where they are;
    /// it gets nowhere: re-located, the wall tried the other way round;
    /// again and again: a rest; then hidden: lost. Never more re-locations
    /// than the timeouts allow.
    #[test]
    fn following_round_a_wall_watches_them_and_never_circles_for_ever() {
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _rt = rt.enter();
        let bridge = crate::bridge::test_bridge();
        let me = Follower::default();
        {
            let mut s = me.inner.lk();
            (s.running, s.stand, s.state) = (true, STAND_M, "following");
        }
        let stop = AtomicBool::new(false);
        // A wall 2.5 m high, 4 m wide, 1 m ahead (-z) of the bot.
        let mut wall = Vec::new();
        for i in 0..=80 {
            for j in 0..=50 {
                wall.push([-2.0 + 0.05 * i as f32, 0.05 * j as f32, -1.0]);
            }
        }
        let t0 = Instant::now();
        let step = Duration::from_millis(200);
        let view = |at: Instant, found: bool| View {
            at,
            then: [0.0, 0.0],
            found,
            yaw: 0.0,
            glance: false,
            tags: usize::from(found),
            eye: [0.0, 1.6, 0.0],
            floor: 0.0,
            metres: 1.0,
            points: wall.clone(),
            all_round: true,
        };
        let mut t = Track::default();
        let (mut sides, mut rests, mut longest) = (Vec::new(), 0, Duration::ZERO);
        let mut replans_at_6s = 0;
        let mut hidden_at = None;
        for k in 0..900u32 {
            let at = t0 + step * k;
            let secs = (step * k).as_secs_f32();
            // Behind the wall, walking along it (0.3 m/s), seen every look
            // for 60 s (the panorama: kept or named); then hidden, the bot
            // going round the wall again by then (after a rest).
            let x = -1.0 + 0.3 * (secs % 10.0);
            let seen = secs < 60.0;
            if seen {
                t.add_fix(at, [x, -4.0], 0.0, 0.3);
            } else {
                hidden_at.get_or_insert(at);
            }
            me.steer(&bridge, &mut t, &view(at, seen), &stop).unwrap();
            if let Some(w) = t.wall {
                if sides.last() != Some(&w.side) {
                    sides.push(w.side);
                }
            }
            if let Some(since) = t.avoid.since {
                longest = longest.max(at.saturating_duration_since(since));
            }
            // The run's loop re-locates (the lens, the panorama): here the
            // target stays where it was seen; the legs go on.
            if t.relocate.take().is_some() {
                assert!(t.wall.is_none() && !t.routing_at(at), "going round dropped");
                t.hold_until = None;
            }
            if t.avoid.rest_until.is_some_and(|r| at < r) {
                rests += 1;
                assert_eq!(t.hold_until, t.avoid.rest_until, "resting, the legs stand");
            }
            if k == 30 {
                // Going round, watching them: planned again for where they
                // are now as they walk.
                assert_eq!(me.inner.lk().avoid.why, "wall");
                replans_at_6s = t.avoid.replans;
                let (p, _) = t.avoid.planned.unwrap();
                assert!((p[0] - t.target_at(at).unwrap()[0]).abs() < REPLAN_M, "{p:?}");
            }
            if !seen && t.avoid.last_relocate == "lost" {
                let hidden = at.saturating_duration_since(hidden_at.unwrap());
                assert!(hidden > AVOID_UNSEEN - step * 2 && hidden <= AVOID_UNSEEN + step * 2, "{hidden:?}");
                break;
            }
        }
        let a = t.avoid.clone();
        assert!(replans_at_6s >= 3, "{a:?}");
        // Re-located now and then (stalled: the bot got no nearer: at 15,
        // 30 and 45 s), the wall tried both ways, a rest after three in a
        // row; no episode longer than the timeouts; and lost once hidden.
        assert_eq!(a.relocates, AVOID_ROUNDS + 1, "{a:?}");
        assert!(sides.len() >= 3 && sides.contains(&1.0) && sides.contains(&-1.0), "{sides:?}");
        assert!(rests > 0, "{a:?}");
        assert!(longest <= AVOID_STALL + step * 2, "{longest:?}");
        assert_eq!(a.last_relocate, "lost", "{a:?}");
        assert!(t.wall.is_none() && t.relocate.is_none());
        let s = me.status();
        assert!(s["avoid"]["relocates"].as_u64() == Some(a.relocates as u64) && s["avoid"]["timeout_s"] == json!(AVOID_FOR.as_secs()), "{s}");
    }

    #[test]
    fn lost_the_lens_looks_first_then_the_bot_turns() {
        // The lens's quick sweep (from where they were) before the bot
        // turns at all; the turning round last, from the second search on.
        assert_eq!(search_stages(1), vec!["lens_ring", "body_turn"]);
        assert_eq!(search_stages(2), vec!["lens_ring", "body_turn", "scan"]);
    }

    /// Decision D41 (the user, 2026-10-09): unseen over a second is lost,
    /// standing or walking, and the search's first view goes out at once;
    /// kept by position with no name, the lens looks after a second too.
    #[test]
    fn a_second_unseen_starts_the_lens() {
        assert!(LOST_AFTER <= Duration::from_millis(1000));
        assert!(CONFIRM_AFTER <= Duration::from_millis(1000));
        // Asked for as lost: the sweep's first Pose on the orbit's next
        // tick (`Orbit::sweep`, `State::snap_next`), within SWEEP_ASK_FOR
        // of the legs letting go; lost to the first Pose: at most 1.1 s
        // with the orbit's 30 Hz tick.
        let tick = Duration::from_secs_f32(1.0 / crate::orbit::OrbitSettings::default().rate_hz);
        assert!(LOST_AFTER + tick <= Duration::from_millis(1100));
    }

    /// A synthetic panorama: the scene's people (feet x, z, height) as
    /// person-shaped boxes; its cloud, bodies (no names) and tracking.
    fn pano_of(s: &mut vrc_pano::synth::Scene, boxes: &[(f32, f32, f32)]) -> (Vec<(Option<String>, Option<&'static str>, vrc_pano::Body)>, vrc_pano::Tracking) {
        use crate::panolook::tests::eyes_of;
        use vrc_pano::{decode, Cloud, PanoParams, PeopleParams};
        s.boxes = boxes.iter().map(|&(x, z, h)| vrc_pano::synth::person(s, x, z, h)).collect();
        let eyes = eyes_of(s);
        let frame = decode(&eyes, &PanoParams::default()).unwrap();
        let cloud = Cloud::new(&frame, 4);
        let tr = vrc_pano::Tracking::new(&frame, &eyes, cloud.floor, FLOOR_Y);
        let people = cloud.bodies(&PeopleParams::default()).into_iter().map(|b| (None, None, b)).collect();
        (people, tr)
    }

    /// The lens's read of `name` at `at`: from `from` (world) through the
    /// plate at `plate` (world); placed by the depth at `feet` when given.
    fn read_of(name: &str, at: Instant, from: [f32; 3], plate: [f32; 3], feet: Option<[f32; 3]>, lens: Lens) -> NameSighting {
        let d = [plate[0] - from[0], plate[1] - from[1], plate[2] - from[2]];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let world_yaw = (plate[0] - from[0]).atan2(plate[2] - from[2]).to_degrees().rem_euclid(360.0);
        NameSighting {
            at,
            name: name.into(),
            world_yaw,
            tracking_yaw: world_yaw,
            bearing_deg: 0.0,
            elevation_deg: (d[1] / d[0].hypot(d[2])).atan().to_degrees(),
            lens,
            glow: None,
            bbox: [0.0; 4],
            feet,
            distance_m: feet.map(|f| (f[0] - from[0]).hypot(f[2] - from[2])),
            ray_from: from,
            ray_dir: d.map(|c| c / n),
        }
    }

    /// Decision D40, the live case: Ann named, then out of sight; a sign
    /// board (person-sized) near where she was is kept by position. With
    /// no name a while the lens looks at it, reads nobody's name there:
    /// dropped, lost at once, the search starting with the lens at where
    /// her name last put her. And with no lens at all, the board is kept
    /// `kept_s` at most.
    #[test]
    fn a_kept_board_is_dropped_when_the_lens_reads_no_name_then_the_search_starts_with_the_lens() {
        use crate::panolook::tests::out;
        let mut s = vrc_pano::synth::Scene::room();
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let kept_for = Duration::from_secs_f32(KEPT_S);
        let (ax, az) = out(&s, s.head_yaw, 2.0);
        let (bx, bz) = out(&s, s.head_yaw + 20.0, 1.5);
        let (both, tr) = pano_of(&mut s, &[(ax, az, 1.6), (bx, bz, 1.7)]);
        let (board, _) = pano_of(&mut s, &[(bx, bz, 1.7)]);
        let head = tr.head_world;
        let read = read_of("Ann", at(0), head, [ax, s.room_min[1] + 2.0, az], None, Lens::Front);
        // Named by the lens (the depth's body under her plate).
        let run = |t: &mut Track| {
            let p = t.place(&both, "Ann", std::slice::from_ref(&read), &tr, at(0), kept_for).unwrap();
            assert_eq!(p.how, "lens");
            assert!((p.gap - 2.0).abs() < 0.2, "{p:?}");
        };
        let mut t = Track::default();
        run(&mut t);
        assert_eq!(t.named_at, Some(at(0)));
        // Half a second on: she is gone, the board is near enough to keep.
        let p = t.place(&board, "Ann", &[], &tr, at(500), kept_for).expect("kept");
        assert_eq!(p.how, "kept");
        assert!(angle_diff(p.world_yaw, s.head_yaw + 20.0).abs() < 4.0, "{p:?}");
        let (yaw, gap) = t.kept.expect("kept");
        // The confirming: nothing yet; from CONFIRM_AFTER the lens looks there.
        assert_eq!(t.confirm.tick(at(900), t.named_at, t.kept, "Ann", &[], 0, kept_for), ConfirmAct::Nothing);
        t.place(&board, "Ann", &[], &tr, at(2500), kept_for).expect("kept");
        let ConfirmAct::Look(look) = t.confirm.tick(at(2600), t.named_at, t.kept, "Ann", &[], 0, kept_for) else { panic!("{:?}", t.confirm) };
        assert!(angle_diff(look, yaw).abs() < 2.0 && gap < CONFIRM_RANGE_M);
        // One read of nothing: wait; CONFIRM_READS: not her.
        assert_eq!(t.confirm.tick(at(3000), t.named_at, t.kept, "Ann", &[], 1, kept_for), ConfirmAct::Nothing);
        let act = t.confirm.tick(at(3400), t.named_at, t.kept, "Ann", &[], CONFIRM_READS, kept_for);
        assert_eq!(act, ConfirmAct::Drop("no name"));
        t.drop_kept(at(3400), "no name");
        // Lost at once (the run's loop: `dropped`), the board no more hers,
        // the target where her name put her, unseen since.
        assert!(t.dropped.is_some() && t.world.is_none() && t.kept.is_none());
        assert!(t.place(&board, "Ann", &[], &tr, at(3600), kept_for).is_none(), "the board is not kept again");
        let f = t.target.unwrap();
        assert!(f.at == at(0) && (f.pos[0].hypot(f.pos[1]) - 2.0).abs() < 0.2, "{:?}", f.pos);
        // The search: the lens first, at where her name last put her.
        let way = t.name_way().expect("her name's way");
        assert!(angle_diff(way, bearing([0.0, 0.0], f.pos)).abs() < 1e-3);
        assert_eq!(search_stages(1)[0], "lens_ring");
        assert_eq!((t.confirm.looks, t.confirm.drops, t.confirm.last), (1, 1, "no name"));

        // No lens at all (it could not look): the board kept `kept_s` at
        // most after her name, then dropped ("expired").
        let mut t = Track::default();
        run(&mut t);
        let mut last_kept = 0;
        for ms in (500..15_000).step_by(500) {
            match t.place(&board, "Ann", &[], &tr, at(ms), kept_for) {
                Some(p) if p.how == "kept" => last_kept = ms,
                Some(p) => panic!("{p:?}"),
                None => {}
            }
        }
        assert!(Duration::from_millis(last_kept) <= kept_for && Duration::from_millis(last_kept + 500) > kept_for, "{last_kept}");
        assert!(t.dropped.is_some() && t.confirm.last == "expired");
    }

    /// Decision D40: what the lens reads while it looks at the one kept.
    #[test]
    fn the_confirming_look_reads_them_someone_else_or_nobody() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let kept_for = Duration::from_secs_f32(KEPT_S);
        let ask = |c: &mut Confirm, gap: f32| {
            let act = c.tick(at(3000), Some(at(0)), Some((90.0, gap)), "Ann", &[], 0, kept_for);
            assert_eq!(act, ConfirmAct::Look(90.0));
        };
        // Her name read: confirmed.
        let mut c = Confirm::default();
        ask(&mut c, 2.0);
        let ann = [Read { name: "Ann", world_yaw: 95.0, distance: Some(2.1) }];
        assert_eq!(c.tick(at(3500), Some(at(0)), Some((90.0, 2.0)), "Ann", &ann, 1, kept_for), ConfirmAct::Nothing);
        assert_eq!((c.confirmed, c.last, c.asked.is_none()), (1, "named", true));
        // Bob's plate over the one kept: not her.
        let mut c = Confirm::default();
        ask(&mut c, 2.0);
        let bob = [Read { name: "Bob", world_yaw: 85.0, distance: Some(2.3) }];
        assert_eq!(c.tick(at(3500), Some(at(0)), Some((90.0, 2.0)), "Ann", &bob, 1, kept_for), ConfirmAct::Drop("other name"));
        // Bob far behind the one kept (another depth): not a verdict.
        let mut c = Confirm::default();
        ask(&mut c, 2.0);
        let far = [Read { name: "Bob", world_yaw: 88.0, distance: Some(6.0) }];
        assert_eq!(c.tick(at(3500), Some(at(0)), Some((90.0, 2.0)), "Ann", &far, 1, kept_for), ConfirmAct::Nothing);
        // Too far for a plate to show: no "no name" verdict; unanswered.
        let mut c = Confirm::default();
        ask(&mut c, 12.0);
        assert_eq!(c.tick(at(3800), Some(at(0)), Some((90.0, 12.0)), "Ann", &[], 5, kept_for), ConfirmAct::Nothing);
        let late = 3000 + (CONFIRM_HOLD + CONFIRM_GRACE).as_millis() as u64 + 100;
        assert_eq!(c.tick(at(late), Some(at(0)), Some((90.0, 12.0)), "Ann", &[], 5, kept_for), ConfirmAct::Nothing);
        assert_eq!(c.last, "unanswered");
        // Asked at most every CONFIRM_EVERY.
        assert_eq!(c.tick(at(late + 100), Some(at(0)), Some((90.0, 12.0)), "Ann", &[], 0, kept_for), ConfirmAct::Nothing);
        assert!(matches!(c.tick(at(3000) + CONFIRM_EVERY, Some(at(0)), Some((90.0, 12.0)), "Ann", &[], 0, kept_for), ConfirmAct::Look(_)));
        // Named meanwhile by a read from before the look (not kept any more):
        // confirmed, whatever the look read since.
        let mut c = Confirm::default();
        ask(&mut c, 2.0);
        assert_eq!(c.tick(at(3600), Some(at(2900)), None, "Ann", &[], 5, kept_for), ConfirmAct::Nothing);
        assert_eq!((c.confirmed, c.last), (1, "named"));
        // Named this look (not kept): nothing to confirm.
        let mut c = Confirm::default();
        assert_eq!(c.tick(at(3000), Some(at(0)), None, "Ann", &[], 0, kept_for), ConfirmAct::Nothing);
    }

    /// Decision D40: the follower kept a board ahead; the lens reads Ann's
    /// name behind and to the side: the follower takes her there at once
    /// (the depth's body along the read; with nobody in the depth there,
    /// the read's bearing at the last distance), seen by the lens.
    #[test]
    fn the_lens_finds_them_behind_and_the_follower_takes_it_at_once() {
        use crate::panolook::tests::out;
        let mut s = vrc_pano::synth::Scene::room();
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let kept_for = Duration::from_secs_f32(KEPT_S);
        let (bx, bz) = out(&s, s.head_yaw, 1.5);
        let (ax, az) = out(&s, s.head_yaw + 150.0, 1.6);
        let (board, tr) = pano_of(&mut s, &[(bx, bz, 1.7)]);
        let (with_her, _) = pano_of(&mut s, &[(bx, bz, 1.7), (ax, az, 1.6)]);
        let head = tr.head_world;
        let y = s.room_min[1];
        // Named on the board at first (as if), then kept there.
        let first = read_of("Ann", at(0), head, [bx, y + 2.05, bz], None, Lens::Front);
        let mut t = Track::default();
        assert_eq!(t.place(&board, "Ann", &[first], &tr, at(0), kept_for).unwrap().how, "lens");
        assert_eq!(t.place(&board, "Ann", &[], &tr, at(1000), kept_for).unwrap().how, "kept");
        // A look of the lens (behind and over the head) reads her name back
        // there; this read was not placed by the depth (no feet).
        let lens = [head[0], head[1] + 0.25, head[2]];
        let behind = read_of("Ann", at(1500), lens, [ax, y + 1.95, az], None, Lens::Look);
        let mut t2 = Track::default();
        t2.place(&board, "Ann", &[read_of("Ann", at(0), head, [bx, y + 2.05, bz], None, Lens::Front)], &tr, at(0), kept_for);
        t2.place(&board, "Ann", &[], &tr, at(1000), kept_for);
        let p = t.place(&with_her, "Ann", std::slice::from_ref(&behind), &tr, at(1600), kept_for).expect("placed");
        assert_eq!(p.how, "lens");
        assert!(angle_diff(p.world_yaw, s.head_yaw + 150.0).abs() < 6.0 && (p.gap - 1.6).abs() < 0.3, "{p:?}");
        assert_eq!(t.named_at, Some(at(1600)), "a body this look: placed now");
        assert!(t.kept.is_none() && t.dropped.is_none());
        let fix = t.target.unwrap();
        assert!(angle_diff(bearing([0.0, 0.0], fix.pos), tr.yaw(s.head_yaw + 150.0)).abs() < 6.0, "{:?}", fix.pos);
        // Nobody in the depth back there (hidden): her read's bearing, at
        // the last distance (the board's, 1.5 m) but never nearer than
        // BEARING_ONLY_MIN_GAP_M, at the read's time.
        let p = t2.place(&board, "Ann", std::slice::from_ref(&behind), &tr, at(1600), kept_for).expect("placed");
        assert_eq!(p.how, "lens");
        assert_eq!(p.when, at(1500));
        assert!(angle_diff(p.world_yaw, s.head_yaw + 150.0).abs() < 6.0 && (p.gap - BEARING_ONLY_MIN_GAP_M).abs() < 0.2, "{p:?}");
        // A read placed by the depth at reading time (feet): there.
        let mut t3 = Track::default();
        let placed = read_of("Ann", at(0), lens, [ax, y + 1.95, az], Some([ax, y, az]), Lens::Orbit);
        let p = t3.place(&with_her, "Ann", &[placed], &tr, at(100), kept_for).expect("placed");
        assert!(p.how == "lens" && (p.gap - 1.6).abs() < 0.3, "{p:?}");
        // A read older than the last name is not taken over it.
        let old = read_of("Ann", at(1400), lens, [ax, y + 1.95, az], None, Lens::Look);
        let p = t.place(&with_her, "Ann", &[old], &tr, at(1700), kept_for).unwrap();
        assert!(p.how == "kept" && angle_diff(p.world_yaw, s.head_yaw + 150.0).abs() < 6.0, "{p:?}");
    }

    /// Decision D40: the head's pitch follows their plate, from the lens's
    /// read when the depth has no body: up on a platform the head goes up,
    /// at someone sitting close by it goes down (clamped), smoothly.
    #[test]
    fn the_pitch_follows_the_plate_from_the_lens() {
        use crate::panolook::tests::out;
        let mut s = vrc_pano::synth::Scene::room();
        let t0 = Instant::now();
        let kept_for = Duration::from_secs_f32(KEPT_S);
        let (_, tr) = pano_of(&mut s, &[]);
        let head = tr.head_world;
        let lens = [head[0], head[1] + 0.35, head[2] - 0.35];
        // Up on a platform 2 m ahead: her plate 1.2 m over the eyes.
        let (ux, uz) = out(&s, s.head_yaw, 2.0);
        let mut up = read_of("Ann", t0, lens, [ux, head[1] + 1.2, uz], None, Lens::Front);
        up.distance_m = Some(2.0);
        let mut t = Track::default();
        t.place(&[], "Ann", &[up], &tr, t0, kept_for).expect("placed");
        let rise = t.target.unwrap().tag_rise;
        assert!((rise - 1.2).abs() < 0.1, "{rise}");
        let pitch = t.tag_pitch().unwrap();
        assert!((pitch - (0.9f32).atan2(2.0).to_degrees()).abs() < 3.0, "{pitch}");
        // Sitting 1 m away, the plate half a metre under the eyes: down, as
        // far as the head goes.
        let (sx, sz) = out(&s, s.head_yaw, 1.0);
        let mut sit = read_of("Ann", t0 + Duration::from_millis(100), lens, [sx, head[1] - 0.5, sz], None, Lens::Front);
        sit.distance_m = Some(1.0);
        let mut t = Track::default();
        t.place(&[], "Ann", &[sit], &tr, t0, kept_for).expect("placed");
        assert_eq!(t.tag_pitch().unwrap(), HEAD_PITCH_MIN);
        // Smoothed: a third of the way in a sixth of a second, nearly there
        // in 2 s, never past it; clamped.
        let mut p = PITCH;
        for _ in 0..4 {
            p = smooth_pitch_step(p, pitch, 0.04);
        }
        assert!(p > PITCH && p < PITCH + 0.5 * (pitch - PITCH), "{p}");
        for _ in 0..50 {
            p = smooth_pitch_step(p, pitch, 0.04);
            assert!(p <= pitch + 1e-4);
        }
        assert!((p - pitch).abs() < 1.0, "{p}");
        let mut q = 0.0;
        for _ in 0..200 {
            q = smooth_pitch_step(q, 80.0, 0.04);
        }
        assert!((q - HEAD_PITCH_MAX).abs() < 0.1, "{q}");
    }

    #[test]
    fn follow_settings_are_checked() {
        let f = Follower::default();
        assert_eq!(f.settings.lk().kept_s, KEPT_S);
        f.set(&json!({"kept_s": 6})).unwrap();
        assert_eq!(f.kept_for(), Duration::from_secs(6));
        assert!(f.set(&json!({"kept_s": 1})).is_err() && f.set(&json!({"kept_s": "x"})).is_err() && f.set(&json!(5)).is_err());
        assert_eq!(f.status()["settings"]["kept_s"], json!(6.0));
    }

    #[test]
    fn bearings() {
        assert!((bearing([0.0, 0.0], [0.0, -1.0])).abs() < 1e-4);
        assert!((bearing([0.0, 0.0], [1.0, 0.0]) - 90.0).abs() < 1e-4);
        assert!((wrap(190.0) + 170.0).abs() < 1e-4);
    }
}
