//! Following a player in the room (VR): their name tag says who and where.
//!
//! Two loops. The eyes (this thread): take the newest frame (looking ahead,
//! a little down), read the name tags (OCR) and place them by stereo
//! (`vrc-players`, the same logic as the model's look around); each sighting
//! of the target is a fix of where they stand. The legs (a thread of their
//! own, 25 times a second): integrate the avatar's own velocity (OSCQuery)
//! into an odometry, so a fix half a second old still says how far the
//! target is *now*; turn towards them (walking follows the head, the body
//! follows) and set the thumbstick from the distance left to the standing
//! distance, braking smoothly as it shrinks: a round of the eyes takes
//! 0.3-0.8 s, and at full stick the avatar runs 4 m/s.
//!
//! In the way (each view's points, along the way to the target, measured
//! from the ground just before it): whatever does not reach the eyes is
//! jumped first, with a run-up (user's rule). What reaches them, or what a
//! jump did not get past, is walked round by following it (a "bug"
//! algorithm, user's rule): keep it on one side (the side away from the
//! target's way round), each view taking the free heading nearest that
//! side, round corners, until the straight way to the target is open (no
//! wall up to the eyes; user's rule: keep checking while going round): out
//! of an enclosure or into one alike. Every GLANCE_EVERY the head turns to
//! the target's way for a view of it, as the walk along the wall faces
//! elsewhere. While following a wall the target may be out of sight: the
//! bot keeps going for where they were. Pushing without moving (stuck) jumps once,
//! then backs off and turns aside.
//!
//! Glances turn only the head: the stick is split into ahead and aside
//! (VRChat walks relative to the head), so the walk keeps its way.
//!
//! Heights go by the avatar's eyes (its OSCQuery eye height), not by fixed
//! numbers: a wall reaches over them; whatever is lower is jumped first.
//! What the avatar carries (a bag, a cup at its side) is in view near it:
//! nearer than 1.1 m a low thing counts only across the way (a ledge, a
//! sofa), not at one side (a prop).
//!
//! Lost (unseen LOST_AFTER walking, LOST_STANDING standing), the bot stands
//! and searches (`search`, decision D45): the head looks round the front
//! half, SEARCH_VIEWS views from where they were last seen out either way
//! by turns (their way first), while the user camera's lens snaps as many
//! views round the other half (`Orbit::sweep_views`). Their name read by
//! the eyes ends it; read by the lens, the head turns there and looks. A search that finds nobody walks a while toward where they
//! were, then searches again. Following ends only when told to stop or
//! when they leave the room.
//!
//! **The lens behind** (decision D45): the user camera's lens looks back
//! over the head while the eyes look ahead (`orbit`, `travel.rear`). A
//! name of theirs it reads while the eyes do not see them places them by
//! its bearing at the last distance (`seen_by: "lens"`): the bot turns to
//! them and the eyes take over.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Value};
use vrc_players::names::match_score;
use vrc_stereo::{SgmParams, Stereo};
use vrc_vr::osc::Osc;
use vrc_vr::remote::FLOOR_Y;
use vrc_vr::scan::{self, angle_diff};
use vrc_vr::tap::EyeFrame;

use crate::bridge::Bridge;
use crate::orbit::NameSighting;
use crate::Lock;

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
/// The search (user: turn the way the player was last going, nothing more
/// clever): from where they were last seen, round one way a view every
/// SEARCH_STEP_DEG (each turn SEARCH_STEP_S, the head tilting into it), a
/// full turn a round. The way: the side their last movement went, across
/// the bot's line of sight to them (moving within SEARCH_MOVED_FOR of being
/// lost); standing, the side of the bot's facing they were on.
const SEARCH_STEP_DEG: f32 = 60.0;
const SEARCH_STEP_S: f32 = 0.35;
const SEARCH_TILT: f32 = 8.0;
const SEARCH_SETTLE: Duration = Duration::from_millis(80);
const SEARCH_MOVED_FOR: Duration = Duration::from_secs(5);
/// Every third round, from the second on (2, 5, 8...), looks up this high
/// (someone up on something), the others at PITCH. The head goes level
/// again over SEARCH_LEVEL_S.
const SEARCH_UP_PITCH: f32 = 20.0;
const SEARCH_LEVEL_S: f32 = 0.3;
/// Feet under a tag: within this of under it, at least this many points.
const FEET_RADIUS_M: f32 = 0.25;
const FEET_POINTS: usize = 15;
/// Up or down from the bot by more than a step: it goes up to them (as near
/// as this), the stairs are no stopping place.
const OTHER_FLOOR_STAND_M: f32 = 0.8;
/// Not seen for this long: stand and search; standing at their side, a
/// name tag missed a view or two is not them gone. (The panorama's second,
/// D41, stopped the stereo follower to search again and again: a look
/// takes 0.3-0.8 s and the OCR misses a plate now and then.)
const LOST_AFTER: Duration = Duration::from_millis(1500);
const LOST_STANDING: Duration = Duration::from_secs(3);
/// The search (decision D45): the head's views round the front half (the
/// lens's sweep takes as many the other way), and how long after the last
/// the lens's reads (OCR) in flight are waited for.
const SEARCH_VIEWS: usize = 3;
const LENS_WAIT: Duration = Duration::from_millis(600);
/// A lens read of their name this recent places them (by its bearing,
/// at the last distance: DEFAULT_GAP_M with none, at least MIN_LENS_GAP_M).
const LENS_FRESH: Duration = Duration::from_millis(1500);
const MIN_LENS_GAP_M: f32 = 1.0;
/// Out of the room for this long: gone.
const GONE_AFTER: Duration = Duration::from_secs(20);
const PITCH: f32 = -10.0;
/// Search views: offsets (degrees) from where the target was last seen.
/// A search that found nobody: walk toward where they were this long.
const SEEK_FOR: Duration = Duration::from_secs(4);
/// A whole search found nothing: wait this long before the next.
const SEARCH_PAUSE: Duration = Duration::from_secs(2);

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
/// (NEAR_POINTS nearer than 1.1 m): stray points of stereo come alone, a
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
const STEREO_THREADS: usize = 6;
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
const GLANCE_EVERY: Duration = Duration::from_millis(1500);
const SAME_SIDE_FOR: Duration = Duration::from_secs(8);
/// Following, a look runs the detector this often (the lasting map's things).
const DETECT_EVERY: Duration = Duration::from_secs(3);
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
/// AVOID_GRACE (decision D37). Every look (the eyes, the lens behind)
/// keeps the target's place; the way round is planned again for where
/// they are now when they moved REPLAN_M since it was last, or every
/// REPLAN_EVERY. Re-located (the eyes look where they were, the
/// map plans afresh, the wall is tried the other way round) when the
/// episode got no nearer by PROGRESS_M for AVOID_STALL, or lasted
/// AVOID_FOR; not seen for AVOID_UNSEEN of it, they are lost (the search:
/// head and lens round them). After AVOID_ROUNDS re-locations with no AVOID_CLEAR
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
/// Going round, the lens is re-aimed less often for this long after each
/// look (`Orbit::calm_travel`).
const AVOID_CALM: Duration = Duration::from_secs(2);
/// Re-locating: the legs stand this long at most.
const RELOCATE_HOLD: Duration = Duration::from_secs(3);
/// A lens read of theirs with no distance (its bearing alone) is taken at
/// their last distance, DEFAULT_GAP_M with none.
const DEFAULT_GAP_M: f32 = 2.0;
/// The head's pitch at them settles over this (seconds), and moves only
/// when it would by more than PITCH_DEADBAND (degrees).
const PITCH_SMOOTH_S: f32 = 0.4;
const PITCH_DEADBAND: f32 = 1.5;
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

#[derive(Default)]
pub struct Follower {
    inner: Mutex<State>,
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
    /// How the last fix was made: "stereo" (a tag read and placed by the
    /// eyes), "lens" (the lens behind read their name: its bearing).
    seen_by: &'static str,
    /// The going round's watch, as the last look left it.
    avoid: Avoid,
    /// Lost: the search's stage ("lens_ring", "body_turn", "scan"); empty
    /// otherwise.
    search_stage: &'static str,
    /// The head's pitch as sent.
    head_pitch: f32,
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
    /// Stop going round; look for them (the eyes their way), then the
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
    /// When a look last ran the detector.
    detected: Option<Instant>,
    /// When a look last chose a way round by the lasting map.
    routed: Option<Instant>,
    /// That way's length (world metres) from where the bot then was
    /// (odometry): near them through glass is not there yet.
    route: Option<([f32; 2], f32)>,
    /// The going round's watch, the re-location it asked for, and the
    /// legs standing until then (re-locating, resting).
    avoid: Avoid,
    relocate: Option<&'static str>,
    hold_until: Option<Instant>,
    /// The last thing in the way on the straight way (world metres).
    blocked_m: Option<f32>,
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
                // Standing still, give or take the noise of stereo.
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
            "head_pitch_deg": (s.head_pitch as f64 * 10.0).round() / 10.0,
        })
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
        // Search rounds since they were lost (see SEARCH_UP_PITCH).
        let mut rounds = 0u32;
        let mut next_search = Instant::now();
        let mut last_glance = Instant::now();
        let mut osc: Option<Osc> = None;
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
            if osc.is_none() {
                osc = bridge.osc_query().ok();
            }
            let metres = match osc.as_ref().map(|o| o.eye_height()) {
                Some(Ok(h)) if h > 0.0 => h as f32 / (bridge.anim.params().head_height - FLOOR_Y),
                _ => {
                    osc = None;
                    1.0
                }
            };
            // Standing (at their side), a name tag missed a few views is not
            // them gone: LOST_STANDING.
            let standing = self.inner.lk().moving.abs() < 0.05;
            // Following a wall, the target may be out of sight a while.
            // (Going round a wall long out of sight of them ends by the
            // episode's watch, `Avoid`: lost, the search.)
            let (lost, unseen) = {
                let t = track.lk();
                let unseen = t.target.map_or(Duration::MAX, |f| f.at.elapsed());
                let seeking = t.seek.is_some_and(|until| Instant::now() < until);
                let after = if standing { LOST_STANDING } else { LOST_AFTER };
                (!t.rounding() && !seeking && unseen > after, unseen)
            };
            // Along a wall, now and then a view the target's way.
            let glance = {
                let t = track.lk();
                match (t.rounding(), t.target_now()) {
                    (true, Some(goal)) if last_glance.elapsed() > GLANCE_EVERY => {
                        let to = bearing(t.pos, goal);
                        (angle_diff(to, t.facing).abs() > VIEW_HALF_DEG).then_some(to)
                    }
                    _ => None,
                }
            };
            let result = if let Some(to) = glance {
                last_glance = Instant::now();
                let pitch = track.lk().tag_pitch().unwrap_or(PITCH);
                self.look(&bridge, &track, &target, &room, metres, Some((to, pitch)), &stop).map(|_| ())
            } else if !lost {
                self.look(&bridge, &track, &target, &room, metres, None, &stop).map(|_| ())
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
                let found = self.search(&bridge, &track, &target, &room, metres, &mut rounds, &stop);
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
            // Going round got nowhere: re-located (the watch asked for it).
            let ask = track.lk().relocate.take();
            if let Some(why) = ask {
                if let Err(e) = self.relocate(&bridge, &track, &target, &room, metres, why, &stop) {
                    tracing::warn!("follow: re-locating failed: {e:#}");
                }
                let mut t = track.lk();
                if t.avoid.rest_until.is_none() {
                    t.hold_until = None;
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

    /// The search (decision D45): the head round the front half from where
    /// the target was last seen, SEARCH_VIEWS views SEARCH_STEP_DEG apart
    /// out either way by turns (their way first), and meanwhile the user
    /// camera's lens round the other half (`Orbit::sweep_views`, the views
    /// opposite): both halves at once. True as soon as the eyes see them, or the lens reads their
    /// name (the head turns there and looks). Some rounds look up.
    #[allow(clippy::too_many_arguments)]
    fn search(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, rounds: &mut u32, stop: &AtomicBool) -> anyhow::Result<bool> {
        let (from, way, facing) = {
            let t = track.lk();
            let from = t.target_now().map_or(t.facing, |p| bearing(t.pos, p));
            (from, t.search_way(from), t.facing)
        };
        bridge.anim.owner_hands.store(true, Ordering::SeqCst);
        *rounds += 1;
        let pitch = if *rounds % 3 == 2 { SEARCH_UP_PITCH } else { PITCH };
        let views = search_views(from, way, SEARCH_VIEWS);
        // The lens behind meanwhile: its quick sweep round the half the
        // head does not look at (the views opposite, out either way), their
        // name sought (a read turns it to them).
        let since = Instant::now();
        if let Some(head) = bridge.orbit.head() {
            let behind = (from + 180.0 + head.offset).rem_euclid(360.0);
            if let Err(e) = bridge.orbit.sweep_views("follow", Some(behind), Some(target), SEARCH_VIEWS as u32) {
                tracing::debug!("follow: searching, no lens sweep: {e:#}");
            }
        }
        let result = (|| {
            let mut last = facing;
            for (k, yaw) in views.into_iter().enumerate() {
                if stop.load(Ordering::SeqCst) {
                    return Ok(false);
                }
                // Read behind (the lens's sweep): the head turns there.
                if let Some(n) = lens_read(bridge, target, since) {
                    tracing::info!(tracking_yaw = n.tracking_yaw.round(), "follow: searching, the lens read their name: turning there");
                    return self.look(bridge, track, target, room, metres, Some((n.tracking_yaw, PITCH)), stop);
                }
                // The pace of a turn its size.
                let secs = (angle_diff(yaw, last).abs() / SEARCH_STEP_DEG * SEARCH_STEP_S).max(SEARCH_STEP_S * 0.5);
                let tilt = if k == 0 { 0.0 } else { angle_diff(yaw, last).signum() * SEARCH_TILT };
                last = yaw;
                {
                    let whitelist = bridge.social.whitelist_names();
                    let mut vr = bridge.vr.lk();
                    vr.rig(&whitelist)?.hmd.hold_still(true)?;
                    let turned = vr.turn_gently(yaw, pitch, secs, tilt, 0.0);
                    if turned.is_ok() {
                        std::thread::sleep(SEARCH_SETTLE);
                    }
                    vr.rig(&whitelist)?.hmd.hold_still(false)?;
                    turned?;
                    track.lk().facing = yaw;
                }
                if self.look(bridge, track, target, room, metres, Some((yaw, pitch)), stop)? {
                    return Ok(true);
                }
            }
            // The lens's last reads (OCR) in flight.
            let until = Instant::now() + LENS_WAIT;
            while Instant::now() < until && !stop.load(Ordering::SeqCst) {
                if let Some(n) = lens_read(bridge, target, since) {
                    tracing::info!(tracking_yaw = n.tracking_yaw.round(), "follow: searching, the lens read their name: turning there");
                    return self.look(bridge, track, target, room, metres, Some((n.tracking_yaw, PITCH)), stop);
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            Ok(false)
        })();
        bridge.orbit.end_sweep("stopped");
        // The head level again (gently, after a round looking up), the hands
        // back at the sides, wherever it ended.
        {
            let mut vr = bridge.vr.lk();
            let yaw = vr.yaw;
            if (vr.pitch - PITCH).abs() > 1.0 {
                let _ = vr.rig(&[]).and_then(|r| r.hmd.hold_still(true));
                let _ = vr.turn_gently(yaw, PITCH, SEARCH_LEVEL_S, 0.0, 0.0);
                let _ = vr.rig(&[]).and_then(|r| r.hmd.hold_still(false));
            }
            let _ = vr.face(yaw, PITCH);
        }
        bridge.anim.owner_hands.store(false, Ordering::SeqCst);
        result
    }

    /// One look: the newest frame (or, with `aim`, the first one looking
    /// that way, the whole bot turned there), its name tags placed; whether
    /// the target was among them.
    #[allow(clippy::too_many_arguments)]
    fn look(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, aim: Option<(f32, f32)>, stop: &AtomicBool) -> anyhow::Result<bool> {
        // Along a wall a look elsewhere is a glance: the head alone.
        let glance = aim.is_some() && track.lk().rounding();
        let whitelist = bridge.social.whitelist_names();
        let (frame, ocr, detect) = {
            let mut vr = bridge.vr.lk();
            let frame = match aim {
                Some((yaw, pitch)) => {
                    // Exactly that way: the animation's sway would miss it.
                    vr.rig(&whitelist)?.hmd.hold_still(true)?;
                    // (After a gentle turn the bot already faces that way:
                    // the hands stay a little back of rest.)
                    let there = vrc_vr::scan::angle_diff(yaw, vr.yaw).abs() < 1.0 && (vr.pitch - pitch).abs() < 1.0;
                    let turned = if glance {
                        vr.aim(yaw, pitch)
                    } else if there {
                        Ok(())
                    } else {
                        vr.face(yaw, pitch)
                    };
                    let frame = turned.and_then(|()| scan::rendered_at(&mut vr.rig(&whitelist)?.tap, yaw, pitch, Duration::from_secs(1)));
                    // The head back the way the walk goes: walking follows
                    // the head, and the next view judges from it.
                    let back = if glance {
                        let t = track.lk();
                        vr.aim(t.facing, t.tag_pitch().unwrap_or(PITCH))
                    } else {
                        Ok(())
                    };
                    vr.rig(&whitelist)?.hmd.hold_still(false)?;
                    back?;
                    if !glance {
                        track.lk().facing = yaw;
                    }
                    frame?
                }
                None => vr.rig(&whitelist)?.tap.read()?.ok_or_else(|| anyhow::anyhow!("no frame yet"))?,
            };
            let rig = vr.rig(&whitelist)?;
            let ocr = rig.ocr.clone().ok_or_else(|| anyhow::anyhow!("following needs OCR"))?;
            (frame, ocr, rig.detect.clone())
        };
        // The frame is at most a frame old: as good as now for the odometry.
        let at = Instant::now();
        let stereo = Stereo::from_frame(&frame, vrc_stereo::match_scale(frame.width)).ok_or_else(|| anyhow::anyhow!("not an 8-bit frame"))?;
        let disp = stereo_pool().install(|| stereo.disparity(&SgmParams::default()));
        let eye = eye_of(&frame);
        let (yaw, _) = frame.views[0].pose.yaw_pitch();
        // The floor, in stereo units (the tracking space's).
        let floor = FLOOR_Y;
        let lines = ocr.lines_rgb(&frame.eye_rgb8(0)?, frame.width as u16, frame.height as u16)?;
        let mut names: Vec<String> = room.to_vec();
        if !names.iter().any(|n| n == target) {
            names.push(target.to_string());
        }
        let seen = vrc_players::sightings(&frame, &stereo, &disp, &lines, &names, &whitelist, floor);
        // Where everyone's plate is, and whether it glows: who is speaking.
        bridge.speaker.saw(&seen, Some(&frame));
        // Whitelisted friends in sight: sightings, for "when did you last see".
        for s in &seen {
            if s.whitelist_rank.is_some() {
                if let Ok(jpeg) = crate::vr::eye_jpeg(&frame, 640) {
                    bridge.sightings.saw(&s.name, &bridge.game.lk().world_name, jpeg);
                }
            }
        }
        let points: Vec<[f32; 3]> = stereo.points(&disp, 2).into_iter().map(|(p, _)| p).collect();
        // The lasting map too (its own thread: dropped when it is busy),
        // where the bot is first (the avatar's beacon, if it has one).
        vrc_nav::beacon_fix(&bridge.mapping.nav, &frame, (eye[1] - floor) * metres, at);
        let people: Vec<[f32; 3]> = seen.iter().map(|s| s.feet).collect();
        bridge.mapping.observe(vrc_nav::vrc_map::Observation::from_tracking(&points, eye, floor, metres, &people, at));
        // And now and then, the things in view.
        let due = track.lk().detected.is_none_or(|t| t.elapsed() >= DETECT_EVERY);
        if let (true, Some(detect)) = (due, detect) {
            track.lk().detected = Some(Instant::now());
            match frame.eye_rgb8(0).and_then(|rgb| detect.detect_rgb(&rgb, frame.width, frame.height)) {
                Ok(found) => {
                    let placed = vrc_players::objects::place(&frame, &stereo, &disp, &found);
                    let rels: Vec<_> = placed
                        .iter()
                        .map(|o| (o, [(o.at[0] - eye[0]) * metres, (o.at[1] - floor) * metres, (o.at[2] - eye[2]) * metres]))
                        .collect();
                    vrc_nav::objects_onto(&bridge.mapping.nav, &rels, metres, at);
                }
                Err(e) => tracing::warn!("follow: detection failed: {e:#}"),
            }
        }
        let hit = seen.iter().filter(|s| match_score(&s.name, target) >= 0.6).max_by(|a, b| a.score.total_cmp(&b.score));
        let feet_up = hit.and_then(|h| feet_height(&points, h.tag, metres, floor));
        // Not in the eyes' view: the lens behind may have read their name.
        let behind = if hit.is_none() {
            let last = track.lk().target.map(|f| f.at);
            lens_read(bridge, target, at.checked_sub(LENS_FRESH).unwrap_or(at)).filter(|n| last.is_none_or(|l| n.at > l))
        } else {
            None
        };
        let mut t = track.lk();
        let then = t.pos_at(at);
        let found = if let Some(hit) = hit {
            let rel = [(hit.feet[0] - eye[0]) * metres, (hit.feet[2] - eye[2]) * metres];
            // Up (or down) stairs: where their feet are; not seen, as before.
            let up = feet_up.or(t.target.map(|f| f.up)).unwrap_or(0.0);
            t.add_fix(at, [then[0] + rel[0], then[1] + rel[1]], up, (hit.tag[1] - eye[1]) * metres);
            t.misses = 0;
            t.seek = None;
            if let Some(mut s) = self.inner_for(stop) {
                s.last_seen = Some(at);
                s.distance = rel[0].hypot(rel[1]);
                s.target_up = t.target.map_or(0.0, |f| f.up);
                s.seen_by = "stereo";
            }
            if let Some(head) = bridge.orbit.head() {
                bridge.orbit.aim_at((bearing(then, [then[0] + rel[0], then[1] + rel[1]]) + head.offset).rem_euclid(360.0));
            }
            true
        } else if let Some(n) = behind {
            // Its bearing at their last distance: the bot turns to them,
            // the eyes take over.
            let then_n = t.pos_at(n.at);
            let gap = t.target_at(n.at).map_or(DEFAULT_GAP_M, |g| (g[0] - then_n[0]).hypot(g[1] - then_n[1])).max(MIN_LENS_GAP_M);
            let (sy, cy) = n.tracking_yaw.to_radians().sin_cos();
            let (up, rise) = t.target.map_or((0.0, 0.0), |f| (f.up, f.tag_rise));
            t.add_fix(n.at, [then_n[0] + sy * gap, then_n[1] - cy * gap], up, rise);
            t.misses = 0;
            t.seek = None;
            tracing::info!(tracking_yaw = n.tracking_yaw.round(), gap, "follow: the lens behind read their name");
            if let Some(mut s) = self.inner_for(stop) {
                s.last_seen = Some(n.at);
                s.distance = gap;
                s.seen_by = "lens";
            }
            true
        } else {
            false
        };
        let view = View { at, then, found, yaw, glance, tags: seen.len(), eye, floor, metres, points, all_round: false };
        self.steer(bridge, &mut t, &view, stop)
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
        // Going round, the way swings about: the lens behind is re-aimed
        // less often (each re-aim stalls the walk).
        if why.is_some() {
            bridge.orbit.calm_travel(AVOID_CALM);
        }
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

    /// Re-locating (the watch over going round asked for it, `why`): a look
    /// where they were last placed (the lens looking the other way). Whether
    /// it found them. The next look plans the way afresh for where they are
    /// (the map, the wall the other way round).
    #[allow(clippy::too_many_arguments)]
    fn relocate(&self, bridge: &Arc<Bridge>, track: &Arc<Mutex<Track>>, target: &str, room: &[String], metres: f32, why: &'static str, stop: &AtomicBool) -> anyhow::Result<bool> {
        tracing::info!(why, "follow: re-locating them");
        let last = {
            let t = track.lk();
            t.target_now().map(|p| (bearing(t.pos, p), t.tag_pitch().unwrap_or(PITCH)))
        };
        match last {
            Some(aim) => self.look(bridge, track, target, room, metres, Some(aim), stop),
            None => Ok(false),
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



/// Following looks again and again: its stereo gets a few cores, not all
/// (all of them made a follow cost about six cores' time, beside the game).
fn stereo_pool() -> &'static rayon::ThreadPool {
    static POOL: std::sync::OnceLock<rayon::ThreadPool> = std::sync::OnceLock::new();
    POOL.get_or_init(|| rayon::ThreadPoolBuilder::new().num_threads(STEREO_THREADS).build().expect("a thread pool"))
}

/// What the corridor along `yaw` holds, for tuning (`/v1/vr/corridor`):
/// per 10 cm from 0.3 m out, the points' count and lowest and highest
/// height over the floor (world metres), and what `corridor` makes of it.
pub fn corridor_report(frame: &EyeFrame, yaw: Option<f32>, metres: f32) -> anyhow::Result<Value> {
    let stereo = Stereo::from_frame(frame, vrc_stereo::match_scale(frame.width)).ok_or_else(|| anyhow::anyhow!("not an 8-bit frame"))?;
    let disp = stereo_pool().install(|| stereo.disparity(&SgmParams::default()));
    let points: Vec<[f32; 3]> = stereo.points(&disp, 2).into_iter().map(|(p, _)| p).collect();
    let eye = eye_of(frame);
    let yaw = yaw.unwrap_or_else(|| frame.views[0].pose.yaw_pitch().0);
    let (s, c) = yaw.to_radians().sin_cos();
    let mut bins: Vec<(usize, f32, f32)> = vec![(0, f32::INFINITY, f32::NEG_INFINITY); 40];
    for p in &points {
        let (dx, dz) = (p[0] - eye[0], p[2] - eye[2]);
        let (ahead, side, up) = ((dx * s - dz * c) * metres, (dx * c + dz * s) * metres, (p[1] - FLOOR_Y) * metres);
        if ahead > 0.3 && side.abs() < 0.25 {
            let i = ((ahead - 0.3) / 0.1) as usize;
            if let Some(b) = bins.get_mut(i) {
                (b.0, b.1, b.2) = (b.0 + 1, b.1.min(up), b.2.max(up));
            }
        }
    }
    let r = |v: f32| (v as f64 * 100.0).round() / 100.0;
    Ok(json!({
        "yaw": yaw.round(),
        "eye_m": r((eye[1] - FLOOR_Y) * metres),
        "points": points.len(),
        "blocker": corridor(&points, eye, yaw, metres, FLOOR_Y, FROM_ANYWAY).map(|b| json!({"distance": r(b.distance), "top": r(b.top), "tall": b.tall})),
        "bins": bins.iter().enumerate().filter(|(_, b)| b.0 > 0).map(|(i, b)| json!([r(0.3 + 0.1 * i as f32), b.0, r(b.1), r(b.2)])).collect::<Vec<_>>(),
    }))
}

/// Where the feet are under a name tag at `tag` (tracking space): the
/// lowest of the points within FEET_RADIUS_M round under it (their feet,
/// the floor they stand on, a stair's edge before them), over the bot's
/// floor (world metres); `None` with too few there.
fn feet_height(points: &[[f32; 3]], tag: [f32; 3], metres: f32, floor: f32) -> Option<f32> {
    let mut under: Vec<f32> = points
        .iter()
        .filter(|p| ((p[0] - tag[0]) * metres).hypot((p[2] - tag[2]) * metres) < FEET_RADIUS_M && (tag[1] - p[1]) * metres > 0.3)
        .map(|p| (p[1] - floor) * metres)
        .collect();
    if under.len() < FEET_POINTS {
        return None;
    }
    // Low, but not a stray point below the floor.
    under.sort_by(f32::total_cmp);
    Some(under[under.len() / 20])
}

/// The middle of the eyes of `frame` (tracking space, stereo units).
fn eye_of(frame: &EyeFrame) -> [f32; 3] {
    let (a, b) = (frame.views[0].pose.position, frame.views[1].pose.position);
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0]
}

/// The lens's latest read of `target`'s name since `since`.
fn lens_read(bridge: &Bridge, target: &str, since: Instant) -> Option<NameSighting> {
    bridge.orbit.names_since(since).into_iter().rev().find(|n| match_score(&n.name, target) >= 0.6)
}

/// The search's head views (tracking yaws): from `from` out either way by
/// turns, `way` (+1 right, -1 left) first, SEARCH_STEP_DEG apart.
fn search_views(from: f32, way: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|k| {
            let m = k.div_ceil(2) as f32 * SEARCH_STEP_DEG;
            wrap(from + if k % 2 == 1 { way * m } else { -way * m })
        })
        .collect()
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

    /// Decision D45: three head views round the front half, the lens
    /// opposite at each: six directions 60 degrees apart all round.
    #[test]
    fn the_search_and_the_lens_behind_cover_all_round() {
        let views = search_views(10.0, 1.0, SEARCH_VIEWS);
        assert_eq!(views, vec![10.0, 70.0, -50.0]);
        let mut all: Vec<f32> = views.iter().flat_map(|&y| [y, wrap(y + 180.0)]).map(|y| (y + 360.0) % 360.0).collect();
        all.sort_by(f32::total_cmp);
        for w in all.windows(2) {
            assert!((w[1] - w[0] - 60.0).abs() < 1e-3, "{all:?}");
        }
        assert_eq!(search_views(10.0, -1.0, SEARCH_VIEWS), vec![10.0, -50.0, 70.0]);
    }

    #[test]
    fn bearings() {
        assert!((bearing([0.0, 0.0], [0.0, -1.0])).abs() < 1e-4);
        assert!((bearing([0.0, 0.0], [1.0, 0.0]) - 90.0).abs() < 1e-4);
        assert!((wrap(190.0) + 170.0).abs() < 1e-4);
    }
}
