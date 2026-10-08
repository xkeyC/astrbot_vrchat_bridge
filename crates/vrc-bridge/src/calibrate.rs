//! Full body calibration by the bot itself (`/v1/vr/calibrate`): VRChat's
//! Quick Menu opened on the raised left hand, the right hand's ray on its
//! 校准 button, then standing straight and pulling both triggers
//! (`docs/full-vr/agent-vr-use.md`).
//!
//! Nothing is clicked on faith:
//!
//! - the button is found by OCR in the eye's frame (its text), not at a
//!   remembered spot;
//! - the right hand goes on the line of sight to it, turned by VRChat's
//!   pointer offset (measured), and a press waits for the button's hover
//!   tooltip (校准全身追踪) to show in a second OCR: aims below and around
//!   the text are tried until it does (the ray lands lower than the model
//!   says, by about 90 px at 1920);
//! - calibration mode is checked (the menu closed), and the result by
//!   VRChat's own `TrackingType` (6: head, hands, hip and feet).
//!
//! The trackers must be on: VRChat shows the button only for trackers it
//! sees. A calibration holds as long as the game runs (trackers stopped
//! and sent again keep it), so this is needed after VRChat starts.

use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use serde_json::{json, Value};
use vrc_players::ocr::OcrLine;
use vrc_vr::tap::EyeFrame;
use vrc_vr::Pose;

use crate::bridge::Bridge;
use crate::vr::VrCore;
use crate::Lock;

/// The head's pitch while the menu is open (degrees).
pub(crate) const MENU_PITCH: f32 = -20.0;
/// The left hand holding the menu up to the eyes: offset (right, up,
/// ahead of the eyes, metres) and turn (yaw, pitch, roll, degrees): the
/// wrist turned up so the menu faces the eyes.
pub(crate) const MENU_HAND: ([f32; 3], [f32; 3]) = ([-0.1, -0.3, 0.3], [0.0, 70.0, 0.0]);
/// Both hands down at the sides, as for standing straight.
pub(crate) const LEFT_DOWN: ([f32; 3], [f32; 3]) = ([-0.25, -0.85, 0.0], [0.0, -80.0, 0.0]);
pub(crate) const RIGHT_DOWN: ([f32; 3], [f32; 3]) = ([0.25, -0.85, 0.0], [0.0, -80.0, 0.0]);
/// VRChat's ray leaves the grip this much lower and to the left of its -Z
/// (degrees, measured): the grip is turned up and right by it.
const POINTER_PITCH: f32 = 28.0;
const POINTER_YAW: f32 = -3.2;
/// The right hand on the line of sight this far from the eye (metres):
/// the Quick Menu is about 0.3-0.4 m out.
const REACH: f32 = 0.25;
/// How long after aiming to look for a tooltip (it fades in), and again.
const TOOLTIP_WAITS: [Duration; 2] = [Duration::from_millis(350), Duration::from_millis(300)];
/// Aims tried around the button's text (pixels at 1920 wide; right, down),
/// likeliest first.
const AIMS: [(f32, f32); 9] =
    [(0.0, -90.0), (0.0, -45.0), (0.0, -135.0), (0.0, 0.0), (-30.0, -90.0), (30.0, -90.0), (0.0, -180.0), (-30.0, -45.0), (30.0, -45.0)];
/// The button's text, and its hover tooltip.
const BUTTON: [&str; 2] = ["校准", "Calibrate"];
const TOOLTIP: [&str; 2] = ["校准全身", "Calibrate Full"];
/// VRChat closes the menu this long after a press that entered
/// calibration mode, at most (it takes seconds).
const MENU_CLOSES: Duration = Duration::from_secs(8);
/// After a hand or the menu moves, until a frame shows it.
const SETTLE: Duration = Duration::from_millis(400);
/// VRChat's TrackingType with hip and feet trackers.
pub const FULL_BODY: i32 = 6;

/// One step of a routine with the hands (a calibration, opening the user
/// camera), for the report.
pub(crate) struct Log {
    pub started: Instant,
    pub steps: Vec<Value>,
    /// Names the routine in the bridge's log.
    pub what: &'static str,
}

impl Log {
    pub fn new(what: &'static str) -> Log {
        Log { started: Instant::now(), steps: Vec::new(), what }
    }

    pub fn step(&mut self, what: &str, detail: Value) {
        tracing::info!("{}: {what} {detail}", self.what);
        self.steps.push(json!({"t_s": self.took_s(), "step": what, "detail": detail}));
    }

    /// Seconds since the start, to a tenth.
    pub fn took_s(&self) -> f64 {
        (self.started.elapsed().as_secs_f64() * 10.0).round() / 10.0
    }
}

/// VRChat's TrackingType with head and hands only.
pub const HEAD_AND_HANDS: i32 = 3;
/// Trackers sent this long without VRChat taking them in full: it needs a
/// calibration (it takes a few seconds to report trackers it keeps).
const UNCALIBRATED_AFTER: Duration = Duration::from_secs(10);
/// Between two calibrations by itself.
const RETRY_AFTER: Duration = Duration::from_secs(120);
/// One calibration at a time.
static RUNNING: Mutex<()> = Mutex::new(());

/// Calibrates now (blocking): the trackers on (they must be seen for the
/// button to show), the follow paused, the hands off the animation, the
/// headset held throughout.
pub fn now(bridge: &Arc<Bridge>, force: bool) -> Result<Value> {
    let Some(_one) = RUNNING.try_lk() else { bail!("a calibration is running") };
    // The menu is found in the usual view: no panorama meanwhile.
    let _normal = bridge.pano.normal_view(Duration::from_secs(2))?;
    crate::motion::stop_and_wait(bridge);
    if !bridge.anim.trackers.lk().on {
        let mut settings = bridge.anim.trackers.lk().clone();
        settings.on = true;
        bridge.anim.set_trackers(settings)?;
    }
    while bridge.anim.trackers_on_for() < Duration::from_millis(1500) {
        std::thread::sleep(Duration::from_millis(100));
    }
    bridge.take_over();
    bridge.anim.manual_hands.store(true, Ordering::Relaxed);
    let report = {
        let mut vr = bridge.vr.lk();
        let r = run(&mut vr, bridge, force);
        if r.is_err() {
            vr.reset();
        }
        r
    };
    bridge.anim.manual_hands.store(false, Ordering::Relaxed);
    bridge.idle_later();
    report
}

/// Watches for a body VRChat no longer tracks in full though the trackers
/// are on (the game started again) and calibrates it (`auto_calibrate`),
/// pausing a follow meanwhile; checked every few seconds by the caller.
pub struct Watch {
    last_try: Option<Instant>,
}

impl Watch {
    pub fn new() -> Watch {
        Watch { last_try: None }
    }

    pub fn check(&mut self, bridge: &Arc<Bridge>) {
        let settings = bridge.anim.trackers.lk().clone();
        if !settings.on || !settings.auto_calibrate || bridge.anim.trackers_on_for() < UNCALIBRATED_AFTER {
            return;
        }
        // (A follow does not wait: calibrating pauses it, as the model's own
        // moves do, and it goes on after.)
        if self.last_try.is_some_and(|t| t.elapsed() < RETRY_AFTER) || !bridge.game.lk().running {
            return;
        }
        if tracking_type(bridge) != Some(HEAD_AND_HANDS) {
            return;
        }
        self.last_try = Some(Instant::now());
        tracing::info!("calibrate: VRChat tracks head and hands only though the trackers are on: calibrating");
        let bridge = bridge.clone();
        std::thread::spawn(move || match now(&bridge, false) {
            Ok(report) => tracing::info!("calibrate: {report}"),
            Err(e) => tracing::warn!("calibrate: {e:#}"),
        });
    }
}

/// VRChat's TrackingType now (OSCQuery).
pub fn tracking_type(bridge: &Bridge) -> Option<i32> {
    bridge.osc_query().ok()?.query("/avatar/parameters/TrackingType").ok().map(|v| v as i32)
}

/// Calibrates; the report of each step and the outcome. The caller holds
/// the headset (`vr`) and has the trackers on and the hands off the
/// animation.
/// Unless `force`, a body VRChat already tracks in full is left alone.
pub fn run(vr: &mut VrCore, bridge: &Bridge, force: bool) -> Result<Value> {
    let mut log = Log::new("calibrate");
    let yaw = vr.yaw;
    let before = tracking_type(bridge);
    if before == Some(FULL_BODY) && !force {
        return Ok(json!({"ok": true, "skipped": "already full body", "tracking_type": {"before": before, "after": before}}));
    }
    log.step("start", json!({"tracking_type": before, "facing_deg": yaw.round()}));
    let result = calibrate(vr, bridge, yaw, &mut log);
    // Whatever happened: the right hand off the menu, the menu closed if it
    // is still up, standing straight again.
    let _ = vr.release_hands();
    if result.is_err() {
        if let Ok(frame) = fresh_frame(vr) {
            if ocr(vr, &frame).map(|lines| find(&lines, &BUTTON).is_some()).unwrap_or(false) {
                toggle_menu(bridge);
            }
        }
    }
    let _ = vr.face(yaw, 0.0);
    let after = tracking_type(bridge);
    let ok = result.is_ok() && after == Some(FULL_BODY);
    let mut report = json!({
        "ok": ok,
        "tracking_type": {"before": before, "after": after},
        // From 3 to 6 proves it; a body already at 6 stays there either way.
        "verified": ok && before != Some(FULL_BODY),
        "took_s": log.took_s(),
        "steps": log.steps,
    });
    match result {
        Err(e) => report["error"] = json!(format!("{e:#}")),
        Ok(()) if !ok => report["error"] = json!("calibrated, but VRChat does not report full body tracking"),
        Ok(()) => {}
    }
    Ok(report)
}

fn calibrate(vr: &mut VrCore, bridge: &Bridge, yaw: f32, log: &mut Log) -> Result<()> {
    // The menu held up, the right hand down out of it.
    vr.aim(yaw, MENU_PITCH)?;
    vr.set_hand("left", MENU_HAND.0, MENU_HAND.1, 0.0, &[])?;
    vr.set_hand("right", RIGHT_DOWN.0, RIGHT_DOWN.1, 0.0, &[])?;
    let button = open_menu(vr, bridge, &BUTTON, log).context("the Quick Menu shows no 校准 button (are the trackers on?)")?;
    // A press that took leaves the menu to VRChat, which closes it a few
    // seconds later (the calibration mirror shows meanwhile); one that did
    // not leaves it up: aim and press again.
    let mut entered = false;
    for press in 1..=2 {
        let grip = hover(vr, button, log)?;
        vr.set_hand_pose("right", grip, 1.0)?;
        std::thread::sleep(Duration::from_millis(150));
        vr.set_hand_pose("right", grip, 0.0)?;
        log.step("press", json!({"press": press}));
        let deadline = Instant::now() + MENU_CLOSES;
        while Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(800));
            let frame = vr.frame()?;
            if find(&ocr(vr, &frame)?, &BUTTON).is_none() {
                entered = true;
                break;
            }
        }
        if entered {
            break;
        }
    }
    if !entered {
        bail!("pressed 校准 twice, but the Quick Menu stayed open: not in calibration mode");
    }
    log.step("calibration mode", json!({}));
    // Standing straight, looking ahead, hands at the sides; a moment for
    // VRChat to settle the body on the trackers, then both triggers.
    vr.aim(yaw, 0.0)?;
    vr.set_hand("left", LEFT_DOWN.0, LEFT_DOWN.1, 0.0, &[])?;
    vr.set_hand("right", RIGHT_DOWN.0, RIGHT_DOWN.1, 0.0, &[])?;
    std::thread::sleep(Duration::from_millis(2000));
    vr.set_hand("left", LEFT_DOWN.0, LEFT_DOWN.1, 1.0, &[])?;
    vr.set_hand("right", RIGHT_DOWN.0, RIGHT_DOWN.1, 1.0, &[])?;
    std::thread::sleep(Duration::from_millis(400));
    vr.release_hands()?;
    log.step("triggers", json!({}));
    // VRChat reports full body within a second or two.
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        if tracking_type(bridge) == Some(FULL_BODY) {
            log.step("full body", json!({"tracking_type": FULL_BODY}));
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(300));
    }
    bail!("VRChat did not report full body tracking after calibrating")
}

/// Aims the right hand's ray at the button (its text at `button`, pixels)
/// until its tooltip shows: the grip that does.
fn hover(vr: &mut VrCore, button: (f32, f32), log: &mut Log) -> Result<Pose> {
    hover_until(vr, button, &TOOLTIP, log).context("no aim at the 校准 button showed its tooltip: not pressed")
}

/// Aims the right hand's ray around `button` (pixels of the left eye; the
/// tries' offsets, `AIMS`, are at 1920 wide and scaled; the menu stays put
/// while the left hand does) until a line starting with one of `tooltips`
/// shows: the grip that does.
pub(crate) fn hover_until(vr: &mut VrCore, button: (f32, f32), tooltips: &[&str], log: &mut Log) -> Result<Pose> {
    let frame = fresh_frame(vr)?;
    // The menu hangs on the left hand: where its button is now holds while
    // the hands stay put.
    let scale = frame.width as f32 / 1920.0;
    for (i, (du, dv)) in AIMS.iter().enumerate() {
        let (u, v) = (button.0 + du * scale, button.1 + dv * scale);
        let grip = grip_at_pixel(&frame, u, v, REACH);
        vr.set_hand_pose("right", grip, 0.0)?;
        // A tooltip fades in a few frames after the ray reaches its button:
        // a look at once misses it, so look a little later, and once more.
        let mut tip = None;
        for wait in TOOLTIP_WAITS {
            std::thread::sleep(wait);
            let seen = fresh_frame(vr)?;
            let lines = ocr(vr, &seen)?;
            if let Some(line) = find_start(&lines, tooltips) {
                tip = Some(line.clone());
                break;
            }
            // The Quick Menu's tooltips hang tilted below their button: OCR
            // finds no line in the whole frame (measured 2026-10-09: the
            // camera's, about 12 degrees), but does once it is cut out,
            // turned level and enlarged.
            if let Some(line) = tooltip_below(vr, &seen, u, v, tooltips)? {
                tip = Some(line);
                break;
            }
        }
        log.step("aim", json!({"try": i + 1, "pixel": [u.round(), v.round()], "tooltip": tip.as_ref().map(|l| l.text.clone())}));
        if tip.is_some() {
            return Ok(grip);
        }
    }
    bail!("no aim showed a tooltip starting with any of {tooltips:?}")
}

/// Opens the Quick Menu (unless it is open): the centre of the first line
/// that is one of `labels` (a button's text) in the left eye (pixels).
pub(crate) fn open_menu(vr: &mut VrCore, bridge: &Bridge, labels: &[&str], log: &mut Log) -> Result<(f32, f32)> {
    // The menu may be open already, and the toggle would close it: look
    // first, toggle, look again (twice: a toggle can close a menu the
    // first look missed).
    for attempt in 0..3 {
        let frame = fresh_frame(vr)?;
        let lines = ocr(vr, &frame)?;
        if let Some(line) = find(&lines, labels) {
            let [x, y, w, h] = line.bbox;
            let at = (x + w / 2.0, y + h / 2.0);
            log.step("menu open", json!({"button": line.text, "pixel": [at.0.round(), at.1.round()], "toggles": attempt}));
            return Ok(at);
        }
        if attempt < 2 {
            toggle_menu(bridge);
            std::thread::sleep(Duration::from_millis(800));
        }
    }
    bail!("the Quick Menu shows none of {labels:?}")
}

pub(crate) fn toggle_menu(bridge: &Bridge) {
    let _ = bridge.osc.send_i32("/input/QuickMenuToggleLeft", 1);
    std::thread::sleep(Duration::from_millis(150));
    let _ = bridge.osc.send_i32("/input/QuickMenuToggleLeft", 0);
}

/// A frame rendered after what was just sent.
pub(crate) fn fresh_frame(vr: &mut VrCore) -> Result<EyeFrame> {
    std::thread::sleep(SETTLE);
    vr.frame()
}

/// Turns tried to level a tooltip below a Quick Menu button (degrees,
/// clockwise), likeliest first.
const TOOLTIP_TURNS: [f32; 3] = [12.0, 8.0, 16.0];

/// A tooltip starting with one of `tooltips` read in the region below the
/// aim (`u`, `v`: left-eye pixels), turned level by each of TOOLTIP_TURNS.
fn tooltip_below(vr: &mut VrCore, frame: &EyeFrame, u: f32, v: f32, tooltips: &[&str]) -> Result<Option<OcrLine>> {
    let rig = vr.rig(&[])?;
    let ocr = rig.ocr.clone().context("working the menus needs OCR")?;
    let rgb = frame.eye_rgb8(0)?;
    let s = frame.width as f32 / 1920.0;
    let (w, h) = (frame.width as usize, frame.height as usize);
    // Below and to the left of the button, where the tooltip bar hangs
    // (1920-wide pixels, scaled).
    let x0 = ((u - 450.0 * s).max(0.0)) as usize;
    let y0 = ((v + 150.0 * s).max(0.0)) as usize;
    let x1 = ((u + 350.0 * s) as usize).min(w);
    let y1 = ((v + 450.0 * s) as usize).min(h);
    if x1 <= x0 + 8 || y1 <= y0 + 8 {
        return Ok(None);
    }
    for turn in TOOLTIP_TURNS {
        let (img, iw, ih) = level_crop(&rgb, w, x0, y0, x1, y1, turn, 2);
        let lines = ocr.lines_rgb(&img, iw as u16, ih as u16)?;
        if let Some(line) = find_start(&lines, tooltips) {
            return Ok(Some(line.clone()));
        }
    }
    Ok(None)
}

/// The region `x0..x1` x `y0..y1` of an RGB8 image `width` wide, turned
/// `clockwise` degrees about its centre and enlarged `scale` times
/// (bilinear; outside it black). With its width and height.
fn level_crop(rgb: &[u8], width: usize, x0: usize, y0: usize, x1: usize, y1: usize, clockwise: f32, scale: usize) -> (Vec<u8>, usize, usize) {
    let (cw, ch) = (x1 - x0, y1 - y0);
    let (ow, oh) = (cw * scale, ch * scale);
    let (cx, cy) = (cw as f32 / 2.0, ch as f32 / 2.0);
    let (sin, cos) = clockwise.to_radians().sin_cos();
    let mut out = vec![0u8; ow * oh * 3];
    for oy in 0..oh {
        for ox in 0..ow {
            // The output pixel back into the crop: undo the turn (y down,
            // so this matrix turns clockwise on screen).
            let dx = (ox as f32 + 0.5) / scale as f32 - cx;
            let dy = (oy as f32 + 0.5) / scale as f32 - cy;
            let sx = cx + cos * dx + sin * dy - 0.5;
            let sy = cy - sin * dx + cos * dy - 0.5;
            if sx < 0.0 || sy < 0.0 || sx >= (cw - 1) as f32 || sy >= (ch - 1) as f32 {
                continue;
            }
            let (ix, iy) = (sx as usize, sy as usize);
            let (fx, fy) = (sx - ix as f32, sy - iy as f32);
            for c in 0..3 {
                let at = |x: usize, y: usize| rgb[((y0 + y) * width + x0 + x) * 3 + c] as f32;
                let top = at(ix, iy) * (1.0 - fx) + at(ix + 1, iy) * fx;
                let bottom = at(ix, iy + 1) * (1.0 - fx) + at(ix + 1, iy + 1) * fx;
                out[(oy * ow + ox) * 3 + c] = (top * (1.0 - fy) + bottom * fy).round() as u8;
            }
        }
    }
    (out, ow, oh)
}

pub(crate) fn ocr(vr: &mut VrCore, frame: &EyeFrame) -> Result<Vec<OcrLine>> {
    let rig = vr.rig(&[])?;
    let ocr = rig.ocr.clone().context("working the menus needs OCR")?;
    let rgb = frame.eye_rgb8(0)?;
    ocr.lines_rgb(&rgb, frame.width as u16, frame.height as u16)
}

fn squeeze(s: &str) -> String {
    s.chars().filter(|c| !c.is_whitespace()).collect::<String>().to_lowercase()
}

/// The first line that is one of `texts` (ignoring spaces and case).
pub(crate) fn find<'a>(lines: &'a [OcrLine], texts: &[&str]) -> Option<&'a OcrLine> {
    lines.iter().find(|l| texts.iter().any(|want| squeeze(&l.text) == squeeze(want)))
}

/// The first line that starts with one of `texts`.
pub(crate) fn find_start<'a>(lines: &'a [OcrLine], texts: &[&str]) -> Option<&'a OcrLine> {
    lines.iter().find(|l| texts.iter().any(|want| squeeze(&l.text).starts_with(&squeeze(want))))
}

/// The right hand's grip that puts VRChat's ray through pixel (`u`, `v`)
/// of the left eye: on the line of sight `reach` metres out, turned along
/// it by the pointer's offset.
pub fn grip_at_pixel(frame: &EyeFrame, u: f32, v: f32, reach: f32) -> Pose {
    let view = &frame.views[0];
    let [fx, fy, cx, cy] = view.fov.intrinsics(frame.width, frame.height);
    let d = [(u - cx) / fx, -(v - cy) / fy, -1.0];
    let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let dir = view.pose.rotate([d[0] / n, d[1] / n, d[2] / n]);
    let eye = view.pose.position;
    let position = [eye[0] + dir[0] * reach, eye[1] + dir[1] * reach, eye[2] + dir[2] * reach];
    let yaw = dir[0].atan2(-dir[2]).to_degrees();
    let pitch = dir[1].clamp(-1.0, 1.0).asin().to_degrees();
    Pose::looking(yaw + POINTER_YAW, pitch + POINTER_PITCH, position)
}

#[cfg(test)]
mod tests {
    use super::*;
    use vrc_vr::tap::EyeView;
    use vrc_vr::Fov;

    fn frame(yaw: f32, pitch: f32) -> EyeFrame {
        let a = 50f32.to_radians();
        let view = EyeView { fov: Fov { left: -a, right: a, up: a, down: -a }, pose: Pose::looking(yaw, pitch, [0.0, 1.5, 0.0]) };
        EyeFrame {
            seq: 0,
            frame_id: 0,
            display_time_ns: 0,
            capture_ns: 0,
            width: 1920,
            height: 1920,
            format: 0,
            bytes_per_pixel: 4,
            views: [view, view],
            pixels: Vec::new(),
        }
    }

    #[test]
    fn the_centre_pixel_points_where_the_eye_looks() {
        let f = frame(30.0, -20.0);
        let grip = grip_at_pixel(&f, 960.0, 960.0, 0.25);
        let (y, p) = grip.yaw_pitch();
        assert!((y - (30.0 + POINTER_YAW)).abs() < 0.01 && (p - (-20.0 + POINTER_PITCH)).abs() < 0.01, "{y} {p}");
        // A quarter metre out along the view.
        let ahead = Pose::looking(30.0, -20.0, [0.0; 3]).rotate([0.0, 0.0, -0.25]);
        for i in 0..3 {
            assert!((grip.position[i] - [0.0, 1.5, 0.0][i] - ahead[i]).abs() < 1e-4);
        }
    }

    #[test]
    fn a_pixel_right_and_below_turns_right_and_down() {
        let f = frame(0.0, 0.0);
        let (y, p) = grip_at_pixel(&f, 960.0 + 805.5, 960.0 + 805.5, 0.25).yaw_pitch();
        // 45 degrees right; down by atan(1/sqrt(2)).
        assert!((y - (45.0 + POINTER_YAW)).abs() < 0.2, "{y}");
        assert!((p - (-35.26 + POINTER_PITCH)).abs() < 0.3, "{p}");
    }

    #[test]
    fn a_tilted_line_is_turned_level() {
        // A bright line rising to the right at 12 degrees (y down) in a dark
        // 200 x 100 image; turned 12 degrees clockwise it lies level.
        let (w, h) = (200usize, 100usize);
        let mut rgb = vec![0u8; w * h * 3];
        let slope = 12f32.to_radians().tan();
        for x in 20..180 {
            let y = (50.0 - (x as f32 - 100.0) * slope).round() as usize;
            for c in 0..3 {
                rgb[(y * w + x) * 3 + c] = 255;
            }
        }
        let (out, ow, oh) = level_crop(&rgb, w, 0, 0, w, h, 12.0, 1);
        assert_eq!((ow, oh), (w, h));
        // The brightest row of the middle columns, at their ends alike.
        let row_of = |x: usize| (0..oh).max_by_key(|&y| out[(y * ow + x) * 3]).unwrap() as i32;
        assert!((row_of(50) - row_of(150)).abs() <= 2, "{} {}", row_of(50), row_of(150));
    }

    #[test]
    fn finds_the_button_and_the_tooltip_apart() {
        let line = |t: &str| OcrLine { text: t.into(), confidence: 0.9, bbox: [0.0; 4] };
        let lines = vec![line("回出生点"), line("校准全身追踪"), line("校 准")];
        assert_eq!(find(&lines, &BUTTON).unwrap().text, "校 准");
        assert_eq!(find_start(&lines, &TOOLTIP).unwrap().text, "校准全身追踪");
        assert!(find(&[line("校准全身追踪")], &BUTTON).is_none());
    }
}
