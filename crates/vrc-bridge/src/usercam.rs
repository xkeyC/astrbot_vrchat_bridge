//! VRChat's own user camera as a remote eye (`/v1/vr/usercam`,
//! `docs/full-vr/agent-vr-use.md` section 7, decision D34): placed anywhere
//! in the world by OSC (`/usercamera/Pose`), it renders what the bot's
//! avatar cameras cannot, the nameplates (VRChat draws them in its own UI
//! camera only, and the camera's UI mask, `/usercamera/ShowUIInCamera`,
//! adds them). In stream mode the game's desktop window shows the camera's
//! view: a grab of it is the remote eye's frame.
//!
//! There is no OSC to open the camera: the bot opens it as it calibrates
//! full body (`calibrate`), with the same reproducible posture: head
//! pitched -20, the Quick Menu on the raised left hand, the right hand's
//! ray on the menu's camera icon (it has no label: found from the 回出生点
//! button's text by OCR and confirmed by its hover tooltip, 查看相机选项),
//! a double-click, and `/usercamera/Mode` (OSCQuery) leaving 0 proves it.
//! Then:
//!
//! - the viewfinder (it spawns by the right hand, a pickup fixed in the
//!   tracking space) is grabbed with the grip and dragged into the body:
//!   out of the eyes and the avatar's panorama cameras (the user's call);
//! - stream mode and the UI mask by OSC;
//! - flying mode off: `/usercamera/Pose` turns it on, and while it is on
//!   the movement inputs fly the camera, not the bot (measured: a step
//!   went nowhere and the camera flew into the bot's face), so every pose
//!   is followed by `/usercamera/Flying` false;
//! - a check that the desktop shows the camera: two poses (ahead, behind)
//!   must give different grabs.
//!
//! Camera settings the game keeps (the lens hidden from the bot itself;
//! the follow mode 玩家位置: with flying off the lens rides along with the
//! bot's position, keeping its world yaw) were set once in its own UI and
//! are not touched here. While the bot stands the lens orbits it, and
//! while it moves it looks the way the bot goes (`crate::orbit`).

use std::io::Read;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use vrc_vr::osc::{encode, Arg, Osc};
use vrc_vr::Pose;

use crate::bridge::Bridge;
use crate::calibrate::{self, Log};
use crate::vr::VrCore;
use crate::Lock;

/// `/usercamera/Mode` (measured 2026-10-08): closed, photo, stream.
pub const MODE_CLOSED: i32 = 0;
pub const MODE_PHOTO: i32 = 1;
pub const MODE_STREAM: i32 = 2;
/// The Quick Menu's 回出生点 (respawn) button: the camera icon is found
/// from its text.
const ANCHOR: [&str; 2] = ["回出生点", "Respawn"];
/// The camera icon in the menu's bottom bar from the centre of 回出生点's
/// text (pixels at 1920; measured: (78..85, 36..56)).
const ICON_FROM_ANCHOR: (f32, f32) = (80.0, 48.0);
/// The icon's hover tooltip: 查看相机选项，双击可唤出相机。 (the English one
/// is a guess).
const ICON_TOOLTIP: [&str; 2] = ["查看相机", "View Camera"];
/// The viewfinder's button row (any page of it) and its mode label: one of
/// them read in the left eye means the viewfinder is in view.
const VIEWFINDER: [&str; 16] = [
    "拍摄照片", "倒计时(5秒)", "拍照模式", "Spout串流", "相机作为录音源", "飞行模式", "跟随方式", "辅助功能", "对焦", "机位", "图层",
    "运镜", "镜头可见度", "滤镜", "分辨率", "直播",
];
/// The double-click: two presses this long, this far apart (measured).
const CLICK: Duration = Duration::from_millis(80);
const CLICK_GAP: Duration = Duration::from_millis(150);
/// The camera opens (Mode leaves 0) this soon after a double-click.
const OPENS_WITHIN: Duration = Duration::from_secs(3);
/// The viewfinder into the body: the right hand grabs it where it spawned
/// (the hand is in its reach right after the double-click: it glows) and
/// moves this far (metres: right, up, ahead of where the body faces), from
/// the viewfinder's middle (about (0.055, -0.30, 0.32) from the eyes) into
/// the chest (about (0, -0.38, 0)). An estimate: see the docs.
pub const STOW_DELTA: [f32; 3] = [-0.055, -0.08, -0.32];
/// The drag: steps, and the time each takes.
const DRAG_STEPS: usize = 12;
const DRAG_STEP: Duration = Duration::from_millis(50);
/// A grip held still this long before moving or after letting go.
const GRIP_SETTLE: Duration = Duration::from_millis(300);
/// After `/usercamera/Pose`: VRChat applies it (and turns flying on) within
/// a frame or two; flying off sent sooner is undone (measured: off sent
/// at once stayed on, 150 ms later it held).
const POSE_TAKES: Duration = Duration::from_millis(150);
/// After flying off, until the desktop shows the pose.
const GRAB_AFTER: Duration = Duration::from_millis(100);
/// Two grabs of the camera turned ahead and behind differ at least this
/// much (mean absolute difference, 0..255) when the desktop shows it.
const DESKTOP_DIFF_MIN: f32 = 10.0;
/// The camera above the head (metres, world) for the check, a shot's
/// default and the sweep's.
pub const ABOVE_HEAD_M: f32 = 0.3;
/// The sweep's tiles (pixels).
const TILE: (usize, usize) = (640, 360);

/// After a world is joined (or the game started), this long before the
/// camera is opened again by itself (the avatar loads, menus settle).
const SETTLE_AFTER_JOIN: Duration = Duration::from_secs(30);
/// Between two openings by itself.
const RETRY_AFTER: Duration = Duration::from_secs(120);

/// One routine with the camera at a time (opening, shots); the orbit
/// (`crate::orbit`) leaves the camera alone while one holds it.
pub(crate) static RUNNING: Mutex<()> = Mutex::new(());

// -- poses ------------------------------------------------------------------------

/// The camera's pose as `/usercamera/Pose` takes it: Unity's world
/// position (metres), then pitch (+ down), yaw (clockwise from +z seen
/// from above, as the position beacon's), roll (degrees).
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct CamPose {
    pub position: [f32; 3],
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
}

impl CamPose {
    pub fn from_args(a: [f32; 6]) -> CamPose {
        CamPose { position: [a[0], a[1], a[2]], pitch: a[3], yaw: a[4].rem_euclid(360.0), roll: a[5] }
    }

    pub fn args(&self) -> [f32; 6] {
        [self.position[0], self.position[1], self.position[2], self.pitch, self.yaw, self.roll]
    }

    /// The OSC message.
    pub fn message(&self) -> Vec<u8> {
        encode("/usercamera/Pose", &self.args().map(Arg::Float))
    }
}

/// Which way a camera placed round the head looks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Look {
    /// Away from the head, along the bearing.
    Out,
    /// Back at the head (the bot itself and what is behind it).
    Back,
}

/// A camera `distance` m from the eyes at `eye` (world) along `bearing`
/// degrees right of `heading` (the head's yaw, the beacon's), `height` m
/// above them; looking `Out` (tilted `pitch_up` degrees up) or `Back` at
/// the eyes.
pub fn around(eye: [f32; 3], heading: f32, bearing: f32, distance: f32, height: f32, look: Look, pitch_up: f32) -> CamPose {
    let way = heading + bearing;
    let (s, c) = way.to_radians().sin_cos();
    let position = [eye[0] + distance * s, eye[1] + height, eye[2] + distance * c];
    let (yaw, pitch) = match look {
        Look::Out => (way, -pitch_up),
        Look::Back => (way + 180.0, height.atan2(distance).to_degrees()),
    };
    CamPose { position, pitch, yaw: yaw.rem_euclid(360.0), roll: 0.0 }
}

/// Where the eyes are (world, between the two) and the head's yaw, from
/// the position beacon in the latest frame.
fn head(vr: &mut VrCore) -> Result<([f32; 3], f32)> {
    let frame = vr.frame()?;
    let read: Vec<_> = (0..2).filter_map(|i| vrc_vr::beacon::read(&frame, i)).collect();
    let first = read.first().context("no position beacon in the eyes (the avatar's PosBeacon)")?;
    let n = read.len() as f32;
    let mut eye = [0.0; 3];
    for b in &read {
        for (k, v) in eye.iter_mut().enumerate() {
            *v += b.position[k] / n;
        }
    }
    Ok((eye, first.yaw))
}

// -- OSC -----------------------------------------------------------------------------

fn send_bool(osc: &Osc, address: &str, on: bool) -> Result<()> {
    osc.send(address, &[Arg::Bool(on)])
}

/// `/usercamera/Mode` now; none when OSCQuery does not answer.
pub fn mode(osc: &Osc) -> Option<i32> {
    osc.query("/usercamera/Mode").ok().map(|v| v as i32)
}

pub fn mode_name(mode: Option<i32>) -> &'static str {
    match mode {
        Some(MODE_CLOSED) => "closed",
        Some(MODE_PHOTO) => "photo",
        Some(MODE_STREAM) => "stream",
        Some(_) => "other",
        None => "unknown",
    }
}

fn wait_mode(osc: &Osc, within: Duration, ok: impl Fn(i32) -> bool) -> Option<i32> {
    let deadline = Instant::now() + within;
    loop {
        let m = mode(osc);
        if m.is_some_and(&ok) || Instant::now() >= deadline {
            return m;
        }
        std::thread::sleep(Duration::from_millis(150));
    }
}

/// Places the camera and turns flying off again (it takes the movement
/// inputs: the bot could not walk). The orbit is told (it tags frames by
/// the poses sent).
pub fn place(bridge: &Bridge, osc: &Osc, pose: &CamPose) -> Result<()> {
    osc.send_raw(&pose.message())?;
    bridge.orbit.note_pose(*pose);
    std::thread::sleep(POSE_TAKES);
    for _ in 0..3 {
        send_bool(osc, "/usercamera/Flying", false)?;
        std::thread::sleep(GRAB_AFTER);
        if osc.query("/usercamera/Flying").map(|v| v == 0.0).unwrap_or(true) {
            bridge.orbit.note_flying_off();
            return Ok(());
        }
    }
    bail!("the camera stays in flying mode (it would take the bot's walking)")
}

/// Stream mode (the desktop shows the camera) and the UI mask (the
/// nameplates in its view), flying off; checked by OSCQuery.
fn stream_mode(osc: &Osc) -> Result<Value> {
    osc.send("/usercamera/Mode", &[Arg::Int(MODE_STREAM)])?;
    send_bool(osc, "/usercamera/ShowUIInCamera", true)?;
    // Not the bot itself: its head blocked the lens's view behind it, and
    // put at the eyes the lens saw the inside of it.
    send_bool(osc, "/usercamera/LocalPlayer", false)?;
    std::thread::sleep(Duration::from_millis(300));
    send_bool(osc, "/usercamera/Flying", false)?;
    std::thread::sleep(Duration::from_millis(150));
    let q = |a: &str| osc.query(a).ok();
    let m = mode(osc);
    let ui = q("/usercamera/ShowUIInCamera");
    let flying = q("/usercamera/Flying");
    if m != Some(MODE_STREAM) {
        bail!("the camera is not in stream mode (Mode {m:?})");
    }
    Ok(json!({"mode": m, "show_ui": ui.map(|v| v != 0.0), "flying": flying.map(|v| v != 0.0)}))
}

// -- the desktop -------------------------------------------------------------------

/// An RGB image.
pub struct Rgb {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl Rgb {
    pub fn jpeg(&self) -> Result<Vec<u8>> {
        let mut out = Vec::new();
        jpeg_encoder::Encoder::new(&mut out, 85).encode(&self.data, self.width as u16, self.height as u16, jpeg_encoder::ColorType::Rgb)?;
        Ok(out)
    }

    /// Box-averaged down to `w` x `h`.
    pub fn scaled(&self, w: usize, h: usize) -> Rgb {
        let mut data = vec![0u8; w * h * 3];
        for y in 0..h {
            let (y0, y1) = (y * self.height / h, ((y + 1) * self.height / h).max(y * self.height / h + 1));
            for x in 0..w {
                let (x0, x1) = (x * self.width / w, ((x + 1) * self.width / w).max(x * self.width / w + 1));
                let mut sum = [0u32; 3];
                for yy in y0..y1.min(self.height) {
                    for xx in x0..x1.min(self.width) {
                        for k in 0..3 {
                            sum[k] += self.data[(yy * self.width + xx) * 3 + k] as u32;
                        }
                    }
                }
                let n = ((x1.min(self.width) - x0) * (y1.min(self.height) - y0)).max(1) as u32;
                for k in 0..3 {
                    data[(y * w + x) * 3 + k] = (sum[k] / n) as u8;
                }
            }
        }
        Rgb { width: w, height: h, data }
    }

    /// Mean absolute difference from `other` (0..255), on a coarse grid of
    /// both at the same small size.
    pub fn diff(&self, other: &Rgb) -> f32 {
        let (a, b) = (self.scaled(64, 36), other.scaled(64, 36));
        let total: u64 = a.data.iter().zip(&b.data).map(|(x, y)| (*x as i32 - *y as i32).unsigned_abs() as u64).sum();
        total as f32 / a.data.len() as f32
    }
}

/// Tiles in rows of `cols`, each `TILE` (black where none is left).
pub fn mosaic(tiles: &[Rgb], cols: usize) -> Rgb {
    let cols = cols.clamp(1, tiles.len().max(1));
    let rows = tiles.len().div_ceil(cols).max(1);
    let (w, h) = (TILE.0 * cols, TILE.1 * rows);
    let mut data = vec![0u8; w * h * 3];
    for (i, t) in tiles.iter().enumerate() {
        let t = t.scaled(TILE.0, TILE.1);
        let (ox, oy) = ((i % cols) * TILE.0, (i / cols) * TILE.1);
        for y in 0..TILE.1 {
            let at = ((oy + y) * w + ox) * 3;
            data[at..at + TILE.0 * 3].copy_from_slice(&t.data[y * TILE.0 * 3..(y + 1) * TILE.0 * 3]);
        }
    }
    Rgb { width: w, height: h, data }
}

/// `WxH+X,Y` (`--desktop-grab`).
pub fn parse_grab(spec: &str) -> Result<(usize, usize, i32, i32)> {
    let bad = || anyhow::anyhow!("--desktop-grab is WxH+X,Y, not {spec}");
    let (size, at) = spec.split_once('+').ok_or_else(bad)?;
    let (w, h) = size.split_once('x').ok_or_else(bad)?;
    let (x, y) = at.split_once(',').ok_or_else(bad)?;
    let (w, h): (usize, usize) = (w.trim().parse()?, h.trim().parse()?);
    if w == 0 || h == 0 || w > 7680 || h > 4320 {
        return Err(bad());
    }
    Ok((w, h, x.trim().parse()?, y.trim().parse()?))
}

/// ffmpeg grabbing the game's desktop window (`--desktop-grab` on
/// `display`) as raw RGB on its output: `input` before the input (a frame
/// rate), `frames` of them (none: a stream). With its width and height.
pub fn x11grab(display: &str, spec: &str, input: &[&str], frames: Option<u32>) -> Result<(Command, usize, usize)> {
    let (w, h, x, y) = parse_grab(spec)?;
    let mut cmd = Command::new("ffmpeg");
    cmd.args(["-loglevel", "error", "-f", "x11grab"])
        .args(input)
        .args(["-video_size", &format!("{w}x{h}"), "-i", &format!("{display}+{x},{y}")]);
    if let Some(n) = frames {
        cmd.args(["-frames:v", &n.to_string()]);
    }
    cmd.args(["-f", "rawvideo", "-pix_fmt", "rgb24", "-"]).env("DISPLAY", display);
    if std::env::var_os("XAUTHORITY").is_none() {
        if let Some(home) = std::env::var_os("HOME") {
            let xauth = PathBuf::from(home).join(".Xauthority");
            if xauth.exists() {
                cmd.env("XAUTHORITY", xauth);
            }
        }
    }
    Ok((cmd, w, h))
}

/// One frame of the game's desktop window (in stream mode: the camera's
/// view), by ffmpeg's x11grab.
pub fn grab_desktop(display: &str, spec: &str) -> Result<Rgb> {
    let (mut cmd, w, h) = x11grab(display, spec, &[], Some(1))?;
    cmd.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
    let mut child = cmd.spawn().context("running ffmpeg")?;
    // The frame (megabytes) is read as it comes: a pipe left unread fills
    // and ffmpeg never ends. A hung grab must not hold the camera.
    let mut stdout = child.stdout.take().context("ffmpeg's output")?;
    let mut stderr = child.stderr.take().context("ffmpeg's errors")?;
    let reader = std::thread::spawn(move || {
        let mut data = Vec::new();
        let _ = stdout.read_to_end(&mut data);
        data
    });
    let errors = std::thread::spawn(move || {
        let mut text = String::new();
        let _ = stderr.read_to_string(&mut text);
        text
    });
    let deadline = Instant::now() + Duration::from_secs(5);
    let status = loop {
        if let Some(status) = child.try_wait()? {
            break status;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            bail!("ffmpeg took too long to grab the desktop");
        }
        std::thread::sleep(Duration::from_millis(20));
    };
    let data = reader.join().unwrap_or_default();
    let errors = errors.join().unwrap_or_default();
    if !status.success() || data.len() < w * h * 3 {
        bail!("grabbing the desktop failed: {}", errors.trim());
    }
    Ok(Rgb { width: w, height: h, data: data[..w * h * 3].to_vec() })
}

// -- settings and status --------------------------------------------------------

/// Kept next to the token (`usercam.json`).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Opened again by itself when it is found closed (the game started
    /// again, a world was joined) while the game runs.
    pub keep_open: bool,
    /// The viewfinder dragged into the body after opening.
    pub stow: bool,
    /// The lens round the bot (`crate::orbit`).
    pub orbit: crate::orbit::OrbitSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { keep_open: false, stow: true, orbit: Default::default() }
    }
}

pub struct UserCam {
    path: PathBuf,
    pub settings: Mutex<Settings>,
    /// The last opening's report.
    pub last_open: Mutex<Option<Value>>,
    /// The last pose sent.
    pub last_pose: Mutex<Option<CamPose>>,
}

impl UserCam {
    pub fn new(path: PathBuf) -> UserCam {
        let settings = std::fs::read(&path)
            .ok()
            .and_then(|b| serde_json::from_slice::<Settings>(&b).map_err(|e| tracing::warn!("{}: {e}", path.display())).ok())
            .unwrap_or_default();
        UserCam { path, settings: Mutex::new(settings), last_open: Mutex::new(None), last_pose: Mutex::new(None) }
    }

    pub fn set(&self, settings: Settings) -> Result<()> {
        std::fs::write(&self.path, serde_json::to_vec_pretty(&settings)?)?;
        *self.settings.lk() = settings;
        Ok(())
    }
}

/// The camera as VRChat reports it (OSCQuery), and the bridge's side.
pub fn status(bridge: &Bridge) -> Value {
    let cam = &bridge.usercam;
    let game = bridge.osc_query().ok();
    let q = |a: &str| game.as_ref().and_then(|o| o.query(a).ok());
    let flag = |a: &str| q(a).map(|v| v != 0.0);
    let m = game.as_ref().and_then(mode);
    json!({
        "mode": m,
        "state": mode_name(m),
        "open": m.map(|m| m != MODE_CLOSED),
        "flying": flag("/usercamera/Flying"),
        "show_ui": flag("/usercamera/ShowUIInCamera"),
        "lock": flag("/usercamera/Lock"),
        "fling_to_close": flag("/usercamera/FlingToClose"),
        "settings": cam.settings.lk().clone(),
        "last_pose": *cam.last_pose.lk(),
        "last_open": cam.last_open.lk().clone(),
        "desktop_grab": bridge.args.desktop_grab,
        "orbit": bridge.orbit.status(),
    })
}

// -- opening ------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct OpenOptions {
    pub stow: bool,
    pub stow_delta: [f32; 3],
    pub check: bool,
}

/// Opens the camera and sets it up (blocking): the follow paused, the
/// hands off the animation, the headset held throughout.
pub fn open(bridge: &Arc<Bridge>, opts: &OpenOptions) -> Result<Value> {
    let Some(_one) = RUNNING.try_lk() else { bail!("the user camera is busy (being opened, or a shot)") };
    crate::motion::stop_and_wait(bridge);
    bridge.take_over();
    bridge.anim.manual_hands.store(true, Ordering::Relaxed);
    let report = {
        let mut vr = bridge.vr.lk();
        let r = run_open(&mut vr, bridge, opts);
        if r.is_err() {
            vr.reset();
        }
        r
    };
    bridge.anim.manual_hands.store(false, Ordering::Relaxed);
    bridge.idle_later();
    if let Ok(r) = &report {
        *bridge.usercam.last_open.lk() = Some(r.clone());
    }
    report
}

fn run_open(vr: &mut VrCore, bridge: &Bridge, opts: &OpenOptions) -> Result<Value> {
    let mut log = Log::new("usercam");
    let (yaw, pitch) = (vr.yaw, vr.pitch);
    let osc = bridge.osc_query()?;
    let before = mode(&osc).context("OSCQuery does not answer for /usercamera/Mode")?;
    log.step("start", json!({"mode": before, "facing_deg": yaw.round()}));
    let result = set_up(vr, bridge, &osc, yaw, before, opts, &mut log);
    // Whatever happened: the grips and triggers let go, the menu closed if
    // it is still up (the left hand still holds it, if it did), standing
    // as before.
    let _ = vr.release_hands();
    if menu_up(vr).unwrap_or(false) {
        calibrate::toggle_menu(bridge);
        log.step("menu closed", json!({}));
    }
    let _ = vr.face(yaw, pitch);
    let after = mode(&osc);
    let mut report = json!({
        "ok": result.is_ok() && after == Some(MODE_STREAM),
        "mode": {"before": before, "after": after, "state": mode_name(after)},
        "took_s": log.took_s(),
    });
    match result {
        Ok(v) => {
            for (k, val) in v.as_object().into_iter().flatten() {
                report[k] = val.clone();
            }
        }
        Err(e) => report["error"] = json!(format!("{e:#}")),
    }
    report["steps"] = json!(log.steps);
    Ok(report)
}

/// Whether the Quick Menu is up: looked for where it would be (the
/// posture that opens it), the right hand down.
fn menu_up(vr: &mut VrCore) -> Result<bool> {
    let yaw = vr.yaw;
    vr.aim(yaw, calibrate::MENU_PITCH)?;
    let (left, right) = (calibrate::MENU_HAND, calibrate::RIGHT_DOWN);
    vr.set_hand("left", left.0, left.1, 0.0, &[])?;
    vr.set_hand("right", right.0, right.1, 0.0, &[])?;
    let frame = calibrate::fresh_frame(vr)?;
    Ok(calibrate::find(&calibrate::ocr(vr, &frame)?, &ANCHOR).is_some())
}

fn set_up(vr: &mut VrCore, bridge: &Bridge, osc: &Osc, yaw: f32, before: i32, opts: &OpenOptions, log: &mut Log) -> Result<Value> {
    let mut out = json!({});
    if before == MODE_CLOSED {
        let grip = open_camera(vr, bridge, osc, yaw, log)?;
        out["opened"] = json!(true);
        // The viewfinder spawned by the right hand, which is still there.
        if opts.stow {
            out["viewfinder"] = stow(vr, osc, grip, opts.stow_delta, log)?;
        }
    } else {
        out["opened"] = json!(false);
        log.step("already open", json!({"mode": before}));
    }
    out["camera"] = stream_mode(osc)?;
    log.step("stream mode", out["camera"].clone());
    if opts.check {
        out["desktop"] = desktop_check(vr, bridge, osc, log)?;
    }
    Ok(out)
}

/// The Quick Menu up, a double-click on its camera icon (confirmed by its
/// tooltip first), until the camera reports a mode. The right hand's grip
/// on the icon.
fn open_camera(vr: &mut VrCore, bridge: &Bridge, osc: &Osc, yaw: f32, log: &mut Log) -> Result<Pose> {
    // The posture of a calibration: the menu held up, the right hand down
    // out of it.
    vr.aim(yaw, calibrate::MENU_PITCH)?;
    let (left, right) = (calibrate::MENU_HAND, calibrate::RIGHT_DOWN);
    vr.set_hand("left", left.0, left.1, 0.0, &[])?;
    vr.set_hand("right", right.0, right.1, 0.0, &[])?;
    let anchor = calibrate::open_menu(vr, bridge, &ANCHOR, log).context("the Quick Menu shows no 回出生点 to find the camera icon by")?;
    let frame = calibrate::fresh_frame(vr)?;
    let scale = frame.width as f32 / 1920.0;
    let icon = (anchor.0 + ICON_FROM_ANCHOR.0 * scale, anchor.1 + ICON_FROM_ANCHOR.1 * scale);
    for attempt in 1..=2 {
        let grip = calibrate::hover_until(vr, icon, &ICON_TOOLTIP, log).context("no aim at the camera icon showed its tooltip: not clicked")?;
        for _ in 0..2 {
            vr.set_hand_pose("right", grip, 1.0)?;
            std::thread::sleep(CLICK);
            vr.set_hand_pose("right", grip, 0.0)?;
            std::thread::sleep(CLICK_GAP);
        }
        let m = wait_mode(osc, OPENS_WITHIN, |m| m != MODE_CLOSED);
        log.step("double click", json!({"attempt": attempt, "mode": m}));
        if m.is_some_and(|m| m != MODE_CLOSED) {
            return Ok(grip);
        }
    }
    bail!("double-clicked the camera icon twice, but the camera did not open")
}

/// Whether the viewfinder is in the left eye (its labels, by OCR).
fn viewfinder_seen(vr: &mut VrCore) -> Result<Option<String>> {
    let frame = calibrate::fresh_frame(vr)?;
    let lines = calibrate::ocr(vr, &frame)?;
    Ok(calibrate::find(&lines, &VIEWFINDER).map(|l| l.text.clone()))
}

/// Grabs the viewfinder by the right hand (at `grip`, where it spawned)
/// and drags it `delta` (body frame) into the body; the left hand down
/// first, out of the view. Not letting it fly off: VRChat closes a camera
/// flung away (`FlingToClose`), so that is off and the drag is slow.
fn stow(vr: &mut VrCore, osc: &Osc, grip: Pose, delta: [f32; 3], log: &mut Log) -> Result<Value> {
    let down = calibrate::LEFT_DOWN;
    vr.set_hand("left", down.0, down.1, 0.0, &[])?;
    send_bool(osc, "/usercamera/FlingToClose", false)?;
    let before = viewfinder_seen(vr)?;
    let body = vr.body_yaw()?;
    let facing = Pose::looking(body, 0.0, [0.0; 3]);
    let (r, a) = (facing.rotate([1.0, 0.0, 0.0]), facing.rotate([0.0, 0.0, -1.0]));
    let world = [r[0] * delta[0] + a[0] * delta[2], delta[1], r[2] * delta[0] + a[2] * delta[2]];
    vr.set_hand_pose("right", grip, 0.0)?;
    vr.set_squeeze("right", 1.0)?;
    std::thread::sleep(GRIP_SETTLE);
    for i in 1..=DRAG_STEPS {
        let t = i as f32 / DRAG_STEPS as f32;
        let p = grip.position;
        let at = [p[0] + world[0] * t, p[1] + world[1] * t, p[2] + world[2] * t];
        vr.set_hand_pose("right", Pose { orientation: grip.orientation, position: at }, 0.0)?;
        std::thread::sleep(DRAG_STEP);
    }
    std::thread::sleep(GRIP_SETTLE);
    vr.set_squeeze("right", 0.0)?;
    std::thread::sleep(GRIP_SETTLE);
    let right = calibrate::RIGHT_DOWN;
    vr.set_hand("right", right.0, right.1, 0.0, &[])?;
    let after = viewfinder_seen(vr)?;
    let v = json!({
        "seen_before": before,
        "seen_after": after,
        // Out of the view by this drag (it was in it before).
        "stowed": before.is_some() && after.is_none(),
    });
    log.step("viewfinder", v.clone());
    Ok(v)
}

/// The desktop shows the camera: grabs with it above the head looking
/// ahead and behind differ. It stays above the head, looking ahead.
fn desktop_check(vr: &mut VrCore, bridge: &Bridge, osc: &Osc, log: &mut Log) -> Result<Value> {
    let (eye, heading) = head(vr)?;
    let ahead = around(eye, heading, 0.0, 0.0, ABOVE_HEAD_M, Look::Out, 0.0);
    let behind = around(eye, heading, 180.0, 0.0, ABOVE_HEAD_M, Look::Out, 0.0);
    let grab = |pose: &CamPose| -> Result<Rgb> {
        place(bridge, osc, pose)?;
        grab_desktop(&bridge.args.display, &bridge.args.desktop_grab)
    };
    let a = grab(&behind)?;
    let b = grab(&ahead)?;
    *bridge.usercam.last_pose.lk() = Some(ahead);
    let diff = a.diff(&b);
    let v = json!({"diff": (diff * 10.0).round() / 10.0, "shows_camera": diff >= DESKTOP_DIFF_MIN, "pose": ahead});
    log.step("desktop", v.clone());
    if diff < DESKTOP_DIFF_MIN {
        bail!("the desktop does not follow the camera (grabs ahead and behind differ by {diff:.1})");
    }
    Ok(v)
}

/// The viewfinder of an open camera dragged into the body: it is fixed in
/// the tracking space wherever it was left, and the drag starts from where
/// it spawns, by the right hand as the camera opens. So the camera is
/// closed and opened again (the game keeps its settings: the lens hidden,
/// the follow mode, measured 2026-10-08) and the viewfinder stowed as on
/// any opening.
pub fn restow(bridge: &Arc<Bridge>, opts: &OpenOptions) -> Result<Value> {
    let closed = {
        let Some(_one) = RUNNING.try_lk() else { bail!("the user camera is busy (being opened, or a shot)") };
        let osc = bridge.osc_query()?;
        let before = mode(&osc).context("OSCQuery does not answer for /usercamera/Mode")?;
        if before != MODE_CLOSED {
            send_bool(&osc, "/usercamera/Close", true)?;
            let m = wait_mode(&osc, Duration::from_secs(3), |m| m == MODE_CLOSED);
            anyhow::ensure!(m == Some(MODE_CLOSED), "the camera did not close (Mode {m:?})");
        }
        before != MODE_CLOSED
    };
    let mut report = open(bridge, &OpenOptions { stow: true, ..opts.clone() })?;
    report["reopened"] = json!(closed);
    Ok(report)
}

/// Closes the camera.
pub fn close(bridge: &Bridge) -> Result<Value> {
    let _one = RUNNING.lk();
    let osc = bridge.osc_query()?;
    send_bool(&osc, "/usercamera/Close", true)?;
    let m = wait_mode(&osc, Duration::from_secs(2), |m| m == MODE_CLOSED);
    Ok(json!({"ok": m == Some(MODE_CLOSED), "mode": m, "state": mode_name(m)}))
}

// -- shots -----------------------------------------------------------------------------

/// Where a shot goes: a world pose, or round the head.
#[derive(Clone, Copy, Debug)]
pub enum Aim {
    World(CamPose),
    Around { bearing: f32, distance: f32, height: f32, look: Look, pitch_up: f32 },
}

/// A shot's pose, from `{"pose": [x, y, z, pitch, yaw, roll]}` or
/// `{"bearing_deg", "distance_m", "height_m", "look": "out"|"back",
/// "pitch_deg"}` (round the head; all may be left out).
pub fn aim_from(body: &Value) -> Result<Aim> {
    if let Some(p) = body.get("pose") {
        let a: Vec<f32> = serde_json::from_value(p.clone()).context("pose is six numbers")?;
        anyhow::ensure!(a.len() == 6 && a.iter().all(|v| v.is_finite()), "pose is [x, y, z, pitch, yaw, roll]");
        return Ok(Aim::World(CamPose::from_args([a[0], a[1], a[2], a[3], a[4], a[5]])));
    }
    let num = |k: &str, d: f32| -> Result<f32> {
        match body.get(k) {
            None | Some(Value::Null) => Ok(d),
            Some(v) => v.as_f64().map(|v| v as f32).filter(|v| v.is_finite()).with_context(|| format!("{k} is a number")),
        }
    };
    let look = match body["look"].as_str().unwrap_or("out") {
        "out" => Look::Out,
        "back" => Look::Back,
        other => bail!("look is out or back, not {other}"),
    };
    let distance = num("distance_m", if look == Look::Back { 1.5 } else { 0.0 })?;
    let height = num("height_m", ABOVE_HEAD_M)?;
    anyhow::ensure!((0.0..=20.0).contains(&distance) && height.abs() <= 10.0, "distance_m 0-20, height_m within 10");
    anyhow::ensure!(look == Look::Out || distance >= 0.2, "looking back needs distance_m of 0.2 or more");
    Ok(Aim::Around { bearing: num("bearing_deg", 0.0)?, distance, height, look, pitch_up: num("pitch_deg", 0.0)?.clamp(-89.0, 89.0) })
}

/// The camera open (any mode: stream and the UI mask are set again).
fn ready(bridge: &Bridge) -> Result<Osc> {
    let osc = bridge.osc_query()?;
    match mode(&osc) {
        None => bail!("OSCQuery does not answer for /usercamera/Mode"),
        Some(MODE_CLOSED) => bail!("the user camera is closed: POST /v1/vr/usercam {{\"open\": true}}"),
        Some(MODE_STREAM) => {}
        Some(_) => {
            osc.send("/usercamera/Mode", &[Arg::Int(MODE_STREAM)])?;
            std::thread::sleep(Duration::from_millis(300));
        }
    }
    send_bool(&osc, "/usercamera/ShowUIInCamera", true)?;
    Ok(osc)
}

/// Reads the head (for poses round it) from the eyes.
fn head_now(bridge: &Bridge) -> Result<([f32; 3], f32)> {
    let mut vr = bridge.vr.lk();
    head(&mut vr)
}

fn resolve(aim: Aim, at: Option<([f32; 3], f32)>) -> CamPose {
    match aim {
        Aim::World(p) => p,
        Aim::Around { bearing, distance, height, look, pitch_up } => {
            let (eye, heading) = at.expect("the head for a pose round it");
            around(eye, heading, bearing, distance, height, look, pitch_up)
        }
    }
}

/// Places the camera and grabs what it sees: the pose, and the frame.
pub fn shot(bridge: &Bridge, aim: Aim) -> Result<(CamPose, Rgb)> {
    let _one = RUNNING.lk();
    let osc = ready(bridge)?;
    let at = match aim {
        Aim::World(_) => None,
        Aim::Around { .. } => Some(head_now(bridge)?),
    };
    let pose = resolve(aim, at);
    place(bridge, &osc, &pose)?;
    *bridge.usercam.last_pose.lk() = Some(pose);
    Ok((pose, grab_desktop(&bridge.args.display, &bridge.args.desktop_grab)?))
}

/// A ring of `views` shots round the head (bearings from straight ahead,
/// clockwise), as one mosaic three wide; the camera goes back to the
/// first after.
pub fn sweep(bridge: &Bridge, views: usize, distance: f32, height: f32, look: Look, pitch_up: f32) -> Result<(Vec<f32>, Rgb)> {
    let _one = RUNNING.lk();
    let osc = ready(bridge)?;
    let (eye, heading) = head_now(bridge)?;
    let bearings: Vec<f32> = (0..views).map(|i| i as f32 * 360.0 / views as f32).collect();
    let mut tiles = Vec::with_capacity(views);
    for &b in &bearings {
        place(bridge, &osc, &around(eye, heading, b, distance, height, look, pitch_up))?;
        tiles.push(grab_desktop(&bridge.args.display, &bridge.args.desktop_grab)?);
    }
    let first = around(eye, heading, 0.0, distance, height, look, pitch_up);
    place(bridge, &osc, &first)?;
    *bridge.usercam.last_pose.lk() = Some(first);
    Ok((bearings, mosaic(&tiles, 3)))
}

// -- opening again by itself ----------------------------------------------------------

/// Opens the camera again when it is found closed while it should stay
/// open (`keep_open`): after VRChat starts, or a world is joined; checked
/// every few seconds by the animation thread, as the calibration's.
pub struct Watch {
    last_try: Option<Instant>,
    session: String,
    since: Instant,
}

impl Watch {
    pub fn new() -> Watch {
        Watch { last_try: None, session: String::new(), since: Instant::now() }
    }

    pub fn check(&mut self, bridge: &Arc<Bridge>) {
        let settings = bridge.usercam.settings.lk().clone();
        let (running, session, in_world) = {
            let g = bridge.game.lk();
            (g.running, g.session(), !g.instance.is_empty())
        };
        if session != self.session {
            self.session = session;
            self.since = Instant::now();
        }
        if !settings.keep_open || !running || !in_world || self.since.elapsed() < SETTLE_AFTER_JOIN {
            return;
        }
        if self.last_try.is_some_and(|t| t.elapsed() < RETRY_AFTER) {
            return;
        }
        let Ok(osc) = bridge.osc_query() else { return };
        if mode(&osc) != Some(MODE_CLOSED) {
            return;
        }
        // A calibration due goes first (it holds the hands too).
        if bridge.anim.trackers.lk().on && calibrate::tracking_type(bridge) == Some(calibrate::HEAD_AND_HANDS) {
            return;
        }
        self.last_try = Some(Instant::now());
        tracing::info!("usercam: closed though it should stay open: opening it");
        let bridge = bridge.clone();
        let opts = OpenOptions { stow: settings.stow, stow_delta: STOW_DELTA, check: true };
        std::thread::spawn(move || match open(&bridge, &opts) {
            Ok(report) => tracing::info!("usercam: {report}"),
            Err(e) => tracing::warn!("usercam: {e:#}"),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-3
    }

    #[test]
    fn a_camera_out_ahead_and_to_the_right() {
        // Heading +z (yaw 0): 2 m ahead is +z.
        let p = around([1.0, 1.2, 2.0], 0.0, 0.0, 2.0, 0.3, Look::Out, 0.0);
        assert!(close(p.position[0], 1.0) && close(p.position[1], 1.5) && close(p.position[2], 4.0), "{p:?}");
        assert!(close(p.yaw, 0.0) && close(p.pitch, 0.0));
        // Clockwise from +z seen from above: 90 to the right is +x.
        let p = around([0.0; 3], 0.0, 90.0, 1.0, 0.0, Look::Out, 10.0);
        assert!(close(p.position[0], 1.0) && close(p.position[2], 0.0), "{p:?}");
        assert!(close(p.yaw, 90.0));
        // Up for the bridge is down negative for VRChat.
        assert!(close(p.pitch, -10.0));
        // Bearings add to the heading, wrapped.
        let p = around([0.0; 3], 300.0, 90.0, 0.0, 0.0, Look::Out, 0.0);
        assert!(close(p.yaw, 30.0), "{p:?}");
    }

    #[test]
    fn a_camera_looking_back_at_the_head() {
        // As measured: 1.5 m ahead of a head facing 250.86, 0.5 m up,
        // looking back and down at it.
        let eye = [5.183, 1.175, 0.97];
        let p = around(eye, 250.86, 0.0, 1.5, 0.5, Look::Back, 0.0);
        assert!((p.yaw - 70.86).abs() < 0.01, "{p:?}");
        assert!((p.pitch - 18.43).abs() < 0.01, "{p:?}");
        assert!((p.position[0] - 3.766).abs() < 0.01 && (p.position[2] - 0.479).abs() < 0.01, "{p:?}");
        // Where it looks comes back to the eyes.
        let (s, c) = p.yaw.to_radians().sin_cos();
        let back = [p.position[0] + 1.5 * s, p.position[2] + 1.5 * c];
        assert!((back[0] - eye[0]).abs() < 0.01 && (back[1] - eye[2]).abs() < 0.01);
    }

    #[test]
    fn the_pose_message_is_six_big_endian_floats() {
        let p = CamPose::from_args([1.0, 2.0, -3.5, 10.0, 370.0, 0.0]);
        assert!(close(p.yaw, 10.0));
        let m = p.message();
        // "/usercamera/Pose" (16) padded to 20, ",ffffff" padded to 8, 24.
        assert_eq!(m.len(), 52);
        assert_eq!(&m[..16], b"/usercamera/Pose");
        assert_eq!(&m[16..20], &[0, 0, 0, 0]);
        assert_eq!(&m[20..28], b",ffffff\0");
        assert_eq!(&m[28..32], &1.0f32.to_be_bytes());
        assert_eq!(&m[36..40], &(-3.5f32).to_be_bytes());
        assert_eq!(&m[44..48], &10.0f32.to_be_bytes());
    }

    #[test]
    fn a_boolean_is_its_type_tag_alone() {
        let m = encode("/usercamera/Flying", &[Arg::Bool(false)]);
        // "/usercamera/Flying" (18) padded to 20, ",F" padded to 4.
        assert_eq!(m.len(), 24);
        assert_eq!(&m[20..24], b",F\0\0");
        assert_eq!(&encode("/usercamera/Mode", &[Arg::Int(MODE_STREAM)])[20..], b",i\0\0\0\0\0\x02");
    }

    #[test]
    fn shots_are_aimed_from_the_request() {
        match aim_from(&json!({"pose": [1, 2, 3, 4, 5, 6]})).unwrap() {
            Aim::World(p) => assert_eq!(p.args(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
            a => panic!("{a:?}"),
        }
        match aim_from(&json!({"bearing_deg": 90, "look": "back"})).unwrap() {
            Aim::Around { bearing, distance, height, look, .. } => {
                assert!(close(bearing, 90.0) && close(distance, 1.5) && close(height, ABOVE_HEAD_M) && look == Look::Back)
            }
            a => panic!("{a:?}"),
        }
        assert!(aim_from(&json!({"look": "back", "distance_m": 0})).is_err());
        assert!(aim_from(&json!({"pose": [1, 2, 3]})).is_err());
        assert!(aim_from(&json!({"look": "up"})).is_err());
    }

    #[test]
    fn the_grab_spec() {
        assert_eq!(parse_grab("1280x720+0,0").unwrap(), (1280, 720, 0, 0));
        assert_eq!(parse_grab("640x360+10,20").unwrap(), (640, 360, 10, 20));
        assert!(parse_grab("1280x720").is_err() && parse_grab("0x720+0,0").is_err());
    }

    #[test]
    fn a_mosaic_and_a_difference() {
        let solid = |v: u8| Rgb { width: 1280, height: 720, data: vec![v; 1280 * 720 * 3] };
        let m = mosaic(&[solid(10), solid(20), solid(30), solid(40)], 3);
        assert_eq!((m.width, m.height), (3 * TILE.0, 2 * TILE.1));
        // The fourth tile starts the second row; the rest of it is black.
        assert_eq!(m.data[(TILE.1 * m.width) * 3], 40);
        assert_eq!(m.data[(TILE.1 * m.width + TILE.0) * 3], 0);
        assert_eq!(m.data[(TILE.0 * 2) * 3], 30);
        assert!(close(solid(10).diff(&solid(30)), 20.0));
        assert!(close(solid(7).diff(&solid(7)), 0.0));
    }
}
