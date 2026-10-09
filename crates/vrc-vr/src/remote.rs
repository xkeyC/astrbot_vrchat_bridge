//! Client of Monado's remote driver (`src/xrt/drivers/remote`), the headset
//! VRChat sees: one TCP connection, the whole state (head, each eye,
//! controllers) as one fixed-size packet each time it changes.
//!
//! Monado config (`~/.config/monado/config_v0.json`):
//! `{"active": "remote", "remote": {"version": 0, "port": 4242, "view_count": 2, ...}}`.
//!
//! The wire format is `struct r_remote_data` of `r_interface.h` (protocol
//! "mndrmt3"), little endian, 376 bytes; on connect the driver first sends
//! its reset state and its latest state.
//!
//! Two may drive one connection: the owner (`RemoteHmd`: where the head
//! looks, the body faces) and an animator (an [`HmdLink`]: an [`Overlay`] of
//! the hands' motion and a little of the head's, see [`crate::anim`]). What
//! is sent is the owner's state with the overlay on top, except while the
//! owner holds still (scans: the views must be exactly where asked, and
//! the arms out of them).

use std::io::{Read, Write};
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use anyhow::{bail, Context, Result};

use crate::anim::{self, AnimParams};
use crate::pose::{quat_axis, quat_mul, Fov, Pose};

/// Size of `struct r_remote_data`.
pub const PACKET_SIZE: usize = 376;
const MAGIC: [u8; 8] = *b"mndrmt3\0";
const HEAD_AT: usize = 8;
const VIEW_SIZE: usize = 48;
const CENTER_AT: usize = HEAD_AT + 2 * VIEW_SIZE;
const PER_VIEW_VALID_AT: usize = CENTER_AT + 28;
const LEFT_AT: usize = 136;
const RIGHT_AT: usize = 256;

/// The eyes' height above VRChat's floor in the tracking space with the
/// head at Monado's default 1.6 m (measured 2026-10-05): the unit of the
/// rest pose and the animation's distances.
pub const EYE_HEIGHT: f32 = 1.93;
/// VRChat's floor in the tracking space: below Monado's (y = 0), where its
/// height calibration put it for this avatar (stereo: -0.31..-0.32 with the
/// head at 1.45-1.6 m; D23). Eyes to floor = head height - FLOOR_Y.
pub const FLOOR_Y: f32 = -0.32;
/// How far back of rest the hands go while the bot looks about (metres).
pub const HANDS_BACK_M: f32 = 0.08;
/// One eye as the driver reports it: its field of view, and its pose
/// relative to the head.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct View {
    pub fov: Fov,
    pub pose: Pose,
}

/// A controller (`struct r_remote_controller_data`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Controller {
    pub active: bool,
    pub hand_tracking_active: bool,
    pub pose: Pose,
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub hand_curl: [f32; 5],
    pub trigger: f32,
    pub squeeze: f32,
    pub squeeze_force: f32,
    pub thumbstick: [f32; 2],
    pub trackpad_force: f32,
    pub trackpad: [f32; 2],
    pub system_click: bool,
    pub system_touch: bool,
    pub a_click: bool,
    pub a_touch: bool,
    pub b_click: bool,
    pub b_touch: bool,
    pub trigger_click: bool,
    pub trigger_touch: bool,
    pub thumbstick_click: bool,
    pub thumbstick_touch: bool,
    pub trackpad_touch: bool,
}

/// Everything the driver reports.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    /// The head (OpenXR view space) in the tracking space.
    pub head: Pose,
    /// Per eye fields of view and poses (relative to the head); `None` lets
    /// the driver use its defaults (85 degrees, 63 mm apart).
    pub views: Option<[View; 2]>,
    pub left: Controller,
    pub right: Controller,
    /// Where the body faces (degrees), as [`State::hands_at_rest`] last
    /// placed the hands for. Not sent.
    pub body_yaw: f32,
}

impl State {
    /// Both hands hanging at the sides of a body under `head` (tracking
    /// space) that faces `body_yaw_deg`, palms in. Monado's remote driver
    /// starts them half a metre in front of the eyes: then turning the head
    /// swings the avatar's arms into the view (they reach for hands that did
    /// not turn). The body is where the hands say: VRChat turns it to keep
    /// them at its sides, so placing them for a new facing turns the body.
    ///
    /// The pose is OpenXR's grip pose; see [`crate::anim::hand_pose`] for
    /// the rest pose and its defaults.
    pub fn hands_at_rest(&mut self, head: [f32; 3], body_yaw_deg: f32) {
        self.body_yaw = body_yaw_deg;
        let p = AnimParams::default();
        for (hand, side) in [(&mut self.left, -1.0f32), (&mut self.right, 1.0)] {
            let pose = anim::hand_pose(&p, head, body_yaw_deg, side, [0.0; 3], [0.0, 0.0, 0.0, 1.0]);
            *hand = anim::hand(&p, pose, p.curl);
        }
    }

    /// Both hands at rest a little further back than at rest (looking about
    /// for someone: out of the lower edge of the view), the body facing
    /// `body_yaw_deg`.
    pub fn hands_back_a_little(&mut self, head: [f32; 3], body_yaw_deg: f32) {
        self.body_yaw = body_yaw_deg;
        let p = AnimParams::default();
        for (hand, side) in [(&mut self.left, -1.0f32), (&mut self.right, 1.0)] {
            // Body frame: right, up, back (metres).
            let pose = anim::hand_pose(&p, head, body_yaw_deg, side, [0.0, 0.0, HANDS_BACK_M], [0.0, 0.0, 0.0, 1.0]);
            *hand = anim::hand(&p, pose, p.curl);
        }
    }

    /// Both hands raised behind the head as seen looking along `yaw_deg`:
    /// out of that view, so only a mirror shows them.
    pub fn hands_up_behind(&mut self, head: [f32; 3], yaw_deg: f32) {
        let (s, c) = yaw_deg.to_radians().sin_cos();
        // View frame: right = (c, 0, s), ahead = (s, 0, -c); behind is -ahead.
        for (hand, side) in [(&mut self.left, -1.0f32), (&mut self.right, 1.0)] {
            let (right, back) = (side * 0.25, 0.45);
            hand.active = true;
            hand.pose = Pose {
                orientation: [0.0, 0.0, 0.0, 1.0],
                position: [head[0] + right * c - back * s, head[1] + 0.3, head[2] + right * s + back * c],
            };
        }
    }

    pub fn encode(&self) -> [u8; PACKET_SIZE] {
        let mut w = Writer { buf: [0; PACKET_SIZE], at: 0 };
        w.bytes(&MAGIC);
        for view in self.views.unwrap_or_default() {
            w.fov(&view.fov);
            w.pose(&view.pose);
            w.at += 4; // padding
        }
        debug_assert_eq!(w.at, CENTER_AT);
        w.pose(&self.head);
        debug_assert_eq!(w.at, PER_VIEW_VALID_AT);
        w.flag(self.views.is_some());
        w.at = LEFT_AT;
        w.controller(&self.left);
        debug_assert_eq!(w.at, RIGHT_AT);
        w.controller(&self.right);
        debug_assert_eq!(w.at, PACKET_SIZE);
        w.buf
    }

    pub fn decode(buf: &[u8; PACKET_SIZE]) -> Result<State> {
        if buf[..8] != MAGIC {
            bail!("not a remote driver packet (protocol {:?})", &buf[..8]);
        }
        let mut r = Reader { buf, at: HEAD_AT };
        let mut views = [View::default(); 2];
        for view in &mut views {
            view.fov = r.fov();
            view.pose = r.pose();
            r.at += 4;
        }
        let head = r.pose();
        let per_view = r.flag();
        r.at = LEFT_AT;
        let left = r.controller();
        let right = r.controller();
        Ok(State { head, views: per_view.then_some(views), left, right, body_yaw: 0.0 })
    }
}

/// A connection to the remote driver.
pub struct RemoteHmd {
    link: Arc<Mutex<Link>>,
    /// What the driver resets to (as it sent it on connect).
    pub reset: State,
    /// The owner's state, as last sent (or the driver's latest on connect).
    pub state: State,
}

/// The connection, shared by the owner and an animator.
struct Link {
    stream: TcpStream,
    /// The owner's state, as last sent.
    base: State,
    overlay: Option<Overlay>,
    still: bool,
    closed: bool,
}

/// An animator's part: both hands as they are now, and a small offset of
/// the head; and the hands at rest, sent while the owner holds still.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Overlay {
    pub left: Controller,
    pub right: Controller,
    pub rest: [Controller; 2],
    pub head: HeadOffset,
    /// The whole head instead of the owner's (a motion clip moves it).
    pub head_pose: Option<Pose>,
}

/// A small motion of the head on top of where the owner points it: turned
/// (degrees: yaw + right, pitch + up, roll + clockwise as seen from behind)
/// and moved (tracking space).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeadOffset {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub position: [f32; 3],
}

impl HeadOffset {
    pub fn apply(&self, head: Pose) -> Pose {
        let turn = quat_axis([0.0, 1.0, 0.0], -self.yaw.to_radians());
        let tilt = quat_mul(quat_axis([1.0, 0.0, 0.0], self.pitch.to_radians()), quat_axis([0.0, 0.0, 1.0], -self.roll.to_radians()));
        Pose {
            orientation: quat_mul(quat_mul(turn, head.orientation), tilt),
            position: [head.position[0] + self.position[0], head.position[1] + self.position[1], head.position[2] + self.position[2]],
        }
    }
}

impl Link {
    fn send(&mut self) -> Result<()> {
        let mut s = self.base;
        match (self.overlay, self.still) {
            (Some(o), false) => {
                s.left = o.left;
                s.right = o.right;
                s.head = match o.head_pose {
                    Some(head) => head,
                    None => o.head.apply(s.head),
                };
            }
            (Some(o), true) => [s.left, s.right] = o.rest,
            (None, _) => {}
        }
        let sent = self.stream.write_all(&s.encode());
        if sent.is_err() {
            // A packet maybe half sent: nothing after it would make sense.
            self.closed = true;
            let _ = self.stream.shutdown(Shutdown::Both);
        }
        sent.context("sending to the remote driver")
    }
}

/// An animator's handle on a [`RemoteHmd`]'s connection.
#[derive(Clone)]
pub struct HmdLink(Arc<Mutex<Link>>);

/// What an animator sees of the owner.
#[derive(Clone, Copy, Debug)]
pub struct Owner {
    pub state: State,
    /// Holding still (a scan): the overlay is not sent.
    pub still: bool,
}

impl HmdLink {
    /// The owner's state; `None` once the connection is gone (the owner
    /// dropped it).
    pub fn owner(&self) -> Option<Owner> {
        let l = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        (!l.closed).then_some(Owner { state: l.base, still: l.still })
    }

    /// Sets the overlay (`None`: the owner's state alone) and sends.
    pub fn set_overlay(&self, overlay: Option<Overlay>) -> Result<()> {
        let mut l = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        if l.closed {
            bail!("the headset connection is closed");
        }
        l.overlay = overlay;
        l.send()
    }
}

impl RemoteHmd {
    pub fn connect(addr: impl ToSocketAddrs) -> Result<RemoteHmd> {
        let mut stream = TcpStream::connect(addr).context("connecting to Monado's remote driver")?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        // Monado stuck (alive, not reading): fail, not block every holder
        // of the link for good.
        stream.set_write_timeout(Some(Duration::from_secs(2)))?;
        let mut buf = [0u8; PACKET_SIZE];
        stream.read_exact(&mut buf).context("reading the reset state")?;
        let reset = State::decode(&buf)?;
        stream.read_exact(&mut buf).context("reading the latest state")?;
        let mut state = State::decode(&buf)?;
        state.body_yaw = state.head.yaw_pitch().0;
        let link = Link { stream, base: state, overlay: None, still: false, closed: false };
        Ok(RemoteHmd { link: Arc::new(Mutex::new(link)), reset, state })
    }

    /// Sends [`RemoteHmd::state`] (with an animator's overlay, if any).
    pub fn send(&mut self) -> Result<()> {
        let mut l = self.link.lock().unwrap_or_else(PoisonError::into_inner);
        if l.closed {
            bail!("the headset connection is closed");
        }
        l.base = self.state;
        l.send()
    }

    /// A handle for an animator.
    pub fn link(&self) -> HmdLink {
        HmdLink(self.link.clone())
    }

    /// Holds still (`true`): only [`RemoteHmd::state`] is sent, no overlay,
    /// until released.
    pub fn hold_still(&mut self, still: bool) -> Result<()> {
        let mut l = self.link.lock().unwrap_or_else(PoisonError::into_inner);
        if l.still != still {
            l.still = still;
            l.base = self.state;
            l.send()?;
        }
        Ok(())
    }

    /// Sets the head and sends.
    pub fn set_head(&mut self, head: Pose) -> Result<()> {
        self.state.head = head;
        self.send()
    }
}

impl Drop for RemoteHmd {
    fn drop(&mut self) {
        let mut l = self.link.lock().unwrap_or_else(PoisonError::into_inner);
        l.closed = true;
        let _ = l.stream.shutdown(Shutdown::Both);
    }
}

struct Writer {
    buf: [u8; PACKET_SIZE],
    at: usize,
}

impl Writer {
    fn bytes(&mut self, b: &[u8]) {
        self.buf[self.at..self.at + b.len()].copy_from_slice(b);
        self.at += b.len();
    }
    fn f32(&mut self, v: f32) {
        self.bytes(&v.to_le_bytes());
    }
    fn f32s(&mut self, vs: &[f32]) {
        vs.iter().for_each(|&v| self.f32(v));
    }
    fn flag(&mut self, v: bool) {
        self.bytes(&[v as u8]);
    }
    fn fov(&mut self, f: &Fov) {
        self.f32s(&[f.left, f.right, f.up, f.down]);
    }
    fn pose(&mut self, p: &Pose) {
        self.f32s(&p.orientation);
        self.f32s(&p.position);
    }
    fn controller(&mut self, c: &Controller) {
        self.pose(&c.pose);
        self.f32s(&c.linear_velocity);
        self.f32s(&c.angular_velocity);
        self.f32s(&c.hand_curl);
        self.f32s(&[c.trigger, c.squeeze, c.squeeze_force]);
        self.f32s(&c.thumbstick);
        self.f32(c.trackpad_force);
        self.f32s(&c.trackpad);
        for v in [
            c.hand_tracking_active,
            c.active,
            c.system_click,
            c.system_touch,
            c.a_click,
            c.a_touch,
            c.b_click,
            c.b_touch,
            c.trigger_click,
            c.trigger_touch,
            c.thumbstick_click,
            c.thumbstick_touch,
            c.trackpad_touch,
        ] {
            self.flag(v);
        }
        self.at += 3; // padding
    }
}

struct Reader<'a> {
    buf: &'a [u8; PACKET_SIZE],
    at: usize,
}

impl Reader<'_> {
    fn f32(&mut self) -> f32 {
        let v = f32::from_le_bytes(self.buf[self.at..self.at + 4].try_into().unwrap());
        self.at += 4;
        v
    }
    fn f32s<const N: usize>(&mut self) -> [f32; N] {
        std::array::from_fn(|_| self.f32())
    }
    fn flag(&mut self) -> bool {
        self.at += 1;
        self.buf[self.at - 1] != 0
    }
    fn fov(&mut self) -> Fov {
        let [left, right, up, down] = self.f32s();
        Fov { left, right, up, down }
    }
    fn pose(&mut self) -> Pose {
        Pose { orientation: self.f32s(), position: self.f32s() }
    }
    fn controller(&mut self) -> Controller {
        let mut c = Controller {
            pose: self.pose(),
            linear_velocity: self.f32s(),
            angular_velocity: self.f32s(),
            hand_curl: self.f32s(),
            trigger: self.f32(),
            squeeze: self.f32(),
            squeeze_force: self.f32(),
            thumbstick: self.f32s(),
            trackpad_force: self.f32(),
            trackpad: self.f32s(),
            ..Default::default()
        };
        for flag in [
            &mut c.hand_tracking_active,
            &mut c.active,
            &mut c.system_click,
            &mut c.system_touch,
            &mut c.a_click,
            &mut c.a_touch,
            &mut c.b_click,
            &mut c.b_touch,
            &mut c.trigger_click,
            &mut c.trigger_touch,
            &mut c.thumbstick_click,
            &mut c.thumbstick_touch,
            &mut c.trackpad_touch,
        ] {
            *flag = self.flag();
        }
        self.at += 3;
        c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hands_rest_at_the_sides() {
        let mut s = State::default();
        s.hands_at_rest([0.0, 1.93, 0.0], 0.0);
        // Fingers (the grip's -z, as Monado simulates the hand) down.
        let fingers = s.right.pose.rotate([0.0, 0.0, -1.0]);
        assert!(fingers[1] < -0.95, "{fingers:?}");
        assert!(s.right.hand_tracking_active && s.right.hand_curl[2] > 0.0);
        assert!(s.left.pose.position[0] < 0.0 && s.right.pose.position[0] > 0.0);
        // Facing right (+x): the right hand is behind (+z), the left ahead.
        s.hands_at_rest([0.0, 1.93, 0.0], 90.0);
        assert!(s.right.pose.position[2] > 0.2 && s.left.pose.position[2] < -0.2);
    }

    #[test]
    fn round_trips() {
        let mut s = State {
            head: Pose::looking(30.0, -10.0, [0.0, 1.6, 0.0]),
            views: Some([
                View { fov: Fov { left: -0.7, right: 0.7, up: 0.75, down: -0.75 }, pose: Pose::IDENTITY },
                View::default(),
            ]),
            ..Default::default()
        };
        s.left.active = true;
        s.left.thumbstick = [0.0, 1.0];
        s.right.trackpad_touch = true;
        s.right.trigger = 0.5;
        let back = State::decode(&s.encode()).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn layout_matches_r_interface_h() {
        let mut s = State::default();
        s.head.position = [1.0, 2.0, 3.0];
        s.right.trackpad_touch = true;
        let buf = s.encode();
        assert_eq!(&buf[..8], b"mndrmt3\0");
        // head.center.position.x: header 8 + 2 views x 48 + orientation 16.
        assert_eq!(f32::from_le_bytes(buf[120..124].try_into().unwrap()), 1.0);
        // right.trackpad_touch: the 13th flag after 26 floats of the right controller.
        assert_eq!(buf[RIGHT_AT + 26 * 4 + 12], 1);
    }
}
