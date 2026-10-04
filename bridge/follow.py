"""Following a player in the room: their name tag says who and where, the
depth under it how far.

Frames of the game window go to local-multimodal-infra: PP-OCRv5 text lines
(``POST /v1/ocr/lines``) find the target's name tag, and Depth Anything V2
metric depth (``POST /v1/depth``, a grid of metres) says how far the target
is, and whether something stands in the way. How far: where the body under
the tag meets the floor, read off the floor's fit (nav.fit_floor; the depth
model reads an avatar's body nearer than it is, the floor it gets right),
else the body's own depth. In third person (the camera behind the avatar,
its own body in the middle of the view: selfview), distances count from the
avatar's feet, its body is no obstacle, and the target is kept a little to
the side of it, not hidden behind its head.

The avatar keeps facing the tag (relative mouse motion: VRChat's OSC look
axis has a large dead zone, nothing below about half speed turns at all) and
keeps its distance in metres: farther than WALK_M it walks, faster the
farther, within STAND_M it stands, closer than BACK_M it steps back; it never
walks into something nearer than OBSTACLE_M. A target out of sight is first
sought where they were heading (dead reckoning: the avatar's own turns and
walking give its pose, each sighting the target's position; their last
known position plus their velocity is where to go, as robot person
followers do), then searched for by turning on the spot there, for as long
as they are in the room: following ends only when told to stop or when they
leave.
"""

from __future__ import annotations

import asyncio
import collections
import contextlib
import difflib
import logging
import math
import time

import aiohttp

import nav
from tracking import TargetTrack

log = logging.getLogger("vrc-bridge.follow")

FPS = 5
# Distance to the target in metres as the depth model reads the game, which
# is about twice the world's (the floor at the bottom edge, ~2.4 m away for
# the eye height and FOV, reads ~5 m). Between STAND_M and WALK_M the last
# move goes on (no start-stop jitter at the edge).
# The standing distance can be set per follow and adjusted while following
# (in world metres, DEPTH_SCALE of the model's); the others keep their ratio
# to it.
DEPTH_SCALE = 2.0
MIN_STAND_M = 1.6
MAX_STAND_M = 14.0
# "closer" / "farther" scale the standing distance by these.
CLOSER = 0.7
FARTHER = 1.4
# (Measured from the feet on the floor since 2026-10-03: the old body depth
# read an avatar nearer, so these were larger.)
BACK_M = 1.8
STAND_M = 3.0
WALK_M = 3.6
# The forward axis is analog (measured: 0.3 ~0.65 m/s, 0.5 ~1.8 m/s, 1.0
# ~4.4 m/s, already a run). The speed is what reaches the standing line in
# BRAKE_S at the soonest, between MIN_SPEED and MAX_SPEED: fast far away,
# slow close by. The distance it goes by is the one measured less what the
# avatar walked since the frame was taken (frames are ~0.2-0.5 s old by
# then), so a fast walk does not run past the line. /input/Run is not used.
MIN_SPEED = 0.35
MAX_SPEED = 1.0
BRAKE_S = 0.7
# A tag this near the top edge (px at 720 rows) is close: closer, it floats
# out of view above (larger nameplates sooner).
TAG_TOP = 100
BACK_SPEED = 0.3
# A move lasts this long unless the next frame renews it: no walking on
# blind while OCR stalls or a turn settles.
MOVE_HOLD = 0.4
# Something nearer than this (same scale) in the middle of the view (body
# height, the floor left out) blocks walking.
OBSTACLE_M = 1.5
# The target's feet: the body under its tag is nearer than FEET_NEAR of the
# floor behind it; the floor showing again below is where it stands. Its
# distance there is believed up to GROUND_MAX times its body's depth (the
# depth model reads an avatar nearer than it is; beyond, something else).
FEET_NEAR = 0.8
GROUND_MAX = 2.0
# In third person the target is kept this far to the side of the middle
# (degrees): straight on, the avatar's own head hides its name tag.
THIRD_PERSON_AIM_DEG = 15.0
# Walking on for STALL_S without getting STALL_M ahead (odometry: the
# avatar's own velocity): something is in the way, too near to be told from
# the avatar itself in third person. Taken as blocked for STUCK_S.
STALL_S = 0.8
STALL_M = 0.15
STUCK_S = 2.0
# The depth grid asked for: 20 px cells on a 1280x720 frame.
GRID_COLS = 64
GRID_ROWS = 36
# The body under a tag: from just under the tag down BODY_PX (frame px, at
# 720 rows), as wide as the tag; its distance is the BODY_QUANTILE of those
# cells (the body is nearer than what shows around it).
BODY_PX = 160
BODY_QUANTILE = 0.25
# Mouse look, measured at 1280x720 with the default desktop FOV: a full turn
# is FULL_TURN_PX of relative mouse motion (against the view: 3665 turned
# 367.1 degrees, again and again within 0.3; 5 turns of 3594 came 3.2
# short), and the view's focal length is FOCAL_PX (a point x px right of the
# centre is atan(x / FOCAL_PX) away).
FULL_TURN_PX = 3600
FOCAL_PX = 566.0
# Turns smaller than this (mouse px, ~1.5 degrees) are left alone; a turn
# corrects TURN_GAIN of the measured angle (the target moves meanwhile).
DEADZONE_PX = 15
TURN_GAIN = 0.7
# A turn larger than this (mouse px, ~12 degrees) slows the walk to
# ALIGN_SLOW of its speed; one larger than WALK_STOP_PX (~30 degrees) is made
# standing: walking on while turning that far heads off sideways.
WALK_ALIGN_PX = 120
WALK_STOP_PX = 300
ALIGN_SLOW = 0.5
# The target unseen this long: go and look for them.
LOST_AFTER = 1.0
# Far away a tag is small and OCR misses it now and then: wait longer.
LOST_AFTER_FAR = 2.0
# Dead reckoning: the forward axis's speed in world m/s (measured; linear in
# between).
AXIS_SPEEDS = ((0.0, 0.0), (0.3, 0.65), (0.5, 1.8), (1.0, 4.4))
# Lost, the avatar heads for the target's last known position moved on by
# their velocity (both Kalman-filtered, tracking.TargetTrack) for the time
# since plus PREDICT_LEAD s, at most PREDICT_MAX_M on: around a corner, that
# is the corner. It walks there at SEEK_SPEED, turning first when off by more
# than SEEK_ALIGN_DEG, and gives up (to search on the spot) when within
# SEEK_ARRIVE_M, blocked, or after SEEK_MAX_S.
PREDICT_LEAD = 1.0
PREDICT_MAX_M = 3.0
SEEK_SPEED = 0.5
SEEK_ALIGN_DEG = 12.0
SEEK_ARRIVE_M = 0.8
SEEK_MAX_S = 8.0
# Searching turns SEARCH_DEG at a time toward where the target was last seen,
# reading a frame taken after each turn.
SEARCH_DEG = 60.0
# Frames taken before a turn ended + SETTLE are skipped: the game (20 fps)
# shows a turn a few frames late, and the capture adds its own delay.
SETTLE = 0.4
# Following ends when the target has not been in the room this long.
GONE_AFTER = 10.0
MATCH_RATIO = 0.6


def normalized(text: str) -> str:
    return "".join(text.lower().split())


def match_score(line: str, name: str) -> float:
    """How well an OCR line matches a display name (0..1): the best of the
    whole line and the name found inside a longer line."""
    a, b = normalized(line), normalized(name)
    if not a or not b:
        return 0.0
    if b in a:
        return 1.0
    return difflib.SequenceMatcher(None, a, b).ratio()


def axis_for(speed: float) -> float:
    """The forward axis value (0..1) for ``speed`` world m/s (AXIS_SPEEDS)."""
    for (a0, v0), (a1, v1) in zip(AXIS_SPEEDS, AXIS_SPEEDS[1:], strict=False):
        if speed <= v1:
            return a0 + (a1 - a0) * (speed - v0) / (v1 - v0)
    return AXIS_SPEEDS[-1][0]


def axis_speed(axis: float) -> float:
    """World m/s of a forward axis value in 0..1 (AXIS_SPEEDS, linear)."""
    for (a0, v0), (a1, v1) in zip(AXIS_SPEEDS, AXIS_SPEEDS[1:], strict=False):
        if axis <= a1:
            return v0 + (v1 - v0) * (axis - a0) / (a1 - a0)
    return AXIS_SPEEDS[-1][1]


def wrap(angle: float) -> float:
    """An angle in -pi..pi."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class DepthGrid:
    """A depth answer: ``cols`` x ``rows`` cells of metres over the frame."""

    def __init__(self, answer: dict, width: int, height: int) -> None:
        self.cols, self.rows = int(answer["cols"]), int(answer["rows"])
        self.depth = answer["depth"]
        self.cell_w, self.cell_h = width / self.cols, height / self.rows

    def cells(self, x0: float, y0: float, x1: float, y1: float) -> list[float]:
        """The depths of the cells a frame rectangle covers (at least one)."""
        c0 = min(self.cols - 1, max(0, int(x0 / self.cell_w)))
        c1 = min(self.cols, max(c0 + 1, math.ceil(x1 / self.cell_w)))
        r0 = min(self.rows - 1, max(0, int(y0 / self.cell_h)))
        r1 = min(self.rows, max(r0 + 1, math.ceil(y1 / self.cell_h)))
        return [self.depth[r * self.cols + c] for r in range(r0, r1) for c in range(c0, c1)]


def quantile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def body_distance(grid: DepthGrid, tag: dict, height: int) -> float:
    """How far the body under a name tag is (metres)."""
    scale = height / 720
    top = tag["y"] + tag["height"] + 4 * scale
    return quantile(grid.cells(tag["x"], top, tag["x"] + tag["width"], top + BODY_PX * scale),
                    BODY_QUANTILE)


def obstacle_distance(grid: DepthGrid, width: int, height: int,
                      seen: nav.Seen = nav.FIRST_PERSON, target: dict | None = None) -> float:
    """The nearest thing in the middle of the view at body height, from the
    avatar: the target itself (the columns of its tag, ``target``) left out,
    and in third person the avatar's own body and anything as near as it
    (what it holds sticks out; a thing that close is found by the avatar
    getting nowhere: Follower._stalled)."""
    behind = seen.offset_m * DEPTH_SCALE
    own = seen.me.far * nav.OWN_MARGIN if seen.me is not None else 0.0
    c0, c1 = int(0.4 * grid.cols), int(0.6 * grid.cols)
    r0, r1 = int(0.35 * grid.rows), int(0.7 * grid.rows)
    skip = range(0)
    if target is not None:
        skip = range(int(target["x"] / grid.cell_w),
                     math.ceil((target["x"] + target["width"]) / grid.cell_w))
    ahead = [grid.depth[r * grid.cols + c] - behind for r in range(r0, r1) for c in range(c0, c1)
             if c not in skip and (seen.me is None or not seen.me.covers(c, r))
             and grid.depth[r * grid.cols + c] > max(behind, own)]
    return min(ahead, default=99.0)


def ground_distance(grid: DepthGrid, tag: dict, height: int, floor: nav.Floor | None,
                    seen: nav.Seen = nav.FIRST_PERSON) -> float | None:
    """How far (model metres, along the view) the target stands, by where
    its body under the tag meets the floor: down the middle of the tag's
    columns, the body is nearer than the floor behind it; the first row
    where the floor shows again is at its feet. None when the feet are not
    in view or the fit does not hold (a target on a step, the floor unseen)."""
    if floor is None:
        return None
    x0 = tag["x"] + tag["width"] * 0.3
    x1 = tag["x"] + tag["width"] * 0.7
    c0 = min(grid.cols - 1, max(0, int(x0 / grid.cell_w)))
    c1 = min(grid.cols, max(c0 + 1, math.ceil(x1 / grid.cell_w)))
    start = int((tag["y"] + tag["height"]) / grid.cell_h) + 1
    body_rows = 0
    for r in range(start, grid.rows):
        expected = floor.depth_at((r + 0.5) * grid.cell_h)
        values = [grid.depth[r * grid.cols + c] for c in range(c0, c1)
                  if seen.me is None or not seen.me.covers(c, r)]
        if expected is None or not values:
            continue
        nearest = min(values)
        if nearest < FEET_NEAR * expected:
            body_rows += 1
        elif body_rows >= 2:
            return expected
    return None


def target_distance(grid: DepthGrid, tag: dict, height: int, floor: nav.Floor | None,
                    seen: nav.Seen = nav.FIRST_PERSON) -> float:
    """How far the target is from the avatar (model metres): by its feet on
    the floor if that agrees roughly with its body's depth, else by the
    body; less the camera's distance behind the avatar in third person."""
    body = body_distance(grid, tag, height)
    ground = ground_distance(grid, tag, height, floor, seen)
    far = ground if ground is not None and body * 0.8 <= ground <= body * GROUND_MAX else body
    return max(0.1, far - seen.offset_m * DEPTH_SCALE)


class Follower:
    def __init__(self, bridge, ocr_url: str, ocr_model: str, ocr_token: str = "",
                 depth_url: str = "", depth_model: str = "") -> None:
        self.bridge = bridge
        self.ocr_url = ocr_url
        self.ocr_model = ocr_model
        self.ocr_token = ocr_token
        self.depth_url = depth_url
        self.depth_model = depth_model
        self.target = ""
        self.state = "idle"  # idle | following | seeking | searching
        self.last_seen = 0.0
        self.distance = 0.0  # metres to the target when last seen
        self.obstacle = 0.0  # metres to the nearest thing ahead
        self.last_offset = 0.0
        self._aside_side = 1  # the side the target is kept on in third person
        self._forward_since: tuple[float, float, float] | None = None  # when, x, y
        self._stuck_until = 0.0
        self._camera_third = False  # switched to third person while searching
        self.last_turn = 0
        self.move = 0.0  # -BACK_SPEED, 0 or 1 (forward)
        self.close = False  # the last sighting was within standing distance
        self.stand_m = STAND_M
        self.hold = False  # stay put: keep facing the target, do not walk
        self.ocr_ms = 0.0
        self.depth_ms = 0.0
        self._task: asyncio.Task | None = None
        self._hold: asyncio.TimerHandle | None = None
        self._still_since = 0.0
        # Dead reckoning (world metres; heading in radians, right positive):
        # the avatar's pose since the follow began, and the target's track.
        self.pose = [0.0, 0.0, 0.0]
        self._odo_at = time.monotonic()
        self.track = TargetTrack()
        self.goal: tuple[float, float] | None = None
        self._seek_since = 0.0
        self._seek_side = 1
        # Per frame: what was seen and done (GET /v1/follow?trace=1).
        self.trace: collections.deque[dict] = collections.deque(maxlen=300)

    def status(self) -> dict:
        return {"target": self.target, "state": self.state,
                "last_seen_s": round(max(0.0, time.monotonic() - self.last_seen), 1)
                if self.last_seen else None,
                "distance_m": round(self.distance, 2), "obstacle_m": round(self.obstacle, 2),
                "stand_m": self.stand_m, "distance": round(self.stand_m / DEPTH_SCALE, 1),
                "hold": self.hold, "move": self.move,
                "pose": [round(v, 2) for v in self.pose],
                "goal": None if self.goal is None else [round(v, 2) for v in self.goal],
                "target_velocity": [round(v, 2) for v in self.track.velocity()],
                "ocr_ms": round(self.ocr_ms), "depth_ms": round(self.depth_ms)}

    def _present(self, name: str) -> str:
        """The display name of the player in the room who best matches
        ``name``; empty when nobody does (or the room is not known)."""
        here = [n for uid, n in self.bridge.state.players.items() if uid != self.bridge.state.self_id]
        best = max(here, key=lambda n: match_score(n, name), default="")
        return best if best and match_score(best, name) >= 0.8 else ""

    def start(self, name: str, distance: float | None = None) -> None:
        """Follows ``name``, standing ``distance`` world metres away (default
        about STAND_M / DEPTH_SCALE)."""
        self.stop()
        self.stand_m = STAND_M if distance is None else self._clamp(float(distance) * DEPTH_SCALE)
        self.hold = False
        self.target = self._present(name) or name
        self.state = "following"
        self.last_seen = time.monotonic()
        self.last_offset = 0.0
        self.move = 0.0
        self.close = False
        self.pose = [0.0, 0.0, 0.0]
        self._odo_at = time.monotonic()
        self.track.reset()
        self.goal = None
        self._task = asyncio.create_task(self._run(self.target))

    @staticmethod
    def _clamp(stand_m: float) -> float:
        return round(max(MIN_STAND_M, min(MAX_STAND_M, stand_m)), 2)

    def adjust(self, change: str) -> None:
        """While following: ``closer`` / ``farther`` (the standing distance),
        ``stay`` (keep facing them, do not walk) or ``resume``."""
        if self._task is None:
            raise RuntimeError("not following anyone")
        if change == "closer":
            self.stand_m, self.hold = self._clamp(self.stand_m * CLOSER), False
        elif change == "farther":
            self.stand_m, self.hold = self._clamp(self.stand_m * FARTHER), False
        elif change == "stay":
            self.hold = True
            self._walk(0.0)
        elif change == "resume":
            self.hold = False
        else:
            raise ValueError("change must be closer, farther, stay or resume")

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        if self.state != "idle":
            self._halt()
            self.state = "idle"
            self.bridge.notify_state()

    def _advance(self) -> None:
        """Moves the dead-reckoned pose on by the current move."""
        now = time.monotonic()
        dt, self._odo_at = now - self._odo_at, now
        if self.move:
            v = math.copysign(axis_speed(abs(self.move)), self.move) * dt
            self.pose[0] += v * math.cos(self.pose[2])
            self.pose[1] += v * math.sin(self.pose[2])

    def _halt(self) -> None:
        self._advance()
        osc = self.bridge.osc
        osc.send("/input/Vertical", 0.0)
        osc.send("/input/Run", 0)
        self.move = 0.0
        if self._hold is not None:
            self._hold.cancel()
            self._hold = None

    def _walk(self, move: float, run: bool = False) -> None:
        """Moves at ``move`` (-1..1) for MOVE_HOLD s unless renewed."""
        self._advance()
        osc = self.bridge.osc
        if move <= 0:
            self._forward_since = None
        elif self.move <= 0 or self._forward_since is None:
            walker = getattr(self.bridge, "navigator", None)
            if walker is not None:
                self._forward_since = (time.monotonic(), walker.x, walker.y)
        self.move = move
        osc.send("/input/Vertical", float(move))
        osc.send("/input/Run", 1 if run else 0)
        if self._hold is not None:
            self._hold.cancel()
            self._hold = None
        if move:
            self._hold = asyncio.get_running_loop().call_later(MOVE_HOLD, self._halt)

    async def _camera(self, third: bool) -> None:
        """Puts the game's camera in third person (searching) or first
        (following); frames from before it settles are skipped."""
        set_camera = getattr(self.bridge, "set_camera", None)
        if set_camera is None:
            return
        await set_camera(third)
        self._camera_third = third
        self._still_since = time.monotonic() + SETTLE

    def _stalled(self, now: float) -> bool:
        """Whether walking on gets the avatar nowhere (or did just now)."""
        if now < self._stuck_until:
            return True
        walker = getattr(self.bridge, "navigator", None)
        if walker is None or not walker.odometry or self._forward_since is None:
            return False
        since, x, y = self._forward_since
        if now - since < STALL_S:
            return False
        if math.hypot(walker.x - x, walker.y - y) < STALL_M:
            self._stuck_until = now + STUCK_S
            self._forward_since = None
            return True
        self._forward_since = (now, walker.x, walker.y)  # moving: watch the next stretch
        return False

    async def _turn(self, px: int) -> None:
        """Turns the view right by ``px`` of mouse motion (negative: left);
        frames taken before it settles are skipped."""
        self._advance()
        await self.bridge.look(dx=px)
        self.pose[2] += px * 2 * math.pi / FULL_TURN_PX
        walker = getattr(self.bridge, "navigator", None)
        if walker is not None:
            walker.turned(px * 360 / FULL_TURN_PX)  # the scene map's pose turns too
        self._still_since = time.monotonic() + SETTLE

    async def _frames(self):
        """JPEG frames of the game window at FPS."""
        args = self.bridge.args
        proc = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", "-f", "x11grab", "-framerate", str(FPS),
            "-video_size", args.screen, "-i", args.display,
            "-c:v", "mjpeg", "-q:v", "4", "-f", "image2pipe", "-",
            stdout=asyncio.subprocess.PIPE, env=self.bridge._env)
        buffer = b""
        try:
            assert proc.stdout is not None
            while chunk := await proc.stdout.read(65536):
                buffer += chunk
                while (end := buffer.find(b"\xff\xd9")) >= 0:
                    start = buffer.find(b"\xff\xd8")
                    frame, buffer = buffer[start:end + 2], buffer[end + 2:]
                    if start >= 0:
                        yield frame
        finally:
            with contextlib.suppress(ProcessLookupError):
                proc.kill()
            await proc.wait()

    async def _infer(self, http: aiohttp.ClientSession, url: str, params: dict,
                     frame: bytes) -> tuple[dict, float]:
        """One of infra's direct image endpoints: its answer, and ms."""
        headers = {"Content-Type": "image/jpeg"}
        if self.ocr_token:
            headers["Authorization"] = f"Bearer {self.ocr_token}"
        started = time.monotonic()
        async with http.post(url, params=params, data=frame, headers=headers,
                             timeout=aiohttp.ClientTimeout(total=5)) as resp:
            body = await resp.json(content_type=None)
            if resp.status != 200:
                raise RuntimeError(body.get("error") or f"HTTP {resp.status}")
        return body, (time.monotonic() - started) * 1000

    async def _ocr(self, http: aiohttp.ClientSession, frame: bytes) -> list[dict]:
        body, self.ocr_ms = await self._infer(http, self.ocr_url, {"model": self.ocr_model}, frame)
        return body.get("lines") or []

    async def _depth(self, http: aiohttp.ClientSession, frame: bytes, width: int,
                     height: int) -> DepthGrid | None:
        if not self.depth_url:
            return None
        params = {"model": self.depth_model, "cols": GRID_COLS, "rows": GRID_ROWS}
        body, self.depth_ms = await self._infer(http, self.depth_url, params, frame)
        return DepthGrid(body, width, height)

    async def _face(self, cx: float, width: int, aside_deg: float = 0.0) -> None:
        """Turns toward screen column ``cx`` (or so that it is ``aside_deg``
        to the side it is on: past the avatar's own head in third person)."""
        offset = cx - width / 2
        self.last_offset = offset
        if aside_deg:
            aside = FOCAL_PX * math.tan(math.radians(aside_deg)) * width / 1280
            offset -= math.copysign(aside, offset if abs(offset) > 1 else self._aside_side)
            self._aside_side = 1 if cx > width / 2 else -1
        px = round(math.atan(offset / FOCAL_PX) * FULL_TURN_PX / (2 * math.pi) * TURN_GAIN)
        self.last_turn = px if abs(px) >= DEADZONE_PX else 0
        if self.last_turn:
            await self._turn(px)

    def _mark(self, now: float, offset: float, distance: float) -> None:
        """Records where the target is: ``offset`` px right of the view's
        centre, ``distance`` world metres away."""
        self._advance()
        x, y, heading = self.pose
        angle = heading + math.atan(offset / FOCAL_PX)
        self.track.update(now, (x, y), (x + distance * math.cos(angle),
                                        y + distance * math.sin(angle)))

    async def _seek(self, now: float) -> bool:
        """One step toward the goal; True once there (or blocked, or out of
        time): time to search on the spot."""
        self._advance()
        x, y, heading = self.pose
        gx, gy = self.goal
        if (math.hypot(gx - x, gy - y) < SEEK_ARRIVE_M or now - self._seek_since > SEEK_MAX_S
                or self.obstacle < OBSTACLE_M):
            return True
        bearing = wrap(math.atan2(gy - y, gx - x) - heading)
        if abs(bearing) > math.radians(SEEK_ALIGN_DEG):
            self._walk(0.0)
            px = round(bearing * FULL_TURN_PX / (2 * math.pi))
            self.last_turn = px
            await self._turn(px)
        else:
            self._walk(SEEK_SPEED)
        return False

    def _keep_distance(self, distance: float, blocked: bool, at_top: bool = False,
                       age: float = 0.0) -> None:
        stand = self.stand_m
        walk, back = stand * WALK_M / STAND_M, stand * BACK_M / STAND_M
        # Where they are now: the frame is ``age`` s old, and the avatar has
        # walked on since (model metres).
        if self.move > 0:
            distance -= axis_speed(self.move) * DEPTH_SCALE * age
        self.close = distance <= walk or at_top
        # World m/s that reach the standing line in BRAKE_S (model metres / 2).
        reach = max(0.0, distance - stand) / DEPTH_SCALE / BRAKE_S
        speed = max(MIN_SPEED, min(MAX_SPEED, axis_for(reach)))
        if abs(self.last_turn) > WALK_ALIGN_PX:
            speed = max(MIN_SPEED, speed * ALIGN_SLOW)  # turning on the way
        if at_top:
            distance = min(distance, stand)  # close, whatever the depth says
        if self.hold:
            self._walk(0.0)  # staying put: only facing them
        elif distance < back:
            self._walk(-BACK_SPEED)
        elif abs(self.last_turn) > WALK_STOP_PX or blocked or distance <= stand:
            self._walk(0.0)  # turning far, something ahead, or close
        elif distance > walk or self.move > 0:
            self._walk(speed)
        else:  # in the band, standing or backing off: stand
            self._walk(0.0)

    async def _run(self, name: str) -> None:
        width, height = (int(v) for v in self.bridge.args.screen.split("x"))
        walker = getattr(self.bridge, "navigator", None)
        if walker is not None:
            with contextlib.suppress(Exception):
                await walker.level()  # looking level: name tags and depth as tuned
        searching_since = 0.0
        self._still_since = 0.0
        last_here = time.monotonic()
        # Only the newest frame matters: a reader keeps it (with the time it
        # arrived) while the models run.
        latest: list[tuple[float, bytes]] = []
        fresh = asyncio.Event()

        async def read_frames() -> None:
            try:
                async for frame in self._frames():
                    latest[:] = [(time.monotonic(), frame)]
                    fresh.set()
            finally:
                fresh.set()  # wakes the loop to see the reader is done

        reader = asyncio.create_task(read_frames())
        try:
            async with aiohttp.ClientSession() as http:
                while not reader.done():
                    await fresh.wait()
                    fresh.clear()
                    if not latest:
                        break  # the reader ended before any frame
                    taken, frame = latest[0]
                    now = time.monotonic()
                    if not self.bridge.state.running:
                        break  # the game is gone: no OCR, no mouse
                    if self._present(name) or not self.bridge.state.players:
                        last_here = now
                    elif now - last_here > GONE_AFTER:
                        await self.bridge.send_event({"type": "follow", "state": "gone",
                                                      "target": name})
                        break
                    if taken < self._still_since:
                        continue  # taken before the last turn settled
                    lines, grid = await asyncio.gather(
                        self._ocr(http, frame), self._depth(http, frame, width, height),
                        return_exceptions=True)
                    failed = next((r for r in (lines, grid) if isinstance(r, Exception)), None)
                    if failed is not None:
                        log.warning("vision failed: %s", failed)
                        await asyncio.sleep(0.5)
                        continue
                    now = time.monotonic()
                    self.last_turn = 0
                    step: dict = {"t": round(now, 2), "age": round(now - taken, 2)}
                    self.trace.append(step)
                    sightings = getattr(self.bridge, "sightings", None)
                    if sightings is not None:
                        sightings.seen_in(lines, frame)
                    tag = max(lines, key=lambda line: match_score(line.get("text", ""), name),
                              default=None)
                    if tag is not None and match_score(tag.get("text", ""), name) < MATCH_RATIO:
                        tag = None
                    seen, floor = nav.FIRST_PERSON, None
                    if grid is not None:
                        if self._camera_third:
                            seen = nav.seen_in(grid.depth, grid.cols, grid.rows, height)
                        floor = nav.fit_floor(grid.depth, grid.cols, grid.rows, height, seen.me)
                        self.obstacle = obstacle_distance(grid, width, height, seen,
                                                          tag["bbox"] if tag else None)
                        step["obstacle"] = round(self.obstacle, 2)
                        if seen.me is not None:
                            step["behind"] = round(seen.offset_m, 2)
                    if tag is not None:
                        box = tag["bbox"]
                        step["tag"] = [tag.get("text", "")] + [round(box[k]) for k in
                                                                ("x", "y", "width", "height")]
                        if self.state != "following":
                            await self.bridge.send_event({"type": "follow", "state": "found",
                                                          "target": name})
                        if self._camera_third:
                            await self._camera(False)  # following by the first person view
                        self.state, self.last_seen, searching_since = "following", now, 0.0
                        self.goal = None
                        cx = box["x"] + box["width"] / 2
                        if grid is not None:
                            self.distance = target_distance(grid, box, height, floor, seen)
                            self._mark(now, cx - width / 2, self.distance / DEPTH_SCALE)
                        await self._face(cx, width, THIRD_PERSON_AIM_DEG if seen.me else 0.0)
                        if grid is not None:
                            self._keep_distance(self.distance,
                                                blocked=(self.obstacle < OBSTACLE_M
                                                         or self._stalled(now)),
                                                at_top=box["y"] < TAG_TOP * height / 720,
                                                age=now - taken)
                        else:  # no depth: face them, stand
                            self._walk(0.0)
                        step.update(distance=round(self.distance, 2), move=self.move,
                                    turn=self.last_turn)
                        continue
                    # Not seen in this frame: stand (walking on blind
                    # overshoots); once lost, go where they were heading,
                    # then search there.
                    lost_after = LOST_AFTER_FAR if not self.close else LOST_AFTER
                    if now - self.last_seen < lost_after:
                        if self.move:
                            self._walk(0.0)
                        continue
                    # Lost close by, the last position is about where the
                    # avatar stands: search on the spot rather than walk into
                    # them.
                    if (self.goal is None and not searching_since and self.track.active
                            and not self.hold and not self.close):
                        gx, gy = self.track.predict(now, PREDICT_LEAD, PREDICT_MAX_M)
                        vx, vy = self.track.velocity()
                        # Search first toward the side they were moving to.
                        across = math.cos(self.pose[2]) * vy - math.sin(self.pose[2]) * vx
                        self._seek_side = (1 if across > 0 else -1) if abs(across) > 0.2 else (
                            -1 if self.last_offset < 0 else 1)
                        if math.hypot(gx - self.pose[0], gy - self.pose[1]) > SEEK_ARRIVE_M:
                            self.goal, self._seek_since = (gx, gy), now
                            self.state = "seeking"
                            await self.bridge.send_event({"type": "follow", "state": "seeking",
                                                          "target": name})
                    if self.goal is not None:
                        arrived = await self._seek(now)
                        step.update(seek=[round(v, 2) for v in self.goal], move=self.move,
                                    turn=self.last_turn)
                        if not arrived:
                            continue
                        self.goal = None
                    if self.move:
                        self._walk(0.0)
                    if not searching_since:
                        searching_since = now
                        await self._camera(True)  # looking for them from behind: more in view
                        self.state = "searching"
                        await self.bridge.send_event({"type": "follow", "state": "searching",
                                                      "target": name})
                    await self._turn(self._seek_side * round(FULL_TURN_PX * SEARCH_DEG / 360))
                    step.update(search=self._seek_side)
        except asyncio.CancelledError:
            raise
        except Exception:  # noqa: BLE001 - reported
            log.exception("follow failed")
        finally:
            reader.cancel()
            with contextlib.suppress(Exception):
                await self._camera(False)  # first person again, whatever happened
        self._halt()
        self.state, self._task = "idle", None
        self.bridge.notify_state()
