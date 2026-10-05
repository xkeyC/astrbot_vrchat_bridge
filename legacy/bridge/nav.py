"""Where the avatar can walk, from one frame's metric depth (VLMnav-style).

The model is shown its view with a few numbered places it can walk to, each
with its distance (plus 0: turn around), and answers with a number; the
bridge turns and walks there itself, braking before obstacles. Picking a
number is a question a VLM answers well; estimating angles and distances
from a picture, and walking blind, it does not.

Free space comes from the floor: on a flat floor, a cell ``y`` px below the
horizon is ``H * F / y`` away (camera height H, focal length F, both in the
depth model's metres and the frame's px). The fit of 1/depth over the rows
of the lower middle (floor, mostly) gives H and the horizon each frame, so
a look up or down does not matter. Going up a column from the bottom, the
floor lasts until a cell is markedly nearer than the floor there (an
obstacle) or markedly farther (an edge, a drop): that floor's distance is
how far the avatar can walk that way.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass

from drive import PX_PER_DEGREE_Y
from selfview import Self, find_self

# The depth model's metres per world metre (as the follower's DEPTH_SCALE).
DEPTH_SCALE = 2.0
# Bearings looked at, degrees (+ right); marks are picked among them.
BEARINGS = tuple(range(-45, 46, 5))
MAX_MARKS = 6
MARK_SPACING_DEG = 15
MIN_WALK_M = 0.8  # world metres: shorter ways are not offered
MAX_WALK_M = 6.0
WALK_SHARE = 2 / 3  # of the free distance (VLMnav): stop short of what blocks
NEAR = 0.8  # a cell nearer than NEAR x the floor's depth there is an obstacle
FAR = 1.6  # farther than FAR x the floor's depth: an edge or a drop
# Off the floor's plane but walkable: ground rising or falling at most this
# steeply (rise per run) from the point before (a grassy mound, a ramp).
MAX_SLOPE = 0.5
# In third person, a cell beside the avatar's body up to this much farther
# than the body may still be its own (depth blurs the edges of what it holds).
OWN_MARGIN = 1.2
BEEN_M = 1.5  # an end this near a visited point is marked as been
# A way is as free as its corridor: the bearing and this many degrees to
# each side (the avatar is not a ray), when picking places and walking.
CORRIDOR_DEG = 5


@dataclass(frozen=True)
class Floor:
    """The floor's fit: depth = height * focal / (y - horizon), y in px."""

    horizon: float
    height_focal: float

    def depth_at(self, y: float) -> float | None:
        below = y - self.horizon
        return self.height_focal / below if below > 1 else None

    def row_of(self, depth: float) -> float:
        """The image row (px) of the floor ``depth`` away."""
        return self.horizon + self.height_focal / depth


@dataclass(frozen=True)
class Mark:
    number: int
    bearing: float  # degrees, + right
    distance: float  # world metres to walk
    been: bool = False


def fit_floor(depth: list[float], cols: int, rows: int, height: int,
              me: Self | None = None) -> Floor | None:
    """The floor's fit from the lower middle of the frame (None: no floor
    there, e.g. facing a wall), the avatar's own body (``me``) left out."""
    cell_h = height / rows
    points = []
    for r in range(rows // 2 + 2, rows):
        middle = sorted(depth[r * cols + c] for c in range(cols // 3, 2 * cols // 3)
                        if me is None or not me.covers(c, r))
        if not middle:
            continue  # all of it the avatar's own body
        d = middle[len(middle) // 2]
        if d > 0:
            points.append(((r + 0.5) * cell_h, 1.0 / d))
    if len(points) < 4:
        return None
    # Least squares: 1/d = (y - horizon) / (H F) = a y + b.
    n = len(points)
    mean_y = sum(y for y, _ in points) / n
    mean_v = sum(v for _, v in points) / n
    var = sum((y - mean_y) ** 2 for y, _ in points)
    if var == 0:
        return None
    a = sum((y - mean_y) * (v - mean_v) for y, v in points) / var
    b = mean_v - a * mean_y
    if a <= 0:  # 1/d not growing down the frame: no floor (a wall, a ceiling)
        return None
    floor = Floor(horizon=-b / a, height_focal=1.0 / a)
    # Implausible horizons (far off the frame) are no floor either.
    if not -height < floor.horizon < height:
        return None
    # The fit must explain its rows: a wall in front fits a line badly.
    error = max(abs((v - (a * y + b)) / v) for y, v in points)
    return floor if error < 0.35 else None


def free_distance(depth: list[float], cols: int, rows: int, height: int,
                  floor: Floor, col: int, focal_px: float | None = None,
                  me: Self | None = None) -> float:
    """How far the floor goes up column ``col`` before something nearer
    (an obstacle) or farther (an edge) than the floor: depth-model metres.
    With ``focal_px``, ground off the floor's plane goes on too while it
    only rises or falls gently (a slope; a face or a step stops it). Cells
    that may be the avatar itself (``me``, third person: its body, or as
    near as it beside it, what it holds) are passed over, unknown."""
    cell_h = height / rows
    free, before = 0.0, None
    for r in range(rows - 1, -1, -1):
        v = (r + 0.5) * cell_h
        expected = floor.depth_at(v)
        if expected is None:
            break  # the horizon: as far as the floor goes
        d = depth[r * cols + col]
        if (me is not None and me.r0 <= r <= me.r1
                and (me.covers(col, r) or d <= me.far * OWN_MARGIN)):
            continue
        flat = NEAR * expected <= d <= FAR * expected
        if focal_px is None:
            if not flat:
                break
            free = expected
            continue
        above = (floor.height_focal - (v - floor.horizon) * d) / focal_px
        if not flat:
            if before is None or d - before[0] < 0.05:
                break  # no ground before it, or not going away: a face
            if abs(above - before[1]) > MAX_SLOPE * (d - before[0]):
                break
        before = (d, above)
        free = expected if flat else d
    return free


def column_of(bearing: float, width: int, cols: int, focal_px: float) -> int:
    x = width / 2 + focal_px * math.tan(math.radians(bearing))
    return min(cols - 1, max(0, int(x / (width / cols))))


@dataclass(frozen=True)
class Seen:
    """How a frame sees the avatar: the camera ``offset_m`` world metres
    behind its feet, its body ``me`` in the frame (third person), or
    neither (first person)."""

    me: Self | None = None
    offset_m: float = 0.0


FIRST_PERSON = Seen()


def seen_in(depth: list[float], cols: int, rows: int, height: int) -> Seen:
    me = find_self(depth, cols, rows)
    if me is None:
        return FIRST_PERSON
    floor = fit_floor(depth, cols, rows, height, me)
    feet = floor.depth_at((me.r1 + 1) * height / rows) if floor is not None else None
    return Seen(me, feet / DEPTH_SCALE if feet else THIRD_PERSON_OFFSET_M)


def free_ahead(depth: list[float], cols: int, rows: int, height: int, floor: Floor,
               col: int, focal_px: float, seen: Seen) -> float:
    """World metres free from the avatar up ``col``: where its own body
    hides the way, the way just beside it, either side, stands for it."""
    if seen.me is not None and seen.me.covers(col):
        sides = [c for c in (seen.me.c0 - 1, seen.me.c1 + 1) if 0 <= c < cols]
        free = min(free_distance(depth, cols, rows, height, floor, c, focal_px, seen.me)
                   for c in sides)
    else:
        free = free_distance(depth, cols, rows, height, floor, col, focal_px, seen.me)
    return max(0.0, free / DEPTH_SCALE - seen.offset_m)


def marks(depth: list[float], cols: int, rows: int, width: int, height: int,
          focal_px: float, been=lambda bearing, distance: False,
          seen: Seen = FIRST_PERSON) -> tuple[list[Mark], Floor | None]:
    """The numbered places to walk to (1..), best spread over the view:
    each bearing's free walk, then local maxima at least MARK_SPACING_DEG
    apart, the farthest first."""
    floor = fit_floor(depth, cols, rows, height, seen.me)
    if floor is None:
        return [], None
    ways = []
    for bearing in BEARINGS:
        free = min(free_ahead(depth, cols, rows, height, floor,
                              column_of(bearing + side, width, cols, focal_px), focal_px, seen)
                   for side in (-CORRIDOR_DEG, 0, CORRIDOR_DEG))
        walk = min(MAX_WALK_M, free * WALK_SHARE)
        if walk >= MIN_WALK_M:
            ways.append((walk, bearing))
    chosen: list[tuple[float, float]] = []
    for walk, bearing in sorted(ways, reverse=True):
        if all(abs(bearing - b) >= MARK_SPACING_DEG for _, b in chosen):
            chosen.append((walk, bearing))
        if len(chosen) == MAX_MARKS:
            break
    chosen.sort(key=lambda wb: wb[1])  # numbered left to right
    return [Mark(i, b, round(w, 1), been(b, w)) for i, (w, b) in enumerate(chosen, 1)], floor


def from_feet(mark: Mark, offset_m: float) -> Mark:
    """A mark of a third person view (its bearing seen from the camera,
    ``offset_m`` behind the avatar; its distance from the feet) as the
    avatar would walk it: bearing and distance from the feet."""
    if not offset_m:
        return mark
    h = math.radians(mark.bearing)
    far = mark.distance + offset_m
    ahead, right = far * math.cos(h) - offset_m, far * math.sin(h)
    return Mark(mark.number, round(math.degrees(math.atan2(right, ahead))),
                round(math.hypot(ahead, right), 1), mark.been)


def label_filters(found: list[Mark], floor: Floor | None, width: int, height: int,
                  focal_px: float, size: int = 24, offset_m: float = 0.0) -> list[str]:
    """ffmpeg filters drawing each mark where its walk ends: a dot and its
    number with the distance (gray when been there), plus 0 to turn around.
    A label that would cover another one goes above it."""
    filters = []
    placed: list[tuple[float, float, float]] = []  # left, right, top of the labels drawn
    for mark in sorted(found, key=lambda m: m.bearing):
        x = width / 2 + focal_px * math.tan(math.radians(mark.bearing))
        y = height * 0.8
        if floor is not None:
            y = min(height - 30, max(height * 0.45,
                                     floor.row_of((mark.distance + offset_m) * DEPTH_SCALE)))
        color = "gray" if mark.been else "cyan"
        dot = size // 2
        filters.append(f"drawbox=x={round(x) - dot // 2}:y={round(y) - dot // 2}:w={dot}:h={dot}:"
                       f"color={color}:t=fill")
        text = f"{mark.number} {mark.distance:.1f}m"
        half = len(text) * size * 0.3 + 4  # monospace: about 0.6 of the size a letter
        x = min(width - half, max(half, x))  # the label whole within the view
        top = y - size - 10
        while any(x - half < r and l < x + half and abs(top - t) < size + 4 for l, r, t in placed):
            top -= size + 4
        placed.append((x - half, x + half, top))
        filters.append(
            f"drawtext=font=monospace:text='{text}':x={round(x)}-tw/2:y={round(top)}:"
            f"fontsize={size}:fontcolor={color}:borderw={max(3, size // 8)}:bordercolor=black")
    filters.append(
        "drawtext=font=monospace:text='0 turn around':x=(w-tw)/2:y=h-62:"
        "fontsize=22:fontcolor=cyan:borderw=3:bordercolor=black")
    return filters


# -- pointing at things: grid cells ---------------------------------------------

CELL_COLS, CELL_ROWS = 8, 5
CELL_LETTERS = "ABCDEFGH"


def cell_filters(width: int, height: int) -> list[str]:
    """A faint grid over the view, columns A.. along the bottom and rows 1..
    along the left, so the model can point at a thing by its cell."""
    filters = []
    cw, ch = width / CELL_COLS, height / CELL_ROWS
    for c in range(1, CELL_COLS):
        filters.append(f"drawbox=x={round(c * cw)}:y=0:w=1:h={height}:color=white@0.25:t=fill")
    for r in range(1, CELL_ROWS):
        filters.append(f"drawbox=x=0:y={round(r * ch)}:w={width}:h=1:color=white@0.25:t=fill")
    for c in range(CELL_COLS):
        filters.append(
            f"drawtext=font=monospace:text='{CELL_LETTERS[c]}':x={round((c + 0.5) * cw)}-tw/2:"
            f"y=h-28:fontsize=22:fontcolor=white@0.8:borderw=2:bordercolor=black")
    for r in range(CELL_ROWS):
        filters.append(
            f"drawtext=font=monospace:text='{r + 1}':x=6:y={round((r + 0.5) * ch)}-th/2:"
            f"fontsize=22:fontcolor=white@0.8:borderw=2:bordercolor=black")
    return filters


def parse_cell(cell: str) -> tuple[int, int]:
    """("D", 4) of "D4" as 0-based (column, row).

    Raises:
        ValueError: Not a cell of the grid.
    """
    text = str(cell).strip().upper()
    if len(text) < 2 or text[0] not in CELL_LETTERS or not text[1:].isdigit():
        raise ValueError(f"a cell is a letter A-{CELL_LETTERS[-1]} and a row 1-{CELL_ROWS}")
    col, row = CELL_LETTERS.index(text[0]), int(text[1:]) - 1
    if not 0 <= row < CELL_ROWS:
        raise ValueError(f"rows are 1-{CELL_ROWS}")
    return col, row


def cell_target(cell: str, depth: dict, width: int, height: int,
                focal_px: float, typical: bool = False,
                seen: Seen = FIRST_PERSON) -> tuple[float, float]:
    """The bearing (degrees) and world distance (m, from the avatar) of what
    is in ``cell`` of a view: its centre column, and the nearest depth over
    its middle (to walk up to it), or with ``typical`` the median one short
    of the sky (where the thing is, past what stands before it); the
    avatar's own body left out."""
    col, row = parse_cell(cell)
    cw, ch = width / CELL_COLS, height / CELL_ROWS
    x = (col + 0.5) * cw
    bearing = math.degrees(math.atan((x - width / 2) / focal_px))
    gw, gh = width / depth["cols"], height / depth["rows"]
    cells = [(c, r) for r in range(int(row * ch / gh), int((row + 1) * ch / gh))
             for c in range(int(col * cw / gw), int((col + 1) * cw / gw))]
    if seen.me is not None and sum(seen.me.covers(c, r) for c, r in cells) * 2 >= len(cells):
        raise ValueError(f"{cell} is you (your view is from behind you): point at the thing "
                         "in another cell")
    # Its middle, the avatar's body left out (the whole cell when only its
    # edge is not the body).
    c0, c1 = int((col + 0.25) * cw / gw), max(int((col + 0.75) * cw / gw), int((col + 0.25) * cw / gw) + 1)
    r0, r1 = int((row + 0.25) * ch / gh), max(int((row + 0.75) * ch / gh), int((row + 0.25) * ch / gh) + 1)
    middle = [(c, r) for c, r in cells if c0 <= c < c1 and r0 <= r < r1]
    if seen.me is not None:
        middle = [(c, r) for c, r in middle if not seen.me.covers(c, r)] or [
            (c, r) for c, r in cells if not seen.me.covers(c, r)]
    values = sorted(depth["depth"][r * depth["cols"] + c] for c, r in middle)
    if typical:
        solid = [v for v in values if v / DEPTH_SCALE < SKY_M] or values[:1]
        far = solid[len(solid) // 2] / DEPTH_SCALE
    else:
        far = values[0] / DEPTH_SCALE
    return round(bearing, 1), max(0.3, far - seen.offset_m)


# -- walking there ---------------------------------------------------------------

GRID_COLS, GRID_ROWS = 64, 36
WALK_AXIS = 0.5  # forward axis while walking to a mark (~1.8 world m/s)
STOP_M = 0.7  # world metres of free floor ahead below which a walk stops
STOP_NOW_M = 0.4  # nearer than this stops at once; else two looks in a row
# Walking on this long without getting this far (odometry): something is in
# the way, unseen (too near, or hidden behind the avatar in third person).
STALL_S, STALL_M = 0.8, 0.2
GOTO_LIMIT_S = 15.0  # a walk to a mark, detours included, never lasts longer
ARRIVE_M = 0.4  # this near the place is there
MAX_DETOURS = 3
DETOUR_M = 1.5  # at most this far aside per detour
DETOURS = ("auto", "left", "right", "none")
PATH_LIMIT_S = 40.0  # a walk along a planned path never lasts longer
DEFAULT_SELF = Self(28, 36, 19, 31, far=4.0)  # where the body usually is (1280x720, 64x36)
# Looking up or down adds up (the game has no "look level"): the view is put
# level again by undoing the looks sent since it was last level (LEVEL_DEG
# off is level enough). Tried and dropped: the floor's horizon (slopes and
# mounds lean it), a look down to the end and back up (left it looking up).
LEVEL_DEG = 2.0
MAX_PITCH_DEG = 89.0
LEVEL_SETTLE_S = 0.25
SELF_MISSES = 2  # third person ends after this many frames in a row without the body, +1
THIRD_PERSON_OFFSET_M = 1.8  # the camera behind the feet when the floor fit fails
SKY_M = 15.0  # world metres of depth beyond which it is sky or far away
UP_TO_PAST_M = 1.0  # walking up to a thing goes at most this far past its depth


class Navigator:
    """Shows the view with its marks, walks to one, and keeps a rough pose
    (dead reckoning from turns and walks) to mark places already been."""

    def __init__(self, bridge, depth_url: str, depth_model: str, focal_px: float,
                 axis_speed) -> None:
        self.bridge = bridge
        self.depth_url = depth_url
        self.depth_model = depth_model
        self.focal_px = focal_px
        self.axis_speed = axis_speed  # world m/s of a forward axis value
        self.x = self.y = self.heading = 0.0  # world metres, degrees (+ right)
        self.visited: list[tuple[float, float]] = [(0.0, 0.0)]
        self.marks: list[Mark] = []
        self.last_view: tuple[dict, int, int, Seen] | None = None
        self.seen = FIRST_PERSON  # how the last frame saw the avatar (third person or not)
        self.camera = "first"  # the game's camera as the bridge set it (F5): first or third
        # How far it looks down (degrees, - up) from level: the mouse motion
        # sent since the model last said the view was level (vrchat_camera_y).
        self.pitch = 0.0
        self._missed = 0  # frames in a row without the body, in third person
        self.odometry = False  # the avatar's own motion moves the pose (odometry.py)
        self.scene = None  # scene.SceneMap of this instance, built as frames come
        self._scene_instance = ""

    # -- pose --------------------------------------------------------------

    def looked(self, dy_px: int) -> None:
        """Mouse motion sent up or down (+ down; the game stops at straight)."""
        self.pitch = max(-MAX_PITCH_DEG,
                         min(MAX_PITCH_DEG, self.pitch + dy_px / PX_PER_DEGREE_Y))

    def turned(self, degrees: float) -> None:
        """A turn sent: mouse motion turns the view exactly (calibrated)."""
        self.heading = (self.heading + degrees) % 360

    def walked(self, forward_m: float, right_m: float = 0.0) -> None:
        """Dead reckoning of a walk sent (none while odometry tracks it)."""
        if self.odometry:
            return
        h = math.radians(self.heading)
        self.moved(self.x + forward_m * math.cos(h) - right_m * math.sin(h),
                   self.y + forward_m * math.sin(h) + right_m * math.cos(h), self.heading)

    def moved(self, x: float, y: float, heading: float) -> None:
        self.x, self.y, self.heading = x, y, heading
        if math.dist((self.x, self.y), self.visited[-1]) >= 0.5:
            self.visited.append((self.x, self.y))
            del self.visited[:-200]

    def been(self, bearing: float, walk: float) -> bool:
        h = math.radians(self.heading + bearing)
        end = (self.x + walk * math.cos(h), self.y + walk * math.sin(h))
        here = (self.x, self.y)
        return any(math.dist(end, p) < BEEN_M and math.dist(here, p) > 1.0
                   for p in self.visited)

    def summary(self) -> str:
        """Where the avatar is, roughly, from where it started driving."""
        dist = math.hypot(self.x, self.y)
        return (f"~{dist:.0f} m from where you started driving, turned "
                f"{(self.heading + 180) % 360 - 180:+.0f} degrees since")

    # -- seeing ------------------------------------------------------------

    async def frame_depth(self) -> tuple[bytes, dict, int, int]:
        import aiohttp

        width, height = (int(v) for v in self.bridge.args.screen.split("x"))
        for attempt in range(2):  # a garbled answer now and then: once more
            jpeg = await self.bridge.screenshot("jpg")
            try:
                async with aiohttp.ClientSession() as http, http.post(
                    self.depth_url, data=jpeg, headers={"Content-Type": "image/jpeg"},
                    params={"model": self.depth_model, "cols": GRID_COLS, "rows": GRID_ROWS},
                    timeout=aiohttp.ClientTimeout(total=5),
                ) as resp:
                    depth = await resp.json(content_type=None)
                    if resp.status != 200:
                        raise RuntimeError(depth.get("error") or f"depth HTTP {resp.status}")
                break
            except (aiohttp.ClientError, asyncio.TimeoutError, ValueError, RuntimeError):
                if attempt:
                    raise
                await asyncio.sleep(0.2)
        self._see_self(depth, height)
        self._map_frame(depth, width, height)
        return jpeg, depth, width, height

    async def level(self) -> float:
        """Looks level again: the looks up and down sent since the view was
        last level undone (the game's camera moves by nothing else; level is
        what the model said it was, vrchat_camera_y). The pitch it undid
        (degrees, + down)."""
        down = self.pitch
        if abs(down) >= LEVEL_DEG:
            await self.bridge.look(dy=-round(down * PX_PER_DEGREE_Y))  # - dy: up
            self.pitch = 0.0
            await asyncio.sleep(LEVEL_SETTLE_S)
        return down

    def _see_self(self, depth: dict, height: int) -> None:
        """How this frame sees the avatar; a third person camera is kept
        through a few frames that miss its body (one look away, a blur)."""
        if self.camera == "first":
            self.seen = FIRST_PERSON
            return
        seen = seen_in(depth["depth"], depth["cols"], depth["rows"], height)
        if seen.me is None and self.seen.me is None:
            seen = Seen(DEFAULT_SELF, THIRD_PERSON_OFFSET_M)  # third person, body not made out
        if seen.me is None and self.seen.me is not None and self._missed < SELF_MISSES:
            self._missed += 1
            return  # still third person: the last body found stands
        self._missed = 0
        self.seen = seen

    def _map_frame(self, depth: dict, width: int, height: int) -> None:
        """Adds a frame to the scene map (a new one in another instance)."""
        import scene

        instance = getattr(self.bridge.state, "instance", "")
        if self.scene is None or instance != self._scene_instance:
            self.scene, self._scene_instance = scene.SceneMap(), instance
            self.x = self.y = self.heading = 0.0
            self.visited = [(0.0, 0.0)]
        cols, me = depth["cols"], self.seen.me
        floor = fit_floor(depth["depth"], cols, depth["rows"], height, me)
        if floor is not None:
            grid = depth["depth"]
            if me is not None:  # the avatar's body is no obstacle where it stands
                grid = [0.0 if me.covers(i % cols, i // cols) else d for i, d in enumerate(grid)]
            h = math.radians(self.heading)
            back = self.seen.offset_m
            camera = (self.x - back * math.cos(h), self.y - back * math.sin(h), self.heading)
            self.scene.integrate(grid, cols, depth["rows"], width, height, self.focal_px,
                                 floor.horizon, floor.height_focal, camera)

    async def view(self, out_width: int, extra_filters: list[str]) -> bytes:
        """The view now with its marks (kept for goto), as a JPEG."""
        jpeg, depth, width, height = await self.frame_depth()
        found, floor = marks(depth["depth"], depth["cols"], depth["rows"], width, height,
                             self.focal_px, self.been, self.seen)
        self.marks = [from_feet(m, self.seen.offset_m) for m in found]
        third = self.seen.me is not None
        # For goto by cell: a first person view only (a cell of a view from
        # behind is not the way from the avatar).
        self.last_view = None if third else (depth, width, height, self.seen)
        filters = ((extra_filters if third else cell_filters(width, height) + extra_filters)
                   + label_filters(found, floor, width, height, self.focal_px,
                                   offset_m=self.seen.offset_m))
        return await self.bridge.scaled_jpeg(jpeg, out_width, filters)

    def cell_point(self, cell: str) -> tuple[float, float]:
        """The world point (m) of what is in ``cell`` of the last view ahead."""
        if self.last_view is None:
            raise ValueError("cells are in the view ahead only (the one with the grid): turn to "
                             "face the thing with vrchat_step (turn), then use its cell there")
        depth, width, height, seen = self.last_view
        bearing, far = cell_target(cell, depth, width, height, self.focal_px, typical=True,
                                   seen=seen)
        h = math.radians(self.heading + bearing)
        return self.x + far * math.cos(h), self.y + far * math.sin(h)

    def cell_place(self, cell: str) -> Mark:
        """A place toward what is in ``cell`` of the last view ahead, as far
        as it is (goto walks up to it)."""
        if getattr(self, "last_view", None) is None:
            raise ValueError("cells are in the view ahead only (the one with the grid): turn to "
                             "face the thing with vrchat_step (turn), then use its cell there")
        depth, width, height, seen = self.last_view
        bearing, far = cell_target(cell, depth, width, height, self.focal_px, seen=seen)
        return Mark(98, bearing, round(max(0.3, min(MAX_WALK_M, far)), 1))

    async def ahead_m(self) -> float:
        """World metres of free floor straight ahead now (0: no floor seen)."""
        _, depth, width, height = await self.frame_depth()
        d, cols, rows = depth["depth"], depth["cols"], depth["rows"]
        floor = fit_floor(d, cols, rows, height, self.seen.me)
        if floor is None:
            return 0.0
        return min(free_ahead(d, cols, rows, height, floor,
                              column_of(b, width, cols, self.focal_px), self.focal_px, self.seen)
                   for b in (-CORRIDOR_DEG, 0, CORRIDOR_DEG))

    # -- walking -----------------------------------------------------------

    def toward(self, point: tuple[float, float]) -> tuple[float, float]:
        """The bearing (degrees, + right, -180..180) and distance (m) of a
        world point from the pose now."""
        dx, dy = point[0] - self.x, point[1] - self.y
        bearing = math.degrees(math.atan2(dy, dx)) - self.heading
        return (bearing + 180) % 360 - 180, math.hypot(dx, dy)

    async def _face(self, bearing: float, driver, step_type) -> None:
        if abs(bearing) >= 2:
            await driver.run([step_type(turn_deg=float(bearing))])
            self.turned(bearing)

    async def _walk_ahead(self, distance: float, deadline: float) -> tuple[float, bool]:
        """Walks straight on ``distance`` m, looking ahead by depth all the
        way; returns how far it went and whether something stopped it."""
        osc = self.bridge.osc
        speed = self.axis_speed(WALK_AXIS)
        loop = asyncio.get_running_loop()
        last = loop.time()
        walked, blocked_looks = 0.0, 0
        watch = (last, self.x, self.y)  # odometry: getting anywhere?
        start = (self.x, self.y)
        try:
            osc.send("/input/Vertical", WALK_AXIS)
            while walked < distance and loop.time() < deadline:
                free = await self.ahead_m()
                now = loop.time()
                if self.odometry and now - watch[0] >= STALL_S:
                    if math.hypot(self.x - watch[1], self.y - watch[2]) < STALL_M:
                        return walked, True  # walking on, going nowhere: in the way
                    watch = (now, self.x, self.y)
                step = speed * (now - last)
                if self.odometry:  # how far it really went, ahead
                    h = math.radians(self.heading)
                    walked = ((self.x - start[0]) * math.cos(h)
                              + (self.y - start[1]) * math.sin(h))
                else:
                    walked += step
                    self.walked(step)  # the pose as it goes: the map's frames need it
                last = now
                # One noisy depth frame does not stop a walk; two do, or one
                # with something right in front.
                blocked_looks = blocked_looks + 1 if free < STOP_M else 0
                if blocked_looks >= 2 or free < STOP_NOW_M:
                    return walked, True
                # Close to the end: walk out the rest without another look.
                rest = distance - walked
                if 0 < rest < speed * 0.3:
                    await asyncio.sleep(rest / speed)
                    self.walked(rest)
                    walked = distance  # (odometry: about; the avatar slides on a little)
        finally:
            osc.send("/input/Vertical", 0.0)
        return walked, False

    async def _detour_way(self, desired: float, side: str) -> Mark | None:
        """A way around what blocks: of the walkable places now, the one
        nearest the goal's bearing (``desired``), not straight on (that is
        what blocked), on ``side`` if one is asked for."""
        _, depth, width, height = await self.frame_depth()
        found, _ = marks(depth["depth"], depth["cols"], depth["rows"], width, height,
                         self.focal_px, seen=self.seen)
        ways = [m for m in found if abs(m.bearing) >= MARK_SPACING_DEG / 2
                and (side != "left" or m.bearing < 0) and (side != "right" or m.bearing > 0)]
        return min(ways, key=lambda m: abs(m.bearing - desired), default=None)

    async def follow_path(self, goal: tuple[float, float], driver, step_type) -> dict:
        """Walks the map's path to ``goal`` (world m), leg by leg, braking
        before obstacles; replans when a leg is blocked; ends facing it."""
        if self.scene is None:
            raise ValueError("no map yet: look around first")
        deadline = asyncio.get_running_loop().time() + PATH_LIMIT_S
        walked, replans, stopped = 0.0, 0, "arrived"
        path = self.scene.path((self.x, self.y), goal)
        while path:
            if asyncio.get_running_loop().time() > deadline:
                stopped = "too long"
                break
            point = path.pop(0)
            bearing, left = self.toward(point)
            if left < ARRIVE_M:
                continue
            await self._face(bearing, driver, step_type)
            went, blocked = await self._walk_ahead(left, deadline)
            walked += went
            if blocked:
                if self.toward(goal)[1] < 1.0:
                    break  # up to it: that is there
                if replans >= 2:
                    stopped = "blocked"
                    break
                replans += 1
                path = self.scene.path((self.x, self.y), goal) or []
        else:
            if path is None:
                stopped = "no way"
        bearing, left = self.toward(goal)
        if stopped == "arrived" and left > 1.5:
            stopped = "no way"
        if left > ARRIVE_M:
            await self._face(bearing, driver, step_type)  # the view ahead shows it
        return {"ok": True, "walked_m": round(walked, 1), "left_m": round(left, 1),
                "replans": replans, "stopped": stopped}

    async def goto(self, number: int, driver, step_type, detour: str = "auto",
                   up_to: bool = False) -> dict:
        """Walks to mark ``number`` of the last view (0: turns around). The
        place is kept as a point of the pose: when something blocks the way,
        the walk goes around it (``detour``: auto, left, right, or none) by
        a walkable place to the side, then heads for the point again."""
        if number == 0:
            await driver.run([step_type(turn_deg=180.0)])
            self.turned(180)
            return {"ok": True, "turned": 180, "walked_m": 0.0, "stopped": "turned"}
        mark = next((m for m in self.marks if m.number == number), None)
        if mark is None:
            raise ValueError(f"no mark {number} in your last view: look again")
        self.marks = []  # the old view's marks are gone with the turn
        if up_to:
            # Right up to the thing (to climb onto it): on until it blocks,
            # never around it, and never much past where it is (a ledge
            # beyond it is a fall).
            mark, detour = Mark(mark.number, mark.bearing,
                                min(MAX_WALK_M, mark.distance + UP_TO_PAST_M)), "none"
        h = math.radians(self.heading + mark.bearing)
        goal = (self.x + mark.distance * math.cos(h), self.y + mark.distance * math.sin(h))
        deadline = asyncio.get_running_loop().time() + GOTO_LIMIT_S
        walked, detours, stopped = 0.0, 0, "blocked"
        while asyncio.get_running_loop().time() < deadline:
            bearing, left = self.toward(goal)
            if left < ARRIVE_M:
                stopped = "arrived"
                break
            await self._face(bearing, driver, step_type)
            went, blocked = await self._walk_ahead(left, deadline)
            walked += went
            if up_to:
                stopped = "reached it" if blocked else "nothing there"
                break
            if not blocked:
                stopped = "arrived" if self.toward(goal)[1] < ARRIVE_M * 2 else "too long"
                break
            if self.toward(goal)[1] < 1.0:
                stopped = "arrived"  # stopped just short of it: that is there
                break
            if detour == "none" or detours >= MAX_DETOURS:
                break
            way = await self._detour_way(self.toward(goal)[0], detour)
            if way is None:
                break
            detours += 1
            await self._face(way.bearing, driver, step_type)
            went, _ = await self._walk_ahead(min(way.distance, DETOUR_M), deadline)
            walked += went
        return {"ok": True, "turned": mark.bearing, "walked_m": round(walked, 1),
                "planned_m": mark.distance, "left_m": round(self.toward(goal)[1], 1),
                "detours": detours, "stopped": stopped}


# -- looking around ----------------------------------------------------------------

AROUND = ((0, "ahead"), (90, "right"), (180, "behind"), (-90, "left"))
TILE_WIDTH = 480


def renumbered(found: list[Mark], first: int) -> list[Mark]:
    return [Mark(first + i, m.bearing, m.distance, m.been) for i, m in enumerate(found)]


async def look_around(walker: Navigator, driver, step_type, ruler: list[str]) -> bytes:
    """Turns a full circle in four quarter turns, marking the walkable places
    of each view; returns them as one 2x2 picture (ahead, right / behind,
    left), the places numbered across all four, their bearings from the
    heading it started at (kept for goto)."""
    import os
    import tempfile

    tiles, everything = [], []
    for offset, name in AROUND:
        if offset:
            await driver.run([step_type(turn_deg=90.0)])
            walker.turned(90)
        jpeg, depth, width, height = await walker.frame_depth()
        found, floor = marks(depth["depth"], depth["cols"], depth["rows"], width, height,
                             walker.focal_px, walker.been, walker.seen)
        found = renumbered(found, len(everything) + 1)
        everything += [Mark(m.number, (f.bearing + offset + 180) % 360 - 180, f.distance,
                            m.been)
                       for m, f in ((m, from_feet(m, walker.seen.offset_m)) for m in found)]
        title = (f"drawtext=font=monospace:text='{name}':x=12:y=h-44:fontsize=40:"
                 "fontcolor=white:borderw=4:bordercolor=black")
        # Large enough to read once shrunk to a tile; no ruler (each tile's
        # bearings are its own).
        filters = label_filters(found, floor, width, height, walker.focal_px, size=60,
                                offset_m=walker.seen.offset_m)[:-1]
        tiles.append(await walker.bridge.scaled_jpeg(jpeg, TILE_WIDTH, [*filters, title]))
    await driver.run([step_type(turn_deg=90.0)])  # facing as it started
    walker.turned(90)
    walker.marks = everything
    walker.last_view = None  # its tiles have no grid: cells need a view ahead again
    with tempfile.TemporaryDirectory() as tmp:
        paths = []
        for i, tile in enumerate(tiles):
            paths.append(os.path.join(tmp, f"{i}.jpg"))
            with open(paths[-1], "wb") as f:
                f.write(tile)
        inputs = [arg for path in paths for arg in ("-i", path)]
        proc = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", *inputs, "-filter_complex",
            "[0][1][2][3]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0",
            "-frames:v", "1", "-c:v", "mjpeg", "-q:v", "4", "-f", "mjpeg", "-",
            stdout=asyncio.subprocess.PIPE)
        out, _ = await proc.communicate()
    if proc.returncode != 0 or not out:
        raise RuntimeError("composing the look around failed")
    return out
