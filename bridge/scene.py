"""A top-down map of the world the avatar walks, built from depth as it goes.

Each depth frame becomes points (the floor fit gives the camera's height and
the horizon; the dead-reckoned pose places them): points on the floor clear
their cells, points above it fill them with an obstacle and its height, and
the rays from the eye to them clear what they cross. On this map:

- platforms: patches of obstacle cells with one height between knee and
  chest (a seat, a table, a step) whose top was seen (the view went on
  over it: a wall seen from close by only shows its foot) are things to
  stand on;
- landmarks: things the model named, kept where they are (seen in a cell of
  a view ahead), so they can be found again out of sight;
- the pose: the avatar's own velocity for walks (odometry.py), the mouse
  motion sent for turns (calibrated); matching frames against the map was
  tried and only fitted the depth's noise;
- ground: cells filled only a little above their neighbours (a grassy
  mound, a ramp: no step higher than STEP_M) are walked over, not around;
- paths: A* over the cells, around obstacles, to a landmark or a platform;
- a picture: the map around the avatar, it facing up, landmarks and
  platforms labelled, for the model to read (as MapNav's annotated maps).

Cheap on purpose: numpy only, one 64x36 depth grid per frame.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass

import numpy as np

CELL_M = 0.25
SIZE = 400  # cells a side: 100 m, the avatar starting in the middle
DEPTH_SCALE = 2.0  # the depth model's metres per world metre
MAX_RANGE_M = 8.0  # world metres of depth trusted
FLOOR_M = 0.12  # points this high or lower are floor
CEILING_M = 2.2  # higher points are no obstacle to walking
STEP_M = 0.25  # cells this much above or below their neighbours are walked on
PLATFORM_LOW_M, PLATFORM_HIGH_M = 0.25, 1.2
PLATFORM_STEP_M = 0.12  # neighbour cells of one surface differ by at most this
PLATFORM_CELLS = (3, 80)
PLATFORM_SPREAD_M = 0.3  # a platform's top spans at most this (depth is noisy); a slope more
PLATFORM_DROP_M = 0.2  # a platform drops this much at an edge (a slope does not)
OVER = 1.08  # the depth just above a point this much farther: the view went over it


@dataclass(frozen=True)
class Platform:
    number: int
    x: float
    y: float
    height: float
    cells: tuple[tuple[int, int], ...] = ()


class SceneMap:
    def __init__(self) -> None:
        self.free = np.zeros((SIZE, SIZE), np.uint16)  # seen through or on the floor
        self.floor = np.zeros((SIZE, SIZE), np.uint16)  # floor points (height 0)
        self.occ = np.zeros((SIZE, SIZE), np.uint16)
        self.top = np.full((SIZE, SIZE), np.nan, np.float32)  # obstacle heights, m
        self.over = np.zeros((SIZE, SIZE), np.uint16)  # points seen with the view going over
        self.landmarks: dict[str, tuple[float, float, float]] = {}  # name -> x, y, when
        self.frames = 0

    # -- cells -----------------------------------------------------------------

    @staticmethod
    def cell(x: float, y: float) -> tuple[int, int]:
        return math.floor(x / CELL_M) + SIZE // 2, math.floor(y / CELL_M) + SIZE // 2

    @staticmethod
    def centre(i: int, j: int) -> tuple[float, float]:
        return (i - SIZE // 2 + 0.5) * CELL_M, (j - SIZE // 2 + 0.5) * CELL_M

    def blocked(self) -> np.ndarray:
        """Cells an obstacle fills (seen as such more than as free)."""
        return (self.occ >= 2) & (self.occ * 2 > self.free)

    def obstacles(self) -> np.ndarray:
        """Blocked cells that are no ground to walk on: higher or lower than
        a known neighbour by more than STEP_M (or of no height known)."""
        blocked = self.blocked()
        level = np.where(blocked, self.top, np.where(self.floor > 0, 0.0, np.nan))
        step = np.zeros((SIZE, SIZE), np.float32)
        known = np.zeros((SIZE, SIZE), bool)
        for axis in (0, 1):
            for shift in (1, -1):
                other = np.roll(level, shift, axis)
                has = ~np.isnan(other)
                step = np.where(has, np.fmax(step, np.abs(np.nan_to_num(level) - other)), step)
                known |= has
        ground = known & ~np.isnan(level) & (step <= STEP_M)
        return blocked & ~ground

    # -- building --------------------------------------------------------------

    def integrate(self, depth: list[float], cols: int, rows: int, width: int, height: int,
                  focal_px: float, horizon: float, height_focal: float,
                  pose: tuple[float, float, float]) -> None:
        """Adds one depth frame seen from ``pose`` (world x, y in m, heading
        in degrees), the floor fit giving the horizon (px) and the camera's
        height times the focal length (model m x px)."""
        d = np.asarray(depth, np.float32).reshape(rows, cols)
        u = (np.arange(cols, dtype=np.float32) + 0.5) * width / cols
        v = (np.arange(rows, dtype=np.float32) + 0.5) * height / rows
        uu, vv = np.meshgrid(u, v)
        eye = height_focal / focal_px  # camera height, model m
        forward = d / DEPTH_SCALE
        right = (uu - width / 2) / focal_px * d / DEPTH_SCALE
        above = (eye - (vv - horizon) / focal_px * d) / DEPTH_SCALE
        keep = (forward > 0.2) & (forward < MAX_RANGE_M) & (above < CEILING_M + 1)
        floor = keep & (above <= FLOOR_M)
        solid = keep & (above > FLOOR_M) & (above <= CEILING_M)
        x0, y0, heading = pose
        h = math.radians(heading)
        wx = x0 + forward * math.cos(h) - right * math.sin(h)
        wy = y0 + forward * math.sin(h) + right * math.cos(h)
        over = np.zeros_like(solid)
        over[1:] = d[:-1] > d[1:] * OVER
        # Rays clear what they cross (short of their end).
        self._clear_rays(x0, y0, wx[keep], wy[keep])
        for mask, grid in ((floor, self.free), (floor, self.floor), (solid, self.occ)):
            ii, jj = self._cells(wx[mask], wy[mask])
            np.add.at(grid, (ii, jj), 1)
        ii, jj = self._cells(wx[solid], wy[solid])
        tops = above[solid]
        current = self.top[ii, jj]
        self.top[ii, jj] = np.where(np.isnan(current), tops, np.fmax(current, tops))
        ii, jj = self._cells(wx[solid & over], wy[solid & over])
        np.add.at(self.over, (ii, jj), 1)
        self.frames += 1

    def _cells(self, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ii = np.clip(np.floor(xs / CELL_M).astype(int) + SIZE // 2, 0, SIZE - 1)
        jj = np.clip(np.floor(ys / CELL_M).astype(int) + SIZE // 2, 0, SIZE - 1)
        return ii, jj

    def _clear_rays(self, x0: float, y0: float, xs: np.ndarray, ys: np.ndarray) -> None:
        if xs.size == 0:
            return
        length = np.hypot(xs - x0, ys - y0)
        steps = np.arange(0.0, MAX_RANGE_M, CELL_M / 2, dtype=np.float32)
        frac = steps[None, :] / np.maximum(length[:, None], 1e-6)
        inside = frac < (1 - CELL_M / np.maximum(length[:, None], CELL_M))
        px = x0 + (xs[:, None] - x0) * frac
        py = y0 + (ys[:, None] - y0) * frac
        ii, jj = self._cells(px[inside], py[inside])
        cells = np.unique(ii * SIZE + jj)
        np.add.at(self.free, (cells // SIZE, cells % SIZE), 1)

    # -- what is on it -----------------------------------------------------------

    def note(self, name: str, x: float, y: float) -> None:
        self.landmarks[name.strip()] = (x, y, time.time())

    def find(self, name: str) -> tuple[str, float, float] | None:
        """The landmark best matching ``name`` (its words in the name, or
        the name in it)."""
        wanted = name.strip().lower()
        best = None
        for known, (x, y, _) in self.landmarks.items():
            k = known.lower()
            if wanted == k:
                return known, x, y
            if (wanted in k or k in wanted) and (best is None or len(k) < len(best[0])):
                best = (known, x, y)
        return best

    def standable(self) -> np.ndarray:
        """Obstacle cells in the height band with their top seen."""
        return (self.blocked() & (self.over > 0)
                & (self.top >= PLATFORM_LOW_M) & (self.top <= PLATFORM_HIGH_M))

    def platforms(self) -> list[Platform]:
        """Patches of one height to stand on, numbered nearest first later
        by the caller; flood fill over standable cells."""
        band = self.standable()
        blocked = self.blocked()
        seen = np.zeros_like(band)
        found = []
        for i, j in zip(*np.nonzero(band), strict=False):
            if seen[i, j]:
                continue
            stack, patch = [(i, j)], []
            seen[i, j] = True
            while stack:
                a, b = stack.pop()
                patch.append((a, b))
                for na, nb in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                    if (0 <= na < SIZE and 0 <= nb < SIZE and band[na, nb] and not seen[na, nb]
                            and abs(float(self.top[na, nb]) - float(self.top[a, b]))
                            <= PLATFORM_STEP_M):
                        seen[na, nb] = True
                        stack.append((na, nb))
            if PLATFORM_CELLS[0] <= len(patch) <= PLATFORM_CELLS[1] and self._drops(patch, blocked):
                xs, ys = zip(*(self.centre(a, b) for a, b in patch), strict=False)
                heights = [float(self.top[a, b]) for a, b in patch]
                found.append(Platform(0, sum(xs) / len(xs), sum(ys) / len(ys),
                                      round(float(np.median(heights)), 2), tuple(patch)))
        return found

    def _drops(self, patch: list[tuple[int, int]], blocked: np.ndarray) -> bool:
        """Whether the patch is a flat top with an edge: floor next to it, or
        something clearly lower (a slope, one surface, rises too much)."""
        cells = set(patch)
        heights = np.array([self.top[c] for c in patch])
        low, level, high = np.percentile(heights, (10, 50, 90))
        if high - low > PLATFORM_SPREAD_M:
            return False  # rising across it: a slope
        for a, b in patch:
            for n in ((a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)):
                if n in cells or not (0 <= n[0] < SIZE and 0 <= n[1] < SIZE):
                    continue
                if blocked[n]:
                    if float(self.top[n]) < level - PLATFORM_DROP_M:
                        return True
                elif self.floor[n] > 0:
                    return True
        return False

    # -- paths -----------------------------------------------------------------

    def path(self, start: tuple[float, float], goal: tuple[float, float],
             goal_blocked_ok: bool = True) -> list[tuple[float, float]] | None:
        """Waypoints (world m) from ``start`` to ``goal`` around obstacles
        (inflated by a cell), unknown cells allowed at a cost; the goal cell
        itself may be an obstacle (a thing to walk up to). With no way all
        the way, the way to the nearest place to it reached (a landmark
        behind a planter); None: not even nearer."""
        blocked = self.obstacles()
        inflated = blocked.copy()
        inflated[1:, :] |= blocked[:-1, :]
        inflated[:-1, :] |= blocked[1:, :]
        inflated[:, 1:] |= blocked[:, :-1]
        inflated[:, :-1] |= blocked[:, 1:]
        s, g = self.cell(*start), self.cell(*goal)
        inflated[s] = False
        near_goal = set()
        if goal_blocked_ok:
            near_goal = {(g[0] + a, g[1] + b) for a in (-2, -1, 0, 1, 2) for b in (-2, -1, 0, 1, 2)}
        known = (self.free > 0) | blocked
        frontier = [(0.0, s)]
        cost = {s: 0.0}
        came: dict[tuple[int, int], tuple[int, int]] = {}
        end, nearest = None, (math.dist(s, g), s)
        limit = 3 * math.dist(s, g) + 100  # cells: a long way round, not the whole map
        while frontier:
            _, cur = heapq.heappop(frontier)
            if cur == g or (cur in near_goal and inflated[g]):
                end = cur
                break
            nearest = min(nearest, (math.dist(cur, g), cur))
            if cost[cur] > limit:
                break  # far longer than the way straight: not a way worth it
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (cur[0] + da, cur[1] + db)
                if not (0 <= nxt[0] < SIZE and 0 <= nxt[1] < SIZE):
                    continue
                if inflated[nxt] and nxt != g:
                    continue
                step = math.hypot(da, db) * (1.0 if known[nxt] else 2.5)
                new = cost[cur] + step
                if new < cost.get(nxt, math.inf):
                    cost[nxt] = new
                    came[nxt] = cur
                    heapq.heappush(frontier, (new + math.dist(nxt, g), nxt))
        if end is None:
            if nearest[0] > math.dist(s, g) - 1 / CELL_M:
                return None  # not even a metre nearer
            end = nearest[1]
        cells = [end]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        cells.reverse()
        return self._smooth([self.centre(*c) for c in cells], inflated)

    def _smooth(self, points: list[tuple[float, float]], inflated: np.ndarray) -> list[tuple[float, float]]:
        """Fewer waypoints: skip those a straight free line passes by."""
        if len(points) <= 2:
            return points[1:]
        out, anchor = [], points[0]
        for k in range(1, len(points)):
            if not self._clear_line(anchor, points[k], inflated):
                out.append(points[k - 1])
                anchor = points[k - 1]
        out.append(points[-1])
        return out

    def _clear_line(self, a, b, inflated) -> bool:
        n = max(1, int(math.dist(a, b) / (CELL_M / 2)))
        for t in range(1, n):
            c = self.cell(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
            if inflated[c]:
                return False
        return True

    # -- a picture ---------------------------------------------------------------

    def render(self, pose: tuple[float, float, float], visited: list[tuple[float, float]],
               span_m: float = 16.0, px: int = 480) -> np.ndarray:
        """An RGB picture of the map around ``pose``, the avatar in the
        middle facing up: unknown dark, floor light, rising ground light
        green, obstacles black, platforms amber (brighter: higher), the way
        walked blue."""
        x0, y0, heading = pose
        h = math.radians(heading)
        half = span_m / 2
        grid = np.linspace(-half, half, px, dtype=np.float32)
        right, up = np.meshgrid(grid, -grid)  # image x to the right, y up = ahead
        wx = x0 + up * math.cos(h) - right * math.sin(h)
        wy = y0 + up * math.sin(h) + right * math.cos(h)
        ii, jj = self._cells(wx, wy)
        img = np.full((px, px, 3), 40, np.uint8)
        free = self.free[ii, jj] > 0
        blocked = self.blocked()[ii, jj]
        obstacle = self.obstacles()[ii, jj]
        img[free] = (200, 200, 200)
        top = self.top[ii, jj]
        img[blocked & ~obstacle] = (170, 215, 160)
        img[obstacle] = (20, 20, 20)
        platform = np.zeros((SIZE, SIZE), bool)
        for p in self.platforms():
            for c in p.cells:
                platform[c] = True
        stand = platform[ii, jj]
        shade = np.clip((np.nan_to_num(top) - PLATFORM_LOW_M) / (PLATFORM_HIGH_M - PLATFORM_LOW_M),
                        0, 1)
        img[stand] = np.stack([200 + 55 * shade[stand], 140 + 60 * shade[stand],
                               np.zeros_like(shade[stand])], axis=-1).astype(np.uint8)
        scale = px / span_m
        for (vx, vy) in visited[-400:]:
            col, row = self.to_pixel(vx, vy, pose, span_m, px)
            if 0 <= col < px and 0 <= row < px:
                img[max(0, row - 1):row + 2, max(0, col - 1):col + 2] = (60, 120, 255)
        c = px // 2
        for k in range(int(0.6 * scale)):  # the avatar: a red arrow up
            w = max(1, int((0.6 * scale - k) / 2))
            img[c - k, max(0, c - w):c + w + 1] = (230, 40, 40)
        return img

    @staticmethod
    def to_pixel(x: float, y: float, pose, span_m: float, px: int) -> tuple[int, int]:
        """The picture pixel (column, row) of a world point (render's frame)."""
        x0, y0, heading = pose
        h = math.radians(heading)
        dx, dy = x - x0, y - y0
        ahead = dx * math.cos(h) + dy * math.sin(h)
        right = -dx * math.sin(h) + dy * math.cos(h)
        scale = px / span_m
        return int(px / 2 + right * scale), int(px / 2 - ahead * scale)
