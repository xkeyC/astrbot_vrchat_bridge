"""Walkable places from depth, and walking to them (python -m pytest bridge)."""
import asyncio
import json
from pathlib import Path

import pytest

import drive
import nav

# A real 64x36 depth grid of a 1280x720 frame: a hall with a couch on the
# left, a pool ahead and a wall far off.
HALL = json.loads((Path(__file__).parent / "testdata_depth_hall.json").read_text())


def test_the_floor_is_fitted_from_the_lower_middle():
    floor = nav.fit_floor(HALL["depth"], 64, 36, 720)
    assert floor is not None
    assert 300 < floor.horizon < 380  # about the middle: looking level
    # Near the bottom the floor is a few model metres away.
    assert 2.5 < floor.depth_at(700) < 4.5


def test_a_wall_in_front_is_no_floor():
    wall = [0.5] * (64 * 36)
    assert nav.fit_floor(wall, 64, 36, 720) is None
    found, floor = nav.marks(wall, 64, 36, 1280, 720, 566)
    assert found == [] and floor is None


def test_marks_spread_left_to_right_and_the_couch_is_near():
    found, _ = nav.marks(HALL["depth"], 64, 36, 1280, 720, 566)
    assert 3 <= len(found) <= nav.MAX_MARKS
    assert [m.number for m in found] == list(range(1, len(found) + 1))
    bearings = [m.bearing for m in found]
    assert bearings == sorted(bearings)
    assert all(b - a >= nav.MARK_SPACING_DEG for a, b in zip(bearings, bearings[1:], strict=False))
    assert all(nav.MIN_WALK_M <= m.distance <= nav.MAX_WALK_M for m in found)
    # The couch on the left leaves less room than the open hall ahead.
    left = min(found, key=lambda m: m.bearing)
    ahead = min(found, key=lambda m: abs(m.bearing))
    assert left.distance < ahead.distance


def test_places_already_walked_are_marked():
    walker = nav.Navigator(None, "", "", 566, lambda axis: 1.8)
    walker.walked(3.0)  # 3 m ahead, then back to the start
    walker.turned(180)
    walker.walked(3.0)
    # Facing back the way it came: 3 m behind it now is where it was.
    assert walker.been(180, 3.0)
    assert not walker.been(90, 3.0)


def test_labels_draw_every_mark_and_turn_around():
    found, floor = nav.marks(HALL["depth"], 64, 36, 1280, 720, 566)
    filters = nav.label_filters(found, floor, 1280, 720, 566)
    texts = [f for f in filters if f.startswith("drawtext")]
    assert len(texts) == len(found) + 1
    assert "0 turn around" in texts[-1]


class FakeOsc:
    def __init__(self):
        self.sent = []

    def send(self, address, value):
        self.sent.append((address, value))


class FakeBridge:
    def __init__(self):
        self.osc = FakeOsc()


def test_a_walk_stops_before_an_obstacle(monkeypatch):
    bridge = FakeBridge()
    walker = nav.Navigator(bridge, "", "", 566, lambda axis: 2.0)
    walker.marks = [nav.Mark(1, 20, 5.0)]
    seen = iter([3.0, 0.5, 2.0, 0.6, 0.5])  # a lone 0.5 (noise), then blocked twice

    async def ahead_m():
        await asyncio.sleep(0.05)
        return next(seen)

    monkeypatch.setattr(walker, "ahead_m", ahead_m)
    turns = []

    class Driver:
        async def run(self, steps):
            turns.extend(step.turn_deg for step in steps)

    result = asyncio.run(walker.goto(1, Driver(), drive.Step, detour="none"))
    assert turns == [20.0]
    assert result["stopped"] == "blocked"
    assert 0 < result["walked_m"] < 5.0
    assert bridge.osc.sent[-1] == ("/input/Vertical", 0.0)  # let go
    assert walker.heading == 20 and walker.x > 0
    with pytest.raises(ValueError):
        asyncio.run(walker.goto(1, Driver(), drive.Step))  # the old view's marks are gone


def test_zero_turns_around():
    walker = nav.Navigator(FakeBridge(), "", "", 566, lambda axis: 1.8)
    turns = []

    class Driver:
        async def run(self, steps):
            turns.extend(step.turn_deg for step in steps)

    assert asyncio.run(walker.goto(0, Driver(), drive.Step))["turned"] == 180
    assert turns == [180.0] and walker.heading == 180


def test_marks_of_the_four_views_are_numbered_across_all():
    first = [nav.Mark(1, -20, 2.0), nav.Mark(2, 10, 3.0)]
    assert [m.number for m in nav.renumbered(first, 3)] == [3, 4]
    assert [m.bearing for m in nav.renumbered(first, 3)] == [-20, 10]


def test_a_blocked_walk_goes_around_and_on_to_the_place(monkeypatch):
    bridge = FakeBridge()
    walker = nav.Navigator(bridge, "", "", 566, lambda axis: 2.0)
    walker.marks = [nav.Mark(1, 0, 3.0)]
    # Blocked at once twice; then free all the way.
    looks = iter([0.5, 0.5] + [9.0] * 400)

    async def ahead_m():
        await asyncio.sleep(0.02)
        return next(looks)

    asked = []

    async def detour_way(desired, side):
        asked.append(side)
        return nav.Mark(9, -40, 1.0)

    monkeypatch.setattr(walker, "ahead_m", ahead_m)
    monkeypatch.setattr(walker, "_detour_way", detour_way)
    monkeypatch.setattr(nav, "STOP_NOW_M", 0.6)
    turns = []

    class Driver:
        async def run(self, steps):
            turns.extend(step.turn_deg for step in steps)

    result = asyncio.run(walker.goto(1, Driver(), drive.Step, detour="left"))
    assert asked == ["left"] and result["detours"] == 1
    assert turns[0] == -40.0  # aside first, then back toward the place
    assert turns[1] > 0
    assert result["stopped"] == "arrived" and result["left_m"] < 2 * nav.ARRIVE_M


def test_up_to_walks_on_until_blocked_and_never_around(monkeypatch):
    walker = nav.Navigator(FakeBridge(), "", "", 566, lambda axis: 2.0)
    walker.marks = [nav.Mark(1, 0, 1.0)]  # the thing about 1 m off
    looks = iter([5.0, 5.0, 5.0, 0.5, 0.5])

    async def ahead_m():
        await asyncio.sleep(0.15)
        return next(looks)

    async def detour_way(desired, side):
        raise AssertionError("no detour when climbing")

    monkeypatch.setattr(walker, "ahead_m", ahead_m)
    monkeypatch.setattr(walker, "_detour_way", detour_way)

    class Driver:
        async def run(self, steps):
            pass

    result = asyncio.run(walker.goto(1, Driver(), drive.Step, up_to=True))
    assert result["stopped"] == "reached it"
    assert result["walked_m"] > 1.0  # past the place, up to the thing


def test_a_cell_points_at_a_bearing_and_its_nearest_depth():
    assert nav.parse_cell("d4") == (3, 3)
    for bad in ("Z1", "A9", "4D", ""):
        with pytest.raises(ValueError):
            nav.parse_cell(bad)
    # The couch on the left of the hall: cell A5 is near, D3 (ahead) far.
    left, near = nav.cell_target("A5", HALL, 1280, 720, 566)
    ahead, far = nav.cell_target("D3", HALL, 1280, 720, 566)
    assert left < -30 and -10 < ahead < 0
    assert near < far


def _column(ground):
    """A 1-column depth grid (36 rows of a 720 px frame, horizon 300 px,
    eye 3 model m up, focal 566 px) looking at ``ground(d) -> height``,
    solved by stepping along each ray."""
    depth = []
    for r in range(36):
        k = ((r + 0.5) * 20 - 300) / 566  # the ray's drop per metre
        d = 0.05
        while d < 60 and 3.0 - k * d > ground(d):
            d += 0.05
        depth.append(d)
    return depth, nav.Floor(horizon=300, height_focal=3.0 * 566)


def test_a_gentle_rise_is_walked_on_but_a_face_is_not():
    rise, floor = _column(lambda d: 0.0 if d < 5 else 0.3 * (d - 5))
    wall, _ = _column(lambda d: 0.0 if d < 5 else 99.0)
    flat_only = nav.free_distance(rise, 1, 36, 720, floor, 0)
    sloped = nav.free_distance(rise, 1, 36, 720, floor, 0, 566)
    walled = nav.free_distance(wall, 1, 36, 720, floor, 0, 566)
    # The wall stops it where the floor meets it (within the floor's
    # tolerance), as the flat floor alone stops at the foot of the rise.
    assert walled < 6.5 and flat_only < 9
    assert sloped > 12
