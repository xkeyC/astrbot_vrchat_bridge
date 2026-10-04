"""The avatar seen from behind (third person) in its own view (python -m pytest bridge)."""
import json
from pathlib import Path

import nav
import selfview

HERE = Path(__file__).parent
BEHIND = json.loads((HERE / "testdata_depth_third_person.json").read_text())
HALL = json.loads((HERE / "testdata_depth_hall.json").read_text())


def test_the_avatar_is_found_from_behind_and_not_in_first_person():
    me = selfview.find_self(BEHIND["depth"], 64, 36)
    assert me is not None and me.c0 <= 31 <= me.c1 and me.c1 - me.c0 < 14
    assert 29 <= me.r1 <= 33  # its feet, the floor seen below them
    assert selfview.find_self(HALL["depth"], 64, 36) is None


def test_the_camera_is_behind_the_feet():
    seen = nav.seen_in(BEHIND["depth"], 64, 36, 720)
    assert 1.2 < seen.offset_m < 2.5
    assert nav.seen_in(HALL["depth"], 64, 36, 720) == nav.Seen()


def test_straight_ahead_is_not_the_avatars_own_back():
    seen = nav.seen_in(BEHIND["depth"], 64, 36, 720)
    found, _ = nav.marks(BEHIND["depth"], 64, 36, 1280, 720, 566, seen=seen)
    ahead = [m for m in found if abs(m.bearing) <= 10]
    # Without it, straight ahead was 1.4 m: the distance to its own back.
    assert all(m.distance > 1.6 for m in ahead)


def test_a_cell_on_the_avatar_is_refused():
    seen = nav.seen_in(BEHIND["depth"], 64, 36, 720)
    depth = {"depth": BEHIND["depth"], "cols": 64, "rows": 36}
    try:
        nav.cell_target("E4", depth, 1280, 720, 566, seen=seen)
    except ValueError as e:
        assert "is you" in str(e)
    else:
        raise AssertionError("E4 is the avatar's back")


def test_a_mark_seen_from_behind_is_walked_from_the_feet():
    # 10 degrees off, 2 m on from the feet, the camera 2 m behind them: from
    # the feet it is twice as far off to the side.
    mark = nav.from_feet(nav.Mark(1, 10, 2.0), 2.0)
    assert 18 <= mark.bearing <= 21 and 1.9 <= mark.distance <= 2.2
    assert nav.from_feet(nav.Mark(2, 0, 3.0), 2.0) == nav.Mark(2, 0, 3.0)
    assert nav.from_feet(nav.Mark(3, 25, 3.0), 0.0) == nav.Mark(3, 25, 3.0)
