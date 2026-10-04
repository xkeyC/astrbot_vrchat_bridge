"""The scene map built from depth (python -m pytest bridge)."""
import json
from pathlib import Path

import nav
import scene

HALL = json.loads((Path(__file__).parent / "testdata_depth_hall.json").read_text())


def hall_map() -> scene.SceneMap:
    floor = nav.fit_floor(HALL["depth"], 64, 36, 720)
    m = scene.SceneMap()
    m.integrate(HALL["depth"], 64, 36, 1280, 720, 566, floor.horizon, floor.height_focal,
                (0.0, 0.0, 0.0))
    return m


def test_a_frame_clears_the_floor_ahead_and_fills_the_couch():
    m = hall_map()
    blocked = m.blocked()
    ahead = m.cell(3.0, 0.0)
    assert m.free[ahead] > 0 and not blocked[ahead]
    # The couch, front left (negative y is left).
    couch = [m.cell(x, y) for x in (2.0, 2.5, 3.0, 3.5) for y in (-2.0, -2.5, -3.0)]
    assert any(blocked[c] for c in couch)


def test_the_couch_is_a_place_to_stand_on():
    platforms = hall_map().platforms()
    assert platforms
    assert any(p.y < -1 and 1 < p.x < 5 and 0.25 <= p.height <= 0.8 for p in platforms)


def test_landmarks_are_found_by_part_of_their_name():
    m = scene.SceneMap()
    m.note("yellow seat", 2.0, 1.0)
    m.note("mirror", -3.0, 0.0)
    assert m.find("seat")[0] == "yellow seat"
    assert m.find("the big mirror")[0] == "mirror"
    assert m.find("door") is None


def test_a_path_goes_around_a_wall():
    m = scene.SceneMap()
    # Floor known all around, a long wall across the way 2 m ahead, one gap.
    m.free[:, :] = 5
    for y in range(-60, 61):
        if y not in (9, 10, 11, 12):  # a 1 m gap (0.5 m is too narrow once inflated)
            m.occ[m.cell(2.0, y * scene.CELL_M)] = 9
    path = m.path((0.0, 0.0), (4.0, 0.0), goal_blocked_ok=False)
    assert path and path[-1] == m.centre(*m.cell(4.0, 0.0))
    # It goes through the gap (y about 2.5-3 m), not through the wall.
    assert any(y > 2.0 for _, y in path)


def test_render_puts_the_avatar_in_the_middle_facing_up():
    m = hall_map()
    image = m.render((0.0, 0.0, 0.0), [(0.0, 0.0)], span_m=16, px=320)
    assert image.shape == (320, 320, 3)
    # A point 4 m ahead is above the middle; 4 m to the right is right of it.
    assert m.to_pixel(4.0, 0.0, (0.0, 0.0, 0.0), 16, 320) == (160, 80)
    assert m.to_pixel(0.0, 4.0, (0.0, 0.0, 0.0), 16, 320) == (240, 160)


def test_a_gentle_slope_is_no_platform_but_a_box_is():
    m = scene.SceneMap()
    m.free[:, :] = 5
    m.floor[:, :] = 5
    for x in range(8, 30):  # a slope rising 5 cm a cell from 2 m on
        for y in range(-4, 5):
            c = m.cell(x * scene.CELL_M, y * scene.CELL_M)
            m.occ[c], m.over[c], m.top[c] = 9, 3, 0.05 * (x - 7)
    for x in range(-12, -9):  # a seat 0.45 m high, 3 m behind
        for y in range(-1, 1):
            c = m.cell(x * scene.CELL_M, y * scene.CELL_M)
            m.occ[c], m.over[c], m.top[c] = 9, 3, 0.45
    platforms = m.platforms()
    assert len(platforms) == 1 and platforms[0].x < -2 and platforms[0].height == 0.45


def test_a_mound_is_walked_over_and_a_wall_around():
    m = scene.SceneMap()
    m.free[:, :] = 5
    m.floor[:, :] = 5
    for x in range(4, 13):  # a mound across the way, 1 to 3 m ahead, 0.5 m high
        for y in range(-40, 41):
            c = m.cell(x * scene.CELL_M, y * scene.CELL_M)
            m.occ[c], m.top[c] = 9, 0.5 - abs(x - 8) * 0.1
    assert not m.obstacles()[m.cell(2.0, 0.0)]
    path = m.path((0.0, 0.0), (4.0, 0.0), goal_blocked_ok=False)
    assert path and all(abs(y) < 1 for _, y in path)  # straight over it
    for y in range(-40, 41):  # a wall on top of it
        m.top[m.cell(2.0, y * scene.CELL_M)] = 2.0
    path = m.path((0.0, 0.0), (4.0, 0.0), goal_blocked_ok=False)
    assert path and any(abs(y) > 9 for _, y in path)  # around its end


def test_a_walled_in_landmark_is_walked_up_to():
    m = scene.SceneMap()
    m.free[:, :] = 5
    m.floor[:, :] = 5
    for x in range(14, 23):  # a planter 3.5-5.5 m ahead, walls 2 m high
        for y in range(-4, 5):
            if x in (14, 22) or y in (-4, 4):
                m.occ[m.cell(x * scene.CELL_M, y * scene.CELL_M)] = 9
                m.top[m.cell(x * scene.CELL_M, y * scene.CELL_M)] = 2.0
    path = m.path((0.0, 0.0), (4.5, 0.0))
    assert path and 2.5 < path[-1][0] < 3.5 and abs(path[-1][1]) < 0.5
