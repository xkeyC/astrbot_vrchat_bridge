"""The avatar's walking integrated into a pose (python -m pytest bridge)."""
import nav
import odometry


def run(pose, forward, right, seconds, dt=0.025):
    for _ in range(round(seconds / dt)):
        pose = odometry.advance(pose, forward, right, dt)
    return pose


def test_walking_ahead_and_to_the_right():
    x, y, h = run((0.0, 0.0, 90.0), 2.0, 0.0, 1.0)  # heading +y
    assert abs(x) < 1e-6 and abs(y - 2.0) < 1e-6 and h == 90.0
    x, y, _ = run((0.0, 0.0, 0.0), 0.0, 1.0, 1.0)  # right of +x is +y
    assert abs(x) < 1e-6 and abs(y - 1.0) < 1e-6


def test_dead_reckoning_of_walks_stands_aside_while_odometry_works():
    walker = nav.Navigator.__new__(nav.Navigator)
    walker.x = walker.y = walker.heading = 0.0
    walker.visited = [(0.0, 0.0)]
    walker.odometry = True
    walker.walked(1.0)
    assert (walker.x, walker.y) == (0.0, 0.0)
    walker.turned(90)  # turns are the mouse's, odometry or not
    assert walker.heading == 90.0
    walker.odometry = False
    walker.walked(1.0)
    assert abs(walker.x) < 1e-9 and walker.y == 1.0
