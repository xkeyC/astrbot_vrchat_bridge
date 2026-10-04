"""Scripts of the model's driving (run: python -m pytest bridge)."""
import asyncio

import pytest

import drive


def test_a_step_holds_its_inputs_together():
    (step,) = drive.parse([{"move": "forward", "speed": 0.5, "run": True, "jump": True,
                            "turn": 30, "ms": 600}])
    assert (step.forward, step.right, step.run, step.jump, step.turn_deg, step.ms) == (
        0.5, 0.0, True, True, 30.0, 600)


@pytest.mark.parametrize("steps", [
    [],
    "forward",
    [{"move": "up"}],
    [{"fly": True}],
    [{"move": "forward", "ms": 3000}] * 4,  # 12 s in all
    [{"move": "forward", "ms": 800}, {"move": "left", "ms": 800}],  # 1.6 s of walking
    [{"jump": True}] * (drive.MAX_STEPS + 1),
])
def test_bad_scripts_are_refused(steps):
    with pytest.raises(ValueError):
        drive.parse(steps)


def test_values_are_bounded():
    (step,) = drive.parse([{"move": "back-left", "speed": 9, "turn": 999, "look": -200,
                            "ms": 1000}])
    assert step.forward == pytest.approx(-0.7) and step.right == pytest.approx(-0.7)
    assert (step.turn_deg, step.look_deg) == (360, -80)
    (step,) = drive.parse([{"turn": 90, "ms": 99999}])
    assert step.ms == drive.MAX_STEP_MS


@pytest.mark.parametrize(("dx", "dy"), [(0, 0), (10, 0), (-917, 120), (3665, -3)])
def test_mouse_moves_add_up_in_small_steps(dx, dy):
    path = drive.mouse_path(dx, dy)
    assert sum(x for x, _ in path) == dx and sum(y for _, y in path) == dy
    assert all(abs(x) <= drive.MOUSE_STEP_PX and abs(y) <= drive.MOUSE_STEP_PX
               for x, y in path)


class FakeOsc:
    def __init__(self):
        self.sent = []

    def send(self, address, value):
        self.sent.append((address, value))


def test_a_script_runs_in_order_and_releases_everything(monkeypatch):
    monkeypatch.setattr(drive, "JUMP_PRESS_S", 0.0)
    monkeypatch.setattr(drive, "MOUSE_STEP_S", 0.0)
    osc, moves = FakeOsc(), []

    async def mouse(dx, dy):
        moves.append((dx, dy))

    driver = drive.Driver(osc, mouse)
    steps = drive.parse([{"move": "forward", "jump": True, "ms": 20},
                         {"turn": 90, "ms": 0}])
    asyncio.run(driver.run(steps))
    assert ("/input/Vertical", 1.0) in osc.sent and ("/input/Jump", 1) in osc.sent
    # A quarter turn of mouse motion, to the right.
    assert sum(dx for dx, _ in moves) == round(90 * drive.PX_PER_DEGREE)
    # Everything let go at the end.
    assert osc.sent[-4:] == [("/input/Vertical", 0.0), ("/input/Horizontal", 0.0),
                             ("/input/Run", 0), ("/input/Jump", 0)]


def test_a_stopped_script_lets_go_too():
    osc = FakeOsc()

    async def mouse(dx, dy):
        pass

    async def main():
        task = asyncio.ensure_future(drive.Driver(osc, mouse).run(
            drive.parse([{"move": "forward", "ms": 1000}])))
        await asyncio.sleep(0.05)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(main())
    assert osc.sent[-4:] == [("/input/Vertical", 0.0), ("/input/Horizontal", 0.0),
                             ("/input/Run", 0), ("/input/Jump", 0)]
