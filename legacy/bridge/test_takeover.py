"""Following paused by the model's driving resumes by itself (python -m pytest bridge)."""
import asyncio
from types import SimpleNamespace

import vrc_bridge
from vrc_bridge import Bridge


class FakeFollower:
    def __init__(self, following=None):
        self.state = "following" if following else "idle"
        self.target = following or ""
        self.started = []

    def status(self):
        return {"distance": 2.7}

    def stop(self):
        self.state = "idle"

    def start(self, name, distance=None):
        self.started.append((name, distance))
        self.state = "following"


def fake_bridge(following=None):
    b = SimpleNamespace(
        follower=FakeFollower(following), takeover=None, _resume=None, _drive_task=None,
        state=SimpleNamespace(running=True), events=[], _background=set())
    b._spawn = lambda coro: Bridge._spawn(b, coro)
    b.notify_state = lambda: None

    async def send_event(data):
        b.events.append(data)

    async def state_changed():
        pass

    b.send_event, b._state_changed = send_event, state_changed
    b._take_over = lambda: Bridge._take_over(b)
    b._idle_later = lambda: Bridge._idle_later(b)
    b._resume_following = lambda: Bridge._resume_following(b)
    return b


def test_following_resumes_once_the_model_stops_moving(monkeypatch):
    monkeypatch.setattr(vrc_bridge, "TAKEOVER_IDLE_S", 0.2)  # wide: Windows timers are coarse

    async def main():
        b = fake_bridge(following="xkeyC")
        b._take_over()  # the model drives
        assert b.follower.state == "idle" and b.takeover["target"] == "xkeyC"
        b._idle_later()  # a move done
        await asyncio.sleep(0.1)
        b._take_over()  # another move before the time is up: no resume yet
        b._idle_later()
        await asyncio.sleep(0.15)  # past the first one's time, not the second's
        assert b.follower.started == []
        await asyncio.sleep(0.2)
        return b

    b = asyncio.run(main())
    assert b.follower.started == [("xkeyC", 2.7)]
    assert b.takeover is None
    assert b.events[-1]["state"] == "resumed"


def test_nothing_resumes_when_nobody_was_followed(monkeypatch):
    monkeypatch.setattr(vrc_bridge, "TAKEOVER_IDLE_S", 0.05)

    async def main():
        b = fake_bridge(following=None)
        b._take_over()
        b._idle_later()
        await asyncio.sleep(0.1)
        return b

    b = asyncio.run(main())
    assert b.follower.started == []
    assert b.takeover == {"target": None, "distance": None}
