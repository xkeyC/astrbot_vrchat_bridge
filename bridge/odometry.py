"""The avatar's own walking, from its built-in parameters (OSCQuery).

VRChat exposes the avatar's velocity in its own frame (VelocityZ ahead,
VelocityX to the right, m/s). Polled and integrated, they place the avatar
better than dead reckoning from the inputs sent: they see walks a wall
stopped, slopes, and the follower's walking too. Turns are not taken from
them: the mouse motion sent turns the view exactly (calibrated), while its
AngularY added up varies by about 0.5% a turn.
"""

from __future__ import annotations

import asyncio
import logging
import math

import aiohttp

log = logging.getLogger("vrc-bridge")

POLL_S = 0.025
MAX_DT_S = 0.2  # a longer gap between polls is not integrated over (stale)


def advance(pose: tuple[float, float, float], forward_mps: float, right_mps: float,
            dt: float) -> tuple[float, float, float]:
    """The pose (x, y m; heading degrees) after ``dt`` s moving so (in the
    avatar's frame)."""
    x, y, heading = pose
    h = math.radians(heading)
    forward, right = forward_mps * dt, right_mps * dt
    return (x + forward * math.cos(h) - right * math.sin(h),
            y + forward * math.sin(h) + right * math.cos(h), heading)


class Odometer:
    """Polls the avatar's velocity and moves ``walker`` (a nav.Navigator) by
    it; while it works, ``walker.odometry`` is True (its dead reckoning of
    walks then stands aside)."""

    def __init__(self, port, walker) -> None:
        self.port = port  # () -> the OSCQuery port, 0 while the game is not up
        self.walker = walker

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        async with aiohttp.ClientSession() as http:
            while True:
                port = self.port()
                if not port:
                    self.walker.odometry = False
                    await asyncio.sleep(1.0)
                    continue
                last, before = loop.time(), None
                try:
                    while self.port() == port:
                        right, forward = await asyncio.gather(
                            self._param(http, port, "VelocityX"),
                            self._param(http, port, "VelocityZ"))
                        now = loop.time()
                        dt, last = now - last, now
                        self.walker.odometry = True
                        if before is not None and dt <= MAX_DT_S and any((*before, right, forward)):
                            # Trapezoids: the mean of the samples either side.
                            walker = self.walker
                            walker.moved(*advance((walker.x, walker.y, walker.heading),
                                                  (before[1] + forward) / 2,
                                                  (before[0] + right) / 2, dt))
                        before = (right, forward)
                        await asyncio.sleep(POLL_S)
                except (aiohttp.ClientError, asyncio.TimeoutError, KeyError, TypeError,
                        ValueError, IndexError) as e:
                    if self.walker.odometry:
                        log.info("odometry off: %s", e)
                    self.walker.odometry = False
                    await asyncio.sleep(1.0)

    @staticmethod
    async def _param(http: aiohttp.ClientSession, port: int, name: str) -> float:
        async with http.get(f"http://127.0.0.1:{port}/avatar/parameters/{name}",
                            timeout=aiohttp.ClientTimeout(total=1)) as resp:
            return float((await resp.json(content_type=None))["VALUE"][0])
