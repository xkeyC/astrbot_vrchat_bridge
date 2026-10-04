"""The last time each whitelisted friend in the room was seen: the frame of
the game window that showed their name tag.

While nobody is followed, a frame is read every WATCH_INTERVAL s (PP-OCRv5
text lines, as the follower uses) when a whitelisted friend is in the room;
while following, the follower's own frames are kept whenever the tag of a
whitelisted friend shows in them. Kept in memory, newest per friend.
"""

from __future__ import annotations

import asyncio
import logging
import time

import aiohttp

from follow import MATCH_RATIO, match_score

log = logging.getLogger("vrc-bridge.sightings")

WATCH_INTERVAL = 2.0


class Sightings:
    def __init__(self, bridge) -> None:
        self.bridge = bridge
        # display name -> {"at": unix time, "world": str, "jpeg": bytes}
        self.last: dict[str, dict] = {}

    def saw(self, name: str, jpeg: bytes) -> None:
        self.last[name] = {"at": time.time(), "world": self.bridge.state.world_name, "jpeg": jpeg}

    def seen_in(self, lines: list[dict], jpeg: bytes) -> list[str]:
        """Keeps ``jpeg`` for every whitelisted friend here whose tag is
        among the OCR ``lines``; returns their names."""
        found = []
        for name in self.bridge.whitelisted_here():
            if any(match_score(line.get("text", ""), name) >= MATCH_RATIO for line in lines):
                self.saw(name, jpeg)
                found.append(name)
        return found

    def listing(self) -> list[dict]:
        now = time.time()
        return sorted(({"name": name, "at": s["at"], "age_s": round(now - s["at"], 1),
                        "world": s["world"]} for name, s in self.last.items()),
                      key=lambda s: -s["at"])

    def latest(self, name: str = "") -> tuple[str, dict] | None:
        """The newest sighting of ``name`` (best match), or of anyone."""
        if not self.last:
            return None
        if not name:
            return max(self.last.items(), key=lambda item: item[1]["at"])
        best = max(self.last, key=lambda n: match_score(n, name))
        return (best, self.last[best]) if match_score(best, name) >= MATCH_RATIO else None

    async def run(self) -> None:
        """Watches for whitelisted friends while nobody is followed."""
        follower = self.bridge.follower
        async with aiohttp.ClientSession() as http:
            while True:
                await asyncio.sleep(WATCH_INTERVAL)
                if (not self.bridge.state.running or follower.state != "idle"
                        or not self.bridge.whitelisted_here()):
                    continue
                try:
                    jpeg = await self.bridge.screenshot("jpg")
                    lines = await follower._ocr(http, jpeg)
                except Exception as exc:  # noqa: BLE001 - next round
                    log.debug("watch failed: %s", exc)
                    continue
                self.seen_in(lines, jpeg)
