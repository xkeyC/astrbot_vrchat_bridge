"""Whitelist, invites and following across instances (VRChat Web API).

The whitelist is an ordered list of friends (display names or ``usr_`` ids);
its order is the priority. Invites from whitelisted friends are accepted,
and with following on the bot goes wherever the highest-priority
whitelisted friend in a joinable instance is. Public instances (and public
group instances) are never joined. Joining restarts the game into the
instance: a running client cannot be sent elsewhere.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from pathlib import Path

from vrc_api import AuthRequired, VRChatApi, instance_kind, joinable, launch_url, redact

log = logging.getLogger("vrc-bridge.social")

# A friend's new location is acted on once it held this long (they may be
# hopping through instances).
FOLLOW_SETTLE = 8.0
# Joins are at least this far apart (each one restarts the game).
JOIN_COOLDOWN = 45.0
FRIENDS_REFRESH = 300.0
RETRY_SECONDS = 60.0

# Seconds a join waits for the game it started to run.
GAME_START_WAIT = 90


def same_instance(a: str, b: str) -> bool:
    return bool(a) and a.split("~")[0] == b.split("~")[0]


class Social:
    def __init__(self, bridge, config_file: Path, cookie_file: Path) -> None:
        self.bridge = bridge
        self.config_file = config_file
        self.api = VRChatApi(cookie_file)
        self.config = {"whitelist": [], "auto_accept": True, "follow": False}
        try:
            self.config.update(json.loads(config_file.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass
        self.logged_in = False
        self.me: dict = {}
        self.friends: dict[str, dict] = {}  # usr id -> {"name", "location"}
        self._pending: asyncio.Task | None = None
        self._refresher: asyncio.Task | None = None
        # The game restart of a join under way (one at a time).
        self._switching: asyncio.Task | None = None
        self._last_join = 0.0

    # -- config / status -------------------------------------------------------

    def set_config(self, update: dict) -> None:
        for key in ("whitelist", "auto_accept", "follow"):
            if key in update:
                self.config[key] = update[key]
        whitelist = self.config["whitelist"]
        if isinstance(whitelist, str):  # a plain text field: comma separated
            whitelist = whitelist.replace("，", ",").replace("\n", ",").split(",")
        self.config["whitelist"] = [str(e).strip() for e in whitelist if str(e).strip()]
        self.config_file.parent.mkdir(parents=True, exist_ok=True)
        self.config_file.write_text(json.dumps(self.config, ensure_ascii=False), encoding="utf-8")
        self._evaluate()
        self.bridge.notify_state()  # whitelist ranks are room context

    def whitelist_ids(self) -> list[str]:
        """The whitelist as user ids, in priority order (unknown names left out)."""
        by_name = {f["name"]: uid for uid, f in self.friends.items()}
        ids = []
        for entry in self.config["whitelist"]:
            uid = entry if entry.startswith("usr_") else by_name.get(entry)
            if uid and uid not in ids:
                ids.append(uid)
        return ids

    def status(self) -> dict:
        entries = []
        by_name = {f["name"]: uid for uid, f in self.friends.items()}
        for entry in self.config["whitelist"]:
            uid = entry if entry.startswith("usr_") else by_name.get(entry, "")
            friend = self.friends.get(uid, {})
            location = friend.get("location", "")
            entries.append({
                "entry": entry, "id": uid, "name": friend.get("name", ""),
                "friend": bool(friend), "location": location,
                "kind": instance_kind(location), "joinable": joinable(location),
            })
        target = self.follow_target()
        return {
            "logged_in": self.logged_in,
            "me": {"id": self.me.get("id", ""), "name": self.me.get("displayName", "")},
            "auto_accept": self.config["auto_accept"],
            "follow": self.config["follow"],
            "follow_target": self.friends.get(target, {}).get("name", "") if target else "",
            "whitelist": entries,
        }

    def follow_target(self) -> str | None:
        """The highest-priority whitelisted friend in a joinable instance."""
        for uid in self.whitelist_ids():
            if joinable(self.friends.get(uid, {}).get("location", "")):
                return uid
        return None

    # -- running ---------------------------------------------------------------

    async def run(self) -> None:
        try:
            await self._connect_loop()
        finally:
            for task in (self._refresher, self._pending):
                if task is not None:
                    task.cancel()

    async def _connect_loop(self) -> None:
        while True:
            try:
                if not self.api.load():
                    raise AuthRequired("not logged in")
                self.me = await self.api.me()
                if not self.logged_in:
                    log.info("logged in as %s", self.me.get("displayName"))
                self.logged_in = True
                await self._refresh()
                if self._refresher is None or self._refresher.done():
                    self._refresher = asyncio.create_task(self._refresh_loop())
                await self.api.pipeline(self._on_event)
                log.warning("pipeline closed, reconnecting")
                await asyncio.sleep(5)
            except AuthRequired as exc:
                if self.logged_in or not self.me:
                    log.warning("VRChat login required: %s", exc)
                    await self.bridge.send_event({"type": "auth_required", "reason": str(exc)})
                self.logged_in = False
                await asyncio.sleep(RETRY_SECONDS)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - retried
                log.warning("VRChat API failed: %s", redact(exc))
                await asyncio.sleep(RETRY_SECONDS)

    async def _refresh_loop(self) -> None:
        while True:
            await asyncio.sleep(FRIENDS_REFRESH)
            try:
                await self._refresh()
            except Exception as exc:  # noqa: BLE001 - next round
                log.warning("friends refresh failed: %s", redact(exc))

    async def _refresh(self) -> None:
        friends = await self.api.friends()
        self.friends = {
            f["id"]: {"name": f.get("displayName", ""), "location": f.get("location", "")}
            for f in friends
        }
        self._evaluate()
        self.bridge.notify_state()  # who is a friend is room context

    async def _on_event(self, kind: str, content: dict) -> None:
        if kind == "notification" and content.get("type") == "invite":
            sender = str(content.get("senderUserId") or "")
            location = str((content.get("details") or {}).get("worldId") or "")
            await self._on_invite(sender, location, content.get("senderUsername", ""))
            return
        if kind == "notification" and content.get("type") == "requestInvite":
            await self._on_request_invite(
                str(content.get("senderUserId") or ""), content.get("senderUsername", ""))
            return
        uid = str(content.get("userId") or "")
        if kind in ("friend-location", "friend-online", "friend-active", "friend-update"):
            friend = self.friends.setdefault(uid, {"name": "", "location": ""})
            user = content.get("user") or {}
            friend["name"] = user.get("displayName") or friend["name"]
            location = content.get("location")
            if location == "traveling":
                location = content.get("travelingToLocation") or location
            if location is not None:
                friend["location"] = location
            self._evaluate()
        elif kind == "friend-offline" and uid in self.friends:
            self.friends[uid]["location"] = "offline"
        elif kind == "friend-add":
            await self._refresh()

    async def _on_invite(self, sender: str, location: str, sender_name: str) -> None:
        ids = self.whitelist_ids()
        verdict = "ignored: not whitelisted"
        if sender in ids:
            target = self.follow_target() if self.config["follow"] else None
            if not self.config["auto_accept"]:
                verdict = "ignored: auto-accept is off"
            elif not joinable(location):
                verdict = f"refused: {instance_kind(location) or 'unknown'} instance"
            elif target and ids.index(target) < ids.index(sender) and not same_instance(
                self.friends[target]["location"], location
            ):
                verdict = "ignored: following a higher-priority friend"
            else:
                verdict = "accepted"
                self._schedule(location, f"invite from {sender_name}", settle=0.0)
        log.info("invite from %s to %s: %s", sender_name, location, verdict)
        await self.bridge.send_event({"type": "invite", "from": sender_name, "location": location,
                                      "verdict": verdict})

    async def _on_request_invite(self, sender: str, sender_name: str) -> None:
        """A whitelisted friend asks to join: invite them to the bot's instance
        (as VRCX's auto-accept of invite requests does)."""
        here = self.bridge.state.instance
        if sender not in self.whitelist_ids():
            verdict = "ignored: not whitelisted"
        elif not self.config["auto_accept"]:
            verdict = "ignored: auto-accept is off"
        elif not self.bridge.state.running or not joinable(here):
            verdict = "ignored: the bot is in no joinable instance"
        else:
            try:
                await self.api.call("POST", f"/invite/{sender}", json={"instanceId": here})
                verdict = "invited"
            except Exception as exc:  # noqa: BLE001 - reported
                verdict = f"invite failed: {exc}"
        log.info("invite request from %s: %s", sender_name, verdict)
        await self.bridge.send_event({"type": "request_invite", "from": sender_name,
                                      "verdict": verdict})

    def _evaluate(self) -> None:
        """Follows the follow target if it is elsewhere."""
        if not self.config["follow"]:
            return
        target = self.follow_target()
        if not target:
            return
        location = self.friends[target]["location"]
        if same_instance(location, self.bridge.state.instance):
            if self._pending is not None:
                self._pending.cancel()
            return
        self._schedule(location, f"following {self.friends[target]['name']}", FOLLOW_SETTLE)

    def _schedule(self, location: str, reason: str, settle: float) -> None:
        if self._pending is not None and not self._pending.done():
            self._pending.cancel()
        self._pending = asyncio.create_task(self._join(location, reason, settle))

    async def _join(self, location: str, reason: str, settle: float) -> None:
        await asyncio.sleep(max(settle, self._last_join + JOIN_COOLDOWN - time.monotonic()))
        if same_instance(location, self.bridge.state.instance) or not joinable(location):
            return
        self._last_join = time.monotonic()
        log.info("joining %s (%s)", location, reason)
        await self.bridge.send_event({"type": "joining", "location": location, "reason": reason})
        # Once the game is being restarted it finishes, whatever is planned
        # meanwhile (cancelled halfway, the game would stay off); a newer join
        # waits for it.
        if self._switching is not None and not self._switching.done():
            await asyncio.shield(self._switching)
            if same_instance(location, self.bridge.state.instance):
                return
        self._switching = asyncio.create_task(self._switch(location))
        await asyncio.shield(self._switching)

    async def _switch(self, location: str) -> None:
        try:
            async with self.bridge.game_lock:
                await self.bridge.stop_game()
                await self.bridge.start_game(launch_url(location))
                # Done once the new game runs: a join after it must not stop
                # or launch it again while Steam is still starting it.
                for _ in range(GAME_START_WAIT):
                    if await self.bridge._game_pid() is not None:
                        break
                    await asyncio.sleep(1.0)
        except Exception as exc:  # noqa: BLE001 - reported
            log.error("join failed: %s", exc)
            await self.bridge.send_event({"type": "join_failed", "location": location,
                                          "error": str(exc)})
