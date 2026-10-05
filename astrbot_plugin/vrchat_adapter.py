"""VRChat platform adapter: the bot's VRChat client as one voice room.

The bridge (``bridge/vrc_bridge.py``) runs next to the game client and streams
its audio over one WebSocket: binary frames carry 16-bit mono PCM at 48 kHz
both ways (other players' voices in, the bot's voice out), text frames carry
the game state (world, instance, players). Wherever the bot is, VRChat is one
group conversation (``room``): its voice turns run as the fixed voice user,
and its text replies go to the chatbox.

Standby as on Mumble: no voice session until someone speaks (energy VAD over
the game audio, which only carries voices: world, avatar and UI sounds are
muted in the client); a session that hears nothing for a while closes.
"""

from __future__ import annotations

import asyncio
import base64
import contextlib
import inspect
import json
import time
import urllib.parse
from collections import deque
from typing import Any

import aiohttp
import numpy as np

from astrbot.api import logger
from astrbot.api.event import MessageChain
from astrbot.api.message_components import At, Plain
from astrbot.api.platform import (
    MessageType,
    Platform,
    PlatformMetadata,
    register_platform_adapter,
)
from astrbot.core.platform.astr_message_event import MessageSesion

ADAPTER_NAME = "vrchat"
# Bridge calls that move or turn the avatar (or its view): one at a time. The
# bridge stops a move when another one starts ("stopped"), and the model now
# and then asks for several in one reply.
MOVING_PATHS = ("/v1/goto", "/v1/drive", "/v1/nav", "/v1/look_around", "/v1/camera_y",
                "/v1/jump")
ROOM_SESSION = "room"
SAMPLE_RATE = 48000
FRAME_BYTES = 960 * 2  # 20 ms
RECONNECT_SECONDS = 5.0
CONNECT_TIMEOUT = 15.0
WATCHDOG_INTERVAL = 5.0
# Voice kept while in standby and replayed into a starting session, so the
# words that woke it are not lost.
PREROLL_SECONDS = 2.0
# A voice session that failed to start is not retried for this long.
START_RETRY_SECONDS = 30.0
# Upper bound of the bot's speech queued ahead.
REALTIME_BUFFER = 3.0
# The local voice server sends speech at real-time pace: a short playout
# buffer is enough (the core's default, 200 ms, is for bursty WebRTC).
LOCAL_PREBUFFER_FRAMES = 3
# Outbound audio queued for the bridge before the oldest is dropped.
OUT_QUEUE_FRAMES = 250

DEFAULT_CONFIG = {
    "id": "vrchat",
    "type": ADAPTER_NAME,
    "enable": False,
    "vrchat_bridge_url": "http://127.0.0.1:6120",
    "vrchat_bridge_token": "",
    "vrchat_voice_name": "AstrBot",
    "vrchat_voice_aliases": [],
    "vrchat_voice_prompt": "",
    "vrchat_voice_idle_timeout": 300,
    # Friends (display names or usr_ ids) in priority order: their invites and
    # invite requests are accepted, and following goes to the first of them.
    "vrchat_whitelist": [],
    "vrchat_auto_accept": True,
    "vrchat_follow": False,
}

CONFIG_METADATA = {
    "vrchat_bridge_url": {
        "description": "VRChat bridge 地址",
        "type": "string",
        "hint": "VRChat 客户端旁边运行的 bridge（vrc_bridge.py）的 HTTP 地址。",
    },
    "vrchat_bridge_token": {
        "description": "VRChat bridge Token",
        "type": "string",
        "hint": "bridge 的 token 文件（~/.config/vrc-bridge/token）的内容。",
        "secret": True,
    },
    "vrchat_voice_name": {
        "description": "语音唤醒名",
        "type": "string",
        "hint": "房间里叫到这个名字才回应（由提示词约束）。",
    },
    "vrchat_voice_aliases": {
        "description": "语音唤醒别名",
        "type": "list",
        "items": {"type": "string"},
        "hint": "名字的其他叫法，例如中文名或昵称。",
    },
    "vrchat_voice_prompt": {
        "description": "语音附加提示词",
        "type": "text",
        "hint": "追加给实时语音模型的说明，例如说话风格。",
    },
    "vrchat_voice_idle_timeout": {
        "description": "语音待机时间",
        "type": "int",
        "hint": "这么多秒没有听到有效语音就断开语音会话，有人说话时重新接入。",
    },
    "vrchat_whitelist": {
        "description": "白名单",
        "type": "list",
        "items": {"type": "string"},
        "hint": "好友的显示名或 usr_ ID，按优先级排列：接受他们的邀请、自动邀请请求加入的人；跟随时去第一个在可进入房间里的人那里。公开房间一律不进。",
    },
    "vrchat_auto_accept": {
        "description": "自动接受白名单邀请",
        "type": "bool",
        "hint": "接受白名单好友的邀请（重启游戏进入，约半分钟），并自动邀请请求加入的白名单好友。",
    },
    "vrchat_follow": {
        "description": "跨房间跟随",
        "type": "bool",
        "hint": "跟随白名单里优先级最高、且在可进入房间的好友，对方换房间时跟过去。",
    },
}

EMOTES = ("wave", "clap", "point", "cheer", "dance", "backflip", "sadness", "die")
# Pictures for the models are this wide (px).
LOOK_WIDTH = 960
VIEW_SHOW = ("around", "ahead", "map", "last_seen")
# Drives within DRIVE_WINDOW seconds before the model must stop and report
# (an agent loop needs an end even when it never finds its way).
DRIVE_BUDGET = 15
DRIVE_WINDOW = 90.0
# After a script, before its picture: the game shows the last motion.
DRIVE_SETTLE = 0.25
# Changes to a running follow (bridge POST /v1/follow {"adjust"}).
FOLLOW_CHANGES = ("closer", "farther", "stay", "resume")
DIRECTIONS = {"forward": (1, 0), "back": (-1, 0), "left": (0, -1), "right": (0, 1)}
MAX_MOVE_SECONDS = 10.0
MAX_TURN_SECONDS = 5.0

ROOM_PROMPT = """Your name is {name}.

You are in VRChat, a social virtual world, as an avatar in a room with other players. You hear the voices of the players near you; most of what you hear is them talking to each other, not to you.

The one rule that matters most: speak ONLY when the speaker says your name{aliases} to you in that utterance, or is directly continuing an exchange with you from a few seconds ago. In every other case produce no audio and no text at all - complete silence. Do not acknowledge, do not react, do not say "mm", do not comment, do not delegate.

When you are addressed, answer briefly in the speaker's language, like a person in the room. You cannot see by yourself and cannot move by yourself: delegate to the backend whatever needs your body or eyes (come here, follow, turn around, jump, look at something, write in the chatbox, who is here) and real tasks (anything needing facts, lookups or work), and tell the speaker the result briefly."""

# Replaces the last paragraph above when the voice model has the room's tools.
ROOM_TOOLS_PROMPT = """When you are addressed, answer briefly in the speaker's language, like a person in the room: spoken to in Chinese, say everything in Chinese, though your tools answer in English. Quick actions you do yourself with your tools, without delegating: gestures (vrchat_emote), a jump, a few steps, turning, looking up or down, stopping, writing in the chatbox, who is here, following someone in this room ("follow me", "come with me") until told to stop, and while following: closer, farther, stay put ("wait here", "don't move"), follow again. For a quick action (a gesture, a jump, a few steps, a turn), call its tool at once, without weighing options, then say a few words. Your view (vrchat_view, and after every move) shows numbered places you can walk to, each with its distance (gray "been": you were there), and bearings along its top (degrees to turn to face that column, + right). You see through your own eyes (first person), and move so; vrchat_view with camera third is a look from behind you (the figure in the middle at the bottom is you, never someone else): to check where you stand (up on a seat or not?) or to see more around you. To get somewhere yourself (to something you see, to find someone): vrchat_goto the number nearest your goal (it turns, walks there and stops before anything in the way), look at the new view it shows you, and again, until the last picture shows you are there (vrchat_goto never ends your turn: its result is for you to look at); never say you are there before; within about 1.5 m of it is there; with someone to find, check it is them (their name tag) before saying so. When what you look for is not in front of you, vrchat_view (all four sides at once, numbered places in every direction), then vrchat_goto the number toward it; prefer places you have not been. 0 turns you around. vrchat_step is for small precise moves: turn by degrees, walk a few metres a way (measured: it stops there, or where something is in the way), jump; never for getting somewhere. Never walk into a portal (a frame or doorway showing another world's picture or name): it takes you to another world. A mirror shows a reflection: places drawn inside a mirror are not real, walk around it. Back off a little from walls and plants you run into. When your view looks up at the sky or down at your feet (the far horizon not across the middle), vrchat_camera_y view, then level by where the horizon is. After a move its view ahead is enough; ask for all four sides (view around, or vrchat_view) only when you do not know where you are or what you look for is not ahead. In a series of moves write nothing between them, just the next call; speak once, when you are there or stuck. Any text you write is spoken aloud: never write thoughts, plans or notes (not even in brackets). Do not describe what you see unless asked. You build a map as you look and walk: name things you may want again with vrchat_note (their cell in the view ahead); vrchat_view show map shows it, with places to stand on (P1..); vrchat_goto landmark=<name> or platform=<P2> goes there by the map, around obstacles, even when out of sight. Walking to a thing only brings you next to it: to get up onto it (a table, a seat, a ledge), vrchat_goto its platform from the map with climb true, or find it in your view ahead (not all four sides: the view ahead has the grid), name the grid cell it is in (letter A-H at the bottom, row 1-5 at the left, e.g. D4) and vrchat_goto with that cell and climb true (it walks straight up to it and jumps on). The same cell, without climb, walks you right up to a thing. The result says how much higher you landed: trust that, not the picture; when it says you are up on it, you are done: stop moving and say so; at the same height you did not make it (step back a little with vrchat_step and climb again). Following is paused while you drive; vrchat_autopilot hands back to it (or ends driving). To see without moving, vrchat_view (around, or ahead); to recall when you last saw a friend, vrchat_view with show last_seen. Delegate real tasks (anything needing facts, lookups or work), and tell the speaker the result briefly."""


class SpeechDetector:
    """Energy VAD over the game audio, used to leave standby: speech is
    ``voiced_frames`` loud 20 ms frames within ``window_frames``."""

    def __init__(
        self, rms: float = 500.0, voiced_frames: int = 10, window_frames: int = 25
    ) -> None:
        self._rms = rms
        self._needed = voiced_frames
        self._recent: deque[bool] = deque(maxlen=window_frames)

    def feed(self, pcm: bytes) -> bool:
        samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
        if samples.size:
            self._recent.append(float(np.sqrt(np.mean(samples * samples))) >= self._rms)
        if sum(self._recent) >= self._needed:
            self._recent.clear()
            return True
        return False

    def clear(self) -> None:
        self._recent.clear()


def as_list(value) -> list[str]:
    """A list setting; a string (from a plain text field) is split on commas
    and line breaks."""
    if isinstance(value, str):
        value = value.replace("，", ",").replace("\n", ",").split(",")
    return [str(v).strip() for v in (value or []) if str(v).strip()]


def as_bool(value) -> bool:
    """A switch setting; strings such as "false", "0" or "off" are false."""
    if isinstance(value, str):
        return value.strip().lower() not in ("", "false", "0", "off", "no")
    return bool(value)


def instance_kind(instance: str) -> str:
    """A VRChat instance id's access, in words."""
    if "~group(" in instance:
        return "group, public" if "groupAccessType(public)" in instance else "group"
    for marker, kind in (("~private(", "invite only"), ("~hidden(", "friends+"),
                         ("~friends(", "friends only")):
        if marker in instance:
            return kind
    return "public"


def room_context(state: dict) -> str:
    """The room as the voice model is told of it: where the avatar is, who
    else is there (friends and whitelisted friends marked, by priority) and
    whom it follows."""
    if not state.get("running"):
        return "(Room now: VRChat is not running; you are in no room.)"
    world = state.get("world") or ""
    if not world:
        return "(Room now: you are between rooms, loading the next one.)"
    people = []
    players = sorted(state.get("players", []),
                     key=lambda p: (p.get("whitelist") or 1_000_000, p.get("name", "")))
    for player in players:
        if player.get("whitelist"):
            people.append(f"{player['name']} (your whitelisted friend #{player['whitelist']})")
        elif player.get("friend"):
            people.append(f"{player['name']} (your friend)")
        else:
            people.append(player.get("name", ""))
    here = ", ".join(people) if people else "nobody"
    text = f"Room now: {world} ({instance_kind(state.get('instance') or '')}). Others here: {here}."
    follow = state.get("follow") or {}
    takeover = state.get("takeover")
    if takeover is not None:
        paused = (f"; following {takeover['target']} is paused and resumes by itself once you "
                  "stop moving (or now with vrchat_autopilot)" if takeover.get("target") else "")
        text += f" You are driving yourself{paused}."
    elif follow.get("target") and follow.get("state", "idle") != "idle":
        how = {"following": "", "seeking": ", looking for them where they went",
               "searching": ", looking around for them"}.get(follow["state"], "")
        text += f" You are following {follow['target']}{how}."
    return f"({text})"


def room_prompt(name: str, aliases: list[str], tools: bool) -> str:
    others = [a for a in aliases if a and a != name]
    alias_text = (
        f' ("{name}"' + "".join(f', "{a}"' for a in others) + ")"
        if others
        else f' "{name}"'
    )
    prompt = ROOM_PROMPT.format(name=name, aliases=alias_text)
    if tools:
        prompt = prompt[: prompt.rindex("When you are addressed")] + ROOM_TOOLS_PROMPT
    # The session adds the voice persona (or the platform's extra prompt) and the time.
    return prompt


@register_platform_adapter(
    ADAPTER_NAME,
    "VRChat 平台适配器（经 VRChat bridge 实时语音 + Chatbox）",
    default_config_tmpl=dict(DEFAULT_CONFIG),
    config_metadata=CONFIG_METADATA,
    adapter_display_name="VRChat",
    logo_path="logo.svg",
    support_streaming_message=False,
)
class VRChatPlatformAdapter(Platform):
    def __init__(
        self,
        platform_config: dict,
        platform_settings: dict,
        event_queue: asyncio.Queue,
    ) -> None:
        super().__init__(platform_config, event_queue)
        cfg = {**DEFAULT_CONFIG, **platform_config}
        self.settings = platform_settings
        self.base_url = str(cfg["vrchat_bridge_url"]).rstrip("/")
        self.token = str(cfg["vrchat_bridge_token"] or "").strip()
        if not self.token:
            raise ValueError("VRChat bridge token 是必需的")
        self.idle_timeout = float(cfg["vrchat_voice_idle_timeout"] or 300)
        self.social_config = {
            "whitelist": as_list(cfg["vrchat_whitelist"]),
            "auto_accept": as_bool(cfg["vrchat_auto_accept"]),
            "follow": as_bool(cfg["vrchat_follow"]),
        }
        platform_id = str(cfg.get("id") or ADAPTER_NAME)
        self.metadata = PlatformMetadata(
            name=ADAPTER_NAME,
            description="VRChat 平台适配器",
            id=platform_id,
            support_streaming_message=False,
        )

        from astrbot.core.voice.session import VoiceOptions

        self.voice_options = VoiceOptions(
            name=str(cfg["vrchat_voice_name"] or "AstrBot"),
            aliases=as_list(cfg["vrchat_voice_aliases"]),
            extra_prompt=str(cfg["vrchat_voice_prompt"] or ""),
        )
        self.state: dict[str, Any] = {}
        self.session = None  # the room's VoiceSession, if any
        self._detector = SpeechDetector()
        self._preroll: deque[tuple[float, bytes]] = deque()
        self._retry_at = 0.0
        self._http: aiohttp.ClientSession | None = None
        self._ws: aiohttp.ClientWebSocketResponse | None = None
        self._out: asyncio.Queue[bytes] = asyncio.Queue(maxsize=OUT_QUEUE_FRAMES)
        self._tasks: set[asyncio.Task] = set()
        # The room context last given to the voice session (room_context).
        self._room_context = ""
        # The walkable places of the last view, in words (view()).
        self.nav_text = ""
        self._moving = asyncio.Lock()  # MOVING_PATHS one at a time
        # When the model drove lately (DRIVE_BUDGET).
        self._drives: list[float] = []
        self._running = True

    def meta(self) -> PlatformMetadata:
        return self.metadata

    @property
    def room_umo(self) -> str:
        return f"{self.meta().id}:{MessageType.GROUP_MESSAGE.value}:{ROOM_SESSION}"

    # -- bridge -----------------------------------------------------------

    def _headers(self) -> dict:
        return {"Authorization": f"Bearer {self.token}"}

    async def camera_y(self, action: str = "view", degrees: float | None = None) -> bytes:
        """Looking up and down (/v1/camera_y): ``view`` a tall picture from
        up to down with degree lines; ``set`` looks by ``degrees`` (+ up);
        ``level`` looks by ``degrees`` (where the horizon is) and takes that
        as level."""
        if self._http is None:
            raise RuntimeError("VRChat bridge 未连接")
        body: dict = {"action": action}
        if degrees is not None:
            body["horizon" if action == "level" else "degrees"] = degrees
        async with self._moving, self._http.post(
                self.base_url + "/v1/camera_y", headers=self._headers(), json=body,
                timeout=aiohttp.ClientTimeout(total=30)) as resp:
            if resp.status >= 300:
                data = await resp.json(content_type=None)
                raise RuntimeError(data.get("error") or f"HTTP {resp.status}")
            return await resp.read()

    async def image(self, path: str) -> tuple[bytes, dict]:
        """GETs an image of the bridge, with its response headers."""
        if self._http is None:
            raise RuntimeError("VRChat bridge 未连接")
        async with self._in_turn(path), self._http.get(
                self.base_url + path, headers=self._headers(),
                timeout=aiohttp.ClientTimeout(total=60)) as resp:
            if resp.status >= 300:
                data = await resp.json(content_type=None)
                raise RuntimeError(data.get("error") or f"HTTP {resp.status}")
            return await resp.read(), dict(resp.headers)

    def _in_turn(self, path: str) -> asyncio.Lock | contextlib.nullcontext:
        """What a call to ``path`` waits for: the moves before it finished."""
        return self._moving if path.startswith(MOVING_PATHS) else contextlib.nullcontext()

    async def request(self, method: str, path: str, body: dict | None = None) -> Any:
        """Calls the bridge's HTTP API; returns its JSON (or bytes for images)."""
        if self._http is None:
            raise RuntimeError("VRChat bridge 未连接")
        async with self._in_turn(path), self._http.request(
            method,
            self.base_url + path,
            json=body,
            headers=self._headers(),
            timeout=aiohttp.ClientTimeout(total=60),
        ) as resp:
            if resp.content_type.startswith("image/"):
                return await resp.read()
            data = await resp.json(content_type=None)
            if resp.status >= 300:
                raise RuntimeError(data.get("error") or f"HTTP {resp.status}")
            return data

    async def run(self) -> None:
        self._http = aiohttp.ClientSession()
        watchdog = asyncio.create_task(self._watchdog(), name="vrchat-voice-watchdog")
        url = self.base_url.replace("http", "ws", 1) + "/v1/stream"
        try:
            while self._running:
                try:
                    ws = await asyncio.wait_for(
                        self._http.ws_connect(url, headers=self._headers(), heartbeat=15),
                        CONNECT_TIMEOUT,
                    )
                    async with ws:
                        self._ws = ws
                        logger.info("VRChat: bridge stream connected")
                        try:
                            await self.request("POST", "/v1/social/config", self.social_config)
                        except Exception as exc:  # noqa: BLE001 - the bridge keeps its own
                            logger.warning("VRChat: whitelist not sent: %s", exc)
                        writer = asyncio.create_task(self._write(ws))
                        try:
                            async for msg in ws:
                                if msg.type == aiohttp.WSMsgType.BINARY:
                                    self._on_audio(msg.data)
                                elif msg.type == aiohttp.WSMsgType.TEXT:
                                    self._on_text(json.loads(msg.data))
                        finally:
                            writer.cancel()
                            self._ws = None
                except asyncio.CancelledError:
                    raise
                except Exception as exc:  # noqa: BLE001 - retried
                    if not self._running:
                        break
                    logger.warning("VRChat: bridge stream failed: %s", exc)
                await self._close_voice("bridge stream lost")
                if self._running:
                    await asyncio.sleep(RECONNECT_SECONDS)
        finally:
            watchdog.cancel()

    async def _write(self, ws: aiohttp.ClientWebSocketResponse) -> None:
        while True:
            await ws.send_bytes(await self._out.get())

    def _send_audio(self, chunk: bytes) -> None:
        if self._ws is None:
            return
        if self._out.full():  # the bridge fell behind: drop the oldest audio
            with contextlib.suppress(asyncio.QueueEmpty):
                self._out.get_nowait()
        self._out.put_nowait(chunk)

    def _on_text(self, data: dict) -> None:
        kind = data.get("type")
        if kind == "state":
            old = {p["id"] for p in self.state.get("players", [])}
            self.state = data.get("state") or {}
            new = {p["id"]: p["name"] for p in self.state.get("players", [])}
            joined = [name for pid, name in new.items() if pid not in old]
            if joined:
                logger.info("VRChat: %s joined %s", ", ".join(joined), self.state.get("world"))
            self._refresh_context()
        elif kind in ("alert", "auth_required", "join_failed"):
            logger.warning("VRChat bridge: %s", data)
        elif kind in ("invite", "request_invite", "joining", "follow"):
            logger.info("VRChat bridge: %s", data)

    def _refresh_context(self) -> None:
        """Gives the voice session the room context when it changed (people
        coming or going, another room, whom the avatar follows)."""
        text = room_context(self.state)
        if text == self._room_context:
            return
        self._room_context = text
        session = self.session
        if session is not None and hasattr(session, "set_context"):
            task = asyncio.get_running_loop().create_task(session.set_context(text))
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)

    async def terminate(self) -> None:
        self._running = False
        await self._close_voice("terminated")
        for task in list(self._tasks):
            task.cancel()
        if self._ws is not None:
            await self._ws.close()
        if self._http is not None:
            await self._http.close()

    def get_client(self) -> VRChatPlatformAdapter:
        return self

    # -- actions (tools of the room's chat and of its voice) ------------------

    async def move(self, direction: str, seconds: float = 1.0, run: bool = False) -> dict:
        """Walks relative to the facing direction, then stops.

        Args:
            direction: forward, back, left or right.
            seconds: How long, at most 10.
            run: Run instead of walking.

        Returns:
            The bridge's answer.

        Raises:
            ValueError: An unknown direction.
        """
        if direction not in DIRECTIONS:
            raise ValueError("direction must be forward, back, left or right")
        forward, right = DIRECTIONS[direction]
        return await self.request("POST", "/v1/move", {
            "forward": forward, "right": right, "run": bool(run),
            "seconds": max(0.0, min(float(seconds), MAX_MOVE_SECONDS))})

    async def turn(self, direction: str, seconds: float = 0.5) -> dict:
        if direction not in ("left", "right"):
            raise ValueError("direction must be left or right")
        return await self.request("POST", "/v1/turn", {
            "speed": -1.0 if direction == "left" else 1.0,
            "seconds": max(0.0, min(float(seconds), MAX_TURN_SECONDS))})

    async def look(self, direction: str, amount: int = 200) -> dict:
        if direction not in ("up", "down"):
            raise ValueError("direction must be up or down")
        amount = min(abs(int(amount)), 600)
        return await self.request("POST", "/v1/look", {"dy": -amount if direction == "up" else amount})

    async def follow(self, name: str = "", stop: bool = False,
                     distance: float | None = None) -> dict:
        """Follows a player in the room by their name tag (by default the
        highest-priority whitelisted one here), standing ``distance`` metres
        away (default about 2.7), or stops following."""
        if stop:
            return await self.request("POST", "/v1/follow", {"stop": True})
        body: dict = {"name": name.strip()}
        if distance is not None:
            body["distance"] = float(distance)
        return await self.request("POST", "/v1/follow", body)

    async def follow_adjust(self, change: str) -> dict:
        """While following: closer, farther, stay (put, still facing them) or
        resume; not following, stay just stops moving."""
        if change not in FOLLOW_CHANGES:
            raise ValueError(f"change must be one of {', '.join(FOLLOW_CHANGES)}")
        return await self.request("POST", "/v1/follow", {"adjust": change})

    async def view(self, camera: str = "first", keep_pitch: bool = False) -> bytes:
        """The game view now (JPEG, LOOK_WIDTH px wide): bearings marked
        along its top, numbered places to walk to (``nav_text`` lists them);
        ``camera`` third: a look from behind the avatar. The view is put
        level first unless ``keep_pitch`` (right after looking up or down)."""
        third = "&camera=third" if camera == "third" else ""
        keep = "&pitch=keep" if keep_pitch else ""
        return self._nav(*await self.image(f"/v1/nav?width={LOOK_WIDTH}{third}{keep}"))

    async def look_around(self, camera: str = "first") -> bytes:
        """Four views around the avatar (2x2: ahead, right / behind, left),
        their walkable places numbered across all (``nav_text``)."""
        third = "?camera=third" if camera == "third" else ""
        return self._nav(*await self.image(f"/v1/look_around{third}"))

    def _nav(self, jpeg: bytes, headers: dict) -> bytes:
        marks = json.loads(headers.get("X-Nav-Marks") or "[]")
        ways = "; ".join(
            f"{m['n']}: {abs(m['bearing'])} degrees {'right' if m['bearing'] > 0 else 'left'}"
            f"{'' if m['bearing'] else ' (ahead)'}, {m['m']} m{' (been there)' if m['been'] else ''}"
            for m in marks)
        if not marks:
            ways = "none in view (vrchat_view shows all four sides)"
        camera = ("This look is in third person, from behind you: the figure in the middle "
                  "at the bottom is you (no one else); the places' distances are from your "
                  "feet; it has no grid. You are back in first person now. "
                  if headers.get("X-Nav-Camera") == "third" else "")
        self.nav_text = (f"{camera}Walkable places: {ways}; 0: turn around. "
                         f"You are {headers.get('X-Nav-Pose', '')}.")
        return jpeg

    async def map_view(self) -> tuple[bytes, str]:
        """The scene map (top-down, facing up) and its places in words."""
        jpeg, headers = await self.image("/v1/map?width=480")
        places = json.loads(headers.get("X-Map-Places") or "[]")
        words = []
        for p in places:
            side = "ahead" if abs(p["bearing"]) < 10 else (
                f"{abs(p['bearing'])} degrees {'right' if p['bearing'] > 0 else 'left'}")
            what = (f"{p['name']} (a place to stand on, {p['height_m']} m high)"
                    if p["kind"] == "platform" else p["name"])
            words.append(f"{what}: {p['m']} m, {side}")
        return jpeg, ("On the map: " + "; ".join(words) + "." if words
                      else "Nothing named on the map yet.")

    async def note(self, name: str, cell: str) -> dict:
        return await self.request("POST", "/v1/note", {"name": name, "cell": cell})

    async def seen_after(self, view: str, keep_pitch: bool = False) -> bytes | None:
        """The view after a move: ``around`` (all four sides), ``ahead``
        or ``none``; ``keep_pitch`` after a look up or down (not levelled)."""
        if view == "none":
            return None
        if view == "ahead":
            return await self.view(keep_pitch=keep_pitch)
        return await self.look_around()

    async def goto(self, mark: int, detour: str = "auto", view: str = "ahead",
                   climb: bool = False, bearing: float | None = None,
                   distance: float | None = None, cell: str | None = None,
                   landmark: str | None = None,
                   platform: str | None = None) -> tuple[dict, bytes | None]:
        """Walks to a numbered place of the last view (0: turns around),
        going around what is in the way (``detour``: auto, left, right,
        none); returns how it went and the view after it (``view``)."""
        body = {"mark": int(mark), "detour": detour, "climb": bool(climb)}
        if landmark:
            body["landmark"] = str(landmark)
        elif platform:
            body["platform"] = str(platform)
        elif cell:
            body["cell"] = str(cell)
        elif bearing is not None:
            body.update(bearing=float(bearing), distance=distance)
        result = await self.request("POST", "/v1/goto", body)
        return result, await self.seen_after(view)

    async def drive(self, steps: list, view: str = "ahead") -> tuple[dict, bytes | None]:
        """Runs a script of steps on the avatar (pausing any following);
        returns how it went and the view after it (``view``)."""
        now = time.monotonic()
        self._drives = [t for t in self._drives if now - t < DRIVE_WINDOW] + [now]
        if len(self._drives) > DRIVE_BUDGET:
            raise RuntimeError(
                f"you drove {DRIVE_BUDGET} times in {DRIVE_WINDOW:.0f} s: stop, tell the speaker "
                "where you are and what is in the way, and ask how to go on")
        result = await self.request("POST", "/v1/drive", {"steps": steps})
        if view == "none":
            return result, None
        await asyncio.sleep(DRIVE_SETTLE)  # the last motion shows
        looked = any(isinstance(s, dict) and s.get("look") for s in steps)
        return result, await self.seen_after(view, keep_pitch=looked)

    async def autopilot(self) -> dict:
        """Ends driving: following whom it paused, if anyone."""
        return await self.request("POST", "/v1/autopilot", {})

    async def last_seen(self, name: str = "") -> tuple[dict, bytes] | None:
        """The last sighting of a whitelisted friend (``name``, or whoever
        was seen last): its details (name, at, age_s, world) and frame."""
        found = await self.request(
            "GET", "/v1/sightings?" + urllib.parse.urlencode({"name": name}))
        sighting = found.get("match")
        if not sighting:
            return None
        query = urllib.parse.urlencode({"name": sighting["name"], "width": LOOK_WIDTH})
        try:
            jpeg = await self.request("GET", f"/v1/sightings/image?{query}")
        except RuntimeError:
            return None
        return sighting, jpeg

    @property
    def bridge_connected(self) -> bool:
        return self._ws is not None

    def players(self) -> list[str]:
        """Display names of the other players in the room."""
        return [p["name"] for p in self.state.get("players", [])]

    def _voice_tools(self) -> list:
        """The room's quick actions for the voice model; none with a core
        that has no ``VoiceTool``."""
        try:
            from astrbot.core.voice.session import VoiceTool
        except ImportError:
            return []

        def spec(name: str, description: str, properties: dict, required: list[str]) -> dict:
            return {"type": "function", "name": name, "description": description,
                    "inputSchema": {"type": "object", "properties": properties,
                                    "required": required, "additionalProperties": False}}

        async def emote(a: dict) -> str:
            await self.request("POST", "/v1/emote", {"name": str(a.get("name"))})
            return "Done."

        async def jump(a: dict) -> str:
            await self.request("POST", "/v1/jump")
            return "Done."

        async def stop(a: dict) -> str:
            await self.request("POST", "/v1/stop")
            return "Done."

        async def chatbox(a: dict) -> str:
            await self.request("POST", "/v1/chatbox", {"text": str(a.get("text", ""))})
            return "Shown."

        async def follow(a: dict) -> str:
            if a.get("stop"):
                await self.follow(stop=True)
                return "Stopped following."
            distance = a.get("distance")
            status = await self.follow(str(a.get("name") or ""),
                                       distance=None if distance is None else float(distance))
            return f"Following {status.get('target')} at about {status.get('distance')} m."

        async def follow_adjust(a: dict) -> str:
            status = await self.follow_adjust(str(a.get("change")))
            if status.get("state") == "idle":
                return "Standing still."
            if status.get("hold"):
                return "Staying here, still facing them."
            return f"Following at about {status.get('distance')} m."

        async def who(a: dict) -> str:
            names = self.players()
            return "Here: " + ", ".join(names) if names else "Nobody else is here."

        def picture(text: str, jpeg: bytes) -> list[dict]:
            data = base64.b64encode(jpeg).decode()
            return [{"type": "inputText", "text": text},
                    {"type": "inputImage", "imageUrl": f"data:image/jpeg;base64,{data}"}]

        async def view(a: dict) -> list[dict] | str:
            show = a.get("show") or "around"
            if show == "last_seen":
                return await last_seen(a)
            if show == "map":
                jpeg, words = await self.map_view()
                return picture("Your map so far (top-down, you in the middle facing up; "
                               f"light: floor, black: obstacles, amber: things to stand on). {words}",
                               jpeg)
            camera = str(a.get("camera") or "first")
            if show == "ahead":
                jpeg = await self.view(camera)
                return picture(f"Ahead of you now ({time.strftime('%H:%M:%S')}). {self.nav_text}",
                               jpeg)
            jpeg = await self.look_around(camera)
            return picture("Around you now (ahead, right / behind, left; you face as before). "
                           f"{self.nav_text}", jpeg)

        def landed(jump: dict | None) -> str:
            if not jump:
                return ""
            if jump.get("fell"):
                return (" You fell a long way (off the world?) and are somewhere else now: "
                        "look around before anything else.")
            rise = jump["height_change_m"]
            if rise >= 0.25:
                return (f" You landed {rise:.1f} m higher than before: you are up on it. Done: "
                        "stay there, no more moves.")
            if rise >= 0.1:
                return (f" You landed only {rise:.1f} m higher: likely not up on it (a "
                        "vrchat_view camera third shows you where you stand).")
            if rise <= -0.15:
                return f" You landed {abs(rise):.1f} m lower than before."
            return " You landed at the same height: you did not get up onto anything."

        def shown(text: str, jpeg: bytes | None, view: str) -> list[dict] | str:
            if jpeg is None:
                return text
            where = ("Around you now (ahead, right / behind, left; you face as before)"
                     if view == "around" else "Your view now")
            return picture(f"{text} {where}: {self.nav_text}", jpeg)

        async def goto(a: dict) -> list[dict] | str:
            view = str(a.get("view") or "ahead")  # never ends the turn: the model looks
            result, jpeg = await self.goto(int(a.get("mark", -1)),
                                           str(a.get("detour") or "auto"), view,
                                           bool(a.get("climb")), a.get("bearing"),
                                           a.get("distance"), a.get("cell"),
                                           a.get("landmark"), a.get("platform"))
            if result.get("stopped") == "turned":
                done = "Turned around."
            elif not result.get("ok"):
                done = "Stopped."
            else:
                done = {"arrived": "Got there", "blocked": "Stopped: blocked, no way around",
                        "no way": "Found no way there on the map",
                        "too long": "Stopped short", "reached it": "Walked up to it",
                        "nothing there": "Found nothing to climb that way"}.get(
                            result.get("stopped"), "Walked")
                done += f" (walked {result.get('walked_m')} m"
                if result.get("detours"):
                    done += f", around {result['detours']} obstacle(s)"
                if result.get("stopped") != "arrived" and not a.get("climb"):
                    done += f", {result.get('left_m')} m still to go"
                done += ")." + landed(result.get("jump"))
                if result.get("to") and not a.get("climb"):
                    done += (f" You face where your map puts {result['to']}; the map drifts "
                             "as you walk: check your view that it is really there before "
                             "saying so.")
            return shown(done, jpeg, view)

        async def step(a: dict) -> list[dict] | str:
            view = str(a.get("view") or "ahead")
            result = await self.request("POST", "/v1/step", {
                "turn": a.get("turn") or 0, "direction": a.get("direction") or "forward",
                "meters": a.get("meters") or 0, "jump": bool(a.get("jump"))})
            jpeg = await self.seen_after(view) if view != "none" else None
            if not result.get("ok"):
                return shown("Stopped before the end.", jpeg, view)
            done = f"Turned {result['turned']:+.0f} degrees. " if result.get("turned") else ""
            ahead, right = result["moved"]["ahead_m"], result["moved"]["right_m"]
            parts = [f"{abs(ahead)} m {'ahead' if ahead > 0 else 'back'}" if ahead else "",
                     f"{abs(right)} m {'right' if right > 0 else 'left'}" if right else ""]
            parts = [p for p in parts if p]
            if a.get("meters"):
                done += (f"You went {' and '.join(parts)} (of where you faced)."
                         if parts else "You did not get anywhere.")
                if result.get("blocked"):
                    done += " Something stopped you."
            return shown((done or "Done.") + landed(result.get("jump")), jpeg, view)

        async def camera_y(a: dict) -> list[dict] | str:
            action = str(a.get("action") or "view")
            if action not in ("view", "set", "level"):
                action = "view"
            key = "horizon" if action == "level" else "degrees"
            if action != "view" and a.get(key) is None:
                where = "the far horizon is on" if action == "level" else "to look to"
                return (f"{action} needs {key}: degrees, the line of the tall picture "
                        f"(action view) {where}, + up, - down.")
            jpeg = await self.camera_y(action, None if action == "view" else float(a[key]))
            if action == "view":
                return picture(
                    "A tall picture from looking up (top) to looking down (bottom), lines every "
                    "10 degrees from where you look now (yellow, now 0). Looking level, the far "
                    "horizon (where the distant ground meets what is beyond) and the eyes of "
                    "people standing far off are on the now line. To look level: action level "
                    "with horizon = the line the horizon is on (e.g. -30). To look at something "
                    "up or down: action set with its line's degrees.", jpeg)
            if action == "level":
                return picture("Looked to the horizon, taken as level from now on: your views "
                               "and walks keep it. The far horizon should be on the yellow line "
                               "now; if not, view again.", jpeg)
            return picture("Your view now, lines every 10 degrees; your next view or walk "
                           "looks level again.", jpeg)

        async def note(a: dict) -> str:
            found = await self.note(str(a.get("name") or ""), str(a.get("cell") or ""))
            return (f"Noted {found['name']} on your map ({found['m']} m away): "
                    "vrchat_goto landmark finds it again.")

        async def autopilot(a: dict) -> str:
            result = await self.autopilot()
            if result.get("following"):
                return f"Following {result['following']} again."
            return "Done driving."

        async def last_seen(a: dict) -> list[dict] | str:
            found = await self.last_seen(str(a.get("name") or ""))
            if found is None:
                return "No whitelisted friend has been seen yet."
            sighting, jpeg = found
            return picture(f"Your view when you last saw {sighting['name']}: "
                           f"{sighting['age_s']:.0f} s ago, in {sighting['world'] or 'an unknown world'}.",
                           jpeg)

        move_view = {"type": "string", "enum": ["ahead", "around", "none"],
                     "description": "What it shows you after the move: ahead (default; it has "
                                    "the grid to point at things), around (all four sides; slow, "
                                    "about 9 s: only when you are lost), or none."}
        actions = [
            (spec("vrchat_emote", "Plays a gesture of your avatar.", {
                "name": {"type": "string", "enum": list(EMOTES)}}, ["name"]), emote),
            (spec("vrchat_jump", "Jumps once.", {}, []), jump),
            (spec("vrchat_stop", "Stops walking or turning at once.", {}, []), stop),
            (spec("vrchat_chatbox", "Shows a short text above your head (the chatbox).", {
                "text": {"type": "string"}}, ["text"]), chatbox),
            (spec("vrchat_follow_player",
                  "Follows a player in this room, facing them at a set distance, until told to stop or they leave.", {
                      "name": {"type": "string",
                               "description": "Their display name; leave out for the first of your friends who is here."},
                      "stop": {"type": "boolean", "description": "True to stop following."},
                      "distance": {"type": "number",
                                   "description": "How far from them to stand, metres (0.8-7; default about 1.5)."},
                      }, []), follow),
            (spec("vrchat_follow_adjust",
                  "While following: come closer, keep farther, stay put (still facing them) or "
                  "follow again; not following, stay just stops moving.", {
                      "change": {"type": "string", "enum": list(FOLLOW_CHANGES)}},
                  ["change"]), follow_adjust),
        ]
        # No action ends the turn by itself: the model sees how it went and
        # answers in its own words (it ended turns early with a walk).
        tools = [VoiceTool(s, run) for s, run in actions]
        return tools + [
            VoiceTool(spec("vrchat_goto",
                           "How you get anywhere: walks to a numbered place of your last view "
                           "(it turns, walks there and goes around what is in the way); 0 "
                           "turns around. Then shows you your view ahead, with new places.", {
                               "mark": {"type": "integer",
                                        "description": "The place's number in your last view "
                                                       "(or give bearing instead)."},
                               "view": move_view,
                               "landmark": {"type": "string",
                                            "description": "Instead of a mark: a thing you named "
                                                           "(vrchat_note), even out of sight: "
                                                           "goes there by the map, around "
                                                           "obstacles."},
                               "platform": {"type": "string",
                                            "description": "Instead of a mark: a place to stand "
                                                           "on from your map (P1..), with climb "
                                                           "true to get onto it."},
                               "cell": {"type": "string",
                                        "description": "Instead of a mark: the grid cell of your "
                                                       "view ahead the thing is in (letter A-H "
                                                       "along the bottom, row 1-5 at the left, "
                                                       "e.g. D4): walks straight up to it."},
                               "bearing": {"type": "number",
                                           "description": "Instead of a mark: degrees off your "
                                                          "view ahead, as its ruler marks them "
                                                          "(+ right; 180: back), e.g. a thing to "
                                                          "climb on, or a distance to go."},
                               "distance": {"type": "number",
                                            "description": "With bearing: metres to walk "
                                                           "(default: as far as is free)."},
                               "climb": {"type": "boolean",
                                         "description": "With cell: true to get up onto the "
                                                        "thing in that cell (a table, a seat, a "
                                                        "ledge): walks up to it and jumps on; the "
                                                        "result says if you got up."},
                               "detour": {"type": "string", "enum": ["auto", "left", "right", "none"],
                                          "description": "Going around what is in the way: "
                                                         "auto (default), by the left or the "
                                                         "right, or none (stop at it)."}},
                           []), goto),
            VoiceTool(spec("vrchat_step",
                           "Small precise moves, not for getting somewhere (that is vrchat_goto): "
                           "turn by degrees, then walk a few metres a way (measured: it stops "
                           "there, or where something stops it), or jump. Says how far you went "
                           "and shows you your view.", {
                               "turn": {"type": "number",
                                        "description": "Degrees to turn first, + right, - left "
                                                       "(90 a quarter turn, 180 around)."},
                               "direction": {"type": "string",
                                             "enum": ["forward", "back", "left", "right"],
                                             "description": "Which way to walk (of where you "
                                                            "face after the turn)."},
                               "meters": {"type": "number",
                                          "description": "How far to walk, 0-5 (0: no walk)."},
                               "jump": {"type": "boolean",
                                        "description": "Jump as you start (in place without "
                                                       "meters); the result says how high you "
                                                       "landed."},
                               "view": move_view}, []), step),
            VoiceTool(spec("vrchat_camera_y",
                           "Looking up and down. view: a tall picture from looking up to looking "
                           "down, degree lines from where you look now; level: look to where the "
                           "far horizon is (its line in that picture), taken as level from then "
                           "on; set: look up or down by degrees to see something. Use it when "
                           "your view looks up at the sky or down at your feet, or the speaker "
                           "says so: view first, then level.", {
                               "action": {"type": "string", "enum": ["view", "level", "set"]},
                               "horizon": {"type": "number",
                                           "description": "level (needed): the line the far "
                                                          "horizon is on in the tall picture, "
                                                          "+ up, - down (e.g. -30)."},
                               "degrees": {"type": "number",
                                           "description": "set (needed): degrees to look, + "
                                                          "up, - down."}},
                           ["action"]), camera_y),
            VoiceTool(spec("vrchat_note",
                           "Puts a thing you see on your map by name, to find it again later "
                           "(even out of sight): its grid cell in your view ahead.", {
                               "name": {"type": "string", "description": "A short name, e.g. "
                                                                         "yellow seat."},
                               "cell": {"type": "string",
                                        "description": "Its cell in your view ahead, e.g. D4."}},
                           ["name", "cell"]), note),
            VoiceTool(spec("vrchat_autopilot",
                           "Stops driving yourself: follows again whom following was paused "
                           "for, if anyone.", {}, []),
                      autopilot),
            VoiceTool(spec("vrchat_who", "Lists the other players in the room.", {}, []), who),
            VoiceTool(spec("vrchat_view",
                           "Shows you what you see. around (default): a full circle, all four "
                           "sides at once (ahead, right / behind, left) with the walkable "
                           "places numbered; ahead: only in front (quick); map: your map so far "
                           "(top-down, the things you named and places to stand on); last_seen: "
                           "your view when you last saw a friend of your whitelist.", {
                               "show": {"type": "string", "enum": list(VIEW_SHOW)},
                               "camera": {"type": "string", "enum": ["first", "third"],
                                          "description": "around or ahead: first (default, your "
                                                         "eyes) or third: a look from behind "
                                                         "you, yourself in it, to see where you "
                                                         "stand (up on a seat?) and more around "
                                                         "you; you move in first person."},
                               "name": {"type": "string",
                                        "description": "last_seen: their display name; leave "
                                                       "out for whoever was seen last."}}, []),
                      view),
        ]

    # -- text: the chatbox -------------------------------------------------

    async def send_chain(self, chain: MessageChain) -> None:
        text = ""
        for component in chain.chain:
            if isinstance(component, Plain):
                text += component.text
            elif isinstance(component, At):
                text += f"@{component.name or component.qq}"
        text = text.strip()
        if not text:
            return
        try:
            await self.request("POST", "/v1/chatbox", {"text": text})
        except Exception as exc:  # noqa: BLE001 - a lost chatbox line
            logger.warning("VRChat: chatbox failed: %s", exc)

    async def send_by_session(
        self, session: MessageSesion, message_chain: MessageChain
    ) -> None:
        await self.send_chain(message_chain)
        await super().send_by_session(session, message_chain)

    # -- voice ------------------------------------------------------------

    def _on_audio(self, pcm: bytes) -> None:
        if self.session is not None:
            self.session.media.feed(pcm)
            return
        # Standby: keep a short pre-roll and wait for real speech.
        now = time.monotonic()
        self._preroll.append((now, pcm))
        while self._preroll and now - self._preroll[0][0] > PREROLL_SECONDS:
            self._preroll.popleft()
        if now < self._retry_at or not self._detector.feed(pcm):
            return
        try:
            session = self._start_voice()
        except Exception as exc:  # noqa: BLE001 - the stream goes on
            logger.error("VRChat voice session could not start: %s", exc)
            self._retry_at = now + START_RETRY_SECONDS
            return
        for _, chunk in self._preroll:
            session.media.feed(chunk)
        self._preroll.clear()

    def _start_voice(self):
        from astrbot.core.voice.chat import VoiceChat
        from astrbot.core.voice.pcm import PcmMedia
        from astrbot.core.voice.session import new_voice_session, realtime_voice_config

        local = realtime_voice_config()["backend"] == "local_infra"
        prebuffer = "prebuffer_frames" in inspect.signature(PcmMedia).parameters
        # Only the local voice thread runs tools itself (VoiceTool).
        tools = self._voice_tools() if local else []
        extra = {"tools": tools} if tools else {}
        session = new_voice_session(
            key=ROOM_SESSION,
            scope_id=f"{self.meta().id}:voice:{ROOM_SESSION}",
            prompt=room_prompt(self.voice_options.name, self.voice_options.aliases, bool(tools)),
            options=self.voice_options,
            media=PcmMedia(
                self._send_audio,
                buffer_seconds=REALTIME_BUFFER,
                # A realtime peer sends silence all along (trimmed from a
                # backlog); the local voice server sends only speech, at
                # real-time pace, whose pauses are part of it.
                trim_silence=not local,
                **({"prebuffer_frames": LOCAL_PREBUFFER_FRAMES} if local and prebuffer else {}),
            ),
            on_closed=self._voice_closed,
            label="VRChat",
            thread_key="vrchat_voice_thread",
            # Speakers cannot be told apart: the room's turns run as the fixed
            # voice user, a member.
            chat=VoiceChat(umo=self.room_umo, private=False, via="VRChat voice"),
            **extra,
        )
        self.session = session
        if hasattr(session, "set_context"):
            # Before the launch: the conversation starts with it.
            self._room_context = room_context(self.state)
            task = asyncio.get_running_loop().create_task(
                session.set_context(self._room_context))
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)

        def failed(exc: Exception) -> None:
            logger.error("VRChat voice session failed to start: %s", exc)
            self._retry_at = time.monotonic() + START_RETRY_SECONDS

        session.launch(failed)
        logger.info("VRChat: voice session started (%s)", "local" if local else "realtime")
        return session

    def _voice_closed(self, session) -> None:
        if self.session is session:
            self.session = None
        self._detector.clear()

    async def _close_voice(self, reason: str) -> None:
        session, self.session = self.session, None
        if session is not None:
            session.media.stop()
            await session.close(reason)
        while not self._out.empty():
            self._out.get_nowait()

    async def _watchdog(self) -> None:
        while True:
            await asyncio.sleep(WATCHDOG_INTERVAL)
            session = self.session
            # A session still starting has its own timeouts.
            if (
                session is not None
                and session.ready
                and time.monotonic() - session.last_activity >= self.idle_timeout
            ):
                await self._close_voice("standby: no speech recognised")


def find_adapter(context) -> VRChatPlatformAdapter | None:
    """The running VRChat adapter, if any."""
    for platform in context.platform_manager.platform_insts:
        if isinstance(platform, VRChatPlatformAdapter):
            return platform
    return None
