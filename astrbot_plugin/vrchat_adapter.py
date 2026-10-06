"""VRChat platform adapter: the bot's VRChat client as one voice room.

The bridge (``crates/vrc-bridge``, Rust) runs next to the game client (VR mode
on a virtual headset, ``docs/full-vr/``) and streams its audio over one
WebSocket: binary frames carry 16-bit mono PCM at 48 kHz
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
# Bridge calls that move or turn the avatar (or its head): one at a time (the
# model now and then asks for several in one reply).
MOVING_PATHS = ("/v1/vr/", "/v1/step", "/v1/jump")
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
# A pause this long ends a stretch of speech (timing logs).
SPEECH_GAP_SECONDS = 0.6
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
        "hint": "VRChat 客户端旁边运行的 bridge（vrc-bridge）的 HTTP 地址。",
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
# Changes to a running follow (bridge POST /v1/follow {"adjust"}).
FOLLOW_CHANGES = ("closer", "farther", "stay", "resume")

ROOM_PROMPT = """Your name is {name}.

You are in VRChat, a social virtual world, as an avatar in a room with other players. You hear the voices of the players near you; most of the talk around you is them talking to each other, not to you.

{rule}

When you are addressed, answer briefly in the speaker's language, like a person in the room. You cannot see by yourself and cannot move by yourself: delegate to the backend whatever needs your body or eyes (come here, follow, turn around, jump, look at something, write in the chatbox, who is here) and real tasks (anything needing facts, lookups or work), and tell the speaker the result briefly."""

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
    if follow.get("target") and follow.get("state", "idle") != "idle":
        how = {"following": "", "searching": ", looking around for them"}.get(follow["state"], "")
        text += f" You are following {follow['target']}{how}."
    takeover = state.get("takeover") or {}
    if takeover.get("target"):
        text += (f" Following {takeover['target']} is paused while you move yourself; it resumes"
                 " by itself about 10 s after your last move (vrchat_stop to stay put).")
    return f"({text})"


VR_LOOK_DESCRIPTION = (
    "Looks all around (about 2 s) and shows you two pictures with the same numbered places: "
    "a panorama (its middle is where you face, its edges behind you) and a top-down map (you "
    "in the middle facing up; green floor, red obstacles, dark unknown). Players are found by "
    "their name tags; whitelisted friends are marked.")
VR_WALK_DESCRIPTION = (
    "Walks to a numbered place of your last look around (it plans a path around obstacles, "
    "walks in short legs and looks again after each), or a distance at a bearing. Then shows "
    "you the new look around.")
VR_WALK_PARAMS = {
    "place": {"type": "integer", "description": "The place's number in your last look around."},
    "bearing": {"type": "number",
                "description": "Instead of a place: degrees from where you face (+ right, 180 behind)."},
    "distance": {"type": "number", "description": "With bearing: metres to walk (default 2)."},
}
# Replaces the last paragraph of ROOM_PROMPT when the voice model has the room's tools.
ROOM_TOOLS_PROMPT = ("When you are addressed, answer briefly in the speaker's language, like a person in the room: spoken to in Chinese, say everything in Chinese, though your tools answer in English. Quick actions you do yourself with your tools, without delegating: gestures (vrchat_emote), a jump, a few steps or a turn (vrchat_step), stopping, writing in the chatbox, who is here, following someone in this room ('follow me', 'come with me') until told to stop, and while following: closer, farther, stay put ('wait here', 'don't move'), follow again. For a quick action call its tool at once, without weighing options, then say a few words. To see where you are, vrchat_look_around: a panorama and a top-down map with numbered places (players by name, places to walk to, edges of what you have seen to look further from, raised tops to jump onto); its text lists them with distance and bearing (+ right of where you face). To get somewhere, vrchat_walk_to the number nearest your goal, then look at the new pictures it shows, and again until you are there (within about 1.5 m); with someone to find, walk to their number. When what you look for is not among the places, walk to an edge toward where it may be and look again. To recall when you last saw a friend, vrchat_last_seen. Told you stand on tiptoe or crouch, or to be taller or shorter, vrchat_height; your view or body stuck or wrong, vrchat_vr_reset. A mirror shows a reflection: places that seem to lie inside or behind a mirror are not real. Never walk into a portal (a frame showing another world). In a series of moves write nothing between them, just the next call; speak once, when you are there or stuck. Any text you write is spoken aloud: never write thoughts, plans or notes (not even in brackets). Do not describe what you see unless asked. Delegate real tasks (anything needing facts, lookups or work), and tell the speaker the result briefly.")


HEIGHT_DESCRIPTION = ("Sets how high your virtual headset stands above the floor, which is your avatar's posture: "
                      "standing on tiptoe means too high, bent knees too low (a few centimetres matter). "
                      "Without arguments, tells the height now.")
HEIGHT_PARAMS = {
    "metres": {"type": "number", "description": "The height, metres (1.2-1.9; about 1.56 fits now)."},
    "change_cm": {"type": "number", "description": "Or a change from now, centimetres (+ higher)."},
}
VR_RESET_DESCRIPTION = ("Resets your virtual headset, like SteamVR's reset: stops walking and following, "
                        "connects the headset again, looks level ahead with the arms at rest, recenters. "
                        "For a view or body that looks stuck or wrong.")


def vr_reset_words(data: dict) -> str:
    recentered = {True: " (recentering asked of Monado)", False: " (recentering failed)", None: ""}[data.get("recentered")]
    return (f"Headset reset{recentered}: looking level ahead, arms at rest, "
            f"standing {data.get('head_height', 0):.2f} m high.")


KIND_WORDS = {
    "open": "open floor",
    "frontier": "edge of what you have seen",
    "platform": "a raised top to jump onto",
    "player": "player",
}


def survey_words(data: dict) -> str:
    """A survey's places and players in words for the model."""
    lines = []
    for c in data.get("candidates", []):
        what = KIND_WORDS.get(c["kind"], c["kind"])
        if c["kind"] == "player":
            what = f"{c['name']}" + (f" (friend, whitelist #{c['whitelist_rank']})" if c.get("whitelist_rank") else "")
        elif c["kind"] == "platform" and c.get("rise_m") is not None:
            what += f" ({c['rise_m']:.1f} m up)"
        walk = f", walk {c['walk_m']:.1f} m" if c.get("walk_m") is not None else ", no path seen"
        lines.append(f"{c['id']}: {what}, {c['distance_m']:.1f} m at {c['bearing_deg']:+.0f} deg{walk}")
    places = "; ".join(lines) if lines else "none (look around again, or step back)"
    room = data.get("room") or []
    seen = {p["name"] for p in data.get("players", [])}
    unseen = [n for n in room if n not in seen]
    others = f" In the room but not in sight: {', '.join(unseen)}." if unseen else ""
    return f"Numbered places (bearing from where you face, + right): {places}.{others}"


def goto_words(data: dict) -> str:
    legs = data.get("legs", [])
    blocked = sum(1 for leg in legs if leg.get("blocked"))
    bumps = f", bumped into something {blocked} time(s)" if blocked else ""
    if data.get("arrived"):
        return f"You are there ({data['remaining_m']:.1f} m off, {data['took_s']:.0f} s{bumps})."
    return (f"You stopped {data['remaining_m']:.1f} m short ({data.get('reason') or 'stuck'}"
            f"{bumps}).")


STEP_DESCRIPTION = (
    "Small precise moves, not for getting somewhere (that is vrchat_walk_to): turn by degrees, "
    "then walk a few metres a way (measured: it stops there, or where something stops it), or "
    "jump. Says how far you went.")
STEP_PARAMS = {
    "turn": {"type": "number", "description": "Degrees to turn first, + right, - left (180 around)."},
    "direction": {"type": "string", "enum": ["forward", "back", "left", "right"],
                  "description": "Which way to walk (of where you face after the turn)."},
    "meters": {"type": "number", "description": "How far to walk, 0-5 (0: no walk)."},
    "jump": {"type": "boolean", "description": "Jump as you start (in place without meters)."},
}


def step_words(result: dict) -> str:
    done = f"Turned {result['turned']:+.0f} degrees. " if result.get("turned") else ""
    moved = result.get("moved") or {}
    ahead, right = moved.get("ahead_m", 0), moved.get("right_m", 0)
    parts = [p for p in (f"{abs(ahead)} m {'ahead' if ahead > 0 else 'back'}" if ahead else "",
                         f"{abs(right)} m {'right' if right > 0 else 'left'}" if right else "") if p]
    if parts:
        done += f"You went {' and '.join(parts)} (of where you faced)."
    if result.get("stopped"):
        done += " You were told to stop."
    elif result.get("blocked"):
        done += " Something stopped you."
    return done or "Done."


def vr_goto_body(a: dict) -> dict:
    if a.get("place") is not None:
        if int(a["place"]) < 1:
            raise RuntimeError("places are numbered from 1")
        return {"candidate": int(a["place"])}
    if a.get("bearing") is not None:
        return {"bearing": float(a["bearing"]), "distance": float(a.get("distance") or 2.0)}
    raise RuntimeError("give a place number, or a bearing")


# When the voice model speaks, for an AstrBot without ``group_rule``.
OLD_RULE = """The one rule that matters most: speak ONLY when the speaker says your name{aliases} to you in that utterance, or is directly continuing an exchange with you from a few seconds ago. In every other case produce no audio and no text at all - complete silence. Do not acknowledge, do not react, do not say "mm", do not comment, do not delegate."""


def room_prompt(name: str, aliases: list[str], tools: bool) -> str:
    """The room's prompt. ``tools``: the voice model runs the room's tools
    (the local_infra backend), whose voice server passes on only what calls
    the bot by name (everything to one other player)."""
    try:
        from astrbot.core.voice.session import VoiceOptions, group_rule
    except ImportError:
        group_rule = None
    if group_rule is not None:
        rule = group_rule(VoiceOptions(name=name, aliases=aliases), gated=tools)
    else:
        others = [a for a in aliases if a and a != name]
        alias_text = (
            f' ("{name}"' + "".join(f', "{a}"' for a in others) + ")"
            if others
            else f' "{name}"'
        )
        rule = OLD_RULE.format(aliases=alias_text)
    prompt = ROOM_PROMPT.format(name=name, rule=rule)
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
        # Timing logs (where a reply's delay goes): the room's speech as the
        # plugin gets it, and the bot's speech as it sends it.
        self._talk = SpeechDetector()
        self._talk_loud_at = 0.0
        self._talking = False
        self._sent_at = 0.0
        self._preroll: deque[tuple[float, bytes]] = deque()
        self._retry_at = 0.0
        self._http: aiohttp.ClientSession | None = None
        self._ws: aiohttp.ClientWebSocketResponse | None = None
        self._out: asyncio.Queue[bytes] = asyncio.Queue(maxsize=OUT_QUEUE_FRAMES)
        self._tasks: set[asyncio.Task] = set()
        # The room context last given to the voice session (room_context).
        self._room_context = ""
        self._moving = asyncio.Lock()  # MOVING_PATHS one at a time
        # The look around each caller ("voice", "text") last saw: its place
        # numbers are what it walks to.
        self._seen: dict[str, int] = {}
        self._running = True

    def meta(self) -> PlatformMetadata:
        return self.metadata

    @property
    def room_umo(self) -> str:
        return f"{self.meta().id}:{MessageType.GROUP_MESSAGE.value}:{ROOM_SESSION}"

    # -- bridge -----------------------------------------------------------

    def _headers(self) -> dict:
        return {"Authorization": f"Bearer {self.token}"}

    async def vr_survey(self, players: bool = True, who: str = "voice") -> tuple[dict, bytes, bytes]:
        """Looks all around: the numbered places and players, the numbered
        panorama (JPEG) and map (PNG), all of the same look around."""
        async with self._moving:
            data = await self._call("POST", "/v1/vr/survey", {"players": players}, timeout=60)
            self._seen[who] = data.get("survey")
            return data, *await self._survey_pictures()

    async def vr_goto(self, body: dict, who: str = "voice") -> tuple[dict, bytes, bytes]:
        """Walks to a place of the last look around ``who`` saw, or by
        bearing and distance; how it went, and the new look around's
        pictures."""
        async with self._moving:
            if "candidate" in body and self._seen.get(who) is not None:
                body = {**body, "survey": self._seen[who]}
            data = await self._call("POST", "/v1/vr/goto", body, timeout=150)
            self._seen[who] = data["after"].get("survey")
            return data, *await self._survey_pictures()

    async def _survey_pictures(self) -> tuple[bytes, bytes]:
        return (await self._call("GET", "/v1/vr/survey/pano.jpg"),
                await self._call("GET", "/v1/vr/survey/map.png"))

    def _in_turn(self, method: str, path: str) -> asyncio.Lock | contextlib.nullcontext:
        """What a call to ``path`` waits for: the moves before it finished
        (looking needs none, and a reset, like a stop, cuts in)."""
        moves = method != "GET" and path.startswith(MOVING_PATHS) and path != "/v1/vr/reset"
        return self._moving if moves else contextlib.nullcontext()

    async def request(self, method: str, path: str, body: dict | None = None, timeout: float = 60) -> Any:
        """Calls the bridge's HTTP API; returns its JSON (or bytes for images)."""
        async with self._in_turn(method, path):
            return await self._call(method, path, body, timeout)

    async def _call(self, method: str, path: str, body: dict | None = None, timeout: float = 60) -> Any:
        if self._http is None:
            raise RuntimeError("VRChat bridge 未连接")
        async with self._http.request(
            method,
            self.base_url + path,
            json=body,
            headers=self._headers(),
            timeout=aiohttp.ClientTimeout(total=timeout),
        ) as resp:
            if resp.status < 300 and resp.content_type.startswith("image/"):
                return await resp.read()
            try:
                data = await resp.json(content_type=None)
            except ValueError:
                data = None
            if resp.status >= 300:
                error = data.get("error") if isinstance(data, dict) else None
                raise RuntimeError(error or f"HTTP {resp.status}")
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
        now = time.monotonic()
        if now - self._sent_at > SPEECH_GAP_SECONDS:
            logger.info("VRChat timing: the bot's speech starts (to the bridge)")
        self._sent_at = now
        if self._out.full():  # the bridge fell behind: drop the oldest audio
            with contextlib.suppress(asyncio.QueueEmpty):
                self._out.get_nowait()
        self._out.put_nowait(chunk)

    def _on_text(self, data: dict) -> None:
        kind = data.get("type")
        if kind == "state":
            old = {p["id"] for p in self.state.get("players", [])}
            had_world = bool(self.state.get("running") and self.state.get("world"))
            self.state = data.get("state") or {}
            new = {p["id"]: p["name"] for p in self.state.get("players", [])}
            joined = [name for pid, name in new.items() if pid not in old]
            if joined:
                logger.info("VRChat: %s joined %s", ", ".join(joined), self.state.get("world"))
            self._refresh_context()
            if set(new) != old or bool(self.state.get("running") and self.state.get("world")) != had_world:
                self._people_changed()
        elif kind in ("alert", "auth_required", "join_failed"):
            logger.warning("VRChat bridge: %s", data)
        elif kind in ("invite", "request_invite", "joining", "follow"):
            logger.info("VRChat bridge: %s", data)

    def _people_changed(self) -> None:
        """Players came or went: the voice session hears accordingly (all
        that one other player says, else only what calls the bot)."""
        session = self.session
        if session is not None and hasattr(session, "set_people"):
            # Not in a room yet (or between rooms): unknown, wake words.
            known = self.state.get("running") and self.state.get("world")
            task = asyncio.get_running_loop().create_task(
                session.set_people(len(self.players()) if known else None)
            )
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)

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

    async def follow(self, name: str = "", stop: bool = False,
                     distance: float | None = None) -> dict:
        """Follows a player in the room by their name tag (by default the
        highest-priority whitelisted one here), standing ``distance`` metres
        away (default 1.5), or stops following."""
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

    async def step(self, turn: float = 0.0, direction: str = "forward", meters: float = 0.0,
                   jump: bool = False) -> dict:
        """Turns by degrees (+ right), walks a few measured metres a way, or jumps."""
        return await self.request("POST", "/v1/step", {
            "turn": float(turn), "direction": direction, "meters": float(meters), "jump": bool(jump)})

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

        def picture(text: str, jpeg: bytes) -> list[dict]:
            data = base64.b64encode(jpeg).decode()
            return [{"type": "inputText", "text": text},
                    {"type": "inputImage", "imageUrl": f"data:image/jpeg;base64,{data}"}]

        def pictures(text: str, pano: bytes, top: bytes) -> list[dict]:
            return [{"type": "inputText", "text": text},
                    {"type": "inputImage", "imageUrl": "data:image/jpeg;base64," + base64.b64encode(pano).decode()},
                    {"type": "inputImage", "imageUrl": "data:image/png;base64," + base64.b64encode(top).decode()}]

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

        async def look_around(a: dict) -> list[dict]:
            data, pano, top = await self.vr_survey(bool(a.get("players", True)))
            return pictures(survey_words(data), pano, top)

        async def walk_to(a: dict) -> list[dict]:
            data, pano, top = await self.vr_goto(vr_goto_body(a))
            return pictures(goto_words(data) + " " + survey_words(data["after"]), pano, top)

        async def step(a: dict) -> str:
            result = await self.step(a.get("turn") or 0, str(a.get("direction") or "forward"),
                                     a.get("meters") or 0, bool(a.get("jump")))
            return step_words(result)

        async def last_seen(a: dict) -> list[dict] | str:
            found = await self.last_seen(str(a.get("name") or ""))
            if found is None:
                return "You have not seen a friend of your whitelist yet."
            sighting, jpeg = found
            return picture(f"Your view when you last saw {sighting['name']}: "
                           f"{sighting['age_s']:.0f} s ago, in {sighting['world'] or 'an unknown world'}.",
                           jpeg)

        async def height(a: dict) -> str:
            body = {}
            if a.get("metres") is not None:
                body["metres"] = float(a["metres"])
            elif a.get("change_cm") is not None:
                body["change_cm"] = float(a["change_cm"])
            if not body:
                data = await self.request("GET", "/v1/vr/height")
                return f"Your headset stands {data['head_height']:.2f} m above the floor."
            data = await self.request("POST", "/v1/vr/height", body)
            return f"Headset height {data['was']:.2f} m -> {data['head_height']:.2f} m."

        async def vr_reset(a: dict) -> str:
            data = await self.request("POST", "/v1/vr/reset")
            return vr_reset_words(data)

        actions = [
            (spec("vrchat_emote", "Plays a gesture of your avatar.", {
                "name": {"type": "string", "enum": list(EMOTES)}}, ["name"]), emote),
            (spec("vrchat_jump", "Jumps once.", {}, []), jump),
            (spec("vrchat_stop", "Stops walking, turning and following at once.", {}, []), stop),
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
            (spec("vrchat_who", "Lists the other players in the room.", {}, []), who),
            (spec("vrchat_look_around", VR_LOOK_DESCRIPTION, {
                "players": {"type": "boolean",
                            "description": "Read name tags to find players (default true; "
                                           "false is a little faster)."}}, []), look_around),
            (spec("vrchat_walk_to", VR_WALK_DESCRIPTION, VR_WALK_PARAMS, []), walk_to),
            (spec("vrchat_step", STEP_DESCRIPTION, STEP_PARAMS, []), step),
            (spec("vrchat_height", HEIGHT_DESCRIPTION, HEIGHT_PARAMS, []), height),
            (spec("vrchat_vr_reset", VR_RESET_DESCRIPTION, {}, []), vr_reset),
            (spec("vrchat_last_seen", "Shows your view when you last saw a friend of your whitelist, "
                  "how long ago and in which world.", {
                      "name": {"type": "string",
                               "description": "Their display name; leave out for whoever was seen last."}},
                  []), last_seen),
        ]
        # No action ends the turn by itself: the model sees how it went and
        # answers in its own words.
        return [VoiceTool(s, run) for s, run in actions]

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
            self._time_speech(pcm)
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

    def _time_speech(self, pcm: bytes) -> None:
        """Logs when the room's speech starts and ends as the plugin gets it
        (an energy VAD, like standby's): set against the voice server's
        turn records (GET /v1/inferences?kind=voice.turn), what came before."""
        now = time.monotonic()
        loud = self._talk.feed(pcm)
        if loud:
            self._talk_loud_at = now
            if not self._talking:
                self._talking = True
                logger.info("VRChat timing: room speech starts")
        elif self._talking and now - self._talk_loud_at > SPEECH_GAP_SECONDS:
            self._talking = False
            logger.info("VRChat timing: room speech ended %.1f s ago", now - self._talk_loud_at)

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
        # Before the launch: the session starts with it.
        self._people_changed()
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
