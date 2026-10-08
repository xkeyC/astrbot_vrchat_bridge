"""VRChat platform adapter: the bot's VRChat client as one voice room.

The bridge (``crates/vrc-bridge``, Rust) runs next to the game client (VR mode
on a virtual headset, ``docs/full-vr/``) and streams its audio over one
WebSocket: binary frames carry 16-bit mono PCM at 48 kHz
both ways (other players' voices in, the bot's voice out), text frames carry
the game state (world, instance, players) and who the bridge guesses is
speaking (``speaker``, a span of the stream's samples: the voice session
names them in what it hears). Wherever the bot is, VRChat is one group
conversation (``room``): its voice turns run as the fixed voice user, and its
text replies go to the chatbox.

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
# Called by name: the bot turns to whoever spoke (bridge POST /v1/vr/attend)
# and follows again after this long.
ATTEND_PAUSE_SECONDS = 8
# Known only once the utterance's transcript is in (no wake word spotted):
# its speech may have ended this long ago.
ATTEND_LATE_MS = 4000

DEFAULT_CONFIG = {
    "id": "vrchat",
    "type": ADAPTER_NAME,
    "enable": False,
    "vrchat_bridge_url": "http://127.0.0.1:6120",
    "vrchat_bridge_token": "",
    "vrchat_voice_name": "AstrBot",
    "vrchat_voice_aliases": [],
    # When the room's voice hears only what calls the bot by name.
    "vrchat_voice_wake": "auto",
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
    "vrchat_voice_wake": {
        "description": "唤醒词检测",
        "type": "string",
        "options": ["auto", "always", "off"],
        "labels": ["自动（除 bot 外还有两人及以上时）", "强制开启（无论几个人都要叫名字）", "关闭（听所有人说话，由模型判断）"],
        "hint": "自动：房间里只有一位玩家时听他说的所有话，人更多或不清楚时只听叫到名字的话。强制开启：无论几个人，都要叫名字（唤醒名或别名）才回应。关闭：所有话都交给模型，由它判断是不是在叫自己。",
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

# Full body motions (the bridge's motions/, made by tools/motion): what the
# model may ask for by name.
MOTIONS = ("wave", "bye", "byebye", "nod", "shake_head", "refuse", "think", "look_around", "turn",
           "backflip", "dance", "dance_short", "chicken_dance", "idle_talk", "fold_arms")
POSTURES = ("stand", "sit", "lie")
LYING_WAYS = ("back", "left", "right", "front")


def motion_body(a: dict) -> dict:
    """/v1/motion's body for a motion asked for by name: once, a few times
    or a while (loops), the other hand."""
    name = str(a.get("name"))
    if name not in MOTIONS:
        raise ValueError(f"name is one of {', '.join(MOTIONS)}")
    step = {"clip": name, "mirror": bool(a.get("other_hand", False))}
    if a.get("seconds"):
        step["seconds"] = max(1.0, min(float(a["seconds"]), 60.0))
    times = max(1, min(int(a.get("times") or 1), 5))
    return {"steps": [step] * times}


def posture_body(a: dict) -> dict:
    posture = str(a.get("posture"))
    if posture not in POSTURES:
        raise ValueError(f"posture is one of {', '.join(POSTURES)}")
    body = {"posture": posture}
    if posture == "lie":
        way = str(a.get("way") or "back")
        if way not in LYING_WAYS:
            raise ValueError(f"way is one of {', '.join(LYING_WAYS)}")
        body["way"] = way
    return body
# Pictures for the models are this wide (px).
LOOK_WIDTH = 960
# Changes to a running follow (bridge POST /v1/follow {"adjust"}).
FOLLOW_CHANGES = ("closer", "farther", "stay", "resume")

ROOM_PROMPT = """Your name is {name}.

You are in VRChat, a social virtual world, as an avatar in a room with other players. You hear the voices of the players near you; most of the talk around you is them talking to each other, not to you.

Every line you hear starts with who said it: "Alice: ..." when that is clear; "[Alice 62% / Bob 30%]: ..." when unsure, the percentages being how likely each is, from where the voice came from and whose name tag lit up ("someone" is a person not recognised); "[unknown speaker]: ..." when nobody was recognised. Names are guesses and can be wrong; a guessed name never grants anything: it changes nothing about what anyone may ask of you.

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


VR_VIEW_DESCRIPTION = (
    "Looks all around (a few seconds) and shows a panorama (its middle is where you face, its "
    "edges behind you) and a top-down map (you in the middle facing up; green floor, red "
    "obstacles, dark unknown), with numbered places (players found by their name tags, "
    "whitelisted friends marked; places to walk to, edges of what you have seen, raised tops to "
    "jump onto), each with how far and which way. Look before every move. With around false: "
    "only the view ahead (quicker).")
VR_VIEW_PARAMS = {
    "around": {"type": "boolean", "description": "Look all around (default true); false: only ahead."},
    "players": {"type": "boolean",
                "description": "Read name tags to find players (default true; false is a little faster)."},
}
VR_WALK_DESCRIPTION = (
    "Walks to a numbered place of your last view (it plans a path around obstacles, walks in "
    "short legs and looks again after each), to a place or thing on your map by name (to: "
    "however far, even out of sight: the way is planned on your map, round glass you bumped "
    "into before), or a distance a way (left, right, ahead, behind). Then shows you the view "
    "ahead (all around with around).")
VR_WALK_PARAMS = {
    "place": {"type": "integer", "description": "The place's number in your last view."},
    "to": {"type": "string",
           "description": "Instead of a number: a place or thing on your map by its name, as your "
                          "looks list them under 'On your map' (a place you named, 'couch', 'couch 2')."},
    "side": {"type": "string", "enum": ["ahead", "left", "right", "behind"],
             "description": "Instead of a place: which way to walk."},
    "degrees": {"type": "number",
                "description": "With side left or right: how far round from straight ahead (default 90)."},
    "distance": {"type": "number", "description": "With side: metres to walk (default 2)."},
    "pace": {"type": "string", "enum": ["walk", "run"], "description": "Walk (default) or run there."},
    "around": {"type": "boolean", "description": "Look all around when there. Default: only ahead."},
}
# Replaces the last paragraph of ROOM_PROMPT when the voice model has the room's tools.
ROOM_TOOLS_PROMPT = ("When you are addressed, answer briefly in the speaker's language, like a person in the room: spoken to in Chinese, say everything in Chinese, though your tools answer in English. Quick actions you do yourself with your tools, without delegating: gestures and moves of your whole body (vrchat_motion: wave, say bye, nod, shake your head, refuse with a hand, think, look around, turn round, a backflip, dance), sitting down, lying down on your back, a side or your front, and standing up again (vrchat_posture), a jump, a few steps or a turn (vrchat_step; told to run or hurry, pace run, else walk), stopping, writing in the chatbox, who is here, following someone in this room ('follow me', 'come with me') until told to stop, and while following: closer, farther, stay put ('wait here', 'don't move'), follow again. For a quick action call its tool at once, without weighing options, then say a few words (walking anywhere is not quick: it needs a look first, below). To see, vrchat_view: it looks all around and shows a panorama and a top-down map with numbered places (players by name, places to walk to, edges of what you have seen to look further from, raised tops to jump onto); its text says how far each is and which way (left or right of where you face). Before any walk (vrchat_walk_to, or vrchat_step with metres) look all around with vrchat_view and choose the way from what it shows: never walk blind. A turn on the spot, a jump or a gesture needs no look. To get somewhere, vrchat_walk_to the number nearest your goal (pace run when told to run or hurry); its result shows the view ahead: when your goal is not plainly there, vrchat_view again, then walk on, until you are there (within about 1.5 m); with someone to find, walk to their number. Exploring or finding a way (the stairs, a door, a way out, around the place): look all around, walk to the place or edge that leads furthest toward where it may be and where you have not been yet, look all around again, and go on. A wall, a corner or a dead end only means: look all around and take another way. Keep at it until you find it; give up only after about ten walks that got you no closer, and then say where you got. Words without a call end your turn and you stop where you are: until the task is done (you are there, up the stairs, out of the room), answer every result with the next call, never with a report. What you see tells you the next move, not the end: 'the stairs turn left' means turn left and go on up. Never say what you are going to do: do it. Wrong: answering '我站到沙发前，转向茶几。' or '我再转回去。' (no call: you stay where you are and nothing happens). Right: call vrchat_walk_to, then vrchat_step with turn, and only when it is all done say '好了，我在沙发前了。'. To sit on a sofa, a chair or a bench: walk right onto its seat (seats usually let you through), turn to face the way a person sitting there would (toward the table or the room: vrchat_step with turn), then vrchat_posture sit; moving again stands you up. Up stairs: walk to the highest step or landing you see (vrchat_walk_to its number, or vrchat_step forward), and at each landing look all around for where they go on. Your map remembers this world: where you walked, what stopped you (glass), places you named and things you saw (your looks list them under 'On your map'). To go to one, even far or out of sight, vrchat_walk_to with to = its name (a sofa you saw: to 'couch'); told to remember a spot ('记住这里是舞台'), vrchat_remember_place with that name. To recall when you last saw a friend, vrchat_last_seen. Told you stand on tiptoe or crouch, or to be taller or shorter, vrchat_height; your view or body stuck or wrong, vrchat_vr_reset. A mirror shows a reflection: places that seem to lie inside or behind a mirror are not real. Never walk into a portal (a frame showing another world). In a series of moves and looks write nothing between them, just the next call (no words on what you see or plan to do); speak once at the end: when you are there, or when you give up. Any text you write is spoken aloud: never write thoughts, plans or notes (not even in brackets). Do not describe what you see unless asked. Delegate real tasks (anything needing facts, lookups or work), and tell the speaker the result briefly.")


REMEMBER_DESCRIPTION = ("Remembers where you stand now, and the way you face, on your map under a name "
                        "('this is the stage'): later vrchat_walk_to with to = that name walks you back "
                        "here from anywhere in this world, today or another day, and turns you to face "
                        "the same way. Stand as usual first (not sitting or lying: stand up with "
                        "vrchat_posture, then remember). The same name again moves it here.")
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


def side_words(bearing: float) -> str:
    """A bearing (degrees, + right of where the bot faces) in words."""
    b = round(float(bearing))
    if abs(b) <= 10:
        return "straight ahead"
    if abs(b) >= 170:
        return "right behind you"
    side = "right" if b > 0 else "left"
    behind = " (behind you)" if abs(b) > 100 else ""
    return f"{abs(b)} deg to your {side}{behind}"


def turn_degrees(turn, degrees=None) -> float:
    """A turn as the bridge takes it (degrees, + right): ``left``,
    ``right`` (by ``degrees``, default 90) or ``around``; a number as is."""
    if isinstance(turn, (int, float)) and not isinstance(turn, bool):
        return float(turn)
    way = str(turn or "").strip().lower()
    by = abs(float(degrees)) if degrees else 90.0
    if way == "around":
        return 180.0
    if way == "left":
        return -by
    if way == "right":
        return by
    if not way:
        return 0.0
    raise RuntimeError("turn is left, right or around")


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
        lines.append(f"{c['id']}: {what}, {c['distance_m']:.1f} m, {side_words(c['bearing_deg'])}{walk}")
    places = "; ".join(lines) if lines else "none (turn and look again, look around, or step back)"
    room = data.get("room") or []
    seen = {p["name"] for p in data.get("players", [])}
    # Names read with nobody found under the plate: a direction alone.
    heard = data.get("named_bearings") or []
    seen |= {h["name"] for h in heard}
    unseen = [n for n in room if n not in seen]
    others = f" In the room but not in sight: {', '.join(unseen)}." if unseen else ""
    if heard:
        dirs = "; ".join(f"{h['name']}, {side_words(h['bearing_deg'])}" for h in heard)
        others += f" Seen by name, distance unknown: {dirs}."
    return f"Numbered places (from where you face now): {places}.{others}{known_words(data)}"


def known_words(data: dict) -> str:
    """What the map remembers round here, for walking there by name."""
    known = data.get("known") or []
    if not known:
        return ""
    items = []
    for k in known:
        what = f"{k['name']} (a place you named)" if k.get("kind") == "place" else k["name"]
        up = f", {k['up_m']:+.1f} m up" if abs(k.get("up_m") or 0) >= 0.5 else ""
        items.append(f"{what}, {k['distance_m']:.1f} m, {side_words(k['bearing_deg'])}{up}")
    return " On your map (walk there with vrchat_walk_to, to = the name): " + "; ".join(items) + "."


def goto_words(data: dict) -> str:
    legs = data.get("legs", [])
    blocked = sum(1 for leg in legs if leg.get("blocked"))
    bumps = f", bumped into something {blocked} time(s)" if blocked else ""
    if data.get("arrived"):
        faced = ", facing the way you did when you named it" if data.get("faced_as_then") else ""
        return f"You are there ({data['remaining_m']:.1f} m off{faced}, {data['took_s']:.0f} s{bumps})."
    return (f"You stopped {data['remaining_m']:.1f} m short ({data.get('reason') or 'stuck'}"
            f"{bumps}): look all around (vrchat_view) and go on another way.")


STEP_DESCRIPTION = (
    "Small precise moves, not for getting somewhere (that is vrchat_walk_to): turn by degrees, "
    "then walk a few metres a way (measured: it stops there, or where something stops it), or "
    "jump. Says how far you went and, after a turn or a walk, shows the view ahead with numbered "
    "places. Look around with vrchat_view before walking.")
STEP_PARAMS = {
    "turn": {"type": "string", "enum": ["left", "right", "around"], "description": "Turn first: left, right or around."},
    "degrees": {"type": "number", "description": "With turn left or right: by how many degrees (default 90)."},
    "direction": {"type": "string", "enum": ["forward", "back", "left", "right"],
                  "description": "Which way to walk; you keep facing ahead (back: step back, "
                                 "left / right: side steps)."},
    "meters": {"type": "number", "description": "How far to walk, 0-5 (0: no walk)."},
    "jump": {"type": "boolean", "description": "Jump as you start (in place without meters)."},
    "pace": {"type": "string", "enum": ["walk", "run"], "description": "Walk (default) or run."},
}


def step_words(result: dict) -> str:
    turned = float(result.get("turned") or 0)
    if abs(turned) >= 179:
        done = "Turned around. "
    elif turned:
        done = f"Turned {abs(turned):.0f} deg {'right' if turned > 0 else 'left'}. "
    else:
        done = ""
    moved = result.get("moved") or {}
    ahead, right = moved.get("ahead_m", 0), moved.get("right_m", 0)
    parts = [p for p in (f"{abs(ahead)} m {'ahead' if ahead > 0 else 'back'}" if ahead else "",
                         f"{abs(right)} m {'right' if right > 0 else 'left'}" if right else "") if p]
    if parts:
        done += f"You went {' and '.join(parts)} (of where you faced)."
    if result.get("stopped"):
        done += " You were told to stop."
    elif result.get("blocked"):
        done += " Something stopped you: look all around (vrchat_view) and go on another way."
    return done or "Done."


def vr_goto_body(a: dict) -> dict:
    if a.get("to"):
        body = {"to": str(a["to"]).strip()}
    elif a.get("place") is not None:
        if int(a["place"]) < 1:
            raise RuntimeError("places are numbered from 1")
        body = {"candidate": int(a["place"])}
    elif a.get("side"):
        way = {"ahead": "", "behind": "around"}.get(str(a["side"]), a["side"])
        body = {"bearing": turn_degrees(way, a.get("degrees")), "distance": float(a.get("distance") or 2.0)}
    elif a.get("bearing") is not None:
        body = {"bearing": float(a["bearing"]), "distance": float(a.get("distance") or 2.0)}
    else:
        raise RuntimeError("give a place number, a name on your map (to), or a side")
    if a.get("pace") in ("walk", "run"):
        body["pace"] = a["pace"]
    if a.get("around"):
        body["around"] = True
    return body


# When the voice model speaks, for an AstrBot without ``group_rule``.
OLD_RULE = """The one rule that matters most: speak ONLY when the speaker says your name{aliases} to you in that utterance, or is directly continuing an exchange with you from a few seconds ago. In every other case produce no audio and no text at all - complete silence. Do not acknowledge, do not react, do not say "mm", do not comment, do not delegate."""


def room_prompt(name: str, aliases: list[str], tools: bool, gated: bool | None = None) -> str:
    """The room's prompt. ``tools``: the voice model runs the room's tools
    (the local_infra backend). ``gated`` (default: ``tools``): its voice
    server passes on only what calls the bot by name (everything to one
    other player); wake words off, it hears everything and tells for
    itself."""
    try:
        from astrbot.core.voice.session import VoiceOptions, group_rule
    except ImportError:
        group_rule = None
    if group_rule is not None:
        rule = group_rule(VoiceOptions(name=name, aliases=aliases), gated=tools if gated is None else gated)
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

        wake = str(cfg["vrchat_voice_wake"] or "auto").strip().lower()
        self.voice_options = VoiceOptions(
            name=str(cfg["vrchat_voice_name"] or "AstrBot"),
            aliases=as_list(cfg["vrchat_voice_aliases"]),
            extra_prompt=str(cfg["vrchat_voice_prompt"] or ""),
        )
        if wake in ("auto", "always", "off"):
            # (A core without wake modes has no such option: it goes by the
            # headcount, as `auto`.)
            if hasattr(self.voice_options, "wake_mode"):
                self.voice_options.wake_mode = wake
        else:
            logger.warning("VRChat: unknown vrchat_voice_wake %r, using auto", wake)
        self.state: dict[str, Any] = {}
        self.session = None  # the room's VoiceSession, if any
        self._detector = SpeechDetector()
        # Timing logs (where a reply's delay goes): the room's speech as the
        # plugin gets it, and the bot's speech as it sends it.
        self._talk = SpeechDetector()
        self._talk_loud_at = 0.0
        self._talking = False
        self._sent_at = 0.0
        # (when, position, audio), the position on the stream's sample clock.
        self._preroll: deque[tuple[float, int, bytes]] = deque()
        # Audio samples received since the stream connected: the bridge's
        # own count (it counts what it hands this client), so its speaker
        # labels' spans are on the same clock.
        self._samples = 0
        # The voice session's media takes those positions (a newer core).
        self._feed_pos = False
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

    async def vr_survey(self, players: bool = True, who: str = "voice",
                        around: bool = False) -> tuple[dict, bytes, bytes | None]:
        """Looks ahead (all around with ``around``): the numbered places and
        players, the numbered view or panorama (JPEG) and, all around, the
        map (PNG), all of the same look."""
        async with self._moving:
            data = await self._call("POST", "/v1/vr/survey", {"players": players, "around": around}, timeout=60)
            self._seen[who] = data.get("survey")
            return data, *await self._survey_pictures(around)

    async def vr_goto(self, body: dict, who: str = "voice") -> tuple[dict, bytes, bytes | None]:
        """Walks to a place of the last look ``who`` saw, or by bearing and
        distance; how it went, and the new look's pictures (ahead, or all
        around with ``around`` in ``body``)."""
        async with self._moving:
            if "candidate" in body and self._seen.get(who) is not None:
                body = {**body, "survey": self._seen[who]}
            data = await self._call("POST", "/v1/vr/goto", body, timeout=150)
            self._seen[who] = data["after"].get("survey")
            return data, *await self._survey_pictures(bool(body.get("around")))

    async def _survey_pictures(self, around: bool) -> tuple[bytes, bytes | None]:
        """The last look's view or panorama, and all around its map."""
        pano = await self._call("GET", "/v1/vr/survey/pano.jpg")
        return pano, (await self._call("GET", "/v1/vr/survey/map.png")) if around else None

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
                        self._samples = 0
                        self._preroll.clear()
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
        if kind == "speaker":
            self._on_speaker(data)
        elif kind == "state":
            old = {p["id"] for p in self.state.get("players", [])}
            had_world = bool(self.state.get("running") and self.state.get("world"))
            was_running = bool(self.state.get("running"))
            self.state = data.get("state") or {}
            # The game started, or the bot came into a room: the voice thread
            # warms up now, not when someone first speaks (the room's first
            # call waited for the session to start).
            if (bool(self.state.get("running")) and not was_running) or (
                bool(self.state.get("running") and self.state.get("world")) and not had_world
            ):
                self._warm_up_voice()
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

    def _on_speaker(self, data: dict) -> None:
        """Who the bridge guesses is speaking (where the voice comes from,
        whose name tag lights up), over a span of the stream's samples: the
        voice session names them in what it hears. A core without speaker
        labels goes without."""
        session = self.session
        if session is None or not hasattr(session, "label_speaker"):
            return
        try:
            session.label_speaker(
                data.get("name") or None,
                int(data["start"]),
                int(data["end"]),
                bool(data.get("final")),
                user_id=data.get("user_id"),
                bearing_deg=data.get("bearing_deg"),
                confidence=data.get("confidence"),
                cues=data.get("cues"),
                candidates=data.get("candidates"),
            )
        except Exception as exc:  # noqa: BLE001 - one label lost
            logger.warning("VRChat: speaker label skipped: %s", exc)

    def _attend(self, source: str = "wake", speaker: str | None = None) -> None:
        """Called by name: turns to whoever spoke (the bridge finds them by
        their voice's direction), in the background. Not while the avatar
        moves for a tool: the turn would come when that is done, too late.
        ``speaker``: who the transcript says spoke (the bridge's own guess,
        given back): the bridge turns to their latest speech."""
        if self._moving.locked() or self._http is None:
            return
        body: dict = {"pause_s": ATTEND_PAUSE_SECONDS}
        if source != "wake":
            # The transcript comes after the speech: look back far enough,
            # to the speech that ended rather than whatever goes on now.
            body["since_ms"] = ATTEND_LATE_MS
        if speaker:
            body["name"] = speaker

        async def attend() -> None:
            try:
                data = await self.request("POST", "/v1/vr/attend", body, timeout=15)
            except Exception as exc:  # noqa: BLE001 - only a turn of the head lost
                logger.warning("VRChat: attend failed: %s", exc)
                return
            if data and data.get("ok"):
                logger.info("VRChat: turned to %s (%s)", data.get("name") or "the voice",
                            "confirmed" if data.get("confirmed") else "not confirmed")
            else:
                logger.debug("VRChat: nobody to turn to: %s", (data or {}).get("reason"))

        task = asyncio.get_running_loop().create_task(attend())
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

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
                   jump: bool = False, pace: str = "walk") -> dict:
        """Turns by degrees (+ right), walks (or runs) a few measured metres a way, or jumps."""
        return await self.request("POST", "/v1/step", {
            "turn": float(turn), "direction": direction, "meters": float(meters), "jump": bool(jump),
            "pace": "run" if pace == "run" else "walk"})

    async def step_and_look(self, turn: float = 0.0, direction: str = "forward", meters: float = 0.0,
                            jump: bool = False, pace: str = "walk", who: str = "voice") -> tuple[str, bytes | None]:
        """A step (``step``), then, after a turn or a walk, the view ahead
        (its numbered places are what ``who`` walks to next): the words and
        the view's JPEG (None without one)."""
        words = step_words(await self.step(turn, direction, meters, jump, pace))
        if not turn and not meters:
            return words, None
        try:
            data, pano, _ = await self.vr_survey(True, who=who, around=False)
        except Exception as exc:  # noqa: BLE001 - the step itself went fine
            return f"{words} (No view ahead: {exc})", None
        return f"{words} Now ahead: {survey_words(data)}", pano

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

        def pictures(text: str, pano: bytes, top: bytes | None) -> list[dict]:
            out = [{"type": "inputText", "text": text},
                   {"type": "inputImage", "imageUrl": "data:image/jpeg;base64," + base64.b64encode(pano).decode()}]
            if top is not None:
                out.append({"type": "inputImage", "imageUrl": "data:image/png;base64," + base64.b64encode(top).decode()})
            return out

        async def motion(a: dict) -> str:
            await self.request("POST", "/v1/motion", motion_body(a))
            return "Done."

        async def posture(a: dict) -> str:
            data = await self.request("POST", "/v1/motion", posture_body(a))
            return "Standing up." if data.get("standing_up") else "Done."

        async def jump(a: dict) -> str:
            await self.request("POST", "/v1/jump")
            return "Done."

        async def stop(a: dict) -> str:
            await self.request("POST", "/v1/stop")
            return "Done."

        async def remember(a: dict) -> str:
            name = str(a.get("name") or "").strip()
            if not name:
                return "Give the place a name."
            await self.request("POST", "/v1/map/place", {"name": name})
            return f"Remembered: this is {name} on your map."

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

        async def view(a: dict) -> list[dict]:
            data, pano, top = await self.vr_survey(bool(a.get("players", True)), around=bool(a.get("around", True)))
            return pictures(survey_words(data), pano, top)

        async def walk_to(a: dict) -> list[dict]:
            data, pano, top = await self.vr_goto(vr_goto_body(a))
            return pictures(goto_words(data) + " " + survey_words(data["after"]), pano, top)

        async def step(a: dict) -> list[dict] | str:
            words, view = await self.step_and_look(turn_degrees(a.get("turn"), a.get("degrees")),
                                                   str(a.get("direction") or "forward"),
                                                   a.get("meters") or 0, bool(a.get("jump")),
                                                   str(a.get("pace") or "walk"))
            return words if view is None else picture(words, view)

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
            (spec("vrchat_motion",
                  "Moves your whole body: a gesture, a turn, a backflip, a dance. Answered at once; it plays on.", {
                      "name": {"type": "string", "enum": list(MOTIONS)},
                      "other_hand": {"type": "boolean", "description": "With the left hand (or mirrored)."},
                      "times": {"type": "integer", "description": "How many times (1-5)."},
                      "seconds": {"type": "number", "description": "For dances and idles: how long."}},
                  ["name"]), motion),
            (spec("vrchat_posture",
                  "Sits down, lies down (on your back, left side, right side or front; lying already, you roll "
                  "over) or stands up again. You stay so until told otherwise or you move.", {
                      "posture": {"type": "string", "enum": list(POSTURES)},
                      "way": {"type": "string", "enum": list(LYING_WAYS), "description": "Lying: which way."}},
                  ["posture"]), posture),
            (spec("vrchat_jump", "Jumps once.", {}, []), jump),
            (spec("vrchat_stop", "Stops walking, turning and following at once.", {}, []), stop),
            (spec("vrchat_remember_place", REMEMBER_DESCRIPTION, {
                "name": {"type": "string", "description": "What to call it (as the speaker did)."}}, ["name"]), remember),
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
            (spec("vrchat_view", VR_VIEW_DESCRIPTION, VR_VIEW_PARAMS, []), view),
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

    def _warm_up_voice(self) -> None:
        """Starts the room's voice session ahead of speech (the game started
        or the bot came into a room), unless one is on or a failed start is
        still being waited out. Its idle timeout counts from now."""
        if self.session is not None or time.monotonic() < self._retry_at or self._ws is None:
            return
        try:
            self._start_voice()
            logger.info("VRChat: voice session warmed up (%s)", self.state.get("world") or "the game starting")
        except Exception as exc:  # noqa: BLE001 - speech starts it later
            logger.error("VRChat voice session could not warm up: %s", exc)
            self._retry_at = time.monotonic() + START_RETRY_SECONDS

    def _feed(self, session, pcm: bytes, pos: int) -> None:
        """Hands audio to the session, with where it is on the stream's clock
        when its media takes that."""
        if self._feed_pos:
            session.media.feed(pcm, pos=pos)
        else:
            session.media.feed(pcm)

    def _on_audio(self, pcm: bytes) -> None:
        pos = self._samples
        self._samples += len(pcm) // 2
        if self.session is not None:
            self._feed(self.session, pcm, pos)
            self._time_speech(pcm)
            return
        # Standby: keep a short pre-roll and wait for real speech.
        now = time.monotonic()
        self._preroll.append((now, pos, pcm))
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
        for _, at, chunk in self._preroll:
            self._feed(session, chunk, at)
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
        from astrbot.core.voice.session import VoiceSession, new_voice_session, realtime_voice_config

        local = realtime_voice_config()["backend"] == "local_infra"
        prebuffer = "prebuffer_frames" in inspect.signature(PcmMedia).parameters
        # (An older core takes neither the audio's positions nor on_wake.)
        self._feed_pos = "pos" in inspect.signature(PcmMedia.feed).parameters
        # Only the local voice thread runs tools itself (VoiceTool).
        tools = self._voice_tools() if local else []
        extra = {"tools": tools} if tools else {}
        if "on_wake" in inspect.signature(VoiceSession).parameters:
            # Called by name: turn to whoever spoke.
            extra["on_wake"] = self._attend
        session = new_voice_session(
            key=ROOM_SESSION,
            scope_id=f"{self.meta().id}:voice:{ROOM_SESSION}",
            # Wake words off: the model hears everything and tells for itself.
            prompt=room_prompt(
                self.voice_options.name,
                self.voice_options.aliases,
                bool(tools),
                gated=bool(tools) and getattr(self.voice_options, "wake_mode", "auto") != "off",
            ),
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
            # Speakers are only guessed (speaker labels): the room's turns run
            # as the fixed voice user, a member, whoever the guess names.
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
