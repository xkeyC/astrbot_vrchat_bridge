#!/usr/bin/env python3
"""VRChat bridge: the game client's audio, inputs and state over HTTP/WebSocket.

Runs as the bot's Linux user next to the VRChat desktop client (Proton) and
is the only thing AstrBot talks to:

* ``/v1/stream`` (WebSocket, one client at a time): binary frames carry audio,
  16-bit mono PCM at 48 kHz, both ways. Inbound is what the game plays
  (``vrc_out``'s monitor: other players' voices). Outbound is the bot's
  voice, played into ``vrc_mic_in`` (looped back to ``vrc_mic``, VRChat's
  microphone) while push-to-talk is held. Text frames carry the state.
* HTTP: chatbox, movement, turning, jumping, looking up/down, screenshots,
  starting/stopping the game, status.

Movement and the chatbox go through VRChat's OSC input (UDP 9000); looking
up/down has no OSC input and uses relative mouse motion on the X display.
State (world, instance, players) is read from the client's output log.
"""

from __future__ import annotations

import argparse
import asyncio
import urllib.parse
import contextlib
import glob
import hmac
import json
import logging
import math
import os
import tempfile
import re
import shutil
import socket
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path

import aiohttp
from aiohttp import WSMsgType, web

import drive
import nav
import odometry
from follow import FOCAL_PX, Follower, axis_speed
from sightings import Sightings
from social import Social
from vrc_api import instance_kind, joinable, launch_location, launch_url, login_main

log = logging.getLogger("vrc-bridge")

SAMPLE_RATE = 48000
FRAME_BYTES = 960 * 2  # 20 ms of 16-bit mono
# Push-to-talk stays held this long after the bot's audio has played out,
# so the end of a sentence is not cut by the release.
PTT_TAIL = 0.3
# Latency of the playback stream into vrc_mic_in.
PLAYBACK_LATENCY_MS = 60
# Inbound audio frames queued for a slow client before the oldest are dropped.
INBOUND_QUEUE_FRAMES = 100
CHATBOX_LIMIT = 144
# VRChat throttles chatbox spam; messages are paced at least this far apart.
CHATBOX_INTERVAL = 1.6
MAX_MOVE_SECONDS = 10.0
# A takeover of following ends by itself after this long without a move or a
# look of the model's: following resumes (only if it was following before).
TAKEOVER_IDLE_S = 10.0
# The jump forward of a goto with climb (onto a table, a seat).
CLIMB_MS = 700
# vrchat_step: walks at STEP_AXIS (STEP_AXIS_SHORT under a metre: precise)
# until the odometry is within STEP_SLIDE_S of sliding there; less than
# STEP_PROGRESS_M on in STEP_STUCK_S is something in the way.
STEP_AXES = {"forward": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, -1.0),
             "right": (0.0, 1.0)}
STEP_MAX_M = 5.0
STEP_AXIS = 0.5
STEP_AXIS_SHORT = 0.35
STEP_SLIDE_S = 0.1
STEP_PROGRESS_M = 0.05
STEP_STUCK_S = 0.6
# The HUD's F5 badge (x, y, w, h on the 1280x720 frame) is lit green with
# the third person camera on (HUD_GREEN_PX of its pixels or more); after F5
# the HUD is polled for up to CAMERA_WAIT_S for the change.
HUD_F5_BOX = (1194, 94, 66, 42)
HUD_GREEN_PX = 120
CAMERA_SETTLE_S = 0.3
# vrchat_camera_y: shots CAMERA_Y_SHOT_DEG up and down from where the view
# looks (together about 60 degrees each way), lines every CAMERA_Y_LINE_DEG.
CAMERA_Y_SHOT_DEG = 30
CAMERA_Y_LINE_DEG = 10
CAMERA_Y_SETTLE_S = 0.35
CAMERA_WAIT_S = 2.5
CAMERA_POLL_S = 0.25
CLIMB_AXIS = 0.6  # forward while jumping: onto a small seat, not over it
# Before the jump, on slowly (axis; ~0.65 m/s) until pressed against the
# thing: VelocityZ under CLOSE_STUCK_MPS once past CLOSE_SETTLE_S.
CLOSE_AXIS = 0.3
CLOSE_MAX_S = 2.0
CLOSE_SETTLE_S = 0.35
CLOSE_STUCK_MPS = 0.15
CLOSE_UP_M = 0.2  # risen this much while closing in: stepped up onto it, no jump
MAX_TURN_SECONDS = 5.0
LOOK_STEP_PX = 25
WATCHDOG_INTERVAL = 5.0
STEAM_APP_ID = "438100"

LOG_DIR = (
    "~/.local/share/Steam/steamapps/compatdata/438100/pfx/drive_c/users/"
    "steamuser/AppData/LocalLow/VRChat/VRChat"
)
RE_AUTH = re.compile(r"User Authenticated: (.+) \((usr_[0-9a-f-]+)\)")
RE_ENTERING = re.compile(r"\[Behaviour\] Entering Room: (.+)$")
RE_JOINING = re.compile(r"\[Behaviour\] Joining (wrld_\S+)")
RE_JOINED = re.compile(r"\[Behaviour\] OnPlayerJoined (.+) \((usr_[0-9a-f-]+)\)")
RE_LEFT = re.compile(r"\[Behaviour\] OnPlayerLeft (.+) \((usr_[0-9a-f-]+)\)")
RE_LEFT_ROOM = re.compile(r"\[Behaviour\] OnLeftRoom")
RE_OSCQUERY = re.compile(r"of type OSCQuery on (\d+)")
# VRCEmote values of VRChat's default emote menu.
EMOTES = {"wave": 1, "clap": 2, "point": 3, "cheer": 4, "dance": 5, "backflip": 6,
          "sadness": 7, "die": 8}


# -- OSC ---------------------------------------------------------------------


def _pad(data: bytes) -> bytes:
    return data + b"\0" * (4 - len(data) % 4)


def osc_message(address: str, *args) -> bytes:
    """Encodes an OSC message (int, float, bool and str arguments)."""
    tags, payload = ",", b""
    for arg in args:
        if isinstance(arg, bool):
            tags += "T" if arg else "F"
        elif isinstance(arg, int):
            tags += "i"
            payload += struct.pack(">i", arg)
        elif isinstance(arg, float):
            tags += "f"
            payload += struct.pack(">f", arg)
        else:
            tags += "s"
            payload += _pad(str(arg).encode())
    return _pad(address.encode()) + _pad(tags.encode()) + payload


class Osc:
    def __init__(self, port: int) -> None:
        self._addr = ("127.0.0.1", port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, address: str, *args) -> None:
        try:
            self._sock.sendto(osc_message(address, *args), self._addr)
        except OSError as exc:
            log.warning("OSC %s failed: %s", address, exc)


# -- game state from the output log ------------------------------------------


@dataclass
class GameState:
    running: bool = False
    self_name: str = ""
    self_id: str = ""
    world_name: str = ""
    instance: str = ""  # wrld_...:id~...
    players: dict[str, str] = field(default_factory=dict)  # usr_id -> name
    oscquery_port: int = 0
    vram_mib: int = 0

    def snapshot(self) -> dict:
        others = {k: v for k, v in self.players.items() if k != self.self_id}
        return {
            "running": self.running,
            "self": {"name": self.self_name, "id": self.self_id},
            "world": self.world_name,
            "instance": self.instance,
            "players": [{"id": k, "name": v} for k, v in others.items()],
            "vram_mib": self.vram_mib,
        }


class LogTail:
    """Follows the newest VRChat output log and keeps ``GameState`` current."""

    def __init__(self, directory: Path, state: GameState, on_change) -> None:
        self.directory = directory
        self.state = state
        self._on_change = on_change
        self._path: str | None = None
        self._pos = 0
        self._partial = ""

    def _newest(self) -> str | None:
        files = glob.glob(str(self.directory / "output_log_*.txt"))
        return max(files, key=os.path.getmtime) if files else None

    def _apply(self, line: str) -> bool:
        s = self.state
        if m := RE_JOINED.search(line):
            s.players[m.group(2)] = m.group(1)
        elif m := RE_LEFT.search(line):
            s.players.pop(m.group(2), None)
        elif m := RE_ENTERING.search(line):
            s.world_name, s.players = m.group(1).strip(), {}
        elif m := RE_JOINING.search(line):
            s.instance = m.group(1)
        elif RE_LEFT_ROOM.search(line):
            s.world_name, s.instance, s.players = "", "", {}
        elif m := RE_AUTH.search(line):
            s.self_name, s.self_id = m.group(1), m.group(2)
        elif m := RE_OSCQUERY.search(line):
            s.oscquery_port = int(m.group(1))
            return False
        else:
            return False
        return True

    async def run(self) -> None:
        while True:
            try:
                await self._poll()
            except Exception:  # noqa: BLE001 - keep following
                log.exception("log tail failed")
            await asyncio.sleep(1.0)

    async def _poll(self) -> None:
        newest = self._newest()
        if newest != self._path:
            # A new client run: its log starts from scratch.
            self._path, self._pos, self._partial = newest, 0, ""
            s = self.state
            s.world_name, s.instance, s.players, s.oscquery_port = "", "", {}, 0
        if not self._path:
            return
        with open(self._path, encoding="utf-8", errors="replace") as f:
            f.seek(self._pos)
            chunk = f.read()
            self._pos = f.tell()
        if not chunk:
            return
        lines = (self._partial + chunk).split("\n")
        self._partial = lines.pop()
        changed = False
        for line in lines:
            changed |= self._apply(line)
        if changed:
            await self._on_change()


# -- the bridge ----------------------------------------------------------------


class Bridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.token = Path(args.token_file).expanduser().read_text(encoding="utf-8").strip()
        if len(self.token) < 16:
            raise SystemExit("token file must hold a token of 16+ characters")
        self.osc = Osc(args.osc_port)
        self.state = GameState()
        self.logtail = LogTail(Path(args.log_dir).expanduser(), self.state, self._state_changed)
        config_dir = Path(args.token_file).expanduser().parent
        self.social = Social(self, config_dir / "social.json", config_dir / "cookies.json")
        ocr_token = ""
        if args.ocr_token_file:
            ocr_token = Path(args.ocr_token_file).expanduser().read_text(encoding="utf-8").strip()
        # Leaving an instance the bot must not be in (``_guard_instance``).
        self._leaving = False
        # One game stop/start at a time (joins, /v1/game/start).
        self.game_lock = asyncio.Lock()
        # The model driving the avatar itself (/v1/drive): the script under
        # way, and the following it paused ({"target", "distance"}), which
        # /v1/autopilot hands back.
        self.driver = drive.Driver(self.osc, self._mouse)
        self._drive_task: asyncio.Task | None = None
        self.takeover: dict | None = None
        self._resume: asyncio.TimerHandle | None = None
        # Numbered places to walk to on the view (/v1/nav, /v1/goto).
        self.navigator = nav.Navigator(self, args.depth_url, args.depth_model, FOCAL_PX,
                                       axis_speed)
        self._background: set[asyncio.Task] = set()
        self.follower = Follower(self, args.ocr_url, args.ocr_model, ocr_token,
                                 args.depth_url, args.depth_model)
        self.sightings = Sightings(self)
        self.ws: web.WebSocketResponse | None = None
        self._inbound: asyncio.Queue[bytes] = asyncio.Queue(maxsize=INBOUND_QUEUE_FRAMES)
        self._player: asyncio.subprocess.Process | None = None
        self._ptt_held = False
        self._speech_until = 0.0  # when the bot's queued audio has played out
        self._ptt_task: asyncio.Task | None = None
        self._move_task: asyncio.Task | None = None
        self._turn_task: asyncio.Task | None = None
        self._chatbox: asyncio.Queue[tuple[str, bool]] = asyncio.Queue(maxsize=20)
        self._env = dict(os.environ, DISPLAY=args.display)

    # -- audio --------------------------------------------------------------

    async def _capture(self) -> None:
        """Reads what the game plays and hands it to the client, if any."""
        cmd = ["parec", f"--device={self.args.capture_device}", "--format=s16le",
               f"--rate={SAMPLE_RATE}", "--channels=1", "--latency-msec=20", "--raw"]
        while True:
            try:
                await self._capture_once(cmd)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - retried
                log.warning("capture failed: %s", exc)
            log.warning("capture ended, restarting")
            await asyncio.sleep(1.0)

    async def _capture_once(self, cmd: list[str]) -> None:
        proc = await asyncio.create_subprocess_exec(*cmd, stdout=asyncio.subprocess.PIPE)
        log.info("capturing %s", self.args.capture_device)
        try:
            assert proc.stdout is not None
            while chunk := await proc.stdout.readexactly(FRAME_BYTES):
                if self.ws is None:
                    continue
                if self._inbound.full():  # the client fell behind
                    self._inbound.get_nowait()
                self._inbound.put_nowait(chunk)
        except asyncio.IncompleteReadError:
            pass
        finally:
            with contextlib.suppress(ProcessLookupError):
                proc.kill()
            await proc.wait()

    async def _play(self, pcm: bytes) -> None:
        """Plays the bot's voice into the microphone, holding push-to-talk."""
        if self._player is None or self._player.returncode is not None:
            self._player = await asyncio.create_subprocess_exec(
                "pacat", "--playback", f"--device={self.args.playback_device}",
                "--format=s16le", f"--rate={SAMPLE_RATE}", "--channels=1",
                f"--latency-msec={PLAYBACK_LATENCY_MS}", "--raw",
                stdin=asyncio.subprocess.PIPE,
            )
        if not self._ptt_held:
            self.osc.send("/input/Voice", 1)
            self._ptt_held = True
        now = time.monotonic()
        self._speech_until = max(self._speech_until, now) + len(pcm) / (SAMPLE_RATE * 2)
        if self._ptt_task is None or self._ptt_task.done():
            self._ptt_task = asyncio.create_task(self._release_ptt())
        assert self._player.stdin is not None
        try:
            self._player.stdin.write(pcm)
            await self._player.stdin.drain()
        except (BrokenPipeError, ConnectionResetError):
            self._player = None

    async def _release_ptt(self) -> None:
        tail = PTT_TAIL + PLAYBACK_LATENCY_MS / 1000
        while (wait := self._speech_until + tail - time.monotonic()) > 0:
            await asyncio.sleep(wait)
        self.osc.send("/input/Voice", 0)
        self._ptt_held = False

    # -- state --------------------------------------------------------------

    def room_state(self) -> dict:
        """The game state, each other player marked as a friend (VRChat's
        friend list) and by their whitelist rank (1 first, 0 not on it), and
        whom the avatar follows (None when nobody)."""
        snap = self.state.snapshot()
        ranks = {uid: rank for rank, uid in enumerate(self.social.whitelist_ids(), 1)}
        for player in snap["players"]:
            player["friend"] = player["id"] in self.social.friends
            player["whitelist"] = ranks.get(player["id"], 0)
        follower = self.follower
        snap["follow"] = (None if follower.state == "idle"
                          else {"target": follower.target, "state": follower.state})
        snap["takeover"] = self.takeover
        return snap

    async def _state_changed(self) -> None:
        await self._guard_instance()
        await self.send_event({"type": "state", "state": self.room_state()})

    async def _guard_instance(self) -> None:
        """Leaves an instance the bot must not be in (public, group, ...):
        only joins it starts itself are checked, yet a portal or a followed
        player can take it anywhere."""
        s = self.state
        if not (s.running and s.instance) or joinable(s.instance) or self._leaving:
            return
        self._leaving = True
        log.error("in a %s instance (%s): leaving", instance_kind(s.instance) or "unknown",
                  s.instance)
        self.follower.stop()
        await self.send_event({"type": "alert", "reason": "not_joinable_instance",
                               "kind": instance_kind(s.instance) or "unknown"})

        async def leave() -> None:
            try:
                await self.stop_game()
            finally:
                self._leaving = False

        self._spawn(leave())

    def _spawn(self, coro) -> asyncio.Task:
        """A background task kept until done (an unreferenced one may vanish)."""
        task = asyncio.create_task(coro)
        self._background.add(task)
        task.add_done_callback(self._background.discard)
        return task

    def notify_state(self) -> None:
        """Pushes the room state soon (from code that cannot wait)."""
        with contextlib.suppress(RuntimeError):
            self._spawn(self._state_changed())

    async def send_event(self, data: dict) -> None:
        ws = self.ws
        if ws is not None and not ws.closed:
            with contextlib.suppress(ConnectionResetError, RuntimeError):
                await ws.send_str(json.dumps(data, ensure_ascii=False))
        if data.get("type") == "follow":
            self.notify_state()  # whom it follows, and how, is room state

    async def _game_pid(self) -> int | None:
        # By process name: Proton's wrappers carry VRChat.exe in their arguments too.
        proc = await asyncio.create_subprocess_exec(
            "pgrep", "-u", str(os.getuid()), "-x", "VRChat.exe", stdout=asyncio.subprocess.PIPE)
        out, _ = await proc.communicate()
        pids = [int(p) for p in out.split()]
        return min(pids) if pids else None

    async def _watchdog(self) -> None:
        """Tracks whether the game runs and its VRAM; stops it above the hard limit."""
        first = True
        while True:
            # The first look at once: actions wait for it after a restart.
            if not first:
                await asyncio.sleep(WATCHDOG_INTERVAL)
            first = False
            try:
                pid = await self._game_pid()
                running = pid is not None
                vram, total = 0, 0
                if running and shutil.which("nvidia-smi"):
                    vram, total = await self._vram(pid)
                changed = running != self.state.running
                self.state.running, self.state.vram_mib = running, vram
                if not running and (self.state.instance or self.state.players
                                    or self.state.world_name):
                    # No game, no instance: not even one an old log replayed
                    # (a join compares against it, the guard checks it as
                    # soon as a new game runs).
                    self.state.world_name, self.state.instance, self.state.players = "", "", {}
                    changed = True  # the clients' room state is stale too
                if changed and not running:
                    # No game: nothing to read (OCR) or steer (mouse, OSC).
                    self.follower.stop()
                    for task in (self._move_task, self._turn_task):
                        if task is not None:
                            task.cancel()
                if changed:
                    await self._state_changed()
                if running and (vram > self.args.vram_hard_mib or total > self.args.gpu_hard_mib):
                    log.error("VRAM over the limit (game %d MiB, GPU %d MiB): stopping the game",
                              vram, total)
                    await self.send_event({"type": "alert", "reason": "vram", "game_mib": vram,
                                           "gpu_mib": total, "action": "stopped"})
                    await self.stop_game()
                elif running and vram > self.args.vram_soft_mib:
                    await self.send_event({"type": "alert", "reason": "vram", "game_mib": vram,
                                           "gpu_mib": total, "action": "none"})
            except Exception:  # noqa: BLE001 - keep watching
                log.exception("watchdog failed")

    async def _vram(self, pid: int) -> tuple[int, int]:
        proc = await asyncio.create_subprocess_exec(
            "nvidia-smi", "--query-compute-apps=pid,used_memory",
            "--format=csv,noheader,nounits", stdout=asyncio.subprocess.PIPE)
        out, _ = await proc.communicate()
        game = 0
        for line in out.decode().splitlines():
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == 2 and parts[0] == str(pid) and parts[1].isdigit():
                game = int(parts[1])
        proc = await asyncio.create_subprocess_exec(
            "nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits",
            stdout=asyncio.subprocess.PIPE)
        out, _ = await proc.communicate()
        total = int(out.split()[0]) if out.split() else 0
        return game, total

    # -- inputs -------------------------------------------------------------

    async def _chatbox_sender(self) -> None:
        while True:
            text, notify = await self._chatbox.get()
            self.osc.send("/chatbox/input", text, True, notify)
            await asyncio.sleep(CHATBOX_INTERVAL)

    def chatbox(self, text: str, notify: bool) -> int:
        """Queues ``text`` for the chatbox, split into 144-character parts."""
        parts = split_chatbox(text)
        for part in parts:
            if self._chatbox.full():
                self._chatbox.get_nowait()
            self._chatbox.put_nowait((part, notify))
        return len(parts)

    async def move(self, forward: float, right: float, seconds: float, run: bool) -> None:
        if self._move_task is not None:
            self._move_task.cancel()
        self._move_task = asyncio.create_task(self._hold_axes(
            [("/input/Vertical", forward), ("/input/Horizontal", right)],
            seconds, run))

    async def turn(self, speed: float, seconds: float) -> None:
        if self._turn_task is not None:
            self._turn_task.cancel()
        self._turn_task = asyncio.create_task(
            self._hold_axes([("/input/LookHorizontal", speed)], seconds, False))

    async def _hold_axes(self, axes: list[tuple[str, float]], seconds: float, run: bool) -> None:
        try:
            if run:
                self.osc.send("/input/Run", 1)
            for address, value in axes:
                self.osc.send(address, float(value))
            await asyncio.sleep(seconds)
        finally:
            for address, _ in axes:
                self.osc.send(address, 0.0)
            if run:
                self.osc.send("/input/Run", 0)

    async def jump(self) -> None:
        self.osc.send("/input/Jump", 1)
        await asyncio.sleep(0.1)
        self.osc.send("/input/Jump", 0)

    async def emote(self, value: int, seconds: float) -> None:
        """Plays an emote of the avatar's emote menu (``VRCEmote``), then resets it.

        Raises:
            RuntimeError: The current avatar has no writable ``VRCEmote``.
        """
        port = self.state.oscquery_port
        if not port:
            raise RuntimeError("the game is not running")
        async with aiohttp.ClientSession() as http, http.get(
            f"http://127.0.0.1:{port}/avatar/parameters/VRCEmote",
            timeout=aiohttp.ClientTimeout(total=3),
        ) as resp:
            node = await resp.json(content_type=None) if resp.status == 200 else {}
        if not node or int(node.get("ACCESS", 0)) & 2 == 0:
            raise RuntimeError("the current avatar has no emotes")
        self.osc.send("/avatar/parameters/VRCEmote", int(value))
        await asyncio.sleep(seconds)
        self.osc.send("/avatar/parameters/VRCEmote", 0)

    def require_game(self) -> None:
        """Refuses an action while the game is not running: the mouse would
        move the desktop's pointer instead."""
        if not self.state.running:
            raise RuntimeError("VRChat is not running")

    async def _mouse(self, dx: int, dy: int) -> None:
        await self._run("xdotool", "mousemove_relative", "--", str(dx), str(dy))
        if dy:
            self.navigator.looked(dy)

    async def look(self, dy: int = 0, dx: int = 0) -> None:
        """Turns the view by relative mouse motion, in steps of at most
        LOOK_STEP_PX: negative ``dy`` looks up, negative ``dx`` left."""
        self.require_game()
        steps = max(abs(dx), abs(dy)) // LOOK_STEP_PX
        if steps == 0 and (dx or dy):
            steps = 1
        done_x = done_y = 0
        for i in range(1, steps + 1):
            x, y = dx * i // steps - done_x, dy * i // steps - done_y
            done_x, done_y = done_x + x, done_y + y
            await self._run("xdotool", "mousemove_relative", "--", str(x), str(y))
            await asyncio.sleep(0.03)
        if dy:
            self.navigator.looked(dy)

    async def screenshot(self, fmt: str, width: int = 0, grid: bool = False) -> bytes:
        """A frame of the game window (jpg or png), scaled to ``width`` px
        wide if given; ``grid`` marks the bearings along its top (degrees
        to turn to face that column, + right)."""
        codec = ["-c:v", "mjpeg", "-q:v", "4", "-f", "mjpeg"] if fmt == "jpg" else \
            ["-c:v", "png", "-f", "image2pipe"]
        filters = bearing_ruler(*(int(v) for v in self.args.screen.split("x"))) if grid else []
        if width:
            filters.append(f"scale={width}:-2")
        scale = ["-vf", ",".join(filters)] if filters else []
        proc = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", "-f", "x11grab", "-video_size",
            self.args.screen, "-i", self.args.display, "-frames:v", "1", *scale, *codec, "-",
            stdout=asyncio.subprocess.PIPE, env=self._env)
        out, _ = await proc.communicate()
        if proc.returncode != 0 or not out:
            raise RuntimeError("screenshot failed")
        return out

    async def scaled_jpeg(self, jpeg: bytes, width: int, filters: list[str] | None = None) -> bytes:
        """``jpeg`` re-encoded ``width`` px wide, ``filters`` (ffmpeg) drawn
        on it first."""
        chain = ",".join([*(filters or []), f"scale={width}:-2"])
        proc = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", "-f", "image2pipe", "-i", "-",
            "-vf", chain, "-c:v", "mjpeg", "-q:v", "4", "-f", "mjpeg", "-",
            env=self._env,
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE)
        out, _ = await proc.communicate(jpeg)
        if proc.returncode != 0 or not out:
            raise RuntimeError("scaling the image failed")
        return out

    async def _run(self, *cmd: str) -> int:
        proc = await asyncio.create_subprocess_exec(*cmd, env=self._env)
        return await proc.wait()

    # -- the game -----------------------------------------------------------

    async def start_game(self, url: str = "") -> None:
        """Starts Steam (if needed) and VRChat; a ``vrchat://launch`` URL picks the instance."""
        if await self._game_pid() is not None:
            raise RuntimeError("the game is already running")
        steam = await asyncio.create_subprocess_exec(
            "pgrep", "-u", str(os.getuid()), "-x", "steam", stdout=asyncio.subprocess.DEVNULL)
        if await steam.wait() != 0:
            await self._run("systemctl", "--user", "reset-failed", "vrc-steam")
            await self._run("systemd-run", "--user", "--unit=vrc-steam",
                            f"--setenv=DISPLAY={self.args.display}",
                            "--setenv=XAUTHORITY=" + os.path.expanduser("~/.Xauthority"),
                            "/usr/bin/steam", "-silent")
            await asyncio.sleep(25)  # login and IPC take a while
        args = [url] if url else []
        await self._run("systemd-run", "--user", "--quiet", "--collect",
                        f"--setenv=DISPLAY={self.args.display}",
                        "--setenv=XAUTHORITY=" + os.path.expanduser("~/.Xauthority"),
                        "/usr/bin/steam", "-applaunch", STEAM_APP_ID, *args)

    async def stop_game(self) -> None:
        await self._run("pkill", "-TERM", "-u", str(os.getuid()), "-f", "VRChat.exe")
        for _ in range(60):
            if await self._game_pid() is None:
                break
            await asyncio.sleep(1.0)

    # -- HTTP / WebSocket ----------------------------------------------------

    @web.middleware
    async def _auth(self, request: web.Request, handler):
        given = request.headers.get("Authorization", "").removeprefix("Bearer ").strip()
        if not hmac.compare_digest(given.encode(), self.token.encode()):
            return web.json_response({"error": "unauthorized"}, status=401)
        try:
            return await handler(request)
        except (ValueError, KeyError, TypeError) as exc:
            return web.json_response({"error": f"bad request: {exc}"}, status=400)
        except RuntimeError as exc:
            return web.json_response({"error": str(exc)}, status=409)

    async def h_stream(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=15, max_msg_size=1 << 20)
        await ws.prepare(request)
        if self.ws is not None and not self.ws.closed:
            await self.ws.close(message=b"replaced by a new client")
        self.ws = ws
        while not self._inbound.empty():
            self._inbound.get_nowait()
        log.info("stream client connected: %s", request.remote)
        sender = asyncio.create_task(self._send_audio(ws))
        try:
            await ws.send_str(json.dumps({"type": "state", "state": self.room_state()},
                                         ensure_ascii=False))
            async for msg in ws:
                if msg.type == WSMsgType.BINARY:
                    await self._play(msg.data)
        finally:
            sender.cancel()
            if self.ws is ws:
                self.ws = None
            log.info("stream client gone")
        return ws

    async def _send_audio(self, ws: web.WebSocketResponse) -> None:
        while not ws.closed:
            chunk = await self._inbound.get()
            try:
                await ws.send_bytes(chunk)
            except (ConnectionResetError, RuntimeError):
                return

    async def h_status(self, request: web.Request) -> web.Response:
        return web.json_response({**self.room_state(), "follow": self.follower.status()})

    async def h_chatbox(self, request: web.Request) -> web.Response:
        body = await request.json()
        parts = self.chatbox(str(body["text"]), bool(body.get("notify", False)))
        return web.json_response({"queued": parts})

    async def h_move(self, request: web.Request) -> web.Response:
        self.require_game()
        body = await request.json()
        seconds = clamp(float(body.get("seconds", 1.0)), 0.0, MAX_MOVE_SECONDS)
        await self.move(clamp(float(body.get("forward", 0)), -1, 1),
                        clamp(float(body.get("right", 0)), -1, 1),
                        seconds, bool(body.get("run", False)))
        return web.json_response({"ok": True, "seconds": seconds})

    async def h_turn(self, request: web.Request) -> web.Response:
        self.require_game()
        body = await request.json()
        seconds = clamp(float(body.get("seconds", 0.5)), 0.0, MAX_TURN_SECONDS)
        await self.turn(clamp(float(body["speed"]), -1, 1), seconds)
        return web.json_response({"ok": True, "seconds": seconds})

    async def h_stop(self, request: web.Request) -> web.Response:
        for task in (self._move_task, self._turn_task, self._drive_task):
            if task is not None:
                task.cancel()
        self.follower.stop()
        if self._resume is not None:
            self._resume.cancel()  # stopping means staying stopped
            self._resume = None
        if self.takeover is not None:
            self.takeover = None
            self.notify_state()
        return web.json_response({"ok": True})

    async def h_drive(self, request: web.Request) -> web.Response:
        """Runs a script of steps (``drive.parse``), pausing any following
        first (handed back by /v1/autopilot); answers when it is done."""
        self.require_game()
        steps = drive.parse((await request.json()).get("steps"))
        self._take_over()
        jumps = any(step.jump for step in steps)
        climb = asyncio.ensure_future(self._climb_while(self._drive_task_done)) if jumps else None
        walker = self.navigator
        start = (walker.x, walker.y, walker.heading)
        result = await self._driving(self.driver.run(steps))
        rose = await climb if climb is not None else None
        if result is None:
            return web.json_response({"ok": False, "stopped": True})
        # The pose, roughly: turns as asked, walks at their axis speeds.
        for step in steps:
            if step.turn_deg:
                self.navigator.turned(step.turn_deg)
            if step.forward or step.right:
                speed = axis_speed(1.0) * (2.0 if step.run else 1.0) * step.ms / 1000
                self.navigator.walked(step.forward * speed, step.right * speed)
        self._idle_later()
        answer = {"ok": True, "steps": len(steps), "ms": result}
        if rose is not None:
            answer["jump"] = rose
        if walker.odometry and any(step.forward or step.right for step in steps):
            # How far it really went (a wall stops it, the speed surprises),
            # ahead (+) and right (+) of where it faced at the start.
            await asyncio.sleep(0.2)  # the last of the motion
            dx, dy = walker.x - start[0], walker.y - start[1]
            h = math.radians(start[2])
            answer["moved"] = {"ahead_m": round(dx * math.cos(h) + dy * math.sin(h), 1),
                               "right_m": round(-dx * math.sin(h) + dy * math.cos(h), 1)}
        return web.json_response(answer)

    async def _climb_onto(self) -> None:
        """Up onto what is in front: on slowly until pressed against it (the
        walk there stops short of it; a low seat the avatar steps up onto by
        itself), then, if not up already, a jump forward."""
        if not await self._close_in():
            await self._hop()

    async def _close_in(self) -> bool:
        """Walks on slowly until the avatar no longer gets ahead (VelocityZ:
        pressed against the thing) or has gone up onto it (VelocityY added
        up), at most CLOSE_MAX_S. True when up on it; False otherwise, or
        when the avatar does not tell its velocity."""
        loop = asyncio.get_running_loop()
        start = last = loop.time()
        rise = 0.0
        try:
            self.osc.send("/input/Vertical", CLOSE_AXIS)
            async with aiohttp.ClientSession() as http:
                while loop.time() - start < CLOSE_MAX_S:
                    await asyncio.sleep(0.03)
                    try:
                        ahead, up = await asyncio.gather(self._avatar_param(http, "VelocityZ"),
                                                         self._avatar_param(http, "VelocityY"))
                        ahead, up = float(ahead), float(up)
                    except (aiohttp.ClientError, asyncio.TimeoutError, TypeError, ValueError):
                        return False
                    now = loop.time()
                    rise += up * (now - last)
                    last = now
                    if rise > CLOSE_UP_M:
                        return True
                    if now - start > CLOSE_SETTLE_S and ahead < CLOSE_STUCK_MPS:
                        return False
        finally:
            self.osc.send("/input/Vertical", 0.0)
        return False

    async def h_step(self, request: web.Request) -> web.Response:
        """A measured move: turns by ``turn`` degrees (+ right), then walks
        ``meters`` (at most STEP_MAX_M) ``direction`` (forward, back, left,
        right) until the avatar's own velocity says it got there, or it gets
        nowhere (something in the way); ``jump`` jumps as it starts (in place
        without meters). Answers how far it went, and how high it landed."""
        self.require_game()
        body = await request.json()
        direction = str(body.get("direction") or "forward")
        if direction not in STEP_AXES:
            raise ValueError(f"direction must be one of {', '.join(STEP_AXES)}")
        turn = clamp(float(body.get("turn") or 0), -180, 180)
        meters = clamp(float(body.get("meters") or 0), 0, STEP_MAX_M)
        jump = bool(body.get("jump"))
        self._take_over()
        climb = asyncio.ensure_future(self._climb_while(self._drive_task_done)) if jump else None
        result = await self._driving(self._step(turn, meters, STEP_AXES[direction], jump))
        rose = await climb if climb is not None else None
        self._idle_later()
        if result is None:
            return web.json_response({"ok": False, "stopped": True})
        if rose is not None:
            result["jump"] = rose
        return web.json_response(result)

    async def _step(self, turn: float, meters: float, axes: tuple[float, float],
                    jump: bool) -> dict:
        """Turns, then walks ``meters`` along ``axes`` (forward, right) by the
        odometry (by time without it), jumping first if asked."""
        walker = self.navigator
        if turn:
            await self.driver.run([drive.Step(turn_deg=turn)])
            walker.turned(turn)
        if jump:
            self.osc.send("/input/Jump", 1)
        loop = asyncio.get_running_loop()
        start = (walker.x, walker.y, walker.heading)
        axis = STEP_AXIS_SHORT if meters < 1.0 else STEP_AXIS
        speed = axis_speed(axis)
        blocked = False
        try:
            began = loop.time()
            progress = (began, 0.0)
            if meters:
                self.osc.send("/input/Vertical", axes[0] * axis)
                self.osc.send("/input/Horizontal", axes[1] * axis)
            else:
                await asyncio.sleep(0.15)  # a jump in place: the press
            while meters:
                await asyncio.sleep(0.05)
                now = loop.time()
                went = (math.hypot(walker.x - start[0], walker.y - start[1]) if walker.odometry
                        else speed * (now - began))
                if went >= meters - speed * STEP_SLIDE_S:
                    break  # it slides the rest
                if went - progress[1] > STEP_PROGRESS_M:
                    progress = (now, went)
                elif now - progress[0] > STEP_STUCK_S:
                    blocked = True
                    break
                if now - began > meters / speed * 2 + 1:
                    break  # slower than it should be: enough
        finally:
            self.osc.send("/input/Vertical", 0.0)
            self.osc.send("/input/Horizontal", 0.0)
            self.osc.send("/input/Jump", 0)
        if not walker.odometry and meters:
            walker.walked(axes[0] * meters, axes[1] * meters)
        await asyncio.sleep(0.2)  # the last of the motion
        dx, dy = walker.x - start[0], walker.y - start[1]
        h = math.radians(start[2])
        return {"ok": True, "turned": turn, "blocked": blocked,
                "moved": {"ahead_m": round(dx * math.cos(h) + dy * math.sin(h), 1),
                          "right_m": round(-dx * math.sin(h) + dy * math.cos(h), 1)}}

    async def _hop(self) -> None:
        """Jumps forward onto what is in front, letting go of forward as soon
        as it lands (Grounded after being in the air): on a small seat it
        stays on it instead of walking on over it."""
        loop = asyncio.get_running_loop()
        end = loop.time() + CLIMB_MS / 1000 * 2
        airborne = False
        try:
            self.osc.send("/input/Vertical", CLIMB_AXIS)
            self.osc.send("/input/Jump", 1)
            await asyncio.sleep(0.1)
            self.osc.send("/input/Jump", 0)
            async with aiohttp.ClientSession() as http:
                while loop.time() < end:
                    try:
                        grounded = await self._avatar_param(http, "Grounded")
                    except (aiohttp.ClientError, asyncio.TimeoutError):
                        await asyncio.sleep(CLIMB_MS / 1000)  # no way to tell: a plain hop
                        break
                    if not grounded:
                        airborne = True
                    elif airborne:
                        break  # landed: stop right there
                    await asyncio.sleep(0.02)
        finally:
            self.osc.send("/input/Vertical", 0.0)
            self.osc.send("/input/Jump", 0)

    def _drive_task_done(self) -> bool:
        return self._drive_task is None or self._drive_task.done()

    async def _avatar_param(self, http: aiohttp.ClientSession, name: str):
        async with http.get(
            f"http://127.0.0.1:{self.state.oscquery_port}/avatar/parameters/{name}",
            timeout=aiohttp.ClientTimeout(total=1),
        ) as resp:
            return (await resp.json(content_type=None)).get("VALUE", [None])[0]

    async def _climb_while(self, done) -> dict | None:
        """How much higher (or lower) the avatar is after a jump: its
        VelocityY (OSCQuery) integrated while the script runs and until it
        lands (Grounded), with how long it was in the air. None when the
        avatar does not expose them."""
        if not self.state.oscquery_port:
            return None
        loop = asyncio.get_running_loop()
        rise = airborne = 0.0
        landed_since = None
        started = last = loop.time()
        try:
            async with aiohttp.ClientSession() as http:
                await asyncio.sleep(0)  # let the script start
                while True:
                    vy, grounded = await asyncio.gather(
                        self._avatar_param(http, "VelocityY"),
                        self._avatar_param(http, "Grounded"))
                    now = loop.time()
                    dt, last = now - last, now
                    rise += float(vy or 0.0) * dt
                    if not grounded:
                        airborne += dt
                        landed_since = None
                    elif done():
                        landed_since = landed_since or now
                        if now - landed_since > 0.2:
                            break
                    if now - started > 12:  # never stuck measuring
                        break
                    await asyncio.sleep(0.03)
        except (aiohttp.ClientError, asyncio.TimeoutError, ValueError, TypeError):
            return None
        if abs(rise) > 5:
            # Not a jump's height: it fell off the world and respawned.
            return {"height_change_m": None, "fell": True, "airborne_s": round(airborne, 2)}
        return {"height_change_m": round(rise, 2), "airborne_s": round(airborne, 2)}

    def _take_over(self) -> None:
        """The model drives: following is paused, handed back by
        /v1/autopilot or by itself once the model is done (_idle_later)."""
        if self._resume is not None:
            self._resume.cancel()  # still at it
            self._resume = None
        if self.follower.state != "idle":
            self.takeover = {"target": self.follower.target,
                             "distance": self.follower.status()["distance"]}
            self.follower.stop()  # notifies the state, takeover included
        elif self.takeover is None:
            self.takeover = {"target": None, "distance": None}
            self.notify_state()

    def _idle_later(self) -> None:
        """After a move or a look: unless the model goes on within
        TAKEOVER_IDLE_S, following whom the takeover paused resumes."""
        if self._resume is not None:
            self._resume.cancel()
            self._resume = None
        if self.takeover and self.takeover.get("target"):
            self._resume = asyncio.get_running_loop().call_later(
                TAKEOVER_IDLE_S, lambda: self._spawn(self._resume_following()))

    async def _resume_following(self) -> None:
        self._resume = None
        takeover = self.takeover
        if not (takeover and takeover.get("target")) or self.follower.state != "idle":
            return
        if self._drive_task is not None and not self._drive_task.done():
            self._idle_later()  # a move is still under way
            return
        self.takeover = None
        if not self.state.running:
            self.notify_state()
            return
        log.info("takeover over: following %s again", takeover["target"])
        self.follower.start(takeover["target"], takeover.get("distance"))
        await self.send_event({"type": "follow", "state": "resumed",
                               "target": takeover["target"]})
        await self._state_changed()

    async def _driving(self, coro):
        """Runs a drive or a walk, one at a time; None when /v1/stop stopped it."""
        if self._drive_task is not None:
            self._drive_task.cancel()
        self._drive_task = asyncio.ensure_future(coro)
        try:
            return await self._drive_task
        except asyncio.CancelledError:
            if asyncio.current_task().cancelling():
                raise
            return None

    async def _hud_third(self, jpeg: bytes) -> bool:
        """Whether the game's HUD shows the third person camera on: its F5
        badge (top right) lit green."""
        x, y, w, h = HUD_F5_BOX
        proc = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", "-i", "-", "-vf", f"crop={w}:{h}:{x}:{y}",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-",
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE)
        raw, _ = await proc.communicate(jpeg)
        green = sum(1 for i in range(0, len(raw) - 2, 3)
                    if raw[i + 1] > 150 and raw[i + 1] > raw[i] + 60 and raw[i + 1] > raw[i + 2] + 30)
        return green >= HUD_GREEN_PX

    async def set_camera(self, third: bool) -> None:
        """Puts the game's camera in third person (from behind the avatar)
        or first (its eyes), F5 toggling between them, the HUD telling
        which it is."""
        now = await self._hud_third(await self.screenshot("jpg"))
        for _ in range(2):
            if now == third:
                break
            await self._run("xdotool", "key", "F5")
            # The HUD shows it a while after: wait for it, never pressing
            # again meanwhile (that would toggle it straight back).
            loop = asyncio.get_running_loop()
            until = loop.time() + CAMERA_WAIT_S
            while now != third and loop.time() < until:
                await asyncio.sleep(CAMERA_POLL_S)
                now = await self._hud_third(await self.screenshot("jpg"))
        if now == third:
            await asyncio.sleep(CAMERA_SETTLE_S)  # the view itself settles too
        self.navigator.camera = "third" if now else "first"
        self.navigator._missed = 0
        if now != third:
            log.warning("camera: wanted %s person, the HUD says otherwise",
                        "third" if third else "first")

    async def h_nav(self, request: web.Request) -> web.Response:
        """The view now, its numbered walkable places drawn (and listed in
        X-Nav-Marks) with the bearing ruler; ``camera=third`` for a look from
        behind the avatar (back to first person after)."""
        self.require_game()
        width = int(clamp(int(request.query.get("width", 960)), 320, 1920))
        screen_w, screen_h = (int(v) for v in self.args.screen.split("x"))
        third = request.query.get("camera") == "third"
        if request.query.get("pitch") != "keep":  # not right after a look up or down
            await self.set_camera(False)
            await self.navigator.level()
        await self.set_camera(third)
        try:
            jpeg = await self.navigator.view(width, bearing_ruler(screen_w, screen_h))
            camera = "third" if self.navigator.seen.me is not None else "first"
        finally:
            if third:
                await self.set_camera(False)
        marks = [{"n": m.number, "bearing": m.bearing, "m": m.distance, "been": m.been}
                 for m in self.navigator.marks]
        self._keep_for_debug("nav", jpeg, marks)
        if self.takeover is not None:
            self._take_over()  # looking is part of it: no resume meanwhile
            self._idle_later()
        return web.Response(body=jpeg, content_type="image/jpeg", headers={
            "X-Nav-Marks": json.dumps(marks), "X-Nav-Pose": self.navigator.summary(),
            "X-Nav-Camera": camera})

    async def h_look_around(self, request: web.Request) -> web.Response:
        """A full circle: four views with their walkable places, numbered
        across all (X-Nav-Marks, bearings from the heading now), 2x2;
        ``camera=third`` from behind the avatar (back to first after)."""
        self.require_game()
        self._take_over()
        screen_w, screen_h = (int(v) for v in self.args.screen.split("x"))
        third = request.query.get("camera") == "third"
        await self.set_camera(False)
        await self.navigator.level()
        await self.set_camera(third)
        try:
            jpeg = await self._driving(nav.look_around(
                self.navigator, self.driver, drive.Step, bearing_ruler(screen_w, screen_h)))
            camera = "third" if self.navigator.seen.me is not None else "first"
        finally:
            if third:
                await self.set_camera(False)
        if jpeg is None:
            raise RuntimeError("stopped")
        marks = [{"n": m.number, "bearing": m.bearing, "m": m.distance, "been": m.been}
                 for m in self.navigator.marks]
        self._keep_for_debug("around", jpeg, marks)
        self._idle_later()
        return web.Response(body=jpeg, content_type="image/jpeg", headers={
            "X-Nav-Marks": json.dumps(marks), "X-Nav-Pose": self.navigator.summary(),
            "X-Nav-Camera": camera})

    async def _goto_place(self, body: dict) -> web.Response:
        """Along the map's path to a landmark or a platform (climbing onto
        it if asked)."""
        self.require_game()
        walker = self.navigator
        if walker.scene is None:
            raise RuntimeError("no map yet: look around first")
        if body.get("landmark"):
            found = walker.scene.find(str(body["landmark"]))
            if found is None:
                raise ValueError(f"no landmark like {body['landmark']!r} on the map "
                                 "(name things with vrchat_note first)")
            name, x, y = found
        else:
            _, platforms = self._map_places()
            n = int(str(body["platform"]).upper().lstrip("P") or 0)
            if not 1 <= n <= len(platforms):
                raise ValueError(f"no platform P{n} on the map now: look at the map again")
            p = platforms[n - 1]
            name, x, y = f"P{n}", p.x, p.y
        self._take_over()
        result = await self._driving(walker.follow_path((x, y), self.driver, drive.Step))
        if result is None:
            return web.json_response({"ok": False, "stopped": "stopped"})
        result["to"] = name
        if body.get("climb") and result["left_m"] < 2.0:
            bearing, _ = walker.toward((x, y))
            await walker._face(bearing, self.driver, drive.Step)
            climb = asyncio.ensure_future(self._climb_while(self._drive_task_done))
            await self._driving(self._climb_onto())
            result["jump"] = await climb
        self._idle_later()
        return web.json_response(result)

    def _keep_for_debug(self, name: str, jpeg: bytes, marks: list) -> None:
        """The last picture of each kind the model was shown, with its
        marks, in ~/.cache/vrc-bridge (to see what it saw)."""
        folder = Path("~/.cache/vrc-bridge").expanduser()
        with contextlib.suppress(OSError):
            folder.mkdir(parents=True, exist_ok=True)
            (folder / f"last_{name}.jpg").write_bytes(jpeg)
            (folder / f"last_{name}.json").write_text(json.dumps(
                {"at": time.time(), "marks": marks, "pose": self.navigator.summary()}))

    def _map_places(self) -> tuple[list[dict], list]:
        """The map's landmarks and platforms, with their bearing (degrees,
        + right) and distance (m) from the avatar; platforms numbered nearest
        first (kept for goto)."""
        walker = self.navigator
        places = []
        if walker.scene is None:
            return places, []
        for name, (x, y, _) in walker.scene.landmarks.items():
            bearing, far = walker.toward((x, y))
            places.append({"kind": "landmark", "name": name, "bearing": round(bearing),
                           "m": round(far, 1), "x": x, "y": y})
        platforms = sorted(walker.scene.platforms(), key=lambda p: walker.toward((p.x, p.y))[1])
        platforms = [p for p in platforms if walker.toward((p.x, p.y))[1] < 15][:9]
        for n, p in enumerate(platforms, 1):
            bearing, far = walker.toward((p.x, p.y))
            places.append({"kind": "platform", "name": f"P{n}", "height_m": p.height,
                           "bearing": round(bearing), "m": round(far, 1), "x": p.x, "y": p.y})
        return places, platforms

    async def h_map(self, request: web.Request) -> web.Response:
        """The scene map around the avatar (it facing up), landmarks and
        platforms labelled (X-Map-Places)."""
        walker = self.navigator
        if walker.scene is None:
            raise RuntimeError("no map yet: look around first")
        span = clamp(float(request.query.get("span", 16)), 6, 40)
        width = int(clamp(int(request.query.get("width", 480)), 240, 960))
        pose = (walker.x, walker.y, walker.heading)
        image = walker.scene.render(pose, walker.visited, span, width)
        places, _ = self._map_places()
        filters = []
        for place in places:
            col, row = walker.scene.to_pixel(place["x"], place["y"], pose, span, width)
            if not (0 <= col < width and 0 <= row < width):
                continue
            color = "lime" if place["kind"] == "landmark" else "yellow"
            text = place["name"].replace("'", "").replace(":", " ")
            filters.append(f"drawbox=x={col - 4}:y={row - 4}:w=8:h=8:color={color}:t=fill")
            filters.append(f"drawtext=font=monospace:text='{text}':x={col}-tw/2:y={row - 26}:"
                           f"fontsize=18:fontcolor={color}:borderw=2:bordercolor=black")
        filters.append(f"drawtext=font=monospace:text='{span:.0f} m across, you face up':"
                       "x=8:y=h-24:fontsize=16:fontcolor=white:borderw=2:bordercolor=black")
        proc = await asyncio.create_subprocess_exec(
            "ffmpeg", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{width}x{width}", "-i", "-", "-vf", ",".join(filters),
            "-frames:v", "1", "-c:v", "mjpeg", "-q:v", "4", "-f", "mjpeg", "-",
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE)
        jpeg, _ = await proc.communicate(image.tobytes())
        if proc.returncode != 0 or not jpeg:
            raise RuntimeError("drawing the map failed")
        listed = [{k: v for k, v in p.items() if k not in ("x", "y")} for p in places]
        self._keep_for_debug("map", jpeg, listed)
        return web.Response(body=jpeg, content_type="image/jpeg",
                            headers={"X-Map-Places": json.dumps(listed, ensure_ascii=True),
                                     "X-Pose": json.dumps({"x": round(walker.x, 2),
                                                           "y": round(walker.y, 2),
                                                           "heading": round(walker.heading),
                                                           "odometry": walker.odometry,
                                                           "frames": walker.scene.frames})})

    async def h_note(self, request: web.Request) -> web.Response:
        """Names what is in a cell of the last view ahead: a landmark of the map."""
        body = await request.json()
        name = str(body.get("name") or "").strip()[:40]
        if not name:
            raise ValueError("name the thing")
        walker = self.navigator
        if walker.scene is None:
            raise RuntimeError("no map yet: look first")
        x, y = walker.cell_point(str(body["cell"]))
        walker.scene.note(name, x, y)
        bearing, far = walker.toward((x, y))
        return web.json_response({"ok": True, "name": name, "bearing": round(bearing),
                                  "m": round(far, 1)})

    async def h_goto(self, request: web.Request) -> web.Response:
        """Walks to a mark of the last /v1/nav view (0: turns around)."""
        self.require_game()
        body = await request.json()
        await self.set_camera(False)  # walking by the first person view, whatever was set
        await self.navigator.level()  # and looking level: the walk brakes by the floor ahead
        if body.get("landmark") or body.get("platform"):
            return await self._goto_place(body)
        if body.get("climb") and not body.get("cell"):
            raise ValueError("climb needs the cell of the thing: look ahead at it "
                             "(vrchat_view show ahead) and give its grid cell, e.g. D4")
        if body.get("cell"):
            # Toward what is in a cell of the last view ahead (a thing to
            # walk or climb up to).
            self.navigator.marks = [self.navigator.cell_place(str(body["cell"]))]
            body["mark"] = 98
        elif body.get("bearing") is not None:
            # Toward a bearing of the view (degrees, + right; read off its
            # ruler): a place of its own, as far as asked or as there is.
            bearing = clamp(float(body["bearing"]), -180, 180)
            far = clamp(float(body.get("distance") or nav.MAX_WALK_M), 0.3, nav.MAX_WALK_M)
            self.navigator.marks = [nav.Mark(99, bearing, far)]
            body["mark"] = 99
        number = int(body["mark"])
        detour = str(body.get("detour") or "auto")
        if detour not in nav.DETOURS:
            raise ValueError(f"detour must be one of {', '.join(nav.DETOURS)}")
        self._take_over()
        climb = bool(body.get("climb")) and number != 0
        result = await self._driving(
            self.navigator.goto(number, self.driver, drive.Step, detour, up_to=climb))
        if result is None:
            return web.json_response({"ok": False, "stopped": "stopped"})
        if climb and result.get("stopped") == "reached it":
            # Up onto what is there: close in from where the walk ended, jump.
            climb = asyncio.ensure_future(self._climb_while(self._drive_task_done))
            await self._driving(self._climb_onto())
            result["jump"] = await climb
            self.navigator.walked(axis_speed(CLIMB_AXIS) * CLIMB_MS / 1000)
        self._idle_later()
        return web.json_response(result)

    async def h_autopilot(self, request: web.Request) -> web.Response:
        """Ends the model's driving: following whom it paused, if anyone."""
        if self._resume is not None:
            self._resume.cancel()
            self._resume = None
        takeover, self.takeover = self.takeover, None
        if takeover and takeover.get("target"):
            self.require_game()
            self.follower.start(takeover["target"], takeover.get("distance"))
            await self._state_changed()
            return web.json_response({"following": takeover["target"],
                                      **self.follower.status()})
        self.notify_state()
        return web.json_response({"following": None})

    def follow_target(self) -> str:
        """The whitelisted player in the room with the highest priority."""
        here = {uid: name for uid, name in self.state.players.items() if uid != self.state.self_id}
        for uid in self.social.whitelist_ids():
            if uid in here:
                return here[uid]
        return ""

    async def h_follow(self, request: web.Request) -> web.Response:
        if request.method == "GET":
            if request.query.get("trace"):
                return web.json_response({**self.follower.status(),
                                          "trace": list(self.follower.trace)})
            return web.json_response(self.follower.status())
        body = await request.json() if request.can_read_body else {}
        if body.get("stop") or not body.get("adjust"):
            self.takeover = None  # following again, or not at all
        if body.get("stop"):
            self.follower.stop()
            await self._state_changed()
            return web.json_response(self.follower.status())
        if body.get("adjust"):
            change = str(body["adjust"])
            if change == "stay" and self.follower.state == "idle":
                await self.h_stop(request)  # not following: just stand still
            else:
                self.follower.adjust(change)
            await self._state_changed()
            return web.json_response(self.follower.status())
        self.require_game()
        name = str(body.get("name") or "").strip() or self.follow_target()
        if not name:
            raise RuntimeError("nobody to follow: name someone, or a whitelisted player must be here")
        distance = body.get("distance")
        self.follower.start(name, None if distance is None else float(distance))
        await self._state_changed()
        return web.json_response(self.follower.status())

    async def h_jump(self, request: web.Request) -> web.Response:
        self.require_game()
        await self.jump()
        return web.json_response({"ok": True})

    async def h_emote(self, request: web.Request) -> web.Response:
        self.require_game()
        body = await request.json()
        name = str(body["name"])
        if name not in EMOTES:
            raise ValueError(f"emote must be one of {', '.join(EMOTES)}")
        seconds = clamp(float(body.get("seconds", 2.0)), 0.5, 10.0)
        if not self.state.oscquery_port:
            raise RuntimeError("the game is not running")
        # Answered at once: the caller (a voice model) should not wait for
        # the emote to play out before it speaks.
        task = self._spawn(self.emote(EMOTES[name], seconds))
        task.add_done_callback(
            lambda t: t.cancelled() or t.exception() is None
            or log.warning("emote failed: %s", t.exception()))
        return web.json_response({"ok": True})

    async def h_social(self, request: web.Request) -> web.Response:
        return web.json_response(self.social.status())

    async def h_social_config(self, request: web.Request) -> web.Response:
        self.social.set_config(await request.json())
        return web.json_response(self.social.status())

    async def h_camera_y(self, request: web.Request) -> web.Response:
        """Looking up and down, for the model to see and set by eye.
        ``action`` view: a tall picture from looking up (top) to looking down
        (bottom), lines every CAMERA_Y_LINE_DEG from where it looks now
        ("now"), back where it was after. ``action`` set: looks up (+) or
        down by ``degrees`` (the next view or walk puts it level again).
        ``action`` level: looks by ``horizon`` (the line of the tall picture
        the far horizon is on) and takes that as level from then on. set and
        level answer the view with its lines."""
        self.require_game()
        body = await request.json() if request.can_read_body else {}
        await self.set_camera(False)
        w, h = (int(v) for v in self.args.screen.split("x"))
        action = body.get("action", "view")
        if action in ("set", "level"):
            key = "horizon" if action == "level" else "degrees"
            if body.get(key) is None:
                raise ValueError(f"{action} needs {key}: degrees, a line of the tall picture "
                                 "(action view), + up, - down")
            degrees = clamp(float(body[key]), -80, 80)
            if degrees:
                await self.look(dy=-round(degrees * drive.PX_PER_DEGREE_Y))
                await asyncio.sleep(CAMERA_Y_SETTLE_S)
            if action == "level":
                self.navigator.pitch = 0.0
            lines = [f for deg in range(-30, 31, CAMERA_Y_LINE_DEG)
                     for f in self._pitch_line(w, h / 2 - FOCAL_PX * math.tan(math.radians(deg)),
                                               deg)]
            jpeg = await self.scaled_jpeg(await self.screenshot("jpg"), 960, lines)
            return web.Response(body=jpeg, content_type="image/jpeg")
        span = CAMERA_Y_SHOT_DEG
        await self.look(dy=-round(span * drive.PX_PER_DEGREE_Y))
        await asyncio.sleep(CAMERA_Y_SETTLE_S)
        above = await self.screenshot("jpg")
        await self.look(dy=round(2 * span * drive.PX_PER_DEGREE_Y))
        await asyncio.sleep(CAMERA_Y_SETTLE_S)
        below = await self.screenshot("jpg")
        await self.look(dy=-round(span * drive.PX_PER_DEGREE_Y))  # as it was

        def row(deg: float, centre: float) -> float:
            return h / 2 - FOCAL_PX * math.tan(math.radians(deg - centre))

        # The upper shot down to "now", the lower one from "now" on.
        cut_above = round(row(0, span))
        cut_below = round(row(0, -span))
        top = [f"crop={w}:{cut_above}:0:0"] + [
            f for deg in range(CAMERA_Y_LINE_DEG, 90, CAMERA_Y_LINE_DEG)
            if 0 <= row(deg, span) < cut_above for f in self._pitch_line(w, row(deg, span), deg)]
        bottom = [f"crop={w}:{h - cut_below}:0:{cut_below}"] + [
            f for deg in range(0, -90, -CAMERA_Y_LINE_DEG)
            if 0 <= row(deg, -span) - cut_below < h - cut_below
            for f in self._pitch_line(w, row(deg, -span) - cut_below, deg)]
        with tempfile.TemporaryDirectory() as tmp:
            paths = [os.path.join(tmp, f"{i}.jpg") for i in range(2)]
            for path, shot in zip(paths, (above, below), strict=True):
                with open(path, "wb") as f:
                    f.write(shot)
            proc = await asyncio.create_subprocess_exec(
                "ffmpeg", "-loglevel", "error", "-i", paths[0], "-i", paths[1], "-filter_complex",
                f"[0:v]{','.join(top)}[a];[1:v]{','.join(bottom)}[b];[a][b]vstack,scale=640:-2",
                "-frames:v", "1", "-c:v", "mjpeg", "-q:v", "4", "-f", "mjpeg", "-",
                stdout=asyncio.subprocess.PIPE)
            jpeg, _ = await proc.communicate()
        if proc.returncode != 0 or not jpeg:
            raise RuntimeError("drawing the picture failed")
        return web.Response(body=jpeg, content_type="image/jpeg")

    @staticmethod
    def _pitch_line(width: int, y: float, deg: int) -> list[str]:
        """ffmpeg filters: a line across at row ``y``, labelled with its
        degrees from where the view looks now (yellow: now)."""
        now = deg == 0
        color = "yellow" if now else "white@0.7"
        text = "now 0" if now else f"{deg:+d}"
        return [(f"drawbox=x=0:y={round(y) - (1 if now else 0)}:w={width}:h={3 if now else 1}:"
                 f"color={color}:t=fill"),
                (f"drawtext=font=monospace:text='{text}':x=10:y={max(0, round(y) - 34)}:"
                 f"fontsize=30:fontcolor={'yellow' if now else 'white'}:borderw=3:bordercolor=black")]

    async def h_look(self, request: web.Request) -> web.Response:
        body = await request.json()
        await self.look(int(clamp(int(body.get("dy", 0)), -600, 600)),
                        int(clamp(int(body.get("dx", 0)), -2000, 2000)))
        return web.json_response({"ok": True})

    async def h_screenshot(self, request: web.Request) -> web.Response:
        self.require_game()
        fmt = "jpg" if request.query.get("format", "jpg") == "jpg" else "png"
        width = int(clamp(int(request.query.get("width", 0)), 0, 3840))
        data = await self.screenshot(fmt, width, request.query.get("grid") == "1")
        return web.Response(body=data, content_type="image/jpeg" if fmt == "jpg" else "image/png")

    def whitelisted_here(self) -> list[str]:
        """Display names of the whitelisted players in the room, by priority."""
        here = {uid: name for uid, name in self.state.players.items() if uid != self.state.self_id}
        return [here[uid] for uid in self.social.whitelist_ids() if uid in here]

    async def h_sightings(self, request: web.Request) -> web.Response:
        """The sightings; with ``name``, also the one ``/v1/sightings/image``
        picks for it (``match``, or null)."""
        body: dict = {"sightings": self.sightings.listing()}
        if "name" in request.query:
            found = self.sightings.latest(request.query["name"])
            body["match"] = next(
                (s for s in body["sightings"] if found and s["name"] == found[0]), None)
        return web.json_response(body)

    async def h_sighting_image(self, request: web.Request) -> web.Response:
        found = self.sightings.latest(request.query.get("name", ""))
        if found is None:
            raise RuntimeError("no whitelisted friend has been seen yet")
        name, sighting = found
        data = sighting["jpeg"]
        width = int(clamp(int(request.query.get("width", 0)), 0, 3840))
        if width:
            data = await self.scaled_jpeg(data, width)
        return web.Response(body=data, content_type="image/jpeg", headers={
            "X-Sighting-Name": urllib.parse.quote(name),
            "X-Sighting-At": str(sighting["at"]),
            "X-Sighting-World": urllib.parse.quote(sighting["world"])})

    async def h_game_start(self, request: web.Request) -> web.Response:
        body = await request.json() if request.can_read_body else {}
        url = str(body.get("url") or "").strip()
        location = launch_location(url) if url else ""
        if body.get("whitelisted_only"):
            friends = self.social.friends
            if not location or not any(
                friends.get(uid, {}).get("location") == location
                for uid in self.social.whitelist_ids()
            ):
                raise ValueError("only the instance a whitelisted friend is in now")
        # Rebuilt from the checked instance: nothing else reaches the game.
        url = launch_url(location) if location else ""
        async with self.game_lock:  # not across a join's restart
            if body.get("restart"):
                await self.stop_game()
            await self.start_game(url)
        return web.json_response({"ok": True})

    async def h_game_stop(self, request: web.Request) -> web.Response:
        await self.stop_game()
        return web.json_response({"ok": True, "running": await self._game_pid() is not None})

    def app(self) -> web.Application:
        app = web.Application(middlewares=[self._auth])
        app.add_routes([
            web.get("/v1/stream", self.h_stream),
            web.get("/v1/status", self.h_status),
            web.post("/v1/chatbox", self.h_chatbox),
            web.post("/v1/move", self.h_move),
            web.post("/v1/turn", self.h_turn),
            web.post("/v1/stop", self.h_stop),
            web.post("/v1/drive", self.h_drive),
            web.get("/v1/nav", self.h_nav),
            web.post("/v1/goto", self.h_goto),
            web.get("/v1/look_around", self.h_look_around),
            web.get("/v1/map", self.h_map),
            web.post("/v1/note", self.h_note),
            web.post("/v1/autopilot", self.h_autopilot),
            web.post("/v1/jump", self.h_jump),
            web.post("/v1/look", self.h_look),
            web.post("/v1/camera_y", self.h_camera_y),
            web.post("/v1/step", self.h_step),
            web.post("/v1/emote", self.h_emote),
            web.get("/v1/follow", self.h_follow),
            web.post("/v1/follow", self.h_follow),
            web.get("/v1/social", self.h_social),
            web.post("/v1/social/config", self.h_social_config),
            web.get("/v1/screenshot", self.h_screenshot),
            web.get("/v1/sightings", self.h_sightings),
            web.get("/v1/sightings/image", self.h_sighting_image),
            web.post("/v1/game/start", self.h_game_start),
            web.post("/v1/game/stop", self.h_game_stop),
        ])

        async def background(_app):
            tasks = [asyncio.create_task(c) for c in
                     (self._capture(), self.logtail.run(), self._watchdog(), self._chatbox_sender(),
                      self.social.run(), self.sightings.run(),
                      odometry.Odometer(lambda: self.state.oscquery_port, self.navigator).run())]
            yield
            for task in tasks:
                task.cancel()
            self.follower.stop()
            if self._ptt_held:
                self.osc.send("/input/Voice", 0)
            await self.social.api.close()

        app.cleanup_ctx.append(background)
        return app


# Bearings marked on a screenshot's top (``grid``), degrees.
RULER_DEGREES = (-45, -30, -15, 0, 15, 30, 45)


def bearing_ruler(width: int, height: int) -> list[str]:
    """ffmpeg filters marking, along the top of a ``width`` x ``height``
    game frame, the bearing of each column: how far to turn (degrees, +
    right) to face it. The view's focal length scales with its width."""
    focal = FOCAL_PX * width / 1280
    filters = []
    for deg in RULER_DEGREES:
        x = round(width / 2 + focal * math.tan(math.radians(deg)))
        if not 0 <= x < width:
            continue
        tick = 26 if deg == 0 else 16
        filters.append(f"drawbox=x={x - 1}:y=0:w=3:h={tick}:color=yellow@0.9:t=fill")
        label = f"{deg:+d}" if deg else "0"
        filters.append(
            f"drawtext=font=monospace:text='{label}':x={x}-tw/2:y={tick + 2}:fontsize=18:"
            "fontcolor=yellow:borderw=2:bordercolor=black")
    return filters


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def split_chatbox(text: str) -> list[str]:
    """Splits ``text`` into chatbox messages of at most 144 characters,
    preferring sentence ends, then spaces."""
    text = " ".join(text.split())
    parts: list[str] = []
    while len(text) > CHATBOX_LIMIT:
        window = text[:CHATBOX_LIMIT]
        cut = max(window.rfind(c) for c in "。！？!?；;，,. ")
        cut = cut + 1 if cut >= CHATBOX_LIMIT // 2 else CHATBOX_LIMIT
        parts.append(text[:cut].strip())
        text = text[cut:].strip()
    if text:
        parts.append(text)
    return parts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", nargs="?", default="serve", choices=["serve", "login"],
                        help="login: log the bot account in to the VRChat Web API (interactive)")
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6120)
    parser.add_argument("--token-file", default="~/.config/vrc-bridge/token")
    parser.add_argument("--log-dir", default=LOG_DIR)
    parser.add_argument("--osc-port", type=int, default=9000)
    parser.add_argument("--display", default=":1")
    parser.add_argument("--screen", default="1280x720")
    parser.add_argument("--capture-device", default="vrc_out.monitor")
    parser.add_argument("--playback-device", default="vrc_mic_in")
    parser.add_argument("--vram-soft-mib", type=int, default=4500)
    parser.add_argument("--vram-hard-mib", type=int, default=6000)
    parser.add_argument("--gpu-hard-mib", type=int, default=11000)
    parser.add_argument("--ocr-url", default="http://127.0.0.1:17890/v1/ocr/lines",
                        help="local-multimodal-infra's text lines endpoint (name tags)")
    parser.add_argument("--ocr-model", default="ppocrv5-mobile-onnx")
    parser.add_argument("--depth-url", default="http://127.0.0.1:17890/v1/depth",
                        help="local-multimodal-infra's metric depth endpoint (how far the "
                             "followed player is); empty to only face them")
    parser.add_argument("--depth-model", default="depth-anything-v2-metric-indoor-small-onnx")
    parser.add_argument("--ocr-token-file", default="",
                        help="the infra inference token, if it requires one")
    args = parser.parse_args()
    if args.command == "login":
        cookies = Path(args.token_file).expanduser().parent / "cookies.json"
        asyncio.run(login_main(cookies))
        return
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    web.run_app(Bridge(args).app(), host=args.listen, port=args.port, print=None)


if __name__ == "__main__":
    main()
