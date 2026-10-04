"""Driving the avatar by a script: the model's hands when it takes over.

A script is a short sequence of steps, run one after another; a step is the
inputs held *together* for its duration (an action chunk): walking in a
direction at a speed, running, a jump, turning and looking by degrees. So
"jump while walking forward" is one step: ``{"move": "forward", "jump":
true, "ms": 600}``. Inputs not in a step are released for it, and all are
released when the script ends or is stopped. The model sees the view after
each script and writes the next: screenshot, script, screenshot, ...

Turning and looking go by relative mouse motion (VRChat's OSC look axis has
a dead zone), spread over the step's duration, or at once in a step of 0 ms.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass

# Mouse motion of one full turn (desktop mode; as follow.FULL_TURN_PX).
FULL_TURN_PX = 3600
PX_PER_DEGREE = FULL_TURN_PX / 360
# Up and down the same mouse motion turns the view less: 200, 300 and -300 px
# turned it 15.3, 23.4 and -21.9 degrees (measured against the view).
PX_PER_DEGREE_Y = 13.1
MOUSE_STEP_PX = 25  # larger single jumps are dropped by the game
MOUSE_STEP_S = 0.03
JUMP_PRESS_S = 0.1

MAX_STEPS = 24
MAX_STEP_MS = 3000
MAX_SCRIPT_MS = 10000
# Walking blind is how the model gets lost (and through portals): a script
# walks at most this long, then the model looks again.
MAX_WALK_MS = 1000
MAX_TURN_DEG = 360
MAX_LOOK_DEG = 80

# Direction of travel relative to where the avatar faces: (forward, right).
MOVES = {
    "forward": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, -1.0), "right": (0.0, 1.0),
    "forward-left": (0.7, -0.7), "forward-right": (0.7, 0.7),
    "back-left": (-0.7, -0.7), "back-right": (-0.7, 0.7),
}

# What a step may hold, for the tools' input schema.
STEP_SCHEMA = {
    "type": "object",
    "properties": {
        "move": {"type": "string", "enum": list(MOVES),
                 "description": "Walk this way relative to where you face, for the step."},
        "speed": {"type": "number",
                  "description": "Walking speed 0.2-1 (default 1: about 4 m a second; "
                                 "0.5: about 2; 0.3: under 1)."},
        "run": {"type": "boolean", "description": "Run instead of walk."},
        "jump": {"type": "boolean", "description": "Jump at the step's start."},
        "turn": {"type": "number",
                 "description": "Turn by degrees, + right, - left (90 is a quarter turn)."},
        "look": {"type": "number", "description": "Look up (+) or down (-) by degrees."},
        "ms": {"type": "integer",
               "description": f"How long the step lasts, ms (0-{MAX_STEP_MS})."},
    },
    "additionalProperties": False,
}


@dataclass(frozen=True)
class Step:
    forward: float = 0.0
    right: float = 0.0
    run: bool = False
    jump: bool = False
    turn_deg: float = 0.0
    look_deg: float = 0.0
    ms: int = 0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse(steps: object) -> list[Step]:
    """A script from its JSON form.

    Raises:
        ValueError: Not a list of steps, an unknown key or move, too many
            steps or too long in all.
    """
    if not isinstance(steps, list) or not steps:
        raise ValueError("steps must be a non-empty list")
    if len(steps) > MAX_STEPS:
        raise ValueError(f"at most {MAX_STEPS} steps")
    parsed = []
    for i, raw in enumerate(steps):
        if not isinstance(raw, dict):
            raise ValueError(f"step {i + 1} must be an object")
        unknown = set(raw) - set(STEP_SCHEMA["properties"])
        if unknown:
            raise ValueError(f"step {i + 1}: unknown {', '.join(sorted(unknown))}")
        forward = right = 0.0
        if raw.get("move"):
            if raw["move"] not in MOVES:
                raise ValueError(f"step {i + 1}: move must be one of {', '.join(MOVES)}")
            speed = clamp(float(raw.get("speed", 1.0)), 0.2, 1.0)
            forward, right = (v * speed for v in MOVES[raw["move"]])
        parsed.append(Step(
            forward=forward, right=right, run=bool(raw.get("run", False)),
            jump=bool(raw.get("jump", False)),
            turn_deg=clamp(float(raw.get("turn", 0)), -MAX_TURN_DEG, MAX_TURN_DEG),
            look_deg=clamp(float(raw.get("look", 0)), -MAX_LOOK_DEG, MAX_LOOK_DEG),
            ms=int(clamp(int(raw.get("ms", 0)), 0, MAX_STEP_MS)),
        ))
    total = sum(step.ms for step in parsed)
    if total > MAX_SCRIPT_MS:
        raise ValueError(f"the steps last {total} ms, at most {MAX_SCRIPT_MS}")
    # A jump's run-up is no blind walk.
    walking = sum(step.ms for step in parsed if (step.forward or step.right) and not step.jump)
    if walking > MAX_WALK_MS:
        raise ValueError(f"the steps walk {walking} ms, at most {MAX_WALK_MS}: to get "
                         "somewhere use vrchat_goto (a numbered place of your view)")
    return parsed


def mouse_path(dx: int, dy: int) -> list[tuple[int, int]]:
    """Relative mouse moves of at most MOUSE_STEP_PX adding up to (dx, dy)."""
    steps = max(abs(dx), abs(dy)) // MOUSE_STEP_PX + (1 if dx or dy else 0)
    path, done_x, done_y = [], 0, 0
    for i in range(1, steps + 1):
        x, y = dx * i // steps - done_x, dy * i // steps - done_y
        done_x, done_y = done_x + x, done_y + y
        if x or y:
            path.append((x, y))
    return path


class Driver:
    """Runs scripts on the bridge's inputs: OSC axes and buttons, the mouse."""

    def __init__(self, osc, mouse) -> None:
        """``osc.send(address, value)``; ``await mouse(dx, dy)`` moves the
        mouse relatively."""
        self.osc = osc
        self.mouse = mouse

    def _hold(self, step: Step) -> None:
        self.osc.send("/input/Vertical", float(step.forward))
        self.osc.send("/input/Horizontal", float(step.right))
        self.osc.send("/input/Run", 1 if step.run else 0)

    def release(self) -> None:
        self.osc.send("/input/Vertical", 0.0)
        self.osc.send("/input/Horizontal", 0.0)
        self.osc.send("/input/Run", 0)
        self.osc.send("/input/Jump", 0)

    async def _jump(self) -> None:
        self.osc.send("/input/Jump", 1)
        await asyncio.sleep(JUMP_PRESS_S)
        self.osc.send("/input/Jump", 0)

    async def _look(self, step: Step, seconds: float) -> None:
        path = mouse_path(round(step.turn_deg * PX_PER_DEGREE),
                          round(-step.look_deg * PX_PER_DEGREE_Y))
        # Spread over the step (a turn while walking curves the path), but
        # never faster than the game takes mouse motion.
        pause = max(MOUSE_STEP_S, seconds / len(path)) if path else 0.0
        for dx, dy in path:
            await self.mouse(dx, dy)
            await asyncio.sleep(pause)

    async def run(self, steps: list[Step]) -> int:
        """Runs ``steps``; returns the milliseconds they took. Every input
        is released at the end, or when cancelled."""
        loop = asyncio.get_running_loop()
        started = loop.time()
        try:
            for step in steps:
                self._hold(step)
                seconds = step.ms / 1000
                parts = [asyncio.sleep(seconds)]
                if step.jump:
                    parts.append(self._jump())
                if step.turn_deg or step.look_deg:
                    parts.append(self._look(step, seconds))
                await asyncio.gather(*parts)
        finally:
            self.release()
        return round((loop.time() - started) * 1000)
