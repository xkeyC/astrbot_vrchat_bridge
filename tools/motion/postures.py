"""Keyed postures made by rule (keyframes.py specs): sitting cross-legged,
lying on the back, either side or the front.

Angles are keyframes.py's: degrees about the body's axes (x its left, y up,
z ahead), turning each joint from the calibration pose.
"""

import json
import random

import numpy as np
from scipy.spatial.transform import Rotation

# Sitting on a seat (the hips where Quaternius' sitting puts them).
SEAT = [0.0, -0.23, -0.15]


# The body sways from one side over to the other in SWAY_S, stays there
# SWAY_GAP_S, then back over to the first (degrees either side).
SWAY_DEG = 3.0
SWAY_S = 3.0
SWAY_GAP_S = 1.0
# The top foot swings to and fro once in SWING_S (degrees at the knee).
SWING_DEG = 5.0
SWING_S = 3.0
# Changing legs: the top leg down, a pause, the other up (seconds; 2 in all).
DOWN_S, PAUSE_S, UP_S = 0.9, 0.2, 0.9
# The crossed leg: thigh raised (degrees ahead), across (toward the other
# side), knee bend; the leg under it, its shin drawn back a little.
CROSS_FLEX, CROSS_IN, CROSS_KNEE = -103.0, 19.0, 66.0
UNDER_KNEE = 93.0
# Between changes (seconds, at random).
CHANGE_EVERY = (30.0, 120.0)


def sway(t):
    """The body's sway at `t` (degrees, + to its left): from the right over
    to the left, a pause, back over to the right, a pause."""
    half = SWAY_S + SWAY_GAP_S
    u = t % (2 * half)
    going_left = u < half
    v = u % half
    k = smooth(v / SWAY_S) if v < SWAY_S else 1.0
    x = -1.0 + 2.0 * k if going_left else 1.0 - 2.0 * k
    return SWAY_DEG * x


def smooth(x):
    """0..1 to 0..1, still at both ends (smootherstep)."""
    x = min(max(x, 0.0), 1.0)
    return x * x * x * (x * (6 * x - 15) + 10)


def _leg_over(p):
    """A leg on its way from beside the other (p=0) to crossed over it
    (p=1): its thigh, knee and foot angles. On the way it rises and its knee
    stretches (most at the middle), so it passes over the other leg; the
    path is one smooth curve, never still in between."""
    s = smooth(p)
    bump = float(np.sin(np.pi * p))
    # Across (adduction) mostly once it is up: later than the rest.
    across = smooth((p - 0.25) / 0.65)
    # Crossed: the thigh up a little more than sitting and over the other
    # knee, the shin slanting ahead so the foot hangs in front of the other
    # shin (not through it).
    flex = -90 + (CROSS_FLEX + 90) * s - 24 * bump
    inward = 4 + (CROSS_IN - 4) * across
    knee = 90 + (CROSS_KNEE - 90) * s - 40 * bump
    foot = -12 * bump
    return flex, inward, knee, foot


def _sit_frame(t, top, moving, p, swing):
    """The sitting pose at `t`: `top` leg over the other (None: side by
    side), or `moving` leg part `p` of the way from beside to over; the top
    foot swung `swing` degrees."""
    sw = sway(t)
    pose = {
        "hips": [-5, 0, 0], "chest": [4, 0, sw], "head": [6, 0, -sw * 0.6],
        "l_upleg": [-90, -4, 0], "r_upleg": [-90, 4, 0], "l_leg": [90, 0, 0], "r_leg": [90, 0, 0],
        # Arms down at the sides, a little back and out, the hands flat on
        # the seat, fingers ahead: propped on it.
        "l_arm": [14, 0, 14], "r_arm": [14, 0, -14], "l_forearm": [-8, 0, 0], "r_forearm": [-8, 0, 0],
        "l_hand": [-75, 0, 0], "r_hand": [-75, 0, 0],
    }
    leg, q = (top, 1.0) if top else (moving, p)
    if leg and q > 0:
        sign = 1 if leg == "r" else -1
        under = "l" if leg == "r" else "r"
        flex, inward, knee, foot = _leg_over(q)
        pose[f"{leg}_upleg"] = [flex, sign * inward, 0]
        pose[f"{leg}_leg"] = [knee + swing, 0, 0]
        pose[f"{leg}_foot"] = [foot - swing * 0.8, 0, 0]
        pose[f"{under}_upleg"] = [-90, -sign * (4 + 2 * smooth(q)), 0]
        pose[f"{under}_leg"] = [90 + (UNDER_KNEE - 90) * smooth(q), 0, 0]
        # The body leans a little away from the raised leg.
        pose["chest"] = [4 - 3 * float(np.sin(np.pi * q)) * (q < 1), 0, sw - sign * 3 * smooth(q)]
    return pose


FLAT = {"l": [-0.25, -0.25, -0.25, -0.25, -0.1], "r": [-0.25, -0.25, -0.25, -0.25, -0.1]}


def sit_cross(seed=7, changes=4, fps=30):
    """A loop of sitting cross-legged, computed frame by frame: the legs
    change over every 30-120 s at random (2 s: down, a pause, the other up),
    the top foot swinging slowly, the body swaying. An even number of
    changes, so it ends as it began (right over left) and loops."""
    rng = random.Random(seed)
    plan, t = [], 0.0
    for _ in range(changes):
        t += rng.uniform(*CHANGE_EVERY)
        plan.append(t)
    length = t + DOWN_S + PAUSE_S + UP_S + rng.uniform(*CHANGE_EVERY) / 2
    frames = []
    top = "r"
    for i in range(int(length * fps)):
        t = i / fps
        # Where in a change, if in one.
        c = next((c for c in plan if c <= t < c + DOWN_S + PAUSE_S + UP_S), None)
        before = sum(1 for c0 in plan if c0 + DOWN_S + PAUSE_S + UP_S <= t)
        top = "r" if before % 2 == 0 else "l"
        if c is None:
            # The swing fades in after a change.
            since = t - max([c0 + DOWN_S + PAUSE_S + UP_S for c0 in plan if c0 <= t] or [-10.0])
            gain = smooth(since / 1.5)
            swing = SWING_DEG * gain * float(np.sin(2 * np.pi * t / SWING_S))
            pose = _sit_frame(t, top, None, 0.0, swing)
        else:
            u = t - c
            leg = top
            if u < DOWN_S:
                pose = _sit_frame(t, None, leg, 1.0 - u / DOWN_S, 0.0)
            elif u < DOWN_S + PAUSE_S:
                pose = _sit_frame(t, None, None, 0.0, 0.0)
            else:
                other = "l" if leg == "r" else "r"
                pose = _sit_frame(t, None, other, (u - DOWN_S - PAUSE_S) / UP_S, 0.0)
        frames.append({"pose": pose, "hips": SEAT, "curl": FLAT})
    return {"name": "sitting", "fps": fps, "loop": True, "frames": frames}


def _euler(R):
    return [round(float(x), 2) for x in Rotation.from_matrix(R).as_euler("xyz", degrees=True)]


def _lying_hips(head_yaw_deg, roll_deg):
    """The hips' rotation of a body lying on its back with its head toward
    `head_yaw_deg` (the body frame's yaw: + toward its left), rolled
    `roll_deg` about its own length (+ onto its left side)."""
    back = Rotation.from_euler("x", -90, degrees=True)  # face up, head toward -z
    roll = Rotation.from_euler("y", roll_deg, degrees=True)
    yaw = Rotation.from_euler("y", head_yaw_deg + 180.0, degrees=True)
    return (yaw * back * roll).as_matrix()


LIMBS = {
    "back": {"l_arm": [0, 0, 18], "r_arm": [0, 0, -18], "l_upleg": [0, 0, 4], "r_upleg": [0, 0, -4],
             "head": [-8, 0, 0], "l_forearm": [-15, 0, 0], "r_forearm": [-15, 0, 0]},
    # On the left side: knees drawn up, the top one further, the lower arm
    # under the head, the upper one resting in front.
    "left": {"l_upleg": [-45, 0, 0], "r_upleg": [-60, 6, 0], "l_leg": [75, 0, 0], "r_leg": [80, 0, 0],
             "l_arm": [-150, 0, 10], "l_forearm": [-80, 0, 0], "r_arm": [-35, 0, 0], "r_forearm": [-50, 0, 0],
             "head": [0, 0, 12], "chest": [6, 0, 0]},
    # On the front: arms folded under the head, which is turned aside, the
    # lower legs up.
    "front": {"l_arm": [-160, 0, 25], "r_arm": [-160, 0, -25], "l_forearm": [-110, 0, 0],
              "r_forearm": [-110, 0, 0], "head": [-25, 55, 0], "l_leg": [95, 0, 0], "r_leg": [110, 0, 0],
              "l_upleg": [0, 0, 6], "r_upleg": [0, 0, -6]},
}


def _mirror_pose(pose):
    out = {}
    for j, (x, y, z) in pose.items():
        k = "r_" + j[2:] if j.startswith("l_") else ("l_" + j[2:] if j.startswith("r_") else j)
        out[k] = [x, -y, -z]
    return out


def lying(way, along, at, length=8.0):
    """A loop of lying `way` (back, left, right, front), the head toward
    `along` (degrees, body frame) with the hips over `at` (x, z), breathing;
    on the front the lower legs swing."""
    roll = {"back": 0.0, "left": 90.0, "right": -90.0, "front": 180.0}[way]
    limbs = LIMBS["left"] if way in ("left", "right") else LIMBS[way]
    if way == "right":
        limbs = _mirror_pose(limbs)
    hips_rot = _euler(_lying_hips(along, roll))
    # Low: keyframes keeps every joint its thickness above the floor.
    hips = [at[0], -0.53, at[1]]
    keys = []
    n = 4
    for i in range(n + 1):
        t = length * i / n
        breath = 2.0 if i % 2 else 0.0
        pose = {j: list(v) for j, v in limbs.items()}
        pose["hips"] = hips_rot
        c = pose.get("chest", [0, 0, 0])
        pose["chest"] = [c[0] - breath, c[1], c[2]]
        if way == "front":
            swing = 18 if i % 2 else -10
            pose["l_leg"] = [95 + swing, 0, 0]
            pose["r_leg"] = [110 - swing, 0, 0]
        keys.append({"t": t, "pose": pose, "hips": hips})
    return {"name": f"lying_{way}", "fps": 30, "loop": True, "keys": keys}


def lying_from_clip(path):
    """Where the CMU lying clip has its body (head direction, hips): the keyed
    lying goes on from there."""
    c = json.load(open(path))
    f = np.asarray(c["frames"][-1], float).reshape(11, 7)
    hips, head = f[0, :3], f[2, :3]
    d = head - hips
    along = float(np.degrees(np.arctan2(d[0], d[2])))
    return along, (float(hips[0]), float(hips[2]))
