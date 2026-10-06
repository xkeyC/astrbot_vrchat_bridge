"""Clips keyed by hand on the canonical skeleton: a few poses, eased between.

    python keyframes.py poses/think.json [--out DIR] [--preview]

A pose file:

    {"name": "think", "fps": 30, "loop": false,
     "keys": [
       {"t": 0.0},                                  # standing
       {"t": 0.6, "pose": {"r_arm": [0, 0, -60], "r_forearm": [0, -120, 0],
                           "head": [10, 0, 8]},
                  "hips": [0, -0.01, 0], "curl": {"r": [0.9, 0.9, 0.8, 0.3, 0.6]}},
       {"t": 2.5, "same": true},                    # hold the previous pose
       {"t": 3.1}                                   # back to standing
     ]}

Each joint's angles (degrees, x then y then z, about the body's axes: x its
left, y up, z ahead) turn it from where it is in the calibration pose
(`skeleton.STANDING`); `hips` moves the body (stature units). Finger curls
are changes from the relaxed hand, little to thumb, per side. A key without
a pose is standing. Between keys the motion eases in and out.
"""

import argparse
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation

import retarget
import skeleton as sk


def local_standing():
    """Each joint's rotation relative to its parent in STANDING."""
    out = {}
    for j in sk.NAMES:
        parent = sk.JOINTS[j][0]
        g = sk.STANDING_R[j][0]
        out[j] = g if parent is None else sk.STANDING_R[parent][0].T @ g
    return out


def pose_rotations(pose):
    """Joint -> extra rotation (3x3), from angles in degrees."""
    return {j: Rotation.from_euler("xyz", a, degrees=True).as_matrix() for j, a in (pose or {}).items()}


def ease(w):
    return w * w * (3 - 2 * w)


def build(spec, fps_out=None):
    """A clip from a spec of keys (eased between), or of `frames` (one pose
    per frame at the spec's rate, as computed by rule: postures.py)."""
    fps = fps_out or spec.get("fps", 30)
    if "frames" in spec:
        keys = [dict(f, t=i / fps) for i, f in enumerate(spec["frames"])]
    else:
        keys = spec["keys"]
    # Resolve "same" (hold) keys.
    for i, k in enumerate(keys):
        if k.get("same") and i > 0:
            for f in ("pose", "hips", "curl"):
                if f in keys[i - 1]:
                    k[f] = keys[i - 1][f]
    base = local_standing()
    times = np.arange(0.0, keys[-1]["t"] + 1e-9, 1.0 / fps)
    n = len(times)
    local = {j: np.zeros((n, 3, 3)) for j in sk.NAMES}
    hips = np.zeros((n, 3))
    curls = np.zeros((n, 2, 5))
    import bisect
    key_times = [k["t"] for k in keys]
    for i, t in enumerate(times):
        b = min(max(bisect.bisect_left(key_times, t - 1e-9), 1), len(keys) - 1)
        a = b - 1
        span = max(keys[b]["t"] - keys[a]["t"], 1e-6)
        w = ease(float(np.clip((t - keys[a]["t"]) / span, 0, 1)))
        ra, rb = pose_rotations(keys[a].get("pose")), pose_rotations(keys[b].get("pose"))
        for j in sk.NAMES:
            qa = Rotation.from_matrix(ra.get(j, np.eye(3)))
            qb = Rotation.from_matrix(rb.get(j, np.eye(3)))
            from scipy.spatial.transform import Slerp
            q = Slerp([0, 1], Rotation.concatenate([qa, qb]))([w])[0]
            # The extra turn is about the body's axes at the joint: applied
            # in the parent's frame before the joint's standing rotation.
            parent = sk.JOINTS[j][0]
            pg = np.eye(3) if parent is None else sk.STANDING_R[parent][0]
            extra = pg.T @ q.as_matrix() @ pg
            local[j][i] = extra @ base[j]
        ha = np.array(keys[a].get("hips", [0, 0, 0]), float)
        hb = np.array(keys[b].get("hips", [0, 0, 0]), float)
        hips[i] = ha + (hb - ha) * w
        for s, side in enumerate(("l", "r")):
            ca = np.array(keys[a].get("curl", {}).get(side, [0] * 5), float)
            cb = np.array(keys[b].get("curl", {}).get(side, [0] * 5), float)
            curls[i, s] = ca + (cb - ca) * w
    root = np.array([0.0, 0.530, 0.0]) + hips
    P, R = sk.fk(local, root)
    retarget.keep_above_floor(P, fps)
    rel = {k: R[k] @ np.linalg.inv(sk.STANDING_R[k][0]) for k in R}
    q = {k: retarget.rot_to_quat(rel[k]) for k in sk.OUTPUT}
    frames = []
    for i in range(n):
        row = []
        for k in sk.OUTPUT:
            row += [round(float(x), 4) for x in P[k][i]] + [round(float(x), 4) for x in q[k][i]]
        frames.append(row)
    return {
        "name": spec["name"],
        "fps": fps,
        "loop": spec.get("loop", False),
        "source": "keyframes",
        "license": "CC0 (keyed by hand)",
        "joints": sk.OUTPUT,
        "standing": {k: [round(float(x), 4) for x in sk.STANDING_P[k][0]] for k in sk.OUTPUT},
        "eyes": [float(x) for x in sk.EYES],
        "end": {"x": 0.0, "z": 0.0, "yaw_deg": 0.0},
        "frames": frames,
        "curls": [[round(float(x), 2) for x in c.reshape(-1)] for c in curls],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spec")
    ap.add_argument("--out", default="clips")
    ap.add_argument("--preview", action="store_true")
    a = ap.parse_args()
    spec = json.load(open(a.spec, encoding="utf-8"))
    clip = build(spec)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, clip["name"] + ".json")
    json.dump(clip, open(path, "w"), separators=(",", ":"))
    print(f"{path}: {len(clip['frames'])} frames, {len(clip['frames']) / clip['fps']:.1f} s")
    if a.preview:
        import preview
        preview.sheet(clip, os.path.join(a.out, clip["name"] + ".png"))


if __name__ == "__main__":
    main()
