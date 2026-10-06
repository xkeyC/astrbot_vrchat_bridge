"""Converts a motion into a clip for the bridge's player (vrc_vr::motion).

    python convert.py NAME SOURCE [--start S] [--end S] [--loop] [--fps 30]
        [--kind bvh] [--license TEXT] [--out DIR]

A clip (JSON) holds, at a fixed rate, the canonical skeleton's tracked
points (skeleton.OUTPUT): each joint's position (stature units, body axes:
x left, y up, z ahead, from the floor under where the hips started) and
its rotation relative to the calibration pose (x, y, z, w). See
docs/full-vr/motion.md.
"""

import argparse
import json
import os

import numpy as np

import retarget
import skeleton as sk
import sources


def build(name, path, kind="cmu", start=0.0, end=None, fps=30.0, loop=False, license="", ref=None, src_fps=None,
          anim=None, origin=None, root=None, cycle=False, min_gap=None):
    if kind in sources.MAPPINGS:
        src, fps_in = sources.bvh(path, sources.MAPPINGS[kind], src_fps, ref)
    elif kind == "gltf":
        src, fps_in = sources.gltf(path, anim)
    else:
        raise SystemExit(f"unknown source kind {kind}")
    if min_gap:
        src["min_gap"] = min_gap
    P, R = retarget.retarget(src)
    n_in = len(P["hips"])
    if origin is None and kind == "gltf":
        origin = "ref"
    P, rel, _ = retarget.to_clip(P, R, fps_in, fps, start, end, origin=origin)
    n = len(P["hips"])
    curls = None
    if "curls" in src:
        t = np.arange(n) / fps + start
        curls = src["curls"][np.clip(np.round(t * fps_in).astype(int), 0, n_in - 1)]
    speed = None
    if cycle:
        # One stride cycle, left heel strike to the next: a loop that starts
        # where every gait cycle starts (so walk and run share a phase).
        i, j = stride_cycle(P, fps)
        P = {k: v[i:j] for k, v in P.items()}
        # How fast a foot on the ground moves back under the hips: played in
        # place, that is the speed over the floor at which it stays put (the
        # hips' own travel says less: the canonical legs are not the
        # source's).
        speed = planted_speed(P, fps)
        rel = {k: v[i:j] for k, v in rel.items()}
        curls = None if curls is None else curls[i:j]
        loop, root = True, "in_place"
        n = len(P["hips"])
    q = {k: retarget.rot_to_quat(rel[k]) for k in sk.OUTPUT}
    frames = []
    for i in range(n):
        row = []
        for k in sk.OUTPUT:
            row += [round(float(x), 4) for x in P[k][i]] + [round(float(x), 4) for x in q[k][i]]
        frames.append(row)
    # Where the body ends up: the hips on the floor, and their yaw.
    ahead = rel["hips"][-1] @ np.array([0.0, 0.0, 1.0])
    end_pose = {
        "x": round(float(P["hips"][-1][0]), 4), "z": round(float(P["hips"][-1][2]), 4),
        "yaw_deg": round(float(np.degrees(np.arctan2(ahead[0], ahead[2]))), 1),
    }
    return {
        "name": name,
        "fps": fps,
        "loop": loop,
        # How the body moves with it: travel (it ends standing elsewhere),
        # in_place (its travel taken out), hold (as is; the next clip goes
        # on from here, e.g. lying down then lying).
        "root": root or ("in_place" if loop else "travel"),
        "source": os.path.basename(path),
        "license": license,
        "joints": sk.OUTPUT,
        "standing": {k: [round(float(x), 4) for x in sk.STANDING_P[k][0]] for k in sk.OUTPUT},
        "eyes": [float(x) for x in sk.EYES],
        "end": end_pose,
        "frames": frames,
        # A gait cycle's speed over the floor (statures a second): played in
        # place at the body's own speed, its feet stay put.
        "speed": None if speed is None else round(speed, 4),
        # Finger curls per frame, as changes from standing: the left hand's
        # then the right's, each little, ring, middle, index, thumb (+1:
        # from straight to a fist).
        "curls": None if curls is None else [[round(float(x), 2) for x in c.reshape(-1)] for c in curls],
    }, P


def planted_speed(P, fps):
    """The speed (statures a second) a planted foot moves back relative to
    the hips, over the frames where it is the lower foot and on the floor."""
    speeds = []
    for k, other in (("l_foot", "r_foot"), ("r_foot", "l_foot")):
        rel = P[k][:, 2] - P["hips"][:, 2]
        back = -(rel[1:] - rel[:-1]) * fps
        y, yo = P[k][:-1, 1], P[other][:-1, 1]
        planted = (y <= yo) & (y < y.min() + 0.02)
        if planted.sum() > 2:
            speeds.append(float(np.median(back[planted])))
    if not speeds:
        raise SystemExit("no planted foot in the cycle")
    return float(np.mean(speeds))


def stride_cycle(P, fps):
    """The frames of one stride cycle: from a left heel strike (the left
    ankle furthest ahead of the hips) to the next."""
    ahead = P["l_foot"][:, 2] - P["hips"][:, 2]
    w = max(2, int(fps * 0.15))
    peaks = [k for k in range(w, len(ahead) - w) if ahead[k] == ahead[k - w:k + w + 1].max()]
    if len(peaks) < 2:
        raise SystemExit("no two left heel strikes to cut a cycle between")
    # The middle of the take is steadiest.
    k = len(peaks) // 2 - 1 if len(peaks) > 2 else 0
    return peaks[k], peaks[k + 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("source")
    ap.add_argument("--kind", default="cmu", help="cmu, bandai (BVH), gltf (Quaternius)")
    ap.add_argument("--anim", default=None, help="gltf: the animation's name")
    ap.add_argument("--ref", type=int, default=None, help="a standing frame (else the rest pose, a T-pose)")
    ap.add_argument("--src-fps", type=float, default=None, help="the source's real frame rate")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--license", default="")
    ap.add_argument("--out", default="clips")
    ap.add_argument("--preview", action="store_true")
    a = ap.parse_args()
    clip, P = build(a.name, a.source, a.kind, a.start, a.end, a.fps, a.loop, a.license, a.ref, a.src_fps, a.anim)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, a.name + ".json")
    with open(path, "w") as f:
        json.dump(clip, f, separators=(",", ":"))
    print(f"{path}: {len(clip['frames'])} frames, {len(clip['frames']) / a.fps:.1f} s, end {clip['end']}")
    if a.preview:
        import preview
        preview.sheet(clip, os.path.join(a.out, a.name + ".png"))


if __name__ == "__main__":
    main()
