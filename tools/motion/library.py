"""The bot's motion library: which source, which part, under what licence.

    python library.py DATA_DIR OUT_DIR [--preview] [--only NAME,...]

DATA_DIR holds the sources (none of them are in the repository):

- cmu/NN_NN.bvh: CMU Graphics Lab Motion Capture Database (free to use and
  modify, not to resell), BVH from github.com/una-dinosauria/cmu-mocap;
- bandai/dataset-1_*.bvh: Bandai Namco Research Motion Dataset (CC BY-NC
  4.0), github.com/BandaiNamcoResearchInc/Bandai-Namco-Research-Motiondataset;
- ual1.glb, ual2.glb: Quaternius Universal Animation Library 1 and 2,
  Standard (CC0), quaternius.com.

`fetch.py` downloads them. The clips go to OUT_DIR, to be copied to the
bridge's `motions/` (docs/full-vr/motion.md).
"""

import argparse
import json
import os

import convert
import keyframes
import postures

CMU = "CMU mocap (mocap.cs.cmu.edu; free to use, not to resell)"
BANDAI = "Bandai Namco Research Motion Dataset (CC BY-NC 4.0)"
QUAT = "Quaternius Universal Animation Library (CC0)"
# The feet of a gait no nearer than this (statures between the ankles;
# standing: 0.191).
GAIT_GAP = 0.15

# name: (kind, source, options)
CLIPS = {
    # Standing.
    "idle": ("gltf", "ual1.glb", {"anim": "Idle_Loop", "loop": True, "license": QUAT}),
    "idle_talk": ("gltf", "ual1.glb", {"anim": "Idle_Talking_Loop", "loop": True, "license": QUAT}),
    "fold_arms": ("gltf", "ual2.glb", {"anim": "Idle_FoldArms_Loop", "loop": True, "license": QUAT}),
    # Greeting, answering.
    "wave": ("cmu", "cmu/141_16.bvh", {"start": 0.25, "end": 1.9, "license": CMU}),
    "bye": ("bandai", "bandai/dataset-1_bye_childish_001.bvh", {"ref": 0, "license": BANDAI}),
    "byebye": ("bandai", "bandai/dataset-1_byebye_childish_001.bvh", {"ref": 0, "license": BANDAI}),
    "nod": ("gltf", "ual2.glb", {"anim": "Yes", "license": QUAT}),
    "shake_head": ("gltf", "ual2.glb", {"anim": "Idle_No_Loop", "license": QUAT}),
    "refuse": ("keys", "poses/refuse.json", {}),
    "think": ("keys", "poses/think.json", {}),
    "look_around": ("keys", "poses/look_around.json", {}),
    # The legs in the air (a jump: the bridge blends it in while not grounded).
    "jump_air": ("keys", "poses/jump_air.json", {"loop": True}),
    # Moving about.
    "turn": ("cmu", "cmu/69_16.bvh", {"start": 0.5, "end": 4.6, "license": CMU}),
    "backflip": ("cmu", "cmu/88_01.bvh", {"start": 0.6, "end": 3.1, "license": CMU}),
    "walk": ("bandai", "bandai/dataset-1_walk_childish_001.bvh", {"ref": 0, "license": BANDAI}),
    "run": ("bandai", "bandai/dataset-1_run_childish_001.bvh", {"ref": 0, "license": BANDAI}),
    # Gait cycles for the legs while the bot moves (the bridge plays them at
    # its own speed): one stride each, from the steady middle of the take.
    # (Walking in the normal style: the feminine one sets one foot before the
    # other like a catwalk, the childish one splays its feet and flails.)
    # (The feet at least GAIT_GAP apart: a gait brings them near the midline,
    # and the foot trackers sit inside the ankles.)
    "walk_cycle": ("bandai", "bandai/dataset-1_walk_normal_001.bvh",
                   {"ref": 0, "cycle": True, "min_gap": GAIT_GAP, "license": BANDAI}),
    "run_cycle": ("bandai", "bandai/dataset-1_run_feminine_001.bvh",
                  {"ref": 0, "cycle": True, "min_gap": GAIT_GAP, "license": BANDAI}),
    # Down and up again.
    # (One take each, cut in three sharing its origin: "hold" goes on into
    # the next; the getting up moves the body to where it stands up.)
    "lie_down": ("cmu", "cmu/113_08.bvh", {"start": 0.3, "end": 5.0, "origin": 0.3, "root": "hold", "exit": "get_up",
                                           "posture": "lying", "license": CMU}),
    "lying": ("cmu", "cmu/113_08.bvh", {"start": 5.0, "end": 9.0, "origin": 0.3, "loop": True, "root": "hold",
                                        "exit": "get_up", "posture": "lying", "license": CMU}),
    "get_up": ("cmu", "cmu/113_08.bvh", {"start": 9.0, "end": 14.5, "origin": 0.3, "license": CMU}),
    # Lying on the back, either side or the front, where the CMU lying is;
    # the sides and front roll back onto the back to get up.
    "lying_back": ("lying", "back", {"exit": "get_up"}),
    "lying_left": ("lying", "left", {"exit": "lying_back"}),
    "lying_right": ("lying", "right", {"exit": "lying_back"}),
    "lying_front": ("lying", "front", {"exit": "lying_back"}),
    "sit_down": ("gltf", "ual1.glb", {"anim": "Sitting_Enter", "root": "hold", "exit": "stand_up", "posture": "sitting",
                                         "license": QUAT}),
    "sitting": ("sit", "", {"exit": "stand_up"}),
    "stand_up": ("gltf", "ual1.glb", {"anim": "Sitting_Exit", "license": QUAT}),
    # Dancing.
    "dance": ("gltf", "ual1.glb", {"anim": "Dance_Loop", "loop": True, "license": QUAT}),
    "dance_short": ("bandai", "bandai/dataset-1_dance-short_normal_001.bvh", {"ref": 0, "license": BANDAI}),
    "chicken_dance": ("cmu", "cmu/143_34.bvh", {"start": 0.3, "license": CMU}),
}


def build_one(name, data, out, preview=False):
    kind, source, opts = CLIPS[name]
    opts = dict(opts)
    exit_clip = opts.pop("exit", None)
    posture = opts.pop("posture", None)
    here = os.path.dirname(os.path.abspath(__file__))
    if kind in ("sit", "lying"):
        if kind == "sit":
            spec = postures.sit_cross()
            posture = "sitting"
        else:
            # Goes on from the CMU lying (built first).
            along, at = postures.lying_from_clip(os.path.join(out, "lying.json"))
            spec = postures.lying(source, along, at)
            posture = "lying"
        spec["name"] = name
        clip = keyframes.build(spec)
        clip["root"] = "hold"
    elif kind == "keys":
        spec = json.load(open(os.path.join(here, source), encoding="utf-8"))
        spec["name"] = name
        spec["loop"] = opts.get("loop", spec.get("loop", False))
        clip = keyframes.build(spec)
        clip["root"] = opts.get("root", "in_place" if spec["loop"] else "travel")
    else:
        o = dict(opts)
        clip, _ = convert.build(name, os.path.join(data, source), kind, o.pop("start", 0.0), o.pop("end", None),
                                30.0, o.pop("loop", False), o.pop("license", ""), o.pop("ref", None),
                                o.pop("src_fps", None), o.pop("anim", None), o.pop("origin", None),
                                o.pop("root", None), o.pop("cycle", False), o.pop("min_gap", None))
    if posture:
        # A program that holds this posture goes on into another clip of it
        # directly (lying on the back, then on a side); into anything else
        # by way of the exit.
        clip["posture"] = posture
    if exit_clip:
        # The clip that leaves this posture (the player plays it when a
        # program that stays here is stopped or followed by another).
        clip["exit"] = exit_clip
    path = os.path.join(out, name + ".json")
    json.dump(clip, open(path, "w"), separators=(",", ":"))
    if preview:
        import preview as pv
        pv.sheet(clip, os.path.join(out, name + ".png"))
    return len(clip["frames"]) / clip["fps"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("out")
    ap.add_argument("--preview", action="store_true")
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    names = [n for n in a.only.split(",") if n] or list(CLIPS)
    for n in names:
        try:
            print(f"{n}: {build_one(n, a.data, a.out, a.preview):.1f} s")
        except (OSError, SystemExit, KeyError) as e:
            print(f"{n}: FAILED {e}")


if __name__ == "__main__":
    main()
