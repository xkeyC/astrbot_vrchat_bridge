"""Motion sources, each read into what retarget.retarget takes."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mocap"))
import bvhfk  # noqa: E402

# Canonical joint -> BVH joint, for the CMU skeleton (and BVH files that
# name joints the same way, e.g. converted Mixamo-like rigs with these names).
CMU = {
    "hips": "Hips", "spine": "Spine", "chest": "Spine1", "neck": "Neck1", "head": "Head",
    "head_top": "Head_end",
    "l_clavicle": "LeftShoulder", "l_arm": "LeftArm", "l_forearm": "LeftForeArm",
    "l_hand": "LeftHand", "l_fingers": "LeftHandIndex1",
    "r_clavicle": "RightShoulder", "r_arm": "RightArm", "r_forearm": "RightForeArm",
    "r_hand": "RightHand", "r_fingers": "RightHandIndex1",
    "l_upleg": "LeftUpLeg", "l_leg": "LeftLeg", "l_foot": "LeftFoot", "l_toe": "LeftToeBase",
    "r_upleg": "RightUpLeg", "r_leg": "RightLeg", "r_foot": "RightFoot", "r_toe": "RightToeBase",
}


# Bandai Namco Research Motion Dataset 1/2 (CC BY-NC 4.0): no rest pose to
# speak of (every bone along its x), so a standing frame is the reference.
BANDAI = {
    "hips": "Hips", "spine": "Spine", "chest": "Chest", "neck": "Neck", "head": "Head",
    "head_top": "Head_end",
    "l_clavicle": "Shoulder_L", "l_arm": "UpperArm_L", "l_forearm": "LowerArm_L",
    "l_hand": "Hand_L", "l_fingers": "Hand_L_end",
    "r_clavicle": "Shoulder_R", "r_arm": "UpperArm_R", "r_forearm": "LowerArm_R",
    "r_hand": "Hand_R", "r_fingers": "Hand_R_end",
    "l_upleg": "UpperLeg_L", "l_leg": "LowerLeg_L", "l_foot": "Foot_L", "l_toe": "Toes_L",
    "r_upleg": "UpperLeg_R", "r_leg": "LowerLeg_R", "r_foot": "Foot_R", "r_toe": "Toes_R",
}
MAPPINGS = {"cmu": CMU, "bandai": BANDAI}


# CMU trials captured at 60 fps though their BVH say 120 (from the index).
CMU_60FPS = {line.strip() for line in open(os.path.join(os.path.dirname(__file__), "cmu_60fps.txt"))
             if line.strip() and not line.startswith("#")}


def bvh(path, mapping=CMU, fps=None, ref=None):
    """A BVH file: the source dict and its frame rate (`fps` overrides the
    file's; CMU's 60 fps trials are known). `ref`: None when the rest pose
    is a T-pose (CMU), else a frame where the body stands."""
    joints, data, ft = bvhfk.parse(path)
    trial = os.path.splitext(os.path.basename(path))[0]
    if fps:
        ft = 1.0 / fps
    elif mapping is CMU and trial in CMU_60FPS:
        ft = 1.0 / 60.0
    P, R = bvhfk.fk(joints, data)
    P0, _ = bvhfk.fk(joints, data, zero=True)
    hips, foot = mapping["hips"], mapping["l_foot"]
    rest = {k: v[0] for k, v in P0.items()}
    # Hips above the floor when standing: hips to ankle, plus the ankle's
    # share (0.039 of 0.530) of the canonical body.
    stand = rest if ref is None else {k: v[ref] for k, v in P.items()}
    leg = stand[hips][1] - stand[foot][1]
    src = {
        "map": mapping,
        "ref": ref,
        "rest": rest,
        "pos": P,
        "rot": {k: R.get(k, R.get(k.removesuffix("_end"))) for k in P},
        "hip_height": leg * 0.530 / (0.530 - 0.039),
    }
    return src, 1.0 / ft


# Quaternius Universal Animation Library (CC0): UAL1 (Rigify DEF-* bones)
# and UAL2 (Unreal-like names). Both have fingers.
QUAT1 = {
    "hips": "DEF-hips", "spine": "DEF-spine.001", "chest": "DEF-spine.003", "neck": "DEF-neck", "head": "DEF-head",
    "head_top": "DEF-head@end",
    "l_clavicle": "DEF-shoulder.L", "l_arm": "DEF-upper_arm.L", "l_forearm": "DEF-forearm.L",
    "l_hand": "DEF-hand.L", "l_fingers": "DEF-f_middle.01.L",
    "r_clavicle": "DEF-shoulder.R", "r_arm": "DEF-upper_arm.R", "r_forearm": "DEF-forearm.R",
    "r_hand": "DEF-hand.R", "r_fingers": "DEF-f_middle.01.R",
    "l_upleg": "DEF-thigh.L", "l_leg": "DEF-shin.L", "l_foot": "DEF-foot.L", "l_toe": "DEF-toe.L",
    "r_upleg": "DEF-thigh.R", "r_leg": "DEF-shin.R", "r_foot": "DEF-foot.R", "r_toe": "DEF-toe.R",
}
QUAT1_FINGERS = {
    side: [[f"DEF-{f}.0{i}.{side}" for i in (1, 2, 3)] + [f"DEF-{f}.03.{side}@end"]
           for f in ("f_pinky", "f_ring", "f_middle", "f_index", "thumb")]
    for side in ("L", "R")
}
QUAT2 = {
    "hips": "pelvis", "spine": "spine_01", "chest": "spine_03", "neck": "neck_01", "head": "Head",
    "head_top": "Head@end",
    "l_clavicle": "clavicle_l", "l_arm": "upperarm_l", "l_forearm": "lowerarm_l",
    "l_hand": "hand_l", "l_fingers": "middle_01_l",
    "r_clavicle": "clavicle_r", "r_arm": "upperarm_r", "r_forearm": "lowerarm_r",
    "r_hand": "hand_r", "r_fingers": "middle_01_r",
    "l_upleg": "thigh_l", "l_leg": "calf_l", "l_foot": "foot_l", "l_toe": "ball_l",
    "r_upleg": "thigh_r", "r_leg": "calf_r", "r_foot": "foot_r", "r_toe": "ball_r",
}
QUAT2_FINGERS = {
    side: [[f"{f}_0{i}_{side.lower()}" for i in (1, 2, 3)] + [f"{f}_04_leaf_{side.lower()}"]
           for f in ("pinky", "ring", "middle", "index", "thumb")]
    for side in ("L", "R")
}


def _quat_mat(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


def _sample_gltf(g, anim, times):
    """Every node's local translation, rotation (3x3) and scale at `times`."""
    from scipy.spatial.transform import Rotation, Slerp
    blob = g.binary_blob()

    def read(accessor_index):
        a = g.accessors[accessor_index]
        bv = g.bufferViews[a.bufferView]
        comps = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[a.type]
        start = (bv.byteOffset or 0) + (a.byteOffset or 0)
        stride = bv.byteStride or comps * 4
        rows = [np.frombuffer(blob, dtype=np.float32, count=comps, offset=start + k * stride) for k in range(a.count)]
        return np.array(rows, dtype=float)

    n = len(times)
    T, R, S = {}, {}, {}
    for i, node in enumerate(g.nodes):
        T[i] = np.tile(node.translation or [0.0, 0.0, 0.0], (n, 1)).astype(float)
        R[i] = np.tile(_quat_mat(node.rotation or [0.0, 0.0, 0.0, 1.0]), (n, 1, 1))
        S[i] = np.tile(node.scale or [1.0, 1.0, 1.0], (n, 1)).astype(float)
    for ch in anim.channels:
        smp = anim.samplers[ch.sampler]
        t_in = read(smp.input)[:, 0]
        v = read(smp.output)
        if smp.interpolation == "CUBICSPLINE":
            v = v.reshape(len(t_in), 3, -1)[:, 1]
        tt = np.clip(times, t_in[0], t_in[-1])
        node = ch.target.node
        if ch.target.path == "rotation":
            if len(t_in) == 1:
                R[node] = np.tile(_quat_mat(v[0]), (n, 1, 1))
            else:
                R[node] = Slerp(t_in, Rotation.from_quat(v))(tt).as_matrix()
        elif ch.target.path in ("translation", "scale"):
            out = np.stack([np.interp(tt, t_in, v[:, k]) for k in range(3)], axis=1)
            (T if ch.target.path == "translation" else S)[node] = out
    return T, R, S


def _world(g, T, R, S):
    """Global positions and rotations of every named node, plus `NAME@end`
    points for bones with no child (one bone length on along the bone's y,
    glTF bones' axis)."""
    parent = {}
    for i, node in enumerate(g.nodes):
        for c in node.children or []:
            parent[c] = i
    P, Rw, Sw = {}, {}, {}

    def solve(i):
        if i in P:
            return
        if i in parent:
            p = parent[i]
            solve(p)
            P[i] = P[p] + np.einsum("nij,nj->ni", Rw[p], T[i] * Sw[p])
            Rw[i] = Rw[p] @ R[i]
            Sw[i] = Sw[p] * S[i]
        else:
            P[i], Rw[i], Sw[i] = T[i], R[i], S[i]

    for i in range(len(g.nodes)):
        solve(i)
    names = {g.nodes[i].name: i for i in range(len(g.nodes))}
    pos = {k: P[i] for k, i in names.items()}
    rot = {k: Rw[i] for k, i in names.items()}
    for k, i in names.items():
        if g.nodes[i].children or i not in parent:
            continue
        length = np.linalg.norm(T[i][0] * Sw[parent[i]][0])
        if k in ("DEF-head", "Head"):
            length = max(length, 0.2)
        pos[k + "@end"] = P[i] + np.einsum("nij,j->ni", Rw[i], [0.0, length, 0.0])
        rot[k + "@end"] = Rw[i]
    return pos, rot


def finger_curls(pos, fingers):
    """Curl per finger (little, ring, middle, index, thumb; 0 straight, 1 a
    fist) from how much each finger's segments bend, per frame."""
    out = []
    for k, chain in enumerate(fingers):
        pts = [pos[j] for j in chain if j in pos]
        segs = [pts[i + 1] - pts[i] for i in range(len(pts) - 1)]
        bend = np.zeros(len(pts[0]))
        for a, b in zip(segs, segs[1:]):
            cos = np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) + 1e-9)
            bend += np.degrees(np.arccos(np.clip(cos, -1, 1)))
        full = 100.0 if k == 4 else 160.0
        out.append(bend / full)
    return np.stack(out, axis=1)


def gltf(path, anim_name, ref_anim="Idle_Loop", fps=60.0):
    """An animation of a glTF file with a Quaternius rig (UAL1 or UAL2): the
    source dict (with `curls`: (n, 2, 5), changes from the reference) and
    its frame rate. The reference
    is the first frame of `ref_anim` (standing)."""
    import pygltflib
    g = pygltflib.GLTF2().load(path)
    anims = {a.name: a for a in g.animations}
    if anim_name not in anims:
        raise SystemExit(f"no animation {anim_name}; there are {sorted(anims)}")
    names = {nd.name for nd in g.nodes}
    mapping, fingers = (QUAT1, QUAT1_FINGERS) if "DEF-hips" in names else (QUAT2, QUAT2_FINGERS)
    if ref_anim not in anims:
        ref_anim = "Idle_FoldArms_Loop" if "Idle_FoldArms_Loop" in anims else anim_name
    a = anims[anim_name]
    end = max(float(np.max(g.accessors[s.input].max or [0.0])) for s in a.samplers)
    times = np.arange(0.0, end + 1e-9, 1.0 / fps)
    pos, rot = _world(g, *_sample_gltf(g, a, times))
    rpos, rrot = _world(g, *_sample_gltf(g, anims[ref_anim], np.array([0.0])))
    leg = rpos[mapping["hips"]][0, 1] - rpos[mapping["l_foot"]][0, 1]
    # As changes from the standing reference: the player adds them to its
    # own relaxed hand (this rig stands with its fingers half curled).
    curls = np.stack([finger_curls(pos, fingers["L"]) - finger_curls(rpos, fingers["L"]),
                      finger_curls(pos, fingers["R"]) - finger_curls(rpos, fingers["R"])], axis=1)
    src = {
        "map": mapping,
        "ref": "pose",
        "ref_pos": {k: v[0] for k, v in rpos.items()},
        "ref_rot": {k: v[0] for k, v in rrot.items()},
        "pos": pos,
        "rot": rot,
        "hip_height": leg * 0.530 / (0.530 - 0.039),
        "curls": curls,
    }
    return src, fps
