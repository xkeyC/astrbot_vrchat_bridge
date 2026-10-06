"""Retargets a source skeleton's motion onto the canonical one (skeleton.py).

A source gives, per frame, its joints' global positions and global
rotations (any rest pose, any facing, y up), and a reference: either its
rest pose, when that is a T-pose, or a frame where it stands naturally
(arms down). The reference is matched to the canonical T-pose or the
calibration pose (`STANDING`):

- the source is turned (and mirrored if its sides come out swapped) so that
  at the reference it faces +z with x its left;
- each canonical bone is turned to point where the matching source bone
  points, its twist following the source joint's rotation since the
  reference; bones the source leaves without length (a hand or head with no
  end) take the source joint's rotation change as is;
- the canonical skeleton's own proportions give the positions.

So every source plays on the same body, the one the trackers were calibrated
for.
"""

import numpy as np

import skeleton as sk


def unit(v):
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def frame(u, hint):
    """Orthonormal frames (columns: u, v, w) with u along `u` and v as close
    to `hint` as allowed; works per row."""
    u = unit(u)
    v = hint - np.sum(hint * u, axis=-1, keepdims=True) * u
    bad = np.linalg.norm(v, axis=-1) < 1e-6
    if np.any(bad):
        alt = np.broadcast_to(np.array([0.0, 1.0, 0.0]), u.shape).copy()
        v[bad] = alt[bad] - np.sum(alt[bad] * u[bad], axis=-1, keepdims=True) * u[bad]
    v = unit(v)
    w = np.cross(u, v)
    return np.stack([u, v, w], axis=-1)


def hint_for(name):
    """The axis that fixes a bone's twist at the reference: ahead (+z) for
    most, up for the feet (they point ahead)."""
    return np.array([0.0, 1.0, 0.0]) if name.endswith("_foot") else np.array([0.0, 0.0, 1.0])


def normalize(src):
    """Turns (and mirrors, if need be) the source so that at its reference it
    faces +z with x its left; returns the turned positions, rotations and
    reference positions/rotations."""
    m, ref = src["map"], src["ref"]
    pos, rot = src["pos"], src["rot"]
    if ref is None:
        rp = {k: np.asarray(v, float) for k, v in src["rest"].items()}
        rr = {k: np.eye(3) for k in rot}
    elif ref == "pose":
        rp, rr = src["ref_pos"], src["ref_rot"]
    else:
        rp = {k: v[ref] for k, v in pos.items()}
        rr = {k: v[ref] for k, v in rot.items()}
    left = rp[m["l_upleg"]] - rp[m["r_upleg"]]
    toes = (rp[m["l_toe"]] + rp[m["r_toe"]]) / 2 - (rp[m["l_foot"]] + rp[m["r_foot"]]) / 2
    left[1] = toes[1] = 0.0
    ahead = np.cross(left, [0.0, 1.0, 0.0])
    mirror = np.dot(ahead, toes) < 0
    M = np.diag([-1.0, 1.0, 1.0]) if mirror else np.eye(3)
    if mirror:
        left = M @ left
        ahead = np.cross(left, [0.0, 1.0, 0.0])
    yaw = np.arctan2(ahead[0], ahead[2])
    T = sk.axis_angle((0, 1, 0), -np.degrees(yaw)) @ M
    # A mirror turns a rotation R into M R M.
    P = {k: v @ T.T for k, v in pos.items()}
    R = {k: T @ v @ M for k, v in rot.items()}
    RP = {k: T @ v for k, v in rp.items()}
    RR = {k: T @ v @ M for k, v in rr.items()}
    return P, R, RP, RR, bool(mirror)


def retarget(src):
    """`src`: dict with
    - `map`: canonical joint -> source joint (every key of skeleton.AIM and
      its aim; ends may map to end sites),
    - `pos`: source joint -> (n, 3) global positions,
    - `rot`: source joint -> (n, 3, 3) global rotations (any rest),
    - `ref`: None (the rest pose is a T-pose, given in `rest`: joint ->
      position) or a frame index where the body stands, arms down,
    - `hip_height`: the source's hips above its floor when standing.
    Returns canonical global positions and rotations (per joint, n frames),
    rotations relative to the canonical T-pose."""
    m = src["map"]
    P, R_src, RP, RR, _ = normalize(src)
    canon = "tpose" if src["ref"] is None else "standing"
    R = {}
    for j, c in sk.AIM.items():
        sj, sc = m[j], m[c]
        h = hint_for(j)
        if canon == "tpose":
            d_c = np.asarray(sk.JOINTS[c][1], float)
            r_c = np.eye(3)
        else:
            d_c = sk.STANDING_P[c][0] - sk.STANDING_P[j][0]
            r_c = sk.STANDING_R[j][0]
        f_c = frame(d_c[None], h[None])[0]
        d_s = RP[sc] - RP[sj]
        # The source joint's rotation since the reference.
        delta = R_src[sj] @ RR[sj].T
        if np.linalg.norm(d_s) < 1e-6:
            # No bone to aim with: the rotation change as is.
            R[j] = delta @ r_c
            continue
        f_s = frame(d_s[None], h[None])[0]
        u = P[sc] - P[sj]
        v = np.einsum("nij,j->ni", delta, f_s[:, 1])
        R[j] = frame(u, v) @ f_c.T @ r_c
    # The feet: their direction (ankle to toe) from the source, but pitched
    # as the source's foot is pitched on the ground when it stands on it
    # (its foot bones slope their own way), and rolled with the shin. VRChat
    # sets the feet, and bends the knees, by the foot trackers' axes.
    toe0 = np.asarray(sk.JOINTS["l_toe"][1], float)
    flat = np.arcsin(toe0[1] / np.linalg.norm(toe0))
    f0 = frame(toe0[None], np.array([[0.0, 1.0, 0.0]]))[0]
    for side in ("l", "r"):
        foot, toe, knee = m[f"{side}_foot"], m[f"{side}_toe"], m[f"{side}_leg"]
        u = unit(P[toe] - P[foot])
        pitch = np.arcsin(np.clip(u[:, 1], -1, 1))
        low = P[foot][:, 1]
        stance = low <= np.percentile(low, 30)
        # Toes from 80 degrees down to 45 up, never past the vertical.
        p = np.clip(pitch - np.median(pitch[stance]) + flat, -np.radians(80), np.radians(45))
        # Heading: where the hips face, the foot turned from it at most 40
        # degrees (a foot pitched steeply has no heading of its own).
        ahead = np.einsum("nij,j->ni", R["hips"], [0.0, 0.0, 1.0])
        body = np.arctan2(ahead[:, 0], ahead[:, 2])
        own = np.arctan2(u[:, 0], u[:, 2])
        turn = (own - body + np.pi) % (2 * np.pi) - np.pi
        steep = np.hypot(u[:, 0], u[:, 2]) < 0.35
        # Without the source's own toe-out (its median on the ground), half
        # the rest, a touch in (cuter than splayed feet).
        flat_turn = turn[stance & ~steep]
        toe_out = np.median(flat_turn) if len(flat_turn) else 0.0
        toe_in = np.radians(TOE_IN_DEG) * (-1.0 if side == "l" else 1.0)
        turn = (turn - toe_out) * 0.5 + toe_in
        turn = np.where(steep, toe_in, np.clip(turn, -np.radians(30), np.radians(30)))
        yaw = body + turn
        d = np.stack([np.sin(yaw) * np.cos(p), np.sin(p), np.cos(yaw) * np.cos(p)], axis=1)
        R[f"{side}_foot"] = frame(d, unit(P[knee] - P[foot])) @ f0.T
    narrow_stance(R, src.get("stance", STANCE))
    if src.get("min_gap"):
        widen_stance(R, src["min_gap"] / 2.0)
    for e in sk.ENDS:
        R[e] = R[sk.JOINTS[e][0]]
    scale = sk.JOINTS["hips"][1][1] / src["hip_height"]
    root = P[m["hips"]] * scale
    P = sk.fk_global(R, root)
    # Where the hips are at the reference (faces +z there).
    P["@ref_hips"] = RP[m["hips"]] * scale
    return P, R


# The feet turned in this much (degrees) when they stand.
TOE_IN_DEG = 2.0

# How much of a thigh's spread out to the side is kept (the sources stand
# and move with their feet wider than a cute avatar should).
STANCE = 0.2


def narrow_stance(R, keep):
    """Turns each whole leg about its hip joint toward the body's midline,
    keeping `keep` of the angle its ankle is out to the side (in the hips'
    front plane)."""
    if keep >= 1.0:
        return
    for leg, side in (("l", 1.0), ("r", -1.0)):
        chain = [f"{leg}_upleg", f"{leg}_leg", f"{leg}_foot"]
        h = R["hips"]
        # Hip to ankle, in the hips' frame.
        v = np.zeros((len(h), 3))
        for j, child in (("upleg", "leg"), ("leg", "foot")):
            off = np.asarray(sk.JOINTS[f"{leg}_{child}"][1], float)
            v += np.einsum("nij,j->ni", R[f"{leg}_{j}"], off)
        vh = np.einsum("nji,nj->ni", h, v)
        r = np.hypot(vh[:, 0], vh[:, 1])
        a = np.arctan2(side * vh[:, 0], -vh[:, 1])
        for i in np.nonzero(a > 0)[0]:
            a2 = a[i] * keep
            target = np.array([side * np.sin(a2) * r[i], -np.cos(a2) * r[i], vh[i, 2]])
            q = h[i] @ rotation_between(vh[i], target) @ h[i].T
            for j in chain:
                R[j][i] = q @ R[j][i]


def widen_stance(R, half):
    """Turns each whole leg about its hip joint out to the side wherever
    its ankle comes nearer the body's midline than `half` (statures; in the
    hips' front plane): a gait's feet nearly touch, as a body's do."""
    for leg, side in (("l", 1.0), ("r", -1.0)):
        chain = [f"{leg}_upleg", f"{leg}_leg", f"{leg}_foot"]
        h = R["hips"]
        hip_x = abs(sk.JOINTS[f"{leg}_upleg"][1][0])
        v = np.zeros((len(h), 3))
        for j, child in (("upleg", "leg"), ("leg", "foot")):
            off = np.asarray(sk.JOINTS[f"{leg}_{child}"][1], float)
            v += np.einsum("nij,j->ni", R[f"{leg}_{j}"], off)
        vh = np.einsum("nji,nj->ni", h, v)
        r = np.hypot(vh[:, 0], vh[:, 1])
        out_now = hip_x + side * vh[:, 0]
        for i in np.nonzero(out_now < half)[0]:
            want = np.clip((half - hip_x) / max(r[i], 1e-6), -1.0, 1.0)
            target = np.array([side * want * r[i], -np.sqrt(1 - want * want) * r[i], vh[i, 2]])
            q = h[i] @ rotation_between(vh[i], target) @ h[i].T
            for j in chain:
                R[j][i] = q @ R[j][i]


def rotation_between(a, b):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


# The body's thickness round each tracked joint (stature units): nothing goes
# into the floor deeper than that (lying, sitting on it, rolling).
RADII = {"hips": 0.07, "chest": 0.08, "head": 0.07, "l_leg": 0.035, "r_leg": 0.035, "l_foot": 0.035,
         "r_foot": 0.035, "l_forearm": 0.035, "r_forearm": 0.035, "l_hand": 0.03, "r_hand": 0.03}


def keep_above_floor(P, fps):
    """Lifts the whole body, frame by frame (smoothly), wherever a joint
    would be in the floor deeper than its thickness."""
    n = len(P["hips"])
    need = np.zeros(n)
    for j, r in RADII.items():
        if j in P:
            need = np.maximum(need, r - P[j][:, 1])
    need = np.maximum(need, 0.0)
    if not need.any():
        return
    # Spread each lift a third of a second either way, then smooth it.
    w = max(1, int(fps / 3))
    padded = np.pad(need, w, mode="edge")
    spread = np.array([padded[i:i + 2 * w + 1].max() for i in range(n)])
    kernel = np.hanning(2 * w + 1)
    kernel /= kernel.sum()
    lift = np.convolve(np.pad(spread, w, mode="edge"), kernel, mode="valid")
    lift = np.maximum(lift, need)
    for j in P:
        P[j][:, 1] += lift


def to_clip(P, R, fps_in, fps_out=30.0, start=0.0, end=None, ground="source", origin=None):
    """Cuts and resamples, puts the origin (on the floor under the hips,
    facing +z) at the cut's start (`origin` None), at `origin` seconds into
    the source (clips cut from one take share it) or at the reference
    (`"ref"`), puts the feet on the floor, and expresses rotations relative
    to STANDING: the clip's frames (dict joint -> positions (n, 3),
    rotations (n, 3, 3)) and the origin's yaw (radians)."""
    ref_hips = P.pop("@ref_hips")
    n_in = len(P["hips"])
    # The source's floor, from the whole take (so clips cut from one take
    # agree): frames standing (the hips up, near their standing height
    # above the lower ankle) say best where it is; else the lowest ankles.
    low = np.minimum(P["l_foot"][:, 1], P["r_foot"][:, 1])
    legs = P["hips"][:, 1] - low
    standing = legs > 0.9 * (sk.STANDING_P["hips"][0, 1] - sk.STANDING_P["l_foot"][0, 1])
    floor = np.percentile(low[standing], 10) if standing.sum() >= 5 else np.percentile(low, 10)
    t_in = np.arange(n_in) / fps_in
    end = t_in[-1] if end is None else min(end, t_in[-1])
    t = np.arange(start, end + 1e-9, 1.0 / fps_out)
    idx = np.clip(np.round(t * fps_in).astype(int), 0, n_in - 1)
    if origin == "ref":
        yaw, at = 0.0, np.asarray(ref_hips, float).copy()
    else:
        o = idx[0] if origin is None else int(np.clip(round(origin * fps_in), 0, n_in - 1))
        # Facing there: the hips' ahead on the floor (relative to standing,
        # which faces +z).
        ahead = (R["hips"][o] @ np.linalg.inv(sk.STANDING_R["hips"][0])) @ np.array([0.0, 0.0, 1.0])
        yaw = np.arctan2(ahead[0], ahead[2])
        at = P["hips"][o].copy()
    P = {k: v[idx] for k, v in P.items()}
    R = {k: v[idx] for k, v in R.items()}
    turn = sk.axis_angle((0, 1, 0), -np.degrees(yaw))
    origin = at
    origin[1] = 0.0
    for k in P:
        P[k] = (P[k] - origin) @ turn.T
        R[k] = turn @ R[k]
    if ground == "first":
        low = min(P["l_foot"][0, 1], P["r_foot"][0, 1])
        lift = sk.STANDING_P["l_foot"][0, 1] - low
    else:
        lift = sk.STANDING_P["l_foot"][0, 1] - floor
    for k in P:
        P[k][:, 1] += lift
    keep_above_floor(P, fps_out)
    rel = {k: R[k] @ np.linalg.inv(sk.STANDING_R[k][0]) for k in R}
    return P, rel, yaw


def rot_to_quat(R):
    """(n, 3, 3) -> (n, 4) x, y, z, w."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()
