"""The canonical skeleton every motion is retargeted onto.

Units are the stature (1 = the body's height); axes are the body's: x to its
left, y up, z ahead (right-handed, the CMU convention). Proportions are
Drillis & Contini's, as the trackers' calibration pose
(`crates/vrc-vr/src/trackers.rs`): the eyes at 0.936, the hip joints at
0.530, the knees at 0.285, the ankles at 0.039.

The rest pose is a T-pose facing +z. `STANDING` turns the arms down to the
sides, palms in: the pose the trackers were calibrated in and the hands'
rest grip was tuned for. Clips store each joint's rotation relative to its
rotation in `STANDING`, so a clip standing still reproduces the
calibration exactly.
"""

import numpy as np

# name: (parent, offset from the parent in the rest pose)
JOINTS = {
    "hips": (None, (0.0, 0.530, 0.0)),
    "spine": ("hips", (0.0, 0.060, 0.0)),
    "chest": ("spine", (0.0, 0.130, 0.0)),
    "neck": ("chest", (0.0, 0.098, 0.0)),
    "head": ("neck", (0.0, 0.050, 0.0)),
    "head_top": ("head", (0.0, 0.132, 0.0)),
    "l_clavicle": ("chest", (0.020, 0.090, 0.0)),
    "l_arm": ("l_clavicle", (0.109, 0.008, 0.0)),
    "l_forearm": ("l_arm", (0.186, 0.0, 0.0)),
    "l_hand": ("l_forearm", (0.146, 0.0, 0.0)),
    "l_fingers": ("l_hand", (0.108, 0.0, 0.0)),
    "r_clavicle": ("chest", (-0.020, 0.090, 0.0)),
    "r_arm": ("r_clavicle", (-0.109, 0.008, 0.0)),
    "r_forearm": ("r_arm", (-0.186, 0.0, 0.0)),
    "r_hand": ("r_forearm", (-0.146, 0.0, 0.0)),
    "r_fingers": ("r_hand", (-0.108, 0.0, 0.0)),
    "l_upleg": ("hips", (0.0955, 0.0, 0.0)),
    "l_leg": ("l_upleg", (0.0, -0.245, 0.0)),
    "l_foot": ("l_leg", (0.0, -0.246, 0.0)),
    "l_toe": ("l_foot", (0.0, -0.039, 0.110)),
    "r_upleg": ("hips", (-0.0955, 0.0, 0.0)),
    "r_leg": ("r_upleg", (0.0, -0.245, 0.0)),
    "r_foot": ("r_leg", (0.0, -0.246, 0.0)),
    "r_toe": ("r_foot", (0.0, -0.039, 0.110)),
}
NAMES = list(JOINTS)
# The joint each rotation is aimed by (the bone it turns), and the joints
# that have no rotation of their own (end points).
AIM = {
    "hips": "spine", "spine": "chest", "chest": "neck", "neck": "head", "head": "head_top",
    "l_clavicle": "l_arm", "l_arm": "l_forearm", "l_forearm": "l_hand", "l_hand": "l_fingers",
    "r_clavicle": "r_arm", "r_arm": "r_forearm", "r_forearm": "r_hand", "r_hand": "r_fingers",
    "l_upleg": "l_leg", "l_leg": "l_foot", "l_foot": "l_toe",
    "r_upleg": "r_leg", "r_leg": "r_foot", "r_foot": "r_toe",
}
ENDS = ("head_top", "l_fingers", "r_fingers", "l_toe", "r_toe")
# The eyes, from the head joint (rest).
EYES = np.array([0.0, 0.068, 0.045])
# What a clip stores: the tracked points and what drives the headset and hands.
OUTPUT = ["hips", "chest", "head", "l_leg", "r_leg", "l_foot", "r_foot",
          "l_forearm", "r_forearm", "l_hand", "r_hand"]


def axis_angle(axis, deg):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    a = np.radians(deg)
    x, y, z = axis
    c, s, t = np.cos(a), np.sin(a), 1 - np.cos(a)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def fk(local_rot, root_pos):
    """Global positions and rotations of the canonical skeleton from each
    joint's rotation relative to its parent (`local_rot[name]`: (n, 3, 3)
    or missing for identity) and the root's position (n, 3)."""
    n = len(root_pos)
    P, R = {}, {}
    for name in NAMES:
        parent, off = JOINTS[name]
        lr = local_rot.get(name)
        if lr is None:
            lr = np.tile(np.eye(3), (n, 1, 1))
        if parent is None:
            P[name] = np.asarray(root_pos, float)
            R[name] = lr
        else:
            P[name] = P[parent] + np.einsum("nij,j->ni", R[parent], np.asarray(off))
            R[name] = R[parent] @ lr
    return P, R


def fk_global(global_rot, root_pos):
    """Positions from each joint's global rotation (rest pose = identity)."""
    P = {}
    for name in NAMES:
        parent, off = JOINTS[name]
        if parent is None:
            P[name] = np.asarray(root_pos, float)
        else:
            P[name] = P[parent] + np.einsum("nij,j->ni", global_rot[parent], np.asarray(off))
    return P


def standing():
    """The calibration pose: arms down at the sides (about 80 degrees from
    the T-pose, the hands a little away from the thighs), palms in."""
    rot = {
        "l_arm": axis_angle((0, 0, 1), -80.0)[None],
        "r_arm": axis_angle((0, 0, 1), 80.0)[None],
    }
    return fk(rot, np.array([[0.0, 0.530, 0.0]]))


STANDING_P, STANDING_R = standing()
