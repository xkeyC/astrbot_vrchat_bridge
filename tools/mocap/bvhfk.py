import numpy as np

UNIT_M = 0.0254 / 0.45  # CMU ASF/BVH unit -> meters


class Joint:
    def __init__(self, name, parent):
        self.name, self.parent = name, parent
        self.offset = np.zeros(3)
        self.channels = []
        self.end = None  # end-site offset
        self.ch0 = 0


def parse(path):
    toks = open(path).read().split()
    i = 0
    joints, stack = [], []
    cur = None
    nch = 0
    while toks[i] != 'MOTION':
        t = toks[i]
        if t in ('ROOT', 'JOINT'):
            cur = Joint(toks[i + 1], stack[-1] if stack else None)
            joints.append(cur)
            i += 2
        elif t == 'End':
            # End Site { OFFSET x y z }
            cur.end = np.array([float(x) for x in toks[i + 4:i + 7]])
            i += 8
            continue
        elif t == '{':
            stack.append(cur)
            i += 1
        elif t == '}':
            stack.pop()
            cur = stack[-1] if stack else None
            i += 1
        elif t == 'OFFSET':
            cur.offset = np.array([float(x) for x in toks[i + 1:i + 4]])
            i += 4
        elif t == 'CHANNELS':
            n = int(toks[i + 1])
            cur.channels = toks[i + 2:i + 2 + n]
            cur.ch0 = nch
            nch += n
            i += 2 + n
        else:
            i += 1
    nf = int(toks[i + 2])
    ft = float(toks[i + 5])
    data = np.array(toks[i + 6:i + 6 + nf * nch], dtype=float).reshape(nf, nch)
    return joints, data, ft


def rot(axis, ang):
    c, s = np.cos(ang), np.sin(ang)
    n = len(ang)
    R = np.zeros((n, 3, 3))
    if axis == 'X':
        R[:, 0, 0] = 1; R[:, 1, 1] = c; R[:, 1, 2] = -s; R[:, 2, 1] = s; R[:, 2, 2] = c
    elif axis == 'Y':
        R[:, 1, 1] = 1; R[:, 0, 0] = c; R[:, 0, 2] = s; R[:, 2, 0] = -s; R[:, 2, 2] = c
    else:
        R[:, 2, 2] = 1; R[:, 0, 0] = c; R[:, 0, 1] = -s; R[:, 1, 0] = s; R[:, 1, 1] = c
    return R


def fk(joints, data, zero=False):
    nf = 1 if zero else len(data)
    P, R = {}, {}
    for j in joints:
        Rl = np.tile(np.eye(3), (nf, 1, 1))
        pos = np.tile(j.offset, (nf, 1))
        for k, ch in enumerate(j.channels):
            v = np.zeros(nf) if zero else data[:, j.ch0 + k]
            if ch.endswith('rotation'):
                Rl = Rl @ rot(ch[0], np.radians(v))
            elif not zero:
                pos[:, 'XYZ'.index(ch[0])] = v
        if j.parent is None:
            P[j.name], R[j.name] = pos, Rl
        else:
            pr = R[j.parent.name]
            P[j.name] = P[j.parent.name] + np.einsum('nij,nj->ni', pr, pos)
            R[j.name] = pr @ Rl
        if j.end is not None:
            P[j.name + '_end'] = P[j.name] + np.einsum('nij,j->ni', R[j.name], j.end)
    return P, R
