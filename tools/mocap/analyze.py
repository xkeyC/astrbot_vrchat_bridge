import glob, os, sys, json
import numpy as np
from bvhfk import parse, fk, UNIT_M

EYE = 1.6
NPH = 100
DESC = {}
for line in open('index.txt', encoding='latin-1'):
    p = line.strip().split('\t')
    if len(p) >= 2 and len(p[0]) == 5 and p[0][2] == '_':
        DESC[p[0]] = p[1]

UP = np.array([0, 1., 0])


def movavg(x, w):
    k = np.ones(w) / w
    pad = w // 2
    xp = np.concatenate([np.repeat(x[:1], pad, 0), x, np.repeat(x[-1:], w - 1 - pad, 0)])
    return np.stack([np.convolve(xp[:, i], k, 'valid') for i in range(x.shape[1])], 1)


def local_max(s, minsep):
    idx = [i for i in range(1, len(s) - 1) if s[i] >= s[i - 1] and s[i] > s[i + 1]]
    out = []
    for i in idx:  # keep the biggest within minsep
        if out and i - out[-1] < minsep:
            if s[i] > s[out[-1]]:
                out[-1] = i
        else:
            out.append(i)
    return out


def resample(t, y, n=NPH):
    tt = np.linspace(t[0], t[-1], n + 1)[:-1]
    y = np.asarray(y)
    if y.ndim == 1:
        return np.interp(tt, t, y)
    return np.stack([np.interp(tt, t, y[:, i]) for i in range(y.shape[1])], 1)


def angles_head(Rb):
    f = Rb @ np.array([0, 0, -1.])  # forward
    u = Rb @ np.array([0, 1., 0])
    pitch = np.degrees(np.arcsin(np.clip(f[:, 1], -1, 1)))  # + nose up
    yaw = np.degrees(np.arctan2(-f[:, 0], -f[:, 2]))  # + turn left
    roll = np.degrees(np.arctan2(-u[:, 0], u[:, 1]))  # + top of head to the left
    return np.stack([pitch, yaw, roll], 1)


def process(path):
    name = os.path.basename(path)[:-4]
    J, D, ft = parse(path)
    P0, _ = fk(J, D, zero=True)
    floor0 = min(P0['LeftToeBase_end'][0, 1], P0['LeftFoot'][0, 1])
    eye_rest = P0['Head_end'][0, 1] - floor0
    s = EYE / eye_rest  # units -> scaled metres
    P, R = fk(J, D)
    n = len(D)
    t = np.arange(n) * ft
    hip = P['Hips'].copy()
    hipH = hip[:, [0, 2]]
    sm = movavg(hipH, int(round(0.6 / ft)))
    vel = np.gradient(sm, ft, axis=0)
    hd = vel / (np.linalg.norm(vel, axis=1, keepdims=True) + 1e-9)
    res = []
    hs = {}
    for side in 'LR':
        foot = P[('Left' if side == 'L' else 'Right') + 'Foot'][:, [0, 2]]
        rel = np.sum((foot - hipH) * hd, 1)
        rel = rel - movavg(rel[:, None], int(round(0.5 / ft)))[:, 0] * 0  # keep raw
        thr = np.percentile(rel, 60)
        cand = local_max(rel, int(0.25 / ft))
        hs[side] = [i for i in cand if rel[i] > thr]
    for side in 'LR':
        other = 'R' if side == 'L' else 'L'
        H = hs[side]
        # drop maxima whose spacing is much shorter than the median spacing
        if len(H) > 2:
            dm = np.median(np.diff(H))
            H2 = [H[0]]
            for i in H[1:]:
                if i - H2[-1] < 0.7 * dm:
                    continue
                H2.append(i)
            H = H2
        for a, b in zip(H[:-1], H[1:]):
            T = (b - a) * ft
            if not (0.45 < T < 1.7):
                continue
            # skip cycles touching clip ends (smoothing edge)
            if a < int(0.05 / ft) or b > n - int(0.05 / ft):
                continue
            seg = slice(a, b + 1)
            tc = t[seg] - t[a]
            # linear fit of pelvis horizontal travel
            A = np.stack([np.ones_like(tc), tc], 1)
            # endpoint velocity (HS->HS is periodic; regression over one period is biased by sway)
            vv = (hipH[b] - hipH[a]) / T
            coef = np.stack([hipH[seg].mean(0) - vv * tc.mean(), vv])
            v2 = coef[1]
            speed = np.linalg.norm(v2) * UNIT_M
            if speed < 0.4:
                continue
            h = v2 / np.linalg.norm(v2)
            # steadiness: compare displacement of first & second half
            m = a + (b - a) // 2
            d1 = np.linalg.norm(hipH[m] - hipH[a]); d2 = np.linalg.norm(hipH[b] - hipH[m])
            if abs(d1 - d2) / (d1 + d2) > 0.08:
                continue
            # heading change within cycle
            hdeg = np.degrees(np.arccos(np.clip(np.sum(hd[a] * hd[b]), -1, 1)))
            if hdeg > 12:
                continue
            fwd = np.array([h[0], 0, h[1]])
            right = np.cross(fwd, UP)
            B = np.stack([right, UP, -fwd], 1)  # columns: body x,y,z in world
            orig = np.zeros((b - a + 1, 3))
            orig[:, [0, 2]] = A @ coef
            def body(p):
                return ((p[seg] - orig) @ B) * s
            head = body(P['Head_end'])
            headj = body(P['Head'])
            hips_b = body(P['Hips'])
            neck = body(P['Neck1'])
            lw = body(P['LeftHand']); rw = body(P['RightHand'])
            lf = body(P['LeftFoot']); rf = body(P['RightFoot'])
            M = np.diag([-1., 1, -1])
            Rh = np.einsum('ji,njk->nik', B, R['Head'][seg]) @ M
            hang = angles_head(Rh)
            def handrot(nm, sgn):
                Rw = np.einsum('ji,njk->nik', B, R[nm][seg])
                ax = Rw @ np.array([sgn, 0, 0.])
                nrm = Rw @ np.array([0, -1., 0])
                return ax, nrm
            lax, lnm = handrot('LeftHand', 1)
            rax, rnm = handrot('RightHand', -1)
            # vertical foot positions -> toe-off/contact info not needed; record other HS phase
            oh = [i for i in hs[other] if a < i < b]
            ophase = (oh[0] - a) / (b - a) if oh else np.nan
            mir = side == 'R'
            def M3(v):  # mirror x
                v = v.copy(); v[:, 0] *= -1; return v
            if mir:
                head, headj, hips_b, neck = M3(head), M3(headj), M3(hips_b), M3(neck)
                lw, rw = M3(rw), M3(lw)
                lax, rax = M3(rax), M3(lax)
                lnm, rnm = M3(rnm), M3(lnm)
                lf, rf = M3(rf), M3(lf)
                hang = hang * np.array([1, -1, -1])
            mh = head.mean(0)
            rec = dict(clip=name, desc=DESC.get(name, ''), side=side, T=T, speed=speed,
                       ophase=ophase, scale=s, eye_rest_m=eye_rest * UNIT_M)
            cur = dict(head=head - mh, headj=headj - mh, hips=hips_b - mh, neck=neck - mh,
                       lw=lw - mh, rw=rw - mh, lf=lf - mh, rf=rf - mh,
                       hang=hang, lax=lax, rax=rax, lnm=lnm, rnm=rnm)
            for k, v in cur.items():
                rec[k] = resample(tc, v)
            res.append(rec)
    return res


if __name__ == '__main__':
    allc = []
    for f in sorted(glob.glob('bvh/*.bvh')):
        try:
            allc += process(f)
        except Exception as e:
            print('ERR', f, e)
    np.save('cycles.npy', np.array(allc, dtype=object), allow_pickle=True)
    for c in allc:
        print(c['clip'], c['side'], c['desc'][:20], 'v=%.2f T=%.2f cad=%.2f oph=%.2f' % (c['speed'], c['T'], 2 / c['T'], c['ophase']))
    print(len(allc))
