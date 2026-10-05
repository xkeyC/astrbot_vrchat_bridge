import numpy as np, collections
C = list(np.load('cycles.npy', allow_pickle=True))
BINS = [('slow', 0.85, 1.12, 'walk'), ('normal', 1.15, 1.45, 'walk'), ('fast', 1.5, 1.95, 'walk'),
        ('run', 2.6, 3.7, 'run')]
NP = 100
ph = np.arange(NP) / NP


def isrun(c):
    return 'run' in c['desc'] or 'jog' in c['desc']


def hand_angles(ax, nm, side):
    inward = 1.0 if side == 'L' else -1.0  # left hand inward = +x
    fwd = -ax[:, 2]; down = -ax[:, 1]
    pitch = np.degrees(np.arctan2(fwd, down))  # 0 hanging, + fingers forward
    splay = np.degrees(np.arcsin(np.clip(ax[:, 0] * inward, -1, 1)))  # + fingers point inward
    inw = np.array([inward, 0, 0.])
    sgn = 1.0 if side == 'L' else -1.0
    # rotation of palm normal about the hand axis, 0 = palm faces the body midline, + = palm turns to face back (pronation)
    twist = np.degrees(np.arctan2(sgn * np.sum(np.cross(inw, nm) * ax, 1), nm @ inw))
    return pitch, splay, twist


def wavg(cs, key):
    subj = collections.Counter(c['clip'][:2] for c in cs)
    w = np.array([1.0 / subj[c['clip'][:2]] for c in cs]); w /= w.sum()
    return np.tensordot(w, np.stack([c[key] for c in cs]), 1)


def p2p(x):
    return x.max() - x.min()


def argph(x, f=np.argmin):
    return ph[f(x)]


def harm(x, k):
    """amplitude and phase (of max) of k-th harmonic"""
    c = np.sum(x * np.exp(-2j * np.pi * k * ph)) * 2 / NP
    amp = abs(c)
    phmax = (np.angle(c) / (2 * np.pi * k)) % (1.0 / k)
    return amp, phmax


rows = []
out = {}
for name, lo, hi, gait in BINS:
    cs = [c for c in C if lo <= c['speed'] <= hi and (isrun(c) == (gait == 'run'))]
    for c in cs:
        for s in 'LR':
            p, sp, tw = hand_angles(c[s.lower() + 'ax'], c[s.lower() + 'nm'], s)
            c[s.lower() + 'ang'] = np.stack([p, sp, tw], 1)
    A = {k: wavg(cs, k) for k in ['head', 'headj', 'hips', 'neck', 'lw', 'rw', 'hang', 'lang', 'rang', 'lax', 'rax', 'lnm', 'rnm', 'lf', 'rf']}
    v = np.mean([c['speed'] for c in cs]); T = np.mean([c['T'] for c in cs])
    subs = sorted(set(c['clip'][:2] for c in cs))
    r = collections.OrderedDict()
    r['n_cycles'] = len(cs); r['subjects'] = ','.join(subs)
    r['speed'] = v; r['speed_sd'] = np.std([c['speed'] for c in cs])
    r['cadence'] = 2 / T; r['stride_m_scaled'] = np.mean([c['speed'] * c['T'] * c['scale'] / 0.0564444 for c in cs])
    h = A['head']; ha = A['hang']
    r['head_y_p2p_cm'] = p2p(h[:, 1]) * 100
    r['head_y_min_ph'] = argph(h[:, 1]); r['head_y_max_ph'] = argph(h[:, 1], np.argmax)
    a2, pm2 = harm(h[:, 1], 2); r['head_y_h2amp_cm'] = a2 * 100; r['head_y_h2_maxph'] = pm2
    r['head_x_p2p_cm'] = p2p(h[:, 0]) * 100; r['head_x_left_ph'] = argph(h[:, 0])  # most left (-x)
    a1, pm1 = harm(h[:, 0], 1); r['head_x_h1amp_cm'] = a1 * 100; r['head_x_h1_rightmax_ph'] = pm1
    r['head_z_p2p_cm'] = p2p(h[:, 2]) * 100
    r['head_pitch_p2p'] = p2p(ha[:, 0]); r['head_pitch_down_ph'] = argph(ha[:, 0])
    a, pm = harm(ha[:, 0], 2); r['head_pitch_h2amp'] = a; r['head_pitch_h2_upmax_ph'] = pm
    r['head_pitch_mean'] = ha[:, 0].mean()
    r['head_yaw_p2p'] = p2p(ha[:, 1]); a, pm = harm(ha[:, 1], 1); r['head_yaw_h1_leftmax_ph'] = pm
    r['head_roll_p2p'] = p2p(ha[:, 2]); a, pm = harm(ha[:, 2], 1); r['head_roll_h1_leftmax_ph'] = pm
    hp = A['hips']; nk = A['neck']
    r['head_ahead_of_hips_cm'] = hp[:, 2].mean() * 100
    r['lean_head_hips_deg'] = np.degrees(np.arctan2(hp[:, 2].mean(), -hp[:, 1].mean()))
    d = (nk - hp).mean(0)
    r['lean_trunk_deg'] = np.degrees(np.arctan2(-d[2], d[1]))
    r['hips_below_eye_m'] = -hp[:, 1].mean()
    lw = A['lw']; la = A['lang']
    f = -lw[:, 2]
    r['wristL_fwd_max_cm'] = f.max() * 100; r['wristL_fwd_min_cm'] = f.min() * 100
    r['wristL_fwd_mean_cm'] = f.mean() * 100
    r['wristL_front_amp_cm'] = (f.max() - f.mean()) * 100; r['wristL_back_amp_cm'] = (f.mean() - f.min()) * 100
    r['wristL_fwd_max_ph'] = argph(f, np.argmax); r['wristL_fwd_min_ph'] = argph(f)
    r['wristL_y_mean_cm'] = lw[:, 1].mean() * 100; r['wristL_y_p2p_cm'] = p2p(lw[:, 1]) * 100
    cq = np.polyfit(f - f.mean(), lw[:, 1] - lw[:, 1].mean(), 2)
    r['wristL_y_vs_f_quad'] = cq[0]; r['wristL_y_vs_f_lin'] = cq[1]; r['wristL_y_vs_f_c'] = cq[2]
    inw = lw[:, 0]  # left wrist: lateral = -x, inward = +x
    r['wristL_lateral_mean_cm'] = -inw.mean() * 100; r['wristL_inward_p2p_cm'] = p2p(inw) * 100
    r['wristL_inward_vs_f_slope'] = np.polyfit(f, inw, 1)[0]
    r['wristL_pitch_min'] = la[:, 0].min(); r['wristL_pitch_max'] = la[:, 0].max()
    r['wristL_pitch_max_ph'] = argph(la[:, 0], np.argmax)
    r['wristL_splay_mean'] = la[:, 1].mean(); r['wristL_splay_p2p'] = p2p(la[:, 1])
    r['wristL_twist_mean'] = la[:, 2].mean(); r['wristL_twist_p2p'] = p2p(la[:, 2])
    r['wristL_twist_max_ph'] = argph(la[:, 2], np.argmax)
    fr = -A['rw'][:, 2]
    r['wristR_fwd_max_ph'] = argph(fr, np.argmax)
    r['wristR_fwd_p2p_cm'] = p2p(fr) * 100
    # foot timing: toe-off proxies (most rearward foot rel hips) for left
    lf = A['lf']; rel = -(lf[:, 2] - hp[:, 2])
    r['L_toeoff_ph'] = argph(rel)
    out[name] = (r, A, cs, v)

keys = list(out['slow'][0].keys())
print('%-28s' % 'metric' + ''.join('%14s' % b[0] for b in BINS))
for k in keys:
    vals = [out[b[0]][0][k] for b in BINS]
    print('%-28s' % k + ''.join(('%14.3f' % x) if not isinstance(x, str) else '%14s' % x[:13] for x in vals))

# cadence vs speed fits
for g in ['walk', 'run']:
    cs = [c for c in C if isrun(c) == (g == 'run') and c['speed'] > 0.8]
    v = np.array([c['speed'] for c in cs]); cad = np.array([2 / c['T'] for c in cs])
    p = np.polyfit(v, cad, 1)
    print(g, 'cadence = %.3f + %.3f*v  (n=%d, v %.2f-%.2f)' % (p[1], p[0], len(cs), v.min(), v.max()),
          'resid sd %.3f' % np.std(cad - np.polyval(p, v)))
print('scales', sorted(set((c['clip'][:2], round(c['scale'], 3), round(c['eye_rest_m'], 3)) for c in C)))

# CSV, 20 samples
import csv
with open('gait_curves.csv', 'w', newline='') as fh:
    w = csv.writer(fh)
    cols = ['bin', 'speed_mps', 'cadence_sps', 'phase',
            'head_x', 'head_y', 'head_z', 'head_pitch', 'head_yaw', 'head_roll',
            'hips_x', 'hips_y', 'hips_z',
            'lw_x', 'lw_y', 'lw_z', 'lw_pitch', 'lw_splay', 'lw_twist',
            'rw_x', 'rw_y', 'rw_z', 'rw_pitch', 'rw_splay', 'rw_twist',
            'lw_ax_x', 'lw_ax_y', 'lw_ax_z', 'lw_palm_x', 'lw_palm_y', 'lw_palm_z',
            'rw_ax_x', 'rw_ax_y', 'rw_ax_z', 'rw_palm_x', 'rw_palm_y', 'rw_palm_z']
    w.writerow(cols)
    for b in BINS:
        r, A, cs, v = out[b[0]]
        for i in range(0, NP, NP // 20):
            row = [b[0], '%.2f' % v, '%.2f' % r['cadence'], '%.2f' % ph[i]]
            for k in ['head', 'hang', 'hips', 'lw', 'lang', 'rw', 'rang', 'lax', 'lnm', 'rax', 'rnm']:
                fmt = '%.1f' if k in ('hang', 'lang', 'rang') else '%.4f'
                row += [fmt % x for x in A[k][i]]
            w.writerow(row)
np.save('bins.npy', np.array({k: (out[k][0], out[k][1]) for k in out}, dtype=object), allow_pickle=True)
