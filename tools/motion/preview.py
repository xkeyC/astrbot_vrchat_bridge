"""A contact sheet of a clip: stick figures from the front and the side at
evenly spaced frames, to check a conversion before it plays.

    python preview.py clips/NAME.json [OUT.png]
"""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BONES = [("hips", "chest"), ("chest", "head"),
         ("chest", "l_forearm"), ("l_forearm", "l_hand"), ("chest", "r_forearm"), ("r_forearm", "r_hand"),
         ("hips", "l_leg"), ("l_leg", "l_foot"), ("hips", "r_leg"), ("r_leg", "r_foot")]


def points(clip):
    names = clip["joints"]
    f = np.asarray(clip["frames"], float).reshape(len(clip["frames"]), len(names), 7)
    return {k: f[:, i, :3] for i, k in enumerate(names)}


def sheet(clip, out, count=10):
    P = points(clip)
    n = len(P["hips"])
    idx = np.linspace(0, n - 1, min(count, n)).astype(int)
    fig, axes = plt.subplots(2, len(idx), figsize=(1.6 * len(idx), 4.4))
    axes = np.atleast_2d(axes).reshape(2, len(idx))
    for col, i in enumerate(idx):
        for row, (a, b, label) in enumerate([(0, 1, "front"), (2, 1, "side")]):
            ax = axes[row, col]
            for p, q in BONES:
                color = "tab:blue" if p.startswith("l_") or q.startswith("l_") else (
                    "tab:red" if p.startswith("r_") or q.startswith("r_") else "k")
                # Front: the body's left on the viewer's right (x as is).
                xa = -P[p][i, a] if row == 0 else P[p][i, a]
                xb = -P[q][i, a] if row == 0 else P[q][i, a]
                ax.plot([xa, xb], [P[p][i, b], P[q][i, b]], color=color, lw=2)
            ax.plot([-1, 1], [0, 0], color="0.7", lw=1)
            cx = -P["hips"][i, a] if row == 0 else P["hips"][i, a]
            ax.set_xlim(cx - 0.6, cx + 0.6)
            ax.set_ylim(-0.1, 1.2)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            if row == 0:
                ax.set_title(f"{i / clip['fps']:.1f}s", fontsize=8)
            if col == 0:
                ax.set_ylabel(label, fontsize=8)
    fig.suptitle(clip["name"], fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=80)
    plt.close(fig)


if __name__ == "__main__":
    c = json.load(open(sys.argv[1]))
    sheet(c, sys.argv[2] if len(sys.argv) > 2 else sys.argv[1].replace(".json", ".png"))
