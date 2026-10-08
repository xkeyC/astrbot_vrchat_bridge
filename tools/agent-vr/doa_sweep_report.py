"""Reads a speaker recording made by doa_sweep.sh (docs/full-vr/speaker.md, 4.2)
and prints the ears' delay against where the head looked:

- per degree of head yaw, the median GCC-PHAT delay of the voiced hops
  (samples at 48 kHz, + when the left ear lags);
- where it crosses zero, and which way: facing the source the delay falls
  as the head turns right (the source moves to the left), unless the ears
  are swapped (then: --swap-ears true);
- the runs of an unchanged delay: steps a few degrees wide mean VRChat
  renders the HRTF with nearest-neighbour interpolation, a smooth curve
  bilinear;
- with --table (one or more VRCHRTF1 files, e.g. the bilinear asset and a
  nearest table from tools/hrtf-render), how far each table's ITD is from
  the measured delays, the source's yaw fitted.

    python3 doa_sweep_report.py <recording dir> [--table steam-default-48k.bin ...]

Standard library only.
"""

import argparse
import json
import math
import statistics
import struct
from pathlib import Path


def read_table(path):
    """(name, azimuths, ITD in samples per azimuth on the elevation nearest 0)."""
    b = Path(path).read_bytes()
    assert b[:8] == b"VRCHRTF1", f"{path}: not a VRCHRTF1 file"
    rate, taps, n_el, n_az = struct.unpack_from("<IIII", b, 8)
    el0, el_step, az0, az_step = struct.unpack_from("<ffff", b, 24)
    (interp,) = struct.unpack_from("<I", b, 40)
    els = [el0 + i * el_step for i in range(n_el)]
    e = min(range(n_el), key=lambda i: abs(els[i]))
    per = 8 + 4 * taps
    azs, itd = [], []
    for a in range(n_az):
        at = 64 + (e * n_az + a) * per
        left, right = struct.unpack_from("<ff", b, at)
        azs.append(az0 + a * az_step)
        itd.append((left - right) * rate)
    return f"{Path(path).name} ({'bilinear' if interp == 1 else 'nearest'})", azs, itd


def wrap(d):
    return (d + 540.0) % 360.0 - 180.0


def table_itd(azs, itd, az):
    i = min(range(len(azs)), key=lambda k: abs(wrap(azs[k] - az)))
    return itd[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--table", action="append", default=[])
    ap.add_argument("--min-strength", type=float, default=0.5)
    a = ap.parse_args()
    by_yaw = {}
    for line in (Path(a.dir) / "hops.jsonl").read_text().splitlines():
        h = json.loads(line)
        if not h.get("voiced") or not h.get("lag") or h["lag"][1] < a.min_strength:
            continue
        by_yaw.setdefault(round(h["head_yaw"]) % 360, []).append(h["lag"][0])
    if not by_yaw:
        print("no voiced hops with a clear delay: was the source speaking?")
        return
    yaws = sorted(by_yaw)
    lag = {y: statistics.median(by_yaw[y]) for y in yaws}
    print("head_yaw  delay  hops")
    for y in yaws:
        print(f"{wrap(y):8.0f} {lag[y]:6.2f} {len(by_yaw[y]):5d}")

    # Zero crossings (of the delay smoothed over 9 degrees), and which way
    # the delay goes through them.
    smooth = {y: statistics.median([lag[(y + k) % 360] for k in range(-4, 5) if (y + k) % 360 in lag]) for y in yaws}
    print("\nzero crossings (head yaw, falling/rising as the head turns right):")
    for y0, y1 in zip(yaws, yaws[1:] + yaws[:1]):
        if (y1 - y0) % 360 > 3:
            continue
        if smooth[y0] > 0 >= smooth[y1] or smooth[y0] < 0 <= smooth[y1]:
            way = "falling" if smooth[y1] < smooth[y0] else "rising"
            print(f"  {wrap(y0):6.0f}  {way}")
    print("  facing the source the delay should be falling (rising there: the ears are swapped)")

    # Runs of the same delay (rounded to a tenth of a sample).
    runs, run = [], 1
    for y0, y1 in zip(yaws, yaws[1:]):
        if y1 - y0 == 1 and round(lag[y0], 1) == round(lag[y1], 1):
            run += 1
        else:
            runs.append(run)
            run = 1
    runs.append(run)
    long = [r for r in runs if r >= 3]
    print(f"\nruns of an unchanged delay: {len(long)} of 3+ degrees, median {statistics.median(long) if long else 0}"
          " (many runs as wide as a grid cell: nearest-neighbour rendering)")

    for path in a.table:
        name, azs, itd = read_table(path)
        best = None
        for s in range(0, 360):
            for sign in (1, -1):
                err = [(sign * lag[y] - table_itd(azs, itd, wrap(s - y))) ** 2 for y in yaws]
                rms = math.sqrt(sum(err) / len(err))
                if best is None or rms < best[0]:
                    best = (rms, s, sign)
        rms, s, sign = best
        print(f"\n{name}: rms {rms:.2f} samples with the source at yaw {wrap(s):.0f}"
              f"{'' if sign == 1 else ', ears swapped'}")


if __name__ == "__main__":
    main()
