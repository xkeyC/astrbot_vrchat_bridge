"""Tracking tests on simulated sightings: python -m pytest bridge/test_tracking.py"""

import math
import random

from tracking import TargetTrack

FPS = 5


def sightings(path, origin=(0.0, 0.0), seed=1, seconds=6.0):
    """Noisy sightings (time, position) of a target at ``path(t)``, with the
    follower's noise model (depth along the line of sight, bearing across)."""
    rng = random.Random(seed)
    for i in range(int(seconds * FPS)):
        t = i / FPS
        x, y = path(t)
        dx, dy = x - origin[0], y - origin[1]
        d, a = math.hypot(dx, dy), math.atan2(dy, dx)
        d += rng.gauss(0, 0.15 * d + 0.2)
        a += rng.gauss(0, 0.03)
        yield t, (origin[0] + d * math.cos(a), origin[1] + d * math.sin(a))


def walk(t):
    """Crossing in front, 3 m away, at 1.2 m/s."""
    return 3.0, -3.0 + 1.2 * t


def test_the_filter_steadies_the_velocity():
    track = TargetTrack()
    errors, naive = [], []
    window = []
    for t, z in sightings(walk):
        track.update(t, (0.0, 0.0), z)
        window = [p for p in window + [(t, z)] if t - p[0] <= 3.0]
        if t >= 3.0:
            vx, vy = track.velocity()
            errors.append(math.hypot(vx - 0.0, vy - 1.2))
            (t0, z0), (t1, z1) = window[0], window[-1]
            naive.append(math.hypot((z1[0] - z0[0]) / (t1 - t0), (z1[1] - z0[1]) / (t1 - t0) - 1.2))
    mean = sum(errors) / len(errors)
    print(f"filter {mean:.3f} m/s, window difference {sum(naive) / len(naive):.3f} m/s")
    assert mean < 0.35
    assert mean < sum(naive) / len(naive)


def test_prediction_leads_along_the_motion_and_is_capped():
    track = TargetTrack()
    for t, z in sightings(walk, seconds=4.0):
        track.update(t, (0.0, 0.0), z)
    x, y = track.predict(4.0, lead=1.0, max_shift=3.0)
    sx, sy = track.seen
    assert y > sy + 0.8  # ahead along +y
    assert math.hypot(x - sx, y - sy) <= 3.0 + 1e-9
    far = track.predict(30.0, lead=1.0, max_shift=3.0)
    assert abs(math.hypot(far[0] - sx, far[1] - sy) - 3.0) < 1e-6


def test_an_outlier_is_gated_and_a_real_jump_restarts():
    track = TargetTrack()
    for t, z in sightings(lambda t: (3.0, 0.0), seconds=3.0):
        track.update(t, (0.0, 0.0), z)
    assert not track.update(3.2, (0.0, 0.0), (3.0, 6.0))  # a misread far off
    assert abs(track.seen[1]) < 0.5
    track.update(3.4, (0.0, 0.0), (3.0, 6.0))
    assert track.update(3.6, (0.0, 0.0), (3.0, 6.0))  # third in a row: restarted there
    assert abs(track.seen[1] - 6.0) < 1e-9
    assert track.velocity() == (0.0, 0.0)


def corner(t):
    """+y at 1.2 m/s for 3 s, then turning to +x (around a corner)."""
    return (3.0, -3.0 + 1.2 * t) if t < 3 else (3.0 + 1.2 * (t - 3), 0.6)


def test_the_heading_follows_a_turn_within_a_second():
    errors = []
    for seed in range(20):
        track = TargetTrack()
        for t, z in sightings(corner, seed=seed, seconds=4.2):
            track.update(t, (0.0, 0.0), z)
            if abs(t - 4.0) < 1e-6:
                vx, vy = track.velocity()
                errors.append(abs(math.degrees(math.atan2(vy, vx))))
    # (A 3 s window difference is still ~60 degrees off here.)
    assert sum(errors) / len(errors) < 25
