"""Where the followed player is and how they move: a constant-velocity
Kalman filter over their ground positions.

State ``[x, y, vx, vy]`` in world metres (the follower's dead-reckoned
frame). Process noise is white acceleration (``ACCEL_SIGMA``). A sighting
measures the position from the avatar by a bearing (the name tag's column:
precise) and a distance (metric depth: rougher, and rougher the farther), so
its covariance is an ellipse long along the line of sight, turned into the
world frame. A sighting too far from the prediction (Mahalanobis gate) is
dropped as an outlier (a misread tag, a depth glitch); GATE_RESETS of them in
a row restart the filter there (they really are elsewhere, or the dead
reckoning drifted).
"""

from __future__ import annotations

import math

import numpy as np

# White acceleration of a walking player (m/s^2).
ACCEL_SIGMA = 1.5
# Measurement noise: along the line of sight RANGE_FRAC of the distance plus
# RANGE_BASE metres; across it BEARING_FRAC of the distance plus BEARING_BASE.
RANGE_FRAC = 0.15
RANGE_BASE = 0.2
BEARING_FRAC = 0.03
BEARING_BASE = 0.1
# A new track's velocity is unknown: this standard deviation (m/s).
INITIAL_SPEED_SIGMA = 1.5
# Squared Mahalanobis distance beyond which a sighting is an outlier (chi^2,
# 2 degrees of freedom, 99.9%), and how many in a row restart the track.
GATE = 13.8
GATE_RESETS = 3


class TargetTrack:
    def __init__(self) -> None:
        self.x: np.ndarray | None = None  # [x, y, vx, vy]
        self.p: np.ndarray | None = None
        self.at = 0.0  # time of the state
        self.seen_at = 0.0  # time of the last accepted sighting
        self.seen = (0.0, 0.0)  # its position
        self.rejected = 0

    @property
    def active(self) -> bool:
        return self.x is not None

    def reset(self) -> None:
        self.x = self.p = None
        self.rejected = 0

    @staticmethod
    def measurement_noise(origin: tuple[float, float], position: tuple[float, float]) -> np.ndarray:
        """The covariance of a sighting at ``position`` seen from ``origin``."""
        dx, dy = position[0] - origin[0], position[1] - origin[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        along = (RANGE_FRAC * distance + RANGE_BASE) ** 2
        across = (BEARING_FRAC * distance + BEARING_BASE) ** 2
        c, s = math.cos(angle), math.sin(angle)
        rot = np.array([[c, -s], [s, c]])
        return rot @ np.diag([along, across]) @ rot.T

    def _predict_to(self, t: float) -> None:
        dt = max(0.0, t - self.at)
        if dt == 0.0:
            return
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q1 = np.array([[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]]) * ACCEL_SIGMA**2
        q = np.zeros((4, 4))
        q[np.ix_([0, 2], [0, 2])] = q1
        q[np.ix_([1, 3], [1, 3])] = q1
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q
        self.at = t

    def update(self, t: float, origin: tuple[float, float], position: tuple[float, float]) -> bool:
        """A sighting at time ``t``: the target at ``position``, seen from
        ``origin``. Returns whether it was taken (not gated out)."""
        r = self.measurement_noise(origin, position)
        z = np.array(position, dtype=float)
        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            self.p = np.zeros((4, 4))
            self.p[:2, :2] = r
            self.p[2, 2] = self.p[3, 3] = INITIAL_SPEED_SIGMA**2
            self.at = self.seen_at = t
            self.seen = (z[0], z[1])
            return True
        self._predict_to(t)
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        innovation = z - h @ self.x
        s = h @ self.p @ h.T + r
        if float(innovation @ np.linalg.solve(s, innovation)) > GATE:
            self.rejected += 1
            if self.rejected >= GATE_RESETS:
                self.reset()
                return self.update(t, origin, position)
            return False
        self.rejected = 0
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ innovation
        self.p = (np.eye(4) - k @ h) @ self.p
        self.seen_at = t
        self.seen = (float(self.x[0]), float(self.x[1]))
        return True

    def velocity(self) -> tuple[float, float]:
        if self.x is None:
            return 0.0, 0.0
        return float(self.x[2]), float(self.x[3])

    def predict(self, t: float, lead: float, max_shift: float) -> tuple[float, float]:
        """Where the target likely is ``lead`` s after ``t``: the filtered
        last position moved on by the velocity for the time since the last
        sighting plus ``lead``, at most ``max_shift`` metres on."""
        if self.x is None:
            raise RuntimeError("no track")
        vx, vy = self.velocity()
        ahead = max(0.0, t - self.seen_at) + lead
        dx, dy = vx * ahead, vy * ahead
        shift = math.hypot(dx, dy)
        if shift > max_shift:
            dx, dy = dx * max_shift / shift, dy * max_shift / shift
        return self.seen[0] + dx, self.seen[1] + dy
