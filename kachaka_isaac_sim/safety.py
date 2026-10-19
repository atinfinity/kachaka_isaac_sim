"""Front obstacle stop: the real Kachaka stops by itself when something is just
in front of it (a setting in the smartphone app turns this off).

Plain Python and numpy, no Isaac Sim imports. The simulator measures the
clearance ahead from the LiDAR at every scan and blocks forward motion while
the robot could not stop within the clearance minus a margin. Turning and
backing up stay allowed, so the robot can get away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class FrontStopZone:
    front_x: float  # m, base_footprint to the front of the body
    half_width: float  # m, half the width of the watched lane
    margin: float  # m, wanted gap between the body and the obstacle once stopped
    latency: float  # s, age of the newest clearance (scan period)
    decel: float  # m/s^2, braking deceleration

    def stop_distance(self, v: float) -> float:
        """Clearance at which a robot moving forward at `v` has to brake."""
        v = max(0.0, v)
        return self.margin + v * self.latency + v * v / (2.0 * self.decel)


def front_clearance(ranges, local_dirs: np.ndarray, mount_xyz, mount_rot: np.ndarray,
                    front_x: float, half_width: float) -> float:
    """Distance from the front of the body to the nearest scan point in the
    lane ahead (|y| < half_width), in base_footprint coordinates. `ranges`
    and `local_dirs` (unit vectors) are in the laser frame, which sits at
    `mount_xyz` with rotation `mount_rot`. Returns inf for an empty lane."""
    r = np.asarray(ranges, dtype=float)
    hit = np.isfinite(r)
    if not hit.any():
        return math.inf
    pts = np.asarray(mount_xyz, dtype=float) + (local_dirs[hit] * r[hit, None]) @ mount_rot.T
    ahead = pts[(pts[:, 0] > 0.0) & (np.abs(pts[:, 1]) < half_width)]
    if len(ahead) == 0:
        return math.inf
    return max(0.0, float(ahead[:, 0].min()) - front_x)


class FrontStop:
    """On/off switch, mute and the latest clearance."""

    def __init__(self, zone: FrontStopZone, enabled: bool = True):
        self.zone = zone
        self.enabled = enabled
        # Set while a command that mutes the sensors runs
        # (MOVE_BY_VELOCITY_WITH_MUTED_SENSORS, MOVE_FORWARD with mute_sensors).
        self.muted = False
        self.clearance = math.inf
        self.blocked = False

    @property
    def active(self) -> bool:
        return self.enabled and not self.muted

    def update(self, v: float) -> bool:
        """Whether forward motion at `v` (m/s) has to stop. Once stopped, the
        robot waits until the clearance grows by RELEASE, so it does not creep
        up to the obstacle in small starts and stops."""
        limit = self.zone.stop_distance(v)
        if self.blocked:
            limit += self.RELEASE
        self.blocked = self.active and self.clearance <= limit
        return self.blocked

    RELEASE = 0.03  # m
