"""Charging dock contact and battery level. Plain Python, no Isaac Sim imports."""

from __future__ import annotations

import math

from .drive import Pose2D, wrap_angle


class Charger:
    """The robot charges while its base_footprint is within tolerance of `pose`
    (the pose of a docked robot)."""

    def __init__(self, pose: Pose2D, tolerance_xy: float, tolerance_yaw: float):
        self.pose = pose
        self.tolerance_xy = tolerance_xy
        self.tolerance_yaw = tolerance_yaw

    def docked(self, robot: Pose2D) -> bool:
        d = math.hypot(robot.x - self.pose.x, robot.y - self.pose.y)
        return d <= self.tolerance_xy and abs(wrap_angle(robot.yaw - self.pose.yaw)) <= self.tolerance_yaw


class Battery:
    """Linear charge and drain in fractions per hour of sim time."""

    def __init__(self, level: float, drain_per_hour: float, charge_per_hour: float):
        self.initial = min(max(level, 0.0), 1.0)
        self.level = self.initial
        self.drain = drain_per_hour / 3600.0
        self.charge = charge_per_hour / 3600.0

    def reset(self) -> None:
        self.level = self.initial

    def update(self, dt: float, charging: bool) -> float:
        rate = self.charge if charging else -self.drain
        self.level = min(max(self.level + rate * dt, 0.0), 1.0)
        return self.level
