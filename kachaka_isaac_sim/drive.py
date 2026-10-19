"""Differential-drive control and odometry. Plain Python, no Isaac Sim imports."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass


def _approach(current: float, target: float, max_step: float) -> float:
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class DriveLimits:
    max_linear_vel: float
    max_angular_vel: float
    max_linear_accel: float
    max_angular_accel: float
    cmd_vel_timeout: float


class VelocityCommand:
    """Clamps, rate-limits and times out cmd_vel like a motor controller would."""

    def __init__(self, limits: DriveLimits):
        self.limits = limits
        self.target = (0.0, 0.0)
        self.current = (0.0, 0.0)
        self.last_cmd_time: float | None = None

    def set_target(self, linear: float, angular: float, now: float) -> None:
        lim = self.limits
        self.target = (
            max(-lim.max_linear_vel, min(lim.max_linear_vel, linear)),
            max(-lim.max_angular_vel, min(lim.max_angular_vel, angular)),
        )
        self.last_cmd_time = now

    def update(self, now: float, dt: float, forward_blocked: bool = False) -> tuple[float, float]:
        """`forward_blocked` brakes forward motion (front obstacle stop) without
        dropping the command: the robot goes on once the way is clear."""
        lim = self.limits
        if self.last_cmd_time is None or now - self.last_cmd_time > lim.cmd_vel_timeout:
            self.target = (0.0, 0.0)
        target_v = min(0.0, self.target[0]) if forward_blocked else self.target[0]
        v = _approach(self.current[0], target_v, lim.max_linear_accel * dt)
        w = _approach(self.current[1], self.target[1], lim.max_angular_accel * dt)
        self.current = (v, w)
        return self.current


def wheel_speeds(v: float, w: float, radius: float, separation: float) -> tuple[float, float]:
    """Return (left, right) wheel angular velocity in rad/s."""
    half = separation / 2.0
    return (v - w * half) / radius, (v + w * half) / radius


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class WheelOdometry:
    """Dead reckoning from wheel joint angles, so wheel slip shows up as drift."""

    def __init__(self, radius: float, separation: float):
        self.radius = radius
        self.separation = separation
        self.pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self._last: tuple[float, float] | None = None

    def reset(self) -> None:
        """Start again at the origin; the next update takes the wheel angles as they are."""
        self.pose = Pose2D()
        self.v = self.w = 0.0
        self._last = None

    def update(self, left_angle: float, right_angle: float, dt: float) -> Pose2D:
        if self._last is None:
            self._last = (left_angle, right_angle)
            return self.pose
        # Joint angles of continuous joints wrap at +-pi; unwrap the step.
        dl = wrap_angle(left_angle - self._last[0]) * self.radius
        dr = wrap_angle(right_angle - self._last[1]) * self.radius
        self._last = (left_angle, right_angle)
        ds = (dl + dr) / 2.0
        dyaw = (dr - dl) / self.separation
        mid = self.pose.yaw + dyaw / 2.0
        self.pose.x += ds * math.cos(mid)
        self.pose.y += ds * math.sin(mid)
        self.pose.yaw = wrap_angle(self.pose.yaw + dyaw)
        if dt > 0:
            self.v, self.w = ds / dt, dyaw / dt
        return self.pose


class NoisyOdometry:
    """Ground truth relative to the start pose, plus a slow random-walk drift.

    The drift only grows while the robot moves, like a fused (wheel + IMU + scan)
    odometry on the real robot. Noise values are per sqrt(metre or radian)
    travelled.
    """

    def __init__(self, noise_xy: float, noise_yaw: float, seed: int | None = None):
        self.noise_xy = noise_xy
        self.noise_yaw = noise_yaw
        self.rng = random.Random(seed)
        self.origin: Pose2D | None = None
        self.drift = Pose2D()
        self._last_gt: Pose2D | None = None

    def reset(self, origin: Pose2D | None = None) -> None:
        """Start again with no drift. `origin` is the ground-truth pose that
        reads as zero (default: the next pose seen)."""
        self.origin = None if origin is None else Pose2D(origin.x, origin.y, origin.yaw)
        self.drift = Pose2D()
        self._last_gt = None

    def teleported(self) -> None:
        """The robot was moved without driving: the jump adds no drift, and the
        drift so far stays, so the odom frame (and SLAM's map -> odom) still
        holds."""
        self._last_gt = None

    def update(self, gt: Pose2D) -> Pose2D:
        if self.origin is None:
            self.origin = Pose2D(gt.x, gt.y, gt.yaw)
        if self._last_gt is not None:
            dist = math.hypot(gt.x - self._last_gt.x, gt.y - self._last_gt.y)
            turn = abs(wrap_angle(gt.yaw - self._last_gt.yaw))
            scale_xy = math.sqrt(dist)
            scale_yaw = math.sqrt(dist + turn)
            self.drift.x += self.rng.gauss(0.0, self.noise_xy) * scale_xy
            self.drift.y += self.rng.gauss(0.0, self.noise_xy) * scale_xy
            self.drift.yaw += self.rng.gauss(0.0, self.noise_yaw) * scale_yaw
        self._last_gt = Pose2D(gt.x, gt.y, gt.yaw)
        # Express gt in the start frame, then apply the drift transform.
        rel = relative_pose(self.origin, gt)
        c, s = math.cos(self.drift.yaw), math.sin(self.drift.yaw)
        return Pose2D(
            self.drift.x + c * rel.x - s * rel.y,
            self.drift.y + s * rel.x + c * rel.y,
            wrap_angle(rel.yaw + self.drift.yaw),
        )


def relative_pose(origin: Pose2D, pose: Pose2D) -> Pose2D:
    """Return `pose` expressed in the frame of `origin`."""
    dx, dy = pose.x - origin.x, pose.y - origin.y
    c, s = math.cos(origin.yaw), math.sin(origin.yaw)
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose.yaw - origin.yaw))


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return (w, x, y, z)."""
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)
