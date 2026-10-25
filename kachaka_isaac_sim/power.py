"""Charging dock contact and battery. Plain Python, no Isaac Sim imports."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

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


def wheel_torque(target: float, velocity: float, damping: float, max_torque: float) -> float:
    """Torque of a velocity drive with no stiffness, as PhysX applies it."""
    return min(max(damping * (target - velocity), -max_torque), max_torque)


@dataclass
class DrivePower:
    """Electrical power drawn by the robot off the charger, in watts.

    idle: electronics, sensors and computer, moving or not.
    per_speed: motor and gear losses per m/s of wheel rim speed (mean of the
      two wheels), times `towing_factor` with a shelf (heavier load).
    efficiency: motor efficiency for the mechanical power the wheels put out
      (accelerating, climbing); braking gives nothing back."""
    idle: float
    per_speed: float
    efficiency: float
    towing_factor: float

    def __call__(self, wheel_speeds, wheel_torques, wheel_radius: float, towing: bool) -> float:
        rim = sum(abs(w) for w in wheel_speeds) * wheel_radius / len(wheel_speeds)
        losses = self.per_speed * rim * (self.towing_factor if towing else 1.0)
        mech = sum(max(t * w, 0.0) for t, w in zip(wheel_torques, wheel_speeds))
        return self.idle + losses + mech / self.efficiency


class EnergyMeter:
    """Energy over the physics steps, read out as mean power."""

    def __init__(self):
        self.energy = 0.0
        self.time = 0.0

    def add(self, power: float, dt: float) -> None:
        self.energy += power * dt
        self.time += dt

    def take(self) -> float | None:
        """Mean power since the last call, None if no time passed."""
        if self.time <= 0.0:
            return None
        p = self.energy / self.time
        self.energy = self.time = 0.0
        return p


class Battery:
    """State of charge from the power drawn, and a constant current, then
    constant voltage charge.

    capacity_wh: energy of a full battery.
    charge_per_hour: charge rate up to `cv_level` (constant current), in
      fractions per hour. Above it the rate falls in proportion to what is left
      (constant voltage), down to `cv_min_fraction` of the full rate, so the
      battery does reach 1.0.
    voltage_curve: (level, volts) points, linear in between.
    """

    def __init__(self, level: float, capacity_wh: float, charge_per_hour: float, cv_level: float = 0.8,
                 cv_min_fraction: float = 0.05, voltage_curve=((0.0, 1.0), (1.0, 1.0)),
                 temperature: float = float("nan")):
        self.initial = min(max(level, 0.0), 1.0)
        self.level = self.initial
        self.capacity_wh = capacity_wh
        self.charge_rate = charge_per_hour / 3600.0  # fraction per second
        self.cv_level = cv_level
        self.cv_min_fraction = cv_min_fraction
        pts = sorted((float(a), float(v)) for a, v in voltage_curve)
        self._curve_x = [a for a, _ in pts]
        self._curve_v = [v for _, v in pts]
        self.nominal_voltage = self.voltage_at(0.5)
        self.temperature = temperature
        self.power = 0.0  # W, + into the battery, - out of it

    @property
    def capacity_ah(self) -> float:
        return self.capacity_wh / self.nominal_voltage

    def reset(self) -> None:
        self.level = self.initial
        self.power = 0.0

    def voltage_at(self, level: float) -> float:
        xs, vs = self._curve_x, self._curve_v
        if level <= xs[0]:
            return vs[0]
        if level >= xs[-1]:
            return vs[-1]
        i = bisect.bisect_right(xs, level)
        f = (level - xs[i - 1]) / (xs[i] - xs[i - 1])
        return vs[i - 1] + f * (vs[i] - vs[i - 1])

    @property
    def voltage(self) -> float:
        return self.voltage_at(self.level)

    @property
    def current(self) -> float:
        """A; positive while charging, negative while discharging (as in
        sensor_msgs/BatteryState)."""
        return self.power / self.voltage

    def charge_fraction_rate(self, level: float) -> float:
        """Fraction per second on the charger at `level`."""
        if level < self.cv_level:
            return self.charge_rate
        taper = (1.0 - level) / max(1.0 - self.cv_level, 1e-9)
        return self.charge_rate * max(taper, self.cv_min_fraction)

    def update(self, dt: float, charging: bool, power: float = 0.0) -> float:
        """Advance by dt seconds. `power` (W) is what the robot draws off the
        charger; on the charger the charger feeds it."""
        if charging:
            # Short steps so the taper above cv_level is followed closely.
            n = max(1, math.ceil(dt / 10.0))
            h = dt / n
            gained = 0.0
            for _ in range(n):
                step = min(self.charge_fraction_rate(self.level) * h, 1.0 - self.level)
                self.level += step
                gained += step
            self.power = gained * self.capacity_wh * 3600.0 / dt if dt > 0 else 0.0
        else:
            self.power = -power
            self.level = max(self.level - power * dt / (self.capacity_wh * 3600.0), 0.0)
        return self.level


def battery_from_config(b) -> Battery:
    """Battery from the `battery` section of config/robot.yaml."""
    return Battery(float(b.initial_percentage), float(b.capacity_wh), float(b.charge_per_hour),
                   float(b.cv_level), float(b.cv_min_fraction), [tuple(p) for p in b.voltage_curve],
                   float(b.temperature))


def drive_power_from_config(b) -> DrivePower:
    return DrivePower(float(b.idle_power), float(b.motion_power_per_speed), float(b.motor_efficiency),
                      float(b.towing_factor))
