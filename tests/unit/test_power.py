import math

import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.drive import Pose2D
from kachaka_isaac_sim.power import (
    Battery,
    Charger,
    DrivePower,
    EnergyMeter,
    battery_from_config,
    drive_power_from_config,
    wheel_torque,
)

CURVE = ((0.0, 9.0), (0.5, 11.0), (1.0, 12.0))


def make_battery(level=1.0, capacity_wh=10.0, charge_per_hour=1.0, cv_level=0.8, cv_min_fraction=0.05):
    return Battery(level, capacity_wh, charge_per_hour, cv_level, cv_min_fraction, CURVE, 25.0)


def test_charger_tolerance():
    charger = Charger(Pose2D(1.0, 0.0, math.pi), tolerance_xy=0.05, tolerance_yaw=0.1)
    assert charger.docked(Pose2D(1.03, 0.0, -math.pi + 0.05))  # yaw wraps
    assert not charger.docked(Pose2D(1.06, 0.0, math.pi))
    assert not charger.docked(Pose2D(1.0, 0.0, math.pi - 0.2))


def test_drain_follows_power_and_clamps():
    bat = make_battery(1.5)
    assert bat.level == 1.0
    assert bat.update(3600.0, charging=False, power=1.0) == pytest.approx(0.9)  # 1 Wh of 10
    assert bat.update(1800.0, charging=False, power=4.0) == pytest.approx(0.7)
    assert bat.update(100 * 3600.0, charging=False, power=1.0) == 0.0


def test_charge_constant_current_then_tapers():
    bat = make_battery(0.2)
    bat.update(1800.0, charging=True)  # 1.0/h for half an hour
    assert bat.level == pytest.approx(0.7)
    # Up to 0.8 at the full rate (6 min), then slower than that.
    bat.update(360.0, charging=True)
    assert bat.level == pytest.approx(0.8, abs=1e-3)
    # Above it what is left falls off exponentially (time constant 0.2 h)
    # instead of the 0.1 the full rate would add in 6 min.
    bat.update(360.0, charging=True)
    assert bat.level == pytest.approx(1.0 - 0.2 * math.exp(-0.5), abs=1e-3)
    assert bat.charge_fraction_rate(0.9) == pytest.approx(bat.charge_fraction_rate(0.1) / 2)
    # The floor brings it to full in finite time.
    bat.update(3 * 3600.0, charging=True)
    assert bat.level == 1.0
    bat.update(60.0, charging=True)
    assert bat.level == 1.0 and bat.current == 0.0


def test_voltage_and_current():
    bat = make_battery(0.5)
    assert bat.voltage == pytest.approx(11.0)
    assert bat.voltage_at(0.25) == pytest.approx(10.0)
    assert bat.voltage_at(-1.0) == 9.0 and bat.voltage_at(2.0) == 12.0
    bat.update(1.0, charging=False, power=11.0)
    assert bat.current == pytest.approx(-1.0, rel=1e-3)  # discharging: negative
    bat.update(1.0, charging=True)
    assert bat.current > 0.0
    assert bat.capacity_ah == pytest.approx(10.0 / 11.0)


def test_battery_reset():
    bat = make_battery(0.8)
    bat.update(3600.0, charging=False, power=2.0)
    bat.reset()
    assert bat.level == 0.8 and bat.power == 0.0


def test_drive_power_idle_motion_and_shelf():
    p = DrivePower(idle=8.0, per_speed=30.0, efficiency=0.5, towing_factor=2.0)
    r = 0.05
    assert p((0.0, 0.0), (0.0, 0.0), r, towing=False) == pytest.approx(8.0)
    # 0.3 m/s straight: rim speed 0.3 m/s.
    moving = p((6.0, 6.0), (0.0, 0.0), r, towing=False)
    assert moving == pytest.approx(8.0 + 9.0)
    assert p((6.0, 6.0), (0.0, 0.0), r, towing=True) == pytest.approx(8.0 + 18.0)
    # Turning in place costs too.
    assert p((-6.0, 6.0), (0.0, 0.0), r, towing=False) == pytest.approx(moving)
    # Accelerating: torque times speed over efficiency; braking gives nothing back.
    assert p((6.0, 6.0), (1.0, 1.0), r, towing=False) == pytest.approx(moving + 12.0 / 0.5)
    assert p((6.0, 6.0), (-1.0, -1.0), r, towing=False) == pytest.approx(moving)


def test_wheel_torque_is_capped():
    assert wheel_torque(2.0, 1.0, damping=10.0, max_torque=20.0) == pytest.approx(10.0)
    assert wheel_torque(2.0, 1.0, damping=10.0, max_torque=2.0) == 2.0
    assert wheel_torque(0.0, 1.0, damping=10.0, max_torque=2.0) == -2.0


def test_energy_meter_mean_power():
    m = EnergyMeter()
    assert m.take() is None
    m.add(10.0, 0.5)
    m.add(20.0, 0.5)
    assert m.take() == pytest.approx(15.0)
    assert m.take() is None


def test_default_config_rates():
    """The defaults give the rates the config comment states."""
    b = load_config().battery
    bat, power = battery_from_config(b), drive_power_from_config(b)
    r = load_config().drive.wheel_radius

    def per_hour(watts):
        return watts / bat.capacity_wh

    assert per_hour(power((0.0, 0.0), (0.0, 0.0), r, False)) == pytest.approx(0.2, abs=0.02)
    w = 0.3 / r
    assert per_hour(power((w, w), (0.0, 0.0), r, False)) == pytest.approx(0.4, abs=0.05)
    assert per_hour(power((w, w), (0.0, 0.0), r, True)) == pytest.approx(0.6, abs=0.05)
    assert 9.0 < bat.voltage_at(0.0) < bat.voltage_at(1.0) < 13.0
