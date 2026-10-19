import math

import pytest

from kachaka_isaac_sim.drive import Pose2D
from kachaka_isaac_sim.power import Battery, Charger


def test_charger_tolerance():
    charger = Charger(Pose2D(1.0, 0.0, math.pi), tolerance_xy=0.05, tolerance_yaw=0.1)
    assert charger.docked(Pose2D(1.03, 0.0, -math.pi + 0.05))  # yaw wraps
    assert not charger.docked(Pose2D(1.06, 0.0, math.pi))
    assert not charger.docked(Pose2D(1.0, 0.0, math.pi - 0.2))


def test_battery_charges_drains_and_clamps():
    bat = Battery(1.5, drain_per_hour=0.1, charge_per_hour=0.5)
    assert bat.level == 1.0
    assert bat.update(3600.0, charging=False) == pytest.approx(0.9)
    assert bat.update(360.0, charging=True) == pytest.approx(0.95)
    assert bat.update(3600.0, charging=True) == 1.0
    assert bat.update(100 * 3600.0, charging=False) == 0.0


def test_battery_reset():
    bat = Battery(0.8, drain_per_hour=0.1, charge_per_hour=0.5)
    bat.update(3600.0, charging=False)
    bat.reset()
    assert bat.level == 0.8
