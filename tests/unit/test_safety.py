import math

import numpy as np
import pytest

from kachaka_isaac_sim.drive import DriveLimits, VelocityCommand
from kachaka_isaac_sim.physics_sensors import quat_to_matrix
from kachaka_isaac_sim.safety import FrontStop, FrontStopZone, front_clearance

# The simulator's LiDAR: laser_frame is base_link turned +90 deg about z,
# 400 beams over 240 deg centred on the robot's forward direction.
MOUNT_XYZ = (0.156, 0.0, 0.1049)
MOUNT_ROT = quat_to_matrix(math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
ANGLES = -math.pi / 2 + np.linspace(-math.radians(120), math.radians(120), 400)
DIRS = np.stack([np.cos(ANGLES), np.sin(ANGLES), np.zeros_like(ANGLES)], axis=1)
FRONT_X, HALF_WIDTH = 0.23, 0.15
ZONE = FrontStopZone(front_x=FRONT_X, half_width=HALF_WIDTH, margin=0.03, latency=0.1, decel=2.0)
LIMITS = DriveLimits(max_linear_vel=0.3, max_angular_vel=1.57, max_linear_accel=2.0,
                     max_angular_accel=3.0, cmd_vel_timeout=0.5)


def wall_ranges(x: float) -> np.ndarray:
    """Ranges to a wall across the robot's path at base x = `x`."""
    d = DIRS @ MOUNT_ROT.T  # beam directions in the base frame
    with np.errstate(divide="ignore"):
        r = (x - MOUNT_XYZ[0]) / d[:, 0]
    return np.where(d[:, 0] > 1e-6, r, np.inf)


def post_ranges(x: float, y: float) -> np.ndarray:
    """One beam hitting a thin post at base (x, y), the others free."""
    d = DIRS @ MOUNT_ROT.T
    to = np.array([x - MOUNT_XYZ[0], y])
    k = int(np.argmax(d[:, :2] @ (to / np.linalg.norm(to))))
    r = np.full(len(DIRS), np.inf)
    r[k] = np.linalg.norm(to)
    return r


def clearance(ranges) -> float:
    return front_clearance(ranges, DIRS, MOUNT_XYZ, MOUNT_ROT, FRONT_X, HALF_WIDTH)


def test_clearance_to_a_wall():
    assert clearance(wall_ranges(0.5)) == pytest.approx(0.5 - FRONT_X)


def test_clearance_is_zero_inside_the_laser_blind_range():
    # Closer than the published min_range (0.1 m from the laser) still counts.
    assert clearance(wall_ranges(0.24)) == pytest.approx(0.01)
    assert clearance(wall_ranges(0.2)) == 0.0


def test_clearance_ignores_points_beside_the_lane():
    assert math.isinf(clearance(np.full(len(DIRS), np.inf)))
    assert clearance(post_ranges(0.4, 0.1)) == pytest.approx(0.4 - FRONT_X, abs=0.005)  # nearest beam
    # Shelf legs stand at +-0.172 m (inner face) while the robot docks under a shelf.
    assert math.isinf(clearance(post_ranges(0.3, 0.172)))
    assert math.isinf(clearance(post_ranges(0.3, -0.172)))


def test_stop_distance_grows_with_speed():
    assert ZONE.stop_distance(0.0) == pytest.approx(0.03)
    assert ZONE.stop_distance(-0.2) == pytest.approx(0.03)
    assert ZONE.stop_distance(0.3) == pytest.approx(0.03 + 0.03 + 0.09 / 4.0)


def test_switch_and_mute():
    fs = FrontStop(ZONE)
    fs.clearance = 0.01
    assert fs.update(0.0)
    fs.enabled = False
    assert not fs.update(0.0)
    fs.enabled, fs.muted = True, True
    assert not fs.update(0.0)
    fs.muted = False
    assert fs.update(0.0)


def test_release_needs_extra_clearance():
    fs = FrontStop(ZONE)
    fs.clearance = 0.03
    assert fs.update(0.0)
    fs.clearance = 0.03 + FrontStop.RELEASE / 2
    assert fs.update(0.0)
    fs.clearance = 0.03 + FrontStop.RELEASE + 0.01
    assert not fs.update(0.0)


def drive_to_wall(wall_x: float, cmd_v: float, fs: FrontStop, seconds: float = 10.0, dt: float = 1 / 60):
    """Kinematic robot driving at `wall_x` (base frame at start); scans at 10 Hz."""
    vc = VelocityCommand(LIMITS)
    x, t, next_scan = 0.0, 0.0, 0.0
    while t < seconds:
        if t >= next_scan:
            next_scan += 0.1
            fs.clearance = clearance(wall_ranges(wall_x - x))
        vc.set_target(cmd_v, 0.0, t)
        v, _ = vc.update(t, dt, fs.update(vc.current[0]))
        x += v * dt
        t += dt
    return wall_x - x - FRONT_X, vc.current[0]  # gap between the body and the wall


@pytest.mark.parametrize("speed", [0.1, 0.2, 0.3])
def test_robot_stops_short_of_a_wall(speed):
    gap, v = drive_to_wall(1.0, speed, FrontStop(ZONE))
    assert v == 0.0
    assert 0.0 < gap <= ZONE.margin + FrontStop.RELEASE


def test_robot_drives_into_the_wall_when_off():
    gap, _ = drive_to_wall(1.0, 0.3, FrontStop(ZONE, enabled=False))
    assert gap < 0.0  # no collision in this kinematic model


def test_backing_up_stays_allowed():
    fs = FrontStop(ZONE)
    fs.clearance = 0.0
    vc = VelocityCommand(LIMITS)
    vc.set_target(-0.2, 0.5, 0.0)
    for i in range(30):
        v, w = vc.update(i / 60, 1 / 60, fs.update(vc.current[0]))
    assert v == pytest.approx(-0.2)
    assert w == pytest.approx(0.5)
