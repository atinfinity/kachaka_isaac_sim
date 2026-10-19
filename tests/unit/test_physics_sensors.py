import math
from types import SimpleNamespace

import numpy as np
import pytest

from kachaka_isaac_sim.physics_sensors import Imu, matrix_to_quat, quat_to_matrix
from kachaka_isaac_sim.urdf_tools import rpy_to_quat

QUATS = [rpy_to_quat(r, p, y) for r, p, y in
         [(0, 0, 0), (0, 0, math.pi), (math.pi, 0, 0), (0.3, -0.4, 2.0), (0, math.pi / 2, 0), (2.5, 0.1, -2.9)]]


@pytest.mark.parametrize("q", QUATS)
def test_quaternion_round_trip(q):
    m = quat_to_matrix(*q)
    assert m @ m.T == pytest.approx(np.eye(3))
    q2 = matrix_to_quat(m)
    sign = 1.0 if np.dot(q, q2) >= 0 else -1.0  # q and -q are the same rotation
    assert np.array(q2) * sign == pytest.approx(np.array(q))


def imu(**kw) -> Imu:
    cfg = SimpleNamespace(gyro_noise=0.0, accel_noise=0.0)
    return Imu(cfg, kw.get("mount", (0, 0, 0)), kw.get("quat", (1, 0, 0, 0)), (0, 0, 0), seed=0)


def test_imu_at_rest_reads_gravity():
    sensor = imu()
    for _ in range(3):
        sensor.update(np.eye(3), np.zeros(3), np.zeros(3), 0.01)
    assert sensor.linear_acceleration == pytest.approx([0, 0, 9.81])
    assert sensor.angular_velocity == pytest.approx([0, 0, 0])
    assert sensor.orientation == pytest.approx((1, 0, 0, 0))


def test_imu_linear_acceleration():
    sensor = imu()
    sensor.update(np.eye(3), np.zeros(3), np.zeros(3), 0.01)
    sensor.update(np.eye(3), np.array([0.01, 0, 0]), np.zeros(3), 0.01)
    assert sensor.linear_acceleration == pytest.approx([1.0, 0, 9.81])


def test_imu_rotated_base_and_mount():
    yaw90 = quat_to_matrix(*rpy_to_quat(0, 0, math.pi / 2))
    sensor = imu(quat=rpy_to_quat(math.pi, 0, 0))  # mounted upside down
    sensor.update(yaw90, np.zeros(3), np.array([0, 0, 0.5]), 0.01)
    assert sensor.angular_velocity == pytest.approx([0, 0, -0.5])
    assert sensor.linear_acceleration == pytest.approx([0, 0, -9.81])


def test_imu_off_center_mount_sees_centripetal_acceleration():
    sensor = imu(mount=(0.1, 0, 0))
    w = np.array([0, 0, 1.0])
    for k in range(200):
        rot = quat_to_matrix(*rpy_to_quat(0, 0, k * 0.001))
        sensor.update(rot, np.zeros(3), w, 0.001)
    # v = w x r, a = -w^2 r: 0.1 m/s^2 toward the center, along -x of the IMU.
    assert sensor.linear_acceleration == pytest.approx([-0.1, 0, 9.81], abs=1e-3)
