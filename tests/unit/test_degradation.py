import math
import random

import numpy as np
import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.degradation import (
    DARK,
    INVISIBLE,
    VISIBLE,
    BiasWalk,
    CameraDegradation,
    LidarDegradation,
    OdometryDegradation,
    SlipArea,
    SurfaceClasses,
    box_blur_x,
    camera_degradation,
    imu_degradation,
    lidar_degradation,
    odometry_degradation,
    section,
)
from kachaka_isaac_sim.drive import Pose2D, WheelOdometry


def test_defaults_are_off():
    cfg = load_config()
    assert not lidar_degradation(section(cfg, "lidar")).enabled
    assert not odometry_degradation(section(cfg, "odometry")).enabled
    assert not camera_degradation(section(cfg, "cameras")).enabled
    assert not imu_degradation(section(cfg, "imu")).enabled


def test_missing_section_is_off():
    assert not lidar_degradation(None).enabled and not odometry_degradation(None).enabled


def test_surface_classes():
    s = SurfaceClasses(invisible=["/World/Room/Glass*"], dark=["/World/Sofa"])
    assert s.classify("/World/Room/GlassDoor/collider") == INVISIBLE
    assert s.classify("/World/Sofa/mesh", "/World/Sofa") == DARK
    assert s.classify("/World/Sofa") == DARK
    assert s.classify("/World/Sofa2") == VISIBLE
    assert s.classify("/World/Room/Wall") == VISIBLE


def test_lidar_noisy_range():
    rng = random.Random(0)
    d = LidarDegradation(dark_range=1.0)
    assert d.noisy_range(math.inf, False, 0.0, rng) == math.inf
    assert d.noisy_range(2.0, True, 0.0, rng) == math.inf  # dark, too far
    assert d.noisy_range(0.5, True, 0.0, rng) == 0.5  # dark, close enough
    drop = LidarDegradation(dropout=0.3)
    lost = sum(drop.noisy_range(1.0, False, 0.0, rng) == math.inf for _ in range(4000)) / 4000
    assert lost == pytest.approx(0.3, abs=0.03)
    far = LidarDegradation(noise_per_m=0.01)
    err = np.array([far.noisy_range(8.0, False, 0.01, rng) - 8.0 for _ in range(4000)])
    assert err.std() == pytest.approx(0.01 + 0.08, rel=0.1)


def straight(odo, n=100, step=0.01, y=0.0):
    p = None
    for i in range(n + 1):
        p = odo.update(Pose2D(i * step, y, 0.0))
    return p


def test_odometry_scale_errors():
    odo = OdometryDegradation(distance_scale_error=0.05)
    assert straight(odo).x == pytest.approx(1.05)
    turn = OdometryDegradation(turn_scale_error=-0.1)
    for i in range(101):
        p = turn.update(Pose2D(0.0, 0.0, i * 0.01))
    assert p.yaw == pytest.approx(0.9)


def test_odometry_slip_area():
    odo = OdometryDegradation([SlipArea(0.5, -1.0, 1.0, 1.0, slip=0.4)])
    assert straight(odo).x == pytest.approx(0.5 + 0.5 * 1.4, abs=0.01)
    outside = OdometryDegradation([SlipArea(0.5, -1.0, 1.0, 1.0, slip=0.4)])
    assert straight(outside, y=2.0).x == pytest.approx(1.0)


def test_odometry_teleport_keeps_error_and_reset_clears_it():
    odo = OdometryDegradation(distance_scale_error=0.1)
    assert straight(odo).x == pytest.approx(1.1)
    odo.teleported()
    p = odo.update(Pose2D(5.0, 5.0, math.pi / 2))  # the 0.1 m ahead error, now along +y
    assert (p.x, p.y, p.yaw) == pytest.approx((5.0, 5.1, math.pi / 2))
    odo.shift(1.0, 0.0)
    p = odo.update(Pose2D(6.0, 5.0, math.pi / 2))
    assert (p.x, p.y) == pytest.approx((6.0, 5.1))
    odo.reset()
    p = odo.update(Pose2D(0.0, 0.0, 0.0))
    assert (p.x, p.y) == (0.0, 0.0)


def test_odometry_config_slip_areas():
    cfg = load_config()
    cfg.degradation.odometry.slip_areas = [type(cfg)(rect=[0, 0, 1, 1], slip=0.2), [2, 2, 3, 3, 0.5]]
    odo = odometry_degradation(cfg.degradation.odometry)
    assert odo.slip_at(0.5, 0.5) == 0.2 and odo.slip_at(2.5, 2.5) == 0.5 and odo.slip_at(5, 5) == 0.0


def test_wheel_odometry_scale():
    wo = WheelOdometry(0.05, 0.2)
    wo.update(0.0, 0.0, 0.01)
    p = wo.update(1.0, 1.0, 0.01, (1.5, 1.0))
    assert p.x == pytest.approx(0.05 * 1.5)


def test_box_blur_x():
    img = np.zeros((2, 9), np.float32)
    img[:, 4] = 9.0
    out = box_blur_x(img, 3)
    assert out[0].tolist() == pytest.approx([0, 0, 0, 3, 3, 3, 0, 0, 0])
    assert box_blur_x(img, 1) is img


def test_camera_noise_and_blur():
    img = np.full((20, 40, 3), 128, np.uint8)
    cam = CameraDegradation(noise=5.0, seed=1)
    out = cam.apply(img, 0.0)
    assert out.dtype == np.uint8 and out.shape == img.shape
    assert (out.astype(float) - 128).std() == pytest.approx(5.0, rel=0.1)
    blur = CameraDegradation(exposure_time=0.02)
    assert blur.blur_px(1.0, 320.0) == 6 and blur.blur_px(0.0, 320.0) == 0
    edge = np.zeros((4, 40, 3), np.uint8)
    edge[:, 20:] = 200
    out = blur.apply(edge, 0.0, yaw_rate=1.0, fx=320.0)
    assert 0 < out[0, 19, 0] < 200  # the edge is smeared
    assert CameraDegradation().apply(img, 0.0) is img


def test_camera_exposure_wanders_and_stays_bounded():
    cam = CameraDegradation(exposure_jitter=0.5, exposure_time_constant=1.0, seed=2)
    stops = [cam.step_exposure(t * 0.1) for t in range(20000)]
    assert np.std(stops) == pytest.approx(0.5, rel=0.2)
    assert abs(stops[1] - stops[0]) < 0.5  # it wanders, it does not jump


def test_bias_walk():
    rng = random.Random(3)
    assert not BiasWalk(rng=rng).step(1.0).any()
    walks = [BiasWalk(0.0, 0.01, random.Random(k)) for k in range(400)]
    for w in walks:
        for _ in range(100):
            w.step(0.01)
    assert np.std([w.value[0] for w in walks]) == pytest.approx(0.01, rel=0.15)  # 0.01 * sqrt(1 s)


def test_imu_yaw_error_integrates_the_z_bias():
    cfg = load_config()
    cfg.degradation.imu.gyro_bias = 0.01
    imu = imu_degradation(cfg.degradation.imu, seed=4)
    assert imu.enabled
    bz = float(imu.gyro.value[2])
    for _ in range(100):
        imu.step(0.01)
    assert imu.yaw_error == pytest.approx(bz * 1.0)
