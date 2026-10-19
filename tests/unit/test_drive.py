import math

import pytest

from kachaka_isaac_sim.drive import (
    DriveLimits,
    NoisyOdometry,
    Pose2D,
    VelocityCommand,
    WheelOdometry,
    quat_from_yaw,
    relative_pose,
    wheel_speeds,
    wrap_angle,
    yaw_from_quat,
)

R, SEP = 0.045, 0.2
LIMITS = DriveLimits(max_linear_vel=0.3, max_angular_vel=1.57, max_linear_accel=2.0,
                     max_angular_accel=3.0, cmd_vel_timeout=0.5)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap_angle(0.5) == pytest.approx(0.5)


def test_wheel_speeds():
    assert wheel_speeds(0.09, 0.0, R, SEP) == pytest.approx((2.0, 2.0))
    left, right = wheel_speeds(0.0, 1.0, R, SEP)
    assert left == pytest.approx(-right)
    assert right * R == pytest.approx(SEP / 2)


def test_velocity_command_clamps_and_rate_limits():
    cmd = VelocityCommand(LIMITS)
    cmd.set_target(1.0, -5.0, now=0.0)
    assert cmd.target == (0.3, -1.57)
    v, w = cmd.update(now=0.01, dt=0.01)
    assert v == pytest.approx(0.02)
    assert w == pytest.approx(-0.03)
    for i in range(100):
        cmd.set_target(1.0, -5.0, now=0.02 + i * 0.01)
        v, w = cmd.update(now=0.02 + i * 0.01, dt=0.01)
    assert (v, w) == pytest.approx((0.3, -1.57))


def test_velocity_command_times_out():
    cmd = VelocityCommand(LIMITS)
    assert cmd.update(now=0.0, dt=0.1) == (0.0, 0.0)  # no command yet
    cmd.set_target(0.3, 0.0, now=0.0)
    for i in range(1, 10):
        v, _ = cmd.update(now=i * 0.05, dt=0.05)
    assert v == pytest.approx(0.3)
    v, _ = cmd.update(now=0.6, dt=0.05)
    assert v == pytest.approx(0.2)  # target dropped, decelerating
    for i in range(10):
        v, _ = cmd.update(now=0.65 + i * 0.05, dt=0.05)
    assert v == 0.0


def drive(odom: WheelOdometry, v: float, w: float, seconds: float, dt: float = 0.01) -> None:
    left, right = odom._last or (0.0, 0.0)
    wl, wr = wheel_speeds(v, w, R, SEP)
    steps = round(seconds / dt)
    dt = seconds / steps
    for _ in range(steps):
        left, right = wrap_angle(left + wl * dt), wrap_angle(right + wr * dt)  # joints wrap at +-pi
        odom.update(left, right, dt)


def test_wheel_odometry_straight_and_turn():
    odom = WheelOdometry(R, SEP)
    odom.update(0.0, 0.0, 0.0)
    drive(odom, 0.2, 0.0, 5.0)  # wheels turn ~22 rad: many wraps
    assert (odom.pose.x, odom.pose.y, odom.pose.yaw) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    assert odom.v == pytest.approx(0.2)
    drive(odom, 0.0, 1.0, math.pi / 2)
    assert odom.pose.yaw == pytest.approx(math.pi / 2)
    assert odom.w == pytest.approx(1.0)
    drive(odom, 0.1, 0.0, 2.0)
    assert (odom.pose.x, odom.pose.y) == pytest.approx((1.0, 0.2), abs=1e-9)


def test_wheel_odometry_arc():
    odom = WheelOdometry(R, SEP)
    odom.update(0.0, 0.0, 0.0)
    drive(odom, 0.1, 0.5, math.pi / 0.5, dt=0.001)  # half circle of radius 0.2
    assert (odom.pose.x, odom.pose.y) == pytest.approx((0.0, 0.4), abs=1e-6)
    assert abs(odom.pose.yaw) == pytest.approx(math.pi)


def test_noisy_odometry_without_noise_is_relative_ground_truth():
    odom = NoisyOdometry(0.0, 0.0, seed=1)
    start = Pose2D(2.0, 1.0, math.pi / 2)
    assert odom.update(start) == Pose2D(0.0, 0.0, 0.0)
    p = odom.update(Pose2D(2.0, 3.0, math.pi))
    assert (p.x, p.y, p.yaw) == pytest.approx((2.0, 0.0, math.pi / 2))


def test_noisy_odometry_drift():
    path = [Pose2D(0.01 * i, 0.0, 0.0) for i in range(200)]
    a, b = NoisyOdometry(0.01, 0.01, seed=7), NoisyOdometry(0.01, 0.01, seed=7)
    pa = [a.update(p) for p in path]
    pb = [b.update(p) for p in path]
    assert pa == pb  # seeded: reproducible
    assert pa[-1] != Pose2D(path[-1].x, 0.0, 0.0)
    drift = Pose2D(a.drift.x, a.drift.y, a.drift.yaw)
    for _ in range(50):
        a.update(path[-1])  # standing still: no new drift
    assert a.drift == drift



def test_odometry_reset():
    wheel = WheelOdometry(R, SEP)
    wheel.update(0.0, 0.0, 0.0)
    drive(wheel, 0.2, 0.0, 1.0)
    wheel.reset()
    assert wheel.pose == Pose2D() and wheel.v == 0.0
    wheel.update(5.0, -3.0, 0.01)  # the wheels have turned since: not a step
    assert wheel.pose == Pose2D()

    fused = NoisyOdometry(0.01, 0.01, seed=3)
    for i in range(100):
        fused.update(Pose2D(0.01 * i, 0.0, 0.0))
    spawn = Pose2D(1.0, 2.0, math.pi / 2)
    fused.reset(spawn)
    assert fused.drift == Pose2D()
    p = fused.update(Pose2D(1.0, 2.5, math.pi / 2))  # robot teleported back to the spawn, then moved
    assert (p.x, p.y, p.yaw) == pytest.approx((0.5, 0.0, 0.0), abs=0.05)
    fused.reset()
    assert fused.update(Pose2D(3.0, 3.0, 1.0)) == Pose2D()  # no origin: the next pose


def test_teleport_keeps_odometry_drift():
    fused = NoisyOdometry(0.01, 0.01, seed=3)
    for i in range(100):
        fused.update(Pose2D(0.01 * i, 0.0, 0.0))
    drift = Pose2D(fused.drift.x, fused.drift.y, fused.drift.yaw)
    assert drift != Pose2D()
    fused.teleported()
    p = fused.update(Pose2D(0.0, 0.0, 0.0))  # back at the origin, 1 m away
    assert fused.drift == drift  # the jump added nothing
    assert (p.x, p.y, p.yaw) == pytest.approx((drift.x, drift.y, drift.yaw))

def test_relative_pose_and_quaternions():
    p = relative_pose(Pose2D(1.0, 1.0, math.pi / 2), Pose2D(1.0, 2.0, 0.0))
    assert (p.x, p.y, p.yaw) == pytest.approx((1.0, 0.0, -math.pi / 2))
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.1):
        assert yaw_from_quat(*quat_from_yaw(yaw)) == pytest.approx(yaw)
