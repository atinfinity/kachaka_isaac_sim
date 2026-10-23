"""Sensor degradation (config/robot.yaml `degradation`), all off by default.

Harder sensor conditions, to see how SLAM, AMCL and Nav2 cope:

- LiDAR: surfaces it cannot see (glass), dark surfaces it sees only up
  close, random dropouts and noise that grows with range.
- Odometry: floor areas where the wheels slip (the odometry reads more travel
  than the robot made), and scale errors in distance and turn.
- Cameras: pixel noise, exposure that wanders and motion blur while turning.
- IMU: gyro and accelerometer biases that wander.

The values are knobs for tests, not measurements of the real robot.
Plain Python (numpy): no Isaac Sim.
"""

from __future__ import annotations

import fnmatch
import math
import random
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from kachaka_isaac_sim.drive import Pose2D, relative_pose, wrap_angle


def _get(cfg, name, default):
    v = getattr(cfg, name, default) if cfg is not None else default
    return default if v is None else v


# --- LiDAR -----------------------------------------------------------------

VISIBLE, INVISIBLE, DARK = 0, 1, 2


class SurfaceClasses:
    """Which collision prims the LiDAR sees through (`invisible`: glass) or
    sees only within `dark_range` (`dark`: black, low-reflectivity surfaces).
    Patterns are fnmatch patterns on prim paths ("/World/Room/Glass*"); a
    pattern matches a prim and everything under it."""

    def __init__(self, invisible=(), dark=()):
        self.invisible = list(invisible or ())
        self.dark = list(dark or ())
        self._cache: dict[str, int] = {}

    def __bool__(self) -> bool:
        return bool(self.invisible or self.dark)

    @staticmethod
    def _match(path: str, patterns) -> bool:
        return any(fnmatch.fnmatchcase(path, p) or fnmatch.fnmatchcase(path, p.rstrip("/") + "/*")
                   for p in patterns)

    def classify(self, *paths: str) -> int:
        """The class of a hit, from its collider and rigid-body paths."""
        key = "|".join(paths)
        c = self._cache.get(key)
        if c is None:
            c = INVISIBLE if any(self._match(p, self.invisible) for p in paths if p) else \
                DARK if any(self._match(p, self.dark) for p in paths if p) else VISIBLE
            self._cache[key] = c
        return c


@dataclass
class LidarDegradation:
    surfaces: SurfaceClasses = field(default_factory=SurfaceClasses)
    dark_range: float = 1.5  # m, a dark surface is seen up to here
    dropout: float = 0.0  # chance a ray returns nothing
    noise_per_m: float = 0.0  # extra range noise stddev per metre of range

    @property
    def enabled(self) -> bool:
        return bool(self.surfaces) or self.dropout > 0 or self.noise_per_m > 0

    def noisy_range(self, r: float, dark: bool, base_stddev: float, rng: random.Random) -> float:
        """The reported range of an exact hit at `r` (inf: nothing seen)."""
        if r == math.inf or (dark and r > self.dark_range):
            return math.inf
        if self.dropout > 0 and rng.random() < self.dropout:
            return math.inf
        return r + rng.gauss(0.0, base_stddev + self.noise_per_m * r)


def lidar_degradation(cfg) -> LidarDegradation:
    return LidarDegradation(SurfaceClasses(_get(cfg, "invisible", []), _get(cfg, "dark", [])),
                            float(_get(cfg, "dark_range", 1.5)), float(_get(cfg, "dropout", 0.0)),
                            float(_get(cfg, "noise_per_m", 0.0)))


# --- Odometry ----------------------------------------------------------------

@dataclass(frozen=True)
class SlipArea:
    """A floor rectangle in the world frame where the wheels slip: the
    odometry reads (1 + slip) times the travel and turn the robot made."""

    x0: float
    y0: float
    x1: float
    y1: float
    slip: float

    def contains(self, x: float, y: float) -> bool:
        return min(self.x0, self.x1) <= x <= max(self.x0, self.x1) and \
            min(self.y0, self.y1) <= y <= max(self.y0, self.y1)


def _compose(a: Pose2D, d: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * d.x - s * d.y, a.y + s * d.x + c * d.y, wrap_angle(a.yaw + d.yaw))


class OdometryDegradation:
    """Turns the ground-truth pose into the pose the odometry believes: each
    step's motion, in the robot frame, scaled by the distance and turn scale
    errors and by the slip of the area the robot is in. The result is fed
    to NoisyOdometry in place of ground truth; its random drift comes on top.
    """

    def __init__(self, slip_areas=(), distance_scale_error: float = 0.0, turn_scale_error: float = 0.0):
        self.areas = [a if isinstance(a, SlipArea) else SlipArea(*a) for a in slip_areas]
        self.k_dist = float(distance_scale_error)
        self.k_turn = float(turn_scale_error)
        self.believed: Pose2D | None = None
        self._last_gt: Pose2D | None = None
        self._error: Pose2D | None = None  # kept over a teleport

    @property
    def enabled(self) -> bool:
        return bool(self.areas) or self.k_dist != 0 or self.k_turn != 0

    def slip_at(self, x: float, y: float) -> float:
        return max((a.slip for a in self.areas if a.contains(x, y)), default=0.0)

    def scale(self, x: float, y: float) -> tuple[float, float]:
        """The (distance, turn) factors the odometry reads at (x, y)."""
        slip = self.slip_at(x, y)
        return (1 + self.k_dist) * (1 + slip), (1 + self.k_turn) * (1 + slip)

    def reset(self) -> None:
        """Start again with no error (the odometry is cleared)."""
        self.believed = self._last_gt = self._error = None

    def teleported(self) -> None:
        """The robot was moved without driving: the error so far stays, as
        the believed pose seen from the true one, and is put on the next
        pose."""
        if self.believed is not None and self._last_gt is not None:
            self._error = relative_pose(self._last_gt, self.believed)
        self._last_gt = None

    def shift(self, dx: float, dy: float) -> None:
        """Carried by (dx, dy) without driving (an elevator ride): both poses move."""
        for p in (self.believed, self._last_gt):
            if p is not None:
                p.x += dx
                p.y += dy

    def update(self, gt: Pose2D) -> Pose2D:
        if self._last_gt is None:
            err, self._error = self._error, None
            self.believed = _compose(gt, err) if err is not None else Pose2D(gt.x, gt.y, gt.yaw)
        else:
            d = relative_pose(self._last_gt, gt)
            kd, kt = self.scale((gt.x + self._last_gt.x) / 2, (gt.y + self._last_gt.y) / 2)
            self.believed = _compose(self.believed, Pose2D(d.x * kd, d.y * kd, d.yaw * kt))
        self._last_gt = Pose2D(gt.x, gt.y, gt.yaw)
        return Pose2D(self.believed.x, self.believed.y, self.believed.yaw)


def odometry_degradation(cfg) -> OdometryDegradation:
    areas = []
    for a in _get(cfg, "slip_areas", []):
        a = vars(a) if isinstance(a, SimpleNamespace) else a  # config.py makes dicts namespaces
        areas.append(SlipArea(*map(float, a["rect"]), float(a.get("slip", 0.0))) if isinstance(a, dict)
                     else SlipArea(*map(float, a)))
    return OdometryDegradation(areas, float(_get(cfg, "distance_scale_error", 0.0)),
                               float(_get(cfg, "turn_scale_error", 0.0)))


# --- Cameras ---------------------------------------------------------------

class CameraDegradation:
    """Pixel noise, a wandering exposure and horizontal motion blur, applied
    to RGB frames (HxWx3 uint8)."""

    def __init__(self, noise: float = 0.0, exposure_jitter: float = 0.0, exposure_time_constant: float = 5.0,
                 exposure_time: float = 0.0, seed: int | None = None):
        self.noise = float(noise)  # stddev, 8-bit levels
        self.jitter = float(exposure_jitter)  # stationary stddev of the exposure, in stops
        self.tau = max(float(exposure_time_constant), 1e-3)  # s
        self.exposure_time = float(exposure_time)  # s, how long the shutter stays open (blur)
        self.rng = np.random.default_rng(seed)
        self.stops = 0.0
        self._t: float | None = None

    @property
    def enabled(self) -> bool:
        return self.noise > 0 or self.jitter > 0 or self.exposure_time > 0

    def step_exposure(self, t: float) -> float:
        """Advance the exposure (an Ornstein-Uhlenbeck walk in stops) to time t."""
        if self.jitter > 0 and self._t is not None and t > self._t:
            a = math.exp(-(t - self._t) / self.tau)
            self.stops = a * self.stops + self.jitter * math.sqrt(1 - a * a) * float(self.rng.normal())
        self._t = t
        return self.stops

    def blur_px(self, yaw_rate: float, fx: float) -> int:
        """Blur length in pixels: how far the image sweeps while the shutter is open."""
        return round(abs(yaw_rate) * self.exposure_time * fx)

    def apply(self, img: np.ndarray, t: float, yaw_rate: float = 0.0, fx: float = 0.0) -> np.ndarray:
        if not self.enabled:
            return img
        out = img.astype(np.float32)
        n = self.blur_px(yaw_rate, fx)
        if n > 1:
            out = box_blur_x(out, n)
        stops = self.step_exposure(t)
        if stops:
            out *= 2.0 ** stops
        if self.noise > 0:
            out += self.rng.normal(0.0, self.noise, out.shape).astype(np.float32)
        return np.clip(out, 0, 255).round().astype(np.uint8)


def box_blur_x(img: np.ndarray, n: int) -> np.ndarray:
    """Mean of n neighbouring pixels along each row (edges repeat)."""
    n = min(int(n), img.shape[1])
    if n <= 1:
        return img
    left = n // 2
    pad = np.pad(img, [(0, 0), (left, n - 1 - left)] + [(0, 0)] * (img.ndim - 2), mode="edge")
    c = np.cumsum(pad, axis=1, dtype=np.float64)
    c = np.concatenate([np.zeros_like(c[:, :1]), c], axis=1)
    return ((c[:, n:] - c[:, :-n]) / n).astype(img.dtype)


def camera_degradation(cfg, seed: int | None = None) -> CameraDegradation:
    return CameraDegradation(float(_get(cfg, "noise", 0.0)), float(_get(cfg, "exposure_jitter", 0.0)),
                             float(_get(cfg, "exposure_time_constant", 5.0)), float(_get(cfg, "exposure_time", 0.0)),
                             seed)


# --- IMU ---------------------------------------------------------------------

class BiasWalk:
    """A 3-axis sensor bias: drawn once (stddev `initial`), then a random walk
    of `walk` per sqrt(second)."""

    def __init__(self, initial: float = 0.0, walk: float = 0.0, rng: random.Random | None = None):
        self.rng = rng or random.Random()
        self.walk = float(walk)
        self.value = np.array([self.rng.gauss(0.0, initial) if initial > 0 else 0.0 for _ in range(3)])

    def step(self, dt: float) -> np.ndarray:
        if self.walk > 0 and dt > 0:
            s = self.walk * math.sqrt(dt)
            self.value = self.value + np.array([self.rng.gauss(0.0, s) for _ in range(3)])
        return self.value


@dataclass
class ImuDegradation:
    gyro: BiasWalk
    accel: BiasWalk
    yaw_error: float = 0.0  # rad, the gyro z bias integrated into the orientation

    @property
    def enabled(self) -> bool:
        return bool(self.gyro.value.any() or self.gyro.walk or self.accel.value.any() or self.accel.walk)

    def step(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """(gyro bias, accel bias) after dt; the yaw error grows by the gyro z bias."""
        g, a = self.gyro.step(dt), self.accel.step(dt)
        self.yaw_error = wrap_angle(self.yaw_error + float(g[2]) * dt)
        return g, a


def imu_degradation(cfg, seed: int | None = None) -> ImuDegradation:
    rng = random.Random(None if seed is None else seed + 7)
    return ImuDegradation(BiasWalk(float(_get(cfg, "gyro_bias", 0.0)), float(_get(cfg, "gyro_bias_walk", 0.0)), rng),
                          BiasWalk(float(_get(cfg, "accel_bias", 0.0)), float(_get(cfg, "accel_bias_walk", 0.0)),
                                   rng))


def section(cfg: SimpleNamespace, name: str):
    """`degradation.<name>` of the robot config, or None."""
    d = getattr(cfg, "degradation", None)
    return getattr(d, name, None) if d is not None else None
