"""Front/back RGB cameras and the ToF depth camera.

Each camera is a USD Camera under base_link at the URDF optical-frame pose,
rendered through a Replicator render product, and published from our own
rclpy node so names, frames and QoS match the real bridge.
Import only after SimulationApp has started and omni.replicator.core is enabled.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
from pxr import Gf, UsdGeom

from .physics_sensors import matrix_to_quat, quat_to_matrix

# A USD camera looks down -Z with +Y up; a ROS optical frame looks down +Z
# with +Y down. Rotating 180 deg about X maps one to the other.
_USD_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0])
_APERTURE_MM = 20.955


class SimCamera:
    def __init__(self, stage, name: str, kind: str, cfg: SimpleNamespace, parent_path: str, mount_xyz, mount_quat,
                 near: float, far: float):
        import omni.replicator.core as rep

        self.name = name
        self.kind = kind  # "rgb" or "depth"
        self.cfg = cfg
        self.width, self.height = int(cfg.width), int(cfg.height)
        self.period = 1.0 / cfg.rate
        self.next_time = 0.0

        path = f"{parent_path}/sim_{name}"
        cam = UsdGeom.Camera.Define(stage, path)
        rot = quat_to_matrix(*mount_quat) @ _USD_FROM_OPTICAL
        w, x, y, z = matrix_to_quat(rot)
        xf = UsdGeom.Xformable(cam.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*mount_xyz))
        xf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        hfov = math.radians(cfg.hfov_deg)
        focal = _APERTURE_MM / (2 * math.tan(hfov / 2))
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(_APERTURE_MM)
        cam.CreateVerticalApertureAttr(_APERTURE_MM * self.height / self.width)
        # A short near clip: the lenses sit right behind the body shell.
        cam.CreateClippingRangeAttr(Gf.Vec2f(near, far))

        self.fx = self.width / (2 * math.tan(hfov / 2))
        self.render_product = rep.create.render_product(path, (self.width, self.height))
        annotator = "rgb" if kind == "rgb" else "distance_to_image_plane"
        self.annotator = rep.AnnotatorRegistry.get_annotator(annotator)
        self.annotator.attach([self.render_product])

    def due(self, t: float) -> bool:
        if t + 1e-9 < self.next_time:
            return False
        self.next_time = max(self.next_time + self.period, t)
        return True

    def read(self) -> np.ndarray | None:
        """Latest frame: HxWx3 uint8 for RGB, HxW float32 metres for depth."""
        data = self.annotator.get_data()
        if data is None or getattr(data, "size", 0) == 0:
            return None
        if self.kind == "rgb":
            return np.ascontiguousarray(data[:, :, :3])
        depth = np.asarray(data, dtype=np.float32)
        cfg = self.cfg
        invalid = ~np.isfinite(depth) | (depth < cfg.min_range) | (depth > cfg.max_range)
        return np.where(invalid, 0.0, depth).astype(np.float32)


def create_cameras(stage, cfg: SimpleNamespace, parent_path: str, joints: list[dict]) -> list[SimCamera]:
    from .urdf_tools import frame_pose

    cams = []
    for name in ("front", "back"):
        c = getattr(cfg.cameras, name)
        if getattr(c, "enabled", True):
            cams.append(SimCamera(stage, f"{name}_camera", "rgb", c, parent_path, *frame_pose(joints, c.frame_id),
                                  near=float(c.near_clip), far=100.0))
    t = cfg.tof
    if getattr(t, "enabled", True):
        cams.append(SimCamera(stage, "tof_camera", "depth", t, parent_path, *frame_pose(joints, t.frame_id),
                              near=float(t.near_clip), far=float(t.max_range) * 2))
    return cams


class RecordCamera:
    """A bird's-eye camera that saves PNG frames, for demo videos. It is
    fixed, or with `follow` moves with the robot: eye and target are then
    relative to the robot's position.

    Frames are written by a background thread; `frames.csv` lists each file
    with its wall-clock and sim time so it can be synced with other captures.
    """

    def __init__(self, stage, out_dir, eye, target, rate: float = 10.0, width: int = 960, height: int = 720,
                 hfov_deg: float = 48.0, follow: bool = False, near: float = 0.05):
        import csv
        import queue
        import threading
        from pathlib import Path

        import omni.replicator.core as rep

        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.period = 1.0 / rate
        self.next_time = 0.0
        self.count = 0

        path = "/World/record_camera"
        cam = UsdGeom.Camera.Define(stage, path)
        eye, target = np.asarray(eye, float), np.asarray(target, float)
        f = target - eye
        f /= np.linalg.norm(f)
        x = np.cross(f, [0.0, 0.0, 1.0])
        x /= np.linalg.norm(x)
        y = np.cross(x, f)
        w, qx, qy, qz = matrix_to_quat(np.column_stack([x, y, -f]))  # USD cameras look down -Z
        xf = UsdGeom.Xformable(cam.GetPrim())
        self.eye, self.follow = eye, follow
        self._translate = xf.AddTranslateOp()
        self._translate.Set(Gf.Vec3d(*eye))
        xf.AddOrientOp().Set(Gf.Quatf(w, qx, qy, qz))
        cam.CreateFocalLengthAttr(_APERTURE_MM / (2 * math.tan(math.radians(hfov_deg) / 2)))
        cam.CreateHorizontalApertureAttr(_APERTURE_MM)
        cam.CreateVerticalApertureAttr(_APERTURE_MM * height / width)
        cam.CreateClippingRangeAttr(Gf.Vec2f(near, 100.0))  # a far near plane cuts away ceilings
        self.render_product = rep.create.render_product(path, (width, height))
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach([self.render_product])

        self._csv_file = open(self.out / "frames.csv", "w", newline="")  # noqa: SIM115 (closed in close())
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["file", "wall_time", "sim_time"])
        self._queue: queue.Queue = queue.Queue(maxsize=64)
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()

    def _write_loop(self) -> None:
        from PIL import Image

        while True:
            item = self._queue.get()
            if item is None:
                return
            name, rgb = item
            Image.fromarray(rgb).save(self.out / name, compress_level=1)

    def capture(self, sim_time: float, robot_xy: tuple[float, float] | None = None) -> None:
        import time

        if self.follow and robot_xy is not None:
            self._translate.Set(Gf.Vec3d(self.eye[0] + robot_xy[0], self.eye[1] + robot_xy[1], self.eye[2]))
        if sim_time + 1e-9 < self.next_time:
            return
        self.next_time = max(self.next_time + self.period, sim_time)
        data = self.annotator.get_data()
        if data is None or getattr(data, "size", 0) == 0:
            return
        name = f"{self.count:06d}.png"
        self.count += 1
        self._queue.put((name, np.ascontiguousarray(data[:, :, :3])))
        self._csv.writerow([name, f"{time.time():.3f}", f"{sim_time:.3f}"])
        self._csv_file.flush()  # readable while the simulator still runs

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=30)
        self._csv_file.close()


class FrontDetector:
    """Ground truth for simulated object detection (kachaka_isaac_sim.detection)
    from the front camera's render product: the prim seen at every pixel and
    its distance to the camera."""

    def __init__(self, camera: SimCamera, targets, params, seed: int | None = None):
        import random

        import omni.replicator.core as rep

        self.camera = camera
        self.targets = targets
        self.params = params
        self.rng = random.Random(seed)
        self.instances = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation")
        self.distance = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        for a in (self.instances, self.distance):
            a.attach([camera.render_product])

    def detect(self) -> list | None:
        """Detections in the latest frame; None before the first one."""
        from .detection import detect

        seg, dist = self.instances.get_data(), self.distance.get_data()
        ids = seg.get("data") if isinstance(seg, dict) else None
        if ids is None or ids.size == 0 or dist is None or getattr(dist, "size", 0) == 0:
            return None
        id_to_path = {int(k): v for k, v in seg.get("info", {}).get("idToLabels", {}).items()}
        return detect(np.asarray(ids), id_to_path, np.asarray(dist, dtype=np.float32), self.targets, self.params,
                      self.rng)
