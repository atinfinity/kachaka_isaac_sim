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
