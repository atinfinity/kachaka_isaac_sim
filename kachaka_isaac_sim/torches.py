"""Front and back torch lights. The real Kachaka has a torch next to its front
and its back camera, set with SetFrontTorchIntensity / SetBackTorchIntensity
(0-255) and with no RPC to read it back.

Each torch is a USD SphereLight with a cone (ShapingAPI) at the camera frame,
aimed along the camera. The cameras sit behind the body shell, so the light
casts no shadows: otherwise the shell would block it. The pxr import is lazy,
so the level logic runs without Isaac Sim.
"""

from __future__ import annotations

from types import SimpleNamespace

MAX_LEVEL = 255
NAMES = ("front", "back")


def clamp_level(level: int) -> int:
    return max(0, min(MAX_LEVEL, int(level)))


def light_intensity(level: int, max_intensity: float) -> float:
    """USD light intensity for a torch level (0-255); 0 is off."""
    return max_intensity * clamp_level(level) / MAX_LEVEL


class Torches:
    """Levels of both torches, and their USD lights once `create` has run."""

    def __init__(self, cfg: SimpleNamespace):
        self.cfg = cfg
        self.levels = dict.fromkeys(NAMES, 0)
        self._lights: dict = {}

    def create(self, stage, parent_path: str, joints: list[dict]) -> None:
        from pxr import Gf, UsdGeom, UsdLux

        from .cameras import _USD_FROM_OPTICAL
        from .physics_sensors import matrix_to_quat, quat_to_matrix
        from .urdf_tools import frame_pose

        c = self.cfg
        for name in NAMES:
            xyz, quat = frame_pose(joints, getattr(c, name).frame_id)
            light = UsdLux.SphereLight.Define(stage, f"{parent_path}/sim_{name}_torch")
            # USD lights shine down -Z like USD cameras.
            w, x, y, z = matrix_to_quat(quat_to_matrix(*quat) @ _USD_FROM_OPTICAL)
            xf = UsdGeom.Xformable(light.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            xf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
            light.CreateRadiusAttr(float(c.radius))
            light.CreateColorAttr(Gf.Vec3f(*c.color))
            light.CreateIntensityAttr(0.0)
            shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
            shaping.CreateShapingConeAngleAttr(float(c.cone_angle_deg))
            shaping.CreateShapingConeSoftnessAttr(float(c.cone_softness))
            UsdLux.ShadowAPI.Apply(light.GetPrim()).CreateShadowEnableAttr(False)
            self._lights[name] = light

    def set(self, name: str, level: int) -> int:
        """Set a torch (call between simulation steps). Returns the clamped level."""
        if name not in self.levels:
            raise ValueError(f"no torch '{name}'")
        level = clamp_level(level)
        self.levels[name] = level
        light = self._lights.get(name)
        if light is not None:
            light.GetIntensityAttr().Set(light_intensity(level, float(self.cfg.max_intensity)))
        return level

    def state(self) -> dict[str, int]:
        return dict(self.levels)

