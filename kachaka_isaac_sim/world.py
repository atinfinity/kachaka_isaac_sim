"""Stage building: physics scene, floor, simple room, robot and contact materials.

Import only after SimulationApp has started.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

ROBOT_PATH = "/World/kachaka"
BASE_LINK = f"{ROBOT_PATH}/Geometry/base_footprint/base_link"
WHEEL_LINKS = ("base_l_drive_wheel_link", "base_r_drive_wheel_link")
CASTER_SHAPES = ("caster_front_left", "caster_front_right", "caster_rear")  # spheres on base_link


def setup_stage(stage: Usd.Stage, physics_hz: int) -> None:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim("/World", "Xform"))

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateTimeStepsPerSecondAttr(physics_hz)
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableGPUDynamicsAttr(False)  # one small robot: CPU is faster
    px.CreateBroadphaseTypeAttr("MBP")

    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(800.0)


def add_ground(stage: Usd.Stage) -> None:
    from isaacsim.core.experimental.objects import GroundPlane

    GroundPlane("/World/GroundPlane")


def add_box(stage: Usd.Stage, path: str, center, size, color=(0.8, 0.8, 0.78)) -> None:
    """Static box collider as an explicit mesh (scaled Cube colliders misbehave)."""
    hx, hy, hz = (s / 2 for s in size)
    pts = [Gf.Vec3f(x * hx, y * hy, z * hz) for z in (-1, 1) for y in (-1, 1) for x in (-1, 1)]
    faces = [0, 2, 3, 1, 4, 5, 7, 6, 0, 1, 5, 4, 2, 6, 7, 3, 0, 4, 6, 2, 1, 3, 7, 5]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    mesh.AddTranslateOp().Set(Gf.Vec3d(*center))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


def add_simple_room(stage: Usd.Stage, size_x: float = 5.0, size_y: float = 4.0, height: float = 0.8) -> None:
    """A walled room with a couple of obstacles, enough to test teleop and SLAM."""
    t = 0.05
    root = "/World/Room"
    UsdGeom.Xform.Define(stage, root)
    add_box(stage, f"{root}/wall_n", (0, size_y / 2 + t / 2, height / 2), (size_x + 2 * t, t, height))
    add_box(stage, f"{root}/wall_s", (0, -size_y / 2 - t / 2, height / 2), (size_x + 2 * t, t, height))
    add_box(stage, f"{root}/wall_e", (size_x / 2 + t / 2, 0, height / 2), (t, size_y, height))
    add_box(stage, f"{root}/wall_w", (-size_x / 2 - t / 2, 0, height / 2), (t, size_y, height))
    add_box(stage, f"{root}/box_a", (1.4, 1.0, 0.25), (0.4, 0.4, 0.5), (0.55, 0.4, 0.3))
    add_box(stage, f"{root}/box_b", (-1.2, -0.9, 0.35), (0.6, 0.3, 0.7), (0.3, 0.45, 0.6))
    add_box(stage, f"{root}/pillar", (-0.6, 1.1, 0.4), (0.2, 0.2, 0.8), (0.6, 0.6, 0.6))


def add_robot(stage: Usd.Stage, usd_path: Path, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Usd.Prim:
    prim = stage.DefinePrim(ROBOT_PATH, "Xform")
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    prim.GetReferences().AddReference(str(usd_path))
    # The importer leaves the Physics variant unselected; pick the PhysX one.
    prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    return prim


def _physics_material(stage: Usd.Stage, path: str, friction: float, combine: str) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateFrictionCombineModeAttr(combine)
    return mat


def _bind_physics_material(prim: Usd.Prim, mat: UsdShade.Material) -> None:
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics"
            )


def apply_contact_materials(stage: Usd.Stage, drive: SimpleNamespace) -> None:
    """Grippy tyres and near-frictionless casters, whatever the floor material.

    PhysX picks the combine mode with the higher priority (average < min <
    multiply < max), so "max" keeps tyre grip and "min" keeps casters slippery.
    """
    tyre = _physics_material(stage, "/World/PhysicsMaterials/tyre", drive.wheel_friction, "max")
    caster = _physics_material(stage, "/World/PhysicsMaterials/caster", drive.caster_friction, "min")
    for name in WHEEL_LINKS:
        _bind_physics_material(stage.GetPrimAtPath(f"{BASE_LINK}/{name}"), tyre)
    for name in CASTER_SHAPES:
        _bind_physics_material(stage.GetPrimAtPath(f"{BASE_LINK}/{name}"), caster)


def load_world_usd(stage: Usd.Stage, usd_path: str) -> None:
    prim = stage.DefinePrim("/World/Environment", "Xform")
    prim.GetReferences().AddReference(usd_path)

