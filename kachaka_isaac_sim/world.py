"""Stage building: physics scene, floor, simple room, shelves, charger, people, robot and contact
materials.

Import only after SimulationApp has started.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from kachaka_isaac_sim.meshes import read_stl
from kachaka_isaac_sim.scenes import (
    ISAAC_WORLDS,
    PERSON_COLLIDER,
    ROOM_BOXES,
    ROOM_HEIGHT,
    ROOM_SIZE,
    ROOM_WALL,
    SHELF_CASTER_RADIUS,
    SHELF_LEG_SIZE,
    SHELF_LEG_XY,
    SHELF_PLATES,
    SHELF_SIZE,
)

ROBOT_PATH = "/World/kachaka"
BASE_LINK = f"{ROBOT_PATH}/Geometry/base_footprint/base_link"
WHEEL_LINKS = ("base_l_drive_wheel_link", "base_r_drive_wheel_link")
CASTER_SHAPES = ("caster_front_left", "caster_front_right", "caster_rear")  # spheres on base_link


def setup_stage(stage: Usd.Stage, physics_hz: int, dome_intensity: float = 800.0) -> None:
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
    light.CreateIntensityAttr(dome_intensity)


def resolve_world(name: str) -> str:
    """Return a USD path or URL for a named Isaac scene, else `name` itself."""
    if name in ISAAC_WORLDS:
        from isaacsim.storage.native import get_assets_root_path

        root = get_assets_root_path()
        if root is None:
            raise RuntimeError("Isaac Sim asset server is not reachable")
        return f"{root}/{ISAAC_WORLDS[name]}"
    return name


def add_ground(stage: Usd.Stage, visible: bool = True, z: float = 0.0) -> None:
    """Flat collider at z=0 (or below, under a floor with drops). Hidden under
    loaded scenes whose own floor may lack collision; visibility does not
    affect physics."""
    from isaacsim.core.experimental.objects import GroundPlane

    GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, z])
    if not visible:
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/GroundPlane")).MakeInvisible()


def add_box(stage: Usd.Stage, path: str, center, size, color=(0.8, 0.8, 0.78), collider: bool = True,
            approximation: str = "convexHull") -> None:
    """Static box as an explicit mesh (scaled Cube colliders misbehave)."""
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
    if collider:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(approximation)


def add_floor_with_drops(stage: Usd.Stage, holes, extent: float = 20.0, thickness: float = 0.02,
                         overlap: float = 0.001) -> None:
    """A floor at z=0 made of boxes around the holes (hazards.floor_tiles);
    the ground plane lies lower (add_ground(z=-depth)). The tiles are exact
    triangle meshes: cooked convex hulls of such large thin boxes come out a
    few millimetres short at the edges, so the step sensors saw a drop at the
    seams. They also overlap by `overlap`."""
    from .hazards import floor_tiles

    UsdGeom.Xform.Define(stage, "/World/Floor")
    for i, (x0, y0, x1, y1) in enumerate(floor_tiles(list(holes), extent)):
        add_box(stage, f"/World/Floor/tile_{i:02d}", ((x0 + x1) / 2, (y0 + y1) / 2, -thickness / 2),
                (x1 - x0 + 2 * overlap, y1 - y0 + 2 * overlap, thickness), (0.62, 0.66, 0.72),
                approximation="none")


def enable_contact_report(stage: Usd.Stage, path: str = BASE_LINK) -> None:
    """PhysX contact reports for the robot body (physics_sensors.ContactSensor).
    Apply before the simulation starts."""
    from pxr import PhysxSchema

    api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
    api.CreateThresholdAttr().Set(0.0)


def add_simple_room(stage: Usd.Stage) -> None:
    """A walled room with a couple of obstacles, enough to test teleop and SLAM
    (scenes.ROOM_*)."""
    (size_x, size_y), height, t = ROOM_SIZE, ROOM_HEIGHT, ROOM_WALL
    root = "/World/Room"
    UsdGeom.Xform.Define(stage, root)
    add_box(stage, f"{root}/wall_n", (0, size_y / 2 + t / 2, height / 2), (size_x + 2 * t, t, height))
    add_box(stage, f"{root}/wall_s", (0, -size_y / 2 - t / 2, height / 2), (size_x + 2 * t, t, height))
    add_box(stage, f"{root}/wall_e", (size_x / 2 + t / 2, 0, height / 2), (t, size_y, height))
    add_box(stage, f"{root}/wall_w", (-size_x / 2 - t / 2, 0, height / 2), (t, size_y, height))
    for name, center, size, color in ROOM_BOXES:
        add_box(stage, f"{root}/{name}", center, size, color)


def add_shelf(stage: Usd.Stage, path: str, stl: Path, pose, cfg: SimpleNamespace) -> str:
    """Kachaka Shelf as a rigid body on four casters, with the mesh as its
    visual. `pose` (x, y, yaw) is the base_footprint pose of a robot docked
    under it. Simple colliders keep the gap under the bottom plate open (the
    robot's body is 0.095 m tall). Also adds the disabled fixed joint that
    tows the shelf while docked (see `ShelfJoint`). Returns the body path."""
    x, y, yaw = pose
    body = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(body.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(float(cfg.mass))

    v = read_stl(stl, 0.001)
    mesh = UsdGeom.Mesh.Define(stage, f"{path}/visual")
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
    mesh.CreateFaceVertexCountsAttr([3] * (len(v) // 3))
    mesh.CreateFaceVertexIndicesAttr(list(range(len(v))))
    mesh.CreateSubdivisionSchemeAttr("none")
    # Matte charcoal steel and boards, as on the real shelf (linear RGB).
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.11, 0.113, 0.12)])
    UsdGeom.Xformable(mesh.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, -float(v[:, 2].min())))

    # Colliders, invisible: the mesh shows the shelf.
    def box(name, center, size):
        add_box(stage, f"{path}/{name}", center, size)
        UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/{name}")).MakeInvisible()

    lx, ly = SHELF_LEG_XY
    top = SHELF_PLATES[-1][1]
    caster_mat = _physics_material(stage, "/World/PhysicsMaterials/shelf_caster", cfg.caster_friction, "min")
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        z0 = 2 * SHELF_CASTER_RADIUS
        box(f"leg_{i}", (sx * lx, sy * ly, (z0 + top) / 2), (*SHELF_LEG_SIZE, top - z0))
        sphere = UsdGeom.Sphere.Define(stage, f"{path}/caster_{i}")
        sphere.CreateRadiusAttr(SHELF_CASTER_RADIUS)
        sphere.AddTranslateOp().Set(Gf.Vec3d(sx * lx, sy * ly, SHELF_CASTER_RADIUS))
        sphere.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        _bind_physics_material(sphere.GetPrim(), caster_mat)
    for name, (z0, z1) in zip(("bottom", "middle", "top"), SHELF_PLATES):
        box(f"plate_{name}", (0.0, 0.0, (z0 + z1) / 2), (*SHELF_SIZE, z1 - z0))

    ShelfJoint.define(stage, path)
    return path


class ShelfJoint:
    """Fixed joint from the robot's base_link to a shelf, disabled until the
    robot docks. It exists from the start so docking only changes attributes
    and never adds prims to a running simulation."""

    def __init__(self, stage: Usd.Stage, shelf_path: str):
        self.joint = UsdPhysics.FixedJoint.Get(stage, f"{shelf_path}_joint")

    @staticmethod
    def define(stage: Usd.Stage, shelf_path: str) -> None:
        j = UsdPhysics.FixedJoint.Define(stage, f"{shelf_path}_joint")
        j.CreateBody0Rel().SetTargets([BASE_LINK])
        j.CreateBody1Rel().SetTargets([shelf_path])
        j.CreateJointEnabledAttr(False)
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateCollisionEnabledAttr(False)
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    def attach(self, rel_xyz, rel_quat) -> None:
        """Lock the shelf at `rel_xyz`, `rel_quat` (w, x, y, z) in base_link."""
        self.joint.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float, rel_xyz)))
        w, x, y, z = map(float, rel_quat)
        self.joint.GetLocalRot0Attr().Set(Gf.Quatf(w, x, y, z))
        self.joint.GetJointEnabledAttr().Set(True)

    def detach(self) -> None:
        self.joint.GetJointEnabledAttr().Set(False)


def disable_pin_collision(stage: Usd.Stage) -> None:
    """The docking pin pokes up into the shelf's bottom plate; real shelves
    have a hole there. Without one in the colliders, let the pin pass."""
    for p in Usd.PrimRange(stage.GetPrimAtPath(f"{BASE_LINK}/docking_link")):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)


def add_charger(stage: Usd.Stage, path: str, pose, cfg: SimpleNamespace) -> None:
    """L-shaped charging dock: a thin floor plate with two contacts and a back
    panel with a green LED, dark grey like the real one.

    `pose` (x, y, yaw) is the base_footprint pose of a docked robot, which
    faces away from the panel. Only the panel collides; the plate is a few
    millimetres thick and stays visual so the wheels do not snag on it.
    """
    x, y, yaw = pose
    root = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(root.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    grey = (0.05, 0.052, 0.055)
    w, h, t = cfg.width, cfg.panel_height, cfg.panel_thickness
    face = -(cfg.robot_rear + cfg.gap)  # panel front face, in the dock frame
    add_box(stage, f"{path}/panel", (face - t / 2, 0.0, h / 2), (t, w, h), grey)
    plate = 0.004
    add_box(stage, f"{path}/plate", (face + cfg.plate_depth / 2, 0.0, plate / 2), (cfg.plate_depth, w, plate),
            grey, collider=False)
    for side in (-1, 1):
        add_box(stage, f"{path}/contact_{'l' if side > 0 else 'r'}",
                (face + cfg.plate_depth - 0.02, side * 0.04, plate + 0.002), (0.02, 0.012, 0.004),
                (0.55, 0.55, 0.5), collider=False)
    add_box(stage, f"{path}/led", (face - t / 2, 0.0, h + 0.002), (0.01, 0.03, 0.004), (0.1, 0.9, 0.2),
            collider=False)


def add_person(stage: Usd.Stage, path: str, pose, height: float = 1.7) -> None:
    """A standing person as a static mannequin of capsules, for object
    detection and for the LiDAR to see. `pose` (x, y, yaw): feet centre and
    facing direction. One invisible box collider covers the body."""
    x, y, yaw = pose
    root = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(root.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    k = height / 1.7
    skin, shirt, pants, shoes = (0.8, 0.6, 0.48), (0.2, 0.42, 0.65), (0.13, 0.14, 0.2), (0.06, 0.05, 0.05)

    def capsule(name, center, radius, length, color, axis="Z", scale=None):
        c = UsdGeom.Capsule.Define(stage, f"{path}/{name}")
        c.CreateRadiusAttr(radius * k)
        c.CreateHeightAttr(length * k)  # the cylinder part; the caps add 2 * radius
        c.CreateAxisAttr(axis)
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        cxf = UsdGeom.Xformable(c.GetPrim())
        cxf.AddTranslateOp().Set(Gf.Vec3d(*(v * k for v in center)))
        if scale:
            cxf.AddScaleOp().Set(Gf.Vec3f(*scale))

    for side, sy in (("l", 0.1), ("r", -0.1)):
        capsule(f"leg_{side}", (0.0, sy, 0.47), 0.065, 0.74, pants)
        capsule(f"shoe_{side}", (0.04, sy, 0.04), 0.04, 0.14, shoes, axis="X")
        capsule(f"arm_{side}", (0.0, sy * 2.3, 1.12), 0.045, 0.5, shirt)
        capsule(f"hand_{side}", (0.0, sy * 2.3, 0.8), 0.04, 0.04, skin)
    capsule("torso", (0.0, 0.0, 1.17), 0.16, 0.38, shirt, scale=(0.7, 1.0, 1.0))
    capsule("neck", (0.0, 0.0, 1.45), 0.05, 0.05, skin)
    head = UsdGeom.Sphere.Define(stage, f"{path}/head")
    head.CreateRadiusAttr(0.11 * k)
    head.CreateDisplayColorAttr([Gf.Vec3f(*skin)])
    UsdGeom.Xformable(head.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.58 * k))
    add_box(stage, f"{path}/collider", (0.0, 0.0, height / 2), (PERSON_COLLIDER[0] * k, PERSON_COLLIDER[1] * k, height))
    UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/collider")).MakeInvisible()


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

