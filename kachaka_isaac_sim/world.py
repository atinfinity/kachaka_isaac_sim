"""Stage building: physics scene, floor, simple room, shelves, charger, people, robot and contact
materials.

Import only after SimulationApp has started.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdSkel, Vt

from kachaka_isaac_sim.legs import LEG_RADIUS, TORSO_Z, capsule_length, leg_capsules
from kachaka_isaac_sim.meshes import read_stl
from kachaka_isaac_sim.scenes import (
    ISAAC_WORLDS,
    PERSON_COLLIDER,
    ROOM_BOXES,
    ROOM_HEIGHT,
    ROOM_WALL,
    SHELF_CASTER_RADIUS,
    SHELF_LEG_SIZE,
    SHELF_LEG_XY,
    SHELF_PLATES,
    SHELF_SIZE,
    room_walls,
)

ROBOT_PATH = "/World/kachaka"
BASE_LINK = f"{ROBOT_PATH}/Geometry/base_footprint/base_link"
WHEEL_LINKS = ("base_l_drive_wheel_link", "base_r_drive_wheel_link")
CASTER_SHAPES = ("caster_front_left", "caster_front_right", "caster_rear")  # spheres on base_link
MANNEQUIN_PANTS = (0.13, 0.14, 0.2)


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
            approximation: str = "convexHull", yaw: float = 0.0) -> None:
    """Static box as an explicit mesh (scaled Cube colliders misbehave),
    turned by `yaw` [rad] about z."""
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
    if yaw:
        mesh.AddRotateZOp().Set(math.degrees(yaw))
    if collider:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(approximation)


def add_moving_box(stage: Usd.Stage, path: str, pose, size, color) -> None:
    """A box as a kinematic body at `pose` (x, y, yaw) on the floor, which
    movers.Movers moves."""
    x, y, yaw = pose
    body = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(body.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr(True)
    add_box(stage, f"{path}/box", (0.0, 0.0, size[2] / 2), size, color)


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


def add_slab(stage: Usd.Stage, root: str, tiles, thickness: float = 0.1, overlap: float = 0.001) -> None:
    """A floor slab of boxes (x0, y0, x1, y1), top at z=0, for a stacked
    floor (the elevator's shaft is the gap between the tiles). Exact triangle
    meshes, overlapping, as add_floor_with_drops."""
    UsdGeom.Xform.Define(stage, root)
    for i, (x0, y0, x1, y1) in enumerate(tiles):
        add_box(stage, f"{root}/tile_{i:02d}", ((x0 + x1) / 2, (y0 + y1) / 2, -thickness / 2),
                (x1 - x0 + 2 * overlap, y1 - y0 + 2 * overlap, thickness), (0.62, 0.66, 0.72),
                approximation="none")


def add_platform(stage: Usd.Stage, path: str, rect, thickness: float, z: float) -> None:
    """The elevator car's floor as a kinematic box over `rect` (x0, y0, x1, y1),
    its middle at `z`; elevator.Elevator moves it through its translate op."""
    x0, y0, x1, y1 = rect
    body = UsdGeom.Xform.Define(stage, path)
    UsdGeom.Xformable(body.GetPrim()).AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, z))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr(True)
    add_box(stage, f"{path}/floor", (0.0, 0.0, 0.0), (x1 - x0, y1 - y0, thickness), (0.55, 0.57, 0.6))


def lift(stage: Usd.Stage, path: str, offset) -> None:
    """Move the prim at `path` by `offset` (x, y, z) in its parent's frame,
    before its own transform: a translate op (suffix "floor") put first, so
    code that sets the prim's own ops (doors) keeps working."""
    if not any(offset):
        return
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    ops = xf.GetOrderedXformOps()
    op = xf.AddTranslateOp(opSuffix="floor")
    op.Set(Gf.Vec3d(*offset))
    xf.SetXformOpOrder([op, *ops])


def lift_children(stage: Usd.Stage, root: str, offset_at) -> None:
    """Lift each child of `root` by `offset_at(x, y)`, where (x, y) is the
    child's place in the world (lift)."""
    prim = stage.GetPrimAtPath(root)
    if not prim.IsValid():
        return
    cache = UsdGeom.XformCache()
    for child in prim.GetChildren():
        if not child.IsA(UsdGeom.Xformable):
            continue
        t = cache.GetLocalToWorldTransform(child).ExtractTranslation()
        lift(stage, str(child.GetPath()), offset_at(t[0], t[1]))


def enable_contact_report(stage: Usd.Stage, path: str = BASE_LINK) -> None:
    """PhysX contact reports for the robot body (physics_sensors.ContactSensor).
    Apply before the simulation starts."""
    from pxr import PhysxSchema

    api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
    api.CreateThresholdAttr().Set(0.0)


def add_simple_room(stage: Usd.Stage, root: str = "/World/Room", offset=(0.0, 0.0), openings=()) -> None:
    """A walled room with a couple of obstacles, enough to test teleop and SLAM
    (scenes.ROOM_*), moved by `offset` (x, y), with doorways cut in its walls
    at `openings` (scenes.room_walls, before the offset)."""
    height, (dx, dy) = ROOM_HEIGHT, offset
    UsdGeom.Xform.Define(stage, root)
    for name, (cx, cy), (sx, sy) in room_walls(openings):
        add_box(stage, f"{root}/{name}", (cx + dx, cy + dy, height / 2), (sx, sy, height))
    for name, (cx, cy, cz), size, color in ROOM_BOXES:
        add_box(stage, f"{root}/{name}", (cx + dx, cy + dy, cz), size, color)


def add_floor_marking(stage: Usd.Stage, path: str, rect, color, z: float = 0.0) -> None:
    """A flat coloured patch (x0, y0, x1, y1) lying on the floor, visual
    only: no collider, so the LiDAR and the step sensors (PhysX raycasts) do
    not see it. A marking `z` higher draws over one below."""
    x0, y0, x1, y1 = rect
    t = 0.002
    add_box(stage, path, ((x0 + x1) / 2, (y0 + y1) / 2, 0.001 + z + t / 2), (x1 - x0, y1 - y0, t), color,
            collider=False)


def add_wall(stage: Usd.Stage, path: str, start, end, thickness: float = ROOM_WALL,
             height: float = ROOM_HEIGHT) -> None:
    """A straight wall from `start` to `end` (x, y), centred on that line."""
    (x0, y0), (x1, y1) = start, end
    length = math.hypot(x1 - x0, y1 - y0)
    add_box(stage, path, ((x0 + x1) / 2, (y0 + y1) / 2, height / 2), (length, thickness, height))
    mesh = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    mesh.AddRotateZOp().Set(math.degrees(math.atan2(y1 - y0, x1 - x0)))


def add_door(stage: Usd.Stage, spec, cfg: SimpleNamespace, wall: float = ROOM_WALL) -> str:
    """A door panel (doors.DoorSpec) as a kinematic body, placed for its
    start state; doors.Doors moves it. Returns the body path."""
    from .doors import panel_pose, panel_size
    from .drive import quat_from_yaw

    length, t = panel_size(spec, float(cfg.thickness))
    h = float(cfg.height)
    p = panel_pose(spec, 1.0 if spec.open else 0.0, t, wall)
    body = UsdGeom.Xform.Define(stage, spec.path)
    xf = UsdGeom.Xformable(body.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(p.x, p.y, h / 2))
    xf.AddOrientOp().Set(Gf.Quatf(*map(float, quat_from_yaw(p.yaw))))
    rb = UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    rb.CreateKinematicEnabledAttr(True)
    add_box(stage, f"{spec.path}/panel", (0.0, 0.0, 0.0), (length, t, h - 0.01), (0.62, 0.45, 0.28))
    # A handle on both faces, to tell the panel from a wall.
    for side in (-1, 1):
        add_box(stage, f"{spec.path}/handle_{'a' if side > 0 else 'b'}", (length / 2 - 0.08, side * (t / 2 + 0.015), 0.0),
                (0.03, 0.03, 0.15), (0.8, 0.8, 0.82), collider=False)
    return spec.path


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


def add_person(stage: Usd.Stage, path: str, pose, height: float = 1.7, character: str | None = None,
               kinematic: bool = False) -> None:
    """A standing person, for object detection and for the LiDAR to see.
    `pose` (x, y, yaw): feet centre and facing direction. `character`: an
    Isaac Sim people USD (URL), standing with the arms down; without one, a
    mannequin of capsules. Colliders (legs.py): a capsule for each thigh and
    shin, which the LiDAR sees as two legs, and an invisible box over the
    upper body. `kinematic`: a kinematic body that movers.Movers moves, legs
    and all."""
    x, y, yaw = pose
    root = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(root.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(yaw))
    if kinematic:
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateKinematicEnabledAttr(True)
    if character:
        add_actor(stage, f"{path}/actor", character)
    else:
        add_mannequin(stage, path, height)
    add_legs(stage, path, height, visible=not character)
    k = height / 1.7
    z0 = TORSO_Z * k
    add_box(stage, f"{path}/collider", (0.0, 0.0, (z0 + height) / 2),
            (PERSON_COLLIDER[0] * k, PERSON_COLLIDER[1] * k, height - z0))
    UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/collider")).MakeInvisible()


def add_legs(stage: Usd.Stage, path: str, height: float = 1.7, visible: bool = False) -> None:
    """Thigh and shin capsule colliders under `path`, standing (legs.py).
    Movers poses them through their translate and rotateY ops (set_legs).
    `visible`: the mannequin's legs; else hidden under a character."""
    for name, center, pitch in leg_capsules({}, height):
        c = UsdGeom.Capsule.Define(stage, f"{path}/{name}")
        c.CreateRadiusAttr(LEG_RADIUS * height / 1.7)
        c.CreateHeightAttr(capsule_length(name, height))
        c.CreateAxisAttr("Z")
        c.CreateDisplayColorAttr([Gf.Vec3f(*MANNEQUIN_PANTS)])
        xf = UsdGeom.Xformable(c.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddRotateYOp().Set(math.degrees(pitch))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if not visible:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()


class Legs:
    """The leg capsules of a person made by add_person, to pose them."""

    def __init__(self, stage: Usd.Stage, path: str, height: float):
        self.height = height
        self.ops = {}
        for name, _, _ in leg_capsules({}, height):
            ops = {op.GetOpName(): op for op in UsdGeom.Xformable(stage.GetPrimAtPath(f"{path}/{name}"))
                   .GetOrderedXformOps()}
            self.ops[name] = (ops["xformOp:translate"], ops["xformOp:rotateY"])

    @classmethod
    def of(cls, stage: Usd.Stage, path: str) -> Legs | None:
        """The legs of the person at `path`, or None (not a person). The
        height comes from the capsules' radius."""
        prim = stage.GetPrimAtPath(f"{path}/thigh_l")
        if not prim or not prim.IsA(UsdGeom.Capsule):
            return None
        return cls(stage, path, UsdGeom.Capsule(prim).GetRadiusAttr().Get() / LEG_RADIUS * 1.7)

    def set(self, angles: dict[str, float]) -> None:
        """`angles`: joint name -> rad forward, as Gait.set."""
        for name, center, pitch in leg_capsules(angles, self.height):
            translate, rotate = self.ops[name]
            translate.Set(Gf.Vec3d(*center))
            rotate.Set(math.degrees(pitch))


# Upper arm joints of the Isaac Sim people rig (Reallusion) and where the
# arms point when standing, in skeleton space: down, a little out to the side.
ARM_JOINTS = {"L_Upperarm": (0.15, 0.0, -1.0), "R_Upperarm": (-0.15, 0.0, -1.0)}


def add_actor(stage: Usd.Stage, path: str, character: str) -> None:
    """Reference an Isaac Sim people character at `path`, standing with the
    arms down. The characters' rest pose has the arms out (T-pose), and the
    bundled animations target another rig (they need the animation graph's
    retargeting), so a one-frame SkelAnimation lowers the upper arms."""
    UsdGeom.Xformable(UsdGeom.Xform.Define(stage, path).GetPrim()).AddRotateZOp().Set(90.0)  # they face -y
    prim = stage.DefinePrim(f"{path}/character", "Xform")
    prim.GetReferences().AddReference(character)
    skel_prim = next((p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton)), None)
    if skel_prim is None:
        return
    skel = UsdSkel.Skeleton(skel_prim)
    joints = list(skel.GetJointsAttr().Get())
    local = [Gf.Matrix4d(m) for m in skel.GetRestTransformsAttr().Get()]
    index = {j: i for i, j in enumerate(joints)}
    parent = [index.get(j.rpartition("/")[0], -1) for j in joints]

    def skel_space(i):  # row vectors: local, then the parents'
        m = Gf.Matrix4d(local[i])
        while parent[i] >= 0:
            i = parent[i]
            m = m * local[i]
        return m

    for i, j in enumerate(joints):
        name = j.rpartition("/")[2]
        if name not in ARM_JOINTS:
            continue
        m = skel_space(i)
        bone = m.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))  # the rig's bones run along +y
        turn = Gf.Matrix4d().SetRotate(Gf.Rotation(bone, Gf.Vec3d(*ARM_JOINTS[name])))
        rot = m.ExtractRotationMatrix() * turn.ExtractRotationMatrix()
        if parent[i] >= 0:
            rot = rot * skel_space(parent[i]).ExtractRotationMatrix().GetInverse()
        t = local[i].ExtractTranslation()
        local[i] = Gf.Matrix4d().SetRotate(rot).SetTranslateOnly(t)
    anim = UsdSkel.Animation.Define(stage, f"{path}/pose")
    anim.CreateJointsAttr(joints)
    anim.SetTransforms(Vt.Matrix4dArray(local))
    UsdSkel.BindingAPI.Apply(skel_prim).CreateAnimationSourceRel().SetTargets([anim.GetPath()])


# The characters' skeleton space is z up with the character facing -y, so
# turning a leg or an arm forward is turning it about -x.
FORWARD_AXIS = (-1.0, 0.0, 0.0)


class Gait:
    """Poses a character made by add_actor: sets joint angles about the
    lateral axis on top of its standing pose (its SkelAnimation)."""

    def __init__(self, anim: UsdSkel.Animation):
        self.anim = anim
        joints = list(anim.GetJointsAttr().Get())
        self.base = [Gf.Matrix4d(m) for m in anim.GetTransforms(Usd.TimeCode.Default())]
        index = {j: i for i, j in enumerate(joints)}
        parent = [index.get(j.rpartition("/")[0], -1) for j in joints]
        self.axes = {}  # joint name -> (index, the lateral axis in the joint's own frame)
        for i, j in enumerate(joints):
            rot, k = self.base[i].ExtractRotationMatrix(), parent[i]
            while k >= 0:
                rot, k = rot * self.base[k].ExtractRotationMatrix(), parent[k]
            self.axes[j.rpartition("/")[2]] = (i, Gf.Vec3d(*FORWARD_AXIS) * rot.GetInverse())

    @classmethod
    def of(cls, stage: Usd.Stage, path: str) -> Gait | None:
        prim = stage.GetPrimAtPath(path)
        return cls(UsdSkel.Animation(prim)) if prim and prim.IsA(UsdSkel.Animation) else None

    def set(self, angles: dict[str, float]) -> None:
        """`angles`: joint name -> rad forward. The others stand."""
        local = list(self.base)
        for name, a in angles.items():
            if name in self.axes and a != 0.0:
                i, axis = self.axes[name]
                local[i] = Gf.Matrix4d().SetRotate(Gf.Rotation(axis, math.degrees(a))) * self.base[i]
        self.anim.SetTransforms(Vt.Matrix4dArray(local))


def add_mannequin(stage: Usd.Stage, path: str, height: float = 1.7) -> None:
    """A mannequin of capsules under `path`, without the legs: add_legs
    makes them, visible, and they walk."""
    k = height / 1.7
    skin, shirt = (0.8, 0.6, 0.48), (0.2, 0.42, 0.65)

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
        capsule(f"arm_{side}", (0.0, sy * 2.3, 1.12), 0.045, 0.5, shirt)
        capsule(f"hand_{side}", (0.0, sy * 2.3, 0.8), 0.04, 0.04, skin)
    capsule("torso", (0.0, 0.0, 1.17), 0.16, 0.38, shirt, scale=(0.7, 1.0, 1.0))
    capsule("neck", (0.0, 0.0, 1.45), 0.05, 0.05, skin)
    head = UsdGeom.Sphere.Define(stage, f"{path}/head")
    head.CreateRadiusAttr(0.11 * k)
    head.CreateDisplayColorAttr([Gf.Vec3f(*skin)])
    UsdGeom.Xformable(head.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.58 * k))


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

