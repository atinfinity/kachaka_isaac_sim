"""Run the Kachaka simulator. Start it through scripts/run_sim.sh, which sets
up the ROS 2 environment that Isaac Sim needs before the process starts.

    scripts/run_sim.sh                 # GUI, simple walled room
    scripts/run_sim.sh --headless      # no window
    scripts/run_sim.sh --world empty   # floor only
    scripts/run_sim.sh --world office  # Isaac Sim indoor scene (needs internet)
    scripts/run_sim.sh --world /path/to/scene.usd
    scripts/run_sim.sh --layout layouts/room.yaml   # charger, shelves, ... from a file
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kachaka_isaac_sim.config import DEFAULT_CONFIG, load_config
from kachaka_isaac_sim.hazards import hole_rect

ROBOT_USD = ROOT / "assets/generated/kachaka/kachaka.usda"
ROBOT_URDF = ROOT / "build/kachaka.urdf"
CEILING_LIGHT = 2.0  # stacked floors: ceiling light intensity per unit of dome light (by eye)
SHELF_STL = ROOT / "third_party/kachaka_shelf_description/meshes/kachaka_shelf_3layer.stl"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot parameters (YAML)")
    p.add_argument("--headless", action="store_true", help="run without a window")
    p.add_argument("--world",
                   help="'room', 'empty', an Isaac scene (office, hospital, warehouse, simple_room) or a USD path/URL "
                        "(default: the layout's world, else 'room')")
    p.add_argument("--layout", metavar="FILE",
                   help="world layout (YAML): spawn pose, charger, shelves, locations, people, boxes, walls, doors "
                        "and drops; see kachaka_isaac_sim/layout.py and layouts/. The options below override it "
                        "(--wall, --door and --drop add to it)")
    p.add_argument("--layout-seed", type=int, metavar="N",
                   help="seed for the layout's ranges (default: the layout's seed, else a random one)")
    p.add_argument("--x", type=float, help="spawn x [m] (default: the world's spawn pose, else 0)")
    p.add_argument("--y", type=float, help="spawn y [m]")
    p.add_argument("--yaw", type=float, help="spawn yaw [rad]")
    p.add_argument("--charger", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                   help="place the charging dock where a docked robot is at X, Y, YAW (world frame). "
                        "Default: on the west wall in 'room', none elsewhere")
    p.add_argument("--no-charger", action="store_true", help="no charging dock")
    p.add_argument("--shelf", type=float, nargs=3, action="append", metavar=("X", "Y", "YAW"),
                   help="place a Kachaka Shelf where a robot docked under it is at X, Y, YAW (world frame; "
                        "repeatable). Default: the shelves in config 'shelf.room' in 'room', none elsewhere")
    p.add_argument("--no-shelves", action="store_true", help="no Kachaka Shelves")
    p.add_argument("--person", type=float, nargs=3, action="append", metavar=("X", "Y", "YAW"),
                   help="place a standing person (static) at X, Y facing YAW (world frame; repeatable). "
                        "Default: the people in config 'people.room' in 'room', none elsewhere")
    p.add_argument("--no-people", action="store_true", help="no people")
    p.add_argument("--wall", type=float, nargs=4, action="append", metavar=("X0", "Y0", "X1", "Y1"),
                   help="a straight wall from X0, Y0 to X1, Y1 (world frame; repeatable)")
    p.add_argument("--door", type=float, nargs=3, action="append", metavar=("X", "Y", "YAW"),
                   help="a door whose doorway is centred at X, Y, with the wall along YAW (world frame; "
                        "repeatable; ids D01, D02, ...). Leave a gap for it between --wall segments. "
                        "Open and close it with the sim/doors/<id>/set_open service")
    p.add_argument("--door-kind", choices=("sliding", "hinged"), help="door type (default: door.kind in the config)")
    p.add_argument("--door-width", type=float, help="doorway width [m] (default: door.width in the config)")
    p.add_argument("--door-closed", action="store_true", help="doors start closed")
    p.add_argument("--door-auto", action="store_true",
                   help="--door doors are automatic sliding doors: they open when the robot or a mover comes near "
                        "(door.auto in the config)")
    p.add_argument("--door-cycle", type=float, nargs=2, metavar=("OPEN_S", "CLOSED_S"),
                   help="open and close the doors on their own, OPEN_S open then CLOSED_S closed, "
                        "until a sim/doors command")
    p.add_argument("--drop", type=float, nargs=4, action="append", metavar=("X", "Y", "SX", "SY"),
                   help="a drop (hole in the floor) centred at X, Y, SX by SY metres; repeatable "
                        "(depth: drop.depth in the config)")
    p.add_argument("--physics-hz", type=int, default=120)
    p.add_argument("--render-hz", type=float, default=10.0,
                   help="render and main-loop rate; keep it a multiple of the camera rate")
    p.add_argument("--realtime", type=float, default=1.0,
                   help="target real-time factor; 0 runs as fast as possible")
    p.add_argument("--max-sim-time", type=float, default=0.0, help="exit after this many sim seconds (0: never)")
    p.add_argument("--seed", type=int, default=None, help="seed for sensor and odometry noise")
    p.add_argument("--no-cameras", action="store_true", help="skip RGB and ToF cameras (faster)")
    p.add_argument("--dome-light", type=float, default=800.0,
                   help="dome light intensity; low values make a dark scene, e.g. to see the torches")
    p.add_argument("--api-port", type=int, default=26400,
                   help="port of the kachaka-api gRPC server (0: no server)")
    p.add_argument("--record", metavar="DIR", help="save bird's-eye PNG frames to DIR (for demo videos)")
    p.add_argument("--record-follow", action="store_true",
                   help="move the --record camera with the robot (--record-view relative to it)")
    p.add_argument("--record-near", type=float, default=0.05,
                   help="--record camera near clip [m]; larger cuts away ceilings above the robot")
    p.add_argument("--record-hz", type=float, default=10.0, help="frame rate for --record")
    p.add_argument("--record-view", type=float, nargs=6, default=[0.0, -2.4, 7.5, 0.0, -0.1, 0.0],
                   metavar=("EX", "EY", "EZ", "TX", "TY", "TZ"), help="--record camera position and target [m]")
    return p.parse_args()


def resolve_layout(args, cfg):
    """Read --layout (if any), settle the world and the spawn pose."""
    from kachaka_isaac_sim.layout import parse_layout, read_layout
    from kachaka_isaac_sim.scenes import SPAWN

    layout = None
    data = read_layout(args.layout) if args.layout else {}
    args.world = args.world or data.get("world") or "room"
    if args.layout:
        room = args.world == "room"
        defaults = {"spawn": [SPAWN.get(args.world, (0.0, 0.0, 0.0))],
                    "charger": [cfg.charger.room_pose] if room else [],
                    "shelves": [e.pose for e in cfg.shelf.room] if room else [],
                    "locations": [e.pose for e in cfg.shelf.room_locations] if room else [],
                    "people": list(cfg.people.room) if room else []}
        layout = parse_layout(data, args.layout_seed, args.world, args.layout, defaults)
        seed = f" (seed {layout.seed})" if layout.randomised else ""
        print(f"[kachaka] layout {args.layout}{seed}", flush=True)
    spawn = layout.spawn if layout and layout.spawn else None
    default = (spawn.x, spawn.y, spawn.yaw) if spawn else SPAWN.get(args.world, (0.0, 0.0, 0.0))
    for i, k in enumerate(("x", "y", "yaw")):
        if getattr(args, k) is None:
            setattr(args, k, default[i])
    return layout


def publish_cameras(node, cameras, t: float, tof_off: bool = False, detector=None) -> None:
    for cam in cameras:
        if not cam.due(t) or (tof_off and cam.kind == "depth"):
            continue
        data = cam.read()
        if data is None:
            continue
        if cam.kind == "rgb":
            node.publish_image(t, cam.name, cam.cfg.frame_id, data, "rgb8", cam.fx)
            if detector is not None and detector.camera is cam:
                objects = detector.detect()
                if objects is not None:
                    node.publish_object_detection(t, cam.cfg.frame_id, [o.to_dict() for o in objects])
        elif cam.cfg.encoding == "16UC1":
            mm = (data * 1000.0).round().clip(0, 65535).astype("uint16")
            node.publish_image(t, cam.name, cam.cfg.frame_id, mm, "16UC1", cam.fx)
        else:
            node.publish_image(t, cam.name, cam.cfg.frame_id, data, "32FC1", cam.fx)


def place_people(args, cfg, layout) -> list[tuple[float, float, float]]:
    if args.no_people:
        return []
    if args.person:
        return [tuple(p) for p in args.person]
    if layout and layout.people is not None:
        return [(p.x, p.y, p.yaw) for p in layout.people]
    if args.world == "room":
        return [tuple(p) for p in cfg.people.room]
    return []


def people_motions(args, layout, people) -> list:
    """layout.Motion (or None: standing) for each person in `people`."""
    if layout and layout.people is not None and not args.person and not args.no_people:
        return list(layout.people_motion)
    return [None] * len(people)


def mover_specs(cfg, layout, people, motions, boxes_root) -> list:
    """movers.MoverSpec for the people and boxes that move."""
    from kachaka_isaac_sim.drive import Pose2D
    from kachaka_isaac_sim.layout import BOX_RADIUS_MIN, PERSON_RADIUS
    from kachaka_isaac_sim.movers import MoverSpec
    from kachaka_isaac_sim.scenes import PERSON_COLLIDER

    seed = layout.seed if layout and layout.seed is not None else 0
    person = (*PERSON_COLLIDER, float(cfg.people.height))
    items = [(f"/World/People/P{i + 1:02d}", Pose2D(*p), PERSON_RADIUS, float(cfg.movers.person_speed), m,
              "person", person) for i, (p, m) in enumerate(zip(people, motions))]
    for b in layout.boxes if layout else []:
        radius = max(BOX_RADIUS_MIN, math.hypot(*b.size[:2]) / 2)
        items.append((f"{boxes_root}/{b.name}", b.pose, radius, float(cfg.movers.box_speed), b.motion, "box",
                      tuple(b.size)))
    return [MoverSpec(path, pose, radius, m.speed or speed, route=m.route, wander=m.wander, pause=m.pause,
                      appear=m.appear, seed=seed * 1000 + k, kind=kind, size=size)
            for k, (path, pose, radius, speed, m, kind, size) in enumerate(items) if m is not None]


def people_assets(cfg) -> list[str]:
    """Character URLs for config 'people.characters', or [] for mannequins
    (none configured or no asset server)."""
    names = list(getattr(cfg.people, "characters", None) or [])
    if not names:
        return []
    from isaacsim.storage.native import get_assets_root_path

    root = get_assets_root_path()
    if root is None:
        print("[kachaka] Isaac Sim asset server not reachable: people are mannequins", flush=True)
        return []
    return [f"{root}/Isaac/People/Characters/{n}/{n}.usd" for n in names]


def detection_targets(people, shelf_specs, charger_pose):
    from kachaka_isaac_sim.detection import LABEL_CHARGER, LABEL_PERSON, LABEL_SHELF, Target

    targets = [Target(f"/World/People/P{i + 1:02d}", LABEL_PERSON) for i in range(len(people))]
    targets += [Target(s.path, LABEL_SHELF) for s in shelf_specs]
    if charger_pose:
        targets.append(Target("/World/Charger", LABEL_CHARGER))
    return targets


def place_shelves(args, cfg, layout):
    """Shelf specs and extra locations for this world."""
    from kachaka_isaac_sim.drive import Pose2D
    from kachaka_isaac_sim.shelves import ShelfSpec

    room = args.world == "room"
    if args.no_shelves or not SHELF_STL.exists():
        entries = []
    elif args.shelf:
        entries = [(f"S{i + 1:02d}", f"Shelf {i + 1}", Pose2D(*pose)) for i, pose in enumerate(args.shelf)]
    elif layout and layout.shelves is not None:
        entries = layout.shelves
    elif room:
        entries = [(e.id, e.name, Pose2D(*e.pose)) for e in cfg.shelf.room]
    else:
        entries = []
    if layout and layout.locations is not None:
        locations = layout.locations
    elif room and not args.shelf and not args.no_shelves:
        locations = [(e.id, e.name, Pose2D(*e.pose)) for e in cfg.shelf.room_locations]
    else:
        locations = []
    specs = [ShelfSpec(i, name, f"/World/Shelves/{i}", pose) for i, name, pose in entries]
    return specs, [{"id": i, "name": name, "pose": pose} for i, name, pose in locations]


def place_charger(args, cfg, layout):
    if args.no_charger:
        return None
    if args.charger:
        return tuple(args.charger)
    if layout and (layout.charger or layout.no_charger):
        return (layout.charger.x, layout.charger.y, layout.charger.yaw) if layout.charger else None
    if args.world == "room":
        return tuple(cfg.charger.room_pose)
    return None


def place_doors(args, cfg, layout):
    """The layout's doors, then --door ones. --door-kind and --door-width are
    the defaults for the layout's doors too."""
    from kachaka_isaac_sim.doors import AutoSpec, DoorSpec
    from kachaka_isaac_sim.drive import Pose2D

    kind = args.door_kind or cfg.door.kind
    width = args.door_width or float(cfg.door.width)
    cycle = tuple(args.door_cycle) if args.door_cycle else None

    def auto(overrides):  # an automatic door is sliding, whatever the default kind
        if overrides is None:
            return None
        a = cfg.door.auto
        return AutoSpec(**{"range": float(a.range), "margin": float(a.margin), "hold": float(a.hold),
                           "movers": bool(a.movers), **overrides})

    doors = [(d.pose, d.kind or ("sliding" if d.auto is not None else kind), d.width or width, d.open, d.cycle,
              auto(d.auto)) for d in (layout.doors if layout else [])]
    doors += [(Pose2D(*pose), "sliding" if args.door_auto else kind, width, not args.door_closed,
               None if args.door_auto else cycle, auto({} if args.door_auto else None)) for pose in args.door or []]
    return [DoorSpec(f"D{i + 1:02d}", f"/World/Doors/D{i + 1:02d}", pose, w, k, open=o, cycle=c, auto=a)
            for i, (pose, k, w, o, c, a) in enumerate(doors)]


def layout_objects(layout, charger_pose, shelf_specs, people, boxes_root):
    """(name, prim path or None, pose, half size x, y, height) of what the
    layout placed, for the overlap check."""
    from kachaka_isaac_sim.drive import Pose2D
    from kachaka_isaac_sim.scenes import PERSON_COLLIDER, SHELF_SIZE

    objs = []
    if charger_pose:
        objs.append(("charger", "/World/Charger", Pose2D(*charger_pose), 0.12, 0.13, 0.09))
    for s in shelf_specs:
        objs.append((s.id, s.path, s.home, SHELF_SIZE[0] / 2, SHELF_SIZE[1] / 2, 0.66))
    for i, p in enumerate(people):
        objs.append((f"P{i + 1:02d}", f"/World/People/P{i + 1:02d}", Pose2D(*p), PERSON_COLLIDER[0] / 2,
                     PERSON_COLLIDER[1] / 2, 1.7))
    for b in layout.boxes:
        objs.append((b.name, f"{boxes_root}/{b.name}", b.pose, b.size[0] / 2, b.size[1] / 2, b.size[2]))
    return objs


def stack_floors(stage, spec, boxes_root) -> None:
    """Stack the floors (elevator.stage_offset): lift each upper floor's room,
    look and car, and the walls, boxes and people on it (by where they are),
    and add the car's platform at the start floor."""
    from kachaka_isaac_sim import world
    from kachaka_isaac_sim.elevator import (
        PLATFORM_OVERLAP,
        PLATFORM_PATH,
        PLATFORM_THICKNESS,
        floor_of,
        platform_z,
        stage_offset,
    )

    n = len(spec.floors)
    for k in range(1, n):
        for group in (f"/World/Floors/F{k + 1}", f"/World/Floors/Look{k + 1}", f"/World/Elevator/F{k + 1}"):
            world.lift(stage, group, stage_offset(k, spec.stack))
    for root in ("/World/Walls", boxes_root, "/World/People"):
        world.lift_children(stage, root, lambda x, y: stage_offset(min(floor_of(x), n - 1), spec.stack))
    x0, y0, x1, y1 = spec.inside_rect(0, world.ROOM_WALL)
    d = PLATFORM_OVERLAP
    world.add_platform(stage, PLATFORM_PATH, (x0 - d, y0 - d, x1 + d, y1 + d), PLATFORM_THICKNESS,
                       platform_z(spec.start))


def check_overlaps(objs, spawn) -> list[str]:
    """Warnings for objects whose outline overlaps another collider (PhysX
    overlap query, from 3 cm above the floor). Needs the simulation running."""
    import carb
    from omni.physx import get_physx_scene_query_interface

    from kachaka_isaac_sim.drive import Pose2D, quat_from_yaw
    from kachaka_isaac_sim.world import ROBOT_PATH

    query = get_physx_scene_query_interface()
    warnings = []
    # The robot may start docked on the charger or under a shelf.
    spawn_obj = ("spawn", (ROBOT_PATH, "/World/Charger", "/World/Shelves"), Pose2D(*spawn), 0.2, 0.13, 0.2)
    for name, own, pose, hx, hy, h in objs + [spawn_obj]:
        hits = set()
        own = (own,) if isinstance(own, str) else own

        def report(hit, own=own, hits=hits):
            col = str(hit.collision)
            if not col.startswith(own + (ROBOT_PATH,)):
                hits.add(col)
            return True

        z0 = 0.03
        w, x, y, z = (float(v) for v in quat_from_yaw(pose.yaw))
        query.overlap_box(carb.Float3(hx, hy, (h - z0) / 2), carb.Float3(pose.x, pose.y, z0 + (h - z0) / 2),
                          carb.Float4(x, y, z, w), report, False)
        if hits:
            more = f" and {len(hits) - 3} more" if len(hits) > 3 else ""
            warnings.append(f"{name} at ({pose.x:.2f}, {pose.y:.2f}) overlaps {', '.join(sorted(hits)[:3])}{more}")
    return warnings


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
    from kachaka_isaac_sim.layout import LayoutError

    try:
        layout = resolve_layout(args, cfg)
    except (LayoutError, OSError) as e:
        print(f"layout: {e}", file=sys.stderr)
        return 1
    if not ROBOT_USD.exists() or not ROBOT_URDF.exists():
        print("Robot assets missing: run scripts/fetch_assets.py and scripts/build_usd.py first", file=sys.stderr)
        return 1
    if os.environ.get("ROS_DISTRO") != "jazzy":
        print("ROS_DISTRO is not 'jazzy'; start the simulator with scripts/run_sim.sh", file=sys.stderr)
        return 1

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.simulation_manager import SimulationManager

    app_utils.enable_extension("isaacsim.ros2.bridge")  # also makes the bundled rclpy importable
    if not args.no_cameras or args.record:
        app_utils.enable_extension("omni.replicator.core")
    app.update()
    SimulationManager.switch_physics_engine("physx")

    from pxr import UsdGeom

    from kachaka_isaac_sim import world
    from kachaka_isaac_sim.ros_node import KachakaSimNode

    stage = stage_utils.create_new_stage()
    world.setup_stage(stage, args.physics_hz, args.dome_light)
    builtin = args.world in ("room", "empty")
    holes = [hole_rect(*d) for d in (layout.drops if layout else []) + [tuple(d) for d in args.drop or []]]
    world.add_ground(stage, visible=builtin, z=-float(cfg.drop.depth) if holes else 0.0)
    if holes:
        world.add_floor_with_drops(stage, holes)
    elevator_spec = None
    if layout and layout.elevator:
        from kachaka_isaac_sim.elevator import (
            FLOOR_HEIGHT,
            car_walls,
            elevator_spec,
            floor_look,
            floor_offset,
            label_segments,
            slab_tiles,
        )
        from kachaka_isaac_sim.scenes import ROOM_SIZE

        elevator_spec = elevator_spec(layout.elevator, layout.floors, cfg.elevator)
        e = elevator_spec
        stacked = e.stack == "vertical"
        opening = [(e.pose.x, e.pose.y, e.pose.yaw, e.door_width)]
        world.add_simple_room(stage, openings=opening)
        for k in range(len(e.floors)):
            for group in (f"/World/Floors/F{k + 1}", f"/World/Floors/Look{k + 1}", f"/World/Elevator/F{k + 1}"):
                UsdGeom.Xform.Define(stage, group)  # lifted as a whole when stacked
            if k:
                world.add_simple_room(stage, f"/World/Floors/F{k + 1}", floor_offset(k), opening)
            if k and stacked:
                # The floor under floor k, with the shaft left open.
                (ox, oy), (sx, sy), w = floor_offset(k), ROOM_SIZE, world.ROOM_WALL
                room = (ox - sx / 2 - w, oy - sy / 2 - w, ox + sx / 2 + w, oy + sy / 2 + w)
                world.add_slab(stage, f"/World/Floors/F{k + 1}/slab",
                               [room, *slab_tiles(e.outer_rect(k, w), e.inside_rect(k, w))])
            if k < len(e.floors) - 1 and stacked:
                # The next floor's slab keeps the dome light out: light the room from its ceiling.
                (ox, oy), (sx, sy) = floor_offset(k), ROOM_SIZE
                world.add_ceiling_light(stage, f"/World/Floors/F{k + 1}/ceiling_light",
                                        (ox - sx / 2, oy - sy / 2, ox + sx / 2, oy + sy / 2),
                                        FLOOR_HEIGHT - 0.12, CEILING_LIGHT * args.dome_light)
            for i, (a, b) in enumerate(car_walls(e, k, world.ROOM_WALL)):
                # Stacked, the car walls are the shaft, floor to floor.
                world.add_wall(stage, f"/World/Elevator/F{k + 1}/wall_{i}", a, b,
                               height=FLOOR_HEIGHT if stacked else world.ROOM_HEIGHT)
            world.add_door(stage, e.door(k), cfg.door)
            # Each floor's tint and name on the floor, so a ride shows (visual only).
            (ox, oy), (sx, sy) = floor_offset(k), ROOM_SIZE
            tint, ink = floor_look(k)
            world.add_floor_marking(stage, f"/World/Floors/Look{k + 1}/tint",
                                    (ox - sx / 2, oy - sy / 2, ox + sx / 2, oy + sy / 2), tint)
            for i, r in enumerate(label_segments(e.floors[k], (ox + 0.0, oy + 0.45))):
                world.add_floor_marking(stage, f"/World/Floors/Look{k + 1}/label_{i:02d}",
                                        (r[0], r[1], r[2], r[3]), ink, z=0.002)
        print(f"[kachaka] floors {', '.join(e.floors)}, elevator at {e.floors[e.start]}", flush=True)
    elif args.world == "room":
        world.add_simple_room(stage)
    elif not builtin:
        world.load_world_usd(stage, world.resolve_world(args.world))
    walls = (layout.walls if layout else []) + [tuple(w) for w in args.wall or []]
    for i, (x0, y0, x1, y1) in enumerate(walls):
        world.add_wall(stage, f"/World/Walls/W{i + 1:02d}", (x0, y0), (x1, y1))
    door_specs = place_doors(args, cfg, layout)
    for spec in door_specs:
        world.add_door(stage, spec, cfg.door)
    boxes_root = "/World/Boxes"
    for b in layout.boxes if layout else []:
        if b.motion:
            world.add_moving_box(stage, f"{boxes_root}/{b.name}", (b.pose.x, b.pose.y, b.pose.yaw), b.size, b.color)
        else:
            world.add_box(stage, f"{boxes_root}/{b.name}", (b.pose.x, b.pose.y, b.size[2] / 2), b.size, b.color,
                          yaw=b.pose.yaw)
    charger_pose = place_charger(args, cfg, layout)
    if charger_pose:
        world.add_charger(stage, "/World/Charger", charger_pose, cfg.charger)
    shelf_specs, locations = place_shelves(args, cfg, layout)
    for spec in shelf_specs:
        world.add_shelf(stage, spec.path, SHELF_STL, (spec.home.x, spec.home.y, spec.home.yaw), cfg.shelf)
    people = place_people(args, cfg, layout)
    motions = people_motions(args, layout, people)
    if layout and layout.floor_people and not args.no_people:
        people += [(p.x, p.y, p.yaw) for p in layout.floor_people]
        motions += list(layout.floor_people_motion)
    actors = people_assets(cfg) if people else []
    for i, (pose, motion) in enumerate(zip(people, motions)):
        character = actors[i % len(actors)] if actors else None
        world.add_person(stage, f"/World/People/P{i + 1:02d}", pose, float(cfg.people.height), character,
                         kinematic=motion is not None)
    moving = mover_specs(cfg, layout, people, motions, boxes_root)
    if elevator_spec and elevator_spec.stack == "vertical":
        stack_floors(stage, elevator_spec, boxes_root)
    world.add_robot(stage, ROBOT_USD, args.x, args.y, args.yaw)
    world.apply_contact_materials(stage, cfg.drive)
    world.disable_pin_collision(stage)
    world.enable_contact_report(stage)
    from kachaka_isaac_sim.torches import Torches
    from kachaka_isaac_sim.urdf_tools import read_joints

    torches = Torches(cfg.torch)
    torches.create(stage, world.BASE_LINK, read_joints(ROBOT_URDF))
    cameras = []
    if not args.no_cameras:
        from kachaka_isaac_sim.cameras import create_cameras

        cameras = create_cameras(stage, cfg, world.BASE_LINK, read_joints(ROBOT_URDF))
    detector = None
    det_cfg = getattr(cfg, "object_detection", None)
    front = next((c for c in cameras if c.name == "front_camera"), None)
    if front and det_cfg and det_cfg.enabled:
        from kachaka_isaac_sim.cameras import FrontDetector
        from kachaka_isaac_sim.detection import DetectionParams

        detector = FrontDetector(front, detection_targets(people, shelf_specs, charger_pose),
                                 DetectionParams.from_config(det_cfg), args.seed)
    recorder = None
    if args.record:
        from kachaka_isaac_sim.cameras import RecordCamera

        v = args.record_view
        recorder = RecordCamera(stage, args.record, v[:3], v[3:], args.record_hz, follow=args.record_follow,
                                near=args.record_near)
    app.update()

    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.core.simulation_manager import SimulationEvent

    from kachaka_isaac_sim.robot import SimRobot

    # Render (and run the main loop) slower than physics: rendering costs far
    # more than a physics step, and sensors need at most 30 Hz.
    render_dt = 1.0 / args.render_hz
    RenderingManager.set_dt(render_dt)
    # PhysX caps the substeps per update by a minimum frame rate; lower it so
    # every update advances the full render_dt at a fixed physics step.
    import carb

    carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", 1)
    app_utils.play()
    app.update()
    if layout:
        for w in check_overlaps(layout_objects(layout, charger_pose, shelf_specs, people, boxes_root),
                                (args.x, args.y, args.yaw)):
            print(f"[kachaka] layout warning: {w}", flush=True)

    node = KachakaSimNode(cfg, ROBOT_URDF)
    for cam in cameras:
        node.add_camera(cam.name)
    node.object_detection_enabled = detector is not None
    node.torches = torches
    node.publish_torch()
    api = None
    if args.api_port:
        from kachaka_isaac_sim.api_server import KachakaApiServer

        api = KachakaApiServer(node, args.api_port)
    from kachaka_isaac_sim.drive import Pose2D, relative_pose

    # SLAM starts its map at the odometry origin, i.e. the spawn pose, so
    # sim-only poses for the ROS side are given relative to it.
    spawn = Pose2D(args.x, args.y, args.yaw)
    charger = None
    if charger_pose:
        from kachaka_isaac_sim.power import Charger

        dock = Pose2D(*charger_pose)
        charger = Charger(dock, cfg.charger.tolerance_xy, cfg.charger.tolerance_yaw)
        node.publish_charger_pose(relative_pose(spawn, dock))
    shelves = None
    if shelf_specs:
        from kachaka_isaac_sim.shelves import Shelves

        shelves = Shelves(stage, shelf_specs, cfg.shelf)
        node.shelf_reset_handler = shelves.reset
    if shelf_specs or locations:
        from kachaka_isaac_sim.world import SHELF_SIZE

        def in_map(p):
            m = relative_pose(spawn, p)
            return [round(m.x, 4), round(m.y, 4), round(m.yaw, 4)]

        node.publish_layout(
            [{"id": s.id, "name": s.name, "home": in_map(s.home), "width": SHELF_SIZE[1], "depth": SHELF_SIZE[0]}
             for s in shelf_specs],
            [{"id": loc["id"], "name": loc["name"], "pose": in_map(loc["pose"])} for loc in locations])
    def to_map(p):
        """A world pose in its floor's map frame (the spawn pose moved to that
        floor; just the spawn pose without floors)."""
        if elevator_spec is None:
            return relative_pose(spawn, p)
        from kachaka_isaac_sim.elevator import floor_of, shifted

        return relative_pose(shifted(spawn, min(floor_of(p.x), len(elevator_spec.floors) - 1)), p)

    doors = None
    if door_specs:
        from kachaka_isaac_sim.doors import Doors

        # Automatic doors sense the robot and the movers, made further down.
        doors = Doors(stage, door_specs, cfg.door, world.ROOM_WALL, lambda: robot.gt,
                      lambda: movers.poses() if movers else [])
        node.add_doors(doors, to_map)
    sim_start = SimulationManager.get_simulation_time()
    from kachaka_isaac_sim.physics_sensors import ContactSensor

    contacts = ContactSensor(world.BASE_LINK, world.CASTER_SHAPES)
    robot = SimRobot(world.BASE_LINK, world.ROBOT_PATH, ROBOT_URDF, cfg, node, sim_start, args.seed, charger,
                     shelves, spawn, contacts)
    movers = None
    if moving:
        from kachaka_isaac_sim.movers import Movers

        movers = Movers(stage, moving, cfg.movers, lambda: robot.gt)
        node.add_movers()
        print(f"[kachaka] {len(moving)} moving: {', '.join(m.path.rpartition('/')[2] for m in moving)}", flush=True)

    def robot_height() -> float:
        """The base's height above where it started."""
        return robot.z - robot.start_z

    def robot_place():
        """The robot's logical pose (elevator.logical_pose): its floor moved
        out along x, also when the floors are stacked."""
        if elevator_spec is None:
            return robot.gt
        from kachaka_isaac_sim.elevator import logical_pose

        return logical_pose(robot.gt, robot_height(), elevator_spec.stack, len(elevator_spec.floors))

    elevator = None
    if elevator_spec:
        from kachaka_isaac_sim.elevator import Elevator

        elevator = Elevator(stage, elevator_spec, cfg.door, world.ROOM_WALL, robot_place,
                            float(cfg.elevator.door_sensor),
                            robot_height if elevator_spec.stack == "vertical" else None)
        node.add_elevator(elevator, robot_place)

    def ride(start: int, end: int) -> None:
        """Carry what is in the car from floor `start` to floor `end` (side by
        side; stacked, the platform has carried it)."""
        if elevator_spec.stack == "vertical":
            print(f"[kachaka] elevator: at {elevator_spec.floors[end]}", flush=True)
            return
        (x0, y0), (x1, y1) = floor_offset(start), floor_offset(end)
        dx, dy = x1 - x0, y1 - y0
        if elevator_spec.inside(robot.gt, start, world.ROOM_WALL):
            robot.shift(dx, dy)
            print(f"[kachaka] elevator: robot to {elevator_spec.floors[end]}", flush=True)
        if shelves:
            loose = [i for i, p in enumerate(shelves.update())
                     if i != shelves.docked and elevator_spec.inside(p, start, world.ROOM_WALL)]
            shelves.shift(loose, dx, dy)

    def reset(clear_odometry: bool = False) -> None:
        robot.reset(clear_odometry)
        if elevator:
            elevator.reset()
        if doors:
            doors.reset()
        if movers:
            movers.reset()

    node.reset_handler = reset
    callbacks = [
        SimulationManager.register_callback(lambda dt, ctx: robot.pre_step(dt), SimulationEvent.PHYSICS_PRE_STEP),
        SimulationManager.register_callback(lambda dt, ctx: robot.post_step(dt), SimulationEvent.PHYSICS_POST_STEP),
    ]
    if doors:
        callbacks.append(SimulationManager.register_callback(lambda dt, ctx: doors.step(dt),
                                                             SimulationEvent.PHYSICS_PRE_STEP))
    if movers:
        callbacks.append(SimulationManager.register_callback(lambda dt, ctx: movers.step(dt),
                                                             SimulationEvent.PHYSICS_PRE_STEP))
        # Ground truth for checks and RViz, at the ground truth's rate.
        movers_period = 1.0 / float(cfg.odometry.rate)
        movers_next = [0.0]

        def publish_movers(dt) -> None:
            if robot.t + 1e-9 >= movers_next[0]:
                movers_next[0] = robot.t + movers_period
                node.publish_movers(movers.report(robot.t, to_map))

        callbacks.append(SimulationManager.register_callback(lambda dt, ctx: publish_movers(dt),
                                                             SimulationEvent.PHYSICS_POST_STEP))
    if elevator:
        callbacks.append(SimulationManager.register_callback(lambda dt, ctx: elevator.step(dt),
                                                             SimulationEvent.PHYSICS_PRE_STEP))

    timing = [0.0, 0.0, 0]  # update seconds, camera seconds, loop count
    wall_start = time.monotonic()
    where = f", charger at {charger_pose}" if charger_pose else ""
    if api:
        where += f", kachaka-api on port {args.api_port}"
    print(f"[kachaka] running: ROS namespace '{node.ns}', world '{args.world}'{where}", flush=True)
    try:
        while app.is_running():
            t0 = time.perf_counter()
            node.spin_once()
            node.run_pending()
            app.update()
            if doors and doors.take_changed():
                node.publish_doors()
            if elevator:
                trip = elevator.take_ride()
                if trip:
                    ride(*trip)
                    elevator.changed = True
                if elevator.take_changed():
                    node.publish_elevator()
            t1 = time.perf_counter()
            publish_cameras(node, cameras, robot.t, tof_off=robot.sensors_paused, detector=detector)
            if recorder:
                recorder.capture(robot.t - sim_start, (robot.gt.x, robot.gt.y, robot_height()))
            timing[0] += t1 - t0
            timing[1] += time.perf_counter() - t1
            timing[2] += 1
            sim = robot.t - sim_start
            if args.max_sim_time and sim >= args.max_sim_time:
                break
            if args.realtime > 0:
                ahead = sim / args.realtime - (time.monotonic() - wall_start)
                if ahead > 0:
                    time.sleep(ahead)
    except KeyboardInterrupt:
        pass
    for uid in callbacks:
        SimulationManager.deregister_callback(uid)
    wall = time.monotonic() - wall_start
    sim = robot.t - sim_start
    print(f"[kachaka] stopped: {sim:.1f} s sim in {wall:.1f} s wall (RTF {sim / max(wall, 1e-6):.2f})", flush=True)
    if timing[2]:
        print(f"[kachaka] per loop: update {timing[0] / timing[2] * 1e3:.1f} ms, "
              f"cameras {timing[1] / timing[2] * 1e3:.1f} ms", flush=True)
    if recorder:
        recorder.close()
        print(f"[kachaka] recorded {recorder.count} frames to {args.record}", flush=True)
    if api:
        api.stop()
    node.shutdown()
    # SimulationApp.close() can abort on Isaac Sim 6.1; nothing is left to save.
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
