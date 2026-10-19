"""Run the Kachaka simulator. Start it through scripts/run_sim.sh, which sets
up the ROS 2 environment that Isaac Sim needs before the process starts.

    scripts/run_sim.sh                 # GUI, simple walled room
    scripts/run_sim.sh --headless      # no window
    scripts/run_sim.sh --world empty   # floor only
    scripts/run_sim.sh --world office  # Isaac Sim indoor scene (needs internet)
    scripts/run_sim.sh --world /path/to/scene.usd
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kachaka_isaac_sim.config import DEFAULT_CONFIG, load_config

ROBOT_USD = ROOT / "assets/generated/kachaka/kachaka.usda"
ROBOT_URDF = ROOT / "build/kachaka.urdf"
SHELF_STL = ROOT / "third_party/kachaka_shelf_description/meshes/kachaka_shelf_3layer.stl"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot parameters (YAML)")
    p.add_argument("--headless", action="store_true", help="run without a window")
    p.add_argument("--world", default="room",
                   help="'room', 'empty', an Isaac scene (office, hospital, warehouse, simple_room) or a USD path/URL")
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
    args = p.parse_args()
    from kachaka_isaac_sim.scenes import SPAWN

    default = SPAWN.get(args.world, (0.0, 0.0, 0.0))
    for i, k in enumerate(("x", "y", "yaw")):
        if getattr(args, k) is None:
            setattr(args, k, default[i])
    return args


def publish_cameras(node, cameras, t: float, tof_off: bool = False) -> None:
    for cam in cameras:
        if not cam.due(t) or (tof_off and cam.kind == "depth"):
            continue
        data = cam.read()
        if data is None:
            continue
        if cam.kind == "rgb":
            node.publish_image(t, cam.name, cam.cfg.frame_id, data, "rgb8", cam.fx)
        elif cam.cfg.encoding == "16UC1":
            mm = (data * 1000.0).round().clip(0, 65535).astype("uint16")
            node.publish_image(t, cam.name, cam.cfg.frame_id, mm, "16UC1", cam.fx)
        else:
            node.publish_image(t, cam.name, cam.cfg.frame_id, data, "32FC1", cam.fx)


def place_shelves(args, cfg):
    """Shelf specs and extra locations for this world."""
    from kachaka_isaac_sim.drive import Pose2D

    if args.no_shelves or not SHELF_STL.exists():
        return [], []
    if args.shelf:
        entries = [{"id": f"S{i + 1:02d}", "name": f"Shelf {i + 1}", "pose": pose}
                   for i, pose in enumerate(args.shelf)]
        locations = []
    elif args.world == "room":
        entries = [vars(e) for e in cfg.shelf.room]
        locations = [{"id": e.id, "name": e.name, "pose": Pose2D(*e.pose)} for e in cfg.shelf.room_locations]
    else:
        return [], []
    from kachaka_isaac_sim.shelves import ShelfSpec

    specs = [ShelfSpec(e["id"], e["name"], f"/World/Shelves/{e['id']}", Pose2D(*e["pose"])) for e in entries]
    return specs, locations


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
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

    from kachaka_isaac_sim import world
    from kachaka_isaac_sim.ros_node import KachakaSimNode

    stage = stage_utils.create_new_stage()
    world.setup_stage(stage, args.physics_hz, args.dome_light)
    builtin = args.world in ("room", "empty")
    world.add_ground(stage, visible=builtin)
    if args.world == "room":
        world.add_simple_room(stage)
    elif not builtin:
        world.load_world_usd(stage, world.resolve_world(args.world))
    charger_pose = None
    if not args.no_charger:
        if args.charger:
            charger_pose = tuple(args.charger)
        elif args.world == "room":
            charger_pose = tuple(cfg.charger.room_pose)
    if charger_pose:
        world.add_charger(stage, "/World/Charger", charger_pose, cfg.charger)
    shelf_specs, locations = place_shelves(args, cfg)
    for spec in shelf_specs:
        world.add_shelf(stage, spec.path, SHELF_STL, (spec.home.x, spec.home.y, spec.home.yaw), cfg.shelf)
    world.add_robot(stage, ROBOT_USD, args.x, args.y, args.yaw)
    world.apply_contact_materials(stage, cfg.drive)
    world.disable_pin_collision(stage)
    from kachaka_isaac_sim.torches import Torches
    from kachaka_isaac_sim.urdf_tools import read_joints

    torches = Torches(cfg.torch)
    torches.create(stage, world.BASE_LINK, read_joints(ROBOT_URDF))
    cameras = []
    if not args.no_cameras:
        from kachaka_isaac_sim.cameras import create_cameras

        cameras = create_cameras(stage, cfg, world.BASE_LINK, read_joints(ROBOT_URDF))
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

    node = KachakaSimNode(cfg, ROBOT_URDF)
    for cam in cameras:
        node.add_camera(cam.name)
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
        from kachaka_isaac_sim.world import SHELF_SIZE

        shelves = Shelves(stage, shelf_specs, cfg.shelf)
        node.shelf_reset_handler = shelves.reset

        def in_map(p):
            m = relative_pose(spawn, p)
            return [round(m.x, 4), round(m.y, 4), round(m.yaw, 4)]

        node.publish_layout(
            [{"id": s.id, "name": s.name, "home": in_map(s.home), "width": SHELF_SIZE[1], "depth": SHELF_SIZE[0]}
             for s in shelf_specs],
            [{"id": loc["id"], "name": loc["name"], "pose": in_map(loc["pose"])} for loc in locations])
    sim_start = SimulationManager.get_simulation_time()
    robot = SimRobot(world.BASE_LINK, world.ROBOT_PATH, ROBOT_URDF, cfg, node, sim_start, args.seed, charger,
                     shelves, spawn)
    node.reset_handler = robot.reset
    callbacks = [
        SimulationManager.register_callback(lambda dt, ctx: robot.pre_step(dt), SimulationEvent.PHYSICS_PRE_STEP),
        SimulationManager.register_callback(lambda dt, ctx: robot.post_step(dt), SimulationEvent.PHYSICS_POST_STEP),
    ]

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
            t1 = time.perf_counter()
            publish_cameras(node, cameras, robot.t, tof_off=robot.sensors_paused)
            if recorder:
                recorder.capture(robot.t - sim_start, (robot.gt.x, robot.gt.y))
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
