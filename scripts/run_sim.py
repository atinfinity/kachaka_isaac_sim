"""Run the Kachaka simulator. Start it through scripts/run_sim.sh, which sets
up the ROS 2 environment that Isaac Sim needs before the process starts.

    scripts/run_sim.sh                 # GUI, simple walled room
    scripts/run_sim.sh --headless      # no window
    scripts/run_sim.sh --world empty   # floor only
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

from kachaka_isaac_sim.config import DEFAULT_CONFIG, load_config  # noqa: E402

ROBOT_USD = ROOT / "assets/generated/kachaka/kachaka.usda"
ROBOT_URDF = ROOT / "build/kachaka.urdf"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot parameters (YAML)")
    p.add_argument("--headless", action="store_true", help="run without a window")
    p.add_argument("--world", default="room", help="'room', 'empty', or a USD file to load as the environment")
    p.add_argument("--x", type=float, default=0.0, help="spawn x [m]")
    p.add_argument("--y", type=float, default=0.0, help="spawn y [m]")
    p.add_argument("--yaw", type=float, default=0.0, help="spawn yaw [rad]")
    p.add_argument("--physics-hz", type=int, default=120)
    p.add_argument("--render-hz", type=float, default=30.0, help="render and main-loop rate")
    p.add_argument("--realtime", type=float, default=1.0,
                   help="target real-time factor; 0 runs as fast as possible")
    p.add_argument("--max-sim-time", type=float, default=0.0, help="exit after this many sim seconds (0: never)")
    p.add_argument("--seed", type=int, default=None, help="seed for odometry noise")
    return p.parse_args()


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
    app.update()
    SimulationManager.switch_physics_engine("physx")

    from kachaka_isaac_sim import world
    from kachaka_isaac_sim.ros_node import KachakaSimNode

    stage = stage_utils.create_new_stage()
    world.setup_stage(stage, args.physics_hz)
    world.add_ground(stage)
    if args.world == "room":
        world.add_simple_room(stage)
    elif args.world != "empty":
        world.load_world_usd(stage, args.world)
    world.add_robot(stage, ROBOT_USD, args.x, args.y, args.yaw)
    world.apply_contact_materials(stage, cfg.drive)
    app.update()

    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.core.simulation_manager import SimulationEvent

    from kachaka_isaac_sim.robot import SimRobot

    # Render (and run the main loop) slower than physics: rendering costs far
    # more than a physics step, and sensors need at most 30 Hz.
    RenderingManager.set_dt(1.0 / args.render_hz)
    app_utils.play()
    app.update()

    node = KachakaSimNode(cfg, ROBOT_URDF)
    sim_start = SimulationManager.get_simulation_time()
    robot = SimRobot(world.BASE_LINK, world.ROBOT_PATH, ROBOT_URDF, cfg, node, sim_start, args.seed)
    callbacks = [
        SimulationManager.register_callback(lambda dt, ctx: robot.pre_step(dt), SimulationEvent.PHYSICS_PRE_STEP),
        SimulationManager.register_callback(lambda dt, ctx: robot.post_step(dt), SimulationEvent.PHYSICS_POST_STEP),
    ]

    wall_start = time.monotonic()
    print(f"[kachaka] running: ROS namespace '{node.ns}', world '{args.world}'", flush=True)
    try:
        while app.is_running():
            node.spin_once()
            app.update()
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
    node.shutdown()
    # SimulationApp.close() can abort on Isaac Sim 6.1; nothing is left to save.
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
