"""Write a saved map per floor of a layout with floors and an elevator, drawn
from the layout (kachaka_isaac_sim/floor_maps.py), into the maps folder
(mounted at /maps in the ROS container):

    python3 scripts/make_maps.py layouts/elevator.yaml   # maps/elevator_1F.yaml, maps/elevator_2F.yaml
    ros2 launch kachaka_sim_bringup navigation.launch.py map:=/maps/elevator_1F.yaml

A layout without floors, or none, gets one map of the room:

    python3 scripts/make_maps.py --spawn -1.5 -0.8 0   # maps/room.yaml, map frame at that spawn

Existing maps of the same names are replaced. Plain Python (numpy, Pillow,
PyYAML): no ROS, no Isaac Sim.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.drive import Pose2D
from kachaka_isaac_sim.elevator import elevator_spec
from kachaka_isaac_sim.floor_maps import floor_map
from kachaka_isaac_sim.layout import LayoutError, parse_layout, read_layout
from kachaka_isaac_sim.scenes import SPAWN


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("layout", nargs="?", help="layout YAML (default: the plain room)")
    ap.add_argument("--config", help="config YAML (default: config/robot.yaml)")
    ap.add_argument("--out", default=str(ROOT / "maps"), help="maps folder (default: maps/)")
    ap.add_argument("--prefix", help="map id prefix (default: the layout's file name)")
    ap.add_argument("--resolution", type=float, default=0.05, help="m per cell (default: 0.05)")
    ap.add_argument("--spawn", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                    help="the robot's spawn, which is the map frame (default: the layout's or the room's); "
                         "with several robots, robot 0's")
    args = ap.parse_args()
    cfg = load_config(args.config)
    try:
        data = read_layout(args.layout) if args.layout else {}
        layout = parse_layout(data, world="room", path=args.layout or "")
    except (LayoutError, OSError) as e:
        print(f"layout: {e}", file=sys.stderr)
        return 1
    if layout.randomised:
        print("layout: randomised poses would differ from the simulator's; fix them first", file=sys.stderr)
        return 1
    spec = elevator_spec(layout.elevator, layout.floors, cfg.elevator) if layout.elevator else None
    spawn = Pose2D(*args.spawn) if args.spawn else layout.spawn or Pose2D(*SPAWN.get("room", (0.0, 0.0, 0.0)))
    charger = layout.charger if layout.charger or layout.no_charger else Pose2D(*cfg.charger.room_pose)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or (Path(args.layout).stem if args.layout else "room")
    for k, name in enumerate(layout.floors if spec else [None]):
        m = floor_map(layout, k, spec, spawn, args.resolution, charger)
        map_id = f"{prefix}_{name}" if name else prefix
        for fname, content in m.files(map_id).items():
            (out / fname).write_bytes(content)
        print(f"{out / map_id}.yaml: {name or 'room'}, {m.grid.shape[1]} x {m.grid.shape[0]} cells")
    return 0


if __name__ == "__main__":
    sys.exit(main())
