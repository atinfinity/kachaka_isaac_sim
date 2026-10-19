"""Convert the official Kachaka URDF into USD for Isaac Sim.

    uv run python scripts/fetch_assets.py   # once
    uv run python scripts/build_usd.py

Output: assets/generated/kachaka/kachaka.usda (plus build/kachaka.urdf, the
expanded URDF that robot_state_publisher on the ROS 2 side also uses).
Physics tuning (drive gains, friction) is applied at run time from
config/robot.yaml, so changing it does not need a rebuild.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kachaka_isaac_sim.urdf_tools import build_sim_urdf, write_urdf

PKG = ROOT / "third_party/kachaka-api/ros2/kachaka_description"
URDF_OUT = ROOT / "build/kachaka.urdf"
USD_DIR = ROOT / "assets/generated"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf-only", action="store_true", help="only write build/kachaka.urdf")
    args = parser.parse_args()

    if not PKG.exists():
        print("third_party/kachaka-api is missing; run scripts/fetch_assets.py first", file=sys.stderr)
        return 1
    robot = build_sim_urdf(PKG / "robot/kachaka.urdf.xacro", {"kachaka_description": PKG})
    write_urdf(robot, URDF_OUT)
    print(f"[build] wrote {URDF_OUT}")
    if args.urdf_only:
        return 0

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    from isaacsim.core.utils.extensions import enable_extension

    for ext in ("isaacsim.asset.importer.urdf", "isaacsim.robot.schema", "omni.scene.optimizer.core"):
        enable_extension(ext)
    app.update()

    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    out_dir = USD_DIR / "kachaka"
    if out_dir.exists():
        shutil.rmtree(out_dir)
    USD_DIR.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(URDF_OUT),
        usd_path=str(USD_DIR),
        merge_fixed_joints=False,  # keep sensor frames as prims
        fix_base=False,
        robot_type="Wheeled",
        ros_package_paths=[{"name": "kachaka_description", "path": str(PKG)}],
        joint_drive_type="force",
        joint_target_type={".*drive_wheel_joint": "velocity", "docking_joint": "position"},
        override_joint_stiffness={".*drive_wheel_joint": 0.0, "docking_joint": 2000.0},
        override_joint_damping={".*drive_wheel_joint": 10.0, "docking_joint": 50.0},
    )
    usd_path = URDFImporter(config).import_urdf()
    print(f"[build] wrote {usd_path}", flush=True)
    # SimulationApp.close() aborts in Isaac Sim 6.1 after an import; the files
    # are already written, so exit directly.
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
