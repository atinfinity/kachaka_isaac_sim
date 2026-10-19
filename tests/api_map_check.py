"""Check the kachaka-api map RPCs that need a saved map: SwitchMap,
SetRobotPose and GetCurrentMapId.

Run on the host against the simulator in the default room, with Nav2 on a
saved map made from the same spawn pose:

    python tests/api_check.py ...           # or any SLAM run, then:
    python tests/api_map_check.py --save    # export the SLAM map into maps/
    docker compose -f docker/compose.yaml run --rm ros \\
        ros2 launch kachaka_sim_bringup navigation.launch.py map:=/maps/slam.yaml rviz:=false
    python tests/api_map_check.py --map slam

The SLAM map frame starts at the spawn pose, as does odometry, so the robot's
odometry pose is its pose on the saved map until it has driven far.
"""

import argparse
import math
import sys
import tempfile
import time
from pathlib import Path

from kachaka_api import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2

MAPS = Path(__file__).resolve().parents[1] / "maps"
ERROR_NAVIGATION = -10
ERROR_MAP = -15
ERROR_PRO_ONLY = -17


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--save", action="store_true", help="export the live SLAM map into maps/ and exit")
    ap.add_argument("--map", default="slam", help="the map navigation.launch.py serves")
    args = ap.parse_args()
    client = KachakaApiClient(args.target)

    if args.save:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "slam.zip"
            if not client.export_map("sim", str(path)).success:
                print("export failed")
                return 1
            res, map_id = client.import_map(str(path))
        print(f"saved maps/{map_id}.yaml" if res.success else "import failed")
        return 0 if res.success else 1

    failures = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    def wait_pose(x, y, timeout=10.0):
        end = time.monotonic() + timeout
        while True:
            p = client.get_robot_pose()
            err = math.hypot(p.x - x, p.y - y)
            if err < 0.15 or time.monotonic() > end:
                return err
            time.sleep(0.5)

    def move(x, y, yaw, wait=60.0):
        """move_to_pose; Nav2 rejects goals until AMCL has a pose and
        bt_navigator is active, so a quick -10 is retried for a while."""
        t0 = time.monotonic()
        while True:
            t = time.monotonic()
            res = client.move_to_pose(x, y, yaw)
            quick = time.monotonic() - t < 1.0
            if res.success or res.error_code != ERROR_NAVIGATION or not quick or time.monotonic() - t0 > wait:
                return res, time.monotonic() - t0
            time.sleep(2.0)

    ids = [m.id for m in client.get_map_list()]
    check("map list", args.map in ids and "sim" not in ids, str(ids))
    check("current map", client.get_current_map_id() == args.map, client.get_current_map_id())

    o = client.get_ros_odometry().pose.pose
    q = o.orientation
    pose = {"x": o.position.x, "y": o.position.y,
            "theta": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))}
    check("set_robot_pose", client.set_robot_pose(pose).success)
    err = wait_pose(pose["x"], pose["y"])
    check("localized", err < 0.15, f"(error {err:.2f} m)")
    res, dt = move(1.6, -0.3, math.pi)
    check("move_to_pose on the saved map", res.success, f"in {dt:.0f} s (code {res.error_code})")

    # A second copy of the map under another id, then switch to it.
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "copy.zip"
        client.export_map(args.map, str(path))
        res, copy_id = client.import_map(str(path))
    check("import a copy", res.success and copy_id != args.map, copy_id)
    p = client.get_robot_pose()
    res = client.switch_map(copy_id, pose={"x": p.x, "y": p.y, "theta": p.theta})
    check("switch_map", res.success, f"(code {res.error_code})")
    check("current map after switch", client.get_current_map_id() == copy_id, client.get_current_map_id())
    err = wait_pose(p.x, p.y)
    check("pose kept", err < 0.15, f"(error {err:.2f} m)")
    res, dt = move(0.0, 0.0, 0.0)
    check("move_to_pose on the new map", res.success, f"in {dt:.0f} s (code {res.error_code})")

    res = client.switch_map("nope")
    check("switch_map unknown fails", not res.success and res.error_code == ERROR_MAP)
    res = client.stub.SwitchMap(pb2.SwitchMapRequest(map_id=args.map, inherit_docking_state_and_docked_shelf=True))
    check("switch_map inherit is Pro only", not res.result.success and res.result.error_code == ERROR_PRO_ONLY)

    p = client.get_robot_pose()
    res = client.switch_map(args.map, pose={"x": p.x, "y": p.y, "theta": p.theta})
    check("switch back", res.success and client.get_current_map_id() == args.map)
    for f in MAPS.glob(f"{copy_id}.*"):
        f.unlink()

    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
