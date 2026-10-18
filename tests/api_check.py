"""Check the simulator's kachaka-api gRPC server with the official client.

Run on the host against the simulator (scripts/run_sim.sh, kachaka-api on
port 26400) with SLAM + Nav2 up (slam.launch.py nav:=true). Needs only the
pip package, not ROS:

    uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_check.py
    python tests/api_check.py --target 127.0.0.1:26400 --no-cameras

Steps:

1. Robot info, pose, battery, odometry, IMU, LiDAR, cameras and the PNG map.
2. Locations and shelves.
3. speak, move_to_pose and move_to_location; command state while running.
4. cancel_command stops a running command.
5. set_robot_velocity turns the robot under manual control.
6. move_shelf and return_shelf (skip with --no-shelf).
7. return_home docks and charges.
"""

import argparse
import io
import math
import sys
import threading
import time

from kachaka_api import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2
from PIL import Image

XY_TOL = 0.3
YAW_TOL = 0.35
ERROR_CANCELED = -5


def _angle(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--no-cameras", action="store_true", help="the simulator runs with --no-cameras")
    ap.add_argument("--no-shelf", action="store_true", help="skip move_shelf / return_shelf")
    args = ap.parse_args()

    client = KachakaApiClient(args.target)
    failures = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    def result(name, res, t0, expect=True, code=None):
        ok = res.success == expect and (code is None or res.error_code == code)
        check(name, ok, f"in {time.monotonic() - t0:.0f} s (code {res.error_code})")

    def at(name, x, y, yaw):
        p = client.get_robot_pose()
        err, dyaw = math.hypot(p.x - x, p.y - y), _angle(p.theta, yaw)
        check(name, err < XY_TOL and dyaw < YAW_TOL, f"(error {err:.2f} m, {dyaw:.2f} rad)")

    # 1. Robot info and sensors.
    check("serial number", client.get_robot_serial_number() != "")
    check("version", client.get_robot_version() != "", client.get_robot_version())
    pose = client.get_robot_pose()
    check("pose", math.isfinite(pose.x) and math.isfinite(pose.theta), f"({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f})")
    level, status = client.get_battery_info()
    check("battery", 0 < level <= 100, f"({level:.0f} %, {pb2.PowerSupplyStatus.Name(status)})")
    odom = client.get_ros_odometry()
    check("odometry", odom.header.stamp_nsec > 0 and odom.pose.pose.orientation.w != 0)
    check("wheel odometry", client.get_ros_wheel_odometry().header.stamp_nsec > 0)
    imu = client.get_ros_imu()
    check("imu", abs(imu.linear_acceleration.z - 9.8) < 1.0, f"(az {imu.linear_acceleration.z:.2f})")
    scan = client.get_ros_laser_scan()
    check("laser scan", len(scan.ranges) > 100 and scan.range_max > 0, f"({len(scan.ranges)} rays)")
    if not args.no_cameras:
        for cam in ("front", "back"):
            info = getattr(client, f"get_{cam}_camera_ros_camera_info")()
            img = getattr(client, f"get_{cam}_camera_ros_image")()
            jpg = getattr(client, f"get_{cam}_camera_ros_compressed_image")()
            size = Image.open(io.BytesIO(jpg.data)).size if jpg.data else None
            check(f"{cam} camera", img.width == info.width > 0 and len(img.data) == img.step * img.height
                  and size == (img.width, img.height), f"({img.width}x{img.height} {img.encoding}, {jpg.format})")
    m = client.get_png_map()
    png = Image.open(io.BytesIO(m.data))
    check("png map", png.size == (m.width, m.height) and m.resolution > 0 and m.name != "",
          f"({m.width}x{m.height}, {m.resolution} m/px)")
    check("map list", [x.id for x in client.get_map_list()] == [client.get_current_map_id()])
    check("no errors", client.get_error() == [])

    # 2. Layout.
    locations = {loc.id: loc for loc in client.get_locations()}
    shelves = {s.id: s for s in client.get_shelves()}
    check("locations", {"home", "L01"} <= locations.keys(), str(sorted(locations)))
    check("default location", client.get_default_location_id() == "home")
    check("shelves", "S01" in shelves and shelves["S01"].home_location_id in locations, str(sorted(shelves)))
    client.update_resolver()

    # 3. Commands.
    t0 = time.monotonic()
    result("speak", client.speak("hello"), t0)
    t0 = time.monotonic()
    result("move_to_pose", client.move_to_pose(1.6, -0.3, math.pi), t0)
    at("at the pose", 1.6, -0.3, math.pi)
    last, cmd = client.get_last_command_result()
    check("last command result", last.success and cmd.HasField("move_to_pose_command")
          and abs(cmd.move_to_pose_command.x - 1.6) < 1e-6)
    t0 = time.monotonic()
    result("move_to_location unknown fails", client.move_to_location("nowhere"), t0, expect=False)

    done = {}
    th = threading.Thread(target=lambda: done.setdefault("res", client.move_to_location("L01")))
    th.start()
    time.sleep(2.0)
    state, cmd = client.get_command_state()
    check("command running", state == pb2.CommandState.COMMAND_STATE_RUNNING
          and cmd.move_to_location_command.target_location_id == "L01")
    th.join(120.0)
    check("move_to_location L01", done.get("res") is not None and done["res"].success)
    p = locations["L01"].pose
    at("at L01", p.x, p.y, p.theta)
    check("no command running", not client.is_command_running())

    # 4. cancel_command.
    t0 = time.monotonic()
    client.move_to_pose(-1.8, 1.3, math.pi / 2, wait_for_completion=False)
    time.sleep(3.0)
    res, cmd = client.cancel_command()
    check("cancel_command", res.success and cmd.HasField("move_to_pose_command"))
    end = time.monotonic() + 10.0
    while client.is_command_running() and time.monotonic() < end:
        time.sleep(0.2)
    last, _ = client.get_last_command_result()
    check("canceled command result", not last.success and last.error_code == ERROR_CANCELED,
          f"in {time.monotonic() - t0:.0f} s (code {last.error_code})")

    # 5. Manual control.
    yaw0 = client.get_robot_pose().theta
    client.set_manual_control_enabled(False)
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        res = client.set_robot_velocity(0.0, 0.8)
        time.sleep(0.1)
    client.set_robot_stop()
    time.sleep(1.0)
    turned = _angle(client.get_robot_pose().theta, yaw0)
    check("set_robot_velocity turns", res.success and turned > 0.5, f"({turned:.2f} rad)")
    check("manual control off", not client.get_manual_control_enabled())

    # 6. Shelf.
    if not args.no_shelf and "S01" in shelves:
        t0 = time.monotonic()
        result("move_shelf S01 -> L01", client.move_shelf("S01", "L01"), t0)
        check("moving shelf", client.get_moving_shelf_id() == "S01")
        t0 = time.monotonic()
        result("return_shelf S01", client.return_shelf("S01"), t0)
        check("no moving shelf", client.get_moving_shelf_id() == "")

    # 7. return_home.
    t0 = time.monotonic()
    result("return_home", client.return_home(), t0)
    end = time.monotonic() + 5.0
    while client.get_battery_info()[1] != pb2.PowerSupplyStatus.POWER_SUPPLY_STATUS_CHARGING and time.monotonic() < end:
        time.sleep(0.2)
    check("charging", client.get_battery_info()[1] == pb2.PowerSupplyStatus.POWER_SUPPLY_STATUS_CHARGING)

    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
