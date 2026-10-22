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
7. move_forward (and stopping in front of a wall), rotate_in_place, lock,
   localize, move_by_velocity_with_muted_sensors.
8. dock_any_shelf_with_registration and reset_shelf_pose (skip with --no-shelf).
9. History, shortcuts, sounds, dynamic TF, object detection features.
10. Maps: export the SLAM map, import it and an image, preview them; SwitchMap
    and SetRobotPose need navigation.launch.py, so here they must fail.
11. return_home docks and charges; depart_from_charger leaves.
12. Emergency stop refuses commands until restart_robot.
"""

import argparse
import io
import math
import sys
import tempfile
import threading
import time
from pathlib import Path

from kachaka_api import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2
from PIL import Image

XY_TOL = 0.3
YAW_TOL = 0.35
ERROR_CANCELED = -5
ERROR_BLOCKED = -9
ERROR_OTHER_SHELF_DOCKED = -11
ERROR_NOT_ON_CHARGER = -13
ERROR_EMERGENCY_STOP = -14
ERROR_NEEDS_SAVED_MAP = -16
MAPS = Path(__file__).resolve().parents[1] / "maps"


def _angle(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _odom(client) -> tuple[float, float, float]:
    """Odometry pose: smooth, unlike the SLAM pose, so better for short moves."""
    p = client.get_ros_odometry().pose.pose
    q = p.orientation
    return p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


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
    check("map list", client.get_current_map_id() in [x.id for x in client.get_map_list()])
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

    # 7. Motion commands, from the room centre facing +x.
    t0 = time.monotonic()
    result("move_to_pose centre", client.move_to_pose(0.0, 0.0, 0.0), t0)
    time.sleep(1.0)  # let the robot settle
    p0 = _odom(client)
    t0 = time.monotonic()
    result("move_forward 0.4", client.move_forward(0.4), t0)
    p1 = _odom(client)
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    check("moved 0.4 m", abs(moved - 0.4) < 0.03 and _angle(p1[2], p0[2]) < 0.05,
          f"({moved:.3f} m, {_angle(p1[2], p0[2]):.3f} rad off)")
    t0 = time.monotonic()
    result("rotate_in_place -pi/2", client.rotate_in_place(-math.pi / 2), t0)
    p2 = _odom(client)
    check("turned -pi/2", _angle(p2[2], p1[2] - math.pi / 2) < 0.05,
          f"({math.atan2(math.sin(p2[2] - p1[2]), math.cos(p2[2] - p1[2])):.3f} rad)")
    t0 = time.monotonic()
    result("lock 2 s", client.lock(2.0), t0)
    p3 = _odom(client)
    check("lock kept still", math.hypot(p3[0] - p2[0], p3[1] - p2[1]) < 0.01 and time.monotonic() - t0 >= 1.0)
    t0 = time.monotonic()
    result("localize", client.localize(), t0)
    cmd = pb2.Command(move_by_velocity_with_muted_sensors_command=pb2.MoveByVelocityWithMutedSensorsCommand(
        signed_velocity=0.1, move_duration_sec=2.0))
    t0 = time.monotonic()
    result("move_by_velocity_with_muted_sensors", client.start_command(cmd), t0)
    p4 = _odom(client)
    moved = math.hypot(p4[0] - p3[0], p4[1] - p3[1])
    check("moved about 0.2 m", 0.12 < moved < 0.28, f"({moved:.3f} m)")
    t0 = time.monotonic()
    result("move_forward into the wall stops", client.move_forward(5.0), t0, expect=False, code=ERROR_BLOCKED)
    scan = client.get_ros_laser_scan()
    ahead = [r for i, r in enumerate(scan.ranges)
             if abs(scan.angle_min + i * scan.angle_increment) < 0.2 and math.isfinite(r)]
    check("stopped short of the wall", ahead and min(ahead) > 0.1, f"(LiDAR {min(ahead or [0]):.2f} m ahead)")
    # DEPART_FROM_CHARGER is in kachaka-api's proto on GitHub but not yet in a release.
    depart = getattr(pb2, "DepartFromChargerCommand", None)
    if depart:
        t0 = time.monotonic()
        result("depart_from_charger off the charger fails",
               client.start_command(pb2.Command(depart_from_charger_command=depart())), t0, expect=False,
               code=ERROR_NOT_ON_CHARGER)
    t0 = time.monotonic()
    result("move_forward backward 0.2", client.move_forward(-0.2), t0)

    # 8. Shelf at a location.
    if not args.no_shelf and "S01" in shelves:
        home = shelves["S01"].home_location_id
        t0 = time.monotonic()
        result(f"dock_any_shelf_with_registration {home}", client.dock_any_shelf_with_registration(home), t0)
        check("moving shelf S01", client.get_moving_shelf_id() == "S01")
        res = client.reset_shelf_pose("S01")
        check("reset_shelf_pose while towing fails", not res.success and res.error_code == ERROR_OTHER_SHELF_DOCKED)
        t0 = time.monotonic()
        result("return_shelf S01", client.return_shelf("S01"), t0)
        check("reset_shelf_pose S01", client.reset_shelf_pose("S01").success)
        time.sleep(1.0)  # shelf poses come at the simulator's publishing rate
        s = {x.id: x for x in client.get_shelves()}["S01"]
        h = locations[home].pose
        check("S01 at home", math.hypot(s.pose.x - h.x, s.pose.y - h.y) < 0.02,
              f"({math.hypot(s.pose.x - h.x, s.pose.y - h.y):.3f} m)")
        check("reset_shelf_pose unknown fails", not client.reset_shelf_pose("nope").success)

    # 9. History, shortcuts, sounds, TF, features.
    history = client.get_history_list()
    last, _ = client.get_last_command_result()
    check("history", len(history) >= 10 and history[0].command_executed_time > 1.6e9, f"({len(history)} entries)")
    shortcuts = client.get_shortcuts()
    check("shortcuts", "hello" in shortcuts, str(shortcuts))
    yaw0 = _odom(client)[2]
    t0 = time.monotonic()
    check("start_shortcut_command hello", client.start_shortcut_command("hello").success)
    end = time.monotonic() + 60.0
    while time.monotonic() < end:
        last, cmd = client.get_last_command_result()
        if cmd.HasField("rotate_in_place_command"):
            break
        time.sleep(0.5)
    turned = _angle(_odom(client)[2], yaw0)
    check("shortcut ran speak then rotate", cmd.HasField("rotate_in_place_command") and last.success and turned < 0.05,
          f"in {time.monotonic() - t0:.0f} s ({last.success}, {turned:.3f} rad from a full turn)")
    stub = client.stub
    sounds = stub.GetSoundList(pb2.GetRequest()).sounds
    added = stub.AddSound(pb2.AddSoundRequest(name="beep", data=b"RIFF"))
    ids = [x.id for x in stub.GetSoundList(pb2.GetRequest()).sounds]
    check("add sound", added.result.success and added.sound_id in ids and len(ids) == len(sounds) + 1)
    check("play sound", stub.PlaySound(pb2.PlaySoundRequest(sound_id=added.sound_id)).result.success)
    check("stop sound", stub.StopSound(pb2.StopSoundRequest()).result.success)
    check("delete sound", stub.DeleteSound(pb2.DeleteSoundRequest(sound_id=added.sound_id)).result.success
          and len(stub.GetSoundList(pb2.GetRequest()).sounds) == len(sounds))
    stream = stub.GetDynamicTransform(pb2.EmptyRequest())
    frames = set()
    for i, msg in enumerate(stream):
        frames |= {(t.header.frame_id, t.child_frame_id) for t in msg.transforms}
        if any(c.endswith("base_footprint") for _, c in frames) or i > 200:
            break
    stream.cancel()
    check("dynamic transform", any(c.endswith("base_footprint") for _, c in frames), str(sorted(frames)))
    header, features = client.get_object_detection_features()
    shapes = {f.name: tuple(f.shape) for f in features}
    want = {"p4_out": (1, 96, 14, 25), "p5_out": (1, 96, 7, 13), "p6_out": (1, 96, 4, 7)}
    check("object detection features", header.stamp_nsec > 0 and shapes == ({} if args.no_cameras else want)
          and all(len(f.data) == math.prod(f.shape) for f in features), str(shapes or "none (no front camera)"))
    # The stamps are wall clock, as on the robot (the ROS side is on sim time).
    lag = time.time() - client.get_ros_odometry().header.stamp_nsec / 1e9
    check("stamps are wall clock", -1.0 < lag < 5.0, f"(odometry stamp {lag:.2f} s behind time.time())")

    # 10. Maps.
    created = []
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "sim.zip"
        check("export SLAM map", client.export_map("sim", str(path)).success and path.stat().st_size > 0)
        res, map_id = client.import_map(str(path))
        created.append(map_id)
        check("import map", res.success and map_id in [x.id for x in client.get_map_list()], map_id)
    live, preview = client.get_png_map(), client.load_map_preview(map_id)
    check("imported map preview", (preview.width, preview.height) == (live.width, live.height)
          and abs(preview.origin.x - live.origin.x) < 1e-6, f"({preview.width}x{preview.height})")
    buf = io.BytesIO()
    img = Image.new("L", (40, 20), 255)
    img.paste(0, (0, 0, 40, 2))
    img.save(buf, format="PNG")
    res = stub.ImportImageAsMap(iter([pb2.ImportImageAsMapRequest(
        data=buf.getvalue(), resolution=0.1, charger_pose=pb2.Pose(x=1.0, y=0.5, theta=0.0))]))
    created.append(res.map_id)
    preview = client.load_map_preview(res.map_id)
    check("import image as map", res.result.success and (preview.width, preview.height) == (40, 20)
          and abs(preview.resolution - 0.1) < 1e-9, res.map_id)
    res = client.switch_map(map_id)
    check("switch_map needs a saved-map launch", not res.success and res.error_code == ERROR_NEEDS_SAVED_MAP)
    check("current map", client.get_current_map_id() == "sim")
    res = client.set_robot_pose({"x": 0.0, "y": 0.0, "theta": 0.0})
    check("set_robot_pose needs AMCL", not res.success and res.error_code == ERROR_NEEDS_SAVED_MAP)
    for map_id in created:
        for f in MAPS.glob(f"{map_id}.*"):
            f.unlink()

    # 11. return_home and depart_from_charger.
    t0 = time.monotonic()
    result("return_home", client.return_home(), t0)
    end = time.monotonic() + 5.0
    while client.get_battery_info()[1] != pb2.PowerSupplyStatus.POWER_SUPPLY_STATUS_CHARGING and time.monotonic() < end:
        time.sleep(0.2)
    check("charging", client.get_battery_info()[1] == pb2.PowerSupplyStatus.POWER_SUPPLY_STATUS_CHARGING)
    if depart:
        t0 = time.monotonic()
        result("depart_from_charger", client.start_command(pb2.Command(depart_from_charger_command=depart())), t0)
        time.sleep(1.0)
        check("off the charger",
              client.get_battery_info()[1] != pb2.PowerSupplyStatus.POWER_SUPPLY_STATUS_CHARGING)

    # 12. Emergency stop.
    check("set_emergency_stop", client.set_emergency_stop() == 0)
    check("error reported", client.get_error() == [ERROR_EMERGENCY_STOP], str(client.get_error()))
    res = client.move_forward(0.1)
    check("command refused", not res.success and res.error_code == ERROR_EMERGENCY_STOP)
    check("restart_robot", client.restart_robot().success and client.get_error() == [])
    t0 = time.monotonic()
    result("move_to_pose after restart", client.move_to_pose(0.0, 0.0, 0.0), t0)

    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
