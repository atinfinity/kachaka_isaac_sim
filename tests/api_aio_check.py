"""Check the simulator's kachaka-api gRPC server with the asyncio client
(kachaka_api.aio.KachakaApiClient).

Run on the host against the simulator (scripts/run_sim.sh, kachaka-api on
port 26400) with SLAM + Nav2 up (slam.launch.py nav:=true). Needs only the
pip package, not ROS:

    uv run --no-project --with kachaka-api==3.17.5.0 python tests/api_aio_check.py
    python tests/api_aio_check.py --target 127.0.0.1:26400 --no-cameras --no-shelf

Steps:

1. Getters: robot info, pose, battery, odometry, LiDAR, map, locations, shelves.
2. Every stream of the client (`client.<name>.stream()`) yields a first value;
   the sensor streams keep yielding new ones.
3. Commands with and without `wait_for_completion`; the pose, command state
   and last result streams follow a move while it runs.
4. Setting streams (auto homing, manual control) yield the new value on a change.
5. `set_robot_pose_callback` gets poses; setting None stops it.
6. move_shelf / return_shelf show up on the moving_shelf_id stream (skip
   with --no-shelf).
7. The error stream reports the emergency stop; restart_robot clears it.
"""

import argparse
import asyncio
import math
import sys
import time

from kachaka_api.aio import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2

FIRST_TIMEOUT = 5.0  # a first value comes at once: the server has one already
NEXT_TIMEOUT = 5.0
ERROR_EMERGENCY_STOP = -14

# Streams that keep yielding new values while the simulator runs, and how to
# tell one value from the next.
SENSOR_STREAMS = {
    "robot_pose": None,
    "ros_odometry": lambda m: m.header.stamp_nsec,
    "ros_wheel_odometry": lambda m: m.header.stamp_nsec,
    "ros_imu": lambda m: m.header.stamp_nsec,
    "ros_laser_scan": lambda m: m.header.stamp_nsec,
    "front_camera_ros_image": lambda m: m.header.stamp_nsec,
    "front_camera_ros_compressed_image": lambda m: m.header.stamp_nsec,
    "object_detection": lambda m: m[0].stamp_nsec,
    "object_detection_features": lambda m: m[0].stamp_nsec,
    "png_map": None,  # slam_toolbox publishes the map every few seconds
}
# Streams that yield again only when their value changes.
EVENT_STREAMS = [
    "command_state", "last_command_result", "shelves", "moving_shelf_id", "locations",
    "map_list", "current_map_id", "history_list", "auto_homing_enabled",
    "manual_control_enabled", "error",
]
CAMERA_STREAMS = {"front_camera_ros_image", "front_camera_ros_compressed_image"}


class Stream:
    """One stream with a timeout per value; close() cancels the pending read."""

    def __init__(self, client, name: str):
        self.name = name
        self.gen = getattr(client, name).stream()

    async def next(self, timeout: float):
        return await asyncio.wait_for(self.gen.__anext__(), timeout)

    async def close(self):
        await self.gen.aclose()


def _ignore_callback_stop(loop, context) -> None:
    """kachaka-api 3.17.5.0: after set_*_callback(None) the handler's task
    calls the None callback once with the value it was waiting for, and the
    TypeError goes unretrieved. Harmless; say so instead of a traceback."""
    exc = context.get("exception")
    if isinstance(exc, TypeError) and "NoneType" in str(exc):
        print("(note: kachaka-api raised its known TypeError after a callback was stopped)", flush=True)
        return
    loop.default_exception_handler(context)


async def run(args) -> list[str]:
    asyncio.get_running_loop().set_exception_handler(_ignore_callback_stop)
    client = KachakaApiClient(args.target)
    failures: list[str] = []

    def check(name, ok, detail=""):
        print(f"{name}: {'OK' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
        if not ok:
            failures.append(name)

    def result(name, res, t0, expect=True):
        check(name, res.success == expect, f"in {time.monotonic() - t0:.0f} s (code {res.error_code})")

    # 1. Getters.
    check("serial number", await client.get_robot_serial_number() != "")
    check("version", await client.get_robot_version() != "", await client.get_robot_version())
    pose = await client.get_robot_pose()
    check("pose", math.isfinite(pose.x) and math.isfinite(pose.theta), f"({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f})")
    level, status = await client.get_battery_info()
    check("battery", 0 < level <= 100, f"({level:.0f} %, {pb2.PowerSupplyStatus.Name(status)})")
    check("odometry", (await client.get_ros_odometry()).header.stamp_nsec > 0)
    scan = await client.get_ros_laser_scan()
    check("laser scan", len(scan.ranges) > 100, f"({len(scan.ranges)} rays)")
    m = await client.get_png_map()
    check("png map", m.width > 0 and m.data[:4] == b"\x89PNG", f"({m.width}x{m.height})")
    locations = {loc.id: loc for loc in await client.get_locations()}
    shelves = {s.id: s for s in await client.get_shelves()}
    check("locations", {"home", "L01"} <= locations.keys(), str(sorted(locations)))
    check("shelves", "S01" in shelves or args.no_shelf, str(sorted(shelves)))
    await client.update_resolver()

    # 2. Every stream yields a first value; the sensor streams keep going.
    for name in [*SENSOR_STREAMS, *EVENT_STREAMS]:
        if args.no_cameras and name in CAMERA_STREAMS:
            continue
        s = Stream(client, name)
        try:
            await s.next(FIRST_TIMEOUT)
        except TimeoutError:
            check(f"stream {name}: first value", False, f"(none in {FIRST_TIMEOUT:.0f} s)")
            await s.close()
            continue
        if name not in SENSOR_STREAMS:
            check(f"stream {name}: first value", True)
            await s.close()
            continue
        key = SENSOR_STREAMS[name]
        t0, n, keys = time.monotonic(), 0, set()
        timeout = 15.0 if name == "png_map" else NEXT_TIMEOUT
        want = 2 if name == "png_map" else 5
        try:
            while n < want:
                v = await s.next(timeout)
                n += 1
                if key is not None:
                    keys.add(key(v))
        except TimeoutError:
            pass
        dt = time.monotonic() - t0
        fresh = key is None or len(keys) == n
        check(f"stream {name}: new values", n == want and fresh, f"({n} in {dt:.1f} s)")
        await s.close()

    # 3. Commands, and streams following a move.
    t0 = time.monotonic()
    result("speak (wait_for_completion)", await client.speak("hello"), t0)
    t0 = time.monotonic()
    res = await client.speak("hello again", wait_for_completion=False)
    check("speak (no wait) returns at once", res.success and time.monotonic() - t0 < 1.0,
          f"in {time.monotonic() - t0:.2f} s")
    while await client.is_command_running():
        await asyncio.sleep(0.2)

    poses: list[pb2.Pose] = []
    states: list[int] = []
    results: list[tuple[pb2.Result, pb2.Command]] = []

    async def follow(name, out, pick=lambda v: v):
        async for v in getattr(client, name).stream():
            out.append(pick(v))

    tasks = [asyncio.create_task(follow("robot_pose", poses)),
             asyncio.create_task(follow("command_state", states, lambda v: v[0])),
             asyncio.create_task(follow("last_command_result", results))]
    await asyncio.sleep(0.5)
    n0 = len(results)
    t0 = time.monotonic()
    res = await client.move_to_pose(1.0, -0.3, 0.0, wait_for_completion=False)
    check("move_to_pose (no wait) returns at once", res.success and time.monotonic() - t0 < 1.0,
          f"in {time.monotonic() - t0:.2f} s")
    while time.monotonic() - t0 < 60.0 and not any(
            r.success and c.HasField("move_to_pose_command") for r, c in results[n0:]):
        await asyncio.sleep(0.2)
    for t in tasks:
        t.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
    check("last_command_result stream: move_to_pose succeeded",
          any(r.success and c.HasField("move_to_pose_command") for r, c in results[n0:]),
          f"in {time.monotonic() - t0:.0f} s")
    check("command_state stream: running then idle",
          pb2.COMMAND_STATE_RUNNING in states and states[-1] != pb2.COMMAND_STATE_RUNNING,
          str([pb2.CommandState.Name(s) for s in dict.fromkeys(states)]))
    moved = math.hypot(poses[-1].x - poses[0].x, poses[-1].y - poses[0].y) if poses else 0.0
    check("robot_pose stream during the move", len(poses) > 20 and moved > 0.1,
          f"({len(poses)} poses, moved {moved:.2f} m)")
    t0 = time.monotonic()
    result("move_to_location home (wait_for_completion)", await client.move_to_location("home"), t0)

    # 4. Setting streams yield on a change.
    for name, setter in (("auto_homing_enabled", client.set_auto_homing_enabled),
                         ("manual_control_enabled", client.set_manual_control_enabled)):
        s = Stream(client, name)
        before = await s.next(FIRST_TIMEOUT)
        await setter(not before)
        try:
            after = await s.next(NEXT_TIMEOUT)
        except TimeoutError:
            after = None
        await setter(before)
        check(f"stream {name}: change", after == (not before), f"({before} -> {after})")
        await s.close()

    # 5. Callback API.
    got: list[pb2.Pose] = []

    async def on_pose(p):
        got.append(p)

    client.set_robot_pose_callback(on_pose)
    await asyncio.sleep(2.0)
    client.set_robot_pose_callback(None)
    await asyncio.sleep(0.5)
    n = len(got)
    await asyncio.sleep(1.0)
    check("set_robot_pose_callback", n > 5 and len(got) <= n + 1, f"({n} poses in 2 s, then {len(got) - n})")

    # 6. Shelves on the moving_shelf_id stream.
    if not args.no_shelf and "S01" in shelves:
        ids: list[str] = []
        task = asyncio.create_task(follow("moving_shelf_id", ids))
        t0 = time.monotonic()
        result("move_shelf S01 -> L01", await client.move_shelf("S01", "L01"), t0)
        t0 = time.monotonic()
        result("return_shelf S01", await client.return_shelf("S01"), t0)
        await asyncio.sleep(0.5)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        check("moving_shelf_id stream", "S01" in ids and ids[-1] == "", str(list(dict.fromkeys(ids))))
        # No return_home after this: docking after a shelf trip fails often
        # (SLAM drift, issue #39), which is not what this check is about.

    # 7. Error stream.
    s = Stream(client, "error")
    await s.next(FIRST_TIMEOUT)
    await client.set_emergency_stop()
    try:
        on = list(await s.next(NEXT_TIMEOUT))
        await client.restart_robot()
        off = list(await s.next(NEXT_TIMEOUT))
    except TimeoutError:
        on = off = None
        await client.restart_robot()
    check("error stream: emergency stop", on == [ERROR_EMERGENCY_STOP] and off == [], f"({on} -> {off})")
    await s.close()

    return failures


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--no-cameras", action="store_true", help="the simulator runs with --no-cameras")
    ap.add_argument("--no-shelf", action="store_true", help="skip move_shelf / return_shelf")
    failures = asyncio.run(run(ap.parse_args()))
    print("ALL PASSED" if not failures else f"FAILED: {', '.join(failures)}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
