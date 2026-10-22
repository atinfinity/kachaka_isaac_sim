"""Run the kachaka-api demo notebooks (pf-robotics/kachaka-api,
python/demos/*.ipynb) headless against the simulator (#64).

Each notebook runs as it is in a fresh kernel, with one cell put in front
that points the clients at the simulator: the notebooks use the default
address of the robot's own JupyterLab (100.94.1.1:26400).

    uv run --no-project --with kachaka-api==3.17.5.0 --with nbclient --with ipykernel \\
        --with opencv-python-headless --with matplotlib --with plotly --with ipywidgets --with anywidget --with ipympl \\
        --with pillow --with fastapi --with uvicorn --with nest_asyncio --with requests \\
        python tests/demo_notebooks_check.py

scripts/demo_notebooks.sh starts the simulator and Nav2 on a saved map first
and runs this. The notebooks come from a clone of kachaka-api at the tag of
the pinned client (--ref), in build/kachaka-api unless --demos says where.

Kinds of notebook:
  run     every cell must finish, without an error;
  stream  the notebook ends in a loop that never stops (camera, scan, error
          streams): each such cell is interrupted after --stream-timeout s
          and must have shown something by then;
  server  the notebook serves HTTP on :26502: the requests are sent while it
          serves, then it is interrupted;
  skip    not runnable here; the reason is printed.
Each kind may check the outputs further (CHECKS). The executed notebooks are
written to build/demo_notebooks/.
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
UPSTREAM = "https://github.com/pf-robotics/kachaka-api"
PORT = 26502  # the server notebooks' port

POINT_AT = """\
# Added by tests/demo_notebooks_check.py: point every client at the simulator.
import kachaka_api, kachaka_api.aio, kachaka_api.base, kachaka_api.aio.base, kachaka_api.util.vision
for _cls in (kachaka_api.KachakaApiClient, kachaka_api.aio.KachakaApiClient,
             kachaka_api.base.KachakaApiClientBase, kachaka_api.aio.base.KachakaApiClientBase,
             kachaka_api.util.vision.LaserScanActivator):
    _cls.__init__.__defaults__ = ({target!r},)
"""

# name: (kind, why). In the order they run: the ones that move the robot or
# change the map last.
NOTEBOOKS = {
    "get_front_camera": ("run", ""),
    "get_front_camera_continuous": ("stream", ""),
    "undistort": ("run", "the simulated camera has no distortion: both images are the same"),
    "feature_matching": ("stream", "nothing is held up to the camera, so few matches"),
    "qrcode": ("stream", "there is no QR code in the simulated rooms"),
    "get_object_detection": ("stream", ""),
    "get_imu": ("run", "the plot is a widget: not shown headless"),
    "get_laser_scan": ("stream", ""),
    "error_code": ("stream", "the simulator's error codes, not the robot's"),
    "speak": ("run", "no sound: the text goes to the log and sim/speech"),
    "template": ("run", ""),
    "teleop": ("run", "builds the buttons only: nobody clicks them headless"),
    "command_gui": ("run", "builds the form only: nobody clicks it headless"),
    "plot_map_robot_lidar": ("run", "the animation needs a live widget: headless draws nothing"),
    "http_server": ("server", ""),
    "url_kachaka_api": ("server", ""),
    "follow_path": ("run", ""),
    "robot_pose": ("run", ""),
    "switch_map": ("run", ""),
    "install_libraries": ("skip", "pip installs only, no API calls"),
    "run_custom_object_detection": ("skip", ("needs a trained predictor.onnx, and the simulator returns no "
                                             "object detection features (#70)")),
    "save_object_detection_features": ("skip", ("never ends: it waits for image stamps newer than the wall "
                                                "clock, and the simulator stamps with simulation time (#69)")),
}

# The HTTP requests sent to a server notebook: (method, path, json body).
REQUESTS = {
    "http_server": [("GET", "/run", None)],
    "url_kachaka_api": [("GET", "/kachaka/get_robot_serial_number", None),
                        ("GET", "/kachaka/get_robot_pose", None),
                        ("GET", "/kachaka/front_camera_image.jpeg", None)],
}


def text_of(nb) -> str:
    out = []
    for cell in nb.cells:
        for o in cell.get("outputs", []):
            if o.get("output_type") == "stream":
                out.append(o.get("text", ""))
            elif "data" in o:
                out.append(o["data"].get("text/plain", ""))
                out.append(o["data"].get("text/html", ""))
    return "".join(out)


def images_of(nb) -> int:
    return sum(1 for cell in nb.cells for o in cell.get("outputs", [])
               if "data" in o and ({"image/png", "image/jpeg"} & set(o["data"])))


def errors_of(nb) -> list[str]:
    return [f"{o.get('ename')}: {o.get('evalue')}" for cell in nb.cells for o in cell.get("outputs", [])
            if o.get("output_type") == "error"]


def check_switch_map(nb, _):
    ids = re.findall(r"current_map_id='([^']*)'", text_of(nb))
    targets = re.findall(r"target_map_id='([^']*)'", text_of(nb))
    ok = len(ids) == 2 and ids == targets and ids[0] != ids[1]
    return ok, f"switched to {ids[0]} and back to {ids[1]}" if ok else f"targets {targets}, current {ids}"


def check_robot_pose(nb, _):
    text = text_of(nb)
    poses = [(float(x), float(y)) for x, y in re.findall(r"pose\.x=(-?[\d.]+), pose\.y=(-?[\d.]+)", text)]
    # The second set_robot_pose puts the robot at (2, 1); the last pose is read 5 s later.
    moved = len(poses) == 2 and math.hypot(poses[1][0] - 2.0, poses[1][1] - 1.0) < 0.3
    ok = text.count("result.success=True") == 2 and moved
    return ok, f"set_robot_pose succeeded twice; poses {poses}, the last one expected near (2.00, 1.00)"


def check_error_code(nb, _):
    text = text_of(nb)
    rows = text.count("<tr><td>")
    ok = rows > 0 and "Waiting for next error" in text
    return ok, f"{rows} error codes listed, then the error stream"


def check_images(n):
    def check(nb, _):
        k = images_of(nb)
        return k >= n, f"{k} image{'s' if k != 1 else ''} shown"
    return check


def check_server(nb, replies):
    bad = [f"{path}: {status}" for path, status, _ in replies if status != 200]
    if bad:
        return False, ", ".join(bad)
    got = "; ".join(f"{path} -> {body}" for path, _, body in replies)
    return True, got


def check_http_server(nb, replies):
    ok, why = check_server(nb, replies)
    finished = "send_command finished." in text_of(nb)
    return ok and finished, why + ("" if finished else "; the speak command did not finish")


CHECKS = {
    "get_front_camera": check_images(2),
    "get_front_camera_continuous": check_images(1),
    "undistort": check_images(2),
    "feature_matching": check_images(1),
    "qrcode": check_images(1),
    "get_object_detection": check_images(1),
    "get_laser_scan": check_images(1),
    "error_code": check_error_code,
    "robot_pose": check_robot_pose,
    "switch_map": check_switch_map,
    "http_server": check_http_server,
    "url_kachaka_api": check_server,
}


def fetch(dest: Path, ref: str) -> None:
    if (dest / "python" / "demos").is_dir():
        return
    print(f"cloning {UPSTREAM} at {ref} into {dest}", flush=True)
    subprocess.run(["git", "clone", "-q", "--depth", "1", "--branch", ref, UPSTREAM, str(dest)], check=True)


def probe(requests_, deadline):
    """Send the requests once the server answers; (path, status, short body) each."""
    import requests

    replies = []
    for method, path, body in requests_:
        while True:
            try:
                r = requests.request(method, f"http://127.0.0.1:{PORT}{path}", json=body, timeout=20)
                break
            except requests.ConnectionError:
                if time.monotonic() > deadline:
                    replies.append((path, "no answer", ""))
                    r = None
                    break
                time.sleep(0.5)
        if r is not None:
            short = f"{len(r.content)} bytes {r.headers.get('content-type')}" \
                if "json" not in r.headers.get("content-type", "") else r.text[:80]
            replies.append((path, r.status_code, short))
    return replies


def save_map(target: str, name: str, copies: int = 2, wait: float = 120.0) -> int:
    """Save the live SLAM map into maps/ `copies` times as `name`, `name`_2, ...:
    switch_map needs two maps. A name sorting first keeps the notebook's
    switches between these (it takes the first map that is not the current)."""
    from kachaka_api import KachakaApiClient

    from kachaka_isaac_sim.maps import MapFile, MapStore

    client = KachakaApiClient(target)
    end = time.monotonic() + wait
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "slam.zip"
        while not client.export_map("sim", str(path)).success:
            if time.monotonic() > end:
                print("no SLAM map to save")
                return 1
            time.sleep(2.0)
        m = MapFile.from_zip(path.read_bytes())
    store = MapStore(REPO / "maps")
    ids = [store.save(m, name) for _ in range(copies)]
    print("saved " + " ".join(f"maps/{i}.yaml" for i in ids), flush=True)
    return 0


def localize(target: str, wait: float = 60.0) -> bool:
    """Give AMCL the robot's pose: odometry starts at the spawn pose, as does
    the SLAM map the saved map came from."""
    from kachaka_api import KachakaApiClient

    client = KachakaApiClient(target)
    end = time.monotonic() + wait
    while True:
        o = client.get_ros_odometry().pose.pose
        q = o.orientation
        pose = {"x": o.position.x, "y": o.position.y,
                "theta": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))}
        if client.set_robot_pose(pose).success:
            time.sleep(2.0)
            p = client.get_robot_pose()
            if math.hypot(p.x - pose["x"], p.y - pose["y"]) < 0.15:
                print(f"localized at ({p.x:.2f}, {p.y:.2f})", flush=True)
                return True
        if time.monotonic() > end:
            print("could not localize on the saved map", flush=True)
            return False
        time.sleep(2.0)


KILL_AFTER = 15.0  # s past a cell's timeout before its kernel is killed


def run(path: Path, kind: str, target: str, timeout: float, stream_timeout: float, out_dir: Path):
    """Execute one notebook. Returns (executed notebook, error list, replies)."""
    import nbformat
    from nbclient import NotebookClient
    from nbclient.exceptions import DeadKernelError

    nb = nbformat.read(path, as_version=4)
    nb.cells.insert(0, nbformat.v4.new_code_cell(POINT_AT.format(target=target)))
    cell_timeout = stream_timeout if kind in ("stream", "server") else timeout
    client = NotebookClient(nb, timeout=cell_timeout, kernel_name="python3", allow_errors=True,
                            interrupt_on_timeout=True, resources={"metadata": {"path": str(path.parent)}})
    replies = []
    # A kernel blocked in grpc.aio can miss the timeout's interrupt and never
    # answer (error_code's async for): kill it KILL_AFTER s past the timeout.
    deadline, killed, done = [math.inf], [], threading.Event()

    def on_cell_start(**_):
        deadline[0] = time.monotonic() + cell_timeout + KILL_AFTER

    def watchdog():
        while not done.wait(1.0):
            process = getattr(getattr(client.km, "provisioner", None), "process", None)
            if time.monotonic() > deadline[0] and process is not None:
                killed.append(True)
                os.kill(process.pid, signal.SIGKILL)
                return

    client.on_cell_start = on_cell_start
    threading.Thread(target=watchdog, daemon=True).start()
    prober = None
    if kind == "server":
        def go():
            replies.extend(probe(REQUESTS[path.stem], time.monotonic() + cell_timeout))
        prober = threading.Thread(target=go, daemon=True)
        prober.start()
    try:
        client.execute()
    except (DeadKernelError, RuntimeError, TimeoutError) as e:
        if not (killed and kind in ("stream", "server")):
            return nb, [f"{type(e).__name__}: {e}"], replies
    finally:
        done.set()
        if prober:
            prober.join(timeout=5)
        out_dir.mkdir(parents=True, exist_ok=True)
        nbformat.write(nb, out_dir / path.name)
    errors = errors_of(nb)
    if kind in ("stream", "server"):  # the loops end in the interrupt
        errors = [e for e in errors if not e.startswith(("KeyboardInterrupt", "CancelledError"))]
    return nb, errors, replies


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--demos", type=Path, help="python/demos of a kachaka-api clone (default: clone --ref)")
    ap.add_argument("--ref", default="v3.17.5", help="the kachaka-api tag to clone (the pinned client's)")
    ap.add_argument("--timeout", type=float, default=120.0, help="s per cell of a run notebook")
    ap.add_argument("--stream-timeout", type=float, default=20.0, help="s before a never-ending cell is stopped")
    ap.add_argument("--out", type=Path, default=REPO / "build" / "demo_notebooks")
    ap.add_argument("--save-map", metavar="NAME", help="save the live SLAM map as two maps and exit")
    ap.add_argument("--localize", action="store_true", help="set the robot's pose on the saved map first")
    ap.add_argument("only", nargs="*", help="notebook names (default: all)")
    args = ap.parse_args()
    if args.save_map:
        return save_map(args.target, args.save_map)
    if args.localize and not localize(args.target):
        return 1
    demos = args.demos
    if demos is None:
        fetch(REPO / "build" / "kachaka-api", args.ref)
        demos = REPO / "build" / "kachaka-api" / "python" / "demos"
    found = sorted(p.stem for p in demos.glob("*.ipynb"))
    unknown = [n for n in found if n not in NOTEBOOKS]
    rows, failed = [], []
    for name, (kind, why) in NOTEBOOKS.items():
        if args.only and name not in args.only:
            continue
        if name not in found:
            rows.append((name, "missing", "not in this kachaka-api"))
            continue
        if kind == "skip":
            rows.append((name, "skipped", why))
            print(f"{name}: skipped ({why})", flush=True)
            continue
        t0 = time.monotonic()
        nb, errors, replies = run(demos / f"{name}.ipynb", kind, args.target, args.timeout, args.stream_timeout,
                                  args.out)
        ok, detail = not errors, "; ".join(errors)
        if ok and name in CHECKS:
            ok, detail = CHECKS[name](nb, replies)
        notes = "; ".join(s for s in (detail, why) if s)
        rows.append((name, "OK" if ok else "FAIL", notes))
        print(f"{name}: {'OK' if ok else 'FAIL'} in {time.monotonic() - t0:.0f} s ({notes})", flush=True)
        if not ok:
            failed.append(name)
    for name in unknown:
        rows.append((name, "unknown", "new notebook: add it to NOTEBOOKS"))
        failed.append(name)
    print("\n| Notebook | Result | Notes |\n|---|---|---|")
    for name, result, notes in rows:
        print(f"| {name} | {result} | {notes} |")
    print("\nALL PASSED" if not failed else f"\nFAILED: {', '.join(failed)}", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
