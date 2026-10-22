"""Drive the kachaka-api demo notebooks in JupyterLab for the demo videos
(scripts/record_demo.sh notebooks <set>): JupyterLab runs on the host,
headless Chromium shows it, and the notebooks run cell by cell against the
simulator, their widgets clicked like by hand.

  sensors  get_front_camera_continuous, get_object_detection, get_laser_scan,
           get_imu, undistort, qrcode, feature_matching; the recorder turns
           the robot meanwhile (velocity commands).
  control  teleop and command_gui (their buttons clicked), speak,
           follow_path, plot_map_robot_lidar (two clicks on the plot send
           the robot there: the notebook's own click handler).
  map      robot_pose, switch_map, error_code, http_server,
           url_kachaka_api (a box on the page shows what the server answers);
           needs Nav2 on two saved maps (record_demo.sh saves them).

Writes into OUT: rviz.mp4 and rviz_start.txt (the browser, at 10 fps: the
right-hand panel of scripts/make_demo_gif.py) and captions.csv. Needs the
simulator, Nav2 and chromium on the host:

    uv run --no-project --with kachaka-api==3.17.5.0 --with jupyterlab --with ipympl ... \\
        python scripts/record_notebook_demo.py build/notebooks_control control
"""

import argparse
import ast
import asyncio
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from pathlib import Path

import websockets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
from demo_notebooks_check import POINT_AT, fetch, localize
from record_dashboard_demo import PORT, Page, page_target

LAB_PORT = 26510
SERVER = "http://127.0.0.1:26502"  # where http_server and url_kachaka_api serve
SIZE = (960, 720)
FPS = 10
CLICKS = [(1.6, -0.3), (-1.8, 1.3)]  # map points to click on the plot (ros_nav_check.py's room goals)
SETS = {
    "sensors": ["get_front_camera_continuous", "get_object_detection", "get_laser_scan", "get_imu", "undistort",
                "qrcode", "feature_matching"],
    "control": ["teleop", "command_gui", "speak", "follow_path", "plot_map_robot_lidar"],
    "map": ["robot_pose", "switch_map", "error_code", "http_server", "url_kachaka_api"],
}

# The pixel on the ipympl canvas of a map point, as fractions of the figure
# (from the top left); run in the plot notebook's kernel.
TO_FIGURE = """\
(lambda p: (lambda d: [float(d[0] / fig.bbox.width), float(1 - d[1] / fig.bbox.height)])(
    ax.transData.transform(map_image_2d_geometry.calculate_robot_pose_matrix_in_pixel(p)[:2, 2])))(
    __import__("kachaka_api").generated.kachaka_api_pb2.Pose(x={x}, y={y}, theta=0.0))"""


class Lab:
    """JupyterLab in the page, through window.jupyterapp. Every use of the
    page goes through the lock: the screenshots share it."""

    def __init__(self, page: Page, lock: asyncio.Lock):
        self.page, self.lock = page, lock

    async def js(self, expr):
        async with self.lock:
            return await self.page.js(expr)

    async def wait(self, expr, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if await self.js(expr):
                return True
            await asyncio.sleep(0.2)
        raise TimeoutError(expr)

    async def open(self, name):
        await self.js(f"jupyterapp.commands.execute('docmanager:open', {{path: '{name}'}}).then(() => true)")
        await self.wait(f"(w => w && w.sessionContext.session?.kernel?.status === 'idle' && "
                        "(jupyterapp.shell.activateById(w.id), true))"
                        f"(Array.from(jupyterapp.shell.widgets('main')).find(w => w.context?.path === '{name}'))")
        await self.wait(f"jupyterapp.shell.currentWidget?.context.path === '{name}'")
        await asyncio.sleep(1.0)

    async def run_cells(self, skip=(), pause=1.0, last_for=None, before_last=None):
        """Run the code cells one by one, each to the end, but those with any
        of `skip` in them. With `last_for`, the last one runs that long and
        is left running (a stream loop or a server); `before_last` is awaited
        just before it starts."""
        sources = await self.js("jupyterapp.shell.currentWidget.content.widgets"
                                ".map(c => c.model.type === 'code' ? c.model.sharedModel.getSource() : null)")
        todo = [i for i, s in enumerate(sources) if s and s.strip() and not any(k in s for k in skip)]
        for i in todo:
            await self.js(f"(n => {{ n.activeCellIndex = {i}; n.scrollToItem({i}); return true; }})"
                          "(jupyterapp.shell.currentWidget.content)")
            last = i == todo[-1] and last_for is not None
            if last and before_last:
                await before_last()
            # Not awaited: the last cell may never end, and the rest are followed below.
            await self.js("(jupyterapp.commands.execute('notebook:run-cell'), true)")
            # Keep the cell's output in view: under a long cell it is off-screen.
            follow = (f"(n => {{ const c = n.widgets[{i}];"
                      " if (c.model.outputs.length) c.outputArea.node.scrollIntoView({block: 'end'});"
                      " return jupyterapp.shell.currentWidget.sessionContext.session.kernel.status; })"
                      "(jupyterapp.shell.currentWidget.content)")
            end = time.monotonic() + (last_for if last else 120.0)
            await asyncio.sleep(0.5)
            while time.monotonic() < end:
                if await self.js(follow) == "idle" and not last:
                    break
                await asyncio.sleep(0.3)
            else:
                if not last:
                    raise TimeoutError(f"cell {i}")
            if last:
                return
            await asyncio.sleep(pause)

    async def evaluate(self, expr):
        """`expr` in the current notebook's kernel, without touching the notebook."""
        js = ("(async () => { const k = jupyterapp.shell.currentWidget.sessionContext.session.kernel;"
              f" const r = await k.requestExecute({{code: '', silent: true, store_history: false,"
              f" user_expressions: {{r: {json.dumps(expr)}}}}}).done;"
              " return r.content.user_expressions.r.data['text/plain']; })()")
        return ast.literal_eval(await self.js(js))

    async def close(self):
        await self.js("(w => w.sessionContext.shutdown().then(() => { w.dispose(); return true; }))"
                      "(jupyterapp.shell.currentWidget)")
        await asyncio.sleep(0.5)

    async def click_xy(self, x, y):
        async with self.lock:
            for kind, buttons in (("mouseMoved", 0), ("mousePressed", 1), ("mouseReleased", 0)):
                await self.page.call("Input.dispatchMouseEvent", type=kind, x=x, y=y, button="left",
                                     buttons=buttons, clickCount=1)

    async def click(self, element):
        """Click the middle of `element` (JS for one), scrolled into view."""
        r = await self.js(f"(e => {{ e.scrollIntoView({{block: 'nearest'}}); const r = e.getBoundingClientRect();"
                          f" return [r.left + r.width / 2, r.top + r.height / 2]; }})({element})")
        await self.click_xy(*r)

    async def type_into(self, element, text):
        """Replace the text of an input (JS for it) as if typed, then Enter."""
        await self.click(element)
        await self.js(f"({element}).select() || true")
        async with self.lock:
            await self.page.call("Input.insertText", text=text)
            for kind in ("keyDown", "keyUp"):
                await self.page.call("Input.dispatchKeyEvent", type=kind, key="Enter", code="Enter",
                                     windowsVirtualKeyCode=13)
        await asyncio.sleep(0.3)

    async def show_box(self, title, url):
        """A box on the page with `url` in it, like a second browser window."""
        await self.js(f"""(() => {{
            let b = document.getElementById('demo-box');
            if (!b) {{
                b = document.createElement('div'); b.id = 'demo-box';
                b.style.cssText = 'position:fixed;right:16px;bottom:40px;width:460px;height:330px;z-index:10000;'
                    + 'background:#fff;border:2px solid #555;box-shadow:0 4px 16px rgba(0,0,0,.4);'
                    + 'display:flex;flex-direction:column;font:13px sans-serif';
                b.innerHTML = '<div style="background:#555;color:#fff;padding:4px 8px"></div>'
                    + '<iframe style="flex:1;border:0;background:#fff"></iframe>';
                document.body.appendChild(b);
            }}
            b.firstChild.textContent = {json.dumps(title)};
            b.lastChild.src = {json.dumps(url)};
            return true; }})()""")

    async def hide_box(self):
        await self.js("(b => (b && b.remove(), true))(document.getElementById('demo-box'))")


def button(text):
    """JS for the widget button labelled `text` in the current notebook."""
    return (f"Array.from(jupyterapp.shell.currentWidget.node.querySelectorAll('button'))"
            f".find(b => b.textContent.trim() === {json.dumps(text)})")


class Driver:
    """Velocity commands at 5 Hz from a thread (the simulator stops the robot
    0.5 s after the last one): `pattern(t)` gives (linear, angular) at time t."""

    def __init__(self, target):
        from kachaka_api import KachakaApiClient
        self.client = KachakaApiClient(target)
        self.pattern = None
        self.client.set_manual_control_enabled(True)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        t0 = time.monotonic()
        while True:
            pattern = self.pattern
            if pattern:
                linear, angular = pattern(time.monotonic() - t0)
                try:
                    self.client.set_robot_velocity(linear=linear, angular=angular)
                except Exception as e:  # noqa: BLE001 (keep driving)
                    print(f"velocity: {e}", flush=True)
            time.sleep(0.2)

    def drive(self, pattern=None):
        self.pattern = pattern


def turn(t):
    return 0.0, 0.35


def turn_slowly(t):
    return 0.0, 0.15


def shuttle(t):
    return (0.25 if t % 3.0 < 1.5 else -0.25), 0.0


async def sensors(lab, say, target):
    drive = Driver(target)

    await lab.open("get_front_camera_continuous.ipynb")
    say("get_front_camera_continuous: the camera stream (the recorder turns the robot)")
    drive.drive(turn)
    await lab.run_cells(last_for=10.0)
    await lab.close()

    await lab.open("get_object_detection.ipynb")
    say("get_object_detection: boxes, labels and scores on the camera image")
    await lab.run_cells(last_for=16.0)
    await lab.close()

    await lab.open("get_laser_scan.ipynb")
    say("get_laser_scan: the LiDAR scan, live")
    # The second cell is the same loop with a LaserScanActivator around it.
    await lab.run_cells(skip=("LaserScanActivator",), last_for=10.0)
    await lab.close()

    await lab.open("get_imu.ipynb")
    say("get_imu: IMU acceleration x (the recorder drives forward and back)")
    drive.drive(shuttle)
    await lab.run_cells(pause=2.0)
    drive.drive(None)
    await lab.close()

    await lab.open("undistort.ipynb")
    say("undistort: the camera image, then undistorted with its camera info")
    await lab.run_cells(pause=1.0)
    await lab.js("(n => (n.scrollToItem(n.widgets.length - 1), true))(jupyterapp.shell.currentWidget.content)")
    await asyncio.sleep(3.0)
    await lab.close()

    await lab.open("qrcode.ipynb")
    say("qrcode: QR code detection on the stream (there is no QR code in this room)")
    drive.drive(turn)
    await lab.run_cells(last_for=6.0)
    drive.drive(None)
    await lab.close()

    await lab.open("feature_matching.ipynb")
    say("feature_matching: a 10 s countdown, then the box is captured")

    async def start_turning():
        say("feature_matching: ORB matches against the capture while the robot turns")
        drive.drive(turn_slowly)
    await lab.run_cells(last_for=15.0, before_last=start_turning)
    drive.drive(None)


async def control(lab, say, target):
    await lab.open("teleop.ipynb")
    say("teleop: the buttons send velocity commands (clicked repeatedly: each lasts 0.5 s)")
    await lab.run_cells()
    await lab.js("jupyterapp.shell.currentWidget.node.querySelector('.widget-gridbox')"
                 ".scrollIntoView({block: 'center'}) || true")
    await asyncio.sleep(1.0)
    for label, seconds in (("Forward", 2.5), ("Turn left", 3.0), ("Backward", 2.0), ("Stop", 0.0)):
        say(f"teleop: {label}")
        end = time.monotonic() + seconds
        while True:
            await lab.click(button(label))
            if time.monotonic() > end:
                break
            await asyncio.sleep(0.25)
        await asyncio.sleep(0.5)
    await asyncio.sleep(1.0)
    await lab.close()

    await lab.open("command_gui.ipynb")
    say("command_gui: a form for the robot's commands")
    await lab.run_cells()
    radio = ("Array.from(jupyterapp.shell.currentWidget.node.querySelectorAll('.widget-radio-box label'))"
             ".find(l => l.textContent.trim() === {!r})")
    field = ("Array.from(jupyterapp.shell.currentWidget.node.querySelectorAll('.widget-text'))"
             ".find(w => w.offsetParent && w.querySelector('label')?.textContent.trim() === {!r})"
             ".querySelector('input')")
    say("command_gui: MoveToPose to (1.6, -0.3)")
    await lab.click(radio.format("MoveToPose"))
    await asyncio.sleep(0.5)
    await lab.type_into(field.format("X:"), "1.6")
    await lab.type_into(field.format("Y:"), "-0.3")
    await lab.click(button("Send"))
    await asyncio.sleep(16.0)
    say("command_gui: ReturnHome")
    await lab.click(radio.format("ReturnHome"))
    await asyncio.sleep(0.5)
    await lab.click(button("Send"))
    await asyncio.sleep(22.0)
    await lab.close()

    await lab.open("speak.ipynb")
    say("speak: a phrase to say (the simulator has no speaker: the command just succeeds)")
    await lab.run_cells(pause=2.0)
    await lab.close()

    await lab.open("follow_path.ipynb")
    say("follow_path: velocity commands along an ellipse (30 s)")
    await lab.run_cells()
    await lab.close()

    await lab.open("plot_map_robot_lidar.ipynb")
    say("plot_map_robot_lidar: map, robot and scan, live")
    # The first cell installs JupyterLab extensions (ipympl from pip has them
    # already), and the last one closes the figure, which then takes no
    # clicks. A smaller figure fits the page.
    await lab.evaluate("__import__('matplotlib').rcParams.update({'figure.dpi': 50}) or 0")
    await lab.run_cells(skip=("labextension", "plt.close()"), pause=0.5)
    await lab.js("document.querySelector('.jupyter-matplotlib-canvas-div').scrollIntoView({block: 'center'}) || true")
    await asyncio.sleep(3.0)
    for x, y in CLICKS:
        fx, fy = await lab.evaluate(TO_FIGURE.format(x=x, y=y))
        r = await lab.js("(r => [r.left, r.top, r.width, r.height])(document.querySelector("
                         "'.jupyter-matplotlib-canvas-div canvas:last-of-type').getBoundingClientRect())")
        px, py = r[0] + fx * r[2], r[1] + fy * r[3]
        say(f"plot_map_robot_lidar: a click on the plot sends the robot there ({x}, {y})")
        await lab.click_xy(px, py)
        await asyncio.sleep(1.0)
        if not await lab.evaluate("last_target is not None"):
            raise RuntimeError(f"the click at ({px:.0f}, {py:.0f}) did not reach the plot")
        await asyncio.sleep(18.0)


async def map_and_pose(lab, say, target):
    await lab.open("robot_pose.ipynb")
    say("robot_pose: get_robot_pose, then set_robot_pose to (0, 1) and (2, 1)")
    await lab.run_cells(pause=2.0)
    await lab.close()

    await lab.open("switch_map.ipynb")
    say("switch_map: the two saved maps; switch to the other, then back with a pose")
    await lab.run_cells(pause=3.0)
    await lab.close()

    await lab.open("error_code.ipynb")
    say("error_code: the error code table, then the error stream")
    await lab.run_cells(last_for=6.0)
    await lab.close()

    await lab.open("http_server.ipynb")
    say("http_server: a FastAPI server in the notebook; GET /run makes the robot speak")
    await lab.run_cells(last_for=4.0)
    await lab.show_box(f"GET {SERVER}/run", f"{SERVER}/run")
    await asyncio.sleep(5.0)
    await lab.hide_box()
    await lab.close()

    await lab.open("url_kachaka_api.ipynb")
    say("url_kachaka_api: every API method as a URL")
    await lab.run_cells(last_for=5.0)
    for path in ("get_robot_pose", "front_camera_image.jpeg", "get_map_list"):
        say(f"url_kachaka_api: GET /kachaka/{path}")
        await lab.show_box(f"GET {SERVER}/kachaka/{path}", f"{SERVER}/kachaka/{path}")
        await asyncio.sleep(5.0)


async def record(ws_url, url, out: Path, scenario, target):
    frames, captions = [], []
    lock = asyncio.Lock()
    async with websockets.connect(ws_url, max_size=2**26) as ws:
        page = Page(ws)
        await page.call("Emulation.setDeviceMetricsOverride", width=SIZE[0], height=SIZE[1], deviceScaleFactor=1,
                        mobile=False)
        # Headless, the page never has the focus, and JupyterLab then has no current notebook.
        await page.call("Emulation.setFocusEmulationEnabled", enabled=True)
        await page.call("Page.navigate", url=url)
        lab = Lab(page, lock)
        await lab.wait("typeof jupyterapp !== 'undefined' && jupyterapp.shell !== undefined", timeout=60.0)
        await lab.js("jupyterapp.restored.then(() => true)")
        await lab.js("jupyterapp.commands.execute('application:set-mode', {mode: 'single-document'})"
                     ".then(() => { jupyterapp.shell.collapseLeft(); jupyterapp.shell.collapseRight(); return true; })")

        def say(text):
            captions.append((time.time(), text))
            print(text, flush=True)

        stop = asyncio.Event()

        async def capture():
            while not stop.is_set():
                t = time.time()
                async with lock:
                    frames.append((t, await page.shot()))
                await asyncio.sleep(max(0.0, 1 / FPS - (time.time() - t)))

        start = time.time()
        task = asyncio.create_task(capture())
        try:
            await scenario(lab, say, target)  # the last notebook stays open to the end
        finally:
            stop.set()
            await task
            if frames:
                frames[-1][1].save(out / "last.png")  # where it got to, if it failed
    return start, frames, captions


def write_video(start, frames, out: Path):
    """The frames on a 10 fps clock (the latest one at each tick) as rviz.mp4."""
    tmp = Path(tempfile.mkdtemp())
    try:
        n = int((frames[-1][0] - start) * FPS)
        k = 0
        for i in range(n):
            t = start + i / FPS
            while k + 1 < len(frames) and frames[k + 1][0] <= t:
                k += 1
            frames[k][1].save(tmp / f"f_{i:06d}.png")
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(FPS),
                        "-i", str(tmp / "f_%06d.png"), "-c:v", "libx264", "-preset", "ultrafast",
                        "-pix_fmt", "yuv420p", str(out / "rviz.mp4")], check=True)
    finally:
        shutil.rmtree(tmp)
    (out / "rviz_start.txt").write_text(f"{start}\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path)
    ap.add_argument("set", choices=list(SETS))
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--ref", default="v3.17.5", help="kachaka-api tag the notebooks come from")
    ap.add_argument("--chromium", default=shutil.which("chromium") or "chromium")
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[1]
    fetch(repo / "build" / "kachaka-api", args.ref)
    scenario = {"sensors": sensors, "control": control, "map": map_and_pose}[args.set]
    if args.set == "map" and not localize(args.target):  # AMCL on the saved map
        sys.exit(1)

    # The notebooks as they are; an IPython startup file points their clients
    # at the simulator, and JupyterLab gets settings of its own (no news).
    work = Path(tempfile.mkdtemp())
    nb_dir = work / "notebooks"
    nb_dir.mkdir()
    for name in SETS[args.set]:
        shutil.copy(repo / "build/kachaka-api/python/demos" / f"{name}.ipynb", nb_dir)
    startup = work / "ipython/profile_default/startup"
    startup.mkdir(parents=True)
    (startup / "00-simulator.py").write_text(POINT_AT.format(target=args.target))
    settings = work / "lab-settings/@jupyterlab/apputils-extension"
    settings.mkdir(parents=True)
    (settings / "notification.jupyterlab-settings").write_text(
        json.dumps({"fetchNews": "false", "checkForUpdates": False}))
    env = dict(os.environ, IPYTHONDIR=str(work / "ipython"), JUPYTERLAB_SETTINGS_DIR=str(work / "lab-settings"),
               JUPYTERLAB_WORKSPACES_DIR=str(work / "workspaces"))
    lab_log = open(out / "jupyterlab.log", "w")  # noqa: SIM115 (open while JupyterLab runs)
    lab = subprocess.Popen([sys.executable, "-m", "jupyterlab", "--no-browser", f"--port={LAB_PORT}",
                            "--ServerApp.token=", "--ServerApp.password=", "--LabApp.expose_app_in_browser=True",
                            f"--ServerApp.root_dir={nb_dir}"], env=env, stdout=subprocess.DEVNULL,
                           stderr=lab_log)
    profile = Path(os.environ["HOME"]) / "snap/chromium/common/notebook-demo-profile"
    browser = subprocess.Popen([args.chromium, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                                f"--remote-debugging-port={PORT}", f"--user-data-dir={profile}", "about:blank"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for url in (f"http://127.0.0.1:{PORT}/json/version", f"http://127.0.0.1:{LAB_PORT}/api"):
            for _ in range(150):
                try:
                    urllib.request.urlopen(url, timeout=1)
                    break
                except OSError:
                    time.sleep(0.2)
        start, frames, captions = asyncio.run(record(page_target(), f"http://127.0.0.1:{LAB_PORT}/lab", out,
                                                     scenario, args.target))
        write_video(start, frames, out)
        with open(out / "captions.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["wall_time", "text"])
            w.writerows(captions)
    finally:
        browser.terminate()
        lab.terminate()
        browser.wait(timeout=10)
        lab.wait(timeout=10)
        lab_log.close()
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
