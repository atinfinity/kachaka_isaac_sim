"""Record the dashboard demo GIF: drive the page in headless Chromium (clicks
on its own buttons) and take screenshots through the DevTools protocol.

Needs the simulator, the ROS 2 side with Nav2 and scripts/dashboard.py
running, and chromium on the host:

    uv run --no-project --with websockets --with pillow python scripts/record_dashboard_demo.py docs/images/dashboard.gif

`--scenario reset` records sim/reset instead (docs/images/sim_reset.gif): the robot
tows a shelf away, then the reset puts both back. It runs tests/sim_reset.py
in the ROS 2 container for that.
"""

import argparse
import asyncio
import base64
import io
import json
import os
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import websockets
from PIL import Image

PORT = 9333
RESET_CMD = ["docker", "run", "--rm", "--network", "host", "--ipc", "host",
             "-e", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304",
             "-v", f"{Path(__file__).resolve().parents[1] / 'tests'}:/tests:ro",
             "kachaka-isaac-sim-ros:jazzy", "python3", "/tests/sim_reset.py"]


class Page:
    def __init__(self, ws):
        self.ws = ws
        self.n = 0

    async def call(self, method, **params):
        self.n += 1
        await self.ws.send(json.dumps({"id": self.n, "method": method, "params": params}))
        while True:
            msg = json.loads(await self.ws.recv())
            if msg.get("id") == self.n:
                return msg.get("result", {})

    async def js(self, expr):
        r = await self.call("Runtime.evaluate", expression=expr, returnByValue=True, awaitPromise=True)
        return r.get("result", {}).get("value")

    async def shot(self) -> Image.Image:
        r = await self.call("Page.captureScreenshot", format="png")
        return Image.open(io.BytesIO(base64.b64decode(r["data"]))).convert("RGB")


def page_target() -> str:
    """The blank tab's DevTools URL (the list also holds Chromium's own pages)."""
    targets = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{PORT}/json/list").read())
    return next(t["webSocketDebuggerUrl"] for t in targets if t["type"] == "page" and t["url"] == "about:blank")


async def record(ws_url: str, url: str, width: int, height: int, fps: float, speed: float,
                 scenario: str = "commands") -> list:
    frames = []
    async with websockets.connect(ws_url, max_size=2**26) as ws:
        page = Page(ws)
        await page.call("Emulation.setDeviceMetricsOverride", width=width, height=height, deviceScaleFactor=1,
                        mobile=False)
        await page.call("Page.navigate", url=url)
        await asyncio.sleep(3.0)

        async def capture_for(seconds: float, until=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                t = time.monotonic()
                frames.append(await page.shot())
                if until and await page.js(until):
                    break
                await asyncio.sleep(max(0.0, speed / fps - (time.monotonic() - t)))

        idle = "document.getElementById('running').textContent === 'idle'"

        async def run(js: str, timeout: float):
            await page.js(js)
            await capture_for(2.0)  # let the command start
            await capture_for(timeout, until=idle)
            await capture_for(2.0)

        await capture_for(2.0)
        if scenario == "reset":
            await page.js("""document.getElementById('shelf').value = 'S01';
                             document.getElementById('shelfDest').value = 'L01';
                             document.querySelector("[data-cmd='move_shelf']").click()""")
            # Until the robot has the shelf and is on its way.
            await capture_for(90.0, until="document.getElementById('shelfPill').textContent.startsWith('Towing')")
            await capture_for(12.0)
            reset = await asyncio.create_subprocess_exec(*RESET_CMD)
            await capture_for(10.0)
            await reset.wait()
            await capture_for(3.0)
            return frames
        await run("""document.getElementById('shelf').value = 'S01';
                     document.getElementById('shelfDest').value = 'L01';
                     document.querySelector("[data-cmd='move_shelf']").click()""", 120)
        await run("document.querySelector(\"[data-cmd='return_shelf']\").click()", 120)
        await run("document.querySelector(\"[data-cmd='return_home']\").click()", 120)
    return frames


def encode(frames: list, out: Path, fps: float) -> None:
    print(f"{len(frames)} frames", flush=True)
    tmp = Path(tempfile.mkdtemp())
    try:
        for i, f in enumerate(frames):
            f.save(tmp / f"f_{i:05d}.png")
        enc = ["-filter_complex", ("split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];"
                                   "[b][p]paletteuse=dither=none:diff_mode=rectangle")]
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(fps),
                        "-i", str(tmp / "f_%05d.png"), *enc, str(out)], check=True)
    finally:
        shutil.rmtree(tmp)
    print(f"{out}: {out.stat().st_size / 1e6:.1f} MB")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--url", default="http://127.0.0.1:8080/")
    ap.add_argument("--size", type=int, nargs=2, default=(1100, 640))
    ap.add_argument("--fps", type=float, default=5.0, help="GIF frame rate")
    ap.add_argument("--speed", type=float, default=4.0, help="playback speed-up")
    ap.add_argument("--scenario", choices=("commands", "reset"), default="commands")
    ap.add_argument("--chromium", default=shutil.which("chromium") or "chromium")
    args = ap.parse_args()
    # The snap Chromium can only write below $HOME.
    profile = Path(os.environ["HOME"]) / "snap/chromium/common/dashboard-demo-profile"
    browser = subprocess.Popen([args.chromium, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                                f"--remote-debugging-port={PORT}", f"--user-data-dir={profile}", "about:blank"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(50):
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{PORT}/json/version", timeout=1)
                break
            except OSError:
                time.sleep(0.2)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        frames = asyncio.run(record(page_target(), args.url, *args.size, args.fps, args.speed,
                                    args.scenario))
        encode(frames, args.out, args.fps)
    finally:
        browser.terminate()
        browser.wait(timeout=10)


if __name__ == "__main__":
    main()
