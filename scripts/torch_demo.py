"""Torch light demo for scripts/record_torch_demo.sh: switch the torches
through kachaka-api in a dark room and turn the robot so the beams sweep the
walls, while saving both camera images.

    uv run --no-project --with kachaka-api==3.17.5.0 --with numpy --with pillow \
        python scripts/torch_demo.py build/torch
    .venv/bin/python scripts/torch_demo.py build/torch --compose docs/images/torch.gif

Run writes OUT/cams/ (camera frames + frames.csv) and OUT/captions.csv;
--compose puts the simulator's view (OUT/sim) next to the two cameras.
"""

import argparse
import bisect
import csv
import math
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FPS = 10


def run(out: Path, target: str) -> None:
    import numpy as np
    from kachaka_api import KachakaApiClient
    from kachaka_api.generated import kachaka_api_pb2 as pb2

    client = KachakaApiClient(target)
    cams = out / "cams"
    cams.mkdir(parents=True, exist_ok=True)
    captions = out / "captions.csv"
    with open(captions, "w", newline="") as f:
        csv.writer(f).writerow(["wall_time", "text"])

    def say(text: str) -> None:
        print(text, flush=True)
        with open(captions, "a", newline="") as f:
            csv.writer(f).writerow([f"{time.time():.3f}", text])

    stop = threading.Event()

    def grab() -> None:
        # The camera RPCs are slow; grab the two cameras in a loop.
        with open(cams / "frames.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["wall_time", "front", "back"])
            i = 0
            while not stop.is_set():
                t = time.time()
                names = []
                for cam in ("front", "back"):
                    img = getattr(client, f"get_{cam}_camera_ros_image")()
                    a = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
                    name = f"{cam}_{i:05d}.jpg"
                    Image.fromarray(a).save(cams / name, quality=90)
                    names.append(name)
                w.writerow([f"{t:.3f}", *names])
                f.flush()
                i += 1

    def torch(cam: str, level: int) -> None:
        rpc = "Front" if cam == "front" else "Back"
        req = getattr(pb2, f"Set{rpc}TorchIntensityRequest")(intensity=level)
        getattr(client.stub, f"Set{rpc}TorchIntensity")(req)

    def both(front: int, back: int, text: str, hold: float) -> None:
        say(text)
        torch("front", front)
        torch("back", back)
        time.sleep(hold)

    def turn(angle: float, angular: float = 0.25) -> None:
        # SetRobotVelocity is a fraction of full scale (1.57 rad/s); resend
        # it faster than the 0.5 s command timeout.
        end = time.monotonic() + abs(angle) / (angular * 1.57)
        while time.monotonic() < end:
            client.set_robot_velocity(0.0, math.copysign(angular, angle))
            time.sleep(0.1)
        client.set_robot_velocity(0.0, 0.0)

    both(0, 0, "Dark room (--dome-light 0), torches off", 0.0)
    thread = threading.Thread(target=grab, daemon=True)
    thread.start()
    time.sleep(3.0)
    both(255, 0, "SetFrontTorchIntensity(intensity=255)", 4.0)
    both(0, 255, "SetFrontTorchIntensity(0), SetBackTorchIntensity(255)", 4.0)
    both(64, 64, "Both torches at 64", 3.0)
    both(255, 255, "Both torches at 255", 2.0)
    client.set_manual_control_enabled(True)
    say("Both torches at 255, turning in place: the beams sweep the walls")
    turn(2 * math.pi)
    time.sleep(1.0)
    both(0, 0, "Torches off (intensity=0)", 3.0)
    client.set_manual_control_enabled(False)
    stop.set()
    thread.join()
    (out / "end.txt").write_text(f"{time.time():.3f}\n")


def _font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans-Bold.ttf", size)
    except OSError:
        return ImageFont.load_default()


def _label(img: Image.Image, text: str, size: int = 18) -> None:
    d = ImageDraw.Draw(img)
    x0, y0, x1, y1 = d.textbbox((10, 8), text, font=_font(size))
    d.rectangle((x0 - 6, y0 - 4, x1 + 6, y1 + 4), fill=(0, 0, 0))
    d.text((10, 8), text, fill=(255, 255, 255), font=_font(size))


def compose(out: Path, dest: Path, height: int = 540, speed: float = 1.0) -> None:
    def rows(path: Path):
        with open(path) as f:
            r = list(csv.DictReader(f))
        return [float(x["wall_time"]) for x in r], r

    sim_t, sim = rows(out / "sim/frames.csv")
    cam_t, cam = rows(out / "cams/frames.csv")
    cap_t, cap = rows(out / "captions.csv")
    t0, t1 = cam_t[0], float((out / "end.txt").read_text())
    half = height // 2
    tmp = Path(tempfile.mkdtemp())
    try:
        n = int((t1 - t0) * FPS / speed)
        for i in range(n):
            t = t0 + i * speed / FPS
            s = Image.open(out / "sim" / sim[min(bisect.bisect_left(sim_t, t), len(sim) - 1)]["file"]).convert("RGB")
            s = s.resize((s.width * height // s.height, height), Image.LANCZOS)
            _label(s, "Isaac Sim (dark room)")
            c = cam[max(bisect.bisect_right(cam_t, t) - 1, 0)]
            frame = Image.new("RGB", (s.width + half * 4 // 3, height))
            frame.paste(s, (0, 0))
            for k, name in enumerate(("front", "back")):
                im = Image.open(out / "cams" / c[name]).convert("RGB").resize((half * 4 // 3, half), Image.LANCZOS)
                _label(im, f"{name}_camera", 16)
                frame.paste(im, (s.width, k * half))
            j = bisect.bisect_right(cap_t, t) - 1
            if j >= 0:
                d = ImageDraw.Draw(frame)
                d.rectangle((0, height - 36, frame.width, height), fill=(0, 0, 0))
                d.text((12, height - 29), cap[j]["text"], fill=(255, 220, 120), font=_font(20))
            frame.save(tmp / f"out_{i:06d}.png")
        dest.parent.mkdir(parents=True, exist_ok=True)
        if dest.suffix == ".mp4":
            encode = ["-c:v", "libx264", "-crf", "28", "-preset", "slow", "-pix_fmt", "yuv420p",
                      "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-movflags", "+faststart"]
        else:
            encode = ["-filter_complex", ("split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];"
                                          "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle")]
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(FPS),
                        "-i", str(tmp / "out_%06d.png"), *encode, str(dest)], check=True)
        print(f"{dest}: {n} frames, {dest.stat().st_size / 1e6:.1f} MB")
    finally:
        shutil.rmtree(tmp)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--compose", type=Path, metavar="GIF", help="compose OUT into GIF (or .mp4) instead")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--height", type=int, default=540, help="output height [px]")
    args = ap.parse_args()
    if args.compose:
        compose(args.out, args.compose, height=args.height, speed=args.speed)
    else:
        run(args.out, args.target)


if __name__ == "__main__":
    main()
