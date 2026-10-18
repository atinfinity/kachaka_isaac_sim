"""Compose the demo GIF from scripts/record_demo.sh output.

Left: simulator bird's-eye frames (frames.csv gives wall times).
Right: the RViz screen capture (starts at rviz_start.txt, 10 fps).

    .venv/bin/python scripts/make_demo_gif.py build/demo docs/demo.gif [--speed 2]
"""

import argparse
import bisect
import csv
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HEIGHT = 360
FPS = 10


def label(img: Image.Image, text: str) -> None:
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    x0, y0, x1, y1 = d.textbbox((10, 8), text, font=font)
    d.rectangle((x0 - 6, y0 - 4, x1 + 6, y1 + 4), fill=(0, 0, 0))
    d.text((10, 8), text, fill=(255, 255, 255), font=font)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("demo_dir", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--speed", type=float, default=2.0, help="playback speed-up")
    p.add_argument("--rviz-crop", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
                   help="keep only this part of the RViz capture (its 3D view)")
    args = p.parse_args()
    d = args.demo_dir

    with open(d / "sim/frames.csv") as f:
        rows = list(csv.DictReader(f))
    sim_t = [float(r["wall_time"]) for r in rows]
    t0 = float((d / "rviz_start.txt").read_text())
    t1 = float((d / "end.txt").read_text())

    tmp = Path(tempfile.mkdtemp())
    try:
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", str(d / "rviz.mp4"),
                        "-vf", f"fps={FPS}", str(tmp / "rviz_%06d.png")], check=True)
        rviz = sorted(tmp.glob("rviz_*.png"))
        n_out = int((t1 - t0) * FPS / args.speed)
        for i in range(n_out):
            t = t0 + i * args.speed / FPS
            k = min(max(bisect.bisect_left(sim_t, t), 0), len(rows) - 1)
            sim = Image.open(d / "sim" / rows[k]["file"]).convert("RGB")
            sim = sim.resize((sim.width * HEIGHT // sim.height, HEIGHT), Image.LANCZOS)
            rv = Image.open(rviz[min(int((t - t0) * FPS), len(rviz) - 1)]).convert("RGB")
            if args.rviz_crop:
                x, y, w, h = args.rviz_crop
                rv = rv.crop((x, y, x + w, y + h))
            rv = rv.resize((rv.width * HEIGHT // rv.height, HEIGHT), Image.LANCZOS)
            label(sim, "Isaac Sim")
            label(rv, "RViz: slam_toolbox + Nav2")
            frame = Image.new("RGB", (sim.width + rv.width, HEIGHT))
            frame.paste(sim, (0, 0))
            frame.paste(rv, (sim.width, 0))
            frame.save(tmp / f"out_{i:06d}.png")
        args.out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(FPS),
            "-i", str(tmp / "out_%06d.png"), "-filter_complex",
            "split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];"
            "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
            str(args.out)], check=True)
        print(f"{args.out}: {n_out} frames, {args.out.stat().st_size / 1e6:.1f} MB")
    finally:
        shutil.rmtree(tmp)


if __name__ == "__main__":
    main()
