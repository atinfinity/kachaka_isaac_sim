"""Compose the demo GIF (or MP4) from scripts/record_demo.sh output.

Left: simulator bird's-eye frames (frames.csv gives wall times).
Right: the RViz screen capture (starts at rviz_start.txt, 10 fps).
With --left-video, the left is a second capture instead (e.g. RViz, with a
browser on the right), starting at <name>_start.txt.
If the demo folder has captions.csv (wall_time,text), the latest caption is
shown along the bottom.

    .venv/bin/python scripts/make_demo_gif.py build/demo docs/demo.gif [--speed 2]
    .venv/bin/python scripts/make_demo_gif.py build/scene_office build/scene_office.mp4 --height 540
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


def _font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans-Bold.ttf", size)
    except OSError:
        return ImageFont.load_default()


def label(img: Image.Image, text: str) -> None:
    d = ImageDraw.Draw(img)
    font = _font(18)
    x0, y0, x1, y1 = d.textbbox((10, 8), text, font=font)
    d.rectangle((x0 - 6, y0 - 4, x1 + 6, y1 + 4), fill=(0, 0, 0))
    d.text((10, 8), text, fill=(255, 255, 255), font=font)


def caption(img: Image.Image, text: str) -> None:
    """A full-width bar along the bottom."""
    d = ImageDraw.Draw(img)
    font = _font(20)
    h = 36
    d.rectangle((0, img.height - h, img.width, img.height), fill=(0, 0, 0))
    d.text((12, img.height - h + 7), text, fill=(255, 220, 120), font=font)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("demo_dir", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--speed", type=float, default=2.0, help="playback speed-up")
    p.add_argument("--rviz-crop", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
                   help="keep only this part of the RViz capture (its 3D view)")
    p.add_argument("--rviz-label", default="RViz: slam_toolbox + Nav2")
    p.add_argument("--height", type=int, default=HEIGHT, help="output height [px]")
    p.add_argument("--mark-robot", type=int, default=0, metavar="R",
                   help="ring of radius R px around the image centre (where a following camera keeps the robot)")
    p.add_argument("--left-video", metavar="NAME",
                   help="left: NAME.mp4 in the demo folder (from NAME_start.txt), not the simulator")
    p.add_argument("--left-crop", type=int, nargs=4, metavar=("X", "Y", "W", "H"))
    p.add_argument("--left-label", default="Isaac Sim")
    args = p.parse_args()
    d = args.demo_dir

    if not args.left_video:
        with open(d / "sim/frames.csv") as f:
            rows = list(csv.DictReader(f))
        sim_t = [float(r["wall_time"]) for r in rows]
    t0 = float((d / "rviz_start.txt").read_text())
    t1 = float((d / "end.txt").read_text())
    captions = []
    if (d / "captions.csv").exists():
        with open(d / "captions.csv") as f:
            captions = [(float(r["wall_time"]), r["text"]) for r in csv.DictReader(f)]
    cap_t = [c[0] for c in captions]

    tmp = Path(tempfile.mkdtemp())
    try:
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", str(d / "rviz.mp4"),
                        "-vf", f"fps={FPS}", str(tmp / "rviz_%06d.png")], check=True)
        rviz = sorted(tmp.glob("rviz_*.png"))
        t1 = min(t1, t0 + len(rviz) / FPS)  # not past the capture's last frame
        if args.left_video:
            subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", str(d / f"{args.left_video}.mp4"),
                            "-vf", f"fps={FPS}", str(tmp / "left_%06d.png")], check=True)
            left = sorted(tmp.glob("left_*.png"))
            left_t0 = float((d / f"{args.left_video}_start.txt").read_text())
        n_out = int((t1 - t0) * FPS / args.speed)
        for i in range(n_out):
            t = t0 + i * args.speed / FPS
            if args.left_video:
                sim = Image.open(left[min(max(int((t - left_t0) * FPS), 0), len(left) - 1)]).convert("RGB")
                if args.left_crop:
                    x, y, w, h = args.left_crop
                    sim = sim.crop((x, y, x + w, y + h))
            else:
                k = min(max(bisect.bisect_left(sim_t, t), 0), len(rows) - 1)
                sim = Image.open(d / "sim" / rows[k]["file"]).convert("RGB")
            if args.mark_robot:
                cx, cy, r = sim.width / 2, sim.height / 2, args.mark_robot
                ImageDraw.Draw(sim).ellipse((cx - r, cy - r, cx + r, cy + r), outline=(255, 60, 60), width=3)
            sim = sim.resize((sim.width * args.height // sim.height, args.height), Image.LANCZOS)
            rv = Image.open(rviz[min(int((t - t0) * FPS), len(rviz) - 1)]).convert("RGB")
            if args.rviz_crop:
                x, y, w, h = args.rviz_crop
                rv = rv.crop((x, y, x + w, y + h))
            rv = rv.resize((rv.width * args.height // rv.height, args.height), Image.LANCZOS)
            label(sim, args.left_label)
            label(rv, args.rviz_label)
            frame = Image.new("RGB", (sim.width + rv.width, args.height))
            frame.paste(sim, (0, 0))
            frame.paste(rv, (sim.width, 0))
            j = bisect.bisect_right(cap_t, t) - 1
            if j >= 0 and captions[j][1]:
                caption(frame, captions[j][1])
            frame.save(tmp / f"out_{i:06d}.png")
        args.out.parent.mkdir(parents=True, exist_ok=True)
        if args.out.suffix == ".mp4":
            encode = ["-c:v", "libx264", "-crf", "28", "-preset", "slow", "-pix_fmt", "yuv420p",
                      "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-movflags", "+faststart"]
        else:
            encode = ["-filter_complex", ("split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];"
                                          "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle")]
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(FPS),
                        "-i", str(tmp / "out_%06d.png"), *encode, str(args.out)], check=True)
        print(f"{args.out}: {n_out} frames, {args.out.stat().st_size / 1e6:.1f} MB")
    finally:
        shutil.rmtree(tmp)


if __name__ == "__main__":
    main()
