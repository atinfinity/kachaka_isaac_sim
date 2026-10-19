"""Turn the robot once in place and draw kachaka-api object detections on the
front camera image, like the real robot's app does. Saves a GIF.

Run on the host against the simulator (scripts/run_sim.sh); needs only the
pip package, not ROS:

    uv run --no-project --with kachaka-api==3.17.5.0 --with pillow python scripts/detection_demo.py
    python scripts/detection_demo.py --out docs/object_detection.gif --turns 1 --rate 0.6
"""

import argparse
import io
import math
import time

from kachaka_api import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2
from PIL import Image, ImageDraw, ImageFont

COLORS = {pb2.OBJECT_LABEL_PERSON: (255, 80, 80), pb2.OBJECT_LABEL_SHELF: (80, 200, 255),
          pb2.OBJECT_LABEL_CHARGER: (120, 255, 120), pb2.OBJECT_LABEL_DOOR: (255, 220, 80)}


def font(size: int):
    try:
        return ImageFont.truetype("DejaVuSans-Bold.ttf", size)
    except OSError:
        return ImageFont.load_default()


def frame(client: KachakaApiClient, after: int) -> tuple[int, Image.Image, list]:
    """The first front image newer than stamp `after` and the detections made
    on it (same stamp)."""
    while True:
        header, objects = client.get_object_detection()
        if header.stamp_nsec <= after:
            time.sleep(0.01)
            continue
        img = client.get_front_camera_ros_compressed_image()
        if img.header.stamp_nsec == header.stamp_nsec:
            return header.stamp_nsec, Image.open(io.BytesIO(img.data)).convert("RGB"), objects


def draw(im: Image.Image, objects, t: float) -> Image.Image:
    d = ImageDraw.Draw(im)
    f = font(15)
    for o in objects:
        r, color = o.roi, COLORS.get(o.label, (255, 255, 255))
        d.rectangle([r.x_offset, r.y_offset, r.x_offset + r.width - 1, r.y_offset + r.height - 1], outline=color,
                    width=3)
        text = f"{pb2.ObjectLabel.Name(o.label).removeprefix('OBJECT_LABEL_').lower()} {o.score:.2f} " \
               f"{o.distance_median:.1f} m"
        x, y = r.x_offset + 2, max(r.y_offset - 20, 0) if r.y_offset >= 20 else r.y_offset + 2
        box = d.textbbox((x, y), text, font=f)
        d.rectangle([box[0] - 2, box[1] - 2, box[2] + 2, box[3] + 2], fill=color)
        d.text((x, y), text, fill=(0, 0, 0), font=f)
    d.text((8, im.height - 24), f"t = {t:4.1f} s   {len(objects)} objects", fill=(255, 255, 255), font=font(16))
    return im


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--out", default="build/object_detection.gif")
    ap.add_argument("--turns", type=float, default=1.0)
    ap.add_argument("--rate", type=float, default=0.6, help="turn rate [rad/s]")
    ap.add_argument("--scale", type=float, default=0.75, help="GIF size relative to the camera image")
    args = ap.parse_args()

    client = KachakaApiClient(args.target)
    client.set_manual_control_enabled(True)
    frames, stamp = [], 0
    stamp, _, _ = frame(client, stamp)
    t0, duration = stamp, args.turns * 2 * math.pi / args.rate
    while (t := (stamp - t0) * 1e-9) < duration:  # sim time
        client.set_robot_velocity(0.0, args.rate)  # the robot stops 0.5 s after the last command
        stamp, im, objects = frame(client, stamp)
        frames.append(draw(im, objects, t))
        print(f"{t:5.1f} s  " + ", ".join(
            f"{pb2.ObjectLabel.Name(o.label)[13:]} {o.distance_median:.1f} m" for o in objects), flush=True)
    client.set_robot_velocity(0.0, 0.0)
    size = (round(frames[0].width * args.scale), round(frames[0].height * args.scale))
    frames = [f.resize(size, Image.LANCZOS) for f in frames]
    frames[0].save(args.out, save_all=True, append_images=frames[1:], duration=round(duration / len(frames) * 1000),
                   loop=0, optimize=True)
    print(f"saved {len(frames)} frames to {args.out}")


if __name__ == "__main__":
    main()
