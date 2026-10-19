"""Check the torch lights through kachaka-api: in a dark room, each camera's
image brightens when its torch is on and the other torch leaves it alone.

Run on the host against the simulator in a dark default room:

    scripts/run_sim.sh --headless --dome-light 0
    uv run --no-project --with kachaka-api==3.17.5.0 --with numpy python tests/api_torch_check.py

Prints the mean brightness of each camera; saves the frames to argv --out.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from kachaka_api import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2
from PIL import Image

failures = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}", flush=True)
    if not ok:
        failures.append(name)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--out", type=Path, default=Path("build/torch_check"))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    client = KachakaApiClient(args.target)

    def frames(label):
        time.sleep(1.0)  # let the light change reach a rendered frame
        means = {}
        for cam in ("front", "back"):
            img = getattr(client, f"get_{cam}_camera_ros_image")()
            a = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
            Image.fromarray(a).save(args.out / f"{label}_{cam}.png")
            means[cam] = float(a.mean())
        print(f"  {label:10s} front {means['front']:6.1f}  back {means['back']:6.1f}", flush=True)
        return means

    def set_torches(front, back):
        for cam, level in (("front", front), ("back", back)):
            # The Python client has no wrapper for the torch RPCs.
            rpc = "Front" if cam == "front" else "Back"
            req = getattr(pb2, f"Set{rpc}TorchIntensityRequest")(intensity=level)
            res = getattr(client.stub, f"Set{rpc}TorchIntensity")(req)
            check(f"Set{rpc}TorchIntensity({level})", res.result.success)

    set_torches(0, 0)
    off = frames("off")
    set_torches(255, 0)
    front = frames("front_on")
    set_torches(0, 255)
    back = frames("back_on")
    set_torches(128, 128)
    half = frames("both_128")
    set_torches(0, 0)
    off2 = frames("off_again")

    check("front torch brightens the front camera", front["front"] > off["front"] + 10,
          f"{off['front']:.1f} -> {front['front']:.1f}")
    check("front torch leaves the back camera", abs(front["back"] - off["back"]) < 5,
          f"{off['back']:.1f} -> {front['back']:.1f}")
    check("back torch brightens the back camera", back["back"] > off["back"] + 10,
          f"{off['back']:.1f} -> {back['back']:.1f}")
    check("back torch leaves the front camera", abs(back["front"] - off["front"]) < 5,
          f"{off['front']:.1f} -> {back['front']:.1f}")
    check("level 128 is dimmer than 255", off["front"] < half["front"] < front["front"]
          and off["back"] < half["back"] < back["back"],
          f"front {half['front']:.1f}, back {half['back']:.1f}")
    check("off again", abs(off2["front"] - off["front"]) < 5 and abs(off2["back"] - off["back"]) < 5,
          f"front {off2['front']:.1f}, back {off2['back']:.1f}")
    print("ALL PASSED" if not failures else f"{len(failures)} FAILED: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
