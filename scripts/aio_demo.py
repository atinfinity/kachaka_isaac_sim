"""Stream the robot pose and the command state while a command runs, with the
asyncio client (kachaka_api.aio.KachakaApiClient).

Run on the host against the simulator with SLAM + Nav2 up:

    uv run --no-project --with kachaka-api==3.17.5.0 python scripts/aio_demo.py
    python scripts/aio_demo.py --target 127.0.0.1:26400 --location L01
"""

import argparse
import asyncio
import time

from kachaka_api.aio import KachakaApiClient
from kachaka_api.generated import kachaka_api_pb2 as pb2


async def print_poses(client: KachakaApiClient, t0: float, period: float = 0.5) -> None:
    last = -period
    async for pose in client.robot_pose.stream():
        t = time.monotonic() - t0
        if t - last >= period:  # the stream yields at the odometry rate; print less often
            print(f"{t:5.1f} s  pose x {pose.x:+.2f}  y {pose.y:+.2f}  theta {pose.theta:+.2f}", flush=True)
            last = t


async def print_command_state(client: KachakaApiClient, t0: float) -> None:
    async for state, command in client.command_state.stream():
        name = command.WhichOneof("command") or "-"
        print(f"{time.monotonic() - t0:5.1f} s  {pb2.CommandState.Name(state)} {name}", flush=True)


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="127.0.0.1:26400")
    ap.add_argument("--location", default="L01", help="location to go to, then back home")
    args = ap.parse_args()

    client = KachakaApiClient(args.target)
    await client.update_resolver()
    t0 = time.monotonic()
    watchers = [asyncio.create_task(print_poses(client, t0)),
                asyncio.create_task(print_command_state(client, t0))]
    for target in (args.location, "home"):
        print(f"{time.monotonic() - t0:5.1f} s  move_to_location {target}", flush=True)
        result = await client.move_to_location(target)  # waits; the streams print meanwhile
        print(f"{time.monotonic() - t0:5.1f} s  -> {'success' if result.success else f'error {result.error_code}'}",
              flush=True)
    for w in watchers:
        w.cancel()
    await asyncio.gather(*watchers, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(main())
