"""Drive the simulated robot with kachaka-api calls, for the api demo video.

Run on the host against the simulator in the room with SLAM + Nav2 up
(scripts/record_demo.sh api does all of it). Each call is written to
CAPTIONS (wall_time,text) so make_demo_gif.py can show it under the video.

    uv run --no-project --with kachaka-api==3.17.5.0 python scripts/api_demo.py build/api/captions.csv
"""

import argparse
import csv
import math
import threading
import time

from kachaka_api import KachakaApiClient


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("captions")
    ap.add_argument("--target", default="127.0.0.1:26400")
    args = ap.parse_args()
    client = KachakaApiClient(args.target)
    with open(args.captions, "w", newline="") as f:
        csv.writer(f).writerow(["wall_time", "text"])

    def say(text: str) -> None:
        print(text, flush=True)
        with open(args.captions, "a", newline="") as f:
            csv.writer(f).writerow([f"{time.time():.3f}", text])

    def call(text: str, fn, *a, pause: float = 1.5, why: str = ""):
        say(f"client.{text}")
        res = fn(*a)
        ok = getattr(res, "success", None)
        if ok is not None:
            detail = "success" if ok else f"{why}error {res.error_code}"
            say(f"client.{text}  ->  {detail}")
        time.sleep(pause)
        return res

    pi = math.pi
    time.sleep(2.0)
    call("move_forward(0.5)", client.move_forward, 0.5)
    call("rotate_in_place(pi / 2)", client.rotate_in_place, pi / 2)
    call("rotate_in_place(-pi / 2)", client.rotate_in_place, -pi / 2)

    shortcuts = client.get_shortcuts()
    say(f"client.get_shortcuts()  ->  {shortcuts}")
    time.sleep(2.0)
    say('client.start_shortcut_command("hello")  (speak "Hello", then a full turn)')
    client.start_shortcut_command("hello")
    time.sleep(1.0)
    end = time.monotonic() + 60.0
    while time.monotonic() < end:
        _, cmd = client.get_last_command_result()
        if cmd.HasField("rotate_in_place_command"):
            break
        time.sleep(0.3)
    say('client.start_shortcut_command("hello")  ->  done')
    time.sleep(1.5)

    home = {s.id: s for s in client.get_shelves()}["S01"].home_location_id
    call(f'dock_any_shelf_with_registration("{home}")', client.dock_any_shelf_with_registration, home)
    call('return_shelf("S01")', client.return_shelf, "S01")

    call("move_to_pose(0.0, 0.0, 0.0)", client.move_to_pose, 0.0, 0.0, 0.0, pause=0.5)
    call("move_forward(5.0)  (towards the wall)", client.move_forward, 5.0, pause=2.0,
         why="stopped short of the wall, ")

    # Emergency stop while a command runs, then refusal and restart.
    say("client.move_to_pose(-1.8, 1.3, pi / 2)")
    th = threading.Thread(target=lambda: client.move_to_pose(-1.8, 1.3, pi / 2))
    th.start()
    time.sleep(4.0)
    say("client.set_emergency_stop()")
    client.set_emergency_stop()
    th.join()
    say(f"client.set_emergency_stop()  ->  get_error() = {client.get_error()}")
    time.sleep(2.0)
    call("move_forward(0.3)  (refused until restart)", client.move_forward, 0.3, why="emergency stop, ")
    call("restart_robot()", client.restart_robot)

    call("return_home()", client.return_home, pause=3.0)
    say("")


if __name__ == "__main__":
    main()
