#!/usr/bin/env bash
# Record the torch light demo: the simulator's view of a dark room next to
# the front and back camera images -> build/torch.gif (or the path given).
#
#   scripts/record_torch_demo.sh [docs/images/torch.gif]
#
# Needs uv and ffmpeg on the host; no ROS container.
set -euo pipefail
cd "$(dirname "$0")/.."
GIF=${1:-build/torch.gif}
OUT=build/torch
VIEW=(${VIEW:-0.0 -1.9 2.3 0.0 0.6 0.2})
rm -rf "$OUT" && mkdir -p "$OUT"

cleanup() {
    pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT

echo "[torch] starting the simulator (dark room)"
scripts/run_sim.sh --headless --dome-light 0 --record "$OUT/sim" --record-view "${VIEW[@]}" > "$OUT/sim.log" 2>&1 &
until grep -q "\[kachaka\] running" "$OUT/sim.log"; do sleep 2; done
sleep 5

echo "[torch] running the demo"
uv run --no-project --with kachaka-api==3.17.5.0 --with numpy --with pillow \
    python scripts/torch_demo.py "$OUT" | tee "$OUT/run.log"

# Stop the simulator so the frame writer finishes the last PNGs.
pgrep -f "^\S*/bin/python \S*run_sim.py" | xargs -r kill -INT
for _ in $(seq 60); do pgrep -f "^\S*/bin/python \S*run_sim.py" >/dev/null || break; sleep 1; done

echo "[torch] composing $GIF"
.venv/bin/python scripts/torch_demo.py "$OUT" --compose "$GIF" --speed "${SPEED:-1.5}" --height "${HEIGHT:-400}"
