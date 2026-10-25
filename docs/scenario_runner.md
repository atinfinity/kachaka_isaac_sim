# Scenario runner

[日本語](ja/scenario_runner.md) · [README](../README.md)

`scripts/run_scenarios.py` runs scenarios headless and in batches and reports
numbers, to compare parameter changes. A scenario file
([`scenarios/`](../scenarios)) gives the world or a layout file (and any layout
key over it: spawn, shelves, people, …), the steps (Nav2 goals, kachaka-api
calls, waits), the number of runs and the seed:

```yaml
layout: ../layouts/room_movers.yaml
repeat: 3
steps:
  - goal: [1.6, -0.3, 3.1416]          # Nav2 NavigateToPose, world frame
  - api: move_shelf                    # any KachakaApiClient method
    args: {shelf_name_or_id: S01, location_name_or_id: L01}
  - api: move_to_pose                  # x, y, yaw in the world frame
    args: {x: 0.0, y: 0.0, yaw: 0.0}
  - wait: 2.0
```

```bash
python3 scripts/run_scenarios.py scenarios/room_nav.yaml scenarios/room_movers.yaml scenarios/room_api.yaml
python3 scripts/run_scenarios.py scenarios/room_movers.yaml --nav2-params my_nav2.yaml --label tuned
python3 scripts/run_scenarios.py --report build/scenarios/*/     # earlier batches side by side
```

For each scenario it starts the simulator (`run_sim.sh --headless` with the
scenario's layout) and SLAM + Nav2 in the ROS image, runs the steps once per
run (`sim/reset` and a cleared SLAM map in between), and tears both down;
`restart: true` starts afresh for each run instead, with seed + run.
`--nav2-params` swaps in another Nav2 parameter file. `--backend fake` uses
`scripts/fake_sim.py` instead of Isaac Sim (no GPU; the default room only,
without people or boxes), as CI does with `scenarios/fake_room.yaml`.

Per run: success (every step; a goal or move_to_pose step must also end within
0.35 m by ground truth), sim time, path length (ground truth), collisions
(`sim/collision`, with what), the smallest clearance between a LiDAR point and
the body's outline, and Nav2 recoveries (NavigateToPose feedback, also for
kachaka-api moves). The results go to `build/scenarios/<date>[-label]/`:
`runs.csv`, `runs.json` (with each step), `summary.csv`, `summary.txt` and the
logs. Time and path are averaged over the successful runs:

```
scenario     success  time [s]     path [m]      collisions  min clear [m]  recoveries
-----------  -------  -----------  ------------  ----------  -------------  ----------
room_nav     3/3      48.8 ± 0.5   10.97 ± 0.13  0 (0 runs)  0.113          0.0
room_movers  2/3      52.8 ± 1.1   11.09 ± 0.04  0 (0 runs)  0.011          6.7
room_api     2/2      111.8 ± 2.0  15.71 ± 0.36  0 (0 runs)  0.000          1.0
```

One batch on an RTX 2080 Ti host. The failed room_movers run never left the
start: the planner found no path to the first goal while a person crossed in
front, and gave up after 20 recoveries. Under a shelf (move_shelf,
return_shelf) the clearance is about 0, since the shelf's legs are next to the
body until it is docked; `runs.json` has the clearance and recoveries per step.
