import csv
import io
import json
import math
from pathlib import Path

import pytest

from kachaka_isaac_sim.scenario import (
    BODY_X,
    BODY_Y,
    LIDAR_X,
    Run,
    ScenarioError,
    body_clearance,
    count_recoveries,
    load_scenario,
    parse_scenario,
    path_length,
    runs_csv,
    runs_json,
    step_pose,
    summarize,
    summary_table,
    to_map,
)

SCENARIOS = Path(__file__).resolve().parents[2] / "scenarios"


def test_example_scenarios_load():
    names = sorted(p.stem for p in SCENARIOS.glob("*.yaml"))
    assert names
    for name in names:
        sc = load_scenario(SCENARIOS / f"{name}.yaml")
        assert sc.name == name and sc.steps and sc.repeat >= 1


def test_layout_file_and_overrides(tmp_path):
    (tmp_path / "lay.yaml").write_text("world: office\nseed: 4\nspawn: [1.0, 2.0, 0.5]\nshelves: []\n")
    sc = parse_scenario({"layout": "lay.yaml", "spawn": [3.0, 4.0, 0.0], "repeat": 2, "restart": True,
                         "steps": [{"goal": [5, 6, 0]}, {"wait": 1}]}, str(tmp_path / "sc.yaml"))
    assert sc.world == "office" and sc.layout["shelves"] == [] and sc.seed == 4
    assert sc.spawn() == (3.0, 4.0, 0.0)
    assert [sc.run_seed(r) for r in range(2)] == [4, 5]
    spec = sc.episode(1, [1])
    assert spec["seeds"] == [5] and spec["steps"][0]["pose"] == (5.0, 6.0, 0.0)
    json.dumps(spec)


def test_same_seed_without_restart_and_world_spawn():
    sc = parse_scenario({"world": "office", "seed": 7, "repeat": 3, "steps": [{"wait": 1}]})
    assert [sc.run_seed(r) for r in range(3)] == [7, 7, 7]
    assert sc.spawn() == (0.0, 0.0, 1.5708)  # scenes.SPAWN
    assert sc.clear_map and sc.timeout == 300.0


def test_random_spawn_follows_the_run_seed():
    sc = parse_scenario({"seed": 1, "restart": True, "repeat": 2, "spawn": [[-1.0, 1.0], [-1.0, 1.0], 0.0],
                         "steps": [{"wait": 1}]})
    assert sc.spawn(0) == sc.spawn(0) != sc.spawn(1)


@pytest.mark.parametrize("data, msg", [
    ({"steps": [{"goal": [0, 0, 0]}], "colour": 1}, "unknown key"),
    ({"steps": []}, "no steps"),
    ({"steps": [{"goal": [0, 0]}]}, "goal is"),
    ({"steps": [{"goal": [0, 0, 0], "wait": 1}]}, "one of"),
    ({"steps": [{"api": "move_to_pose", "args": {"x": 0}}]}, "needs x, y and yaw"),
    ({"steps": [{"wait": 1, "args": {}}]}, "unknown key"),
    ({"repeat": 0, "steps": [{"wait": 1}]}, "repeat"),
    ({"spawn": [[0, 1], 0, 0], "steps": [{"wait": 1}]}, "give a seed"),
    ({"layout": "missing.yaml", "steps": [{"wait": 1}]}, "not found"),
    ({"shelves": "S01", "steps": [{"wait": 1}]}, "shelves"),
])
def test_rejects_bad_scenarios(data, msg):
    with pytest.raises(ScenarioError, match=msg):
        parse_scenario(data, "sc.yaml")


def test_to_map_and_step_pose():
    x, y, yaw = to_map((1.0, 2.0, math.pi / 2), 1.0, 3.0, math.pi)
    assert (x, y, yaw) == pytest.approx((1.0, 0.0, math.pi / 2))
    assert step_pose({"kind": "goal", "pose": [1, 2, 3]}) == (1, 2, 3)
    assert step_pose({"kind": "api", "method": "move_to_pose", "args": {"x": 1, "y": 2, "yaw": 3}}) == (1, 2, 3)
    assert step_pose({"kind": "api", "method": "return_home", "args": {}}) is None


def test_path_length_skips_jitter():
    pts = [(0, 0), (0.001, 0), (0, 0.001), (1, 0), (1, 1)]
    assert path_length(pts) == pytest.approx(2.0)
    assert path_length([]) == 0.0


def test_body_clearance():
    # Laser frame is turned +90 deg: laser angle -pi/2 looks along base +x.
    ahead = 1.0
    c = body_clearance(-math.pi / 2, 0.0, [ahead])
    assert c == pytest.approx(LIDAR_X + ahead - BODY_X[1])
    # Laser angle 0 looks to the robot's left (base +y).
    c = body_clearance(0.0, 0.0, [0.5])
    assert c == pytest.approx(0.5 - BODY_Y[1])
    assert body_clearance(0.0, 0.01, [math.inf, float("nan"), 0.01], range_min=0.05) == math.inf


def test_count_recoveries_per_goal():
    assert count_recoveries([("a", 0), ("a", 1), ("a", 1), ("b", 0), ("b", 2), ("a", 1)]) == 3
    assert count_recoveries([]) == 0


def run(name, i, ok, t=10.0, path=2.0, coll=0, clear=0.2, rec=0):
    return Run(name, i, 1, ok, t, t, path, coll, "/World/P" if coll else "", clear, rec, "" if ok else "x", [])


def test_summary_and_outputs():
    runs = [run("a", 0, True, 10.0), run("a", 1, True, 12.0, coll=2, clear=0.1, rec=1),
            run("a", 2, False, 99.0, clear=math.inf), run("b", 0, False, math.nan, clear=math.nan)]
    rows = summarize(runs)
    a, b = rows
    assert (a["runs"], a["success"], a["collisions"], a["runs_with_collisions"]) == (3, 2, 2, 1)
    assert a["time_mean_s"] == pytest.approx(11.0)  # successful runs only
    assert a["min_clearance_m"] == pytest.approx(0.1) and a["recoveries_mean"] == pytest.approx(1 / 3)
    assert b["success_rate"] == 0.0 and math.isnan(b["time_mean_s"]) and math.isnan(b["min_clearance_m"])
    table = summary_table(rows)
    assert "2/3" in table and "0/1" in table
    rows_csv = list(csv.DictReader(io.StringIO(runs_csv(runs))))
    assert len(rows_csv) == 4 and rows_csv[2]["min_clearance_m"] == "inf" and rows_csv[3]["time_s"] == ""
    data = json.loads(runs_json(runs, {"label": "x"}))
    assert data["meta"]["label"] == "x" and data["runs"][3]["time_s"] is None and len(data["summary"]) == 2
    assert Run.from_dict(data["runs"][0]) == runs[0]
