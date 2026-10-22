"""Run scenarios headless and in batches, and report numbers: to compare
parameter changes (kachaka_isaac_sim/scenario.py has the file format).

    python3 scripts/run_scenarios.py scenarios/room_nav.yaml scenarios/room_movers.yaml
    python3 scripts/run_scenarios.py scenarios/room_nav.yaml --nav2-params my_nav2.yaml --label tuned
    python3 scripts/run_scenarios.py --report build/scenarios/*/      # earlier results, side by side

For each scenario it starts the simulator (scripts/run_sim.sh --headless,
with the scenario's layout) and SLAM + Nav2 in the ROS image, runs
scripts/scenario_episode.py there for each run, and tears both down. With
`restart: true` that happens once per run (seed + run), else once per
scenario, with sim/reset between runs.

--backend fake uses scripts/fake_sim.py (no GPU; the default room only,
without people or boxes: CI does this) instead of Isaac Sim.

Writes to --out (default build/scenarios/<date>-<label>): runs.csv,
runs.json (with the steps of each run), summary.csv, summary.txt and
logs/. Exit code 0 when every run succeeded. Needs the ROS image
(docker compose -f docker/compose.yaml build) and PyYAML on the host.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from kachaka_isaac_sim.scenario import (
    Run,
    Scenario,
    ScenarioError,
    load_scenario,
    runs_csv,
    runs_json,
    summarize,
    summary_table,
)

IMAGE = "kachaka-isaac-sim-ros:jazzy"
SIM_CONTAINER = "kachaka-scenario-sim"
NAV_CONTAINER = "kachaka-scenario-nav"
EPISODE_CONTAINER = "kachaka-scenario-episode"
SIM_READY = {"isaac": r"\[kachaka\] running", "fake": r"\[fake_sim\] running"}
# The log has colour codes around the message.
NAV_READY = r"\[lifecycle_manager_navigation\]: .*Managed nodes are active"
SIM_STARTUP = {"isaac": 900.0, "fake": 120.0}  # s
NAV_STARTUP = 180.0
SIM_PATTERN = r"^\S*/bin/python \S*run_sim.py"


class StartupError(RuntimeError):
    pass


def docker(*args: str) -> list[str]:
    return ["docker", "run", "--rm", "--network", "host", "--ipc", "host",
            "-e", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4?sockets_size=4194304", *args]


def fake_args(sc: Scenario) -> list[str]:
    """scripts/fake_sim.py options for a scenario's layout: it has the
    default room only, so only shelves, charger: null and people: [] carry over."""
    lay = sc.layout
    unsupported = set(lay) - {"world", "seed", "shelves", "charger", "people"}
    if lay.get("world", "room") != "room":
        unsupported.add("world")
    if "charger" in lay and lay["charger"] is not None:
        unsupported.add("charger")
    if lay.get("people"):
        unsupported.add("people")
    if unsupported:
        raise ScenarioError(f"{sc.path}: --backend fake has the default room only; "
                            f"not supported: {', '.join(sorted(unsupported))}")
    out = []
    if "charger" in lay:
        out.append("--no-charger")
    if "people" in lay:
        out.append("--no-people")
    if "shelves" in lay:
        if not lay["shelves"]:
            out.append("--no-shelves")
        for s in lay["shelves"]:
            pose = s["pose"] if isinstance(s, dict) else s
            out += ["--shelf", *(str(float(v)) for v in pose)]
    return out


class Session:
    """One simulator and one SLAM + Nav2 stack, torn down together."""

    def __init__(self, args, sc: Scenario, seed: int | None, layout_file: Path, log_dir: Path, tag: str):
        self.args, self.sc, self.seed, self.layout_file = args, sc, seed, layout_file
        self.sim_log = log_dir / f"{tag}_sim.log"
        self.nav_log = log_dir / f"{tag}_nav.log"
        self.episode_log = log_dir / f"{tag}_episode.log"
        self.procs: list[subprocess.Popen] = []

    def _start(self, cmd: list[str], log: Path) -> subprocess.Popen:
        f = open(log, "w")  # noqa: SIM115 (closed with the process)
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, cwd=ROOT)
        self.procs.append(p)
        return p

    @staticmethod
    def _wait_for(p: subprocess.Popen, log: Path, pattern: str, timeout: float, what: str) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if re.search(pattern, log.read_text(errors="replace")):
                return
            if p.poll() is not None:
                raise StartupError(f"{what} exited (see {log})")
            time.sleep(1.0)
        raise StartupError(f"{what} did not come up in {timeout:.0f} s (see {log})")

    def start(self) -> None:
        a, sc = self.args, self.sc
        seed = [] if self.seed is None else ["--seed", str(self.seed)]
        if a.backend == "isaac":
            cmd = ["scripts/run_sim.sh", "--headless", "--layout", str(self.layout_file), *seed, *sc.sim_args]
            if self.seed is not None:
                cmd += ["--layout-seed", str(self.seed)]
        else:
            cmd = docker("--name", SIM_CONTAINER, "-v", f"{ROOT}:/repo:ro", a.image,
                         "python3", "/repo/scripts/fake_sim.py", *seed, *fake_args(sc), *sc.sim_args)
        print(f"[scenario] starting the simulator: {' '.join(cmd)}", flush=True)
        self._wait_for(self._start(cmd, self.sim_log), self.sim_log, SIM_READY[a.backend],
                       SIM_STARTUP[a.backend], "the simulator")
        mounts, launch = [], []
        if a.nav2_params:
            mounts = ["-v", f"{Path(a.nav2_params).resolve()}:/params/nav2_params.yaml:ro"]
            launch = ["params_file:=/params/nav2_params.yaml"]
        cmd = docker("--name", NAV_CONTAINER, *mounts, a.image, "ros2", "launch", "kachaka_sim_bringup",
                     "slam.launch.py", "nav:=true", "rviz:=false", *launch)
        print("[scenario] starting SLAM + Nav2", flush=True)
        self._wait_for(self._start(cmd, self.nav_log), self.nav_log, NAV_READY, NAV_STARTUP, "Nav2")

    def episode(self, runs: list[int]) -> list[Run]:
        spec = self.sc.episode(runs[0], runs)
        steps_timeout = sum(self.sc.timeout if s.kind != "wait" else s.seconds * 20 for s in self.sc.steps)
        timeout = len(runs) * (steps_timeout + 120.0) + 120.0
        cmd = docker("--name", EPISODE_CONTAINER, "-v", f"{ROOT}:/repo:ro", self.args.image,
                     "python3", "/repo/scripts/scenario_episode.py", json.dumps(spec))
        results = []
        end = time.monotonic() + timeout
        with open(self.episode_log, "w") as log:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)
            for line in p.stdout:
                log.write(line)
                if line.startswith("RESULT "):
                    results.append(Run.from_dict(json.loads(line[len("RESULT "):])))
                elif line.lstrip().startswith("[") or "not found" in line:
                    print(line.rstrip(), flush=True)
                if time.monotonic() > end:
                    subprocess.run(["docker", "rm", "-f", EPISODE_CONTAINER], capture_output=True, check=False)
                    break
            p.wait()
        done = {r.run for r in results}
        results += [failed_run(self.sc, r, self.sc.run_seed(r), f"no result (see {self.episode_log})")
                    for r in runs if r not in done]
        return results

    def stop(self) -> None:
        subprocess.run(["docker", "rm", "-f", SIM_CONTAINER, NAV_CONTAINER, EPISODE_CONTAINER],
                       capture_output=True, check=False)
        for p in self.procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        end = time.monotonic() + 60.0
        for p in self.procs:
            try:
                p.wait(max(0.1, end - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        self.procs.clear()


def failed_run(sc: Scenario, run: int, seed, why: str) -> Run:
    return Run(scenario=sc.name, run=run, seed=seed, success=False, time_s=math.nan, wall_s=math.nan,
               path_m=math.nan, collisions=0, collided_with="", min_clearance_m=math.nan, recoveries=0,
               failed_step=why, steps=[])


def run_scenario(args, sc: Scenario, out: Path) -> list[Run]:
    layout_file = out / "layouts" / f"{sc.name}.yaml"
    layout_file.parent.mkdir(parents=True, exist_ok=True)
    layout_file.write_text(yaml.safe_dump(sc.layout, sort_keys=False))
    groups = [[r] for r in range(sc.repeat)] if sc.restart else [list(range(sc.repeat))]
    results = []
    for group in groups:
        tag = sc.name if len(groups) == 1 else f"{sc.name}_{group[0]}"
        seed = sc.run_seed(group[0])
        session = Session(args, sc, seed, layout_file, out / "logs", tag)
        try:
            session.start()
            results += session.episode(group)
        except (StartupError, ScenarioError) as e:
            print(f"[scenario] {sc.name}: {e}", flush=True)
            results += [failed_run(sc, r, sc.run_seed(r), str(e)) for r in group]
        finally:
            session.stop()
        for r in results[-len(group):]:
            print(f"[scenario] {sc.name} #{r.run}: {'OK' if r.success else 'FAIL ' + r.failed_step}", flush=True)
    return results


def git_head() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=ROOT,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def write_outputs(out: Path, runs: list[Run], meta: dict) -> str:
    out.mkdir(parents=True, exist_ok=True)
    (out / "runs.csv").write_text(runs_csv(runs))
    (out / "runs.json").write_text(runs_json(runs, meta))
    rows = summarize(runs)
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]) if rows else ["scenario"], lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    table = summary_table(rows)
    (out / "summary.txt").write_text(table + "\n")
    return table


def report(dirs: list[str]) -> int:
    """Summaries of earlier results, one block per directory."""
    runs = []
    for d in dirs:
        data = json.loads((Path(d) / "runs.json").read_text())
        label = data["meta"].get("label") or Path(d).name
        for r in data["runs"]:
            r = {k: (math.nan if v is None and k.endswith(("_s", "_m")) else v) for k, v in r.items()}
            r["scenario"] = f"{r['scenario']} [{label}]"
            runs.append(Run.from_dict(r))
    runs.sort(key=lambda r: r.scenario)
    print(summary_table(summarize(runs)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("scenarios", nargs="*", help="scenario YAML files")
    ap.add_argument("--out", help="output directory (default build/scenarios/<date>[-label])")
    ap.add_argument("--label", default="", help="name for this batch, e.g. the parameter set")
    ap.add_argument("--nav2-params", metavar="FILE", help="Nav2 parameters instead of the bringup package's")
    ap.add_argument("--repeat", type=int, help="runs per scenario, over the scenario's repeat")
    ap.add_argument("--backend", choices=("isaac", "fake"), default="isaac")
    ap.add_argument("--image", default=IMAGE, help="ROS image")
    ap.add_argument("--report", nargs="+", metavar="DIR", help="print the summaries of earlier results and exit")
    args = ap.parse_args()
    if args.report:
        return report(args.report)
    if not args.scenarios:
        ap.error("give scenario files (or --report DIR ...)")
    try:
        scenarios = [load_scenario(p) for p in args.scenarios]
        for sc in scenarios:
            if args.repeat:
                sc.repeat = args.repeat
            if args.backend == "fake":
                fake_args(sc)
    except (OSError, ScenarioError) as e:
        print(f"[scenario] {e}", file=sys.stderr)
        return 2
    if len({sc.name for sc in scenarios}) != len(scenarios):
        print("[scenario] scenario names must differ", file=sys.stderr)
        return 2
    if shutil.which("docker") is None:
        print("[scenario] docker not found", file=sys.stderr)
        return 2
    busy = subprocess.run(["pgrep", "-f", SIM_PATTERN], capture_output=True, text=True, check=False).stdout.split()
    if args.backend == "isaac" and busy:
        print(f"[scenario] a simulator is already running (pid {', '.join(busy)}); stop it first", file=sys.stderr)
        return 2
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = Path(args.out or ROOT / "build" / "scenarios" / (stamp + (f"-{args.label}" if args.label else "")))
    (out / "logs").mkdir(parents=True, exist_ok=True)
    meta = {"label": args.label, "date": stamp, "git": git_head(), "backend": args.backend,
            "nav2_params": str(args.nav2_params or ""), "scenarios": [sc.path for sc in scenarios]}
    runs: list[Run] = []
    for sc in scenarios:
        print(f"[scenario] {sc.name}: {sc.repeat} run(s), {len(sc.steps)} step(s)", flush=True)
        runs += run_scenario(args, sc, out)
        write_outputs(out, runs, meta)  # keep what is done if a later scenario hangs
    print(write_outputs(out, runs, meta))
    print(f"[scenario] results in {out}")
    return 0 if all(r.success for r in runs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
