"""Load config/robot.yaml into attribute-style objects.

A config file may name another as its `base` (a path relative to the file):
it is loaded first, and the file's keys replace or extend it (mappings
merge, anything else is replaced). config/degraded.yaml is one.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/robot.yaml"


def _to_ns(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k: _to_ns(v) for k, v in value.items()})
    if isinstance(value, list):
        return [_to_ns(v) for v in value]
    return value


def merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _load(path: Path, seen: tuple[Path, ...] = ()) -> dict:
    path = path.resolve()
    if path in seen:
        raise ValueError(f"config base loop: {' -> '.join(map(str, (*seen, path)))}")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    base = data.pop("base", None)
    return merge(_load(path.parent / base, (*seen, path)), data) if base else data


def load_config(path: Path | str | None = None) -> SimpleNamespace:
    return _to_ns(_load(Path(path or DEFAULT_CONFIG)))
