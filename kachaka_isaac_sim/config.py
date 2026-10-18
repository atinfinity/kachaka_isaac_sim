"""Load config/robot.yaml into attribute-style objects."""

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


def load_config(path: Path | str | None = None) -> SimpleNamespace:
    with open(path or DEFAULT_CONFIG) as f:
        return _to_ns(yaml.safe_load(f))
