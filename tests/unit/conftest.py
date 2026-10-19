"""Unit tests for the modules that need neither Isaac Sim nor ROS.

    uv run --no-project --with pytest,numpy,pyyaml,pillow python -m pytest
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

KACHAKA_DESCRIPTION = ROOT / "third_party/kachaka-api/ros2/kachaka_description"


@pytest.fixture(scope="session")
def kachaka_description() -> Path:
    if not KACHAKA_DESCRIPTION.exists():
        pytest.skip("third_party/kachaka-api is missing; run scripts/fetch_assets.py")
    return KACHAKA_DESCRIPTION
