"""Download the upstream robot and shelf descriptions at pinned commits.

    python scripts/fetch_assets.py

Files land in third_party/ (git-ignored). Nothing from those repos is executed.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = ROOT / "third_party"

SOURCES = {
    # name: (url, commit)
    "kachaka-api": ("https://github.com/pf-robotics/kachaka-api.git", "4370c3d"),
    "kachaka_shelf_description": ("https://github.com/GAI-313/kachaka_shelf_description.git", None),
}


def run(*cmd: str, cwd: Path | None = None) -> None:
    subprocess.run(cmd, cwd=cwd, check=True)


def fetch(name: str, url: str, commit: str | None, force: bool) -> None:
    dest = THIRD_PARTY / name
    if dest.exists() and force:
        shutil.rmtree(dest)
    if not dest.exists():
        run("git", "clone", "--quiet", "--filter=blob:none", url, str(dest))
    if commit:
        run("git", "checkout", "--quiet", commit, cwd=dest)
    head = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"], cwd=dest, capture_output=True, text=True, check=True
    ).stdout.strip()
    print(f"[fetch] {name} @ {head}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="re-clone even if present")
    args = parser.parse_args()
    THIRD_PARTY.mkdir(exist_ok=True)
    for name, (url, commit) in SOURCES.items():
        fetch(name, url, commit, args.force)
    return 0


if __name__ == "__main__":
    sys.exit(main())
