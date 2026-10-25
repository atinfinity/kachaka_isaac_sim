"""The Isaac scene tables agree with each other and with the nav check routes."""

import ast
from pathlib import Path

from kachaka_isaac_sim.scenes import FROZEN_WORLDS, ISAAC_WORLDS, SPAWN

NAV_CHECK = Path(__file__).resolve().parents[1] / "ros_nav_check.py"


def nav_check_worlds():
    """WORLDS from tests/ros_nav_check.py, read without importing rclpy."""
    tree = ast.parse(NAV_CHECK.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "WORLDS" for t in node.targets):
            src = ast.unparse(node.value).replace("math.pi", "3.141592653589793")
            return eval(src, {})  # tuples of numbers and math.pi only
    raise AssertionError("WORLDS not found")


def test_every_isaac_world_has_a_spawn_pose():
    assert set(ISAAC_WORLDS) <= set(SPAWN)


def test_kitchens_are_listed():
    kitchens = {k for k in ISAAC_WORLDS if k.startswith("kitchen_")}
    assert kitchens == {f"kitchen_{k}" for k in ("l_shape", "u_shape", "g_shape", "l_island", "peninsula")}
    assert all(ISAAC_WORLDS[k].endswith(f"replicator_kitchen/{k}.usda") for k in kitchens)


def test_nav_check_routes_start_at_the_spawn_pose():
    worlds = nav_check_worlds()
    assert set(ISAAC_WORLDS) <= set(worlds)
    for name in ISAAC_WORLDS:
        assert worlds[name][0] == SPAWN[name], name


def test_kitchens_are_frozen():
    assert FROZEN_WORLDS == {k for k in ISAAC_WORLDS if k.startswith("kitchen_")}
