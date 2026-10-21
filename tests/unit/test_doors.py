import math

import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.doors import SLIDING_OVERLAP, DoorSpec, DoorState, door_rate, panel_pose, panel_size
from kachaka_isaac_sim.drive import Pose2D


def spec(**kw):
    args = {"id": "D01", "path": "/World/Doors/D01", "pose": Pose2D(1.0, 2.0, 0.0), "width": 0.8}
    args.update(kw)
    return DoorSpec(**args)


def test_sliding_panel_covers_then_clears_the_doorway():
    s = spec()
    length, _ = panel_size(s, 0.04)
    closed = panel_pose(s, 0.0, 0.04, 0.05)
    assert (closed.x, closed.yaw) == (pytest.approx(1.0), 0.0)
    assert closed.y < 2.0 - 0.025  # on the wall's -y face
    opened = panel_pose(s, 1.0, 0.04, 0.05)
    # The panel's -x end is at the doorway's +x edge.
    assert opened.x - length / 2 == pytest.approx(1.0 + 0.4)


def test_hinged_panel_turns_about_its_hinge():
    s = spec(kind="hinged", pose=Pose2D(0.0, 0.0, math.pi / 2))  # wall along y
    length, _ = panel_size(s, 0.04)
    closed = panel_pose(s, 0.0, 0.04, 0.05)
    assert (closed.x, closed.y) == (pytest.approx(0.0), pytest.approx(0.0, abs=1e-9))
    opened = panel_pose(s, 1.0, 0.04, 0.05)
    hinge = (0.0, -length / 2)  # the doorway's -x side, turned onto -y
    # Open 90 degrees: the panel stands out of the wall on the door's -y side (world +x).
    assert opened.yaw == pytest.approx(0.0)
    assert opened.x == pytest.approx(hinge[0] + length / 2)
    assert opened.y == pytest.approx(hinge[1])
    half = panel_pose(s, 0.5, 0.04, 0.05)
    assert math.dist((half.x, half.y), hinge) == pytest.approx(length / 2)


def test_state_moves_at_the_rate_and_reports_motion():
    st = DoorState(spec(), rate=0.5)
    assert st.state == "open" and not st.step(0.1, 0.1)
    st.command(False)
    assert st.step(0.2, 1.0) and st.opening == pytest.approx(0.5) and st.state == "closing"
    st.step(1.2, 1.0)
    assert st.opening == 0.0 and st.state == "closed"
    st.command(True)
    st.step(2.2, 0.4)
    assert st.state == "opening"


def test_cycle_starts_in_the_start_state_and_a_command_stops_it():
    st = DoorState(spec(open=False, cycle=(2.0, 3.0)), rate=100.0)
    st.step(1.0, 0.01)
    assert st.state == "closed"
    st.step(3.5, 0.01)
    assert st.state == "open"
    st.step(5.5, 0.01)
    assert st.state == "closed"
    st.command(True)
    st.step(6.0, 0.01)
    assert st.state == "open" and not st.cycling
    st.reset()
    assert st.cycling and st.state == "closed"


def test_rate_from_config():
    door = load_config().door
    s = spec(width=0.8)
    assert door_rate(s, door) == pytest.approx(door.sliding_speed / (0.8 + SLIDING_OVERLAP))
    assert door_rate(spec(kind="hinged"), door) == pytest.approx(door.hinged_speed / (math.pi / 2))
