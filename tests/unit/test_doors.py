import math
from types import SimpleNamespace

import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.doors import (
    SLIDING_OVERLAP,
    AutoSpec,
    Doors,
    DoorSpec,
    DoorState,
    door_rate,
    panel_pose,
    panel_size,
)
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


def bare_doors(specs):
    """doors.Doors without a USD stage: panels are not moved."""
    cfg = SimpleNamespace(thickness=0.04, height=2.0, sliding_speed=0.5, hinged_speed=1.5)
    d = Doors.__new__(Doors)
    d.specs, d.cfg, d.wall = specs, cfg, 0.1
    d.states = [DoorState(s, door_rate(s, cfg)) for s in specs]
    d.t, d.changed, d.next_motion = 0.0, True, 0.0
    d.robot_pose, d.mover_poses = (lambda: None), list
    d._move = lambda idx: None
    return d


def test_doors_publish_while_a_panel_moves():
    d = bare_doors([DoorSpec("D01", "/World/Doors/D01", Pose2D(0.0, 0.0, 0.0), 0.8)])
    assert d.take_changed()
    d.step(0.01)
    assert not d.take_changed()  # open and still
    d.command("D01", False)
    assert d.take_changed()
    sent = 0
    for _ in range(50):  # 0.5 s of closing at 100 Hz
        d.step(0.01)
        sent += d.take_changed()
    assert d.states[0].state == "closing"
    assert 4 <= sent <= 6  # every MOTION_PERIOD (0.1 s)


def test_doors_panels():
    d = bare_doors([DoorSpec("D01", "/World/Doors/D01", Pose2D(1.0, 2.0, 0.0), 0.8, kind="hinged", open=False)])
    (pose, size), = d.panels()
    assert size == pytest.approx((0.79, 0.04, 2.0))
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 2.0, 0.0))  # closed: across the doorway


def auto_spec(**kw):
    return spec(open=False, auto=AutoSpec(**kw))


def test_auto_sensor_area_is_on_both_sides_of_the_doorway():
    s = auto_spec(range=1.0, margin=0.3)  # doorway at (1, 2) along x, 0.8 wide
    assert s.senses(1.0, 2.9) and s.senses(1.0, 1.1)  # either side, within range
    assert not s.senses(1.0, 3.1)  # too far out
    assert s.senses(1.0 + 0.4 + 0.29, 2.5) and not s.senses(1.0 + 0.4 + 0.31, 2.5)  # margin past the edge
    turned = spec(pose=Pose2D(0.0, 0.0, math.pi / 2), auto=AutoSpec(range=1.0, margin=0.3))
    assert turned.senses(0.9, 0.0) and not turned.senses(0.0, 0.8)
    assert not spec().senses(1.0, 2.0)  # not automatic


def test_auto_door_opens_holds_and_closes():
    st = DoorState(auto_spec(hold=2.0), rate=100.0)
    assert st.automatic and st.state == "closed"
    st.step(0.0, 0.01, sensed=True)
    assert st.state == "open"
    st.step(1.0, 0.01, sensed=False)
    assert st.state == "open"  # held
    st.step(2.05, 0.01, sensed=False)
    assert st.state == "closed"


def test_auto_door_reopens_while_closing():
    st = DoorState(auto_spec(hold=1.0), rate=1.0)
    st.opening = 1.0
    st.step(0.0, 0.01, sensed=True)
    st.step(1.5, 0.5, sensed=False)
    assert st.state == "closing"
    st.step(1.6, 0.1, sensed=True)
    assert st.state == "opening"


def test_command_overrides_the_sensor_until_reset():
    st = DoorState(auto_spec(), rate=100.0)
    st.command(False)
    st.step(0.0, 0.01, sensed=True)
    assert st.state == "closed" and not st.automatic
    st.reset()
    assert st.automatic


def test_auto_door_must_be_sliding_without_a_cycle():
    with pytest.raises(ValueError):
        spec(kind="hinged", auto=AutoSpec())
    with pytest.raises(ValueError):
        spec(cycle=(1.0, 1.0), auto=AutoSpec())


def test_doors_sense_the_robot_and_movers():
    d = bare_doors([auto_spec(movers=True), DoorSpec("D02", "/World/Doors/D02", Pose2D(5.0, 2.0, 0.0), 0.8,
                                                     open=False, auto=AutoSpec(movers=False))])
    d.robot_pose = lambda: Pose2D(9.0, 9.0, 0.0)
    d.mover_poses = lambda: [None, Pose2D(1.0, 2.5, 0.0), Pose2D(5.0, 2.5, 0.0)]
    assert d._sensed() == [True, False]  # D02 ignores movers
    d.robot_pose = lambda: Pose2D(5.0, 1.5, 0.0)
    assert d._sensed() == [True, True]
    d.step(0.01)
    assert [st.target for st in d.states] == [True, True]


def test_auto_from_config():
    a = load_config().door.auto
    assert a.range > 0 and a.margin >= 0 and a.hold > 0 and isinstance(a.movers, bool)
