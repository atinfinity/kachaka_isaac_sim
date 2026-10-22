"""mover_markers and door_markers: sim/movers and sim/doors JSON to RViz markers.

    colcon test --packages-select kachaka_sim_bringup
"""

import math

import pytest
from kachaka_sim_bringup.mover_markers import PARTS, mover_markers
from visualization_msgs.msg import Marker


def mover(**kw):
    m = {"id": "P01", "kind": "person", "present": True, "pose": [1.0, 0.0, 0.0], "map_pose": [0.0, 1.0, math.pi / 2],
         "velocity": [0.5, 0.0], "size": [0.28, 0.5, 1.7], "blocked": False}
    m.update(kw)
    return m


def test_clears_then_draws_present_movers():
    msg = {"movers": [mover(), mover(id="P02", present=False, pose=None, map_pose=None),
                      mover(id="cart", kind="box", size=[0.5, 0.4, 0.8], velocity=[0.0, 0.0])]}
    markers = mover_markers(msg, "map")
    assert markers[0].action == Marker.DELETEALL
    by_id = {m.id: m for m in markers[1:]}
    assert sorted(by_id) == [0, 1, 2, 2 * PARTS, 2 * PARTS + 1]  # the still cart has no arrow
    assert by_id[0].type == Marker.CYLINDER and by_id[0].pose.position.z == pytest.approx(0.85)
    assert by_id[1].text == "P01"
    assert by_id[2 * PARTS].type == Marker.CUBE and by_id[2 * PARTS].scale.z == pytest.approx(0.8)
    assert all(m.header.frame_id == "map" for m in markers)


def test_arrow_turns_the_velocity_into_the_map_frame():
    arrow = {m.id: m for m in mover_markers({"movers": [mover()]}, "map")}[2]
    start, end = arrow.points
    assert (start.x, start.y) == pytest.approx((0.0, 1.0))
    # World +x is map +y rotated by the 90 degrees between them: +0.5 m along map y.
    assert (end.x, end.y) == pytest.approx((0.0, 1.5))


def test_blocked_is_red():
    body = mover_markers({"movers": [mover(blocked=True)]}, "map")[1]
    assert body.color.r > 0.8 and body.color.g < 0.3


def test_door_markers():
    from kachaka_sim_bringup.door_markers import CLOSED, MOVING, door_markers

    door = {"id": "D01", "kind": "sliding", "pose": [1.0, 2.0, 0.0], "width": 0.8, "state": "closing",
            "opening": 0.4, "cycling": False, "panel": {"pose": [1.4, 1.9, 0.0], "size": [0.9, 0.04, 2.0]}}
    body, label = door_markers({"doors": [door]}, "map")
    assert body.type == Marker.CUBE and body.color == MOVING
    assert (body.pose.position.x, body.pose.position.y, body.pose.position.z) == pytest.approx((1.4, 1.9, 1.0))
    assert (body.scale.x, body.scale.y, body.scale.z) == pytest.approx((0.9, 0.04, 2.0))
    assert label.text == "D01 closing"
    body, _ = door_markers({"doors": [dict(door, state="closed")]}, "map")
    assert body.color == CLOSED
    assert door_markers({"doors": [dict(door, panel=None)]}, "map") == []


def test_auto_door_markers_show_the_sensor_area():
    from kachaka_sim_bringup.door_markers import SENSOR, door_markers

    door = {"id": "D01", "kind": "sliding", "pose": [1.0, 2.0, 0.5], "width": 0.8, "state": "closed",
            "opening": 0.0, "cycling": False, "auto": True, "sensor": {"size": [1.4, 2.0]},
            "panel": {"pose": [1.0, 1.9, 0.5], "size": [0.9, 0.04, 2.0]}}
    _, label, area = door_markers({"doors": [door]}, "map")
    assert label.text == "D01 closed (auto)"
    assert area.color == SENSOR and (area.scale.x, area.scale.y) == pytest.approx((1.4, 2.0))
    assert (area.pose.position.x, area.pose.position.y) == pytest.approx((1.0, 2.0))


def test_door_colours_stand_out_from_the_scan():
    """The LaserScan is drawn red (255, 64, 64) in config/*.rviz."""
    from kachaka_sim_bringup.door_markers import CLOSED, MOVING, OPEN

    for c in (OPEN, CLOSED, MOVING):
        assert abs(c.r - 1.0) + abs(c.g - 0.25) + abs(c.b - 0.25) > 0.7
