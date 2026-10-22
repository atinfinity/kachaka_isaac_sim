import itertools
import math

import pytest

from kachaka_isaac_sim.drive import Pose2D
from kachaka_isaac_sim.layout import LayoutError, parse_layout
from kachaka_isaac_sim.movers import ROBOT_RADIUS, STRIDE, MoverSpec, MoverState, gait_angles


def spec(**kw):
    args = {"path": "/World/People/P01", "start": Pose2D(0.0, 0.0, 0.0), "radius": 0.3, "speed": 0.5,
            "route": [(2.0, 0.0)]}
    args.update(kw)
    return MoverSpec(**args)


def run(st, seconds, robot=None, dt=0.05):
    track = []
    for _ in range(round(seconds / dt)):
        st.step(dt, robot)
        track.append((st.pose.x, st.pose.y, st.pose.yaw, st.present))
    return track


def test_walks_the_route_and_back_at_its_speed():
    st = MoverState(spec())
    run(st, 2.0)
    assert st.pose.x == pytest.approx(1.0, abs=0.06)
    run(st, 2.2)
    assert st.pose.x == pytest.approx(2.0, abs=1e-6)  # ends on the waypoint
    run(st, 2.0)  # turns about (pi at 2 rad/s), then walks back
    assert st.pose.x < 2.0
    assert abs(st.pose.yaw) == pytest.approx(math.pi, abs=0.01)
    run(st, 6.0)
    assert st.pose.x < 1.0  # back towards the start


def test_turns_before_walking():
    st = MoverState(spec(route=[(0.0, 2.0)]))  # 90 degrees to the left
    st.step(0.05)
    assert (st.pose.x, st.pose.y) == (0.0, 0.0)
    assert st.pose.yaw == pytest.approx(0.1)
    run(st, 1.0)
    assert st.pose.y > 0.0


def test_pauses_at_a_waypoint():
    st = MoverState(spec(route=[(0.5, 0.0)], pause=1.0))
    run(st, 1.05)
    assert st.pose.x == pytest.approx(0.5)
    track = run(st, 0.9)
    assert all((x, y, yaw) == track[0][:3] for x, y, yaw, _ in track)


def test_waits_while_the_robot_is_in_its_way():
    st = MoverState(spec())
    robot = Pose2D(1.0, 0.0, math.pi)  # right ahead
    run(st, 2.5, robot)
    assert st.pose.x < 1.0 - 0.3 - ROBOT_RADIUS
    assert st.blocked
    run(st, 5.0, Pose2D(1.0, 1.5, 0.0))  # gone to the side
    assert st.pose.x == pytest.approx(2.0)
    assert not st.blocked


def test_turns_back_after_waiting_long_for_the_robot():
    st = MoverState(spec(start=Pose2D(1.0, 0.0, 0.0), route=[(-1.0, 0.0), (3.0, 0.0)]))
    run(st, 2.0)  # turned round, walking towards (-1, 0)
    robot = Pose2D(st.pose.x - 0.5, 0.0, 0.0)
    run(st, 2.9, robot)
    assert st.blocked and st.target == (-1.0, 0.0)
    run(st, 2.6, robot)  # gives way: back to where it came from ...
    assert st.pose.x == pytest.approx(1.0) and not st.blocked
    assert st.target == (-1.0, 0.0)  # ... then on as before


def test_wanders_elsewhere_after_waiting_long_for_the_robot():
    area = ((-1.0, 1.0), (-1.0, 1.0))
    st = MoverState(spec(route=[], wander=area, seed=1))
    target = st.target
    robot = Pose2D(0.5 * target[0], 0.5 * target[1], 0.0)
    run(st, 3.5, robot)
    assert st.target != target


def test_does_not_wait_for_a_robot_beside_its_path():
    st = MoverState(spec())
    run(st, 5.0, Pose2D(1.0, 1.0, 0.0))
    assert st.pose.x == pytest.approx(2.0)


def test_wanders_inside_its_area_the_same_with_the_same_seed():
    area = ((-1.0, 1.0), (2.0, 3.0))
    a, b = (MoverState(spec(route=[], wander=area, start=Pose2D(0.0, 2.5, 0.0), seed=4)) for _ in range(2))
    ta, tb = run(a, 30.0), run(b, 30.0)
    assert ta == tb
    assert all(-1.0 - 1e-9 <= x <= 1.0 + 1e-9 and 2.0 - 1e-9 <= y <= 3.0 + 1e-9 for x, y, _, _ in ta)
    assert len({(round(x, 1), round(y, 1)) for x, y, _, _ in ta}) > 20
    c = MoverState(spec(route=[], wander=area, start=Pose2D(0.0, 2.5, 0.0), seed=5))
    assert run(c, 30.0) != ta


def test_appears_and_goes_away_in_turn():
    st = MoverState(spec(route=[], appear=(2.0, 1.0)))
    seen = [p for *_, p in run(st, 9.0)]
    assert 0.6 < sum(seen) / len(seen) < 0.73
    changes = sum(1 for a, b in itertools.pairwise(seen) if a != b)
    assert 5 <= changes <= 7


def test_does_not_appear_on_the_robot():
    st = MoverState(spec(route=[], appear=(1.0, 1.0)))
    run(st, 4.0, Pose2D(0.2, 0.0, 0.0))
    assert not st.present
    run(st, 2.1, Pose2D(3.0, 0.0, 0.0))
    assert st.present or any(p for *_, p in run(st, 2.0, Pose2D(3.0, 0.0, 0.0)))


def test_reset_goes_back_to_the_start():
    st = MoverState(spec(wander=None, route=[(2.0, 1.0)], seed=3))
    first = run(st, 5.0)
    st.reset()
    assert (st.pose.x, st.pose.y) == (0.0, 0.0)
    assert run(st, 5.0) == first


def test_counts_the_distance_walked_not_turned_or_waited():
    st = MoverState(spec(route=[(1.0, 0.0)], pause=1.0))
    run(st, 2.5)  # 1 m out, then waits
    assert st.walked == pytest.approx(1.0)
    run(st, 0.5)  # still waiting
    assert st.walked == pytest.approx(1.0)


def test_gait_swings_legs_and_arms_in_turn():
    assert all(a == 0.0 for a in gait_angles(0.3, 0.0).values())
    quarter = gait_angles(STRIDE / 4, 1.0)
    assert quarter["L_Thigh"] > 0 > quarter["R_Thigh"]
    assert quarter["L_Upperarm"] < 0 < quarter["R_Upperarm"]  # against the legs
    three = gait_angles(3 * STRIDE / 4, 1.0)
    assert three["L_Thigh"] == pytest.approx(-quarter["L_Thigh"])
    assert all(a <= 0.0 for t in (0.0, 0.2, 0.5, 0.9) for k, a in gait_angles(t, 1.0).items() if "Calf" in k)
    assert gait_angles(0.4, 0.5)["L_Thigh"] == pytest.approx(gait_angles(0.4, 1.0)["L_Thigh"] / 2)


def test_layout_motion_keys():
    lay = parse_layout({
        "world": "flat",
        "people": [[1.0, 1.0, 0.0],
                   {"pose": [0.0, 1.0, 0.0], "route": [[2, 1], [2, 2]], "speed": 0.9, "pause": 1}],
        "boxes": [{"name": "cart", "pose": [0, -1, 0], "size": [0.5, 0.4, 0.8], "wander": [[-1, 1], [-2, -1]],
                   "appear": [20, 10]},
                  {"name": "crate", "pose": [3, 3, 0], "size": [0.4, 0.4, 0.4]}],
    }, seed=1)
    assert lay.people_motion[0] is None
    m = lay.people_motion[1]
    assert (m.route, m.speed, m.pause, m.wander) == ([(2.0, 1.0), (2.0, 2.0)], 0.9, 1.0, None)
    cart, crate = lay.boxes
    assert cart.motion.wander == ((-1.0, 1.0), (-2.0, -1.0))
    assert cart.motion.appear == (20.0, 10.0)
    assert cart.motion.speed is None
    assert crate.motion is None


@pytest.mark.parametrize("entry, message", [
    ({"pose": [0, 0, 0], "route": [[1, 1]], "wander": [[0, 1], [0, 1]]}, "not both"),
    ({"pose": [0, 0, 0], "route": []}, "route"),
    ({"pose": [0, 0, 0], "wander": [[1, 0], [0, 1]]}, "lo <= hi"),
    ({"pose": [0, 0, 0], "route": [[1, 1]], "speed": 0}, "positive"),
    ({"pose": [0, 0, 0], "appear": [0, 1]}, "appear"),
    ({"pose": [0, 0, 0], "walk": [[1, 1]]}, "unknown"),
])
def test_layout_motion_errors(entry, message):
    with pytest.raises(LayoutError, match=message):
        parse_layout({"world": "flat", "people": [entry]}, seed=1)
