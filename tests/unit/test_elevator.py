import math

import pytest

from kachaka_isaac_sim.config import load_config
from kachaka_isaac_sim.drive import NoisyOdometry, Pose2D
from kachaka_isaac_sim.elevator import (
    FLOOR_HEIGHT,
    FLOOR_LOOKS,
    FLOOR_SPACING,
    PLATFORM_THICKNESS,
    ElevatorSpec,
    ElevatorState,
    car_walls,
    elevator_spec,
    floor_look,
    floor_of,
    label_segments,
    logical_pose,
    platform_z,
    shifted,
    slab_tiles,
    stage_offset,
)
from kachaka_isaac_sim.floor_maps import floor_map
from kachaka_isaac_sim.layout import LayoutError, parse_layout
from kachaka_isaac_sim.maps import FREE, OCCUPIED
from kachaka_isaac_sim.scenes import ROOM_SIZE, ROOM_WALL, room_walls

POSE = Pose2D(2.525, 0.0, -math.pi / 2)  # doorway in the east wall, car outside the room


def spec(**kw):
    return ElevatorSpec(POSE, ["1F", "2F", "3F"], **kw)


# --- room walls ---------------------------------------------------------------
def test_room_walls_without_openings_are_the_four_walls():
    assert [w[0] for w in room_walls()] == ["wall_n", "wall_s", "wall_e", "wall_w"]


def test_room_walls_split_around_a_doorway():
    walls = {name: (c, s) for name, c, s in room_walls([(POSE.x, POSE.y, POSE.yaw, 0.9)])}
    assert "wall_e" not in walls and {"wall_e_a", "wall_e_b"} <= set(walls)
    (ca, sa), (cb, sb) = walls["wall_e_a"], walls["wall_e_b"]
    assert ca[1] + sa[1] / 2 == pytest.approx(-0.45)
    assert cb[1] - sb[1] / 2 == pytest.approx(0.45)
    assert sa[1] + sb[1] + 0.9 == pytest.approx(ROOM_SIZE[1])


# --- geometry -----------------------------------------------------------------
def test_floor_of_rounds_to_the_nearest_floor():
    assert [floor_of(x) for x in (-3.0, 0.0, 4.9, 5.1, 2 * FLOOR_SPACING)] == [0, 0, 0, 1, 2]


def test_inside_the_car_and_in_the_doorway():
    s = spec()
    car = Pose2D(POSE.x + 0.75, 0.0, 0.0)  # middle of the car on the first floor
    assert s.inside(car, 0, ROOM_WALL)
    assert not s.inside(car, 1, ROOM_WALL)
    assert s.inside(shifted(car, 1), 1, ROOM_WALL)
    assert not s.inside(Pose2D(2.0, 0.0, 0.0), 0, ROOM_WALL)  # in the room
    assert not s.in_doorway(car, 0, 0.23, ROOM_WALL)
    assert s.in_doorway(Pose2D(POSE.x - 0.2, 0.3, 0.0), 0, 0.23, ROOM_WALL)


def test_car_walls_close_the_car_but_the_doorway():
    s = spec()
    segs = car_walls(s, 1, ROOM_WALL)
    assert len(segs) == 5
    xs = [p[0] for seg in segs for p in seg]
    assert min(xs) == pytest.approx(POSE.x + FLOOR_SPACING)  # the front stands on the doorway
    assert max(xs) == pytest.approx(POSE.x + FLOOR_SPACING + ROOM_WALL + 1.4 + ROOM_WALL / 2)


def test_door_per_floor_open_where_the_car_starts():
    s = spec(start=1)
    doors = [s.door(k) for k in range(3)]
    assert [d.id for d in doors] == ["elevator_1F", "elevator_2F", "elevator_3F"]
    assert [d.open for d in doors] == [False, True, False]
    assert doors[2].pose.x == pytest.approx(POSE.x + 2 * FLOOR_SPACING)


def test_elevator_spec_takes_config_defaults():
    from kachaka_isaac_sim.layout import Elevator

    cfg = load_config().elevator
    s = elevator_spec(Elevator(POSE, None, None, None, "2F"), ["1F", "2F"], cfg)
    assert s.size == tuple(cfg.size) and s.door_width == cfg.door_width and s.ride == cfg.ride
    assert s.start == 1


# --- state --------------------------------------------------------------------
def run(st, door="closed", blocked=False, dt=0.5, n=1):
    out = []
    for _ in range(n):
        out += st.step(dt, door, blocked)
    return out


def test_go_rejects_unknown_floors_and_calls_while_riding():
    st = ElevatorState(spec(ride=2.0))
    assert st.go("B1")[0] is False
    assert st.go("1F") == (True, "at 1F")
    assert st.go("3F")[0]
    run(st, "open")
    run(st, "closed")
    assert st.phase == "riding"
    assert st.go("2F")[0] is False


def test_ride_closes_waits_per_floor_arrives_and_opens():
    st = ElevatorState(spec(ride=2.0))
    st.go("3F")
    assert run(st, "open") == [("door", 0, False)]
    assert st.phase == "closing"
    assert run(st, "closing") == []
    assert run(st, "closed") == [] and st.phase == "riding"
    assert run(st, n=7) == []  # two floors: 4 s
    assert run(st) == [("arrive", 0, 2), ("door", 2, True)]
    assert (st.floor, st.phase) == (2, "opening")
    run(st, "opening")
    assert st.phase == "opening"
    run(st, "open")
    assert st.phase == "open"


def test_door_reopens_while_blocked_and_waits():
    st = ElevatorState(spec())
    st.go("2F")
    run(st, "open")
    assert run(st, "closing", blocked=True) == [("door", 0, True)]
    assert run(st, "open", blocked=True, n=3) == []  # held: no repeats
    assert st.phase == "closing"
    assert run(st, "open") == [("door", 0, False)]
    run(st, "closed")
    assert st.phase == "riding"


def test_called_back_before_leaving_reopens():
    st = ElevatorState(spec())
    st.go("2F")
    run(st, "open")
    st.go("1F")
    assert run(st, "closing") == [("door", 0, True)]
    assert st.phase == "opening"


def test_reset_back_to_start():
    st = ElevatorState(spec(start=1, ride=0.5))
    st.go("1F")
    run(st, "open")
    run(st, "closed", n=3)
    assert st.floor == 0
    st.reset()
    assert (st.floor, st.target, st.phase) == (1, 1, "open")


# --- layout -------------------------------------------------------------------
FLOORS = {"world": "room", "floors": [{"name": "1F", "walls": [[0, 0, 1, 0]]},
                                      {"name": "2F", "walls": [[0, 1, 1, 1]], "people": [[-1.0, 0.5, 0.0]],
                                       "boxes": [{"name": "desk", "pose": [-1.0, 1.0, 0.0], "size": [1, 0.5, 0.7]}]}],
          "elevator": {"pose": [2.525, 0.0, -1.5708], "ride": 3}}


def test_layout_floors_are_moved_to_their_place():
    lay = parse_layout(FLOORS, world="room")
    assert lay.floors == ["1F", "2F"]
    assert lay.elevator.ride == 3.0 and lay.elevator.size is None
    assert lay.walls == [(0, 0, 1, 0), (FLOOR_SPACING, 1, FLOOR_SPACING + 1, 1)]
    assert [(b.name, b.pose.x) for b in lay.boxes] == [("desk_f2", FLOOR_SPACING - 1.0)]
    assert [(p.x, p.y) for p in lay.floor_people] == [(FLOOR_SPACING - 1.0, 0.5)]


@pytest.mark.parametrize("data, msg", [
    ({"floors": [{"name": "1F"}, {"name": "2F"}]}, "elevator"),
    ({"floors": [{"name": "1F"}], "elevator": {"pose": [0, 0, 0]}}, "two floors"),
    ({"floors": [{"name": "1F"}, {"name": "1F"}], "elevator": {"pose": [0, 0, 0]}}, "1F"),
    ({"floors": [{"name": "1F"}, {"name": "2F", "shelves": []}], "elevator": {"pose": [0, 0, 0]}}, "shelves"),
    ({"floors": [{"name": "1F"}, {"name": "2F"}], "elevator": {"pose": [0, 0, 0], "start": "B1"}}, "B1"),
    ({"floors": [{"name": "1F"}, {"name": "2F"}], "elevator": {}}, "pose"),
])
def test_layout_floor_errors(data, msg):
    with pytest.raises(LayoutError, match=msg):
        parse_layout({"world": "room", **data}, world="room")


def test_layout_floors_need_the_room():
    with pytest.raises(LayoutError, match="room"):
        parse_layout(FLOORS, world="empty")


# --- maps ---------------------------------------------------------------------
def cell(m, x, y):
    i = int((x - m.origin[0]) / m.resolution)
    j = int((y - m.origin[1]) / m.resolution)
    return m.grid[j, i]


@pytest.mark.parametrize("k", [0, 1])
def test_floor_map_has_the_doorway_open_and_the_car_closed(k):
    lay = parse_layout(FLOORS, world="room")
    s = elevator_spec(lay.elevator, lay.floors, load_config().elevator)
    spawn = Pose2D(0.0, 0.0, 0.0)
    m = floor_map(lay, k, s, spawn, charger=Pose2D(-2.0, 0.0, 0.0))
    assert m.name == lay.floors[k]
    assert (m.charger_pose is not None) == (k == 0)
    # Map frame = spawn moved to the floor: the same poses on every floor.
    assert cell(m, 1.0, -1.5) == FREE
    assert cell(m, POSE.x, 0.0) == FREE  # doorway
    assert cell(m, POSE.x, 0.7) == OCCUPIED  # wall beside it
    assert cell(m, POSE.x + 0.75, 0.0) == FREE  # inside the car
    assert cell(m, POSE.x + ROOM_WALL + 1.4 + ROOM_WALL / 2, 0.0) == OCCUPIED  # back of the car
    assert cell(m, 0.5, 1.0) == (OCCUPIED if k == 1 else FREE)  # 2F's wall
    assert cell(m, 0.5, 0.0) == (OCCUPIED if k == 0 else FREE)  # 1F's wall


# --- odometry -----------------------------------------------------------------
def test_noisy_odometry_does_not_see_a_shift():
    odom = NoisyOdometry(0.0, 0.0, seed=1)
    odom.update(Pose2D(1.0, 2.0, 0.3))
    before = odom.update(Pose2D(1.5, 2.0, 0.3))
    odom.shift(FLOOR_SPACING, 0.0)
    after = odom.update(Pose2D(1.5 + FLOOR_SPACING, 2.0, 0.3))
    assert (after.x, after.y, after.yaw) == pytest.approx((before.x, before.y, before.yaw))


def test_floor_label_segments():
    one = label_segments("1F", (0.0, 0.0), height=1.0)
    assert len(one) == 2 + 4  # "1": b, c; "F": a, e, f, g
    two = label_segments("2F", (0.0, 0.0), height=1.0)
    assert len(two) == 5 + 4
    for x0, y0, x1, y1 in one + two:
        assert 0.0 <= x0 < x1 <= 1.2 and 0.0 <= y0 < y1 <= 1.0
    # "F" starts a character later, after "1"'s space.
    assert min(r[0] for r in one[2:]) == pytest.approx(0.7)
    assert label_segments("?", (0.0, 0.0)) == []


def test_floors_look_different():
    assert floor_look(0) != floor_look(1)
    assert floor_look(0) == floor_look(len(FLOOR_LOOKS))


# --- stacked floors -----------------------------------------------------------
def test_stage_offset_lifts_upper_floors_over_the_first():
    assert stage_offset(0, "vertical") == (0.0, 0.0, 0.0)
    assert stage_offset(2, "side") == (0.0, 0.0, 0.0)
    assert stage_offset(2, "vertical") == (-2 * FLOOR_SPACING, 0.0, 2 * FLOOR_HEIGHT)


def test_logical_pose_takes_the_floor_from_the_height():
    p = Pose2D(1.0, 0.5, 0.3)
    assert logical_pose(p, 0.0, "vertical", 3) == p
    assert logical_pose(p, FLOOR_HEIGHT + 0.01, "vertical", 3) == shifted(p, 1)
    assert logical_pose(p, 0.4 * FLOOR_HEIGHT, "vertical", 3) == p  # still nearer the first
    assert logical_pose(p, 9 * FLOOR_HEIGHT, "vertical", 3) == shifted(p, 2)
    assert logical_pose(p, FLOOR_HEIGHT, "side", 3) == p


def test_slab_tiles_cover_all_but_the_hole():
    outer, hole = (0.0, 0.0, 4.0, 3.0), (1.0, 1.0, 2.0, 2.0)
    tiles = slab_tiles(outer, hole)
    assert sum((x1 - x0) * (y1 - y0) for x0, y0, x1, y1 in tiles) == pytest.approx(12.0 - 1.0)
    for x, y in [(0.5, 0.5), (3.5, 2.5), (1.5, 0.5), (2.5, 1.5)]:
        assert any(x0 < x < x1 and y0 < y < y1 for x0, y0, x1, y1 in tiles)
    assert not any(x0 < 1.5 < x1 and y0 < 1.5 < y1 for x0, y0, x1, y1 in tiles)
    assert slab_tiles(outer, (5.0, 5.0, 6.0, 6.0)) == [outer]
    assert len(slab_tiles(outer, (0.0, 1.0, 1.0, 3.0))) == 2  # at a corner


def test_inside_rect_is_the_car_inside():
    s = spec()
    x0, y0, x1, y1 = s.inside_rect(0, ROOM_WALL)
    w, d = s.size
    # Yaw -90 degrees: the car lies along +x from the doorway.
    assert (x0, x1) == pytest.approx((POSE.x + ROOM_WALL / 2, POSE.x + ROOM_WALL / 2 + d))
    assert (y0, y1) == pytest.approx((-w / 2, w / 2))
    assert s.inside(Pose2D((x0 + x1) / 2, 0.0, 0.0), 0, ROOM_WALL)
    assert s.inside_rect(1, ROOM_WALL)[0] == pytest.approx(x0 + FLOOR_SPACING)
    assert s.outer_rect(0, ROOM_WALL) == pytest.approx((x0 - ROOM_WALL, y0 - ROOM_WALL, x1 + ROOM_WALL,
                                                        y1 + ROOM_WALL))


def test_position_eases_between_floors_while_riding():
    st = ElevatorState(spec(ride=2.0))
    assert st.position == 0.0
    st.go("3F")
    run(st, "open")
    run(st, "closed")
    assert st.phase == "riding" and st.position == 0.0
    seen = []
    while st.phase == "riding":
        run(st)
        seen.append(st.position)
    assert seen == sorted(seen) and seen[-1] == 2.0
    assert 0.9 < seen[len(seen) // 2 - 1] < 1.1  # halfway at half time
    assert platform_z(st.position) == pytest.approx(2 * FLOOR_HEIGHT + 0.001 - PLATFORM_THICKNESS / 2)


def test_layout_vertical_stack():
    lay = parse_layout({**FLOORS, "elevator": {**FLOORS["elevator"], "stack": "vertical"}}, world="room")
    assert lay.elevator.stack == "vertical"
    assert lay.walls[1] == (FLOOR_SPACING, 1, FLOOR_SPACING + 1, 1)  # still laid out side by side
    assert parse_layout(FLOORS, world="room").elevator.stack == "side"
    assert elevator_spec(lay.elevator, lay.floors, load_config().elevator).stack == "vertical"


@pytest.mark.parametrize("elevator, floor2, msg", [
    ({"stack": "up"}, {}, "stack"),
    ({"stack": "vertical", "pose": [2.525, 0.0, 0.5]}, {}, "90 degrees"),
    ({"stack": "vertical"}, {"doors": [{"pose": [0.0, 0.0, 0.0]}]}, "first floor"),
    ({"stack": "vertical"}, {"people": [{"pose": [0.0, 0.0, 0.0], "route": [[1.0, 0.0]]}]}, "first floor"),
])
def test_layout_vertical_stack_errors(elevator, floor2, msg):
    data = {"world": "room", "floors": [{"name": "1F"}, {"name": "2F", **floor2}],
            "elevator": {"pose": [2.525, 0.0, -1.5708], **elevator}}
    with pytest.raises(LayoutError, match=msg):
        parse_layout(data, world="room")
