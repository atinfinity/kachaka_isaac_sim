from kachaka_isaac_sim.wall_clock import WallClock

S = 1_000_000_000


def test_passthrough_before_first_tick():
    assert WallClock().to_wall(5 * S) == 5 * S


def test_maps_with_the_tick_at_or_before():
    c = WallClock()
    c.record(1 * S, 1000 * S)
    c.record(2 * S, 1003 * S)  # sim ran slower than real time
    assert c.to_wall(1 * S) == 1000 * S
    assert c.to_wall(1 * S + 500) == 1000 * S + 500
    assert c.to_wall(2 * S) == 1003 * S
    assert c.to_wall(3 * S) == 1004 * S
    assert c.to_wall(0) == 999 * S  # before the history: the oldest offset


def test_same_stamp_same_time_after_more_ticks():
    c = WallClock()
    c.record(1 * S, 1000 * S)
    before = c.to_wall(1 * S + 7)
    c.record(2 * S, 1010 * S)
    assert c.to_wall(1 * S + 7) == before


def test_monotonic_in_sim_time():
    c = WallClock()
    for i in range(100):
        c.record(i * S // 60, 10**18 + i * S // 30)
    walls = [c.to_wall(t) for t in range(0, 2 * S, S // 97)]
    assert walls == sorted(walls)


def test_sim_time_going_back_restarts():
    c = WallClock()
    c.record(10 * S, 1000 * S)
    c.record(1 * S, 2000 * S)
    assert c.to_wall(1 * S) == 2000 * S


def test_history_is_bounded():
    c = WallClock(keep=10)
    for i in range(1, 100):
        c.record(i, i * 2)
    assert len(c.sim) <= 20 and c.to_wall(99) == 198
