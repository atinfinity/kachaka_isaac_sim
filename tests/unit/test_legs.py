import math

import pytest

from kachaka_isaac_sim.legs import (
    HIP_Y,
    HIP_Z,
    LEG_RADIUS,
    LIDAR_Z,
    capsule_length,
    leg_capsules,
    leg_points,
)
from kachaka_isaac_sim.movers import STRIDE, gait_angles


def ends(name, center, pitch, height=1.7):
    """Bottom and top of a capsule's axis, (x, z)."""
    half = capsule_length(name, height) / 2
    ux, uz = math.sin(pitch), math.cos(pitch)
    return (center[0] - half * ux, center[2] - half * uz), (center[0] + half * ux, center[2] + half * uz)


def test_standing_legs_reach_the_floor_under_the_hips():
    caps = {n: (c, p) for n, c, p in leg_capsules({})}
    assert set(caps) == {"thigh_l", "shin_l", "thigh_r", "shin_r"}
    for side, y in (("l", HIP_Y), ("r", -HIP_Y)):
        (tc, tp), (sc, sp) = caps[f"thigh_{side}"], caps[f"shin_{side}"]
        assert tp == sp == 0.0 and tc[1] == sc[1] == y
        assert ends(f"thigh_{side}", tc, tp)[1] == pytest.approx((0.0, HIP_Z))
        assert ends(f"shin_{side}", sc, sp)[1] == pytest.approx(ends(f"thigh_{side}", tc, tp)[0])
        assert ends(f"shin_{side}", sc, sp)[0][1] - LEG_RADIUS == pytest.approx(0.0)  # the cap on the floor


def test_legs_stay_jointed_with_a_foot_on_the_floor_while_walking():
    for walked in (0.1, 0.33, 0.4, 0.9, 1.0):
        caps = {n: (c, p) for n, c, p in leg_capsules(gait_angles(walked, 1.0))}
        hips = []
        for side in "lr":
            knee = ends(f"thigh_{side}", *caps[f"thigh_{side}"])[0]
            assert ends(f"shin_{side}", *caps[f"shin_{side}"])[1] == pytest.approx(knee)
            hips.append(ends(f"thigh_{side}", *caps[f"thigh_{side}"])[1])
        assert hips[0] == pytest.approx(hips[1]) and hips[0][0] == pytest.approx(0.0) and hips[0][1] <= HIP_Z
        feet = [ends(f"shin_{side}", *caps[f"shin_{side}"])[0][1] for side in "lr"]
        assert min(feet) == pytest.approx(LEG_RADIUS)


def test_a_forward_hip_swings_the_leg_forward():
    caps = {n: (c, p) for n, c, p in leg_capsules({"L_Thigh": 0.3})}
    assert caps["shin_l"][0][0] > 0.05 and caps["shin_r"][0][0] == 0.0


def test_scan_cuts_two_legs_that_swap_front_and_back():
    standing = leg_points({})
    assert standing == [pytest.approx((0.0, HIP_Y)), pytest.approx((0.0, -HIP_Y))]
    (l1, _), (r1, _) = leg_points(gait_angles(STRIDE / 4, 1.0))
    (l2, _), (r2, _) = leg_points(gait_angles(3 * STRIDE / 4, 1.0))
    assert l1 > 0.1 > -0.1 > r1  # left leg forward
    assert r2 > 0.1 > -0.1 > l2  # then the right one


def test_legs_scale_with_height():
    small = {n: c for n, c, _ in leg_capsules({}, 0.85)}
    full = {n: c for n, c, _ in leg_capsules({})}
    assert small["thigh_l"] == pytest.approx(tuple(v / 2 for v in full["thigh_l"]))
    assert len(leg_points({}, 0.85, LIDAR_Z)) == 2
