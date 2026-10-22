"""A person's legs as colliders: a thigh and a shin capsule per leg, posed by
the hip and knee angles of the walking pose (movers.gait_angles). Plain
Python, no Isaac Sim imports; world.add_person makes the capsules and
movers.Movers moves them.

The LiDAR scans 0.105 m above the floor (kachaka_description
`_kachaka.urdf.xacro`, `lidar_joint`), where it cuts the shins: a person shows
as two small arcs that swap front and back as they walk. Sizes are for a
1.7 m person and scale with the height; they follow the mannequin's capsule
legs (world.add_mannequin) and are assumed, not measured on people.
"""

from __future__ import annotations

import math

HIP_Z = 0.9  # m, hip joint above the floor, standing
HIP_Y = 0.1  # m, from the middle to each hip
LEG_RADIUS = 0.065  # m: legs 0.13 m across (assumed)
THIGH = 0.43  # m, hip to knee
SHIN = 0.835 - THIGH  # m, knee to the bottom of the shin capsule's cylinder: its cap reaches the floor
# The upper body: an invisible box from here to the top of the head, as wide
# as the shoulders and arms (scenes.PERSON_COLLIDER), over the thighs.
TORSO_Z = 0.75  # m

LEGS = (("l", HIP_Y, "L_Thigh", "L_Calf"), ("r", -HIP_Y, "R_Thigh", "R_Calf"))


def leg_capsules(angles: dict[str, float], height: float = 1.7) -> list[tuple[str, tuple[float, float, float], float]]:
    """(name, centre (x, y, z), pitch [rad]) of thigh_l, shin_l, thigh_r and
    shin_r in the person's frame (x forward, z up, feet at z = 0) for joint
    `angles` (rad forward, as gait_angles; missing ones stand). Each capsule
    runs along z, turned about y by the pitch. The hips sit at HIP_Z when
    standing and lower while walking, keeping a foot on the floor."""
    k = height / 1.7
    legs = []
    for side, y, hip, knee in LEGS:
        a = angles.get(hip, 0.0)
        b = a + angles.get(knee, 0.0)
        kx, kz = THIGH * math.sin(a), -THIGH * math.cos(a)  # from the hip
        fx, fz = kx + SHIN * math.sin(b), kz - SHIN * math.cos(b)
        legs.append((side, y, a, b, kx, kz, fx, fz))
    # The hips drop as the legs swing, so that the lower foot stays on the floor.
    hz = LEG_RADIUS - min(leg[7] for leg in legs)
    out = []
    for side, y, a, b, kx, kz, fx, fz in legs:
        # A capsule along +z turned by -a about y points (-sin a, cos a): up the leg.
        out.append((f"thigh_{side}", (kx / 2 * k, y * k, (hz + kz / 2) * k), -a))
        out.append((f"shin_{side}", ((kx + fx) / 2 * k, y * k, (hz + (kz + fz) / 2) * k), -b))
    return out


def capsule_length(name: str, height: float = 1.7) -> float:
    """Length of the capsule's cylinder part (without the caps)."""
    return (THIGH if name.startswith("thigh") else SHIN) * height / 1.7


LIDAR_Z = 0.105  # m, the LiDAR's scan height above the floor (see above)


def leg_points(angles: dict[str, float], height: float = 1.7, z: float = LIDAR_Z) -> list[tuple[float, float]]:
    """(x, y) in the person's frame where each leg's axis crosses height `z`
    (left, then right): the middle of the circle a scan at `z` cuts out. A
    leg whose shin is lifted above `z` crosses at its thigh, or is left out."""
    caps = leg_capsules(angles, height)
    out = []
    for i in range(0, len(caps), 2):
        for name, (cx, cy, cz), pitch in (caps[i + 1], caps[i]):  # the shin first
            half = capsule_length(name, height) / 2
            dz = half * math.cos(pitch)  # the axis runs up along (sin pitch, cos pitch) in x, z
            if dz > 0 and abs(z - cz) <= dz:
                t = (z - cz) / dz * half
                out.append((cx + t * math.sin(pitch), cy))
                break
    return out
