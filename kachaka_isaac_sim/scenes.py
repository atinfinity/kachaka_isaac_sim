"""Isaac Sim scenes the simulator knows by name. No Isaac imports, so the
command line can use it before Isaac Sim starts."""

# Indoor scenes shipped with Isaac Sim (streamed from the Isaac asset server).
ISAAC_WORLDS = {
    "office": "Isaac/Environments/Office/office.usd",
    "hospital": "Isaac/Environments/Hospital/hospital.usd",
    "warehouse": "Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "simple_room": "Isaac/Environments/Simple_Room/simple_room.usd",
}
# Default spawn pose (x, y, yaw) per world: on the floor, in free space, with
# room to turn. Checked with SLAM and Nav2 (tests/ros_nav_check.py --world).
SPAWN = {
    "office": (0.0, 0.0, 1.5708),  # lobby, facing the corridors to the north
    "hospital": (0.0, 0.0, 0.0),  # lobby south of the ward blocks
    "warehouse": (-13.0, 11.0, 1.5708),  # in an aisle, rack legs on both sides
    "simple_room": (-2.5, 0.0, 0.0),  # beside the table, not under it
}

# The built-in 'room' world (world.add_simple_room; scripts/fake_sim.py draws
# the same room in 2D): an inner floor of ROOM_SIZE with walls of
# ROOM_WALL thickness, and static boxes as (name, center xyz, size xyz, color).
ROOM_SIZE = (5.0, 4.0)  # m, x and y
ROOM_HEIGHT = 2.4
ROOM_WALL = 0.05
ROOM_BOXES = (
    ("box_a", (1.4, 1.0, 0.25), (0.4, 0.4, 0.5), (0.55, 0.4, 0.3)),
    ("box_b", (-1.2, -0.9, 0.35), (0.6, 0.3, 0.7), (0.3, 0.45, 0.6)),
    ("pillar", (-0.6, 1.1, 0.4), (0.2, 0.2, 0.8), (0.6, 0.6, 0.6)),
)
# A person's upper-body collider (world.add_person, over the leg capsules of
# legs.py) and footprint, x and y for 1.7 m height.
PERSON_COLLIDER = (0.28, 0.5)

# Kachaka Shelf colliders, measured from the 3-layer mesh (metres, shelf frame,
# z from the floor). The robot drives in along x between the caster rows at
# y = +-0.195, under the bottom plate.
SHELF_LEG_XY = (0.131, 0.195)
SHELF_LEG_SIZE = (0.012, 0.046)
# The bottom plate's collider starts at 0.107 m, not the mesh's 0.104 m: the
# LiDAR scans at 0.1049 m and sees only the legs under a shelf, as the real
# robot does, so a shelf is not mapped as a closed box the robot cannot leave.
SHELF_PLATES = ((0.107, 0.121), (0.435, 0.452), (0.649, 0.666))  # bottom, middle, top (z ranges)
SHELF_SIZE = (0.328, 0.488)  # x (width the robot drives across), y
SHELF_CASTER_RADIUS = 0.03
