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
