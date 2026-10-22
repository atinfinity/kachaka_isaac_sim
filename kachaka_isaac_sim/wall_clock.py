"""Sim time -> wall clock, for the stamps of kachaka-api.

The ROS side runs on simulation time (/clock). The real robot's kachaka-api
stamps are wall clock: the demo notebooks compare them with time.time() (for
example python/demos/save_object_detection_features.ipynb records until
`header.stamp_nsec` passes a time.time() deadline). So the API server turns
each sim stamp into wall clock time.

Every /clock tick records (sim time, wall time). A stamp s maps with the
offset of the newest tick at or before s: wall = s + (w_i - s_i). The same
stamp always maps to the same time, in every stream, and stamps keep their
order and their spacing (sim seconds) between two ticks. Before the first
tick, stamps pass through unchanged.
"""

from __future__ import annotations

import bisect
import threading
import time

KEEP = 4096  # ticks (about a minute at 60 Hz); older stamps use the oldest offset


class WallClock:
    def __init__(self, keep: int = KEEP):
        self.keep = keep
        self.lock = threading.Lock()
        self.sim: list[int] = []
        self.offset: list[int] = []  # wall - sim [ns]

    def record(self, sim_ns: int, wall_ns: int | None = None) -> None:
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        with self.lock:
            if self.sim and sim_ns <= self.sim[-1]:
                if sim_ns == self.sim[-1]:
                    return
                self.sim.clear()  # sim time went back (a restarted timeline)
                self.offset.clear()
            self.sim.append(sim_ns)
            self.offset.append(wall_ns - sim_ns)
            if len(self.sim) > 2 * self.keep:
                del self.sim[:-self.keep], self.offset[:-self.keep]

    def to_wall(self, sim_ns: int) -> int:
        with self.lock:
            if not self.sim:
                return sim_ns
            i = max(bisect.bisect_right(self.sim, sim_ns) - 1, 0)
            return sim_ns + self.offset[i]


CLOCK = WallClock()  # fed by ros_node.SimNode.publish_clock, read by api_server
