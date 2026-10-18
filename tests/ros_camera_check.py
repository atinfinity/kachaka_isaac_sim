"""Save one frame from each camera and print rates. Runs in a ROS 2 Jazzy
environment, not in the Isaac Sim venv. Writes PNG/PGM files to argv[1]."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
names = ["front_camera", "back_camera", "tof_camera"]
rclpy.init()
n = Node("camera_check")
last, stamps, infos = {}, {k: [] for k in names}, {}
for k in names:
    def cb(m, k=k):
        last[k] = m
        stamps[k].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    n.create_subscription(Image, f"/kachaka/{k}/image_raw", cb, qos_profile_sensor_data)
    n.create_subscription(CameraInfo, f"/kachaka/{k}/camera_info", lambda m, k=k: infos.__setitem__(k, m),
                          qos_profile_sensor_data)
end = time.time() + 60
while (len(last) < len(names) or len(infos) < len(names)) and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
for k in names:
    stamps[k].clear()
end = time.time() + 5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
for k in names:
    if k not in last:
        print(k, "MISSING")
        continue
    m, s = last[k], stamps[k]
    rate = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 1 else 0
    print(f"{k:13s} {m.width}x{m.height} {m.encoding} frame={m.header.frame_id} rate={rate:.1f} Hz "
          f"fx={infos[k].k[0]:.1f}")
    if m.encoding == "rgb8":
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
        with open(f"{out}/{k}.ppm", "wb") as f:
            f.write(f"P6 {m.width} {m.height} 255\n".encode() + a.tobytes())
        print(f"   mean rgb {a.reshape(-1, 3).mean(0).round(1)}")
    else:
        a = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width)
        valid = a[a > 0]
        print(f"   valid {valid.size}/{a.size} depth mm min/median/max "
              f"{valid.min() if valid.size else 0}/{int(np.median(valid)) if valid.size else 0}/{valid.max() if valid.size else 0}")
        img = (255 - np.clip(a / 2000.0 * 255, 0, 255)).astype(np.uint8) * (a > 0)
        with open(f"{out}/{k}.pgm", "wb") as f:
            f.write(f"P5 {m.width} {m.height} 255\n".encode() + img.astype(np.uint8).tobytes())
