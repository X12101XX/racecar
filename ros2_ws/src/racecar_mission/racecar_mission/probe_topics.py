"""车上接口自检：列出话题/类型/频率，并给出本程序会选用哪一个。

    ros2 run racecar_mission probe_topics
    ros2 run racecar_mission probe_topics --seconds 5
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from .topics import detect_topics, list_topics


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args(argv)
    rclpy.init()
    node = Node("racecar_probe")
    prefer = {"scan": "/scan", "image": "/usb_cam/image_raw", "imu": "/imu/data",
              "odom": "/odom", "cmd_vel": "/crossfire/desired_cmd"}
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("\n=== 当前可用话题 ===")
    for name, types in sorted(list_topics(node).items()):
        print(f"  {name:<40s} {','.join(types)}")
    print("\n=== 本程序选用 ===")
    picked = detect_topics(node, prefer)
    for k, v in picked.items():
        print(f"  {k:<16s} {v}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
