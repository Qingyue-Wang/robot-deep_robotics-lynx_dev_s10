#!/usr/bin/env python3
"""轻量 ROS 探针：不依赖 ros2 CLI 守护进程，直接 rclpy 统计话题频率。"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

TOPICS = [
    ("/camera_435i/color/image_raw", "sensor_msgs/msg/Image", qos_profile_sensor_data),
    ("/arm/joint_states_single", "sensor_msgs/msg/JointState", 10),
    ("/odom", "nav_msgs/msg/Odometry", 10),
    ("/scan", "sensor_msgs/msg/LaserScan", qos_profile_sensor_data),
]

TYPES = {
    "sensor_msgs/msg/Image": __import__("sensor_msgs.msg", fromlist=["Image"]).Image,
    "sensor_msgs/msg/JointState": __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState,
    "nav_msgs/msg/Odometry": __import__("nav_msgs.msg", fromlist=["Odometry"]).Odometry,
    "sensor_msgs/msg/LaserScan": __import__("sensor_msgs.msg", fromlist=["LaserScan"]).LaserScan,
}

rclpy.init()
node = Node("probe_hz")
counts = {}
first_last = {}


def make_cb(name):
    def cb(msg):
        counts[name] = counts.get(name, 0) + 1
        now = time.monotonic()
        if name not in first_last:
            first_last[name] = [now, now]
        else:
            first_last[name][1] = now
    return cb


for topic, typ, qos in TOPICS:
    node.create_subscription(TYPES[typ], topic, make_cb(topic), qos)

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
end = time.monotonic() + duration
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)

for topic, _, _ in TOPICS:
    n = counts.get(topic, 0)
    if n >= 2:
        span = first_last[topic][1] - first_last[topic][0]
        hz = (n - 1) / span if span > 0 else 0.0
        print(f"{topic}: {n} msgs  {hz:.2f} Hz")
    else:
        print(f"{topic}: {n} msgs  (无数据)")
node.destroy_node()
rclpy.shutdown()
