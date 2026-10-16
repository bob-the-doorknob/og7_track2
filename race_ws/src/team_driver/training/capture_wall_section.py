"""Passively record pose, scans and existing commands for wall-exit analysis."""

import argparse
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seconds", type=float, default=25.0)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    rclpy.init(args=[])
    node = Node("wall_section_observer")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.VOLATILE)
    state = dict(pose=None, odom_time=None, steering=0.0, throttle=0.0)
    samples, scans = [], []
    scan_angles = None
    start = time.monotonic()

    def odom(msg):
        q = msg.pose.pose.orientation
        state["pose"] = (
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
            math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))
        state["odom_time"] = time.monotonic()

    def scan(msg):
        nonlocal scan_angles
        if state["pose"] is None:
            return
        now = time.monotonic()
        scans.append(np.asarray(msg.ranges, dtype=np.float32))
        samples.append((now-start, *state["pose"], state["steering"],
                        state["throttle"], now-state["odom_time"]))
        scan_angles = msg.angle_min + np.arange(len(msg.ranges))*msg.angle_increment

    ns = "/autodrive/roboracer_1"
    node.create_subscription(Odometry, ns+"/odom", odom, qos)
    node.create_subscription(LaserScan, ns+"/lidar", scan, qos)
    node.create_subscription(Float32, ns+"/steering_command",
                             lambda msg: state.update(steering=msg.data), qos)
    node.create_subscription(Float32, ns+"/throttle_command",
                             lambda msg: state.update(throttle=msg.data), qos)
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not samples:
        raise RuntimeError("no paired LiDAR/odometry samples received")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as handle:
        np.savez_compressed(handle, samples=np.asarray(samples),
                            scans=np.asarray(scans), angles=scan_angles,
                            columns=np.asarray(("elapsed_s", "x_m", "y_m", "yaw_rad",
                                                "speed_mps", "steering", "throttle", "odom_age_s")))
    print(f"Saved {len(samples)} passive samples to {args.output}")


if __name__ == "__main__":
    main()
