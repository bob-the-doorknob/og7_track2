#!/usr/bin/env python3
"""Track 2 ROS driver using a centreline pure pursuit baseline."""

import math
import time
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from team_driver.pure_pursuit import PurePursuit
from team_driver.safety import LidarSafety
from team_driver.speed_controller import SpeedController

# The steering command is normalised; this is what +-1.0 actually means.
MAX_STEERING_RAD = 0.5236


def devkit_qos() -> QoSProfile:
    """Match the QoS the AutoDRIVE bridge publishes and subscribes with.

    RELIABLE / KEEP_LAST(1) / VOLATILE. Get this wrong and the topics simply do
    not connect: a BEST_EFFORT subscriber will never see a RELIABLE publisher's
    messages, and `ros2 topic list` will still cheerfully show the topic.
    """
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        # config/driver_params.yaml supplies values during evaluation.
        self.declare_parameter('vehicle_ns', '/autodrive/roboracer_1')
        self.declare_parameter('max_range', 10.0)         # [m] clip the scan here

        self.declare_parameter('centerline_path', '')
        self.declare_parameter('target_speed', 1.8)
        self.declare_parameter('max_lateral_accel', 1.5)
        self.declare_parameter('wheelbase', 0.324)
        self.declare_parameter('lookahead_min', 0.55)
        self.declare_parameter('lookahead_max', 1.8)
        self.declare_parameter('lookahead_speed_gain', 0.35)
        self.declare_parameter('speed_ff_gain', 0.16)
        self.declare_parameter('speed_kp', 0.24)
        self.declare_parameter('speed_ki', 0.08)
        self.declare_parameter('baseline_only', True)
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('safety_side_margin', 0.08)
        self.declare_parameter('safety_stop_margin', 0.25)
        self.declare_parameter('safety_max_deceleration', 2.0)
        self.declare_parameter('safety_horizon', 3.0)

        self.max_range = float(self.get_parameter('max_range').value)
        self.target_speed = max(float(self.get_parameter('target_speed').value), 0.0)
        self.max_lateral_accel = max(
            float(self.get_parameter('max_lateral_accel').value), 0.1
        )
        self.baseline_only = bool(self.get_parameter('baseline_only').value)
        self.safety_enabled = bool(self.get_parameter('safety_enabled').value)
        configured_path = str(self.get_parameter('centerline_path').value).strip()
        if configured_path:
            centerline_path = Path(configured_path)
        else:
            try:
                centerline_path = Path(get_package_share_directory('team_driver')) \
                    / 'maps' / 'icra26_compete_centerline.csv'
            except Exception:
                centerline_path = Path(__file__).resolve().parents[4] / 'maps' \
                    / 'icra26_compete_centerline.csv'

        self.controller = PurePursuit(
            centerline_path=centerline_path,
            wheelbase=float(self.get_parameter('wheelbase').value),
            max_steering_rad=MAX_STEERING_RAD,
            lookahead_min=float(self.get_parameter('lookahead_min').value),
            lookahead_max=float(self.get_parameter('lookahead_max').value),
            lookahead_speed_gain=float(
                self.get_parameter('lookahead_speed_gain').value
            ),
        )
        self.speed_controller = SpeedController(
            feedforward_gain=float(self.get_parameter('speed_ff_gain').value),
            kp=float(self.get_parameter('speed_kp').value),
            ki=float(self.get_parameter('speed_ki').value),
        )
        self.safety = LidarSafety(
            wheelbase=float(self.get_parameter('wheelbase').value),
            max_steering_rad=MAX_STEERING_RAD,
            side_margin=float(self.get_parameter('safety_side_margin').value),
            stop_margin=float(self.get_parameter('safety_stop_margin').value),
            max_deceleration=float(
                self.get_parameter('safety_max_deceleration').value
            ),
            horizon=float(self.get_parameter('safety_horizon').value),
            max_range=self.max_range,
        )
        self.get_logger().info(
            f'Loaded {len(self.controller.points)} centreline points from '
            f'{centerline_path}; target speed {self.target_speed:.2f} m/s'
        )

        ns = str(self.get_parameter('vehicle_ns').value).rstrip('/')
        qos = devkit_qos()

        self.throttle_pub = self.create_publisher(Float32, f'{ns}/throttle_command', qos)
        self.steering_pub = self.create_publisher(Float32, f'{ns}/steering_command', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/driver/markers', 1)

        self.create_subscription(LaserScan, f'{ns}/lidar', self.scan_callback, qos)
        self.create_subscription(Odometry, f'{ns}/odom', self.odom_callback, qos)
        self.create_subscription(Point, f'{ns}/ips', self.ips_callback, qos)

        # Latest known pose and speed. Ground truth from the simulator, which
        # the hackathon rules allow you to use - so use it.
        self.position = None      # (x, y) in the `world` frame
        self.yaw = 0.0            # [rad]
        self.speed = 0.0          # [m/s]
        self.last_odom_time = None
        self.last_control_time = None
        self.last_safety_warning = 0.0

        self._marker_divisor = 0
        if not self.baseline_only:
            self.get_logger().warn(
                'baseline_only is false, but no residual policy is installed yet; '
                'running the classical baseline.'
            )

    # ------------------------------------------------------------------
    # Where the car is. Free, accurate, and worth building on.
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.last_odom_time = time.monotonic()

    def ips_callback(self, msg):
        """Position alone, at the same rate. /odom carries this plus velocity."""
        self.position = (msg.x, msg.y)

    # ------------------------------------------------------------------
    # The control loop, once per LiDAR scan (40 Hz).
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        now = time.monotonic()
        dt = 0.025 if self.last_control_time is None else now - self.last_control_time
        self.last_control_time = now
        ranges, angles = self.preprocess(scan)
        steering, throttle = self.plan(ranges, angles, dt, scan.ranges)
        self.publish(steering, throttle)

        self._marker_divisor = (self._marker_divisor + 1) % 10
        if self._marker_divisor == 0:
            self.publish_marker(steering * MAX_STEERING_RAD)

    def preprocess(self, scan):
        """Turn a raw scan into clean ranges plus the angle of each beam.

        Kept because every approach needs some version of it and the details
        are fiddly rather than interesting: the LiDAR reports NaN and inf, and
        arithmetic on those propagates silently through everything downstream.
        """
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        return ranges, angles

    # ==================================================================
    # Classical baseline controller.
    # ==================================================================
    def plan(self, ranges, angles, dt=0.025, raw_ranges=None):
        """Follow the centreline and lower the target speed when LiDAR requires."""
        odom_fresh = (
            self.last_odom_time is not None
            and time.monotonic() - self.last_odom_time <= 0.5
        )
        pose_valid = (
            self.position is not None
            and all(math.isfinite(value) for value in self.position)
            and math.isfinite(self.yaw)
            and math.isfinite(self.speed)
        )
        if not odom_fresh or not pose_valid:
            self.speed_controller.reset()
            return 0.0, 0.0

        steering, _, curvature = self.controller.command(
            self.position, self.yaw, self.speed
        )
        curve_speed = math.sqrt(
            self.max_lateral_accel / max(abs(curvature), 1e-4)
        )
        target_speed = min(self.target_speed, curve_speed)
        if self.safety_enabled:
            decision = self.safety.apply(
                ranges if raw_ranges is None else raw_ranges,
                angles,
                baseline_steering=steering,
                baseline_target_speed=target_speed,
            )
            steering, target_speed = decision.steering, decision.target_speed
            if not decision.sensor_valid and time.monotonic() - self.last_safety_warning > 5.0:
                self.get_logger().warn('LiDAR forward sector is invalid; holding zero target speed')
                self.last_safety_warning = time.monotonic()
        throttle = self.speed_controller.command(target_speed, self.speed, dt)
        return steering, throttle

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def publish(self, steering, throttle):
        self.steering_pub.publish(Float32(data=float(np.clip(steering, -1.0, 1.0))))
        self.throttle_pub.publish(Float32(data=float(np.clip(throttle, -1.0, 1.0))))

    def publish_marker(self, target_angle):
        """Draw where the car thinks it is going. Add /driver/markers in RViz."""
        marker = Marker()
        marker.header.frame_id = 'roboracer_1'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'team_driver'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 1.5, 0.15, 0.15
        marker.color.g, marker.color.b, marker.color.a = 0.8, 1.0, 0.9
        marker.pose.orientation.z = math.sin(target_angle / 2.0)
        marker.pose.orientation.w = math.cos(target_angle / 2.0)
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
