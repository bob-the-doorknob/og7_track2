"""Control pose must not mix an IPS position with another frame's heading."""

import pytest


def test_ips_does_not_overwrite_odometry_control_state():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from team_driver.driver import Driver

    rclpy.init(args=[])
    node = None
    try:
        node = Driver()
        msg = Odometry()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 5.0
        node.odom_callback(msg)
        original = (node.position, node.yaw, node.speed, node.last_odom_time)

        node.ips_callback(Point(x=1.25, y=2.0))

        assert (node.position, node.yaw, node.speed, node.last_odom_time) == original
        assert node.pose_source == 'odom'
        assert node.ips_position == (1.25, 2.0)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_odometry_subscription_precedes_scan_when_both_are_ready():
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver

    rclpy.init(args=[])
    node = None
    try:
        node = Driver()
        topics = [s.topic_name for s in node.subscriptions]
        assert topics.index('/autodrive/roboracer_1/odom') < topics.index('/autodrive/roboracer_1/lidar')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
