import math
import time

import numpy as np
import pytest


def test_delayed_speed_profile_queries_predicted_position(monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver
    from team_driver.pure_pursuit import predict_pose

    rclpy.init(args=[])
    node = None
    try:
        node = Driver()
        node.position = tuple(node.controller.points[20])
        node.yaw, node.speed, node.yaw_rate = 0., 3., 0.
        node.last_odom_time = time.monotonic()
        node.steering_prediction_s = .1
        node.speed_prediction_enabled = True
        positions = []
        original = node.speed_profile.target_speed
        def record(position):
            positions.append(position)
            return original(position)
        monkeypatch.setattr(node.speed_profile, 'target_speed', record)

        node.plan(np.full(1080, 10.), np.linspace(-3*math.pi/4, 3*math.pi/4, 1080))

        expected, _ = predict_pose(node.position, node.yaw, node.speed, node.yaw_rate, .1)
        assert positions[0] == pytest.approx(expected)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_trace_target_is_the_waypoint_selected_from_predicted_pose(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver
    from team_driver.pure_pursuit import predict_pose
    from team_driver.diagnostics import ControlTrace
    rclpy.init(args=[])
    node = None
    trace = ControlTrace(tmp_path/'prediction.csv')
    try:
        node=Driver()
        node.position=tuple(node.controller._point_at(4.))
        node.yaw, node.speed, node.yaw_rate=0.,4.,0.
        node.last_odom_time=time.monotonic()
        node.steering_prediction_s=.1
        node.trace=trace
        node.plan(np.full(1080,10.),np.linspace(-3*math.pi/4,3*math.pi/4,1080))
        position,_=predict_pose(node.position,node.yaw,node.speed,node.yaw_rate,.1)
        _,_,arc=node.controller._project(position)
        target=node.controller._point_at(arc+node.trace_sample['lookahead_m'])
        assert (node.trace_sample['target_x_m'],node.trace_sample['target_y_m']) == pytest.approx(target)
    finally:
        trace.close()
        if node is not None: node.destroy_node()
        rclpy.shutdown()


def test_bare_defaults_select_the_validated_latency_controls():
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver
    rclpy.init(args=[])
    node=None
    try:
        node=Driver()
        assert node.steering_prediction_frames == 2.
        assert node.steering_prediction_s == 0.
        assert node.speed_prediction_enabled
        assert node.adaptive_lookahead_enabled
        assert node.controller.lookahead_max == 1.5
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
