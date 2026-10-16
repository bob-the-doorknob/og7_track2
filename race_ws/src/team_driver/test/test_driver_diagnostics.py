"""Exercise the real driver's planning path with and without tracing."""

import math
import time

import numpy as np
import pytest


def test_trace_preserves_controls_and_records_safety_and_stale_pose(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from team_driver.diagnostics import ControlTrace
    from team_driver.driver import Driver
    from team_driver.pure_pursuit import predict_pose

    rclpy.init(args=[])
    node = None
    trace = ControlTrace(tmp_path / "driver.csv")
    try:
        node = Driver()
        assert node.speed_controller.approach_band_mps == 0.0
        assert node.exit_preview.enabled
        assert node.exit_preview.max_curvature_inv_m == .30
        ranges = np.full(1080, 10.0)
        angles = np.linspace(-3 * math.pi / 4, 3 * math.pi / 4, 1080)
        for index, speed in ((0, 0.0), (50, 5.0), (150, 7.0)):
            point = node.controller.points[index]
            ahead = node.controller.points[(index + 1) % len(node.controller.points)]
            node.position = tuple(point)
            node.yaw = math.atan2(ahead[1] - point[1], ahead[0] - point[0])
            node.speed = speed
            node.last_odom_time = time.monotonic()
            node.trace = None
            node.speed_controller.reset()
            original = node.plan(ranges, angles)
            node.trace = trace
            node.speed_controller.reset()
            traced = node.plan(ranges, angles)
            assert traced == original
            assert node.trace_sample["status"] == "ok"
            assert node.trace_sample["final_target_mps"] >= 0
            assert "path_clearance_m" in node.trace_sample
            assert "policy_target_mps" in node.trace_sample
            predicted_position, predicted_yaw = predict_pose(node.position,
                node.yaw, node.speed, node.yaw_rate,
                node.trace_sample['prediction_horizon_s'])
            extra = min(.3, 2*speed*max(0.,node.prediction_interval_s-.02))
            expected_steering, lookahead, curvature = node.controller.command(
                predicted_position, predicted_yaw, node.speed, lookahead_extra=extra)
            _, _, arc = node.controller._project(predicted_position)
            target = node.controller._point_at(arc + lookahead)
            future_curves, _ = node.speed_profile.future_features(node.position)
            assert node.trace_sample['lookahead_m'] == lookahead
            assert node.trace_sample['command_curvature_inv_m'] == curvature
            assert node.trace_sample['path_arc_m'] == node.controller._project(node.position)[2]
            assert node.trace_sample['target_x_m'] == target[0]
            assert node.trace_sample['target_y_m'] == target[1]
            assert node.trace_sample['route_curvature_1m_inv_m'] == future_curves[1]
            assert node.trace_sample['profile_target_mps'] == node.speed_profile.target_speed(predicted_position)
            assert node.trace_sample['limiting_stage'] in (
                'profile', 'steering_curvature', 'policy', 'safety')
            assert node.trace_sample['braking_reason'] == (
                'overspeed_' + node.trace_sample['limiting_stage']
                if traced[1] < -.01 else 'none')

        node.speed = 4.0
        node.plan(np.full_like(ranges, .2), angles)
        assert node.trace_sample['final_target_mps'] == 0.0
        assert node.trace_sample['limiting_stage'] == 'safety'
        assert node.trace_sample['braking_reason'] == 'overspeed_safety'

        node.last_odom_time = time.monotonic() - 1.0
        assert node.plan(ranges, angles) == (0.0, 0.0)
        assert node.trace_sample["status"] == "stale_odom"
        # The real callback must append a complete row, including timing.
        from sensor_msgs.msg import LaserScan
        scan = LaserScan()
        scan.ranges = ranges.tolist()
        scan.angle_min = float(angles[0])
        scan.angle_increment = float(angles[1] - angles[0])
        # Observe callback output without sending commands to a live bridge.
        commands = []
        node.publish = lambda steering, throttle: commands.append((steering, throttle))
        node.scan_callback(scan)
        assert trace.total_samples == 1
        assert commands == [(0.0, 0.0)]
    finally:
        trace.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('suffix, parameter, value', [
    ('smooth', 'speed_approach_band_mps', 0.3),
    ('smooth15', 'speed_approach_band_mps', 0.15),
    ('lookahead16', 'lookahead_max', 1.6),
    ('exit_preview', 'safety_exit_preview_enabled', True),
])
def test_candidate_changes_only_one_control_parameter(suffix, parameter, value):
    from pathlib import Path
    import yaml
    rclpy = pytest.importorskip("rclpy")
    from team_driver.driver import Driver

    config = Path(__file__).resolve().parents[1] / "config"
    reference = config / "fast_corner_24_residual_v2_sub12_params.yaml"
    candidate = config / f"fast_corner_24_residual_v2_sub12_{suffix}_params.yaml"
    parameters = yaml.safe_load(candidate.read_text())["driver"]["ros__parameters"]
    original = yaml.safe_load(reference.read_text())["driver"]["ros__parameters"]
    assert parameters.pop(parameter) == value
    original.pop(parameter, None)
    assert parameters == original
    rclpy.init(args=["--ros-args", "--params-file", str(candidate)])
    node = None
    try:
        node = Driver()
        assert node.get_parameter(parameter).value == value
        if parameter == 'lookahead_max':
            assert node.controller.lookahead_max == value
            assert node.speed_controller.approach_band_mps == 0.0
        elif parameter == 'safety_exit_preview_enabled':
            assert node.exit_preview.enabled
            assert node.speed_controller.approach_band_mps == 0.0
        else:
            assert node.speed_controller.approach_band_mps == value
        assert node.policy is not None
        assert node.safety_enabled
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_wall_exit_preview_is_connected_to_the_real_driver(tmp_path):
    from pathlib import Path
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver
    from team_driver.diagnostics import ControlTrace

    config = (Path(__file__).resolve().parents[1] / 'config' /
              'fast_corner_24_residual_v2_sub12_wall_exit_params.yaml')
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    node = None
    trace = ControlTrace(tmp_path / 'exit.csv')
    try:
        node = Driver()
        assert node.exit_preview.enabled
        assert node.speed_controller.approach_band_mps == .15
        point = node.controller._point_at(20)
        tangent = node.controller._point_at(20.1) - node.controller._point_at(19.9)
        node.position = tuple(point)
        node.yaw = math.atan2(tangent[1], tangent[0])
        node.speed = 4
        node.last_odom_time = time.monotonic()
        node.trace = trace
        ranges = np.full(1080, 10.)
        angles = np.linspace(-3*math.pi/4, 3*math.pi/4, 1080)
        previews = []
        original_apply = node.safety.apply
        def observe_apply(*args, **kwargs):
            previews.append(kwargs.get('trajectory'))
            return original_apply(*args, **kwargs)
        node.safety.apply = observe_apply
        # plan() alone never publishes to a bridge.
        steering, throttle = node.plan(ranges, angles)
        assert previews[0] is not None
        assert not node.trace_sample['exit_preview_used']  # Original clear arc is longer.
        assert node.trace_sample['path_clearance_m'] > 5.9
        assert math.isfinite(steering) and math.isfinite(throttle)
        node.last_odom_time = time.monotonic() - 1
        assert node.plan(ranges, angles) == (0, 0)
    finally:
        trace.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_bare_driver_defaults_match_the_selected_launch_configuration():
    from pathlib import Path
    import yaml
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver

    config = Path(__file__).resolve().parents[1] / 'config/driver_params.yaml'
    selected = yaml.safe_load(config.read_text())['driver']['ros__parameters']
    rclpy.init(args=[])
    node = None
    try:
        node = Driver()
        for name, value in selected.items():
            assert node.get_parameter(name).value == value, name
        assert node.exit_preview.enabled
        assert node.exit_preview.max_curvature_inv_m == .30
        assert node.vehicle_limits.lateral_mps2 == 18.0
        assert node.speed_controller.approach_band_mps == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_pose_trace_exposes_independent_ips_updates(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from team_driver.driver import Driver
    from team_driver.diagnostics import ControlTrace

    rclpy.init(args=[])
    node = None
    trace = ControlTrace(tmp_path / 'pose.csv')
    try:
        node = Driver()
        point = node.controller.points[20]
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, point)
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 1.0
        msg.twist.twist.angular.z = .5
        node.odom_callback(msg)
        node.ips_callback(Point(x=float(point[0]+.1), y=float(point[1]), z=0.0))
        node.trace = trace
        node.plan(np.full(1080, 10.), np.linspace(-3*math.pi/4, 3*math.pi/4, 1080))
        assert abs(node.trace_sample['ips_odom_delta_m'] - .1) < 1e-12
        assert node.trace_sample['yaw_rate_radps'] == .5
        assert node.trace_sample['pose_source'] == 'odom'
    finally:
        trace.close()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_exit_profile_retains_straight_speed_and_plans_earlier_corner_braking():
    from pathlib import Path
    import yaml
    rclpy = pytest.importorskip('rclpy')
    from team_driver.driver import Driver

    config = Path(__file__).resolve().parents[1] / 'config'
    reference = yaml.safe_load((config / 'fast_corner_24_residual_v2_sub12_params.yaml').read_text())['driver']['ros__parameters']
    candidate_file = config / 'fast_corner_18_residual_v2_exit_preview_params.yaml'
    candidate = yaml.safe_load(candidate_file.read_text())['driver']['ros__parameters']
    assert {key for key in reference.keys() | candidate.keys()
            if reference.get(key) != candidate.get(key)} == {
                'planning_lateral_mps2', 'safety_exit_preview_enabled'}
    rclpy.init(args=['--ros-args', '--params-file', str(candidate_file)])
    node = None
    try:
        node = Driver()
        assert node.exit_preview.enabled
        assert node.vehicle_limits.lateral_mps2 == 18.0
        assert node.target_speed == 7.2
        assert node.speed_controller.approach_band_mps == 0.0
        assert node.speed_profile.target_speed(tuple(node.controller._point_at(4))) == 7.2
        assert node.speed_profile.target_speed(tuple(node.controller._point_at(35))) < 3.6
        assert node.speed_profile.target_speed(tuple(node.controller._point_at(20))) > 5.5
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
