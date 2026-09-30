"""Training-only Gymnasium wrapper for AutoDRIVE residual steering/speed RL.

This module is intentionally outside the installed ``team_driver`` package.
The scored ``driver`` executable never imports it and never publishes reset.
Run it only outside scored evaluations, without a referee or another driver.
The caller must enable training mode and declare the simulator variant. The
simulator does not expose its build identity over ROS, so that declaration
must match the actual executable.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import gymnasium as gym
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

from team_driver.driver import MAX_STEERING_RAD, devkit_qos
from team_driver.map_guard import MapGuard
from team_driver.pure_pursuit import PurePursuit
from team_driver.residual_policy import (OBSERVATION_DIM, OBSERVATION_DIM_V2,
                                         decode_action, pack_observation,
                                         pack_observation_v2)
from team_driver.safety import LidarSafety
from team_driver.speed_controller import SpeedController
from team_driver.speed_profile import SpeedProfile
from team_driver.vehicle_limits import load_limits, planning_limits

REWARD_WEIGHTS = {
    "progress": 3.0, "cross_track": -0.10, "heading": -0.01,
    "unsafe_speed": -0.10, "action_change": -0.03,
    "action_magnitude": -0.12,
    "rejected": -0.10, "collision": -50.0,
    "off_course": -30.0, "lap_complete": 20.0,
    "sensor_failure": -2.0,
    "step_time": -0.02,
}


class AutoDriveResidualEnv(gym.Env):
    """One residual action per ``frame_skip`` LiDAR frames, using ROS topics.

    Actions are the two raw policy outputs in [-1, 1]. Zero means the measured
    classical controller. Observations use the selected version of the same
    packer as the race driver. Each instance owns its own ROS context.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        simulator_variant: str,
        training_mode: bool = False,
        vehicle_ns: str = "/autodrive/roboracer_1",
        centerline_path: str | Path | None = None,
        vehicle_limits_path: str | Path | None = None,
        map_guard_path: str | Path | None = None,
        planning_lateral_mps2: float = 0.0,
        planning_top_speed_mps: float = 0.0,
        observation_version: int = 1,
        target_speed: float = 4.4,
        max_lateral_accel: float = 1.5,
        max_braking_decel: float = 2.0,
        safety_horizon: float = 5.5,
        max_range: float = 10.0,
        lookahead_min: float = 0.55,
        lookahead_max: float = 1.8,
        lookahead_speed_gain: float = 0.35,
        frame_skip: int = 4,
        max_steps: int = 500,
        episode_laps: int = 1,
        sensor_timeout_s: float = 3.0,
        off_course_m: float = 0.70,
    ) -> None:
        super().__init__()
        if not training_mode:
            raise ValueError("training resets require training_mode=True")
        if simulator_variant not in ("practice", "compete"):
            raise ValueError("simulator_variant must be 'practice' or 'compete'")
        if observation_version not in (1, 2):
            raise ValueError("observation_version must be 1 or 2")
        if frame_skip < 1 or max_steps < 1 or episode_laps < 1:
            raise ValueError("frame_skip, max_steps and episode_laps must be positive")
        if not all(math.isfinite(x) and x > 0.0 for x in (
                target_speed, max_lateral_accel, max_braking_decel,
                safety_horizon, max_range,
                sensor_timeout_s, off_course_m, lookahead_min,
                lookahead_max)) or not math.isfinite(lookahead_speed_gain) or lookahead_speed_gain < 0:
            raise ValueError("training speed, limits and timeout must be finite and positive")

        self.target_speed = float(target_speed)
        self.max_lateral_accel = float(max_lateral_accel)
        if vehicle_limits_path == "packaged":
            vehicle_limits_path = (Path(get_package_share_directory("team_driver")) /
                                   "config" / "vehicle_limits.yaml")
        self.vehicle_limits = (load_limits(Path(vehicle_limits_path))
                               if vehicle_limits_path else None)
        if self.vehicle_limits is not None:
            self.vehicle_limits = planning_limits(self.vehicle_limits,
                                                  planning_lateral_mps2,
                                                  planning_top_speed_mps)
        if self.vehicle_limits is not None:
            self.target_speed = self.vehicle_limits.top_speed_mps
            self.max_lateral_accel = self.vehicle_limits.lateral_mps2
        self.max_braking_decel = float(max_braking_decel)
        self.safety_horizon = float(safety_horizon)
        self.max_range = float(max_range)
        self.frame_skip = int(frame_skip)
        self.max_steps = int(max_steps)
        self.episode_laps = int(episode_laps)
        self.sensor_timeout_s = float(sensor_timeout_s)
        self.off_course_m = float(off_course_m)
        self.observation_version = int(observation_version)
        observation_dim = (OBSERVATION_DIM_V2 if observation_version == 2
                           else OBSERVATION_DIM)
        self.observation_space = gym.spaces.Box(
            low=np.array([0.0] * 18 + [-1.0] * (observation_dim - 18),
                         dtype=np.float32),
            high=np.ones(observation_dim, dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        if simulator_variant == "practice" and centerline_path is None:
            raise ValueError("practice training needs a matching centerline_path")
        if centerline_path is None:
            centerline_path = (Path(get_package_share_directory("team_driver")) /
                               "maps" / "icra26_compete_racing_line.csv")
        self.controller = PurePursuit(
            centerline_path, lookahead_min=lookahead_min,
            lookahead_max=lookahead_max,
            lookahead_speed_gain=lookahead_speed_gain,
        )
        if map_guard_path == "packaged":
            map_guard_path = (Path(get_package_share_directory("team_driver")) /
                              "maps" / "icra26_compete.yaml")
        self.map_guard = MapGuard(map_guard_path, self.controller) if map_guard_path else None
        self.speed_profile = SpeedProfile(
            self.controller, straight_speed=self.target_speed,
            max_lateral_accel=self.max_lateral_accel,
            max_braking_decel=self.max_braking_decel,
            limits=self.vehicle_limits,
        )
        self.safety = LidarSafety(max_range=self.max_range,
                                  horizon=self.safety_horizon,
                                  max_steering_rad=MAX_STEERING_RAD)
        self.speed_controller = SpeedController(
            throttle_min=-1.0 if self.vehicle_limits is not None else -0.12,
            throttle_max=1.0 if self.vehicle_limits is not None else 0.65,
            active_braking=self.vehicle_limits is not None,
        )

        self.context = Context()
        rclpy.init(context=self.context)
        try:
            self.node = Node("residual_training_env", context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            ns = vehicle_ns.rstrip("/")
            qos = devkit_qos()
            self.steering_topic = f"{ns}/steering_command"
            self.throttle_topic = f"{ns}/throttle_command"
            self.steering_pub = self.node.create_publisher(
                Float32, self.steering_topic, qos)
            self.throttle_pub = self.node.create_publisher(
                Float32, self.throttle_topic, qos)
            # The only reset publisher in our entry. It exists only here.
            self.reset_pub = self.node.create_publisher(
                Bool, "/autodrive/reset_command", qos)
            self.node.create_subscription(LaserScan, f"{ns}/lidar",
                                          self._on_scan, qos)
            self.node.create_subscription(Odometry, f"{ns}/odom",
                                          self._on_odom, qos)
            self.node.create_subscription(Int32, f"{ns}/collision_count",
                                          self._on_collision, qos)
            self.node.create_subscription(Int32, f"{ns}/lap_count",
                                          self._on_lap, qos)
        except Exception:
            if hasattr(self, "executor"):
                self.executor.shutdown()
            if hasattr(self, "node"):
                self.node.destroy_node()
            self.context.try_shutdown()
            raise

        self.scan_ranges: np.ndarray | None = None
        self.scan_angles: np.ndarray | None = None
        self.scan_seq = 0
        self.position: tuple[float, float] | None = None
        self.yaw = 0.0
        self.speed = 0.0
        self.odom_received_at = 0.0
        self.collision_count: int | None = None
        self.lap_count: int | None = None
        self.previous_observation: np.ndarray | None = None
        self.previous_arc = 0.0
        self.previous_action = np.zeros(2, dtype=np.float32)
        self.start_laps = 0
        self.start_collisions = 0
        self.steps = 0
        self.done = True

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_ranges = np.asarray(msg.ranges, dtype=np.float64)
        self.scan_angles = (msg.angle_min +
                            np.arange(len(msg.ranges), dtype=np.float64) *
                            msg.angle_increment)
        self.scan_seq += 1

    def _on_odom(self, msg: Odometry) -> None:
        point = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.position = (point.x, point.y)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed = math.hypot(msg.twist.twist.linear.x,
                                msg.twist.twist.linear.y)
        self.odom_received_at = time.monotonic()

    def _on_collision(self, msg: Int32) -> None:
        self.collision_count = int(msg.data)

    def _on_lap(self, msg: Int32) -> None:
        self.lap_count = int(msg.data)

    def _guard_practice(self) -> None:
        """Refuse to drive or reset if referee or another driver is present."""
        if any("referee" in name.lower() for name in self.node.get_node_names()):
            raise RuntimeError("referee detected; training must run alone")
        if (self.node.count_publishers(self.steering_topic) > 1 or
                self.node.count_publishers(self.throttle_topic) > 1):
            raise RuntimeError("another driver is publishing commands; stop it first")

    def _wait_for_frames(self, count: int) -> None:
        target = self.scan_seq + count
        deadline = time.monotonic() + self.sensor_timeout_s
        while self.scan_seq < target or not self._telemetry_ready():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("AutoDRIVE LiDAR/odom/lap/collision telemetry timed out")
            self.executor.spin_once(timeout_sec=min(remaining, 0.05))

    def _telemetry_ready(self) -> bool:
        return (self.scan_ranges is not None and self.position is not None and
                self.collision_count is not None and self.lap_count is not None
                and time.monotonic() - self.odom_received_at <= 0.5 and
                all(math.isfinite(x) for x in (*self.position, self.yaw, self.speed)))

    def _state(self) -> dict:
        if not self._telemetry_ready() or self.scan_ranges is None or self.scan_angles is None:
            raise ValueError("training telemetry is incomplete or stale")
        path = self.controller.path_state(self.position, self.yaw)
        baseline_steering, _, curvature = self.controller.command(
            self.position, self.yaw, self.speed)
        baseline_speed = min(
            self.speed_profile.target_speed(self.position),
            math.sqrt(self.max_lateral_accel / max(abs(curvature), 1e-4)),
        )
        if self.observation_version == 2:
            future_curvature, future_targets = self.speed_profile.future_features(
                self.position)
            observation = pack_observation_v2(
                self.scan_ranges, self.scan_angles, self.speed,
                path.heading_error, path.cross_track_error,
                future_curvature, (baseline_speed, *future_targets[1:]),
                baseline_steering, self.target_speed, self.max_range,
            )
        else:
            observation = pack_observation(
                self.scan_ranges, self.scan_angles, self.speed,
                path.heading_error, path.cross_track_error, path.upcoming_curvature,
                baseline_steering, baseline_speed, self.max_range,
            )
        _, _, arc = self.controller._project(self.position)
        return dict(observation=observation, arc=arc, path=path,
                    baseline_steering=baseline_steering,
                    baseline_speed=baseline_speed, speed=self.speed,
                    collision_count=self.collision_count, lap_count=self.lap_count)

    def _publish(self, steering: float, throttle: float) -> None:
        self.steering_pub.publish(Float32(data=float(np.clip(steering, -1.0, 1.0))))
        self.throttle_pub.publish(Float32(data=float(np.clip(throttle, -1.0, 1.0))))

    def _apply_action(self, state: dict, action: np.ndarray, dt: float) -> dict:
        steering_delta, speed_multiplier = decode_action(action)
        proposed_steering = float(np.clip(
            state["baseline_steering"] + steering_delta, -1.0, 1.0))
        proposed_speed = min(state["baseline_speed"] * speed_multiplier,
                             self.target_speed)
        map_certified = (self.map_guard is not None and
                         self.map_guard.certifies_clear_horizon(
                             self.position, self.yaw, self.speed, proposed_steering))
        decision = self.safety.apply(
            self.scan_ranges, self.scan_angles,
            baseline_steering=state["baseline_steering"],
            baseline_target_speed=state["baseline_speed"],
            proposed_steering=proposed_steering,
            proposed_target_speed=proposed_speed,
            speed_ceiling=self.target_speed,
            clear_horizon_certified=map_certified,
        )
        throttle = self.speed_controller.command(
            decision.target_speed, state["speed"], dt)
        self._publish(decision.steering, throttle)
        return dict(safe_target_speed=decision.target_speed,
                    safety_rejected=decision.residual_rejected,
                    sensor_valid=decision.sensor_valid)

    def reset(self, *, seed: int | None = None,
              options: dict | None = None) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        # Give DDS a chance to discover a running referee or manual driver.
        self.executor.spin_once(timeout_sec=0.2)
        self._guard_practice()
        self.done = True
        self._publish(0.0, 0.0)
        # Match the referee's reset hold/settle timing. A single-frame pulse
        # can be lost by the websocket bridge, especially after a long drive.
        try:
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                self.reset_pub.publish(Bool(data=True))
                self.executor.spin_once(timeout_sec=0.05)
        finally:
            # AutoDRIVE's reset is level-triggered: hold low through settling.
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                self.reset_pub.publish(Bool(data=False))
                self.executor.spin_once(timeout_sec=0.05)

        self._wait_for_frames(2)
        state = self._state()
        for _ in range(20):
            if (state["collision_count"] == 0 and state["lap_count"] == 0 and
                    state["speed"] < 0.25):
                break
            self._wait_for_frames(1)
            state = self._state()
        else:
            raise RuntimeError(
                "simulator did not confirm training reset "
                f"(laps={state['lap_count']}, collisions={state['collision_count']}, "
                f"speed={state['speed']:.2f} m/s)"
            )

        self.speed_controller.reset()
        self.previous_observation = state["observation"].copy()
        self.previous_arc = float(state["arc"])
        self.previous_action = np.zeros(2, dtype=np.float32)
        self.start_collisions = int(state["collision_count"])
        self.start_laps = int(state["lap_count"])
        self.steps = 0
        self.done = False
        # Zero residual exactly reproduces the measured baseline controls.
        control = self._apply_action(state, self.previous_action, 0.025)
        return self.previous_observation.copy(), dict(
            baseline_steering=state["baseline_steering"],
            baseline_target_speed=state["baseline_speed"], **control)

    def step(self, action) -> tuple[np.ndarray, float, bool, bool, dict]:
        if self.done or self.previous_observation is None:
            raise RuntimeError("call reset() before step() or after episode end")
        self._guard_practice()
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (2,) or not np.all(np.isfinite(action)):
            raise ValueError("action must be two finite residual controls")
        action = np.clip(action, -1.0, 1.0)
        old_state = self._state()
        control = self._apply_action(old_state, action, self.frame_skip / 40.0)
        try:
            self._wait_for_frames(self.frame_skip)
            state = self._state()
        except (TimeoutError, ValueError) as exc:
            self._publish(0.0, 0.0)
            self.done = True
            return (self.previous_observation.copy(), REWARD_WEIGHTS["sensor_failure"], False, True,
                    {"reason": "sensor_failure", "error": str(exc)})

        progress = float(state["arc"] - self.previous_arc)
        length = self.controller.total_length
        progress = (progress + length / 2.0) % length - length / 2.0
        progress = float(np.clip(progress, -2.0, 2.0))
        path = state["path"]
        new_collisions = state["collision_count"] - self.start_collisions
        laps = state["lap_count"] - self.start_laps
        counter_reset = new_collisions < 0 or laps < 0
        off_course = abs(path.cross_track_error) > self.off_course_m
        collision = new_collisions > 0
        success = laps >= self.episode_laps
        overspeed = max(0.0, state["speed"] - control["safe_target_speed"])
        change = float(np.linalg.norm(action - self.previous_action))
        terms = dict(
            progress=REWARD_WEIGHTS["progress"] * progress,
            cross_track=REWARD_WEIGHTS["cross_track"] * abs(path.cross_track_error),
            heading=REWARD_WEIGHTS["heading"] * abs(path.heading_error),
            unsafe_speed=REWARD_WEIGHTS["unsafe_speed"] * overspeed,
            action_change=REWARD_WEIGHTS["action_change"] * change,
            action_magnitude=REWARD_WEIGHTS["action_magnitude"]
            * float(np.linalg.norm(action)),
            rejected=REWARD_WEIGHTS["rejected"] if control["safety_rejected"] else 0.0,
            collision=REWARD_WEIGHTS["collision"] if collision else 0.0,
            off_course=REWARD_WEIGHTS["off_course"] if off_course else 0.0,
            lap_complete=REWARD_WEIGHTS["lap_complete"] if success and not collision and not off_course else 0.0,
            step_time=REWARD_WEIGHTS["step_time"],
        )
        reward = float(sum(terms.values()))
        self.steps += 1
        terminated = bool(collision or off_course or success)
        truncated = bool(not terminated and (
            self.steps >= self.max_steps or not control["sensor_valid"] or
            counter_reset))
        reason = ("collision" if collision else "off_course" if off_course else
                  "lap_complete" if success else "counter_reset" if counter_reset else
                  "invalid_lidar" if not control["sensor_valid"]
                  else "time_limit" if truncated else None)
        self.done = terminated or truncated
        self.previous_observation = state["observation"].copy()
        self.previous_arc = float(state["arc"])
        self.previous_action = action.copy()
        if self.done:
            self._publish(0.0, 0.0)
        info = dict(reason=reason, progress_m=progress,
                    cross_track_error_m=path.cross_track_error,
                    heading_error_rad=path.heading_error,
                    collisions=new_collisions, laps=laps,
                    reward_terms=terms, **control)
        return state["observation"].copy(), reward, terminated, truncated, info

    def close(self) -> None:
        if hasattr(self, "node"):
            self._publish(0.0, 0.0)
            self.executor.shutdown()
            self.node.destroy_node()
            del self.node
        if hasattr(self, "context"):
            self.context.try_shutdown()
