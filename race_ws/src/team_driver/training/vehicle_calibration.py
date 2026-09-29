"""Record non-scored AutoDRIVE telemetry and estimate vehicle dynamics.

Run as ``python3 -m training.vehicle_calibration`` with the bridge and one
normal driver running. This process subscribes only; it never commands the car.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np

from team_driver.vehicle_limits import VehicleLimits, save_limits


COLUMNS = (
    "time_s", "x_m", "y_m", "yaw_rad", "speed_mps", "throttle",
    "steering", "arc_m", "clearance_m", "lap_count", "collision_count",
)


def estimate_limits(rows: np.ndarray) -> VehicleLimits:
    """Estimate conservative longitudinal and lateral limits from clean laps."""
    rows = np.asarray(rows, dtype=np.float64)
    if (rows.ndim != 2 or rows.shape[1] != len(COLUMNS) or len(rows) < 30
            or not np.all(np.isfinite(rows))):
        raise ValueError("calibration requires at least 30 finite telemetry rows")
    col = {name: rows[:, index] for index, name in enumerate(COLUMNS)}
    dt = np.diff(col["time_s"])
    if np.any(dt <= 0.0) or np.any(dt > 0.25):
        raise ValueError("calibration timestamps must advance without gaps")
    if (np.any(col["speed_mps"] < 0.0)
            or np.any(np.diff(col["collision_count"]) != 0.0)):
        raise ValueError("calibration run contains invalid speed or a collision")

    longitudinal = np.diff(col["speed_mps"]) / dt
    throttle = col["throttle"][1:]
    accel = longitudinal[(throttle >= 0.10) & (longitudinal > 0.10)]
    brake = -longitudinal[(throttle <= -0.05) & (longitudinal < -0.10)]
    yaw_rate = np.diff(np.unwrap(col["yaw_rad"])) / dt
    if len(brake) < 5:
        # The current PI driver normally coasts instead of commanding reverse
        # torque. Straight coasting is a conservative deceleration floor;
        # slowing in a bend also includes tire scrub and is excluded.
        coast = (throttle <= 0.02) & (longitudinal < -0.10) \
            & (np.abs(yaw_rate) < 0.20) & (col["speed_mps"][1:] > 1.0)
        brake = -longitudinal[coast]
    lateral = np.abs(col["speed_mps"][1:] * yaw_rate)
    lateral = lateral[(col["speed_mps"][1:] >= 1.0) & (lateral > 0.10)]
    if min(len(accel), len(brake), len(lateral)) < 5:
        raise ValueError("need at least five acceleration, braking, and corner samples")

    delay = 0.0
    rising = np.flatnonzero((col["throttle"][1:] >= 0.10)
                            & (col["throttle"][:-1] < 0.10)) + 1
    delays = []
    for index in rising:
        end = min(index + 10, len(rows))
        response = np.flatnonzero(longitudinal[index - 1:end - 1] > 0.10)
        if response.size:
            delays.append(col["time_s"][index - 1 + response[0]]
                          - col["time_s"][index])
    if delays:
        delay = max(0.0, float(np.median(delays)))

    return VehicleLimits(
        top_speed_mps=float(np.max(col["speed_mps"])),
        accel_mps2=float(np.quantile(accel, 0.20)),
        brake_mps2=float(np.quantile(brake, 0.20)),
        # High clean corner loads show what the vehicle already sustained.
        # Reserve 20% rather than mistaking gentle corners for a grip limit.
        lateral_mps2=float(0.8 * np.quantile(lateral, 0.90)),
        control_delay_s=delay,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--simulator-build", required=True)
    parser.add_argument("--duration-s", type=float, default=90.0)
    parser.add_argument("--output-dir", type=Path, default=Path("training/runs"))
    parser.add_argument("--limits-output", type=Path,
                        default=Path("config/vehicle_limits.yaml"))
    parser.add_argument("--path", type=Path,
                        default=Path("../../../maps/icra26_compete_racing_line.csv"))
    args = parser.parse_args()
    if (not args.run_id or any(ch in args.run_id for ch in "/\\..")
            or not math.isfinite(args.duration_s) or args.duration_s <= 0.0):
        parser.error("run-id must be a simple name and duration must be positive")

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32, Int32

    from team_driver.pure_pursuit import PurePursuit

    path = PurePursuit(args.path)
    qos = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    namespace = "/autodrive/roboracer_1"
    rows: list[list[float]] = []

    class Recorder(Node):
        def __init__(self):
            super().__init__("vehicle_calibration")
            self.throttle = 0.0
            self.steering = 0.0
            self.clearance = 0.0
            self.laps = 0
            self.collisions = 0
            self.create_subscription(Float32, f"{namespace}/throttle_command",
                                     lambda msg: setattr(self, "throttle", float(msg.data)), qos)
            self.create_subscription(Float32, f"{namespace}/steering_command",
                                     lambda msg: setattr(self, "steering", float(msg.data)), qos)
            self.create_subscription(Int32, f"{namespace}/lap_count",
                                     lambda msg: setattr(self, "laps", int(msg.data)), qos)
            self.create_subscription(Int32, f"{namespace}/collision_count",
                                     lambda msg: setattr(self, "collisions", int(msg.data)), qos)
            self.create_subscription(LaserScan, f"{namespace}/lidar", self.on_scan, qos)
            self.create_subscription(Odometry, f"{namespace}/odom", self.on_odom, qos)

        def on_scan(self, msg):
            values = np.asarray(msg.ranges, dtype=np.float64)
            angles = msg.angle_min + np.arange(values.size) * msg.angle_increment
            forward = np.isfinite(values) & (values > 0.0) & (np.abs(angles) < 0.35)
            if np.any(forward):
                self.clearance = float(np.min(values[forward]))

        def on_odom(self, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            x, y = float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)
            speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            stamp = msg.header.stamp
            sim_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if sim_time <= 0.0:
                sim_time = time.monotonic()
            _, _, arc = path._project((x, y))
            rows.append([sim_time, x, y, yaw, speed, self.throttle,
                         self.steering, arc, self.clearance,
                         float(self.laps), float(self.collisions)])

    rclpy.init()
    node = Recorder()
    deadline = time.monotonic() + args.duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    run_dir = args.output_dir / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    trace = run_dir / "telemetry.csv"
    with trace.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    print(f"Saved {len(rows)} telemetry rows to {trace}")
    limits = estimate_limits(np.asarray(rows, dtype=np.float64))
    save_limits(args.limits_output, limits, sample_count=len(rows),
                simulator_build=args.simulator_build, trace_id=args.run_id)
    print(f"Saved measured limits to {args.limits_output}: {limits}")


if __name__ == "__main__":
    main()
