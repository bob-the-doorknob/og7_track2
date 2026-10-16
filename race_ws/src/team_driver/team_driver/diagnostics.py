"""Bounded, opt-in control traces; file writes happen after driving stops."""

from __future__ import annotations

import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path


FIELDS = (
    "sample_index", "elapsed_s", "wall_dt_s", "control_dt_s", "compute_ms",
    "odom_age_s", "x_m", "y_m", "speed_mps", "baseline_target_mps",
    "policy_target_mps", "final_target_mps", "baseline_steering", "steering",
    "throttle", "speed_multiplier", "cross_track_m", "heading_error_rad",
    "map_certified", "path_clearance_m", "residual_rejected", "sensor_valid",
    "status",
)
PREVIEW_FIELDS = ("arc_clearance_m", "exit_preview_used")
PLANNING_FIELDS = (
    "yaw_rad", "path_arc_m", "path_segment_index", "target_segment_index",
    "target_x_m", "target_y_m", "lookahead_m", "command_curvature_inv_m",
    "route_curvature_inv_m", "route_curvature_1m_inv_m", "route_curvature_3m_inv_m",
    "profile_target_mps", "steering_curve_target_mps", "steering_rad",
    "limiting_stage", "braking_reason",
    "pose_source", "ips_odom_delta_m", "yaw_rate_radps",
)


class ControlTrace:
    def __init__(self, path: str | Path, max_samples: int = 24000):
        if max_samples < 1:
            raise ValueError("max_samples must be positive")
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Reserve a new file before driving; never replace an existing run.
        self._file = self.path.open("x", newline="", encoding="utf-8")
        self._samples = deque(maxlen=max_samples)
        self.total_samples = 0

    @property
    def dropped_samples(self) -> int:
        return self.total_samples - len(self._samples)

    def record(self, sample: dict) -> None:
        self._samples.append(dict(sample, sample_index=self.total_samples))
        self.total_samples += 1

    def close(self) -> None:
        if self._file.closed:
            return
        try:
            writer = csv.DictWriter(
                self._file, fieldnames=FIELDS + PREVIEW_FIELDS + PLANNING_FIELDS)
            writer.writeheader()
            writer.writerows(self._samples)
        finally:
            self._file.close()


def _number(row: dict, name: str, default: float = math.nan) -> float:
    try:
        return float(row.get(name, default))
    except (TypeError, ValueError):
        return default


def summarize(rows) -> dict:
    """Report evidence counts, not an automatic diagnosis of hesitation."""
    rows = list(rows)
    report = dict(samples=len(rows), retained_duration_s=0.0,
                  dropped_samples=0, braking_samples=0,
                  near_target_braking_samples=0, safety_limited_samples=0,
                  policy_slowdown_samples=0, stale_odom_samples=0,
                  invalid_pose_samples=0, invalid_lidar_samples=0,
                  residual_rejected_samples=0, callback_gaps_over_100ms=0,
                  accel_brake_switches=0, max_compute_ms=0.0,
                  max_callback_gap_s=0.0)
    if rows:
        report["retained_duration_s"] = max(
            0.0, _number(rows[-1], "elapsed_s", 0) - _number(rows[0], "elapsed_s", 0))
        report["dropped_samples"] = int(_number(rows[0], "sample_index", 0))
    previous_sign = 0
    for row in rows:
        throttle = _number(row, "throttle", 0)
        sign = 1 if throttle > 0.01 else -1 if throttle < -0.01 else 0
        if sign:
            if previous_sign and sign != previous_sign:
                report["accel_brake_switches"] += 1
            previous_sign = sign
        braking = throttle < -0.01
        report["braking_samples"] += int(braking)
        gap = _number(row, "wall_dt_s", 0)
        report["callback_gaps_over_100ms"] += int(gap > 0.1)
        report["max_callback_gap_s"] = max(report["max_callback_gap_s"], gap)
        report["max_compute_ms"] = max(report["max_compute_ms"],
                                        _number(row, "compute_ms", 0))
        status = row.get("status", "")
        report["stale_odom_samples"] += int(status == "stale_odom")
        report["invalid_pose_samples"] += int(status == "invalid_pose")
        report["invalid_lidar_samples"] += int(str(row.get("sensor_valid")) in ("0", "False"))
        report["residual_rejected_samples"] += int(str(row.get("residual_rejected")) in ("1", "True"))
        baseline = _number(row, "baseline_target_mps")
        policy = _number(row, "policy_target_mps")
        final = _number(row, "final_target_mps")
        safety_limited = final < policy - 0.05
        report["safety_limited_samples"] += int(safety_limited)
        report["policy_slowdown_samples"] += int(policy < baseline - 0.05)
        error = _number(row, "speed_mps") - final
        report["near_target_braking_samples"] += int(
            braking and 0.0 < error <= 0.2 and not safety_limited)
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    args = parser.parse_args(args)
    with args.trace.open(newline="", encoding="utf-8") as handle:
        rows = csv.DictReader(handle)
        if not rows.fieldnames or not set(FIELDS).issubset(rows.fieldnames):
            parser.error("not a complete driver diagnostic CSV")
        print(json.dumps(summarize(rows), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
