"""LiDAR clearance limits shared by the classical and residual controllers."""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np


class SafetyDecision(NamedTuple):
    steering: float
    target_speed: float
    front_clearance: float
    path_clearance: float
    residual_rejected: bool
    sensor_valid: bool


class LidarSafety:
    """Limit speed along the car's swept path without raising baseline speed."""

    def __init__(
        self,
        wheelbase: float = 0.324,
        max_steering_rad: float = 0.5236,
        vehicle_half_width: float = 0.135,
        side_margin: float = 0.08,
        stop_margin: float = 0.25,
        max_deceleration: float = 2.0,
        horizon: float = 3.0,
        max_range: float = 10.0,
        min_range: float = 0.06,
        min_front_beams: int = 10,
        min_front_valid_fraction: float = 0.60,
        reject_clearance: float = 0.9,
    ) -> None:
        self.wheelbase = max(float(wheelbase), 1e-3)
        self.max_steering_rad = max(float(max_steering_rad), 1e-3)
        self.footprint_radius = max(float(vehicle_half_width), 0.0) + max(
            float(side_margin), 0.0
        )
        self.stop_margin = max(float(stop_margin), 0.0)
        self.max_deceleration = max(float(max_deceleration), 1e-3)
        self.horizon = max(float(horizon), 0.5)
        self.max_range = max(float(max_range), self.horizon)
        self.min_range = max(float(min_range), 0.0)
        self.min_front_beams = max(int(min_front_beams), 1)
        self.min_front_valid_fraction = float(np.clip(
            min_front_valid_fraction, 0.0, 1.0
        ))
        self.reject_clearance = max(float(reject_clearance), 0.0)
        self.path_step = 0.10

    def _scan_points(
        self, ranges: np.ndarray, angles: np.ndarray
    ) -> tuple[np.ndarray, bool]:
        """Discard invalid returns and close isolated missing rays conservatively."""
        ranges = np.asarray(ranges, dtype=np.float64)
        angles = np.asarray(angles, dtype=np.float64)
        if ranges.ndim != 1 or angles.ndim != 1 or ranges.size != angles.size:
            return np.empty((0, 2), dtype=np.float64), False
        if ranges.size == 0:
            return np.empty((0, 2), dtype=np.float64), False

        angle_valid = np.isfinite(angles)
        positive_infinity = np.isposinf(ranges)
        finite_valid = np.isfinite(ranges) & (ranges >= self.min_range)
        valid = angle_valid & (finite_valid | positive_infinity)
        clean = np.where(positive_infinity, self.max_range, ranges)
        clean = np.where(valid, np.clip(clean, self.min_range, self.max_range),
                         self.max_range)

        # A single dropped ray at a wall edge must not appear as a gap wide
        # enough for the car. Adjacent valid rays fill only isolated holes.
        if len(clean) >= 3:
            isolated = ~valid[1:-1] & valid[:-2] & valid[2:]
            clean[1:-1] = np.where(
                isolated, np.minimum(clean[:-2], clean[2:]), clean[1:-1]
            )
            valid[1:-1] |= isolated

        forward = angle_valid & (np.abs(angles) <= math.radians(35.0))
        front_total = int(np.count_nonzero(forward))
        sensor_valid = (
            front_total >= self.min_front_beams
            and np.count_nonzero(valid & forward) / front_total
            >= self.min_front_valid_fraction
        )
        # Keep side returns for steering proposals that curve out of the
        # straight-ahead cone; the front cone above is only a health check.
        obstacle = valid & (clean < self.max_range) & angle_valid \
            & (np.abs(angles) <= math.pi / 2.0)
        x = clean[obstacle] * np.cos(angles[obstacle])
        y = clean[obstacle] * np.sin(angles[obstacle])
        points = np.column_stack((x, y))
        return points, bool(sensor_valid)

    def _path_clearance(self, points: np.ndarray, steering: float) -> float:
        """Find the first LiDAR return inside the vehicle's swept footprint."""
        steering = float(np.clip(steering, -1.0, 1.0))
        curvature = math.tan(steering * self.max_steering_rad) / self.wheelbase
        # A forward LiDAR scan cannot certify a path that doubles back behind
        # the car. Stop the swept path at a quarter turn.
        visible_horizon = self.horizon
        if abs(curvature) > 1e-6:
            visible_horizon = min(visible_horizon, math.pi / (2.0 * abs(curvature)))
        arc = np.arange(0.0, visible_horizon + self.path_step,
                        self.path_step, dtype=np.float64)
        arc = np.clip(arc, 0.0, visible_horizon)
        if abs(curvature) < 1e-6:
            path_x, path_y = arc, np.zeros_like(arc)
        else:
            path_x = np.sin(curvature * arc) / curvature
            path_y = (1.0 - np.cos(curvature * arc)) / curvature

        if points.size == 0:
            return visible_horizon
        ahead = points[:, 0] > 0.0
        nearby = np.sum(points * points, axis=1) <= (
            visible_horizon + self.footprint_radius
        ) ** 2
        points = points[ahead & nearby]
        if points.size == 0:
            return visible_horizon

        distance_squared = (
            (points[:, 0, None] - path_x[None, :]) ** 2
            + (points[:, 1, None] - path_y[None, :]) ** 2
        )
        occupied = np.any(
            distance_squared <= self.footprint_radius ** 2, axis=0
        )
        first = np.flatnonzero(occupied)
        return float(arc[first[0]]) if first.size else visible_horizon

    def apply(
        self,
        ranges: np.ndarray,
        angles: np.ndarray,
        baseline_steering: float,
        baseline_target_speed: float,
        proposed_steering: float | None = None,
        proposed_target_speed: float | None = None,
    ) -> SafetyDecision:
        """Clamp a proposed action to the measured baseline and LiDAR limit.

        A future residual policy may propose steering and speed. A proposal
        that enters a substantially tighter obstacle corridor is rejected.
        """
        baseline_steering = (
            float(np.clip(baseline_steering, -1.0, 1.0))
            if math.isfinite(baseline_steering) else 0.0
        )
        baseline_target_speed = (
            max(float(baseline_target_speed), 0.0)
            if math.isfinite(baseline_target_speed) else 0.0
        )
        if proposed_steering is None:
            proposed_steering = baseline_steering
        if proposed_target_speed is None:
            proposed_target_speed = baseline_target_speed
        proposal_valid = (
            math.isfinite(proposed_steering)
            and math.isfinite(proposed_target_speed)
        )
        if not proposal_valid:
            proposed_steering = baseline_steering
            proposed_target_speed = baseline_target_speed
        proposed_steering = float(np.clip(proposed_steering, -1.0, 1.0))
        proposed_target_speed = max(float(proposed_target_speed), 0.0)

        points, sensor_valid = self._scan_points(ranges, angles)
        if not sensor_valid:
            return SafetyDecision(baseline_steering, 0.0, 0.0, 0.0,
                                  not proposal_valid, False)

        front = self._path_clearance(points, 0.0)
        baseline_clearance = self._path_clearance(points, baseline_steering)
        proposed_clearance = (
            baseline_clearance if proposed_steering == baseline_steering
            else self._path_clearance(points, proposed_steering)
        )
        rejected = not proposal_valid or (
            proposed_steering != baseline_steering
            and proposed_clearance < self.reject_clearance
            and proposed_clearance + self.footprint_radius < baseline_clearance
        )
        steering = baseline_steering if rejected else proposed_steering
        clearance = baseline_clearance if rejected else proposed_clearance
        speed_limit = math.sqrt(
            2.0 * self.max_deceleration
            * max(0.0, clearance - self.stop_margin)
        )
        target_speed = min(
            baseline_target_speed, proposed_target_speed, speed_limit
        )
        return SafetyDecision(
            steering, float(target_speed), float(front), float(clearance),
            bool(rejected), True
        )
