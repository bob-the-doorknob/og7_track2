"""Closed-track speed planning with curvature and braking constraints."""

from __future__ import annotations

import math

import numpy as np

from team_driver.pure_pursuit import PurePursuit
from team_driver.vehicle_limits import VehicleLimits


def closed_lap_limits(curvatures: np.ndarray, spacing: float,
                      limits: VehicleLimits) -> np.ndarray:
    """Forward/backward feasible speed plan over a circular sampled track."""
    curvature = np.abs(np.asarray(curvatures, dtype=np.float64))
    if (curvature.ndim != 1 or curvature.size < 3
            or not np.all(np.isfinite(curvature))
            or not math.isfinite(spacing) or spacing <= 0.0):
        raise ValueError("invalid closed-lap curvature samples")
    speeds = np.minimum(
        limits.top_speed_mps,
        np.sqrt(limits.lateral_mps2 / np.maximum(curvature, 1e-6)),
    )
    count = speeds.size
    for _ in range(2 * count):
        previous = speeds.copy()
        for step in range(2 * count - 1, -1, -1):
            index = step % count
            ahead = (index + 1) % count
            fraction = min(1.0, (speeds[index] ** 2 * curvature[index]
                                 / limits.lateral_mps2) ** 2)
            available = limits.brake_mps2 * math.sqrt(max(0.0, 1.0 - fraction))
            speeds[index] = min(speeds[index],
                                math.sqrt(speeds[ahead] ** 2
                                          + 2.0 * available * spacing))
        for step in range(1, 2 * count + 1):
            index = step % count
            behind = (index - 1) % count
            fraction = min(1.0, (speeds[behind] ** 2 * curvature[behind]
                                 / limits.lateral_mps2) ** 2)
            available = limits.accel_mps2 * math.sqrt(max(0.0, 1.0 - fraction))
            speeds[index] = min(speeds[index],
                                math.sqrt(speeds[behind] ** 2
                                          + 2.0 * available * spacing))
        if np.max(previous - speeds) < 1e-6:
            break
    return speeds


class SpeedProfile:
    """Plan fast straights while braking early enough for the next bend."""

    def __init__(
        self,
        path: PurePursuit,
        straight_speed: float = 4.4,
        max_lateral_accel: float = 1.5,
        max_braking_decel: float = 2.0,
        sample_spacing: float = 0.20,
        curvature_span: float = 0.60,
        limits: VehicleLimits | None = None,
    ) -> None:
        if not all(math.isfinite(value) and value > 0.0 for value in (
                straight_speed, max_lateral_accel, max_braking_decel,
                sample_spacing, curvature_span)):
            raise ValueError("speed profile parameters must be finite and positive")
        self.path = path
        self.straight_speed = float(straight_speed)
        count = max(3, int(math.ceil(path.total_length / sample_spacing)))
        self.spacing = path.total_length / count
        self.arcs = np.arange(count, dtype=np.float64) * self.spacing

        curvature_values = np.empty(count, dtype=np.float64)
        for index, arc in enumerate(self.arcs):
            before = path._point_at(arc - curvature_span)
            middle = path._point_at(arc)
            after = path._point_at(arc + curvature_span)
            first, second = middle - before, after - middle
            chord = after - before
            denominator = (np.linalg.norm(first) * np.linalg.norm(second)
                           * np.linalg.norm(chord))
            curvature = (
                2.0 * float(np.cross(first, second)) / denominator
                if denominator > 1e-8 else 0.0
            )
            curvature_values[index] = curvature

        self.curvatures = curvature_values

        if limits is not None:
            self.limits = closed_lap_limits(curvature_values, self.spacing, limits)
            return

        speed_limits = np.empty(count, dtype=np.float64)
        for index, curvature in enumerate(curvature_values):
            speed_limits[index] = min(
                self.straight_speed,
                math.sqrt(max_lateral_accel / max(abs(curvature), 1e-4)),
            )

        # Relax backwards twice around the closed track. A fast point must be
        # able to brake to the speed of every later bend before reaching it.
        for step in range(2 * count - 1, -1, -1):
            index = step % count
            ahead = (index + 1) % count
            speed_limits[index] = min(
                speed_limits[index],
                math.sqrt(speed_limits[ahead] ** 2 + 2.0 * max_braking_decel * self.spacing),
            )
        self.limits = speed_limits

    def predicted_lap_time_s(self) -> float:
        """Simple kinematic lap-time estimate for comparing candidate lines."""
        return float(np.sum(self.spacing / np.maximum(self.limits, 0.1)))

    def target_speed(self, position: tuple[float, float]) -> float:
        """Interpolate the planned speed at the projected path position."""
        _, _, arc = self.path._project(position)
        return self._sample(self.limits, arc)

    def _sample(self, values: np.ndarray, arc: float) -> float:
        sample = arc / self.spacing
        index = int(math.floor(sample)) % len(values)
        fraction = sample - math.floor(sample)
        ahead = (index + 1) % len(values)
        return float((1.0 - fraction) * values[index]
                     + fraction * values[ahead])

    def future_features(self, position: tuple[float, float]) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Curvature and target speed now, 1 m ahead, and 3 m ahead."""
        _, _, arc = self.path._project(position)
        offsets = (0.0, 1.0, 3.0)
        return (tuple(self._sample(self.curvatures, arc + offset) for offset in offsets),
                tuple(self._sample(self.limits, arc + offset) for offset in offsets))
