"""Closed-track speed planning with curvature and braking constraints."""

from __future__ import annotations

import math

import numpy as np

from team_driver.pure_pursuit import PurePursuit


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

        limits = np.empty(count, dtype=np.float64)
        for index, arc in enumerate(self.arcs):
            before = path._point_at(arc - curvature_span)
            middle = path._point_at(arc)
            after = path._point_at(arc + curvature_span)
            first, second = middle - before, after - middle
            chord = after - before
            denominator = (np.linalg.norm(first) * np.linalg.norm(second)
                           * np.linalg.norm(chord))
            curvature = (
                2.0 * abs(float(np.cross(first, second))) / denominator
                if denominator > 1e-8 else 0.0
            )
            limits[index] = min(
                self.straight_speed,
                math.sqrt(max_lateral_accel / max(curvature, 1e-4)),
            )

        # Relax backwards twice around the closed track. A fast point must be
        # able to brake to the speed of every later bend before reaching it.
        for step in range(2 * count - 1, -1, -1):
            index = step % count
            ahead = (index + 1) % count
            limits[index] = min(
                limits[index],
                math.sqrt(limits[ahead] ** 2 + 2.0 * max_braking_decel * self.spacing),
            )
        self.limits = limits

    def target_speed(self, position: tuple[float, float]) -> float:
        """Interpolate the planned speed at the nearest centreline arc."""
        _, _, arc = self.path._project(position)
        sample = arc / self.spacing
        index = int(math.floor(sample)) % len(self.limits)
        fraction = sample - math.floor(sample)
        ahead = (index + 1) % len(self.limits)
        return float((1.0 - fraction) * self.limits[index]
                     + fraction * self.limits[ahead])
