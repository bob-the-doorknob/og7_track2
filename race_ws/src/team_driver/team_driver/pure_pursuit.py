"""Pure pursuit path following for the Track 2 centreline."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


class PurePursuit:
    """Follow a closed centreline and return a bounded steering command."""

    def __init__(
        self,
        centerline_path: str | Path,
        wheelbase: float = 0.324,
        max_steering_rad: float = 0.5236,
        lookahead_min: float = 0.55,
        lookahead_max: float = 1.8,
        lookahead_speed_gain: float = 0.35,
    ) -> None:
        self.points = self._load_centerline(centerline_path)
        self.wheelbase = max(float(wheelbase), 1e-3)
        self.max_steering_rad = max(float(max_steering_rad), 1e-3)
        self.lookahead_min = max(float(lookahead_min), 0.1)
        self.lookahead_max = max(float(lookahead_max), self.lookahead_min)
        self.lookahead_speed_gain = max(float(lookahead_speed_gain), 0.0)
        self.segments = np.roll(self.points, -1, axis=0) - self.points
        self.segment_lengths = np.linalg.norm(self.segments, axis=1)
        if np.any(self.segment_lengths < 1e-6):
            raise ValueError("centreline contains duplicate consecutive points")
        self.cumulative = np.concatenate(([0.0], np.cumsum(self.segment_lengths)))
        self.total_length = float(self.cumulative[-1])

    @staticmethod
    def _load_centerline(path: str | Path) -> np.ndarray:
        points = []
        with Path(path).open(newline="", encoding="utf-8") as stream:
            for row in csv.reader(stream):
                if not row or row[0].lstrip().startswith("#"):
                    continue
                if len(row) < 2:
                    raise ValueError(f"invalid centreline row: {row}")
                points.append((float(row[0]), float(row[1])))
        result = np.asarray(points, dtype=np.float64)
        if result.ndim != 2 or result.shape[0] < 3 or result.shape[1] != 2:
            raise ValueError(f"invalid centreline (expected at least 3 x/y rows): {path}")
        if not np.all(np.isfinite(result)):
            raise ValueError(f"centreline contains non-finite coordinates: {path}")
        return result

    def _project(self, position: tuple[float, float]) -> tuple[int, np.ndarray, float]:
        """Find the closest point on any segment, including the closing segment."""
        position_array = np.asarray(position, dtype=np.float64)
        starts = self.points
        fractions = np.clip(
            np.einsum("ij,ij->i", position_array - starts, self.segments)
            / (self.segment_lengths ** 2),
            0.0,
            1.0,
        )
        projections = starts + fractions[:, None] * self.segments
        distances_squared = np.einsum(
            "ij,ij->i", projections - position_array, projections - position_array
        )
        index = int(np.argmin(distances_squared))
        arc = float(self.cumulative[index] + fractions[index] * self.segment_lengths[index])
        return index, projections[index], arc

    def _point_at(self, arc: float) -> np.ndarray:
        arc = arc % self.total_length
        index = int(np.searchsorted(self.cumulative, arc, side="right") - 1)
        index = min(index, len(self.segment_lengths) - 1)
        fraction = (arc - self.cumulative[index]) / self.segment_lengths[index]
        return self.points[index] + fraction * self.segments[index]

    def command(
        self, position: tuple[float, float], yaw: float, speed: float
    ) -> tuple[float, float, float]:
        """Return (normalized steering, lookahead distance, curvature estimate)."""
        if not all(math.isfinite(value) for value in (*position, yaw, speed)):
            raise ValueError("pose and speed must be finite")
        _, _, projected_arc = self._project(position)
        lookahead = float(np.clip(
            self.lookahead_min + self.lookahead_speed_gain * abs(speed),
            self.lookahead_min,
            self.lookahead_max,
        ))
        target = self._point_at(projected_arc + lookahead)
        dx, dy = target - np.asarray(position, dtype=np.float64)
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(left, forward)
        curvature = 2.0 * math.sin(alpha) / max(lookahead, 1e-3)
        steering_rad = math.atan(self.wheelbase * curvature)
        steering = float(np.clip(
            steering_rad / self.max_steering_rad, -1.0, 1.0
        ))
        return steering, lookahead, curvature
