"""Offline occupancy and conservative vehicle-clearance queries."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def _distance_1d(values: np.ndarray) -> np.ndarray:
    """Squared Euclidean distance transform of a one-dimensional array."""
    count = len(values)
    sites = np.zeros(count, dtype=np.int32)
    boundaries = np.empty(count + 1, dtype=np.float64)
    level = 0
    boundaries[0], boundaries[1] = -np.inf, np.inf
    for point in range(1, count):
        while True:
            old = sites[level]
            boundary = ((values[point] + point * point - values[old] - old * old)
                        / (2.0 * (point - old)))
            if boundary > boundaries[level] or level == 0:
                break
            level -= 1
        if boundary <= boundaries[level]:
            sites[0], boundaries[1], level = point, np.inf, 0
        else:
            level += 1
            sites[level] = point
            boundaries[level], boundaries[level + 1] = boundary, np.inf
    result = np.empty(count, dtype=np.float64)
    level = 0
    for point in range(count):
        while boundaries[level + 1] < point:
            level += 1
        site = sites[level]
        result[point] = (point - site) ** 2 + values[site]
    return result


class TrackMap:
    """Trinary ROS map: only cells strictly below free_thresh are driveable."""

    def __init__(self, yaml_path: str | Path) -> None:
        path = Path(yaml_path)
        meta = yaml.safe_load(path.read_text(encoding="utf-8"))
        image_path = path.parent / meta["image"]
        pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
        self.resolution = float(meta["resolution"])
        self.origin = np.asarray(meta["origin"][:2], dtype=np.float64)
        if (self.resolution <= 0 or not math.isfinite(self.resolution)
                or not np.all(np.isfinite(self.origin))):
            raise ValueError("invalid map resolution or origin")
        probability = (pixels / 255.0 if int(meta.get("negate", 0))
                       else (255 - pixels) / 255.0)
        self.free = probability < float(meta.get("free_thresh", 0.196))
        self.height, self.width = self.free.shape
        self._clearance = None

    def _cells(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        point = np.asarray(points, dtype=np.float64)
        finite = np.all(np.isfinite(point), axis=-1)
        col = np.floor((point[..., 0] - self.origin[0]) / self.resolution).astype(int)
        row = self.height - 1 - np.floor(
            (point[..., 1] - self.origin[1]) / self.resolution).astype(int)
        valid = finite & (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        return row, col, valid

    def is_free(self, x: float, y: float) -> bool:
        row, col, valid = self._cells(np.array([x, y]))
        return bool(valid and self.free[row, col])

    def _build_clearance(self) -> None:
        # Padding makes leaving the image equivalent to hitting an obstacle.
        free = np.pad(self.free, 1, constant_values=False)
        squared = np.where(free, 1e12, 0.0)
        for row in range(squared.shape[0]):
            squared[row] = _distance_1d(squared[row])
        for col in range(squared.shape[1]):
            squared[:, col] = _distance_1d(squared[:, col])
        self._clearance = np.sqrt(squared[1:-1, 1:-1]) * self.resolution

    def clearance(self, points: np.ndarray) -> np.ndarray:
        """Lower bound on free distance from points to any blocked cell."""
        point = np.asarray(points, dtype=np.float64)
        if self._clearance is None:
            self._build_clearance()
        row, col, valid = self._cells(point)
        safe_row = np.clip(row, 0, self.height - 1)
        safe_col = np.clip(col, 0, self.width - 1)
        center_x = self.origin[0] + (safe_col + 0.5) * self.resolution
        center_y = self.origin[1] + (self.height - 1 - safe_row + 0.5) * self.resolution
        offset = np.hypot(point[..., 0] - center_x, point[..., 1] - center_y)
        lower = self._clearance[safe_row, safe_col] - offset - self.resolution / math.sqrt(2.0)
        return np.where(valid & self.free[safe_row, safe_col], np.maximum(lower, 0.0), 0.0)

    def segment_has_clearance(self, first: tuple[float, float],
                              second: tuple[float, float], margin_m: float) -> bool:
        a, b = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
        if margin_m <= 0 or not np.all(np.isfinite([*a, *b, margin_m])):
            raise ValueError("segment and margin must be finite; margin positive")
        count = max(2, int(math.ceil(np.linalg.norm(b - a) / (self.resolution / 2))))
        samples = a + np.linspace(0.0, 1.0, count + 1)[:, None] * (b - a)
        return bool(np.all(self.clearance(samples) >= margin_m))

    def line_clearance(self, points: np.ndarray) -> float:
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
            raise ValueError("line must have at least three x/y points")
        following = np.roll(points, -1, axis=0)
        distances = np.linalg.norm(following - points, axis=1)
        parts = [points]
        for index, distance in enumerate(distances):
            count = max(2, int(math.ceil(distance / (self.resolution / 2))))
            fractions = np.arange(1, count, dtype=float) / count
            parts.append(points[index] + fractions[:, None] * (following[index] - points[index]))
        return float(np.min(self.clearance(np.concatenate(parts))))
