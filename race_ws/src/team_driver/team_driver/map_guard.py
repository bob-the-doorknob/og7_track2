"""Certify a mapped clear straight beyond the LiDAR's finite range."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from team_driver.pure_pursuit import PurePursuit


class MapGuard:
    """Use only the known free track; unknown map cells are blocked."""

    def __init__(self, yaml_path: str | Path, path: PurePursuit,
                 footprint_m: float = 0.215) -> None:
        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image = (yaml_path.parent / meta["image"]).read_bytes()
        header = image.split(b"\n", 3)
        if len(header) != 4 or header[0] != b"P5" or header[2] != b"255":
            raise ValueError("map guard expects the provided 8-bit P5 map")
        width, height = map(int, header[1].split())
        if len(header[3]) != width * height:
            raise ValueError("map image dimensions do not match its data")
        pixels = np.frombuffer(header[3], dtype=np.uint8).reshape(height, width)
        probability = (pixels / 255.0 if int(meta.get("negate", 0))
                       else (255 - pixels) / 255.0)
        self.free = probability < float(meta.get("free_thresh", 0.196))
        self.width, self.height = width, height
        self.origin = np.asarray(meta["origin"][:2], dtype=float)
        self.resolution = float(meta["resolution"])
        if (not np.all(np.isfinite(self.origin)) or self.resolution <= 0.0
                or not math.isfinite(self.resolution) or footprint_m <= 0.0):
            raise ValueError("invalid map origin, resolution, or footprint")
        self.path = path
        self.footprint_m = float(footprint_m)
        angles = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
        self._disk = np.vstack((np.zeros((1, 2)),
                               self.footprint_m * np.column_stack((np.cos(angles),
                                                                   np.sin(angles)))))
        # Certify every link once, before driving. The runtime check only
        # needs to confirm that the current car remains close to this line.
        arcs = np.arange(0.0, path.total_length, self.resolution)
        for arc in arcs:
            if not self._free_disk(path._point_at(float(arc))):
                raise ValueError(f"racing line intersects mapped wall near arc {arc:.2f} m")

    def _free_points(self, points: np.ndarray) -> bool:
        points = np.asarray(points, dtype=float)
        if not np.all(np.isfinite(points)):
            return False
        col = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = self.height - 1 - np.floor(
            (points[:, 1] - self.origin[1]) / self.resolution).astype(int)
        if np.any((row < 0) | (row >= self.height) | (col < 0) | (col >= self.width)):
            return False
        return bool(np.all(self.free[row, col]))

    def _free_disk(self, point: np.ndarray) -> bool:
        return self._free_points(np.asarray(point) + self._disk)

    def certifies_clear_horizon(self, position: tuple[float, float], yaw: float,
                                speed: float, steering: float) -> bool:
        """Allow a clear LiDAR horizon only on a mapped, aligned straight."""
        if (not all(math.isfinite(value) for value in (*position, yaw, speed, steering))
                or speed < 0.0 or abs(steering) > 0.08
                or not self._free_disk(np.asarray(position))):
            return False
        state = self.path.path_state(position, yaw)
        return (abs(state.cross_track_error) <= 0.35
                and abs(state.heading_error) <= 0.25)
