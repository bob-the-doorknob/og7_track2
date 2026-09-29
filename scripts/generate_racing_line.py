#!/usr/bin/env python3
"""Generate a map-constrained, smoothed racing line from a closed centreline.

This is an offline planning tool. It only writes a CSV for the driver and has
no access to the simulator's lap or referee topics.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

from track_tool import OccupancyMap


def read_path(path: Path) -> np.ndarray:
    points = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if row and not row[0].lstrip().startswith("#"):
                points.append((float(row[0]), float(row[1])))
    result = np.asarray(points, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 2 or len(result) < 10:
        raise ValueError("centreline needs at least ten x/y points")
    if not np.all(np.isfinite(result)):
        raise ValueError("centreline contains non-finite points")
    return result


def generate(center: np.ndarray, grid: OccupancyMap, margin: float,
             min_clearance: float, iterations: int) -> np.ndarray:
    tangents = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    lengths = np.linalg.norm(tangents, axis=1)
    if np.any(lengths < 1e-6):
        raise ValueError("centreline has duplicate or collapsed neighbours")
    tangents /= lengths[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    lower = np.empty(len(center))
    upper = np.empty(len(center))
    for index, (point, normal) in enumerate(zip(center, normals)):
        if not grid.is_free(float(point[0]), float(point[1])):
            raise ValueError(f"centreline point {index} is outside the track map")
        left = grid.free_distance(*point, *normal, limit=2.0)
        right = grid.free_distance(*point, *(-normal), limit=2.0)
        lower[index] = -max(0.0, right - margin)
        upper[index] = max(0.0, left - margin)

    offset = np.zeros(len(center), dtype=np.float64)
    # Projected Laplacian relaxation shortens bends while spreading the
    # lateral movement over their approach and exit. The map bounds keep
    # every waypoint inside a corridor with room for the car's footprint.
    for _ in range(iterations):
        points = center + offset[:, None] * normals
        midpoint = (np.roll(points, 1, axis=0)
                    + np.roll(points, -1, axis=0)) / 2.0
        desired = np.einsum("ij,ij->i", midpoint - center, normals)
        offset = np.clip(0.7 * offset + 0.3 * desired, lower, upper)

    result = center + offset[:, None] * normals
    # A finite-width car cannot follow a path whose links cut a wall even if
    # every vertex is free. Check each link against the traced occupancy grid.
    for index in range(len(result)):
        first = tuple(result[index])
        second = tuple(result[(index + 1) % len(result)])
        clear, collision = grid.segment_is_clear(first, second)
        if not clear:
            raise ValueError(f"racing-line segment {index} leaves the track at {collision}")
        midpoint = (result[index] + result[(index + 1) % len(result)]) / 2.0
        for point in (result[index], midpoint):
            for angle in np.arange(0.0, 2.0 * math.pi, math.pi / 4.0):
                clearance = grid.free_distance(
                    float(point[0]), float(point[1]),
                    math.cos(angle), math.sin(angle), limit=min_clearance,
                )
                if clearance < min_clearance - grid.resolution:
                    raise ValueError(
                        f"racing-line segment {index} has only "
                        f"{clearance:.2f} m clearance"
                    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=Path("maps/icra26_compete.yaml"))
    parser.add_argument("--centerline", type=Path,
                        default=Path("maps/icra26_compete_centerline.csv"))
    parser.add_argument("--output", type=Path,
                        default=Path("maps/icra26_compete_racing_line.csv"))
    parser.add_argument("--margin", type=float, default=0.42,
                        help="metres of wall clearance reserved at waypoints")
    parser.add_argument("--min-clearance", type=float, default=0.30,
                        help="minimum checked clearance along every path link")
    parser.add_argument("--iterations", type=int, default=400)
    args = parser.parse_args()
    if (not math.isfinite(args.margin) or not math.isfinite(args.min_clearance)
            or args.margin < args.min_clearance or args.min_clearance <= 0.0
            or args.iterations < 1):
        parser.error("margin >= min-clearance > 0 and iterations > 0 are required")
    center = read_path(args.centerline)
    line = generate(center, OccupancyMap(str(args.map)), args.margin,
                    args.min_clearance, args.iterations)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        stream.write("# x_m, y_m\n")
        stream.write(f"# Racing line from {args.centerline.name}; margin {args.margin:.2f} m\n")
        writer = csv.writer(stream)
        writer.writerows((f"{x:.4f}", f"{y:.4f}") for x, y in line)
    distance = np.linalg.norm(np.roll(line, -1, axis=0) - line, axis=1).sum()
    displacement = np.linalg.norm(line - center, axis=1)
    print(f"Wrote {args.output}: {len(line)} points, {distance:.2f} m, "
          f"maximum shift {displacement.max():.2f} m")


if __name__ == "__main__":
    main()
