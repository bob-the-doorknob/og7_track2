"""Offline whole-lap racing-line search with hard map clearance."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from team_driver.pure_pursuit import PurePursuit
from team_driver.speed_profile import SpeedProfile
from team_driver.vehicle_limits import VehicleLimits, load_limits
from training.track_map import TrackMap


def predicted_time(points: np.ndarray, limits: VehicleLimits) -> float:
    """Use exactly the fast driver's speed planner to compare whole laps."""
    return SpeedProfile(PurePursuit(np.asarray(points)), limits=limits).predicted_lap_time_s()


def _normals(center: np.ndarray) -> np.ndarray:
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    length = np.linalg.norm(tangent, axis=1)
    if np.any(length < 1e-6):
        raise ValueError("collapsed centerline tangent")
    return np.column_stack((-tangent[:, 1], tangent[:, 0])) / length[:, None]


def generate_line(center: np.ndarray, grid: TrackMap, limits: VehicleLimits,
                  margin_m: float, *, initial: np.ndarray | None = None,
                  max_offset_m: float = 0.8, passes: int = 2) -> np.ndarray:
    """Improve an existing closed line by bounded, smooth lateral perturbations.

    The entire link must have clearance, and only strictly lower predicted lap
    times are accepted. A failed search returns the unchanged initial line.
    """
    center = np.asarray(center, dtype=np.float64)
    seed = center.copy() if initial is None else np.asarray(initial, dtype=np.float64).copy()
    if (center.ndim != 2 or center.shape[1] != 2 or len(center) < 12
            or seed.shape != center.shape or not np.all(np.isfinite(center))
            or not np.all(np.isfinite(seed)) or margin_m <= 0
            or max_offset_m <= 0 or passes < 1):
        raise ValueError("invalid closed-line optimization inputs")
    if grid.line_clearance(seed) < margin_m:
        raise ValueError("initial line lacks the requested whole-link clearance")
    normal = _normals(center)
    offset = np.einsum("ij,ij->i", seed - center, normal)
    if np.max(np.abs(offset)) > max_offset_m + 1e-6:
        raise ValueError("initial line exceeds centerline offset bound")
    best_line = seed.copy()
    best_time = predicted_time(best_line, limits)
    count = len(center)
    knots = np.unique(np.linspace(0, count - 1, min(24, count), dtype=int))
    indices = np.arange(count)
    for sweep in range(passes):
        amplitude = 0.12 / (2 ** sweep)
        width = max(3.0, count / 24.0)
        for knot in knots:
            distance = np.minimum((indices - knot) % count, (knot - indices) % count)
            bump = np.exp(-0.5 * (distance / width) ** 2)
            for sign in (-1.0, 1.0):
                proposed_offset = offset + sign * amplitude * bump
                if np.max(np.abs(proposed_offset)) > max_offset_m:
                    continue
                candidate = seed + (proposed_offset - offset)[:, None] * normal
                if grid.line_clearance(candidate) < margin_m:
                    continue
                try:
                    estimate = predicted_time(candidate, limits)
                except ValueError:
                    continue
                if math.isfinite(estimate) and estimate < best_time - 1e-4:
                    best_line, best_time = candidate, estimate
                    seed, offset = candidate, proposed_offset
    return best_line


def _write_csv(path: Path, points: np.ndarray) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        stream.write("# x_m, y_m\n# Offline lap-time optimization; see matching JSON\n")
        writer = csv.writer(stream)
        writer.writerows((f"{x:.6f}", f"{y:.6f}") for x, y in points)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--center", type=Path, required=True)
    parser.add_argument("--initial", type=Path, required=True)
    parser.add_argument("--limits", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--margin", type=float, default=0.30)
    parser.add_argument("--max-offset", type=float, default=0.8)
    parser.add_argument("--passes", type=int, default=2)
    args = parser.parse_args()
    center = PurePursuit._load_centerline(args.center)
    initial = PurePursuit._load_centerline(args.initial)
    grid = TrackMap(args.map)
    limits = load_limits(args.limits)
    before = predicted_time(initial, limits)
    initial_clearance = grid.line_clearance(initial)
    print(f"initial: {before:.3f} s, clearance {initial_clearance:.3f} m", flush=True)
    result = generate_line(center, grid, limits, args.margin,
                           initial=initial, max_offset_m=args.max_offset,
                           passes=args.passes)
    after = predicted_time(result, limits)
    clearance = grid.line_clearance(result)
    print(f"candidate: {after:.3f} s, clearance {clearance:.3f} m")
    if after >= before - 1e-4:
        print("No predicted improvement; previous line retained.")
        return
    args.output.parent.mkdir(parents=True, exist_ok=True)
    _write_csv(args.output, result)
    metadata = {
        "generator": "training.optimize_line v1",
        "map": str(args.map), "center": str(args.center),
        "initial": str(args.initial), "limits": str(args.limits),
        "margin_m": args.margin, "max_offset_m": args.max_offset,
        "passes": args.passes, "initial_predicted_lap_s": before,
        "predicted_lap_s": after, "initial_clearance_m": initial_clearance,
        "minimum_clearance_m": clearance,
        "initial_length_m": float(np.linalg.norm(np.roll(initial, -1, axis=0) - initial, axis=1).sum()),
        "length_m": float(np.linalg.norm(np.roll(result, -1, axis=0) - result, axis=1).sum()),
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.output} and matching JSON")


if __name__ == "__main__":
    main()
