"""Fixed residual-policy interface shared by the race driver and trainer.

Observation v1 has 24 float32 values: 18 minimum LiDAR ranges over equal
15-degree sectors from -135 to +135 degrees, then speed, path heading error,
signed cross-track error, upcoming curvature, baseline steering, and baseline
target speed. All are scaled and clipped to [-1, 1] (LiDAR to [0, 1]).

Model format v1 is an uncompressed or compressed NumPy .npz with scalar integer
``format_version`` and ``observation_version`` (both 1), plus float arrays
``w0`` (64,24), ``b0`` (64,), ``w1`` (64,64), ``b1`` (64,), ``w2``
(2,64), and ``b2`` (2,). Layers use ReLU, ReLU, tanh. The two outputs are
bounded steering adjustment and target-speed multiplier controls.

Version 2 has 28 values: the same 18 LiDAR sectors, then measured speed,
heading and cross-track error, signed curvature and target speed at the
current point, 1 m, and 3 m ahead, and baseline steering. The same network
shape uses a (64, 28) first layer and declares both versions as 2.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np


OBSERVATION_VERSION = 1
MODEL_FORMAT_VERSION = 1
OBSERVATION_VERSION_V2 = 2
MODEL_FORMAT_VERSION_V2 = 2
LIDAR_SECTORS = 18
OBSERVATION_DIM = LIDAR_SECTORS + 6
OBSERVATION_DIM_V2 = LIDAR_SECTORS + 10
MAX_STEERING_DELTA = 0.15
MIN_SPEED_MULTIPLIER = 0.70
MAX_SPEED_MULTIPLIER = 1.20
STACKED_STEERING_SCALE = 1.0 / 3.0
STACKED_SPEED_SCALE = 0.5
_SHAPES = {
    "w0": (64, OBSERVATION_DIM), "b0": (64,),
    "w1": (64, 64), "b1": (64,),
    "w2": (2, 64), "b2": (2,),
}


def _lidar_features(ranges, angles, max_range: float) -> np.ndarray:
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    if (ranges.ndim != 1 or angles.shape != ranges.shape
            or not math.isfinite(max_range) or max_range <= 0.0):
        raise ValueError("invalid residual observation input")

    valid = (np.isfinite(angles)
             & ((np.isfinite(ranges) & (ranges > 0.0)) | np.isposinf(ranges)))
    sectors = np.linspace(-3.0 * math.pi / 4.0, 3.0 * math.pi / 4.0,
                          LIDAR_SECTORS + 1)
    lidar = np.empty(LIDAR_SECTORS, dtype=np.float64)
    for index in range(LIDAR_SECTORS):
        in_sector = valid & (angles >= sectors[index]) & (
            angles < sectors[index + 1] if index < LIDAR_SECTORS - 1
            else angles <= sectors[index + 1]
        )
        if not np.any(in_sector):
            raise ValueError(f"LiDAR sector {index} has no valid rays")
        lidar[index] = min(float(np.min(ranges[in_sector])), max_range) / max_range
    return lidar


def pack_observation(
    ranges, angles, speed: float, heading_error: float,
    cross_track_error: float, upcoming_curvature: float,
    baseline_steering: float, baseline_target_speed: float,
    max_range: float = 10.0,
) -> np.ndarray:
    """Keep the selected 4.4 m/s actor's exact v1 observation format."""
    state = np.asarray((
        speed, heading_error, cross_track_error, upcoming_curvature,
        baseline_steering, baseline_target_speed,
    ), dtype=np.float64)
    if not np.all(np.isfinite(state)):
        raise ValueError("invalid residual observation input")
    lidar = _lidar_features(ranges, angles, max_range)

    scaled_state = np.clip(state / np.asarray((
        4.0, math.pi, 1.0, 2.0, 1.0, 4.0,
    )), -1.0, 1.0)
    return np.concatenate((lidar, scaled_state)).astype(np.float32)


def pack_observation_v2(
    ranges, angles, speed: float, heading_error: float,
    cross_track_error: float, curvatures, target_speeds,
    baseline_steering: float, speed_scale: float,
    max_range: float = 10.0,
) -> np.ndarray:
    """High-speed observation: 18 LiDAR sectors and 10 path/control values."""
    curvature = np.asarray(curvatures, dtype=np.float64)
    targets = np.asarray(target_speeds, dtype=np.float64)
    if (curvature.shape != (3,) or targets.shape != (3,)
            or not math.isfinite(speed_scale) or speed_scale <= 0.0):
        raise ValueError("invalid v2 path features or speed scale")
    state = np.asarray((speed, heading_error, cross_track_error,
                        *curvature, *targets, baseline_steering), dtype=np.float64)
    if not np.all(np.isfinite(state)):
        raise ValueError("invalid v2 residual observation input")
    scaled = np.clip(state / np.asarray((
        speed_scale, math.pi, 1.0, 2.0, 2.0, 2.0,
        speed_scale, speed_scale, speed_scale, 1.0,
    )), -1.0, 1.0)
    return np.concatenate((_lidar_features(ranges, angles, max_range),
                           scaled)).astype(np.float32)


def decode_action(raw_action) -> tuple[float, float]:
    """Map bounded network outputs to steering delta and speed multiplier."""
    raw = np.asarray(raw_action, dtype=np.float64)
    if raw.shape != (2,) or not np.all(np.isfinite(raw)):
        raise ValueError("residual action must contain two finite values")
    raw = np.clip(raw, -1.0, 1.0)
    steering_delta = float(raw[0] * MAX_STEERING_DELTA)
    speed_control = float(raw[1])
    speed_multiplier = (
        1.0 + speed_control * (MAX_SPEED_MULTIPLIER - 1.0)
        if speed_control >= 0.0 else
        1.0 + speed_control * (1.0 - MIN_SPEED_MULTIPLIER)
    )
    return steering_delta, speed_multiplier


def scale_stacked_adjustment(steering_delta: float,
                             speed_multiplier: float,
                             steering_scale: float = STACKED_STEERING_SCALE,
                             speed_scale: float = STACKED_SPEED_SCALE) -> tuple[float, float]:
    """Keep a second residual smaller than the proven first-stage action."""
    if not all(np.isfinite(value) and 0.0 <= value <= 1.0 for value in
               (steering_scale, speed_scale)):
        raise ValueError("stacked action scales must be finite and within [0, 1]")
    return (steering_delta * steering_scale,
            1.0 + (speed_multiplier - 1.0) * speed_scale)


class ResidualPolicy:
    """Load validated small MLP weights once and run pure NumPy inference."""

    def __init__(self, weights: dict[str, np.ndarray],
                 observation_version: int = 1) -> None:
        self.weights = weights
        self.observation_version = observation_version
        self.observation_dim = (OBSERVATION_DIM_V2 if observation_version == 2
                                else OBSERVATION_DIM)

    @classmethod
    def load(cls, path: str | Path) -> "ResidualPolicy":
        with np.load(Path(path), allow_pickle=False) as archive:
            versions = []
            for key in ("format_version", "observation_version"):
                value = np.asarray(archive[key])
                if value.shape != () or value.dtype.kind not in "iu":
                    raise ValueError(f"unsupported {key}: {value}")
                versions.append(int(value))
            if tuple(versions) not in ((1, 1), (2, 2)):
                raise ValueError(f"unsupported model_format_version/observation_version pair: {versions}")
            dimension = OBSERVATION_DIM_V2 if versions[1] == 2 else OBSERVATION_DIM
            shapes = {**_SHAPES, "w0": (64, dimension)}
            weights = {}
            for key, shape in shapes.items():
                value = np.asarray(archive[key])
                if value.shape != shape or value.dtype.kind not in "f" \
                        or not np.all(np.isfinite(value)):
                    raise ValueError(f"invalid residual model array {key}")
                weights[key] = value.astype(np.float32, copy=True)
        return cls(weights, versions[1])

    def predict(self, observation) -> tuple[float, float]:
        obs = np.asarray(observation, dtype=np.float32)
        if obs.shape != (self.observation_dim,) or not np.all(np.isfinite(obs)):
            raise ValueError("invalid residual observation")
        w = self.weights
        hidden = np.maximum(w["w0"] @ obs + w["b0"], 0.0)
        hidden = np.maximum(w["w1"] @ hidden + w["b1"], 0.0)
        action = np.tanh(w["w2"] @ hidden + w["b2"])
        return decode_action(action)
