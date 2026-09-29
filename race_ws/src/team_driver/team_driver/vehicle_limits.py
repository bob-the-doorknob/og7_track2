"""Measured vehicle limits shared by offline planning and the ROS driver."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Sequence

import yaml


@dataclass(frozen=True)
class VehicleLimits:
    top_speed_mps: float
    accel_mps2: float
    brake_mps2: float
    lateral_mps2: float
    control_delay_s: float

    def __post_init__(self) -> None:
        positive = (
            self.top_speed_mps, self.accel_mps2,
            self.brake_mps2, self.lateral_mps2,
        )
        if (not all(math.isfinite(value) and value > 0.0 for value in positive)
                or not math.isfinite(self.control_delay_s)
                or self.control_delay_s < 0.0):
            raise ValueError("vehicle limits must be finite and physically valid")


def planning_limits(measured: VehicleLimits, lateral_override: float = 0.0,
                    top_speed_override: float = 0.0) -> VehicleLimits:
    """Use a scored experimental cornering allowance without rewriting telemetry."""
    if (not math.isfinite(lateral_override) or lateral_override < 0.0
            or not math.isfinite(top_speed_override) or top_speed_override < 0.0):
        raise ValueError("planning overrides must be finite and nonnegative")
    if lateral_override == 0.0 and top_speed_override == 0.0:
        return measured
    return replace(
        measured,
        lateral_mps2=float(lateral_override or measured.lateral_mps2),
        top_speed_mps=float(top_speed_override or measured.top_speed_mps),
    )


def combine_limits(runs: Sequence[VehicleLimits]) -> VehicleLimits:
    """Combine clean runs without treating gentle turns as the grip limit."""
    if not runs:
        raise ValueError("at least one measured calibration run is required")
    return VehicleLimits(
        top_speed_mps=max(run.top_speed_mps for run in runs),
        accel_mps2=min(run.accel_mps2 for run in runs),
        brake_mps2=min(run.brake_mps2 for run in runs),
        lateral_mps2=max(run.lateral_mps2 for run in runs),
        control_delay_s=max(run.control_delay_s for run in runs),
    )


def save_limits(path: Path, limits: VehicleLimits, *, sample_count: int,
                simulator_build: str, trace_id: str) -> None:
    if sample_count < 30 or not simulator_build.strip() or not trace_id.strip():
        raise ValueError("vehicle limits require sufficient samples and provenance")
    payload = {
        "top_speed_mps": limits.top_speed_mps,
        "accel_mps2": limits.accel_mps2,
        "brake_mps2": limits.brake_mps2,
        "lateral_mps2": limits.lateral_mps2,
        "control_delay_s": limits.control_delay_s,
        "sample_count": int(sample_count),
        "simulator_build": simulator_build,
        "trace_id": trace_id,
        "top_speed_interpretation": "highest observed; not a physical maximum",
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def load_limits(path: Path) -> VehicleLimits:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("vehicle limits YAML must be a mapping")
    return VehicleLimits(*(
        float(payload[key]) for key in (
            "top_speed_mps", "accel_mps2", "brake_mps2",
            "lateral_mps2", "control_delay_s",
        )
    ))
