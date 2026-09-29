"""Vehicle limits must come from clean measured telemetry."""

from pathlib import Path

import numpy as np
import pytest

_COLUMNS = (
    "time_s", "x_m", "y_m", "yaw_rad", "speed_mps", "throttle",
    "steering", "arc_m", "clearance_m", "lap_count", "collision_count",
)


def sample_rows() -> np.ndarray:
    count = 101
    rows = np.zeros((count, len(_COLUMNS)), dtype=np.float64)
    rows[:, _COLUMNS.index("time_s")] = np.arange(count) * 0.1
    speed = np.concatenate((
        np.arange(31) * 0.1,
        3.0 - np.arange(1, 31) * 0.1,
        np.full(40, 2.0),
    ))
    rows[:, _COLUMNS.index("speed_mps")] = speed
    rows[:31, _COLUMNS.index("throttle")] = 0.5
    rows[31:61, _COLUMNS.index("throttle")] = -0.2
    rows[61:, _COLUMNS.index("throttle")] = 0.2
    rows[61:, _COLUMNS.index("yaw_rad")] = np.arange(40) * 0.05
    rows[:, _COLUMNS.index("clearance_m")] = 3.0
    return rows


def test_estimate_limits_uses_measured_speed_and_braking():
    from training.vehicle_calibration import estimate_limits

    limits = estimate_limits(sample_rows())
    assert limits.top_speed_mps == pytest.approx(3.0)
    assert limits.accel_mps2 == pytest.approx(1.0)
    assert limits.brake_mps2 == pytest.approx(1.0)
    assert limits.lateral_mps2 == pytest.approx(0.95)
    assert limits.control_delay_s >= 0.0


def test_estimate_limits_rejects_empty_nonfinite_and_collision_rows():
    from training.vehicle_calibration import estimate_limits

    with pytest.raises(ValueError):
        estimate_limits(np.empty((0, len(_COLUMNS))))
    bad = sample_rows()
    bad[4, _COLUMNS.index("speed_mps")] = np.nan
    with pytest.raises(ValueError):
        estimate_limits(bad)
    collided = sample_rows()
    collided[5:, _COLUMNS.index("collision_count")] = 1
    with pytest.raises(ValueError):
        estimate_limits(collided)


def test_estimate_limits_uses_straight_coasting_without_active_braking():
    from training.vehicle_calibration import estimate_limits

    rows = sample_rows()
    rows[31:61, _COLUMNS.index("throttle")] = 0.0
    assert estimate_limits(rows).brake_mps2 == pytest.approx(1.0)


def test_limits_yaml_roundtrip_preserves_provenance(tmp_path: Path):
    from team_driver.vehicle_limits import VehicleLimits, load_limits, save_limits

    limits = VehicleLimits(3.0, 1.0, 1.0, 1.0, 0.05)
    path = tmp_path / "vehicle_limits.yaml"
    save_limits(path, limits, sample_count=101,
                simulator_build="2026-icra compete", trace_id="calibration-a")
    assert load_limits(path) == limits
    assert "calibration-a" in path.read_text(encoding="utf-8")


def test_combine_limits_keeps_highest_clean_speed_and_conservative_dynamics():
    from team_driver.vehicle_limits import VehicleLimits, combine_limits

    older = VehicleLimits(4.8, 3.4, 2.8, 1.65, 0.011)
    faster = VehicleLimits(6.3, 2.8, 3.9, 1.10, 0.016)
    assert combine_limits([older, faster]) == VehicleLimits(
        6.3, 2.8, 2.8, 1.65, 0.016)
