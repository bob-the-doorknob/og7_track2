"""Closed-lap targets must honor cornering, braking, and acceleration."""

import numpy as np
import pytest

from team_driver.pure_pursuit import PurePursuit
from team_driver.speed_profile import SpeedProfile
from team_driver.vehicle_limits import VehicleLimits


def test_gentle_bend_keeps_more_speed_than_sharp_bend():
    from team_driver.speed_profile import closed_lap_limits

    curvature = np.zeros(40)
    curvature[10] = 0.03
    curvature[30] = 0.50
    limits = VehicleLimits(6.0, 2.0, 2.0, 2.0, 0.0)
    speed = closed_lap_limits(curvature, spacing=0.5, limits=limits)
    assert speed[10] == pytest.approx(6.0)
    assert speed[30] == pytest.approx(2.0)
    assert speed[33] > speed[31] >= speed[30]


def test_braking_and_acceleration_hold_across_closed_seam():
    from team_driver.speed_profile import closed_lap_limits

    curvature = np.zeros(16)
    curvature[0] = 0.5
    limits = VehicleLimits(6.0, 2.0, 2.0, 2.0, 0.0)
    speed = closed_lap_limits(curvature, spacing=0.5, limits=limits)
    assert speed[0] == pytest.approx(2.0)
    assert speed[-1] ** 2 <= speed[0] ** 2 + 2.0 * 2.0 * 0.5 + 1e-6
    assert speed[1] ** 2 <= speed[0] ** 2 + 2.0 * 2.0 * 0.5 + 1e-6


def test_straight_uses_observed_vehicle_speed_without_a_fixed_cap():
    from team_driver.speed_profile import closed_lap_limits

    limits = VehicleLimits(9.2, 2.0, 2.0, 2.0, 0.0)
    assert np.allclose(closed_lap_limits(np.zeros(30), 0.5, limits), 9.2)


def test_future_features_are_finite_across_path_seam():
    from pathlib import Path

    path = PurePursuit(Path(__file__).resolve().parents[4] / "maps" /
                       "icra26_compete_racing_line.csv")
    profile = SpeedProfile(path, limits=VehicleLimits(6.3, 2.8, 2.8, 15.0, 0.0))
    curvature, targets = profile.future_features(tuple(path._point_at(path.total_length - 0.1)))
    assert len(curvature) == len(targets) == 3
    assert np.all(np.isfinite(curvature))
    assert np.all(np.isfinite(targets))
    assert np.all(np.asarray(targets) > 0.0)


