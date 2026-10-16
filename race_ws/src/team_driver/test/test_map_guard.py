"""Known-map certification must fail closed when pose or map is invalid."""

import numpy as np
from pathlib import Path

from team_driver.map_guard import MapGuard
from team_driver.pure_pursuit import PurePursuit


def test_real_track_certifies_only_a_valid_aligned_pose():
    maps = Path(__file__).resolve().parents[4] / "maps"
    path = PurePursuit(maps / "icra26_compete_racing_line.csv")
    guard = MapGuard(maps / "icra26_compete.yaml", path)
    point = path._point_at(2.0)
    tangent = path._point_at(2.1) - path._point_at(1.9)
    yaw = np.arctan2(tangent[1], tangent[0])
    assert guard.certifies_clear_horizon(tuple(point), yaw, speed=8.0, steering=0.0)
    assert not guard.certifies_clear_horizon((1e3, 1e3), yaw, 8.0, 0.0)
    assert not guard.certifies_clear_horizon(tuple(point), np.nan, 8.0, 0.0)
    assert not guard.certifies_clear_horizon(tuple(point), yaw + np.pi, 8.0, 0.0)
    assert not guard.certifies_clear_horizon(tuple(point), yaw, 8.0, 0.5)
