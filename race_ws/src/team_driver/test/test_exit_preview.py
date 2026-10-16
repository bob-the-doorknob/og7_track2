"""Turn-exit preview must keep real obstacle and invalid-map protection."""

import math
from pathlib import Path

import numpy as np

from team_driver.map_guard import MapGuard
from team_driver.pure_pursuit import PurePursuit
from team_driver.safety import LidarSafety
from team_driver.exit_preview import ExitPreview


def test_preview_gates_and_bounded_speed_gain():
    from types import SimpleNamespace
    safety = LidarSafety(horizon=10, max_deceleration=2.8)
    preview = ExitPreview(enabled=True)
    state = SimpleNamespace(cross_track_error=.25, heading_error=0.)
    path = SimpleNamespace(path_state=lambda *args: state,
                           preview_trajectory=lambda *args, **kwargs: straightening_path())
    curves, targets = [0.3, .1], [5, 6]
    profile = SimpleNamespace(future_features=lambda *args: (curves, targets))
    guard = SimpleNamespace(certifies_trajectory=lambda *args, **kwargs: True)
    args = (path, profile, guard, safety, (0,0), 0, 4, .16, 6)
    assert preview.trajectory(*args) is None  # Too far off line.
    state.cross_track_error = 0
    state.heading_error = .25
    assert preview.trajectory(*args) is None
    state.heading_error = 0
    curves[1] = .35
    assert preview.trajectory(*args) is None  # Turn is tightening.
    curves[1] = .1
    curves[0] = .8
    assert preview.trajectory(*args) is None  # Tight exits need the original braking margin.
    curves[0] = .3
    assert preview.trajectory(*args[:-2], .5, 6) is None  # Large steering correction.
    targets[1] = 4
    assert preview.trajectory(*args) is None  # Profile calls for braking.
    targets[1] = 6
    assert preview.trajectory(*args) is not None
    curves[:] = [.02, .07]
    targets[:] = [7.2, 7.0]
    assert preview.trajectory(*args) is not None  # Aligned continuation after exit.
    targets[1] = 5
    assert preview.trajectory(*args) is None  # Next curve needs a lower speed.
    targets[1] = 7
    guard.certifies_trajectory = lambda *args, **kwargs: False
    assert preview.trajectory(*args) is None
    decision = SimpleNamespace(preview_used=True, target_speed=7, arc_clearance=2,
                               steering=0.0)
    old_limit = math.sqrt(2 * 2.8 * (2 - safety.stop_margin))
    assert preview.target_speed(decision, safety) == old_limit + 1
    decision.target_speed = 2  # New obstacle cap must never be overridden.
    assert preview.target_speed(decision, safety) == 2


def straightening_path():
    # Keep the initial short arc, then straighten alongside a parallel wall.
    s = np.arange(0, 6.01, .09)
    y = .08 * (1-np.exp(-s / .5))
    return np.column_stack((s, y))


def parallel_wall_scan(angles, wall_y=.55):
    result = np.full(len(angles), np.inf)
    valid = angles > .02
    result[valid] = wall_y / np.sin(angles[valid])
    return result


def test_parallel_wall_can_be_clear_when_the_follower_straightens():
    angles = np.linspace(-math.pi/2, math.pi/2, 721)
    safety = LidarSafety(horizon=6, max_range=10, max_deceleration=2.8)
    scan = parallel_wall_scan(angles)
    old = safety.apply(scan, angles, .16, 6.0)
    new = safety.apply(scan, angles, .16, 6.0, trajectory=straightening_path())
    assert new.preview_used
    assert old.target_speed < 4.0
    assert new.path_clearance > old.path_clearance + 2.0
    assert new.target_speed > old.target_speed + 1.0


def test_shorter_preview_preserves_an_already_clear_arc():
    angles = np.linspace(-math.pi/2, math.pi/2, 721)
    safety = LidarSafety(horizon=10, max_range=10, max_deceleration=2.8)
    scan = np.full(len(angles), np.inf)
    old = safety.apply(scan, angles, 0, 7.2)
    new = safety.apply(scan, angles, 0, 7.2, trajectory=straightening_path())
    assert new.target_speed == old.target_speed
    assert not new.preview_used


def test_obstacle_on_preview_keeps_braking_even_with_parallel_wall():
    angles = np.linspace(-math.pi/2, math.pi/2, 721)
    safety = LidarSafety(horizon=6, max_range=10, max_deceleration=2.8)
    scan = parallel_wall_scan(angles)
    scan[np.argmin(np.abs(angles-.04))] = .20
    decision = safety.apply(scan, angles, .16, 6.0, trajectory=straightening_path())
    assert decision.target_speed == 0.0
    missing = safety.apply(np.full(len(angles), np.nan), angles, .16, 6.0,
                           trajectory=straightening_path())
    assert not missing.sensor_valid
    assert missing.target_speed == 0.0


def test_invalid_or_unseen_preview_preserves_arc_limit():
    angles = np.linspace(-math.pi/2, math.pi/2, 721)
    safety = LidarSafety(horizon=6, max_range=10, max_deceleration=2.8)
    scan = parallel_wall_scan(angles)
    expected = safety.apply(scan, angles, .16, 6.0)
    for trajectory in (np.array([[0,0],[np.nan,1]]), np.array([[0,0],[-1,0]]),
                       np.array([[0,0],[1,1,1]], dtype=object), np.array([[0,0]])):
        actual = safety.apply(scan, angles, .16, 6.0, trajectory=trajectory)
        assert actual.target_speed == expected.target_speed
        assert not actual.preview_used


def test_crossing_wall_still_limits_speed_at_stopping_distance():
    angles = np.linspace(-math.pi/2, math.pi/2, 721)
    safety = LidarSafety(horizon=6, max_range=10, max_deceleration=2.8)
    scan = 2 / np.maximum(np.cos(angles), 1e-6)
    decision = safety.apply(scan, angles, .16, 6, trajectory=straightening_path())
    assert decision.preview_used
    assert decision.path_clearance < 2
    assert decision.target_speed < 3.2


def test_map_guard_rejects_occupied_cell_inside_footprint():
    guard = MapGuard.__new__(MapGuard)
    guard.resolution = .05
    guard.origin = np.zeros(2)
    guard.width = guard.height = 40
    guard.free = np.ones((40,40), dtype=bool)
    guard.footprint_m = .215
    trajectory = np.array([[0,0],[.05,0]])
    assert guard.certifies_trajectory((1,1), 0, trajectory)
    guard.free[19,22] = False  # Interior cell, away from centre and perimeter.
    assert not guard.certifies_trajectory((1,1), 0, trajectory)


def test_map_guard_checks_full_preview_and_actual_footprint():
    maps = Path(__file__).resolve().parents[4] / "maps"
    path = PurePursuit(maps / "icra26_compete_racing_line.csv")
    guard = MapGuard(maps / "icra26_compete.yaml", path)
    position = tuple(path._point_at(20.0))
    tangent = path._point_at(20.1)-path._point_at(19.9)
    yaw = float(np.arctan2(tangent[1],tangent[0]))
    steering = path.command(position, yaw, 4.0)[0]
    preview = path.preview_trajectory(position, yaw, 4.0, steering, horizon=4.0,
                                      hold_s=.15)
    assert preview is not None
    assert np.allclose(preview[0], [0,0])
    assert guard.certifies_trajectory(position, yaw, preview, footprint_m=.215)
    assert not guard.certifies_trajectory(position, yaw, preview, footprint_m=10)
    assert not guard.certifies_trajectory((1000,1000), yaw, preview)
    assert not guard.certifies_trajectory(position, yaw, np.array([[0,0],[np.nan,1]]))
