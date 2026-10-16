"""Diagnostic traces must preserve brief events without altering controls."""

import csv

import pytest


def test_trace_keeps_recent_samples_and_does_not_overwrite(tmp_path):
    from team_driver.diagnostics import ControlTrace

    destination = tmp_path / "trace.csv"
    trace = ControlTrace(destination, max_samples=3)
    for index in range(5):
        trace.record({"elapsed_s": index, "throttle": -0.2 if index == 3 else 0.5})
    assert trace.dropped_samples == 2
    trace.close()
    rows = list(csv.DictReader(destination.open()))
    assert [float(row["elapsed_s"]) for row in rows] == [2, 3, 4]
    assert float(rows[1]["throttle"]) == -0.2
    with pytest.raises(FileExistsError):
        ControlTrace(destination)


def test_summary_separates_safety_braking_and_stale_telemetry():
    from team_driver.diagnostics import summarize

    rows = [
        dict(elapsed_s=0, wall_dt_s=0.025, odom_age_s=0.01,
             speed_mps=5, baseline_target_mps=5, policy_target_mps=5,
             final_target_mps=5, throttle=0.5, status="ok"),
        dict(elapsed_s=0.025, wall_dt_s=0.025, odom_age_s=0.01,
             speed_mps=5.1, baseline_target_mps=5, policy_target_mps=5,
             final_target_mps=5, throttle=-0.03, status="ok"),
        dict(elapsed_s=0.05, wall_dt_s=0.025, odom_age_s=0.01,
             speed_mps=5, baseline_target_mps=5, policy_target_mps=5,
             final_target_mps=3, throttle=-0.6, status="ok"),
        dict(elapsed_s=0.25, wall_dt_s=0.2, odom_age_s=0.6,
             speed_mps=5, throttle=0, status="stale_odom"),
    ]
    report = summarize(rows)
    assert report["near_target_braking_samples"] == 1
    assert report["safety_limited_samples"] == 1
    assert report["stale_odom_samples"] == 1
    assert report["callback_gaps_over_100ms"] == 1
    assert report["accel_brake_switches"] == 1
    assert report["samples"] == 4


def test_zero_throttle_does_not_hide_accel_brake_switches():
    from team_driver.diagnostics import summarize

    report = summarize([dict(throttle=value) for value in (0.5, 0, -0.1, 0, 0.4)])
    assert report["accel_brake_switches"] == 2


def test_empty_trace_summary_is_valid():
    from team_driver.diagnostics import summarize

    assert summarize([])["samples"] == 0


def test_trace_persists_path_and_braking_diagnostics(tmp_path):
    from team_driver.diagnostics import ControlTrace

    destination = tmp_path / 'geometry.csv'
    trace = ControlTrace(destination)
    trace.record(dict(speed_mps=4.5, final_target_mps=4.0, lookahead_m=1.5,
                      route_curvature_inv_m=-.14, route_curvature_1m_inv_m=-.02,
                      target_segment_index=23, target_x_m=3.0, target_y_m=2.0,
                      limiting_stage='safety', braking_reason='overspeed_safety'))
    trace.close()
    with destination.open() as stream:
        row = next(csv.DictReader(stream))
    assert float(row['speed_mps']) == 4.5
    assert float(row['final_target_mps']) == 4.0
    assert float(row['route_curvature_1m_inv_m']) == -.02
    assert row['braking_reason'] == 'overspeed_safety'
