"""Spatial comparisons must exclude startup and gaps between separate visits."""

from training.analyze_corner_exit import metrics, steady_rows


def test_steady_selection_excludes_partial_passes_without_wall_time_bias():
    rows = [dict(status='ok', elapsed_s=10 + index*.1,
                 path_arc_m=arc)
            for index, arc in enumerate([0, 10, 20, 30]*5)]
    selected = steady_rows(rows)
    assert {row['route_pass'] for row in selected} == {2, 3}
    assert [row['path_arc_m'] for row in selected] == [0, 10, 20, 30]*2


def test_command_variation_does_not_count_a_jump_between_separate_visits():
    rows = []
    for index, throttle in ((0, .5), (1, .5), (20, -.5), (21, -.5)):
        rows.append(dict(sample_index=index, throttle=throttle, steering=0, speed_mps=5,
                         final_target_mps=5, profile_target_mps=7,
                         policy_target_mps=7, route_curvature_inv_m=-.1,
                         route_curvature_1m_inv_m=0, cross_track_m=.1,
                         heading_error_rad=.02, exit_preview_used='True',
                         limiting_stage='safety'))
    report = metrics(rows)
    assert report['throttle_step_p95'] == 0
    assert report['direct_accel_brake_switches_per_1000'] == 0
    assert report['braking_fraction'] == .5
