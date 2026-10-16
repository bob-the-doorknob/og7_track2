"""Compare complete simulator trials and spatially matched corner-exit traces.

Trace speed_mps is measured odometry speed; final_target_mps is the command
passed to the speed controller. Command variation is a smoothness proxy, not
a measurement of physical jerk. Referee JSONs remain the lap-time evidence.
"""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from team_driver.diagnostics import summarize
from team_driver.map_guard import MapGuard
from team_driver.pure_pursuit import PurePursuit


def steady_rows(rows):
    """Exclude initial reset and partial route passes without using wall pacing."""
    valid = [r for r in rows if r['status'] == 'ok' and float(r['elapsed_s']) >= 5]
    # The CSV arc wraps at the map origin, not necessarily the referee line.
    # Discard the first two wraps and the final partial route pass in both runs.
    arcs = np.asarray([float(r['path_arc_m']) for r in valid])
    if len(arcs) == 0:
        raise ValueError('no valid trace rows')
    wraps = np.r_[False, np.diff(arcs) < -10]
    laps = np.cumsum(wraps)
    selected = (laps >= 2) & (laps < laps[-1])
    return [dict(r, route_pass=int(lap)) for r, lap, keep in zip(valid, laps, selected) if keep]


def metrics(rows):
    if not rows:
        raise ValueError('no samples in selected section')
    def values(key):
        return np.asarray([float(r[key]) for r in rows])
    speed, target, throttle = (values(k) for k in
                              ('speed_mps', 'final_target_mps', 'throttle'))
    # Adjacent samples only: do not count the jump from one visit to the next.
    adjacent = np.diff(values('sample_index')) == 1
    throttle_steps = np.abs(np.diff(throttle))[adjacent]
    steering_steps = np.abs(np.diff(values('steering')))[adjacent]
    target_steps = np.abs(np.diff(target))[adjacent]
    signs = np.sign(np.where(np.abs(throttle) > .01, throttle, 0))
    switches = (signs[:-1] * signs[1:] < 0)[adjacent]
    return dict(samples=len(rows), speed_median=float(np.median(speed)),
                speed_min=float(np.min(speed)), speed_p10=float(np.quantile(speed, .1)),
                target_median=float(np.median(target)), target_min=float(np.min(target)),
                profile_target_median=float(np.median(values('profile_target_mps'))),
                policy_target_median=float(np.median(values('policy_target_mps'))),
                route_curvature_median=float(np.median(values('route_curvature_inv_m'))),
                curvature_1m_median=float(np.median(values('route_curvature_1m_inv_m'))),
                cross_track_p95=float(np.quantile(np.abs(values('cross_track_m')), .95)),
                cross_track_max=float(np.max(np.abs(values('cross_track_m')))),
                heading_error_p95=float(np.quantile(np.abs(values('heading_error_rad')), .95)),
                heading_error_max=float(np.max(np.abs(values('heading_error_rad')))),
                braking_fraction=float(np.mean(throttle < -.01)),
                preview_fraction=float(np.mean([r['exit_preview_used'] == 'True' for r in rows])),
                limiter_counts=dict(Counter(r['limiting_stage'] for r in rows)),
                throttle_step_p95=float(np.quantile(throttle_steps, .95)) if len(throttle_steps) else 0,
                target_step_p95_mps=float(np.quantile(target_steps, .95)) if len(target_steps) else 0,
                target_step_max_mps=float(np.max(target_steps)) if len(target_steps) else 0,
                steering_step_p95=float(np.quantile(steering_steps, .95)) if len(steering_steps) else 0,
                throttle_jumps_over_half_per_1000=float(1000*np.mean(throttle_steps > .5)) if len(throttle_steps) else 0,
                direct_accel_brake_switches_per_1000=float(1000*np.mean(switches)) if len(switches) else 0)


def inspect(directory, run_id, team, guard):
    score = json.loads((directory / f'{team}__{run_id}.json').read_text())
    with (directory / f'{run_id}.csv').open() as stream:
        all_rows = list(csv.DictReader(stream))
    rows = steady_rows(all_rows)
    outside = [r for r in rows if not guard._free_disk(
        np.asarray((float(r['x_m']), float(r['y_m']))))]
    section = [r for r in rows if 19 <= float(r['path_arc_m']) < 22]
    bins = []
    for start in np.arange(16, 26, .25):
        selected = [r for r in rows if start <= float(r['path_arc_m']) < start+.25]
        if selected:
            bins.append(dict(arc_m=float(start+.125), **metrics(selected)))
    per_pass = []
    for lap in sorted({r['route_pass'] for r in section}):
        selected = [r for r in section if r['route_pass'] == lap]
        exiting = [r for r in selected if float(r['path_arc_m']) >= 21.5]
        if exiting:
            per_pass.append(dict(route_pass=lap,
                                 minimum_speed=min(float(r['speed_mps']) for r in selected),
                                 exit_speed_median=float(np.median([float(r['speed_mps']) for r in exiting]))))
    representative = min(section, key=lambda r: float(r['final_target_mps']))
    return dict(run_id=run_id, score={k: score[k] for k in (
        'status', 'scored', 'laps_completed', 'collisions', 'best_lap_time', 'total_time', 'environment')},
        diagnostics=summarize(all_rows), whole_track=metrics(rows),
        map_disk_check=dict(radius_m=guard.footprint_m, samples=len(rows),
                            occupied_or_unknown_samples=len(outside),
                            note='Static-map footprint proxy, not a full vehicle-dynamics or boundary proof.'),
        section=metrics(section), per_pass=per_pass,
        representative_low_target_sample=representative, bins=bins)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('reference')
    parser.add_argument('candidate')
    parser.add_argument('--team', default='OG7')
    maps = Path(__file__).resolve().parents[4] / 'maps'
    parser.add_argument('--map', type=Path, default=maps / 'icra26_compete.yaml')
    parser.add_argument('--line', type=Path, default=maps / 'icra26_compete_racing_line.csv')
    args = parser.parse_args()
    guard = MapGuard(args.map, PurePursuit(args.line))
    items = [inspect(args.directory, run_id, args.team, guard)
             for run_id in (args.reference, args.candidate)]
    report = dict(note='Section is racing-line arc [19,22) m. Steady complete route passes '
                  'after two arc wraps, excluding the final partial pass; these are not '
                  'referee lap boundaries. Throttle variation measures command smoothness, '
                  'not physical acceleration or jerk.', reference=items[0], candidate=items[1])
    output = args.directory / f'{args.candidate}.comparison'
    output.with_suffix('.comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    for item in items:
        print(json.dumps({k:item[k] for k in ('run_id', 'score', 'section', 'per_pass')}, indent=2))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('JSON written; matplotlib unavailable for plot.')
        return
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for item in items:
        bins = item['bins']
        arcs = [b['arc_m'] for b in bins]
        label = 'Baseline' if item is items[0] else 'Candidate controller'
        axes[0].plot(arcs, [b['speed_median'] for b in bins], label=label)
        axes[1].plot(arcs, [b['target_median'] for b in bins], label=label)
        axes[2].plot(arcs, [100*b['braking_fraction'] for b in bins], label=label)
    for ax in axes:
        ax.axvspan(19, 22, alpha=.12, color='orange')
        ax.grid(alpha=.25)
        ax.legend()
    axes[0].set_ylabel('Actual speed (m/s)')
    axes[1].set_ylabel('Final target (m/s)')
    axes[2].set_ylabel('Braking samples (%)')
    axes[2].set_xlabel('Distance along racing line (m)')
    fig.suptitle('Simulator telemetry: median in 0.25 m bins')
    fig.tight_layout()
    fig.savefig(str(output)+'.png', dpi=150)


if __name__ == '__main__':
    main()
