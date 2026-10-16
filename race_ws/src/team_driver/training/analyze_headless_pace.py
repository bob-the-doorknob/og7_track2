"""Compare spatial telemetry and original referee results across environments.

The fitted lag is a response estimate, not a measurement of pure actuator delay.
It includes simulator, bridge, callback and vehicle-response effects. Wall lag
must not be confused with the referee's simulated lap seconds.
"""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from training.analyze_corner_exit import metrics, steady_rows


def response_lag(rows):
    def values(key): return np.asarray([float(r[key]) for r in rows])
    t, speed, yaw, steer = (values(k) for k in
        ('elapsed_s', 'speed_mps', 'yaw_rate_radps', 'steering'))
    actual = yaw / np.maximum(speed, .5)
    commanded = np.tan(steer * .5236) / .324
    fits = []
    for lag in np.arange(0, .201, .005):
        shifted = np.interp(t-lag, t, commanded)
        mask = ((t >= t[0]+.2) & (speed > 2) & (speed < 6)
                & (np.abs(shifted) > .05) & (np.abs(shifted) < .5))
        if np.count_nonzero(mask) < 40:
            continue
        gain = np.dot(actual[mask], shifted[mask]) / np.dot(shifted[mask], shifted[mask])
        mse = np.mean((actual[mask]-gain*shifted[mask])**2)
        fits.append((float(mse), float(lag), float(gain)))
    if not fits:
        return None
    mse, lag, gain = min(fits)
    return dict(lag_wall_s=lag, gain=gain, curvature_mse=mse)


def inspect(trace, score):
    with trace.open() as f:
        all_rows = list(csv.DictReader(f))
    rows = steady_rows(all_rows)
    report = metrics(rows)
    section = [r for r in rows if 19 <= float(r['path_arc_m']) < 22]
    if section:
        report['corner_exit_19_22m'] = metrics(section)
    report['all_status_counts'] = dict(Counter(r['status'] for r in all_rows))
    report['pose_ips_fraction'] = float(np.mean([r['pose_source'] == 'ips' for r in rows]))
    for key in ['wall_dt_s', 'control_dt_s', 'odom_age_s', 'ips_odom_delta_m',
                'prediction_horizon_s', 'motion_interval_s']:
        values = np.asarray([float(r[key]) for r in rows if r.get(key, '') != ''])
        if len(values):
            report[key] = dict(zip(['p50', 'p95', 'max'], np.quantile(values, [.5,.95,1]).tolist()))
    report['response_lag'] = response_lag(rows)
    result = json.loads(score.read_text())
    report['referee'] = {k: result[k] for k in ['run_id', 'status', 'scored',
        'laps_completed', 'collisions', 'best_lap_time', 'total_time', 'environment']}
    bins = []
    for start in np.arange(0, 51, 1.):
        selected = [r for r in rows if start <= float(r['path_arc_m']) < start+1]
        if selected:
            bins.append(dict(arc_m=float(start+.5), **metrics(selected)))
    return dict(summary=report, bins=bins)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True, help='baseline CSV')
    parser.add_argument('--candidate', type=Path, required=True, help='candidate CSV')
    parser.add_argument('--baseline-score', type=Path)
    parser.add_argument('--candidate-score', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='output prefix')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    data = {}
    for label in ('baseline', 'candidate'):
        trace = getattr(args, label)
        score = getattr(args, label+'_score') or trace.with_name('OG7__'+trace.stem+'.json')
        data[label] = inspect(trace, score)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.json').write_text(json.dumps(data, indent=2)+'\n')
    for label, run in data.items():
        s=run['summary']
        print(label, json.dumps({k:s[k] for k in ['referee', 'cross_track_p95',
            'cross_track_max', 'heading_error_p95', 'response_lag']}))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(10,8), sharex=True)
        for label, run in data.items():
            x=[b['arc_m'] for b in run['bins']]
            for ax, key in zip(axes, ['speed_median','target_median','cross_track_p95']):
                ax.plot(x, [b[key] for b in run['bins']], label=label.capitalize())
                ax.grid(alpha=.2)
        for ax, label in zip(axes,['Actual speed (m/s)','Final target (m/s)','Tracking error, p95 (m)']):
            ax.set_ylabel(label)
            ax.legend()
        axes[-1].set_xlabel('Distance along racing line (m)')
        fig.suptitle('Linux headless: spatially matched telemetry, 1 m bins')
        fig.tight_layout()
        fig.savefig(args.output.with_suffix('.png'), dpi=150)


if __name__ == '__main__':
    main()
