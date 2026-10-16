"""Compare official scores and steady-state traces for the wall-exit trial."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from team_driver.diagnostics import summarize
from team_driver.pure_pursuit import PurePursuit


def inspect(directory, run_id, path):
    score_file = directory / ('DeepSpeed__' + run_id + '.json')
    score = json.loads(score_file.read_text())
    trace_file = directory / (run_id + '.csv')
    rows = list(csv.DictReader(trace_file.open()))
    steady = [row for row in rows if float(row['elapsed_s']) >= 60
              and row['status'] == 'ok']
    arc = np.asarray([path._project((float(row['x_m']), float(row['y_m'])))[2]
                      for row in steady])
    values = {key: np.asarray([float(row[key] or 'nan') for row in steady])
              for key in ('speed_mps','final_target_mps','cross_track_m','throttle')}
    bins = []
    for start in np.arange(0, path.total_length, 1):
        selected = (arc >= start) & (arc < start + 1)
        if not np.any(selected):
            continue
        bins.append(dict(arc_m=float(start), samples=int(np.sum(selected)),
                         speed_median=float(np.median(values['speed_mps'][selected])),
                         speed_p10=float(np.quantile(values['speed_mps'][selected],.1)),
                         target_median=float(np.median(values['final_target_mps'][selected])),
                         cross_track_p95=float(np.quantile(np.abs(values['cross_track_m'][selected]),.95)),
                         braking_fraction=float(np.mean(values['throttle'][selected] < -.01)),
                         preview_fraction=float(np.mean([steady[i].get('exit_preview_used') == 'True'
                                               for i in np.flatnonzero(selected)]))))
    diagnostics = summarize(steady)
    full_diagnostics = summarize(rows)
    # Filtering out startup rows is not recorder data loss.
    diagnostics['dropped_samples'] = full_diagnostics['dropped_samples']
    diagnostics['excluded_startup_or_non_ok_samples'] = len(rows) - len(steady)
    return dict(run_id=run_id, score={key:score[key] for key in (
        'status','scored','laps_completed','collisions','best_lap_time','total_time','environment')},
        trace_note='Steady trace after 60 wall seconds; section metrics are not referee lap scores.',
        diagnostics=diagnostics, all_trace_diagnostics=full_diagnostics, bins=bins,
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (score_file,trace_file,directory/(run_id+'_params.yaml'))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('reference')
    parser.add_argument('candidate')
    parser.add_argument('--line',type=Path,required=True)
    args=parser.parse_args()
    path=PurePursuit(args.line)
    reference=inspect(args.directory,args.reference,path)
    candidate=inspect(args.directory,args.candidate,path)
    report=dict(reference=reference,candidate=candidate)
    output=args.directory/(args.candidate+'.analysis')
    Path(str(output)+'.json').write_text(json.dumps(report,indent=2)+'\n')
    for item in (reference,candidate):
        print(json.dumps({key:item[key] for key in ('run_id','score','diagnostics')},indent=2))
        print('Middle-wall bins:',[row for row in item['bins'] if 19<=row['arc_m']<=21])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,7))
    for item in (reference,candidate):
        x=[row['arc_m']+.5 for row in item['bins']]
        axes[0].plot(x,[row['speed_median'] for row in item['bins']],label=item['run_id'])
        axes[1].plot(x,[row['target_median'] for row in item['bins']],label=item['run_id'])
    for ax in axes:
        ax.axvspan(19,21,color='orange',alpha=.15)
        ax.grid(alpha=.25)
        ax.set_xlabel('Distance along racing line (m)')
    axes[0].set_ylabel('Measured median speed (m/s)')
    axes[1].set_ylabel('Median target speed (m/s)')
    axes[0].legend(fontsize=8)
    fig.suptitle('Measured controller response; shaded area is the middle-wall turn exit')
    fig.tight_layout()
    fig.savefig(str(output)+'.png',dpi=150)


if __name__=='__main__':
    main()
