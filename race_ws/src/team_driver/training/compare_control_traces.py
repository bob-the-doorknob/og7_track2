"""Compare real practice traces; reconstructed wall laps are not scored laps."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from team_driver.diagnostics import summarize
from team_driver.pure_pursuit import PurePursuit


def inspect(filename, path):
    with filename.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    values = {name: np.asarray([float(row[name] or "nan") for row in rows])
              for name in ("elapsed_s", "wall_dt_s", "speed_mps", "throttle",
                           "final_target_mps", "cross_track_m", "x_m", "y_m")}
    arc = np.asarray([path._project((x, y))[2]
                      for x, y in zip(values["x_m"], values["y_m"])])
    wraps = np.flatnonzero(np.diff(arc) < -path.total_length / 2) + 1
    selected = np.zeros(len(rows), dtype=bool)
    wall_laps = []
    for start, end in zip(wraps[:-1], wraps[1:]):
        if np.max(values["wall_dt_s"][start:end]) > .5:
            continue
        wall_laps.append(float(values["elapsed_s"][end]-values["elapsed_s"][start]))
        selected[start:end] = True
    report = summarize(rows)
    duration = report["retained_duration_s"]
    jumps = np.abs(np.diff(values["throttle"]))
    report.update(
        file=filename.name,
        switches_per_wall_second=report["accel_brake_switches"] / duration,
        command_jumps_over_0_5=int(np.sum(jumps > .5)),
        command_jumps_over_0_5_per_wall_second=float(np.sum(jumps > .5) / duration),
        reconstructed_complete_wall_laps_s=wall_laps,
        telemetry_fault_samples=[{name: row[name] for name in
                                 ("elapsed_s", "odom_age_s", "wall_dt_s", "status")}
                                for row in rows if row["status"] != "ok"],
        complete_lap_mean_speed_mps=float(np.mean(values["speed_mps"][selected])),
        complete_lap_abs_cross_track_p95_m=float(np.quantile(
            np.abs(values["cross_track_m"][selected]), .95)),
    )
    bins = np.linspace(0, path.total_length, 101)
    indexes = np.clip(np.digitize(arc, bins) - 1, 0, 99)
    spatial_speed = np.asarray([np.median(values["speed_mps"][selected & (indexes == index)])
                                if np.any(selected & (indexes == index)) else np.nan
                                for index in range(100)])
    return report, spatial_speed, (bins[:-1] + bins[1:]) / 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--line", type=Path, required=True)
    args = parser.parse_args()
    path = PurePursuit(args.line)
    reference, original, arcs = inspect(args.reference, path)
    candidate, smooth, _ = inspect(args.candidate, path)
    report = dict(reference=reference, candidate=candidate,
                  note="Trace contains wall time and no lap/collision telemetry. No scored performance claim.",
                  matched_path_median_speed_difference_mps=float(np.nanmean(smooth-original)))
    output = args.candidate.with_suffix(".comparison.json")
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(11, 7))
    axes[0].plot(arcs, original, label="Original")
    axes[0].plot(arcs, smooth, label="Smooth candidate")
    axes[0].set_xlabel("Distance along racing line (m)")
    axes[0].set_ylabel("Median measured speed (m/s)")
    axes[0].legend()
    names = ["Original", "Smooth candidate"]
    axes[1].bar(names, [reference["command_jumps_over_0_5_per_wall_second"],
                        candidate["command_jumps_over_0_5_per_wall_second"]])
    axes[1].set_ylabel("Command jumps >0.5 per wall second")
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("Actual practice traces — speed by track position and command oscillation")
    fig.tight_layout()
    fig.savefig(args.candidate.with_suffix(".comparison.png"), dpi=150)


if __name__ == "__main__":
    main()
