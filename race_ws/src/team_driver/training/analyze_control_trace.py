"""Inspect measured control transitions without treating replay as a race."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from team_driver.diagnostics import summarize
from team_driver.speed_controller import SpeedController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    with args.trace.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    def values(name):
        return np.asarray([float(row[name] or "nan") for row in rows])
    t = values("elapsed_s")
    throttle = values("throttle")
    target = values("final_target_mps")
    speed = values("speed_mps")
    policy = values("policy_target_mps")
    direction = np.sign(np.where(np.abs(throttle) > 0.01, throttle, 0))
    events = np.flatnonzero(direction[1:] * direction[:-1] < 0) + 1
    rapid = events[np.diff(t)[events - 1] < 0.05]
    stable = rapid[np.abs(np.diff(target)[rapid - 1]) < 0.1]
    summary = summarize(rows)
    summary.update(
        direct_switches=len(events), switches_within_50ms=len(rapid),
        rapid_switches_target_change_under_0_1mps=len(stable),
        wall_interval_quantiles_s=np.quantile(values("wall_dt_s"), [.5, .95, .99]).tolist(),
        compute_quantiles_ms=np.quantile(values("compute_ms"), [.5, .95, .99]).tolist(),
        max_odom_age_s=float(np.max(values("odom_age_s"))),
        mean_safety_target_reduction_mps=float(np.mean(np.maximum(policy-target, 0))),
        max_final_target_drop_mps=float(np.max(-np.diff(target))),
    )
    summary["representative_rapid_switches"] = [
        {name: [rows[index-1][name], rows[index][name]] for name in
         ("elapsed_s", "speed_mps", "final_target_mps", "throttle", "path_clearance_m")}
        for index in stable[:5]]
    counts = Counter(np.floor(t[events]).astype(int))
    window = int(counts.most_common(1)[0][0]) if counts else int(t[0])
    summary["most_switches_in_one_wall_second"] = counts.get(window, 0)
    summary["zoom_start_s"] = window
    controllers = [SpeedController(throttle_min=-1.0, throttle_max=1.0,
                                   active_braking=True, approach_band_mps=band)
                   for band in (0.0, 0.3)]
    replays = [np.asarray([controller.command(float(row["final_target_mps"]),
                                             float(row["speed_mps"]),
                                             float(row["control_dt_s"]))
                          for row in rows]) for controller in controllers]
    braking_mask = throttle < -0.01
    summary["offline_fixed_telemetry_replay"] = dict(
        baseline_max_command_error=float(np.max(np.abs(replays[0]-throttle))),
        candidate_approach_band_mps=0.3,
        baseline_command_jumps_over_0_5=int(np.sum(np.abs(np.diff(replays[0])) > 0.5)),
        candidate_command_jumps_over_0_5=int(np.sum(np.abs(np.diff(replays[1])) > 0.5)),
        max_change_to_recorded_braking=float(np.max(np.abs(replays[1][braking_mask]-throttle[braking_mask]))),
        note="Fixed recorded speeds and targets; does not predict new lap time or closed-loop stability.")
    destination = args.trace.with_suffix(".analysis.json")
    destination.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    selected = (t >= window - .25) & (t <= window + 1.25)
    axes[0].plot(t[selected], speed[selected], label="Measured speed")
    axes[0].plot(t[selected], policy[selected], label="Policy target", alpha=.65)
    axes[0].plot(t[selected], target[selected], label="After safety", drawstyle="steps-post")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].legend(loc="best")
    axes[1].plot(t[selected], throttle[selected], drawstyle="steps-post", label="Recorded command")
    axes[1].plot(t[selected], replays[1][selected], drawstyle="steps-post",
                 label="Candidate on fixed recorded telemetry", alpha=.8)
    axes[1].legend(loc="best")
    axes[1].axhline(0, color="gray", linewidth=.7)
    axes[1].set_ylabel("Throttle command")
    axes[2].plot(t[selected], speed[selected]-target[selected], label="Speed above target")
    axes[2].axhline(.05, color="red", linestyle="--", label="Existing brake threshold")
    axes[2].set_ylabel("Speed error (m/s)")
    axes[2].set_xlabel("Elapsed wall time (s)")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("Measured acceleration/braking switching — diagnostic practice run")
    fig.tight_layout()
    fig.savefig(args.trace.with_suffix(".analysis.png"), dpi=150)


if __name__ == "__main__":
    main()
