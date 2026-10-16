"""Reconstruct the controller's speed limits from a passive pose/scan capture."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from team_driver.map_guard import MapGuard
from team_driver.exit_preview import ExitPreview
from team_driver.pure_pursuit import PurePursuit
from team_driver.residual_policy import ResidualPolicy, pack_observation_v2
from team_driver.safety import LidarSafety
from team_driver.speed_profile import SpeedProfile
from team_driver.vehicle_limits import load_limits, planning_limits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    p = yaml.safe_load(args.config.read_text())["driver"]["ros__parameters"]
    pkg = args.root / "race_ws/src/team_driver"
    path = PurePursuit(args.root / "maps/icra26_compete_racing_line.csv",
                      lookahead_min=p["lookahead_min"], lookahead_max=p["lookahead_max"],
                      lookahead_speed_gain=p["lookahead_speed_gain"])
    guard = MapGuard(args.root / "maps/icra26_compete.yaml", path)
    limits = planning_limits(load_limits(pkg/"config/vehicle_limits.yaml"),
                             p["planning_lateral_mps2"], p["planning_top_speed_mps"])
    profile = SpeedProfile(path, limits=limits)
    safety = LidarSafety(side_margin=p["safety_side_margin"],
                         stop_margin=p["safety_stop_margin"],
                         max_deceleration=p["safety_max_deceleration"],
                         horizon=p["safety_horizon"], max_range=p["max_range"])
    policy = ResidualPolicy.load(pkg / "weights" / p["policy_path"].removeprefix("packaged:"))
    preview = ExitPreview(enabled=args.preview)
    payload = np.load(args.capture)
    angles = payload["angles"]
    data = []
    for sample, scan in zip(payload["samples"], payload["scans"]):
        elapsed, x, y, yaw, speed, observed_steer, throttle, age = sample
        position = (x, y)
        baseline_steer, lookahead, curvature = path.command(position, yaw, speed)
        profile_target = profile.target_speed(position)
        baseline = min(profile_target, math.sqrt(limits.lateral_mps2 / max(abs(curvature), 1e-4)))
        state = path.path_state(position, yaw)
        future_curve, future_target = profile.future_features(position)
        observation = pack_observation_v2(scan, angles, speed, state.heading_error,
                         state.cross_track_error, future_curve, (baseline, *future_target[1:]),
                         baseline_steer, limits.top_speed_mps, p["max_range"])
        delta, multiplier = policy.predict(observation)
        steering = float(np.clip(baseline_steer+delta, -1, 1))
        proposal = min(baseline*multiplier, limits.top_speed_mps)
        certified = guard.certifies_clear_horizon(position, yaw, speed, steering)
        decision = safety.apply(scan, angles, baseline_steer, baseline, steering, proposal,
                                speed_ceiling=limits.top_speed_mps,
                                clear_horizon_certified=certified)
        new = safety.apply(scan, angles, baseline_steer, baseline, steering, proposal,
                           speed_ceiling=limits.top_speed_mps,
                           clear_horizon_certified=certified,
                           trajectory=preview.trajectory(path, profile, guard, safety,
                                       position, yaw, speed, steering, proposal))
        arc = path._project(position)[2]
        data.append((elapsed, x, y, arc, speed, profile_target, baseline, proposal,
                     decision.target_speed, steering, decision.path_clearance,
                     state.cross_track_error, state.heading_error, future_target[1],
                     future_target[2], future_curve[0], future_curve[1], future_curve[2],
                     preview.target_speed(new, safety), new.path_clearance, new.preview_used))
    values = np.asarray(data)
    output = args.capture.with_suffix('.preview' if args.preview else '.limits')
    np.savez_compressed(str(output) + '.npz', values=values)
    bins = []
    for start in np.arange(0, path.total_length, 1):
        selected = values[(values[:,3]>=start)&(values[:,3]<start+1)]
        if not len(selected):
            continue
        med = np.median(selected, axis=0)
        bins.append(dict(arc_m=float(start), x=float(med[1]), y=float(med[2]),
                         speed=float(med[4]), baseline=float(med[6]), proposal=float(med[7]),
                         final=float(med[8]), clearance=float(med[10]),
                         cross_track=float(med[11]), heading_error=float(med[12]),
                         target_1m=float(med[13]), target_3m=float(med[14]),
                         curvature_now=float(med[15]), curvature_1m=float(med[16]),
                         curvature_3m=float(med[17])))
        bins[-1].update(preview_final=float(med[18]), preview_clearance=float(med[19]),
                        preview_fraction=float(np.mean(selected[:,20])))
    report = dict(samples=len(data), track_length_m=path.total_length,
                  worst_safety_reductions=sorted(bins,key=lambda row:row["proposal"]-row["final"],reverse=True)[:12],
                  bins=bins)
    Path(str(output) + '.json').write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({key:report[key] for key in ("samples","track_length_m","worst_safety_reductions")},indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10,10))
    # Plot occupied boundaries only, preserving the organizer's map coordinates.
    yy, xx = np.nonzero(~guard.free)
    ax.scatter(guard.origin[0]+(xx+.5)*guard.resolution,
               guard.origin[1]+(guard.height-yy-.5)*guard.resolution,
               s=.25, color="lightgray")
    ax.plot(path.points[:,0],path.points[:,1],color="black",linewidth=.7)
    points=ax.scatter(values[:,1],values[:,2],c=values[:,7]-values[:,8],s=8,cmap="inferno")
    for item in bins[::4]:
        ax.text(item["x"],item["y"],str(int(item["arc_m"])),fontsize=8)
    fig.colorbar(points,ax=ax,label="Safety target reduction (m/s)")
    ax.set_aspect("equal")
    ax.set_xlabel("Map x (m)")
    ax.set_ylabel("Map y (m)")
    ax.set_title("Current wall-exit slowdown: reconstructed speed limits")
    fig.tight_layout()
    fig.savefig(str(output) + '.png',dpi=130)


if __name__ == "__main__":
    main()
