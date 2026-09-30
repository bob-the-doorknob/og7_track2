# Residual RL on the clean sub-12 classical driver

The preserved classical reference is
`race_ws/src/team_driver/config/fast_corner_24_map72_safety28_params.yaml`.
Its two ten-lap scored runs had zero collisions and penalties, with best laps
of 11.652 and 11.639 s and totals of 117.874 and 117.428 s. The original
config and earlier `fast_corner_15_params.yaml` are unchanged.

The new race config is
`race_ws/src/team_driver/config/fast_corner_24_residual_v2_sub12_params.yaml`.
It uses the same racing line, 24 m/s^2 lateral planning allowance, 7.2 m/s
requested top speed, short lookahead, map guard, and 2.8 m/s^2 LiDAR braking
cap. It adds a bounded v2 residual actor loaded from the package's weights
directory: `residual_policy_v2_sub12_candidate.npz`. The default driver
config remains separate; select this config explicitly to run the actor.

## Training and selection

The trainer now accepts `--baseline-config` and reads these settings from the
classical config. It records the config SHA-256 in each checkpoint's metadata.
It also ends a training episode safely if telemetry goes stale before an
action. The first resumed attempt stopped on that previously unhandled
condition at about 3,700 steps; it produced no selected actor.

The completed run `training/runs/task_sub12_v2_s50` resumed the clean v2
checkpoint from `task5_map72_safe_action_v2_s47/step_0002000.zip`, then ran
4,000 new steps with seed 50 and one simulator frame per action. Its first
1,000 steps collected zero-residual baseline transitions. The replay buffer
was fresh because the source checkpoint had none. Random exploration
included off-course episodes and collisions; these are training events, not
scored race results. The deterministic 4,000-step actor and metadata were
copied byte-for-byte to `race_ws/src/team_driver/weights/`.

| Actor checkpoint | Timed laps | Collisions | Best lap | Total |
| --- | ---: | ---: | ---: | ---: |
| 2,000 new steps | 3 | 0 | 11.182 s | 33.719 s |
| 3,000 new steps | 3 | 0 | 11.298 s | 34.130 s |
| 4,000 new steps | 3 | 0 | 11.200 s | 33.671 s |
| Packaged 4,000-step actor | 10 | 0 | **10.960 s** | **111.841 s** |
| Packaged actor repeat | 10 | 0 | 11.100 s | 112.697 s |

All five scored runs had zero penalties. Every timed lap in both ten-lap
runs was below 12 seconds. The full run improves the classical repeat by
0.679 s on best lap and 5.587 s over ten laps. The unedited referee JSON
files are in `results/submitted/` under `sub12rl_step*` and
`sub12rl_packaged_*` names.

This establishes performance on the connected AutoDRIVE compete build and
the current `icra26_compete` track. The real-time factor was about 0.66-0.67
in the RL runs. The training environment applies a nominal 25 ms controller
step, while the race driver uses wall-clock controller timing; the scored
runtime results include that difference.

To continue training, use `training.train_sac` with
`--baseline-config config/fast_corner_24_map72_safety28_params.yaml`,
`--observation-version 2`, and a new run ID. Training must run with the bridge
connected and no manual driver or referee. The `.npz` actor is the compact
race artifact; the SAC `.zip` and replay buffer remain local in the ignored
training run directory.
