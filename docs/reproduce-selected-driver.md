# Reproducing OG7's selected driver

## Run the packaged model

Training is not needed to run or judge this submission. The selected inference
artifact is `race_ws/src/team_driver/weights/residual_policy_v2_sub12_candidate.npz`.
Its SHA-256 is `e76d8b8baf50bf452c59fc00002fd6efda7aad0c21206302630a77654e9bbe22`.
Build the full race workspace, source its overlay, connect the simulator
bridge, and run `ros2 run team_driver driver`. All runtime assets are local.

Current controller defaults: 7.2 m/s requested top speed, 18 m/s^2 empirical
lateral planning allowance, guarded exit preview enabled with a 0.30 per m
curvature gate, and the original throttle response. Coherent odometry,
motion-based prediction, a predicted speed-profile query, and adaptive
lookahead account for feedback delay; see `headless-pace-2026-10-16.md`.
The explicit config is `fast_corner_18_residual_v2_latency_params.yaml`.
The actor was trained
before this controller tuning, with a 24 m/s^2 planning allowance.

## Regenerate the racing line

The packaged occupancy grid is the supplied compete map, not a new OG7 map.
The official map reader and centreline tool remain unmodified in `scripts/`.
The OG7 racing-line producer is in the driver training directory. From the
repository root, inside the ROS container:

```bash
python3 race_ws/src/team_driver/training/generate_racing_line.py \
  --map maps/icra26_compete.yaml \
  --centerline maps/icra26_compete_centerline.csv \
  --output /tmp/og7-regenerated-racing-line.csv \
  --margin 0.42 --min-clearance 0.30 --iterations 400
```

This writes a separate candidate and does not overwrite the selected line.
The source centreline was produced with the official
`scripts/track_tool.py centerline` tool. Refer to that tool's `--help` for
retracing it from the grid. Numerical reproduction of the selected racing line
must be checked before replacing the packaged asset. The command above was
checked on 16 October: all 272 generated waypoint coordinates exactly matched
the selected line (maximum coordinate difference 0.0 m).

## Training environment and reward

The exported actor uses SAC, Stable-Baselines3 2.3.2, Torch 2.3.1+cpu, and a
28-input, 64/64-hidden-layer, two-output network. Training uses live AutoDRIVE
compete telemetry through `training.gym_env.AutoDriveResidualEnv`, with bounded
residual steering and speed actions over the map72 classical controller.
It is simulator interaction, not supervised training on a fixed dataset.

The reward weights recorded beside the selected actor are: progress +3,
lap completion +20, collision -50, off-course -30, sensor failure -2,
cross-track error -0.1, heading error -0.01, unsafe speed -0.1,
safety-rejected action -0.1, action magnitude -0.12, action change -0.03,
and step time -0.02. `training/gym_env.py` defines the exact terms and episode
termination rules; weights alone do not define the reward calculation.

The selected run resumed `task5_map72_safe_action_v2_s47/step_0002000.zip`
and collected 4,000 additional steps with seed 50, frame skip 1, observation
version 2, and a fresh replay buffer. The first 1,000 steps used zero residual
actions. Metadata records **197.493 seconds** for this resumed stage, with
13 completed episodes. That duration excludes preceding training stages.
Training included exploratory collisions; only deterministic referee runs
were used as race performance evidence.

## Training commands and reproducibility limits

Install the isolated training dependencies using the instructions in
`race_ws/src/team_driver/training/README.md`. These dependencies are for offline
training and must not be installed into the judging runtime environment.
Connect the compete simulator and bridge, then stop all drivers and referees.
From `race_ws/src/team_driver/`, in the ROS-sourced container:

```bash
PYTHONPATH=".:${PYTHONPATH:-}" /opt/deepspeed-training-venv/bin/python -m training.train_sac \
  --training-mode --simulator-variant compete \
  --simulator-build 2026-icra-compete \
  --baseline-config config/fast_corner_24_map72_safety28_params.yaml \
  --observation-version 2 --steps 4000 --baseline-steps 1000 \
  --checkpoint-every 1000 --frame-skip 1 --seed 50 \
  --run-id og7_fresh_v2_s50
```

This starts a **new** actor and is not a recipe for byte-identical selected
weights. To repeat the recorded resumed stage, use a new run ID and add:

```bash
--resume training/runs/task5_map72_safe_action_v2_s47/step_0002000.zip
```

That historical trainer checkpoint is local and ignored; it is not included
in this submission. The compact `.npz` contains inference weights, not the
SAC critic/optimiser state, so it cannot replace that checkpoint for resuming.
Exact retraining of the selected weights is therefore not available from the
repository alone. Live simulator timing and later source changes also prevent
a promise of deterministic retraining. The exported actor, its metadata, and
the producer code are included and sufficient for offline race inference.

Selection used complete zero-collision referee runs, not training reward or
isolated fast laps. The current controller comparison and rejected variants
are documented in `corner-exit-2026-10-16.md`; the clean-clone record is in
`submission-review-2026-10-16.md`.
