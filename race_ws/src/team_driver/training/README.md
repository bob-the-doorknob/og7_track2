# AutoDRIVE training environment

## Measure the vehicle before raising speed

`training.vehicle_calibration` only subscribes to telemetry; run it beside
one normal driver outside a scored evaluation. From the `team_driver` source
directory in the ROS-sourced container shell:

```bash
PYTHONPATH=.:$PYTHONPATH python3 -m training.vehicle_calibration \
  --run-id calibration_44_a --simulator-build 2026-icra-compete \
  --duration-s 60
```

The CSV is saved under the ignored `training/runs/<run-id>/telemetry.csv`.
The YAML in `config/vehicle_limits.yaml` records the measured values and trace
provenance. The recorder rejects collisions and insufficient acceleration,
slowing, or corner samples. If the normal driver never issues negative
throttle, its braking estimate uses **straight coasting** with zero throttle;
this is a conservative deceleration floor, not a measurement of maximum
active braking. The `top_speed_mps` field is the highest speed **observed so
far**, not the car's physical maximum. Recalibrate after faster clean runs.

For the first calibration, use the selected 4.4 m/s driver. To measure above
that, stop it and start the classical driver with `-p baseline_only:=true`, a
small increase in `-p target_speed`, and a LiDAR horizon that can support the
trial. Do not use the 4.4 m/s actor outside its training range. The first
three clean traces used targets 4.4, 5.2, and 6.0 m/s. They observed peaks
4.83, 5.51, and 6.30 m/s with zero collisions. The 5.2 and 6.0 trials used
`-p safety_horizon:=10.0`; their raw traces remain local and uncommitted.
The current calibrated YAML combines the highest clean observed speed and
lateral load with the conservative acceleration and coasting deceleration
from those runs. Recreate that combination from the CSVs:

```bash
PYTHONPATH=.:$PYTHONPATH python3 - <<'PY'
from pathlib import Path
import numpy as np
from training.vehicle_calibration import COLUMNS, estimate_limits
from team_driver.vehicle_limits import combine_limits, save_limits

ids = ('calibration_44_a', 'calibration_52_a', 'calibration_60_a')
runs, count = [], 0
for run_id in ids:
    trace = np.genfromtxt(Path('training/runs') / run_id / 'telemetry.csv',
                          delimiter=',', names=True)
    rows = np.column_stack([trace[name] for name in COLUMNS])
    runs.append(estimate_limits(rows))
    count += len(rows)
save_limits(Path('config/vehicle_limits.yaml'), combine_limits(runs),
            sample_count=count, simulator_build='2026-icra-compete',
            trace_id=','.join(ids))
PY
```

The lateral value uses 95% of the 99th percentile clean observed corner
load. A lower 90th-percentile setting caused the first fast candidate to
brake too heavily and run about 27.7 s laps; the revised setting completed
a clean 19.340 s best lap and 194.005 s ten-lap run. See
`docs/fast-racing-progress.md` for the scored comparison.

`gym_env.py` is the Gymnasium environment. It subscribes to
AutoDRIVE LiDAR, odometry, lap count, and collision count, and publishes the
same steering/throttle topics as the race driver. It reuses the race driver's
centreline controller, PI speed controller, LiDAR safety, observation packing,
speed profile, and residual action limits. `action=[0, 0]` produces the
baseline command. The current speed profile targets up to 4.4 m/s on straights and
plans braking before bends using curvature and a 2.0 m/s² deceleration limit.
It follows the generated racing line rather than the original centreline.

Reset publishing is confined to this file. The wrapper can train against the
**compete** build outside a scored run, using the supplied compete racing line.
Stop the manual driver and referee before creating the environment:

```python
from training.gym_env import AutoDriveResidualEnv

env = AutoDriveResidualEnv(simulator_variant="compete", training_mode=True)
try:
    observation, info = env.reset()
    observation, reward, terminated, truncated, info = env.step([0.0, 0.0])
finally:
    env.close()
```

For the separate practice build, supply
`simulator_variant="practice", training_mode=True, centerline_path=...` with
a path traced for that build. This repository contains only compete-build
paths. The simulator does not report its build identity over ROS, so check
the actual AutoDRIVE executable before choosing the variant. The environment
refuses to reset when a referee or another command publisher is visible. Never
use it for a scored run.

## Install training dependencies

The judging image includes Python 3.10, ROS Humble, and NumPy 1.21.5. Keep
their system packages intact. In a shell inside the running container, after
sourcing the **worktree** ROS overlay, install training dependencies on the
container's Linux filesystem:

```bash
source /opt/ros/humble/setup.bash
source /hackathon/.worktrees/task1-residual-rl-baseline/race_ws/install/local_setup.bash
cd /hackathon/.worktrees/task1-residual-rl-baseline/race_ws/src/team_driver
python3 -m venv --without-pip --system-site-packages /opt/deepspeed-training-venv
python3 -m pip install --no-deps \
  --target /opt/deepspeed-training-venv/lib/python3.10/site-packages \
  -r training/requirements.txt
python3 -m pip install --no-deps \
  --target /opt/deepspeed-training-venv/lib/python3.10/site-packages \
  --index-url https://download.pytorch.org/whl/cpu 'torch==2.3.1+cpu'
python3 -m pip install --no-deps \
  --target /opt/deepspeed-training-venv/lib/python3.10/site-packages \
  -r training/requirements-sac.txt
```

The image does not include `ensurepip`, so this virtual environment is made
without its own `pip`; the system `pip` installs into its site-packages.
`--no-deps` avoids replacing system NumPy and the AutoDRIVE websocket stack.
The virtual environment is container-local: recreate it if you delete the
container. When working from a fresh team checkout mounted at `/hackathon`,
replace the long worktree path above with `/hackathon`.

`reset()` requires live telemetry from a connected simulator. `step()` returns Gymnasium's
`(observation, reward, terminated, truncated, info)` tuple.

## Run SAC training

Start the AutoDRIVE bridge and connect the simulator at `127.0.0.1:4567`,
Autonomous mode. Stop `ros2 run team_driver driver` and any referee process.
In the container shell, from the `team_driver` source directory shown above:

```bash
/opt/deepspeed-training-venv/bin/python -m training.train_sac \
  --training-mode \
  --simulator-variant compete \
  --simulator-build 'AutoDRIVE compete executable (verify version locally)' \
  --steps 20000 --baseline-steps 500 --checkpoint-every 2000 \
  --seed 43 --target-speed 4.4 --max-lateral-accel 1.5 \
  --max-braking-decel 2.0 --safety-horizon 5.5
```

For the practice build, change `--simulator-variant practice` and add
`--centerline /path/to/practice_centerline.csv`. The script cannot verify which
executable is open, so enter its actual version/name in `--simulator-build`.
Training explicitly enables resets and must never run during a scored race.

`training/runs/<run-id>/` contains `baseline.zip/.npz/.json`, periodic
`step_*.zip/.npz/.json`, `final.zip/.npz/.json`, and `episodes.jsonl`.
The JSON stores the seed, declared simulator build, reward weights, elapsed
time, speed profile settings, and step count. The `.zip` retains the full SAC trainer; `.npz` is the
small NumPy actor used by the race driver. Run outputs are gitignored. The
checked-in `weights/baseline_residual.npz` is the zero-residual fallback.
The selected 4.4 m/s actor is `weights/residual_policy.npz`, loaded by the
default driver configuration. `config/baseline_44_params.yaml` retains the
classical baseline for comparison.
The earlier `task5_first_1000` checkpoint used a 1.8 m/s centreline baseline.
The `task5_speed_profile_1500b` checkpoint used a 3.0 m/s centreline profile.
Those checkpoints were superseded as the line and speed plan changed. The
observation format stays v1, but the input distribution and controls differ.

The first faster-profile run is `task5_speed_profile_1500b` (seed 42,
1,500 steps). Its exported actor and metadata are checked in as
`weights/speed_profile_1500_candidate.npz` and `.json`. It completed five
single-lap training episodes with zero reported collisions. Those episodes
include exploration and are not a deterministic evaluation of this actor.
To watch the candidate outside a scored run, use the normal bridge and driver
with `--ros-args --params-file ... -p baseline_only:=false` and set
`-p policy_path:=/hackathon/.worktrees/task1-residual-rl-baseline/race_ws/src/team_driver/weights/speed_profile_1500_candidate.npz`.
Stop the training environment first; only one driver may publish commands.
That candidate was trained on the prior 3.0 m/s centreline profile. Do not
use those old weights as a measure of the new 4.4 m/s racing line.

The new run is `task5_racing_line_2000` (seed 42, 2,000 steps). Its exported
actor and metadata are checked in as `weights/racing_line_2000_candidate.npz`
and `.json`. Training recorded six completed single-lap episodes and one
collision episode. These were exploratory training episodes, not deterministic
or scored runs. To try the candidate with the normal driver, use
`-p baseline_only:=false` and
`-p policy_path:=/hackathon/.worktrees/task1-residual-rl-baseline/race_ws/src/team_driver/weights/racing_line_2000_candidate.npz`.
Check the terminal for `Loaded residual policy`; a missing or mistyped path
causes the driver to fall back to the baseline.

The selected run is `task5_racing_line_44_2500b` (seed 43, 2,500 steps) with
the 4.4 m/s target and 5.5 m LiDAR horizon. Its exported actor and metadata
are checked in as `weights/racing_line_44_2500_candidate.npz` and `.json`;
the chosen copy is `weights/residual_policy.npz`. Training episodes included
exploration and one collision. The deterministic scored comparison is in
`docs/track2-44-comparison.md`; its selected-policy runs completed without
collisions and were faster than the baseline runs.

## Offline racing-line search

With the container running, from `race_ws/src/team_driver` in the isolated
worktree, run this only outside a scored race:

```bash
PYTHONPATH=. python3 -m training.optimize_line \
  --map /hackathon/.worktrees/task1-residual-rl-baseline/maps/icra26_compete.yaml \
  --center /hackathon/.worktrees/task1-residual-rl-baseline/maps/icra26_compete_centerline.csv \
  --initial /hackathon/.worktrees/task1-residual-rl-baseline/maps/icra26_compete_racing_line.csv \
  --limits config/vehicle_limits.yaml \
  --output /hackathon/.worktrees/task1-residual-rl-baseline/results/line_candidate.csv \
  --margin 0.28
```

This searches the full-lap speed-planner estimate while requiring conservative
clearance along every path link. The existing line's estimated minimum is
0.291 m. The first 0.28 m candidate reduced predicted time but was slower
in the AutoDRIVE three-lap pilot, so it was not installed. Use scored laps
to select a line; predicted time alone does not capture path-tracking error.

## Reward and episode end

Each step rewards forward centreline progress. It charges small costs for
cross-track and heading error, unsafe speed, sudden action changes, and a
safety-rejected proposal. A collision or leaving the centreline corridor ends
the episode with a larger cost. Finishing the requested lap count ends the
episode with a bonus. Missing telemetry or the step limit truncates it.

The `info` dictionary includes the reward terms, progress, path errors,
collision/lap counts, and the reason for an episode ending. Save these with
training checkpoints in Task 5.
