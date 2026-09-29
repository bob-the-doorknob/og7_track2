# AutoDRIVE training environment

`gym_env.py` is the Gymnasium environment. It subscribes to
AutoDRIVE LiDAR, odometry, lap count, and collision count, and publishes the
same steering/throttle topics as the race driver. It reuses the race driver's
centreline controller, PI speed controller, LiDAR safety, observation packing,
and residual action limits. `action=[0, 0]` produces the baseline command.

Reset publishing is confined to this file. The wrapper can train against the
**compete** build outside a scored run, using the supplied compete centreline.
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
a centreline traced for that build. This repository contains only the compete
centreline. The simulator does not report its build identity over ROS, so check
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
  --seed 42
```

For the practice build, change `--simulator-variant practice` and add
`--centerline /path/to/practice_centerline.csv`. The script cannot verify which
executable is open, so enter its actual version/name in `--simulator-build`.
Training explicitly enables resets and must never run during a scored race.

`training/runs/<run-id>/` contains `baseline.zip/.npz/.json`, periodic
`step_*.zip/.npz/.json`, `final.zip/.npz/.json`, and `episodes.jsonl`.
The JSON stores the seed, declared simulator build, reward weights, elapsed
time, and step count. The `.zip` retains the full SAC trainer; `.npz` is the
small NumPy actor used by the race driver. Run outputs are gitignored. The
checked-in `weights/baseline_residual.npz` is the zero-residual fallback.
Task 6 must compare a candidate against the baseline before selecting it for
`weights/residual_policy.npz` and changing `baseline_only` in the driver config.

## Reward and episode end

Each step rewards forward centreline progress. It charges small costs for
cross-track and heading error, unsafe speed, sudden action changes, and a
safety-rejected proposal. A collision or leaving the centreline corridor ends
the episode with a larger cost. Finishing the requested lap count ends the
episode with a bonus. Missing telemetry or the step limit truncates it.

The `info` dictionary includes the reward terms, progress, path errors,
collision/lap counts, and the reason for an episode ending. Save these with
training checkpoints in Task 5.
