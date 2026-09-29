# Practice simulator training environment

`gym_env.py` is the Gymnasium environment for Task 4. It subscribes to
AutoDRIVE LiDAR, odometry, lap count, and collision count, and publishes the
same steering/throttle topics as the race driver. It reuses the race driver's
centreline controller, PI speed controller, LiDAR safety, observation packing,
and residual action limits. `action=[0, 0]` produces the baseline command.

Reset publishing is confined to this file. Instantiate the environment only
against the **practice** simulator, without the referee or another driver:

```python
from training.gym_env import AutoDriveResidualEnv

env = AutoDriveResidualEnv(simulator_variant="practice")
try:
    observation, info = env.reset()
    observation, reward, terminated, truncated, info = env.step([0.0, 0.0])
finally:
    env.close()
```

The simulator does not report its build identity over ROS. The
`simulator_variant="practice"` argument is an explicit declaration by the
caller; check the actual AutoDRIVE executable before using it. The environment
also refuses to reset when a referee or another command publisher is visible.
Never use it for a scored run.

## Install training dependencies

The judging image includes Python 3.10, ROS Humble, and NumPy 1.21.5. Keep
their system packages intact. In a shell inside the running container, after
sourcing the **worktree** ROS overlay:

```bash
source /opt/ros/humble/setup.bash
source /hackathon/.worktrees/task1-residual-rl-baseline/race_ws/install/local_setup.bash
cd /hackathon/.worktrees/task1-residual-rl-baseline/race_ws/src/team_driver
python3 -m venv --without-pip --system-site-packages training/.venv
python3 -m pip install --no-deps \
  --target training/.venv/lib/python3.10/site-packages \
  -r training/requirements.txt
```

The image does not include `ensurepip`, so this virtual environment is made
without its own `pip`; the system `pip` installs into the virtual environment's
site-packages directory. `--no-deps` places only the four pinned pure Python
packages there. It leaves system NumPy and the AutoDRIVE websocket stack
untouched. Run training programs with `training/.venv/bin/python`. When working
from a fresh team checkout mounted at `/hackathon`, replace the long worktree
path above with `/hackathon`.

Task 5 will provide the training loop and an executable smoke run. Until then,
this module defines the environment contract and can be imported after the
installation above. `reset()` requires live telemetry from a connected
practice simulator. `step()` returns Gymnasium's
`(observation, reward, terminated, truncated, info)` tuple.

## Reward and episode end

Each step rewards forward centreline progress. It charges small costs for
cross-track and heading error, unsafe speed, sudden action changes, and a
safety-rejected proposal. A collision or leaving the centreline corridor ends
the episode with a larger cost. Finishing the requested lap count ends the
episode with a bonus. Missing telemetry or the step limit truncates it.

The `info` dictionary includes the reward terms, progress, path errors,
collision/lap counts, and the reason for an episode ending. Save these with
training checkpoints in Task 5.
