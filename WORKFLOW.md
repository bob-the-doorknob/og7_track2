# Track 2 Windows workflow

This project uses **WSL 2 terminals**, a **Docker ROS 2 Humble container**, and
the **native Windows AutoDRIVE Simulator**. The simulator connects to the ROS
bridge on `127.0.0.1:4567`. The source code and result files live in this Git
repository, not inside the container image.

The current driver uses the packaged v2 residual actor. Current corner-exit
settings and simulator evidence are in
[the controller investigation](docs/corner-exit-2026-10-16.md).

## One-time setup on a new Windows checkout

1. Start Docker Desktop and enable **Settings > Resources > WSL integration**
   for your Ubuntu distribution. Install WSL 2 and Ubuntu first if needed.
2. Stop the container started from your older NTU checkout before switching
   repositories. In an **Ubuntu/WSL terminal**:

   ```bash
   cd '/mnt/c/Users/Andrew/OneDrive/Documents/NTU_code/DeepSpeed/~/Recruitment_Hackathon_2627'
   ./install/windows/stop.sh
   ```

   Then clone the team's repository into your WSL home directory and enter it:

   ```bash
   git clone https://github.com/bob-the-doorknob/DeepSpeedRobot.git ~/DeepSpeedRobot
   cd ~/DeepSpeedRobot
   git switch main
   ```

3. Build the competition container once:

   ```bash
   ./install/windows/setup.sh
   ```

4. Keep the Windows `AutoDRIVE Simulator.exe` you have already installed. The
   simulator binary is intentionally excluded from Git. If it is missing on a
   new machine, follow [the official simulator setup](docs/01-setup.md#13-fetch-the-simulator).

All commands below assume the WSL checkout is `~/DeepSpeedRobot`. Run each host
script from that repository root so Docker mounts it at `/hackathon`. Do not
mix terminals launched from different checkouts during one session.

## Every session: drive manually

### Terminal 1: start the ROS bridge

Open WSL, start Docker Desktop if it is not running, and run:

```bash
cd ~/DeepSpeedRobot
git pull --ff-only                  # skip while you have local uncommitted work
./install/windows/run.sh
```

`run.sh` starts the container and opens a shell inside it. In that shell:

```bash
source /opt/ros/humble/setup.bash
cd /hackathon/race_ws
colcon build --symlink-install --packages-select team_driver
source install/local_setup.bash
ros2 launch roboracer_referee simulator.launch.py simulator:=false rviz:=false
```

Leave Terminal 1 running. `simulator:=false` starts the bridge for the Windows
app; it does not launch a second simulator inside Docker. Rebuild after adding
Python files or changing `setup.py`. Rebuilding after ordinary Python edits is
also safe.

### Windows: connect AutoDRIVE

1. Open **AutoDRIVE Simulator.exe**.
2. In its Menu Panel, set **IP `127.0.0.1`** and **port `4567`**.
3. Click **Connect**, then choose **Autonomous**.
4. If the app reconnects or you restart the bridge, repeat Connect and
   Autonomous. The bridge must be running before you connect.

### Terminal 2: run the driver

Open a second WSL terminal from the **same checkout**:

```bash
cd ~/DeepSpeedRobot
./install/windows/shell.sh
```

Inside the container shell:

```bash
source /opt/ros/humble/setup.bash
source /hackathon/race_ws/install/local_setup.bash
ros2 run team_driver driver --ros-args --params-file /hackathon/race_ws/src/team_driver/config/driver_params.yaml
```

The car should follow the centreline. Press **Ctrl+C** in Terminal 2 to stop
the driver before starting an official evaluation, so two drivers do not send
commands at the same time.

If the car does not move, check that AutoDRIVE says **Connected** and
**Autonomous**, then in another container shell run:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /autodrive/roboracer_1/odom --once
```

An odometry message confirms simulator telemetry is reaching ROS. If it hangs,
reconnect the Windows simulator to the bridge. Check Terminal 1 for errors.

## Optional: run a scored evaluation

Keep Terminal 1's bridge and the Windows simulator running. Stop the manual
driver in Terminal 2 with **Ctrl+C**. From Terminal 2, run:

```bash
source /opt/ros/humble/setup.bash
source /hackathon/race_ws/install/local_setup.bash
ros2 launch roboracer_referee evaluate.launch.py \
  team:=deepspeedrobot \
  track:=icra26_compete \
  simulator:=false \
  driver_pkg:=team_driver \
  driver_exec:=driver \
  driver_params:=/hackathon/race_ws/src/team_driver/config/driver_params.yaml \
  output_dir:=/hackathon/results
```

The referee starts its **own** driver and writes JSON in `results/`. Its
default full run includes a warm-up lap and ten timed laps. For a quick wiring
check, add `warmup_laps:=0 timed_laps:=2` to the command; do not treat that as
a full competition result. The referee owns reset during scored runs. If
telemetry stops after a run, reconnect AutoDRIVE and select Autonomous again.

## Edit, save, and push

Edit files on the host, especially `race_ws/src/team_driver/team_driver/`,
`race_ws/src/team_driver/config/driver_params.yaml`, and later `training/`.
The `/hackathon` directory in Docker is the same checkout, so saving a host
file immediately updates the mounted source. Re-run the driver after edits;
rebuild when package files or installed resources change.

From a WSL host terminal in `~/DeepSpeedRobot`:

```bash
git status
git add race_ws/src/team_driver WORKFLOW.md
git commit -m "Describe your change"
git push origin main
```

The team checkout cloned above has the team repository as `origin`. The older
NTU checkout has the official hackathon repository as `origin`; do not push
your work there.

`results/` practice files, `race_ws/build/`, `race_ws/install/`, and the
simulator binary are ignored by Git. When you choose a final scored result,
copy its unedited JSON into `results/submitted/` and commit that file. Exported
policy weights should be placed in `race_ws/src/team_driver/weights/` and
committed only after validation.

## Finish the session

Stop the driver and bridge with **Ctrl+C**. Close AutoDRIVE. From a WSL host
terminal in the same checkout:

```bash
./install/windows/stop.sh
```

Your code and results remain on the host checkout after the container stops.
