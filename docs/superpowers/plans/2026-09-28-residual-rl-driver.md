# Residual RL Driver Implementation Plan

> **For agentic workers:** Use `superpowers:executing-plans` to implement this plan task by task after the user approves it. Keep the checkboxes updated.

**Goal:** Build a reliable Track 2 driver around the provided map, then train and evaluate a small residual reinforcement learning policy without losing the classical driver fallback.

**Architecture:** A pure pursuit controller follows the provided centreline and a PI speed controller converts target speed into throttle. A separate training-only Gymnasium environment trains a bounded SAC policy to adjust steering and target speed. At race time, a small NumPy inference module applies the saved policy; a LiDAR safety check can limit speed or reject unsafe corrections.

**Tech Stack:** Python 3.10, ROS 2 Humble (`rclpy`), NumPy for the judging-time policy, Gymnasium and Stable-Baselines3/SAC with PyTorch for offline training.

**Spec:** The approved residual RL direction from the 2026-09-28 conversation, captured in this plan. Track 2 rules and documentation are the source of truth.

## Global Constraints

- Keep the judging entry point `ros2 run team_driver driver` working.
- The simulator publishes LiDAR at 40 Hz; bound callback work so control does not miss scans.
- Use the official `2026-icra` compete build for scored runs; train on practice only as a training environment and validate on compete.
- `/autodrive/reset_command` must never be published by the judging-time driver. Any reset logic belongs only in an isolated training tool and must not run during a scored run.
- Do not modify the referee, AutoDRIVE Devkit, Docker configuration, `scripts/`, `install/`, or `maps/tracks.yaml`.
- Keep all runtime dependencies declared in `race_ws/src/team_driver/requirements.txt` or `apt-packages.txt`. Do not replace the pinned NumPy or websocket dependencies.
- Store model weights and the code that produced them in the repository. No network access is available during judging.
- The camera is empty in headless runs; the policy uses LiDAR and permitted `/odom` state instead.
- A collision adds 10 seconds; more than 10 collisions disqualifies a run. Completion and clean driving take priority over a small lap-time gain.

## Review Focus

- Missing, NaN, or infinite LiDAR ranges must not make the policy output invalid commands; sanitize and clamp observations.
- Missing or stale odometry must fall back to safe bounded commands rather than use uninitialized state.
- A missing or invalid model file must leave the classical baseline usable and clearly report the fallback.
- Training resets must remain isolated from the scored driver; verify the production entry point never publishes the reset topic.
- Policy output must remain bounded and finish inference within the control-loop budget; include a deterministic baseline-only mode for comparison.

---

## Code Locations

Work from the Track 2 repository root. The local checkout is currently under the workspace's `~\Recruitment_Hackathon_2627` directory; inside the provided container it is mounted at `/hackathon`.

| Path | Responsibility |
|---|---|
| `race_ws/src/team_driver/team_driver/driver.py` | Existing ROS 2 entry point, subscriptions, and integration of baseline, safety check, policy, and throttle controller. |
| `race_ws/src/team_driver/team_driver/pure_pursuit.py` | Load the committed centreline, project pose onto it, select a lookahead point, and calculate baseline steering/target speed. |
| `race_ws/src/team_driver/team_driver/speed_controller.py` | Feed-forward plus clamped PI controller from target speed to normalized throttle. |
| `race_ws/src/team_driver/team_driver/residual_policy.py` | Runtime observation packing, weight loading, NumPy MLP inference, and bounded residual outputs. |
| `race_ws/src/team_driver/team_driver/safety.py` | LiDAR-based speed limit and unsafe-action fallback. |
| `race_ws/src/team_driver/training/gym_env.py` | Training-only ROS/Gymnasium adapter, reward calculation, episode lifecycle, and practice-only resets. Never imported by `driver.py`. |
| `race_ws/src/team_driver/training/train_sac.py` | SAC training entry point, checkpointing, and export of a compact runtime model. |
| `race_ws/src/team_driver/training/requirements.txt` | Training-only Python dependencies and pinned versions; do not add training-only packages to runtime requirements unless the driver needs them. |
| `race_ws/src/team_driver/weights/` | Exported, versioned policy weights required at judging time. |
| `race_ws/src/team_driver/requirements.txt` | Runtime packages only. NumPy is already supplied by the judging image. |
| `SUBMISSION.md` | Team approach, dependencies, results, training method, validation, and known failure modes. |

All policy, training, and weight files belong under `race_ws/src/team_driver/`. Do not put new training tools under the repository's protected top-level `scripts/` directory.

## Language and Day-to-Day Workflow

The driver and training code are **Python**. ROS 2 runs the driver inside the repository's Ubuntu 22.04/ROS 2 Humble container, using Python 3.10. Edit the bind-mounted files with the normal Windows editor or from WSL; the container sees those same files under `/hackathon`.

The recurring loop is:

1. Edit Python files under `race_ws/src/team_driver/`.
2. In the container, build the package after adding files or changing package metadata, then source the workspace:

   ```sh
   cd /hackathon/race_ws
   colcon build --symlink-install --packages-select team_driver
   source install/local_setup.bash
   ```

   With `--symlink-install`, edits to existing Python files take effect without a rebuild.

3. Start the simulator/bridge and run the driver with the documented ROS command:

   ```sh
   ros2 run team_driver driver
   ```

4. For scored-format comparisons, run from the host repository root:

   ```sh
   ./scripts/evaluate.sh --team <team_or_variant> --runs 3
   ```

5. Keep a known-good baseline commit and compare complete runs, collisions, best penalized lap, and penalized 10-lap total before promoting a policy change.
6. Save only intended source, configuration, weights, docs, and final result JSONs to Git. Do not commit the downloaded simulator or build/install/log artifacts.

The current local Git branch is `track2`, and its `origin` remote points to the official `NTUDeepSpeed/Recruitment_Hackathon_2627` repository. Do not push to `origin` unless the team has write access and intends to use that repository. Normally, fork the repository or use the team's assigned repository, add that as a remote (for example `team`), and push the `track2` branch there. The organizers require the submitted repository URL and branch name.

## Implementation Tasks

### Task 1: Establish a measurable classical fallback

**Files:**
- Modify: `race_ws/src/team_driver/team_driver/driver.py`
- Create: `race_ws/src/team_driver/team_driver/pure_pursuit.py`
- Create: `race_ws/src/team_driver/team_driver/speed_controller.py`
- Use: `maps/icra26_compete_centerline.csv`

- [x] Implement centerline loading and pose projection using the existing `/odom` pose.
- [x] Implement pure pursuit with a speed-scaled lookahead and a configurable conservative target speed.
- [x] Implement a feed-forward plus PI speed controller with a clamped integral; avoid sustained hard reverse throttle.
- [x] Add a baseline-only parameter/mode that remains available after RL integration.
- [x] Keep the LiDAR preprocessing and `devkit_qos()` behavior from the starter driver.
- [x] Confirm the baseline completes a 10-lap evaluation and retain those result files as the comparison point.

Evaluation attempt on 2026-09-29: the package built, but the referee aborted before scoring because the AutoDRIVE bridge published no telemetry for 120 seconds. The unscored diagnostic is in `results/task1_baseline__task1_baseline_20260929.json`; it is not a baseline comparison run.

Completed baseline on 2026-09-29: `results/task1_baseline__task1_baseline_connected_20260929.json` is `COMPLETE`, `scored: true`, with 10/10 timed laps, zero collisions, a 29.3355 s best lap, and a 295.6529 s penalized 10-lap total (real-time factor 0.811). This run used the compete build in the Windows simulator connected to the container bridge.

### Task 2: Add a bounded safety layer

**Files:**
- Create: `race_ws/src/team_driver/team_driver/safety.py`
- Modify: `race_ws/src/team_driver/team_driver/driver.py`

- [x] Implement front-clearance and steering-aware speed limits from LiDAR.
- [x] Sanitize discontinuities and empty/invalid ranges before calculating clearance.
- [x] Allow the safety layer to reduce target speed or reject a residual action; never let it increase the baseline command beyond the configured limit.
- [x] Preserve a baseline-only mode so every later training or runtime failure has a usable fallback.

Completed safety comparison on 2026-09-29: `results/task2_safety__task2_safety_20260929.json` is `COMPLETE`, `scored: true`, with 10/10 timed laps, zero collisions, a 29.4345 s best lap, and a 294.7702 s penalized 10-lap total (real-time factor 0.754, callback errors 0). This single run checks completion and safety behavior; the 0.8827 s total-time difference from Task 1 is too small to treat as an established speed gain.

### Task 3: Define the residual policy interface

**Files:**
- Create: `race_ws/src/team_driver/team_driver/residual_policy.py`
- Modify: `race_ws/src/team_driver/team_driver/driver.py`

- [x] Define one fixed observation vector containing compact LiDAR sectors, current speed, path heading error, cross-track error, and upcoming path curvature.
- [x] Define two bounded policy outputs: steering adjustment and a target-speed multiplier.
- [x] Load exported weights once during node startup; perform only small NumPy matrix operations in the scan callback.
- [x] If the policy is disabled, missing, or produces a non-finite value, use the classical baseline and emit a clear diagnostic.
- [x] Ensure the same observation normalization and action limits are used by the trainer and runtime driver.

Implemented in `team_driver/residual_policy.py` as observation/model format v1.
The trainer in Task 4/5 should import `pack_observation` and `decode_action`
directly. Keep `baseline_only: true` until a learned model beats the classical
controller. A zero-weight policy completed a two-lap compete-build integration
run on 2026-09-29: `results/task3_zero__task3_zero_20260929.json` reports
`COMPLETE`, `scored: true`, 2/2 laps, zero collisions, zero callback errors,
and a 58.625 s total. This validates the policy path, not learned performance.

### Task 4: Build the training-only simulator environment

**Files:**
- Create: `race_ws/src/team_driver/training/gym_env.py`
- Create: `race_ws/src/team_driver/training/requirements.txt`
- Do not modify: `race_ws/src/team_driver/team_driver/driver.py` to add reset publishing.

- [x] Wrap the simulator sensors, odometry, and collision telemetry as a Gymnasium observation/reward interface.
- [x] Keep all reset-topic publishing inside this training-only module and use it only for practice/training episodes, never a scored run.
- [x] Reward forward progress; penalize cross-track and heading error, unsafe speed, abrupt action changes, contact, and leaving the course. End unsafe episodes promptly.
- [x] Start the policy at zero residual so the initial behavior equals the measured baseline.
- [x] Pin compatible training dependencies separately. Do not override the image's NumPy or websocket package pins.

Implemented in `race_ws/src/team_driver/training/gym_env.py`, with dependency
pins and setup details in `training/requirements.txt` and `training/README.md`.
Gymnasium 0.29.1 imports alongside ROS Humble and the image's NumPy 1.21.5
from the isolated training environment. Live `reset()` and rollout remain to
be checked. Training can use the compete build outside scored runs with its
supplied centreline, or a practice build with a matching centreline. No
training reset has been sent yet.

### Task 5: Train, checkpoint, and export the policy

**Files:**
- Create: `race_ws/src/team_driver/training/train_sac.py`
- Create: `race_ws/src/team_driver/weights/`

- [x] Train continuous residual actions with SAC outside a scored run, saving checkpoints and episode metrics. The first 1,000-step run used the compete build; it is a pipeline run, not a performance claim.
- [x] Record random seed, user-declared simulator build, reward coefficients, training duration, and training step count with each checkpoint.
- [x] Export the actor to the simple weight format consumed by `residual_policy.py`; the race driver does not require PyTorch inference.
- [x] Keep a zero-residual baseline checkpoint in `weights/baseline_residual.npz`. Candidate selection and full-run comparison remain Task 6.

Before Task 6, the baseline gained a 3.0 m/s straight-line ceiling and a
closed-track curvature/braking speed profile shared by `driver.py` and
`gym_env.py`. A fresh 1,500-step SAC run completed five single-lap training
episodes without reported collisions. Its exported candidate is in
`weights/speed_profile_1500_candidate.npz`; deterministic full-run comparison
is still required before enabling it by default.

The next pre-Task-6 iteration generated a map-constrained racing-line CSV,
raised the straight-line target to 3.6 m/s, and extended the LiDAR safety
horizon to 4.5 m in both driver and trainer. The 2,000-step SAC training run
completed six single-lap exploratory episodes and had one collision episode.
Its candidate is `weights/racing_line_2000_candidate.npz`. This line is a
geometric candidate, not a proven minimum-time route; Task 6 must compare
deterministic compete-build runs before promoting it.

### Task 6: Compare on the official compete build

**Files:**
- Modify: `race_ws/src/team_driver/team_driver/driver.py`
- Record: `results/` during evaluation; promote required final runs to `results/submitted/` only at submission time.

- [ ] Compare baseline-only and baseline-plus-policy using the same compete build and evaluation settings.
- [ ] Use multiple complete runs; compare collision count, best lap after penalties, and 10-lap total after penalties.
- [ ] Keep RL only when it improves the measured result without making completion or collision reliability materially worse.
- [ ] Check policy inference is bounded and the 40 Hz loop keeps up.
- [ ] Leave the classical fallback as the selectable default if RL has not earned promotion.

### Task 7: Prepare and submit the entry

**Files:**
- Create/update: `SUBMISSION.md`
- Commit: selected source, training code/config, exported weights, and unedited result files in `results/submitted/`.

- [ ] Run `./scripts/verify_judging_env.sh` and resolve protected-environment changes before submission.
- [ ] Test the submitted branch from a clean clone with declared dependencies and the official evaluation flow.
- [ ] Include at least one unedited result with `status: COMPLETE` and `scored: true`; describe the machine and typical real-time factor.
- [ ] Document the RL inputs, outputs, reward, environment/build, training amount, validation method, failure modes, dependencies, third-party references, and AI assistance.
- [ ] Push to the fork/team remote, not the official `origin` by accident, and send the repository URL and branch to the organizers before the team's internal 17 October cutoff.

## Suggested Schedule

- **29–30 September:** build pure pursuit and a throttle controller; establish the fallback driver and first complete evaluation.
- **1–4 October:** tune safe baseline and collision protection; record repeatable baseline runs.
- **5–10 October:** implement the training wrapper and collect/train residual policy checkpoints.
- **11–13 October:** validate on compete and compare against the baseline; drop changes that do not improve full-run results.
- **14–17 October:** freeze the selected version, clean-clone and submission checks, finalize documentation and push.

The official site lists 18 October 2026, 23:59 SGT; this plan uses 17 October as the safer internal deadline.

## Completion Criteria

- The baseline and learned variant both run through the unchanged `ros2 run team_driver driver` entry point.
- Policy-off mode remains usable, and runtime inference uses only declared packages.
- The selected result completes all ten timed laps and has a better or acceptably comparable penalized result to baseline across repeated evaluations.
- The repository contains the selected weights and the code/configuration needed to explain and reproduce their training process.
- The protected judging environment and AutoDRIVE Devkit remain unchanged; submission artifacts and `SUBMISSION.md` meet chapter 7 requirements.
