# Faster Track-Aware Racing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Minimize penalized lap and 10-lap times using the simulated car's achievable speed, including acceleration through gentle corners when faster, while preserving clean-run reliability.

**Architecture:** Measure the car's limits, then optimize a map-constrained line and a closed-lap speed profile with acceleration, braking, and lateral-grip constraints. Track that reference with feedforward and feedback control; keep a map-aware and LiDAR guard outside the optional residual policy. Retrain RL only after the classical controller improves scored runs.

**Tech Stack:** Python 3.10, ROS 2 Humble, NumPy, SciPy, Pillow, PyYAML, Gymnasium and SAC for offline training, AutoDRIVE compete simulator.

**Spec:** `docs/superpowers/specs/2026-09-29-faster-racing-design.md`

## Global Constraints

- No arbitrary global numeric speed cap; local speed is limited by measured car capability, curvature, braking distance, clearance, and control uncertainty.
- Gentle bends may be accelerated through only when the predicted full-lap time improves and the next braking zone remains feasible.
- Keep the old 4.4 m/s driver config, selected actor, and five scored results available for rollback.
- Keep `ros2 run team_driver driver`; keep the 40 Hz scan callback bounded and do not require training packages during a judged run.
- Do not edit `race_ws/src/roboracer_referee/`, `external/autodrive_devkit/`, `scripts/`, `docker/`, `install/`, `maps/tracks.yaml`, or `.github/workflows/`.
- Only the driver publishes car controls during a scored run; training-only resets never run with the referee.
- A candidate is promoted only after repeated complete official scored runs improve both penalized time components without a collision regression.

## Review Focus

- Missing or stale odometry must produce neutral or braking controls, never reuse a fast stale target (Task 4).
- Unknown map cells and links that cross ducts must fail clearance validation (Task 3).
- A closed-lap braking zone straddling the path seam must obey the same speed constraint as every other bend (Task 2).
- An invalid or old-format actor must select the classical fallback, never silently apply incompatible weights (Task 5).
- A gentle corner followed immediately by a sharp one must accelerate only as far as the later braking constraint permits (Task 2).

---

### Task 1: Record and estimate vehicle limits

**Files:**
- Create: `race_ws/src/team_driver/team_driver/vehicle_limits.py`
- Create: `race_ws/src/team_driver/training/vehicle_calibration.py`
- Create: `race_ws/src/team_driver/config/vehicle_limits.yaml`
- Create: `race_ws/src/team_driver/test/test_vehicle_calibration.py`
- Modify: `race_ws/src/team_driver/training/README.md`

**Interfaces:**
- Consumes ROS odometry, LiDAR, lap/collision counters, and the driver's published throttle/steering during a non-scored calibration run.
- Produces `team_driver.vehicle_limits.VehicleLimits` with `top_speed_mps`, `accel_mps2`, `brake_mps2`, `lateral_mps2`, `control_delay_s`; `estimate_limits(rows: np.ndarray) -> VehicleLimits` and `load_limits(path: Path) -> VehicleLimits` use a YAML file containing estimates, sample count, simulator build, and provenance.

- [x] **Step 1: Specify acceptance examples.** Include empty/non-finite trace rejection, speed estimates based on measured odometry rather than targets, conservative braking/grip quantiles, and valid YAML round-trip. Capture the same examples in `test/test_vehicle_calibration.py` during implementation.
- [x] **Step 2: Implement capture and estimation.** Record timestamps, pose, speed, throttle, steering, projected arc, LiDAR clearance, and lap/collision counts to ignored `training/runs/<id>/`; derive limits from clean intervals and write `config/vehicle_limits.yaml` only when sufficient clean data exists.
- [x] **Step 3: Collect non-scored calibration laps.** Run the existing 4.4 m/s champion first, then cautiously increase baseline targets on safe straights in separate runs. Record measured top speed and braking response; never infer a physical maximum merely from a configured target.
- [x] **Step 4: Check and commit.** Inspect raw traces and the derived YAML, record simulator version and sample counts, and commit the calibration code, documentation, and chosen limits. Keep raw logs ignored locally.

### Task 2: Closed-lap speed planning and longitudinal control

**Files:**
- Modify: `race_ws/src/team_driver/team_driver/speed_profile.py`
- Modify: `race_ws/src/team_driver/team_driver/speed_controller.py`
- Modify: `race_ws/src/team_driver/team_driver/driver.py`
- Modify: `race_ws/src/team_driver/training/gym_env.py`
- Create: `race_ws/src/team_driver/config/fast_baseline_params.yaml`
- Create: `race_ws/src/team_driver/test/test_speed_profile.py`
- Create: `race_ws/src/team_driver/test/test_speed_controller.py`

**Interfaces:**
- Consumes Task 1 `VehicleLimits` and a closed `PurePursuit` path.
- Produces `SpeedProfile(path, straight_speed=4.4, ..., limits: VehicleLimits | None = None)` with `target_speed(position) -> float` and `predicted_lap_time_s() -> float`; `limits=None` retains the legacy behavior, while limits mode uses `limits.top_speed_mps` instead of `straight_speed`. `SpeedController.command(target_speed, speed, dt) -> float` remains the driver/trainer throttle interface.

- [ ] **Step 1: Pin profile examples.** A zero-curvature straight reaches the measured car limit when long enough; a gentle bend may have a higher target than a sharp bend; forward acceleration and backward braking constraints hold across the cyclic seam; a gentle bend before a hairpin is limited by the hairpin's braking point.
- [ ] **Step 2: Implement the profile.** Compute local `sqrt(a_lateral/abs(curvature))` limits, then cyclic backward and forward passes until no sample changes materially. Scale available longitudinal acceleration by `sqrt(max(0, 1-(v*v*curvature/a_lateral)^2))`; include latency and uncertainty margins from the limits file. Return finite nonnegative targets.
- [ ] **Step 3: Implement control changes.** Allow full normalized positive throttle when needed (`<= 1.0`), use feedforward plus feedback with anti-windup, and apply earlier braking when speed exceeds the local target. Keep negative throttle within the simulator's allowed `[-1, 1]` range and calibrate its effect before relying on it.
- [ ] **Step 4: Share the planner with driver and training.** Add a candidate-only limits-file parameter to the driver and `fast_baseline_params.yaml`; remove duplicate speed calculations from `gym_env.py` so `action=[0,0]` matches the race driver's new baseline. Leave the default 4.4 m/s YAML unchanged.
- [ ] **Step 5: Evaluate and commit.** Run the focused checks, compare a classical candidate to the unchanged 4.4 m/s champion in complete scored runs, and record best lap, 10-lap total, collisions, per-lap spread, and speed trace. Commit the candidate only with its measured settings; do not promote yet.

### Task 3: Offline line optimization and map clearance

**Files:**
- Create: `race_ws/src/team_driver/training/track_map.py`
- Create: `race_ws/src/team_driver/training/optimize_line.py`
- Create: `race_ws/src/team_driver/test/test_line_optimizer.py`
- Create: `maps/icra26_compete_fast_line.csv` when a better valid line is found
- Create: `maps/icra26_compete_fast_line.json` for inputs, objective, margin, and generator version
- Modify: `race_ws/src/team_driver/setup.py` to package the selected line

**Interfaces:**
- Consumes `maps/icra26_compete.yaml`, the centerline CSV, Task 1 limits, and Task 2's `SpeedProfile.predicted_lap_time_s()`.
- Produces `generate_line(center: np.ndarray, grid: TrackMap, limits: VehicleLimits, margin_m: float) -> np.ndarray`; output is a closed ordered x/y CSV accepted by `PurePursuit`.

- [ ] **Step 1: Pin map and geometry examples.** Treat occupied and unknown pixels as non-drivable, flip image rows into world coordinates correctly, reject out-of-bounds points and links crossing a duct, and enforce at least the vehicle footprint plus uncertainty margin along every segment.
- [ ] **Step 2: Implement the map reader.** Read the existing PGM/YAML from an offline module under `team_driver`; do not modify or depend on the protected `scripts/` files at runtime.
- [ ] **Step 3: Optimize full-lap time.** Parameterize bounded lateral offsets from the centerline, use the current geometric racing line as the initial candidate, and minimize Task 2's estimated lap time subject to clearance and smoothness. Reject invalid candidates and keep the previous line if no valid improvement is found.
- [ ] **Step 4: Validate and commit.** Record line length, minimum map clearance, predicted lap time, and actual complete scored results against the same speed/controller settings. Package a new line only after a clean measured improvement; retain the old line for rollback.

### Task 4: Online tracking and map-aware safety

**Files:**
- Create: `race_ws/src/team_driver/team_driver/path_tracker.py`
- Create: `race_ws/src/team_driver/team_driver/map_guard.py`
- Modify: `race_ws/src/team_driver/team_driver/driver.py`
- Modify: `race_ws/src/team_driver/team_driver/safety.py`
- Modify: `race_ws/src/team_driver/config/fast_baseline_params.yaml`
- Modify: `race_ws/src/team_driver/setup.py` to package the occupancy map YAML and PGM
- Create: `race_ws/src/team_driver/test/test_path_tracker.py`
- Create: `race_ws/src/team_driver/test/test_map_guard.py`

**Interfaces:**
- Consumes Task 2 speed targets, Task 3 path and map clearance, odometry, and live LiDAR.
- Produces `PathTracker.command(position, yaw, speed) -> float` normalized steering and `MapGuard.limit(position, yaw, speed, steering, target_speed) -> float` local speed ceiling. The unchanged ROS driver entry point publishes bounded steering/throttle.

- [ ] **Step 1: Pin tracking and failure examples.** Include seam crossing, large lateral error, non-finite pose, stale odometry, invalid scan, and a map boundary inside the stopping corridor. Expected behavior is bounded steering and conservative speed or neutral/braking output.
- [ ] **Step 2: Add curvature feedforward and preview feedback.** Keep pure pursuit selectable so the two trackers can be compared on identical line and speed settings.
- [ ] **Step 3: Separate mapped boundaries from unexpected obstacles.** Use map and pose to anticipate known track geometry farther ahead than LiDAR; use LiDAR for live unexpected-obstacle and sensor-health response. Do not let the old 10 m swept-path formula impose a global 6.24 m/s request on an otherwise clear, map-validated straight.
- [ ] **Step 4: Integrate and measure.** Keep the control callback bounded for 40 Hz scans. Compare tracker error, callback duration, peak measured speed, clean laps, and penalized scored times. Leave the current default config and actor unchanged until promotion.
- [ ] **Step 5: Commit.** Include a documented fast-classical config and fallback behavior. Do not edit protected judging files.

### Task 5: Residual RL v2 on the faster baseline

**Files:**
- Modify: `race_ws/src/team_driver/team_driver/residual_policy.py`
- Modify: `race_ws/src/team_driver/training/gym_env.py`
- Modify: `race_ws/src/team_driver/training/train_sac.py`
- Modify: `race_ws/src/team_driver/team_driver/driver.py`
- Create: `race_ws/src/team_driver/test/test_residual_policy_v2.py`
- Add: candidate `.npz` and metadata under `race_ws/src/team_driver/weights/` only after scored selection

**Interfaces:**
- Consumes Task 4's exact baseline command and safety output.
- Produces observation/model format v2 with version checks; preserve `pack_observation` v1 for the old actor, add `pack_observation_v2(...) -> np.ndarray`, and dispatch by `ResidualPolicy.load(path).observation_version`. `predict(observation) -> (steering_delta, speed_multiplier)` remains the runtime boundary.

- [ ] **Step 1: Define the v2 observation.** Use 18 LiDAR sectors plus 10 state values: measured speed, heading error, cross-track error, curvature now/1 m/3 m, target speed now/1 m/3 m, and baseline steering. The v2 dimension is 28. Scale speed targets by Task 1's `top_speed_mps` rather than 4 m/s. Dispatch old v1 weights to the old v1 observation and reject shape/version mismatches; keep the old champion's config working.
- [ ] **Step 2: Match training and runtime.** Use one observation/action decoder, identical new baseline and safety constraints, and deterministic exported NumPy inference. Check the zero action reproduces the classical command.
- [ ] **Step 3: Train seeded candidates.** Use the connected compete simulator only outside referee runs; record training seed, build, limits, line, reward, steps, collisions, and actor export. Rank by deterministic scored runs rather than exploratory episode reward.
- [ ] **Step 4: Compare and select.** Evaluate the best RL candidate against the fast classical baseline and the retained 4.4 m/s champion with repeated complete official runs. Promote only on improved penalized best lap and 10-lap total without collision regression; otherwise keep the faster classical candidate.
- [ ] **Step 5: Commit the selected behavior and evidence.** Preserve unedited result JSONs in `results/submitted/`, the selected actor/metadata if RL wins, the prior champion and fallback configs, and a concise comparison report. Push to the `team` remote only after the selected behavior is reproducible.

## Execution order and stop conditions

Tasks 1-4 produce an independently raceable classical driver. Task 5 starts only
after that driver improves the current champion in scored runs. At any stage,
a collision regression, missed 40 Hz runtime budget, stale-sensor behavior, or
unexplained map disagreement blocks promotion; the previous champion remains
the default while the candidate is revised. No fixed speed number is a success
criterion: the simulator's measured capability and clean scored lap time decide.
