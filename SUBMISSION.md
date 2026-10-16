# Team OG7

**Registered track:** Track 2 (AutoDRIVE Simulator)

**Team leader contact:** Anderson Andrew Chandra — Telegram `@Andsonc`,
Achandra001@e.ntu.edu.sg

## Members

| Name | Matric number | Email |
| --- | --- | --- |
| Anderson Andrew Chandra (Team Leader) | U2620664A | Achandra001@e.ntu.edu.sg |
| Nicholas Wayne Wiryawan | U2620365A | NICH0107@e.ntu.edu.sg |
| Anthony Halim | U2620294G | anth0031@e.ntu.edu.sg |
| Faiz Haydar Akmal | U2620860J | FAIZ0004@e.ntu.edu.sg |
| Kevin Mangara | U2620966G | kevi0030@e.ntu.edu.sg |

## Approach

The selected driver combines a map-guided racing-line planner with a learned
residual policy. Pure pursuit follows the packaged `icra26_compete` racing
line and the speed profile anticipates upcoming corners. The v2 residual actor
makes bounded steering and speed adjustments. A LiDAR safety layer filters the
proposed action, and the speed controller tracks the final target.
At aligned, gently straightening exits, a guarded follower preview can replace
the conservative constant-steering safety arc. It requires map and LiDAR
clearance and caps the lift over the original speed limit at 1.0 m/s. Current
route and commanded steering curvature must both be at most 0.30 per m.
The selected planning lateral allowance is 18 m/s^2, reduced from 24 to plan
earlier braking at the tight hairpins. It was tuned after training the actor;
the learned weights were not retrained or changed for this controller update.

The selected actor is
`race_ws/src/team_driver/weights/residual_policy_v2_sub12_candidate.npz`.
Its SHA-256 is
`e76d8b8baf50bf452c59fc00002fd6efda7aad0c21206302630a77654e9bbe22`.
The selected controller settings are the defaults in both `driver.py` and
`config/driver_params.yaml`; the explicit equivalent is
`config/fast_corner_18_residual_v2_exit_preview_params.yaml`.
It was trained with residual SAC on the clean map72 classical driver. The
recorded run resumed a v2 checkpoint and collected 4,000 further training
steps with seed 50; the first 1,000 steps used the baseline action. The actor
and metadata are included with the driver. The packaged policy runs locally
without network access.

## Running the selected driver

Inside the judging container, from the submitted repository's `race_ws/`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 run team_driver driver
```

Only one driver should publish commands during a run. The simulator bridge
must already be connected. Judging launches the driver itself; stop a manual
driver before starting evaluation. No external parameter path or model download
is needed for the selected defaults.

## How it uses the inputs

- **LiDAR:** used for path clearance and the safety speed limit; the residual
  policy also receives sector summaries.
- **Pose and speed:** uses simulator odometry (`/odom`) for vehicle position,
  heading, and speed. The `/ips` position feed can also update position; its
  disagreement with odometry is logged for diagnostics.
- **Camera, IMU, and encoders:** not used.

The planner follows the packaged racing line and map guard for the
`icra26_compete` circuit.

## Speed control

The planner sets a target speed from the racing line and vehicle limits. The
speed controller converts target speed into throttle, while the LiDAR safety
layer can cap the target when clearance is limited. The residual actor may
adjust the target within its bounded action range. The selected configuration
uses a 7.2 m/s requested top speed and a 2.8 m/s^2 LiDAR braking limit.
The existing forward/backward profile uses measured longitudinal acceleration
and braking limits with a friction ellipse. The 18 m/s^2 lateral allowance is
an empirically tested planning setting, not a measured physical grip limit.

## Results on our own machine

The 24 m/s^2 preview variants were rejected after a repeat reached a larger
tracking excursion. The selected 18 m/s^2 profile with guarded preview completed
two full runs with zero collisions across all 20 timed laps. Their mean ten-lap
total is 111.1333 s, versus 112.3732 s across two reference runs (1.10% faster).
The fastest single lap did not improve; we selected for race pace and tracking.

The current worktree was rebuilt and evaluated through the judging launch on
16 October 2026. The selected guarded controls completed all ten timed laps
with zero collisions and no penalties. Runs use two unscored laps: the out lap
and the single warm-up lap. Original referee JSONs are in `results/submitted/`.

| Run | Best lap | 10-lap total | Collisions | Status |
| --- | ---: | ---: | ---: | --- |
| `cornerexit_preview18_20261016_a` (selected controls, explicit parameters) | **11.0708 s** | **111.0617 s** | 0 | COMPLETE, scored |
| `cornerexit_default_20261016_b` (selected bare defaults; trace-only override) | **11.0588 s** | **111.2049 s** | 0 | COMPLETE, scored |
| `cornerexit_baseline_20261016_a` (preview disabled) | 11.0879 s | 112.8843 s | 0 | COMPLETE, scored |
| `cornerexit_baseline_20261016_b` (preview disabled, repeat) | 11.0208 s | 111.8620 s | 0 | COMPLETE, scored |
| `og7_final_20261016_v1` (before exit changes) | 11.0618 s | 114.6310 s | 0 | COMPLETE, scored |

Other October experiments are retained in `results/submitted/`, including an
unguarded-preview DNF_STUCK and rejected taper/gradual-lift trials. Their faster
individual laps are not used to select the final driver. Full parameters,
selection rationale, and telemetry metrics are described in
`docs/corner-exit-2026-10-16.md`. The final repeat verified the actual default
entry point, and a ROS parameter readback confirmed the selected settings.
Three-lap diagnostic pilots from this session were not added to that folder.

Two earlier packaged-actor runs, from 30 September 2026, are retained as
historical performance evidence. They predate the current driver changes:

| Run | Best lap | 10-lap total | Collisions | Status |
| --- | ---: | ---: | ---: | --- |
| `sub12rl_packaged_full` | 10.9598 s | 111.8410 s | 0 | COMPLETE, scored |
| `sub12rl_packaged_repeat` | 11.0999 s | 112.6974 s | 0 | COMPLETE, scored |

### The machine that produced the current run

The selected runs recorded real-time factors of 0.666 and 0.670.
The October comparison runs ranged from 0.458 to 0.698; the earlier pre-change
run recorded 0.952. Simulator pacing varied during testing.

| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core Ultra 7 255HX | NVIDIA GeForce RTX 5060 Laptop GPU (8 GB) | 16 GB (15.4 GiB visible) | Lenovo 83F3; Windows 25H2, build 26200, 64-bit (product label: Windows 10 Home Single Language) | 0.666-0.670 (selected runs); 0.458-0.698 in comparisons |

## Anything precomputed

The driver uses the packaged `icra26_compete` racing line and map guard. The
learned residual policy weights and their metadata are in
`race_ws/src/team_driver/weights/`. `setup.py` installs the policy weights,
vehicle limits, track centerline and racing line, and map YAML/image into the
ROS package. The current scored run confirmed the installed package loaded
both the racing line and `residual_policy_v2_sub12_candidate.npz` from its
package share directory. No additional runtime files need to be fetched.
The submission must include the complete `race_ws/src/team_driver/` package,
including its safety, map guard, pure-pursuit, exit-preview, and diagnostic
modules, plus the packaged map/racing-line assets. `driver.py` alone is not
sufficient. Installed configuration and actor bytes were checked against the
source files after the final build.

The training code and run documentation are in
`race_ws/src/team_driver/training/` and `docs/residual-rl-sub12-2026-09-30.md`.
The actor was trained with residual SAC from a v2 checkpoint, using a fresh
replay buffer and 4,000 additional steps (seed 50). The packaged `.npz` file is
the inference artifact; the training checkpoint and replay buffer are not
needed at runtime.

## Dependencies we added

None. The driver uses the ROS 2 packages and Python dependencies in the
provided judging image. `requirements.txt` and `apt-packages.txt` contain no
additional packages.

## Third-party code and references

The offline training code uses Stable-Baselines3 2.3.2 for SAC. Its code is not
needed by the runtime driver; the exported NumPy actor is included in this
repository.

## AI assistance

Codex was used to assist with implementation, debugging, analysis, and
submission preparation. Team members should review the submitted code and be
prepared to explain its behavior and the model-training process.

## Known issues

- The selected controls have two complete ten-lap runs on this machine.
  Different simulator pacing or judging hardware may change the results.
- Throttle switching is reduced in frequency at the exit, but individual
  command jumps remain. Physical jerk has not been measured.
- The geometric preview is deliberately excluded from tight turns; its model
  does not guarantee grip or yaw stability.
- A clean-clone build and evaluation are still pending.

## Local preparation status

This is a local draft. Nothing has been submitted or published from this
preparation step. Publication requires the team leader's final approval.

Before requesting that approval:

- Destination: `https://github.com/bob-the-doorknob/DeepSpeedRobot` (the
  configured `team` remote). Prepared submission branch: `submission/og7-track2`.
  This branch remains local until the team leader approves publication.
- Freeze the selected driver and its runtime assets in a local commit,
  excluding simulator binaries, build/install/log directories, and unrelated
  ZIP archives.
- Verify the judging environment and build/evaluate a clean copy of that
  exact commit. The current worktree results do not replace this check.
- Review the final commit, result files, and submission record with the team
  leader.

After final approval, push the approved branch, check the Judge workflow, and
provide the organisers with its repository URL and branch before
18 October 2026, 23:59 SGT, as specified in `docs/07-submission.md`.
