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

The controller keeps a coherent odometry pose and compensates for feedback
delay using two measured motion intervals. Steering and speed-profile queries
use that bounded predicted pose. The maximum lookahead grows by at most 0.3 m
only at slower feedback rates. This correction was tested on Linux headless
and Windows; see `docs/headless-pace-2026-10-16.md`.

The selected actor is
`race_ws/src/team_driver/weights/residual_policy_v2_sub12_candidate.npz`.
Its SHA-256 is
`e76d8b8baf50bf452c59fc00002fd6efda7aad0c21206302630a77654e9bbe22`.
The selected controller settings are the defaults in both `driver.py` and
`config/driver_params.yaml`; the explicit equivalent is
`config/fast_corner_18_residual_v2_latency_params.yaml`.
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
  heading, and speed as one control state. The `/ips` position feed is used
  only for disagreement diagnostics and never overwrites the control pose.
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

### Latest cross-platform correction

The actor is unchanged. The corrected controls substantially reduce the
headless gap, with two final Linux ten-lap totals of 117.3531 and 116.9162 s
(mean 117.1347 s), versus 129.0319 and 133.3047 s with the original submission
(mean 131.1683 s): **10.70% faster**. All forty timed laps had zero collisions.
The final Windows cross-check preserved local pace at 110.9386 s with zero
collisions. These are local results, not the organisers' score.

| Run | Platform | Best lap | 10-lap total | Collisions |
| --- | --- | ---: | ---: | ---: |
| `linux_final_defaults_full_c` (bare defaults; trace-only override) | Linux headless | **11.4230 s** | **116.9162 s** | 0 |
| `linux_final_candidate_full_b` (final settings, explicit flags) | Linux headless | 11.4880 s | 117.3531 s | 0 |
| `windows_final_candidate_full_b` (final settings) | Windows native | **10.9938 s** | **110.9386 s** | 0 |
| `linux_baseline_a` (original submission) | Linux headless | 12.3304 s | 129.0319 s | 0 |
| `linux_original_full_b` (original submission, repeat) | Linux headless | 12.4224 s | 133.3047 s | 0 |

Full-run Linux telemetry shows tracking-error p95 improving from 0.395 m to
0.253 m and maximum error from 0.703 m to 0.416 m. Corner-exit median actual
speed at racing-line arc 19–22 m improved from 3.899 to 5.367 m/s. The report
includes rejected trials, commands, a plot, and clearance/smoothness limits.
Original full-run referee JSONs are retained in `results/submitted/`.

The earlier GitHub run loaded the right model but took 130.212 s. Its race
completed; the workflow was cancelled during cache cleanup. Check the fork's
Actions page for the correction's validation rather than treating that run
as a green workflow or a final score.

### Earlier Windows-only corner-exit comparison

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
| `20261016T062854Z` (official evaluate.sh, clean clone, bare defaults) | **11.0458 s** | **111.0577 s** | 0 | COMPLETE, scored |
| `cleanclone_20261016_a` (clean source/build, bare defaults) | **11.0478 s** | **111.0947 s** | 0 | COMPLETE, scored |
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
The additional clean-clone run recorded 0.543.
The final official-script run recorded 0.514.
The corrected Linux tests recorded about 1.006–1.016; the final Windows
cross-check recorded 0.673. Linux used the pinned official `2026-icra` compete
simulator with headless graphics in an isolated ROS domain in Docker.
The October comparison runs ranged from 0.458 to 0.698; the earlier pre-change
run recorded 0.952. Simulator pacing varied during testing.

| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core Ultra 7 255HX | NVIDIA GeForce RTX 5060 Laptop GPU (8 GB) | 16 GB (15.4 GiB visible) | Lenovo 83F3; Windows 25H2, build 26200, 64-bit (product label: Windows 10 Home Single Language); Linux headless in ROS Humble Docker | About 1.01 (Linux headless); 0.673 (final Windows cross-check) |

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

The recorded resumed stage took 197.493 seconds and completed 13 exploratory
episodes; earlier training time is not included. The reward encourages forward
progress and completed laps, and penalises collisions, leaving the course,
tracking error, unsafe speed, large/changing actions, and safety rejections.
Exact weights, offline racing-line generation, and fresh/resumed training
commands are documented in `docs/reproduce-selected-driver.md`.
The historical trainer checkpoint is local and not committed, so the repository
does not support byte-identical retraining of the selected weights. It includes
the exported weights, metadata, and producer code needed for race inference.

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

- The latest controls have two final Linux ten-lap runs and a Windows
  cross-check; the bare defaults were tested on Linux. Earlier comparisons
  above use earlier controls and are labelled accordingly.
  Different simulator pacing or judging hardware may change the results.
- Individual command jumps remain. The corrected headless run does not
  establish reduced braking frequency or physical jerk; jerk was not measured.
- The geometric preview is deliberately excluded from tight turns; its model
  does not guarantee grip or yaw stability.
- Headless testing used the existing image. Follow the correction's GitHub
  Judge run for fresh image validation. Cross-platform timing equivalence
  and unchanged collision risk on other hardware are not guaranteed.
- A sampled 0.215 m clearance-margin disk flagged 32/2413 corrected Linux
  samples versus zero in the reference; a 0.135 m disk flagged zero in both.
  This proxy is not a full oriented-footprint proof. Actual collisions were
  zero and tracking error decreased; see the report for limits.

## Local preparation status

The team leader authorised continuation with publication. The prepared branch
has been pushed to the confirmed public fork. GitHub Judge validation and
notification to the organisers are separate remaining steps; no organiser
notification has been sent by Codex.

Prepared locally:

- All five member names, matric numbers, and email addresses are complete.
- Confirmed destination: `https://github.com/bob-the-doorknob/og7_track2`
  (the configured `team` remote). GitHub metadata confirms it is a public fork
  of `NTUDeepSpeed/Recruitment_Hackathon_2627`, with the official `track2`
  branch present. Prepared submission branch: `submission/og7-track2`.
  It is published on that fork. Internal draft validation PR:
  `https://github.com/bob-the-doorknob/og7_track2/pull/1`.
- The corrected implementation is frozen in commit
  `5dd751189e21984fb49949f8092d2d3865107b02`. The earlier published
  `a24a8c0` remains the exact pre-correction rollback. Simulator binaries,
  build/install/log directories, and unrelated ZIP archives are excluded.
- A local Git bundle of that commit was cloned into the container. The
  official integrity check passed all 64 protected files with no extra files;
  both ROS packages built, 63 driver tests passed, and all eight installed
  runtime assets matched source bytes.
- The clean clone's bare driver completed ten timed laps with no parameter
  overrides: 11.0478 s best lap, 111.0947 s total, zero collisions. Its original
  result is `results/submitted/OG7__cleanclone_20261016_a.json`.
- The unmodified `scripts/evaluate.sh` also completed a full clean-clone run
  with `--team OG7 --runs 1 --no-simulator` and the clean race workspace:
  11.0458 s best lap, 111.0577 s total, zero collisions. The original result
  is `results/submitted/OG7__20261016T062854Z.json`.
- The final review record is `docs/submission-review-2026-10-16.md`.
  The latest controller evidence is `docs/headless-pace-2026-10-16.md`.
  Publication was authorised; the organisers' notification is pending.

Check the Judge workflow, then provide the organisers with the repository URL
and branch before
18 October 2026, 23:59 SGT, as specified in `docs/07-submission.md`.
Judge does not automatically run on a push to `submission/og7-track2`: its
push filter covers `main` and `track2`. Follow `docs/SUBMISSION_HANDOVER.md`
to dispatch it manually for the submission branch or use an internal pull
request targeting an eligible branch.
