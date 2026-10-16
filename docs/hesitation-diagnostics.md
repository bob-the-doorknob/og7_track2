# Diagnosing hesitation before changing the selected controller

For the latest controller selection, stability comparison, and additional
telemetry, see [the 16 October investigation](corner-exit-2026-10-16.md).
References to the selected default below describe the earlier October trials.

The sub-12 settings and weights remain the reference. The opt-in trace records
each scan's baseline target, policy target, final safety-filtered target, speed,
steering, throttle, residual multiplier, path clearance, map certification,
odometry age, wall callback interval, and planning/publication time. Normal
execution has tracing disabled.

## Capture a practice run inside the RoboRacer container

Stop the existing driver with Ctrl+C first. Keep only one driving process.
Source the worktree in this shell so ROS selects the implemented driver.

```bash
cd /hackathon/.worktrees/task1-residual-rl-baseline/race_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
TRACE="/hackathon/results/hesitation_$(date -u +%Y%m%dT%H%M%SZ).csv"
ros2 run team_driver driver --ros-args \
  --params-file src/team_driver/config/fast_corner_24_residual_v2_sub12_params.yaml \
  -p trace_path:="$TRACE"
```

Drive 3-5 laps, then press Ctrl+C to send neutral controls and save the trace.
The recorder reserves a new file at startup and refuses to overwrite one.
It buffers the last 24,000 scans in memory (10 minutes at 40 Hz); older scans
are dropped and reported. There are no CSV writes during driving. A force kill
or container crash loses the buffered trace. Use Ctrl+C / clean shutdown.

```bash
python3 -m team_driver.diagnostics "$TRACE"
```

Summary counts identify where to inspect; they do not establish a fault.
`near_target_braking_samples` counts braking above 0.01 command magnitude when
actual speed is within 0.2 m/s above the final target and safety has not lowered
the policy target by more than 0.05 m/s. Normal planned braking can qualify.
`safety_limited_samples` includes both clearance clamps and residual rejection
that lowers the target. `accel_brake_switches` ignores neutral samples and
counts sign transitions above 0.01 magnitude. Normal corner braking also adds
switches, so look for repeated clusters in the CSV at the same track position.

Wall gaps above 100 ms are a diagnostic threshold, not a safety fault. Compare
gaps with odometry age, compute time, and simulator pacing: slower-than-real-time
simulation can lower wall frequency without a driver fault. `compute_ms`
includes preprocessing, planning, and command publication, but excludes trace
recording and marker publication; their time can appear in the next wall gap.
`stale_odom` means the existing 0.5 s freshness check held neutral controls.
Blank stage fields mean that stage was not reached or was disabled.

## Decide which experiment to run

1. Repeated near-target acceleration/braking: test a positive-torque taper in a
   separate candidate; preserve necessary corner braking.
2. Abrupt drops from policy to final target: inspect path clearance, steering,
   map certification, and residual rejection before changing safety logic.
3. Drops already present in the policy target: inspect the residual multiplier
   and baseline profile to distinguish learned slowdowns from planned braking.
4. Large wall gaps or stale odometry: address simulator/bridge/control scheduling
   before changing speed or steering gains.

Change one mechanism per candidate and compare repeated scored ten-lap runs,
including collisions, best lap, total time, and lap consistency. A diagnostic
practice run is not evidence that a faster candidate is race-ready.

## Captured evidence and first candidate (5 October 2026)

The trace `/hackathon/results/hesitation_20261005T125358Z.csv` retained 3,901
samples over 61.351 wall seconds. There were 446 direct acceleration/braking
switches, 443 within 50 ms of the preceding sample. Of those, 370 had a target
change below 0.1 m/s. The target stayed exactly 6.3718 m/s at elapsed times
5.0317 and 5.0491 s while speed changed from 6.3781 to 6.4468 m/s and throttle
changed from +1.0 to -0.0225. This directly demonstrates the discontinuity at
the existing 0.05 m/s active-braking threshold.

Odometry and LiDAR remained valid in all samples; maximum odometry age was
84.4 ms. Median computation time was 1.586 ms and its 99th percentile was
2.631 ms. Four wall gaps exceeded 100 ms, but none triggered stale odometry.
Safety limited 2,837 samples (72.7%), with no residual steering rejections.
Those safety limits are not automatically faults; inspect them after the
first controller experiment.

The optional candidate
`config/fast_corner_24_residual_v2_sub12_smooth_params.yaml` adds only
`speed_approach_band_mps: 0.3`. Positive torque is multiplied by the speed
deficit divided by 0.3, capped between zero and one. Deficits of at least
0.3 m/s keep the original positive command. Above target, no positive torque
is applied. The existing active-braking threshold and negative commands are
unchanged. The original config and bare-driver default keep the band at zero.

Replay of the recorded speed/target/time samples reproduced every original
command exactly. On the same fixed telemetry, command jumps exceeding 0.5
fell from 351 to 63 with the candidate, and every recorded braking command
was unchanged. Replay does not simulate the vehicle responding to new
commands: it proves command-level smoothing, not fewer closed-loop switches,
faster laps, or collision-free driving. The candidate may also approach its
target more slowly, which is why it needs a fresh driving comparison.

Stop the old driver, then capture a candidate practice run:

```bash
cd /hackathon/.worktrees/task1-residual-rl-baseline/race_ws
source install/local_setup.bash
TRACE="/hackathon/results/hesitation_smooth_$(date -u +%Y%m%dT%H%M%SZ).csv"
ros2 run team_driver driver --ros-args \
  --params-file src/team_driver/config/fast_corner_24_residual_v2_sub12_smooth_params.yaml \
  -p trace_path:="$TRACE"
```

After 3-5 laps, press Ctrl+C, then run
`python3 -m team_driver.diagnostics "$TRACE"`. Compare hesitation and lap
times before promoting this configuration. Formal ten-lap scoring remains
required for performance selection.

The detailed report and plot are beside the original trace as
`hesitation_20261005T125358Z.analysis.json` and `.analysis.png`. To regenerate
them, use the already-installed plotting dependencies in the training venv:

```bash
cd /hackathon/.worktrees/task1-residual-rl-baseline/race_ws/src/team_driver
training/.venv/bin/python -m training.analyze_control_trace \
  /hackathon/results/hesitation_20261005T125358Z.csv
```

## Actual candidate trace and paired scored evaluation

The candidate practice trace is
`/hackathon/results/hesitation_smooth_20261005T130410Z.csv`.
Compared with the original practice trace, command jumps exceeding 0.5 fell
from 5.721 to 0.877 per wall second (84.7% reduction). Acceleration/braking
switches fell from 7.270 to 4.651 per wall second (36.0% reduction). Mean
measured speed over reconstructed complete laps was 4.506 versus 4.501 m/s.
The 95th percentile of absolute cross-track error fell from 0.248 to 0.193 m.
There was one initial stale-odometry sample before the first odometry arrived;
no later stale samples or invalid LiDAR were recorded. Practice traces contain
no collision telemetry and are not scored race evidence. Their comparison
JSON and plot are beside the candidate trace as `.comparison.json` and
`.comparison.png`.

Both configurations were then evaluated sequentially against the connected
native simulator with the unmodified referee: 2 warm-up laps and 10 timed laps.

| Configuration | Best scored lap | Ten-lap total | Collisions | Real-time factor |
| --- | ---: | ---: | ---: | ---: |
| Original sub-12 | 11.1159 s | 113.8048 s | 0 | 0.846 |
| Smooth, 0.3 m/s band | 11.2129 s | 113.6795 s | 0 | 0.713 |

Both JSONs have `status: COMPLETE` and `scored: true`. The total improvement
was only 0.1253 s (0.11%), while the candidate's best lap was 0.0970 s slower.
This one pair establishes clean completion and smoother measured controls,
but no reliable performance advantage. The original remains the selected
default. Keep the smoothing candidate available; compare repeats or a narrower
approach band before changing performance selection. Wall pacing differed
between runs; referee lap times are the timing evidence, and pacing can still
affect the wall-timed controller's behaviour.

Unedited official results:

- `/hackathon/results/hesitation_comparison/DeepSpeed__hesitation_original_20261005_a.json`
- `/hackathon/results/hesitation_comparison/DeepSpeed__hesitation_smooth_20261005_a.json`

`comparison_provenance.json` in the same directory records config, actor, and
runtime source hashes. Evaluation launch needs the referee from the main
workspace beneath the driver's worktree overlay:

```bash
source /opt/ros/humble/setup.bash
source /hackathon/race_ws/install/local_setup.bash
source /hackathon/.worktrees/task1-residual-rl-baseline/race_ws/install/local_setup.bash
```

## Narrower-band experiment

`config/fast_corner_24_residual_v2_sub12_smooth15_params.yaml` changes only
the approach band to 0.15 m/s. The integration test verifies that all other
parameters match the original reference and that ROS loads the 0.15 value.
All 46 driver tests passed before evaluation. The driver used the external
evaluation parameter copy `results/hesitation_comparison/smooth15_a_params.yaml`,
which adds only a trace destination to the candidate.

The official ten-lap result
`DeepSpeed__hesitation_smooth15_20261005_a.json` is COMPLETE and scored, with
zero collisions, 11.1489 s best lap, and 114.1138 s total. Its total was
0.3090 s slower than the original and 0.4343 s slower than the 0.3-band run.
Its best lap was 0.0330 s slower than the original. These small differences
from single trials do not establish a general ranking, and the candidate
has not shown a time improvement.

The scored run saved 11,301 control samples in `smooth15_a_trace.csv`.
It contained 432 jumps larger than 0.5: 38.23 per 1,000 samples versus
89.98 in the original practice trace, approximately 57.5% fewer. This is a
comparison of different sessions, including warm-up samples in the scored
trace, rather than a matched-pacing experiment. There was one stale-odometry
sample at initial startup, no later stale telemetry, no invalid LiDAR, no
residual rejection, and no wall callback gap above 100 ms. The trace contains
no dropped samples. The candidate's trace comparison report and plot are
`smooth15_a_trace.comparison.json` and `.comparison.png`.

Selection remains unchanged: original for the existing speed reference,
0.3-band candidate when evaluating reduced hesitation. Narrowing the band
has not yielded a better scored time. Further speed work should examine
where the planned path and safety speed cap constrain corner exits rather
than assume another throttle-band adjustment will beat the reference.

## Corner-exit lookahead experiment

In the original practice trace, 2,827 moving samples had a safety target
reduction above 0.05 m/s. In 1,346 of them, reported path clearance was within
0.101 m of the theoretical visible horizon
`min(10, pi / (2 * abs(tan(steering * 0.5236) / 0.324)))`.
The remaining 1,481 encountered returns before that horizon. This distinguishes
samples near the constant-steering visibility bound from shorter clearances;
it does not prove that either bound was unnecessarily restrictive.

The new experimental config
`config/fast_corner_24_residual_v2_sub12_lookahead16_params.yaml` changes only
maximum lookahead from 1.5 to 1.6 m. It uses the original throttle behaviour,
actor, speed limits, and safety limits. The integration test verifies this
single change and ROS parameter loading; all 47 driver tests passed.

| Run | Best scored lap | Ten-lap total | Collisions | Real-time factor |
| --- | ---: | ---: | ---: | ---: |
| Original A | 11.1159 s | 113.8048 s | 0 | 0.846 |
| Lookahead 1.6 A | 11.085 s | 112.883 s | 0 | 0.76 |
| Original B | 11.044 s | 112.452 s | 0 | 0.67 |
| Lookahead 1.6 B | 10.963 s | 112.632 s | 0 | 0.71 |

Values shown to 0.001 s for new runs; exact values are in the referee JSONs
and `lookahead16_provenance.json`. All four runs completed ten scored laps.
The first candidate total beat Original A by about 0.922 s, but the candidate
repeat was about 0.180 s slower than Original B. The lower best lap is
promising, but these mixed results do not establish a repeatable ten-lap gain.
Original remains the default; the 1.6 m config remains an explicit experiment.

The first candidate saved `lookahead16_a_trace.csv`, with no dropped samples,
no invalid LiDAR, no residual rejection, and one initial pre-odometry neutral
sample. No wall gap exceeded 100 ms. Absolute cross-track error at its 95th
percentile was 0.280 m versus 0.248 m in the earlier original practice trace,
so this change did not demonstrate improved tracking. No claim that the
safety limiter itself was improved follows from this experiment. Its trace
comparison JSON/plot are saved beside the trace.

Unedited race evidence is under `/hackathon/results/hesitation_comparison/`:
`DeepSpeed__hesitation_lookahead16_20261005_a.json`,
`DeepSpeed__hesitation_lookahead16_20261005_b.json`, and the original A/B JSONs.
The provenance file hashes the evaluated parameters, actor, driver, speed
controller, path controller, and safety filter. Run A recorded controls via
an external params copy; both B repeats disabled tracing. The two lookahead
candidate runs completed with zero collisions over 20 timed laps.
