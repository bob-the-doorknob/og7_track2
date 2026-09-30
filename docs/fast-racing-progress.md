# Faster racing experiments

Update, 30 September 2026: a separate classical config completed two clean
ten-lap scored runs with every lap below 12 seconds. See
[`sub12-racing-2026-09-30.md`](sub12-racing-2026-09-30.md) for the selected
config and unedited referee evidence. The historical experiments below remain
as recorded.

The 4.4 m/s residual RL driver remains the default in `driver_params.yaml`.
The new classical candidate is selectable with
`config/fast_baseline_params.yaml`; it uses the same generated racing line,
measured vehicle limits, a closed-lap braking and acceleration speed plan,
and active throttle/brake control. It has no arbitrary 15 m/s cap. The
`top_speed_mps` value in `config/vehicle_limits.yaml` is the fastest speed
observed in current clean calibration runs, not the motor's known maximum.
The current 10 m LiDAR guard still limits the requested speed on a clear
straight. Faster work will require map-aware advance planning and new
calibration.

| Driver | Timed laps | Status | Collisions | Best penalized lap | Penalized total |
| --- | ---: | --- | ---: | ---: | ---: |
| Prior selected 4.4 m/s RL (`rl44_b`) | 10 | COMPLETE | 0 | 19.7158 s | 197.7789 s |
| Fast classical v2 pilot | 3 | COMPLETE | 0 | 19.336 s | 58.178 s |
| Fast classical v2 full run | 10 | COMPLETE | 0 | 19.340 s | 194.0049 s |
| Fast classical v3 delay-preview experiment | 10 | COMPLETE | 0 | 19.418 s | 194.6293 s |
| Fast classical + offline-optimized line pilot | 3 | COMPLETE | 0 | 19.850 s | 60.223 s |

The full candidate run improved the prior selected run by 0.3758 s on best
lap and 3.7740 s over ten timed laps. Its result is the unedited
`results/submitted/DeepSpeedRobot_fast_baseline_v2__20260929T134611Z.json`.
The full run was against the connected Windows AutoDRIVE compete build on
29 September 2026; its real-time factor was 0.705. No collision was reported.
The measured 16 ms control-delay preview in v3 was slower over ten laps,
despite also finishing cleanly, so v2 is retained. Its unedited result is
`results/submitted/DeepSpeedRobot_fast_baseline_v3__20260929T135535Z.json`.
One earlier v1 fast candidate used a much lower lateral limit, ran two
27.7 s timed laps, and was stopped. Its partial result is local-only and
must not be compared as a complete run.

The Task 3 offline search checked every point and link against the trinary
occupancy map, treating unknown as blocked. It found a candidate with a
0.28 m conservative minimum clearance and reduced the speed planner's
predicted lap from 19.875 s to 19.455 s. On the connected simulator, this
line ran three clean timed laps in 19.850, 19.993, and 20.380 s. That is
slower than the fast baseline pilot's 58.178 s three-lap total, so the new
line was rejected. The unedited pilot result is
`results/submitted/DeepSpeedRobot_fast_line_pilot__20260929T173851Z.json`.
Its line and metadata remain in ignored `results/` for analysis; the
packaged racing line remains unchanged. The offline estimate omits tracking
error, which likely explains the ranking reversal and should be included
when tracking is improved in Task 4.

The calibration traces used 10,130 clean samples across 4.4, 5.2, and
6.0 m/s configured trials; observed peaks were 4.83, 5.51, and 6.30 m/s.
The 6.0 m/s trial's peak exceeded its configured target because throttle
controls acceleration, not speed directly. Active braking was absent in the
old controller, so the current 2.80 m/s^2 braking input is a conservative
straight-coasting measurement. The lateral planning limit of 3.06 m/s^2
uses 95% of the 99th percentile clean observed lateral acceleration from
the three calibration runs. These are empirical starting values, not known
friction or motor limits.

The faster classical line and controller have now completed repeat ten-lap
runs. Later tracker changes and RL training will establish the final selection. The default
RL actor was trained at 4.4 m/s and must not be reused with the faster
baseline without retraining its observation and action policy.

## High-speed corner sweep after visual review

The hand-sketched outside-entry/inside-apex line motivated a map-valid
0.35 m outward entry adjustment near path arc 41 m. It kept a conservative
0.291 m map clearance, but scored 53.5544 s over three laps versus 51.9443 s
for the existing line at the same 7.0 m/s² planning setting. It was rejected.
The starting grid pose belongs to the referee; a line can bias where the car
drives after the start, but cannot move the reset pose.

The original measured lateral estimate, 3.06 m/s², reflected the earlier
slow runs and did not establish the simulator's maximum corner capability.
`planning_lateral_mps2` now allows candidate-only cornering sweeps without
rewriting that measured YAML. These are requested planning accelerations,
not measurements of achieved tire force.

| Planning lateral acceleration | Timed laps | Best lap | Penalized total | Collisions |
| ---: | ---: | ---: | ---: | ---: |
| 7.0 m/s² | 10 | 17.1222 s | 174.8131 s | 0 |
| 9.0 m/s² | 3 | 16.9663 s | 51.4716 s | 0 |
| 10.0 m/s² | 3 | 16.4256 s | 51.1168 s | 0 |
| 12.0 m/s² | 3 | 16.0808 s | 48.9810 s | 0 |
| 15.0 m/s² | 3 | 15.3006 s | 46.1528 s | 0 |
| 17.5 m/s² | 3 | 15.5176 s | 46.6200 s | 0 |
| 20.0 m/s² | 3 | 15.1325 s | 46.6659 s | 0 |
| 15.0 m/s² | 10 | 15.4846 s | 161.9163 s | 0 |
| 15.0 m/s² repeat | 10 | 15.6030 s | 160.3860 s | 0 |

The 15.0 m/s² ten-lap run is the leading complete classical candidate.
Its unedited referee JSON is
`results/submitted/DeepSpeedRobot_fast_corner_15_full__20260929T185715Z.json`.
It improves the earlier 6.30 m/s fast baseline by 4.5203 s on best lap and
32.0886 s on ten-lap total. The selected 4.4 m/s RL driver remains the
default until repeat full-run evidence and the new RL training are complete.
The repeat complete run, also collision-free, scored 15.6030 s best and
160.3860 s total. Its unedited result is
`results/submitted/DeepSpeedRobot_fast_corner_15_repeat__20260929T190302Z.json`.
Both complete runs improve both score components relative to the prior
19.340 s / 194.0049 s fast classical run and 19.7158 s / 197.7789 s
selected RL run. The normal `driver_params.yaml` still loads the previous RL
model while residual RL v2 is pending; select `fast_corner_15_params.yaml`
explicitly to drive the new classical candidate.

A separate map-certified 9.0 m/s *requested top-speed* pilot finished three
clean laps in 55.5135 s, slower than the 7.0 m/s² corner pilot's 51.9443 s.
The map guard code and `fast_map_params.yaml` are experimental; the 9.0 m/s
request is not a measurement of achieved speed. The finite LiDAR horizon
alone cannot verify stopping from 9 m/s at the measured 2.8 m/s² deceleration.
The 10 m scan's clear-horizon speed cap is only lifted for a prechecked
map line and an aligned straight pose; detected obstacles still constrain
speed. Its on/off behavior needs telemetry analysis before another full run.

Residual RL v2 training and the subsequent live simulator regression are
recorded in `docs/rl-v2-training-2026-09-30.md`. No v2 actor has been promoted.
