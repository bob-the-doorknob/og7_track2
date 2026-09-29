# Faster racing experiments

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

The new baseline is a candidate until multiple complete ten-lap runs and
the later line/tracker changes establish which version is best. The default
RL actor was trained at 4.4 m/s and must not be reused with the faster
baseline without retraining its observation and action policy.
