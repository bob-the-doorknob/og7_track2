# Middle-wall turn-exit investigation, 2026-10-06

This is historical evidence. The later repeat and profile investigation is in
[the 16 October report](corner-exit-2026-10-16.md). The older `wall_exit`
configuration described here is an experiment, rather than the current default.

## Cause traced before the control change

The running driver used `fast_corner_24_residual_v2_sub12_smooth15_params.yaml`
and packaged `residual_policy_v2_sub12_candidate.npz`. A passive observer recorded
1,256 raw scans and poses without publishing driving commands. Replay at the
pictured exit (approximately racing-line arc 20 m, used only for analysis) found:

| Quantity | Median |
| --- | ---: |
| Measured speed | 4.34 m/s |
| Baseline target | 6.69 m/s |
| Actor target | 6.87 m/s |
| Final safety target | 4.46 m/s |
| Constant-steering arc clearance | 3.80 m |
| Cross-track error | 0.10 m |
| Heading error | -0.014 rad |

`LidarSafety._path_clearance` holds the current steering for its whole forward
projection. The resulting arc meets a wall even though the actual racing line
straightens over the next metre: route curvature falls from -0.142 to -0.017/m.
This is a projected intersection, not a blanket penalty for the nearest wall.
The speed limit is `sqrt(2 * 2.8 * (3.8 - .25)) = 4.46 m/s`.
The footprint includes half-width .135 m plus side margin .08 m.

Other inspected mechanisms:

- Pure pursuit uses `.55 + .25 * speed`, capped at 1.5 m. Its steering curvature
  speed limit and the waypoint profile both allow materially more speed here.
- The profile already accounts for forward acceleration, braking into upcoming
  curves, and the vehicle friction ellipse. This remains active.
- The actor is not requesting the observed 4.46 m/s target.
- Forward LiDAR validity requires at least ten rays and 60% valid coverage in
  the 35-degree cone. Invalid scans still request zero speed.
- Map certification previously lifts only a nearly full 10 m straight horizon,
  with normalized steering <= .08; it cannot lift a 3.8 m arc collision.
- Residual steering rejection applies below .9 m clearance; this is not the
  median exit slowdown. It remains active.
- The existing .15 m/s throttle taper acts near the target, while active braking
  begins above a .05 m/s overspeed. The low safety target causes earlier braking;
  increasing acceleration gains would not remove its cause.

## Focused change

An optional `ExitPreview` replaces the long fixed-steering projection only on
aligned, easing turn exits. It holds the actual proposed steering for .15 s
(at least .25 m), then simulates the existing pure-pursuit follower. It does not
change the steering command, actor, racing line, lookahead, global 7.2 m/s ceiling,
or speed-control gains.

Eligibility requires cross-track <= .20 m and heading error <= .20 rad. An easing
turn has current absolute route curvature >= .05/m and curvature one metre ahead
<= 60% of current. A straight continuation has current absolute curvature < .05/m
and curvature one metre ahead <= .10/m. The profile one metre ahead must permit
the smaller of the actor's target and the preview's finite stopping-speed cap.
The entire preview footprint must be free in the map, including unknown cells,
with padding for sampling and map resolution. LiDAR obstacles are intersected
with the preview footprint. A parallel wall outside that footprint does not
reduce speed; a crossing wall or obstacle on the path still does.

The preview ends at six metres or before its heading leaves the forward LiDAR
hemisphere. Its finite endpoint still imposes a stopping-distance cap. Six
metres was chosen because a full ten-metre follower projection reaches the next
hairpin and correctly fails the map check. The new target can rise by at most
1.0 m/s over the old arc cap and cannot exceed the new obstacle cap. Invalid
or uncertified previews use the original logic. No coordinate gate or forced
minimum speed bypasses safety.

A clear preview only replaces the original arc when it offers more clearance;
this avoids reducing speed in sections where the original arc was already safe.
An actual LiDAR obstacle on the preview still imposes its stopping-distance cap.

Final recorded replay raised the median target in arc 20 from 4.46 to 5.46 m/s;
all of those samples qualified. At arc 21 the median target rose from 5.24 to
5.67 m/s, preserving momentum after straightening. These are replay targets,
not measured faster driving or scored lap-time results.

## Parameters and rollback

Candidate: `config/fast_corner_24_residual_v2_sub12_wall_exit_params.yaml`.
Reference: `config/fast_corner_24_residual_v2_sub12_smooth15_params.yaml`.
The feature defaults off, preserving previous configurations.

| Parameter | Candidate value | Meaning |
| --- | ---: | --- |
| `safety_exit_preview_enabled` | true | Set false to restore the old projection. |
| `safety_exit_preview_max_speed_gain_mps` | 1.0 | Maximum lift above the original arc cap. |
| `safety_exit_preview_hold_s` | .15 | Longer holds are more conservative about straightening. |
| `safety_exit_preview_horizon_m` | 6.0 | Finite preview distance; shorter distances lower the stopping cap. Longer paths may fail map checks. |
| `safety_exit_preview_curvature_ratio` | .60 | Larger values allow less pronounced easing. Must be below 1. |
| `safety_exit_preview_max_cross_track_m` | .20 | Larger values allow preview when farther off line. |
| `safety_exit_preview_max_heading_rad` | .20 | Larger values allow more heading error. |

Tune the speed-gain bound first. Preserve the shared stop/side margins and
deceleration limit when tuning this section; they affect the whole track.
Watch understeer after an accelerated exit, loss of tracking, wall contact,
preview on/off target oscillation, and callback latency. The preview predicts
future baseline steering; it cannot guarantee future residual corrections,
traction, or actuator response.

## Verification artifacts

Passive capture and replay: `/hackathon/results/hesitation_comparison/middle_wall_20261006_before.*`.
Official results, exact run YAMLs, and traces: `/hackathon/results/middle_wall/`.
The short-watchdog reference run `middlewall_reference_20261006_a` was manually
stopped after two laps to restart with sufficient wall time. Reference B reached
nine clean laps but expired at 480 wall seconds after the simulator was minimized
or paused (confirmed by the user). Both references are unscored; exclude them
from complete-run comparisons. Referee files and scoring code remain unchanged.

Initial candidate A completed ten laps with zero collisions, best 10.9988 s and
total 117.3202 s. Its measured median speed at arc 20 rose from reference B's
4.44 to 5.37 m/s, but this was not an overall race-time improvement. The final
refinement preserves already-clear arcs and continues preview eligibility on
aligned straight exits, preventing the early preview cut-off and subsequent
braking at arc 21. `provenance.json` records initial source hashes;
`provenance_v2.json` records refined source hashes for reference C and candidate B.

### Completed final comparison

| Official run | Best lap | Ten-lap total | Collisions | Real-time factor |
| --- | ---: | ---: | ---: | ---: |
| Reference C | 11.2349 s | 118.1325 s | 0 | .652 |
| Refined candidate B | 10.9398 s | 113.1185 s | 0 | .699 |

Both are COMPLETE and scored by the unchanged referee, with two unscored warm-up
laps plus ten timed laps. The candidate improved this paired total by 5.014 s
(4.24%). This is one completed paired comparison; simulator timing and lap
variation mean it is not a guarantee of the same gain on every subsequent run.

Measured steady-state traces (after the first 60 wall seconds):

| Section | Reference speed | Candidate speed | Reference braking | Candidate braking |
| --- | ---: | ---: | ---: | ---: |
| Turn exit, arc 19 | 4.72 m/s | 5.12 m/s | 39.0% | 19.5% |
| Middle wall, arc 20 | 4.41 m/s | 5.43 m/s | 12.0% | 10.6% |
| Aligned exit, arc 21 | 4.75 m/s | 5.59 m/s | 3.9% | 2.2% |

Speeds are medians; braking means throttle < -.01. Arc 20's tenth-percentile
speed rose from 4.27 to 5.24 m/s; arc 21's rose from 4.32 to 5.46 m/s. Cross-track
95th percentiles stayed similar: .111 to .109 m at arc 20, .088 to .086 m at arc 21.
Candidate maximum observed controller computation was 9.98 ms, with no callback
gap over 100 ms or telemetry fault in the steady trace. Global throttle switching
remains; this change addresses braking and momentum at safe wall-adjacent exits.
The plot and full per-section statistics are
`middlewall_candidate_20261006_b.analysis.png` and `.analysis.json`.

The package built successfully and all 56 controller tests passed, including
parallel walls, crossing walls, invalid scans, invalid/out-of-map previews,
occupied interior footprint cells, original clear-arc preservation, eligibility
gates, bounded gain, upcoming braking, and real-driver integration. The evaluation
driver shut down cleanly and sent neutral controls after completion.

### Start the selected wall-exit configuration

In the RoboRacer container, with the existing bridge running, use:

```bash
ws=/hackathon/.worktrees/task1-residual-rl-baseline/race_ws
source /opt/ros/humble/setup.bash
source "$ws/install/local_setup.bash"
ros2 run team_driver driver --ros-args --params-file "$ws/src/team_driver/config/fast_corner_24_residual_v2_sub12_wall_exit_params.yaml"
```

The root `/hackathon/race_ws` driver is still the template; the worktree overlay
above selects the implemented controller and packaged actor. Evaluation left no
driver running. Run one driver at a time. Feature rollback: set
`safety_exit_preview_enabled: false` or use the preserved smooth15 configuration.
