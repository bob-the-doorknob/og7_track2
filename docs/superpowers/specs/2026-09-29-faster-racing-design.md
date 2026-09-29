# Track 2 faster racing design

Date: 29 September 2026

## Goal and scope

Minimize the fastest penalized lap and penalized 10-lap total on the official
ICRA 2026 compete circuit. Complete every run with zero collisions as the
reliability target. Use the maximum speed the simulated car can actually
achieve when the track allows it; impose no arbitrary global speed ceiling.
Choose lower local speeds where grip, braking distance, track geometry,
sensing, or control require them. Keep the existing 4.4 m/s selected policy
and baseline available as rollback references.
Submission preparation (old Task 7) is outside this project.

## Current state and constraints

The current driver follows a geometrically smoothed, map-constrained line with
pure pursuit. `SpeedProfile` limits speed from path curvature and runs a
backward braking pass, but has no forward acceleration pass or combined tire
force constraint. `SpeedController` regulates normalized throttle with PI and
has allowed a 4.4 m/s target to overshoot to 4.76 m/s. `LidarSafety` uses at
most 10 m of scan range and assumes 2 m/s^2 deceleration. Even with no visible
obstacle, its current formula limits the requested speed to about 6.24 m/s.
For illustration, stopping from 15 m/s at that deceleration takes 56.25 m
before margins and latency. Raising `target_speed` alone cannot achieve the
goal; the actual maximum achievable speed must be measured.

The map describes a roughly 54.4 m lap, about 2 m wide typically and under
1 m at its narrowest point (`maps/README.md`). The official rules permit
ground-truth pose and offline line generation. They require a real-time capable
40 Hz driver, preserve the existing `ros2 run team_driver driver` entry point,
and prohibit edits to the referee, devkit, `scripts/`, `docker/`, `install/`,
`maps/tracks.yaml`, and workflow files. A collision adds 10 seconds; only a
completed run scores. The baseline evidence is in `docs/track2-44-comparison.md`.

## Selected approach

Use a model-based, track-aware baseline before training a new residual policy.
This gives the car a fast and explainable reference trajectory, while RL can
learn small steering and speed corrections from the actual simulator. Pure
end-to-end RL would require more exploration and provides less direct control
over track and speed constraints. A full nonlinear MPC is a possible later
upgrade, but requires a better-identified vehicle model and more runtime
budget than the first iteration needs.

### 1. Measure the simulated car

Record timestamped pose, speed, steering, throttle, LiDAR clearance, projected
path arc, lap count, and collision count during non-scored calibration runs.
Estimate the car's achievable top speed, usable positive acceleration, braking
deceleration, steering response, tracking error, and lateral acceleration by
speed and curvature. Use robust lower-bound braking and grip estimates for
planning, not best observed values.
Keep raw traces and the derived parameter summary reproducible. Do not reset
or interfere with the referee during scored runs.

### 2. Generate a faster track-boundary-aware line offline

Build a new generator inside `team_driver` (the protected `scripts/` directory
must remain untouched). Start with the existing centerline and occupancy grid.
Represent candidate lines by bounded lateral offsets, reserve footprint and
uncertainty clearance from the ducts, and check both sampled points and links
against occupied cells. Optimize estimated lap time, accounting for path
length, curvature, measured grip, braking, and acceleration. A minimum
curvature line may be an initial candidate, not the final objective. Corner
entry, apex, and exit positions emerge from the optimized full-lap time; no
fixed late-apex rule is applied to every bend. Export the chosen line and its
provenance as reproducible files under `maps/`.

### 3. Plan speed around the full closed lap

Compute local corner speed from measured lateral capability and curvature.
Apply a backward pass for braking before bends and a forward pass for
acceleration out of bends. Limit combined braking/acceleration and cornering
force so the plan does not demand full tire grip in both directions at once.
Add control latency and model-uncertainty margins. Limit every planned speed
by the measured vehicle capability and any tighter local constraint, with no
fixed numeric global cap. In a gentle corner, maintain or increase speed if
curvature, clearance, and the following braking zone allow it and the predicted
full-lap time improves. In sharper corners, accelerate when the upcoming path
opens and remaining lateral demand leaves longitudinal capacity; a simple
apex crossing is insufficient.

### 4. Track the reference and govern speed online

Retain the ROS topic contract and compact NumPy runtime. Compare pure pursuit
with a curvature-aware preview controller, then select using measured tracking
error and scored lap time. Feed forward the planned path curvature and use
odometry feedback to correct steering. Use the planned speed and measured
speed in a throttle/brake controller with anti-windup and explicit overspeed
braking. On an open straight, allow the motor to accelerate toward its measured
achievable speed until the next planned braking point. Anticipate actuator and
sensor delay so measured speed stays within the locally planned grip and
clearance envelope. If the car overshoots that local envelope, increase the
guard margin or reduce the requested profile and repeat evaluation.

LiDAR remains an independent live obstacle and sensor-health check. Its
current 10 m range cannot certify an emergency stop from speeds whose braking
distance exceeds the visible clearance. The
controller must use the known map and pose for advance boundary planning,
reserve conservative margins for uncertainty, and treat unexpected LiDAR
obstacles or missing telemetry as reasons to slow down. Never claim that the
existing swept-path check alone makes high-speed driving safe.

### 5. Retrain residual RL against the new baseline

Do not reuse the selected 4.4 m/s actor at higher speed: its observation
normalization clips speed and baseline target at 4 m/s, and its training
distribution is different. Version the observation and weight format. Include
speed, heading and lateral errors, local curvature ahead, upcoming speed
profile, baseline steering/target, and LiDAR clearance at scales appropriate
to the new operating range. Keep bounded steering and speed residuals and
apply the same map, grip, LiDAR, and local speed constraints after every
RL proposal. The environment must share the race driver's reference plan and
safety logic. Train several seeded candidates; preserve the 4.4 m/s model as
the previous champion until a new actor wins scored comparisons.

## Evaluation and selection

Change one class of behavior at a time: vehicle estimates, line, speed plan,
tracker, then RL. For each candidate, record the exact configuration and run
repeated official referee evaluations against the same simulator build.
Compare `status`, collision count, best penalized lap, penalized 10-lap total,
per-lap spread, maximum measured speed, maximum tracking error, and command
callback time. Promote only a candidate that completes repeated runs, stays
within the locally planned safe-speed envelope, and improves both scored time
components without a collision regression. Retain the previous champion and
a selectable classical fallback. No candidate becomes the default solely
because an unscored lap looks fast.

## Failure behavior

Missing or stale odometry, an invalid scan, non-finite geometry, or an invalid
policy output causes a neutral or braking command and a throttled diagnostic.
If the new line fails clearance checks, retain the previous validated line.
If the new RL actor fails to load or misses its runtime budget, run the new
classical baseline. The driver must send neutral commands on shutdown.

## References

- Kapania, Subosits, and Gerdes, *A Sequential Two-Step Algorithm for Fast
  Generation of Vehicle Racing Trajectories*:
  https://arxiv.org/abs/1902.00606
- TUMFTM, *Global Racetrajectory Optimization*:
  https://github.com/TUMFTM/global_racetrajectory_optimization
- Rosolia and Borrelli, *Learning How to Autonomously Race a Car*:
  https://arxiv.org/abs/1901.08184
- Local competition rules: `docs/06-rules.md` and `docs/05-evaluation.md`.
