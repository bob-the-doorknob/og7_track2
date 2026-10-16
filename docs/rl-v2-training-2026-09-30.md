# Fast-baseline residual RL, 30 September 2026

Latest update: v2 residual RL trained on the proven 11.639-second classical
baseline completed two clean ten-lap scored runs, with a 10.960-second best
lap. See [`residual-rl-sub12-2026-09-30.md`](residual-rl-sub12-2026-09-30.md).

Later update: a resumed v2 checkpoint completed ten scored laps with zero
collisions or penalties (12.726 s best, 127.571 s total). It remains slower
than the separately proven sub-12-second classical driver. See
[`sub12-racing-2026-09-30.md`](sub12-racing-2026-09-30.md). The blocker below
describes the earlier actor and simulator state during that experiment.

The supplied MonacoF1 footage analysis describes a repeatable outside-apex-outside
line, lookahead steering, curvature-dependent speed, braking before bends, and
acceleration on exit. Those are useful racing principles, but the footage
does not establish the other team's exact algorithm. This repository already
has a mapped racing line, speed-dependent pure pursuit, a cyclic
curvature/braking/acceleration speed profile, and a LiDAR safety layer. The
previous outside-entry line candidate was slower in a scored three-lap run,
so its hand-drawn geometry was not copied into the selected map.

## Changes in this experiment

- Residual observation/model format v2 adds signed curvature and planned
  target speeds at 0, 1, and 3 m ahead. The old v1 actor remains loadable.
- The trainer and race driver use the same v2 packing and NumPy actor export.
- A `control_dt_mode: nominal` classical candidate uses 25 ms controller
  integration per LiDAR scan for a simulator pacing comparison. The default
  stays wall-clock based pending a valid paired comparison.

## Training

The connected AutoDRIVE compete simulator was trained outside scored runs with
the bridge as the only other ROS node. The existing manual driver was stopped
first. SAC seed 43 ran 5,000 steps, including 500 zero-residual baseline steps,
with 28 observation values, four LiDAR frames per action, the packaged racing
line, packaged vehicle limits, a 15 m/s^2 requested lateral planning allowance,
and a 10 m LiDAR horizon. It completed 17 one-lap training episodes, each
reported with zero collisions. The local run is
`race_ws/src/team_driver/training/runs/task5_fast_v2_s43/`; it contains the
actor archives, SAC checkpoints, metadata, and episode log. The run directory
is ignored by Git because checkpoints are experimental and large. The final
inference actor and its unedited metadata are preserved under
`race_ws/src/team_driver/training/artifacts/` with the name
`rl_v2_s43_5000_experimental`. They are not packaged or selected by the
default driver configuration. The SAC `.zip` checkpoint remains local to
the run directory.

To repeat, source the ROS and worktree overlays, stop all other driver and
referee processes, then run from `race_ws/src/team_driver`:

```bash
PYTHONPATH=.:$PYTHONPATH /opt/deepspeed-training-venv/bin/python -m training.train_sac \
  --training-mode --simulator-variant compete --simulator-build 2026-icra-compete \
  --vehicle-limits-path config/vehicle_limits.yaml \
  --planning-lateral-mps2 15.0 --observation-version 2 \
  --safety-horizon 10.0 --steps 5000 --baseline-steps 500 \
  --checkpoint-every 1000 --seed 43 --run-id task5_fast_v2_s43
```

The `PYTHONPATH=.:$PYTHONPATH` form is necessary: setting it to `.` alone
hides the ROS Python packages from the training virtual environment.

## Scored evidence and current blocker

The 3,000-step actor scored 22.819 s best and 68.583 s over three timed laps,
with one collision on each scored lap. The 5,000-step actor scored 22.874 s
best and 68.724 s total, also with a collision on each scored lap. These are
penalized results; their roughly 12.8 s raw laps cannot be treated as clean
performance. Unedited JSON files are in `results/submitted/`.

Crucially, a diagnostic pilot using temporary scaling controls with **both
residual controls scaled to zero**
still collided once per lap, scoring 22.823 s over one timed lap. The
unchanged `fast_corner_15_params.yaml` classical driver then reproduced the
same pattern, scoring 22.820 s, and the nominal-dt variant scored 22.773 s.
The classical driver had earlier completed two ten-lap runs without collisions:
15.4846/161.9163 s and 15.6030/160.3860 s for best/total. The new crash
therefore cannot be attributed to the v2 actor. The change in live simulator
behavior must be diagnosed before selecting a policy or claiming a faster car.
The temporary scaling controls were removed from the driver after this
diagnosis; the pilot result JSON remains unedited.

The referee records the simulator's own lap time. Its real-time factor was
about 0.69-0.74 during these pilots and 0.74 in the earlier clean repeat.
Rendering load can change how long a run takes on the wall clock and may
indirectly change control timing. We have not yet made a paired focused vs
background run under otherwise matching conditions, so the reported 1-2 s
focus effect is unconfirmed as a change in scored lap time. Fixed nominal
controller dt did not remove the current collision.

The previous clean classical candidate remains the reference. Keep the old
default 4.4 m/s RL actor and the classical config unchanged. Reestablish a
clean baseline on the same simulator build and track, then compare v2 actor
checkpoints with repeated complete referee runs before promoting any model.
