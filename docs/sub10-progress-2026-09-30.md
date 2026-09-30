# Sub-10 AutoDRIVE search: current status

The selected driver remains `fast_corner_24_residual_v2_sub12_params.yaml`
with `residual_policy_v2_sub12_candidate.npz`. Its proven best lap is
**10.9598 s** on the compete track. Two previously saved ten-lap runs completed
with zero collisions and penalties. The classical `fast_corner_15` and
`fast_corner_24_map72_safety28` backups are unchanged.

## Scored work since the sub-11 run

All pilots used the same `icra26_compete` AutoDRIVE track, two unscored warmup
laps, and three timed laps. These are representative experiments; all raw
referee JSON files remain in the local ignored `results/` directory. The key
side-margin and stacked-policy results are also copied into
`results/submitted/` with `sub10_` prefixes.

| Candidate | Best timed lap | Collisions | Decision |
| --- | ---: | ---: | --- |
| Reduced LiDAR side margin to 0.02 m | 10.939 s | 0 | Ten-lap validation did not improve the selected run |
| Stop margin 0.10 m | 11.009 s | 0 | Reject |
| Braking cap 3.2 m/s² | 11.177 s | 0 | Reject |
| Braking cap 5.0 m/s² | 12.026 s | 0 | Reject |
| Throttle feedforward 0.22 | 11.150 s | 0 | Reject |
| Corner planning 20 m/s² | 11.095 s | 0 | Reject |
| Corner planning 32 m/s² | 11.685 s | 0 | Reject |
| Straight speed request 9.0 m/s | 11.119 s | 0 | Reject |
| Shorter lookahead | 11.560 s | 0 | Reject |
| Longer lookahead | 11.327 s | 0 | Reject |
| Greedy map-clear line | 11.230 s | 0 | Reject |
| Shorter map-clear line | 11.227 s | 0 | Reject |
| Gentler active braking | 11.183 s | 0 | Reject |
| Localized corner speed-cap boost, 1.25x | 11.183 s | 0 | Reject |
| Wider 1.35x corner boost | 11.447 s | 1 in warmup | Reject |

The 0.02 m side margin completed a separate ten-lap run with **zero
collisions and penalties**, a best lap of **11.074 s**, and a total of
**112.148 s**. That compares with the selected driver's saved best of
10.9598 s and 111.841 s. The small three-lap gain did not hold up.

The resumed v2 SAC attempt `task_sub10_v2_s51` restored the actor and replay
buffer, but its first checkpoint scored only 12.052 s best over three clean
timed laps. It was not selected. An earlier launch accidentally omitted its
intended flags because the simulator build string contained spaces; that
misconfigured run was stopped and excluded.

## Stacked residual experiment

The driver and trainer can now take an optional fixed `base_policy_path`.
With a second actor outputting zero, the composition reproduces the saved v2
actor. The diagnostic zero stack completed three scored laps with zero
collisions and 11.147 s best. The first learned second-stage checkpoint
completed three clean scored laps at 11.544 s best, so it was not promoted.

The stacked training environment had repeated off-course and collision
episodes after random exploration began. A separate zero-action probe stayed
on course for 1,000 updates, with exactly one LiDAR frame per update and
cross-track error below the 0.70 m training cutoff. This narrows the problem
to exploration or learning after the warmup phase. The second-stage action
was reduced to one third of the normal steering adjustment and half of the
speed adjustment; its first checkpoint still had off-course training episodes.
That checkpoint scored three timed laps with zero collisions and penalties,
but only **11.177 s** best. It was not promoted.

I made the stacked steering and speed ranges configurable, with the same
settings passed to training and the race driver. A tighter seed-54 run used
0.1 steering scale and 0.25 speed scale. It completed two laps during training
but also repeatedly went off-course at the same bends. All three exported
checkpoints completed clean three-lap referee runs:

| Seed-54 checkpoint | Best timed lap | Three-lap total | Collisions/penalties |
| --- | ---: | ---: | ---: |
| 1,000 steps | 11.031 s | 33.554 s | 0/0 |
| 2,000 steps | 11.272 s | 33.853 s | 0/0 |
| 3,000 steps | 11.200 s | 33.813 s | 0/0 |

The closest, 1,000-step actor then completed a full ten-lap scored run with
zero collisions and penalties: **11.021 s best**, **112.301 s total**. The
selected actor's earlier ten-lap best and total remain better at 10.9598 s
and 111.841 s, so the stacked actor remains experimental.

The seed-55 speed-only experiment fixed second-stage steering at zero and
used 0.5 speed scale. It completed a lap after exploration began, then its
training episodes deteriorated into repeated collisions and off-course exits.
I stopped it at 1,347 steps. Its 1,000-step actor completed three clean scored
laps at **11.326 s best** and **34.175 s total**. None of these checkpoints
beats the selected actor, so none was promoted. Their checkpoint directories
remain local under `race_ws/src/team_driver/training/runs/`.

One first attempt to score seed-54 pointed at the wrong checkpoint directory;
the driver logged that it had fallen back to the classical controller. I
stopped and excluded that run, then corrected the path and scored the actor.
The race driver now fails to start when an explicitly configured actor is
missing, and the trainer's default output directory is anchored to its own
`training/runs/` directory.

After the fail-fast change, the unchanged selected actor loaded and completed
another clean three-lap scored run (11.323 s best, 34.229 s total). The
seed-54 and seed-55 scored JSON files and this check are in
`results/submitted/`.

The speed-only run's failures after gradient updates suggest the training
environment may hold a previously published steering command while SAC
computes. The zero-action probe did not include those gradient updates, so it
does not rule out this timing problem. Further training should keep the fixed
actor controlling the car on each LiDAR scan while learning runs separately.

## Measured bottleneck

A local scored trace of the selected driver showed a peak speed of about
7.34 m/s on straights. In the three timed laps, the LiDAR safety cap lowered
the actor's requested speed in about 71% of control samples. The slowest
sections were approximately track arcs 12.5–15 m and 30–37.5 m, where the
car averaged roughly 2.7–3.2 m/s. Raising the cap globally or only in one
bend did not improve the scored lap. The next useful work is a path-following
or training-control-rate change grounded in those sections, followed by
ten-lap zero-penalty validation.

No sub-10 lap has been measured. No official submission or remote push was
performed during this search.

After removing discarded pilot configs and rebuilding the package, the
unchanged selected actor completed another three scored laps with zero
collisions or penalties (best 11.123 s). That result is saved as
`results/submitted/sub10_final_baseline_check__20260930T152308Z.json`.
