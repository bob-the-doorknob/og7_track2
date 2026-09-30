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
episodes even during its zero-adjustment phase. The race driver updates on
each LiDAR scan, while the training loop holds an action while the learner
computes between steps. That control-rate mismatch is a plausible cause, not
yet a proven root cause. Further stacked training should first keep the fixed
base policy publishing at the race control rate and let learning update only
the second-stage action.

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
