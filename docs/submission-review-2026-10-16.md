# OG7 Track 2 submission review

Status: prepared locally; publication and submission require Anderson's final approval.

## Destination

- Repository: <https://github.com/bob-the-doorknob/DeepSpeedRobot>
- Remote name: `team`
- Prepared branch: `submission/og7-track2`
- Live remote heads checked on 16 October 2026: `main` and `codex/fast-racing`.
  The prepared submission branch is new and has not been pushed.
- Frozen implementation commit: `8ac1d0f3335a6d90aca21679947ded24501b69ba`.

## Selected model and controls

- Actor: `race_ws/src/team_driver/weights/residual_policy_v2_sub12_candidate.npz`
- Actor SHA-256: `e76d8b8baf50bf452c59fc00002fd6efda7aad0c21206302630a77654e9bbe22`
- Both bare driver defaults and the default YAML select the same controller.
- Planning lateral allowance: 18 m/s^2; requested top speed: 7.2 m/s.
- Guarded exit preview enabled; curvature threshold: 0.30 per m.
- Explicit equivalent: `fast_corner_18_residual_v2_exit_preview_params.yaml`.
- Two earlier full runs: 111.0617 s and 111.2049 s, zero collisions over
  twenty timed laps. Faster single-lap experimental variants were rejected
  because of tracking excursions or incomplete runs.

This is the selected stable configuration from the latest comparisons;
it is not a claim of the fastest possible single lap.

## Contents and checks

- All five roster entries are complete in `SUBMISSION.md`.
- Complete driver package, runtime actor, map, racing line, vehicle limits,
  offline producer code, and original referee result files are included.
- No additional runtime pip/apt dependencies were declared.
- No simulator binary, race workspace build/install/log directory, or ZIP
  archive is tracked in the prepared commit.
- Existing untracked `maps.zip` and `race_ws/src/team_driver.zip` are excluded.
- Moved our offline racing-line generator out of protected `scripts/` into
  `race_ws/src/team_driver/training/`; the official map reader is unchanged.
- Protected paths have no diff against the official `origin/track2` revision.

## Clean-clone validation

The frozen implementation commit was exported through a local Git bundle and
cloned into `/tmp/og7-submission-20261016-v1` inside the existing ROS container.
No existing race workspace overlay was sourced. The stock ROS and `/sim_ws`
devkit overlays supplied the base dependencies.

- Official `scripts/verify_judging_env.sh`: all 64 protected files match;
  no unexpected protected files.
- Full race workspace `colcon build --symlink-install`: both packages built.
- Driver tests: 63 passed. Test command from `race_ws/`:
  `PYTHONPATH="$PWD/src/team_driver${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest -q src/team_driver/test`
- Eight installed runtime assets match the clean-clone source bytes; actor
  hash matches the selected model.
- Referee and driver package prefixes resolve to the clean clone.
- Clean-clone simulator run: **COMPLETE**, scored, ten of ten timed laps,
  zero collisions and zero penalty seconds; best lap **11.0478 s**,
  ten-lap total **111.0947 s**, real-time factor **0.543**.
- Original result: `results/submitted/OG7__cleanclone_20261016_a.json`.
  SHA-256: `ad21db1b4831c76172b60de325b16eae44fab5e568bb25a6b2a79f6f11247206`.
  The submitted copy is byte-identical to the referee output.

The implementation commit above is the one tested. The following local commit
adds only this review record, the unedited clean-clone result, and updated
submission documentation; no driver or configuration changes follow that test.

The simulator check uses the native Windows simulator and the existing
unmodified bridge. It launches the clean clone's official referee and bare
driver with no driver parameter overrides, ten timed laps and two unscored laps.
This checks a clean source/build and installed assets in the existing image;
it does not claim a new Docker image or headless Linux simulator test.

## Approval and remaining external steps

Review `SUBMISSION.md`, this record, the final local commit, and the selected
model before approving publication. No push, organiser message, or submission
has been performed.

After Anderson explicitly approves:

1. Push `submission/og7-track2` to the configured `team` remote.
2. Check the GitHub Judge workflow on that exact branch and resolve failures.
3. Give the organisers the repository URL and branch before
   **18 October 2026, 23:59 SGT**.

The organisers' evaluation determines the score. Local and GitHub results
establish reproducibility and execution, not a guaranteed judging time.
