# OG7: remaining submission steps

Nothing has been pushed, published, or sent to the organisers by Codex.
The steps below are for Anderson to perform after his final review and decision
to publish. Deadline: **18 October 2026, 23:59 SGT**.

Official instructions:
<https://ntudeepspeed.github.io/Recruitment_Hackathon_2627/track2/07-submission.html>

## 1. Confirmed destination (completed)

Use <https://github.com/bob-the-doorknob/og7_track2>.
GitHub metadata checked on 16 October 2026 confirms it is a public fork of
`NTUDeepSpeed/Recruitment_Hackathon_2627`. Its remote branches include `track2`.
The local `team` remote now points to this fork. No new fork is needed.
Do not push directly to the upstream NTUDeepSpeed repository.

## 2. Public access (verified)

The destination is public, so collaborator invitations are not needed to read
it. After publishing the submission branch, open that branch while signed out
to confirm the actual submission is visible. If you later make it private,
arrange organiser access before submitting its URL.

## 3. Review the prepared local branch

Open **Windows PowerShell** (these are Windows paths, not WSL commands):

```powershell
$submissionRepo = 'C:/Users/Andrew/OneDrive/Documents/NTU_code/DeepSpeed/~/Recruitment_Hackathon_2627/.worktrees/task1-residual-rl-baseline'
git -c "safe.directory=$submissionRepo" -C $submissionRepo branch --show-current
git -c "safe.directory=$submissionRepo" -C $submissionRepo log -3 --oneline
git -c "safe.directory=$submissionRepo" -C $submissionRepo status --short
```

The branch must be `submission/og7-track2`. Review root `SUBMISSION.md` and
`docs/submission-review-2026-10-16.md`. The selected actor is
`residual_policy_v2_sub12_candidate.npz`; the default controls use the
18 m/s^2 profile with guarded exit preview. The model and every required
runtime asset are committed. No manual model upload is needed.

Untracked `maps.zip` and `race_ws/src/team_driver.zip` are excluded on purpose.
Do not use `git add .` or add these archives. Stop if there are unexpected
tracked modifications or if the branch is different.

## 4. Publish only after your final decision

Run these commands yourself only when ready to publish:

```powershell
$destination = 'https://github.com/bob-the-doorknob/og7_track2.git'
git -c "safe.directory=$submissionRepo" -C $submissionRepo remote set-url team $destination
git -c "safe.directory=$submissionRepo" -C $submissionRepo remote -v
```

Check the printed `team` URL carefully. Then:

```powershell
git -c "safe.directory=$submissionRepo" -C $submissionRepo push -u team submission/og7-track2
git -c "safe.directory=$submissionRepo" -C $submissionRepo rev-parse submission/og7-track2
git -c "safe.directory=$submissionRepo" -C $submissionRepo ls-remote team refs/heads/submission/og7-track2
```

The local and remote commit hashes must match. Do not force-push or overwrite
the destination's `main` or `track2` branch. On GitHub select
`submission/og7-track2` and check `SUBMISSION.md`, driver code, model weights,
map assets, and `results/submitted/` are present.

## 5. Run Judge on the actual submission

The protected workflow `.github/workflows/judge.yml` automatically runs on
pushes to `main`/`track2` and pull requests targeting those branches. It also
supports manual dispatch. **Pushing our submission branch alone does not
trigger Judge.** Do not edit the workflow to change this.

Preferred route, if manual dispatch is available in your repository:

1. Open your repository's **Actions** page. Enable workflows if GitHub asks.
2. Select **Judge**, then **Run workflow**.
3. Select `submission/og7-track2`, keep 10 scored laps and 1 run, and start it.
4. Verify the run uses the same commit hash printed in step 4.
5. Wait for checks and the evaluation to pass. Read the summary and confirm
   the race actually ran; a skipped/template race is not validation.

If Judge or its Run workflow button is unavailable:

1. Open a pull request **within your own submission repository**, from
   `submission/og7-track2` into its official `track2` branch. If your assigned
   repository instead keeps this Judge workflow on `main`, target `main`.
2. GitHub may default the base repository to upstream NTUDeepSpeed. Change it
   to your own repository before creating the pull request.
3. Enable/approve workflow execution if requested, then check the Judge run
   on the pull request. Keep the submission branch as the branch you report;
   merging the pull request is not needed for the submission instructions.

If Judge fails, retain its logs and fix the stated error before reporting
completion. A Docs workflow failure and a Judge failure are different checks.
If the error is permissions, billing, or unavailable workflow controls, ask
the repository owner or organisers; do not modify protected judging files.

## 6. Notify the organisers

After repository access and Judge are confirmed, send through the organisers'
specified submission channel. The linked page requires the repository URL and
branch but does not give a dedicated submission form. If no channel was
provided, ask the organisers; the site lists `ntu-deepspeed@e.ntu.edu.sg` as
its question contact.

Copy and fill this message yourself:

```text
Team: OG7
Track: Track 2 (AutoDRIVE Simulator)
Leader: Anderson Andrew Chandra
Email: Achandra001@e.ntu.edu.sg
Telegram: @Andsonc
Repository: <accepted repository URL>
Branch: submission/og7-track2
Commit: <verified remote commit hash>
Judge workflow: <successful run URL>
Private repository access: <confirmed / public repository>

Our submission record is SUBMISSION.md at the root of the branch.
The packaged model, supporting runtime assets, and original local referee
results are included. Please confirm receipt and repository access.
```

Keep the sent message and acknowledgement. Do not report a successful
submission until the URL/branch have actually been sent.

## 7. Prepare for the interview

Each member should be able to explain the controller, speed planning, residual
actor/reward, safety checks, failed experiments, and observed limitations.
Use `reproduce-selected-driver.md` and `corner-exit-2026-10-16.md` to prepare.
The organisers' run determines the score; the local and GitHub runs do not.
