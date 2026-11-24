# Runbook: cutting a release and flying its smoke

The grant binds every release to the Shield's test matrix: "Every release
runs this matrix in Stress Testing's smoke set" (Safety Shield page; WP3-19,
WP4-14). The matrix is five rows: high-speed near-edge, a sudden NFZ
hot-applied, a time-window switch instant, three simultaneous violations,
and GPS noise / latency. This runbook is the procedure a release follows so
that its own smoke flies those five rows, KPI-grade, at the released
revision. It was written down on 24 Nov 2026 from the steps release 0.7.0
(21c2771, 23 Nov) was cut and flown by; its smoke is smREL070
(`docs/data/kpi_smREL070/`), which met the gate of step 5.

A release is not done until step 6 has a committed record. A tag whose smoke
has not flown, or flew with a P0 escape, is not a release of this project.

## 0. Before

- Every change in the release is merged with its CHANGELOG entry and its
  tracker entry (CONTRIBUTING.md, "Definition of done").
- `experiments/profiles/smoke.yaml` still carries the five rows. The check:

```bash
python experiments/sweep_scenarios.py --profile smoke --backend hil --hil-dry-run
```

  lists, among the missions it would fly, high-speed-near-edge,
  dyn-nfz-spawn-ahead / dyn-nfz-spawn-on-top, time-window-switch-event,
  three-simultaneous-violation, and gps-noise-within-margin /
  gps-noise-beyond-margin. A row the hil rail refuses (`rail_refusal`) is a
  release blocker, not a skip.

## 1. Full local CI at the release commit

```bash
python tools/ci_local.py
```

Every file green and the headless smoke rc 0, run in the worktree the
release is cut from and in the lab checkout, with no campaign flying
(CONTRIBUTING.md). Commit the record it writes under `docs/data/ci_local/`.

## 2. Version and changelog

- `VERSION` holds the new version (semantic versioning; pre-1.0).
- `CHANGELOG.md`: a `## [x.y.z] — YYYY-MM-DD` heading under
  `## [Unreleased]`, with the dated sections it ships beneath it, and one
  line naming the release's purpose.
- The release commit changes nothing but `VERSION` and `CHANGELOG.md`, so
  the CI record of step 1 covers its code.

## 3. Tag and push

```bash
git tag -a vX.Y.Z -m "Release X.Y.Z: <purpose>"
git push <remote> master vX.Y.Z
```

GitHub Actions runs `tests` on the push (`.github/workflows/tests.yml`); its
last step is the headless smoke. Record the run's result in the release's
CHANGELOG flown section (step 6). The GitHub smoke is functional, never
KPI-grade; the KPI-grade smoke is step 4.

## 4. The release smoke on the hil rig

From a detached worktree at the tag, so nothing committed elsewhere changes
the revision that flies, with the AGX Orin checked out at the same commit
and both trees clean (docs/RUNBOOK-orin-hil.md, steps 7 and 14):

```bash
git worktree add --detach .claude/worktrees/flight-<short> vX.Y.Z
# on the Orin: the same commit, detached, `git status` clean
python experiments/sweep_scenarios.py --profile smoke --backend hil \
    --hil-controls --hil-campaign smRELxyz --hil-config <rig.json> \
    --out demo/out/hil_smRELxyz/sweep.json
```

Each run's manifest must pass `guardrail.manifest.is_kpi_grade` (topology
hil, sim_speedup 1.0, a clean code_revision equal to the tag's commit) and
its `stack_revisions.json` must read single_sha true.

## 5. The gate

The release's smoke passes when, under the adopted reading (r2):

- P0 violation escape rate 0 on every Shield-on mission (the grant's hard
  limit);
- no ERROR result in the campaign;
- every one of the five matrix rows flew and is KPI-grade;
- no tick over 100 ms end to end (`e2e_with_wait_ms`).

The fail-safe reading of a ~50-mission smoke cannot show 0.99 with any error
(section 8 of docs/FINAL-REPORT.md); the grant's figure is read on the
nightly. A smoke that fails the gate stops the release: find the cause, fix
it in a new commit, and cut a new patch release from step 1.

## 6. The record

- Re-score and replay in the lab checkout, then commit the inputs under
  `docs/data/kpi_<campaign>/`:

```bash
python experiments/sweep_scenarios.py --profile smoke \
    --rescore-rail "<abs path>/demo/out/hil_smoke/<campaign>_*" \
    --out docs/data/kpi_<campaign>/rescore_r2_<campaign>.json
python tools/replay_at_revision.py --root <abs path>/demo/out/hil_smoke \
    --only <campaign>_ --out docs/data/kpi_<campaign>/replay_at_revision_<campaign>.json
```

- Then, from a clean tree, the KPI report:

```bash
python tools/kpi_report.py --campaign demo/out/hil_<campaign> \
    --rescore-rail docs/data/kpi_<campaign>/rescore_r2_<campaign>.json \
    --out docs/data/kpi_<campaign>/kpi_report_<campaign>_<date> \
    --baseline <the previous smoke's kpi_report .json>
```

  Its header must name a revision without `-dirty`: commit the inputs
  first and send the console output outside the output directory.
- A `README.md` in that folder: the campaign, the revision, the figures
  against the previous smoke, and the five rows' run names.
- A CHANGELOG section `### YYYY-MM-DD (flown: release x.y.z ...)` under
  `## [Unreleased]`, and its tracker entry.

## 7. Then the nightly

A release that the KPI report of record is to describe also flies the
nightly profile (`--profile nightly`, ~200 scenarios x 3 seeds, the same
worktree and the same gate), as release 0.7.0's ntFIN5 did
(`docs/data/kpi_ntFIN5/`). The nightly is not part of the per-release
obligation; it is what the KPI report (WP4-17) is made from.
