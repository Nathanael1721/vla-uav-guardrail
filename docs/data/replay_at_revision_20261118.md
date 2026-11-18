# Episodes replayed at the revision they flew (WP4-11), 18 Nov 2026

`replay_at_revision_20261118.json` (the summary) and
`replay_at_revision_20261118.per_run.jsonl.gz` (one record per episode) are
the output of `tools/replay_at_revision.py`, run on 18 Nov 2026 on the
desktop (Windows, Python 3.10.20, CPU only, no rig), with the lab checkout's
`demo/out` as the data and the tool at its own commit (`tool_revision` in
the summary, clean):

    python tools/replay_at_revision.py --root <lab checkout>/demo/out \
        --window 2026-08-01:2026-10-31 --topology hil --jobs 12 \
        --scratch <scratch> --out docs/data/replay_at_revision_20261118.json

The selection is every run folder with a manifest.json dated 1 Aug - 31 Oct
2026 (Q3, the file's own date) plus every run whose manifest says topology
`hil` (the hil campaigns, 6-17 Nov): 7,377 episodes.

## What was done

The grant's determinism contract (03-simulation/data-flow.md) makes an
episode reproducible "from six fields", `code_revision` first. For each
episode the tool read that field, refused it when no checkout can reproduce
it, and otherwise replayed the episode **with the code of that commit**: one
temporary detached `git worktree` per revision (51 revisions), the replay run
in an isolated interpreter whose imports come from that worktree only, the
worktree removed and pruned afterwards (`worktrees`, all `removed: true`;
`worktrees_left_behind: []`; nothing was linked into them).

- **refly** (headless-kinematic episodes): re-flown closed loop by that
  revision's own `tools/rerun_episode.py` from the bundle's
  manifest_extras.json. Compared: harness_events.jsonl event by event (except
  `policy_source.path`, the checkout the policy was read from), the whole
  kpi.json `kpi` table re-scored by that revision's `score()`, the policy
  hash at t = 0, and shield_audit.jsonl / flight_log.jsonl where the bundle
  stored them (the Q3 bundles below stored neither).
- **open_loop** (flown rail episodes): that revision's Shield run on the
  inputs the episode logged, tick by tick. Compared: every
  shield_audit.jsonl record as JSON text except `ts` (wall time) and the
  `fsm_*` fields (the node's own escalation FSM, which steps on the
  autopilot's mode and is not re-derived open loop), plus per flight-log row
  the raw action, rule ids, repair operators, touched, braked and the four
  emitted floats.

## Result

| | Q3 window | hil campaigns |
|---|---:|---:|
| episodes | 5,509 | 1,868 |
| refused: flown from a `-dirty` tree | 4,724 | 0 |
| refused: flown from an `unversioned` tree | 20 | 0 |
| replayed at the flown revision, identical | **720** | **972** |
| replayed, differs: body-frame turn only, last digits | 0 | 513 |
| replayed, differs: floats only, last digits | 0 | 283 |
| replayed, differs: decision | 0 | 3 |
| not replayable (an input the episode does not hold) | 45 | 97 |

**Q3.** 720 of the 765 Q3 episodes flown from a clean commit were re-flown
at that commit and are identical: the 360 headless episodes of `sweep/`
flown at fb8f87c8ce18 (22 Oct) and the 360 of `sweep_cap_20261029/` at
d9ba81045c95 (29 Oct). For these the comparison is the harness events and
the KPI table, because those bundles kept no flight log and predate the
audit in bundles (29 Oct). The other 45 are flown rail runs that cannot be
replayed from what they stored (26 store no policy IR, 10 flew at an August
revision without the replay pieces, 8 read an unlogged velocity / pipe, 1 a
forecast configuration the replay does not rebuild). The remaining 4,744 Q3
runs - most of the Q3 work - were flown from uncommitted trees and **cannot
be replayed at the code that flew them**; they are refused, not approximated.

**hil.** 1,771 of the 1,868 hil episodes were replayed at their revision:
563,412 audit records and 1,284,939 ticks compared. 972 are identical record
for record and tick for tick; 1,517 have no flight-log difference at all. Of
the 799 that differ:

- 513 differ only in the audit's body-frame actions (`raw_action`,
  `emitted_action`) by at most 3.6e-15: the world-frame actions and the
  heading compared equal, the code is the same, and the writer turns the
  world action into the body frame with cos / sin of the heading. Every one
  of these episodes flew on the AGX Orin (aarch64); the replay ran on this
  x86-64 desktop.
- 283 differ in floats only, by at most 3.7e-14 (m/s or m), with every rule
  id, operator, verdict, string and touched / braked equal.
- 3 differ in a decision: hs05_004 and hs05_005 (9eed35ba09ec) and
  nt1111_302 (22d434bf04cf), all `dyn-nfz-rotate-spin`, a zone rotating
  continuously. Floats differ by ~1e-14 as above, and on one tick of hs05_004
  and hs05_005 the repair operators differ (nt1111_302: the repair attempt
  records differ beyond numbers). The cause is not established here.

The tool calls the two float classes `differs`, never `identical`: the
contract is bit for bit, and these episodes are not, on another host. That
the last-digit differences come from the hosts' arithmetic (aarch64 against
x86-64 libm) is the reading the data supports - same code, same inputs,
differences at the scale of one or a few ulps - not a proof; re-running the
replay on the Orin would settle it (rig item).

97 hil episodes are not replayable: 65 lost the GPS fix in flight (the
Shield then dead-reckons on what the rail sent, a pipe no rail logs per
tick), 26 flew the own-velocity forecast (the same pipe), and 6 are the
harness rail-session bundles of hh02, which did not carry metrics.json (the
Shield's forecast); since this branch a rail-session bundle carries
metrics.json, trajectory.json and the run's signed replay tarball.

ntFIN2 (116214f, 583 runs): 276 identical, 190 body-frame last digits, 96
floats-only last digits, 0 decision, 21 not replayable (GPS dropout).

## What this is not

An open-loop replay re-derives the Shield's half of a flown episode at its
revision; it does not re-fly the autopilot and simulator, which needs the
rig. The refly replays re-fly the whole headless episode. None of this makes
the 4,744 Q3 runs flown from dirty trees reproducible.
