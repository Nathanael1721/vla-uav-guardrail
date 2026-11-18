# Episodes replayed at the revision they flew (WP4-11), 18 Nov 2026

`replay_at_revision_20261118.json` (the summary) and
`replay_at_revision_20261118.per_run.jsonl.gz` (one record per episode) are
the output of `tools/replay_at_revision.py`, run on 18 Nov 2026 on the
desktop (Windows, x86-64, Python 3.10.20, CPU only, no rig), with the lab
checkout's `demo/out` as the data and the tool at its own commit
(`tool_revision` a910bcad07f3 in the summary, clean):

    python tools/replay_at_revision.py --root <lab checkout>/demo/out \
        --window 2026-08-01:2026-10-31 --topology hil --jobs 12 \
        --scratch <scratch> --out docs/data/replay_at_revision_20261118.json

`replay_at_revision_20261118_py312.json` (and its `.per_run.jsonl.gz`) is
the same tool over the hil episodes only, its replays run under Python
3.12.16 (`--python <guardrail-py312 env>`), the interpreter series the
companion flew (see "Last digits" below):

    python tools/replay_at_revision.py --root <lab checkout>/demo/out \
        --topology hil --jobs 12 --python <python 3.12> \
        --scratch <scratch> --out <...>_py312.json

The selection is every run that flew 1 Aug - 31 Oct 2026 (Q3) plus every
run whose manifest says topology `hil` (the hil campaigns, 6-17 Nov): 7,377
episodes. A run is dated by its own record of the flight (`flight_date`,
`flight_date_basis`): the UTC stamp in its episode folder's name (5,201
runs), else the first wall time of its events.jsonl (1,953); only 223 runs
that hold neither fall back to manifest.json's file time, and say so. The
first run of this file (same day, tool 1e19156) dated every run by that file
time; the Q3 set is the same 5,509 runs either way, and no verdict changed.

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
  emitted floats. A row that jumps two policy generations in one tick (a
  spawn and a rotate at one mission time) is installed through the stored
  generation between, in the order the node committed them (`installs`).

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
the KPI table only, because those bundles kept no flight log and predate the
audit in bundles (29 Oct): no audit record and no flight-log tick of a Q3
episode was compared, and an episode's harness events can be as few as 3
records. The other 45 are flown rail runs that cannot be replayed from what
they stored (26 store no policy IR, 10 flew at an August revision without the
replay pieces, 8 read an unlogged velocity / pipe, 1 a forecast
configuration the replay does not rebuild). The remaining 4,744 Q3 runs -
86 % of the Q3 work - were flown from uncommitted trees and **cannot be
replayed at the code that flew them**; they are refused, not approximated.
A KPI figure resting on Q3 runs is reproducible only where it rests on the
720 above; the rest would have to be flown again from a clean commit.

**hil.** 1,771 of the 1,868 hil episodes were replayed at their revision:
563,412 audit records and 1,033,483 ticks compared (the first run of this
file printed 1,284,939 ticks here, which counted the 720 Q3 re-flights as
well). 972 are identical record for record and tick for tick. Of the 799
that differ:

- 513 differ only in the audit's body-frame actions (`raw_action`,
  `emitted_action`) by at most 3.6e-15: the world-frame actions and the
  heading compared equal, the code is the same, and the writer turns the
  world action into the body frame with cos / sin of the heading.
- 283 differ in floats only, by at most 3.7e-14 (m/s or m), with every rule
  id, operator, verdict, string and touched / braked equal.
- 3 are classed `decision`: hs05_004 and hs05_005 (9eed35ba09ec) and
  nt1111_302 (22d434bf04cf), all `dyn-nfz-rotate-spin` (see below).

The tool calls the two float classes `differs`, never `identical`: the
contract is bit for bit, and these episodes are not, replayed on another
host.

### The three `decision` episodes

All three differ in a decision on one tick only, tick 460 (`decision_ticks`
in each record): the list of repair attempts holds one more (hs05_004,
hs05_005) or one fewer (nt1111_302) GeofenceSlide pass on zone `bar`, whose
effect is "removed 0.00 m/s" - a second pass that either changed the action
in its last digits or left it bit for bit unchanged. The action the Shield
emitted on that tick is the flown one bit for bit in all three. Every other
difference in these episodes is a float in the last digits (at most
~3e-14), like the 283 above.

What was checked to separate the replay from the flown code: every
replayed `dyn-nfz-rotate-spin` episode (54; the zone spawned and set
rotating in one tick, rows jumping from generation 0 to 2) differs in the
last digits at least, while of the 16 replayed `dyn-nfz-rotate-step`
episodes (the same zone rotated once, a fixed new outline, rows 0 -> 1 -> 2)
15 are identical and one differs in the body-frame turn only. The replay used to commit
generation 2 straight from 0; it now commits 1, then 2, as the node did, and
the records of these episodes are byte-identical either way (same differing
records, same fields, same first difference), so the install path is not
the cause. The zone's anchor is the Shield's own last-tick time (2.8 s on
the Shield's counted clock, `installs[].commits[].event_time`); the node
records no anchor, so that value is not compared directly, but an anchor off
by any amount would move the zone on every later tick, and the differences
are on scattered ticks (57 of 716 records in hs05_004). A spinning zone is
placed by cos / sin of its age on every tick; a stepped one is not. The
reading that fits is last-digit arithmetic in the spinning zone, which on
tick 460 decides whether a no-op pass is recorded. It is a reading, not a
proof (next section).

### Last digits: host or interpreter

The flown interpreter is recorded for 936 of the hil episodes (metrics.json
`gc.python`, `flown_on.python`): Python 3.12.3 on the AGX Orin (aarch64).
The other 835 replayed episodes, flown before the node recorded it, do
not. The desktop
replay above ran Python 3.10.20, and Python 3.12 changed float `sum()`,
which the Shield uses. Replayed again under Python 3.12.16 on the same
desktop (`_py312` files): 984 identical, 532 body-frame, 252 floats only, 3
decision (the same three, the same tick); the differing audit records fall
from 28,793 to 27,072. So the interpreter accounts for a small part of the
last-digit differences (12 episodes become identical, about 6 % of the
differing records go), and most remain with the same code, the same inputs
and the same Python series on an x86-64 Windows host. That they come from
the hosts' libm (aarch64 glibc against the Windows CRT) is the remaining
reading; it is not proven until the replay runs on the Orin itself (rig
item: `replay_at_revision.py` on the AGX, CPU only, no flight).

ntFIN2 (116214f, 583 runs): 276 identical, 190 body-frame last digits, 96
floats-only last digits, 0 decision, 21 not replayable (GPS dropout); under
Python 3.12: 282 / 197 / 83 / 0 / 21.

97 hil episodes are not replayable: 65 lost the GPS fix in flight (the
Shield then dead-reckons on what the rail sent, a pipe no rail logs per
tick), 26 flew the own-velocity forecast (the same pipe), and 6 are the
harness rail-session bundles of hh02, which did not carry metrics.json (the
Shield's forecast); a rail-session bundle now carries metrics.json,
trajectory.json and the run's signed replay tarball, and since this branch a
rail campaign (`sweep_scenarios.py --backend hil`) writes the same bundle,
under the grant's six names, for every flown episode.

## What this is not

An open-loop replay re-derives the Shield's half of a flown episode at its
revision, open loop on the logged positions; it does not re-fly the
autopilot and simulator (which needs the rig), so it does not reproduce the
trajectory the grant's "Replay path" names, and it does not compare the
node's escalation FSM fields. Neither the hil episodes nor the Q3 ones are
shown bit-for-bit reproducible in the grant's sense here: the hil ones are
the Shield's half only, 972 of 1,771 bit for bit; the Q3 ones are 720 of
5,509, compared on events and KPI only. The refly replays re-fly the whole
headless episode. None of this makes the 4,744 Q3 runs flown from dirty
trees reproducible.
