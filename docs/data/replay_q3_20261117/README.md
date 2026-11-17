# Q3 runs: today's Shield re-run on their logged inputs (WP4-11), 17 Nov 2026

`summary.json`, `per_run.jsonl.gz` (one record per run in the window) and
`table.md` are the output of the command below. It was run on 17 Nov 2026 on
the desktop (76.3 s), with the lab checkout's `demo/out` as the data and the
tool at commit d9175e4 of branch wf17-replay. That tree was clean:
`code_revision_now` is `d9175e44aca7`, with no `-dirty` flag. Its Shield code
(`guardrail/`) is master 47b3fde's, which the branch does not change. The
next commit on the branch changes only a sentence of the tool's docstring.

    python tools/replay_q3.py --root demo/out --out docs/data/replay_q3_20261117

## What this is, and what it is not

The tool takes every stored run whose manifest.json is dated 1 Aug - 31 Oct
2026 (the file's own date; the manifest holds no date field). For each one,
it rebuilds the Safety Shield the run flew from what the run stored:

- every policy generation (policy_g<N>.json). A hot-apply is installed
  before the first tick that flew it, through the steps today's
  `Shield.hot_apply` accepts at that point (a new or edited moving zone is
  anchored at the event time). The installed generation's hash is checked
  against the stored one.
- the forecast metrics.json recorded.
- the rule clock's reading per tick, where the run logged one.

Then every logged raw action goes through **today's** Shield again, on the
logged state (trajectory.json's own doubles). For each tick it compares the
rule ids violated, the repair operators, touched, braked and the emitted
action, bit for bit. It is open loop: the path is the flown one.

**This is not the grant's reproducibility replay.** The grant's determinism
contract (03-simulation/data-flow.md, six fields with code_revision first;
layer-requirements.md L5, "a single git SHA + seed + policy hash") defines a
replay at the revision the run was flown at. This tool runs today's Shield
code (d9175e4 / master 47b3fde) on old inputs, so it is a regression and
coverage check. **None of the 15 replayed runs was flown from a clean
revision**: 14 name `e5b2e51492ec-dirty` (9) or `401305abd019-dirty` (5),
and 1 is `unversioned` (`replayed_flown_at_a_clean_revision: 0`). The 3 runs
refused below for a hot-apply are no different (2 `unversioned`, 1
`401305abd019-dirty`). None of them can be
checked out and replayed at the code it flew, and the contract keeps such a
run out of the KPI report. A mode that replays each run in a git worktree at
its clean code_revision is not built; on this data it would have nothing to
replay.

## Result

5,509 run folders in the window. 15 could be re-run: **10 identical under
today's Shield code** (every tick, emitted bit for bit) and **5 differ**. The
other 5,494 are `not_replayable`, each with its reason. Every replayed row
logs touched and braked, so no field was left out of the comparison
(`ticks_field_not_logged` is empty on all 15).

| | runs |
|---|---|
| re-run, identical under today's Shield (every tick, emitted bit for bit) | 10 |
| re-run, differs | 5 |
| not replayable: a headless-kinematic episode | 5,195 |
| not replayable: a map rule, and the map flown is not recorded | 213 |
| not replayable: the Shield's forecast is not recorded | 82 |
| not replayable: a generation today's Shield.hot_apply refuses where the run installed it | 3 |
| not replayable: no flight log | 1 |

Eight of the identical runs are eight of the nine ROS 2 rail runs flown on
29 Oct at e5b2e51-dirty (`ros2_action_*`, `ros2_hotapply_01`/`_03`; up to five
generations, up to 889 touched ticks). The other two are Shield-off arms of
7 Oct (`ros2fix2_off_fence` at 401305a-dirty, `ros2fix_off_fence`
unversioned), where only raw and the violations are compared.

The five that differ (`table.md` has the first difference of each):

- `ros2_hotapply_02` (29 Oct): one tick of 346 differs, in the last bit of
  one emitted float (8.9e-16 m/s); every decision identical.
- `ros2fix_on_policy55` (7 Oct): every decision identical; on 13 ticks the
  emitted action differs by up to 2.0 m/s. On the first tick, the logged
  GeofenceEscape exits the zone at 2.0 m/s (emitted vy -2.0); today's
  emitted escape is -4.0 m/s, the policy's 4.0 m/s speed cap.
- `ros2fix2_on_fsm` and `_hot_apply_not_in_lock/ros2fix_on_dyn_nfz` (7 Oct;
  the second hot-applies a dynamic_nfz, which today's hot_apply accepts):
  the repair chain differs on 14 and 17 ticks. The log has SpeedClamp,
  ClimbClamp, GeofenceSlide; today's chain adds AltitudeFix and a second
  ClimbClamp. The emitted actions agree.
- `ros2fix2_on_proxy` (7 Oct): on one tick the log names SpeedClamp twice
  and today's chain once; on two ticks an emitted float differs in its last
  bit (8.9e-16).

All five ran on today's code, not their own revision, and none was flown at
a clean commit. A difference may come from a code change since the flight,
from the flight itself, or from an input the log does not carry. This table
does not tell them apart, and it does not attribute any of them to a commit.

## Why the rest are not replayable

- **Headless-kinematic episodes (5,195; 3,461 without a flight log, 1,734
  with one).** The Shield's configuration is the scenario cell's (forecast,
  wind, response arm), which the episode does not store. They are re-flown
  from the scenario library by `tools/rerun_episode.py`. A sample of 12 of
  the 1,734 run through it gave 10 `cannot_compare`, because the library's
  policy now hashes differently from the one flown (e.g. sim_demo_policy
  8a311f9d... then, b665adba... now), and 2 that cannot be rebuilt (the cell
  is no longer in the library's expansion). So none of them re-flies today
  without the policy and the library as they were.
- **A map rule (213: the CityLife follow, VLA and PAS runs - aiot_*, ft_*,
  vla_*, orin_*, cmp*_*, ntut_*, pas_* and others).** Their policy has an
  obstacle_clearance rule over the CityLife occupancy map. 200 of them name
  the map file they loaded (metrics.json `params.citymap`); the 13 PAS runs
  name none. No run records a digest of the map's content, and the maps
  were rebuilt on 4 Nov (the 24 Sep build is kept beside them). Replaying
  them on a map chosen today would be a guess.
- **The forecast not recorded (82).** The August and September follow and
  demo rails, and the early-October ROS 2 / SITL rail runs (`ros2if_*`,
  `sitl_if_*`, `ros2_shield_*`, `sitl_shield_*`, `*_ped_*`), wrote no
  `shield_lookahead` into metrics.json.
- **A generation today's hot_apply refuses (3, all 7 Oct):**
  `_hot_apply_not_in_lock/ros2fix_on_unsafe_start` (generation 1 adds
  polygon_fence `nfz-on-aircraft`), `ros2fix_on_dyn_yaw` and
  `sitlfix_on_dyn` (generation 1 adds polygon_fence `nfz-dynamic`). Each
  added the fence after the first tick, which the rails did before the
  mission-start lock. Today's `Shield.hot_apply` refuses that with
  `LockedRuleClass`, so today's Shield cannot reach the state these runs
  flew. Until this fix, the tool forced the generation in through
  `Shield._commit` and reported `ros2fix_on_unsafe_start` as differing (a
  missing `nfz-on-aircraft` violation from tick 80) and the other two as
  re-run (`sitlfix_on_dyn` identical, `ros2fix_on_dyn_yaw` differing on
  repairs and a last-bit float). That was a Shield state no code path of
  today's Shield reaches.
- **No flight log (1):** `ntut_dagger_*` holds no flight_log.jsonl.

No run in the window was refused for the pipe (a row with `shield_velocity`
or `fix_ok` false). The tool now refuses such a run, because the setpoints
the rail sent (Shield.note_flown, on a real-time pipe clock) are not logged
per tick.

## Limits

- Open loop: a tick after a difference is judged where the flown aircraft
  was.
- Rail calls between ticks that may touch the Shield's state (the hold
  planner's queries, the FSM's legality checks) are not replayed.
- The selection is by the manifest file's date on this disk.
- This is 15 of the 5,509 Q3 run folders, re-run on today's code. It is not
  the full KPI matrix replayed for reproducibility that the grant's final
  delivery names.
