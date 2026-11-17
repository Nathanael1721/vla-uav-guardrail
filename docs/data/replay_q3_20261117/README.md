# Q3 runs replayed for reproducibility (WP4-11), 17 Nov 2026

`summary.json`, `per_run.jsonl.gz` (one record per run in the window) and
`table.md` are the output of, run on 17 Nov 2026 from the lab checkout on the
desktop (72.6 s; the Shield code is master 47b3fde's, which this branch does
not change):

    python tools/replay_q3.py --root demo/out --out docs/data/replay_q3_20261117

What it is: for every stored run whose manifest.json is dated 1 Aug - 31 Oct
2026 (the file's own date; the manifest holds no date field), the Safety
Shield the run flew is rebuilt from what the run stored - every policy
generation (policy_g<N>.json, a hot-apply installed before the first tick
that flew it, its hash checked against the stored one), the forecast
metrics.json recorded, the rule clock's reading per tick where the run logged
one - and every logged raw action is put through it again on the logged
state (trajectory.json's own doubles). Per tick it compares the rule ids
violated, the repair operators, touched, braked and the emitted action, bit
for bit. It is open loop (the path is the flown one) and it runs today's
Shield code, not the revision the run was flown at.

## Result

5,509 run folders in the window. 18 could be replayed: **11 identical, bit
for bit, 7 differ**. The other 5,491 are `not_replayable`, each with its
reason.

| | runs |
|---|---|
| replayed, identical (every tick, emitted bit for bit) | 11 |
| replayed, differs | 7 |
| not replayable: a headless-kinematic episode | 5,195 |
| not replayable: a map rule, and the map flown is not recorded | 213 |
| not replayable: the Shield's forecast is not recorded | 82 |
| not replayable: no flight log | 1 |

Eight of the identical runs are eight of the nine ROS 2 rail runs flown on
29 Oct at e5b2e51 (`ros2_action_*`, `ros2_hotapply_01`/`_03`; up to five
generations, up to 889 touched ticks); the other three are of 7 Oct (`ros2fix2_off_fence`,
`ros2fix_off_fence`: Shield-off arms, only the violations compared;
`sitlfix_on_dyn`, two generations).

The seven that differ (`table.md` has the first difference of each):

- `ros2_hotapply_02` (29 Oct): one tick of 346 differs, in the last bit of
  one emitted float (8.9e-16 m/s); every decision identical.
- `ros2fix_on_policy55` (7 Oct): every decision identical; on 13 ticks the
  emitted action differs by up to 2.0 m/s. On the first, the logged
  GeofenceEscape exits the zone at 2.0 m/s (emitted vy -2.0); today's
  emitted escape is -4.0 m/s, the policy's 4.0 m/s speed cap.
- `ros2fix2_on_fsm`, `_hot_apply_not_in_lock/ros2fix_on_dyn_nfz` (7 Oct):
  the repair chain differs (logged SpeedClamp, ClimbClamp, GeofenceSlide;
  today also AltitudeFix and a second ClimbClamp) on 14 and 17 ticks; the
  emitted actions agree.
- `ros2fix2_on_proxy`, `ros2fix_on_dyn_yaw` (7 Oct): one and two ticks where
  the log names SpeedClamp twice and today's chain once; on two ticks of
  each an emitted float differs in its last bit (8.9e-16).
- `_hot_apply_not_in_lock/ros2fix_on_unsafe_start` (7 Oct): from tick 80 the
  log names a violation of `nfz-on-aircraft` that today's Shield does not
  raise; 35 ticks' repairs and emitted actions differ (up to 6.0 m/s).

Six of the seven were flown on 7 Oct at 401305a (or from an unversioned
tree): their differences are what today's Shield decides on the same inputs,
which is the change a reproducibility replay exists to show; this table does
not attribute them to a commit. The runs flown on 29 Oct reproduce, one with
a last-bit difference.

## Why the rest are not replayable

- **Headless-kinematic episodes (5,195; 3,461 without a flight log, 1,734
  with one).** The Shield's configuration is the scenario cell's (forecast,
  wind, response arm), which the episode does not store; they are re-flown
  from the scenario library by `tools/rerun_episode.py`. A sample of 12 of
  the 1,734 run through it: 10 `cannot_compare` (the library's policy now
  hashes differently from the one flown, e.g. sim_demo_policy 8a311f9d...
  then, b665adba... now), 2 cannot be rebuilt (the cell is no longer in the
  library's expansion). So none of them re-flies today without the policy
  and the library as they were.
- **A map rule (213: the CityLife follow, VLA and PAS runs - aiot_*, ft_*,
  vla_*, orin_*, cmp*_*, ntut_*, pas_* and others).** Their policy has an
  obstacle_clearance rule over the CityLife occupancy map. 200 of them name
  the map file they loaded (metrics.json `params.citymap`), the 13 PAS runs
  none, and no run records a digest of its content; the maps were rebuilt
  on 4 Nov (the 24 Sep build is kept beside them). Replaying them on a map
  chosen today would be a guess.
- **The forecast not recorded (82).** August and September follow and demo
  rails, and the early-October ROS 2 / SITL rail runs (`ros2if_*`,
  `sitl_if_*`, `ros2_shield_*`, `sitl_shield_*`, `*_ped_*`) wrote no
  `shield_lookahead` into metrics.json.
- **No flight log (1):** `ntut_dagger_*` holds no flight_log.jsonl.

## Limits

Open loop: a tick after a difference is judged where the flown aircraft was.
Rail calls between ticks that may touch the Shield's state (the hold
planner's queries, the FSM's legality checks) are not replayed. The selection
is by the manifest file's date on this disk.
