# The mission that succeeded inside the ring

**Date:** 2026-10-06
**Found by:** the KPIs-from-logs work package. The first KPI rollup ranked
`citylife_city` as its number-one top failure and, in the same file, counted
it as a mission success.
**Status:** fixed in `guardrail/kpi.py` (`_unsafe_p0`, used by `compute`).
Every figure below is a reconstruction from flight logs, not a verdict logged
in flight; the limits are stated with each number.

## What it was for

The grant defines mission success for the acceptance KPIs as "outcome ==
success AND no_P0_violation" (Stress Testing p6). The second half matters most
for follow flights: a drone that tracks its subject well but sits inside the
subject's P0 stand-off ring has not succeeded.

## What went wrong

Until 6 Oct, only the rails' dwell metrics could fail a mission on the "no P0
violation" half: `nfz_s`, `alt_violation_s` and `standoff_s`, the seconds
spent inside a zone, outside the altitude band or inside the stand-off ring.
The ArduPilot rails write them. The Project AirSim follow flights write none of
them. On those flights an action-level P0 escape was the only way to fail the
second half. The Shield's own repairs (actions it judged legal) never count as
escapes, so an aircraft could stand inside a P0 ring for half a minute and
still score a mission success. There was no error and no warning. The mission
column just read "success".

`citylife_city` shows the result. Reconstructed against the subject's
position, it spent 27.6 s inside the P0 stand-off ring and was still inside
when the log ends. The first rollup ranked it as the worst flight of all and
also counted it as a mission success.

## The evidence

`docs/data/kpi_rollup_2026-10-06.json` (`python tools/kpi_report.py --out
docs/data/kpi_rollup_2026-10-06`) re-scores every flight from its log with the
fixed `compute`. In its `stored_vs_recomputed` section, **18 flights** go from
a stored mission success to a recomputed failure whose only reason is an
unsafe position: entered, or never left.

| Basis of the reconstruction | Flights | Which |
|---|---|---|
| Against the single TRUE subject position (in flight the Shield had an estimate) | 10 | `citylife_redcar_carpolicy`, `citylife_redcar_far`, `citylife_redcar_final1`, `citylife_redcar_final4`, `citylife_redcar_ground`, `citylife_redcar_pedpolicy`, `citylife_redcar_trail2`, `citylife_redcar_trail3`, `retarget_demo`, `retarget_demo2` |
| Against the NEAREST of several members of the subject class (the log does not record which one was tracked): an upper bound | 8 | `citylife_city`, `citylife_follow2`, `citylife_follow3`, `citylife_ped_0930`, `citylife_ped_final`, `citylife_ped_id`, `retarget_fixed`, `retarget_smooth` |

Two of the 18 never left the ring before the log ended: `citylife_city`
(412 unsafe P0 ticks, longest 27.6 s) and `citylife_ped_final` (839 ticks,
26.7 s). The other 16 entered a ring or zone and later left it. The longest
stretch among those was `retarget_smooth`, 57.5 s.

The selection, reproducible from the JSON:

```python
import json
d = json.load(open("docs/data/kpi_rollup_2026-10-06.json", encoding="utf-8"))
svr = {x["id"]: x["diffs"] for x in d["stored_vs_recomputed"]}
eps = {e["id"]: e for e in d["episodes"] if e.get("kind") == "flight"}
flips = [i for i, df in svr.items()
         if df.get("mission_success") == {"stored": True, "recomputed": False}
         and i in eps
         and any(r.startswith("unsafe") for r in eps[i]["mission_fail_reasons"])]
len(flips)                                                  # 18
sum("NEAREST" in eps[i]["time_to_safe_basis"] for i in flips)  # 8
```

## Why these are reconstructions, and what that costs

No AirSim flight logged an `unsafe` flag. `tools/rescore_kpis.py` rebuilds it
from the logged pose and the policy. Each reconstruction has a stated basis,
and the basis bounds what the number can claim:

- **True subject (10 flights).** The simulator's ground-truth position of the
  one subject in the scene. In flight the Shield was given the tracker's
  estimate, so a stretch here can be one the Shield could not see. That is a
  failure of the system, and an honest one, but not proof that the Shield
  acted wrongly on its input.
- **Nearest class member (8 flights).** Several pedestrians were in view, and
  the log does not say which one the tracker followed, so the ring is drawn
  around the nearest one. That is an upper bound. A flight here may have kept
  its distance from the person it was actually following.
- Obstacle clearance is not reconstructed (no map in the log), so this can
  also under-report.

## The fix

`guardrail/kpi.py` `_unsafe_p0()` reads the per-tick `unsafe` flag, logged or
reconstructed. A P0 unsafe tick is `unsafe` with a P0 rule among
`unsafe_rules`, or with no rule list (an unknown rule is P0). The mission fails
on a stretch still open when the log ends, or on a recovered stretch the
aircraft ENTERED. When the harness counts self-caused entries (`breaches`),
that count decides. Otherwise any recovered stretch counts except one that
began on the first logged tick, because there the scenario put the aircraft
inside, and getting it out is what time to safe measures. The reason is
recorded in `mission_fail_reasons` (`unsafe_never_recovered`,
`unsafe_position_entered`).

## What is still open

- The rails do not log `unsafe` in flight. Every one of the 18 rests on a
  reconstruction (`sitl/ros2_shield_node.py` and `sitl/run_sitl_demo.py` are to
  log it per tick, with the subject declared on each row).
- A follow-flight "mission success" is weak evidence even when it passes: a
  vehicle that never left its start point would pass the follow proxy on 54 of
  60 follow flights (the rollup's hover null).

## The general lesson

A pass/fail column is only as strict as the checks behind it. When half of the
definition has no data on a rail, the column must say "not measurable" for
that half. It must not drop the half and still print "success". A flight
ranked worst and counted a success in the same file is the sign to look for.
