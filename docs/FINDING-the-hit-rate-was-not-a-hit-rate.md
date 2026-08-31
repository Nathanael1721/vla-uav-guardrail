# The hit rate was not a hit rate

**Date:** 2026-08-31
**Status:** metric added, lock fixed and enabled, flights re-measured.

Found because the operator watched a video and said the drone appeared to be
following a different vehicle of similar shape and colour. It was, and the
metric that existed to catch exactly that could not see it.

## What `det_hit_rate` actually measured

`demo/follow_vlm.py` computes it as `n_seen / (n_seen + n_miss)`, and `n_seen`
increments whenever `det is not None` - whenever *any* box survived the score,
jump and colour filters. It never compares the box to the target.

**It is a detector-liveness rate.** A box on a parked lookalike, on a building,
or on road paint scores exactly like a box on the taxi.

That distinction was not academic. The ground truth (`tgt_x`, `tgt_y`, `x`,
`y`, `psi`) was in every flight-log row the whole time; projecting the target
into image coordinates and comparing gives:

| flight | `det_hit_rate` | **`frac_on_target`** | median err | p95 | detections with target off-frame |
|---|---|---|---|---|---|
| `city_full` | **0.995** | **0.728** | 8.2 px | 216.8 px | **37** |
| `city_kpi` | 1.000 | 0.930 | 3.6 px | 113.2 px | 12 |
| `demo_traffic` | 0.977 | **1.000** | 4.9 px | 48.9 px | 0 |
| `city_demo` | 1.000 | 1.000 | 2.7 px | 27.4 px | 0 |
| `people_check` | 0.629 | 0.693 | 17.2 px | 807.7 px | 154 |

**The two are anti-correlated at the top of the table.** `city_full` - the run
delivered as a demo video - has the best reported hit rate and the worst
tracking: 27 % of its boxes more than 100 px from the taxi, including 37 frames
where the taxi was outside the 90-degree field of view entirely and the
controller followed a box regardless. `demo_traffic`, the "worst" at 0.977,
tracked perfectly.

This retracts a claim made repeatedly during the week's work: `det_hit_rate
1.000` was cited as evidence that pedestrians and parked vehicles were not
stealing the tracker's lock. It could not have been evidence of that.

## Why the existing lock did not help

`TargetLock` already binds the controller to one instance, and
`tests/test_range_and_lock.py` covers it well. Two things stopped it working:

**It was never switched on.** `--lock-target` is opt-in and no flight has ever
passed it. `switched` is `False` on every record of every run on disk.

**Its gate was far too loose to have helped anyway.** The gate was
`0.28 * img_w` = 112 px on a 400 px frame. The switches that began the bad
episode were:

| tick | jump | box width | error after |
|---|---|---|---|
| 55 | **76.4 px** | 47 px | 86 px |
| 66 | **84.7 px** | 60 px | 94 px |
| 197 | 128.8 px | 62 px | 132 px |
| 256 | 101.0 px | 84 px | 125 px |

The first two - the ones that mattered - passed the old gate comfortably.
Meanwhile legitimate tracking moves the box by a **median of 0.0 px and a p95
of 8.8 px** between ticks, so 112 px admitted roughly thirteen times the motion
it was meant to allow.

The box width was the signal sitting in plain sight: the taxi measured 20-24 px
across, every wrong box 47-84 px.

## The fix

1. **`demo/track_truth.py`** projects the target and scores each detection:
   `frac_on_target`, `det_gt_err_px_median` / `_p95`, and
   `n_det_with_target_out_of_fov` - the unambiguous failures. It needs no new
   instrumentation, only arithmetic that was never done.
   `det_hit_rate` stays, with its docstring corrected to say what it measures.
2. **Gate 0.28 -> 0.12** (48 px), five times the p95 of real motion instead of
   thirteen.
3. **Size consistency**: a candidate whose width differs from the held
   instance's by more than 1.8x is a different object, however close to the
   prediction it lands. The held width follows the instance across a switch, so
   re-acquiring does not leave the lock comparing against a vehicle it stopped
   following.
4. **The runner-up is logged.** Only the winner used to be, so the margin
   between first and second choice was unrecoverable from any artefact - which
   is why establishing this needed a ground-truth reconstruction rather than a
   query. Two numbers a tick.
5. **`target_lock` in the metrics**, reporting `enabled: False` explicitly. A
   lock that was never on looked identical to one that was, and that is how a
   flight followed the wrong vehicle for thirteen seconds unnoticed.

## Result

Same scene - 12 parked vehicles, 3 moving, 12 pedestrians - with the lock on:

| | lock off (`city_full`) | lock on (`lock_on`) |
|---|---|---|
| `frac_on_target` | 0.728 | **1.000** |
| median error | 8.2 px | **1.9 px** |
| p95 error | 216.8 px | **23.0 px** |
| detections with target off-frame | 37 | **0** |
| `sep_end` | 16.6 m | 16.2 m |
| `det_hz` | 3.7 | 4.39 |

Guardrail invariants unchanged: P0 escape 0.0 with `p0_ticks_not_measurable` 0,
NFZ 0.0 s, altitude 0.0 s.

On the recorded run the lock reports `locked 194, switched 1,
rejected_candidates 4, size_rejected 59` - the size test did most of the work,
which is what the box-width numbers above predicted.

**The SITL KPI figures are untouched by this.** They are stub-pilot waypoint
missions with no detector in the loop; the canonical-hil evidence and
`p0_violation_escape_rate` stand as reported.

## Still open

- **Appearance re-ID.** `appearance()` computes a 32-bin HSV histogram of the
  chosen box on every inference and throws it away: `app_sim` is null on every
  row of every flight, because `appear_min` defaults to 0 and has no CLI flag.
  It is the strongest tool against a same-shape same-colour lookalike. Position
  plus size was cheaper and has so far been sufficient; the histogram is the
  next move if it is not.
- **The runner-up is logged only for the chosen tick's candidate list.** The
  full candidate set is still not persisted, so a post-hoc question about a
  third candidate cannot be answered.
- **`frac_on_target` is horizontal only.** It uses `cx` against a projected
  bearing. A box at the right bearing but the wrong elevation or range scores
  as on-target. Sufficient for distinguishing vehicles in a street scene, not
  for a general 3-D claim.
