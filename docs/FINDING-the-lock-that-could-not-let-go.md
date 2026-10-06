# The lock that could not let go

**Date:** 2026-09-29
**Status:** fixed offline and replayed; flights in the last section.
Day-by-day record (Indonesian): `docs/WORKLOG.md`.

The reference video for the red-car mission is `citylife_redcar_trail`. At one
point the car disappears, the drone looks lost, and it never finds the car
again. This is what happened, why nothing in the old system could stop it, and
what replaced each piece.

---

## What the video shows

| t | what happened |
|---|---|
| ~110 s | The red car goes behind a white box truck 38 m ahead. The drone coasts. |
| ~140 s | It searches, toward a bearing of -34 deg - the wrong way - and the HUD says "NFZ AHEAD - HOLDING" although the flight has no no-fly zone (a building corner is holding it). |
| ~180 s | The HUD says **TARGET LOCKED** on a red pedestrian signal, 110 m from the car. The drone sits at the building corner. |

Across the seven trail flights the estimate the controller flies on was on the
car on **36 %** of estimator ticks (74 % on `_trail`, the best). The corner was
never the limit; **whose box it is** was.

## Why nothing stopped it

1. **The ground check let the signal in.** It rejected a box only when the ray
   through its bottom edge met the road more than 1.5x beyond the depth range,
   which at 8 m altitude is a bottom edge above ~2.7 m. A kerbside pedestrian
   signal stands 2-3 m up and passed about 95 % of the time.
2. **The lock could not say "none".** `TargetLock.select` took the candidate
   nearest its prediction; when nothing was near, it took the best-scoring box
   and called it a switch. Once its 2 s hold lapsed, every box was "the first".
3. **The estimator's gate grew with the loss.** After seconds of prediction the
   covariance had grown enough that a box 75-190 m away passed the Mahalanobis
   gate and re-seeded the estimate - and the HUD said TARGET LOCKED.
4. **The search rotated in place**, wherever the drone happened to stop.

And three measurement faults under all of it: bearings used a linear
pixel-to-angle map (off by up to ~4 deg, physical widths ~21 % small); each box
met the pose of the tick that used it and the latest depth frame, 0.2-0.4 s
after its own image; and `det_hz` in every metrics.json was inflated (below).

## What is physically true of a car, and what is not

`demo/identity.py` turns each detector candidate into what it physically is,
at the pose and depth OF ITS OWN FRAME: width in metres, aspect, the height of
its bottom edge above the road, and its distance from the street mask. Three
tiers, because the costs are not symmetric:

- **HARD** - impossible for a car: bottom edge > 1.5 m above the road, > 4 m
  off every street, > 8 m wide, a box filling the frame. Never steers.
- **SOFT** - unlikely, but a real car can look like this: a car half behind a
  truck is an 11-15 px sliver, 0.6-1.0 m "wide", aspect 0.55-0.65 - exactly a
  pedestrian signal's signature. SOFT may continue a live track, never start
  one.
- **OK** - the rest.

`tools/replay_identity.py` labelled 5,815 logged boxes from 12 red-car flights
on-car / off-car from ground truth (at capture time) and tuned the thresholds
leave-one-flight-out:

| | wrong boxes not OK | true boxes not OK | true boxes HARD |
|---|---|---|---|
| held-out, pooled | **86.5 %** | 15.2 % | **0.6 %** |
| in-sample, pooled | 86.4 % | 14.1 % | 0.3 % |
| the brief's defaults | 71.1 % | 8.5 % | 2.4 % |
| HARD rules only | 38.4 % | 0.3 % | 0.3 % |

The gate set before flying was >= 70 % / <= 15 % / <= 3 %. Held out, the
true-box rate misses it by 0.2 points; only the in-sample figure meets it.
It is reported as a near miss, not a pass (a draft said 15.4 % and "passed";
that run predates the bottom-clip guard moving into `identity.features_for`).

What still passes as OK is other vehicles: 282 of the 331 passing wrong boxes
are car-sized and on the street. Physics cannot tell one car from another;
that is the tracker's job. The only true car HARD-rejected near the aircraft
was a box cut off by the bottom of the frame (the car under the nose), so a
box touching the frame bottom now has no bottom edge.

## A lock that can answer "none"

`TargetLock.select_strict` (on with `--identity`):

- nothing is adopted that a start gate or a re-acquisition did not seed;
- an OK box must lie within 12 % of the frame of the predicted position, a
  SOFT one within 6 %, and a SOFT one does not change the width the lock holds;
- once its own prediction is stale, the estimator's, projected into the frame
  at the capture pose, stands in - and the candidate's map point must also be
  within 10 m (OK) or 6 m (SOFT) of the predicted one. That is what refuses a
  signal in the car's pixel column but 30 m behind it.

## Losing the car, and taking it back

After 3 s without an accepted measurement the estimate is dropped. Only
`Reacquirer` restarts it:

- 4 OK sightings in a row, each within 45 m of the aircraft;
- consecutive map points consistent (<= 3 m + v dt);
- **within reach** of where the car was last measured:
  |P - P_last| <= 10 m + v_r dt, v_r = clip(1.5 x measured speed, 3, 12) m/s;
- and when nothing constrains where the car is - no anchor (the start, a
  retarget) or a reach grown past 60 m - **seen moving**: the median map point
  of the second half of the streak must lie 2 m **plus three standard errors
  of that difference** from the first half's (the jitter measured from the
  streak's own successive differences). A streak that has spanned 8 s
  without clearing that bar marks its place STATIC; a static place is
  refused for 30 s, then forgotten - people wait at kerbs, cars at red
  lights.

The motion rule came from the replay. The first version re-seeded twice on
`citylife_redcar_far` on a red fire-hydrant sign on a stand by the launch
point - car-wide by depth, on the street, perfectly still - and, after long
losses, on whatever was within a reach that had grown to hundreds of metres.
First-to-last displacement could not tell the sign from a car: its map point
jitters 3 m between frames (1 m depth quantisation, road behind the box).
Medians over half-streaks can - with the bar raised by the noise. A fixed
2 m let a sign jittering 1.5 m per axis through on 186 of 200 seeded runs;
the noise-aware bar refuses it on all 200 and still passes a car at 3.2 m/s
(1 m jitter) on all 200. The first version also counted sightings, not seconds, so a
pedestrian at 1.3 m/s seen at 5.5 Hz was marked a sign before it had
walked 2 m.

The start gate uses the same world-based gate, with the start range (30 m).

## Replayed on what was flown

`tools/replay_pipeline.py` runs both pipelines open-loop over the 12 recorded
red-car flights, with exactly the same boxes (the one the old lock chose per
inference - the log kept every candidate only while acquiring). The old side
is the estimator as flown (vendored from 1d09786, never reset, presence gate
honoured); the rates are pooled integers:

| | as flown | new |
|---|---|---|
| estimate served (younger than 3 s) | 45.1 % of ticks | 7.6 % |
| of those, within 6 m of the car | **22.9 %** | **94.5 %** |
| wrong seeds / first accepts after a gap (> 10 m from the car) | **210 of 241** | **0 of 12** |

The as-flown arm reproduces the flight's own `served` flag on 99.9 % of the
rows of the four flights flown by that loop (`_final1..4`); on those four
alone it is on the car on 10.7 % of served ticks, the new pipeline 96.4 %.
An earlier draft of this table compared against an "old" arm that reset and
re-seeded after 3 s, which the flown code never did (22.4 % vs 95.5 %,
260 of 293 vs 0 of 18).

The served share is low because the replay only has the boxes the OLD lock
chose; a true car it passed over is not in the log. In flight every candidate
is judged. What the replay can show, it shows: a wrong box can no longer
capture the estimate.

## Searching on its streets

`--search-planner` (`demo/search.py`), after the coast and the bounded sweep:
pursue the car's street to the next junction ahead, at about its measured
speed, sweeping +-35 deg down the street; look down each exit 2 s; then hold
over the junction, turning slowly - a car on a loop comes back round. A car
last measured stopped is not passed: the drone waits at the follow's stand-off.

## Smaller faults found on the way

- **One pinhole camera model** (`demo/camera_model.py`) for every bearing and
  width; `--linear-bearing` reproduces older flights, and an AST test keeps the
  linear formula out.
- **Capture-time pairing.** `SemanticObs.get_front_capture` returns the image
  with the pose interpolated at its capture (sim stamps when both have them)
  and the depth frame nearest in time (none beyond 70 ms). The estimator folds
  each measurement in at its capture time: it keeps its posterior at the last
  update and no longer lets `predict` move it.
- **The HUD** says OBSTACLE, not NFZ, when a building holds the drone, and
  TARGET LOCKED only on a fresh OK box (else TRACKING (PREDICTED),
  RE-ACQUIRING n/4, PURSUING ...).
- **Landing** flies, through the Shield, to a pavement cell `demo/landing.py`
  calls landable before the descent (the old descents ended in a hedge on
  `_trail2` and on a parked car on `citylife_ped_final`).
- **`det_hz` was inflated on almost every flight.** It divided every
  inference since the detector loaded - start-gate waiting included - by
  ticks x 0.1 s. `citylife_follow2` reported 7.60 Hz for 3.06; the red car
  6.9-7.5 for 3.5-4.0. The "level still streaming" confound written this
  morning was this artefact, and is retracted with the others in CHANGELOG.

## Flights

Video: `docs/video/citylife_redcar_identity.mp4` (`_id1`, the first-person view
with the HUD beside the chase camera).

`run_citylife_follow.ps1 -Object "a red car" -LevelCar Car_10 -Seconds 240`,
with the editor closed, no other load, and frames written to C:. Each flight
started within 18-22 m of the car after a 210-225 s wait at the start gate.

| | within 30 m | max separation | estimate on the car | lapses / re-acquired | landing |
|---|---|---|---|---|---|
| `citylife_redcar_id1` | **0.992** | 31.1 m | **1.000** (median 1.83 m) | 0 / 0 | pavement |
| `citylife_redcar_id2` | **0.980** | 31.6 m | **1.000** (1.86 m) | 0 / 0 | pavement |
| `citylife_redcar_id3` | 0.495 | 224 m | **0.996** (1.85 m) | 2 / 1 | 2.94 m from an obstacle (3.0 needed) |
| `citylife_redcar_id4` (both fixes) | **0.970** | 31.5 m | **1.000** (1.84 m) | 0 / 0 | pavement, 13.9 m to a margin site |
The seven trail/final flights before this work were within 30 m 9-44 % of
the time, and their flown estimate was on the car on 1-70 % of the ticks it
was served.

On `_id1` and `_id2` the drone never lost the car. Both flights left 30 m
only for moments, by at most 1.6 m. On `_id1` at t = 150 s the HUD reads
TARGET LOCKED on the car at 19.9 m, centred, green lamps lit and pedestrians
on the pavement.

On `_id3` the drone lost the car twice. Neither time did its estimate move to
anything else.

- At t = 74 s the car nearly stopped (0.67 m/s) and the estimate lapsed.
  The Reacquirer took the car back 1.35 s later at 15.5 m, on four OK
  sightings.
- At t = 110-114 s the car turned sharply close under the aircraft: its box
  grew from 46 to 94 px at 15-19 m, and then it left the frame. The junction
  planner took over.
- The car then stood at a red light 51.8 m away, in view, every box OK and
  within reach. Every box was refused "too far" (65 times), because
  re-acquisition stops at 45 m. The drone held over another junction until
  the end.

The fix is the far lead (above): a candidate refused only for its range is
flown toward, and the ordinary gate takes it from inside 45 m. Replayed on
`_id3`'s own logged candidates after the second loss, the lead forms at
t = 126 s, 4.3 m from the car, while the drone is 51 m away. `_id4` flew
with it and never lost the car, so the steering itself has not been
exercised in flight. The landing
near-miss led to a 1 m margin on every site the drone flies to.

**What this does not cover.** Re-acquisition has been exercised in flight
once (1.35 s on `_id3`). The rest of the evidence is the replay. The Demo_day
regression under `-Identity` was within 30 m 93.6 % against 100 %. Every
miss came in the 4 s after a 7.1 s acquisition: that runner has no start
gate, and with no anchor the Reacquirer waits to see the car move. People are
not covered. With `--identity` a person box ranged on the facade behind it
passes as OK, because the presence gate's width test no longer runs (46 % of
estimates near anybody on `citylife_ped_id`). The CityLife runner therefore
uses identity for the car mission only.
