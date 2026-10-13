# The escape that stayed inside the ring

**Date:** 2026-10-09 (the flight: 2026-10-08)
**Found by:** the first follow flight through the Shield and the escalation
FSM on the ArduPilot rail, `demo/out/pas_ap_20261008_224440` (Project AirSim
+ ArduPilot SITL, 120 s red-car follow, 1101 ticks, commanded over pymavlink,
none of the grant's topologies, not KPI-grade). Its `kpi.json` reported a P0 escape rate of 0.005336: 5 escapes
over 937 Shield-flown ticks.
**Status:** fixed in `guardrail/shield.py` (`_check_standoff`,
`_recover_from_overhead`, `_rescue` / `_deepens`) and
`demo/pas_ardupilot/perception_node.py` (`depth_implausible`). Every number
below is an offline replay of the logged ticks, flown before the fix; the
flights since are in the last two sections.

## What it was for

The grant's headline Shield KPI is the P0 violation escape rate, target 0:
"hard limit; report cannot ship if non-zero on P0" (Safety Shield, KPI
table). An escape is a tick on which the action the Shield emitted still
violates a P0 rule. On this flight the P0 rule was `standoff-any`: keep 5 m
horizontally from the followed car (hard, P0, `follow_car_citylife`).

## What went wrong

Five ticks (857, 927, 928, 999, 1100) emitted an action that still violated
`standoff-any`. On each of them the aircraft was "inside the stand-off ring,
stopping was illegal, and the escape repair did not leave the ring in one
tick", which is how the flight was first described. The log says more: on
all five the subject sat on the aircraft itself. The logged range from the
aircraft to the subject is 0.002-0.005 m, which is the log's rounding (the
subject to 1 cm, the state to 1 mm) of a range of exactly 0.

Four things lined up, and none of them raised an error:

1. **The perception node put the car under the aircraft.** It turns the
   depth inside the detection box (a slant range `r`) into a horizontal range
   with `sqrt(max(0, r^2 - dh^2))`, where `dh` is the camera's height above a
   car's centre (7.2 m at 8 m). A depth return shorter than `dh` became a
   horizontal range of 0. FrontCamera is mounted 20 deg down with a half
   vertical field of about 29 deg, so it cannot see the ground under the
   aircraft at all: whatever returned that depth was not the car. On tick
   857 the box's bottom-edge ray meets the road about 36 m out. The rail
   records no ground truth of the car; the ticks either side of the five
   ranged it at 3.5-55 m (tick 926, just before escape tick 927: 54.6 m), so
   the depth inside the box was erratic in general, and this was its worst
   failure.
2. **The stand-off check could not see any action opening a range of 0.**
   Already inside a ring, the check asks whether the range is opening, as the
   radial velocity along the unit vector to the subject. At range 0 the code
   set that vector to (0, 0), so the radial velocity was 0 for every action,
   and every action, including flying straight away, read "not opening".
3. **The repair stood aside.** The stand-off repair returned the action
   unchanged at range 0 ("directly overhead; no defined line").
4. **The rescue flew its best effort with the violation on it.** With the
   chain not converging and a stop illegal (range 0 < 5 m), the P0 guard
   searched 24 headings for one that re-checks clean. None could (point 2),
   so it flew the heading with the most building clearance (2.5, 4.33) m/s
   and logged `ClearanceEscape: stopping is itself illegal here ->
   best-effort recovery`. That action did leave a ring of radius 5 m around
   a point under the aircraft; the check said it did not.

From range 0 every horizontal motion opens the range at its full speed (the
one-sided derivative of |p - s| at p = s); the check simply could not
compute it. The FSM read the five ticks as "no operator converged": X1 into
Brake twice and G3 into LOITER three times.

## The evidence

A replay of the logged state, subject and raw action of every tick through
the Shield (`rail_shield`, the 5 s / 0.1 s lookahead, the CityLife occupancy
band 6-14 m the node used):

| Shield | P0 escape ticks | ticks agreeing with the log on escape / no escape |
|---|---|---|
| shield.py at 10451bc (the flight's) | 5: 857, 927, 928, 999, 1100 | 1101 of 1101 |
| shield.py after this fix | 0 | 1096 (the five, now clean) |

The old Shield reproduces the flight tick for tick, which is what makes the
replay evidence and not a model of it. `tests/test_flight_pas_ap_20261008.py`
runs this replay when the flight directory is on disk (it is not in git) and
reports SKIP, never a pass, when it is not.

A second, related number in the same log could not have been true: 20 of
the 25 `StandoffHold` repairs reported a `magnitude_m` deeper than the 5 m
ring's radius, up to 11.0 m (ticks 201-293, 750-759, 841, 1001). It computed
`ring - (d - closing x lookahead)`, a straight-line range that runs through
the subject and out the other side. No forecast can be deeper in a ring than
its radius. The five repairs "over theta" that sent the FSM to LOITER
(3.6-9.8 m; ticks 202, 282, 480, 750, 1001) were all measured this way. On
four of them the straight line ran through the subject, and they reported
more than the ring's radius (8.08, 9.81, 5.29, 9.23 m); on the fifth (tick
480: 16.5 m out, closing at 3.02 m/s) it ended 1.4 m short of the subject
(3.63 m reported). Tick 201: 8.3 m out, closing at 2.27 m/s; the forecast's
closest pose is 4.22 m from the car, 0.78 m deep, against the 8.08 m
reported. Theta stays on this field (docs/DESIGN-escalation-fsm.md, "Theta:
two readings, one default"); with the closest approach, the replayed flight
has 2 Brake -> Loiter edges instead of 8, both on head-on approaches whose
forecast went 2.4-4.3 m into the ring.

## The fix

- `guardrail/shield.py` `_check_standoff`: within 1e-6 m of the subject the
  opening rate is the horizontal speed. Any motion above the 0.1 m/s trend
  threshold passes; a hover is still a violation, as it must be (a stop
  there keeps the aircraft in the ring).
- `_recover_from_overhead`: `StandoffRecover` at range 0 keeps the commanded
  heading (the least change that leaves) or, for a hover, backs away from the
  aircraft's heading (which brings the subject back in front of a forward
  camera), at no less than the ring's recovery speed and no more than the
  speed cap. Flagged `recovery`, exempt from theta like the off-centre case.
- `_rescue` / `_deepens`: the best-effort heading, used only when nothing
  re-checks clean and a stop is illegal, is now chosen first among headings
  that go deeper into no hard ring or zone the aircraft is already inside,
  then by clearance, then toward the commanded heading. Before, a pilot asking
  to go north from inside a zone whose exit was north, 5.5 m south of a
  subject inside its 10 m ring, got 4 m/s north: straight at the subject. It
  now slides along the zone's face.
- `StandoffHold.magnitude_m` is the forecast's closest approach inside the
  ring (`ring - d_min`), never more than the ring's radius.
- `demo/pas_ardupilot/perception_node.py` `depth_implausible`: a depth
  return is refused, and the box width ranges the subject instead (source
  `width:depth-rejected`), when it is nearer than the camera's height above
  any subject (2.5 m), when Pythagoras would put the subject under the
  aircraft and the box centre's ray says the return sits above any subject,
  or when it is less than two thirds of the range at which the box's bottom
  edge ray meets the road. A fourth rule came out of the review of
  2026-10-09: the first three let a box low in the frame through at
  7.5-8.5 m from 7.96 m up, which Pythagoras placed 2.1-4.5 m away, 58-74 deg
  below the horizon - under the box's own bottom edge, outside what the
  camera saw, and inside the 5 m P0 ring. A range is now also refused when
  the placement it implies (the subject's centre asin(dh / r) below the
  horizon) is more than 3 deg below the box's bottom-edge ray.

Tests that fail on the old code and pass now:
`tests/test_shield.py::test_the_8_oct_escapes_with_the_subject_directly_below_are_gone`
(tick 857: `standoff-any` still flown), `::test_a_hover_directly_over_the_subject_backs_away_from_the_heading`,
`::test_a_best_effort_recovery_never_deepens_the_ring_it_is_inside`
(4.00 m/s at the subject), `::test_a_best_effort_recovery_never_heads_deeper_into_the_zone_it_is_inside`,
`::test_a_stand_off_holds_magnitude_is_a_depth_the_ring_can_have`
(8.08 m), `tests/test_pas_ardupilot.py::test_a_depth_return_nearer_than_the_road_below_is_not_the_subject`,
`::test_a_depth_that_would_place_the_subject_under_its_own_box_is_refused`,
and `tests/test_flight_pas_ap_20261008.py` (the replay, on both theta
readings).

## What this does not fix

- The depth inside the box is still erratic: the four rules remove the
  returns they can rule out, not every wrong one. A wrong but possible range
  (a car 30 m out read at 20 m) still reaches the Shield. The rail has no
  ground truth of the car, so the follow's stand-off numbers cannot be
  checked against one.
- How often the fourth rule refuses a good return is not known. The 8 Oct
  log does not record the body pitch, and the rule depends on it: at an
  assumed pitch of -3 / 0 / +3 deg it would have refused 29 / 36 / 49 of the
  156 depth fixes whose box is in detections.jsonl (and whose range is not
  0), all of them 13-177 m out, where a few degrees of
  pitch decide it (several of those returns, at 100-177 m for a box 15 px
  wide, were probably not the car). A refused return falls back to the box
  width, which is noisier but places nothing inside a ring. The node now logs
  `pitch_deg` on every row, so the next flight can answer this.
- The refusal reads the body pitch from ArduPilot's ATTITUDE and assumes the
  camera is fixed to the body at -20 deg. A gimballed camera would need its
  own pitch.
- The best effort that goes deeper into nothing can still be a violation of
  every rule it is inside: it is the least bad heading, and the FSM still
  escalates (X1, then G3 or X6 / X7).

## The re-fly of 2026-10-12

The node flew the same 120 s red-car follow with these fixes
(`demo/out/pas_ap_20261012_033329`, pymavlink, not KPI-grade): 1098 ticks,
43 Shield interventions, 0 escape ticks, and the FSM never reached Loiter.
That run does not test this fix. The detector's estimate was tracked on 129
ticks (269 on 8 Oct) and never came nearer than 16.5 m, so the 5 m ring was
never approached and no range-0 placement could happen. That run logged no
ground truth, so it cannot say the estimate was the car.

It did answer part of the question above. 43 depth returns were refused and
the subject ranged by box width instead, at 56-156 m; on the 31 refused ticks
with a depth-ranged tick within 5 ticks, the two ranges disagree by up to
122 m (width farther on 22, nearer on 9). "Places nothing inside a ring"
still holds for these, but the width fallback is not a close second
estimate. The log did not keep the refused depth or the rule, so whether the
refusals were right cannot be told; since this run the node logs both
(`subject_depth` on every row, from `depth_check`). The depth image itself
arrives quantised to 1 m.

## The close flights of 2026-10-13 and 2026-10-14

Four close-approach flights flew with these fixes and a pilot that closes
until its box fills half the frame (`demo/out/pas_ap_close_on`,
`pas_ap_close_off`, `pas_ap_video_on_20261014`, `pas_ap_video_off_20261014`;
pymavlink, not KPI-grade). Neither Shield-on flight had a P0 escape tick.
None of the four tests this fix against the car, because the detector was not
on the car: across the four close flights 0 estimates came within 6 m of it, and
the 2026-10-14 video shows the box on a red traffic signal (CHANGELOG
2026-10-14).

No estimate collapsed onto the aircraft. From each `flight_log.jsonl` (the
logged `subject` position against the aircraft's `x`, `y`), the nearest
estimate was 16.5 m and 15.3 m from the aircraft on the two Shield-on
flights and 5.3 m and 8.3 m on the two Shield-off flights; none came within
1 m.

The last sentence of the first point under "What this does not fix" no
longer holds; the rest of that point does (the depth is still erratic, and a
wrong but possible range still reaches the Shield). Since 2026-10-13 the rail
logs the followed car's simulator pose (`--subject-actor`, for evaluation
only), and that truth is what showed the detector off the car. The stand-off
numbers can now be checked against the car; on these flights the subject was
not the car, so there was nothing to check.
