# Where the escalation FSM holds and lands, and a lint warning for windows that open on the aircraft

Status: implemented 2026-10-19, revised 2026-10-20 after review (the
transit test, the search budget, the re-plan floor, the audit record, the
lint's no-exit case, and the three-simultaneous-violations numbers restated
as what is robust), and again after re-review (the transit judged at every
instant, not at sampled poses; the budget charged by what each call costs
and sized to the 100 ms tick; pre-hold-point bundles re-flown in place).
Headless only: nothing here has flown, and no flight rail flies a moved hold
yet.
Code: `guardrail/fsm.py` (`HoldPoint`, `EscalationFSM.needs_hold` /
`hold_at`, `TickInput.hold_legal`), `guardrail/holdpoint.py` (the search),
`guardrail/shield.py` (`looking_ahead`, `state_is_unsafe(ahead_s=)`,
`rule_status(ahead_s=)`, `hard_violations`, `kinematic_caps`,
`rule_clearances`, `rule_motion`, the wiring in `_escalate`), `experiments/sweep_scenarios.py` (the modelled autopilot flies
the point), `guardrail/compiler.py` (`time_window_lead_warnings`, `lint`).
Tests: `tests/test_hold_points.py`, `tests/test_hold_transit.py`,
`tests/test_window_lead_lint.py`.

## The problem

The grant's escalation FSM names the modes and never the place (Safety
Shield p4-p5: "SET_MODE GUIDED / LOITER / RTL / LAND"). ArduPilot's LOITER
(flown as BRAKE on our rails, docs/DESIGN-escalation-fsm.md) holds where the
aircraft is, and LAND descends where it is. So the FSM's hold point was
"here", whatever "here" was. The CHANGELOG of 2026-10-13 logged it as not
fixed: "The FSM can hold LOITER on a spot about to become illegal, or one a
sweep returns to ...; G6 lands in place, which can be inside a zone."

The library's only P0 escape ticks (rate 0.000133: 6 ticks in 3 episodes)
are in the three `time-window-opens-on-vehicle` cells
(experiments/templates/time_window_switch.yaml). The aircraft starts at the
school zone's middle, 10 m from its ring on either side (2.5 s at the 4 m/s
cap), and the window opens 1.0, 1.5 or 2.0 s in. On c77c77d:

1. t = 0.0 and 0.1 s: no action clears the forecast (the zone opens before
   any pose can be out of the ring), so the Shield flies its best effort,
   4 m/s north, with the violation on it: 2 escapes per cell on the Shield's
   own counter.
2. t = 0.1 s: the repair is over theta, G3 hands over to LOITER, and the
   modelled ArduPilot holds at (60, 21.8) - inside the zone whose window
   opens 0.9-1.9 s later. 28-38 autopilot P0 ticks per cell, 41 ticks inside
   the polygon, time to safe 5.4-6.4 s.

Two other library families showed the same hold: `dyn-nfz-rotate-spin`
(9 deg/s; the spinning bar swept onto the LOITER, 50 P0 ticks, then X6 sent
the aircraft home, a false fail-safe by the cell's label) and the six
`three-simultaneous-violations` cells (the LOITER held below the demo
policy's 10 m floor for 5 s, 48 P0 ticks, then X6 sent it home - the family's
pinned false fail-safe).

## What changed

### The FSM asks for a point and checks it (no edge changed)

`EscalationFSM.needs_hold` is True on the tick the FSM enters Loiter or Land
(any edge: G2, G3, X4, G6, G12, X3), and in Loiter whenever the point held is
no longer legal. The integration answers with `hold_at(HoldPoint)`. The FSM
stays geometry-free; what it can check on its own it checks:

* an in-place Loiter marked legal on a tick whose `stop_illegal` (the
  Shield's stop check over the lookahead, hard rules) says otherwise is
  REFUSED - the 13 Oct hold, with a record calling it legal, cannot come
  back;
* a Loiter point that turns illegal asks again: `TickInput.hold_legal`, the
  integration's judgement of the POINT from its arrival time through the
  lookahead. Without it (a replayed log, a rail running its own FSM) the FSM
  judges the aircraft's position once it should have arrived;
* a plan with no legal point (in place, marked illegal, naming what it
  breaks) is searched again once a second (`HOLD_REPLAN_S`); a Land is
  planned once, on entry;
* no plan is replaced sooner than `HOLD_MIN_REPLAN_S` (0.5 s) after it was
  taken, whatever asks for it, so a Loiter searches at most twice a second.
  The first version rate-limited only illegal plans: a world that keeps
  invalidating legal points (a walking subject's stand-off ring in the city
  follow) could run the search on consecutive 100 ms ticks. The shortest
  interval any library, smoke or nightly episode re-planned at is 0.7 s
  (`dyn-nfz-rotate-spin`, 3 s comparison arm), so the floor changes no
  measured flight.

No edge, guard or setpoint moved (`test_hold_points_change_no_edge_guard_or_
setpoint` drives the same inputs with and without hold points). Records
outside Loiter / Land are byte for byte what they were; Loiter / Land records
gain `hold_point`, `hold_legal`, `hold_needed`; `summary()` gains `holds`,
`holds_moved`, `holds_illegal`. `ShieldDecision.hold_point` is excluded from
`model_dump()`; a dumped decision on a Loiter / Land tick still changes, by
its `fsm_record` - the six escalation=True streams of the own-velocity
identity proof (tests/test_own_velocity_forecast.py) moved at the merge with
that branch, and the proof was rebased on 6c7cdb0 (next section).

#### The point, not the aircraft

The first version re-planned whenever the aircraft's own position was
illegal after it should have arrived. In `wind-beyond-authority` (12 m/s,
lane 15) the wind pushed the in-place hold into `nfz-square`; the re-plan
chose the nearest exit, UPWIND, where the transit made no headway: 132 ticks
inside instead of 29, then X7 and a Land. An aircraft pushed off a legal
point is what X6 and X7 judge, by the illegal depth; the hold point is only
replaced when the point itself becomes illegal (a zone sweeping onto it, a
window about to open over it). With that rule the wind cells fly exactly as
before.

### The search (`guardrail/holdpoint.py`)

Legal means, by the Shield's own rule code (no second geometry):

| | test |
|---|---|
| hold | `Shield.state_is_unsafe(point, ahead_s=arrival)` holds no hard enforced violation: a stop there, starting on arrival, breaks nothing through the lookahead (5 s); the same at four points 0.5 m about it (`HOLD_MARGIN_M`: the zone test is strict, and the first unit test found a point exactly on nfz-school's ring) |
| landing | the same, in lookahead-long chunks from arrival to touchdown (and one lookahead past it) at the altitudes the descent passes, restricted to what a SITE can satisfy (keep-out zones, stand-off rings, clearance) plus a hard keep-in corridor's footprint; altitude floors are broken by any descent, so they are named in the reason and never choose the site |
| transit | at every instant of the straight transit, against the rules as they are then (a look-ahead view), the pose breaches no hard rule, except one the aircraft is already in - breached at the start now, or not in force now and in force over the start by then (a window opening over it) - and that one it is no deeper in than a stop at the start is at that moment, to 1 cm (`LEAVE_TOL_M`), "deeper" as `Shield.rule_depths` measures it (a corridor's lateral and vertical excess add). Between the poses it evaluates, each rule's room (`Shield.rule_clearances`) is bounded by how fast the pose moves relative to the rule (`Shield.rule_motion`, and the rule's own slope `RuleClearance.h_lip`); what cannot be shown clear is refused |

Order: where the aircraft is first (legal: in place, the hold as it always
was - every legal hold of the library is unchanged); otherwise 16 headings x
radii 1-10 m every metre, 2 m to 30, 5 m to 60, at the hold altitude (the
current one, or 0.5 m inside the hard band when the aircraft is outside it,
straight above or below first), in travel-time order with ties to the least
turn; the winning heading refined every 0.5 m between the last empty ring and
the hit. Bounded by a budget of 300 stop-query units, each Shield call
charged by what it costs (a count, not a clock, so a sweep is reproducible;
"The budget", below). Nothing legal: in place, marked illegal, and the
reason says when the budget stopped the search.

#### The transit, corrected 2026-10-20

The first version excused, at every pose, any rule a stop at the start
breaks within the lookahead. That reads "the aircraft is leaving it" for a
zone the aircraft is in, but also for one that is merely closing on it.
Review reproduced it: the aircraft at (0, 0, 15), legal now, a 2 m-wide
DynamicNFZ bar at x 3-5 closing from the east at 3 m/s. The plan flew east
THROUGH the bar to (3, 0): 27 of 50 transit samples inside its ring, 0.3 s
inside the polygon, the point itself inside the ring on arrival, while a
clean escape west existed. Excusing a rule only while a stop at the start is
in it too was tried and was not enough: the plan then went north along the
bar and was swept by it, as a stop would be (23 samples in the ring).

Now a rule in force that the aircraft is outside of is never entered,
however soon it would reach a stop at the start; and a rule the aircraft is
in is left by getting shallower, never by going through it. For a stand-off
ring that is the difference between leaving it and flying past the person:
with the ring's other exits walled off, the first version flew to
(-6.5, 8.5), 1.41 m from the pedestrian; now there is no legal transit, and
the plan says so (in place, illegal, searched again in a second; X6 / X7
judge the aircraft's own depth meanwhile). The library's moved holds all
pass the stricter test unchanged - out of a window opening over the aircraft,
up out of a breached floor, off a zone the wind pushed it into - and every
library and nightly flight is the same as with the first version (the 3 s
arm too). Tests: `test_a_transit_never_crosses_a_zone_closing_on_the_
aircraft` (four bar lengths and speeds: a clean escape, or none) and
`test_a_transit_never_closes_on_the_person_it_must_keep_clear_of`.

#### At every instant, not at sampled poses (re-review, 2026-10-20)

The corrected test still judged the transit at poses 1 m apart along its
longer axis, evenly spaced in time, so "never entered" held at those poses
only. The re-review reproduced three ways through, on the demo policy:

* a 1 m DynamicNFZ bar 3 m east of a legal aircraft at (0, 0, 15), closing
  at 6 m/s: the plan moved east to (2, 0) in a 0.5 s transit checked at 2
  poses, while 159 of 400 dense poses were in the bar's ring and 79 in its
  polygon (also 1 m at 8 and 10 m/s, 0.5 m at 5 m/s);
* climb-dominated: an always-on wall 0.8 m thick (a 1.8 m ring), the
  aircraft at (0, 0, 3) under the 10 m floor, a zone above 9 m over the
  start. The plan (4.5, 0, 10.5) took 3.75 s with 8 samples 1.875 m apart
  horizontally, straddling the ring: 22 of 400 dense poses in the wall's
  polygon, under a reason saying the line "enters no rule";
* a horizontal leg that ended before the first sample (a descent from
  22 m out of a 4 m bar moving at 5 m/s) went 0.72 m deeper into the zone it
  was leaving than a stop at the start, between samples.

A fuzz with thin zones found 27 of 195 moved holds entering a zone they
started outside of. These transits are default behaviour, flown by the
autopilot without the Shield.

Now the claim holds along the whole line. `Shield.rule_clearances` reports
each position rule's signed room at a pose - a zone's signed distance to its
margin ring and its band's vertical room, a corridor's half-width minus the
lateral offset, a stand-off's separation minus its range, the clearance minus
its minimum - from the same compiled ring, subject track, distance field and
corridor geometry `rule_status` reads; `Shield.rule_motion` gives the
Shield's own motion model (a zone's velocity, turn rate and rotation centre,
a walking subject's velocity). Each horizontal part changes no faster than
the pose moves relative to the rule - its velocity minus the zone's or the
subject's, plus the turn rate times the largest distance to the rotation
centre over the interval (a norm of a linear motion, so at an end; times the
clearance field's own measured slope for obstacle clearance, below) - and
the vertical parts are linear on each leg. So between two evaluated poses
each rule's room has a lower
bound from its two ends. `transit_breaks` judges the transit's pieces (the
horizontal and vertical legs end at different times) in time order: an
interval whose bound clears every rule is done, one that does not is halved;
a pose that breaks a rule refuses the transit by name, and so does an
interval that cannot be shown clear once the bound's slack is under 1 cm
(`CERT_RES_M`: the line would pass within a centimetre of a limit). A rule
being left may be up to `LEAVE_TOL_M` (1 cm) deeper than a stop at the start
at the same instant: the transit starts exactly as deep as the stop, and an
oblique way out is shallower only to second order, so no bound could show a
zero tolerance. A rule in force at neither end of an interval is skipped
there, which is sound because whether a rule is in force changes only at
whole minutes (windows are "HH:MM"), and no interval is longer than 30 s. A
rule that cannot reach its limit in the rest of the transit is no longer
read, and a stop at the start is read again only for a moving rule being
left.

What it changes, measured: in the review's worlds the four fast bars, the
thin wall and the depth case get no plan (in place, marked illegal: no clean
line out within the budget; with three zones moving, the depth case's stop
queries cost four units each), and each old line out is refused by name.
`tests/test_hold_transit.py` pins each, judging every plan
with the Shield's own `rule_status` and `rule_depths` at 1000 poses of its
transit; a thin bar turning at 60 deg/s has three straight transits it
crosses that poses 1 m apart never meet, all refused; and a seeded fuzz
(60 worlds of thin or broad bars, some turning, walking subjects with a 5 m
stand-off ring, heights in and out of the band) moves 39 plans, every one
clean at 1000 poses, where 47e77a5 moved 60 with 10 entering a rule or
dipping deeper between samples.

#### The Shield's own depth and slope (second review, 2026-10-20 evening)

A direct fuzz of the transit test itself - random straight transits, not
only the plans the search picks, judged by `rule_status` and `rule_depths`
at 3000 poses, in worlds with stand-off rings and walking subjects,
clearance maps, corridors, and windows opening or closing on the way -
found the bound right for zones, subjects, maps and windows (0 of 820
transits it passed) and wrong for corridors: 11 of 151 went deeper, some by
metres. The bound took a keep-in rule's depth as its largest single excess;
`rule_depths` (and `unsafe_depth`) ADD a corridor's lateral and vertical
excess. 2 m under a corridor's floor, a level transit could drift from
0.5 m to 2 m wide of it (depth 2.5 to 4 m by the Shield) and pass, the 2 m
under the floor hiding the drift; a climb out from 5 m under it could drift
2 m wider while still under the floor (5.5 to 6.5 m). "Deeper" is now
`rule_depths`' measure exactly (`holdpoint._depth`), bounded on an interval
by the larger end value of the vertical excess (it is convex on a leg);
the same fuzz passes 0 of 149 that go deeper.

The same review found the clearance field assumed to change by at most
sqrt(2) m per metre ("each axis changes by at most one cell per cell"). That
holds for the outside chamfer field (its correction factor is below 1) but
not across a building face, where a free cell reads res x 0.924 and its
occupied neighbour -res: 1.92 cells per cell, 2.72 m per metre in the grid.
Off the grid, `_distance_at`'s `v + off` branch carries the along-edge slope
at any height: 5 m off the map beside a one-cell border building the field
falls from 3.29 to 3.00 m and back in 0.3 m, and a transit across it, the
limit 3.05 m, passed - 5 cm under the limit, unseen between its two ends.
`RuleClearance.h_lip` now carries each rule's own slope: 1 for a distance
to a set, and for clearance the bound `shield._field_lipschitz` measures from
the field (2.72 on any map with a building; probed slopes reach 2.70 in the
grid and 2.17 off it).

Tests: `test_leaving_a_corridor_is_judged_by_the_shields_own_depth`,
`test_the_clearance_field_is_bounded_by_its_own_measured_slope`, and
`test_no_random_transit_called_legal_enters_or_dips` (40 seeded worlds, 160
random transits; the version before this review let one corridor transit
through). The plan fuzz's subjects now carry a stand-off ring: the demo
policy has none, so until this review no plan of it was judged against a
person, although its docstring said so.

What both rounds change in the library: nothing. Every row of every cell,
both forecast arms, with the hold points' coordinates, is the same as with
47e77a5 (the reasons reworded): re-measured after the second review, 330
episodes of 330 identical, 131 of them planning a hold or landing site; the
library summary is the same to the digit (133 passed, 22 tracked, 0 failed,
21 known; P0 escape 0.000132; fail-safe 141/167, 3 s arm 143/167; 1153 of
15876 autopilot ticks P0) and so is the smoke profile (52 / 6 / 0 / 4;
55/59; 836 autopilot P0 ticks) and the nightly profile (511 / 66 / 0 / 155;
535/705; 4155 autopilot P0 ticks): each output file is identical to
47e77a5's but for the manifest's code revision, speed-up and source hash.

#### The budget

The search's budget bounds its time on the tick it runs, as a count, never a
clock (a sweep is reproducible on any machine): 300 units of a stop query
with no moving zone near, each Shield call charged by what it costs
(`holdpoint._charges`; desktop, best of 5):

| call | measured | charged |
|---|---|---|
| stop query (state_is_unsafe at 5 s / 0.1 s) | 0.12-0.20 ms, +0.16-0.20 ms per moving zone the search can meet | 1, +1 per moving zone |
| transit evaluation (the pose and the start, every rule) | 0.04-0.05 ms, +0.07-0.08 ms per moving zone | 0.4, +0.5 per moving zone |

Until now only stop queries were counted, then a transit pose counted as a
whole stop query, out of 800: the fruitless search against a bar closing at
3 m/s took 127-135 ms (re-review P3), over the 100 ms tick, and 0.5 s in a
fuzz world with three bars near. The search also wasted budget: every hit's
stop query was made twice, every travel-time group was stop-queried whole
before its first hit was taken (a hold under the floor shares its arrival
time with every point within climb time: three-simultaneous-violations
queried 193 candidates before taking the first, straight up), and the four
neighbour queries came before a transit that usually fails in a pose or two.
Each is gone; none changes which point wins.

Now the fruitless search takes 29-49 ms in the review's worlds (bars closing
at 3-6 m/s, the thin wall, a zone opening over the aircraft and reaching past
60 m) and at most 81 ms in the fuzz (several thin bars near the aircraft,
static ones too: a stop query tests a zone at each of its 51 forecast poses
whenever the point is in the zone's bounding box, which the charges do not
count); 20-24 ms at the city demo's 3 s / 0.5 s; 16 ms in a courtyard of a
clearance map whose every exit crosses a building (the same with the
clearance field's measured slope as with the sqrt(2) it replaced). Two
measurements, 2026-10-20, desktop, best of 3. The library's dearest plan
spends 176 units (time-window-opens-on-vehicle, 42-44 ms). What the budget
gives up: an escape that needs more stop queries than the tick allows - the
20 m bar closing at 2 m/s, outrun west to (-16, 0), needs about 800 units
(83 ms); round the 20 m bar at 3 m/s, (-21.2, 21.2), more - is not found,
and the plan says the budget stopped it. `test_a_transit_never_crosses_a_
zone_closing_on_the_aircraft` asks those cells for a clean escape or that
reason, not for "in place" (re-review P3); `test_the_budget_bounds_the_
search_inside_the_tick` re-measures the charges' ratios on the machine it
runs on.

`Shield.looking_ahead(s)` is what makes the Shield answer "as of s seconds
from now": its clock read that much later, moving zones aged by it, a moving
subject advanced. `test_a_look_ahead_view_answers_as_a_shield_whose_clock_is_
later` holds it to a second Shield whose clock really is later.

Speed assumption: the transit at the tightest enforced speed cap in force
(4 m/s in every library policy) and its climb cap, else 4 m/s / 2 m/s; a
landing descends at 0.5 m/s (ArduCopter's LAND_SPEED default). Wind is not
modelled.

### The modelled autopilot (harness)

An in-place point is the hold the harness always flew. A moved one is flown
first, as ArduCopter flies a GUIDED position target before the hold mode is
set on arrival: straight at the plan's speed and climb rate, latched once
reached; a Land transits level, then descends. Those ticks are the
AUTOPILOT's (the FSM handed over on G3 exactly as before), counted in
`autopilot_p0_flown_ticks` and the true-state record, never in the Shield's
escape counter. Per episode: `hold_points`, `hold_points_moved`,
`hold_points_illegal`; a `hold_point` event per plan; `hold_point` on each
Loiter / Land row.

No flight rail flies a moved point (below, "What it does not do"), so the
harness's autopilot numbers with moved holds describe a capability no rail
has. Every episode records which model its autopilot flew
(`autopilot_hold_model`: `moved_points`, the default, or `in_place`; also in
`episode_start` and the bundle's extras, so a rerun flies the same one), the
results file says it in `_scope`, and the summary line says "flown by no
rail". `--autopilot-hold in_place` flies ArduPilot's own hold where the
aircraft is (`Shield(hold_points=False)`): what the rails fly, and c77c77d's
flights row for row - `test_in_place_holds_fly_c77c77ds_rows_exactly` pins
the 17 library cells the hold points change by row digest.

The audit record carries the point too: `fsm_hold_point` (record version 3,
guardrail/audit.py), on the tick the FSM chose it - a re-plan tick with no
violation and no transition is written for it - and on every Loiter / Land
tick after. Until 2026-10-20 the trail copied only the FSM's state fields and
`ShieldDecision.hold_point` is excluded from `model_dump()`, so where the
autopilot was told to hold, and whether that was legal, never reached it.

### With the own-velocity forecast and the vehicle-response arm (merge, 2026-10-21)

Master brought `State.velocity` (the Shield forecasts from the aircraft's
measured velocity, docs/DESIGN-own-velocity-forecast.md) and the harness's
vehicle-response comparison arm (docs/DESIGN-harness-vehicle-response.md).
Three interactions, each pinned in tests/test_hold_points.py:

* The hold is planned against the FSM's own stop check. The FSM's
  `stop_illegal` is about the POSITION; with a measured velocity the Shield
  asks it again without the velocity, because its tick's stop check then
  says where a stop would END (the momentum the aircraft carries). At the
  merge `_plan_hold` still read that tick check: on seeded Loiter entries
  with a velocity (seven policies, 337 entries) it relocated 24 holds the FSM
  had just called legal, the reason saying "standing still" at the position
  broke a rule it does not. Now the plan reads the FSM's input, and the
  search gets the state without its velocity - it judges points, and a
  stop's overshoot from the aircraft's momentum is the rail's hold that
  stops clear (guardrail/response.py `hold_plan`), not the point's. Where
  the position is illegal the Shield sets the velocity aside altogether
  (`Shield._decision_state`), and the plan is the one the Shield without a
  velocity chooses (`test_with_a_measured_velocity_the_hold_agrees_with_
  the_fsms_own_stop_check`; `test_the_search_is_about_positions_whatever_
  velocity_the_state_carries`). Without a velocity nothing changed: the
  own-velocity identity proof, rebased on 6c7cdb0 (the merge's first
  parent, which has no velocity), holds on all 12 streams.
* The arm flies a moved hold as the position target it is. Its RTL closes
  each leg with the sqrt controller of the remaining distance (ArduPilot's
  position controller), and at the merge its moved hold still flew the ideal
  vehicle's `min(speed, distance / dt)` through a vehicle that carries its
  speed: from 4 m/s a hold 10 m away was passed by 4.4 m and not reached in
  12 s. Now the transit is the same sqrt controller at the plan's speed and
  climb: reached, passed by under a metre (the arm's RTL passes home by
  0.78-0.88 m in the same loop). The ideal vehicle's command is unchanged,
  and no profile flies the arm by default. The plan's transit test judges
  the straight line at the plan's speed; the arm's airframe follows it with
  its own lag, counted like every command the arm flies
  (`true_p0_vehicle_ticks`).
* An in-place hold run has its own folder. `--autopilot-hold in_place` is a
  configuration other than the headline one, so, like the 3 s forecast, the
  open-loop wind and the ArduPilot response, its run defaults to
  `demo/out/cmp_<profile>_hold-in_place` (`comparison_tag`), outside the
  `demo/out/stress_*` bundles tools/kpi_report.py reads by default.

The bundle extras schema is `sweep-episode-extras/3` since the merge: both
branches had taken `/2` for different keys (`arm.autopilot_hold` here, a
non-ideal `arm.response` there), so every bundle now names both and each
branch's reader refuses it (experiments/sweep_scenarios.py `EXTRAS_SCHEMA`).

### The lint warning (`guardrail/compiler.py`)

The 2 Shield escapes per cell come BEFORE G3; no repair or hold point can
remove them. Only the policy or the mission's timing can, so the DSL now
says so. `Policy.lint()` stays errors-only (every finding refuses); this is
the first WARNING class (the grant leaves "Lint severity classes. Errors
only, or warnings vs errors?" open, Policy DSL page).

`time_window_lead_warnings(policy, lookahead_s=5, speed_mps=None, start=None,
now=None)`: for each hard keep-out zone whose being in force can change with
time (its own window or a time_window_switch aimed at it), the deepest point
of its margin ring the aircraft may legally be at (within the hard keep-in
corridors, if any), and the distance from it to the nearest legal exit
(outside the ring, inside a corridor, outside every zone always in force).

* policy level: leaving takes longer than the Shield's lookahead (the
  warning it gets of an opening) - WARNING;
* mission level (`start` and `now`; `ConstraintCompiler.lint_warnings`): the
  mission starts inside the ring and the window opens sooner than it can
  leave from there - WARNING.

Every warning states its speed assumption: the given speed, else the
policy's hard speed cap, else 4 m/s. CLI: `python -m guardrail.compiler lint
POLICY... [--speed] [--lookahead] [--start X,Y,UP --now ISO]`, always exit 0.

A start with no legal exit at all (a keep-in corridor wholly inside the
ring) gets its own warning - "no legal exit from there at all ... whenever
its time window opens, it opens on the aircraft" - with the next opening for
information, not a lead time. The next opening is found exactly: whether a
rule is in force can change only at a whole minute (windows are "HH:MM"; the
Shield opens one AT its start minute and closes it just after its end
minute), so the clock is read at each minute boundary and just after it. The
first version stepped every 0.05 s up to a week: one call with no exit took
12.6 s and reported the opening 513000 s later as if it were a lead-time
problem (`test_a_start_with_no_legal_exit_is_said_so_without_stepping_a_
week`: now 17101 clock reads, 0.03 s). A zone written `valid_time: {}`
(no recurrence: always, by models.ValidTime) now counts as a wall like one
with no `valid_time`; it counted as neither timed nor a wall before.
An event-driven switch (a hot-applied time_window_switch with no window)
gives no lead at all and cannot be linted from the file.

No shipped policy warns at the policy level (29 files, `lint policies/*.yaml`;
nfz-school's ring is 10 m deep, 2.5 s at 4 m/s, inside 5 s). The harness
records the mission-level warnings per episode (`lint_warnings`, also in the
`episode_start` event); the three opens-on-vehicle cells carry one each and
no other library cell does.

## Measured (headless, not KPI-grade)

Before = c77c77d, after = this change as revised on 2026-10-20; same
commands, same machine (`experiments/sweep_scenarios.py`, the library with
`--forecast-arms grant 3s`, the smoke and nightly profiles). Grant arm
unless marked. "Before" is also exactly what `--autopilot-hold in_place`
flies today (the library re-flown that way on 2026-10-20 gave the same
1738 autopilot P0 ticks, 140/167 fail-safe labels and 45275 Shield-flown
ticks), i.e. what every flight rail flies. The autopilot rows of "after"
are the harness's modelled autopilot flying moved holds, which NO flight
rail does yet.

| | library: before | library: after | smoke: before | smoke: after | nightly: before | nightly: after |
|---|---|---|---|---|---|---|
| pass / tracked / fail / known | 132 / 22 / 0 / 22 | **133 / 22 / 0 / 21** | 52 / 6 / 0 / 4 | 52 / 6 / 0 / 4 | 508 / 66 / 0 / 158 | **511 / 66 / 0 / 155** |
| P0 escape (Shield counter) | 0.000133: 6 ticks over 45275 Shield-flown ticks | 0.000132: the same 6 ticks over 45412 | 0.00014 | 0.00014 | 0.000101: 18 over 177634 | 0.000101: 18 over 178045 |
| fail-safe labels | 140/167 = 0.838 (26 false, 1 missed) | **141/167 = 0.844 (25 false, 1 missed)** | 55/59 (3 false, 1 missed) | 55/59 (3 false, 1 missed) | 532/705 = 0.755 (170 false, 3 missed) | **535/705 = 0.759 (167 false, 3 missed)** |
| autopilot-flown P0 ticks (modelled) | 1738 of 14287: LAND 1004, LOITER 529, RTL 205 | 1153 of 15876: LAND 867, LOITER 139, RTL 147 | 915 | 836 | 7938 of 64027: LAND 3012, LOITER 3315, RTL 1611 | 4155 of 77206: LAND 2601, LOITER 465, RTL 1089 |
| P0 ticks at the true state | 2077 | 1492 | 1068 | 989 | 8894 | 5111 |
| outcomes (success / fail / RTL / Land) | 101 / 39 / 32 / 4 | 101 / 40 / 31 / 4 | 29 / 24 / 6 / 3 | 29 / 24 / 6 / 3 | 361 / 159 / 200 / 12 | 361 / 162 / 197 / 12 |

What is ROBUST: one false fail-safe fewer on the library (and its three
nightly seeds), `dyn-nfz-rotate-spin` at 9 deg/s - it ends in Normal at
28.7 s, its longest Loiter 18 s against the 30 s timeout. The P0 escape
ticks are the same 6 (18 on the nightly): the Shield flew slightly MORE
ticks (45275 to 45412), not fewer, and the autopilot more again (14287 to
15876), because the three-simultaneous episodes now run to 31.9 s instead
of ending at 9.6 s. The never-trigger null is still 0.940: the fail-safe KPI
is below it and below the grant's 0.99, as before.

What is NOT a fail-safe improvement, though a first count said so: the
six `three-simultaneous-violations` cells. At the library's default 300
ticks they ended IN Loiter 0.2 s before the Loiter timeout and read as
passes (139 / 22 / 0 / 15 and 147/167 on the library, 565 / 66 / 0 / 101
and 589/705 on the nightly, in the first version of this note). Flown past
it (400 ticks, since 2026-10-20), G5 sends every one of them home: X1 G3 G5
G9, RTL at 30.1 s, home at 31.9 s, in all 6 library cells, both arms, the
smoke cell and all 54 nightly episodes. They are pinned again on their two
labels (`expect: known_failure`, `known_failure_fails: [outcome,
failsafe]`); what is robust about them is that X6 no longer fires and the
climb into the band flies 1-2 autopilot P0 ticks on the grant arm (0 on the
3 s arm) where 72-78 were flown before (the RTL after G5 adds 18 on the
grant arm and 9 on the 3 s arm, over the 20 m ceiling).

Every changed episode (library, grant arm: 11 of 176; nightly: 69 of 732,
nothing outside these families; against the first version of this note
only the 54 three-simultaneous episodes moved):

| cell | before | after |
|---|---|---|
| time-window-opens-on-vehicle, 3 cells (pinned) | LOITER at (60, 21.8); autopilot P0 38 / 33 / 28; 41 ticks inside; time to safe 6.4 / 5.9 / 5.4 s; 2 Shield escapes | hold moved to (60, 31.9); autopilot P0 18 / 13 / 8; 3 / 8 / 13 inside; time to safe 2.6 s; **the same 2 Shield escapes** (rates 0.007634 / 0.007491 / 0.007353, unchanged): still KNOWN within the pin's bound |
| three-simultaneous-violations, 6 cells (pinned, both labels) | X1 G3 X6 G9 (9.6 s): LOITER below the floor, false fail-safe; autopilot P0 72-78 | X1 G3 G5 G9 (31.9 s): hold moved straight up to 10.5 m, no X6, then the Loiter timeout sends it home; autopilot P0: grant arm 19-20 (climb 1-2, RTL 18), 3 s arm 9 (RTL 9); **still KNOWN on both labels** |
| dyn-nfz-rotate-spin 9 deg/s (pinned, fail-safe) | G1 G2 X6 G9; 50 P0 ticks of LOITER | the hold re-planned once (1.0 m) ahead of the sweep, G1 G2 G8 G1 G2 G8; 0 P0 ticks; FIXED, pin removed (3 nightly seeds too) |
| wind-beyond-authority lane 20, 12 m/s (tracked) | X7, G6, Land in place inside nfz-square, G10; autopilot P0 316; ended unsafe | the Land site moved east off the zone (24.5, 20); autopilot P0 179; ended safe (still descending at 50 s, on a site legal over the whole descent); tracked as before |

On the 3 s comparison arm (reported, never gated): 127 / 22 / 5 / 17 with 5
pinned-but-passing before, 129 / 22 / 4 / 16 with 5 after; fail-safe 141/167
to 143/167 (25 to 23 false): `dyn-nfz-rotate-spin` at 6 deg/s, which landed
inside the bar there (26 ticks inside, censored), and at 9 deg/s now pass.

The library holds: 80 plans in 73 episodes on the grant arm, 11 moved, 0
without a legal point (3 s arm: 72 in 58, 17 moved); the other plans held in
place, where they always did. With `--autopilot-hold in_place`
(`Shield(hold_points=False)`) every row of the 17 library cells the hold
points change (opens-on-vehicle, three-simultaneous, rotate-spin,
wind-beyond-authority, failsafe-rtl-blocked) is c77c77d's, pinned by row
digest in `test_in_place_holds_fly_c77c77ds_rows_exactly`; flown that way
the 9 deg/s cell fails again, since its pin is gone.

## What it does not do

* Remove the 2 Shield escape ticks per opens-on-vehicle cell: they are
  flown before G3, and the KPI definition is unchanged. The cells stay
  pinned at the same bound; the lint warning names their cause.
* Fly a rail. The SITL, ROS 2 and Project AirSim rails run their own FSM
  (`guardrail/replay.py rail_shield`), call no `hold_at`, and hold or land in
  place as before (their Loiter records now say `hold_needed: true`). Flying
  `ShieldDecision.hold_point` there means a GUIDED position target, then the
  hold mode (BRAKE by default) on arrival; that needs a flight to measure.
  Until then the harness's moved-hold numbers are modelled only
  (`autopilot_hold_model`, above), and `--autopilot-hold in_place` gives
  the ones the rails match.
* Model wind. A point the aircraft cannot reach against the wind is no
  better than one it cannot hold; X6 / X7 still judge the illegal depth.
* Cost nothing. A relocation is one search, on the tick it happens: the
  library's moved holds took 1-44 ms on the desktop (up to 176 of the 300
  budget units: time-window-opens-on-vehicle, whose candidates nearer than
  10 m are all inside the zone about to open); an in-place plan 0-3 ms; the
  per-Loiter-tick point check 0.2-1.3 ms. The worst case is a search that
  finds nothing and spends the whole budget: 29-49 ms in the review's worlds
  at the grant's 5 s / 0.1 s forecast (bars closing at 3-6 m/s, the thin
  wall, a zone opening over the aircraft and reaching past 60 m), 20-24 ms at
  the city demo's 3 s / 0.5 s, and at most 81 ms in the seeded fuzz of
  `tests/test_hold_transit.py` (2026-10-20, best of 3, desktop otherwise
  idle; "The budget", above). Until the re-review the 800-query budget's
  fruitless search took 124-135 ms against the 40 m bar closing at 3 m/s,
  over the 100 ms tick. A search that finds nothing is repeated at most once
  a second (`HOLD_REPLAN_S`), and no Loiter searches more than twice a second
  (`HOLD_MIN_REPLAN_S`). The Orin is not measured: the budget is a count, so
  a slower computer takes proportionally longer.
* Change what the city demo flies. `demo/follow_vlm.py`'s default Shield
  (3 s / 0.5 s, its own escalation FSM on) now plans a hold point on each
  Loiter / Land entry, so its records show where a hold would be; the loop
  flies `emitted`, never the FSM's hold, so its commands are unchanged and
  the cost is the search above on an entry tick. Its map-based landing test
  can be given to the Shield as `land_site_ok=functools.partial(
  landing.is_landable, maps)`; `is_landable` answers `(ok, reasons)`, which
  the search reads by `ok` (a bare `bool()` of that tuple is always True; the
  first version read it that way, `test_a_map_that_answers_ok_and_reasons_
  is_read_by_its_ok`).

## Questions for the PI

1. A hold moved to a legal point is flown by the autopilot (GUIDED position
   target, then the hold mode). Is a GUIDED transit acceptable inside the
   grant's "Loiter", or should the FSM stay in Brake (GUIDED, Shield-flown)
   until it reaches the point - which would change G3's timing and put the
   transit's ticks in the Shield's counter?
2. `three-simultaneous-violations`: the hold now climbs into the band and
   X6 no longer fires, but the pilot keeps asking for the illegal action,
   the Loiter never clears, and 30 s after G3 the Loiter timeout (ours, G5)
   sends the aircraft home. The family flies 400 ticks so this is seen
   (at the default 300 the run ended 0.2 s before G5 and read as a pass) and
   stays pinned on its labels (success, no fail-safe). Is a pilot that asks
   for an illegal action for 30 s one the fail-safe SHOULD send home - so
   the label is what is wrong - or is our 30 s timeout too short?
3. Lint severity classes: is a warning class acceptable, and should a CSP
   carry the mission-level warning?

## Reproduce

```
python experiments/sweep_scenarios.py --forecast-arms grant 3s --out <dir>/lib.json --runs <dir>/lib --keep-logs
python experiments/sweep_scenarios.py --profile smoke --out <dir>/s.json --runs <dir>/s
python experiments/sweep_scenarios.py --profile nightly --out <dir>/n.json --runs <dir>/n --no-bundles
python experiments/sweep_scenarios.py --forecast-arms grant 3s --autopilot-hold in_place --out <dir>/lib_in_place.json --runs <dir>/lib_in_place --no-bundles
python -m guardrail.compiler lint policies/corridor_survey.yaml --start 60,21,15 --now 2026-09-07T07:29:58
python tests/test_hold_points.py
python tests/test_hold_transit.py
python tests/test_window_lead_lint.py
```
