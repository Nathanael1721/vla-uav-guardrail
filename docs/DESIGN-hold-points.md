# Where the escalation FSM holds and lands, and a lint warning for windows that open on the aircraft

Status: implemented 2026-10-19, revised 2026-10-20 after review (the
transit test, the search budget, the re-plan floor, the audit record, the
lint's no-exit case, and the three-simultaneous-violations numbers restated
as what is robust), and again after re-review (the transit judged at every
instant, not at sampled poses; the budget charged by what each call costs
and sized to the 100 ms tick; pre-hold-point bundles re-flown in place).
Headless only: nothing here has flown, and no flight rail flies a moved hold
yet. (Since 2026-11-07 the pymavlink rail and the ROS 2 Shield node fly the
moved hold - guardrail/hold_flight.py - and a hold must stay legal for as
long as it is held: "A hold legal for as long as it is held, flown by the
rails", below. First flown on the rig on the hil shakedown hs02, where the
tick that planned it broke the Orin's 100 ms budget; since that evening the
search is spread over ticks, a fixed count of its budget a tick: "The
per-tick bound", below. hs03 then gave the transit up while the aircraft was
still braking out of the hand-over and X6 sent it home; the give-up rule is
fixed and the harness's rail arm flies it too: "The transit's give-up, and
the rail arm flying it", below.)
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
the four hold-point keys of its `fsm_record` and nothing else: the
own-velocity identity proof (tests/test_own_velocity_forecast.py) leaves
those four out and hashes the merged tree's escalation=True decisions to
c77c77d's digests, 18,000 decisions (next section).

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
the hit. A candidate inside a rule that stands still is dropped by a cheap
read before its stop query (the rule that dropped the last candidate read
first, alone). Bounded by a budget of 600 units of about 0.1 ms,
each Shield call charged by what it costs in the world it is made in (a
count, not a clock, so a sweep is reproducible; "The budget", below).
Nothing legal: in place, marked illegal, and the reason says when the budget
stopped the search.

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
clock (a sweep is reproducible on any machine): 600 units of about 0.1 ms of
the desktop's time (`holdpoint.UNIT_MS`), each Shield call charged by what it
costs in the world it is made in (`holdpoint._Ledger`; desktop, best of 5,
2026-10-21, at the grant's 5 s / 0.1 s forecast):

| call | measured | charged (units) |
|---|---|---|
| stop query (state_is_unsafe of one point: its 51-pose forecast) | 0.074-0.078 ms | 0.8 |
| + per static zone whose ring's box holds the point | +0.027-0.033 ms in its band (inside or outside the ring), +0.010-0.012 ms out of it | +0.35 |
| + per moving zone the search can meet | +0.14-0.20 ms | +2 |
| + per altitude envelope / corridor segment / stand-off or clearance rule | +0.04 / +0.046 / +0.005-0.009 ms | +0.4 / +0.5 / +0.1 |
| read of one position (rule_clearances) | 0.002-0.004 ms | 0.05 |
| + per zone read / other rule read | +0.016-0.018 / +0.0003-0.0035 ms | +0.2 / +0.05 |
| + per moving zone read (built at the view's time) | +0.04-0.05 ms | +0.5 |

A transit evaluation is the reads it makes; the start's own reads are made
once a search (until then every transit re-read the start's status).

**The cheap test** (final review of T1c, P1). Every candidate used to cost a
whole stop query whatever it held, the budget was 300 of them, and 16
headings over the rings out to 28 m spent it: nothing past about 28 m was
ever tried, in any world, so the module's "60 m" was 28 m and a hold that
needed 30 m was held inside the zone (the review's reach world: a square
always-on zone around the aircraft, left at a half-width of 26 m, not from
28 m; `test_a_static_world_is_searched_to_its_edge`). Now, before a
candidate's stop query, the rules that stand still - static zones whose box
holds the point, altitude envelopes, corridors, clearance, a stand-off from
a still subject - are read at the point in the arrival view, and one of
them breached there, hard and enforced, drops the candidate: each of those
checks flags a stop inside it at t = 0 of its own forecast ("not escaping",
"not returning", "not climbing", "not opening", "not receding"). A moving
rule is never read for it: a stop inside a zone that moves away from it can
be legal. `test_the_cheap_test_drops_only_what_the_stop_query_refuses` holds
it to the stop query in 40 random worlds.

**One rule first** (integration review of T1c). The rule that dropped the
last candidate is read alone first (0.25 units for a zone), and the others
only if it is not breached at the point; of several breached, the one the
point is deepest in is kept. Until then every still rule whose box held the
candidate was read and charged, so a candidate inside k nested zones cost
0.05 + 0.2 k units: with 8 or 10 squares about the aircraft (half-widths 50
down to 46.5 m) the budget was spent on the candidates nearer than the 55 m
ring and the hold stayed in place, illegal, where 51.5 m out was legal.
The answer is the same either way; only the price falls.
`test_nested_static_zones_cost_one_zone_read_a_candidate`.

**What it reaches and what it costs** (desktop, best of 3, 2026-10-21; the
final review's worlds and three more, the "before" column measured on the
merged tree just before the change; the rows that "One rule first" changed
measured again after it, on 08cd0be and on the change, same desktop, best of
3, the 08cd0be figure in brackets):

| world | before (300 stop queries) | now (600 units) |
|---|---|---|
| square zone around the aircraft, half-width 30 / 50 / 57.5 m | held inside, budget spent, 35-36 ms | moved to (31.5, 0) / (51.5, 0) / (59, 0), 10-12 ms (landing 14-16 ms) |
| square zone, half-width 80 m (nothing within 60 m) | budget spent, 36 ms | "nothing within 60 m", 11 ms |
| a 150 m zone opening over the aircraft in 2 s | budget spent, 35 ms | nothing within 60 m, 23 ms |
| 5 / 20 overlapping always-on zones | 77 / 226 ms | nothing within 60 m, 12 / 15 ms [40 / 54 ms, the 20 spending the budget] |
| 20 stacked out-of-band zones over the aircraft + a big zone | 114 ms | 29 ms (budget spent) [30 ms]: its candidates nearer than 8 m arrive before the big zone opens, so each is a stop query in 21 boxes (0.8 + 21 x 0.35 = 8.15 units) |
| a core zone and 20 diamonds whose boxes hold every candidate | 907 ms | nothing within 60 m, 15 ms [52 ms, budget spent] |
| the same with a 12 m core (legal at 13.5 m) | 651 ms | 15 ms (landing 33 ms) [37 ms, landing 54 ms] |
| 8 / 10 squares nested about the aircraft, half-widths 50 down to 46.5 m (legal at 51.5 m) | not measured | moved to (51.5, 0), 14 ms (landing 18-19 ms) [held inside, budget spent, 54 ms] |
| 40 m bar closing at 3 m/s; 20 m bar at 2 m/s | 32 ms, budget spent | 41 ms, budget spent [unchanged: 38-39 ms on both] |
| 3 m bar at 3 m/s | 17 ms | 14 ms |

Static zones used to be charged nothing, so the budget did not bound the
search's time with many of them (the review measured 224 ms with 20
overlapping zones); the diamonds were worse, because a stop query tested a
zone at all 51 forecast poses of the stop whenever the point was in the
zone's box but outside its ring, about 0.45 ms a zone. shield.py
`_check_fence` now tests a run of poses at one point once (a pure function;
every decision is the same, which the own-velocity identity proof's 18,000
decisions and `test_shieldcore_identity` hold), and the box is charged. With
moving zones the reach is what the budget buys at their price: the bars
closing at 2-3 m/s are still fruitless inside it. `test_the_budget_charges_
what_the_search_makes` holds every charge to the call it paid for, and
`test_each_charge_bounds_what_its_call_costs` re-measures the prices on the
machine it runs on - wall clock, so only with `GUARDRAIL_TIMING_TESTS=1`
(its predecessor failed 1 of 40 runs with eight copies in parallel, final
review P4; on the desktop each call and each fruitless search came to
0.65-0.98 of its charge, against a limit of 2). Since One rule first the
whole searches are held to what their ledger spent, not to the budget: the
bar's came to 0.67 of it, and the overlapping zones' and the diamonds' to
1.37 (desktop, best of 3) - they end at the last ring after 416 reads of
one zone (0.25 units each), and the search's own work per candidate (its
view, the box test), which is not charged, is then about a quarter of the
time. That work grows with the candidates tried, at most 416 and the
refinement, not with the budget: about 4 ms in these two worlds, which try
every candidate.

`Shield.looking_ahead(s)` is what makes the Shield answer "as of s seconds
from now": its clock read that much later, moving zones aged by it, a moving
subject advanced. `test_a_look_ahead_view_answers_as_a_shield_whose_clock_is_
later` holds it to a second Shield whose clock really is later.

Speed assumption: the transit at the tightest enforced speed cap in force
at any time a transit the search plans can still be flying - each lookahead
up to the longest transit read in its own look-ahead view - (4 m/s in every
library policy) and its climb cap, else 4 m/s / 2 m/s. Until 2026-10-21 the
first lookahead's caps were the plan's, so a tighter hard cap whose window
opened after it but before arrival was broken by the transit (final review
P3: a 1 m/s P0 cap opening 5.6 s ahead, 5 of 100 transit instants at 4 m/s;
`test_a_speed_cap_that_opens_on_the_way_slows_the_whole_transit`); a
landing descends at 0.5 m/s (ArduCopter's LAND_SPEED default). Wind is not
modelled. The reason words the speed and the climb rate each by its source
- a cap in force, one that comes into force on the way, or the default
assumed (a later cap no looser than the default becomes the source). Until
the integration review of T1c one flag worded both, so with no cap now and
a 5 m/s / 1 m/s cap opening on the way the reason called the default 4 m/s
an enforced cap, and with a 4 m/s / 2 m/s one opening on the way it said no
cap was in force; the same test asserts each reason whole.

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
  own-velocity identity proof holds on all 12 streams against c77c77d,
  with the four hold-point keys left out of each dump (they are the hold
  points' own, pinned here).
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
  (`true_p0_vehicle_ticks`). Flown with `--response-arms ideal ardupilot` on
  the two families whose holds move (9 paired episodes, 2026-10-21): in
  time-window-opens-on-vehicle the arm's true P0 ticks on the command fell
  from 66 at the merge to 54 (45 on the ideal vehicle; the airframe's 45
  either way); three-simultaneous-violations, whose arm numbers are its RTL
  leg's, went 258 -> 260 on the command and 352 -> 350 on the airframe.
* An in-place hold run has its own folder. `--autopilot-hold in_place` is a
  configuration other than the headline one, so, like the 3 s forecast, the
  open-loop wind and the ArduPilot response, its run defaults to
  `demo/out/cmp_<profile>_hold-in_place` (`comparison_tag`), outside the
  `demo/out/stress_*` bundles tools/kpi_report.py reads by default.

Re-confirmed after the merge and the final review's changes (5891cb2,
2026-10-21): the library (grant and 3 s forecast arms), smoke and nightly
results equal 6c7cdb0's on every field both write - 0 value differences,
ignoring the code revision, timings, sim speed-up, the pilot's source hash
and the command line - and the only fields added are master's
vehicle-response records (`response_arm`, `vehicle_response`, the empty
comparison section). Against master the differences are exactly 6c7cdb0's
against master (the hold points' own). The library's dearest search now
spends 317 of its 600 units (time-window-opens-on-vehicle at 07:29:58).

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

## The opens-on-vehicle escapes, removed by another route (C2, 2026-10-21)

The hold points could not remove the 2 Shield escape ticks per
opens-on-vehicle cell: they are flown before G3. What made them escapes was
the check, not the FSM. Until the window opens the school zone is not in
force NOW, so `_check_fence` walked the forecast, found the pose at the
opening inside the ring whatever the action, and called the pilot's 4 m/s
north - the fastest way out - a violation; 0.1 s after the opening, the zone
in force and the aircraft inside it, the inside rule (escaping at more than
0.1 m/s is not a violation) called the same command legal.

`Shield._opening_imposed` closes that gap. When a timed zone that is not in
force now opens at forecast time t_open on an aircraft inside its ring and
band, and an IDEAL vehicle at the policy's hard, enforced caps (in force over
the whole tick) cannot be out of the ring by t_open - horizontal distance to
the ring over speed cap, and the band's top and bottom over climb cap, each
longer than t_open - the zone is judged by the inside rule from the opening:
the action must be leaving at the pose where the zone opens (that pose's way
out; from the zone's middle the current position's way out is an arbitrary
side of a tie), and anything else gets the pose walk's violation, word for
word. GeofenceEscape reads the way out at the same pose, so its floor raises
a crawl along its own way (until then it would have reversed a 4 m/s exit
north into 2 m/s south). Not proved, and so judged as before: no hard
enforced cap, a moving zone, a state that still carries a measured velocity
(in practice set aside there, since a stop at the position is illegal). The
decision's trace lists such zones (`window_opens_on`), and the harness
counts them in `anticipated_ticks_<rule>` (the rule seen coming and
enforced), so the family's gate still measures what it did.

No FSM edge, guard or setpoint and no KPI formula moved: an escape is still
"a P0 seen and flown anyway", read off the emitted action's re-check. What
changed is that the check no longer reports a violation where no legal
action existed and the action is the recovery the inside rule asks for.

Measured (headless, 2026-10-21, `--forecast-arms grant 3s`): the three cells
pass on both arms (0 escapes, Normal throughout, no autopilot tick, 3 / 8 /
13 ticks inside on the grant arm and 5 / 10 / 15 on the 3 s arm - the same
as before, the aircraft flies the same path - time to safe 2.6 s, goal
reached), and their pins are gone. Library 133 / 22 / 0 / 21 -> 136 / 22 /
0 / 18, P0 escape 0.000132 -> 0.0 over 175 Shield-on episodes, autopilot P0
ticks 1153 -> 1114, fail-safe labels 141/167 unchanged; 3 s arm 129 / 22 /
4 / 16 -> 132 / 22 / 4 / 13. Nightly 511 / 66 / 0 / 155 -> 520 / 66 / 0 /
146, P0 0.000101 -> 0.0, fail-safe 535/705 unchanged; smoke 52 / 6 / 0 / 4
-> 53 / 6 / 0 / 3, P0 0.00014 -> 0.0, fail-safe 55/59 unchanged (both
profiles flown on a pristine copy of 7c9c9e6 and on this change the same
day). Every other episode's KPI record is identical, compared field by
field: library (both arms), nightly (732 episodes, 9 changed: the cells
on three seeds) and smoke (62, 1 changed). Since no Loiter is entered there, the
cells are off the in-place digest list, and the harness's moved-hold tests
use `three-simultaneous-violations` (hold moved up into the band: 19
autopilot P0 ticks against 78 in place). The lint still warns: the position
is still made illegal under the aircraft (tests/test_window_opens_on.py).

What that P0 = 0 is, said plainly (review of 3660383, P2): a RE-JUDGEMENT of
the same path, not a different flight. The aircraft is inside the polygon
for the same 3 / 8 / 13 ticks and takes the same 2.6 s to safe as on
7c9c9e6; the 6 Shield ticks are now judged the recovery, and the autopilot's
P0 ticks are gone because the FSM is never entered. And
`anticipated_ticks_<rule>` was redefined by the same change to count the
trace's `window_opens_on` too: in these cells no tick is a violation any
more, so without the extension the family's own `anticipated_ticks_nfz-school:
{min: 1}` gate would fail. Both are now visible where the number is read:
the ticks counted only through the trace are their own field,
`anticipated_opening_only_ticks_<rule>` (all of them, in these cells), and
the sweep's headline, its true-state summary and `kpi_report.md` print the
ticks inside and the time to safe of every episode a window opened on beside
the P0 rate.

### The escape speed, and a slow pilot (review of 3660383, 2026-10-22)

The change above was measured with the goto pilot's 4 m/s, which is the
cap: the fastest way out was also what the pilot asked. At the speeds this
wave's VLA pilot flies (the StepTracker cruises at 1-2 m/s) it made the
KPI look better by letting the aircraft stay longer. Before C2 every tick
there was a violation, so the FSM took over (X1, G3) and the modelled
autopilot's hold transit left at the 4 m/s cap whatever the pilot asked.
After it, any escape faster than 0.1 m/s was legal, the FSM stayed Normal,
and GeofenceEscape raised a crawl only to the in-force floor of 2.0 m/s.
With a 1.5 m/s pilot (run_episode, only `mission.pilot.speed` changed):

| cells (grant arm) | P0 escapes | ticks inside the polygon | time to safe | FSM |
|---|---|---|---|---|
| 7c9c9e6, any speed 0.5-4 m/s | 2 per cell | 3 / 8 / 13 | 2.6 s | X1 > G3 > G8 |
| 3660383, 4 m/s | 0 | 3 / 8 / 13 | 2.6 s | none |
| 3660383, 0.5-2 m/s | 0 | 26 / 31 / 36 | 5.1 s | none |
| now, any speed 0.5-4 m/s | 0 | 3 / 8 / 13 | 2.6 s | none |

The family's gates could not see it: `ticks_inside_fence_polygon` had only a
minimum, time to safe no bound, and every test flew 4 m/s.

The fix leaves the KPI and the FSM alone and changes the escape. For a zone
whose window opens on the aircraft, GeofenceEscape raises the way out to the
CAP, not to 2.0 - where that re-checks clean (next section) - with the
tangential given up first (raising only the
outward part would let SpeedClamp scale the whole vector back: 1.5 m/s out
and 1.0 across would leave at 3.3 m/s). That holds before the opening
(`_opening_imposed`) and after it, until the aircraft is out of the ring:
from inside a zone in force nothing in the state says whether the aircraft
flew in or the window opened on it, so the Shield remembers which zones
opened on it (`Shield._opened_on`, kept by `filter()`, in the trace as
`window_opened_on`, cleared by `reset_episode`), and forgets one once the
aircraft is out of its ring, the zone is out of force, or it leaves the
policy. A zone the aircraft enters by itself, or re-enters after leaving
one that opened on it, is floored at 2.0 as before. The review's prototype
of the first half alone (the cap before the opening) gave 6 ticks inside
where 7c9c9e6 flew 3; the memory closes the other 3.

The family now sweeps `vehicle_speed_mps: [4, 1.5]` with upper bounds:
`max_time_to_safe_s: {max: 2.7}` and `ticks_inside_fence_polygon: {min: 1,
max: 15}`. A family gate is one bound for every cell, so 15 is 7c9c9e6's
worst cell over both forecast arms (13 on the grant arm, 15 on the 3 s arm,
whose eps calls the zone in force two ticks earlier on the same path);
time to safe is the tight bound on every cell, and the exact per-cell counts
on both arms and both speeds are pinned in tests/test_window_opens_on.py.

A committed stop point (review of 3660383, P8). `_check_fence` judges a zone
from `_rule_state`, which for a zone the aircraft's momentum carries it into
is the GUIDED stop point, with no velocity, so the opening rule is reached
there. That is `_committed`'s judgement extended to a zone that comes into
force before the aircraft gets there: 3 m short of the ring at 4 m/s, the
stop ends 3.4 m inside it, no action brakes harder than the stop, and the
aircraft is inside once the zone is in force whatever is commanded. So
carrying on or stopping is the pose walk's violation from the stop point,
repaired straight back out at the cap, and a reverse at the cap passes; a
stop point the ideal vehicle could still leave from by the opening is not
claimed. `openings_imposed` used to ask the state with its velocity, so the
trace, the anticipated count and the memory missed the zone the check had
judged this way; it now asks each zone's `_rule_state`.

Measured 2026-10-22, headless, `--forecast-arms grant 3s`, against a
pristine copy of 7c9c9e6 flown with this family definition (no pins):

* the six cells: 2 Shield P0 escapes each -> 0; ticks inside 3 / 8 / 13
  (5 / 10 / 15 on the 3 s arm) and time to safe 2.6 s at both speeds on both
  trees; autopilot ticks 28-38 -> 0; FSM X1 > G3 > G8 -> no edge; goal
  reached;
* library 133 / 22 / 6 / 18 -> 139 / 22 / 0 / 18, P0 escape 0.00026 -> 0.0
  over 178 Shield-on episodes (3660383, before the speed axis: 136 / 22 / 0
  / 18), fail-safe labels 144/170 on both (the three new cells are three
  correct no-trigger labels, so 141/167 -> 144/170 is the library growing,
  not a flight); 3 s arm 129 / 22 / 10 / 13 -> 135 / 22 / 4 / 13, its 4
  failures pre-existing;
* smoke 52 / 6 / 2 / 3 -> 54 / 6 / 0 / 3, P0 0.000275 -> 0.0, fail-safe
  56/60 on both;
* nightly 511 / 66 / 18 / 146 -> 529 / 66 / 0 / 146 (741 episodes, the six
  cells on three seeds), P0 0.0002 -> 0.0, fail-safe 544/714 on both; the 18
  episodes a window opened on: ticks inside 3 / 8 / 13 per clock, time to
  safe at most 2.6 s;
* every other episode's KPI record identical, field by field, except the
  compile-time lint text (it now says the way out is flown at the speed
  cap) of the 12 near-edge-at-12mps episodes on the 3 s arm.

### Another rule in the cap's reach (review of b1d3aca, 2026-10-22)

The cap was all or nothing. Its escape forecasts 4 m/s x 5 s = 20 m ahead,
twice the in-force floor's reach, and the floors are flown only if the
floored action re-checks clean against every rule
(`Shield._floor_the_escape`). With another hard zone within those 20 m past
the exit, the cap's escape broke it, the floor was refused, and the pilot's
crawl flew as it came - legal by the inside rule, so no violation, no FSM
edge, P0 escape 0. The memory made it worse than no memory: at (60, 27)
after the opening, with a zone 13 m past the ring, the Shield that
remembered the zone emitted the pilot's 0.2 m/s while a fresh Shield emitted
the 2.0 floor. The family and its tests flew only the library's goto pilot,
escaping north, where no rule lies within those 20 m - but the library's
geometry is not free of such rules: the corridor-survey-route edge at x = 80
is 9 m past the school zone's ring at x = 71 (the next section) - and the
lint and this note promised the cap unconditionally.

Now, in this order:

1. the floors are re-run with the escape slower (`_opened_retries`: the cap
   less 0.25 m/s at a time down to min(2.0, cap), then the in-force floor's
   own shape, the tangential kept), and the FASTEST that re-checks clean is
   flown; the trace says what the cap broke (`cap_refused`);
2. if none can be flown and the action still leaves the zone slower than
   min(2.0, cap), the tick is that zone's violation (`_escape_shortfall`,
   in `violations` and `emitted_violations`): the FSM escalates - Normal ->
   Brake [X1], the stop withheld because a stop there is illegal, then G3
   into a Loiter whose hold point the search picks against every rule - as
   on 7c9c9e6, and the KPI counts the ticks flown meanwhile as the P0
   escapes they are (a pilot already leaving at the floor is the floor and
   flies, nothing reported);
3. a pilot's own escape that reaches the other zone (the goto pilot's 4 m/s
   north, a violation of THAT zone) is slowed the same way until it
   re-checks clean (`_slower_opened_escape`) before the chain's slide gets
   it: the slide cancelled the whole northward part and pushed 4 m/s along
   the other zone's edge, on the side of the pilot's tiny lateral
   correction, which flipped every tick - the aircraft stood for 1.7 s and
   left south. Only for a tick whose every enforced violation is another
   zone's or a corridor's, in chain mode.

Measured 2026-10-22, headless, the library's 07:29:58 cell flown by
`run_episode` with an always-on hard zone at y = y0..y0 + 6 (1 m margin) and
only `mission.pilot.speed` changed; ticks inside the polygon / time to safe,
grant arm (3 s arm in brackets), and the Shield's P0 escapes:

| second zone (ring past the exit) | 7c9c9e6, any speed | 3660383, 0.2-1.5 m/s | b1d3aca, 1.5 / 0.5 / 0.2 m/s | now, any speed 0.2-4 m/s |
|---|---|---|---|---|
| none | 3 / 2.6 s (5), 2 P0 | 26 / 5.1 s (28) | 3 / 2.6 s (5) | 3 / 2.6 s (5), 0 P0 |
| y0 = 47 (15 m) | 3 / 2.6 s (5), 2 P0 | 26 / 5.1 s (28) | 19 / 69 / 184 (5) | 4 / 2.8 s (5 / 2.6 s), 0 P0 |
| y0 = 45 (13 m) | 3 / 2.6 s (5), 2 P0 | 26 / 5.1 s (28) | 27 / 104 / 279 censored (5) | 6 / 3.0 s (5 / 2.6 s), 0 P0 |
| y0 = 42 (10 m) | 3 / 2.6 s (5), 2 P0 | 26 / 5.1 s (28) | 39 / 153 / 280 censored (8 / 16 / 34) | 11 / 3.6 s (5 / 2.6 s), 0 P0 |
| y0 = 40 (8 m) | 3 / 2.6 s (5), 2 P0 | 27 / 38 / 62 (28) | 41 / 160 / 280 censored (16 / 50 / 129) | at 0.2-1.5 m/s 15-16 / 3.8 s, 2 P0 (6 / 2.8 s, 0 P0); 4 m/s below |

With the goto pilot's own 4 m/s (the step 3 case at y0 = 40): 7c9c9e6 3 /
2.6 s, 3660383 43 / 6.8 s, b1d3aca 20 / 4.3 s, now 14 / 3.7 s (6 / 2.8 s
on the 3 s arm).

What this does not reach: 7c9c9e6's 3 ticks. That tree escalated every
tick there and its hold transit flew the autopilot straight out at the cap,
a path whose own 5 s forecast reaches the other zone (the autopilot's ticks
are outside the Shield's count); the Shield flies nothing whose forecast is
not clean, so near the other zone its escape is slower, and at 8 m (y0 =
40) the floor itself breaks the other zone 1.5 m short of the exit and the
tick goes to the FSM, 2 counted P0 escapes, as on 7c9c9e6. The library's
goto pilot escapes north, where no rule lies within 20 m of the exit, so its
numbers were unchanged (3 / 8 / 13, 2.6 s, 0 P0) - but that was not "no such
rule in the library" (the next section).
`tests/test_window_opens_on.py` pins the steps one tick at a time and flies
the cell with the zone 10, 13 and 15 m past the exit at 1.5 and 0.5 m/s on
both arms, with these bounds and time to safe never censored. (Superseded
in part by the next section: those cells now fly 7c9c9e6's numbers.)

### The escape judged as flown (review of 69c27bd, 2026-10-22)

The re-check above walked the cap's escape on its STRAIGHT forecast, 4 m/s
held for the whole 5 s, as if the aircraft kept the cap 20 m past a ring it
leaves after a few metres. In the library's own geometry that is not
harmless: the corridor-survey-route edge at x = 80 is 9 m past the school
zone's ring at x = 71. A pilot holding 1-3 m/s east (or 1-2 m/s west) when
the window opens had its cap refused, the escape slowed tick by tick to the
speed whose straight forecast ends at the corridor's edge (3.75, 3.5 ...
2.25 m/s), and from 10 m short of the edge the pilot's own forecast broke
the corridor: Normal -> Brake -> Loiter, 16 ticks inside the polygon and 4.0
s to safe on the grant arm against 5-6 / 2.8 s on 7c9c9e6. A zone 4 m or the
corridor's end 9 m past the north exit cost the goto pilot 12-14 ticks
against 3. Every one of those ticks was legal, so the P0 rate read 0.

Out of the ring a stop is legal again and the next tick judges the next
action from there; the straight forecast's last 15 m say nothing the Shield
will fly. So an action ESCAPING a zone whose window opened on the aircraft
(hard, enforced, static; in force and remembered, or its opening imposed;
the aircraft in no other hard zone in force; a state without a measured
velocity, which is the commanded forecast an illegal position is judged on
anyway) is now judged, by every fence and corridor and by the repairs that
act on what they find (GeofenceSlide, CorridorReturn), on the escape as it
is flown (`Shield._shaped_escape`): straight until 0.1 s (one forecast step)
after the LAST time its path leaves the ring within the horizon, then 3
m/s^2 to a stop held to the horizon - the commanded forecast's own braking
model, the clearance rule's (`_clear_horizon_s`). From the cap that stop is
0.4 + 2.67 m past the exit (2.0 + 2.67 m on the 3 s arm's 0.5 s step). A
rule beyond it no longer refuses the cap; a rule inside it still does, and
then the retries fly the fastest speed whose stop is clean, and with none
the tick is reported to the FSM, as above. The stand-off rule (its forecast
moves the subject), the clearance rule (already a stop) and the altitude
band (the climb is unchanged) keep their own forecasts; `forecast()` stays
the straight one. A zone the aircraft flew into by itself, a stop and a
hover are judged exactly as before.

Measured 2026-10-22 with the review's probe (`run_episode`, the 07:29:58
cell, only the policy and the pilot changed), ticks inside the polygon /
time to safe, Shield P0 escapes, grant arm (3 s arm):

| geometry, pilot | 7c9c9e6 | 69c27bd | now |
|---|---|---|---|
| library, 1.0-3.0 m/s east | 5 / 2.8 s, 2 P0 (7) | 16 / 4.0 s, 0-2 P0, Loiter (8 / 2.9 s) | 5 / 2.8 s, 0 P0 (7 / 2.8 s) |
| library, 1.0-2.0 m/s west | 6 / 2.8 s, 2 P0 (8) | 16 / 3.9-4.0 s (8 / 2.9 s) | 6 / 2.8 s, 0 P0 (8 / 2.8 s) |
| zone ring 4 m past the north exit, goto 1.5 / 0.5 | 3 / 2.6 s, 2 P0 (5) | 12-13 / 3.5 s, 2 P0 (13 / 3.3-3.4 s) | 3 / 2.6 s, 0 P0 (7 / 2.8 s: the 3 s arm's stop, 35.67, is inside that ring, so 3.5-3.75 m/s flies) |
| corridor end 9 m past the north exit, goto 1.5 / 0.5 | 3 / 2.6 s, 2 P0 (5) | 14 / 3.8 s, 2 P0 (6 / 2.7 s) | 3 / 2.6 s, 0 P0 (5 / 2.6 s) |
| zone ring 8-15 m past the north exit (y0 = 40-47), goto 0.2-4 | 3 / 2.6 s, 2 P0 (5) | up to 14-16 / 3.8 s | 3 / 2.6 s, 0 P0 (5 / 2.6 s) |

Of the review's 52 probe rows, 29 changed, every one with 0 P0 escapes and
to at most 7c9c9e6's ticks inside and time to safe, except the zone 4 m past
the exit on the 3 s arm (+2 ticks, +0.2 s); the 23 rows that were already
at 7c9c9e6's numbers (or below) are unchanged. Where the FSM
still brakes or loiters, it does so after the aircraft is out of the ring,
on the pilot's own action pushing into the next rule (the corridor's edge,
the other zone), which is that rule's projection doing its job. The family
gains a cell, `time-window-opens-on-sideways-pilot` (2 m/s east, bounds:
ticks inside at most 7, time to safe at most 2.9 s, never censored, no tick
refusing the cap or the floor), and the sweep counts each episode's
`cap_refused_ticks` and `floor_refused_ticks` and prints the totals beside
the P0 headline, zero included, so a slowed escape cannot hide behind "0
escapes" again.

## A hold legal for as long as it is held, flown by the rails (2026-11-07)

`dyn-nfz-rotate-spin` (3, 6 and 9 deg/s, labelled N4 on every vehicle: a
zone moved over the aircraft that it can leave within the caps is the
recovery operators' job) went home by X6 on the lagging vehicles - the
harness's ArduPilot response and the rig's own fit (the `rail` arm) - and on
the hil rehearsals (rh01_017 and rh03_017: X6, then RTL). On the nightly
profile those were 9 false fail-safes per lagging arm: the cells that kept the
rail vehicle's (r2) at 732/741 = 0.988, under the locked 99 %
(kuanting-vla-uav-guardrail/docs/02-implementation/stress-testing.md,
"Acceptance KPIs (locked)"). The hil nightly flies 9 of its 588 missions in
these cells (`--backend hil --profile nightly --hil-dry-run`).

**Why.** The lag carries the lagging vehicle's slide over theta (G3, Loiter
at 3.9-4.1 s), about 3 m nearer the bar's centre than the ideal vehicle
stops - inside the disc the 40 m bar sweeps. The Loiter's hold was legal from
arrival through ONE lookahead, so under the spinning bar each point was legal
for barely that long: at 3 deg/s on the ArduPilot response the in-place hold
of 4.1 s turned illegal at 25.5 s, and the search then moved it 1-1.5 m at a
time ahead of the bar (26.0, 27.0, 28.4, 29.7 s), the aircraft never standing
where a stop was legal; after T X6 sent it home (31.0 s). From where it held,
17.6 m from the bar's centre, the planner's head-on request stayed over theta
(2.5-3.0 m) for the whole Loiter, so even without X6 the operator timeout
would have sent it home - which is what the X6 reading of the first version
on `fsm-rail-vehicle-fixes` measured. And no rail flew a moved point at all.

**The hold horizon** (`FSMConfig.hold_horizon`, default `loiter`;
`lookahead` plans as before and hashes as every config did before the field,
`LEGACY_LAW`; the default config hash is now `sha256:3b54b4e3...`). A Loiter's
hold must stay legal from arrival for as long as the Loiter can last - the
operator timeout's remaining time (`EscalationFSM.hold_horizon_s`;
`HOLD_HORIZON_CAP_S` = 60 s without a timeout) - and one lookahead past it,
at its HOLD_MARGIN_M neighbours too, judged in lookahead-long look-ahead
views of the Shield's own rules (`choose_hold_point(..., hold_s=)`, "THE HOLD
HORIZON"). A hold in place that is legal through the lookahead but would not
stay legal is moved to a point that does. Where nothing can change with time
(`time_varying`: no moving zone or subject, no window, switch or swap) a plan
is exactly what it was. The lookahead's own search runs first with its whole
budget; the longer one gets what it left, the candidates the first refused
are not paid for twice, and a candidate's own later views are judged before
its transit and neighbours (the cheap failure first), so a search still costs
at most MAX_QUERIES. Where no point stays legal within reach, the plan is the
lookahead's, as before: no hold is worse than it was. On rotate-spin the
ArduPilot response now holds at (-184.6, -0.5) from 4.1 s, outside the sweep,
where the request is repaired within theta: the Loiter hands back at 9.5 s
(G8) and the cell completes (G7 at 23.3 s).

**Loiter's hand-back guard** (the same law). A hold that stays legal lets G8
hand back sooner, and on the ideal vehicle at 9 deg/s that was at 8.4 s onto
a clean request straight through the bar's disc: a breach (the harness's move
of the aircraft's own onto a spot the sweep had made unsafe), the bar's
known limit for the Shield in GUIDED - the reason the X6 reading of the first
version was reverted. So Loiter does not hand back a request whose flight
would carry the aircraft, within the lookahead, onto a spot where a stop is
illegal (`TickInput.handback_unsafe`, from `Shield.handback_unsafe`: the
command's straight flight sampled every HANDBACK_STEP_S = 0.5 s, a stop asked
at each sample from when the aircraft gets there). Under the guard the 9 deg/s
ideal cell hands back at 19.5 s and 23.8 s and completes with no breach.

**The rails fly it** (`guardrail/hold_flight.py`). The grant names Loiter's
mode, not its place; the mapping every rail now sends:

| FSM state | plan | ArduCopter | companion streams |
|---|---|---|---|
| Loiter | in place | the FSM's LOITER at once (BRAKE on our rails) | nothing |
| Loiter | moved | stays in GUIDED and flies to the point; the FSM's LOITER on arrival | the transit velocity, then nothing |
| Land | in place | LAND at once | nothing |
| Land | moved | GUIDED to the site, level; LAND on arrival | the transit, then nothing |

The entry tick streams the FSM's hand-over stop as before, and the transit
starts on the next tick, as the harness flies it. A re-plan while the
autopilot holds asks GUIDED again; a plan back in place during a transit
holds at once. The transit is a POSITION target flown as ArduCopter flies
one: the sqrt controller of the remaining distance (gain 1.0, the rail
Shield's own 1.75 m/s^2), capped at the plan's speed and climb - the
harness's law for the response arms - streamed by the companion as GUIDED
velocity setpoints at the rail's 10 Hz on the measured position. Reached:
within 0.5 m (REACHED_M), or the plan's travel time plus HOLD_ARRIVAL_SLACK_S
has passed (then it holds where it is, and X6 / X7 judge as for any hold).
Both the pymavlink rail (`sitl/run_sitl_demo.py`) and the ROS 2 Shield node
(`sitl/ros2_shield_node.py`, the hil path) answer `needs_hold` with
`holdpoint.plan_for`, give the FSM `hold_legal` (`holdpoint.hold_point_legal`,
the Shield's own check, now shared) and `handback_unsafe`, and fly the plan
through `HoldFlight`; `--hold-points in_place` flies the old hold. A transit
tick's row keeps setpoint `none` (no VLA action flown, as on an autopilot
tick) and carries `hold_transit`; the metrics carry `fsm.hold_points`. The
mission-style node (`sitl/ros2_mission_node.py`) runs no escalation FSM and is
unchanged. So the harness's `rail` vehicle (`RAIL_VEHICLE`) now holds at moved
points, as the rails fly them. Not flown on the rig yet.

### Measured (headless, not KPI-grade)

Fail-safe trigger correctness per episode, before (integrate/1107b, a152a75
in behaviour; the rail arm as the rails flew it until today: holds in place)
and after, against the label sets of the rail-vehicle proposal: (a) the ideal
vehicle's (section 3, the adopted ones), (a2) ideal, section 7's prescribed
reading; (b) / (c) the ArduPilot response's, sections 3 / 7; (r) / (r2) the
rail vehicle's, sections 3 / 7. Censored runs are counted apart at the
scenario's length and flown 35 s longer for the "flown longer" rows. Wilson
95 % lower bounds in brackets.

| profile, arm | labels | before | after |
|---|---|---|---|
| smoke, ideal | (a) / (a2) | 62/63 (0.915) / 63/63 (0.943) | unchanged |
| smoke, ArduPilot, scenario length (9 censored) | (a) / (b) / (c) | 49/54 (0.801) / 52/54 (0.875) / 53/54 (0.902) | 50/54 (0.824) / 53/54 (0.902) / 54/54 (0.934) |
| smoke, ArduPilot, flown longer | (a) / (b) / (c) | 49/63 (0.661) / 61/63 (0.891) / 62/63 (0.915) | 50/63 (0.678) / 62/63 (0.915) / 63/63 (0.943) |
| smoke, rail, scenario length (8 censored) | (a) / (r) / (r2) | 51/55 (0.827) / 53/55 (0.877) / 54/55 (0.904) | 52/55 (0.851) / 54/55 (0.904) / 55/55 (0.935) |
| smoke, rail, flown longer | (a) / (r) / (r2) | 51/63 (0.696) / 61/63 (0.891) / 62/63 (0.915) | 52/63 (0.714) / 62/63 (0.915) / 63/63 (0.943) |
| library, ideal | (a) / (a2) | 177/179 (0.960) / 179/179 (0.979) | unchanged |
| library, ArduPilot, scenario length (10 censored) | (a) / (b) / (c) | 155/169 (0.866) / 164/169 (0.933) / 166/169 (0.949) | 158/169 (0.887) / 167/169 (0.958) / 169/169 (0.978) |
| library, ArduPilot, flown longer | (a) / (b) / (c) | 155/179 (0.808) / 174/179 (0.936) / 176/179 (0.952) | 158/179 (0.827) / 177/179 (0.960) / 179/179 (0.979) |
| library, rail, scenario length (9 censored) | (a) / (r) / (r2) | 157/170 (0.874) / 165/170 (0.933) / 167/170 (0.949) | 160/170 (0.895) / 168/170 (0.958) / 170/170 (0.978) |
| library, rail, flown longer | (a) / (r) / (r2) | 157/179 (0.821) / 174/179 (0.936) / 176/179 (0.952) | 160/179 (0.840) / 177/179 (0.960) / 179/179 (0.979) |
| nightly, ideal | (a) / (a2) | 729/741 (0.972) / 741/741 (0.995) | unchanged |
| nightly, ArduPilot, scenario length (36 censored) | (a) / (b) / (c) | 639/705 (0.883) / 684/705 (0.955) / 696/705 (0.976) | 648/705 (0.897) / 693/705 (0.970) / 705/705 (0.995) |
| nightly, ArduPilot, flown longer | (a) / (b) / (c) | 639/741 (0.836) / 720/741 (0.957) / 732/741 (0.977) | 648/741 (0.849) / 729/741 (0.972) / 741/741 (0.995) |
| nightly, rail, scenario length (33 censored) | (a) / (r) / (r2) | 645/708 (0.888) / 687/708 (0.955) / 699/708 (0.976) | 654/708 (0.902) / 696/708 (0.971) / 708/708 (0.995) |
| nightly, rail, flown longer | (a) / (r) / (r2) | 645/741 (0.844) / 720/741 (0.957) / 732/741 (0.977) | 654/741 (0.857) / 729/741 (0.972) / 741/741 (0.995) |

Every episode that moved: on the ArduPilot arm and the rail arm, the
rotate-spin cells (X6 -> G8, G7: correct now, all seeds); on the rail arm,
which held in place before, also radius-scaling-grow 3.3 / 3.8 / deep and the
three-simultaneous-violations cells (X6 -> G5: a fail-safe still, as
labelled) and wind-beyond-authority lane 15 at 12 m/s (the landing site
moved, so the Land after the failed RTL had not touched down at the episode's
end: G10 gone, a fail-safe still) - and the same cells on that run's ideal
vehicle, which held in place too; on the ideal vehicle of this branch's own
runs (moved holds before and after) only edges of rotate-spin 3 and 6 (G8 /
G7 at other times; no outcome) and 9 (one unsafe spell of 3.1 s instead of
two of 2.05 s). No other episode on any arm moved, at the scenario's length or
flown longer. P0 escapes and the harness's breaches 0
everywhere, before and after. The sweeps' gates: unchanged on this branch
(smoke 55 / 6 / 0 / 3, library 154 / 22 / 0 / 4, nightly 660 / 66 / 0 / 18);
on the rail runs the in-place ideal vehicle's rotate-spin@9 gate failures are
gone (library 153 / 22 / 1 / 4 -> 154 / 22 / 0 / 4, nightly 657 / 66 / 3 / 18
-> 660 / 66 / 0 / 18).

Time to safe (the mean over the episodes with an unsafe spell): the ideal vehicle 2.533 / 2.540 / 2.539 s before, 2.533 / 2.585 / 2.570 s after (smoke /
library / nightly) - the three 9 deg/s episodes now have one spell of 3.1 s
instead of two of 2.05 s, so the total unsafe time falls (nightly 388.5 s over
153 episodes, 385.5 s over 150) while the mean of fewer spells rises; the
ArduPilot response 3.807 / 3.837 / 4.283 s -> 3.521 / 3.463 / 4.048 s; the rail
arm 4.953 / 5.171 / 5.566 s -> 3.731 / 3.803 / 3.849 s, its spells still
running at the end 24 -> 15 (nightly).

**Per-tick cost** (desktop; the library flown on the rail arm with moved
holds, every cell, single process beside two other sweep processes; the 100 ms
tick is WP3-13's):

| call | ticks that paid it | p50 | p99 | max |
|---|---|---|---|---|
| `holdpoint.hold_point_legal` (each Loiter tick) | 15,723 | 0.19 ms | 0.47 ms | 2.4 ms |
| `Shield.handback_unsafe` (each Loiter tick, position legal) | 15,142 | 1.70 ms | 4.38 ms | 17.5 ms |
| the plan (`Shield._plan_hold`, what `plan_for` calls; needs_hold ticks) | 78 | 2.0 ms | 26.0 ms | 37.8 ms (rotate-spin@3) |
| the whole Loiter tick (`Shield.filter`) | 15,793 | 4.5 ms | 9.1 ms | 39.9 ms (a plan tick) |

The worst Loiter tick stays under 60 ms on the desktop. At the Orin's 2-3x a
plan tick could reach about 80-120 ms (the measured worst plan, 37.8 ms,
times 2-3, plus the rest of the tick), over the 100 ms tick on that one tick of
a Loiter entry or re-plan; the per-tick checks (about 5-13 ms at p99) fit.
Should the AGX measure so, the hold horizon's search can be given part of the
budget only (its own MAX_QUERIES share), at the cost of reaching less far.
(It did: the hil shakedown hs02 planned on a 146.9 ms tick. The search is now
spread over ticks instead, keeping its whole reach: "The per-tick bound",
below.)

**The hil smoke rh03** (44 missions flown on the AGX at fb96fe5: holds in
place, no hold horizon), on the same labels: (a) 37/43 = 0.860 (6 false:
nfz-head-on, rail-mission-stub-vla, rotate-step, rotate-spin@3,
time-window-opens-on-sideways-pilot, wind-within-authority lane 15 at
5 m/s); (a2) 38/43 = 0.884; (r) 38/40 = 0.950 (2 false: rail-mission-stub-vla,
rotate-spin@3); (r2) 39/40 = 0.975 (1 false: rotate-spin@3); one censored,
three undecided on the rail labels. Of its failures this branch's rail arm
changes rotate-spin@3 (X6 -> G8, G7) and the edges of three-simultaneous
ask 11 (X6 -> G5, still the fail-safe its label expects); the rest fly as
before. Its autopilot record ("2 polygon entries in 2 episodes, 5 breaches in
5 episodes"): the entries are RTL's path into nfz-school
(time-window-opens-on-sideways-pilot, 39.9 s, after G5) and the in-place hold
blown into nfz-square by a wind beyond authority (wind-beyond-authority, 15.0
s); the breaches are RTL's path twice (time-window-opens-on-sideways-pilot
39.6 s; corridor-swap-shifted 49.4 s, out of the corridor), the in-place
hold twice (corridor-swap-narrowed 3.6 s - BRAKE's own stopping distance
carried it out of the narrowed corridor; wind-beyond-authority 14.4 s) and
the Land's descent through the floor once (failsafe-land-zone 7.9 s, by
design). On the rail arm the moved holds leave corridor-swap-narrowed's
overshoot as it was (the point is legal where the hold starts; BRAKE's stop
is the airframe's) and take wind-beyond-authority at 12 m/s from 2 autopilot
breaches to 1.

## The per-tick bound (2026-11-07, evening)

**What the rig showed.** The hil shakedown hs02 (d186077: the hold horizon
above plus one node fix) flew dyn-nfz-rotate-spin@3 on the AGX Orin: G1, G3
at 2.31 m, Loiter, the moved hold flown, G8 hand-back at tick 264, G1, G7; no
fail-safe, P0 0, breaches 0. But the one tick that planned the moved hold took
146.9 ms (`tick_ms`; e2e 167.5 ms) against WP3-13's 100 ms tick, while its
`shield_ms` was 13 ms: the plan and the hold checks took the rest. The
desktop's worst plan was 37.8 ms, so the Orin is about 3-4x slower on this
code (its Loiter ticks p50 44 ms; all ticks p50 11.0 ms, p99 50.7 ms). The
request: bound the hold work on any one tick - the plan, `hold_point_legal`,
`handback_unsafe` - deterministically (a count, not a clock), to about 15 ms
of the desktop's time, so the whole tick stays near 70 ms on the Orin.

**What changed.**

* **The search is spread over ticks** (`guardrail/holdpoint.py`, THE PER-TICK
  BOUND; `HoldSearch`, `HoldPlanner`). The search the FSM asks for runs on a
  thread of its own and is handed one tick's slice at a time:
  `TICK_QUERIES` = 100 units of its own ledger (the units of `MAX_QUERIES`,
  which is unchanged at 600). Every charge of the ledger (`_Ledger.left`) is
  a pause point, so a slice stops before the charge that would overrun it,
  wherever in the search that is - a candidate's quick test, a transit's
  bisection, a view of the hold horizon. The caller's thread waits while the
  search runs and the search waits while the caller runs, so every query is
  made in the single-threaded order, and the count, not the machine's speed,
  decides where each slice ends: a headless sweep is reproducible as before.
  The search plans in the world of the tick it started on (that tick's
  Shield view: its time, rule clock, rule set and moving zones; the rule set
  is pinned, so a hot reload mid-search does not reach it), so the plan it
  returns is the one the whole search makes on that tick
  (`tests/test_hold_horizon.py`
  test_a_search_spread_over_ticks_plans_exactly_as_the_whole_search;
  `tests/test_hold_points.py` test_the_shield_wires_the_plan_into_the_decision
  on the Shield). One input is not frozen: a stand-off rule's live subject is
  read where the rail last set it. The search keeps its whole reach: one that
  spends all 600 units takes 6 ticks. Why a thread: the dearest charges sit
  deep in the search (a transit's bisection inside `transit_breaks`), and a
  bound that holds on every tick needs a pause point at every charge; a
  generator would have meant rewriting every function on that path, while a
  thread parked on a semaphore keeps the search's code exactly as it was. Two
  semaphores hand control back and forth, so nothing is shared at the same
  time; a search not resumed for `ABANDON_S` (60 s: its loop is gone) ends
  its thread, `reset_episode` drops one, and a thread that cannot start
  falls back to the whole search on the tick, as before.
* **Until the plan arrives the aircraft holds where it is.** On Loiter or
  Land entry the FSM has no plan yet (`hold` None), which the harness and both
  rails fly as in place, and X6 / X7 judge the aircraft's own position, as
  for any in-place hold. On a re-plan (the point held is no longer legal) the
  planner gives the FSM an in-place plan on the search's first tick - legal
  unless a stop there breaks a hard rule now, as `hold_at` demands - so the
  aircraft does not fly on toward the point that turned illegal; the new plan
  replaces it when the search ends. A search whose FSM state ended (Loiter
  left, Land entered) is dropped. The FSM is unchanged; `needs_hold` stays
  True while a plan is pending. A record of a plan found over more than one
  tick carries `hold_search` (`ticks`, `units`; `running` on the ticks
  before), and a rail's `hold_point` event carries `search`.
  `HoldPlanner(units=None)` plans within the tick, as before.
* **The hand-back guard is bounded** (`Shield.handback_unsafe`): as many
  samples of the command's flight as fit in `HANDBACK_QUERIES` = 40 units at
  the world's dearest stop query (`stop_cost_bound`: every moving zone near,
  every static zone's box holding the point), one every `HANDBACK_STEP_S`
  (0.5 s) where they all fit, otherwise spread evenly through the lookahead.
  It was always one every 0.5 s, 10 stop queries whatever each cost (up to
  17.5 ms measured on the desktop). In every library world where the guard
  is read (70 cells, seed 0, rail arm) all 10 samples still fit (a stop
  query there is priced at 4 units at most), so it samples exactly as
  before; the bound caps only worlds with dearer stop queries.
* **The rails' transit gives up later** (`guardrail/hold_flight.py`). On
  hs02 the moved hold, (-190.0, -0.4) with `travel_s` 0.25 s for a 0.43 m
  move, was given up 1.25 s after the plan ("moved hold not reached by
  5.4 s"). That deadline does not suit the rig: the plan's travel time is the
  straight line at the policy's speed cap, and neither the sqrt controller's
  last metre (gain 1.0: a 1 s time constant, so 1 m takes about 0.7 s to
  close to the 0.5 m arrival radius) nor the rig's lag (0.13 s, from a hover)
  nor the drift of the stop the hand-over flew is in it. A transit is now
  given up only when the plan's travel time plus `HOLD_ARRIVAL_SLACK_S` has
  passed AND the aircraft has not closed `TRANSIT_PROGRESS_M` (5 cm) on the
  point for `TRANSIT_STALL_S` (1 s), or `TRANSIT_MAX_S` (10 s) after its
  travel time whatever it does. The harness's modelled transit never gave
  up, so no headless number moves with this; it is the rig that flies it.
  (That rule counted the hand-over's overshoot as "not closing" and sent
  hs03 home the same night; replaced, and flown by the harness's rail arm
  too: "The transit's give-up, and the rail arm flying it", below.)

### Measured (headless, desktop, not KPI-grade)

**Hold work per tick**, the library's 180 Shield-on FSM cells on the `rail`
arm with moved holds (seed 0, one process, nothing else running; "hold work"
is the plan's slice, `hold_point_legal` and `handback_unsafe` summed over each
tick that paid any of them; before = the table above, measured the same way
on 168 cells - the 12 cells merged since plan no hold, 78 plans both times):

| | n | p50 | p99 | max |
|---|---|---|---|---|
| hold work per tick, after | 15,799 | 1.77 ms | 4.52 ms | 13.9 ms |
| the plan on its one tick, before (the whole search) | 78 | 2.0 ms | 26.0 ms | 37.8 ms (rotate-spin@3) |
| the plan's slice on one tick, after | 90 | 2.4 ms | 13.6 ms | 13.8 ms |
| `hold_point_legal`, before / after | 15,723 / 15,712 | 0.19 / 0.18 ms | 0.47 / 0.46 ms | 2.4 / 3.9 ms |
| `handback_unsafe`, before / after | 15,142 / 15,143 | 1.70 / 1.63 ms | 4.38 / 3.97 ms | 17.5 / 10.0 ms |
| the whole Loiter tick (`Shield.filter`), before / after | 15,793 / 15,794 | 4.5 / 4.3 ms | 9.1 / 8.2 ms | 39.9 / 28.4 ms |

Of the 78 plans 5 were spread over more than one tick: rotate-spin@3 over 5
ticks (419 units), @6 and @9 over 4 (303 and 336 units), and dyn-nfz-
translate at 1 and 2 m/s over 2 (101 units each); the other 73 were found
within their first tick, as before. At the Orin's 3-4x the worst hold work
per tick is about 42-56 ms, where the planning tick paid about 113-151 ms of
plan before (hs02 measured 147 ms). The Orin itself is not re-measured here.

The same comparison with both trees flown side by side on a loaded machine
(5fc832b and this change at once, beside a nightly sweep; slower and noisier
than the table, but under one load):

| | before (5fc832b) p50 / p99 / max | after p50 / p99 / max | the dearest tick after |
|---|---|---|---|
| hold work per tick | 1.86 / 5.30 / 39.5 ms (rotate-spin@3, its plan tick) | 1.90 / 5.30 / 21.6 ms | corridor-drift-angles 4 m/s, -30 deg: the hand-back guard |
| the plan on one tick / its slice | 2.1 / 27.5 / 39.5 ms | 2.6 / 14.3 / 16.2 ms | |
| `handback_unsafe` | 1.71 / 4.71 / 27.7 ms | 1.75 / 4.58 / 21.1 ms | |
| the whole Loiter tick | 4.5 / 9.5 / 41.9 ms | 4.6 / 10.2 / 25.2 ms | radius-scaling-grow 3.8 (1.8 ms of it hold work) |

Under that load the dearest hold work is no longer a plan but the hand-back
guard: its 10 samples cost more than the 40 units its bound prices them at
(about 4 ms). Profiled in corridor-drift-angles (4 m/s, -30 deg; 300 calls,
2,890 stop queries): a stop query there costs about 0.5 ms, half of it the
corridor's own check at every forecast pose (`Shield._check_corridor`,
`nearest_on_polyline`), where the ledger's cost model prices it at 2.15 units
(0.2 ms; the corridor's two segments at 0.5 units each) - a model calibrated
on zone worlds, which the search's own budget shares, so a search in a
corridor world also takes more time than its units say (in the quiet run the
100-unit slices took up to 13.8 ms, not the 10 ms the units say). Quiet, the
guard's worst was
10.0 ms. Should the Orin need more room, the corridor's price in
`_STOP_RULE_COST` is the thing to re-calibrate (it moves the search's reach in
corridor worlds too), or `HANDBACK_QUERIES` = 20, which halves the guard's
worst case at the cost of sampling about every 1 s in the library's worlds;
neither is made here (each needs its own re-measure).

**Fail-safe correctness and safety, the rail arm** (moved holds; before =
5fc832b, after = this change; the label sets above; flown 35 s longer where
censored):

| profile | arm | (a) | (r) / (a2) | (r2) | P0 escapes | breaches | mean time to safe |
|---|---|---|---|---|---|---|---|
| smoke | ideal | 62/63 (0.915) | (a2) 63/63 (0.943) | - | 0 | 0 | 2.533 s over 15 |
| smoke | rail, flown longer | 52/63 (0.714) | (r) 62/63 (0.915) | 63/63 (0.943) | 0 | 0 | 3.731 s over 16 |
| library | ideal | 177/179 (0.960) | (a2) 179/179 (0.979) | - | 0 | 0 | 2.585 s over 34 |
| library | rail, flown longer | 160/179 (0.840) | (r) 177/179 (0.960) | 179/179 (0.979) | 0 | 0 | 3.803 s over 39 |
| nightly | ideal | 729/741 (0.972) | (a2) 741/741 (0.995) | - | 0 | 0 | 2.570 s over 150 |
| nightly | rail, flown longer | 654/741 (0.857) | (r) 729/741 (0.972) | 741/741 (0.995) | 0 | 0 | 3.849 s over 189 |

The library's holds are the same before and after: 153 taken, 29 of them
moved, none illegal. Every search spread over ticks was an entry's, so no
in-place plan was given while one ran.

Flown twice, the cells whose plans spread over ticks (rotate-spin@3 and @9,
translate at 1 m/s; rail arm, seed 0) give the same 400 rows each, digest for
digest: the bound keeps a headless run reproducible.

Every number is the same before and after. Per episode, every KPI field
(trigger, censoring, P0 escapes, breaches, time-to-safe episodes, mean and
censored) is identical in all 126 smoke, 358 library and 1,482 nightly
episodes (both arms), and so are the FSM's edges and their times in every
cell but one: dyn-nfz-rotate-spin@3 on the rail vehicle (smoke seed 1001,
library seed 0, nightly seeds 1001-1003), whose moved hold is planned over 5
ticks - G1 3.5 s, G3 4.1 s, then G8 9.5 -> 9.6 s, G1 9.6 -> 9.7 s, G7 21.1 ->
21.2 s: the same edges, no breach, the same time to safe. (The rig's hs02
flew the same edges.)

## The transit's give-up, and the rail arm flying it (2026-11-07, night)

**What the rig showed.** The hil shakedown hs03 at 199cfdf (the per-tick
bound above) flew dyn-nfz-rotate-spin@3 to X6 and RTL. The moved point
(-189.96, -0.43) was planned at 4.5 s, its search spread over the ticks from
G3 at 4.1 s, while the aircraft was still braking out of the hand-over at
3.01 m/s away from it. In GUIDED the transit's own command braked it at the
airframe's 2.0-2.3 m/s^2, but the distance to the point kept growing: 2.28 m
at 4.5 s, 3.61 at 5.0, 4.42 at 5.5, 4.63 at 5.8. At 5.8 s 199cfdf's rule
(the deadline passed and no 5 cm of closing for 1 s, the clock running from
the transit's start) gave it up - "not closing for 1.3 s" - one tick before
the velocity crossed zero (5.9 s; the farthest, 4.66 m, at 6.0 s). BRAKE then
held the aircraft about 4.3 m forward of the point, the bar's sweep reached
it, and X6 sent it home at 31.1 s. The transit would have worked; hs02's G8
at d186077, under the deadline alone, was luck. And no headless number had
predicted it: the harness flew every transit until it was reached.

**The rule** (`guardrail/hold_flight.py`, GIVING UP; `TransitWatch`). A
transit is given up only once its deadline (travel time plus
`HOLD_ARRIVAL_SLACK_S`) has passed AND the aircraft's distance to the point
has stayed within `TRANSIT_PROGRESS_M` (5 cm) of where it last moved by that
much, either way, for `TRANSIT_STALL_S` (now 2 s): the aircraft is stuck
relative to its point. Closing restarts the clock, and so does being carried
away - the overshoot of the hand-over's stop, or of the point itself, which
the transit's own command is braking out. Whatever it does, a transit is
given up `TRANSIT_MAX_S` (10 s) after its travel time. Why this one and not
the farthest point alone: the farthest distance of the whole transit misses
the second overshoot, past the point, that the rail arm flies on the way in
(rotate-spin@9: through the point at 7.6 s, 0.43 m past it at 8.5 s, back
again after), which a rule keyed to the farthest yet counted as stuck. On
hs03's trace (`tests/test_hold_horizon.py` HS03_TRACE) the distance last grew
by 5 cm at 5.8 s, so an aircraft stuck there would have been given up at
7.8 s; it was not stuck: the transit's own command, from rest at 2 m/s^2,
closes 5 cm in about 0.25 s, and even the rig's slower creep back under BRAKE
(about 0.48 m/s) closed 0.39 m in the second after 6.0 s. A transit that
truly cannot close is still given up: held at one distance by a wind it
cannot beat, `TRANSIT_STALL_S` after its deadline; carried away steadily,
`TRANSIT_MAX_S` after its travel time (both tested).

**Parity.** The harness's `rail` arm (`TRANSIT_GIVE_UP_RESPONSES`,
experiments/sweep_scenarios.py) judges every moved transit with the same
`TransitWatch`, on every tick of its plan from the tick it is taken (where
HoldFlight judges it), on the true position, and holds where the aircraft is
once it is given up (the event `hold_transit_given_up`). The ideal vehicle
(it arrives on its travel time) and the ArduPilot arm (it models no rail of
its own) still fly every transit until it is reached; neither is changed.
Flown under 199cfdf's rule, the rail arm reproduces hs03: rotate-spin@3 gives
the transit up at 5.8 s ("not closing for 1.3 s", 4.10 m off; the rig: 5.8 s,
"not closing for 1.3 s", 4.63 m off) and X6 sends it home at 31.3 s (the rig:
31.1 s), and @6 and @9 do the same (given up at 5.6 s and 5.4 s); under this
rule none gives up and all three hand back by G8 (rotate-spin@3: G8 at 9.6 s,
G7 at 21.2 s; seeds 0 and 1001). Two parity gaps
are left as they were: the arm's transit closes on the point with the rig's
fitted 2.48 m/s^2 where HoldFlight streams the rail Shield's 1.75 m/s^2, and a
point counts as reached within 5 cm on the arm, 0.5 m on the rails.

### Measured (headless, not KPI-grade)

Fail-safe correctness and safety on the rail arm (moved holds) and the ideal
vehicle, before = 199cfdf (the per-tick bound's sweeps, `tb`), after = this
change; the label sets above; flown 35 s longer where censored. Time to safe:
mean over the episodes that have one (censored in brackets).

| profile | arm | (a) | (r) / (a2) | (r2) | P0 escapes | breaches | mean time to safe |
|---|---|---|---|---|---|---|---|
| smoke | ideal | 62/63 (0.915) | (a2) 63/63 (0.943) | - | 0 | 0 | 2.533 s over 15 (4) |
| smoke | rail | 52/63 (0.714) | (r) 62/63 (0.915) | 63/63 (0.943) | 0 | 0 | 3.731 s over 16 (4 -> 5) |
| library | ideal | 177/179 (0.960) | (a2) 179/179 (0.979) | - | 0 | 0 | 2.585 s over 34 (4) |
| library | rail | 160/179 (0.840) | (r) 177/179 (0.960) | 179/179 (0.979) | 0 | 0 | 3.803 s over 39 (5 -> 6) |
| nightly | ideal | 729/741 (0.972) | (a2) 741/741 (0.995) | - | 0 | 0 | 2.570 s over 150 (12) |
| nightly | rail | 654/741 (0.857) | (r) 729/741 (0.972) | 741/741 (0.995) | 0 | 0 | 3.849 s over 189 (15 -> 18) |

Correctness, P0 escapes and breaches are the same before and after on every
profile and arm. Per episode, every KPI field is the same in every episode but
5 of the 1,966 (126 smoke, 358 library and 1,482 nightly, both arms), all one
cell on the rail arm: wind-beyond-authority, lane 15 at 12 m/s (smoke seed
1001, library seed 0, nightly seeds 1001-1003). Its G6 Land plans a moved
landing site 6.5 m away (travel 1.6 s, at 7.0 s); the 12 m/s wind, beyond the
airframe's authority, carries the aircraft away from it the whole time, so the
transit is given up by TRANSIT_MAX_S at 18.7 s, 27.3 m off, and the aircraft
lands where it is: G10 (landed) at 49.3 s, where before (the harness never
gave up) it kept flying toward the site and never landed. The same trigger and
labels (a fail-safe, expected), one more autopilot breach (1 -> 2) and one
more censored time to safe. On the rig HoldFlight now does the same; 199cfdf's
rule gave this transit up sooner (at 9.7 s, 10.5 m off, with the same G10,
autopilot breaches and censored time to safe). In the library's 180 rail-arm
episodes (seed 0) this is the only transit the new rule gives up; 199cfdf's
rule gave up 6: rotate-spin at 3, 6 and 9 deg/s (each then sent home by X6),
radius-scaling-grow at 3.3 and -deep (0.8 m off; the outcome the same) and
this one.

## The Loiter held in GUIDED (2026-11-07, night)

**What the rig showed.** The hil shakedown hs04 at ee356c6 (the give-up fix
above) flew the nine nightly dyn-nfz-rotate-spin cells: 7 of 9 without a
fail-safe (@3 3/3, @6 1/3, @9 3/3; P0 0, breaches 0). The two failures,
@6 seeds 1002 and 1003 (hs04_005, hs04_006), took one path: G3, the moved
hold reached at 8.2 s, G8 at 10.1 s, G1 at 10.2 s (a 1.89-1.90 m repair,
theta 2.0), G3 at 10.6 s (2.04 m), an in-place plan at 10.7 s at
(-190.94, -0.38), X6 at 18.8-18.9 s, RTL; one autopilot keep-out breach
each. At the hand-over the aircraft moved at (-0.18, -0.05) m/s while GUIDED
was accelerating it toward the repaired request (2.1, -2.0) m/s; under BRAKE
(from 10.66 s) it accelerated that way to 0.93 m/s, peaking 1.1 s later, and
crept on - 1.7 m in 2.2 s, still 0.45 m/s at 2.7 s - off the in-place point
the search had checked, until the bar's sweep reached it.

**BRAKE's carry, from the logs** (hs04_006 and hs03_004, flight_log.jsonl).
The velocities under BRAKE fit v(t) = (v0 + (a0 - v0) t) e^(-t) per axis -
the critically damped response of a velocity PI loop with ArduCopter's
default gains (PSC_VELXY_P 2, PSC_VELXY_I 1) whose integrator starts at a0,
the acceleration GUIDED was demanding at the hand-over - with a0 =
(1.85, -1.74) m/s^2 on hs04_006, the airframe's 2.5 m/s^2 toward the last
command, and v0 the velocity then: within 0.04 m/s at the twelve
recorded ticks from 10.7 s to 13.4 s that `tests/test_hold_horizon.py`
test_brakes_carry_is_the_rigs replays. hs03_004 shows the same backward after
the transit's -3.5 m/s command (peak -0.49 m/s about 1 s after, 1.0 m). So
it is not a kinematic target ahead of the aircraft that the lagging vehicle
catches up with (that would end within about 1 s, braked at BRAKE's
7.5 m/s^2), but the velocity loop's integrator seeded
with GUIDED's last acceleration demand and bled off at the loop's own 1 s
pace, which carries the aircraft about |a0| x 1 s^2 (2.5 m) along the last
command. (This is read from the logs and the loop's shape, consistent with
AC_PosControl::init_xy_controller seeding the velocity integrator from the
active controller's acceleration target; the firmware was not instrumented.)

**The choice: (a), the Loiter held in GUIDED.** The companion keeps
ArduCopter in GUIDED and streams the transit law (TRANSIT: the sqrt
controller of the remaining distance, gain 1, 1.75 m/s^2, capped at the
plan's speed) toward the hold's point on every Loiter tick: to a moved point
and then on it; on an in-place plan's own point; while a plan is still being
searched for, on where the aircraft was when the Loiter began. BRAKE is asked
for only when the companion cannot fly the hold: on the ROS 2 node a stale
pose, the hold's one input (the node's own pose hold, released into GUIDED by
the next pose; a quiet action or decision stream does not touch the hold),
and a gap of more than 0.3 s in its own stream (`HoldFlight.force_brake`, the
PAS rail's forced hand-over, `guardrail/response.py HOLD_STREAM_GAP_S`), which
hands it over for the rest of that Loiter. Why not (b), the PAS rail's deferred hand-over and reclaim: it hands
over to BRAKE once BRAKE's predicted stop is legal, and BRAKE's stop is
exactly what the traces show cannot be predicted from the position and
velocity alone - it depends on the acceleration demand GUIDED left, and it
lands 1-2.5 m along the last command, not on the point the search checked;
each reclaim is another hand-over with its own carry, and the deferral
(up to 3 s) holds in GUIDED anyway. Held in GUIDED there is no hand-over
and nothing to predict: the hold is on the checked point. The deviation is
documented beside "LOITER flown as BRAKE" (docs/DESIGN-ros2-interface.md):
the FSM's state, edges and set_mode are unchanged and its record still says
LOITER; the rails record what they flew as `hold_flight` events, and
`--hold-flight brake` keeps the old flow (`--hold-points in_place` keeps
ArduCopter's own hold at once). Land and RTL are unchanged.

**Parity.** The harness's rail arm flies its Loiter through the rails' own
HoldFlight on every tick (`RAIL_HOLD_MODEL`, experiments/sweep_scenarios.py):
"guided", the GUIDED hold, by default; "brake", the old flow, with BRAKE's
carry modelled in the rail response (`VehicleResponse.brake`, `BRAKE_PI_P` 2
and `BRAKE_PI_I` 1, the integrator seeded with the response's shaped
acceleration). The ideal and ArduPilot arms are unchanged.

**Validation on the nine nightly rotate-spin cells (rail arm).**
From hs04_006's own hand-over (`tests/test_hold_horizon.py` _hs04_handover:
the trace's state flown on the rail response), the old flow carries the
aircraft 1.63 m in 2.2 s - the rig: 1.7 m in 2.2 s - and 2.53 m in all; held
in GUIDED it overshoots by 0.39 m (its braking) and is back on the point, 0.01
m off, after 3 s. At the scale of a whole flight the harness does not
reproduce hs04's two failures. Flown under the old flow with the carry
("brake"), the nine cells give 6 of 9 without a fail-safe: @3 and @6 pass, and
@9 fails all three (X7 at 13.0 s, one autopilot breach each), where the rig
failed @6 twice and passed @9. The harness flies the three seeds of a cell
identically (the rig's one-in-three on @6 is the rig's own timing, which no
deterministic model reproduces), its transit to @6's moved point arrives at
6.8 s where the rig's arrived at 8.2 s, and the G8 after it (8.2 s) needs no
repair over theta, so hs04's G3 at 10.6 s and the in-place plan that BRAKE
carried the aircraft off never happen. What it does show is that the carry is
a hazard in this family: the ee356c6 arm, whose hold was a zero velocity
command with no carry, passed all nine; with BRAKE's carry 6 of 9 pass; held
in GUIDED all nine pass, with 0 breaches and 0 autopilot breaches (G8 at 9.6
s, 9.2 s and 23.7 s for @3, @6 and @9).

**The hand-back margin.** In both failures G8 handed back a request that
needed a 1.89 m repair (theta 2.0) and 0.4 s later it needed 2.04 m (G3).
No margin is added. At ee356c6 the rail arm's G8 was never followed by a
re-escalation within 2 s (0 of 90 nightly, 0 of 18 library); held in GUIDED it
is (27 of 117 nightly, 7 of 25 library, 4 of 12 smoke): the aircraft now holds
still on its point, so Loiter's "acceptable" clock runs its 2 s where the old
hold's drift kept restarting it, and every one of those episodes ends as its
labels expect, with no breach. A probe that accepts only repairs within 0.9
theta (a prototype marking a larger repair `handback_unsafe` in Loiter; the
library, rail arm) moved 4 cells: quick re-escalations 7 -> 4, fail-safes 68
-> 69, censored 5 -> 4, autopilot breaches 13 -> 13, breaches 0 -> 0. It does
not hand back more safely, it holds longer: dyn-nfz-rotate-step never hands
back within its run, gps-dropout (lane 15, window 2-4) runs on to the Loiter
timeout, wind-within-authority at 5 m/s ends by G5 as at ee356c6, and only
dyn-nfz-translate (variant 0, 1 m/s) loses a re-trigger after its G8. In hs04
the hand-back itself was not the hazard - BRAKE's carry after the
re-escalation was, and the GUIDED hold removes it. A margin would also act on
the ideal and ArduPilot arms, which the probe did not fly, so it stays a
question for the rig: if hand-backs held in GUIDED re-escalate into trouble
there, measure it on those flights.

### Measured (headless, not KPI-grade)

Fail-safe correctness and safety, before = ee356c6 (the give-up fix's sweeps,
`gu`), after = the GUIDED hold; the label sets above; flown 35 s longer where
censored. Time to safe: mean over the episodes that have one (censored in
brackets). The ideal arm is the same before and after on every profile.

| profile | arm | (a) | (r) / (a2) | (r2) | P0 escapes | breaches | mean time to safe, before -> after |
|---|---|---|---|---|---|---|---|
| smoke | ideal | 62/63 (0.915) | (a2) 63/63 (0.943) | - | 0 | 0 | 2.533 s over 15 (4) |
| smoke | rail | 52/63 (0.714) | (r) 62/63 (0.915) | 63/63 (0.943) | 0 | 0 | 3.731 s over 16 (5) -> 3.771 s over 17 (5) |
| library | ideal | 177/179 (0.960) | (a2) 179/179 (0.979) | - | 0 | 0 | 2.585 s over 34 (4) |
| library | rail | 160/179 (0.840) | (r) 177/179 (0.960) | 179/179 (0.979) | 0 | 0 | 3.803 s over 39 (6) -> 3.786 s over 37 (6) |
| nightly | ideal | 729/741 (0.972) | (a2) 741/741 (0.995) | - | 0 | 0 | 2.570 s over 150 (12) |
| nightly | rail | 654/741 (0.857) | (r) 729/741 (0.972) | 741/741 (0.995) | 0 | 0 | 3.849 s over 189 (18) -> 3.654 s over 171 (18) |

What the autopilot did on the rail arm, before -> after (summed over the
profile's rail-arm episodes; the Shield's own breaches are 0 throughout):

| profile | autopilot breaches | X6 | X7 | G5 | G8 | G10 |
|---|---|---|---|---|---|---|
| smoke | 8 -> 9 | 0 -> 0 | 3 -> 1 | 16 -> 17 | 8 -> 12 | 4 -> 3 |
| library | 15 -> 13 | 2 -> 0 | 7 -> 4 | 43 -> 47 | 18 -> 25 | 5 -> 3 |
| nightly | 69 -> 51 | 18 -> 0 | 33 -> 24 | 213 -> 234 | 90 -> 117 | 15 -> 9 |

Correctness is the same flown longer on every profile; what moved is how the
rail arm's Loiters end. 19 of the smoke's 126 episodes (both arms), 57 of the
library's 358 and 297 of the nightly's 1482 differ in some field, all of them
on the rail arm. By kind (library and smoke, rail arm only):

* **Held instead of drifting (better).** dyn-nfz-spawn-ahead 6 m ahead (4 and
  8 m/s): X6 at 12.7 s -> the Loiter timeout (G5) at 35 s, the autopilot
  breach and the unsafe episode gone; wind-beyond-authority at 8 m/s (lanes 15
  and 20): X7 at 14.7 s -> G5, the same; corridor-swap-narrowed at 2 m/s: X7
  -> G5, time to safe 3.3 -> 1.7 s. The old hold was a zero velocity command:
  the airframe stopped wherever its braking (and, at 8 m/s, the wind) took it;
  the GUIDED hold brings it back onto the checked point.
* **Handed back, and re-escalated at once.** corridor-drift-angles (4 m/s -60
  deg; 8 m/s -60 and +60 deg), gps-dropout lane 15 window 2-4,
  dyn-nfz-rotate-step and wind-within-authority lane 15 at 5 m/s now take G8
  and then X1 / G1 -> G3 within 0.1-0.5 s: the aircraft held still on its
  point lets Loiter's "acceptable" clock run its 2 s where the old hold's
  drift kept restarting it. Each ends as its label expects;
  wind-within-authority at 5 m/s ends its run in the second Loiter (censored)
  where it ended by G5 before, and flown longer it is decided as before.
* **Worse.** wind-beyond-authority at 12 m/s (lanes 15 and 20): G6 at 12.1 s
  instead of 7.0-7.3 s and no landing (G10) within the run; mean time to safe
  3.1 -> 13.6 s and 3.3 -> 9.8 s. standoff-reclassified: 2 autopilot breaches
  where there were none - the GUIDED hold pulls the aircraft back onto its
  in-place point (23.9, -4.2) as the pedestrian's stand-off ring reaches it
  (17.3 s), and its RTL from there later crosses the ring (43.7 s).
* **The nightly** (26 cells, 78 episodes whose fail-safe outcome or edges
  moved; each decided as before when flown longer): held instead of drifting -
  dyn-nfz-spawn-ahead 6 m ahead (all six variant and speed cells) X6 -> G5,
  wind-beyond-authority at 8 m/s (lanes 15 and 20) X7 -> G5,
  corridor-swap-narrowed at 2 m/s X7 -> G5, and dyn-nfz-spawn-ahead 12 m ahead
  at 4 and 8 m/s now reaches home (G9) within the run; handed back and
  escalated again - corridor-drift-angles (4 m/s -60 deg; 8 m/s -60 and +60
  deg), gps-dropout lane 15 window 2-4 and lane 23.6 window 6-13,
  dyn-nfz-rotate-step, dyn-nfz-spawn-ahead variant 2 at 2 m/s (12 and 30 m
  ahead) and wind-within-authority lane 15 at 5 m/s, where the last and
  spawn-ahead 12 m now end their run in the second Loiter (censored) instead
  of by G5; worse - wind-beyond-authority at 12 m/s (lanes 15 and 20) no
  longer lands (G10) within the run, and standoff-reclassified has 2 autopilot
  breaches per seed where it had none, as in the library. The nine
  dyn-nfz-rotate-spin cells did not move: none fails safe, before or after.
  The rail-arm episodes wrong against (r) or (r2) flown longer are the same 12
  before and after (rail-mission-stub-vla and standoff-approach-angles at 7 m,
  against (r)). Loiter's quick re-escalations (within 2 s of G8) on the rail
  arm: 0 of 90 G8 -> 27 of 117 (15 of those episodes then fail safe).

## What it does not do

* Remove the 2 Shield escape ticks per opens-on-vehicle cell: they are
  flown before G3, and the KPI definition is unchanged. (Removed since
  2026-10-21 by the check instead, section above; the cells are unpinned.)
* Fly a rail - until 2026-11-07. The SITL, ROS 2 and Project AirSim rails
  run their own FSM (`guardrail/replay.py rail_shield`), called no `hold_at`,
  and held or landed in place (their Loiter records said `hold_needed:
  true`). Since 2026-11-07 the pymavlink rail and the ROS 2 Shield node plan
  and fly the hold (`guardrail/hold_flight.py`: a GUIDED transit, then the
  hold mode on arrival; `--hold-points in_place` for the old hold), and the
  harness's `rail` vehicle holds at moved points; Project AirSim / ArduPilot
  and the city loop still hold in place. Flown on the rig since the hil
  shakedowns hs02-hs04; since that night the Loiter is held in GUIDED on the
  plan's point ("The Loiter held in GUIDED", above).
* Model wind. A point the aircraft cannot reach against the wind is no
  better than one it cannot hold; X6 / X7 still judge the illegal depth.
* Cost nothing. A relocation is one search, on the tick it happens (since
  2026-11-07 spread over ticks: "The per-tick bound", above): the
  library's moved holds took 1-44 ms on the desktop (time-window-opens-on-
  vehicle the dearest, whose candidates nearer than 10 m are all inside the
  zone about to open); an in-place plan 0-3 ms; the per-Loiter-tick point
  check 0.2-1.3 ms (2026-10-20). The worst case is a search that finds
  nothing and spends the whole budget, about 60 ms of the desktop's time:
  31-55 ms in the worlds of "The budget", above (2026-10-21, best of 3,
  desktop otherwise idle), and 29-41 ms in those that still spend it since
  One rule first (the bars and the stacked zones; the overlapping zones and
  the diamonds now end at the last ring in 12-15 ms). Until the re-review
  the 800-query budget's fruitless search took 124-135 ms against the 40 m
  bar closing at 3 m/s, over the 100 ms tick, and until the final review
  226 ms with 20 overlapping zones and 907 ms with 20 diamonds. A search
  that finds nothing is repeated at most once a second (`HOLD_REPLAN_S`),
  and no Loiter searches more than twice a second (`HOLD_MIN_REPLAN_S`).
  The Orin is not measured: the budget is a count, so a slower computer
  takes proportionally longer.
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
   transit's ticks in the Shield's counter? (Since 2026-11-07 the rails fly
   it so, the transit streamed by the companion and recorded as the hold's
   own leg, `hold_transit`, not the Shield's.)
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
4. Since 2026-11-07 (the per-tick bound) a Loiter's plan can arrive up to
   6 ticks (0.5 s) after it is asked for, the aircraft holding where it is
   meanwhile, so that no tick pays for a whole search (one Orin tick paid
   146.9 ms, over WP3-13's 100 ms). Is that latency acceptable, or does the
   grant prefer an occasional long tick to a later plan?
5. The grant's FSM says "SET_MODE LOITER". Since 2026-11-07 night the rails
   hold a Loiter at the search's point in GUIDED (the companion streams a
   position hold) and ask ArduCopter's BRAKE only on a stream fault, because
   every hand-over to BRAKE carried the aircraft 1-2.5 m along the last GUIDED
   command (hs04). The FSM's record still says LOITER. Is a companion-held
   GUIDED hold acceptable as the grant's Loiter on the rig?

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
