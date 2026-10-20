# Where the escalation FSM holds and lands, and a lint warning for windows that open on the aircraft

Status: implemented 2026-10-19, revised 2026-10-20 after review (the
transit test, the search budget, the re-plan floor, the audit record, the
lint's no-exit case, and the three-simultaneous-violations numbers restated
as what is robust). Headless only: nothing here has flown, and no flight rail
flies a moved hold yet.
Code: `guardrail/fsm.py` (`HoldPoint`, `EscalationFSM.needs_hold` /
`hold_at`, `TickInput.hold_legal`), `guardrail/holdpoint.py` (the search),
`guardrail/shield.py` (`looking_ahead`, `state_is_unsafe(ahead_s=)`,
`rule_status(ahead_s=)`, `hard_violations`, `kinematic_caps`, the wiring in
`_escalate`), `experiments/sweep_scenarios.py` (the modelled autopilot flies
the point), `guardrail/compiler.py` (`time_window_lead_warnings`, `lint`).
Tests: `tests/test_hold_points.py`, `tests/test_window_lead_lint.py`.

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
`model_dump()`, so no dumped decision or digest changes.

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
| transit | every pose of the straight transit, at its own time (`rule_status` in a look-ahead view), breaches no hard rule, except one the aircraft is already in - breached at the start now, or not in force now and in force over the start by then (a window opening over it) - and then no deeper than a stop at the start is at that moment (`Shield.rule_depths`, the per-rule terms of `unsafe_depth`) |

Order: where the aircraft is first (legal: in place, the hold as it always
was - every legal hold of the library is unchanged); otherwise 16 headings x
radii 1-10 m every metre, 2 m to 30, 5 m to 60, at the hold altitude (the
current one, or 0.5 m inside the hard band when the aircraft is outside it,
straight above or below first), in travel-time order with ties to the least
turn; the winning heading refined every 0.5 m between the last empty ring and
the hit. Bounded by 800 Shield queries - a stop query or a sampled transit
pose, one each (a query budget, not a time budget, so a sweep is
reproducible). Nothing legal: in place, marked illegal.

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

The search budget now counts a sampled transit pose like a stop query (800
in all). Once transits could fail, an uncounted transit check per candidate
whose stop was legal made a fruitless search 30 % slower (a 40 m bar closing
at 3 m/s: 106 transit checks, 325 poses); a pose costs about 0.7 of a stop
query on the desktop.

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
climb into the band flies 0-2 autopilot P0 ticks where 72-78 were flown
before (the RTL after G5 adds 9-18, over the 20 m ceiling).

Every changed episode (library, grant arm: 11 of 176; nightly: 69 of 732,
nothing outside these families; against the first version of this note
only the 54 three-simultaneous episodes moved):

| cell | before | after |
|---|---|---|
| time-window-opens-on-vehicle, 3 cells (pinned) | LOITER at (60, 21.8); autopilot P0 38 / 33 / 28; 41 ticks inside; time to safe 6.4 / 5.9 / 5.4 s; 2 Shield escapes | hold moved to (60, 31.9); autopilot P0 18 / 13 / 8; 3 / 8 / 13 inside; time to safe 2.6 s; **the same 2 Shield escapes** (rates 0.007634 / 0.007491 / 0.007353, unchanged): still KNOWN within the pin's bound |
| three-simultaneous-violations, 6 cells (pinned, both labels) | X1 G3 X6 G9 (9.6 s): LOITER below the floor, false fail-safe; autopilot P0 72-78 | X1 G3 G5 G9 (31.9 s): hold moved straight up to 10.5 m, no X6, then the Loiter timeout sends it home; autopilot P0 19-20 (climb 0-2, RTL 9-18); **still KNOWN on both labels** |
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
* Cost nothing. A relocation is one search: 28-48 ms on the desktop (16-160
  stop queries plus margins and transit), on the tick it happens; an
  in-place plan 0-3 ms; the per-Loiter-tick point check 0.2-1.3 ms. The
  worst case is a search that finds nothing and spends its whole 800-query
  budget (a zone opening over the aircraft and reaching past 60 m): 58-83 ms
  for a hold or a landing site at the grant's 5 s / 0.1 s forecast, 34-40 ms
  at the city demo's 3 s / 0.5 s (2026-10-20, desktop loaded by a sweep and
  the CI list running alongside); against a moving zone, whose stop queries
  cost more, 124 ms (the 40 m bar closing at 3 m/s of the transit tests,
  2026-10-20, desktop loaded by four sweeps), over the 100 ms tick on that
  one tick. A search that finds nothing is repeated at most once a second
  (`HOLD_REPLAN_S`), and no Loiter searches more than twice a second
  (`HOLD_MIN_REPLAN_S`). The Orin is not measured.
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
python tests/test_window_lead_lint.py
```
