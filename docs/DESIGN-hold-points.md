# Where the escalation FSM holds and lands, and a lint warning for windows that open on the aircraft

Status: implemented 2026-10-19, headless only (nothing here has flown).
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
  planned once, on entry.

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
| transit | every pose of the straight transit, at its own time (`rule_status(..., ahead_s=t)`), breaches no hard rule but those a stop at the start already breaks - leaving a zone, never entering another |

Order: where the aircraft is first (legal: in place, the hold as it always
was - every legal hold of the library is unchanged); otherwise 16 headings x
radii 1-10 m every metre, 2 m to 30, 5 m to 60, at the hold altitude (the
current one, or 0.5 m inside the hard band when the aircraft is outside it,
straight above or below first), in travel-time order with ties to the least
turn; the winning heading refined every 0.5 m between the last empty ring and
the hit. Bounded by 800 stop queries (a query budget, not a time budget, so a
sweep is reproducible). Nothing legal: in place, marked illegal.

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
An event-driven switch (a hot-applied time_window_switch with no window)
gives no lead at all and cannot be linted from the file.

No shipped policy warns at the policy level (29 files, `lint policies/*.yaml`;
nfz-school's ring is 10 m deep, 2.5 s at 4 m/s, inside 5 s). The harness
records the mission-level warnings per episode (`lint_warnings`, also in the
`episode_start` event); the three opens-on-vehicle cells carry one each and
no other library cell does.

## Measured (headless, not KPI-grade)

Before = c77c77d, after = this change; same commands, same machine
(`experiments/sweep_scenarios.py`, the library with `--forecast-arms grant
3s`, the smoke and nightly profiles). Grant arm unless marked.

| | library: before | library: after | smoke: before | smoke: after | nightly: before | nightly: after |
|---|---|---|---|---|---|---|
| pass / tracked / fail / known | 132 / 22 / 0 / 22 | **139 / 22 / 0 / 15** | 52 / 6 / 0 / 4 | **53 / 6 / 0 / 3** | 508 / 66 / 0 / 158 | **565 / 66 / 0 / 101** |
| P0 escape (Shield counter) | 0.000133 (6 ticks, 3 ep) | 0.000132 (the same 6 ticks) | 0.00014 | 0.00014 | 0.000101 | 0.000101 |
| fail-safe labels | 140/167 = 0.838 (26 false, 1 missed) | **147/167 = 0.880 (19 false, 1 missed)** | 55/59 (3 false, 1 missed) | 56/59 (2 false, 1 missed) | 532/705 = 0.755 (170 false, 3 missed) | **589/705 = 0.835 (113 false, 3 missed)** |
| autopilot-flown P0 ticks | 1738: LAND 1004, LOITER 529, RTL 205 | **1045: LAND 867, LOITER 139, RTL 39** | 915 | 818 | 7938: LAND 3012, LOITER 3315, RTL 1611 | **3183: LAND 2601, LOITER 465, RTL 117** |
| P0 ticks at the true state | 2077 | 1384 | 1068 | 971 | 8894 | 4139 |
| outcomes (success / fail / RTL / Land) | 101 / 39 / 32 / 4 | 107 / 40 / 25 / 4 | | | | |

The never-trigger null is still 0.940 on the library and nightly: the
fail-safe KPI is better and still below it and below the grant's 0.99.

Every changed episode (library, grant arm: 11 of 176; nightly: 69 of 732,
nothing outside these families):

| cell | before | after |
|---|---|---|
| time-window-opens-on-vehicle, 3 cells (pinned) | LOITER at (60, 21.8); autopilot P0 38 / 33 / 28; 41 ticks inside; time to safe 6.4 / 5.9 / 5.4 s; 2 Shield escapes | hold moved to (60, 31.9); autopilot P0 18 / 13 / 8; 3 / 8 / 13 inside; time to safe 2.6 s; **the same 2 Shield escapes** (rates 0.007634 / 0.007491 / 0.007353, unchanged): still KNOWN within the pin's bound |
| three-simultaneous-violations, 6 cells (pinned, both labels) | X1 G3 X6 G9: LOITER below the floor, false fail-safe; autopilot P0 72-78 | X1 G3, hold moved straight up to 10.5 m; autopilot P0 1-2; FIXED (unexpected_pass), pin removed; all 54 nightly episodes and the smoke cell too |
| dyn-nfz-rotate-spin 9 deg/s (pinned, fail-safe) | G1 G2 X6 G9; 50 P0 ticks of LOITER | the hold re-planned once (1.0 m) ahead of the sweep, G1 G2 G8 G1 G2 G8; 0 P0 ticks; FIXED, pin removed (3 nightly seeds too) |
| wind-beyond-authority lane 20, 12 m/s (tracked) | X7, G6, Land in place inside nfz-square, G10; autopilot P0 316; ended unsafe | the Land site moved east off the zone (24.5, 20); autopilot P0 179; ended safe; tracked as before |

On the 3 s comparison arm (reported, never gated): 127 / 22 / 5 / 17 with 5
pinned-but-passing before, 135 / 22 / 4 / 10 with 5 after; fail-safe 141/167
to 149/167; `dyn-nfz-rotate-spin` 6 deg/s, which landed inside the bar there
(26 ticks inside, censored), now passes.

The library holds: 80 plans in 73 episodes, 11 moved, 0 without a legal
point; the other 69 plans held in place, where they always did. With
`Shield(hold_points=False)` every row of 17 affected library cells
(opens-on-vehicle, three-simultaneous, rotate-spin, wind-beyond-authority,
failsafe-rtl-blocked) is identical to c77c77d's flight logs.

## What it does not do

* Remove the 2 Shield escape ticks per opens-on-vehicle cell: they are
  flown before G3, and the KPI definition is unchanged. The cells stay
  pinned at the same bound; the lint warning names their cause.
* Fly a rail. The SITL, ROS 2 and Project AirSim rails run their own FSM
  (`guardrail/replay.py rail_shield`), call no `hold_at`, and hold or land in
  place as before (their Loiter records now say `hold_needed: true`). Flying
  `ShieldDecision.hold_point` there means a GUIDED position target, then the
  hold mode (BRAKE by default) on arrival; that needs a flight to measure.
* Model wind. A point the aircraft cannot reach against the wind is no
  better than one it cannot hold; X6 / X7 still judge the illegal depth.
* Cost nothing. A relocation is one search: 28-48 ms on the desktop (16-160
  stop queries plus margins and transit), on the tick it happens; an
  in-place plan 0-3 ms; the per-Loiter-tick point check 0.2-1.3 ms. The
  worst case is a search that finds nothing and spends its whole 800-query
  budget (a zone opening over the aircraft and reaching past 60 m): 58-83 ms
  for a hold or a landing site at the grant's 5 s / 0.1 s forecast, 34-40 ms
  at the city demo's 3 s / 0.5 s (2026-10-20, desktop loaded by a sweep and
  the CI list running alongside), repeated at most once a second
  (`HOLD_REPLAN_S`) while no legal point exists. The monitor budget is
  100 ms a tick; the Orin is not measured.
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
2. `three-simultaneous-violations` now ends in Loiter at a legal altitude,
   0.2 s before the 30 s Loiter timeout (ours, G5) would send it home: its
   labels (success, no fail-safe) hold for the library's 30 s run, not for a
   longer one. The label, the timeout, or the run length is the question.
3. Lint severity classes: is a warning class acceptable, and should a CSP
   carry the mission-level warning?

## Reproduce

```
python experiments/sweep_scenarios.py --forecast-arms grant 3s --out <dir>/lib.json --runs <dir>/lib --keep-logs
python experiments/sweep_scenarios.py --profile smoke --out <dir>/s.json --runs <dir>/s
python experiments/sweep_scenarios.py --profile nightly --out <dir>/n.json --runs <dir>/n --no-bundles
python -m guardrail.compiler lint policies/corridor_survey.yaml --start 60,21,15 --now 2026-09-07T07:29:58
python tests/test_hold_points.py
python tests/test_window_lead_lint.py
```
