# DESIGN - the Safety Shield enforces what the policy says

2026-10-07. Audit cards WP3-07, WP3-06, WP3-08, WP3-15 (Shield side), WP1-10,
WP1-11, WP1-12, WP3-05, WP3-02, WP4-03 (Shield side), WP1-15, WP1-09, X-12,
WP3-12, WP3-22, X-13, WP1-06 (enforcement), WP2-16 (IR part).
Code: `guardrail/shield.py`, `guardrail/ir.py`, `guardrail/fsm.py`,
`guardrail/api.py`, `guardrail/audit.py`, `demo/policy_hud.py`,
`demo/follow_vlm.py` (FenceGuard and the end-of-flight zone count).
Tests: `tests/test_shield.py`, `tests/test_shield_events.py`, `tests/test_api.py`,
`tests/test_ir.py`, `tests/test_fsm.py`, `tests/test_policy_hud.py`,
`tests/test_fenceguard.py`.
2026-10-12 (section 9): `guardrail/repair.py` (new), `guardrail/audit.py`,
`guardrail/ir.py`, `guardrail/shield.py`, `guardrail/frames.py`,
`policies/audit_record.schema.json` (new); tests
`tests/test_shieldcore_repair.py`, `tests/test_shieldcore_audit.py`,
`tests/test_shieldcore_identity.py` (new), `tests/test_flight_pas_ap_20261008.py`,
`tests/test_shield.py`, `tests/test_frame_contract.py`, fixtures in
`tests/fixtures/`.

## The problem

Up to 401305a the Shield repaired every violation the same way and braked
when the repair failed. Everything else a rule could say had no effect:

- `violation_action` (monitor_only, project_fix, brake, loiter, RTL, land),
  `constraint_type` (hard / soft) and `priority` were read by nobody on the
  Shield's side. A rule written `brake` was repaired. A soft rule was enforced
  exactly like a hard one.
- The escalation state machine (`guardrail/fsm.py`, docs/DESIGN-escalation-fsm.md)
  existed but nothing fed it. `Repair` had no `magnitude_m` / `axis`, so the
  conservative cap theta could not be applied to any repair.
- The lookahead was 3 s at 0.5 s (7 poses), not the grant's 5 s at 10 Hz (50
  future poses). Time windows were read once per tick at minute resolution
  with an inclusive end, so a window written to end at 17:30 stayed in force
  until 17:30:59, and no forecast pose was judged at its own time.
- `hot_apply` appended a `PolygonFence` and bumped the generation. That is a
  class the grant locks at mission start; the three classes it does allow
  mid-flight (dynamic_nfz, time_window_switch, corridor_swap) did not exist
  as events. No test file imported `guardrail/api.py`.
- The on-screen policy panel and the controller's FenceGuard each measured
  the zones with their own geometry. The grant allows "exactly one
  rule-evaluation code path in the system" (Safety Shield page).
- The audit record lacked the grant's `generation`, `monitor_tick`,
  `repair_attempts` and FSM fields, and the file was one append-only log.

## What the Shield does now

### 1. Every tick feeds the escalation FSM; the emitted action does not depend on it

`Shield(policy, escalation=True)` (the default) owns one `fsm.EscalationFSM`
with the grant's thresholds (N = 3 in T = 5 s, theta 2.0 m lateral / 0.5 m
vertical, T_recover 2 s; `fsm.FSMConfig`). `filter()` runs the repair stack,
then steps the FSM with the decision. The decision carries the verdict:

| field | meaning |
|---|---|
| `emitted` | the repair stack's output, exactly as before |
| `fsm_state_before` / `fsm_state_after` / `fsm_edge` | the FSM around this tick |
| `set_mode` | the ArduPilot mode to request now (only on the tick the state changes) |
| `setpoint` | `pass`, `brake` or `none` |
| `command` | what to stream: `emitted`, a stop, or `None` (LOITER / RTL / LAND own the aircraft) |
| `fsm_record` | the FSM's audit record (reason, governing rule, theta, config hash) |
| `fsm_fault` | the FSM refused this tick's input (see below) |
| `generation` / `policy_hash` | the rule set the tick pinned when it began |

`emitted` is unchanged on purpose. docs/DESIGN-escalation-fsm.md asks that
what `filter()` decides must not depend on escalation state, because most
callers (the sweep, the demos, the deck builders) have no autopilot whose
mode they could change. A caller that flies the FSM's verdict streams
`command` and requests `set_mode`. The flight rails run their own FSM, fed
with what only they see (the autopilot's mode, home reached, landed); they
build the Shield through `guardrail/replay.py` `rail_shield`, which passes
`escalation=False`. A caller that flies the Shield's own FSM reports the
autopilot's side through `filter(..., rtl_failed=, home_reached=, landed=)`,
which drives the terminal edges G6, G9, G10 and X5.

Theta is applied by the FSM, never inside the repair stack, from each repair's
own `magnitude_m` and `axis` (the grant's audit fields, Safety Shield p5;
`Shield(theta_horizon_s=None)`, the default, `fsm.DEFAULT_THETA_HORIZON_S`).
A second reading is selectable, `theta_horizon_s=fsm.THETA_HORIZON_S`: the
setpoint change |dv| over one 0.1 s control period, p4's
"||action_repaired - action_original||" read literally. It is NOT the
default. A builder made it the default on 2026-10-08 and called it the
grant's basis; the review of 2026-10-09 reverted that, because the grant's
own audit example (p5: LateralProjection `magnitude_m` 0.7 for a raw
(5.0, 1.2) emitted as (4.7, -0.3), a 1.53 m/s change, which one period would
record as 0.15 m) and p3 ("project the predicted point to the nearest
feasible point", a position depth) read the other way, and under every
shipped speed cap the one-period reading can never exceed theta - a waiver
the 6 Oct decision ("conform rather than waive") does not allow without the
PI. docs/DESIGN-escalation-fsm.md, "Theta: two readings, one default", has
both sides and the measurements.

| operator | axis | magnitude_m |
|---|---|---|
| AltitudeFix, CorridorAltitudeFix | vertical | the altitude error at the lookahead end |
| GeofenceSlide | lateral | the deepest forecast pose inside the zone's margin ring |
| ClearanceFix | lateral | the deepest forecast incursion into the clearance ring |
| StandoffHold | lateral | the forecast's closest approach inside the ring (`ring - d_min`; until 2026-10-08 a straight-line range through the subject, up to 11.0 m into a 5 m ring) |
| CorridorReturn | lateral | the excursion past the half-width |
| GeofenceEscape, StandoffRecover | lateral | the current depth; `recovery=True`, exempt |
| SpeedClamp, ClimbClamp, YawClamp, Sanitise, Brake, SoftRelax | none | velocity or record only |

The PI reference (`safety_shield/repair.py`, LateralProjection) applies theta
to penetration depth ("a deep head-on dive therefore reports a large
magnitude and is escalated rather than projected"), measured into the
AUTHORED polygon, with no margin (`policy_dsl/ir.py` signed_distance). This
Shield's per-operator field measures into the zone's margin ring, the shape it
enforces, so with a 1 m margin a forecast reports up to 1 m more than the
reference would; that is this project's reading, and an open question for
the PI (see "Interpretations"). On the default basis a pilot that keeps
pushing into a zone escalates (X1, then G3 to LOITER) once the forecast
reaches more than 2 m inside the ring. On the one-period reading a slide
changes the setpoint by at most 2 x the speed cap, 0.8-1.0 m per period under
the 4-5 m/s caps of every policy here, so theta would never fire on them and
only the other escalation arms (no operator converged, N-in-T, X6, X7, a
rule's own action) would carry it.
`tests/test_shield.py::test_theta_defaults_to_the_operators_depth_and_one_period_is_opt_in`
and `::test_theta_still_escalates_a_setpoint_change_beyond_it` pin both.

If the FSM refuses a tick (a position repair without a magnitude, an
inconsistent input), the control loop does not stop: the decision carries
`fsm_fault`, the first one requests LOITER, and `command` is `None` until
`reset_episode()`.

### 2. A rule's breach action, hard/soft type and priority change what happens

The effective action is the FSM's own table (`fsm.RuleHit.effective_action`),
so the repair stack and the FSM cannot disagree.

| written | what the Shield does |
|---|---|
| `monitor_only` | recorded, never repaired; the command flies and the rule stays in `emitted_violations` (a P0 monitor_only rule counts as an escape, which is the honest place for it) |
| `project_fix` (alias `repair`) | the repair stack, as before |
| `brake`, `loiter`, `RTL`, `land` | projection skipped and BRAKE emitted, where a stop is legal; the FSM requests the mode |
| any action on a soft rule | capped at `brake` (a soft rule never changes the flight mode) |

"Where a stop is legal": inside a zone or under a floor, stopping holds the
aircraft in the violation (the module docstring's deadlock). There the repair
stack still recovers and the FSM still makes the mode change, with the stop
withheld (`fsm_record["stop_withheld"]`).

What the PI reference does here is narrower. Its `safety_shield/shield.py`
skips projection only when the WORST violation's action is `RTL` or `land`,
and then emits the raw action while its FSM changes mode; for `brake` and
`loiter` it still runs the repair. Skipping projection and stopping for
`brake` and `loiter` too, and stopping rather than passing the raw action
through for `RTL` and `land`, is this project's reading (follow-up #112,
docs/DESIGN-escalation-fsm.md), as is the stop-illegal exception above.

Priority decides what gives way when the repair stack cannot satisfy every
rule: soft rules are given up lowest priority first (P2, then P1, then P0),
re-running the chain after each level, and only a rule that is actually still
broken is listed in `decision.relaxed` (with a `SoftRelax` record). A hard
rule is never given up; when hard rules conflict, the existing fallback
(rescue heading or stop) runs and the FSM escalates. This is the grant's
severity order "hard >> soft, P0 > P1 > P2" (Prefix Compiler page).

A policy with a rule the Shield does not enforce (`distance_envelope`, any
`altitude_ref: MSL`) is refused at construction (`Shield.unenforceable`), so a
`Policy` built directly cannot fly with a rule silently ignored.

### 3. The grant's lookahead, read at each pose's own time

The default is `lookahead_s=5.0, dt=0.1` (51 poses including t = 0). A caller
that passes its own horizon keeps it.

The clock (`now=`, a zero-argument callable) is read once per tick. Each
forecast pose at time tau is judged against the rules in force at now + tau,
checked on both sides of that instant, now + tau - eps and now + tau + eps,
with eps = dt / 2 (the grant: "handle the 'boundary instant' by checking both
sides at t- and t+"). Either side in force counts as in force: the
conservative reading. With eps = dt / 2 every window edge inside the horizon
is seen from both sides by the pose nearest it.

The check samples the two instants; it does not intersect the window with
[tau - eps, tau + eps]. A window shorter than eps can fall between the
samples, and a zero-length window (`start_time == end_time`) is never seen:
hovering inside a 12:00-12:00 zone across 12:00:00 raised nothing in the
review's probe, at four start times. Such a window is ambiguous anyway - an
instant here, a whole minute in `models.Recurrence.active_at` - and the DSL
lint does not refuse one yet (see "Needed outside this unit").

Windows are evaluated to the microsecond (`shield.recurrence_active`) on a
closed interval [start, end]: a window ending 17:30 is last in force at
17:30:00.000, and with eps = 50 ms the zone is off for a tick that starts at
17:30:00.050 or later. "23:59" (the DSL default end) and "24:00" mean the end
of the day; read as the instant 23:59:00 they would switch every all-day rule
off for a minute each night. Day attribution is models.py's. With no clock
every rule is in force, as `ConstraintBase.active_at` has always said.

Fast path: when no schedule in the policy has an edge anywhere in
[now - eps, now + horizon + eps], each rule's state is read once per tick.

`models.Recurrence.active_at` still compares whole minutes with an inclusive
end; the compiler's CSP time filter and `experiments/sweep_scenarios.py` read
it, not the Shield's evaluator (see "Needed outside this unit").

### 4. Mid-flight events: the grant's three classes, atomic

Policy DSL page: "Three constraint classes can be hot-applied mid-mission:
dynamic_nfz, time_window_switch, corridor_swap. All other classes are locked
at mission start. Every hot-apply bumps the bundle's generation counter and
re-derives policy_hash."

| event | Shield call | REST |
|---|---|---|
| dynamic_nfz add | `hot_apply(DynamicNFZ)` / `spawn_nfz` | `POST /dynamic_nfz` |
| move / translate / rotate / scale | `move_nfz`, `translate_nfz`, `rotate_nfz`, `scale_nfz` | `PATCH /dynamic_nfz/{id}` |
| expire | `expire_nfz` | `DELETE /dynamic_nfz/{id}` |
| time_window_switch | `hot_apply(TimeWindowSwitch)` / `set_rule_active` | `POST /time_window_switch` |
| corridor_swap | `hot_apply(CorridorSwap)` | `POST /corridor_swap` |
| any of the three, by type | `hot_apply` | `POST /hot_apply` |

Each event builds the new rule list, checks it, and commits in one step: the
live policy gets the list and generation + 1, the compiled runtime (rule
lists, effective actions, schedule, IR) is rebuilt with unchanged zones
keeping their compiled rings, and swapped in with one assignment. A tick reads
the runtime it pinned when it began, so an event landing mid-tick (the REST
thread) is never half-seen; the next tick has it. `Shield.events` lists every
event with its generation and hash.

The checks before a commit: a new id must be new (409 over REST), a named id
must exist (404), the event must not relax a hard rule of another layer (422,
below), and the resulting policy must not add a finding to the DSL's own
consistency lint, `Policy.lint()`, which every loader runs (422). The lint is
the patch validator for consistency: expiring a zone that a switch still
targets, for example, would leave a policy no loader accepts.

**The layer rule.** Policy DSL page, "Layered authoring model": a
mission-layer rule cannot relax a higher-priority hard constraint from
regulation. `models.merge_layers` enforces it at ingest; until the 2026-10-07
review nothing enforced it mid-flight, because the lint has no relaxation
check. On a merged regulation + mission policy, a mission switch-off of the
hard regulation zone `nfz-airport` was accepted over REST (with and without
`"layer": "mission"`), and a 4 m/s command into the zone then flew with no
violation; a swap of a hard regulation corridor to a soft, monitor_only, 2 km
wide one was accepted the same way. An event now has a layer like any rule
(its `layer` field, absent = mission) and is refused (`LayerRelaxation`, 422)
when it would:

| event | refused when |
|---|---|
| time_window_switch, `active: false` | the target is hard and of another layer (merge_layers' own test), or another layer's switch holds the target on (the last switch in force decides, so appending would overrule it) |
| corridor_swap | the target corridor is hard and of another layer, and the corridor the swap puts in force is not provably no looser (`models._not_looser`, on the corridor merge_layers builds) |
| move / translate / rotate / scale / expire of a dynamic_nfz | the zone is hard and of another layer (each can relax it) |

What stays allowed: tightening (switching a rule on, spawning a zone), any
soft rule, and any rule of the event's own layer. A switch-off of a
mission-layer hard rule by a mission-layer event is accepted, exactly as
merge_layers accepts it; that is what the grant's time_window_switch is for.
An unscheduled switch or swap supersedes earlier ones from its own layer only.

Every REST event is a mission-layer event: a body that names another layer
(`"layer": "regulation"` or `"site"`) is refused with 422, since nothing on
that channel establishes that the sender speaks for the regulator or the
site. The Python API takes `layer=` on `set_rule_active` and the zone edits,
for code that does.

What each event means:

- A **dynamic_nfz** with `motion` moves from the moment it is spawned
  (`DynamicNFZ.ring_at(age)`). Each tick the Shield materialises the zone at
  its current position; inside the tick each forecast pose is moved into the
  zone's frame at its own time (`_to_zone_frame`), so a zone drifting into the
  forecast is caught where it will be. Escape and slide speeds are measured
  relative to the zone's velocity. An edit re-anchors a moving zone where it
  is now. A moving zone cannot go in the STRtree; it is skipped when the box
  it can sweep over the horizon (its ring translated along its velocity, or
  the disc it turns in) misses the forecast's box, which changes no answer
  (`test_the_moving_zone_prefilter_never_changes_an_answer`).
- A **time_window_switch** holds its target on (`active: true`) or off while
  the switch itself is in force. Of the switches in force on one rule, the last
  in policy order decides. A switch whose window cannot be read (no clock)
  may turn a rule on but never off: nothing that cannot establish "this rule
  is off now" may switch it off. An unscheduled switch supersedes earlier
  unscheduled switches on the same rule.
- A **corridor_swap** replaces its target corridor while the swap is in
  force. Unknown window (no clock) enforces both, the stricter reading. An
  unscheduled swap supersedes an earlier unscheduled swap of the same corridor.

`reset_episode()` starts a new episode on the same Shield (FSM, fault latch,
sliding window, monitor clock, and the mission-start lock) and keeps the rule
set, its generation, and every zone where it is: a moving zone continues
from its position at the reset. On a clock that restarts with the episode
(the counted clock does, at 0) its age carries on from the reset; on a clock
that runs on (`t` passed in) it keeps moving in real time, gap included.
Before the review's fix a zone spawned 120 s into an episode jumped back to
its spawn position on reset and stood still until the new episode's clock
passed 120 s (`test_a_moving_zone_carries_on_across_reset_episode`).

Switches and swaps are rules in the policy, so they are hashed and audited
like any other. Declared in a policy file, all three are enforced from the
first tick. Since 2026-10-08 the compiler renders all three
(`models.RUNTIME_TYPES`), and switches and swaps in a policy file fly. A
dynamic_nfz in a policy file is held from flight (`models.FLIGHT_HELD_TYPES`).
The hold was added on 2026-10-08 for the rails' dwell metric `nfz_s`. Then,
seven rails under `demo/` counted `policy.by_type(PolygonFence)` (polygon and
circle fences), so a declared zone scored 0 s. Since 2026-10-11 every rail
counts dwell through `guardrail.replay.zone_dwell` (`demo/follow_vlm.py`'s
count, read from `Shield.rule_status`). It counts polygon and circle fences
and every dynamic_nfz that does not move, and names a moving one in
`nfz_moving_not_counted` instead of scoring it. On 2026-10-11 the hold is
still in place, with its pre-2026-10-11 reason; what the rails still cannot
score is a moving zone (`docs/DESIGN-policy-dsl-grant-form.md`, decision 5).
A dynamic_nfz injected mid-flight with `hot_apply` is unaffected.

Mid-flight hot-apply latency on the 50-rule load, after mission start: a
dynamic_nfz spawn (lint and layer checks, runtime rebuild included) median
1.17 ms, p99 1.34 ms, max 1.50 ms over 300 spawns, against the grant's 50 ms;
null, the same spawn on an empty policy: median 0.16 ms (scratch script
`bench_spawn.py`). The stock bench's own hot-apply row (median 1.20 ms) adds
a polygon_fence before the first tick, the `add_before_start` path, not the
mid-flight one.

### Migrating a polygon_fence hot-apply

`hot_apply(PolygonFence)` (and `circle_fence`) is accepted only before the
first `filter()` tick, while the fence is still part of the mission-start
policy. After it, it raises `LockedRuleClass` with this note. The migration
is one line and keeps id, vertices, band, margin, priority, hard/soft type and
breach action:

```python
from guardrail.shield import as_dynamic_nfz
shield.hot_apply(as_dynamic_nfz(fence))          # was: shield.hot_apply(fence)
```

Over REST, `POST /nfz` (the 2026-07 body) now spawns the dynamic_nfz it stands
for, and its response says so (`"type": "dynamic_nfz"`, `"deprecated"`).
`POST /hot_apply` with a locked type is refused with 422 and this note.
`reset_episode()` starts a new mission, so a polygon_fence may be added again
before its first tick.

A hot-applied dynamic_nfz has a different policy hash than the same zone
appended as a polygon_fence. Runs recorded before 2026-10-07 keep resolving
through the lock's `derived` recipes; new runs record the dynamic_nfz form.

### 5. Circle fences and multi-scale geometry in the IR

`ir.FENCE_TYPES` = polygon_fence, circle_fence (the DSL's circumscribed 32-gon)
and dynamic_nfz. All three are compiled, indexed and enforced the same way.

Each zone carries three scales, computed once at compile time
(`PolicyIR.multiscale`): `fine` (the authored vertices), `coarse` (a polygon
that contains the fine one: its convex hull, or the minimum-area oriented
rectangle when the hull has more than 8 vertices) and `bbox`. Corridors carry
`fine` only: coarsening a keep-in shape outward would widen where a reader
believes it may fly. `geometry_ref(scale, rule_id)` names one scale of one
rule of this generation in the compiler's format
(`coarse:nfz-square@v0.1.0/g0`), and `resolve_geometry_ref` refuses a
reference minted for another version or generation. Since 2026-10-08
`compiler.multiscale_geometry` returns `PolicyIR.multiscale` (through
`compiler.compiled_ir`, one IR per policy hash) and computes nothing; the
compiler's own hull code is deleted, and its P0 refs are minted by the IR.

### 6. One rule checker

`Shield.rule_status(state, action=None)` reports every rule, in policy order,
from the Shield's own geometry and schedule: `in_force` (both sides of now),
`bound` (applies to the present situation), `breach` (standing still here
breaks it; for the speed rule, `action` does), `value`, `distance_m` (room
left; for a zone, the distance to its margin ring). Zones add `inside`,
`in_margin`, `in_band`, `vertices` (where the zone is now) and `margin_m`.
`zones_now(up=None)` and `fence_distance(x, y)` are the zone views.

- `demo/policy_hud.py` reads `rule_status` for every row and for the inset
  map. Its own `_poly_distance`, `_polyline_distance` and breach tests are
  deleted. Given the flight's Shield (`PolicyIndicator(..., shield=)`), it
  shows hot-applied rules as they arrive and rules out of their window as idle.
  Its rows and their values come from one `rule_status` snapshot (each row
  carries its `rule`): the first version read the rule list and the status in
  two calls, and an event landing between them (the REST thread) raised
  IndexError inside the control loop
  (`test_an_event_landing_mid_update_never_misaligns_a_row`).
- FenceGuard (`demo/follow_vlm.py`) reads `shield.zones_now()`: the Shield's
  compiled margin rings, in force now, a moving zone where it is this tick. Its
  own copy, built once at construction, never saw a zone that arrived later.
- The end-of-flight NFZ dwell count in `follow_vlm.fly()` reads `rule_status`
  for every static zone. It cannot count a MOVING dynamic_nfz: `rule_status`
  reads a zone where it is at the end of the flight, not where it was at each
  trajectory point. Such zones are left out and named (a printed line and
  `metrics["nfz_moving_not_counted"]`) rather than counted as zero. No flight
  has one yet (the loaders refuse dynamic_nfz); a per-tick count logged from
  `rule_status` during the flight is the fix when one does. Since
  2026-10-11 the other nine rails run the same count
  (`guardrail.replay.zone_dwell`), with the same moving-zone gap.
- The panel says what the Shield did, not what it saw. A rule the command
  breaks and the Shield did not correct is shown as WATCH (with the banner
  "RULE BROKEN - MONITORED, NOT CORRECTED"), never ACTING: a monitor_only
  rule always (the row's `effective_action`, from `rule_status`), and every
  violation of a decision with no repair and no brake (the log of a flight
  flown with the Shield off). "SHIELD CORRECTED COMMAND" and the "Shield
  corrections" count need a repair or a brake in the decision. Before the
  2026-10-07 review a violated monitor_only zone read ACTING / "NO-FLY ZONE -
  SHIELD CORRECTED COMMAND" while the command flew into it unchanged.

The panel's output on recorded flights. Re-rendered from each flight's own
`flight_log.jsonl` with the 401305a panel and with this one, same inputs as
`tools/rerender_policy_hud.py`: 88 flights (81,578 ticks), every row, banner,
count and map shape compared per tick. 2 flights (`vlm_nfz_smooth`,
`vlm_stopgo`) resolve to no policy and were not compared.

- Every flight flown with the Shield on: unchanged but for 1 tick
  (`citylife_redcar_nfz2`, tick 1880), where a zone about 156.5 m away reads
  "157 m away" instead of "156 m away", because the shapely ring's arcs are
  chords and the old panel measured the exact offset.
- The 9 flights flown with the Shield off (`ros2_ped_off`, `ros2_shield_off`,
  `ros2fix_off_fence`, `ros2fix2_off_fence`, `ros2if_off_fence`,
  `ros2if_off_fence_noavoid`, `sitl_ped_off`, `sitl_ped_side_off`,
  `sitl_shield_off`; 1,754 ticks) differ on every tick, on purpose: their
  logs carry the violations with no repair, and the old panel called them
  Shield corrections. The "Shield corrections" count now reads 0 instead of
  80-488; on 341 ticks a row reads WATCH instead of ACTING; on 547 ticks the
  breach banner ends "MONITORED, NOT CORRECTED" instead of "SHIELD
  CORRECTING". The banner level (the escape and breach events), the P0 escape
  count and the map are unchanged on all of them. Every differing tick is on
  or within the 1.5 s latch of a violation with no repair (0 unexplained).
  The same comparison against the panel as the 2026-10-07 review saw it gives
  exactly these 1,754 ticks and nothing else.
- The null: a panel that forgets the margin differs on 5,308 of 5,333 ticks
  of the four fence flights compared (`citylife_redcar_nfz1`,
  `citylife_redcar_nfz2`, `ros2_shield_on_dynamic`, `sitl_shield_on_dynamic`).

### 7. The audit record

`guardrail/audit.py` writes the grant's record (Safety Shield page, "Audit
log"): `ts`, `policy_hash` and `generation` (the decision's own, so a
hot-apply between ticks cannot restamp it), `monitor_tick`, `raw_action`,
`violations` with `boundary_intersect_at_s` and the grant's `category`
written as `grant_category` (geometric / envelope, and since 2026-10-12 the
grant's third, time-window, for a rule a window edge inside the lookahead
brings into force; section 9).
The record's own `category` field is NOT the grant's: it keeps this project's
finer vocabulary (geofence, corridor, altitude, kinematic, clearance,
standoff) for the readers that already use it, and `contract` (a non-finite
command zeroed) is this project's extension, written as itself under both
names. `repair_attempts` (operator, result ok / fallback /
relaxed, reason, `magnitude_m`, `axis`, `recovery`), `emitted_action`,
`fsm_state_before`, `fsm_state_after`, plus the edge, edge source, requested
mode, setpoint, reason and FSM config hash. Until 2026-10-12 the grant's
`iterations` was not written (the chain iterated as a whole and did not count
per operator); it is now measured, with every operator considered per pass,
and the record is a validated Pydantic model (section 9). A record is written when the
decision saw a violation and also when the FSM changed state, requested a
mode or refused its input, so a Brake -> Normal recovery on a clean tick is
on record. `AuditLogger.for_episode(dir, policy, episode_id)` writes one
`audit_<episode_id>.jsonl` per episode ("rotated per mission", Outputs table);
`rotate()` starts the next.

### 8. The first follow flight on ArduPilot (2026-10-08)

`demo/out/pas_ap_20261008_224440` (120 s red-car follow, Project AirSim +
ArduPilot SITL, Shield and FSM in the loop) left five P0 escape ticks and
eight Brake -> Loiter edges. What changed in the Shield because of it:

- **Range 0 to a stand-off subject.** The perception node had put the car
  under the aircraft on the five escape ticks, and the check could not see
  any action opening a range of 0. The check now reads the horizontal speed
  as the opening rate there; `StandoffRecover` leaves along the commanded
  heading (a hover: backwards). Replayed through the old Shield the log
  reproduces all five escapes on all 1101 ticks; through this one it has
  none. docs/FINDING-the-escape-that-stayed-inside-the-ring.md.
- **The best effort never deepens a hard ring or zone it is inside**
  (`_deepens`, relative to a moving zone; soft and `monitor_only` rules do
  not steer it), where before it was ranked by clearance alone.
- **`StandoffHold.magnitude_m` is a depth the ring can have** (the forecast's
  closest approach), where before 20 of the flight's 25 reported more than
  the ring's radius. Theta stays on this field (section 1). On the replayed
  flight the FSM's Brake -> Loiter edges go from 8 (as flown) to 2, and the
  repaired ticks over theta from 23 to 19: every one a StandoffHold whose
  forecast's closest approach is 2.1-4.3 m inside the 5 m ring. The
  one-period reading (opt-in) would trust all of them: 0 and 0.
- **`Shield.applied_caps`**: the caps the chain clamped against on this tick,
  for a rail's FSM measuring |dv| on the one-period reading. A
  `monitor_only` cap, one a time_window_switch turned off, or a soft cap the
  chain gave up (`_relax`, `tick.given_up`) is not one. Read off the whole
  policy, the first two made the FSM refuse the tick; a first version of this
  property left out every soft cap once any soft rule was given up, and the
  FSM then refused a tick whose soft P1 speed cap had clamped
  (`test_applied_caps_leave_out_only_the_soft_caps_the_chain_gave_up`).
- **`Shield.unsafe_depth(state)`**: metres from legal, summed over the hard
  rules a stop here breaks (an altitude error, a depth into a zone's ring, a
  stand-off or clearance ring, a corridor's half-width). The FSM's X6 reads
  its trend to leave a working recovery alone, and its X7 to send home a
  recovery that is losing ground (docs/DESIGN-escalation-fsm.md).

### 9. Headroom, the audit record as a model, the repair library (2026-10-12)

**The moving-zone budget.** At the grant's load (50 rules, 5 s at 0.1 s) the
48-moving-zone `_check` had a median of 2.6 ms on a quiet desktop, 6.5 ms on
a loaded one and 5.71 ms on GitHub's runner, against 5 ms, and the Orin will
be slower. Profiled (cProfile, 200 near samples): 70 % of it was building
each moving zone's polygon at the tick's time (`compile_fence`, a `buffer()`
each) - all 48 on every check, reachable or not, because the swept-box skip
of 2026-10-07 judged a zone's box only after building it. `ir.anchor_reach`
now bounds where a moving zone can be over the horizon from its anchor record
alone: a translating zone's margin ring is the anchor ring moved by its
velocity times its age; a turning one stays within (farthest vertex + margin)
of its moving centre, for the buffer GEOS builds at any rotation; a 1e-6 m
pad covers rounding. `Shield._reach` builds a zone only when that box meets
the forecast's, and then applies the old swept-box test to the built zone,
so the zones judged are exactly the old ones (the anchor box contains the
built zone's swept box: `test_the_anchor_box_contains_the_materialised_swept_box`;
the answers equal the old judgement on 3600 `_check` / `filter()` samples of
translating, turning and mixed zones aged 0-90 s). In 200 near checks the
zones built fall from 9600 to 199. Two savings on every path, both exact:
plain rules that share a weekly window read it once per pose
(`_Schedule._plain_key`), and forecast poses are a NamedTuple instead of a
validated State (a fifth of a static `filter()`; `forecast()` still returns
States). Numbers in "Measurements"; `tests/test_check_contract.py` is
unchanged and passes.

**The audit record.** `guardrail/audit.py` now builds an `AuditRecord`
(Pydantic v2, `extra="forbid"`) and validates it before writing; a record
with an undeclared key or a result nobody defined is refused and nothing is
written. Its JSON Schema is `policies/audit_record.schema.json` (`python -m
guardrail.audit schema`; a test fails when file and model disagree). New
fields, the old keys all kept: `raw_action_body` / `emitted_action_body` in
the grant's keys {vx, vy, vz, yaw_rate} (`frames.to_body` at the tick's
heading, `body_yaw_deg`; null for a decision that does not carry its
heading, never a guessed one), `repair_passes` (every repair operator
considered, pass by pass, in every run of the repair stack - each pass
carries its `run`, so a run that failed before soft rules were given up
stays on the record, and `flown_run` names the run whose action flew - with
result ok / skip / fail / fallback / relaxed and the reason for every skip),
the measured `iterations` (loop passes that ran, summed over the runs; 0 on
a passthrough) and `chain_runs`, `repair_mode`, `record_version` (2), a
`priority` per violation when the logger holds the policy, and `iterations`
on each `repair_attempts` entry: the pass of the flown run that first logged
that repair, operator and detail (`ShieldDecision.trace["repair_iterations"]`,
index for index with `repairs`, keeping `_dedupe`'s first-of-each rule). The
first version mapped that pass by operator name, so a second repair the same
operator logged in a later pass (the ClimbClamp that closes pass 1 after one
in pass 0) inherited the first one's pass: wrong on 1324 of 4257 records over
the 28 policies and the 50-rule load (150 seeded ticks each, CityLife map),
0 now. It also kept only the flown run's passes, so a soft-rule conflict
read chain_runs 2, iterations 1; it now reads 2 and 4 (review, 2026-10-13).
A violation of a rule
not in force now that the forecast breaks after a window edge brings it into
force is `grant_category: "time-window"` (`ShieldDecision.trace`, which the
Shield attaches to every decision and `model_dump()` leaves out, so logs and
digests are unchanged). The grant's example lists skips inside
`repair_attempts`; here they live in `repair_passes`, so `repair_attempts`
still means "repairs made" for the readers that count it. `read_audit`
reads all three shapes stored since July (no `emitted_violations`; no
generation or FSM fields; the grant's fields) into the model; what a shape
did not record stays null, never an empty list. It reads every audit stored
one or two folders under `demo/out`: 22123 records in 73 files (2026-10-13;
a reviewer timed a recursive walk at 62 s through the frame-folder
junctions; the bounded glob plus reading every record took 3.9 s here). An
old record's derived results (Brake and ClearanceEscape
"fallback", SoftRelax "relaxed") and `boundary_intersect_at_s` are pinned
by a test, on the one stored old-shape record that ends in a fallback and
on a synthetic July-shape record holding a Brake and a SoftRelax (no stored
record of those shapes holds either).

**The repair operator library.** `guardrail/repair.py`: a `RepairOperator`
Protocol (`name`, `categories`, `repair(action, violation, ir, *, ctx) ->
RepairAttempt`; `ctx` carries what the grant's three arguments leave
implicit, as the reference's `state` and `cfg` do), a registry, and the six
existing operators wrapped (KinematicClamp, AltitudeClamp,
ClearanceProjection, StandoffProjection, CorridorProjection,
LateralProjection), each calling its `_repair_*` method on the Shield by
name so a subclass override still runs. An operator written to the grant's
signature exactly (three arguments, no `ctx`, no `categories`) is accepted
by `register`, which wraps it (`GrantOperator`: it covers every category,
and a change it makes without logging a Repair is logged in its name), and
`Shield(repair_order=...)` puts a registered name to work, in place of the
iterated loop's stack or the first-feasible priority order (default None:
the stacks below). The default chain calls them in the
old order with the old arguments: its decisions equal a verbatim copy of the
old `_repair_chain` on every policy in policies/ (`test_the_default_chain_decides_as_the_old_chain_on_every_policy`,
1980 ticks, 1825 of them repaired, most by a speed clamp). Three modes, each off by default:

- `repair_mode="first_feasible"`: the grant's selection. Per iteration (up
  to 3) the earliest open violation is the target; operators are tried in
  priority order; the first whose action clears the target with every
  non-recovery repair's `magnitude_m` within theta (2.0 m lateral, 0.5 m
  vertical) wins; an attempt that says ok but leaves its target violated is
  a fail, and a recovery is exempt from theta. Head-on 6 m outside a zone at
  4 m/s the chain slides (a 13 m GeofenceSlide, left for the FSM to judge);
  first-feasible refuses it and the fallback stops. Sinking to 1.9 m under a
  floor, the chain's AltitudeFix is applied and first-feasible's is refused
  (over the vertical theta).
- `path_repair=True`: the grant's PathRepair, a detour waypoint logged in
  body frame (metres forward / right of the heading), taken when projection
  has lost at least half the commanded progress and only if the detour
  re-checks clean under every rule the chain enforces. A stand-off ring is
  passed at the tangent of a circle its soft margin (at least 1 m) outside
  it; a static zone at a corner of its margin ring's coarse ring pushed 1 m
  out. It frees the standoff-wedge scenario (FSM off, as tests/test_sweep.py
  flies the LEGACY set): without it the aircraft hovers at (0, 0) for the
  whole episode (300 StandoffHold repairs, goal not reached); with it the
  goal is reached, the closest range is 10.022 m against the 10 m ring and no
  tick flies a P0 violation (60 StandoffHold, 49 PathRepair; the 10.022 m is
  StandoffHold sliding along the ring on the ticks where projection keeps at
  least half the progress - the detour itself aims at the tangent of the
  12 m circle, pinned by a test). A detour whose leg would enter another
  zone is not taken. With the FSM
  on, the detour's magnitude (the 10 m the raw forecast went into the ring)
  is over theta and the FSM goes X1 > G3: theta's call, unchanged here.
- `segment_check=True`: the straight segment between consecutive forecast
  poses is judged against every zone's margin ring (its interior, the set the
  pose test reads, in the zone's frame for a moving zone) and every
  corridor's tube (a polygon that contains the true tube, so it never flags a
  segment the pose test would pass; it misses an exit shallower than 0.12 %
  of the half-width). At 12 m/s on the 0.1 s grid a 0.5 m ring between the
  poses at x = 3.6 and 4.8 m was flown straight through; it is now a
  violation at t + 0.321 s, and the slide repairs it. Off by default because
  it changes decisions.

Each mode keeps the budgets ("Measurements"). The 13 LEGACY scenarios and
the 8 Oct replay are unchanged with the flags spelled out off. Two
optimisations of the default path that no earlier test reached are pinned
against their memo-less form (review, 2026-10-13): the shared window memo
(`_plain_key`; rules on the same clock window but other days or another end,
a switched-off and a switched-on rule beside a plain one, at a window edge,
and 500 answers of the 50-rule edge load), and `unsafe_depth` measuring a
moving zone where it has moved to.

**The 8 Oct regression on CI.** `tests/test_flight_pas_ap_20261008.py` and
the stored-audit test read the run folder in `demo/out`, which git ignores,
so they reported SKIP on every CI run: the 8 Oct fix had never been checked
on GitHub. `tests/fixtures/` now holds the flight trimmed to what the replay
reads (all 1101 ticks, 18 row fields, 53 KB gzipped), its policy, the
CityLife 6-14 m map (1.7 KB) and a 12-record October audit; the full log is
still read where the folder exists, and a test checks fixture and folder
agree field for field. Shown to matter: with the 8 Oct range-0 fix reverted
in a scratch copy with no `demo/out`, the fixture replay fails with escapes on
exactly the five logged ticks; the HEAD test file there reports 0/2, 2 skipped.
`tests/test_shieldcore_identity.py` pins the 81 decisions 4418ae0 made on the
replay (the other 1020 ticks pass the raw action untouched). These files
make a CI run possible; they are not yet in the test list of
`.github/workflows/tests.yml` (the lead's file), and until they are listed
the regression still does not run on GitHub.

## Measurements

### 2026-10-13 (after the review of section 9)

The same bench, re-run back to back on three scratch copies: 4418ae0, the
section 9 code as reviewed, and this code (two rounds each; `--horizon 5
--dt 0.1 --variant <v> --impl current --n-check 300 --n-filter 150`, near
and grid; median / p99 `_check`, ms, FSM off). The desktop was quieter than
on 2026-10-12, so 4418ae0's moving figures are lower than the table below:

| variant | samples | 4418ae0 | reviewed | now |
|---|---|---|---|---|
| static | near | 0.346-0.347 / 0.74-0.75 | 0.293-0.296 / 0.68-0.69 | 0.296-0.299 / 0.68-0.71 |
| edge | near | 0.807-0.820 / 1.19-1.22 | 0.564-0.572 / 0.78-0.83 | 0.563-0.570 / 0.79 |
| moving (48 zones) | near | 2.526-2.642 / 3.06-3.51 | 0.447-0.452 / 0.85-0.87 | 0.448-0.454 / 0.88-0.89 |
| moving (48 zones) | grid | 5.908-6.160 / 11.66-13.65 | 0.297-0.304 / 0.91 | 0.299-0.302 / 0.92 |

`filter()` near, median: static 2.30-2.32 / 2.07-2.08 / 2.09-2.12 ms, edge
2.75-2.80 / 2.18-2.27 / 2.21-2.22 ms, moving 3.40-3.43 / 0.92-0.95 /
0.92-0.93 ms. So the review's fixes (the per-tick ledger, the per-repair
pass) cost nothing measurable. The nulls, this session, this code: the
per-fence loop 222.0 / 248.6 ms per `_check` and 1047.6 / 1083.9 ms per
`filter()` (50 samples each, near), every moving zone built and judged at
every pose 76.0 / 82.6 ms near and 79.4 / 99.5 ms grid per `_check`; decisions
identical to `current` on every compared sample (50 + 50, 300 + 150 + 300 +
150).

### 2026-10-12 (section 9)

Desktop as below, shared with three other jobs throughout; HEAD (4418ae0, a
pristine `git archive` copy) and this tree timed back to back in one
session, `experiments/bench_shield_50rules.py --horizon 5 --dt 0.1 --variant
<v> --impl current --n-check 400 --n-filter 150` (median / p99, ms, FSM off):

| variant | samples | `_check` HEAD | `_check` now | `filter()` HEAD | `filter()` now |
|---|---|---|---|---|---|
| static | near | 0.386 / 0.847 | 0.320 / 0.749 | 2.494 / 4.294 | 2.223 / 3.940 |
| static | clear | 0.211 / 0.345 | 0.153 / 0.210 | | |
| edge | near | 0.845 / 1.310 | 0.576 / 0.860 | 2.929 / 4.138 | 2.229 / 3.483 |
| moving (48 zones) | near | 6.925 / 9.995 | 0.446 / 0.914 | 8.859 / 19.746 | 0.905 / 4.096 |
| moving (48 zones) | grid | 5.673 / 8.865 | 0.295 / 0.960 | | |

The nulls, same session: the per-fence loop (`tests/legacy_fence_loop.py`,
static) 210.6 / 232.6 ms per `_check` and 1068 / 1128 ms per `filter()` at
this forecast; every moving zone built and judged at every pose (bench
`--impl allzones`) 77.5 / 89.3 ms and 86.7 / 395.4 ms, with decisions
identical to `current` on 400 + 150 + 400 + 150 + 400 + 150 samples. The test
methodology (`tests/test_shield.py`, 200 near samples, seed 3, FSM on),
medians: static 0.37 -> 0.31 ms, edge 0.83 -> 0.65 ms, moving 2.55-2.79 ->
0.43-0.49 ms; `filter()` 2.7 -> 2.4, 3.1 -> 2.9, 4.4-4.6 -> 1.7-2.1 ms.

With each new mode on (same test methodology, medians `_check` / `filter()`,
p99 `filter()`): first_feasible 0.31 / 2.90 / 3.9 static, 0.45 / 1.89 / 21.5
moving; path_repair 0.32 / 6.00 / 7.5 static, 0.44 / 1.79 / 5.0 moving;
segment_check 0.31 / 2.40 / 3.8 static, 0.45 / 1.73 / 5.1 moving; all three
0.31 / 7.33 / 11.5 static, 0.46 / 2.05 / 24.1 moving. Every `_check` median
is under 5 ms and every `filter()` p99 under 100 ms
(`test_every_new_mode_keeps_the_50_rule_budget`). PathRepair triples the
static `filter()` median: on the near samples it runs on most ticks (head-on
into a zone), with a monitor check per candidate waypoint. The heaviest tail
is first-feasible on the moving zones: where it refuses an over-theta slide,
the rescue search runs (24 headings, a monitor check each), 20-25 ms on a
quiet moment; a reviewer's loaded desktop read 79 ms for all three modes
there, and on 2026-10-13 this desktop, loaded by three other jobs, read
65.4 ms once. The test therefore re-measures a configuration that is over a
bound once and judges the better run, and prints its figures.
`tests/test_shield.py`'s 50-rule test keeps its 15 ms `_check` bound on a
shared CI runner (5 ms elsewhere) until a GitHub run has printed the three
medians it now prints; the new path has not been timed on GitHub.

### 2026-10-07

Desktop (Intel64 Family 6 Model 183, Python 3.10.20, shapely 2.1.2 / GEOS 3.13.1), shared
with other work while measuring; results swing up to 2x between runs.
50-rule bench load, 5 s at 0.1 s:

| path | `_check` p99 | `filter()` p99 | budget |
|---|---|---|---|
| static zones, near / grid / clear | 0.80 / 0.78 / 0.26 ms | 4.25 / 3.94 / 1.33 ms | 5 ms / 100 ms |
| the same samples, no zones (floor) | 0.10-0.11 ms | 0.52-0.54 ms | |
| every rule with a window edge inside the lookahead | 0.80-1.24 ms | 1.82-4.17 ms | |
| 48 moving zones (1 m/s), with the swept-box skip | 2.92-4.24 ms (max 6.68) | 3.75-7.45 ms | |
| 48 moving zones, without it (first version) | 34.0-35.6 ms | 152-155 ms | over |

Re-measured after the review's fixes (2026-10-07, ~13:30); the earlier run
of the same code paths gave 0.81 / 0.79 / 0.28 ms and 4.31 / 4.16 / 1.30 ms.
The moving-zone `_check` p99 reached 4.24 ms on the near samples in this run
(3.78 ms before), within the 5 ms budget but with the least room of any row.
Shield construction at 50 rules: median 4.7 ms (2.7 ms at 401305a; the
coarse rings and the runtime are built here; the grant's cold start is
<= 1 s). Commands: `python experiments/bench_shield_50rules.py --impl current
--horizon 5 --dt 0.1`; the window-edge and moving-zone rows come from a
scratch variant of that bench and are pinned by
`tests/test_shield.py::test_the_50_rule_monitor_fits_the_grant_budget`.

The repair stack itself did not change behaviour: with the FSM off and the
record-only fields (`magnitude_m`, `axis`, `recovery`) left out, every
`filter()` decision equals the frozen pre-IR fence loop
(`tests/legacy_fence_loop.py`) on 180/180 bench samples, and
`tests/test_ir.py` checks the same on every shipped policy.

## Shown failing first

The unit's first tests, run against the 401305a versions of the seven owned
source files (everything else as now); the tests added after the review are
in the next subsection:

| test file | on 401305a | now |
|---|---|---|
| tests/test_shield.py | 38/61 (23 new tests fail) | 61/61 |
| tests/test_shield_events.py | 0/24 (ImportError: no HotApplyRefused) | 24/24 |
| tests/test_ir.py | 0/19 (ImportError: no as_dynamic_nfz) | 19/19 |
| tests/test_api.py | 0/8 on 3.11 (no `apply_*` handlers; the new routes 404) | 8/8 on 3.11 (6/8 + 2 visible SKIPs on 3.10, no fastapi) |
| tests/test_fsm.py | 94/96 | 96/96 |
| tests/test_policy_hud.py | 36/43 | 43/43 |
| tests/test_fenceguard.py | 23/27 | 27/27 |

Several of those fail on a missing keyword. The behaviour itself, probed on
both versions with the same inputs:

| probe | 401305a | now |
|---|---|---|
| FenceGuard speed scale after a zone is added mid-flight, its margin ring 5 m ahead | 1.0 (unseen) | 0.22 |
| ticks (of 141) on which a zone drifting at 2 m/s toward a hovering aircraft is seen | 0 | 43 (from t = 9.8 s) |
| panel row for an aircraft inside a circle_fence | "ok", not on the map | "breach", "INSIDE", drawn |
| audit records for a repair then a recovery / grant keys missing | 1 / 5 | 2 / 0 |
| soft-rule conflict (hard 0-10 m, soft 20-30 m P2, soft 3-8 m P1, at 8 m, vx 1) | flies a 4 m/s rescue heading | keeps vx 1, gives up the P2 rule |
| a 07:30-17:30 zone at 17:30:30 / 17:30:59 | in force / in force | off / off |
| hovering in that zone at 07:29:57 | no violation | predicted at t + 3.0 s |

Two later fixes in this unit were shown failing on the code before them:
`test_a_moving_zone_out_of_reach_costs_no_point_test` (312 point tests on a
zone 400 m away, now 0) and
`test_an_event_that_would_leave_the_policy_inconsistent_is_refused` (an expire
that left a dangling switch was accepted).

The tests were then checked by mutation: 33 deliberate breaks of the owned
code (eps = 0, a window end read to the minute, polygon_fence accepted after
take-off, the swept box without velocity or rotation, the lint validator
removed, a slide reporting 0 m, the audit stamping the live hash, FenceGuard
ignoring windows, and so on), each run against the six owned test files that
need no 80 s IR sweep. The first run caught 25 of 32; six of the seven
survivors pointed at a missing test (priority order where either soft rule
would do, a turning zone, a stop inside a soft zone, a repair acting on an
event that landed mid-tick, a record logged after an event, escaping a
moving zone), which now exists and catches it. The seventh
(`enforced = list(violations)`) is equivalent: monitor_only rules are also
left out of the repair chain by its skip set, and a stronger form that drops
the skip too is caught by
`test_monitor_only_is_left_alone_when_another_rule_is_repaired`. Final:
32 of 33 caught, 1 equivalent.

### After the 2026-10-07 review

An independent review re-ran the mutation idea with its own 39 mutations and
found 9 documented behaviours no test pinned, plus three defects (the layer
rule, the panel's "corrected" claim, a moving zone teleporting on reset). The
new tests, run against the owned sources as the review saw them (everything
else as now; scratch copy):

| test file | review-time code | now |
|---|---|---|
| tests/test_shield_events.py | 27/32: the 5 new tests of the layer rule and the reset fail (the 3 coverage tests pass) | 32/32 |
| tests/test_api.py | 6/10 on 3.10 (the REST layer test fails), 8/10 on 3.11 (that test and its route twin fail) | 7/10 + 3 visible SKIPs on 3.10, 10/10 on 3.11 |
| tests/test_policy_hud.py | 43/45 (both "not corrected" tests fail) | 45/45 |
| tests/test_shield.py | 66/66: the 5 new tests pin behaviour that was already right | 66/66 |

The coverage tests are checked the other way, by mutation: the review's 9
surviving mutations, re-applied to the current code, and one mutation per fix
of this pass (11: each layer check removed in turn, the REST layer claim
accepted, the reset re-anchoring skipped, a between-episode read at t = 0,
the panel's WATCH shown as ACTING, any violation counted as a correction,
the uncorrected-log rule and the WATCH banner removed). 20 of 20 caught,
each by the test written for it (`scratchpad` script `mutrun3.py`, six owned
test files).

### 2026-10-12 (section 9)

Run against a pristine `git archive` of 4418ae0 with the new test files and
fixtures copied in (scratch copy):

| test | on 4418ae0 | now |
|---|---|---|
| tests/test_shieldcore_repair.py | ImportError: no `guardrail.repair` | 24/24 |
| tests/test_shieldcore_audit.py | ImportError: no `AuditRecord` | 11/11 |
| tests/test_shieldcore_identity.py | 5/7: the flags and `anchor_reach` do not exist; the 8 Oct pin, the fast-path comparison and the memo, edge-load and moving-depth tests pass by construction (they pin what 4418ae0 did) | 7/7 |
| `test_a_50_rule_moving_check_builds_only_the_zones_it_can_reach` | fails: 9600 zones built in 200 checks | 199 |
| `test_the_frames_docstring_names_every_module_that_crosses_the_boundary` | fails: "Nothing in production" | passes |
| tests/test_flight_pas_ap_20261008.py with no demo/out, as on CI | 0/2, 2 skipped (the file as it was) | 2/3, 1 skipped (the folder-vs-fixture check) |

(Counts are the files as they stand after the review below.)

The behaviour the new modes change, probed on 4418ae0: a 12 m/s command at a
0.5 m ring lying between two poses flies straight through it, no violation; a
window-edge violation is written `grant_category: "geometric"`; the record
has no body-frame key and no `iterations`. Mutations, each in a scratch copy:
the 8 Oct range-0 fix reverted makes both fixture replays fail (escapes on
ticks 857, 927, 928, 999 and 1100); `anchor_reach` without the zone's
displacement fails the containment and fast-path identity tests, and without
the horizon sweep both again; two chain stages swapped fails the old-chain
comparison on its 10th follow_car_citylife tick. One regression the first
version had and the suite caught: `_check` no longer read `_fence_records`, so
tests/test_profile_shield_tick.py's fence-blind Shield was no longer blind
(its test failed); `_fence_anchors` now reads through `_fence_records` when a
subclass overrides it or no zone moves, and `_reach` asks `_may_reach` of the
anchor record first, so a subclass whose `_may_reach` says True still judges
every zone at every pose (tests/test_shield_events.py's prefilter test).

### After the 2026-10-13 review

Two reviews of section 9 found one defect and a set of behaviours no test
pinned. The defect: `repair_attempts[].iterations` was mapped by operator
name, so a second repair the same operator logged in a later pass carried
the first one's pass (1324 of 4257 seeded records; section 9). Fixed, with
the ledger now holding every chain run. Also from the reviews: a 3-argument
operator is accepted (`GrantOperator`) and `repair_order` can run a new
name; the bench's `--flags` no longer reports a FAIL against a flag-less
null; the stored-audit test globs two levels instead of walking the
junctions; `tests/test_shield.py` keeps 15 ms on CI and prints its medians;
`test_the_replay_is_the_same_with_every_new_repair_flag_spelled_off` was
dropped (it compared the defaults with themselves; the 4418ae0 pin in
`tests/test_shieldcore_identity.py` is the identity test).

The new tests against the section 9 code as the reviews saw it (a scratch
copy of 4418ae0 with that code and the new tests; no demo/out):
`tests/test_shieldcore_audit.py` 9/11 - the per-repair pass test fails with
the name-mapped `[..., ("GeofenceSlide", 1), ...]` where 2 is right, and the
every-run test finds no `flown_run`; `tests/test_shieldcore_repair.py` 22/24
- the 3-argument operator is refused (TypeError) in both tests that use one;
the rest pass, as they pin behaviour that was already right.

Those are checked the other way, by mutation (`scratchpad` script
`mutants.py`, each mutation alone in a scratch copy of 4418ae0 plus this
unit's files, the owning test file run): 30 mutations, the reviews'
surviving ones and one per fix - the per-repair pass constant, last-pass or
name-keyed; `chain_runs` or `iterations` hard-coded to 1; the fallback filed
in pass 0; only the flown run kept; the window memo shared with a switch
target or a swap target, or keyed without the days or without the end; the
time-window label without its not-in-force-now test; `unsafe_depth` on a
moving zone's anchor geometry; first-feasible targeting the latest
violation, holding a vertical repair to the lateral theta, accepting an ok
that leaves its target violated, without the recovery exemption or the
category skip; a hook "ok" only when it logged a Repair; PathRepair without
its clean re-check, its leg-visibility filter or its waypoint clearance, or
at the end of the stack; `register` refusing the grant's signature; the
segment check needing the rule at both ends, ignoring the altitude band,
on an unenlarged corridor tube, or sizing its slide at 0 m; the reader
deriving every old result as ok or dropping `boundary_intersect_at_s`. 30
of 30 caught. One needed a second try: a memo keyed without the end
survived the first memo test, whose sharing rule sat outside the forecast's
box and so was read only at the first pose time; the sharer now sits inside
the box, beside the path. Equivalent and not tested: PathRepair's speed cap
(the KinematicClamp after it caps the speed anyway).

## Findings: the silent defects this unit fixed

CONTRIBUTING (section 3) asks for a finding document per silent defect. This
unit may create only this design document, so each is written here in that
form; the separate files are listed under "Needed outside this unit".

**A window ending 17:30 lasted until 17:30:59.** *Claimed:* rules are in
force inside their `valid_time`. *Found:* reading `models.Recurrence.active_at`
for the t- / t+ card (WP3-05): it compares whole minutes with an inclusive
end. *Evidence:* the 07:30-17:30 zone was in force at 17:30:30 and at
17:30:59 on 401305a, and the forecast never judged a pose at its own time
(hovering in it at 07:29:57 raised nothing). *Fix:* the Shield's own
evaluator, closed, to the microsecond, per pose, both sides of the instant
(section 3); `test_a_window_ending_17_30_ends_at_17_30_00`,
`test_the_lookahead_reads_the_rules_in_force_at_each_poses_time`.
`models.py` still reads minutes (outside this unit).

**The controller could not see a zone added in flight.** *Claimed:* the
controller's FenceGuard mirrors the policy's zones. *Found:* wiring the HUD to
`rule_status`: FenceGuard built its rings once, at construction. *Evidence:*
a zone added mid-flight with its margin ring 5 m ahead: speed scale 1.0
(unseen) on 401305a, 0.22 now; a moving or out-of-window zone was equally
invisible or wrongly visible. *Fix:* `Shield.zones_now()` (section 6);
`tests/test_fenceguard.py`.

**The panel showed a circle_fence as OK.** *Claimed:* every rule of the
policy is on the panel with its status. *Found:* adding circle_fence
enforcement. *Evidence:* aircraft inside a circle_fence: "ok" and nothing on
the map on 401305a; now "breach", "INSIDE", drawn as its 32-gon. *Fix:* the
panel reads `rule_status`; `test_a_circle_fence_is_drawn_and_judged_like_any_zone`.

**A mission event could switch off a regulation zone.** *Claimed:* a
mission-layer rule cannot relax a regulation-layer hard rule (grant; enforced
by `merge_layers`). *Found:* the 2026-10-07 review (probe on a merged
regulation + mission policy). *Evidence:* the REST switch-off of hard
`nfz-airport` was accepted twice (generation 1, then 2), after which a 4 m/s
command into the zone raised no violation and flew unrepaired; a swap of a
hard regulation corridor to a soft, monitor_only, 2 km wide one was accepted
and a pose 300 m off the corridor and 500 m up raised nothing. *Fix:* the
layer rule on every event (section 4); five tests in
`tests/test_shield_events.py`, two in `tests/test_api.py`.

**The panel said "corrected" when nothing was.** *Claimed:* the on-screen
panel states what the Shield did. *Found:* the 2026-10-07 review. *Evidence:*
a violated P1 monitor_only zone, flown unchanged: row ACTING, banner
"NO-FLY ZONE - SHIELD CORRECTED COMMAND (corrected)", `repaired` count + 1.
The same claim was made on every tick of a Shield-off flight's log whose
violations carried no repair (re-render figures in "Measurements"). No
shipped policy uses monitor_only, and no rendered video was found beside the
Shield-off flights in `demo/out` (they are SITL / ROS 2 runs), so no video is
known to carry the claim. *Fix:* WATCH (section 6);
`test_a_monitor_only_rule_is_watched_never_shown_as_corrected`,
`test_a_violation_with_no_repair_is_not_a_correction`.

## Interpretations where the grant is silent

| question | choice | why |
|---|---|---|
| eps for the t- / t+ check | dt / 2, two samples per pose | every window edge in the horizon is seen from both sides; a window shorter than eps (zero-length in particular) can be missed |
| window end | closed, to the microsecond; 23:59 = end of day | the grant's "17:30" reads as 17:30:00; the DSL's default end must not open a nightly gap |
| which breach actions skip projection | brake, loiter, RTL, land (the reference: RTL and land only, and it emits the raw action) | follow-up #112 / docs/DESIGN-escalation-fsm.md; a stop is the conservative response while the mode change is requested |
| a stop action where stopping is illegal | recover, request the mode, withhold the stop | a stop there is the deadlock the Shield exists to avoid |
| soft | enforced, capped at brake, given up first (by priority) in a conflict | the escalation FSM's reading (docs/DESIGN-escalation-fsm.md); PI question open |
| theta's metric | per-operator penetration depth into the margin ring (the reference: into the authored polygon, no margin); the setpoint change over one 0.1 s period is selectable (`theta_horizon_s=0.1`), not the default | the grant's p5 audit example and p3 LateralProjection read as a depth, and under the shipped caps the one-period reading can never fire (docs/DESIGN-escalation-fsm.md); PI question open |
| range 0 to a stand-off subject | every horizontal motion opens the range at its full speed; a hover recovers along the reverse of the heading | the one-sided derivative of the range at the subject, and what a one-second probe of the range (the trend test the PI reference takes for a zone) would read |
| best effort where nothing re-checks clean and a stop is illegal | the heading that goes deeper into no hard ring or zone the aircraft is inside, then clearance, then the commanded heading | "never deepen" is the least a P0 fallback must keep; the grant is silent |
| polygon_fence before the first tick | accepted (still the mission-start policy) | "locked at mission start" |
| switch / swap without a readable window | may turn on, never off; a swap enforces both | nothing that cannot establish "off" may switch a rule off |
| an event's layer | its `layer` field, absent = mission; REST events are always mission | the DSL's default layer; nothing on the REST channel authenticates a higher layer |
| a mission-layer event on a hard mission-layer rule | allowed (switch-off, any swap, any edit) | merge_layers allows the same; the grant's time_window_switch exists to deactivate rules |
| moving zones across reset_episode | continue from where they are | an event is part of the policy's history, and the zone is a thing in the world |

## Needed outside this unit

- `experiments/sweep_scenarios.py`: hot-apply `as_dynamic_nfz(fence)` instead
  of the PolygonFence (4 tests in `tests/test_sweep.py` fail on the lock), and
  read the switch instant from the Shield (`rule_status(...)["in_force"]` or
  `shield.recurrence_active`), not `ConstraintBase.active_at`.
  `experiments/scenarios.yaml` time-window-closes-mid-run: the description
  pins the old 17:30:59 instant as "the Shield's"; the clocks should move to
  17:29:40 / 17:29:50 so the window still closes 20 s / 10 s in.
- `experiments/bench_shield_50rules.py`: compare current and legacy with
  `escalation=False` and without the record-only repair fields, as
  `tests/test_ir.py` does; with the FSM on, the frozen legacy loop's repairs
  carry no magnitude and its FSM faults, so grid/filter and near/filter report
  0/60 identical though the repair stack agrees 180/180.
- `guardrail/models.py`: `Recurrence.active_at` to the Shield's reading (or
  calling `shield.recurrence_active`), so the CSP and the Shield agree on a
  window's last instant.
- ~~`guardrail/compiler.py`: read `PolicyIR.multiscale` and emit
  `PolicyIR.geometry_ref(...)` in P0_constraints instead of recomputing.~~
  Done 2026-10-08 (`docs/DESIGN-prefix-compiler.md` section 9).
- `docs/DESIGN-escalation-fsm.md`: lines saying shield.py has no
  `magnitude_m` and was not changed are now historical.
- The flight rails (`sitl/ros2_shield_node.py`, `sitl/run_sitl_demo.py`) build
  the Shield without `now=`, so no time window is evaluated in flight (card
  WP3-02), and call `filter()` without `t=`, so a moving zone's age is counted
  in 0.1 s ticks rather than read from the rail's clock. Both are one keyword.
- `demo/follow_vlm.py` (its Shield construction), `demo/run_demo.py` and
  `demo/real_vla_demo.py` build the Shield with the FSM on but fly `emitted`
  and pass 3 s / 0.5 s. Their audit files now carry an FSM verdict the demo
  never acted on; pass `escalation=False` (as the rails do through
  `rail_shield`) or fly `command`, and move to the grant's 5 s / 0.1 s.
- `demo/run_demo.py --dynamic` calls `shield.hot_apply(DYNAMIC_FENCE)`, a
  PolygonFence, after ticks have run (~line 202): it now raises
  `LockedRuleClass`. Use `shield.hot_apply(as_dynamic_nfz(DYNAMIC_FENCE))`.
  Its `audit.policy_hash = ...` assignments fail already at 401305a (a
  property with no setter); drop them, the decisions carry their own hash.
- `tools/profile_shield_tick.py` (~line 833) counts `bool(dec.emitted_violations)`
  as an escape. That list also holds monitor_only rules and soft rules given
  up, of any priority; filter to P0 rules as `guardrail/kpi.py` does.
- `guardrail/models.py` lint: refuse `start_time == end_time` in a
  recurrence (an instant to the Shield, a minute to `Recurrence.active_at`,
  arguably a whole day to an author; the Shield cannot see an instant).
- `experiments/bench_shield_50rules.py`: its hot-apply row adds a
  polygon_fence BEFORE the first tick (the `add_before_start` path). A
  post-start `as_dynamic_nfz` spawn row would measure the mid-flight path
  the grant budgets; the figure in "Mid-flight events" is from a scratch
  script until then.
- Finding documents (CONTRIBUTING, section 3) for the silent defects in
  "Findings" below: `docs/FINDING-a-window-ending-1730-lasted-until-173059.md`,
  `docs/FINDING-the-controller-could-not-see-a-zone-added-in-flight.md`,
  `docs/FINDING-the-panel-showed-a-circle-fence-as-ok.md`,
  `docs/FINDING-a-mission-event-could-switch-off-a-regulation-zone.md`,
  `docs/FINDING-the-panel-said-corrected-when-nothing-was.md`. This unit may
  create only this design document, so each is written below in the FINDING
  form.

## Not done

- No flight or SITL run exercised any of this; every number above is
  offline.
- `distance_envelope` is enforced by nothing (refused at construction), and
  MSL heights have no conversion.
- The Shield's own FSM never hears from an autopilot unless the caller passes
  `rtl_failed` / `home_reached` / `landed`; the rails use their own FSM.
- The sliding window (`history`) is still a record; no rule reads past actions.
- PathRepair and the segment check exist (section 9) but are off by default,
  and no flight has flown either. PathRepair's detour has no planner behind
  it (one waypoint, a tangent or a hull corner); it leaves moving zones and
  corridors to projection, and with the FSM on theta still escalates the
  head-on cases it was built for. A segment found by the segment check
  inside a corridor bend has no repair operator of its own (CorridorReturn
  reads the lateral offset, not segments): the chain does not converge and
  the rescue / brake fallback decides.
- First-feasible mode reads theta from the Shield's FSM configuration (the
  grant's 2.0 / 0.5 m without one); whether the grant's selection should
  replace the chain is a decision for the PI, not taken here.
- The 2026-10-12 and 2026-10-13 timings are this desktop's; nothing was
  measured on the Orin or on GitHub's runner. `tests/test_shield.py` keeps
  15 ms on CI until a run prints the new medians, and
  `test_every_new_mode_keeps_the_50_rule_budget` has never run on a shared
  runner (its heaviest configuration read 65 ms once here under load,
  against 100 ms).
- `tests/test_flight_pas_ap_20261008.py` and the three
  `tests/test_shieldcore_*.py` files are not in `.github/workflows/tests.yml`
  (the lead's file), so CI does not yet run the 8 Oct regression.
- `repair_passes` skip reasons come from the violations open at the start of
  the pass, not a re-check per operator. `time-window` is written only for a
  window that opens inside the lookahead; a closing edge cannot cause a
  violation. A record's per-violation `priority` is read from the logger's
  live policy, not the decision's generation.
- The jsonschema check of written records runs only where jsonschema is
  installed (not a project dependency; it says so when it skips).
- A window shorter than eps (zero-length in particular) is not guaranteed to
  be seen (section 3).
- The REST channel has no authentication; the layer rule only stops it from
  claiming a layer above mission. A mission-layer sender can still switch off
  any mission-layer rule, hard or not, as the grant's time_window_switch
  allows.
- The end-of-flight NFZ count in `follow_vlm.fly()` does not count moving
  zones (named, not zeroed; section 6).
