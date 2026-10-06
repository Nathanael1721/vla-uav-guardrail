# The escalation state machine: Brake, Loiter, RTL, Land

**Date:** 2026-10-06 (second review round the same day)
**Status:** the pure module is built and tested: `guardrail/fsm.py`, with
`tests/test_fsm.py` at 95/95 on Python 3.10 and 3.11, and 69/69 deliberate
defects caught (see "How the tests were checked"). It also includes a
read-only replay of delivered flight logs (`python -m guardrail.fsm replay`).
**Nothing in the flight path calls it yet.**
This unit did not change `guardrail/shield.py` or `sitl/ros2_shield_node.py`.
No flight has used the FSM, and no KPI number has changed. The integration is
a later wave, and the section "Integration, call site by call site" exists so
that wave is mechanical.
**Cards:** WP3-07 (this module). Groundwork for WP3-06 (theta cap), WP3-08 and
WP1-10 (breach actions), WP1-11 (hard/soft), WP1-12 (priority), and WP3-15
(fail-safe KPI).

## The gap, stated exactly

The grant's Safety Shield repairs an unsafe action when the repair is small,
and steps up through fail-safes when it is not. The second half never existed
here. `Shield.filter()` has one fallback, a zero-velocity BRAKE. No code
changes the ArduPilot flight mode during a mission. The only `SET_MODE` calls
are GUIDED at bring-up and LAND at mission end, and neither is the Shield's.
`brakes = 0` on all five KPI runs, so "fail-safe trigger correctness" in
`guardrail/kpi.py` was one minus the escape rate, a number that reads 1.0 for a
Shield with no fail-safe at all.

## What the grant asks for

Safety Shield PDF p4, Escalation FSM. The mermaid source is
`kuanting-vla-uav-guardrail/docs/02-implementation/safety-shield.md`:

```
Normal --> Brake   : violation, repairable, magnitude < θ
Brake  --> Loiter  : N-in-T threshold OR magnitude > θ
Loiter --> RTL     : N-in-T persists OR operator-defined timeout
RTL    --> Land    : RTL fails (battery / blocked path)
Brake  --> Normal  : cleared for ≥ T_recover
Loiter --> Normal  : cleared for ≥ T_recover
RTL    --> [*]     : home reached
Land   --> [*]     : landed
Normal --> RTL     : violation_action == RTL (direct), unrepairable
Normal --> Land    : violation_action == Land (direct), unrepairable
```

**Defaults:** N = 3, T = 5 s, θ = 2.0 m lateral / 0.5 m vertical,
T_recover = 2 s, "config-file overridable per mission profile" (p4). The
Outputs table (p7) asks for a "Python state machine; YAML-overridable
thresholds".

Related grant text this module follows:

- p1: "detect violations → repair via projection → trigger fail-safe (RTL /
  Land) when repair is unsafe".
- p2, node diagram: "Converged? magnitude < θ?" → *no* → "Escalation FSM
  (Brake → Loiter → RTL → Land)" → "Mode setter SET_MODE
  GUIDED/LOITER/RTL/LAND".
- p3–p4: the first operator "with magnitude ≤ threshold wins". Conservative
  cap: "If ‖action_repaired − action_original‖ > threshold, or no operator
  converges, the Shield abandons projection and triggers fail-safe."
- p5, audit record: `fsm_state_before`, `fsm_state_after`, and
  `repair_attempts[].magnitude_m`.
- Policy DSL PDF p2: `constraint_type: hard | soft`, `priority: P0 | P1 | P2`,
  `violation_action: monitor_only | project_fix | brake | loiter | RTL | land`.
- Prefix Compiler PDF p4: "severity: hard ≫ soft, P0 > P1 > P2".
- Stress Testing PDF p5–p6: outcome ∈ {success, fail, RTL_triggered,
  Land_triggered}. Fail-safe trigger correctness ≥ 99 %, "triggered when
  expected, not when not expected".
- Grant overview p1, WP3 row: "ArduPilot fence / RTL / Land / Loiter as
  ultimate backstop".

## States, and what each asks of ArduPilot

| State | ArduPilot mode | Setpoint streamed | Meaning |
|---|---|---|---|
| Normal | GUIDED | `pass` (the Shield's emitted action, i.e. the raw one when nothing is violated) | nothing enforced is violated |
| Brake | GUIDED | `pass` (the repaired action) when the repair is trusted; `brake` (zero velocity) when it is not, or when a rule asks for a stop. Exception: `pass` when a stop is illegal here (`stop_illegal`) | the Shield is intervening |
| Loiter | LOITER | `none` | the autopilot holds position |
| RTL | RTL | `none` | the autopilot flies home |
| Land | LAND | `none` | the autopilot lands where it is |

Brake is **not** an ArduPilot mode. It is the Shield intervening inside
GUIDED. A trusted repair is still flown in Brake, which matches the PI's
reference (`safety_shield/shield.py`: state BRAKE, emitted = the repaired
action). On the tick that hands the aircraft from GUIDED to an autopilot mode,
the setpoint is `brake`, so the last GUIDED setpoint is not still flying while
SET_MODE goes through. After that tick it is `none`. `set_mode` is
edge-triggered: it is sent once, on the transition, never on every tick.

**A stop the Shield calls illegal is never streamed.** `stop_illegal` means
standing still here would break a **hard, enforced** rule, as it does inside
a no-fly polygon, a clearance ring, or under the altitude floor. The node
passes `Shield.state_is_unsafe(state)` (the list of violations) and
`tick_input_from_decision` keeps only the hard, enforced rules. A soft rule is
capped at `brake`, so a stop that breaks only a soft rule is a response the
soft rule already allows. On such a tick every `brake` above becomes `pass`,
which streams the Shield's own action. For a blocked tick that is the
Shield's BRAKE where a stop is legal and its best-effort recovery where it is
not, because shield.py brakes only "where standing still is legal". The state
still escalates exactly as the table says. Only the setpoint changes, and the
record carries `stop_withheld: true`. The reference calls the alternative,
stopping inside the zone, the "brake while inside = deadlock" failure.

## Transition table

`G` edges are the grant's. `X` edges fill gaps where the grant is silent. Each
`X` edge is labelled `extension` in the audit record, so a choice of ours is
never presented as a contract term. One transition per tick at most.

| Edge | From → To | Condition | set_mode | Source |
|---|---|---|---|---|
| G1 | Normal → Brake | enforced violation, repair converged, magnitude ≤ θ | – | grant p4 |
| G2 | Brake → Loiter | ≥ N hard-rule onsets within the last T s | LOITER | grant p4 |
| G3 | Brake → Loiter | a hard, non-kinematic `project_fix` rule's repair is over θ, or did not converge | LOITER | grant p4 (diagram + conservative cap) |
| G4 | Loiter → RTL | ≥ N new hard onsets within T, counted from Loiter entry | RTL | grant p4 |
| G5 | Loiter → RTL | time in Loiter ≥ `loiter_timeout_s` | RTL | grant p4 ("operator-defined timeout") |
| G6 | RTL → Land | autopilot side reports `rtl_failed` | LAND | grant p4 |
| G7 | Brake → Normal | nothing enforced violated for ≥ T_recover | – | grant p4 |
| G8 | Loiter → Normal | no **hard** enforced violation for ≥ T_recover | GUIDED | grant p4 |
| G9 | RTL → [*] | `home_reached` | – | grant p4 |
| G10 | Land → [*] | `landed` | – | grant p4 |
| G11 | Normal → RTL | a violated rule's effective action is RTL | RTL | grant p4, with "unrepairable" read as "skips projection" (see Interpretations) |
| G12 | Normal → Land | a violated rule's effective action is land | LAND | grant p4, same reading |
| X1 | Normal → Brake | enforced violation whose repair is over θ or did not converge, or a `brake` rule; setpoint `brake` (`pass` if `stop_illegal`) | – | extension: p2 routes "not converged" into the chain, whose first state is Brake |
| X2 | Brake, Loiter → RTL | a violated rule's effective action is RTL | RTL | extension: a P0 RTL rule cannot be ignored because a P2 repair put the FSM in Brake one tick earlier |
| X3 | Brake, Loiter → Land | a violated rule's effective action is land | LAND | extension, same reason |
| X4 | Normal, Brake → Loiter | a violated rule's effective action is loiter | LOITER | extension: the DSL allows `loiter`; the diagram never says what it does |
| X5 | RTL → [*] | `landed` during RTL without `home_reached` | – | extension: ArduPilot's RTL ends by landing at home |
| X6 | Brake, Loiter → RTL | `stop_illegal` on every tick for ≥ T, and a hard rule violated on this tick | RTL | extension: p1 "trigger fail-safe when repair is unsafe". See "X6" below |

The order of checks inside each state is fixed, and it is the order the table
is resolved in when two conditions hold at once. It follows the strongest
response. **Brake:** direct land, direct RTL, X6, direct loiter, then G3,
G2, G7. **Loiter:** direct land/RTL, then G4, G8, G5, X6. Recovery wins over
the timeout when both hold on the same tick. **RTL:** G9, X5, G6, so a tick
that reports home together with an RTL failure is a finished RTL. In RTL and
Land the pilot's proposals do not move the FSM. The autopilot is flying, and
the grant draws no edge out of RTL for them.

### X6: a position that stays illegal

The first review round found that one case could never escalate. An aircraft
held inside a P0 zone (a wind beyond the 4 m/s speed cap, say) gets a
`GeofenceEscape` every tick. The escape re-checks clean, so the tick is a
trusted repair (G1). Recovery operators are exempt from θ, so G3 cannot fire.
The violation never clears, so it is one N-in-T onset, and G2 cannot fire.
Measured on the real Shield with `sim_demo_policy`: an aircraft held 3 m
inside `nfz-square` for 600 ticks stayed in Brake with setpoint `pass` and
`failsafe_triggered = False` for all 60 s, under both magnitude sources. That
breaks p1, and `experiments/scenarios.yaml`'s `wind-beyond-authority` expects
`RTL_triggered` for exactly this.

X6 sends it to RTL when the position has been illegal on every tick for T
(the grant's own persistence window, 5 s), while the Shield is still
intervening on a hard rule. The same 600-tick run now gives G1 at t = 0 and
X6 at t = 5.0 s, RTL_triggered.

- **Why not count such ticks as N-in-T events?** At 10 Hz, three of them put
  the aircraft in Loiter 0.3 s after any incursion, and three more in RTL.
  Every GeofenceEscape that was about to succeed would be aborted. X6 gives a
  recovery the same 5 s the grant gives N-in-T.
- **Why skip Loiter?** LOITER holds position, and holding position where a
  stop is illegal keeps the aircraft in the violation: the reference's
  "brake while inside = deadlock". If the FSM is already in Loiter there (G3
  on an unconverged repair), X6 takes it to RTL T after the position became
  illegal, instead of after the 30 s timeout.
- **Why require a hard violation on the tick?** A tick on which the raw
  action is already escaping (clean) is the pilot leaving by itself. The
  clock keeps running over such ticks, but X6 fires only on a tick where the
  Shield is intervening.

## How a rule's fields become a response

The grant defines `violation_action`, `constraint_type` and `priority`. It
never says what the Shield does with the last two. `RuleHit.effective_action`
is the single place this is decided. shield.py should call it, so the repair
stack and the FSM cannot disagree.

| violation_action | hard rule | soft rule |
|---|---|---|
| `monitor_only` | recorded; no repair, no state change, no N-in-T | same |
| `project_fix` (alias `repair`) | repair; G1 if trusted, else X1, then G3 | repair; Brake at most |
| `brake` | X1 into Brake, setpoint `brake`; never G3 | same; Brake at most |
| `loiter` | X4 | **capped at `brake`**, flagged `capped: true` |
| `RTL` | G11 / X2 | **capped at `brake`**, flagged |
| `land` | G12 / X3 | **capped at `brake`**, flagged |

- **Soft = enforced, capped at `brake`.** A soft rule never changes the flight
  mode, never counts toward N-in-T, and never holds the FSM in Loiter (G8 looks
  at hard rules only). Without that last rule, a soft violation could reach RTL
  through the Loiter timeout. `brake` is the strongest action the grant itself
  gives a soft rule (`envelope-default`, Policy DSL PDF p4), so a soft `brake`
  rule is obeyed, not flagged as capped. The audit card suggested "soft =
  monitor only". That would contradict the grant's own example, so it was
  not used.
- **Priority never lightens the response.** The response is the strongest
  effective action among the enforced violations. If priority chose it, adding
  a P0 `project_fix` violation to a P2 `land` would turn a landing into a
  repair. Priority orders the record (hard before soft, then P0, P1, P2) and
  breaks ties: with equal actions, the P0 rule is cited, whatever order the
  Shield listed violations in. The governing rule's priority is written as
  `risk_level`, the grant's auto-label (Stress Testing PDF p5).
- **Kinematic rules are not cited for θ.** A kinematic envelope's repairs are
  clamps: they always converge and θ never judges them. `RuleHit.kinematic`
  (set from the policy's `kinematic_envelope` type, or from a `kinematic`
  violation category when there is no policy) keeps such a rule from making a
  tick untrusted, and an untrusted projection is attributed to a rule whose
  repair θ does judge. Before, with every rule unresolved and P0, the replay
  cited `kin-caps` for every θ escalation, because it sorts first by id.
- **Action names are case-insensitive.** The grant spells the same action
  `land` (DSL literal) and `Land` (FSM diagram). Unknown names are refused, not
  defaulted.
- A violated id that is not in the policy becomes hard / P0 / `project_fix`
  with `resolved: false`. In practice this means the Shield's own
  `action-finite` contract check. `guardrail/kpi.py` makes the same "unknown
  means P0" choice.

## Interpretations where the grant is silent

| Term | Chosen reading | Why |
|---|---|---|
| "N-in-T" | counts hard-rule **onsets** (violated after a tick with none), not violated ticks. `event_mode: tick` is kept as a knob | At 10 Hz, counting ticks escalates any violation that lasts 0.3 s, so every sustained repair, such as sliding along a fence, would put the aircraft in LOITER. "3 in 5 s" reads as three separate violations, the thrash of a pilot fighting the Shield. The grant leaves the defaults to "stress-run data" (p7). The one case onsets cannot see, a position that stays illegal, is X6's |
| "N-in-T persists" | N **new** onsets within T after entering Loiter (the window is cleared on entry) | otherwise the onsets that caused the Loiter fire G4 on the next tick |
| window edges | inclusive: an onset exactly T old counts. Floating slack 1e-9 s | the more conservative reading; fifty additions of 0.1 s give 4.999999999999998 s |
| "cleared for ≥ T_recover" | from the **first clean tick** | the later of the two possible starts, so recovery is never early |
| "unrepairable" (G11, G12) | a rule whose action is RTL or land **skips projection**: G11/G12 fire even on a tick whose repair would have converged | the reference's shield.py: "Rules whose action is a direct fail-safe skip projection entirely". Repairing first and going home only if the repair fails would let a rule written to send the aircraft home be satisfied by a nudge |
| θ comparison | exactly θ is accepted; escalate on > θ | p3 "magnitude ≤ threshold wins", p4 "> threshold". The diagram's "< θ" is the only exception |
| θ per axis | escalate if lateral > 2.0 m **or** vertical > 0.5 m | the two are separate thresholds in p4 |
| θ over several operators | the per-axis **sum** of the operators that fired this tick | p4 caps the total change, ‖repaired − original‖, and the sum bounds it from above. The reference checks each attempt on its own (see "Where this differs from the PI's reference code") |
| which repairs θ judges | position operators only. Exempt under **both** magnitude sources: kinematic clamps, `Sanitise`, `Brake`, and **recovery** operators: `GeofenceEscape` and `StandoffRecover` by name, a `ClearanceEscape` only where a stop is illegal, and any repair flagged `recovery: true` | the reference exempts recovery operators: capping them "would re-introduce the 'brake while inside = deadlock' failure". An aircraft inside a zone needs a correction as large as its penetration to get out. An operator name the module has never seen is judged, not exempt |
| a rescue where a stop is legal | `ClearanceEscape` that re-checked clean but `stop_illegal` is false: **not converged** (outcome `blocked`), the legal stop is streamed, and the FSM escalates | shield.py runs it when "repair not converged", which can happen with BRAKE legal (a predicted clearance breach). That is p4's "no operator converges". The reference exempts only GeofenceEscape, which runs only for an aircraft already inside, and `guardrail/kpi.py` counts rescues apart (`converged_via_rescue`) |
| `stop_illegal` | standing still here breaks a **hard, enforced** rule | soft rules are capped at `brake`, so a stop that breaks only a soft rule is allowed |
| a stop where stopping is illegal | never streamed (see States) | standing still inside a no-fly polygon or a clearance ring keeps the aircraft in the violation |
| a position illegal for T | X6 → RTL | see "X6" |
| operator timeout | `loiter_timeout_s = 30`, `null` disables it; must exceed T_recover | no grant default. A timeout ≤ T_recover would make every Loiter an RTL |
| fail-safe "triggered" | entering RTL or Land | the outcome vocabulary has RTL_triggered and Land_triggered, and no Loiter_triggered. `summary()` reports `loiter_entered` separately. (The Grant overview lists Loiter among the "ultimate backstop" modes; PI question 5) |

## Where this differs from the PI's reference code

The reference (`kuanting-vla-uav-guardrail/packages/safety-shield/src/safety_shield/`)
ships a Phase-1 slice. Its own docstring says: "The full FSM (Brake -> Loiter
-> RTL -> Land with N-in-T thresholds and recovery) lands in Phase 2." This
module is that Phase 2, built from the grant's diagram, which is also the
reference's own design page. Where the slice and this module disagree:

| Behaviour | Reference slice (`fsm.py`, `shield.py`, `repair.py`) | This module | Why |
|---|---|---|---|
| States | Normal, Brake, RTL, Land | adds Loiter | the grant's diagram has five |
| Back to Normal | on the first clean tick, from any state, RTL and Land included | only from Brake and Loiter, after ≥ T_recover clean | grant edges G7 and G8. Nothing leaves RTL or Land except [*] |
| Unrepairable, no direct action | straight to RTL ("conservative RTL") | X1 into Brake, then G3 into Loiter on the next untrusted tick, then RTL by G4, G5 or X6 | the grant's diagram steps Brake → Loiter → RTL. The slice skips two steps |
| A position that stays illegal | not handled (the slice has no N-in-T) | X6 → RTL after T | p1; without it the case never escalates (see "X6") |
| Which rule decides | `violations[0]`, the earliest predicted hit (`checker.py` sorts by `first_hit_s`). Priority and hard/soft are unused | the strongest effective action among enforced violations. Priority breaks ties, soft is capped at `brake`, and a θ escalation is cited to a non-kinematic rule | a `land` rule hit 2 s ahead should not be repaired away because a `project_fix` rule was hit 1 s ahead. The grant is silent |
| Direct actions | `RTL` and `land`, exact spelling | also `loiter` (X4) and `brake`, case-insensitive | the DSL's six actions (Policy DSL p2). The grant spells both `land` and `Land` |
| θ check | inside the repair loop, per attempt, against the target's axis (`repair.py` `repair_action`). An over-cap attempt means "not converged" | in the FSM, per-axis sum over the tick's operators | p4 caps the total change. Both give the same edge |
| Recovery operators | `GeofenceEscape` exempt from θ | `GeofenceEscape` and `StandoffRecover` exempt; `ClearanceEscape` only where a stop is illegal | the reference's "brake while inside = deadlock"; this repo has two more inside-recoveries and a rescue the reference does not |
| Setpoint on a direct fail-safe tick | the raw action | `brake` on the hand-over tick, or the Shield's action if a stop is illegal | the last GUIDED setpoint should not keep flying while SET_MODE goes through |

## The θ decision, measured — the one that matters most

The grant states θ in **metres** of correction. Our actions are velocities.
How a velocity repair becomes metres decides whether the cap ever fires, and
it is not a detail. The module offers two sources and picks neither silently.
`tick_input_from_decision` uses the per-operator source unless `horizon_s` is
given, and that source raises while shield.py lacks the field. So no flight
can run on a default nobody chose:

1. **Per-operator `magnitude_m`** (`magnitude_from_repairs`), the grant's own
   audit field (p5). The PI's reference fills it with the position correction:
   penetration depth for lateral projection, altitude error for the altitude
   clamp. Any operator θ governs (`theta_governs`) and that has no
   `magnitude_m` is refused. **shield.py's `Repair` has no such field today.**
2. **Velocity proxy** (`magnitude_from_actions`): |Δv| × h, the gap after h
   seconds between where the clamped raw action and the repaired one would
   put the aircraft. h has no default.

Both go through `repair_magnitude`, and both apply the same exemptions. A
tick whose repairs are all exempt is 0 m under either source.

**The proxy measures from the clamped raw action.** shield.py runs the
kinematic clamps first, on the raw action, and the position operators work on
the clamped result (`_decide`: `fixed = self._repair_kinematic(raw, repairs)`
before the clearance / standoff / corridor / geofence loop). `KinematicCaps`
reproduces SpeedClamp and ClimbClamp exactly, and the proxy measures the
repaired action against `caps.clamp(raw)`. `KinematicCaps.from_policy` reads
the caps, so the node needs nothing extra. On a tick that has a SpeedClamp or
ClimbClamp and no caps, the proxy is **refused**. A policy whose caps
contradict the decision (a raw action over the caps and no clamp fired) is
refused too, because it is the wrong policy for that run.

### Retracted: the first version of this table

The first version of this section, published in this same file earlier on
2026-10-06, measured the proxy from the **unclamped** raw action. Every one of
the 1763 θ-governed ticks also carried a SpeedClamp (973 StandoffHold +
SpeedClamp, 562 GeofenceSlide + SpeedClamp, 148 StandoffHold + SpeedClamp +
ClimbClamp, 51 AltitudeFix + SpeedClamp + ClimbClamp, 16 of those with a
StandoffRecover and 13 with a StandoffHold), so most of what that table
measured was the clamp's Δv, which this module says θ never judges. What it said, and what is right:

| Claim in the first version | Correct value | What was wrong |
|---|---|---|
| h = 0.1 s: 91 / 1763 governed ticks over θ; 1 / 7 runs escalates (`sitl_ped_on`, vertical, t = 98.5 s) | 0 / 1763; 0 / 7 runs | the one escalation (tick 915) is a ClimbClamp 7.04 → 2.00 m/s alone: 5.04 m/s × 0.1 s = 0.504 m. The only position operator on that tick is StandoffHold, which is lateral |
| h ≥ 0.5 s: 1763 / 1763 over θ at every horizon | 91 / 1763 at 0.5 s; 1696 / 1763 at 1.0 s | clamp Δv counted |
| ROS runs: θ first exceeded between h = 0.277 s and 0.494 s | between 0.354 s and 0.803 s | clamp Δv counted |
| median lateral \|Δv\| on governed ticks 6.0 m/s, minimum 2.0 | 3.99 m/s, minimum 0.00 (49 governed ticks change nothing after the clamp) | 5.98 / 2.0 is the clamp-included figure |
| "642 carry speed clamps only" | 629 clamps only, plus 13 clamps with a StandoffRecover | the 13 are exempt through the recovery, not as clamps |
| "29 of the 1763 also contain a StandoffRecover" | 16 of the 1763 | 29 is the count over all 2405 repaired ticks |
| "the two sources can differ only in how the metres are measured, never in which repairs θ applies to" | true only since the clamp-free proxy | on these runs the old proxy disagreed on 100 % of governed ticks |

The fix in the first review round (sharing the exemptions) covered pure-clamp
ticks only. Mixed ticks were still measured with the clamp in.

### The table, clamp-free

What option 2 would have done to the seven delivered shield-ON flights. This
is a read-only replay of their `flight_log.jsonl` through the FSM, with the
policy each run flew (`--policy`): `sitl_pedestrian.yaml` for the three
`*_ped_*` runs and `sim_demo_policy.yaml` for the four `*_shield_on*` runs.
Both have a 4 m/s speed cap and a 2 m/s climb cap. The replay checks the
loaded policy's hash against the run's manifest: five match. The two
`_dynamic` runs do not, because a hot-applied NFZ bumped the policy
generation in flight, so that fence's id resolves as unknown (hard, P0). The
replay is counterfactual after the first escalation, because the real flight
never escalated, so only the first escalation is reported.

Of the 2405 repaired ticks, 642 are never judged by θ: **629 carry clamps
only**, and 13 carry clamps with a StandoffRecover. The other **1763** contain
a position operator (`GeofenceSlide`, `StandoffHold`, `AltitudeFix`), **every
one of them together with a SpeedClamp**, and 16 of them also with a
StandoffRecover. 408 of the 1763 are in the three ROS runs.

| h | θ-governed ticks over θ, all 7 runs | ROS runs | runs that would escalate | first escalation |
|---|---|---|---|---|
| 0.1 s (one monitor tick) | 0 / 1763 (0 %) | 0 / 408 (0 %) | 0 / 7 | – |
| 0.5 s (the Shield's `dt`) | 91 / 1763 (5.2 %) | 59 / 408 (14.5 %) | 4 / 7 (the four `sim_demo_policy` runs) | tick 2 (t = 0.20–0.24 s), G3, `nfz-square`, lateral 2.83 m |
| 1.0 s | 1696 / 1763 (96.2 %) | 408 / 408 | 7 / 7 | tick 2 (5 runs), tick 15 (`ros2_ped_on`, `sitl_ped_side_on`), all G3 |
| 3.0 s (the Shield's lookahead) | 1699 / 1763 (96.4 %) | 408 / 408 | 7 / 7 | same |
| 5.0 s (the grant's horizon) | 1701 / 1763 (96.5 %) | 408 / 408 | 7 / 7 | same |

Every escalation now cites the rule whose repair broke θ (`nfz-square`,
`standoff-pedestrian`, `standoff-any`), not `kin-caps`. The two runs that
first escalate on tick 15 spend their first 14 repaired ticks on clamps alone,
before the standoff ring comes into the lookahead.

Runs: ros2_ped_on, ros2_shield_on, ros2_shield_on_dynamic, sitl_ped_on,
sitl_ped_side_on, sitl_shield_on, sitl_shield_on_dynamic. All have `brakes = 0`.
Reproduce with (one line per run; change `--horizon` for the other rows):

```
python -m guardrail.fsm replay demo/out/ros2_ped_on demo/out/sitl_ped_on \
    demo/out/sitl_ped_side_on --policy policies/sitl_pedestrian.yaml --horizon 1.0
python -m guardrail.fsm replay demo/out/ros2_shield_on demo/out/ros2_shield_on_dynamic \
    demo/out/sitl_shield_on demo/out/sitl_shield_on_dynamic \
    --policy policies/sim_demo_policy.yaml --horizon 1.0
```

Without `--policy` the replay of these runs exits 2 and says why: every
governed tick has a clamp, and the proxy needs the caps.

**The proxy is still close to a switch.** On the ROS runs, the horizon at
which a position repair first exceeds θ lies between 0.354 s and 0.803 s for
every tick (median 0.539 s). Below 0.354 s the cap never fires on any of the
seven runs; at 1 s it fires on 96 % of position repairs. The stub flies
straight at the zone or the person and keeps asking to, so the clamped
command is 4 m/s and a position operator removes most of it every tick:
median lateral |Δv| on the governed ticks is 3.99 m/s, measured from the
clamped raw action.

Read plainly: with a one-tick horizon the cap is **inert**, because no
delivered flight would ever exceed it. That is a check that cannot fail. With
any horizon from 1 s up, almost every stub flight goes to LOITER as soon as a
position operator fires. The grant may well intend the second outcome: its
reference escalates "a deep head-on dive … rather than projected". But it
would change what the KPI runs show. **Recommendation:** option 1. It is the
grant's schema and the PI's reference metric, and it needs shield.py to
report `magnitude_m` and `axis` per position operator (see Integration).
Until then, any run that uses option 2 must state its h beside every trigger
count. This is a question for the PI (below).

### N-in-T and X6 on the same flights

Each delivered run holds **one** violation that, once it starts, never clears
until the log ends: 1 onset per run. In onset mode N-in-T can never fire on
them. In tick mode, every one of them would loiter after three ticks
(t = 0.30–0.34 s). These flights cannot exercise N-in-T or recovery. The
labelled stress episodes have to (see "The KPI").

The delivered logs do not record `stop_illegal`, so the plain replay cannot
see X6 (`rows_recording_stop_illegal: 0`). `tools/rescore_kpis.py`
reconstructs it per row (`unsafe_rules`, from `Shield.state_is_unsafe`), and
the replay reads that field when present. With it reconstructed, at
h = 0.1 s, X6 fires on **1 of 7** runs: `sitl_ped_on`, tick 943,
t = 101.5 s. That run is under the 10 m altitude floor (`alt-band`, hard P0)
on 216 ticks in three episodes, the longest 10.5 s unbroken (96.5–107.0 s),
at about 2 m altitude, while the Shield was emitting the +2.0 m/s climb cap
on every tick. A repair that has not lifted the aircraft back over the floor
in 5 s is the case X6 exists for. The other six runs have no illegal
position. (Why the aircraft kept sinking under a +2 m/s climb command is a
question about the pymavlink SITL rail, not about this module. The run's
own `metrics.json` already reports `alt_violation_s: 21.6`.) The
reconstruction is a measurement, not a replay option. To repeat it: load the
rows and `metrics.json`, call `tools/rescore_kpis.py`
`reconstruct_unsafe(rows, policy, metrics)`, then
`guardrail.fsm.replay_rows(rows, horizon_s=0.1, policy=policy)`.

## Inputs, outputs, audit record

**Per tick in:** `TickInput(t, outcome, violations, magnitude, rtl_failed,
home_reached, landed, note, stop_illegal)`

- `t` is monotonic seconds. Repeats are allowed, and going backwards raises.
- `outcome` ∈ `clean | repaired | blocked`. `blocked` = the Shield braked, or
  its own re-check of what it emitted is not clean, or it rescued a
  non-converged chain with ClearanceEscape where a stop was legal.
- `violations` is every rule the raw action broke, as `RuleHit`s, including
  monitor_only and soft rules.
- `magnitude` is required when `repaired`. A repair of unknown size is never
  waved through as a small one.
- `stop_illegal`: give `tick_input_from_decision` the list
  `Shield.state_is_unsafe(state)` returns (it keeps only hard, enforced
  rules), or a bool. Left unset, it is read from a blocked decision (the
  Shield did not brake, or its brake still re-checks dirty) and taken as
  false on a repaired one. A ClearanceEscape that re-checked clean is the
  exception: whether it is a recovery or an unconverged repair depends on
  exactly this flag, so unset is refused there.

Inconsistent inputs raise: `clean` with an enforced violation, `repaired` /
`blocked` with none, NaN anything.

**Per tick out:** `FSMOutput(state, before, transition, edge, set_mode,
setpoint, reason, terminal, record)`.

**Audit record** (a plain dict, JSON-serialisable; WP3-11 will wrap it in the
Pydantic model the grant asks for): `fsm_tick, t_s, fsm_state_before,
fsm_state_after, transition, edge, edge_source (grant | extension), terminal,
set_mode, setpoint, reason, outcome, governing_rule, risk_level, violations
(sorted, each with effective_action / capped / resolved / kinematic),
magnitude (with recovery_exempt), theta, theta_exceeded, n_in_t, n_threshold,
window_s, cleared_s, cleared_hard_s, stop_illegal_s, flags (incl.
stop_illegal), stop_withheld, note, fsm_config_hash`.

`fsm_config_hash` is the full SHA-256 of the thresholds in force, over
canonical float values, so a trigger count can be traced to the N / T / θ
that produced it and `T: 5` hashes the same as `T: 5.0`.

**Per episode:** `summary()` gives `max_state`, `visited`,
`failsafe_triggered` (RTL or Land entered), `loiter_entered`, `outcome_label`
(`Land_triggered` if Land was entered, else `RTL_triggered` if RTL was, else
None), `transitions` and `fsm_config_hash`.

## Config (YAML, per mission profile)

```yaml
escalation:            # grant symbols (N, T, T_recover) or the field names
  N: 3
  T: 5.0
  theta_lateral_m: 2.0
  theta_vertical_m: 0.5
  T_recover: 2.0
  loiter_timeout_s: 30.0   # ours; null disables
  event_mode: onset        # ours; or "tick"
  profiles:
    night:
      t_recover_s: 3.0     # either spelling, in either layer
```

`FSMConfig.from_yaml(path, profile="night")`. Unknown keys are **refused**: a
misspelt `t_recovr` that silently kept the 2 s default is the zero that looks
like a setting. Aliases are resolved per layer, so a key given twice inside
one layer is refused while a profile may override the base in the other
spelling. No YAML file is shipped yet. Where it lives (its own file, or an
`escalation:` block in the policy) is open, because `load_policy` would need
to accept the key.

## Integration, call site by call site

Other units are editing these files today, so this section names each call
site by its anchor text only, not by line number. Search for the quoted text.

**Order matters.** Land steps 1 and 2 (models.py accepts the grant's actions,
and shield.py honours `effective_action`) together, and before step 3 wires
the node. If models.py accepts `monitor_only` while shield.py still repairs
every violated rule, the Shield would report a repair that no enforced rule
asked for. The FSM refuses that input as a contract break, and the node's
fault handler (below) would hold the aircraft in LOITER on the first such
tick. That is safe, but it is a bug report, not a flight.

### 1. `guardrail/models.py` (WP1-10)

- `ConstraintBase.violation_action` (`Literal["repair", "brake"] = "repair"`).
  Accept the grant's six plus `repair`, with a `field_validator` that calls
  `fsm.canonical_action` only to **check** the value. Store it as authored:
  rewriting `repair` to `project_fix` in the model would change the canonical
  IR, and with it the policy hash of every policy on disk. `RuleHit`
  canonicalises at use, so nothing downstream needs the rewritten form. Keep
  the default `repair`. (`guardrail/fsm.py` imports nothing from the package,
  so models.py can import it without a cycle.)

### 2. `guardrail/shield.py` (WP3-06, WP3-08, WP1-11, WP1-12)

Keep the FSM out of `Shield.filter()`. filter() keeps a sliding window of its
own decisions (`self._window`), but what it decides must not depend on
escalation state. Its call sites outside tests (sweep, deck builders, demos,
both SITL rails) use it for repair only and have no autopilot whose mode they
could change. The FSM is per-episode state, and the node owns it. The Shield
changes in three ways:

1. **Honour `effective_action` before repairing.** In `_decide`, after
   `violations = self._check(state, raw)`, build
   `hits = fsm.rules_from_policy(self.policy, [v.rule_id for v in violations])`.
   Drop the `monitor_only` ones from the set to repair; they stay in
   `violations` for the record. If any hit's effective action is RTL / land /
   loiter, skip projection, as the reference does ("Rules whose action is a
   direct fail-safe skip projection entirely"), and return `braked=True,
   emitted=BRAKE`. If one is `brake`, return BRAKE without projecting. (The
   FSM keeps a hard `brake` rule in Brake however long it stays blocked; that
   is pinned by a test.)
2. **Report the position correction per operator.** Add
   `magnitude_m: float | None = None`,
   `axis: Literal["lateral", "vertical"] | None = None` and
   `recovery: bool = False` to `class Repair`, and fill them where each
   position operator appends its `Repair`. Vertical: `AltitudeFix`,
   `CorridorAltitudeFix` (the altitude error, m). Lateral: `GeofenceSlide`,
   `ClearanceFix`, `StandoffHold` (the predicted penetration depth, m).
   `GeofenceEscape` and `StandoffRecover` are recovery by name. The flag can
   only add an operator: `recovery=False`, the model default, never
   un-exempts a named one. `CorridorReturn` should set `recovery=True` on its
   `d > half` branch, where the aircraft is already off the corridor, and
   report a lateral magnitude on the other branch. Leave `ClearanceEscape`
   without the flag: whether it is a recovery depends on `stop_illegal`, which
   the FSM side already has. Kinematic clamps stay without a magnitude. This
   is what `fsm.magnitude_from_repairs` reads, and what the grant's audit
   record (p5) carries.
3. **Do not apply θ inside filter().** The FSM decides trust from the
   reported magnitude. filter() keeps braking only when nothing converged
   (the block under `# P0 escape guard: repaired action must re-check
   clean.`). The reference applies θ inside its repair loop instead. The
   result is the same edge, because an over-θ attempt there returns "not
   converged" and the FSM escalates. Keeping it out of filter() keeps every
   call site's repairs unchanged.

Giving up lower-priority rules first, when the stack cannot satisfy all of
them (WP1-12), is a repair-chain change inside the `for _ in
range(_REPAIR_PASSES):` loop. It is separate from the FSM.

### 3. `sitl/ros2_shield_node.py` (WP3-07 wiring, WP3-09)

- **imports** (beside `from guardrail import ...`): `from guardrail.fsm
  import EscalationFSM, FSMConfig, FAILSAFE_STATES, tick_input_from_decision`.
- **`__init__`** (after `self.shield = Shield(...)`):
  `self.fsm = EscalationFSM(FSMConfig.from_yaml(p) if p else FSMConfig())`.
  Add subscriptions next to the existing ones: `/mavros/extended_state`
  (`landed_state` gives `landed`) and `/mavros/home_position/home` (with the
  local pose, `home_reached` = within `REACH_M` of home). Store
  `MavState.mode` in `_on_state`, so a requested mode can be confirmed and an
  autopilot that leaves RTL by itself can be seen.
- **`_tick`**, right after `decision = self.shield.filter(st, raw)`, only when
  `self.shield_on` (the control arm records `fsm_state_after: null`):

  ```python
  try:
      inp = tick_input_from_decision(now, decision, self.policy,
                                     horizon_s=self.theta_horizon_s,  # None = per-operator
                                     rtl_failed=self._rtl_failed(st),
                                     home_reached=self._home_reached(st),
                                     landed=self._landed,
                                     stop_illegal=self.shield.state_is_unsafe(st))
      out = self.fsm.step(inp)
  except ValueError as e:              # a Shield/FSM contract break mid-flight
      # Never let it kill the timer: hold, say why, and stop streaming.
      self.get_logger().error(f"FSM input refused: {e}; requesting LOITER")
      if not self._fsm_fault:
          self.cli_mode.call_async(SetMode.Request(custom_mode="LOITER"))
          self._fsm_fault = str(e)     # into metrics.json; the run is not KPI-grade
      return
  ```

  `stop_illegal` takes the violation list as returned; the helper keeps only
  hard, enforced rules. The FSM raises only on a contract break: an
  inconsistent decision, time going backwards, a repaired tick with no
  magnitude, a position repair that reports no `magnitude_m` (per-operator
  source), or a clamped tick with no caps (proxy source; the policy supplies
  them). A flight that hit one is a bug report, not a data point. AP
  GeoFence stays the backstop (Safety Shield PDF p5).

- **setpoint**: replace `emitted = decision.emitted if self.shield_on else
  raw` with `pass` → `decision.emitted`, `brake` → `Action4D()`, `none` →
  skip `self.pub.publish(tw)` for this tick. Never substitute a zero for
  `pass` on a blocked tick: `decision.emitted` already is the Shield's BRAKE
  where a stop is legal.
- **mode**: `if out.set_mode: self.cli_mode.call_async(SetMode.Request(custom_mode=out.set_mode))`.
  Fire-and-forget, the same pattern as `_finish`, because `_tick` runs inside
  a timer callback. MAVROS's ArduCopter names are exactly `GUIDED`, `LOITER`,
  `RTL`, `LAND`. Log the time to `MavState.mode` confirmation.
- **`rtl_failed`**: true when, after RTL was confirmed, `MavState.mode` stops
  reading RTL without `home_reached`, or `shield.state_is_unsafe(st)` holds a
  hard geofence violation (the RTL track is blocked), or battery is below the
  failsafe level. Which of these counts is an integration choice, and it
  should be written into the manifest.
- **rows** (`self.rows.append({...})`): add `"fsm_state_before":
  out.before.value`, `"fsm_state_after": out.state.value` (the grant's audit
  field names, and what `guardrail/kpi.py` `_escalated` / `_held` read),
  `"fsm_edge": out.edge`, `"set_mode": out.set_mode`, `"setpoint":
  out.setpoint`, `"stop_illegal": inp.stop_illegal` (so later replays can see
  X6 without a reconstruction), and `"failsafe": bool(out.transition and
  out.state in FAILSAFE_STATES)`. kpi.py's `_repair_outcome` already reads
  `row["failsafe"]` and reports `failsafe_instrumented`, so this is the field
  that turns its 0 into a measured 0.
- **audit** (`self.audit.log(len(self.traj), decision)`): pass `out.record`,
  so `fsm_state_before` / `fsm_state_after` land in `audit.jsonl` (WP3-11).
- **`_finish`**: the mission-end LAND is **not** a fail-safe and must never
  set `failsafe`. If the FSM is terminal (`G9` / `G10` / `X5`), do not send
  LAND again. If it is in RTL or Land, let the autopilot finish. Write
  `self.fsm.summary()` into `metrics.json` (`_report`).
- The hard-coded mission landing and the FSM's modes both go through
  `self.cli_mode`. A separate MAVLink adapter node (WP3-09) would own that
  client, with this node publishing the requested mode on a topic instead.

### 4. `sitl/run_sitl_demo.py` (pymavlink rail)

Same pattern after `decision = shield.filter(state, raw)`. Modes go through
the existing mode helper (`self.m.set_mode(mode_id)` with
`self.m.mode_mapping()[out.set_mode]`).

### 5. `guardrail/audit.py` (WP3-11)

`AuditLogger.log` takes an optional `fsm_record` and copies
`fsm_state_before`, `fsm_state_after`, `edge`, `edge_source`, `reason`,
`fsm_config_hash` into the record. Write one file per episode, not appended
(WP4-20).

### 6. `guardrail/kpi.py` and the harness (WP3-15, WP4)

- Per flight: `outcome` takes `RTL_triggered` / `Land_triggered` from
  `summary()["outcome_label"]`. The vocabulary is already in `kpi.OUTCOMES`.
- The grant's KPI is **per episode** with a label, not per tick:
  `fsm.score_failsafe_triggers(episodes)`, where each episode is
  `{"expected_failsafe": bool, "triggered": summary["failsafe_triggered"]}`.
  The in-progress per-tick fields in kpi.py ("expected" = a P0 on the raw
  action) are a proxy. They should be renamed so they do not carry this KPI's
  name.
- **The label exists; the definition of "triggered" does not match yet.**
  `ScenarioSpec.expected_failsafe: bool | None` (guardrail/scenario_spec.py)
  is the label, and `experiments/sweep_scenarios.py` `score()` already
  reports `failsafe_matches_label` and a `_failsafe_labels` summary. But it
  counts `triggered = any(r.get("braked") for r in rows)`, a Shield BRAKE,
  and this module counts entry into RTL or Land. When the FSM is wired in,
  the sweep should take `triggered` from `fsm.summary()["failsafe_triggered"]`
  and report `score_failsafe_triggers` beside its own summary, which adds the
  always-trigger and never-trigger nulls that `_failsafe_labels` lacks.
  Every label written while "triggered" meant BRAKE must be re-read against
  the new definition before the two are compared.
- `experiments/scenarios.yaml` `wind-beyond-authority` describes the expected
  route as "N-in-T persists -> Loiter -> RTL". With this FSM the route is X6
  (the position stays illegal for T while GeofenceEscape keeps running:
  Brake → RTL), or G3 → Loiter → X6 if the escape stops converging. Its
  labels (`RTL_triggered`, `expected_failsafe: true`) stay right. Only the
  description's route should change.
- Expected True: a breached rule whose action is RTL / land; a position that
  stays illegal for T; an unrepairable or over-θ repair that persists past
  the Loiter timeout; an RTL that fails. Expected False: clean control runs,
  and small trusted repairs along a fence. If a scenario is meant to count a
  Loiter, say so in its label.

## The KPI: fail-safe trigger correctness, and its nulls

`score_failsafe_triggers` returns
`(correct triggers + correct non-triggers) / scored episodes`, with:

| Field | Why |
|---|---|
| `null_always_trigger` = expected triggers / scored | what a Shield that RTLs every episode scores |
| `null_never_trigger` = expected non-triggers / scored | what a Shield with no fail-safe scores. This was the delivered Shield |
| `beats_null` | strictly above both. A tie is not evidence |
| `discriminating` | false when every episode has the same label. Then one of the two stubs ties whatever the Shield does, a warning names it, and **`meets_target` / `meets_target_at_95` are None**, not True |
| `wilson_low_95`, `meets_target_at_95` | 99 % shown at 95 % confidence needs **381** error-free scored episodes (`min_error_free_episodes_for_target_at_95`). With 100 perfect episodes the point estimate meets the target and the bound does not |
| `unlabelled`, `not_measured` | episodes left out are counted, never silently dropped |

The first version returned `meets_target: True` and `meets_target_at_95:
True` for 400 episodes of (expected False, triggered False): correctness 1.0
from a Shield with no fail-safe at all, and a passing flag a dashboard would
read as a pass. The correctness figure is still returned, because it is what
was measured, but the target is not judged on a set that cannot fail it.

Labels must be real booleans: `"false"` and `0`/`1` are refused. An empty set
returns `None`, not 0 and not 1.

## How the tests were checked

- **Failing first, the module.** With `guardrail.fsm` made unimportable (the
  tree before this unit), `tests/test_fsm.py` stops at import:
  `ModuleNotFoundError: import of guardrail.fsm halted`.
- **Failing first, review round 1** (three fixes). The first version of this
  module (72 tests) had three defects its own tests could not see. The same
  inputs, run through the first version and then the round-1 version:

  | Input | First version | Round 1 |
  |---|---|---|
  | real Shield decision, aircraft 3 m inside `nfz-square`, `GeofenceEscape`, proxy h = 1 s | X1, setpoint `brake`, 3.0 m over θ: freezes the aircraft inside a P0 zone | G1, `pass`, 0 m (`recovery_exempt: GeofenceEscape`) |
  | speed clamp only (8 → 4 m/s), proxy h = 3 s | X1, `brake`, 12.0 m | G1, `pass`, 0 m |
  | per-operator `GeofenceEscape` with `magnitude_m` 6.0 | 6.0 m counted | 0 m, exempt |
  | blocked decision where the Shield chose a best-effort recovery over a stop | X1, `brake` | X1, `pass` |

  The round-1 tests against the first version give **67/75**, not the 68/75
  this file first said. Eight fail or error: the real-decision test, the
  stop-illegal setpoint and inference tests, the recovery exemption, both
  random-episode tests, the replay's first-escalation test, and
  `test_replay_cli_prints_one_json_line_per_run` (whose exit-2 refusal was
  added after the 68/75 was measured).
- **Failing first, review round 2.** The current 95 tests against the round-1
  `fsm.py` give **76/95**. The behaviour failures, one per fix:
  the real Shield held inside `nfz-square` for 60 s stays in Brake (edges
  `[G1]` only, no X6); 400 all-negative episodes report `meets_target: True`;
  `FSMConfig(window_s=5)` and `FSMConfig()` hash differently; a profile that
  overrides `N` with `n_violations` is refused; a ClearanceEscape where a
  stop is legal is a trusted G1; a stop that breaks only a soft rule counts
  as illegal; the clamp-free proxy, `KinematicCaps`, `RuleHit.kinematic`
  and the replay's `--policy` do not exist. Against the first version the
  current tests give 69/95.
- **Every edge.** `test_every_edge_is_driven` runs one shortest script per
  edge id and fails if `EDGES` gains an id without a script.
- **Random-episode invariants.** 60 noisy episodes check, on every tick: only
  listed edges, mode requested only on a transition, Brake recovers only
  after T_recover clean, Loiter holds ≥ T_recover, no setpoint streamed in
  autopilot modes except the hand-over tick, no stop streamed on a tick whose
  input says a stop is illegal (2157 such ticks, 21 with a stop withheld),
  nothing moves after [*]. A further 60 soft-only episodes never leave
  GUIDED.
- **Mutation.** 69 deliberate defects, each applied to a scratch copy of
  `fsm.py` (never to the repository), and all 69 make at least one named test
  fail. They are the 37 of round 1 (patterns re-pointed where code moved),
  the nine the round-2 reviewer found surviving (a hard `brake` rule sent to
  Loiter by G3, G3 checked before a direct RTL, Loiter counted as a
  fail-safe trigger, the RTL label winning over Land after G6, a soft
  `brake` rule flagged as capped, `rtl_failed` checked before
  `home_reached`, the θ float slack removed, `meets_target` judged on a
  non-discriminating set, `discriminating` true with only positives), and
  23 new ones for X6 (each condition, its order, its clock, its slack), the
  clamp-free proxy and its two refusals, the rescue rule, the hard-only
  `stop_illegal` filter, kinematic attribution, the canonical full digest,
  per-layer aliases and the replay's new counters. The run refuses to report
  if the unmutated scratch copy is not green, because other units were
  editing `guardrail/` at the same time. The harness is a scratch script, not
  committed: each mutant is a literal (old text, new text) pair applied once
  to a copy of `fsm.py`, with `tests/test_fsm.py` run against the copy. The
  reviewer's R9 was not described, so it could not be re-created.

## Questions for the PI

1. **θ's unit.** Is θ the position correction per operator (the reference's
   penetration depth / altitude error), or ‖Δaction‖ over some horizon? With
   the proxy measured from the clamped command, the delivered flights never
   escalate at h = 0.1 s, escalate on 5 % of position repairs at 0.5 s, and
   on 96 % at 1 s (ROS runs: first over θ between 0.35 s and 0.80 s).
2. **Soft rules.** Is "enforced up to brake, never a mode change" what the
   grant means by soft?
3. **N-in-T.** Onsets or ticks? (The grant lists this as open, p7.)
4. **Loiter timeout.** 30 s is ours. What should the operator default be?
5. **Is Loiter a fail-safe trigger** for the ≥ 99 % KPI, or only RTL / Land?
   The outcome vocabulary (Stress Testing p5) has only RTL_triggered and
   Land_triggered, but the Grant overview lists "ArduPilot fence / RTL /
   Land / Loiter as ultimate backstop".
6. Should a **P0 `monitor_only`** rule be allowed at all? It would show up in
   the escape rate, which is honest, but it is probably an authoring mistake
   that the policy lint should catch.
7. **Nowhere legal to stop and no repair converging.** G3 still sends this
   to LOITER, which holds the aircraft in the violation. X6 now bounds that
   to T (5 s after the position became illegal) instead of the 30 s
   timeout. Should G3 go straight to RTL whenever a stop is illegal?
8. **X6.** Is "position illegal for T → RTL, skipping Loiter" acceptable, and
   is the grant's T (5 s) the right persistence time? The alternative,
   counting such ticks as N-in-T events, reaches RTL 0.6 s after any
   incursion.
9. **A rescue where a stop is legal.** When the repair chain does not
   converge and the Shield finds a clean heading by search (ClearanceEscape)
   although a stop was legal, is that "no operator converges" (stop and
   escalate, as this module reads it) or a converged repair?

## Not done

- No integration: this unit did not touch shield.py, models.py, the ROS
  node, run_sitl_demo.py, audit.py, kpi.py, the sweep or scenarios.yaml
  (other units own them).
- No SITL flight per transition (card WP3-07's last step). It needs the
  wiring above, and a stop in WSL.
- No mission-profile YAML shipped. The audit record is a dict, not the
  grant's Pydantic model (WP3-11).
- `rtl_failed` / `home_reached` / `landed` detection is specified above,
  not built.
- The recovery list names this repo's operators as of 2026-10-06. A new
  recovery operator is capped until it is added to `RECOVERY_OPERATORS` or
  sets `recovery=True`. That is the safe direction, but it would show up as
  spurious escalations.
- The clamp-free proxy reproduces shield.py's first clamp pass. The clamp
  pass at the end of each repair iteration can trim a position operator's
  output again; the proxy then measures the trimmed correction. Only the
  per-operator source is exact.
- The X6 measurement on delivered flights depends on reconstructing
  `stop_illegal` offline; the plain replay cannot see it until the node logs
  the flag.
