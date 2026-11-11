# The fail-safe rubric: what "triggered" means, and when a scenario expects it

**Date:** 2026-11-02. **Status:** decided. The PI delegated the rubric as
"best practice / follow the grant" (PI-DECISIONS of 21 Oct, decision 2,
answered 2 Nov). This file was written and committed **before** any scenario
label was changed; the labels are then derived from it mechanically, and the
change list (section 5) names the clause behind every label that moved.
**KPI:** WP3-15, "Fail-safe trigger correctness | >= 99 % | Triggered when
expected, not when not expected" (Stress Testing, Acceptance KPIs).
**Code:** `guardrail/fsm.py` (`FAILSAFE_STATES`, `score_failsafe_triggers`),
`experiments/sweep_scenarios.py` (`score`, `failsafe_summary`),
`guardrail/scenario_spec.py` (the labels and their clauses),
`tools/failsafe_rubric.py` (the clause facts, measured per cell).

## 1. What counts as "the fail-safe triggered"

**The escalation FSM entering RTL or Land during the episode.** Brake and
Loiter are holds, not triggers. They are reported beside the KPI
(`loiter_entered`, the edge paths), never inside it.

Why, from the grant's own text:

- Safety Shield p1 names the fail-safe and its two modes: "detect violations
  -> repair via projection -> trigger fail-safe (RTL / Land) when repair is
  unsafe".
- Stress Testing, data record schema: `outcome` is `success | fail |
  RTL_triggered | Land_triggered`. The KPI's "triggered" is the outcome
  vocabulary's; there is no Brake or Loiter outcome.
- Safety Shield p4, the FSM: Brake and Loiter each have an edge back to
  Normal ("cleared for >= T_recover"). They pause the mission and can resume
  it. RTL and Land have no edge back: they end it.
- ArduPilot practice agrees: BRAKE and LOITER are position holds a mission
  resumes from; a failsafe action (FENCE_ACTION, FS_*) is RTL, SmartRTL or
  LAND.
- The Grant overview's WP3 row ("ArduPilot fence / RTL / Land / Loiter as
  ultimate backstop") lists the autopilot modes the Shield may command, not
  the KPI's trigger. Counting Loiter would make every N-in-T hold a "trigger".

## 2. When a scenario expects a fail-safe

A scenario expects a fail-safe if **any** clause E1-E5 holds within the
episode. Each clause is read from the scenario's physics, its pilot, its
policy and its ground truth (the true state, the true wind, the true
position error), **never from what the FSM did**: not from its state, its
edges or its outcome.

| clause | holds when | grant / practice |
|---|---|---|
| **E1 Policy demand** | a hard, enforced rule whose `violation_action` is `RTL` or `land` is violated by the planner's request (a soft rule is capped at `brake` and never demands one) | Policy DSL p2 (the six actions); Safety Shield p4, Normal -> RTL / Land "violation_action == RTL / Land (direct)" |
| **E2 Control authority** | a disturbance the aircraft cannot reject acts toward a hard constraint: a steady wind beyond the hold authority (the harness's `AP_HOLD_AUTHORITY_MPS`, 6.4 m/s), so no setpoint the Shield or the autopilot can fly keeps the aircraft legal | p1 ("when repair is unsafe": no repair can hold it); the reason ArduPilot has RTL / LAND failsafes at all |
| **E3 Position knowledge** | the position the Shield acts on is wrong by more than the policy's margin around a hard P0 boundary (`margin_m`) - white noise whose 3 sigma exceeds it, a state latency whose distance at the flown speed exceeds it, or a frozen fix held while the aircraft moves farther than it - while the aircraft is within that error of the boundary: the margin can no longer certify the P0 rule | the margin is sized for a position-error budget (experiments/templates/sensor_degradation.yaml); ArduPilot's EKF / GPS failsafes act when position knowledge is lost |
| **E4 No legal continuation** | no action within the kinematic caps keeps or makes the aircraft legal at all (every exit is out of reach before the constraint binds, and stays so) | p1. No library cell is judged under it today; it is listed so a future scenario has a clause |
| **E5 Unsafe repair that persists** | the planner keeps requesting, without a break, an action the Shield may not fly - a hard rule's repair over theta on the decided basis (each repair's forecast depth in metres; theta 2.0 m lateral / 0.5 m vertical) or one no operator converges on, or a violation of a hard rule whose action is `loiter` - for at least the operator's Loiter timeout plus one control period (`loiter_timeout_s` + 0.1 s = 30.1 s), judged from the Shield's decision on the request at the true position, on the episode **flown without a fail-safe** (the Shield's projection flown every tick; section 3) | p1 "trigger fail-safe (RTL / Land) when repair is unsafe"; p4 "Conservative cap. If ... > threshold, or no operator converges, the Shield abandons projection and triggers fail-safe", and "Loiter -> RTL: ... operator-defined timeout"; run-time assurance practice (a supervised function that stays untrusted past a bounded wait ends the mission) |

**Not expected**, stated so that no reader has to infer it:

- **N1** A planner the Shield can keep flying: its requests are clean or
  repaired within theta, however long it persists and however often its
  forecast re-touches a margin ring, and any spell of unsafe requests ends
  (clean, or repairable within theta) before the operator timeout on the
  flight without a fail-safe - the negation of E5. Brake is the grant's
  state for "violation, repairable, magnitude < theta" and has no timeout;
  p1 reserves the fail-safe for an unsafe repair.
- **N2** Position error within the margin (the margin was sized for it), or
  beyond it only where no hard P0 boundary is within that error - the
  negation of E3. N-in-T onsets that exist only in the Shield's noisy view
  of the position are not a planner fighting the Shield.
- **N3** A disturbance within the hold authority.
- **N4** A constraint imposed on the aircraft - a zone spawned, moved or
  grown over it, a window opening on it, a stand-off ring reclassified - that
  it can leave within the caps: the recovery operators' job (GeofenceEscape,
  StandoffRecover, the altitude recovery), not a fail-safe's. (E5 can still
  hold afterwards, if the planner then keeps asking for an unsafe repair.)
- **N5** N-in-T on its own. The grant's G2 (Brake -> Loiter) is a hold. An
  RTL reached by "N-in-T persists" (G4) is a correct trigger only where E1-E5
  holds; a planner that changes its OWN request in and out of a violation
  (deliberate thrash) is not in the library (open, section 6).

Why E5 and not a physics-only rubric: under a rubric of physics alone
(E1-E4), every Loiter -> RTL edge the grant draws - the operator timeout and
"N-in-T persists" - would be a false trigger by definition, because the
autopilot's hold keeps the aircraft legal. The grant draws those edges to end
a mission whose planner stays unsafe; the library's own
`failsafe-loiter-timeout` already labels that pattern "expected" for a
`loiter` rule. E5 applies the same reading to an over-theta repair, which p1
and p4 name as the fail-safe's condition in so many words. Section 5 gives
the KPI with and without E5, so the weight of this choice is visible.

## 3. How an episode is judged

- **Over an episode that shows the outcome.** An episode that ends while the
  FSM holds in Loiter with its operator timeout still running has not shown
  whether the fail-safe fires: its outcome is **censored**. The harness
  reports such an episode as not measured (`censored_in_loiter`, counted in
  `_failsafe_labels`), never as a correct non-trigger; a library family
  whose cells would end so flies long enough not to (`ticks`). Until
  2026-11-02, 27 library episodes ended in Loiter before the timeout, and
  the 25 of them that were labelled were scored "correct, not triggered"
  (rail-mission-stub-vla and altitude-ceiling-hold ended 0.1 s before it).
- **Per cell.** A family's label holds for every cell unless a cell is
  listed with its own label and clause (`failsafe_cells`). Every label names
  its clause (`failsafe_clause`), E1-E5 or N1-N5.
- **On the flight without a fail-safe.** Where a clause reads the episode
  (E1, E3's frozen fix and closest approach, E5), it reads the cell flown
  with the escalation FSM's verdict ignored: the Shield's projection is
  flown every tick, the aircraft is never held, sent home or landed. The
  flight the FSM shaped cannot be the ground truth for its own label: a
  Loiter that holds the aircraft beside a zone keeps the planner's request
  deep in it, so on that flight E5 would hold BECAUSE the FSM held - and an
  early RTL takes the aircraft away from the boundary E3 asks about.
  (Clarified on 2026-11-02 before any label was changed; the first wording
  said "on the flown episode".)
- **Mechanically.** E1 from the policy and the events; E2 from the wind
  parameter; E3 from the noise and latency parameters against the mission's
  planned route (start to target as declared) and, for a frozen fix, from
  the true-versus-seen error against the true distance to the boundary,
  tick by tick ("within" that error is inclusive); E5 from the measured
  spell of unsafe requests. (Amended 2026-11-03, after the relabelling:
  the first wording read noise and latency against the closest true
  approach of the flight, which depends on the noise draw - the nightly
  check flew gps-noise-beyond-margin latency 5 / pass_y 23.2 / sigma 1 a
  few millimetres beyond its 3.0 m error on seed 1003 and to 2.7 m on
  seeds 1001 and 1002, one cell with two labels, which this section
  forbids. No label changed with the amendment.) `tools/failsafe_rubric.py`
  prints these facts per cell and the label they give, beside the declared
  one, and exits non-zero when they disagree or when a spell is still
  running at the end of an episode too short to decide it.
- **On the grant's configuration.** Labels are derived for the headline
  arm: the grant's 5 s / 0.1 s forecast on the ideal vehicle. The 3 s
  forecast and ArduPilot-response comparison arms are scored against the
  same labels and reported apart, never gated.

## 4. What this rubric does not decide

- How good the FSM is: it says when a trigger is right, not how to produce
  one. A clause the FSM has no input for (E3 with a frozen fix: the Shield
  had no fix-quality field until 2026-11-06) still makes the scenario expect
  a fail-safe; where the FSM does not trigger, that is a missed trigger,
  reported with its mechanism. Since 2026-11-06 the Shield reads a rail's
  fix validity and accuracy (`State.fix_ok`, `State.pos_sigma_m`) and the
  FSM answers E3 through X8 (docs/DESIGN-escalation-fsm.md); a rail that
  reports neither is still in the position this paragraph describes, and
  the rubric's clauses did not change with it. E2 had the same gap on the
  hil rig until 2026-11-11: the FSM's only answer to a wind beyond the hold
  authority was X7, which needs the aircraft pushed somewhere illegal, and
  on sm1111_048 the hold was carried round the zone with the position legal
  throughout. Since then the FSM judges the hold itself (X9: the aircraft
  carried beyond what a kept hold can carry it, and still receding;
  section 5, "2026-11-11"); the clauses and labels did not change with it.
- The theta basis (decided separately: each repair's forecast depth in
  metres, reading A of docs/DESIGN-escalation-fsm.md).
- The operator timeout (30 s, `loiter_timeout_s`): the rubric reads it as
  configured. Changing it changes which E5 cells complete inside an episode,
  and is not done to move the KPI.

## 5. Applied to the library (2026-11-02, re-checked 2026-11-03, re-run 2026-11-06)

`tools/failsafe_rubric.py` on the library (seed 0), the smoke profile and
the nightly profile (seeds 1001-1003, 672 cell-seeds): every declared label
agrees with the rubric and no cell is undecided in its length (after the
E3 amendment of section 3). Every label in experiments/templates names its
clause.

**Labels that changed** (family, or cell where listed; was -> now; clause):

| scenario | was -> now | clause | the fact, on the flight without a fail-safe |
|---|---|---|---|
| altitude-ceiling-hold | false -> **true** | E5 | the climb into the ceiling is a vertical repair over theta for the whole 49.9 s |
| corridor-drift-angles | false -> **true** | E5 | drifting out at an angle asks for a repair over theta for 34.7-65.0 s |
| corridor-drift-angles, drift_deg 0 | false (unchanged) | N1 | along the axis: nothing to repair |
| corridor-curfew-in-hours | false -> **true** | E5 | asks for the curfewed corridor for 49.9 s, over theta |
| standoff-reclassified | false -> **true** | E5 | into the pedestrian ring, over theta, for 33.4 s |
| standoff-wedge | false -> **true** | E5 | through the subject's ring for 49.9 s, over theta |
| dyn-nfz-spawn-ahead | false -> **true** | E5 | through the zone, over theta, for 34.9 s where the spawn lands more than theta inside the forecast |
| dyn-nfz-spawn-ahead, ahead 12 m at 2 m/s; ahead 30 m | false (unchanged) | N1 | the forecast meets the ring within theta: repairable |
| time-window-opens-ahead | false -> **true** | E5 | into the zone opened ahead, over theta, for 45.9-46.9 s |
| rule-activated-mid-run | false -> **true** | E5 | into the activated zone, over theta, for 45.9-47.3 s |
| radius-scaling-grow | false -> **true** | E5 | into the grown zone, over theta, for 35.7-38.2 s |
| radius-scaling-grow-deep | false -> **true** | E5 | into the grown zone, over theta, for 33.2 s |
| corridor-swap-shifted | false -> **true** | E5 | out of the swapped corridor, over theta, for 40.9 s |
| corridor-swap-narrowed | false -> **true** | E5 | out of the narrowed corridor, over theta, for 44.9-46.9 s |
| three-simultaneous-violations | false -> **true** | E5 | the descent below the floor and the dash into the zone are repairs over theta for the whole 39.9 s |
| gps-noise-beyond-margin | false -> **true** | E3 | 3 sigma 3-6 m or latency 2-4 m against the 1 m margin, on a route 0.2-0.6 m from nfz-square's face |
| gps-dropout | unlabelled -> **true** | E3 | a frozen fix held while the aircraft moves farther than the margin, within that error of nfz-square |
| gps-dropout, window 2-4 s; lane 15 with window 6-13 s | unlabelled -> false | N2 | the error never comes within reach of the boundary (far from the zone; frozen on a held aircraft) |

Every other family kept its label and gained its clause: E1 (the four
RTL / land zone families), E2 (wind-beyond-authority), E5
(failsafe-loiter-timeout), N1, N2 (gps-noise-within-margin), N3 (the
wind families within authority) and N4 (zones, windows and rings imposed
on the aircraft).

**Outcome labels** the clause contradicted, now RTL_triggered (E5):
altitude-ceiling-hold, corridor-drift-angles (not at 0 deg),
standoff-reclassified (was fail), standoff-wedge, three-simultaneous-
violations.

**Lengths** (section 3): 13 families fly 50 s (corridor-drift-angles 70 s).

**Pins.** Removed: dyn-nfz-spawn-ahead (4 cells) and three-simultaneous-
violations (now the fail-safe their label expects), near-edge-at-12mps (2
cells; fixed in the FSM, not by a label). Added: rail-mission-stub-vla (a
false fail-safe, below) and corridor-swap-shifted (its expected RTL flies
home outside the swapped corridor: guardrail/rtl_path.py does not judge a
keep-in corridor). Re-described: standoff-approach-angles 7 m pedestrian;
standoff-wedge now pins the goal only.

**Results** (headless, not KPI-grade; correctness, false / missed, the
never- and always-trigger nulls, Wilson 95 % lower bound):

| profile | before (labels and FSM of 7bfd6dc) | after |
|---|---|---|
| library | 145/171 = 0.848 (25 / 1), 0.942 / 0.058, 0.787 | 175/179 = 0.978 (2 / 2), 0.659 / 0.341, 0.944 |
| smoke | 57/61 = 0.934 (3 / 1), 0.885 / 0.115, 0.843 | 62/63 = 0.984 (1 / 0), 0.635 / 0.365, 0.915 |
| nightly | 547/717 = 0.763 (167 / 3), 0.941 / 0.059, 0.730 | 721/741 = 0.973 (14 / 6), 0.623 / 0.377, 0.959 |

With E5 removed (its labels read as "not expected"): library 136/179 =
0.760 (41 false, 2 missed; never-trigger null 0.877), smoke 48/63 = 0.762
(15 false), nightly 520/741 = 0.702 (215 false, 6 missed). E5 carries most of the change; it is the grant's own condition
(p1, p4) and is kept (decided 2026-11-03).

**What remains wrong:**

- False: rail-mission-stub-vla and standoff-approach-angles 7 m
  pedestrian. Theta escalates the first forecast that meets the zone or
  ring (8.1 m deep from the pad; more than 2 m into the pedestrian ring),
  the autopilot holds the aircraft where the request stays deep, and the
  30 s timeout sends it home - while, flown without a fail-safe, the
  projection takes the aircraft round in 10.9 s / 1.5 s of unsafe requests
  (N1). The Gazebo Harmonic + ArduCopter SITL flight of the rail mission
  (master 70ab5e8) flew the same edges.
- Missed: gps-dropout on lane 23.6 with windows 2-9 s and 6-13 s. The
  Shield has no fix-quality input; a frozen fix while sliding along the
  edge never makes a repair unsafe.

### Re-run on 2026-11-06: X8, the noise hysteresis, every recovery at its cap

No clause and no label changed; E5 is as written above. What changed is
the FSM and the Shield (docs/DESIGN-escalation-fsm.md, "X8, the position
the Shield is handed"; docs/DESIGN-shield-enforcement.md, "Every recovery
at its cap"): the Shield reads a rail's fix validity and accuracy, the FSM
returns the aircraft when the position can no longer certify a hard P0
margin (X8), noise inside the margin no longer starts new N-in-T onsets,
and the Shield's recoveries fly at their caps. The flight without a
fail-safe changed with the recovery law, so the labels were checked again:
`tools/failsafe_rubric.py` on the final code finds every declared label in
agreement with the rubric and none undecided in its length (library 168
cells, smoke 56, nightly 672 cell-seeds).

| profile | 3 Nov (above) | 6 Nov | never / always null | Wilson 95 % low |
|---|---|---|---|---|
| library | 175/179 = 0.978 (2 / 2) | 177/179 = 0.989 (2 / 0) | 0.659 / 0.341 | 0.944 -> 0.960 |
| smoke | 62/63 = 0.984 (1 / 0) | 62/63 = 0.984 (1 / 0) | 0.635 / 0.365 | 0.915 -> 0.915 |
| nightly | 721/741 = 0.973 (14 / 6) | 729/741 = 0.984 (12 / 0) | 0.623 / 0.377 | 0.959 -> 0.972 |

(headless, not KPI-grade; false / missed in brackets.)

- **Missed: none.** The gps-dropout misses (lane 23.6, windows 2-9 s and
  6-13 s: library 2, nightly 6, every seed) are X8 triggers at 5.3 s and
  7.6 s.
- **The two noise false triggers are gone** (gps-noise-within-margin
  latency 0 / pass_y 23.2 / sigma 0.3, seeds 1001 and 1003): with the
  rail's accuracy reported, a clean tick within 3 sigma of the ring no
  longer ends the violation, so the noise starts no new onsets.
- **What remains wrong: the 15 false triggers of theta on the first deep
  forecast** - rail-mission-stub-vla (library, smoke, nightly x 3 seeds)
  and standoff-approach-angles 7 m pedestrian (library; nightly, 3 variants
  x 3 seeds). The first forecast that meets the zone or ring is already
  more than theta deep, so the FSM abandons projection and holds the
  aircraft (X1, G3), the held request stays deep, and the operator timeout
  sends it home (G5 at 30.1 s); flown without a fail-safe, the projection
  takes the aircraft round in 10.9 s and 1.5 s of unsafe requests (N1).
  Under the decided theta basis and E5 as written they are false triggers,
  and no change to the FSM can answer them: holding on an over-theta
  repair is what p4 prescribes, and the hold is what keeps the request
  over theta. Reading E5 on the flight the grant prescribes (projection
  abandoned, the aircraft held) instead of the flight without a fail-safe
  would label them "expected"; that is a change to this rubric, a decision
  for the user, and not made here.
- No profile reaches the grant's 0.99, as a point estimate or as a Wilson
  lower bound; the library's 0.989 is one episode short of it (178/179 =
  0.994).

### 2026-11-11: E2 on the rig, and X9 (a hold that is not kept)

No clause and no label changed. What changed is the FSM
(docs/DESIGN-escalation-fsm.md, "2026-11-11: X9, a hold that is not kept").

**The miss.** On the KPI-grade hil smoke sm1111 (48 missions, 22d434b),
read with the rail vehicle's labels and E5 (r2), the KPI is 44/45 = 0.978,
and the one miss is sm1111_048: wind-beyond-authority, lane 15, 12 m/s,
labelled expected by E2. The FSM went G1 (5.3 s), G3 (5.8 s: the repair
over theta), and the companion held the point (-11.8, 15.0) in GUIDED,
commanding up to 4 m/s back to it. The SITL airframe flew the mission's
4 m/s with the 12 m/s tailwind (3.3-3.9 m/s over the ground) but could not
hold still in it: from the Loiter's start the aircraft went 2.0-4.5 m/s
away from its point, 28 m in 9.4 s, and sideways round nfz-square (2.4 m
from its face at 13.1 s, never inside). The position stayed legal, so X6
and X7 never ran; once the aircraft was past the zone the request turned
clean, and G8 handed it back at 15.3 s. The mission then reached its goal
downwind. The 8 m/s cell (sm1111_047) held: it overshot its point by 3.6 m
and was back on it 6 s later, and the operator timeout sent it home (G5 at
35.8 s, correct by E2). Within authority (5 m/s, sm1111_045 / 046) the hold
was kept too (G5 on lane 15, which r2 expects; nothing on lane 25).

**The answer, X9.** In Loiter, the FSM is told how far the aircraft is from
the point its hold flies to and how far a kept hold can carry it (the
distance when the point was set, plus the braking distance from the speed
then on the rail Shield's own stop model, plus 3 m of settling;
guardrail/hold_flight.py HoldWatch). Beyond that for T_recover and still
receding (X7's own trend test on that distance), it returns: RTL, the
rung after Loiter. Replayed on every Loiter the hil rig has flown (413
spells, hs02 to nt1111), X9 fires on the five 12 m/s cells
(hs02_009, hs03_011, rh03_044, sm1110_048, sm1111_048: at 11.2-11.4 s,
12-15 m out, receding at 1.7-2.8 m/s) and on nt1111_001 (nfz-head-on, whose
SITL restarted 2.5 s before the Loiter and handed the node a stale pose and
velocity: the aircraft accelerated to 4 m/s inside its hold and went 9.3 m
past its point; r2 expects a fail-safe there, which G5 gave at 38.8 s). It
fires on no 8 m/s cell and no within-authority cell; the closest any other
spell came to its reach was 0.8 m short (hs04_005 / 006, the BRAKE
hand-over flow of 7 Nov). Predicted for sm1111 under r2: 45/45. Headless,
the wind-beyond-authority cells on the ideal vehicle go home by X9 (6.8 s at
12 m/s, 9.0 s at 8 m/s) instead of X7 (7.3 s, 17.3 s): the same label met,
earlier, before the drift reaches the zone. The rail vehicle's GUIDED hold
keeps the 8 m/s cell, as the rig did. Not re-run here: the library, smoke
and nightly profiles (headless) and any hil flight.

**A question for the PI (a documented deviation).** X9 is an extension
edge; the grant's FSM draws Loiter -> RTL only for N-in-T persisting (G4)
and the operator timeout (G5). It reads p1 ("trigger fail-safe (RTL / Land)
when repair is unsafe") for the last repair the Shield has, the hold, as X7
already did for a hold pushed into a zone. And it judges the hold, not E2's
"acts toward a hard constraint": a wind beyond the hold authority blowing
away from every constraint would trigger it too (no library cell has one).
An aircraft that cannot hold still cannot keep any stop the Shield may
need next, which is why it was not narrowed.

## 6. Open

- No scenario has a planner that thrashes by changing its own request (N5):
  N-in-T persisting (G4) is exercised only by the harness's position noise.
- E3's frozen fix is a ground-truth clause the Shield could not observe
  until 2026-11-06. It now can where a rail reports fix validity
  (`State.fix_ok`), and the headless harness models a rail that does
  (`stress.fix_quality_reported`, default true); no flight rail reports it
  yet (docs/DESIGN-escalation-fsm.md, "Where each rail's signal lives"),
  and ArduPilot's own EKF failsafe is still not modelled.
- E2 is a parameter clause (the declared wind against the harness's
  6.4 m/s authority), and the SITL airframe on the rig holds 8 m/s: its
  8 m/s cells are fail-safes by the operator timeout (G5), not by a hold
  lost. X9 (2026-11-11) judges the hold the rig flies, so it fires where the
  airframe's own authority ends, not where the label's parameter says it
  does; the two agree on every rig cell flown so far.
- Nothing here is KPI-grade: the grant's KPI is measured on the hil topology.
