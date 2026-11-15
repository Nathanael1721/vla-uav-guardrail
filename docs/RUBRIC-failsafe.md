# The fail-safe rubric: what "triggered" means, and when a scenario expects it

**Date:** 2026-11-02. **Status:** decided. The PI delegated the rubric as
"best practice / follow the grant" (PI-DECISIONS of 21 Oct, decision 2,
answered 2 Nov). This file was written and committed **before** any scenario
label was changed; the labels are then derived from it mechanically, and the
change list (section 5) names the clause behind every label that moved.
**Amended 2026-11-10:** sections 7 and 8 are adopted (the user's decision of
9-10 November): E5 is read on the flight the grant prescribes, and the
KPI-grade labels are derived on the vehicle that flies them. Unlike the
rubric of 2 November, that amendment came AFTER flights it bears on (the rig
showed the autopilot's lag on 7 November), so it is not a pre-registered
reading; the readings of sections 2-5 are still scored and printed beside it.
**Amended 2026-11-15:** section 9, two decisions: a sixth clause, E6 "N-in-T
persists (the grant's G4)", and an ArduPilot GeoFence RTL / LAND taken while
the FSM holds counted as a fail-safe triggered by the backstop. Decided
2026-11-15 by the lab under the PI's instruction of 6 Oct (follow the grant;
best practice where it is silent); the PI's confirmation is still to be
recorded. Both came AFTER flights they bear on (hs10_003 on 14 November,
ntFIN2 on 13 November), so neither is a pre-registered reading, and the
readings before them are still scored and printed beside them.
**KPI:** WP3-15, "Fail-safe trigger correctness | >= 99 % | Triggered when
expected, not when not expected" (Stress Testing, Acceptance KPIs).
**Code:** `guardrail/fsm.py` (`FAILSAFE_STATES`, `score_failsafe_triggers`),
`experiments/sweep_scenarios.py` (`score`, `failsafe_summary`; since
2026-11-15 `failsafe_backstop` and `BACKSTOP_RULE`),
`guardrail/scenario_spec.py` (the labels and their clauses),
`tools/failsafe_rubric.py` (the clause facts, measured per cell; since
2026-11-15 `e6_facts`).

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

**Since 2026-11-15 (section 9, decision 14): also the autopilot's GeoFence
taking the aircraft to RTL or LAND on a fence breach while the FSM holds in
Brake or Loiter.** That is a fail-safe triggered by the grant's backstop, not
by the FSM: recorded as such (`failsafe_by` "autopilot_geofence", against
"fsm"), scored against the scenario's label like any other trigger - correct
where a fail-safe is expected, a false trigger where it is not - and counted
beside the KPI wherever the KPI is printed, with the reading before it (such
a run left out as censored) printed beside. A hold the autopilot did not take
is unchanged: Brake and Loiter are still holds, and an episode that ends in
Loiter with its timeout running is still censored (section 3).

## 2. When a scenario expects a fail-safe

A scenario expects a fail-safe if **any** clause E1-E6 holds within the
episode (E6 since 2026-11-15, section 9). Each clause is read from the scenario's physics, its pilot, its
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

| **E6 N-in-T persists (the grant's G4)** | hard-violation ONSETS **made by the world** recur at or above the FSM's N-in-T rate through a Loiter: a hard, enforced rule is switched on or over the planner's request again and again (spawned, activated, its window opened, swapped, resized or set moving: a world event of the scenario lands on the very tick the violation starts, and names the violated rule), at least 2 x N times, every N consecutive ones within T, with the request never free of a hard violation for `t_recover_s` between them (N = 3, T = 5 s, T_recover = 2 s; at most one onset per `onset_clear_s` = 1 s, as the grant's N-in-T is counted) - the N that take the grant's G2 (Brake -> Loiter) and N more after them (G4) - judged from the Shield's decision on the request at the true position, on the flight without a fail-safe (section 3) | p4, the FSM: "Brake -> Loiter: N-in-T threshold", "Loiter -> RTL: N-in-T persists". Added 2026-11-15, after the flown run hs10_003 (section 9) |

(E5's flight since 2026-11-10: the prescribed one, section 7. The row above
is kept as written on 2 November; it is the reading (a) and (r) are still
scored on. E6 was added on 2026-11-15: section 9 says why, and that the label
it gives followed the flight.)

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
  RTL reached by "N-in-T persists" (G4) is a correct trigger only where E1-E6
  holds (until 2026-11-15: E1-E5; E6 now names the one case in which G4 is
  itself the reason, onsets made by the world); a planner that changes its
  OWN request in and out of a violation (deliberate thrash) is not in the
  library (open, section 6).

**The boundary between E6 and N1 / N2** (2026-11-15). N1 and N2 are as
written above for everything else; E6 is narrow, and what separates it from
them is who makes the onsets:

- **E6**: the WORLD makes them. Each counted onset starts on the tick a
  world event of the scenario puts the violated hard rule in force or changes
  it (a hot-apply that spawns, activates, swaps, resizes or sets moving that
  rule, or its time window opening), 2 x N of them at the N-in-T rate with no
  T_recover of clear request in between. How the Shield answers each onset
  does not matter: fsm-n-in-t-flicker's are each answered by the `brake`
  rule's own trusted stop, which under N1 alone would expect no fail-safe.
- **N1**: the planner's forecast makes them. A rule that is simply in force,
  whose margin ring the forecast leaves and re-touches as the aircraft or the
  zone moves, starts an onset on a tick with no such world event; those
  onsets are never counted for E6, however many there are, and a planner
  that is otherwise repairable stays N1.
- **N2**: the Shield's view of the position makes them. An onset that exists
  only because the position the Shield acts on is noisy, late or frozen
  lands on no world event either, and is never counted for E6.
- A single world event (one spawn, one window opening, one swap: N4) is one
  onset, far from 2 x N; E6 needs the switch to recur. An onset fewer than
  `onset_clear_s` after the last counted one is the same switch, not a new
  onset.

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
  its clause (`failsafe_clause`), E1-E6 or N1-N5.
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
  forbids. No label changed with the amendment.) E6 (since 2026-11-15) from
  the same decisions and the episode's own event record: the onsets of a
  hard violation (a tick whose request violates a hard, enforced rule after
  a tick whose request violated none), kept where a world event recorded
  since the previous tick names a rule violated on that tick, at most one
  per `onset_clear_s`; E6 holds on a run of 2 x N of them in which every N
  consecutive ones lie within T and no spell without a hard violation
  between them lasts `t_recover_s`. `tools/failsafe_rubric.py`
  prints these facts per cell and the label they give, beside the declared
  one, and exits non-zero when they disagree or when a spell is still
  running at the end of an episode too short to decide it.
- **On the grant's configuration.** Labels are derived for the headline
  arm: the grant's 5 s / 0.1 s forecast on the ideal vehicle. The 3 s
  forecast and ArduPilot-response comparison arms are scored against the
  same labels and reported apart, never gated. (Since 2026-11-10 a KPI-grade
  run on the hil rail is scored on labels derived on the rail vehicle:
  section 8.)

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
  (Since 2026-11-14 also by fsm-n-in-t-flicker, flown by the fsm_edges
  profile alone, whose onsets the WORLD makes; since 2026-11-15 that cell is
  expected by E6, section 9. A planner thrashing its own request is still
  not in the library, and E6 does not cover it.)
- E3's frozen fix is a ground-truth clause the Shield could not observe
  until 2026-11-06. It now can where a rail reports fix validity
  (`State.fix_ok`), and the headless harness models a rail that does
  (`stress.fix_quality_reported`, default true); no flight rail reports it
  yet (docs/DESIGN-escalation-fsm.md, "Where each rail's signal lives"),
  and ArduPilot's own EKF failsafe is still not modelled. (Since 2026-11-07
  the hil node reports `fix_ok`; since 2026-11-12 the rail labels read a
  denial as the rail realises it, EKF3's dead-reckoned estimate - section 8,
  "2026-11-12" - and on the rig X8's 0.5 m/s drift allowance, not E3, is
  what fires on lane 23.6 at 4-11 and 6-13 s: an open decision. Later on
  2026-11-12 the node reads the autopilot's own verdict instead, ArduPilot's
  EKF failsafe input included: section 8.)
- E2 is a parameter clause (the declared wind against the harness's
  6.4 m/s authority), and the SITL airframe on the rig holds 8 m/s: its
  8 m/s cells are fail-safes by the operator timeout (G5), not by a hold
  lost. X9 (2026-11-11) judges the hold the rig flies, so it fires where the
  airframe's own authority ends, not where the label's parameter says it
  does; the two agree on every rig cell flown so far.
- Nothing in sections 1-5 is KPI-grade: the grant's KPI is measured on the
  hil topology, on the labels of section 8.

## 7. E5 read on the flight the grant prescribes (adopted 2026-11-10; drafted 2026-11-06)

**Status: adopted.** Decided by the user on 9-10 November 2026 ("follow the
best practice that meets the KPI criteria"), with section 8. Three things a
reader must know about that adoption:

- **It is not a pre-registered reading.** The section was drafted on
  6 November and adopted after the rig had shown the autopilot's lag (rh01,
  7 November) and after hil campaigns had been flown and scored on the
  earlier reading. The rubric of 2 November was written before any label
  moved; this amendment was not.
- **The earlier readings are still printed.** Every campaign is scored on
  section 3's reading as well - (a) on the ideal-vehicle labels, (r) on the
  rail-vehicle labels - beside the adopted (r2); the label artefact
  (docs/data/failsafe_labels_rail.json) carries all four readings, and the
  scoring tool prints the false and missed cells of each by name.
- **What changed in the library:** the tool's default (`--e5-flight
  prescribed`), the labels of the two cells below (experiments/templates:
  rail-mission-stub-vla, standoff-approach-angles 7 m pedestrian), their
  pins, the tests.

The text below is the draft as it was put to the decision; its numbers were
measured at the time, on the code named.

**The question.** E5 asks whether the planner keeps requesting an action the
Shield may not fly for the operator timeout plus one control period. Section
3 reads it on the flight *without a fail-safe*: the Shield's projection flown
every tick. On that flight an over-theta repair is flown - which is exactly
what the grant says does not happen (Safety Shield p4: "If ... > threshold,
or no operator converges, the Shield abandons projection and triggers
fail-safe"). The fifteen remaining false triggers (section 5,
rail-mission-stub-vla and standoff-approach-angles 7 m pedestrian) are the
cells where this matters: flown with the over-theta projection the aircraft
is taken round the zone in 10.9 s / 1.5 s of unsafe requests (N1), while
flown as the grant prescribes - projection abandoned, the aircraft held -
the request stays over theta past the timeout.

**The reading.** E5 is read on the *prescribed* flight
(`tools/failsafe_rubric.py`, `prescribed`):

- on a tick whose request the Shield may not fly (the E5 tick of section 2:
  a hard rule's repair over theta or not converged, or a hard `loiter` rule
  violated), the projection is not flown and the aircraft is held - a stop,
  as Brake and the autopilot's Loiter hold it on the ideal vehicle - unless
  a stop there is illegal, where the Shield's own action flies (a hold
  cannot);
- on every other tick the Shield's action flies: projection resumes the
  tick the request is acceptable again;
- nothing reads the escalation FSM: not its state, edges, T_recover,
  N-in-T or the Loiter timeout. The flight is the grant's prescription, not
  this FSM's implementation of it, so a label still cannot follow from what
  the FSM did.

E5 holds when the longest unbroken spell of such requests on that flight
reaches `loiter_timeout_s` + 0.1 s (30.1 s). E1-E4 are read as before (E1
and E3 on the flight without a fail-safe).

**Why it is not circular.** The objection to judging E5 on the FSM's own
flight (section 3) is that the FSM's hold makes the request persist, so the
label would follow from the FSM. The prescribed flight holds too - that is
the point of the grant's Loiter -> RTL timeout: a planner that keeps asking
for an unsafe repair while held is the case the operator timeout exists to
end. What this reading removes is every choice of this implementation:
it holds on the grant's own condition (the E5 tick) and resumes on its
negation, with no timers or counters of ours.

**What it costs.** It labels "expected" a planner whose own path would have
been safe had the projection been flown over theta - the rail mission's stub
is taken round the zone in 10.9 s on that flight. Whether the KPI should
count that as a needed fail-safe is the decision. The decided theta basis
is unchanged either way.

**Labels it moves** (`tools/failsafe_rubric.py`, both readings printed;
library seed 0, smoke, nightly seeds 1001-1003; run on the Shield and FSM
of e43bceb, which nothing up to c07998d changes on Python 3.10): 15
episodes, every one from "not expected" (N1) to "expected" (E5), none the
other way - exactly the fifteen false triggers of section 5:

| cell | episodes | E5 spell, held / free |
|---|---|---|
| rail-mission-stub-vla | library s0; smoke s1001; nightly s1001-1003 | 49.9 s / 10.9 s |
| standoff-approach-angles, offset 7 m, pedestrian | library variant 0; nightly variants 0-2 x s1001-1003 | 39.9 s / 1.5 s |

Every other cell keeps its label. Where the two spells differ at all they
stay on the same side of 30.1 s: corridor-drift-angles at -30 and -60 deg
(34.7-56.2 s free, 56.2-56.7 s held: E5 either way) and
gps-noise-beyond-margin (0.0-0.2 s either way: E3). No cell is undecided in
its length under either reading (wind-beyond-authority at 8 m/s ends inside
a spell on both, and is labelled by E2).

**Measured when it was drafted** (labels as above, on branch 1's code at
c07998d; headless, not KPI-grade):

| profile | section 3's reading (branch 1) | section 7's reading | Wilson 95 % low |
|---|---|---|---|
| library | 177/179 = 0.989 (2 / 0) | 179/179 = 1.000 (0 / 0) | 0.960 -> 0.979 |
| smoke | 62/63 = 0.984 (1 / 0) | 63/63 = 1.000 (0 / 0) | 0.915 -> 0.943 |
| nightly | 729/741 = 0.984 (12 / 0) | 741/741 = 1.000 (0 / 0) | 0.972 -> 0.995 |

(false / missed in brackets.) The never-trigger nulls fall with the
relabelling (library 0.659 -> 0.648, smoke 0.635 -> 0.619, nightly
0.623 -> 0.607). Nothing else moves: the code is branch 1's, so P0
escape (0.0), time to safe, the true-state counts and the flights are
identical; the sweeps' own gates: library 155 pass / 22 tracked / 0 fail /
3 known, smoke 56 / 6 / 0 / 2, nightly 663 / 66 / 0 / 15 (the rail
mission's pin is off, the 7 m cell's keeps only the goal).

**Re-derived on 2026-11-11** on the code that flies (section 8): the same two
cells move and nothing else does - on the ideal vehicle rail-mission-stub-vla
(smoke 1, nightly 3 cell-seeds: 49.9 s held, 10.9 s free) and
standoff-approach-angles 7 m pedestrian (nightly 9: 39.9 s held, 1.5 s free),
and no declared label disagrees with the tool (0 of 764 judged cell-seeds); on
the rail vehicle the same two cells (r to r2: 49.9 s held, 9.8 s free; 39.9 s
held, 2.3 s free).

## 8. The KPI-grade labels are derived on the vehicle that flies them (adopted 2026-11-10; drafted 2026-11-07)

**Status: adopted**, with section 7, by the user's decision of 9-10 November
2026 ("follow the best practice that meets the KPI criteria"). The same three
things hold as for section 7: it was adopted AFTER the rig had shown the
autopilot's lag (rh01, 7 November) and after KPI-grade campaigns had flown,
so it is not a pre-registered reading; the earlier readings are still
printed beside it; and what changed is named below.

**The adopted reading.** The fail-safe correctness of a KPI-grade run on the
hil rail is judged on **(r2)**: this rubric's labels derived on the rail
vehicle, with E5 read on the prescribed flight (section 7). Beside it, on
every campaign, by name: **(a)**, the ideal-vehicle labels on section 3's
reading (what every campaign was scored on until now), and **(r)**, the
rail-vehicle labels on section 3's reading. The headless labels in
experiments/templates stay the ideal vehicle's (a2: section 7's reading).

- **The vehicle** is `tools/failsafe_rubric.py --response rail`: the harness's
  `RAIL_VEHICLE`, the response fitted to the rig's own flights and entered
  from a hover, as a rail starts (docs/DESIGN-harness-vehicle-response.md).
  Every flight the rubric reads - section 3's flight without a fail-safe and
  section 7's prescribed one - is flown on it; a held aircraft is the zero
  setpoint flown through that response. The draft of 7 November used the
  Copter-4.5.7 fit (`--response ardupilot`), the only one there was then.
- **The artefact** is docs/data/failsafe_labels_rail.json
  (`tools/failsafe_rubric.py --build-labels`): for every Shield-on, FSM-on
  cell and seed of the smoke, grant_example and nightly profiles, the four
  readings a, a2, r, r2, each with the clauses that hold, its E5 spell and
  the length it was read at, the commands that derived them and the code
  revision each ran at.
- **Undecided is not guessed, and is read at the length the rail flies.**
  A label is derived at the scenario's own length. Where no clause holds
  and an E5 spell is still running when the episode ends, it is undecided
  there. The draft asked that "a label left undecided" be flown max_s +
  `loiter_timeout_s` + 5 s, and the rails have flown exactly that long
  since (`sweep_scenarios.rail_max_s`; a rail run's fail-safe verdict is
  read over the whole flight). So a rail label undecided at the scenario's
  length is read on the cell flown 35 s longer and marked `length: "rail"`;
  one undecided even then would stay undecided (`expected` null), left out
  of the count and named. A label decided at the scenario's length is never
  replaced by the longer flight's; a difference would be listed
  (`built.differ_flown_longer`). Every score names the runs judged on a
  label read at the rail's length and prints the count without them.
- **The scoring path** is `experiments/sweep_scenarios.py --rescore-rail
  <run folders> --profile <the campaign's>`: every fetched run re-scored
  with the checkout's scorer, then the fail-safe trigger correctness under
  (a), (r) and (r2) - count, rate, Wilson 95 % lower bound, both nulls, and
  by name the censored, undecided and unlabelled runs and the false and
  missed triggers. (r2) is marked the adopted KPI reading. Shield-off runs
  are left out and counted.

**Why the labels do not depend on the FSM that flew.** Neither flight the
labels read steps on the FSM's verdict (sections 3 and 7), so a change of
the FSM's law cannot move a label: the smoke's rail labels derived on the
harness of 9eed35b (hand-back "as_held", 100-unit slices) and on the code
that flies (hand-back "at_speed", 60-unit slices) are identical: 112 of 112
readings (56 cell-seeds, both readings of E5), label, clauses and E5 spell
alike. The
KPI-grade campaigns flown at 9eed35b are therefore scored on the same
artefact as the ones flown after it.

### Re-derived on 2026-11-11, on the code that flies

`tools/failsafe_rubric.py` at 442397f (the scenario-length derivations) and
897dac7 (the longer flights): this branch's tool on b36c0e6 and on 22d434b,
the commit that flies, which differ only in the rail's node, both vehicles:

Expected / not expected, per reading; in brackets, how many of the expected
were undecided at the scenario's length and are read on the cell flown as long
as the rail flies:

| profile (cell-seeds) | a | a2 | r | r2, adopted |
|---|---|---|---|---|
| smoke (56) | 23 / 33 | 24 / 32 | 28 / 28 (3) | 29 / 27 (3) |
| grant_example (36) | 0 / 36 | 0 / 36 | 0 / 36 | 0 / 36 |
| nightly (672) | 279 / 393 | 291 / 381 | 333 / 339 (18) | 345 / 327 (18) |

708 cell-seeds in all (the smoke's are the nightly's seed 1001). No reading is
undecided after the longer flight, and no label decided at the scenario's
length is read differently on it (checked on every smoke cell and on the four
nightly families that had an undecided cell: 0 differ;
`built.differ_flown_longer`). The ideal labels are the templates': 0 of 764
judged cell-seeds disagree (`a2`).

Where the rail labels differ from the ideal ones, on the same reading (smoke
5, nightly 60 cell-seeds):

| cells (nightly cell-seeds) | ideal -> rail | read |
|---|---|---|
| dyn-nfz-spawn-ahead 12 m at 2 m/s (9) and 30 m at 4 and 8 m/s (18); gps-dropout lane 15, windows 2-4 and 6-13 s (6); wind-gusts lane 15 at 2 m/s (3); wind-within-authority lane 15 at 5 m/s (3) | not expected -> expected (E5, spells of 32.1-44.1 s) | at the scenario's length |
| nfz-head-on (3); dyn-nfz-rotate-step (3); time-window-opens-on-sideways-pilot (3); dyn-nfz-spawn-ahead 30 m at 2 m/s (9) | not expected -> expected (E5) | undecided at the scenario's length (spells of 20.1-26.6 s still running when the episode ends); E5 on the flight as long as the rail flies (55.1-61.6 s) |
| gps-dropout lane 23.6, window 6-13 s (3) | expected (E3) -> not expected | at the scenario's length: on the rig's response the aircraft's closest true approach to the fence is 4.36 m (ideal 1.0 m), so E3 does not hold |

Section 7's two cells (rail-mission-stub-vla, standoff-approach-angles 7 m
pedestrian: nightly 12 cell-seeds) move from r to r2 exactly as from a to a2.

**Every label that moved since the draft** (48abc9a: the labels of 7
November, derived on the Copter-4.5.7 fit at 9cfbac0), and why:

No label moved because the code moved. Checked four ways: the rail derivation
of 7 November (the rig's own fit, kept from that day's work) and today's give
the same label for every cell-seed on both readings (smoke 56, nightly 672);
the smoke derived on the harness of 9eed35b and on the code that flies is
identical reading for reading (above); the ideal labels are those of 7
November; and the owner rule of 7 November (`untrusted_owner` "chain", which
the tool now follows) moves none (the Copter-fit derivations of that day with
and without it: 0 of 112 smoke and 1,344 nightly readings).

What moved is the vehicle the draft was derived on. 48abc9a used the
Copter-4.5.7 fit, entered mid-flight at the asked speed (`RESPONSE_ENTRY`
"cruise"); the adopted labels use the rig's own fit, entered from a hover as a
rail starts ("rest"). Smoke 2, nightly 12 cell-seeds, the same on both
readings:

| cells (nightly cell-seeds) | 48abc9a | adopted | why |
|---|---|---|---|
| corridor-drift-angles at 0 deg, 4 and 8 m/s (6) | expected (E5, 69.3 s) | not expected (0.0 s) | entered at 4 or 8 m/s along the corridor the Copter fit's request is over theta for the whole episode; entered from a hover it is never over theta |
| corridor-drift-out (3) | undecided (a spell of 29.3 s still running) | not expected (0.0 s) | the same |
| gps-dropout lane 23.6, window 6-13 s (3) | expected (E3) | not expected | E3 reads the frozen fix against the true distance to the fence: closest true approach 2.32 m on the Copter fit, 4.36 m on the rig's |

Everything else the draft moved stands: the other cells of its table are the
cells of the table above, with the same clause.

**What stays undecided on the rail vehicle** (r and r2 alike): none, once the
labels undecided at the scenario's length are read on the flight as long as
the rail flies. Those were 18 nightly cell-seeds (3 of the smoke's):
nfz-head-on, dyn-nfz-rotate-step, time-window-opens-on-sideways-pilot and
dyn-nfz-spawn-ahead 30 m at 2 m/s. Runs scored on such a label are named in
every table (`longer`), with the count without them beside it.

### 2026-11-12: a GPS denial is read as the rails fly it

No clause changed. What changed is the physics the rail labels of gps-dropout
are read on: the stressor as the rail realises it, which section 8 always
asked for ("derived on the vehicle that flies them") and which the
derivation had not done for this one stressor.

**The finding.** Until today every rail flight of gps-dropout the tool read
handed the Shield the FROZEN last fix, the headless model ("what a bridge that
forwards the last pose would do"). No rail does that: the hil rig sets SITL's
SIM_GPS_DISABLE and ArduPilot's EKF3 keeps estimating on its IMU, so the pose
the Shield is handed keeps moving (sweep_scenarios, the rail stressors). The
rig's own raw MAVLink (each bag's /uas1/mavlink_source) carries SITL's truth
(SIMSTATE) beside EKF3's estimate (GLOBAL_POSITION_INT): over the 30 GPS
denials the rig has flown (nt1111_550-573, rh03_039/040, sm1110_043/044,
sm1111_043/044) the estimate stayed within 1.47 m of the aircraft (envelope
0.37 m at 2 s, 0.77 m at 4 s, 1.35 m at 6 s), lagging along the track, and it
reset when the fix came back. On nt1111's 24 gps-dropout flights E3 - read on
that truth, tick by tick - holds on none: past the 1 m margin only in the last
second of a 7 s window, with the aircraft 8.4 m or more from nfz-square (lane
23.6, 2-9 s; at 4-11 s the error stayed at 0.94 m or less). The frozen model
expected E3 on lane 23.6 at 2-9 s and 4-11 s: nt1111_565-567 scored "missed"
for a fail-safe the physics never called for, and nt1111_568-570 "correct"
for one it did not need either.

**The reading.** On the rail vehicle a GPS denial hands the Shield the
autopilot's dead-reckoned estimate: the true position less
`RAIL_EKF_DRIFT_MPS` (0.23 m/s, the upper envelope of error / time since the
loss on those 30 flights) x the time since the loss, along the ground track at
the loss, with `fix_ok` False (sweep_scenarios GPS_DROPOUT_MODELS
"dead_reckoned", in RAIL_VEHICLE; the ideal vehicle keeps the frozen fix, so
the templates' labels do not move). On it the lane-23.6 cells fly as the rig
flew them: the Shield brakes and slides along the ring, closest 2.85-2.88 m
(the rig: 2.80-2.90 m). The labels do not hang on the rate: flown at 0,
0.35 and 0.5 m/s, E3 holds on no cell and every label is the same.

**Labels it moved** (`tools/failsafe_rubric.py --update-labels`, recorded in
the artefact's `built.updates` with its commands at 49c455c; nightly 24
cell-seeds, the smoke's 2 the same):

| cells (nightly cell-seeds) | r and r2 | why |
|---|---|---|
| gps-dropout lane 23.6, 2-9 s and 4-11 s (6) | expected (E3) -> not expected | the estimate is never wrong by more than the margin while the aircraft is within that error of the zone |
| gps-dropout lane 15, 2-9 s and 4-11 s (6) | expected (E3, E5) -> expected (E5) | the head-on planner still asks for an over-theta repair for 43.8-44.0 s on the prescribed flight |

Every other reading of every cell is as it was (lane 15 at 2-4 and 6-13 s,
lane 23.6 at 2-4 and 6-13 s keep their label; a and a2 are untouched).

**nt1111 re-scored** (22d434b's flights, the same 591 folders): (r2) 555/567 =
0.979 before and after. Before: 3 missed (565-567, lane 23.6 2-9 s), 3 false
(571-573, lane 23.6 6-13 s) and the 6 wind-beyond-authority misses at 12 m/s.
After: 6 false (568-573, lane 23.6 4-11 and 6-13 s) and the same 6 wind
misses. The six false triggers are one mechanism, the FSM's X8 on the rail:
the Shield dead-reckons on the commands it streamed and takes the difference
from the handed pose as that pose's error, plus a 0.5 m/s drift allowance
(shield.DR_DRIFT_MPS). On the rig the handed pose is EKF3's, wrong by
0.13-0.78 m when X8 fired (9.3-10.4 s), while the Shield's command
integration was 1.3-1.5 m off it at 4-11 s and 3.0-3.5 m at 6-13 s - the rig
flies the Brake's lateral repair and its release late - so with the allowance
X8 judged the pose "4.4-5.0 m off". Replayed on
the logged flights, X8 still fires on these cells on the 0.5 m/s allowance
alone (10.2 s at 4-11 s, 10.8 s at 6-13 s); at SITL's measured 0.23 m/s it
would not, by 0.01 m on the closest trajectory (at 0.24 m/s it fires at
13.2 s). The allowance is a judgement recorded with every X8, an eighth of
the speed cap, and a real IMU drifts faster than SITL's; choosing it for a
rail is a decision, not a label question, and nothing here changed it.

**The answer taken (2026-11-12, later the same day): not the constant,
the autopilot's own verdict.** The ROS 2 node now reads ArduPilot's
EKF_STATUS_REPORT. While the autopilot reports a valid absolute horizontal
position, the handed pose is certified: it is the estimate the autopilot
itself navigates on. X8 fires when the autopilot declares that position
lost, by its own EKF failsafe rule. The dead reckoning and its allowance
remain the law only where no such verdict is reported
(docs/DESIGN-escalation-fsm.md, "2026-11-12: X8 on the autopilot's own
estimator").

- No clause and no label changed.
- On the 30 rig denials EKF3 kept its position for 6.8-6.9 s and dropped it
  for at most 0.21 s before the GPS returned, so the verdict never went
  against the position.
- Replayed on all 731 rig flights, no flight takes X8 under it, including
  nt1111_568-573.
- The headless rail vehicle flies those cells as the rig flew their
  lane-23.6 neighbours (G1, G7, no fail-safe).
- Predicted (r2) for nt1111 with X9: 567/567. Not flown.

What it leaves to the policy: while the autopilot vouches for its estimate,
the estimate's error (up to 1.47 m on SITL) is covered by the margin alone.

### The KPI-grade campaigns, scored

`--rescore-rail` on the fetched run folders (demo/out/hil_smoke), the
re-scored verdict of each run against the artefact:

| campaign | flown at | Shield-on runs | (a) | (r) | (r2), adopted | left out |
|---|---|---|---|---|---|---|
| **sm1111, smoke** | **22d434b** | 47 (45 accepted) | 39/45 = 0.867 (0.738) | 43/45 = 0.956 (0.852) | **44/45 = 0.978 (0.884)** | 2 not accepted (gps-dropout lane 23.6 4-11 s, wind-beyond-authority 8 m/s: their stress was not confirmed at take-off) |
| rh03, smoke | fb96fe5 | 44 | 37/43 = 0.860 (0.727) | 41/43 = 0.953 (0.845) | 42/43 = 0.977 (0.879) | 1 censored (wind-beyond-authority at 12 m/s) |
| hs05, the nine rotate-spin cells | 9eed35b | 9 | 7/7 (0.646) | 7/7 (0.646) | 7/7 (0.646) | 2 censored (6 deg/s, seeds 1001 and 1002) |
| sm1110, smoke | 9eed35b | 47 | 40/45 = 0.889 (0.765) | 43/45 = 0.956 (0.852) | 44/45 = 0.978 (0.884) | 2 censored (corridor-drift-angles 8 m/s -60 deg, wind-within-authority 5 m/s: the hand-back churn) |
| ge1110, grant example | 9eed35b | 32 (of 36; 4 folders hold no flight log) | 32/32 (0.893) | 32/32 (0.893) | 32/32 (0.893) | none |

Wilson 95 % lower bound in brackets. Nulls under (r2): sm1111 always-trigger
0.489, never-trigger 0.511; rh03 always-trigger
0.512, never-trigger 0.488; sm1110 0.489 and 0.511; hs05 and ge1110 fly no
cell that expects a fail-safe (never-trigger 1.000), so their 7/7 and 32/32
say nothing a Shield with no fail-safe would not also score.

- **Scored on a label read at the rail's length** (nfz-head-on,
  dyn-nfz-rotate-step, time-window-opens-on-sideways-pilot): 3 runs in rh03
  and 3 in sm1110, all correct triggers. Without them (r) is 38/40 and (r2)
  39/40 on rh03 - the figures reported on 7 November - and 40/42 and 41/42 on
  sm1110.
- **Wrong under (r2):** rh03_017 (dyn-nfz-rotate-spin at 3 deg/s: X6 sent it
  home at 30.4 s while no clause holds; the same cell is a correct non-trigger
  in sm1110); sm1110_048 (wind-beyond-authority, lane 15 at 12 m/s: E2 expects
  a fail-safe, the Loiter was handed back at 14.3 s and none followed).
- **Wrong under (a) and not under (r2):** the three cells above in both
  smokes, rail-mission-stub-vla in both (section 7's cell: wrong under (r)
  too) and wind-within-authority at 5 m/s in rh03 - each a fail-safe the
  flight took that the ideal vehicle's labels do not expect.
- **sm1111** is the smoke at the commit that flies (22d434b). Its (a) is the
  campaign's own line, 39/45 = 0.867 with the same 5 false and 1 missed. Its
  two runs the campaign did not accept (sm1111_044, sm1111_047: no
  scenario_score.json, the Shield node read no simulator parameters at
  take-off) flew and triggered, but are not the cell's flights: they are left
  out of every reading by name (`not_accepted`), as the campaign leaves them
  out. Under (r2) its one wrong run is the same as sm1110's, sm1111_048
  (wind-beyond-authority at 12 m/s, missed: E2); with the three labels read at
  the rail's length left out it is 41/42.

### The draft of 7 November, as it was put to the decision

Its numbers were measured on the Copter-4.5.7 fit at 9cfbac0 and are kept
for the record; the tables above replace them. Where it says the hil rail
passes the scenario's max_s, that was true on 7 November: the rails have
flown max_s + `loiter_timeout_s` + 5 s since.

**What the hil rig showed.** shk03_001 (nfz-head-on, the Shield at 8bfe08f,
ArduCopter 4.5.7 SITL in GUIDED on the hil topology): the zone enters the
5 s forecast at 2.9 s and the Shield's repair is 0.27 m deep (G1, Brake).
The autopilot answers the setpoint late and the aircraft keeps advancing,
so the forecast's depth at the horizon's end grows to 2.20 m in 0.5 s, over
the 2.0 m lateral theta: G3 at 3.4 s, Loiter, still there when the run
ends at 30 s - censored. The headless ideal vehicle flies the same cell with
one G1 and no escalation, and the ideal vehicle is what every label of
sections 2-5 was derived on. Section 2 reads a label from "the scenario's
physics"; the response of the vehicle is part of that physics, and the
KPI-grade runs fly a vehicle that does not stop where the ideal one does.

**The reading as drafted.** Two label sets, each derived by this same rubric
on the vehicle its runs fly:

- headless (the ideal vehicle): the labels in experiments/templates today;
- KPI-grade (the hil rail, ArduCopter 4.5.7 SITL in GUIDED): every flight
  the rubric reads - section 3's flight without a fail-safe and section
  7's prescribed one - flown on the harness's measured ArduPilot response
  (`tools/failsafe_rubric.py --response ardupilot`, the sweep's
  `--response-arms ardupilot`: 0.08 s delay, 2.4 m/s^2, jerk 5.7 m/s^3,
  0.03 s airframe lag, fitted to the recorded Copter-4.5.7 flights; a held
  aircraft is the zero setpoint flown through that response).

And every run flies long enough to show its outcome (section 3): a run that
would end censored in Loiter, or a label left undecided, is flown max_s +
`loiter_timeout_s` + 5 s. The hil rail passes `--max-s` = the scenario's
max_s today, so a Loiter entered at 3.4 s with a 30 s timeout is censored
by construction.

**Labels it moves**, ideal -> ArduPilot (`tools/failsafe_rubric.py` on both
vehicles; smoke, library seed 0, nightly seeds 1001-1003): smoke 7, library
14, nightly 66 cell-seeds, every one from "not expected", none the other
way, and the same cells on both readings of E5:

| cells (nightly cell-seeds) | moves to | why |
|---|---|---|
| dyn-nfz-spawn-ahead 12 m at 2 m/s and 30 m at 4 and 8 m/s (27), corridor-drift-angles at 0 deg (6), gps-dropout on lane 15, windows 2-4 and 6-13 s (6), wind-gusts 2 m/s and wind-within-authority 5 m/s on lane 15 (3 + 3) | expected (E5, both readings) | braking the ideal vehicle finishes inside theta, the ArduPilot response does not: the request stays over theta for 32-69 s |
| corridor-drift-out, dyn-nfz-rotate-step, dyn-nfz-spawn-ahead 30 m at 2 m/s, nfz-head-on, time-window-opens-on-sideways-pilot (3 + 3 + 9 + 3 + 3) | undecided at the scenario's length; expected (E5, both readings) flown 35 s longer | exactly the cells whose ArduPilot runs end censored in Loiter |

Where sections 3 and 7 differ (rail-mission-stub-vla, standoff-approach-
angles 7 m pedestrian) they differ the same way on both vehicles.

**Measured.** Fail-safe trigger correctness of the FSM flown on the
ArduPilot response (headless, not KPI-grade; integrate/1107 at 9cfbac0 with
this branch's tool; episodes, as the sweep counts them), against (a) the
ideal-vehicle labels on section 3's reading (the adopted labels), (b) the
ArduPilot labels on section 3's reading, (c) the ArduPilot labels on
section 7's reading; Wilson 95 % lower bound in brackets:

| profile | censored in Loiter | (a) | (b) | (c) |
|---|---|---|---|---|
| smoke, scenario length | 9 | 49/54 = 0.907 (0.801) | 52/54 = 0.963 (0.875) | 53/54 = 0.981 (0.902) |
| smoke, flown longer | 0 | 49/63 = 0.778 (0.661) | 61/63 = 0.968 (0.891) | 62/63 = 0.984 (0.915) |
| library, scenario length | 10 | 154/169 = 0.911 (0.859) | 163/169 = 0.964 (0.925) | 165/169 = 0.976 (0.941) |
| library, flown longer | 0 | 154/179 = 0.860 (0.802) | 173/179 = 0.966 (0.929) | 175/179 = 0.978 (0.944) |
| nightly, scenario length | 36 | 636/705 = 0.902 (0.878) | 681/705 = 0.966 (0.950) | 693/705 = 0.983 (0.970) |
| nightly, flown longer | 0 | 636/741 = 0.858 (0.831) | 717/741 = 0.968 (0.952) | 729/741 = 0.984 (0.972) |

"Flown longer": only the runs censored in Loiter (and the labels undecided
with them), 35 s longer; every one of them resolves - the ArduPilot labels
become E5 on both readings and the FSM's operator timeout returns the
aircraft (G5; time-window-opens-on-sideways-pilot then lands, G6). The
ideal labels with section 7's reading (section 7's own proposal, on the
ideal vehicle) give 50/54, 156/169 and 648/705 at the scenario's length.

**What is left under (c)**, all of it FSM behaviour on the rail vehicle,
none of it a label question:

- dyn-nfz-rotate-spin at 3, 6 and 9 deg/s (nightly 9 false): X6 sends the
  aircraft home - the rotating zone keeps a stop illegal and the slower
  vehicle does not get out within T - while no E clause holds (N4: the
  zone is imposed and can be left within the caps).
- corridor-drift-angles at 8 m/s, -30 deg (nightly 3 missed): E5 holds on
  both vehicles, but on the ArduPilot response the FSM's own Brake keeps
  the request within theta and it never escalates past G1.

**The hil runs, scored under (c)** (a sanity check, not a measurement):
shk03_001 ends censored at 30 s and its label is undecided at 30 s too;
flown to 65 s the cell is E5 and the headless ArduPilot run shows the edges
the rig flew, earlier by the rig's take-off (G1 at 1.6 s, G3 at 2.1 s, then
G5 at 32.1 s, home at 35.5 s) - a correct trigger under (b) and (c), a
false one under (a). shk01_001 predates the float32 cap-slack fix (8bfe08f):
its G1 at 0.3 s was the kinematic cap's own spurious violation and its
Loiter came from N-in-T on those onsets (G2 at 2.9 s), so it is evidence
of the bug, not of either label set.

## 9. Two decisions of 2026-11-15: clause E6, and the GeoFence backstop as a fail-safe

**Status: decided 2026-11-15 by the lab under the PI's instruction of 6 Oct
(follow the grant; best practice where it is silent); the PI's confirmation
is still to be recorded.** These were the two questions the final report had
left open for the PI (docs/FINAL-REPORT.md, PI decisions 13 and 14). Three
things a reader must know about them, as for sections 7 and 8:

- **Neither is a pre-registered reading.** Each was decided AFTER the flights
  it bears on: hs10_003 (14 November) for the first, the KPI nightly ntFIN2
  (13 November) for the second. The grant's text is the basis of both; the
  flights are what raised the questions.
- **The readings before them are still printed.** A cell labelled by E6 says
  so (`failsafe_clause`), and the tool prints E6's facts for every cell. Every
  figure that counts a fail-safe triggered by the backstop prints the count
  and, beside it, the figure with those runs read as before the rule and with
  them counted as not triggered.
- **What changed is named below**, label by label and field by field.

### 9.1 Decision 13: the grant's G4 governs a thrash of onsets the world makes (E6)

**The question.** fsm-n-in-t-flicker (experiments/templates/
failsafe_escalation.yaml, flown by experiments/profiles/fsm_edges.yaml alone)
takes the grant's N-in-T edges on purpose: a hard `brake` zone is hot-applied
ahead of the aircraft and switched off and on again every 1.4 s, the planner
asking to fly into it throughout. Each switch-on is an onset; three in 5 s
take G2 (Brake -> Loiter), three more inside the Loiter take G4 (Loiter ->
RTL, "N-in-T persists"), and the aircraft goes home. Every one of those
requests is answered by the rule's own trusted stop, so under N1 no fail-safe
is expected - and the grant's FSM text sends the aircraft home all the same.
The cell carried no fail-safe label, on purpose, from 14 to 15 November.

**The decision.** The grant's G4 governs: p4 draws "Loiter -> RTL: N-in-T
persists" as an edge of its own, beside the operator timeout, and a rubric
under which that edge can only ever be a false trigger contradicts the text
it scores (the argument section 2 already makes for E5). It enters the rubric
as a new, narrow clause, **E6 "N-in-T persists (the grant's G4)"** (section 2,
the table): hard-violation onsets made by the world, recurring at or above
the N-in-T rate through a Loiter.

**Why narrow.** N-in-T on its own stays a hold (N5), and the two ways the
library had reached G4 before stay "not expected": noise in the Shield's view
of the position (N2; since 2026-11-06 such a return is not even a new onset)
and a forecast re-touching a margin ring by a planner that is otherwise
repairable (N1). E6 counts only onsets that start on the tick a world event
puts the violated rule in force or changes it; section 2 states the boundary.
A planner that thrashes its OWN request in and out of a violation is still
not judged: no scenario has one (section 6).

**How it is read** (section 3, "Mechanically"; `tools/failsafe_rubric.py`
`e6_facts`): from the Shield's decision on the request, tick by tick, on the
flight without a fail-safe, and from the episode's own event record. N, T,
T_recover and the onset rate limit are the FSM's configured values, as E5
reads its timeout; nothing reads the FSM's state, edges or counters.

**The label it changed** (one):

| scenario | was -> now | clause | the fact, on the flight without a fail-safe |
|---|---|---|---|
| fsm-n-in-t-flicker | unlabelled -> **true** | E6 | six world-made onsets at 3.0, 4.4, 5.8, 7.2, 8.6 and 10.0 s (2 x N = 6 needed), each on a switch-on of flicker-zone, every three within 2.8 s, the request clear for 0.8 s at most between them; no E5 spell (0.0 s) |

No outcome label was added. No other label changed: every cell of the
library and of every profile (library 180 episodes, smoke 64, nightly 744,
grant_example 36, fsm_edges 5, broad 14,950) was dumped with its label, its
clause and its outcome label before and after the change, and the two dumps
differ in that one cell.

**The tool, after the change** (`tools/failsafe_rubric.py` at f08f16d; the
ideal vehicle unless said): library 168 cells (seed 0), smoke 56, nightly
672 cell-seeds (seeds 1001-1003; flown as ten `--only` parts that together
are the profile), grant_example 36, and fsm_edges 5 on the ideal and on the
rail vehicle: 0 declared labels disagree with the rubric, and none is
undecided in its length. E6 holds on fsm-n-in-t-flicker, on both vehicles, and on no other
cell: no other cell has more than 1 world-made onset (a single spawn, window
or swap) against the 6 needed, and none has more than 2 world events at all.
The rail label artefact (docs/data/failsafe_labels_rail.json) holds the
smoke, grant_example and nightly cells, which E6 does not touch (a world
event falls when the scenario says, on either vehicle), and was not
rebuilt. It holds no fsm_edges cell, so `--rescore-rail` names an fsm_edges
run "unlabelled" under (a), (r) and (r2); such a run's fail-safe verdict is
the campaign's own, on the template's label.

**The label followed the flight.** hs10_003 flew the cell on the hil rig on
14 November (8a473fa): X1 at 3.03 s, G2 at 5.83 s ("N-in-T: 3 onsets in
5 s"), G4 at 10.03 s ("N-in-T persists"), home at 18.83 s (G9). Re-scored on
the label it now has, it is a correct trigger; when it flew it had none. It
is not in a KPI figure either way: the fsm_edges profile is
`grant_kpi_bearing: false`, and no smoke, nightly or broad run flies the
cell.

### 9.2 Decision 14: a GeoFence RTL / LAND while the FSM holds is a fail-safe triggered by the backstop

**The question.** In ntFIN2, 19 KPI-grade Shield-on runs ended "censored in
Loiter" and were left out of the adopted reading (r2). One of them,
ntFIN2_374 (corridor-swap-shifted), flew its whole cap and ended in a Loiter
it had re-entered: section 3's censoring, which a longer flight can decide.
The other 18 are dyn-nfz-spawn-ahead cells (a zone hot-applied 3 m ahead at
4 and 8 m/s; three variants, three seeds): the FSM entered Loiter at
5.0-5.2 s (X1 or G1, then G3), and 0.6-0.8 s later ArduPilot's own GeoFence
took the aircraft, GUIDED to RTL at 5.67-5.80 s, with its "Fence Breached"
text 1-18 ms after the mode change. The Shield node stops stepping its FSM
after a takeover, so the Loiter was frozen 0.5-0.7 s into its 30 s timeout,
and no flight length decides such a run. The KPI was therefore stated three
ways (CHANGELOG, 2026-11-14): those 18 counted as triggered, counted as
missed, or left out as censored.

**The decision.** The autopilot's GeoFence is the grant's backstop (Grant
overview, the WP3 row: "ArduPilot fence / RTL / Land / Loiter as ultimate
backstop"), and the KPI's own words - "triggered when expected, not when not
expected", with the outcome vocabulary `RTL_triggered | Land_triggered` - do
not say by whom. So a run in which the autopilot took the aircraft to RTL or
LAND on a fence breach while the FSM was in Brake or Loiter **is a fail-safe
that triggered**: by the backstop, not by the FSM. It is scored against the
scenario's label like any other trigger - correct where a fail-safe is
expected, a false trigger where it is not.

**The rule, from each run's own records** (`experiments/sweep_scenarios.py`
`failsafe_backstop`, `BACKSTOP_RULE`; never a list of runs). All three must
hold:

- metrics.json `autopilot_took_over`: the autopilot entered a mode the node
  had not asked for, and that mode is RTL or LAND;
- events.jsonl: the autopilot's own STATUSTEXT naming a fence breach ("Fence
  Breached") within 2 s of that mode change (MAVROS relays the mode with the
  1 Hz heartbeat and the text as it is sent, so either may arrive first);
- metrics.json `fsm`: the FSM had triggered no fail-safe of its own and was
  in Brake or Loiter when it stopped stepping.

The run's score then says `failsafe_by: "autopilot_geofence"` (an FSM
trigger says `"fsm"`), with the takeover it was derived from
(`failsafe_backstop`: the mode, the time, the FSM's state, the breach text),
`failsafe_triggered` true and `failsafe_censored` false.

**What is not counted, and stays what it was.** A takeover into any other
mode (LOITER, BRAKE, SMART_RTL, a pilot's mode change), one with no fence
breach on record (a run whose events.jsonl was not read included), and one
with the FSM in Normal: each keeps the verdict it had - censored where the
FSM held in Loiter with its timeout pending, named with its cause - and its
record says why it does not count (`failsafe_backstop.why_not`). ntFIN2_374
had no takeover and stays censored.

**What is unchanged.** How a fail-safe the FSM triggers is scored. Section
3's censoring of a flight that ends still holding. Brake and Loiter as holds.
The labels: no clause and no label moved with this decision. The outcome
label (`mission_outcome`): still the FSM's `RTL_triggered` / `Land_triggered`
or fail / success, so a backstop run's outcome is what it was (the 18 are
`fail`); whether the grant's outcome vocabulary should name a backstop RTL is
not decided here. And every measure of what the autopilot flew: a fence
breach means the aircraft crossed a fence the Shield's hold did not keep it
out of (the 18 runs spent 3.4-3.6 s inside the hot-applied zone,
metrics.json `nfz_by_zone_s`), which the P0 escape rate and the autopilot's
keep-out entries report in their own rows. This decision makes the trigger
correct; it does not make the breach go away.

**Where it is printed.** Wherever the fail-safe KPI is: a sweep's summary
line and `_failsafe_labels` (`triggered_by`, `backstop_triggered_episodes`,
`before_backstop_rule`, `backstop_as_not_triggered`); `--rescore-rail`, under
each of (a), (r) and (r2), with every backstop run named; and
tools/kpi_report.py - the KPI summary's row and the readings table. A
re-score table written before the rule carries no `backstop_rule`:
kpi_report prints "not recorded" for the count, never 0, and refuses to merge
such a table with one written after.

**Applied to ntFIN2 and its supplement ntFIN2s** (docs/data/
kpi_ntFIN2_20261115/, generated at f08f16d from the run folders flown on
12-13 November; nothing was re-flown, and docs/data/kpi_ntFIN2/ keeps the
report of 13 November as it was). The commands are the ones of 13 November:
`experiments/sweep_scenarios.py --profile nightly --rescore-rail` over each
campaign's run folders, then `tools/kpi_report.py --campaign <ntFIN2>
--campaign <ntFIN2s> --rescore-rail <table> --rescore-rail <table>`. Over
the 586 KPI-grade Shield-on flights, Wilson 95 % lower bound in brackets:

| reading | the backstop's triggers counted (since 2026-11-15) | those 18 runs as before the rule: left out as censored | those 18 runs counted as not triggered |
|---|---|---|---|
| (a) | 513/585 = 0.877 (0.848) | 495/567 = 0.873 (0.843) | 495/585 = 0.846 (0.815) |
| (r) | 572/585 = 0.978 (0.962) | 554/567 = 0.977 (0.961) | 554/585 = 0.947 (0.926) |
| (r2), adopted | **584/585 = 0.998 (0.990)** | 566/567 = 0.998 (0.990) | 566/585 = 0.968 (0.950) |

- **The KPI under the adopted reading is 584/585 = 0.998291**, Wilson 95 %
  lower bound 0.990382 (before the rule 0.998236 and 0.990078; with the 18
  counted as not triggered 0.967521 and 0.949832). Nulls: always-trigger
  0.513, never-trigger 0.487.
- **Triggered by the backstop: 18**, all dyn-nfz-spawn-ahead, a zone
  hot-applied 3 m ahead: 9 at 4 m/s (ntFIN2_178-186) and 9 at 8 m/s
  (ntFIN2_214-222), three variants by three seeds each. Every one: GUIDED to
  RTL at 5.67-5.80 s, the FSM in Loiter, "Fence Breached" 1-18 ms after the
  mode change. All 18 are labelled expected (E5) under (a), (r) and (r2):
  18 correct, 0 false.
- **Still left out: 1**, ntFIN2_374 (corridor-swap-shifted, seed 1002): no
  takeover, the flight ended still holding. Censored, as before.
- **Still wrong: 1**, ntFIN2_177 (dyn-nfz-spawn-ahead 30 m ahead at 2 m/s,
  seed 1003; E5): a missed trigger, as on 13 November.
- **Nothing else moved.** Against the tables of 13 November
  (docs/data/kpi_ntFIN2/rescore_r2_*.json), compared run by run over the 583
  ntFIN2 and 15 ntFIN2s folders, the re-score changed `failsafe_triggered`,
  `failsafe_censored` and `failsafe_matches_label` on those 18 runs and
  took the "censored, not measured" fail-safe gate off their lists of failed
  gates; each still fails on the gates it failed before
  (`time_to_safe_episodes`, `autopilot_keepout_breaches` = 1: the autopilot
  flew into the zone). Beyond the fields added since 13 November
  (`rail_window.autopilot_took_over` on 14 November; `failsafe_by`,
  `failsafe_backstop` and `rail_window.takeover_fence_breach` now), no other
  field of any run's score and no other run's verdict differs. Three
  Shield-off control runs (ntFIN2_004-006) were also taken by the GeoFence;
  they fly no FSM, are not counted by the rule, and a Shield-off run is
  outside this KPI anyway. No takeover in either campaign was into a mode
  other than RTL or lacked the fence-breach text.
- **Read with care**, as on 13 November: no flown cell uses its seed, so the
  18 are six cells flown three times each, and the Wilson bound treats
  repeats as independent. The bound is above 0.99 under the adopted reading
  with and without the rule, by 0.0004 and 0.0001.
