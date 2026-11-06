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
  the rubric's clauses did not change with it.
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

## 6. Open

- No scenario has a planner that thrashes by changing its own request (N5):
  N-in-T persisting (G4) is exercised only by the harness's position noise.
- E3's frozen fix is a ground-truth clause the Shield could not observe
  until 2026-11-06. It now can where a rail reports fix validity
  (`State.fix_ok`), and the headless harness models a rail that does
  (`stress.fix_quality_reported`, default true); no flight rail reports it
  yet (docs/DESIGN-escalation-fsm.md, "Where each rail's signal lives"),
  and ArduPilot's own EKF failsafe is still not modelled.
- Nothing here is KPI-grade: the grant's KPI is measured on the hil topology.

## 7. PROPOSED, NOT ADOPTED: E5 read on the flight the grant prescribes (draft, 2026-11-06)

**Status: a draft for the user's decision. It is not the rubric.** Sections
1-6 are the rubric; nothing in them changes unless this is adopted. This
section exists only on the branch `e5-prescribed-flight-proposal` (and on
`e5-prescribed-on-rail-vehicle`, which builds section 8 on it), which
also carries the tool, labels and tests that would go with it, so the
numbers below are measured, not estimated.

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

**The proposed reading.** E5 is read on the *prescribed* flight
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
end. What the proposal removes is every choice of this implementation:
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

**Measured on this branch** (labels as above, on branch 1's code at
c07998d; headless, not KPI-grade):

| profile | adopted rubric (branch 1) | this proposal | Wilson 95 % low |
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

**To adopt it**: merge this branch (the tool's default, the labels, the
pins, the tests) and fold this section into sections 2-5. **To reject it**:
discard the branch; sections 1-6 stand.

## 8. PROPOSED, NOT ADOPTED: the KPI-grade labels derived on the vehicle that flies them (draft, 2026-11-07)

**Status: a draft for the user's decision. It is not the rubric.** Sections
1-6 are the rubric. This section exists only on the branch
`e5-prescribed-on-rail-vehicle`, which also carries section 7's proposal
(its tool, labels and tests); the vehicle and the reading of E5 are two
separate decisions, and the numbers below give every combination.

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

**The proposed reading.** Two label sets, each derived by this same rubric
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

**To adopt it**: the KPI-grade label set is derived with `--response
ardupilot` (on section 3's reading, or on section 7's if that is adopted
too), the hil runs fly max_s + `loiter_timeout_s` + 5 s, and the headless
labels stay as they are. **To reject it**: discard the branch; sections 1-6
stand.
