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

- **N1** A planner whose requests the Shield repairs within theta, however
  long it persists and however often its forecast re-touches a margin ring.
  Brake is the grant's state for "violation, repairable, magnitude < theta"
  and has no timeout; p1 reserves the fail-safe for an unsafe repair.
- **N2** Position error within the margin: the margin was sized for it.
  N-in-T onsets that exist only in the Shield's noisy view of the position
  are not a planner fighting the Shield.
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
  parameter; E3 from the noise and latency parameters against the closest
  true approach to the boundary and, for a frozen fix, from the true-versus-
  seen error against the true distance to the boundary, tick by tick; E5
  from the measured spell of unsafe requests. `tools/failsafe_rubric.py`
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
  has no fix-quality field) still makes the scenario expect a fail-safe;
  where the FSM does not trigger, that is a missed trigger, reported with its
  mechanism.
- The theta basis (decided separately: each repair's forecast depth in
  metres, reading A of docs/DESIGN-escalation-fsm.md).
- The operator timeout (30 s, `loiter_timeout_s`): the rubric reads it as
  configured. Changing it changes which E5 cells complete inside an episode,
  and is not done to move the KPI.

## 5. Applied to the library (2026-11-02)

Filled in by the relabelling commit that follows this one; the clauses above
do not change there.

## 6. Open

- No scenario has a planner that thrashes by changing its own request (N5):
  N-in-T persisting (G4) is exercised only by the harness's position noise.
- E3's frozen fix is a ground-truth clause the Shield cannot observe: the
  headless harness models neither the rails' stale-pose watchdog nor
  ArduPilot's EKF failsafe, which would answer it on a flight.
- Nothing here is KPI-grade: the grant's KPI is measured on the hil topology.
