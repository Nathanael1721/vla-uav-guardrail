# Perception on the KPI-grade rail — what it would take

**Date:** 2026-09-07
**Status:** scoping only. **No code written**, deliberately — this cannot fit
before 18 September and starting it would put the demo at risk.
**Purpose:** be able to answer *"when will HIL be done?"* with something better
than silence. That was the one question from the 2 September review with no
answer.

## The gap, stated exactly

Every tracking result this project has — detector rate, `frac_on_target`, the
instance lock, the follow demos, the pedestrians — was produced on **Project
AirSim**, and is therefore *not* KPI-grade. The contractual figures come from
**ArduPilot SITL over MAVROS 2**, which has no renderer and no camera; its runs
fly a stub pilot along waypoints.

So the project can say "the Shield never let an illegal action through, measured
on the topology the grant requires", and it can say "the tracker held the right
vehicle for 100 % of a flight". It cannot yet say both about the *same* flight.

## The obstacle is not plumbing. It is the gate.

The instinct is that this is a wiring job: pipe AirSim's imagery to a Guardrail
driven over MAVROS. The plumbing is real but ordinary. The hard part is that
**`build_manifest` refuses the combination on purpose**:

> `canonical-hil` is the ArduPilot SITL + MAVROS 2 topology and has no simulator
> scene file, but `scene_path=…` was given. A run with a Project AirSim scene is
> `projectairsim-single-host`, whatever evidence accompanies it.

That refusal was added for a good reason, recorded in the code: without it, our
own Project AirSim scene "passed straight through and got stamped
`canonical-hil`, which is the one label the grant reads as KPI-grade." The check
exists because the failure already happened.

A HIL bridge is, by construction, a run with **both** a scene and the canonical
topology. So it cannot be delivered without deciding what the gate should mean —
and that decision has to be made deliberately, in the open, not by loosening a
guard until the run passes.

**This is the single most important thing in this document.** The engineering is
tractable; the honesty question is the one that needs a considered answer, and
probably the PI's agreement.

## Two ways to satisfy it, and what each really claims

**A — a fourth topology.** Add something like `hil-airsim-mavros`, sitting
between the two existing labels: real MAVLink and real MAVROS, with imagery from
a renderer. `is_kpi_grade()` then decides explicitly whether that label is
grade-eligible for perception KPIs while remaining ineligible for flight-dynamics
ones.

Honest, and it does not touch the existing refusal. The cost is that the grant
names a canonical topology and this is not it, so it needs the PI's agreement
that perception evidence on this rail counts.

**B — sensor injection, so it genuinely is the canonical rail.** Feed AirSim's
camera and pose into ArduPilot as `HIL_GPS` / `HIL_SENSOR`, so ArduPilot really
is the flight controller and MAVROS really is the bridge; AirSim degrades from
"the simulator" to "the sensor source and the renderer".

Closer to what the grant describes, and it would make the existing refusal
wrong rather than inconvenient — the scene file would no longer indicate a
different stack. It is also markedly more work, and it puts ArduPilot's EKF in
the loop with synthetic sensors, which is its own tuning problem.

**Leaning:** A first, because it is deliverable and honest, with B as the
follow-on if the funder wants the stronger claim. But this is a decision to put
to Prof. Lai, not one to take quietly in a commit.

## What the plumbing needs, in either case

| Piece | State today |
|---|---|
| Guardrail driven over MAVROS 2 | **exists** — `sitl/ros2_shield_node.py`, canonical-hil, KPI-grade |
| Pose into the Shield | exists, `LOCAL_POSITION_NED` at 10 Hz |
| Action out to the autopilot | exists, `SET_POSITION_TARGET_LOCAL_NED` at 10 Hz |
| Camera frames from AirSim | exists, but only inside `demo/follow_vlm.py`'s own loop |
| Detector + lock + estimator | exists, same place |
| **The two joined in one process** | **does not exist** |
| **A topology label the joint run may honestly wear** | **does not exist** |

The perception stack and the MAVROS stack are each complete. Neither has ever
been asked to run inside the other's loop, and that is the actual work: one
process that reads AirSim frames, runs the detector, drives the Shield, and
speaks MAVROS — while ArduPilot, not AirSim, flies the aircraft.

## The risks worth naming before starting

**Two simulators, one machine.** AirSim's renderer and ArduPilot SITL would run
together with the detector. The camera rail already fails its own rate gate —
loop 7.45–8.03 Hz against 9.5, detector 3.68–5.15 against 4.0, **zero of six
runs** — and this adds load rather than removing it. There is a real chance the
joint rail cannot hold 10 Hz, in which case the honest outcome is a measured
"not at this frame rate" rather than a quietly relaxed gate.

**Coordinate frames.** AirSim's pose and ArduPilot's `LOCAL_POSITION_NED` are
both NED but not necessarily the same origin or yaw reference. A silent offset
here would look like a tracking error and would be debugged in the wrong place
for days.

**`sim_speedup`.** The canonical rail reads it from the autopilot; the AirSim
rail reads it from the scene file. A joint run has both, and they can disagree.

## Estimate, with its uncertainty stated

Roughly **two to three weeks** of working time for option A, assuming the topology
decision is settled first and the frame alignment behaves.

That number carries real uncertainty, and the honest form is: the joint-process
work is perhaps a week and is well understood; the rate question could be a day
or could be the whole thing, because it may end in "this configuration cannot do
it" and a back-off study. Option B adds one to two weeks for HIL sensor injection
and EKF tuning, with wider error bars.

**Not startable before 18 September**, and not worth starting in the fortnight
after the demo either if the topology decision is still open — the code would be
written against a gate nobody has agreed on.

## What to say if asked at the meeting

> "It is the largest remaining item and I have scoped it rather than started it.
> The plumbing is about a week; the part that needs your decision is what
> topology label a run with both a renderer and MAVROS may carry, because the
> manifest currently refuses that combination on purpose — it was how a Project
> AirSim run once got stamped KPI-grade. Two to three weeks once that is settled,
> and I would rather agree the label with you than pick one myself."
