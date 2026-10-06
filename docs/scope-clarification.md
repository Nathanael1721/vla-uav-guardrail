# Scope Clarification — What This Project Delivers (and what it does NOT)
NTUT AIoT Lab · Constrained VLA for ArduPilot · 2026-07-13, corrected 2026-10-06
One-page alignment note. Source of truth: the 7 grant design PDFs + Prof. Lai's
reference repo `kuanting-vla-uav-guardrail/`. Both agree.

> **Corrected 2026-10-06** against the grant audit of 5 October
> (`docs/AUDIT-KONTRAK-2026-10-05.md`). The July text said:
> - WP2 was "natural language → structured mission";
> - the KPIs were "P0 escape rate = 0, mean repair magnitude, mean time to safe";
> - `vla_policy_v2` was "WP4 evidence";
> - the project was "on/ahead of schedule";
> - Jetson hardware was "a stretch goal, not a gate".
>
> Each is corrected in place below, citing the grant page. The quote of the
> switchable-backend clause also dropped the words "for unit tests"; they are
> restored. The current list of open items is the audit, not this note.

## The project name, decoded
> *"**Semantic-Spatial Translation** and **Safety-Constrained VLA** for ArduPilot UAVs"*
- **WP2 = Prefix Constraint Compiler:** policy bundle + mission context → a
  Constraint Summary Pack (CSP) injected into the VLA / planner prompt (Grant
  overview p.1; Prefix Compiler p.1: the CSP is its only output). Parsing natural
  language into a mission is not part of the WP2 spec. The grant title's
  "Semantic-Spatial Translation" is not defined on any grant page as a work
  package.
- **Safety-Constrained VLA** = making a VLA obey hard limits. WP1 + WP2 + WP3.
- The word **"VLA" names the technology we CONSTRAIN — not a thing we build.**

## What IS the deliverable (contractual, WP1–WP4)
| WP | Output artefact |
|---|---|
| WP1 | Policy DSL → signed policy bundle (GeoFence, envelope, corridor, time windows) |
| WP2 | Prefix / Constraint Compiler → Constraint Summary Pack injected into the VLA prompt |
| WP3 | Safety Shield (ROS 2 node) → MAVLink to ArduPilot |
| WP4 | Stress-testing harness → scenario library, KPI rollups, replay harness |

Plus the **five acceptance KPIs** the grant lists (Grant overview p.2):
1. mission success rate;
2. P0 violation escape rate, target 0 (the only hard limit);
3. fail-safe trigger correctness, target ≥ 99 % (Stress Testing p.3, p.6);
4. mean repair magnitude;
5. mean time to safe.

The work packages own further KPIs: policy load round-trip and bundle
replayability (WP1); prefix-token budget and CSP coverage (WP2); per-paraphrase
robustness (WP4). Every reported KPI must come from a Stress Testing run in the
*hil* topology (Stress Testing p.1).

Final delivery, 2026-11-30 (Grant overview p.2):
- DSL, Prefix Compiler, Shield and Stress Testing complete;
- a KPI report on dynamic-scenario stress;
- perception-rail integration;
- a signed final report.

## What is NOT the deliverable
- ❌ Training / building a deployable VLA model.
- The grant states the VLA backend is **switchable** — *"(CognitiveDrone, OpenVLA
  generic, BitVLA, in-house stubs for unit tests, etc.); every backend must
  conform to this 4-D output shape, but the project does not commit to any one
  of them as a 'default'"* (Architecture constraints p.2-3).
  → the VLA is an **external, pluggable dependency**, supplied off-the-shelf.
  The clause is about VLA backends. It does not cover the detector, and it
  names stubs only for unit tests, not a hand-written pilot in the demos.
- The grant mentions VLA training **only as OPTIONAL**: the stress harness
  *"collect[s] labelled data for analysis and (optionally) VLA training."*
- Prof. Lai's reference repo ships a **VLA stub only** — no model, no training.
  That is the authoritative interpretation.

## Where our trained model (`vla_policy_v2`) fits
NOT "our VLA". It is an advanced stub that proves the swappable slot works
end-to-end. It is not WP4 evidence: WP4's outputs are a scenario library, KPI
rollups and a replay harness (Stress Testing p.6-7). The July measurement
(interventions 4.3 % → 0.04 %) is a result about a learned policy, not a
Stress Testing run. Present it as *"a learned flight policy"*, never as "our
VLA model".

## Corrected direction
1. Primary focus stays the **Guardrail** (WP1–4). In the 6 October plan
   built from the audit, two mid-term (2026-07-20) deliverables were still
   open: the IR as the grant defines it, with spatial indices and geometry
   caches (tracker cards WP1-03, WP1-21), and the Gazebo Harmonic functional
   rail, of which no run is kept (ARCH-16, ARCH-17). WP2 and WP4 are partial
   against the grant (see the audit). The reference repo leaves WP2 and WP4
   unbuilt, so "ahead of the reference" is not the same as "done against the
   grant".
2. To show a REAL VLA in a demo: **plug an existing one** (AeroVLA / CognitiveDrone)
   into the slot — do not train from scratch. This is exactly the "switchable
   backend" design. See `guardrail/vla_backends.py` (the July text pointed to
   `docs/vla-backend-howto.md`, which is not in the repository).
3. Highest-value next work: align our code to the reference contracts (WGS84
   frame, `vlaguard_common`), then fold our Compiler + harness into the trunk
   as contributions.

## Solo + agentic-AI + simulation
Most of the deliverable is software, KPIs and a report, and can be produced in
simulation. Two exceptions:
- **The final KPI report needs the *hil* topology** (Architecture constraints
  p.4): a Jetson Orin running the VLA + Shield, with the desktop running the
  simulators. That is a gate, not a stretch goal. It needs the Orin even without
  flying (tracker card ARCH-12).
- **The *flight* topology is named for the final demos.** The grant says all
  three topologies are first-class.

The grant staffs the project with 1 PI + 4 Master students (Grant overview
p.1); one developer is doing it. Whether the desktop rail may stand in for hil,
and whether the flight topology may be waived, are decisions for the PI, and
they have not been asked yet.
