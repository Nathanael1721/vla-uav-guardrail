# Recommendation

This page consolidates the report's two recommendations: (1) the **implementation roadmap** — what to build first, what mid-term must demo, what final must demo; and (2) the **simulation framework** decision (the original v1 recommendation, kept intact).

---

## 1. Implementation roadmap

> **Build in this order:** Policy DSL skeleton → Safety Shield skeleton (parallel, both Q1) → Prefix Compiler (Q2) → Stress Testing harness (Q3) → final integration + KPI runs (Q4). Mid-term (2026-07-20) must demo Policy DSL + Safety Shield end-to-end on the functional rail with at least one projection operator firing on a scripted violation. Final (2026-11-30) must demo all four components, with a KPI report from a nightly stress run.

### Why this order

- **Policy DSL is upstream of everything.** The Prefix Compiler consumes its IR; the Safety Shield consumes its IR. Until the IR shape is stable, both downstream consumers are building against a moving target. Q1 is the right time to lock the IR shape (see [R6](../04-risks-and-fallbacks/risks.md#r6-policy-dsl-taxonomy-scope-creep)).
- **Safety Shield can start in parallel.** The skeleton (one violation check + one repair operator + one MAVLink emit) only needs the *shape* of the IR, not its full content. Starting Q1 means the Q3 stress sweeps already have a hardened Shield to push against.
- **Prefix Compiler waits for Q2.** It needs the Policy DSL ingest pipeline emitting bundles and the Safety Shield emitting repair logs (the Prefix's effectiveness eval consumes both). Q2 is the earliest both inputs are real.
- **Stress Testing waits for Q3.** It needs all upstream components stable and a working RPC plane. Nightly stress runs in Q3 produce the data the final KPI report builds on.

### Mid-term (2026-07-20) demo gate

The mid-term delivery must show, on the functional rail (Gazebo Harmonic), end-to-end:

1. Load a signed policy bundle containing one polygon NFZ and one envelope rule.
2. Issue a natural-language task that *wants* to enter the NFZ.
3. Show the Safety Shield intercepting and applying lateral projection.
4. Show the audit log entry with `policy_hash` matching the loaded bundle.

If this demo doesn't run by 2026-07-15, the mid-term delivery is at risk — escalate.

### Final (2026-11-30) demo gate

The final delivery must show:

1. All four components in the implementation plan, exercised by a non-trivial scenario from the [Stress Testing](../02-implementation/stress-testing.md) scenario suite.
2. A KPI report from a nightly stress run, with the P0 violation escape rate at 0.
3. The same scenario replayed bit-for-bit from its episode bundle (proves the determinism contract holds).

### Suggested next deep-dives (after this report is approved)

Sequenced by criticality:

1. **Safety Shield ROS 2 node implementation** — the load-bearing safety component. Detailed design of projection operators on the 4-D action space, the escalation FSM, the audit-log schema. Ships first because it's the longest-pole task and the most exposed to integration risk.
2. **Project AirSim ↔ ArduPilot HIL bridge adapter** — Q1 critical-path; unblocks the perception rail. See [R1](../04-risks-and-fallbacks/risks.md) and the [2026-06-30 fallback gate](../04-risks-and-fallbacks/fallback-gates.md).
3. **HIL topology DDS layout** — Cyclone DDS partitioning, QoS profiles, host-pair networking, SROS 2 + mTLS. Operationally important once the Shield is on the Orin in Q2.
4. **Paraphraser service + Scenario YAML schema** — Q3 deliverable; required for the per-paraphrase robustness KPI rollup.

---

## 2. Simulation framework

> **Keep the 2026-04-27 dual-rail lock** — Gazebo Harmonic (functional rail) + Project AirSim (perception rail) + ArduPilot SITL (shared inner loop). **Add** the Colosseum fallback gate at 2026-06-30. **Run** a one-day Pegasus + Isaac Sim spike in Q1 before further investment, so the decision to keep Project AirSim is informed rather than inherited.

This is a conservative recommendation. It changes nothing operationally beyond adding a written fallback gate (already defensible project hygiene) and a one-day spike (cheap insurance against the largest unaddressed counterfactual).

### Why not change more

Three more aggressive options were considered and rejected:

1. **Demote Project AirSim to fallback, lead with Colosseum.** Lower risk on license / HIL-bridge axes. Rejected because Project AirSim is the more capable and better-supported codebase if the HIL bridge works; trying first costs less than the perceived risk premium.
2. **Reopen perception rail to Pegasus + Isaac Sim.** Strategic alignment with Orin hardware. Rejected as a Q1 *decision* — but kept as a Q1 *spike* so the data exists when the question is re-asked.
3. **Add VIVID as a third rail.** Institutional fit. Rejected because three rails triple bridge / docker maintenance cost. Kept as a demo overlay, not a rail.

### Decision summary

| Question | Recommendation |
|---|---|
| Functional rail | Gazebo Harmonic + `ardupilot_gazebo` (locked) |
| Perception rail | Project AirSim, with Colosseum as documented fallback at 2026-06-30 gate |
| Inner loop | ArduPilot SITL (required) |
| Safety bridge | MAVROS 2 first; AP_DDS migration optional Q3 |
| GCS | Mission Planner via `mavlink-router` fan-out |
| VLA backend | Switchable across modern VLA backends; 4-D action contract; no specific backend committed as default |
| Topologies | dev / hil / flight, all first-class |
| VIVID | Demo overlay only, not a rail |
| Pegasus + Isaac Sim | One-day Q1 spike; decision recorded after spike |

---

## Open decision points for the user

1. **Approve both recommendations as written.** Implementation roadmap proceeds; simulation lock unchanged. Then proceed to the Safety Shield deep-dive (#1 in the next-deep-dives list above).
2. **Fund the Pegasus spike now.** Run the one-day Pegasus + Isaac Sim spike before further perception-rail engineering.
3. **Demote Project AirSim to fallback** preemptively, lead with Colosseum. Saves HIL-bridge engineering effort; loses Microsoft's feature velocity.
4. **Add VIVID as a third rail** — overrules the "two rails maximum" simplification.

Default: option 1.
