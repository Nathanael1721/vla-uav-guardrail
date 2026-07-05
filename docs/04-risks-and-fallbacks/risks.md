# Risks

Five tracked risks. Each has a written mitigation; if mitigation fails the corresponding fallback in [Fallback gates](fallback-gates.md) fires.

## R1 — Project AirSim HIL bridge slip

> **Probability:** medium · **Impact:** high · **Owner:** ArduPilot/MAVLink students

The Project AirSim ↔ ArduPilot bridge has to ingest `HIL_GPS` and `HIL_SENSOR` from AirSim into AP SITL, and route AP's actuator PWM back into AirSim's actuator model. This is not upstream code — the team builds it.

**Why it matters.** The perception rail cannot run KPI scenarios end-to-end until the bridge is closed-loop. Every perception-robustness number depends on it.

**Mitigation.**

- Treat the bridge as **Q1 critical-path**. No other Q1 task may starve it of attention.
- Build it incrementally: start with HIL ingest only (one-way, AirSim → AP), validate position tracking; then add actuator return.
- Stand up the [2026-06-30 fallback gate](fallback-gates.md): if the bridge is not closed-loop by that date, swap the perception rail to Colosseum.

---

## R2 — Project AirSim license / academic-use review

> **Probability:** low · **Impact:** medium · **Owner:** PI

Project AirSim is closed-source under a Microsoft commercial license. The terms must be reviewed for academic-use compatibility, especially for derivative works and redistribution of recorded scenes.

**Why it matters.** If the license is incompatible with NTUT publication or ITRI deliverable distribution, the perception rail must swap to Colosseum (MIT-licensed, no review burden).

**Mitigation.**

- Read the Project AirSim license before committing engineering effort beyond the HIL bridge spike.
- Confirm with NTUT legal / ITRI contracts office.
- Colosseum is the no-license-risk fallback.

---

## R3 — Desktop GPU procurement

> **Probability:** low · **Impact:** medium · **Owner:** PI / lab admin

Both Unreal-based perception rail candidates (Project AirSim, Colosseum) need a discrete RTX-class GPU on the desktop host. Isaac Sim + Pegasus would push the requirement higher (RTX 4090 ideal).

**Why it matters.** No GPU → no perception rail → perception-robustness scope narrows.

**Mitigation.**

- Confirm GPU availability before stress-testing ramp (Q3 start, 2026-08-01).
- If procurement slips, narrow perception scope and document the waiver in the final report. The functional rail still runs all KPIs.

---

## R4 — Pegasus + Isaac Sim is the road not taken

> **Probability:** decision risk, not failure risk · **Impact:** strategic · **Owner:** PI

The 2026-04-27 lock chose Project AirSim for the perception rail. NVIDIA Isaac Sim + Pegasus Simulator was not surfaced for explicit consideration. The case for it is strong: NVIDIA-vertical alignment with the chosen Jetson Orin inference hardware, USD scene format, and Isaac Lab's domain-randomization tooling.

**Why it matters.** A late switch from Project AirSim to Pegasus mid-project would cost weeks. A spike in Q1 (before Project AirSim engineering effort piles up) is cheap.

**Mitigation.**

- One-day spike in Q1: stand up Pegasus + Isaac Sim with the existing AP SITL, measure bring-up cost vs Project AirSim's HIL bridge.
- Decision recorded in [Recommendation](../05-recommendation/recommendation.md).

---

## R5 — Gazebo Harmonic has no realistic substitute on the functional axis

> **Probability:** very low · **Impact:** very high · **Owner:** PI

The functional rail is the load-bearing rail for Policy DSL, Prefix Compiler, Safety Shield, and the functional regression set. If Gazebo Harmonic itself becomes unworkable (upstream regression, plugin breakage, etc.), there is no clean drop-in.

**Why it matters.** The functional rail being broken is a project-existential risk.

**Mitigation.**

- Pin Gazebo Harmonic to a tested LTS minor.
- Pin `ardupilot_gazebo` plugin to a known-good commit; mirror it in the project repo.
- Snapshot the working dev container so a regression in upstream Gazebo can be rolled back without losing a week of debugging.

This is the rail the project must protect at all costs. No swap is on the menu — only defensive pinning.

---

## R6 — Policy DSL taxonomy scope creep

> **Probability:** medium · **Impact:** medium · **Owner:** Policy DSL team

The constraint taxonomy in the [Policy DSL](../02-implementation/policy-dsl.md) page enumerates spatial / kinematic / mission / temporal / behavioural classes. Each class can grow indefinitely as new operational scenarios appear (e.g. new sensor noise envelopes, new airspace rules). Without a freeze, the IR shape changes throughout the grant and every downstream consumer (Prefix Compiler, Safety Shield) keeps chasing it.

**Why it matters.** A moving IR breaks the single-source-of-truth invariant. Reproducibility of KPI numbers depends on the IR being stable across a release.

**Mitigation.**

- Freeze the IR shape at the end of Q1 (2026-04-30 cutoff). Additions after that go into a v0.4.x branch, not a v0.3.x patch.
- Every IR-shape change carries a semver bump and a migration note in the changelog.

---

## R7 — CSP token-budget overflow on 7B VLA

> **Probability:** medium · **Impact:** medium · **Owner:** VLA / Prefix Compiler team

Modern 7B-class VLA backends (OpenVLA family, BitVLA, etc.) have a finite prompt-token budget. The CSP must fit *and* leave room for the natural-language task prompt. If the policy bundle is large, the Prefix Compiler's truncation rule (P0 always retained, P1/P2 summarised) may not be enough.

**Why it matters.** A truncated CSP that drops a P0 rule violates the "P0 coverage = 100%" acceptance criterion.

**Mitigation.**

- Pre-flight check at CSP emit time: if P0-only summary already exceeds the budget, fail loudly (do not silently truncate P0).
- Treat budget overflow as a Policy DSL authoring problem (too many P0 rules in scope) and lint for it during ingest.
- Fallback path: emit a "structured-only" CSP (skip the natural-language summary) for missions with high P0 density.

---

## R8 — Repair operator non-convergence under simultaneous violations

> **Probability:** low · **Impact:** high · **Owner:** Safety Shield team

The Safety Shield's repair stack tries altitude clamp, then lateral projection, then path repair. When multiple violations fire simultaneously (e.g. envelope + corridor + time window), repairs can interact: fixing the altitude violation may push the point into a corridor edge, fixing the corridor edge may push it back outside the envelope.

**Why it matters.** Non-convergent repair leads to thrashing and eventually fail-safe escalation — measured as a repair-success-rate degradation and an inflated mean-time-to-safe.

**Mitigation.**

- Bound the repair iteration count (e.g. ≤3 iterations); on non-convergence go straight to fail-safe.
- Add a regression scenario for "three simultaneous violations" to the [Safety Shield](../02-implementation/safety-shield.md) test matrix.
- Log every repair attempt (not just the final emitted action) so non-convergent cases are visible in the audit log.

---

## R9 — RPC harness throughput vs nightly-window budget

> **Probability:** medium · **Impact:** low · **Owner:** Stress Testing team

The grant scope calls for hundreds-to-thousands of parameter sets per nightly stress run. With `sim_speedup=1.0` mandatory in the HIL topology, episode wall-clock dominates throughput. A single 2-minute episode × 3000 sets × 1 host = 100 hours — not nightly.

**Why it matters.** If the harness can't complete a nightly sweep in the wall-clock window, KPI freshness slips and regressions are discovered too late.

**Mitigation.**

- Run the **smoke set** (~50 scenarios) at `sim_speedup=1.0` nightly; run the **broad sweep** (thousands of param sets) on the functional rail with `sim_speedup>1.0` accepted (documented in the sweep manifest).
- Parallelise across multiple desktop hosts when available (each host runs an independent session).
- Triage discipline: nightly only reports *new* failures; regressions are diffed against the previous night's report.

---

## Summary table

| ID | Risk | Probability | Impact | Mitigation owner | Fallback |
|---|---|---|---|---|---|
| R1 | Project AirSim HIL bridge slip | medium | high | AP/MAVLink students | Swap to Colosseum at 2026-06-30 gate |
| R2 | Project AirSim license review | low | medium | PI | Swap to Colosseum |
| R3 | Desktop GPU procurement | low | medium | PI / lab admin | Narrow perception; waiver in report |
| R4 | Pegasus + Isaac Sim road-not-taken | decision | strategic | PI | One-day spike; record decision |
| R5 | Gazebo Harmonic regression | very low | very high | PI | Defensive pinning; container snapshot |
| R6 | Policy DSL taxonomy scope creep | medium | medium | Policy DSL team | Freeze IR shape end-Q1; semver discipline |
| R7 | CSP token-budget overflow on 7B VLA | medium | medium | VLA / Prefix Compiler team | Fail loudly on P0 overflow; structured-only fallback |
| R8 | Repair operator non-convergence | low | high | Safety Shield team | Bounded iteration; fail-safe on non-convergence |
| R9 | RPC harness throughput | medium | low | Stress Testing team | Smoke+broad split; parallel hosts; regression diff |
