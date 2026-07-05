# Constrained VLA for ArduPilot — Implementation Plan & Simulation Framework

## TL;DR

> This is the working architecture document for the ITRI ICL 115-year subcontract on **Semantic-Spatial Translation and Safety-Constrained VLA for ArduPilot UAVs** (Feb 2026 – Nov 2026). The grant ships **four interlocking components**: a Policy DSL, a Prefix Compiler, a Safety Shield, and a Stress Testing harness. The **[Implementation plan](02-implementation/overview.md)** section captures how each component is being built — the detailed sub-design is being authored over the coming weeks; open questions are tracked there in-line. The **[Simulation framework](03-simulation/dual-rail.md)** section documents the dual-rail simulator decision (Gazebo Harmonic + Project AirSim, sharing one ArduPilot SITL) that hosts those components in test. Two contractual delivery dates anchor the schedule: mid-term **2026-07-20** and final **2026-11-30**.

## Reading guide

| If you are… | Read in this order |
|---|---|
| **Stakeholder / reviewer** (want the bottom line) | [Recommendation](05-recommendation/recommendation.md) → [Implementation overview](02-implementation/overview.md) |
| **Engineer / student** (joining the project) | [Grant overview](01-context/grant-overview.md) → [Architecture constraints](01-context/architecture-constraints.md) → [Implementation overview](02-implementation/overview.md) → the four implementation pages in order |
| **Auditor / second-opinion** (challenging the decisions) | [Layer requirements](03-simulation/layer-requirements.md) → [World/perception survey](03-simulation/world-perception.md) → [Comparison matrix](03-simulation/comparison-matrix.md) → [Risks](04-risks-and-fallbacks/risks.md) → [Fallback gates](04-risks-and-fallbacks/fallback-gates.md) |
| **Returning later for a specific topic** | Use the nav: Implementation plan / Simulation framework / Risks & fallbacks / Recommendation |

## Status

| Item | State |
|---|---|
| Implementation plan structure | In progress (4 components, sub-design being authored; open questions tracked per page) |
| Simulation dual-rail lock | Locked 2026-04-27 |
| HIL bridge adapter | **Q1 critical-path**; gate at 2026-06-30 |
| MAVROS 2 vs AP_DDS | MAVROS 2 first; AP_DDS migration is Q3 optional |
| VLA backend | Switchable across modern VLA backends; 4-D action contract `(vx, vy, vz, yaw_rate)` (no specific backend committed as default) |
| GCS | Mission Planner via `mavlink-router` fan-out |
| Deployment topologies | dev / hil / flight (see [Topologies](03-simulation/topologies.md)) |

## Section map

- **[Context](01-context/grant-overview.md)** — what the grant is, what it must deliver, what architectural constraints are locked.
- **[Implementation plan](02-implementation/overview.md)** — the four components and their sub-steps. *Primary content.*
- **[Simulation framework](03-simulation/dual-rail.md)** — where the components run during test; survey of simulator candidates and the dual-rail decision.
- **[Risks & fallbacks](04-risks-and-fallbacks/risks.md)** — tracked risks across both implementation and simulation; the 2026-06-30 perception-rail fallback gate.
- **[Recommendation](05-recommendation/recommendation.md)** — final position on what to build first and what simulator to host it on.
- **[References](references.md)** — grouped by topic.

## How to build this site

```bash
. .venv/bin/activate
mkdocs serve   # http://127.0.0.1:8000
```
