# Constrained VLA for ArduPilot — VLA Guardrail

Safety-constrained Vision-Language-Action stack for ArduPilot UAVs (ITRI ICL 115-year
subcontract, Feb–Nov 2026). This repository holds both the **design site** (`docs/`,
served with MkDocs) and the **implementation** of the four contractual components.

The design is the source of truth; see `docs/` (`mkdocs serve`) or the implementation
plan at the top of `02-implementation/`.

## Components

| Package | WP | Role |
|---|---|---|
| `packages/vlaguard-common` | — | Cross-cutting contracts: `policy_hash`, the 6-field determinism manifest, the 4-D action + body→NED boundary |
| `packages/policy-dsl` | WP1 | Authored YAML/JSON → validated, indexed, signed policy bundle (single IR contract) |
| `packages/safety-shield` | WP3 | ROS-free core: monitor → repair (lateral projection / altitude clamp) → escalate; audit log |
| `ros2_ws/` | WP3 | ROS 2 Humble nodes wrapping the core (`safety_shield_node`, `mavlink_adapter`) |
| `sim/` | — | Functional rail: Gazebo Harmonic + ArduPilot SITL + MAVROS 2 (dev topology compose) |
| `demo/` | — | In-house VLA stub + the offline mid-term acceptance demo |

`prefix-compiler` (WP2) and `stress-harness` (WP4) are Phase 2 / Phase 3 — see the
implementation plan.

## Quick start

```bash
make sync          # uv sync the workspace (Python 3.11)
make check         # ruff + mypy + pytest
make demo          # mid-term acceptance demo (offline, no ROS needed)
```

`make demo` runs the four mid-term gate steps end-to-end and asserts the hard KPI
(**P0 violation escape rate = 0**), the `policy_hash` audit match, and that lateral
projection fired.

## Status

Phase 0 (foundations) + Phase 1 (mid-term critical path) are implemented and tested.
The functional-rail docker-compose and ROS nodes are Phase-1 skeletons pending a ROS 2
Humble host; the Shield/Policy logic they wrap is fully exercised by `make demo` and
the unit suite today.
