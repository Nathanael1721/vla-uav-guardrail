# Study — Prof. Lai's reference repo `kuanting-vla-uav-guardrail/`
Studied 2026-07-12, corrected 2026-10-06. This is the AUTHORITATIVE
implementation of the grant. Our own code (`guardrail/`, `training/`, `demo/`,
`sitl/`) was built independently and converged on most of the same design. This
doc maps the two.

> **Corrected 2026-10-06** against the grant audit of 5 October
> (`docs/AUDIT-KONTRAK-2026-10-05.md`, card C-11). The July text said:
> - `make demo` "passes the mid-term gate (P0 escape = 0)";
> - our action was the same 4-D body-frame contract;
> - "4 rails run", including Gazebo;
> - the trained model was WP4 work;
> - WP2 was "NL → mission".
>
> Each is corrected in place below. In short: three rails have kept run
> evidence, and Gazebo has none (no run output is kept; July notes record a
> headless pymavlink run, the 2 Sept memo says "never run"). Our `Action4D`
> is world frame, not the grant's body frame. The trained model is a learned
> flight policy, not a WP4 output. The grant's
> mid-term gate is DSL + IR, a Shield ROS 2 node skeleton and a Gazebo
> functional rail, not P0 = 0.

## What the repo is
Professional monorepo (uv workspace, Python 3.11, ruff+mypy+pytest, MkDocs site).
29 unit tests pass. `make demo` runs `demo/mid_term_demo.py`: an offline,
pure-Python demo that asserts P0 escape = 0 and calls exit code 0 "the mid-term
gate". The grant's mid-term delivery (2026-07-20, Grant overview p.2) also names
a Shield ROS 2 node skeleton with at least one projection operator and a
Gazebo Harmonic functional rail demoable end-to-end. An offline demo does not
show those. The repo IS the 7 PDFs we studied on day 1, turned into working code.

Layout:
- `packages/vlaguard-common` — contracts: `Action4D` (frozen), `body_to_local_ned` (single boundary), `DeterminismManifest` (6 fields; HIL must be sim_speedup=1.0)
- `packages/policy-dsl` (WP1) — YAML → validated IR → signed bundle (tar.gz, policy_hash = SHA-256 over canonical IR)
- `packages/safety-shield` (WP3) — ROS-free core: checker → repair stack → FSM → audit
- `ros2_ws/` (WP3) — ROS 2 Humble nodes wrapping the core
- `sim/` — Gazebo Harmonic + SITL + MAVROS compose
- `demo/` — VLA stub + mid_term_demo + **projectairsim_demo** (UE5 perception rail) + sitl_flight_demo + ros_vla_stub
- WP2 (prefix-compiler) + WP4 (stress-harness) = Phase 2/3, not yet built

## Independent convergence (we found the same things)
| Insight | Prof repo | Our code |
|---|---|---|
| 4-D action, single body→NED boundary | `vlaguard_common.frames` (body frame, as the grant locks) | **Not converged.** Our `Action4D` is world frame (`guardrail/models.py:41-43`: vx North, vy East). `guardrail/frames.py` converts exactly to and from body frame, but only the tests call it (tracker card ARCH-04). |
| policy_hash reproducibility | signed tar.gz bundle | `Policy.policy_hash` |
| Bounded repair loop (3 iters) | `repair_action` | `Shield.filter` |
| LateralProjection = approaching, **GeofenceEscape = inside, exempt from magnitude cap** | `repair.py` (explicit `recovery=True`) | our "trend-aware + GeofenceEscape" (found via the deadlock bug) |
| "brake while inside = deadlock" | documented in `GeofenceEscape` docstring | our finding-report |
| VLA stub = reckless straight-at-target | `demo/vla_stub.py` | `guardrail/vla_stub.py` |
| Project AirSim UE5 perception rail | `demo/projectairsim_demo.py` | our `docs/projectairsim-setup.md` (installed + flew hello_drone) |
| Gazebo functional rail + SITL + MAVROS | `sim/` | our `sitl/`: ArduPilot SITL over pymavlink and over MAVROS 2 both ran and kept their output. The Gazebo launch scripts (`sitl/setup_gazebo.sh`, `sitl/run_gazebo_demo.sh`) exist, but no run output is kept (no `demo/out/gazebo_*`), so the mid-term item cannot be evidenced. July notes record a headless run over pymavlink; nothing has gone through MAVROS 2. |

**Two implementations, same architecture, mostly the same hard-won bug fixes —
strong mutual validation, except for the action frame above.**

## Where OURS goes beyond the reference (net-new, not in prof repo)
- **WP2 Prefix/Constraint Compiler**: `guardrail/compiler.py` builds a
  constraint summary pack (CSP) from the policy and the mission and renders a
  prompt from it. It also parses a command into a mission, which is not part of
  the WP2 spec. The grant's WP2 is policy bundle + mission context → CSP
  injected into the VLA prompt (Prefix Compiler p.1). Against that, ours was
  partial at the 5 Oct audit: no filter / risk-grade / truncate / token-budget
  pipeline, no CSP schema, and no flown VLA had read it. The prof repo lists
  WP2 as Phase 2 (unbuilt).
- **A learned flight policy (not WP4)**: our `training/` auto-research loop,
  gate metrics, and `models/vla_policy_v2.pt/.onnx`. It is an advanced stub for
  the switchable slot. The prof repo has no learned model at all (stub only).
  WP4's outputs are a scenario library, KPI rollups and a replay harness
  (Stress Testing p.6-7). Our WP4 work started later (September): a headless
  13-scenario regression sweep and replay bundles, which is not the grant's
  stress harness.
- **REST hot-apply API** (`guardrail/api.py`).
- **Three rails run end-to-end**: Project AirSim, ArduPilot SITL over
  pymavlink, and ArduPilot SITL over MAVROS 2 on ROS 2. All have A/B and
  dynamic-NFZ runs, plus the AirSimNH urban world. Gazebo has scripts but no
  kept run.

## Where the REFERENCE is stronger / more canonical (adopt from it)
- **Coordinate frame**: prof uses true WGS84 lat/lon + a projection layer; ours uses local meters. Theirs is grant-canonical (the DSL spec says WGS84).
- **Action frame**: theirs is body frame, as the grant locks (Architecture constraints p.2); ours is world frame.
- **Packaging discipline**: uv workspace, separate installable packages, mypy strict, MkDocs design site. Ours is a flat script tree.
- **Signed bundle as tar.gz** (policy_id/hash/generation/changelog/signature). In July ours hashed in memory only. Since September `guardrail/bundle.py` writes the same tar.gz container layout. The rule fields inside and the hash length (ours is cut to 16 hex digits) still differ, so neither repo can load the other's bundle (tracker card WP1-05).
- **ROS 2 = Humble** (grant/AI-Wings parity); we used Jazzy.
- **FSM** matches the spec's names exactly (Normal/Brake/RTL/Land).

## Recommended action
Treat the prof repo as the trunk. Our net-new pieces (Compiler, scenario sweep,
trained model, REST API, the MAVROS 2 rail) are candidate CONTRIBUTIONS to fold
in. First align them to its contracts: the WGS84 frame and the Action4D frame.
Their `vz` is up-positive like ours, but their vx/vy are body frame and ours are
world frame. Import `vlaguard_common` instead of our `guardrail.models`. Do NOT
diverge into a parallel fork.
