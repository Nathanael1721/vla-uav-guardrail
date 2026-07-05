"""Mid-term acceptance demo (recommendation.md mid-term demo gate).

Runs the four steps the 2026-07-20 mid-term delivery must show, offline and
end-to-end in pure Python (the Gazebo/SITL functional-rail wiring lives in
``ros2_ws/`` + ``sim/`` and runs the same core):

1. Load a signed policy bundle (one polygon NFZ + one altitude envelope).
2. Issue a natural-language task whose straight-line path clips the NFZ.
3. Show the Safety Shield intercepting and applying lateral projection.
4. Show the audit-log entry whose ``policy_hash`` matches the loaded bundle.

It asserts the hard KPI — **P0 violation escape rate = 0** — over the flight.
Exit code 0 means the mid-term gate passes.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

from policy_dsl import ingest_file, load_bundle, write_bundle
from safety_shield import AuditLog, SafetyShield, VehicleState
from safety_shield.kinematics import body_to_enu
from vlaguard_common import DeterminismManifest, Topology

from demo.vla_stub import STUB_MODEL_HASH, Target, bearing_rad, stub_action

ROOT = Path(__file__).resolve().parents[1]
DEMO_SRC = ROOT / "bundles" / "itri-icl-2026-demo.yaml"
DT = 0.1  # 10 Hz
N_TICKS = 120  # 12 s flight


def main() -> int:
    print("=" * 70)
    print("Constrained VLA — mid-term acceptance demo (functional rail, offline)")
    print("=" * 70)

    # --- Step 1: load a signed policy bundle ------------------------------
    bundle_path = ROOT / "bundles" / "itri-icl-2026-demo-v0.3.0.tar.gz"
    write_bundle(ingest_file(DEMO_SRC), bundle_path)
    ir = load_bundle(bundle_path)
    print(f"\n[1] Loaded bundle {bundle_path.name}")
    print(f"    policy_id={ir.policy_id} version={ir.version} generation={ir.generation}")
    print(f"    policy_hash={ir.policy_hash}")
    print(
        f"    constraints: {len(ir.polygons)} polygon fence(s), "
        f"{len(ir.envelopes)} envelope(s)"
    )

    # --- Step 2: issue a natural-language task that clips the NFZ ----------
    task = "Fly north past the school-yard to waypoint W3, hugging its west edge."
    # straight path runs ~1 m inside the west wall of the NFZ -> Shield must project it out
    state = VehicleState(lat=25.04195, lon=121.531012, alt_agl_m=50.0, yaw_rad=0.0)
    target = Target(lat=25.04320, lon=121.531012, alt_agl_m=50.0)
    print(f"\n[2] Task: {task!r}")
    print(
        f"    start=({state.lat:.5f},{state.lon:.6f})  "
        f"target=({target.lat:.5f},{target.lon:.6f})"
    )

    manifest = DeterminismManifest(
        code_revision="dev-uncommitted",
        vla_model_hash=STUB_MODEL_HASH,
        policy_hash=ir.policy_hash,
        random_seed=1001,
        sim_speedup=1.0,
        topology=Topology.DEV,
    )

    # --- Step 3: run the Shield over the flight ---------------------------
    audit_path = ROOT / "episodes" / "midterm-demo" / "shield_audit.jsonl"
    audit_path.unlink(missing_ok=True)
    audit = AuditLog(audit_path, ir.policy_hash, ir.generation)
    shield = SafetyShield(ir, audit=audit)

    intercepts = 0
    lateral_projections = 0
    p0_escapes = 0
    p0_polys = [p for p in ir.polygons if p.priority == "P0"]

    for k in range(N_TICKS):
        yaw = bearing_rad(state, target)
        state = replace(state, yaw_rad=yaw)
        action = stub_action(state, target)
        decision = shield.tick(state, action, ts=f"2026-06-23T00:00:{k * DT:06.3f}Z")

        if decision.intercepted:
            intercepts += 1
        if any(
            a.operator == "LateralProjection" and a.result == "ok"
            for a in decision.repair_attempts
        ):
            lateral_projections += 1

        # integrate the EMITTED (post-Shield) action to advance the vehicle
        ve, vn, vu = body_to_enu(decision.emitted_action, yaw)
        e, n = ir.projection.to_xy(state.lat, state.lon)
        lat, lon = ir.projection.to_latlon(e + ve * DT, n + vn * DT)
        state = VehicleState(lat=lat, lon=lon, alt_agl_m=state.alt_agl_m + vu * DT, yaw_rad=yaw)

        # P0 escape = the ACTUAL (executed) position entered a P0 NFZ
        if any(ir.signed_distance(p, state.lat, state.lon) < 0 for p in p0_polys):
            p0_escapes += 1

    audit.close()
    print("\n[3] Flight complete:")
    print(
        f"    ticks={N_TICKS}  intercepts={intercepts}  "
        f"lateral_projections={lateral_projections}"
    )
    print(f"    final pos=({state.lat:.5f},{state.lon:.6f})  (held safely outside the NFZ)")

    # --- Step 4: audit log carries the matching policy_hash ---------------
    lines = audit_path.read_text().strip().splitlines()
    import json

    first = json.loads(lines[0]) if lines else {}
    print(f"\n[4] Audit log: {len(lines)} record(s) at {audit_path.relative_to(ROOT)}")
    print(f"    first record policy_hash={first.get('policy_hash')}")
    hash_ok = bool(lines) and first.get("policy_hash") == ir.policy_hash

    # --- KPI gate --------------------------------------------------------
    print("\n" + "-" * 70)
    print(f"KPI  P0 violation escape rate : {p0_escapes}  (target 0)")
    print(
        f"KPI  determinism manifest     : {len(manifest.model_dump())} fields, "
        f"topology={manifest.topology}"
    )
    print(f"     audit policy_hash match  : {'OK' if hash_ok else 'FAIL'}")
    print(f"     lateral projection fired : {'OK' if lateral_projections > 0 else 'FAIL'}")
    print("-" * 70)

    ok = p0_escapes == 0 and hash_ok and intercepts > 0 and lateral_projections > 0
    print("\nRESULT:", "PASS — mid-term gate criteria met" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
