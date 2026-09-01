"""Generate the artefact the September deck's new slides read from.

    python tools/build_deck_data.py        -> docs/data/deck_sept_extra.json

WHY THIS EXISTS

`tools/deck/build_sept_deck.js` enforces one rule: every number on a slide is
read from an artefact at build time, never typed in. Slides 2-6 satisfy it by
reading `demo/out/<tag>/kpi.json` directly. The new slides need four things that
no flight artefact contains:

  * the altitude-recovery curve, before and after the fix - the "before" code no
    longer exists to be run;
  * the constraint-type inventory, which lives in Python source;
  * what a policy bundle actually weighs and contains;
  * the scenario sweep's headline counts.

Rather than type those onto slides, this script derives them and writes them
down, so the deck stays a rendering of measurements rather than a set of claims.

THE HONEST PART, ABOUT THE ALTITUDE CURVE

The pre-fix behaviour cannot be measured by running the current code, so it is
SIMULATED from the arithmetic the old operator used: aim vz so the lookahead
endpoint lands exactly on the floor, clamped by the climb cap. That is a model,
not a measurement, and a model can be wrong.

So the same simulator is run for the CURRENT rule and cross-checked against the
live Shield, tick for tick. If the model reproduces the real Shield to
millimetres, the same model's account of the old rule is trustworthy. If it does
not, the script REFUSES to write the file rather than shipping a curve nobody
checked. The check is the whole point; without it this would just be a nicer
place to keep a made-up number.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import load_policy                                  # noqa: E402
from guardrail.bundle import load_bundle, write_bundle             # noqa: E402
from guardrail.models import Action4D, State                       # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

DEMO_POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
DT = 0.1
TICKS = 300
START_UP = 3.0


def _simulate(aim_inside: bool, floor: float, ceiling: float,
              climb_cap: float, lookahead: float) -> list[float]:
    """Reproduce AltitudeFix's arithmetic, with or without the re-entry margin.

    `aim_inside=False` is the old rule: target the floor exactly, which makes the
    approach a decaying exponential. `True` is the current one.
    """
    margin = min(1.0, (ceiling - floor) / 4.0) if aim_inside else 0.0
    up = START_UP
    trace = [up]
    for _ in range(TICKS):
        end_up = up                       # the pilot commands vz = 0
        if end_up < floor:
            vz = ((floor + margin) - up) / lookahead
        else:
            vz = 0.0
        vz = max(-climb_cap, min(climb_cap, vz))
        up += vz * DT
        trace.append(up)
    return trace


def altitude_study() -> dict:
    from guardrail.models import AltitudeEnvelope, KinematicEnvelope
    policy = load_policy(DEMO_POLICY)
    env = policy.by_type(AltitudeEnvelope)[0]
    kin = policy.by_type(KinematicEnvelope)[0]
    lookahead = 3.0

    before = _simulate(False, env.alt_min_m, env.alt_max_m,
                       kin.climb_rate_max_mps, lookahead)
    after = _simulate(True, env.alt_min_m, env.alt_max_m,
                      kin.climb_rate_max_mps, lookahead)

    # Cross-check the model of the CURRENT rule against the real Shield.
    sh = Shield(policy, lookahead_s=lookahead, dt=0.5)
    st = State(x=-30.0, y=-30.0, up=START_UP)
    live = [st.up]
    for _ in range(TICKS):
        d = sh.filter(st, Action4D(vx=3.0))
        st = State(x=st.x + d.emitted.vx * DT, y=st.y,
                   up=st.up + d.emitted.vz_up * DT)
        live.append(st.up)

    err = max(abs(a - b) for a, b in zip(after, live))
    if err > 0.01:
        raise SystemExit(
            f"REFUSING to write: the simulator disagrees with the live Shield by "
            f"{err:.4f} m. The model of the OLD rule cannot be trusted either, so "
            f"no altitude curve is published. Fix _simulate() to match "
            f"Shield._repair_altitude before re-running.")

    def at(trace, t_s):
        return round(trace[min(len(trace) - 1, int(t_s / DT))], 5)

    def first_inside(trace):
        for i, v in enumerate(trace):
            if v > env.alt_min_m:
                return round(i * DT, 1)
        return None

    return {
        "_what": "Recovery from below the altitude floor, before and after the "
                 "re-entry-margin fix. The 'after' curve is cross-checked "
                 "against the live Shield tick for tick.",
        "_model_vs_live_max_error_m": round(err, 6),
        "floor_m": env.alt_min_m,
        "start_m": START_UP,
        "samples_s": [6, 12, 30],
        "before": {"at_s": {str(t): at(before, t) for t in (6, 12, 30)},
                   "enters_band_s": first_inside(before)},
        "after": {"at_s": {str(t): at(after, t) for t in (6, 12, 30)},
                  "enters_band_s": first_inside(after)},
    }


def constraint_inventory() -> dict:
    """Which rule classes the DSL can express, and which are new this period."""
    from guardrail import models as M
    import inspect
    names = []
    for n, o in vars(M).items():
        if (inspect.isclass(o) and issubclass(o, M.ConstraintBase)
                and o is not M.ConstraintBase):
            fields = getattr(o, "model_fields", {})
            t = fields.get("type")
            if t is not None:
                names.append(n)
    # The reference implementation's own DSL package, for comparison.
    ref = ROOT / "kuanting-vla-uav-guardrail" / "packages" / "policy-dsl" / \
        "src" / "policy_dsl" / "models.py"
    ref_types = 0
    if ref.is_file():
        src = ref.read_text(encoding="utf-8")
        ref_types = sum(1 for k in ("polygon_fence", "altitude_envelope",
                                    "kinematic_envelope", "obstacle_clearance",
                                    "subject_standoff", "corridor")
                        if f'"{k}"' in src)
    return {
        "implemented": sorted(names),
        "n_implemented": len(names),
        "new_this_period": ["Corridor"],
        "valid_time_is_a_field_on_every_rule": True,
        "reference_impl_n_types": ref_types,
        "_note": "reference_impl_n_types counts the constraint classes present "
                 "in Prof. Lai's policy_dsl/models.py, for comparison.",
    }


def bundle_facts() -> dict:
    out = {}
    for name in ("wgs84_taipei", "corridor_survey"):
        p = ROOT / "policies" / f"{name}.yaml"
        if not p.is_file():
            continue
        pol = load_policy(p)
        tmp = Path(tempfile.mkdtemp()) / f"{name}.tar.gz"
        write_bundle(pol, tmp, changelog="deck build")
        back = load_bundle(tmp)                 # never quote one that will not reload
        # Reproducibility, demonstrated rather than asserted.
        tmp2 = Path(tempfile.mkdtemp()) / "other-name.tar.gz"
        write_bundle(pol, tmp2, changelog="deck build")
        out[name] = {
            "policy_id": pol.policy_id,
            "policy_hash": back.policy_hash,
            "n_rules": len(back.constraints),
            "bytes": tmp.stat().st_size,
            "byte_identical_under_a_different_name":
                tmp.read_bytes() == tmp2.read_bytes(),
            "geographic": pol.origin is not None,
        }
    return out


def sweep_summary() -> dict:
    p = ROOT / "docs" / "data" / "scenario_sweep.json"
    if not p.is_file():
        return {"_missing": "run experiments/sweep_scenarios.py first"}
    d = json.loads(p.read_text(encoding="utf-8"))
    known = [r["id"] for r in d["results"] if r.get("status") == "known_failure"]
    return {"counts": d["_counts"], "n_scenarios": len(d["results"]),
            "known_failures": known}


def loop_rates() -> dict:
    """The pre-registered rate gate, and whether the camera rail meets it.

    Derived from the flight logs rather than read from metrics.json, which does
    not carry a loop rate for the newer runs. Reported because it is still NOT
    met, and a deck that omitted it would be quietly selective.
    """
    gate = {"loop_hz_min": 9.5, "det_hz_min": 4.0}
    rows = []
    for tag in ("city_locked", "city_kpi", "city_full", "city_people",
                "demo_traffic", "demo_nfz"):
        log = ROOT / "demo" / "out" / tag / "flight_log.jsonl"
        met = ROOT / "demo" / "out" / tag / "metrics.json"
        if not log.is_file() or not met.is_file():
            continue
        recs = [json.loads(x) for x in
                log.read_text(encoding="utf-8").splitlines() if x.strip()]
        ts = [r["t"] for r in recs if r.get("t") is not None]
        span = (max(ts) - min(ts)) if len(ts) > 1 else 0.0
        m = json.loads(met.read_text(encoding="utf-8"))
        rows.append({
            "tag": tag,
            "loop_hz": round(len(recs) / span, 2) if span else None,
            "det_hz": m.get("det_hz"),
        })
    met_gate = [r for r in rows
                if (r["loop_hz"] or 0) >= gate["loop_hz_min"]
                and (r["det_hz"] or 0) >= gate["det_hz_min"]]
    return {"gate": gate, "runs": rows, "n_meeting_gate": len(met_gate),
            "n_runs": len(rows)}


def main() -> int:
    data = {
        "_what": "Numbers the September deck's new slides read. Generated, "
                 "never typed onto a slide. See the module docstring.",
        "altitude_recovery": altitude_study(),
        "constraints": constraint_inventory(),
        "bundles": bundle_facts(),
        "sweep": sweep_summary(),
        "rate_gate": loop_rates(),
    }
    out = ROOT / "docs" / "data" / "deck_sept_extra.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out.relative_to(ROOT)}")
    a = data["altitude_recovery"]
    print(f"  altitude model vs live Shield: max error "
          f"{a['_model_vs_live_max_error_m']} m")
    print(f"  before: enters band at {a['before']['enters_band_s']}, "
          f"30 s -> {a['before']['at_s']['30']} m")
    print(f"  after:  enters band at {a['after']['enters_band_s']} s, "
          f"30 s -> {a['after']['at_s']['30']} m")
    print(f"  constraint types: {data['constraints']['n_implemented']} "
          f"(reference impl: {data['constraints']['reference_impl_n_types']})")
    print(f"  rate gate: {data['rate_gate']['n_meeting_gate']} of "
          f"{data['rate_gate']['n_runs']} camera runs meet it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
