"""
The AGX run's BitVLA answers (responses.jsonl, written by probe_agx.py)
through the client side of the 4-D slot and a headless Shield (ARCH-06,
2026-11-22). Flies nothing.

For each frame's first answer: the client's strict reader re-reads the step
text (guardrail.aerialvla_codec.parse_action, lateral) and must agree with
the server's step; the StepTracker in the hil launcher's rate execution
(FAMILY=bitvla: --vla-step-as-rate 1.0, cruise 4.5 m/s, cap 5 m/s, turn while
moving; tests/test_vla_backends_cd_bitvla.LAUNCHER_TRACKER) turns it into the
Action4D the rail hands the Shield; Shield.filter (policies/demo_policy.yaml)
judges it at three states: open air, 2.5 m south of the demo no-fly square
heading North (into it), and 0.2 m above the altitude band's 2 m floor.

    python docs/data/bitvla_loader_20261122/shield_check.py
writes shield_check.json beside this file.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for _p in (ROOT, ROOT / "demo"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Shield                                         # noqa: E402
from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.models import Action4D, State                         # noqa: E402
from guardrail.models import load_policy                             # noqa: E402
from guardrail.vla_link import Decision, Pose, Step, StepTracker, TrackerConfig  # noqa: E402

LAUNCHER_TRACKER = TrackerConfig(cruise_mps=4.5, speed_cap_mps=5.0, rate_horizon_s=1.0,
                                 turn_while_moving=True)
STATES = {"open_air": State(x=-20.0, y=-20.0, up=4.0, yaw_deg=0.0),
          "south_of_nfz_square_heading_north": State(x=4.5, y=15.0, up=4.0, yaw_deg=0.0),
          "just_above_alt_band_floor": State(x=-20.0, y=-20.0, up=2.2, yaw_deg=0.0)}


def action4d(step_wire: dict, t0: float = 100.0) -> Action4D:
    tr = StepTracker(LAUNCHER_TRACKER)
    pose = Pose(0.0, 0.0, 10.0, 0.0)
    tr.accept(Decision.of(1, t0, pose, Step.from_wire(step_wire)), t0 + 0.05)
    return tr.tick(t0 + 0.15, pose).action


def a4(a: Action4D) -> dict:
    return {k: round(float(getattr(a, k)), 4) for k in ("vx", "vy", "vz_up", "yaw_rate")}


def main() -> int:
    rows = [json.loads(line) for line in (HERE / "responses.jsonl").read_text(
        encoding="utf-8").splitlines() if line.strip()]
    first = [r for r in rows if r["call"] == 0]
    out = []
    for r in first:
        resp = r["response"]
        p = C.parse_action(resp["raw_text"], require_eos=True, lateral=True)
        agree = bool(p.parse_ok and p.step.to_dict() == resp["step"])
        a = action4d(resp["step"])
        finite = all(math.isfinite(v) for v in (a.vx, a.vy, a.vz_up, a.yaw_rate))
        judged = {}
        for name, st in STATES.items():
            sh = Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"))
            sh.filter(st, Action4D())                           # the mission has started
            d = sh.filter(st, a)
            judged[name] = {
                "state": {"x": st.x, "y": st.y, "up": st.up, "yaw_deg": st.yaw_deg},
                "emitted": a4(d.emitted), "violations": [v.rule_id for v in d.violations],
                "repairs": [f"{x.operator}: {x.detail}" for x in d.repairs],
                "braked": d.braked, "fsm": [d.fsm_state_before, d.fsm_state_after],
                "command": a4(d.command) if d.command is not None else None}
        chunk0 = resp["native"]["action"]["chunk"][0]
        out.append({"frame": r["frame"], "chunk_row0": chunk0,
                    "velocity4": {k: resp["mapping"]["geometry"][k]
                                  for k in ("vx", "vy", "vz", "omega")},
                    "step_text": resp["raw_text"], "bins": resp["bins"],
                    "clipped": resp["mapping"]["clipped"], "step": resp["step"],
                    "client_reader_agrees": agree, "action4d": a4(a), "action4d_finite": finite,
                    "shield": judged})
        print(f"{r['frame']}: row0 {[round(v, 3) for v in chunk0]} -> Action4D {a4(a)}; "
              + "; ".join(f"{k}: emitted {v['emitted']} violations {v['violations']}"
                          for k, v in judged.items()))
    ok = all(o["client_reader_agrees"] and o["action4d_finite"] for o in out)
    (HERE / "shield_check.json").write_text(json.dumps(
        {"tracker": "rate execution H 1 s, cruise 4.5 m/s, cap 5 m/s, turn while moving",
         "policy": "policies/demo_policy.yaml", "frames": out, "all_conform": ok},
        indent=1) + "\n", encoding="utf-8")
    print("all conform:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
