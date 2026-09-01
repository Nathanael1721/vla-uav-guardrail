"""Run the scenario library and score every run with the contractual KPIs.

    python experiments/sweep_scenarios.py                  # the whole library
    python experiments/sweep_scenarios.py --only nfz-head-on corridor-along
    python experiments/sweep_scenarios.py --backend sitl   # through ArduPilot

WHY THIS EXISTS

`docs/MIDTERM-REPORT-Aug2026.md` says, in those words, "Scenario sweep harness
not built." Everything this project claims about the Shield rests on a handful of
missions somebody chose to fly by hand, one at a time. A rule with no scenario is
a rule nobody has watched fail.

THE HEADLESS BACKEND, AND WHY IT IS THE DEFAULT

Shield plus a plain kinematic integrator: no simulator, no GPU, no autopilot, no
WSL. The whole library runs in about a second on any machine, which is the
difference between a harness that runs on every change and one that runs when
somebody remembers. It is not a flight-dynamics model and does not pretend to be
- it integrates exactly the 4-D action the contract defines, which is the only
thing the Shield is responsible for.

What it therefore CANNOT test: anything about the autopilot's tracking, timing
under load, or perception. Those live on the SITL rail and on Project AirSim, and
`--backend sitl` shells out to `sitl/run_sitl_demo.py` for them. Where that rail
is unavailable the scenario is reported SKIPPED - never passed. A harness that
quietly scores 11/11 while running 4 is worse than no harness.

SCORING

Through `guardrail.kpi.compute`, the same function the delivered flights use.
Writing a second KPI implementation here would let the sweep and the artefacts
disagree about what a P0 escape is, and the sweep would be the one nobody
checked.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import kpi as K, load_policy                        # noqa: E402
from guardrail.geometry import nearest_on_polyline                 # noqa: E402
from guardrail.models import Action4D, Corridor, State             # noqa: E402
from guardrail.shield import Shield                                # noqa: E402

LIB = ROOT / "experiments" / "scenarios.yaml"
GOAL_TOL_M = 3.0


# --------------------------------------------------------------------------- #
# pilots - deliberately dumb
# --------------------------------------------------------------------------- #

def _pilot(spec: dict, st: State) -> Action4D:
    """What the operator ASKS for, which may be entirely illegal.

    A pilot that avoids violations on its own would leave the Shield with
    nothing to do and every scenario would pass for the wrong reason.
    """
    kind = spec.get("type", "constant")
    if kind == "constant":
        return Action4D(**spec.get("action", {}))
    if kind == "goto":
        gx, gy = spec["to"]
        speed = float(spec.get("speed", 4.0))
        dx, dy = gx - st.x, gy - st.y
        d = math.hypot(dx, dy)
        vx, vy = (0.0, 0.0) if d < 1e-6 else (speed * dx / d, speed * dy / d)
        if d < GOAL_TOL_M:
            vx = vy = 0.0
        vz = 0.0
        if "up" in spec:
            vz = max(-2.0, min(2.0, float(spec["up"]) - st.up))
        return Action4D(vx=vx, vy=vy, vz_up=vz)
    raise ValueError(f"unknown pilot type {kind!r}")


def _advance(st: State, a: Action4D, dt: float) -> State:
    return State(x=st.x + a.vx * dt, y=st.y + a.vy * dt,
                 up=st.up + a.vz_up * dt, yaw_deg=st.yaw_deg)


# --------------------------------------------------------------------------- #
# the headless run
# --------------------------------------------------------------------------- #

def run_headless(sc: dict, defaults: dict) -> tuple[list[dict], dict]:
    dt = float(sc.get("dt", defaults.get("dt", 0.1)))
    ticks = int(sc.get("ticks", defaults.get("ticks", 300)))
    look = float(sc.get("lookahead_s", defaults.get("lookahead_s", 3.0)))
    policy = load_policy(ROOT / sc["policy"])

    clock = None
    if sc.get("at"):
        when = datetime.fromisoformat(sc["at"])
        clock = lambda: when                                     # noqa: E731
    shield = Shield(policy, lookahead_s=look, dt=0.5, now=clock)

    on = bool(sc.get("shield", True))
    subj = sc.get("subject")
    if subj:
        shield.set_subject(subj["x"], subj["y"], subj.get("class"))

    corridors = policy.by_type(Corridor)
    st = State(**sc["start"])
    rows: list[dict] = []
    max_speed = 0.0
    min_range = math.inf
    max_offset = 0.0
    finite = True

    for i in range(ticks):
        raw = _pilot(sc["pilot"], st)
        d = shield.filter(st, raw)
        # Shield OFF flies the RAW action, so the violations found on `raw` are
        # the flown action's - the same rule the SITL rails follow, and what
        # makes the control arm able to score a genuine escape.
        flown = d.emitted if on else raw
        flown_vios = d.emitted_violations if on else d.violations

        rows.append({
            "t": round(i * dt, 4), "tick": i,
            "x": st.x, "y": st.y, "up": st.up,
            "raw": raw.model_dump(), "emitted": flown.model_dump(),
            "violations": [v.model_dump() for v in d.violations],
            "emitted_violations": [v.model_dump() for v in flown_vios],
            "repairs": [r.model_dump() for r in d.repairs],
            "braked": d.braked,
            "unsafe": bool(shield.state_is_unsafe(st)),
        })

        max_speed = max(max_speed, math.hypot(flown.vx, flown.vy))
        finite = finite and all(math.isfinite(v) for v in
                                (flown.vx, flown.vy, flown.vz_up, flown.yaw_rate))
        if subj:
            min_range = min(min_range, math.hypot(st.x - subj["x"],
                                                  st.y - subj["y"]))
        for c in corridors:
            off, _, _ = nearest_on_polyline(st.x, st.y, c.points())
            max_offset = max(max_offset, off)

        st = _advance(st, flown, dt)

    reached = None
    if sc["pilot"].get("type") == "goto":
        gx, gy = sc["pilot"]["to"]
        reached = math.hypot(st.x - gx, st.y - gy) <= GOAL_TOL_M

    extra = {
        "max_speed_flown": round(max_speed, 4),
        "all_actions_finite": finite,
        "ended_safe": not rows[-1]["unsafe"],
        "min_range_to_subject": (None if min_range is math.inf
                                 else round(min_range, 3)),
        "max_offset_from_corridor": (round(max_offset, 3) if corridors else None),
        "reached_goal": reached,
        "final": {"x": round(st.x, 2), "y": round(st.y, 2), "up": round(st.up, 2)},
    }
    return rows, extra


# --------------------------------------------------------------------------- #
# the SITL backend
# --------------------------------------------------------------------------- #

def sitl_available() -> bool:
    """Can this host actually fly the ArduPilot rail?

    Checked rather than assumed, because the alternative - running nothing and
    printing a pass - is the failure mode this harness exists to remove.
    """
    return bool(shutil.which("wsl")) and (ROOT / "sitl" / "run_sitl_demo.py").is_file()


def run_sitl(sc: dict, out_dir: Path) -> tuple[list[dict], dict] | None:
    if not sitl_available():
        return None
    cmd = ["wsl", "python3", "sitl/run_sitl_demo.py",
           "--policy", sc["policy"], "--out", str(out_dir),
           "--shield", "on" if sc.get("shield", True) else "off"]
    if sc.get("subject"):
        cmd += ["--subject", f"{sc['subject']['x']},{sc['subject']['y']}"]
        if sc["subject"].get("class"):
            cmd += ["--subject-class", sc["subject"]["class"]]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    log = out_dir / "flight_log.jsonl"
    if r.returncode != 0 or not log.is_file():
        raise RuntimeError(f"SITL run failed ({r.returncode}): "
                           f"{(r.stderr or r.stdout)[-400:]}")
    rows = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()
            if x.strip()]
    return rows, {}


# --------------------------------------------------------------------------- #
# gates
# --------------------------------------------------------------------------- #

def check_gates(gates: dict, result: dict) -> list[str]:
    """Returns the failures. An absent or None field is a FAILURE, not a pass:
    a gate on something that was never measured has not been satisfied."""
    bad = []
    for field, rule in (gates or {}).items():
        got = result.get(field)
        if got is None:
            bad.append(f"{field}: not measured")
            continue
        if "max" in rule and got > rule["max"]:
            bad.append(f"{field}={got} > max {rule['max']}")
        if "min" in rule and got < rule["min"]:
            bad.append(f"{field}={got} < min {rule['min']}")
        if "equals" in rule and got != rule["equals"]:
            bad.append(f"{field}={got} != {rule['equals']}")
    return bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--library", default=str(LIB))
    ap.add_argument("--backend", choices=("headless", "sitl"), default="headless")
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--out", default=str(ROOT / "docs" / "data" / "scenario_sweep.json"))
    ap.add_argument("--runs", default=str(ROOT / "demo" / "out" / "sweep"))
    args = ap.parse_args()

    lib = yaml.safe_load(Path(args.library).read_text(encoding="utf-8"))
    defaults = lib.get("defaults", {})
    scenarios = [s for s in lib["scenarios"]
                 if not args.only or s["id"] in args.only]

    print(f"{len(scenarios)} scenarios, backend={args.backend}\n")
    rows_out, n_pass, n_fail, n_skip, n_known = [], 0, 0, 0, 0

    for sc in scenarios:
        sid = sc["id"]
        run_dir = Path(args.runs) / sid
        try:
            if args.backend == "sitl":
                got = run_sitl(sc, run_dir)
                if got is None:
                    n_skip += 1
                    print(f"  SKIP  {sid:<30} ArduPilot rail not available here")
                    rows_out.append({"id": sid, "status": "skipped",
                                     "reason": "sitl backend unavailable"})
                    continue
                rows, extra = got
            else:
                rows, extra = run_headless(sc, defaults)
        except Exception as e:                                   # noqa: BLE001
            n_fail += 1
            print(f"  ERROR {sid:<30} {type(e).__name__}: {e}")
            rows_out.append({"id": sid, "status": "error", "error": str(e)})
            continue

        policy = load_policy(ROOT / sc["policy"])
        res = K.compute(rows, K.rule_priorities(policy), {})
        res.update(extra)
        bad = check_gates(sc.get("gates"), res)
        known = sc.get("expect") == "known_failure"

        if bad and known:
            n_known += 1
            status, mark = "known_failure", "KNOWN"
        elif bad:
            n_fail += 1
            status, mark = "fail", "FAIL "
        elif known:
            # A known failure that started passing is news, not a quiet pass.
            n_fail += 1
            status, mark = "unexpected_pass", "FIXED"
            bad = ["marked known_failure but every gate passed - "
                   "the defect is fixed; remove the marker"]
        else:
            n_pass += 1
            status, mark = "pass", "pass "

        print(f"  {mark} {sid:<30} escape={res['p0_violation_escape_rate']:<9} "
              f"repairs={res['repair_count']:<5} t2safe_eps={res['time_to_safe_episodes']}")
        for b in bad:
            print(f"          {b}")

        rows_out.append({"id": sid, "status": status, "why": sc.get("why", "").strip(),
                         "gates_failed": bad, "kpi": res})

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "_what": "Every scenario in experiments/scenarios.yaml, scored with "
                 "guardrail.kpi.compute - the same function the delivered "
                 "flights use.",
        "_backend": args.backend,
        "_counts": {"pass": n_pass, "fail": n_fail,
                    "known_failure": n_known, "skipped": n_skip},
        "results": rows_out,
    }, indent=2) + "\n", encoding="utf-8")

    print(f"\n{n_pass} passed, {n_fail} failed, {n_known} known failures, "
          f"{n_skip} skipped -> {out.relative_to(ROOT)}")
    if n_known:
        print("Known failures are open defects, not passes. See the scenario's "
              "`why` for what is broken.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
