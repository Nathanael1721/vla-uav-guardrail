"""Apply the fail-safe rubric (docs/RUBRIC-failsafe.md) to the scenario library,
cell by cell, and compare it with the labels the library declares.

    python tools/failsafe_rubric.py                      # the library, seed 0
    python tools/failsafe_rubric.py --profile nightly    # a profile's cells and seeds
    python tools/failsafe_rubric.py --only dyn-nfz-spawn-ahead --json out.json

For every Shield-on, FSM-on episode (the grant's 5 s forecast, the ideal
vehicle, the canonical wording) it flies the cell's COUNTERFACTUAL - the same
episode with no fail-safe, the Shield's projection flown every tick
(`counterfactual`) - and prints the clause facts the rubric reads on that
flight, so no fact can depend on where the FSM's hold, RTL or landing took
the aircraft - never the FSM's state, edges or outcome:

  E1  a hard rule whose violation_action is RTL / land was violated by the
      planner's request (the Shield's decision, every tick);
  E2  the steady wind exceeds the hold authority (AP_HOLD_AUTHORITY_MPS);
  E3  the position the Shield acts on is wrong by more than the margin of the
      hard P0 polygon fences while the aircraft is within that error of one:
      noise (3 sigma) or latency (ticks x dt x the planned speed, capped)
      against the closest true approach to a fence, or - for a frozen fix -
      tick by tick, |true - seen| against the true distance to the fence;
  E5  the longest unbroken spell of planner requests the Shield may not fly
      (a hard rule's repair over theta or not converged, or a hard `loiter`
      rule violated), against loiter_timeout_s + one control period. Read
      from the Shield's decision at the state it was given: the TRUE state
      unless the cell degrades perception (then flagged `seen`).

PROPOSED, NOT ADOPTED (2026-11-06, docs/RUBRIC-failsafe.md section 7): on
this branch E5 is read on a second flight, the one the grant PRESCRIBES
(`prescribed`): projection abandoned on every tick whose request the Shield
may not fly, the aircraft held there, projection resumed the tick the
request is acceptable again (`--e5-flight prescribed`, this branch's
default). E1 and E3 still read the flight without a fail-safe. The adopted
rubric's reading is `--e5-flight no-failsafe`. Both spells are printed, so
every label the choice moves is listed.

PROPOSED, NOT ADOPTED (2026-11-07, docs/RUBRIC-failsafe.md section 8): the
vehicle the labels are derived on. Every flight above (and the FSM's own,
printed beside them) is flown on the ideal vehicle by default; `--response
ardupilot` flies them on the harness's measured ArduPilot response
(sweep_scenarios.VehicleResponse, `--response-arms ardupilot` of the sweep),
the vehicle that stands for the KPI-grade hil rail (ArduCopter 4.5.7 SITL in
GUIDED): a held aircraft is a zero setpoint flown through that response, as
the arm flies the Shield's stop and the autopilot's hold. `--extend-s S`
flies every episode S seconds longer than the scenario does, to show what a
run censored in Loiter, or a spell still running at its end, would have
become (the rubric's section 3: fly it longer).

The derived label is "expected" when any E clause holds. A cell whose
declared label differs is listed; the exit code is 1 if any does, 0 if none,
2 if nothing was judged. The FSM's own trigger is printed beside it for the
reader, and read by nothing here.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import sweep_scenarios as SW                                      # noqa: E402
from guardrail import fsm as F                                    # noqa: E402
from guardrail import shield as SH                                # noqa: E402
from guardrail.scenario_spec import Library, expand               # noqa: E402


def unsafe_request(rec: dict) -> bool:
    """The rubric's E5 tick: what the FSM's G3 calls an untrusted repair of a
    hard rule (over theta or not converged, not the legal stop a `brake`
    rule asks for), or a violated hard `loiter` rule. Read from the decision
    record's own fields (outcome, theta_exceeded, the violated rules), which
    do not depend on the FSM's state."""
    vios = [v for v in rec.get("violations") or []
            if v.get("effective_action") != "monitor_only"]
    hard = [v for v in vios if v.get("constraint_type") == "hard"]
    if not hard:
        return False
    if any(v.get("effective_action") == "loiter" for v in hard):
        return True
    untrusted = rec.get("outcome") == "blocked" or bool(rec.get("theta_exceeded"))
    projected = [v for v in hard if v.get("effective_action") == "project_fix"
                 and not v.get("kinematic")]
    strongest = (rec.get("governing_rule") or {}).get("effective_action")
    stop_rule = strongest == "brake" and not (rec.get("flags") or {}).get("stop_illegal")
    return bool(untrusted and projected and not stop_rule)


def demands_failsafe(rec: dict) -> bool:
    """The rubric's E1 tick: a hard rule asking for RTL or Land is violated."""
    return any(v.get("constraint_type") == "hard"
               and v.get("effective_action") in ("RTL", "land")
               for v in rec.get("violations") or [])


def _fences(policy):
    """Hard P0 polygon fences of the policy as loaded (shapely polygons and
    their margins). Hot-applied zones are not here: E3 is judged on cells
    whose fences are the policy's own (the sensor families)."""
    from shapely.geometry import Polygon
    out = []
    for c in getattr(policy, "constraints", None) or []:
        if getattr(c, "type", None) != "polygon_fence":
            continue
        if getattr(c, "constraint_type", "hard") != "hard" or getattr(c, "priority", "P0") != "P0":
            continue
        verts = [(v.x, v.y) for v in c.vertices]
        out.append((Polygon(verts), float(getattr(c, "margin_m", 0.0) or 0.0)))
    return out


def e3_facts(spec, run, dt: float) -> dict:
    st = spec.stress
    fences = _fences(run.policy)
    if not fences or not st.degrades_perception:
        return {"holds": False, "why": "no degraded perception" if fences else "no P0 fence"}
    from shapely.geometry import Point
    margin = min(m for _, m in fences)

    def dist(x, y):
        return min(poly.exterior.distance(Point(x, y)) if not poly.contains(Point(x, y))
                   else 0.0 for poly, _ in fences)

    caps = F.KinematicCaps.from_policy(run.policy)
    speed = float(spec.mission.pilot.speed or 4.0)
    if caps is not None:
        speed = min(speed, caps.speed_max_mps)
    closest = min(dist(r["x"], r["y"]) for r in run.rows)
    if st.gps_dropout_s is not None:
        worst = None
        for r in run.rows:
            p = r.get("perceived")
            if not p:
                continue
            err = math.hypot(r["x"] - p["x"], r["y"] - p["y"])
            if err > margin and dist(r["x"], r["y"]) <= err:   # "within" that error: inclusive
                worst = max(worst or 0.0, err)
        return {"holds": worst is not None, "kind": "frozen fix", "margin_m": margin,
                "worst_error_m": None if worst is None else round(worst, 2),
                "closest_true_m": round(closest, 2)}
    # White noise and latency are judged against the mission's PLANNED route
    # (start to target, as the scenario declares it), never against where a
    # noisy flight happened to go: the flown closest approach depends on the
    # noise draw, and a label belongs to the cell, not to a seed (rubric,
    # section 3; amended 2026-11-03 after the nightly check flew
    # gps-noise-beyond-margin latency 5 / 23.2 / sigma 1 on seed 1003 a few
    # millimetres beyond its 3.0 m error while seeds 1001 and 1002 came to
    # 2.7 m: same cell, two labels).
    from shapely.geometry import LineString
    import random as _random
    m = SW._resolve_mission(spec, run.policy, _random.Random(0))
    route_d = None
    if m.target is not None:
        route = LineString([(m.start.x, m.start.y), (m.target[0], m.target[1])])
        route_d = min(poly.exterior.distance(route) if not poly.intersects(route) else 0.0
                      for poly, _ in fences)
    err = max(3.0 * st.gps_noise_m, st.latency_ticks * dt * speed)
    near = route_d if route_d is not None else closest
    return {"holds": err > margin and near <= err, "kind": "noise/latency",
            "error_m": round(err, 2), "margin_m": margin,
            "route_to_boundary_m": None if route_d is None else round(route_d, 2),
            "closest_true_m": round(closest, 2)}


def counterfactual(spec, lib, seed: int, response: str = SW.DEFAULT_RESPONSE_ARM):
    """The cell flown WITHOUT a fail-safe: the Shield projects every tick and
    its action is flown (spec.escalation False), so where the aircraft goes
    cannot depend on the FSM's holds, RTLs or landings. The flying Shield
    still carries an FSM, only so that each tick's decision record (outcome,
    theta, the violated rules, stop_illegal) is computed exactly as the real
    Shield computes it; the harness never reads its verdict (Shield._escalate
    builds `emitted` before the FSM sees it). `response` is the vehicle the
    actions are flown on (the sweep's RESPONSE_ARMS; proposed 2026-11-07,
    section 8: "ardupilot" for the rail's labels). Returns (run, records)."""
    log: list[dict] = []
    orig_cls, orig_filter = SW.Shield, SH.Shield.filter
    made = [0]

    def factory(*a, **k):
        made[0] += 1
        if made[0] == 1:                      # the flying Shield (_episode builds it first)
            k["escalation"] = True
        return orig_cls(*a, **k)

    def rec_filter(self, *a, **k):
        d = orig_filter(self, *a, **k)
        if getattr(self, "fsm", None) is not None:
            log.append(getattr(d, "fsm_record", None) or {})
        return d

    SW.Shield, SH.Shield.filter = factory, rec_filter
    try:
        run = SW.run_episode(spec.model_copy(update={"escalation": False}),
                             lib.defaults, seed, response=response)
    finally:
        SW.Shield, SH.Shield.filter = orig_cls, orig_filter
    return run, log


def prescribed(spec, lib, seed: int, response: str = SW.DEFAULT_RESPONSE_ARM):
    """The cell flown as the grant PRESCRIBES the fail-safe path, without the
    fail-safe itself (proposed 2026-11-06, NOT adopted: docs/RUBRIC-
    failsafe.md, section 7). Safety Shield p4: "If ... > threshold, or no
    operator converges, the Shield abandons projection" - so on a tick whose
    request the Shield may not fly (`unsafe_request`: a hard rule's repair
    over theta or not converged, or a hard `loiter` rule violated) the
    projection is not flown and the aircraft is HELD: a stop, as Brake and
    the autopilot's Loiter hold it on the ideal vehicle. Where a stop is
    illegal (`stop_illegal`: holding there would itself break a rule) the
    Shield's own action flies, as a hold cannot. On every other tick the
    Shield's action flies: projection resumes the tick the request is
    acceptable again. Nothing here reads the FSM's state, edges or timers -
    not T_recover, not N-in-T, not the Loiter timeout - so the flight is the
    grant's prescription, not this FSM's implementation of it. On the
    "ardupilot" response the stop is the zero setpoint flown through the
    measured response - the arm's own physics for the Shield's stop and the
    autopilot's hold - so the aircraft carries on while it brakes, as on the
    rail. Returns (run, records), as `counterfactual`."""
    log: list[dict] = []
    orig_cls, orig_filter = SW.Shield, SH.Shield.filter
    made = [0]

    def factory(*a, **k):
        made[0] += 1
        if made[0] == 1:                      # the flying Shield (_episode builds it first)
            k["escalation"] = True
        return orig_cls(*a, **k)

    def held_filter(self, state, *a, **k):
        d = orig_filter(self, state, *a, **k)
        if getattr(self, "fsm", None) is None:
            return d
        rec = getattr(d, "fsm_record", None) or {}
        log.append(rec)
        if unsafe_request(rec) and not (rec.get("flags") or {}).get("stop_illegal"):
            d = d.model_copy(update={"emitted": SH.BRAKE,
                                     "emitted_violations": self._check(state, SH.BRAKE)})
        return d

    SW.Shield, SH.Shield.filter = factory, held_filter
    try:
        run = SW.run_episode(spec.model_copy(update={"escalation": False}),
                             lib.defaults, seed, response=response)
    finally:
        SW.Shield, SH.Shield.filter = orig_cls, orig_filter
    return run, log


def longer(spec, lib, extend_s: float):
    """`spec` flown `extend_s` seconds longer than it declares (whole ticks,
    rounded up); the same spec when `extend_s` is 0. The rubric's answer to
    a censored run (section 3: fly it longer), applied without editing the
    library."""
    if not extend_s:
        return spec
    if not (math.isfinite(extend_s) and extend_s > 0):
        raise ValueError(f"extend_s must be finite and > 0, got {extend_s}")
    dt = float(spec.dt or lib.defaults.dt)
    ticks = int(spec.ticks or lib.defaults.ticks)
    return spec.model_copy(update={"ticks": ticks + int(math.ceil(extend_s / dt - 1e-9))})


# Which flight E5 is read on: "no-failsafe" (the adopted rubric, section 3)
# or "prescribed" (proposed 2026-11-06, not adopted; this branch's labels).
E5_FLIGHTS = ("no-failsafe", "prescribed")
E5_FLIGHT = "prescribed"


def _spell(log: list[dict], dt: float) -> tuple[float, float, int]:
    """(longest unbroken spell of unsafe requests, s; the spell still running
    at the end, s; its length in ticks)."""
    best = cur = 0
    for rec in log:
        cur = cur + 1 if unsafe_request(rec) else 0
        best = max(best, cur)
    return (round((best - 1) * dt, 1) if best else 0.0,
            round((cur - 1) * dt, 1) if cur else 0.0, cur)


def judge(e, lib, timeout_s: float, with_fsm: bool = True,
          e5_flight: str = E5_FLIGHT, response: str = SW.DEFAULT_RESPONSE_ARM,
          extend_s: float = 0.0) -> dict:
    """Every clause fact of one cell and seed, both readings of E5, on the
    vehicle `response`, the episode flown `extend_s` seconds longer than it
    declares; with `with_fsm` also the FSM's own flight on that vehicle
    (whether it triggered, and whether it ended censored in Loiter), which
    the labels never read."""
    if e5_flight not in E5_FLIGHTS:
        raise ValueError(f"e5_flight {e5_flight!r}: one of {E5_FLIGHTS}")
    spec = longer(e.cell.spec, lib, extend_s)
    run, log = counterfactual(spec, lib, e.seed, response)
    _, held_log = prescribed(spec, lib, e.seed, response)
    flown = (SW.run_episode(spec, lib.defaults, e.seed, response=response)
             if with_fsm else None)
    dt = float(spec.dt or lib.defaults.dt)
    spells = {"no-failsafe": _spell(log, dt), "prescribed": _spell(held_log, dt)}
    spell, tail, cur = spells[e5_flight]
    other = "prescribed" if e5_flight == "no-failsafe" else "no-failsafe"
    need = timeout_s + F.CONTROL_PERIOD_S
    e1 = any(demands_failsafe(r) for r in log)
    e2 = spec.stress.wind_speed_mps > SW.AP_HOLD_AUTHORITY_MPS
    e3 = e3_facts(spec, run, dt)
    e5 = spell >= need - 1e-9
    # A spell still running when the episode ends, shorter than the timeout:
    # E5 is undecided, and the family must fly longer (rubric, section 3).
    e5_censored = bool(cur) and tail < need - 1e-9
    held = [c for c, h in (("E1", e1), ("E2", e2), ("E3", e3["holds"]), ("E5", e5)) if h]
    o_spell, o_tail, o_cur = spells[other]
    o_e5 = o_spell >= need - 1e-9
    o_held = [c for c, h in (("E1", e1), ("E2", e2), ("E3", e3["holds"]), ("E5", o_e5)) if h]
    fsm = (flown.extra.get("fsm") or {}) if flown is not None else {}
    censored = None
    if flown is not None:
        censored = bool(SW.score(spec, flown.rows, flown.extra, flown.policy,
                                 flown.priorities).get("failsafe_censored"))
    return {"episode": e.cell.episode_id, "seed": e.seed,
            "response": response, "extend_s": extend_s,
            "declared": spec.expected_failsafe, "clause": spec.failsafe_clause,
            "derived": bool(held), "clauses_held": held,
            "e5_flight": e5_flight,
            "e5_spell_s": spell, "e5_need_s": round(need, 1), "e5_censored": e5_censored,
            "e5_judged_at": "seen" if spec.stress.degrades_perception else "true",
            # The same cell under the other reading of E5, for the flip list.
            "other": {"e5_flight": other, "e5_spell_s": o_spell,
                      "e5_censored": bool(o_cur) and o_tail < need - 1e-9,
                      "derived": bool(o_held), "clauses_held": o_held},
            "e3": e3, "wind_mps": spec.stress.wind_speed_mps,
            "episode_s": round(len(run.rows) * dt, 1),
            "fsm_triggered": fsm.get("failsafe_triggered"),
            "fsm_censored": censored,
            "fsm_edges": " > ".join(t["edge"] for t in fsm.get("transitions", []))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--profile", default=None)
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--json", default=None, help="write every cell's facts here")
    ap.add_argument("--no-fsm", action="store_true",
                    help="skip the flight WITH the FSM (printed for the reader only)")
    ap.add_argument("--e5-flight", choices=E5_FLIGHTS, default=E5_FLIGHT,
                    help="the flight E5 is read on: no-failsafe (the adopted rubric) or "
                         "prescribed (proposed 2026-11-06, not adopted; default here)")
    ap.add_argument("--response", choices=SW.RESPONSE_ARMS, default=SW.DEFAULT_RESPONSE_ARM,
                    help="the vehicle every flight is flown on: ideal (the headless "
                         "labels) or ardupilot (the measured response that stands for "
                         "the hil rail; proposed 2026-11-07, not adopted)")
    ap.add_argument("--extend-s", type=float, default=0.0,
                    help="fly every episode this many seconds longer than it declares")
    args = ap.parse_args(argv)
    prof = SW._resolve_profile(args.profile)
    lib = Library(ROOT / (prof.library if prof else "experiments/scenarios.yaml"))
    eps, _ = expand(lib, prof, None, args.only)
    eps = [e for e in eps if not e.comparison and e.prefix == "on"
           and e.paraphrase_arm in (None, -1)
           and e.cell.spec.shield and e.cell.spec.escalation]
    timeout = F.FSMConfig().loiter_timeout_s
    print(f"vehicle {args.response}, E5 on the {args.e5_flight} flight"
          + (f", every episode {args.extend_s:g} s longer" if args.extend_s else ""))
    out, bad, flips = [], [], []
    for e in eps:
        j = judge(e, lib, timeout, with_fsm=not args.no_fsm, e5_flight=args.e5_flight,
                  response=args.response, extend_s=args.extend_s)
        out.append(j)
        mark = ""
        if j["e5_censored"] and not j["derived"]:
            mark = "CENSORED"
            bad.append(j)
        elif j["declared"] is None:
            mark = "UNLABELLED" + (" (rubric: expected)" if j["derived"] else "")
        elif j["declared"] != j["derived"]:
            mark = "MISMATCH"
            bad.append(j)
        o = j["other"]
        if o["derived"] != j["derived"]:
            flips.append(j)
        print(f"{mark:11} {j['episode'][:70]:70} s{j['seed']:<5} declared "
              f"{str(j['declared']):5} {str(j['clause']):4} derived {str(j['derived']):5} "
              f"{','.join(j['clauses_held']) or '-':8} E5 {j['e5_spell_s']:5.1f}/"
              f"{j['e5_need_s']} ({j['e5_judged_at']}, {j['e5_flight']}; {o['e5_flight']} "
              f"{o['e5_spell_s']:.1f}{' FLIPS' if o['derived'] != j['derived'] else ''}) "
              f"ep {j['episode_s']:5.1f} s fsm {j['fsm_triggered']}"
              f"{' CENSORED-IN-LOITER' if j['fsm_censored'] else ''} {j['fsm_edges']}")
        sys.stdout.flush()
    if args.json:
        Path(args.json).write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(f"\n{len(out)} cells judged, {len(bad)} declared labels disagree with the rubric "
          f"or cannot be decided in the episode's length (CENSORED)")
    print(f"{len(flips)} cells derive another label with E5 read on the "
          f"{out[0]['other']['e5_flight'] if out else 'other'} flight")
    for j in flips:
        o = j["other"]
        print(f"  {j['episode'][:70]:70} s{j['seed']:<5} {j['e5_flight']} "
              f"{j['derived']!s:5} (E5 {j['e5_spell_s']:.1f} s) | {o['e5_flight']} "
              f"{o['derived']!s:5} (E5 {o['e5_spell_s']:.1f} s)")
    if not out:
        return 2
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
