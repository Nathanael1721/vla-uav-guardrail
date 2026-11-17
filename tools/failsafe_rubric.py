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
      against the closest true approach to a fence, or - for a GPS denial -
      tick by tick, |true - seen| against the true distance to the fence
      (seen: the frozen last fix on the ideal vehicle, the autopilot's dead-
      reckoned estimate on the rail vehicle, as the rails realise it;
      2026-11-12);
  E5  the longest unbroken spell of planner requests the Shield may not fly
      (a hard rule's repair over theta or not converged, or a hard `loiter`
      rule violated), against loiter_timeout_s + one control period. Read
      from the Shield's decision at the state it was given: the TRUE state
      unless the cell degrades perception (then flagged `seen`).
  E6  (since 2026-11-15, the rubric's section 9: "N-in-T persists", the
      grant's G4) hard-violation onsets MADE BY THE WORLD recur at the
      N-in-T rate through a Loiter: onsets that start on the tick a world
      event of the episode's own record puts the violated hard rule in force
      or changes it (`e6_facts`), 2 x N of them, every N consecutive ones
      within T, the request never clear of a hard violation for T_recover in
      between. An onset on no world event - a forecast re-touching a ring
      (N1), noise in the Shield's view of the position (N2) - is never
      counted. Decided 2026-11-15 by the lab under the PI's instruction of
      6 Oct (follow the grant; best practice where it is silent), the PI's
      confirmation still to be recorded, and AFTER the flown run hs10_003.

ADOPTED 2026-11-10 (docs/RUBRIC-failsafe.md sections 7 and 8; the user's
decision of 9-10 November, "follow the best practice that meets the KPI
criteria"), AFTER the rig had shown the autopilot's lag (rh01, 2026-11-07):
not a pre-registered reading, and the earlier readings are still printed.

  * E5 is read on the flight the grant PRESCRIBES (`prescribed`, section 7):
    projection abandoned on every tick whose request the Shield may not fly,
    the aircraft held there, projection resumed the tick the request is
    acceptable again (`--e5-flight prescribed`, the default). E1 and E3 still
    read the flight without a fail-safe. Section 3's reading is `--e5-flight
    no-failsafe`. Both spells are printed, so every label the choice moves
    is listed.
  * The KPI-grade labels are derived on the vehicle that flies them (section
    8): `--response rail` flies every flight above on the rig's own fitted
    response (sweep_scenarios.RAIL_VEHICLE: the response fitted to the hil
    rig's flights, entered from a hover; `ardupilot` is the Copter-4.5.7
    fit the proposal of 7 November used). A held aircraft is a zero setpoint
    flown through that response. Since 2026-11-12 a GPS denial on the rail
    vehicle reaches the Shield as the rails realise it - EKF3's dead-
    reckoned estimate, drifting at the rig's measured RAIL_EKF_DRIFT_MPS -
    not as the frozen last fix (`_rail_kw`). The headless labels (experiments/templates)
    stay the ideal vehicle's.
  * `--extend-s S` flies every episode S seconds longer than the scenario
    does, to show what a spell still running at its end would have become
    (section 3: fly it longer). A rail flies every mission 35 s longer than
    the scenario (sweep_scenarios.rail_max_s) and its fail-safe is read over
    that whole flight, so the artefact below reads a rail label left
    undecided at the scenario's length on the cell flown those 35 s longer;
    what is undecided even then stays UNDECIDED, not guessed.

THE LABEL ARTEFACT (docs/data/failsafe_labels_rail.json). Four readings per
cell and seed, each with the clauses that hold and the E5 spell:

    a   ideal vehicle, E5 on the flight without a fail-safe (section 3)
    a2  ideal vehicle, E5 on the prescribed flight (the templates' labels)
    r   rail vehicle,  E5 on the flight without a fail-safe
    r2  rail vehicle,  E5 on the prescribed flight - THE ADOPTED KPI READING

built by `--build-labels OUT IN.json ...` from this tool's own `--json`
outputs (one per profile and vehicle; the command lines and the code
revision they ran at are written into it), and read by `experiments/
sweep_scenarios.py --rescore-rail` to print a fetched campaign's fail-safe
correctness under (a), (r) and (r2). A change that bears on some cells only
re-derives those in place (`--update-labels ART IN.json ... --why ...`,
since 2026-11-12): their readings are replaced, every other one is kept,
and the update - why, its commands and revisions, every reading it moved,
before and after - is appended to `built.updates`.

The derived label is "expected" when any E clause holds (E1-E3, E5, E6; no
cell is judged under E4). A cell whose
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
    # Whose repair it is, as the FSM's G3 reads it since 2026-11-07
    # (FSMConfig.untrusted_owner "chain"): the violated non-kinematic
    # project_fix rules, else - when none is projected - the kinematic ones
    # the chain ran for; and it must be a hard one.
    owners = [v for v in vios if v.get("effective_action") == "project_fix"
              and not v.get("kinematic")]
    if not owners:
        owners = [v for v in vios if v.get("effective_action") == "project_fix"
                  and v.get("kinematic")]
    projected = [v for v in owners if v.get("constraint_type") == "hard"]
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
    from guardrail.geometry import area_meets, edge_gap, signed_edge_distance
    margin = min(m for _, m in fences)

    def dist(x, y):
        return min(max(0.0, signed_edge_distance(poly, x, y)) for poly, _ in fences)

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
        model = ((run.events[0] if run.events else {}).get("gps_dropout") or {}).get(
            "model", SW.DEFAULT_GPS_DROPOUT)
        return {"holds": worst is not None,
                "kind": "frozen fix" if model == "frozen" else "dead-reckoned fix",
                "margin_m": margin,
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
        route_d = min(edge_gap(poly, route) if not area_meets(poly, route) else 0.0
                      for poly, _ in fences)
    err = max(3.0 * st.gps_noise_m, st.latency_ticks * dt * speed)
    near = route_d if route_d is not None else closest
    return {"holds": err > margin and near <= err, "kind": "noise/latency",
            "error_m": round(err, 2), "margin_m": margin,
            "route_to_boundary_m": None if route_d is None else round(route_d, 2),
            "closest_true_m": round(closest, 2)}


# E6 (2026-11-15; docs/RUBRIC-failsafe.md sections 2 and 9). The harness's
# hot-apply operations that can START a hard violation on the tick they land:
# a rule spawned, switched on, swapped, resized or set moving. (A time window
# opening by the clock is the episode's `time_window_switch` record; a
# switch-off ends a violation and starts none.)
E6_WORLD_OPS = ("spawn", "switch_on", "swap", "scale", "move")


def _hard_rules(rec: dict) -> set:
    """The hard, enforced rules the planner's request violates on one tick
    (the FSM's own `hard_violated`: not monitor_only), by id."""
    return {str(v.get("rule_id")) for v in rec.get("violations") or []
            if v.get("constraint_type") == "hard"
            and v.get("effective_action") != "monitor_only"}


def world_changes(events: list[dict]) -> list[tuple[float, set]]:
    """(when, the rules it names) for every world event of an episode's own
    record (run.events) that puts a rule in force or changes it: an accepted
    hot-apply of E6_WORLD_OPS (its `rule_id`, `target` and `zone`) or a time
    window opening (`time_window_switch`, its `activated`)."""
    out = []
    for ev in events or []:
        t = ev.get("t")
        if isinstance(t, bool) or not isinstance(t, (int, float)) or ev.get("refused"):
            continue
        if ev.get("type") == "hot_apply" and ev.get("op") in E6_WORLD_OPS:
            names = {str(ev[k]) for k in ("rule_id", "target", "zone") if ev.get(k)}
        elif ev.get("type") == "time_window_switch":
            names = {str(x) for x in ev.get("activated") or []}
        else:
            continue
        if names:
            out.append((float(t), names))
    return out


def e6_facts(events: list[dict], log: list[dict], dt: float, cfg=None) -> dict:
    """The rubric's E6 (section 9, 2026-11-15; the grant's G4, "N-in-T
    persists"), from one flight's decision records `log` (one per tick, as
    `counterfactual` returns them) and its event record `events`.

    An ONSET is a tick whose request violates a hard, enforced rule after a
    tick whose request violated none. It is WORLD-MADE when a world event
    (`world_changes`) recorded since the previous tick names a rule violated
    on that tick - the world switched the rule on or over the request - and
    it is counted at most once per `onset_clear_s`, as the grant's N-in-T is
    counted. E6 holds on a run of 2 x N counted world-made onsets in which
    every N consecutive ones lie within T (the N-in-T rate: the N of the
    grant's G2 and N more after them, G4) and no spell without a hard
    violation between two of them lasts T_recover (the grant's "cleared for
    >= T_recover" would end the hold there). N, T, T_recover and
    onset_clear_s are the FSM's CONFIGURED values (`cfg`, FSMConfig), as E5
    reads its timeout; nothing here reads the FSM's state, edges or counters
    (the record's own `n_in_t` among them). An onset on no world event - a
    forecast re-touching a ring (N1), noise in the Shield's view of the
    position (N2) - is counted in `onsets` and never in `world_onsets`."""
    cfg = cfg or F.FSMConfig()
    n, win = int(cfg.n_violations), float(cfg.window_s)
    clear_s, t_rec = float(cfg.onset_clear_s), float(cfg.t_recover_s)
    changes = world_changes(events)
    onsets: list[float] = []                # every onset of a hard violation
    world: list[float] = []                 # the counted world-made ones
    clean: list[tuple[float, float]] = []   # every spell with no hard violation
    prev = False
    t_prev = clean_from = last = None
    for i, rec in enumerate(log):
        t = rec.get("t_s")
        if isinstance(t, bool) or not isinstance(t, (int, float)):
            t = round(i * dt, 6)
        t = float(t)
        hard = _hard_rules(rec)
        if hard:
            if clean_from is not None:
                clean.append((clean_from, t_prev))
                clean_from = None
            if not prev:
                onsets.append(t)
                since = t - dt if t_prev is None else t_prev
                made = any(since + F.EPS < ct <= t + F.EPS and names & hard
                           for ct, names in changes)
                if made and (last is None or t - last >= clear_s - F.EPS):
                    world.append(t)
                    last = t
        elif clean_from is None:
            clean_from = t
        prev, t_prev = bool(hard), t
    if clean_from is not None:
        clean.append((clean_from, t_prev))

    def cleared(a: float, b: float) -> bool:
        return any(a < c0 and c1 < b and c1 - c0 >= t_rec - F.EPS for c0, c1 in clean)
    best = start = 0
    for k in range(len(world)):
        if k and cleared(world[k - 1], world[k]):
            start = k
        if k - start + 1 >= n and world[k] - world[k - n + 1] > win + F.EPS:
            start = k - n + 2
        best = max(best, k - start + 1)
    return {"holds": best >= 2 * n, "need": 2 * n, "n": n, "window_s": win,
            "t_recover_s": t_rec, "onset_clear_s": clear_s,
            "world_events": len(changes), "onsets": len(onsets),
            "world_onsets": [round(x, 3) for x in world], "longest_run": best}


def _rail_kw(response: str) -> dict:
    """What the rail vehicle flies besides its response (sweep_scenarios.
    RAIL_VEHICLE): a GPS denial reaches the Shield as the autopilot's own
    dead-reckoned estimate, as SITL's EKF3 hands it on the rails, not as the
    frozen last fix the headless library models (2026-11-12; GPS_DROPOUT_MODELS).
    Nothing for any other vehicle."""
    if response == SW.RAIL_VEHICLE["response"]:
        return {"gps_dropout": SW.RAIL_VEHICLE["gps_dropout"]}
    return {}


def counterfactual(spec, lib, seed: int, response: str = SW.DEFAULT_RESPONSE_ARM):
    """The cell flown WITHOUT a fail-safe: the Shield projects every tick and
    its action is flown (spec.escalation False), so where the aircraft goes
    cannot depend on the FSM's holds, RTLs or landings. The flying Shield
    still carries an FSM, only so that each tick's decision record (outcome,
    theta, the violated rules, stop_illegal) is computed exactly as the real
    Shield computes it; the harness never reads its verdict (Shield._escalate
    builds `emitted` before the FSM sees it). `response` is the vehicle the
    actions are flown on (the sweep's RESPONSE_ARMS; section 8, adopted
    2026-11-10: "rail" for the KPI-grade labels). Returns (run, records)."""
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
                             lib.defaults, seed, response=response, **_rail_kw(response))
    finally:
        SW.Shield, SH.Shield.filter = orig_cls, orig_filter
    return run, log


def prescribed(spec, lib, seed: int, response: str = SW.DEFAULT_RESPONSE_ARM):
    """The cell flown as the grant PRESCRIBES the fail-safe path, without the
    fail-safe itself (docs/RUBRIC-failsafe.md, section 7; adopted
    2026-11-10). Safety Shield p4: "If ... > threshold, or no
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
                             lib.defaults, seed, response=response, **_rail_kw(response))
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


# Which flight E5 is read on: "prescribed" (adopted 2026-11-10, section 7; the
# templates' labels) or "no-failsafe" (section 3's reading, still printed).
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
    hold = (SW.RAIL_VEHICLE["autopilot_hold"] if response == SW.RAIL_VEHICLE["response"]
            else SW.DEFAULT_AUTOPILOT_HOLD)
    flown = (SW.run_episode(spec, lib.defaults, e.seed, response=response,
                            autopilot_hold=hold, **_rail_kw(response))
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
    # E6 (2026-11-15): read, like E1, on the flight without a fail-safe; the
    # same on both readings of E5.
    e6 = e6_facts(run.events, log, dt)
    # A spell still running when the episode ends, shorter than the timeout:
    # E5 is undecided, and the family must fly longer (rubric, section 3).
    e5_censored = bool(cur) and tail < need - 1e-9
    held = [c for c, h in (("E1", e1), ("E2", e2), ("E3", e3["holds"]), ("E5", e5),
                           ("E6", e6["holds"])) if h]
    o_spell, o_tail, o_cur = spells[other]
    o_e5 = o_spell >= need - 1e-9
    o_held = [c for c, h in (("E1", e1), ("E2", e2), ("E3", e3["holds"]), ("E5", o_e5),
                             ("E6", e6["holds"])) if h]
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
            "e3": e3, "e6": e6, "wind_mps": spec.stress.wind_speed_mps,
            "episode_s": round(len(run.rows) * dt, 1),
            "fsm_triggered": fsm.get("failsafe_triggered"),
            "fsm_censored": censored,
            "fsm_edges": " > ".join(t["edge"] for t in fsm.get("transitions", []))}


LABELS_SCHEMA = "failsafe-labels/1"
LABELS_PATH = ROOT / "docs" / "data" / "failsafe_labels_rail.json"
# reading -> (vehicle, the flight E5 is read on)
READINGS = {"a": ("ideal", "no-failsafe"), "a2": ("ideal", "prescribed"),
            "r": ("rail", "no-failsafe"), "r2": ("rail", "prescribed")}
ADOPTED_READING = "r2"


def code_revision() -> str:
    """The commit the flights ran at, `-dirty` when a tracked file under
    guardrail/, experiments/, policies/ or tools/failsafe_rubric.py differs
    from it (what a label depends on)."""
    import subprocess

    def git(*a):
        return subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True,
                              text=True, check=False).stdout.strip()
    rev = git("rev-parse", "HEAD") or "unknown"
    dirty = git("status", "--porcelain", "--", "guardrail", "experiments", "policies",
                "tools/failsafe_rubric.py")
    return rev + ("-dirty" if dirty else "")


def _reading(rec: dict) -> dict:
    """One reading of one cell: expected True / False, or None (UNDECIDED:
    no clause holds and an E5 spell was still running when the episode
    ended - the family must fly longer, section 3)."""
    undecided = bool(rec["e5_censored"]) and not rec["derived"]
    return {"expected": None if undecided else bool(rec["derived"]),
            "clauses": list(rec["clauses_held"]), "e5_spell_s": rec["e5_spell_s"],
            "undecided": undecided}


def rail_extend_s() -> float:
    """How much longer than the scenario a rail flies (sweep_scenarios.
    rail_max_s: the Loiter operator timeout plus 5 s), so that a Loiter the
    lagging autopilot provokes shows its outcome. A rail run's fail-safe is
    read over that whole flight, so a label undecided at the scenario's
    length is read on the cell flown that much longer."""
    return float(SW.rail_max_s(0.0))


def _read_inputs(inputs: list[Path]) -> tuple[dict, dict, list, float | None]:
    """The readings in `--json` outputs of this tool, by build_labels' rules:
    (labels {cell: {seed: {reading: ...}}} read at the scenario's length,
    longer {(cell, seed, reading): ...} read on the rail's longer flight,
    the runs they came from, the E5 need). Refused: an input not on one
    profile and the ideal or the rail vehicle, not at one clean code
    revision, flown longer by anything but the rail's own rail_extend_s(),
    or a cell and seed two inputs read differently."""
    labels: dict[str, dict[str, dict]] = {}
    longer: dict[tuple, dict] = {}
    runs = []
    need = None
    ext = rail_extend_s()
    for path in inputs:
        rows = json.loads(Path(path).read_text(encoding="utf-8"))
        if not rows:
            raise ValueError(f"{path}: no cells")
        veh = {r["response"] for r in rows}
        prof = {r.get("profile") for r in rows}
        revs = {r.get("code_revision") for r in rows}
        exts = {float(r.get("extend_s") or 0.0) for r in rows}
        only = {tuple(r.get("only") or ()) for r in rows}
        if len(veh) != 1 or len(prof) != 1 or next(iter(veh)) not in ("ideal", "rail"):
            raise ValueError(f"{path}: one profile on the ideal or the rail vehicle, got "
                             f"{sorted(prof, key=str)} on {sorted(veh)}")
        if len(revs) != 1 or None in revs or next(iter(revs)).endswith("-dirty"):
            raise ValueError(f"{path}: one clean code revision per input, got "
                             f"{sorted(map(str, revs))}")
        veh, prof, x = next(iter(veh)), next(iter(prof)), next(iter(exts))
        if len(exts) != 1 or x not in (0.0, ext) or (x and veh != "rail"):
            raise ValueError(f"{path}: flown longer by {sorted(exts)} s on the {veh} vehicle; "
                             f"only the rail vehicle's own {ext:g} s resolves a label")
        need = rows[0]["e5_need_s"]
        sel = next(iter(only)) if len(only) == 1 else ()
        runs.append({"profile": prof, "vehicle": veh, "cell_seeds": len(rows),
                     "extend_s": x, "code_revision": next(iter(revs)),
                     "command": (f"python tools/failsafe_rubric.py"
                                 + ("" if prof == "library" else f" --profile {prof}")
                                 + (f" --only {' '.join(sel)}" if sel else "")
                                 + f" --response {veh} --no-fsm"
                                 + (f" --extend-s {x:g}" if x else "") + " --json <out>")})
        for r in rows:
            for rec in (r, r["other"]):
                key = next(k for k, v in READINGS.items() if v == (veh, rec["e5_flight"]))
                got = {**_reading(rec), "length": "rail" if x else "scenario"}
                if x:
                    was = longer.setdefault((r["episode"], str(r["seed"]), key), got)
                    if was != got:
                        raise ValueError(f"{r['episode']} seed {r['seed']} reading {key} flown "
                                         f"longer: {was} in one input, {got} in {path}")
                    continue
                slot = labels.setdefault(r["episode"], {}).setdefault(str(r["seed"]), {})
                if key in slot and slot[key] != got:
                    raise ValueError(f"{r['episode']} seed {r['seed']} reading {key}: "
                                     f"{slot[key]} in one input, {got} in {path}")
                slot[key] = got
    return labels, longer, runs, need


def _resolve_longer(labels: dict, longer: dict) -> tuple[int, list]:
    """The rail's longer flight applied to `labels` in place: a reading left
    undecided at the scenario's length takes the longer flight's decision; a
    decided one is never replaced, and a difference is listed. Returns
    (resolved, differ)."""
    resolved, differ = 0, []
    for (cell, seed, key), got in sorted(longer.items()):
        base = (labels.get(cell) or {}).get(seed, {}).get(key)
        if base is None:
            raise ValueError(f"{cell} seed {seed}: flown longer but not at the scenario's length")
        if base["expected"] is None:
            if got["expected"] is not None:
                labels[cell][seed][key] = got
                resolved += 1
        elif got["expected"] != base["expected"]:
            differ.append({"cell": cell, "seed": int(seed), "reading": key,
                           "scenario": base["expected"], "flown_longer": got["expected"],
                           "clauses_flown_longer": got["clauses"],
                           "e5_spell_s_flown_longer": got["e5_spell_s"]})
    return resolved, differ


def vehicle_record(vehicle: str) -> dict:
    """The vehicle a reading was derived on, as the artefact records it: its
    response (sweep_scenarios.response_for) and, since 2026-11-12, how a GPS
    denial reaches its Shield (sweep_scenarios.gps_dropout_for)."""
    return {**SW.response_for(vehicle),
            "gps_dropout": SW.gps_dropout_for(
                _rail_kw(vehicle).get("gps_dropout", SW.DEFAULT_GPS_DROPOUT))}


def build_labels(inputs: list[Path]) -> dict:
    """The label artefact from `--json` outputs of this tool (module
    docstring, THE LABEL ARTEFACT). Inputs are on the ideal or the rail
    vehicle, each at a clean code revision (named per run), at the scenario's
    length; a cell and seed judged twice (a seed two profiles share) must
    agree. An input on the rail vehicle flown rail_extend_s() longer RESOLVES
    the rail readings left undecided at the scenario's length (section 8:
    the rail flies that long, and its run's fail-safe is read over the whole
    flight): such a reading says `length` "rail" instead of "scenario". A
    reading already decided at the scenario's length is never replaced; one
    that the longer flight reads differently is listed (`built.
    differ_flown_longer`), not changed."""
    labels, longer, runs, need = _read_inputs(inputs)
    ext = rail_extend_s()
    short = [(c, s) for c, v in labels.items() for s, lab in v.items() if set(lab) != set(READINGS)]
    if short:
        raise ValueError(f"{len(short)} cell-seeds lack a reading (both vehicles are "
                         f"needed), e.g. {short[:3]}")
    resolved, differ = _resolve_longer(labels, longer)
    return {
        "schema": LABELS_SCHEMA,
        "what": ("Fail-safe labels of the scenario library's cells, derived by "
                 "tools/failsafe_rubric.py (docs/RUBRIC-failsafe.md) on the ideal and the "
                 "rail vehicle"),
        "readings": {
            "a": "ideal vehicle, E5 on the flight without a fail-safe (section 3)",
            "a2": "ideal vehicle, E5 on the prescribed flight (section 7; the templates' labels)",
            "r": "rail vehicle, E5 on the flight without a fail-safe (sections 3 and 8)",
            "r2": "rail vehicle, E5 on the prescribed flight (sections 7 and 8)"},
        "adopted_kpi_reading": ADOPTED_READING,
        "adopted": ("2026-11-10, the user's decision of 9-10 November (\"follow the best "
                    "practice that meets the KPI criteria\"), after the rig had shown the "
                    "autopilot's lag (rh01, 2026-11-07): not a pre-registered reading; "
                    "(a) and (r) are printed beside it"),
        "length": ("scenario: read at the scenario's own length. rail: undecided there (no "
                   "clause holds and an E5 spell was still running at its end) and read on "
                   f"the cell flown {ext:g} s longer, as a rail flies it (sweep_scenarios."
                   "rail_max_s; section 8)"),
        "undecided": ("expected null: undecided at the scenario's length and not resolved "
                      "by the longer flight (or none was flown); not guessed"),
        "built": {"e5_need_s": need, "rail_extend_s": ext,
                  "fsm_config_hash": F.FSMConfig().digest, "runs": runs,
                  "code_revisions": sorted({r["code_revision"] for r in runs}),
                  "resolved_flown_longer": resolved, "differ_flown_longer": differ,
                  "then": "python tools/failsafe_rubric.py --build-labels <this file> <out> ..."},
        "vehicles": {"ideal": vehicle_record("ideal"), "rail": vehicle_record("rail")},
        "labels": {c: {s: labels[c][s] for s in sorted(labels[c])} for c in sorted(labels)},
    }


def update_labels(doc: dict, inputs: list[Path], why: str) -> dict:
    """`doc` (a built artefact) with the readings in `inputs` RE-DERIVED
    (2026-11-12): every reading the inputs give, for a cell and seed the
    artefact already holds, replaces the one there; every other reading stays
    as it was. The inputs follow build_labels' rules (one clean code revision
    each, the scenario's length or the rail's own longer flight, one answer
    per reading), and a longer flight resolves only what was re-derived with
    it. For a change that bears on some cells only - how the rail realises a
    stressor - without re-flying the whole library. Recorded in
    `built.updates`: why, the runs (commands and revisions) and every reading
    it moved, before and after; the revisions join `built.code_revisions`,
    and `vehicles` becomes the current record. Returns a new document."""
    if not str(why).strip():
        raise ValueError("an update says why it was made")
    new, longer, runs, need = _read_inputs(inputs)
    if not new:
        raise ValueError("an update needs readings at the scenario's length")
    if need != doc["built"]["e5_need_s"]:
        raise ValueError(f"E5 needs {need} s in the inputs, {doc['built']['e5_need_s']} s in "
                         f"the artefact: not the same rubric")
    for cell, seeds in new.items():
        for seed in seeds:
            if seed not in (doc["labels"].get(cell) or {}):
                raise ValueError(f"{cell} seed {seed}: not in the artefact; an update "
                                 f"re-derives cells, it does not add them")
    resolved, differ = _resolve_longer(new, longer)
    out = json.loads(json.dumps(doc))
    moved, same = [], 0
    for cell in sorted(new):
        for seed in sorted(new[cell]):
            for key in sorted(new[cell][seed]):
                was, now = out["labels"][cell][seed].get(key), new[cell][seed][key]
                if was == now:
                    same += 1
                    continue
                out["labels"][cell][seed][key] = now
                moved.append({"cell": cell, "seed": int(seed), "reading": key,
                              "was": None if was is None else
                              {k: was[k] for k in ("expected", "clauses", "e5_spell_s")},
                              "now": {k: now[k] for k in ("expected", "clauses", "e5_spell_s")}})
    b = out["built"]
    b.setdefault("updates", []).append({
        "why": str(why), "runs": runs, "cell_seeds": sum(len(v) for v in new.values()),
        "readings_moved": moved, "readings_unchanged": same,
        "resolved_flown_longer": resolved, "differ_flown_longer": differ,
        "fsm_config_hash": F.FSMConfig().digest,
        "then": ("python tools/failsafe_rubric.py --update-labels <this file> <in> ... "
                 "--why <reason>")})
    b["code_revisions"] = sorted(set(b["code_revisions"]) | {r["code_revision"] for r in runs})
    out["vehicles"] = {"ideal": vehicle_record("ideal"), "rail": vehicle_record("rail")}
    return out


def write_labels(path: Path, doc: dict) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(doc, indent=1, sort_keys=False) + "\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--profile", default=None)
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--json", default=None, help="write every cell's facts here")
    ap.add_argument("--no-fsm", action="store_true",
                    help="skip the flight WITH the FSM (printed for the reader only)")
    ap.add_argument("--e5-flight", choices=E5_FLIGHTS, default=E5_FLIGHT,
                    help="the flight E5 is read on: prescribed (adopted 2026-11-10, the "
                         "default) or no-failsafe (section 3's reading)")
    ap.add_argument("--response", choices=SW.RESPONSE_ARMS, default=SW.DEFAULT_RESPONSE_ARM,
                    help="the vehicle every flight is flown on: ideal (the headless "
                         "labels), rail (the rig's own fitted response: the KPI-grade "
                         "labels, adopted 2026-11-10) or ardupilot (the Copter-4.5.7 fit)")
    ap.add_argument("--build-labels", nargs="+", default=None, metavar=("OUT", "IN"),
                    help="build the label artefact OUT from this tool's --json outputs "
                         "IN ... (one per profile and vehicle) and stop")
    ap.add_argument("--update-labels", nargs="+", default=None, metavar=("ART", "IN"),
                    help="re-derive, in the artefact ART, the readings this tool's --json "
                         "outputs IN ... give (cells already in it; --why says why) and stop")
    ap.add_argument("--why", default="", help="the reason an --update-labels is made")
    ap.add_argument("--extend-s", type=float, default=0.0,
                    help="fly every episode this many seconds longer than it declares")
    args = ap.parse_args(argv)
    if args.update_labels is not None:
        if len(args.update_labels) < 2:
            ap.error("--update-labels needs ART and at least one IN.json")
        art = Path(args.update_labels[0])
        doc = update_labels(json.loads(art.read_text(encoding="utf-8")),
                            [Path(x) for x in args.update_labels[1:]], args.why)
        write_labels(art, doc)
        u = doc["built"]["updates"][-1]
        print(f"{u['cell_seeds']} cell-seeds re-derived; {len(u['readings_moved'])} reading(s) "
              f"moved, {u['readings_unchanged']} unchanged -> {art}")
        for m in u["readings_moved"]:
            was = m["was"] or {}
            print(f"  {m['cell'][:70]:70} s{m['seed']:<5} {m['reading']:3} "
                  f"{was.get('expected')!s:5} {','.join(was.get('clauses') or []) or '-':7} -> "
                  f"{m['now']['expected']!s:5} {','.join(m['now']['clauses']) or '-'}")
        return 0
    if args.build_labels is not None:
        if len(args.build_labels) < 2:
            ap.error("--build-labels needs OUT and at least one IN.json")
        doc = build_labels([Path(x) for x in args.build_labels[1:]])
        write_labels(Path(args.build_labels[0]), doc)
        n = sum(len(v) for v in doc["labels"].values())
        und = sum(1 for v in doc["labels"].values() for lab in v.values()
                  if lab["r2"]["expected"] is None)
        b = doc["built"]
        print(f"{n} cell-seeds; under r2 {und} undecided, {b['resolved_flown_longer']} "
              f"reading(s) resolved on the flight as long as the rail flies, "
              f"{len(b['differ_flown_longer'])} decided reading(s) that flight reads "
              f"otherwise (listed, not changed) -> {args.build_labels[0]}")
        return 0
    prof = SW._resolve_profile(args.profile)
    lib = Library(ROOT / (prof.library if prof else "experiments/scenarios.yaml"))
    eps, _ = expand(lib, prof, None, args.only)
    eps = [e for e in eps if not e.comparison and e.prefix == "on"
           and e.paraphrase_arm in (None, -1)
           and e.cell.spec.shield and e.cell.spec.escalation]
    timeout = F.FSMConfig().loiter_timeout_s
    revision = code_revision()
    print(f"vehicle {args.response}, E5 on the {args.e5_flight} flight"
          + (f", every episode {args.extend_s:g} s longer" if args.extend_s else ""))
    out, bad, flips = [], [], []
    for e in eps:
        j = judge(e, lib, timeout, with_fsm=not args.no_fsm, e5_flight=args.e5_flight,
                  response=args.response, extend_s=args.extend_s)
        j["profile"], j["code_revision"] = args.profile or "library", revision
        if args.only:
            j["only"] = list(args.only)
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
              f"E6 {j['e6']['longest_run']}/{j['e6']['need']} "
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
