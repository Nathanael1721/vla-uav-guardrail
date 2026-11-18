"""The dev rails hand the Shield a BODY-frame action (WP3-04 / ARCH-04, 2026-11-18).

Run either way:
    pytest tests/test_rail_body_frame.py -v
    python tests/test_rail_body_frame.py

The grant: "Action input frame is body frame (vx, vy, vz, yaw_rate);
body->local-NED conversion happens after projection, in the MAVLink adapter."
The Shield takes the body frame at its own boundary since 2026-11-04
(`Shield.filter_body`), and the ROS 2 rail hands it the body action as it
arrives. The four dev rails whose producers speak the world frame - the city
follow rail (demo/follow_vlm.py), pymavlink SITL (sitl/run_sitl_demo.py), PAS
ArduPilot (demo/pas_ardupilot/perception_node.py, --action-frame world) and
the headless harness (experiments/sweep_scenarios.py, action_frame "world") -
handed it `Shield.filter(state, world_action)`. They now turn the world action
into the body frame at their boundary, at the heading the Shield itself turns
it back with, and hand that (guardrail.frames.filter_as_body: filter_body with
the arc OFF). They fly the Shield's emitted world action, as before.

Pinned here, each against the old call (`Shield.filter` on the world action)
run side by side on the same inputs:
  * the decision is the same to 1e-9 in every emitted channel, with the same
    rules broken, the same repair operators, the same brake / FSM fields -
    over seeded sequences at headings of exactly +-180 degrees and within
    1e-7 / 1e-12 degrees of them, and at a heading of 0 exactly (==; a
    turn at 0 can change only a zero's sign);
  * a world action with a non-finite horizontal channel is handed as it is
    (no body form keeps its finite channel) and decided to the bit;
  * the arc forecast stays off on these rails even on a Shield whose own
    `arc_forecast` default is on;
  * the headless harness: all 53 library scenarios, closed loop, at their own
    heading (to the bit where it is 0), and 14 of them turned to 179.9999999
    and -180 degrees - every row's position, raw and emitted action within
    1e-9, the same repairs, violations and FSM edges, and the same breaches /
    P0 / goal verdicts;
  * PAS ArduPilot: the fake ArduPilot flight (tests/test_pas_ardupilot.py
    _fly) at headings pi, -pi + 1e-9 and 0.6 rad, three policies - every
    action sent to the link within 1e-9, the same rows otherwise;
  * pymavlink SITL and the city follow rail: their Shield gets no heading
    (State.yaw_deg 0), so the turns are the identity (==); the call
    sites are pinned, and their stored flights (demo/out/, when present -
    gitignored, so a clean clone SKIPs; GUARDRAIL_DEMO_OUT points at another
    checkout's) are replayed through both calls.
"""
import json
import math
import os
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.frames import body_action, filter_as_body, from_body  # noqa: E402
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.replay import rail_shield                             # noqa: E402
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S, Shield     # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
TOL = 1e-9
SKIP = "SKIP"

# The headings the rotations are least kind at: +-180 exactly, and just
# inside it from both sides, where sin(psi) is a tiny number of either sign.
EDGE_HEADINGS = [180.0, -180.0, 179.9999999, -179.9999999, 180.0 - 1e-12,
                 -180.0 + 1e-12, 90.0, -90.0, 37.0, 0.0]


def _old(shield, state, world, t=None, **kw):
    """The call every dev rail made until 2026-11-18."""
    return shield.filter(state, world, t, **kw)


def _ids(d):
    return ([(v.rule_id, v.category) for v in d.violations],
            [r.operator for r in d.repairs])


def _diff(a: Action4D, b: Action4D) -> float:
    return max(abs(a.vx - b.vx), abs(a.vy - b.vy), abs(a.vz_up - b.vz_up),
               abs(a.yaw_rate - b.yaw_rate))


def _same(d_old, d_new, where) -> float:
    """The same decision to TOL: returns the largest channel difference."""
    worst = max(_diff(d_old.emitted, d_new.emitted), _diff(d_old.raw, d_new.raw))
    assert worst <= TOL, (where, worst, d_old.emitted, d_new.emitted)
    assert _ids(d_old) == _ids(d_new), (where, _ids(d_old), _ids(d_new))
    for f in ("braked", "touched", "fsm_state_before", "fsm_state_after", "fsm_edge",
              "set_mode", "fsm_fault"):
        assert getattr(d_old, f) == getattr(d_new, f), (where, f)
    if d_old.command is not None or d_new.command is not None:
        assert _diff(d_old.command, d_new.command) <= TOL, where
    return worst


def _walk(rng, heading, n=30):
    """A seeded sequence of (state, world action) near nfz-square (x, y in
    [7, 23]), fast enough to break the speed and yaw-rate caps."""
    x, y = rng.uniform(-5.0, 30.0), rng.uniform(-5.0, 30.0)
    out = []
    for _ in range(n):
        st = State(x=x, y=y, up=rng.uniform(4.0, 30.0), yaw_deg=heading)
        a = Action4D(vx=rng.uniform(-6.0, 6.0), vy=rng.uniform(-6.0, 6.0),
                     vz_up=rng.uniform(-3.0, 3.0), yaw_rate=rng.uniform(-1.5, 1.5))
        out.append((st, a))
        x, y = x + 0.1 * a.vx, y + 0.1 * a.vy
    return out


def _shields(kind):
    pol = load_policy(POLICY)
    if kind == "rail":            # PAS ArduPilot, pymavlink SITL
        return (rail_shield(pol, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S),
                rail_shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S))
    if kind == "follow":          # the city follow rail's flight_shield, no maps
        from guardrail.replay import FOLLOW_RAIL_PATH_REPAIR as pr
        return Shield(pol, path_repair=pr), Shield(load_policy(POLICY), path_repair=pr)
    return Shield(pol, lookahead_s=3.0, dt=0.5), Shield(load_policy(POLICY), lookahead_s=3.0, dt=0.5)


# --- the boundary itself ----------------------------------------------------

def test_the_body_action_is_to_body_and_turns_back():
    rng = random.Random(20261118)
    for _ in range(200):
        h = rng.choice(EDGE_HEADINGS + [rng.uniform(-180.0, 180.0)])
        a = Action4D(vx=rng.uniform(-8, 8), vy=rng.uniform(-8, 8),
                     vz_up=rng.uniform(-3, 3), yaw_rate=rng.uniform(-2, 2))
        b = body_action(a, h)
        back = from_body(b.vx, b.vy, b.vz_up, b.yaw_rate, h)
        assert _diff(a, back) <= 1e-12, (h, a, back)
        assert b.vz_up == a.vz_up and b.yaw_rate == a.yaw_rate


def test_the_rails_shield_decides_as_filter_did_at_every_heading():
    """Seeded sequences through each rail's kind of Shield (rail_shield: PAS,
    SITL; the follow rail's PathRepair Shield; a Shield with its own FSM),
    the old call and the new side by side, so the sliding window and the
    FSM see the same history. Not blind: repairs fire on most ticks."""
    rng = random.Random(20261118)
    worst = 0.0
    n = touched = 0
    headings = EDGE_HEADINGS + [rng.uniform(-180.0, 180.0) for _ in range(10)]
    for kind in ("rail", "follow", "fsm"):
        for h in headings:
            old, new = _shields(kind)
            for k, (st, a) in enumerate(_walk(rng, h)):
                d_old = _old(old, st, a, t=0.1 * k)
                d_new = filter_as_body(new, st, a, t=0.1 * k)
                worst = max(worst, _same(d_old, d_new, (kind, h, k)))
                n += 1
                touched += int(d_old.touched)
    assert touched > n // 2, (touched, n)
    print(f"      seeded: {n} ticks, {touched} repaired, worst |emitted diff| {worst:.3g}")


def test_a_heading_of_zero_is_filter_to_the_bit():
    """The city follow and pymavlink SITL rails give the Shield no heading
    (State.yaw_deg 0): there the two turns are the identity, so the decision
    is filter()'s exactly (==), not merely to rounding. The one thing a turn
    at heading 0 can change is the sign of a zero (-0.0 + 0.0 is +0.0), which
    == does not see; the zero-speed ticks below include them."""
    rng = random.Random(4)
    zeros = [Action4D(vx=-0.0, vy=-0.0), Action4D(vx=1.0, vy=-0.0),
             Action4D(vx=-0.0, vy=-2.0, yaw_rate=-0.0), Action4D(vx=-3.0, vy=0.0)]
    for kind in ("rail", "follow"):
        old, new = _shields(kind)
        seq = _walk(rng, 0.0, n=60)
        seq += [(State(x=15.0, y=2.0, up=12.0), z) for z in zeros]
        for k, (st, a) in enumerate(seq):
            d_old, d_new = _old(old, st, a), filter_as_body(new, st, a)
            assert d_old.emitted == d_new.emitted and d_old.raw == d_new.raw, (kind, k)
            assert _ids(d_old) == _ids(d_new), (kind, k)


def test_a_nonfinite_horizontal_channel_is_handed_as_it_is():
    """A NaN North speed turned into the body frame would come back NaN North
    AND East, and the Shield would zero both: such a tick goes to filter() as
    it is, and the finite East channel is judged as it always was."""
    for h in (0.0, 37.0, 180.0, -179.9999999):
        for bad in (Action4D(vx=float("nan"), vy=1.5), Action4D(vx=2.0, vy=float("inf")),
                    Action4D(vx=float("-inf"), vy=float("nan"), vz_up=0.5)):
            st = State(x=-20.0, y=-20.0, up=15.0, yaw_deg=h)
            old, new = _shields("rail")
            d_old, d_new = _old(old, st, bad), filter_as_body(new, st, bad)
            assert d_old.emitted == d_new.emitted, (h, bad, d_old.emitted, d_new.emitted)
            assert _ids(d_old) == _ids(d_new)
            assert any(v.category == "contract" for v in d_new.violations), d_new.violations
    # The heading itself non-finite: handed as it is too.
    st = State(x=-20.0, y=-20.0, up=15.0, yaw_deg=float("nan"))
    old, new = _shields("rail")
    a = Action4D(vx=1.0, vy=0.5)
    assert _old(old, st, a).emitted == filter_as_body(new, st, a).emitted


def test_the_arc_stays_off_on_the_rails():
    """filter() never arcs; neither does the rails' body hand-over, even on a
    Shield built with arc_forecast on - which no dev rail builds."""
    pol = load_policy(POLICY)
    st = State(x=10.0, y=0.0, up=15.0, yaw_deg=0.0)
    turn = Action4D(vx=4.0, vy=0.0, yaw_rate=0.4)   # its arc reaches nfz-square
    arced = Shield(pol, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S, arc_forecast=True)
    d, _ = arced.filter_body(st, turn)
    assert (d.trace or {}).get("forecast_arc") is True and d.violations, \
        "the probe does not reach the zone on the arc: it proves nothing"
    for h in (0.0, 180.0):
        st_h = st.model_copy(update={"yaw_deg": h})
        world = from_body(4.0, 0.0, 0.0, 0.4, h)
        a = Shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S,
                   arc_forecast=True)
        b = Shield(load_policy(POLICY), lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
        d_new, d_old = filter_as_body(a, st_h, world), _old(b, st_h, world)
        assert "forecast_arc" not in (d_new.trace or {}), h
        _same(d_old, d_new, h)


# --- the call sites -----------------------------------------------------------

RAIL_CALLS = {
    "demo/follow_vlm.py": ["d = filter_as_body(shield, state, flown_in)",
                           "dec = filter_as_body(shield, State(x=p[\"x\"], y=p[\"y\"], up=up),"],
    "sitl/run_sitl_demo.py": ["decision = filter_as_body(shield, st_sh, raw)"],
    "demo/pas_ardupilot/perception_node.py": [
        "d, emitted_body = filter_as_body(shield, st_sh, raw), None"],
    "experiments/sweep_scenarios.py": ["d = filter_as_body(shield, seen, raw, t=t_now, **flags)"],
}


def test_every_dev_rail_hands_the_shield_the_body_frame():
    """No dev rail calls Shield.filter on its world action any more; each
    goes through frames.filter_as_body (or, on the opt-in body arms,
    Shield.filter_body itself)."""
    import ast
    for rel, calls in RAIL_CALLS.items():
        src = (ROOT / rel).read_text(encoding="utf-8")
        for c in calls:
            assert c in src, (rel, c)
        left = []
        for node in ast.walk(ast.parse(src)):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "filter" and isinstance(node.func.value, ast.Name)
                    and node.func.value.id in ("shield", "self.shield")):
                left.append(node.lineno)
        assert not left, f"{rel} still calls shield.filter() at lines {left}"
    # The two rails whose Shield is given no heading (the bit-exact case).
    sitl = (ROOT / "sitl" / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert "return State(x=newest.x, y=newest.y, up=-newest.z)" in sitl
    follow = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    assert 'state = State(x=p["x"], y=p["y"], up=-p["z"])' in follow


# --- the headless harness, closed loop ---------------------------------------

def _harness():
    if str(ROOT / "experiments") not in sys.path:
        sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    return S, Library(ROOT / "experiments" / "scenarios.yaml")


def _row_names(items):
    return [i.get("rule_id") or i.get("operator") if isinstance(i, dict) else str(i)
            for i in (items or [])]


# Turned to +-180 degrees: the scenarios whose repairs act in the most
# directions (zones met head-on and at angles, a spinning and a translating
# zone, corridors, stand-offs, a zone opening beside the pilot, three rules at
# once, wind, GPS noise, the fail-safes, a non-finite command).
TURNED = ["nfz-head-on", "nonfinite-action", "corridor-drift-angles", "standoff-approach-angles",
          "standoff-wedge", "dyn-nfz-translate", "dyn-nfz-rotate-spin",
          "time-window-opens-on-sideways-pilot", "three-simultaneous-violations",
          "failsafe-rtl-zone", "high-speed-near-edge", "gps-noise-beyond-margin",
          "wind-gusts", "distance-envelope-lanes"]


def test_the_headless_harness_flies_every_scenario_as_before():
    """run_episode (action_frame "world", the default) closed loop, once with
    the old call patched back in and once as it now is: every row and every
    verdict the same, positions and actions within 1e-9. All 53 library
    scenarios at their own heading (to the bit where it is 0), and 14 of
    them turned to 179.9999999 and -180 degrees (the headless vehicle does
    not yaw, so the start heading is every tick's)."""
    S, lib = _harness()
    worst = 0.0
    n_rows = n_rep = n_eps = 0
    real = S.filter_as_body
    by_id = lib.by_id()
    assert set(TURNED) <= set(by_id), set(TURNED) - set(by_id)
    for sid, entry in by_id.items():
        for h in ((None, 179.9999999, -180.0) if sid in TURNED else (None,)):
            spec = entry.first.spec.model_copy(deep=True)
            m = spec.mission
            if h is not None:
                # The heading flown is the start's plus the mission's
                # rotation (_resolve_mission): this one is h.
                m.start_pose.yaw_deg = h - m.rotate_deg
            own_zero = h is None and m.start_pose.yaw_deg + m.rotate_deg == 0.0
            try:
                S.filter_as_body = _old
                w = S.run_episode(spec, lib.defaults, 1001)
            finally:
                S.filter_as_body = real
            b = S.run_episode(spec, lib.defaults, 1001)
            assert len(w.rows) == len(b.rows), (sid, h, len(w.rows), len(b.rows))
            if own_zero:
                assert (json.dumps(w.rows, sort_keys=True, default=str)
                        == json.dumps(b.rows, sort_keys=True, default=str)), \
                    f"{sid}: a heading of 0 is not the identity"
            for rw, rb in zip(w.rows, b.rows):
                for k in ("x", "y", "up"):
                    worst = max(worst, abs(rw[k] - rb[k]))
                for k in ("raw", "emitted"):
                    if isinstance(rw.get(k), dict):
                        worst = max(worst, max(abs(rw[k][c] - rb[k][c]) for c in rw[k]
                                               if isinstance(rw[k][c], (int, float))
                                               and math.isfinite(rw[k][c])))
                for k in ("violations", "repairs", "emitted_violations"):
                    assert _row_names(rw.get(k)) == _row_names(rb.get(k)), (sid, h, rw["t"], k)
                for k in ("braked", "fsm_edge", "fsm_state_after", "set_mode", "setpoint",
                          "owner", "unsafe", "stop_illegal"):
                    assert rw.get(k) == rb.get(k), (sid, h, rw["t"], k)
                n_rep += int(bool(rw.get("repairs")))
            assert worst <= TOL, (sid, h, worst)
            for k in ("breaches", "polygon_entries", "true_p0_flown_ticks", "reached_goal",
                      "ended_safe", "rtl_failed_ticks", "all_actions_finite"):
                assert w.extra.get(k) == b.extra.get(k), (sid, h, k)
            n_rows += len(w.rows)
            n_eps += 1
    assert n_rep > 1000, f"the Shield repaired {n_rep} rows: blind"
    print(f"      harness: {n_eps} episodes, {n_rows} rows, {n_rep} repaired, "
          f"worst diff {worst:.3g}")


# --- PAS ArduPilot, the fake ArduPilot flight ---------------------------------

def _pas():
    for p in (ROOT / "demo" / "pas_ardupilot", ROOT / "demo", ROOT / "tests"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import perception_node as pn
    import test_pas_ardupilot as T
    return pn, T


def test_the_pas_rail_flies_as_before_at_headings_of_plus_minus_pi():
    """perception_node.run on the fake ArduPilot (--action-frame world, the
    default), the old call patched back in against the new, at headings of
    pi, -pi + 1e-9 and 0.6 rad, on three policies (a speed cap that repairs,
    a zone ahead that repairs, the same zone that brakes): the same rows,
    every flown action within 1e-9."""
    pn, T = _pas()
    brake = T.NFZ_AHEAD_POLICY.replace("violation_action: repair", "violation_action: brake")
    real = pn.filter_as_body
    worst = 0.0
    n = touched = 0
    for pol in (T.CAP_POLICY, T.NFZ_AHEAD_POLICY, brake):
        for yaw in (math.pi, -math.pi + 1e-9, 0.6):
            try:
                pn.filter_as_body = _old
                rc_o, run_o, link_o = T._fly("on", policy=pol, link_kw={"yaw": yaw})
            finally:
                pn.filter_as_body = real
            rc_n, run_n, link_n = T._fly("on", policy=pol, link_kw={"yaw": yaw})
            assert rc_o == rc_n == 0, (rc_o, rc_n)
            # What the link was sent, at full precision (the rows round x, y).
            assert len(link_o.sent) == len(link_n.sent) > 0
            for a, b in zip(link_o.sent, link_n.sent):
                worst = max(worst, _diff(a, b))
            ro, rn = T._rows(run_o), T._rows(run_n)
            assert len(ro) == len(rn) > 0, (len(ro), len(rn))
            for a, b in zip(ro, rn):
                for k in ("raw", "emitted", "flown_action"):
                    if a.get(k) is None:
                        assert b.get(k) is None, k
                        continue
                    worst = max(worst, max(abs(a[k][c] - b[k][c]) for c in a[k]))
                for k in ("x", "y", "up"):
                    worst = max(worst, abs(a[k] - b[k]))
                for k in ("violations", "repairs", "emitted_violations"):
                    assert _row_names(a.get(k)) == _row_names(b.get(k)), (yaw, a["t"], k)
                for k in ("touched", "braked", "flown", "setpoint", "fsm_setpoint",
                          "fsm_state_after", "fsm_edge", "set_mode", "stop_advised"):
                    assert a.get(k) == b.get(k), (yaw, a["t"], k)
                n += 1
                touched += int(bool(a.get("touched")))
            assert worst <= TOL, (yaw, worst)
    assert touched > 0, "the Shield never acted: blind"
    print(f"      PAS fake flight: {n} rows, {touched} touched, worst diff {worst:.3g}")


# --- stored flights (demo/out/ is gitignored) ---------------------------------

STORED = {
    # rail: (run folders, the heading the rail gave its Shield, the action key)
    "follow": (["citylife_redcar_final4", "citylife_redcar_nfz_20261013"], "zero", ("smooth", "raw")),
    "sitl": (["sitl_shield_on", "sitl_shield_on_dynamic", "sitl_ped_on"], "zero", ("raw",)),
    "pas": (["pas_ap_close_on", "pas_ap_ovf_on_20261021", "pas_ap_video_on_20261014"],
            "row", ("raw",)),
}


def _demo_out() -> Path:
    return Path(os.environ.get("GUARDRAIL_DEMO_OUT") or ROOT / "demo" / "out")


def _policy_of(run: Path):
    """The run's first generation as guardrail.replay reads it (a stored
    policy of an older schema era included), else the demo policy."""
    from guardrail.replay import _generation_files, generation_table
    try:
        _, policies, problems = generation_table(_generation_files(run))
    except Exception:                                       # noqa: BLE001
        policies, problems = {}, ["unreadable"]
    if policies and not problems:
        return policies[min(policies)], "own"
    return load_policy(POLICY), "demo"


def test_stored_flights_replay_through_both_calls():
    """Each rail's stored flight log (positions, the Shield's heading as the
    rail gave it, the action the Shield was handed) replayed open loop through
    the old call and the new, side by side, on the run's own first-generation
    policy where it is stored (else the demo policy)."""
    out = _demo_out()
    found = own = 0
    worst = 0.0
    n = touched = 0
    for rail, (runs, heading, keys) in STORED.items():
        for name in runs:
            log = out / name / "flight_log.jsonl"
            if not log.is_file():
                continue
            found += 1
            (pol_a, src), (pol_b, _) = _policy_of(out / name), _policy_of(out / name)
            own += int(src == "own")
            old = rail_shield(pol_a, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
            new = rail_shield(pol_b, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S)
            for line in log.read_text(encoding="utf-8").splitlines():
                r = json.loads(line)
                act = next((r[k] for k in keys if isinstance(r.get(k), dict)), None)
                if act is None or not all(k in r for k in ("x", "y", "up")):
                    continue
                yaw = float(r.get("yaw_deg") or 0.0) if heading == "row" else 0.0
                st = State(x=float(r["x"]), y=float(r["y"]), up=float(r["up"]), yaw_deg=yaw)
                a = Action4D(**{c: float(act[c]) for c in ("vx", "vy", "vz_up", "yaw_rate")})
                d_old, d_new = _old(old, st, a), filter_as_body(new, st, a)
                worst = max(worst, _same(d_old, d_new, (name, r.get("t"))))
                if heading == "zero":
                    assert d_old.emitted == d_new.emitted, (name, r.get("t"))
                n += 1
                touched += int(d_old.touched)
    if not found:
        return SKIP
    print(f"      stored: {found} flights ({own} on their own stored policy), {n} ticks, "
          f"{touched} repaired, worst diff {worst:.3g} (from {out})")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {str(e)[:2000]}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
