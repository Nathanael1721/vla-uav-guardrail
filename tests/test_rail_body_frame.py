"""The dev rails hand the Shield a BODY-frame action where that is exact
(WP3-04 / ARCH-04, 2026-11-18; made exact after the review of cf17291).

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
handed it `Shield.filter(state, world_action)`. They now go through
guardrail.frames.filter_as_body: the world action turned into the body frame
at the heading the Shield turns it back with, and handed over with
filter_body (the arc OFF) on every tick where the Shield's from_body gives
the world action back bit for bit (`body_handover`); any other tick is handed
to Shield.filter as it is. They fly the Shield's emitted world action, as
before.

The heading: PAS ArduPilot gives the Shield the aircraft's measured heading;
the harness the episode's heading (the headless vehicle does not yaw); the
city follow and pymavlink SITL rails none (State.yaw_deg 0), so there the
body action is the world action relabelled, not the aircraft's body frame.

Pinned here, each against the old call (`Shield.filter` on the world action)
run side by side on the same inputs, TO THE BIT (struct-packed doubles, so a
zero's sign counts), and never blind - each check counts the ticks handed in
the body frame and the ticks whose round trip was inexact and went to filter:
  * seeded sequences through each rail's kind of Shield at +-180, just inside
    it, +-90, 37, 0 and ten random headings;
  * a non-finite horizontal channel or heading goes to filter as it is;
  * the arc forecast stays off on these rails even on a Shield whose own
    `arc_forecast` default is on;
  * the call sites: no flown `.filter(` left on the four rails (every
    `.filter(` call checked by its receiver against an allowlist of the
    harness's judging Shields);
  * the headless harness, closed loop, seed 1001: every library cell flown
    at a non-zero heading, the first cell of every scenario, and 14
    scenarios' first cells turned to 37 degrees - every row equal as JSON
    and every verdict equal;
  * PAS ArduPilot: the fake ArduPilot flight (tests/test_pas_ardupilot.py
    _fly) at headings pi, -pi + 1e-9 and 0.6 rad, three policies, the
    pilot's body action given a lateral channel so that some ticks' round
    trips are inexact - every flight_log.jsonl row equal as JSON but for its
    wall-clock fields, and every action sent to the link to the bit;
  * stored flights (demo/out/, when present - gitignored, so a clean clone
    and CI SKIP; GUARDRAIL_DEMO_OUT points at another checkout's) replayed
    through both calls on guardrail.replay.rail_shield (the follow flights
    too: not the follow rail's own flight_shield).
"""
import ast
import json
import math
import os
import random
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.frames import (body_action, body_handover,              # noqa: E402
                              filter_as_body, from_body)
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.replay import rail_shield                             # noqa: E402
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S, Shield     # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
SKIP = "SKIP"

# The headings the rotations are least kind at: +-180 exactly, and just
# inside it from both sides, where sin(psi) is a tiny number of either sign;
# and 37, where cos and sin are both inexact and most round trips are too.
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


def _pack(a: Action4D) -> bytes:
    return struct.pack("<4d", a.vx, a.vy, a.vz_up, a.yaw_rate)


def _same(d_old, d_new, where) -> None:
    """The same decision TO THE BIT (raw, emitted, command), the same rules,
    repairs and FSM fields."""
    assert _pack(d_old.emitted) == _pack(d_new.emitted), (where, d_old.emitted, d_new.emitted)
    assert _pack(d_old.raw) == _pack(d_new.raw), (where, d_old.raw, d_new.raw)
    assert _ids(d_old) == _ids(d_new), (where, _ids(d_old), _ids(d_new))
    for f in ("braked", "touched", "fsm_state_before", "fsm_state_after", "fsm_edge",
              "set_mode", "fsm_fault"):
        assert getattr(d_old, f) == getattr(d_new, f), (where, f)
    if d_old.command is not None or d_new.command is not None:
        assert _pack(d_old.command) == _pack(d_new.command), where


class _Count:
    """Ticks handed over in the BODY frame vs handed to filter() as they are."""

    def __init__(self):
        self.body = self.world = 0

    def see(self, world: Action4D, yaw_deg: float) -> None:
        if body_handover(world, yaw_deg) is None:
            self.world += 1
        else:
            self.body += 1

    def spy(self, real):
        def wrapped(shield, state, world, t=None, **kw):
            self.see(world, state.yaw_deg)
            return real(shield, state, world, t, **kw)
        return wrapped

    def __str__(self):
        return f"{self.body} handed in the body frame, {self.world} to filter()"


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


def test_the_body_action_is_handed_over_only_where_it_turns_back_exactly():
    """body_handover returns the body action exactly when from_body (what
    Shield.filter_body computes) gives the world action back bit for bit,
    else None. Both happen at headings like 37 degrees, so the check is not
    blind. A zero whose sign the turn changes goes to filter() too."""
    rng = random.Random(37)
    c = _Count()
    for _ in range(500):
        h = rng.choice([37.0, rng.uniform(-180.0, 180.0)])
        a = Action4D(vx=rng.uniform(-8, 8), vy=rng.uniform(-8, 8),
                     vz_up=rng.uniform(-3, 3), yaw_rate=rng.uniform(-2, 2))
        b = body_action(a, h)
        exact = _pack(from_body(b.vx, b.vy, b.vz_up, b.yaw_rate, h)) == _pack(a)
        got = body_handover(a, h)
        if exact:
            assert got is not None and _pack(got) == _pack(b), (h, a)
        else:
            assert got is None, (h, a)
        c.see(a, h)
    assert c.body > 0 and c.world > 0, str(c)
    # At heading 0 a -0.0 East speed comes back +0.0 (1 * 0.0 + -0.0 * 1).
    z = Action4D(vx=1.0, vy=-0.0)
    b = body_action(z, 0.0)
    assert _pack(from_body(b.vx, b.vy, b.vz_up, b.yaw_rate, 0.0)) != _pack(z)
    assert body_handover(z, 0.0) is None
    print(f"      boundary: 500 seeded actions, {c}")


def test_the_rails_shield_decides_as_filter_did_at_every_heading():
    """Seeded sequences through each rail's kind of Shield (rail_shield: PAS,
    SITL; the follow rail's PathRepair Shield; a Shield with its own FSM),
    the old call and the new side by side, so the sliding window and the
    FSM see the same history: to the bit. Not blind: repairs fire on most
    ticks, and ticks go both ways (body hand-over, filter())."""
    rng = random.Random(20261118)
    n = touched = 0
    c = _Count()
    headings = EDGE_HEADINGS + [rng.uniform(-180.0, 180.0) for _ in range(10)]
    for kind in ("rail", "follow", "fsm"):
        for h in headings:
            old, new = _shields(kind)
            for k, (st, a) in enumerate(_walk(rng, h)):
                d_old = _old(old, st, a, t=0.1 * k)
                d_new = filter_as_body(new, st, a, t=0.1 * k)
                _same(d_old, d_new, (kind, h, k))
                c.see(a, h)
                n += 1
                touched += int(d_old.touched)
    assert touched > n // 2, (touched, n)
    assert c.body > n // 4 and c.world > 0, str(c)
    print(f"      seeded: {n} ticks, {touched} repaired, all to the bit; {c}")


def test_a_heading_of_zero_is_filter_to_the_bit():
    """The city follow and pymavlink SITL rails give the Shield no heading
    (State.yaw_deg 0): there the two turns are the identity except for the
    sign of a zero (-0.0 + 0.0 is +0.0); such a tick goes to filter() as it
    is, so the decision is filter()'s to the bit, signed zeros included."""
    rng = random.Random(4)
    zeros = [Action4D(vx=-0.0, vy=-0.0), Action4D(vx=1.0, vy=-0.0),
             Action4D(vx=-0.0, vy=-2.0, yaw_rate=-0.0), Action4D(vx=-3.0, vy=0.0)]
    for kind in ("rail", "follow"):
        old, new = _shields(kind)
        seq = _walk(rng, 0.0, n=60)
        seq += [(State(x=15.0, y=2.0, up=12.0), z) for z in zeros]
        for k, (st, a) in enumerate(seq):
            d_old, d_new = _old(old, st, a), filter_as_body(new, st, a)
            _same(d_old, d_new, (kind, k))


def test_a_nonfinite_horizontal_channel_is_handed_as_it_is():
    """A NaN North speed turned into the body frame would come back NaN North
    AND East, and the Shield would zero both: such a tick goes to filter() as
    it is, and the finite East channel is judged as it always was."""
    for h in (0.0, 37.0, 180.0, -179.9999999):
        for bad in (Action4D(vx=float("nan"), vy=1.5), Action4D(vx=2.0, vy=float("inf")),
                    Action4D(vx=float("-inf"), vy=float("nan"), vz_up=0.5)):
            st = State(x=-20.0, y=-20.0, up=15.0, yaw_deg=h)
            old, new = _shields("rail")
            assert body_handover(bad, h) is None, (h, bad)
            d_old, d_new = _old(old, st, bad), filter_as_body(new, st, bad)
            _same(d_old, d_new, (h, bad))
            assert any(v.category == "contract" for v in d_new.violations), d_new.violations
    # The heading itself non-finite: handed as it is too.
    st = State(x=-20.0, y=-20.0, up=15.0, yaw_deg=float("nan"))
    old, new = _shields("rail")
    a = Action4D(vx=1.0, vy=0.5)
    assert body_handover(a, st.yaw_deg) is None
    _same(_old(old, st, a), filter_as_body(new, st, a), "nan heading")


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
        assert body_handover(world, h) is not None, "the probe must take the body path"
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

# The only `.filter(` calls left on the four rails, by receiver: the harness's
# judging Shields (what was flown, re-judged; nothing they return is flown).
# Any other receiver - `shield`, `self.shield`, `sh`, ... - fails.
JUDGES = {"experiments/sweep_scenarios.py": {"checker", "oracle", "vjudge"}}


def _filter_calls(src: str) -> list:
    """(line, receiver) of every `<receiver>.filter(...)` call."""
    return [(n.lineno, ast.unparse(n.func.value)) for n in ast.walk(ast.parse(src))
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == "filter"]


def test_every_dev_rail_hands_the_shield_the_body_frame():
    """No dev rail calls Shield.filter on its world action any more; each
    goes through frames.filter_as_body (or, on the opt-in body arms,
    Shield.filter_body itself). Every `.filter(` call is listed with its
    receiver, attribute chains included, against an allowlist."""
    probe = "self.shield.filter(s, a)\nsh.filter(s, a)\nshield.filter(s, a)\n"
    assert [r for _, r in sorted(_filter_calls(probe))] == ["self.shield", "sh", "shield"]
    for rel, calls in RAIL_CALLS.items():
        src = (ROOT / rel).read_text(encoding="utf-8")
        for c in calls:
            assert c in src, (rel, c)
        left = [(ln, r) for ln, r in _filter_calls(src) if r not in JUDGES.get(rel, set())]
        assert not left, f"{rel} still calls .filter() at {left}"
    # The two rails whose Shield is given no heading.
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


# Turned to 37 degrees: the scenarios whose repairs act in the most
# directions (zones met head-on and at angles, a spinning and a translating
# zone, corridors, stand-offs, a zone opening beside the pilot, three rules at
# once, wind, GPS noise, the fail-safes, a non-finite command).
TURNED = ["nfz-head-on", "nonfinite-action", "corridor-drift-angles", "standoff-approach-angles",
          "standoff-wedge", "dyn-nfz-translate", "dyn-nfz-rotate-spin",
          "time-window-opens-on-sideways-pilot", "three-simultaneous-violations",
          "failsafe-rtl-zone", "high-speed-near-edge", "gps-noise-beyond-margin",
          "wind-gusts", "distance-envelope-lanes"]


def _flown_heading(spec) -> float:
    """The heading flown is the start's plus the mission's rotation
    (_resolve_mission); the headless vehicle does not yaw."""
    m = spec.mission
    return m.start_pose.yaw_deg + m.rotate_deg


def test_the_headless_harness_flies_every_scenario_as_before():
    """run_episode (action_frame "world", the default) closed loop at seed
    1001, once with the old call patched back in and once as it now is:
    every row equal as JSON (sorted keys) and every verdict the same. Flown:
    every library cell at a non-zero heading, the first cell of every
    scenario, and 14 scenarios' first cells turned to 37 degrees, where most
    round trips are inexact (at +-180 they came out exact: the seeded test
    covers those headings)."""
    S, lib = _harness()
    n_rows = n_rep = n_eps = n_cells = n_cells_nonzero = 0
    c = _Count()
    real = S.filter_as_body
    by_id = lib.by_id()
    assert set(TURNED) <= set(by_id), set(TURNED) - set(by_id)
    runs = []
    for sid, fam in by_id.items():
        cells = fam.cells()
        n_cells += len(cells)
        for k, cell in enumerate(cells):
            nonzero = _flown_heading(cell.spec) != 0.0
            n_cells_nonzero += int(nonzero)
            if k == 0 or nonzero:
                runs.append((cell.episode_id, cell.spec, None))
        if sid in TURNED:
            for h in (37.0,):
                runs.append((fam.first.episode_id, fam.first.spec, h))
    for eid, spec0, h in runs:
        spec = spec0.model_copy(deep=True)
        if h is not None:
            spec.mission.start_pose.yaw_deg = h - spec.mission.rotate_deg
        try:
            S.filter_as_body = _old
            w = S.run_episode(spec, lib.defaults, 1001)
            S.filter_as_body = c.spy(real)
            b = S.run_episode(spec, lib.defaults, 1001)
        finally:
            S.filter_as_body = real
        assert len(w.rows) == len(b.rows), (eid, h, len(w.rows), len(b.rows))
        for rw, rb in zip(w.rows, b.rows):
            assert (json.dumps(rw, sort_keys=True, default=str)
                    == json.dumps(rb, sort_keys=True, default=str)), (eid, h, rw.get("t"))
            n_rep += int(bool(rw.get("repairs")))
        for k in ("breaches", "polygon_entries", "true_p0_flown_ticks", "reached_goal",
                  "ended_safe", "rtl_failed_ticks", "all_actions_finite"):
            assert w.extra.get(k) == b.extra.get(k), (eid, h, k)
        n_rows += len(w.rows)
        n_eps += 1
    assert n_rep > 1000, f"the Shield repaired {n_rep} rows: blind"
    assert c.body > 0 and c.world > 0, f"not exercised both ways: {c}"
    print(f"      harness: {n_eps} episodes ({n_cells} library cells, {n_cells_nonzero} at "
          f"a non-zero heading, all flown), {n_rows} rows equal as JSON, {n_rep} "
          f"repaired; {c}")


# --- PAS ArduPilot, the fake ArduPilot flight ---------------------------------

def _pas():
    for p in (ROOT / "demo" / "pas_ardupilot", ROOT / "demo", ROOT / "tests"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    import perception_node as pn
    import test_pas_ardupilot as T
    return pn, T


# Row fields that read the wall clock: they differ between any two runs of
# the same flight, old call against old call too (rule_clock to the second).
WALL_CLOCK = ("shield_ms", "episode_id", "rule_clock")


def test_the_pas_rail_flies_as_before_at_headings_of_plus_minus_pi():
    """perception_node.run on the fake ArduPilot (--action-frame world, the
    default), the old call patched back in against the new, at headings of
    pi, -pi + 1e-9 and 0.6 rad, on three policies (a speed cap that repairs,
    a zone ahead that repairs, the same zone that brakes): every
    flight_log.jsonl row equal as JSON but for its wall-clock fields
    (shield_ms, episode_id, rule_clock), and every action sent to the link to the bit. The fake
    pilot flies the same forward-only action on every tick, whose round trip
    came out exact (the review of cf17291), so the follow servo's body action
    is given a lateral channel that changes every tick, in both arms alike:
    then some ticks' round trips are inexact and go to filter(), which this
    asserts."""
    pn, T = _pas()
    brake = T.NFZ_AHEAD_POLICY.replace("violation_action: repair", "violation_action: brake")
    real = pn.filter_as_body
    real_step = pn.FollowPilot.step

    def sideways(self, now, up):
        # A lateral channel that changes every tick (the fake box does not
        # move, so the servo's action is the same on every tick).
        body, info = real_step(self, now, up)
        side = body[0] * (0.37 + 0.2 * math.sin(7.0 * now)) + 0.11
        return (body[0], body[1] + side, body[2], body[3]), info

    n = touched = 0
    c = _Count()
    pn.FollowPilot.step = sideways
    try:
        for pol in (T.CAP_POLICY, T.NFZ_AHEAD_POLICY, brake):
            for yaw in (math.pi, -math.pi + 1e-9, 0.6):
                try:
                    pn.filter_as_body = _old
                    rc_o, run_o, link_o = T._fly("on", policy=pol, link_kw={"yaw": yaw})
                    pn.filter_as_body = c.spy(real)
                    rc_n, run_n, link_n = T._fly("on", policy=pol, link_kw={"yaw": yaw})
                finally:
                    pn.filter_as_body = real
                assert rc_o == rc_n == 0, (rc_o, rc_n)
                assert len(link_o.sent) == len(link_n.sent) > 0
                for a, b in zip(link_o.sent, link_n.sent):
                    assert _pack(a) == _pack(b), (yaw, a, b)
                rows, rows_n = T._rows(run_o), T._rows(run_n)
                assert len(rows) == len(rows_n) > 0
                for a, b in zip(rows, rows_n):
                    # Every field as written (JSON text), but the two that
                    # read the wall clock and differ between any two runs.
                    a, b = ({k: v for k, v in r.items() if k not in WALL_CLOCK}
                            for r in (a, b))
                    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True), \
                        (yaw, a.get("t"))
                n += len(rows)
                touched += sum(int(bool(r.get("touched"))) for r in rows)
    finally:
        pn.FollowPilot.step = real_step
    assert touched > 0, "the Shield never acted: blind"
    assert c.body > 0 and c.world > 0, f"not exercised both ways: {c}"
    print(f"      PAS fake flight: {n} rows equal as JSON, {touched} touched; {c}")


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
    the old call and the new, side by side, to the bit, on the run's own
    first-generation policy where it is stored (else the demo policy), on
    guardrail.replay.rail_shield for every rail (the follow flights too, not
    the follow rail's own flight_shield with its maps). Local only: demo/out
    is gitignored, so a clean clone and CI SKIP."""
    out = _demo_out()
    found = own = 0
    n = touched = 0
    c = _Count()
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
                a = Action4D(**{c_: float(act[c_]) for c_ in ("vx", "vy", "vz_up", "yaw_rate")})
                d_old, d_new = _old(old, st, a), filter_as_body(new, st, a)
                _same(d_old, d_new, (name, r.get("t")))
                c.see(a, yaw)
                n += 1
                touched += int(d_old.touched)
    if not found:
        return SKIP
    print(f"      stored: {found} flights ({own} on their own stored policy), {n} ticks, "
          f"{touched} repaired, all to the bit; {c} (rail_shield, from {out})")


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
