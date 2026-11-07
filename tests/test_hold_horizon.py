"""A Loiter's hold legal for as long as it is held, and a hand-back that does
not fly into a sweep (2026-11-07; docs/DESIGN-hold-points.md, "A hold legal
for as long as it is held"; guardrail/hold_flight.py for how the rails fly it).

dyn-nfz-rotate-spin on the lagging vehicles (the harness's ArduPilot response
and the rig's own `rail` arm) went home by X6, a false fail-safe by the
rubric's N4: the hold-point search judged a point through one lookahead from
arrival, so under the spinning bar each point was legal for barely that long,
the hold was re-planned every 0.5-1.4 s ahead of the bar and the aircraft
never stood anywhere a stop was legal for T.

Run:  python tests/test_hold_horizon.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from guardrail import holdpoint as HP  # noqa: E402
from guardrail import load_policy  # noqa: E402
from guardrail.fsm import (HOLD_HORIZON_CAP_S, EscalationFSM, FSMConfig,  # noqa: E402
                           FSMState, HoldPoint, RuleHit, TickInput)
from guardrail.hold_flight import REACHED_M, HoldFlight, transit_velocity  # noqa: E402
from guardrail.models import Action4D, DynamicNFZ, State  # noqa: E402
from guardrail.shield import Shield  # noqa: E402

HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}
DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
S = FSMState
HERE = State(x=0.0, y=0.0, up=15.0)


def _slow_bar(half_len: float = 3.0, speed: float = 1.0, x0: float = 12.0) -> Shield:
    """The demo policy, the aircraft at (0, 0, 15), and a 2 m-wide DynamicNFZ
    bar at x0..x0+2, `half_len` m either side of y = 0, closing from the east
    at `speed` m/s (margin 0.5): a stop here is legal through the 5 s
    lookahead, and the bar sweeps over it about 11 s from now."""
    bar = DynamicNFZ(id="bar", type="dynamic_nfz", **HARD_P0,
                     vertices=[{"x": x0, "y": -half_len}, {"x": x0 + 2, "y": -half_len},
                               {"x": x0 + 2, "y": half_len}, {"x": x0, "y": half_len}],
                     altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
                     motion={"vx_mps": -speed, "vy_mps": 0.0})
    sh = Shield(load_policy(DEMO), escalation=False)
    sh.spawn_nfz(bar, t=0.0)
    sh.filter(HERE, Action4D(), t=0.0)
    return sh


def _legal_through(sh: Shield, p: HoldPoint, until_s: float) -> bool:
    """A stop at the plan's point is legal in every lookahead-long view from
    its arrival to `until_s` from now (and one lookahead past it)."""
    q = State(x=p.x, y=p.y, up=p.up)
    t = p.travel_s
    while t <= until_s + 1e-9:
        with sh.looking_ahead(t):
            if sh.hard_violations(sh.state_is_unsafe(q)):
                return False
        t += sh.lookahead_s
    return True


# --------------------------------------------------------------------------- #
# The search (guardrail/holdpoint.py choose_hold_point, hold_s)
# --------------------------------------------------------------------------- #

def test_a_hold_legal_for_one_lookahead_only_is_moved_out_of_the_sweep():
    sh = _slow_bar(half_len=1.5)
    assert not sh.hard_violations(sh.state_is_unsafe(HERE)), "here must be legal now"
    assert HP.time_varying(sh)
    before = HP.choose_hold_point(sh, HERE)                      # the lookahead law
    assert (before.kind, before.legal, before.checked_s) == ("in_place", True, 5.0), before
    assert not _legal_through(sh, before, 30.0), "the bar must sweep the spot in place"
    p = HP.choose_hold_point(sh, HERE, hold_s=30.0)
    assert (p.kind, p.legal) == ("moved", True), p
    assert p.checked_s >= 35.0 - 1e-9, p.checked_s
    assert _legal_through(sh, p, 30.0), p
    assert abs(p.y) > 1.5, f"the point must leave the bar's swept strip: {p}"
    assert "for as long as the Loiter can last" in p.reason, p.reason
    # Its HOLD_MARGIN_M neighbours stay legal too.
    for n in HP._neighbours(State(x=p.x, y=p.y, up=p.up)):
        assert _legal_through(sh, HoldPoint("LOITER", "moved", n.x, n.y, n.up, legal=True,
                                            travel_s=p.travel_s, speed_mps=4.0), 30.0)


def test_a_hold_that_stays_legal_is_kept_in_place_and_says_how_long():
    sh = _slow_bar(half_len=3.0, x0=60.0)                        # 60 m out: never in time
    p = HP.choose_hold_point(sh, HERE, hold_s=30.0)
    assert (p.kind, p.legal) == ("in_place", True), p
    assert p.checked_s >= 35.0 - 1e-9 and "for as long as" in p.reason, p


def test_where_nothing_stays_legal_the_lookahead_plan_is_kept():
    """The budget spent before a point legal for the span is found: the plan
    the lookahead alone gives (here: in place), never a worse one."""
    sh = _slow_bar()
    keep = HP.MAX_QUERIES
    HP.MAX_QUERIES = 12
    try:
        p = HP.choose_hold_point(sh, HERE, hold_s=30.0)
    finally:
        HP.MAX_QUERIES = keep
    assert (p.kind, p.legal, p.checked_s) == ("in_place", True, 5.0), p
    assert "the lookahead's plan" in p.reason, p.reason


def test_a_world_that_cannot_change_plans_exactly_as_before():
    sh = Shield(load_policy(DEMO), escalation=False)
    assert not HP.time_varying(sh)
    st = State(x=6.5, y=15.0, up=15.0)                           # in nfz-square's ring
    assert sh.hard_violations(sh.state_is_unsafe(st))
    assert HP.choose_hold_point(sh, st, hold_s=30.0) == HP.choose_hold_point(sh, st)


def test_the_search_off_ignores_the_horizon():
    sh = _slow_bar()
    assert HP.choose_hold_point(sh, HERE, search=False, hold_s=30.0) == \
        HP.choose_hold_point(sh, HERE, search=False)


# --------------------------------------------------------------------------- #
# The FSM (guardrail/fsm.py: hold_horizon, hold_horizon_s, handback_unsafe)
# --------------------------------------------------------------------------- #

def _hit():
    return RuleHit("nfz", "project_fix", "hard", "P0")


def _to_loiter(f: EscalationFSM, t0: float = 0.0):
    f.step(TickInput(t=t0, outcome="blocked", violations=(_hit(),)))
    o = f.step(TickInput(t=t0 + 0.1, outcome="blocked", violations=(_hit(),)))
    assert o.state is S.LOITER, o.reason
    return o


def test_the_horizon_is_the_loiters_remaining_time():
    f = EscalationFSM()
    assert f.hold_horizon_s() is None                            # not in Loiter
    _to_loiter(f)
    assert abs(f.hold_horizon_s() - 30.0) < 1e-9
    f.hold_at(HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True))
    f.step(TickInput(t=10.1, outcome="clean"))
    assert abs(f.hold_horizon_s() - 20.0) < 1e-9, f.hold_horizon_s()
    assert EscalationFSM(FSMConfig(hold_horizon="lookahead")).hold_horizon_s() is None
    g = EscalationFSM(FSMConfig(hold_horizon="lookahead"))
    _to_loiter(g)
    assert g.hold_horizon_s() is None
    h = EscalationFSM(FSMConfig(loiter_timeout_s=None))
    _to_loiter(h)
    assert h.hold_horizon_s() == HOLD_HORIZON_CAP_S


def test_loiter_does_not_hand_back_a_flight_into_a_stop_illegal_spot():
    """Rotate-spin at 9 deg/s on the ideal vehicle, the hold legal for the
    whole Loiter: handed back at 8.4 s onto a clean request straight through
    the bar's disc, the harness counted a breach. Loiter keeps such a request
    (TickInput.handback_unsafe) and hands back once its flight is clear; the
    lookahead law ignores the input, as every rail did."""
    for law, g8 in (("loiter", 4.2), ("lookahead", 2.2)):
        f = EscalationFSM(FSMConfig(hold_horizon=law))
        _to_loiter(f)
        f.hold_at(HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True))
        edges = []
        t = 0.2
        while t < 6.0 and not edges:
            o = f.step(TickInput(t=round(t, 1), outcome="clean",
                                 handback_unsafe=t < 2.15))
            if o.edge:
                edges.append((round(t, 1), o.edge))
            if law == "loiter" and t < 2.15:
                assert o.record.get("handback_unsafe") is True, o.record
            else:
                assert "handback_unsafe" not in o.record, o.record
            t += 0.1
        assert edges == [(g8, "G8")], (law, edges)


# --------------------------------------------------------------------------- #
# How a rail flies it (guardrail/hold_flight.py)
# --------------------------------------------------------------------------- #

class _FakeFSM:
    def __init__(self, state=S.LOITER, hold=None, terminal=False):
        self.state, self.hold, self.terminal = state, hold, terminal


def _moved(x=10.0, y=0.0, up=15.0, t_s=0.0, travel=2.5, mode="LOITER"):
    return HoldPoint(mode, "moved", x, y, up, legal=True, travel_s=travel,
                     speed_mps=4.0, climb_mps=2.0, t_s=t_s)


def test_an_in_place_hold_is_sent_as_the_fsm_asks():
    hf = HoldFlight()
    f = _FakeFSM(hold=HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True, t_s=0.0))
    assert hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1).mode == "LOITER"
    nxt = hf.tick(f, None, 0.0, 0.0, 15.0, 0.1, 0.1)
    assert (nxt.mode, nxt.command, nxt.transit) == (None, None, False)
    # Outside Loiter / Land the FSM's request passes through untouched.
    out = hf.tick(_FakeFSM(state=S.NORMAL), "GUIDED", 0.0, 0.0, 15.0, 0.2, 0.1)
    assert (out.mode, out.command) == ("GUIDED", None)


def test_a_moved_hold_is_a_guided_transit_then_the_hold():
    hf = HoldFlight()
    plan = _moved()
    f = _FakeFSM(hold=plan)
    first = hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1)
    assert first.mode is None and first.transit and first.command[0] > 0.0, first
    assert abs(first.command[1]) < 1e-9 and "GUIDED transit" in first.note
    assert hf.transits == 1
    # Never faster than the plan's speed, and slowing onto the point.
    far = hf.tick(f, None, 2.0, 0.0, 15.0, 0.5, 0.1)
    near = hf.tick(f, None, 9.0, 0.0, 15.0, 2.0, 0.1)
    assert far.command[0] <= 4.0 + 1e-9 and 0.0 < near.command[0] < far.command[0]
    done = hf.tick(f, None, 10.0 - REACHED_M / 2, 0.0, 15.0, 2.4, 0.1)
    assert (done.mode, done.command, done.transit) == ("LOITER", None, False), done
    assert hf.tick(f, None, 10.0, 0.0, 15.0, 2.5, 0.1).mode is None
    # A new point while the autopilot holds: GUIDED again, then the hold.
    f.hold = _moved(x=20.0, t_s=3.0)
    again = hf.tick(f, None, 10.0, 0.0, 15.0, 3.0, 0.1)
    assert again.mode == "GUIDED" and again.transit and hf.transits == 2, again
    # The plan back in place during a transit: hold where it is, at once.
    f.hold = HoldPoint("LOITER", "in_place", 12.0, 0.0, 15.0, legal=True, t_s=3.5)
    back = hf.tick(f, None, 12.0, 0.0, 15.0, 3.5, 0.1)
    assert (back.mode, back.command) == ("LOITER", None), back


def test_a_transit_not_reached_in_time_holds_where_it_is():
    hf = HoldFlight()
    f = _FakeFSM(hold=_moved(travel=1.0))
    assert hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1).transit
    late = hf.tick(f, None, 3.0, 0.0, 15.0, 2.1, 0.1)              # due at 1.0 + 1.0 s
    assert (late.mode, late.command) == ("LOITER", None) and "not reached" in late.note


def test_a_moved_landing_site_is_reached_level_then_landed():
    hf = HoldFlight()
    site = _moved(x=5.0, up=0.0, mode="LAND")
    f = _FakeFSM(state=S.LAND, hold=site)
    go = hf.tick(f, "LAND", 0.0, 0.0, 15.0, 0.0, 0.1)
    assert go.mode is None and go.transit and go.command[2] == 0.0, go
    land = hf.tick(f, None, 5.0, 0.0, 15.0, 1.5, 0.1)
    assert (land.mode, land.command) == ("LAND", None), land


def test_the_transit_law_is_the_harness_rail_arms():
    """The sqrt controller of the remaining distance, capped at the plan's
    speed and climb (experiments/sweep_scenarios.py `_autopilot` under a
    vehicle response)."""
    p = _moved(x=10.0, y=10.0, up=17.0)
    vx, vy, vz = transit_velocity(0.0, 0.0, 15.0, p, 0.1)
    assert abs(vx - vy) < 1e-9 and (vx * vx + vy * vy) ** 0.5 <= 4.0 + 1e-9
    assert 0.0 < vz <= 2.0
    assert transit_velocity(10.0, 10.0, 17.0, p, 0.1) == (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# The cells (headless harness): dyn-nfz-rotate-spin
# --------------------------------------------------------------------------- #

def _spin(arm: str, deg_per_s: int, hold: str = "moved_points", ext_s: float = 0.0):
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    cell = next(c for c in lib.by_id()["dyn-nfz-rotate-spin"].cells()
                if c.params.get("deg_per_s") == deg_per_s)
    spec = cell.spec
    if ext_s:
        spec = spec.model_copy(update={"ticks": int(spec.ticks or lib.defaults.ticks)
                                       + int(round(ext_s / float(spec.dt or lib.defaults.dt)))})
    run = SW.run_episode(spec, lib.defaults, 0, response=arm, autopilot_hold=hold)
    sc = SW.score(spec, run.rows, run.extra, run.policy, run.priorities)
    return sc, [t["edge"] for t in (run.extra.get("fsm") or {}).get("transitions", [])]


def test_rotate_spin_on_the_rails_vehicle_is_no_longer_sent_home():
    """The cells' label is N4 on every vehicle. On the rig's own response
    with the hold flown as the rails now fly it, X6 sent all three home
    (rh01_017 flew it at 3 deg/s: X6 at 30.3 s); now none is."""
    for dps in (3, 6, 9):
        sc, edges = _spin("rail", dps)
        assert not sc["failsafe_triggered"] and "X6" not in edges, (dps, edges)
        assert (sc["p0_escapes"], sc["breaches"]) == (0, 0), (dps, sc)


def test_rotate_spin_on_the_ideal_vehicle_is_not_handed_back_into_the_bar():
    sc, edges = _spin("ideal", 9, ext_s=35.0)
    assert not sc["failsafe_triggered"] and sc["breaches"] == 0, (sc, edges)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
