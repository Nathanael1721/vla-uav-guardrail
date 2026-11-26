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
import math
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
from wgs84_fixture import geo, geo_ring              # noqa: E402

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
    sh.spawn_nfz(geo(bar, sh), t=0.0)
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
    keep, keep_ray = HP.MAX_QUERIES, HP.RETREAT_QUERIES
    HP.MAX_QUERIES, HP.RETREAT_QUERIES = 12, 0      # the retreat ray finds nothing
    try:
        p = HP.choose_hold_point(sh, HERE, hold_s=30.0)
    finally:
        HP.MAX_QUERIES, HP.RETREAT_QUERIES = keep, keep_ray
    assert (p.kind, p.legal, p.checked_s) == ("in_place", True, 5.0), p
    assert "the lookahead's plan" in p.reason, p.reason


def test_a_spent_budget_moves_along_the_retreat_ray_to_a_point_held_for_the_span():
    """THE RETREAT RAY (2026-11-21): the same spent budget, the ray's units
    left in: a point the hold horizon's own test accepts, legal for the span."""
    sh = _slow_bar()
    keep = HP.MAX_QUERIES
    HP.MAX_QUERIES = 12
    try:
        p = HP.choose_hold_point(sh, HERE, hold_s=30.0)
    finally:
        HP.MAX_QUERIES = keep
    assert (p.kind, p.legal) == ("moved", True), p
    assert "retreat ray" in p.reason, p.reason
    assert _legal_through(sh, p, 30.0), p


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


def test_loiter_does_not_hand_back_a_request_untrusted_at_speed_from_its_point():
    """2026-11-10, the hil smoke sm1110_010 (corridor-drift-angles 8 m/s,
    -60 deg): the hold sat 0.3 m short of its point, the unchanged request's
    repair read 1.72-1.78 m there (theta 2.0) for T_recover, and Loiter handed
    it back 14 times; its label expects the operator timeout. Loiter keeps a
    request whose repair, judged from the hold point at the speed it asks for,
    is untrusted (TickInput.handback_untrusted): held to the timeout while it
    stays so, handed back once it is not; the "as_held" law ignores the
    input, as the FSM did until then."""
    for law, edge in (("at_speed", (30.1, "G5")), ("as_held", (2.2, "G8"))):
        f = EscalationFSM(FSMConfig(handback_request=law))
        _to_loiter(f)
        f.hold_at(HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True))
        edges = []
        t = 0.2
        while t < 31.0 and not edges:
            # This tick's repair reads within theta from the held aircraft.
            o = f.step(TickInput(t=round(t, 1), outcome="clean", handback_unsafe=False,
                                 handback_untrusted=True))
            if o.edge:
                edges.append((round(t, 1), o.edge))
            elif law == "at_speed":
                assert o.record.get("handback_untrusted") is True, o.record
            else:
                assert "handback_untrusted" not in o.record, o.record
            t += 0.1
        assert edges == [edge], (law, edges)
    f = EscalationFSM()
    _to_loiter(f)
    f.hold_at(HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True))
    got, t = [], 0.2
    while t < 8.0 and not got:
        o = f.step(TickInput(t=round(t, 1), outcome="clean", handback_unsafe=False,
                             handback_untrusted=t < 3.15))
        if o.edge:
            got.append((round(t, 1), o.edge))
        t += 0.1
    assert got == [(5.2, "G8")], got                    # T_recover after it cleared
    raises = None
    try:
        TickInput(t=0.0, outcome="clean", handback_untrusted="yes")
    except ValueError as e:
        raises = e
    assert raises is not None
    try:
        FSMConfig(handback_request="sometimes")
        raise AssertionError("an unknown handback_request was accepted")
    except ValueError:
        pass


def test_untrusted_repair_is_g3s_own_test():
    """EscalationFSM.untrusted_repair reads a tick as step() does for G3 -
    over theta or not converged, of a hard projected rule - and moves
    nothing."""
    from guardrail.fsm import RepairMagnitude
    f = EscalationFSM()
    big = RepairMagnitude(lateral_m=2.5, vertical_m=0.0)
    small = RepairMagnitude(lateral_m=1.5, vertical_m=0.0)
    before = (f.state, f._n, f._t_last)
    assert f.untrusted_repair(TickInput(t=0.0, outcome="repaired", violations=(_hit(),),
                                        magnitude=big))
    assert not f.untrusted_repair(TickInput(t=0.0, outcome="repaired", violations=(_hit(),),
                                            magnitude=small))
    assert f.untrusted_repair(TickInput(t=0.0, outcome="blocked", violations=(_hit(),)))
    assert not f.untrusted_repair(TickInput(t=0.0, outcome="clean"))
    soft = RuleHit("soft", "project_fix", "soft", "P2")
    assert not f.untrusted_repair(TickInput(t=0.0, outcome="blocked", violations=(soft,)))
    assert (f.state, f._n, f._t_last) == before
    # ... and step() agrees: from Brake, the untrusted one is G3.
    for mag, edge in ((big, "G3"), (small, None)):
        g = EscalationFSM()
        g.step(TickInput(t=0.0, outcome="repaired", violations=(_hit(),), magnitude=small))
        inp = TickInput(t=0.1, outcome="repaired", violations=(_hit(),), magnitude=mag)
        want = g.untrusted_repair(inp)
        assert (g.step(inp).edge == "G3") == want == (edge == "G3"), (mag, edge)


def test_the_request_is_judged_from_the_hold_point_at_the_speed_it_asks_for():
    """Shield.handback_untrusted on the demo policy: 4 m/s north at nfz-square
    is a GeofenceSlide of 8.8 m (theta 2.0) from 13 m short of it, and clean
    from 35 m short - judged where the Loiter holds (`at`), whatever the
    aircraft's own state is, and with nothing moved."""
    sh = Shield(load_policy(DEMO))
    raw = Action4D(vx=4.0, vy=0.0, vz_up=0.0)
    near, far = State(x=2.0, y=15.0, up=15.0), State(x=-20.0, y=15.0, up=15.0)
    assert sh.handback_untrusted(raw, near) is True
    assert sh.handback_untrusted(raw, far) is False
    assert sh.handback_untrusted(Action4D(), near) is False          # a hover asks nothing
    # A rail's own FSM judges by its own theta, on its own basis: the same
    # slide is 8.8 m by its own magnitude and under theta on the one-period
    # reading (|dv| over 0.1 s), which a rail may select.
    wide = EscalationFSM(FSMConfig(theta_lateral_m=20.0))
    assert sh.handback_untrusted(raw, near, fsm=wide) is False
    assert sh.handback_untrusted(raw, near, horizon_s=None) is True
    assert sh.handback_untrusted(raw, near, horizon_s=0.1) is False
    assert sh.fsm.state is S.NORMAL and sh._t_last is None and not len(sh.history)


def test_the_drift_that_churned_on_the_rig_holds_to_its_timeout():
    """corridor-drift-angles 8 m/s, -60 deg on the rig's vehicle: at 9eed35b
    Loiter handed the unchanged request back twice headless (G8 at 18.6 s and
    24.5 s) and 14 times on the rig (sm1110_010), where the label expects the
    operator timeout. Judged from its hold point at speed it is not handed
    back: G3, then G5 thirty seconds later, inside the episode."""
    import sweep_scenarios as SW
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    cell = next(c for c in lib.by_id()["corridor-drift-angles"].cells()
                if c.params.get("ask_mps") == 8 and c.params.get("drift_deg") == -60)
    run = SW.run_episode(cell.spec, lib.defaults, 0, response="rail",
                         autopilot_hold=SW.RAIL_VEHICLE["autopilot_hold"])
    sc = SW.score(cell.spec, run.rows, run.extra, run.policy, run.priorities)
    tr = [(t["edge"], t["t_s"]) for t in (run.extra.get("fsm") or {}).get("transitions", [])]
    edges = [e for e, _ in tr]
    assert "G8" not in edges and edges[:3] == ["G1", "G3", "G5"], tr
    assert abs((tr[2][1] - tr[1][1]) - 30.0) < 0.11, tr
    assert sc["failsafe_triggered"] and not sc.get("failsafe_censored"), sc
    assert (sc["p0_escapes"], sc["breaches"]) == (0, 0), sc


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
    hf = HoldFlight(guided_hold=False)
    f = _FakeFSM(hold=HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True, t_s=0.0))
    assert hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1).mode == "LOITER"
    nxt = hf.tick(f, None, 0.0, 0.0, 15.0, 0.1, 0.1)
    assert (nxt.mode, nxt.command, nxt.transit) == (None, None, False)
    # Outside Loiter / Land the FSM's request passes through untouched.
    out = hf.tick(_FakeFSM(state=S.NORMAL), "GUIDED", 0.0, 0.0, 15.0, 0.2, 0.1)
    assert (out.mode, out.command) == ("GUIDED", None)


def test_a_moved_hold_is_a_guided_transit_then_the_hold():
    hf = HoldFlight(guided_hold=False)
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


def test_a_loiter_is_held_in_guided_on_its_point():
    """Since 2026-11-07 night (guardrail/hold_flight.py, THE MODE MAPPING): the
    FSM's LOITER is not sent; the companion streams the transit law toward the
    hold's point every tick - where the Loiter began until a plan arrives, an
    in-place plan's own point, a moved point and then on it. BRAKE only when
    the stream is forced off (force_brake), once, for the rest of the Loiter."""
    hf = HoldFlight()
    f = _FakeFSM(hold=None)
    entry = hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1)
    assert entry.mode is None and entry.command is not None, entry
    assert "held in GUIDED" in entry.note and hf.guiding, entry
    # Carried 1 m east by the hand-over: streamed back west, still in GUIDED.
    back = hf.tick(f, None, 1.0, 0.0, 15.0, 0.1, 0.1)
    assert back.mode is None and back.command[0] < 0.0, back
    # An in-place plan holds ITS point, not wherever the aircraft has got to.
    f.hold = HoldPoint("LOITER", "in_place", -0.5, 0.0, 15.0, legal=True, t_s=0.2)
    on = hf.tick(f, None, 1.0, 0.0, 15.0, 0.2, 0.1)
    assert on.mode is None and on.command[0] < 0.0 and "in_place point" in on.note, on
    # A moved point: flown to, then held on it, never handed to the autopilot.
    f.hold = _moved(x=10.0, t_s=0.3)
    go = hf.tick(f, None, 0.0, 0.0, 15.0, 0.3, 0.1)
    assert go.transit and go.command[0] > 0.0 and hf.transits == 1, go
    at = hf.tick(f, None, 10.0 - REACHED_M / 2, 0.0, 15.0, 3.0, 0.1)
    assert at.mode is None and not at.transit and 0.0 < at.command[0] < 0.5, at
    assert "reached: held there in GUIDED" in at.note, at
    # A stream fault hands it to BRAKE once; nothing is streamed after that.
    assert hf.force_brake("no setpoint for 0.45 s")
    forced = hf.tick(f, None, 10.0, 0.0, 15.0, 3.1, 0.1)
    assert (forced.mode, forced.command) == ("LOITER", None), forced
    after = hf.tick(f, None, 10.0, 0.0, 15.0, 3.2, 0.1)
    assert (after.mode, after.command) == (None, None), after
    # Leaving the Loiter resets it: the next Loiter is held in GUIDED again.
    hf.tick(_FakeFSM(state=S.NORMAL), "GUIDED", 10.0, 0.0, 15.0, 3.3, 0.1)
    assert not hf.forced and hf.tick(_FakeFSM(hold=None), "LOITER", 10.0, 0.0, 15.0,
                                     3.4, 0.1).command is not None


def _hs04_handover(model: str, seconds: float = 3.0):
    """hs04_006 (the hil shakedown at ee356c6) from its hand-over: at 10.6 s
    the aircraft moves at (-0.18, -0.05) m/s while GUIDED accelerates it toward
    the repaired request (2.1, -2.0) m/s - the rig's fitted 2.5 m/s^2 that way
    (the response's shaped acceleration) - and the in-place plan is the spot
    it stands on (the origin here). Flown on the harness's rail response
    (experiments/sweep_scenarios.py RAIL_RESPONSE) by the old flow (ArduCopter
    BRAKE, with its carry) or the GUIDED hold; the farthest and the last
    distance from the planned point."""
    import sweep_scenarios as SW
    veh = SW.VehicleResponse(SW.RAIL_RESPONSE, v0=(-0.18, -0.05, 0.0))
    veh.at = [1.85, -1.74]
    hf = HoldFlight(guided_hold=(model == "guided"))
    f = _FakeFSM(hold=HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True,
                                t_s=10.6))
    x = y = 0.0
    far = 0.0
    t = 10.6
    while t < 10.6 + seconds - 1e-9:
        ht = hf.tick(f, "LOITER" if t == 10.6 else None, x, y, 15.0, t, 0.1)
        if ht.command is not None:
            veh.brake(False)
            vx, vy, _ = veh.step(ht.command, 0.1)
        else:
            veh.brake(True)
            vx, vy, _ = veh.step((0.0, 0.0, 0.0), 0.1)
        x, y, t = x + vx * 0.1, y + vy * 0.1, round(t + 0.1, 6)
        far = max(far, math.hypot(x, y))
    return far, math.hypot(x, y)


def test_brakes_carry_is_the_rigs():
    """ArduCopter's BRAKE as the rail arm models it (sweep_scenarios.py
    BRAKE_PI_P / BRAKE_PI_I) against hs04_006's recorded velocities from the
    hand-over (BRAKE at 10.66 s, -0.10 / -0.11 m/s, GUIDED's last demand
    (1.85, -1.74) m/s^2): within 0.12 m/s at every tick to 13.4 s."""
    import sweep_scenarios as SW
    rec = {10.7: (-0.03, -0.17), 10.8: (0.15, -0.31), 10.9: (0.32, -0.43),
           11.0: (0.43, -0.51), 11.2: (0.55, -0.58), 11.4: (0.63, -0.62),
           11.7: (0.68, -0.64), 12.0: (0.66, -0.61), 12.3: (0.61, -0.56),
           12.6: (0.54, -0.49), 12.9: (0.46, -0.42), 13.4: (0.33, -0.30)}
    veh = SW.VehicleResponse(SW.RAIL_RESPONSE, v0=(-0.10, -0.11, 0.0), t0=10.66)
    veh.at = [1.85, -1.74]
    veh.brake(True)
    t = 10.66
    worst = 0.0
    for tr in sorted(rec):
        veh.advance(tr - t)
        t = tr
        vx, vy, _ = veh.velocity
        worst = max(worst, abs(vx - rec[tr][0]), abs(vy - rec[tr][1]))
    assert worst <= 0.12, worst


def test_the_hs04_hand_over_holds_its_point_in_guided():
    """hs04_005/006: handed to BRAKE, the aircraft was carried 1.7 m in 2.2 s
    from the in-place point the search had checked, and the bar's sweep
    reached it (X6 at 18.8 s). From the same hand-over the old flow carries it
    as far on the rail response; held in GUIDED it comes back onto the point."""
    far_b, last_b = _hs04_handover("brake")
    far_g, last_g = _hs04_handover("guided")
    assert far_b > 1.5 and last_b > 1.5, (far_b, last_b)
    assert far_g < 0.6 and last_g < 0.15, (far_g, last_g)


def test_a_transit_not_reached_in_time_holds_where_it_is():
    """Given up once its travel time + HOLD_ARRIVAL_SLACK_S has passed and the
    aircraft has not moved TRANSIT_PROGRESS_M relative to the point, either
    way, for TRANSIT_STALL_S."""
    hf = HoldFlight(guided_hold=False)
    f = _FakeFSM(hold=_moved(travel=1.0))
    assert hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1).transit
    late = hf.tick(f, None, 0.0, 0.0, 15.0, 2.1, 0.1)              # due at 1.0 + 1.0 s
    assert (late.mode, late.command) == ("LOITER", None) and "not reached" in late.note
    assert "neither closing nor carried away" in late.note, late.note


def test_a_transit_still_closing_past_its_travel_time_is_flown_on():
    """The hil shakedown (hs02, rotate-spin@3): a 0.43 m move planned at 0.25 s
    was given up 1.25 s after the plan (the sqrt controller's last metre and
    the rig's lag are not in the plan's travel time). Flown on while it
    closes; TRANSIT_MAX_S caps it."""
    from guardrail.hold_flight import TRANSIT_MAX_S
    hf = HoldFlight(guided_hold=False)
    f = _FakeFSM(hold=_moved(x=40.0, travel=1.0))
    assert hf.tick(f, "LOITER", 0.0, 0.0, 15.0, 0.0, 0.1).transit
    on = hf.tick(f, None, 10.0, 0.0, 15.0, 2.1, 0.1)               # past due, closing
    assert on.transit and on.mode is None, on
    on = hf.tick(f, None, 12.0, 0.0, 15.0, 2.9, 0.1)
    assert on.transit, on
    capped = hf.tick(f, None, 20.0, 0.0, 15.0, 1.0 + TRANSIT_MAX_S, 0.1)
    assert (capped.mode, capped.command) == ("LOITER", None), capped
    assert "past its travel time" in capped.note, capped.note


# hs03_004_dyn-nfz-rotate-spin (the hil shakedown at 199cfdf), flight_log.jsonl:
# the moved point (-189.96, -0.43) planned at 4.5 s while the aircraft was still
# braking out of the hand-over (3.01 m/s away from it). (t, distance to the
# point): the recorded 2.28 m at 4.5 s, 3.61 at 5.0, 4.42 at 5.5, 4.63 at 5.8,
# 4.66 at 6.0 (the farthest) and 4.27 at 7.0, the ticks between filled from the
# recorded velocities (3.01 m/s at 4.5 s, 2.06 at 5.0, 0.87 at 5.5, 0.24 at 5.8,
# 0.05 at 5.9, -0.13 at 6.0); after 6.0 s the rig's own creep back toward the
# point under BRAKE (about 0.48 m/s), a slower return than GUIDED's.
HS03_PLAN_T, HS03_TRAVEL_S = 4.5, 0.11
HS03_TRACE = [(4.5, 2.28), (4.6, 2.57), (4.7, 2.85), (4.8, 3.11), (4.9, 3.37),
              (5.0, 3.61), (5.1, 3.82), (5.2, 4.01), (5.3, 4.17), (5.4, 4.31),
              (5.5, 4.42), (5.6, 4.51), (5.7, 4.58), (5.8, 4.63), (5.9, 4.65),
              (6.0, 4.66), (6.1, 4.64), (6.2, 4.60), (6.3, 4.56), (6.4, 4.52),
              (6.5, 4.47), (6.6, 4.43), (6.7, 4.39), (6.8, 4.35), (6.9, 4.31),
              (7.0, 4.27)]


def _watch_trace(trace, t0, travel):
    """Feed (t, d) to HoldFlight along +x from a point at the origin; the
    first tick the hold mode is sent, or None."""
    hf = HoldFlight(guided_hold=False)
    f = _FakeFSM(hold=HoldPoint("LOITER", "moved", 0.0, 0.0, 15.0, legal=True,
                                travel_s=travel, speed_mps=4.0, climb_mps=2.0, t_s=t0))
    for t, d in trace:
        out = hf.tick(f, None, d, 0.0, 15.0, t, 0.1)
        if out.mode == "LOITER":
            return t, out.note
    return None


def test_the_hs03_overshoot_is_not_given_up():
    """hs03 (199cfdf) gave the transit up at 5.8 s, 4.63 m off ("not closing
    for 1.3 s"), one tick before the aircraft's velocity crossed zero, and the
    hold BRAKE then kept it about 4.3 m forward of the point, where the bar's
    sweep reached it (X6 at 31.1 s). The overshoot is braked out by the
    transit's own command: it is not "not closing"."""
    assert _watch_trace(HS03_TRACE, HS03_PLAN_T, HS03_TRAVEL_S) is None
    # Stuck at its farthest from then on, it is given up TRANSIT_STALL_S after
    # its distance last moved 5 cm (4.58 -> 4.63 m at 5.8 s).
    from guardrail.hold_flight import TRANSIT_STALL_S
    stuck = [(t, d) for t, d in HS03_TRACE if t <= 6.0 + 1e-9]
    stuck += [(round(6.0 + 0.1 * k, 1), 4.66) for k in range(1, 40)]
    got = _watch_trace(stuck, HS03_PLAN_T, HS03_TRAVEL_S)
    assert got is not None and abs(got[0] - (5.8 + TRANSIT_STALL_S)) < 1e-9, got
    assert "neither closing nor carried away" in got[1], got


def test_a_transit_that_cannot_close_is_given_up():
    """A wind the aircraft cannot beat: held at one distance it is given up
    TRANSIT_STALL_S after its plan (its deadline passed); carried away
    steadily it is given up TRANSIT_MAX_S after its travel time."""
    from guardrail.hold_flight import TRANSIT_MAX_S, TRANSIT_STALL_S
    held = [(round(0.1 * k, 1), 3.0) for k in range(0, 60)]
    t, note = _watch_trace(held, 0.0, 0.75)
    assert abs(t - max(0.75 + 1.0, TRANSIT_STALL_S)) < 1e-9, (t, note)
    assert "neither closing nor carried away" in note, note
    away = [(round(0.1 * k, 1), 3.0 + 0.08 * k) for k in range(0, 200)]
    t, note = _watch_trace(away, 0.0, 0.75)
    assert abs(t - (0.75 + TRANSIT_MAX_S)) < 0.1 + 1e-9, (t, note)
    assert "past its travel time" in note, note


def test_the_rails_and_the_harness_judge_the_same_distance():
    """transit_distance reads a HoldPoint (the rails) and its dict (the
    harness's ShieldDecision.hold_point) alike: 3-D for a Loiter, level for a
    Land."""
    from guardrail.hold_flight import transit_distance
    for mode in ("LOITER", "LAND"):
        p = HoldPoint(mode, "moved", 3.0, 4.0, 12.0, legal=True, travel_s=1.25,
                      speed_mps=4.0, climb_mps=2.0, t_s=0.0)
        a = transit_distance(0.0, 0.0, 15.0, p)
        b = transit_distance(0.0, 0.0, 15.0, p.to_dict())
        assert a == b and a == (5.0 if mode == "LAND" else (25.0 + 9.0) ** 0.5), (mode, a, b)


def test_the_rail_arm_gives_up_by_the_rails_own_rule():
    """Parity (experiments/sweep_scenarios.py TRANSIT_GIVE_UP_RESPONSES): the
    harness's rail arm judges each moved transit with hold_flight.TransitWatch,
    the class HoldFlight uses, on every tick of its plan; the ideal arm does
    not give up."""
    import sweep_scenarios as SW
    from guardrail import hold_flight as HF
    seen = []

    class Spy(HF.TransitWatch):
        def update(self, dist, t):
            seen.append((t, dist))
            return super().update(dist, t)
    keep = HF.TransitWatch
    HF.TransitWatch = Spy
    try:
        _spin("rail", 3)
        rail = list(seen)
        seen.clear()
        _spin("ideal", 3)
    finally:
        HF.TransitWatch = keep
    assert SW.HF is HF and "rail" in SW.TRANSIT_GIVE_UP_RESPONSES
    assert rail and not seen, (len(rail), len(seen))


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
# The per-tick bound (guardrail/holdpoint.py, THE PER-TICK BOUND; the hil
# shakedown hs02: one planning tick of 146.9 ms on the Orin)
# --------------------------------------------------------------------------- #

def _paced(sh: Shield, units: float, run):
    """Run `run` as a HoldSearch of `units` a tick until it ends: its result,
    and the units each tick's slice charged."""
    per = []
    s = HP.HoldSearch(sh, run, sh._new_tick(None), units)
    while True:
        before = s.spent
        done = s.step()
        per.append(s.spent - before)
        if done:
            return s, per
        assert s.ticks < 1000, "the search must end"


def test_a_search_spread_over_ticks_plans_exactly_as_the_whole_search():
    """The same plan, field for field, as the search made within one call in
    the same world; no tick's slice charges more than its units (a single
    charge larger than a slice is made alone), and the search takes
    ceil(spent / units) ticks at most a slice's last charge apart."""
    sh = _slow_bar(half_len=1.5)
    whole = HP.choose_hold_point(sh, HERE, hold_s=30.0)
    s, per = _paced(sh, 20.0, lambda: HP.choose_hold_point(sh, HERE, hold_s=30.0))
    assert s.error is None and not s.cancelled, s.error
    assert s.result == whole, (s.result, whole)
    assert s.ticks > 1 and len(per) == s.ticks, per
    assert max(per) <= 20.0 + HP.MOVING_ZONE_STOP_COST + 5.0, per    # one charge over at most
    s2, per2 = _paced(sh, 20.0, lambda: HP.choose_hold_point(sh, HERE, hold_s=30.0))
    assert (s2.result, per2) == (s.result, per), "the slices must be reproducible"


def test_a_search_dropped_mid_way_ends_its_thread():
    sh = _slow_bar(half_len=1.5)
    s = HP.HoldSearch(sh, lambda: HP.choose_hold_point(sh, HERE, hold_s=30.0),
                      sh._new_tick(None), 5.0)
    assert not s.step()
    s.cancel()
    assert s.done and s.cancelled and s.result is None
    s._thread.join(timeout=5.0)
    assert not s._thread.is_alive()


def test_a_replan_holds_in_place_while_its_search_runs():
    """A Loiter whose point is no longer legal: on the search's first tick an
    in-place plan (legal unless a stop here is illegal), then the new plan."""
    sh = _slow_bar(half_len=1.5)
    old = HoldPoint("LOITER", "moved", 5.0, 5.0, 15.0, legal=True, travel_s=1.8,
                    speed_mps=4.0, t_s=0.0)

    class _Asks(_FakeFSM):
        @property
        def needs_hold(self):
            return self.state in (S.LOITER, S.LAND)

    f = _Asks(hold=old)
    planner = HP.HoldPlanner(units=20.0)
    run = (lambda: HP.choose_hold_point(sh, HERE, hold_s=30.0))
    first = planner.tick(sh, f, run, here=HERE, unsafe=[])
    assert first is not None and (first.kind, first.legal) == ("in_place", True), first
    assert "while the hold-point search runs" in first.reason, first.reason
    f.hold = first
    got = None
    for _ in range(200):
        got = planner.tick(sh, f, run, here=HERE, unsafe=[])
        if got is not None:
            break
    assert got == HP.choose_hold_point(sh, HERE, hold_s=30.0), got
    assert planner.last["ticks"] > 1, planner.last
    # A Loiter left mid-search drops it.
    f.hold = old
    assert planner.tick(sh, f, run, here=HERE, unsafe=[]) is not None   # in place again
    f.state = S.NORMAL
    assert planner.tick(sh, f, run, here=HERE, unsafe=[]) is None
    assert planner.search is None


def test_the_guard_is_refused_unsampled_while_a_search_runs():
    """2026-11-10 (the hil shakedown hs05: the Orin paid 93-121 ms for a
    slice and the hand-back guard on one tick). HoldPlanner.handback samples
    nothing while a search runs and refuses the hand-back; with no search it
    is Shield.handback_unsafe."""
    sh = _slow_bar(half_len=1.5)
    calls = []
    orig = sh.handback_unsafe

    def count(*a, **kw):
        calls.append(a)
        return orig(*a, **kw)
    sh.handback_unsafe = count

    class _Asks(_FakeFSM):
        @property
        def needs_hold(self):
            return self.state in (S.LOITER, S.LAND) and self.hold is None

    f = _Asks()
    planner = HP.HoldPlanner(units=20.0)
    run = (lambda: HP.choose_hold_point(sh, HERE, hold_s=30.0))
    act = Action4D(vx=0.0, vy=-0.2, vz_up=0.0)
    try:
        assert planner.tick(sh, f, run, here=HERE, unsafe=[]) is None    # entry: no guard ran
        assert planner.search is not None and planner.slice > 0.0 and planner.guard is None
        got, slices = None, []
        while got is None:
            assert planner.handback(sh, HERE, act) == (True, None) and planner.guard == "search"
            got = planner.tick(sh, f, run, here=HERE, unsafe=[])
            slices.append(planner.slice)
            assert len(slices) < 200
        assert not calls, "the guard sampled on a search tick"
        assert all(0.0 < s <= 20.0 + HP.MOVING_ZONE_STOP_COST + 5.0 for s in slices), slices
        assert got == HP.choose_hold_point(sh, HERE, hold_s=30.0)
        f.hold = got
        assert planner.handback(sh, HERE, act) == (False, None) and len(calls) == 1
        assert (planner.guard, planner.checked) == ("ran", "path")
        # Within the tick (units None) there is no slice to keep the guard from.
        whole = HP.HoldPlanner(units=None)
        assert whole.handback(sh, HERE, act) == (False, None) and len(calls) == 2
    finally:
        del sh.handback_unsafe


def test_a_replan_that_starts_on_a_guard_tick_takes_its_first_slice_the_next():
    """The tick the point held stops being legal already paid for the guard:
    the in-place plan is still given at once, and the search's first slice is
    the next tick's (`last["waited"]`)."""
    sh = _slow_bar(half_len=1.5)
    old = HoldPoint("LOITER", "moved", 5.0, 5.0, 15.0, legal=True, travel_s=1.8,
                    speed_mps=4.0, t_s=0.0)

    class _Asks(_FakeFSM):
        @property
        def needs_hold(self):
            return self.state in (S.LOITER, S.LAND)

    f = _Asks(hold=old)
    planner = HP.HoldPlanner(units=20.0)
    run = (lambda: HP.choose_hold_point(sh, HERE, hold_s=30.0))
    planner.handback(sh, HERE, Action4D(vx=0.0, vy=-0.2, vz_up=0.0))
    first = planner.tick(sh, f, run, here=HERE, unsafe=[])
    assert first is not None and (first.kind, first.legal) == ("in_place", True), first
    assert planner.slice == 0.0 and planner.search is not None and planner.search.ticks == 0
    f.hold = first
    got = None
    for _ in range(200):
        assert planner.handback(sh, HERE, Action4D(vx=0.0, vy=-0.2, vz_up=0.0)) == (True, None)
        got = planner.tick(sh, f, run, here=HERE, unsafe=[])
        assert planner.slice > 0.0
        if got is not None:
            break
    assert got == HP.choose_hold_point(sh, HERE, hold_s=30.0), got
    assert planner.last["waited"] == 1 and planner.last["ticks"] > 1, planner.last


def test_the_request_check_is_asked_once_while_nothing_it_was_asked_about_changed():
    """One decision of the Shield's is as dear as the tick's own, so the
    guard's request check is not made every other tick: in a still world
    once, then the path every tick; again when the request, the hold point
    or the rule set changes; in turn with the path, tick by tick, where the
    world moves (an answer kept 0.5 s there handed rotate-spin at 9 deg/s
    back late, into the bar's next sweep); and never on two ticks running, so
    a request that changes every tick still leaves the path sampled."""
    class _Cfg:
        handback_request = "at_speed"

    def run(sh, raws, f):
        asked = []
        orig_rq, orig_hb = sh.handback_untrusted, sh.handback_unsafe
        sh.handback_untrusted = lambda *a, **kw: (asked.append("request"), False)[1]
        sh.handback_unsafe = lambda *a, **kw: (asked.append("path"), False)[1]
        planner = HP.HoldPlanner(units=20.0)
        outs = []
        try:
            for raw in raws:
                outs.append(planner.handback(sh, HERE, raw, fsm=f, raw=raw))
                planner.tick(sh, f, lambda: None, here=HERE, unsafe=[])
        finally:
            del sh.handback_untrusted, sh.handback_unsafe
        return asked, outs

    f = _FakeFSM(hold=HoldPoint("LOITER", "in_place", 0.0, 0.0, 15.0, legal=True, t_s=0.0))
    f.config, f.needs_hold = _Cfg(), False
    still = Shield(load_policy(DEMO), escalation=False)
    still.filter(HERE, Action4D(), t=0.0)
    assert still.world_is_still()
    same = [Action4D(vx=1.0, vy=0.0, vz_up=0.0)] * 12
    asked, outs = run(still, same, f)
    assert asked == ["request"] + ["path"] * 11, asked
    assert outs[0] == (True, False) and outs[1] == (False, False), outs[:2]   # refused until both answered
    # A request that changes (by more than 1 cm/s) is asked about again, a tick apart.
    drift = [Action4D(vx=1.0 + 0.05 * i, vy=0.0, vz_up=0.0) for i in range(8)]
    asked, _ = run(still, drift, f)
    assert asked == ["request", "path"] * 4, asked
    jitter = [Action4D(vx=1.0 + 1e-6 * i, vy=0.0, vz_up=0.0) for i in range(6)]
    assert run(still, jitter, f)[0] == ["request"] + ["path"] * 5
    # Where a zone moves, in turn.
    moving = _slow_bar(half_len=1.5)
    assert not moving.world_is_still()
    asked, _ = run(moving, same, f)
    assert asked == ["request", "path"] * 6, asked


def test_no_tick_of_the_cells_pays_for_two_kinds_of_hold_work():
    """dyn-nfz-rotate-spin on the rig's vehicle, every Shield tick: the
    hand-back guard's path samples, or its one decision for the request at
    speed, or a search slice of at most TICK_QUERIES units - never two of
    them; a hand-back refused for the search says so on its record."""
    from guardrail import shield as SH
    ticks = []
    cur = {"path": 0, "request": 0, "slice": 0.0}
    orig_hb, orig_rq, orig_tick, orig_filter, orig_filter_body = (
        SH.Shield.handback_unsafe, SH.Shield.handback_untrusted, HP.HoldPlanner.tick,
        SH.Shield.filter, SH.Shield.filter_body)

    def hb(self, *a, **kw):
        cur["path"] += 1
        return orig_hb(self, *a, **kw)

    def rq(self, *a, **kw):
        cur["request"] += 1
        return orig_rq(self, *a, **kw)

    def ptick(self, *a, **kw):
        try:
            return orig_tick(self, *a, **kw)
        finally:
            cur["slice"] += self.slice

    def record(d):
        rec = getattr(d, "fsm_record", None) or {}
        ticks.append((cur["path"], cur["request"], cur["slice"], rec.get("handback_refused"),
                      rec.get("handback_unsafe")))

    def filt(self, *a, **kw):
        cur.update(path=0, request=0, slice=0.0)
        d = orig_filter(self, *a, **kw)
        record(d)
        return d

    # The harness hands the flying Shield the body frame (filter_body), which
    # does not go through filter, so both entries are counted.
    def filt_body(self, *a, **kw):
        cur.update(path=0, request=0, slice=0.0)
        d, eb = orig_filter_body(self, *a, **kw)
        record(d)
        return d, eb
    (SH.Shield.handback_unsafe, SH.Shield.handback_untrusted, HP.HoldPlanner.tick,
     SH.Shield.filter, SH.Shield.filter_body) = hb, rq, ptick, filt, filt_body
    try:
        for dps in (3, 6):
            _spin("rail", dps)
    finally:
        (SH.Shield.handback_unsafe, SH.Shield.handback_untrusted, HP.HoldPlanner.tick,
         SH.Shield.filter, SH.Shield.filter_body) = (orig_hb, orig_rq, orig_tick, orig_filter,
                                                     orig_filter_body)
    n = [sum(1 for t in ticks if t[k]) for k in (0, 1, 2)]
    assert min(n) > 5 and n[0] > 20 and n[1] > 20, n
    two = [t for t in ticks if (t[0] > 0) + (t[1] > 0) + (t[2] > 0) > 1]
    assert not two, two[:3]
    assert max(t[0] for t in ticks) == 1 and max(t[1] for t in ticks) == 1
    assert max(t[2] for t in ticks) <= HP.TICK_QUERIES + 1e-9, max(t[2] for t in ticks)
    refused = [t for t in ticks if t[3]]
    assert refused and all(t[3] == "hold search running" and t[4] and not t[0] and not t[1]
                           for t in refused), refused[:3]


def test_a_search_that_raises_holds_in_place_marked_illegal():
    """An exception on the search's thread never reaches the rail's loop: the
    plan is an in-place one marked illegal that names it."""
    sh = _slow_bar(half_len=1.5)

    class _Asks(_FakeFSM):
        @property
        def needs_hold(self):
            return self.state in (S.LOITER, S.LAND)

    def boom():
        raise RuntimeError("no geometry")
    got = HP.HoldPlanner(units=20.0).tick(sh, _Asks(), boom, here=HERE, unsafe=[])
    assert got is not None and (got.kind, got.legal) == ("in_place", False), got
    assert "RuntimeError" in got.reason, got.reason


def test_the_hand_back_guard_is_bounded_by_its_units():
    """Shield.handback_unsafe samples at most HANDBACK_QUERIES units' worth of
    stop queries at the world's dearest (stop_cost_bound), every
    HANDBACK_STEP_S where that fits."""
    from guardrail import shield as SH
    sh = _slow_bar(half_len=1.5)
    calls = []
    orig = sh.state_is_unsafe

    def count(p, *a, **kw):
        calls.append(p)
        return orig(p, *a, **kw)
    sh.state_is_unsafe = count
    try:
        assert not sh.handback_unsafe(HERE, Action4D(vx=0.0, vy=-0.2, vz_up=0.0))
    finally:
        del sh.state_is_unsafe
    full = int(round(sh.lookahead_s / SH.HANDBACK_STEP_S))
    bound = HP.stop_cost_bound(sh)
    assert len(calls) == max(1, min(full, int(HP.HANDBACK_QUERIES // bound))), (len(calls), bound)
    assert len(calls) * bound <= HP.HANDBACK_QUERIES + bound - 1e-9


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
