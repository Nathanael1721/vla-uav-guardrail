"""The Shield's escalation FSM (guardrail/fsm.py) and the fail-safe KPI scorer.

Run either way:
    pytest tests/test_fsm.py -v
    python tests/test_fsm.py

WHY THIS FILE EXISTS

Until 2026-10-06 the Shield had exactly one fallback, a zero-velocity brake,
and no code could change the ArduPilot flight mode. The grant's escalation
chain (Safety Shield PDF p4: Normal -> Brake -> Loiter -> RTL -> Land, N=3 in
T=5 s, theta 2.0 m / 0.5 m, T_recover 2 s) did not exist, so "fail-safe
trigger correctness" could only ever be reported as the escape rate under
another name (audit card WP3-15).

These tests are mostly about the places a state machine is quietly wrong
while still looking right:

  * every edge, grant and extension, is driven at least once, and
    `test_every_edge_is_driven` fails if someone adds an edge and no test;
  * the one case nothing else escalates: a position that stays illegal while
    a recovery keeps re-checking clean (X6), with the real Shield;
  * the theta proxy measured from the CLAMPED command, because the clamp's
    delta-v is not a position correction;
  * timing at the exact window edges, with clocks accumulated in 0.1 s steps
    the way a 10 Hz loop accumulates them, because fifty additions of 0.1
    are 4.999999999999998, not 5.0;
  * soft rules and monitor_only rules, which must NOT escalate however hard
    they are pushed;
  * priority, which must order the record and never lighten the response;
  * the KPI scorer, tested against the two zero-skill Shields (always
    trigger, never trigger) as well as a good one. A scorer that cannot tell
    those apart is the defect this file exists to prevent.
"""
import json
import math
import random
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.fsm import (ACTIONS, EDGES, SETPOINTS, EscalationFSM,  # noqa: E402
                           FSMConfig, FSMState, RepairMagnitude, RuleHit, TickInput,
                           canonical_action, magnitude_from_actions,
                           rules_from_policy, score_failsafe_triggers,
                           tick_input_from_decision)

S = FSMState
SMALL = RepairMagnitude(0.5, 0.0)       # well inside theta
BIG = RepairMagnitude(3.0, 0.0)         # lateral over theta (2.0 m)


def hit(rid="nfz", action="project_fix", ctype="hard", prio="P0"):
    return RuleHit(rid, action, ctype, prio)


def clock(n, dt=0.1, t0=0.0):
    """n timestamps accumulated the way a 10 Hz loop accumulates them."""
    out, t = [], t0
    for _ in range(n):
        out.append(t)
        t += dt
    return out


def raises(exc, fn, *a, **k):
    try:
        fn(*a, **k)
    except exc as e:
        return e
    raise AssertionError(f"did not raise {exc.__name__}")


def to_loiter_via_g3(f, t0=0.0):
    """Two untrusted (over-theta) repairs: X1 into Brake, then G3 into Loiter."""
    assert f.tick(t0, "repaired", [hit()], BIG).edge == "X1"
    o = f.tick(t0 + 0.1, "repaired", [hit()], BIG)
    assert o.edge == "G3" and o.state == S.LOITER, o.reason
    return o


# --------------------------------------------------------------------------- #
# Grant defaults and the resting state
# --------------------------------------------------------------------------- #

def test_defaults_are_the_grants():
    """Safety Shield PDF p4: N=3, T=5 s, theta 2.0 m / 0.5 m, T_recover 2 s."""
    c = FSMConfig()
    assert (c.n_violations, c.window_s, c.theta_lateral_m, c.theta_vertical_m,
            c.t_recover_s) == (3, 5.0, 2.0, 0.5, 2.0)
    assert ACTIONS == ("monitor_only", "project_fix", "brake", "loiter", "RTL", "land")


def test_clean_flight_stays_normal_and_passes_the_action():
    f = EscalationFSM()
    for t in clock(50):
        o = f.tick(t)
        assert (o.state, o.setpoint, o.set_mode, o.transition) == (S.NORMAL, "pass", None, None)
    s = f.summary()
    assert s["visited"] == ["Normal"] and not s["failsafe_triggered"]
    assert s["outcome_label"] is None and s["ticks"] == 50


# --------------------------------------------------------------------------- #
# Every edge
# --------------------------------------------------------------------------- #

def test_g1_repairable_violation_enters_brake_and_flies_the_repair():
    o = EscalationFSM().tick(0.0, "repaired", [hit()], SMALL)
    assert (o.state, o.edge, o.setpoint, o.set_mode) == (S.BRAKE, "G1", "pass", None)
    assert o.record["edge_source"] == "grant"
    assert o.record["fsm_state_before"] == "Normal" and o.record["fsm_state_after"] == "Brake"


def test_x1_unconverged_repair_enters_brake_with_a_stop():
    """The FSM diagram has no Normal edge for this; the node diagram (p2) sends
    'not converged' into the chain, and p4 says projection is abandoned."""
    o = EscalationFSM().tick(0.0, "blocked", [hit()])
    assert (o.state, o.edge, o.setpoint) == (S.BRAKE, "X1", "brake")
    assert o.record["edge_source"] == "extension"


def test_x1_over_theta_repair_is_not_flown():
    o = EscalationFSM().tick(0.0, "repaired", [hit()], BIG)
    assert (o.state, o.edge, o.setpoint) == (S.BRAKE, "X1", "brake")
    assert o.record["theta_exceeded"] is True


def test_x1_brake_rule_stops_even_when_repairable():
    o = EscalationFSM().tick(0.0, "repaired", [hit(action="brake")], SMALL)
    assert (o.state, o.edge, o.setpoint) == (S.BRAKE, "X1", "brake")


def test_g3_second_over_theta_repair_loiters():
    f = EscalationFSM()
    o = to_loiter_via_g3(f)
    assert o.set_mode == "LOITER"
    # Handing over from GUIDED: stop on the transition tick...
    assert o.setpoint == "brake"
    # ...then stream nothing while the autopilot holds, and never re-send the mode.
    for t in clock(20, t0=0.2):
        o = f.tick(t, "repaired", [hit()], BIG)
        assert (o.state, o.set_mode, o.setpoint) == (S.LOITER, None, "none")


def test_g3_unconverged_repair_in_brake_loiters():
    f = EscalationFSM()
    f.tick(0.0, "repaired", [hit()], SMALL)
    o = f.tick(0.1, "blocked", [hit()])
    assert (o.state, o.edge) == (S.LOITER, "G3")


def test_trusted_repairs_alone_never_leave_brake():
    """Sliding along a fence: a repair every tick for 30 s, every one inside
    theta, is the Shield working. It is one onset, not 300."""
    f = EscalationFSM()
    for t in clock(300):
        o = f.tick(t, "repaired", [hit()], SMALL)
        assert o.state == S.BRAKE and o.setpoint == "pass"
    assert f.summary()["max_state"] == "Brake"


def test_g2_flicker_escalates_on_the_third_onset():
    f = EscalationFSM()
    seq = ["V", "C", "V", "C", "V"]
    states = []
    for t, k in zip(clock(5), seq):
        o = (f.tick(t, "repaired", [hit()], SMALL) if k == "V" else f.tick(t))
        states.append((o.state, o.edge))
    assert states[:4] == [(S.BRAKE, "G1"), (S.BRAKE, None), (S.BRAKE, None), (S.BRAKE, None)]
    assert states[4] == (S.LOITER, "G2"), states


def test_g2_slow_thrash_escalates_the_tick_after_the_third_onset():
    """Violate, recover fully (G7), violate, recover, violate: three onsets in
    4.4 s. The third lands in Normal (G1); the window still holds three on the
    next tick, so Brake goes to Loiter even though that tick is clean."""
    f = EscalationFSM()
    edges = {}
    for i, t in enumerate(clock(50)):
        o = (f.tick(t, "repaired", [hit()], SMALL) if i in (0, 22, 44) else f.tick(t))
        if o.edge:
            edges[i] = o.edge
    assert edges == {0: "G1", 21: "G7", 22: "G1", 43: "G7", 44: "G1", 45: "G2"}, edges


def test_g4_new_onsets_in_loiter_send_rtl():
    f = EscalationFSM()
    to_loiter_via_g3(f)
    seq = ["C", "V", "C", "V", "C", "V"]
    out = [(f.tick(t, "repaired", [hit()], SMALL) if k == "V" else f.tick(t))
           for t, k in zip(clock(6, t0=0.2), seq)]
    assert [o.state for o in out[:5]] == [S.LOITER] * 5
    assert (out[5].state, out[5].edge, out[5].set_mode) == (S.RTL, "G4", "RTL")
    # Loiter -> RTL is autopilot to autopilot: nothing to stop in GUIDED.
    assert out[5].setpoint == "none"


def test_g4_the_onsets_that_caused_the_loiter_do_not_count_again():
    """Without clearing the window on entry, the very next onset would make
    four in the window and fire G4 at once."""
    f = EscalationFSM()
    seq = ["V", "C", "V", "C", "V", "C", "V"]
    out = [(f.tick(t, "repaired", [hit()], SMALL) if k == "V" else f.tick(t))
           for t, k in zip(clock(7), seq)]
    assert out[4].edge == "G2"
    assert out[6].state == S.LOITER and out[6].record["n_in_t"] == 1, out[6].reason


def test_g5_loiter_timeout_at_its_exact_edge():
    f = EscalationFSM()                      # loiter_timeout_s = 30
    f.tick(0.5, "blocked", [hit()])
    assert f.tick(1.0, "blocked", [hit()]).edge == "G3"     # entered at 1.0
    # A violation that never clears and never re-onsets: only the timeout can act.
    o = f.tick(30.999, "blocked", [hit()])
    assert o.state == S.LOITER
    o = f.tick(31.0, "blocked", [hit()])
    assert (o.state, o.edge, o.set_mode) == (S.RTL, "G5", "RTL"), o.reason


def test_g5_disabled_timeout_holds_loiter():
    f = EscalationFSM(FSMConfig(loiter_timeout_s=None))
    to_loiter_via_g3(f)
    for t in (60.0, 600.0, 6000.0):
        assert f.tick(t, "blocked", [hit()]).state == S.LOITER


def test_recovery_is_checked_before_the_timeout():
    """At 31.0 both hold: hard rules clear for exactly T_recover, and the dwell
    is exactly the timeout. The grant's recovery edge wins."""
    f = EscalationFSM()
    f.tick(0.5, "blocked", [hit()])
    f.tick(1.0, "blocked", [hit()])
    f.tick(28.9, "blocked", [hit()])
    f.tick(29.0)
    o = f.tick(31.0)
    assert (o.state, o.edge) == (S.NORMAL, "G8"), o.reason


def test_g6_failed_rtl_lands():
    f = EscalationFSM()
    assert f.tick(0.0, "blocked", [hit(action="RTL")]).state == S.RTL
    o = f.tick(0.1, rtl_failed=True, note="battery")
    assert (o.state, o.edge, o.set_mode, o.setpoint) == (S.LAND, "G6", "LAND", "none")
    assert "battery" in o.reason


def test_g7_brake_recovers_at_exactly_t_recover_not_a_tick_before():
    f = EscalationFSM()
    ts = clock(40)
    out = [(f.tick(t, "repaired", [hit()], SMALL) if i == 10 else f.tick(t))
           for i, t in enumerate(ts)]
    assert out[10].edge == "G1"
    # Clean from tick 11. Tick 30 is 1.9 s of clearing, tick 31 is 2.0 s.
    assert out[30].state == S.BRAKE, out[30].reason
    assert (out[31].state, out[31].edge, out[31].set_mode) == (S.NORMAL, "G7", None)


def test_brake_does_not_recover_while_still_violated():
    f = EscalationFSM()
    for t in clock(200):
        o = f.tick(t, "repaired", [hit()], SMALL)
    assert o.state == S.BRAKE and o.record["cleared_s"] is None


def test_g8_loiter_recovers_and_asks_for_guided_back():
    f = EscalationFSM()
    to_loiter_via_g3(f)                        # entry tick 0.1 was violated
    out = [f.tick(t) for t in clock(25, t0=0.2)]
    # Clean from 0.2, so 2.0 s of clearing is reached at index 20 (t = 2.2).
    assert out[19].state == S.LOITER
    o = out[20]
    assert (o.state, o.edge, o.set_mode, o.setpoint) == (S.NORMAL, "G8", "GUIDED", "pass")


def test_g9_home_reached_ends_the_episode():
    f = EscalationFSM()
    f.tick(0.0, "blocked", [hit(action="RTL")])
    o = f.tick(0.1, home_reached=True)
    assert (o.transition, o.edge, o.terminal, o.setpoint) == ("RTL->[*]", "G9", True, "none")
    # Nothing moves a finished episode: not a violation, not a Land rule.
    o = f.tick(0.2, "blocked", [hit(action="land")])
    assert (o.state, o.transition, o.set_mode) == (S.RTL, None, None)
    s = f.summary()
    assert s["outcome_label"] == "RTL_triggered" and s["terminal"]


def test_g10_landed_ends_the_episode():
    f = EscalationFSM()
    assert f.tick(0.0, "blocked", [hit(action="land")]).edge == "G12"
    o = f.tick(9.0, landed=True)
    assert (o.transition, o.edge, o.terminal) == ("Land->[*]", "G10", True)
    assert f.summary()["outcome_label"] == "Land_triggered"


def test_g11_direct_rtl_skips_projection_even_when_repairable():
    o = EscalationFSM().tick(0.0, "repaired", [hit(action="RTL")], SMALL)
    assert (o.state, o.edge, o.set_mode, o.setpoint) == (S.RTL, "G11", "RTL", "brake")


def test_g12_direct_land():
    o = EscalationFSM().tick(0.0, "blocked", [hit(action="land")])
    assert (o.state, o.edge, o.set_mode) == (S.LAND, "G12", "LAND")


def test_x2_x3_direct_failsafes_are_not_ignored_in_brake_or_loiter():
    for action, target, edge in (("RTL", S.RTL, "X2"), ("land", S.LAND, "X3")):
        f = EscalationFSM()
        f.tick(0.0, "repaired", [hit("p2-env", prio="P2")], SMALL)         # Brake
        o = f.tick(0.1, "blocked", [hit("p0-nfz", action=action)])
        assert (o.state, o.edge) == (target, edge), o.reason
        f = EscalationFSM()
        to_loiter_via_g3(f)
        o = f.tick(0.2, "blocked", [hit("p0-nfz", action=action)])
        assert (o.state, o.edge, o.set_mode) == (target, edge, target.value.upper())


def test_x4_loiter_rule_goes_straight_to_loiter():
    o = EscalationFSM().tick(0.0, "blocked", [hit(action="loiter")])
    assert (o.state, o.edge, o.set_mode) == (S.LOITER, "X4", "LOITER")
    f = EscalationFSM()
    f.tick(0.0, "repaired", [hit()], SMALL)
    assert f.tick(0.1, "blocked", [hit(action="loiter")]).edge == "X4"
    # Already loitering: a loiter rule asks for what is already happening.
    assert f.tick(0.2, "blocked", [hit(action="loiter")]).transition is None


def test_x5_landed_during_rtl_ends_the_episode():
    f = EscalationFSM()
    f.tick(0.0, "blocked", [hit(action="RTL")])
    o = f.tick(40.0, landed=True)
    assert (o.edge, o.terminal) == ("X5", True)


def test_rtl_ignores_the_pilots_proposals():
    """In RTL the autopilot flies; what the VLA proposes is not flown and has
    no edge out of RTL in the grant."""
    f = EscalationFSM()
    f.tick(0.0, "blocked", [hit(action="RTL")])
    for t in clock(30, t0=0.1):
        o = f.tick(t, "repaired", [hit()], BIG)
        assert (o.state, o.set_mode, o.setpoint) == (S.RTL, None, "none")


def test_x6_a_position_illegal_for_t_reaches_rtl_not_a_permanent_brake():
    """A recovery that re-checks clean every tick is trusted (G1), exempt from
    theta, and one onset however long it lasts. Without X6 nothing could ever
    escalate an aircraft held inside a zone, which breaks p1's "trigger
    fail-safe when repair is unsafe"."""
    f = EscalationFSM()
    edges = {}
    for i, t in enumerate(clock(100)):
        o = f.tick(t, "repaired", [hit()], SMALL, stop_illegal=True)
        if o.edge:
            edges[i] = o.edge
        if i < 50:
            assert o.state == S.BRAKE and o.setpoint == "pass", (i, o.reason)
    # Tick 50 is t = 5.0 (accumulated 4.999999999999998): exactly T.
    assert edges == {0: "G1", 50: "X6"}, edges
    s = f.summary()
    assert s["failsafe_triggered"] and s["outcome_label"] == "RTL_triggered"
    assert not s["loiter_entered"]          # loitering in the zone is the deadlock


def test_x6_needs_the_position_illegal_on_every_tick_and_a_hard_rule():
    """One legal tick restarts the clock; a soft rule never escalates; a tick
    on which the Shield is not intervening on a hard rule does not fire it."""
    f = EscalationFSM(FSMConfig(t_recover_s=10.0))
    for i, t in enumerate(clock(100)):
        o = f.tick(t, "repaired", [hit()], SMALL, stop_illegal=(i != 49))
        assert o.edge in (None, "G1"), (i, o.reason)
    assert o.record["stop_illegal_s"] is not None and o.record["stop_illegal_s"] < 5.0
    f = EscalationFSM()
    for t in clock(200):
        o = f.tick(t, "repaired", [hit("env", ctype="soft", prio="P1")], SMALL,
                   stop_illegal=True)
        assert o.set_mode is None, o.reason
    f = EscalationFSM(FSMConfig(t_recover_s=10.0))
    f.tick(0.0, "repaired", [hit()], SMALL, stop_illegal=True)
    for t in clock(60, t0=0.1):
        o = f.tick(t, stop_illegal=True)          # raw already escaping: clean
        assert o.state == S.BRAKE, o.reason
    assert f.tick(6.1, "repaired", [hit()], SMALL, stop_illegal=True).edge == "X6"


def test_x6_bounds_a_loiter_held_inside_the_zone():
    """Nowhere legal to stop and no repair converging: the grant's ladder puts
    the aircraft in LOITER (G3), which holds it in the violation. X6 sends it
    home T after the position became illegal, not 30 s later at the timeout."""
    f = EscalationFSM()
    assert f.tick(0.0, "blocked", [hit()], stop_illegal=True).edge == "X1"
    o = f.tick(0.1, "blocked", [hit()], stop_illegal=True)
    assert (o.edge, o.setpoint) == ("G3", "pass")
    for t in clock(48, t0=0.2):
        assert f.tick(t, "blocked", [hit()], stop_illegal=True).state == S.LOITER
    o = f.tick(5.0, "blocked", [hit()], stop_illegal=True)
    assert (o.state, o.edge, o.set_mode, o.setpoint) == (S.RTL, "X6", "RTL", "none"), o.reason


def test_x6_order_in_brake_follows_the_strongest_response():
    def at_t(rule):
        f = EscalationFSM()
        f.tick(0.0, "repaired", [hit()], SMALL, stop_illegal=True)
        return f.tick(5.0, "blocked", [hit(), rule], stop_illegal=True)
    assert at_t(hit("r", action="land")).edge == "X3"       # land beats RTL
    assert at_t(hit("r", action="RTL")).edge == "X2"        # the rule is cited
    assert at_t(hit("r", action="loiter")).edge == "X6"     # RTL beats loiter


def test_x6_with_the_real_shield_held_inside_nfz_square_for_60_s():
    """The review's probe. Real Shield, sim_demo_policy, the aircraft held
    3 m inside nfz-square (as a wind beyond the 4 m/s cap would hold it)
    for 600 ticks. Every tick: GeofenceEscape, re-checks clean, a stop here
    is illegal. Before X6 the FSM sat in Brake for all 60 s with
    failsafe_triggered False, under both magnitude sources."""
    from guardrail import Action4D, Shield, State, load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    inside = State(x=10, y=15, up=15)
    d = sh.filter(inside, Action4D(vx=1.0))
    assert {r.operator for r in d.repairs} == {"GeofenceEscape"}, d.repairs
    stop = bool(sh.state_is_unsafe(inside))
    assert stop
    # The node will pass the violation list itself; nfz-square is hard P0.
    assert tick_input_from_decision(0.0, d, pol,
                                    stop_illegal=sh.state_is_unsafe(inside)).stop_illegal
    for h in (None, 1.0):
        f = EscalationFSM()
        edges = []
        for t in clock(600):
            o = f.step(tick_input_from_decision(t, d, pol, h, stop_illegal=stop))
            if o.edge:
                edges.append((round(t, 3), o.edge))
        assert edges == [(0.0, "G1"), (5.0, "X6")], (h, edges)
        assert f.summary()["outcome_label"] == "RTL_triggered", h


def test_a_hard_brake_rule_that_stays_blocked_stays_in_brake():
    """A `brake` rule is answered by the stop itself, not by projection, so an
    unconverged tick under it is not an untrusted REPAIR. Only the
    project_fix filter keeps it from loitering on tick 2, and integration
    step 2 makes shield.py return braked=True for every brake rule."""
    f = EscalationFSM()
    for t in clock(100):
        o = f.tick(t, "blocked", [hit("hold", action="brake")])
        assert (o.state, o.setpoint, o.set_mode) == (S.BRAKE, "brake", None), o.reason
    assert f.summary()["max_state"] == "Brake"


def test_a_direct_rtl_beats_g3_in_brake():
    """Both hold on one tick: an unconverged hard project_fix (G3 -> Loiter)
    and a hard RTL rule (X2 -> RTL). The stronger response wins."""
    f = EscalationFSM()
    f.tick(0.0, "repaired", [hit()], SMALL)
    o = f.tick(0.1, "blocked", [hit("nfz"), hit("home", action="RTL")])
    assert (o.state, o.edge) == (S.RTL, "X2"), o.reason


def test_entering_loiter_is_not_a_failsafe_trigger():
    """The KPI's `triggered` is RTL or Land entry (the outcome vocabulary has
    no Loiter_triggered). The sweep is told to consume this field."""
    f = EscalationFSM()
    assert f.tick(0.0, "blocked", [hit(action="loiter")]).edge == "X4"
    for t in clock(10, t0=0.1):
        f.tick(t, "blocked", [hit(action="loiter")])
    s = f.summary()
    assert (s["failsafe_triggered"], s["loiter_entered"], s["outcome_label"]) == \
        (False, True, None), s


def test_a_failed_rtl_that_lands_is_labelled_land():
    f = EscalationFSM()
    f.tick(0.0, "blocked", [hit(action="RTL")])
    assert f.tick(0.1, rtl_failed=True).edge == "G6"
    assert f.tick(9.0, landed=True).edge == "G10"
    s = f.summary()
    assert s["outcome_label"] == "Land_triggered" and s["visited"] == \
        ["Normal", "RTL", "Land"], s


def test_home_reached_wins_over_a_late_rtl_failure_report():
    """On the tick the autopilot reports home, the RTL succeeded, whatever
    else arrives with it."""
    for flags in (dict(home_reached=True, rtl_failed=True),
                  dict(landed=True, rtl_failed=True)):
        f = EscalationFSM()
        f.tick(0.0, "blocked", [hit(action="RTL")])
        o = f.tick(0.1, **flags)
        assert o.terminal and o.edge in ("G9", "X5"), (flags, o.reason)
        assert f.summary()["outcome_label"] == "RTL_triggered"


def _edge_scripts():
    """One shortest script per edge id; each returns the edges it produced."""
    def run(cfg, steps):
        f, seen = EscalationFSM(cfg), []
        for t, kw in steps:
            o = f.tick(t, **kw)
            if o.edge:
                seen.append(o.edge)
        return seen
    rep = dict(outcome="repaired", violations=[hit()], magnitude=SMALL)
    big = dict(outcome="repaired", violations=[hit()], magnitude=BIG)
    blk = dict(outcome="blocked", violations=[hit()])
    rtl = dict(outcome="blocked", violations=[hit(action="RTL")])
    land = dict(outcome="blocked", violations=[hit(action="land")])
    loit = dict(outcome="blocked", violations=[hit(action="loiter")])
    stuck = dict(rep, stop_illegal=True)
    clean = {}
    flick = [(0.0, rep), (0.1, clean), (0.2, rep), (0.3, clean), (0.4, rep)]
    return {
        "G1": run(None, [(0.0, rep)]),
        "G2": run(None, flick),
        "G3": run(None, [(0.0, big), (0.1, big)]),
        "G4": run(None, [(0.0, big), (0.1, big), (0.2, clean), (0.3, rep),
                         (0.4, clean), (0.5, rep), (0.6, clean), (0.7, rep)]),
        "G5": run(None, [(0.0, blk), (0.1, blk), (30.1, blk)]),
        "G6": run(None, [(0.0, rtl), (0.1, dict(rtl_failed=True))]),
        "G7": run(None, [(0.0, rep), (0.1, clean), (2.1, clean)]),
        "G8": run(None, [(0.0, big), (0.1, big), (0.2, clean), (2.2, clean)]),
        "G9": run(None, [(0.0, rtl), (0.1, dict(home_reached=True))]),
        "G10": run(None, [(0.0, land), (0.1, dict(landed=True))]),
        "G11": run(None, [(0.0, rtl)]),
        "G12": run(None, [(0.0, land)]),
        "X1": run(None, [(0.0, blk)]),
        "X2": run(None, [(0.0, rep), (0.1, rtl)]),
        "X3": run(None, [(0.0, rep), (0.1, land)]),
        "X4": run(None, [(0.0, loit)]),
        "X5": run(None, [(0.0, rtl), (0.1, dict(landed=True))]),
        "X6": run(None, [(0.0, stuck), (2.5, stuck), (5.0, stuck)]),
    }


def test_every_edge_is_driven():
    """A gate: adding an edge to EDGES without a script here fails this test."""
    scripts = _edge_scripts()
    assert set(scripts) == set(EDGES), set(EDGES) ^ set(scripts)
    for edge, seen in scripts.items():
        assert seen and seen[-1] == edge, f"{edge} script produced {seen}"


# --------------------------------------------------------------------------- #
# Window edges and the theta cap
# --------------------------------------------------------------------------- #

def test_n_in_t_window_is_inclusive_at_exactly_t():
    cfg = FSMConfig(t_recover_s=10.0, loiter_timeout_s=None)   # stay in Brake
    f = EscalationFSM(cfg)
    for t, k in ((0.0, "V"), (0.1, "C"), (2.5, "V"), (2.6, "C")):
        f.tick(t, "repaired", [hit()], SMALL) if k == "V" else f.tick(t)
    o = f.tick(5.0, "repaired", [hit()], SMALL)                # 0.0 is exactly 5 s old
    assert (o.state, o.edge) == (S.LOITER, "G2"), o.reason


def test_an_onset_older_than_t_has_expired():
    cfg = FSMConfig(t_recover_s=10.0, loiter_timeout_s=None)
    f = EscalationFSM(cfg)
    for t, k in ((0.0, "V"), (0.1, "C"), (2.5, "V"), (2.6, "C")):
        f.tick(t, "repaired", [hit()], SMALL) if k == "V" else f.tick(t)
    o = f.tick(5.001, "repaired", [hit()], SMALL)
    assert o.state == S.BRAKE and o.record["n_in_t"] == 2, o.reason


def test_window_edge_survives_a_float_accumulated_clock():
    """Fifty 0.1 s ticks are not 5 s on a float clock. Ticks 0 to 50 span
    4.999999999999998 s, and ticks 122 to 172 span 5.000000000000002 s. Both
    are "exactly T" to a 10 Hz loop, so both must count. The edge must not
    depend on which way the last bit rounded."""
    cfg = FSMConfig(t_recover_s=10.0, loiter_timeout_s=None)
    ts = clock(200)
    for a, b, c in ((0, 25, 50), (122, 147, 172)):
        assert ts[c] - ts[a] != 5.0
        f = EscalationFSM(cfg)
        out = [(f.tick(t, "repaired", [hit()], SMALL) if i in (a, b, c) else f.tick(t))
               for i, t in enumerate(ts[:c + 1])]
        assert out[c].edge == "G2", (a, c, ts[c] - ts[a], out[c].reason)
    assert ts[172] - ts[122] > 5.0                 # the case the slack exists for


def test_t_recover_survives_a_float_accumulated_clock():
    """Ticks 24 to 44 are 1.9999999999999996 s apart: twenty ticks, T_recover
    to a 10 Hz loop. Without the slack, recovery would slip a tick."""
    ts = clock(60)
    assert ts[44] - ts[24] < 2.0
    f = EscalationFSM()
    out = [(f.tick(t, "repaired", [hit()], SMALL) if i == 23 else f.tick(t))
           for i, t in enumerate(ts)]
    assert out[43].state == S.BRAKE
    assert out[44].edge == "G7", out[44].reason


def test_theta_exactly_is_accepted_and_just_over_is_not():
    """p3: the first operator with magnitude <= threshold wins; p4: escalate on
    '> threshold'."""
    for mag, edge in ((RepairMagnitude(2.0, 0.0), "G1"), (RepairMagnitude(2.000001, 0.0), "X1"),
                      (RepairMagnitude(0.0, 0.5), "G1"), (RepairMagnitude(0.0, 0.500001), "X1")):
        o = EscalationFSM().tick(0.0, "repaired", [hit()], mag)
        assert o.edge == edge, (mag, o.reason)


def test_theta_at_the_cap_survives_float_noise():
    """A magnitude computed as |dv| * h can land a few ulps over 2.0 m when the
    repair is exactly theta. That is "exactly theta" (accepted), not over."""
    for mag in (RepairMagnitude(2.0 + 1e-12, 0.0), RepairMagnitude(0.0, 0.5 + 1e-12)):
        assert EscalationFSM().tick(0.0, "repaired", [hit()], mag).edge == "G1", mag
    assert EscalationFSM().tick(0.0, "repaired", [hit()],
                                RepairMagnitude(2.0 + 1e-6, 0.0)).edge == "X1"


def test_theta_comes_from_the_config():
    cfg = FSMConfig(theta_lateral_m=1.0)
    assert EscalationFSM(cfg).tick(0.0, "repaired", [hit()], RepairMagnitude(1.5, 0)).edge == "X1"
    assert EscalationFSM().tick(0.0, "repaired", [hit()], RepairMagnitude(1.5, 0)).edge == "G1"


# --------------------------------------------------------------------------- #
# Soft rules and monitor_only: enforced at most to a brake, never escalated
# --------------------------------------------------------------------------- #

def test_soft_rule_asking_for_rtl_is_capped_at_brake_and_flagged():
    h = hit("envelope-default", action="RTL", ctype="soft", prio="P1")
    assert h.effective_action == "brake" and h.capped
    o = EscalationFSM().tick(0.0, "blocked", [h])
    assert (o.state, o.setpoint, o.set_mode) == (S.BRAKE, "brake", None)
    assert o.record["governing_rule"]["capped"] is True
    assert "capped at brake" in o.reason


def test_soft_rules_never_change_the_flight_mode_however_hard_they_are_pushed():
    """60 s of everything that escalates a hard rule: unconverged repairs,
    over-theta repairs, a flicker of onsets, and a soft rule asking to land."""
    f = EscalationFSM()
    soft = [hit("env", action="land", ctype="soft", prio="P0"),
            hit("speed", action="RTL", ctype="soft", prio="P0")]
    for i, t in enumerate(clock(600)):
        k = i % 4
        if k == 0:
            o = f.tick(t, "blocked", soft)
        elif k == 2:
            o = f.tick(t, "repaired", soft, BIG)
        else:
            o = f.tick(t)
        assert o.set_mode is None, o.reason
        if k == 0:
            assert o.setpoint == "brake"
    s = f.summary()
    assert s["max_state"] == "Brake" and not s["failsafe_triggered"], s


def test_the_grants_own_soft_example_brakes():
    """envelope-default (Policy DSL PDF p4): soft, P1, violation_action brake.
    That is what a soft rule may do, so it is obeyed, not 'capped'."""
    h = hit("envelope-default", "brake", "soft", "P1")
    assert (h.capped, h.effective_action) == (False, "brake")
    o = EscalationFSM().tick(0.0, "repaired", [h], SMALL)
    assert (o.state, o.setpoint) == (S.BRAKE, "brake")
    assert o.record["governing_rule"]["capped"] is False and "capped" not in o.reason


def test_a_soft_violation_cannot_hold_the_aircraft_in_loiter():
    """Otherwise a soft rule would reach RTL through the Loiter timeout."""
    f = EscalationFSM()
    to_loiter_via_g3(f)
    soft = [hit("env", ctype="soft", prio="P1")]
    edges = []
    for t in clock(400, t0=0.2):
        o = f.tick(t, "repaired", soft, SMALL)
        if o.edge:
            edges.append(o.edge)
    assert edges and edges[0] == "G8", edges
    assert "G5" not in edges and not f.failsafe_triggered(), edges


def test_monitor_only_is_recorded_and_does_nothing_else():
    f = EscalationFSM()
    m = hit("watch", action="monitor_only")
    for i, t in enumerate(clock(40)):
        o = f.tick(t, "clean", [m] if i % 2 == 0 else [])
        assert (o.state, o.setpoint, o.set_mode) == (S.NORMAL, "pass", None)
    assert o.record["violations"] == [] or o.record["violations"][0]["effective_action"] \
        == "monitor_only"
    o = f.tick(4.0, "clean", [m])
    assert o.record["violations"][0]["effective_action"] == "monitor_only"


# --------------------------------------------------------------------------- #
# Priority: orders the record, breaks ties, never lightens the response
# --------------------------------------------------------------------------- #

def test_p0_is_cited_over_p2_whatever_order_the_shield_listed_them():
    p0, p2 = hit("p0-nfz", prio="P0"), hit("p2-zone", prio="P2")
    recs = []
    for order in ([p2, p0], [p0, p2]):
        o = EscalationFSM().tick(0.0, "repaired", order, SMALL)
        assert o.record["governing_rule"]["rule_id"] == "p0-nfz"
        assert o.record["risk_level"] == "P0"
        assert [v["rule_id"] for v in o.record["violations"]] == ["p0-nfz", "p2-zone"]
        recs.append(json.dumps(o.record, sort_keys=True))
    assert recs[0] == recs[1]


def test_priority_never_makes_the_response_lighter():
    """A P2 rule that asks to land still lands when a P0 repair is also
    pending. Otherwise one MORE violation would make the response weaker."""
    alone = EscalationFSM().tick(0.0, "blocked", [hit("p2-land", action="land", prio="P2")])
    both = EscalationFSM().tick(0.0, "blocked", [hit("p0-nfz", prio="P0"),
                                                 hit("p2-land", action="land", prio="P2")])
    assert alone.state == both.state == S.LAND
    assert both.record["governing_rule"]["rule_id"] == "p2-land"
    assert both.record["risk_level"] == "P2"


def test_a_theta_escalation_is_attributed_to_a_rule_theta_judges():
    """A speed-cap rule's clamps always converge and theta never judges them,
    so it is neither the reason a repair is untrusted nor the rule cited for
    it. Before, with every rule P0 (no policy), `kin-caps` sorted first by id
    and was cited for every theta escalation on the delivered flights."""
    kin = RuleHit("kin-caps", "repair", "hard", "P0", kinematic=True)
    ring = hit("standoff-pedestrian")
    f = EscalationFSM()
    o = f.tick(0.0, "repaired", [kin, ring], BIG)
    assert o.edge == "X1" and o.record["governing_rule"]["rule_id"] == "standoff-pedestrian"
    o = f.tick(0.1, "repaired", [kin, ring], BIG)
    assert o.edge == "G3" and "standoff-pedestrian" in o.reason and "kin-caps" not in o.reason
    # A blocked tick whose only HARD projection is the speed cap: the soft
    # rule is the one that did not converge, and soft never leaves Brake.
    f = EscalationFSM()
    for t in clock(20):
        o = f.tick(t, "blocked", [kin, hit("zone", ctype="soft", prio="P1")])
        assert o.state == S.BRAKE, o.reason


def test_hard_is_listed_before_soft_whatever_the_priority():
    o = EscalationFSM().tick(0.0, "repaired", [hit("soft-p0", ctype="soft", prio="P0"),
                                               hit("hard-p2", prio="P2")], SMALL)
    assert [v["rule_id"] for v in o.record["violations"]] == ["hard-p2", "soft-p0"]


# --------------------------------------------------------------------------- #
# Vocabulary and input validation: refuse, never default
# --------------------------------------------------------------------------- #

def test_action_vocabulary_aliases_and_case():
    assert canonical_action("repair") == "project_fix"          # this repo's spelling
    assert canonical_action("Land") == "land"                   # FSM diagram's spelling
    assert canonical_action("rtl") == "RTL"
    for bad in ("rtl_now", "", "return", None):
        raises(ValueError, canonical_action, bad)
    raises(ValueError, RuleHit, "x", "repair", "firm", "P0")
    raises(ValueError, RuleHit, "x", "repair", "hard", "P3")
    raises(ValueError, RuleHit, "", "repair", "hard", "P0")


def test_inconsistent_inputs_are_refused():
    e = raises(ValueError, TickInput, t=0.0, outcome="repaired", violations=(hit(),))
    assert "theta" in str(e)
    raises(ValueError, TickInput, t=0.0, outcome="clean", violations=(hit(),))
    raises(ValueError, TickInput, t=0.0, outcome="blocked")
    raises(ValueError, TickInput, t=0.0, outcome="repaired",
           violations=(hit(action="monitor_only"),), magnitude=SMALL)
    raises(ValueError, TickInput, t=float("nan"), outcome="clean")
    raises(ValueError, TickInput, t=0.0, outcome="fixed")
    raises(ValueError, TickInput, t=0.0, outcome="clean", landed=1)
    raises(ValueError, RepairMagnitude, float("nan"), 0.0)
    raises(ValueError, RepairMagnitude, -0.1, 0.0)


def test_time_may_repeat_but_not_go_backwards():
    f = EscalationFSM()
    f.tick(1.0)
    f.tick(1.0)
    e = raises(ValueError, f.tick, 0.9)
    assert "backwards" in str(e)


# --------------------------------------------------------------------------- #
# Determinism and the audit record
# --------------------------------------------------------------------------- #

def _mixed_script(seed):
    rnd = random.Random(seed)
    rules = [hit("a", prio="P0"), hit("b", prio="P2"), hit("c", "brake", "soft", "P1"),
             hit("d", "monitor_only", "hard", "P1"), hit("e", "RTL", "soft", "P0")]
    steps, t = [], 0.0
    for _ in range(300):
        t += 0.1
        k = rnd.choice(["clean", "clean", "repaired", "blocked"])
        if k == "clean":
            vs = [rules[3]] if rnd.random() < 0.3 else []
            steps.append(TickInput(t=t, outcome="clean", violations=tuple(vs)))
            continue
        vs = rnd.sample(rules[:3] + rules[4:], rnd.randint(1, 3))
        mag = RepairMagnitude(rnd.choice([0.3, 1.9, 2.5]), 0.0) if k == "repaired" else None
        steps.append(TickInput(t=t, outcome=k, violations=tuple(vs), magnitude=mag))
    return steps


def test_same_inputs_same_outputs_and_reset_is_a_fresh_episode():
    steps = _mixed_script(7)
    a = [json.dumps(o.record, sort_keys=True) for o in EscalationFSM().run(steps)]
    f = EscalationFSM()
    b = [json.dumps(o.record, sort_keys=True) for o in f.run(steps)]
    f.reset()
    c = [json.dumps(o.record, sort_keys=True) for o in f.run(steps)]
    assert a == b == c
    # The script is not trivial: it visits more than Normal.
    assert len(set(json.loads(r)["fsm_state_after"] for r in a)) >= 2


def test_violation_order_does_not_change_anything():
    steps = _mixed_script(11)
    shuffled = [TickInput(t=s.t, outcome=s.outcome, magnitude=s.magnitude,
                          violations=tuple(reversed(s.violations))) for s in steps]
    a = [json.dumps(o.record, sort_keys=True) for o in EscalationFSM().run(steps)]
    b = [json.dumps(o.record, sort_keys=True) for o in EscalationFSM().run(shuffled)]
    assert a == b


def _random_episode(seed, soft_only=False):
    """A noisy episode: every outcome, every action, hard and soft, plus the
    autopilot's own reports. Direct fail-safes are rare so episodes run long."""
    rnd = random.Random(seed)
    # A second stream for stop_illegal, so adding it left every other draw
    # (and so every episode the older invariants were tuned on) unchanged.
    rnd_stop = random.Random(10_000 + seed)
    kinds = ("hard", "soft") if not soft_only else ("soft",)
    steps, t = [], 0.0
    for _ in range(400):
        t += rnd.choice([0.1, 0.1, 0.1, 0.05, 0.5])
        roll = rnd.random()
        flags = {}
        if roll < 0.01:
            flags = {rnd.choice(["rtl_failed", "home_reached", "landed"]): True}
        if rnd_stop.random() < 0.2:
            flags["stop_illegal"] = True
        k = rnd.choice(["clean", "clean", "clean", "repaired", "blocked"])
        if k == "clean":
            steps.append(TickInput(t=t, outcome="clean", **flags))
            continue
        acts = ["project_fix"] * 12 + ["brake"] * 3 + ["loiter", "RTL", "land"]
        vs = [hit(f"r{i}", rnd.choice(acts), rnd.choice(kinds), rnd.choice(["P0", "P1", "P2"]))
              for i in range(rnd.randint(1, 3))]
        mag = (RepairMagnitude(rnd.choice([0.2, 1.0, 2.0, 2.4]), rnd.choice([0.0, 0.3, 0.6]))
               if k == "repaired" else None)
        steps.append(TickInput(t=t, outcome=k, violations=tuple(vs), magnitude=mag, **flags))
    return steps


def test_invariants_hold_on_random_episodes():
    """Structural properties checked on every tick of 60 noisy episodes, rather
    than on hand-picked cases:
      * every transition is a listed edge, between the states that edge names;
      * a mode is requested only on a transition, and it is the new state's;
      * Brake returns to Normal only after T_recover with nothing enforced
        violated, and a Loiter always holds for at least T_recover;
      * the autopilot states stream no setpoint, except the GUIDED hand-over tick;
      * no stop is ever streamed on a tick whose input says a stop is illegal;
      * a finished episode never moves again."""
    cfg = FSMConfig()
    allowed = {}
    for e, (src, dst, _, _) in EDGES.items():
        for s in src.split("|"):
            allowed.setdefault(e, set()).add((s, dst))
    n_transitions = n_stop_illegal = withheld = 0
    for seed in range(60):
        f = EscalationFSM(cfg)
        loiter_in, clean_since, done = None, None, False
        for inp in _random_episode(seed):
            o = f.step(inp)
            assert o.setpoint in SETPOINTS and o.state in tuple(S), (seed, o.setpoint)
            enforced = [h for h in inp.violations if h.effective_action != "monitor_only"]
            clean_since = None if enforced else (inp.t if clean_since is None else clean_since)
            if done:
                assert o.transition is None and o.set_mode is None and o.setpoint == "none"
                continue
            if o.transition:
                n_transitions += 1
                src, dst = o.transition.split("->")
                assert (src, dst) in allowed[o.edge], (seed, o.transition, o.edge)
                if dst == "[*]":
                    done = True
                    assert o.set_mode is None
                else:
                    want = {"Loiter": "LOITER", "RTL": "RTL", "Land": "LAND"}.get(dst)
                    if dst == "Normal" and src == "Loiter":
                        want = "GUIDED"
                    assert o.set_mode == want, (seed, o.transition, o.set_mode)
                if o.edge == "G7":
                    assert clean_since is not None and inp.t - clean_since >= cfg.t_recover_s - 1e-9
                if dst == "Loiter":
                    loiter_in = inp.t
                if o.edge == "G8":
                    assert inp.t - loiter_in >= cfg.t_recover_s - 1e-9, (seed, inp.t, loiter_in)
            else:
                assert o.set_mode is None, (seed, o.reason)
            if o.state in (S.LOITER, S.RTL, S.LAND):
                handover = o.transition and o.before in (S.NORMAL, S.BRAKE)
                want = ("pass" if inp.stop_illegal else "brake") if handover else "none"
                assert o.setpoint == want, (seed, o.reason)
            if inp.stop_illegal:
                n_stop_illegal += 1
                assert o.setpoint != "brake", (seed, o.reason)
                withheld += int(o.record["stop_withheld"])
    assert n_transitions > 200, n_transitions       # the episodes are not trivial
    # 2157 stop-illegal ticks, 21 of them ones where a stop was wanted and
    # withheld: few, but enough that the invariant is exercised, not vacuous.
    assert n_stop_illegal > 1000 and withheld > 10, (n_stop_illegal, withheld)


def test_soft_only_random_episodes_never_leave_guided():
    for seed in range(60):
        f = EscalationFSM()
        for inp in _random_episode(seed, soft_only=True):
            assert f.step(inp).set_mode is None
        assert f.summary()["max_state"] in ("Normal", "Brake"), seed


def test_audit_record_carries_the_grants_fields_and_the_config_hash():
    cfg = FSMConfig()
    o = EscalationFSM(cfg).tick(0.0, "repaired", [hit()], SMALL)
    rec = json.loads(json.dumps(o.record))                       # serialisable as-is
    for k in ("fsm_state_before", "fsm_state_after", "violations", "reason",
              "set_mode", "setpoint", "risk_level", "theta", "n_in_t"):
        assert k in rec, k
    assert rec["fsm_config_hash"] == cfg.digest
    assert FSMConfig(n_violations=4).digest != cfg.digest


# --------------------------------------------------------------------------- #
# YAML-overridable thresholds (Safety Shield PDF p4, p7)
# --------------------------------------------------------------------------- #

def _yaml(text):
    p = Path(tempfile.mkdtemp()) / "escalation.yaml"
    p.write_text(text, encoding="utf-8")
    return p


def test_yaml_profile_overrides_with_the_grants_symbols():
    p = _yaml("escalation:\n  N: 4\n  T: 6\n  theta_lateral_m: 1.5\n"
              "  profiles:\n    night:\n      T_recover: 3\n      loiter_timeout_s: null\n")
    c = FSMConfig.from_yaml(p)
    assert (c.n_violations, c.window_s, c.theta_lateral_m, c.theta_vertical_m,
            c.t_recover_s) == (4, 6, 1.5, 0.5, 2.0)
    n = FSMConfig.from_yaml(p, profile="night")
    assert (n.n_violations, n.t_recover_s, n.loiter_timeout_s) == (4, 3, None)
    raises(KeyError, FSMConfig.from_yaml, p, profile="dawn")


def test_a_misspelt_threshold_is_refused_not_ignored():
    e = raises(ValueError, FSMConfig.from_yaml, _yaml("escalation:\n  t_recovr: 5\n"))
    assert "t_recovr" in str(e)
    raises(ValueError, FSMConfig.from_mapping, {"N": 3, "n_violations": 4})
    e = raises(ValueError, FSMConfig.from_mapping,
               {"profiles": {"night": {"T_recover": 3, "t_recover_s": 4}}}, "night")
    assert "night" in str(e)


def test_a_profile_may_override_in_the_other_spelling():
    """'Overridable per mission profile' (p4). A base written with the grant's
    symbols and a profile written with field names (or the reverse) is an
    override, not a key given twice."""
    data = {"escalation": {"N": 3, "t_recover_s": 2.0,
                           "profiles": {"tight": {"n_violations": 2, "T_recover": 3}}}}
    c = FSMConfig.from_mapping(data, profile="tight")
    assert (c.n_violations, c.t_recover_s) == (2, 3.0)
    assert FSMConfig.from_mapping(data).n_violations == 3


def test_one_set_of_thresholds_has_one_hash():
    """YAML 'T: 5' arrives as an int; json writes 5 and 5.0 differently. The
    hash must follow the thresholds, not their spelling, and keep all 64 hex
    digits like policy_hash."""
    assert FSMConfig(window_s=5).digest == FSMConfig().digest
    assert FSMConfig.from_mapping({"T": 5, "T_recover": 2, "theta_lateral": 2,
                                   "theta_vertical_m": 0.5, "loiter_timeout_s": 30}
                                  ).digest == FSMConfig().digest
    assert len(FSMConfig().digest) == len("sha256:") + 64
    assert FSMConfig(window_s=5.5).digest != FSMConfig().digest


def test_config_values_are_checked():
    raises(ValueError, FSMConfig, n_violations=0)
    raises(ValueError, FSMConfig, n_violations=True)
    raises(ValueError, FSMConfig, window_s=0)
    raises(ValueError, FSMConfig, theta_vertical_m=float("inf"))
    raises(ValueError, FSMConfig, t_recover_s=-1)
    raises(ValueError, FSMConfig, event_mode="sometimes")
    e = raises(ValueError, FSMConfig, loiter_timeout_s=2.0)     # == T_recover
    assert "never recover" in str(e)


def test_tick_counting_is_a_knob_and_it_changes_the_answer():
    """A sustained violation is one onset by default and three ticks in
    'tick' mode. The grant leaves this to stress-run data (p7)."""
    def run(mode):
        f = EscalationFSM(FSMConfig(event_mode=mode))
        return [f.tick(t, "repaired", [hit()], SMALL).state for t in clock(3)]
    assert run("onset") == [S.BRAKE, S.BRAKE, S.BRAKE]
    assert run("tick") == [S.BRAKE, S.BRAKE, S.LOITER]


# --------------------------------------------------------------------------- #
# Integration helpers, against the real Shield and policy
# --------------------------------------------------------------------------- #

def test_magnitude_from_actions_needs_an_explicit_horizon():
    raw, em = {"vx": 8.0, "vy": 0.0, "vz": 0.0}, {"vx": 4.0, "vy": 3.0, "vz": 1.0}
    m = magnitude_from_actions(raw, em, 0.5)
    assert math.isclose(m.lateral_m, math.hypot(-4, 3) * 0.5)
    assert math.isclose(m.vertical_m, 0.5)
    raises(TypeError, magnitude_from_actions, raw, em)          # no default
    for h in (0, -1, float("nan"), True):
        raises(ValueError, magnitude_from_actions, raw, em, h)
    raises(ValueError, magnitude_from_actions, {"vx": float("nan"), "vy": 0, "vz": 0}, em, 1.0)
    # With caps the raw action is clamped first: (8, 0, 0) -> (4, 0, 0).
    from guardrail.fsm import KinematicCaps
    m = magnitude_from_actions(raw, em, 0.5, KinematicCaps(4.0, 2.0))
    assert math.isclose(m.lateral_m, 3.0 * 0.5) and math.isclose(m.vertical_m, 0.5)
    m = magnitude_from_actions({"vx": 0, "vy": 0, "vz": -6.0}, {"vx": 0, "vy": 0, "vz": -1.5},
                               1.0, KinematicCaps(4.0, 2.0))
    assert math.isclose(m.vertical_m, 0.5)                      # -6 -> -2, then to -1.5


def test_rules_resolve_from_the_loaded_policy():
    from guardrail import load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    hits = rules_from_policy(pol, ["nfz-square", "kin-caps", "action-finite", "nfz-square"])
    assert [h.rule_id for h in hits] == ["nfz-square", "kin-caps", "action-finite"]
    assert hits[0].violation_action == "project_fix" and hits[0].priority == "P0"
    assert hits[1].priority == "P1"
    # Not a policy rule (the Shield's own finiteness check): hard P0, and flagged.
    assert not hits[2].resolved and hits[2].hard and hits[2].priority == "P0"
    # A policy that uses the grant's direct actions (models.py cannot load one yet).
    stub = SimpleNamespace(constraints=[SimpleNamespace(id="nfz", violation_action="RTL",
                                                        constraint_type="hard", priority="P0")])
    assert rules_from_policy(stub, ["nfz"])[0].violation_action == "RTL"


def test_a_real_shield_decision_maps_onto_the_fsm():
    """The magnitude source is the integration's decision, and it decides the
    edge, but only for repairs theta governs.

    An overspeed clamp (8 -> 4 m/s) changes a speed, not a position, so it is
    0 m under BOTH sources. The velocity proxy used to call it 12 m over 3 s
    and send a pure speed clamp to X1, while the per-operator source called
    it 0 m: the two sources disagreed about which repairs theta applies to,
    not only about how to measure them. A fence slide is a position
    correction: the per-operator source refuses it until shield.py reports
    magnitude_m, and the proxy calls it 0.42 m over one tick and 12.7 m over
    3 s."""
    from guardrail import Action4D, Shield, State, load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    st = State(x=-20, y=-20, up=15)
    clean = tick_input_from_decision(0.0, sh.filter(st, Action4D(vx=2.0)), pol)
    assert clean.outcome == "clean" and clean.violations == ()
    d = sh.filter(st, Action4D(vx=8.0))
    assert {r.operator for r in d.repairs} == {"SpeedClamp"}, d.repairs
    tick = tick_input_from_decision(0.0, d, pol)                # per-operator path
    assert tick.outcome == "repaired" and "kin-caps" in [h.rule_id for h in tick.violations]
    assert (tick.magnitude.lateral_m, tick.magnitude.vertical_m) == (0.0, 0.0)
    assert EscalationFSM().step(tick).edge == "G1"
    for h in (0.1, 3.0):
        o = EscalationFSM().step(tick_input_from_decision(0.0, d, pol, h))
        assert o.edge == "G1" and o.record["magnitude"]["lateral_m"] == 0.0, (h, o.reason)
    braked = SimpleNamespace(violations=d.violations, repairs=d.repairs, braked=True,
                             emitted_violations=[], raw=d.raw, emitted=d.emitted)
    assert tick_input_from_decision(0.0, braked, pol).outcome == "blocked"

    slide = sh.filter(State(x=0, y=15, up=15), Action4D(vx=3.0))
    assert {r.operator for r in slide.repairs} == {"GeofenceSlide"}, slide.repairs
    e = raises(ValueError, tick_input_from_decision, 0.0, slide, pol)
    assert "GeofenceSlide" in str(e) and "magnitude_m" in str(e)
    one_tick = tick_input_from_decision(0.0, slide, pol, 0.1)
    assert math.isclose(one_tick.magnitude.lateral_m, math.hypot(3.0, 3.0) * 0.1)
    assert EscalationFSM().step(one_tick).edge == "G1"
    assert EscalationFSM().step(tick_input_from_decision(0.0, slide, pol, 3.0)).edge == "X1"


def test_the_proxy_measures_from_the_clamped_raw_action():
    """shield.py clamps first, then the position operators work on the clamped
    action. Measured from the unclamped raw action, the proxy added the
    clamp's delta-v to the position correction: on the delivered flights
    every theta-governed tick also carried a SpeedClamp, and the one
    escalation at h = 0.1 s was a ClimbClamp (7.04 -> 2.00 m/s) alone."""
    from guardrail import Action4D, Shield, State, load_policy
    from guardrail.fsm import KinematicCaps, repair_magnitude
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    caps = KinematicCaps.from_policy(pol)
    assert (caps.speed_max_mps, caps.climb_rate_max_mps) == (4.0, 2.0)
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    st = State(x=0, y=15, up=15)
    # 8 m/s at the zone: SpeedClamp to 4 m/s, then GeofenceSlide swaps the
    # 4 m/s into-zone component for a 4 m/s tangent.
    d = sh.filter(st, Action4D(vx=8.0))
    assert [r.operator for r in d.repairs] == ["SpeedClamp", "GeofenceSlide"], d.repairs
    m = tick_input_from_decision(0.0, d, pol, 1.0).magnitude
    assert math.isclose(m.lateral_m, math.hypot(4.0, 4.0))            # not hypot(8, 4)
    assert math.isclose(magnitude_from_actions(d.raw, d.emitted, 1.0).lateral_m,
                        math.hypot(8.0, 4.0))                         # the old number
    # Without caps the proxy is refused on a clamped tick, not measured the old way.
    e = raises(ValueError, repair_magnitude, d.repairs, d.raw, d.emitted, 1.0)
    assert "caps" in str(e)
    raises(ValueError, tick_input_from_decision, 0.0, d, None, 1.0)
    # Vertical: ClimbClamp 5 -> 2, AltitudeFix 2 -> 1.67. The position
    # operator moved vz by 0.33 m/s, inside theta at h = 1 s; the old
    # measurement called it 3.33 m and escalated.
    d2 = sh.filter(st, Action4D(vx=3.0, vz_up=5.0))
    assert {"ClimbClamp", "AltitudeFix"} <= {r.operator for r in d2.repairs}, d2.repairs
    v = tick_input_from_decision(0.0, d2, pol, 1.0)
    assert math.isclose(v.magnitude.vertical_m, 2.0 - d2.emitted.vz_up)
    assert not v.magnitude.vertical_m > 0.5
    # Caps that contradict the decision (a raw action over them, no clamp
    # fired) mean the wrong policy was given for this run: refused.
    slide = sh.filter(st, Action4D(vx=3.0))
    e = raises(ValueError, repair_magnitude, slide.repairs, slide.raw, slide.emitted, 1.0,
               caps=KinematicCaps(1.0, 1.0))
    assert "wrong policy" in str(e)
    raises(ValueError, KinematicCaps, 0.0, 2.0)
    raises(ValueError, KinematicCaps, 4.0, float("nan"))
    assert KinematicCaps.from_policy(SimpleNamespace(constraints=[])) is None


def test_recovery_operators_are_exempt_from_theta_as_in_the_reference():
    """The PI's reference exempts recovery operators from the magnitude cap
    (safety_shield/repair.py: capping them "would re-introduce the 'brake
    while inside = deadlock' failure"). An aircraft 3 m inside the no-fly
    square gets a GeofenceEscape. Before this test, the proxy at h = 1 s
    measured that escape as 3 m, over theta, and the FSM answered X1 with a
    zero setpoint: it would have frozen the aircraft inside a P0 zone."""
    from guardrail import Action4D, Shield, State, load_policy
    from guardrail.fsm import magnitude_from_repairs
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    sh = Shield(pol, lookahead_s=3.0, dt=0.5)
    inside = State(x=10, y=15, up=15)
    d = sh.filter(inside, Action4D(vx=1.0))
    assert {r.operator for r in d.repairs} == {"GeofenceEscape"}, d.repairs
    assert sh.state_is_unsafe(inside)                    # standing still here is illegal
    tick = tick_input_from_decision(0.0, d, pol, 1.0)
    assert (tick.magnitude.lateral_m, tick.magnitude.recovery_exempt) == (0.0, ("GeofenceEscape",))
    o = EscalationFSM().step(tick)
    assert (o.edge, o.setpoint) == ("G1", "pass"), o.reason
    assert o.record["magnitude"]["recovery_exempt"] == ["GeofenceEscape"]

    # The per-operator source applies the same rule, and keeps the projection.
    m = magnitude_from_repairs([
        {"operator": "GeofenceSlide", "magnitude_m": 0.7, "axis": "lateral"},
        {"operator": "GeofenceEscape", "magnitude_m": 6.0, "axis": "lateral"},
        {"operator": "StandoffRecover"}])
    assert math.isclose(m.lateral_m, 0.7) and not m.exceeds(FSMConfig())
    assert m.recovery_exempt == ("GeofenceEscape", "StandoffRecover")
    # An operator can declare itself a recovery...
    assert magnitude_from_repairs([{"operator": "CorridorReturn", "recovery": True}]).lateral_m == 0
    raises(ValueError, magnitude_from_repairs, [{"operator": "CorridorReturn"}])
    raises(ValueError, magnitude_from_repairs,
           [{"operator": "CorridorReturn", "recovery": False}])
    # ...but False cannot un-exempt a named one. A Pydantic Repair with a
    # `recovery: bool = False` default writes False on EVERY repair, and that
    # must not quietly cap the escape out of a no-fly zone again.
    m = magnitude_from_repairs([{"operator": "GeofenceEscape", "recovery": False,
                                 "magnitude_m": 6.0, "axis": "lateral"}])
    assert (m.lateral_m, m.recovery_exempt) == (0.0, ("GeofenceEscape",))
    raises(ValueError, magnitude_from_repairs, [{"operator": "GeofenceEscape", "recovery": "yes"}])
    raises(ValueError, magnitude_from_repairs,
           [{"operator": "GeofenceEscape", "magnitude_m": -1.0, "axis": "lateral"}])


def test_a_stop_the_shield_calls_illegal_is_never_streamed():
    """Where standing still breaks a rule (Shield.state_is_unsafe), a zero
    setpoint holds the aircraft in the violation. The FSM still escalates,
    but it streams the Shield's own action instead of the stop."""
    for kw in (dict(outcome="blocked", violations=[hit()]),
               dict(outcome="repaired", violations=[hit()], magnitude=BIG),
               dict(outcome="repaired", violations=[hit(action="brake")], magnitude=SMALL)):
        o = EscalationFSM().tick(0.0, stop_illegal=True, **kw)
        assert (o.edge, o.setpoint) == ("X1", "pass"), (kw, o.setpoint)
        assert o.record["stop_withheld"] is True and "stop withheld" in o.reason
        assert o.record["flags"]["stop_illegal"] is True
        assert EscalationFSM().tick(0.0, **kw).setpoint == "brake", kw   # legal stop: unchanged
    f = EscalationFSM()
    f.tick(0.0, "repaired", [hit()], BIG, stop_illegal=True)
    o = f.tick(0.1, "repaired", [hit()], BIG, stop_illegal=True)
    assert (o.edge, o.set_mode, o.setpoint) == ("G3", "LOITER", "pass"), o.reason
    assert f.tick(0.2, "repaired", [hit()], BIG, stop_illegal=True).setpoint == "none"
    # A trusted repair is flown either way, and nothing is "withheld".
    o = EscalationFSM().tick(0.0, "repaired", [hit()], SMALL, stop_illegal=True)
    assert (o.setpoint, o.record["stop_withheld"]) == ("pass", False)
    raises(ValueError, TickInput, t=0.0, outcome="clean", stop_illegal=1)


def test_stop_illegal_is_read_from_a_blocked_decision():
    """shield.py brakes only where standing still is legal. A blocked decision
    that did not brake flew a best-effort recovery because the stop itself
    breaks a rule, and a brake that still re-checks dirty had no legal option.
    The node passes Shield.state_is_unsafe explicitly; the inference covers
    replays and callers that do not."""
    from guardrail.fsm import tick_input_from_row
    v = SimpleNamespace(rule_id="nfz-square")
    raw = {"vx": 1.0, "vy": 0.0, "vz_up": 0.0}

    def dec(**k):
        base = dict(violations=[v], repairs=[SimpleNamespace(operator="ClearanceEscape")],
                    braked=False, emitted_violations=[], raw=raw, emitted=raw)
        base.update(k)
        return SimpleNamespace(**base)
    best_effort = tick_input_from_decision(0.0, dec(emitted_violations=[v]), None)
    assert (best_effort.outcome, best_effort.stop_illegal) == ("blocked", True)
    legal_brake = tick_input_from_decision(0.0, dec(braked=True), None)
    assert (legal_brake.outcome, legal_brake.stop_illegal) == ("blocked", False)
    assert tick_input_from_decision(0.0, dec(braked=True, emitted_violations=[v]),
                                    None).stop_illegal is True
    assert tick_input_from_decision(0.0, dec(emitted_violations=[v]), None,
                                    stop_illegal=False).stop_illegal is False
    slide = SimpleNamespace(operator="GeofenceSlide")
    rep = tick_input_from_decision(0.0, dec(repairs=[slide]), None, 1.0)
    assert (rep.outcome, rep.stop_illegal) == ("repaired", False)
    assert tick_input_from_decision(0.0, dec(repairs=[slide]), None, 1.0,
                                    stop_illegal=True).stop_illegal
    row = {"t": 0.1, "violations": [{"rule_id": "nfz"}], "repairs": [{"operator": "ClearanceEscape"}],
           "braked": False, "emitted_violations": [{"rule_id": "nfz"}], "raw": raw, "emitted": raw}
    assert tick_input_from_row(row, None, 0.1).stop_illegal is True
    assert tick_input_from_row(dict(row, braked=True, emitted_violations=[]),
                               None, 0.1).stop_illegal is False


def test_a_clearance_rescue_is_trusted_only_where_a_stop_is_illegal():
    """shield.py's P0 escape guard runs ClearanceEscape when "repair not
    converged". Where a stop is legal that is p4's "no operator converges":
    the projection is abandoned, the legal stop is streamed, and the FSM
    escalates. Where a stop is illegal it is a recovery, exempt like
    GeofenceEscape. Before, it was exempt by name in both cases and mapped to
    a trusted repair, so a non-converged chain passed as G1."""
    from guardrail.fsm import magnitude_from_repairs, tick_input_from_row
    v = SimpleNamespace(rule_id="bld-clearance", category="clearance")
    raw = {"vx": 2.2, "vy": -0.5, "vz_up": 0.0}
    out = {"vx": 0.0, "vy": -3.0, "vz_up": 0.0}
    d = SimpleNamespace(violations=[v], braked=False, emitted_violations=[], raw=raw,
                        emitted=out,
                        repairs=[SimpleNamespace(operator="ClearanceFix"),
                                 SimpleNamespace(operator="ClearanceEscape",
                                                 detail="repair not converged -> recovery heading")])
    legal = tick_input_from_decision(0.0, d, None, 1.0, stop_illegal=False)
    assert (legal.outcome, legal.stop_illegal) == ("blocked", False) and "p4" in legal.note
    o = EscalationFSM().step(legal)
    assert (o.edge, o.setpoint) == ("X1", "brake"), o.reason
    inside = tick_input_from_decision(0.0, d, None, 1.0, stop_illegal=True)
    assert inside.outcome == "repaired"
    assert inside.magnitude.recovery_exempt == ("ClearanceEscape",)
    # Unknown is refused: the answer depends on exactly this.
    e = raises(ValueError, tick_input_from_decision, 0.0, d, None, 1.0)
    assert "stop_illegal" in str(e)
    # The per-operator source draws the same line.
    rescue = {"operator": "ClearanceEscape", "magnitude_m": 4.0, "axis": "lateral"}
    assert magnitude_from_repairs([rescue], stop_illegal=True).lateral_m == 0.0
    assert magnitude_from_repairs([rescue]).lateral_m == 4.0
    # A replayed log has no stop flag: the rescue is read as not converged,
    # and said so. With tools/rescore_kpis.py's `unsafe` reconstructed, it is not.
    row = {"t": 0.0, "violations": [{"rule_id": "bld-clearance", "category": "clearance"}],
           "repairs": [{"operator": "ClearanceEscape"}], "braked": False,
           "emitted_violations": [], "raw": raw, "emitted": out}
    r = tick_input_from_row(row, None, 1.0)
    assert r.outcome == "blocked" and "does not record" in r.note
    assert tick_input_from_row(dict(row, unsafe=True), None, 1.0).outcome == "repaired"
    assert tick_input_from_row(dict(row, unsafe_rules=[]), None, 1.0).outcome == "blocked"


def test_stop_illegal_counts_only_hard_enforced_rules():
    """Pass what Shield.state_is_unsafe returns; it is resolved through the
    policy. A soft rule is capped at brake, so a stop that breaks only a soft
    rule is a response it already allows; monitor_only enforces nothing."""
    pol = SimpleNamespace(constraints=[
        SimpleNamespace(id="nfz", violation_action="repair", constraint_type="hard", priority="P0"),
        SimpleNamespace(id="env", violation_action="brake", constraint_type="soft", priority="P1"),
        SimpleNamespace(id="watch", violation_action="monitor_only", constraint_type="hard",
                        priority="P2")])
    raw = {"vx": 1.0, "vy": 0.0, "vz_up": 0.0}
    d = SimpleNamespace(violations=[SimpleNamespace(rule_id="nfz", category="geofence")],
                        repairs=[SimpleNamespace(operator="GeofenceEscape")], braked=False,
                        emitted_violations=[], raw=raw, emitted=raw)
    viol = lambda rid: SimpleNamespace(rule_id=rid, category="geofence")   # noqa: E731
    for unsafe, want in (([viol("nfz")], True), ([viol("env")], False),
                         ([viol("watch")], False), ([], False),
                         ([viol("env"), viol("nfz")], True), ([viol("unknown")], True)):
        got = tick_input_from_decision(0.0, d, pol, 1.0, stop_illegal=unsafe).stop_illegal
        assert got is want, (unsafe, got)
    raises(ValueError, tick_input_from_decision, 0.0, d, pol, 1.0, stop_illegal="yes")


def test_a_position_repair_without_magnitude_is_refused_not_zeroed():
    """Today's shield.py Repair has no magnitude_m. A GeofenceSlide read as a
    0 m correction would wave every fence repair through theta, the silent
    zero this project keeps finding."""
    from guardrail.fsm import magnitude_from_repairs
    e = raises(ValueError, magnitude_from_repairs,
               [SimpleNamespace(operator="GeofenceSlide", detail="nfz: removed 4 m/s")])
    assert "GeofenceSlide" in str(e)
    m = magnitude_from_repairs([
        {"operator": "SpeedClamp"},                                    # exempt
        {"operator": "GeofenceSlide", "magnitude_m": 0.7, "axis": "lateral"},
        {"operator": "ClearanceFix", "magnitude_m": 0.6, "axis": "lateral"},
        {"operator": "AltitudeFix", "magnitude_m": 0.3, "axis": "vertical"}])
    assert math.isclose(m.lateral_m, 1.3) and math.isclose(m.vertical_m, 0.3)
    assert m.exceeds(FSMConfig(theta_lateral_m=1.2)) and not m.exceeds(FSMConfig())
    raises(ValueError, magnitude_from_repairs,
           [{"operator": "AltitudeFix", "magnitude_m": 0.3, "axis": "up"}])
    raises(ValueError, magnitude_from_repairs,
           [{"operator": "AltitudeFix", "magnitude_m": float("nan"), "axis": "vertical"}])


def _rows(n, dv=5.0, braked_at=()):
    """flight_log.jsonl rows: a stub pilot corrected by dv m/s laterally every tick."""
    out = []
    for i, t in enumerate(clock(n, t0=0.1)):
        out.append({"t": t, "tick": i + 1, "raw": {"vx": 6.0, "vy": 0.0, "vz_up": 0.0},
                    "emitted": {"vx": 6.0 - dv, "vy": 0.0, "vz_up": 0.0},
                    "violations": [{"rule_id": "nfz-square"}],
                    "repairs": [{"operator": "GeofenceSlide"}],
                    "braked": i in braked_at, "emitted_violations": []})
    return out


def test_replay_reports_the_first_escalation_and_the_horizon_it_used():
    from guardrail.fsm import replay_rows
    rows = _rows(30)
    long_h = replay_rows(rows, horizon_s=3.0)
    assert long_h["first_escalation"]["edge"] == "G3" and long_h["first_escalation"]["tick"] == 2
    assert long_h["repaired_over_theta"] == 30 and long_h["hard_onsets"] == 1
    tick_h = replay_rows(rows, horizon_s=0.1)              # 0.5 m: inside theta
    assert tick_h["first_escalation"] is None and tick_h["repaired_over_theta"] == 0
    assert "velocity proxy" in tick_h["magnitude_source"]
    assert long_h["repaired_theta_exempt_only"] == 0
    # Without a horizon the grant's per-operator field is required, and these
    # rows (like every delivered log) do not have it.
    e = raises(ValueError, replay_rows, rows)
    assert "GeofenceSlide" in str(e)
    # A stub cruising 6 m/s against a 4 m/s cap: speed clamps only. Theta never
    # judges them, under either source, and the replay says so instead of
    # reporting "0 over theta" as if they had been measured and passed.
    clamps = [dict(r, repairs=[{"operator": "SpeedClamp"}],
                   emitted={"vx": 4.0, "vy": 0.0, "vz_up": 0.0}) for r in rows]
    for h in (0.1, 3.0, None):
        out = replay_rows(clamps, horizon_s=h)
        assert out["first_escalation"] is None and out["repaired_over_theta"] == 0, h
        assert out["repaired_theta_exempt_only"] == 30 == out["repaired_ticks"], h


def _clamped_rows(n, stop_after=None):
    """Rows the way the delivered stub flights look: 6 m/s at a ring, a speed
    clamp to 4 m/s and a StandoffHold on every tick, both rules hard."""
    out = []
    for i, t in enumerate(clock(n, t0=0.1)):
        row = {"t": t, "tick": i + 1, "raw": {"vx": 6.0, "vy": 0.0, "vz_up": 0.0},
               "emitted": {"vx": 0.0, "vy": 3.0, "vz_up": 0.0},
               "violations": [{"rule_id": "kin-caps", "category": "kinematic"},
                              {"rule_id": "standoff-pedestrian", "category": "standoff"}],
               "repairs": [{"operator": "SpeedClamp"}, {"operator": "StandoffHold"}],
               "braked": False, "emitted_violations": []}
        if stop_after is not None:
            row["unsafe_rules"] = ["standoff-pedestrian"] if i >= stop_after else []
        out.append(row)
    return out


def test_replay_of_clamped_rows_needs_the_caps_and_cites_the_ring():
    from guardrail import load_policy
    from guardrail.fsm import KinematicCaps, replay_rows
    rows = _clamped_rows(30)
    e = raises(ValueError, replay_rows, rows, 1.0)
    assert "caps" in str(e)
    pol = load_policy(ROOT / "policies" / "sitl_pedestrian.yaml")
    out = replay_rows(rows, 1.0, policy=pol)
    assert out["kinematic_caps"] == {"speed_max_mps": 4.0, "climb_rate_max_mps": 2.0}
    # Clamped raw is (4, 0); emitted (0, 3): 5 m of correction at h = 1 s,
    # where the unclamped raw would have said hypot(6, 3) = 6.7 m.
    assert out["first_escalation"]["edge"] == "G3"
    assert "standoff-pedestrian" in out["first_escalation"]["reason"]
    assert "kin-caps" not in out["first_escalation"]["reason"]
    assert (out["repaired_theta_governed"], out["governed_with_velocity_clamp"]) == (30, 30)
    # h = 0.3 s: 1.5 m, inside theta. Measured from the unclamped raw action
    # it would be 2.01 m and every tick would be "over theta".
    assert replay_rows(rows, 0.3, caps=KinematicCaps(4.0, 2.0))["repaired_over_theta"] == 0
    # Without a policy the category still marks kin-caps kinematic.
    no_pol = replay_rows(rows, 1.0, caps=KinematicCaps(4.0, 2.0))
    assert "standoff-pedestrian" in no_pol["first_escalation"]["reason"]


def test_replay_reads_a_reconstructed_unsafe_flag_and_finds_x6():
    """tools/rescore_kpis.py writes `unsafe_rules` per row (Shield.state_is_unsafe).
    With it, the replay can see X6; without it, every row is stop-legal and
    the replay says how many rows recorded the flag."""
    from guardrail.fsm import KinematicCaps, replay_rows
    caps = KinematicCaps(4.0, 2.0)
    rows = _clamped_rows(120, stop_after=10)
    out = replay_rows(rows, 0.1, caps=caps)
    assert out["first_escalation"]["edge"] == "X6", out["first_escalation"]
    assert math.isclose(out["first_escalation"]["t_s"], rows[60]["t"])  # 5 s after row 10
    assert out["rows_recording_stop_illegal"] == 120 and out["stop_illegal_ticks"] == 110
    plain = replay_rows(_clamped_rows(120), 0.1, caps=caps)
    assert plain["first_escalation"] is None and plain["rows_recording_stop_illegal"] == 0


def test_replay_refuses_a_shield_off_control_arm():
    from guardrail.fsm import tick_input_from_row
    row = {"t": 0.1, "violations": [{"rule_id": "nfz"}], "repairs": [], "braked": False,
           "raw": {"vx": 1, "vy": 0, "vz_up": 0}, "emitted": {"vx": 1, "vy": 0, "vz_up": 0}}
    e = raises(ValueError, tick_input_from_row, row, None, 0.1)
    assert "control arm" in str(e)
    assert tick_input_from_row(dict(row, braked=True), None, 0.1).outcome == "blocked"
    assert tick_input_from_row({"t": 0.2, "violations": []}, None, 0.1).outcome == "clean"


def test_replay_cli_prints_one_json_line_per_run():
    import contextlib
    import io
    from guardrail.fsm import _main
    d = Path(tempfile.mkdtemp())
    (d / "flight_log.jsonl").write_text("\n".join(json.dumps(r) for r in _rows(5)) + "\n",
                                        encoding="utf-8")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        assert _main(["replay", str(d), "--horizon", "3.0"]) == 0
    out = json.loads(buf.getvalue())
    assert out["first_escalation"]["edge"] == "G3" and out["horizon_s"] == 3.0
    # No horizon and no magnitude_m in the log: refused with the reason and a
    # non-zero exit, and nothing on stdout that a script could mistake for a result.
    buf, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
        assert _main(["replay", str(d)]) == 2
    assert buf.getvalue() == "" and "GeofenceSlide" in err.getvalue()
    # Clamped rows: refused without the policy, measured clamp-free with it,
    # and the output says which policy and whether its hash is the run's.
    c = Path(tempfile.mkdtemp())
    (c / "flight_log.jsonl").write_text("\n".join(json.dumps(r) for r in _clamped_rows(5)),
                                        encoding="utf-8")
    (c / "manifest.json").write_text(json.dumps({"policy_hash": "sha256:0123456789abcdef"}),
                                     encoding="utf-8")
    buf, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
        assert _main(["replay", str(c), "--horizon", "1.0"]) == 2
    assert buf.getvalue() == "" and "caps" in err.getvalue()
    pol = str(ROOT / "policies" / "sitl_pedestrian.yaml")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        assert _main(["replay", str(c), "--horizon", "1.0", "--policy", pol]) == 0
    out = json.loads(buf.getvalue())
    assert out["policy"] == pol and out["policy_hash_run"] == "sha256:0123456789abcdef"
    assert out["policy_hash_matches_run"] is False and out["kinematic_caps"]["speed_max_mps"] == 4.0
    # A missing log is a refusal too, not a traceback.
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        assert _main(["replay", str(Path(tempfile.mkdtemp()) / "nope")]) == 2


# --------------------------------------------------------------------------- #
# The KPI: fail-safe trigger correctness and its nulls
# --------------------------------------------------------------------------- #

LABELS = [True] * 3 + [False] * 7            # 3 episodes expect a fail-safe, 7 do not


def _score(triggered):
    return score_failsafe_triggers([{"expected_failsafe": e, "triggered": t}
                                    for e, t in zip(LABELS, triggered)])


def test_a_working_fail_safe_beats_both_nulls():
    r = _score(LABELS)
    assert r["failsafe_trigger_correctness"] == 1.0 and r["beats_null"] is True
    assert (r["null_always_trigger"], r["null_never_trigger"]) == (0.3, 0.7)
    assert r["discriminating"] and r["warnings"] == []


def test_the_never_trigger_shield_scores_exactly_the_null():
    """The Shield as delivered before this module: no fail-safe ever fires."""
    r = _score([False] * 10)
    assert r["failsafe_trigger_correctness"] == r["null_never_trigger"] == 0.7
    assert r["beats_null"] is False and r["missed_triggers"] == 3


def test_the_always_trigger_shield_scores_exactly_the_other_null():
    r = _score([True] * 10)
    assert r["failsafe_trigger_correctness"] == r["null_always_trigger"] == 0.3
    assert r["beats_null"] is False and r["false_triggers"] == 7


def test_false_and_missed_triggers_both_cost():
    r = _score([True, True, False] + [False] * 6 + [True])
    assert (r["true_triggers"], r["missed_triggers"], r["false_triggers"],
            r["true_non_triggers"]) == (2, 1, 1, 6)
    assert r["failsafe_trigger_correctness"] == 0.8


def test_a_set_with_no_expected_trigger_cannot_score_a_fail_safe():
    """100 % here is what a Shield with no fail-safe at all would also get, so
    the target is not judged: a dashboard reading meets_target sees no pass."""
    r = score_failsafe_triggers([(False, False)] * 400)
    assert r["failsafe_trigger_correctness"] == 1.0 and r["beats_null"] is False
    assert not r["discriminating"] and "never fires" in r["warnings"][0]
    assert r["meets_target"] is None and r["meets_target_at_95"] is None
    assert r["wilson_low_95"] > 0.99                 # the bound alone would "pass"


def test_a_set_where_every_episode_expects_a_trigger_is_not_judged_either():
    r = score_failsafe_triggers([(True, True)] * 400)
    assert not r["discriminating"] and r["beats_null"] is False
    assert r["meets_target"] is None and r["meets_target_at_95"] is None
    assert "every episode" in r["warnings"][0]
    # One label of the other kind makes the set discriminating again.
    r = score_failsafe_triggers([(True, True)] * 400 + [(False, False)])
    assert r["discriminating"] and r["meets_target"] is True


def test_no_episodes_is_undefined_not_zero_or_one():
    r = score_failsafe_triggers([])
    assert r["failsafe_trigger_correctness"] is None and r["beats_null"] is None
    assert r["meets_target"] is None and r["warnings"]


def test_unlabelled_and_unmeasured_episodes_are_counted_not_scored():
    r = score_failsafe_triggers([(True, True), (None, True), (False, None), (False, False)])
    assert (r["episodes"], r["scored"], r["unlabelled"], r["not_measured"]) == (4, 2, 1, 1)
    assert any("left out" in w for w in r["warnings"])


def test_labels_must_be_booleans():
    raises(ValueError, score_failsafe_triggers, [{"expected_failsafe": "false", "triggered": False}])
    raises(ValueError, score_failsafe_triggers, [(1, True)])
    raises(ValueError, score_failsafe_triggers, [{"triggered": True}])


def test_ninety_nine_percent_needs_381_error_free_episodes_to_show():
    r100 = score_failsafe_triggers([(True, True)] * 50 + [(False, False)] * 50)
    assert r100["meets_target"] is True and r100["meets_target_at_95"] is False
    assert r100["min_error_free_episodes_for_target_at_95"] == 381
    n = 381
    r = score_failsafe_triggers([(True, True)] * 100 + [(False, False)] * (n - 100))
    assert r["meets_target_at_95"] is True
    r = score_failsafe_triggers([(True, True)] * 100 + [(False, False)] * (n - 101))
    assert r["meets_target_at_95"] is False


def test_fsm_episodes_feed_the_scorer():
    """End to end: the FSM's own summary supplies `triggered`."""
    def episode(steps):
        f = EscalationFSM()
        for t, kw in steps:
            f.tick(t, **kw)
        return f.summary()["failsafe_triggered"]
    eps = [
        {"expected_failsafe": True,                       # an RTL rule is breached
         "triggered": episode([(0.0, dict(outcome="blocked", violations=[hit(action="RTL")]))])},
        {"expected_failsafe": False,                      # a small repair
         "triggered": episode([(0.0, dict(outcome="repaired", violations=[hit()],
                                          magnitude=SMALL))])},
        {"expected_failsafe": False, "triggered": episode([(0.0, {})])},
    ]
    r = score_failsafe_triggers(eps)
    assert r["failsafe_trigger_correctness"] == 1.0 and r["beats_null"] is True


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
