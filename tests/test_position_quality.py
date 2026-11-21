"""The position the Shield is handed, and every recovery at its cap
(2026-11-06; docs/DESIGN-escalation-fsm.md "X8", docs/RUBRIC-failsafe.md).

Run either way:
    pytest tests/test_position_quality.py -v
    python tests/test_position_quality.py

WHY THIS FILE EXISTS

The fail-safe KPI's missed triggers on the library and nightly profiles were
gps-dropout's frozen-fix cells (rubric E3): the Shield had no input that said
its pose was stale, so it flew on it for 7 s. Two nightly false triggers were
GPS noise inside the margin budget counted by N-in-T (rubric N2). And mean
time to safe was held up by three lateral recoveries still flying slower than
their cap. Since then:

  * State.fix_ok / State.pos_sigma_m: what a rail reports about the position
    (None, every rail on that date: nothing changes);
  * Shield._position_verdict and the FSM's X8 (-> RTL): the Shield
    dead-reckons on its own streamed commands while the fix is invalid, with
    a drift allowance (DR_DRIFT_MPS), and escalates once the error can no
    longer be certified by a hard P0 margin - the rubric's E3 on the
    Shield's own estimate; with a reported accuracy, 3 sigma beyond the
    margin near a boundary does the same;
  * TickInput.noise_marginal: with a reported accuracy, a ring re-touched
    within 3 sigma is not a new N-in-T onset;
  * recovery_law "cap_all", the default: every recovery at its cap, a zone's
    escape judged as flown through the path windows that open on the
    aircraft already take;
  * guardrail/kpi.py's false-trigger invariant leaves X8 out by its reason;
  * bundles: extras schema /5 records the report and the drift allowance,
    and a /1-/4 bundle re-flies blind.

These tests pin each of those, and that a rail reporting nothing flies
exactly as before.
"""
import copy
import functools
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from shapely.geometry import Point, Polygon                          # noqa: E402

import sweep_scenarios as SW                                         # noqa: E402
from guardrail import kpi as K                                       # noqa: E402
from guardrail.models import Action4D, Policy, State                 # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec            # noqa: E402
from guardrail.shield import (DEFAULT_RECOVERY_LAW, DR_DRIFT_MPS,    # noqa: E402
                              RECOVERY_LAWS)
from guardrail.shield import Shield as _Shield                       # noqa: E402


class Shield(_Shield):
    """The Shield with PathRepair OFF. Every flight this file pins (the
    rig's lane-23.6 / lane-15 timelines, X8 against E3's first tick) was
    flown before PathRepair joined the default chain (2026-11-17): a detour
    taken while the fix is still good moves the aircraft, and so every later
    edge time. What PathRepair changes in these cells is measured in
    docs/data/pathrepair_default_20261117/ (gps-dropout, gps-noise rows);
    its no-detour-without-a-fix gate is pinned in tests/
    test_shieldcore_repair.py."""

    def __init__(self, *a, path_repair=False, **kw):
        super().__init__(*a, path_repair=path_repair, **kw)


SW.Shield = Shield

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
DEFAULTS = LIBRARY.defaults
FAMILIES = LIBRARY.by_id()
E3_CELLS = [(15.0, [2.0, 9.0]), (15.0, [4.0, 11.0]), (23.6, [2.0, 9.0]),
            (23.6, [4.0, 11.0]), (23.6, [6.0, 13.0])]
N2_CELLS = [(15.0, [2.0, 4.0]), (23.6, [2.0, 4.0]), (15.0, [6.0, 13.0])]


def _cell(sid, samples=3, **params):
    for c in FAMILIES[sid].cells(samples=samples):
        if all(c.params.get(k) == v for k, v in params.items()):
            return c.spec
    raise KeyError(f"{sid}: no cell with {params}")


def _with(spec: ScenarioSpec, **changes) -> ScenarioSpec:
    d = spec.model_dump(mode="json", by_alias=True)
    for path, value in changes.items():
        node = d
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return ScenarioSpec.model_validate(d)


def _fly(spec, seed=0, **shield_kw):
    """run_episode, its Shields built with `shield_kw` (dr_drift_mps, ...)."""
    if not shield_kw:
        return SW.run_episode(spec, DEFAULTS, seed)
    orig = SW.Shield
    SW.Shield = functools.partial(orig, **shield_kw)
    try:
        return SW.run_episode(spec, DEFAULTS, seed)
    finally:
        SW.Shield = orig


def _edges(run):
    return [(t["edge"], round(t["t_s"], 1)) for t in (run.extra.get("fsm") or {}).get("transitions", [])]


def _x8_t(run):
    return next((t for e, t in _edges(run) if e == "X8"), None)


def _e3_onset(run):
    """The first tick on which the rubric's frozen-fix E3 holds
    (tools/failsafe_rubric.py e3_facts): the pose handed in is wrong by more
    than nfz-square's margin and the TRUE position is within that error of
    the zone. Read off a flight's own rows, which are the counterfactual's
    until the FSM first changes what flies."""
    pol = run.policy
    zones = [(Polygon([(v.x, v.y) for v in c.vertices]), float(c.margin_m or 0.0))
             for c in pol.constraints if c.type == "polygon_fence"
             and c.constraint_type == "hard" and c.priority == "P0"]
    margin = min(m for _, m in zones)
    for r in run.rows:
        p = r.get("perceived")
        if not p:
            continue
        err = math.hypot(r["x"] - p["x"], r["y"] - p["y"])
        here = Point(r["x"], r["y"])
        if err > margin and min(poly.distance(here) for poly, _ in zones) <= err:
            return round(r["t"], 1)
    return None


def _digest(rows):
    return hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()


# --------------------------------------------------------------------------- #
# X8: the frozen fix (gps-dropout)
# --------------------------------------------------------------------------- #

def test_x8_fires_on_the_tick_e3_first_holds_on_the_ideal_vehicle():
    """With no drift allowance the Shield's dead reckoning is exact on the
    ideal vehicle, and X8 is the rubric's E3 on it: the same tick, in every
    frozen-fix cell the rubric expects a fail-safe for. It returns (RTL)."""
    for lane, window in E3_CELLS:
        run = _fly(_cell("gps-dropout", lane_y=lane, window=window), dr_drift_mps=0.0)
        onset = _e3_onset(run)
        assert onset is not None, (lane, window)
        assert _x8_t(run) == onset, (lane, window, _edges(run), onset)
        assert run.extra["fsm"]["failsafe_triggered"], (lane, window)
        assert "RTL" in run.extra["fsm"]["visited"], (lane, window, run.extra["fsm"])


def test_x8_with_the_drift_allowance_comes_no_later_and_never_in_an_n2_cell():
    """The default allowance (DR_DRIFT_MPS) only makes X8 earlier, never
    later, and it never fires where the rubric expects no fail-safe: a
    dropout that ends 20 m out, or a fix frozen on a held aircraft."""
    assert DR_DRIFT_MPS > 0
    for lane, window in E3_CELLS:
        spec = _cell("gps-dropout", lane_y=lane, window=window)
        exact = _e3_onset(_fly(spec, dr_drift_mps=0.0))
        t = _x8_t(_fly(spec))
        assert t is not None and t <= exact, (lane, window, t, exact)
    for lane, window in N2_CELLS:
        run = _fly(_cell("gps-dropout", lane_y=lane, window=window))
        assert _x8_t(run) is None and not run.extra["fsm"]["failsafe_triggered"], (
            lane, window, _edges(run))
        assert _e3_onset(_fly(_cell("gps-dropout", lane_y=lane, window=window),
                              dr_drift_mps=0.0)) is None, (lane, window)


def test_x8_keeps_the_head_on_dropout_out_of_the_zone():
    """The finding the family is tracked for - frozen at 4 s, the aircraft
    ended 7 m inside nfz-square - does not happen once the rail reports the
    stale pose: X8 returns it 10 m short, and no tick is inside."""
    run = _fly(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]))
    assert run.extra["ticks_inside_fence_polygon"] == 0, run.extra["ticks_inside_fence_polygon"]
    assert run.extra["polygon_entries"] == 0 and run.extra["breaches"] == 0
    assert run.extra["min_true_dist_to_fence_m"] > 5.0, run.extra["min_true_dist_to_fence_m"]


# --------------------------------------------------------------------------- #
# a rail that reports nothing
# --------------------------------------------------------------------------- #

def test_fix_ok_none_and_true_and_no_sigma_fly_exactly_as_before():
    """None (the field absent), and True, are the position as every rail
    hands it today: every decision of a Shield flown into a zone's slide is
    identical, field for field, to one fed a State without the fields; a
    State dump without them is the old dump."""
    from guardrail import load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    flown = {}
    for label, extra in (("absent", {}), ("none", {"fix_ok": None, "pos_sigma_m": None}),
                         ("true", {"fix_ok": True})):
        sh = Shield(pol)
        x, y = -6.0, 0.0
        out = []
        for i in range(80):
            st = State(x=x, y=y, up=15.0, **extra)
            raw = Action4D(vx=3.0, vy=3.0)
            d = sh.filter(st, raw, t=i * 0.1)
            cmd = d.command if d.command is not None else Action4D()
            x, y = x + cmd.vx * 0.1, y + cmd.vy * 0.1
            out.append(json.dumps({"em": d.emitted.model_dump(),
                                   "vios": [v.model_dump() for v in d.violations],
                                   "fsm": [d.fsm_state_before, d.fsm_state_after, d.fsm_edge,
                                           d.setpoint], "rec": d.fsm_record}, sort_keys=True,
                                  default=str))
        flown[label] = out
    assert flown["none"] == flown["absent"]
    assert flown["true"] == flown["absent"]
    assert set(State(x=1.0, y=2.0, up=3.0).model_dump()) == {"x", "y", "up", "yaw_deg"}


def test_a_rail_that_withholds_the_report_flies_blind_as_before():
    """fix_quality_reported false: the frozen pose is handed on as is and
    the noise's sigma is not reported - the missed trigger and the noise
    flicker of every run before 2026-11-06, edge for edge."""
    blind = {"stress.fix_quality_reported": False}
    run = _fly(_with(_cell("gps-dropout", lane_y=23.6, window=[2.0, 9.0]), **blind))
    assert _edges(run) == [("G1", 9.0), ("G7", 11.4)], _edges(run)
    assert all("fix_ok" not in json.dumps(r.get("perceived")) for r in run.rows)
    flick = _fly(_with(_cell("gps-noise-within-margin", latency=0, pass_y=23.2, sigma=0.3),
                       **blind), seed=1001)
    assert [e for e, _ in _edges(flick)][:3] == ["G1", "G2", "G4"], _edges(flick)


# --------------------------------------------------------------------------- #
# noise: the reported accuracy
# --------------------------------------------------------------------------- #

def test_noise_inside_the_margin_is_not_counted_as_onsets_when_reported():
    """gps-noise-within-margin latency 0 / 23.2 / 0.3, the two nightly seeds
    that flickered into G2 and G4: with sigma reported the ring re-touched
    within 3 sigma is one violation, the aircraft slides on in Brake and
    reaches its goal, and X8 never fires (3 sigma = 0.9 m, inside the 1 m
    margin: rubric N2)."""
    for seed in (1001, 1003):
        run = _fly(_cell("gps-noise-within-margin", latency=0, pass_y=23.2, sigma=0.3),
                   seed=seed)
        edges = [e for e, _ in _edges(run)]
        assert "G2" not in edges and "G4" not in edges and "X8" not in edges, (seed, edges)
        assert not run.extra["fsm"]["failsafe_triggered"] and run.extra["reached_goal"], seed


def test_noise_beyond_the_margin_returns_and_the_sigma_branch_needs_a_report():
    """3 sigma beyond the margin with the request's forecast within it of a
    P0 zone: X8 (rubric E3 for noise, read against the planned route). The
    same State without a sigma, or with one whose 3 sigma is inside the
    margin, never takes X8."""
    from guardrail import load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    raw = Action4D(vx=4.0)                       # east of nfz-square's north face, along it
    edges = {}
    for sigma in (None, 0.3, 1.0):
        sh = Shield(pol)
        st = State(x=-14.0, y=23.4, up=15.0, pos_sigma_m=sigma)
        d = sh.filter(st, raw, t=0.0)
        edges[sigma] = (d.fsm_edge, d.set_mode, (d.fsm_record or {}).get("position_lost"))
    assert edges[None][0] != "X8" and edges[None][2] is None, edges
    assert edges[0.3][0] != "X8" and edges[0.3][2] is None, edges
    assert edges[1.0][:2] == ("X8", "RTL") and "3 sigma" in edges[1.0][2], edges
    for lane in (23.2, 23.6):
        run = _fly(_cell("gps-noise-beyond-margin", latency=5, pass_y=lane, sigma=1.0), seed=1001)
        assert _x8_t(run) is not None and run.extra["polygon_entries"] == 0, (lane, _edges(run))


def test_a_clean_tick_within_three_sigma_of_a_ring_is_marked_for_the_fsm():
    """The FSM's hysteresis input: a clean request whose forecast passes
    within 3 sigma of a hard zone's ring is `noise_marginal` in the record;
    with no sigma reported, or the ring farther than 3 sigma, it is not
    written at all (every other record is as it was)."""
    from guardrail import load_policy
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    raw = Action4D(vx=4.0)                       # 0.5 m outside the ring, along it
    rec = {}
    for sigma in (None, 0.3):
        for y in (24.5, 30.0):
            sh = Shield(pol)
            d = sh.filter(State(x=-14.0, y=y, up=15.0, pos_sigma_m=sigma), raw, t=0.0)
            assert not d.violations, (sigma, y, d.violations)
            rec[(sigma, y)] = (d.fsm_record or {}).get("noise_marginal")
    assert rec == {(None, 24.5): None, (None, 30.0): None,
                   (0.3, 24.5): True, (0.3, 30.0): None}, rec


def test_a_rail_running_its_own_fsm_gets_the_same_noise_marginal_verdict():
    """A rail's Shield has no FSM of its own (guardrail.replay.rail_shield),
    so until 2026-11-21 the ROS 2 node's FSM never saw `noise_marginal`: a
    stressed pose with pos_sigma_m (the node's PerceptionStress) would have
    counted ring re-touches as onsets the harness's FSM does not. Now the
    decision carries it, the very verdict the Shield's own FSM is given, for
    a state with pos_sigma_m only (None otherwise: every rail before that
    date), and it stays out of model_dump()."""
    from guardrail import load_policy
    from guardrail.replay import rail_shield
    from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S
    pol = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
    raw = Action4D(vx=4.0)
    for sigma in (None, 0.3):
        for y in (24.5, 30.0):
            st = State(x=-14.0, y=y, up=15.0, pos_sigma_m=sigma)
            own = (Shield(pol).filter(st, raw, t=0.0).fsm_record or {}).get("noise_marginal")
            d = rail_shield(pol, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S,
                            path_repair=False).filter(
                st, raw, t=0.0)
            if sigma is None:
                assert d.noise_marginal is None, (y, d.noise_marginal)
            else:
                assert d.noise_marginal is bool(own), (y, d.noise_marginal, own)
            assert "noise_marginal" not in d.model_dump()


# --------------------------------------------------------------------------- #
# recovery_law cap_all
# --------------------------------------------------------------------------- #

def _box(rid, x0, y0, x1, y1, **over):
    z = {"id": rid, "type": "polygon_fence", "constraint_type": "hard", "priority": "P0",
         "violation_action": "repair", "scope": "mission", "margin_m": 1.0,
         "altitude_floor_m": 0.0,
         "altitude_ceiling_m": 100.0,
         "vertices": [{"x": x0, "y": y0}, {"x": x1, "y": y0}, {"x": x1, "y": y1},
                      {"x": x0, "y": y1}]}
    z.update(over)
    return z


CAPS = {"id": "caps", "type": "kinematic_envelope", "constraint_type": "hard",
        "priority": "P1", "violation_action": "repair", "scope": "mission",
        "speed_max_mps": 4.0,
        "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
INSIDE = State(x=10.0, y=18.0, up=15.0)        # zone A's east edge 2 m away, its ring 3 m
CRAWL_EAST = Action4D(vy=0.3)                  # leaving A, legal by the inside rule


def _escape(law, b_ring_past_exit_m=None):
    rules = [_box("a", 0, 0, 20, 20), CAPS]
    if b_ring_past_exit_m is not None:
        y0 = 21.0 + b_ring_past_exit_m + 1.0     # B's ring starts 1 m before B
        rules.append(_box("b", 0, y0, 20, y0 + 6))
    sh = Shield(Policy.model_validate({"policy_id": "cap-all", "constraints": rules}),
                recovery_law=law)
    return sh.filter(INSIDE, CRAWL_EAST, t=0.0)


def test_cap_all_is_the_default_and_the_older_laws_stay_selectable():
    assert DEFAULT_RECOVERY_LAW == SW.DEFAULT_RECOVERY_LAW == "cap_all"
    assert set(RECOVERY_LAWS) == {"cap_all", "cap", "lookahead"}
    from guardrail import load_policy
    assert Shield(load_policy(ROOT / "policies" / "sim_demo_policy.yaml")).recovery_law == "cap_all"
    deep = _cell("radius-scaling-grow-deep")
    t2s = {law: SW.run_episode(deep, DEFAULTS, 0, recovery=law).extra
           for law in RECOVERY_LAWS}
    by = {law: K.compute(SW.run_episode(deep, DEFAULTS, 0, recovery=law).rows,
                         {})["max_time_to_safe_s"] for law in RECOVERY_LAWS}
    assert by["cap_all"] <= 3.5 and by["cap"] >= 6.5 and by["lookahead"] >= 6.5, by
    assert all(e["recovery_law"] == law for law, e in t2s.items()), t2s


def test_a_zone_is_left_at_the_cap_judged_as_flown_not_on_the_straight_20m_forecast():
    """Zone B's ring 10 m past A's exit: the cap's straight 5 s forecast
    (20 m) would reach it, the escape as flown - out of A's ring, then a stop
    3.1 m on - does not. cap_all leaves at 4 m/s; "cap" keeps the old 2.0
    m/s floor."""
    d = _escape("cap_all", 10.0)
    assert abs(d.emitted.vy - 4.0) < 1e-6 and not d.emitted_violations, d.emitted
    assert "escape_at_cap" in d.trace and "cap_refused" not in d.trace, d.trace.keys()
    old = _escape("cap", 10.0)
    assert abs(old.emitted.vy - 2.0) < 1e-6 and "escape_at_cap" not in old.trace, old.emitted
    alone = _escape("cap_all")
    assert abs(alone.emitted.vy - 4.0) < 1e-6, alone.emitted


def test_a_zone_escape_whose_stop_breaks_another_rule_is_retried_slower():
    """B's ring 1.5 m past A's exit, inside the cap's stop: the cap is
    refused and the fastest speed whose stop is clean flies (the windows'
    retries), still faster than the old floor, and it re-checks clean."""
    d = _escape("cap_all", 1.5)
    assert "cap_refused" in d.trace, d.trace.keys()
    assert 2.0 < d.emitted.vy < 4.0 - 1e-6 and not d.emitted_violations, d.emitted


def test_a_moving_zone_keeps_the_floor():
    """No shaped escape for a moving ring: a dynamic zone with motion is not
    escaped at the cap under cap_all; its floor (2.0 m/s plus the zone's own
    outward speed) stays."""
    zone = _box("a", 0, 0, 20, 20)
    zone.update({"type": "dynamic_nfz", "motion": {"vx_mps": 0.5}})
    pol = Policy.model_validate({"policy_id": "moving", "constraints": [zone, CAPS]})
    sh = Shield(pol)
    d = sh.filter(INSIDE, CRAWL_EAST, t=0.0)
    assert "escape_at_cap" not in d.trace, d.trace.get("escape_at_cap")
    assert abs(d.emitted.vy - 2.0) < 1e-6, d.emitted


# --------------------------------------------------------------------------- #
# the KPI invariant
# --------------------------------------------------------------------------- #

def _row(t, before, after, edge, vios=()):
    return {"t": t, "tick": int(round(t * 10)), "x": 0.0, "y": 0.0, "up": 15.0,
            "raw": {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
            "emitted": {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
            "violations": list(vios), "emitted_violations": [], "repairs": [],
            "braked": False, "unsafe": False, "owner": "shield",
            "fsm_state_before": before, "fsm_state_after": after, "fsm_edge": edge}


def test_the_false_trigger_invariant_leaves_x8_out_and_still_catches_a_logging_fault():
    """X8 leaves Normal on a tick whose request may be clean - its input is
    the position, not the request - so it is not counted as a false
    trigger; any other edge out of Normal on a violation-free row still is
    (a row that does not show what the FSM saw)."""
    x8 = [_row(0.0, "Normal", "Normal", None), _row(0.1, "Normal", "RTL", "X8")]
    assert K.compute(x8, {})["false_trigger_ticks"] == 0
    fault = [_row(0.0, "Normal", "Normal", None), _row(0.1, "Normal", "Brake", "G1")]
    assert K.compute(fault, {})["false_trigger_ticks"] == 1
    run = _fly(_cell("gps-dropout", lane_y=23.6, window=[2.0, 9.0]))
    assert _x8_t(run) is not None
    assert K.compute(run.rows, {})["false_trigger_ticks"] == 0


# --------------------------------------------------------------------------- #
# X8 on the autopilot's own estimator (2026-11-12)
# --------------------------------------------------------------------------- #

RAIL_POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _rail_denial(estimator, stream_vx=2.0, ticks=90, lost_at=10, fix=True):
    """A rail Shield (no FSM of its own, as the ROS 2 node's) handed the
    aircraft 2 m south of nfz-square's face (x 5, y 15), the GPS fix lost at
    tick `lost_at`, and `estimator(k)` as the autopilot's verdict on tick k
    (None: not reported). The rail says it streams `stream_vx` north every
    tick (note_flown) while the pose it hands on stays put: the hil rig's
    lane 23.6 cells in miniature, where the Shield's command dead reckoning
    ran 3.0-3.5 m from the autopilot's estimate. Returns {k: position_lost}
    for the ticks with a verdict."""
    from guardrail import load_policy
    from guardrail.replay import rail_shield
    sh = rail_shield(load_policy(RAIL_POLICY), lookahead_s=5.0, dt=0.1)
    out = {}
    for k in range(ticks):
        kw = {"fix_ok": (k < lost_at) if fix else None}
        est = estimator(k)
        if est is not None:
            kw["estimator_ok"] = est
        d = sh.filter(State(x=5.0, y=15.0, up=15.0, yaw_deg=0.0, **kw), Action4D(), t=k * 0.1)
        sh.note_flown(Action4D(vx=stream_vx))
        if d.position_lost:
            out[k] = d.position_lost
    return out


def test_the_autopilots_verdict_certifies_its_estimate_through_a_gps_denial():
    """While the autopilot vouches for the estimate it hands the Shield
    (State.estimator_ok True: ArduPilot's EKF_STATUS_REPORT with an absolute
    horizontal position), the position is certified, GPS or not: no dead
    reckoning of the Shield's own is held against it, so the same flight
    that takes X8 on the old law (no verdict reported) takes none."""
    old = _rail_denial(lambda k: None)
    assert old and min(old) < 40, sorted(old)[:3]
    assert all(rid == "nfz-square" and "dead reckoning" in why for why, rid in old.values())
    assert _rail_denial(lambda k: True) == {}
    # Nor does the verdict change a tick whose fix is valid.
    assert _rail_denial(lambda k: True, lost_at=10 ** 6) == {}
    # A dump without the field is the old dump; with it, it is recorded.
    assert "estimator_ok" not in State(x=1.0, y=2.0, up=3.0).model_dump()
    assert State(x=1.0, y=2.0, up=3.0, estimator_ok=True).model_dump()["estimator_ok"] is True


def test_the_autopilot_declaring_its_position_lost_is_x8_at_once():
    """estimator_ok False (the autopilot reports no valid absolute horizontal
    position for as long as its own EKF failsafe waits): the position
    certifies no margin, so X8 fires on that tick, naming the nearest hard
    P0 zone - with or without a GPS fix, however far the zone."""
    lost = _rail_denial(lambda k: k < 20)
    assert min(lost) == 20, sorted(lost)[:3]
    why, rid = lost[20]
    assert rid == "nfz-square" and "autopilot reports no valid absolute" in why, why
    assert "certifies no margin" in why and "2.0 m from it" in why, why
    with_fix = _rail_denial(lambda k: k < 20, lost_at=10 ** 6, stream_vx=0.0)
    assert min(with_fix) == 20, sorted(with_fix)[:3]
    # Far from every zone (the old law would wait for the error to reach one).
    from guardrail import load_policy
    from guardrail.replay import rail_shield
    sh = rail_shield(load_policy(RAIL_POLICY), lookahead_s=5.0, dt=0.1)
    d = sh.filter(State(x=-40.0, y=-40.0, up=15.0, fix_ok=False, estimator_ok=False),
                  Action4D(), t=0.0)
    assert d.position_lost and "autopilot reports" in d.position_lost[0], d.position_lost


def test_a_verdict_that_stops_falls_back_on_the_dead_reckoning_from_the_last_certified_pose():
    """A report that goes stale (estimator_ok None again) is not a verdict:
    the old law takes over, anchored on the last position the autopilot
    vouched for, not on the loss of the fix - X8 comes as many ticks after
    the verdict stops as after the fix was lost with none reported."""
    base = _rail_denial(lambda k: None)
    stale = _rail_denial(lambda k: True if k < 30 else None)
    assert base and stale, (base, stale)
    assert min(stale) - 30 == min(base) - 10, (min(stale), min(base))
    assert "no valid fix for" in stale[min(stale)][0]


def test_on_the_rail_vehicle_the_autopilots_estimate_is_flown_as_the_rig_flew_it():
    """The headless rail vehicle (RAIL_VEHICLE: the dead-reckoned denial)
    hands the Shield the modelled autopilot's verdict with its estimate:
    True until RAIL_EKF_POS_VALID_S after the loss (EKF3's 6.8 s, then the
    autopilot's 1 s failsafe rule), so no 7 s window reaches False. On it
    gps-dropout lane 23.6 at 4-11 s and 6-13 s fly as the rig's lane-23.6
    flights without X8 did (G1, G7, no fail-safe), where the old law -
    the verdict withheld - takes X8 at 10.3 s and 9.9 s (the rig's
    nt1111_568-573: 10.2-10.4 s and 9.3-9.5 s). Lane 15 keeps its G5. A
    frozen denial reports no verdict."""
    seen = []

    class Spy(Shield):
        def filter(self, state, *a, **kw):
            if getattr(state, "fix_ok", None) is False:
                seen.append(getattr(state, "estimator_ok", None))
            return super().filter(state, *a, **kw)

    class Withheld(Shield):
        def filter(self, state, *a, **kw):
            return super().filter(state.model_copy(update={"estimator_ok": None}), *a, **kw)

    kw = dict(response=SW.RAIL_VEHICLE["response"],
              autopilot_hold=SW.RAIL_VEHICLE["autopilot_hold"],
              gps_dropout=SW.RAIL_VEHICLE["gps_dropout"])
    assert 7.0 < SW.RAIL_EKF_POS_VALID_S < 8.0
    orig = SW.Shield
    try:
        for window, old_x8 in (([4.0, 11.0], 10.3), ([6.0, 13.0], 9.9)):
            spec = _cell("gps-dropout", lane_y=23.6, window=window)
            SW.Shield = Spy
            seen.clear()
            run = SW.run_episode(spec, DEFAULTS, 1001, **kw)
            assert len(seen) == 70 and set(seen) == {True}, (window, len(seen), set(seen))
            assert _edges(run) == [("G1", 5.4), ("G7", 8.6)], (window, _edges(run))
            assert not run.extra["fsm"]["failsafe_triggered"]
            assert run.extra["min_true_dist_to_fence_m"] > 2.5, run.extra["min_true_dist_to_fence_m"]
            SW.Shield = Withheld
            assert _x8_t(SW.run_episode(spec, DEFAULTS, 1001, **kw)) == old_x8, window
        SW.Shield = orig
        lane15 = SW.run_episode(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]),
                                DEFAULTS, 1001, **kw)
        assert [e for e, _ in _edges(lane15)][:3] == ["G1", "G3", "G5"], _edges(lane15)
        SW.Shield = Spy
        seen.clear()
        SW.run_episode(_cell("gps-dropout", lane_y=23.6, window=[4.0, 11.0]), DEFAULTS, 1001)
        assert seen and set(seen) == {None}, set(seen)
    finally:
        SW.Shield = orig


# --------------------------------------------------------------------------- #
# bundles
# --------------------------------------------------------------------------- #

def test_a_bundle_names_the_report_and_an_older_one_re_flies_blind():
    """/5 records the stress's fix report and the drift allowance and
    re-flies bit for bit; the same bundle read as /4 was flown before the
    report existed and re-flies blind (the old missed trigger); a /5 bundle
    flown with another allowance, or an older one naming one, is refused."""
    spec = _cell("gps-dropout", lane_y=23.6, window=[2.0, 9.0])
    run = SW.run_episode(spec, DEFAULTS, 0)
    ex = json.loads(json.dumps(SW.episode_extras(spec, DEFAULTS, 0, run)))
    assert ex["extras_schema"] == SW.EXTRAS_SCHEMA == "sweep-episode-extras/5"
    assert ex["scenario_spec"]["stress"]["fix_quality_reported"] is True
    assert ex["arm"]["dr_drift_mps"] == DR_DRIFT_MPS and ex["arm"]["recovery"] == "cap_all"
    assert _digest(SW.rerun_from_extras(ex).rows) == _digest(run.rows)
    old = copy.deepcopy(ex)
    old["extras_schema"] = "sweep-episode-extras/4"
    old["arm"].pop("dr_drift_mps")
    again = SW.rerun_from_extras(old)
    assert _edges(again) == [("G1", 9.0), ("G7", 11.4)], _edges(again)
    for bad in ({"extras_schema": SW.EXTRAS_SCHEMA, "drift": 0.25},
                {"extras_schema": "sweep-episode-extras/4", "drift": DR_DRIFT_MPS}):
        odd = copy.deepcopy(ex)
        odd["extras_schema"] = bad["extras_schema"]
        odd["arm"]["dr_drift_mps"] = bad["drift"]
        try:
            SW.rerun_from_extras(odd)
        except ValueError as e:
            assert "dr_drift_mps" in str(e), e
        else:
            raise AssertionError(f"{bad} was re-flown")


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
