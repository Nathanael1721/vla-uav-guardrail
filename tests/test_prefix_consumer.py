"""The prefix consumer (guardrail/prefix_consumer.py) and the Prefix A/B it
makes measurable, end to end: the adapter, the VLA stub's --prefix arm, the
stress harness's arms and refusals, a hil campaign on the fake rig, and the
KPI report's A/B section (audit card WP4-15, 2026-11-16).

    python tests/test_prefix_consumer.py      (no pytest needed)

NOTHING HERE FLIES. The headless episodes are the kinematic harness's; the
"hil" campaign runs on tests/test_sweep.py's fake rig (`_FakeRig`), here with
a mission that reads `--prefix` through the VLA stub's own argument parser
and pre-flight (sitl/ros2_vla_stub_node.py, no ROS). They show what the
harness sends, refuses, checks, scores and reports - nothing about ArduPilot,
MAVROS or the Orin.

What these guard:

  * the adapter clamps to the CSP's ranges and band (the yaw cap in deg/s
    against rad/s), and, with the CSP's rule ids resolved to the policy's
    compiled geometry (the Shield's margin rings included), turns to
    the nearest admissible heading of a fixed fan, stops when no heading is
    admissible, hands on what it cannot repair, and without a CSP returns
    the law's action untouched;
  * one CSP for the whole flight: a policy whose action sets change in
    flight is refused, and so is a scenario with mid-flight events;
  * NOTHING MOVES WITHOUT THE KEY: a run with no prefix consumer builds the
    episodes, arm records, rail arguments, scenario records and identity it
    always did (the KPI-grade campaign ntFIN2 must stay reproducible);
  * the harness and the VLA stub compile the same CSP from the same
    arguments, and a rail mission that did not fly its arm is an error,
    never a scored episode (the null: the rig that ignores --prefix);
  * "the arms must be one flight" holds for every pilot that reads nothing,
    and only a consuming pilot's pair may differ;
  * the KPI report measures the A/B over such pairs - mission success, P0
    escapes, repairs per episode, repair magnitude, time to safe, the A/A
    pairs as the noise floor - and leaves the per-paraphrase section alone.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments", ROOT / "tools", ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import sweep_scenarios as S                                       # noqa: E402
import kpi_report as KR                                           # noqa: E402
import ros2_vla_stub_node as V                                    # noqa: E402
from guardrail import load_policy                                 # noqa: E402
from guardrail import prefix_consumer as PC                       # noqa: E402
from guardrail import scenario_spec as SP                         # noqa: E402
from guardrail import vla_stub as VS                              # noqa: E402
from guardrail.compiler import (ConstraintCompiler, action_violations,  # noqa: E402
                                compiled_ir)
from guardrail.csp import CSP, csp_content_hash                   # noqa: E402
from guardrail.models import Action4D, State                      # noqa: E402
from guardrail.scenario_spec import Episode, Profile, load_profile  # noqa: E402


def _load(name: str, file: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tests" / file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


TS = _load("test_sweep_for_prefix", "test_sweep.py")
LIBRARY, DEFAULTS, FAMILIES, BY_ID = TS.LIBRARY, TS.DEFAULTS, TS.FAMILIES, TS.BY_ID
SKIP = "SKIP"
PROFILE = ROOT / "experiments" / "profiles" / "prefix_ab.yaml"
POL = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")
CORRIDOR = load_policy(ROOT / "policies" / "corridor_survey.yaml")
CELLS = ("nfz-head-on", "corridor-drift-out", "altitude-ceiling-hold",
         "altitude-floor-recovery", "speed-cap")


class _Law:
    """A pilot law that asks for the same action whatever the state."""

    def __init__(self, action: Action4D):
        self.action = action

    def act(self, state: State) -> Action4D:
        return self.action


def _csp(pol=POL, start=(-20.0, 15.0, 15.0), target=(40.0, 15.0, 15.0), speed=4.0, **kw):
    return PC.static_csp(pol, PC.consumer_mission(start, target, speed), **kw)


def _episodes(only=None, seed=1001):
    eps, info = S.expand(LIBRARY, load_profile(PROFILE), seed, only)
    return eps, info


def _arms(cell: str, seed: int = 1001) -> dict[str, Episode]:
    """The prefix-on, prefix-off and (where the profile has one) A/A episode
    of one cell of the profile, by arm tag."""
    return {e.arm_tag: e for e in _episodes([cell], seed)[0]}


def _zone_box() -> tuple[float, float, float, float]:
    """nfz-square's margin ring in local metres (min x, min y, max x, max y)."""
    return compiled_ir(POL).fences[0].buffered.bounds


# --------------------------------------------------------------------------- #
# the adapter
# --------------------------------------------------------------------------- #

def test_the_off_arm_hands_the_laws_action_on_untouched():
    """No CSP: the very object the law returned, whatever it asks for - the
    prefix-off arm is the pilot as it always flew."""
    wild = Action4D(vx=20.0, vy=3.0, vz_up=5.0, yaw_rate=3.0)
    c = PC.PrefixConsumer(_Law(wild), wraps="x")
    assert c.act(State(x=0.0, y=0.0, up=15.0)) is wild
    assert c.arm == "off" and c.stats["ticks"] == 1 and c.stats["clamped"] == 0
    rec = c.record()
    assert rec["arm"] == "off" and rec["id"] == PC.MODEL_ID and rec["adapter"] == PC.ADAPTER
    assert rec["action_set_sha256"] is None and rec["csp_content_hash"] is None
    assert rec["policy_hash"] is None and rec["wraps"] == "x"
    # a callable is a law too
    assert PC.PrefixConsumer(lambda s: wild).act(State(x=0.0, y=0.0, up=15.0)) is wild


def test_the_on_arm_clamps_to_the_csps_ranges_and_band():
    """Speed to the 4 m/s cap (as a disc: a diagonal is scaled, not boxed),
    climb to the range and to the band one lookahead ahead, yaw to 45 deg/s
    expressed in rad/s. The clamped action breaks nothing of the CSP's sets;
    the raw one breaks each of them (the null)."""
    csp = _csp()
    far = State(x=-40.0, y=-40.0, up=15.0)
    wild = Action4D(vx=20.0, vy=0.0, vz_up=5.0, yaw_rate=3.0)
    raw = action_violations(wild, csp, far, POL, dt=csp.lookahead_s)
    assert {"vx", "speed", "vz", "yaw_rate", "altitude"} <= set(raw), raw
    c = PC.PrefixConsumer(_Law(wild), csp, POL)
    out = c.act(far)
    assert action_violations(out, csp, far, POL, dt=csp.lookahead_s) == [], out
    assert (out.vx, out.vy) == (4.0, 0.0)
    # 15 m + vz * 5 s <= 20 m: 1 m/s, inside the 2 m/s climb range.
    assert abs(out.vz_up - 1.0) < 1e-9, out
    # The yaw cap is 45 deg/s; Action4D carries rad/s. A consumer that
    # compared the two raw would leave 3 rad/s (172 deg/s) alone.
    assert abs(out.yaw_rate - math.radians(45.0)) < 1e-9, out
    assert c.stats["clamped"] == 1
    diag = PC.PrefixConsumer(_Law(Action4D(vx=10.0, vy=10.0)), csp, POL).act(far)
    assert abs(math.hypot(diag.vx, diag.vy) - 4.0) < 1e-9 and abs(diag.vx - diag.vy) < 1e-9
    # From outside the band: toward it as fast as the climb range allows,
    # whatever the law asks (2 m/s up from below the floor, 2 m/s down from
    # above the ceiling), but never past its far edge within the lookahead
    # (from 9.5 m, 2 m/s for 5 s ends at 19.5 m: allowed; a 3 m band would
    # hold it lower - the corridor's 10-20 m band is the same here).
    level = _Law(Action4D(vx=3.0))
    assert PC.PrefixConsumer(level, csp, POL).act(State(x=-40.0, y=-40.0, up=3.0)).vz_up == 2.0
    assert PC.PrefixConsumer(level, csp, POL).act(State(x=-40.0, y=-40.0, up=24.0)).vz_up == -2.0
    assert PC.PrefixConsumer(level, csp, POL).act(State(x=-40.0, y=-40.0, up=9.5)).vz_up == 2.0
    # Inside the band the law's own climb is kept while one lookahead of it
    # stays inside: level flight is untouched, a descent toward the floor held.
    assert PC.PrefixConsumer(level, csp, POL).act(State(x=-40.0, y=-40.0, up=10.5)).vz_up == 0.0
    sink = PC.PrefixConsumer(_Law(Action4D(vx=3.0, vz_up=-2.0)), csp, POL)
    assert abs(sink.act(State(x=-40.0, y=-40.0, up=12.0)).vz_up - (10.0 - 12.0) / 5.0) < 1e-9
    # A legal action is the law's own object, uncounted.
    fine = Action4D(vx=3.0)
    c2 = PC.PrefixConsumer(_Law(fine), csp, POL)
    assert c2.act(far) is fine and c2.stats["clamped"] == 0


def test_the_fan_turns_to_the_nearest_admissible_heading_and_keeps_its_side():
    """Aimed straight at the zone from 10 m: the candidate would be inside
    the margin ring within the lookahead, so the nearest heading of the fan
    that would not is flown at the same speed. Deterministic, and the side
    taken is tried first on the next tick."""
    csp = _csp()
    x0, y0, x1, y1 = _zone_box()
    st = State(x=x0 - 10.0, y=(y0 + y1) / 2.0, up=15.0)
    ahead = Action4D(vx=4.0)
    c = PC.PrefixConsumer(_Law(ahead), csp, POL)
    assert c.inadmissible(ahead, st) == ["polygon:nfz-square"]
    out = c.act(st)
    assert c.inadmissible(out, st) == [] and abs(math.hypot(out.vx, out.vy) - 4.0) < 1e-9
    turn = math.degrees(math.atan2(out.vy, out.vx))
    assert abs(turn) in PC.FAN_DEG or min(abs(abs(turn) - d) for d in PC.FAN_DEG) < 1e-6, turn
    # the nearest: no smaller turn of the fan, to either side, is admissible
    smaller = [d for d in PC.FAN_DEG if d < abs(turn) - 1e-6]
    assert all(c.inadmissible(PC._turned(ahead, s * d), st) for d in smaller for s in (1, -1))
    assert c.stats["turned"] == 1 and c.stats["max_turn_deg"] == round(abs(turn))
    # deterministic: a second consumer gives the same action
    assert PC.PrefixConsumer(_Law(ahead), csp, POL).act(st) == out
    # the side last taken is tried first (dead centre, both sides are equal)
    side = c._side
    assert c.act(st) == out and c._side == side
    c._side = -side
    other = c.act(st)
    assert abs(other.vy + out.vy) < 1e-9 and abs(other.vx - out.vx) < 1e-9, (out, other)


def test_inside_a_zone_the_candidate_is_handed_on_and_a_dead_end_stops():
    """Already inside the ring no heading and no stop is admissible: the
    (clamped) candidate goes on unchanged - recovery is the Shield's. In a
    corridor too short to fly five seconds in any direction, stopping is
    admissible and is what is flown."""
    csp = _csp()
    x0, y0, x1, y1 = _zone_box()
    inside = State(x=(x0 + x1) / 2.0, y=(y0 + y1) / 2.0, up=15.0)
    c = PC.PrefixConsumer(_Law(Action4D(vx=9.0)), csp, POL)
    out = c.act(inside)
    assert (out.vx, out.vy) == (4.0, 0.0) and c.stats["no_admissible"] == 1
    assert c.stats["turned"] == 0 and c.stats["stopped"] == 0
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "tube.yaml"
        p.write_text(
            "policy_id: short-tube\nversion: 0.0.1\nconstraints:\n"
            "  - id: tube\n    type: corridor\n    constraint_type: hard\n"
            "    scope: mission\n    priority: P0\n    violation_action: project_fix\n"
            "    centerline: [{x: 0.0, y: 0.0}, {x: 6.0, y: 0.0}]\n    width_m: 4.0\n"
            "    altitude_floor_m: 10.0\n    altitude_ceiling_m: 20.0\n", encoding="utf-8")
        tube = load_policy(p)
    csp2 = _csp(tube, start=(3.0, 0.0, 15.0), target=(5.0, 0.0, 15.0))
    c2 = PC.PrefixConsumer(_Law(Action4D(vx=4.0)), csp2, tube)
    got = c2.act(State(x=3.0, y=0.0, up=15.0))
    assert (got.vx, got.vy) == (0.0, 0.0) and c2.stats["stopped"] == 1, (got, c2.stats)


def _corridor_cell():
    """corridor-drift-out's prefix-on arm (seed 1001): its spec, policy, CSP
    and arm record, as the harness builds them."""
    ep = _arms("corridor-drift-out")["prefix-on"]
    spec = ep.cell.spec
    pol = spec.policy.load(ROOT)
    return spec, pol, S.prefix_csp(spec, pol, DEFAULTS, 1001), S.build_arm(ep, defaults=DEFAULTS)


def test_the_turn_does_not_cut_a_dog_legs_inner_corner():
    """The hil prefix A/B pab1 (2026-11-16), pab1_008 tick 61: from
    (26.302, 12.483), asked for (0, 4), the 0.5 s point test admitted the
    60-degree heading (3.4641, 2.0) although its path cuts the inner corner of
    the dog-leg (notch vertex (40, 20)) for 0.21 s, about 0.25 m out - between
    the 3.5 s and 4.0 s samples. The Shield's 0.1 s forecast found it at
    t+3.8 s and CorridorReturn sized it at 2.48 m along the nearest leg, over
    theta 2.0: Loiter, and RTL on the loiter timeout. The path test refuses
    that heading, and what the consumer flies instead the Shield leaves alone."""
    from guardrail.replay import rail_shield
    spec, pol, csp, _ = _corridor_cell()
    state = State(x=26.302, y=12.483, up=14.96)
    cut = Action4D(vx=2.0 * math.sqrt(3.0), vy=2.0)
    # the null: no 0.5 s sample of the cut is outside, the 3.8 s point is
    look = float(csp.lookahead_s)
    n = int(math.ceil(look / PC.HORIZON_STEP_S - 1e-9))
    assert not [v for k in range(n) for v in action_violations(
        cut, csp, state, pol, dt=look * (k + 1) / n) if v.startswith("corridor:")]
    assert [v for v in action_violations(cut, csp, state, pol, dt=3.8)
            if v.startswith("corridor:")], "the cut does not leave the corridor at t+3.8 s"
    c = PC.PrefixConsumer(_Law(Action4D(vx=0.0, vy=4.0)), csp, pol)
    assert c.inadmissible(cut, state), "the path test admits the corner cut"
    out = c.act(state)
    assert (round(out.vx, 4), round(out.vy, 4)) != (3.4641, 2.0), out
    assert not c.inadmissible(out, state) and c.stats["turned"] == 1, (out, c.stats)
    sh = rail_shield(pol, lookahead_s=look, dt=0.1)
    res = sh.filter(state, out)
    assert not res.violations and not res.repairs, (out, res.violations, res.repairs)
    # and the same Shield does repair the cut (the check is not vacuous)
    cut_res = rail_shield(pol, lookahead_s=look, dt=0.1).filter(state, cut)
    assert "CorridorReturn" in [r.operator for r in cut_res.repairs], cut_res.repairs


def test_the_shields_corner_sizing_is_pinned_as_found():
    """The Shield half of pab1's finding, pinned so a change to it is a
    deliberate edit of this test (docs: CorridorReturn's dog-leg
    conservatism, guardrail/fsm.py). From the same state the corner cut is
    repaired by CorridorReturn sized as a straight line along the nearest
    leg, d + outward x L - half = 12.483 + 2.0 x 5 - 20 = 2.483 m, while the
    path's true worst excursion is about 0.25 m: about ten times over, and
    over theta 2.0 - the safe direction (it over-escalates, never
    under-repairs)."""
    from guardrail.replay import rail_shield
    from guardrail.shield import corridor_offset
    spec, pol, csp, _ = _corridor_cell()
    state = State(x=26.302, y=12.483, up=14.96)
    cut = Action4D(vx=2.0 * math.sqrt(3.0), vy=2.0)
    res = rail_shield(pol, lookahead_s=float(csp.lookahead_s), dt=0.1).filter(state, cut)
    cr = [r for r in res.repairs if r.operator == "CorridorReturn"]
    assert cr and abs(cr[0].magnitude_m - 2.483) < 0.01, [(r.operator, r.magnitude_m)
                                                          for r in res.repairs]
    corr = next(c for c in pol.constraints if c.id in
                csp.forbidden_action_set.no_translate_out_of_corridors)
    worst = max(corridor_offset(corr, state.x + cut.vx * t / 1000.0,
                                state.y + cut.vy * t / 1000.0) for t in range(5001))
    assert 0.2 < worst - corr.half_width_m < 0.3, worst


def test_closed_loop_the_corridor_arm_holds_no_loiter():
    """corridor-drift-out's prefix-on arm, headless and closed loop: no
    repair and no FSM edge (before the path test: 2 repairs, X1 then G3),
    and P0 escapes 0."""
    spec, pol, csp, rec = _corridor_cell()
    on = S.run_episode(spec, DEFAULTS, 1001, arm=rec)
    k = S.score(spec, on.rows, on.extra, on.policy, on.priorities)
    assert k["p0_escapes"] == 0 and k["repair_count"] == 0, k
    edges = [r.get("fsm_edge") for r in on.rows if r.get("fsm_edge")]
    assert not edges, edges


def test_a_nonfinite_candidate_is_not_repaired_here():
    """The action contract's non-finite channel is the Shield's to refuse:
    the consumer hands it on and counts it."""
    nan = Action4D(vx=float("nan"))
    c = PC.PrefixConsumer(_Law(nan), _csp(), POL)
    assert c.act(State(x=-40.0, y=-40.0, up=15.0)) is nan
    assert c.stats["nonfinite_passed"] == 1


def test_a_csp_of_another_policy_is_refused_before_anything_flies():
    csp = _csp()
    for bad in (lambda: PC.PrefixConsumer(_Law(Action4D()), csp, CORRIDOR),
                lambda: PC.PrefixConsumer(_Law(Action4D()), csp),
                lambda: PC.consumer_record("on", None),
                lambda: PC.consumer_record("off", csp)):
        try:
            bad()
        except ValueError:
            continue
        raise AssertionError("accepted")


def test_one_csp_for_the_whole_flight_or_none():
    """static_csp compiles with the spatial filter off (a zone far from the
    mission line is still in the action sets: the default filter drops it,
    the null) and refuses a policy whose action sets change in flight: the
    school zone's weekday window closes at 17:30."""
    far = PC.consumer_mission((-5000.0, 0.0, 15.0), (-4990.0, 0.0, 15.0), 4.0)
    kept = PC.static_csp(POL, far)
    assert kept.forbidden_action_set.no_translate_into_polygons == ["nfz-square"]
    assert kept.selection.region is None
    dropped = ConstraintCompiler(POL).compile_csp(far)
    assert dropped.forbidden_action_set.no_translate_into_polygons == []
    m = PC.consumer_mission((30.0, 0.0, 15.0), None, 4.0)
    assert (m.target_x, m.target_y, m.cruise_alt_m) == (30.0, 0.0, 15.0), "no target: its start"
    saturday = PC.static_csp(CORRIDOR, m, now=datetime(2026, 9, 5, 9, 0), duration_s=65.0)
    assert saturday.forbidden_action_set.no_translate_into_polygons == []
    no_clock = PC.static_csp(CORRIDOR, m)
    assert no_clock.forbidden_action_set.no_translate_into_polygons == ["nfz-school"]
    friday = datetime(2026, 9, 4, 17, 29, 30)
    assert PC.static_csp(CORRIDOR, m, now=friday).forbidden_action_set \
        .no_translate_into_polygons == ["nfz-school"], "in force at take-off"
    try:
        PC.static_csp(CORRIDOR, m, now=friday, duration_s=120.0)
    except ValueError as e:
        assert "static" in str(e) and "17:3" in str(e), e
    else:
        raise AssertionError("a rule switching in flight was accepted")
    # The identity a rail flight is bound by covers what is consumed, and
    # moves when it moves.
    a, b = PC.action_set_sha256(saturday), PC.action_set_sha256(no_clock)
    assert a != b and a == PC.action_set_sha256(
        PC.static_csp(CORRIDOR, m, now=datetime(2026, 9, 5, 9, 0)))


def test_the_consumer_is_its_own_model_and_the_stubs_keep_theirs():
    """Its id is pinned by ITS file's source; the three stub pilots' ids and
    their module are untouched (their source is the vla_model_hash of every
    KPI-grade campaign flown so far). The profile's adapter names are the
    module's."""
    from guardrail.manifest import model_hash
    src = (ROOT / "guardrail" / "prefix_consumer.py").read_bytes()
    assert model_hash(PC.MODEL_ID) == f"{PC.MODEL_ID}@src:{hashlib.sha256(src).hexdigest()}"
    assert VS.MODEL_IDS == {"stub_vla": "guardrail.vla_stub.StubVLA",
                            "goto": "guardrail.vla_stub.GotoPilot",
                            "constant": "guardrail.vla_stub.ConstantPilot"}
    assert PC.MODEL_ID not in VS.MODEL_IDS.values()
    stub_src = (ROOT / "guardrail" / "vla_stub.py").read_text(encoding="utf-8")
    assert "prefix" not in stub_src.lower(), "guardrail/vla_stub.py must not change for this"
    assert SP.PREFIX_CONSUMERS == PC.ADAPTERS
    # The law it wraps is pinned by ITS source, in the consumer's record: a
    # consumer flight's vla_model_hash pins prefix_consumer.py only, so
    # without this the law that flies the prefix-off arm is pinned by name.
    vs_digest = hashlib.sha256((ROOT / "guardrail" / "vla_stub.py").read_bytes()).hexdigest()
    pins = {}
    for kind, mid in VS.MODEL_IDS.items():
        rec = PC.consumer_record("off", None, mid)
        assert rec["wraps_model_hash"] == model_hash(mid) == f"{mid}@src:{vs_digest}", rec
        pins[kind] = rec["wraps_model_hash"]
    assert len(set(pins.values())) == 3, "a goto and a constant cell share no pin"
    assert PC.PrefixConsumer(_Law(Action4D()), wraps=VS.MODEL_IDS["goto"]).record()[
        "wraps_model_hash"] == pins["goto"]
    assert PC.consumer_record("off", None)["wraps_model_hash"] is None
    # It is not a CSP-only consumer, and its docstring does not say it is:
    # the turn reads the policy's geometry with the Shield's margins.
    assert "nothing else" not in PC.__doc__ and "margin" in PC.__doc__
    assert "compiled_ir(policy)" in PC.__doc__


def test_the_csp_file_is_the_csp_that_is_consumed():
    csp = _csp()
    with tempfile.TemporaryDirectory() as td:
        p = PC.write_csp(Path(td) / "run", csp)
        assert p == Path(td) / "run" / PC.PREFIX_DIR / PC.CSP_FILE
        raw = p.read_bytes()
        assert b"\r" not in raw and raw.endswith(b"\n")
        back = CSP.model_validate_json(raw.decode("utf-8"))
    assert back == csp and csp_content_hash(back) == csp_content_hash(csp)


# --------------------------------------------------------------------------- #
# the VLA stub's --prefix arm (no ROS)
# --------------------------------------------------------------------------- #

def _stub_args(cell: str, arm: str, seed: int = 1001):
    """The arguments the harness sends for a prefix arm (the desktop rail's
    form: the YAML policy), read by the stub's own parser."""
    ep = _arms(cell, seed)[f"prefix-{arm}"]
    args = S.rail_node_args(ep.cell.spec, seed, DEFAULTS, backend="sitl", episode=ep)
    vs, _ = V.make_parser().parse_known_args(args)
    vs.policy = str(ROOT / vs.policy)
    return ep, args, vs


def test_without_prefix_the_stub_is_the_stub_it_was():
    """No --prefix: no consumer, no policy read by the pre-flight, the pilot
    and its record as pilot_from_args made them, and an identity record with
    the keys it always had."""
    vs, _ = V.make_parser().parse_known_args(["--pilot", "goto", "--start=-20,15,15",
                                              "--target=40,15,15", "--speed=4"])
    assert vs.prefix is None and V.prefix_preflight(vs) is None
    mission = ConstraintCompiler(POL).parse_command(vs.command)
    mission, target, start = V.mission_from_args(vs, mission)
    pilot, rec = V.pilot_from_args(vs, mission, target)
    assert V.prefix_pilot(vs, POL, mission, pilot, rec, start) == (pilot, rec, None, None)
    ident = V.identity_record(POL, {"kind": "yaml"}, mission, yaw_rate=0.0, seed=0, host={},
                              pilot=rec, start=start)
    assert set(ident) == {"model_id", "kind", "frame", "yaw_rate", "seed", "policy_id",
                          "policy_hash", "policy_source", "target", "speed_pref_mps",
                          "host", "pilot", "start"}, sorted(ident)
    assert ident["model_id"] == VS.MODEL_IDS["goto"]
    assert set(V.identity_record(POL, {"kind": "yaml"}, mission, yaw_rate=0.0, seed=0,
                                 host={})) == set(ident) - {"pilot", "start"}
    # The Shield node's --clock-start, --max-s and --tag reach this parser
    # too (both nodes get every argument). Without --prefix their values are
    # the Shield node's business alone: nothing here converts or checks them,
    # so no mission that flew before this parser knew them is refused now.
    base = ["--pilot", "goto", "--start=-20,15,15", "--target=40,15,15", "--speed=4"]
    odd = ["--clock-start", "teatime", "--max-s=soon", "--tag", "../x"]
    vs, rest = V.make_parser().parse_known_args(base + odd + ["--shield", "on"])
    assert rest == ["--shield", "on"], rest
    V.check_args(vs)
    assert V.prefix_preflight(vs) is None
    for arm in ("on", "off"):
        vs, _ = V.make_parser().parse_known_args(base + odd + ["--prefix", arm])
        try:
            V.check_args(vs)
        except ValueError as e:
            assert "--clock-start" in str(e), e
        else:
            raise AssertionError(f"--prefix {arm}: a --clock-start that is no time passed")
    # ...and the clock is read by the Shield node's own parser: one instant.
    from guardrail.replay import parse_clock_start
    for text in ("2026-09-05T09:00:00", "2026-09-05T09:00:00Z", "2026-09-05T09:00:00+08:00"):
        assert V.clock_start_from(text) == parse_clock_start(text), text
    assert V.clock_start_from(None) is None and V.flight_length_from(None) is None
    assert V.flight_length_from("65") == 65.0
    for bad in ("0", "-3", "nan", "inf", "soon"):
        try:
            V.flight_length_from(bad)
        except ValueError:
            continue
        raise AssertionError(f"--max-s {bad} read as a flight length")


def test_the_stub_compiles_the_csp_the_harness_expects():
    """For every cell of the profile: the arguments the harness sends, read
    by the stub's parser and pre-flight, give the consumer record the
    harness's arm record carries - the same action sets, the same policy,
    the same whole-CSP content - and the off arm none. One cell and seed fly
    one CSP headless and on a rail."""
    for cell in CELLS:
        ep, args, vs = _stub_args(cell, "on")
        assert args[-1] == "--prefix=on" and vs.prefix == "on", args
        V.check_args(vs)
        got = V.prefix_preflight(vs)
        want = S.build_arm(ep, defaults=DEFAULTS)["consumer"]
        assert got == want, (cell, got, want)
        assert got["arm"] == "on" and got["wraps"] == VS.MODEL_IDS[ep.cell.spec.mission.pilot.type]
        assert got["action_set_sha256"].startswith("sha256:") and got["generation"] == 0
        ep_off, args_off, vs_off = _stub_args(cell, "off")
        off = V.prefix_preflight(vs_off)
        assert args_off[-1] == "--prefix=off" and off["arm"] == "off"
        assert off == S.build_arm(ep_off, defaults=DEFAULTS)["consumer"]
        assert off["action_set_sha256"] is None and off["wraps"] == got["wraps"]
    # What the node announces: the consumer's own model id, the law it wraps,
    # and the prefix record under its own key.
    ep, args, vs = _stub_args("nfz-head-on", "on")
    mission = ConstraintCompiler(POL).parse_command(vs.command)
    mission, target, start = V.mission_from_args(vs, mission)
    pilot, rec = V.pilot_from_args(vs, mission, target)
    flown, rec2, prefix, csp = V.prefix_pilot(vs, POL, mission, pilot, rec, start)
    assert isinstance(flown, PC.PrefixConsumer) and flown.csp is csp and csp is not None
    ident = V.identity_record(POL, {"kind": "yaml"}, mission, yaw_rate=0.0, seed=1001,
                              host={}, pilot=rec2, start=start, prefix=prefix)
    assert ident["model_id"] == PC.MODEL_ID and ident["kind"] == "stub"
    assert ident["pilot"]["wraps"] == VS.MODEL_IDS["goto"] and ident["pilot"]["prefix"] == "on"
    assert ident["prefix"] == prefix and json.loads(json.dumps(ident)) == ident
    # ...and the body-frame action it publishes is the consumer's.
    st = State(x=-20.0, y=15.0, up=15.0, yaw_deg=0.0)
    assert V.body_action(flown, st, 0.0)[:2] != V.body_action(pilot, st, 0.0)[:2] \
        or flown.stats["ticks"] == 1


def test_the_csp_is_written_under_the_runs_tag():
    ep, args, vs = _stub_args("nfz-head-on", "on")
    csp = S.prefix_csp(ep.cell.spec, None, DEFAULTS, 1001)
    assert V.write_prefix_csp(vs, csp) is None, "no --tag: nothing is written"
    saved = V.ROOT
    with tempfile.TemporaryDirectory() as td:
        V.ROOT = Path(td)
        try:
            vs.tag = "t_prefix"
            rel = V.write_prefix_csp(vs, csp)
        finally:
            V.ROOT = saved
        assert rel == "demo/out/t_prefix/vla_prefix/csp_g0.json"
        back = CSP.model_validate_json((Path(td) / rel).read_text(encoding="utf-8"))
    assert PC.action_set_sha256(back) == S.build_arm(ep, defaults=DEFAULTS)["consumer"][
        "action_set_sha256"]


def test_a_prefix_arm_that_cannot_be_compiled_is_refused_before_ros():
    """A --clock-start that is no time, a policy that is not static over
    --max-s, a --tag that is a path: refused by main() before ROS is touched
    (the message says REFUSED, not that rclpy is missing). A good prefix arm
    gets past the pre-flight (this host has no rclpy: that is the message)."""
    base = ["--pilot", "constant", "--action=0,4,0,0", "--no-target", "--start=30,0,15",
            "--speed=4", "--policy", str(ROOT / "policies" / "corridor_survey.yaml"),
            "--prefix", "on"]
    for more, word in ((["--clock-start", "teatime"], "ISO 8601"),
                       (["--max-s", "soon"], "positive number"),
                       (["--clock-start", "2026-09-04T17:29:30", "--max-s", "120"], "static"),
                       (["--tag", "../x"], "plain directory name")):
        try:
            V.main(base + more)
        except SystemExit as e:
            assert "REFUSED" in str(e) and word in str(e), (more, e)
        else:
            raise AssertionError(f"{more} was accepted")
    good = base + ["--clock-start", "2026-09-05T09:00:00", "--max-s", "65"]
    vs, _ = V.make_parser().parse_known_args(good)
    V.check_args(vs)
    assert V.prefix_preflight(vs)["arm"] == "on"
    if not V.HAVE_ROS:
        # main() itself, where there is no ROS to start: past the pre-flight.
        try:
            V.main(good)
        except SystemExit as e:
            assert "rclpy" in str(e) and "REFUSED" not in str(e), e
        else:
            raise AssertionError("main() returned on a host without rclpy")


# --------------------------------------------------------------------------- #
# the profile, the episodes, and every run without the key
# --------------------------------------------------------------------------- #

def test_the_profile_flies_fifteen_pairs_and_three_aa_repeats():
    eps, info = _episodes(seed=None)
    assert info["cells"] == 5 and len(eps) == 33, (info["cells"], len(eps))
    assert {e.cell.scenario_id for e in eps} == set(CELLS)
    assert all(e.prefix_consumer == "action_set" and e.ab and e.paraphrase_arm is None
               for e in eps)
    on = [e for e in eps if e.prefix == "on" and not e.repeat]
    off = [e for e in eps if e.prefix == "off"]
    aa = [e for e in eps if e.repeat]
    assert (len(on), len(off), len(aa)) == (15, 15, 3)
    assert {(e.episode_id, e.seed) for e in on} == {(e.episode_id, e.seed) for e in off}
    assert sorted((e.episode_id, e.seed, e.arm_tag) for e in aa) == [
        ("altitude-floor-recovery", 1001, "prefix-on,aa1"),
        ("corridor-drift-out", 1001, "prefix-on,aa1"),
        ("nfz-head-on", 1001, "prefix-on,aa1")]
    assert len({(e.run_id, e.seed) for e in eps}) == 33, "one id per flight"
    assert "--prefix-on,aa1--seed1001" in aa[0].dir_name("x")
    assert info["arms"]["prefix_consumer"] == "action_set"
    assert info["arms"]["prefix_aa_episodes"] == 3 and info["arms"]["prefix_off_episodes"] == 15
    # every cell is static (no mid-flight event) and flyable by the consumer
    assert all(not e.cell.spec.events and S.prefix_consumer_refusal(e.cell.spec) is None
               for e in eps)
    prof = load_profile(PROFILE)
    assert prof.grant_kpi_bearing is False and prof.paraphrase_k == 0


def test_a_profile_that_misuses_the_keys_is_refused():
    raw = {"profile": "p", "cadence": "c", "grant_scope": "g", "grant_sim_speedup": "s",
           "seeds": [1, 2], "expected_cells": [1, 9], "families": ["nfz-head-on"],
           "paraphrase_k": 0, "prefix_ab": True, "prefix_consumer": "action_set"}
    Profile.model_validate(raw)
    for change, word in (({"paraphrase_k": None}, "paraphrase"),
                         ({"paraphrase_k": 2}, "paraphrase"),
                         ({"prefix_consumer": None, "prefix_aa": ["nfz-head-on"]}, "prefix_aa"),
                         ({"prefix_consumer": "words"}, "action_set"),
                         ({"prefix_aa": ["a", "a"]}, "repeats"),
                         ({"prefix_aa_seeds": [1]}, "prefix_aa"),
                         ({"prefix_aa": ["nfz-head-on"], "prefix_aa_seeds": [7]}, "seeds")):
        try:
            Profile.model_validate({**raw, **change})
        except ValueError as e:
            assert word in str(e), (change, e)
        else:
            raise AssertionError(f"{change} was accepted")
    # a name no cell of the run answers to, and a cell with no prefix-on arm
    for change, word in (({"prefix_aa": ["no-such-cell"]}, "no cell"),
                         ({"prefix_aa": ["nfz-head-on"], "prefix_ab": False,
                           "families": ["nfz-head-on"]}, None)):
        prof = Profile.model_validate({**raw, **change})
        try:
            eps, _ = S.expand(LIBRARY, prof)
        except ValueError as e:
            assert word and word in str(e), (change, e)
        else:
            # prefix_ab false: nfz-head-on flies its own `prefix` (on) once,
            # and the A/A repeat of it.
            assert word is None and [e.arm_tag for e in eps if e.seed == 1] == [
                "prefix-on", "prefix-on,aa1"], [e.arm_tag for e in eps]


def test_a_run_without_the_key_builds_what_it_always_built():
    """The library run and the smoke and nightly profiles: no episode names
    a consumer or a repeat, their arm tags, run ids and arm records are the
    ones they were (an arm record gains a key only on a consumer arm), and
    the scope record carries no consumer key."""
    for prof in (None, "smoke", "nightly"):
        p = None if prof is None else load_profile(
            ROOT / "experiments" / "profiles" / f"{prof}.yaml")
        eps, info = S.expand(LIBRARY, p)
        assert all(e.prefix_consumer is None and e.repeat == 0 for e in eps), prof
        assert "prefix_consumer" not in info["arms"] and "prefix_aa_episodes" not in info["arms"]
        assert not any("aa" in e.arm_tag.split(",") for e in eps)
    eps, _ = S.expand(LIBRARY, None, 1001, ["nfz-head-on", "speed-cap"])
    tags = sorted(e.arm_tag for e in eps)
    assert tags == ["", "canonical,prefix-off", "canonical,prefix-on", "pp0,prefix-off",
                    "pp0,prefix-on", "pp1,prefix-off", "pp1,prefix-on"], tags
    plain = next(e for e in eps if e.arm_tag == "")
    assert S.build_arm(plain) is None
    for e in eps:
        rec = S.build_arm(e)
        if rec is not None:
            assert set(rec) == {"arm", "prefix", "pilot_reads_text", "instruction_source",
                                "manifest_extras", "paraphrase", "prompt_sha256",
                                "prefix_sha256"}, sorted(rec)
            assert set(S.manifest_extras(rec)) >= {"prefix_arm", "pilot_reads_text"}
            assert "pilot_reads_prefix" not in S.manifest_extras(rec)
        assert S.scoring_spec(e.cell.spec, e) is e.cell.spec
        assert S.prefix_family_suffix(e) == ""
    cell = FAMILIES["nfz-head-on"].first
    assert Episode(cell=cell, seed=1).arm_tag == ""
    assert Episode(cell=cell, seed=1, prefix="off").arm_tag == "prefix-off"
    assert Episode(cell=cell, seed=1, ab=True).arm_tag == "prefix-on"


# --------------------------------------------------------------------------- #
# headless flights
# --------------------------------------------------------------------------- #

def test_the_two_arms_are_one_pilot_with_and_without_its_prefix():
    """nfz-head-on, the legible case. Prefix off: the flight is, row for
    row, the cell's plain flight (the consumer without a CSP changes
    nothing) - held at the zone's edge, the goal never reached. Prefix on:
    the pilot goes round, reaches the goal, and the Shield repairs far less.
    The A/A repeat is the prefix-on flight again, row for row. The arm
    record's CSP must be the cell's, or nothing flies."""
    arms = _arms("nfz-head-on")
    spec = arms["prefix-on"].cell.spec
    recs = {t: S.build_arm(e, defaults=DEFAULTS) for t, e in arms.items()}
    assert set(recs) == {"prefix-on", "prefix-off", "prefix-on,aa1"}
    for tag, rec in recs.items():
        assert rec["pilot_reads_prefix"] is True and rec["pilot_reads_text"] is False
        assert rec["arm"] == tag and rec["consumer"]["id"] == PC.MODEL_ID
        assert rec["repeat"] == (1 if tag.endswith("aa1") else 0)
        ex = S.manifest_extras(rec)
        assert ex["pilot_reads_prefix"] is True and ex["prefix_consumer"] == PC.MODEL_ID
        # the consumer's own source pin, beside the manifest's vla_model_hash
        # (which, headless, names the harness's pilot function)
        from guardrail.manifest import model_hash
        assert ex["prefix_consumer_model_hash"] == model_hash(PC.MODEL_ID)
        assert ex["prefix_consumer_model_hash"].startswith(PC.MODEL_ID + "@src:")
        assert ex["prefix_action_set_sha256"] == rec["consumer"]["action_set_sha256"]
        assert ex["prefix_repeat"] == rec["repeat"]
    plain = S.run_episode(spec, DEFAULTS, 1001)
    off = S.run_episode(spec, DEFAULTS, 1001, arm=recs["prefix-off"])
    on = S.run_episode(spec, DEFAULTS, 1001, arm=recs["prefix-on"])
    aa = S.run_episode(spec, DEFAULTS, 1001, arm=recs["prefix-on,aa1"])
    assert off.rows == plain.rows, "the prefix-off arm is not the cell's plain flight"
    assert aa.rows == on.rows, "an A/A pair differs on the deterministic backend"
    assert on.rows != off.rows
    assert "prefix_consumer" not in plain.extra
    assert off.extra["prefix_consumer"]["arm"] == "off"
    st = on.extra["prefix_consumer"]["stats"]
    assert st["turned"] > 0 and st["no_admissible"] == 0 and st["ticks"] == len(on.rows), st
    k_on = S.score(spec, on.rows, on.extra, on.policy, on.priorities)
    k_off = S.score(spec, off.rows, off.extra, off.policy, off.priorities)
    assert (on.extra["reached_goal"], off.extra["reached_goal"]) == (True, False)
    assert k_on["p0_escapes"] == 0 and k_off["p0_escapes"] == 0
    # Since the path test (2026-11-16) the prefix-on arm clips no ring
    # between samples: no repair at all (19 before, two Loiters).
    assert k_on["repair_count"] < k_off["repair_count"] / 4, (
        k_on["repair_count"], k_off["repair_count"])
    assert k_on["repair_count"] == 0 and k_on["mean_repair_magnitude_mps"] is None, k_on
    assert k_off["mean_repair_magnitude_mps"] > 0
    # A record whose CSP is not the cell's is refused, not flown.
    forged = json.loads(json.dumps(recs["prefix-on"]))
    forged["consumer"]["action_set_sha256"] = "sha256:" + "0" * 64
    for bad in (forged, {**recs["prefix-on"], "consumer": {"id": "someone.else"}}):
        try:
            S.run_episode(spec, DEFAULTS, 1001, arm=bad)
        except ValueError as e:
            assert "CSP" in str(e) or "prefix consumer" in str(e), e
        else:
            raise AssertionError("flown on a CSP that is not the arm record's")


def test_a_scenario_with_events_is_refused_and_an_injected_event_stops_the_episode():
    """The consumer flies generation 0's CSP: a scenario that declares
    mid-flight events is refused before take-off (the arm record, the rail
    refusal), and an event injected into a stepped consumer episode stops it
    rather than fly on under rules the CSP does not hold."""
    cell = FAMILIES["dyn-nfz-translate"].first
    ep = Episode(cell=cell, seed=1001, prefix="on", ab=True, prefix_consumer="action_set")
    why = S.prefix_consumer_refusal(cell.spec)
    assert why and "WP4-19" in why and "event" in why
    try:
        S.build_arm(ep, defaults=DEFAULTS)
    except ValueError as e:
        assert "WP4-19" in str(e)
    else:
        raise AssertionError("an arm record was built for a scenario with events")
    refusal = S.rail_refusal(cell.spec, "hil", episode=ep, defaults=DEFAULTS)
    assert refusal and "WP4-19" in refusal
    arms = _arms("nfz-head-on")
    spec = arms["prefix-on"].cell.spec
    r = S.EpisodeRunner(spec, DEFAULTS, 1001, S.build_arm(arms["prefix-on"], defaults=DEFAULTS))
    r.inject({"at_sim_t": 0.5, "type": "spawn_polygon_fence",
              "payload": {"id": "late", "vertices": [{"x": 200, "y": 200}, {"x": 210, "y": 200},
                                                     {"x": 210, "y": 210}, {"x": 200, "y": 210}]}})
    try:
        r.finish()
    except ValueError as e:
        assert "WP4-19" in str(e), e
    else:
        raise AssertionError("the episode flew on past an injected event")


def test_the_prefix_on_arm_is_scored_without_the_cells_labels():
    """altitude-ceiling-hold is labelled RTL_triggered / fail-safe expected:
    what its rule-ignorant pilot provokes by climbing into the ceiling for
    50 s. The consuming pilot does not ask, so the label does not apply:
    scored on the cell's own spec the prefix-on arm FAILS its labels (the
    null), on the scoring spec it passes its gates. The off arm, and every
    episode without a consumer, keeps the cell's spec."""
    arms = _arms("altitude-ceiling-hold", 1002)
    on, off = arms["prefix-on"], arms["prefix-off"]
    spec = on.cell.spec
    assert spec.expected_outcome == "RTL_triggered" and spec.expected_failsafe is True
    sspec = S.scoring_spec(spec, on)
    assert sspec is not spec and sspec.expected_outcome is None
    assert sspec.expected_failsafe is None and sspec.gates == spec.gates
    assert S.scoring_spec(spec, off) is spec and S.scoring_spec(spec, None) is spec
    run = S.run_episode(spec, DEFAULTS, 1002, arm=S.build_arm(on, defaults=DEFAULTS))
    own = S.score(spec, run.rows, run.extra, run.policy, run.priorities)
    assert S.episode_verdict(spec, own, {}, 1002)[0] == "fail"
    res = S.score(sspec, run.rows, run.extra, run.policy, run.priorities)
    status, bad, _, _ = S.episode_verdict(sspec, res, {}, 1002)
    assert status == "pass" and not bad, bad
    assert res["failsafe_matches_label"] is None and res["outcome_matches_label"] is None
    assert res["repair_count"] == 0 and res["mission_outcome"] == "success"


# --------------------------------------------------------------------------- #
# the rails
# --------------------------------------------------------------------------- #

def test_the_rail_refusal_is_lifted_only_for_a_consumers_prefix_arms():
    """A prefix-off arm of a pilot that reads nothing is refused as before;
    of a consuming pilot it is flown. A paraphrase arm stays refused either
    way (the consumer reads no words)."""
    cell = FAMILIES["nfz-head-on"].first
    spec = cell.spec

    def why(**kw):
        return S.rail_refusal(spec, "hil", episode=Episode(cell=cell, seed=1001, **kw),
                              defaults=DEFAULTS)
    assert "reads no text" in why(prefix="off", ab=True)
    assert "reads no text" in why(paraphrase_arm=0, ab=True)
    assert why(prefix="on", ab=True) is None
    assert why(prefix="off", ab=True, prefix_consumer="action_set") is None
    assert why(prefix="on", ab=True, prefix_consumer="action_set") is None
    assert why(prefix="on", ab=True, prefix_consumer="action_set", repeat=1) is None
    assert "reads no text" in why(paraphrase_arm=0, ab=True, prefix_consumer="action_set")
    assert "reads no text" in why(paraphrase_arm=1, prefix="off", ab=True,
                                  prefix_consumer="action_set")


def test_the_rail_arguments_gain_one_flag_and_only_for_a_consumer():
    """--prefix=on|off, last, for a consumer episode; for every other episode
    (and none) the list it always was. Both nodes' parsers read it as the
    harness means it: the stub takes it, the Shield node leaves it; and a
    stub that did not declare it would be refused, not flown."""
    import ros2_shield_node as N
    arms = _arms("corridor-drift-out")
    spec = arms["prefix-on"].cell.spec
    base = S.rail_node_args(spec, 1001, DEFAULTS, backend="sitl")
    plain = Episode(cell=arms["prefix-on"].cell, seed=1001, prefix="on", ab=True)
    assert S.rail_node_args(spec, 1001, DEFAULTS, backend="sitl", episode=plain) == base
    assert "--prefix" not in TS._flag_names(base)
    for tag, arm in (("prefix-on", "on"), ("prefix-off", "off"), ("prefix-on,aa1", "on")):
        args = S.rail_node_args(spec, 1001, DEFAULTS, backend="sitl", episode=arms[tag])
        assert args == base + [f"--prefix={arm}"], args
        ns, rest_n = N.make_parser().parse_known_args(args)
        vs, rest_v = V.make_parser().parse_known_args(args)
        assert vs.prefix == arm and "--prefix" in TS._flag_names(rest_n)
        assert TS._flag_names(rest_n) <= (set(S.RAIL_STUB_FLAGS + S.RAIL_STUB_PREFIX_FLAGS)
                                          - set(S.RAIL_SHIELD_FLAGS)), rest_n
        # the clock and the flight length the CSP is compiled and checked over
        assert vs.clock_start == ns.clock_start == "2026-09-05T09:00:00"
        assert V.flight_length_from(vs.max_s) == float(ns.max_s) == S.rail_max_s(30.0)
        assert V.clock_start_from(vs.clock_start) == datetime(2026, 9, 5, 9, 0, 0)
        V.check_args(vs)
    saved = S.sitl_node_flags
    for missing in S.RAIL_STUB_PREFIX_FLAGS:
        S.sitl_node_flags = lambda script=None, m=missing: saved(script) - (
            {m} if script is not None and Path(script).name == Path(S.SITL_STUB).name
            else set())
        try:
            S.rail_node_args(spec, 1001, DEFAULTS, backend="sitl", episode=arms["prefix-on"])
        except S.UnsupportedScenario as e:
            assert missing in str(e), e
        else:
            raise AssertionError(f"a stub without {missing} was sent a prefix arm")
        finally:
            S.sitl_node_flags = saved
        # ...and a mission without a prefix arm is not held to those flags
        S.sitl_node_flags = lambda script=None, m=missing: saved(script) - (
            {m} if script is not None and Path(script).name == Path(S.SITL_STUB).name
            and m == "--prefix" else set())
        try:
            assert S.rail_node_args(spec, 1001, DEFAULTS, backend="sitl") == base
        finally:
            S.sitl_node_flags = saved


def test_a_lengthened_campaign_checks_the_csp_over_the_flight_the_stub_is_sent():
    """--hil-max-s-add sends the stub a longer --max-s, which its CSP must be
    static over. corridor-drift-out at 17:29:30 on a Friday: nfz-school's
    window (to 17:30) holds over the rail's own 65 s but not over 35 s more.
    The stub refuses the lengthened arguments (the null: what the rig would
    have done after SITL booted); rail_node_args - the dry run's and the
    mission's - now refuses both arms first. At the cell's own clock the
    lengthened mission is sent as before."""
    arms = _arms("corridor-drift-out")
    on, off = arms["prefix-on"], arms["prefix-off"]
    spec = on.cell.spec
    late = spec.model_copy(update={"mission": spec.mission.model_copy(
        update={"clock_start": datetime(2026, 9, 4, 17, 29, 30)})})
    S.prefix_csp(late, None, DEFAULTS, 1001)
    try:
        S.prefix_csp(late, None, DEFAULTS, 1001, max_s_add=35.0)
    except ValueError as e:
        assert "17:31:00" in str(e), e
    else:
        raise AssertionError("not static over the lengthened flight, yet compiled")
    args = S.rail_node_args(late, 1001, DEFAULTS, backend="sitl", episode=on)
    longer = [a if not a.startswith("--max-s=") else
              f"--max-s={S._num(S.rail_max_s(30.0, 35.0))}" for a in args]
    vs, _ = V.make_parser().parse_known_args(longer)
    vs.policy = str(ROOT / vs.policy)
    assert V.flight_length_from(vs.max_s) == S.rail_max_s(30.0) + 35.0
    try:
        V.prefix_preflight(vs)
    except ValueError as e:
        assert "a rule switches" in str(e), e
    else:
        raise AssertionError("the stub flew a CSP that is not static over its --max-s")
    for ep in (on, off):
        try:
            S.rail_node_args(late, 1001, DEFAULTS, backend="sitl", episode=ep, max_s_add=35.0)
        except S.UnsupportedScenario as e:
            assert "--hil-max-s-add" in str(e) and ep.arm_tag in str(e), e
        else:
            raise AssertionError(f"{ep.arm_tag}: a lengthened mission the stub refuses was sent")
    # no consumer, no check; the cell's own clock, the lengthened flight as before
    assert S.rail_node_args(late, 1001, DEFAULTS, backend="sitl", max_s_add=35.0)
    got = S.rail_node_args(spec, 1001, DEFAULTS, backend="sitl", episode=on, max_s_add=35.0)
    assert f"--max-s={S._num(S.rail_max_s(30.0, 35.0))}" in got and got[-1] == "--prefix=on"


def test_a_rail_flight_that_did_not_fly_its_arm_is_not_that_arms_flight():
    """The scenario record of a consumer mission names the arm and the CSP
    the node must announce; a node that announced no prefix arm (it flew its
    plain pilot), the other arm, another law or another CSP is a problem,
    and only the matching identity is none. A mission without a consumer has
    no such record and no such check."""
    arms = _arms("altitude-ceiling-hold")
    spec = arms["prefix-on"].cell.spec
    meta = S.rail_meta(spec, arms["prefix-on"], 1001, "hil", "t_001", "n", None, "c", DEFAULTS)
    c = meta["arm_record"]["consumer"]
    assert meta["family"] == ("hil-stress/altitude_envelope/altitude-ceiling-hold/shield-on"
                              "/prefix-consumer-on")
    assert meta["arm_tag"] == "prefix-on" and meta["arm_record"]["pilot_reads_prefix"] is True
    assert meta["expected_outcome"] is None and meta["expected_failsafe"] is None
    assert meta["labels_not_applied"] == S.LABELS_NOT_APPLIED
    off = S.rail_meta(spec, arms["prefix-off"], 1001, "hil", "t_002", "n", None, "c", DEFAULTS)
    assert off["family"].endswith("/shield-on/prefix-consumer-off")
    assert off["expected_outcome"] == "RTL_triggered" and off["expected_failsafe"] is True
    assert "labels_not_applied" not in off and off["arm_record"]["consumer"]["arm"] == "off"

    def ident(**change):
        return {"vla_identity": {"model_id": PC.MODEL_ID, "kind": "stub",
                                 "prefix": {**c, **change}}}
    assert S.prefix_flight_problems(ident(), meta) == []
    assert S.prefix_flight_problems(ident(csp_content_hash="sha256:other"), meta) == [], \
        "the content hash is recorded, not required"
    assert S.prefix_flight_record(ident(csp_content_hash="sha256:other"), meta)[
        "csp_content_hash_agrees"] is False
    assert S.prefix_flight_record(ident(), meta)["csp_content_hash_agrees"] is True
    none = S.prefix_flight_problems({"vla_identity": {"model_id": VS.MODEL_IDS["constant"],
                                                      "kind": "stub"}}, meta)
    assert len(none) == 1 and "announced no prefix arm" in none[0], none
    assert "announced no prefix arm" in S.prefix_flight_problems({}, meta)[0]
    assert c["wraps_model_hash"].startswith(VS.MODEL_IDS["constant"] + "@src:"), c
    for change, word in (({"arm": "off"}, "prefix arm"),
                         ({"action_set_sha256": "sha256:" + "1" * 64}, "action sets"),
                         ({"wraps": VS.MODEL_IDS["goto"]}, "pilot law"),
                         # the same law's id from another source (vla_stub.py
                         # changed on the rig), and a node that pins none
                         ({"wraps_model_hash": VS.MODEL_IDS["constant"] + "@src:" + "3" * 64},
                          "source pin"),
                         ({"wraps_model_hash": None}, "source pin"),
                         # a rig checkout from before the path test
                         ({"corridor_path_test": None}, "path test"),
                         ({"policy_hash": "sha256:" + "2" * 64}, "policy")):
        got = S.prefix_flight_problems(ident(**change), meta)
        assert len(got) == 1 and word in got[0], (change, got)
    assert S.manifest_extras(meta["arm_record"])["prefix_wraps_model_hash"] == \
        c["wraps_model_hash"]
    wrong_model = {"vla_identity": {"model_id": VS.MODEL_IDS["constant"], "prefix": c}}
    assert any("model" in p for p in S.prefix_flight_problems(wrong_model, meta))
    # No consumer: the record it always was, and nothing to check.
    plain = S.rail_meta(spec, None, 1001, "hil", "t_003", "n", None, "c", DEFAULTS)
    assert "arm_record" not in plain and "labels_not_applied" not in plain
    assert plain["family"] == "hil-stress/altitude_envelope/altitude-ceiling-hold/shield-on"
    assert plain["expected_outcome"] == "RTL_triggered"
    assert S.prefix_flight_problems({}, plain) == []
    assert S.prefix_flight_record({}, plain) is None


class _PrefixRig(TS._FakeRig):
    """TS._FakeRig with a mission that honours --prefix as the VLA stub does:
    its arguments read by the stub's own parser and pre-flight
    (sitl/ros2_vla_stub_node.py: the CSP compiled from the bundle the mission
    was told to load), the headless flight flown through that consumer, and
    the node's /vla/identity written to metrics.json as the Shield node
    records it."""

    def run_mission(self, args, tag, shield, timeout_s):
        vs, _ = V.make_parser().parse_known_args(args)
        vs.bundle = str(Path(self.cfg.bundles_dir) / Path(vs.bundle).name)
        vs.allow_unverified_bundle = True        # the test's bundles are unsigned
        rec = V.prefix_preflight(vs)
        saved = S.run_episode

        def flown(spec, defaults, seed, **kw):
            arm = None if rec is None else {"pilot_reads_prefix": True, "prefix": rec["arm"],
                                            "consumer": rec}
            return saved(spec, defaults, seed, arm=arm, **kw)
        S.run_episode = flown
        try:
            out = super().run_mission(args, tag, shield, timeout_s)
        finally:
            S.run_episode = saved
        mp = self.remote / "demo" / "out" / tag / "metrics.json"
        m = json.loads(mp.read_text(encoding="utf-8"))
        m["vla_identity"] = {"model_id": PC.MODEL_ID if rec else VS.MODEL_IDS[vs.pilot],
                             "kind": "stub", **({"prefix": rec} if rec else {})}
        mp.write_text(json.dumps(m), encoding="utf-8")
        return out


class _HotAppliedRig(_PrefixRig):
    """_PrefixRig whose Shield also moved to generation 1 in flight (a
    hot-apply that reached the mission's REST listener from a client the
    harness does not control): the run folder holds policy_g1.json beside
    policy_g0.json, while the node still announces the generation-0 CSP."""

    def run_mission(self, args, tag, shield, timeout_s):
        out = super().run_mission(args, tag, shield, timeout_s)
        run = self.remote / "demo" / "out" / tag
        (run / "policy_g1.json").write_bytes((run / "policy_g0.json").read_bytes())
        return out


def _hil(td: Path, rig, argv):
    saved = TS._FakeRig
    TS._FakeRig = rig
    try:
        return TS._hil_campaign(argv, td)
    finally:
        TS._FakeRig = saved


def test_a_rail_flight_whose_rules_changed_in_flight_is_not_the_arms_flight():
    """prefix_generation_problems: a consumer flight whose Shield flew a
    generation above 0 (a policy_g<N>.json, or a row's `generation`) is not
    its arm's flight - the node announces the CSP it took off with, so
    prefix_flight_problems cannot see it. Every other flight has no such
    problem. On the fake rig whose Shield moved to generation 1, every
    mission of the campaign is an ERROR, never a scored arm."""
    arms = _arms("nfz-head-on")
    spec = arms["prefix-on"].cell.spec
    meta = S.rail_meta(spec, arms["prefix-on"], 1001, "hil", "t_001", "n", None, "c", DEFAULTS)
    plain = S.rail_meta(spec, None, 1001, "hil", "t_002", "n", None, "c", DEFAULTS)
    g0 = {0: POL}
    assert S.prefix_generation_problems(meta, g0, [{"generation": 0}]) == []
    assert S.prefix_generation_problems(meta, {}, []) == []
    for gens, rows in (({0: POL, 1: POL}, []), (g0, [{"generation": 0}, {"generation": 2}])):
        got = S.prefix_generation_problems(meta, gens, rows)
        assert len(got) == 1 and "WP4-19" in got[0] and "generation" in got[0], got
    assert "[1, 2]" in S.prefix_generation_problems(meta, {1: POL}, [{"generation": 2}])[0]
    # (a bool is not a generation)
    assert S.prefix_generation_problems(meta, g0, [{"generation": True}]) == []
    # no consumer: nothing to check, whatever the generations
    assert S.prefix_generation_problems(plain, {0: POL, 1: POL}, [{"generation": 1}]) == []
    argv = ["--profile", "prefix_ab", "--only", "nfz-head-on", "--hil-campaign", "tstpa"]
    with tempfile.TemporaryDirectory() as td:
        rc, doc, _ = _hil(Path(td), _HotAppliedRig, argv)
        assert rc == 1 and len(doc["results"]) == 3, (rc, doc["results"])
        assert all(r["status"] == "error" and "rules changed in flight" in r["error"]
                   and "WP4-19" in r["error"] for r in doc["results"]), doc["results"]
        assert not doc.get("_arms")


def test_a_consumer_episode_takes_no_mid_flight_rule_change():
    """guardrail/harness_service.py: a HotApply or an EmitEvent to an episode
    of a pilot that consumes the CSP is FAILED_PRECONDITION (WP4-19) before
    anything is queued, acked or forwarded - headless, on a rail (refused
    before the rail's forward), and a session HotApply while such an episode
    flies a rail - and the episode then flies and scores as its arm. THE
    NULL: the same cell under no profile takes the same HotApply and the
    Shield validates generation 1."""
    from types import SimpleNamespace
    from guardrail import harness_service as H
    zone = {"type": "dynamic_nfz", "constraint_type": "hard", "scope": "mission",
            "priority": "P0", "violation_action": "project_fix", "id": "late-zone",
            "vertices": [{"x": 210, "y": 230}, {"x": 220, "y": 230},
                         {"x": 220, "y": 250}, {"x": 210, "y": 250}]}
    fence = {"id": "late", "vertices": [{"x": 200, "y": 200}, {"x": 210, "y": 200},
                                        {"x": 210, "y": 210}, {"x": 200, "y": 210}]}

    def refused(fn, *a, **kw):
        try:
            fn(*a, **kw)
        except H.HarnessError as err:
            return err.code, err.message
        raise AssertionError(f"{fn.__name__} was accepted")

    def never(*a, **kw):
        raise AssertionError("a consumer episode's event reached the rail or the endpoint")

    d = Path(tempfile.mkdtemp(prefix="prefix_events_"))
    try:
        h = H.Harness(runs_dir=d)
        for arm in ("prefix-on", "prefix-off"):
            for live in (False, True):
                eid = h.start_episode(scenario_id="nfz-head-on", profile="prefix_ab",
                                      seed=1001, arm=arm, live=live)["episode_id"]
                code, why = refused(h.hot_apply, eid, 1.0, zone)
                assert code == "FAILED_PRECONDITION" and "WP4-19" in why, (code, why)
                assert arm in why and "HotApply" in why, why
                code, why = refused(h.emit_event, eid, 1.0, "spawn_polygon_fence", fence)
                assert code == "FAILED_PRECONDITION" and "WP4-19" in why, (code, why)
                assert h._get(eid).queued == [] and h.status(eid)["events"] == []
                res = h.end_episode(eid)
                assert res["status"] == "pass", res
        # On a rail: refused before the rail's own forward is reached.
        eid = h.start_episode(scenario_id="nfz-head-on", profile="prefix_ab", seed=1001,
                              arm="prefix-on")["episode_id"]
        e = h._get(eid)
        e.rail = SimpleNamespace(phase="flying")
        h._rail_hot_apply = never
        h._rail_emit_event = never
        assert refused(h.hot_apply, eid, 5.0, zone)[0] == "FAILED_PRECONDITION"
        assert refused(h.emit_event, eid, 5.0, "spawn_polygon_fence", fence)[0] == \
            "FAILED_PRECONDITION"
        # A session HotApply while that episode flies its rail: refused, not
        # forwarded to the mission's Shield.
        sess = SimpleNamespace(session_id="s1", rest_endpoint="http://127.0.0.1:9")
        h._open_session = lambda sid: sess
        h._rail_episode_of = lambda s: e
        h._forward = never
        code, why = refused(h.hot_apply, "", 0.0, zone, session_id="s1")
        assert code == "FAILED_PRECONDITION" and "WP4-19" in why and "session" in why, why
        # The null: the cell's own pilot takes it.
        h2 = H.Harness(runs_dir=d / "plain")
        eid = h2.start_episode(scenario_id="nfz-head-on", seed=1001)["episode_id"]
        ack = h2.hot_apply(eid, 1.0, zone)
        assert ack["validated_generation"] == 1 and ack["rule_id"] == "late-zone", ack
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_hil_campaign_flies_both_arms_and_fails_closed_on_a_rig_that_does_not():
    """--backend hil --profile prefix_ab on the fake rig, one cell and seed:
    three missions (prefix on, off, the A/A repeat), each sent its arm, each
    accepted because its node announced that arm and the expected CSP, each
    scored; the summary reports the pair as a measurement and the A/A pair
    as identical (this rig is deterministic); the score document carries
    what the node announced. THE NULL: on the rig whose mission ignores
    --prefix (an old checkout on the Orin), every mission is an ERROR and
    nothing is scored under the arms' names."""
    argv = ["--profile", "prefix_ab", "--only", "nfz-head-on", "--hil-campaign", "tstpa"]
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc, doc, rig = _hil(td, _PrefixRig, argv)
        res = {r["id"]: r for r in doc["results"]}
        assert rc == 0 and set(res) == {"nfz-head-on#prefix-on", "nfz-head-on#prefix-off",
                                        "nfz-head-on#prefix-on,aa1"}, (rc, sorted(res))
        assert all(r["status"] == "pass" for r in res.values()), res
        sent = {c[1]: c[3][-1] for c in rig.calls if c[0] == "run_mission"}
        assert sent == {"tstpa_001_nfz-head-on": "--prefix=on",
                        "tstpa_002_nfz-head-on": "--prefix=off",
                        "tstpa_003_nfz-head-on": "--prefix=on"}, sent
        on, off = res["nfz-head-on#prefix-on"], res["nfz-head-on#prefix-off"]
        assert on["kpi"]["reached_goal"] is True and off["kpi"]["reached_goal"] is False
        assert on["arm_record"]["pilot_reads_prefix"] is True and on["prefix_consumer"]
        assert res["nfz-head-on#prefix-on,aa1"]["repeat"] == 1
        pc = doc["_arms"]["prefix_consumer"]
        assert (pc["pairs"], pc["aa_pairs"]) == (1, 1), pc
        assert pc["pairs_that_differ"] == ["nfz-head-on seed=1001"]
        assert pc["aa_pairs_that_differ"] == []
        assert doc["_arms"]["groups_whose_arms_flew_differently"] == []
        fig = pc["figures"]["measures"]
        assert fig["mission_success_rate"] == {"on": 1.0, "off": 0.0, "delta_on_minus_off": 1.0}
        assert fig["mean_repair_count_per_episode"]["delta_on_minus_off"] < 0
        score = json.loads((td / "store" / "tstpa_001_nfz-head-on" / S.SCENARIO_SCORE)
                           .read_text(encoding="utf-8"))
        assert score["scenario"]["family"].endswith("/shield-on/prefix-consumer-on")
        assert score["prefix_consumer"]["csp_content_hash_agrees"] is True
        assert score["prefix_consumer"]["announced"]["arm"] == "on"
        # tools/kpi_report.py reads the store's runs as a measured A/B
        eps = [KR.load_run(p, {}, {}) for p in sorted((td / "store").glob("tstpa_*"))]
        assert [e["arms"]["repeat"] for e in eps] == [0, 0, 1]
        assert eps[0]["family"].endswith("/prefix-consumer-on")
        ab, para = KR.arms_sections(eps)
        assert ab["status"] == "measured" and ab["consumer"]["pairs"] == 1, ab
        assert ab["consumer"]["figures"]["aa"]["pairs"] == 1 and para["status"] == "n/m"
        # ...and the report of the campaign (--campaign <its sweep.json>): the
        # campaign's record and header say what pilot flew, the A/B section
        # is measured with its A/A line, the per-paraphrase section is not.
        camp = KR.load_campaign(td / "s.json")
        assert camp["prefix_consumer"] == {"adapter": "action_set", "prefix_off_episodes": 1,
                                           "aa_episodes": 1}, camp["prefix_consumer"]
        rep = KR.build_report([], [], 10, False, "cmd", wp1_path=None, csp_json=None,
                              tick_profiles=None, campaign=str(td / "s.json"))
        assert rep["ab"]["status"] == "measured" and rep["paraphrase"]["status"] == "n/m"
        assert rep["ab"]["consumer"]["figures"]["aa"]["status"] == "measured"
        fams = {e["family"].split("/shield-on")[1] for e in rep["episodes"]}
        assert fams == {"/prefix-consumer-on", "/prefix-consumer-off"}, fams
        md = KR.render_markdown(rep)
        for said in ("**PREFIX-CONSUMER campaign**", "NOT the KPI campaign's",
                     "### Flown by a pilot that consumes the CSP",
                     "| repairs per episode |", "A/A (the noise floor): 1 pair(s)",
                     "not a learned VLA", "csp_flights_20261107.json"):
            assert said in md, said
        # One report, one pilot: these flights beside a flight of the cell's
        # own pilot (the same run without its arm record) are refused.
        plain = dict(eps[1], id="ntx_001_nfz-head-on", arms=None)
        why = KR.mixed_pilots_refusal(eps + [plain])
        assert why and "two pilots" in why and "ntx_001_nfz-head-on" in why, why
        assert KR.mixed_pilots_refusal(eps) is None
        assert KR.mixed_pilots_refusal([plain]) is None
        # (a headless sweep's rows and other runs are not rail scenario flights)
        assert KR.mixed_pilots_refusal(eps + [dict(plain, kind="sweep-json")]) is None
        assert KR.mixed_pilots_refusal(eps + [dict(plain, scenario_id=None)]) is None
    with tempfile.TemporaryDirectory() as td:
        rc, doc, rig = _hil(Path(td), TS._FakeRig, argv)
        assert rc == 1 and len(doc["results"]) == 3
        assert all(r["status"] == "error" and "announced no prefix arm" in r["error"]
                   for r in doc["results"]), doc["results"]
        assert not doc.get("_arms")


# --------------------------------------------------------------------------- #
# the summaries and the KPI report
# --------------------------------------------------------------------------- #

def _scored(cell, seed, prefix, repairs, *, consumer=False, repeat=0, ok=True):
    arm = {"arm": f"prefix-{prefix}", "prefix": prefix, "pilot_reads_text": False}
    if consumer:
        arm.update(pilot_reads_prefix=True, repeat=repeat,
                   consumer={"id": PC.MODEL_ID, "adapter": PC.ADAPTER})
    return {"id": f"{cell}#prefix-{prefix}", "id_cell": cell, "seed": seed, "status": "pass",
            "arm_record": arm,
            "kpi": {"ticks": 100, "p0_escapes": 0, "p0_violation_ticks": 0,
                    "repair_count": repairs, "repaired_ticks": repairs,
                    "mean_repair_magnitude_mps": 2.0 if repairs else None,
                    "mission_success": ok, "reached_goal": ok,
                    "mean_time_to_safe_s": None, "time_to_safe_episodes": 0,
                    "time_to_safe_not_measurable": False}}


def test_the_arms_must_be_one_flight_for_every_pilot_but_a_consumer():
    """arms_summary: a pair flown by a pilot that reads nothing and differs
    is still named a defect; a consuming pilot's differing pair is not, and
    is reported as the measurement, with its A/A pair beside it."""
    blind = [_scored("b", 1, "on", 5), _scored("b", 1, "off", 9)]
    eats = [_scored("c", 1, "on", 2, consumer=True),
            _scored("c", 1, "off", 40, consumer=True, ok=False),
            _scored("c", 1, "on", 2, consumer=True, repeat=1)]
    s = S.arms_summary(blind + eats)
    assert s["groups_whose_arms_flew_differently"] == ["b seed=1"], s
    assert s["prefix_ab_pairs"] == 1 and s["prefix_ab_pairs_that_differ"] == ["b seed=1"]
    assert s["episodes_with_arms"] == 5 and s["cell_seed_groups"] == 2
    assert s["pilot_reads_text"] is False
    pc = s["prefix_consumer"]
    assert pc["pilot_reads_prefix"] is True and pc["consumers"] == [PC.MODEL_ID]
    assert (pc["pairs"], pc["pairs_that_differ"]) == (1, ["c seed=1"])
    assert (pc["aa_pairs"], pc["aa_pairs_that_differ"]) == (1, [])
    m = pc["figures"]["measures"]
    assert m["mean_repair_count_per_episode"] == {"on": 2.0, "off": 40.0,
                                                  "delta_on_minus_off": -38.0}
    assert m["mission_success_rate"]["delta_on_minus_off"] == 1.0
    assert pc["figures"]["aa"]["measures"]["mean_repair_count_per_episode"]["abs_diff"] == 0.0
    assert "not a learned VLA" in pc["note"] and "by construction" in pc["note"]
    # Without a consumer the block is the one it always was, key for key.
    old = S.arms_summary(blind)
    assert set(old) == {"episodes_with_arms", "cell_seed_groups",
                        "groups_whose_arms_flew_differently", "paraphrase_trials",
                        "per_paraphrase", "prefix_ab_pairs", "prefix_ab_pairs_that_differ",
                        "pilot_reads_text", "note"}, sorted(old)
    # An A/A pair that differs is named (headless: a defect).
    noisy = eats[:2] + [_scored("c", 1, "on", 3, consumer=True, repeat=1)]
    assert S.arms_summary(noisy)["prefix_consumer"]["aa_pairs_that_differ"] == ["c seed=1"]


def _ep(cell, seed, prefix, repairs, *, consumer=True, repeat=0, ok=True, rail="hil",
        t2s=None, grade=True, text=False):
    arms = {"arm": f"prefix-{prefix}", "prefix": prefix, "pilot_reads_text": text,
            "paraphrase_id": None, "backend": None, "cell": cell, "seed": seed}
    if consumer:
        arms.update(pilot_reads_prefix=True, consumer=PC.MODEL_ID, adapter=PC.ADAPTER,
                    repeat=repeat)
    return {"id": f"{cell}#{prefix}{repeat}--seed{seed}", "family": "f", "arm": "on",
            "rail": rail, "kpi_grade": grade, "arms": arms, "ticks": 100, "p0_escapes": 0,
            "p0_violation_ticks": 0, "repair_count": repairs, "repaired_ticks": repairs,
            "mean_repair_magnitude_mps": 2.0 if repairs else None,
            "mission_success": ok, "mission_goal_status": "reached" if ok else "missed",
            "mission_outcome": "success" if ok else "fail",
            "mean_time_to_safe_s": t2s, "max_time_to_safe_s": t2s,
            "time_to_safe_episodes": 1 if t2s is not None else 0, "time_to_safe_censored": 0,
            "time_to_safe_not_measurable": False}


def test_the_kpi_report_measures_the_ab_of_a_consuming_pilot_with_its_noise_floor():
    """arms_sections: pairs of a pilot that consumes the CSP make the A/B
    `measured` - every measure on, off and on minus off, the A/A pairs as
    the noise floor, one row per pair - while the per-paraphrase section
    stays n/m (it reads no words). Pairs are per rail. The rendering names
    the pilot and says what is not KPI-grade."""
    eps = [_ep("c", 1, "on", 4, t2s=3.0), _ep("c", 1, "off", 60, ok=False, t2s=5.0),
           _ep("c", 1, "on", 6, repeat=1, t2s=3.4),
           _ep("d", 2, "on", 0), _ep("d", 2, "off", 10),
           # the same cell and seed headless: its own pair, never mixed in
           _ep("c", 1, "on", 1, rail="headless-kinematic", grade=False),
           _ep("c", 1, "off", 99, rail="headless-kinematic", grade=False)]
    ab, para = KR.arms_sections(eps)
    assert ab["status"] == "measured" and ab["pairs"] == 3 and ab["headless_pairs"] == 0, ab
    assert para["status"] == "n/m" and para["trials"] == 0 and para["headless_trials"] == 0
    c = ab["consumer"]
    assert c["pairs"] == 3 and c["consumers"] == [PC.MODEL_ID]
    assert c["rails"] == ["headless-kinematic", "hil"]
    fig = c["figures"]
    assert fig["kpi_grade_episodes"] == {"on": 2, "off": 2} and fig["pairs_on"] == 3
    m = fig["measures"]
    assert m["mission_success_rate"] == {"on": 1.0, "off": round(2 / 3, 6),
                                         "delta_on_minus_off": round(1 - 2 / 3, 6)}
    assert m["mean_repair_count_per_episode"]["on"] == round(5 / 3, 3)
    assert m["mean_repair_count_per_episode"]["off"] == round(169 / 3, 3)
    assert m["mean_time_to_safe_s"] == {"on": 3.0, "off": 5.0, "delta_on_minus_off": -2.0}
    assert m["p0_escapes"] == {"on": 0, "off": 0, "delta_on_minus_off": 0}
    aa = fig["aa"]
    assert aa["status"] == "measured" and aa["pairs"] == 1
    assert aa["measures"]["mean_repair_count_per_episode"] == {
        "first": 4.0, "repeat": 6.0, "abs_diff": 2.0, "max_pair_abs_diff": 2,
        "pairs_one_side_unmeasured": 0}
    assert abs(aa["measures"]["mean_time_to_safe_s"]["abs_diff"] - 0.4) < 1e-9
    assert [(p["cell"], p["seed"], p["rail"]) for p in c["per_cell"]] == [
        ("c", 1, "headless-kinematic"), ("c", 1, "hil"), ("d", 2, "hil")]
    md = "\n".join(KR._render_arms(ab, para))
    assert "### Flown by a pilot that consumes the CSP" in md and PC.MODEL_ID in md
    assert "NOT KPI-GRADE" in md and "not a learned VLA" in md and "by construction" in md
    assert "| repairs per episode | 1.667 | 56.333 | -54.666 | 2.000 (2) |" in md, md
    assert "| mean time to safe (s) | 3.000 | 5.000 | -2.000 | 0.400 (0.400) |" in md, md
    assert "A/A (the noise floor): 1 pair(s)" in md
    assert md.count("n/m -") == 1, "the per-paraphrase section is still n/m"
    # No A/A pair flown: the floor is n/m, said, never a 0.
    ab2, _ = KR.arms_sections(eps[3:5])
    f2 = ab2["consumer"]["figures"]
    assert f2["aa"]["status"] == "n/m" and "no A/A pair" in f2["aa"]["why"]
    md2 = "\n".join(KR._render_arms(ab2, para))
    assert "| repairs per episode | 0.000 | 10.000 | -10.000 | n/m |" in md2, md2
    assert "KPI-grade flights: 1 of 1 with the prefix on, 1 of 1 off." in md2
    # A repair in one A/A flight and none in the other: a difference with no
    # size is counted, not read as 0.
    odd = [_ep("e", 1, "on", 0), _ep("e", 1, "off", 9), _ep("e", 1, "on", 3, repeat=1)]
    f3 = KR.arms_sections(odd)[0]["consumer"]["figures"]["aa"]["measures"]
    assert f3["mean_repair_magnitude_mps"]["max_pair_abs_diff"] is None
    assert f3["mean_repair_magnitude_mps"]["pairs_one_side_unmeasured"] == 1


def test_seeds_that_fly_alike_are_counted_as_one_flight():
    """Headless, a seed does not vary a cell: three seeds of one cell are one
    flight three times. The sweep's summary, its console figures and the KPI
    report give the number of distinct flight pairs (and distinct time-to-
    safe episodes) beside the number of pairs, so 15 pairs of five cells are
    never quoted as 15 pieces of evidence. Pairs that differ are counted
    apart; the figures themselves are unchanged."""
    rows = []
    for seed in (1001, 1002, 1003):
        rows += [_scored("c", seed, "on", 2, consumer=True),
                 _scored("c", seed, "off", 40, consumer=True, ok=False)]
    rows += [_scored("d", 1001, "on", 0, consumer=True), _scored("d", 1001, "off", 10,
                                                                 consumer=True),
             _scored("d", 1002, "on", 1, consumer=True), _scored("d", 1002, "off", 11,
                                                                 consumer=True)]
    pc = S.arms_summary(rows)["prefix_consumer"]
    assert (pc["pairs"], pc["distinct_pairs"]) == (5, 3), pc
    d = pc["figures"]["distinct"]
    assert d["flights"] == {"on": 3, "off": 3} and d["pairs"] == 3, d
    assert pc["figures"]["measures"]["mean_repair_count_per_episode"]["on"] == round(7 / 5, 3)
    lines = KR.prefix_ab_lines(pc["figures"])
    assert any("5 pair(s), 3 distinct flight pair(s)" in x for x in lines), lines
    # all distinct: said so
    one = S.arms_summary(rows[6:])["prefix_consumer"]
    assert (one["pairs"], one["distinct_pairs"]) == (2, 2)
    assert "- Distinct evidence: all 2 pair(s) are distinct flights." in \
        KR.prefix_ab_lines(one["figures"])
    # the KPI report over episodes: distinct per rail and cell; time to safe
    # over distinct episodes
    eps = []
    for seed in (1001, 1002, 1003):
        eps += [_ep("c", seed, "on", 4, t2s=3.0, rail="headless-kinematic", grade=False),
                _ep("c", seed, "off", 60, ok=False, t2s=5.0, rail="headless-kinematic",
                    grade=False)]
    ab, para = KR.arms_sections(eps)
    fig = ab["consumer"]["figures"]
    assert ab["consumer"]["pairs"] == 3 and fig["distinct"]["pairs"] == 1, fig["distinct"]
    assert fig["time_to_safe_episodes"] == {"on": 3, "off": 3}
    assert fig["distinct"]["time_to_safe_episodes"] == {"on": 1, "off": 1}
    md = "\n".join(KR._render_arms(ab, para))
    assert "- 3 pair(s) of one cell and seed (1 distinct flight pair(s))" in md, md
    assert "time to safe is over 1 distinct episode(s) with the prefix on, 1 off" in md, md


def test_the_note_quotes_the_learned_vla_ab_as_its_file_records_it():
    """Every A/B through the consumer is printed beside the one A/B flown by
    a learned VLA reading the CSP (verifier's plan for WP4-15, step 6), and
    with that A/B's own reading: the CSP's effect on the repair rate was not
    above a rule-free neutral prefix's. The figures in the note are the
    file's, so the note cannot drift from the evidence it cites.

    The note also carries the adverse on-versus-off figures of the same
    file, so it does not quote only the replay's least adverse reading: the
    FLOWN A/B (score.ab: mission success on / off over its valid pairs, the
    fail reason of every failed prefix-on flight, and the mean on-minus-off
    repair rate) and the replay's CSP-minus-off repair rate, the replay
    labelled as replayed open loop, not flown."""
    doc = json.loads((ROOT / "docs" / "data" / "csp_flights_20261107.json")
                     .read_text(encoding="utf-8"))
    rs = doc["replay_summary"]
    mean = rs["csp_minus_neutral"]["mean"]
    note = KR.PREFIX_CONSUMER_NOTE
    assert "docs/data/csp_flights_20261107.json" in note
    assert f"({rs['runs']} runs, not flown)" in note and "Replayed open loop" in note, note
    assert f"minus a rule-free neutral prefix of its length mean {mean:.4f}" in note, (
        mean, note)
    assert mean <= 0 and "not above the neutral prefix's" in note
    off_mean = rs["csp_minus_off_repair_rate"]["mean"]
    assert f"repair rate CSP minus off mean {off_mean:+.4f}" in note, (off_mean, note)
    # The flown A/B, from score.ab
    ab = doc["score"]["ab"]
    valid = [p for p in ab["pairs"] if p["valid"]]
    assert len(valid) == ab["valid_pairs"]
    won = sum(1 for p in valid if p["on"]["mission_success"] is True)
    woff = sum(1 for p in valid if p["off"]["mission_success"] is True)
    assert (f"over {len(valid)} seeds: mission success {won}/{len(valid)} on against "
            f"{woff}/{len(valid)} off") in note, (won, woff, note)
    reasons = {r for p in valid if p["on"]["mission_success"] is False
               for r in p["on"]["fail_reasons"]}
    assert won == 0 and reasons == {"unsafe_position_entered"}
    assert "every prefix-on flight failed with unsafe_position_entered" in note
    rr = ab["mean_delta_on_minus_off"]["repair_rate"]
    assert f"repair rate on minus off mean {rr:+.4f}" in note, (rr, note)
    assert rr > 0 and off_mean > 0 and "the CSP did not help" in note
    assert "not a learned VLA" in note and "by construction" in note


def test_the_ab_section_is_unchanged_without_a_consuming_pilot():
    """Episodes of pilots that read nothing: n/m, with the section's text
    and the dict's keys exactly as before; a pilot that reads TEXT: measured
    with the keys it always had (no `consumer`). An arm record without the
    new key is reduced to the seven keys it always was."""
    blind = [_ep("c", 1, "on", 4, consumer=False), _ep("c", 1, "off", 4, consumer=False)]
    ab, para = KR.arms_sections(blind)
    assert ab == {"status": "n/m", "pairs": 0, "headless_pairs": 1,
                  "why": "no prefix on/off pair has been flown by a pilot that reads "
                         "the prompt; the headless pilots ignore text, so their pairs "
                         "are plumbing checks, not an A/B"}, ab
    md = KR._render_arms(ab, para)
    assert md[:3] == ["## A/B: prefix on / off", "",
                      "n/m - no prefix on/off pair has been flown by a pilot that reads the "
                      "prompt; the headless pilots ignore text, so their pairs are plumbing "
                      "checks, not an A/B (1 headless pairs)."], md[:3]
    reads = [_ep("c", 1, "on", 4, consumer=False, text=True),
             _ep("c", 1, "off", 4, consumer=False, text=True, ok=False)]
    ab, _ = KR.arms_sections(reads + blind)
    assert set(ab) == {"status", "pairs", "mission_success_prefix_on", "scored_on",
                       "mission_success_prefix_off", "scored_off", "delta_on_minus_off",
                       "p0_escapes_prefix_on", "p0_escapes_prefix_off", "headless_pairs"}, ab
    assert ab["pairs"] == 1 and ab["headless_pairs"] == 1 and ab["delta_on_minus_off"] == 1.0
    old = {"arm": "pp0,prefix-on", "prefix": "on", "pilot_reads_text": False,
           "paraphrase": {"paraphrase_id": "p", "backend": "stored"}}
    assert KR._arms_of({"arm_record": old, "id_cell": "c", "seed": 3}) == {
        "arm": "pp0,prefix-on", "prefix": "on", "pilot_reads_text": False,
        "paraphrase_id": "p", "backend": "stored", "cell": "c", "seed": 3}
    assert KR.consumer_suffix(old) == "" and KR.consumer_suffix(None) == ""
    assert KR.scenario_family({"backend": "hil", "template": "t", "cell": "c"}, "on") == \
        "hil-stress/t/c/shield-on"
    assert KR._bundle_family({"template": "t", "prefix": "off", "forecast_arm": "grant"},
                             "c", {}, "on") == "stress-bundle/t/c/shield-on/prefix-off"
    eat = {"arm": "prefix-off", "prefix": "off", "pilot_reads_prefix": True}
    assert KR.scenario_family({"backend": "hil", "template": "t", "cell": "c",
                               "arm_record": eat, "max_s_add": 35}, "on") == \
        "hil-stress/t/c/shield-on/prefix-consumer-off/flown-longer-35s"
    assert KR._bundle_family({"template": "t", "prefix": "off", "forecast_arm": "grant",
                              "arm_record": eat}, "c", {}, "on") == \
        "stress-bundle/t/c/shield-on/prefix-consumer-off"


def test_the_headless_profile_reports_the_delta_end_to_end():
    """sweep_scenarios.py --profile prefix_ab (two cells, one seed), headless:
    every episode passes its gates; the summary carries the pairs as the
    measurement and the A/A pairs as identical; the run's own kpi_report.md
    prints the A/B table, the pilot, and that it is functional evidence; and
    tools/kpi_report.py over the bundles reads the same pairs as `measured`,
    none of them KPI-grade, with the per-paraphrase section n/m."""
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        rc = S.main(["--profile", "prefix_ab", "--seed", "1001", "--only", "nfz-head-on",
                     "altitude-floor-recovery", "--keep-logs", "--out", str(td / "s.json"),
                     "--runs", str(td / "runs")])
        doc = json.loads((td / "s.json").read_text(encoding="utf-8"))
        assert rc == 0 and doc["_counts"]["pass"] == 6 and doc["_counts"]["fail"] == 0, doc[
            "_counts"]
        pc = doc["_arms"]["prefix_consumer"]
        assert (pc["pairs"], pc["aa_pairs"], pc["aa_pairs_that_differ"]) == (2, 2, [])
        # (by KPI signature: the floor recovery's two arms differ in repair
        # magnitude only, which the signature does not hold)
        assert "nfz-head-on seed=1001" in pc["pairs_that_differ"]
        assert doc["_arms"]["groups_whose_arms_flew_differently"] == []
        m = pc["figures"]["measures"]
        assert m["mission_success_rate"]["on"] == 1.0 and m["mission_success_rate"]["off"] == 0.5
        assert m["mean_repair_count_per_episode"]["delta_on_minus_off"] < -50
        # no repair on the prefix-on arms (the path test, 2026-11-16): no
        # magnitude to compare, said so rather than read as 0
        assert m["mean_repair_magnitude_mps"]["on"] is None, m["mean_repair_magnitude_mps"]
        assert m["mean_repair_magnitude_mps"]["off"] > 0
        assert m["p0_escapes"] == {"on": 0, "off": 0, "delta_on_minus_off": 0}
        # altitude-floor-recovery starts 7 m below the floor: a time to safe
        # in both arms (the consumer climbs at the cap, as the Shield does)
        assert pc["figures"]["time_to_safe_episodes"] == {"on": 1, "off": 1}
        assert m["mean_time_to_safe_s"]["on"] is not None
        aa = pc["figures"]["aa"]["measures"]
        # every A/A measure identical; the repair magnitude has none to
        # compare on an arm with no repair (None, not 0)
        assert all(v["abs_diff"] in (0, 0.0) for k, v in aa.items()
                   if k != "mean_repair_magnitude_mps"), aa
        assert aa["mean_repair_magnitude_mps"]["abs_diff"] is None, aa
        on = next(r for r in doc["results"] if r["id"] == "nfz-head-on#prefix-on")
        assert on["labels_not_applied"] == S.LABELS_NOT_APPLIED and on["prefix_consumer"]
        keys = set(doc["_cell_rollup"])
        assert "static_nfz/nfz-head-on/shield-on/prefix-consumer-on" in keys
        assert "static_nfz/nfz-head-on/shield-on/prefix-consumer-off" in keys, sorted(keys)
        md = (td / "runs" / "kpi_report.md").read_text(encoding="utf-8")
        assert "## A/B: prefix on / off, flown by a pilot that consumes the CSP" in md
        assert "FUNCTIONAL EVIDENCE, NOT KPI-GRADE" in md and PC.MODEL_ID in md
        assert "| mission success rate | 1.000 | 0.500 | 0.500 | 0.000 (0.000) |" in md, md
        assert "`nfz-head-on` | 1001 |" in md
        # the bundles re-fly from their own extras, consumer and all
        b_on = next(p for p in (td / "runs").iterdir()
                    if p.is_dir() and "nfz-head-on--prefix-on--" in p.name)
        extras = json.loads((b_on / "manifest_extras.json").read_text(encoding="utf-8"))
        assert extras["pilot_reads_prefix"] is True and extras["arm_tag"] == "prefix-on"
        again = S.rerun_from_extras(extras)
        assert again.extra["reached_goal"] is True
        assert again.extra["prefix_consumer"]["stats"]["turned"] > 0
        rep = KR.build_report([str(td / "runs" / "episode-*")], [], 10, False, "test",
                              wp1_path=None, csp_json=None, tick_profiles=None)
        ab = rep["ab"]
        assert ab["status"] == "measured" and ab["consumer"]["pairs"] == 2, ab
        assert ab["consumer"]["figures"]["kpi_grade_episodes"] == {"on": 0, "off": 0}
        assert ab["consumer"]["figures"]["aa"]["pairs"] == 2
        assert rep["paraphrase"]["status"] == "n/m"
        text = KR.render_markdown(rep)
        assert "### Flown by a pilot that consumes the CSP" in text
        assert "FUNCTIONAL EVIDENCE, NOT KPI-GRADE" in text


def test_the_grpc_harness_flies_the_profiles_arms_as_the_sweep_does():
    """guardrail/harness_service.py: a StartEpisode under the profile takes
    the arm by its tag (the A/A repeat included), builds the sweep's arm
    record, flies the consumer and scores the prefix-on arm without the
    cell's labels; a scenario with events is refused before an episode
    exists."""
    from guardrail import harness_service as H
    d = Path(tempfile.mkdtemp(prefix="prefix_harness_"))
    try:
        h = H.Harness(runs_dir=d)
        out = {}
        for arm in ("prefix-on", "prefix-off", "prefix-on,aa1"):
            ref = h.start_episode(scenario_id="altitude-floor-recovery", profile="prefix_ab",
                                  seed=1001, arm=arm)
            rec = h._episodes[ref["episode_id"]].arm_record
            assert rec["pilot_reads_prefix"] is True and rec["arm"] == arm, rec
            out[arm] = h.end_episode(ref["episode_id"])
            assert out[arm]["status"] == "pass", out[arm]
        k = {a: json.loads(b["kpi_json"]) for a, b in out.items()}
        assert k["prefix-on"] == k["prefix-on,aa1"]
        # below the floor the consumer climbs at the cap, as the Shield's
        # recovery does: nothing left to repair
        assert k["prefix-on"]["repair_count"] == 0 < k["prefix-off"]["repair_count"]
        doc = json.loads((Path(out["prefix-on"]["bundle_path"]) / "kpi.json")
                         .read_text(encoding="utf-8"))
        assert doc["arm_record"]["consumer"]["id"] == PC.MODEL_ID
        try:
            h.start_episode(scenario_id="altitude-floor-recovery", profile="prefix_ab",
                            seed=1002, arm="prefix-on,aa1")
        except H.HarnessError as e:
            assert e.code == "INVALID_ARGUMENT" and "prefix-off" in e.message, e.message
        else:
            raise AssertionError("an A/A repeat the profile does not fly was started")
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")
           and (len(sys.argv) < 2 or any(a in k for a in sys.argv[1:]))]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            import traceback
            where = traceback.extract_tb(e.__traceback__)[-1]
            print(f"FAIL  {fn.__name__}\n      line {where.lineno}: {where.line}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            import traceback
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    for _d in (ROOT / S.CAMPAIGN_DIR).glob("tstpa*"):
        shutil.rmtree(_d, ignore_errors=True)
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
