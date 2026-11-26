"""One rule-evaluation code path, and the Shield node's buffer and index.

Run:  python tests/test_shieldcore_onepath.py

The grant (Safety Shield page, "Violation checking"): "The checker reuses the
Policy DSL IR - there is exactly one rule-evaluation code path in the
system." The independent review of 29 Oct (cards WP3-22 / X-13) found more:
the Prefix Compiler's `_RuleClock` and `action_violations`, the autopilot
fence export, the RTL hand-over and the city controller's FenceGuard each
kept their own clock or geometry, held equal to the Shield by tests. Since
2026-11-03 they call the Shield's evaluator instead (guardrail/shield.py:
RuleClock, the per-position tests, escape_direction, ZoneRuler), so this
file no longer tests that copies agree; it tests that there is one:

  * the compiler's clock IS the Shield's (the same functions, inherited),
    and its minute-grained scope test is models.active_at's answer read
    through that clock, over every minute of a week;
  * the source holds no second evaluator: the schedule readers, the
    keep-out and corridor tests and the way out of a zone are defined in
    guardrail/shield.py only, and nothing else in the system calls the
    primitives they wrap. Adding a second one fails here, with its file and
    line;
  * the readers that ask a different question (the fence export, the RTL
    leg, FenceGuard, the dwell count, the CSP's action check) are seen to
    call the Shield's evaluator - a patched evaluator changes their answer;
  * the clearance measures are pinned by what they do. Outside the library
    only planning cost and target identity measure clearance: since
    2026-11-18 the semantic demo's and semantic seek's minimum building
    distance and altitude band, and the semantic A/B counter's fence,
    altitude and clearance seconds, ask the Shield. Since 2026-11-18 the
    follow controller's FenceGuard measures AND judges building clearance
    with the Shield's ClearanceRuler (the Shield's field, clearance_breached)
    - a flown-default change - and its old grid scan
    (nearest_cell_clearance) is DIAGNOSTIC ONLY: one reader,
    tools/onepath_guard_replay.py, replaying stored flights as flown;
  * no comparison with a rule threshold (any float limit of the IR's rule
    classes: min_clearance_m, alt_min_m, speed_max_mps, ...) is made outside
    the checker: every one outside guardrail/{geometry,planar,shield,ir}.py
    is found by an AST scan (aliases followed) and fails, unless it is named
    here as not a verdict (a planning margin that shares a field's name, a
    scenario generator's spawn placement).

The Shield node's architecture (card WP3-23) is tested at the end: the
sliding-window buffer holds the last 50 actions and their predicted
trajectories and is what the checker reads, and moving dynamic_nfz zones go
into an R-tree re-indexed per tick.
"""
from __future__ import annotations

import ast
import math
import sys
from datetime import datetime, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import shield as SH                                      # noqa: E402
from guardrail.models import Action4D, Policy, State                    # noqa: E402
from guardrail.shield import RuleClock, Shield                          # noqa: E402

HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

# Where the system's code lives. Not scanned: tests/ (its legacy copies are
# the nulls the identity tests compare against). training/ is scanned since
# 2026-11-17: the behaviour-cloned pilots' input features (features.py, the
# nearest fence and an "inside" flag) are still computed there - what the
# pilot sees, not a verdict - but with the library's measures
# (guardrail/geometry.py edge_distance, nearest_edge_point and the Shield's
# zone_breached), the same numbers as before (test below).
SOURCE_DIRS = ("guardrail", "demo", "sitl", "tools", "experiments", "deploy", "scripts",
               "training")


def _sources():
    for d in SOURCE_DIRS:
        base = ROOT / d
        if not base.is_dir():
            continue
        for p in sorted(base.rglob("*.py")):
            if "__pycache__" in p.parts:
                continue
            yield p


def _rel(p: Path) -> str:
    return p.relative_to(ROOT).as_posix()


def _trees():
    out = []
    for p in _sources():
        try:
            tree = ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
        except (SyntaxError, UnicodeDecodeError):
            continue
        out.append((_rel(p), tree))
    return out


_TREES = None


def trees():
    global _TREES
    if _TREES is None:
        _TREES = _trees()
    return _TREES


def _called_name(node: ast.Call) -> str | None:
    f = node.func
    if isinstance(f, ast.Name):
        return f.id
    if isinstance(f, ast.Attribute):
        return f.attr
    return None


# ----------------------------------------------------------- the rule clock

def test_the_compilers_rule_clock_is_the_shields():
    """compiler._RuleClock adds sampling and wording only: every method that
    decides whether a rule is in force is the Shield's own function object,
    not a copy held equal by a test."""
    from guardrail import compiler as K
    assert issubclass(K._RuleClock, RuleClock)
    for name in ("on", "enforced", "holders", "involved", "off_switches",
                 "has_window", "minute_on", "_base"):
        assert name not in K._RuleClock.__dict__, f"compiler._RuleClock redefines {name}"
    assert K._RuleClock.on is RuleClock.on and K._RuleClock.holders is RuleClock.holders


def _pol(*rules):
    return Policy.model_validate({"policy_id": "clock", "constraints": list(rules)})


def _sq(x0, y0, x1, y1):
    return [{"x": x0, "y": y0}, {"x": x1, "y": y0}, {"x": x1, "y": y1}, {"x": x0, "y": y1}]


def _timed_policy():
    return _pol(
        {"id": "yard", "type": "polygon_fence", **HARD_P0, "vertices": _sq(0, 0, 10, 10),
         "valid_time": {"recurrence": {"start_time": "07:30", "end_time": "17:30"}}},
        {"id": "held", "type": "polygon_fence", **HARD_P0, "vertices": _sq(40, 0, 50, 10)},
        {"id": "sw-off", "type": "time_window_switch", **HARD_P0, "target_id": "held",
         "active": False, "valid_time": {"recurrence": {"start_time": "08:00",
                                                        "end_time": "08:30"}}})


def test_every_reader_of_the_clock_asks_the_shields_rule_clock():
    """A RuleClock whose answer is patched changes what the Shield's
    schedule, the CSP's exact clock, the fence export's in_force_now and the
    dwell count say - so each of them asks it, and none keeps a copy."""
    from guardrail import compiler as K
    from guardrail.fence_export import policy_zones
    from guardrail.replay import zone_dwell
    pol = _timed_policy()
    when = datetime(2026, 10, 12, 8, 15)                 # Monday: switch holds 'held' off
    sh = Shield(pol, now=lambda: when, escalation=False, legacy_metres=True)
    status = {r["id"]: r["in_force"] for r in sh.rule_status(State(x=-50, y=-50, up=10))}
    assert status["yard"] and not status["held"], status
    held = next(c for c in pol.constraints if c.id == "held")
    assert not K._RuleClock(pol).on(held, when)
    zones, skipped = policy_zones(pol, now=when, column_m=30.0)
    assert {e["id"]: e.get("in_force_now") for e in skipped}["held"] is False
    pts = [{"x": 45.0, "y": 5.0, "up": 10.0, "t": 0.0, "clock": when.isoformat()}]
    assert zone_dwell(sh, pts, 0.1, clock_key="clock")["nfz_off_by_clock_s"] == {"held": 0.1}

    orig = RuleClock.on
    RuleClock.on = lambda self, rule, x: True             # everything in force
    try:
        sh2 = Shield(pol, now=lambda: when, escalation=False, legacy_metres=True)
        status2 = {r["id"]: r["in_force"] for r in sh2.rule_status(State(x=-50, y=-50, up=10))}
        assert status2["held"], "the Shield's schedule does not read RuleClock"
        assert K._RuleClock(pol).on(held, when), "the compiler keeps its own clock"
        _, skipped2 = policy_zones(pol, now=when, column_m=30.0)
        assert {e["id"]: e.get("in_force_now") for e in skipped2}["held"] is True, \
            "the fence export keeps its own clock"
        assert zone_dwell(sh2, pts, 0.1, clock_key="clock")["nfz_by_zone_s"] == {"held": 0.1}, \
            "the dwell count keeps its own clock"
    finally:
        RuleClock.on = orig


def test_the_minute_reading_is_models_active_at_through_the_one_clock():
    """The Prefix Compiler's scope test for a rule no switch touches has
    always been models.ConstraintBase.active_at (whole minutes, inclusive
    end). It now reads THE clock at the start of the minute instead
    (RuleClock.minute_on) - the same answer at every minute of a week, at
    the minute, a microsecond in, mid-minute and its last microsecond, for
    ordinary, all-day, wrapping, one-minute and 24:00 windows."""
    windows = [("07:30", "17:30", ["Mon", "Tue", "Wed", "Thu", "Fri"]),
               ("00:00", "23:59", ["Sat", "Sun"]),
               ("22:00", "06:00", ["Mon", "Wed", "Sun"]),
               ("12:00", "12:00", ["Tue"]),
               ("18:00", "24:00", ["Thu"]),
               ("23:59", "00:01", ["Fri", "Sat"])]
    rules = [Policy.model_validate({"policy_id": "w", "constraints": [{
        "id": f"z{i}", "type": "polygon_fence", **HARD_P0, "vertices": _sq(0, 0, 1, 1),
        "valid_time": {"recurrence": {"start_time": a, "end_time": b, "days": d}}}]}
    ).constraints[0] for i, (a, b, d) in enumerate(windows)]
    t0 = datetime(2026, 10, 12)                          # a Monday
    n = 0
    for m in range(7 * 1440):
        base = t0 + timedelta(minutes=m)
        for off in (timedelta(0), timedelta(microseconds=1), timedelta(seconds=30),
                    timedelta(seconds=59, microseconds=999999)):
            x = base + off
            for r in rules:
                assert RuleClock.minute_on(r, x) == r.active_at(x), (r.id, x)
                n += 1
    assert RuleClock.minute_on(rules[0], None) is True
    assert n == 7 * 1440 * 4 * len(rules)


# ---------------------------------------------------- no second evaluator

# name -> the files allowed to DEFINE it: the rule evaluators.
_DEFINED_ONLY_IN = {
    "recurrence_active": {"guardrail/shield.py"},
    "_hhmm_s": {"guardrail/shield.py"},
    "escape_direction": {"guardrail/shield.py"},
    "zone_breached": {"guardrail/shield.py"},
    "corridor_breached": {"guardrail/shield.py"},
    "corridor_offset": {"guardrail/shield.py"},
    "in_zone_band": {"guardrail/shield.py"},
    "band_holds": {"guardrail/shield.py"},
    "nearest_cell_clearance": {"guardrail/shield.py"},
    "altitude_band_report": {"guardrail/shield.py"},
    "min_clearance_on_map": {"guardrail/shield.py"},
    # the rule thresholds' one comparison each (2026-11-18)
    "clearance_breached": {"guardrail/shield.py"},
    "altitude_outside": {"guardrail/shield.py"},
    "altitude_out_of_band": {"guardrail/shield.py"},
    "zone_band_outside": {"guardrail/shield.py"},
    "cap_exceeded": {"guardrail/fsm.py"},
    # FenceGuard's only threshold test, asking the ruler's
    "building_breached": {"demo/follow_vlm.py"},
}

# name -> the files allowed to CALL it: primitives the Shield wraps. Anything
# else asks the Shield's evaluator (zone_breached, escape_direction,
# corridor_offset, RuleClock), so the same comparison is made in one place.
_CALLED_ONLY_IN = {
    # models' own minute-grained schedule reader: the system reads schedules
    # through RuleClock (minute_on is its answer); models.py defines it.
    "active_at": {"guardrail/models.py"},
    "point_in_fence": {"guardrail/shield.py"},
    "push_out_direction": {"guardrail/shield.py"},
    "nearest_on_polyline": {"guardrail/shield.py"},
    # DIAGNOSTIC ONLY since 2026-11-18: the grid scan every earlier follow
    # flight's FenceGuard flew, kept to replay those flights as flown
    # (GridScanGuard). Nothing in a flight loop may call it.
    "nearest_cell_clearance": {"tools/onepath_guard_replay.py"},
}


def test_no_second_rule_evaluator_is_defined_anywhere_in_the_system():
    """Every schedule reader, keep-out / corridor test and escape direction
    is defined in guardrail/shield.py; and no class outside it implements a
    rule clock (a method `on(self, rule, x)` or `_base(rule, x)`, the shape
    the compiler's copy had)."""
    bad = []
    for rel, tree in trees():
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                allowed = _DEFINED_ONLY_IN.get(node.name)
                if allowed is not None and rel not in allowed:
                    bad.append(f"{rel}:{node.lineno} defines {node.name}")
            if isinstance(node, ast.ClassDef) and rel != "guardrail/shield.py":
                for item in node.body:
                    if not isinstance(item, ast.FunctionDef):
                        continue
                    args = [a.arg for a in item.args.args]
                    if (item.name == "on" and args[:3] == ["self", "rule", "x"]) or \
                            (item.name == "_base" and args[:2] == ["rule", "x"]):
                        bad.append(f"{rel}:{item.lineno} {node.name}.{item.name}: "
                                   f"a rule clock of its own")
    assert not bad, "a second rule evaluator:\n      " + "\n      ".join(bad)


def test_nothing_outside_the_shield_calls_the_primitives_it_wraps():
    bad = []
    n = 0
    for rel, tree in trees():
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = _called_name(node)
                allowed = _CALLED_ONLY_IN.get(name)
                if allowed is not None:
                    n += 1
                    if rel not in allowed:
                        bad.append(f"{rel}:{node.lineno} calls {name}")
    assert not bad, ("rule evaluation outside the Shield's evaluator:\n      "
                     + "\n      ".join(bad))
    assert n > 10, f"the scan found only {n} calls: is it reading the source?"


def test_the_scan_finds_a_second_evaluator_when_there_is_one():
    """The null for the two scans: a module that re-implements the clock and
    calls the zone test itself is reported."""
    src = ("from guardrail.geometry import point_in_fence\n"
           "class MyClock:\n"
           "    def on(self, rule, x):\n"
           "        return rule.active_at(x)\n"
           "def inside(z, x, y, up):\n"
           "    return point_in_fence(x, y, up, z)\n"
           "def recurrence_active(rec, when):\n"
           "    return True\n")
    global _TREES
    saved = trees()
    _TREES = saved + [("guardrail/mine.py", ast.parse(src))]
    try:
        for fn in (test_no_second_rule_evaluator_is_defined_anywhere_in_the_system,
                   test_nothing_outside_the_shield_calls_the_primitives_it_wraps):
            try:
                fn()
            except AssertionError as e:
                assert "guardrail/mine.py" in str(e), e
            else:
                raise AssertionError(f"{fn.__name__} missed a planted second evaluator")
    finally:
        _TREES = saved


# ------------------------------------------ WP3-23: the buffer is the input

def _fenced():
    return Policy.model_validate({"policy_id": "buf", "constraints": [
        {"id": "box", "type": "polygon_fence", **HARD_P0, "margin_m": 1.0,
         "vertices": _sq(20, -5, 30, 5)}]})


def test_the_checker_reads_the_trajectory_the_buffer_holds():
    """Grant node architecture: action -> sliding-window buffer (last 50
    actions + predicted trajectory) -> violation checker. A buffer whose
    admitted trajectory is moved 1 km away blinds the checker to a zone the
    real forecast runs into - so the checker reads the buffer, not a forecast
    of its own. The null: with the buffer untouched the zone is seen."""
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=6.0)       # 6 m/s at the box
    sh = Shield(_fenced(), escalation=False, legacy_metres=True)
    d = sh.filter(st, a, t=0.0)
    assert [v.rule_id for v in d.violations] == ["box"], d.violations
    orig = SH.SlidingWindow.admit

    def far(self, tick, state, action, trajectory):
        moved = [(t, p._replace(x=p.x + 1000.0)) for t, p in trajectory]
        return orig(self, tick, state, action, moved)
    SH.SlidingWindow.admit = far
    try:
        blind = Shield(_fenced(), escalation=False, legacy_metres=True).filter(st, a, t=0.0)
    finally:
        SH.SlidingWindow.admit = orig
    assert blind.violations == [], "the checker forecast the action itself"


def test_the_buffer_keeps_the_last_50_actions_with_their_trajectories():
    sh = Shield(_fenced(), escalation=False, legacy_metres=True)
    x = -40.0
    for k in range(60):
        sh.filter(State(x=x, y=float(k % 3), up=10.0), Action4D(vx=2.0, vy=0.1 * k), t=0.1 * k)
        x += 0.2
    hist = sh.history
    assert len(hist) == SH.HISTORY_LEN == 50
    assert [round(r.decision.raw.vy, 9) for r in hist] == [round(0.1 * k, 9) for k in range(10, 60)]
    for rec in (hist[0], hist[-1]):
        want = sh.forecast(rec.state, rec.decision.raw)
        got = rec.trajectory
        assert got is not None and len(got) == len(want) == 51, (len(got or ()), len(want))
        assert all(abs(t1 - t2) < 1e-12 and (p.x, p.y, p.up) == (q.x, q.y, q.up)
                   for (t1, p), (t2, q) in zip(got, want))
        assert got[0][0] == 0.0 and abs(got[-1][0] - sh.lookahead_s) < 1e-9
    n = len(sh.history)
    sh.advise(State(x=0.0, y=0.0, up=10.0), Action4D(vx=1.0))     # admits nothing
    assert len(sh.history) == n and sh._buffer.head is None
    sh.reset_episode()
    assert sh.history == () and sh._buffer.head is None


# ---------------------------------- WP3-23: moving zones in a per-tick R-tree

def _moving_load():
    import bench_shield_50rules as B
    data = B.fifty_rule_policy().model_dump(mode="json", exclude_none=True)
    for i, c in enumerate(data["constraints"]):
        if c["type"] == "polygon_fence":
            c.update(type="dynamic_nfz",
                     motion={"vx_mps": 1.0 - 0.05 * (i % 7), "vy_mps": 0.3 * (i % 3 - 1),
                             "yaw_rate_dps": 6.0 * (i % 4 == 0)})
    return Policy.model_validate(data), B


def test_moving_zones_are_queried_through_an_rtree_rebuilt_per_tick():
    """The tick's R-tree over the moving zones' reach boxes returns exactly
    the zones whose box meets the query (boundary contact included), for
    random boxes and for boxes that touch a reach box edge to edge; it is
    built once per tick and horizon."""
    import random
    from guardrail.ir import anchor_reach
    pol, _ = _moving_load()
    sh = Shield(pol, escalation=False)
    sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D(), t=0.0)
    rng = random.Random(5)
    n_hit = 0
    for k in range(1, 40):
        t = 2.5 * k
        sh.filter(State(x=0.0, y=0.0, up=10.0), Action4D(), t=t)
        tick = sh._new_tick(None)
        moving = [r for r in sh.ir.fences if r.moving]
        assert len(moving) == 48
        horizon = 5.0
        boxes = {r.index: anchor_reach(r, tick._age(r.rule), horizon) for r in moving}
        queries = []
        for _ in range(25):
            x0, y0 = rng.uniform(-60, 360), rng.uniform(-60, 260)
            queries.append((x0, y0, x0 + rng.uniform(0, 30), y0 + rng.uniform(0, 30)))
        for b in list(boxes.values())[:5]:                  # edge to edge
            queries.append((b[2], b[3], b[2] + 3.0, b[3] + 3.0))
        for q in queries:
            got = tick.moving_near(sh.ir, q, horizon)
            want = {i for i, b in boxes.items()
                    if b[0] <= q[2] and b[2] >= q[0] and b[1] <= q[3] and b[3] >= q[1]}
            assert want <= got, (t, q, sorted(want - got))
            # The pad may add a zone only within _INDEX_PAD_M of the box.
            for i in got - want:
                b = boxes[i]
                gap = max(b[0] - q[2], q[0] - b[2], b[1] - q[3], q[1] - b[3])
                assert gap <= SH._INDEX_PAD_M, (i, gap)
            n_hit += len(want)
        assert ("moving_tree", id(sh.ir), horizon) in tick._swept
    assert n_hit > 50, n_hit
    tick = sh._new_tick(None)
    assert tick.moving_near(sh.ir, (math.nan, 0.0, 1.0, 1.0), 5.0) is None
    # The tree is the IR's (PolicyIR.moving_index), built once per tick and
    # horizon however many queries the tick makes.
    from guardrail.ir import PolicyIR
    built = []
    orig = PolicyIR.moving_index

    def counted(self, age_of, horizon_s):
        built.append(horizon_s)
        return orig(self, age_of, horizon_s)
    PolicyIR.moving_index = counted
    try:
        for q in queries:
            tick.moving_near(sh.ir, q, 5.0)
        tick.moving_near(sh.ir, queries[0], 3.0)
    finally:
        PolicyIR.moving_index = orig
    assert built == [5.0, 3.0], built


def test_the_moving_rtree_changes_no_decision_and_prunes_the_rest():
    """The 50-rule load with 48 moving zones (translating and turning), aged
    0-90 s: every _check and filter() decision equals the one from a Shield
    that looks at every moving zone one by one (the index switched off), and
    the index leaves most zones unasked."""
    pol, B = _moving_load()

    class NoIndex(Shield):
        def _moving_near(self, ir, tick, box, horizon):
            return None

    import random
    rng = random.Random(9)
    pairs = B.samples("near", 150, 21) + B.samples("grid", 150, 22)
    rng.shuffle(pairs)
    fast, slow = Shield(pol.model_copy(deep=True)), NoIndex(pol.model_copy(deep=True))
    asked = {"fast": 0, "slow": 0}
    for name, sh in (("fast", fast), ("slow", slow)):
        orig = sh._reach

        def counted(rec, tick, box, horizon, _o=orig, _n=name):
            if rec.moving:
                asked[_n] += 1
            return _o(rec, tick, box, horizon)
        sh._reach = counted
    for k, (st, a) in enumerate(pairs):
        t = 90.0 * k / len(pairs)
        if k % 3 == 0:
            da, db = fast.filter(st, a, t=t), slow.filter(st, a, t=t)
            assert da.model_dump() == db.model_dump(), k
        else:
            va = [v.model_dump() for v in fast._check(st, a)]
            vb = [v.model_dump() for v in slow._check(st, a)]
            assert va == vb, (k, va, vb)
    assert asked["fast"] < asked["slow"] / 4, asked


# ------------------- WP3-22 / X-13: one geometry library, found by what it does
#
# The scans above go by NAME: a second evaluator called something else, or a
# shapely call made inline, was not seen (Appendix A, WP3-22: "the source
# scan is by name and shape: a copy under a new name is not caught"). This
# one goes by what each function's own body DOES. A function outside the
# library that calls a shapely predicate or distance on a geometry
# (`.contains(p)`, `.distance(p)`, `shapely.covers(a, b)`, `nearest_points`,
# ...), or carries the body of a hand-written containment test (the even-odd
# ray cast: `(y1 > y) != (y2 > y)`) or point-to-segment distance (a
# projection clamped to [0, 1] measured with hypot/sqrt), is a second
# geometry - whatever it is called - and fails here with its file, line and
# what it did. Readers call the library's functions (guardrail/geometry.py,
# planar.py; the Shield's verdicts in shield.py) instead.

_LIBRARY = {"guardrail/geometry.py", "guardrail/planar.py", "guardrail/shield.py",
            "guardrail/ir.py"}

_SPATIAL = {"contains", "contains_xy", "contains_properly", "covers", "covered_by", "within",
            "intersects", "crosses", "overlaps", "touches", "disjoint", "relate",
            "relate_pattern", "distance", "dwithin", "hausdorff_distance",
            "frechet_distance", "nearest_points"}

# The functions outside the library that still measure a point against a
# line of their own, each with why it is not a rule's geometry. An entry
# that no longer matches anything fails too, so the list cannot rot.
_NOT_A_RULE = {
    ("demo/pedestrians.py", "_dist_to_route"):
        "where scenery pedestrians stand along the demo route; no rule reads it",
    ("demo/trail.py", "_project"):
        "the controller's breadcrumb trail of the car (index and fraction along it)",
    ("tools/citylife_layout.py", "_dist_to_segment"):
        "CityLife level authoring: kerb and road layout",
    ("tools/citylife_mcp/verify_peds.py", "_seg_dist"):
        "CityLife level authoring: checks pedestrian routes against roads",
    ("tools/citylife_peds.py", "_project_s"):
        "CityLife level authoring: pedestrian route arc length",
    ("tools/citylife_routes.py", "distance_to_path"):
        "CityLife level authoring: car route geometry",
    ("tools/citylife_traffic_model.py", "update_eff"):
        "the scenery traffic model's car-following (a projection along a lane)",
    ("tools/profile_shield_tick.py", "calibrate"):
        "host-speed benchmark (a fixed shapely workload profiles are normalised "
        "against); no rule verdict, and changing its workload would break "
        "comparison with earlier calibrations",
}

_ORDER = (ast.Gt, ast.Lt, ast.GtE, ast.LtE)


def _spatial_call(n: ast.Call) -> str | None:
    """The shapely predicate / distance `n` makes, or None. A method call
    with ONE geometry argument (`a.contains(p)`, `a.distance(p)`), any call
    on the shapely module (`shapely.covers(a, b)`), or nearest_points.
    `ruler.distance(x, y)` (two numbers: the library's ZoneRuler) is not."""
    f = n.func
    if isinstance(f, ast.Attribute) and f.attr in _SPATIAL:
        if isinstance(f.value, ast.Name) and f.value.id == "shapely":
            return f"shapely.{f.attr}"
        if len(n.args) == 1 and not isinstance(n.args[0], ast.Starred) and not n.keywords:
            return f".{f.attr}"
    if isinstance(f, ast.Name) and f.id == "nearest_points":
        return "nearest_points"
    return None


def _ray_cast(n) -> bool:
    """`(a > y) != (b > y)`: the even-odd crossing test of a hand-written
    point-in-polygon."""
    if not (isinstance(n, ast.Compare) and len(n.ops) == 1 and isinstance(n.ops[0], ast.NotEq)):
        return False
    a, b = n.left, n.comparators[0]
    return (isinstance(a, ast.Compare) and isinstance(b, ast.Compare)
            and len(a.ops) == 1 and len(b.ops) == 1
            and isinstance(a.ops[0], _ORDER) and isinstance(b.ops[0], _ORDER)
            and ast.dump(a.comparators[0]) == ast.dump(b.comparators[0]))


def _const(n, vals) -> bool:
    return isinstance(n, ast.Constant) and not isinstance(n.value, bool) and n.value in vals


def _clamp01(n) -> bool:
    """max(0, min(1, <a quotient>)) or min(1, max(0, ...)): a projection
    parameter clamped to a segment."""
    if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id in ("max", "min") and len(n.args) == 2):
        return False
    outer = n.func.id
    lo, hi = (0, 0.0), (1, 1.0)
    want_o, want_i = (lo, hi) if outer == "max" else (hi, lo)
    inner_name = "min" if outer == "max" else "max"
    for c, inner in ((n.args[0], n.args[1]), (n.args[1], n.args[0])):
        if not (_const(c, want_o) and isinstance(inner, ast.Call)
                and isinstance(inner.func, ast.Name) and inner.func.id == inner_name
                and len(inner.args) == 2):
            continue
        for c2, e in ((inner.args[0], inner.args[1]), (inner.args[1], inner.args[0])):
            if _const(c2, want_i) and any(isinstance(x, ast.BinOp) and isinstance(x.op, ast.Div)
                                          for x in ast.walk(e)):
                return True
    return False


def _own_nodes(fn):
    """The nodes of `fn`'s own body: a nested def is a function of its own
    (scanned on its own); a lambda's body counts as the enclosing one's."""
    out, stack = [], list(ast.iter_child_nodes(fn))
    while stack:
        x = stack.pop()
        if isinstance(x, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        out.append(x)
        stack.extend(ast.iter_child_nodes(x))
    return out


def _measures(nodes) -> bool:
    for n in nodes:
        if isinstance(n, ast.Call):
            f = n.func
            name = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None)
            if name in ("hypot", "sqrt"):
                return True
        if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Pow) and _const(n.right, (0.5,)):
            return True
    return False


def geometry_of_their_own(tree_list=None) -> dict:
    """{(file, function): (line, [what it does])} for every function outside
    the library that measures geometry itself."""
    found = {}
    for rel, tree in (tree_list if tree_list is not None else trees()):
        if rel in _LIBRARY:
            continue
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            nodes = _own_nodes(fn)
            what = set()
            for n in nodes:
                if isinstance(n, ast.Call):
                    s = _spatial_call(n)
                    if s:
                        what.add(s)
                    if _clamp01(n):
                        what.add("clamp")
                if _ray_cast(n):
                    what.add("ray cast")
            if "clamp" in what:
                what.discard("clamp")
                if _measures(nodes):
                    what.add("point-to-segment distance")
            if what:
                found[(rel, fn.name)] = (fn.lineno, sorted(what))
    return found


def test_no_function_outside_the_library_measures_a_zone_itself():
    """By what the code does, not what it is called: every containment,
    distance, crossing and covering test outside the library calls the
    library's functions. What is left (`_NOT_A_RULE`) is route geometry no
    rule reads, each with its reason; a stale entry fails as well."""
    found = geometry_of_their_own()
    bad = [f"{rel}:{line} {name}() {', '.join(what)}"
           for (rel, name), (line, what) in sorted(found.items())
           if (rel, name) not in _NOT_A_RULE]
    assert not bad, ("geometry of its own outside the library (call guardrail/geometry.py "
                     "or the Shield's verdicts instead):\n      " + "\n      ".join(bad))
    stale = sorted(k for k in _NOT_A_RULE if k not in found)
    assert not stale, f"_NOT_A_RULE names functions that no longer measure anything: {stale}"
    assert len(trees()) > 200, f"the scan read only {len(trees())} files"


def test_the_geometry_scan_catches_a_copy_under_any_name():
    """The null: copies of the library's tests planted under new names - an
    inline shapely containment, a polygon distance, a vectorised shapely
    call, a hand-written ray cast and a point-to-segment distance - are each
    reported; the same questions asked of the library are not; and neither
    is a two-number `.distance(x, y)` (the ZoneRuler) or a clamp that is not
    a distance."""
    src = ("import math, shapely\n"
           "from guardrail.geometry import ring_contains, signed_edge_distance\n"
           "def zebra(poly, p):\n    return poly.contains(p)\n"
           "def quokka(poly, p):\n    return -poly.exterior.distance(p)\n"
           "def wombat(region, pts):\n    return shapely.covers(region, pts)\n"
           "def kiwi(x, y, vs):\n"
           "    inside = False\n"
           "    for (x1, y1), (x2, y2) in zip(vs, vs[1:] + vs[:1]):\n"
           "        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):\n"
           "            inside = not inside\n"
           "    return inside\n"
           "def emu(px, py, ax, ay, bx, by):\n"
           "    vx, vy = bx - ax, by - ay\n"
           "    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))\n"
           "    return math.hypot(px - ax - t * vx, py - ay - t * vy)\n"
           "def fine(poly, x, y, ruler):\n"
           "    a = ring_contains(poly, x, y) and signed_edge_distance(poly, x, y) < 0\n"
           "    return a, ruler.distance(x, y), max(0.0, min(1.0, x / 3.0))\n")
    planted = [("tools/mine.py", ast.parse(src))]
    found = geometry_of_their_own(planted)
    assert set(found) == {("tools/mine.py", n) for n in
                          ("zebra", "quokka", "wombat", "kiwi", "emu")}, found
    assert found[("tools/mine.py", "kiwi")][1] == ["ray cast"]
    assert found[("tools/mine.py", "emu")][1] == ["point-to-segment distance"]
    global _TREES
    saved = trees()
    _TREES = saved + planted
    try:
        try:
            test_no_function_outside_the_library_measures_a_zone_itself()
        except AssertionError as e:
            assert "tools/mine.py:3 zebra()" in str(e), e
        else:
            raise AssertionError("the scan missed the planted copies")
    finally:
        _TREES = saved


def _rand_ring(rng, n=None):
    """A random simple (star-shaped) ring round a random centre."""
    n = n or rng.randint(3, 9)
    cx, cy = rng.uniform(-30, 30), rng.uniform(-30, 30)
    angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(n))
    return [(cx + r * math.cos(a), cy + r * math.sin(a))
            for a, r in ((a, rng.uniform(3, 20)) for a in angs)]


def test_the_shared_planar_measures_give_the_copies_answers_bit_for_bit():
    """The four private copies the library replaced (the compiler's, the
    fence export's, models' and the October deck's), kept here as nulls:
    every answer bit-identical on 4000 random points against random rings,
    vertices and edges included."""
    import random
    from guardrail import compiler as K, fence_export as FE, models as MD
    from guardrail.planar import point_in_ring_xy, ring_edge_distance, segment_distance

    def old_pip(x, y, poly):
        inside = False
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < xc:
                    inside = not inside
        return inside

    def old_pt_seg(px, py, a, b, eps):
        vx, vy = b[0] - a[0], b[1] - a[1]
        L2 = vx * vx + vy * vy
        t = 0.0 if (L2 == 0 if eps is None else L2 < eps) else \
            max(0.0, min(1.0, ((px - a[0]) * vx + (py - a[1]) * vy) / L2))
        return math.hypot(px - (a[0] + t * vx), py - (a[1] + t * vy))

    def old_models_pir(x, y, ring):
        inside = False
        n = len(ring)
        for i in range(n):
            (ax, ay), (bx, by) = ring[i], ring[(i + 1) % n]
            if MD._segments_meet((ax, ay), (bx, by), (x, y), (x, y)):
                return True
            if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
                inside = not inside
        return inside

    rng = random.Random(1117)
    n = 0
    for _ in range(400):
        ring = _rand_ring(rng)
        pts = [(rng.uniform(-55, 55), rng.uniform(-55, 55)) for _ in range(8)] + \
            [ring[rng.randrange(len(ring))]] + [((ring[0][0] + ring[1][0]) / 2,
                                                  (ring[0][1] + ring[1][1]) / 2)]
        for x, y in pts:
            assert point_in_ring_xy(x, y, ring) == old_pip(x, y, ring) == \
                K._point_in_poly(x, y, ring) == FE._point_in_polygon(x, y, ring)
            assert MD._point_in_ring(x, y, ring) == old_models_pir(x, y, ring)
            want = min(old_pt_seg(x, y, ring[i], ring[(i + 1) % len(ring)], None)
                       for i in range(len(ring)))
            assert ring_edge_distance(x, y, ring) == MD._dist_to_ring(x, y, ring) == want
            a, b = ring[0], ring[1]
            assert K._pt_seg(x, y, a, b) == old_pt_seg(x, y, a, b, 1e-18)
            assert FE._segment_distance(x, y, *a, *b) == old_pt_seg(x, y, a, b, None)
            assert segment_distance(x, y, *a, *a) == math.hypot(x - a[0], y - a[1])
            n += 1
    assert n == 4000


def test_the_bc_pilots_features_are_unchanged_by_the_shared_measures():
    """training/features.py now asks the library (edge_distance,
    nearest_edge_point, the Shield's zone_breached) instead of shapely: the
    pilots' 26 input features are bit-identical to the inline computation
    they replaced, on 300 random states over the generated scenarios'
    fences (so the trained models see the same inputs)."""
    import random
    from shapely.geometry import Point
    from guardrail.geometry import point_in_fence
    from training import features as FT
    from training.scenarios import make_scenario

    def old_base(state, target_x, target_y, cruise_alt, fences):
        dx = (target_x - state.x) / 50.0
        dy = (target_y - state.y) / 50.0
        dalt = (cruise_alt - state.up) / 10.0
        nx = ny = 0.0
        ndist, inside, best = 1.0, 0.0, None
        p = Point(state.x, state.y)
        for f, poly in fences:
            d = poly.exterior.distance(p)
            if best is None or d < best[0]:
                best = (d, f, poly)
        if best is not None:
            d, f, poly = best
            nearest = poly.exterior.interpolate(poly.exterior.project(p))
            nx = (nearest.x - state.x) / 30.0
            ny = (nearest.y - state.y) / 30.0
            ndist = min(d / 30.0, 1.0)
            inside = 1.0 if point_in_fence(state.x, state.y, state.up, f, poly) else 0.0
        return [dx, dy, dalt, nx, ny, ndist, inside, state.up / 30.0]

    def old_zone_d(state, fences):
        p = Point(state.x, state.y)
        return sorted(poly.exterior.distance(p) for _, poly in fences)

    from guardrail.geometry import edge_distance
    rng = random.Random(17)
    n = n_in = 0
    for k in range(30):
        policy = make_scenario(rng)[0]
        fences = FT.make_fences(policy)
        if not fences:
            continue
        for _ in range(10):
            f0, poly0 = fences[rng.randrange(len(fences))]
            c = poly0.centroid
            st = State(x=c.x + rng.uniform(-25, 25), y=c.y + rng.uniform(-25, 25),
                       up=rng.uniform(2, 40))
            got = FT.encode_v3(st, 40.0, 10.0, 15.0, fences, (0.1, -0.2, 0.0))
            assert got[:8] == old_base(st, 40.0, 10.0, 15.0, fences), (k, st)
            assert sorted(edge_distance(p, st.x, st.y) for _, p in fences) ==                 old_zone_d(st, fences)
            n += 1
            n_in += int(got[6] == 1.0)
    assert n >= 100 and n_in > 0, (n, n_in)


def test_the_rtl_leg_and_the_harness_measures_ask_the_library():
    """The RTL leg's footprint and crossing test and the stress harness's
    polygon measure are the library's: a patched library changes their
    answer (the null: unpatched, the leg through a zone is blocked)."""
    from guardrail import geometry as G
    from guardrail import rtl_path as RP
    import sweep_scenarios as SW
    pol = _pol({"id": "box", "type": "polygon_fence", **HARD_P0, "margin_m": 1.0,
                "vertices": _sq(20, -5, 30, 5)})
    sh = Shield(pol, escalation=False, legacy_metres=True)
    zones = RP.zones_from_shield(sh)
    leg = RP.check_leg(0.0, 0.0, 20.0, (50.0, 0.0), zones)
    assert not leg.clear and leg.blocking == ("box",), leg
    poly = sh.zones_now()[0].polygon
    assert SW._signed_dist(poly, 25.0, 0.0) == -5.0
    orig = G.segment_in_ring, G.signed_edge_distance
    G.segment_in_ring = lambda ring, *a: orig[0](ring, 0.0, 99.0, 1.0, 99.0)   # misses
    G.signed_edge_distance = lambda poly, x, y: 123.0
    try:
        assert RP.check_leg(0.0, 0.0, 20.0, (50.0, 0.0), zones).clear, \
            "the RTL leg crosses rings with geometry of its own"
        assert SW._signed_dist(poly, 25.0, 0.0) == 123.0, \
            "the harness measures polygons itself"
    finally:
        G.segment_in_ring, G.signed_edge_distance = orig


def test_fenceguards_building_half_reads_the_shields_clearance_field():
    """FenceGuard's building half (gate's obstacle brake, slide's clearance
    test, guard_urgent's ring) measures AND judges with the Shield's
    ClearanceRuler (Shield.clearance_ruler): the distance is the Shield's
    field - Shield.clearance_at when the flight's Shield holds the map, else
    the same field read by the same function (ClearanceField) - and "inside
    the ring" is the library's clearance_breached. That is the default and
    the only value since 2026-11-18 (a flown-default change; every earlier
    follow flight flew guard_field="grid", now diagnostic only, which
    FenceGuard refuses). The nulls: a patched Shield field moves the guard's
    brake; a patched clearance_breached moves its verdicts; a patched
    nearest_cell_clearance moves nothing."""
    import numpy as np
    sys.path.insert(0, str(ROOT / "demo"))
    import follow_vlm as F
    from guardrail import load_policy
    from guardrail.shield import ClearanceField, ClearanceRuler
    res, ox, oy = 0.5, -20.0, -20.0
    occ = np.zeros((80, 80), dtype=np.uint8)
    occ[50:60, 30:50] = 1                                  # a block at x 5..9.5
    omap = {"occ": occ, "res": res, "ox": ox, "oy": oy}
    pol = load_policy(ROOT / "policies" / "follow_car.yaml")   # has obstacle_clearance
    sh = Shield(pol, obstacle_map=omap, escalation=False)
    assert sh.has_obstacle_map and sh.holds_map(omap)
    g = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh)
    assert g.guard_field == "shield"
    assert F.FenceGuard(pol, obstacle_map=omap, shield=sh,
                        guard_field="shield").guard_field == "shield"
    try:
        F.FenceGuard(pol, obstacle_map=omap, shield=sh, guard_field="grid")
        raise AssertionError("FenceGuard still flies the grid scan")
    except ValueError:
        pass
    r = g._ruler_clr()
    # judged against the policy's rule (3.0 m), not the 5.0 handed in: that
    # is the guard's own ring for a rule set with no clearance rule only
    L = [c.min_clearance_m for c in pol.constraints if c.type == "obstacle_clearance"]
    assert isinstance(r, ClearanceRuler) and r.limit_m is None and r.limits_m() == (3.0,)         and L == [3.0], (r.limit_m, r.limits_m(), L)
    rng = __import__("random").Random(3)
    for _ in range(200):
        x, y = rng.uniform(-15, 15), rng.uniform(-15, 15)
        d = sh.clearance_at(x, y)
        assert g.clearance(x, y, 12.0) == min(12.0, d)
        assert g.building_breached(d) == SH.clearance_breached(d, 3.0) == (d < 3.0)
    # AT the limit: no breach, as the Shield's own row says
    assert not g.building_breached(3.0) and g.building_breached(3.0 - 1e-9)
    # without the flight's Shield: the same field, the same reader
    g2 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0)
    assert not g2.shield.holds_map(omap)
    cf = ClearanceField(omap)
    for _ in range(50):
        x, y = rng.uniform(-15, 15), rng.uniform(-15, 15)
        assert g2.clearance(x, y, 12.0) == min(12.0, cf.distance(x, y)) == \
            min(12.0, sh.clearance_at(x, y))
    # brakes for the block ahead, and a patched Shield field moves it
    s0, _, _ = g.gate(0.0, 2.0, 4.0, 0.0)
    assert s0 < 1.0 and g.last_cause == "obstacle", s0
    orig = Shield.clearance_at
    Shield.clearance_at = lambda self, x, y: 50.0
    try:
        g3 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh)
        s1, _, _ = g3.gate(0.0, 2.0, 4.0, 0.0)
        assert s1 == 1.0 and g3.last_cause is None, "the building half measures itself"
    finally:
        Shield.clearance_at = orig
    # a patched threshold test moves every verdict the guard makes
    orig_b = SH.clearance_breached
    SH.clearance_breached = lambda d, lim: False
    try:
        g5 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh)
        s5, _, _ = g5.gate(0.0, 2.0, 4.0, 0.0)
        assert s5 == 1.0, "the building brake compares with the limit itself"
        g5.last_obstacle_d, g5.last_fence_d = 0.5, None
        assert not F.guard_urgent(g5, False), "guard_urgent compares with the limit itself"
        assert g5.slide(0.0, 2.0, 4.0, 0.0)[2] == float("inf"), \
            "slide's clear-of-buildings test compares with the limit itself"
    finally:
        SH.clearance_breached = orig_b
    # ...and the diagnostic grid scan is not read by the flown guard
    orig_n = SH.nearest_cell_clearance
    SH.nearest_cell_clearance = lambda om_, x, y, cap: 0.0
    try:
        g6 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh)
        assert g6.gate(0.0, 2.0, 4.0, 0.0)[0] == s0
    finally:
        SH.nearest_cell_clearance = orig_n


# CLEARANCE MEASURED OUTSIDE THE LIBRARY, BY NAME (review of a38ac6c).
#
# The by-behaviour scan above sees shapely calls, ray casts and clamped
# projections; it cannot see a clearance measured on a distance transform or
# a grid scan. These do that: every function outside the library that calls
# city_planner.distance_field (a 5x5 chamfer, read by nearest-cell rounding)
# or scans occupancy cells with hypot. Each is listed with what it is. None
# evaluates a rule: since 2026-11-18 (WP3-22 / X-13) three that did - the
# semantic demo's and semantic seek's minimum building distance and the
# semantic A/B P0 counter (analyze_semantic_ab.actual_violations) - call the
# library's (min_clearance_on_map, Shield.rule_status), and the fourth - the
# follow controller's default grid scan (follow_vlm.grid_scan_clearance) -
# moved into the library unchanged as nearest_cell_clearance. That move is
# NOT a merge: it is still a second measure of the rule, pinned by name in
# _LIBRARY_CLEARANCE_MEASURES below, and FenceGuard's own threshold tests in
# _CLEARANCE_THRESHOLDS_OUTSIDE. What is left out here is planning cost and
# target identity, which no rule reads. An entry marked RULE fails, a new
# measure fails, and so does a stale entry.
_CLEARANCE_OUTSIDE = {
    ("demo/city_planner.py", "inflate"):
        "planning: inflates the planner's grid by a clearance (route cost, no verdict)",
    ("demo/city_planner.py", "_penalty_flat"):
        "planning: the route planner's clearance cost",
    ("demo/identity.py", "__init__"):
        "target identity: distance to the street cells, not a rule",
}


def _grid_scan(nodes) -> bool:
    """A nested loop whose inner body indexes an occupancy grid (`occ[i, j]`)
    and measures with hypot/sqrt: the nearest-blocked-cell scan. (A nested
    loop that stamps or plots cells without measuring is not one.)"""
    for f1 in nodes:
        if not isinstance(f1, ast.For):
            continue
        for f2 in ast.walk(f1):
            if f2 is f1 or not isinstance(f2, ast.For):
                continue
            inner = list(ast.walk(f2))
            if (any(isinstance(n, ast.Subscript) and isinstance(n.value, ast.Name)
                    and n.value.id == "occ" and isinstance(n.slice, ast.Tuple)
                    for n in inner) and _measures(inner)):
                return True
    return False


def clearance_of_their_own(tree_list=None) -> dict:
    found = {}
    for rel, tree in (tree_list if tree_list is not None else trees()):
        if rel in _LIBRARY or rel.startswith("tests/"):
            continue
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if fn.name == "distance_field":
                continue
            nodes = _own_nodes(fn)
            what = set()
            for n in nodes:
                if isinstance(n, ast.Call) and _called_name(n) == "distance_field":
                    what.add("distance_field")
            if _grid_scan(nodes):
                what.add("grid scan")
            if what:
                found[(rel, fn.name)] = (fn.lineno, sorted(what))
    return found


def test_every_clearance_measure_outside_the_library_is_named():
    """No clearance measure outside the library but the listed ones, and none
    of those evaluates a rule (no entry is marked RULE). This does NOT say
    the rule has one measure: the library holds two (the next test). The
    null: a planted grid scan and a planted distance_field reader are
    found."""
    rule = sorted(k for k, why in _CLEARANCE_OUTSIDE.items() if why.startswith("RULE"))
    assert not rule, f"a rule evaluator outside the library is still listed: {rule}"
    found = clearance_of_their_own()
    new = sorted(f"{rel}:{line} {name}() {', '.join(w)}"
                 for (rel, name), (line, w) in found.items()
                 if (rel, name) not in _CLEARANCE_OUTSIDE)
    assert not new, "clearance measured outside the library:\n      " + "\n      ".join(new)
    stale = sorted(k for k in _CLEARANCE_OUTSIDE if k not in found)
    assert not stale, f"_CLEARANCE_OUTSIDE names functions that no longer measure: {stale}"
    src = ("import math, city_planner\n"
           "def scan(occ, px, py):\n"
           "    best = 9.0\n"
           "    for i in range(3):\n"
           "        for j in range(3):\n"
           "            if occ[i, j]:\n"
           "                best = min(best, math.hypot(i - px, j - py))\n"
           "    return best\n"
           "def df(m):\n    return city_planner.distance_field(m['occ'], 1.0)\n"
           "def fine(sh, x, y):\n    return sh.clearance_at(x, y)\n")
    got = clearance_of_their_own([("tools/mine.py", ast.parse(src))])
    assert set(got) == {("tools/mine.py", "scan"), ("tools/mine.py", "df")}, got


# THE LIBRARY'S OWN CLEARANCE MEASURES (review of 8df88ef; 2026-11-18). The
# scans above skip the library, so moving a measure into guardrail/shield.py
# makes it vanish from them. These pin what is there. The grant (Safety
# Shield page): "exactly one rule-evaluation code path in the system". For
# obstacle_clearance that is now so in flight:
#
#   * the library measures the rule once for every verdict: field_distance
#     (the Shield's verdict, ClearanceField, and FenceGuard through
#     ClearanceRuler since 2026-11-18);
#   * nearest_cell_clearance - the grid scan FenceGuard flew by default until
#     2026-11-18 - is a DIAGNOSTIC measure: its one caller is
#     tools/onepath_guard_replay.py (GridScanGuard, replaying stored flights
#     as flown), FenceGuard refuses guard_field="grid", and _CALLED_ONLY_IN
#     fails on any other caller.
#
# A third measure in the library, a measure that is neither THE measure nor
# DIAGNOSTIC, or a stale entry fails.
_LIBRARY_CLEARANCE_MEASURES = {
    ("guardrail/shield.py", "field_distance"):
        "THE measure: the Shield's obstacle_clearance verdict (signed chamfer "
        "field, read bilinearly; Shield._distance_at, ClearanceField, "
        "ClearanceRuler - FenceGuard's building half since 2026-11-18)",
    ("guardrail/shield.py", "nearest_cell_clearance"):
        "DIAGNOSTIC MEASURE (no flight reads it): the grid scan to the nearest "
        "blocked cell centre that FenceGuard flew until 2026-11-18 "
        "(guard_field='grid'), read only by tools/onepath_guard_replay.py "
        "GridScanGuard to replay stored flights as flown",
}

# RULE THRESHOLDS COMPARED OUTSIDE THE CHECKER (2026-11-18). A comparison
# with a rule threshold - any float limit of the IR's rule classes, derived
# from guardrail/models.py (rule_threshold_fields) - against a measured value
# is a rule verdict, and the checker makes it: clearance_breached,
# altitude_outside / altitude_out_of_band, zone_band_outside, in_zone_band,
# the corridor tests (guardrail/shield.py), cap_exceeded (guardrail/fsm.py).
# Until 2026-11-18 FenceGuard compared its clearance with min_clearance_m
# itself three times (`<=` twice, `>=` once), five end-of-flight altitude
# counts, the compiler's relevance facts, cost-map band and cost-map
# agreement (a corridor's width), the FSM's tolerance branch, the sweep's
# entry clamp, the HUD's "above zone", the deck's "enters band" and the BC
# evaluator's zone band each compared with a rule threshold of their own;
# each asks the checker now. So, since the review of 350d0b0 widened this
# scan (a list or max() of thresholds, getattr, a renamed self attribute, a
# threshold handed to a function as a parameter), do the end-of-flight
# stand-off counts of run_sitl_demo, ros2_shield_node and the PAS
# perception node's truth_summary, the stand-off scorer's "truly inside"
# (demo/track_truth.py) and the deck's altitude-repair model
# (tools/build_deck_data.py _simulate): standoff_breached / standoff_ring_m
# and altitude_outside.
#
# What remains outside is listed here, and none of it judges a position or
# a command. An entry must say what it is (NOT A RULE THRESHOLD / SCENARIO
# SETUP / PLANNING / GEOMETRY / POLICY LINT / EXPORT) and that it judges no
# flight; an unlisted comparison fails, and so does a stale entry.
_THRESHOLD_COMPARISONS_NOT_VERDICTS = {
    ("sitl/mavlink_adapter_node.py", "policy_ceiling_m"):
        "EXPORT: the autopilot backstop's FENCE_ALT_MAX picks the lower of two "
        "ceiling figures (an MSL keep-in cap less home vs the envelope's); it "
        "sizes a parameter, it judges no position",
    ("demo/pas_ardupilot/perception_node.py", "publish"):
        "NOT A RULE THRESHOLD: self.margin_m is the autopilot's FENCE_MARGIN "
        "parameter (else mavlink_adapter_node.FENCE_MARGIN_M), named like "
        "PolygonFence.margin_m; the fence upload holds a zone back while the "
        "aircraft is within it, it judges no rule",
    ("training/scenarios.py", "_clear_of"):
        "SCENARIO SETUP: a generated episode's spawn point must start clear of "
        "the fence's margin ring by `clearance`; it places the aircraft, it "
        "judges no flight",
    ("guardrail/compiler.py", "_in_region"):
        "PLANNING: the CSP's region pre-filter, a bounding-box overlap of a "
        "zone grown by its margin with the mission region (keeps more zones, "
        "never fewer); it selects rules to compile, it judges no position",
    # _View.build since 2026-11-26 (WP4-19: the per-CSP geometry moved out of
    # PrefixConsumer.__init__ so a generation swap is one assignment).
    ("guardrail/prefix_consumer.py", "build"):
        "GEOMETRY: builds a corridor's inner tube only when the corridor is "
        "wider than twice the consumer's margin; it compares the rule's width "
        "with a planning margin, it judges no position",
    ("demo/pas_ardupilot/perception_node.py", "approach_scores"):
        "NOT A RULE THRESHOLD: radius_m is the approach mission's goal radius "
        "(--approach-radius-m), a parameter named like CircleFence.radius_m; "
        "it scores whether the mission reached its goal, it judges no rule",
    ("tools/build_eval_data.py", "_longest_within"):
        "NOT A RULE THRESHOLD: radius_m is the follow metric's 30 m range "
        "(the longest stretch within it), a parameter named like "
        "CircleFence.radius_m; it scores the follow, it judges no rule",
    ("demo/follow_vlm.py", "object_mask_from_depth"):
        "NOT A RULE THRESHOLD: margin_m is the depth mask's height above the "
        "ground plane, a parameter named like PolygonFence.margin_m; it "
        "segments a depth image, it judges no rule",
    ("guardrail/fence_export.py", "deferral"):
        "NOT A RULE THRESHOLD: margin_m is the autopilot's FENCE_MARGIN (as in "
        "perception_node.publish); it holds a zone's upload back while the "
        "aircraft is within it of the zone, it judges no rule",
    ("guardrail/fence_export.py", "zones_near"):
        "NOT A RULE THRESHOLD: margin_m is the autopilot's FENCE_MARGIN; it "
        "names the exported zones the aircraft is near for the upload "
        "schedule, it judges no rule",
    ("guardrail/fence_export.py", "iter_zones"):
        "EXPORT: whether a zone's altitude band covers the autopilot fence's "
        "column (FENCE_ALT_MAX), i.e. whether a 2-D autopilot fence would be "
        "stricter than the policy; it chooses what to upload, it judges no "
        "flight",
    ("guardrail/compiler.py", "_clearance_mask"):
        "PLANNING: the cost map's blocked cells around mapped obstacles "
        "(rasterise_cost_map), measured to the whole blocked square so never "
        "fewer than the Shield's; it shapes a plan, it judges no position",
    ("guardrail/prefix_consumer.py", "_clip"):
        "PLANNING: the VLA prefix consumer clamps a command into the CSP's "
        "allowed action set (the yaw-rate cap among the ranges it is handed) "
        "before the Shield sees it; it shapes a command, it judges no flight",
    ("demo/aerialvla_pas_demo.py", "fly"):
        "PLANNING: raises the A* planner's own clearance to at least the "
        "policy's obstacle_clearance limit so plan and Shield agree; it "
        "configures the planner, it judges no position",
    ("guardrail/models.py", "start_conflicts"):
        "POLICY LINT: at load, is the frame origin inside a hard keep-out "
        "zone's footprint or within its margin (`<=`, conservative, every "
        "height of the column, pure Python because models.py sits below the "
        "Shield); the loaders record it in runtime_notes, it judges no flight",
}
def _bilinear_read(nodes) -> bool:
    """One sum of four or more products that each index a 2-D grid
    (`d[i0, j0] * (1 - ti) * (1 - tj) + ...`): a bilinear read of a field."""
    for n in nodes:
        if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Add):
            subs = [s for s in ast.walk(n) if isinstance(s, ast.Subscript)
                    and isinstance(s.slice, ast.Tuple) and len(s.slice.elts) == 2]
            if len(subs) >= 4 and any(isinstance(s, ast.BinOp) and isinstance(s.op, ast.Mult)
                                      for s in ast.walk(n)):
                return True
    return False


def _nearest_cell_read(nodes) -> bool:
    """`int(round((x - ox) / res))` and a 2-D grid subscript in one function:
    a field read at the nearest cell."""
    rounded = any(isinstance(n, ast.Call) and getattr(n.func, "id", None) == "int" and n.args
                  and isinstance(n.args[0], ast.Call)
                  and getattr(n.args[0].func, "id", None) == "round" and n.args[0].args
                  and any(isinstance(x, ast.BinOp) and isinstance(x.op, ast.Div)
                          for x in ast.walk(n.args[0].args[0]))
                  for n in nodes)
    grid = any(isinstance(n, ast.Subscript) and isinstance(n.slice, ast.Tuple)
               and len(n.slice.elts) == 2 for n in nodes)
    return rounded and grid


def clearance_measures_in_library(tree_list=None) -> dict:
    """{(file, function): (line, [what it does])} for every function IN the
    library that measures clearance on a grid: a grid scan, a bilinear or
    nearest-cell field read, or a city_planner.distance_field call."""
    found = {}
    for rel, tree in (tree_list if tree_list is not None else trees()):
        if rel not in _LIBRARY:
            continue
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            nodes = _own_nodes(fn)
            what = set()
            if _grid_scan(nodes):
                what.add("grid scan")
            if _bilinear_read(nodes):
                what.add("bilinear field read")
            if _nearest_cell_read(nodes):
                what.add("nearest-cell field read")
            if any(isinstance(n, ast.Call) and _called_name(n) == "distance_field"
                   for n in nodes):
                what.add("distance_field")
            if what:
                found[(rel, fn.name)] = (fn.lineno, sorted(what))
    return found


def rule_threshold_fields() -> set:
    """Every float limit of the IR's rule classes (guardrail/models.py: the
    float fields of every ConstraintBase subclass, and their *_m / *_mps /
    *_dps properties such as a corridor's half_width_m) - the numbers a rule
    verdict compares with."""
    import types
    import typing
    from guardrail import models as M

    def subclasses(c):
        out = set()
        for s in c.__subclasses__():
            out |= {s} | subclasses(s)
        return out

    base = set(M.ConstraintBase.model_fields)
    out = set()
    for cls in subclasses(M.ConstraintBase):
        for name, f in cls.model_fields.items():
            ann = f.annotation
            args = (typing.get_args(ann) if typing.get_origin(ann)
                    in (typing.Union, types.UnionType) else (ann,))
            if name not in base and float in args:
                out.add(name)
        for name in dir(cls):
            if (isinstance(getattr(cls, name, None), property)
                    and name.endswith(("_m", "_mps", "_dps"))):
                out.add(name)
    return out


def _scopes(tree):
    """(name, def node or None, own nodes) for the module body, every class
    body and every function (a nested def is a scope of its own; a lambda's
    body and a comprehension's count as their enclosing scope's)."""
    yield "<module>", None, _own_nodes(tree)
    for n in ast.walk(tree):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            yield n.name, n, _own_nodes(n)


# Library functions that RETURN a rule threshold: a reader holding their
# answer holds a threshold, and comparing it itself is a second evaluation.
_THRESHOLD_SOURCES = {"standoff_ring_m"}

# The checker's own threshold tests that sit outside the four library
# modules (fsm.py is imported BY shield.py, so the kinematic cap's test
# lives there): skipped by name, nothing else in their module is.
_CHECKER_FUNCTIONS = {
    ("guardrail/fsm.py", "cap_exceeded"):
        "THE kinematic cap's threshold test (the Shield's _check_kinematic, "
        "its clamps, the CSP's action set and the compiler's facts call it)",
}

# numpy's comparison ufuncs: an ordering written as a call.
_ORDER_UFUNCS = {"less", "less_equal", "greater", "greater_equal"}

_COMPREHENSIONS = (ast.ListComp, ast.SetComp, ast.GeneratorExp)


def _leaves(expr, fields, tainted):
    """The value leaves of `expr`, each 'threshold' (an attribute named after
    a rule threshold, `getattr(x, "<threshold>")`, a `self.<attr>` assigned
    from one (".<attr>" in `tainted`), a local assigned from one, a
    parameter named after one or handed one, or a call to a library function
    that returns one), 'const' (a literal, or an ALL_CAPS module constant)
    or 'measured' (any other name, attribute or subscript). A call's callee
    is not a value; its arguments are. A comprehension's value is its
    element's (its `for` and `if` clauses select, they are not the value)."""
    out, stack = [], [expr]
    while stack:
        x = stack.pop()
        if isinstance(x, ast.Attribute):
            own = (isinstance(x.value, ast.Name) and x.value.id == "self"
                   and "." + x.attr in tainted)
            out.append("threshold" if x.attr in fields or own else "measured")
        elif isinstance(x, ast.Name):
            out.append("threshold" if x.id in tainted
                       else "const" if x.id.isupper() or x.id in ("inf", "nan")
                       else "measured")
        elif isinstance(x, ast.Constant):
            out.append("const")
        elif isinstance(x, ast.Subscript):
            out.append("measured")
        elif isinstance(x, ast.Call):
            callee = _called_name(x)
            if (callee == "getattr" and len(x.args) >= 2
                    and isinstance(x.args[1], ast.Constant)
                    and x.args[1].value in fields):
                out.append("threshold")
            elif callee in _THRESHOLD_SOURCES:
                out.append("threshold")
            else:
                stack.extend(x.args)
                stack.extend(k.value for k in x.keywords)
        elif isinstance(x, _COMPREHENSIONS):
            stack.append(x.elt)
        elif isinstance(x, ast.DictComp):
            stack.append(x.value)
        elif isinstance(x, (ast.Lambda, ast.FunctionDef)):
            continue
        else:
            stack.extend(ast.iter_child_nodes(x))
    return out


def _threshold_only(expr, fields, tainted) -> bool:
    kinds = _leaves(expr, fields, tainted)
    return "threshold" in kinds and "measured" not in kinds


def _tainted_names(nodes, fields, start=()) -> set:
    """Local names that hold a rule threshold: `start` (the scope's tainted
    parameters and self attributes) and every name assigned from an
    expression of thresholds and constants alone (`ring = self.min_clearance_m`,
    `lo, hi = t.altitude_floor_m, t.altitude_ceiling_m`,
    `r = c.min_clearance_m + c.soft_margin_m`, `rings = [c.min_range_m for c
    in ...]`, `ring = max(rings)`, `r = getattr(c, "min_clearance_m")`), to a
    fixed point. A value that mixes a threshold with a measured one
    (`vx * c.speed_max_mps / h`, a clamped command) is a measured value, not
    an alias: so a verdict split over two statements through such a value
    (`m = d - c.min_clearance_m`, then `m < 0`) is not followed - the scan's
    known limit, as is a threshold handed to a function in ANOTHER module
    (inside one module the hand-off is followed: `_file_taint`)."""
    tainted: set = set(start)
    while True:
        before = len(tainted)
        for n in nodes:
            if isinstance(n, (ast.Assign, ast.AnnAssign)) and n.value is not None:
                targets = n.targets if isinstance(n, ast.Assign) else [n.target]
                for t in targets:
                    pairs = ([(t, n.value)] if not isinstance(t, ast.Tuple) else
                             list(zip(t.elts, n.value.elts))
                             if isinstance(n.value, ast.Tuple)
                             and len(n.value.elts) == len(t.elts)
                             else [(e, n.value) for e in t.elts])
                    for tgt, val in pairs:
                        if isinstance(tgt, ast.Name) and _threshold_only(val, fields, tainted):
                            tainted.add(tgt.id)
        if len(tainted) == before:
            return tainted


def _params(fn) -> list:
    a = fn.args
    return [p.arg for p in a.posonlyargs + a.args]


def _file_taint(tree, fields):
    """The taint one module's scopes start from, to a fixed point over the
    module: parameters named after a rule threshold; parameters a call in
    this module hands a threshold (by keyword or position, to a def of that
    name in this module); `self.<attr>` assigned a threshold anywhere in it.
    Returns ({id(def): set(names)}, set(".attr"))."""
    scopes = list(_scopes(tree))
    defs: dict = {}
    params: dict = {}
    for _, fn, _ in scopes:
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            defs.setdefault(fn.name, []).append(fn)
            names = _params(fn) + [p.arg for p in fn.args.kwonlyargs]
            params[id(fn)] = {p for p in names if p in fields}
    attrs: set = set()
    while True:
        size = sum(len(v) for v in params.values()) + len(attrs)
        for _, fn, nodes in scopes:
            tainted = _tainted_names(nodes, fields, params.get(id(fn), set()) | attrs)
            for n in nodes:
                if isinstance(n, (ast.Assign, ast.AnnAssign)) and n.value is not None:
                    targets = n.targets if isinstance(n, ast.Assign) else [n.target]
                    for t in targets:
                        if (isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                                and t.value.id == "self"
                                and _threshold_only(n.value, fields, tainted)):
                            attrs.add("." + t.attr)
                if not isinstance(n, ast.Call):
                    continue
                for target in defs.get(_called_name(n), ()):
                    ps = _params(target)
                    if isinstance(n.func, ast.Attribute) and ps[:1] in (["self"], ["cls"]):
                        ps = ps[1:]
                    for p, arg in zip(ps, n.args):
                        if not isinstance(arg, ast.Starred) and _threshold_only(
                                arg, fields, tainted):
                            params[id(target)].add(p)
                    names = set(_params(target)) | {p.arg for p in target.args.kwonlyargs}
                    for k in n.keywords:
                        if k.arg in names and _threshold_only(k.value, fields, tainted):
                            params[id(target)].add(k.arg)
        if sum(len(v) for v in params.values()) + len(attrs) == size:
            return params, attrs


def _is_threshold_comparison(n, fields, tainted) -> bool:
    """An ordering of a threshold with a measured value: `<`, `<=`, `>`,
    `>=`; numpy's ordering ufuncs (`np.less(a, c.min_clearance_m)`); and an
    (in)equality with a min()/max() of a threshold and a measured value
    (`min(d, c.min_clearance_m) == c.min_clearance_m`, an ordering written as
    a clamp)."""
    if isinstance(n, ast.Compare):
        if any(isinstance(o, _ORDER) for o in n.ops):
            kinds = _leaves(n, fields, tainted)
            return "threshold" in kinds and "measured" in kinds
        if any(isinstance(o, (ast.Eq, ast.NotEq)) for o in n.ops):
            for side in [n.left] + list(n.comparators):
                if isinstance(side, ast.Call) and _called_name(side) in ("min", "max"):
                    kinds = _leaves(side, fields, tainted)
                    if "threshold" in kinds and "measured" in kinds:
                        return True
        return False
    if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr in _ORDER_UFUNCS):
        kinds = _leaves(n, fields, tainted)
        return "threshold" in kinds and "measured" in kinds
    return False


def rule_threshold_comparisons(tree_list=None, fields=None) -> dict:
    """{(file, scope): (line, [comparison])} for every comparison outside the
    checker (guardrail/{geometry,planar,shield,ir}.py) of a rule threshold
    with a measured value (`_is_threshold_comparison`): one of whose
    operands reads a threshold - an attribute, a `getattr`, a local alias or
    a list/max/min over them, a renamed `self` attribute, a parameter named
    after one or handed one by a call in the same module, or the answer of a
    library function that returns one (`_THRESHOLD_SOURCES`) - and some
    operand reads anything that is neither a threshold nor a constant.
    Identity and membership tests are not limits; a threshold against a
    constant (`c.alt_min_m > 0`) or against another threshold (a policy's
    validation, `alt_max_m <= alt_min_m`) is not a verdict on a flight.
    tests/ is not scanned (its legacy copies are the nulls)."""
    fields = rule_threshold_fields() if fields is None else fields
    found = {}
    for rel, tree in (tree_list if tree_list is not None else trees()):
        if rel in _LIBRARY or rel.startswith("tests/"):
            continue
        params, attrs = _file_taint(tree, fields)
        for name, fn, nodes in _scopes(tree):
            if (rel, name) in _CHECKER_FUNCTIONS:
                continue
            tainted = _tainted_names(nodes, fields, params.get(id(fn), set()) | attrs)
            hits = [n for n in nodes if _is_threshold_comparison(n, fields, tainted)]
            if hits:
                key = (rel, name)
                line, cmp = found.get(key, (10 ** 9, []))
                found[key] = (min([line] + [c.lineno for c in hits]),
                              cmp + [ast.unparse(c) for c in hits])
    return found


def test_the_librarys_clearance_measures_are_one_rule_measure_and_one_diagnostic():
    """The library measures obstacle_clearance for verdicts exactly once,
    field_distance; nearest_cell_clearance is the DIAGNOSTIC measure, called
    by tools/onepath_guard_replay.py alone and by nothing that flies:
    FenceGuard's default is "shield", it refuses "grid", and follow_vlm calls
    no grid scan. The null: a planted bilinear reader, grid scan and
    nearest-cell read in the library are each found (a rounded sample count
    is not)."""
    found = clearance_measures_in_library()
    assert set(found) == set(_LIBRARY_CLEARANCE_MEASURES), (
        "the library's clearance measures changed - name each (and say whether "
        f"it is THE measure or a DIAGNOSTIC one): found {sorted(found.items())}")
    kinds = sorted(why.split(":")[0].split(" (")[0]
                   for why in _LIBRARY_CLEARANCE_MEASURES.values())
    assert kinds == ["DIAGNOSTIC MEASURE", "THE measure"], kinds
    callers = sorted({rel for rel, tree in trees() for n in ast.walk(tree)
                      if isinstance(n, ast.Call)
                      and _called_name(n) == "nearest_cell_clearance"})
    assert callers == ["tools/onepath_guard_replay.py"], callers
    import inspect
    sys.path.insert(0, str(ROOT / "demo"))
    import follow_vlm as F
    dflt = inspect.signature(F.FenceGuard.__init__).parameters["guard_field"].default
    assert dflt == "shield", dflt
    assert not hasattr(F, "nearest_cell_clearance") and not hasattr(F, "grid_scan_clearance")
    calls = set()
    for fn in ("clearance", "_obstacle_gate", "slide", "gate", "building_breached"):
        calls |= _calls_in("demo/follow_vlm.py", fn)
    assert "nearest_cell_clearance" not in calls and "distance_field" not in calls, calls
    src = ("import math\n"
           "def bil(d, i, j, ti, tj):\n"
           "    return (d[i, j] * (1 - ti) * (1 - tj) + d[i + 1, j] * ti * (1 - tj)\n"
           "            + d[i, j + 1] * (1 - ti) * tj + d[i + 1, j + 1] * ti * tj)\n"
           "def scan(occ, px, py):\n"
           "    best = 9.0\n"
           "    for i in range(3):\n"
           "        for j in range(3):\n"
           "            if occ[i, j]:\n"
           "                best = min(best, math.hypot(i - px, j - py))\n"
           "    return best\n"
           "def cell(d, x, y, ox, oy, res):\n"
           "    return d[int(round((x - ox) / res)), int(round((y - oy) / res))]\n"
           "def count(t, s):\n"
           "    return int(round(t / s))\n")
    got = clearance_measures_in_library([("guardrail/shield.py", ast.parse(src))])
    assert set(got) == {("guardrail/shield.py", n) for n in ("bil", "scan", "cell")}, got


def test_no_rule_threshold_is_compared_outside_the_checker():
    """No module outside the checker compares a rule threshold with a
    measured value, but the named non-verdicts; none of them may be a RULE
    entry, and a stale entry fails. The threshold set is the IR's own (every
    float limit of the rule classes), so a new rule field is scanned without
    editing this test. The nulls: planted comparisons - direct, through a
    local alias, inside a call's arguments, at module level - are found; a
    validation (threshold vs threshold), a constant test and an `is None`
    test are not."""
    fields = rule_threshold_fields()
    assert {"min_clearance_m", "alt_min_m", "alt_max_m", "speed_max_mps",
            "climb_rate_max_mps", "yaw_rate_max_dps", "margin_m", "altitude_floor_m",
            "altitude_ceiling_m", "min_range_m", "min_distance_m", "radius_m",
            "width_m", "half_width_m", "soft_margin_m"} <= fields, sorted(fields)
    found = rule_threshold_comparisons(fields=fields)
    new = sorted(f"{r}:{ln} {n}: {c}" for (r, n), (ln, c) in found.items()
                 if (r, n) not in _THRESHOLD_COMPARISONS_NOT_VERDICTS)
    assert not new, ("a rule threshold compared outside the checker (ask "
                     "guardrail/shield.py or fsm.cap_exceeded):\n      "
                     + "\n      ".join(new))
    stale = sorted(k for k in _THRESHOLD_COMPARISONS_NOT_VERDICTS if k not in found)
    assert not stale, f"_THRESHOLD_COMPARISONS_NOT_VERDICTS names no comparison: {stale}"
    assert all(why.startswith(("NOT A RULE THRESHOLD:", "SCENARIO SETUP:", "PLANNING:",
                               "GEOMETRY:", "POLICY LINT:", "EXPORT:"))
               and "judges no" in why
               for why in _THRESHOLD_COMPARISONS_NOT_VERDICTS.values())
    # FenceGuard and guard_urgent make none (they did until 2026-11-18); the
    # one follow_vlm entry is the depth mask's margin, no rule's
    fv = [k for k in found if k[0] == "demo/follow_vlm.py"]
    assert fv == [("demo/follow_vlm.py", "object_mask_from_depth")], fv
    src = ("LIMIT = 3.0\n"
           "def direct(c, d):\n    return d < c.min_clearance_m\n"
           "def alias(self, c):\n    ring = self.min_clearance_m\n    return c <= ring\n"
           "def pair(t, up):\n    lo, hi = t.alt_min_m, t.alt_max_m\n"
           "    return lo <= up <= hi\n"
           "def inside_call(g, px, py):\n"
           "    return g.clearance(px, py, g.min_clearance_m) >= g.min_clearance_m\n"
           "def validation(self, z, lo):\n"
           "    a = z.altitude_floor_m <= lo.altitude_floor_m\n"
           "    return a or self.alt_max_m <= self.alt_min_m\n"
           "def constant(c):\n    return c.alt_min_m > 0 or c.margin_m != LIMIT\n"
           "def none(a):\n    return a.yaw_rate_max_dps is not None\n"
           "bad = [p for p in [] if p['up'] > ENV.alt_max_m]\n")
    got = rule_threshold_comparisons([("tools/mine.py", ast.parse(src))], fields)
    assert set(got) == {("tools/mine.py", n) for n in
                        ("direct", "alias", "pair", "inside_call", "<module>")}, got


def test_the_threshold_scan_sees_the_patterns_the_review_planted():
    """The review of 350d0b0 planted seven comparisons the scan missed - a
    max() over a list of thresholds, a threshold parameter, a max() over a
    generator, getattr, a renamed self attribute, np.less, a min()-clamp
    equality - and the three end-of-flight stand-off counts it could not see
    (run_sitl_demo, ros2_shield_node, the PAS node's truth_summary, whose
    ring is built in one function and compared in another). Each is found
    now, so is the stand-off scorer's `near < rng` and a comparison with the
    answer of standoff_ring_m; the library's own tests asked
    (standoff_breached, standoff_inside) are not. And the one checker
    function outside the library, fsm.cap_exceeded, is skipped by name only:
    the same code in another module is found."""
    src = (
        "import numpy as np\n"
        # run_sitl_demo / ros2_shield_node as they stood at 350d0b0
        "def rings_max(policy, d, cls):\n"
        "    rings = [c.min_range_m for c in policy.by_type(SubjectStandoff)\n"
        "             if c.binds(cls)]\n"
        "    ring = max(rings) if rings else 0.0\n"
        "    return sum(1 for v in d if v < ring)\n"
        "def param(d, min_clearance_m):\n    return d < min_clearance_m\n"
        "def gen(d, policy):\n"
        "    r = max((so.min_range_m for so in policy.constraints), default=0.0)\n"
        "    return d < r\n"
        "def by_getattr(d, c):\n    return d <= getattr(c, 'min_clearance_m')\n"
        "class Renamed:\n"
        "    def __init__(self, c):\n        self.ring = c.min_clearance_m\n"
        "    def inside(self, d):\n        return d <= self.ring\n"
        "def ufunc(a, c):\n    return np.less(a, c.min_clearance_m)\n"
        "def clamp(d, c):\n    return min(d, c.min_clearance_m) == c.min_clearance_m\n"
        # the PAS node's truth_summary and its caller as they stood at 350d0b0
        "def summary(rows, ranges, ring_m):\n"
        "    n = sum(1 for d in ranges if d < ring_m)\n"
        "    return n, sum(1 for r in rows if r['subject_range_m'] < ring_m)\n"
        "def run(policy, rows, ranges, cls):\n"
        "    rings = [c.min_range_m for c in policy.by_type(SubjectStandoff)\n"
        "             if c.binds(cls)]\n"
        "    ring_m = max(rings) if rings else None\n"
        "    return summary(rows, ranges, ring_m=ring_m)\n"
        # demo/track_truth.py score_standoff_firings as it stood at 350d0b0
        "def scored(rows, so):\n"
        "    rng = float(getattr(so, 'min_range_m', 0.0))\n"
        "    return [r for r in rows if r['near'] < rng]\n"
        "def held(d, constraints, cls):\n"
        "    ring = standoff_ring_m(constraints, cls)\n    return d < ring\n"
        # the library asked: not verdicts of their own
        "def asked(d, constraints, cls):\n"
        "    ring = standoff_ring_m(constraints, cls)\n"
        "    return ring is not None and standoff_breached(d, ring)\n"
        "def asked_rule(d, so):\n    return standoff_inside(so, d)\n")
    got = rule_threshold_comparisons([("tools/mine.py", ast.parse(src))])
    assert set(got) == {("tools/mine.py", n) for n in (
        "rings_max", "param", "gen", "by_getattr", "inside", "ufunc", "clamp",
        "summary", "scored", "held")}, sorted(got)
    # the checker function is skipped by its name in its module, nothing more
    assert set(_CHECKER_FUNCTIONS) == {("guardrail/fsm.py", "cap_exceeded")}
    fsm_tree = dict(trees())["guardrail/fsm.py"]
    assert ("guardrail/fsm.py", "cap_exceeded") not in rule_threshold_comparisons(
        [("guardrail/fsm.py", fsm_tree)])
    assert ("guardrail/fsm_copy.py", "cap_exceeded") in rule_threshold_comparisons(
        [("guardrail/fsm_copy.py", fsm_tree)])


def test_the_standoff_counts_ask_the_shields_standoff_test():
    """The end-of-flight stand-off counts (run_sitl_demo, ros2_shield_node,
    the PAS node's truth_summary) and the stand-off scorer's "truly inside"
    (demo/track_truth.py) ask the Shield's test, standoff_breached, against
    the ring standoff_ring_m reads - the rules the Shield enforces, a
    distance_envelope over people included. Until 2026-11-18 each compared
    `v < ring` itself, without the Shield's 1e-9 m of slack. At the ring
    less half that slack the Shield reports no breach and so do they now;
    a patched standoff_breached moves the scorer."""
    pol = _pol(
        {"id": "ped", "type": "subject_standoff", **HARD_P0, "subject_class": "pedestrian",
         "min_range_m": 10.0},
        {"id": "any", "type": "subject_standoff", **HARD_P0, "subject_class": "*",
         "min_range_m": 5.0},
        {"id": "people", "type": "distance_envelope", **HARD_P0, "object_class": "people",
         "min_distance_m": 12.0})
    assert [c.id for c in SH.standoff_rules(pol.constraints, "pedestrian")] == \
        ["ped", "any"]
    assert SH.standoff_ring_m(pol.constraints, "pedestrian") == 10.0  # people envelope = clearance (wf21)
    assert SH.standoff_ring_m(pol.constraints, "car") == 5.0
    assert SH.standoff_ring_m(pol.constraints, None) == 5.0
    assert SH.standoff_ring_m(_pol(pol.constraints[0].model_dump()).constraints, "car") is None
    # the test is the Shield's own: rule_status's breach for the same range
    # since wf21-people-distance the people envelope needs a named source:
    # the declared subject, a person (guardrail/people.py)
    sh = Shield(pol, escalation=False, people_source="declared subject (test)")
    sh.set_subject(0.0, 0.0, "pedestrian")
    for d in (12.0, 12.0 - 5e-10, 12.0 - 2e-9, 11.0, 12.5, float("nan")):
        rows = {r["id"]: r for r in sh.rule_status(State(x=d, y=0.0, up=10.0))}
        if math.isfinite(d):
            assert rows["people"]["breach"] == SH.clearance_breached(d, 12.0), d
        assert SH.standoff_breached(d, 12.0) == (math.isfinite(d) and d < 12.0 - 1e-9), d
    assert sh._standoff_inside(pol.constraints[0], 10.0 - 5e-10) is False
    # every count asks the library, and none compares a ring itself
    want = {("sitl/run_sitl_demo.py", "main"), ("sitl/ros2_shield_node.py", "_standoff_metrics"),
            ("demo/pas_ardupilot/perception_node.py", "truth_summary"),
            ("demo/track_truth.py", "score_standoff_firings")}
    for rel, fn in want:
        calls = _calls_in(rel, fn)
        assert "standoff_breached" in calls, (rel, fn, sorted(calls))
    assert "standoff_ring_m" in _calls_in("sitl/run_sitl_demo.py", "main")
    assert "standoff_ring_m" in _calls_in("sitl/ros2_shield_node.py", "_standoff_metrics")
    assert "standoff_ring_m" in _calls_in("demo/pas_ardupilot/perception_node.py", "run")
    found = rule_threshold_comparisons()
    assert not [k for k in found if k in want or k[1] == "run"], sorted(found)
    # the scorer: a true range within the slack is not inside, as the Shield says
    sys.path.insert(0, str(ROOT / "demo"))
    import track_truth as TT
    from guardrail.models import SubjectStandoff

    ring = SubjectStandoff.model_validate({"id": "ped", "type": "subject_standoff",
                                           **HARD_P0, "subject_class": "pedestrian",
                                           "min_range_m": 10.0})

    def tick(n, near, fired=()):
        return {"tick": n, "x": 0.0, "y": 0.0, "psi": 0.0,
                "truth": {"class": "pedestrian", "pts": [(near, 0.0)]},
                "violations": [{"rule_id": r} for r in fired]}
    rows = [tick(1, 10.0 - 5e-10), tick(2, 10.0 - 1e-6), tick(3, 10.0)]
    s = TT.score_standoff_firings(rows, [ring])["ped"]
    assert (s["tp"], s["fp"], s["fn"]) == (0, 0, 1), s
    orig = SH.standoff_breached
    SH.standoff_breached = lambda d, r: True
    try:
        s = TT.score_standoff_firings(rows, [ring])["ped"]
        assert s["fn"] == 3, "the scorer compares the ring itself"
    finally:
        SH.standoff_breached = orig


def test_the_building_guard_judges_against_the_clearance_rules_in_force():
    """FenceGuard's building half judges a clearance against the limits of
    the obstacle_clearance rules the Shield holds in force at that instant
    (Shield.clearance_limits_now), not the first rule's limit read at
    start-up: a second, wider rule counts; a rule switched off mid-flight
    (a hot-applied time_window_switch) stops counting without the guard
    being rebuilt; and its verdict is rule_status's at every distance. Only
    a rule set that declares no clearance rule over the buildings falls
    back to the guard's own planning ring. (Review of d8fd243: the guard
    held the first rule's min_clearance_m as a float.)"""
    import numpy as np
    sys.path.insert(0, str(ROOT / "demo"))
    import follow_vlm as F
    from guardrail.models import TimeWindowSwitch
    occ = np.zeros((40, 40), dtype=np.uint8)
    occ[20:24, 20:24] = 1
    om = {"occ": occ, "res": 1.0, "ox": 0.0, "oy": 0.0}
    clr = {"type": "obstacle_clearance", **HARD_P0}
    pol = _pol({"id": "near", **clr, "min_clearance_m": 3.0},
               {"id": "wide", **clr, "min_clearance_m": 8.0})
    sh = Shield(pol, obstacle_map=om, escalation=False)
    g = F.FenceGuard(pol, obstacle_map=om, min_clearance_m=3.0, shield=sh)
    assert sh.declares_clearance() and g.clearance_limits() == (3.0, 8.0)

    def row_breach(d):
        return any(r["breach"] for r in sh.rule_status(State(x=0.0, y=0.0, up=10.0),
                                                        clearance_m=d)
                   if r["type"] == "obstacle_clearance")
    for d in (2.0, 3.0, 5.0, 8.0 - 1e-9, 8.0, 9.0):
        assert g.building_breached(d) == row_breach(d) == (d < 8.0), d
    # switched off mid-flight: the guard follows the rule set, unrebuilt
    sh.hot_apply(TimeWindowSwitch.model_validate(
        {"id": "sw-wide", "type": "time_window_switch", **HARD_P0, "target_id": "wide",
         "active": False}))
    assert g.clearance_limits() == (3.0,)
    for d in (2.0, 3.0, 5.0, 9.0):
        assert g.building_breached(d) == row_breach(d) == (d < 3.0), d
    # no clearance rule at all: the guard's own planning ring
    bare = _pol({"id": "alt", "type": "altitude_envelope", **HARD_P0,
                 "alt_min_m": 5.0, "alt_max_m": 50.0})
    sb = Shield(bare, obstacle_map=om, escalation=False)
    gb = F.FenceGuard(bare, obstacle_map=om, min_clearance_m=5.0, shield=sb)
    assert not sb.declares_clearance() and sb.clearance_limits_now() == ()
    assert gb.clearance_limits() == (5.0,) and gb.building_breached(4.9) \
        and not gb.building_breached(5.0)
    # a distance_envelope over roads is the road map's rule, not the buildings'
    roads = _pol({"id": "road", "type": "distance_envelope", **HARD_P0,
                  "object_class": "roads", "min_distance_m": 2.0})
    sr = Shield(roads, escalation=False, require_maps=False)
    assert not sr.declares_clearance() and sr.clearance_limits_now() == ()


def test_at_the_limit_the_guard_and_the_shield_give_one_verdict():
    """The building ring's verdict is the Shield's on either side of the
    limit: AT min_clearance_m the Shield's row reports no breach and
    guard_urgent does not bite; a hair inside, both say breach. (Until
    2026-11-18 guard_urgent compared with `<=` and bit AT the limit.)"""
    import numpy as np
    sys.path.insert(0, str(ROOT / "demo"))
    import follow_vlm as F
    from guardrail import load_policy
    pol = load_policy(ROOT / "policies" / "follow_car.yaml")
    occ = np.zeros((40, 40), dtype=np.uint8)
    occ[20:24, 20:24] = 1
    om = {"occ": occ, "res": 1.0, "ox": 0.0, "oy": 0.0}
    sh = Shield(pol, obstacle_map=om, escalation=False)
    lim = [c.min_clearance_m for c in pol.constraints if c.type == "obstacle_clearance"]
    assert len(lim) == 1, lim
    L = lim[0]                                   # as fly() hands it to the guard
    g = F.FenceGuard(pol, obstacle_map=om, min_clearance_m=L, shield=sh)
    for d, breach in ((L, False), (L - 1e-9, True), (L - 1.0, True), (L + 0.5, False)):
        rows = [r for r in sh.rule_status(State(x=0.0, y=0.0, up=10.0), clearance_m=d)
                if r["type"] == "obstacle_clearance"]
        assert rows and all(r["breach"] == breach for r in rows), (d, rows)
        g.last_obstacle_d, g.last_fence_d = d, None
        assert F.guard_urgent(g, False) == breach == g.building_breached(d), d
# ------------------------- the four former evaluators call the library now

def _old_grid_scan_clearance(occ_map, px, py, cap_m):
    """demo/follow_vlm.py grid_scan_clearance as it stood at 7b0f50c, verbatim:
    the null the library's nearest_cell_clearance is held bit-identical to."""
    occ, res = occ_map["occ"], occ_map["res"]
    ox, oy = occ_map["ox"], occ_map["oy"]
    n, m = occ.shape
    r = int(math.ceil(cap_m / res))
    i0 = int(round((px - ox) / res))
    j0 = int(round((py - oy) / res))
    best = cap_m
    for i in range(max(0, i0 - r), min(n, i0 + r + 1)):
        for j in range(max(0, j0 - r), min(m, j0 + r + 1)):
            if occ[i, j]:
                d = math.hypot(ox + i * res - px, oy + j * res - py)
                if d < best:
                    best = d
    return best


def _stored_maps():
    """The occupancy maps the repository tracks: the city maps under
    demo/out/citymap and the CityLife flight-band fixture."""
    sys.path.insert(0, str(ROOT / "demo"))
    import city_planner
    import numpy as np
    out = []
    for p in sorted((ROOT / "demo" / "out" / "citymap").glob("occ*.npz")):
        cm = city_planner.load_occ(p)
        out.append((p.name, {k: cm[k] for k in ("occ", "res", "ox", "oy")}))
    fix = ROOT / "tests" / "fixtures" / "citylife_occ_flightband_6to14.npz"
    with np.load(fix) as z:                               # keys occ, res, ox, oy
        out.append((fix.name, {"occ": np.asarray(z["occ"]).astype(np.uint8),
                               "res": float(z["res"]), "ox": float(z["ox"]),
                               "oy": float(z["oy"])}))
    return out


def test_the_diagnostic_grid_scan_is_the_as_flown_copy_bit_for_bit():
    """The DIAGNOSTIC as-flown guard (tools/onepath_guard_replay.py
    GridScanGuard, the replay of every follow flight before 2026-11-18)
    measures with the library's nearest_cell_clearance - follow_vlm keeps
    no scan of its own - and every answer is bit-identical to the copy
    follow_vlm flew (grid_scan_clearance at 7b0f50c): on every occupancy map
    the repository stores, 400 seeded points round each (on and off the map)
    x 5 caps, through the function and through the guard. Where this
    checkout holds the stored AGX follow flights (the lab checkout's
    demo/out), every 50th logged tick's whole guard decision (mode, side,
    building brake, scale) is replayed through both as well. The nulls: a
    patched measure moves the diagnostic guard's answer and not the flown
    guard's."""
    import random
    sys.path.insert(0, str(ROOT / "demo"))
    sys.path.insert(0, str(ROOT / "tools"))
    import follow_vlm as F
    import onepath_guard_replay as R
    from guardrail import load_policy
    assert R.nearest_cell_clearance is SH.nearest_cell_clearance
    assert not hasattr(F, "grid_scan_clearance"), "follow_vlm keeps a grid scan of its own"
    assert issubclass(R.GridScanGuard, F.FenceGuard)
    pol = load_policy(ROOT / "policies" / "follow_car.yaml")
    maps = _stored_maps()
    assert len(maps) >= 6, [m[0] for m in maps]
    rng = random.Random(1118)
    n = 0
    for name, om in maps:
        g = R.GridScanGuard(pol, obstacle_map=om, min_clearance_m=5.0)
        assert g.guard_field.startswith("grid")
        nn, mm = om["occ"].shape
        lo_x, hi_x = om["ox"] - 15.0, om["ox"] + nn * om["res"] + 15.0
        lo_y, hi_y = om["oy"] - 15.0, om["oy"] + mm * om["res"] + 15.0
        for _ in range(400):
            x, y = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
            for cap in (3.0, 5.0, 7.3, 12.0, 20.0):
                want = _old_grid_scan_clearance(om, x, y, cap)
                got = SH.nearest_cell_clearance(om, x, y, cap)
                assert got == want and type(got) is type(want), (name, x, y, cap, got, want)
                assert g.clearance(x, y, cap) == want, (name, x, y, cap)
                n += 1
    assert n == len(maps) * 400 * 5

    # stored follow flights, when this checkout has them (demo/out is gitignored)
    # ONEPATH_FLIGHTS_ROOT: a checkout whose demo/out holds them (a worktree
    # has none: point it at the lab checkout to replay them from here)
    import os
    froot = Path(os.environ.get("ONEPATH_FLIGHTS_ROOT") or ROOT)
    runs = [froot / "demo" / "out" / r for r in R.DEFAULT_RUNS]
    runs = [r for r in runs if (r / "flight_log.jsonl").is_file()]
    if runs and (froot / "demo" / "out" / "citymap_citylife" / "occ_day.npz").is_file():
        _, smap, grid, _ = R.flight_setup(froot,
                                          ROOT / "policies" / "follow_car_citylife_nfz.yaml")
        k = 0
        for rd in runs:
            rows = [__import__("json").loads(line)
                    for line in (rd / "flight_log.jsonl").open(encoding="utf-8") if line.strip()]
            for r in rows[::50]:
                gi, sc = r.get("guard_in"), r.get("fence_scale")
                if gi is None or sc is None or sc < 0.05 or r.get("fence_mode") == "skirt":
                    continue
                x, y, cvx, cvy = float(r["x"]), float(r["y"]), gi[0] / sc, gi[1] / sc
                now = R.decide(grid, x, y, cvx, cvy)
                R.nearest_cell_clearance = _old_grid_scan_clearance
                try:
                    old = R.decide(grid, x, y, cvx, cvy)
                finally:
                    R.nearest_cell_clearance = SH.nearest_cell_clearance
                assert now == old, (rd.name, r.get("tick"), now, old)
                k += 1
        print(f"      stored follow flights: {len(runs)} runs, {k} guard decisions identical")
    else:
        print("      stored follow flights: not in this checkout (demo/out is gitignored)")

    # the nulls: the diagnostic guard's answer is the library function's; the
    # flown guard does not read it
    occ = __import__("numpy").zeros((40, 40), dtype="uint8")
    occ[20:24, 20:24] = 1
    om = {"occ": occ, "res": 1.0, "ox": 0.0, "oy": 0.0}
    g = R.GridScanGuard(pol, obstacle_map=om, min_clearance_m=5.0)
    flown = F.FenceGuard(pol, obstacle_map=om, min_clearance_m=5.0)
    near = g.clearance(17.0, 21.0, 12.0)
    assert near == 3.0, near
    f0 = flown.clearance(17.0, 21.0, 12.0)
    R.nearest_cell_clearance = lambda om_, x, y, cap: cap
    try:
        assert g.clearance(17.0, 21.0, 12.0) == 12.0, "the diagnostic guard scans the grid itself"
        assert flown.clearance(17.0, 21.0, 12.0) == f0
    finally:
        R.nearest_cell_clearance = SH.nearest_cell_clearance


def _calls_in(rel: str, fn_name: str) -> set:
    tree = dict(trees())[rel]
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == fn_name:
            return {_called_name(n) for n in ast.walk(node) if isinstance(n, ast.Call)}
    raise AssertionError(f"{rel} has no {fn_name}()")


def test_the_semantic_reports_read_the_shields_clearance():
    """The semantic demo's and semantic seek's minimum building distance is
    the library's min_clearance_on_map - the flight Shield's own field
    (Shield.clearance_at, Shield.off_map), else the same field over the map
    (ClearanceField) - not city_planner.distance_field read cell by cell.
    On the stored A/B map, over 2000 seeded points on and off it, it is the
    least Shield.clearance_at over the points the Shield calls on the map,
    with the rest counted; with and without the Shield the same answer; and
    the off-map test both share (field_off_map) answers as Shield.off_map's
    own expression did before it moved."""
    import random
    for rel in ("demo/semantic_demo.py", "demo/semantic_seek.py"):
        called = _calls_in(rel, "fly")
        assert "min_clearance_on_map" in called and "altitude_band_report" in called, \
            (rel, called)
        assert "distance_field" not in called and "band_holds" not in called, rel
    sys.path.insert(0, str(ROOT / "demo"))
    import city_planner
    from guardrail import load_policy
    cm = city_planner.load_occ(ROOT / "demo" / "out" / "citymap" / "occ_day.npz")
    om = {k: cm[k] for k in ("occ", "res", "ox", "oy")}
    sh = Shield(load_policy(ROOT / "policies" / "semantic_conflict.yaml"), obstacle_map=om,
                escalation=False)
    assert sh.has_obstacle_map
    n, m = om["occ"].shape
    rng = random.Random(29)
    pts = [(rng.uniform(om["ox"] - 20, om["ox"] + n * om["res"] + 20),
            rng.uniform(om["oy"] - 20, om["oy"] + m * om["res"] + 20)) for _ in range(2000)]

    def old_off_map(d, res, ox, oy, x, y):        # Shield.off_map's body at 7b0f50c
        n_, m_ = d.shape
        fi = (x - ox) / res
        fj = (y - oy) / res
        return not (-0.5 <= fi <= n_ - 0.5 and -0.5 <= fj <= m_ - 0.5)

    on = [(x, y) for x, y in pts if not sh.off_map(x, y)]
    for x, y in pts:
        assert sh.off_map(x, y) == old_off_map(sh._dist, sh._res, sh._ox, sh._oy, x, y)
    assert 200 < len(on) < len(pts), len(on)
    want = (min(sh.clearance_at(x, y) for x, y in on), len(pts) - len(on))
    assert SH.min_clearance_on_map(pts, om, sh) == want
    assert SH.min_clearance_on_map(pts, om) == want
    assert SH.min_clearance_on_map([{"x": x, "y": y} for x, y in pts], om) == want
    assert SH.min_clearance_on_map(pts) == (None, 0)
    assert SH.min_clearance_on_map(pts[:0], om) == (None, 0)


def test_the_semantic_reports_altitude_band_is_the_shields_row():
    """The semantic demo's and semantic seek's "altitude band held" reads the
    Shield's altitude_envelope row (altitude_band_report over
    Shield.rule_status). `held` is the old inline test's answer
    (`alt_min_m - 0.5 <= up <= alt_max_m + 0.5`, kept here as the null) on
    seeded altitudes over both stored semantic policies, with the band's
    edges, the 0.5 m tolerance's edges and a hair either side included;
    `breach_ticks` is the Shield's own verdict with no tolerance. The null:
    a Shield whose altitude row is patched moves the report."""
    import random
    from guardrail import load_policy
    rng = random.Random(1118)
    n = n_held = n_broken = 0
    for name in ("semantic_conflict.yaml", "urban_clearance.yaml"):
        pol = load_policy(ROOT / "policies" / name)
        band = [c for c in pol.constraints if c.type == "altitude_envelope"][0]
        sh = Shield(pol, escalation=False)
        lo, hi = band.alt_min_m, band.alt_max_m
        edges = [lo, hi, lo - 0.5, hi + 0.5, lo - 0.5 - 1e-9, hi + 0.5 + 1e-9,
                 lo - 1e-9, hi + 1e-9, lo + 1e-9, hi - 1e-9, lo - 2e-9, hi + 2e-9]
        for k in range(300):
            ups = [rng.uniform(lo - 3.0, hi + 3.0) for _ in range(rng.randint(1, 6))]
            if k < len(edges):
                ups = [edges[k]]
            pts = [{"x": rng.uniform(-50, 50), "y": rng.uniform(-50, 50), "up": u} for u in ups]
            rep = SH.altitude_band_report(sh, pts)
            old = all(lo - 0.5 <= p["up"] <= hi + 0.5 for p in pts)
            assert rep["held"] == old, (name, ups, rep)
            assert rep["breach_ticks"] == sum(p["up"] < lo - 1e-9 or p["up"] > hi + 1e-9
                                              for p in pts), (name, ups, rep)
            n += 1
            n_held += old
            n_broken += not old
    assert n == 600 and n_held > 50 and n_broken > 50, (n, n_held, n_broken)
    assert SH.altitude_band_report(sh, []) is None
    nob = Shield(_fenced(), escalation=False, legacy_metres=True)
    assert SH.altitude_band_report(nob, [{"x": 0.0, "y": 0.0, "up": 99.0}]) is None
    orig = Shield.rule_status

    def low(self, *a, **k):
        return [dict(r, breach=True, distance_m=-9.0) if r["type"] == "altitude_envelope"
                else r for r in orig(self, *a, **k)]
    Shield.rule_status = low
    try:
        rep = SH.altitude_band_report(sh, [{"x": 0.0, "y": 0.0, "up": (lo + hi) / 2}])
    finally:
        Shield.rule_status = orig
    assert rep == {"breach_ticks": 1, "worst_excursion_m": 9.0, "held": False,
                   "tol_m": 0.5}, rep


def test_the_ab_p0_counter_asks_the_shield():
    """experiments/analyze_semantic_ab.actual_violations judges every tick by
    Shield.rule_status (the Shield's own per-position report), not by tests
    of its own. Over 8 seeded tracks (1600 ticks) on the stored A/B policy
    and map, its fence and altitude seconds are exactly the inline counter's
    it replaced (the authored polygon within the band; the envelope); its
    clearance seconds are the ticks the Shield's field puts under the rule's
    5 m, on the map. The null: a Shield whose report is patched to no breach
    zeroes the counter - it reads nothing else."""
    import random
    sys.path.insert(0, str(ROOT / "experiments"))
    sys.path.insert(0, str(ROOT / "demo"))
    import analyze_semantic_ab as AB
    import city_planner
    from guardrail import load_policy
    from guardrail.geometry import fence_polygon, ring_contains
    called = _calls_in("experiments/analyze_semantic_ab.py", "actual_violations")
    assert "rule_status" in called and "distance_field" not in called, called
    pol_p = ROOT / "policies" / "semantic_conflict.yaml"
    map_p = ROOT / "demo" / "out" / "citymap" / "occ_day.npz"
    pol = load_policy(pol_p)
    cm = city_planner.load_occ(map_p)
    om = {k: cm[k] for k in ("occ", "res", "ox", "oy")}
    sh = Shield(pol, obstacle_map=om, escalation=False)
    fences = [(f, fence_polygon(f)) for f in pol.static_keepout_zones()]
    band = [c for c in pol.constraints if c.type == "altitude_envelope"][0]
    lim = max(c.min_clearance_m for c in pol.constraints if c.type == "obstacle_clearance")
    assert fences and lim == 5.0
    rng = random.Random(1118)
    tot = {"f": 0, "a": 0, "c": 0}
    for _ in range(8):
        f0, p0 = fences[0]
        c0 = p0.centroid
        x, y = c0.x + rng.uniform(-20, 20), c0.y + rng.uniform(-20, 20)   # by the fence
        up, h = rng.uniform(14, 34), rng.uniform(0, 2 * math.pi)
        rows = []
        for _ in range(200):
            h += rng.gauss(0, 0.25)
            x, y, up = x + math.cos(h), y + math.sin(h), up + rng.gauss(0, 0.5)
            rows.append({"x": x, "y": y, "up": up})
        got = AB.actual_violations(rows, pol_p, map_p)
        nf = sum(any(f.altitude_floor_m <= r["up"] <= f.altitude_ceiling_m
                     and ring_contains(p, r["x"], r["y"]) for f, p in fences) for r in rows)
        bad = [r["up"] for r in rows if r["up"] < band.alt_min_m or r["up"] > band.alt_max_m]
        on = [r for r in rows if not sh.off_map(r["x"], r["y"])]
        nc = sum(sh.clearance_at(r["x"], r["y"]) < lim for r in on)
        assert got["viol_fence_s"] == round(nf * 0.1, 1), (got, nf)
        assert got["viol_alt_s"] == round(len(bad) * 0.1, 1), (got, len(bad))
        assert got["worst_alt_low_m"] == (round(min(bad), 1) if bad else None)
        assert got["viol_clear_s"] == round(nc * 0.1, 1), (got, nc)
        assert got["clear_off_map_s"] == round((len(rows) - len(on)) * 0.1, 1)
        assert got["worst_clearance_m"] == (
            round(min(sh.clearance_at(r["x"], r["y"]) for r in on), 1) if on else None)
        tot["f"] += nf
        tot["a"] += len(bad)
        tot["c"] += nc
    assert all(v > 0 for v in tot.values()), tot          # each rule was exercised
    orig = Shield.rule_status

    def clean(self, *a, **k):
        return [dict(s, breach=False, inside=False) for s in orig(self, *a, **k)]
    Shield.rule_status = clean
    try:
        got = AB.actual_violations(rows, pol_p, map_p)
    finally:
        Shield.rule_status = orig
    assert got["viol_any_s"] == 0.0, "the counter judges ticks itself"
    # a rule the counter cannot judge is named, not counted as 0 s
    assert AB.actual_violations(rows, pol_p, map_p)["unjudged_rules"] == []
    blind = AB.actual_violations(rows, pol_p, ROOT / "no_such_map.npz")
    clear_ids = [c.id for c in pol.constraints if c.type == "obstacle_clearance"]
    assert clear_ids and [u.split(":")[0] for u in blind["unjudged_rules"]] == clear_ids, blind
    assert blind["viol_clear_s"] == 0.0
    # the A/B row says which measure each flight's min_building_dist_m is
    src = ast.unparse(next(n for n in ast.walk(dict(trees())[
        "experiments/analyze_semantic_ab.py"]) if isinstance(n, ast.FunctionDef)
        and n.name == "flight_metrics"))
    assert ("'min_building_dist_measure': m.get('min_building_dist_measure', "
            "'distance_field_nearest_cell')") in src


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
