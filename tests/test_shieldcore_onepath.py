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
    call the Shield's evaluator - a patched evaluator changes their answer.

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
    sh = Shield(pol, now=lambda: when, escalation=False)
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
        sh2 = Shield(pol, now=lambda: when, escalation=False)
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
    sh = Shield(_fenced(), escalation=False)
    d = sh.filter(st, a, t=0.0)
    assert [v.rule_id for v in d.violations] == ["box"], d.violations
    orig = SH.SlidingWindow.admit

    def far(self, tick, state, action, trajectory):
        moved = [(t, p._replace(x=p.x + 1000.0)) for t, p in trajectory]
        return orig(self, tick, state, action, moved)
    SH.SlidingWindow.admit = far
    try:
        blind = Shield(_fenced(), escalation=False).filter(st, a, t=0.0)
    finally:
        SH.SlidingWindow.admit = orig
    assert blind.violations == [], "the checker forecast the action itself"


def test_the_buffer_keeps_the_last_50_actions_with_their_trajectories():
    sh = Shield(_fenced(), escalation=False)
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
    sh = Shield(pol, escalation=False)
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
    """With guard_field="shield" (opt-in, --guard-field shield; not yet
    flown) FenceGuard's building half (gate's obstacle brake, slide's
    clearance test) measures on the Shield's distance field -
    Shield.clearance_at when the flight's Shield holds the map, else the same
    field read by the same function (ClearanceField) - not on a grid scan of
    its own. A Shield whose field is patched moves the guard's answer; the
    guard's clearance is min(cap, the Shield's). The default stays the grid
    scan every stored follow flight flew (follow_vlm.grid_scan_clearance)."""
    import numpy as np
    sys.path.insert(0, str(ROOT / "demo"))
    import follow_vlm as F
    from guardrail import load_policy
    from guardrail.shield import ClearanceField
    res, ox, oy = 0.5, -20.0, -20.0
    occ = np.zeros((80, 80), dtype=np.uint8)
    occ[50:60, 30:50] = 1                                  # a block at x 5..9.5
    omap = {"occ": occ, "res": res, "ox": ox, "oy": oy}
    pol = load_policy(ROOT / "policies" / "follow_car.yaml")   # has obstacle_clearance
    sh = Shield(pol, obstacle_map=omap, escalation=False)
    assert sh.has_obstacle_map
    dflt = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh)
    assert dflt.guard_field == "grid"
    g = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh,
                     guard_field="shield")
    rng = __import__("random").Random(3)
    for _ in range(100):
        x, y = rng.uniform(-15, 15), rng.uniform(-15, 15)
        assert dflt.clearance(x, y, 12.0) == F.grid_scan_clearance(omap, x, y, 12.0)
    for _ in range(200):
        x, y = rng.uniform(-15, 15), rng.uniform(-15, 15)
        assert g.clearance(x, y, 12.0) == min(12.0, sh.clearance_at(x, y))
    # without the flight's Shield: the same field, the same reader
    g2 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, guard_field="shield")
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
        g3 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh,
                          guard_field="shield")
        s1, _, _ = g3.gate(0.0, 2.0, 4.0, 0.0)
        assert s1 == 1.0 and g3.last_cause is None, "the building half scans the grid itself"
        # ...and the default does not read the Shield's field at all
        g4 = F.FenceGuard(pol, obstacle_map=omap, min_clearance_m=5.0, shield=sh)
        assert g4.gate(0.0, 2.0, 4.0, 0.0)[0] == s0
    finally:
        Shield.clearance_at = orig


# CLEARANCE MEASURED OUTSIDE THE LIBRARY, BY NAME (review of a38ac6c).
#
# The by-behaviour scan above sees shapely calls, ray casts and clamped
# projections; it cannot see a clearance measured on a distance transform or
# a grid scan. These do that: every function outside the library that calls
# city_planner.distance_field (a 5x5 chamfer, read by nearest-cell rounding)
# or scans occupancy cells with hypot. Each is listed with what it is; the
# ones marked RULE evaluate the obstacle_clearance rule on a measure that is
# NOT the Shield's (guardrail/shield.py field_distance) and are the rows'
# remaining work, stated here rather than hidden. A new one fails, and so
# does a stale entry.
_CLEARANCE_OUTSIDE = {
    ("demo/city_planner.py", "inflate"):
        "planning: inflates the planner's grid by a clearance (route cost, no verdict)",
    ("demo/city_planner.py", "_penalty_flat"):
        "planning: the route planner's clearance cost",
    ("demo/identity.py", "__init__"):
        "target identity: distance to the street cells, not a rule",
    ("demo/follow_vlm.py", "grid_scan_clearance"):
        "RULE (controller side): FenceGuard's building half by default; the "
        "Shield-field read is opt-in (--guard-field shield) until flown on the AGX",
    ("demo/semantic_demo.py", "fly"):
        "RULE (analysis side): min_building_dist_m reported by the semantic demo",
    ("demo/semantic_seek.py", "fly"):
        "RULE (analysis side): min_building_dist_m reported by semantic seek",
    ("experiments/analyze_semantic_ab.py", "actual_violations"):
        "RULE (analysis side): the semantic A/B P0 counter's obstacle_clearance "
        "check (and its inline fence band and altitude envelope tests); not "
        "folded: re-scoring would change stored A/B results",
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
    """No clearance measure outside the library but the listed ones; the
    RULE ones are the known remaining evaluators of WP3-22 / X-13. The null:
    a planted grid scan and a planted distance_field reader are found."""
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
