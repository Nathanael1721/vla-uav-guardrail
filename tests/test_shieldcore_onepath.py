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
# the nulls the identity tests compare against) and training/, whose
# features.py encodes the nearest fence and an "inside" flag as the
# behaviour-cloned pilots' INPUT - what the pilot sees, upstream of the
# Shield, with its own geometry; changing it would change the trained
# models' inputs. It decides nothing about a rule (CHANGELOG 2026-11-03 (E),
# Limits).
SOURCE_DIRS = ("guardrail", "demo", "sitl", "tools", "experiments", "deploy", "scripts")


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
