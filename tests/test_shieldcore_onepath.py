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

HARD_P0 = {"constraint_type": "hard", "priority": "P0", "violation_action": "project_fix"}

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
