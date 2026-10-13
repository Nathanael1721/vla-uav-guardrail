"""Time the Shield at the grant's design load: 50 rules, 48 of them polygons.

    python experiments/bench_shield_50rules.py                     # 3 s / 0.5 s
    python experiments/bench_shield_50rules.py --horizon 5 --dt 0.1
    python experiments/bench_shield_50rules.py --impl current --json out.json
    python experiments/bench_shield_50rules.py --horizon 5 --dt 0.1 --variant moving
    python experiments/bench_shield_50rules.py --horizon 5 --dt 0.1 --flags segment_check=1

WHY THIS EXISTS

The grant locks two numbers on the monitor, and the Shield had never been timed
against either of them at the load they are stated for:

  * Policy DSL page, acceptance KPIs: "Monitor query latency at 10 Hz with 50
    active rules <= 5 ms (10% of monitor budget)".
  * Safety Shield page: "With ~50 active rules and 50 future-pose checks per
    tick, total query budget is <= 5 ms (10% of the 100 ms monitor budget)" -
    and the tick itself is 100 ms end to end at 10 Hz.

The audit of 2026-10-05 measured it once, by hand, in a scratch script that was
not kept: `_check` 14 ms in clear sky and 34 ms near a fence, `filter()` about
170 ms near a fence - longer than the tick it has to fit in. A number nobody can
re-run is a number nobody can re-check, so this is the script.

WHAT IT MEASURES

Three scenarios, each a seeded list of (state, action) pairs:

  clear    no fence within reach of the forecast. Most of a real flight.
  grid     anywhere among the 48 fences, any heading. Dense airspace.
  near     1-8 m outside a fence's margin, flying at it. The repair path runs,
           and this is where `filter()` used to blow the tick.

For each: `Shield._check` (the monitor query the 5 ms budget is about) and
`Shield.filter` (the whole tick: check, repair, re-check, rescue), as median, p99
and max in milliseconds over every sample.

THE NULL

Two baselines are printed beside every number, because a fast time on its own
cannot say whether the Shield got faster or simply stopped looking:

  floor    the same samples against a policy with the 48 fences removed (only
           the speed and altitude rules). No fence index can beat this; it is
           the cost of everything that is not fence geometry.
  legacy   the per-fence loop as it stood before guardrail/ir.py
           (tests/legacy_fence_loop.py, a verbatim copy of the old methods,
           rebuilding every buffered polygon on every test).

and the run fails loudly unless (a) the new Shield and the legacy loop reach
IDENTICAL decisions on every sample - violations, repairs, emitted action - and
(b) the `near` scenario actually raises geofence violations while `clear` raises
none. Without (b) a Shield that skipped every fence would post the best time in
the table.

THE VARIANTS (--variant, 2026-10-12)

The same 50 rules on the Shield's two slower paths, as tests/test_shield.py
`_fifty_variant` builds them:

  edge     every rule with a weekly window ending 17:30, the clock at
           17:29:57, so the window's edge falls inside the lookahead and every
           pose reads the schedule on both sides of its instant;
  moving   every polygon fence a dynamic_nfz drifting North at 1 m/s.

The legacy loop knows neither (it predates windows read to the second and
moving zones), so a variant is timed against its own null: for `moving`,
`allzones` - the Shield with every moving zone built at the tick's time and
judged at every pose, no box test (the per-fence loop the swept-box skip and
then ir.anchor_reach replaced) - and the run fails unless `current` and
`allzones` decide identically. `--flags` passes the repair library's modes
(repair_mode=first_feasible, path_repair=1, segment_check=1) to `current`
and to `allzones`, so the moving variant is still compared decision for
decision; the legacy loop has no flags, so with `--flags` on the static
variant the agreement check is skipped (and says so) rather than failing on
the decisions a flag is meant to change.

Desktop numbers only. The grant's budget is for the onboard computer (a Jetson
Orin), which this machine is not; re-run this script there before quoting a
margin against it.
"""
from __future__ import annotations

import argparse
import gc
import json
import math
import platform
import random
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import shapely                                                      # noqa: E402

from guardrail.models import (Action4D, AltitudeEnvelope,            # noqa: E402
                              KinematicEnvelope, Policy, PolygonFence,
                              State)
from guardrail.shield import Shield                                 # noqa: E402

# The grid. 8 x 6 squares of 12 m at a 40 m pitch, 2 m margin: 28 m of open
# air between neighbouring margins, so a 4 m/s forecast over 3 s (12 m) can see
# at most one or two fences from a gap - which is the case the index is for.
COLS, ROWS, SIDE_M, PITCH_M, MARGIN_M = 8, 6, 12.0, 40.0, 2.0
TICK_BUDGET_MS = 100.0      # Safety Shield page: monitor tick 100 ms at 10 Hz
QUERY_BUDGET_MS = 5.0       # Policy DSL page: 50 rules, <= 5 ms


def fifty_rule_policy(with_fences: bool = True) -> Policy:
    """2 envelopes + 48 polygon fences = 50 rules, the grant's stated load.

    Alternate fences stop at 25 m so the altitude band short-circuit is part of
    the measurement too; samples fly between 5 and 30 m, so both kinds bind.
    """
    rules = [
        KinematicEnvelope(id="kin", type="kinematic_envelope", speed_max_mps=5.0,
                          climb_rate_max_mps=3.0, yaw_rate_max_dps=90.0),
        AltitudeEnvelope(id="alt", type="altitude_envelope",
                         alt_min_m=2.0, alt_max_m=60.0),
    ]
    if with_fences:
        for r in range(ROWS):
            for c in range(COLS):
                x0, y0 = c * PITCH_M, r * PITCH_M
                rules.append(PolygonFence(
                    id=f"nfz-{r}-{c}", type="polygon_fence",
                    vertices=[{"x": x0, "y": y0}, {"x": x0 + SIDE_M, "y": y0},
                              {"x": x0 + SIDE_M, "y": y0 + SIDE_M},
                              {"x": x0, "y": y0 + SIDE_M}],
                    altitude_floor_m=0.0,
                    altitude_ceiling_m=120.0 if (r + c) % 2 == 0 else 25.0,
                    margin_m=MARGIN_M))
    return Policy(policy_id="bench-50", constraints=rules)


# The clock of the `edge` variant: the 17:30 window edge is 3 s ahead.
EDGE_NOW = datetime(2026, 10, 7, 17, 29, 57)


def variant_policy(kind: str) -> Policy:
    """The 50-rule policy on one of the Shield's slow paths (module
    docstring, "The variants"); "static" is fifty_rule_policy() itself."""
    if kind == "static":
        return fifty_rule_policy()
    if kind not in ("edge", "moving"):
        raise ValueError(kind)
    data = fifty_rule_policy().model_dump(mode="json", exclude_none=True)
    for c in data["constraints"]:
        if kind == "edge":
            c["valid_time"] = {"recurrence": {"start_time": "07:30", "end_time": "17:30"}}
        elif c["type"] == "polygon_fence":
            c.update(type="dynamic_nfz", motion={"vx_mps": 1.0})
    return Policy.model_validate(data)


def _n_zones(policy: Policy) -> int:
    return sum(1 for c in policy.constraints if c.type in ("polygon_fence", "dynamic_nfz"))


def _action(rng: random.Random, vx: float, vy: float) -> Action4D:
    return Action4D(vx=vx, vy=vy, vz_up=rng.uniform(-1.0, 1.0),
                    yaw_rate=rng.uniform(-0.5, 0.5))


def samples(scenario: str, n: int, seed: int) -> list[tuple[State, Action4D]]:
    rng = random.Random(f"{scenario}:{seed}")
    out = []
    span_x = (COLS - 1) * PITCH_M + SIDE_M
    span_y = (ROWS - 1) * PITCH_M + SIDE_M
    for _ in range(n):
        if scenario == "clear":
            # >= 60 m west of the grid: 8 m/s x 5 s = 40 m of forecast plus the
            # 2 m margin cannot reach it, at either horizon.
            st = State(x=rng.uniform(-200.0, -60.0), y=rng.uniform(-50.0, span_y + 50),
                       up=rng.uniform(5.0, 30.0))
            hd, spd = rng.uniform(0, 2 * math.pi), rng.uniform(0.0, 6.0)
            a = _action(rng, spd * math.cos(hd), spd * math.sin(hd))
        elif scenario == "grid":
            st = State(x=rng.uniform(-10.0, span_x + 10), y=rng.uniform(-10.0, span_y + 10),
                       up=rng.uniform(5.0, 30.0))
            hd, spd = rng.uniform(0, 2 * math.pi), rng.uniform(0.0, 6.0)
            a = _action(rng, spd * math.cos(hd), spd * math.sin(hd))
        elif scenario == "near":
            # A point d metres outside one face of a fence's margin, flying at
            # the fence with a random sideways component.
            r, c = rng.randrange(ROWS), rng.randrange(COLS)
            cx, cy = c * PITCH_M + SIDE_M / 2, r * PITCH_M + SIDE_M / 2
            half = SIDE_M / 2 + MARGIN_M
            d = rng.uniform(1.0, 8.0)
            face = rng.randrange(4)
            along = rng.uniform(-half, half)
            nx, ny = ((1, 0), (-1, 0), (0, 1), (0, -1))[face]
            st = State(x=cx + nx * (half + d) + ny * along,
                       y=cy + ny * (half + d) + nx * along,
                       up=rng.uniform(5.0, 30.0))
            spd, side = rng.uniform(1.0, 6.0), rng.uniform(-1.5, 1.5)
            a = _action(rng, -nx * spd + ny * side, -ny * spd + nx * side)
        else:
            raise ValueError(scenario)
        out.append((st, a))
    return out


# Repair fields that are a record for the escalation FSM, not behaviour
# (added 2026-10-07). The frozen legacy loop cannot report them, so the
# comparison leaves them out, exactly as tests/test_ir.py does.
_RECORD_ONLY = ("magnitude_m", "axis", "recovery")


def _decision_key(d) -> str:
    out = d.model_dump()
    out["repairs"] = [{k: v for k, v in r.items() if k not in _RECORD_ONLY}
                      for r in out.get("repairs", [])]
    return json.dumps(out, sort_keys=True, default=str)


def _time(fn, pairs, warmup: int = 20) -> tuple[list[float], list]:
    for st, a in pairs[:warmup]:
        fn(st, a)
    out, ms = [], []
    gc.disable()
    try:
        for st, a in pairs:
            t0 = time.perf_counter()
            r = fn(st, a)
            ms.append((time.perf_counter() - t0) * 1e3)
            out.append(r)
    finally:
        gc.enable()
    return ms, out


def _stats(ms: list[float]) -> dict:
    s = sorted(ms)
    p99 = s[min(len(s) - 1, int(math.ceil(0.99 * len(s))) - 1)]
    return {"n": len(s), "median_ms": statistics.median(s), "p99_ms": p99,
            "max_ms": s[-1], "mean_ms": statistics.fmean(s)}


class AllZonesShield(Shield):
    """The null for the moving variant: every moving zone built at the tick's
    time and judged at every pose, with no box test at all."""

    def _reach(self, rec, tick, box, horizon):
        return tick.current(rec) if (rec.moving and tick is not None) else rec


def _make(impl: str, policy: Policy, horizon: float, dt: float, now=None,
          flags: dict | None = None) -> Shield:
    # "current" and "legacy" are compared decision for decision, so both run
    # without the escalation FSM: the frozen legacy repairs carry no
    # magnitude_m, and the FSM would fault on them (tests/test_ir.py compares
    # the same way). "current_fsm" is the Shield as it flies, FSM on, and is
    # timed only.
    flags = flags or {}
    if impl == "current":
        return Shield(policy, lookahead_s=horizon, dt=dt, escalation=False, now=now, **flags)
    if impl == "current_fsm":
        return Shield(policy, lookahead_s=horizon, dt=dt, now=now, **flags)
    if impl == "legacy":
        from legacy_fence_loop import LegacyFenceShield
        return LegacyFenceShield(policy, lookahead_s=horizon, dt=dt, escalation=False)
    if impl == "allzones":
        return AllZonesShield(policy, lookahead_s=horizon, dt=dt, escalation=False, now=now,
                              **flags)
    raise ValueError(impl)


def _flags(text: str) -> dict:
    """"repair_mode=first_feasible,path_repair=1" as keyword arguments."""
    out: dict = {}
    for kv in filter(None, (t.strip() for t in text.split(","))):
        k, _, v = kv.partition("=")
        out[k] = (v in ("1", "true", "True")) if k in ("path_repair", "segment_check") else v
    return out


def _git() -> dict:
    def run(*a):
        try:
            return subprocess.run(["git", *a], cwd=ROOT, capture_output=True,
                                  text=True, timeout=20).stdout.strip()
        except Exception:                                    # noqa: BLE001
            return ""
    return {"rev": run("rev-parse", "--short", "HEAD"),
            "dirty_guardrail": bool(run("status", "--porcelain", "guardrail"))}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--impl", nargs="+", default=None,
                    choices=["current", "legacy", "allzones"],
                    help="default: current legacy (static), current allzones "
                         "(moving), current (edge)")
    ap.add_argument("--variant", default="static", choices=["static", "edge", "moving"])
    ap.add_argument("--flags", default="",
                    help="Shield flags for 'current', e.g. repair_mode=first_feasible,"
                         "path_repair=1,segment_check=1")
    ap.add_argument("--horizon", type=float, default=3.0)
    ap.add_argument("--dt", type=float, default=0.5)
    ap.add_argument("--n-check", type=int, default=1000)
    ap.add_argument("--n-filter", type=int, default=300)
    ap.add_argument("--legacy-cap", type=int, default=0,
                    help="cap legacy sample counts (it is ~100x slower); 0 = same n")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--scenarios", nargs="+", default=["clear", "grid", "near"])
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)
    if args.impl is None:
        args.impl = {"static": ["current", "legacy"], "moving": ["current", "allzones"],
                     "edge": ["current"]}[args.variant]
    if "legacy" in args.impl and args.variant != "static":
        ap.error("the legacy loop predates windows read to the second and moving zones; "
                 "use --impl current allzones with --variant moving")
    flags = _flags(args.flags)
    now = (lambda: EDGE_NOW) if args.variant == "edge" else None

    full, floor = variant_policy(args.variant), fifty_rule_policy(False)
    assert len(full.constraints) == 50, len(full.constraints)
    print(f"policy: {len(full.constraints)} rules "
          f"({_n_zones(full)} keep-out zones), "
          f"horizon {args.horizon:g} s at dt {args.dt:g} s "
          f"({int(round(args.horizon / args.dt)) + 1} poses per forecast), "
          f"variant {args.variant}, flags {flags or 'none'}")
    print(f"python {platform.python_version()}  shapely {shapely.__version__}  "
          f"GEOS {'.'.join(map(str, shapely.geos_version))}  {platform.processor()}")

    results: dict = {}
    agree: dict = {}
    sanity: dict = {}
    for sc in args.scenarios:
        for fn_name, n in (("_check", args.n_check), ("filter", args.n_filter)):
            pairs = samples(sc, n, args.seed)
            row = {}
            decisions = {}
            timed = [*args.impl, "floor"]
            if "current" in args.impl and fn_name == "filter":
                timed.insert(timed.index("current") + 1, "current_fsm")
            for impl in timed:
                use = pairs
                if impl == "legacy" and args.legacy_cap:
                    use = pairs[:args.legacy_cap]
                sh = (_make("current", floor, args.horizon, args.dt) if impl == "floor"
                      else _make(impl, full.model_copy(deep=True), args.horizon, args.dt,
                                 now=now, flags=flags))
                ms, outs = _time(getattr(sh, fn_name), use)
                row[impl] = _stats(ms)
                if impl in ("current", "legacy", "allzones"):
                    decisions[impl] = outs
                if impl == "current" or (impl != "floor" and "current" not in args.impl):
                    if fn_name == "_check":
                        n_geo = sum(any(v.category == "geofence" for v in o) for o in outs)
                    else:
                        n_geo = sum(any(v.category == "geofence" for v in o.violations)
                                    for o in outs)
                    sanity[f"{sc}/{fn_name}"] = {"samples": len(outs),
                                                 "with_geofence_violation": n_geo}
            null = next((n for n in ("legacy", "allzones") if n in decisions), None)
            if "current" in decisions and null == "legacy" and flags:
                agree[f"{sc}/{fn_name}"] = {"skipped": "the legacy loop has no flags",
                                            "null": null}
            elif "current" in decisions and null is not None:
                k = len(decisions[null])
                same = sum(
                    (json.dumps([v.model_dump() for v in a]) == json.dumps([v.model_dump() for v in b]))
                    if fn_name == "_check" else (_decision_key(a) == _decision_key(b))
                    for a, b in zip(decisions["current"][:k], decisions[null]))
                agree[f"{sc}/{fn_name}"] = {"identical": same, "compared": k, "null": null}
            results[f"{sc}/{fn_name}"] = row

    # Building the IR moved work to construction and to hot_apply, which have
    # budgets of their own (Policy DSL page: cold start of a 50-rule bundle
    # <= 1 s, hot-apply <= 50 ms). Shield construction here is only the IR part
    # of a cold start - no YAML parse, no bundle signature check.
    life: dict = {}
    for impl in [i for i in args.impl if i != "allzones"]:
        ms_new, ms_hot = [], []
        for i in range(30):
            pol = full.model_copy(deep=True)
            t0 = time.perf_counter()
            sh = _make(impl, pol, args.horizon, args.dt, now=now, flags=flags)
            ms_new.append((time.perf_counter() - t0) * 1e3)
            fence = PolygonFence(id=f"hot-{i}", type="polygon_fence",
                                 vertices=[{"x": -30, "y": -30}, {"x": -20, "y": -30},
                                           {"x": -20, "y": -20}, {"x": -30, "y": -20}])
            t0 = time.perf_counter()
            sh.hot_apply(fence)
            ms_hot.append((time.perf_counter() - t0) * 1e3)
        life[impl] = {"construct": _stats(ms_new), "hot_apply": _stats(ms_hot)}

    print()
    for impl, row in life.items():
        print(f"{impl:<8} Shield(...) with 50 rules median {row['construct']['median_ms']:.2f} ms"
              f" (max {row['construct']['max_ms']:.2f});  hot_apply median "
              f"{row['hot_apply']['median_ms']:.3f} ms (max {row['hot_apply']['max_ms']:.3f}; "
              f"budget 50 ms)")
    print()
    hdr = f"{'scenario/call':<16}{'impl':<12}{'n':>6}{'median':>10}{'p99':>10}{'max':>10}   budget"
    print(hdr)
    print("-" * len(hdr))
    for key, row in results.items():
        budget = QUERY_BUDGET_MS if key.endswith("_check") else TICK_BUDGET_MS
        for impl, s in row.items():
            verdict = "ok" if s["p99_ms"] <= budget else "OVER"
            print(f"{key:<16}{impl:<12}{s['n']:>6}{s['median_ms']:>9.3f}m{s['p99_ms']:>9.3f}m"
                  f"{s['max_ms']:>9.3f}m   {budget:g} ms p99 {verdict}")
    print()
    for key, s in sanity.items():
        print(f"sanity {key:<16} geofence violation on {s['with_geofence_violation']}"
              f"/{s['samples']} samples")
    for key, s in agree.items():
        if "skipped" in s:
            print(f"agree  {key:<16} not compared: {s['skipped']} ({', '.join(flags)} on)")
        else:
            print(f"agree  {key:<16} identical decisions {s['identical']}/{s['compared']}")

    failures = []
    for key, s in agree.items():
        if "skipped" not in s and s["identical"] != s["compared"]:
            failures.append(f"{key}: current and {s['null']} disagree on "
                            f"{s['compared'] - s['identical']} samples")
    for key, s in sanity.items():
        if key.startswith("near/") and s["with_geofence_violation"] == 0:
            failures.append(f"{key}: no geofence violation at all - the scenario "
                            f"tests nothing, or the fences were skipped")
        if key.startswith("clear/") and s["with_geofence_violation"] != 0:
            failures.append(f"{key}: a geofence violation in clear sky - the "
                            f"scenario is not clear")

    cmd = "python experiments/bench_shield_50rules.py " + " ".join(
        argv if argv is not None else sys.argv[1:])
    print(f"\nreproduce: {cmd.strip()}")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps({
            "command": cmd.strip(), "git": _git(),
            "python": platform.python_version(), "shapely": shapely.__version__,
            "geos": ".".join(map(str, shapely.geos_version)),
            "machine": platform.processor(), "horizon_s": args.horizon, "dt_s": args.dt,
            "rules": len(full.constraints), "variant": args.variant, "flags": flags,
            "fences": _n_zones(full),
            "budgets_ms": {"_check": QUERY_BUDGET_MS, "filter": TICK_BUDGET_MS},
            "results": results, "lifecycle": life, "agreement": agree,
            "sanity": sanity,
            "failures": failures}, indent=2), encoding="utf-8")
        print(f"wrote {args.json}")
    for f in failures:
        print(f"FAIL  {f}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
