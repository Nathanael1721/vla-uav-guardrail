"""The 2026-10-12 Shield work must not move a decision with its new flags off.

Run:  python tests/test_shieldcore_identity.py

Three changes went into guardrail/shield.py and guardrail/ir.py that were
meant to change only how fast the Shield answers, never what it answers:

  * a moving dynamic_nfz is materialised only when its anchor box
    (ir.anchor_reach) meets the forecast's box - before, all 48 moving zones
    of the 50-rule load were rebuilt on every check;
  * plain rules that share a weekly window read it once per pose;
  * forecast poses are a NamedTuple instead of a validated State;

and the repair chain now runs as the operator library guardrail/repair.py,
which records every operator it considers. tests/test_check_contract.py pins
`_check` over 28 800 samples of the static policies; this file pins the rest:

  * the 8 Oct follow (tests/fixtures/, 1101 ticks, the CityLife map) replays
    to the decisions the Shield made on it at commit 4418ae0, before any of
    the above, tick for tick (`DECISIONS`, regenerated only on purpose:
    `python tests/test_shieldcore_identity.py --regenerate-decisions`);
  * the 13 LEGACY scenarios of tests/test_sweep.py fly to the pinned end
    pose and repair count with every new flag spelled out off;
  * on moving zones (translating, turning, mixed with static ones), the fast
    path answers exactly as materialising every zone and judging it the old
    way, on `_check` and on `filter()`, and `unsafe_depth` measures a zone
    where it has moved to;
  * the shared per-window memo answers exactly as each rule reading its own
    window: rules on the same clock window but different days or a
    different end, and a rule a time_window_switch turns off or on beside a
    plain rule of the same window, at a window edge; and on the 50-rule
    load with every window ending inside the lookahead.
"""
import functools
import json
import math
import random
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.models import Action4D, Policy, State                    # noqa: E402
from guardrail.shield import Shield                                     # noqa: E402

DECISIONS = ROOT / "tests" / "fixtures" / "pas_ap_20261008_decisions_4418ae0.json"


def _flight():
    import test_flight_pas_ap_20261008 as FL
    return FL


def _row(d) -> list:
    """A decision, rounded to 1e-6 so a last-bit difference between
    platforms' libm cannot fail the comparison, but any decision can."""
    r6 = lambda v: None if v is None else round(v, 6)              # noqa: E731
    return [[r6(v) for v in (d.emitted.vx, d.emitted.vy, d.emitted.vz_up, d.emitted.yaw_rate)],
            [[v.rule_id, v.category, r6(v.predicted_at_s)] for v in d.violations],
            [[r.operator, r6(r.magnitude_m), r.axis, r.recovery] for r in d.repairs],
            d.braked, [v.rule_id for v in d.emitted_violations]]


def _replay_decisions():
    """{tick: row} for every tick that saw a violation, and the ticks that
    did not (whose decision must be the raw action, untouched)."""
    FL = _flight()
    got = FL._inputs()
    if got is None:
        return None
    pol, rows, smap, src = got
    sh = FL.rail_shield(pol, lookahead_s=FL.LOOKAHEAD_S, dt=FL.LOOKAHEAD_DT_S,
                        obstacle_map=smap)
    touched, clean = {}, []
    for r in rows:
        st = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"])
        FL._subject(sh, r)
        d = sh.filter(st, Action4D(**r["raw"]), t=r["t"])
        if d.violations:
            touched[r["tick"]] = _row(d)
        else:
            assert d.emitted == d.raw and not d.repairs, r["tick"]
            clean.append(r["tick"])
    return touched, clean, src


def test_the_8_oct_flight_replays_to_the_decisions_of_4418ae0():
    want = json.loads(DECISIONS.read_text(encoding="utf-8"))
    got = _replay_decisions()
    assert got is not None, "the 8 Oct fixtures are missing from tests/fixtures"
    touched, clean, _ = got
    assert clean == want["clean_ticks"], "a different set of ticks saw a violation"
    moved = [t for t in want["touched"] if touched.get(int(t)) != want["touched"][t]]
    assert not moved, (f"{len(moved)} of {len(want['touched'])} decisions moved, first "
                       f"tick {moved[0]}: {touched.get(int(moved[0]))} != "
                       f"{want['touched'][moved[0]]}")
    # 81 ticks see a violation on replay (the log has 85: its states are
    # rounded to 1 mm and its subject to 1 cm, so a replay is not the flight).
    assert len(want["touched"]) == 81 and len(clean) == 1101 - 81


def test_the_13_legacy_scenarios_fly_as_before_with_every_new_flag_off():
    """tests/test_sweep.py's pins of the 13 LEGACY scenarios (end pose,
    repair count), flown by a Shield whose repair_mode, path_repair and
    segment_check are spelled out off. Where test_sweep pins them on two
    forecasts (its `_moved`: LEGACY on the 3 s comparison arm, LEGACY_GRANT
    on the grant's 5 s at 0.1 s), both are flown; on a test_sweep without
    them, LEGACY as `_run_nofsm` flies it."""
    import sweep_scenarios as S
    import test_sweep as TS
    orig = S.Shield
    S.Shield = functools.partial(Shield, repair_mode="chain", path_repair=False,
                                 segment_check=False)
    try:
        if hasattr(TS, "_moved") and hasattr(TS, "LEGACY_GRANT"):
            moved = TS._moved(TS.LEGACY, "3s") + TS._moved(TS.LEGACY_GRANT, "grant")
        else:
            moved = []
            for sid, (final, reps) in TS.LEGACY.items():
                rows, extra = TS._run_nofsm(sid)
                got = (extra["final"]["x"], extra["final"]["y"], extra["final"]["up"])
                n = sum(len(r["repairs"]) for r in rows)
                if got != final or n != reps:
                    moved.append(f"{sid}: final {got} (was {final}), repairs {n} (was {reps})")
    finally:
        S.Shield = orig
    assert not moved, "\n      ".join(moved)


# ------------------------------------------------- the moving-zone fast path

class _Materialise(Shield):
    """The judgement as it was before ir.anchor_reach: every moving zone
    built at the tick's time, then `_may_reach` on its own swept box."""

    def _reach(self, rec, tick, box, horizon):
        if not rec.moving or tick is None:
            return rec
        cur = tick.current(rec)
        return cur if self._may_reach(cur, box, horizon) else None


def _moving_policy(kind: str) -> Policy:
    import bench_shield_50rules as B
    data = B.fifty_rule_policy().model_dump(mode="json", exclude_none=True)
    for i, c in enumerate(data["constraints"]):
        if c["type"] != "polygon_fence":
            continue
        if kind == "translate":
            c.update(type="dynamic_nfz", motion={"vx_mps": 1.0})
        elif kind == "turn":
            c.update(type="dynamic_nfz", motion={"vx_mps": -0.5, "vy_mps": 0.7,
                                                  "yaw_rate_dps": 9.0})
        elif kind == "mixed" and i % 3:
            c.update(type="dynamic_nfz",
                     motion={"vx_mps": 2.0 * (i % 2) - 1.0, "yaw_rate_dps": 4.0 * (i % 4)})
    return Policy.model_validate(data)


def _samples(n: int, seed: int):
    import bench_shield_50rules as B
    rng = random.Random(seed)
    out = B.samples("near", n // 2, seed) + B.samples("grid", n - n // 2, seed)
    rng.shuffle(out)
    return out


def test_the_moving_fast_path_judges_exactly_the_zones_the_old_path_did():
    """1200 _check calls and 300 filter() ticks per policy, the zones aged
    from 0 to 90 s (so a turning zone has turned through 810 degrees): the
    anchor box never drops a zone the old judgement would have reached."""
    for kind in ("translate", "turn", "mixed"):
        pol = _moving_policy(kind)
        fast, slow = Shield(pol.model_copy(deep=True)), _Materialise(pol.model_copy(deep=True))
        pairs = _samples(1200, seed={"translate": 11, "turn": 12, "mixed": 13}[kind])
        for k, (st, a) in enumerate(pairs):
            t = 90.0 * k / len(pairs)
            if k % 4 == 0:
                da, db = fast.filter(st, a, t=t), slow.filter(st, a, t=t)
                assert da.model_dump() == db.model_dump(), (kind, k)
            else:
                va = [v.model_dump() for v in fast._check(st, a)]
                vb = [v.model_dump() for v in slow._check(st, a)]
                assert va == vb, (kind, k, va, vb)


def test_the_anchor_box_contains_the_materialised_swept_box():
    """The geometric claim behind the fast path, checked directly: at every
    age and horizon, the box ir.anchor_reach computes without building the
    zone contains the box `_Tick.swept` computes from the built zone."""
    from guardrail.ir import anchor_reach
    for kind in ("translate", "turn", "mixed"):
        sh = Shield(_moving_policy(kind))
        # The first tick anchors the policy's zones (their age counts from it).
        sh.filter(State(x=-500.0, y=-500.0, up=10.0), Action4D(), t=0.0)
        recs = [r for r in sh.ir.fences if r.moving]
        assert recs
        for k, t in enumerate((0.0, 0.1, 3.7, 41.0, 90.0)):
            tick = sh._new_tick(t)
            assert abs(tick._age(recs[0].rule) - t) < 1e-12
            for rec in recs:
                for horizon in (0.0, 0.5, 5.0):
                    a = anchor_reach(rec, tick._age(rec.rule), horizon)
                    b = tick.swept(tick.current(rec), horizon)
                    assert a[0] <= b[0] and a[1] <= b[1] and a[2] >= b[2] and a[3] >= b[3], \
                        (kind, t, rec.rule.id, horizon, a, b)
                    # ...and not a uselessly loose one: within a few metres.
                    assert max(b[0] - a[0], b[1] - a[1], a[2] - b[2], a[3] - b[3]) < 5.0


def test_unsafe_depth_measures_a_moving_zone_where_it_has_moved_to():
    """A zone drifting North at 1 m/s, anchored at t = 0: at t = 10 s it
    spans x 20-30, and an aircraft hovering at its middle (25, 0) is 6.0 m
    inside its margin ring. Measured against where the zone was anchored
    (x 10-20) that position is outside it, and the depth would read 0: the
    FSM's X6 would see no progress on a recovery that is working."""
    pol = Policy.model_validate({"policy_id": "mz", "constraints": [
        {"id": "mz", "type": "dynamic_nfz", "margin_m": 1.0, "motion": {"vx_mps": 1.0},
         "vertices": [{"x": 10, "y": -5}, {"x": 20, "y": -5}, {"x": 20, "y": 5},
                      {"x": 10, "y": 5}]}]})
    sh = Shield(pol, escalation=False)
    sh.filter(State(x=-50.0, y=-50.0, up=10.0), Action4D(), t=0.0)
    here = State(x=25.0, y=0.0, up=10.0)
    d = sh.filter(here, Action4D(), t=10.0)
    assert [v.rule_id for v in d.violations] == ["mz"]
    assert abs(sh.unsafe_depth(here) - 6.0) < 1e-9, sh.unsafe_depth(here)
    assert sh.unsafe_depth(State(x=15.0, y=0.0, up=10.0)) == 0.0     # where it was


# ------------------------------------------------- the shared window memo

def _no_memo():
    """Every rule reads its own window (`_Schedule._plain_key` -> None), as
    before the memo. Returns the restore function."""
    import guardrail.shield as S
    had = "_plain_key" in S._Schedule.__dict__
    old = S._Schedule.__dict__.get("_plain_key")
    S._Schedule._plain_key = lambda self, rule: None

    def restore():
        if had:
            S._Schedule._plain_key = old
        else:
            del S._Schedule._plain_key
    return restore


def _square(zid, x0, y0, side, window, days=None):
    rec = {"start_time": window[0], "end_time": window[1]}
    if days:
        rec["days"] = days
    return {"id": zid, "type": "polygon_fence", "margin_m": 1.0,
            "valid_time": {"recurrence": rec},
            "vertices": [{"x": x0, "y": y0}, {"x": x0 + side, "y": y0},
                         {"x": x0 + side, "y": y0 + side}, {"x": x0, "y": y0 + side}]}


def _switch(sid, target, active, window):
    return {"id": sid, "type": "time_window_switch", "target_id": target, "active": active,
            "valid_time": {"recurrence": {"start_time": window[0], "end_time": window[1]}}}


def _memo_case(name, window_a, window_b, clock, want, days=(None, None), switch=None):
    """Two zones: the SHARER, listed first, beside the forecast (inside its
    box, so it is judged at every pose and fills the memo at every pose
    time, but never entered), and the TARGET on the forecast. The aircraft
    is at (0, 0, 10 m) flying North-East at 3 m/s each way; it enters the
    target's ring after 2.76 s, 0.76 s after the window edge at the clock
    + 2 s."""
    cons = [_square("sharer", 11, 0, 3, window_a, days[0]),
            _square("target", 9, 9, 10, window_b, days[1])]
    if switch is not None:
        cons.append(_switch("sw", "target", *switch))
    return name, cons, clock, want


# (name, constraints, clock, the rule ids the forecast must break)
_MEMO_CASES = (
    _memo_case("same clock window, other days", ("07:30", "17:30"), ("07:30", "17:30"),
               datetime(2026, 10, 12, 7, 29, 58), ["target"], days=(["Sun"], ["Mon"])),
    _memo_case("same start, other end", ("07:30", "17:30"), ("07:30", "18:00"),
               datetime(2026, 10, 12, 17, 29, 58), ["target"]),
    _memo_case("switched off beside a plain rule", ("07:00", "18:00"), ("07:00", "18:00"),
               datetime(2026, 10, 12, 11, 59, 58), [], switch=(False, ("12:00", "12:01"))),
    _memo_case("switched on beside a plain rule", ("13:00", "14:00"), ("13:00", "14:00"),
               datetime(2026, 10, 12, 11, 59, 58), ["target"], switch=(True, ("12:00", "12:01"))),
    # A corridor a corridor_swap replaces from the clock + 2 s, beside a plain
    # corridor of the same window that holds the whole forecast: the target
    # runs along the forecast to (7, 7) and turns North, so the forecast
    # leaves it after 2.5 s - after it was replaced. Nothing is broken.
    ("replaced by a swap beside a plain rule",
     [{"id": "wide", "type": "corridor", "width_m": 200.0, "altitude_floor_m": 0,
       "altitude_ceiling_m": 50, "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}],
       "valid_time": {"recurrence": {"start_time": "07:00", "end_time": "18:00"}}},
      {"id": "target", "type": "corridor", "width_m": 1.0, "altitude_floor_m": 0,
       "altitude_ceiling_m": 50,
       "centerline": [{"x": 0, "y": 0}, {"x": 7, "y": 7}, {"x": 7, "y": 30}],
       "valid_time": {"recurrence": {"start_time": "07:00", "end_time": "18:00"}}},
      {"id": "swap", "type": "corridor_swap", "target_id": "target", "width_m": 200.0,
       "altitude_floor_m": 0, "altitude_ceiling_m": 50,
       "centerline": [{"x": -50, "y": 0}, {"x": 50, "y": 0}],
       "valid_time": {"recurrence": {"start_time": "12:00", "end_time": "12:01"}}}],
     datetime(2026, 10, 12, 11, 59, 58), []),
)


def test_the_shared_window_memo_answers_as_each_rule_alone():
    """Rules on one weekly window read it once per pose (`_plain_key`). Two
    rules may share only when their in-force state is the same function of
    time: same start, end AND days, and neither the target of a switch or
    swap. Each case breaks one of those, at a window edge, with the sharing
    rule judged first; the answer must be the per-rule one, which the cases
    pin and the memo-less Shield gives. A memo keyed without the days, or
    without the end, misses the target (a P0 miss); one shared with a
    switched rule reports the target switched off, or misses it switched on,
    and one shared with a swapped corridor reports it after it was replaced
    (reviewers' mutants, which passed every other test). A first version of
    this test put the sharer far from the forecast, where only its first
    pose time is read, and a key without the end survived it."""
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=3.0, vy=3.0)
    for name, cons, clock, want in _MEMO_CASES:
        pol = Policy.model_validate({"policy_id": "memo", "constraints": cons})
        got = [v.rule_id for v in Shield(pol.model_copy(deep=True), escalation=False,
                                         now=lambda c=clock: c)._check(st, a)]
        restore = _no_memo()
        try:
            alone = [v.rule_id for v in Shield(pol.model_copy(deep=True), escalation=False,
                                               now=lambda c=clock: c)._check(st, a)]
        finally:
            restore()
        assert alone == want, (name, "the case does not test what it says", alone)
        assert got == want, (name, got, want)


def test_the_50_rule_edge_load_answers_as_each_rule_alone():
    """The bench's edge variant (every rule's window ends 17:30, the clock
    17:29:57): 400 `_check` answers and 100 `filter()` decisions with the
    shared memo equal those without it, decision for decision."""
    from test_shield import _fifty_variant
    pairs = _samples(500, seed=21)
    edge = datetime(2026, 10, 7, 17, 29, 57)        # the bench's EDGE_NOW, a Wednesday
    now = lambda: edge                                          # noqa: E731
    fast = Shield(_fifty_variant("edge"), now=now)
    got_c = [[v.model_dump() for v in fast._check(st, a)] for st, a in pairs[:400]]
    got_f = [fast.filter(st, a, t=0.1 * k).model_dump() for k, (st, a) in enumerate(pairs[400:])]
    restore = _no_memo()
    try:
        slow = Shield(_fifty_variant("edge"), now=now)
        want_c = [[v.model_dump() for v in slow._check(st, a)] for st, a in pairs[:400]]
        want_f = [slow.filter(st, a, t=0.1 * k).model_dump()
                  for k, (st, a) in enumerate(pairs[400:])]
    finally:
        restore()
    assert sum(map(bool, want_c)) > 100, "the samples see too few violations to compare"
    bad = [k for k, (x, y) in enumerate(zip(got_c, want_c)) if x != y]
    assert not bad, f"{len(bad)} of 400 _check answers differ, first sample {bad[0]}"
    bad = [k for k, (x, y) in enumerate(zip(got_f, want_f)) if x != y]
    assert not bad, f"{len(bad)} of 100 decisions differ, first sample {bad[0]}"


def regenerate_decisions() -> Path:
    got = _replay_decisions()
    if got is None:
        raise SystemExit("the 8 Oct flight is neither in demo/out nor in tests/fixtures")
    touched, clean, src = got
    doc = {"what": "Shield.filter decisions on the 8 Oct follow (rail_shield, 5 s at "
                   "0.1 s, CityLife 6-14 m map), rounded to 1e-6; regenerated by "
                   "tests/test_shieldcore_identity.py --regenerate-decisions",
           "source": src, "clean_ticks": clean,
           "touched": {str(k): v for k, v in sorted(touched.items())}}
    with open(DECISIONS, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(doc, separators=(",", ":")) + "\n")
    return DECISIONS


if __name__ == "__main__":
    if "--regenerate-decisions" in sys.argv:
        print(f"wrote {regenerate_decisions()}")
        sys.exit(0)
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
