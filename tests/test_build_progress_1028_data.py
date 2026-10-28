"""tools/deck/build_progress_1028_data.py: the 28 Oct progress deck's numbers.

Synthetic flights in a temporary folder (the real ones are gitignored), so this
runs on CI. What is checked is what the deck would get wrong silently:
  * the score's basis - an end-to-end flight from its first acquisition
    (frac_within_30m_after_acquire), every other flight from the start gate
    (frac_within_30m);
  * which folders count - an aborted re-run and the Jetson's earlier
    orin_follow_* flights do not, and 27 Oct's control flight (whose scripted
    answers never parsed) counts as the hand-written follow, not as a
    fixed-12 m control;
  * the per-zone counts - inside a zone's polygon, in its margin ring, or
    outside, for the drone and for the car.
Shown failing first: with the bases swapped, or the aborted run counted, or a
point inside a zone counted in its margin, the assertions below fail.
"""
import importlib.util
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _mod():
    spec = importlib.util.spec_from_file_location(
        "build_progress_1028_data", ROOT / "tools" / "deck" / "build_progress_1028_data.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _flight(out: Path, tag: str, metrics: dict, e2e: bool = False, log=None):
    d = out / tag
    d.mkdir(parents=True)
    (d / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
    (d / "kpi.json").write_text(json.dumps({"p0_violation_escape_rate": 0.0, "p0_violation_ticks": 3}),
                                encoding="utf-8")
    if e2e:
        (d / "vla_steps.jsonl").write_text("{}\n", encoding="utf-8")
    if log is not None:
        (d / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in log), encoding="utf-8")
    return d


def test_the_score_basis_follows_the_kind_of_flight():
    M = _mod()
    with tempfile.TemporaryDirectory() as tmp:
        M.OUT = Path(tmp)
        _flight(M.OUT, "e2e", {"frac_within_30m": 0.5, "frac_within_30m_after_acquire": 0.2}, e2e=True)
        _flight(M.OUT, "hier", {"frac_within_30m": 0.9, "frac_within_30m_after_acquire": 0.1,
                                "vla_hierarchy": {}}, e2e=True)
        _flight(M.OUT, "manual", {"frac_within_30m": 0.95})
        got = {t: M.score(M.load(t)[0], t) for t in ("e2e", "hier", "manual")}
    assert got == {"e2e": 0.2, "hier": 0.9, "manual": 0.95}, got


def test_only_the_comparison_flights_count_and_the_failed_control_is_the_controller():
    M = _mod()
    with tempfile.TemporaryDirectory() as tmp:
        M.OUT = Path(tmp)
        base = {"frac_within_30m": 0.9, "collisions": {"mission": 0}}
        _flight(M.OUT, "cmp2_e2e_r1_20261028", {"frac_within_30m_after_acquire": 0.1}, e2e=True)
        _flight(M.OUT, "cmp2_e2e_r1_20261028_aborted", {"frac_within_30m_after_acquire": 0.0}, e2e=True)
        _flight(M.OUT, "orin_follow_02", base)
        _flight(M.OUT, "cmp_hier_fixed12_20261027", base)
        _flight(M.OUT, "cmp2_hier_fixed12_20261028", {**base, "vla_hierarchy": {}})
        _flight(M.OUT, "orin_hier_dronecats_20261028", {**base, "vla_hierarchy": {}})
        rows = {r["tag"]: r for r in M.matrix()}
    assert set(rows) == {"cmp2_e2e_r1_20261028", "cmp_hier_fixed12_20261027",
                         "cmp2_hier_fixed12_20261028", "orin_hier_dronecats_20261028"}, sorted(rows)
    assert rows["cmp_hier_fixed12_20261027"]["kind"] == "manual"
    assert rows["cmp2_hier_fixed12_20261028"]["kind"] == "fixed12"
    assert rows["orin_hier_dronecats_20261028"]["platform"] == "Jetson"
    assert (rows["cmp2_e2e_r1_20261028"]["kind"], rows["cmp2_e2e_r1_20261028"]["score"]) == ("pure", 0.1)


def test_zone_counts_separate_inside_margin_and_outside():
    M = _mod()
    with tempfile.TemporaryDirectory() as tmp:
        M.OUT = Path(tmp) / "out"
        pol = Path(tmp) / "p.yaml"
        pol.write_text("policy_id: t\nversion: 0.1.0\nconstraints:\n"
                       "  - id: z\n    type: polygon_fence\n    margin_m: 1.0\n"
                       "    vertices: [{x: 0, y: 0}, {x: 10, y: 0}, {x: 10, y: 10}, {x: 0, y: 10}]\n",
                       encoding="utf-8")
        car = lambda x, y: {"class": "car", "pts": [[x, y]]}
        log = [{"x": 5, "y": 5, "truth": car(5, 5)},        # inside
               {"x": 10.5, "y": 5, "truth": car(20, 5)},    # margin ring
               {"x": 15, "y": 5, "truth": car(-0.5, 5)},    # outside (car: margin)
               {"x": 30, "y": 30}]                          # outside, no truth
        _flight(M.OUT, "nfz", {}, log=log)
        z = M.zone_report("nfz", pol)["z"]
    assert (z["drone_ticks_inside"], z["drone_ticks_in_margin"]) == (1, 1), z
    assert z["drone_min_dist_m"] < 0 and z["drone_ticks_within_12m"] == 3, z
    assert (z["car_ticks_inside"], z["car_ticks_in_margin"]) == (1, 1), z


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            import traceback
            traceback.print_exc()
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
