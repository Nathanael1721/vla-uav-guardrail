"""tools/wp1_roundtrip_kpi.py: the WP1 round-trip KPI and the columns that give
it meaning.

Run either way:
    pytest tests/test_wp1_roundtrip_kpi.py -v
    python tests/test_wp1_roundtrip_kpi.py

A round-trip score alone cannot fail: a loader that checks nothing passes every
round trip. So these tests are mostly about the tool's ability to say NO - a
tampered bundle refused, a broken policy refused, an empty corpus refused
rather than reported as 0/0 - and about one deliberately crippled loader that
must score the stated null instead of a perfect run.
"""
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import wp1_roundtrip_kpi as W                                      # noqa: E402
from guardrail.models import load_policy                           # noqa: E402

SKIP = "SKIP"
N_POLICIES = len(list((ROOT / "policies").glob("*.yaml")))


def test_every_policy_round_trips_and_every_tamper_is_refused():
    rt = W.round_trip(ROOT / "policies", signer=None)
    assert rt["n"] == N_POLICIES and N_POLICIES >= 20, rt["n"]
    bad = [r for r in rt["rows"] if not r.get("ok")]
    assert not bad, bad
    t = rt["tamper_refusal"]
    assert t["run"] == rt["n"] and t["refused"] == t["run"], t
    assert rt["signatures"] == {"unsigned": N_POLICIES}, (
        "written unsigned on purpose here; the column must say so")


def test_the_negative_corpus_headers_say_what_the_loader_really_does():
    """Each case declares `currently: refused` or `currently: ACCEPTED`. If a
    later change tightens validation, the header must be updated with it - so
    the list of accepted-although-broken policies can never go stale quietly."""
    neg = W.negative_corpus(W.NEGATIVE_DIR)
    assert neg["n"] >= 20, neg["n"]
    for r in neg["rows"]:
        assert r["defect"], f"{r['file']} does not say what is wrong with it"
        assert r["declared_matches"], (
            f"{r['file']}: header says {r['declared']}, loader {r['verdict']}")
    accepted = {x["file"] for x in neg["accepted_findings"]}
    # The three the unit was asked to probe, recorded as findings, not fixed.
    assert {"misspelled_rule_key.yaml", "bowtie_polygon.yaml",
            "duplicate_rule_id.yaml"} <= accepted, accepted
    for r in neg["rows"]:
        if r["verdict"] == "ACCEPTED":
            assert r["finding"], f"{r['file']}: accepted with no finding written"
    assert 0 < neg["refused"] < neg["n"], (
        "a corpus that is all-refused or all-accepted is not testing the line")


def test_a_loader_that_checks_nothing_scores_the_null_not_a_pass():
    """Cripple the tool's checks and require the score to collapse to the null.
    If it did not, the refusal columns would be decoration."""
    saved_load, saved_check = W.load_policy, W.check_bundle
    any_policy = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")

    class _Pass:
        def __init__(self, pol):
            self.policy, self.signature = pol, "unsigned"

    W.load_policy = lambda path: any_policy
    try:
        neg = W.negative_corpus(W.NEGATIVE_DIR)
        assert neg["refused"] == 0, neg["refused"]
    finally:
        W.load_policy = saved_load
    W.check_bundle = lambda path, *a, **k: _Pass(any_policy)
    try:
        rt = W.round_trip(ROOT / "policies", signer=None)
        assert rt["tamper_refusal"]["refused"] == 0, rt["tamper_refusal"]
    finally:
        W.check_bundle = saved_check


def test_nothing_to_measure_is_refused_not_reported_as_zero_of_zero():
    empty = Path(tempfile.mkdtemp(prefix="wp1empty_"))
    for fn in (lambda: W.round_trip(empty), lambda: W.negative_corpus(empty)):
        try:
            fn()
        except SystemExit:
            continue
        raise AssertionError("an empty directory produced a score")


def test_the_report_carries_its_null_and_its_provenance():
    out = Path(tempfile.mkdtemp(prefix="wp1out_")) / "wp1.json"
    assert W.main(["--out", str(out), "--no-runs"]) == 0
    res = json.loads(out.read_text(encoding="utf-8"))
    for k in ("generated", "command", "code_revision", "python",
              "cryptography_available", "signature_verifier", "kpi",
              "null_accept_everything",
              "version_lock_problems", "schema", "schema_validation"):
        assert k in res, k
    n = res["round_trip"]["n"]
    assert res["kpi"]["round_trip"] == f"{n}/{n}"
    assert res["null_accept_everything"]["tamper_refusal"] == f"0/{n}"
    assert res["version_lock_problems"] == []
    assert res["schema"]["current"] is True


def test_the_stored_run_census_finds_every_run():
    sr = W.stored_run_census(ROOT)
    if not sr.get("n"):
        return SKIP                         # demo/out is gitignored
    assert sr["matched_after"] == sr["n"], sr["unmatched_after"]
    assert sr["matched_before"] < sr["matched_after"], sr
    # The five ArduPilot SITL + MAVROS 2 runs; `dev` topology, so not named
    # KPI-grade (the 2026-10-06 relabel).
    assert "kpi_grade_five" not in sr, "the census still calls the dev runs KPI-grade"
    five = sr["mavros_dev_five"]
    if five:
        assert not any(v["matched_before"] for v in five.values()), five
        assert all(v["matched_after"] for v in five.values()), five


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
