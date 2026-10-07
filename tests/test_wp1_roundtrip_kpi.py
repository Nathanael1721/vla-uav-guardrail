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
    later change moves validation, the header must move with it - so the list
    of accepted-although-broken policies can never go stale quietly.

    Since 2026-10-06 (strict validation) every case is refused - 14/25 before.
    An all-refused corpus tests the line only beside a positive corpus that
    all LOADS, so that is asserted here too: a loader that refused everything
    would pass the first half and fail the second."""
    neg = W.negative_corpus(W.NEGATIVE_DIR)
    assert neg["n"] >= 25, neg["n"]
    for r in neg["rows"]:
        assert r["defect"], f"{r['file']} does not say what is wrong with it"
        assert r["declared_matches"], (
            f"{r['file']}: header says {r['declared']}, loader {r['verdict']}")
        if r["verdict"] == "refused":
            assert r["reason"], f"{r['file']}: refused without a reason"
    assert neg["accepted_findings"] == [], neg["accepted_findings"]
    # The three the 2026-10-05 unit probed and recorded as findings.
    refused = {r["file"] for r in neg["rows"] if r["verdict"] == "refused"}
    assert {"misspelled_rule_key.yaml", "bowtie_polygon.yaml",
            "duplicate_rule_id.yaml"} <= refused, refused
    for f in sorted((ROOT / "policies").glob("*.yaml")):
        load_policy(f)                     # the line has a side that loads


def test_each_negative_case_is_refused_for_its_own_defect():
    """Refused for the RIGHT reason: a case refused for something else (a typo
    in the fixture, the flight gate) would keep scoring after the check it
    exists for was deleted."""
    want = {
        "bowtie_polygon.yaml": "cross", "collinear_polygon.yaml": "no area",
        "duplicate_rule_id.yaml": "used by 2 rules",
        "duplicate_yaml_key.yaml": "written twice",
        "empty_constraints.yaml": "no rules", "empty_policy_id.yaml": "at least 1 character",
        "infinite_speed_cap.yaml": "finite", "misspelled_rule_key.yaml": "Extra inputs",
        "nan_altitude.yaml": "finite", "negative_margin.yaml": "shrinks the zone",
        "non_semver_version.yaml": "pattern",
        "conflicting_hard_altitude_envelopes.yaml": "no height in common",
        "mixed_frames_in_one_rule.yaml": "mix frames",
        "switch_to_missing_rule.yaml": "names no rule",
        "geometry_and_flat_both.yaml": "both inside `geometry`",
        "grant_and_own_name_both.yaml": "name the same field",
        "layers_merged_mismatch.yaml": "not in layers_merged",
    }
    rows = {r["file"]: r for r in W.negative_corpus(W.NEGATIVE_DIR)["rows"]}
    for name, word in want.items():
        assert name in rows, name
        assert word in (rows[name]["reason"] or ""), (name, rows[name]["reason"])


def test_a_reason_names_the_file_not_the_checkout():
    """Shown failing before 2026-10-07: each reason began with the case's
    absolute path, so a long checkout path pushed the defect past the
    truncation (conflicting_hard_altitude_envelopes lost "no height in
    common" under %TEMP%), and docs/data/wp1_roundtrip.json published local
    paths that changed with every run."""
    neg = W.negative_corpus(W.NEGATIVE_DIR)
    for r in neg["rows"]:
        reason = r["reason"] or ""
        assert str(W.NEGATIVE_DIR) not in reason and \
            W.NEGATIVE_DIR.as_posix() not in reason, (r["file"], reason[:120])
    published = (ROOT / "docs" / "data" / "wp1_roundtrip.json")
    if published.is_file():
        text = published.read_text(encoding="utf-8")
        for local in (str(ROOT), ROOT.as_posix(), str(Path(tempfile.gettempdir())),
                      json.dumps(str(ROOT))[1:-1], json.dumps(tempfile.gettempdir())[1:-1]):
            assert local not in text, f"the published report names {local!r}"


def test_a_loader_that_checks_nothing_scores_the_null_not_a_pass():
    """Cripple the tool's checks and require the score to collapse to the null.
    If it did not, the refusal columns would be decoration."""
    saved_load, saved_check = W.load_policy, W.check_bundle
    any_policy = load_policy(ROOT / "policies" / "sim_demo_policy.yaml")

    class _Pass:
        def __init__(self, pol):
            self.policy, self.signature = pol, "unsigned"

    W.load_policy = lambda path, **kw: any_policy
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
    assert W.main(["--out", str(out), "--no-runs", "--no-reference"]) == 0
    res = json.loads(out.read_text(encoding="utf-8"))
    for k in ("generated", "command", "code_revision", "python",
              "cryptography_available", "signature_verifier", "kpi",
              "null_accept_everything", "grant_form", "reference",
              "version_lock_problems", "schema", "schema_validation"):
        assert k in res, k
    n = res["round_trip"]["n"]
    assert res["kpi"]["round_trip"] == f"{n}/{n}"
    assert res["kpi"]["grant_form_round_trip"] == f"{n}/{n}"
    assert res["null_accept_everything"]["tamper_refusal"] == f"0/{n}"
    assert res["version_lock_problems"] == []
    assert res["schema"]["current"] is True
    assert res["kpi"]["reference_hash_agrees"] == "not run", (
        "--no-reference must say the reference was not run, never a score")


def test_the_grant_form_round_trip_can_fail():
    """Cripple the reload (drop the last rule) and the column must say so."""
    saved = W.policy_from_raw

    def lossy(raw, **kw):
        raw = dict(raw)
        raw["constraints"] = raw["constraints"][:-1] or raw["constraints"]
        return saved(raw, **kw)
    W.policy_from_raw = lossy
    try:
        gf = W.grant_form_round_trip(ROOT / "policies")
    finally:
        W.policy_from_raw = saved
    assert gf["passed"] < gf["n"], gf["passed"]


def test_the_reference_cross_load():
    """Our loader reads all of the reference's documents and its bundle; where
    the reference's environment exists, its own loader agrees on every hash
    and reads the bundles written here. The null is the 2026-10-05 loader."""
    x = W.reference_cross_load(run_reference_loader=True)
    if not x.get("run"):
        return SKIP
    docs = x["our_loader_on_reference_documents"]
    assert docs and all(r["loads"] for r in docs.values()), docs
    assert all(r["loads"] and r["hash_form"] == "reference-policy-dsl-0.1"
               for r in x["our_loader_on_reference_bundles"].values()), x
    assert x["null_2026_10_05"]["documents_loaded"].startswith("0/")
    if not x["reference_loader"].get("ok"):
        return SKIP                         # its 3.11 env is not on this machine
    assert all(x["reference_hash_agrees"].values()), x["reference_hash_agrees"]
    assert all(x["reference_loader_reads_our_reference_form_bundle"].values()), x


def test_a_run_set_aside_is_listed_not_dropped():
    """Shown failing on the census code of 2026-10-07 06:30 (no `set_aside`):
    two runs another unit moved into demo/out/_hot_apply_not_in_lock/ fell
    out of the census glob, and the census rose from 83/84 to 85/85 with
    nothing said. A set-aside run is not counted, and it is listed with
    whether it resolves."""
    root = Path(tempfile.mkdtemp(prefix="wp1census_"))
    good = load_policy(ROOT / "policies" / "sim_demo_policy.yaml").policy_hash
    for rel, h in (("demo/out/flown", good), ("demo/out/_parked/odd", "sha256:" + "0" * 64)):
        (root / rel).mkdir(parents=True)
        (root / rel / "manifest.json").write_text(json.dumps({"policy_hash": h}),
                                                  encoding="utf-8")
    sr = W.stored_run_census(root)
    assert (sr["n"], sr["matched_after"]) == (1, 1), sr
    assert sr["set_aside"] == [{"run": "odd", "folder": "_parked",
                                "policy_hash": "sha256:" + "0" * 64,
                                "resolves": False, "form": None}], sr["set_aside"]


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
