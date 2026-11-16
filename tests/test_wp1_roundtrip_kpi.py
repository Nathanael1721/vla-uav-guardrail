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
import atexit
import contextlib
import json
import shutil
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
# What the cold start's refusal checks must all say (bad_signature since the
# T1d review, 2026-10-20).
REFUSED = {"tampered_ir": "refused", "untrusted_signer": "refused", "bad_signature": "refused"}

# Every temporary folder a test makes, removed when the interpreter exits
# (either runner): the T1d review found these tests leaving them behind.
_TMP: list[Path] = []
atexit.register(lambda: [shutil.rmtree(d, ignore_errors=True) for d in _TMP])


def _tmp(prefix: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix=prefix))
    _TMP.append(d)
    return d


def gb_unsigned() -> str:
    import guardrail.bundle as gb
    return gb.UNSIGNED


def test_every_policy_round_trips_and_every_tamper_is_refused():
    rt = W.round_trip(ROOT / "policies", signer=None)
    assert rt["n"] == N_POLICIES and N_POLICIES >= 20, rt["n"]
    bad = [r for r in rt["rows"] if not r.get("ok")]
    assert not bad, bad
    t = rt["tamper_refusal"]
    assert t["run"] == rt["n"] and t["refused"] == t["run"], t
    assert rt["signatures"] == {"unsigned": N_POLICIES}, (
        "written unsigned on purpose here; the column must say so")
    # ...and so must who signed and the signer's certificate chain (WP1-04):
    # nobody, and none - never a chain the run did not establish.
    assert rt["signed_by"] == [gb_unsigned()] and rt["chains"] == {"none": N_POLICIES}, (
        rt["signed_by"], rt["chains"])
    assert all(r["chain"] == "none" and r["signed_by"] == gb_unsigned() for r in rt["rows"])


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
    empty = _tmp("wp1empty_")
    for fn in (lambda: W.round_trip(empty), lambda: W.negative_corpus(empty)):
        try:
            fn()
        except SystemExit:
            continue
        raise AssertionError("an empty directory produced a score")


def test_the_report_carries_its_null_and_its_provenance():
    out = _tmp("wp1out_") / "wp1.json"
    assert W.main(["--out", str(out), "--no-runs", "--no-reference",
                   "--cold-start-runs", "3", "--cold-start-fresh", "1"]) == 0
    res = json.loads(out.read_text(encoding="utf-8"))
    for k in ("generated", "command", "code_revision", "python",
              "cryptography_available", "signature_verifier", "kpi",
              "null_accept_everything", "grant_form", "reference",
              "version_lock_problems", "schema", "schema_validation", "cold_start"):
        assert k in res, k
    # 2026-10-19: the cold start is in the KPI table, with its reading named.
    assert "fresh process" in res["kpi"]["cold_start_50_rules"], res["kpi"]
    assert res["cold_start"]["verdict"]["checks_hold"] is True, res["cold_start"]["checks"]
    n = res["round_trip"]["n"]
    assert res["kpi"]["round_trip"] == f"{n}/{n}"
    assert res["kpi"]["grant_form_round_trip"] == f"{n}/{n}"
    assert res["null_accept_everything"]["tamper_refusal"] == f"0/{n}"
    assert res["version_lock_problems"] == []
    assert res["schema"]["current"] is True
    assert res["kpi"]["reference_hash_agrees"] == "not run", (
        "--no-reference must say the reference was not run, never a score")
    # Review of the T1d branch (P7): `command` alone hid the flags the
    # published file was made with. Shown failing at 044f0fc: no
    # command_args / command_env keys. The scratch --out is not a path a
    # reader has, so it is reduced to its name.
    assert res["command"] == "python tools/wp1_roundtrip_kpi.py", res["command"]
    assert res["command_args"] == ["--out", "<elsewhere>/wp1.json", "--no-runs",
                                   "--no-reference", "--cold-start-runs", "3",
                                   "--cold-start-fresh", "1"], res["command_args"]
    assert set(res["command_env"]) == {"VLAGUARD_SIGNING_KEY"}, res["command_env"]


def test_paths_in_the_provenance_name_the_checkout_not_the_machine():
    """_provenance_path: inside this checkout, relative; inside another
    checkout of this repository, under "<main checkout>" (it holds a .git
    folder) or "<another worktree>" (a .git file); elsewhere, the name only."""
    assert W._provenance_path(W.ROOT / "policies") == "policies"
    assert W._provenance_path(W.ROOT) == "."
    for git_is_dir, label in ((True, "<main checkout>"), (False, "<another worktree>")):
        other = _tmp("wp1prov_") / "VLA Drone"
        (other / "tools").mkdir(parents=True)
        (other / "tools" / "wp1_roundtrip_kpi.py").write_text("", encoding="utf-8")
        (other / "guardrail").mkdir()
        if git_is_dir:
            (other / ".git").mkdir()
        else:
            (other / ".git").write_text("gitdir: elsewhere\n", encoding="utf-8")
        ref = other / "kuanting-vla-uav-guardrail"
        ref.mkdir()
        assert W._provenance_path(other) == label
        assert W._provenance_path(ref) == f"{label}/kuanting-vla-uav-guardrail"
    assert W._provenance_path(_tmp("wp1prov_") / "x.json") == "<elsewhere>/x.json"


def test_the_tool_leaves_no_temporary_folder_behind():
    """Review of the T1d branch (P7): cold_start never removed its wp1cold_*
    folder, and round_trip / both_hashes / reference_cross_load left theirs
    until the interpreter's end. cold_start now removes its own when it
    returns (and when it raises); the others are registered for exit."""
    import glob
    import os
    pattern = os.path.join(tempfile.gettempdir(), "wp1cold_*")
    before = set(glob.glob(pattern))
    W.cold_start(warm_runs=1, fresh_runs=0)
    assert set(glob.glob(pattern)) <= before, sorted(set(glob.glob(pattern)) - before)
    real = W._cold_start_policies

    def broken():
        raise RuntimeError("fixture build failed")
    W._cold_start_policies = broken
    try:
        W.cold_start(warm_runs=1, fresh_runs=0)
    except RuntimeError:
        pass
    finally:
        W._cold_start_policies = real
    assert set(glob.glob(pattern)) <= before, "a failed cold start left its folder"


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


def test_every_manifest_names_both_hashes_and_a_forged_one_is_refused():
    """2026-10-08 (deviation 10): each policy's bundle records this project's
    hash AND the reference's (null, with reasons, where the reference cannot
    hold it); both are verified; `policy_hash` did not move; a forged
    reference hash is refused - and the measured null (a reader that ignores
    the record, as 10451bc did) accepts every forgery, so the column can fail."""
    bh = W.both_hashes(ROOT / "policies")
    assert bh["n"] == N_POLICIES, bh["n"]
    bad = [r for r in bh["rows"] if not r.get("ok")]
    assert not bad, bad
    assert bh["forged_run"] == bh["n"] and bh["forged_refused"] == bh["n"], bh
    assert bh["null_record_ignored_refused"] == 0, bh["null_record_ignored_refused"]
    # Every shipped policy that is not representable says why.
    for r in bh["rows"]:
        assert r["reference_hash"] or r["reference_problems"], r
    assert sum(bh["not_representable_because"].values()) >= bh["n"] - bh["representable"]


def test_the_report_carries_the_both_hash_columns_and_their_null():
    out = _tmp("wp1bh_") / "wp1.json"
    assert W.main(["--out", str(out), "--no-runs", "--no-reference", "--no-cold-start"]) == 0
    res = json.loads(out.read_text(encoding="utf-8"))
    # Skipped is said, never scored.
    assert res["kpi"]["cold_start_50_rules"] == "not run", res["kpi"]
    assert res["kpi"]["cold_start_within_budget"] == "not run", res["kpi"]
    n = res["round_trip"]["n"]
    assert res["kpi"]["both_hashes_recorded_and_verified"] == f"{n}/{n}"
    assert res["kpi"]["forged_reference_hash_refused"] == f"{n}/{n}"
    assert res["null_record_ignored"]["forged_reference_hash_refused"] == f"0/{n}"
    x = res["reference"]
    if x.get("run"):
        docs = x["our_loader_on_reference_documents"]
        assert res["kpi"]["reference_form_source_verified"] == f"{len(docs)}/{len(docs)}"


# --------------------------------------------------------------------------- #
# cold start (2026-10-19): the grant's "cold-start ingest of a typical 50-rule
# bundle <= 1 s" (Policy DSL page, Acceptance KPIs). On 2026-10-20 the desktop
# measured, in fresh interpreters (medians, the published docs/data/wp1_roundtrip.json),
# imports ~344 ms + bundle -> verified policy ~7-12 ms + Shield ~4-5.5 ms, the
# KPI reading p95 376 ms (379 ms on 2026-10-19), whole process ~440 ms
# (docs/DESIGN-wp1-cold-start.md).
# --------------------------------------------------------------------------- #

def test_the_cold_start_fits_the_grants_budget_and_proves_it_did_the_work():
    """A signed 50-rule bundle -> load_for_flight -> rail_shield, for the
    typical mixed WGS84 bundle and the bench grid, in this process and in
    fresh interpreters. The budget is pinned on the path itself (first call in
    a fresh process, no imports) everywhere, and imports included on the
    desktop; a shared CI runner (GITHUB_ACTIONS) gets 2 s for the
    import-inclusive reading, as the 50-rule monitor test gets 15 ms there,
    and the numbers are printed so one green run says what the runner does."""
    import os
    cs = W.cold_start(warm_runs=10, fresh_runs=3)
    assert set(cs["bundles"]) == {"typical", "grid"}, cs["bundles"]
    for name, b in cs["bundles"].items():
        assert b["rules"] == 50, (name, b)
    typ = cs["bundles"]["typical"]
    assert typ["geographic"] and typ["scheduled_rules"] >= 15, typ
    assert len(typ["by_type"]) >= 7, "the typical bundle must mix rule classes"
    # Proof of work: verified, the issued hash, every zone compiled, a fence
    # that answers, and the path can say no.
    for name, c in cs["checks"].items():
        assert c["runs_failing_a_check"] == 0, (name, c)
        assert c["fresh_runs_failing_a_check"] == 0, (name, c)
        assert c["fences_compiled"] == cs["bundles"][name]["zones"] > 0, (name, c)
        assert c["geofence_raised_inside_first_fence"], (name, c)
        assert "verified" in c["verifier"], (name, c)
    assert "pure-python" in cs["pure_python_verifier"]["typical"]["verifier"]
    assert cs["refusals"] == REFUSED, cs["refusals"]
    assert cs["author_side"]["typical"]["same_policy_hash"] is True
    on_ci = os.environ.get("GITHUB_ACTIONS") == "true"
    bound = 2000.0 if on_ci else W.COLD_START_BUDGET_MS
    for name, fr in cs["fresh_process"].items():
        assert fr["path_first_call"]["p95_ms"] <= W.COLD_START_BUDGET_MS, (name, fr)
        assert fr["ingest_with_imports"]["p95_ms"] <= bound, (name, fr)
        assert cs["in_process"][name]["total"]["p95_ms"] <= W.COLD_START_BUDGET_MS
    v = cs["verdict"]
    assert v["checks_hold"] is True and v["budget_ms"] == 1000.0, v
    if not on_ci:
        assert v["within_budget"] is True, v
    print("      cold start p95: " + ", ".join(
        f"{n} imports {fr['import']['p95_ms']:.0f} + path "
        f"{fr['path_first_call']['p95_ms']:.0f} ms (wall {fr['whole_process_wall']['p95_ms']:.0f})"
        for n, fr in cs["fresh_process"].items()) + f" (bound {bound:g} ms)")


def test_a_cold_start_that_skips_the_work_is_not_within_budget():
    """The null: a loader that verifies nothing, and a Shield that compiles no
    zone, are both FASTER than the real path. Each must read as a failure
    (checks_hold False, within_budget False, and the loader accepting the
    tampered bundle named), never as a better time."""
    import guardrail.bundle as gb
    import guardrail.replay as gr
    from guardrail.models import Policy

    def no_checks(bundle, trust_store=None, **kw):
        import tarfile
        with tarfile.open(bundle, "r:gz") as tar:
            ir = tar.extractfile("ir.json").read()
        pol = Policy.model_validate(json.loads(ir))
        return pol, {"signature": "unsigned", "loaded_policy_hash": pol.policy_hash}

    saved = gb.load_for_flight
    gb.load_for_flight = no_checks
    try:
        cs = W.cold_start(warm_runs=3, fresh_runs=0)
    finally:
        gb.load_for_flight = saved
    assert cs["verdict"]["checks_hold"] is False, cs["verdict"]
    assert cs["verdict"]["within_budget"] is False, cs["verdict"]
    assert cs["refusals"]["tampered_ir"] == "ACCEPTED", cs["refusals"]
    assert all(c["runs_failing_a_check"] > 0 for c in cs["checks"].values())

    real_shield = gr.rail_shield

    def zoneless(policy, **kw):
        keep = [c for c in policy.constraints
                if c.type not in ("polygon_fence", "circle_fence", "dynamic_nfz")]
        return real_shield(policy.model_copy(update={"constraints": keep}), **kw)
    gr.rail_shield = zoneless
    try:
        cs = W.cold_start(warm_runs=3, fresh_runs=0)
    finally:
        gr.rail_shield = real_shield
    assert cs["verdict"]["within_budget"] is False, cs["verdict"]
    for name, c in cs["checks"].items():
        assert c["fences_compiled"] == 0 and not c["geofence_raised_inside_first_fence"], c
    assert cs["refusals"] == REFUSED, cs["refusals"]

    # Review of the T1d branch (P3): a loader that checks the hash and the
    # signer's identity but never verifies the Ed25519 signature. The real
    # load_for_flight runs (hash, both hash forms, lint, flight gate); only
    # the verification is gone, so the tampered IR is still refused (hash)
    # and the untrusted signer too (trust-store lookup). Shown failing at
    # 044f0fc: this cold start reported checks_hold True and within_budget
    # True, because nothing it refused needed the signature checked.
    real_verify = gb.verify_signature

    def identity_only(signed_by, sig_hex, data, trust_store=None):
        trust = (trust_store if isinstance(trust_store, dict)
                 else gb.load_trust_store(trust_store))
        if signed_by not in trust:
            return gb.SIG_UNTRUSTED, f"{signed_by!r} is not in the trust store"
        return gb.SIG_VERIFIED, f"Ed25519 signature by {signed_by!r} verified (never checked)"
    gb.verify_signature = identity_only
    try:
        cs = W.cold_start(warm_runs=3, fresh_runs=0)
    finally:
        gb.verify_signature = real_verify
    assert cs["refusals"]["tampered_ir"] == "refused", cs["refusals"]
    assert cs["refusals"]["untrusted_signer"] == "refused", cs["refusals"]
    assert cs["refusals"]["bad_signature"] == "ACCEPTED", cs["refusals"]
    assert all(c["runs_failing_a_check"] == 0 for c in cs["checks"].values()), cs["checks"]
    assert cs["verdict"]["checks_hold"] is False, cs["verdict"]
    assert cs["verdict"]["within_budget"] is False, cs["verdict"]


def test_the_bad_signature_refusal_is_the_signature_check_and_nothing_else():
    """The bad_signature fixture differs from the issued bundle in one hex
    digit of its Ed25519 line: the hash, the manifest and the signer are as
    issued, and the real loader refuses it with the verification's own words
    (a refusal for any other reason would not show the signature is checked,
    and is not counted as one)."""
    import tarfile
    import guardrail.bundle as gb
    tmp = _tmp("wp1badsig_")
    signer = W._cold_start_signer()
    store = W._trust_store_file(tmp / "trusted_signers.json", signer)
    pol = W._cold_start_policies()["typical"][0]
    good = gb.write_bundle(pol, tmp / "good.tar.gz", changelog="t",
                           issued_at="2026-01-01T00:00:00Z", signer=signer)
    bad = tmp / "bad.tar.gz"
    W._flip_signature(good, bad)

    def members(p):
        with tarfile.open(p, "r:gz") as tar:
            return {m.name: tar.extractfile(m).read() for m in tar.getmembers()}
    g, b = members(good), members(bad)
    assert g.keys() == b.keys() and g[gb.IR_NAME] == b[gb.IR_NAME]
    assert g[gb.MANIFEST_NAME] == b[gb.MANIFEST_NAME]
    sg, sb = g[gb.SIGNATURE_NAME].decode(), b[gb.SIGNATURE_NAME].decode()
    assert sum(x != y for x, y in zip(sg, sb)) == 1 and len(sg) == len(sb), (sg, sb)
    gb.load_for_flight(good, trust_store=store)
    for engine in ("cryptography", "pure-python"):
        with (W._PurePythonVerifier() if engine == "pure-python" else contextlib.nullcontext()):
            try:
                gb.load_for_flight(bad, trust_store=store)
            except gb.BundleSignatureError as e:
                raise AssertionError(f"refused as unverified, not as a bad signature: {e}")
            except ValueError as e:
                assert "verification failed" in str(e), (engine, e)
            else:
                raise AssertionError(f"{engine}: a flipped signature digit was accepted")


def test_a_reference_repository_elsewhere_is_named_as_in_the_checkout():
    """--reference-repo (a git worktree has no gitignored reference repo):
    a missing virtualenv there is reported, not a ValueError from
    Path.relative_to(ROOT) - which is what the note computed before."""
    saved = W.REFERENCE_REPO, W.REFERENCE_PY
    elsewhere = _tmp("wp1ref_elsewhere_")
    W.REFERENCE_REPO, W.REFERENCE_PY = elsewhere, elsewhere / ".venv" / "Scripts" / "python.exe"
    try:
        r = W.run_reference("print(1)")
    finally:
        W.REFERENCE_REPO, W.REFERENCE_PY = saved
    assert r["run"] is False, r
    assert r["note"] == ("no reference virtualenv at kuanting-vla-uav-guardrail/"
                         ".venv/Scripts/python.exe"), r


def test_a_run_set_aside_is_listed_not_dropped():
    """Shown failing on the census code of 2026-10-07 06:30 (no `set_aside`):
    two runs another unit moved into demo/out/_hot_apply_not_in_lock/ fell
    out of the census glob, and the census rose from 83/84 to 85/85 with
    nothing said. A set-aside run is not counted, and it is listed with
    whether it resolves."""
    root = _tmp("wp1census_")
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


def test_a_rest_flown_run_resolves_through_its_own_generation_chain():
    """The 29 Oct ROS 2 rail flights changed their policy over REST in flight;
    their manifests record the last generation's hash, which no policy file
    or lock recipe rebuilds. They resolve through their own policy_g<N>.json
    chain, and only from a g0 that is a known policy."""
    from guardrail.bundle import policy_candidates
    from guardrail.models import policy_from_raw
    cands = policy_candidates()
    base = load_policy(ROOT / "policies" / "sim_demo_policy.yaml", runtime=False)
    # What the run writer writes (guardrail.replay EpisodeRecord.write_generation:
    # the canonical IR) - with the policy's `origin` since the file is WGS84
    # (2026-11-03); a dump of the rules alone would lose the frame.
    g0 = json.loads(base.canonical_bytes())
    g0["generation"] = 0
    if policy_from_raw(g0, runtime=False).policy_hash != base.policy_hash:
        return SKIP                      # the dump form drifted from the run writer's
    g1 = json.loads(json.dumps(g0))
    g1["generation"] = 1
    g1["constraints"][0]["margin_m"] = float(g1["constraints"][0].get("margin_m", 0.0)) + 2.0
    h1 = policy_from_raw(g1, runtime=False).policy_hash
    d = Path(tempfile.mkdtemp())
    try:
        (d / "policy_g0.json").write_text(json.dumps(g0), encoding="utf-8")
        (d / "policy_g1.json").write_text(json.dumps(g1), encoding="utf-8")
        label = W.run_generation_chain(d, h1, cands)
        assert label and "policy_g1" in label and "sim_demo_policy" in label, label
        assert W.run_generation_chain(d, "sha256:" + "0" * 64, cands) is None
        # a g0 that is no known policy resolves nothing, whatever the chain holds
        g0b = json.loads(json.dumps(g0))
        g0b["constraints"][0]["margin_m"] = 77.0
        (d / "policy_g0.json").write_text(json.dumps(g0b), encoding="utf-8")
        assert W.run_generation_chain(d, h1, cands) is None
    finally:
        shutil.rmtree(d, ignore_errors=True)


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
