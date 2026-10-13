"""The KPI report's WP4 outputs (tools/kpi_report.py, guardrail/kpi.py), 2026-10-12.

Run either way:
    pytest tests/test_wp4outputs_kpi_report.py -v
    python tests/test_wp4outputs_kpi_report.py

WHAT EACH TEST GUARDS, AND HOW IT WAS SHOWN TO FAIL FIRST

Every test below fails on the pristine 4418ae0 copy of the tree (run with
PYTHONPATH pointing at a `git archive HEAD` extract):

  * KPI grade needs the replay bundle as evidence. The fixture's manifest
    PASSES `is_kpi_grade` today (hil topology with valid hil_evidence, a clean
    revision, sim_speedup 1.0 - the test asserts that first), so the only
    thing that differs between the graded and the refused cases is the
    replay bundle beside the run: signed by a trusted key, unsigned, signed
    by a key the trust store does not hold, or absent. At 4418ae0 all four
    were KPI-grade.
  * The mean repair magnitude per priority and per operator class, with a
    priority or class that never acted reported as such (None), and a
    Shield-off arm carrying no breakdown at all.
  * A zero whose event could not happen (no P0 tick to escape) is named in
    rollup()'s `vacuous_zeros` and printed as vacuous; its control, a family
    WITH P0 ticks and no escape, still prints the measured 0.
  * WP1 renders whatever keys the round-trip file holds, nulls beside.
  * WP2 finds the budget where the naive cut drops a P0 rule, on a policy
    folder of its own, and writes docs/data/csp_kpis.json's shape.
  * The tick-budget section reads stored profiles and the flown logs.
  * A/B and per-paraphrase robustness are n/m until a pilot that reads the
    prompt flies them, with the headless count beside; measured once it does.
  * --baseline lists new, fixed and gone failures and per-family deltas.

Added 2026-10-13 (review of the 2026-10-12 version). Each fails on
4418ae0, where none of this exists; all but the unknown-rule / sole-pooling
test also fail on the 2026-10-12 version of the tools (that one pins two
behaviours a deliberate break got past, not a defect):

  * The evidence is the run's own: a signed bundle of another flight, or a
    folder log edited after packing, demotes a graded run; the run's own
    bundle beside a newer foreign one still grades it, and a log whose line
    endings a checkout converted is the same log.
  * A report written anywhere leaves docs/data/csp_kpis.json alone: WP2 is
    read from it (and says so) unless --wp2 recomputes it, and it is
    written only by --wp2-only or an explicit --csp-json with --wp2.
  * guardrail/repair.py's PathRepair is counted under the grant's PathRepair
    class; an operator no class lists gets an Unclassified column.
  * A rule missing from the priority table counts as P0 in the breakdown;
    the per-class `sole_*` figures pool from sums; a run whose policy did
    not resolve keeps its class table and says the priority half is n/m.
  * The tick budget: nearest-rank p50 / p99, a 100.0 ms tick is within
    budget, a rail keeps its worst episode, and the Shield's columns count
    only the episodes that logged `shield_ms`.
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from guardrail import bundle as B                                  # noqa: E402
from guardrail import kpi as K, load_policy                        # noqa: E402
from guardrail.manifest import is_kpi_grade                        # noqa: E402
import kpi_report as KR                                            # noqa: E402

SKIP = "SKIP"
POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
A0 = {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0}

# Evidence a Jetson Orin in the hil topology records (the shape
# tests/test_manifest.py's ORIN_HIL passes is_kpi_grade with).
GOOD_HIL = {"ros_distro": "jazzy", "mavros_node": "/mavros", "fcu_connected": True,
            "ardupilot_version": "ArduCopter V4.5.7 (2a3dc4b7)"}
ORIN_HIL = {**GOOD_HIL, "host_arch": "aarch64",
            "device_model": "NVIDIA Jetson AGX Orin Developer Kit",
            "l4t_release": "R36.4.3", "jetpack": "6.2", "autopilot_kind": "sitl",
            "network_link": {"fcu_url": "udp://:14555@192.168.10.2:14550",
                             "peer": "192.168.10.2", "peer_is_loopback": False,
                             "iface": "eth0", "link_kind": "ethernet"}}


def _tmp(prefix: str) -> Path:
    return Path(tempfile.mkdtemp(prefix=prefix))


def _row(i, *, vios=(), reps=(), raw=None, em=None, em_vios=()):
    raw = raw or dict(A0, vx=4.0)
    return {"t": round(0.1 * i, 3), "tick": i, "x": 0.4 * i, "y": 0.0, "up": 15.0,
            "raw": raw, "emitted": em or raw, "violations": list(vios),
            "emitted_violations": list(em_vios), "repairs": list(reps),
            "braked": False, "unsafe": False, "shield_ms": 1.0 + 0.01 * i}


def _hil_run(root: Path, name: str, pol) -> Path:
    """A flown run whose manifest passes is_kpi_grade: hil with the Orin's
    evidence, sim_speedup 1.0, a clean revision, every field resolved."""
    run = root / name
    run.mkdir(parents=True)
    rows = [_row(i) for i in range(1, 21)]
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    metrics = {"shield": "on", "hil_evidence": ORIN_HIL}
    (run / "metrics.json").write_text(json.dumps(metrics), encoding="utf-8")
    manifest = {"code_revision": "4418ae0abcde", "vla_model_hash":
                "guardrail.vla_stub.StubVLA@src:0123456789abcdef",
                "policy_hash": pol.policy_hash, "random_seed": 0, "sim_speedup": 1.0,
                "topology": "hil"}
    (run / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (run / "kpi.json").write_text(json.dumps(K.compute(rows, K.rule_priorities(pol),
                                                       metrics)), encoding="utf-8")
    return run


def _signer(d: Path):
    trust = d / "trust.json"
    if B.HAVE_CRYPTO:
        return B.generate_signing_key(d / "k.pem", authority="test-lab",
                                      trust_store=trust), trust
    signer = B.Signer(authority="test-lab",
                      private_key=B._PyEd25519TestKey(os.urandom(32)))
    B.register_public_key(signer, trust)
    return signer, trust


@contextlib.contextmanager
def _trust_store(path: Path):
    """The default trust store, pointed at `path` for the test (the variable
    guardrail.bundle reads), so the check runs exactly as the report runs it."""
    old = os.environ.get("VLAGUARD_TRUST_STORE")
    os.environ["VLAGUARD_TRUST_STORE"] = str(path)
    try:
        yield
    finally:
        if old is None:
            os.environ.pop("VLAGUARD_TRUST_STORE", None)
        else:
            os.environ["VLAGUARD_TRUST_STORE"] = old


# --------------------------------------------------------------------------- #
# KPI grade: the manifest AND the replay bundle as evidence
# --------------------------------------------------------------------------- #

def test_a_graded_manifest_with_an_unsigned_or_untrusted_bundle_is_not_kpi_grade():
    from guardrail.replay import write_replay
    pol = load_policy(POLICY)
    root = _tmp("kr_evidence_")
    signer, trust = _signer(root / "keys")
    stranger, _ = _signer(root / "stranger")         # a key the trust store lacks
    cases = {}
    for name, sig in (("signed", signer), ("unsigned", None), ("untrusted", stranger),
                      ("no_bundle", "none")):
        run = _hil_run(root, name, pol)
        if sig != "none":
            write_replay(run, pol, run / f"{name}.replay.tar.gz", signer=sig)
        cases[name] = run
    # The precondition that makes this a real test: the manifest alone passes.
    man = json.loads((cases["unsigned"] / "manifest.json").read_text(encoding="utf-8"))
    met = json.loads((cases["unsigned"] / "metrics.json").read_text(encoding="utf-8"))
    ok, why = is_kpi_grade(man, met)
    assert ok, f"the fixture must pass is_kpi_grade on its manifest: {why}"
    by_hash = {pol.policy_hash: pol}
    with _trust_store(trust):
        eps = {n: KR.load_run(r, by_hash, {}) for n, r in cases.items()}
    assert eps["signed"]["kpi_grade"] is True, eps["signed"]["kpi_grade_reasons"]
    for n in ("unsigned", "untrusted", "no_bundle"):
        e = eps[n]
        assert e["kpi_grade"] is False, (n, e["kpi_grade_reasons"])
        why = " ".join(e["kpi_grade_reasons"])
        assert "replay bundle" in why, (n, why)
    assert "UNSIGNED" in " ".join(eps["unsigned"]["kpi_grade_reasons"])
    assert eps["no_bundle"]["evidence_bundle"] is None
    # The manifest's own verdict is kept beside the demoted one.
    assert all(e.get("kpi_grade_manifest") is True for e in eps.values())
    # ... and the report tabulates only the signed run as KPI-grade.
    with _trust_store(trust):
        rep = KR.build_report([str(root / "*")], [], 10, False, "cmd")
    assert rep["counts"]["kpi_grade"] == 1, rep["kpi_grade_episodes"]
    assert [e["id"] for e in rep["kpi_grade_episodes"]] == ["signed"]
    shutil.rmtree(root, ignore_errors=True)


def test_the_evidence_must_be_the_runs_own_bundle():
    """The signed bundle beside a graded run must hold the log and manifest
    the report scores the run from (bundle_binding)."""
    from guardrail.replay import write_replay
    pol = load_policy(POLICY)
    root = _tmp("kr_bound_")
    signer, trust = _signer(root / "keys")
    by_hash = {pol.policy_hash: pol}

    def run_with_bundle(name, bundle_name=None, x1=None):
        run = _hil_run(root, name, pol)
        if x1 is not None:                     # a flight of its own, before packing
            log = run / "flight_log.jsonl"
            log.write_text(log.read_text(encoding="utf-8").replace(
                '"x": 0.4,', f'"x": {x1},', 1), encoding="utf-8")
        write_replay(run, pol, run / (bundle_name or f"{name}.replay.tar.gz"), signer=signer)
        return run

    # control: the run's own bundle, signed and trusted
    own = run_with_bundle("own")
    # the folder's log edited after packing: two escapes the bundle never saw
    edited = run_with_bundle("edited")
    p0 = {"rule_id": "nfz-square", "category": "geofence"}
    rows = [json.loads(x) for x in (edited / "flight_log.jsonl").read_text(
        encoding="utf-8").splitlines()]
    for r in rows[3:5]:
        r.update(violations=[p0], emitted_violations=[p0])
    (edited / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                             encoding="utf-8")
    # another flight's signed bundle copied into a folder that has none
    other = run_with_bundle("other")
    foreign = _hil_run(root, "foreign", pol)
    (foreign / "flight_log.jsonl").write_text(
        (foreign / "flight_log.jsonl").read_text(encoding="utf-8").replace(
            '"x": 0.4,', '"x": 0.41,'), encoding="utf-8")
    shutil.copy(other / "other.replay.tar.gz", foreign / "copied.replay.tar.gz")
    # the run's own bundle under another name, beside a NEWER foreign one
    both = run_with_bundle("both", "packed.replay.tar.gz", x1=0.42)
    shutil.copy(foreign / "copied.replay.tar.gz", both / "zz_newer.replay.tar.gz")
    os.utime(both / "zz_newer.replay.tar.gz")
    # a checkout that converted the folder's log's line endings (LF <-> CRLF)
    # after packing: the same log
    crlf = run_with_bundle("crlf")
    data = (crlf / "flight_log.jsonl").read_bytes()
    flipped = (data.replace(b"\r\n", b"\n") if b"\r\n" in data
               else data.replace(b"\n", b"\r\n"))
    assert flipped != data
    (crlf / "flight_log.jsonl").write_bytes(flipped)
    with _trust_store(trust):
        eps = {r.name: KR.load_run(r, by_hash, {})
               for r in (own, edited, foreign, both, crlf)}
    assert eps["own"]["kpi_grade"] is True, eps["own"]["kpi_grade_reasons"]
    assert eps["crlf"]["kpi_grade"] is True, eps["crlf"]["kpi_grade_reasons"]
    for n in ("edited", "foreign"):
        e = eps[n]
        assert e["kpi_grade"] is False and e["kpi_grade_manifest"] is True, (n, e)
        why = " ".join(e["kpi_grade_reasons"])
        assert "not this run's evidence" in why and "flight_log.jsonl" in why, (n, why)
    assert eps["edited"]["p0_escapes"] == 2          # the figures it would have graded
    assert eps["both"]["kpi_grade"] is True, eps["both"]["kpi_grade_reasons"]
    assert eps["both"]["evidence_bundle"].endswith("packed.replay.tar.gz")
    assert KR.bundle_binding(own, own / "own.replay.tar.gz") == []
    shutil.rmtree(root, ignore_errors=True)


def test_a_report_written_anywhere_leaves_the_tracked_wp2_file_alone():
    """docs/data/csp_kpis.json is a committed deliverable: a report written to
    any --out (a test's temp folder, a review copy) reads it and says so, and
    never rewrites it with its own provenance."""
    tracked = ROOT / KR.DEFAULT_CSP_JSON
    before = tracked.read_bytes() if tracked.is_file() else None
    d = _tmp("kr_wp2file_")
    runs = d / "runs"
    _hil_run(runs, "r1", load_policy(POLICY))
    stored = {"generated": "2026-10-13", "code_revision": "abc", "command": "python x",
              "default": {"budget_tokens": 256, "p0_coverage": 1.0, "n_compiled": 1,
                          "n_policies": 1, "p0_coverage_null": 0.0,
                          "p0_coverage_baseline": 1.0, "csp_budget_exceeded": 0,
                          "max_tokens_used": 99, "kpi_discriminates": False},
              "scan": [], "scan_step_tokens": 8, "scan_floor_tokens": 16,
              "discriminating": None}
    side = d / "csp.json"
    side.write_text(json.dumps(stored), encoding="utf-8")
    side_bytes = side.read_bytes()
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        assert KR.main(["--runs", str(runs / "*"), "--no-sweep", "--csp-json", str(side),
                        "--out", str(d / "out" / "a")]) == 0
        assert KR.main(["--runs", str(runs / "*"), "--no-sweep",
                        "--out", str(d / "out" / "b")]) == 0
    after = tracked.read_bytes() if tracked.is_file() else None
    assert after == before, f"{KR.DEFAULT_CSP_JSON} was rewritten by a report"
    assert side.read_bytes() == side_bytes, "the WP2 file named for reading was rewritten"
    assert "wrote " + KR.DEFAULT_CSP_JSON not in buf.getvalue(), buf.getvalue()
    # ... and the report still carries WP2, read from the file and saying so
    a = json.loads((d / "out" / "a.json").read_text(encoding="utf-8"))
    assert a["wp2"]["read_from"].endswith("csp.json") and a["wp2"]["code_revision"] == "abc"
    assert "Read from `" in (d / "out" / "a.md").read_text(encoding="utf-8")
    shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------------------------- #
# repair magnitude per priority and per operator class
# --------------------------------------------------------------------------- #

def _breakdown_rows():
    p0 = {"rule_id": "nfz-square", "category": "geofence"}
    p1 = {"rule_id": "kin-caps", "category": "kinematic"}
    slide = {"operator": "GeofenceSlide", "detail": "", "magnitude_m": 1.0,
             "axis": "lateral", "recovery": False}
    clamp = {"operator": "SpeedClamp", "detail": "", "magnitude_m": None,
             "axis": None, "recovery": False}
    rows = [_row(1),
            # a P0 tick repaired by a lateral projection: |dv| = 3.0
            _row(2, vios=[p0], reps=[slide], em=dict(A0, vx=1.0)),
            # a P1-only tick clamped: |dv| = 1.0
            _row(3, vios=[p1], reps=[clamp], raw=dict(A0, vx=5.0), em=dict(A0, vx=4.0)),
            # both at once: the tick is P0, and both classes acted (|dv| = 5.0)
            _row(4, vios=[p0, p1], reps=[clamp, slide], raw=dict(A0, vx=6.0),
                 em=dict(A0, vx=1.0))]
    return rows, {"nfz-square": "P0", "kin-caps": "P1"}


def test_repair_magnitude_is_broken_down_by_priority_and_operator_class():
    rows, prios = _breakdown_rows()
    k = K.compute(rows, prios, {"shield": "on"})
    lv, cl = k["repair_magnitude_by_priority"], k["repair_magnitude_by_operator_class"]
    assert lv["P0"]["ticks"] == 2 and abs(lv["P0"]["mean_mps"] - 4.0) < 1e-9, lv
    assert lv["P1"]["ticks"] == 1 and abs(lv["P1"]["mean_mps"] - 1.0) < 1e-9, lv
    assert lv["P2"] == {"ticks": 0, "sum_mps": 0.0, "mean_mps": None}, lv["P2"]
    lp, kc = cl["LateralProjection"], cl["KinematicClamp"]
    assert (lp["ticks"], lp["sole_ticks"]) == (2, 1) and abs(lp["sole_mean_mps"] - 3.0) < 1e-9
    assert (kc["ticks"], kc["sole_ticks"]) == (2, 1) and abs(kc["sole_mean_mps"] - 1.0) < 1e-9
    assert cl["PathRepair"]["mean_mps"] is None and cl["PathRepair"]["ticks"] == 0
    # the overall mean is unchanged by the breakdown (three repaired ticks),
    # and so is the violation count per level it sits beside
    assert abs(k["mean_repair_magnitude_mps"] - 3.0) < 1e-9
    assert k["violations_by_risk_level"] == {"P0": 2, "P1": 2}, k["violations_by_risk_level"]
    assert k["repair_magnitude_breakdown_status"] == "measured"
    # pooled across episodes from the sums, not averaged
    r = K.rollup([k, K.compute(rows[:2], prios, {"shield": "on"})])
    assert r["mean_repair_magnitude_by_priority"]["P0"] == {"ticks": 3, "mean_mps": 3.6667}
    assert r["mean_repair_magnitude_by_priority"]["P2"]["mean_mps"] is None
    # a Shield-off arm flew none of its logged repairs: no breakdown at all
    off = K.compute(rows, prios, {"shield": "off"})
    assert off["repair_magnitude_by_priority"] is None
    assert off["repair_magnitude_breakdown_status"] == "shield_off"
    assert K.rollup([off])["mean_repair_magnitude_by_priority"] is None
    # and the report prints "never" for a class that never acted
    md = "\n".join(KR._render_magnitudes({"other": {"fam": r}}))
    row = next(line for line in md.splitlines() if line.startswith("| `fam`"))
    assert "never" in row and "4.000 (2)" not in row and "3.667 (3)" in row, row


def test_the_operator_classes_name_the_grants_three_and_mark_the_rest():
    assert K.GRANT_OPERATOR_CLASSES == ("AltitudeClamp", "LateralProjection", "PathRepair")
    assert K.operator_class("GeofenceSlide") == "LateralProjection"
    assert K.operator_class("AltitudeFix") == "AltitudeClamp"
    assert K.operator_class("SomethingNew") == K.UNCLASSIFIED
    md = "\n".join(KR._render_magnitudes({}))
    assert "KinematicClamp*" in md and "| PathRepair |" in md


def test_a_pathrepair_repair_counts_under_pathrepair_and_an_unknown_operator_is_shown():
    """guardrail/repair.py logs Repair(operator="PathRepair"); it is the
    grant's PathRepair class, not Unclassified, and an operator no class lists
    gets a column of its own instead of vanishing from the table."""
    p0 = {"rule_id": "nfz-square", "category": "geofence"}
    path = {"operator": "PathRepair", "detail": "", "magnitude_m": 2.0, "axis": "lateral",
            "recovery": False}
    odd = {"operator": "SomethingNew", "detail": "", "magnitude_m": None, "axis": None,
           "recovery": False}
    rows = [_row(1, vios=[p0], reps=[path], em=dict(A0, vx=1.0)),       # |dv| 3
            _row(2, vios=[p0], reps=[path], em=dict(A0, vx=2.0)),       # |dv| 2
            _row(3, vios=[p0], reps=[odd], em=dict(A0, vx=3.0))]        # |dv| 1
    k = K.compute(rows, {"nfz-square": "P0"}, {"shield": "on"})
    cl = k["repair_magnitude_by_operator_class"]
    assert K.operator_class("PathRepair") == "PathRepair"
    assert cl["PathRepair"]["ticks"] == 2 and abs(cl["PathRepair"]["mean_mps"] - 2.5) < 1e-9, cl
    assert cl[K.UNCLASSIFIED]["ticks"] == 1, cl
    r = K.rollup([k])
    md = "\n".join(KR._render_magnitudes({"other": {"fam": r}}))
    header = next(x for x in md.splitlines() if x.startswith("| Family"))
    row = next(x for x in md.splitlines() if x.startswith("| `fam`"))
    cols = [c.strip() for c in header.strip("|").split("|")]
    cells = [c.strip() for c in row.strip("|").split("|")]
    assert cols[-1] == K.UNCLASSIFIED and cells[-1] == "1.000 (1)", (cols, cells)
    assert cells[cols.index("PathRepair")] == "2.500 (2)", (cols, cells)
    # the null: no unclassified tick, no Unclassified column
    plain = K.rollup([K.compute(rows[:2], {"nfz-square": "P0"}, {"shield": "on"})])
    assert K.UNCLASSIFIED not in "\n".join(KR._render_magnitudes({"other": {"f": plain}}))


def test_an_unknown_rule_is_p0_and_the_sole_figures_pool_from_sums():
    rows, prios = _breakdown_rows()
    ghost = {"rule_id": "rule-nobody-wrote", "category": "standoff"}
    slide = {"operator": "GeofenceSlide", "detail": "", "magnitude_m": 1.0,
             "axis": "lateral", "recovery": False}
    k = K.compute([_row(1, vios=[ghost], reps=[slide], em=dict(A0, vx=2.0))],
                  {"kin-caps": "P1"}, {"shield": "on"})
    lv = k["repair_magnitude_by_priority"]
    assert lv["P0"]["ticks"] == 1 and lv["P2"]["ticks"] == 0, lv
    assert K.governing_priority([ghost], {}) == "P0"
    # two episodes: LateralProjection acted alone on 1 tick (3.0) in the
    # first and on 1 tick (2.0) in the second: pooled sole mean 2.5
    k1 = K.compute(rows, prios, {"shield": "on"})
    r = K.rollup([k1, k])
    lp = r["mean_repair_magnitude_by_operator_class"]["LateralProjection"]
    assert (lp["sole_ticks"], lp["sole_mean_mps"]) == (2, 2.5), lp
    assert (lp["ticks"], lp["mean_mps"]) == (3, round((3.0 + 5.0 + 2.0) / 3, 4)), lp


def test_a_run_whose_policy_did_not_resolve_keeps_its_class_table():
    d = _tmp("kr_unres_")
    run = d / "unres"
    run.mkdir()
    rows, _ = _breakdown_rows()
    (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                          encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({"shield": "on"}), encoding="utf-8")
    (run / "manifest.json").write_text(json.dumps({
        "code_revision": "abc", "vla_model_hash": "stub", "random_seed": 0,
        "policy_hash": "sha256:" + "e" * 64, "sim_speedup": 1.0, "topology": "dev"}),
        encoding="utf-8")
    e = KR.load_run(run, {}, {})
    assert e["repair_magnitude_by_priority"] is None
    assert e["repair_magnitude_by_operator_class"]["LateralProjection"]["ticks"] == 2
    assert e["repair_magnitude_breakdown_status"] == "priorities_unresolved", e[
        "repair_magnitude_breakdown_status"]
    r = K.rollup([e])
    assert r["mean_repair_magnitude_by_priority"] is None
    assert r["mean_repair_magnitude_by_operator_class"]["LateralProjection"]["ticks"] == 2
    assert r["episodes_repair_magnitude_breakdown_not_measurable"] == 0
    assert r["episodes_repair_magnitude_by_priority_not_measurable"] == 1
    md = "\n".join(KR._render_magnitudes({"other": {"fam": r}}))
    row = next(x for x in md.splitlines() if x.startswith("| `fam`"))
    cells = [c.strip() for c in row.strip("|").split("|")]
    # the four priority cells are n/m; the class cells are measured
    assert cells[1:5] == ["n/m (priorities unresolved)"] * 4, cells
    assert cells[6] == "4.000 (2)", cells                   # LateralProjection
    shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------------------------- #
# vacuous zeros
# --------------------------------------------------------------------------- #

def test_a_zero_with_no_p0_tick_is_printed_as_vacuous_and_its_control_is_not():
    rows, prios = _breakdown_rows()
    quiet = K.compute([_row(i) for i in range(1, 6)], prios, {"shield": "on"})
    busy = K.compute(rows, prios, {"shield": "on"})
    rq, rb = K.rollup([quiet]), K.rollup([busy])
    assert rq["p0_escape_rate"] == 0.0 and rq["p0_violation_ticks"] == 0
    assert "p0_escape_rate" in rq["vacuous_zeros"], rq["vacuous_zeros"]
    # control: P0 ticks happened and none escaped - a measured 0
    assert rb["p0_escape_rate"] == 0.0 and rb["p0_violation_ticks"] == 2
    assert "p0_escape_rate" not in rb["vacuous_zeros"]
    table = "\n".join(KR._family_table({"quiet": rq, "busy": rb}))
    q = next(line for line in table.splitlines() if "`quiet`" in line)
    b = next(line for line in table.splitlines() if "`busy`" in line)
    assert "vacuous (no P0 tick)" in q and "0.0000 (0)" not in q, q
    assert "0.0000 (0)" in b and "vacuous" not in b.split("|")[3], b
    nulls = "\n".join(KR._null_table({"quiet": rq}))
    assert "vacuous (no P0 tick) / vacuous (no P0 tick)" in nulls, nulls


def test_a_false_trigger_count_on_logs_without_fsm_state_is_vacuous():
    rows, prios = _breakdown_rows()
    r = K.rollup([K.compute(rows, prios, {"shield": "on"})])
    assert r["false_trigger_rate"] == 0.0 and "false_trigger_rate" in r["vacuous_zeros"]
    with_fsm = [dict(x, fsm_state_before="Normal", fsm_state_after="Normal") for x in rows]
    r2 = K.rollup([K.compute(with_fsm, prios, {"shield": "on"})])
    assert r2["false_trigger_rate"] == 0.0 and "false_trigger_rate" not in r2["vacuous_zeros"]


# --------------------------------------------------------------------------- #
# WP1, WP2, tick budget
# --------------------------------------------------------------------------- #

def test_wp1_renders_whatever_keys_the_round_trip_file_holds():
    d = _tmp("kr_wp1_")
    p = d / "wp1.json"
    p.write_text(json.dumps({
        "generated": "2026-10-08T19:42:47Z", "command": "python tools/wp1_roundtrip_kpi.py",
        "code_revision": "abc", "kpi": {"round_trip": "29/29", "brand_new_figure": "3/4",
                                        "negative_accepted": []},
        "null_accept_everything": {"round_trip": "29/29", "tamper_refusal": "0/29",
                                   "why": "a reader that verifies nothing"},
        "unrelated": {"x": 1}}), encoding="utf-8")
    w = KR.wp1_section(p)
    md = "\n".join(KR._render_wp1(w))
    assert "| `brand_new_figure` | 3/4 |" in md, md
    assert "| `negative_accepted` | none |" in md, md
    assert "| `tamper_refusal` | - | 0/29 |" in md, md
    assert "`null_accept_everything`: a reader that verifies nothing" in md
    assert "unrelated" not in md
    assert "n/m" in "\n".join(KR._render_wp1(KR.wp1_section(d / "missing.json")))
    # the committed file renders every one of its kpi keys
    real = KR.wp1_section(ROOT / KR.DEFAULT_WP1)
    if real is not None:
        md = "\n".join(KR._render_wp1(real))
        assert all(f"| `{k}` |" in md for k in real["kpi"]), md


def test_wp2_reports_the_budget_where_the_naive_cut_drops_a_p0_rule():
    src = ROOT / "policies" / "urban_clearance.yaml"
    if not src.is_file():
        return SKIP
    d = _tmp("kr_wp2_")
    shutil.copy(src, d / src.name)
    w = KR.wp2_section(d)
    assert w["default"]["kpi_discriminates"] is False, w["default"]
    c = w["discriminating"]
    assert c is not None, w["scan"]
    assert c["budget_tokens"] < w["default"]["budget_tokens"]
    assert c["kpi_discriminates"] is True
    assert c["p0_coverage"] > c["p0_coverage_baseline"], c
    lost = c["policies_where_the_naive_cut_drops_p0"]
    assert lost and lost[0]["baseline_p0_missing"] and not lost[0]["compiler_p0_missing"]
    # the scan stops at the first budget that separates them, one step below
    # the last that did not
    assert w["scan"][-1]["budget_tokens"] == c["budget_tokens"]
    assert w["scan"][-2]["kpi_discriminates"] is False
    out = d / "csp.json"
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        assert KR.main(["--wp2-only", "--policies", str(d), "--csp-json", str(out)]) == 0
    doc = json.loads(out.read_bytes())
    assert b"\r\n" not in out.read_bytes()
    assert doc["discriminating"]["budget_tokens"] == c["budget_tokens"]
    assert doc["default"]["source"] == "guardrail.compiler.coverage_report"
    assert "naive cut drops" in buf.getvalue()


def test_tick_timing_counts_ticks_over_the_100_ms_budget():
    rows = [{"t": 0.1 * i, "shield_ms": ms} for i, ms in enumerate([1.0, 2.0, 150.0, 3.0])]
    rows.append({"t": 0.9, "shield_ms": 1.0})                 # a 0.6 s gap
    tt = KR.tick_timing(rows)
    assert tt["shield_ms"]["over_budget"] == 1 and tt["shield_ms"]["max"] == 150.0
    assert tt["period_ms"]["over_budget"] == 1, tt
    assert KR.tick_timing([{"x": 1}]) is None
    d = _tmp("kr_tick_")
    prof = {"schema": "vlaguard.shield_tick_profile/2", "topology": "dev",
            "host": {"machine": "AMD64", "jetson": {"is_orin": False}},
            "headline": {"check_p99_ms": 3.4, "check_budget_ms": 5.0,
                         "check_verdict": "within", "filter_p99_ms": 11.9,
                         "filter_budget_ms": 100.0, "filter_verdict": "within"},
            "host_speed": {"steady": True}, "failures": []}
    (d / "shield_tick_profile.json").write_text(json.dumps(prof), encoding="utf-8")
    t = KR.tick_budget_section(str(d / "shield_tick_profile*.json"),
                               [{"rail": "dev", "tick_timing": tt}])
    assert t["profiles"][0]["within_budget"] is True
    assert t["flown_by_rail"]["dev"]["shield_over_budget"] == 1
    md = "\n".join(KR._render_tick_budget(t))
    assert "| dev | 1 | 5 |" in md and "11.90 / 100" in md, md


def test_the_tick_budget_keeps_the_worst_episode_and_counts_episodes_per_measure():
    # nearest rank over 1..100 ms: p50 is the 50th value, p99 the 99th
    tt = KR.tick_timing([{"t": 0.1 * i, "shield_ms": float(i)} for i in range(1, 101)])
    sh = tt["shield_ms"]
    assert (sh["n"], sh["p50"], sh["p99"], sh["max"]) == (100, 50.0, 99.0, 100.0), sh
    assert sh["over_budget"] == 0, "a 100.0 ms tick is within a 100 ms budget"
    assert KR.tick_timing([{"t": 0.0, "shield_ms": 100.001}])["shield_ms"]["over_budget"] == 1
    assert KR._pct([5.0, 1.0, 3.0], 0.5) == 3.0 and KR._pct([], 0.5) is None
    calm = KR.tick_timing([{"t": 0.1 * i, "shield_ms": 2.0} for i in range(10)])
    spiky = KR.tick_timing([{"t": 0.1 * i, "shield_ms": 2.0 if i else 80.0}
                            for i in range(10)])
    old = KR.tick_timing([{"t": 0.1 * i} for i in range(10)])     # periods only
    t = KR.tick_budget_section(str(_tmp("kr_noprof_") / "none*.json"), [
        {"rail": "r", "tick_timing": calm}, {"rail": "r", "tick_timing": spiky},
        {"rail": "r", "tick_timing": old}])
    r = t["flown_by_rail"]["r"]
    assert (r["shield_worst_p99_ms"], r["shield_max_ms"]) == (80.0, 80.0), r
    assert (r["episodes_with_timing"], r["shield_episodes"], r["period_episodes"]) == \
        (3, 2, 3), r
    assert r["shield_ticks"] == 20 and r["period_ticks"] == 27, r
    md = "\n".join(KR._render_tick_budget(t))
    assert "| r | 2 | 20 | 80.00 | 80.00 | 0 | 3 | 27 |" in md, md


# --------------------------------------------------------------------------- #
# A/B and per-paraphrase robustness
# --------------------------------------------------------------------------- #

def _armed(cell, seed, prefix, reads, ok, pid=None, backend=None):
    return {"id": f"{cell}#{prefix}--seed{seed}", "family": "f", "arm": "on",
            "mission_success": ok, "p0_escapes": 0, "mission_goal_status": "reached",
            "arms": {"cell": cell, "seed": seed, "prefix": prefix, "pilot_reads_text": reads,
                     "paraphrase_id": pid, "backend": backend}}


def test_ab_and_paraphrase_are_nm_until_a_pilot_that_reads_text_flies_them():
    headless = [_armed("c", 1, "on", False, True, "p0", "identity"),
                _armed("c", 1, "off", False, True, "p0", "identity")]
    ab, para = KR.arms_sections(headless)
    assert ab["status"] == "n/m" and ab["headless_pairs"] == 1, ab
    assert para["status"] == "n/m" and para["headless_trials"] == 2, para
    md = "\n".join(KR._render_arms(ab, para))
    assert md.count("n/m") == 2 and "1 headless pairs" in md, md
    flown = [_armed("c", 1, "on", True, True, "p0", "identity"),
             _armed("c", 1, "off", True, False, "p0", "identity"),
             _armed("c", 2, "on", True, True, "p1", "llm:x")]
    ab, para = KR.arms_sections(flown + headless)
    assert ab["status"] == "measured" and ab["pairs"] == 1, ab
    assert ab["delta_on_minus_off"] == 1.0 and ab["headless_pairs"] == 1
    assert para["status"] == "measured" and para["trials"] == 3
    assert para["canonical_trials"] == 2 and para["paraphrase_arms"] == 1


# --------------------------------------------------------------------------- #
# --baseline
# --------------------------------------------------------------------------- #

def _ep(eid, fam, *, esc=0, ok=True):
    return {"id": eid, "family": fam, "arm": "on", "p0_escapes": esc,
            "mission_success": ok, "mission_goal_status": "reached" if ok else "missed",
            "ticks": 10, "p0_violation_ticks": 2}


def test_baseline_lists_new_fixed_and_gone_failures_and_family_deltas():
    base_eps = [_ep("a", "F", esc=1), _ep("b", "F"), _ep("c", "G", ok=False),
                _ep("ctl", "F", esc=3)]
    base_eps[-1]["arm"] = "off"                     # a control arm: never listed
    now_eps = [_ep("a", "F"), _ep("b", "F", esc=2), _ep("d", "G"),
               _ep("e", "F", ok=False)]                  # e: not in the baseline
    base = {"generated": "2026-10-06", "code_revision": "old",
            "families": {"other": {"F": dict(K.rollup(base_eps[:2])),
                                   "G": dict(K.rollup(base_eps[2:3]))}},
            "episodes": base_eps}
    now = {"families": {"other": {"F": dict(K.rollup(now_eps[:2])),
                                  "H": dict(K.rollup(now_eps[2:3]))}},
           "episodes": now_eps, "failures": KR.failure_index(now_eps)}
    b = KR.compare_baseline(now, base, "base.json")
    assert [(r["id"], r["was"]) for r in b["new_failures"]] == [
        ("b", "passing"), ("e", "absent")], b["new_failures"]
    assert [r["id"] for r in b["fixed_failures"]] == ["a"], b["fixed_failures"]
    assert [r["id"] for r in b["gone_failures"]] == ["c"], b["gone_failures"]
    assert "F" in b["family_deltas"]
    d = b["family_deltas"]["F"]["p0_escapes"]
    assert (d["baseline"], d["now"], d["delta"]) == (1, 2, 1), d
    # a figure that did not move is not a delta (two episodes then and now)
    assert "episodes" not in b["family_deltas"]["F"], b["family_deltas"]["F"]
    assert b["families_added"] == ["H"] and b["families_removed"] == ["G"]
    md = "\n".join(KR._render_baseline(b))
    assert "2 new failures (1 of episodes the baseline had and did not count as "         "failing, 1 of episodes it did not have), 1 fixed, 1 no longer" in md, md
    assert "### New failures: regressions" in md and "did not have" in md
    assert "| `F` | `p0_escapes` | 1 | 2 | 1 |" in md, md
    # a baseline in the new format compares from its own `failures`
    base2 = dict(base, failures=KR.failure_index(base_eps))
    assert KR.compare_baseline(now, base2, "b2")["new_failures"] == b["new_failures"]


def test_the_cli_compares_against_a_baseline_report():
    d = _tmp("kr_base_")
    runs = d / "runs"
    pol = load_policy(POLICY)
    for name, esc in (("r1", 0), ("r2", 1)):
        run = runs / name
        run.mkdir(parents=True)
        p0 = {"rule_id": "nfz-square", "category": "geofence"}
        rows = [_row(i, vios=[p0], em_vios=[p0] if (esc and i == 3) else [])
                for i in range(1, 6)]
        (run / "flight_log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows),
                                              encoding="utf-8")
        (run / "metrics.json").write_text(json.dumps({"shield": "on"}), encoding="utf-8")
        (run / "manifest.json").write_text(json.dumps({
            "code_revision": "abc", "vla_model_hash": "stub", "random_seed": 0,
            "policy_hash": pol.policy_hash, "sim_speedup": 1.0, "topology": "dev"}),
            encoding="utf-8")
    base = d / "base"
    with contextlib.redirect_stdout(io.StringIO()):
        KR.main(["--runs", str(runs / "*"), "--no-sweep", "--out", str(base)])
    doc = json.loads(base.with_suffix(".json").read_text(encoding="utf-8"))
    assert any(k.endswith("::r2") for k in doc["failures"]), doc["failures"]
    shutil.rmtree(runs / "r2")
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        KR.main(["--runs", str(runs / "*"), "--no-sweep",
                 "--baseline", str(base.with_suffix(".json"))])
    out = buf.getvalue()
    assert "## Against the baseline" in out and "0 new failures (" in out, out
    assert "0 fixed, 1 no longer in the inputs" in out, out
    shutil.rmtree(d, ignore_errors=True)


def test_the_3s_comparison_arm_is_its_own_family_row():
    """A --publish run flies the grant's 5 s forecast and a labelled 3 s
    comparison arm. Until 2026-10-13 the report's family keys ignored the arm
    and pooled the two; the comparison is now its own row."""
    assert KR.forecast_suffix({"kpi": {"forecast_arm": "grant"}}) == ""
    assert KR.forecast_suffix({"kpi": {"forecast_arm": "3s"}}) == "/forecast-3s"
    assert KR.forecast_suffix({"forecast": {"arm": "3s", "grant_config": False}}) == "/forecast-3s"
    assert KR.forecast_suffix({"id": "nfz-head-on#canonical,forecast-3s"}) == "/forecast-3s"
    assert KR.forecast_suffix({}, None) == ""
    grant = KR._bundle_family({"template": "nfz"}, "nfz-head-on", {}, "on")
    cmp_ = KR._bundle_family({"template": "nfz", "kpi": {"forecast_arm": "3s"}},
                             "nfz-head-on", {}, "on")
    assert cmp_ == grant + "/forecast-3s", (grant, cmp_)


def test_a_follow_the_rail_could_not_score_is_not_a_success():
    """The perception rail logs follow_scored False when it has no subject
    ground truth; frac_within_30m and standoff_s are then None, which
    compute() read as "no failure", so the run claimed mission success on a
    stand-off it never measured. It is now not measurable (None), with the
    reason named; a scored or older run is unchanged."""
    rows = [_row(i) for i in range(1, 6)]
    base = K.compute(rows, {}, {"shield": "on"})
    assert base["mission_success"] is True, base["mission_fail_reasons"]
    k = K.compute(rows, {}, {"shield": "on", "follow_scored": False})
    assert k["mission_success"] is None and "follow_not_scored" in k["mission_fail_reasons"]
    assert K.compute(rows, {}, {"shield": "on", "follow_scored": True})["mission_success"] is True
    failed = K.compute(rows, {}, {"shield": "on", "follow_scored": False, "nfz_s": 1.0})
    assert failed["mission_success"] is False, "a measured failure stays a failure"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
