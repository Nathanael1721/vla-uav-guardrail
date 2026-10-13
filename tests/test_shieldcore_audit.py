"""The audit record as a validated Pydantic model (guardrail/audit.py).

Run:  python tests/test_shieldcore_audit.py

  * a record is validated before it is written, and a bad one is refused;
  * the raw and emitted actions are also written in the grant's body-frame
    keys {vx, vy, vz, yaw_rate}, frames.to_body at the tick's heading;
  * a violation a window edge causes is the grant's "time-window" category;
  * repair_attempts keeps its old keys and gains the measured `iterations`,
    the pass that logged each repair (operator AND detail);
    repair_passes lists every operator considered, in every chain run;
  * policies/audit_record.schema.json is the model's schema;
  * the stored audit files read into the model (tests/fixtures/ always;
    every audit*.jsonl one or two folders under demo/out when it is there),
    an old record's derived fields are the ones its writer would have
    written, and the stored 8 Oct audit agrees with the 8 Oct flight log it
    sits beside.

Each test of a new behaviour fails on 4418ae0, where there was no
AuditRecord, no body-frame key and no "time-window" category.
"""
import json
import sys
import tempfile
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import load_policy                                       # noqa: E402
from guardrail.audit import (AuditLogger, AuditRecord, SCHEMA_PATH,      # noqa: E402
                             audit_record_schema, read_audit, upgrade_record)
from guardrail.frames import to_body                                    # noqa: E402
from guardrail.models import Action4D, Policy, State                    # noqa: E402
from guardrail.shield import Shield                                     # noqa: E402

DEMO = ROOT / "policies" / "demo_policy.yaml"
FIXTURES = ROOT / "tests" / "fixtures"
FIX_AUDIT = FIXTURES / "pas_ap_20261008_audit.short.jsonl"
FIX_LEGACY = FIXTURES / "audit_legacy_shapes.jsonl"
DEMO_OUT = ROOT / "demo" / "out"
# Where the legacy-shape fixture's lines come from (regenerate_fixtures):
# run folder, 1-based line numbers of its audit.jsonl. citylife_city line 2
# is the one stored record of an older shape that ends in a fallback
# (ClearanceEscape); no stored record of those shapes holds a Brake or a
# SoftRelax (every audit under demo/out scanned, 2026-10-13).
LEGACY_SOURCES = (("adopt_check", (1, 2)), ("ros2fix_on_dyn_yaw", (1, 2)),
                  ("citylife_city", (2,)))


def _log(decisions, policy=None):
    tmp = Path(tempfile.mkdtemp())
    al = AuditLogger(tmp / "audit.jsonl", policy or "sha256:test")
    for i, d in enumerate(decisions, 1):
        al.log(i, d)
    p = tmp / "audit.jsonl"
    return [json.loads(l) for l in p.read_text(encoding="utf-8").splitlines()] if p.exists() else []


def test_a_record_is_validated_before_it_is_written():
    sh = Shield(load_policy(DEMO))
    d = sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3))
    rec = AuditLogger(Path(tempfile.mkdtemp()) / "a.jsonl", "h").record(1, d)
    assert isinstance(rec, AuditRecord) and rec.record_version == 2
    # A ledger entry with a result nobody defined is refused, and nothing is
    # written - a reader never meets it.
    bad = d.model_copy()
    bad.trace = {**d.trace, "passes": [{"pass": 0, "attempts": [
        {"operator": "X", "result": "maybe"}]}]}
    tmp = Path(tempfile.mkdtemp())
    al = AuditLogger(tmp / "a.jsonl", "h")
    try:
        al.log(1, bad)
        raise AssertionError("an attempt with result 'maybe' was written")
    except ValueError:
        pass
    assert al.records_written == 0
    assert not (tmp / "a.jsonl").exists() or (tmp / "a.jsonl").read_text() == ""
    try:
        AuditRecord.model_validate({**rec.model_dump(by_alias=True), "typo_field": 1})
        raise AssertionError("an undeclared key validated")
    except ValueError:
        pass


def test_the_body_frame_actions_are_to_body_at_the_ticks_heading():
    sh = Shield(load_policy(DEMO))
    st, a = State(x=0, y=15, up=4, yaw_deg=30.0), Action4D(vx=1.3, vy=-0.4, yaw_rate=0.1)
    d = sh.filter(st, a)
    r = _log([d])[0]
    assert r["body_yaw_deg"] == 30.0
    for key, act in (("raw_action_body", d.raw), ("emitted_action_body", d.emitted)):
        want = dict(zip(("vx", "vy", "vz", "yaw_rate"), to_body(act, 30.0)))
        assert r[key] == want, (key, r[key], want)
    # The old world-frame keys are still there, vz_up and all.
    assert r["raw_action"] == {"vx": 1.3, "vy": -0.4, "vz_up": 0.0, "yaw_rate": 0.1}
    # A decision that does not carry the heading (one built by hand) writes
    # no body-frame action rather than one at a guessed heading.
    from guardrail.shield import ShieldDecision, Violation
    hand = ShieldDecision(raw=a, emitted=a, violations=[Violation(
        rule_id="r", category="geofence", detail="", predicted_at_s=0.0)])
    r = _log([hand])[0]
    assert r["raw_action_body"] is None and r["body_yaw_deg"] is None


def test_a_violation_a_window_edge_causes_is_time_window():
    """A zone in force from 07:30, the clock at 07:29:58, the aircraft
    heading in: the forecast enters it after the window opens. Its
    violation is the grant's time-window category; the same flight an hour
    later, the zone already in force, is geometric."""
    pol = Policy.model_validate({"policy_id": "tw", "constraints": [
        {"id": "school", "type": "polygon_fence", "margin_m": 1.0,
         "valid_time": {"recurrence": {"start_time": "07:30", "end_time": "17:30"}},
         "vertices": [{"x": 10, "y": -10}, {"x": 30, "y": -10}, {"x": 30, "y": 10},
                      {"x": 10, "y": 10}]}]})
    st, a = State(x=0.0, y=0.0, up=10.0), Action4D(vx=3.0)
    edge = datetime(2026, 10, 12, 7, 29, 58)                  # a Monday
    later = datetime(2026, 10, 12, 8, 30, 0)
    d_edge = Shield(pol.model_copy(deep=True), now=lambda: edge).filter(st, a)
    d_later = Shield(pol.model_copy(deep=True), now=lambda: later).filter(st, a)
    assert d_edge.trace["window_edge"] == ["school"] and d_later.trace["window_edge"] == []
    r_edge, r_later = _log([d_edge])[0], _log([d_later])[0]
    assert [v["grant_category"] for v in r_edge["violations"]] == ["time-window"]
    assert [v["grant_category"] for v in r_later["violations"]] == ["geometric"]
    assert r_edge["violations"][0]["category"] == "geofence"      # ours, unchanged
    # A CLOSING edge inside the lookahead causes nothing: the zone is in force
    # now (17:29:57) and until 17:30:00, and the forecast from x = 5 m enters
    # its ring at t + 1.3 s, before it closes. Geometric, not time-window.
    closing = datetime(2026, 10, 12, 17, 29, 57)
    d_close = Shield(pol.model_copy(deep=True), now=lambda: closing).filter(
        State(x=5.0, y=0.0, up=10.0), a)
    assert [v.rule_id for v in d_close.violations] == ["school"], d_close.violations
    assert d_close.trace["window_edge"] == [], d_close.trace["window_edge"]
    assert [v["grant_category"] for v in _log([d_close])[0]["violations"]] == ["geometric"]


def test_repair_attempts_keep_their_keys_and_gain_the_measured_iteration():
    pol = load_policy(DEMO)
    sh = Shield(pol)
    r = _log([sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3))], pol)[0]
    for k in ("ts", "policy_hash", "generation", "monitor_tick", "raw_action",
              "violations", "repair_attempts", "emitted_action",
              "fsm_state_before", "fsm_state_after", "repairs", "emitted_violations",
              "braked", "relaxed", "fsm_edge", "fsm_fault", "tick", "episode_id"):
        assert k in r, k
    att = r["repair_attempts"][0]
    assert (att["operator"], att["result"], att["axis"], att["iterations"]) == \
        ("GeofenceSlide", "ok", "lateral", 1), att
    assert r["iterations"] == 1 and r["chain_runs"] == 1 and r["repair_mode"] == "chain"
    ops = [(x["operator"], x["result"]) for p in r["repair_passes"] for x in p["attempts"]]
    assert ("LateralProjection", "ok") in ops and len(ops) == 7, ops
    assert r["violations"][0]["priority"] == "P0"


def test_each_repair_attempt_carries_the_pass_that_logged_it():
    """A tick the chain runs three passes on and still has to rescue
    (edge_patrol.yaml, no map: a stop is illegal, so ClearanceEscape's best
    effort flies). GeofenceSlide logs a repair in pass 1 and ANOTHER, with a
    new detail, in pass 2; ClimbClamp first appears in pass 1. Each entry
    carries its own pass; the fallback, after the loop, carries none and
    sits in the last pass of the ledger. The first version mapped the pass
    by operator name: [0, 0, 1, 1, 1, 1, None]. And corridor_survey.yaml's
    ClimbClamp closing pass 1 after one in pass 0 gets 1, not 0 (wrong on
    1324 of 4257 seeded records before the fix, 0 after)."""
    pol = load_policy(ROOT / "policies" / "edge_patrol.yaml")
    d = Shield(pol).filter(State(x=42.45, y=2.99, up=13.58, yaw_deg=-148.3),
                           Action4D(vx=-1.67, vy=7.95, vz_up=1.56, yaw_rate=-0.3))
    r = _log([d], pol)[0]
    got = [(a["operator"], a["iterations"]) for a in r["repair_attempts"]]
    assert got == [("SpeedClamp", 0), ("AltitudeFix", 0), ("GeofenceSlide", 1),
                   ("GeofenceEscape", 1), ("ClimbClamp", 1), ("GeofenceSlide", 2),
                   ("ClearanceEscape", None)], got
    assert r["iterations"] == 3 and r["chain_runs"] == 1 and r["flown_run"] == 1
    assert [p["pass"] for p in r["repair_passes"]] == [0, 1, 2, 3]
    fb = [(p["pass"], a["operator"]) for p in r["repair_passes"] for a in p["attempts"]
          if a["result"] == "fallback"]
    assert fb == [(3, "ClearanceEscape")], fb
    # A Brake fallback likewise closes the last pass the loop ran.
    pol = load_policy(ROOT / "policies" / "corridor_survey.yaml")
    d = Shield(pol).filter(State(x=-6.67, y=4.66, up=14.34, yaw_deg=158.9),
                           Action4D(vx=3.19, vy=6.02, vz_up=3.54, yaw_rate=-1.44))
    r = _log([d], pol)[0]
    assert r["braked"] and r["iterations"] == 3, (r["braked"], r["iterations"])
    assert [(p["pass"], a["operator"]) for p in r["repair_passes"] for a in p["attempts"]
            if a["result"] == "fallback"] == [(3, "Brake")]
    assert [a["iterations"] for a in r["repair_attempts"]] == [0, 0, 0, 1, None]
    d = Shield(pol).filter(State(x=-22.39, y=-62.64, up=0.01, yaw_deg=-125.5),
                           Action4D(vx=-6.38, vy=-2.18, vz_up=-3.8, yaw_rate=2.25))
    r = _log([d], pol)[0]
    got = [(a["operator"], a["iterations"]) for a in r["repair_attempts"]]
    assert got == [("SpeedClamp", 0), ("ClimbClamp", 0), ("YawClamp", 0),
                   ("CorridorAltitudeFix", 1), ("CorridorReturn", 1), ("ClimbClamp", 1)], got
    # A passthrough ran no loop at all.
    d = Shield(load_policy(DEMO)).filter(State(x=-40, y=-40, up=4), Action4D(vx=0.5))
    assert not d.violations and d.trace["iterations"] == 0 and d.trace["chain_runs"] == 0
    assert d.trace["flown_run"] is None and d.trace["repair_iterations"] == []


def test_every_chain_run_is_on_the_record_when_soft_rules_are_relaxed():
    """Hard band 0-10 m, soft P2 band 20-30 m, soft P1 band 3-8 m, at 8 m
    (tests/test_shield.py `_soft_conflict`). The first run of the repair
    stack cannot satisfy them all and runs its three passes; the second,
    with the P2 rule given up, converges in one. Both runs are on the
    ledger, `iterations` counts all four passes, and the flown run is the
    second. The first version kept only the flown run: chain_runs 2,
    iterations 1."""
    from test_shield import _soft_conflict
    pol = _soft_conflict()
    d = Shield(pol).filter(State(x=0, y=0, up=8), Action4D(vx=1.0))
    r = _log([d], pol)[0]
    assert r["chain_runs"] == 2 and r["flown_run"] == 2, (r["chain_runs"], r["flown_run"])
    assert [(p["run"], p["pass"]) for p in r["repair_passes"]] == \
        [(1, 0), (1, 1), (1, 2), (1, 3), (2, 0), (2, 1)]
    assert r["iterations"] == 4, r["iterations"]
    last = r["repair_passes"][-1]["attempts"][-1]
    assert (last["operator"], last["result"]) == ("SoftRelax", "relaxed")
    assert [(a["operator"], a["result"], a["iterations"]) for a in r["repair_attempts"]] == \
        [("SoftRelax", "relaxed", None)]
    assert AuditRecord.model_validate(r).repair_passes[0].run == 1


def test_the_exported_schema_is_the_models():
    on_disk = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    assert on_disk == audit_record_schema(), (
        "policies/audit_record.schema.json is stale: python -m guardrail.audit schema")
    try:
        import jsonschema
    except ImportError:                     # not a project dependency
        print("      (jsonschema not installed: the records were not validated "
              "against the file, only the file against the model)")
        return
    sh = Shield(load_policy(DEMO))
    r = _log([sh.filter(State(x=0, y=15, up=4), Action4D(vx=1.3))])[0]
    jsonschema.validate(r, on_disk)
    for rec in read_audit(FIX_AUDIT):
        jsonschema.validate(rec.model_dump(by_alias=True), on_disk)


def _stored_audits():
    """Every non-empty audit*.jsonl one or two folders under demo/out (a run
    folder, or a run inside a campaign folder). Not rglob: demo/out holds
    junctions to the flights' frame folders, and a review timed walking them
    at 62 s (this glob and reading every record: about 4 s)."""
    if not DEMO_OUT.is_dir():
        return []
    found = set(DEMO_OUT.glob("*/audit*.jsonl")) | set(DEMO_OUT.glob("*/*/audit*.jsonl"))
    return sorted(p for p in found if p.stat().st_size)


def test_the_stored_audits_read_into_the_model():
    """The fixtures always (the 8 Oct audit's first six records, its five
    escape ticks and its last; two August, one September and two 6-7 Oct
    records of the two older shapes); every audit stored under demo/out when
    present. A shape that did not record emitted_violations reads as None
    (not measured), never as an empty list."""
    recs = read_audit(FIX_AUDIT)
    assert len(recs) == 12 and all(r.record_version == 1 for r in recs)
    assert {r.tick for r in recs} >= {857, 927, 928, 999, 1100}
    legacy = read_audit(FIX_LEGACY)
    assert len(legacy) == 5
    assert [r.emitted_violations is None for r in legacy] == [True, True, False, False, False]
    assert all(r.monitor_tick == r.tick and r.repair_attempts is not None for r in legacy)
    for r in recs + legacy:
        again = AuditRecord.model_validate(json.loads(json.dumps(r.model_dump(by_alias=True))))
        assert again == r
    files = _stored_audits()
    if files:
        n = sum(len(read_audit(p)) for p in files)
        assert n > 0
        print(f"      read {n} records from {len(files)} audit files under demo/out")


def test_an_old_records_derived_fields_are_what_its_writer_would_write():
    """A record of the two older shapes has no repair_attempts and no
    boundary_intersect_at_s: the reader derives them as the writer of the
    third shape did - result "fallback" for Brake and ClearanceEscape,
    "relaxed" for SoftRelax, "ok" otherwise; boundary_intersect_at_s =
    predicted_at_s; the grant's category from ours. The stored September
    record ends in a ClearanceEscape; no stored record of those shapes holds
    a Brake or a SoftRelax, so those two are a synthetic record of the July
    shape."""
    stored = read_audit(FIX_LEGACY)[4]
    assert [(a.operator, a.result) for a in stored.repair_attempts] == \
        [("ClearanceFix", "ok")] * 3 + [("ClearanceEscape", "fallback")]
    v = stored.violations[0]
    assert v.boundary_intersect_at_s == v.predicted_at_s == 0.8835775033671844
    assert (v.category, v.grant_category) == ("clearance", "envelope")
    july = {"ts": "2026-07-30T10:00:00.000+00:00", "policy_hash": "sha256:synthetic",
            "tick": 7, "raw_action": {"vx": 1.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
            "violations": [{"rule_id": "z", "category": "geofence", "detail": "",
                            "predicted_at_s": 1.5},
                           {"rule_id": "band", "category": "altitude", "detail": "",
                            "predicted_at_s": 0.0}],
            "repairs": [{"operator": "GeofenceSlide", "detail": "z: removed 1.00 m/s"},
                        {"operator": "SoftRelax", "detail": "soft rule(s) band given up"},
                        {"operator": "Brake", "detail": "repair not converged -> stop"}],
            "emitted_action": {"vx": 0.0, "vy": 0.0, "vz_up": 0.0, "yaw_rate": 0.0},
            "braked": True}
    rec = AuditRecord.model_validate(upgrade_record(july))
    assert [(a.operator, a.result, a.reason) for a in rec.repair_attempts] == [
        ("GeofenceSlide", "ok", "z: removed 1.00 m/s"),
        ("SoftRelax", "relaxed", "soft rule(s) band given up"),
        ("Brake", "fallback", "repair not converged -> stop")]
    assert [(v.boundary_intersect_at_s, v.grant_category) for v in rec.violations] == \
        [(1.5, "geometric"), (0.0, "envelope")]
    assert rec.emitted_violations is None and rec.record_version == 1
    assert rec.monitor_tick == rec.tick == 7


def test_the_stored_8_oct_audit_agrees_with_its_flight_log():
    """Every record of the short audit names a tick of the flight-log
    fixture with the same raw action, violations and emitted action - the
    two artefacts of one flight, read through the compatibility reader."""
    import gzip
    import test_flight_pas_ap_20261008 as FL
    with gzip.open(FL.FIX_LOG, "rt", encoding="utf-8") as fh:
        rows = {r["tick"]: r for r in (json.loads(l) for l in fh if l.strip())}
    for rec in read_audit(FIX_AUDIT):
        row = rows[rec.tick]
        assert rec.raw_action.model_dump() == row["raw"], rec.tick
        assert rec.emitted_action.model_dump() == row["emitted"], rec.tick
        assert [v.rule_id for v in rec.violations] == [v["rule_id"] for v in row["violations"]]
        assert all(v.boundary_intersect_at_s == v.predicted_at_s for v in rec.violations)


def test_upgrading_keeps_every_stored_value():
    for line in FIX_LEGACY.read_text(encoding="utf-8").splitlines():
        raw = json.loads(line)
        up = upgrade_record(raw)
        for k, v in raw.items():
            if k == "violations":
                assert [{kk: vv for kk, vv in x.items() if kk in y} for x, y in
                        zip(up[k], v)] == v
            elif k != "emitted_violations":
                assert up[k] == v, k
        assert isinstance(up["repair_attempts"], list)


def regenerate_fixtures() -> Path:
    lines = []
    for run, nums in LEGACY_SOURCES:
        src = (DEMO_OUT / run / "audit.jsonl").read_text(encoding="utf-8").splitlines()
        lines += [src[n - 1] for n in nums]
    with open(FIX_LEGACY, "w", encoding="utf-8", newline="\n") as fh:
        fh.writelines(l + "\n" for l in lines)
    return FIX_LEGACY


if __name__ == "__main__":
    if "--regenerate-fixtures" in sys.argv:
        print(f"wrote {regenerate_fixtures()}")
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
