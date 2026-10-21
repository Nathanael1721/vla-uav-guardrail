"""The Dashboard's data: tools/build_dashboard.py -> tracker/data/dashboard.json.

Run either way:
    python tests/test_build_dashboard.py
    pytest tests/test_build_dashboard.py -v

Every test builds from a fixture repository in a temporary folder (runs,
docs/data, cards, updates, a CHANGELOG), so the real tracker data is never
written. Pinned, each because the opposite already happened in this project:
- every KPI number equals what its `source` ("path#/json/pointer") points
  at, and carries an as_of date;
- a KPI with no source is null and "not_measured", never 0, and never green
  (a self-test feeds the builder nothing at all);
- green needs the target met AND KPI-grade AND the null beaten: the fixture's
  0.838 fail-safe value is below its 0.940 never-trigger null, as the
  published one is;
- an OWL-ViT detector, the StubVLA or a hand-written controller in
  vla_model_hash is never a VLA flight (65 of the 99 real manifests name the
  detector); a set-aside "_" folder is skipped; a manifest that names a VLA
  is one only with the server's answers in the run's own records
  (vla_steps.jsonl), else "vla_unverified" (review of 8035d0a, P2);
- KPI-grade is recomputed from the topology: a stored kpi_grade: true on a
  "canonical-hil" run is reported as not KPI-grade (5 real kpi.json files say
  true; the 6 Oct rollup counts 0);
- an MPEG-4 Part 2 `_demo.mp4` is never linked, the H.264 `_phone` copy is,
  and a cached ffprobe answer overrides the name; the quick build runs no
  ffprobe or ffmpeg;
- the needs-review count: cards cited by an update newer than their review;
- `--check` exits 1 when an input is newer than dashboard.json, or when the
  newest CHANGELOG section is newer than the newest card review;
- tracker/data/state.json (the user's board) is never opened;
- no environment value, command line or log text reaches the file;
- the first-VLA-flight milestone turns done only on a real VLA run.
"""
import builtins
import io
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import build_dashboard as BD                                    # noqa: E402

SKIP = "SKIP"
CHANGELOG = """# Changelog

## [Unreleased]

### 2026-10-21 — flights and a wave

#### Added
- x

### 2026-10-14 — earlier

#### Added
- y

## [0.5.1] — 2026-09-14

### Added
- z
"""
SECRET = "SIGNING-KEY-PATH-hunter2"


def _w(p: Path, obj):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(obj if isinstance(obj, str) else json.dumps(obj), encoding="utf-8")


def vla_steps(n=24, digest="3f" * 32, vla_ticks=8):
    """vla_steps.jsonl lines as guardrail/vla_link.py writes them: answers
    the model parsed, accepted by the tracker, with the server's identity."""
    return "".join(json.dumps({
        "schema": "vla_steps/1", "seq": i, "kind": "step", "ok": True, "parse_ok": True,
        "accepted": True, "raw_text": "20 49 49",
        "model": {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
                  "weights_sha256": digest, "precision": "nf4", "attn": "eager", "merged": False},
        "outcome": {"end": "complete", "ticks": {"vla_step": vla_ticks}}}) + "\n" for i in range(n))


def _run(root, tag, vla_hash, topology="projectairsim-ardupilot-pymavlink", kpi=None, metrics=None,
         videos=(), rev="08cd0be9a9af", speed=1.0, when=None, steps=None):
    d = root / "demo" / "out" / tag
    _w(d / "manifest.json", {"code_revision": rev, "vla_model_hash": vla_hash,
                             "policy_hash": "sha256:" + "a" * 64, "random_seed": 7,
                             "sim_speedup": speed, "topology": topology})
    if kpi is not None:
        _w(d / "kpi.json", kpi)
    if metrics is not None:
        _w(d / "metrics.json", metrics)
    if steps is not None:
        _w(d / "vla_steps.jsonl", steps)
    for v in videos:
        (d / v).write_bytes(b"\x00" * 64)
    if when is not None:
        os.utime(d / "manifest.json", (when, when))
    return d


class Fixture:
    """A small repository with everything the builder reads."""

    def __init__(self, empty=False, with_vla=False, vla_configured_only=False):
        self.empty, self.with_vla, self.vla_configured_only = empty, with_vla, vla_configured_only

    def __enter__(self):
        self.root = r = Path(tempfile.mkdtemp(prefix="dash_test_"))
        _w(r / "CHANGELOG.md", CHANGELOG)
        _w(r / "VERSION", "0.5.1\n")
        data = r / "tracker" / "data"
        (data / "cards").mkdir(parents=True)
        self.P = BD.paths(r)
        if self.empty:
            return self
        t0 = time.time() - 10 * 86400
        _run(r, "city_owl", "google/owlvit-base-patch32@cbc355fb36458835", "projectairsim-single-host",
             kpi={"ticks": 10, "p0_escapes": 0, "p0_violation_escape_rate": 0.0,
                  "p0_violation_escape_rate_status": "measured", "kpi_grade": False}, when=t0)
        _run(r, "ros2_shield_on", "guardrail.vla_stub.StubVLA@src:" + "b" * 64, "canonical-hil",
             kpi={"ticks": 20, "kpi_grade": True, "kpi_grade_reasons": []}, when=t0 + 100)
        _run(r, "pas_x_on_20261021", "none:hand-written-controller",
             kpi={"ticks": 1190, "p0_escapes": 0, "p0_violation_escape_rate": 0.0,
                  "p0_violation_escape_rate_status": "measured", "unsafe_p0_ticks": 0, "kpi_grade": False},
             metrics={"shield": "on", "loop_hz": 9.905, "end_reason": "max_s reached",
                      "tick_work_ms": {"median": 11.6, "p95": 23.159, "max": 1028.2}, "frac_within_30m": 0.0,
                      "nfz_s": 0, "det_hz": 2.863, "pilot": {"kind": "none"}},
             videos=("pas_x_on_20261021_demo.mp4", "pas_x_on_20261021_demo_phone.mp4"), when=t0 + 200)
        _run(r, "pas_x_off_20261021", "none:hand-written-controller",
             kpi={"ticks": 564, "p0_escapes": 3, "p0_violation_escape_rate": 0.5,
                  "p0_violation_escape_rate_status": "measured", "unsafe_p0_ticks": 79},
             metrics={"shield": "off", "loop_hz": 9.98, "tick_work_ms": {"median": 1, "p95": 2.0, "max": 3}},
             videos=("pas_x_off_20261021_demo.mp4",), when=t0 + 300)
        _run(r, "_set_aside", "openvla/openvla-7b@47a0ec7f", "hil", when=t0 + 400)
        (r / "demo" / "out" / "pas_x_compare_20261021_phone.mp4").write_bytes(b"\x00" * 64)
        if self.with_vla:
            _run(r, "vla_goto_20261101", "aerialvla:openvla-7b+aero_vla@sha256:" + "c" * 64, "hil",
                 kpi={"ticks": 600}, metrics={"shield": "on"}, when=t0 + 500, steps=vla_steps())
        if self.vla_configured_only:
            # Configured with AerialVLA, a server that never answered: the
            # manifest names the VLA, the run has no record of an answer.
            _run(r, "vla_dead_20261102", "aerialvla:openvla-7b+aero_vla@sha256:" + "c" * 64, "hil",
                 kpi={"ticks": 600}, metrics={"shield": "on"}, when=t0 + 600)
        dd = r / "docs" / "data"
        _w(dd / "scenario_sweep.json", {
            "_rollup": {"_all_shield_on": {
                "episodes": 175, "p0_escape_rate": 0.000133, "p0_escapes": 6, "p0_violation_ticks": 10671,
                "null_passthrough_p0_escape_rate": 0.235693, "mission_success_rate": 0.577143,
                "episodes_mission_scored": 175, "null_hover_mission_success_rate": 0.0,
                "mean_repair_count_per_episode": 96.137, "mean_repair_magnitude_mps": 7.0385,
                "repair_success_rate": 0.993096, "null_always_brake_repair_success_rate": 0.0,
                "mean_time_to_safe_s": 5.847, "time_to_safe_episodes": 32, "time_to_safe_censored": 4}},
            "_failsafe_labels": {"labelled": 167, "correct": 140, "false_triggers": 26, "missed_triggers": 1,
                                 "fail_safe_correctness": 0.838323, "null_never_trigger": 0.94012}})
        _w(dd / "wp1_roundtrip.json", {
            "generated": "2026-10-20T11:06:52Z", "command_env": {"VLAGUARD_TRUST_STORE": SECRET},
            "command": f"python tools/wp1_roundtrip_kpi.py --key {SECRET}",
            "kpi": {"round_trip": "29/29", "negative_refusal": "35/35"},
            "cold_start": {"verdict": {"p95_ms": 375.792, "whole_process_wall_p95_ms": 504.191}},
            "stored_runs": {"n": 97, "matched_after": 97,
                            "set_aside": [{"run": "a", "resolves": False}, {"run": "b", "resolves": False}]}})
        _w(dd / "csp_kpis.json", {"generated": "2026-10-13", "default": {
            "p0_coverage": 1.0, "p0_coverage_null": 0.0, "p0_coverage_baseline": 1.0, "kpi_discriminates": False}})
        _w(dd / "progress_oct.json", {"generated": "2026-10-13T21:27:50Z", "wp3_bench": {"grant_load": {
            "near/_check": {"current": {"p99_ms": 0.745}}, "clear/_check": {"current": {"p99_ms": 0.226}},
            "near/filter": {"current": {"p99_ms": 3.911}}}}})
        cards = [
            {"id": "WP3-15", "area": "WP3", "status": "PARTIAL", "priority": "gate", "updated": "2026-10-14"},
            {"id": "WP3-07", "area": "WP3", "status": "PARTIAL", "priority": "high", "updated": "2026-10-21"},
            {"id": "WP4-06", "area": "WP4", "status": "DONE", "priority": "medium", "updated": "2026-10-08"},
        ]
        _w(data / "cards" / "batch_00.json", cards)
        _w(data / "updates.json", {"schema": 1, "entries": [
            {"seq": 1, "date": "2026-10-19", "kind": "work", "cards": ["WP3-15"], "changes": []},
            {"seq": 2, "date": "2026-10-21", "kind": "work", "cards": [],
             "changes": [{"cat": "flown", "cards": ["WP3-07", "WP4-06"]}]}]})
        _w(data / "content_overview.json", {"timeline": [
            {"start": "2026-09-01", "end": "2026-10-14", "status": "PARTIAL"},
            {"start": "2026-10-01", "end": "2026-12-01", "status": "PARTIAL"},
            {"start": "2026-02-01", "end": "2026-03-01", "status": "DONE"}]})
        _w(data / "todo.json", {"schema": 1, "items": [
            {"id": "VLA-05", "kind": "milestone", "wp": "ARCH", "owner": "user", "status": "open",
             "cards": [], "due": "2026-10-01", "milestone": "vla", "auto": "first_vla_flight", "probe": None,
             "i18n": {"en": {"title": "First VLA flight"}, "id": {"title": "x"}, "zh": {"title": "首次"}}},
            {"id": "VLA-02", "kind": "milestone", "wp": "ARCH", "owner": "agent", "status": "doing",
             "cards": [], "due": None, "milestone": "vla", "auto": None,
             "probe": {"file": "guardrail/vla_link.py", "contains": "class StepTracker"},
             "i18n": {"en": {"title": "Step tracker"}, "id": {"title": "x"}, "zh": {"title": "步階"}}}]})
        _w(data / "state.json", {"cards": {"WP3-15": {"state": "done"}}})     # must never be read
        return self

    def build(self, **kw):
        return BD.build(self.P, quick=True, today="2026-10-21", **kw)

    def __exit__(self, *a):
        shutil.rmtree(self.root, ignore_errors=True)


def resolve(root: Path, source: str):
    path, _, pointer = source.partition("#")
    return BD.ptr(json.loads((root / path).read_text(encoding="utf-8")), pointer)


def expected(root: Path, v: dict) -> float:
    """What a KPI value must equal: its pointer, a "k/n" string there read as
    a fraction, or for a `derived` ratio, its two pointers divided."""
    if v.get("derived"):
        return resolve(root, v["derived"]["k"]) / resolve(root, v["derived"]["n"])
    want = resolve(root, v["source"])
    return BD._frac(want)[0] if isinstance(want, str) else float(want)


def flight(doc, tag):
    return next(f for f in doc["flights"] if f["tag"] == tag)


# ------------------------------------------------------------------ numbers and sources
def test_every_kpi_number_equals_its_source_pointer():
    with Fixture() as f:
        doc = f.build()
        seen = 0
        for k in doc["kpis"]:
            for v in ([k["primary"]] if k["primary"] else []) + k["also"]:
                want = expected(f.root, v)
                assert abs(want - v["value"]) < 1e-12, (k["key"], v["source"], want, v["value"])
                assert v.get("as_of"), (k["key"], "no as_of")
                seen += 1
            if v_null := (k["primary"] or {}).get("null_source"):
                assert resolve(f.root, v_null) == k["primary"]["null"], k["key"]
        assert seen >= 12, f"only {seen} values checked"
        fs = next(k for k in doc["kpis"] if k["key"] == "failsafe_trigger_correctness")
        assert fs["primary"]["as_of"] and fs["primary"]["as_of_basis"] in ("commit", "mtime")


def test_a_kpi_with_no_source_is_not_measured_never_zero():
    with Fixture(empty=True) as f:
        doc = f.build()
        assert doc["kpis"], "no KPI rows at all"
        for k in doc["kpis"]:
            assert k["value"] is None and k["status"] == "not_measured", k
            assert k["meets_target"] is None and k["green"] is False and k["primary"] is None, k
        assert doc["flights"] == [] and doc["vla"]["counts"]["vla"] == 0
        assert doc["vla"]["last_vla_flight"] is None
    with Fixture() as f:
        doc = f.build()
        hot = next(k for k in doc["kpis"] if k["key"] == "hot_apply")
        assert hot["value"] is None and hot["status"] == "not_measured", "hot-apply has no JSON source"


def test_green_needs_the_target_kpi_grade_and_the_null_beaten():
    with Fixture() as f:
        doc = f.build()
        by = {k["key"]: k for k in doc["kpis"]}
        fs = by["failsafe_trigger_correctness"]
        assert fs["beats_null"] is False and fs["meets_target"] is False and not fs["green"]
        cold = by["cold_start"]
        assert cold["meets_target"] is True and cold["kpi_grade"] is False and not cold["green"], \
            "a desktop bench met the target and went green"
        assert by["p0_escape_rate"]["meets_target"] is False      # 0.000133 > 0
        assert not any(k["green"] for k in doc["kpis"]), "no fixture value is KPI-grade"


def test_the_latest_flight_reader_never_takes_a_shield_off_run():
    with Fixture() as f:
        doc = f.build()
        p0 = next(k for k in doc["kpis"] if k["key"] == "p0_escape_rate")
        runs = [a.get("run") for a in p0["also"]]
        assert "pas_x_off_20261021" not in runs and "pas_x_on_20261021" in runs, runs
        tick = next(k for k in doc["kpis"] if k["key"] == "shield_tick")
        assert tick["primary"]["run"] == "pas_x_on_20261021" and tick["value"] == 23.159


def test_the_latest_flight_reader_prefers_the_arm_with_the_change_on():
    """Review of 8035d0a (P5): the reader skipped only shield == "off", so
    the newest Shield-on run, pas_ap_ovf_off_20261021 - the Shield on but
    the own-velocity forecast OFF, the deliberately degraded control arm
    that gave 79 unsafe margin ticks - fed the shipped shield_tick primary
    and the P0 "also" row as "latest Shield-on flight". A run whose A/B
    variant is "off" is taken only when no other Shield-on run has the
    value, and the row names the arm."""
    with Fixture() as f:
        t1 = time.time() - 86400
        _run(f.root, "pas_y_off_20261022", "none:hand-written-controller",
             kpi={"ticks": 900, "p0_escapes": 4, "p0_violation_escape_rate": 0.02,
                  "p0_violation_escape_rate_status": "measured", "unsafe_p0_ticks": 79},
             metrics={"shield": "on", "loop_hz": 9.9, "tick_work_ms": {"median": 9, "p95": 31.5, "max": 50}},
             when=t1)
        doc = f.build()
        by = {k["key"]: k for k in doc["kpis"]}
        assert by["shield_tick"]["primary"]["run"] == "pas_x_on_20261021", by["shield_tick"]["primary"]
        assert "on arm of pas_x_*_20261021" in by["shield_tick"]["primary"]["what"], by["shield_tick"]["primary"]
        assert by["shield_tick"]["primary"]["arm"] == "on"
        runs = [a.get("run") for a in by["p0_escape_rate"]["also"]]
        assert runs == ["pas_x_on_20261021"], runs
        # only the control arm has the value: it is used, and says it is the off arm
        for tag in ("pas_x_on_20261021", "pas_x_off_20261021"):
            shutil.rmtree(f.root / "demo" / "out" / tag)
        doc = f.build()
        tick = next(k for k in doc["kpis"] if k["key"] == "shield_tick")["primary"]
        assert tick["run"] == "pas_y_off_20261022" and tick["arm"] == "off", tick
        assert "off arm" in tick["what"], tick["what"]


def _hil_fixture(f, with_off_arm):
    """Replace the harness sources with clean hil flights (KPI-grade by the
    recomputed rule): the flight readers become the primaries."""
    for p in (f.root / "docs" / "data").glob("*.json"):
        p.unlink()
    shutil.rmtree(f.root / "demo" / "out")
    t1 = time.time() - 3600
    _run(f.root, "hil_g_on_20261103", "none:hand-written-controller", "hil",
         kpi={"ticks": 1200, "p0_escapes": 0, "p0_violation_escape_rate": 0.0,
              "p0_violation_escape_rate_status": "measured"},
         metrics={"shield": "on", "tick_work_ms": {"median": 10, "p95": 41.0, "max": 70}}, when=t1 + 60)
    if with_off_arm:
        _run(f.root, "hil_g_off_20261103", "none:hand-written-controller", "hil",
             kpi={"ticks": 1200, "p0_escapes": 30, "p0_violation_escape_rate": 0.3,
                  "p0_violation_escape_rate_status": "measured"},
             metrics={"shield": "off", "tick_work_ms": {"median": 1, "p95": 2.0, "max": 3}}, when=t1)


def test_a_kpi_with_a_null_goes_green_only_when_it_beats_that_null():
    """Review of 8035d0a (P4): green was `meets_target and kpi_grade and
    beats_null is not False`, so a row with NO null passed, while the
    claim, the README and the page's rule say green needs the null beaten.
    The flight-derived P0 reader never carries a null, so the first clean
    hil flight with zero P0 escapes would have gone green with no null
    comparison although its Shield-off pair (the natural null for P0
    escape) is in the same data. Now every KPI declares its null rule:
    P0 escape, fail-safe, mission success, repair success and P0 coverage
    need their null beaten (a flight's P0 null is its paired Shield-off
    arm); the timing budgets and the tracked counts have no null by
    definition."""
    with Fixture() as f:
        doc = f.build()
        rules = {k["key"]: k["null_rule"] for k in doc["kpis"]}
        assert {k for k, v in rules.items() if v == "required"} == {
            "p0_escape_rate", "failsafe_trigger_correctness", "mission_success_rate",
            "repair_success_rate", "p0_coverage"}, rules
        assert set(rules.values()) == {"required", "none"}
    with Fixture() as f:
        _hil_fixture(f, with_off_arm=False)
        by = {k["key"]: k for k in f.build()["kpis"]}
        p0 = by["p0_escape_rate"]
        assert p0["primary"]["run"] == "hil_g_on_20261103" and p0["meets_target"] and p0["kpi_grade"]
        assert p0["beats_null"] is None and p0["green"] is False, "a P0 row with no null went green"
        tick = by["shield_tick"]
        assert tick["meets_target"] and tick["kpi_grade"] and tick["null_rule"] == "none"
        assert tick["green"] is True, "a timing budget has no null by definition"
    with Fixture() as f:
        _hil_fixture(f, with_off_arm=True)
        doc = f.build()
        p0 = next(k for k in doc["kpis"] if k["key"] == "p0_escape_rate")
        assert p0["primary"]["null"] == 0.3, p0["primary"]
        assert p0["primary"]["null_source"] == "demo/out/hil_g_off_20261103/kpi.json#/p0_violation_escape_rate"
        assert resolve(f.root, p0["primary"]["null_source"]) == 0.3
        assert p0["beats_null"] is True and p0["green"] is True, p0


# ------------------------------------------------------------------ pilots and grades
def test_a_detector_stub_or_hand_written_pilot_is_never_a_vla_flight():
    with Fixture() as f:
        doc = f.build()
        c = doc["vla"]["counts"]
        assert c == {"vla": 0, "vla_unverified": 0, "stub": 1, "detector": 1, "none": 2, "unknown": 0,
                     "missing": 0}, c
        assert doc["vla"]["last_vla_flight"] is None
        assert "_set_aside" not in {r["tag"] for r in doc["flights"]}, "a set-aside folder was read"
        assert flight(doc, "city_owl")["pilot"]["class"] == "detector"
    with Fixture(with_vla=True) as f:
        doc = f.build()
        assert doc["vla"]["counts"]["vla"] == 1
        assert doc["vla"]["last_vla_flight"]["tag"] == "vla_goto_20261101"
        ev = flight(doc, "vla_goto_20261101")["pilot"]["evidence"]
        assert ev["decisions"] == 24 and ev["vla_step_ticks"] == 192 and ev["weights_sha256"] == "3f" * 32
        assert ev["source"] == "demo/out/vla_goto_20261101/vla_steps.jsonl", ev


def test_a_configured_vla_without_the_servers_answers_is_not_a_vla_flight():
    """Review of 8035d0a (P2): the pilot class came from the manifest's
    vla_model_hash alone, a hash the PILOT computes from the configured
    model id and its local weight files. A flight configured with AerialVLA
    whose server never answered counted as a VLA flight, became
    last_vla_flight and closed the first-VLA-flight milestone (VLA-05); the
    first version of this file's with_vla fixture (a manifest hash and kpi
    ticks, nothing else) expected exactly that. Such a run is now
    "vla_unverified", shown apart with its reason, and closes nothing."""
    with Fixture(vla_configured_only=True) as f:
        doc = f.build()
        c = doc["vla"]["counts"]
        assert c["vla"] == 0 and c["vla_unverified"] == 1, c
        assert doc["vla"]["last_vla_flight"] is None and doc["vla"]["first_vla_flight"] is None
        p = flight(doc, "vla_dead_20261102")["pilot"]
        assert p["class"] == "vla_unverified" and p["configured"] == "vla" and p["unverified_reason"] == "no_records", p
        assert doc["vla"]["last_vla_unverified"]["tag"] == "vla_dead_20261102"
        assert doc["vla"]["last_vla_unverified"]["reason"] == "no_records"
        it = {i["id"]: i for i in doc["todo"]["items"]}
        assert it["VLA-05"]["status_effective"] == "open" and it["VLA-05"]["auto_evidence"] is None
        assert all(d["vla"] == 0 for d in doc["timeline"]["flights"])
        # the server's answers arrive in the run folder: now it is a VLA flight
        _w(f.root / "demo" / "out" / "vla_dead_20261102" / "vla_steps.jsonl", vla_steps())
        doc = f.build()
        assert doc["vla"]["counts"]["vla"] == 1 and doc["vla"]["last_vla_flight"]["tag"] == "vla_dead_20261102"
        assert {i["id"]: i for i in doc["todo"]["items"]}["VLA-05"]["status_effective"] == "done"
        # and the run's records are an input of the build (a new line makes it stale)
        assert any(p.name == "vla_steps.jsonl" for p in BD.inputs(f.P))


def test_a_stored_kpi_grade_on_canonical_hil_is_not_kpi_grade():
    with Fixture(with_vla=True) as f:
        doc = f.build()
        r = flight(doc, "ros2_shield_on")
        assert r["stored_kpi_grade"] is True and r["kpi_grade"] is False
        assert "topology:canonical-hil" in r["kpi_grade_reasons"]
        assert sum(1 for x in doc["flights"] if x["kpi_grade"]) == 1      # only the clean hil VLA run
        assert flight(doc, "vla_goto_20261101")["kpi_grade"] is True
    dirty = {"topology": "hil", "sim_speedup": 1.0, "code_revision": "abc-dirty",
             "vla_model_hash": "x@sha256:1", "policy_hash": "sha256:1"}
    ok, why = BD.recompute_kpi_grade(dirty)
    assert not ok and why == ["code_revision"]
    ok, why = BD.recompute_kpi_grade(dict(dirty, code_revision="abc", sim_speedup=4.0,
                                          vla_model_hash="x@unresolved"))
    assert not ok and set(why) == {"sim_speedup", "vla_model_hash"}


# ------------------------------------------------------------------ videos
def test_mpeg4_is_never_linked_and_the_h264_copy_is():
    with Fixture() as f:
        doc = f.build()
        on = flight(doc, "pas_x_on_20261021")
        assert [v["path"] for v in on["videos"]] == ["run/pas_x_on_20261021/pas_x_on_20261021_demo_phone.mp4"]
        assert on["videos"][0]["codec"] == "h264" and on["videos"][0]["basis"] == "name"
        assert flight(doc, "pas_x_off_20261021")["videos"] == [], "an MPEG-4 Part 2 original was linked"
        cmp_ = doc["compares"][0]
        assert cmp_["key"] == "pas_x_compare_20261021" and cmp_["videos"][0]["phone"]
        # which runs a compare stacks is never guessed from its name (the real
        # citylife_motion_compare_20261019 stacks a 13 Oct run): only a sidecar names them
        assert cmp_["runs"] == [], cmp_["runs"]
        _w(f.root / "demo" / "out" / "pas_x_compare_20261021.json",
           {"a": "pas_x_on_20261021", "b": "pas_x_off_20261021"})
        assert f.build()["compares"][0]["runs"] == ["pas_x_on_20261021", "pas_x_off_20261021"]
        # a cached ffprobe answer beats the file name
        key = "pas_x_on_20261021/pas_x_on_20261021_demo_phone.mp4"
        st = (f.root / "demo" / "out" / key).stat()
        BD.write_atomic(f.P["dash"] / "media_cache.json",
                        {key: {"size": st.st_size, "mtime_ns": st.st_mtime_ns, "codec": "mpeg4"}})
        assert flight(f.build(), "pas_x_on_20261021")["videos"] == []


def test_the_quick_build_runs_no_ffprobe_or_ffmpeg():
    calls = []
    real = subprocess.run

    def spy(cmd, *a, **kw):
        calls.append(cmd[0] if isinstance(cmd, (list, tuple)) else cmd)
        return real(cmd, *a, **kw)
    with Fixture() as f:
        subprocess.run = spy
        try:
            f.build()
        finally:
            subprocess.run = real
    media = [c for c in calls if "ffprobe" in str(c).lower() or "ffmpeg" in str(c).lower()]
    assert not media, media


def test_every_git_call_takes_no_optional_lock():
    """Review of 8035d0a (P7): the server's background --quick rebuild runs
    `git status --porcelain` in the user's checkout up to every 30 s while
    the data is stale, and a plain status may take .git/index.lock to
    refresh the index - a concurrent `git commit` or `git checkout` by the
    user then fails with "index.lock exists". Every git call of the build
    passes --no-optional-locks (and GIT_OPTIONAL_LOCKS=0)."""
    calls = []
    real = subprocess.run

    def spy(cmd, *a, **kw):
        if isinstance(cmd, (list, tuple)) and cmd and Path(str(cmd[0])).stem.lower() == "git":
            calls.append((list(cmd), (kw.get("env") or {}).get("GIT_OPTIONAL_LOCKS")))
        return real(cmd, *a, **kw)
    with Fixture() as f:
        subprocess.run = spy
        try:
            f.build()
        finally:
            subprocess.run = real
    assert calls, "the build ran no git at all (the test would prove nothing)"
    assert any("status" in c for c, _ in calls), [c for c, _ in calls]
    bad = [c for c, env in calls if c[1:2] != ["--no-optional-locks"] or env != "0"]
    assert not bad, bad


# ------------------------------------------------------------------ cards, todo, timeline
def test_needs_review_counts_cards_cited_by_a_newer_update():
    with Fixture() as f:
        w = f.build()["wps"]
        by = {a["area"]: a for a in w["areas"]}
        assert by["WP3"]["needs_review"] == ["WP3-15"], by["WP3"]      # WP3-07 was reviewed on 10-21
        assert by["WP4"]["needs_review"] == ["WP4-06"]
        assert w["needs_review_total"] == 2
        assert by["WP3"]["gate_open"] == ["WP3-15"] and by["WP3"]["newest_review"] == "2026-10-21"


def test_the_first_vla_flight_milestone_and_overdue_flags():
    with Fixture() as f:
        doc = f.build()
        it = {i["id"]: i for i in doc["todo"]["items"]}
        assert it["VLA-05"]["status_effective"] == "open" and it["VLA-05"]["overdue"] is True
        assert it["VLA-02"]["probe_ok"] is False
        plan = doc["timeline"]["plan"]
        assert [p["overdue"] for p in plan] == [True, False, False]
    with Fixture(with_vla=True) as f:
        (f.root / "guardrail").mkdir()
        (f.root / "guardrail" / "vla_link.py").write_text("class StepTracker:\n    pass\n", encoding="utf-8")
        it = {i["id"]: i for i in f.build()["todo"]["items"]}
        assert it["VLA-05"]["status_effective"] == "done" and it["VLA-05"]["overdue"] is False
        assert it["VLA-05"]["auto_evidence"]["run"] == "vla_goto_20261101"
        assert it["VLA-02"]["probe_ok"] is True


# ------------------------------------------------------------------ staleness, privacy
def test_check_exits_1_on_an_older_output_or_stale_cards():
    with Fixture() as f:
        args = ["--root", str(f.root), "--quick"]
        assert BD.main(args) == 0 and f.P["out"].is_file()
        old = time.time() - 3600
        os.utime(f.P["out"], (old, old))
        assert BD.main(args + ["--check"]) == 1, "an output older than its inputs passed --check"
        assert BD.main(args) == 0
        # fresh file, but the newest CHANGELOG section (10-21) is newer than the newest card review
        cards = f.root / "tracker" / "data" / "cards" / "batch_00.json"
        cs = json.loads(cards.read_text(encoding="utf-8"))
        for c in cs:
            c["updated"] = "2026-10-14"
        cards.write_text(json.dumps(cs), encoding="utf-8")
        assert BD.main(args) == 0
        why = BD.stale_reasons(f.P)
        assert len(why) == 1 and "newer than the newest card review" in why[0], why
        assert BD.main(args + ["--check"]) == 1
        for c in cs:
            c["updated"] = "2026-10-21"
        cards.write_text(json.dumps(cs), encoding="utf-8")
        assert BD.main(args) == 0 and BD.main(args + ["--check"]) == 0


def test_the_board_state_file_is_never_opened():
    opened = []
    real = (builtins.open, io.open, pathlib.Path.open, pathlib.Path.read_text, pathlib.Path.read_bytes)

    def guard(fn):
        def wrapped(*a, **kw):
            target = a[0]
            if Path(str(target)).name == "state.json":
                opened.append(str(target))
                raise AssertionError("state.json opened")
            return fn(*a, **kw)
        return wrapped
    with Fixture() as f:
        builtins.open, io.open = guard(real[0]), guard(real[1])
        pathlib.Path.open, pathlib.Path.read_text, pathlib.Path.read_bytes = (guard(real[2]), guard(real[3]),
                                                                             guard(real[4]))
        try:
            doc = f.build()
            reasons = BD.stale_reasons(f.P, doc)
            ins = BD.inputs(f.P)
        finally:
            builtins.open, io.open, pathlib.Path.open, pathlib.Path.read_text, pathlib.Path.read_bytes = real
        assert not opened, opened
        assert all(p.name != "state.json" for p in ins) and isinstance(reasons, list)
        # the guard works: it does catch a read of state.json
        builtins.open = guard(real[0])
        try:
            try:
                open(f.root / "tracker" / "data" / "state.json", encoding="utf-8")
                caught = False
            except AssertionError:
                caught = True
        finally:
            builtins.open = real[0]
        assert caught, "the guard would not have noticed"


def test_no_environment_command_or_log_text_reaches_the_file():
    with Fixture() as f:
        (f.root / "demo" / "out" / "pas_x_on_20261021" / "flight_log.jsonl").write_text(
            json.dumps({"tick": 0, "secret": SECRET}) + "\n", encoding="utf-8")
        text = json.dumps(f.build(), ensure_ascii=False)
        assert SECRET not in text and "command_env" not in text and "flight_log" not in text


def test_the_written_file_is_lf_and_atomic():
    with Fixture() as f:
        assert BD.main(["--root", str(f.root), "--quick"]) == 0
        raw = f.P["out"].read_bytes()
        assert b"\r\n" not in raw and json.loads(raw.decode("utf-8"))["meta"]["schema"] == BD.SCHEMA
        assert not list(f.P["out"].parent.glob(".dashboard.json.*")), "a temp file was left"


# ------------------------------------------------------------------ the real data
def test_the_real_build_holds_the_invariants():
    """On this checkout's own data (tracker/data is local-only and demo/out
    gitignored: SKIP on a clone without them). Written to a temp file, never
    to tracker/data/dashboard.json."""
    P = BD.paths(ROOT)
    if not list((P["data"] / "cards").glob("batch_*.json")):
        print("      SKIP: tracker/data/cards is not here (local-only data)")
        return SKIP
    tmp = Path(tempfile.mkdtemp(prefix="dash_real_"))
    try:
        P["out"] = tmp / "dashboard.json"
        P["dash"] = tmp / "dash"
        doc = BD.build(P, quick=True)
        c = doc["vla"]["counts"]
        assert sum(c.values()) == len(doc["flights"]) == doc["meta"]["flights_total"]
        for r in doc["flights"]:
            if r["pilot"]["id"] and "owlvit" in r["pilot"]["id"].lower():
                assert r["pilot"]["class"] == "detector", r["tag"]
            if r["kpi_grade"]:
                assert r["topology"] == "hil", r["tag"]
            for v in r["videos"]:
                assert v["codec"] in BD.PLAYABLE and not v["path"].endswith("_demo.mp4"), v
        for k in doc["kpis"]:
            if k["primary"]:
                src = k["primary"]["source"]
                if not src.startswith("demo/out/"):
                    assert abs(expected(ROOT, k["primary"]) - k["value"]) < 1e-12, (k["key"], src)
            else:
                assert k["value"] is None and k["status"] == "not_measured"
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


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
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
