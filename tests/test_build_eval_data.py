"""tools/build_eval_data.py: the numbers file the September report and decks read.

Run either way:
    pytest tests/test_build_eval_data.py -v
    python tests/test_build_eval_data.py

WHY THIS FILE EXISTS

Three things in the generator were only asserted by inspection until the
review of 2026-10-06:

  * `--no-tests` carries the last measured test count forward. Before
    2026-10-06 it dropped `repo.tests_fast`, and the mid-evaluation report
    generator then died with KeyError.
  * The file is CRLF in the repository; a rebuild that flips the endings is a
    whole-file diff that hides the values that changed.
  * A rebuild printed TODAY's counts under the September title: 71 shielded
    flights and "2 of 63 measurable", where the corrected September figures
    are 41 and "0 of 34 (33 measurable)" (correction note rows A3 and A15).
    `--as-of` (default 2026-09-14) now counts only flights on disk by then,
    and reads the test count and the hand-typed status records from the
    commit of that day.

Every test here fails on the 401305a generator (no carry-forward, no
--as-of, no line-ending rule) or on a mutant of the current one.
"""
import datetime as dt
import json
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import build_eval_data as B                                         # noqa: E402


def _tmp() -> Path:
    return Path(tempfile.mkdtemp())


def _with_as_of(value):
    """Set the module's cut-off for one test and restore it."""
    class _Ctx:
        def __enter__(self):
            self.old = B.AS_OF
            B.AS_OF = value
        def __exit__(self, *exc):
            B.AS_OF = self.old
    return _Ctx()


# --------------------------------------------------------------------------- #

def test_no_tests_carries_the_last_count_forward_with_its_date():
    d = _tmp()
    prev = d / "eval.json"
    prev.write_text(json.dumps({"generated": "2026-09-29",
                                "repo": {"tests_fast": {"passed": 553, "total": 553}}}),
                    encoding="utf-8")
    tf = B._previous_tests_fast(prev)
    assert (tf["passed"], tf["total"]) == (553, 553), tf
    assert tf["carried_forward"] is True and tf["measured"] == "2026-09-29", tf
    assert "2026-09-29" in tf["note"], tf
    # a count that already says when it was measured keeps that date
    prev.write_text(json.dumps({"generated": "2026-10-06",
                                "repo": {"tests_fast": {"passed": 1, "total": 2,
                                                        "measured": "2026-09-29"}}}),
                    encoding="utf-8")
    assert B._previous_tests_fast(prev)["measured"] == "2026-09-29"
    # nothing to carry: None, never an invented zero
    assert B._previous_tests_fast(d / "missing.json") is None
    prev.write_text(json.dumps({"generated": "2026-10-06", "repo": {}}), encoding="utf-8")
    assert B._previous_tests_fast(prev) is None


def test_a_rewrite_keeps_the_files_line_endings():
    d = _tmp()
    crlf, lf, new = d / "crlf.json", d / "lf.json", d / "new.json"
    crlf.write_bytes(b'{\r\n  "a": 1\r\n}\r\n')
    lf.write_bytes(b'{\n  "a": 1\n}\n')
    text = '{\n  "a": 2\n}\n'
    for p in (crlf, lf, new):
        B.write_keeping_line_endings(p, text)
    assert crlf.read_bytes() == b'{\r\n  "a": 2\r\n}\r\n', crlf.read_bytes()
    assert lf.read_bytes() == b'{\n  "a": 2\n}\n', lf.read_bytes()
    assert new.read_bytes() == b'{\n  "a": 2\n}\n', "a new file must be LF"
    # the repository's own copy is CRLF and must stay so
    real = (ROOT / "docs/data/eval_sep2026.json").read_bytes()
    assert real.count(b"\r\n") == real.count(b"\n") > 0, "eval_sep2026.json lost its CRLF"


def test_within_cutoff_reads_the_manifest_date():
    d = _tmp()
    early, late, bare = d / "early", d / "late", d / "bare"
    for run in (early, late, bare):
        run.mkdir()
    for run, day in ((early, dt.datetime(2026, 9, 14, 23, 0)),
                     (late, dt.datetime(2026, 9, 15, 0, 30))):
        (run / "manifest.json").write_text("{}", encoding="utf-8")
        t = time.mktime(day.timetuple())
        os.utime(run / "manifest.json", (t, t))
    with _with_as_of(dt.date(2026, 9, 14)):
        assert B.within_cutoff(early) and not B.within_cutoff(late)
        assert not B.within_cutoff(bare), "a run with no manifest has no date"
    with _with_as_of(None):
        assert B.within_cutoff(early) and B.within_cutoff(late) and B.within_cutoff(bare)


def _fake_run(root: Path, tag: str, day: dt.datetime, topo: str, det_seq_step: int,
              ticks: int = 50):
    run = root / "demo" / "out" / tag
    run.mkdir(parents=True)
    (run / "manifest.json").write_text(json.dumps({"topology": topo}), encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({"det_hz": 9.9}), encoding="utf-8")
    rows = [{"t": i * 0.1, "det_seq": i * det_seq_step // 2} for i in range(ticks)]
    (run / "flight_log.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n",
                                          encoding="utf-8")
    (run / "kpi.json").write_text(json.dumps({"p0_violation_escape_rate": 0.0}),
                                  encoding="utf-8")
    t = time.mktime(day.timetuple())
    os.utime(run / "manifest.json", (t, t))


def test_rails_and_the_escape_count_respect_the_cut_off():
    """The September report must count the flights of September: a flight
    written after the cut-off is not in its denominators."""
    root = _tmp()
    _fake_run(root, "sept", dt.datetime(2026, 9, 10), "projectairsim-single-host", 10)
    _fake_run(root, "oct", dt.datetime(2026, 10, 1), "projectairsim-single-host", 10)
    old_root = B.ROOT
    B.ROOT = root
    try:
        with _with_as_of(dt.date(2026, 9, 14)):
            r = B.rails()
            assert r["counts"] == {"projectairsim-single-host": 1}, r
            assert (r["camera_flights_all"], r["camera_flights"]) == (1, 1), r
            assert [Path(f).parent.name for f in B._kpi_files()] == ["sept"]
        with _with_as_of(None):
            r = B.rails()
            assert r["counts"] == {"projectairsim-single-host": 2}, r
            assert len(B._kpi_files()) == 2
    finally:
        B.ROOT = old_root


def test_an_unmeasurable_camera_flight_is_counted_but_not_measured():
    """'0 of 34 (33 measurable)': a one-tick log has no time span."""
    root = _tmp()
    _fake_run(root, "ok", dt.datetime(2026, 9, 10), "projectairsim-single-host", 10)
    _fake_run(root, "one_tick", dt.datetime(2026, 9, 10), "projectairsim-single-host", 10,
              ticks=1)
    old_root = B.ROOT
    B.ROOT = root
    try:
        with _with_as_of(None):
            r = B.rails()
    finally:
        B.ROOT = old_root
    assert (r["camera_flights_all"], r["camera_flights"]) == (2, 1), r


def test_as_of_the_mid_evaluation_reads_the_september_record_from_git():
    """On the real repository: as of 2026-09-14 the repository facts are the
    ones committed that day - the 460/460 test count measured on 14 Sept
    (not 553/553 from 29 Sept), version 0.5.1, and the 16 commits since the
    2 September meeting - and the CityLife record is the one typed then."""
    if not (ROOT / ".git").exists():
        print("      (skipped: no git checkout)")
        return
    with _with_as_of(dt.date(2026, 9, 14)):
        rev = B.cutoff_revision()
        assert rev and rev.startswith("2457987"), rev
        g = B.repo(run_tests=False, previous=ROOT / "docs/data/eval_sep2026.json")
        old = B.eval_at_cutoff()
    tf = g["tests_fast"]
    assert (tf["passed"], tf["total"], tf["measured"]) == (460, 460, "2026-09-14"), tf
    assert tf["after_cutoff"] is False and tf["carried_forward"] is True, tf
    assert g["version"] == "0.5.1" and len(g["commits_since_meeting"]) == 16, \
        (g["version"], len(g["commits_since_meeting"]))
    assert all(c["date"] <= "2026-09-14" for c in g["commits_since_meeting"])
    cl = old["unflown"]["citylife_level"]
    assert (cl["pedestrians"], cl["cars"]) == (16, 8), cl


def test_the_committed_file_is_the_september_view():
    """docs/data/eval_sep2026.json as regenerated on 2026-10-07 (pas env,
    default --as-of): the corrected September figures, not October's."""
    d = json.loads((ROOT / "docs/data/eval_sep2026.json").read_text(encoding="utf-8"))
    assert d.get("as_of") == "2026-09-14", d.get("as_of")
    k, r = d["kpi"], d["rails"]
    assert (k["flights_total"], k["flights_escape_zero"], len(k["unshielded_controls"])) == (46, 41, 5)
    assert (r["camera_flights_meeting_both_gates"], r["camera_flights_all"],
            r["camera_flights"]) == (0, 34, 33), r
    assert sorted(x["det_hz"] for x in d["detector"]["flights"].values()) == \
        [2.77, 3.11, 3.87, 4.06, 4.31]
    assert d["unflown"]["camera_768x432"]["flight_artefacts"] == []


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
