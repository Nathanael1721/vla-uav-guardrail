"""tools/build_mideval_report.py: the mid-evaluation report generator.

Run either way:
    pytest tests/test_build_mideval_report.py -v
    python tests/test_build_mideval_report.py

WHY THIS FILE EXISTS

The review of 2026-10-06 rendered the "corrected" generator and found it did
not reproduce the corrected September report: it read whatever the numbers
file held on the day, so it printed October counts (71 shielded flights,
"2 of 63 measurable") under the September title, and it contradicted itself -
Section 7 said the 768x432 camera had flown on 26 flights while plan item 3
still said "Fly the 768x432 camera". Each test renders the report to a
temporary file (never the delivered docs/MID-EVALUATION-REPORT-Sep2026.md)
from the committed numbers file or a modified copy of it, and reads the text.
"""
import copy
import json
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import build_mideval_report as R                                    # noqa: E402
import check_claims as C                                            # noqa: E402

EVAL = ROOT / "docs/data/eval_sep2026.json"
DELIVERED = ROOT / "docs/MID-EVALUATION-REPORT-Sep2026.md"


def _render(mutate=None) -> str:
    d = json.loads(EVAL.read_text(encoding="utf-8"))
    if mutate:
        d = copy.deepcopy(d)
        mutate(d)
    tmp = Path(tempfile.mkdtemp())
    src, out = tmp / "eval.json", tmp / "report.md"
    src.write_text(json.dumps(d), encoding="utf-8")
    before = DELIVERED.read_bytes() if DELIVERED.exists() else None
    R.main(["--eval", str(src), "--out", str(out)])
    after = DELIVERED.read_bytes() if DELIVERED.exists() else None
    assert before == after, "rendering to --out touched the delivered report"
    return out.read_text(encoding="utf-8")


def _flown(d, cam=True, city=True):
    u = d["unflown"]
    if cam:
        u["camera_768x432"]["flight_artefacts"] = ["citylife_a", "citylife_b"]
    if city:
        u["citylife_level"]["flights"] = {"citylife_a": {}, "citylife_b": {}}


# --------------------------------------------------------------------------- #

def test_the_september_report_prints_the_corrected_september_figures():
    md = _render()
    assert "on or before 2026-09-14" in md, "the header must name the cut-off"
    assert "0.0 on 41 shielded demo and SITL flights" in md
    assert "| Camera flights meeting both gates | 0 of 34 (33 measurable) |" in md
    assert "0 of 34 (33 measurable) camera flights meet both" in md
    assert "| Detector rate in flight | 2.77–4.31 Hz |" in md
    assert "**460/460 fast** (run 2026-09-14)" in md
    # the September state of the camera, consistently
    assert "Fly the 768×432 camera" in md and "configured at 768×432 (not yet flown)" in md
    assert "- **Not yet flown.** The 768×432 camera and the CityLife scene." in md


def test_a_flown_camera_is_never_also_still_to_fly():
    md = _render(lambda d: _flown(d))
    assert "Fly the 768×432" not in md, "plan item 3 asks to fly a camera that has flown"
    assert "Not yet flown" not in md
    assert "Evaluate a narrower field of view on the 768×432 camera (2 flights flown" in md
    assert "(2 flights flown with it)" in md


def test_one_flown_one_not_names_only_the_unflown_one():
    md = _render(lambda d: _flown(d, cam=True, city=False))
    assert "- **Not yet flown.** The CityLife scene." in md, \
        [l for l in md.splitlines() if "Not yet flown" in l]
    assert "Fly the 768×432" not in md
    md = _render(lambda d: _flown(d, cam=False, city=True))
    assert "- **Not yet flown.** The 768×432 camera." in md
    assert "Fly the 768×432 camera" in md


def test_a_test_count_measured_after_the_cut_off_says_so():
    def later(d):
        d["repo"]["tests_fast"]["measured"] = "2026-09-29"
    md = _render(later)
    assert re.search(r"\(run 2026-09-29, after this report's 2026-09-14 cut-off\)", md), \
        [l for l in md.splitlines() if "fast**" in l]


def test_an_older_numbers_file_without_a_cut_off_still_renders():
    def old(d):
        d.pop("as_of", None)
        d["rails"].pop("camera_flights_all", None)
    md = _render(old)
    assert "from every artefact on disk" in md
    assert "0 of 33 (33 measurable)" in md


def test_the_rendered_report_carries_no_withdrawn_claim():
    for mutate in (None, lambda d: _flown(d)):
        md = _render(mutate)
        hits = C.scan_text(md, "docs/rendered.md")
        assert not hits, [(h.claim, h.text) for h in hits]


if __name__ == "__main__":
    import contextlib
    import io
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            with contextlib.redirect_stdout(io.StringIO()):
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
