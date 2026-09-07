"""The deck computes tracking accuracy in JavaScript. It must agree with Python.

Run either way:
    pytest tests/test_deck_scorer_parity.py -v
    python tests/test_deck_scorer_parity.py

`tools/deck/build_sept_deck.js` re-derives `frac_on_target` from the flight logs
at build time rather than trusting a stored number — a good rule that earned its
keep. But it re-derives it with its OWN copy of the projection, and a copy is a
thing that drifts.

It drifted immediately. When `demo/track_truth.py` was fixed on 2026-09-07 so
that a row carrying `truth` with an empty `pts` counts as UNSCORABLE, the JS
mirror kept an `&& r.truth.pts.length` guard and fell through to `tgt_x/tgt_y` —
the car. On the next flight recorded with a pedestrian subject that would have
put **0.406 on target** on a slide while `metrics.json` beside it said **0.931**,
which is the exact defect the Python fix was written to remove.

So this test runs the JS out of the deck builder under node, on rows built here,
and compares it to the Python. It skips if node is absent.
"""
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from track_truth import score_rows                    # noqa: E402

DECK = ROOT / "tools" / "deck" / "build_sept_deck.js"
W = 400


def _node() -> str | None:
    return shutil.which("node")


def _js_functions() -> str:
    """Lift truthPts + scoreRows out of the deck builder, verbatim.

    Extracted rather than duplicated: a copy in this file would be a third
    implementation, and three is worse than two.
    """
    src = DECK.read_text(encoding="utf-8")
    out = []
    for name in ("function truthPts(", "function scoreRows("):
        i = src.index(name)
        depth, j = 0, src.index("{", i)
        k = j
        while True:
            if src[k] == "{":
                depth += 1
            elif src[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        out.append(src[i:k + 1])
    return "\n".join(out)


def _run_js(rows: list) -> dict:
    node = _node()
    if node is None:
        return {}
    script = (_js_functions() + "\nconst rows = " + json.dumps(rows) +
              ";\nconsole.log(JSON.stringify(scoreRows(rows)));\n")
    d = Path(tempfile.mkdtemp(prefix="deckparity_"))
    f = d / "parity.js"
    f.write_text(script, encoding="utf-8")
    try:
        r = subprocess.run([node, str(f)], capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            raise AssertionError(f"the deck's scorer threw:\n{r.stderr.strip()}")
        return json.loads(r.stdout.strip())
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _row(cx, tick, truth=None, tgt=(10.0, 0.0), x=0.0, y=0.0, psi=0.0):
    r = {"tick": tick, "x": x, "y": y, "psi": psi,
         "det": {"cx": cx, "img_w": W},
         "tgt_x": tgt[0] if tgt else None, "tgt_y": tgt[1] if tgt else None}
    if truth is not None:
        r["truth"] = truth
    return r


def _compare(rows, label):
    js = _run_js(rows)
    if not js:
        return                                     # node absent
    py = score_rows(rows)
    assert js["n"] == py["det_scored"], (label, js, py)
    assert js["unscorable"] == py["det_unscorable"], (label, js, py)
    assert js["outOfFov"] == py["n_det_with_target_out_of_fov"], (label, js, py)
    if py["frac_on_target"] is None:
        assert js["onTarget"] is None, (label, js)
    else:
        assert abs(js["onTarget"] - py["frac_on_target"]) < 1e-6, (label, js, py)
        assert abs(js["median"] - py["det_gt_err_px_median"]) < 0.05, (label, js, py)


def test_the_deck_still_defines_both_functions():
    assert DECK.is_file()
    src = _js_functions()
    assert "truthPts" in src and "scoreRows" in src


def test_old_logs_with_no_truth_field_agree():
    _compare([_row(200.0, 1), _row(390.0, 2)], "no-truth")


def test_a_truth_with_points_agrees():
    rows = [_row(200.0, 1, truth={"class": "pedestrian", "pts": [[10.0, 0.0]]}),
            _row(400.0, 2, truth={"class": "pedestrian",
                                  "pts": [[10.0, 0.0], [10.0, 10.0]]})]
    _compare(rows, "truth-with-points")


def test_an_empty_truth_is_unscorable_in_BOTH_and_never_falls_back_to_the_car():
    """The drift that was caught. A pedestrian phase logged with no pedestrian
    truth must be unscorable in both languages — not scored against the car."""
    rows = [_row(200.0, i, truth={"class": "pedestrian", "pts": []})
            for i in range(1, 6)]
    js = _run_js(rows)
    if not js:
        return
    assert js["n"] == 0 and js["unscorable"] == 5, js
    assert js["onTarget"] is None, js
    _compare(rows, "empty-truth")


def test_an_all_unscorable_phase_does_not_kill_the_deck_build():
    """It used to return bare null, and slide 12 dereferenced it. The deck died
    with a TypeError and produced no .pptx at all."""
    js = _run_js([_row(200.0, 1, truth={"class": "van", "pts": []})])
    if not js:
        return
    assert js is not None and js["unscorable"] == 1, js
    assert js["median"] is None and js["p95"] is None, js


def test_a_mixed_flight_agrees_end_to_end():
    """A retarget flight as it will actually be logged from now on: car truth
    before the switch, pedestrian truth after."""
    rows = [_row(200.0, i, truth={"class": "car", "pts": [[10.0, 0.0]]})
            for i in range(1, 5)]
    rows += [_row(400.0, i, truth={"class": "pedestrian",
                                   "pts": [[10.0, 10.0], [-10.0, 0.0]]})
             for i in range(5, 9)]
    rows += [_row(120.0, i, truth={"class": "pedestrian", "pts": []})
             for i in range(9, 12)]
    _compare(rows, "mixed")


def test_the_recorded_retarget_flight_agrees():
    log = ROOT / "demo" / "out" / "retarget_demo2" / "flight_log.jsonl"
    if not log.is_file():
        return
    rows = [json.loads(line) for line in log.open(encoding="utf-8") if line.strip()]
    _compare(rows[:200], "retarget_demo2-head")


def test_the_js_has_no_length_guard_that_would_reintroduce_the_fallback():
    """A source-level tripwire for the specific edit that caused the drift, so a
    future 'tidy-up' cannot silently restore it."""
    src = _js_functions()
    m = re.search(r"if \(r\.truth[^\n]*\n", src)
    assert m, src
    assert "Array.isArray(r.truth.pts)) return" in m.group(0), m.group(0)


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
