"""Gate G3 (Orin VLA inference latency): the judge and the committed evidence.

Run either way:
    pytest tests/test_g3_gate.py -v
    python tests/test_g3_gate.py

tools/g3_gate.py reads the hil C2 flights the backend table selects, takes the
per-decision round trip from each flight's vla_steps.jsonl, and passes a backend
only when every one of at least 3 flights on an Orin has p95 <= 100 ms and
>= 10 decisions/s while asked. These tests hold it to that on synthetic run
folders (no rig, no model), and hold the committed table in
docs/data/g3_gate_20261117 to the verdict its README states.
"""
from __future__ import annotations

import contextlib
import io
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import g3_gate as G                                                   # noqa: E402

COMMITTED = ROOT / "docs" / "data" / "g3_gate_20261117" / "g3_gate.json"
README = ROOT / "docs" / "data" / "g3_gate_20261117" / "README.md"
ORIN = "NVIDIA Jetson AGX Orin Developer Kit"
_TMP: list[Path] = []


def _tmp() -> Path:
    d = Path(tempfile.mkdtemp(prefix="g3_gate_"))
    _TMP.append(d)
    return d


def _run(root: Path, tag: str, rtts_s: list[float], host: str = ORIN) -> None:
    d = root / tag
    d.mkdir(parents=True)
    with open(d / "vla_steps.jsonl", "w", encoding="utf-8") as fh:
        for i, r in enumerate(rtts_s):
            fh.write(json.dumps({"kind": "step", "ok": True, "parse_ok": True, "rtt_s": r,
                                 "latency_server_s": r * 0.8, "latency_client_s": r + 0.03,
                                 "t_capture": 100.0 + i, "t_sent": 100.02 + i}) + "\n")
    (d / "metrics.json").write_text(json.dumps(
        {"hil_evidence": {"vla_host": {"device_model": host}}}), encoding="utf-8")


def _table(flights: dict[str, list[tuple[str, float]]]) -> dict:
    rows = [{"backend": "stub", "label": "Stub", "records": []}]
    for name, fl in flights.items():
        rows.append({"backend": name, "label": name, "records": [
            {"protocol": "C2", "not_c2": [], "c2_censored": False, "tag": tag,
             "decision_hz_asked": hz, "decision_hz_mission": hz, "code_revision": "abc1234"}
            for tag, hz in fl]})
    return {"rows": rows, "g3": {"seven_b": [], "smaller": []}}


def _by(out: dict) -> dict:
    return {b["backend"]: b for b in out["backends"]}


def test_percentile_is_linear_like_numpy():
    xs = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    assert G.pct(xs, 0.5) == 5.5
    assert abs(G.pct(xs, 0.95) - 9.55) < 1e-9
    assert G.pct([], 0.5) is None


def test_a_fast_backend_passes_and_a_slow_one_does_not():
    root = _tmp()
    for i in range(3):
        _run(root, f"fast_{i}", [0.05] * 95 + [0.09] * 5)
        _run(root, f"slow_{i}", [1.5] * 10)
    t = _table({"fast": [(f"fast_{i}", 13.0) for i in range(3)],
                "slow": [(f"slow_{i}", 0.6) for i in range(3)]})
    out = _by(G.build(t, [root]))
    assert out["fast"]["passes"], out["fast"]["why_not"]
    assert not out["slow"]["passes"]
    assert "stub" not in out, "the stub is not a learned VLA backend"
    assert out["fast"]["in_flight"]["pooled"]["round_trip_ms"]["n"] == 300


def test_p95_over_100_ms_fails_even_at_a_high_rate():
    root = _tmp()
    for i in range(3):
        _run(root, f"f_{i}", [0.05] * 90 + [0.15] * 10)    # p95 lands in the 150 ms tail
    out = _by(G.build(_table({"b": [(f"f_{i}", 13.0) for i in range(3)]}), [root]))
    assert not out["b"]["passes"]
    assert any("round trip p95" in w for w in out["b"]["why_not"])


def test_under_10_hz_while_asked_fails_even_when_fast():
    root = _tmp()
    for i in range(3):
        _run(root, f"f_{i}", [0.05] * 20)
    out = _by(G.build(_table({"b": [("f_0", 13.0), ("f_1", 13.0), ("f_2", 9.9)]}), [root]))
    assert not out["b"]["passes"]
    assert any("decisions/s while asked" in w for w in out["b"]["why_not"])


def test_a_flight_off_the_orin_or_missing_does_not_count():
    root = _tmp()
    _run(root, "f_0", [0.05] * 20)
    _run(root, "f_1", [0.05] * 20)
    _run(root, "f_2", [0.05] * 20, host="NVIDIA GeForce desktop")
    t = _table({"b": [("f_0", 13.0), ("f_1", 13.0), ("f_2", 13.0), ("f_gone", 13.0)]})
    b = _by(G.build(t, [root]))["b"]
    assert not b["passes"]
    text = " ".join(b["why_not"])
    assert "not an Orin" in text and "not found" in text
    assert "2 C2 flights on the Orin read" in text


def test_fewer_than_three_flights_cannot_pass():
    root = _tmp()
    _run(root, "f_0", [0.05] * 20)
    _run(root, "f_1", [0.05] * 20)
    b = _by(G.build(_table({"b": [("f_0", 13.0), ("f_1", 13.0)]}), [root]))["b"]
    assert not b["passes"]


def test_exit_codes():
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        _exit_codes()


def _exit_codes():
    root = _tmp()
    for i in range(3):
        _run(root, f"slow_{i}", [1.5] * 5)
    tp = root / "table.json"
    tp.write_text(json.dumps(_table({"slow": [(f"slow_{i}", 0.6) for i in range(3)]})),
                  encoding="utf-8")
    assert G.main(["--table", str(tp), "--runs", str(root)]) == 1
    assert G.main(["--table", str(tp), "--runs", str(root / "nowhere")]) == 2
    assert G.main(["--check", str(root / "absent.json")]) == 2
    w = root / "written"
    for i in range(3):
        _run(root, f"fast_{i}", [0.05] * 5)
    tp.write_text(json.dumps(_table({"fast": [(f"fast_{i}", 13.0) for i in range(3)]})),
                  encoding="utf-8")
    assert G.main(["--table", str(tp), "--runs", str(root), "--write", str(w)]) == 0
    assert G.main(["--check", str(w / "g3_gate.json")]) == 0
    assert (w / "g3_gate.md").read_text(encoding="utf-8").startswith("# Gate G3")


def test_committed_table_passes_aiotvla_only_and_no_7b():
    out = json.loads(COMMITTED.read_text(encoding="utf-8"))
    with contextlib.redirect_stdout(io.StringIO()):
        assert G.main(["--check", str(COMMITTED)]) == 0
    v = G.finish(out)["verdict"]
    assert v["passing"] == ["aiotvla_v10b"], v
    assert v["seven_b_passing"] == []
    b = _by(out)["aiotvla_v10b"]
    assert len(b["in_flight"]["flights"]) == 3
    for f in b["in_flight"]["flights"]:
        assert f["vla_host"] == ORIN
        assert f["round_trip_ms"]["p95"] <= 100.0 and f["decision_hz_asked"] >= 10.0


def test_readme_states_the_committed_numbers():
    out = _by(json.loads(COMMITTED.read_text(encoding="utf-8")))
    text = README.read_text(encoding="utf-8")
    pooled = out["aiotvla_v10b"]["in_flight"]["pooled"]["round_trip_ms"]
    assert f"{pooled['p50']} / {pooled['p95']} / {pooled['p99']}" in text
    assert f"{pooled['n']} answers" in text
    for f in out["aiotvla_v10b"]["in_flight"]["flights"]:
        r = f["round_trip_ms"]
        assert f"{f['tag']} " in text and f"{r['p50']} / {r['p95']} / {r['p99']}" in text
    assert "No new flight or measurement was made" in text
    assert "claims no int8 7B number" in text


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
    for d in _TMP:
        shutil.rmtree(d, ignore_errors=True)
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
