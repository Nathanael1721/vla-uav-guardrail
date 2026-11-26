"""tools/nonsetpoint_ticks.py: timing the ticks that publish no setpoint
(WP3-13), on synthetic flight logs.

    python tests/test_nonsetpoint_ticks.py      (no pytest needed)
"""
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import nonsetpoint_ticks as N                              # noqa: E402


def _phases(shield=5.0, hold=1.0, post=2.0):
    return {"pre_shield": 0.2, "hot_apply": 0.0, "shield": shield, "fsm": 1.0,
            "hold": hold, "publish": 0.0, "post_publish": post,
            "deferred": 0.0, "n_deferred": 0}


def _write(root: Path, name: str, rows: list[dict]) -> None:
    d = root / name
    d.mkdir(parents=True)
    with (d / "flight_log.jsonl").open("w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
        fh.write("{not json\n")


def _scan(rows_by_run: dict) -> dict:
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        for name, rows in rows_by_run.items():
            _write(root, name, rows)
        _write(root, "other_001", [{"tick": 1, "tick_ms": None,
                                    "tick_phases_ms": _phases(shield=500.0)}])
        out = root / "out.json"
        assert N.main([str(root), "--prefix", "ntX_", "--out", str(out)]) == 0
        return json.loads(out.read_text(encoding="utf-8"))


def test_setpoint_ticks_are_counted_not_timed():
    res = _scan({"ntX_001": [{"tick": 1, "tick_ms": 12.0, "tick_phases_ms": _phases()},
                             {"tick": 2, "tick_ms": None, "fsm_state_after": "RTL",
                              "tick_phases_ms": _phases()}]})
    assert res["runs_read"] == 1                 # the prefix keeps other_001 out
    assert res["setpoint_ticks"] == 1
    assert res["groups"]["all"]["to_publish_ms"]["n"] == 1
    assert res["unparsed_lines"] == 1


def test_to_publish_stops_before_post_publish_and_whole_tick_includes_it():
    res = _scan({"ntX_001": [{"tick": 1, "tick_ms": None, "set_mode": "RTL",
                              "fsm_state_after": "RTL",
                              "tick_phases_ms": _phases(shield=5.0, hold=1.0, post=120.0)}]})
    g = res["groups"]
    assert g["all"]["to_publish_ms"]["max"] == 7.2           # 0.2 + 0 + 5 + 1 + 1 + 0
    assert g["all"]["to_publish_ms"]["over_budget"] == 0
    assert g["all"]["whole_tick_ms"]["max"] == 127.2
    assert g["all"]["whole_tick_ms"]["over_budget"] == 1
    assert g["set_mode_request"]["to_publish_ms"]["n"] == 1
    assert g["fsm_after:RTL"]["to_publish_ms"]["n"] == 1
    assert res["slowest"][0]["whole_tick_ms"] == 127.2


def test_a_row_without_phases_is_not_measured_never_zero():
    res = _scan({"ntX_001": [{"tick": 1, "tick_ms": None, "fsm_state_after": "Land"}]})
    assert res["nonsetpoint_ticks_without_phases"] == 1
    assert "all" not in res["groups"]


def test_percentiles_are_nearest_rank_like_kpi_report():
    vals = [float(v) for v in range(1, 101)]
    s = N.stats(vals)
    assert (s["p50"], s["p95"], s["p99"], s["max"]) == (50.0, 95.0, 99.0, 100.0)
    assert N.stats([]) == {"n": 0, "p50": None, "p95": None, "p99": None,
                           "max": None, "over_budget": 0}


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
