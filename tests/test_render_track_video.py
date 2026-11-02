"""tools/render_track_video.py: a flight-log replay video for rails with no camera.

Run either way:
    pytest tests/test_render_track_video.py -v
    python tests/test_render_track_video.py

What these guard against: a video that does not say it is a log replay (it
could be taken for a camera recording), a run folder with no policy drawn as
an empty map, and a failed writer leaving nothing while reporting success.
"""
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import render_track_video as R                     # noqa: E402

SKIP = "SKIP"

POLICY = {"constraints": [
    {"id": "nfz", "type": "polygon_fence", "constraint_type": "hard", "priority": "P0",
     "margin_m": 1.0, "vertices": [{"x": 5, "y": 5}, {"x": 9, "y": 5}, {"x": 9, "y": 9},
                                   {"x": 5, "y": 9}]},
    {"id": "ring", "type": "circle_fence", "constraint_type": "soft", "priority": "P1",
     "center": {"x": -4, "y": 2}, "radius_m": 2.0, "margin_m": 0.5},
    {"id": "band", "type": "altitude_envelope", "alt_min_m": 10, "alt_max_m": 20}]}


def _row(i, state_before="Normal", state_after="Normal", edge=None):
    return {"t": i * 0.1, "tick": i + 1, "x": 0.5 * i, "y": 0.2 * i, "up": 15.0,
            "raw": {"vx": 4.0, "vy": 4.0, "vz_up": 0.0}, "emitted": {"vx": 2.0, "vy": -1.0, "vz_up": 0.0},
            "violations": [{"rule_id": "nfz"}], "repairs": [{"operator": "GeofenceSlide", "magnitude_m": 3.14159}],
            "flown": True, "setpoint": "velocity", "generation": 0,
            "fsm_state_before": state_before, "fsm_state_after": state_after, "fsm_edge": edge}


def _run(d: Path, rows, policy=POLICY) -> Path:
    run = d / "demo" / "out" / "fake_run"
    run.mkdir(parents=True)
    (run / "flight_log.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n",
                                          encoding="utf-8")
    if policy is not None:
        (run / "policy_g0.json").write_text(json.dumps(policy), encoding="utf-8")
    (run / "prompt.yaml").write_text("mission:\n  target:\n    x: 12\n    y: 12\n", encoding="utf-8")
    return run


def test_zones_read_polygons_and_circles_with_margins_and_hardness():
    z = R.zones(POLICY)
    assert [(q["id"], q["kind"], q["hard"]) for q in z] == [("nfz", "poly", True),
                                                            ("ring", "circle", False)]
    assert z[0]["margin"] == 1.0 and z[1]["r"] == 2.0
    assert R.altitude_band(POLICY) == (10.0, 20.0)


def test_the_fsm_state_and_last_edge_follow_the_log():
    rows = [_row(0), _row(1, "Normal", "Brake", "X1"), _row(2, "Brake", "Brake"),
            _row(3, "Brake", "Loiter", "G3")]
    assert R.state_at(rows, 0) == ("Normal", None)
    s, e = R.state_at(rows, 2)
    assert s == "Brake" and "[X1]" in e
    s, e = R.state_at(rows, 3)
    assert s == "Loiter" and "[G3]" in e


def test_a_run_without_a_policy_is_refused_not_drawn_empty():
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_row(0)], policy=None)
        try:
            R.render(run, run / "x.mp4")
        except SystemExit as e:
            assert "policy" in str(e)
        else:
            raise AssertionError("rendered a run with no policy")


def test_the_mission_target_is_read_from_the_prompt():
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_row(0)])
        assert R.mission_target(run) == (12.0, 12.0)


def test_a_short_run_renders_a_non_empty_video_and_says_it_is_a_log_replay():
    try:
        from tools.encode_deliverable import find_ffmpeg
        find_ffmpeg()
        import matplotlib                                      # noqa: F401
    except (Exception, SystemExit) as e:                       # noqa: BLE001
        print(f"      SKIP: no ffmpeg or matplotlib ({e})")
        return SKIP
    src = (ROOT / "tools" / "render_track_video.py").read_text(encoding="utf-8")
    assert "not a camera" in src and "recording." in src
    with tempfile.TemporaryDirectory() as d:
        run = _run(Path(d), [_row(i, "Normal", "Brake" if i else "Normal", "X1" if i == 1 else None)
                             for i in range(4)])
        out = R.render(run, run / "v.mp4", fps=4)
        assert out.is_file() and out.stat().st_size > 0
        assert not (run / "v.partial.mp4").exists()


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
            else:
                print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed, {skipped} skipped")
    sys.exit(1 if failed else 0)
