"""tools/vla_replay_gate.py: recorded camera pairs through the VLA server,
checked before a flight trusts its answers.

Run either way:
    pytest tests/test_vla_replay_gate.py -v
    python tests/test_vla_replay_gate.py

WHAT IS BEING CLAIMED

The gate reads a demo/vla_fly.py run folder (vla_steps.jsonl + the JPEGs in
vla_frames/ by SHA-256), sends each distinct pair again with its recorded
phrase, then the five left-to-right sweep phrases and the empty phrase, and
fails - exit 1 - when:
  * the heading change does not grow from "to your left " to "to your right "
    (mean over the frames, half a bin of tolerance, and a minimum spread), or
    too few frames are monotone on their own;
  * more than 2 % of the answers are malformed;
  * the model answers LAND under a hint away from the run's goal (an
    approach run, with no goal, allows none);
  * any request goes unanswered.
It passes a scripted server that steers by the phrase and lands only on "",
and it never sends the goal: the requests carry the phrase only. No model,
GPU or simulator: tests/vla_fake_server.py answers.

Shown failing first: tools/vla_replay_gate.py did not exist before this
change, so every test failed at import.
"""
from __future__ import annotations

import hashlib
import io
import json
import contextlib
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.vla_link import RemoteVLA                             # noqa: E402
import vla_fake_server as F                                          # noqa: E402
import vla_replay_gate as G                                          # noqa: E402

SKIP = "SKIP"
MODEL = {"id": "aerialvla-lora/aero_vla@196f2f32", "base": "openvla-7b@47a0ec7f",
         "weights_sha256": hashlib.sha256(b"gate weights").hexdigest(),
         "precision": "bf16", "attn": "sdpa", "merged": False}
DPSI = {"to your left ": -0.45, "forward-left ": -0.17, "straight ahead ": 0.0,
        "forward-right ": 0.17, "to your right ": 0.45, "to your right rear ": 1.0,
        "to your left rear ": -1.0}
GOAL = (43.0, 40.0)


def _ours(reply):
    return dict(reply, model=dict(MODEL)) if isinstance(reply, dict) and "model" in reply else reply


def steering(sign=1.0, land_on=("",), parse_fail_every=0, error=False):
    """A scripted AerialVLA: the heading change set by the phrase (times
    `sign`), LAND on the phrases in `land_on`, a malformed answer every
    `parse_fail_every`-th request, or nothing but server errors."""
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        if error:
            return F.error_reply(req, "inference failed: CUDA out of memory")
        if parse_fail_every and n["k"] % parse_fail_every == 0:
            return _ours(F.parse_fail_reply(req))
        if req["phrase"] in land_on:
            return _ours(F.land_reply(req))
        return _ours(F.step_reply(req, 4.0, 0.0, sign * DPSI[req["phrase"]]))
    return respond


def _run_dir(poses, goal=GOAL, mission="goto", phrase="straight ahead ", with_frames=True):
    d = Path(tempfile.mkdtemp(prefix="vlagate_")) / "vla_goto_x"
    (d / "vla_frames").mkdir(parents=True)
    lines = []
    for k, (x, y) in enumerate(poses):
        front, down = f"front{k}".encode(), f"down{k}".encode()
        fs, ds = hashlib.sha256(front).hexdigest(), hashlib.sha256(down).hexdigest()
        if with_frames:
            (d / "vla_frames" / f"{fs}.jpg").write_bytes(front)
            (d / "vla_frames" / f"{ds}.jpg").write_bytes(down)
        lines.append({"schema": "vla_steps/1", "seq": k + 1, "t_capture": float(k),
                      "pose_capture": {"x": x, "y": y, "up": 10.0, "yaw_rad": 1.55},
                      "phrase": phrase, "hint_source": "goal" if mission == "goto" else "detector",
                      "object": "", "front_jpg_sha256": fs, "down_jpg_sha256": ds,
                      "kind": "step", "bins": [78, 49, 49], "image_sha256": "a" * 64})
        lines.append(dict(lines[-1], seq=k + 100))          # a repeat: one pair, sent once
    (d / "vla_steps.jsonl").write_text("".join(json.dumps(r) + "\n" for r in lines),
                                       encoding="utf-8")
    m = {"mission": {"kind": mission, "goal_xy": list(goal) if mission == "goto" else None,
                     "monitor": {"config": {"arrive_radius_m": 3.0}}}}
    (d / "metrics.json").write_text(json.dumps(m), encoding="utf-8")
    return d


def _gate(respond, run, **kw):
    items, info = G.frames_from_run(run)
    with F.FakeVLAServer(respond, model=MODEL) as srv:
        c = RemoteVLA("127.0.0.1", srv.port, response_timeout_s=5.0, reconnect_backoff_s=0.0)
        try:
            c.connect()
            rep = G.run_gate(c, items, info, timeout_s=5.0, **kw)
        finally:
            c.close()
        return rep, list(srv.requests)


FAR = [(42.0 + k, -18.0 + 3 * k) for k in range(6)]


def test_a_model_that_steers_by_the_phrase_and_lands_only_on_empty_passes():
    run = _run_dir(FAR)
    rep, reqs = _gate(steering(), run)
    assert rep["pass"], json.dumps(rep["checks"], indent=1)
    assert rep["frames"] == 6 and rep["input"]["distinct_pairs"] == 6
    assert rep["requests"] == 6 * (1 + 5 + 1) == len(reqs)
    m = rep["checks"]["monotonic"]
    assert m["mean_dpsi_rad"]["to your left "] < 0 < m["mean_dpsi_rad"]["to your right "]
    assert m["frame_monotone_share"] == 1.0 and rep["no_hint_land_rate"] == 1.0
    assert rep["recorded_replay"]["n"] == 6 and rep["identity"]["id"] == MODEL["id"]
    for q in reqs:                                 # the goal never travels
        assert not ({"target", "goal", "goal_xy", "xy"} & set(q))
        assert str(GOAL[0]) not in q["object"]
    assert {q["phrase"] for q in reqs} == set(G.SWEEP) | {"", "straight ahead "}


def test_an_inverted_or_flat_phrase_response_fails_the_monotonic_check():
    run = _run_dir(FAR)
    rep, _ = _gate(steering(sign=-1.0), run)
    assert not rep["pass"] and not rep["checks"]["monotonic"]["pass"]
    assert rep["checks"]["parse_fail"]["pass"] and rep["checks"]["answered"]["pass"]
    rep, _ = _gate(steering(sign=0.0), run)
    assert not rep["checks"]["monotonic"]["pass"]
    assert rep["checks"]["monotonic"]["spread_rad"] == 0.0


def test_land_under_a_hint_fails_away_from_the_goal_and_passes_beside_it():
    rep, _ = _gate(steering(land_on=("", "straight ahead ")), _run_dir(FAR))
    lc = rep["checks"]["land_only_near_goal_or_without_hint"]
    assert not lc["pass"] and lc["violations"] >= 6 and lc["near_goal_m"] == 6.0
    near = [(GOAL[0] - 1.0 - 0.5 * k, GOAL[1] - 2.0) for k in range(4)]
    rep, _ = _gate(steering(land_on=("", "straight ahead ")), _run_dir(near))
    assert rep["checks"]["land_only_near_goal_or_without_hint"]["pass"], rep["checks"]
    rep, _ = _gate(steering(land_on=("", "straight ahead ")),
                   _run_dir(near, mission="approach", phrase="forward-left "))
    lc = rep["checks"]["land_only_near_goal_or_without_hint"]
    assert not lc["pass"] and not lc["goal_known"], lc


def test_malformed_answers_and_unanswered_requests_fail_the_gate():
    rep, _ = _gate(steering(parse_fail_every=5), _run_dir(FAR))
    pf = rep["checks"]["parse_fail"]
    assert not pf["pass"] and pf["rate"] > 0.02 and not rep["pass"]
    rep, _ = _gate(steering(error=True), _run_dir(FAR))
    assert not rep["checks"]["answered"]["pass"] and not rep["pass"]
    assert rep["checks"]["answered"]["unanswered"] == rep["requests"]


def test_the_cli_writes_its_report_and_says_pass_fail_or_no_input():
    run = _run_dir(FAR)
    with F.FakeVLAServer(steering(), model=MODEL) as srv:
        with contextlib.redirect_stdout(io.StringIO()) as out:
            code = G.main(["--run", str(run), "--vla-server", f"127.0.0.1:{srv.port}",
                           "--max-frames", "3"])
    assert code == 0, out.getvalue()
    rep = json.loads((run / "replay_gate.json").read_text(encoding="utf-8"))
    assert rep["pass"] and rep["frames"] == 3 and rep["server"]["mode"] == "attached"
    assert b"\r\n" not in (run / "replay_gate.json").read_bytes()
    with F.FakeVLAServer(steering(sign=-1.0), model=MODEL) as srv:
        with contextlib.redirect_stdout(io.StringIO()):
            assert G.main(["--run", str(run), "--vla-server", f"127.0.0.1:{srv.port}"]) == 1
    # a stand-in identity is refused unless asked for
    with F.FakeVLAServer(F.goto_responder((0, 0), 10.0)) as srv:
        with contextlib.redirect_stderr(io.StringIO()) as err:
            assert G.main(["--run", str(run), "--vla-server", f"127.0.0.1:{srv.port}"]) == 2
        assert "not AerialVLA" in err.getvalue()
    bare = _run_dir(FAR, with_frames=False)
    with contextlib.redirect_stderr(io.StringIO()) as err:
        assert G.main(["--run", str(bare), "--vla-server", "127.0.0.1:1"]) == 2
    assert "no recorded camera pair" in err.getvalue()
    with contextlib.redirect_stderr(io.StringIO()):
        assert G.main(["--run", str(run)]) == 2           # neither attach nor start


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
        except Exception as e:                        # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
