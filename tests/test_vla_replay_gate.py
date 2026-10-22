"""tools/vla_replay_gate.py: recorded camera pairs through the VLA server,
checked before a flight trusts its answers.

Run either way:
    pytest tests/test_vla_replay_gate.py -v
    python tests/test_vla_replay_gate.py

WHAT IS BEING CLAIMED

The gate reads a demo/vla_fly.py run folder (vla_steps.jsonl + the JPEGs in
vla_frames/ by SHA-256), sends each distinct pair again with its recorded
phrase, then the seven sweep phrases from left rear to right rear and the
empty phrase, and fails - exit 1 - when:
  * fewer than --min-frames frames were replayed;
  * the heading change does not grow from "to your left rear " to "to your
    right rear " (mean over the frames, half a bin of tolerance, and a
    minimum left-right spread), or too few frames are monotone on their own;
  * more than 2 % of the answers are malformed;
  * the model answers LAND under a hint away from the run's goal (an
    approach run, with no goal, allows none);
  * too many steps descend a lot, or toward the ground, away from the goal;
  * any request goes unanswered.
It passes a scripted server that steers by the phrase and lands only on "",
and it never sends the goal: the requests carry the phrase only. A server
that is not the model the flight recorded is NOT COMPARABLE (exit 3; with
--frames-only the frames alone are gated), a stand-in is never a pass (exit
4, no PASS printed), and a gate that could not run exits 2 - from the real
command line too. No model, GPU or simulator: tests/vla_fake_server.py
answers.

Shown failing first: tools/vla_replay_gate.py did not exist before this
change, so every test failed at import. The review of the CityLife flight
(P2-7) items failed on 46ca291: 6 frames passed, the sweep had no rear
phrase, another model's answers passed for the flight's, a stand-in printed
PASS and exited 0, and nothing judged a step's descent.
"""
from __future__ import annotations

import hashlib
import io
import json
import contextlib
import socket
import subprocess
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


def steering(sign=1.0, land_on=("",), parse_fail_every=0, error=False, down=0.0):
    """A scripted AerialVLA: the heading change set by the phrase (times
    `sign`), LAND on the phrases in `land_on`, a malformed answer every
    `parse_fail_every`-th request, nothing but server errors, or `down`
    metres of descent on every step."""
    n = {"k": 0}

    def respond(req):
        n["k"] += 1
        if error:
            return F.error_reply(req, "inference failed: CUDA out of memory")
        if parse_fail_every and n["k"] % parse_fail_every == 0:
            return _ours(F.parse_fail_reply(req))
        if req["phrase"] in land_on:
            return _ours(F.land_reply(req))
        return _ours(F.step_reply(req, 4.0, down, sign * DPSI[req["phrase"]]))
    return respond


def _run_dir(poses, goal=GOAL, mission="goto", phrase="straight ahead ", with_frames=True,
             identity=MODEL):
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
    if identity is not None:
        m["pilot"] = {"kind": "vla", "server_identity": dict(identity)}
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


FAR = [(42.0 + k, -18.0 + 3 * k) for k in range(12)]
OTHER = dict(MODEL, weights_sha256=hashlib.sha256(b"run2 fine-tune").hexdigest())


def test_a_model_that_steers_by_the_phrase_and_lands_only_on_empty_passes():
    run = _run_dir(FAR)
    rep, reqs = _gate(steering(), run)
    assert rep["checks_pass"], json.dumps(rep["checks"], indent=1)
    assert rep["frames"] == 12 and rep["input"]["distinct_pairs"] == 12
    assert rep["checks"]["frames"] == {"pass": True, "used": 12, "distinct_pairs": 12, "min": 10}
    assert rep["requests"] == 12 * (1 + 7 + 1) == len(reqs)
    m = rep["checks"]["monotonic"]
    assert m["mean_dpsi_rad"]["to your left "] < 0 < m["mean_dpsi_rad"]["to your right "]
    assert m["mean_dpsi_rad"]["to your left rear "] < m["mean_dpsi_rad"]["to your left "]
    assert m["frame_monotone_share"] == 1.0 and rep["no_hint_land_rate"] == 1.0
    assert rep["recorded_replay"]["n"] == 12 and rep["identity"]["id"] == MODEL["id"]
    assert rep["checks"]["plausible_steps"]["pass"]
    for q in reqs:                                 # the goal never travels
        assert not ({"target", "goal", "goal_xy", "xy"} & set(q))
        assert str(GOAL[0]) not in q["object"]
    assert {q["phrase"] for q in reqs} == set(G.SWEEP) | {"", "straight ahead "}
    assert {"to your left rear ", "to your right rear "} <= set(G.SWEEP)
    assert rep["input"]["flight_identity"]["weights_sha256"] == MODEL["weights_sha256"]


def test_an_inverted_or_flat_phrase_response_fails_the_monotonic_check():
    run = _run_dir(FAR)
    rep, _ = _gate(steering(sign=-1.0), run)
    assert not rep["checks_pass"] and not rep["checks"]["monotonic"]["pass"]
    assert rep["checks"]["parse_fail"]["pass"] and rep["checks"]["answered"]["pass"]
    rep, _ = _gate(steering(sign=0.0), run)
    assert not rep["checks"]["monotonic"]["pass"]
    assert rep["checks"]["monotonic"]["spread_rad"] == 0.0
    # A rear phrase that turns the wrong way breaks the seven-phrase order.
    wrong = dict(DPSI, **{"to your right rear ": -1.0})

    def rear_wrong(req):
        if req["phrase"] == "":
            return _ours(F.land_reply(req))
        return _ours(F.step_reply(req, 4.0, 0.0, wrong[req["phrase"]]))
    rep, _ = _gate(rear_wrong, run)
    assert not rep["checks"]["monotonic"]["pass"] and rep["checks"]["monotonic"]["spread_rad"] > 0.1


def test_too_few_frames_and_steps_toward_the_ground_fail_the_gate():
    """Six frames are not a gate (at least --min-frames, 10, and the report
    says how many it used); a model that descends 3 m on every step far
    from the goal fails plausible_steps, and the same descent beside the
    goal does not count."""
    rep, _ = _gate(steering(), _run_dir(FAR[:6]))
    assert not rep["checks"]["frames"]["pass"] and rep["checks"]["frames"]["used"] == 6
    assert not rep["checks_pass"]
    rep, _ = _gate(steering(down=3.0), _run_dir(FAR))
    ps = rep["checks"]["plausible_steps"]
    assert not ps["pass"] and ps["share"] > 0.9 and ps["down_m_max"] > 2.9, ps
    assert "large_descent" in ps["examples"][0]["flags"]
    near = [(GOAL[0] - 1.0 - 0.3 * k, GOAL[1] - 2.0) for k in range(12)]
    rep, _ = _gate(steering(down=3.0), _run_dir(near))
    assert rep["checks"]["plausible_steps"]["pass"], rep["checks"]["plausible_steps"]
    rep, _ = _gate(steering(down=-1.0), _run_dir(FAR))          # climbing is plausible
    assert rep["checks"]["plausible_steps"]["pass"]


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
    assert not pf["pass"] and pf["rate"] > 0.02 and not rep["checks_pass"]
    rep, _ = _gate(steering(error=True), _run_dir(FAR))
    assert not rep["checks"]["answered"]["pass"] and not rep["checks_pass"]
    assert rep["checks"]["answered"]["unanswered"] == rep["requests"]


def _cli(args, model=MODEL, respond=None):
    """G.main on the fake server: (exit code, stdout, stderr)."""
    with F.FakeVLAServer(respond or steering(), model=model) as srv:
        with contextlib.redirect_stdout(io.StringIO()) as out, \
                contextlib.redirect_stderr(io.StringIO()) as err:
            code = G.main(args + ["--vla-server", f"127.0.0.1:{srv.port}"])
    return code, out.getvalue(), err.getvalue()


def test_the_cli_writes_its_report_and_says_pass_fail_or_no_input():
    run = _run_dir(FAR)
    code, out, _ = _cli(["--run", str(run)])
    assert code == G.EXIT_PASS == 0, out
    rep = json.loads((run / "replay_gate.json").read_text(encoding="utf-8"))
    assert rep["pass"] and rep["status"] == "pass" and rep["frames"] == 12
    assert rep["server"]["mode"] == "attached" and rep["comparison"]["match"] is True
    assert "GATE PASSED" in out and "12 frames from" in out
    assert b"\r\n" not in (run / "replay_gate.json").read_bytes()
    code, out, _ = _cli(["--run", str(run), "--max-frames", "3"])        # too few frames
    assert code == 1 and "FAIL  frames: used=3" in out, out
    code, _, _ = _cli(["--run", str(run)], respond=steering(sign=-1.0))
    assert code == 1
    # a stand-in identity is refused unless asked for, and is never a pass
    code, _, err = _cli(["--run", str(run)], model=F.MODEL,
                        respond=F.goto_responder((0, 0), 10.0))
    assert code == G.EXIT_NO_RUN == 2 and "not AerialVLA" in err
    code, out, _ = _cli(["--run", str(run), "--allow-stand-in", "--frames-only"],
                        model=F.MODEL, respond=lambda q: steering()(q) | {"model": F.MODEL})
    rep = json.loads((run / "replay_gate.json").read_text(encoding="utf-8"))
    assert code == G.EXIT_STAND_IN == 4 and rep["status"] == "stand_in" and not rep["pass"], out
    assert rep["checks_pass"] and "PASS" not in out and "STAND-IN" in out, out
    bare = _run_dir(FAR, with_frames=False)
    with contextlib.redirect_stderr(io.StringIO()) as err:
        assert G.main(["--run", str(bare), "--vla-server", "127.0.0.1:1"]) == 2
    assert "no recorded camera pair" in err.getvalue()
    with contextlib.redirect_stderr(io.StringIO()):
        assert G.main(["--run", str(run)]) == 2           # neither attach nor start


def test_a_server_that_is_not_the_flights_model_is_not_comparable():
    """The flight recorded one model (metrics.json pilot.server_identity);
    a gate served by another - here the same adapter id with other weights,
    a fine-tune - is NOT COMPARABLE (exit 3), never a pass for that flight;
    with --frames-only the frames alone are gated and it may pass. A run
    that recorded no identity is not comparable either."""
    run = _run_dir(FAR)
    code, out, _ = _cli(["--run", str(run)], model=OTHER,
                        respond=lambda q: steering()(q) | {"model": OTHER})
    rep = json.loads((run / "replay_gate.json").read_text(encoding="utf-8"))
    assert code == G.EXIT_NOT_COMPARABLE == 3, out
    assert rep["status"] == "not_comparable" and rep["checks_pass"] and not rep["pass"]
    assert rep["comparison"]["match"] is False and "NOT COMPARABLE" in out
    code, out, _ = _cli(["--run", str(run), "--frames-only"], model=OTHER,
                        respond=lambda q: steering()(q) | {"model": OTHER})
    rep = json.loads((run / "replay_gate.json").read_text(encoding="utf-8"))
    assert code == 0 and rep["frames_only"] and rep["recorded_replay"]["n"] == 0, out
    assert rep["comparison"]["match"] is None and rep["requests"] == 12 * 8
    code, out, _ = _cli(["--run", str(_run_dir(FAR, identity=None))])
    assert code == 3 and "records no server identity" in out, out


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_the_gate_that_could_not_run_exits_non_zero_from_the_command_line():
    """Review of the CityLife flight, P2-7: a hello that never came printed
    "the gate could not run: TimeoutError ..." and the shell saw exit 0.
    The real command line, as a process: nothing listens on the port, a
    1 s hello timeout - exit 2, the reason on stderr, the wait reported -
    and the default hello timeout is long enough for a bf16 cold load."""
    run = _run_dir(FAR)
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "vla_replay_gate.py"),
                        "--run", str(run), "--vla-server", f"127.0.0.1:{_free_port()}",
                        "--hello-timeout-s", "1"],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 2, (r.returncode, r.stdout, r.stderr)
    assert "the gate could not run: TimeoutError" in r.stderr, r.stderr
    assert not (run / "replay_gate.json").exists()
    ap = G.build_arg_parser()
    assert ap.parse_args(["--run", "x"]).hello_timeout_s >= 1800


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
