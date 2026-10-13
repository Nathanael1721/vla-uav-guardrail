"""The REST hot-apply API on a flying Shield: guardrail.api's hooks
(build_app lock / on_applied / refuse_when), FlightHotApply, and the
pymavlink rail's --hotapply-port.

    python tests/test_railinputs_hotapply.py      (no pytest needed)

Until 2026-10-12 only demo/run_demo.py served the hot-apply API; no SITL rail
did, so the grant's hot-apply channel never reached an aircraft ArduPilot
flew. The routes and bodies are unchanged (tests/test_api.py); what is new is
that an accepted event is recorded as a generation by the rail that flies,
under the rail's own lock, with its receipt-to-swap time, and that an episode
that has ended takes no more events. The ROS 2 node's listener is tested in
tests/test_railinputs_node.py.

FlightHotApply runs everywhere; the routes need fastapi (+ uvicorn and httpx
for the rail flight): the 3.11 env. Elsewhere those tests print SKIP.

Shown failing first against a pristine HEAD 4418ae0 copy (git archive):
build_app takes no hooks (TypeError), FlightHotApply does not exist
(ImportError), and run_sitl_demo.py refuses --hotapply-port (argparse).
"""
from __future__ import annotations

import json
import socket
import sys
import tempfile
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sitl"))
sys.path.insert(0, str(ROOT / "tests"))

from guardrail import load_policy                                    # noqa: E402
from guardrail import api as A                                       # noqa: E402
from guardrail.compiler import ConstraintCompiler                    # noqa: E402
from guardrail.models import Action4D, State                         # noqa: E402
from guardrail.replay import EpisodeRecord, rail_shield              # noqa: E402

POLICY = ROOT / "policies" / "sim_demo_policy.yaml"
SKIP = "SKIP"
ZONE = {"id": "nfz-rest", "vertices": [{"x": 40, "y": 40}, {"x": 50, "y": 40},
                                       {"x": 50, "y": 50}, {"x": 40, "y": 50}]}


def _flying():
    pol = load_policy(POLICY)
    mission = ConstraintCompiler(pol).parse_command("fly to the northeast pad at 6 m/s")
    d = Path(tempfile.mkdtemp(prefix="hot_"))
    rec = EpisodeRecord(d, pol, mission, lookahead_s=5.0)
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1)
    sh.filter(State(x=0, y=0, up=15), Action4D())           # mission started
    return sh, rec, d


def _client(sh, **hooks):
    try:
        from fastapi.testclient import TestClient
    except ImportError:
        return None
    return TestClient(A.build_app(sh, **hooks))


def test_an_accepted_event_is_recorded_as_a_generation_with_its_latency():
    sh, rec, d = _flying()
    lock = threading.RLock()
    seen = []
    hot = A.FlightHotApply(sh, rec, lock, mission_t=lambda: 3.456,
                           after=lambda res, t: seen.append((res["applied"], t)))
    res = A.apply_dynamic_nfz(sh, dict(ZONE))
    entry = hot.on_applied(res, received_ns=1_000_000, applied_ns=3_500_000)
    assert entry["receipt_to_swap_ms"] == 2.5 and entry["generation"] == 1
    assert (d / "policy_g1.json").is_file() and (d / "csp_g1.json").is_file()
    assert [g["generation"] for g in rec.generations] == [0, 1]
    assert seen == [("nfz-rest", 3.456)]
    assert hot.live_from == {"nfz-rest": 3.46}           # the trajectory's grid
    ev = [json.loads(x) for x in (d / "events.jsonl").read_text(
        encoding="utf-8").splitlines() if '"hot_apply_rest"' in x]
    assert ev[0]["rule_id"] == "nfz-rest" and ev[0]["t"] == 3.456
    s = hot.summary()
    assert s["n_applied"] == 1 and s["receipt_to_swap_ms_p50"] == 2.5
    assert s["spawn_target_ms"] == 50.0 and s["spawns_over_target"] == 0
    hot.close("over")
    assert hot.refuse_when() == "over"


def test_the_app_runs_the_hook_under_the_callers_lock_and_refuses_when_told():
    sh, rec, d = _flying()
    lock = threading.RLock()
    calls = []

    def on_applied(res, *, received_ns, applied_ns):
        # Under the caller's lock: a control tick cannot run beside it.
        assert lock._is_owned()
        calls.append((res["generation"], applied_ns >= received_ns))
    closed = [None]
    c = _client(sh, lock=lock, on_applied=on_applied,
                refuse_when=lambda: closed[0])
    if c is None:
        return SKIP
    r = c.post("/dynamic_nfz", json=ZONE)
    assert r.status_code == 200 and calls == [(1, True)], (r.text, calls)
    assert c.get("/health").status_code == 200 and len(calls) == 1   # reads: no hook
    assert c.post("/dynamic_nfz", json=ZONE).status_code == 409 and len(calls) == 1
    closed[0] = "the episode ended"
    r = c.post("/dynamic_nfz", json={**ZONE, "id": "late"})
    assert r.status_code == 503 and "ended" in r.json()["detail"]
    assert sh.policy.generation == 1, "a refused event touched the Shield"


def test_a_failing_hook_does_not_turn_an_applied_event_into_a_refusal():
    sh, rec, d = _flying()

    def broken(res, **kw):
        raise RuntimeError("disk full")
    c = _client(sh, on_applied=broken)
    if c is None:
        return SKIP
    r = c.post("/dynamic_nfz", json=ZONE)
    assert r.status_code == 200 and sh.policy.generation == 1, r.text


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_the_pymavlink_rail_takes_a_hot_apply_between_two_ticks():
    """The rail's loop holds its lock except while it sleeps; a request sent
    from inside the stand-in sleep (the lock released) is applied, recorded
    as generation 1 with its mission time, and the rows after it carry
    generation 1. A zone spawned across the route counts from its arrival."""
    try:
        import fastapi  # noqa: F401
        import httpx
        import uvicorn  # noqa: F401
    except ImportError:
        return SKIP
    import test_sitl_rails as T
    port = _free_port()
    sent = []

    def patch(R):
        real_sleep = R.time.sleep

        def sleep(dt):
            real_sleep(dt)
            if not sent and R.time.time() >= 3.0:
                sent.append(httpx.post(f"http://127.0.0.1:{port}/dynamic_nfz",
                                       json={"id": "nfz-rest", "vertices": [
                                           {"x": 24, "y": 24}, {"x": 28, "y": 24},
                                           {"x": 28, "y": 28}, {"x": 24, "y": 28}]},
                                       timeout=10))
        R.time.sleep = sleep
    out, metrics, rows, events, sim, R = T._fly_pymavlink_rail(
        ["--shield", "off", "--hotapply-port", str(port)], patch)
    assert sent and sent[0].status_code == 200, sent and sent[0].text
    ha = metrics["hot_apply"]
    assert ha["n_applied"] == 1 and ha["events"][0]["t"] >= 3.0, ha
    gens = [r["generation"] for r in rows]
    k = gens.index(1)
    assert set(gens[:k]) == {0} and set(gens[k:]) == {1}, gens
    assert rows[k]["t"] >= ha["events"][0]["t"]
    assert (out / "policy_g1.json").is_file()
    assert "nfz-rest" in metrics["nfz_by_zone_s"], metrics["nfz_by_zone_s"]
    assert any(e["kind"] == "hot_apply_listener" for e in events)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (needs fastapi, uvicorn, httpx: the 3.11 env)")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        # SystemExit too: a rail whose parser refuses a flag exits, and
        # that is this test failing, not the run ending.
        except (Exception, SystemExit) as e:         # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
