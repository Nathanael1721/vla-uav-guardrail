"""
The 8 Oct red-car follow on the Project AirSim + ArduPilot rail, replayed.

demo/out/pas_ap_20261008_224440 (120 s, 1101 ticks, Shield + escalation FSM
commanding ArduPilot) showed two defects:

  * 5 ticks flew a P0 stand-off violation. The perception node had put the
    car exactly under the aircraft, and the Shield could find no action that
    opened a range of 0 (docs/FINDING-the-escape-that-stayed-inside-the-ring.md).
  * the FSM went Brake -> Loiter eight times: five for stand-off repairs "of
    3.6-9.8 m" over theta, three for the range-0 ticks. Four of those five
    sizes were StandoffHold extrapolating the range in a straight line
    through the subject, deeper than the 5 m ring itself; it now reports the
    forecast's closest approach. Theta stays on each repair's own depth (the
    default basis; the one-period reading is a PI question,
    docs/DESIGN-escalation-fsm.md "Theta: two readings, one default").

These tests replay the logged state, subject and raw action of every tick
through the Shield and the FSM. That the replay's decisions are the ones the
Shield made on it at 4418ae0, tick for tick, is pinned in
tests/test_shieldcore_identity.py.

WHERE THE FLIGHT COMES FROM. The run folder is not in git (demo/out is
ignored), so until 2026-10-12 every test here reported SKIP on every CI run:
the regression for the 8 Oct fix had never run on GitHub. tests/fixtures/
now holds the flight trimmed to what the replay reads (every tick - the FSM's
N-in-T and recovery timers need the whole timeline - and only the row fields
LOG_FIELDS lists), its policy, and the CityLife flight-band map the run
used. When the run folder and the map are on disk the full log is read
instead, and `test_the_fixture_is_the_flight_trimmed` checks that the two
agree field for field. Regenerate the fixtures from the run folder with

    python tests/test_flight_pas_ap_20261008.py --regenerate-fixtures

Run:  python tests/test_flight_pas_ap_20261008.py
"""
import gzip
import io
import json
import math
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "demo" / "pas_ardupilot"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np                                                      # noqa: E402

from guardrail import fsm as F                                          # noqa: E402
from guardrail.models import Action4D, Policy, State                    # noqa: E402
from guardrail.replay import rail_shield                                # noqa: E402
from guardrail.shield import LOOKAHEAD_DT_S, LOOKAHEAD_S                # noqa: E402

RUN = ROOT / "demo" / "out" / "pas_ap_20261008_224440"
CITYMAP = ROOT / "demo" / "out" / "citymap_citylife"
FIXTURES = ROOT / "tests" / "fixtures"
FIX_LOG = FIXTURES / "pas_ap_20261008_flight_log.slim.jsonl.gz"
FIX_POLICY = FIXTURES / "pas_ap_20261008_policy_g0.json"
FIX_MAP = FIXTURES / "citylife_occ_flightband_6to14.npz"
FIX_AUDIT = FIXTURES / "pas_ap_20261008_audit.short.jsonl"
# What the replay reads of a row: the Shield's inputs (pose, raw action,
# subject), what the flight logged it decided (for the FSM replay of the old
# decisions, fsm.tick_input_from_row) and the flight's own FSM edge.
LOG_FIELDS = ("t", "tick", "x", "y", "up", "yaw_deg", "raw", "emitted", "subject",
              "violations", "repairs", "emitted_violations", "relaxed", "braked",
              "unsafe", "unsafe_rules", "unsafe_depth_m", "fsm_edge")
ESCAPE_TICKS = (857, 927, 928, 999, 1100)
SKIP = "SKIP"


def _slim(row: dict) -> dict:
    return {k: row[k] for k in LOG_FIELDS if k in row}


def _full_inputs():
    """(policy, rows, obstacle map) from the run folder, or None."""
    if not (RUN / "flight_log.jsonl").is_file() or not (CITYMAP / "occ_day.npz").is_file():
        return None
    try:
        import occ_bands
    except Exception:                                            # noqa: BLE001
        return None
    pol = Policy.model_validate(json.loads((RUN / "policy_g0.json").read_text(encoding="utf-8")))
    rows = [json.loads(l) for l in
            (RUN / "flight_log.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    sel = occ_bands.select_for_band(CITYMAP, 6.0, 14.0)
    if sel["map"] is None:
        return None
    m = sel["map"]
    return pol, rows, {"occ": m["occ"], "res": m["res"], "ox": m["ox"], "oy": m["oy"]}


def _fixture_inputs():
    """The same three from tests/fixtures/, or None when they are missing."""
    if not (FIX_LOG.is_file() and FIX_POLICY.is_file() and FIX_MAP.is_file()):
        return None
    pol = Policy.model_validate(json.loads(FIX_POLICY.read_text(encoding="utf-8")))
    with gzip.open(FIX_LOG, "rt", encoding="utf-8") as fh:
        rows = [json.loads(l) for l in fh if l.strip()]
    with np.load(FIX_MAP) as z:
        smap = {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["ox"]),
                "oy": float(z["oy"])}
    return pol, rows, smap


def _inputs():
    """(policy, rows, obstacle map, source): the full run folder when it is
    on disk, else the committed fixtures; None when neither is."""
    got = _full_inputs()
    if got is not None:
        return (*got, "run folder")
    got = _fixture_inputs()
    return None if got is None else (*got, "fixture")


def _subject(sh, row):
    """The subject as the node set it. The log rounds it to 1 cm and the state
    to 1 mm, so a logged range under 1 cm is the node's horizontal range of 0:
    the subject exactly under the aircraft."""
    s = row["subject"]
    if s is None:
        sh.set_subject(None)
    elif math.hypot(s[0] - row["x"], s[1] - row["y"]) < 0.01:
        sh.set_subject(row["x"], row["y"], "vehicle")
    else:
        sh.set_subject(s[0], s[1], "vehicle")


def _replay(pol, rows, smap, horizon_s, **shield_kw):
    """Every tick through the Shield and the rail's FSM, as perception_node.py
    steps them. Returns (P0 escape ticks, FSM edge counts, ticks over theta).
    Counterfactual after the first divergence: the logged trajectory is the
    one the flight's own FSM produced (LOITER sank the aircraft)."""
    sh = rail_shield(pol, lookahead_s=LOOKAHEAD_S, dt=LOOKAHEAD_DT_S, obstacle_map=smap,
                     **shield_kw)
    fsm = F.EscalationFSM()
    p0 = {c.id for c in pol.constraints if c.priority == "P0"}
    escapes, edges, over = [], Counter(), 0
    for r in rows:
        st = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"])
        _subject(sh, r)
        d = sh.filter(st, Action4D(**r["raw"]), t=r["t"])
        if any(v.rule_id in p0 for v in d.emitted_violations):
            escapes.append(r["tick"])
        unsafe = sh.state_is_unsafe(st)
        o = fsm.step(F.tick_input_from_decision(
            r["t"], d, pol, horizon_s=horizon_s, stop_illegal=unsafe,
            unsafe_depth_m=sh.unsafe_depth(st, unsafe),
            caps=sh.applied_caps or False))
        over += bool(o.record.get("theta_exceeded"))
        if o.edge:
            edges[o.edge] += 1
    return escapes, edges, over


def test_the_five_escape_ticks_are_gone_on_replay():
    """The old shield.py replays this log with P0 escapes on exactly the five
    ticks the flight logged (857, 927, 928, 999, 1100), and agrees with the
    log on every one of the 1101 ticks. The fixed one has none, whichever
    basis theta is measured on (the escape fix is in the Shield's check and
    recovery, not in the FSM)."""
    got = _inputs()
    if got is None:
        print("      SKIP: neither demo/out/pas_ap_20261008_224440 nor tests/fixtures holds the flight")
        return SKIP
    pol, rows, smap, _ = got
    assert len(rows) == 1101, len(rows)
    logged = [r["tick"] for r in rows if r["emitted_violations"]]
    assert logged == list(ESCAPE_TICKS), logged
    for h in (F.DEFAULT_THETA_HORIZON_S, F.THETA_HORIZON_S):
        escapes, _, _ = _replay(pol, rows, smap, h)
        assert escapes == [], (h, escapes)


def test_brake_to_loiter_on_the_replayed_flight_before_and_after():
    """The FSM replay of the logged decisions (the old Shield's) on the basis
    the flight used, per-operator magnitude_m, reproduces the flight's own
    edges: X1 7, G3 8, G8 7, G1 3, G7 2. With the fixed Shield on the same,
    default basis, Brake -> Loiter falls from 8 to 2 and the repaired ticks
    over theta from 23 to 19: every one a StandoffHold whose forecast's
    closest approach goes 2.1-4.3 m into the 5 m ring (a head-on approach
    the reference escalates "rather than projected"), none deeper than the
    ring. The one-period reading (opt-in) would trust all of them: no
    Brake -> Loiter, nothing over theta."""
    got = _inputs()
    if got is None:
        print("      SKIP: neither demo/out/pas_ap_20261008_224440 nor tests/fixtures holds the flight")
        return SKIP
    pol, rows, smap, _ = got
    flown = Counter(r["fsm_edge"] for r in rows if r.get("fsm_edge"))
    assert flown == {"X1": 7, "G3": 8, "G8": 7, "G1": 3, "G7": 2}, flown
    before = F.replay_rows(rows, horizon_s=None, policy=pol)
    assert before["edge_counts_counterfactual"] == dict(sorted(flown.items())), before
    assert before["repaired_over_theta"] == 23, before["repaired_over_theta"]
    escapes, edges, over = _replay(pol, rows, smap, F.DEFAULT_THETA_HORIZON_S)
    assert not escapes and edges["G3"] == 2 and edges["G2"] == 0 and over == 19, \
        (escapes, edges, over)
    escapes, edges, over = _replay(pol, rows, smap, F.THETA_HORIZON_S)
    assert edges["G2"] + edges["G3"] == 0 and over == 0, (edges, over)
    assert not escapes and set(edges) <= {"G1", "G7"}, edges


def test_the_fixture_is_the_flight_trimmed():
    """Where the run folder is on disk: the committed fixture is its log with
    LOG_FIELDS kept, row for row, the same policy and the same map."""
    full, fix = _full_inputs(), _fixture_inputs()
    if full is None or fix is None:
        print("      SKIP: needs both the run folder and tests/fixtures")
        return SKIP
    assert full[0].model_dump() == fix[0].model_dump()
    assert [_slim(r) for r in full[1]] == fix[1]
    for k in ("occ", "res", "ox", "oy"):
        assert np.array_equal(np.asarray(full[2][k]), np.asarray(fix[2][k])), k


def regenerate_fixtures() -> list[Path]:
    """Write the fixtures from the run folder (refuses without it)."""
    got = _full_inputs()
    if got is None:
        raise SystemExit("the run folder or the CityLife map is not on disk")
    pol, rows, smap = got
    FIXTURES.mkdir(parents=True, exist_ok=True)
    text = "".join(json.dumps(_slim(r), separators=(",", ":")) + "\n" for r in rows)
    buf = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buf, mtime=0) as gz:
        gz.write(text.encode("utf-8"))            # mtime 0: the bytes are reproducible
    FIX_LOG.write_bytes(buf.getvalue())
    with open(FIX_POLICY, "w", encoding="utf-8", newline="\n") as fh:
        fh.write((RUN / "policy_g0.json").read_text(encoding="utf-8").strip() + "\n")
    np.savez_compressed(FIX_MAP, occ=np.asarray(smap["occ"]), res=float(smap["res"]),
                        ox=float(smap["ox"]), oy=float(smap["oy"]))
    # A short audit of the same flight: its first six records, the five
    # escape ticks' and the last (tests/test_shieldcore_audit.py).
    aud = [json.loads(l) for l in (RUN / "audit.jsonl").read_text(encoding="utf-8")
           .splitlines() if l.strip()]
    keep = aud[:6] + [a for a in aud if a["tick"] in ESCAPE_TICKS] + aud[-1:]
    seen, short = set(), []
    for a in keep:
        if a["tick"] not in seen:
            seen.add(a["tick"])
            short.append(a)
    with open(FIX_AUDIT, "w", encoding="utf-8", newline="\n") as fh:
        fh.writelines(json.dumps(a) + "\n" for a in short)
    return [FIX_LOG, FIX_POLICY, FIX_MAP, FIX_AUDIT]


if __name__ == "__main__":
    if "--regenerate-fixtures" in sys.argv:
        for p in regenerate_fixtures():
            print(f"wrote {p.relative_to(ROOT)} ({p.stat().st_size} bytes)")
        sys.exit(0)
    got = _inputs()
    print(f"flight source: {got[3] if got else 'none'}")
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
