"""
The AerialVLA follow's re-acquisition after a long loss (demo/vla_fly.py
_acquire_step, make_acquirer, --acquire-gap-s; follow_vlm.Reacquirer
gap_s), replayed on the flight that found the fault.

Run either way:
    pytest tests/test_vla_follow_reacquire.py -v
    python tests/test_vla_follow_reacquire.py

WHAT WENT WRONG (vla_follow_ft_07_r3_off_20261024, flown on 834288b)

The car was acquired at 3.4 s and lost at 39.3 s ("unseen 3.0 s"). From
52 s to the end (637 s, target_lost) the aircraft hovered at (52.9, 113.4)
with the acquisition open - after ~72 s with no phrase left to ask with
(the hint's memory and search spent) - while the car drove past it twice,
7-26 m away: OWL-ViT boxes up to 0.67, identity OK, in range. Nothing was
committed, no box was published again, AerialVLA was not asked again, and
metrics said only "reacquires 1". Replayed through the same Reacquirer,
frame by frame:

  * 14 s after the last sighting (at 50 s) the reach (10 m + 3.47 m/s x
    dt) passed reach_cap_m (60 m); from then on the start gate's evidence
    applies:
    identity OK and SEEN MOVING, the median point of the streak's second
    half beyond the noise-aware bar from its first, over at least
    2 x need = 10 sightings IN A ROW.
  * The detector ran at ~2.2 Hz while acquiring. Pass 1: 7 OK inferences
    in a row (captured 280.7-283.5 s, 57.7 -> 63.0 m north), one with no
    car, 5 more (285.5-287.4 s, 70.1 -> 75.3). Pass 2: 9 in a row
    (530.7-534.8 s, 57.4 -> 68.2), three with no car, 3 more. The frame
    with nothing acceptable ended each streak, so the moving test was never
    asked - though by its own bar the first 7 had moved 3.35 m against
    2.86 m and the first 9 4.42 m against 3.95 m.
  * And SILENTLY: a streak too short to judge returns False with no
    refusal, so `refused` counted "not ok" and nothing else for 600 s. No
    "out of reach", "too far", "static" or "discontinuous" in either pass:
    the anchor, its time base and the acquisition's restart were right.

WHAT IS PINNED HERE

  1. THE FLIGHT, from tests/fixtures/vla_follow_ft_07_reacquire.slim.json
     (its inferences 1-1245 with what _acquire_step reads from the
     Grounder) through vla_fly._acquire_step: the replay reproduces it up
     to the lapse (the start gate's commit, the lapse anchored where it was
     flown), and with the rule as flown commits nothing afterwards.
  2. THE FIX: vla_fly builds its Reacquirer with --acquire-gap-s (default
     LAPSE_S, 3 s), so the car is re-committed on its first pass, at 287.1
     s; the real Grounder's strict lock then holds it on the next frames and
     the DetectorHint hands AerialVLA a fresh phrase again. The 243 s hover
     between the passes commits nothing (a standing red thing at (62.4,
     116.3) is seen in runs of up to five), and pass 2 alone is taken too.
  3. THE RULE (follow_vlm.Reacquirer gap_s) on synthetic sightings: 0 is
     follow_vlm's own rule, unchanged (one empty frame ends a streak; the
     fly() loop never passes gap_s); with gaps a pass is judged moving once
     it has its 10 sightings, a gap longer than gap_s or a jump beyond
     step_m + v_r dt still ends it, an anchored short loss is judged as
     before, a standing jittering sign seen with misses never passes while
     a car and a walker with the same misses do, and a streak too short to
     judge is not marked static however long it spans.
  4. THE RECORDS: every judged frame's outcome (the Reacquirer's
     last_outcome) and every candidate refusal, per attempt (the start
     gate, each lapse, each perception reset), in metrics.follow.
     acquisition, summed over the flight; --acquire-gap-s is refused
     negative and recorded in params.follow.start_gate.

Shown failing first: with 834288b's demo/vla_fly.py and demo/follow_vlm.py
(the tree that flew) 0/13 pass - make_acquirer, gap_s, last_outcome, the
attempts and the flag do not exist. With make_acquirer stood in by the
construction that flew (no gaps), that tree's _acquire_step over records
1-1245 commits the start (3.35 s) and nothing else, lapse anchored at
(58.34, 118.75), and the first-pass test fails on its missing commit.

No simulator, GPU or model: the flight's records, follow_vlm's real
Reacquirer, Grounder state and strict TargetLock (OWL-ViT never loaded).
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import random
import statistics
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "tests", ROOT / "demo", ROOT / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import follow_vlm as FV                                              # noqa: E402
import vla_fly                                                       # noqa: E402

SKIP = "SKIP"
FIXTURE = ROOT / "tests" / "fixtures" / "vla_follow_ft_07_reacquire.slim.json"
HOVER_XY = (52.92, 113.39)          # where the aircraft hovered after the lapse
PASS1 = (280.0, 288.0)              # capture times of the car's passes (mission s)
PASS2 = (530.0, 538.5)


def _flight():
    """The fixture: (its header, its inferences as dicts in the Grounder's
    terms: 8-tuple candidates, tiers, features {P, rng_h})."""
    doc = json.loads(FIXTURE.read_text(encoding="utf-8"))
    W, H = doc["img"]
    recs = []
    for r in doc["records"]:
        seq, t, tc = r[:3]
        det_k, lock_why, yaw, cands = (r[3:] + [None, None, None, []])[:4]
        recs.append({"seq": seq, "t": t, "t_capture": tc, "det_k": det_k,
                     "lock_why": lock_why, "yaw": yaw,
                     "cands": [(c[0], 0.0, c[1], 0.0, c[2], W, H, 0.0) for c in cands],
                     "tiers": [c[3] for c in cands],
                     "feats": [{"P": None if c[4] is None else (c[4], c[5]), "rng_h": c[6]}
                               for c in cands]})
    return doc, recs


def _from_mission_start(recs):
    """The records the flight's loop read: its first tick, at mission 0,
    read the newest publication; the ones before it were never stepped."""
    k = max(i for i, r in enumerate(recs) if r["t"] <= 0.0)
    return recs[k:]


class _Replayed:
    """follow_vlm.Grounder as _acquire_step reads it (latest, commit,
    begin_reacquire), its publications taken from the flight's records: one
    inference per record, the candidates and the strict lock's answer as
    logged, the published box kept until drop_after (8 s) as the Grounder
    keeps it. OPEN LOOP: after a commit the flight's Grounder never made,
    the records still show what the flight saw (no box held)."""

    drop_after = 8.0

    def __init__(self):
        self.state = {"seq": 0, "t": 0.0, "t_det": 0.0, "det": None, "cands": [],
                      "cand_tiers": [], "cand_feats": [], "tier": None, "feat": None,
                      "cap": None, "lock_why": None, "inf_cap": None}
        self.commits: list = []
        self.reacquires = 0

    def publish(self, r) -> None:
        s = self.state
        s.update(seq=r["seq"], t=r["t"], cands=r["cands"], cand_tiers=r["tiers"],
                 cand_feats=r["feats"], lock_why=r["lock_why"],
                 inf_cap={"t_cap": r["t_capture"], "t_capture": r["t_capture"],
                          "pose": {"yaw": r["yaw"]}})
        k = r["det_k"]
        if k is not None:
            s.update(det=r["cands"][k], t_det=r["t"], tier=r["tiers"][k], feat=r["feats"][k],
                     cap={"t_cap": r["t_capture"], "t_capture": r["t_capture"]})
        elif s["t_det"] and r["t"] - s["t_det"] > self.drop_after:
            s["det"] = None

    def latest(self) -> dict:
        return dict(self.state)

    def commit(self, cand, yaw=None) -> None:
        self.commits.append({"t": self.state["t"], "seq": self.state["seq"], "cand": cand,
                             "yaw": yaw})

    def begin_reacquire(self) -> None:
        self.reacquires += 1
        self.state.update(det=None, t_det=0.0)


def _replay(recs, acq, box=None, stop_after_commits=None):
    """Each record through vla_fly._acquire_step, read at its publication
    (mission seconds, t0 = 0; the flight read each 0.05-0.1 s later).
    Returns the acquisition box and the replayed Grounder."""
    g = _Replayed()
    if box is None:
        box = vla_fly.new_acquisition(acq, t0=0.0)
    args = SimpleNamespace(follow_memory_s=6.0, follow_search_s=20.0)
    for r in recs:
        g.publish(r)
        now = max(r["t"], 0.0)
        with contextlib.redirect_stdout(io.StringIO()):
            vla_fly._acquire_step(g, box, args, clock=lambda: now)
        if stop_after_commits is not None and len(box["commits"]) >= stop_after_commits:
            break
    return box, g


def _as_flown():
    """The Reacquirer 834288b built: need 5, 60 m (-AcquireMaxRange 60, as
    flown), no gaps."""
    acq = FV.Reacquirer(need=5, max_range_m=60.0)
    acq.start(None)
    return acq


def _fixed(*extra):
    """The Reacquirer vla_fly builds now, from its CLI as flown (60 m)."""
    a = vla_fly.parse_cli(vla_fly.build_arg_parser(),
                          ["--mission", "follow", "--acquire-max-range-m", "60", *extra])
    acq = vla_fly.make_acquirer(a, FV)
    acq.start(None)
    return acq


def _ok_runs(recs, lo, hi):
    """Lengths of the runs of inferences with an OK ranged candidate within
    the capture window [lo, hi]; a frame without one ends a run."""
    runs, n = [], 0
    for r in recs:
        if not lo <= r["t_capture"] <= hi:
            continue
        if any(t == "ok" and f["P"] is not None for t, f in zip(r["tiers"], r["feats"])):
            n += 1
        elif n:
            runs.append(n)
            n = 0
    return runs + ([n] if n else [])


# =========================================================================== #
# 1. The flight, as flown
# =========================================================================== #
def test_the_replay_reproduces_the_flight_up_to_its_lapse():
    """The start gate commits within 0.05 s of the flown 3.401 s and the
    lapse is anchored exactly where it was flown, at the car's last sighting
    (58.34, 118.75) with its measured 2.29 m/s. (The replay reads each frame
    at its publication, the flight 0.05-0.1 s later, so its lapse lands one
    frame later: 39.56 s, flown 39.30.) The same with the fix: the start was
    never the problem."""
    doc, recs = _flight()
    flown = doc["flown"]
    assert flown["start_gate"] == {"kind": "identity_reacquirer", "need": 5,
                                   "max_range_m": 60.0}, flown["start_gate"]
    assert flown["commits_s"] == [3.401] and flown["reacquires"] == 1, flown
    for make in (_as_flown, _fixed):
        box, g = _replay(_from_mission_start(recs), make(), stop_after_commits=2)
        assert abs(box["commits"][0] - 3.401) < 0.06, box["commits"]
        assert g.commits[0]["seq"] == 31, g.commits[0]
        lapse, want = box["lapses"][0], flown["lapses"][0]
        assert lapse["P"] == want["P"] == [58.34, 118.75], (lapse, want)
        assert abs(lapse["v"] - want["v"]) < 0.05 and abs(lapse["t"] - want["t"]) < 0.3, lapse
        assert lapse["why"].startswith("unseen") and g.reacquires == 1, lapse


def test_as_flown_the_car_that_came_back_twice_was_never_judged():
    """The rule as flown, over the whole fixture: one commit (the start),
    then 1079 frames judged and nothing taken. The passes' OK runs are
    7 + 5 and 9 + 3 inferences, each under the 10 the moving test needs, so
    it was never asked (no "not moving", no "static"); every candidate
    refusal of the lapse's attempt is "not ok" - nothing refused the car
    itself. Had the test been asked, the first runs had already moved."""
    _, recs = _flight()
    box, g = _replay(_from_mission_start(recs), _as_flown())
    assert box["commits"] == [3.353] and len(g.commits) == 1, box["commits"]
    acq = box["acq"]                        # the anchor stopped constraining at ~50 s
    t_free = acq.anchor["t"] + (acq.reach_cap_m - acq.reach_base_m) / acq.v_reach()
    assert 49.5 < t_free < 51.0 and t_free < PASS1[0], t_free
    assert [a["kind"] for a in box["attempts"]] == ["start", "lapse"], box["attempts"]
    lapse = box["attempts"][1]
    assert lapse["committed_s"] is None and lapse["judged"] == 1079, lapse
    assert lapse["best_streak"] == 9 < 2 * 5, lapse
    assert lapse["outcomes"].get("short pass", 0) == 9, lapse["outcomes"]
    assert not {"not moving", "static", "committed", "gap"} & set(lapse["outcomes"]), lapse
    assert sum(lapse["outcomes"].values()) == lapse["judged"]
    assert lapse["refused"] == {"not ok": 103}, lapse["refused"]
    assert _ok_runs(recs, *PASS1) == [7, 5] and _ok_runs(recs, *PASS2) == [9, 3]
    # The moving test's own verdict on the first runs, had it been asked:
    by = {r["seq"]: r for r in recs}
    for seqs, moved, bar in ((range(691, 698), 3.35, 2.86), (range(1224, 1233), 4.42, 3.95)):
        rq = FV.Reacquirer(need=3, max_range_m=60.0)          # 2 x 3 <= the run
        rq.start(None)
        for s in seqs:
            rq.step(by[s]["cands"], by[s]["tiers"], by[s]["feats"], by[s]["t_capture"])
        assert rq.streak == len(seqs), rq.streak
        assert abs(rq.moved_m() - moved) < 0.01 and abs(rq.motion_bar_m() - bar) < 0.01, (
            rq.moved_m(), rq.motion_bar_m())
    # ...at the detector rate the flight had while acquiring
    hover = [r["t"] for r in recs if 300.0 <= r["t"] <= 520.0]
    rate = 1.0 / statistics.median(b - a for a, b in zip(hover, hover[1:]))
    assert 1.8 < rate < 2.6, rate


# =========================================================================== #
# 2. The fix, on the same frames
# =========================================================================== #
def test_with_the_gap_the_car_is_retaken_on_its_first_pass():
    """make_acquirer with the default --acquire-gap-s (3 s): the start gate
    as before, the lapse as before, and the car re-committed at 287.07 s on
    its first pass - the 10th OK sighting, across the 2.0 s gap - at its
    place on the street 21 m from the hovering aircraft. The attempt says
    how: the gap frames it bridged, the short-pass frames before."""
    _, recs = _flight()
    box, g = _replay(_from_mission_start(recs), _fixed(), stop_after_commits=2)
    assert box["commits"][0] == 3.353 and len(box["lapses"]) == 1, box
    t = box["commits"][1]
    assert 286.0 <= t <= 288.0, box["commits"]
    c = g.commits[1]
    assert c["seq"] == 701, c
    acq = box["acq"]
    P, rng = acq.pick_feat["P"], acq.pick_feat["rng_h"]
    assert 70.0 < P[0] < 76.0 and 118.5 < P[1] < 120.0 and rng < 25.0, acq.pick_feat
    assert math.hypot(P[0] - HOVER_XY[0], P[1] - HOVER_XY[1]) < 25.0
    att = box["attempts"][1]
    assert att["kind"] == "lapse" and att["committed_s"] == t, att
    assert att["outcomes"]["gap"] >= 1 and att["outcomes"]["short pass"] >= 1, att
    assert att["best_streak"] == 10, att


def _strict_grounder():
    """follow_vlm's REAL Grounder (not started: no OWL-ViT) with its REAL
    strict TargetLock, as vla_fly builds it with identity on."""
    with contextlib.redirect_stdout(io.StringIO()):
        return FV.Grounder(SimpleNamespace(pose=None), "a red car",
                           lock=FV.TargetLock(gate_frac=0.12), identity={"hfov": 90.0})


def _strict_inference(g, r):
    """One recorded inference through the Grounder's identity-path
    bookkeeping (follow_vlm.Grounder._worker): the seed a commit() handed
    over taken first, then - unless acquiring - TargetLock.select_strict
    (no estimator prior: vla_fly sets none), then the publication. Only the
    detector and the identity judging are left out: they are the record."""
    now, pose = r["t"], {"x": HOVER_XY[0], "y": HOVER_XY[1], "up": 7.4, "yaw": r["yaw"]}
    if g._seed is not None:
        (seed, seed_yaw), g._seed = g._seed, None
        g.lock.reset()
        g.lock.seed(seed, seed_yaw, now)
    det = None
    if r["cands"] and not g.acquiring:
        k, why = g.lock.select_strict(r["cands"], r["tiers"], r["feats"], r["cands"][0][5],
                                      r["yaw"], now, prior=None)
        if k is not None:
            det = k
    with g._lock:
        g._seq += 1
        g._t = now
        g._cands, g._cand_tiers, g._cand_feats = list(r["cands"]), r["tiers"], r["feats"]
        g._inf_cap = {"t_cap": r["t_capture"], "t_capture": r["t_capture"], "pose": pose}
        if det is not None:
            g._det, g._t_det = r["cands"][det], now
            g._det_tier, g._det_feat = r["tiers"][det], r["feats"][det]
            g._det_cap = {"t_cap": r["t_capture"], "t_capture": r["t_capture"], "pose": pose}
    return det


def test_the_retaken_car_is_held_and_its_phrase_reaches_the_vla_again():
    """Closed through follow_vlm's real Grounder state and strict lock from
    the commit on: seeded with the replay's pick at its capture yaw, the lock
    holds the car on the next two recorded frames (702, 703: published
    boxes), and vla_mission.DetectorHint, reading the Grounder as vla_fly's
    latest_box does, hands AerialVLA a fresh phrase - the request the flight
    never sent again after 39 s."""
    import vla_mission as VM
    from guardrail.vla_link import Capture, Pose
    from vla_bridge import VLABridge
    _, recs = _flight()
    box, rg = _replay(_from_mission_start(recs), _fixed(), stop_after_commits=2)
    c = rg.commits[1]
    g = _strict_grounder()
    g.begin_reacquire()
    g.commit(c["cand"], yaw=c["yaw"])

    def latest_box():
        gw = g.latest()
        if gw.get("det") is None:
            return None, None, None, None
        cap = gw.get("cap") or {}
        return gw["det"], cap.get("t_capture"), (cap.get("pose") or {}).get("yaw"), gw["t_det"]
    hint = VM.DetectorHint(latest_box, VLABridge("a red car", hfov_deg=90.0), fresh_s=1.0,
                           memory_s=6.0, search_s=20.0, record_kind=True)
    by = {r["seq"]: r for r in recs}
    for s in (702, 703):
        k = _strict_inference(g, by[s])
        assert k is not None, s
        P = by[s]["feats"][k]["P"]                           # one of the car's boxes
        assert 73.5 < P[0] < 76.0 and 118.5 < P[1] < 120.0, (s, P)
        assert g.latest()["det"] == by[s]["cands"][k] and g.latest()["tier"] == "ok"
    h = hint(Capture(by[703]["t"] + 0.1, Pose(HOVER_XY[0], HOVER_XY[1], 7.4, by[703]["yaw"]),
                     b"jpg", b"jpg"))
    assert h is not None and h.kind == "fresh" and h.source == "detector" and h.phrase, h


def test_the_hover_between_the_passes_commits_nothing_and_pass_two_is_taken_too():
    """From the flown lapse anchor, the 243 s between the passes (records
    704-1223): a standing red thing at (62.4, 116.3) seen in runs of up to
    five with gaps, single flickers - no commit, only "short pass" and
    "streak" judged. Continued through pass 2 (to record 1245), the car is
    taken at 537.5 s. The rule as flown takes neither."""
    doc, recs = _flight()
    by = {r["seq"]: r for r in recs}
    lapse = doc["flown"]["lapses"][0]
    anchor = {"t": by[157]["t_capture"], "P": tuple(lapse["P"]), "v": lapse["v"]}
    hover = [r for r in recs if 704 <= r["seq"] <= 1223]
    both = [r for r in recs if 704 <= r["seq"]]
    for make, takes in ((_fixed, True), (_as_flown, False)):
        acq = make()
        acq.start(anchor)
        box, _ = _replay(hover, acq)
        assert box["commits"] == [], (make.__name__, box["commits"])
        out = box["attempts"][0]["outcomes"]
        assert not {"committed", "static"} & set(out), out
        acq = make()
        acq.start(anchor)
        box, g = _replay(both, acq, stop_after_commits=1)
        if not takes:
            assert box["commits"] == [], box["commits"]
            continue
        assert len(box["commits"]) == 1 and 536.5 <= box["commits"][0] <= 538.5, box["commits"]
        assert g.commits[0]["seq"] == 1236, g.commits
        P = box["acq"].pick_feat["P"]
        assert 70.0 < P[0] < 78.0 and 118.0 < P[1] < 120.0, P


# =========================================================================== #
# 3. The rule
# =========================================================================== #
def _car(n, e=0.0, rng=15.0):
    return {"P": (n, e), "rng_h": rng}


BOX = (400.0, 200.0, 40.0, 25.0, 0.3, 768, 432, 0.6)


def _feed(rq, seq, t0=100.0, dt=0.45):
    """seq: per inference a feature dict (one OK candidate) or None (no
    candidate). Returns (the index of the committing inference or None,
    the outcomes in order)."""
    outs = []
    for i, f in enumerate(seq):
        got = rq.step([] if f is None else [BOX], [] if f is None else ["ok"],
                      [] if f is None else [f], t0 + i * dt)
        outs.append(rq.last_outcome)
        if got:
            return i, outs
    return None, outs


def _pass(v=2.7, dt=0.45, n=14, miss=(7,)):
    """A car driving north at v m/s seen every dt, with no candidate at all
    on the inferences in `miss`."""
    return [None if i in miss else _car(57.0 + v * dt * i) for i in range(n)]


def test_without_gaps_the_reacquirer_is_follow_vlms():
    """gap_s defaults to 0 and follow_vlm's fly() never passes it: one empty
    frame ends the streak (7 + 6 sightings never reach 10), as before; the
    outcomes name each frame's verdict."""
    rq = FV.Reacquirer(need=5)
    assert rq.gap_s == 0.0 and rq.last_outcome is None
    rq.start(None)
    k, outs = _feed(rq, _pass(n=14, miss=(7,)))
    assert k is None, outs
    assert outs == ["streak"] * 4 + ["short pass"] * 3 + ["no candidate"] + ["streak"] * 4 \
        + ["short pass"] * 2, outs
    import ast
    tree = ast.parse((ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8"))
    fly = next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    calls = [n for n in ast.walk(fly) if isinstance(n, ast.Call)
             and getattr(n.func, "id", None) == "Reacquirer"]
    assert len(calls) == 4 and not any(k.arg == "gap_s" for c in calls for k in c.keywords), (
        [ast.unparse(c) for c in calls])
    # ...and the same pass, unbroken, is taken on its 10th sighting
    rq.start(None)
    k, _ = _feed(rq, _pass(n=14, miss=()))
    assert k == 9, k


def test_with_gaps_a_pass_is_judged_moving_once_it_has_its_sightings():
    """gap_s = 3, nothing constraining the place: the empty frame is a
    "gap" - the streak kept, nothing added - and the pass is taken on its
    10th SIGHTING (the 11th inference), as the flight's first pass was. A
    gap is never a sighting: 9 sightings and any number of gaps are not
    enough."""
    rq = FV.Reacquirer(need=5, gap_s=3.0)
    rq.start(None)
    k, outs = _feed(rq, _pass(n=14, miss=(7,)))
    assert k == 10, outs
    assert outs[7] == "gap" and outs[-1] == "committed" and rq.streak == 10, outs
    assert len(rq.trace) == 10
    rq.start(None)
    k, outs = _feed(rq, _pass(n=14, miss=(2, 4, 6, 8, 10, 12, 13)))
    assert k is None and rq.streak == 7 and outs.count("gap") == 7, outs


def test_a_long_gap_or_a_jump_still_ends_the_pass():
    """Past gap_s the streak is over, as without gaps; across a short gap
    the next pick must still lie within step_m + v_r dt of the last (3 m +
    3 m/s x 1.35 s here): a box 30 m off starts a new streak, counted
    "discontinuous"."""
    rq = FV.Reacquirer(need=5, gap_s=3.0)
    rq.start(None)
    seq = [_car(57.0 + 1.2 * i) for i in range(5)] + [None] * 7 + [_car(64.0)]
    k, outs = _feed(rq, seq)                       # 7 x 0.45 = 3.15 s > 3 s
    assert k is None and outs[5:7] == ["gap", "gap"] and "no candidate" in outs, outs
    assert rq.streak == 1, rq.streak
    rq.start(None)
    seq = [_car(57.0 + 1.2 * i) for i in range(5)] + [None, None] + [_car(95.0)]
    k, outs = _feed(rq, seq)
    assert k is None and rq.streak == 1 and rq.refused.get("discontinuous") == 1, outs


def test_gaps_never_bridge_an_anchored_short_loss():
    """While the anchor still constrains the place (reach <= reach_cap_m)
    the Reacquirer commits on `need` sightings in a row without the moving
    test, and a gap there would let a standing thing near the last sighting
    through on scattered frames: an empty frame still ends the streak, as
    in follow_vlm."""
    rq = FV.Reacquirer(need=5, gap_s=3.0)
    rq.start({"t": 99.0, "P": (50.0, 0.0), "v": 3.0})       # reach 15-30 m: constrained
    seq = [_car(55.0)] * 3 + [None] + [_car(55.0)] * 3
    k, outs = _feed(rq, seq)
    assert k is None and outs[3] == "no candidate" and "gap" not in outs, outs
    rq.start({"t": 99.0, "P": (50.0, 0.0), "v": 3.0})
    k, _ = _feed(rq, [_car(55.0)] * 6)
    assert k == 4                                            # need in a row: taken, as before


def _jitter_with_misses(v, sigma, seed, gap_s=3.0, dt=0.45, miss=0.3, T=60.0):
    """Seconds until a thing moving at v m/s with sigma m of map-point
    jitter, seen on 1 - miss of the inferences at 1/dt Hz, is taken (None:
    never in T)."""
    rng = random.Random(seed)
    rq = FV.Reacquirer(need=5, gap_s=gap_s)
    rq.start(None)
    t = 0.0
    while t < T:
        if rng.random() < miss:
            rq.step([], [], [], 10.0 + t)
        else:
            f = _car(20.0 + v * t + rng.gauss(0, sigma), rng.gauss(0, sigma))
            if rq.step([BOX], ["ok"], [f], 10.0 + t):
                return t
        t += dt
    return None


def test_a_standing_sign_seen_with_misses_never_passes_and_a_car_or_walker_does():
    """The noise-aware bar is what keeps the fire-hydrant sign out
    (test_identity_lock's jitter test, at the start gate's 5.5 Hz); here at
    the VLA flight's 2.2 Hz with 30 % of frames empty and gaps bridged: a
    sign with 1-2 m of jitter is never taken in 40 simulated minutes per
    jitter, while a car at 2.7 m/s and a walker at 1.3 m/s are, every seed
    (medians 6.8 and 6.3 s). As flown, with no gaps, the same car is not
    taken within 20 s on 29 of the 40 seeds."""
    for sigma in (1.0, 1.5, 2.0):
        passed = [s for s in range(40) if _jitter_with_misses(0.0, sigma, s) is not None]
        assert not passed, (sigma, passed)
    cars = [_jitter_with_misses(2.7, 1.0, s, T=20.0) for s in range(40)]
    assert all(t is not None for t in cars) and sorted(cars)[20] < 8.0, cars
    walk = [_jitter_with_misses(1.3, 0.5, s, T=20.0) for s in range(40)]
    assert all(t is not None for t in walk) and sorted(walk)[20] < 8.0, walk
    flown = [_jitter_with_misses(2.7, 1.0, s, gap_s=0.0, T=20.0) for s in range(40)]
    assert sum(t is None for t in flown) >= 25, flown


def test_a_streak_too_short_to_judge_is_not_marked_static_however_long_it_spans():
    """With gaps, five sightings may span more than static_after_s (8 s)
    before the moving test can be asked: that is no evidence the place
    stands still, so it is not marked static (a car seen every 2.2 s at the
    edge of the frame would have been blacklisted for 30 s); once judged,
    a standing thing still is."""
    rq = FV.Reacquirer(need=5, gap_s=3.0)
    rq.start(None)
    seq = []
    for i in range(6):
        seq += [_car(40.0 + 5.0 * i)] + [None] * 4               # one sighting per 2.25 s
    k, outs = _feed(rq, seq)
    assert k is None and not rq.static and "short pass" in outs, (outs, rq.static)
    rq.start(None)
    k, outs = _feed(rq, [_car(40.0)] * 30)                       # 13 s standing, judged
    assert k is None and rq.static and "static" in outs, outs


# =========================================================================== #
# 4. The records
# =========================================================================== #
def test_metrics_count_every_judged_frame_and_every_refusal_per_attempt():
    """metrics.follow.acquisition (follow_measured): the flight-wide sums -
    judged, outcomes, refused - and each attempt with its kind, start, what
    its frames decided and when it committed; the rule as flown on this
    flight shows the 600 s of nothing as counts, not silence. A perception
    reset opens its own attempt; all of it is JSON."""
    _, recs = _flight()
    box, _ = _replay(_from_mission_start(recs), _as_flown())
    m = vla_fly.follow_measured(box, [], SimpleNamespace(n_close_hold=0))["acquisition"]
    json.dumps(m)
    assert m["commits_s"] == [3.353] and m["reacquires"] == 1
    assert m["judged"] == sum(a["judged"] for a in m["attempts"]) == 12 + 1079, m["judged"]
    assert m["outcomes"] == {"no candidate": 950, "none acceptable": 88, "streak": 38,
                             "short pass": 14, "committed": 1}, m["outcomes"]
    assert sum(m["outcomes"].values()) == m["judged"]
    assert m["refused"] == {"not ok": 105, "discontinuous": 1}, m["refused"]
    assert [(a["kind"], a["committed_s"]) for a in m["attempts"]] == [
        ("start", 3.353), ("lapse", None)], m["attempts"]
    assert m["attempts"][1]["t"] == box["lapses"][0]["t"]
    # a perception reset: the re-acquisition unanchored, its own attempt
    g = _Replayed()
    st = SimpleNamespace(x=52.9, y=113.4)
    ev = vla_fly._perception_reset("stuck", 700.0, 640.0, g, None, box, st)
    assert ev.get("reacquiring") and box["attempts"][-1]["kind"] == "reset:stuck"
    assert box["attempts"][-1]["t"] == 640.0 and box["attempt"] is box["attempts"][-1]
    # a fresh acquisition box without a Reacquirer records none
    assert vla_fly.follow_measured(vla_fly.new_acquisition(None), [], None)["acquisition"] is None


def test_the_gap_is_a_flag_refused_negative_and_recorded_with_the_start_gate():
    """--acquire-gap-s: default LAPSE_S (3 s, ACQUIRE_GAP_S), 0 restores
    follow_vlm's rule, a negative or non-finite value is refused before
    take-off, and params.follow.start_gate records it (follow_perception)."""
    ap = vla_fly.build_arg_parser()
    a = vla_fly.parse_cli(ap, ["--mission", "follow"])
    assert a.acquire_gap_s == vla_fly.ACQUIRE_GAP_S == vla_fly.LAPSE_S == 3.0
    assert vla_fly.make_acquirer(a, FV).gap_s == 3.0
    gate = vla_fly.follow_perception(a, True)["start_gate"]
    assert gate == {"kind": "identity_reacquirer", "need": 5, "max_range_m": 30.0,
                    "gap_s": 3.0}, gate
    assert vla_fly.follow_params(a, vla_fly.follow_perception(a, True))["start_gate"] == gate
    z = vla_fly.parse_cli(ap, ["--mission", "follow", "--acquire-gap-s", "0"])
    assert vla_fly.make_acquirer(z, FV).gap_s == 0.0
    for bad in ("-1", "nan", "inf"):
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                vla_fly.parse_cli(ap, ["--mission", "follow", "--acquire-gap-s", bad])
        except SystemExit as exc:
            assert exc.code == 2, bad
            continue
        raise AssertionError(f"accepted --acquire-gap-s {bad}")


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
        except Exception as e:                       # noqa: BLE001
            failed += 1
            import traceback
            traceback.print_exc()
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
