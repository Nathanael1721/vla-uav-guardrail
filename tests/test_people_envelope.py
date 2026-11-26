"""distance_envelope over people binds EVERY known person (WP1-07, 2026-11-18).

Run either way:
    pytest tests/test_people_envelope.py -v
    python tests/test_people_envelope.py

WHAT IS PINNED

The grant's distance_envelope is a "min distance to typed object class
(people, buildings, roads)". Until 2026-11-18 the people class bound ONE
position, the tracked subject while perception called it a person; a person
beside a tracked car was outside the rule (tests/test_world_map.py pinned
that limit). Now:

  (a) guardrail/people.py: the people field (distance to the nearest
      person's reach disc), the memory that turns per-frame fixes into
      tracks and drops a track unseen for more than PEOPLE_STALE_S (3.0 s),
      with association, ages and counters.
  (b) The Shield: a people envelope is the clearance runtime over that field
      - every person set_people names, plus the subject while it is a person
      - and is refused without a people source (never inert). A bystander
      beside a tracked car binds; a car subject does not join; the reach of
      an old fix widens the disc; the repair moves away; non-finite people
      are dropped and counted; a policy without the rule decides exactly as
      before whatever set_people is told.
  (c) The compiler renders it as every person, and the cost map draws a disc
      round every given person (and agrees with the Shield on it).
  (d) The perception rail, with SYNTHETIC detections (no GPU, no simulator):
      the people phrase's boxes out of OWL-ViT-shaped logits (non-maximum
      suppression, the subject's own column), each box placed with the
      geometry the follow places its subject with (subject_estimate, from
      the capture's pose), frames into the memory, the memory into the
      Shield - a person seen binds, and is dropped once unseen for longer
      than the staleness.
  (e) The sim rails: the simulator's pedestrian list (a level crowd's polls,
      the pre-mission poll never observed; client figures) into the memory.
  (f) The SITL rails and the headless harness: the declared subject alone,
      when it is a person; otherwise refused. Each rail names its source.

Synthetic inputs only; no rig step runs.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "demo" / "pas_ardupilot", ROOT / "experiments"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np                                                      # noqa: E402

from guardrail import people as P                                      # noqa: E402
from guardrail.models import Action4D, State, policy_from_raw           # noqa: E402
from guardrail.shield import Shield                                     # noqa: E402
from wgs84_fixture import wgs84_doc                     # noqa: E402

H = {"scope": "mission", "constraint_type": "hard", "priority": "P0",
     "violation_action": "project_fix"}
SRC = "test: synthetic people"


def _refused(fn, *needles, exc=ValueError):
    try:
        fn()
    except exc as e:
        msg = str(e)
        for n in needles:
            assert n in msg, (n, msg)
        return msg
    raise AssertionError(f"accepted; expected a refusal mentioning {needles}")


def _people_policy(d=5.0, extra=()):
    return policy_from_raw(wgs84_doc({"policy_id": "people-test", "version": "0.1.0", "constraints": [
        {"id": "de-people", "type": "distance_envelope", **H, "object_class": "people",
         "min_distance_m": d},
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}, *extra]}))


def _shield(**kw):
    return Shield(_people_policy(**kw), people_source=SRC, escalation=False)


ST = State(x=0.0, y=0.0, up=10.0)
HOVER = Action4D(vx=0.0, vy=0.0)


# ------------------------------------------------------------------ (a) people.py

def test_the_people_field_is_the_distance_to_the_nearest_reach_disc():
    f = P.PeopleField([(3.0, 4.0, 0.0), (10.0, 0.0, 2.0)])
    assert len(f) == 2
    assert abs(f.distance(0.0, 0.0) - 5.0) < 1e-12
    assert abs(f.distance(10.0, 0.0) - (-2.0)) < 1e-12        # inside the reach disc
    assert abs(f.distance(13.0, 0.0) - 1.0) < 1e-12
    assert P.PeopleField().distance(0.0, 0.0) == math.inf     # nobody
    # The reach is how far the person may have walked since the fix.
    assert P.Person(0.0, 0.0, age_s=2.0).reach_m() == 2.0 * P.PERSON_SPEED_MPS
    assert P.Person(0.0, 0.0, age_s=-1.0).reach_m() == 0.0
    assert P.as_person((1, 2)) == P.Person(1.0, 2.0, 0.0)
    assert P.as_person((1, 2, 0.5)) == P.Person(1.0, 2.0, 0.5)


def test_the_memory_tracks_people_and_drops_one_unseen_for_longer_than_the_staleness():
    assert P.PEOPLE_STALE_S == 3.0 and P.PERSON_SPEED_MPS == 1.5 and P.PEOPLE_ASSOC_M == 2.0
    m = P.PeopleMemory()
    assert m.observe(0.0, [(0.0, 0.0), (20.0, 0.0)], key=1) == 2
    # The same frame read again on the next tick is not a new observation.
    assert m.observe(0.0, [(0.0, 0.0), (20.0, 0.0)], key=1) == 0
    # A walk of 1 m in 0.5 s is the same person; 30 m away is a new one.
    assert m.observe(0.5, [(1.0, 0.0), (50.0, 0.0)], key=2) == 1
    now = m.current(1.0)
    assert sorted((p.x, p.y, round(p.age_s, 6)) for p in now) == \
        [(1.0, 0.0, 0.5), (20.0, 0.0, 1.0), (50.0, 0.0, 0.5)], now
    # Unseen for exactly the staleness: kept; a moment longer: dropped.
    assert len(m.current(3.0)) == 3                # the 20 m one, unseen 3.0 s
    left = m.current(3.0 + 1e-6)
    assert sorted(p.x for p in left) == [1.0, 50.0], left
    assert m.n_dropped_stale == 1
    assert len(m.current(10.0)) == 0 and m.n_dropped_stale == 3
    # One fix per track per frame: two people side by side stay two.
    m2 = P.PeopleMemory()
    m2.observe(0.0, [(0.0, 0.0), (1.0, 0.0)], key="a")
    m2.observe(0.2, [(0.1, 0.0), (1.1, 0.0)], key="b")
    assert len(m2.current(0.2)) == 2 and m2.n_tracks == 2
    # A non-finite fix is dropped and counted, never stored.
    m2.observe(0.3, [(float("nan"), 0.0), (0.0, float("inf"))], key="c")
    assert m2.n_nonfinite == 2 and len(m2.current(0.3)) == 2
    rec = m2.record()
    assert rec["stale_s"] == 3.0 and rec["frames"] == 3 and rec["fixes"] == 4, rec
    for bad in (0.0, -1.0, float("nan"), "3"):
        _refused(lambda: P.PeopleMemory(stale_s=bad), "stale_s")


# ------------------------------------------------------------------ (b) the Shield

def test_a_people_envelope_without_a_people_source_is_refused_never_inert():
    _refused(lambda: Shield(_people_policy()), "de-people", "people_source")
    sh = Shield(_people_policy(), require_maps=False)          # an offline reader
    assert sh.blind_clearance_rules() == ["de-people: no people_source - the rule is "
                                          "inert here"], sh.blind_clearance_rules()
    ok = _shield()
    assert ok.people_source == SRC and ok.blind_clearance_rules() == []


def test_a_bystander_beside_a_tracked_car_binds_the_recorded_limit_closed():
    """Before (tests/test_world_map.py, 17 Nov): "a person standing beside it
    is not passed to the Shield ... so nothing binds"."""
    sh = _shield(d=10.0)
    sh.set_subject(5.0, 0.0, "car")
    assert sh.state_is_unsafe(ST) == []                  # a car is not a person
    sh.set_people([(5.0, 2.0)])                          # the person beside it
    vs = sh.state_is_unsafe(ST)
    assert [v.rule_id for v in vs] == ["de-people"] and "person dist" in vs[0].detail, vs
    row = {r["id"]: r for r in sh.rule_status(ST)}["de-people"]
    assert row["bound"] and row["breach"] and abs(row["value"] - math.hypot(5, 2)) < 1e-9, row
    # Nobody reported (and a car subject): the rule binds nobody - unbound in
    # the status, not "bound, not breached".
    sh.set_people(None)
    row = {r["id"]: r for r in sh.rule_status(ST)}["de-people"]
    assert not row["bound"] and sh.state_is_unsafe(ST) == [], row


def test_the_subject_joins_the_people_while_it_is_a_person():
    sh = _shield(d=10.0)
    for cls, binds in (("pedestrian", True), ("person", True), ("Human", True),
                       ("car", False), (None, False)):
        sh.set_subject(5.0, 0.0, cls)
        assert bool(sh.state_is_unsafe(ST)) == binds, cls
    # In a look-ahead view the moving subject is where its velocity puts it.
    sh.set_subject(20.0, 0.0, "pedestrian", velocity=(-2.0, 0.0))
    assert sh.state_is_unsafe(ST) == []
    with sh.looking_ahead(6.0):
        assert {v.rule_id for v in sh.state_is_unsafe(ST)} == {"de-people"}


def test_an_old_fix_widens_the_disc_by_how_far_the_person_may_have_walked():
    sh = _shield(d=5.0)
    sh.set_people([(7.0, 0.0, 0.0)])
    assert sh.state_is_unsafe(ST) == []                  # 7 m > 5 m
    sh.set_people([(7.0, 0.0, 2.0)])                     # 2 s old: 3 m of reach
    assert {v.rule_id for v in sh.state_is_unsafe(ST)} == {"de-people"}


def test_the_nearest_of_several_people_decides_and_the_repair_moves_away():
    sh = _shield(d=5.0)
    sh.set_people([(30.0, 0.0), (3.0, 0.0), (0.0, -40.0)])
    r = sh.filter(ST, Action4D(vx=2.0, vy=0.0))          # towards the person 3 m ahead
    assert [v.rule_id for v in r.violations] == ["de-people"], r.violations
    assert r.emitted.vx < 0.0, r.emitted                 # backs away from them
    assert any(x.operator == "ClearanceFix" for x in r.repairs), r.repairs
    assert sh.filter(ST, r.emitted).violations == []     # what flew is legal
    # Leaving already is legal: nothing is changed.
    r2 = sh.filter(ST, Action4D(vx=-2.0, vy=0.0))
    assert r2.violations == [] and r2.emitted.vx == -2.0


def test_non_finite_people_are_dropped_and_counted():
    sh = _shield()
    sh.set_people([(float("nan"), 0.0), (1.0, float("inf")), (2.0, 0.0, float("nan")),
                   (8.0, 0.0)])
    assert sh.n_people_nonfinite == 3 and sh.people == (P.Person(8.0, 0.0, 0.0),)


def test_a_policy_without_a_people_envelope_ignores_set_people_decision_for_decision():
    """Behaviour identity for every policy flown before: nothing reads the
    people unless a distance_envelope over people is in the policy."""
    pol = policy_from_raw(wgs84_doc({"policy_id": "no-people", "constraints": [
        {"id": "nfz", "type": "polygon_fence", **H, "altitude_floor_m": 0,
         "altitude_ceiling_m": 100,
         "vertices": [{"x": 10, "y": -5}, {"x": 20, "y": -5}, {"x": 20, "y": 5},
                      {"x": 10, "y": 5}]},
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))
    a, b = Shield(pol, escalation=False), Shield(pol, escalation=False)
    b.set_people([(x, y) for x in range(-10, 30, 3) for y in range(-10, 10, 3)])
    n = 0
    for x in range(-4, 26, 3):
        for vx in (-4.0, 0.0, 4.0):
            for vy in (-4.0, 0.0, 4.0):
                st, act = State(x=float(x), y=0.0, up=10.0), Action4D(vx=vx, vy=vy)
                ra, rb = a.filter(st, act), b.filter(st, act)
                assert ra.emitted == rb.emitted and \
                    [(v.rule_id, v.detail) for v in ra.violations] == \
                    [(v.rule_id, v.detail) for v in rb.violations], (x, vx, vy)
                n += 1
    assert n == 90


# ------------------------------------------------------------------ (c) the compiler

def test_the_compiler_renders_every_person_and_the_cost_map_draws_each():
    from guardrail.compiler import (ConstraintCompiler, Mission, cost_map_agreement,
                                    rasterise_cost_map)
    pol = _people_policy(d=5.0)
    m = Mission(task_text="follow the car", start_x=0, start_y=0, target_x=40,
                target_y=0, cruise_alt_m=10, speed_pref_mps=3)
    text = ConstraintCompiler(pol).compile_csp(m).natural_language_prompt
    assert "Keep at least 5 m from any person." in text, text
    # Nobody given: not drawn, said why; the header holds no people key.
    cm0 = rasterise_cost_map(pol, bounds=(-20, -20, 20, 20))
    assert "de-people" in {d["id"] for d in cm0.header["not_drawn"]}
    assert "people" not in cm0.header
    cm = rasterise_cost_map(pol, bounds=(-20, -20, 20, 20), resolution_m=1.0,
                            people=[(10.0, 0.0), (-10.0, 5.0, 2.0)],
                            subject=(0.0, -10.0, "pedestrian"))
    layers = {L["name"]: L for L in cm.header["cost_map"]["layers"]}
    assert "people_hard" in layers and layers["people_hard"]["source"] == "de-people"
    assert cm.header["people"] == [[10.0, 0.0, 0.0], [-10.0, 5.0, 2.0]]
    assert math.isinf(cm.cost[cm.cell_of(10.0, 0.0)])          # a given person
    assert math.isinf(cm.cost[cm.cell_of(0.0, -10.0)])         # the person subject
    assert math.isinf(cm.cost[cm.cell_of(-10.0, 12.5)])        # 7.5 m: 5 + 3 m reach
    assert cm.cost[cm.cell_of(0.0, 0.0)] == 0.0
    agree = cost_map_agreement(cm, pol, n_points=1500, seed=3)
    assert agree["n_forbidden"] > 100 and agree["map"]["missed"] == 0, agree


# ------------------------------------------------------------------ (d) perception

def _fv():
    import perception_node as pn
    return pn, pn._fv()


def _logit(p):
    return math.log(p / (1.0 - p))


def test_the_people_phrase_gives_every_person_box_once():
    """OWL-ViT gives one box per image patch, so one person is several
    overlapping boxes; the people phrase's column picks them and NMS keeps
    one per person."""
    _, fv = _fv()
    # Columns: subject ("a red car"), people phrase. Boxes: two overlapping
    # on person A, one on person B, one on the car, one low-scoring.
    boxes = np.array([[100, 100, 120, 160], [102, 101, 121, 162], [300, 100, 318, 150],
                      [400, 200, 480, 240], [10, 10, 20, 20]], dtype=np.float32)
    lg = np.array([[_logit(0.01), _logit(0.40)], [_logit(0.01), _logit(0.35)],
                   [_logit(0.02), _logit(0.20)], [_logit(0.30), _logit(0.03)],
                   [_logit(0.01), _logit(0.05)]])
    out, note = fv.people_boxes(lg, boxes, 1, 0.10, 768, 432)
    assert note is None and len(out) == 2, out
    a, b = out
    assert (round(a[0], 1), round(a[1], 1), round(a[2], 1), round(a[3], 1)) == \
        (110.0, 130.0, 20.0, 60.0) and abs(a[4] - 0.40) < 1e-9 and a[5:] == (768, 432)
    assert round(b[0], 1) == 309.0 and abs(b[4] - 0.20) < 1e-9
    # Rows that cannot be paired with the boxes give nothing, and say so.
    none, why = fv.people_boxes(lg[:4], boxes, 1, 0.10, 768, 432)
    assert none == [] and "not paired" in why
    # The subject's own scores, not post-processing's best over both phrases.
    post = [max(0.01, 0.40), 0.35, 0.20, 0.30, 0.05]
    labels = [1, 1, 1, 0, 1]
    sc = fv.subject_column_scores(lg, post, labels)
    assert [round(v, 6) for v in sc] == [0.01, 0.01, 0.02, 0.30, 0.01], sc
    assert fv.subject_column_scores(lg[:4], post, labels) == [0.0, 0.0, 0.0, 0.30, 0.0]


def test_a_person_box_is_placed_as_the_follow_places_its_subject():
    pn, fv = _fv()
    W, Hh, hfov = 768, 432, 90.0
    pose = {"x": 10.0, "y": 20.0, "up": 12.0, "yaw": math.radians(90.0), "pitch": 0.0,
            "src": "ring"}
    # Straight ahead (image centre column), 20 m slant range from depth: a
    # box low in the frame, as a person 12 m below is.
    box = (W / 2.0, 330.0, 14.0, 40.0, 0.4, W, Hh, 20.0)
    pts, rec = pn.people_estimates({"pose": pose, "boxes": [box]}, ST, 0.0, 0.0, hfov,
                                   0.5, fv)
    horiz = math.sqrt(20.0 ** 2 - (12.0 - pn.SUBJECT_UP_M) ** 2)
    assert rec["placed"] == 1 and rec["pose"] == "capture:ring" and rec["src"] == {"depth": 1}
    assert abs(pts[0][0] - 10.0) < 1e-6 and abs(pts[0][1] - (20.0 + horiz)) < 1e-6, pts
    # Exactly subject_estimate's answer for the same box and capture.
    ref = pn.subject_estimate(box[:7], None, State(x=10.0, y=20.0, up=12.0, yaw_deg=90.0),
                              math.radians(90.0), hfov, 0.5, fv, range_m=20.0)
    assert abs(ref[0] - pts[0][0]) < 1e-9 and abs(ref[1] - pts[0][1]) < 1e-9
    # No depth range: the box width at a person's width; no capture pose:
    # this tick's pose, said so.
    nodepth = (W / 2.0, 330.0, 14.0, 40.0, 0.4, W, Hh, None)
    pts2, rec2 = pn.people_estimates({"pose": None, "boxes": [nodepth]}, ST, 0.0, 0.0,
                                     hfov, 0.5, fv)
    assert rec2["pose"] == "tick" and rec2["src"] == {"width": 1} and pts2[0][0] > 0
    assert pn.people_estimates(None, ST, 0.0, 0.0, hfov, 0.5, fv) == \
        ([], {"boxes": 0, "placed": 0, "pose": None, "src": {}})


def test_synthetic_detections_reach_the_shield_and_go_stale():
    """End to end on the perception rail's path, synthetic frames: each
    frame's person boxes placed, into the memory at the capture time, the
    memory into the Shield. A person the detector sees 12 m ahead binds a
    15 m envelope; once it has not been seen for longer than the staleness
    it is dropped and the rule binds nobody. The box sits where the camera
    (20 deg down) sees a person 12 m ahead from 10 m up, so the depth rules
    the follow applies (depth_implausible) accept its range."""
    pn, fv = _fv()
    W, Hh, hfov = 768, 432, 90.0
    sh = _shield(d=15.0)
    mem = P.PeopleMemory()
    pose = {"x": 0.0, "y": 0.0, "up": 10.0, "yaw": 0.0, "pitch": 0.0}
    dh = 10.0 - pn.SUBJECT_UP_M
    slant = math.hypot(12.0, dh)
    f = (W / 2.0) / math.tan(math.radians(hfov) / 2.0)
    row = Hh / 2.0 + f * math.tan(math.atan2(dh, 12.0) - math.radians(20.0))
    frame = {"pose": pose, "boxes": [(W / 2.0, row, 12.0, 30.0, 0.3, W, Hh, slant)]}
    t_last = None
    for k in range(5):                                 # seen at 3 Hz
        t = k / 3.0
        pts, rec = pn.people_estimates(dict(frame, seq=k), ST, 0.0, 0.0, hfov, 0.5, fv)
        assert rec["src"] == {"depth": 1}, rec           # placed from its depth range
        mem.observe(t, pts, key=k)
        sh.set_people(mem.current(t + 0.05))
        assert {v.rule_id for v in sh.state_is_unsafe(ST)} == {"de-people"}, k
        t_last = t
    assert mem.n_tracks == 1 and abs(sh.people[0].x - 12.0) < 1e-6, sh.people
    # Out of view: frames with no person. Held while within the staleness...
    for t in (t_last + 1.0, t_last + P.PEOPLE_STALE_S):
        mem.observe(t, [], key=f"empty{t}")
        sh.set_people(mem.current(t))
        assert len(sh.people) == 1 and sh.state_is_unsafe(ST), t
    # ...its disc widened by its age; dropped once older than it.
    mem.observe(t_last + P.PEOPLE_STALE_S + 0.1, [], key="late")
    sh.set_people(mem.current(t_last + P.PEOPLE_STALE_S + 0.1))
    assert sh.people == () and sh.state_is_unsafe(ST) == []


def test_the_rail_asks_for_people_only_for_a_people_policy():
    pn, fv = _fv()
    args = SimpleNamespace(people_query=None, people_thresh=None, people_stale_s=3.0)
    plain = policy_from_raw(wgs84_doc({"policy_id": "p", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))
    assert pn.people_source_for(plain, args, detector=True) == (None, None)
    src, phrase = pn.people_source_for(_people_policy(), args, detector=True)
    assert phrase == fv.PEOPLE_QUERY == "a person" and src.startswith("perception: ")
    _refused(lambda: pn.people_source_for(_people_policy(), args, detector=False),
             "de-people", "no detector", exc=SystemExit)
    _refused(lambda: pn.people_source_for(
        _people_policy(), SimpleNamespace(people_query="a red car", people_thresh=None,
                                          people_stale_s=3.0), detector=True),
        "does not name a person", exc=SystemExit)
    _refused(lambda: pn.people_source_for(
        _people_policy(), SimpleNamespace(people_query=None, people_thresh=None,
                                          people_stale_s=0.0), detector=True),
        "people-stale-s", exc=SystemExit)
    # The Grounder's phrase list: unchanged without a people phrase, the
    # people phrase last with one (after the negatives).
    g = fv.Grounder(None, "a red car", negatives=["a traffic light"])
    assert g._phrases() == [["a red car", "a traffic light"]] and g.people_query is None
    assert "people" not in g.latest()
    g2 = fv.Grounder(None, "a red car", negatives=["a traffic light"],
                     people_query="a person")
    assert g2._phrases() == [["a red car", "a traffic light", "a person"]]
    assert g2.latest()["people"] is None
    # The perception node hands the source to its Shield and the phrase to its
    # detector (text pins: the node's run() needs the rig).
    src_txt = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(
        encoding="utf-8")
    assert '{"people_source": people_source}))' in src_txt
    assert "people_query=people_query)" in src_txt
    assert "shield.set_people(people_now)" in src_txt


# ------------------------------------------------------------------ (e) the sim rails

class LevelActors:
    """Stands in for demo/level_actors.LevelActors (matched by class name)."""

    def __init__(self, figures):
        self.figures = figures
        self.n_calls = 1          # the pre-mission poll resolve() made
        self.walk = 0.0

    def refresh(self):
        self.n_calls += 1
        for f in self.figures:
            f.x += self.walk

    def update(self, t):
        self.refresh()


def test_the_simulators_pedestrian_list_feeds_the_people():
    _, fv = _fv()
    figs = [SimpleNamespace(x=1.0, y=0.0, seen=True, stale=0),
            SimpleNamespace(x=50.0, y=0.0, seen=True, stale=0),
            SimpleNamespace(x=9.0, y=9.0, seen=True, stale=2),        # last poll NaN
            SimpleNamespace(x=0.0, y=0.0, seen=False, stale=0)]       # never resolved
    crowd = LevelActors(figs)
    feed = fv.SimPeopleFeed(crowd, P.PeopleMemory())
    # The first step polls afresh: the pre-mission poll is never observed.
    crowd.walk = 0.5
    now = feed.step(0.1)
    assert crowd.n_calls == 2 and sorted(p.x for p in now) == [1.5, 50.5], now
    # No new poll on the next tick: nothing re-observed, the fix ages.
    now = feed.step(0.3)
    assert all(abs(p.age_s - 0.2) < 1e-9 for p in now), now
    crowd.update(0.6)
    now = feed.step(0.6)
    assert sorted(p.x for p in now) == [2.0, 51.0] and all(p.age_s == 0.0 for p in now)
    assert feed.memory.n_tracks == 2
    # Client figures: where they were commanded; a failed spawn is no fix.
    client = SimpleNamespace(figures=[SimpleNamespace(x=3.0, y=4.0, _failed=False),
                                      SimpleNamespace(x=7.0, y=7.0, _failed=True)])
    cf = fv.SimPeopleFeed(client, P.PeopleMemory())
    assert [(p.x, p.y) for p in cf.step(1.0)] == [(3.0, 4.0)]
    # No list at all: the memory only ages.
    assert fv.SimPeopleFeed(None, P.PeopleMemory()).step(1.0) == []
    # follow_vlm polls a level crowd on a car follow only for the people feed
    # (update_level_truth; before: only while the subject was a pedestrian).
    crowd2 = LevelActors([SimpleNamespace(x=0.0, y=0.0, seen=True, stale=0)])
    assert fv.update_level_truth(crowd2, "car", 1.0) is False and crowd2.n_calls == 1
    assert fv.update_level_truth(crowd2, "car", 1.0, feed_people=True) is True
    assert crowd2.n_calls == 2


def test_the_sim_rails_name_their_source_or_refuse():
    _, fv = _fv()
    plain = policy_from_raw(wgs84_doc({"policy_id": "p", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))
    a = SimpleNamespace(level_peds=40, level_ped_prefix="Ped_", level_truth_period=0.5,
                        pedestrians=0)
    assert fv.sim_people_source(plain, a) is None
    assert fv.sim_people_source(_people_policy(), a).startswith(
        "simulator: the level's 40 pedestrians")
    c = SimpleNamespace(level_peds=0, level_ped_prefix="Ped_", level_truth_period=0.1,
                        pedestrians=6)
    assert "client's 6 spawned figures" in fv.sim_people_source(_people_policy(), c)
    none = SimpleNamespace(level_peds=0, level_ped_prefix="Ped_", level_truth_period=0.1,
                           pedestrians=0)
    _refused(lambda: fv.sim_people_source(_people_policy(), none), "--level-peds",
             exc=SystemExit)
    import vla_fly
    assert vla_fly.people_source_vla(plain, SimpleNamespace(level_peds=0)) is None
    _refused(lambda: vla_fly.people_source_vla(_people_policy(),
                                               SimpleNamespace(level_peds=0)),
             "--level-peds", exc=SystemExit)
    vf = (ROOT / "demo" / "vla_fly.py").read_text(encoding="utf-8")
    assert '{"people_source": people_source}))' in vf and "shield.set_people(people_now)" in vf
    fvt = (ROOT / "demo" / "follow_vlm.py").read_text(encoding="utf-8")
    assert "people_source=people_source)" in fvt and "shield.set_people(people_now)" in fvt


# ------------------------------------------------------------------ (f) SITL, harness

def test_the_sitl_rails_and_the_harness_judge_the_declared_person_only():
    plain = policy_from_raw(wgs84_doc({"policy_id": "p", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))
    assert P.declared_subject_source(plain, None, None, rail="r") == ({}, [])
    kw, gaps = P.declared_subject_source(_people_policy(), (30.0, 0.0), "pedestrian",
                                         rail="the ArduPilot SITL rail")
    assert gaps == [] and kw["people_source"].startswith("the declared subject only "
                                                         "(pedestrian at (30.0, 0.0)")
    sh = Shield(_people_policy(d=10.0), people_source=kw["people_source"])
    sh.set_subject(30.0, 0.0, "pedestrian")
    assert sh.state_is_unsafe(State(x=25.0, y=0.0, up=10.0))
    for subj, cls in (((30.0, 0.0), "car"), (None, None)):
        kw, gaps = P.declared_subject_source(_people_policy(), subj, cls, rail="r")
        assert kw == {} and len(gaps) == 1 and "de-people" in gaps[0], gaps
    node = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "declared_subject_source(" in node and "self.world_kw.update(ppl_kw)" in node
    demo = (ROOT / "sitl" / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert "declared_subject_source(" in demo and "path_repair=FOLLOW_RAIL_PATH_REPAIR, **ppl_kw)" \
        in demo
    # A subject given as the CLI's "X,Y" text places as numbers.
    kw, _ = P.declared_subject_source(_people_policy(), "-5.5,3".split(","), "pedestrian",
                                      rail="r")
    assert "(-5.5, 3.0)" in kw["people_source"], kw
    sw = (ROOT / "experiments" / "sweep_scenarios.py").read_text(encoding="utf-8")
    assert "declared_subject_source(" in sw and "rec_kw.update(ppl_kw)" in sw


HARNESS_PEOPLE_POLICY = """policy_id: harness-people-envelope
version: 0.1.0
origin:
  lat: -35.363261
  lon: 149.16523
constraints:
  - id: de-people
    type: distance_envelope
    constraint_type: hard
    scope: mission
    priority: P0
    violation_action: project_fix
    object_class: people
    min_distance_m: 6.0
  - id: alt-band
    type: altitude_envelope
    constraint_type: hard
    scope: mission
    priority: P0
    violation_action: project_fix
    alt_min_m: 10
    alt_max_m: 20
  - id: kin-caps
    type: kinematic_envelope
    constraint_type: hard
    scope: mission
    priority: P1
    violation_action: project_fix
    speed_max_mps: 4.0
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 45.0
"""


def _harness_spec(policy_path, cls="pedestrian", shield=True):
    from guardrail.scenario_spec import ScenarioSpec
    return ScenarioSpec.model_validate(dict(
        scenario_id="people-envelope-pass", template="subject_standoff",
        description="a goto past a declared pedestrian 2 m off the route",
        policy={"path": str(policy_path)}, shield=shield,
        mission={"start_pose": {"x": 0, "y": 0, "up": 15},
                 "target": {"x": 40, "y": 0, "up": 15},
                 "pilot": {"type": "goto", "speed": 4.0},
                 "subject": {"x": 20.0, "y": 2.0, "class": cls}},
        expected_kpis={"p0_violation_escape_rate": {"max": 0.0}}))


def test_a_headless_episode_holds_the_people_envelope_off_a_declared_pedestrian():
    """Review 2026-11-18: the harness's people wiring was pinned by source
    text only, and no scenario carried a people policy. One episode, built
    here (no profile carries it, so the five digested profiles do not move):
    a goto past a declared pedestrian 2 m off the route under a 6 m
    distance_envelope over people. The Shield names de-people, holds the
    aircraft >= 6 m from the person (6.04 m measured, seed 1001, ideal
    vehicle), never lets the position become unsafe, and the mission still
    reaches its goal; the Shield-off arm flies within 2 m. A car subject is
    refused before take-off (the harness has no person to judge the rule
    against)."""
    import tempfile
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    with tempfile.TemporaryDirectory() as td:
        pp = Path(td) / "harness_people.yaml"
        pp.write_text(HARNESS_PEOPLE_POLICY, encoding="utf-8")
        run = S.run_episode(_harness_spec(pp), lib.defaults, 1001)
        named = {v["rule_id"] for r in run.rows for v in r["violations"]}
        dmin = min(math.hypot(r["x"] - 20.0, r["y"] - 2.0) for r in run.rows)
        assert named == {"de-people"}, named
        assert dmin >= 6.0 - 1e-6, dmin
        assert not any(r["unsafe"] for r in run.rows)
        assert run.extra["reached_goal"] is True and run.extra["breaches"] == 0, run.extra
        off = S.run_episode(_harness_spec(pp, shield=False), lib.defaults, 1001)
        assert min(math.hypot(r["x"] - 20.0, r["y"] - 2.0) for r in off.rows) < 2.5
        assert any("de-people" in r.get("unsafe_rules", []) or r["unsafe"] for r in off.rows)
        try:
            S.run_episode(_harness_spec(pp, cls="car"), lib.defaults, 1001)
        except S.UnsupportedScenario as e:
            assert "de-people" in str(e) and "'car'" in str(e), e
        else:
            raise AssertionError("a people envelope flew over a car subject")


def test_the_rig_policy_loads_for_flight_and_needs_its_people_source():
    """experiments/policies/follow_car_citylife_people.yaml: the CityLife
    follow's rules plus a 4 m people envelope, the policy the rig's
    CityLife follow with pedestrians flies (unlocked, beside its experiment,
    as msl_dem_sitl.yaml)."""
    from guardrail.models import load_policy
    pol = load_policy(ROOT / "experiments" / "policies" / "follow_car_citylife_people.yaml")
    assert pol.flight_problems() == [] and P.needs_people(pol) == ["de-people"]
    de = next(c for c in pol.constraints if c.id == "de-people")
    assert (de.object_class, de.min_distance_m, de.priority) == ("people", 4.0, "P1")
    _refused(lambda: Shield(pol), "de-people", "people_source")
    Shield(pol, people_source=SRC, obstacle_map={"occ": np.zeros((4, 4), dtype=np.uint8),
                                                 "res": 2.0, "ox": 0.0, "oy": 0.0})
    from guardrail.bundle import policy_candidates
    assert any(name == "experiments/policies/follow_car_citylife_people.yaml"
               for name, _ in policy_candidates()), "a run flown under it resolves"


def test_the_rails_hand_the_shield_the_people_within_reach_of_the_decision():
    """people.within / PEOPLE_RANGE_M: a person whose reach disc is more than
    100 m away is not handed to the Shield (the 5 s lookahead at 5 m/s
    reaches 25 m); the memory keeps it."""
    assert P.PEOPLE_RANGE_M == 100.0
    ppl = [P.Person(50.0, 0.0), P.Person(150.0, 0.0), P.Person(102.0, 0.0, 2.0)]
    assert [p.x for p in P.within(ppl, 0.0, 0.0)] == [50.0, 102.0]   # 102 - 3 m reach
    assert P.within([(0.0, 101.0)], 0.0, 0.0) == []
    # Within the range, the culled and the full set decide alike.
    sh_all, sh_cut = _shield(d=5.0), _shield(d=5.0)
    crowd = [(3.0, 0.0), (-4.0, 2.0)] + [(float(200 + 7 * k), float(-300 + 11 * k))
                                         for k in range(40)]
    sh_all.set_people(crowd)
    sh_cut.set_people(P.within(crowd, 0.0, 0.0))
    assert len(sh_cut.people) == 2
    for vx in (-4.0, 0.0, 4.0):
        for vy in (-4.0, 0.0, 4.0):
            a, b = sh_all.filter(ST, Action4D(vx=vx, vy=vy)), \
                sh_cut.filter(ST, Action4D(vx=vx, vy=vy))
            assert a.emitted == b.emitted and \
                [v.rule_id for v in a.violations] == [v.rule_id for v in b.violations]
    for rail, at in (("demo/follow_vlm.py", "state.x, state.y, people_range)"),
                     ("demo/vla_fly.py", "people_range)"),
                     ("demo/pas_ardupilot/perception_node.py", "st.x, st.y, people_range)")):
        src = (ROOT / rail).read_text(encoding="utf-8")
        i = src.index("people_now = people_feed.within(")
        assert at in src[i:i + 200], rail
        assert "people_feed.cull_range_m(" in src and \
            "people_range = (None if people_source is None" in src, rail
        assert '"range_m": people_feed.range_record(people_range)' in src, rail


def test_the_cull_range_is_sized_by_the_envelope_and_the_forecast():
    """Review 2026-11-18: the 100 m cull was checked against nothing, so an
    envelope over ~75 m, or a faster cap or a longer lookahead, would have
    dropped people the rule must judge. people.cull_range_m sizes it as
    max(min_distance_m) + lookahead * speed + margin, floored at 100 m
    (every shipped people policy culls at the floor), and an unbounded
    speed gets no cull at all."""
    def pol(d, cap=5.0):
        cons = [{"id": "de-people", "type": "distance_envelope", **H,
                 "object_class": "people", "min_distance_m": d}]
        if cap is not None:
            cons.append({"id": "kin", "type": "kinematic_envelope", **H,
                         "speed_max_mps": cap, "climb_rate_max_mps": 2.0,
                         "yaw_rate_max_dps": 45.0})
        return policy_from_raw(wgs84_doc({"policy_id": "p", "constraints": cons}))
    from guardrail.models import load_policy
    rig = load_policy(ROOT / "experiments" / "policies" / "follow_car_citylife_people.yaml")
    assert P.cull_range_m(rig, 5.0) == P.PEOPLE_RANGE_M          # 4 + 25 + 10 < 100
    assert P.cull_range_m(rig, 5.0, 4.0) == P.PEOPLE_RANGE_M
    # A large envelope: 80 + 5 * 5 + 10 = 115 m.
    assert P.cull_range_m(pol(80.0), 5.0) == 115.0
    # A faster rail bound than the cap, or a longer lookahead, widens it.
    assert P.cull_range_m(pol(80.0), 5.0, 9.0) == 80.0 + 45.0 + P.PEOPLE_RANGE_MARGIN_M
    assert P.cull_range_m(pol(4.0, cap=20.0), 10.0) == 4.0 + 200.0 + 10.0
    # No speed bound anywhere: nothing bounds the forecast, no cull.
    assert P.cull_range_m(pol(4.0, cap=None), 5.0) == math.inf
    assert P.range_record(math.inf).startswith("no cull")
    assert P.range_record(115.0) == 115.0
    # No people rule: the floor.
    plain = policy_from_raw(wgs84_doc({"policy_id": "p", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))
    assert P.cull_range_m(plain, 5.0) == P.PEOPLE_RANGE_M
    _refused(lambda: P.cull_range_m(rig, float("nan")), "lookahead_s")
    # The equivalence the cull rests on, for a large envelope (98 m): a
    # person 101 m away, outside the old 100 m floor and inside the sized
    # 133 m range, decides the tick exactly as the full crowd does (the
    # clearance forecast's stopping horizon carries the aircraft ~5 m on) -
    # and the old floor would have dropped it.
    big = pol(98.0)
    rng = P.cull_range_m(big, 5.0)
    assert rng == 98.0 + 25.0 + P.PEOPLE_RANGE_MARGIN_M
    crowd = [(101.0, 0.0), (400.0, 0.0)]
    sh_all = Shield(big, lookahead_s=5.0, dt=0.1, people_source=SRC)
    sh_cut = Shield(big, lookahead_s=5.0, dt=0.1, people_source=SRC)
    sh_old = Shield(big, lookahead_s=5.0, dt=0.1, people_source=SRC)
    sh_all.set_people(crowd)
    sh_cut.set_people(P.within(crowd, 0.0, 0.0, rng))
    sh_old.set_people(P.within(crowd, 0.0, 0.0))
    assert len(sh_cut.people) == 1 and len(sh_old.people) == 0
    a = Action4D(vx=5.0)
    da, dc, do = sh_all.filter(ST, a), sh_cut.filter(ST, a), sh_old.filter(ST, a)
    assert [v.rule_id for v in da.violations] == ["de-people"], da.violations
    assert da.emitted == dc.emitted and \
        [v.rule_id for v in da.violations] == [v.rule_id for v in dc.violations]
    assert do.violations == [], "the fixed 100 m floor would have missed it"


def test_an_offline_reader_takes_its_people_from_the_log_or_refuses():
    """people.logged_people_source / row_people (review 2026-11-18): what
    tools/rescore_kpis.py and tools/rerender_policy_hud.py rebuild a people
    envelope's Shield from - every posed row's `people`, else a declared
    person subject (metrics `people_source` of the SITL rails, or
    `subject`), else a stated refusal."""
    pol = _people_policy()
    rows = [{"x": 0.0, "y": 0.0, "people": [[1.0, 2.0, 0.5]]},
            {"x": 1.0, "y": 0.0, "people": []}, {"t": 9.0}]
    kw, gaps = P.logged_people_source(pol, rows, {"people_feed": {"source": "sim: x"}})
    assert gaps == [] and kw["people_source"] == (
        "replayed from the flight log's per-row `people` (as flown: sim: x)"), kw
    assert P.row_people(rows[0]) == [P.Person(1.0, 2.0, 0.5)]
    assert P.row_people(rows[1]) == [] and P.row_people(rows[2]) is None
    kw, gaps = P.logged_people_source(pol, rows[:1] + [{"x": 2.0, "y": 0.0}], {})
    assert kw == {} and "1 of 2 ticks" in gaps[0], gaps
    sitl = {"people_source": "the declared subject only (pedestrian at (30.0, 0.0); x)"}
    kw, gaps = P.logged_people_source(pol, [{"x": 0.0, "y": 0.0}], sitl)
    assert gaps == [] and kw["people_source"].startswith("replayed: the declared subject")
    for m in ({}, {"subject": {"class": "car", "x": 1.0, "y": 1.0}}):
        kw, gaps = P.logged_people_source(pol, [{"x": 0.0, "y": 0.0}], m)
        assert kw == {} and "records no people" in gaps[0], gaps
    plain = policy_from_raw(wgs84_doc({"policy_id": "p", "constraints": [
        {"id": "kin", "type": "kinematic_envelope", **H, "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}]}))
    assert P.logged_people_source(plain, [{"x": 0.0, "y": 0.0}], {}) == ({}, [])


def test_rows_people_is_what_a_flight_log_keeps():
    assert P.rows_people([P.Person(1.234567, -2.0, 0.12345), (3.0, 4.0)]) == \
        [[1.23, -2.0, 0.123], [3.0, 4.0, 0.0]]


if __name__ == "__main__":
    import traceback
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as e:                                # noqa: BLE001
            failed += 1
            print(f"FAIL  {name}\n      {type(e).__name__}: {e}")
            traceback.print_exc()
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
