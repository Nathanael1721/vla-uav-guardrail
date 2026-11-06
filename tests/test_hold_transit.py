"""A moved hold's transit enters no rule the aircraft is outside of and goes no
deeper into one it is in - at EVERY instant of the transit, not only at the
poses the search evaluates (guardrail/holdpoint.py transit_breaks,
Shield.rule_clearances / rule_motion).

Run either way:
    pytest tests/test_hold_transit.py -v
    python tests/test_hold_transit.py

WHY THIS FILE EXISTS

Until 2026-10-20 the transit was judged at poses 1 m apart along its longer
axis, evenly spaced in time, and the claim "a zone in force that the aircraft
is outside of is never entered" held at those poses only. The re-review of
T1c (P1) reproduced it three ways, each pinned here:

  * a 1 m bar closing at 6 m/s on a legal aircraft at (0, 0, 15): the plan
    moved east to (2, 0) in a 0.5 s transit checked at 2 poses, while 159 of
    400 dense poses were in the bar's ring and 79 in its polygon (also 1 m at
    8 and 10 m/s, 0.5 m at 5 m/s);
  * a climb-dominated transit: an always-on wall 0.8 m thick, the aircraft
    below the 10 m floor, a zone above 9 m over the start - the plan's
    horizontal samples, 1.875 m apart, straddled the wall's 1.8 m ring (22 of
    400 dense poses in the polygon) while the reason said the line "enters no
    rule a stop here would not break";
  * a transit out of a moving zone whose horizontal leg ended before the
    first sample: 0.72 m deeper than a stop at the start, between samples.

Each test judges the plan with the Shield's own rule_status and rule_depths
(the reviewer's oracle) at 1000 poses of the transit, at their own times -
so it would catch a crossing the search's own bound missed - and a seeded
fuzz of thin, fast and turning zones and walking subjects does the same for
every moved hold and landing site it produces.

A second review of the fix (2026-10-20, evening) found two more ways the
bound was not the Shield's: a keep-in rule's depth was its largest single
excess where rule_depths ADDS a corridor's lateral and vertical excess (a
transit out from under a corridor drifted wider of it while the climb hid
it: 11 of 151 random transits the test passed went deeper, up to metres),
and the clearance field was assumed to change by at most sqrt(2) m per
metre where a building face and the off-map branch reach 2.7 and 2.2 (a 0.3 m
transit beside a border building dipped 5 cm under the limit unseen). Both
are pinned below, and a seeded fuzz of RANDOM transits - not only the
search's plans - in worlds with stand-off rings, clearance maps, corridors
and windows opening or closing on the way checks the test itself.
"""
import math
import os
import random
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail import load_policy  # noqa: E402
from guardrail import holdpoint as HP  # noqa: E402
from guardrail.models import Action4D, DynamicNFZ, Policy, State  # noqa: E402
from guardrail.shield import Shield  # noqa: E402

# The four ConstraintBase fields the grant requires (IR schema 3.0.0, 2026-11-02;
# scope since 4.0.0, 2026-11-06), at the values they defaulted to before
# (repair = project_fix; an absent scope meant mission).
HARD_P0 = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
           "violation_action": "project_fix"}

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
DENSE = 1000
SKIP = "SKIP"
KIN = {"id": "kin", "type": "kinematic_envelope", "scope": "mission", "constraint_type": "hard",
       "priority": "P1", "violation_action": "repair", "speed_max_mps": 4.0,
       "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
RING = {"id": "ring", "type": "subject_standoff", "scope": "mission", "constraint_type": "hard",
        "priority": "P0", "violation_action": "repair", "min_range_m": 5.0}


def _demo_with(*extra: dict) -> Policy:
    """The demo policy with more rules (a stand-off ring for a fuzz world
    with a subject: the demo policy has none, so a subject alone binds
    nothing)."""
    body = yaml.safe_load(DEMO.read_text(encoding="utf-8"))
    body["constraints"] = list(body["constraints"]) + [dict(e) for e in extra]
    return Policy.model_validate(body)


def _binding(row) -> bool:
    return (row.get("constraint_type", "hard") == "hard"
            and row.get("effective_action") != "monitor_only")


def _transit_problems(sh: Shield, st: State, end: State, travel: float, speed: float,
                      climb: float, escaping, n: int = DENSE) -> list[tuple]:
    """The reviewer's oracle, at n poses of the transit st -> end at their own
    times: ('ENTERED', rule, tau) for a hard rule in force that the pose
    breaches although the aircraft was not leaving it - not in `escaping`, or
    in force at the plan and not breached, or a stop at the start no longer
    in it then; ('DEEPER', rule, tau, excess) for one it is leaving where the
    pose is more than LEAVE_TOL_M deeper than a stop at the start then
    (Shield.rule_depths); [] for a clean transit."""
    esc = set(escaping)
    rows0 = {r["id"]: r for r in sh.rule_status(st) if _binding(r)}
    out = []
    for i in range(1, n + 1):
        tau = travel * i / n
        q = HP._pose_at(st, end, tau, speed, climb)
        with sh.looking_ahead(tau):
            hit = {r["id"] for r in sh.rule_status(q) if r["breach"] and _binding(r)}
            if not hit:
                continue
            start_hit = {r["id"] for r in sh.rule_status(st) if r["breach"] and _binding(r)}
            for rid in sorted(hit):
                r0 = rows0[rid]
                leaving = rid in esc and (r0["breach"] or not r0["in_force"])
                if not leaving or rid not in start_hit:
                    out.append(("ENTERED", rid, round(tau, 4)))
                    continue
                dq = sh.rule_depths(q, [rid])[rid]
                ds = sh.rule_depths(st, [rid])[rid]
                if dq > ds + HP.LEAVE_TOL_M + 1e-6:
                    out.append(("DEEPER", rid, round(tau, 4), round(dq - ds, 4)))
    return out


def _dense_problems(sh: Shield, st: State, p, n: int = DENSE) -> list[tuple]:
    """_transit_problems for a plan's transit, leaving what a stop at the
    start breaks (the search's own `escaping`); [] for an in-place plan."""
    if p.kind != "moved":
        return []
    esc = {v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))}
    return _transit_problems(sh, st, State(x=p.x, y=p.y, up=p.up), p.travel_s,
                             p.speed_mps, p.climb_mps, esc, n)


def _bar(x0: float, w: float, half: float, speed: float, margin: float = 0.5) -> Shield:
    """The re-review's setup: the demo policy, the aircraft at (0, 0, 15) -
    legal now - and a DynamicNFZ bar `w` m wide from x = x0, `half` m either
    side of y = 0, closing from the east at `speed` m/s."""
    bar = DynamicNFZ(id="bar", type="dynamic_nfz", **HARD_P0,
                     vertices=[{"x": x0, "y": -half}, {"x": x0 + w, "y": -half},
                               {"x": x0 + w, "y": half}, {"x": x0, "y": half}],
                     altitude_floor_m=0, altitude_ceiling_m=100, margin_m=margin,
                     motion={"vx_mps": -speed, "vy_mps": 0.0})
    sh = Shield(load_policy(DEMO), escalation=False)
    sh.spawn_nfz(bar, t=0.0)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(), t=0.0)
    return sh


def test_a_fast_thin_bar_is_never_crossed_between_two_poses():
    """Re-review P1 (a), the four variants b7fc6aa and 47e77a5 both flew
    through: (x0, width, speed) = (3, 1, 6), (3, 1, 8), (3, 0.5, 5), (2, 1,
    10). The plan is a transit that stays out of the bar at every one of
    1000 poses, or none (in place, marked illegal, naming the bar)."""
    st = State(x=0.0, y=0.0, up=15.0)
    for x0, w, speed in ((3.0, 1.0, 6.0), (3.0, 1.0, 8.0), (3.0, 0.5, 5.0), (2.0, 1.0, 10.0)):
        sh = _bar(x0, w, 20.0, speed)
        assert sh.hard_violations(sh.state_is_unsafe(st)), "the start must be illegal"
        p = HP.choose_hold_point(sh, st)
        if p.kind == "moved":
            assert p.legal, p
            bad = _dense_problems(sh, st, p)
            assert not bad, (x0, w, speed, p.x, p.y, p.travel_s, bad[:3], len(bad))
        else:
            assert not p.legal and "bar" in p.breaks, (x0, w, speed, p)
        # The transit the old test passed (east to (2, 0) in 0.5 s at 4 m/s)
        # is refused by name.
        assert "bar" in HP.transit_breaks(sh, st, State(x=2.0, y=0.0, up=15.0), 0.5, 4.0,
                                          2.0, ("bar",)), (x0, w, speed)


def _thin_wall() -> tuple[Shield, State]:
    """Re-review P1 (b): an always-on wall 0.8 m thick (x 2.4-3.2, margin
    0.5), the aircraft at (0, 0, 3) below the 10-20 m band, and a zone above
    9 m over the start (x -20..1.5), so straight up is not legal."""
    def fence(zid, x0, y0, x1, y1, floor=0, ceil=100, margin=0.5):
        return {"id": zid, "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
                "priority": "P0", "violation_action": "repair", "margin_m": margin,
                "vertices": [{"x": x0, "y": y0}, {"x": x1, "y": y0}, {"x": x1, "y": y1},
                             {"x": x0, "y": y1}],
                "altitude_floor_m": floor, "altitude_ceiling_m": ceil}
    pol = Policy.model_validate({"policy_id": "thin", "constraints": [
        {"id": "alt-band", "type": "altitude_envelope", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "alt_min_m": 10, "alt_max_m": 20},
        {"id": "kin", "type": "kinematic_envelope", "scope": "mission", "constraint_type": "hard",
         "priority": "P1", "violation_action": "repair", "speed_max_mps": 4.0,
         "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0},
        fence("high", -20, -20, 1.5, 20, floor=9, ceil=100),
        fence("wall", 2.4, -20, 3.2, 20)]})
    sh = Shield(pol, escalation=False)
    st = State(x=0.0, y=0.0, up=3.0)
    sh.filter(st, Action4D(), t=0.0)
    return sh, st


def test_a_climb_dominated_transit_is_judged_between_its_horizontal_poses():
    """Re-review P1 (b): the plan was (4.5, 0, 10.5) in 3.75 s, 8 samples
    1.875 m apart horizontally, straddling the wall's 1.8 m ring - 22 of 400
    dense poses inside the wall's polygon. That line is refused by name now,
    and whatever the search returns enters nothing it started outside of."""
    sh, st = _thin_wall()
    assert [v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))] == ["alt-band"]
    assert "wall" in HP.transit_breaks(sh, st, State(x=4.5, y=0.0, up=10.5), 3.75, 4.0, 2.0,
                                       ("alt-band",))
    p = HP.choose_hold_point(sh, st)
    if p.kind == "moved":
        assert p.legal, p
        bad = _dense_problems(sh, st, p)
        assert not bad, (p.x, p.y, p.up, p.travel_s, bad[:3], len(bad))
        assert not (p.x > 1.9 and abs(p.y) < 20.5), f"crossed the wall: {p}"
    else:
        assert not p.legal and p.breaks == ("alt-band",), p


def _leaving_a_moving_zone() -> tuple[Shield, State]:
    """Re-review P1, the depth case (its fuzz hold #97): the demo policy, the
    aircraft at (1.18, 0.97, 22) - above the 20 m ceiling and inside z0, a
    4 m-wide bar moving at 5 m/s diagonally - and two slow neighbours. The
    plan descended to (2.56, 1.54, 19.5) in 1.25 s with 3 samples; its 1.5 m
    horizontal leg ended at 0.37 s, before the first, and went 0.72 m deeper
    into z0 than a stop at the start (3.00 against 2.28 m, at 0.19 s)."""
    def nfz(zid, pts, margin, vx, vy):
        return DynamicNFZ(id=zid, type="dynamic_nfz", **HARD_P0,
                          vertices=[{"x": x, "y": y} for x, y in pts],
                          altitude_floor_m=0, altitude_ceiling_m=100, margin_m=margin,
                          motion={"vx_mps": vx, "vy_mps": vy})
    sh = Shield(load_policy(DEMO), escalation=False)
    sh.spawn_nfz(nfz("z0", [(17.872, -18.39), (20.779, -15.629), (-5.464, 12.004),
                            (-8.372, 9.243)], 1.0, -3.634, 3.4185), t=0.0)
    sh.spawn_nfz(nfz("z1", [(13.288, -21.737), (14.868, -18.107), (-17.016, -4.228),
                            (-18.596, -7.858)], 0.5, 0.3786, 0.2964), t=0.0)
    sh.spawn_nfz(nfz("z2", [(-8.087, 14.514), (-9.469, 14.91), (-15.247, -5.262),
                            (-13.866, -5.657)], 0.5, -0.4135, 1.097), t=0.0)
    st = State(x=1.1756, y=0.9677, up=22.0)
    sh.filter(st, Action4D(), t=0.0)
    return sh, st


def test_a_transit_out_of_a_zone_never_dips_deeper_between_two_poses():
    sh, st = _leaving_a_moving_zone()
    unsafe = sorted(v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st)))
    assert unsafe == ["alt-band", "z0"], unsafe
    old = State(x=2.56, y=1.54, up=19.5)
    assert "z0" in HP.transit_breaks(sh, st, old, 1.25, 4.0, 2.0, unsafe), \
        "the old plan's transit goes deeper into z0 between its samples"
    for fn in (HP.choose_hold_point, HP.choose_landing_site):
        p = fn(sh, st)
        bad = _dense_problems(sh, st, p)
        assert not bad, (fn.__name__, p.kind, p.x, p.y, p.up, bad[:3], len(bad))


def _poses_in(sh: Shield, st: State, end: State, travel: float, rid: str,
              n: int) -> int:
    """How many of n evenly timed poses of the transit st -> end (4 m/s,
    2 m/s climb) are inside rule `rid` at their own time."""
    return sum(any(r["id"] == rid and r["breach"]
                   for r in sh.rule_status(HP._pose_at(st, end, travel * i / n, 4.0, 2.0),
                                           ahead_s=travel * i / n))
               for i in range(1, n + 1))


def test_a_turning_zone_and_a_walking_person_are_judged_where_they_will_be():
    """The bound includes the rule's own motion. A 0.2 m bar 10 m long,
    turning at 60 deg/s about its centre (0, 7) - its tip sweeps at 5.2 m/s
    through a ring 0.6 m wide - and the aircraft 3 m from the centre, inside
    the disc it sweeps. Of 16 straight 8 m transits, every one the bar
    crosses at any of 1000 poses is refused, among them three whose poses
    1 m apart (the old test) never meet it; and the plan is clean. Then a
    pedestrian walking at 2 m/s onto a hovering aircraft: the plan leaves
    the 5 m ring clean, judged where the person will be."""
    st = State(x=3.0, y=7.0, up=15.0)
    spin = DynamicNFZ(id="spin", type="dynamic_nfz", **HARD_P0,
                      vertices=[{"x": -0.1, "y": 2.0}, {"x": 0.1, "y": 2.0},
                                {"x": 0.1, "y": 12.0}, {"x": -0.1, "y": 12.0}],
                      altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.2,
                      motion={"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 60.0})
    kin = {"id": "kin", "type": "kinematic_envelope", "scope": "mission", "constraint_type": "hard",
           "priority": "P1", "violation_action": "repair", "speed_max_mps": 4.0,
           "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
    sh = Shield(Policy.model_validate({"policy_id": "spin", "constraints": [kin]}),
                escalation=False)
    sh.spawn_nfz(spin, t=0.0)
    sh.filter(st, Action4D(), t=0.0)
    assert [v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))] == ["spin"]
    crossed = missed_by_metres = 0
    for k in range(16):
        a = 2.0 * math.pi * k / 16
        end = State(x=st.x + 8.0 * math.cos(a), y=st.y + 8.0 * math.sin(a), up=15.0)
        got = HP.transit_breaks(sh, st, end, 2.0, 4.0, 2.0, ("spin",))
        if _poses_in(sh, st, end, 2.0, "spin", DENSE):
            crossed += 1
            assert got == ["spin"], (k, got)
            missed_by_metres += _poses_in(sh, st, end, 2.0, "spin", 8) == 0
    assert crossed >= 4 and missed_by_metres >= 2, (crossed, missed_by_metres)
    p = HP.choose_hold_point(sh, st)
    assert p.kind == "moved" and p.legal and not _dense_problems(sh, st, p), p

    ped = Policy.model_validate({"policy_id": "ped", "constraints": [
        {"id": "ring", "type": "subject_standoff", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "min_range_m": 5.0}, kin]})
    st = State(x=0.0, y=0.0, up=15.0)
    sh = Shield(ped, escalation=False)
    sh.set_subject(7.0, 0.0, "pedestrian", velocity=(-2.0, 0.0))
    sh.filter(st, Action4D(), t=0.0)
    assert [v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))] == ["ring"]
    p = HP.choose_hold_point(sh, st)
    assert p.kind == "moved" and p.legal and not _dense_problems(sh, st, p), p


def _fuzz_shield(rng: random.Random, thin: bool) -> tuple[Shield, State]:
    """One random world on the demo policy: 1-3 DynamicNFZ bars (thin: 0.2-
    1.5 m wide at 0 or 2-8 m/s; else 1-6 m wide at 0-5 m/s), some turning,
    a walking or standing pedestrian 3 times in 10 with a 5 m stand-off ring
    to keep (until the second review the subject came without one, and the
    demo policy has none, so no transit was ever judged against a person),
    and the aircraft anywhere in +-8 m at a height inside or outside the
    10-20 m band - the re-review's fuzz, with turning zones added."""
    person = rng.random() < 0.3
    sh = Shield(_demo_with(RING) if person else load_policy(DEMO), escalation=False)
    if person:
        sh.set_subject(rng.uniform(-10, 10), rng.uniform(-10, 10), "pedestrian",
                       velocity=((rng.uniform(-2, 2), rng.uniform(-2, 2))
                                 if rng.random() < 0.5 else None))
    for j in range(rng.randint(1, 3)):
        cx, cy = rng.uniform(-15, 15), rng.uniform(-15, 15)
        w = rng.uniform(0.2, 1.5) if thin else rng.uniform(1, 6)
        h = rng.uniform(4, 40) if thin else rng.uniform(1, 40)
        ang = rng.uniform(0, math.pi)
        c, s = math.cos(ang), math.sin(ang)
        pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
        sp = (rng.choice([0.0, rng.uniform(2, 8)]) if thin else rng.uniform(0, 5))
        d = rng.uniform(0, 2 * math.pi)
        yaw = rng.choice([0.0, 0.0, rng.uniform(-20, 20)])
        sh.spawn_nfz(DynamicNFZ(
            id=f"z{j}", type="dynamic_nfz", **HARD_P0,
            vertices=[{"x": cx + c * x - s * y, "y": cy + s * x + c * y} for x, y in pts],
            altitude_floor_m=0, altitude_ceiling_m=100, margin_m=rng.choice([0.5, 1.0]),
            motion={"vx_mps": sp * math.cos(d), "vy_mps": sp * math.sin(d),
                    "yaw_rate_dps": yaw}), t=0.0)
    st = State(x=rng.uniform(-8, 8), y=rng.uniform(-8, 8),
               up=rng.choice([15.0, 15.0, 3.0, 5.0, 8.0, 22.0]))
    sh.filter(st, Action4D(), t=0.0)
    return sh, st


def test_no_moved_plan_of_a_seeded_fuzz_enters_or_dips_between_poses():
    """The re-review's fuzz (thin zones: 27 of 195 moved holds entered a zone
    they started outside of, 16 into the polygon; ordinary zones: 13 of 115
    went up to 0.72 m deeper between samples), seeded, with turning zones
    and stand-off rings: every moved hold and landing site is clean at 1000
    poses. 47e77a5 moved 60 plans here, 10 of them entering a rule or going
    deeper between its samples; now 39 move, all clean."""
    rng = random.Random(20261020)
    moved = 0
    problems = []
    for i in range(60):
        sh, st = _fuzz_shield(rng, thin=(i % 2 == 0))
        if not sh.hard_violations(sh.state_is_unsafe(st)):
            continue
        for fn in (HP.choose_hold_point, HP.choose_landing_site):
            p = fn(sh, st)
            if p.kind != "moved":
                continue
            moved += 1
            bad = _dense_problems(sh, st, p)
            if bad:
                problems.append((i, fn.__name__, (round(st.x, 2), round(st.y, 2), st.up),
                                 (round(p.x, 2), round(p.y, 2), p.up), bad[:2], len(bad)))
    assert moved >= 20, f"the fuzz moved only {moved} plans: it tests little"
    assert not problems, problems[:5]


def _corridor_shield() -> Shield:
    """A straight corridor along y = 0, 4 m wide, 10-20 m, and nothing else
    that a position can break."""
    tube = {"id": "tube", "type": "corridor", "scope": "mission", "constraint_type": "hard",
            "priority": "P0", "violation_action": "repair",
            "centerline": [{"x": 0, "y": 0}, {"x": 100, "y": 0}], "width_m": 4.0,
            "altitude_floor_m": 10.0, "altitude_ceiling_m": 20.0}
    return Shield(Policy.model_validate({"policy_id": "tube", "constraints": [tube, KIN]}),
                  escalation=False)


def test_leaving_a_corridor_is_judged_by_the_shields_own_depth():
    """Second review of the fix: "deeper" is Shield.rule_depths' measure, in
    which a corridor's lateral and vertical excess ADD. The test took a
    keep-in rule's depth as its largest single excess, so with the aircraft
    0.5 m wide of the corridor and 2 m under its floor (depth 2.5 m), a level
    transit drifting to 2 m wide (depth 4 m) passed - the 2 m under the floor
    hid the drift - and so did a climb out from 5 m under it that drifted
    2 m wider while still under the floor (5.5 m to 6.5 m). Both are refused
    by name now; a way out that really gets shallower is not."""
    sh = _corridor_shield()
    for st, end, travel, deepest in (
            (State(x=10.0, y=2.5, up=8.0), State(x=10.0, y=4.0, up=8.0), 0.375, 4.0),
            (State(x=10.0, y=2.5, up=5.0), State(x=10.0, y=4.5, up=10.5), 2.75, 6.5)):
        sh.filter(st, Action4D(), t=0.0)
        esc = [v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))]
        assert esc == ["tube"], esc
        worst = max(sh.rule_depths(HP._pose_at(st, end, travel * i / 400, 4.0, 2.0),
                                   ["tube"])["tube"] for i in range(401))
        assert abs(worst - deepest) < 0.01 and worst > sh.rule_depths(st, ["tube"])["tube"]
        assert HP.transit_breaks(sh, st, end, travel, 4.0, 2.0, esc) == ["tube"], (st, end)
        assert _transit_problems(sh, st, end, travel, 4.0, 2.0, esc), "the oracle must see it"
    # Back in and up: shallower at every instant, so legal.
    st, end = State(x=10.0, y=2.5, up=5.0), State(x=10.0, y=0.0, up=10.5)
    sh.filter(st, Action4D(), t=0.0)
    assert HP.transit_breaks(sh, st, end, 2.75, 4.0, 2.0, ["tube"]) == []
    assert not _transit_problems(sh, st, end, 2.75, 4.0, 2.0, ["tube"])
    # And the search's plan from under the corridor is clean.
    st = State(x=10.0, y=2.5, up=5.0)
    sh.filter(st, Action4D(), t=0.0)
    p = HP.choose_hold_point(sh, st)
    assert p.kind == "moved" and p.legal and not _dense_problems(sh, st, p), p


def test_the_clearance_field_is_bounded_by_its_own_measured_slope():
    """Second review of the fix: the transit test bounded the obstacle
    clearance between two poses by sqrt(2) m per metre, "each axis changes
    by at most one cell per cell". The signed field jumps 1.92 cells per
    cell across a building face (a free cell reads res * 0.924, its
    occupied neighbour -res), and Shield._distance_at's off-map branch
    carries that along-edge slope at any height. RuleClearance.h_lip is now
    the field's bound measured from the field itself; here it bounds the
    slope between every probed pair, in the grid and off it, where sqrt(2)
    does not; and a 0.3 m transit 5 m off the map past a one-cell border
    building - 3.29 m clear at both ends, 3.00 at the middle, the limit
    3.05 - is refused, while the same transit 4 m along, clear all the
    way, is not."""
    occ = np.zeros((20, 20), dtype=np.uint8)
    occ[9:11, 9:11] = 1                    # a building in the middle
    occ[0, 5] = 1                          # and one cell on the border
    pol = Policy.model_validate({"policy_id": "map", "constraints": [KIN, {
        "id": "clear", "type": "obstacle_clearance", "scope": "mission", "constraint_type": "hard",
        "priority": "P0", "violation_action": "repair", "min_clearance_m": 3.05}]})
    sh = Shield(pol, escalation=False,
                obstacle_map={"occ": occ, "res": 2.0, "ox": 0.0, "oy": 0.0})
    st = State(x=-5.0, y=9.85, up=15.0)
    sh.filter(st, Action4D(), t=0.0)
    lip = sh.rule_clearances(st)["clear"].h_lip
    rng = random.Random(7)
    worst = 0.0
    for _ in range(20000):
        x, y = rng.uniform(-12.0, 44.0), rng.uniform(-6.0, 44.0)
        a, s = rng.uniform(0.0, 2.0 * math.pi), rng.uniform(1e-3, 0.5)
        q = (x + s * math.cos(a), y + s * math.sin(a))
        worst = max(worst, abs(sh._distance_at(*q) - sh._distance_at(x, y)) / s)
    assert math.sqrt(2.0) < worst <= lip + 1e-9, (worst, lip)
    end = State(x=-5.0, y=10.15, up=15.0)
    on_way = min(sh._distance_at(-5.0, 9.85 + 0.3 * i / 300) for i in range(301))
    assert abs(on_way - 3.0) < 1e-6 and min(sh._distance_at(st.x, st.y),
                                           sh._distance_at(end.x, end.y)) > 3.28
    assert HP.transit_breaks(sh, st, end, 0.075, 4.0, 2.0, ()) == ["clear"]
    assert _transit_problems(sh, st, end, 0.075, 4.0, 2.0, ()), "the oracle must see it"
    clear_st, clear_end = State(x=-5.0, y=13.85, up=15.0), State(x=-5.0, y=14.15, up=15.0)
    assert HP.transit_breaks(sh, clear_st, clear_end, 0.075, 4.0, 2.0, ()) == []
    # A rule that is a distance to a set changes 1 m per metre.
    assert _corridor_shield().rule_clearances(st)["tube"].h_lip == 1.0


def _random_world(rng: random.Random, kind: str) -> Shield:
    """One random world for the direct fuzz: the 10-20 m band and speed caps,
    plus, by kind, a 2 or 5 m stand-off ring with a walking or standing
    pedestrian and 1-3 moving or turning bars ("zones"); a clearance rule
    (0.5-5 m) over a random 40 x 40 occupancy map at 2 m ("map"); a random
    2-4 segment corridor ("corridor"); or a 6 m square zone whose one-minute
    window opens or closes 0-6 s after the plan ("window"). Bars are added to
    the last three 3 times in 10."""
    cons = [{"id": "alt-band", "type": "altitude_envelope", "scope": "mission", "constraint_type": "hard",
             "priority": "P0", "violation_action": "repair", "alt_min_m": 10,
             "alt_max_m": 20}, KIN]
    omap, now = None, None
    if kind == "zones":
        cons.append(dict(RING, min_range_m=rng.choice([2.0, 5.0])))
    elif kind == "map":
        occ = np.zeros((40, 40), dtype=np.uint8)
        for _ in range(rng.randint(2, 6)):
            i0, j0 = rng.randrange(0, 38), rng.randrange(0, 38)
            occ[i0:i0 + rng.randint(1, 6), j0:j0 + rng.randint(1, 6)] = 1
        omap = {"occ": occ, "res": 2.0, "ox": -40.0, "oy": -40.0}
        cons.append({"id": "clear", "type": "obstacle_clearance", "scope": "mission", "constraint_type": "hard",
                     "priority": "P0", "violation_action": "repair",
                     "min_clearance_m": rng.choice([0.5, 1.0, 2.0, 3.0, 5.0])})
    elif kind == "corridor":
        pts = [{"x": rng.uniform(-20, -5), "y": rng.uniform(-20, 20)}]
        for _ in range(rng.randint(1, 3)):
            pts.append({"x": pts[-1]["x"] + rng.uniform(3, 15),
                        "y": pts[-1]["y"] + rng.uniform(-15, 15)})
        cons.append({"id": "tube", "type": "corridor", "scope": "mission", "constraint_type": "hard",
                     "priority": "P0", "violation_action": "repair", "centerline": pts,
                     "width_m": rng.uniform(2, 12),
                     "altitude_floor_m": rng.choice([8.0, 10.0, 12.0]),
                     "altitude_ceiling_m": rng.choice([16.0, 20.0])})
    else:
        cons.append({"id": "timed", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
                     "priority": "P0", "violation_action": "repair",
                     "margin_m": rng.choice([0.2, 1.0]),
                     "vertices": [{"x": x, "y": y} for x, y in
                                  ((-3, -3), (3, -3), (3, 3), (-3, 3))],
                     "altitude_floor_m": 0, "altitude_ceiling_m": 100,
                     "valid_time": {"recurrence": {"days": ["Mon"], "start_time": "10:00",
                                                   "end_time": "10:01"}}})
        edge = (datetime(2026, 10, 19, 10, 0, 0) if rng.random() < 0.5
                else datetime(2026, 10, 19, 10, 2, 0))
        t0 = edge - timedelta(seconds=rng.uniform(-1.0, 6.0))
        now = (lambda t0=t0: t0)
    sh = Shield(Policy.model_validate({"policy_id": kind, "constraints": cons}),
                escalation=False, obstacle_map=omap, now=now)
    if kind == "zones" or rng.random() < 0.3:
        for j in range(rng.randint(1, 3)):
            cx, cy = rng.uniform(-12, 12), rng.uniform(-12, 12)
            w = rng.uniform(0.2, 1.5) if rng.random() < 0.5 else rng.uniform(1, 6)
            h = rng.uniform(4, 30)
            ang = rng.uniform(0, math.pi)
            c, s = math.cos(ang), math.sin(ang)
            sp, d = rng.choice([0.0, rng.uniform(1, 8)]), rng.uniform(0, 2 * math.pi)
            sh.spawn_nfz(DynamicNFZ(
                id=f"z{j}", type="dynamic_nfz", **HARD_P0,
                vertices=[{"x": cx + c * x - s * y, "y": cy + s * x + c * y}
                          for x, y in ((-w / 2, -h / 2), (w / 2, -h / 2),
                                       (w / 2, h / 2), (-w / 2, h / 2))],
                altitude_floor_m=rng.choice([0, 0, 8, 12]),
                altitude_ceiling_m=rng.choice([100, 100, 14, 18]),
                margin_m=rng.choice([0.2, 0.5, 1.0]),
                motion={"vx_mps": sp * math.cos(d), "vy_mps": sp * math.sin(d),
                        "yaw_rate_dps": rng.choice([0.0, 0.0, rng.uniform(-40, 40)])}), t=0.0)
    if kind == "zones":
        sh.set_subject(rng.uniform(-8, 8), rng.uniform(-8, 8), "pedestrian",
                       velocity=((rng.uniform(-3, 3), rng.uniform(-3, 3))
                                 if rng.random() < 0.7 else None))
    return sh


def test_no_random_transit_called_legal_enters_or_dips():
    """The transit test itself, not only the plans the search happens to
    pick: 40 seeded worlds (10 of each kind, _random_world), four random
    straight transits from a random start in each (0.5-12 m, level or
    climbing to and past the band), escaping what a stop at the start
    breaks. Every transit transit_breaks calls legal is judged by the
    Shield's own rule_status and rule_depths at 1000 poses. The test made
    before the second review let one of these go deeper (world 26, a
    corridor: 735 of 1000 poses, by up to 1.5 cm past the tolerance), and 11
    of 151 corridor transits in a larger run, by up to metres."""
    rng = random.Random(11)
    kinds = ("zones", "map", "corridor", "window")
    legal = refused = 0
    problems = []
    for i in range(40):
        sh = _random_world(rng, kinds[i % 4])
        st = State(x=rng.uniform(-8, 8), y=rng.uniform(-8, 8),
                   up=rng.choice([15.0, 15.0, 9.0, 11.0, 5.0, 21.0]))
        sh.filter(st, Action4D(), t=0.0)
        esc = sorted({v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))})
        for _ in range(4):
            r, a = rng.choice([0.5, 1, 2, 4, 8, 12]), rng.uniform(0, 2 * math.pi)
            up = rng.choice([st.up, st.up, 15.0, 10.5, 19.5, st.up + rng.uniform(-6, 6)])
            end = State(x=st.x + r * math.cos(a), y=st.y + r * math.sin(a), up=up)
            travel = max(r / 4.0, abs(up - st.up) / 2.0)
            if HP.transit_breaks(sh, st, end, travel, 4.0, 2.0, esc):
                refused += 1
                continue
            legal += 1
            bad = _transit_problems(sh, st, end, travel, 4.0, 2.0, esc)
            if bad:
                problems.append((i, kinds[i % 4], (round(st.x, 2), round(st.y, 2), st.up),
                                 (round(end.x, 2), round(end.y, 2), round(end.up, 2)),
                                 bad[:2], len(bad)))
    assert legal >= 80 and refused >= 20, (legal, refused)
    assert not problems, problems[:4]


def _square_world(half: float, margin: float = 1.0, *extra: dict, now=None,
                  kin: bool = True) -> Shield:
    """The final review's reach world (scratch rv_t1d_scripts/reach.py): a
    square always-on zone `half` m either side of the aircraft at (0, 0, 15),
    margin 1 m, the 4 m/s cap (none with `kin` False), nothing moving - the
    nearest legal hold is half + margin + HOLD_MARGIN_M away."""
    pol = Policy.model_validate({"policy_id": "sq", "constraints": ([KIN] if kin else []) + [{
        "id": "zone", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "repair", "margin_m": margin,
        "vertices": [{"x": -half, "y": -half}, {"x": half, "y": -half},
                     {"x": half, "y": half}, {"x": -half, "y": half}],
        "altitude_floor_m": 0, "altitude_ceiling_m": 100}, *extra]})
    sh = Shield(pol, escalation=False, now=(lambda: now) if now else None)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(), t=0.0)
    return sh


def _overlapping(k: int) -> Shield:
    """k always-on zones over the whole search area (the final review's cost
    probe): nothing within 60 m is legal, and every candidate is in k boxes."""
    zones = [{"id": f"o{j}", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
              "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
              "vertices": [{"x": 0.5 * j - 100 - j, "y": -0.5 * j - 100 - j},
                           {"x": 0.5 * j + 100 + j, "y": -0.5 * j - 100 - j},
                           {"x": 0.5 * j + 100 + j, "y": -0.5 * j + 100 + j},
                           {"x": 0.5 * j - 100 - j, "y": -0.5 * j + 100 + j}],
              "altitude_floor_m": 0, "altitude_ceiling_m": 100} for j in range(k)]
    sh = Shield(Policy.model_validate({"policy_id": "over", "constraints": [KIN, *zones]}),
                escalation=False)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(), t=0.0)
    return sh


def _charged_search(sh: Shield, st: State, fn=None):
    """Run a search with its Shield calls recorded: (plan, the units its
    ledger spent, the units its calls cost by the ledger's own prices, the
    calls by kind). Not the search's: the start's own stop query (the caller
    passes `unsafe`), the hold altitude's rule_status (made before the
    ledger) and the cap and motion reads (not charged)."""
    made, calls, depth = [], [], [0]
    orig = {n: getattr(sh, n) for n in ("state_is_unsafe", "rule_status", "rule_clearances")}

    def wrap(name):
        def call(state, *a, **k):
            # Outermost calls only: outside a tick, rule_clearances opens one
            # and calls itself again inside it - one read.
            if made and not depth[0]:      # the ledger is being built, or exists
                ids = a[0] if a else k.get("rule_ids")
                calls.append((name, state, None if ids is None else list(ids)))
            depth[0] += 1
            try:
                return orig[name](state, *a, **k)
            finally:
                depth[0] -= 1
        return call

    class Recorded(HP._Ledger):
        def __init__(self, *a, **k):
            made.append(self)
            super().__init__(*a, **k)

    real = HP._Ledger
    HP._Ledger = Recorded
    for n in orig:
        setattr(sh, n, wrap(n))
    try:
        unsafe = sh.hard_violations(orig["state_is_unsafe"](st))
        p = (fn or HP.choose_hold_point)(sh, st, unsafe=unsafe)
    finally:
        HP._Ledger = real
        for n in orig:
            delattr(sh, n)
    led = made[0]
    cost, kinds = 0.0, {}
    for name, state, ids in calls:
        kinds[name] = kinds.get(name, 0) + 1
        if name == "state_is_unsafe":
            cost += led.stop_cost(state)
        elif name == "rule_status":
            cost += led.read_cost(r["id"] for r in led.rows)
        else:
            cost += led.read_cost(ids if ids is not None else [r["id"] for r in led.rows])
    return p, led.spent, cost, kinds


def test_the_budget_charges_what_the_search_makes():
    """MAX_QUERIES is in units of about 0.1 ms (THE SEARCH'S BUDGET): a stop
    query costs STOP_QUERY_COST, plus STATIC_ZONE_STOP_COST per static zone
    whose box holds the point and MOVING_ZONE_STOP_COST per moving zone the
    search can meet; a read READ_COST plus RULE_READ_COST per rule and
    MOVING_ZONE_READ_COST per moving zone. On a hold search, what the ledger
    charged is what the calls it made cost at those prices; on one that
    spends it all (a 40 m bar closing at 3 m/s) the spend is the budget to
    within one candidate, and the plan says the budget stopped it. 20
    overlapping zones and the thin wall spent it too until the cheap test
    read the rule that dropped the last candidate first (integration review
    of T1c): now the 20 zones drop every candidate out to 60 m for one zone
    read each, 416 reads for about 112 units, the wall's search reaches the
    last ring at about 592 units, and both plans say nothing is legal within
    60 m. Deterministic: a count, not a clock (the wall-clock half is
    test_each_charge_bounds_what_its_call_costs)."""
    st = State(x=0.0, y=0.0, up=15.0)
    led = HP._Ledger(_bar(3.0, 2.0, 40.0, 3.0), st, 20.0)
    # The bar can meet the search; the demo policy's nfz-square (box 6-24 m
    # both ways) holds (10, 10), not the aircraft.
    assert led.m == 1 and led.boxes == [(6.0, 6.0, 24.0, 24.0)], (led.m, led.boxes)
    # The demo policy's altitude band makes every stop query dearer.
    base = HP.STOP_QUERY_COST + HP._STOP_RULE_COST["altitude_envelope"]
    assert abs(led.stop_base - base) < 1e-12, led.stop_base
    assert abs(led.stop_cost(st) - (base + HP.MOVING_ZONE_STOP_COST)) < 1e-12
    assert abs(led.stop_cost(State(x=10.0, y=10.0, up=15.0))
               - (base + HP.MOVING_ZONE_STOP_COST + HP.STATIC_ZONE_STOP_COST)) < 1e-12
    assert abs(led.read_cost(["bar", "alt-band"]) - (
        HP.READ_COST + HP.ZONE_READ_COST + HP.RULE_READ_COST + HP.MOVING_ZONE_READ_COST)) < 1e-12
    over = _overlapping(20)
    led = HP._Ledger(over, st, 20.0)
    assert led.m == 0 and len(led.boxes) == 20
    assert abs(led.stop_cost(st) - (HP.STOP_QUERY_COST + 20 * HP.STATIC_ZONE_STOP_COST)) < 1e-12
    assert abs(led.read_cost(["o1", "o2"]) - (HP.READ_COST + 2 * HP.ZONE_READ_COST)) < 1e-12
    # (world, how it ends, stop queries made): in the 20 overlapping zones
    # every candidate is inside a still zone, so the cheap test drops each
    # one and no stop query is made at all.
    wall, low = _thin_wall()
    for sh, at, ends, stops in ((_bar(3.0, 2.0, 40.0, 3.0), st, "budget", True),
                                (over, st, "edge", False),
                                (_square_world(20.0), st, "moved", True),
                                (wall, low, "edge", True)):
        p, spent, cost, kinds = _charged_search(sh, at)
        assert abs(spent - cost) < 1e-6, (spent, cost, kinds)
        assert kinds.get("rule_clearances", 0) > 0, kinds
        assert (kinds.get("state_is_unsafe", 0) > 0) == stops, kinds
        if ends == "budget":
            worst = HP._Ledger(sh, at, 20.0).stop_cost(at) + HP.READ_COST + 30 * (
                HP.ZONE_READ_COST + HP.MOVING_ZONE_READ_COST)
            assert HP.MAX_QUERIES <= spent <= HP.MAX_QUERIES + worst, (spent, kinds)
            assert p.kind == "in_place" and not p.legal and HP.BUDGET_NOTE in p.reason, p
        else:
            assert spent < HP.MAX_QUERIES and HP.BUDGET_NOTE not in p.reason, (spent, p)
            assert (p.kind == "moved") == (ends == "moved"), (ends, p)
        if ends == "edge":
            assert not p.legal and f"within {HP.RADII_M[-1]:g} m" in p.reason, p.reason
        if sh is over:
            # One read a candidate (16 headings x 26 rings): the first reads
            # the 20 zones, each later one the zone that dropped the last.
            assert kinds["rule_clearances"] == 16 * len(HP.RADII_M), kinds
            assert spent < 0.25 * HP.MAX_QUERIES, spent


def test_a_static_world_is_searched_to_its_edge():
    """Final review of T1c, P1: every candidate cost at least one stop query,
    the budget was 300, and 16 headings over the rings out to 28 m spent it,
    so the search's advertised 60 m reach was about 28 m in every world: a
    square always-on zone around the aircraft was left at a half-width of
    26 m, and from 28 m on the hold stayed inside it, marked illegal, with
    the budget's reason (the review's reach.py). Now a candidate inside a
    rule that stands still is dropped by a cheap read before any stop query
    (THE CHEAP TEST): the 30 m and 50 m cases, and the last ring (57.5 m:
    exit at 59 m), are left to the nearest legal point, hold and landing,
    well inside the budget."""
    st = State(x=0.0, y=0.0, up=15.0)
    for half in (30.0, 50.0, 57.5):
        sh = _square_world(half)
        for fn in (HP.choose_hold_point, HP.choose_landing_site):
            p, spent, _, _ = _charged_search(sh, st, fn)
            assert p.kind == "moved" and p.legal, (half, fn.__name__, p)
            assert abs(math.hypot(p.x, p.y) - (half + 1.0 + HP.HOLD_MARGIN_M)) <= \
                HP.REFINE_STEP_M + 1e-9, (half, fn.__name__, p)
            assert spent < 0.5 * HP.MAX_QUERIES, (half, fn.__name__, spent)
    # Past the last ring the plan says so, not the budget.
    p = HP.choose_hold_point(_square_world(80.0), st)
    assert p.kind == "in_place" and not p.legal and HP.BUDGET_NOTE not in p.reason, p
    assert f"within {HP.RADII_M[-1]:g} m" in p.reason, p.reason


def _nested(k: int) -> Shield:
    """k always-on squares nested about the aircraft at (0, 0, 15), their
    half-widths evenly from 50 m down to 46.5 m, margin 1 m, the 4 m/s cap
    (the integration review of T1c): the outermost decides, so the nearest
    legal hold is 50 + 1 + HOLD_MARGIN_M = 51.5 m off, and every candidate
    nearer than 47.5 m (the innermost ring's box) is inside all k boxes."""
    zones = []
    for j in range(k):
        h = 50.0 - 3.5 * j / max(1, k - 1)
        zones.append({"id": f"n{j}", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
                      "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
                      "vertices": [{"x": -h, "y": -h}, {"x": h, "y": -h}, {"x": h, "y": h},
                                   {"x": -h, "y": h}],
                      "altitude_floor_m": 0, "altitude_ceiling_m": 100})
    sh = Shield(Policy.model_validate({"policy_id": "nest", "constraints": [KIN, *zones]}),
                escalation=False)
    sh.filter(State(x=0.0, y=0.0, up=15.0), Action4D(), t=0.0)
    return sh


def test_nested_static_zones_cost_one_zone_read_a_candidate():
    """Integration review of T1c: THE CHEAP TEST read every static zone whose
    box held a candidate and was charged for each, so a candidate inside k
    nested zones cost READ_COST + k * ZONE_READ_COST (1.65 units at k = 8),
    and the candidates nearer than the 55 m ring (384 of them) cost more
    than the 600-unit budget. With 8 or 10 squares (half-widths 50 down to
    46.5 m) the hold and the landing stayed in place, illegal, with the
    budget's reason, where 51.5 m out was legal (measured on 08cd0be: the
    budget spent after 363 and 292 reads; 6 squares still reached it, at
    494 units). Now the rule that dropped the last candidate is read first,
    alone, and the rest only if it is not breached (_Ledger.breached): a
    candidate inside the same zone costs READ_COST + ZONE_READ_COST whatever
    k, and both are moved to 51.5 m well inside the budget."""
    st = State(x=0.0, y=0.0, up=15.0)
    sh = _nested(10)
    led = HP._Ledger(sh, st, 20.0)
    # The first candidate reads every zone whose box holds it; the next one
    # inside the zone that dropped it reads that zone alone.
    left = led.left
    assert led.breached(sh, State(x=10.0, y=0.0, up=15.0))
    paid = left - led.left
    assert abs(paid - (HP.READ_COST + 10 * HP.ZONE_READ_COST)) < 1e-12, paid
    left = led.left
    assert led.breached(sh, State(x=0.0, y=12.0, up=15.0))
    paid = left - led.left
    assert abs(paid - (HP.READ_COST + HP.ZONE_READ_COST)) < 1e-12, paid
    for k in (8, 10):
        sh = _nested(k)
        for fn in (HP.choose_hold_point, HP.choose_landing_site):
            p, spent, cost, _ = _charged_search(sh, st, fn)
            assert p.kind == "moved" and p.legal, (k, fn.__name__, p)
            assert abs(math.hypot(p.x, p.y) - 51.5) <= HP.REFINE_STEP_M + 1e-9, \
                (k, fn.__name__, p)
            assert spent < 0.5 * HP.MAX_QUERIES, (k, fn.__name__, spent)
            if fn is HP.choose_hold_point:
                # Each read, the one-rule read included, is charged as what it
                # read (a landing's descent chunks are charged an estimate).
                assert abs(spent - cost) < 1e-6, (k, spent, cost)


def test_the_cheap_test_drops_only_what_the_stop_query_refuses():
    """THE CHEAP TEST drops a candidate on a still, hard, enforced rule
    breached at the point - never one the stop query would take. In 40
    seeded worlds (_random_world: stand-off rings with walking and standing
    pedestrians, clearance maps, corridors, windows opening and closing,
    moving and turning bars), 25 random points each at a random arrival
    time: every point the cheap test drops is a stop the Shield refuses, by
    one of the rules the test read."""
    rng = random.Random(21)
    kinds = ("zones", "map", "corridor", "window")
    dropped = kept = 0
    wrong = []
    for i in range(40):
        sh = _random_world(rng, kinds[i % 4])
        st = State(x=rng.uniform(-8, 8), y=rng.uniform(-8, 8), up=15.0)
        sh.filter(st, Action4D(), t=0.0)
        for site_only in (False, True):
            led = HP._Ledger(sh, st, 20.0, site_only=site_only)
            for _ in range(25):
                p = State(x=rng.uniform(-25, 25), y=rng.uniform(-25, 25),
                          up=rng.choice([15.0, 9.0, 21.0, 0.0, 12.5]))
                with sh.looking_ahead(rng.choice([0.0, 0.5, 2.0, 4.0, 7.0])):
                    if not led.breached(sh, p):
                        kept += 1
                        continue
                    dropped += 1
                    vios = sh.hard_violations(sh.state_is_unsafe(p))
                    read = set(led.pre_rules) | {rid for rid, _ in led.pre_zones}
                    if not {v.rule_id for v in vios} & read:
                        wrong.append((i, site_only, (p.x, p.y, p.up), [v.rule_id for v in vios]))
    assert dropped >= 200 and kept >= 200, (dropped, kept)
    assert not wrong, wrong[:4]


def test_a_speed_cap_that_opens_on_the_way_slows_the_whole_transit():
    """Final review of T1c, P3: the transit was planned at the caps in force
    within the lookahead (Shield.kinematic_caps), so a tighter hard cap whose
    window opened after it but before arrival was broken on the way: a
    4 m/s cap, a P0 1 m/s cap opening 5.6 s ahead, the aircraft inside a
    44 m always-on square - the plan moved to (23.5, 0) in 5.875 s at
    4 m/s, and 5 of 100 transit instants flew 4 m/s with the 1 m/s cap in
    force (the review's windowed_cap2.py). Now the caps in force through the
    longest transit the search plans are read one lookahead at a time
    (holdpoint._speeds), and the transit flies at the tighter cap from the
    start: no instant over a cap in force, and the reason says why.

    Integration review of T1c: the reason was built from one found/later
    flag for both caps, so it could call the DEFAULT speed an enforced cap
    (no cap now, a 5 m/s / 1 m/s cap opening on the way: "planned at the
    tightest enforced 4 m/s speed cap and 1 m/s climb cap in force before
    the longest transit ...", where 4 m/s is DEFAULT_SPEED_MPS) or miss a
    cap that comes into force (no cap now, a 4 m/s / 2 m/s one opening on
    the way: "no enforced speed cap in force: 4 m/s and 2 m/s climb
    assumed"). Now each cap's source is kept apart and worded on its own,
    and every case's reason is asserted whole."""
    def opening(speed: float, climb: float) -> dict:
        return {"id": "slow", "type": "kinematic_envelope", "scope": "mission", "constraint_type": "hard",
                "priority": "P0", "violation_action": "repair", "speed_max_mps": speed,
                "climb_rate_max_mps": climb, "yaw_rate_max_dps": 45.0,
                "valid_time": {"recurrence": {"days": ["Mon"], "start_time": "10:00",
                                              "end_time": "11:00"}}}

    by = "before the longest transit the search plans ends"

    def in_force(axis, v):
        return f"{axis} {v:g} m/s: the tightest enforced {axis} cap in force"

    def later(axis, v, h):
        return (f"{axis} {v:g} m/s: the tightest enforced {axis} cap, which comes into "
                f"force {by} ({h:.1f} s)")

    def assumed(axis, v, h):
        return f"{axis} {v:g} m/s: assumed, no enforced {axis} cap in force {by} ({h:.1f} s)"

    def looser(axis, v, h):
        return (f"{axis} {v:g} m/s: assumed, no enforced {axis} cap in force now and none at "
                f"or under {v:g} m/s {by} ({h:.1f} s)")

    st = State(x=0.0, y=0.0, up=15.0)
    # (seconds until the cap opens, its speed and climb caps, the 4 / 2 m/s
    # cap always on, the plan's speed and climb, the reason's words). A cap
    # opening only after the longest transit could end (40 s) changes
    # nothing.
    cases = (
        (5.6, (1.0, 1.0), True, 1.0, 1.0, later("speed", 1, 60), later("climb", 1, 60)),
        (12.0, (1.0, 1.0), True, 1.0, 1.0, later("speed", 1, 60), later("climb", 1, 60)),
        (5.6, (1.0, 3.0), True, 1.0, 2.0, later("speed", 1, 60), in_force("climb", 2)),
        (5.6, (5.0, 1.0), False, 4.0, 1.0, looser("speed", 4, 15), later("climb", 1, 15)),
        (5.6, (4.0, 2.0), False, 4.0, 2.0, later("speed", 4, 15), later("climb", 2, 15)),
        (40.0, (1.0, 1.0), True, 4.0, 2.0, in_force("speed", 4), in_force("climb", 2)),
        (40.0, (1.0, 1.0), False, 4.0, 2.0, assumed("speed", 4, 15), assumed("climb", 2, 15)),
    )
    for ahead, caps, kin, speed, climb, speed_words, climb_words in cases:
        now = datetime(2026, 10, 19, 10, 0, 0) - timedelta(seconds=ahead)
        sh = _square_world(22.0, 1.0, opening(*caps), now=now, kin=kin)
        p = HP.choose_hold_point(sh, st)
        case = (ahead, caps, kin)
        assert p.kind == "moved" and p.legal, (case, p)
        assert (p.speed_mps, p.climb_mps) == (speed, climb), (case, p)
        words = f"({speed_words}; {climb_words})"
        assert words in p.reason, (case, words, p.reason)
        end = State(x=p.x, y=p.y, up=p.up)
        d = math.hypot(p.x, p.y)
        v = Action4D(vx=p.speed_mps * p.x / d, vy=p.speed_mps * p.y / d)
        for i in range(1, 101):
            tau = p.travel_s * i / 100
            if tau >= p.travel_s - 1e-9:
                break
            q = HP._pose_at(st, end, tau, p.speed_mps, p.climb_mps)
            with sh.looking_ahead(tau):
                over = [r["id"] for r in sh.rule_status(q, v)
                        if r["type"] == "kinematic_envelope" and r["breach"]]
            assert not over, (case, tau, over)


def test_the_start_of_a_turning_zone_being_left_is_bounded_between_poses():
    """Final review of T1c, P5 (a coverage gap): no test pinned the rate at
    which a stop at the start can get shallower in a MOVING rule being left
    (transit_breaks.rates' `ls`): set to 0, both hold test files and 426
    adversarial fuzz transits still passed. For a zone that only translates
    the start's depth is concave in time (the least of linear distances to
    its edges), so the ends' average never overstates it where it matters
    and the term is idle - which is why the fuzz could not see it. For an
    L-shaped zone that turns (-53 deg/s, 4.7 m/s; found by a seeded search,
    2026-10-21) it is not: with LEAVE_TOL_M at 0, this transit out of the
    zone goes up to 1.1 cm deeper than a stop at the start between two
    evaluated poses (the dense oracle, 1500 poses), transit_breaks refuses
    it, and the same code with ls = 0 accepted it (checked by mutation when
    the test was written). At the shipped 1 cm tolerance the claim holds
    either way; the test pins the bound, not a flight."""
    zone = DynamicNFZ(id="z", type="dynamic_nfz", **HARD_P0, vertices=[
        {"x": 2.396358572284961, "y": -5.098755250654967},
        {"x": 4.718608957224717, "y": -1.3929372773947586},
        {"x": 0.8331504836065773, "y": 1.0418848053771956},
        {"x": -0.3279747088633007, "y": -0.8110241812529084},
        {"x": -4.21343318248144, "y": 1.6237979015190458},
        {"x": -5.374558374951318, "y": -0.22911108511105827}],
        altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.5,
        motion={"vx_mps": 2.2642283198335083, "vy_mps": 4.082530812152869,
                "yaw_rate_dps": -53.01150607113388})
    sh = Shield(load_policy(DEMO), escalation=False)
    sh.spawn_nfz(zone, t=0.0)
    st = State(x=0.0, y=0.0, up=15.0)
    sh.filter(st, Action4D(), t=0.0)
    assert "z" in {v.rule_id for v in sh.hard_violations(sh.state_is_unsafe(st))}
    end = State(x=1.9664562273650419, y=2.5305850936578915, up=15.0)
    travel = 0.8012026511121528
    keep = HP.LEAVE_TOL_M
    HP.LEAVE_TOL_M = 0.0
    try:
        dense = _transit_problems(sh, st, end, travel, 4.0, 2.0, ("z",), n=1500)
        got = HP.transit_breaks(sh, st, end, travel, 4.0, 2.0, ("z",))
    finally:
        HP.LEAVE_TOL_M = keep
    deeper = [k[3] for k in dense if k[0] == "DEEPER"]
    assert deeper and max(deeper) > 0.005, dense[:3]
    assert got == ["z"], got


def test_each_charge_bounds_what_its_call_costs():
    """The budget is a count, so it bounds the search's time only if no call
    costs more than it is charged. Here, on this machine, best of 3: the
    unit (a stop query that meets no zone, over STOP_QUERY_COST), then each
    priced call - a stop query in 20 zones' boxes, one against a moving
    zone, a read of 20 zones, a read of a moving zone in a fresh view - and
    three fruitless searches whole (a bar closing at 3 m/s, 20 overlapping
    zones, 20 diamonds whose boxes hold every candidate), each within twice
    what its charge predicts from the unit. Ratios, not milliseconds, so a
    slower machine does not fail it.

    WALL CLOCK, SO OPT-IN: it runs with GUARDRAIL_TIMING_TESTS=1 and returns
    SKIP otherwise. Its predecessor (test_the_budget_bounds_the_search_
    inside_the_tick, best of 3 for two of its three timings, 25 % headroom)
    failed 1 of 40 runs with eight copies in parallel on the desktop (final
    review of T1c, P4), and the CI runners are shared; the deterministic half
    - every charge is what the call made - is test_the_budget_charges_what_
    the_search_makes, which runs everywhere."""
    if os.environ.get("GUARDRAIL_TIMING_TESTS") != "1":
        print("      (wall clock: set GUARDRAIL_TIMING_TESTS=1 to run it)")
        return SKIP

    def best(fn, n=100):
        out = math.inf
        for _ in range(3):
            t = time.perf_counter()
            for i in range(n):
                fn(i)
            out = min(out, (time.perf_counter() - t) / n)
        return out

    def stop_at(sh, x, y):
        def f(i):
            with sh.looking_ahead(1.0 + 0.001 * i):
                sh.state_is_unsafe(State(x=x + 0.001 * i, y=y, up=15.0))
        return f

    def view(sh):
        def f(i):
            with sh.looking_ahead(1.0 + 0.001 * i):
                pass
        return f

    st = State(x=0.0, y=0.0, up=15.0)
    far = _square_world(10.0)
    unit = (best(stop_at(far, 40.0, 40.0)) - best(view(far))) / HP.STOP_QUERY_COST
    checks = []
    over = _overlapping(20)
    led = HP._Ledger(over, st, 20.0)
    checks.append(("stop, 20 zone boxes", best(stop_at(over, 0.0, 0.0)) - best(view(over)),
                   led.stop_cost(st)))
    ids = [f"o{j}" for j in range(20)]

    def read(i):
        with over.looking_ahead(1.0 + 0.001 * i):
            over.rule_clearances(State(x=0.001 * i, y=0.0, up=15.0), ids)
    checks.append(("read of 20 zones", best(read) - best(view(over)), led.read_cost(ids)))
    bar = _bar(3.0, 2.0, 40.0, 3.0)
    led = HP._Ledger(bar, st, 20.0)
    checks.append(("stop, a moving zone", best(stop_at(bar, -1.0, 1.0)) - best(view(bar)),
                   led.stop_cost(State(x=-1.0, y=1.0, up=15.0))))

    def read_bar(i):
        with bar.looking_ahead(1.0 + 0.001 * i):
            bar.rule_clearances(State(x=-1.0, y=1.0, up=15.0), ["bar"])
    checks.append(("read of a moving zone", best(read_bar) - best(view(bar)),
                   led.read_cost(["bar"])))
    for name, took, charged in checks:
        assert took <= 2.0 * charged * unit, (name, took * 1e3, charged, unit * 1e3)
    diamonds = Policy.model_validate({"policy_id": "d", "constraints": [KIN, {
        "id": "core", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard", "priority": "P0",
        "violation_action": "repair", "margin_m": 1.0,
        "vertices": [{"x": -80, "y": -80}, {"x": 80, "y": -80}, {"x": 80, "y": 80},
                     {"x": -80, "y": 80}], "altitude_floor_m": 0, "altitude_ceiling_m": 100}] + [
        {"id": f"d{q}{j}", "type": "polygon_fence", "scope": "mission", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "margin_m": 1.0,
         "vertices": [{"x": sx * 40 + r, "y": sy * 40}, {"x": sx * 40, "y": sy * 40 + r},
                      {"x": sx * 40 - r, "y": sy * 40}, {"x": sx * 40, "y": sy * 40 - r}],
         "altitude_floor_m": 0, "altitude_ceiling_m": 100}
        for q, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1)))
        for j, r in enumerate(40.0 + 0.2 * k for k in range(5))]})
    dsh = Shield(diamonds, escalation=False)
    dsh.filter(st, Action4D(), t=0.0)

    class Kept(HP._Ledger):
        """The search's ledger, kept to read what it spent (no call wrapped)."""
        made: list = []

        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            Kept.made.append(self)

    # The bar spends the budget; since the cheap test reads the rule that
    # dropped the last candidate first (integration review of T1c), the 20
    # overlapping zones and the diamonds' core drop every candidate for one
    # zone read and end at the last ring - each within twice what it spent.
    for name, sh, budget in (("bar closing at 3 m/s", _bar(3.0, 2.0, 40.0, 3.0), True),
                             ("20 overlapping zones", _overlapping(20), False),
                             ("20 diamonds", dsh, False)):
        took = math.inf
        real = HP._Ledger
        HP._Ledger = Kept
        try:
            for _ in range(3):
                t = time.perf_counter()
                p = HP.choose_hold_point(sh, st)
                took = min(took, time.perf_counter() - t)
        finally:
            HP._Ledger = real
        spent = Kept.made[-1].spent
        assert p.kind == "in_place" and not p.legal, (name, p)
        assert (HP.BUDGET_NOTE in p.reason) == budget, (name, p.reason)
        assert took <= 2.0 * spent * unit, (name, took * 1e3, spent, unit * 1e3)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    skipped = 0
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
