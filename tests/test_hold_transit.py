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

DEMO = ROOT / "policies" / "sim_demo_policy.yaml"
DENSE = 1000
KIN = {"id": "kin", "type": "kinematic_envelope", "constraint_type": "hard",
       "priority": "P1", "violation_action": "repair", "speed_max_mps": 4.0,
       "climb_rate_max_mps": 2.0, "yaw_rate_max_dps": 45.0}
RING = {"id": "ring", "type": "subject_standoff", "constraint_type": "hard",
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
    bar = DynamicNFZ(id="bar", type="dynamic_nfz",
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
        return {"id": zid, "type": "polygon_fence", "constraint_type": "hard",
                "priority": "P0", "violation_action": "repair", "margin_m": margin,
                "vertices": [{"x": x0, "y": y0}, {"x": x1, "y": y0}, {"x": x1, "y": y1},
                             {"x": x0, "y": y1}],
                "altitude_floor_m": floor, "altitude_ceiling_m": ceil}
    pol = Policy.model_validate({"policy_id": "thin", "constraints": [
        {"id": "alt-band", "type": "altitude_envelope", "constraint_type": "hard",
         "priority": "P0", "violation_action": "repair", "alt_min_m": 10, "alt_max_m": 20},
        {"id": "kin", "type": "kinematic_envelope", "constraint_type": "hard",
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
        return DynamicNFZ(id=zid, type="dynamic_nfz",
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
    spin = DynamicNFZ(id="spin", type="dynamic_nfz",
                      vertices=[{"x": -0.1, "y": 2.0}, {"x": 0.1, "y": 2.0},
                                {"x": 0.1, "y": 12.0}, {"x": -0.1, "y": 12.0}],
                      altitude_floor_m=0, altitude_ceiling_m=100, margin_m=0.2,
                      motion={"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_dps": 60.0})
    kin = {"id": "kin", "type": "kinematic_envelope", "constraint_type": "hard",
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
        {"id": "ring", "type": "subject_standoff", "constraint_type": "hard",
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
            id=f"z{j}", type="dynamic_nfz",
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
    tube = {"id": "tube", "type": "corridor", "constraint_type": "hard",
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
        "id": "clear", "type": "obstacle_clearance", "constraint_type": "hard",
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
    cons = [{"id": "alt-band", "type": "altitude_envelope", "constraint_type": "hard",
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
        cons.append({"id": "clear", "type": "obstacle_clearance", "constraint_type": "hard",
                     "priority": "P0", "violation_action": "repair",
                     "min_clearance_m": rng.choice([0.5, 1.0, 2.0, 3.0, 5.0])})
    elif kind == "corridor":
        pts = [{"x": rng.uniform(-20, -5), "y": rng.uniform(-20, 20)}]
        for _ in range(rng.randint(1, 3)):
            pts.append({"x": pts[-1]["x"] + rng.uniform(3, 15),
                        "y": pts[-1]["y"] + rng.uniform(-15, 15)})
        cons.append({"id": "tube", "type": "corridor", "constraint_type": "hard",
                     "priority": "P0", "violation_action": "repair", "centerline": pts,
                     "width_m": rng.uniform(2, 12),
                     "altitude_floor_m": rng.choice([8.0, 10.0, 12.0]),
                     "altitude_ceiling_m": rng.choice([16.0, 20.0])})
    else:
        cons.append({"id": "timed", "type": "polygon_fence", "constraint_type": "hard",
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
                id=f"z{j}", type="dynamic_nfz",
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


def _counted_search(sh: Shield, st: State, fn=None):
    """Run a search, counting the Shield calls it makes: (plan, charged
    units, stop queries, transit evaluations). Not the search's: the start's
    own stop query (unsafe=None) and the hold altitude's rule_status. A
    transit check reads the start once (rule_status and rule_clearances, one
    evaluation) and each evaluated pose once (with the start again, for a
    moving rule being left); _charges reads the start's rooms once."""
    calls = {"stop": 0, "status": 0, "pose": 0}
    orig_stop, orig_status, orig_clear = sh.state_is_unsafe, sh.rule_status, sh.rule_clearances

    def stop(*a, **k):
        calls["stop"] += 1
        return orig_stop(*a, **k)

    def status(*a, **k):
        calls["status"] += 1
        return orig_status(*a, **k)

    def clear(state, *a, **k):
        calls["pose"] += state is not st
        return orig_clear(state, *a, **k)
    sh.state_is_unsafe, sh.rule_status, sh.rule_clearances = stop, status, clear
    try:
        p = (fn or HP.choose_hold_point)(sh, st)
    finally:
        del sh.state_is_unsafe, sh.rule_status, sh.rule_clearances
    stop_cost, eval_cost, _ = HP._charges(sh, st, 20.0)
    stops, evals = calls["stop"] - 1, (calls["status"] - 1) + calls["pose"]
    return p, stops * stop_cost + evals * eval_cost, stops, evals


def test_the_budget_charges_what_the_search_makes():
    """MAX_QUERIES is in stop-query units: a stop query costs 1 plus
    MOVING_ZONE_STOP_COST per moving zone the search can meet, a transit
    evaluation TRANSIT_EVAL_COST plus MOVING_ZONE_EVAL_COST per zone
    (_charges). On a search that spends it all - a 40 m bar closing at
    3 m/s, nothing clean within reach - what it charged is what it made,
    and the plan says the budget stopped it."""
    sh = _bar(3.0, 2.0, 40.0, 3.0)
    st = State(x=0.0, y=0.0, up=15.0)
    assert HP._charges(sh, st, 20.0) == (1.0 + HP.MOVING_ZONE_STOP_COST,
                                          HP.TRANSIT_EVAL_COST + HP.MOVING_ZONE_EVAL_COST, 1)
    p, spent, stops, evals = _counted_search(sh, st)
    assert p.kind == "in_place" and not p.legal and p.breaks == ("bar",), p
    assert stops > 0 and evals > 0, (stops, evals)
    worst = 1.0 + HP.MOVING_ZONE_STOP_COST
    assert HP.MAX_QUERIES - worst - 1e-9 <= spent <= HP.MAX_QUERIES + worst + 1e-9, (
        spent, stops, evals)
    assert f"budget of {HP.MAX_QUERIES:g} stop queries" in p.reason, p.reason
    # A world with no moving zone: a stop query is one unit.
    sh, st = _thin_wall()
    assert HP._charges(sh, st, 20.0) == (1.0, HP.TRANSIT_EVAL_COST, 0)


def test_the_budget_bounds_the_search_inside_the_tick():
    """The budget is deterministic (a count, never a clock), so it bounds the
    search's time only if each call really costs what it is charged. Here,
    on this machine, best of 3: a full transit evaluation (the pose and the
    start, every rule) against a stop query, in a world with no moving zone
    and one with one; and the fruitless search's whole time against what
    its charges predict from the stop query's own time. Ratios, not
    milliseconds, so a slower machine does not fail it; the milliseconds of
    the desktop are in docs/DESIGN-hold-points.md ("Cost")."""
    def best(fn, n):
        out = math.inf
        for _ in range(3):
            t = time.perf_counter()
            for i in range(n):
                fn(i)
            out = min(out, (time.perf_counter() - t) / n)
        return out

    st = State(x=0.0, y=0.0, up=15.0)
    for sh in (_bar(3.0, 2.0, 40.0, 3.0), Shield(load_policy(DEMO), escalation=False)):
        sh.filter(st, Action4D(), t=0.0)
        stop_cost, eval_cost, m = HP._charges(sh, st, 20.0)

        def stop_q(i, sh=sh):
            with sh.looking_ahead(1.0 + 0.001 * i):
                sh.state_is_unsafe(State(x=-1.0 + 0.002 * i, y=1.0, up=15.0))
        ids = tuple(sorted(sh.rule_clearances(st)))

        def ev(i, sh=sh, ids=ids):
            with sh.looking_ahead(0.5 + 0.001 * i):
                sh.rule_clearances(State(x=-1.0 - 0.01 * i, y=3.0, up=15.0), ids)
                sh.rule_clearances(st, ids)
        stop_s, eval_s = best(stop_q, 200), best(ev, 200)
        assert eval_s / stop_s <= 1.5 * eval_cost / stop_cost, (
            m, eval_s * 1e3, stop_s * 1e3, eval_cost, stop_cost)
        if m:
            t = time.perf_counter()
            p = HP.choose_hold_point(sh, st)
            took = time.perf_counter() - t
            assert p.kind == "in_place", p
            assert took <= 1.6 * HP.MAX_QUERIES / stop_cost * stop_s, (
                took * 1e3, stop_s * 1e3, stop_cost)


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
