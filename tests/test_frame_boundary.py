"""WP3-04 / ARCH-04: body in, body out at the Shield; ONE body -> local-NED
crossing, in every rail's adapter, after the projection.

Run either way:
    pytest tests/test_frame_boundary.py -v
    python tests/test_frame_boundary.py

The grant (kuanting-vla-uav-guardrail/docs/02-implementation/safety-shield.md):
"Action input frame: body frame (vx, vy, vz, yaw_rate). This is the contract
with the VLA. The body->local-NED conversion happens in the MAVLink adapter,
*after* projection - keeps repair operators acting on the same frame the VLA
emits in", and in "MAVLink adapter": "Body-frame -> local-NED conversion lives
here, applied to every emitted setpoint. Repair operators above never see
local-NED; the conversion is a single boundary in the codebase."

Pinned here:
  * THE ROUND TRIP. The Shield's projection computes in the world frame its
    rules are written in, between `from_body` and `to_body` at the state's
    heading. That round trip is exact at 0, 90, -90 (and 180) degrees - bit
    for bit, signed zeros and a non-finite channel included - and within a
    few ulp at 179.9 and at random headings.
  * BODY IN, BODY OUT, AND THE REPAIR IS THE BODY-FRAME ONE. The same body
    action, at the same place in a scene that turns with the aircraft (a
    zone ahead of the nose, the kinematic and altitude envelopes), gives the
    same emitted BODY action, the same rules and the same repair operators
    at headings 0, 90, 179.9, -90, 37.3 and -123.4: the repair is a function
    of the scene as the VLA sees it, not of the world's axes. The same for
    each rule family: stand-off (StandoffHold, StandoffRecover, the overhead
    recovery straight back), PathRepair's stand-off detour, the corridor at
    those headings; the clearance rule on a raster map, its direction search
    included, at the four quarter turns (where a raster turns exactly). This
    is the sense in which the grant's own reference implementation keeps its
    repair "on the same frame the VLA emits in": its operators take and
    return the body action and turn it into ENU for their geometry
    (safety_shield/repair.py body_to_enu ... enu_to_body).
  * PRODUCER-SIDE SMOOTHING. The demo loops' rate limiters cap the change of
    the body action's WORLD reading (frames.limit_in_world), as they capped
    the world action before wf21, through a turn too; frames.rebase_body
    turns a body action from its capture heading to a later one.
  * ONE CROSSING. Parsed from the source of every module outside tests/ and
    the reference checkout: `from_body` / `to_local_ned` are referenced only
    by guardrail/frames.py, by the Shield's projection (filter_body,
    advise_body, check_body) and by one named record helper;
    `body_to_local_ned` - the adapter function - is called only by the
    listed adapters, and each of them calls it; no velocity command,
    Action4D or vx / vy assignment uses cos(E) and sin(E) of one angle E,
    whatever it is called, except at the listed TRIG_SITES; every call of
    the Shield's world-frame entries (.filter / .advise / ._check with a
    state and an action, on any receiver) outside the Shield is a listed
    judge or offline tool, and none is in a module that flies. The
    detectors are mutation-tested on planted modules. A new crossing fails
    here.
  * THE REAL HEADING. The pymavlink SITL rail (stand-in autopilot at 37
    degrees), the PAS ArduPilot rail (fake flight at 0.6 rad) and the
    headless harness hand the Shield the BODY action at the aircraft's own
    heading - the body action is NOT the world action relabelled - and the
    adapter's NED setpoint is body_to_local_ned of the Shield's emitted
    body action at that heading.
"""
import ast
import math
import random
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from guardrail.frames import (BODY_STOP, body_action,                 # noqa: E402
                              body_to_local_ned, filter_world_as_body,
                              from_body, local_ned_to_world, streamed_body,
                              to_body)
from guardrail.models import (Action4D, AltitudeEnvelope,            # noqa: E402
                              Corridor, KinematicEnvelope, Policy, PolygonFence,
                              State, SubjectStandoff, XY)
from guardrail.shield import Shield                                  # noqa: E402

SKIP = "SKIP"
GRANT_HEADINGS = (0.0, 90.0, 179.9, -90.0)


def _bits(*xs) -> bytes:
    return struct.pack(f"<{len(xs)}d", *xs)


def _same(a, b) -> bool:
    """Channel by channel: both NaN (a NaN's sign is not a value), or the
    same bits (so -0.0 is not 0.0)."""
    return all((math.isnan(x) and math.isnan(y)) or _bits(x) == _bits(y)
               for x, y in zip(a, b))


# --------------------------------------------------------------------------- #
# The round trip
# --------------------------------------------------------------------------- #

def test_the_round_trip_is_exact_to_float_tolerance_at_the_grant_headings():
    """to_body(from_body(b, h), h) == b: to the bit at 0, 90 and -90 (a
    quarter turn is a permutation, frames._quarter_turns), within 4 ulp of
    the action's size at 179.9 and at 200 random headings; and the other way
    round (world -> body -> world) the same."""
    rng = random.Random(20261118)
    headings = list(GRANT_HEADINGS) + [180.0, -180.0, 270.0, 360.0] + [
        rng.uniform(-720.0, 720.0) for _ in range(200)]
    worst = {}
    n = 0
    for h in headings:
        exact = h % 90.0 == 0.0
        for _ in range(60):
            b = (rng.uniform(-20.0, 20.0), rng.uniform(-20.0, 20.0),
                 rng.uniform(-5.0, 5.0), rng.uniform(-2.0, 2.0))
            w = from_body(*b, h)
            back = to_body(w, h)
            w2 = from_body(*back, h)
            size = max(1.0, abs(b[0]), abs(b[1]))
            err = max(abs(back[0] - b[0]), abs(back[1] - b[1])) / size
            err2 = max(abs(w2.vx - w.vx), abs(w2.vy - w.vy)) / size
            assert back[2:] == b[2:] and (w.vz_up, w.yaw_rate) == b[2:], \
                "the vertical and yaw channels are never rotated"
            if exact:
                assert _bits(*back) == _bits(*b), (h, b, back)
            assert err <= 4 * 2.2e-16 and err2 <= 4 * 2.2e-16, (h, b, back, err, err2)
            worst[h] = max(worst.get(h, 0.0), err)
            n += 1
    assert n == len(headings) * 60
    for h in (0.0, 90.0, -90.0):
        assert worst[h] == 0.0, (h, worst[h])
    assert worst[179.9] > 0.0, "179.9 is not a quarter turn: its round trip is not exact"


def test_a_quarter_turn_keeps_signed_zeros_and_a_nonfinite_channel_in_place():
    """At a quarter turn the rotation is a permutation, so -0.0 stays -0.0
    and a NaN in one world channel stays in one body channel (through
    cos/sin, 0 * NaN would put it in both and the Shield would zero both).
    The headless nonfinite-action probe flies at heading 0 and relies on it."""
    nan = float("nan")
    for h, world, body in ((0.0, (nan, 2.0), (nan, 2.0)), (90.0, (nan, 2.0), (2.0, nan)),
                           (-90.0, (2.0, nan), (nan, 2.0)), (180.0, (-0.0, 0.0), (0.0, -0.0))):
        got = to_body(Action4D(vx=world[0], vy=world[1]), h)[:2]
        assert _same(got, body), (h, got, body)
        back = from_body(got[0], got[1], 0.0, 0.0, h)
        assert _same((back.vx, back.vy), world), (h, back)


def test_body_to_local_ned_is_from_body_with_down_positive():
    """The adapter function: (v_north, v_east, v_down, yaw_rate rad/s)."""
    for h in GRANT_HEADINGS + (37.0,):
        for b in ((2.0, 0.0, 0.5, 0.3), (0.0, 1.5, -0.4, -0.2), (-1.0, -1.0, 0.0, 0.7)):
            w = from_body(*b, h)
            ned = body_to_local_ned(b, h)
            assert _bits(*ned) == _bits(w.vx, w.vy, -w.vz_up, w.yaw_rate), (h, b)
            assert body_to_local_ned(Action4D(vx=b[0], vy=b[1], vz_up=b[2],
                                              yaw_rate=b[3]), h) == ned
            back = local_ned_to_world(ned)
            assert _bits(back.vx, back.vy, back.vz_up, back.yaw_rate) == \
                _bits(w.vx, w.vy, w.vz_up, w.yaw_rate)
    n, e, d, _ = body_to_local_ned((1.0, 0.0, 1.0, 0.0), 90.0)
    assert (n, e, d) == (0.0, 1.0, -1.0), "forward at heading East is East; up is -down"


# --------------------------------------------------------------------------- #
# Body in, body out; the repair is the body-frame one
# --------------------------------------------------------------------------- #

HARD = {"constraint_type": "hard", "scope": "mission", "priority": "P0",
        "violation_action": "project_fix"}
# A zone ahead of the nose, in (forward, right) metres.
AHEAD = [(8.0, -4.0), (16.0, -4.0), (16.0, 4.0), (8.0, 4.0)]


def _scene(x0: float, y0: float, h: float) -> Policy:
    """The zone AHEAD placed in the world at heading `h` from (x0, y0), with a
    kinematic and an altitude envelope (which do not depend on a heading)."""
    verts = []
    for f, r in AHEAD:
        w = from_body(f, r, 0.0, 0.0, h)
        verts.append(XY(x=x0 + w.vx, y=y0 + w.vy))
    return Policy(policy_id="body-frame-scene", version="0.0.1", constraints=[
        PolygonFence(id="nfz-ahead", type="polygon_fence", vertices=verts, margin_m=1.0, **HARD),
        KinematicEnvelope(id="kin", type="kinematic_envelope", scope="mission",
                          constraint_type="hard", priority="P1", violation_action="repair",
                          speed_max_mps=6.0, climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0),
        AltitudeEnvelope(id="alt", type="altitude_envelope", scope="mission",
                         constraint_type="hard", priority="P1", violation_action="repair",
                         alt_min_m=5.0, alt_max_m=40.0),
    ])


def test_the_same_body_action_in_the_same_body_scene_is_repaired_the_same_at_every_heading():
    """Body in, body out (Shield.filter_body), and the repair acts in the
    frame the VLA emits in: rotate the scene with the aircraft and the
    emitted BODY action, the rules broken and the repair operators do not
    change, to 1e-12 m/s, at the grant's headings and two others. Counted,
    not blind: most cases break the zone ahead and are repaired."""
    rng = random.Random(1118)
    x0, y0 = 100.0, -50.0
    worst, n_zone, n_repaired, n = 0.0, 0, 0, 0
    for k in range(40):
        body = Action4D(vx=rng.uniform(-2.0, 7.0), vy=rng.uniform(-3.0, 3.0),
                        vz_up=rng.uniform(-3.0, 3.0), yaw_rate=rng.uniform(-1.0, 1.0))
        up = rng.uniform(4.0, 30.0)
        ref = None
        for h in GRANT_HEADINGS + (37.3, -123.4):
            d, em = Shield(_scene(x0, y0, h)).filter_body(
                State(x=x0, y=y0, up=up, yaw_deg=h), body)
            got = (em, sorted(v.rule_id for v in d.violations), [r.operator for r in d.repairs])
            if ref is None:
                ref = got
                n_zone += "nfz-ahead" in got[1]
                n_repaired += bool(got[2])
                continue
            dd = max(abs(em.vx - ref[0].vx), abs(em.vy - ref[0].vy),
                     abs(em.vz_up - ref[0].vz_up), abs(em.yaw_rate - ref[0].yaw_rate))
            worst = max(worst, dd)
            assert dd <= 1e-12 and got[1:] == ref[1:], (k, h, em, ref[0], got[1:], ref[1:])
            n += 1
    assert n == 40 * 5
    assert n_zone >= 10 and n_repaired >= 20, (n_zone, n_repaired)


def test_filter_world_as_body_hands_over_every_tick_in_the_body_frame():
    """A world-frame producer's action is handed to the Shield in the BODY
    frame on every tick (no fall-back to Shield.filter), and what the rail
    streams is body frame: the emitted body action, a stop, or with the
    Shield off the body action handed in."""
    calls = []

    class Spy(Shield):
        def filter(self, *a, **k):
            calls.append("filter")
            return super().filter(*a, **k)

        def filter_body(self, state, body, t=None, **k):
            calls.append(("filter_body", state.yaw_deg, body))
            return super().filter_body(state, body, t, **k)

    rng = random.Random(3)
    sh = Spy(_scene(0.0, 0.0, 0.0))
    for i in range(50):
        h = rng.uniform(-180.0, 180.0)
        w = Action4D(vx=rng.uniform(-6, 6), vy=rng.uniform(-6, 6), vz_up=rng.uniform(-2, 2))
        st = State(x=rng.uniform(-5, 5), y=rng.uniform(-5, 5), up=15.0, yaw_deg=h)
        d, b_in, eb = filter_world_as_body(sh, st, w, t=0.1 * i)
        assert calls[-2:] == [("filter_body", h, b_in), "filter"], calls[-2:]
        assert b_in == body_action(w, h)
        want = {None: eb, "pass": eb, "brake": BODY_STOP, "none": None}[d.setpoint]
        assert streamed_body(d, eb) == want, (d.setpoint, streamed_body(d, eb), want)
        assert streamed_body(d, eb, shield_on=False, body_in=b_in) is b_in
    assert streamed_body(type("D", (), {"setpoint": "brake"})(), eb) == BODY_STOP
    assert streamed_body(type("D", (), {"setpoint": "none"})(), eb) is None
    assert sum(1 for c in calls if c == "filter") == 50


# Every rule family, the same way: a scene built in the aircraft's own
# (forward, right) metres, placed in the world at heading h, and the same body
# action at every heading. The grant's own reference implementation keeps its
# repair "on the same frame the VLA emits in" the same way - each operator
# takes and returns the body action and turns it into the IR's ENU plane for
# its geometry (kuanting-vla-uav-guardrail packages/safety-shield/src/
# safety_shield/repair.py: body_to_enu ... enu_to_body) - so what the frame
# of the repair means, checkably, is this: it does not depend on the world's
# axes.

ROT_HEADINGS = GRANT_HEADINGS + (37.3, -123.4)
QUARTER_TURNS = (0.0, 90.0, 180.0, -90.0)
X0, Y0 = 100.0, -50.0


def _kin() -> KinematicEnvelope:
    return KinematicEnvelope(id="kin", type="kinematic_envelope", scope="mission",
                             constraint_type="hard", priority="P1", violation_action="repair",
                             speed_max_mps=6.0, climb_rate_max_mps=2.0, yaw_rate_max_dps=45.0)


def _at(h: float, f: float, r: float, x0: float = X0, y0: float = Y0) -> tuple:
    """The world point (forward f, right r) of (x0, y0) at heading h."""
    w = from_body(f, r, 0.0, 0.0, h)
    return x0 + w.vx, y0 + w.vy


def _bodies(seed: int, n: int = 40) -> list:
    rng = random.Random(seed)
    return [Action4D(vx=rng.uniform(-2.0, 6.0), vy=rng.uniform(-3.0, 3.0),
                     vz_up=rng.uniform(-1.0, 1.0), yaw_rate=rng.uniform(-0.5, 0.5))
            for _ in range(n)]


def _same_repair_at_every_heading(make, bodies, headings, tol=1e-12) -> dict:
    """make(h) -> (a fresh Shield, the State at heading h) for the scene at
    heading h. Asserts the emitted BODY action (to `tol`), the rules broken
    and the repair operators are the heading-`headings[0]` ones at every
    other heading; returns how often each operator ran there."""
    ops: dict = {}
    for k, body in enumerate(bodies):
        ref = None
        for h in headings:
            sh, st = make(h)
            d, em = sh.filter_body(st, body)
            got = (em, sorted(v.rule_id for v in d.violations), [r.operator for r in d.repairs])
            if ref is None:
                ref = got
                for o in got[2]:
                    ops[o] = ops.get(o, 0) + 1
                continue
            dd = max(abs(em.vx - ref[0].vx), abs(em.vy - ref[0].vy),
                     abs(em.vz_up - ref[0].vz_up), abs(em.yaw_rate - ref[0].yaw_rate))
            assert dd <= tol and got[1:] == ref[1:], (k, h, body, em, ref[0], got[1:], ref[1:])
    return ops


def _standoff_scene(h, at=(14.0, 2.0), velocity=None, path_repair=True):
    pol = Policy(policy_id="body-frame-standoff", version="0.0.1", constraints=[
        SubjectStandoff(id="so", type="subject_standoff", subject_class="*", min_range_m=10.0,
                        soft_margin_m=2.0, **HARD),
        _kin()])
    sh = Shield(pol, path_repair=path_repair)
    v = None
    if velocity is not None:
        w = from_body(velocity[0], velocity[1], 0.0, 0.0, h)
        v = (w.vx, w.vy)
    sh.set_subject(*_at(h, *at), "pedestrian", velocity=v)
    return sh, State(x=X0, y=Y0, up=15.0, yaw_deg=h)


def test_the_stand_off_repair_and_path_repair_s_detour_are_the_body_frame_ones():
    """A subject ahead and to the right, still and moving, outside the ring
    and inside it: StandoffHold, StandoffRecover and PathRepair's stand-off
    detour (path_repair on, the Shield's default) give the same emitted body
    action at every heading, to 1e-12 m/s. Counted, not blind."""
    ops = _same_repair_at_every_heading(lambda h: _standoff_scene(h), _bodies(5), ROT_HEADINGS)
    moving = _same_repair_at_every_heading(lambda h: _standoff_scene(h, velocity=(1.5, -0.5)),
                                           _bodies(5), ROT_HEADINGS)
    inside = _same_repair_at_every_heading(lambda h: _standoff_scene(h, at=(6.0, 1.0)),
                                           _bodies(5), ROT_HEADINGS)
    off = _same_repair_at_every_heading(lambda h: _standoff_scene(h, path_repair=False),
                                        _bodies(5), ROT_HEADINGS)
    assert ops.get("StandoffHold", 0) >= 10 and off.get("StandoffHold", 0) >= 10, (ops, off)
    assert ops.get("PathRepair", 0) + moving.get("PathRepair", 0) >= 5, (ops, moving)
    assert inside.get("StandoffRecover", 0) >= 30, inside


def test_the_overhead_recovery_backs_away_in_the_body_frame_at_any_heading():
    """Directly over the subject with no horizontal command there is no line
    to it; StandoffRecover leaves backwards from the aircraft's heading
    (shield.py _recover_from_overhead). Since wf21 the city follow and
    pymavlink SITL rails give the Shield the real heading, so this is
    'backwards' in the body frame there too (with heading 0 it was always
    South): the emitted body action is straight back, at every heading."""
    for body in (Action4D(), Action4D(vz_up=0.5, yaw_rate=0.2)):
        for h in ROT_HEADINGS:
            sh, st = _standoff_scene(h, at=(0.0, 0.0))
            d, em = sh.filter_body(st, body)
            assert [r.operator for r in d.repairs] == ["StandoffRecover"], (h, d.repairs)
            assert "backing away from the aircraft's heading" in d.repairs[0].detail
            assert em.vx < -0.4 and abs(em.vy) <= 1e-12, (h, em)
            assert (em.vz_up, em.yaw_rate) == (body.vz_up, body.yaw_rate)


def test_the_corridor_repair_is_the_body_frame_one():
    """A corridor along the nose, the aircraft 2.5 m right of its centerline:
    CorridorReturn (and the brakes it leaves) the same at every heading."""
    def scene(h):
        a, b = _at(h, -20.0, -2.5), _at(h, 60.0, -2.5)
        pol = Policy(policy_id="body-frame-corridor", version="0.0.1", constraints=[
            Corridor(id="cor", type="corridor", centerline=[XY(x=a[0], y=a[1]), XY(x=b[0], y=b[1])],
                     width_m=8.0, altitude_floor_m=0.0, altitude_ceiling_m=100.0, **HARD),
            _kin()])
        return Shield(pol), State(x=X0, y=Y0, up=15.0, yaw_deg=h)
    ops = _same_repair_at_every_heading(scene, _bodies(7), ROT_HEADINGS, tol=1e-11)
    assert ops.get("CorridorReturn", 0) >= 15, ops


def test_the_clearance_repair_on_a_raster_map_is_the_body_frame_one_at_the_quarter_turns():
    """ObstacleClearance reads a distance field on a world-aligned raster.
    Buildings placed in body-frame cells (a block ahead, a wall to the right,
    a corner the aircraft sits in) and the map turned with the aircraft give
    the same emitted body action and operators at 0, 90, 180 and -90 degrees,
    where turning a raster is exact (a cell to a cell); the corner case
    reaches the direction search (_best_clear_dir), whose bearings are the
    aircraft's own since 2026-11-18. A raster turned by any other angle is
    not the same map, so no other heading is compared."""
    import numpy as np
    from guardrail.models import ObstacleClearance
    n, res, c = 81, 2.0, 40                        # the aircraft on cell (40, 40) = (0, 0)

    def scene(cells):
        def make(h):
            occ = np.zeros((n, n), np.uint8)
            for f, r in cells:
                w = from_body(float(f), float(r), 0.0, 0.0, h)   # exact at a quarter turn
                occ[c + int(w.vx), c + int(w.vy)] = 1
            pol = Policy(policy_id="body-frame-clearance", version="0.0.1", constraints=[
                ObstacleClearance(id="clr", type="obstacle_clearance", min_clearance_m=6.0,
                                  soft_margin_m=2.0, **HARD),
                _kin()])
            return (Shield(pol, lookahead_s=3.0, dt=0.5,
                           obstacle_map={"occ": occ, "res": res, "ox": -80.0, "oy": -80.0}),
                    State(x=0.0, y=0.0, up=15.0, yaw_deg=h))
        return make

    searched = []
    real = Shield._best_clear_dir

    def spy(self, *a, **k):
        searched.append(1)
        return real(self, *a, **k)
    Shield._best_clear_dir = spy
    try:
        block = [(f, r) for f in range(4, 8) for r in range(-3, 4)]
        wall = [(f, r) for f in range(-20, 21) for r in range(3, 6)]
        corner = ([(f, r) for f in range(1, 6) for r in range(1, 6)]
                  + [(f, r) for f in range(-5, 6) for r in range(-6, -3)])
        tot = {}
        for cells in (block, wall, corner):
            for o, k in _same_repair_at_every_heading(scene(cells), _bodies(11),
                                                      QUARTER_TURNS).items():
                tot[o] = tot.get(o, 0) + k
    finally:
        Shield._best_clear_dir = real
    assert tot.get("ClearanceFix", 0) >= 60, tot
    assert len(searched) >= 8, len(searched)


def test_the_direction_search_is_the_same_body_frame_set_at_every_quarter_turn():
    """The recovery direction search (_clear_dir_candidates: the clearance
    fallback and the unconverged chain's rescue) tries 12 bearings every 30
    degrees from North at every heading (the integration of 2026-11-19 kept
    the North grid: a nose-relative grid moved stored follow decisions with
    no flight behind it). 30 divides 90, so at every quarter turn the
    candidates read in the body frame are the same SET as at heading 0; the
    bearing passed in is not used."""
    sh = Shield(_scene(0.0, 0.0, 0.0))
    a = Action4D(vx=1.0, vy=0.5, vz_up=0.3, yaw_rate=0.1)
    old = [(math.cos(k * math.pi / 6.0) * (4.0 * f), math.sin(k * math.pi / 6.0) * (4.0 * f))
           for k in range(12) for f in (1.0, 0.5)]
    for h in (0.0, 37.0, -121.5):
        assert [(c.vx, c.vy) for c in sh._clear_dir_candidates(a, 4.0, h)] == old, h

    def body_set(h):
        return sorted((round(b.vx, 9) + 0.0, round(b.vy, 9) + 0.0, b.vz_up, b.yaw_rate)
                      for b in (body_action(c, h) for c in sh._clear_dir_candidates(a, 4.0, h)))
    ref = body_set(0.0)
    assert len(ref) == 24
    for h in (90.0, 180.0, -90.0, 270.0):
        assert body_set(h) == ref, h


# --------------------------------------------------------------------------- #
# Producer-side smoothing: the demo loops' rate limiters
# --------------------------------------------------------------------------- #

def _world_limiter(dv_h: float, dv_z: float):
    """The cap every demo loop applied to its WORLD action before wf21."""
    prev = [Action4D()]

    def clip(new, old, m):
        return old + max(-m, min(m, new - old))

    def lim(a: Action4D) -> Action4D:
        p = prev[0]
        out = Action4D(vx=clip(a.vx, p.vx, dv_h), vy=clip(a.vy, p.vy, dv_h),
                       vz_up=clip(a.vz_up, p.vz_up, dv_z), yaw_rate=a.yaw_rate)
        prev[0] = out
        return out
    return lim


def test_the_demo_loops_rate_limiters_cap_the_world_reading_through_a_turn():
    """The demo loops hand the Shield a body action and smooth it first. A
    per-axis clip does not commute with a rotation, and a body-frame clip
    does not see a turn: a steady 3 m/s forward while the heading turns 90
    degrees swings the world velocity through 90 degrees. The limiters
    (demo/aerialvla_demo.py RateLimiter - also aerialvla_pas_demo,
    semantic_demo, semantic_seek -, demo/real_vla_demo.py RateLimiter,
    demo/run_demo.py RateLimiter.smooth_body) cap the change of the WORLD
    reading, the cap they applied before wf21, and hand back the body frame:
    tick for tick the world limiter's output, in the body frame. The turn is
    capped (the body action handed back differs from the one handed in),
    which a body-frame cap would not do."""
    sys.path.insert(0, str(ROOT / "demo"))
    if "airsim" not in sys.modules:                   # run_demo imports it at the top
        import types
        sys.modules["airsim"] = types.ModuleType("airsim")
    import aerialvla_demo
    import real_vla_demo
    import run_demo
    from guardrail.frames import limit_in_world
    makers = {
        "aerialvla_demo": (lambda: aerialvla_demo.RateLimiter(0.25, 0.15),
                           lambda lim, b, h: lim(b, h), 0.25, 0.15),
        "real_vla_demo": (lambda: real_vla_demo.RateLimiter(0.25, 0.15),
                          lambda lim, b, h: lim(b, h), 0.25, 0.15),
        "run_demo": (lambda: run_demo.RateLimiter(0.45, 0.25),
                     lambda lim, b, h: lim.smooth_body(b, h), 0.45, 0.25),
    }
    for name, (make, call, dv_h, dv_z) in makers.items():
        lim, ref = make(), _world_limiter(dv_h, dv_z)
        if name == "run_demo":
            call(lim, Action4D(), 0.0)                # its first call is passed through
        capped = 0
        # 30 ticks steady (up to speed), a 90-degree turn in 6 ticks, 30
        # steady: all off the quarter turns
        heads = [0.37] * 30 + [0.37 + 15.0 * i for i in range(1, 7)] + [90.37] * 30
        for i, h in enumerate(heads):
            body = Action4D(vx=3.0, vy=0.0, vz_up=0.8, yaw_rate=0.5)
            got = call(lim, body, h)
            want = body_action(ref(from_body(3.0, 0.0, 0.8, 0.5, h)), h)
            assert max(abs(got.vx - want.vx), abs(got.vy - want.vy),
                       abs(got.vz_up - want.vz_up)) <= 1e-12, (name, i, got, want)
            assert got.yaw_rate == body.yaw_rate
            if i == 29:
                assert abs(got.vx - 3.0) <= 1e-12 and abs(got.vy) <= 1e-12, (name, got)
            if i >= 30:
                capped += math.hypot(got.vx - body.vx, got.vy - body.vy) > 0.05
        assert capped >= 3, (name, capped)
        # once the turn is over and the cap has caught up, the steady body
        # action comes back as it went in (to the round trip's few ulp)
        assert abs(got.vx - 3.0) <= 1e-12 and abs(got.vy) <= 1e-12, (name, got)
    # the helper itself: limit_in_world(identity) is the round trip
    for h in ROT_HEADINGS:
        b = Action4D(vx=2.0, vy=-1.0, vz_up=0.3, yaw_rate=0.1)
        out = limit_in_world(lambda w: w, b, h)
        assert max(abs(out.vx - b.vx), abs(out.vy - b.vy)) <= 4 * 2.2e-16 * 2.0


def test_rebase_body_turns_a_body_action_by_the_heading_change():
    """frames.rebase_body: forward at a capture heading of 90 (East) is, seen
    from heading 0, to the right; in general the same world velocity, turned
    in the body frame (no world frame in between)."""
    from guardrail.frames import rebase_body
    got = rebase_body(Action4D(vx=1.5, vy=0.0, vz_up=0.2, yaw_rate=0.0), 90.0, 0.0)
    assert (got.vx, got.vy, got.vz_up) == (0.0, 1.5, 0.2)
    rng = random.Random(9)
    for _ in range(200):
        h0, h1 = rng.uniform(-360, 360), rng.uniform(-360, 360)
        b = Action4D(vx=rng.uniform(-5, 5), vy=rng.uniform(-5, 5))
        w = from_body(b.vx, b.vy, 0.0, 0.0, h0)
        want = body_action(w, h1)
        got = rebase_body(b, h0, h1)
        assert max(abs(got.vx - want.vx), abs(got.vy - want.vy)) <= 1e-13, (h0, h1, got, want)


# --------------------------------------------------------------------------- #
# One crossing: the source
# --------------------------------------------------------------------------- #

SCAN_EXCLUDE = ("tests/", "kuanting-vla-uav-guardrail/", ".claude/", "reference/",
                "node_modules/", ".git/")

# Modules allowed to reference the body -> world rotation itself, and where.
ROTATION_USERS = {
    "guardrail/frames.py": None,                         # the definitions
    # The Shield's projection: its internal world frame between from_body
    # and to_body (body in, body out at its interface).
    "guardrail/shield.py": {"filter_body", "advise_body", "check_body"},
    # A RECORD: the step log's world_action and its offline replay; the
    # flight hands the Shield the body action.
    "demo/real_vla_demo.py": {"openvla_raw_to_action"},
}

# Every rail's adapter: the modules that call body_to_local_ned (each must).
ADAPTERS = {
    "sitl/mavlink_adapter_node.py",       # the ROS 2 rail (and AP_DDS through it)
    "demo/pas_ardupilot/autopilot.py",    # PAS ArduPilot: PymavlinkLink, MavrosLink
    "sitl/run_sitl_demo.py",              # pymavlink SITL
    "experiments/sweep_scenarios.py",     # the headless harness
    "demo/follow_vlm.py",                 # the city follow rail
    "demo/vla_fly.py",
    "demo/aerialvla_demo.py",
    "demo/aerialvla_pas_demo.py",
    "demo/real_vla_demo.py",
    "demo/semantic_demo.py",
    "demo/semantic_seek.py",
    "demo/run_demo.py",
    "training/collect_aerialvla_data.py",   # the expert data collector (no Shield)
}

# Hand-written rotations of a velocity: every function, in every scanned
# module, whose velocity commands / Action4D arguments / vx-vy assignments use
# BOTH cos(E) and sin(E) of the same expression E - whatever E is called -
# directly or through a name bound to one (c, s = cos(t), sin(t)). Only these
# sites may, each with its reason; a new one fails here.
TRIG_SITES = {
    # Inside the projection's own world frame (body in / body out at its
    # interface): directions that turn with the aircraft.
    ("guardrail/shield.py", "_recover_from_overhead"):
        "the projection's internal world frame: backing away from the heading",
    ("guardrail/shield.py", "_clear_dir_candidates"):
        "the projection's internal world frame: 12 bearings from the nose",
    # A relative turn of an action by an angle: 2-D rotations commute, so it
    # is the same turn in the body frame and in the world frame.
    ("guardrail/prefix_consumer.py", "_turned"): "a relative turn of the action",
    # World-frame PRODUCERS upstream of the Shield's input contract: their
    # controller composes its command in the world frame, and the rail hands
    # the Shield its body-frame reading (frames.to_body / filter_world_as_body).
    ("guardrail/vla_link.py", "_rate_tick"):
        "the step tracker (a world-frame controller): a rate step's (forward, right) "
        "speed at the heading, in the world frame",
    ("demo/follow_vlm.py", "fly"):
        "the follow controller's orbit and look terms, composed in the world frame",
    ("demo/pas_ardupilot/perception_node.py", "step"):
        "FollowPilot's world record of its command (it hands over the body action)",
    # The expert data collector (no Shield): its bearing-error steering
    # makes a body (forward) speed; the command goes through body_to_local_ned.
    ("training/collect_aerialvla_data.py", "main"): "the expert's bearing-error steering",
    # The headless harness's simulated world: the wind disturbance on the
    # vehicle and the scenario's rotation, not a command crossing.
    ("experiments/sweep_scenarios.py", "_episode"):
        "the harness's wind model and scenario rotation (the simulated world)",
    # Offline reading of logs, never sent.
    ("tools/replay_from_bag.py", "world_from_body"): "offline bag replay record",
    ("experiments/analyze_semantic_ab.py", "_world_xy"): "offline analysis of logged rows",
}

VEL_SINKS = {"move_by_velocity_async", "moveByVelocityAsync", "moveByVelocityZAsync",
             "set_position_target_local_ned_send", "send_velocity"}
# A velocity's name: vx, vy, vn, ve, v_n, v_e, with an optional one-letter
# prefix (cvx, gvy, ...).
VEL_NAME = re.compile(r"^[a-z]?v_?(x|y|n|e)$")


def _modules():
    for p in sorted(ROOT.rglob("*.py")):
        rel = p.relative_to(ROOT).as_posix()
        if rel.startswith(SCAN_EXCLUDE) or "/build/" in rel or "/install/" in rel:
            continue
        try:
            yield rel, ast.parse(p.read_text(encoding="utf-8"))
        except (SyntaxError, UnicodeDecodeError):
            continue


def _enclosing(tree):
    """id(node) -> name of the innermost function holding it ('' at module)."""
    out = {}

    def visit(node, fn):
        for ch in ast.iter_child_nodes(node):
            name = ch.name if isinstance(ch, (ast.FunctionDef, ast.AsyncFunctionDef)) else fn
            out[id(ch)] = name
            visit(ch, name)
    visit(tree, "")
    return out


def _name(node) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ""


def _own_nodes(fn):
    """The nodes of a function body, not those of a function nested in it."""
    stack = list(ast.iter_child_nodes(fn))
    while stack:
        n = stack.pop()
        yield n
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            continue
        stack.extend(ast.iter_child_nodes(n))


def _trig_atoms(expr, bound) -> set:
    """("cos" | "sin", source of the angle) for each cos / sin call in
    `expr`, and those of each name in it that `bound` maps."""
    out = set()
    for n in ast.walk(expr):
        if isinstance(n, ast.Name) and n.id in bound:
            out |= bound[n.id]
        if isinstance(n, ast.Call) and _name(n.func) in ("cos", "sin") and n.args:
            out.add((_name(n.func), ast.unparse(n.args[0])))
    return out


def _rotation_sites(modules) -> dict:
    """(module, function) -> sorted line numbers of the velocity sinks that use
    cos(E) and sin(E) of the same E (see TRIG_SITES)."""
    found = {}
    for rel, tree in modules:
        if rel == "guardrail/frames.py":
            continue
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            nodes = list(_own_nodes(fn))
            bound: dict = {}
            changed = True
            while changed:                        # names bound to a cos / sin, to a fixpoint
                changed = False
                for n in nodes:
                    if not isinstance(n, (ast.Assign, ast.AnnAssign, ast.AugAssign)) \
                            or n.value is None:
                        continue
                    pairs = []
                    for t in (n.targets if isinstance(n, ast.Assign) else [n.target]):
                        if isinstance(t, ast.Tuple) and isinstance(n.value, ast.Tuple) \
                                and len(t.elts) == len(n.value.elts):
                            pairs += list(zip(t.elts, n.value.elts))
                        elif isinstance(t, ast.Tuple):
                            pairs += [(e, n.value) for e in t.elts]
                        else:
                            pairs.append((t, n.value))
                    for t, v in pairs:
                        a = _trig_atoms(v, bound)
                        if isinstance(t, ast.Name) and a and not a <= bound.get(t.id, set()):
                            bound[t.id] = bound.get(t.id, set()) | a
                            changed = True
            sinks = []
            for n in nodes:
                exprs = []
                if isinstance(n, ast.Call) and (_name(n.func) == "Action4D"
                                                or _name(n.func) in VEL_SINKS):
                    exprs = list(n.args) + [k.value for k in n.keywords
                                            if k.arg in (None, "vx", "vy")]
                elif isinstance(n, (ast.Assign, ast.AugAssign)):
                    targets = n.targets if isinstance(n, ast.Assign) else [n.target]
                    tnames = {_name(t) for t in targets}
                    tnames |= {_name(e) for t in targets if isinstance(t, ast.Tuple)
                               for e in t.elts}
                    if any(VEL_NAME.match(t or "") for t in tnames):
                        exprs = [n.value]
                elif isinstance(n, ast.Dict):
                    exprs = [v for k, v in zip(n.keys, n.values)
                             if isinstance(k, ast.Constant) and k.value in ("vx", "vy")]
                atoms = set()
                for e in exprs:
                    atoms |= _trig_atoms(e, bound)
                if atoms:
                    sinks.append((n.lineno, atoms))
            every = set().union(*(a for _, a in sinks)) if sinks else set()
            turned = {e for f, e in every if f == "cos" and ("sin", e) in every}
            if turned:
                found[(rel, fn.name)] = sorted(
                    {ln for ln, a in sinks
                     if any((f, e) in a for e in turned for f in ("cos", "sin"))})
    return found


def test_from_body_and_to_local_ned_are_used_only_by_the_projection_and_one_record():
    """No body -> world (or world -> NED) rotation outside guardrail/frames.py,
    the Shield's projection and the one named record helper. A rail that
    rotates the action itself again fails here."""
    bad, seen = [], set()
    for rel, tree in _modules():
        fn_of = _enclosing(tree)
        for node in ast.walk(tree):
            names = []
            if isinstance(node, (ast.Name, ast.Attribute)):
                names = [_name(node)]
            elif isinstance(node, ast.ImportFrom):
                names = [a.name for a in node.names]
            for nm in names:
                if nm not in ("from_body", "to_local_ned", "_from_body", "body_to_world"):
                    continue
                allowed = ROTATION_USERS.get(rel, set())
                fn = fn_of.get(id(node), "")
                if allowed is None or isinstance(node, ast.ImportFrom) and rel in ROTATION_USERS \
                        or (allowed and fn in allowed):
                    seen.add(rel)
                    continue
                bad.append(f"{rel}:{node.lineno} {nm} in {fn or '<module>'}")
    assert not bad, "body -> world / NED rotations outside the one boundary: " + "; ".join(bad)
    assert seen == set(ROTATION_USERS), seen


def test_every_rail_s_adapter_calls_the_one_body_to_ned_function_and_nothing_else_does():
    calls = {}
    for rel, tree in _modules():
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and _name(node.func) == "body_to_local_ned":
                calls.setdefault(rel, []).append(node.lineno)
    calls.pop("guardrail/frames.py", None)
    assert set(calls) == ADAPTERS, (
        f"body_to_local_ned called by {sorted(set(calls) - ADAPTERS)} (a new crossing: "
        f"name it in ADAPTERS and in frames.py) / not by {sorted(ADAPTERS - set(calls))}")


def test_no_velocity_is_built_from_a_hand_written_rotation_of_the_heading():
    """A velocity command, an Action4D, or a vx / vy assignment built from
    cos(E) and sin(E) of the same angle E - whatever the angle is called -
    is a rotation written by hand; only the listed TRIG_SITES may."""
    found = _rotation_sites(_modules())
    bad = [f"{rel}:{lines} in {fn}" for (rel, fn), lines in sorted(found.items())
           if (rel, fn) not in TRIG_SITES]
    assert not bad, "hand-written rotations into a velocity: " + "; ".join(bad)
    stale = set(TRIG_SITES) - set(found)
    assert not stale, f"stale TRIG_SITES entries: {stale}"


LOOPS = sorted(ADAPTERS - {"sitl/mavlink_adapter_node.py", "demo/pas_ardupilot/autopilot.py"}
               | {"demo/vla_mission.py", "demo/pas_ardupilot/perception_node.py"})

# Every call of the Shield's WORLD-frame entries - `.filter(state, action)`,
# `.advise(state, action)`, `._check(state, action)`: an attribute call of
# that name with two or more positional arguments, on ANY receiver - outside
# the Shield itself, each with its reason. None of them is on a command path:
# the harness's judging Shields re-check what flew, the rest are offline
# (replays, KPI and dataset tools, the compiler's cost-map agreement). The
# key is (module, function, receiver as written); a new caller anywhere, or a
# new receiver at a listed site, fails until it is named here.
WORLD_ENTRY_CALLERS = {
    ("experiments/sweep_scenarios.py", "_episode", "checker"):
        "the harness's judge: re-checks what flew",
    ("experiments/sweep_scenarios.py", "_episode", "oracle"):
        "the harness's oracle: the hindsight verdict",
    ("experiments/sweep_scenarios.py", "_episode", "vjudge"):
        "the harness's vehicle judge: the vehicle's velocity, re-checked",
    ("experiments/verify_shield_yaw.py", "replay", "shield"): "offline replay of a log",
    ("guardrail/compiler.py", "cost_map_agreement", "sh"): "offline cost-map agreement",
    ("guardrail/repair.py", "repair", "sh"):
        "PathRepair's re-check, inside the projection's internal world frame",
    ("tools/build_deck_data.py", "*", "sh"): "offline deck figures",
    ("tools/failsafe_rubric.py", "*", "self"): "offline rubric (a Shield subclass)",
    ("tools/msl_dem_kpi.py", "*", "sh"): "offline KPI tool",
    ("tools/own_velocity_closed_loop.py", "*", "sh"): "offline closed loop (no adapter)",
    ("tools/prefix_eval.py", "*", "sh"): "offline replay of a recorded run",
    ("tools/profile_shield_tick.py", "*", "shield"): "offline timing",
    ("tools/replay_q3.py", "*", "shield"): "offline open-loop replay",
    ("tools/replay_at_revision.py", "*", "shield"):
        "offline open-loop replay of a flight log at its flown revision",
    ("tools/replay_standoff_feed.py", "*", "sh"): "offline replay of logged flights",
    ("tools/wgs84_bind_compare.py", "*", "sh"): "offline comparison",
    ("tools/wp1_roundtrip_kpi.py", "*", "shield"): "offline KPI tool",
    ("training/evaluate_policy.py", "*", "shield"): "offline evaluator (pos += v dt)",
    ("training/generate_dataset.py", "*", "shield"): "offline dataset rollouts",
    ("training/generate_v2.py", "*", "shield"): "offline dataset rollouts",
}


def _world_entry_calls(modules):
    """[(module, function, receiver, line)] of every world-frame Shield entry
    call outside guardrail/shield.py (see WORLD_ENTRY_CALLERS)."""
    out = []
    for rel, tree in modules:
        if rel == "guardrail/shield.py":
            continue
        fn_of = _enclosing(tree)
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr in ("filter", "advise", "_check")
                    and len(node.args) >= 2):
                out.append((rel, fn_of.get(id(node), ""), ast.unparse(node.func.value),
                            node.lineno))
    return out


def _unlisted(calls):
    return [c for c in calls if (c[0], c[1], c[2]) not in WORLD_ENTRY_CALLERS
            and (c[0], "*", c[2]) not in WORLD_ENTRY_CALLERS]


def test_no_rail_or_demo_loop_hands_its_shield_a_world_frame_action():
    """Every rail and demo loop calls the Shield with the body frame
    (filter_body / filter_world_as_body / advise_body / check_body). Every
    call of a world-frame entry anywhere outside the Shield, on any receiver,
    is a listed judge or offline tool (WORLD_ENTRY_CALLERS); none is in a
    module that sends a velocity command or calls the adapter function,
    except the harness's judges."""
    calls = _world_entry_calls(_modules())
    bad = _unlisted(calls)
    assert not bad, "world-frame Shield calls not named in WORLD_ENTRY_CALLERS: " + "; ".join(
        f"{m}:{ln} {r}.<entry> in {f or '<module>'}" for m, f, r, ln in bad)
    keys = {(m, f, r) for m, f, r, _ in calls} | {(m, "*", r) for m, _, r, _ in calls}
    stale = set(WORLD_ENTRY_CALLERS) - keys
    assert not stale, f"stale WORLD_ENTRY_CALLERS entries: {stale}"
    flying = set(LOOPS) | ADAPTERS
    for rel, tree in _modules():
        if any(isinstance(n, ast.Call) and _name(n.func) in VEL_SINKS for n in ast.walk(tree)):
            flying.add(rel)
    on_flying = {m for m, _, _, _ in calls if m in flying}
    assert on_flying <= {"experiments/sweep_scenarios.py"}, on_flying
    n_body = 0
    for rel in LOOPS:
        src = (ROOT / rel).read_text(encoding="utf-8")
        n_body += len(re.findall(r"\.(filter_body|advise_body|check_body)\(", src))
        n_body += src.count("filter_world_as_body(")
    assert n_body >= len(LOOPS), n_body


def test_the_crossing_detectors_catch_a_planted_crossing():
    """The detectors above, mutation-tested: a synthetic new rail module
    with (a) a hand rotation through an angle named nothing like a heading,
    (b) the same through names bound to cos / sin, (c) a world-frame filter
    call on a receiver not named "shield", is caught by each detector; and
    the real modules plus a harmless scalar cos (no matching sin) are not."""
    planted = {
        "demo/new_rail_a.py": (
            "import math\n"
            "def loop(drone, fwd, th):\n"
            "    vx = fwd * math.cos(th); vy = fwd * math.sin(th)\n"
            "    drone.move_by_velocity_async(vx, vy, 0)\n"),
        "demo/new_rail_b.py": (
            "import math\n"
            "def loop(link, f, r, ang):\n"
            "    c, s = math.cos(ang), math.sin(ang)\n"
            "    a, b = f * c - r * s, f * s + r * c\n"
            "    link.send_velocity(a, b, 0.0)\n"),
        "demo/new_rail_c.py": (
            "def loop(guard, st, a):\n"
            "    d = guard.filter(st, a)\n"
            "    return d.emitted\n"),
        "demo/harmless.py": (
            "import math\n"
            "from guardrail.models import Action4D\n"
            "def scale(a, err):\n"
            "    k = max(0.25, math.cos(err))\n"
            "    return Action4D(vx=a.vx * k, vy=a.vy * k)\n"),
    }
    mods = [(rel, ast.parse(src)) for rel, src in planted.items()]
    rot = _rotation_sites(mods)
    assert set(rot) == {("demo/new_rail_a.py", "loop"), ("demo/new_rail_b.py", "loop")}, rot
    assert _unlisted(_world_entry_calls(mods)) == [("demo/new_rail_c.py", "loop", "guard", 2)]
    real = _rotation_sites(_modules())
    assert set(real) == set(TRIG_SITES), set(real) ^ set(TRIG_SITES)


# --------------------------------------------------------------------------- #
# The real heading, flown on the stand-ins
# --------------------------------------------------------------------------- #

def test_the_pymavlink_rail_hands_the_shield_the_body_action_at_the_autopilot_heading():
    """sitl/run_sitl_demo.py against the stand-in autopilot of
    tests/test_sitl_rails.py, reporting heading 37 degrees: every
    filter_body call is at 37 degrees with the stub's world action turned
    into the body frame there (not relabelled), every setpoint the link
    sends is body_to_local_ned of the Shield's emitted body action at 37,
    and the flight is the heading-0 flight (the stub steers in the world)."""
    try:
        import test_sitl_rails as TS
    except Exception as e:                                   # noqa: BLE001
        print(f"      (test_sitl_rails not importable: {e})")
        return SKIP
    seen, sent = [], []

    def at(heading):
        def patch(R):
            real_fwb = R.filter_world_as_body

            def spy(shield, state, world, t=None, **kw):
                d, b, eb = real_fwb(shield, state, world, t, **kw)
                seen.append((heading, state.yaw_deg, world, b, eb))
                return d, b, eb
            R.filter_world_as_body = spy
            real_link = R.MavlinkAdapter

            class Link(real_link):
                def send_body(self, body, yaw_deg):
                    ned = super().send_body(body, yaw_deg)
                    sent.append((heading, body, yaw_deg, ned))
                    return ned
            R.MavlinkAdapter = Link
        return patch

    finals = {}
    for heading in (0.0, 37.0):
        sim_box = {}
        orig = TS._Sim.__init__

        def init(self, up=0.0, yaw_deg=0.0, _h=heading):
            orig(self, up, _h)
            sim_box["sim"] = self
        TS._Sim.__init__ = init
        try:
            out, metrics, rows, events, sim, R = TS._fly_pymavlink_rail(["--shield", "on"],
                                                                          patch=at(heading))
        finally:
            TS._Sim.__init__ = orig
        finals[heading] = (round(sim.x, 6), round(sim.y, 6), round(sim.up, 6))
        assert metrics["action_frame"]["heading_missing_ticks"] == 0
    s37 = [s for s in seen if s[0] == 37.0]
    assert s37 and all(s[1] == 37.0 for s in s37), "the Shield was not given the heading"
    moving = [s for s in s37 if math.hypot(s[2].vx, s[2].vy) > 0.1]
    assert moving and all(s[3] == body_action(s[2], 37.0) and
                          abs(s[3].vx - s[2].vx) > 1e-3 for s in moving), \
        "the body action handed over is the world action relabelled"
    p37 = [p for p in sent if p[0] == 37.0]
    assert p37 and all(p[2] == 37.0 and p[3] == body_to_local_ned(p[1], 37.0) for p in p37)
    assert max(abs(a - b) for a, b in zip(finals[0.0], finals[37.0])) < 1e-6, finals


def test_the_headless_harness_hands_over_the_body_frame_at_the_episode_heading():
    """experiments/sweep_scenarios.py, closed loop: a library cell turned to
    37 degrees hands the Shield the body action at 37 on every tick, and the
    integrator flies body_to_local_ned of what the Shield streams."""
    sys.path.insert(0, str(ROOT / "experiments"))
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library
    lib = Library(ROOT / "experiments" / "scenarios.yaml")
    spec = lib.by_id()["nfz-head-on"].first.spec.model_copy(deep=True)
    # The heading flown is the start's plus the mission's rotation.
    spec.mission.start_pose.yaw_deg = 37.0 - spec.mission.rotate_deg
    seen, flown = [], []
    real, real_ned = S.filter_world_as_body, S.body_to_local_ned

    def spy(shield, state, world, t=None, **kw):
        out = real(shield, state, world, t, **kw)
        seen.append((state.yaw_deg, world, out[1], out[2]))
        return out

    def ned_spy(body, yaw_deg):
        flown.append((body, yaw_deg))
        return real_ned(body, yaw_deg)
    S.filter_world_as_body, S.body_to_local_ned = spy, ned_spy
    try:
        run = S.run_episode(spec, lib.defaults, 1001)
    finally:
        S.filter_world_as_body, S.body_to_local_ned = real, real_ned
    assert len(seen) == len(run.rows) > 0
    assert all(abs(s[0] - 37.0) < 1e-9 for s in seen), {s[0] for s in seen}
    assert all(s[2] == body_action(s[1], s[0]) for s in seen)
    moving = [s for s in seen if math.hypot(s[1].vx, s[1].vy) > 0.1]
    assert moving and all(abs(s[2].vx - s[1].vx) > 1e-3 for s in moving), \
        "the body action handed over is the world action relabelled"
    n_rep = sum(1 for r in run.rows if r.get("repairs"))
    assert n_rep > 10, n_rep
    # the adapter: every shield-flown tick streams the Shield's emitted body action
    em = {id(s[3]) for s in seen}
    assert flown and all(abs(h - 37.0) < 1e-9 for _, h in flown)
    assert sum(1 for b, _ in flown if id(b) in em) >= len(seen) - 5, (len(flown), len(seen))


def test_the_pas_rail_flies_the_shield_s_body_action_at_the_aircraft_heading():
    """demo/pas_ardupilot/perception_node.py on test_pas_ardupilot's fake
    flight at 0.6 rad: the Shield is handed the pilot's body action at the
    aircraft's heading on every tick, and the link sends body_to_local_ned
    of what the rail streams at the heading it read."""
    try:
        import test_pas_ardupilot as T
    except Exception as e:                                   # noqa: BLE001
        print(f"      (test_pas_ardupilot not importable: {e})")
        return SKIP
    calls = []

    class Link(T.FakeLink):
        def send(self, a=None, raw_body=None, body=None):
            super().send(a, raw_body, body=body)
            calls.append((body, math.degrees(self.snap.yaw), self.v))

    rc, run, link = T._fly("on", link_cls=Link, policy=T.CAP_POLICY, link_kw={"yaw": 0.6})
    assert rc == 0 and calls
    rows = [r for r in T._rows(run) if r["flown"]]
    assert len(calls) == len(rows)
    for (body, h, v), r in zip(calls, rows):
        assert abs(h - math.degrees(0.6)) < 1e-9 and abs(r["yaw_deg"] - h) < 0.01
        assert body is not None
        assert v == local_ned_to_world(body_to_local_ned(body, h))


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
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
