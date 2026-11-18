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
    of the scene as the VLA sees it, not of the world's axes.
  * ONE CROSSING. Parsed from the source of every module outside tests/ and
    the reference checkout: `from_body` / `to_local_ned` are referenced only
    by guardrail/frames.py, by the Shield's projection (filter_body,
    advise_body, check_body) and by one named record helper;
    `body_to_local_ned` - the adapter function - is called only by the
    listed adapters, and each of them calls it; no velocity command or
    Action4D is built from a hand-written cos / sin of a heading except at
    the listed producer and record sites; no rail or demo loop calls
    `Shield.filter` on the action it flies. A new crossing fails here.
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
                              KinematicEnvelope, Policy, PolygonFence,
                              State, XY)
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

# Hand-written cos / sin of a heading inside a velocity: the sites that are
# NOT a body -> NED crossing of a flown action, each with its reason.
TRIG_SITES = {
    # Inside the projection's own world frame (body in / body out at its
    # interface): the direction that backs away from the heading.
    ("guardrail/shield.py", "_recover_from_overhead"):
        "the projection's internal world frame",
    # World-frame PRODUCERS upstream of the Shield's input contract: their
    # controller composes its command in the world frame, and the rail hands
    # the Shield its body-frame reading (frames.to_body / filter_world_as_body).
    ("guardrail/vla_link.py", "_rate_tick"):
        "the step tracker: a step's speed along the heading, in the world frame",
    ("demo/follow_vlm.py", "fly"):
        "the follow controller's orbit term, composed in the world frame",
    ("demo/pas_ardupilot/perception_node.py", "step"):
        "FollowPilot's world record of its command (it hands over the body action)",
    # Offline reading of logs, never sent.
    ("tools/replay_from_bag.py", "world_from_body"): "offline bag replay record",
    ("experiments/analyze_semantic_ab.py", "_world_xy"): "offline analysis of logged rows",
}

VEL_SINKS = {"move_by_velocity_async", "moveByVelocityAsync", "moveByVelocityZAsync",
             "set_position_target_local_ned_send", "send_velocity"}
HEADINGISH = ("yaw", "psi", "heading", "hdg")
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
    """A velocity command or an Action4D whose arguments - or a vx / vy
    assignment whose value - contain cos / sin of a heading is a crossing
    written by hand; only the listed producer and record sites may."""
    bad, used = [], set()

    def trig_of_heading(expr, trig_names=frozenset()) -> bool:
        for n in ast.walk(expr):
            if isinstance(n, ast.Name) and n.id in trig_names:
                return True              # a cos / sin of the heading computed above
            if isinstance(n, ast.Call) and _name(n.func) in ("cos", "sin"):
                inner = {(_name(x) or "").lower() for x in ast.walk(n)}
                if any(h in s for s in inner for h in HEADINGISH):
                    return True
        return False

    for rel, tree in _modules():
        if rel == "guardrail/frames.py":
            continue
        fn_of = _enclosing(tree)
        # Per function: names bound to a cos / sin of the heading (c, s =
        # math.cos(psi), math.sin(psi)), so a rotation written through them
        # is seen too.
        trig = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign) and trig_of_heading(node.value):
                for t in node.targets:
                    for e in (t.elts if isinstance(t, ast.Tuple) else [t]):
                        if isinstance(e, ast.Name):
                            trig.setdefault(fn_of.get(id(node), ""), set()).add(e.id)
        for node in ast.walk(tree):
            exprs = []
            if isinstance(node, ast.Call) and (_name(node.func) == "Action4D"
                                               or _name(node.func) in VEL_SINKS):
                exprs = list(node.args) + [k.value for k in node.keywords
                                           if k.arg in (None, "vx", "vy")]
            elif isinstance(node, (ast.Assign, ast.AugAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                tnames = {_name(t) for t in targets}
                tnames |= {_name(e) for t in targets if isinstance(t, ast.Tuple)
                           for e in t.elts}
                if any(VEL_NAME.match(t or "") for t in tnames):
                    exprs = [node.value]
            elif isinstance(node, ast.Dict):
                exprs = [v for k, v in zip(node.keys, node.values)
                         if isinstance(k, ast.Constant) and k.value in ("vx", "vy")]
            fn = fn_of.get(id(node), "")
            if not any(trig_of_heading(e, frozenset(trig.get(fn, ()))) for e in exprs):
                continue
            key = (rel, fn)
            if key in TRIG_SITES:
                used.add(key)
                continue
            bad.append(f"{rel}:{node.lineno} in {fn or '<module>'}")
    assert not bad, "hand-written heading rotations into a velocity: " + "; ".join(bad)
    assert used == set(TRIG_SITES), f"stale TRIG_SITES entries: {set(TRIG_SITES) - used}"


LOOPS = sorted(ADAPTERS - {"sitl/mavlink_adapter_node.py", "demo/pas_ardupilot/autopilot.py"}
               | {"demo/vla_mission.py", "demo/pas_ardupilot/perception_node.py"})


def test_no_rail_or_demo_loop_hands_its_shield_a_world_frame_action():
    """Every rail and demo loop calls the Shield with the body frame
    (filter_body / filter_world_as_body / advise_body / check_body); a
    `<...shield...>.filter(` call there - the world-frame entry - fails. The
    harness's judging Shields (checker, oracle, vjudge) re-check what flew
    and are not the flying Shield."""
    bad, n_body = [], 0
    for rel in LOOPS:
        tree = ast.parse((ROOT / rel).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)):
                continue
            recv = ast.unparse(node.func.value).lower()
            if node.func.attr in ("filter_body", "advise_body", "check_body"):
                n_body += 1
            if node.func.attr in ("filter", "advise", "_check") and "shield" in recv:
                bad.append(f"{rel}:{node.lineno} {ast.unparse(node.func)}(")
        src = (ROOT / rel).read_text(encoding="utf-8")
        n_body += src.count("filter_world_as_body(")
    assert not bad, "world-frame Shield calls on a flying loop: " + "; ".join(bad)
    assert n_body >= len(LOOPS), n_body


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
