"""
Hierarchical control of the red-car follow (demo/vla_hierarchy.py, follow_vlm
--vla-hierarchy HOST:PORT, scripts/run_citylife_follow.ps1 -VlaHierarchy):
AerialVLA decides the intent at its own rate, the hand-written controller
executes it at 10 Hz.

Run either way:
    pytest tests/test_vla_hierarchy.py -v
    python tests/test_vla_hierarchy.py

WHAT IS AT STAKE (2026-10-25)

End-to-end AerialVLA reached at best 0.238 within 30 m against the
hand-written follow's 0.963, and the ceiling flights showed the decision rate
is the bottleneck. The user chose a hierarchy: the VLA sets the intent, the
controller flies it. So, pinned here:

  1. OFF BY DEFAULT, and the flight unchanged when off: nothing imported,
     built or called without the flag; one plan() before the controller's
     branches, one apply() at the perturbers' call site before the one
     Shield call; the scene with AerialVLA's cameras only with the flag.
  2. THE INTENT: a step's fwd_m maps to a stand-off of 22 m (fwd 0) to 12 m
     (fwd 5), clamped, moved at most 2 m/s; LAND holds; an answer 1.5 s old
     expires and the controller's own behaviour flies, recorded.
  3. FOLLOW: locked, the controller flies (its own command object) with
     want_range = the VLA's stand-off, the one parameter every arm of its
     controller reads; the VLA's dpsi steers nothing.
  4. SEARCH: unlocked with a fresh answer, the executed command is what an
     independent StepTracker with vla_fly's follow config makes of the same
     step, tick by tick; without a fresh answer (silent, refusing,
     malformed, timed out, expired) the controller's planner flies; the
     hand-over ramps; back to FOLLOW when the lock returns.
  5. THE RECORDS: every tick's row, the summary (authority shares, VLA-flown
     share, intents, s distribution, answers, latency, server identity and
     adapter), vla_steps.jsonl, the manifest and pilot record naming the VLA.
  6. REFUSALS before the simulator: with -RecordPerturb / -RecordPerturbVla /
     -DecideEveryS / -RecordVla, a sub-flag alone, a bad address or range, a
     server that is not AerialVLA, an adapter's tile or speed words.
  7. THE LAUNCHER passes -VlaHierarchy and its flags and refuses the
     mutually exclusive modes before the simulator starts.

No simulator, GPU or model: tests/test_vla_dagger.py's stub client on a fake
clock, and tests/vla_fake_server.py on an ephemeral loopback port (never
8790) with the real protocol-v2 client and threads.
"""
from __future__ import annotations

import ast
import contextlib
import io
import json
import math
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "tools", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail.models import Action4D                               # noqa: E402
from guardrail.vla_link import (Capture, Decision, Pose, Step,       # noqa: E402
                                StepTracker)
import vla_fake_server as F                                          # noqa: E402

SKIP = "SKIP"
FOLLOW = ROOT / "demo" / "follow_vlm.py"
PS1 = ROOT / "scripts" / "run_citylife_follow.ps1"
JPG = b"\xff\xd8\xff\xe0fake-jpeg\xff\xd9"
T0 = 1_000_000.0                 # a wall-clock origin far from 0, as a flight's
EXPERT = Action4D(vx=4.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
DEFAULT_S = 15.83                # the controller's own: --want-width 0.16 on a 4 m car
NO_PORT = "127.0.0.1:59999"      # parsed only, never connected to


def _H():
    import vla_hierarchy as H
    return H


def _TD():
    import test_vla_dagger as TD
    return TD


def _D():
    import vla_dagger as D
    return D


class _Rig:
    """A deterministic hierarchical run: a fake clock, test_vla_dagger's stub
    client, a point vehicle flying what was executed, a fresh box ahead
    (`box`). One tick: the test may answer the pending request, the thread's
    round runs, the loop's plan() and apply() (the controller's command is
    EXPERT, its stand-off the one plan() returned), the thread's round again."""

    def __init__(self, follow=None, **kw):
        H, TD, D = _H(), _TD(), _D()
        self.t, self.s = T0, 0.0
        self.client = TD._Stub(lambda: self.t, kw.pop("identity", None))
        self.veh = TD._Vehicle()
        self.box = True
        vargs, _argv = D.follow_args("a red car", cruise_alt=8.0, **(follow or {}))
        self.cfg = D.follow_tracker_config(vargs)
        self.h = H.VLAHierarchy(client=self.client, get_capture=self.capture,
                                latest_box=D.latest_box_from(self.grounder),
                                object_text="a red car", tracker_config=self.cfg,
                                clock=lambda: self.t, **D.follow_hint_config(vargs), **kw)
        self.h.start(T0, thread=False)

    def capture(self):
        return Capture(self.t - 0.05, Pose(*self.veh.pose()), JPG, JPG)

    def grounder(self):
        if not self.box:
            return None
        return {"det": _TD()._det_at_bearing(0.0), "t_det": self.t - 0.02, "seq": 1,
                "tier": "ok", "cap": {"t_capture": self.t - 0.2, "pose": {"yaw": self.veh.yaw}}}

    def run(self, seconds, answer=None, locked=True, expert=EXPERT, default=DEFAULT_S, dt=0.1):
        """`answer(msg, waited_s) -> None | reply dict | "timeout"`; `locked`
        a bool or a function of the mission second."""
        rows = []
        for _ in range(int(round(seconds / dt))):
            self.t = T0 + self.s
            c = self.client
            if c._pending is not None and answer is not None:
                got = answer(c._pending, self.t - c.t_sent)
                if got is not None:
                    c.answer(self.t, got)
            self.h._service_once()
            lk = bool(locked(self.s) if callable(locked) else locked)
            pose = self.veh.pose()
            want = self.h.plan(self.s, locked=lk, pose=pose, default_m=default)
            out, rec = self.h.apply(expert, self.s, pose=pose)
            self.h._service_once()
            json.dumps(rec)                       # every record is plain JSON
            rows.append({"s": self.s, "t": self.t, "want": want, "out": out, "rec": rec,
                         "pose": pose, "locked": lk})
            self.veh.step(out, dt)
            self.s = round(self.s + dt, 10)
        return rows


def _step(fwd, down, dpsi, after=0.5):
    return lambda msg, waited: (F.step_reply(msg, fwd, down, dpsi)
                                if waited >= after - 1e-9 else None)


def _tree():
    src = FOLLOW.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fly = next(n for n in ast.walk(tree) if isinstance(n, ast.AsyncFunctionDef) and n.name == "fly")
    parents = {c: n for n in ast.walk(tree) for c in ast.iter_child_nodes(n)}

    def guarded(node, test):
        p = node
        while p in parents:
            p = parents[p]
            if isinstance(p, ast.If) and test(ast.unparse(p.test)):
                return True
        return False
    return src, tree, fly, guarded


def _calls(fly, obj: str, attr: str) -> list:
    return [n for n in ast.walk(fly) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == attr
            and isinstance(n.func.value, ast.Name) and n.func.value.id == obj]


def _refused(argv: list[str]) -> str:
    """follow_vlm.main(argv) must exit 2 before flying; its stderr. fly() is a
    stub that fails the test, so an accepted argv never starts a flight."""
    import follow_vlm as FV

    async def _no_flight(args):
        raise AssertionError(f"{argv} was accepted and would have flown")

    real, FV.fly = FV.fly, _no_flight
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            FV.main(argv)
    except SystemExit as exc:
        assert exc.code == 2, (argv, exc.code, err.getvalue()[-300:])
    else:
        raise AssertionError(f"{argv} was accepted")
    finally:
        FV.fly = real
    return err.getvalue()


# =========================================================================== #
# 1. Off by default; the flight unchanged when off
# =========================================================================== #
def test_off_by_default_and_the_flight_unchanged_when_off():
    import follow_vlm as FV
    H = _H()
    a = FV.build_arg_parser().parse_args([])
    assert a.vla_hierarchy is None and a.vla_hierarchy_allow_stand_in is False
    assert a.vla_hierarchy_range == H.S_RANGE_M == (12.0, 22.0)
    assert (a.vla_hierarchy_bottom_tile, a.vla_hierarchy_prev_dt_s,
            a.vla_hierarchy_step_as_rate, a.vla_hierarchy_speed_words) == ("down", None, None,
                                                                          False)
    # The scene with AerialVLA's cameras only with the flag (or --record-vla).
    assert FV.flight_scene(a) == FV.SCENE == "scene_semantic.jsonc"
    b = FV.build_arg_parser().parse_args(["--vla-hierarchy", NO_PORT])
    assert b.vla_hierarchy == NO_PORT and FV.flight_scene(b) == FV.SCENE_VLA_RECORD
    src, tree, fly, guarded = _tree()
    flag = lambda s: "args.vla_hierarchy" in s                        # noqa: E731
    on = lambda s: "hierarchy is not None" in s or "args.vla_hierarchy" in s   # noqa: E731
    # vla_hierarchy is imported only under the flag, never at module level.
    imports = [n for n in ast.walk(tree) if (isinstance(n, ast.ImportFrom)
                                              and n.module == "vla_hierarchy")
               or (isinstance(n, ast.Import) and any(x.name == "vla_hierarchy" for x in n.names))]
    assert len(imports) >= 2, "vla_hierarchy is not wired into follow_vlm (main and fly)"
    for n in imports:
        assert guarded(n, flag), f"line {n.lineno}: vla_hierarchy imported outside its flag"
    # `hierarchy` starts None, is built once under the flag, every use guarded.
    made = [n for n in ast.walk(fly) if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == "hierarchy" for t in n.targets)]
    none = [n for n in made if isinstance(n.value, ast.Constant) and n.value.value is None]
    assert len(none) == 1 and not guarded(none[0], lambda s: True), "hierarchy starts as None"
    built = [n for n in made if n not in none]
    assert len(built) == 1 and guarded(built[0], flag), [n.lineno for n in built]
    uses = [n for n in ast.walk(fly) if isinstance(n, ast.Attribute)
            and isinstance(n.value, ast.Name) and n.value.id == "hierarchy"]
    assert len(uses) >= 8, "the hierarchy is not wired into fly()"
    for u in uses:
        assert guarded(u, on), f"line {u.lineno}: hierarchy.{u.attr} outside its guard"
    # Built (hello asked) before the simulator is touched.
    assert built[0].lineno < src[:src.index("client = ProjectAirSimClient(address=args.sim_address)")].count("\n") + 1
    # ONE apply(), on the controller's `smooth`, after the perturbers' and the
    # hold's call sites and before the one Shield call, its result the Shield's input.
    calls = _calls(fly, "hierarchy", "apply")
    assert len(calls) == 1, [c.lineno for c in calls]
    c = calls[0]
    assert guarded(c, lambda s: "hierarchy is not None" in s)
    assert isinstance(c.args[0], ast.Name) and c.args[0].id == "smooth"
    assert any(k.arg == "pose" for k in c.keywords), "the VLA's tracker needs the pose"
    assign = next(n for n in ast.walk(fly) if isinstance(n, ast.Assign) and n.value is c)
    tgt = assign.targets[0]
    assert isinstance(tgt, ast.Tuple) and [e.id for e in tgt.elts] == ["flown_in", "hier_rec"]
    pcall = _calls(fly, "perturber", "apply")[0]
    dcall = _calls(fly, "decider", "apply")[0]
    shield = next(n for n in ast.walk(fly) if isinstance(n, ast.Assign)
                  and ast.unparse(n) == "d = shield.filter(state, flown_in)")
    assert pcall.lineno < dcall.lineno < c.lineno < shield.lineno
    assert src.count("d = shield.filter(state, flown_in)") == 1, "the loop's Shield call moved"
    # The limiter never sees what the VLA flies: its state stays the controller's.
    for n in ast.walk(fly):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "limiter":
            names = {x.id for a_ in n.args for x in ast.walk(a_) if isinstance(x, ast.Name)}
            assert not names & {"flown_in", "hier_rec"}, n.lineno
    # A row says "hierarchy", and metrics "vla_hierarchy", only on such a flight.
    rows = [n for n in ast.walk(fly) if isinstance(n, ast.Subscript)
            and isinstance(n.slice, ast.Constant) and n.slice.value == "hierarchy"]
    assert rows and all(guarded(n, lambda s: "hier_rec is not None" in s) for n in rows)
    mets = [n for n in ast.walk(fly) if isinstance(n, ast.Subscript)
            and isinstance(n.slice, ast.Constant) and n.slice.value == "vla_hierarchy"]
    assert mets and all(guarded(n, lambda s: "is not None" in s) for n in mets)
    # The VLA cameras are subscribed for the hierarchy only, outside --record-vla.
    subs = [n for n in ast.walk(fly) if isinstance(n, ast.Call)
            and ast.unparse(n.func) == "client.subscribe" and "hierarchy.put_" in ast.unparse(n)]
    assert len(subs) == 2 and all(guarded(n, lambda s: "hierarchy is not None" in s)
                                  for n in subs), [ast.unparse(n) for n in subs]
    assert not any(guarded(n, lambda s: "args.record_vla" in s) for n in subs)
    # Started with the mission clock, stopped on every way out.
    assert len(_calls(fly, "hierarchy", "start")) == 1 and _calls(fly, "hierarchy", "stop")


# =========================================================================== #
# 2. The intent's arithmetic
# =========================================================================== #
def test_the_intent_mapping_fwd_to_the_standoff_its_clamp_and_its_rate_limit():
    H = _H()
    assert (H.S_RANGE_M, H.S_RATE_MPS, H.EXPIRY_S) == ((12.0, 22.0), 2.0, 1.5)
    dr = H.desired_range
    assert dr(0.0) == 22.0 and dr(5.0) == 12.0 and math.isclose(dr(2.5), 17.0)
    assert dr(-1.0) == 22.0 and dr(9.0) == 12.0, "clamped to the range"
    vals = [dr(0.1 * k) for k in range(51)]
    assert all(12.0 <= v <= 22.0 for v in vals)
    assert all(b <= a for a, b in zip(vals, vals[1:])), "a longer step never asks for more range"
    assert math.isclose(dr(2.5, (10.0, 30.0)), 20.0) and dr(5.0, (10.0, 30.0)) == 10.0
    # The stand-off in force: starts at the controller's own, moves <= 2 m/s.
    so = H.StandOff()
    assert so.step(12.0, 0.0, DEFAULT_S) == DEFAULT_S, "the first tick is the controller's own"
    seen = [DEFAULT_S]
    for k in range(1, 40):
        seen.append(so.step(12.0, 0.1 * k, DEFAULT_S))
    assert all(abs(b - a) <= 0.2 + 1e-9 for a, b in zip(seen, seen[1:])), seen
    assert seen[-1] == 12.0 and min(seen) == 12.0, "reaches the target and never passes it"
    assert so.step(None, 4.1, DEFAULT_S) == 12.0, "None holds it"
    # A stalled tick buys at most max_dt_s of motion; a clock going back none.
    st = H.StandOff()
    st.step(None, 0.0, 15.0)
    assert math.isclose(st.step(22.0, 5.0, 15.0), 15.0 + 2.0 * 0.25)
    assert st.step(12.0, 4.0, 15.0) == 15.5
    for bad in ((22.0, 12.0), (12.0, 12.0), (0.0, 5.0), (float("nan"), 22.0),
                (12.0, float("inf")), "x", (12.0,)):
        assert H.check_range(bad), bad
    assert H.check_range((12.0, 22.0)) is None and H.check_range([8, 30]) is None


# =========================================================================== #
# 3. FOLLOW, HOLD and the expiry (locked: the controller flies)
# =========================================================================== #
def test_locked_the_controller_flies_at_the_vlas_standoff_land_holds_it_and_an_old_answer_expires():
    rig = _Rig()
    phase = {"kind": "step"}

    def answer(msg, waited):
        if waited < 0.5 - 1e-9 or phase["kind"] is None:
            return None
        return (F.step_reply(msg, 5.0, 0.0, 0.3) if phase["kind"] == "step"
                else F.land_reply(msg))
    rows = rig.run(5.0, answer)
    for r in rows:
        assert r["out"] is EXPERT and not r["rec"]["vla_flown"], r["rec"]
        assert math.isclose(r["want"], r["rec"]["s"], abs_tol=1e-4), "plan() returns s"
    early = [r for r in rows if r["s"] < 0.5 - 1e-9]
    assert early and all(r["rec"]["authority"] == "controller_fallback"
                         and r["rec"]["why"] == "no_answer" and r["want"] == DEFAULT_S
                         for r in early), [r["rec"] for r in early]
    follow = [r for r in rows if r["rec"]["authority"] == "controller_follow"]
    assert follow and math.isclose(follow[0]["s"], 0.5), "from the first answer on"
    assert all(r["rec"]["intent"] == "follow" and r["rec"]["kind"] == "step" for r in follow)
    assert all(r["rec"]["s_target"] == 12.0 and math.isclose(r["rec"]["fwd_m"], 5.0)
               for r in follow)
    assert all(abs(r["rec"]["dpsi_rad"] - 0.3) < 0.02 for r in follow), "dpsi recorded..."
    # ...and steering nothing: the controller's own command object flew (above).
    ws = [r["want"] for r in rows]
    assert all(abs(b - a) <= 0.2 + 1e-6 for a, b in zip(ws, ws[1:])), "s never jumps"
    assert rows[-1]["want"] == 12.0 and min(ws) == 12.0
    # LAND: HOLD at the stand-off in force, the controller flying (locked).
    phase["kind"] = "land"
    hold_rows = rig.run(2.0, answer)
    hold = [r for r in hold_rows if r["rec"]["authority"] == "hold"]
    assert len(hold) >= 10, [r["rec"]["authority"] for r in hold_rows]
    for r in hold:
        assert r["out"] is EXPERT and r["want"] == 12.0 and r["rec"]["intent"] == "hold"
        assert r["rec"]["kind"] == "land" and not r["rec"]["vla_flown"]
    # Silence: the intent expires 1.5 s after the last answer arrived.
    phase["kind"] = None
    tail = rig.run(5.0, answer)
    ages = [(r["rec"]["answer_age_s"], r["rec"]["authority"]) for r in tail]
    assert all(a == "hold" for age, a in ages if age <= 1.5 - 1e-6), ages
    fb = [r for r in tail if r["rec"]["answer_age_s"] > 1.5 + 1e-6]
    assert fb and all(r["rec"]["authority"] == "controller_fallback"
                      and r["rec"]["why"] == "expired" and r["rec"]["intent"] is None
                      and r["out"] is EXPERT for r in fb)
    # ...and the stand-off walks back to the controller's own at <= 2 m/s.
    w = [r["want"] for r in fb]
    assert all(0.0 <= b - a <= 0.2 + 1e-6 for a, b in zip(w, w[1:])), w
    assert w[-1] == DEFAULT_S and w[0] < DEFAULT_S
    assert rig.h.handovers == 0 and rig.h.tracker.counts.get("accepted", 0) == 0, \
        "locked, the VLA's tracker never flew"


def test_follow_puts_the_vlas_standoff_into_every_arm_of_the_controller():
    src, tree, fly, guarded = _tree()
    plans = _calls(fly, "hierarchy", "plan")
    assert len(plans) == 1, [p.lineno for p in plans]
    p = plans[0]
    assert guarded(p, lambda s: "hierarchy is not None" in s)
    assign = next(n for n in ast.walk(fly) if isinstance(n, ast.Assign) and n.value is p)
    assert [ast.unparse(t) for t in assign.targets] == ["want_range"], \
        "the VLA's stand-off is the controller's want_range"
    kw = {k.arg: ast.unparse(k.value) for k in p.keywords}
    # The lock is the controller's own `track` branch, condition for condition.
    br = next(n for n in ast.walk(fly) if isinstance(n, ast.If)
              and ast.unparse(n.test) == "est_obs is not None" and n.orelse
              and isinstance(n.orelse[0], ast.If)
              and "args.det_max_age" in ast.unparse(n.orelse[0].test))
    lock = f"{ast.unparse(br.test)} or ({ast.unparse(br.orelse[0].test)})"
    assert kw["locked"] == lock, (kw["locked"], lock)
    assert kw["default_m"] == "_stand_off()" and kw["pose"] == "(state.x, state.y, state.up, yaw)"
    est = next(n for n in ast.walk(fly) if isinstance(n, ast.Assign)
               and [ast.unparse(t) for t in n.targets] == ["est_obs"])
    assert est.lineno < assign.lineno < br.lineno, "after the estimate, before the branches"
    # Every arm reads want_range: the estimate's stand-off law, the box
    # servo's width, the coast's stop.
    so = [n for n in ast.walk(fly) if isinstance(n, ast.Call)
          and ast.unparse(n.func) == "standoff_speed"]
    assert so and all(ast.unparse(n.args[1]) == "want_range" for n in so)
    sw = [n for n in ast.walk(fly) if isinstance(n, ast.Call)
          and ast.unparse(n.func) == "servo_want_width"]
    assert sw and all(ast.unparse(n.args[0]) == "want_range" for n in sw)
    assert "max(trail_stop_short, want_range, floor)" in src
    # And the law with the VLA's 12 m closes on a car 15 m out that the
    # controller's own 15.83 m backs off from.
    import follow_vlm as FV
    assert FV.standoff_speed(15.0, 12.0, 0.4, 0.0, 5.0, 2.0) > 0.0
    assert FV.standoff_speed(15.0, DEFAULT_S, 0.4, 0.0, 5.0, 2.0) < 0.0
    assert FV.servo_want_width(12.0, 4.0) > FV.servo_want_width(DEFAULT_S, 4.0)


# =========================================================================== #
# 4. SEARCH (unlocked)
# =========================================================================== #
def test_search_flies_vla_flys_follow_tracker_on_the_stand_ins_step():
    """One answer (request 1, 0.5 s after it was sent), every later request
    in flight. Unlocked, each VLA tick's executed command must be what an
    independent StepTracker with the same config makes of the same step from
    the same hand-over and poses; before the answer and after it expires the
    controller's own command object."""
    TD = _TD()
    rig = _Rig()
    rows = rig.run(3.0, answer=TD._first_only(4.0, 0.0, 0.3, after=0.5), locked=False)
    before = [r for r in rows if r["s"] < 0.5 - 1e-9]
    assert before and all(r["out"] is EXPERT and r["rec"]["why"] == "no_answer" for r in before)
    vla = [r for r in rows if r["rec"]["vla_flown"]]
    assert len(vla) >= 14, len(vla)
    for r in vla:
        rec = r["rec"]
        assert rec["authority"] == "vla_search" and rec["intent"] == "search", rec
        assert r["out"] is not EXPERT and rec["tk_seq"] == 1
        assert rec["vla_cmd"] == {k: getattr(r["out"], k) for k in ("vx", "vy", "vz_up",
                                                                     "yaw_rate")}
    assert math.isclose(vla[0]["s"], 0.5), "flown from the tick the answer arrives"
    rec1 = next(x for x in rig.h.records if x["seq"] == 1)
    assert rec1["accepted"] and rec1["outcome"]["t_start"] == vla[0]["t"], rec1["outcome"]
    assert rec1["outcome"]["end"] == "suspended", "the controller took the aircraft back"
    d = Decision.of(1, rec1["t_capture"], Pose.from_wire(rec1["pose_capture"]),
                    Step.from_wire(rec1["step"]))
    ref = StepTracker(rig.cfg)
    ref.set_alt_band(rig.h.tracker.alt_band)
    i0 = rows.index(vla[0])
    ref.note_flown(rows[i0 - 1]["out"], rows[i0 - 1]["t"])
    assert ref.accept(d, vla[0]["t"], Pose(*vla[0]["pose"]))
    for r in vla:
        tk = ref.tick(r["t"], Pose(*r["pose"]), pending=True, hold_horizontal=False)
        assert r["out"] == tk.action, (r["s"], r["out"], tk.action)
        assert r["rec"]["author"] == tk.author and r["rec"]["reason"] == tk.reason
    # The hand-over ramps from the command in force (2.5 m/s^2, 0.6 rad/s^2).
    first, prev = vla[0]["out"], rows[i0 - 1]["out"]
    assert prev is EXPERT
    assert math.hypot(first.vx - prev.vx, first.vy - prev.vy) <= 2.5 * 0.1 + 1e-9
    assert abs(first.yaw_rate - prev.yaw_rate) <= 0.6 * 0.1 + 1e-9
    # Expired (1.5 s after the answer arrived): the controller's planner flies.
    after = [r for r in rows if r["s"] > vla[-1]["s"] + 1e-9]
    assert after and all(r["out"] is EXPERT and r["rec"]["authority"] == "controller_fallback"
                         and r["rec"]["why"] == "expired" for r in after)
    assert vla[-1]["rec"]["answer_age_s"] <= 1.5 + 1e-6 < after[0]["rec"]["answer_age_s"]
    assert rig.h.tracker.suspended and rig.h.handovers == 1


def test_search_without_a_fresh_answer_flies_the_controllers_planner():
    cases = {
        "silent": (lambda msg, waited: None, "no_answer"),
        "timeout": (lambda msg, waited: "timeout" if waited >= 0.3 else None, "no_answer"),
        "server_error": (lambda msg, waited: F.error_reply(msg, "inference error: CUDA")
                         if waited >= 0.3 else None, "no_answer"),
        "parse_fail": (lambda msg, waited: F.parse_fail_reply(msg) if waited >= 0.3 else None,
                       "no_answer"),
    }
    for name, (answer, why) in cases.items():
        rig = _Rig()
        rows = rig.run(3.0, answer=answer, locked=False)
        for r in rows:
            assert r["out"] is EXPERT and r["rec"]["authority"] == "controller_fallback", \
                (name, r["rec"])
            assert r["rec"]["why"] == why and r["want"] == DEFAULT_S, (name, r["rec"])
        s = rig.h.summary()
        assert s["vla_flown_ticks"] == 0 and s["fallback_why"] == {why: len(rows)}, (name, s)
        assert rig.h.handovers == 0 and rig.h.intent is None, name
    # A fresh step, then the link fails: the tracker drops the step and hovers,
    # so the controller flies those ticks - recorded with the tracker's
    # reason while the intent is still SEARCH - until the intent expires.
    rig = _Rig()

    def answer(msg, waited):
        if msg["seq"] == 1 and waited >= 0.5 - 1e-9:
            return F.step_reply(msg, 4.0, 0.0, 0.0)
        if msg["seq"] == 2 and waited >= 0.3 - 1e-9:
            return "timeout"
        return None
    rows = rig.run(3.0, answer=answer, locked=False)
    lost = [r for r in rows if r["rec"].get("planned") == "vla_search"]
    assert lost, [r["rec"]["authority"] for r in rows]
    for r in lost:
        assert r["out"] is EXPERT and r["rec"]["authority"] == "controller_fallback"
        assert r["rec"]["why"] == "tracker_hover:link_lost" and r["rec"]["intent"] == "search"
    assert any(r["rec"]["vla_flown"] for r in rows if r["s"] < 0.8), "the VLA flew before"
    assert rows[-1]["rec"]["why"] == "expired" and rig.h.summary()["answers"]["link_failure"] >= 1


def test_the_lock_coming_back_is_follow_again_and_a_parked_answer_flies_the_hand_over():
    """Locked, then lost for 1.5 s, then locked again, answers every 0.4 s:
    the answer parked while the controller flew is taken at the first
    unlocked tick, anchored there; with the lock back the controller flies at
    once and the tracker is suspended."""
    rig = _Rig()
    lost = lambda s: 1.0 - 1e-9 <= s < 2.5 - 1e-9                      # noqa: E731
    rows = rig.run(4.0, answer=_step(3.0, 0.0, 0.1, after=0.4), locked=lambda s: not lost(s))
    on = [r for r in rows if lost(r["s"])]
    assert on and on[0]["rec"]["vla_flown"], on[0]["rec"]
    assert all(r["rec"]["authority"] == "vla_search" and r["rec"]["vla_flown"] for r in on)
    seq0 = on[0]["rec"]["tk_seq"]
    rec0 = next(x for x in rig.h.records if x["seq"] == seq0)
    assert rec0["t_received"] < on[0]["t"] - 1e-9, "answered while the controller flew"
    assert rec0["outcome"]["t_start"] == on[0]["t"] and rec0["target"]["anchor"] == "answer"
    assert any(x.get("refused") == "superseded_while_parked" for x in rig.h.records)
    i0 = rows.index(on[0])
    prev = rows[i0 - 1]["out"]
    assert math.hypot(on[0]["out"].vx - prev.vx, on[0]["out"].vy - prev.vy) <= 0.25 + 1e-9
    back = [r for r in rows if r["s"] >= 2.5 - 1e-9]
    assert back and all(r["out"] is EXPERT and r["rec"]["authority"] == "controller_follow"
                        for r in back)
    assert rig.h.tracker.suspended and rig.h.handovers == 1
    before = [r for r in rows if 0.4 <= r["s"] < 1.0 - 1e-9]
    assert before and all(r["rec"]["authority"] == "controller_follow" for r in before)


def test_a_land_without_the_lock_is_the_trackers_arrival_hold():
    rig = _Rig()
    rows = rig.run(2.0, answer=lambda msg, waited: F.land_reply(msg) if waited >= 0.4 else None,
                   locked=False)
    hold = [r for r in rows if r["rec"]["authority"] == "hold"]
    assert hold and all(r["rec"]["vla_flown"] and r["rec"]["author"] == "hold"
                        and r["rec"]["reason"] == "land_as_arrival" for r in hold), \
        [r["rec"] for r in hold]
    # A hover ramped down from the command in force: never a landing.
    assert all(r["out"].vz_up >= -1.0 for r in hold)
    sp = [math.hypot(r["out"].vx, r["out"].vy) for r in hold]
    assert all(b <= a + 1e-9 for a, b in zip(sp, sp[1:])) and sp[0] < 4.0
    assert not rig.h.tracker.land_requested


def test_a_fault_never_raises_into_the_loop_and_the_controller_flies():
    rig = _Rig()
    out, rec = rig.h.apply(EXPERT, 0.0, pose=rig.veh.pose())        # apply() without plan()
    assert out is EXPERT and rec["authority"] == "controller_fallback" and "error" in rec
    rig.run(1.0, answer=_step(3.0, 0.0, 0.0, after=0.3), locked=False)
    want = rig.h.plan(1.0, locked=False, pose=None, default_m=DEFAULT_S)
    out, rec = rig.h.apply(EXPERT, 1.0, pose=None)
    assert out is EXPERT and rec["why"] == "no_pose" and rec["authority"] == "controller_fallback"
    assert want == rec["s"]
    rig2 = _Rig()

    def boom():
        raise RuntimeError("camera gone")
    rig2.h.get_capture = boom
    rows = rig2.run(1.0, answer=_step(3.0, 0.0, 0.0))
    assert all(r["out"] is EXPERT for r in rows)
    s = rig2.h.summary()
    assert s["errors"]["n"] >= 1 and "camera gone" in s["errors"]["first"][0], s["errors"]


# =========================================================================== #
# 5. The records
# =========================================================================== #
KEYS = {"source", "authority", "intent", "locked", "why", "answer_age_s", "capture_age_s", "seq",
        "kind", "fwd_m", "dpsi_rad", "latency_s", "s", "s_target", "s_default", "vla_flown"}


def test_every_tick_is_recorded_and_the_summary_says_who_flew_and_why():
    H = _H()
    TD = _TD()
    rig = _Rig()
    phase = {"kind": "step"}

    def answer(msg, waited):
        if waited < 0.5 - 1e-9 or phase["kind"] is None:
            return None
        return (F.step_reply(msg, 2.5, 0.0, 0.1) if phase["kind"] == "step"
                else F.land_reply(msg))
    lost = lambda s: 2.0 - 1e-9 <= s < 3.5 - 1e-9                      # noqa: E731
    rows = rig.run(5.0, answer, locked=lambda s: not lost(s))
    phase["kind"] = "land"
    rows += rig.run(1.5, answer)
    phase["kind"] = None
    rows += rig.run(2.5, answer)
    for r in rows:
        assert KEYS <= set(r["rec"]), sorted(KEYS - set(r["rec"]))
        assert r["rec"]["source"] == H.SOURCE == "vla_hierarchy"
        assert r["rec"]["authority"] in H.AUTHORITIES
        if r["rec"]["vla_flown"]:
            assert {"author", "reason", "tk_seq", "vla_cmd", "close_hold"} <= set(r["rec"])
    s = rig.h.stop()
    json.dumps(s)
    by = {a: sum(1 for r in rows if r["rec"]["authority"] == a) for a in H.AUTHORITIES}
    assert s["ticks"] == len(rows) and s["authority"] == by, (s["authority"], by)
    assert all(by[a] for a in H.AUTHORITIES), by
    assert abs(sum(s["authority_share"].values()) - 1.0) < 1e-3
    assert s["vla_flown_ticks"] == sum(1 for r in rows if r["rec"]["vla_flown"]) > 0
    assert math.isclose(s["vla_flown_share"], s["vla_flown_ticks"] / len(rows), abs_tol=1e-4)
    assert s["intents"] == {k: sum(1 for r in rows if (r["rec"]["intent"] or "none") == k)
                            for k in H.INTENTS + ("none",)}
    assert s["intents_set"]["step"] >= 5 and s["intents_set"]["land"] >= 1
    assert s["fallback_why"].get("no_answer") and s["fallback_why"].get("expired")
    assert s["s_m"]["n"] == len(rows) and s["s_follow_m"]["n"] == by["controller_follow"]
    assert 12.0 <= s["s_follow_m"]["min"] <= s["s_follow_m"]["max"] <= 22.0
    assert set(s["s_m"]) == {"n", "min", "p5", "p50", "p95", "max"}
    assert s["answers"]["step"] >= 5 and s["answers"]["land"] >= 1 and s["requests"] >= 7
    lat = s["latency_s"]
    assert lat["n"] >= 6 and 0.5 <= lat["p50"] <= 0.7 and set(lat) == {"n", "p50", "p95", "max"}
    sv = s["server"]
    assert sv["identity"]["id"] == TD.FT_MODEL["id"] and sv["adapter_variant"] == "fine-tune"
    assert sv["adapter"]["sha256"] == TD.FT_MODEL["adapter"]["sha256"] and sv["is_vla"] is True
    cfg = s["config"]
    assert cfg["s_range_m"] == [12.0, 22.0] and cfg["s_rate_mps"] == 2.0
    assert cfg["expiry_s"] == 1.5 and cfg["tracker"]["anchor_at_answer"] is True
    for word in ("Shield", "slew limiter", "controller_follow", "vla_search", "expir"):
        assert word in s["rule"], word
    assert s["handovers"] >= 1 and s["errors"]["n"] == 0
    line = rig.h.summary_line(s)
    assert "follow" in line and "VLA search" in line and "fallback" in line, line
    d = rig.h.describe()
    for word in ("FOLLOW", "SEARCH", "LAND", "Shield", "22 m", "12 m"):
        assert word in d, (word, d)
    # One line per request in vla_steps (the stub run keeps them in records).
    seqs = sorted({x["seq"] for x in rig.h.records})
    assert seqs == list(range(1, s["requests"] + 1)), (seqs, s["requests"])


def test_the_manifest_and_pilot_record_name_the_vla_that_decided():
    from guardrail.manifest import NO_VLA
    import vla_mission as VM
    TD = _TD()
    rig = _Rig()
    rig.run(1.0, answer=_step(3.0, 0.0, 0.0, after=0.3), locked=False)
    rig.run(1.0, answer=_step(3.0, 0.0, 0.0, after=0.3), locked=True)
    rig.h.stop()
    p = rig.h.pilot_entry(detector={"model_id": "google/owlvit-base-patch32", "hash": "x"})
    json.dumps(p)
    assert p["kind"] == "vla" and p["model_id"] == VM.model_id_for(TD.FT_MODEL)
    assert p["reported_digest"] == TD.FT_MODEL["weights_sha256"]
    assert p["authority"] == "hierarchical" and p["detector"]["model_id"].startswith("google/")
    v = p["vla"]
    assert v["decisions_accepted"] == rig.h.tracker.counts["accepted"] > 0
    assert v["identity"]["weights_sha256"] == TD.FT_MODEL["weights_sha256"]
    assert sum(v["ticks_by_author"].values()) == rig.h.n_vla_flown > 0, \
        "only the ticks the VLA flew count as VLA ticks"
    m = rig.h.manifest("sha256:deadbeefdeadbeef", 42,
                       ROOT / "demo" / "pas_config" / "scene_semantic_vla.jsonc")
    assert m["vla_model_hash"] != NO_VLA and m["random_seed"] == 42, m
    # A stand-in is kind stub, never a VLA.
    stub = dict(F.MODEL)
    rig2 = _Rig(identity=stub, is_vla=False, why_not="a stand-in")
    q = rig2.h.pilot_entry()
    assert q["kind"] == "stub" and q["model_id"].startswith("fake:") and q["not_a_vla_because"]
    # fly() writes them on a hierarchical flight only.
    src, tree, fly, guarded = _tree()
    on = lambda s: "hierarchy is not None" in s                       # noqa: E731
    for attr in ("manifest", "pilot_entry"):
        calls = _calls(fly, "hierarchy", attr)
        assert len(calls) == 1 and guarded(calls[0], on), attr


def test_the_follow_score_is_computed_as_on_any_flight():
    import track_truth as TT
    rows = []
    for i in range(80):
        x, tx, ty = 1.5 * i, 1.5 * i + 18.0 + 4.0 * math.sin(i / 9.0), 3.0 * math.cos(i / 7.0)
        brg = math.atan2(ty, tx - x)
        rows.append({"t": round(0.1 * i, 3), "tick": i + 1, "x": x, "y": 0.0, "up": 8.0,
                     "psi": 0.0, "seen": i % 4 != 3,
                     "det": (None if i % 4 == 3 else
                             {"cx": 384 + 384 * math.tan(brg), "cy": 260.0, "w": 60.0,
                              "score": 0.3, "colour": 0.5, "img_w": 768, "switched": False}),
                     "est_xy": [tx + 1.0, ty - 0.5],
                     "truth": {"class": "car", "pts": [[tx, ty]], "names": ["Car_10"]},
                     "tgt_x": tx, "tgt_y": ty})
    tagged = [dict(r, hierarchy={"authority": "vla_search", "s": 14.0, "vla_flown": True})
              for r in rows]
    assert TT.score_rows(rows)["det_scored"] > 0
    for fn in (TT.score_rows, TT.score_estimate, TT.score_instance):
        assert fn(rows) == fn(tagged), fn.__name__
    src, tree, fly, guarded = _tree()
    line = ('"frac_within_30m": round(float(np.mean([s <= 30 for s in seps])), 3) '
            'if seps else None,')
    assert src.count(line) == 1, "frac_within_30m is no longer computed as before"
    sep = next(n for n in ast.walk(fly) if isinstance(n, ast.FunctionDef) and n.name == "_sep")
    assert "hierarchy" not in ast.unparse(sep)


# =========================================================================== #
# 6. Through the real protocol-v2 client and threads, against a loopback server
# =========================================================================== #
def _fly_threaded(td: Path, port: int, seconds: float, *, locked, kill_after_s=None,
                  server=None, **kw):
    """make_vla_hierarchy against 127.0.0.1:port with the real SemanticObs
    pose ring and VLAFrames (a fake encoder), 10 Hz of wall-clock ticks."""
    H, TD = _H(), _TD()
    from semantic_demo import SemanticObs
    obs = SemanticObs()
    box = {"g": None}
    h = H.make_vla_hierarchy(f"127.0.0.1:{port}", obs=obs, box_source=lambda: box["g"],
                             policy=None, object_text="a red car", cruise_alt=8.0,
                             record_dir=td / "vla_hierarchy",
                             encode=lambda m: bytes(m["data"]), **kw)
    veh = TD._Vehicle()
    t0 = time.time()
    h.start(t0)
    stamp0 = 7_000_000_000
    outs = []
    k = 0
    while time.time() - t0 < seconds:
        if kill_after_s is not None and server is not None and time.time() - t0 > kill_after_s:
            server.stop()
            kill_after_s = None
        t = time.time()
        st = stamp0 + k * 100_000_000
        obs.put_pose_full(t, veh.x, veh.y, veh.up, veh.yaw, 0.0, 0.0, stamp=st)
        box["g"] = {"det": TD._det_at_bearing(0.0), "t_det": t - 0.02, "seq": k, "tier": "ok",
                    "cap": {"t_capture": t - 0.15, "pose": {"yaw": veh.yaw}}}
        lk = locked(t - t0)
        want = h.plan(t - t0, locked=lk, pose=veh.pose(), default_m=DEFAULT_S)
        out, rec = h.apply(EXPERT, t - t0, pose=veh.pose())
        msg = {"data": f"f-{st}".encode(), "width": 256, "height": 256, "time_stamp": st}
        h.put_down(dict(msg, data=f"d-{st}".encode()))
        h.put_front(msg)
        outs.append((t - t0, want, out, rec))
        veh.step(out)
        k += 1
        time.sleep(max(0.0, t0 + k * 0.1 - time.time()))
    t_stop = time.time()
    s = h.stop()
    return h, s, outs, time.time() - t_stop


def test_through_the_real_client_the_records_and_the_summary():
    TD = _TD()
    srv = F.FakeVLAServer(lambda req: F.Delay(0.15, TD._as_ft(F.step_reply(req, 4.0, 0.0, 0.1))),
                          model=TD.FT_MODEL)
    with srv, tempfile.TemporaryDirectory() as td:
        td = Path(td)
        assert srv.port != 8790
        h, s, outs, stop_s = _fly_threaded(td, srv.port, 4.0, locked=lambda t: t < 2.0)
        assert stop_s < 12.0, stop_s
        assert s["answers"].get("step", 0) >= 5 and not s["answers"].get("link_failure"), s
        assert len(srv.requests) <= s["requests"] <= len(srv.requests) + 1
        assert srv.overlaps == 0, "one request outstanding at a time"
        assert s["authority"]["controller_follow"] > 0 and s["vla_flown_ticks"] > 0, s["authority"]
        assert {r["phrase"] for r in srv.requests} == {"straight ahead "}
        for t, want, out, rec in outs:
            if rec["vla_flown"]:
                assert t >= 2.0 - 0.15 and out == Action4D(**rec["vla_cmd"])
            else:
                assert out is EXPERT
            if rec["authority"] == "controller_follow":
                assert math.isclose(want, rec["s"], abs_tol=1e-4), (want, rec["s"])
                assert 12.0 - 1e-9 <= want <= DEFAULT_S + 1e-9, want
        sv = s["server"]
        assert sv["address"] == f"127.0.0.1:{srv.port}" and sv["is_vla"] is True
        assert sv["adapter"]["sha256"] == TD.FT_MODEL["adapter"]["sha256"]
        assert s["config"]["vla_fly_argv"][:4] == ["--mission", "follow", "--cadence",
                                                   "pipelined"]
        d = td / "vla_hierarchy"
        steps = [json.loads(ln) for ln in (d / "vla_steps.jsonl").read_text(
            encoding="utf-8").splitlines()]
        assert len(steps) == s["requests"] and {x["seq"] for x in steps} == \
            set(range(1, s["requests"] + 1))
        assert all("accepted" in x and "refused" in x for x in steps)
        assert not (d / "vla_frames").exists(), "a measurement flight keeps no frames"
        assert s["errors"]["n"] == 0, s["errors"]


def test_a_dead_server_never_crashes_the_flight():
    TD = _TD()
    srv = F.FakeVLAServer(lambda req: TD._as_ft(F.step_reply(req, 3.0, 0.0, 0.0)),
                          model=TD.FT_MODEL)
    srv.start()
    with tempfile.TemporaryDirectory() as td:
        try:
            h, s, outs, stop_s = _fly_threaded(Path(td), srv.port, 4.5, locked=lambda t: False,
                                               kill_after_s=1.5, server=srv)
        finally:
            srv.stop()
        assert stop_s < 12.0, stop_s
        assert s["answers"].get("link_failure", 0) >= 1, s["answers"]
        assert any(rec["vla_flown"] for t, w, o, rec in outs if t < 1.5), "the VLA flew before"
        late = [(o, rec) for t, w, o, rec in outs if t > 3.5]
        assert late and all(o is EXPERT and rec["authority"] == "controller_fallback"
                            for o, rec in late), "after the server died the controller flies"
        assert s["errors"]["n"] == 0, s["errors"]


# =========================================================================== #
# 7. Refusals, before the simulator
# =========================================================================== #
def test_refused_with_the_other_modes_alone_or_with_a_bad_address_or_range():
    H = _H()
    assert H.check_args(NO_PORT) is None
    assert H.check_args(NO_PORT, bottom_tile="prev_front", step_as_rate=1.0) is None
    for kw in ({"s_range": (22.0, 12.0)}, {"s_range": (12.0, 12.0)}, {"bottom_tile": "side"},
               {"step_as_rate": -1.0}, {"expiry_s": 0.0}, {"expiry_s": float("nan")}):
        assert H.check_args(NO_PORT, **kw), kw
    assert H.check_args(NO_PORT, expiry_s=12.0) is None
    for addr in ("nohost", "127.0.0.1:x"):
        assert "--vla-hierarchy" in H.check_args(addr), addr
    for argv, says in (
            (["--vla-hierarchy", NO_PORT, "--record-perturb"], "--vla-hierarchy"),
            (["--vla-hierarchy", NO_PORT, "--record-vla", "d", "--record-perturb-vla", NO_PORT],
             "--record-perturb-vla"),
            (["--vla-hierarchy", NO_PORT, "--decide-every-s", "1.0"], "--decide-every-s"),
            (["--vla-hierarchy", NO_PORT, "--record-vla", "d"], "training"),
            (["--vla-hierarchy-range", "10,20"], "need --vla-hierarchy"),
            (["--vla-hierarchy-bottom-tile", "prev_front"], "need --vla-hierarchy"),
            (["--vla-hierarchy-prev-dt-s", "1.0"], "need --vla-hierarchy"),
            (["--vla-hierarchy-step-as-rate", "1.0"], "need --vla-hierarchy"),
            (["--vla-hierarchy-speed-words"], "need --vla-hierarchy"),
            (["--vla-hierarchy-allow-stand-in"], "need --vla-hierarchy"),
            (["--vla-hierarchy-expiry-s", "12"], "need --vla-hierarchy"),
            (["--vla-hierarchy", NO_PORT, "--vla-hierarchy-expiry-s", "-1"],
             "--vla-hierarchy-expiry-s"),
            (["--vla-hierarchy", "nohost"], "host:port"),
            (["--vla-hierarchy", NO_PORT, "--vla-hierarchy-range", "15,15"], "NEAR < FAR"),
            (["--vla-hierarchy", NO_PORT, "--vla-hierarchy-range", "22,12"], "LO <= HI"),
            (["--vla-hierarchy", NO_PORT, "--vla-hierarchy-bottom-tile", "side"], "invalid"),
            (["--vla-hierarchy", NO_PORT, "--vla-hierarchy-step-as-rate", "-1"],
             "--vla-hierarchy-step-as-rate")):
        err = _refused(argv)
        assert says in err, (argv, err[-400:])
    import follow_vlm as FV
    a = FV.build_arg_parser().parse_args(
        ["--vla-hierarchy", NO_PORT, "--vla-hierarchy-range", "10,25",
         "--vla-hierarchy-bottom-tile", "prev_front", "--vla-hierarchy-prev-dt-s", "1.0",
         "--vla-hierarchy-step-as-rate", "1.0", "--vla-hierarchy-speed-words",
         "--vla-hierarchy-allow-stand-in"])
    assert a.vla_hierarchy_range == (10.0, 25.0) and a.vla_hierarchy_step_as_rate == 1.0
    assert a.vla_hierarchy_bottom_tile == "prev_front" and a.vla_hierarchy_prev_dt_s == 1.0
    assert a.vla_hierarchy_speed_words and a.vla_hierarchy_allow_stand_in
    assert a.vla_hierarchy_expiry_s == 1.5
    b = FV.build_arg_parser().parse_args(["--vla-hierarchy", NO_PORT,
                                          "--vla-hierarchy-expiry-s", "12"])
    assert b.vla_hierarchy_expiry_s == 12.0


def test_the_server_must_be_aerialvla_with_the_adapters_tile_and_speed_words():
    H, TD = _H(), _TD()
    from semantic_demo import SemanticObs
    kw = dict(obs=SemanticObs(), box_source=lambda: None, policy=None,
              object_text="a red car", cruise_alt=8.0, record_dir=None)
    port = TD._free_port()
    try:
        H.make_vla_hierarchy(f"127.0.0.1:{port}", **kw)
    except H.HierarchyRefused as exc:
        assert str(port) in str(exc)
    else:
        raise AssertionError("no server, yet accepted")
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0)) as srv:
        assert srv.port != 8790
        try:
            H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", **kw)
        except H.HierarchyRefused as exc:
            assert "--vla-hierarchy-allow-stand-in" in str(exc), exc
            assert "--perturb-vla" not in str(exc), exc
        else:
            raise AssertionError("a stand-in identity was accepted without being allowed")
        h = H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", allow_stand_in=True, **kw)
        try:
            sv = h.summary()["server"]
            assert sv["is_vla"] is False and sv["why_not_vla"], sv
            assert "STAND-IN" in h.describe()
            assert h.expiry_s == H.EXPIRY_S == 1.5
        finally:
            h.stop()
        h = H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", allow_stand_in=True, expiry_s=12.0,
                                 **kw)
        try:
            assert h.expiry_s == 12.0 and "held 12 s" in h.describe(), h.describe()
        finally:
            h.stop()
    # The adapter's lower tile and speed words, against the flight's.
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), model=TD.FT_MODEL) as srv:
        for flags, word in (({"bottom_tile": "prev_front"}, "lower tile"),
                            ({"speed_words": True}, "speed words")):
            try:
                H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", **kw, **flags)
            except H.HierarchyRefused as exc:
                assert word in str(exc) and "--vla-hierarchy" in str(exc), exc
            else:
                raise AssertionError(f"{flags} was accepted against a down, wordless adapter")
        H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", **kw).stop()
    pf = json.loads(json.dumps(TD.FT_MODEL))
    pf["adapter"]["provenance"] = {"id_root": "aerialvla-ft-follow", "run": "follow_v4_r3",
                                   "epoch": 1, "epoch_dir": "epoch1", "label_horizon_s": 1.0,
                                   "bottom_tile": {"mode": "prev_front", "dt_s": 1.0,
                                                   "tol_s": 0.25}}
    with F.FakeVLAServer(lambda req: F.step_reply(req, 1.0, 0.0, 0.0), model=pf) as srv:
        h = H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", bottom_tile="prev_front",
                                 step_as_rate=1.0, **kw)
        try:
            c = h.summary()["config"]
            assert c["bottom_tile"]["ok"] and c["step_as_rate_s"] == 1.0
            assert c["tracker"]["rate_horizon_s"] == 1.0
        finally:
            h.stop()
        try:
            H.make_vla_hierarchy(f"127.0.0.1:{srv.port}", **kw)
        except H.HierarchyRefused as exc:
            assert "lower tile" in str(exc), exc
        else:
            raise AssertionError("a prev_front adapter was flown with the down tile")


# =========================================================================== #
# 8. The launcher
# =========================================================================== #
def test_the_launcher_passes_the_hierarchy_and_refuses_the_other_modes_before_the_simulator():
    src = PS1.read_text(encoding="utf-8")
    for p in ("[string]$VlaHierarchy", "[string]$VlaHierarchyBottomTile",
              "[double]$VlaHierarchyPrevDtS", "[double]$VlaHierarchyStepAsRate",
              "[switch]$VlaHierarchySpeedWords", "[string]$VlaHierarchyRange",
              "[switch]$VlaHierarchyAllowStandIn",
              # still there, and still passed to the simulator
              "[int]$SimMaxFps", "[switch]$SimOffscreen"):
        assert p in src, p
    assert "t.MaxFPS $SimMaxFps" in src and "'-RenderOffscreen'" in src
    lines = src.splitlines()
    # The pass-through block: the last top-level `if ($VlaHierarchy) {` before
    # the --vla-hierarchy argument (the first one checks the server).
    f0 = next(n for n, ln in enumerate(lines) if '"--vla-hierarchy"' in ln)
    top = max(n for n, ln in enumerate(lines[:f0]) if ln.startswith("if ($VlaHierarchy) {"))
    for flag, guard in (('"--vla-hierarchy"', "if ($VlaHierarchy) {"),
                        ('"--vla-hierarchy-bottom-tile"', "if ($VlaHierarchyBottomTile)"),
                        ('"--vla-hierarchy-prev-dt-s"', "if ($VlaHierarchyPrevDtS)"),
                        ('"--vla-hierarchy-step-as-rate"', "if ($VlaHierarchyStepAsRate)"),
                        ('"--vla-hierarchy-speed-words"', "if ($VlaHierarchySpeedWords)"),
                        ('"--vla-hierarchy-range"', "if ($VlaHierarchyRange)"),
                        ('"--vla-hierarchy-allow-stand-in"', "if ($VlaHierarchyAllowStandIn)")):
        assert src.count(flag) == 1, flag
        fi = next(n for n, ln in enumerate(lines) if flag in ln)
        gi = max(n for n, ln in enumerate(lines[:fi]) if ln.lstrip().startswith(guard))
        assert top <= gi < fi, f"{flag} is not inside the -VlaHierarchy block"
        for start in (gi, top):
            ind = len(lines[start]) - len(lines[start].lstrip())
            for ln in lines[start + 1:fi + 1]:
                assert not ln.strip() or len(ln) - len(ln.lstrip()) > ind, (flag, ln)
    # The block is at the top level, not inside -RecordVla's.
    vi = next(n for n, ln in enumerate(lines) if ln.startswith("if ($RecordVla) {"))
    ve = next(n for n in range(vi + 1, len(lines)) if lines[n] == "}")
    assert not vi < top < ve
    sim = src.index("if (-not $SkipSim) { Start-Sim }")
    for refusal in ("if ($VlaHierarchy -and ($RecordPerturb -or $RecordPerturbVla -or "
                    "$DecideEveryS))",
                    "if ($VlaHierarchy -and $RecordVla)",
                    "-and -not $VlaHierarchy)",
                    "Test-NetConnection $hierHost -Port $hierPort"):
        assert refusal in src and src.index(refusal) < sim, refusal
    for sub in ("$VlaHierarchyBottomTile", "$VlaHierarchyPrevDtS", "$VlaHierarchyStepAsRate",
                "$VlaHierarchySpeedWords", "$VlaHierarchyRange", "$VlaHierarchyAllowStandIn"):
        i = src.index("-and -not $VlaHierarchy)")
        assert sub in src[src.rindex("if (", 0, i):i], f"{sub} is not refused without -VlaHierarchy"
    assert "-VlaHierarchy 127.0.0.1:8790" in src, "the header shows a hierarchical flight"
    assert "vla_hierarchy" in src.split("Say \"done.", 1)[1], "the closing lines name the metrics"


# --------------------------------------------------------------------------- #
# runner
# --------------------------------------------------------------------------- #
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
