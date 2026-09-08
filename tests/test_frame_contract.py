"""Ours is world-frame, the reference is body-frame. Which is right, and where.

Run either way:
    pytest tests/test_frame_contract.py -v
    python tests/test_frame_contract.py

`docs/CHECKLIST-remaining-work.md` item 9 has said for weeks that our `Action4D`
(vx North, vy East) and `vlaguard_common.Action4D` (vx forward, vy right)
disagree, that "both cannot be right", and that "no test compares them". This
file is that test, and the conclusion it pins is that both ARE right in their own
frame: the conversion is exact, so the mismatch is a boundary, not a defect.

It also pins the thing that mattered far more and was nowhere on the list. The
first version of this file asserted that `yaw_rate` was deg/s here and rad/s in
the reference — a factor of 57.3 — on the strength of a comment in
`guardrail/models.py`. The comment was wrong. Everything that ENFORCES the cap
reads radians, and two adapters had believed the comment and converted radians to
radians a second time. So the tests below check the yaw contract against the code
that enforces it rather than against any comment, and check that the adapters
agree with it. See `docs/FINDING-the-contract-disagreed-with-itself-about-yaw.md`.
"""
import importlib.util
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from guardrail.frames import (REFERENCE_YAW_RATE_UNITS,    # noqa: E402
                              YAW_RATE_UNITS, from_body,
                              to_body, to_local_ned)
from guardrail.models import (Action4D, KinematicEnvelope,  # noqa: E402
                              Policy, State)
from guardrail.shield import Shield                         # noqa: E402

REF = (ROOT / "kuanting-vla-uav-guardrail" / "packages" / "vlaguard-common" /
       "src" / "vlaguard_common" / "frames.py")

HEADINGS = [0.0, 30.0, 90.0, 145.0, 180.0, 271.5, 359.0, -45.0]
ACTIONS = [(3.0, 0.0, 0.0), (0.0, 2.0, 0.0), (1.5, -2.5, 1.0),
           (-4.0, 0.5, -0.75), (0.0, 0.0, 0.0)]

CAP_DPS = 45.0


def _reference():
    """The reference module loaded from its own file, or None."""
    if not REF.is_file():
        return None
    spec = importlib.util.spec_from_file_location("_ref_frames", REF)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception:                     # pydantic missing, etc.
        return None
    return mod


def _yaw_only_shield():
    """A Shield whose only rule is a yaw-rate cap, so nothing else can fire."""
    pol = Policy(policy_id="yaw-units-probe", version="0.0.1", constraints=[
        KinematicEnvelope(id="kin", type="kinematic_envelope",
                          constraint_type="hard", priority="P1",
                          violation_action="repair",
                          speed_max_mps=50.0, climb_rate_max_mps=50.0,
                          yaw_rate_max_dps=CAP_DPS),
    ])
    return Shield(pol, lookahead_s=1.0, dt=0.5)


# --- the frame conversion -------------------------------------------------

SKIP = "SKIP"


def test_a_heading_of_zero_makes_the_two_frames_identical():
    """Facing North, forward IS North and right IS East. Any conversion bug that
    survives this test is a rotation bug, which is the easy kind."""
    a = from_body(3.0, 2.0, 1.0, 0.0, 0.0)
    assert abs(a.vx - 3.0) < 1e-12 and abs(a.vy - 2.0) < 1e-12
    assert abs(a.vz_up - 1.0) < 1e-12


def test_facing_east_turns_forward_into_east():
    a = from_body(3.0, 0.0, 0.0, 0.0, 90.0)
    assert abs(a.vx) < 1e-9, a.vx           # nothing North
    assert abs(a.vy - 3.0) < 1e-9, a.vy     # all of it East


def test_right_is_ninety_degrees_clockwise_of_forward():
    """Facing North, 'right' is East. This is the sign that is easy to invert
    and impossible to notice in a straight line."""
    a = from_body(0.0, 2.0, 0.0, 0.0, 0.0)
    assert abs(a.vy - 2.0) < 1e-12, a.vy


def test_the_round_trip_is_exact_at_every_heading():
    for yaw in HEADINGS:
        for vx, vy, vz in ACTIONS:
            a = from_body(vx, vy, vz, 0.25, yaw)
            bx, by, bz, br = to_body(a, yaw)
            assert abs(bx - vx) < 1e-9 and abs(by - vy) < 1e-9, (yaw, vx, vy)
            assert abs(bz - vz) < 1e-12
            assert abs(br - 0.25) < 1e-12


def test_our_world_frame_action_is_heading_independent():
    """The reason the Shield is world-frame at all: a fence, a stand-off and an
    altitude band are world geometry, so the action they are checked against must
    not change meaning when the aircraft turns."""
    a = Action4D(vx=2.0, vy=0.0, vz_up=0.0, yaw_rate=0.0)
    for yaw in HEADINGS:
        n, e, d, _ = to_local_ned(a)
        assert (n, e, d) == (2.0, 0.0, -0.0), yaw


def test_up_positive_on_both_sides_and_down_positive_only_at_the_wire():
    a = from_body(0.0, 0.0, 1.5, 0.0, 33.0)
    assert a.vz_up == 1.5
    assert to_local_ned(a)[2] == -1.5      # NED down is positive


def test_we_agree_with_the_reference_implementation_itself():
    """Not a restatement of their formula — their function, run."""
    ref = _reference()
    if ref is None:
        return SKIP   # reference package not in this tree
    for yaw in HEADINGS:
        for vx, vy, vz in ACTIONS:
            theirs = ref.body_to_local_ned(
                ref.Action4D(vx=vx, vy=vy, vz=vz, yaw_rate=0.3),
                math.radians(yaw))
            ours = to_local_ned(from_body(vx, vy, vz, 0.3, yaw))
            for k in range(4):
                assert abs(theirs[k] - ours[k]) < 1e-9, (yaw, vx, vy, vz, k)


# --- the yaw unit, checked against what ENFORCES it -----------------------

def test_the_declared_yaw_unit_is_what_the_shield_actually_enforces():
    """The test that would have caught the original error. It asks the Shield,
    not a comment: an action at the cap expressed in the declared unit must be
    legal, and one just over it must not."""
    assert YAW_RATE_UNITS == "rad/s"
    sh = _yaw_only_shield()
    st = State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0)
    at_cap = Action4D(vx=0.0, vy=0.0, vz_up=0.0,
                      yaw_rate=math.radians(CAP_DPS) - 1e-6)
    over = Action4D(vx=0.0, vy=0.0, vz_up=0.0,
                    yaw_rate=math.radians(CAP_DPS) * 1.5)
    assert sh.filter(st, at_cap).violations == [], sh.filter(st, at_cap).violations
    assert sh.filter(st, over).violations, "a 1.5x yaw was not flagged"


def test_a_degrees_reading_of_the_cap_would_be_flagged_absurdly():
    """Pins the failure the first version of this file shipped. 28.6 deg/s is
    legal under a 45 dps cap; handed over as if it were rad/s it reads as
    1641 dps and the Shield clamps a perfectly lawful turn."""
    sh = _yaw_only_shield()
    st = State(x=0.0, y=0.0, up=10.0, yaw_deg=0.0)
    as_degrees = Action4D(vx=0.0, vy=0.0, vz_up=0.0, yaw_rate=28.6478897565)
    v = sh.filter(st, as_degrees).violations
    assert v, "the mis-scaled action was not flagged"
    assert "1641" in v[0].detail, v[0].detail


def test_the_reference_and_ours_use_the_same_yaw_unit_so_nothing_converts():
    """The mismatch item 9 named is the FRAME. There is no unit mismatch, and
    inserting a conversion because one was expected is what broke it."""
    assert YAW_RATE_UNITS == REFERENCE_YAW_RATE_UNITS == "rad/s"
    a = from_body(0.0, 0.0, 0.0, 0.5, 137.0)
    assert a.yaw_rate == 0.5
    assert to_body(a, 137.0)[3] == 0.5
    assert to_local_ned(a)[3] == 0.5


def test_the_reference_docstring_still_says_rad_s():
    """If the reference ever changes its unit, this contract changes with it and
    the change must not be silent."""
    if not REF.is_file():
        return SKIP
    assert "rad/s" in REF.read_text(encoding="utf-8")


# --- the adapters, which are where the error actually lived ---------------

ADAPTERS = ["sitl/run_sitl_demo.py", "sitl/ros2_shield_node.py"]


def test_no_adapter_converts_the_yaw_rate_a_second_time():
    """The regression guard. Both of these applied math.radians() to a value
    that was already radians, dividing every commanded yaw by 57.3. It never
    reached a flight only because the stub pilot this rail flies has never
    commanded a non-zero yaw rate.

    Checked in the source because importing them needs pymavlink and rclpy,
    which are not present in every environment that runs this suite. A
    line-level check is enough: the defect was one call on one line.
    """
    bad = []
    for rel in ADAPTERS:
        p = ROOT / rel
        if not p.is_file():
            continue
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if line.lstrip().startswith("#"):
                continue
            if "yaw_rate" in line and re.search(r"math\.(radians|degrees)\s*\(", line):
                bad.append(f"{rel}:{i}: {line.strip()}")
    assert not bad, "yaw_rate is rad/s on both sides of these:\n" + "\n".join(bad)


def test_the_shield_is_allowed_to_convert_because_the_cap_is_in_degrees():
    """The counterpart. shield.py MUST convert — `yaw_rate_max_dps` is degrees
    by name and by policy file — so the rule above is about adapters only."""
    src = (ROOT / "guardrail" / "shield.py").read_text(encoding="utf-8")
    assert "math.radians(k.yaw_rate_max_dps)" in src
    assert "yaw_rate_max_dps" in (ROOT / "guardrail" / "models.py").read_text(
        encoding="utf-8")


if __name__ == "__main__":
    # A test that short-circuits on a missing fixture must NOT print PASS - on a
    # clean clone demo/out/ is gitignored and those tests assert nothing. See
    # tests/test_replay.py, where that hid 7 no-ops behind "8/8 passed".
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
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
