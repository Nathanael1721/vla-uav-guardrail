"""The perception rail's subject ground truth, checked against the Shield's
frame (rail-video unit, part 2).

    python tests/test_railvideo_truth_frame.py      (no pytest needed)

WHY. pas_ap_close_on / pas_ap_close_off (2026-10-13) logged Car_10 119-235 m
north-east of the aircraft while every detector estimate sat south-east, and
the record could not say whether the truth was in another frame or the
detector was on another object (CHANGELOG 2026-10-13 (later), "Found"). Read
offline (this unit's report), it is the detector: the node's Car_10 poses lie
on the Car_10 track follow_vlm logged in the same level
(demo/out/citylife_redcar_nfz_20261013: nearest point a median 0.52 / 0.54 m
away, max 1.27 m, over all 532 / 1100 rows), and the subject was behind the
aircraft on most estimate ticks. The "123 m offset" in the brief compared
follow_vlm's start-gate `truth_trace` - [t_s, range_m, bearing_deg] rows
(demo/follow_vlm.py, the start gate) - with the node's [x, y]: (95.6, -3.8)
is 95.6 m at -3.8 deg, not a position.

So nothing here changes where the truth is put. What changes is that a
flight now answers the question itself:

  * SubjectTruth.check_frame: the AIRCRAFT read through the subject's own RPC
    (World.get_object_poses on pn.AIRCRAFT_ACTOR) and projection
    (scene_to_anchor), against the EKF position the Shield flies on; beside
    it the EKF against the simulator's kinematics (ekf_vs_sim). Over
    TRUTH_FRAME_MAX_M the truth scores nothing, its figures move to
    `as_read` and the cause says which was off. Without the aircraft's
    object pose the frame is only PARTLY checked and never "agrees".
  * truth_view: on the ticks with an estimate, whether the subject was inside
    the camera's field of view (at the tick's heading, and at any heading
    since the estimate's frame could have been taken), the estimate's
    bearing error, both ranges, and the estimates within 6 m of it.

The stand-ins fly with a policy origin 400 m north and 250 m west of the
scene's home-geo-point (a non-zero scene offset), so the truth and the EKF
reach the Shield's frame by different projections.

On review (2026-10-14) the first version read the aircraft through
Drone.get_ground_truth_kinematics, so an offset in the level-actor poses
alone - the hypothesis the check is for - passed as "agrees", and its
regression test shifted both reads alike. Here the stand-in shifts ONLY the
object poses (`frame_dy`) while the kinematics stay put, as the reviewer's
counter-example did.

Shown failing on a pristine HEAD 5cf7a08 copy (git archive, the stored
flight logs copied in) with this file copied in: 0/19. The regression flight
fails on its own assertion - scored (follow_scored True, frac_within_30m
0.0, standoff_s 0.0, "measured: the pose was read on 30/30 ticks") off a
truth 123.3 m from where the car was - and the ordering test on its own (no
aircraft read at all); the rest on what HEAD lacks (truth_view,
check_frame, object_pose, the frame_check record).

The stored-flight test re-reads the two close-approach flights and skips
visibly where demo/out (gitignored) does not hold them.
"""
from __future__ import annotations

import inspect
import json
import math
import random
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PAS = ROOT / "demo" / "pas_ardupilot"
for _p in (ROOT, ROOT / "demo", PAS, ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import test_pas_ardupilot as T                                       # noqa: E402
import autopilot as ap                                               # noqa: E402
import perception_node as pn                                         # noqa: E402
from guardrail.projection import LocalProjection                     # noqa: E402
from guardrail.models import State                                   # noqa: E402

SKIP = "SKIP"
HOME = (35.6895, 139.6917, 40.0)          # the CityLife scene's home-geo-point
# The policy's origin, in the scene's frame: 400 m north, 250 m west of home.
ORIGIN_IN_SCENE = (400.0, -250.0)
# The offset the brief hypothesised between the truth and the Shield's frame.
BRIEF_OFFSET_M = 123.3
OUT = ROOT / "demo" / "out"
NAN = float("nan")
# The actor the node reads the aircraft under. Spelled with a fallback so the
# stand-in still serves the CAR correctly to a node without the frame check
# (the HEAD comparison): only the frame-check tests then fail.
AIRCRAFT = getattr(pn, "AIRCRAFT_ACTOR", "UnrealRobot")
# Every row field derived from the subject's position (the scoring fields a
# quarantine must take out), spelled here, not read from the node.
TRUTH_DERIVED = ("tgt_x", "tgt_y", "subject_range_m", "subject_gap_m",
                 "subject_truth_off_nose_deg", "subject_bearing_gap_deg")


def _origin_latlon() -> tuple[float, float]:
    return LocalProjection(HOME[0], HOME[1]).to_latlon(*ORIGIN_IN_SCENE)


def _policy(standoff_m: float = 5.0) -> str:
    lat, lon = _origin_latlon()
    return f"""policy_id: pas-rv-truthframe
version: 0.1.0
origin:
  lat: {lat!r}
  lon: {lon!r}
constraints:
  - id: standoff-any
    type: subject_standoff
    constraint_type: hard
    priority: P0
    violation_action: repair
    subject_class: "*"
    min_range_m: {standoff_m}
"""


class SimCamera(T.FakeCamera):
    """A simulator. Level-actor poses (World.get_object_poses: the car, and
    the aircraft under pn.AIRCRAFT_ACTOR) and the vehicle's kinematics
    (truth_geo) in ITS NED frame about the scene's home; the autopilot's GPS
    (and so the EKF) lives in the link's frame.

      frame_dy   shifts ONLY the object poses east (a truth in another frame:
                 the kinematics stay where the EKF is);
      sim_dx     puts the simulator's aircraft that far north of the EKF's,
                 in every read (an EKF off the simulator);
      car        the car relative to the simulator's aircraft (north, east);
      aircraft   "actor" (found), "missing" (NaN: no actor matches), or an
                 exception to raise;
      yaw_off    the actor's heading minus the EKF's, in degrees;
      calls      a shared call log (with CallLink) for the ordering test."""

    def __init__(self, link, log=None, frame_dy=0.0, car=(0.0, 2.0), sim_dx=0.0,
                 aircraft="actor", yaw_off=0.0, calls=None):
        super().__init__(link, log)
        self.frame_dy, self.car, self.sim_dx = frame_dy, car, sim_dx
        self.aircraft, self.yaw_off, self.calls = aircraft, yaw_off, calls
        self.aircraft_reads = 0
        self.asked: list = []

    def _sim_xy(self):
        return self.link.x + self.sim_dx, self.link.y

    def object_pose(self, name):
        self.asked.append(name)
        if self.calls is not None:
            self.calls.append(("pose", name))
        ax, ay = self._sim_xy()
        if name == AIRCRAFT:
            self.aircraft_reads += 1
            if isinstance(self.aircraft, BaseException):
                raise self.aircraft
            if self.aircraft == "missing":
                return NAN, NAN, None
            return ax, ay + self.frame_dy, math.degrees(self.link.yaw) + self.yaw_off
        return ax + self.car[0], ay + self.car[1] + self.frame_dy, 0.0

    def object_xy(self, name):
        x, y, _ = self.object_pose(name)
        return x, y

    def truth_geo(self):
        if self.calls is not None:
            self.calls.append(("geo",))
        lat, lon = self.link.anchor.to_latlon(*self._sim_xy())
        return {"lat": lat, "lon": lon, "alt": 48.0}

    def get_front_native(self):
        """A frame source for --save-view; the spy recorder never asks."""
        return None


class NoGeoCamera(SimCamera):
    """The same simulator without the kinematics read (no ekf_vs_sim)."""

    @property
    def truth_geo(self):
        raise AttributeError("truth_geo")


class CallLink(T.FakeLink):
    """The stand-in autopilot, logging each tick's pump and send. `stall`
    (n, s): the n-th pump takes s seconds of the loop's clock (a slow tick,
    after which every periodic check is overdue at once)."""

    def __init__(self, *a, calls=None, stall=None, **k):
        super().__init__(*a, **k)
        self.calls = calls if calls is not None else []
        self.stall, self.n_pump = stall, 0

    def pump(self):
        self.calls.append(("pump",))
        self.n_pump += 1
        if self.stall and self.n_pump == self.stall[0]:
            self.clock.t += self.stall[1]
        super().pump()

    def send(self, a, raw_body=None):
        self.calls.append(("send",))
        super().send(a, raw_body)


def _fly(policy: str, *, camera_kw=None, max_s="3", extra=(), link_kw=None,
         recorder_cls=None, camera_cls=SimCamera, link_cls=T.FakeLink):
    """One stand-in flight; `recorder_cls` (with --save-view in `extra`)
    stands in for the recorder, as in tests/test_railvideo_recording.py."""
    out = Path(tempfile.mkdtemp(prefix="pas_tf_"))
    pol = out / "policy.yaml"
    pol.write_text(policy, encoding="utf-8")
    args = pn.build_parser().parse_args([
        "--out-root", str(out), "--tag", "run", "--policy", str(pol), "--max-s", max_s,
        "--hfov", "90", "--citymap", str(out / "no_map.npz"), "--network", "nat",
        "--windows-ip", T.WIN, "--wsl-ip", T.WSL, "--shield", "on", "--speed-max", "4",
        "--dev-yaml", "--object", "a red car", *extra])
    clk = T.Clock()
    # The link's own frame is the scene's: its x, y are simulator NED about
    # the home-geo-point, and its GPS fixes are that point plus them, as
    # ArduPilot's AirSim backend makes them (rail.sitl_home).
    link = link_cls(clk, ap.FrameAnchor(*HOME, "scene.home-geo-point"), **(link_kw or {}))
    cam = camera_cls(link, **(camera_kw or {}))
    rec_kw = {} if recorder_cls is None else {"recorder_cls": recorder_cls}
    rc = pn.run(args, link=link, camera=cam, grounder=T.FakeDetections(clk),
                obs=T.FakeObs(), clock=clk, sleep=clk.sleep, **rec_kw)
    run = out / "run"
    rows = [json.loads(x) for x in (run / "flight_log.jsonl").read_text().splitlines()]
    return rc, run, rows, cam


def _json(run, name):
    return json.loads((run / name).read_text())


# =========================================================================== #
# The frame check (pure)
# =========================================================================== #

def test_the_frame_check_reads_the_aircraft_through_the_subjects_own_rpc():
    """With the anchor 470 m from the scene's home, the aircraft's object pose
    agrees with its EKF position only through scene_to_anchor - the
    projection the subject's pose takes (skipping it would read ~472 m) -
    and it is asked for under pn.AIRCRAFT_ACTOR, through object_pose: the
    RPC the subject is read with."""
    home_anchor = ap.FrameAnchor(*HOME, "scene.home-geo-point")
    lat, lon = _origin_latlon()
    anchor = ap.FrameAnchor(lat, lon, None, "policy.origin")
    sim = (35.0, -20.0)                          # the scene's spawn point
    ex, ey = anchor.to_scene(*home_anchor.to_latlon(*sim))   # what the EKF reads
    assert math.hypot(ex - sim[0], ey - sim[1]) > 400.0, "no offset: the test is blind"
    st = State(x=ex, y=ey, up=8.0, yaw_deg=10.0)
    asked = []

    def pose(n):
        asked.append(n)
        return (sim[0], sim[1], 13.0) if n == pn.AIRCRAFT_ACTOR else (sim[0], sim[1] + 2.0, 0.0)
    cam = SimpleNamespace(object_xy=lambda n: pose(n)[:2], object_pose=pose)
    tr = pn.SubjectTruth(cam, "Car_10", None, HOME, anchor)
    got = tr.check_frame(st)
    assert asked == [pn.AIRCRAFT_ACTOR], asked
    assert pn.AIRCRAFT_ACTOR != pn.rail.ROBOT_NAME, \
        "the robot id names no actor in the plugin (UnrealScene.cpp spawns it unnamed)"
    assert got is not None and got["err_m"] < 0.01, got
    assert abs(got["sim_xy"][0] - ex) < 0.01 and abs(got["sim_xy"][1] - ey) < 0.01
    assert abs(got["yaw_err_deg"] - 3.0) < 1e-9, got
    fs = tr.frame_summary()
    assert fs["checked"] and fs["agrees"] is True and fs["err_m"]["n"] == 1, fs
    assert fs["aircraft_actor"] == pn.AIRCRAFT_ACTOR and fs["check_ms"]["n"] == 1
    # The same simulator with its object poses 123.3 m east of the EKF's frame.
    cam2 = SimpleNamespace(object_xy=lambda n: (sim[0], sim[1] + BRIEF_OFFSET_M),
                           object_pose=lambda n: (sim[0], sim[1] + BRIEF_OFFSET_M, 10.0))
    tr2 = pn.SubjectTruth(cam2, "Car_10", None, HOME, anchor)
    got2 = tr2.check_frame(st)
    assert abs(got2["err_m"] - BRIEF_OFFSET_M) < 0.05, got2
    fs2 = tr2.frame_summary()
    assert fs2["agrees"] is False and fs2["cause"] == "frame", fs2
    # A camera that reads only positions (no object_pose) is still checked,
    # through object_xy, with no heading.
    cam3 = SimpleNamespace(object_xy=lambda n: sim)
    tr3 = pn.SubjectTruth(cam3, "Car_10", None, HOME, anchor)
    assert tr3.check_frame(st)["yaw_err_deg"] is None
    assert tr3.frame_summary()["heading_agrees"] is None


def test_the_frame_limit_is_twice_g1s_ekf_tolerance_and_judged_on_the_median():
    """TRUTH_FRAME_MAX_M is g1_check.EKF_TOL_M doubled, as its comment says.
    The stored flights' EKF error against the simulator (ekf_vs_sim_horiz_m
    median 0.324 / 0.182 m, p95 0.628 / 0.591 m: demo/out/pas_ap_close_on|
    off/metrics.json) must agree; a few metres must not; and one good check
    does not outvote four bad ones (the median, not the best)."""
    import g1_check
    assert pn.TRUTH_FRAME_MAX_M == 2 * g1_check.EKF_TOL_M
    anchor = ap.FrameAnchor(*HOME, "scene.home-geo-point")
    st = State(x=35.0, y=-20.0, up=8.0)

    def judged(offsets, ekf=()):
        it = iter(offsets)
        cam = SimpleNamespace(object_xy=lambda n: (35.0, -20.0 + next(it)))
        tr = pn.SubjectTruth(cam, "Car_10", None, HOME, anchor)
        for _ in offsets:
            tr.check_frame(st)
        for e in ekf:
            tr.note_ekf_vs_sim(e)
        return tr
    assert judged([0.6] * 5).frame_verdict() == (True, None)
    assert judged([0.63, 0.59, 0.68, 0.3, 0.2]).frame_summary()["agrees"] is True
    assert judged([3.0] * 5).frame_verdict() == (False, "frame")
    assert judged([0.0, 10.0, 10.0, 10.0, 10.0]).frame_verdict()[0] is False
    assert judged([0.0, 0.0, 0.0, 0.0, 10.0]).frame_verdict()[0] is True
    assert judged([0.0, 0.0, 0.0, 0.0, 10.0]).frame_disagrees() is False
    # Disagreeing while the EKF is off the kinematics too: the EKF's fault.
    assert judged([5.0] * 5, ekf=[5.0] * 5).frame_verdict() == (False, "ekf")
    assert judged([5.0] * 5, ekf=[0.1] * 5).frame_verdict() == (False, "frame")


def test_an_unchecked_frame_is_never_reported_as_agreeing():
    """No pose reader, a failing one, a NaN one, a reader that is off, or
    only the EKF-against-kinematics check: `agrees` is None with the reason
    (or False when that EKF check itself fails), failures are counted, and a
    failing aircraft read backs off like the subject's."""
    anchor = ap.FrameAnchor(*HOME, "scene.home-geo-point")
    st = State(x=35.0, y=-20.0, up=8.0)
    tr = pn.SubjectTruth(SimpleNamespace(), "Car_10", None, HOME, anchor)
    assert tr.check_frame(st) is None
    fs = tr.frame_summary()
    assert fs["checked"] is False and fs["agrees"] is None and "no simulator poses" in fs["why"]

    def boom(n):
        raise TimeoutError("rpc")
    tr = pn.SubjectTruth(SimpleNamespace(object_xy=boom), "Car_10", None, HOME, anchor)
    for _ in range(pn.TRUTH_MAX_ERRORS + pn.TRUTH_RETRY_TICKS):
        assert tr.check_frame(st) is None
    fs = tr.frame_summary()
    assert fs["agrees"] is None and "TimeoutError" in fs["why"], fs
    # After TRUTH_MAX_ERRORS failures the next TRUTH_RETRY_TICKS checks are
    # skipped, not each paid for.
    assert fs["attempts"] == fs["failures"] == pn.TRUTH_MAX_ERRORS and fs["backoffs"] == 1, fs
    assert tr.check_frame(st) is None and tr.frame_summary()["attempts"] == pn.TRUTH_MAX_ERRORS + 1
    tr = pn.SubjectTruth(SimpleNamespace(object_xy=lambda n: (NAN, NAN)), "Car_10",
                         None, HOME, anchor)
    assert tr.check_frame(st) is None
    assert tr.frame_summary()["failures"] == 1 and "NaN" in tr.frame_summary()["why"]
    # Only the EKF against the kinematics: partial, never "agrees".
    tr.note_ekf_vs_sim(0.3)
    fs = tr.frame_summary()
    assert fs["partial"] is True and fs["checked"] is False and fs["agrees"] is None, fs
    assert "does not test the frame of get_object_poses" in fs["why"]
    tr.note_ekf_vs_sim(9.0)
    tr.note_ekf_vs_sim(9.0)
    assert tr.frame_verdict() == (False, "ekf") and tr.frame_disagrees()
    tr = pn.SubjectTruth(SimpleNamespace(object_xy=lambda n: (35.0, -20.0)),
                         None, "--subject-actor none: the subject's pose is not read",
                         HOME, anchor)
    assert tr.check_frame(st) is None
    assert "--subject-actor none" in tr.frame_summary()["why"]


def test_the_camera_reads_an_actors_position_and_heading_in_one_answer():
    """ProjectAirSimCamera.object_pose: World.get_object_poses([name]), the
    translation and the heading from the rotation (NED, clockwise from north:
    a quaternion about z by +90 deg is east), in the dict form and the
    client's attribute form; no rotation is no heading."""
    c, s = math.cos(math.radians(45)), math.sin(math.radians(45))
    answers = {
        "dict": {"translation": {"x": 35.25, "y": -19.5, "z": -8.0},
                 "rotation": {"w": c, "x": 0.0, "y": 0.0, "z": s}},
        "attr": SimpleNamespace(translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
                                rotation=SimpleNamespace(w=c, x=0.0, y=0.0, z=-s)),
        "bare": {"translation": {"x": 3.0, "y": 4.0, "z": 0.0}},
        "nan": {"translation": {"x": NAN, "y": NAN, "z": NAN},
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}},
    }
    asked = []

    class World:
        def get_object_poses(self, names):
            asked.append(list(names))
            return [answers[names[0]]]
    cam = pn.ProjectAirSimCamera(T.FakeObs(), {"dir": ".", "scene": "x"})
    cam.world = World()
    x, y, yaw = cam.object_pose("dict")
    assert (x, y) == (35.25, -19.5) and abs(yaw - 90.0) < 1e-9, (x, y, yaw)
    x, y, yaw = cam.object_pose("attr")
    assert (x, y) == (1.0, 2.0) and abs(yaw + 90.0) < 1e-9
    assert cam.object_pose("bare") == (3.0, 4.0, None)
    assert cam.object_xy("dict") == (35.25, -19.5)
    assert all(math.isnan(v) for v in cam.object_xy("nan"))
    assert asked[0] == ["dict"], asked


# =========================================================================== #
# Flights on the stand-in
# =========================================================================== #

def test_a_truth_in_the_shields_frame_is_checked_and_scores():
    """The car 2 m east of the aircraft; the simulator's frame is the EKF's.
    Every TRUTH_FRAME_PERIOD_S the check runs and reads ~0 m, the EKF check
    between them reads ~0 m too, and never both on one tick; the follow
    scores, and the basis says the frame agreed through get_object_poses."""
    rc, run, rows, cam = _fly(_policy())
    assert rc == 0 and rows
    m = _json(run, "metrics.json")
    assert m["frame_anchor"]["source"] == "policy.origin", m["frame_anchor"]
    assert abs(rows[0]["x"] - (35.0 - ORIGIN_IN_SCENE[0])) < 1.0, rows[0]["x"]
    checks = [r["subject_frame_err_m"] for r in rows if "subject_frame_err_m" in r]
    period = round(pn.TRUTH_FRAME_PERIOD_S / pn.TICK)
    assert len(rows) // (period + 1) <= len(checks) <= len(rows) // period + 1, \
        (len(checks), len(rows))
    assert cam.aircraft_reads == len(checks) and max(checks) < 0.01, checks
    assert all(abs(r["subject_frame_yaw_err_deg"]) < 1e-6 for r in rows
               if "subject_frame_err_m" in r)
    st = m["subject_truth"]
    fc = st["frame_check"]
    assert fc["checked"] and fc["agrees"] is True and fc["err_m"]["n"] == len(checks), fc
    assert fc["heading_agrees"] is True and fc["cause"] is None
    # The EKF check ran between the frame checks: as often, never on their tick.
    n_geo = m["ekf_vs_sim_horiz_m"]["n"]
    assert abs(n_geo - len(checks)) <= 1 and fc["ekf_vs_sim_m"]["n"] == n_geo, (n_geo, fc)
    assert all(abs(r["subject_range_m"] - 2.0) < 0.05 for r in rows)
    assert m["follow_scored"] is True and m["frac_within_30m"] == 1.0
    assert m["standoff_basis"].startswith("measured") and "frame agrees" in m["standoff_basis"]
    assert "get_object_poses" in m["standoff_basis"]
    assert st["as_read"] is None


def test_a_truth_in_another_frame_scores_nothing():
    """REGRESSION for the question pas_ap_close_on/off left open, as the
    reviewer posed it: ONLY the level-actor poses sit 123.3 m east of the
    EKF's frame (the brief's offset); the kinematics agree with the EKF. The
    car is really 2 m away, the reader puts it 125.3 m away. Before the frame
    check this flight was scored - the follow 'failed' at 125 m and the
    stand-off 'held' - off a truth in the wrong place, and the rows carried
    that place to every offline scorer. A check on the kinematics passed it
    as agreeing."""
    rc, run, rows, cam = _fly(_policy(), camera_kw={"frame_dy": BRIEF_OFFSET_M})
    assert rc == 0 and rows
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    st = m["subject_truth"]
    assert m["ekf_vs_sim_horiz_m"]["median"] < 0.01, "the kinematics agree with the EKF"
    assert m["follow_scored"] is False and st["scored"] is False, \
        (m["follow_scored"], m["frac_within_30m"], m["standoff_s"], m["standoff_basis"])
    assert m["frac_within_30m"] is None and m["standoff_s"] is None
    fc = st["frame_check"]
    assert fc["checked"] and fc["agrees"] is False and fc["cause"] == "frame", fc
    assert abs(fc["err_m"]["median"] - BRIEF_OFFSET_M) < 0.1, fc["err_m"]
    # The rows keep the pose as read, out of every field scorers read.
    assert st["rows_quarantined"] == len(rows)
    for r in rows:
        assert r["subject_truth"] is None and "frame disagrees" in r["subject_truth_why"], r
        assert not any(k in r for k in TRUTH_DERIVED), [k for k in TRUTH_DERIVED if k in r]
        assert set(r["subject_truth_as_read"]) == {"subject_truth", *TRUTH_DERIVED}, \
            sorted(r["subject_truth_as_read"])
    assert all(abs(r["subject_truth_as_read"]["subject_range_m"] - (BRIEF_OFFSET_M + 2.0))
               < 0.1 for r in rows), rows[0]["subject_truth_as_read"]
    assert m["standoff_min_range_m"] is None and st["min_range_m"] is None
    assert st["ticks_in_ring"] is None and st["view"] is None
    assert st["estimate_gap_m"]["n"] == 0 and st["as_read"]["estimate_gap_m"]["n"] == len(rows)
    assert "disagrees" in m["standoff_basis"] and "get_object_poses" in m["standoff_basis"]
    assert abs(st["as_read"]["min_range_m"] - (BRIEF_OFFSET_M + 2.0)) < 0.1, st["as_read"]
    assert any("frame disagrees" in w for w in kpi["kpi_grade_reasons"]), kpi["kpi_grade_reasons"]


def test_a_few_metres_of_frame_offset_quarantine_and_less_than_a_metre_does_not():
    """The limit at flight level: 0.6 m (the stored flights' EKF error p95)
    scores; 3 m does not."""
    rc, run, rows, _ = _fly(_policy(), camera_kw={"frame_dy": 0.6})
    m = _json(run, "metrics.json")
    assert m["subject_truth"]["frame_check"]["agrees"] is True and m["follow_scored"] is True
    rc, run, rows, _ = _fly(_policy(), camera_kw={"frame_dy": 3.0})
    m = _json(run, "metrics.json")
    st = m["subject_truth"]
    assert st["frame_check"]["agrees"] is False and m["follow_scored"] is False, st["frame_check"]
    assert st["rows_quarantined"] == len(rows) and st["as_read"] is not None


def test_an_ekf_off_the_simulator_is_named_not_blamed_on_the_frame():
    """The simulator has the aircraft 5 m north of where the EKF puts it,
    in every read: the truth's ranges (from the EKF) are off, so nothing
    scores - and the cause is the EKF, said so."""
    rc, run, rows, _ = _fly(_policy(), camera_kw={"sim_dx": 5.0})
    m, kpi = _json(run, "metrics.json"), _json(run, "kpi.json")
    st = m["subject_truth"]
    fc = st["frame_check"]
    assert fc["agrees"] is False and fc["cause"] == "ekf", fc
    assert abs(m["ekf_vs_sim_horiz_m"]["median"] - 5.0) < 0.05
    assert m["follow_scored"] is False and st["rows_quarantined"] == len(rows)
    assert "EKF position is off the simulator's" in m["standoff_basis"], m["standoff_basis"]
    assert "frame disagrees" not in m["standoff_basis"]
    assert any("EKF position is off" in w for w in kpi["kpi_grade_reasons"])


def test_without_the_aircraft_actor_the_frame_is_partly_or_not_checked():
    """The likeliest first-flight case: the simulator finds no actor matching
    pn.AIRCRAFT_ACTOR (NaN), or the read fails. The follow scores as before;
    the frame is only PARTLY checked (the EKF against the kinematics) or,
    without that, NOT checked - never "agrees" - and the aircraft read backs
    off instead of being paid for every 0.5 s."""
    rc, run, rows, cam = _fly(_policy(), camera_kw={"aircraft": "missing"}, max_s="6")
    m = _json(run, "metrics.json")
    fc = m["subject_truth"]["frame_check"]
    assert fc["checked"] is False and fc["partial"] is True and fc["agrees"] is None, fc
    assert "NaN" in fc["why"] and "kinematics" in fc["why"]
    assert m["follow_scored"] is True and m["frac_within_30m"] == 1.0
    assert "only PARTLY checked" in m["standoff_basis"], m["standoff_basis"]
    due = len(rows) // round(pn.TRUTH_FRAME_PERIOD_S / pn.TICK)
    assert due > pn.TRUTH_MAX_ERRORS + 2, due
    assert cam.aircraft_reads == fc["attempts"] == pn.TRUTH_MAX_ERRORS, (cam.aircraft_reads, fc)
    assert fc["backoffs"] == 1 and not any("subject_frame_err_m" in r for r in rows)
    # A failing read and no kinematics: NOT checked.
    rc, run, rows, cam = _fly(_policy(), camera_cls=NoGeoCamera,
                              camera_kw={"aircraft": TimeoutError("rpc")})
    m = _json(run, "metrics.json")
    fc = m["subject_truth"]["frame_check"]
    assert fc["partial"] is False and fc["agrees"] is None and "TimeoutError" in fc["why"]
    assert "frame was NOT checked" in m["standoff_basis"] and m["follow_scored"] is True
    assert m["ekf_vs_sim_horiz_m"]["n"] == 0


def test_the_frame_check_runs_after_the_setpoint_goes_out():
    """Like the truth read, the aircraft's read is an RPC on the simulator's
    socket: on every tick that makes it, the tick's setpoint was sent first.
    The EKF-against-kinematics read is the other simulator check: after the
    setpoint too, and never on the same tick as the aircraft's read - not
    even after a 0.7 s stall, when both are overdue at once."""
    calls: list = []
    rc, run, rows, cam = _fly(_policy(), camera_kw={"calls": calls},
                              link_cls=CallLink, link_kw={"calls": calls, "stall": (12, 0.7)})
    assert rc == 0
    ticks, cur = [], None
    for c in calls:
        if c == ("pump",):
            cur = []
            ticks.append(cur)
        elif cur is not None:
            cur.append(c)
    checked = [t for t in ticks if ("pose", AIRCRAFT) in t]
    geo = [t for t in ticks if ("geo",) in t]
    assert len(checked) >= 4 and len(geo) >= 4, (len(checked), len(geo))
    for t in checked:
        assert ("send",) in t and t.index(("send",)) < t.index(("pose", AIRCRAFT)), t
        assert ("geo",) not in t, "both simulator checks on one tick"
    for t in geo:
        assert ("send",) in t and t.index(("send",)) < t.index(("geo",)), t


def test_a_heading_off_the_simulators_withholds_the_view_verdict():
    """The actor's heading 30 deg off the EKF's: where the camera looked is
    then not known, so truth_view's verdict is not given as a finding; the
    positions still score."""
    rc, run, rows, _ = _fly(_policy(), camera_kw={"yaw_off": 30.0})
    m = _json(run, "metrics.json")
    st = m["subject_truth"]
    assert st["frame_check"]["heading_agrees"] is False, st["frame_check"]
    assert abs(st["frame_check"]["yaw_err_deg"]["median"] - 30.0) < 1e-6
    assert st["view"]["verdict"].startswith("not judged: the EKF heading"), st["view"]["verdict"]
    assert m["follow_scored"] is True and st["frame_check"]["agrees"] is True


def test_the_view_says_when_the_estimates_cannot_be_the_subject():
    """The fake detector's box sits right of centre, ranged ~96 m by its
    width. With the car 150 m BEHIND the aircraft no estimate can be of it,
    and the record says so; with the car 50 m ahead it was in view."""
    rc, run, rows, _ = _fly(_policy(), camera_kw={"car": (-150.0, 0.0)})
    assert rc == 0
    est = [r for r in rows if r["subject"] is not None]
    assert est, "the stand-in produced no estimate: the test is blind"
    assert all(abs(abs(r["subject_truth_off_nose_deg"]) - 180.0) < 15.0 for r in est), \
        [r["subject_truth_off_nose_deg"] for r in est[:3]]
    assert all(r["subject_bearing_gap_deg"] > 120.0 for r in est)
    v = _json(run, "metrics.json")["subject_truth"]["view"]
    assert v["estimate_ticks"] == len(est) and v["estimate_ticks_truth_in_view"] == 0, v
    assert v["estimate_ticks_truth_maybe_in_view"] == 0
    assert v["estimate_on_subject"] == {"near_m": 6.0, "ticks": 0, "frac": 0.0}, \
        v["estimate_on_subject"]
    assert v["verdict"].endswith("every estimate the Shield flew on was of something else"), \
        v["verdict"]
    assert abs(v["truth_range_m"]["median"] - 150.0) < 5.0, v["truth_range_m"]
    rc, run, rows, _ = _fly(_policy(), camera_kw={"car": (50.0, 0.0)})
    v = _json(run, "metrics.json")["subject_truth"]["view"]
    assert v["estimate_ticks"] > 0
    assert v["estimate_ticks_truth_in_view"] == v["estimate_ticks"], v
    assert "something else" not in v["verdict"] and "in view the estimate's bearing" in v["verdict"]


def test_the_rows_carry_angles_wrapped_across_the_seam():
    """Facing south (yaw 180): the car at bearing -170 is 10 deg off the
    nose, not -350; the estimate (right of centre, bearing about -153) is
    about 37 deg from a car at bearing +170, not 323."""
    def car_at(brg_deg, d=50.0):
        return (d * math.cos(math.radians(brg_deg)), d * math.sin(math.radians(brg_deg)))
    rc, run, rows, _ = _fly(_policy(), camera_kw={"car": car_at(-170.0)},
                            link_kw={"yaw": math.pi})
    assert rc == 0 and rows and rows[0]["yaw_deg"] == 180.0
    offs = [r["subject_truth_off_nose_deg"] for r in rows]
    assert all(abs(o - 10.0) < 1.0 for o in offs), offs[:3]
    rc, run, rows, _ = _fly(_policy(), camera_kw={"car": car_at(170.0)},
                            link_kw={"yaw": math.pi})
    gaps = [r["subject_bearing_gap_deg"] for r in rows if "subject_bearing_gap_deg" in r]
    assert gaps and all(30.0 < g < 45.0 for g in gaps), gaps[:3]
    assert all(-180.0 <= r["subject_truth_off_nose_deg"] <= 180.0 for r in rows)


def test_the_screen_shows_no_true_range_from_a_truth_in_another_frame():
    """--save-view: the HUD's separation is the TRUE range (RailHud `sep`).
    From a truth the frame check has put in another frame it is blank; from
    one in the Shield's frame it is the 2 m it should be."""
    import test_railvideo_recording as RV
    RV.SpyRecorder.made = []
    rc, run, rows, _ = _fly(_policy(), camera_kw={"frame_dy": BRIEF_OFFSET_M},
                            extra=("--save-view",), recorder_cls=RV.SpyRecorder)
    huds = RV.SpyRecorder.made[-1].huds
    assert rc == 0 and len(huds) == len(rows), (rc, len(huds), len(rows))
    assert all(h["sep"] is None for h in huds), [h["sep"] for h in huds[:3]]
    rc, run, rows, _ = _fly(_policy(), extra=("--save-view",), recorder_cls=RV.SpyRecorder)
    huds = RV.SpyRecorder.made[-1].huds
    assert huds and all(abs(h["sep"] - 2.0) < 0.05 for h in huds), [h["sep"] for h in huds[:3]]


# =========================================================================== #
# truth_view over synthetic rows (pure)
# =========================================================================== #

def _row(t, seq, yaw, truth, est=None, x=0.0, y=0.0):
    return {"t": t, "det_seq": seq, "x": x, "y": y, "yaw_deg": yaw,
            "subject_truth": None if truth is None else list(truth),
            "subject": None if est is None else [est[0], est[1], "width"]}


def _at(brg_deg, d):
    return (d * math.cos(math.radians(brg_deg)), d * math.sin(math.radians(brg_deg)))


def test_an_estimate_on_the_subject_is_counted_within_six_metres():
    """ESTIMATE_NEAR_M is track_truth.score_estimate's near_m (6.0). Of four
    estimates, those 3 m and 5.9 m from the truth count; 6.1 m and 7 m do
    not; and with one on the subject the verdict never says 'something
    else', even with the subject out of view."""
    import track_truth
    assert pn.ESTIMATE_NEAR_M == \
        inspect.signature(track_truth.score_estimate).parameters["near_m"].default == 6.0
    truth = (50.0, 0.0)
    rows = [_row(0.1 * i, i, 0.0, truth, est) for i, est in enumerate(
        [(53.0, 0.0), (50.0, 5.9), (50.0, -6.1), (57.0, 0.0)])]
    v = pn.truth_view(rows, 90.0, 4.0)
    assert v["estimate_on_subject"] == {"near_m": 6.0, "ticks": 2, "frac": 0.5}, v
    assert "2 of the 4 estimates were within 6 m" in v["verdict"], v["verdict"]
    behind = (-50.0, 0.0)
    rows = [_row(0.1 * i, i, 0.0, behind, (behind[0] - 3.0, 0.0)) for i in range(4)]
    v = pn.truth_view(rows, 90.0)
    assert v["estimate_ticks_truth_maybe_in_view"] == 0 and v["estimate_on_subject"]["ticks"] == 4
    assert "something else" not in v["verdict"], v["verdict"]


def test_the_view_wraps_bearings_across_the_seam():
    """Heading 179 deg, the subject at bearing -179: 2 deg off the nose (in
    view), and an estimate at bearing 178 is 3 deg from it, not 357."""
    rows = [_row(0.0, 1, 179.0, _at(-179.0, 50.0), _at(178.0, 40.0))]
    v = pn.truth_view(rows, 90.0)
    assert v["estimate_ticks_truth_in_view"] == 1, v
    assert abs(v["truth_off_nose_deg"]["median"] - 2.0) < 0.01, v["truth_off_nose_deg"]
    assert abs(v["bearing_gap_deg"]["median"] - 3.0) < 0.01, v["bearing_gap_deg"]
    assert pn._wrap_deg(190.0) == -170.0 and pn._wrap_deg(-190.0) == 170.0


def test_in_view_is_counted_over_the_estimate_ticks_only():
    """The subject ahead on the first ticks (no estimate then) and behind on
    the later ones (with estimates, seconds later): in view on 3 ticks, on
    none of the estimate ticks."""
    ahead, behind = _at(0.0, 50.0), _at(180.0, 50.0)
    rows = ([_row(0.1 * i, i + 1, 0.0, ahead) for i in range(3)]
            + [_row(5.0 + 0.1 * i, 50 + i, 0.0, behind, _at(20.0, 60.0)) for i in range(3)])
    v = pn.truth_view(rows, 90.0)
    assert v["truth_in_view_ticks"] == 3 and v["ticks_truth_read"] == 6, v
    assert v["estimate_ticks"] == 3 and v["estimate_ticks_truth_in_view"] == 0
    assert v["estimate_ticks_truth_maybe_in_view"] == 0, v
    assert v["verdict"].endswith("something else"), v["verdict"]


def test_in_view_allows_for_the_heading_the_frame_was_taken_at():
    """The aircraft turned 90 deg after the frame was taken: at the tick's
    heading the subject is out of view, at the capture heading (the previous
    det_seq's first row, one row back) it was in view. The verdict then does
    not call the estimate something else; the window reaches back at most
    DETECTION_STALE_S plus one inference."""
    ahead = _at(0.0, 50.0)
    rows = [_row(0.0, 1, 0.0, ahead), _row(0.1, 1, 90.0, ahead),
            _row(0.2, 2, 90.0, ahead, _at(90.0, 40.0))]
    assert pn.capture_window_start(rows, 2) == 0
    v = pn.truth_view(rows, 90.0)
    assert v["estimate_ticks_truth_in_view"] == 0 and v["estimate_ticks_truth_maybe_in_view"] == 1
    assert "something else" not in v["verdict"] and "on 1 at any heading" in v["verdict"]
    # The box was found up to DETECTION_STALE_S before the row (a later
    # inference may have found nothing): the window reaches that far, then
    # back to the inference before - and not across a gap in the rows.
    # Rows every 0.1 s, a new inference every third row: row 29 (t 2.9 s)
    # reaches back to the rows within 1.0 s (from t 1.9-2.0, det_seq 16),
    # then to the first row of det_seq 15 (row 15), then one row more.
    rows = ([_row(0.1 * k, 10 + k // 3, 0.0, ahead) for k in range(30)])
    lo = pn.capture_window_start(rows, 29)
    assert lo == 14 and rows[lo]["t"] <= rows[29]["t"] - pn.DETECTION_STALE_S, (lo, rows[lo])
    gap = [_row(0.0, 1, 0.0, ahead), _row(5.0, 5, 90.0, ahead),
           _row(5.1, 6, 90.0, ahead, _at(90.0, 40.0))]
    assert pn.capture_window_start(gap, 2) == 1
    v = pn.truth_view(gap, 90.0)
    assert v["estimate_ticks_truth_maybe_in_view"] == 0 and v["verdict"].endswith("something else")


# =========================================================================== #
# The stored flights
# =========================================================================== #

def _hypotheses(rows: list, shuffles: int = 200, seed: int = 7) -> dict:
    """The frame corrections tried on a stored flight, each scored as the
    median distance between the estimate the Shield flew on and the truth,
    over the ticks that carry both (x North, y East, bearings clockwise from
    north):
      as_logged       as flown;
      east_negated    the truth's y sign flipped (NED read as NWU);
      axes_swapped    the truth's x and y exchanged (NED read as ENU);
      heading_enu     the estimate re-placed as if the heading were ENU
                      (90 - yaw): rotated about the aircraft by 90 - 2 yaw;
      image_mirrored  the estimate's bearing off the nose negated (a
                      left-right flipped image): placed at 2 yaw - bearing;
      offset_fit      the truth minus the component-wise median of
                      (truth - estimate), i.e. the best constant offset;
      offset_null     the same fit with the truth rows shuffled (seeded,
                      `shuffles` times; the median of the fits): what a
                      constant offset buys with NO relation between the two;
      best_rotation   the estimates rotated about the aircraft by the angle
                      (1 deg steps) that brings them nearest."""
    E = [r for r in rows if r.get("subject") and r.get("subject_truth")]
    A = [(r["x"], r["y"]) for r in E]
    S = [(r["subject"][0], r["subject"][1]) for r in E]
    Tt = [(r["subject_truth"][0], r["subject_truth"][1]) for r in E]
    yaw = [math.radians(r["yaw_deg"]) for r in E]

    def med(vals):
        vals = sorted(vals)
        n = len(vals)
        return vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])

    def gap(ss, tt):
        return med([math.hypot(s[0] - t[0], s[1] - t[1]) for s, t in zip(ss, tt)])

    def rot(a, s, ang):
        dx, dy = s[0] - a[0], s[1] - a[1]
        return (a[0] + dx * math.cos(ang) - dy * math.sin(ang),
                a[1] + dx * math.sin(ang) + dy * math.cos(ang))

    def mirror(a, s, ps):
        b = math.atan2(s[1] - a[1], s[0] - a[0])
        d = math.hypot(s[0] - a[0], s[1] - a[1])
        return (a[0] + d * math.cos(2 * ps - b), a[1] + d * math.sin(2 * ps - b))

    def fit(tt):
        dx = med([t[0] - s[0] for s, t in zip(S, tt)])
        dy = med([t[1] - s[1] for s, t in zip(S, tt)])
        return gap([(s[0] + dx, s[1] + dy) for s in S], tt)
    rng = random.Random(seed)
    null = []
    for _ in range(shuffles):
        tt = list(Tt)
        rng.shuffle(tt)
        null.append(fit(tt))
    best = min((gap([rot(a, s, math.radians(th)) for a, s in zip(A, S)], Tt), th)
               for th in range(360))
    return {"n": len(E), "as_logged": gap(S, Tt),
            "east_negated": gap(S, [(t[0], -t[1]) for t in Tt]),
            "axes_swapped": gap(S, [(t[1], t[0]) for t in Tt]),
            "heading_enu": gap([rot(a, s, math.pi / 2 - 2 * p) for a, s, p in zip(A, S, yaw)], Tt),
            "image_mirrored": gap([mirror(a, s, p) for a, s, p in zip(A, S, yaw)], Tt),
            "offset_fit": fit(Tt), "offset_null": med(null),
            "best_rotation": best[0], "best_rotation_deg": best[1]}


def _box_widths(rows: list, dets: list) -> dict:
    """The detected box widths in FrontCamera pixels: the median over every
    box found (detections.jsonl rows with a det), and over the estimate
    ticks, each taking the latest box found at or before the tick's det_seq
    (the box FollowPilot steered on)."""
    import bisect
    found = sorted((d["seq"], d["det"]["w"], d["det"]["score"]) for d in dets if d.get("det"))
    seqs = [f[0] for f in found]
    per_tick = []
    for r in rows:
        if r.get("subject") is not None and r.get("det_seq") is not None:
            k = bisect.bisect_right(seqs, r["det_seq"]) - 1
            if k >= 0:
                per_tick.append(found[k][1])

    def med(v):
        v = sorted(v)
        return v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2])
    return {"boxes_found": len(found), "w_per_box_px": med([f[1] for f in found]),
            "w_per_estimate_tick_px": med(per_tick),
            "score_per_box": med([f[2] for f in found]),
            "score_max": max(f[2] for f in found)}


def test_the_stored_close_approach_flights_followed_something_else():
    """The evidence, re-read from the stored logs (demo/out is gitignored:
    SKIP where they are absent). The truth is Car_10 on the track follow_vlm
    recorded in the same level; the detector's estimates were never on it,
    and no frame correction puts them there. Prints the hypothesis table and
    the box widths with the methods above."""
    need = [OUT / t / "flight_log.jsonl" for t in ("pas_ap_close_on", "pas_ap_close_off")]
    if not all(p.is_file() for p in need):
        print("      (stored flights pas_ap_close_on/off not in demo/out)")
        return SKIP
    width_m = pn._fv().subject_width("a red car")[0]
    # (estimate ticks, in view at the tick's heading, in view at any capture
    # heading, ticks of the whole flight with the car in view)
    want = {"pas_ap_close_on": (149, 10, 19, 47), "pas_ap_close_off": (145, 0, 0, 0)}
    fv_log = OUT / "citylife_redcar_nfz_20261013" / "flight_log.jsonl"
    track = None
    if fv_log.is_file():
        track = [r["truth"]["pts"][0] for r in map(json.loads, fv_log.read_text().splitlines())
                 if (r.get("truth") or {}).get("pts")]
    for p in need:
        rows = [json.loads(x) for x in p.read_text().splitlines()]
        v = pn.truth_view(rows, 90.0, width_m)
        got = (v["estimate_ticks"], v["estimate_ticks_truth_in_view"],
               v["estimate_ticks_truth_maybe_in_view"], v["truth_in_view_ticks"])
        assert got == want[p.parent.name], (p.parent.name, got)
        assert v["estimate_on_subject"]["ticks"] == 0, v["estimate_on_subject"]
        assert v["truth_range_m"]["median"] > 3 * v["estimate_range_m"]["median"], v
        if track is not None:
            worst = max(min(math.hypot(r["subject_truth"][0] - a, r["subject_truth"][1] - b)
                            for a, b in track)
                        for r in rows[::25] if r.get("subject_truth"))
            assert worst < 2.0, (p.parent.name, worst)
        # The heading at the first mission tick against the spawn's rpy-deg
        # "0 0 0" (scene_citylife_ardupilot.jsonc): no heading-frame offset.
        assert abs(rows[0]["yaw_deg"]) < 1.0, rows[0]["yaw_deg"]
        h = _hypotheses(rows)
        print(f"      {p.parent.name}: n {h['n']}; median estimate-truth gap, m: "
              + ", ".join(f"{k} {h[k]:.1f}" for k in (
                  "as_logged", "east_negated", "axes_swapped", "heading_enu",
                  "image_mirrored", "offset_fit", "offset_null", "best_rotation"))
              + f" (at {h['best_rotation_deg']} deg)")
        # No correction brings the estimates within the follow range of the
        # truth, and the best constant offset is no better than shuffled rows.
        assert min(h[k] for k in ("east_negated", "axes_swapped", "heading_enu",
                                  "image_mirrored", "offset_fit", "best_rotation")) \
            > pn.FOLLOW_RANGE_M, h
        assert h["offset_fit"] > 0.9 * h["offset_null"], h
        dets = p.parent / "detections.jsonl"
        if dets.is_file():
            b = _box_widths(rows, [json.loads(x) for x in dets.read_text().splitlines()])
            print(f"      {p.parent.name}: boxes {b['boxes_found']}, width median "
                  f"{b['w_per_box_px']} px per box / {b['w_per_estimate_tick_px']} px per "
                  f"estimate tick; score median {b['score_per_box']}, max {b['score_max']}; "
                  f"Car_10 at its true range spans a median "
                  f"{v['truth_width_deg']['median']} deg")


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
        except (Exception, SystemExit) as e:          # noqa: BLE001  (argparse exits)
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
