"""The Shield's own-velocity forecast (docs/DESIGN-own-velocity-forecast.md).

Until 2026-10-19 every forecast pose started from the COMMANDED velocity:
`guardrail.models.State` had no velocity, and `Shield._predict` advanced the
candidate action at constant velocity from t = 0. On the 14 Oct ArduPilot
close approach (demo/out/pas_ap_video_on_20261014) the aircraft was inside the
P0 building margin on 71 of 517 ticks, up to 1.0 m deep, because the Shield
believed a command of +0.49 m/s away from the wall moved the aircraft away
from it at once, while ArduCopter kept flying ~0.4 s on its old velocity and
then braked at under 3 m/s^2.

This file holds:

  * the IDENTITY proof: with no measured velocity (the default, and every rail
    that does not pass one) every filter() decision is byte-identical to the
    code before the change. The digests in tests/fixtures/
    own_velocity_identity_digests.json were computed on c77c77d, before a line
    of the change existed (with the escalation FSM's hold-point fields left
    out of each dump, HOLD_POINT_FIELDS: they came with the hold points, a
    separate change merged on 2026-10-21);
  * the response model's own tests (guardrail/response.py);
  * the 14 Oct replay: the tick at which the new forecast first flags the
    building margin, against the old one, and the stop-point counterfactual;
  * the offline closed loop: the measured response model as the aircraft,
    the Shield in the loop, old forecast against new;
  * the hold that stops clear (BRAKE's predicted stop point against the
    margin), and what the rail records of it: the stop-advised ticks' audit
    records, read back by tools/export_episodes as one flight; the P0
    escapes on the commanded basis, re-derived row by row.

Run:  python tests/test_own_velocity_forecast.py [--full] [--regenerate]
"""
import hashlib
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "demo", ROOT / "demo" / "pas_ardupilot"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np                                                      # noqa: E402

from guardrail import load_policy                                      # noqa: E402
from guardrail.models import Action4D, ObstacleClearance, State        # noqa: E402
from guardrail.shield import Shield                                    # noqa: E402

FIXTURES = ROOT / "tests" / "fixtures"
IDENTITY = FIXTURES / "own_velocity_identity_digests.json"
CITY_MAP = FIXTURES / "citylife_occ_flightband_6to14.npz"
SKIP = "SKIP"

# The identity sample. The policies cover every rule family the Shield
# forecasts: stand-off and building clearance (the 14 Oct policy), a keep-out
# zone beside them, a corridor, an altitude-heavy pedestrian policy, and the
# 21-rule hard policy.
IDENTITY_POLICIES = ("follow_car_citylife.yaml", "follow_car_citylife_nfz.yaml",
                     "corridor_survey.yaml", "sitl_pedestrian.yaml",
                     "follow_car_gap.yaml", "hard_21.yaml")
IDENTITY_SEED = 20261019
N_CI = 500            # per policy, per Shield configuration: 6000 decisions
N_FULL = 1500         # 18,000 decisions
# The commit before State had a velocity: the identity digests are ITS
# decisions (`--regenerate` computes them on a git archive of it).
IDENTITY_BASE = "c77c77d"
# The escalation FSM's hold point (guardrail/holdpoint.py, merged on
# 2026-10-21) adds these keys to the fsm_record of a Loiter / Land decision
# and changes nothing else a decision dumps ("No edge, guard or setpoint
# moved", docs/DESIGN-hold-points.md). They are left out of every dump the
# identity digests hash, so the proof stays about the velocity and stays on
# c77c77d. Evidence: with them left out, the six escalation=True streams of
# the merged tree hash exactly to c77c77d's digests (ci, 2026-10-21); with
# them in, all six moved - first by the hold points themselves, then again
# by the search's later refinements (its reach and reason text), which is
# why a proof rebased on the hold-point branch's tip (6c7cdb0, done at the
# merge) could not survive the next change to the search. At c77c77d the
# keys do not exist, so `--regenerate` gives the digests it always gave.
HOLD_POINT_FIELDS = ("hold_point", "hold_legal", "hold_needed", "hold_chosen")


def _city_map():
    with np.load(CITY_MAP) as z:
        return {"occ": z["occ"], "res": float(z["res"]), "ox": float(z["ox"]),
                "oy": float(z["oy"])}


def _identity_lines(name: str, n: int, *, escalation: bool, velocity: str = "none"):
    """`n` seeded filter() decisions on policy `name`, one JSON line each.

    The sampling IS the fixture: change it and every digest changes.
    `velocity`: "none" builds every State without a velocity (the only form
    that existed at c77c77d), "explicit_none" passes velocity=None, and
    "ignored" passes a measured velocity to a Shield built with
    velocity_response=None, which must ignore it."""
    pol = load_policy(ROOT / "policies" / name)
    smap = _city_map() if pol.by_type(ObstacleClearance) else None
    kw = {}
    if velocity == "ignored":
        kw["velocity_response"] = None
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=smap,
                escalation=escalation, **kw)
    rng = random.Random(f"{IDENTITY_SEED}:{name}:{escalation}")
    out = []
    t = 0.0
    for i in range(n):
        t += 0.1
        x, y = rng.uniform(-60.0, 120.0), rng.uniform(-55.0, 120.0)
        up = rng.uniform(3.0, 22.0)
        if i % 3 == 0:
            sh.set_subject(x + rng.uniform(-14, 14), y + rng.uniform(-14, 14),
                           rng.choice(["pedestrian", "car", "thing"]))
        elif i % 3 == 1:
            sh.set_subject(None)
        vel = (rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-1.5, 1.5))
        if velocity == "none":
            st = State(x=x, y=y, up=up, yaw_deg=rng.uniform(-180, 180))
        elif velocity == "explicit_none":
            st = State(x=x, y=y, up=up, yaw_deg=rng.uniform(-180, 180), velocity=None)
        else:
            st = State(x=x, y=y, up=up, yaw_deg=rng.uniform(-180, 180), velocity=vel)
        a = Action4D(vx=rng.uniform(-6, 6), vy=rng.uniform(-6, 6),
                     vz_up=rng.uniform(-2.5, 2.5), yaw_rate=rng.uniform(-1.0, 1.0))
        d = sh.filter(st, a, t=t)
        dump = d.model_dump(mode="json")
        if isinstance(dump.get("fsm_record"), dict):
            for k in HOLD_POINT_FIELDS:
                dump["fsm_record"].pop(k, None)
        out.append(json.dumps(dump, sort_keys=True, separators=(",", ":")))
    return out


def _identity_digests(n: int, velocity: str = "none") -> dict:
    got = {}
    for name in IDENTITY_POLICIES:
        for esc in (False, True):
            lines = _identity_lines(name, n, escalation=esc, velocity=velocity)
            got[f"{name}|escalation={esc}"] = hashlib.sha256(
                "\n".join(lines).encode()).hexdigest()
    return got


def _check_identity(n_key: str, velocity: str):
    spec = json.loads(IDENTITY.read_text(encoding="utf-8"))
    # The baseline must be the base commit's own decisions, stamped by
    # `--regenerate` (which runs on a git archive of it), never a later
    # commit's output under the base's name.
    assert str(spec.get("computed_on", "")).startswith(IDENTITY_BASE), (
        f"the identity fixture says computed_on={spec.get('computed_on')!r}, not "
        f"{IDENTITY_BASE}: regenerate it with --regenerate")
    want = spec[n_key]
    got = _identity_digests(spec[n_key + "_n"], velocity=velocity)
    moved = sorted(k for k in want if got.get(k) != want[k])
    assert not moved, (f"{len(moved)} of {len(want)} decision streams moved with "
                       f"velocity={velocity!r}: {moved}")
    return sum(1 for _ in want) * spec[n_key + "_n"]


def test_decisions_without_a_velocity_are_byte_identical_to_c77c77d():
    """6000 seeded filter() decisions (12 streams of 500: six policies, with
    and without the Shield's own escalation FSM) hash exactly as they did on
    c77c77d, the commit before State had a velocity. `--full` runs the
    18,000-decision version of the same proof (1500 per stream)."""
    n = _check_identity("ci", "none")
    assert n == 6000, n


def test_an_explicit_none_and_an_ignored_velocity_change_nothing():
    """velocity=None written out is the default; a measured velocity given to
    a Shield built with velocity_response=None is ignored. Both reproduce the
    c77c77d digests."""
    _check_identity("ci", "explicit_none")
    _check_identity("ci", "ignored")


MAIN_OUT = ROOT / "demo" / "out"
APPROACH = FIXTURES / "pas_ap_close_approaches.slim.jsonl.gz"
APPROACH_RUNS = ("pas_ap_video_on_20261014", "pas_ap_close_on")
APPROACH_FIELDS = ("tick", "t", "x", "y", "up", "yaw_deg", "raw", "emitted", "flown_action",
                   "setpoint", "subject", "ap_mode", "unsafe", "unsafe_rules", "unsafe_depth_m",
                   "fsm_edge", "fsm_state_after", "set_mode", "autopilot_set_mode")


def _slim_approach(run: str, r: dict) -> dict:
    out = {"run": run, **{k: r.get(k) for k in APPROACH_FIELDS}}
    out["violations"] = [[v["rule_id"], v.get("predicted_at_s")] for v in r.get("violations", [])]
    out["repairs"] = [p["operator"] for p in r.get("repairs", [])]
    return out


def regenerate_approaches(out_root: Path) -> Path:
    """The two recorded close approaches, trimmed to what the replay and the
    closed loop read, with their policies (one line each, first)."""
    import gzip
    import io
    lines = []
    for run in APPROACH_RUNS:
        folder = out_root / run
        pol = json.loads((folder / "policy_g0.json").read_text(encoding="utf-8"))
        lines.append(json.dumps({"run": run, "policy": pol}, separators=(",", ":")))
        for l in (folder / "flight_log.jsonl").read_text(encoding="utf-8").splitlines():
            if l.strip():
                lines.append(json.dumps(_slim_approach(run, json.loads(l)), separators=(",", ":")))
    buf = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=buf, mtime=0) as gz:
        gz.write(("\n".join(lines) + "\n").encode("utf-8"))
    APPROACH.write_bytes(buf.getvalue())
    return APPROACH


def test_the_identity_sample_exercises_the_shield_and_a_velocity_changes_it():
    """A digest over decisions nothing touched proves nothing: the sample
    repairs on a fair share of its ticks, in every rule family the forecast
    feeds. And the null: the same sample WITH a measured velocity (the default
    response model) does not reproduce the digest - the velocity is read."""
    from collections import Counter
    cats, touched, n = Counter(), 0, 0
    for name in IDENTITY_POLICIES:
        for line in _identity_lines(name, 200, escalation=False):
            d = json.loads(line)
            n += 1
            touched += bool(d["violations"] or d["repairs"])
            cats.update(v["category"] for v in d["violations"])
    assert touched > 0.3 * n, (touched, n)
    for need in ("clearance", "standoff", "geofence", "corridor", "altitude"):
        assert cats[need] > 0, (need, cats)
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=_city_map(), escalation=False)
    st = State(x=36.0, y=-48.6, up=8.0, yaw_deg=-90.0)
    a = Action4D(vx=0.0, vy=-3.5)
    assert not sh.filter(st, a, t=0.1).violations, "commanded forecast: clean"
    d = sh.filter(st.model_copy(update={"velocity": (0.0, -3.7, 0.0)}), a, t=0.2)
    assert [v.rule_id for v in d.violations] == ["bld-clearance"], d.violations


# --------------------------------------------------------------------------- #
# The response model (guardrail/response.py)
# --------------------------------------------------------------------------- #

def test_the_closed_form_is_the_integral_of_the_rate_limited_response():
    """`integrate` (and the Shield's per-pose form) against a 1 ms Euler
    integration of the same rule, through a pipe, a command and a stop."""
    from guardrail import response as R
    segs = [(0.12, 0.5, -3.0), (0.23, None, None), (0.4, 2.0, 1.0), (math.inf, 0.0, 0.0)]
    times = [0.0, 0.05, 0.12, 0.3, 0.35, 0.6, 0.75, 1.5, 3.0]
    got = R.integrate(-1.0, -3.2, segs, times, 2.0)
    x = y = 0.0
    vx, vy = -1.0, -3.2
    t, h, k, start = 0.0, 1e-4, 0, 0.0
    ref = {}
    for tt in times:
        while t < tt - 1e-12:
            while k < len(segs) and t >= start + segs[k][0] - 1e-12:
                start += segs[k][0]
                k += 1
            cx, cy = segs[k][1], segs[k][2]
            if cx is not None:
                dx, dy = cx - vx, cy - vy
                m = math.hypot(dx, dy)
                if m > 2.0 * h:
                    dx, dy = dx * 2.0 * h / m, dy * 2.0 * h / m
                vx, vy = vx + dx, vy + dy
            x, y, t = x + vx * h, y + vy * h, t + h
        ref[tt] = (x, y, vx, vy)
    for tt, g in zip(times, got):
        assert all(abs(a - b) < 2e-3 for a, b in zip(g, ref[tt])), (tt, g, ref[tt])
    m = R.ARDUCOPTER_GUIDED
    dx, dy = m.displacement(3.2, 0.0, 0.0, 0.0, m.stop_time(3.2))
    assert abs(dx - m.stop_distance(3.2)) < 1e-9 and abs(dy) < 1e-12
    assert abs(m.stop_distance(3.2)
               - (3.2 * m.reaction_s + 3.2 ** 2 / (2.0 * m.accel_mps2))) < 1e-9


def test_the_pipe_is_what_was_sent_in_the_last_reaction_time():
    """pipe_segments: a setpoint sent at t acts from t + reaction_s; the one
    acting now is the last sent at least reaction_s ago; a tick that sent
    nothing coasts; anything sent at or after now is not in it."""
    from guardrail import response as R
    flown = [(9.0, 1.0, 0.0), (9.6, 2.0, 0.0), (9.8, None, None), (9.9, 3.0, 0.0),
             (10.0, 9.0, 9.0)]
    segs = R.pipe_segments(flown, 10.0, 0.35)
    # acting at 0: sent 9.6 (9.6 + 0.35 <= 10); 9.8's "nothing" from 0.15; 9.9's from 0.25.
    want = [(0.15, 2.0, 0.0), (0.10, None, None), (0.10, 3.0, 0.0)]
    assert len(segs) == 3, segs
    for (d, cx, cy), (wd, wx, wy) in zip(segs, want):
        assert abs(d - wd) < 1e-9 and cx == wx and cy == wy, segs
    assert abs(sum(s[0] for s in segs) - 0.35) < 1e-9
    assert R.pipe_segments([], 10.0, 0.35) == [(0.35, None, None)]


def test_the_shield_forecast_flies_the_pipe_then_the_command():
    """Shield.forecast with a measured velocity: the first reaction_s are the
    setpoints already sent (note_flown), then the candidate. With no record
    it keeps the measured velocity for the reaction time."""
    from guardrail import response as R
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False)
    st = State(x=0.0, y=0.0, up=8.0, velocity=(0.0, 0.0, 0.0))
    a = Action4D(vx=2.0, vy=0.0)

    def at(fc):
        # forecast times are sums of 0.1 s steps: key them to the millisecond
        return {round(t, 3): p for t, p in fc}
    poses = at(sh.forecast(st, a))
    assert abs(poses[0.3].x) < 1e-12, "no pipe: at rest for the reaction time"
    sh.filter(State(x=0.0, y=0.0, up=8.0), Action4D(), t=1.0)
    sh.note_flown(Action4D(vx=4.0, vy=0.0), 1.0)
    sh.filter(State(x=0.0, y=0.0, up=8.0), Action4D(), t=1.1)
    sh.note_flown(Action4D(vx=4.0, vy=0.0), 1.1)
    sh.filter(State(x=0.0, y=0.0, up=8.0), Action4D(), t=1.2)
    fc = list(sh.forecast(st, a))
    poses = at(fc)
    ts = [fc[3][0], fc[10][0], fc[50][0]]
    m = R.ARDUCOPTER_GUIDED
    segs = R.pipe_segments([(1.0, 4.0, 0.0), (1.1, 4.0, 0.0)], 1.2, m.reaction_s)
    want = R.integrate(0.0, 0.0, segs + [(math.inf, 2.0, 0.0)], ts, m.accel_mps2)
    for t, (dx, _, _, _) in zip(ts, want):
        assert abs(poses[round(t, 3)].x - dx) < 1e-9, (t, poses[round(t, 3)].x, dx)
    assert poses[0.3].x > 0.0, "the 4 m/s already sent accelerates it in the reaction time"


def test_the_measured_velocity_only_adds_what_the_commanded_forecast_refuses_stays_refused():
    """THE MEASURED VELOCITY ONLY ADDS (Shield._predict). 3.7 m off the 14 Oct
    wall, the aircraft drifting AWAY at 0.5 m/s, the pilot commanding 3.2 m/s
    at the wall: the own-velocity "stop after one control period" forecast
    alone is content (the aircraft cannot take up that command within one
    period), the commanded forecast is not. The first build of this branch
    judged the first alone and streamed such commands one tick at a time; in
    the closed loop the old KPI basis counted 6 P0 escapes for it on the
    14 Oct approach. Both are judged now, and what is emitted re-checks clean
    on the commanded forecast too.

    And the property, over a seeded sample around that building and over the
    identity sample's six policies: every decision whose emitted action
    re-checks clean WITH the velocity also re-checks clean on the commanded
    forecast from the position (the P0 escape count's basis on every stored
    run). On the first build 16 of 394 near-building decisions did not."""
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    smap = _city_map()
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=smap, escalation=False)
    here = State(x=36.0, y=-52.0, up=8.0)
    away = here.model_copy(update={"velocity": (0.0, 0.5, 0.0)})
    a = Action4D(vx=0.0, vy=-3.2)
    assert [v.rule_id for v in sh.filter(here, a, t=0.1).violations] == ["bld-clearance"]
    c = sh._clear[0]
    steps, dists = sh._clear_forecast(away, a)
    assert not sh._clearance_verdict(c, steps, dists, sh._mask(c)), \
        "the own-velocity form alone must be content here, or this test proves nothing"
    d = sh.filter(away, a, t=0.2)
    assert [v.rule_id for v in d.violations] == ["bld-clearance"], d.violations
    assert not d.emitted_violations and d.emitted.vy > 0.0, d.emitted
    assert not sh._check(here, d.emitted)

    import random
    checked = 0
    samples = [("follow_car_citylife.yaml", (25.0, 47.0), (-58.0, -42.0), 300)]
    samples += [(name, (-60.0, 120.0), (-55.0, 120.0), 150) for name in IDENTITY_POLICIES]
    for name, (x0, x1), (y0, y1), n in samples:
        pol = load_policy(ROOT / "policies" / name)
        sh = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False,
                    obstacle_map=smap if pol.by_type(ObstacleClearance) else None)
        rng = random.Random(f"20261020:{name}:{n}")
        t = 0.0
        for i in range(n):
            t += 0.1
            x, y, up = rng.uniform(x0, x1), rng.uniform(y0, y1), rng.uniform(4.0, 20.0)
            if i % 2:
                sh.set_subject(x + rng.uniform(-12, 12), y + rng.uniform(-12, 12),
                               rng.choice(["pedestrian", "car"]))
            else:
                sh.set_subject(None)
            if i % 3 == 0:
                sh.note_flown(Action4D(vx=rng.uniform(-4, 4), vy=rng.uniform(-4, 4)), t - 0.05)
            pos = State(x=x, y=y, up=up)
            st = pos.model_copy(update={"velocity": (rng.uniform(-4, 4), rng.uniform(-4, 4), 0.0)})
            d = sh.filter(st, Action4D(vx=rng.uniform(-4, 4), vy=rng.uniform(-4, 4),
                                       vz_up=rng.uniform(-1, 1)), t=t)
            if d.emitted_violations:
                continue
            old = sh._check(pos, d.emitted)
            assert not old, (name, i, st, d.emitted, old)
            checked += 1
    assert checked > 1000, checked


def test_a_position_question_ignores_the_velocity_and_a_nan_one_is_ignored():
    """state_is_unsafe / unsafe_depth / stop_breaks stay about the POSITION
    (the rails' unsafe column, the FSM's stop_illegal); a non-finite velocity
    is not flown on - the commanded forecast flies that tick - and is
    counted once per tick."""
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=_city_map(), escalation=False)
    here = State(x=36.0, y=-52.5, up=8.0)            # 3.2 m from the wall, legal
    fast = here.model_copy(update={"velocity": (0.0, -3.5, 0.0)})
    assert not sh.state_is_unsafe(here) and not sh.state_is_unsafe(fast)
    assert sh.unsafe_depth(fast) == 0.0 and sh.stop_breaks(fast) == []
    inside = State(x=36.2, y=-53.9, up=8.0, velocity=(0.0, -1.0, 0.0))
    assert [v.rule_id for v in sh.state_is_unsafe(inside)] == ["bld-clearance"]
    a = Action4D(vx=0.0, vy=-1.0)
    want = sh.filter(here, a, t=0.1).model_dump()
    nan = here.model_copy(update={"velocity": (float("nan"), 0.0, 0.0)})
    got = sh.filter(nan, a, t=0.2).model_dump()
    assert got == want, "a NaN velocity must fly the commanded forecast"
    assert sh.n_velocity_nonfinite == 1 and sh.n_velocity_ticks == 0


def test_the_shield_s_own_fsm_reads_stop_illegal_from_the_position():
    """The Shield's own escalation FSM (Shield(escalation=True)) is given
    stop_illegal and unsafe_depth about where the aircraft IS, as every rail
    gives its FSM, never about where a stop would end: 3.2 m off the 14 Oct
    wall and legal, closing at 3.5 m/s, a stop would end in the building
    margin (the rule is committed: the decision judges it from the stop
    point), yet the FSM's input says stop_illegal False and depth 0, and no
    X6/X7 fires (2026-10-20 review: reading the stop point there instead
    went unnoticed)."""
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=_city_map(), escalation=True)
    here = State(x=36.0, y=-52.5, up=8.0)
    fast = here.model_copy(update={"velocity": (0.0, -3.5, 0.0)})
    assert not sh.state_is_unsafe(here)
    com = sh._committed(fast)
    assert com is not None and "bld-clearance" in com[1], "the stop must end in the margin"
    edges = []
    for k in range(8):
        d = sh.filter(fast, Action4D(vx=0.0, vy=-3.5), t=0.1 * (k + 1))
        rec = d.fsm_record
        assert rec["flags"]["stop_illegal"] is False, rec["flags"]
        assert rec["unsafe_depth_m"] in (None, 0.0), rec["unsafe_depth_m"]
        edges.append(rec["edge"])
    assert not {"X6", "X7"} & set(edges), edges


def test_from_an_illegal_position_the_decision_is_the_commanded_forecast_s():
    """pas_ap_close_off tick 733 (the Shield-off close run, replayed open
    loop): the aircraft 0.96 m inside the building footprint, still flying
    deeper at 1.2 m/s, the car 4.6 m away. With the measured velocity the
    car's stand-off was "committed" (a stop would end inside its ring) and
    judged from the stop point, where only an action opening the range
    passes; no action both left the building and opened the range from
    there, the chain fell to the ClearanceEscape rescue, and the rescue broke
    the stand-off: a P0 escape (7 such ticks, 733-739) where the Shield
    without the velocity pushed slowly out and broke nothing (2026-10-20
    review). While the position breaks a hard rule the velocity is set aside
    (Shield._decision_state): the decision is the commanded forecast's, byte
    for byte, and it is counted."""
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    smap = _city_map()
    sent = [(79.556, -2.808, -0.942), (79.666, -2.794, -0.981), (79.775, -2.282, -0.844),
            (79.885, -2.264, -0.890)]
    raw = Action4D(vx=-2.241, vy=-0.946, vz_up=0.001, yaw_rate=0.524)
    pos = State(x=34.565, y=-61.038, up=8.0, yaw_deg=-157.1)
    out = {}
    for name, vel in (("commanded", None), ("own", (-1.19, -0.31, 0.0))):
        sh = Shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=smap, escalation=False)
        sh.filter(pos, Action4D(), t=79.4)
        for t, vx, vy in sent:
            sh.note_flown(Action4D(vx=vx, vy=vy), t)
        sh.set_subject(30.99, -64.99, "vehicle")
        st = pos if vel is None else pos.model_copy(update={"velocity": vel})
        assert [v.rule_id for v in sh.state_is_unsafe(st)] == ["bld-clearance"]
        out[name] = (sh.filter(st, raw, t=79.995), sh)
    d_old, _ = out["commanded"]
    d_new, sh = out["own"]
    p0 = {c.id for c in pol.constraints if c.priority == "P0"}
    assert not [v for v in d_old.emitted_violations if v.rule_id in p0], d_old.emitted_violations
    assert not [v for v in d_new.emitted_violations if v.rule_id in p0], d_new.emitted_violations
    assert d_new.model_dump() == d_old.model_dump()
    assert sh.n_velocity_set_aside == 1 and sh.n_velocity_ticks == 0, (
        sh.n_velocity_set_aside, sh.n_velocity_ticks)
    assert sh.history[-1].state.velocity is None and sh.history[-1].pipe is None


def _seg_policy(verts):
    from guardrail.models import Policy
    return Policy.model_validate({"policy_id": "seg", "constraints": [
        {"id": "zone", "type": "polygon_fence", "margin_m": 0.0,
         "vertices": [{"x": x, "y": y} for x, y in verts]}]})


def test_the_segment_check_walks_each_forecast_path_never_the_connectors():
    """Shield(segment_check=True) with a measured velocity: the forecast is two
    paths, and the grant's "trajectory segment crosses polygon" test walks each
    on its own. Aircraft at the origin moving North at 4 m/s, commanded East
    at 4 m/s. (A) A 0.1 m square between the two paths, on the straight line
    joining the own-velocity pose and the commanded pose at t + 0.3 s, which
    neither path enters: the first build walked the interleaved stream and
    REFUSED the command ("predicted segment (0.0,1.2)->(1.2,0.0) crosses
    NFZ"). (B) A 0.02 x 0.2 m strip across the own-velocity path between its
    t + 1.5 s and t + 1.6 s poses (after the reaction time, off the stop
    path): the first build walked only connectors and MISSED it."""
    st = State(x=0.0, y=0.0, up=10.0, velocity=(0.0, 4.0, 0.0))
    a = Action4D(vx=4.0, vy=0.0)
    sq = [(0.55, 0.55), (0.65, 0.55), (0.65, 0.65), (0.55, 0.65)]
    assert not Shield(_seg_policy(sq), escalation=False)._check(st, a), "no pose inside"
    sh = Shield(_seg_policy(sq), escalation=False, segment_check=True)
    assert sh._check(st, a) == [], "a zone neither forecast path enters is not broken"
    assert sh.filter(st, a).emitted == a
    # (B) the strip, placed on the Shield's own forecast.
    probe = Shield(_seg_policy([(50, 50), (51, 50), (51, 51)]), escalation=False)
    poses = {round(t, 3): p for t, p in probe.forecast(st, a)}
    p0, p1 = poses[1.5], poses[1.6]
    mx, my = (p0.x + p1.x) / 2.0, (p0.y + p1.y) / 2.0
    L = math.hypot(p1.x - p0.x, p1.y - p0.y)
    ux, uy = (p1.x - p0.x) / L, (p1.y - p0.y) / L
    strip = [(mx + sn * 0.1 * uy + sd * 0.01 * ux, my - sn * 0.1 * ux + sd * 0.01 * uy)
             for sn, sd in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    assert not Shield(_seg_policy(strip), escalation=False)._check(st, a), "no pose inside"
    sh = Shield(_seg_policy(strip), escalation=False, segment_check=True)
    com = sh._committed(st)
    assert com is not None and not com[1] and not com[3], "the stop path must not cross it"
    vs = sh._check(st, a)
    assert [v.rule_id for v in vs] == ["zone"], vs
    assert 1.5 < vs[0].predicted_at_s < 1.6 and "segment" in vs[0].detail, vs[0]
    assert sh.filter(st, a).repairs, "the repair acts on the crossing too"


def test_the_pipe_is_aged_on_the_rail_s_real_clock():
    """Shield(pipe_clock=...) (2026-10-20 review): the rail lets filter()
    count its monitor ticks at the nominal 0.1 s while its real ticks are
    longer (0.109 s median in every recorded pas_ap log, 0.73 s once on
    14 Oct), so a setpoint was aged on the wrong clock. With a pipe clock the
    record of what was sent is kept, and read, on that clock: a setpoint sent
    0.4 s of REAL time ago has already taken effect (past the 0.45 s reaction
    time, give or take a tick), however many monitor ticks that was. Without
    a velocity nothing reads it: the decisions are those of a Shield with no
    pipe clock."""
    from guardrail import response as R
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    real = [100.0]
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False, pipe_clock=lambda: real[0])
    plain = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False)
    st = State(x=0.0, y=0.0, up=8.0)
    for k in range(3):                         # three monitor ticks, 0.25 s of real time each
        for s in (sh, plain):
            s.filter(st, Action4D(vx=1.0))
            s.note_flown(Action4D(vx=3.0 if k == 0 else 0.0))
        real[0] += 0.25
    sh.filter(st, Action4D())
    plain.filter(st, Action4D())
    lag = R.ARDUCOPTER_GUIDED.reaction_s
    want = R.pipe_segments([(100.0, 3.0, 0.0), (100.25, 0.0, 0.0), (100.5, 0.0, 0.0)],
                           100.75, lag)
    assert sh.pipe() == want, (sh.pipe(), want)
    assert sh._flown[0][0] == 100.0 and plain._flown[0][0] == 0.0, (sh._flown, plain._flown)
    assert plain.pipe() != sh.pipe(), "the monitor clock ages the same sends differently"
    # Without a velocity the pipe clock changes no decision.
    for x in (0.0, 30.0, 36.0):
        for a in (Action4D(vx=2.0), Action4D(vy=-3.5)):
            s0 = State(x=x, y=-52.0, up=8.0)
            assert sh._check(s0, a) == plain._check(s0, a)
    src = (ROOT / "demo" / "pas_ardupilot" / "perception_node.py").read_text(encoding="utf-8")
    assert "pipe_clock=clock" in src, "the ArduPilot rail must age its pipe on its own clock"


def test_a_filter_tick_reads_its_pipe_on_the_pipe_clock():
    """THE PIPE A DECISION IS MADE WITH (2026-10-20 re-review). The test above
    reads `pipe()` between ticks; the decisions read it INSIDE a filter()
    tick, where `Shield._pipe_now` latches the pipe clock on the tick. The
    mutation `tick.pipe_t = tick.t` (the monitor clock) passed every test of
    this file. On the real rail the two clocks differ by thousands of
    seconds (the loop clock against a monitor clock counted from 0), and
    with the mutation every setpoint noted on the loop clock lies "in the
    future" of the tick: the pipe is the measured velocity coasted for the
    whole reaction time - the "pipe never filled" failure the node's
    noted_setpoints count exists to catch, which it cannot see, since the
    setpoints WERE noted. Here: the pipe clock starts at 5000 s, the monitor
    ticks at 0.1-0.6 s, five 3 m/s setpoints are noted on the pipe clock, and
    the sixth tick's record must carry the pipe of those five."""
    from guardrail import response as R
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    real = [5000.0]
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False, pipe_clock=lambda: real[0])
    st = State(x=0.0, y=0.0, up=8.0, velocity=(0.0, -1.0, 0.0))
    sent = []
    for k in range(5):
        sh.filter(st, Action4D(vy=-3.0), t=0.1 * (k + 1))
        sh.note_flown(Action4D(vy=-3.0))
        sent.append((real[0], 0.0, -3.0))
        real[0] += 0.1
    sh.filter(st, Action4D(vy=-3.0), t=0.6)
    lag = R.ARDUCOPTER_GUIDED.reaction_s
    want = tuple(R.pipe_segments(sent, real[0], lag))
    assert [round(f[0], 3) for f in sent] == [5000.0, 5000.1, 5000.2, 5000.3, 5000.4], sent
    assert sh.history[-1].pipe == want, (sh.history[-1].pipe, want)
    # The pipe is FILLED: what was sent acts through the reaction time, not
    # the measured velocity coasted (the monitor clock's reading).
    assert want != ((lag, None, None),) and all(s[2] == -3.0 for s in want[1:]), want
    assert abs(sum(s[0] for s in want) - lag) < 1e-9, want


def test_a_velocity_tick_s_forecast_is_rebuilt_exactly_from_its_record():
    """TickRecord's contract (2026-10-20 review): the predicted trajectory is
    recomputable from the record. With a measured velocity the forecast also
    depends on the pipe and the tick's time, which the State does not hold;
    the record now carries the tick's pipe, and forecast(..., pipe=rec.pipe)
    rebuilds the forecast the tick judged, whatever was sent since. A record
    without a velocity carries none, as before."""
    pol = load_policy(ROOT / "policies" / "follow_car_citylife.yaml")
    sh = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False)
    sh.filter(State(x=0.0, y=0.0, up=8.0), Action4D(), t=1.0)
    sh.note_flown(Action4D(vx=3.0), 1.0)
    st = State(x=0.0, y=0.0, up=8.0, velocity=(1.0, 0.0, 0.0))
    d = sh.filter(st, Action4D(vx=0.5), t=1.1)
    then = sh.forecast(st, d.emitted)
    rec = sh.history[-1]
    assert rec.pipe and rec.pipe == tuple(sh.pipe(1.1)), rec.pipe
    for k in range(4):                         # the world moves on
        sh.note_flown(Action4D(vx=-2.0), 1.1 + 0.1 * k)
        sh.filter(State(x=0.0, y=0.0, up=8.0), Action4D(), t=1.2 + 0.1 * k)
    assert sh.forecast(rec.state, rec.decision.emitted) != then, "the live pipe moved on"
    assert sh.forecast(rec.state, rec.decision.emitted, pipe=rec.pipe) == then
    assert sh.history[-1].pipe is None, "no velocity, no pipe on the record"


# --------------------------------------------------------------------------- #
# The fit (tools/fit_velocity_response.py on the committed recorded flights)
# --------------------------------------------------------------------------- #

def test_the_response_constants_are_the_fit_of_the_recorded_flights():
    """Re-running the fit on tests/fixtures/pas_ap_response_id.jsonl.gz (the
    six recorded pas_ap flights, trimmed to time, position, setpoint and mode)
    lands on guardrail/response.py's constants, and the table that justifies
    them holds: the old forecast puts a decelerating aircraft 1.5 m short of
    where it went (median, 1.5 s); the Shield's forecast, in the form it flies
    (the pipe, then the candidate), is NOT short (2026-10-20 review: the
    identified numbers were a median 0.25 m short in that form and the bias
    rule had fallen back to them in silence); the identified aircraft is the
    closed loop's plant; and the BRAKE envelopes are the ones the hold plan
    and the closed loop predict with."""
    sys.path.insert(0, str(ROOT / "tools"))
    import fit_velocity_response as FV
    import own_velocity_closed_loop as CL
    from guardrail import response as R
    rep = FV.fit(FV.load_fixture(FV.FIXTURE))
    assert rep["windows"] == 4417 and rep["step_windows"] == 550, rep["windows"]
    c = rep["chosen"]
    assert (c["reaction_s"], c["accel_mps2"]) == (R.ARDUCOPTER_GUIDED.reaction_s,
                                                  R.ARDUCOPTER_GUIDED.accel_mps2), c
    # The bias rule, in the form the Shield flies: the MEDIAN braking error at
    # 1.5 s is not short. A median at one horizon, not a bound (2026-10-20
    # re-review: the commit that introduced it was titled "Never be
    # optimistic about a stop"), so the spread and the later horizon that the
    # docstrings quote beside it are pinned too: p25 -0.33 m at 1.5 s; at
    # 2.0 s the median is short, -0.16 m (p25 -0.51 m), while the physical
    # form with the same numbers stays >= 0 in the median at every horizon.
    assert rep["fallback"] is None, rep["fallback"]
    assert rep["shield_form"] == "pipe" and c["shield_decel_bias_m"] >= 0.0, c
    sf = rep["table"]["Shield form, with the pipe (note_flown)"]["decel_along_by_h_m"]
    phys = rep["table"]["physical form, the Shield's numbers"]["decel_along_by_h_m"]
    for h, p25, med in ((1.5, -0.33, 0.03), (2.0, -0.51, -0.16)):
        assert abs(sf[h]["p25"] - p25) < 0.01 and abs(sf[h]["median"] - med) < 0.01, \
            (h, sf[h])
    assert sf[2.0]["median"] < 0.0, "the claim is a 1.5 s median: at 2.0 s it is short"
    assert all(q["median"] >= 0.0 for q in phys.values()), phys
    i = rep["identified"]
    assert (i["reaction_s"], i["accel_mps2"]) == CL.PLANT_GUIDED, i
    assert i["shield_decel_bias_m"] < -0.2, "the identified numbers are short in the Shield's form"
    b = rep["brake"]["chosen"]
    assert (b["reaction_s"], b["accel_mps2"]) == (R.ARDUCOPTER_BRAKE.reaction_s,
                                                  R.ARDUCOPTER_BRAKE.accel_mps2), b
    bi = rep["brake_identified"]["chosen"]
    assert (bi["reaction_s"], bi["accel_mps2"]) == (CL.PLANT_BRAKE_LAG_S,
                                                    CL.PLANT_BRAKE_ACCEL), bi
    t = rep["table"]
    old = t["commanded (old forecast)"]
    new = t["Shield form, with the pipe (note_flown)"]
    assert old["decel_bias_m"] < -1.4, old["decel_bias_m"]
    assert 0.0 <= new["decel_bias_m"] < 0.3, new["decel_bias_m"]
    assert t["Shield form, with the pipe, identified numbers"]["decel_bias_m"] < -0.2
    for h in (0.5, 1.0, 1.5, 2.0):
        assert new["steps"][h]["rms"] < 0.6 * old["steps"][h]["rms"], (h, new["steps"][h],
                                                                       old["steps"][h])
        # ... and no less accurate in the Shield's form than the identified
        # numbers (within 0.03 m), which is why the choice costs nothing.
        idn = t["Shield form, with the pipe, identified numbers"]["steps"][h]["rms"]
        assert new["steps"][h]["rms"] < idn + 0.03, (h, new["steps"][h]["rms"], idn)
    v = rep["vertical"]["rms_m"]["1.0"]
    assert v["commanded"] < v["response"], "the vertical channel is kept commanded"


def test_the_fit_says_so_when_the_bias_rule_falls_back():
    """The first fit fell back to the ridge in silence (`ok = [...] or
    ridge`). On a grid with nothing long enough on braking - here a single
    instant, very stiff aircraft - the report carries `fallback` and the
    tool prints a WARNING."""
    import contextlib
    import io
    sys.path.insert(0, str(ROOT / "tools"))
    import fit_velocity_response as FV
    keep = FV.GRID_L, FV.GRID_A
    try:
        FV.GRID_L, FV.GRID_A = [0.0], [6.0]
        rep = FV.fit(FV.load_fixture(FV.FIXTURE))
        assert rep["fallback"] and "least short" in rep["fallback"], rep["fallback"]
        assert rep["chosen"]["shield_decel_bias_m"] < 0.0
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            FV.main(["--fixture", str(FV.FIXTURE)])
        assert "WARNING: the bias rule FELL BACK" in buf.getvalue(), buf.getvalue()[-400:]
    finally:
        FV.GRID_L, FV.GRID_A = keep


# --------------------------------------------------------------------------- #
# The 14 Oct replay, and the stop-point counterfactual
# --------------------------------------------------------------------------- #

def _approaches():
    sys.path.insert(0, str(ROOT / "tools"))
    import own_velocity_closed_loop as CL
    return CL, *CL.load_approaches()


def _first_clearance_flag(CL, rows, pol, smap, velocity: str):
    """Every logged tick through the Shield (open loop), the logged flown
    action fed back as the pipe; the first tick after 300 whose decision
    flags bld-clearance, with the Shield at that tick."""
    from guardrail.replay import rail_shield
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=smap)
    for i, r in enumerate(rows):
        vel = None
        if velocity == "causal":
            vel = CL.causal_velocity(rows, i)
        elif velocity == "central":
            vx, vy = CL._ls_velocity(rows, i)
            vel = (vx, vy, 0.0)
        st = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"], velocity=vel)
        s = r["subject"]
        if s is None:
            sh.set_subject(None)
        elif math.hypot(s[0] - r["x"], s[1] - r["y"]) < 0.01:
            sh.set_subject(r["x"], r["y"], "vehicle")
        else:
            sh.set_subject(s[0], s[1], "vehicle")
        d = sh.filter(st, Action4D(**r["raw"]), t=r["t"])
        flag = (any(v.rule_id == "bld-clearance" for v in d.violations)
                or any("bld-clearance" in p.detail for p in d.repairs))
        if flag and r["tick"] > 300:
            return r, i, sh
        fa = r.get("flown_action")
        sh.note_flown(None if fa is None else Action4D(**fa), r["t"])
    return None, None, sh


def test_the_new_forecast_flags_the_14_oct_wall_a_second_earlier():
    """14 Oct (pas_ap_video_on_20261014): replayed with the commanded forecast
    the Shield first flags the building margin at tick 364, as it did in
    flight (4.61 m from the wall, 1.61 m outside the 3 m margin, at 3.4 m/s);
    with the measured velocity - a causal three-tick slope of the logged
    positions, the stand-in for the EKF velocity no run logged - at tick 354,
    ten ticks (1.09 s) earlier, 7.83 m from the wall. The 13 Oct close run
    (pas_ap_close_on): 382 against 373 (0.98 s). THE STOP POINT, on the
    IDENTIFIED aircraft (the physical fit, 0.35 s / 2.0 m/s^2 - the best
    estimate of where it would have gone): a GUIDED stop commanded at the old
    tick, through the response and the pipe of what was sent, ends 0.66 m
    from the wall (2.3 m inside the margin; the flight, which flew the
    Shield's 0.5-0.8 m/s push instead of a stop, reached 1.99 m); commanded
    at the new tick it ends 3.71 m away, outside the margin. The Shield's own
    numbers (0.45 s / 1.75 m/s^2, chosen so that their median braking error
    at 1.5 s is not short) forecast every one of those stops at least as
    long."""
    from guardrail import response as R
    CL, rows_by, pols = _approaches()
    smap = CL.city_map()
    ident = R.VelocityResponse(*CL.PLANT_GUIDED, source="identified")
    want = {"pas_ap_video_on_20261014": (364, 354), "pas_ap_close_on": (382, 373)}
    for run, (t_old, t_new) in want.items():
        rows, pol = rows_by[run], pols[run]
        r_old, i_old, sh_old = _first_clearance_flag(CL, rows, pol, smap, "none")
        r_new, i_new, sh_new = _first_clearance_flag(CL, rows, pol, smap, "causal")
        assert r_old["tick"] == t_old and r_new["tick"] == t_new, (run, r_old["tick"],
                                                                   r_new["tick"])
        assert any(r["tick"] == t_old and "ClearanceFix" in r["repairs"] for r in rows), \
            "the replay's old tick is the flight's"
        r_c, _, _ = _first_clearance_flag(CL, rows, pol, smap, "central")
        assert r_c["tick"] == t_new, (run, r_c["tick"])
        for r, i, sh, outside in ((r_old, i_old, sh_old, False), (r_new, i_new, sh_new, True)):
            vx, vy = CL._ls_velocity(rows, i)
            sent = [(q["t"], *((q["flown_action"]["vx"], q["flown_action"]["vy"])
                               if q.get("flown_action") else (None, None)))
                    for q in rows[:i]]
            path = R.stop_path(ident, None, r["x"], r["y"], vx, vy,
                               pipe=R.pipe_segments(sent, r["t"], ident.reaction_s))
            low = min(sh.clearance_at(x, y) for _, x, y in path)
            assert (low >= 3.0) is outside, (run, r["tick"], low)
            own = R.stop_path(R.ARDUCOPTER_GUIDED, None, r["x"], r["y"], vx, vy,
                              pipe=sh.pipe(r["t"]))
            assert min(sh.clearance_at(x, y) for _, x, y in own) <= low, \
                "the Shield's numbers forecast a stop shorter than the identified aircraft's"
    # The flight's own minimum on 14 Oct, the margin it entered.
    rows = rows_by["pas_ap_video_on_20261014"]
    assert max(r["unsafe_depth_m"] or 0.0 for r in rows) > 1.0
    assert sum(bool(r["unsafe"]) for r in rows) == 71


def test_brake_s_predicted_stop_at_the_13_oct_loiter_entry_breaks_the_margin():
    """pas_ap_close_on tick 387: Brake -> Loiter [G3] at 2.7 m/s with the
    position legal; the rail asked for BRAKE, which carried the aircraft 2.9 m
    on and 3.4 m into the margin by the X7 RTL. The hold plan, given the
    velocity and the setpoints the rail had sent, predicts that BRAKE's stop
    path breaks bld-clearance and holds the Loiter in GUIDED; without a
    velocity it plans BRAKE, as the rail flew."""
    from guardrail import response as R
    from guardrail.replay import rail_shield
    CL, rows_by, pols = _approaches()
    rows, pol = rows_by["pas_ap_close_on"], pols["pas_ap_close_on"]
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1, obstacle_map=CL.city_map())
    i = next(k for k, r in enumerate(rows) if r["tick"] == 387)
    assert rows[i]["fsm_edge"] == "G3" and rows[i]["autopilot_set_mode"] == "BRAKE"
    assert not rows[i]["unsafe"], "the position was legal when BRAKE was asked for"
    for r in rows[:i]:
        fa = r.get("flown_action")
        sh.note_flown(None if fa is None else Action4D(**fa), r["t"])
    sh.filter(State(x=rows[i]["x"], y=rows[i]["y"], up=rows[i]["up"]),
              Action4D(**rows[i]["raw"]), t=rows[i]["t"])
    vx, vy = CL._ls_velocity(rows, i)
    st = State(x=rows[i]["x"], y=rows[i]["y"], up=rows[i]["up"],
               yaw_deg=rows[i]["yaw_deg"], velocity=(vx, vy, 0.0))
    plan = R.hold_plan(sh, st)
    assert plan["choice"] == "guided" and plan["breaks"] == ["bld-clearance"], plan
    assert R.hold_plan(sh, State(x=st.x, y=st.y, up=st.up))["choice"] == "brake"
    # The BRAKE envelope's stop point is past where the flight was when the
    # FSM sent RTL two seconds later (y -55.46).
    assert plan["stop_xy"][1] < -55.46, plan["stop_xy"]


# --------------------------------------------------------------------------- #
# The offline closed loop (tools/own_velocity_closed_loop.py)
# --------------------------------------------------------------------------- #

def test_the_closed_loop_keeps_both_approaches_outside_the_margin():
    """The IDENTIFIED aircraft (the physical fit, 0.35 s / 2.0 m/s^2, with
    its own BRAKE envelope: not the Shield's numbers, so the result is not
    circular - 2026-10-20 review), the rail's FSM and its hold, the recorded
    pilot's raw actions: with the commanded forecast the simulated aircraft
    enters the building margin on both recorded approaches, 73 ticks each
    (the flights: 71 and 78), and goes home by X7; with the measured velocity
    it stays outside the margin - min clearance >= 3.0 m, no tick with the
    position illegal, no fail-safe - and flies no P0 escape on either basis.
    So does an aircraft exactly as slow as the Shield's model (0.45 s,
    1.75 m/s^2: the model matched; on the first build, whose model was the
    identified one, this aircraft entered the margin on 19-26 ticks). On an
    aircraft slower than the model (0.55 s, 1.5 m/s^2) it enters - compared
    like for like with the commanded forecast on that SAME aircraft (the
    first version of this test compared it with the fitted one): on fewer
    ticks, over 2 m less deep, still no P0 escape."""
    CL, rows_by, pols = _approaches()
    smap = CL.city_map()
    (lag_m, acc_m), (lag_s, acc_s) = CL.ROBUST_PLANES[1], CL.ROBUST_PLANES[2]
    for run in CL.WINDOWS:
        old = CL.fly(run, rows_by[run], pols[run], smap, own_velocity=False)
        new = CL.fly(run, rows_by[run], pols[run], smap, own_velocity=True)
        assert (old["lag"], old["accel"]) == CL.PLANT_GUIDED == (new["lag"], new["accel"])
        assert old["unsafe_ticks"] >= 70 and old["failsafe"] == "RTL", (run, old["unsafe_ticks"])
        assert new["unsafe_ticks"] == 0 and new["min_clearance_m"] >= 3.0, \
            (run, new["unsafe_ticks"], new["min_clearance_m"])
        assert new["failsafe"] is None and new["p0_escapes"] == 0 == old["p0_escapes"]
        # ... and none on the OLD basis either (the flown action re-checked on
        # the commanded forecast from the position): 6 and 1 on the first
        # build of this branch, before the measured velocity only added.
        assert new["p0_escapes_commanded_basis"] == 0 == old["p0_escapes_commanded_basis"], \
            (run, new["p0_escapes_commanded_basis"])
        assert new["first_clearance_flag"] < old["first_clearance_flag"]
        matched = CL.fly(run, rows_by[run], pols[run], smap, own_velocity=True,
                         lag=lag_m, accel=acc_m)
        assert matched["unsafe_ticks"] == 0 and matched["min_clearance_m"] >= 3.0, \
            (run, matched["unsafe_ticks"], matched["min_clearance_m"])
        assert matched["failsafe"] is None and matched["p0_escapes_commanded_basis"] == 0
        slow = CL.fly(run, rows_by[run], pols[run], smap, own_velocity=True,
                      lag=lag_s, accel=acc_s)
        slow_old = CL.fly(run, rows_by[run], pols[run], smap, own_velocity=False,
                          lag=lag_s, accel=acc_s)
        assert slow_old["unsafe_ticks"] >= 70, (run, slow_old["unsafe_ticks"])
        assert slow["unsafe_ticks"] < slow_old["unsafe_ticks"] and slow["p0_escapes"] == 0
        assert slow["p0_escapes_commanded_basis"] == 0, (run, slow["p0_escapes_commanded_basis"])
        assert slow["min_clearance_m"] > slow_old["min_clearance_m"] + 2.0, \
            (run, slow["min_clearance_m"], slow_old["min_clearance_m"])


# --------------------------------------------------------------------------- #
# The rails (demo/pas_ardupilot: the links and the node; the SITL rails' helper)
# --------------------------------------------------------------------------- #

def _pas():
    sys.path.insert(0, str(ROOT / "tests"))
    import test_pas_ardupilot as T
    return T


class _Msg:
    def __init__(self, kind, src=1, **f):
        self._kind, self._src = kind, src
        self.__dict__.update(f)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return self._src


def test_the_links_read_the_ekf_velocity_freshest_first():
    """pymavlink: LOCAL_POSITION_NED (m/s, NED) first, the velocity
    GLOBAL_POSITION_INT carries (cm/s) when LOCAL_POSITION_NED is stale, None
    when both are; MAVROS: velocity_local's ENU twist swapped to North / East
    / up. Up is minus NED's down."""
    import autopilot as ap
    from types import SimpleNamespace
    T = _pas()
    now = [100.0]
    link = ap.PymavlinkLink.__new__(ap.PymavlinkLink)
    link.snap, link.target_system, link.messages_seen = ap.Snapshot(), 1, {}
    link.events, link.on_message, link.ignored_heartbeats = [], None, 0
    link.clock = lambda: now[0]
    assert link.velocity() == (None, "no velocity received")
    link._handle(_Msg("GLOBAL_POSITION_INT", lat=356895000, lon=1396917000, alt=48000,
                      relative_alt=8000, vx=120, vy=-310, vz=-50), now[0])
    assert link.velocity() == ((1.2, -3.1, 0.5), "global_position_int")
    link._handle(_Msg("LOCAL_POSITION_NED", vx=1.25, vy=-3.15, vz=-0.4), now[0])
    assert link.velocity() == ((1.25, -3.15, 0.4), "local_velocity")
    now[0] += 0.25
    link._handle(_Msg("GLOBAL_POSITION_INT", lat=356895000, lon=1396917000, alt=48000,
                      relative_alt=8000, vx=100, vy=-300, vz=0), now[0])
    now[0] += 0.1                                   # LOCAL 0.35 s old, GLOBAL 0.1 s
    assert link.velocity() == ((1.0, -3.0, 0.0), "global_position_int")
    now[0] += 1.0
    assert link.velocity() == (None, "velocity stale or not finite")
    msgs = SimpleNamespace(**vars(T.FAKE_MSGS), TwistStamped=object)
    node = T.FakeNode()
    clk = T.Clock(0.01)
    import mavros_link as ml
    m = ml.MavrosLink(node=node, rclpy=T.FAKE_RCLPY, msgs=msgs, clock=clk)
    node.subs[ml.TOPIC_VELOCITY](SimpleNamespace(twist=SimpleNamespace(
        linear=SimpleNamespace(x=-3.1, y=1.2, z=0.5))))
    v, src = m.velocity()
    assert v == (1.2, -3.1, 0.5) and src == "local_velocity", (v, src)
    # A message set without TwistStamped (the old stand-ins) subscribes nothing.
    node2 = T.FakeNode()
    ml.MavrosLink(node=node2, rclpy=T.FAKE_RCLPY, msgs=T.FAKE_MSGS, clock=clk)
    assert ml.TOPIC_VELOCITY not in node2.subs


def _vel_link(T):
    class VelLink(T.FakeLink):
        """FakeLink flies its last setpoint at once in GUIDED; that is its
        measured velocity (LOCAL_POSITION_NED's, here)."""
        def velocity(self):
            v = self.v if self.snap.mode == "GUIDED" else Action4D()
            return (v.vx, v.vy, v.vz_up), "local_velocity"
    return VelLink


def test_the_ardupilot_rail_gives_the_shield_the_velocity_and_logs_it_either_way():
    """demo/pas_ardupilot: on by default, every row carries state_v and
    shield_velocity, metrics.shield_forecast says own_velocity and counts the
    ticks; `--own-velocity-forecast off` logs the same velocity and gives the
    Shield none (mode commanded); with the Shield off it is never given, so
    the control arm's escapes stay judged on the commanded forecast."""
    from guardrail import response as R
    T = _pas()
    VelLink = _vel_link(T)
    for extra, shield, mode in (((), "on", "own_velocity"),
                                (("--own-velocity-forecast", "off"), "on", "commanded"),
                                ((), "off", "commanded")):
        rc, run, link = T._fly(shield, link_cls=VelLink, extra=extra)
        assert rc == 0, (extra, shield)
        rows = T._rows(run)
        m = T._json(run, "metrics.json")
        assert m["shield_forecast"]["mode"] == mode, (extra, shield, m["shield_forecast"])
        assert all(r["state_v"] is not None and r["state_v_source"] == "local_velocity"
                   for r in rows)
        given = mode == "own_velocity"
        assert all(r["shield_velocity"] is given for r in rows), (extra, shield)
        assert m["shield_forecast"]["velocity_ticks"] == (len(rows) if given else 0)
        assert m["shield_forecast"]["response"]["reaction_s"] == R.ARDUCOPTER_GUIDED.reaction_s
        # THE PIPE (2026-10-20 review: dropping the node's note_flown call
        # left every forecast coasting the measured velocity through the
        # reaction time, and nothing failed): every setpoint the node sent
        # was noted, so the forecast flew what was sent.
        sent = sum(1 for r in rows if r["flown_action"] is not None)
        assert sent and m["shield_forecast"]["noted_setpoints"] == sent, \
            (m["shield_forecast"]["noted_setpoints"], sent)
        # The P0 escapes on the stored runs' basis are logged beside the
        # rail's own, never above it.
        assert all(isinstance(r["p0_escape_commanded_basis"], bool) for r in rows)
        assert m["shield_forecast"]["p0_escape_ticks_commanded_basis"] == sum(
            r["p0_escape_commanded_basis"] for r in rows)
        assert m["shield_forecast"]["p0_escape_ticks_commanded_basis"] <= T._json(
            run, "kpi.json")["p0_escapes"]
    # A link with no velocity (every stored test link): nothing given, said so.
    rc, run, _ = T._fly("on")
    rows = T._rows(run)
    assert all(r["state_v"] is None and not r["shield_velocity"] for r in rows)
    assert T._json(run, "metrics.json")["shield_forecast"]["velocity_sources"] == {
        "none: the link reads no velocity": len(rows)}


# Two P0 rules the Shield records and never repairs (monitor_only: "a P0
# monitor_only rule shows up in the escape count", shield.py), so every flown
# tick is a P0 escape on the rail's basis and the test can tell the two bases
# apart. WATCH: a speed cap the pilot's ~2.6 m/s breaks whatever the forecast
# (an escape on both bases). WALL: a keep-out zone 2 m East of the start,
# parallel to the pilot's northbound track; only the momentum carries the
# aircraft into it (an escape on the rail's basis only).
P0_WATCH_POLICY = """policy_id: pas-ap-p0-watch
version: 0.1.0
constraints:
  - id: watched-speed
    type: kinematic_envelope
    constraint_type: hard
    priority: P0
    violation_action: monitor_only
    speed_max_mps: 0.5
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 90.0
"""
P0_WALL_POLICY = """policy_id: pas-ap-p0-wall
version: 0.1.0
constraints:
  - id: wall
    type: polygon_fence
    constraint_type: hard
    priority: P0
    violation_action: monitor_only
    margin_m: 1.0
    vertices: [{x: 20, y: -17}, {x: 50, y: -17}, {x: 50, y: -10}, {x: 20, y: -10}]
"""


def test_the_commanded_basis_p0_escape_is_the_position_recheck_of_what_flew():
    """p0_escape_commanded_basis (rows) and metrics.shield_forecast.
    p0_escape_ticks_commanded_basis: the action that FLEW, re-checked from
    the POSITION on the commanded forecast - the basis of every stored P0
    escape count, which the lead compares a re-fly against (2026-10-20
    re-review: a constant False on the velocity branch passed the rail test
    above, which checks only the type, the sum and the bound). Each row is
    re-derived here by a Shield that never saw a velocity. WATCH (the
    velocity link: the aircraft flies its setpoint, which it reports): an
    escape on both bases on every tick, so the field is True on every tick.
    WALL (a link whose EKF reports a 4 m/s drift East, towards the zone, while
    the aircraft flies the pilot's command north): the rail's re-check, made
    with that momentum, finds the zone on every tick; the commanded basis
    does not (the flown track is parallel to it, 2 m off), so the field is
    False on every tick - a field copied from the rail's basis would say True."""
    from guardrail.models import Policy
    T = _pas()
    VelLink = _vel_link(T)

    class DriftLink(T.FakeLink):
        def velocity(self):
            return (0.0, 4.0, 0.0), "local_velocity"

    for policy, link_cls, cmd_basis in ((P0_WATCH_POLICY, VelLink, True),
                                        (P0_WALL_POLICY, DriftLink, False)):
        rc, run, _ = T._fly("on", link_cls=link_cls, policy=policy)
        assert rc == 0
        rows = T._rows(run)
        pol = Policy.model_validate(json.loads((run / "policy_g0.json").read_text()))
        p0 = {c.id for c in pol.constraints if c.priority == "P0"}
        plain = Shield(pol, lookahead_s=5.0, dt=0.1, escalation=False)
        # Every tick took the velocity branch and is an escape on the rail's basis.
        assert rows and all(r["shield_velocity"] and r["setpoint"] == "pass" and r["flown"]
                            and {v["rule_id"] for v in r["emitted_violations"]} & p0
                            for r in rows), rows[0]
        for r in rows:
            pos = State(x=r["x"], y=r["y"], up=r["up"], yaw_deg=r["yaw_deg"])
            again = {v.rule_id for v in plain._check(pos, Action4D(**r["flown_action"]))}
            assert r["p0_escape_commanded_basis"] is bool(again & p0) is cmd_basis, \
                (policy[:30], r["tick"], r["p0_escape_commanded_basis"], again)
        m, k = T._json(run, "metrics.json"), T._json(run, "kpi.json")
        assert m["shield_forecast"]["p0_escape_ticks_commanded_basis"] == \
            (len(rows) if cmd_basis else 0), m["shield_forecast"]
        assert k["p0_escapes"] == len(rows), k["p0_escapes"]


HOLD_POLICY = """policy_id: pas-ap-hold
version: 0.1.0
constraints:
  - id: alt-ceiling
    type: altitude_envelope
    constraint_type: hard
    priority: P0
    violation_action: loiter
    alt_min_m: 1
    alt_max_m: 7.5
"""


def _audit(run):
    return [json.loads(l) for l in (run / "audit.jsonl").read_text().splitlines() if l.strip()]


def _assert_stop_advice_on_record(rows, run):
    """Every stop-advised row has its own audit record (decision_kind
    "stop_advised", the same tick, the action that flew), and the row's
    stop_advice is that decision (2026-10-20 review: audit.jsonl held only
    the decision on the pilot's action, which was not flown)."""
    adv = {a["tick"]: a for a in _audit(run) if a.get("decision_kind") == "stop_advised"}
    advised = [r for r in rows if r["stop_advised"]]
    assert advised, "no stop-advised tick: proves nothing"
    assert sorted(adv) == [r["tick"] for r in advised], (sorted(adv)[:5],
                                                         [r["tick"] for r in advised][:5])
    for r in advised:
        a = adv[r["tick"]]
        assert a["emitted_action"] == r["flown_action"] == r["stop_advice"]["emitted"], r["tick"]
        assert a["repair_passes"] is not None, "the advised decision's trace is on record"
        assert [v["rule_id"] for v in a["emitted_violations"]] == \
            [v["rule_id"] for v in r["emitted_violations"]], r["tick"]
    assert all(r["stop_advice"] is None for r in rows if not r["stop_advised"])


def test_the_ardupilot_rail_holds_in_guided_until_brake_would_stop_legal():
    """The FSM's Loiter (X4: a loiter rule; the aircraft starts at 8 m under a
    7.5 m ceiling, so its POSITION breaks the rule): with the forecast off the
    rail asks for BRAKE at once and BRAKE holds the aircraft above the
    ceiling; on, the hold plan finds BRAKE's stop path illegal, the rail
    stays in GUIDED streaming the Shield's answer to a stop - a descent -
    and hands the hold to BRAKE once BRAKE's stop would end legal. The FSM's
    edges are the same edges; only how its Loiter is flown differs. Every
    tick it flew the Shield's stop has that decision on record."""
    T = _pas()
    VelLink = _vel_link(T)
    rc, run, link = T._fly("on", link_cls=VelLink, policy=HOLD_POLICY, max_s="6",
                           extra=("--own-velocity-forecast", "off"))
    rows = T._rows(run)
    entry = next(i for i, r in enumerate(rows) if r.get("fsm_state_after") == "Loiter")
    assert rows[entry]["autopilot_set_mode"] == "BRAKE" and link.requested[0] == "BRAKE"
    assert not any(r["hold_deferred"] for r in rows)
    assert rows[entry + 3]["up"] >= 7.9, "BRAKE held it above the ceiling"
    rc, run, link = T._fly("on", link_cls=VelLink, policy=HOLD_POLICY, max_s="6")
    rows = T._rows(run)
    m = T._json(run, "metrics.json")
    plans = m["hold"]["stop_clear"]["plans"]
    assert plans[0]["at"] == "loiter entry" and plans[0]["choice"] == "guided", plans[0]
    assert plans[0]["breaks"] == ["alt-ceiling"]
    held = [r for r in rows if r["hold_deferred"]]
    assert held and all(r["setpoint"] == "pass" and r["stop_advised"]
                        and r["fsm_setpoint"] in ("none", "brake", "pass") for r in held)
    assert all(r["flown_action"]["vz_up"] < 0 for r in held), "the Shield's stop descends"
    hand = next(p for p in plans if p["at"] == "handover")
    assert hand["choice"] == "brake" and not hand.get("forced"), hand
    assert m["hold"]["stop_clear"]["forced_handovers"] == {}
    k = next(i for i, r in enumerate(rows) if r.get("autopilot_set_mode") == "BRAKE")
    assert rows[k]["up"] <= 7.5 + 1e-6 and not rows[k]["unsafe"], rows[k]["up"]
    assert link.requested[0] == "BRAKE" and m["hold"]["stop_clear"]["guided_hold_ticks"] == len(held)
    _assert_stop_advice_on_record(rows, run)


def test_a_deferred_hold_goes_to_brake_when_the_stream_cannot_be_trusted():
    """A DEFERRED HOLD IS FLOWN BY THE NODE'S STREAM (2026-10-20 review):
    while the Loiter is held in GUIDED nothing but the node's setpoints stops
    the aircraft, and ArduCopter flies the last one for up to GUID_TIMEOUT if
    the node stalls. The hold goes to BRAKE whatever the plan says on (a) the
    deferral cap - here a 7 m ceiling, too far to descend within
    HOLD_DEFER_MAX_S; (b) a gap in the stream (the node stalled 0.5 s); (c) a
    stale link. Each is recorded (metrics.hold.stop_clear.forced_handovers,
    the plan's `forced`). Before: the hold stayed deferred through all three."""
    from guardrail import response as R
    T = _pas()
    VelLink = _vel_link(T)
    far = HOLD_POLICY.replace("alt_max_m: 7.5", "alt_max_m: 7")
    # (a) the cap
    rc, run, link = T._fly("on", link_cls=VelLink, policy=far, max_s="6")
    rows, m = T._rows(run), T._json(run, "metrics.json")
    sc = m["hold"]["stop_clear"]
    assert sc["forced_handovers"] == {"deferral_cap": 1}, sc["forced_handovers"]
    hand = next(p for p in sc["plans"] if p["at"] == "handover")
    assert hand["forced"] == "deferral_cap" and hand["choice"] == "brake", hand
    assert abs(hand["t"] - sc["plans"][0]["t"] - R.HOLD_DEFER_MAX_S) < 0.11, hand
    assert link.requested[0] == "BRAKE"
    assert not any(r["hold_deferred"] for r in rows if r["t"] > hand["t"] + 1e-6)

    # (b) a stall: the clock jumps 0.5 s inside one tick, 1 s into the hold.
    class StallLink(VelLink):
        stalled = False

        def pump(self):
            if not self.stalled and self.clock.t - self.t_start > 1.0:
                self.stalled = True
                self.clock.t += 0.5
            return super().pump()
    rc, run, link = T._fly("on", link_cls=StallLink, policy=HOLD_POLICY, max_s="6")
    sc = T._json(run, "metrics.json")["hold"]["stop_clear"]
    assert sc["forced_handovers"] == {"stream_gap": 1}, sc["forced_handovers"]
    hand = next(p for p in sc["plans"] if p["at"] == "handover")
    assert hand["forced"] == "stream_gap" and "no setpoint for" in hand["why"], hand

    # (c) the link goes stale while the hold is deferred.
    class DeadVelLink(T.InboundDeadLink):
        def velocity(self):
            v = self.v if self.snap.mode == "GUIDED" else Action4D()
            return (v.vx, v.vy, v.vz_up), "local_velocity"
    rc, run, link = T._fly("on", link_cls=DeadVelLink, link_kw={"die_at": 1.0},
                           policy=HOLD_POLICY, max_s="10")
    sc = T._json(run, "metrics.json")["hold"]["stop_clear"]
    assert sc["forced_handovers"] == {"link_stale": 1}, sc["forced_handovers"]
    assert link.requested[0] == "BRAKE", link.requested
    events = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "mode_request" and "link stale" in e.get("why", "")
               for e in events)


BRAKE_RULE_POLICY = """policy_id: pas-ap-brake-rule
version: 0.1.0
constraints:
  - id: kin-brake
    type: kinematic_envelope
    constraint_type: hard
    priority: P1
    violation_action: brake
    speed_max_mps: 2.0
    climb_rate_max_mps: 2.0
    yaw_rate_max_dps: 45.0
"""


def test_the_fsm_s_brake_setpoint_flies_the_shield_s_stop_and_it_is_on_record():
    """The FSM's "brake" setpoint (a brake-action rule broken in GUIDED) is
    flown, with the velocity on, as the Shield's answer to a stop: rows say
    stop_advised, setpoint "pass" and fsm_setpoint "brake" (2026-10-20 review:
    keeping a raw zero there instead went unnoticed), and every such tick has
    the advised decision on record in audit.jsonl. Off, the zero is flown as
    before and nothing is advised."""
    T = _pas()
    VelLink = _vel_link(T)
    rc, run, link = T._fly("on", link_cls=VelLink, policy=BRAKE_RULE_POLICY)
    rows = T._rows(run)
    adv = [r for r in rows if r["fsm_setpoint"] == "brake"]
    assert adv, "the FSM never asked for a stop: proves nothing"
    assert all(r["stop_advised"] and r["setpoint"] == "pass" for r in adv), adv[0]
    assert T._json(run, "metrics.json")["hold"]["stop_clear"]["stop_advised_ticks"] == len(adv)
    _assert_stop_advice_on_record(rows, run)
    rc, run, link = T._fly("on", link_cls=VelLink, policy=BRAKE_RULE_POLICY,
                           extra=("--own-velocity-forecast", "off"))
    rows = T._rows(run)
    assert any(r["setpoint"] == "brake" for r in rows)
    assert not any(r["stop_advised"] for r in rows)
    assert not any(a.get("decision_kind") for a in _audit(run))


def test_a_stop_advised_rail_run_exports_as_one_flight_every_row_joined():
    """tools/export_episodes on the rail runs above (2026-10-20 re-review).
    A stop-advised tick has two audit records at one tick (its filter()
    record, then decision_kind "stop_advised"), and the exporter split the
    audit into a new "flight" wherever the tick did not advance, then joined
    the best single fragment. Measured on e0264dd with these two runs: the
    hold (6 s) 81 records, 21 stop-advised, 22 "flights", 40 of 60 rows
    joined; the FSM's brake setpoint (6 s) 120 records, 60 stop-advised, 61
    "flights", 2 of 60 rows joined - the per-step policy_state source and the
    priority / category labels lost in silence. The forecast is on by
    default on this rail, so the WP4 export of the planned re-fly would have
    lost them. Now: one flight, every row joined to its tick's filter()
    record, and each stop-advised step carries the stop's record."""
    sys.path.insert(0, str(ROOT / "tools"))
    import export_episodes as X
    T = _pas()
    VelLink = _vel_link(T)
    for policy in (HOLD_POLICY, BRAKE_RULE_POLICY):
        rc, run, _ = T._fly("on", link_cls=VelLink, policy=policy, max_s="6")
        assert rc == 0
        rows = T._rows(run)
        audit = _audit(run)
        n_adv = sum(1 for a in audit if a.get("decision_kind") == "stop_advised")
        assert n_adv and n_adv == sum(r["stop_advised"] for r in rows), (n_adv, policy[:30])
        filter_ticks = {a["tick"] for a in audit if a.get("decision_kind") is None}
        steps, ep = X.export(X.load_episode(run))
        j = ep["audit_join"]
        assert j["flights_in_audit"] == 1 and j["records_of_other_flights"] == 0, j
        assert j["joined"] == len(filter_ticks) == len(rows), (j, len(filter_ticks), len(rows))
        assert j["raw_matched"] == j["raw_comparable"] == len(rows), j
        assert j["stop_advice_records"] == n_adv, j
        for s, r in zip(steps, rows):
            assert s["tick"] == r["tick"] and s["audit_record"], s["tick"]
            assert s["action"]["executed"] == r["flown_action"], s["tick"]
            if r["stop_advised"]:
                assert s["stop_advice"]["emitted"] == r["flown_action"] \
                    == r["stop_advice"]["emitted"], s["tick"]
            else:
                assert s["stop_advice"] is None, s["tick"]


def test_the_sitl_rails_give_the_velocity_only_when_asked():
    """sitl/ros2_shield_node.shield_state: the measured velocity reaches the
    Shield only with the flag on (off by default on both SITL rails: their
    physics' response is not the measured one), fresh (0.3 s) and finite;
    velocity_local's ENU is swapped to North / East / up."""
    sys.path.insert(0, str(ROOT / "sitl"))
    import ros2_shield_node as rn
    from types import SimpleNamespace
    # run_sitl_demo imports pymavlink at module level (absent on CI): read it.
    src = (ROOT / "sitl" / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert rn.OWN_VELOCITY_DEFAULT == "off" and 'OWN_VELOCITY_DEFAULT = "off"' in src
    assert "default=OWN_VELOCITY_DEFAULT" in src and "shield.note_flown(flown)" in src
    # Both age the pipe on their real clock (Shield(pipe_clock=...)).
    rsrc = (ROOT / "sitl" / "ros2_shield_node.py").read_text(encoding="utf-8")
    assert "pipe_clock=lambda: time.time()" in src and "pipe_clock=time.monotonic" in rsrc
    assert rn.velocity_from_enu(SimpleNamespace(x=-3.0, y=1.0, z=0.5)) == (1.0, -3.0, 0.5)
    st = State(x=1.0, y=2.0, up=8.0)
    assert rn.shield_state(st, (1.0, 2.0, 0.0), 0.1, False) is st
    assert rn.shield_state(st, (1.0, 2.0, 0.0), 0.5, True) is st
    assert rn.shield_state(st, (float("nan"), 2.0, 0.0), 0.1, True) is st
    assert rn.shield_state(st, (1.0, 2.0, 0.0), 0.1, True).velocity == (1.0, 2.0, 0.0)
    assert rn.make_parser().parse_args([]).own_velocity_forecast == "off"


def _git(*args, text=True):
    import subprocess
    return subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True,
                          check=True, text=text).stdout


def regenerate_identity() -> None:
    """Rewrite the identity fixture FROM THE BASE COMMIT, never from the code
    checked out (2026-10-20 review: the first version hashed whatever was
    checked out and stamped it "computed on c77c77d", so a run on a later
    commit would have replaced the baseline with that commit's own output and
    voided the proof). The base's guardrail/, policies/ and the city map are
    extracted from `git archive IDENTITY_BASE` into a scratch directory, a
    copy of this file is run there (`--emit-digests`, which imports that
    tree's guardrail), and the fixture records the base's full hash in
    `computed_on`, which the identity tests check."""
    import io
    import shutil
    import subprocess
    import tarfile
    import tempfile
    base = _git("rev-parse", "--verify", IDENTITY_BASE + "^{commit}").strip()
    with tempfile.TemporaryDirectory() as tmp:
        tar = _git("archive", "--format=tar", base, "guardrail", "policies",
                   "tests/fixtures/citylife_occ_flightband_6to14.npz", text=False)
        with tarfile.open(fileobj=io.BytesIO(tar)) as tf:
            tf.extractall(tmp)
        here = Path(tmp) / "tests" / Path(__file__).name
        shutil.copyfile(__file__, here)
        got = subprocess.run([sys.executable, str(here), "--emit-digests"], cwd=tmp,
                             capture_output=True, text=True, check=True).stdout
    dig = json.loads(got.strip().splitlines()[-1])
    spec = {"note": ("sha256 of the JSON dumps of seeded Shield.filter() decisions, "
                     "computed by `--regenerate` on a git archive of the commit before "
                     "State had a velocity (computed_on); see "
                     "tests/test_own_velocity_forecast.py"),
            "computed_on": base,
            "seed": IDENTITY_SEED, "policies": list(IDENTITY_POLICIES),
            "ci_n": N_CI, "full_n": N_FULL, "ci": dig["ci"], "full": dig["full"]}
    with open(IDENTITY, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps(spec, indent=2) + "\n")


def _run(fns) -> int:
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
    return 1 if failed else 0


if __name__ == "__main__":
    if "--emit-digests" in sys.argv:
        # Run by regenerate_identity inside a git archive of IDENTITY_BASE:
        # this file's ROOT is then that tree, and so is the guardrail imported.
        print(json.dumps({"ci": _identity_digests(N_CI), "full": _identity_digests(N_FULL)}))
        sys.exit(0)
    if "--regenerate" in sys.argv:
        regenerate_identity()
        print(f"wrote {IDENTITY.relative_to(ROOT)} (computed on a git archive of "
              f"{IDENTITY_BASE})")
        sys.exit(0)
    if "--regenerate-approaches" in sys.argv:
        # The run folders are gitignored; point at the checkout that holds
        # them: python tests/test_own_velocity_forecast.py
        #   --regenerate-approaches <...>/demo/out
        k = sys.argv.index("--regenerate-approaches")
        root = Path(sys.argv[k + 1]) if len(sys.argv) > k + 1 else MAIN_OUT
        p = regenerate_approaches(root)
        print(f"wrote {p.relative_to(ROOT)} ({p.stat().st_size} bytes)")
        sys.exit(0)
    if "--full" in sys.argv:
        for v in ("none", "explicit_none", "ignored"):
            n = _check_identity("full", v)
            print(f"PASS  full identity, velocity={v}: {n} decisions")
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    sys.exit(_run(fns))
