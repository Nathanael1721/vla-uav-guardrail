"""The simulator's stressors on the flight rails (2026-11-07, contract item
X-14): wind, GPS denial and start jitter flown on ArduPilot SITL, set from
the simulator's side, and the GPS fix the ROS 2 Shield node now hands the
Shield.

    python tests/test_sitl_stressors.py      (no pytest needed)

No SITL, no ROS, no MAVLink: the parameter mapping is checked against the
convention read in ArduCopter 4.5.7's source (the harness's comment "THE
SIMULATOR'S STRESSORS ON THE RAILS" names the lines), the simulator side's
scheduler is driven on a virtual clock through a stand-in link, the Shield
node on tests/test_railinputs_node.py's stand-in ROS stack, and the hil
campaign on a stand-in rig whose desktop side writes the records a real one
would. What only a flight can show - SITL's physics under the wind, EKF3
through the dropout, the marker's real delay - is listed in
docs/RUNBOOK-orin-hil.md step 14 as what must be flown.

  * the mapping: SIM_WIND_DIR is where the wind comes FROM, so it is the
    harness's "toward" plus 180; a drawn direction is the harness's own draw
    for the seed; a wind speed comes with SIM_WIND_T = 1 (no altitude
    profile) and no turbulence; a dropout is SIM_GPS_DISABLE 1 / 0 on the
    episode's clock;
  * the jitter: the rail's start (SITL_HOME and --start) is the headless
    episode's start for the same seed;
  * the scheduler: aligned on the node's t = 0 markers, never early, and a
    boot parameter that does not read back stops the mission;
  * the check after the flight fails closed: another launch's record, a
    boot value the node did not read back, a change late or never seen, a
    denial that never reached fix_ok, an unconfirmed jittered home;
  * the refusals: what SITL realises is flown, the rest refused with why;
  * fix_ok from NavSatFix, the rail Shield's X8 verdict, the node's FSM
    taking X8 (RTL), its records;
  * the scripts and the pymavlink rail share one rule and one record.
"""
from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import math
import random
import shutil
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "experiments", ROOT / "sitl", ROOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import sweep_scenarios as S                                       # noqa: E402
import sitl_stressors as SST                                      # noqa: E402
from guardrail import load_policy                                 # noqa: E402
from guardrail.fsm import EscalationFSM, FSMConfig, tick_input_from_decision  # noqa: E402
from guardrail.models import Action4D, State                      # noqa: E402
from guardrail.projection import LocalProjection                  # noqa: E402
from guardrail.replay import rail_shield                          # noqa: E402
from guardrail.scenario_spec import Library, ScenarioSpec         # noqa: E402

LIBRARY = Library(ROOT / "experiments" / "scenarios.yaml")
DEFAULTS = LIBRARY.defaults
FAMILIES = LIBRARY.by_id()
BY_ID = {sid: fam.first.spec for sid, fam in FAMILIES.items()}
POLICY = ROOT / "policies" / "sim_demo_policy.yaml"


def _cell(sid, **params):
    for c in FAMILIES[sid].cells(samples=3):
        if all(c.params.get(k) == v for k, v in params.items()):
            return c.spec
    raise KeyError(f"{sid}: no cell with {params}")


def _with(spec: ScenarioSpec, **changes) -> ScenarioSpec:
    d = spec.model_dump(mode="json", by_alias=True)
    for path, value in changes.items():
        node = d
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return ScenarioSpec.model_validate(d)


def _plan(spec, seed=0, nonce="0123456789abcdef", run_id="t1"):
    return S.rail_stress_plan(spec, seed, DEFAULTS, run_id=run_id, nonce=nonce)


# --------------------------------------------------------------------------- #
# the mapping
# --------------------------------------------------------------------------- #

def test_sitl_is_told_where_the_wind_comes_from():
    """SIM_WIND_DIR is the direction the wind blows FROM (SIM_Aircraft.cpp:
    wind_ef = -(cos, sin) x speed), the harness's wind_dir_deg the one it
    blows TOWARD: the air SITL moves is the harness's wind vector."""
    for toward in (0.0, 45.0, 90.0, 180.0, 225.0, 270.0, 359.5):
        sim = S.sim_wind_dir_deg(toward)
        assert 0.0 <= sim < 360.0
        # SITL's air velocity for SIM_WIND_DIR = sim, unit speed:
        air = (-math.cos(math.radians(sim)), -math.sin(math.radians(sim)))
        want = (math.cos(math.radians(toward)), math.sin(math.radians(toward)))
        assert abs(air[0] - want[0]) < 1e-9 and abs(air[1] - want[1]) < 1e-9, toward
    assert S.sim_wind_dir_deg(0.0) == 180.0 and S.sim_wind_dir_deg(270.0) == 90.0


def test_a_wind_cell_is_set_at_boot_and_switched_on_at_t0():
    spec = _cell("wind-within-authority", lane_y=15.0, wind_speed_mps=5)
    plan = _plan(spec)
    boot = {b["param"]: b["value"] for b in plan["boot"]}
    assert boot == {"SIM_WIND_T": 1.0, "SIM_WIND_TURB": 0, "SIM_WIND_DIR": 180.0,
                    "SIM_WIND_SPD": 0}, boot
    assert [(s["t"], s["param"], s["value"]) for s in plan["schedule"]] == \
        [(0.0, "SIM_WIND_SPD", 5.0)]
    assert SST.load_plan(plan) is plan
    # Turned with the mission: a tailwind stays a tailwind.
    turned = _with(spec, **{"mission.rotate_deg": 90.0})
    assert S.rail_wind(turned, 0)["sim_wind_dir_deg"] == 270.0
    # The pymavlink rail's rule gives the same profile.
    assert SST.with_uniform_wind({"SIM_WIND_SPD": 5.0}) == {"SIM_WIND_SPD": 5.0,
                                                           "SIM_WIND_T": 1.0}
    assert SST.with_uniform_wind({"SIM_WIND_SPD": 5.0, "SIM_WIND_T": 0.0})["SIM_WIND_T"] == 0.0
    assert SST.with_uniform_wind({"SIM_GPS_DISABLE": 1.0}) == {"SIM_GPS_DISABLE": 1.0}


def test_a_drawn_wind_direction_is_the_headless_draw_for_the_seed():
    """A scenario that leaves wind_dir_deg None draws it from the seed: the
    rail is given exactly the direction the headless episode flew."""
    base = _cell("wind-gusts", lane_y=15.0, wind_speed_mps=2)
    spec = _with(base, **{"stress.gust_mps": 0.0, "stress.start_jitter_m": 0.0})
    assert spec.stress.wind_dir_deg is None
    seen = set()
    for seed in (0, 1001, 1002, 1003):
        run = S.run_episode(spec, DEFAULTS, seed)
        wx, wy = run.events[0]["wind_mps"]
        flown = math.degrees(math.atan2(wy, wx)) % 360.0
        rail = S.rail_wind(spec, seed)
        assert rail["drawn_from_seed"] and abs((rail["toward_deg"] - flown + 180) % 360 - 180) < 0.1, \
            (seed, rail, flown)
        seen.add(round(rail["toward_deg"], 3))
    assert len(seen) == 4, "every seed draws its own direction"


def test_a_dropout_is_the_gps_switched_off_and_on_on_the_episode_clock():
    spec = _cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0])
    plan = _plan(spec)
    assert plan["boot"] == [{"param": "SIM_GPS_DISABLE", "value": 0,
                             "why": "GPS on until the window opens"}]
    assert [(s["t"], s["param"], s["value"], s.get("restore", False))
            for s in plan["schedule"]] == [(4.0, "SIM_GPS_DISABLE", 1, False),
                                           (11.0, "SIM_GPS_DISABLE", 0, True)]
    # Nothing to set: no wind, no dropout (a jittered start is SITL_HOME's).
    assert S.rail_stress_plan(BY_ID["nfz-head-on"], 0, DEFAULTS) is None
    jit = _with(BY_ID["nfz-head-on"], **{"stress.start_jitter_m": 1.0})
    assert S.rail_stress_plan(jit, 1001, DEFAULTS) is None


def test_a_plan_names_only_the_simulators_stressor_parameters():
    good = _plan(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]))
    SST.load_plan(good)
    for bad in ({"boot": [{"param": "FENCE_ENABLE", "value": 0}]},
                {"boot": [{"param": "SIM_GPS_NOISE", "value": 1}]},
                {"boot": [{"param": "SIM_WIND_SPD", "value": float("nan")}]},
                {"schedule": [{"t": -1.0, "param": "SIM_GPS_DISABLE", "value": 1}]},
                {"schedule": [{"t": 5.0, "param": "SIM_GPS_DISABLE", "value": 1},
                              {"t": 2.0, "param": "SIM_GPS_DISABLE", "value": 0}]},
                {"run_nonce": "short"}, {"schema": "something-else"}):
        try:
            SST.load_plan({**good, **bad})
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad}")


# --------------------------------------------------------------------------- #
# the jitter
# --------------------------------------------------------------------------- #

def test_a_jittered_start_is_the_headless_start_for_the_same_seed():
    """start_jitter_m: SITL_HOME and --start carry the offset the headless
    harness draws for the seed (random.Random(f"{seed}:jitter")), so the
    headless and the rail episode of one cell start at one point."""
    spec = _with(_cell("wind-gusts", lane_y=15.0, wind_speed_mps=2),
                 **{"stress.gust_mps": 0.0})
    assert spec.stress.start_jitter_m == 1.0
    pol = spec.policy.load(ROOT)
    starts = set()
    for seed in (0, 1001, 1002, 1003):
        run = S.run_episode(spec, DEFAULTS, seed)
        hx, hy = run.rows[0]["x"], run.rows[0]["y"]
        rm = S.rail_mission(spec, pol, DEFAULTS, seed)
        assert abs(rm["start"][0] - hx) < 1e-9 and abs(rm["start"][1] - hy) < 1e-9, seed
        dx, dy = S.start_jitter(spec, seed)
        assert abs(dx) <= 1.0 and abs(dy) <= 1.0 and rm["jitter_m"] == (dx, dy)
        lat, lon = (float(v) for v in S.sitl_home(spec, pol, DEFAULTS, seed).split(",")[:2])
        n, e = LocalProjection(*pol.frame_origin).to_local(lat, lon)
        assert abs(n - hx) < 0.01 and abs(e - hy) < 0.01, (seed, n, e, hx, hy)
        args = S.rail_node_args(spec, seed, DEFAULTS, backend="sitl")
        sx, sy, _ = (float(v) for v in next(a for a in args if a.startswith("--start="))
                     .split("=", 1)[1].split(","))
        assert abs(sx - hx) < 1e-5 and abs(sy - hy) < 1e-5
        starts.add((round(hx, 6), round(hy, 6)))
    assert len(starts) == 4
    # No jitter: the start is the same for every seed, as before.
    plain = BY_ID["nfz-head-on"]
    p2 = plain.policy.load(ROOT)
    assert {S.rail_mission(plain, p2, DEFAULTS, s)["start"] for s in (0, 7, 1001)} == \
        {S.rail_mission(plain, p2, DEFAULTS)["start"]}
    assert S.start_jitter(plain, 1001) == (0.0, 0.0)


# --------------------------------------------------------------------------- #
# the scheduler (sitl/sitl_stressors.py) on a virtual clock
# --------------------------------------------------------------------------- #

class _Link:
    """SitlLink's surface on a virtual clock: markers arrive at the receipt
    times given; a parameter in `reject` never echoes."""

    def __init__(self, markers=(), reject=()):
        self.t = 100.0
        self.link = "stand-in"
        self.markers: list[dict] = []
        self.params: dict[str, float] = {}
        self.sent: list[tuple] = []
        self.autopilot = None
        self._due = sorted(markers)
        self.reject = set(reject)

    def clock(self):
        return self.t

    def wait_autopilot(self, timeout):
        self.autopilot = {"sysid": 1, "compid": 1}
        return self.autopilot

    def pump(self, timeout):
        self.t += max(float(timeout), 0.001)
        while self._due and self._due[0][0] <= self.t:
            rx, nonce8, t = self._due.pop(0)
            self.markers.append({"rx": rx, "nonce8": nonce8, "t": t, "src": [1, 191],
                                 "wall": 0.0})

    def set_param(self, name, value, timeout=0.4, tries=3):
        self.t += 0.004
        sent = self.t
        self.sent.append((name, float(value), sent))
        if name in self.reject:
            return None, None, None
        self.params[name] = float(value)
        self.t += 0.008
        return float(value), sent, self.t

    def read_param(self, name, timeout=2.0, tries=3):
        return self.params.get(name)


def test_the_scheduler_applies_on_the_nodes_clock_and_never_early():
    plan = _plan(_cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0]))
    n8 = plan["run_nonce"][:8]
    t0 = 200.0                       # the node's t = 0, on the scheduler's clock
    # The t = 0 marker 3 ms late, the t = 1 one 40 ms late, one of another run.
    link = _Link(markers=[(t0 + 0.003, n8, 0.0), (t0 + 1.04, n8, 1.0),
                          (t0 + 0.5, "ffffffff", 0.0)])
    said: list[str] = []
    with tempfile.TemporaryDirectory() as td:
        rec_path = Path(td) / "rec.json"
        rc = SST.run(plan, rec_path, link, t0_timeout_s=600.0, say=said.append)
        rec = json.loads(rec_path.read_text(encoding="utf-8"))
    assert rc == 0 and rec["state"] == "done" and rec["boot"]["ok"], (rc, rec)
    assert said[0].startswith("SITL_STRESS BOOT OK") and said[-1] == "SITL_STRESS DONE"
    assert rec["run_nonce"] == plan["run_nonce"] and rec["t0"]["markers"] == 2
    # The boot parameter in the pymavlink rail's record.
    b = rec["boot"]["params"][0]
    assert (b["param"], b["set"], b["read_back"], b["applied"]) == \
        ("SIM_GPS_DISABLE", 0.0, 0.0, True)
    # Scheduled changes: never before the node's own t = 0 + t, late by the
    # marker's delay and the loop's 20 ms at most.
    sched_sent = [s for s in link.sent if s[2] > t0]
    assert [(n, v) for n, v, _ in sched_sent] == [("SIM_GPS_DISABLE", 1.0),
                                                  ("SIM_GPS_DISABLE", 0.0)]
    for (_, _, at), it in zip(sched_sent, plan["schedule"]):
        assert at >= t0 + it["t"] and at - (t0 + it["t"]) < 0.04, (at, it)
    for e in rec["schedule"]:
        assert e["applied"] and e["read_back"] == e["value"] and 0 <= e["late_s"] < 0.04, e


def test_a_boot_parameter_that_does_not_read_back_stops_the_mission():
    plan = _plan(_cell("wind-within-authority", lane_y=15.0, wind_speed_mps=5))
    said: list[str] = []
    with tempfile.TemporaryDirectory() as td:
        rec_path = Path(td) / "rec.json"
        rc = SST.run(plan, rec_path, _Link(reject={"SIM_WIND_DIR"}), t0_timeout_s=60.0,
                     say=said.append)
        rec = json.loads(rec_path.read_text(encoding="utf-8"))
    assert rc == 1 and rec["state"] == "failed" and not rec["boot"]["ok"]
    assert said[-1].startswith("SITL_STRESS BOOT FAILED") and "SIM_WIND_DIR" in said[-1]
    # No marker at all: the schedule never starts, and says so.
    with tempfile.TemporaryDirectory() as td:
        rec_path = Path(td) / "rec.json"
        rc = SST.run(plan, rec_path, _Link(), t0_timeout_s=2.0, say=said.append)
        rec = json.loads(rec_path.read_text(encoding="utf-8"))
    assert rc == 1 and "no t=0 marker" in rec["error"] and not rec["schedule"][0]["applied"]


def test_the_marker_and_the_t0_estimate():
    txt = SST.marker_text("abcdef0123456789", 1.0)
    assert txt == "GRDL T0 abcdef01 t=1.000" and len(txt) <= 50
    assert SST.parse_marker(txt) == ("abcdef01", 1.0)
    assert SST.parse_marker(txt.encode() + b"\x00\x00") == ("abcdef01", 1.0)
    assert SST.parse_marker("PreArm: GPS 1: Bad fix") is None
    assert abs(SST.t0_estimate([(10.02, 0.0), (11.005, 1.0), (12.3, 2.0)]) - 10.005) < 1e-9
    assert SST.t0_estimate([]) is None
    sched = [{"t": 0.0}, {"t": 2.0}]
    assert SST.due(sched, set(), None, 1e9) == []
    assert SST.due(sched, set(), 10.0, 11.0) == [0] and SST.due(sched, {0}, 10.0, 12.0) == [1]


# --------------------------------------------------------------------------- #
# the check after the flight fails closed
# --------------------------------------------------------------------------- #

def _flight(plan, *, lag=0.03, onset=0.2, last_t=30.0, events=True, node_boot=None):
    """A flight's (record, metrics, rows) as the simulator side and the node
    would leave them, each change seen `lag` s after its time and the fix lost
    `onset` s after the window opens."""
    rec = {"run_nonce": plan["run_nonce"], "state": "done", "error": None,
           "boot": {"ok": True, "params": [SST.stressor_entry(b["param"], b["value"],
                                                              b["value"])
                                           for b in plan["boot"]]},
           "schedule": [{**s, **SST.stressor_entry(s["param"], s["value"], s["value"]),
                         "t_echo": s["t"] + 0.012} for s in plan["schedule"]]}
    vals = {k: 0.0 for k in SST.WATCHED_SIM_PARAMS}
    vals.update({b["param"]: float(b["value"]) for b in plan["boot"]})
    vals.update(node_boot or {})
    evs = ([{"t": s["t"] + lag, "param": s["param"], "value": float(s["value"])}
            for s in plan["schedule"]] if events else [])
    drop = plan.get("gps_dropout_s")
    rows = []
    for k in range(int(last_t * 10) + 1):
        t = round(k * 0.1, 3)
        r = {"t": t, "x": -30.0, "y": 15.0, "up": 15.0}
        if drop is not None:
            r["fix_ok"] = not (drop[0] + onset <= t < drop[1] + onset)
        rows.append(r)
    metrics = {"sim_params": {"at_takeoff": {"sitl": True, "values": vals}, "events": evs},
               "home_check": {"checked": True, "miss_m": 0.004}}
    return rec, metrics, rows


def test_a_realised_dropout_passes_the_check_with_its_times_on_record():
    spec = _cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0])
    plan = _plan(spec)
    rec, met, rows = _flight(plan)
    problems, s = S.rail_stress_check(spec, plan, rec, met, rows)
    assert problems == [], problems
    keys = ("param", "set", "read_back", "applied")
    assert all(tuple(e)[:4] == keys for e in s["entries"]), s["entries"]
    sched = [e for e in s["entries"] if e["t"] is not None]
    assert [e["error_s"] for e in sched] == [0.03, 0.03]
    assert all(e["measured_by"].startswith("node") for e in sched)
    assert s["dropout"]["onset_s"] == 0.2 and s["dropout"]["no_fix_ticks"] == 70
    extra = S.sitl_extra(spec, rows, {}, spec.policy.load(ROOT))
    assert extra["gps_dropout_ticks"] == 70
    # The node saw no parameter event at all: the simulator side's own
    # timing (its t0 from the markers) is the measure, and says so.
    rec, met, rows = _flight(plan, events=False)
    problems, s = S.rail_stress_check(spec, plan, rec, met, rows)
    assert problems == [] and all(e["measured_by"].startswith("simulator side")
                                  for e in s["entries"] if e["t"] is not None)
    # The episode ended inside the window: the GPS never came back, as a
    # headless episode that ends there.
    rec, met, rows = _flight(plan, last_t=8.0)
    met["sim_params"]["events"] = met["sim_params"]["events"][:1]
    problems, s = S.rail_stress_check(spec, plan, rec, met, rows)
    assert problems == [] and "before the window's end" in s["entries"][-1]["note"]


def test_a_stress_that_was_not_realised_is_never_scored():
    spec = _cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0])
    plan = _plan(spec)

    def problems(**kw):
        mutate = kw.pop("mutate", None)
        rec, met, rows = _flight(plan, **kw)
        if mutate:
            rec, met, rows = mutate(rec, met, rows)
        return " | ".join(S.rail_stress_check(spec, plan, rec, met, rows)[0])

    assert "left no record" in problems(mutate=lambda r, m, w: (None, m, w))
    assert "not this launch's" in problems(
        mutate=lambda r, m, w: ({**r, "run_nonce": "another"}, m, w))
    assert "did not set and read back" in problems(
        mutate=lambda r, m, w: ({**r, "boot": {"ok": False}, "error": "x"}, m, w))
    assert "not the plan's" in problems(node_boot={"SIM_GPS_DISABLE": 1.0})
    assert "read no simulator parameters" in problems(
        mutate=lambda r, m, w: (r, {**m, "sim_params": {}}, w))
    assert "applied +0.500 s" in problems(lag=0.5)
    # A change seen BEFORE its time is not the scheduled one (the simulator
    # side never applies early): it reads as the change never seen.
    assert "saw no such change" in problems(lag=-0.3)
    assert "saw no such change" in problems(
        mutate=lambda r, m, w: (r, {**m, "sim_params": {**m["sim_params"], "events": [
            {"t": 4.03, "param": "SIM_GPS_DISABLE", "value": 1.0}]}}, w))
    assert "did not reach the Shield's fix_ok" in problems(onset=1.5)
    assert "reported no GPS fix at all" in problems(
        mutate=lambda r, m, w: (r, m, [{"t": x["t"]} for x in w]))
    assert "before the window opened" in problems(last_t=3.0)
    # A cell that sets nothing: SITL must be at its neutral values.
    plain = BY_ID["nfz-head-on"]
    _, met, rows = _flight(plan)
    met["sim_params"]["at_takeoff"]["values"]["SIM_GPS_DISABLE"] = 0.0
    assert S.rail_stress_check(plain, None, None, met, rows)[0] == []
    met["sim_params"]["at_takeoff"]["values"]["SIM_WIND_SPD"] = 5.0
    assert "does not set" in " ".join(S.rail_stress_check(plain, None, None, met, rows)[0])
    # A jittered start: the autopilot's home must be where SITL was told.
    jit = _with(plain, **{"stress.start_jitter_m": 1.0})
    _, met, rows = _flight(plan)
    met["sim_params"]["at_takeoff"]["values"]["SIM_GPS_DISABLE"] = 0.0
    assert S.rail_stress_check(jit, None, None, met, rows, seed=1001)[0] == []
    met["home_check"]["miss_m"] = 0.6
    assert "start jitter" in " ".join(S.rail_stress_check(jit, None, None, met, rows,
                                                          seed=1001)[0])


# --------------------------------------------------------------------------- #
# what the rails realise, and what they refuse
# --------------------------------------------------------------------------- #

def test_the_rails_fly_wind_dropout_and_jitter_and_refuse_the_rest_with_why():
    for backend in S.RAIL_BACKENDS:
        def why(spec):
            return S.rail_refusal(spec, backend, defaults=DEFAULTS) or ""
        for sid in ("gps-dropout", "wind-within-authority", "wind-beyond-authority"):
            assert not why(BY_ID[sid]), (backend, sid, why(BY_ID[sid]))
        jit = _with(_cell("wind-gusts", lane_y=15.0, wind_speed_mps=2),
                    **{"stress.gust_mps": 0.0})
        assert not why(jit), why(jit)
        noise = why(_cell("gps-noise-within-margin", pass_y=23.2, sigma=0.3, latency=2))
        assert "gps_noise_m" in noise and "SIM_GPS_NOISE" in noise and "latency_ticks" in noise
        assert "gust_mps" in why(BY_ID["wind-gusts"]) and "SIM_WIND_TURB" in why(BY_ID["wind-gusts"])
        assert "battery_critical_at_s" in why(BY_ID["failsafe-low-battery"])
        blind = _with(BY_ID["gps-dropout"], **{"stress.fix_quality_reported": False})
        assert "fix_quality_reported false" in why(blind)
        late = _with(BY_ID["gps-dropout"], **{"stress.gps_dropout_s": [70.0, 80.0]})
        assert "after the run's last tick" in why(late)
        assert "no SIM_* wind or GPS stressor is wired" not in noise


# --------------------------------------------------------------------------- #
# the GPS fix on the ROS 2 Shield node, and X8 on its FSM
# --------------------------------------------------------------------------- #

def test_fix_ok_is_the_navsatfix_status_while_it_is_fresh():
    import ros2_shield_node as N
    assert N.FIX_TOPIC == "/mavros/global_position/raw/fix"
    assert N.fix_ok_from(None, None) is None              # no message: as before
    assert N.fix_ok_from(0, 0.1) is True                  # STATUS_FIX
    assert N.fix_ok_from(2, 0.1) is True                  # GBAS
    assert N.fix_ok_from(-1, 0.1) is False                # STATUS_NO_FIX
    assert N.fix_ok_from(-1, N.FIX_STALE_S) is False
    assert N.fix_ok_from(-1, N.FIX_STALE_S + 0.01) is None      # stale: not a report
    assert N.fix_ok_from(0, float("nan")) is None
    rows = [{"t": 0.0, "fix_ok": True}, {"t": 0.1, "fix_ok": False},
            {"t": 0.2, "fix_ok": False}, {"t": 0.3, "fix_ok": True},
            {"t": 0.4, "fix_ok": False}]
    assert N.fix_spells(rows) == [{"t_start": 0.1, "t_end": 0.3, "ticks": 2},
                                  {"t_start": 0.4, "t_end": None, "ticks": 1}]


def _rail_flight(stream_toward: bool, fix=True):
    """A rail Shield (no FSM of its own) and the node's FSM, the aircraft
    held 2 m south of nfz-square's face (x 7) on y 15, the fix lost at 1 s.
    `stream_toward`: what the rail says it sent (note_flown) - 2 m/s north,
    or nothing. Returns (first tick with position_lost, its FSM edge)."""
    pol = load_policy(POLICY)
    sh = rail_shield(pol, lookahead_s=5.0, dt=0.1)
    fsm = EscalationFSM(FSMConfig())
    for k in range(80):
        ok = True if k < 10 else False
        st = State(x=5.0, y=15.0, up=15.0, yaw_deg=0.0, fix_ok=ok if fix else None)
        d = sh.filter(st, Action4D())
        lost = d.position_lost
        inp = tick_input_from_decision(k * 0.1, d, pol,
                                       stop_illegal=sh.state_is_unsafe(st),
                                       caps=sh.applied_caps or False,
                                       position_lost=lost[0] if lost else None)
        out = fsm.step(inp)
        sh.note_flown(Action4D(vx=2.0) if stream_toward else None)
        if lost:
            return k, out.edge, lost
    return None, None, None


def test_a_rail_shield_judges_x8_and_the_nodes_fsm_returns():
    """The ROS 2 node's Shield has no FSM of its own; since 2026-11-07 it
    still judges X8's input on a state that reports its fix
    (ShieldDecision.position_lost), dead-reckoning on what the rail SAID it
    sent (Shield.note_flown), and the node's FSM takes X8 to RTL."""
    k_hold, edge, lost = _rail_flight(stream_toward=False)
    assert k_hold is not None and edge == "X8" and lost[1] == "nfz-square", (k_hold, edge, lost)
    # 2 m from the zone, 0.5 m/s of drift allowance either way: about 2 s
    # after the last valid fix (tick 9).
    assert 28 <= k_hold <= 33, k_hold
    k_go, edge, _ = _rail_flight(stream_toward=True)
    assert edge == "X8" and k_go < k_hold, (k_go, k_hold)
    # No report, no verdict: the rail flies exactly as before.
    assert _rail_flight(stream_toward=True, fix=False) == (None, None, None)
    pol = load_policy(POLICY)
    a, b = rail_shield(pol, lookahead_s=5.0, dt=0.1), rail_shield(pol, lookahead_s=5.0, dt=0.1)
    for k in range(20):
        st = State(x=5.0 + 0.1 * k, y=15.0, up=15.0)
        da = a.filter(st, Action4D(vx=2.0))
        db = b.filter(st.model_copy(update={"fix_ok": True}), Action4D(vx=2.0))
        assert da.model_dump() == db.model_dump() and db.position_lost is None


def _node_module():
    """sitl/ros2_shield_node.py on tests/test_railinputs_node.py's stand-in
    ROS stack, plus the two message types this change reads."""
    import test_railinputs_node as R
    fakes = R._fake_ros()
    for n in ("sensor_msgs", "sensor_msgs.msg"):
        fakes[n] = types.ModuleType(n)
    fakes["sensor_msgs.msg"].NavSatFix = type("NavSatFix", (R._Msg,), {})
    fakes["mavros_msgs.msg"].ParamEvent = type("ParamEvent", (R._Msg,), {})
    fakes["mavros_msgs.msg"].Mavlink = type("Mavlink", (R._Msg,), {})
    saved = {k: sys.modules.get(k) for k in fakes}
    sys.modules.update(fakes)
    try:
        spec = importlib.util.spec_from_file_location("ros2_shield_node_fix",
                                                      ROOT / "sitl" / "ros2_shield_node.py")
        N = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(N)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return R, N


def _node(R, N, argv):
    clk = R._Clock()
    N.time = clk
    # Ticked by hand (R.HAND_TICKED: the 10 Hz timer's tick, CPython's own GC).
    args, _ = N.make_parser().parse_known_args(["--policy", str(POLICY), "--dev-yaml"]
                                               + R.HAND_TICKED + argv)
    out = Path(tempfile.mkdtemp(prefix="ros2fix_"))
    node = N.ShieldNode(args, out)
    node.state = State(x=5.0, y=15.0, up=15.0, yaw_deg=0.0)
    return node, clk, out


def test_the_node_hands_the_shield_its_fix_and_takes_x8():
    R, N = _node_module()
    assert N.NavSatFix is not None and N.ParamEvent is not None
    with tempfile.TemporaryDirectory() as td:
        meta = Path(td) / "scenario.json"
        meta.write_text(json.dumps({"run_nonce": "abcdef0123456789", "run_id": "t"}),
                        encoding="utf-8")
        node, clk, out = _node(R, N, [f"--scenario-meta={meta}"])
    node.start_mission()
    texts = node.pubs[N.STATUSTEXT_SEND_TOPIC].sent
    assert [m.text for m in texts] == ["GRDL T0 abcdef01 t=0.000"]
    for k in range(60):
        status = 0 if k < 10 else -1                 # the fix lost at 1 s
        node._on_fix(N.NavSatFix(status=R._Msg(status=status)))
        if k == 15:
            node._on_param_event(N.ParamEvent(param_id="SIM_GPS_DISABLE", value=R._Msg(
                type=2, integer_value=1, double_value=0.0, string_value="")))
            node._on_param_event(N.ParamEvent(param_id="FENCE_ENABLE", value=R._Msg(
                type=2, integer_value=1, double_value=0.0, string_value="")))
        node._on_pose(R._pose_msg(N, node.state))
        R._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()
    rows = node.rows
    assert [r["fix_ok"] for r in rows[:9]] == [True] * 9 and rows[12]["fix_ok"] is False
    x8 = [r for r in rows if r.get("fsm_edge") == "X8"]
    assert x8 and x8[0]["position_lost"] and 3.0 <= x8[0]["t"] <= 3.5, \
        [(r["t"], r.get("fsm_edge"), r.get("position_lost")) for r in rows[25:40]]
    assert "RTL" in [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]
    # The repeats of the t = 0 marker carry the mission time they were sent
    # at: the first tick at or after each of MARKER_TIMES_S.
    stated = [float(m.text.split("t=")[1]) for m in texts]
    assert len(stated) == len(SST.MARKER_TIMES_S) and stated[0] == 0.0, stated
    assert all(0.0 <= s - n <= 0.1 + 1e-6 for s, n in zip(stated, SST.MARKER_TIMES_S)), stated
    node._finish("test over")
    met = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    fq = met["fix_quality"]
    assert fq["messages"] == 60 and fq["ticks_no_fix"] > 0 and fq["position_lost_ticks"] > 0
    assert fq["pos_sigma_m"] is None and "policy decision" in fq["pos_sigma_m_why"]
    sp = met["sim_params"]
    assert [(e["param"], e["value"]) for e in sp["events"]] == [("SIM_GPS_DISABLE", 1.0)]
    assert 1.5 <= sp["events"][0]["t"] <= 1.7 and sp["markers_sent"][0] == 0.0
    shutil.rmtree(out, ignore_errors=True)


def test_a_node_without_the_fix_topic_flies_as_before():
    R, N = _node_module()
    node, clk, out = _node(R, N, [])
    node.start_mission()
    assert node.pub_text is None, "no scenario, no t = 0 marker"
    for _ in range(5):
        node._on_pose(R._pose_msg(N, node.state))
        R._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()
    assert all("fix_ok" not in r and "position_lost" not in r for r in node.rows)
    assert all("estimator_ok" not in r and "ekf_flags" not in r for r in node.rows)
    node._finish("test over")
    met = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    assert met["fix_quality"]["messages"] == 0 and met["fix_quality"]["ticks_no_fix"] == 0
    assert met["fix_quality"]["estimator"]["messages"] == 0
    shutil.rmtree(out, ignore_errors=True)


# --------------------------------------------------------------------------- #
# the autopilot's own estimator behind State.estimator_ok (2026-11-12)
# --------------------------------------------------------------------------- #

def _ekf_words(flags=0x33F, ph=0.007, vv=0.022, airspeed=0.0):
    """EKF_STATUS_REPORT as MAVLink 2 carries it (trailing zero bytes cut)
    and mavros_msgs/Mavlink holds it: (len, payload64)."""
    import struct
    raw = struct.pack("<5fHf", vv, ph, 0.11, 0.03, 0.0, flags, airspeed).rstrip(b"\0")
    pad = raw.ljust((len(raw) + 7) // 8 * 8, b"\0")
    return len(raw), list(struct.unpack(f"<{len(pad) // 8}Q", pad))


def test_ekf_status_report_is_read_from_the_raw_mavlink_feed():
    """No MAVROS 2 plugin publishes ArduPilot's EKF_STATUS_REPORT (193), so
    the node reads it from /uas1/mavlink_source: the autopilot's own report
    only (system 1, component 1), fields in MAVLink's size order, a payload
    MAVLink 2 truncated restored. Healthy is what ArduCopter navigates on
    (EKF_POS_HORIZ_ABS, not EKF_CONST_POS_MODE), initialised, and the
    position and velocity test ratios below FS_EKF_THRESH."""
    import ros2_shield_node as N
    assert N.MAVLINK_SOURCE_TOPIC == "/uas1/mavlink_source" and N.EKF_STATUS_REPORT_ID == 193
    n, words = _ekf_words()
    f = N.ekf_status_from_mavlink(193, 1, 1, n, words)
    assert f["flags"] == 0x33F and math.isclose(f["pos_horiz_variance"], 0.007, rel_tol=1e-6)
    assert math.isclose(f["velocity_variance"], 0.022, rel_tol=1e-6)
    n25, w25 = _ekf_words(flags=0x25, ph=0.0, vv=0.0)   # the rig's 7 s timeout report
    assert n25 < 22 and N.ekf_status_from_mavlink(193, 1, 1, n25, w25)["flags"] == 0x25
    assert N.ekf_status_from_mavlink(24, 1, 1, n, words) is None          # GPS_RAW_INT
    assert N.ekf_status_from_mavlink(193, 246, 25, n, words) is None      # the stressor
    assert N.ekf_status_from_mavlink(193, 1, 191, n, words) is None
    assert N.ekf_status_from_mavlink(193, 1, 1, n, ["x"]) is None
    ok = N.ekf_position_healthy
    assert ok(f) == (True, "absolute horizontal position valid")
    assert not ok({**f, "flags": 0x25})[0] and "EKF_POS_HORIZ_ABS" in ok({**f, "flags": 0x25})[1]
    assert not ok({**f, "flags": 0x33F | 128})[0] and not ok({**f, "flags": 0x33F | 1024})[0]
    assert ok({**f, "flags": 0x33F | N.EKF_GPS_GLITCHING})[0], "the autopilot flies through it"
    assert not ok({**f, "pos_horiz_variance": 0.8})[0] and ok({**f, "pos_horiz_variance": 0.79})[0]
    assert not ok({**f, "velocity_variance": float("nan")})[0]
    assert ok({**f, "pos_horiz_variance": 0.9}, thresh=1.0)[0]
    assert N.ekf_thresh_from(None)[0] == 0.8 and "not read" in N.ekf_thresh_from(None)[1]
    assert N.ekf_thresh_from(0.0)[0] == 0.8 and N.ekf_thresh_from(0.5) == (
        0.5, "FS_EKF_THRESH (read from the autopilot)")


def test_the_estimator_verdict_is_the_autopilots_own_failsafe_rule():
    """EstimatorHealth: True while the autopilot vouches for its estimate;
    False once unhealthy reports outweigh healthy ones by 1 s (ArduCopter's
    ekf_check: 10 failing 10 Hz checks, each passing one taking one back),
    until they are paid back; None before any report or once the last is
    older than the pose timeout. The rig's 7 s GPS denials dropped the
    absolute position for one or two reports before the GPS returned: never
    a verdict."""
    import ros2_shield_node as N
    f = N.ekf_status_from_mavlink(193, 1, 1, *_ekf_words())
    bad = {**f, "flags": 0x25}
    h = N.EstimatorHealth()
    assert h.verdict(0.0) is None
    t = 0.0
    for _ in range(68):                       # 6.8 s of a denial: healthy
        h.report(t, f)
        assert h.verdict(t + 0.05) is True
        t += 0.1
    for _ in range(2):                        # EKF3's 7 s timeout, as on the rig
        h.report(t, bad)
        assert h.verdict(t + 0.05) is True
        t += 0.1
    for _ in range(5):                        # the GPS back
        h.report(t, f)
        t += 0.1
    assert h.bad_s < 1e-9 and h.verdict(t) is True and len(h.spells) == 1
    assert h.spells[0]["reports"] == 2 and not h.declared_spells
    verdicts = []
    for _ in range(12):                       # position lost for good
        h.report(t, bad)
        verdicts.append(h.verdict(t + 0.01))
        t += 0.1
    assert verdicts[:9] == [True] * 9 and verdicts[9:] == [False] * 3, verdicts
    back = []
    for _ in range(12):                       # paid back report by report
        h.report(t, f)
        back.append(h.verdict(t + 0.01))
        t += 0.1
    assert back[:9] == [False] * 9 and back[9:] == [True] * 3, back
    assert h.verdict(t + N.EKF_STALE_S + 0.01) is None, "a stale report is not a verdict"
    s = h.summary(to_mission_t=lambda v: v - 1.0)
    assert s["messages"] == 99 and s["unhealthy_reports"] == 14 and len(s["declared_spells"]) == 1
    assert s["declared_spells"][0]["t_end"] is not None and s["thresh"] == 0.8
    h.set_thresh(0.5)
    assert h.thresh == 0.5 and "read from the autopilot" in h.thresh_source
    for bad_arg in ({"failsafe_s": 0}, {"stale_s": float("nan")}, {"thresh": -1}):
        try:
            N.EstimatorHealth(**bad_arg)
        except ValueError:
            pass
        else:
            raise AssertionError(f"{bad_arg} accepted")


def test_the_node_hands_the_shield_the_autopilots_verdict_and_x8_waits_for_it():
    """On the node: the GPS fix lost at 1 s while the autopilot keeps
    vouching for its estimate (EKF_STATUS_REPORT healthy) - no X8, where
    the old law alone took it at 3.0-3.5 s (the test above); the autopilot
    then reports its position lost from 4 s on, and X8 comes with its own
    failsafe's rule, 1 s later, naming the autopilot."""
    R, N = _node_module()
    assert N.Mavlink is not None
    node, clk, out = _node(R, N, [])
    node.start_mission()
    n, good = _ekf_words()
    nb, bad = _ekf_words(flags=0x25)
    for k in range(70):
        node._on_fix(N.NavSatFix(status=R._Msg(status=0 if k < 10 else -1)))
        node._on_mavlink(N.Mavlink(msgid=33, sysid=1, compid=1, len=28, payload64=[0] * 4))
        node._on_mavlink(N.Mavlink(msgid=193, sysid=1, compid=1,
                                   **({"len": n, "payload64": good} if k < 40
                                      else {"len": nb, "payload64": bad})))
        node._on_pose(R._pose_msg(N, node.state))
        R._act(N, node, vx=0.0)
        clk.t += 0.1
        node._tick()
    rows = node.rows
    assert rows[20]["fix_ok"] is False and rows[20]["estimator_ok"] is True
    assert rows[20]["ekf_flags"] == 0x33F and rows[20]["ekf_age_ms"] == 100.0
    x8 = [r for r in rows if r.get("fsm_edge") == "X8"]
    assert x8, [(r["t"], r.get("estimator_ok"), r.get("position_lost")) for r in rows[35:55]]
    assert 4.8 <= x8[0]["t"] <= 5.2 and "autopilot reports" in x8[0]["position_lost"], \
        (x8[0]["t"], x8[0]["position_lost"])
    assert all(r.get("position_lost") is None for r in rows if r["t"] < x8[0]["t"] - 1e-6)
    assert "RTL" in [m.data for m in node.pubs[N.TOPIC_MODE_REQUEST].sent]
    node._finish("test over")
    met = json.loads((out / "metrics.json").read_text(encoding="utf-8"))
    est = met["fix_quality"]["estimator"]
    assert est["topic"] == "/uas1/mavlink_source" and est["messages"] == 70
    assert est["unhealthy_reports"] == 30 and len(est["declared_spells"]) == 1
    assert est["ticks_lost"] > 0 and est["ticks_ok"] >= 40 and est["thresh"] == 0.8
    assert 3.9 <= est["unhealthy_spells"][0]["t_start"] <= 4.1, est["unhealthy_spells"]
    report = (out / "report.md").read_text(encoding="utf-8")
    assert "Autopilot estimator" in report and "70 report(s)" in report
    shutil.rmtree(out, ignore_errors=True)


# --------------------------------------------------------------------------- #
# the rig: the scripts, the transport, the hil campaign
# --------------------------------------------------------------------------- #

def test_the_desktop_side_starts_the_stressors_before_it_says_ready():
    hil = (ROOT / "sitl" / "hil_desktop.sh").read_text(encoding="utf-8")
    assert "\r\n" not in hil
    assert "--stress-plan)" in hil and "--stress-record)" in hil
    start = hil.index('"$STRESS_PY" "$DIR/sitl_stressors.py"')
    assert hil.index("^SITL_STRESS BOOT OK", start) < hil.index("HIL_DESKTOP READY", start)
    assert 'pkill -TERM -f "[s]itl_stressors.py"' in hil
    demo = (ROOT / "sitl" / "run_ros2_demo.sh").read_text(encoding="utf-8")
    assert "\r\n" not in demo
    s = demo.index('"$STRESS_PY" "$DIR/sitl_stressors.py"')
    assert demo.index("^SITL_STRESS BOOT OK", s) < demo.index("=== mavros", s)
    assert "SITL_STRESS_PLAN needs ROUTER=1" in demo
    assert 'pkill -TERM -f "[s]itl_stressors.py"' in demo


def test_the_transport_hands_the_plan_to_the_wsl_helper():
    calls = []

    class _Popen:
        def __init__(self, cmd, **kw):
            calls.append(cmd)
            self.stdout = iter(["HIL_DESKTOP READY sitl_pid=1\n"])

        def terminate(self):
            pass

    t = S.SubprocessTransport(S.HilConfig(wsl_repo="/mnt/x/repo"),
                              run=lambda *a, **k: types.SimpleNamespace(
                                  returncode=0, stdout="", stderr=""), popen=_Popen)
    plan = ROOT / S.RAIL_STRESS_DIR / "m1" / "plan.json"
    t.restart_sitl("-35.1,149.2,584,0.00", stress=(plan, plan.with_name("record.json")))
    cmd = calls[-1]
    assert cmd[cmd.index("--stress-plan") + 1] == "/mnt/x/repo/demo/out/_stress/m1/plan.json"
    assert cmd[cmd.index("--stress-record") + 1] == "/mnt/x/repo/demo/out/_stress/m1/record.json"
    t.restart_sitl(None)
    assert "--stress-plan" not in calls[-1]


def test_the_desktop_rail_passes_the_plan_through_wslenv():
    spec = _cell("gps-dropout", lane_y=15.0, window=[4.0, 11.0])
    seen = {}

    def run(cmd, **kw):
        seen.update(kw.get("env") or {})
        return types.SimpleNamespace(returncode=2, stdout="", stderr="refused (test)")
    saved = (S.sitl_available, S.subprocess.run)
    S.sitl_available, S.subprocess.run = (lambda: True), run
    try:
        try:
            S.run_sitl(spec, "tstwslenv", 1001, DEFAULTS)
        except RuntimeError as e:
            assert "SITL run failed" in str(e)
    finally:
        S.sitl_available, S.subprocess.run = saved
        shutil.rmtree(ROOT / S.RAIL_STRESS_DIR / "tstwslenv", ignore_errors=True)
        shutil.rmtree(ROOT / S.CAMPAIGN_DIR / "tstwslenv", ignore_errors=True)
    assert seen["SITL_STRESS_PLAN"].endswith("demo/out/_stress/tstwslenv/plan.json")
    assert seen["SITL_STRESS_RECORD"].endswith("demo/out/_stress/tstwslenv/record.json")
    assert {"SITL_HOME/u", "SITL_STRESS_PLAN/u", "SITL_STRESS_RECORD/u"} <= set(
        seen["WSLENV"].split(":")), seen["WSLENV"]


class _StressRig:
    """A hil rig whose desktop side DOES set the plan: restart_sitl reads
    the plan and writes the record the simulator side would, and the
    "Orin" flies the cell headless and leaves the rows (fix_ok) and
    metrics (sim_params) the Shield node would on SITL."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.calls: list[tuple] = []
        self.remote = Path(tempfile.mkdtemp(prefix="fake_orin_"))
        self.plan = None

    def restart_sitl(self, home, stress=None):
        self.calls.append(("restart_sitl", home, stress is not None))
        self.plan = None
        if stress is not None:
            self.plan = json.loads(Path(stress[0]).read_text(encoding="utf-8"))
            rec, _, _ = _flight(self.plan)
            Path(stress[1]).write_text(json.dumps(rec), encoding="utf-8")
        return "HIL_DESKTOP READY"

    def stop_sitl(self):
        self.calls.append(("stop_sitl",))

    def push(self, local_dir, remote_rel):
        shutil.copytree(local_dir, self.remote / remote_rel)

    def restart_mavros(self):
        pass

    def run_mission(self, args, tag, shield, timeout_s):
        val = next(a.split("=", 1)[1] for a in args if a.startswith("--scenario-meta="))
        meta = json.loads((self.remote / val).read_text(encoding="utf-8"))
        cell = next(c for c in FAMILIES[meta["scenario_id"]].cells(samples=3)
                    if c.episode_id == meta["cell"])
        run = S.run_episode(cell.spec, DEFAULTS, meta["seed"])
        _, met, frows = _flight(self.plan, last_t=run.rows[-1]["t"])
        fix = {r["t"]: r.get("fix_ok") for r in frows}
        out = self.remote / "demo" / "out" / tag
        out.mkdir(parents=True)
        ep = 7
        (out / "flight_log.jsonl").write_text("".join(
            json.dumps(S._json_safe(dict(r, flown=r.get("owner") != "autopilot",
                                         emitted_body=r["emitted"], episode_id=ep,
                                         fix_ok=fix.get(round(r["t"], 3), True))))
            + "\n" for r in run.rows), encoding="utf-8")
        (out / "metrics.json").write_text(json.dumps({
            "episode_id": ep, "shield": shield, "reached": run.extra["reached_goal"],
            "fsm": run.extra.get("fsm"), "scenario": meta,
            "scenario_family": meta["family"], **met}), encoding="utf-8")
        (out / "events.jsonl").write_text(
            json.dumps({"kind": "episode_start", "episode_id": ep}) + "\n"
            + json.dumps({"kind": "mission_end"}) + "\n", encoding="utf-8")
        (out / "policy_g0.json").write_bytes(run.policy.canonical_bytes())
        return 0, f"fake mission {tag}"

    def fetch(self, tag, local_parent):
        local_parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(self.remote / "demo" / "out" / tag, local_parent / tag)
        return local_parent / tag


def test_a_hil_campaign_flies_a_dropout_cell_whose_stress_was_realised():
    """--backend hil end to end on a rig whose simulator side sets the plan:
    the dropout cells are flown and scored (no longer refused, never an
    error), the stress plan goes to the desktop side only, and
    scenario_score.json carries every SIM_* parameter in the pymavlink rail's
    record with the time the node saw it."""
    from guardrail.bundle import write_bundle
    from guardrail.manifest import flight_bundle_path
    rigs = []
    saved = S.make_hil_transport
    S.make_hil_transport = lambda c: rigs.append(_StressRig(c)) or rigs[-1]
    try:
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            b = td / "bundles"
            for name in ("sim_demo_policy", "corridor_survey", "sitl_pedestrian",
                         "wgs84_taipei"):
                pol = load_policy(ROOT / "policies" / f"{name}.yaml")
                write_bundle(pol, flight_bundle_path(pol, b), signer=None)
            cfg = td / "hil.json"
            cfg.write_text(json.dumps({"store": str(td / "store"), "bundles_dir": str(b)}),
                           encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                rc = S.main(["--backend", "hil", "--hil-config", str(cfg), "--seed", "1001",
                             "--only", "gps-dropout", "--hil-campaign", "tstsr",
                             "--out", str(td / "s.json"), "--no-bundles"])
            doc = json.loads((td / "s.json").read_text(encoding="utf-8"))
            res = [r for r in doc["results"]]
            assert res and all(r["status"] not in ("skipped", "error") for r in res), \
                [(r["id"], r["status"], r.get("error") or r.get("reason")) for r in res]
            assert all(c[2] for c in rigs[0].calls if c[0] == "restart_sitl")
            score = json.loads((td / "store" / res[0]["mission_tag"] / S.SCENARIO_SCORE)
                               .read_text(encoding="utf-8"))
            ents = score["sim_stressors"]
            assert [(e["param"], e["set"], e["applied"]) for e in ents] == [
                ("SIM_GPS_DISABLE", 0.0, True), ("SIM_GPS_DISABLE", 1.0, True),
                ("SIM_GPS_DISABLE", 0.0, True)], ents
            assert score["sim_stressors_check"]["problems"] == []
            assert (td / "store" / res[0]["mission_tag"] / "sitl_stress.json").is_file()
            assert score["kpi"]["gps_dropout_ticks"] > 0
            assert rc in (0, 1)
    finally:
        S.make_hil_transport = saved
        for d in (ROOT / S.CAMPAIGN_DIR).glob("tstsr*"):
            shutil.rmtree(d, ignore_errors=True)
        for d in (ROOT / S.RAIL_STRESS_DIR).glob("tstsr*"):
            shutil.rmtree(d, ignore_errors=True)


def test_the_pymavlink_rail_and_the_simulator_side_share_one_rule_and_record():
    src = (ROOT / "sitl" / "run_sitl_demo.py").read_text(encoding="utf-8")
    assert "from sitl_stressors import" in src and "stressor_entry(name, want, got)" in src
    assert "with_uniform_wind(parse_sim_params(" in src
    e = SST.stressor_entry("SIM_WIND_SPD", 8, 8.0004)
    assert list(e)[:4] == ["param", "set", "read_back", "applied"] and e["applied"]
    assert not SST.stressor_entry("SIM_WIND_SPD", 8, 7.9)["applied"]
    assert not SST.stressor_entry("SIM_WIND_SPD", 8, None)["applied"]
    assert SST.read_back_ok(2000.0, 2001.5) and not SST.read_back_ok(2000.0, 2003.0)
    assert set(S.SIM_NEUTRAL) <= set(SST.WATCHED_SIM_PARAMS)


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
        except (Exception, SystemExit) as e:             # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
