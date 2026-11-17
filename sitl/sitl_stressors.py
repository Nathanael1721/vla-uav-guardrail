"""
The simulator's stressors for one rail mission: ArduPilot SITL's own SIM_*
parameters, set from the SIMULATOR'S side of the rig - this process, on the
desktop beside SITL - never by a node of the system under test.

    ~/venv-ap/bin/python sitl/sitl_stressors.py --plan PLAN.json --record RECORD.json \\
        [--link tcp:127.0.0.1:5760]

sitl/hil_desktop.sh (`start --stress-plan ... --stress-record ...`, the hil
rail) and sitl/run_ros2_demo.sh (SITL_STRESS_PLAN / SITL_STRESS_RECORD, the
desktop rail) start it once SITL is up; experiments/sweep_scenarios.py
(rail_stress_plan) writes the plan and checks the record after the mission
(rail_stress_check). Contract item X-14: wind and GPS denial in the parameter
sweeps, flown on ArduPilot's own physics.

WHAT IT DOES

  1. Connects to SITL's SERIAL0 (tcp:5760; the rails' router uses SERIAL2,
     so this link is free with ROUTER=1), waits for the autopilot's
     heartbeat and sends its own, once a second, as a system of its own
     (SCHED_SYSID / SCHED_COMPID, MAV_TYPE_GCS; not ArduPilot's SYSID_MYGCS
     255, so it is never taken for the operator's ground station).
  2. BOOT: sets every boot parameter of the plan with PARAM_SET and reads
     each back (the PARAM_VALUE echo, then a PARAM_REQUEST_READ), then
     prints `SITL_STRESS BOOT OK`; a parameter that does not read back
     prints `SITL_STRESS BOOT FAILED ...` and exits 1, and the helpers then
     refuse the mission before anything flies.
  3. T0: the episode's t = 0 is the Shield node's mission start, on the
     node's own clock. The node announces it in a STATUSTEXT through MAVROS
     (`GRDL T0 <nonce8> t=<mission seconds>`, marker_text; at t = 0 and once
     a second for MARKER_TIMES_S). The text has no target, so ArduPilot
     forwards it to every link with a learned route
     (libraries/GCS_MAVLink/MAVLink_routing.cpp, check_and_forward), this one
     included once its heartbeat taught the route. The node does not know
     the plan: it announces its clock on every scenario mission. t0 on this
     process's clock is min(receipt - t) over the markers carrying the
     plan's nonce: never earlier than the node's own t = 0, so a change is
     never applied before its time, and late by the marker's one-way delay.
  4. SCHEDULE: each scheduled change at t0 + t, PARAM_SET and its echo read
     back, and the times relative to t0 recorded. The Shield node records
     the autopilot's own PARAM_VALUE broadcast of each change on ITS clock
     (/mavros/param/event, metrics.json `sim_params`), which is the
     alignment the harness checks against the plan.
  5. RECORD (RECORD_SCHEMA): rewritten after every step, so a mission that
     ends early, or a `stop` that kills this process, leaves what happened.
     It exits 0 once the schedule is done (or on SIGTERM), 1 on a failure;
     with parameters to restore it holds after the schedule until SIGTERM,
     restores them, then exits (THE BATTERY CELL'S FAIL-SAFE SETUP).

WHICH PARAMETERS (ArduCopter 4.5.7, libraries/SITL/SITL.cpp; the harness's
mapping and its source lines are in experiments/sweep_scenarios.py, "THE
SIMULATOR'S STRESSORS"): SIM_WIND_SPD, SIM_WIND_DIR (the direction the wind
comes FROM), SIM_WIND_T (1: no altitude profile), SIM_WIND_TURB,
SIM_GPS_DISABLE and SIM_BATT_VOLTAGE (since 2026-11-17). Only SIM_*
parameters are accepted as stressors (load_plan): a stressor changes the
simulated world, never the firmware under test.

THE BATTERY CELL'S FAIL-SAFE SETUP (2026-11-17, WP3-07 G6). A falling
SIM_BATT_VOLTAGE alone makes ArduPilot declare nothing: its battery monitor
reaches CRITICAL only below BATT_CRT_VOLT (or BATT_CRT_MAH), and for a
voltage only after BATT_LOW_TIMER seconds (AP_BattMonitor_Backend::
update_failsafes) - and the copter's defaults leave BATT_CRT_VOLT and
BATT_CRT_MAH at 0, disabled. So a plan may also carry `firmware`: the battery
fail-safe configuration of the SIMULATED vehicle (FIRMWARE_SITL_PARAMS
only, the setup a real vehicle's battery would be given: BATT_CRT_VOLT
10.1, the 10.0 V drop and no low or critical action of the autopilot's own
as in ArduCopter's autotest BatteryFailsafe; BATT_LOW_VOLT 10.5 is the
copter default; BATT_LOW_TIMER 1 s is the cell's own choice, a lab
vehicle-setup choice of 2026-11-17, where the default is 10 s and the
autotest leaves it there), set at boot before the stressors and restored
when this process is stopped. Guarded three ways,
because these names exist on a real flight controller too: the link must be
a loopback tcp/udp one (firmware_link_ok: never a serial port), the
autopilot must answer SIM_BATT_VOLTAGE (a hardware autopilot has no SIM_*
parameter) before any of them is set, and the value each held before is
read first - one that does not read is never changed (the boot fails). Every
parameter of `firmware`, and every boot SIM_* parameter marked
`restore_after`, is set back to that value on SIGTERM (`stop`), after the
schedule is done (the process then holds until stopped) or after a failed
boot, and the record says what was restored.

ONE MECHANISM WITH THE PYMAVLINK RAIL'S (sitl/run_sitl_demo.py --sim-param,
since 2026-11-02). That rail is the Shield's own pymavlink process, so it sets
its static stressors itself at mission start; the ROS 2 rails' Shield node is
the system under test, so this process sets them from the simulator's side
and on the episode's clock. Both use what is defined here: the parameter rule
(parse_sim_params, moved here from run_sitl_demo.py unchanged), the read-back
test (read_back_ok, the same 1e-3 x max(1, |value|)) and the record of each
parameter (stressor_entry: `param`, `set`, `read_back`, `applied`, the keys
run_sitl_demo.py writes into metrics.json `sim_stressors`), and the uniform
wind (with_uniform_wind: a wind speed comes with SIM_WIND_T = 1, see below).

THE WIND'S ALTITUDE PROFILE. SIM_WIND_T defaults to 0, a square law: below
SIM_WIND_T_ALT (60 m) SITL scales the wind by sqrt(altitude / 60 m)
(AP_HAL_SITL/SITL_State.cpp:443-446), half the speed at 15 m. The 2 Nov
pymavlink runs (sitl_wind8_shield_on/off_20261102) set SIM_WIND_SPD 8 without
it, so at their 15 m the wind was 4 m/s, not the 8 their record names. Both
rails now set SIM_WIND_T = 1 with a wind speed unless told otherwise.

Needs pymavlink (~/venv-ap has the version sitl/setup_sitl.sh pins) and the
standard library only; the plan logic below imports neither, so the harness
and the tests read it on any host.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import sys
import time
from pathlib import Path

PLAN_SCHEMA = "sitl-stress-plan/1"
RECORD_SCHEMA = "sitl-stress-record/1"
DEFAULT_LINK = "tcp:127.0.0.1:5760"
# This process on the MAVLink network: a GCS-type component of its own system
# (ArduPilot's SYSID_MYGCS is 255, MAVROS's defaults are 1/191).
SCHED_SYSID = 246
SCHED_COMPID = 25                 # MAV_COMP_ID_USER1
AP_SYSID, AP_COMPID = 1, 1
PARAM_TOL = 1e-3
# The episode's t = 0 marker the Shield node sends through MAVROS
# (/mavros/statustext/send): "GRDL T0 <first 8 of the run nonce> t=<s>".
# STATUSTEXT carries 50 characters; this is 24.
MARKER_PREFIX = "GRDL T0"
MARKER_RE = re.compile(r"^GRDL T0 ([0-9A-Za-z]{1,16}) t=(-?[0-9]+(?:\.[0-9]+)?)\s*$")
# When the node sends it, seconds of mission time: t = 0 and four repeats, so
# a lost UDP datagram costs at most the second it takes the next to arrive
# (and a change due at t = 0 is then late by it, which the harness refuses).
MARKER_TIMES_S = (0.0, 1.0, 2.0, 3.0, 4.0)
# The parameters a plan may name, and the ones the Shield node reads back at
# take-off and records when the autopilot broadcasts a change
# (sitl/ros2_shield_node.py, metrics.json `sim_params`).
WATCHED_SIM_PARAMS = ("SIM_WIND_SPD", "SIM_WIND_DIR", "SIM_WIND_T", "SIM_WIND_TURB",
                      "SIM_GPS_DISABLE", "SIM_BATT_VOLTAGE")
# The simulated vehicle's battery fail-safe configuration a plan may set at
# boot (`firmware`; module docstring, THE BATTERY CELL'S FAIL-SAFE SETUP):
# never a stressor, never on anything but SITL, always restored.
FIRMWARE_SITL_PARAMS = ("BATT_LOW_VOLT", "BATT_CRT_VOLT", "BATT_LOW_TIMER",
                        "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT")
# Answers only on SITL: read before any firmware parameter is set.
SITL_PROBE_PARAM = "SIM_BATT_VOLTAGE"
# A restore at SIGTERM must fit the helpers' wait before they kill SITL
# (sitl/hil_desktop.sh stop_all, sitl/run_ros2_demo.sh cleanup: 5 s).
RESTORE_ECHO_TIMEOUT_S = 0.25
RESTORE_TRIES = 2
# How long a scheduled change may wait for its echo before it is sent again.
ECHO_TIMEOUT_S = 0.4
SET_TRIES = 3


# --------------------------------------------------------------------------- #
# The plan and the marker: no pymavlink, read by the harness and the tests
# --------------------------------------------------------------------------- #

def marker_text(nonce: str, t: float) -> str:
    """The node's t = 0 marker for the run whose nonce is `nonce`, sent at
    mission time `t` (seconds)."""
    return f"{MARKER_PREFIX} {str(nonce)[:8]} t={float(t):.3f}"


def parse_marker(text) -> tuple[str, float] | None:
    """(nonce8, mission seconds) of a marker STATUSTEXT, or None."""
    if isinstance(text, (bytes, bytearray)):
        text = bytes(text).decode("ascii", "replace")
    m = MARKER_RE.match(str(text or "").rstrip("\x00").strip())
    if not m:
        return None
    t = float(m.group(2))
    return (m.group(1), t) if math.isfinite(t) else None


def t0_estimate(markers: list[tuple[float, float]]) -> float | None:
    """The node's t = 0 on this process's clock from [(receipt, t stated)]:
    the earliest receipt - t, i.e. the least delayed marker. Never earlier
    than the node's own t = 0 (a receipt follows its send)."""
    vals = [float(rx) - float(t) for rx, t in markers]
    return min(vals) if vals else None


def _finite_number(v) -> bool:
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(float(v)))


def parse_sim_params(items: list[str] | None) -> dict[str, float]:
    """`--sim-param NAME=VALUE` (repeatable): the simulator's own stressors.

    Only SITL's SIM_* parameters are accepted (wind SIM_WIND_SPD / _DIR /
    _TURB, GPS SIM_GPS_DISABLE / SIM_GPS_GLITCH_X ...): they change the
    simulated world, never the firmware under test, which is what the
    stressor runs of X-14 need (until 2026-11-02 wind and GPS loss acted on
    the headless kinematic model only). Anything else is refused here.
    (sitl/run_sitl_demo.py's own until 2026-11-07, moved here unchanged so
    the pymavlink rail and the simulator side of the ROS 2 rails share it.)"""
    out: dict[str, float] = {}
    for it in items or []:
        name, sep, val = it.partition("=")
        name = name.strip().upper()
        if not sep or not name.startswith("SIM_") or not name[4:]:
            raise ValueError(f"--sim-param {it!r}: give SIM_<NAME>=<number>; only the "
                             f"simulator's SIM_* parameters are stressors")
        try:
            v = float(val)
        except ValueError:
            raise ValueError(f"--sim-param {it!r}: {val!r} is not a number") from None
        if v != v or v in (float("inf"), float("-inf")):
            raise ValueError(f"--sim-param {it!r}: not a finite number")
        out[name] = v
    return out


def with_uniform_wind(params: dict[str, float]) -> dict[str, float]:
    """`params` with SIM_WIND_T = 1 (no altitude profile) added when they set
    a wind speed and say nothing of the profile: SITL's default square law
    would scale the speed by sqrt(altitude / 60 m) below 60 m (module
    docstring, "THE WIND'S ALTITUDE PROFILE"). A profile given is kept."""
    out = dict(params)
    if "SIM_WIND_SPD" in out and "SIM_WIND_T" not in out:
        out["SIM_WIND_T"] = 1.0
    return out


def read_back_ok(want: float, got) -> bool:
    """Did a parameter read back as set? Within 1e-3 x max(1, |want|): the
    pymavlink rail's rule since 2026-11-02, shared."""
    try:
        return got is not None and abs(float(got) - float(want)) <= PARAM_TOL * max(
            1.0, abs(float(want)))
    except (TypeError, ValueError):
        return False


def stressor_entry(param: str, want: float, got, **more) -> dict:
    """One stressor parameter as both mechanisms record it: `param`, `set`,
    `read_back`, `applied` (read_back_ok), then anything more (when, the
    echo, why)."""
    return {"param": param, "set": float(want),
            "read_back": None if got is None else float(got),
            "applied": read_back_ok(want, got), **more}


def load_plan(source) -> dict:
    """A plan (PLAN_SCHEMA) from a path or a dict, validated: every
    parameter one of WATCHED_SIM_PARAMS with a finite value (the shared rule,
    parse_sim_params), every scheduled time finite and >= 0, the schedule in
    time order. Raises ValueError."""
    if isinstance(source, dict):
        plan = source
    else:
        plan = json.loads(Path(source).read_text(encoding="utf-8"))
    if not isinstance(plan, dict) or plan.get("schema") != PLAN_SCHEMA:
        raise ValueError(f"not a {PLAN_SCHEMA} plan")
    nonce = plan.get("run_nonce")
    if not isinstance(nonce, str) or not re.fullmatch(r"[0-9A-Za-z]{8,64}", nonce):
        raise ValueError(f"run_nonce {nonce!r}: 8-64 letters or digits")
    for key in ("boot", "schedule"):
        items = plan.get(key)
        if not isinstance(items, list):
            raise ValueError(f"{key}: expected a list")
        for it in items:
            if not isinstance(it, dict):
                raise ValueError(f"{key}: an entry is not an object")
            name = it.get("param")
            if name not in WATCHED_SIM_PARAMS:
                raise ValueError(f"{key}: {name!r} is not a stressor parameter "
                                 f"({', '.join(WATCHED_SIM_PARAMS)})")
            if not _finite_number(it.get("value")):
                raise ValueError(f"{key}: {name} = {it.get('value')!r} is not a finite number")
            parse_sim_params([f"{name}={it['value']}"])
            if key == "schedule":
                t = it.get("t")
                if not _finite_number(t) or float(t) < 0:
                    raise ValueError(f"schedule: {name} at t={t!r}: a time >= 0 s")
    ts = [float(it["t"]) for it in plan["schedule"]]
    if ts != sorted(ts):
        raise ValueError("schedule: not in time order")
    fw = plan.get("firmware", [])
    if not isinstance(fw, list):
        raise ValueError("firmware: expected a list")
    for it in fw:
        if not isinstance(it, dict):
            raise ValueError("firmware: an entry is not an object")
        name = it.get("param")
        if name not in FIRMWARE_SITL_PARAMS:
            raise ValueError(f"firmware: {name!r} is not a battery fail-safe parameter of "
                             f"the simulated vehicle ({', '.join(FIRMWARE_SITL_PARAMS)})")
        if not _finite_number(it.get("value")):
            raise ValueError(f"firmware: {name} = {it.get('value')!r} is not a finite number")
    names = [it["param"] for it in fw] + [it["param"] for it in plan["boot"]]
    if len(names) != len(set(names)):
        raise ValueError("boot / firmware: a parameter is named twice")
    return plan


def firmware_link_ok(link: str) -> bool:
    """May a firmware parameter be set over `link`? Only a loopback tcp/udp
    link (SITL beside this process, tcp:127.0.0.1:5760 by default): never a
    serial device (COM8, /dev/ttyACM0 - a real flight controller) nor another
    host."""
    m = re.fullmatch(r"(tcp|udp|udpin|udpout):(127\.0\.0\.1|localhost):(\d{1,5})",
                     str(link or "").strip())
    return m is not None


def restore_entries(plan: dict) -> list[dict]:
    """The plan's parameters to set back after the mission: every
    `firmware` entry, then every boot entry marked `restore_after`."""
    return ([it for it in plan.get("firmware") or []]
            + [it for it in plan["boot"] if it.get("restore_after")])


def due(schedule: list[dict], done: set[int], t0: float | None, now: float) -> list[int]:
    """Indices of the scheduled changes whose time has come and that have not
    been applied: none before t0 is known."""
    if t0 is None:
        return []
    return [i for i, it in enumerate(schedule)
            if i not in done and now >= t0 + float(it["t"])]


def write_record(path: Path, rec: dict) -> None:
    """The record, replaced whole (a temporary file renamed over it), so a
    reader never sees half of it."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(rec, indent=1, default=str) + "\n", encoding="utf-8",
                   newline="\n")
    os.replace(tmp, path)


# --------------------------------------------------------------------------- #
# The link to SITL (pymavlink, imported here only)
# --------------------------------------------------------------------------- #

def _pid(raw) -> str:
    if isinstance(raw, (bytes, bytearray)):
        raw = bytes(raw).decode("ascii", "replace")
    return str(raw).rstrip("\x00")


class SitlLink:
    """One pymavlink connection to SITL, pumped by one loop: every message
    read goes through pump(), so a marker is never lost while a parameter's
    echo is awaited."""

    def __init__(self, link: str, connect_timeout_s: float = 60.0, clock=time.monotonic):
        # MAVLink 2, as ArduPilot's SERIAL0 speaks it and as the node's
        # STATUSTEXT arrives (sitl/gcs_backstop.py does the same); set before
        # pymavlink is first imported, and only in this process.
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.clock = clock
        end = clock() + connect_timeout_s
        while True:
            try:
                self.m = mavutil.mavlink_connection(link, source_system=SCHED_SYSID,
                                                    source_component=SCHED_COMPID,
                                                    dialect="ardupilotmega")
                break
            except Exception as e:                        # noqa: BLE001
                if clock() >= end:
                    raise ConnectionError(f"{link}: {type(e).__name__}: {e}") from e
                time.sleep(1.0)
        self.link = link
        self._last_hb = -1e9
        self.markers: list[dict] = []
        self.params: dict[str, tuple[float, float]] = {}   # name -> (value, receipt)
        self.autopilot: dict | None = None

    def heartbeat(self, force: bool = False) -> None:
        now = self.clock()
        if force or now - self._last_hb >= 1.0:
            ml = self.mavutil.mavlink
            self.m.mav.heartbeat_send(ml.MAV_TYPE_GCS, ml.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_hb = now

    def pump(self, timeout: float) -> None:
        """Read what arrives within `timeout` (at least one try), sending the
        heartbeat as it falls due."""
        end = self.clock() + max(0.0, timeout)
        while True:
            self.heartbeat()
            msg = self.m.recv_match(blocking=True,
                                    timeout=max(0.005, min(0.05, end - self.clock())))
            if msg is not None:
                self._dispatch(msg)
            if self.clock() >= end:
                return

    def _dispatch(self, msg) -> None:
        kind = msg.get_type()
        rx = self.clock()
        if kind == "STATUSTEXT":
            got = parse_marker(getattr(msg, "text", ""))
            if got is not None:
                self.markers.append({"rx": rx, "nonce8": got[0], "t": got[1],
                                     "src": [msg.get_srcSystem(), msg.get_srcComponent()],
                                     "wall": time.time()})
        elif kind == "PARAM_VALUE":
            if msg.get_srcSystem() == AP_SYSID and msg.get_srcComponent() == AP_COMPID:
                self.params[_pid(msg.param_id)] = (float(msg.param_value), rx)
        elif kind == "HEARTBEAT" and self.autopilot is None:
            ml = self.mavutil.mavlink
            if (msg.get_srcSystem() == AP_SYSID and msg.get_srcComponent() == AP_COMPID
                    and msg.autopilot == ml.MAV_AUTOPILOT_ARDUPILOTMEGA):
                self.autopilot = {"sysid": AP_SYSID, "compid": AP_COMPID,
                                  "mav_type": int(msg.type),
                                  "mavlink_version": int(msg.mavlink_version)}

    def wait_autopilot(self, timeout: float) -> dict:
        end = self.clock() + timeout
        while self.autopilot is None and self.clock() < end:
            self.pump(0.5)
        if self.autopilot is None:
            raise TimeoutError(f"no ArduPilot heartbeat on {self.link} within {timeout:g} s")
        return self.autopilot

    def _await_param(self, name: str, since: float, timeout: float) -> tuple[float, float] | None:
        end = self.clock() + timeout
        while self.clock() < end:
            got = self.params.get(name)
            if got is not None and got[1] >= since:
                return got
            self.pump(min(0.05, max(0.0, end - self.clock())))
        got = self.params.get(name)
        return got if got is not None and got[1] >= since else None

    def set_param(self, name: str, value: float, timeout: float = ECHO_TIMEOUT_S,
                  tries: int = SET_TRIES) -> tuple[float | None, float | None, float | None]:
        """PARAM_SET until its PARAM_VALUE echo carries the value: (echo, the
        send time of the try that took, the echo's receipt), Nones when no
        try did."""
        ml = self.mavutil.mavlink
        for _ in range(tries):
            sent = self.clock()
            self.m.mav.param_set_send(AP_SYSID, AP_COMPID, name.encode(), float(value),
                                      ml.MAV_PARAM_TYPE_REAL32)
            got = self._await_param(name, sent, timeout)
            if got is not None and read_back_ok(value, got[0]):
                return got[0], sent, got[1]
        return None, None, None

    def read_param(self, name: str, timeout: float = 2.0, tries: int = 3) -> float | None:
        for _ in range(tries):
            sent = self.clock()
            self.m.mav.param_request_read_send(AP_SYSID, AP_COMPID, name.encode(), -1)
            got = self._await_param(name, sent, timeout)
            if got is not None:
                return got[0]
        return None


# --------------------------------------------------------------------------- #
# The run
# --------------------------------------------------------------------------- #

class _Stop(Exception):
    pass


def _boot_failed(rec: dict, record_path: Path, why: str, say) -> int:
    rec["state"] = "failed"
    rec["error"] = why
    write_record(record_path, rec)
    say(f"SITL_STRESS BOOT FAILED {why}")
    return 1


def restore(link, record_path: Path, say=print) -> list[dict]:
    """Set back every parameter the record says is to be restored (`restore`:
    the value each held before boot), once, and record each as stressor_entry
    plus `restored`. Returns the entries; [] when there is nothing (left) to
    restore. Never raises: it runs on the way out."""
    try:
        rec = json.loads(Path(record_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    res = rec.get("restore") if isinstance(rec.get("restore"), dict) else None
    if not res or res.get("done"):
        return []
    out = []
    for p in res.get("params") or []:
        want = p.get("restore_to")
        if want is None:
            continue
        try:
            echo, _, _ = link.set_param(p["param"], float(want),
                                        timeout=RESTORE_ECHO_TIMEOUT_S, tries=RESTORE_TRIES)
        except Exception as e:                             # noqa: BLE001
            echo = None
            p["error"] = f"{type(e).__name__}: {e}"
        p.update(stressor_entry(p["param"], float(want), echo), restored=echo is not None)
        out.append(p)
    res["done"] = True
    res["wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    write_record(record_path, rec)
    bad = [p["param"] for p in out if not p["restored"]]
    say("SITL_STRESS RESTORED " + " ".join(f"{p['param']}={p['set']:g}" for p in out)
        + (f" (NOT restored: {', '.join(bad)})" if bad else ""))
    return out


def run(plan: dict, record_path: Path, link: SitlLink, *, t0_timeout_s: float,
        say=print) -> int:
    nonce8 = plan["run_nonce"][:8]
    rec: dict = {"schema": RECORD_SCHEMA, "tool": "sitl/sitl_stressors.py",
                 "run_id": plan.get("run_id"), "run_nonce": plan["run_nonce"],
                 "link": link.link, "pid": os.getpid(),
                 "sched": {"sysid": SCHED_SYSID, "compid": SCHED_COMPID},
                 "started_wall": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                 "state": "boot", "error": None, "autopilot": None,
                 "boot": {"ok": False, "params": []},
                 "markers": [], "t0": None,
                 # Each entry: the plan's (t, param, value, field) and, once
                 # due, stressor_entry's set / read_back (the autopilot's
                 # PARAM_VALUE echo) / applied, with the times from t0.
                 "schedule": [{**it, "applied": False} for it in plan["schedule"]]}
    write_record(record_path, rec)
    rec["autopilot"] = link.wait_autopilot(60.0)
    # THE BATTERY CELL'S FAIL-SAFE SETUP (module docstring): guarded, and
    # every value to restore read before anything is set.
    firmware = plan.get("firmware") or []
    to_restore = restore_entries(plan)
    if to_restore:
        rec["restore"] = {"done": False, "params": []}
    if firmware:
        rec["firmware"] = {"ok": False, "params": [], "guard": None}
        if not firmware_link_ok(link.link):
            refused = (f"the link {link.link!r} is not a loopback tcp/udp link to SITL: "
                       f"no firmware parameter is set over it")
            probe = None
        else:
            probe = link.read_param(SITL_PROBE_PARAM)
            refused = (None if probe is not None else
                       f"the autopilot does not answer {SITL_PROBE_PARAM}: not SITL, so no "
                       f"firmware parameter is set")
        rec["firmware"]["guard"] = {"link": link.link, "probe": SITL_PROBE_PARAM,
                                    "probe_value": probe, "refused": refused}
        if refused:
            return _boot_failed(rec, record_path, refused, say)
    for it in to_restore:
        before = link.read_param(it["param"])
        rec["restore"]["params"].append({"param": it["param"], "restore_to": before,
                                         "restored": False})
    unread = [p["param"] for p in (rec.get("restore") or {}).get("params", [])
              if p["restore_to"] is None]
    if unread:
        return _boot_failed(rec, record_path, "the value(s) to restore after the mission "
                            "could not be read, so none is changed: " + ", ".join(unread), say)
    # On disk before the first set: restore() reads the record, so a SIGTERM
    # in the middle of the sets below still finds every value to set back.
    if to_restore:
        write_record(record_path, rec)
    for it in firmware:
        name, want = it["param"], float(it["value"])
        echo, _, _ = link.set_param(name, want, timeout=2.0)
        back = link.read_param(name)
        rec["firmware"]["params"].append(stressor_entry(name, want, back, echo=echo,
                                                        why=it.get("why")))
    if firmware:
        rec["firmware"]["ok"] = all(p["applied"] for p in rec["firmware"]["params"])
    # BOOT: every boot parameter set, echoed and read back (stressor_entry,
    # the pymavlink rail's record of a stressor).
    for it in plan["boot"]:
        name, want = it["param"], float(it["value"])
        echo, _, _ = link.set_param(name, want, timeout=2.0)
        back = link.read_param(name)
        rec["boot"]["params"].append(stressor_entry(name, want, back, echo=echo,
                                                    why=it.get("why")))
    ok = (all(p["applied"] for p in rec["boot"]["params"])
          and (rec.get("firmware") or {}).get("ok", True))
    rec["boot"]["ok"] = ok
    rec["boot"]["wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    if not ok:
        bad = [p for p in ((rec.get("firmware") or {}).get("params") or [])
               + rec["boot"]["params"] if not p["applied"]]
        return _boot_failed(rec, record_path, "boot parameter(s) did not read back: "
                            + ", ".join(f"{p['param']} set {p['set']:g} read {p['read_back']}"
                                        for p in bad), say)
    rec["state"] = "waiting_t0" if plan["schedule"] else "done"
    write_record(record_path, rec)
    say("SITL_STRESS BOOT OK " + " ".join(
        f"{p['param']}={p['read_back']:g}"
        for p in ((rec.get("firmware") or {}).get("params") or []) + rec["boot"]["params"]))
    if not plan["schedule"]:
        say("SITL_STRESS DONE (no scheduled change)")
        return 0
    # T0 and the schedule.
    sched = plan["schedule"]
    done: set[int] = set()
    seen = 0
    wait_end = link.clock() + t0_timeout_s
    t0 = None
    while len(done) < len(sched):
        mine = [m for m in link.markers if m["nonce8"] == nonce8]
        if len(mine) != seen:
            seen = len(mine)
            t0 = t0_estimate([(m["rx"], m["t"]) for m in mine])
            rec["markers"] = [{"t": m["t"], "rx_rel_t0": round(m["rx"] - t0, 4),
                               "src": m["src"], "wall": m["wall"]} for m in mine]
            rec["t0"] = {"markers": seen, "wall": time.time() - (link.clock() - t0),
                         "method": "min(receipt - t) over the node's markers"}
            if rec["state"] == "waiting_t0":
                rec["state"] = "running"
                say(f"SITL_STRESS T0 from {seen} marker(s)")
            write_record(record_path, rec)
        if t0 is None and link.clock() > wait_end:
            rec["state"] = "failed"
            rec["error"] = f"no t=0 marker with nonce {nonce8} within {t0_timeout_s:g} s"
            write_record(record_path, rec)
            say(f"SITL_STRESS FAILED {rec['error']}")
            return 1
        for i in due(sched, done, t0, link.clock()):
            it = sched[i]
            echo, sent, rx = link.set_param(it["param"], float(it["value"]))
            entry = rec["schedule"][i]
            entry.update(stressor_entry(it["param"], float(it["value"]), echo),
                         t_sent=None if sent is None else round(sent - t0, 4),
                         t_echo=None if rx is None else round(rx - t0, 4),
                         late_s=None if sent is None else round(sent - t0 - float(it["t"]), 4))
            if echo is None:
                entry["error"] = f"no PARAM_VALUE echo of {it['value']} after {SET_TRIES} tries"
            done.add(i)
            write_record(record_path, rec)
            say(f"SITL_STRESS t={float(it['t']):g} {it['param']}={it['value']} "
                + ("applied" if echo is not None else "NOT APPLIED"))
        nxt = [t0 + float(sched[i]["t"]) for i in range(len(sched)) if i not in done] \
            if t0 is not None else []
        link.pump(0.02 if not nxt else max(0.0, min(0.02, min(nxt) - link.clock())))
    rec["state"] = "done"
    rec["ended_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    write_record(record_path, rec)
    failed = [e for e in rec["schedule"] if not e.get("applied")]
    say("SITL_STRESS DONE" + (f" ({len(failed)} change(s) NOT applied)" if failed else ""))
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sitl_stressors.py",
                                 description=__doc__.split("\n\n")[1].strip())
    ap.add_argument("--plan", required=True, help=f"the mission's plan ({PLAN_SCHEMA})")
    ap.add_argument("--record", required=True, help=f"where the record goes ({RECORD_SCHEMA})")
    ap.add_argument("--link", default=DEFAULT_LINK,
                    help=f"pymavlink connection to SITL (default {DEFAULT_LINK}, SERIAL0)")
    ap.add_argument("--t0-timeout-s", type=float, default=1800.0,
                    help="how long to wait for the node's t=0 marker")
    args = ap.parse_args(argv)
    record = Path(args.record)
    try:
        plan = load_plan(args.plan)
    except (OSError, ValueError) as e:
        print(f"SITL_STRESS BOOT FAILED the plan: {e}", flush=True)
        return 1

    def stop(signum, frame):                               # noqa: ARG001
        raise _Stop()
    signal.signal(signal.SIGTERM, stop)

    def say(line: str) -> None:
        print(line, flush=True)

    def restore_once() -> None:
        # Never interrupted by a second TERM while it sets parameters back.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if link is not None:
            restore(link, record, say)

    link = None
    try:
        link = SitlLink(args.link)
        rc = run(plan, record, link, t0_timeout_s=args.t0_timeout_s, say=say)
        if restore_entries(plan):
            if rc == 0:
                # Parameters to set back once the mission is over: hold, with
                # the heartbeat, until `stop` (SIGTERM), then restore.
                say("SITL_STRESS HOLDING until stop (parameters to restore after the "
                    "mission)")
                while True:
                    try:
                        link.pump(0.5)
                    except _Stop:
                        raise
                    except Exception:                      # noqa: BLE001
                        break          # the link is gone: the restore below says so
            restore_once()
        return rc
    except _Stop:
        restore_once()
        try:
            rec = json.loads(record.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            rec = {"schema": RECORD_SCHEMA, "run_nonce": plan["run_nonce"]}
        if rec.get("state") not in ("done", "failed"):
            rec["state"] = "stopped"
            rec["stopped_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            write_record(record, rec)
        say("SITL_STRESS stopped")
        return 0
    except Exception as e:                                 # noqa: BLE001
        try:
            rec = json.loads(record.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            rec = {"schema": RECORD_SCHEMA, "run_nonce": plan["run_nonce"],
                   "boot": {"ok": False, "params": []}, "schedule": []}
        rec["state"] = "failed"
        rec["error"] = f"{type(e).__name__}: {e}"
        write_record(record, rec)
        say(f"SITL_STRESS BOOT FAILED {rec['error']}" if not (rec.get("boot") or {}).get("ok")
            else f"SITL_STRESS FAILED {rec['error']}")
        try:
            restore_once()
        except Exception:                                  # noqa: BLE001
            pass
        return 1


if __name__ == "__main__":
    sys.exit(main())
