"""
The ground station's step: set the ArduPilot GeoFence backstop over the GCS
link and read it back, before the Shield process exists.

    ~/venv-ros/bin/python sitl/gcs_backstop.py --bundle bundles/fase3-sim-demo-v0.1.4.tar.gz \\
        --link udpin:127.0.0.1:14560 --out demo/out/<tag>

WHAT THE GRANT ASKS FOR (WP3-10)

Safety Shield page, "MAVLink adapter": "GCS-set AP GeoFence parameters
(FENCE_ENABLE, FENCE_TYPE, FENCE_ACTION) remain configured as a last-line
backstop. They fire only if the Shield itself crashes or fails to emit."
Until 2026-11-03 the parameters came from a SITL defaults file
(sitl/fence/guardrail_fence.parm, loaded by start_sitl.sh) and the polygons
from the Shield's own MAVLink adapter (/mavros/geofence/push): no ground
station set either (independent review, 29 Oct).

WHAT THIS IS

A process of its own, standing in for the operator's ground station (Mission
Planner) on the GCS endpoint of mavlink-router - the link the router fans out
to the GCS beside MAVROS (sitl/mavlink-router/main.conf, [UdpEndpoint
mission_planner]). It speaks MAVLink as a GCS does (system 255, component 190,
MAV_TYPE_GCS heartbeats; ArduPilot's SYSID_MYGCS is 255) and does what an
operator does in Mission Planner before a flight:

  1. waits for the autopilot's heartbeat on the link;
  2. reads the fence parameters as they stand (`before`);
  3. sets FENCE_TYPE, FENCE_ACTION, FENCE_ALT_MAX, FENCE_MARGIN and
     AVOID_ENABLE, then FENCE_ENABLE, with PARAM_SET, and reads each back
     (the PARAM_VALUE the autopilot answers, and a PARAM_REQUEST_READ after);
     the values are the operator's fence file (sitl/fence/guardrail_fence.parm,
     --parm), except FENCE_ALT_MAX, raised to the policy's ceiling + 5 m when
     the file's value sits below it (mavlink_adapter_node.fence_alt_plan), so
     the backstop is never stricter than the policy;
  4. uploads the signed bundle's keep-out zones as ArduPilot exclusion
     polygons / circles with the mission protocol (MAV_MISSION_TYPE_FENCE,
     MISSION_COUNT then MISSION_ITEM_INT per item), converted to lat/lon about
     the autopilot's own home (HOME_POSITION), and reads the list back item by
     item (MISSION_REQUEST_LIST / MISSION_REQUEST_INT) and compares it. The
     zones are the ones the Shield's adapter would upload
     (guardrail.fence_export via mavlink_adapter_node.policy_fences: a clock
     cannot turn them off and their band covers the fence column), and a
     zone over the pad is held back as the adapter holds it;
  5. writes <out>/gcs_backstop.json - the link, the autopilot, the bundle,
     every parameter before / set / read back, the items sent and read back -
     and exits: 0 when every parameter and every item read back as set, 1
     otherwise, 2 when it could not run at all. sitl/run_ros2_demo.sh
     (GCS_BACKSTOP=1) refuses the flight on anything but 0.

The Shield node reads the parameters again at take-off through MAVROS
(hil_evidence.autopilot_fence) and records this file's summary beside them,
so the run shows the same values from two processes on two links.

A DECLARED GROUND (X-11, 2026-11-18). A bundle with an MSL rule loads only
over the ground it flies over, so the GCS takes the Shield node's
--ground FILE (with --ground-sha256 HEX, the file's hash, checked before it
is read) and --ground-home-msl M (the autopilot's home elevation; required
over a DEM - the GCS never guesses it). With them FENCE_ALT_MAX's ceiling
is a height above home (mavlink_adapter_node.policy_ceiling_m: an MSL
ceiling less home, and a hard MSL ceiling in force all flight caps it) and
each zone's band is read above home before the column test
(fence_export.iter_zones). The autopilot's own HOME_POSITION altitude, and
over a DEM the DEM's elevation at home, must agree with --ground-home-msl
within terrain.GROUND_HOME_TOL_M, or the step fails (exit 1) without
uploading. Without --ground: as before.

WHAT IT IS NOT

  * Not Mission Planner. It is the scripted equivalent of an operator's
    parameter edit and fence upload, on the same link; no display is shown
    or recorded (ARCH-10 is the Mission Planner item).
  * Not a hot-apply path. A GCS-set fence is static for the flight: the
    zones a REST event adds mid-flight are the Shield's to enforce (the grant:
    "the Shield handles finer-grained constraints ... that AP's GeoFence
    cannot express"), and the adapter uploads none on such a run
    (mavlink_adapter_node --fences-by-gcs).

Runs with any Python that has pymavlink, cryptography (the bundle's
signature) and the Guardrail's dependencies: ~/venv-ros/bin/python. No ROS.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

# MAVLink 2: MISSION_COUNT / MISSION_ITEM_INT carry mission_type only there.
os.environ.setdefault("MAVLINK20", "1")

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "sitl"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from mavlink_adapter_node import (FENCE_MARGIN_M, ORIGIN_TOLERANCE_M,    # noqa: E402
                                  fence_alt_plan, fence_items, origin_offset_m,
                                  policy_ceiling_m, policy_fences, read_parm)

GCS_SYSID = 255          # ArduPilot SYSID_MYGCS default: the operator's GCS
GCS_COMPID = 190         # MAV_COMP_ID_MISSIONPLANNER
AP_SYSID, AP_COMPID = 1, 1
MAV_MISSION_TYPE_FENCE = 1
MAV_MISSION_ACCEPTED = 0
DEFAULT_LINK = "udpin:127.0.0.1:14560"
DEFAULT_PARM = ROOT / "sitl" / "fence" / "guardrail_fence.parm"
# Set in this order: the fence is switched on last, once the rest is in place.
PARAM_ORDER = ("FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX", "FENCE_MARGIN",
               "AVOID_ENABLE", "FENCE_ENABLE")
# The grant's three, which the record names as such.
GRANT_PARAMS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION")
PARAM_TOL = 1e-3


def desired_params(parm: dict[str, float], policy, cruise_alt_m: float | None,
                   ground_kw: dict | None = None) -> tuple[dict[str, float], dict]:
    """The values the GCS sets: the operator's fence file, with FENCE_ALT_MAX
    raised above the policy's ceiling when the file's value is below it.
    Returns `(values, alt_plan)`. A parameter the file does not give is
    refused: the GCS sets what the operator wrote, never a guess.
    `ground_kw` ({"ground", "home_msl_m"}, X-11): the ceiling above home over
    a declared ground (policy_ceiling_m); None or {}: as before."""
    missing = [k for k in PARAM_ORDER if k not in parm]
    if missing:
        raise ValueError(f"the fence file gives no {', '.join(missing)}")
    vals = {k: float(parm[k]) for k in PARAM_ORDER}
    ceiling, src = policy_ceiling_m(policy, cruise_alt_m, **(ground_kw or {}))
    plan = fence_alt_plan(ceiling, vals["FENCE_ALT_MAX"])
    if plan["raise_to"] is not None:
        vals["FENCE_ALT_MAX"] = float(plan["raise_to"])
    return vals, {**plan, "policy_ceiling_m": ceiling, "policy_ceiling_source": src,
                  "file_value": float(parm["FENCE_ALT_MAX"])}


def items_match(sent: list[dict], got: list[dict]) -> list[str]:
    """Why the read-back fence list differs from the one sent ([] = same):
    count, command, vertex count / radius, and lat/lon to the 1e-7 degree the
    protocol carries."""
    probs = []
    if len(sent) != len(got):
        probs.append(f"{len(got)} item(s) read back, {len(sent)} sent")
    for i, (a, b) in enumerate(zip(sent, got)):
        if int(a["command"]) != int(b["command"]):
            probs.append(f"item {i}: command {b['command']} != {a['command']}")
        if abs(float(a["param1"]) - float(b["param1"])) > 1e-3:
            probs.append(f"item {i}: param1 {b['param1']} != {a['param1']}")
        for k in ("x", "y"):
            if abs(int(a[k]) - int(b[k])) > 1:
                probs.append(f"item {i}: {k} {b[k]} != {a[k]}")
    return probs


def int_items(items: list[dict]) -> list[dict]:
    """mavlink_adapter_node.fence_items entries as MISSION_ITEM_INT fields."""
    return [{"seq": i, "frame": int(it["frame"]), "command": int(it["command"]),
             "param1": float(it["param1"]), "x": int(round(it["x_lat"] * 1e7)),
             "y": int(round(it["y_long"] * 1e7)), "z": float(it["z_alt"]),
             "id": it["id"]} for i, it in enumerate(items)]


class Gcs:
    """A minimal ground station on one pymavlink connection."""

    def __init__(self, link: str, log=print) -> None:
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(link, source_system=GCS_SYSID,
                                            source_component=GCS_COMPID,
                                            dialect="ardupilotmega")
        self.log = log
        self._last_hb = 0.0
        self.autopilot: dict | None = None
        # The request messages the autopilot used in the upload (a
        # MISSION_REQUEST_INT means it asked for MISSION_ITEM_INT).
        self._kinds: set[str] = set()

    def heartbeat(self, force: bool = False) -> None:
        now = time.time()
        if force or now - self._last_hb >= 1.0:
            ml = self.mavutil.mavlink
            self.m.mav.heartbeat_send(ml.MAV_TYPE_GCS, ml.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_hb = now

    def recv(self, types, timeout: float):
        """The next message of `types` from the autopilot (system 1,
        component 1), heartbeats sent meanwhile; None on the timeout."""
        end = time.time() + timeout
        while time.time() < end:
            self.heartbeat()
            msg = self.m.recv_match(type=types, blocking=True,
                                    timeout=min(0.25, max(0.01, end - time.time())))
            if msg is None:
                continue
            if msg.get_srcSystem() != AP_SYSID or msg.get_srcComponent() != AP_COMPID:
                continue
            return msg
        return None

    def wait_autopilot(self, timeout: float) -> dict:
        ml = self.mavutil.mavlink
        end = time.time() + timeout
        while time.time() < end:
            hb = self.recv("HEARTBEAT", min(2.0, end - time.time()))
            if hb is not None and hb.autopilot == ml.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.autopilot = {"sysid": AP_SYSID, "compid": AP_COMPID,
                                  "mav_type": int(hb.type),
                                  "autopilot": int(hb.autopilot),
                                  "mavlink_version": int(hb.mavlink_version),
                                  "system_status": int(hb.system_status)}
                return self.autopilot
        raise TimeoutError(f"no ArduPilot heartbeat on the link within {timeout:g} s")

    def read_param(self, name: str, timeout: float = 3.0, tries: int = 3) -> float | None:
        for _ in range(tries):
            self.m.mav.param_request_read_send(AP_SYSID, AP_COMPID, name.encode(), -1)
            end = time.time() + timeout
            while time.time() < end:
                pv = self.recv("PARAM_VALUE", end - time.time())
                if pv is not None and _pid(pv.param_id) == name:
                    return float(pv.param_value)
        return None

    def set_param(self, name: str, value: float, timeout: float = 3.0,
                  tries: int = 3) -> float | None:
        """PARAM_SET, and the PARAM_VALUE the autopilot answers with."""
        ml = self.mavutil.mavlink
        for _ in range(tries):
            self.m.mav.param_set_send(AP_SYSID, AP_COMPID, name.encode(), float(value),
                                      ml.MAV_PARAM_TYPE_REAL32)
            end = time.time() + timeout
            while time.time() < end:
                pv = self.recv("PARAM_VALUE", end - time.time())
                if pv is not None and _pid(pv.param_id) == name:
                    if abs(float(pv.param_value) - float(value)) <= PARAM_TOL:
                        return float(pv.param_value)
                    break
        return None

    def home(self, timeout: float) -> dict:
        """The autopilot's HOME_POSITION, asked for until it has one."""
        ml = self.mavutil.mavlink
        end = time.time() + timeout
        while time.time() < end:
            self.m.mav.command_long_send(AP_SYSID, AP_COMPID,
                                         ml.MAV_CMD_REQUEST_MESSAGE, 0,
                                         ml.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
            hp = self.recv("HOME_POSITION", min(2.0, end - time.time()))
            if hp is not None and (hp.latitude or hp.longitude):
                return {"lat": hp.latitude / 1e7, "lon": hp.longitude / 1e7,
                        "alt_m": hp.altitude / 1e3,
                        # local NED of home about the EKF origin (x north, y east)
                        "local_ne": [float(hp.x), float(hp.y)]}
        raise TimeoutError(f"the autopilot had no home position within {timeout:g} s "
                           f"(no GPS fix yet?)")

    def upload_fence(self, items: list[dict], timeout: float = 15.0) -> dict:
        """MISSION_COUNT (fence) then MISSION_ITEM_INT for each request, until
        the autopilot's MISSION_ACK."""
        ml = self.mavutil.mavlink
        self.m.mav.mission_count_send(AP_SYSID, AP_COMPID, len(items),
                                      mission_type=MAV_MISSION_TYPE_FENCE)
        sent, end = set(), time.time() + timeout
        while time.time() < end:
            msg = self.recv(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                            end - time.time())
            if msg is None:
                break
            if getattr(msg, "mission_type", MAV_MISSION_TYPE_FENCE) != MAV_MISSION_TYPE_FENCE:
                continue
            if msg.get_type() == "MISSION_ACK":
                return {"ack": int(msg.type), "accepted": msg.type == MAV_MISSION_ACCEPTED,
                        "items_requested": len(sent), "request_kind": sorted(self._kinds)}
            self._kinds.add(msg.get_type())
            if not 0 <= int(msg.seq) < len(items):
                continue
            it = items[msg.seq]
            self.m.mav.mission_item_int_send(
                AP_SYSID, AP_COMPID, it["seq"], it["frame"], it["command"], 0, 1,
                it["param1"], 0, 0, 0, it["x"], it["y"], it["z"],
                mission_type=MAV_MISSION_TYPE_FENCE)
            sent.add(msg.seq)
        return {"ack": None, "accepted": False, "items_requested": len(sent),
                "why": f"no MISSION_ACK within {timeout:g} s",
                "request_kind": sorted(self._kinds)}

    def read_fence(self, timeout: float = 15.0) -> list[dict] | None:
        """The fence list as the autopilot holds it, item by item."""
        ml = self.mavutil.mavlink
        self.m.mav.mission_request_list_send(AP_SYSID, AP_COMPID,
                                             mission_type=MAV_MISSION_TYPE_FENCE)
        cnt = self.recv("MISSION_COUNT", 5.0)
        if cnt is None:
            return None
        got = []
        for seq in range(int(cnt.count)):
            item = None
            for _ in range(3):
                self.m.mav.mission_request_int_send(AP_SYSID, AP_COMPID, seq,
                                                    mission_type=MAV_MISSION_TYPE_FENCE)
                item = self.recv("MISSION_ITEM_INT", 3.0)
                if item is not None and item.seq == seq:
                    break
            if item is None:
                return None
            got.append({"seq": int(item.seq), "frame": int(item.frame),
                        "command": int(item.command), "param1": float(item.param1),
                        "x": int(item.x), "y": int(item.y), "z": float(item.z)})
        self.m.mav.mission_ack_send(AP_SYSID, AP_COMPID, MAV_MISSION_ACCEPTED,
                                    mission_type=MAV_MISSION_TYPE_FENCE)
        return got


def _pid(raw) -> str:
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", "replace")
    return str(raw).rstrip("\x00")


def gcs_ground(args) -> tuple[object, dict | None, dict]:
    """(ground, its record, the policy_ceiling_m / policy_fences keywords)
    from --ground / --ground-sha256 / --ground-home-msl (module docstring, A
    DECLARED GROUND); (None, None, {}) without --ground. Over a DEM the home
    elevation must be given: the GCS does not take one from the DEM."""
    from guardrail.terrain import ground_from_args, is_dem
    hmsl = getattr(args, "ground_home_msl", None)
    if hmsl is not None and not getattr(args, "ground", None):
        raise ValueError("--ground-home-msl is the home elevation of a --ground")
    ground, gfile = ground_from_args(args, args.bundle, None, root=ROOT)
    if ground is None:
        return None, None, {}
    if is_dem(ground):
        if hmsl is None:
            raise ValueError("over a DEM --ground the GCS needs --ground-home-msl (the "
                             "autopilot's home elevation, m MSL): it never takes one "
                             "from the DEM")
        home = float(hmsl)
    else:
        home = float(ground.elevation_at())
        if hmsl is not None and abs(float(hmsl) - home) > 1e-6:
            raise ValueError(f"--ground-home-msl {float(hmsl):g} and the flat ground "
                             f"{ground!r} differ")
    if not math.isfinite(home):
        raise ValueError(f"home elevation {home!r} is not finite")
    rec = {"file": gfile, "ground": ground.record(), "home_msl_m": home}
    return ground, rec, {"ground": ground, "home_msl_m": home}


def home_ground_problem(ground, home_msl_m: float, home: dict,
                        home_ne: tuple[float, float] | None) -> str | None:
    """Why the autopilot's home does not stand where --ground-home-msl says
    (None when it does): its HOME_POSITION altitude, and over a DEM the
    DEM's elevation at home's policy-frame position, must each be within
    terrain.GROUND_HOME_TOL_M of it."""
    from guardrail.terrain import GROUND_HOME_TOL_M, is_dem
    alt = home.get("alt_m")
    if alt is not None and abs(float(alt) - home_msl_m) > GROUND_HOME_TOL_M:
        return (f"the autopilot's home altitude {float(alt):g} m MSL is "
                f"{abs(float(alt) - home_msl_m):.2f} m from --ground-home-msl "
                f"{home_msl_m:g} (over {GROUND_HOME_TOL_M:g} m)")
    if is_dem(ground):
        ne = home_ne if home_ne is not None else (0.0, 0.0)
        g = float(ground.elevation_at(ne[0], ne[1]))
        if abs(g - home_msl_m) > GROUND_HOME_TOL_M:
            return (f"the DEM's elevation at home ({ne[0]:.1f}, {ne[1]:.1f}) is "
                    f"{g:.2f} m MSL, {abs(g - home_msl_m):.2f} m from "
                    f"--ground-home-msl {home_msl_m:g} (over {GROUND_HOME_TOL_M:g} m)")
    return None


def run(args) -> tuple[int, dict]:
    from guardrail.bundle import load_for_flight
    from guardrail.compiler import ConstraintCompiler

    rec: dict = {"role": "gcs", "tool": "sitl/gcs_backstop.py", "pid": os.getpid(),
                 "link": args.link, "gcs_sysid": GCS_SYSID, "gcs_compid": GCS_COMPID,
                 "router_endpoint": args.router_endpoint,
                 "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "ok": False}
    ground, ground_rec, gkw = gcs_ground(args)
    policy, source = load_for_flight(args.bundle, None,
                                     **({} if ground is None else {"ground": ground}))
    rec["bundle"] = {"path": str(args.bundle), "policy_id": policy.policy_id,
                     "version": policy.version, "policy_hash": policy.policy_hash,
                     "signature": source.get("signature")}
    if ground_rec is not None:
        rec["ground"] = ground_rec
    parm_path = Path(args.parm)
    parm = read_parm(parm_path)
    comp = (ConstraintCompiler(policy) if ground is None
            else ConstraintCompiler(policy, ground=ground))
    cruise = comp.parse_command("fly to the northeast pad at 6 m/s").cruise_alt_m
    want, alt_plan = desired_params(parm, policy, cruise, gkw)
    rec["fence_file"] = {"path": str(parm_path.relative_to(ROOT) if parm_path.is_relative_to(ROOT)
                                     else parm_path),
                         "values": {k: parm.get(k) for k in PARAM_ORDER}}
    rec["fence_alt_plan"] = alt_plan

    gcs = Gcs(args.link)
    rec["autopilot"] = gcs.wait_autopilot(args.timeout)
    print(f"[gcs] autopilot heartbeat on {args.link}: {rec['autopilot']}", flush=True)

    params: dict = {}
    for name in PARAM_ORDER:
        params[name] = {"before": gcs.read_param(name)}
    for name in PARAM_ORDER:
        echo = gcs.set_param(name, want[name])
        back = gcs.read_param(name)
        ok = back is not None and abs(back - want[name]) <= PARAM_TOL
        params[name].update({"set": want[name], "echo": echo, "read_back": back,
                             "ok": ok, "grant_named": name in GRANT_PARAMS})
        print(f"[gcs] {name}: {params[name]['before']} -> set {want[name]:g}, "
              f"read back {back}", flush=True)
    rec["params"] = params

    home = gcs.home(args.home_timeout)
    rec["home"] = home
    org = getattr(policy, "frame_origin", None)
    off = origin_offset_m(org, home["lat"], home["lon"]) if org else None
    fence: dict = {"origin_offset_m": None if off is None else [round(v, 2) for v in off]}
    if off is not None and math.hypot(*off) > ORIGIN_TOLERANCE_M:
        fence["why"] = (f"the policy's frame origin is {math.hypot(*off):.1f} m from the "
                        f"autopilot's home: the zones would be uploaded in the wrong place")
        rec["fence"] = fence
        return 1, rec
    if ground is not None:
        why = home_ground_problem(ground, gkw["home_msl_m"], home, off)
        rec["ground"]["home_check"] = why or "ok"
        if why:
            fence["why"] = f"the declared ground does not stand under home: {why}"
            rec["fence"] = fence
            return 1, rec
    margin = params["FENCE_MARGIN"]["read_back"] or FENCE_MARGIN_M
    zones, skipped = policy_fences(policy, column_m=params["FENCE_ALT_MAX"]["read_back"],
                                   position=tuple(home["local_ne"]), margin_m=margin,
                                   **gkw)
    items = int_items(fence_items(zones, home["lat"], home["lon"],
                                  tuple(home["local_ne"])))
    fence.update({"zones": [z["id"] for z in zones], "skipped": skipped,
                  "items_sent": len(items),
                  "items": [{k: v for k, v in it.items()} for it in items]})
    up = gcs.upload_fence(items)
    fence["upload"] = up
    got = gcs.read_fence() if up.get("accepted") else None
    fence["items_read_back"] = None if got is None else len(got)
    fence["read_back"] = got
    probs = ["the read-back failed"] if got is None else items_match(items, got)
    fence["match"] = not probs
    fence["problems"] = probs
    rec["fence"] = fence
    print(f"[gcs] fence: {len(items)} item(s) of {fence['zones']} sent, "
          f"{fence['items_read_back']} read back, match {fence['match']}"
          + (f" ({'; '.join(probs)})" if probs else ""), flush=True)
    rec["ok"] = bool(all(p["ok"] for p in params.values()) and up.get("accepted")
                     and fence["match"])
    rec["t_end"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    return (0 if rec["ok"] else 1), rec


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="gcs_backstop.py",
                                 description=__doc__.split("\n\n")[1].strip())
    ap.add_argument("--bundle", required=True, help="the signed policy bundle")
    ap.add_argument("--link", default=DEFAULT_LINK,
                    help=f"pymavlink connection on the GCS endpoint (default {DEFAULT_LINK})")
    ap.add_argument("--router-endpoint", default=None,
                    help="how the link reaches the autopilot, for the record")
    ap.add_argument("--parm", default=str(DEFAULT_PARM),
                    help="the operator's fence file (NAME VALUE lines)")
    ap.add_argument("--out", required=True, help="the run folder: gcs_backstop.json")
    ap.add_argument("--timeout", type=float, default=60.0,
                    help="seconds to wait for the autopilot's heartbeat")
    ap.add_argument("--home-timeout", type=float, default=120.0,
                    help="seconds to wait for the autopilot's home position")
    ap.add_argument("--ground", default=None, metavar="FILE",
                    help="the Shield node's declared ground (guardrail/terrain.py "
                         "load_ground, repo-relative; X-11): a bundle with an MSL rule "
                         "loads over it, and FENCE_ALT_MAX and the zones' column test "
                         "are read above home over it. Default: none, as before")
    ap.add_argument("--ground-sha256", default=None, metavar="HEX",
                    help="the sha256 of the --ground file; it must hash to it")
    ap.add_argument("--ground-home-msl", type=float, default=None, metavar="M",
                    help="the autopilot's home elevation, m MSL (required over a DEM "
                         "--ground; checked against HOME_POSITION and the DEM)")
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    try:
        code, rec = run(args)
    except Exception as e:                                   # noqa: BLE001
        code, rec = 2, {"role": "gcs", "ok": False, "error": f"{type(e).__name__}: {e}",
                        "link": args.link, "bundle": {"path": str(args.bundle)}}
    with open(out / "gcs_backstop.json", "w", encoding="utf-8", newline="\n") as fh:
        json.dump(rec, fh, indent=1, default=str)
        fh.write("\n")
    print(f"[gcs] {'OK' if code == 0 else 'FAILED'}: {out / 'gcs_backstop.json'}"
          + (f" ({rec.get('error')})" if rec.get("error") else ""), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
