"""Bench test of the real flight controller (CUAV V5 Nano, USB COM8), 17 Nov 2026.

Never arms. Steps:
  1. back up every parameter and the stored fence;
  2. set the EKF origin and home to the policy bundle's frame origin (the FC has no GPS fix);
  3. run sitl/gcs_backstop.py's own procedure on the FC (fence parameters set + read back,
     the bundle's keep-out zones uploaded as exclusion polygons + read back);
  4. request each FSM mode by MAVLink (GUIDED, BRAKE, LOITER, RTL, LAND, STABILIZE) and record
     the autopilot's answer;
  5. restore every parameter that changed, clear the uploaded fence, reboot the FC (clears the
     origin/home set in step 2), and diff the parameters against the backup.
Writes everything to OUT.
"""
import json, os, sys, time
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("MAVLINK20", "1")
ROOT = Path(sys.argv[1]).resolve()
OUT = Path(sys.argv[2]); OUT.mkdir(parents=True, exist_ok=True)
LINK = sys.argv[3] if len(sys.argv) > 3 else "COM8"
BUNDLE = sys.argv[4] if len(sys.argv) > 4 else str(ROOT / "bundles/fase3-sim-demo-v0.1.4.tar.gz")
for p in (ROOT, ROOT / "sitl"):
    sys.path.insert(0, str(p))
from pymavlink import mavutil  # noqa: E402

log_lines = []
def log(*a):
    s = " ".join(str(x) for x in a); print(s, flush=True); log_lines.append(s)

def connect():
    m = mavutil.mavlink_connection(LINK, baud=115200, source_system=255, source_component=191,
                                   dialect="ardupilotmega")
    hb = m.wait_heartbeat(timeout=20)
    if hb is None:
        raise SystemExit("no heartbeat")
    return m, hb

def all_params(m, timeout=60):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    got, total, t0 = {}, None, time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            if total and len(got) >= total:
                break
            continue
        got[msg.param_id] = msg.param_value
        total = msg.param_count
        if total and len(got) >= total:
            break
    return got, total

def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < 5:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            return msg.param_value
    return None

def fence_count(m):
    m.mav.mission_request_list_send(m.target_system, m.target_component, mission_type=1)
    msg = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
    return None if msg is None else msg.count

rec = {"date": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "link": LINK, "armed_ever": False,
       "note": "steps 4-5 in a second process: step 3's GCS connection held COM8"}
prev = json.loads((OUT / "fc_bench_part1.json").read_text()) if (OUT / "fc_bench_part1.json").exists() else {}
before = {}
for line in (OUT / "params_before.param").read_text().splitlines():
    k, v = line.split(","); before[k] = float(v)
# 4. mode requests (disarmed)
m, hb = connect()
modes = {}
mapping = m.mode_mapping()
for name in ("GUIDED", "BRAKE", "LOITER", "RTL", "LAND", "STABILIZE"):
    num = mapping.get(name)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    texts, t0, cur = [], time.time(), None
    while time.time() - t0 < 1.5:
        msg = m.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            texts.append(msg.text)
        elif msg.get_srcSystem() == m.target_system:
            cur = msg.custom_mode
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                rec["armed_ever"] = True
    modes[name] = {"custom_mode": num, "ack": None if ack is None else ack.result,
                   "mode_after": cur, "statustext": texts}
    log("mode", name, modes[name])
rec["modes"] = modes

# 5. restore
changed = {}
after_test, _ = all_params(m)
for k, v in before.items():
    if k in after_test and abs(after_test[k] - v) > 1e-6:
        changed[k] = {"before": v, "during": after_test[k], "restored": set_param(m, k, v)}
rec["params_restored"] = changed
m.mav.mission_clear_all_send(m.target_system, m.target_component, mission_type=1)
ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
rec["fence_clear_ack"] = None if ack is None else ack.type
rec["fence_items_after_clear"] = fence_count(m)
log("restored", list(changed), "fence clear ack", rec["fence_clear_ack"], "items", rec["fence_items_after_clear"])
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
rec["reboot_ack"] = None if ack is None else ack.result
m.close()
time.sleep(15)
m, hb = connect()
final, total = all_params(m)
diff = {k: [before.get(k), final.get(k)] for k in set(before) | set(final)
        if before.get(k) is None or final.get(k) is None or abs(before[k] - final[k]) > 1e-6}
rec["params_diff_after_reboot"] = diff
rec["fence_items_final"] = fence_count(m)
rec["armed_final"] = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
m.close()
log("final params differ:", diff, "fence items", rec["fence_items_final"], "armed", rec["armed_final"])
(OUT / "fc_bench_part2.json").write_text(json.dumps(rec, indent=1, default=str))
(OUT / "fc_bench_part2.log").write_text("\n".join(log_lines) + "\n")
print("done ->", OUT)
