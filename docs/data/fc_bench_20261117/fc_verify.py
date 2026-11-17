import json, os, sys, time
os.environ.setdefault("MAVLINK20", "1")
from pathlib import Path
from pymavlink import mavutil
OUT = Path(sys.argv[1])
before = {}
for line in (OUT / "params_before.param").read_text().splitlines():
    k, v = line.split(","); before[k] = float(v)
m = mavutil.mavlink_connection("COM8", baud=115200, source_system=255, source_component=191)
hb = m.wait_heartbeat(timeout=30)
m.mav.param_request_list_send(m.target_system, m.target_component)
got, total, t0 = {}, None, time.time()
while time.time() - t0 < 60:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg is None:
        if total and len(got) >= total: break
        continue
    got[msg.param_id] = msg.param_value; total = msg.param_count
    if total and len(got) >= total: break
m.mav.mission_request_list_send(m.target_system, m.target_component, mission_type=1)
mc = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
diff = {k: [before.get(k), got.get(k)] for k in set(before) | set(got)
        if before.get(k) is None or got.get(k) is None or abs(before[k] - got[k]) > 1e-6}
rec = {"params_read": [len(got), total], "diff_vs_backup": diff,
       "fence_items": None if mc is None else mc.count,
       "armed": bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
       "custom_mode": hb.custom_mode}
(OUT / "fc_verify.json").write_text(json.dumps(rec, indent=1))
print(json.dumps(rec, indent=1))
