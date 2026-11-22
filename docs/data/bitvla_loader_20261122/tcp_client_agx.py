"""
The real client against the real BitVLA server, both on the AGX, loopback
only (ARCH-06, 2026-11-22). The server is started first with the CLI
(demo/vla_server_foreign.py serve --model bitvla ... --host 127.0.0.1); this
connects with RemoteVLA (strict identity), runs RemoteVLABackend.act() for
--seconds on one fixed frame (the StepTracker in the hil launcher's rate
execution) and hands every Action4D to Shield.filter (policies/
demo_policy.yaml, open air). Flies nothing.

    ~/bitvla-env/bin/python docs/data/bitvla_loader_20261122/tcp_client_agx.py \
        --port 8797 --out ~/bitvla-run/out
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for _p in (ROOT, ROOT / "demo", HERE):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from guardrail import Shield                                         # noqa: E402
from guardrail import aerialvla_codec as C                           # noqa: E402
from guardrail.models import Action4D, State, load_policy            # noqa: E402
from guardrail.vla_backends import RemoteVLABackend                  # noqa: E402
from guardrail.vla_link import (Capture, Hint, Pose, RemoteVLA,      # noqa: E402
                                StepTracker, check_identity)
from shield_check import LAUNCHER_TRACKER, a4                        # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()
    out = Path(os.path.expanduser(a.out))
    from PIL import Image
    img = Image.open(ROOT / "docs/img/progress_0930_before_fpv.jpg").convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    jpg = buf.getvalue()

    cli = RemoteVLA("127.0.0.1", a.port, response_timeout_s=10.0, strict_identity=True)
    ident = cli.connect()
    check_identity(ident)
    state = State(x=-20.0, y=-20.0, up=4.0, yaw_deg=0.0)
    be = RemoteVLABackend(cli, lambda: Capture(time.monotonic(), Pose(-20, -20, 4, 0), jpg, None),
                          lambda cap: Hint("straight ahead ", "detector"), "a red car",
                          tracker=StepTracker(LAUNCHER_TRACKER))
    sh = Shield(load_policy(ROOT / "policies" / "demo_policy.yaml"))
    sh.filter(state, Action4D())
    ticks, t_end = [], time.time() + a.seconds
    while time.time() < t_end:
        act = be.act(state)
        d = sh.filter(state, act)
        ticks.append({"t": time.time(), "action4d": a4(act), "emitted": a4(d.emitted),
                      "violations": [v.rule_id for v in d.violations],
                      "finite": all(math.isfinite(v) for v in
                                    (act.vx, act.vy, act.vz_up, act.yaw_rate))})
        time.sleep(0.1)                                            # the rail's 10 Hz
    st = be.stats()
    be.close()
    res = {"identity": {k: ident.get(k) for k in ("id", "base", "weights_sha256", "precision",
                                                  "attn", "variant")},
           "family": C.foreign_family(ident["id"]), "seconds": a.seconds,
           "ticks": len(ticks), "all_finite": all(t["finite"] for t in ticks),
           "distinct_action4d": sorted({json.dumps(t["action4d"], sort_keys=True)
                                        for t in ticks}),
           "violations_seen": sorted({v for t in ticks for v in t["violations"]}),
           "backend_stats": st, "last_ticks": ticks[-5:]}
    (out / "tcp_client.json").write_text(json.dumps(res, indent=1, default=repr) + "\n",
                                         encoding="utf-8")
    print(json.dumps({k: res[k] for k in ("identity", "ticks", "all_finite",
                                          "distinct_action4d", "violations_seen")}, indent=1))
    print("decisions", st.get("decisions_accepted"), "decision_hz", st.get("decision_hz"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
