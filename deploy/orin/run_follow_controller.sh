#!/usr/bin/env bash
# The hand-written red-car follow (demo/follow_vlm.py: OWL-ViT + identity lock
# + the 10 Hz controller + the Shield) run ON THE JETSON against the desktop's
# Project AirSim (CityLife). The arguments are run_citylife_follow.ps1's for
# -LevelCar Car_10 (car mode), so a Jetson flight compares with the desktop's
# (citylife_redcar_nfz_motion2_20261019: 0.963 within 30 m).
#
#   SIM=192.168.50.1 TAG=orin_follow_01 bash deploy/orin/run_follow_controller.sh
#
# Start the simulator on the desktop first (deploy/orin/start_sim_for_jetson.ps1)
# and allow inbound TCP 8989-8990 there. Environment knobs:
#   SIM        the desktop's address (required)
#   TAG        the run folder under demo/out/ (default orin_follow_<date>)
#   SECONDS_   mission length in s after the start gate (default 420)
#   POLICY     policy YAML; its signed bundle must be in bundles/ (default the NFZ one)
#   SAVE_VIEW  1 = record view frames for the video (about 1.8 GB per 240 s), 0 = off
#   VENV       the Python environment (default ~/vla-drone-env)
#   EXTRA      more follow_vlm arguments, e.g. EXTRA="--seed 7"
set -euo pipefail
: "${SIM:?set SIM to the desktop address, e.g. SIM=192.168.50.1}"
TAG="${TAG:-orin_follow_$(date +%Y%m%d_%H%M%S)}"
SECONDS_="${SECONDS_:-420}"
POLICY="${POLICY:-policies/follow_car_citylife_nfz.yaml}"
SAVE_VIEW="${SAVE_VIEW:-1}"
VENV="${VENV:-$HOME/vla-drone-env}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source "$VENV/bin/activate"

for f in "$POLICY" demo/out/citymap_citylife/occ_day.npz demo/out/citymap_citylife/street.npz \
         demo/out/citymap_citylife/occ_day_flightband_6to14.npz; do
    [ -f "$f" ] || { echo "*** missing $f (deploy/orin/sync_follow_controller.ps1 copies it)"; exit 2; }
done
python - "$SIM" <<'EOF'
import socket, sys
for port in (8989, 8990):
    s = socket.socket(); s.settimeout(3)
    try:
        s.connect((sys.argv[1], port))
    except OSError as exc:
        sys.exit(f"*** {sys.argv[1]}:{port} unreachable ({exc}): is the simulator up, "
                 f"and does the desktop firewall allow it?")
    finally:
        s.close()
print(f"==> simulator reachable at {sys.argv[1]} (8989, 8990)")
EOF

args=(demo/follow_vlm.py --sim-address "$SIM"
      --object "a red car" --tag "$TAG" --policy "$POLICY"
      --max-s "$SECONDS_" --det-thresh 0.008 --cruise-alt 8
      --lock-target --no-car --presence-gates-control
      --start-when-seen 5 --start-timeout-s 330
      --level-peds 40 --level-truth-period 0.5
      --citymap demo/out/citymap_citylife/occ_day.npz
      --identity --land-site --search-planner
      --level-car Car_10 --want-width 0.16 --start-max-range-m 30
      --trail-follow --speed-max 5.0
      # FenceGuard measures and judges building clearance with the Shield's
      # ruler (WP3-22 / X-13; the default and only value since 2026-11-18).
      # Naming it does NOT make an older checkout fly that: one synced
      # between 497f016 and the one-path change accepts the flag but flies
      # its old opt-in field with its own `<=` tests, and records no
      # guard_field. The check after the flight refuses such a run.
      --guard-field shield)
[ "$SAVE_VIEW" = "1" ] && args+=(--save-view)
# shellcheck disable=SC2206
[ -n "${EXTRA:-}" ] && args+=($EXTRA)
rm -f "demo/out/$TAG/metrics.json"
echo "==> code $(git rev-parse HEAD 2>/dev/null || echo unversioned)$(git diff --quiet HEAD 2>/dev/null || echo ' (dirty)')"
echo "==> python ${args[*]}"
python "${args[@]}"
# The run is evidence for the one-path building guard only if THIS checkout's
# FenceGuard flew the Shield's ruler, which follow_vlm records in metrics.json.
python - "demo/out/$TAG/metrics.json" <<'EOF'
import json, sys
gf = json.load(open(sys.argv[1], encoding="utf-8")).get("guard_field")
if gf != "shield":
    sys.exit(f"*** metrics.json guard_field = {gf!r}, not 'shield': this checkout's "
             f"FenceGuard did not fly the Shield's ruler (synced before the WP3-22 / X-13 "
             f"one-path change?). Not evidence for the one-path guard: sync and re-fly.")
print("==> metrics.json guard_field = 'shield' (FenceGuard flew the Shield's ruler)")
EOF
echo "==> read demo/out/$TAG/metrics.json (frac_within_30m, det_hz) and kpi.json"
