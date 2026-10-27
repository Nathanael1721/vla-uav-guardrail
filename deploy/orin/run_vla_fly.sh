#!/usr/bin/env bash
# The VLA follow end to end (demo/vla_fly.py: the VLA flies every step, the
# Shield filters it) run ON THE JETSON against the desktop's Project AirSim
# and a VLA server on the desktop's GPU (the 7B models do not fit the Orin
# Nano). The arguments are run_vla_fly.ps1's for the red-car follow, so a
# Jetson flight compares with the desktop's cmp2_e2e_* flights.
#
#   SIM=10.42.0.1 VLA=127.0.0.1:8790 TAG=orin_e2e_aerovla bash deploy/orin/run_vla_fly.sh
#
# On the desktop first: the simulator (deploy/orin/start_sim_for_jetson.ps1)
# and the VLA server on its usual 127.0.0.1 port, carried to the Jetson by an
# SSH reverse tunnel the desktop opens (outbound only, no firewall change):
#   ssh -o BatchMode=yes -N -R 8790:127.0.0.1:8790 nathan@10.42.0.2
# so the Jetson reaches the server at its own 127.0.0.1:8790. Environment knobs:
#   SIM        the desktop's address (required)
#   VLA        the server, HOST:PORT as the Jetson sees it (required)
#   TAG        the run folder under demo/out/ (default orin_e2e_<date>)
#   SECONDS_   mission length in s (default 420)
#   BUNDLE     the signed policy bundle (default the NFZ follow's)
#   VENV       the Python environment (default ~/vla-drone-env)
#   EXTRA      more vla_fly arguments, e.g. EXTRA="--step-as-rate 1.0"
set -euo pipefail
: "${SIM:?set SIM to the desktop address, e.g. SIM=10.42.0.1}"
: "${VLA:?set VLA to the server, e.g. VLA=10.42.0.1:8790}"
TAG="${TAG:-orin_e2e_$(date +%Y%m%d_%H%M%S)}"
SECONDS_="${SECONDS_:-420}"
BUNDLE="${BUNDLE:-bundles/follow-car-citylife-nfz-v0.1.0.tar.gz}"
VENV="${VENV:-$HOME/vla-drone-env}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source "$VENV/bin/activate"

for f in "$BUNDLE" demo/out/citymap_citylife/occ_day.npz demo/out/citymap_citylife/street.npz; do
    [ -f "$f" ] || { echo "*** missing $f (deploy/orin/sync_follow_controller.ps1 copies it)"; exit 2; }
done
python - "$SIM" "${VLA%:*}" "${VLA##*:}" <<'EOF'
import socket, sys
sim, vla_host, vla_port = sys.argv[1], sys.argv[2], int(sys.argv[3])
for host, port, what in ((sim, 8989, "simulator"), (sim, 8990, "simulator"),
                         (vla_host, vla_port, "VLA server")):
    s = socket.socket(); s.settimeout(3)
    try:
        s.connect((host, port))
    except OSError as exc:
        sys.exit(f"*** {what} {host}:{port} unreachable ({exc}): is it up, and for the "
                 f"server is the desktop's reverse tunnel (ssh -N -R) open?")
    finally:
        s.close()
print(f"==> simulator {sim} (8989, 8990) and VLA server {vla_host}:{vla_port} reachable")
EOF

args=(demo/vla_fly.py --mission follow --tag "$TAG" --max-s "$SECONDS_"
      --cadence pipelined --citymap demo/out/citymap_citylife/occ_day.npz
      --level-car Car_10 --cruise-mps 4.5 --speed-cap-mps 5
      --vla-server "$VLA" --sim-address "$SIM" --bundle "$BUNDLE"
      --cruise-alt 8 --lost-timeout-s 600 --close-bottom-frac 1.0
      --det-thresh 0.008 --acquire-max-range-m 60)
# shellcheck disable=SC2206
[ -n "${EXTRA:-}" ] && args+=($EXTRA)
rm -f "demo/out/$TAG/metrics.json"
echo "==> python ${args[*]}"
python "${args[@]}"
echo "==> read demo/out/$TAG/metrics.json (frac_within_30m_after_acquire) and vla_steps.jsonl"
