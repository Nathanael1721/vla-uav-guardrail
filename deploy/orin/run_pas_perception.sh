#!/usr/bin/env bash
# The perception rail's hil follow flown by a VLA - by default the lab's own,
# AIOT-VLA - ON THE AGX ORIN (docs/DESIGN-projectairsim-ardupilot.md, "hil"):
# the perception node
# (demo/pas_ardupilot/perception_node.py --pilot vla --mission follow) runs
# natively on the Orin - ROS 2 Jazzy's rclpy and mavros_msgs from apt, and
# ~/vla-drone-env for torch, the detector and the Project AirSim client, which
# the companion image never carries - beside MAVROS 2 in the companion
# container on the same Orin and the VLA server on 127.0.0.1; Project
# AirSim CityLife and ArduPilot SITL run on the desktop.
#
#   G1_VERDICT=~/g1_verdict.json TAG=pas_hil_aiot_01 bash deploy/orin/run_pas_perception.sh [node args...]
#   FAMILY=aerialvla G1_VERDICT=~/g1_verdict.json bash deploy/orin/run_pas_perception.sh [node args...]
#
# First, in this order (the design doc has each command):
#   desktop  .\scripts\run_pas_ardupilot.ps1 -Step scene -VlaCameras
#            .\scripts\run_pas_ardupilot.ps1 -Step sitl -SkipSim -Orin <the Orin's address>
#   Orin     docker compose --env-file deploy/topologies/hil.env \
#                -f deploy/compose/docker-compose.orin.yml up -d mavros
#            ~/vla-drone-env/bin/python demo/vla_server_foreign.py serve --model aiotvla \
#                --weights ~/models/aiotvla/v10b --port 8791 --front-source detector --horizon-s 0.5
#
# FAMILY: WHICH MODEL THE SERVER AT VLA_PORT IS (default aiotvla). The node is
# the same and no code changes between families; what changes is the option
# set a family's model needs, and the server started before this script. One
# server at a time, on VLA_PORT. The option sets are the ones each family
# flew with on demo/vla_fly.py in the 3-4 November AGX matrix (each run's
# metrics.json `params`), on this node's own follow defaults:
#   aiotvla    (default) exactly the option set this script passed before
#              FAMILY existed (afd040c): the detector's FrontCamera frame,
#              each step flown as a rate over its 0.5 s label horizon with
#              the yaw as a rate, pipelined, the feed between ticks
#              (VLA_FEED stream). The stream feed is the default since
#              2026-11-15: the hil follows before it flew one request per
#              tick (what VLA_FEED=tick gives; their params predate the
#              option), and pas_hil_feed_01 .. _04 a stream capped at 12 Hz,
#              so reproducing one of those takes VLA_FEED=tick, or the node
#              argument --vla-feed-max-hz 12 (each run's metrics.json params).
#              Server: the command above (demo/vla_server_foreign.py serve
#              --model aiotvla --weights <v10b folder> --port <VLA_PORT>
#              --front-source detector --horizon-s 0.5; ~/vla-drone-env)
#   aerialvla  VLAFront + VLADown (the scene step's -VlaCameras), steps flown
#              as steps, pipelined, one request per tick.
#              Server: demo/vla_server_v2.py serve --base ~/models/openvla-7b
#              --adapter ~/models/aerialvla-lora/aero_vla --precision bf16
#              --attn sdpa --merge --missing-down refuse --port <VLA_PORT>
#              (~/vla-drone-env)
#   dronecats  the detector's FrontCamera frame, steps flown as steps,
#              pipelined, one request per tick, a 30 s answer timeout (its
#              planning answers outlasted 10 s on the board, CHANGELOG
#              2026-11-04).
#              Server: demo/vla_server_foreign.py serve --model dronecats
#              --weights <DroneCATS-Cosmos3-Edge-2B folder> --port <VLA_PORT>
#              --front-source detector  (its OWN venv, ~/vla-dronecats-env,
#              deploy/orin/agx_dronecats_env.sh: transformers 5)
#   servo      NO VLA: the node's hand-written detector follow (--pilot servo
#              --mission follow), the backend table's C2 null
#              (tools/vla_backend_table.py). No VLA option is passed, no VLA
#              server is needed or checked; without --target-estimator the node
#              flies the box servo (with it, estimator below); --identity and
#              --negative-query are the follow's own and pass through.
#   estimator  NO VLA: the lab's hand-written follow WITH its target estimator
#              (--pilot servo --mission follow --identity --target-estimator:
#              the node's EstimatorFollowPilot, follow_vlm.fly()'s servo on the
#              estimate over demo/subject_track.SubjectTrack's estimator, strict
#              lock and Reacquirer, forward along the subject's trail as
#              fly()'s --trail-follow; not all of fly(): no FenceGuard, slew
#              limiter or search planner), at --want-width 0.16 and --speed-max 5, the
#              values agx_manual_20261103 (follow_vlm.py on simple_flight, 0.952
#              within 30 m) flew with: the backend table's C2 estimator null, kept
#              apart from servo's. No VLA server is needed or checked.
#   racevla    VLAFront, each step flown as a rate over 1.0 s (the server's
#              --horizon-s default), pipelined, one request per tick.
#              Server: demo/vla_server_foreign.py serve --model racevla
#              --weights <drone6_all>/<run folder> --port <VLA_PORT>
#              (~/vla-drone-env). The checkpoint is the run folder one level
#              under drone6_all (openvla-7b+drone_set6_all+...), as that
#              server's own docstring gives it; the server names the model
#              drone6_all from the parent either way.
# WHAT EACH FAMILY STILL LACKS HERE (updated 2026-11-16 evening, after the
# backend table's C2 flights; deploy/evidence/vla_backend_table.md has each
# flight). aerialvla, dronecats and racevla have now flown this node's
# follow on hil, with P0 escapes 0 on every flight; none of the three
# followed the car the way aiotvla does; servo (the C2 null) has not flown:
#   aerialvla  a shakedown and 3 C2 flights; each lost the car (target_lost),
#              within 30 m of it on 0.05-0.16 of the ticks. It answers in
#              about 1.1 s on the board (bf16, merged), so the feed between
#              ticks is left off.
#   dronecats  a shakedown and 3 C2 flights; each lost the car, within 30 m
#              on 0.13-0.19 of the ticks. Its server runs from another venv
#              than the node.
#   racevla    a shakedown and 1 C2 flight that lost the car (within 30 m on
#              0.06 of the ticks); 2 more flights never started: the start
#              gate (the detector's, not the model's) found no red car within
#              30 m in 330 s. Its forward commands stayed under 0.3 m/s on
#              simple_flight (CHANGELOG 2026-10-27).
#   servo      the C2 null: not yet flown through this script.
#   estimator  the C2 estimator null (added 2026-11-17): not yet flown at all;
#              its law has run only on the desktop's stand-ins
#              (tests/test_pas_estimator_follow.py).
# A first flight of each needs a shakedown (SECONDS_=60) before a 420 s one,
# with _shake in its TAG (TAG=pas_hil_<family>_shake_01): the backend table
# (tools/vla_backend_table.py, C2) does not count a flight so marked.
#
# The ROS 2 graph is hil.env's, read from it here and never copied: the domain,
# the RMW, the Cyclone DDS peers (deploy/dds/cyclonedds.xml expands them), and
# MAVROS's FCU_URL as MAVROS_FCU_URL, which the node's link record falls back
# to; the simulator is hil.env's DESKTOP_IP. Refused here: --dev-yaml (an
# unsigned YAML policy is never KPI-grade), --start-sitl (ArduPilot runs on
# the desktop, after its scene) and the flags that fix the topology (--link,
# --scene-mode / --scene-only, --sim-host, --camera), in any abbreviation
# argparse would take. Environment knobs:
#   G1_VERDICT  the desktop's demo/out/g1_*/g1_verdict.json, copied here (required)
#   FAMILY      aiotvla (default), aerialvla, dronecats, racevla, servo or estimator
#               (no VLA: the C2 nulls): above
#   TAG         the run folder under demo/out/ (default pas_hil_aiot_<date>,
#               pas_hil_aiot_<stress>_<date> with STRESS; pas_hil_<family>_...
#               for another family)
#   SECONDS_    mission length in s (default 420)
#   BUNDLE      a signed policy bundle (default: the node's own, follow_car_citylife's)
#   FRONT_CAPTURE_INTERVAL_S  FrontCamera's capture interval the desktop's scene
#               step was given (-FrontCaptureIntervalS), recorded with the run
#   STRESS      the perception stressor the desktop's scene step was given
#               (-Stress day|dusk|night|rain|fog|shadow|texture; demo/pas_ardupilot/
#               perception_stress.py): the node attaches only to that
#               stressor's scene and reads its weather back; never KPI-grade
#   VLA_PORT    the VLA server's port on 127.0.0.1 (default 8791, every family)
#   VLA_FEED    stream (aiotvla's default): a new front frame goes to the model
#               between ticks too, the grant's 10 Hz observation (needs the
#               scene's -FrontCaptureIntervalS 0.05: a 15 Hz camera); tick (the
#               other families' default): one request per 10 Hz tick at most,
#               as every flight before 2026-11-15
#   VENV        the Python environment (default ~/vla-drone-env)
#   ROS_SETUP   ROS 2's setup script (default /opt/ros/jazzy/setup.bash)
#   VLAGUARD_REQUIRE_CHAIN  1 = the node flies the bundle only if its signer's
#               certificate chained to a listed authority here (guardrail.bundle
#               load_for_flight, WP1-04); default hil.env's value (0 when it sets
#               none). The node runs natively, outside compose, so this script
#               is what hands it on; the run's policy_source records it
#               (require_chain)
# VLAGUARD_DRY_RUN=1 prints the graph's values (ENV:) and the command (EXEC:)
# and starts nothing (tests/test_deploy_configs.py).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HIL_ENV="$ROOT/deploy/topologies/hil.env"
DRY="${VLAGUARD_DRY_RUN:-0}"

die() { echo "*** $2" >&2; exit "$1"; }

# One value of hil.env (KEY=VALUE lines; the last one wins).
topo() {
  local v
  v="$(sed -n "s/^$1=//p" "$HIL_ENV" | tail -n 1 | tr -d '\r')"
  [ -n "$v" ] || die 64 "$HIL_ENV sets no $1"
  printf '%s' "$v"
}

for a in "$@"; do
  case "$a" in
    --dev*)
      die 64 "$a: a YAML policy flown with --dev-yaml is a development run, never KPI-grade; this is the hil run" ;;
    --start-s*)
      die 64 "$a: ArduPilot SITL runs on the desktop in hil (run_pas_ardupilot.ps1 -Step sitl -Orin), after its scene" ;;
    --lin*|--scene*|--sim*|--cam*)
      die 64 "$a: the hil topology fixes it here (--link mavros --scene-mode attach --sim-host <hil.env DESKTOP_IP>)" ;;
  esac
done

# (No apostrophe in this message: inside "${...:?word}" bash reads one as a quote.)
: "${G1_VERDICT:?set G1_VERDICT to a copy of the desktop g1_verdict.json (demo/out/g1_*/)}"
[ -f "$G1_VERDICT" ] || die 66 "no G1 verdict at $G1_VERDICT (copy the desktop's demo/out/g1_*/g1_verdict.json here)"
# Absolute: the node runs from the checkout's root (cd below), not from here.
G1_VERDICT="$(cd "$(dirname "$G1_VERDICT")" && pwd)/$(basename "$G1_VERDICT")"

# The family's option set (header, FAMILY). aiotvla's is word for word the
# set this script passed before the switch existed, in the same order, so a
# run without FAMILY is the run it always was.
FAMILY="${FAMILY:-aiotvla}"
case "$FAMILY" in
  aiotvla)
    tag_family=aiot
    family_args=(--vla-front-source detector --vla-step-as-rate 0.5 --vla-rate-yaw rate
                 --cadence pipelined --allow-unstamped-pipelined
                 --vla-feed "${VLA_FEED:-stream}"
                 --vla-cruise-mps 4.5 --vla-speed-cap-mps 5) ;;
  aerialvla)
    tag_family=aerialvla
    family_args=(--vla-front-source vla
                 --cadence pipelined --allow-unstamped-pipelined
                 --vla-feed "${VLA_FEED:-tick}"
                 --vla-cruise-mps 4.5 --vla-speed-cap-mps 5) ;;
  dronecats)
    tag_family=dronecats
    family_args=(--vla-front-source detector
                 --cadence pipelined --allow-unstamped-pipelined
                 --vla-feed "${VLA_FEED:-tick}" --vla-decision-timeout-s 30
                 --vla-cruise-mps 4.5 --vla-speed-cap-mps 5) ;;
  racevla)
    tag_family=racevla
    family_args=(--vla-front-source vla --vla-step-as-rate 1.0
                 --cadence pipelined --allow-unstamped-pipelined
                 --vla-feed "${VLA_FEED:-tick}"
                 --vla-cruise-mps 4.5 --vla-speed-cap-mps 5) ;;
  servo)
    tag_family=servo
    family_args=() ;;
  estimator)
    tag_family=estimator
    family_args=(--identity --target-estimator --want-width 0.16 --speed-max 5) ;;
  *)
    die 64 "FAMILY '$FAMILY' is none of aiotvla aerialvla dronecats racevla servo estimator" ;;
esac
TAG="${TAG:-pas_hil_${tag_family}_${STRESS:+${STRESS}_}$(date +%Y%m%d_%H%M%S)}"
case "$TAG" in
  ""|*/*|.*) die 64 "TAG '$TAG' must be a plain directory name under demo/out" ;;
esac
SECONDS_="${SECONDS_:-420}"
VLA_PORT="${VLA_PORT:-8791}"
VENV="${VENV:-$HOME/vla-drone-env}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# The ROS 2 graph MAVROS in the companion container is on (hil.env, compose's
# companion environment), for this host's node.
ROS_DOMAIN_ID="$(topo ROS_DOMAIN_ID)"
RMW_IMPLEMENTATION="$(topo RMW_IMPLEMENTATION)"
VLAGUARD_DDS_PEER_A="$(topo VLAGUARD_DDS_PEER_A)"
VLAGUARD_DDS_PEER_B="$(topo VLAGUARD_DDS_PEER_B)"
MAVROS_FCU_URL="$(topo FCU_URL)"
DESKTOP_IP="$(topo DESKTOP_IP)"
CYCLONEDDS_URI="file://$ROOT/deploy/dds/cyclonedds.xml"
# The caller's value wins, as compose's ${VLAGUARD_REQUIRE_CHAIN:-0} lets the
# shell's win there; else hil.env's; else 0 (a hil.env from before 2026-11-16).
if [ -z "${VLAGUARD_REQUIRE_CHAIN:-}" ]; then
  VLAGUARD_REQUIRE_CHAIN="$(sed -n 's/^VLAGUARD_REQUIRE_CHAIN=//p' "$HIL_ENV" | tail -n 1 | tr -d '\r')"
  VLAGUARD_REQUIRE_CHAIN="${VLAGUARD_REQUIRE_CHAIN:-0}"
fi
export ROS_DOMAIN_ID RMW_IMPLEMENTATION VLAGUARD_DDS_PEER_A VLAGUARD_DDS_PEER_B
export MAVROS_FCU_URL CYCLONEDDS_URI VLAGUARD_REQUIRE_CHAIN

args=(demo/pas_ardupilot/perception_node.py
      --link mavros --scene-mode attach --sim-host "$DESKTOP_IP"
      --g1-verdict "$G1_VERDICT" --tag "$TAG" --max-s "$SECONDS_"
      --pilot vla --mission follow --vla-server "127.0.0.1:$VLA_PORT"
      ${family_args[@]+"${family_args[@]}"})
# The C2 nulls fly the hand-written follow: no VLA pilot, no server.
if [ "$FAMILY" = servo ] || [ "$FAMILY" = estimator ]; then
  args=(demo/pas_ardupilot/perception_node.py
        --link mavros --scene-mode attach --sim-host "$DESKTOP_IP"
        --g1-verdict "$G1_VERDICT" --tag "$TAG" --max-s "$SECONDS_"
        --pilot servo --mission follow
        ${family_args[@]+"${family_args[@]}"})
fi
if [ -n "${BUNDLE:-}" ]; then
  args+=(--bundle "$BUNDLE")
fi
if [ -n "${FRONT_CAPTURE_INTERVAL_S:-}" ]; then
  args+=(--front-capture-interval-s "$FRONT_CAPTURE_INTERVAL_S")
fi
if [ -n "${STRESS:-}" ]; then
  case "$STRESS" in
    day|dusk|night|rain|fog|shadow|texture) args+=(--stress "$STRESS") ;;
    *) die 64 "STRESS '$STRESS' is none of day dusk night rain fog shadow texture (perception_stress.py)" ;;
  esac
fi
args+=("$@")

cd "$ROOT"
if [ "$DRY" = "1" ]; then
  for k in ROS_DOMAIN_ID RMW_IMPLEMENTATION VLAGUARD_DDS_PEER_A VLAGUARD_DDS_PEER_B \
           MAVROS_FCU_URL CYCLONEDDS_URI VLAGUARD_REQUIRE_CHAIN; do
    printf 'ENV: %s=%s\n' "$k" "${!k}"
  done
  printf 'EXEC: python'
  printf ' %q' "${args[@]}"
  printf '\n'
  exit 0
fi

for f in demo/out/citymap_citylife/occ_day.npz demo/out/citymap_citylife/occ_day_flightband_6to14.npz; do
  [ -f "$f" ] || die 2 "missing $f (deploy/orin/sync_follow_controller.ps1 copies it)"
done
[ -f "$ROS_SETUP" ] || die 2 "no ROS 2 at $ROS_SETUP (apt: ros-jazzy-ros-base ros-jazzy-mavros-msgs ros-jazzy-rmw-cyclonedds-cpp)"
rmw_lib="$(dirname "$ROS_SETUP")/lib/lib${RMW_IMPLEMENTATION}.so"
[ -e "$rmw_lib" ] || die 2 "hil.env's RMW $RMW_IMPLEMENTATION is not installed beside ROS 2 ($rmw_lib)"
[ -f "$VENV/bin/activate" ] || die 2 "no Python environment at $VENV (deploy/orin/agx_vla_env.sh builds it)"
# ROS's setup script and the venv's activate read unset variables.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
set -u

python - <<'EOF'
import importlib
import sys
bad = []
for m in ("rclpy", "mavros_msgs.msg", "torch", "transformers", "projectairsim"):
    try:
        importlib.import_module(m)
    except Exception as exc:                                      # noqa: BLE001
        bad.append(f"{m} ({type(exc).__name__}: {exc})")
if bad:
    sys.exit("*** not importable with ROS 2 sourced and the venv active: " + "; ".join(bad)
             + " (ROS 2 from apt: ros-jazzy-ros-base, ros-jazzy-mavros-msgs; the rest: "
               "deploy/orin/agx_vla_env.sh)")
print(f"==> rclpy, mavros_msgs, torch, transformers and projectairsim import "
      f"(Python {sys.version.split()[0]})")
EOF
python - "$DESKTOP_IP" "$VLA_PORT" "$FAMILY" <<'EOF'
import socket
import sys
desk, vla_port, family = sys.argv[1], int(sys.argv[2]), sys.argv[3]
no_vla = family in ("servo", "estimator")
ends = [(desk, 8989, "Project AirSim (topics)"), (desk, 8990, "Project AirSim (services)")]
if not no_vla:
    ends.append(("127.0.0.1", vla_port, f"the VLA server (FAMILY={family})"))
for host, port, what in ends:
    s = socket.socket()
    s.settimeout(3)
    try:
        s.connect((host, port))
    except OSError as exc:
        sys.exit(f"*** {what} {host}:{port} unreachable ({exc}): is it up (and, for the "
                 f"simulator, does the desktop's firewall let the Orin in)?")
    finally:
        s.close()
print(f"==> Project AirSim {desk} (8989, 8990)"
      + ("" if no_vla else f" and the VLA server 127.0.0.1:{vla_port}")
      + " reachable")
EOF

echo "==> ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION peers" \
     "$VLAGUARD_DDS_PEER_A $VLAGUARD_DDS_PEER_B; MAVROS fcu_url $MAVROS_FCU_URL;" \
     "VLAGUARD_REQUIRE_CHAIN=$VLAGUARD_REQUIRE_CHAIN"
rm -f "demo/out/$TAG/metrics.json"
echo "==> python ${args[*]}"
exec python "${args[@]}"
