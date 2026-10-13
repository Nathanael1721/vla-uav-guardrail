#!/usr/bin/env bash
# SROS 2 keystore and enclaves for the dev graph, seeded with the harness CA.
#
#   bash tools/sros2_setup.sh --pki <harness_pki folder> [--keystore DIR] [--dry-run]
#
# WHAT THE GRANT ASKS FOR
#
# Stress Testing page, "Locked design choices": "Control plane: gRPC over
# mTLS. Same root CA as SROS 2 for cert reuse." The reference design's
# data-flow page: "SROS 2 turned on for the ROS 2 graph using the same root
# CA". Audit card X-15 recorded that neither existed. The gRPC half is built
# (guardrail/harness_service.py, gen-pki writes the lab root CA, ECDSA P-256,
# as sros2's ca.cert.pem / ca.key.pem). This script is the ROS 2 half: it
# makes a SROS 2 keystore whose identity and permissions CA IS that root, and
# one enclave per node of the dev graph.
#
# WHERE IT RUNS
#
# On the ROS 2 host (WSL Ubuntu on the desktop, or the Orin's ROS 2
# container), with ROS 2 (Humble or Jazzy) sourced and openssl on the PATH.
# It has NOT been run where it was written (no ROS 2 there); `bash -n` checks
# its syntax and `--dry-run` prints every command without running any.
#
#   --pki DIR        the folder `python -m guardrail.harness_service gen-pki`
#                    wrote. On Windows that is
#                    %LOCALAPPDATA%\vla-guardrail\harness_pki, which WSL sees
#                    as /mnt/c/Users/<you>/AppData/Local/vla-guardrail/harness_pki.
#                    Also $HARNESS_PKI.
#   --keystore DIR   default ~/sros2_keystore. On the Linux filesystem: a
#                    keystore under /mnt/ (a Windows drive, maybe synced, where
#                    chmod is not enforced) is refused unless --allow-mnt.
#   --dry-run        print the commands, run none.
#   --force          seed a keystore that already holds another CA key.
#
# WHAT IT DOES
#
#   1. Copies the lab CA into keystore/public/ca.cert.pem and
#      keystore/private/ca.key.pem (umask 077: the key is owner-only).
#   2. `ros2 security create_keystore keystore`. sros2 reuses a CA pair it
#      finds there ("found CA key/cert pair") and points identity_ca and
#      permissions_ca at it - our reading of the sros2 source; step 3 is the
#      check, not an assumption.
#   3. Compares the SHA-256 fingerprints of identity_ca.cert.pem and
#      permissions_ca.cert.pem with the lab CA's. A difference means sros2
#      made its own CA: the script stops, and the keystore must be rebuilt.
#   4. `ros2 security create_enclave` for each node of the dev graph
#      (sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py and
#      sitl/run_ros2_demo.sh): the Shield node, the MAVLink adapter, the VLA
#      stub, MAVROS, and the rail's bag recorder.
#   5. `openssl verify -CAfile <pki>/ca.cert.pem` on every enclave
#      certificate.
#   6. Prints the environment that enforces security and the enclave each
#      process must run with.
#
# WHAT IT DOES NOT DO
#
#   * Write permissions narrower than sros2's default. create_enclave grants
#     the enclave every topic; topic-level permissions (the Shield may
#     publish /shield/setpoint, the VLA only /vla/action_4d, ...) are a policy
#     XML per enclave, still to be written.
#   * Start any node with its enclave. The launch file gives the three nodes
#     one shared environment, so each needs its own
#     ROS_SECURITY_ENCLAVE_OVERRIDE (or `--ros-args --enclave`); MAVROS and
#     `ros2 bag record` take `--ros-args --enclave` in sitl/run_ros2_demo.sh.
#     Those files are not changed here.
#   * Copy the CA key anywhere else. It seeds this keystore on this host; the
#     Orin and the drone get enclave keys, never the CA key.
set -euo pipefail

PKI="${HARNESS_PKI:-}"
KEYSTORE="${HOME}/sros2_keystore"
DRY=0
FORCE=0
ALLOW_MNT=0
# The dev graph's ROS 2 processes, as enclave paths. The rclpy node names are
# safety_shield, mavlink_adapter and vla_stub; MAVROS runs as mavros_node.
ENCLAVES=(/guardrail/safety_shield /guardrail/mavlink_adapter /guardrail/vla_stub
          /guardrail/mavros /guardrail/bag_recorder)

usage() {
  sed -n '2,4p' "$0"
  exit 2
}

while [ $# -gt 0 ]; do
  case "$1" in
    --pki) PKI="${2:?--pki needs a folder}"; shift 2 ;;
    --keystore) KEYSTORE="${2:?--keystore needs a folder}"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --force) FORCE=1; shift ;;
    --allow-mnt) ALLOW_MNT=1; shift ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1" >&2; usage ;;
  esac
done

die() { echo "sros2_setup: $*" >&2; exit 1; }

# Every step that changes something goes through run(): printed, and run
# unless --dry-run.
run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  if [ "$DRY" -eq 0 ]; then
    "$@"
  fi
}

fingerprint() {
  openssl x509 -noout -fingerprint -sha256 -in "$1" | sed 's/^.*=//'
}

[ -n "$PKI" ] || die "--pki is required: the folder gen-pki wrote (ca.cert.pem, ca.key.pem)"
CA_CERT="$PKI/ca.cert.pem"
CA_KEY="$PKI/ca.key.pem"
case "$KEYSTORE" in
  /mnt/*)
    [ "$ALLOW_MNT" -eq 1 ] || die "keystore $KEYSTORE is on a Windows drive (/mnt/): chmod is not enforced there and the folder may be synced; use the Linux filesystem, or --allow-mnt on purpose" ;;
esac

if [ "$DRY" -eq 0 ]; then
  [ -f "$CA_CERT" ] || die "no $CA_CERT (run: python -m guardrail.harness_service gen-pki)"
  [ -f "$CA_KEY" ] || die "no $CA_KEY"
  command -v openssl >/dev/null || die "openssl is not on the PATH"
  command -v ros2 >/dev/null || die "ros2 is not on the PATH: source /opt/ros/<distro>/setup.bash first"
  ros2 security --help >/dev/null 2>&1 || die "ros2 security is not available (install ros-<distro>-sros2)"
  if [ -f "$KEYSTORE/private/ca.key.pem" ] && ! cmp -s "$KEYSTORE/private/ca.key.pem" "$CA_KEY"; then
    [ "$FORCE" -eq 1 ] || die "$KEYSTORE already holds another CA key: every enclave it signed would stop verifying. Use another --keystore, or --force on purpose"
  fi
  CA_FP="$(fingerprint "$CA_CERT")"
  echo "lab CA $CA_CERT"
  echo "  SHA-256 $CA_FP"
else
  CA_FP="<the fingerprint gen-pki printed>"
  echo "(dry run: nothing is read, written or started)"
fi

# 1. seed the keystore with the lab CA
umask 077
run mkdir -p "$KEYSTORE/public" "$KEYSTORE/private"
run cp "$CA_CERT" "$KEYSTORE/public/ca.cert.pem"
run cp "$CA_KEY" "$KEYSTORE/private/ca.key.pem"
run chmod 600 "$KEYSTORE/private/ca.key.pem"

# 2. the keystore around it
run ros2 security create_keystore "$KEYSTORE"

# 3. did sros2 use the lab CA?
for which in identity_ca permissions_ca; do
  f="$KEYSTORE/public/$which.cert.pem"
  if [ "$DRY" -eq 1 ]; then
    echo "+ check: fingerprint of $f == $CA_FP"
    continue
  fi
  [ -f "$f" ] || die "create_keystore wrote no $f"
  got="$(fingerprint "$f")"
  [ "$got" = "$CA_FP" ] || die "$which is $got, not the lab CA $CA_FP: sros2 made its own CA. Remove $KEYSTORE and rerun"
  echo "  $which = lab CA"
done

# 4. one enclave per process of the dev graph
for enclave in "${ENCLAVES[@]}"; do
  run ros2 security create_enclave "$KEYSTORE" "$enclave"
done

# 5. every enclave certificate chains to the lab CA
for enclave in "${ENCLAVES[@]}"; do
  run openssl verify -CAfile "$CA_CERT" "$KEYSTORE/enclaves$enclave/cert.pem"
done

# 6. how the graph runs with it
cat <<EOF

Keystore ready: $KEYSTORE (identity and permissions CA = the lab root CA).
Enforce security in every shell that starts a node:

  export ROS_SECURITY_KEYSTORE=$KEYSTORE
  export ROS_SECURITY_ENABLE=true
  export ROS_SECURITY_STRATEGY=Enforce

Each process with its own enclave:

  ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/safety_shield   python3 sitl/ros2_shield_node.py ...
  ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/mavlink_adapter python3 sitl/mavlink_adapter_node.py ...
  ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/vla_stub        python3 sitl/ros2_vla_stub_node.py ...
  ros2 run mavros mavros_node --ros-args --enclave /guardrail/mavros -p fcu_url:=...
  ros2 bag record -s mcap -o <out>/bag --ros-args --enclave /guardrail/bag_recorder ...

A process started without an enclave cannot join the graph under Enforce;
that is the test that it is on: with the variables above, start the Shield
node without its override and check it is refused.
EOF
