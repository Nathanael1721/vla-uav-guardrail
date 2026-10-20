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
# No root needed: everything it writes is under --keystore. First RUN on
# 2026-10-19, in WSL Ubuntu 24.04 with ROS 2 Jazzy (ros-jazzy-sros2 0.13.6,
# OpenSSL 3.0.13) against a throwaway lab PKI (docs/DESIGN-sros2.md); until
# then it had been checked with `bash -n` and `--dry-run` only.
#
# WHAT THAT FIRST RUN FOUND. The script as written on 2026-10-12 copied the
# lab CA in as ca.cert.pem / ca.key.pem and relied on create_keystore reusing
# "a CA pair it finds there". It does not: sros2 0.13.6 reuses a CA only when
# identity_ca.* AND permissions_ca.* (cert and key) are all present, and
# otherwise writes a NEW CA over ca.cert.pem / ca.key.pem ("sros2CA") and
# links both roles to it. Step 3's fingerprint check caught it ("identity_ca
# is E1:4E:..., not the lab CA 61:17:...: sros2 made its own CA") - the check
# worked, the procedure did not. Step 1 now also links identity_ca.* and
# permissions_ca.* to the copied CA, as sros2 itself links them, so
# create_keystore finds the full set and keeps the lab CA; step 3 still
# checks, and step 3b checks that the governance file was signed by it.
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
#      keystore/private/ca.key.pem (umask 077: the key is owner-only), and
#      links identity_ca.{cert,key}.pem and permissions_ca.{cert,key}.pem to
#      them (relative links, the layout create_keystore itself writes).
#   2. `ros2 security create_keystore keystore`. With all four CA files
#      present it keeps them and only writes and signs the governance file
#      (see WHAT THAT FIRST RUN FOUND).
#   3. Compares the SHA-256 fingerprints of identity_ca.cert.pem and
#      permissions_ca.cert.pem with the lab CA's. A difference means sros2
#      made its own CA: the script stops, and the keystore must be rebuilt.
#      3b. `openssl smime -verify` of enclaves/governance.p7s against the lab
#      CA: the DDS security plugins reject a governance file the permissions
#      CA did not sign.
#   4. `ros2 security create_enclave` for each enclave in
#      sitl/sros2_enclaves.txt (the processes of
#      sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py and
#      sitl/run_ros2_demo.sh): the Shield node, the MAVLink adapter, the VLA
#      stub, MAVROS and the rail's bag recorder.
#   5. `openssl verify -CAfile <pki>/ca.cert.pem` on every enclave
#      certificate.
#   6. Prints how to run the rail with it (SROS2=1 bash sitl/run_ros2_demo.sh)
#      and how to check it (bash tools/sros2_check.sh).
#
# WHAT IT DOES NOT DO
#
#   * Write permissions narrower than sros2's default. create_enclave grants
#     the enclave every topic; topic-level permissions (the Shield may
#     publish /shield/setpoint, the VLA only /vla/action_4d, ...) are a policy
#     XML per enclave, still to be written.
#   * Start any node with its enclave: sitl/run_ros2_demo.sh does that under
#     SROS2=1 (sitl/sros2_env.sh; the launch file's enclave_root:= gives each
#     of its three nodes its own ROS_SECURITY_ENCLAVE_OVERRIDE).
#   * Copy the CA key anywhere else. It seeds this keystore on this host; the
#     Orin and the drone get enclave keys, never the CA key.
#   * Pin the DDS domain. create_keystore signs a governance file for
#     ROS_DOMAIN_ID as set when it runs (0 when unset), and each enclave's
#     permissions name that domain: run the rail on the same ROS_DOMAIN_ID.
set -euo pipefail

PKI="${HARNESS_PKI:-}"
KEYSTORE="${HOME}/sros2_keystore"
DRY=0
FORCE=0
ALLOW_MNT=0
# The rail's ROS 2 processes, as enclave paths: one list for this script,
# sitl/sros2_env.sh and the tests.
ENCLAVE_LIST="$(cd "$(dirname "$0")/.." && pwd)/sitl/sros2_enclaves.txt"
[ -f "$ENCLAVE_LIST" ] || { echo "sros2_setup: no enclave list at $ENCLAVE_LIST" >&2; exit 1; }
mapfile -t ENCLAVES < <(grep -v -e '^[[:space:]]*#' -e '^[[:space:]]*$' "$ENCLAVE_LIST")
[ "${#ENCLAVES[@]}" -gt 0 ] || { echo "sros2_setup: $ENCLAVE_LIST names no enclave" >&2; exit 1; }

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

# 1. seed the keystore with the lab CA, in BOTH roles: create_keystore keeps a
# CA only when identity_ca.* and permissions_ca.* are all there (sros2 0.13.6
# _keystore.create_keystore: "Create new CA if one doesn't already exist").
umask 077
run mkdir -p "$KEYSTORE/public" "$KEYSTORE/private"
run cp "$CA_CERT" "$KEYSTORE/public/ca.cert.pem"
run cp "$CA_KEY" "$KEYSTORE/private/ca.key.pem"
run chmod 600 "$KEYSTORE/private/ca.key.pem"
for which in identity_ca permissions_ca; do
  run ln -sfn ca.cert.pem "$KEYSTORE/public/$which.cert.pem"
  run ln -sfn ca.key.pem "$KEYSTORE/private/$which.key.pem"
done

# 2. the keystore around it (create_keystore refuses a complete keystore, so a
# rerun on one this script built keeps it; step 3 still checks its CA)
if [ "$DRY" -eq 0 ] && [ -f "$KEYSTORE/enclaves/governance.p7s" ]; then
  echo "  keystore $KEYSTORE exists (governance.p7s present): kept, not recreated"
else
  run ros2 security create_keystore "$KEYSTORE"
fi

# 3. did sros2 keep the lab CA?
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
# 3b. the governance file is signed by it
if [ "$DRY" -eq 1 ]; then
  echo "+ check: openssl smime -verify enclaves/governance.p7s against $CA_CERT"
else
  openssl smime -verify -in "$KEYSTORE/enclaves/governance.p7s" -CAfile "$CA_CERT" \
    -out /dev/null 2>/dev/null \
    || die "enclaves/governance.p7s does not verify against the lab CA"
  echo "  governance.p7s signed by the lab CA"
fi

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

Check it live (two probe nodes in the Shield's and the adapter's enclaves, a
process with no enclave refused, an unsecured one deaf, the launch file's
per-node enclaves):

  bash tools/sros2_check.sh --keystore $KEYSTORE --pki $PKI

Fly the ROS 2 rail with it (every ROS 2 process in its own enclave, Enforce):

  SROS2=1 SROS2_KEYSTORE=$KEYSTORE bash sitl/run_ros2_demo.sh on

By hand, in every shell that starts a node:

  export ROS_SECURITY_KEYSTORE=$KEYSTORE ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce
  ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/<process> <command>
EOF
