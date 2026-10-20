# SROS 2 for the ROS 2 rail, behind SROS2=1. Sourced by sitl/run_ros2_demo.sh;
# needs no ROS itself, so tests/test_ros2_package.py runs it with plain bash.
#
#   SROS2=1 [SROS2_KEYSTORE=~/sros2_keystore] bash sitl/run_ros2_demo.sh on ...
#
# WHAT THE GRANT ASKS FOR
#
# The reference design's data-flow page: "SROS 2 turned on for the ROS 2 graph
# using the same root CA" as the gRPC control plane's mTLS; Stress Testing,
# "Locked design choices": "Same root CA as SROS 2 for cert reuse". Until
# 2026-10-19 the rail ran unsecured and tools/sros2_setup.sh had never been run
# (docs/DESIGN-harness-grpc.md, audit card X-15). docs/DESIGN-sros2.md records
# what was run on 2026-10-19 and what it showed.
#
# WHAT SROS2=1 DOES
#
#   1. sros2_prepare: refuses (exit 2, before SITL starts) unless the keystore
#      holds the signed governance file and, for every enclave in
#      sitl/sros2_enclaves.txt, its certificate, key, signed permissions and
#      the two CA certificates. A missing file is named; nothing is guessed.
#      Then it exports ROS_SECURITY_KEYSTORE, ROS_SECURITY_ENABLE=true and
#      ROS_SECURITY_STRATEGY=Enforce: under Enforce a process with no enclave,
#      or one the keystore's CA did not sign, cannot join the graph at all
#      (checked live by tools/sros2_check.sh).
#   2. sros2_prefix ARRAY NAME: sets the bash array ARRAY to the prefix that
#      runs one process in its own enclave, `env
#      ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/NAME`, and to nothing when SROS
#      2 is off - so with SROS2=0 every command line of the rail is exactly
#      what it was. NAME must be in the enclave list.
#   3. sros2_launch_args ARRAY: `enclave_root:=/guardrail` for the launch file
#      (one enclave per node), nothing when off.
#
# SROS2=0 (the default) does nothing at all, and any other value is refused.

SROS2_ENCLAVE_ROOT=/guardrail
SROS2_ENCLAVES_FILE="${SROS2_ENCLAVES_FILE:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sros2_enclaves.txt}"
# What sros2's create_enclave writes into each enclave folder (Jazzy, sros2
# 0.13.6, read 2026-10-19), less permissions.xml (the unsigned source of
# permissions.p7s, not read by the DDS security plugins).
SROS2_ENCLAVE_FILES=(cert.pem key.pem permissions.p7s governance.p7s
                     identity_ca.cert.pem permissions_ca.cert.pem)

sros2_enclaves() {
  grep -v -e '^[[:space:]]*#' -e '^[[:space:]]*$' "$SROS2_ENCLAVES_FILE"
}

sros2_prepare() {
  case "${SROS2:-0}" in
    0) return 0 ;;
    1) ;;
    *) echo "[sros2] REFUSED: SROS2 must be 0 or 1, not '${SROS2}'" >&2; return 2 ;;
  esac
  local ks="${SROS2_KEYSTORE:-$HOME/sros2_keystore}" enc f n=0
  local missing=()
  if [ ! -f "$SROS2_ENCLAVES_FILE" ]; then
    echo "[sros2] REFUSED: no enclave list at $SROS2_ENCLAVES_FILE" >&2
    return 2
  fi
  [ -f "$ks/enclaves/governance.p7s" ] || missing+=("enclaves/governance.p7s")
  while IFS= read -r enc; do
    n=$((n + 1))
    for f in "${SROS2_ENCLAVE_FILES[@]}"; do
      [ -e "$ks/enclaves$enc/$f" ] || missing+=("enclaves$enc/$f")
    done
  done < <(sros2_enclaves)
  if [ "$n" -eq 0 ]; then
    echo "[sros2] REFUSED: $SROS2_ENCLAVES_FILE names no enclave" >&2
    return 2
  fi
  if [ "${#missing[@]}" -gt 0 ]; then
    echo "[sros2] REFUSED: keystore $ks lacks ${#missing[@]} file(s), first:" \
         "${missing[*]:0:4}" >&2
    echo "        build it on this host: bash tools/sros2_setup.sh --pki <harness_pki> --keystore $ks" >&2
    return 2
  fi
  export ROS_SECURITY_KEYSTORE="$ks"
  export ROS_SECURITY_ENABLE=true
  export ROS_SECURITY_STRATEGY=Enforce
  echo "[sros2] enforced: keystore $ks, $n enclaves under $SROS2_ENCLAVE_ROOT (strategy Enforce)"
}

sros2_prefix() {
  local -n _out="$1"
  _out=()
  [ "${SROS2:-0}" = "1" ] || return 0
  if ! sros2_enclaves | grep -qx "$SROS2_ENCLAVE_ROOT/$2"; then
    echo "[sros2] no enclave $SROS2_ENCLAVE_ROOT/$2 in $SROS2_ENCLAVES_FILE" >&2
    return 2
  fi
  _out=(env "ROS_SECURITY_ENCLAVE_OVERRIDE=$SROS2_ENCLAVE_ROOT/$2")
}

sros2_launch_args() {
  local -n _args="$1"
  _args=()
  [ "${SROS2:-0}" = "1" ] && _args=("enclave_root:=$SROS2_ENCLAVE_ROOT")
  return 0
}
