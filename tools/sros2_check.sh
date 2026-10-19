#!/usr/bin/env bash
# Check, live, that SROS 2 is ENFORCED on the ROS 2 rail's graph.
#
#   bash tools/sros2_check.sh [--keystore DIR] [--pki DIR] [--python PY]
#
# Runs on the ROS 2 host (WSL Ubuntu, or the Orin's ROS 2 container) after
# tools/sros2_setup.sh built the keystore. No root, no SITL, no MAVROS, no
# flight: small rclpy probe nodes (written to a temporary folder) stand in for
# the rail's processes, with the interpreter the rail's nodes use
# (~/venv-ros/bin/python, or --python / $GUARDRAIL_PYTHON). Each check prints
# PASS or FAIL; the exit code is 0 only when every check passes.
#
# WHY SIX CHECKS. "Security enabled" is a set of environment variables; that
# they are set proves nothing about the graph. Two secured nodes talking
# shows the keystore works, not that it is enforced - an unsecured graph
# talks too. So the checks are mostly about who CANNOT join:
#
#   1 keystore   sitl/sros2_env.sh's own refusal check (governance and every
#                enclave of sitl/sros2_enclaves.txt present), the identity and
#                permissions CA = the lab CA (fingerprints, with --pki), the
#                governance file and every enclave certificate verified
#                against it with openssl;
#   2 secured    a publisher in /guardrail/safety_shield and a subscriber in
#                /guardrail/mavlink_adapter, both Enforce, on /shield/setpoint:
#                messages arrive;
#   3 enforced   the same subscriber with NO enclave under Enforce: refused at
#                start-up (it cannot even create its node);
#   4 sealed     the same subscriber with security OFF: it starts, and hears
#                nothing from the secured publisher;
#   5 one root   a subscriber whose keystore was made by sros2 with its own CA
#                (not the lab CA), same enclave name, Enforce: hears nothing -
#                the lab CA is the trust anchor, not the enclave name;
#   6 launch     sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py
#                run with enclave_root:=/guardrail and the probes in place of
#                the three node scripts (python:= points at a stand-in that
#                runs a probe per script): each node reports its own enclave
#                and the chain vla_stub -> safety_shield -> mavlink_adapter
#                carries messages under Enforce; the launch ends when the
#                Shield stand-in exits, as the rail does.
#
# The security variables are set inside this script only; the calling shell
# is not changed.
set -uo pipefail

KEYSTORE="${SROS2_KEYSTORE:-$HOME/sros2_keystore}"
PKI="${HARNESS_PKI:-}"
PY="${GUARDRAIL_PYTHON:-$HOME/venv-ros/bin/python}"
SECS=6

while [ $# -gt 0 ]; do
  case "$1" in
    --keystore) KEYSTORE="${2:?}"; shift 2 ;;
    --pki) PKI="${2:?}"; shift 2 ;;
    --python) PY="${2:?}"; shift 2 ;;
    -h|--help) sed -n '2,4p' "$0"; exit 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LAUNCH="$ROOT/sitl/ros2_ws/src/safety_shield/launch/guardrail_rail.launch.py"
if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
command -v ros2 >/dev/null || { echo "ros2 is not on the PATH" >&2; exit 2; }
[ -x "$PY" ] || { echo "no interpreter at $PY (--python)" >&2; exit 2; }

WORK="$(mktemp -d /tmp/sros2_check.XXXXXX)"
cleanup() {
  jobs -p | xargs -r kill 2>/dev/null
  rm -rf "$WORK"
}
trap cleanup EXIT
FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }

# The probe: one rclpy node. pub / sub / relay on Float32MultiArray (the
# rail's message type on /vla/action_4d and /shield/setpoint).
cat > "$WORK/probe.py" <<'PY'
import os, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

role, name, topic, secs = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
out_topic = sys.argv[5] if len(sys.argv) > 5 else None
got = sent = 0
rclpy.init()
node = Node(name)
pub = None
if role in ("pub", "relay"):
    pub = node.create_publisher(Float32MultiArray, out_topic or topic, 10)
if role in ("sub", "relay"):
    def cb(msg):
        global got, sent
        got += 1
        if pub is not None:
            pub.publish(msg)
            sent += 1
    node.create_subscription(Float32MultiArray, topic, cb, 10)
try:
    end = time.time() + secs
    while time.time() < end:
        if role == "pub":
            pub.publish(Float32MultiArray(data=[1.0, 0.0, 0.0, 0.0]))
            sent += 1
        rclpy.spin_once(node, timeout_sec=0.1)
except BaseException:          # SIGINT from launch's shutdown, or the context gone
    pass
print(f"PROBE role={role} node={name} "
      f"enclave={os.environ.get('ROS_SECURITY_ENCLAVE_OVERRIDE', '-')} "
      f"received={got} sent={sent}", flush=True)
try:
    node.destroy_node()
    rclpy.try_shutdown()
except BaseException:
    pass
PY

probe() {  # probe ROLE NAME TOPIC SECS [OUT] -> output on stdout
  timeout $((${4%.*} + 20)) "$PY" "$WORK/probe.py" "$@" 2>&1
}
received() { sed -n 's/.*received=\([0-9]*\).*/\1/p' | tail -1; }

secured_env() {  # the variables sitl/sros2_env.sh exports under SROS2=1
  export ROS_SECURITY_KEYSTORE="$KEYSTORE" ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce
}

echo "keystore $KEYSTORE, interpreter $PY, ROS $ROS_DISTRO, RMW ${RMW_IMPLEMENTATION:-default}"

# 1 keystore -----------------------------------------------------------------
# shellcheck disable=SC1091
source "$ROOT/sitl/sros2_env.sh"
if out="$(SROS2=1 SROS2_KEYSTORE="$KEYSTORE" sros2_prepare 2>&1)"; then
  ok=1
  CA="$KEYSTORE/public/ca.cert.pem"
  if [ -n "$PKI" ]; then
    want="$(openssl x509 -noout -fingerprint -sha256 -in "$PKI/ca.cert.pem" | sed 's/^.*=//')"
    for which in identity_ca permissions_ca; do
      got="$(openssl x509 -noout -fingerprint -sha256 -in "$KEYSTORE/public/$which.cert.pem" | sed 's/^.*=//')"
      [ "$got" = "$want" ] || { ok=0; echo "      $which is $got, lab CA $want"; }
    done
    CA="$PKI/ca.cert.pem"
  fi
  openssl smime -verify -in "$KEYSTORE/enclaves/governance.p7s" -CAfile "$CA" -out /dev/null 2>/dev/null \
    || { ok=0; echo "      governance.p7s does not verify against $CA"; }
  n=0
  while IFS= read -r enc; do
    n=$((n + 1))
    openssl verify -CAfile "$CA" "$KEYSTORE/enclaves$enc/cert.pem" >/dev/null 2>&1 \
      || { ok=0; echo "      $enc/cert.pem does not chain to $CA"; }
    openssl smime -verify -in "$KEYSTORE/enclaves$enc/permissions.p7s" -CAfile "$CA" -out /dev/null 2>/dev/null \
      || { ok=0; echo "      $enc/permissions.p7s does not verify against $CA"; }
  done < <(sros2_enclaves)
  if [ "$ok" = 1 ]; then
    pass "1 keystore: $n enclaves, governance and permissions signed by and certificates chaining to $( [ -n "$PKI" ] && echo "the lab CA" || echo "the keystore CA (no --pki: lab fingerprint not compared)")"
  else
    fail "1 keystore"
  fi
else
  fail "1 keystore: $out"
fi

# 2 secured ------------------------------------------------------------------
secured_env
ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/mavlink_adapter probe sub mavlink_adapter /shield/setpoint "$SECS" > "$WORK/sub2.txt" &
SUB=$!
sleep 1
ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/safety_shield probe pub safety_shield /shield/setpoint $((SECS + 2)) > "$WORK/pub2.txt" &
PUB=$!
wait "$SUB" 2>/dev/null
n2="$(received < "$WORK/sub2.txt")"
if [ "${n2:-0}" -gt 0 ]; then
  pass "2 secured: /guardrail/safety_shield -> /guardrail/mavlink_adapter, $n2 messages under Enforce"
else
  fail "2 secured: the adapter's enclave heard nothing from the Shield's: $(tail -3 "$WORK/sub2.txt" | tr '\n' ' ')"
fi
wait "$PUB" 2>/dev/null

# 3 enforced -----------------------------------------------------------------
( unset ROS_SECURITY_ENCLAVE_OVERRIDE; probe sub no_enclave /shield/setpoint 2 ) > "$WORK/sub3.txt"
rc3=$?
if [ "$rc3" -ne 0 ] && ! grep -q '^PROBE' "$WORK/sub3.txt"; then
  pass "3 enforced: a process with no enclave was refused (exit $rc3: $(grep -m1 -E 'RCLError|security files' "$WORK/sub3.txt" | sed 's/^ *//' | cut -c1-120))"
else
  fail "3 enforced: a process with no enclave started (exit $rc3): $(tail -2 "$WORK/sub3.txt" | tr '\n' ' ')"
fi

# 4 sealed -------------------------------------------------------------------
# With a secured subscriber listening in the same window (the control: the
# publisher WAS heard by someone while the outsider heard nothing).
ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/mavlink_adapter probe sub mavlink_adapter /shield/setpoint "$SECS" > "$WORK/ctl4.txt" &
CTL=$!
ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/safety_shield probe pub safety_shield /shield/setpoint $((SECS + 2)) > "$WORK/pub4.txt" &
PUB=$!
sleep 1
( unset ROS_SECURITY_ENABLE ROS_SECURITY_STRATEGY ROS_SECURITY_KEYSTORE ROS_SECURITY_ENCLAVE_OVERRIDE
  probe sub outsider /shield/setpoint "$SECS" ) > "$WORK/sub4.txt"
wait "$CTL" 2>/dev/null
n4="$(received < "$WORK/sub4.txt")"
c4="$(received < "$WORK/ctl4.txt")"
if grep -q '^PROBE' "$WORK/sub4.txt" && [ "${n4:-1}" -eq 0 ] && [ "${c4:-0}" -gt 0 ]; then
  pass "4 sealed: an unsecured process started and heard 0 messages; the secured control heard $c4"
else
  fail "4 sealed: unsecured heard ${n4:-?}, secured control ${c4:-?}: $(tail -2 "$WORK/sub4.txt" | tr '\n' ' ')"
fi
wait "$PUB" 2>/dev/null

# 5 one root -----------------------------------------------------------------
( unset ROS_SECURITY_KEYSTORE ROS_SECURITY_ENABLE ROS_SECURITY_STRATEGY
  ros2 security create_keystore "$WORK/foreign" >/dev/null 2>&1 &&
  ros2 security create_enclave "$WORK/foreign" /guardrail/mavlink_adapter >/dev/null 2>&1 )
ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/mavlink_adapter probe sub mavlink_adapter /shield/setpoint "$SECS" > "$WORK/ctl5.txt" &
CTL=$!
ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/safety_shield probe pub safety_shield /shield/setpoint $((SECS + 2)) > "$WORK/pub5.txt" &
PUB=$!
sleep 1
ROS_SECURITY_KEYSTORE="$WORK/foreign" ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/mavlink_adapter \
  probe sub mavlink_adapter /shield/setpoint "$SECS" > "$WORK/sub5.txt"
wait "$CTL" 2>/dev/null
n5="$(received < "$WORK/sub5.txt")"
c5="$(received < "$WORK/ctl5.txt")"
if [ -f "$WORK/foreign/enclaves/guardrail/mavlink_adapter/cert.pem" ] && [ "${n5:-0}" -eq 0 ] \
    && [ "${c5:-0}" -gt 0 ]; then
  pass "5 one root: the same enclave name under another CA heard 0 messages$(grep -q '^PROBE' "$WORK/sub5.txt" || echo ' (refused at start-up)'); the lab-CA control heard $c5"
else
  fail "5 one root: another CA heard ${n5:-?}, lab-CA control ${c5:-?}: $(tail -2 "$WORK/sub5.txt" | tr '\n' ' ')"
fi
wait "$PUB" 2>/dev/null

# 6 launch -------------------------------------------------------------------
cat > "$WORK/stand_in_python" <<SH
#!/usr/bin/env bash
# Stands in for the rail's interpreter: one probe per node script.
case "\$(basename "\$1")" in
  ros2_vla_stub_node.py)   exec "$PY" "$WORK/probe.py" pub vla_stub /vla/action_4d 30 ;;
  ros2_shield_node.py)     exec "$PY" "$WORK/probe.py" relay safety_shield /vla/action_4d $((SECS + 2)) /shield/setpoint ;;
  mavlink_adapter_node.py) exec "$PY" "$WORK/probe.py" sub mavlink_adapter /shield/setpoint 30 ;;
  *) echo "stand-in: unexpected script \$1" >&2; exit 9 ;;
esac
SH
chmod +x "$WORK/stand_in_python"
# `ros2 launch` itself runs with no enclave, as sitl/run_ros2_demo.sh starts
# it (sitl/sros2_enclaves.txt says why).
( unset ROS_SECURITY_ENCLAVE_OVERRIDE
  timeout 90 ros2 launch "$LAUNCH" python:="$WORK/stand_in_python" repo_root:="$ROOT" \
    enclave_root:=/guardrail ) > "$WORK/launch.txt" 2>&1
rc6=$?
ok=1
for node in vla_stub safety_shield mavlink_adapter; do
  line="$(grep -o "PROBE role=[a-z]* node=$node enclave=[^ ]* received=[0-9]* sent=[0-9]*" "$WORK/launch.txt" | tail -1)"
  echo "      ${line:-no report from $node}"
  case "$line" in
    *"enclave=/guardrail/$node "*) ;;
    *) ok=0 ;;
  esac
done
nsh="$(grep -o 'node=safety_shield enclave=[^ ]* received=[0-9]*' "$WORK/launch.txt" | sed 's/.*received=//' | tail -1)"
nad="$(grep -o 'node=mavlink_adapter enclave=[^ ]* received=[0-9]*' "$WORK/launch.txt" | sed 's/.*received=//' | tail -1)"
if [ "$ok" = 1 ] && [ "${nsh:-0}" -gt 0 ] && [ "${nad:-0}" -gt 0 ] && [ "$rc6" -ne 124 ]; then
  pass "6 launch: three nodes in their own enclaves, vla_stub -> safety_shield ($nsh) -> mavlink_adapter ($nad) under Enforce; the launch ended with the Shield (exit $rc6)"
else
  fail "6 launch (exit $rc6): $(grep -i -m3 -E 'error|security' "$WORK/launch.txt" | cut -c1-160 | tr '\n' ' ')"
fi

echo
if [ "$FAILS" -eq 0 ]; then
  echo "SROS 2 enforced on the rail's graph: 6/6 checks passed"
else
  echo "$FAILS of 6 checks FAILED"
fi
[ "$FAILS" -eq 0 ]
