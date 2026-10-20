# SROS 2 on the ROS 2 rail - enforced behind `SROS2=1`, checked live

**Date:** 2026-10-19
**Status:** built and run in WSL Ubuntu 24.04 with ROS 2 Jazzy, no root needed.
`tools/sros2_setup.sh` builds a keystore whose identity and permissions CA is the
harness's lab root CA; `SROS2=1 bash sitl/run_ros2_demo.sh` runs every ROS 2 process
of the rail in its own enclave under `ROS_SECURITY_STRATEGY=Enforce`;
`tools/sros2_check.sh` proves it is enforced (6/6 on 2026-10-19, below). **Not yet
flown:** no SITL flight has run with `SROS2=1` (the lead flies; command at the end).
The default, `SROS2=0`, changes nothing.
**Card:** X-15 (docs/DESIGN-harness-grpc.md, whose "SROS 2 is not enabled" status
line was brought up to date on 2026-10-20 after the branch review).

## What the grant asks for

| Where | What it says |
|---|---|
| Reference design, `docs/03-simulation/data-flow.md`, Control plane | "gRPC over mTLS. Per-component certificates from `cfssl` or `step-ca` with a shared root CA; SROS 2 turned on for the ROS 2 graph using the same root CA." |
| Stress Testing page, Locked design choices | "Control plane: gRPC over mTLS. Same root CA as SROS 2 for cert reuse." |
| Recommendation, item 3 | "HIL topology DDS layout - ... SROS 2 + mTLS. Operationally important once the Shield is on the Orin in Q2." |

The harness half (gRPC mTLS from one lab root CA, `python -m guardrail.harness_service
gen-pki`) was built on 2026-10-12. The ROS 2 half was a script that had never run.

## What the WSL environment provides (read 2026-10-19, nothing installed)

| Item | Present |
|---|---|
| OS | Ubuntu 24.04.4 LTS (WSL 2), user `nathan`, no root used |
| ROS 2 | Jazzy (`/opt/ros/jazzy`), default RMW `rmw_fastrtps_cpp` |
| `ros-jazzy-sros2` | 0.13.6 (`ros2 security create_keystore / create_enclave / ...`) |
| Fast DDS | `ros-jazzy-fastrtps` 2.14.6, `libfastrtps.so` linked to `libssl.so.3` / `libcrypto.so.3` (the builtin security plugins) |
| OpenSSL | 3.0.13 |
| `~/venv-ros` (the rail's interpreter) | Python 3.12.3, rclpy, std_msgs, mavros_msgs (system site-packages), cryptography 41.0.7 |
| Harness PKI on Windows | none yet (`%LOCALAPPDATA%\vla-guardrail\harness_pki` absent): the run below used a throwaway PKI in `/tmp` |

Nothing needed root. Nothing was installed.

## What the first run found: the setup script made its own CA

`tools/sros2_setup.sh` as committed (c77c77d) copied the lab CA into the keystore as
`public/ca.cert.pem` + `private/ca.key.pem` and relied on `ros2 security
create_keystore` "reusing a CA pair it finds there". Run against a fresh lab PKI it
stopped at its own step 3:

```
sros2_setup: identity_ca is E1:4E:CA:EA:35:1A:DD:71:..., not the lab CA 61:17:CC:4E:13:AA:7D:C9:...:
sros2 made its own CA. Remove /tmp/t1d_sros2_before/keystore and rerun
```

and `public/ca.cert.pem` had become `subject=CN = sros2CA`. Reading sros2 0.13.6
(`sros2/keystore/_keystore.py`, `create_keystore`): it keeps a CA only when
`identity_ca.cert.pem`, `identity_ca.key.pem`, `permissions_ca.cert.pem` and
`permissions_ca.key.pem` are ALL present; otherwise it writes a new CA over
`ca.cert.pem` / `ca.key.pem` and links both roles to it. The script's check worked; its
procedure could not have. The fix links the four role files to the copied CA before
`create_keystore` (the same relative links sros2 writes itself), so it finds the set and
only writes and signs the governance file. Step 3b now also verifies `governance.p7s`
against the lab CA with `openssl smime -verify`, and a rerun on a keystore the script
already built keeps it (`create_keystore` refuses a complete keystore).

## The design

**One list of enclaves**, `sitl/sros2_enclaves.txt`: `/guardrail/safety_shield`,
`/guardrail/mavlink_adapter`, `/guardrail/vla_stub`, `/guardrail/mavros`,
`/guardrail/bag_recorder`. `tools/sros2_setup.sh` creates exactly these;
`sitl/sros2_env.sh` refuses a keystore missing any file of any of them;
`tests/test_ros2_package.py` checks that every process the launch file and the runner
start under `SROS2=1` has one here.

**No enclave for `ros2 launch` itself.** `guardrail_rail.launch.py` starts plain
processes and creates no ROS node of its own; under Enforce it ran with no enclave
(experiment b below). Giving it one would be worse than useless: a node started without
`enclave_root:=` inherits the launch's `ROS_SECURITY_ENCLAVE_OVERRIDE`, so all three
nodes would join as the launch's single identity (experiment c showed exactly that).
Without one, such a node is refused at start-up.

**One enclave per process.** The launch file's three nodes share its environment, so
the new `enclave_root:=/guardrail` argument gives each its own
`ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/<process name>` (`safety_shield`,
`mavlink_adapter`, `vla_stub`). MAVROS and `ros2 bag record` get theirs from an `env`
prefix in `sitl/run_ros2_demo.sh` (an override works for any rcl process; `ros2 bag
record` takes no `--ros-args`). With `SROS2=0` every prefix array is empty and the
launch gets no `enclave_root:=`, so every command line is the one it was.

**Refused before anything starts.** `sros2_prepare` runs after the policy check and
before the clean slate and SITL: a missing keystore file is named and the run exits 2.
Only then are `ROS_SECURITY_KEYSTORE`, `ROS_SECURITY_ENABLE=true` and
`ROS_SECURITY_STRATEGY=Enforce` exported.

**The run records it.** The Shield node puts `ros2_security` in `hil_evidence`
(`guardrail.manifest.ros2_security_evidence`) when `ROS_SECURITY_ENABLE` is `true`:
strategy, keystore, enclave, whether its six security files are present, and the SHA-256
fingerprints of the identity and permissions CA in OpenSSL's format, which a reader
compares with the fingerprint `gen-pki` printed for the lab CA. With security off
nothing is added, so a run without SROS 2 records what it always did.

**Not changed:** sros2's default permissions. `create_enclave` grants each enclave
every topic of the domain; per-enclave policies (the Shield alone publishes
`/shield/setpoint`, the VLA alone `/vla/action_4d`) are permissions XML still to be
written. The governance and permissions are signed for the `ROS_DOMAIN_ID` in force
when the keystore was made (0 when unset): fly on the same domain.

## The live check, run 2026-10-19 (`tools/sros2_check.sh`)

Throwaway lab PKI from `gen-pki` in `/tmp`, keystore built by the fixed
`tools/sros2_setup.sh`, probe nodes run by `~/venv-ros/bin/python` (the rail's
interpreter), Fast DDS (default RMW). 40 s.

```
PASS  1 keystore: 5 enclaves, governance and permissions signed by and certificates chaining to the lab CA
PASS  2 secured: /guardrail/safety_shield -> /guardrail/mavlink_adapter, 49 messages under Enforce
PASS  3 enforced: a process with no enclave was refused (exit 1: rclpy._rclpy_pybind11.RCLError: error creating node: couldn't find all security files!, ...)
PASS  4 sealed: an unsecured process started and heard 0 messages; the secured control heard 58
PASS  5 one root: the same enclave name under another CA heard 0 messages; the lab-CA control heard 56
      PROBE role=pub node=vla_stub enclave=/guardrail/vla_stub received=0 sent=81
      PROBE role=relay node=safety_shield enclave=/guardrail/safety_shield received=78 sent=78
      PROBE role=sub node=mavlink_adapter enclave=/guardrail/mavlink_adapter received=76 sent=0
PASS  6 launch: three nodes in their own enclaves, vla_stub -> safety_shield (78) -> mavlink_adapter (76) under Enforce; the launch ended with the Shield (exit 0)

SROS 2 enforced on the rail's graph: 6/6 checks passed
```

Checks 3 to 5 are the ones a disabled or permissive graph would fail: with security
off, check 3's process starts and check 4's outsider hears everything. Checks 4 and 5
each run a secured subscriber in the same window as the control, so "heard nothing" is
not "nobody was publishing".

**Tightened on 2026-10-20 (branch review).** As first written, check 3 passed on any
non-zero exit without a PROBE line, and check 5 read a missing `received=` count as 0,
so a probe that died for an unrelated reason (an import error, a missing module) read as
"refused by Enforce" or "heard 0". Both now count a start-up refusal only in the security
layer's words (`security files`, `RCLError ... security`, `SECURITY ERROR`), and check 5
otherwise needs a PROBE line saying `received=0`; the two verdicts are shell functions
(`refused_by_security`, `heard_nothing`) that `tests/test_ros2_package.py` runs with plain
bash on recorded outputs. Re-run in WSL the same day (throwaway PKI in `/tmp`, removed
afterwards): 6/6 again with the rail's interpreter (check 3 quoting "couldn't find all
security files!", check 5 a PROBE line with received=0). With a stand-in interpreter that
exits with `ModuleNotFoundError: No module named 'std_msgs'` for exactly check 3's and
check 5's processes, the tightened script FAILs both (4/6) where the 044f0fc script
PASSed both. Check 6 runs the real launch file with `python:=` pointing
at a stand-in that starts one probe per node script, so the launch file's per-node
enclaves and its shutdown-with-the-Shield are what was exercised.

Experiments behind the design (same keystore, Enforce):

| | What ran | Result |
|---|---|---|
| a | an rclpy node with no enclave | `RCLError: error creating node: couldn't find all security files!` |
| b | `ros2 launch` with no enclave of its own, `enclave_root:=/guardrail` | all three stand-ins start, each in `/guardrail/<name>` |
| c | `ros2 launch` with `ROS_SECURITY_ENCLAVE_OVERRIDE=/guardrail/launch`, no `enclave_root:=` | all three start as `/guardrail/launch` (one shared identity) |
| d | security off, no `enclave_root:=` (the rail as before) | all three start, no enclave |

## How to fly it (the lead)

On the desktop, once: the lab PKI (Windows, `vla-drone` env, outside any synced folder),
then the keystore and the check in WSL:

```
python -m guardrail.harness_service gen-pki
# WSL:
bash tools/sros2_setup.sh --pki /mnt/c/Users/<you>/AppData/Local/vla-guardrail/harness_pki --keystore ~/sros2_keystore
bash tools/sros2_check.sh --keystore ~/sros2_keystore --pki /mnt/c/Users/<you>/AppData/Local/vla-guardrail/harness_pki
SROS2=1 bash sitl/run_ros2_demo.sh on - --tag ros2_shield_on_sros2
```

(The `-` is the runner's second positional, "no --dynamic"; without it `--tag` would
be read as that argument.)

The flight's `metrics.json` then carries `hil_evidence.ros2_security` with
`strategy: Enforce`, `enclave: /guardrail/safety_shield`, `files_present: true` and the
lab CA's fingerprint. A rail that answers MAVROS (`fcu_connected` true) under Enforce
also shows MAVROS joined the secured graph, since an unsecured MAVROS cannot be heard
there (check 4).

## What this does not give you

- **A flight.** The SITL + MAVROS rail has not flown with `SROS2=1`; MAVROS under
  Enforce is untested here (it is one rclcpp process, so one override covers its nodes).
- **Topic-level permissions** (above).
- **The Orin.** The keystore is per host; the Orin's ROS 2 container needs enclaves
  signed by the lab CA. The CA key stays on the desktop: create the Orin's enclaves in
  the desktop keystore and copy only those enclave folders (with the public CA
  certificates and the signed governance file) to the Orin.
- **The Project AirSim rail** (`pas` env) has no ROS 2 graph; nothing to secure.
