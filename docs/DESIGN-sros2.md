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
**Update 2026-11-03:** per-enclave, per-topic permissions (`sitl/sros2_policy.xml`,
section "Per-topic permissions" below) replace sros2's allow-all default; the check
has eight checks (8/8 in WSL); flown with `SROS2=1` (first on 2026-11-02 with the
allow-all permissions, again on 2026-11-03 with the policy's).

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

**Changed on 2026-11-03:** sros2's default permissions. `create_enclave` grants
each enclave every topic of the domain; since this date `tools/sros2_setup.sh` replaces
them with the policy's (next section). The governance and permissions are signed for
the `ROS_DOMAIN_ID` in force when the keystore was made (0 when unset): fly on the
same domain.

## Per-topic permissions (2026-11-03)

Until this date any process of the rail could publish any topic: Enforce kept out a
process with no enclave or another CA, but the VLA's enclave could publish
`/shield/setpoint` and fly around the Shield. `sitl/sros2_policy.xml` is an sros2
access-control policy with one entry per enclave of `sitl/sros2_enclaves.txt`, each
holding a profile per node that runs in it, with exactly the interfaces the node
creates:

| Enclave | Publishes | Notes |
|---|---|---|
| `/guardrail/safety_shield` | `/shield/setpoint`, `/shield/mode_request`, `/shield/fences`, `/shield/intercept`; for the mission-style node also `/shield/mission`, `/shield/mission_update` | the only publisher of `/shield/setpoint`; calls only the MAVROS services it uses (set_mode, arming, takeoff, stream rate, the parameter get/set, vehicle_info_get) |
| `/guardrail/vla_stub` | `/vla/action_4d`, `/vla/identity` | the only publisher of `/vla/action_4d`; the stub and the AerialVLA bridge share it |
| `/guardrail/mavlink_adapter` | `/mavros/setpoint_raw/local`, `/mavlink_adapter/status`, `/mavlink_adapter/sent` (2026-11-17, the Shield's delivery monitor subscribes) | calls set_mode, geofence push / pull, mission push / pull |
| `/guardrail/mavros` | `/mavros/*`, `/uas1/*`, `/tf`, `/tf_static`, `/diagnostics` | wildcards over MAVROS's own namespaces (and its services `/mavros/*`, `/mavros_node/*`, `/mavros_router/*`) |
| `/guardrail/bag_recorder` | `/events/write_split` | subscribes the recorded topics and `/parameter_events` (an rclcpp node's time source subscribes to it: without it the recorder died at start on the first flight, `sros2_policy_on_20261103`) |

Every node also publishes `/rosout` and `/parameter_events` and serves its own
parameter and type-description services (`~/*`). sros2 turns each enclave's profiles
into DDS permissions (`rt/<topic>`, `rq/<service>Request`, `rr/<service>Reply`;
anything not granted is DENY). `bash tools/sros2_setup.sh --pki <lab PKI>
--permissions-only` re-signs an existing keystore from the policy with the keystore's
permissions CA (the lab CA): `ros2 security create_permission` per enclave, each
signed file then verified against the lab CA and refused if it still grants `rt/*`. A
full setup (no `--permissions-only`) does the same after creating the enclaves.

Run on 2026-11-03 against the keystore the 2 November flight used (backed up first as
`~/sros2_keystore.before-policy-20261102T201841Z.tar.gz` in WSL), then
`tools/sros2_check.sh --pki ~/harness_pki`:

```
PASS  1 keystore: 5 enclaves, governance and permissions signed by and certificates chaining to the lab CA
PASS  2 secured: /guardrail/safety_shield -> /guardrail/mavlink_adapter, 48 messages under Enforce
PASS  3 enforced: a process with no enclave was refused (exit 1: rclpy._rclpy_pybind11.RCLError: error creating node: couldn't find all security files!, ...)
PASS  4 sealed: an unsecured process started and heard 0 messages; the secured control heard 58
PASS  5 one root: the same enclave name under another CA heard 0 messages; the lab-CA control heard 56
PASS  6 launch: three nodes in their own enclaves, vla_stub -> safety_shield (77) -> mavlink_adapter (77) under Enforce; the launch ended with the Shield (exit 0)
PASS  7 refused: /guardrail/vla_stub publishing /shield/setpoint (exit 1) and /guardrail/safety_shield publishing /vla/action_4d (exit 1) were refused at the publisher, their nodes up: "rt/shield/setpoint topic not found in allow rule", "rt/vla/action_4d topic not found in allow rule"; the adapter heard 0 on /shield/setpoint
PASS  8 policy: blanket=- shield_setpoint=['/guardrail/safety_shield'] vla_action_4d=['/guardrail/vla_stub'] enclaves=5

SROS 2 enforced on the rail's graph: 8/8 checks passed
```

Check 7 is the refusal live: the probe's node joins the graph (it prints `NODE-UP`),
then creating the forbidden publisher fails in Fast DDS's access control
(`[SECURITY Error] rt/shield/setpoint topic not found in allow rule ... ->
check_create_datawriter`, then rclpy's `RCLError: Failed to create publisher`), while
the same probe in the Shield's enclave publishes `/shield/setpoint` and is heard (check
2). Check 8 reads the signed files themselves: no enclave keeps `rt/*`, and matching
the publish grants with DDS's own fnmatch rule, only the Shield's enclave may publish
`/shield/setpoint` and only the VLA slot's `/vla/action_4d`. A flown run records each
process's `permissions_scope` (`per_topic` or `allow_all`) and the SHA-256 of its
`permissions.p7s` in `hil_evidence.ros2_security` (guardrail.manifest).

A node that adds a publisher, subscription or service client must be added to the
policy and the keystore re-signed, or Enforce refuses it when it creates the
interface.

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

- **A flight** (as of 2026-10-19; flown since, on 2026-11-02 with the allow-all
  permissions and on 2026-11-03 with the per-topic ones: CHANGELOG).
- **The Orin.** The keystore is per host; the Orin's ROS 2 container needs enclaves
  signed by the lab CA. The CA key stays on the desktop: create the Orin's enclaves in
  the desktop keystore and copy only those enclave folders (with the public CA
  certificates and the signed governance file) to the Orin.
- **The Project AirSim rail** (`pas` env) has no ROS 2 graph; nothing to secure.
