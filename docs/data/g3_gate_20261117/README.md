# Gate G3 on the AGX Orin: evidence assembled 17 Nov 2026

Row ARCH-03, "Real VLA backends produce the 4-D action at 10 Hz (and Orin latency gate G3)".

The reference design's gate (docs/04-risks-and-fallbacks/fallback-gates.md, "Gate G3
candidate - Orin VLA inference latency"): if the chosen 7B-class VLA backend cannot produce
4-D actions at 10 Hz on the Orin in int8, "if Orin inference latency > 100 ms p95 by
2026-08-01, swap to a smaller / quantized backend (e.g. BitVLA, TinyVLA)".

No new flight or measurement was made for this folder. Every number comes from files
already on disk: the hil flights' `vla_steps.jsonl` and `metrics.json` (15-17 Nov 2026) and
the Orin benches the backend table cites (`deploy/evidence/vla_backend_table.json`, its
`g3` section).

## Reproduce

    python tools/g3_gate.py --runs F:/vla_drone_offload/jetson_runs --write docs/data/g3_gate_20261117
    python tools/g3_gate.py --check docs/data/g3_gate_20261117/g3_gate.json

The first reads the run folders (they are not in git; on the lab desktop they are on F:
under `vla_drone_offload/jetson_runs`, linked from `demo/out`). The second re-judges the
committed `g3_gate.json` without them. Both exit 0 when a learned backend passes the gate
and 1 when none does. `g3_gate.md` is the table the first command printed.

## What is measured, and on which host

- **In flight (the gate's measure here).** The C2 flights of the backend table (the
  Project AirSim follow on hil, 420 s limit, KPI-grade; three per backend). On every one,
  the perception node, the Shield and the VLA server ran on the AGX Orin: each run's
  `metrics.json` `hil_evidence.vla_host.device_model` reads "NVIDIA Jetson AGX Orin
  Developer Kit" (L4T R39.2.1). ArduPilot SITL 4.5.7 and Project AirSim ran on the desktop.
  The tool reads only `hil_evidence.vla_host.device_model` (never the Shield host's
  `device_model`): a flight whose VLA host is null or absent is "VLA host not shown", one
  whose VLA host is not an AGX Orin (an Orin Nano included) is refused, and a flight with any
  failed request (`ok` false or no `rtt_s`) fails. All twelve flights here have 0 failed
  requests.
  - *Latency* is the round trip of one request (`rtt_s`: the node sends the frame, the
    server answers; both on the Orin). Beside it: the server's own model time
    (`latency_server_s`) and the time from the frame reaching the node to the parsed answer
    (`latency_client_s`).
  - *Rate* is answers parsed per second while the model was being asked (the backend table's
    `decision_hz_asked`: not held by the rail, not withheld because the detector had no box).
    The rate over the whole mission is shown beside it.
  - *Configuration the rate depends on.* AIoT-VLA's 13.15-13.22 decisions/s while asked holds
    with the stream feed (`--vla-feed stream`, no cap) and a 0.05 s FrontCamera capture
    interval (`-FrontCaptureIntervalS 0.05`), the C2 configuration the backend table's C2
    filter selects. `deploy/orin/run_pas_perception.sh` makes stream the AIoT-VLA default, but
    `perception_node.py`'s own default is `--vla-feed tick`; with the tick feed the same
    backend decided 6.3-9.6 times a second while asked (pas_hil_aiot_kpi_01, est_01, para_*,
    obs_02 in the backend table). pas_hil_aiot_c2_01 itself records
    `vla_observation.below_grant.per_wall_s = true` (7.5 Hz over the wall clock: the flight
    that lost the car, below).
- **Open loop on the AGX Orin (shown, not judged).** R1 and the servers' benches from the
  backend table. The R1 benches record the power mode: `vla_rate_openvla_7b_bf16.json` (the
  969 ms p95 below), the other `vla_rate_*.json` files and `host_facts.json` hold nvpmodel
  "MAXN" (`/var/lib/nvpmodel/status`); `g3_gate.json` carries it as `power_mode` on each
  bench entry. The servers' benches (`*_bench_agx.json`, `vla_bf16_check.json`) and the hil
  run folders do not record the power mode or jetson_clocks.

A backend passes when every one of at least 3 C2 flights has a round-trip p95 <= 100 ms and
>= 10 decisions/s while asked.

## Result

| Backend | Class | In flight on the AGX Orin: round trip p50 / p95 / p99 ms (pooled) | Decisions/s while asked | Open loop on the AGX Orin, p95 | G3 |
|---|---|---|---|---|---|
| OpenVLA-7B bf16 | 7B | not flown on hil (base OpenVLA is given no target) | - | 969 ms (R1, n=50, MAXN): about 1.05 Hz | fails |
| OpenVLA-7B int8 weight-only | 7B | - | - | no timing: the 16 Nov attempt was refused at import (torchao not installed in the AGX Orin's environment) | not measured |
| OpenVLA-7B 4-bit, AerialVLA 4-bit, AerialVLA + our fine-tune 4-bit | 7B | - | - | no timing: bitsandbytes is not installed in the AGX Orin's environment | not measured |
| AerialVLA LoRA, bf16 | 7B | 1374.5 / 1608.5 / 2385.0 (61 answers) | 0.42-0.71 | 1100 ms merged, 1538 ms unmerged (server bench, n=50) | fails |
| RaceVLA drone6_all, bf16 | 7B | 1375.5 / 2156.9 / 2172.5 (7 answers) | 0.57-0.62 | 843 ms (server bench, n=50) | fails |
| DroneCATS-Cosmos3-Edge-2B, bf16 | smaller | 2475.3 / 2957.7 / 18064.7 (61 answers) | 0.34-0.58 | 17529 ms (server bench, n=50) | fails |
| **AIoT-VLA v10b** (the lab's own: frozen OWL-ViT-B/32 + a trained head, fp16) | smaller | **51.7 / 84.5 / 103.8** (10716 answers) | **13.15-13.22** | 41.7 ms (own inputs, n=300) | **passes** |

AIoT-VLA v10b per flight (round trip p50 / p95 / p99 ms; decisions/s asked; decisions/s
over the mission):

| Flight | Date | Answers | Round trip | Server (model) | Frame at node to answer | Asked | Mission |
|---|---|---|---|---|---|---|---|
| pas_hil_feed_07 | 15 Nov | 4648 | 51.9 / 84.7 / 104.2 | 39.7 / 71.4 / 90.6 | 86.7 / 134.6 / 156.4 | 13.15 | 10.93 |
| pas_hil_feed_08 | 15 Nov | 4500 | 51.6 / 84.2 / 104.5 | 39.4 / 70.7 / 90.1 | 82.5 / 135.2 / 162.6 | 13.22 | 10.62 |
| pas_hil_aiot_c2_01_20261116 | 16 Nov | 1568 | 51.7 / 84.0 / 98.1 | 39.7 / 70.7 / 83.2 | 89.7 / 134.1 / 154.0 | 13.21 | 7.45 |

The 7.45 over the mission in pas_hil_aiot_c2_01 is the flight that lost the car: the model
is not asked while the detector has no box, and the mission ran 90 s more before it ended on
`target_lost`. While asked, its rate is the same as the other two.

## What this says

1. **The 7B backends fail the gate as measured.** On the AGX Orin, OpenVLA-7B bf16 answers in
   969 ms p95 open loop; in flight, AerialVLA bf16 and RaceVLA bf16 answer in 1.4-2.4 s and
   decide 0.4-0.7 times a second while asked. That is 10-20 times too slow for the gate's
   100 ms and 10 Hz.
2. **No int8 figure for a 7B model exists on the Orin.** The gate names int8. The int8
   weight-only attempt (16 Nov) was refused at import, and the 4-bit loads need
   bitsandbytes, which the AGX Orin's environment does not install. The only int8 figure is
   one layer on the Orin Nano (CHANGELOG 2026-10-28). This folder claims no int8 7B number.
3. **The gate's prescribed action is the swap to a smaller backend.** The grant's examples,
   BitVLA and TinyVLA, have no timing here (BitVLA's weights were not obtained; TinyVLA is not
   in the backend set). The smaller backend the lab built and named for the follow task on 15 Nov 2026 is
   AIoT-VLA v10b. **It passes the gate on the AGX Orin in flight**: round-trip p95 84.0-84.7 ms
   on each of its three C2 flights (84.5 ms pooled over 10716 answers), and 13.15-13.22
   decisions/s while asked (stream feed, 0.05 s capture; see above). Its model time alone is
   70.7-71.4 ms p95. Another smaller
   backend, DroneCATS 2B, does not pass (2.7-3.1 s p95).
4. **The 4-D action at 10 Hz.** AIoT-VLA answers with a step (forward, down, heading change,
   right). The rail's step tracker (guardrail/vla_link.py) turns the newest step into the
   4-D setpoint (vx, vy, vz, yaw_rate) on every 10 Hz Shield tick, and the Shield then filters
   it. With 13 answers a second while asked, a fresh answer arrives for every tick, on average.
   The 7B backends' answers are held by the same tracker for about 1-2 s each.
5. **The gate date.** The gate's date, 2026-08-01, had passed before these measurements
   (November). The gate is judged here as a rule, not as a date that was met.

## What is above 100 ms, and what would fix it

- **Round-trip p99** is 104.2 and 104.5 ms in two of the three flights. The gate is p95, so
  this does not change the verdict.
- **Frame at the node to answer** is 134-135 ms p95 (85.4 ms p50 pooled). This is the age of
  the observation when its action arrives. It is not the gate's inference latency, but it is
  above 100 ms. Before the request is sent, the frame waits 26-33 ms p50 and 65-67 ms p95
  (`t_sent - t_capture`, in `g3_gate.json` as `capture_to_send_ms`). The runs do not record
  why each frame waited. What would bring it under 100 ms (none of these has been tried for
  this row):
  - send the newest frame as soon as the previous answer returns, so a frame does not wait
    for a request that is still out;
  - shorten the model's 71 ms p95: build an ONNX/TensorRT fp16 engine for the OWL-ViT
    backbone, or use a smaller input;
  - fix the AGX Orin's clocks (MAXN power mode with jetson_clocks) and record the power mode
    in the run folder. The R1 benches record MAXN; no hil run folder records the power mode
    today.
- **Follow quality is a separate question.** These flights score 0.21-0.35 of ticks within
  30 m of the car, and each fails its KPI on `unsafe_position_entered` with 0 P0 escapes
  (backend table, C2). G3 is about latency and rate only.
