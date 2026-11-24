# The grant's mid-term demo gate on the Gazebo functional rail, 25 Nov 2026 (ARCH-16)

Grant (kuanting-vla-uav-guardrail/docs/05-recommendation/recommendation.md,
"Mid-term (2026-07-20) demo gate"): on the functional rail (Gazebo Harmonic),
end to end, (1) load a signed policy bundle with one polygon NFZ and one
envelope rule, (2) issue a natural-language task that wants to enter the NFZ,
(3) show the Safety Shield intercepting and applying lateral projection,
(4) show the audit log entry with `policy_hash` matching the loaded bundle.

Before this run no Gazebo flight showed steps 2 and 3 together: the rail's
own mission (rail-mission-stub-vla, the 2 Nov demo) flies through the zone's
diagonal, its first forecast slides 8.13 m against theta 2.0 m, so the
projection is abandoned and the FSM escalates (Brake, Loiter, RTL).

- **Scenario:** `midterm-demo-graze` (experiments/templates/static_nfz.yaml),
  flown only by `experiments/profiles/gazebo_midterm.yaml`. Start (0, 3) at
  15 m, pad (45, 12); the straight line cuts nfz-square's corner shallowly.
  Pilot: the rails' StubVLA at 6 m/s (it reads no text). The rail's Shield
  node parses the task "fly to the northeast pad at 6 m/s" and the scenario's
  target moves the pad.
- **Code:** 7525b36 (branch feat/midterm-gazebo-demo, clean worktree), each
  run's manifest records `7525b362e2d8`.
- **Command:** `python experiments/sweep_scenarios.py --profile gazebo_midterm
  --backend sitl --out demo/out/gzmidterm/sweep.json` (284.5 s).
- **Rail:** Gazebo Harmonic gz_sim 8.11.0, iris_runway.sdf, ArduPilot SITL
  JSON, MAVROS 2, ROS 2 Shield node, topology `dev`, sim_speedup 1.0.
  Functional evidence, not KPI-grade (functional_only, the lab's choice of
  17 Nov).

## Result (`gate_check.json`, from each run's own files)

| Step | Evidence, all three seeds (1001-1003) |
|---|---|
| 1 signed bundle | `bundles/fase3-sim-demo-v0.1.4.tar.gz` (polygon NFZ nfz-square, envelopes alt-band and kin-caps), `--expected-hash` pinned; kpi.json policy_source: "Ed25519 signature by 'lab-dev:ed25519:6ec75b535ef5c533' verified" |
| 2 task that wants to enter | the pilot's raw command at tick 1 is (5.88, 1.18) m/s world, its 5 s forecast inside nfz-square's margin |
| 3 interception + lateral projection | tick 1: G1 "nfz-square (hard P0 project_fix) repaired, lat 0.91 m vs theta 2.00 m"; GeofenceSlide on the first 22-23 ticks, slide 0.91 m falling to 0.03 m, none braked; emitted (0.63, -1.10) m/s at tick 1; the pilot then flies past the corner outside the margin to the pad |
| 4 policy_hash | every audit record and the run manifest: `sha256:b665adba...f358`, the bundle's hash |

Sweep: 3/3 pass, P0 escape 0, outcome success, no fail-safe (labelled none,
none triggered); every episode bundle (demo/out/stress_gazebo_midterm/) records
the grant's six names as written. The FSM enters Brake on tick 1 (G1) and the
repaired pilot action is flown in it for the rest of the mission (kin-caps
clamps 6 to 4 m/s on every tick).

The same three missions were first flown at 0cd298d from the unsigned YAML
(`--dev-yaml`, the sitl backend's default): 3/3 pass, slide 0.91-0.92 m. They
are kept in demo/out/_gzmidterm_unsigned_0cd298d/ and are not the evidence
for step 1.

## Files

| File | What it is |
|---|---|
| `sweep.json`, `sweep_stdout.txt`, `kpi_report_sweep.md` | the sweep's record, console and per-cell report |
| `gate_check.json` | the four steps read from each run folder (audit, manifest, kpi.json, prompt.yaml, physics.json, episode bundle) |

Run folders (worktree feat/midterm-gazebo-demo, gitignored):
`demo/out/gzsweep_midterm-demo-graze--seed100{1,2,3}/` with the flight log,
rosbag2, signed replay bundle and the flight-log replay video
`<run>_track.mp4` (tools/render_track_video.py).
