# Every other KPI-grade hil campaign replayed on the AGX Orin (WP4-11), 28 Nov 2026

`tools/replay_at_revision.py` (at c44a360, release 0.7.3) over the KPI-grade
hil episodes not yet replayed on the Orin, run inside the hil companion
container (`vlaguard-orin-mavros-1`) with the interpreter the Shield node
flies with (`/opt/venv/bin/python`: Python 3.12.3, numpy 1.26.4, pydantic
2.13.4, aarch64). Each episode's Shield is rebuilt from a detached worktree
at the revision it flew and run open loop on the episode's logged inputs.

| Campaign | Flown at | Episodes | Identical | Differs | Not replayable |
|---|---|---|---|---|---|
| ntFIN4 (nightly) | 59e19a5 | 654 | 654 | 0 | 0 |
| smFIN2 (smoke) | 59e19a5 | 52 | 52 | 0 | 0 |
| smREL070 (release 0.7.0 smoke) | 21c2771 | 52 | 52 | 0 | 0 |
| smREL071 (release 0.7.1 smoke) | a3d616e | 52 | 52 | 0 | 0 |
| smREL072 (release 0.7.2 smoke) | 418b151 | 52 | 52 | 0 | 0 |
| pdyn1126 (four components, dynamic cells) | a3d616e | 12 | 12 | 0 | 0 |
| wgs01_001 (harness WGS84 bundle) | a3d616e | 1 | 1 | 0 | 0 |
| ca1127_001 (Lab CA verification flight) | 9589623 | 1 | 1 | 0 | 0 |
| **All** | | **876** | **876** | **0** | **0** |

With ntFIN5 (654 of 654, `../replay_agx_ntFIN5/`) and smREL073 (52 of 52
from the run folders and 52 of 52 from the episode bundles,
`../replay_agx_smREL073/`), every KPI-grade hil campaign from ntFIN4 on is
reproduced bit for bit by its own revision on the runtime it flew on: 1,582
episodes, 0 differences.

How it was run: the run folders (logs and replay tarballs, without bags,
videos and frames; 3.5 GB) were copied from the lab checkout's
`demo/out/hil_smoke` (ntFIN4, smFIN2, smREL070, ca1127) and the offload
store's `hil_runs` (smREL071, smREL072, pdyn1126, wgs01) to
`demo/out/replay_src_rest` on the AGX, then

    cd /opt/vlaguard && /opt/venv/bin/python tools/replay_at_revision.py \
        --repo /tmp/vlarepo --root /opt/vlaguard/demo/out/replay_src_rest \
        --jobs 6 --out <out>

with `/tmp/vlarepo` a clone of the AGX checkout at c44a360. 315.7 s.

The replay is open loop on the logged inputs; it does not re-fly the hil
episode closed loop (SITL and the network are not bit-reproducible).

Files: `replay_at_revision_rest_agx.json` (summary),
`replay_at_revision_rest_agx.per_run.jsonl.gz` (per episode), `console.txt`.
