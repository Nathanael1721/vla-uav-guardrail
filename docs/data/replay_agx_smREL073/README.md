# smREL073 replayed at its revision on the AGX Orin (WP4-11), 28 Nov 2026

`tools/replay_at_revision.py` (at c44a360, release 0.7.3) over all 52
KPI-grade smREL073 episodes, run on the AGX Orin inside the hil companion
container (`vlaguard-orin-mavros-1`) with the interpreter the Shield node
flies with (`/opt/venv/bin/python`: Python 3.12.3, numpy 1.26.4, pydantic
2.13.4, aarch64, Linux 6.8.12-tegra). Each episode's Shield is rebuilt from
a detached worktree at the revision it flew (c44a360) and run open loop on
the episode's logged inputs.

| Replayed on | Identical | Differs | Not replayable |
|---|---|---|---|
| AGX Orin, the node's runtime, from the run folders (this folder) | **52** | 0 | 0 |
| AGX Orin, from the 52 episode bundles alone (`episode-*/`, this folder, `*_bundles_agx.*`) | **52** | 0 | 0 |
| Desktop, AMD64, Python 3.10 (`../kpi_smREL073/replay_at_revision_smREL073.json`) | 42 | 10 | 0 |

The second row replays the grant-named episode bundles
(`<store>/hil_campaigns/hil_smREL073/episode-*`: manifest.json,
metrics.json, trajectory.json, shield_audit.jsonl, the policy bundle and the
`<tag>.replay.tar.gz` the harness bound to the episode), copied without
episode.bag2 (158 MB), with `--root` at that folder and no run folder in
reach: the self-contained bundle alone reproduces each episode bit for bit.
26.7 s.

Every smREL073 episode is reproduced bit for bit (shield_audit.jsonl records
except `ts`, and per flight-log row raw, violations, repairs, touched,
braked, emitted) by its own revision on the architecture and runtime it flew
on, as ntFIN5 was (`../replay_agx_ntFIN5/`, 654 of 654). The desktop
differences are in the last digits of floating point values (raw and emitted
actions, repair attempts), the cross-architecture effect described there.

How it was run (the container mounts the repository read only, so the
worktree comes from a local clone at the same commit):

    git clone /opt/vlaguard /tmp/vlarepo        # HEAD c44a360
    cd /opt/vlaguard && /opt/venv/bin/python tools/replay_at_revision.py \
        --repo /tmp/vlarepo --root /opt/vlaguard/demo/out/replay_src_smREL073 \
        --only smREL073_ --jobs 6 --out <out>

The 52 run folders (logs and replay tarballs, without bags and videos;
219 MB) were copied from the desktop store to the AGX for this run. 23 s.

The replay is open loop on the logged inputs; it does not re-fly the hil
episode closed loop (SITL and the network are not bit-reproducible).

Files: `replay_at_revision_smREL073_agx.json` (summary),
`replay_at_revision_smREL073_agx.per_run.jsonl.gz` (per episode),
`console.txt`; the bundle replay `replay_at_revision_smREL073_bundles_agx.*`
and `console_bundles.txt`.
