# ntFIN5 replayed at its revision on the AGX Orin (WP4-11), 26 Nov 2026

`tools/replay_at_revision.py` (at 21c2771, the release 0.7.0 tool) over all
654 KPI-grade ntFIN5 episodes, run on the AGX Orin inside the hil companion
container (`vlaguard-orin-mavros-1`) with the interpreter the Shield node flies
with (`/opt/venv/bin/python`: Python 3.12.3, numpy 1.26.4, pydantic 2.13.4,
aarch64, Linux 6.8.12-tegra). Each episode's Shield is rebuilt from a detached
worktree at the revision it flew (21c2771) and run open loop on the episode's
logged inputs.

| Replayed on | Identical | Differs | Not replayable | Ticks compared | Audit records compared |
|---|---|---|---|---|---|
| AGX Orin, the node's runtime (this folder) | **654** | 0 | 0 | 303,048 | 191,106 |
| Desktop, AMD64, Python 3.10 (docs/data/kpi_ntFIN5/replay_at_revision_ntFIN5.json) | 352 | 302 | 0 | 303,048 | 191,106 |

So every ntFIN5 episode is reproduced bit for bit (shield_audit.jsonl records
except `ts`, and per flight-log row raw, violations, repairs, touched, braked,
emitted) by its own revision on the architecture and runtime it flew on. The
302 desktop differences (192 in the last digits of the body-frame rotation,
109 other float last digits, 1 decision) are a cross-architecture floating
point effect of replaying on another CPU and numerics stack, not a
nondeterminism of the Shield.

How it was run (the container mounts the repository read only, so the
worktrees come from a local clone at the same commit):

    git clone /opt/vlaguard /tmp/vlarepo        # HEAD 21c2771
    cd /opt/vlaguard && /opt/venv/bin/python tools/replay_at_revision.py \
        --repo /tmp/vlarepo --root /opt/vlaguard/demo/out/replay_src_ntFIN5 \
        --only ntFIN5_ --jobs 6 --out <out>

The episodes' run folders (logs and replay tarballs, without bags and videos)
were copied from the desktop store to the AGX for this run. 224 s.

What this does not show: the replay is open loop on the logged inputs; it does
not re-fly the hil episode closed loop (SITL and the network are not
reproducible bit for bit), so the grant's "reproducible bit-for-bit" for the
trajectory itself stays as stated in Appendix A (WP4-11).

Files: `replay_at_revision_ntFIN5_agx.json` (summary),
`replay_at_revision_ntFIN5_agx.per_run.jsonl.gz` (per episode), `console.txt`.
