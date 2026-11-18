"""Digest every headless episode of some profiles, per episode, so two
checkouts (a base commit and a branch) can be compared flight for flight.

    python tools/episode_digest.py <checkout> <out.json> <profile> [<profile> ...]
    python tools/episode_digest.py --compare <before.json> <after.json>

Each episode of each profile (experiments/profiles/<profile>.yaml, expanded
over the library experiments/scenarios.yaml exactly as the sweep's headless
backend expands it; its arm from sweep_scenarios.build_arm) is flown with
THAT checkout's experiments/sweep_scenarios.run_episode, and six SHA-256
digests are kept: the rows, the events, the run's extra (the KPI inputs),
the flying Shield's audit records, the episode's extras record
(episode_extras: what a bundle's manifest holds) and the canonical IR of
every policy generation. Wall-clock fields (WALL) are dropped and the
checkout's own path is replaced by "<ROOT>" (the episode_start event's
policy source names the policy file by absolute path), so the same tree in
two folders digests the same; nothing else is normalised.

First written for X-11 (2026-11-18, docs/data/msl_rail_identity_20261118.json):
the branch that added an opt-in ground to the harness had to fly every
episode without a ground byte for byte as before.
"""
from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path

WALL = {"wall_s", "wall_ms", "wall_ns", "shield_ms", "tick_ms", "elapsed_ms", "ts"}
FIELDS = ("rows", "events", "extra", "audit", "extras_record", "generations")


def _strip(o):
    if isinstance(o, dict):
        return {k: _strip(v) for k, v in o.items() if k not in WALL}
    if isinstance(o, (list, tuple)):
        return [_strip(v) for v in o]
    return o


def digest(root: Path, profiles: list[str]) -> dict:
    root = root.resolve()
    sys.path[:0] = [str(root), str(root / "experiments")]
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library, expand, load_profile
    forms = sorted({str(root), root.as_posix(), str(root).replace("\\", "\\\\")},
                   key=len, reverse=True)

    def h(o) -> str:
        s = json.dumps(_strip(o), sort_keys=True, default=repr)
        for f in forms:
            s = s.replace(f, "<ROOT>")
        return hashlib.sha256(s.encode()).hexdigest()

    lib = Library(root / "experiments" / "scenarios.yaml")
    out = {}
    for prof in profiles:
        eps, _ = expand(lib, load_profile(root / "experiments" / "profiles" / f"{prof}.yaml"))
        for ep in eps:
            key = (f"{prof}:{ep.episode_id}:seed={ep.seed}:arm={getattr(ep, 'arm_tag', '')}:"
                   f"fc={getattr(ep, 'forecast', '')}:rsp={getattr(ep, 'response', '')}")
            spec = ep.cell.spec
            try:
                arm = S.build_arm(ep, defaults=lib.defaults)
                run = S.run_episode(spec, lib.defaults, ep.seed, arm=arm,
                                    forecast=ep.forecast, response=ep.response)
                ex = S.episode_extras(spec, lib.defaults, ep.seed, run, episode=ep)
            except Exception as e:                                   # noqa: BLE001
                out[key] = {"error": f"{type(e).__name__}: {e}"}
                continue
            audit = [a if isinstance(a, dict) else
                     (a.model_dump(mode="json") if hasattr(a, "model_dump") else repr(a))
                     for a in run.audit]
            out[key] = {"rows": h(run.rows), "events": h(run.events), "extra": h(run.extra),
                        "audit": h(audit), "extras_record": h(ex),
                        "generations": h({str(k): v.hex() if isinstance(v, bytes) else v
                                          for k, v in run.generations.items()}),
                        "n_rows": len(run.rows)}
    return out


def compare(before: dict, after: dict) -> dict:
    diffs = sorted({(k, f) for k in before for f in FIELDS
                    if before[k].get(f) != after.get(k, {}).get(f)})
    return {"episodes_before": len(before), "episodes_after": len(after),
            "same_episodes": sorted(before) == sorted(after),
            "errors": sorted(k for d in (before, after) for k in d if "error" in d[k]),
            "rows_total": sum(v.get("n_rows", 0) for v in before.values()),
            "differing": [list(d) for d in diffs],
            "identical_episodes": sum(1 for k in before if all(
                before[k].get(f) == after.get(k, {}).get(f) for f in FIELDS))}


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "--compare":
        b, a = (json.loads(Path(p).read_text(encoding="utf-8")) for p in argv[1:3])
        res = compare(b, a)
        print(json.dumps(res, indent=1))
        return 0 if res["same_episodes"] and not res["differing"] and not res["errors"] else 1
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 64
    t0 = time.perf_counter()
    out = digest(Path(argv[0]), argv[2:])
    Path(argv[1]).write_text(json.dumps(out, indent=1, sort_keys=True) + "\n",
                             encoding="utf-8", newline="\n")
    print(f"{len(out)} episodes in {time.perf_counter() - t0:.1f} s -> {argv[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
