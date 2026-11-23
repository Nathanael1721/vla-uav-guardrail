"""Explain an episode_digest difference leaf by leaf.

    python tools/episode_leafdiff.py dump <checkout> <keys.json> <out.json>
    python tools/episode_leafdiff.py compare <base.json> <branch.json> <out.json>

tools/episode_digest.py says THAT an episode's rows, events, extra (the KPI
inputs), audit records or extras record changed between two checkouts; this
says HOW. `dump` flies the named episodes (keys as episode_digest writes them,
e.g. "smoke:nfz-head-on:seed=1001:arm=canonical,prefix-off:fc=grant:rsp=ideal")
with THAT checkout's experiments/sweep_scenarios.run_episode, exactly as
episode_digest does, and keeps every record (wall-clock fields dropped, the
checkout's path replaced by "<ROOT>"). `compare` walks two dumps leaf by leaf
and sorts each difference into

  numeric     two finite numbers that differ (|a - b| kept: the largest, where);
  zero_sign   0.0 against -0.0 (equal as numbers, not as JSON);
  discrete    anything else - a string, a boolean, None against a value, a
              list length, a key on one side only (the first few kept).

An episode whose differences are numeric at rounding size and zero_sign only,
with its `extra` (the KPI inputs) unchanged, flew the same flight to float
tolerance; a discrete difference is a different decision, or a hash or label
computed from a number that moved (the dump names which).

First written for wf21 (WP3-04 / ARCH-04, 2026-11-18): the body-frame
hand-over on every tick changes the last bits of the harness's actions at
headings that are not a whole number of quarter turns.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

WALL = {"wall_s", "wall_ms", "wall_ns", "shield_ms", "tick_ms", "elapsed_ms", "ts",
        "tick_phases_ms"}
FIELDS = ("rows", "events", "extra", "audit", "extras_record")


def _strip(o):
    if isinstance(o, dict):
        return {k: _strip(v) for k, v in o.items() if k not in WALL}
    if isinstance(o, (list, tuple)):
        return [_strip(v) for v in o]
    if hasattr(o, "model_dump"):
        return _strip(o.model_dump(mode="json"))
    return o


def dump(root: Path, keys: list[str]) -> dict:
    root = root.resolve()
    sys.path[:0] = [str(root), str(root / "experiments")]
    import sweep_scenarios as S
    from guardrail.scenario_spec import Library, expand, load_profile
    forms = sorted({str(root), root.as_posix(), str(root).replace("\\", "\\\\")},
                   key=len, reverse=True)
    lib = Library(root / "experiments" / "scenarios.yaml")
    want, out = set(keys), {}
    for prof in sorted({k.split(":")[0] for k in keys}):
        eps, _ = expand(lib, load_profile(root / "experiments" / "profiles" / f"{prof}.yaml"))
        for ep in eps:
            key = (f"{prof}:{ep.episode_id}:seed={ep.seed}:arm={getattr(ep, 'arm_tag', '')}:"
                   f"fc={getattr(ep, 'forecast', '')}:rsp={getattr(ep, 'response', '')}")
            if key not in want:
                continue
            spec = ep.cell.spec
            arm = S.build_arm(ep, defaults=lib.defaults)
            run = S.run_episode(spec, lib.defaults, ep.seed, arm=arm,
                                forecast=ep.forecast, response=ep.response)
            ex = S.episode_extras(spec, lib.defaults, ep.seed, run, episode=ep)
            audit = [a if isinstance(a, dict) else
                     (a.model_dump(mode="json") if hasattr(a, "model_dump") else repr(a))
                     for a in run.audit]
            s = json.dumps(_strip({"rows": run.rows, "events": run.events, "extra": run.extra,
                                   "audit": audit, "extras_record": ex}),
                           sort_keys=True, default=repr)
            for f in forms:
                s = s.replace(f, "<ROOT>")
            out[key] = json.loads(s)
    missing = sorted(want - set(out))
    if missing:
        raise SystemExit(f"no such episode(s): {missing}")
    return out


def _walk(a, b, path, acc):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                acc["discrete"].append([f"{path}/{k}", "on one side only"])
            else:
                _walk(a[k], b[k], f"{path}/{k}", acc)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            acc["discrete"].append([path, f"length {len(a)} vs {len(b)}"])
        for i, (x, y) in enumerate(zip(a, b)):
            _walk(x, y, f"{path}[{i}]", acc)
        return
    num = (int, float)
    if (isinstance(a, num) and isinstance(b, num)
            and not isinstance(a, bool) and not isinstance(b, bool)
            and math.isfinite(a) and math.isfinite(b)):
        if a == b:
            if math.copysign(1.0, a) != math.copysign(1.0, b):
                acc["zero_sign"] += 1
            return
        acc["numeric"] += 1
        if abs(a - b) > acc["max_abs"][0]:
            acc["max_abs"] = [abs(a - b), path, a, b]
        return
    if a != b and not (isinstance(a, float) and isinstance(b, float)
                       and math.isnan(a) and math.isnan(b)):
        acc["discrete"].append([path, f"{a!r} vs {b!r}"[:240]])


def compare(base: dict, new: dict) -> dict:
    out = {}
    for k in sorted(set(base) | set(new)):
        if k not in base or k not in new:
            out[k] = {"missing": "base" if k not in base else "branch"}
            continue
        per = {}
        for f in FIELDS:
            acc = {"numeric": 0, "zero_sign": 0, "max_abs": [0.0, None, None, None],
                   "discrete": []}
            _walk(base[k][f], new[k][f], f, acc)
            if acc["numeric"] or acc["zero_sign"] or acc["discrete"]:
                per[f] = {"numeric_leaves": acc["numeric"], "zero_sign_leaves": acc["zero_sign"],
                          "max_abs": acc["max_abs"], "discrete_leaves": len(acc["discrete"]),
                          "discrete_first": acc["discrete"][:8]}
        out[k] = per
    return out


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) == 4 and argv[0] == "dump":
        keys = json.loads(Path(argv[2]).read_text(encoding="utf-8"))
        res = dump(Path(argv[1]), keys)
    elif len(argv) == 4 and argv[0] == "compare":
        b, n = (json.loads(Path(p).read_text(encoding="utf-8")) for p in argv[1:3])
        res = compare(b, n)
        for k, v in res.items():
            print(k, {f: (x["numeric_leaves"], x["zero_sign_leaves"], x["discrete_leaves"],
                          float(f"{x['max_abs'][0]:.3g}")) for f, x in v.items()}
                  if "missing" not in v else v)
    else:
        print(__doc__, file=sys.stderr)
        return 64
    Path(argv[3]).write_text(json.dumps(res, indent=1, sort_keys=True) + "\n",
                             encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
