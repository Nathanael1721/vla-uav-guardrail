"""Every hot-apply generation of one flight, with its mission time and source.

A rail's Shield gets a new policy generation mid-flight from three places:

    REST            the hot-apply API (guardrail.api.FlightHotApply.applied)
    scenario event  the stress harness's --events (guardrail.scenario_events
                    records, as sitl/ros2_shield_node.py keeps them)
    --dynamic       the rail's own built-in zone (apply_zone at DYNAMIC_AT_S)

Until 2026-11-27 (WP3-19) each rail's report.md looked at one source per
row: "Hot-apply API" at the REST listener only and "Dynamic NFZ" at the
--dynamic zone only, so a flight whose generations all came from scenario
events read "not served" and "not used". hot_apply_generations() merges the
three, in generation order, and names any generation the episode record wrote
(EpisodeRecord.generations) that none of them accounts for, so the row cannot
read "none" over a generation that was flown. generations_cell() is the
report.md cell.

Pure functions over the records the rails already keep: nothing here reads a
file, so an old run folder is not re-rendered by it.
"""
from __future__ import annotations

from typing import Any, Iterable

SOURCE_REST = "REST"
SOURCE_SCENARIO = "scenario event"
SOURCE_DYNAMIC = "--dynamic"
SOURCE_UNRECORDED = "source not recorded"


def _gen(v: Any) -> int | None:
    try:
        return None if v is None or isinstance(v, bool) else int(v)
    except (TypeError, ValueError):
        return None


def _t(v: Any) -> float | None:
    try:
        return None if v is None or isinstance(v, bool) else float(v)
    except (TypeError, ValueError):
        return None


def hot_apply_generations(*, rest: Iterable[dict] | None = None,
                          scenario: Iterable[dict] | None = None,
                          dynamic: dict | None = None,
                          recorded: Iterable[dict] | None = None) -> list[dict]:
    """One row per hot-apply generation: {"generation", "t", "source",
    "what", "rule_id"} (plus "record_failed" for a REST event whose record
    write raised), sorted by generation.

    `rest`: FlightHotApply.applied (or its summary()["events"]); `t` is None
    for an event before take-off. `scenario`: the rail's scenario-event
    records; a refused one made no generation and is left out. `dynamic`:
    {"t", "generation", "rule_id"} of the --dynamic zone, or None.
    `recorded`: EpisodeRecord.generations; a generation >= 1 in it that no
    source names is listed with source SOURCE_UNRECORDED and its reason."""
    rows: list[dict] = []
    for e in rest or ():
        row = {"generation": _gen(e.get("generation")), "t": _t(e.get("t")),
               "source": SOURCE_REST,
               "what": " ".join(str(x) for x in (e.get("op"), e.get("type")) if x),
               "rule_id": e.get("rule_id")}
        if e.get("record_failed"):
            row["record_failed"] = e["record_failed"]
        rows.append(row)
    for r in scenario or ():
        if r.get("refused"):
            continue
        idx = r.get("index")
        what = (f"#{idx} " if idx is not None else "") + str(r.get("event") or "")
        rows.append({"generation": _gen(r.get("generation")), "t": _t(r.get("t")),
                     "source": SOURCE_SCENARIO, "what": what.strip(),
                     "rule_id": r.get("rule_id") or r.get("zone") or r.get("target")})
    if dynamic:
        rows.append({"generation": _gen(dynamic.get("generation")),
                     "t": _t(dynamic.get("t")), "source": SOURCE_DYNAMIC,
                     "what": "dynamic_nfz", "rule_id": dynamic.get("rule_id")})
    named = {r["generation"] for r in rows if r["generation"] is not None}
    for g in recorded or ():
        n = _gen(g.get("generation"))
        if n is None or n < 1 or n in named:
            continue
        named.add(n)
        rows.append({"generation": n, "t": None, "source": SOURCE_UNRECORDED,
                     "what": str(g.get("reason") or ""), "rule_id": None})
    big = float("inf")
    rows.sort(key=lambda r: (big if r["generation"] is None else r["generation"],
                             big if r["t"] is None else r["t"]))
    return rows


def _md(text: Any) -> str:
    """Safe inside a Markdown table cell."""
    return str(text).replace("|", "\\|").replace("\n", " ")


def generations_cell(rows: list[dict]) -> str:
    """The report.md cell: 'none', or every generation as
    'g<N> at t=<s> s (<source>: <what> `<rule>`)', separated by '; '."""
    if not rows:
        return "none"
    parts = []
    for r in rows:
        g = "g?" if r["generation"] is None else f"g{r['generation']}"
        if r["source"] == SOURCE_UNRECORDED:
            when = "t not recorded"
        elif r["t"] is None:
            when = "before take-off"
        else:
            when = f"t={r['t']:.2f} s"
        what = r["source"] + (f": {r['what']}" if r["what"] else "")
        if r.get("rule_id"):
            what += f" `{r['rule_id']}`"
        if r.get("record_failed"):
            what += f", record write failed ({r['record_failed']})"
        parts.append(f"{g} {when} ({what})")
    return f"{len(rows)}: " + "; ".join(_md(p) for p in parts)
