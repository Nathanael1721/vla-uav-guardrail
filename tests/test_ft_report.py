"""training/ft_report.py: the fine-tune report says how each row flew.

    python tests/test_ft_report.py      (no pytest, no matplotlib needed)

WHY THIS FILE EXISTS

docs/aerialvla-ft-report.md headed its table "pure VLA — goal assist OFF",
said every eval flight ran with `--goal-blend 0`, and its chart said "no
guidance assist" - while the three 100 % rows (base_deploy, ft2_deploy,
ft3_deploy) flew with goal_blend 0.55 (training/ft_campaign_stage2.ps1). The
text was hard-coded in the generator, so a rebuild wrote it again; the data
always held the truth (results[*].params.goal_blend). CHANGELOG 2026-10-07
retracted the claim; the report and its generator kept it until 2026-10-19.

Shown failing first: at f93283d ft_report.py has no render_markdown(), and
its hard-coded header is exactly the "goal assist OFF" these tests forbid.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "training"))

import ft_report as F                                              # noqa: E402

SKIP = "SKIP"
COMMITTED = ["baseline", "ft2_yg04", "base_deploy", "ft2_deploy", "ft3_deploy"]
FORBIDDEN = ("goal assist OFF", "PURE VLA", "no guidance assist", "--goal-blend 0`")


def _eval(name, goal_blend, yaw_gain=0.4, n=5, reached=1.0):
    """A stand-in eval; each episode gets its OWN params dict, so a test can
    change one episode without changing them all."""
    def params():
        return None if goal_blend is None else {"goal_blend": goal_blend, "yaw_gain": yaw_gain}
    return {"name": name, "episodes": n, "reached_rate": reached,
            "mean_efficiency": 0.9, "mean_interventions": 10.0, "nfz_total_s": 0.0,
            "results": [{"params": params()} for _ in range(n)]}


def _row(md: str, name: str) -> str:
    return next(ln for ln in md.splitlines() if ln.startswith(f"| {name} |"))


def _committed():
    return [json.loads((ROOT / "training" / f"eval_{n}.json").read_text(encoding="utf-8"))
            for n in COMMITTED]


def test_an_assisted_row_is_labelled_and_nothing_says_pure_for_it():
    md = F.render_markdown([_eval("pure", 0.0), _eval("helped", 0.55)], "2026-07-17", False)
    assert "| 0.55 |" in _row(md, "helped") and "assisted" in _row(md, "helped"), md
    assert "pure VLA" in _row(md, "pure"), md
    for bad in FORBIDDEN:
        assert bad not in md, f"{bad!r} printed beside an assisted row:\n{md}"
    assert "pure VLA" not in F.comparison_title([_eval("pure", 0.0), _eval("helped", 0.55)])


def test_an_eval_without_params_reads_unknown_never_pure():
    e = _eval("old", None)
    assert F.flown_as(e) == F.UNKNOWN
    assert F.flown_as({"name": "x", "results": []}) == F.UNKNOWN
    md = F.render_markdown([e], "2026-07-17", False)
    assert "pure VLA" not in _row(md, "old"), _row(md, "old")


def test_episodes_that_differ_read_mixed():
    e = _eval("mix", 0.0)
    e["results"][0]["params"]["goal_blend"] = 0.55
    assert F.flown_as(e) == "mixed"


def test_the_committed_evals_read_as_they_flew():
    """The July campaign: the two pure rows reached 1 of 5, the three
    *_deploy rows flew goal_blend 0.55 (ft_campaign_stage2.ps1)."""
    got = {e["name"]: F.flown_as(e) for e in _committed()}
    assert got == {"baseline": "pure VLA", "ft2_yg04": "pure VLA",
                   "base_deploy": "assisted", "ft2_deploy": "assisted",
                   "ft3_deploy": "assisted"}, got
    # The script passes each argument as its own string: "--goal-blend", "0.55".
    ps1 = (ROOT / "training" / "ft_campaign_stage2.ps1").read_text(encoding="utf-8")
    runs = re.findall(r'"--name",\s*"(\w+)"[^\n]*"--goal-blend",\s*"0\.55"', ps1)
    assert set(runs) == {"base_deploy", "ft2_deploy", "ft3_deploy"}, \
        f"the campaign script's goal_blend 0.55 runs are now {runs}"


def test_the_committed_report_is_the_generator_s_output_under_its_dated_note():
    """docs/aerialvla-ft-report.md, its dated correction note left out, is
    exactly what render_markdown() writes for the July evals, so the report
    and the generator cannot say different things."""
    text = (ROOT / "docs" / "aerialvla-ft-report.md").read_text(encoding="utf-8")
    text = text.replace("\r\n", "\n")
    lines = text.split("\n")
    assert lines[2].startswith("> **Corrected 2026-10-19.**"), lines[:4]
    end = next(i for i in range(2, len(lines)) if not lines[i].startswith(">"))
    body = "\n".join(lines[:2] + lines[end + 1:])
    want = F.render_markdown(_committed(), "2026-07-17", True)
    assert body == want, "the committed report differs from the generator's output"
    for bad in FORBIDDEN[:3]:
        assert bad not in body, bad


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)
