"""A 2-D obstacle map is only true for the altitude band it was flattened over.

Run either way:
    pytest tests/test_occ_bands.py -v
    python tests/test_occ_bands.py

`docs/CHECKLIST-remaining-work.md` carried this as tidiness: "follow_pedestrian.yaml
permits descent to 4 m but loads the 6-14 m map; ground_2to4.npz exists and
nothing chooses it." Measured, it is not tidiness, and it is not even the right
statement — `ground_2to4` ends exactly where the policy's band begins, so it
would not have helped. The real gap is that **nothing maps 4-6 m at all**.

These tests pin that against the map files themselves, so the claim cannot drift
from the artefacts the way the last three did.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "demo"))

from occ_bands import (BANDS, DEGENERATE_FRACTION,        # noqa: E402
                       available, gaps, select_for_band)

MAPS = ROOT / "demo" / "out" / "citymap"


def _have() -> bool:
    return (MAPS / "occ_day_flightband_6to14.npz").is_file()


# --- the arithmetic, independent of any file ------------------------------

SKIP = "SKIP"


def test_a_band_fully_inside_one_map_has_no_gap():
    b = [{"lo": 6.0, "hi": 14.0}]
    assert gaps(8.0, 10.0, b) == []


def test_a_hole_between_two_bands_is_reported_with_its_edges():
    b = [{"lo": 2.0, "hi": 4.0}, {"lo": 6.0, "hi": 14.0}]
    assert gaps(4.0, 10.0, b) == [(4.0, 6.0)]


def test_bands_that_merely_touch_leave_no_hole():
    """2-4 then 4-6 is continuous. A zero-width gap is not a gap, and reporting
    one would train the reader to ignore the warning that matters."""
    b = [{"lo": 2.0, "hi": 4.0}, {"lo": 4.0, "hi": 6.0}]
    assert gaps(2.0, 6.0, b) == []


def test_a_band_entirely_above_every_map_is_all_gap():
    b = [{"lo": 0.0, "hi": 4.0}]
    assert gaps(10.0, 20.0, b) == [(10.0, 20.0)]


def test_overlapping_bands_do_not_manufacture_a_gap():
    b = [{"lo": 2.0, "hi": 9.0}, {"lo": 6.0, "hi": 14.0}]
    assert gaps(4.0, 12.0, b) == []


# --- against the real maps -------------------------------------------------

def test_the_four_band_maps_are_on_disk_and_declared():
    if not _have():
        return SKIP
    stems = {b["stem"] for b in available(MAPS)}
    assert stems <= set(BANDS), stems
    assert "occ_day_flightband_6to14" in stems and "ground_2to4" in stems


def test_the_pedestrian_policy_band_has_an_unmapped_slice():
    """THE finding. follow_pedestrian.yaml permits 4-10 m; ground_2to4 ends at
    exactly 4 m, so it never applies, and nothing maps 4-6 m."""
    if not _have():
        return SKIP
    sel = select_for_band(MAPS, 4.0, 10.0)
    assert sel["gaps"] == [(4.0, 6.0)], sel["gaps"]
    assert [b["stem"] for b in sel["used"]] == ["occ_day_flightband_6to14"]
    assert any("NO map" in line for line in sel["report"]), sel["report"]


def test_neither_low_nor_cruise_map_contains_the_other():
    """Why picking one map is the wrong question. The low band holds 300 cells
    of structure the cruise map does not; the cruise map holds the building at
    (64, 40) — the documented 9 m collision — which is open at 2-4 m."""
    if not _have():
        return SKIP
    band = np.load(MAPS / "occ_day_flightband_6to14.npz")["occ"].astype(bool)
    low = np.load(MAPS / "ground_2to4.npz")["occ"].astype(bool)
    assert int(band.sum()) == 2015 and int(low.sum()) == 1391
    assert int((low & ~band).sum()) == 300
    assert band[64, 40] and not low[64, 40]


def test_the_ground_plane_is_rejected_rather_than_merged():
    """ground_0to2 is 100 % occupied. Unioned in, every cell is blocked and the
    Shield vetoes everything — which looks identical to a Shield working hard."""
    if not _have():
        return SKIP
    sel = select_for_band(MAPS, 0.0, 3.0)
    assert [b["stem"] for b in sel["rejected"]] == ["ground_0to2"], sel["rejected"]
    assert [b["stem"] for b in sel["used"]] == ["ground_2to4"]
    assert int(sel["map"]["occ"].sum()) == 1391


def test_a_band_spanning_two_real_maps_unions_them():
    if not _have():
        return SKIP
    sel = select_for_band(MAPS, 3.0, 10.0)
    stems = [b["stem"] for b in sel["used"]]
    assert stems == ["ground_2to4", "occ_day_flightband_6to14"], stems
    merged = int(sel["map"]["occ"].sum())
    assert merged == 2015 + 300, merged          # union, not sum
    assert sel["gaps"] == [(4.0, 6.0)], sel["gaps"]


def test_the_merged_map_keeps_the_grid_the_planner_expects():
    if not _have():
        return SKIP
    sel = select_for_band(MAPS, 3.0, 10.0)
    m = sel["map"]
    assert set(m) == {"occ", "height", "res", "ox", "oy", "N"}
    assert m["occ"].dtype == np.uint8 and m["occ"].shape == (80, 80)
    assert m["res"] == 2.0 and m["N"] == 80


def test_an_uncovered_band_returns_no_map_rather_than_the_nearest_one():
    """Returning the closest map would be the silent failure: a plausible grid,
    for the wrong altitude, with nothing saying so."""
    if not _have():
        return SKIP
    sel = select_for_band(MAPS, 100.0, 120.0)
    assert sel["map"] is None and sel["gaps"] == [(100.0, 120.0)]
    assert any("no band map covers" in line for line in sel["report"])


def test_strict_raises_and_names_the_uncovered_slice():
    if not _have():
        return SKIP
    try:
        select_for_band(MAPS, 4.0, 10.0, strict=True)
    except ValueError as exc:
        assert "4-6 m" in str(exc), exc
    else:
        raise AssertionError("strict accepted a partially mapped band")


def test_the_degenerate_threshold_sits_clear_of_the_real_maps():
    """0.90 has to be above every real map and below the ground plane, or the
    guard either fires on good data or never fires at all."""
    if not _have():
        return SKIP
    fracs = {b["stem"]: float(np.load(b["path"])["occ"].astype(bool).mean())
             for b in available(MAPS)}
    assert fracs["ground_0to2"] == 1.0, fracs
    for stem, f in fracs.items():
        if stem != "ground_0to2":
            assert f < DEGENERATE_FRACTION, (stem, f)


if __name__ == "__main__":
    # A test that short-circuits on a missing fixture must NOT print PASS - on a
    # clean clone demo/out/ is gitignored and those tests assert nothing. See
    # tests/test_replay.py, where that hid 7 no-ops behind "8/8 passed".
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__} (fixture missing)")
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
