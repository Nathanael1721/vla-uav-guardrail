"""The voxel-map builder's geometry, without a simulator.

Run either way:
    pytest tests/test_voxel_map.py -v
    python tests/test_voxel_map.py

Every obstacle map in this repository was the 160 m cube the builder was
hard-wired to. CityLife needs a 360 x 200 m rectangle, and a non-cubic grid is
where an axis mix-up in the reshape would stop being invisible: in a cube, the
wrong reshape still returns an array of the right shape. These tests build the
simulator's flat array with the simulator's own index formula
(WorldSimApi::createVoxelGrid) and check a known block lands where it is.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "demo"))

from build_voxel_map import (band_occ, cell_of, grid_geometry,   # noqa: E402
                             ground_index, reshape_voxels)


def _sim_flat(geo, solid):
    """The flat bool array createVoxelGrid returns, for a world where
    `solid(x_north, y_east, z_up)` says what is occupied. Same loop, same index
    and same cell centres as the C++."""
    nx, ny, nz, res = geo["nx"], geo["ny"], geo["nz"], geo["res"]
    flat = np.zeros(nx * ny * nz, dtype=bool)
    for i in range(nx):
        X = geo["cx"] + (i - nx // 2) * res
        for j in range(ny):
            Y = geo["cy"] + (j - ny // 2) * res
            for k in range(nz):
                Z = (k - nz // 2) * res
                flat[i + nx * (k + nz * j)] = solid(X, Y, Z)
    return flat


def _world(X, Y, Z):
    if Z == -2.0:                                   # the ground slab
        return True
    # one building, NED x 10..14, y 100..104, 0..12 m up
    return 10.0 <= X <= 14.0 and 100.0 <= Y <= 104.0 and 0.0 <= Z <= 12.0


def test_the_default_is_the_old_cube():
    g = grid_geometry(-80, 80, -80, 80, 160, 2.0)
    assert (g["cx"], g["cy"], g["nx"], g["ny"], g["nz"]) == (0.0, 0.0, 80, 80, 80)
    assert (g["origin_x"], g["origin_y"]) == (-80, -80)


def test_a_rectangle_puts_cell_zero_on_its_corner():
    g = grid_geometry(-140, 220, -60, 140, 40, 2.0)
    assert (g["nx"], g["ny"], g["nz"]) == (180, 100, 20)
    # cell 0 centre = centre - (n // 2) * res = the corner
    assert g["cx"] - (g["nx"] // 2) * 2.0 == -140
    assert g["cy"] - (g["ny"] // 2) * 2.0 == -60


def test_a_block_lands_where_it_is_on_a_non_cubic_grid():
    g = grid_geometry(-140, 220, -60, 140, 40, 2.0)
    vox = reshape_voxels(_sim_flat(g, _world), g["nx"], g["ny"], g["nz"])
    gz = ground_index(vox)
    occ = band_occ(vox, gz, 6.0, 14.0, 2.0)
    assert occ.shape == (180, 100)                   # [North][East]
    i, j = cell_of(12.0, 102.0, g)
    assert occ[i, j] == 1
    for x, y in ((20.0, 102.0), (12.0, 110.0), (-100.0, -50.0), (200.0, 130.0)):
        a, b = cell_of(x, y, g)
        assert occ[a, b] == 0, (x, y)
    assert int(occ.sum()) == 3 * 3                    # x 10,12,14 by y 100,102,104


def test_a_transposed_reshape_would_be_caught():
    """The same flat array read with the axes in the wrong order yields an
    array of the right SHAPE and the wrong map - the failure a cube hides and
    this file exists to catch. (The check above is what pins the right one.)"""
    g = grid_geometry(-140, 220, -60, 140, 40, 2.0)
    flat = _sim_flat(g, _world)
    right = band_occ(reshape_voxels(flat, g["nx"], g["ny"], g["nz"]),
                     ground_index(reshape_voxels(flat, g["nx"], g["ny"], g["nz"])),
                     6.0, 14.0, 2.0)
    wrong = np.asarray(flat).reshape(g["nx"], g["nz"], g["ny"]).transpose(2, 1, 0)
    occ = band_occ(wrong, ground_index(wrong), 6.0, 14.0, 2.0)
    assert occ.shape == right.shape and not np.array_equal(occ, right)


def test_bad_extents_are_refused():
    for args in ((-80, 81, -80, 80, 160, 2.0), (0, 0, -80, 80, 160, 2.0)):
        try:
            grid_geometry(*args)
        except ValueError:
            continue
        raise AssertionError(f"accepted {args}")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
