"""Tests for Source Map identity (content hash).

Rules per docs/DEVELOPMENT.md: the hash input is resolution + width + height
+ origin (position/orientation) + raw int8 cell data; stamp/frame_id/
map_load_time are excluded and no trinarization is applied. Any change to
any input must produce a new identity.
"""

import numpy as np

from fixtures import make_rooms_map

from oomwoo_cleaning_jobs_core import SourceMap


def _variant(**overrides):
    base = make_rooms_map()
    params = {
        'resolution': base.resolution,
        'width': base.width,
        'height': base.height,
        'origin': base.origin,
        'cells': base.cells.copy(),
    }
    params.update(overrides)
    return SourceMap(**params)


def test_identity_stable_and_format():
    a, b = make_rooms_map(), make_rooms_map()
    assert a.identity == b.identity
    assert len(a.identity) == 64
    int(a.identity, 16)  # valid hex


def test_short_id_is_prefix():
    m = make_rooms_map()
    assert len(m.short_id) == 12
    assert m.identity.startswith(m.short_id)


def test_cell_change_changes_identity():
    base = make_rooms_map()
    cells = base.cells.copy()
    cells[10, 10] = 100 if cells[10, 10] != 100 else 0
    assert _variant(cells=cells).identity != base.identity


def test_resolution_canonicalized_to_float32():
    """resolution is canonicalized to float32: the same map from
    OccupancyGrid (float32) and from map.yaml (float64) must yield the same
    identity."""
    base = make_rooms_map()
    f32_resolution = float(np.float32(base.resolution))
    assert f32_resolution != base.resolution  # 0.05 differs in float64 vs float32
    assert _variant(resolution=f32_resolution).identity == base.identity


def test_resolution_change_changes_identity():
    base = make_rooms_map()
    assert _variant(resolution=0.051).identity != base.identity
    # a difference resolvable in float32 still means a new map
    assert _variant(resolution=base.resolution + 1e-4).identity != base.identity


def test_origin_change_changes_identity():
    base = make_rooms_map()
    assert _variant(origin=(-2.5, -2.0, 0.1)).identity != base.identity
    assert _variant(origin=(-2.4, -2.0, 0.0)).identity != base.identity


def test_shape_change_changes_identity():
    base = make_rooms_map()
    bigger = np.full((base.height, base.width + 1), -1, dtype=np.int8)
    bigger[:, :base.width] = base.cells
    variant = _variant(width=base.width + 1, cells=bigger)
    assert variant.identity != base.identity


def test_identity_depends_on_raw_cells_not_trinary_projection():
    """No trinarization: distinct raw values within the free threshold
    (0 and 10) produce different identities."""
    base = make_rooms_map()
    cells = base.cells.copy()
    free_cells = np.argwhere(base.free_mask())
    row, col = free_cells[0]
    cells[row, col] = 10  # still free (< 25), but a different raw value
    assert _variant(cells=cells).identity != base.identity


def test_masks_on_fixture():
    m = make_rooms_map()
    free = m.free_mask()
    unknown = m.unknown_mask()
    occupied = m.occupied_mask()
    total = m.width * m.height
    assert free.sum() + unknown.sum() + occupied.sum() == total
    # the doorway exists: the interior-wall column is free across the doorway rows
    door_rows = slice(35, 45)
    assert free[door_rows, 30].all()
    # interior and exterior walls are occupied
    assert occupied[4, 10]
    assert occupied[10, 30]
