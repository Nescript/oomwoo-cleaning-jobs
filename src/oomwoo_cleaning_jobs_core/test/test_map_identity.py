"""Source Map identity（内容 hash）测试。

规则见 docs/DEVELOPMENT.md：hash 输入为 resolution + width + height
+ origin(position/orientation) + 原始 int8 cell 数据；排除 stamp/frame_id/
map_load_time，不做三值化。任何一项变化都必须产生新 identity。
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
    int(a.identity, 16)  # 合法 hex


def test_short_id_is_prefix():
    m = make_rooms_map()
    assert len(m.short_id) == 12
    assert m.identity.startswith(m.short_id)


def test_cell_change_changes_identity():
    base = make_rooms_map()
    cells = base.cells.copy()
    cells[10, 10] = 100 if cells[10, 10] != 100 else 0
    assert _variant(cells=cells).identity != base.identity


def test_resolution_change_changes_identity():
    base = make_rooms_map()
    assert _variant(resolution=0.051).identity != base.identity


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
    """不做三值化：free 阈值内的不同原始值（0 与 10）产生不同 identity。"""
    base = make_rooms_map()
    cells = base.cells.copy()
    free_cells = np.argwhere(base.free_mask())
    row, col = free_cells[0]
    cells[row, col] = 10  # 仍是 free（< 25），但原始值不同
    assert _variant(cells=cells).identity != base.identity


def test_masks_on_fixture():
    m = make_rooms_map()
    free = m.free_mask()
    unknown = m.unknown_mask()
    occupied = m.occupied_mask()
    total = m.width * m.height
    assert free.sum() + unknown.sum() + occupied.sum() == total
    # 门洞存在：内墙列在门洞行内是 free
    door_rows = slice(35, 45)
    assert free[door_rows, 30].all()
    # 内墙与外墙是 occupied
    assert occupied[4, 10]
    assert occupied[10, 30]
