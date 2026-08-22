"""多房间（5/6/7）与家具场景的分割测试。"""

import numpy as np
import pytest

from fixtures import (
    make_corridor_apartment_map,
    make_room_grid_map,
)

from oomwoo_cleaning_jobs_core.segmentation import UNCLASSIFIED, segment

ROOM = 30  # fixtures 默认 room_cells


def _room_label(result, rooms, key, margin=3):
    """断言某房间内部（缩进 margin）恰好属于一个候选，返回其 label。"""
    r0, c0 = rooms[key]
    inner = result.labels[
        r0 + margin:r0 + ROOM - margin,
        c0 + margin:c0 + ROOM - margin,
    ]
    values = {int(v) for v in np.unique(inner) if v != UNCLASSIFIED}
    assert len(values) == 1, f'房间 {key} 内部出现多个 label: {values}'
    return values.pop()


def _assert_grid_exact_rooms(result, rooms, expected_count):
    assert len(result.regions) == expected_count
    room_labels = {key: _room_label(result, rooms, key) for key in rooms}
    assert len(set(room_labels.values())) == expected_count


@pytest.mark.parametrize(('n_cols', 'n_rows', 'skip', 'expected'), [
    (3, 2, frozenset({(2, 1)}), 5),
    (3, 2, frozenset(), 6),
    (4, 2, frozenset({(3, 1)}), 7),
], ids=['5_rooms', '6_rooms', '7_rooms'])
def test_room_grid_exact_count(n_cols, n_rows, skip, expected):
    """N 房间网格（0.5 m 门洞）应精确分出 N 个候选，每房间各一个、互不混淆。"""
    source, rooms = make_room_grid_map(n_cols, n_rows, skip=skip)
    assert len(rooms) == expected
    result = segment(source)
    _assert_grid_exact_rooms(result, rooms, expected)
    # 候选不重叠且不越出自由空间
    covered = np.zeros(source.cells.shape, dtype=bool)
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()
        assert not (mask & covered).any()
        covered |= mask


def test_room_grid_with_furniture_keeps_room_count():
    """6 房间网格中 3 个房间摆放家具（通道足够宽）：
    房间数不变，候选不含障碍 cell。"""
    furniture = (
        ((0, 0), 2, 2, 6),     # 0.3 m 方块，贴角
        ((1, 0), 2, 10, 8),    # 0.4 m 方块，贴底墙
        ((2, 1), 5, 22, 5),    # 0.25 m 方块，贴右墙
    )
    source, rooms = make_room_grid_map(3, 2, furniture=furniture)
    result = segment(source)
    _assert_grid_exact_rooms(result, rooms, 6)
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()
        assert not (mask & source.occupied_mask()).any()


def test_corridor_apartment_rooms_plus_corridor():
    """走廊户型：4 房间 + 1 走廊 = 5 个候选；走廊为单一 label。"""
    source, room_cols = make_corridor_apartment_map(n_rooms=4)
    result = segment(source)
    assert len(result.regions) == 5
    # 走廊内部（rows 3-18，避开门洞脊线）单一 label
    corridor = result.labels[3:18, 3:-3]
    corridor_labels = {int(v) for v in np.unique(corridor) if v != UNCLASSIFIED}
    assert len(corridor_labels) == 1
    corridor_label = corridor_labels.pop()
    # 每个房间内部单一 label，且都不是走廊
    room_labels = set()
    for c0 in room_cols:
        inner = result.labels[25:49, c0 + 3:c0 + ROOM - 3]
        values = {int(v) for v in np.unique(inner) if v != UNCLASSIFIED}
        assert len(values) == 1
        room_labels.add(values.pop())
    assert len(room_labels) == 4
    assert corridor_label not in room_labels
