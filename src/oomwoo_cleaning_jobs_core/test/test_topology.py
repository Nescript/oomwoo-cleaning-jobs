"""门口记录与拓扑（混合路线）测试。

区域生成仍是 maximin 淹没 + 合并树；门口记录由合并树山口给出，
包含 center（山口 cell）、width_m（≈ 2×clearance 的近似门宽，非实测）、
clearance_m（山口高度）。
"""

import cv2

from fixtures import (
    make_corridor_apartment_map,
    make_open_plan_map,
    make_room_grid_map,
    make_tiny_room_map,
    make_two_rooms_map,
)

import oomwoo_cleaning_jobs_core.segmentation as segmentation
from oomwoo_cleaning_jobs_core.segmentation import segment


def test_two_rooms_single_doorway():
    """双房间由一扇 0.5 m 门连接：恰一条拓扑边，位置在内墙门洞处。"""
    result = segment(make_two_rooms_map())
    assert len(result.regions) == 2
    assert len(result.doorways) == 1
    doorway = result.doorways[0]
    labels = {r.label for r in result.regions}
    assert set(doorway.regions) == labels
    assert 0.35 <= doorway.width_m <= 0.65
    # 门洞在内墙 col 30、rows 35-44
    row, col = doorway.center
    assert 33 <= row <= 46
    assert 28 <= col <= 32
    assert doorway.likely_door


def test_corridor_apartment_star_topology():
    """走廊户型：4 条拓扑边，每条都是 房间<->走廊（星型拓扑）。"""
    source, _room_cols = make_corridor_apartment_map(n_rooms=4)
    result = segment(source)
    assert len(result.regions) == 5
    assert len(result.doorways) == 4
    # 走廊是出现在所有边上的那个 label
    counts: dict[int, int] = {}
    for doorway in result.doorways:
        for label in doorway.regions:
            counts[label] = counts.get(label, 0) + 1
    corridor_label = max(counts, key=counts.get)
    assert counts[corridor_label] == 4
    for doorway in result.doorways:
        assert corridor_label in doorway.regions
        assert doorway.likely_door
    # 邻接查询
    assert len(result.adjacent_labels(corridor_label)) == 4


def test_grid6_seven_doorways():
    """3x2 房间网格：7 扇门（4 横 + 3 纵）= 7 条拓扑边，门宽均 ≈0.5 m。"""
    source, _rooms = make_room_grid_map(3, 2)
    result = segment(source)
    assert len(result.regions) == 6
    assert len(result.doorways) == 7
    for doorway in result.doorways:
        assert 0.35 <= doorway.width_m <= 0.65
        assert doorway.likely_door


def test_transitive_connection_uses_local_contact_saddle():
    """传递连接的合并树山口不得替代当前区域对的实际门洞。"""
    source, _ = make_room_grid_map(3, 2)
    result = segment(source)
    dist = cv2.distanceTransform(
        result.free_mask.astype('uint8'), cv2.DIST_L2, 5) * source.resolution
    connections = segmentation._connection_values(
        result.labels, dist, result.free_mask)

    # 3x2 网格会产生 direct=False 的传递连接；至少一条真实门洞的
    # 合并树山口位于其他区域对的门洞，正是溢出裁剪必须避开的情形。
    divergent = []
    for doorway in result.doorways:
        pair = doorway.regions
        _clearance, tree_cell, direct = connections[pair]
        contact = (
            segmentation._geodesic_dilate(result.labels == pair[0], result.free_mask)
            & segmentation._geodesic_dilate(result.labels == pair[1], result.free_mask)
        )
        local_cell = segmentation._contact_saddle_cell(contact, dist)
        if not direct and tree_cell != local_cell:
            divergent.append((doorway, local_cell))

    assert divergent
    for doorway, local_cell in divergent:
        assert local_cell == doorway.center


def test_merged_regions_have_no_doorway():
    """宽开口（鞍部合并为 1 个区域）与极小房间（面积合并）均无拓扑边。"""
    assert len(segment(make_open_plan_map(opening_cells=26)).doorways) == 0
    assert len(segment(make_tiny_room_map()).doorways) == 0


def test_doorway_clearance_below_merge_threshold():
    """保留下来的门口其 ratio 必低于鞍部合并阈值（否则已被合并）。"""
    source, _rooms = make_room_grid_map(3, 2)
    result = segment(source)
    for doorway in result.doorways:
        assert doorway.ratio < result.params.saddle_merge_ratio
