"""segmentation 自动分割测试（合成夹具，无头可跑）。"""

import numpy as np

from fixtures import (
    make_open_plan_map,
    make_open_room_map,
    make_rooms_map,
    make_tiny_room_map,
    make_two_rooms_map,
)

from oomwoo_cleaning_jobs_core.segmentation import (
    UNCLASSIFIED,
    SegmentationParams,
    segment,
)


def test_two_rooms_split_by_door():
    """干净双房间+门洞夹具应精确分出 2 个正常置信候选区域。"""
    source = make_two_rooms_map()
    result = segment(source)
    assert len(result.regions) == 2
    assert all(not r.low_confidence for r in result.regions)
    # 每个区域都在 free 空间内、互不重叠、面积合理（远大于 min_region_area）
    covered = np.zeros(source.cells.shape, dtype=bool)
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()
        assert not (mask & covered).any()
        covered |= mask
        assert region.area_m2 > 2.0


def test_regions_stay_within_free_space():
    """任何候选都不得包含障碍/未知 cell（含房间内未知块周围的分裂情形）。"""
    source = make_rooms_map()
    result = segment(source, SegmentationParams(saddle_merge_ratio=99.0))
    assert len(result.regions) >= 2
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()


def test_watershed_ridge_cells_marked_unclassified():
    """watershed 脊线 cell 应显式标为未分类，且占比很小。"""
    source = make_rooms_map()
    result = segment(source)
    unclassified = result.unclassified_free_mask
    assert unclassified.any()
    assert unclassified.sum() < 0.05 * source.free_mask().sum()


def test_watershed_does_not_cross_walls():
    """区域不得越过内墙：左/右房间的**纵深内部**（离墙 >=2 cell）必须分属
    不同区域（曾用 cv2.watershed 全图淹没，洪水穿墙导致越界）。
    门洞附近 ±2 cell 的分界线摆动是 maximin 淹没的正常边界行为，不在此限。"""
    source = make_two_rooms_map()
    result = segment(source)
    left_inner = result.labels[8:72, 5:28]
    right_inner = result.labels[8:72, 32:70]
    left_labels = {int(v) for v in np.unique(left_inner) if v != UNCLASSIFIED}
    right_labels = {int(v) for v in np.unique(right_inner) if v != UNCLASSIFIED}
    assert len(left_labels) == 1
    assert len(right_labels) == 1
    assert left_labels != right_labels


def test_unknown_patch_lobes_area_merged():
    """未知块周围的小凸瓣（均 < min_region_area）由面积合并吸收：
    含未知块的双房间夹具最终精确 2 个候选。"""
    result = segment(make_rooms_map())
    assert len(result.regions) == 2


def test_saddle_merge_recombines_wide_opening():
    """宽开口（26 cells = 1.3 m，鞍部/峰高比 ≈0.87 >= 0.8）的开放式双区
    由鞍部合并并为 1 个候选；禁用鞍部合并（ratio=99）时为 2 个。"""
    source = make_open_plan_map(opening_cells=26)
    merged = segment(source)
    unmerged = segment(source, SegmentationParams(saddle_merge_ratio=99.0))
    assert len(unmerged.regions) == 2
    assert len(merged.regions) == 1


def test_door_opening_not_saddle_merged():
    """真门洞（10 cells = 0.5 m，鞍部/峰高比 ≈0.33）不会被鞍部合并误并。"""
    result = segment(make_open_plan_map(opening_cells=10))
    assert len(result.regions) == 2


def test_open_room_degenerates_to_single_low_confidence_candidate():
    """开间只有一个距离峰：整体作为单一低置信候选。"""
    result = segment(make_open_room_map())
    assert len(result.regions) == 1
    assert result.regions[0].low_confidence
    # 低置信候选覆盖几乎全部 free 空间
    coverage = result.mask_of(result.regions[0].label).sum() / result.free_mask.sum()
    assert coverage > 0.95


def test_tiny_room_merged_into_neighbor():
    """小于 min_region_area 的独立峰区域并入共享边界最长的近邻。"""
    source = make_tiny_room_map()
    result = segment(source)
    assert len(result.regions) == 1
    region = result.regions[0]
    assert not region.low_confidence
    # 合并后区域覆盖小房间内部（col > 68 的 free cell 绝大部分归属该区域，
    # 允许门洞附近的分水岭脊线 cell 留在未分类）
    right_room_free = source.free_mask()[:, 69:78]
    covered = result.mask_of(region.label)[:, 69:78][right_room_free].sum()
    assert covered >= 0.9 * right_room_free.sum()


def test_unknown_cells_never_in_candidates():
    """unknown（含房间内未知块）不可进入任何候选区域。"""
    source = make_rooms_map()
    result = segment(source)
    assert not (result.labels[source.unknown_mask()] != UNCLASSIFIED).any()


def test_min_region_area_respected():
    """所有最终候选都不小于 min_region_area。"""
    params = SegmentationParams(min_region_area_m2=1.0)
    for make in (make_rooms_map, make_tiny_room_map, make_open_room_map):
        result = segment(make(), params)
        for region in result.regions:
            assert region.area_m2 >= params.min_region_area_m2 * 0.99
