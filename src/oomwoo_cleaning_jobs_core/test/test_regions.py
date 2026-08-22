"""RegionSet 编辑语义测试（对应 DEVELOPMENT.md 编辑语义决定）。"""

import numpy as np
import pytest

from fixtures import make_two_rooms_map

from oomwoo_cleaning_jobs_core.regions import UNASSIGNED, RegionSet
from oomwoo_cleaning_jobs_core.segmentation import segment


def _make_region_set():
    source = make_two_rooms_map()
    result = segment(source)
    return source, RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin)


def _stroke(rows, cols, shape):
    mask = np.zeros(shape, dtype=bool)
    mask[rows, cols] = True
    return mask


def test_from_segmentation_initializes_regions():
    _, rs = _make_region_set()
    regions = rs.regions()
    assert len(regions) == 2
    assert all(r.name for r in regions)
    assert all(r.area_m2 > 2.0 for r in regions)


def test_paint_clips_to_cleanable():
    """即时裁剪：盖到墙/未知的笔画部分被裁掉，只落进可清扫空间。"""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    before = rs.mask_of(label).sum()
    # 笔画横跨右房间（free）与右侧 unknown 区（cols 75-90）
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[40, 60:90] = True
    assert rs.paint(label, stroke)
    gained = rs.mask_of(label)[:, 60:90]
    # 只获得 free cell（cols 60-69），unknown（cols 71+）不得进入
    assert (gained[:, 11:] == 0).all()
    assert rs.mask_of(label).sum() > before


def test_paint_empty_stroke_is_invalid():
    """裁空即无效：整个笔画在墙上时不产生任何变化。"""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    before = rs.labels.copy()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[4, 10:20] = True  # 底墙（occupied）
    assert not rs.paint(label, stroke)
    assert np.array_equal(rs.labels, before)


def test_paint_preempts_existing_region():
    """后画者抢占：压到已有 Region 的笔画，重叠 cell 归新 Region。"""
    source, rs = _make_region_set()
    a, b = [r.label for r in rs.regions()]
    size_a_before = rs.mask_of(a).sum()
    # b 的画笔压进 a 的领地一条带
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[10:20, 10:25] = True
    band_a = int(rs.mask_of(a)[10:20, 10:25].sum())
    assert band_a > 0
    assert rs.paint(b, stroke)
    lost = size_a_before - rs.mask_of(a).sum()
    assert lost == band_a  # a 恰好失去带内全部 cell，不多不少
    assert rs.mask_of(b)[10:20, 10:25].all()
    assert not rs.mask_of(a)[10:20, 10:25].any()


def test_create_and_delete_and_rename():
    source, rs = _make_region_set()
    assert len(rs.regions()) == 2
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[60:70, 40:50] = True
    new_label = rs.create(stroke, name='阳台')
    assert new_label is not None
    assert len(rs.regions()) == 3
    names = {r.label: r.name for r in rs.regions()}
    assert names[new_label] == '阳台'
    # create 的抢占语义：该带原属某 Region，现在归新 Region
    assert (rs.labels[60:70, 40:50] == new_label).all()
    assert rs.rename(new_label, '书房')
    assert rs.names[new_label] == '书房'
    assert rs.delete(new_label)
    assert len(rs.regions()) == 2
    assert (rs.labels[60:70, 40:50] == UNASSIGNED).all()


def test_create_empty_stroke_returns_none():
    source, rs = _make_region_set()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[0, 0] = True  # unknown
    assert rs.create(stroke) is None
    assert len(rs.regions()) == 2


def test_erase_shrinks_and_auto_deletes():
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    before = rs.mask_of(label).sum()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[5:8, :] = True
    rs.erase(label, stroke)
    assert rs.mask_of(label).sum() < before
    # 减空自动删除
    rs.erase(label, np.ones(source.cells.shape, dtype=bool))
    assert all(r.label != label for r in rs.regions())
    with pytest.raises(ValueError):
        rs.paint(label, stroke)


def test_merge_combines_regions():
    source, rs = _make_region_set()
    a, b = [r.label for r in rs.regions()]
    total = rs.mask_of(a).sum() + rs.mask_of(b).sum()
    assert rs.merge(a, b)
    regions = rs.regions()
    assert len(regions) == 1
    assert regions[0].label == a
    assert rs.mask_of(a).sum() == total


def test_split_by_cut_line():
    """画线拆分：竖线贯穿左房间 → 两片，大片保留原 label。"""
    source, rs = _make_region_set()
    a, b = sorted(r.label for r in rs.regions())
    # 左房间（cols 5-29）竖切 col 15
    cut = np.zeros(source.cells.shape, dtype=bool)
    cut[5:75, 15] = True
    new_labels = rs.split(a, cut)
    assert new_labels is not None
    assert len(new_labels) == 2
    assert new_labels[0] == a  # 大片保留原 label
    # 两片互不重叠，且原 Region cell 要么在片中、要么在切割线/未划分
    for lb in new_labels:
        assert rs.mask_of(lb).any()
    assert not (rs.mask_of(new_labels[0]) & rs.mask_of(new_labels[1])).any()


def test_split_without_separation_is_invalid():
    """切割线没有把 Region 分成两片时无效。"""
    source, rs = _make_region_set()
    a = sorted(r.label for r in rs.regions())[0]
    cut = np.zeros(source.cells.shape, dtype=bool)
    cut[5, 10] = True  # 一个点，切不断
    before = rs.labels.copy()
    assert rs.split(a, cut) is None
    assert np.array_equal(rs.labels, before)


def test_outline_derives_polygon_in_map_frame():
    """轮廓由掩码派生：外环在 map frame（米）下大致等于房间边界。"""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    rings = rs.outline(label)
    assert rings
    outer = rings[0]
    res = source.resolution
    ox, oy = source.origin[0], source.origin[1]
    # 房间范围 cells rows 5-74, cols 5-29 → 米制边界检查
    assert outer[:, 0].min() >= ox + 4 * res
    assert outer[:, 0].max() <= ox + 31 * res
    assert outer[:, 1].min() >= oy + 4 * res
    assert outer[:, 1].max() <= oy + 76 * res


def test_unassigned_cleanable_mask():
    _, rs = _make_region_set()
    # 两个 Region 加未分类脊线：unassigned 应非空且全部可清扫
    unassigned = rs.unassigned_cleanable_mask
    assert (unassigned <= rs.cleanable).all()
