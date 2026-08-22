"""validation 校验分级测试。"""

import numpy as np

from fixtures import make_two_rooms_map

from oomwoo_cleaning_jobs_core.regions import RegionSet
from oomwoo_cleaning_jobs_core.segmentation import segment
from oomwoo_cleaning_jobs_core.validation import (
    LEVEL_ERROR,
    LEVEL_WARNING,
    check_masks_overlap,
    validate_region_set,
)


def _make_region_set():
    source = make_two_rooms_map()
    result = segment(source)
    return source, RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin)


def _codes(report, level):
    return {i.code for i in report.issues if i.level == level}


def test_clean_region_set_has_no_errors():
    """正常编辑路径产生的 Region Set 不触发任何 error（不变量天然成立）。"""
    _, rs = _make_region_set()
    report = validate_region_set(rs)
    assert report.ok
    assert not report.errors


def test_unassigned_cleanable_is_warning_not_error():
    """存在未划分可清扫空间是 warning，可发布。"""
    _, rs = _make_region_set()
    report = validate_region_set(rs)
    assert 'unassigned_cleanable' in _codes(report, LEVEL_WARNING)
    assert report.ok


def test_fully_assigned_set_has_no_unassigned_warning():
    source, rs = _make_region_set()
    # 把所有未划分的可清扫 cell 都画给第一个 Region
    label = rs.regions()[0].label
    rs.paint(label, rs.unassigned_cleanable_mask)
    report = validate_region_set(rs)
    assert 'unassigned_cleanable' not in _codes(report, LEVEL_WARNING)
    assert report.ok


def test_region_outside_cleanable_is_error():
    """不变量检查：手改数据把 Region cell 戳到墙上 → error。"""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    rs.labels[4, 10] = label  # 底墙（occupied）
    rs.labels[0, 0] = label   # unknown
    report = validate_region_set(rs)
    assert not report.ok
    assert 'region_outside_cleanable' in _codes(report, LEVEL_ERROR)


def test_narrow_region_unreachable_is_error():
    """1-cell 宽的细长条区域经 footprint 腐蚀后为空 → error。"""
    source, rs = _make_region_set()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[40, 5:29] = True  # 单行，宽 0.05 m，机器人进不去
    label = rs.create(stroke, name='细缝')
    assert label is not None
    report = validate_region_set(rs)
    assert not report.ok
    narrow = [i for i in report.errors if i.code == 'region_unreachable']
    assert len(narrow) == 1
    assert narrow[0].region == label


def test_disconnected_core_is_warning():
    """两片宽裕区域由 1-cell 窄喉连接：可达核心断成两片 → warning 而非 error。

    不用「不可达比例」指标：任何房间的周界一圈都是机器人中心不可达的
    （约 30%），该指标对正常房间必然误报。"""
    source, rs = _make_region_set()
    # 在左房间上画：两个 10x10 块（0.5 m，各有可达核心）+ 1-cell 窄喉
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[10:20, 8:18] = True    # 块 1
    stroke[10:20, 20:29] = True   # 块 2
    stroke[15, 18:20] = True      # 窄喉（0.05 m < 机器人直径）
    label = rs.create(stroke, name='哑铃')
    assert label is not None
    report = validate_region_set(rs)
    assert report.ok  # warning 不阻止发布
    assert 'region_disconnected_core' in _codes(report, LEVEL_WARNING)
    assert 'region_unreachable' not in _codes(report, LEVEL_ERROR)
    # 正常房间的核心是连通的一片，不应误报
    normal = [i for i in report.warnings if i.code == 'region_disconnected_core']
    assert all(i.region == label for i in normal)


def test_keepout_intersection_is_error():
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    keepout = np.zeros(source.cells.shape, dtype=bool)
    keepout[40, 10:20] = True
    # Keepout 与 Region 相交 → error
    assert rs.mask_of(label)[40, 10:20].any()
    report = validate_region_set(rs, keepout_mask=keepout)
    assert not report.ok
    assert 'region_in_keepout' in _codes(report, LEVEL_ERROR)
    # 不相交的 Keepout → 无 error
    keepout2 = np.zeros(source.cells.shape, dtype=bool)
    keepout2[4, 10:20] = True  # 墙上，不在任何 Region 内
    report2 = validate_region_set(rs, keepout_mask=keepout2)
    assert 'region_in_keepout' not in _codes(report2, LEVEL_ERROR)


def test_empty_region_set_is_error():
    source = make_two_rooms_map()
    rs = RegionSet(
        labels=np.zeros(source.cells.shape, dtype=np.int32),
        cleanable=source.free_mask(),
        resolution=source.resolution,
    )
    report = validate_region_set(rs)
    assert not report.ok
    assert 'empty_region_set' in _codes(report, LEVEL_ERROR)


def test_check_masks_overlap():
    source = make_two_rooms_map()
    mask_a = np.zeros(source.cells.shape, dtype=bool)
    mask_b = np.zeros(source.cells.shape, dtype=bool)
    mask_a[10:20, 10:20] = True
    mask_b[15:25, 15:25] = True
    issues = check_masks_overlap({1: mask_a, 2: mask_b})
    assert len(issues) == 1
    assert issues[0].code == 'region_overlap'
    mask_b[15:25, 15:25] = False
    mask_b[30:40, 30:40] = True
    assert check_masks_overlap({1: mask_a, 2: mask_b}) == []
