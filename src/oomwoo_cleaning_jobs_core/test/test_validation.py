"""validation severity-grading tests."""

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
    """A Region Set produced by the normal editing path triggers no errors
    (invariants hold by construction)."""
    _, rs = _make_region_set()
    report = validate_region_set(rs)
    assert report.ok
    assert not report.errors


def test_unassigned_cleanable_is_warning_not_error():
    """Unassigned cleanable space is a warning; publishing is still allowed."""
    _, rs = _make_region_set()
    report = validate_region_set(rs)
    assert 'unassigned_cleanable' in _codes(report, LEVEL_WARNING)
    assert report.ok


def test_fully_assigned_set_has_no_unassigned_warning():
    source, rs = _make_region_set()
    # Paint every unassigned cleanable cell into the first Region
    label = rs.regions()[0].label
    rs.paint(label, rs.unassigned_cleanable_mask)
    report = validate_region_set(rs)
    assert 'unassigned_cleanable' not in _codes(report, LEVEL_WARNING)
    assert report.ok


def test_region_outside_cleanable_is_error():
    """Invariant check: hand-edited data pokes a Region cell onto a wall ->
    error."""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    rs.labels[4, 10] = label  # bottom wall (occupied)
    rs.labels[0, 0] = label   # unknown
    report = validate_region_set(rs)
    assert not report.ok
    assert 'region_outside_cleanable' in _codes(report, LEVEL_ERROR)


def test_narrow_region_unreachable_is_error():
    """A 1-cell-wide strip region is empty after footprint erosion -> error."""
    source, rs = _make_region_set()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[40, 5:29] = True  # single row, 0.05 m wide; robot cannot enter
    label = rs.create(stroke, name='Slit')
    assert label is not None
    report = validate_region_set(rs)
    assert not report.ok
    narrow = [i for i in report.errors if i.code == 'region_unreachable']
    assert len(narrow) == 1
    assert narrow[0].region == label


def test_disconnected_core_is_warning():
    """Two roomy pieces joined by a 1-cell narrow throat: reachable core is
    split into two pieces -> warning, not error.

    The "unreachable ratio" metric is deliberately not used: the perimeter
    ring of any room is unreachable to the robot center (~30%), so that
    metric would always be a false positive for normal rooms."""
    source, rs = _make_region_set()
    # Paint on the left room: two 10x10 blocks (0.5 m, each with a reachable
    # core) + a 1-cell narrow throat
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[10:20, 8:18] = True    # block 1
    stroke[10:20, 20:29] = True   # block 2
    stroke[15, 18:20] = True      # narrow throat (0.05 m < robot diameter)
    label = rs.create(stroke, name='Dumbbell')
    assert label is not None
    report = validate_region_set(rs)
    assert report.ok  # warnings do not block publishing
    assert 'region_disconnected_core' in _codes(report, LEVEL_WARNING)
    assert 'region_unreachable' not in _codes(report, LEVEL_ERROR)
    # A normal room's core is one connected piece and must not be flagged
    normal = [i for i in report.warnings if i.code == 'region_disconnected_core']
    assert all(i.region == label for i in normal)


def test_keepout_intersection_is_error():
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    keepout = np.zeros(source.cells.shape, dtype=bool)
    keepout[40, 10:20] = True
    # Keepout intersecting a Region -> error
    assert rs.mask_of(label)[40, 10:20].any()
    report = validate_region_set(rs, keepout_mask=keepout)
    assert not report.ok
    assert 'region_in_keepout' in _codes(report, LEVEL_ERROR)
    # Non-intersecting Keepout -> no error
    keepout2 = np.zeros(source.cells.shape, dtype=bool)
    keepout2[4, 10:20] = True  # on a wall, inside no Region
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
