"""RegionSet editing-semantics tests (see the DEVELOPMENT.md editing-semantics
decision)."""

import math

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
    """Immediate clipping: stroke parts over walls/unknown are clipped away;
    only cleanable space is gained."""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    before = rs.mask_of(label).sum()
    # Stroke spans the right room (free) and the unknown area on the right
    # (cols 75-90)
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[40, 60:90] = True
    assert rs.paint(label, stroke)
    gained = rs.mask_of(label)[:, 60:90]
    # Only free cells are gained (cols 60-69); unknown (cols 71+) must not enter
    assert (gained[:, 11:] == 0).all()
    assert rs.mask_of(label).sum() > before


def test_paint_empty_stroke_is_invalid():
    """Clipped-to-empty means invalid: a stroke entirely on a wall changes
    nothing."""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    before = rs.labels.copy()
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[4, 10:20] = True  # bottom wall (occupied)
    assert not rs.paint(label, stroke)
    assert np.array_equal(rs.labels, before)


def test_paint_preempts_existing_region():
    """Later-painter preemption: a stroke overlapping an existing Region
    transfers the overlapping cells to the new Region."""
    source, rs = _make_region_set()
    a, b = [r.label for r in rs.regions()]
    size_a_before = rs.mask_of(a).sum()
    # b's brush pushes a band into a's territory
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[10:20, 10:25] = True
    band_a = int(rs.mask_of(a)[10:20, 10:25].sum())
    assert band_a > 0
    assert rs.paint(b, stroke)
    lost = size_a_before - rs.mask_of(a).sum()
    assert lost == band_a  # a loses exactly the cells in the band, no more, no less
    assert rs.mask_of(b)[10:20, 10:25].all()
    assert not rs.mask_of(a)[10:20, 10:25].any()


def test_full_preemption_prunes_empty_region_name():
    """When the later painter fully takes over an old Region, no unreachable
    name metadata is kept."""
    _, rs = _make_region_set()
    a, b = [r.label for r in rs.regions()]

    assert rs.paint(b, rs.mask_of(a))

    assert not rs.mask_of(a).any()
    assert a not in rs.names
    assert all(region.label != a for region in rs.regions())


def test_create_and_delete_and_rename():
    source, rs = _make_region_set()
    assert len(rs.regions()) == 2
    stroke = np.zeros(source.cells.shape, dtype=bool)
    stroke[60:70, 40:50] = True
    new_label = rs.create(stroke, name='Balcony')
    assert new_label is not None
    assert len(rs.regions()) == 3
    names = {r.label: r.name for r in rs.regions()}
    assert names[new_label] == 'Balcony'
    # create preemption semantics: this band belonged to a Region, now to the new one
    assert (rs.labels[60:70, 40:50] == new_label).all()
    assert rs.rename(new_label, 'Study')
    assert rs.names[new_label] == 'Study'
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
    # Erasing to empty auto-deletes
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
    """Split by a drawn line: a vertical line across the left room yields two
    pieces; the larger piece keeps the original label."""
    source, rs = _make_region_set()
    a, b = sorted(r.label for r in rs.regions())
    # Left room (cols 5-29), vertical cut at col 15
    cut = np.zeros(source.cells.shape, dtype=bool)
    cut[5:75, 15] = True
    new_labels = rs.split(a, cut)
    assert new_labels is not None
    assert len(new_labels) == 2
    assert new_labels[0] == a  # larger piece keeps the original label
    # Pieces do not overlap; original Region cells are either in a piece or
    # on the cut line / unassigned
    for lb in new_labels:
        assert rs.mask_of(lb).any()
    assert not (rs.mask_of(new_labels[0]) & rs.mask_of(new_labels[1])).any()


def test_split_without_separation_is_invalid():
    """A cut line that does not split the Region into two pieces is invalid."""
    source, rs = _make_region_set()
    a = sorted(r.label for r in rs.regions())[0]
    cut = np.zeros(source.cells.shape, dtype=bool)
    cut[5, 10] = True  # a single point cannot cut through
    before = rs.labels.copy()
    assert rs.split(a, cut) is None
    assert np.array_equal(rs.labels, before)


def test_outline_derives_polygon_in_map_frame():
    """Outline derived from mask: the outer ring in map frame (meters)
    roughly equals the room boundary."""
    source, rs = _make_region_set()
    label = rs.regions()[0].label
    rings = rs.outline(label)
    assert rings
    outer = rings[0]
    res = source.resolution
    ox, oy = source.origin[0], source.origin[1]
    # Room spans cells rows 5-74, cols 5-29 -> metric boundary check
    assert outer[:, 0].min() >= ox + 4 * res
    assert outer[:, 0].max() <= ox + 31 * res
    assert outer[:, 1].min() >= oy + 4 * res
    assert outer[:, 1].max() <= oy + 76 * res


def test_outline_honors_map_origin_yaw():
    labels = np.zeros((10, 10), dtype=np.int32)
    labels[2:4, 4:6] = 1
    region_set = RegionSet(
        labels=labels,
        cleanable=np.ones(labels.shape, dtype=bool),
        resolution=1.0,
        origin=(10.0, 20.0, math.pi / 2),
        names={1: 'Rotated Area'},
    )

    outline = region_set.outline(1)[0]

    # Local cells cols 4..5, rows 2..3 after a 90-degree rotation:
    # x = 10 - local_y, y = 20 + local_x.
    assert np.isclose(outline[:, 0].min(), 6.5)
    assert np.isclose(outline[:, 0].max(), 7.5)
    assert np.isclose(outline[:, 1].min(), 24.5)
    assert np.isclose(outline[:, 1].max(), 25.5)


def test_unassigned_cleanable_mask():
    _, rs = _make_region_set()
    # Two Regions plus unclassified ridges: unassigned should be non-empty
    # and entirely cleanable
    unassigned = rs.unassigned_cleanable_mask
    assert (unassigned <= rs.cleanable).all()
