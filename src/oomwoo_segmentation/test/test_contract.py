import numpy as np
import pytest

from oomwoo_segmentation.models import SegmentationError, SegmentationResult, WallSegment
from oomwoo_segmentation.source_map import SourceMap
from oomwoo_segmentation.validation import canonicalize_labels, validate_result


def make_map():
    cells = np.full((6, 8), 100, dtype=np.int8)
    cells[1:5, 1:7] = 0
    return SourceMap(0.1, 8, 6, (0.0, 0.0, 0.0), cells)


def test_canonicalize_clips_and_stably_relabels():
    source = make_map()
    labels = np.zeros(source.cells.shape, dtype=np.int64)
    labels[1:3, 1:3] = 9
    labels[3:5, 5:7] = 4
    labels[0, 0] = 8  # occupied: must be clipped

    canonical, regions, cleanable = canonicalize_labels(labels, source, None)

    assert canonical.dtype == np.int32
    assert set(np.unique(canonical)) == {0, 1, 2}
    assert canonical[1, 1] == 1
    assert canonical[3, 5] == 2
    assert canonical[0, 0] == 0
    assert [region.cell_count for region in regions] == [4, 4]
    assert cleanable.shape == source.cells.shape


def test_cleanable_mask_cannot_enable_occupied_cells():
    source = make_map()
    requested = np.ones(source.cells.shape, dtype=bool)
    labels = np.ones(source.cells.shape, dtype=np.int32)
    canonical, _regions, cleanable = canonicalize_labels(labels, source, requested)
    assert not cleanable[0, 0]
    assert canonical[0, 0] == 0


def test_rejects_wrong_shape_and_negative_labels():
    source = make_map()
    with pytest.raises(SegmentationError, match='shape'):
        canonicalize_labels(np.zeros((2, 2), dtype=np.int32), source, None)
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    labels[1, 1] = -1
    with pytest.raises(SegmentationError, match='non-negative'):
        canonicalize_labels(labels, source, None)


def test_validate_result_checks_metadata():
    source = make_map()
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    labels[1:3, 1:3] = 1
    canonical, regions, cleanable = canonicalize_labels(labels, source, None)
    result = SegmentationResult(
        canonical, regions, cleanable, 'test', '1.0.0')
    validate_result(result, source)


def test_validate_result_rejects_negative_labels():
    source = make_map()
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    labels[1, 1] = -1
    result = SegmentationResult(
        labels, (), source.free_mask(), 'malformed', '1.0.0')

    with pytest.raises(SegmentationError, match='non-negative'):
        validate_result(result, source)


def test_validate_result_rejects_cleanable_cells_outside_source_free_space():
    source = make_map()
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    cleanable = np.ones(source.cells.shape, dtype=bool)
    result = SegmentationResult(
        labels, (), cleanable, 'malformed', '1.0.0')

    with pytest.raises(SegmentationError, match='source free space'):
        validate_result(result, source)


def _valid_result(source, walls=()):
    return SegmentationResult(
        np.zeros(source.cells.shape, dtype=np.int32),
        (),
        source.free_mask(),
        'test',
        '1.0.0',
        walls=walls,
    )


def test_validate_result_accepts_walls_inside_map():
    source = make_map()  # 8x6 cells @ 0.1, origin (0, 0, 0)
    wall = WallSegment(x1=0.05, y1=0.25, x2=0.75, y2=0.25,
                       support=1.0, direction_rad=0.0)
    validate_result(_valid_result(source, (wall,)), source)


def test_validate_result_rejects_malformed_walls():
    source = make_map()
    base = dict(x1=0.05, y1=0.25, x2=0.75, y2=0.25,
                support=1.0, direction_rad=0.0)
    with pytest.raises(SegmentationError, match='finite'):
        validate_result(_valid_result(
            source, (WallSegment(**{**base, 'x1': float('nan')}),)), source)
    with pytest.raises(SegmentationError, match='support'):
        validate_result(_valid_result(
            source, (WallSegment(**{**base, 'support': 1.5}),)), source)
    with pytest.raises(SegmentationError, match='direction'):
        validate_result(_valid_result(
            source, (WallSegment(**{**base, 'direction_rad': -0.1}),)), source)
    with pytest.raises(SegmentationError, match='bounds'):
        validate_result(_valid_result(
            source, (WallSegment(**{**base, 'x2': 5.0}),)), source)


def test_map_frame_pixel_round_trip_with_yaw():
    source = SourceMap(0.05, 40, 30, (1.0, -2.0, 0.7),
                       np.zeros((30, 40), dtype=np.int8))
    for px, py in ((0.0, 0.0), (39.0, 29.0), (12.3, 4.7)):
        x, y = source.map_frame_from_pixel(px, py)
        back_px, back_py = source.pixel_from_map_frame(x, y)
        assert back_px == pytest.approx(px)
        assert back_py == pytest.approx(py)
