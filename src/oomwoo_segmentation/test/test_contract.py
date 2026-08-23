import numpy as np
import pytest

from oomwoo_segmentation.models import SegmentationError, SegmentationResult
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
