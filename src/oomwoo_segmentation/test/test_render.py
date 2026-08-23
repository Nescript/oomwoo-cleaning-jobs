import numpy as np

from oomwoo_segmentation.models import SegmentationResult
from oomwoo_segmentation.render import (
    COLOR_FREE,
    COLOR_OCCUPIED,
    COLOR_UNKNOWN,
    render_segmentation,
    render_source_map,
)
from oomwoo_segmentation.source_map import SourceMap
from oomwoo_segmentation.validation import canonicalize_labels


def make_map():
    cells = np.array([
        [100, 100, 100, 100, 100],
        [100,   0,   0,   0, 100],
        [100,   0,  -1,   0, 100],
        [100,   0,   0,   0, 100],
        [100, 100, 100, 100, 100],
    ], dtype=np.int8)
    return SourceMap(0.1, 5, 5, (0.0, 0.0, 0.0), cells)


def test_source_map_colors_match_masks():
    source = make_map()
    image = render_source_map(source)
    for color, count in (
        (COLOR_FREE, source.free_mask().sum()),
        (COLOR_OCCUPIED, source.occupied_mask().sum()),
        (COLOR_UNKNOWN, source.unknown_mask().sum()),
    ):
        assert (image == color).all(axis=2).sum() == count


def test_segmentation_render_is_scaled_and_numbered():
    source = make_map()
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    labels[1:4, 1:2] = 3
    labels[1:4, 3:4] = 8
    canonical, regions, cleanable = canonicalize_labels(labels, source, None)
    result = SegmentationResult(canonical, regions, cleanable, 'test', '1')
    image = render_segmentation(source, result, scale=2)
    assert image.shape == (10, 10, 3)
    assert image.flags.c_contiguous
    assert len(np.unique(image.reshape(-1, 3), axis=0)) > 4
