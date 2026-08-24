import numpy as np

from oomwoo_segmentation.models import SegmentationResult, WallSegment
from oomwoo_segmentation.render import (
    COLOR_FREE,
    COLOR_OCCUPIED,
    COLOR_UNKNOWN,
    render_segmentation,
    render_source_map,
    render_walls,
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


def test_render_walls_draws_support_colored_segments():
    source = make_map()
    base = render_source_map(source)
    # horizontal wall along row 2 (map y = 0.25), full support -> pure red
    wall = WallSegment(x1=0.05, y1=0.25, x2=0.45, y2=0.25,
                       support=1.0, direction_rad=0.0)
    image = render_walls(source, (wall,))

    assert image.shape == base.shape
    assert image.flags.c_contiguous
    # row 2 in cell order is image row height - 1 - 2 = 2 for a 5x5 map;
    # anti-aliasing blends edges, so check the color trend at the midpoint
    b, g, r = (int(v) for v in image[2, 2])
    assert r > 200 and b < 100  # full support trends to red
    # untouched cells keep the base rendering
    assert np.array_equal(image[0, 0], base[0, 0])

    # low support trends to yellow; output stays deterministic
    weak = WallSegment(x1=0.05, y1=0.25, x2=0.45, y2=0.25,
                       support=0.0, direction_rad=0.0)
    weak_image = render_walls(source, (weak,))
    b, g, r = (int(v) for v in weak_image[2, 2])
    assert g > 200 and b < 100  # zero support trends to yellow (high G, low B)
