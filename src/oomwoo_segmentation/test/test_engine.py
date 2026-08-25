from types import SimpleNamespace

import numpy as np
import pytest

from oomwoo_segmentation.engine import SegmentationConfig, SegmentationEngine
from oomwoo_segmentation.engine.postprocessing import geodesic_coverage, rasterize_rooms
from oomwoo_segmentation.models import SegmentationError
from oomwoo_segmentation.source_map import SourceMap
from oomwoo_segmentation.validation import canonicalize_labels


class Point:
    def __init__(self, x, y):
        self.x, self.y = x, y


class Ring:
    def __init__(self, coords):
        self.coords = coords


class Polygon:
    geom_type = 'Polygon'
    is_empty = False

    def __init__(self, exterior, *, holes=(), centroid=(0, 0), area=1.0):
        self.exterior = Ring(exterior)
        self.interiors = tuple(Ring(hole) for hole in holes)
        self.centroid = Point(*centroid)
        self.area = area


def make_map():
    cells = np.full((10, 12), 100, dtype=np.int8)
    cells[1:9, 1:11] = 0
    cells[4, 4] = -1
    return SourceMap(0.1, 12, 10, (0.0, 0.0, 0.0), cells)


def test_launch_profile_parameter_validation():
    config = SegmentationConfig()
    config.validate()
    assert config.lines_threshold == pytest.approx(0.22)
    assert config.hard_wall_threshold == pytest.approx(0.40)
    assert config.min_component_size == 10
    assert config.enable_geodesic_coverage is True
    with pytest.raises(SegmentationError, match='between 0 and 1'):
        SegmentationConfig(filter_level=1.1).validate()
    with pytest.raises(SegmentationError, match='between 0 and 1'):
        SegmentationConfig(hard_wall_threshold=1.5).validate()
    with pytest.raises(SegmentationError, match='non-negative'):
        SegmentationConfig(min_component_size=-1).validate()


def test_geodesic_coverage_fills_unassigned_free_cells():
    source = make_map()
    cleanable = source.free_mask()
    labels = np.zeros((10, 12), dtype=np.int32)
    # Seed region 1 in the upper half and region 2 in lower half
    labels[1:3, 1:11] = 1
    labels[7:9, 1:11] = 2

    filled_labels, final_cleanable = geodesic_coverage(
        labels, source, cleanable, min_component_size=5
    )
    # Check that all free cleanable space is assigned to either 1 or 2
    assert not np.any(final_cleanable & (filled_labels == 0))
    assert filled_labels[2, 5] == 1
    assert filled_labels[8, 5] == 2
    assert filled_labels[0, 0] == 0  # wall/boundary remains 0


def test_source_image_matches_occupancy_conversion_and_mask():
    source = make_map()
    cleanable = source.free_mask()
    cleanable[2, 2] = False
    image = SegmentationEngine._source_image(source, cleanable)
    assert image[1, 1] == 255
    assert image[4, 4] == 200
    assert image[0, 0] == 0
    assert image[2, 2] == 0


def test_room_polygons_are_stably_rasterized_with_holes():
    upper = Polygon(
        [(6, 5), (10, 5), (10, 8), (6, 8)],
        centroid=(8, 6), area=12,
    )
    lower = Polygon(
        [(1, 1), (5, 1), (5, 4), (1, 4)],
        holes=([(2, 2), (3, 2), (3, 3), (2, 3)],),
        centroid=(3, 2), area=11,
    )
    labels = rasterize_rooms([upper, lower], (10, 12))
    assert labels[1, 1] == 1
    assert labels[2, 2] == 0
    assert labels[6, 8] == 2

    source = make_map()
    canonical, regions, _cleanable = canonicalize_labels(labels, source, None)
    assert [region.label for region in regions] == [1, 2]
    assert canonical[0, 0] == 0


def test_cancellation_during_segmentation():
    engine = SegmentationEngine()
    with pytest.raises(SegmentationError, match='cancelled'):
        engine.segment(
            make_map(),
            cancelled=lambda: True,
        )
