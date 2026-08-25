import numpy as np

from oomwoo_segmentation.models import DiagnosticImage, SegmentationResult, WallSegment
from oomwoo_segmentation.ros_conversions import (
    array_from_mask_image,
    image_from_mask,
    occupancy_grid_from_source_map,
    result_from_ros_messages,
    result_to_ros_messages,
    source_map_from_occupancy_grid,
)
from oomwoo_segmentation.source_map import SourceMap
from oomwoo_segmentation.validation import canonicalize_labels, validate_result


def make_map():
    cells = np.full((5, 7), 100, dtype=np.int8)
    cells[1:4, 1:6] = 0
    cells[2, 3] = -1
    return SourceMap(0.05, 7, 5, (-1.0, 2.0, 0.3), cells)


def test_occupancy_grid_round_trip_preserves_identity():
    source = make_map()
    restored = source_map_from_occupancy_grid(occupancy_grid_from_source_map(source))
    assert restored.identity == source.identity
    assert restored == source


def test_mask_image_round_trip():
    source = make_map()
    mask = source.free_mask()
    mask[1, 1] = False
    restored = array_from_mask_image(image_from_mask(mask))
    assert np.array_equal(restored, mask)


def test_result_messages_round_trip():
    source = make_map()
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    labels[1:4, 1:3] = 1
    labels[1:4, 4:6] = 2
    canonical, regions, cleanable = canonicalize_labels(labels, source, None)
    diagnostic = np.zeros((4, 6, 3), dtype=np.uint8)
    wall = WallSegment(
        x1=-0.9, y1=2.15, x2=-0.8, y2=2.15, support=0.8, direction_rad=0.0)
    result = SegmentationResult(
        canonical, regions, cleanable, 'test-provider', '1.2.3',
        walls=(wall,),
        diagnostics=(DiagnosticImage('stage', diagnostic),),
    )

    labels_msg, rooms_msg, walls_msg, diagnostics_msg = result_to_ros_messages(result, source)
    restored = result_from_ros_messages(
        labels_msg, rooms_msg, walls_msg, diagnostics_msg, source, None, 'test-provider', '1.2.3')

    validate_result(restored, source)
    assert np.array_equal(restored.labels, result.labels)
    assert len(restored.regions) == len(result.regions)
    assert restored.walls == (wall,)
    assert restored.diagnostics[0].stage == 'stage'
    assert np.array_equal(restored.diagnostics[0].image, diagnostic)
