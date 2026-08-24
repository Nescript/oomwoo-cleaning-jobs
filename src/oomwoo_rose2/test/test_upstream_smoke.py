import numpy as np
import pytest

pytest.importorskip('skimage')
pytest.importorskip('sklearn')
pytest.importorskip('shapely')

from oomwoo_rose2.engine import Rose2Segmenter, _bootstrap_upstream

_bootstrap_upstream()
from object.Segment import Segment, structural_raster_support
from object.Surface import Cell
from shapely.geometry import box
from util.layout import remove_frame_fringes
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_segmentation.validation import validate_result


def test_structural_raster_support_samples_perpendicular_neighbors():
    structural = np.zeros((8, 12), dtype=bool)
    structural[3, :4] = True
    structural[3, 6:10] = True
    edge = Segment(0.0, 3.4, 9.0, 3.8)

    assert structural_raster_support(edge, structural) == pytest.approx(8 / 11)


def test_remove_frame_fringes_requires_strong_long_wall_and_small_frame_cell():
    shared_wall = Segment(0.0, 10.0, 100.0, 10.0)
    shared_wall.set_weight(0.95)
    small = Cell([shared_wall])
    interior = Cell([shared_wall])
    polygons = [box(0.0, 0.0, 100.0, 10.0), box(0.0, 10.0, 100.0, 100.0)]

    cells, kept_polygons = remove_frame_fringes(
        [small, interior], polygons, (0.0, 0.0, 100.0, 100.0))

    assert cells == [interior]
    assert kept_polygons == [polygons[1]]

    shared_wall.set_weight(0.89)
    cells, kept_polygons = remove_frame_fringes(
        [small, interior], polygons, (0.0, 0.0, 100.0, 100.0))

    assert cells == [small, interior]
    assert kept_polygons == polygons


def make_two_room_map():
    width, height = 100, 80
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    cells[5:75, 5:70] = FREE
    cells[4, 4:71] = OCCUPIED
    cells[75, 4:71] = OCCUPIED
    cells[4:76, 4] = OCCUPIED
    cells[4:76, 70] = OCCUPIED
    cells[5:75, 30] = OCCUPIED
    cells[35:45, 30] = FREE
    return SourceMap(0.05, width, height, (-2.5, -2.0, 0.0), cells)


@pytest.mark.slow
def test_pinned_upstream_pipeline_smoke():
    source = make_two_room_map()
    result = Rose2Segmenter().segment(source, include_diagnostics=True)
    validate_result(result, source)
    assert result.implementation_id == 'rose2'
    assert len(result.regions) >= 1
    assert not np.any(result.labels[~source.free_mask()])
    assert {item.stage for item in result.diagnostics} == {
        'cleaned_map', 'extended_lines', 'labels_overlay'}
