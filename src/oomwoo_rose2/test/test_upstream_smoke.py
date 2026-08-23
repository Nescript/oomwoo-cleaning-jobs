import numpy as np
import pytest

pytest.importorskip('skimage')
pytest.importorskip('sklearn')
pytest.importorskip('shapely')

from oomwoo_rose2.engine import Rose2Segmenter
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_segmentation.validation import validate_result


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
