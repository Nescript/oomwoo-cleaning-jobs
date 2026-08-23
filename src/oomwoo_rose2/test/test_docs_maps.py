"""Regression coverage for maps that exposed upstream ROSE/ROSE2 crashes."""

from pathlib import Path

import cv2
import numpy as np
import pytest

pytest.importorskip('skimage')
pytest.importorskip('sklearn')
pytest.importorskip('shapely')

from oomwoo_rose2.engine import Rose2Segmenter
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_segmentation.validation import validate_result

_REPOSITORY = Path(__file__).resolve().parents[3]


def _source_from_render(relative_path: str, embedded_scale: int) -> SourceMap:
    pixels = cv2.imread(
        str(_REPOSITORY / relative_path), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise RuntimeError(f'cannot load docs fixture {relative_path}')
    pixels = pixels[::embedded_scale, ::embedded_scale]
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[pixels >= 250] = FREE
    cells[pixels <= 10] = OCCUPIED
    cells = np.ascontiguousarray(cells[::-1, :])
    height, width = cells.shape
    return SourceMap(0.05, width, height, (0.0, 0.0, 0.0), cells)


@pytest.mark.slow
@pytest.mark.parametrize(
    ('relative_path', 'embedded_scale', 'expected_rooms', 'expected_unassigned'), (
        ('docs/demo/corridor4.render.png', 3, 5, 0),
        ('docs/demo/grid6_furniture.render.png', 3, 5, 0),
        ('docs/demo/living_room.render.png', 2, 2, 0),
        ('docs/demo/room3.png', 1, 5, 0),
        ('docs/demo/two_rooms.render.png', 1, 2, 0),
    ))
def test_docs_map_produces_a_valid_result(
    relative_path, embedded_scale, expected_rooms, expected_unassigned,
):
    source = _source_from_render(relative_path, embedded_scale)

    result = Rose2Segmenter().segment(source, include_diagnostics=True)

    validate_result(result, source)
    assert len(result.regions) == expected_rooms
    assert int(result.unassigned_cleanable_mask.sum()) == expected_unassigned
    assert not np.any(result.labels[~source.free_mask()])
    assert {item.stage for item in result.diagnostics} == {
        'cleaned_map', 'extended_lines', 'labels_overlay'}
