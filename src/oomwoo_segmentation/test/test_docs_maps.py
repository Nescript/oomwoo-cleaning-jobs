"""Regression coverage for benchmark maps using the native segmentation engine."""

from pathlib import Path

import cv2
import numpy as np
import pytest

pytest.importorskip('skimage')
pytest.importorskip('sklearn')
pytest.importorskip('shapely')

from oomwoo_segmentation.engine import SegmentationEngine
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_segmentation.validation import validate_result

_MAPS_DIR = Path(__file__).resolve().parent / 'maps' / 'demo'
_IPA_MAPS_DIR = Path(__file__).resolve().parent / 'maps' / 'ipa'


def _source_from_render(filename: str, embedded_scale: int) -> SourceMap:
    path = _MAPS_DIR / filename
    pixels = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise RuntimeError(f'cannot load maps fixture {path}')
    pixels = pixels[::embedded_scale, ::embedded_scale]
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[pixels >= 250] = FREE
    cells[pixels <= 10] = OCCUPIED
    cells = np.ascontiguousarray(cells[::-1, :])
    height, width = cells.shape
    return SourceMap(0.05, width, height, (0.0, 0.0, 0.0), cells)


@pytest.mark.slow
@pytest.mark.parametrize(
    ('filename', 'embedded_scale', 'expected_rooms', 'expected_unassigned'), (
        ('corridor4.render.png', 3, 5, 0),
        ('grid6_furniture.render.png', 3, 6, 0),
        ('living_room.render.png', 2, 1, 0),
        ('room3.png', 1, 6, 0),
        ('room4.png', 1, 5, 0),
        ('two_rooms.render.png', 1, 2, 0),
    ))
def test_docs_map_produces_a_valid_result(
    filename, embedded_scale, expected_rooms, expected_unassigned,
):
    source = _source_from_render(filename, embedded_scale)

    result = SegmentationEngine().segment(source, include_diagnostics=True)

    validate_result(result, source)
    assert len(result.regions) == expected_rooms
    assert int(result.unassigned_cleanable_mask.sum()) == expected_unassigned
    assert not np.any(result.labels[~source.free_mask()])
    assert {item.stage for item in result.diagnostics} == {
        '01_cleaned_map', '02_extended_lines', '03_labels_overlay'}


@pytest.mark.slow
@pytest.mark.parametrize(
    ('filename', 'embedded_scale', 'expected_walls'), (
        ('corridor4.render.png', 3, 7),
        ('room4.png', 1, 10),
        ('two_rooms.render.png', 1, 8),
    ))
def test_docs_map_exposes_detected_walls(
    filename, embedded_scale, expected_walls,
):
    source = _source_from_render(filename, embedded_scale)

    result = SegmentationEngine().segment(source)

    assert len(result.walls) == expected_walls
    assert all(wall.support > 0.0 for wall in result.walls)
    supports = [wall.support for wall in result.walls]
    assert supports == sorted(supports, reverse=True)


# Additional review cases from ipa_coverage_planning
# (ipa_room_segmentation/common/files/test_maps/, see THIRD_PARTY.md).
# No ground-truth room counts are asserted; the contract is that every map
# segments into at least one valid room that covers all cleanable cells.
# Freiburg52_scan is excluded: upstream ROSE itself fails on it (its FFT
# preprocessing raises on the nearly wall-less map), so it is a batch-review
# case only.
@pytest.mark.slow
@pytest.mark.parametrize('filename', (
    'lab_a.png',
    'lab_ipa.png',
    'office_a.png',
    'office_h.png',
    'NLB.png',
    'office_a_furnitures.png',
    'lab_d_scan_furnitures.png',
))
def test_ipa_map_produces_a_valid_result(filename):
    path = _IPA_MAPS_DIR / filename
    pixels = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise RuntimeError(f'cannot load maps fixture {path}')
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[pixels >= 250] = FREE
    cells[pixels <= 10] = OCCUPIED
    cells = np.ascontiguousarray(cells[::-1, :])
    height, width = cells.shape
    source = SourceMap(0.05, width, height, (0.0, 0.0, 0.0), cells)

    result = SegmentationEngine().segment(source)

    validate_result(result, source)
    assert len(result.regions) >= 1
    assert not np.any(result.labels[~source.free_mask()])
