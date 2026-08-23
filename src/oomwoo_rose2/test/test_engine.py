import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from oomwoo_rose2.engine import Rose2Config, Rose2Segmenter
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
    config = Rose2Config()
    config.validate()
    assert config.lines_threshold == pytest.approx(0.22)
    with pytest.raises(SegmentationError, match='between 0 and 1'):
        Rose2Config(filter_level=1.1).validate()


def test_source_image_matches_upstream_occupancy_conversion_and_mask():
    source = make_map()
    cleanable = source.free_mask()
    cleanable[2, 2] = False
    image = Rose2Segmenter._source_image(source, cleanable)
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
    labels = Rose2Segmenter._rasterize_rooms([upper, lower], (10, 12))
    assert labels[1, 1] == 1
    assert labels[2, 2] == 0
    assert labels[6, 8] == 2

    source = make_map()
    canonical, regions, _cleanable = canonicalize_labels(labels, source, None)
    assert [region.label for region in regions] == [1, 2]
    assert canonical[0, 0] == 0


def test_cancellation_after_main_extraction_does_not_return_success(monkeypatch):
    cancellation_requested = {'value': False}

    class FakeRose:
        def __init__(self, image, **_kwargs):
            self.analysed_map = image
            self.main_directions = [0.0, 90.0, 180.0]

        def process_map(self):
            pass

        def simple_filter_map(self, _level):
            pass

        def generate_initial_hypothesis_simple(self):
            pass

        def find_walls_flood_filing(self):
            pass

    class FakeParameters:
        class ParameterObj:
            pass

    class FakeBatch:
        def start_main(self, *_args, **_kwargs):
            self.rooms_th1 = [Polygon(
                [(1, 1), (10, 1), (10, 8), (1, 8)],
                centroid=(5, 4), area=63,
            )]
            cancellation_requested['value'] = True

    rose_v1 = ModuleType('rose_v1_repo')
    rose_v1.__path__ = []
    fft_module = ModuleType('rose_v1_repo.fft_structure_extraction')
    fft_module.FFTStructureExtraction = FakeRose
    rose_v2 = ModuleType('rose_v2_repo')
    rose_v2.__path__ = []
    rose_v2.minibatch = SimpleNamespace(Minibatch=FakeBatch)
    rose_v2.parameters = FakeParameters
    monkeypatch.setitem(sys.modules, 'rose_v1_repo', rose_v1)
    monkeypatch.setitem(
        sys.modules, 'rose_v1_repo.fft_structure_extraction', fft_module)
    monkeypatch.setitem(sys.modules, 'rose_v2_repo', rose_v2)

    with pytest.raises(SegmentationError, match='cancelled'):
        Rose2Segmenter().segment(
            make_map(),
            cancelled=lambda: cancellation_requested['value'],
        )
