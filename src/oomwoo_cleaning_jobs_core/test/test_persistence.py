"""Tests for local persistence of draft / published Region Sets."""

import cv2
import numpy as np
import pytest
import yaml

from fixtures import make_two_rooms_map

from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout
from oomwoo_cleaning_jobs_core.persistence import RegionSetStore
from oomwoo_cleaning_jobs_core.regions import RegionSet
from oomwoo_cleaning_jobs_core.segmentation import segment


def _draft(source, constraints=ConstraintSet()):
    keepout = constraints.mask_for(source)
    result = segment(source, cleanable_mask=source.free_mask() & ~keepout)
    return RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin,
        base_cleanable=source.free_mask(), keepout_mask=keepout)


def test_draft_round_trip_writes_snapshot_masks_and_constraints(tmp_path):
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        keepouts=(Keepout('table', ((-2.0, 0.0), (-1.8, 0.0),
                                    (-1.8, 0.2), (-2.0, 0.2))),))
    region_set = _draft(source, constraints)
    store = RegionSetStore(tmp_path)

    draft_path = store.save_draft(source, region_set, constraints)
    loaded = store.load_draft(source)

    assert (draft_path.parent / 'map_snapshot.yaml').is_file()
    assert (draft_path.parent / 'map_snapshot.pgm').is_file()
    assert draft_path.is_symlink()
    assert (draft_path / 'regions.yaml').is_file()
    assert len(list((draft_path / 'masks').glob('*.png'))) == len(region_set.regions())
    assert loaded is not None
    assert np.array_equal(loaded.region_set.labels, region_set.labels)
    assert loaded.region_set.names == region_set.names
    assert loaded.constraints == constraints


def test_publish_versions_and_replaces_single_published_set(tmp_path):
    source = make_two_rooms_map()
    store = RegionSetStore(tmp_path)
    region_set = _draft(source)

    first = store.publish(source, region_set, ConstraintSet())
    second = store.publish(source, region_set, ConstraintSet())
    loaded = store.load_published(source)

    assert first.version == 1
    assert second.version == 2
    assert first.published_at
    assert loaded is not None
    assert loaded.version == 2
    assert (tmp_path / source.identity / 'published').is_dir()


def test_load_rejects_map_identity_mismatch_and_overlapping_masks(tmp_path):
    source = make_two_rooms_map()
    store = RegionSetStore(tmp_path)
    region_set = _draft(source)
    path = store.save_draft(source, region_set, ConstraintSet())

    changed = source.cells.copy()
    changed[10, 10] = 100
    changed_map = type(source)(source.resolution, source.width, source.height,
                               source.origin, changed)
    assert store.load_draft(changed_map) is None  # identity maps to a different directory

    metadata_path = path / 'regions.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    first, second = metadata['regions'][:2]
    second_mask = path / second['mask']
    first_image = cv2.imread(str(path / first['mask']), cv2.IMREAD_GRAYSCALE)
    assert cv2.imwrite(str(second_mask), first_image)
    with pytest.raises(ValueError, match='overlap'):
        store.load_draft(source)


def test_snapshot_keeps_lossless_raw_cells_sidecar(tmp_path):
    source = make_two_rooms_map()
    cells = source.cells.copy()
    cells[10, 10] = 10  # non-trinary raw OccupancyGrid cell that changes identity
    source = type(source)(source.resolution, source.width, source.height,
                          source.origin, cells)
    store = RegionSetStore(tmp_path)

    store.save_draft(source, _draft(source), ConstraintSet())

    raw = np.load(tmp_path / source.identity / 'map_snapshot.cells.npy')
    assert np.array_equal(raw, source.cells)


def test_load_published_revalidates_after_on_disk_edit(tmp_path):
    source = make_two_rooms_map()
    store = RegionSetStore(tmp_path)
    store.publish(source, _draft(source), ConstraintSet())
    published = tmp_path / source.identity / 'published'
    metadata = yaml.safe_load((published / 'regions.yaml').read_text(encoding='utf-8'))
    image_path = published / metadata['regions'][0]['mask']
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    image[source.height - 1 - 4, 10] = 255  # SourceMap row 4 is the occupied outer wall
    assert cv2.imwrite(str(image_path), image)

    with pytest.raises(ValueError, match='region_outside_cleanable'):
        store.load_published(source)


def test_publish_refuses_validation_errors(tmp_path):
    source = make_two_rooms_map()
    region_set = RegionSet(
        labels=np.zeros(source.cells.shape, dtype=np.int32),
        cleanable=source.free_mask(), resolution=source.resolution,
        origin=source.origin,
    )

    with pytest.raises(ValueError, match='empty_region_set'):
        RegionSetStore(tmp_path).publish(source, region_set, ConstraintSet())
