"""Tests for local persistence of draft / published Region Sets."""

import math

import cv2
import numpy as np
import pytest
import yaml

from fixtures import fake_segmentation, make_two_rooms_map

from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout, SpotArea, VirtualWall
from oomwoo_cleaning_jobs_core.persistence import PublishError, RegionSetStore
from oomwoo_cleaning_jobs_core.regions import RegionSet


def _map_point(source, row, col):
    """Map-frame center point of a given cell, honoring the SourceMap yaw."""
    x, y, yaw = source.origin
    local_x = (col + 0.5) * source.resolution
    local_y = (row + 0.5) * source.resolution
    return (
        x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
        y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
    )


def _sealing_constraints(source):
    return ConstraintSet(
        virtual_walls=(VirtualWall('door-seal', _map_point(source, 34, 30),
                                   _map_point(source, 45, 30), source.resolution),))


def _draft(source, constraints=ConstraintSet()):
    keepout = constraints.mask_for(source)
    result = fake_segmentation(
        source, cleanable_mask=source.free_mask() & ~keepout)
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


def test_spot_area_persists_in_constraints_round_trip(tmp_path):
    source = make_two_rooms_map()
    spot = SpotArea.from_box(center=(1.0, 2.0), width_m=0.6, height_m=0.8,
                             identifier='sp1', name='Kitchen Sink')
    constraints = ConstraintSet(
        keepouts=(Keepout('table', ((-2.0, 0.0), (-1.8, 0.0),
                                    (-1.8, 0.2), (-2.0, 0.2))),),
        spot_area=spot,
    )
    region_set = _draft(source, constraints)
    store = RegionSetStore(tmp_path)

    # Test draft persistence
    store.save_draft(source, region_set, constraints)
    loaded_draft = store.load_draft(source)
    assert loaded_draft is not None
    assert loaded_draft.constraints.spot_area == spot

    # Test published persistence
    store.publish(source, region_set, constraints)
    loaded_pub = store.load_published(source)
    assert loaded_pub is not None
    assert loaded_pub.constraints.spot_area == spot


def test_publish_writes_nav2_compatible_keepout_mask(tmp_path):
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        keepouts=(Keepout('table', ((-2.0, 0.0), (-1.8, 0.0),
                                    (-1.8, 0.2), (-2.0, 0.2))),))
    store = RegionSetStore(tmp_path)
    store.publish(source, _draft(source, constraints), constraints)

    published = tmp_path / source.identity / 'published'
    image_path = published / 'keepout_mask.pgm'
    yaml_path = published / 'keepout_mask.yaml'
    assert image_path.is_file() and yaml_path.is_file()

    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    assert metadata['image'] == 'keepout_mask.pgm'
    assert metadata['resolution'] == source.resolution
    assert metadata['origin'] == list(source.origin)
    assert metadata['negate'] == 0
    assert metadata['mode'] == 'trinary'

    # Read back with the Nav2 trinary convention: occ = 1 - color / 255.
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)[::-1, :]
    occ = 1.0 - image.astype(np.float64) / 255.0
    loaded_mask = occ >= metadata['occupied_thresh']
    assert np.array_equal(loaded_mask, constraints.mask_for(source))
    # Non-constraint cells must read as free, never as unknown.
    free_occ = occ[~constraints.mask_for(source)]
    assert (free_occ <= metadata['free_thresh']).all()


def test_draft_writes_keepout_mask_for_preview(tmp_path):
    source = make_two_rooms_map()
    store = RegionSetStore(tmp_path)
    draft = store.save_draft(source, _draft(source), ConstraintSet())
    assert (draft / 'keepout_mask.pgm').is_file()
    assert (draft / 'keepout_mask.yaml').is_file()


def test_publish_records_margin_and_seed_pose_in_metadata(tmp_path):
    source = make_two_rooms_map()
    store = RegionSetStore(tmp_path)
    seed = _map_point(source, 40, 15)
    store.publish(source, _draft(source), ConstraintSet(),
                  seed_pose=seed, keepout_margin_m=0.1)
    published = tmp_path / source.identity / 'published'
    metadata = yaml.safe_load((published / 'regions.yaml').read_text(encoding='utf-8'))
    assert metadata['keepout_margin_m'] == 0.1
    assert metadata['seed_pose'] == [pytest.approx(seed[0]), pytest.approx(seed[1])]


def test_publish_with_seed_pose_blocks_enclosure(tmp_path):
    source = make_two_rooms_map()
    constraints = _sealing_constraints(source)
    store = RegionSetStore(tmp_path)

    with pytest.raises(ValueError, match='region_enclosed'):
        store.publish(source, _draft(source, constraints), constraints,
                      seed_pose=_map_point(source, 40, 15))

    # The validation failure happens before any write: no published set exists.
    assert not (tmp_path / source.identity / 'published').exists()

    # Without a seed the same content publishes fine (global semantics).
    store.publish(source, _draft(source, constraints), constraints)
    assert (tmp_path / source.identity / 'published').is_dir()


def test_publish_rolls_back_when_post_publish_hook_fails(tmp_path):
    source = make_two_rooms_map()
    store = RegionSetStore(tmp_path)
    store.publish(source, _draft(source), ConstraintSet())

    def failing_hook(_target):
        raise RuntimeError('mask publisher unreachable')

    with pytest.raises(PublishError, match='rolled back'):
        store.publish(source, _draft(source), ConstraintSet(),
                      post_publish_hook=failing_hook)

    loaded = store.load_published(source)
    assert loaded is not None
    assert loaded.version == 1  # pointer restored to the first generation

    # A subsequent publish without a failing hook recovers cleanly.
    recovered = store.publish(source, _draft(source), ConstraintSet())
    assert recovered.version == 2

