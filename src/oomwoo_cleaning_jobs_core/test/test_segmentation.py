"""Automatic segmentation tests (synthetic fixtures, headless)."""

import numpy as np

from fixtures import (
    make_open_plan_map,
    make_open_room_map,
    make_rooms_map,
    make_tiny_room_map,
    make_two_rooms_map,
)

from oomwoo_cleaning_jobs_core.segmentation import (
    UNCLASSIFIED,
    SegmentationParams,
    segment,
)


def test_two_rooms_split_by_door():
    """Clean two-room + doorway fixture: exactly 2 normal-confidence candidates."""
    source = make_two_rooms_map()
    result = segment(source)
    assert len(result.regions) == 2
    assert all(not r.low_confidence for r in result.regions)
    # Each region stays within free space, no overlaps, sane area
    # (well above min_region_area)
    covered = np.zeros(source.cells.shape, dtype=bool)
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()
        assert not (mask & covered).any()
        covered |= mask
        assert region.area_m2 > 2.0


def test_regions_stay_within_free_space():
    """No candidate may contain occupied/unknown cells (including the split
    lobes around the in-room unknown patch)."""
    source = make_rooms_map()
    result = segment(source, SegmentationParams(saddle_merge_ratio=99.0))
    assert len(result.regions) >= 2
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()


def test_watershed_ridge_cells_marked_unclassified():
    """Watershed ridge cells are explicitly marked unclassified, and their
    share stays small."""
    source = make_rooms_map()
    result = segment(source)
    unclassified = result.unclassified_free_mask
    assert unclassified.any()
    assert unclassified.sum() < 0.05 * source.free_mask().sum()


def test_watershed_does_not_cross_walls():
    """Regions must not cross the inner wall: the deep interiors (>=2 cells
    from walls) of the left/right rooms must belong to different regions
    (cv2.watershed flooded the whole image and crossed walls, previously).
    Boundary-line wobble within +/-2 cells of the doorway is normal maximin
    flooding behavior and is excluded here."""
    source = make_two_rooms_map()
    result = segment(source)
    left_inner = result.labels[8:72, 5:28]
    right_inner = result.labels[8:72, 32:70]
    left_labels = {int(v) for v in np.unique(left_inner) if v != UNCLASSIFIED}
    right_labels = {int(v) for v in np.unique(right_inner) if v != UNCLASSIFIED}
    assert len(left_labels) == 1
    assert len(right_labels) == 1
    assert left_labels != right_labels


def test_unknown_patch_lobes_area_merged():
    """Small lobes around the unknown patch (each < min_region_area) are
    absorbed by area merging: the two-room fixture with an unknown patch
    yields exactly 2 candidates."""
    result = segment(make_rooms_map())
    assert len(result.regions) == 2


def test_saddle_merge_recombines_wide_opening():
    """Open-plan two zones with a wide opening (26 cells = 1.3 m,
    saddle/peak ratio ~0.87 >= 0.8) are fused into 1 candidate by saddle
    merging; with saddle merging disabled (ratio=99) they stay 2."""
    source = make_open_plan_map(opening_cells=26)
    merged = segment(source)
    unmerged = segment(source, SegmentationParams(saddle_merge_ratio=99.0))
    assert len(unmerged.regions) == 2
    assert len(merged.regions) == 1


def test_door_opening_not_saddle_merged():
    """A real doorway (10 cells = 0.5 m, saddle/peak ratio ~0.33) is not
    fused by saddle merging."""
    result = segment(make_open_plan_map(opening_cells=10))
    assert len(result.regions) == 2


def test_doorway_clip_eliminates_spill():
    """After doorway clipping, regions must not cross the door plane (no
    spill even with a wide 0.9 m door).

    maximin flooding itself assigns cells near the door whose dist is below
    the door saddle to the opposite region; `_clip_doorway_spills` forces
    the boundary onto the doorway cut line."""
    # Two rooms: inner wall col 30, doorway rows 35-44
    source = make_two_rooms_map()
    result = segment(source)
    left = result.labels[40, 15]
    right = result.labels[40, 50]
    assert left != right and left != UNCLASSIFIED and right != UNCLASSIFIED
    # Allow a 1-cell boundary band (ridge unclassified); no left-room cell
    # right of col 33
    assert (result.labels[:, 33:] == left).sum() == 0
    assert (result.labels[:, :28] == right).sum() == 0

    # Wide door 0.9 m: inner wall col 32
    wide = make_open_plan_map(opening_cells=18)
    result = segment(wide)
    assert len(result.regions) == 2
    left = result.labels[17, 16]
    right = result.labels[17, 48]
    assert (result.labels[:, 35:] == left).sum() == 0
    assert (result.labels[:, :30] == right).sum() == 0


def test_open_room_degenerates_to_single_low_confidence_candidate():
    """An open room has a single distance peak: the whole space becomes one
    low-confidence candidate."""
    result = segment(make_open_room_map())
    assert len(result.regions) == 1
    assert result.regions[0].low_confidence
    # The low-confidence candidate covers almost all free space
    coverage = result.mask_of(result.regions[0].label).sum() / result.free_mask.sum()
    assert coverage > 0.95


def test_tiny_room_merged_into_neighbor():
    """A below-min_region_area standalone-peak region merges into the
    neighbor sharing the longest boundary."""
    source = make_tiny_room_map()
    result = segment(source)
    assert len(result.regions) == 1
    region = result.regions[0]
    assert not region.low_confidence
    # After merging, the region covers the small room's interior (most free
    # cells with col > 68 belong to it; watershed ridge cells near the
    # doorway may stay unclassified)
    right_room_free = source.free_mask()[:, 69:78]
    covered = result.mask_of(region.label)[:, 69:78][right_room_free].sum()
    assert covered >= 0.9 * right_room_free.sum()


def test_unknown_cells_never_in_candidates():
    """Unknown cells (including the in-room unknown patch) never enter any
    candidate region."""
    source = make_rooms_map()
    result = segment(source)
    assert not (result.labels[source.unknown_mask()] != UNCLASSIFIED).any()


def test_min_region_area_respected():
    """All final candidates are no smaller than min_region_area."""
    params = SegmentationParams(min_region_area_m2=1.0)
    for make in (make_rooms_map, make_tiny_room_map, make_open_room_map):
        result = segment(make(), params)
        for region in result.regions:
            assert region.area_m2 >= params.min_region_area_m2 * 0.99
