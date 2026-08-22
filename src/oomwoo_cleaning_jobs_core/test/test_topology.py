"""Doorway records and topology (hybrid approach) tests.

Region generation is still maximin flooding + merge tree; doorway records
come from merge-tree saddles and carry center (saddle cell), width_m
(~ 2x clearance approximation, not measured), and clearance_m (saddle
height).
"""

import cv2

from fixtures import (
    make_corridor_apartment_map,
    make_open_plan_map,
    make_room_grid_map,
    make_tiny_room_map,
    make_two_rooms_map,
)

import oomwoo_cleaning_jobs_core.segmentation as segmentation
from oomwoo_cleaning_jobs_core.segmentation import segment


def test_two_rooms_single_doorway():
    """Two rooms joined by one 0.5 m door: exactly one topology edge, located
    at the inner-wall doorway."""
    result = segment(make_two_rooms_map())
    assert len(result.regions) == 2
    assert len(result.doorways) == 1
    doorway = result.doorways[0]
    labels = {r.label for r in result.regions}
    assert set(doorway.regions) == labels
    assert 0.35 <= doorway.width_m <= 0.65
    # Doorway at inner wall col 30, rows 35-44
    row, col = doorway.center
    assert 33 <= row <= 46
    assert 28 <= col <= 32
    assert doorway.likely_door


def test_corridor_apartment_star_topology():
    """Corridor apartment: 4 topology edges, each room<->corridor (star
    topology)."""
    source, _room_cols = make_corridor_apartment_map(n_rooms=4)
    result = segment(source)
    assert len(result.regions) == 5
    assert len(result.doorways) == 4
    # The corridor is the label appearing on every edge
    counts: dict[int, int] = {}
    for doorway in result.doorways:
        for label in doorway.regions:
            counts[label] = counts.get(label, 0) + 1
    corridor_label = max(counts, key=counts.get)
    assert counts[corridor_label] == 4
    for doorway in result.doorways:
        assert corridor_label in doorway.regions
        assert doorway.likely_door
    # Adjacency query
    assert len(result.adjacent_labels(corridor_label)) == 4


def test_grid6_seven_doorways():
    """3x2 room grid: 7 doors (4 horizontal + 3 vertical) = 7 topology edges,
    all ~0.5 m wide."""
    source, _rooms = make_room_grid_map(3, 2)
    result = segment(source)
    assert len(result.regions) == 6
    assert len(result.doorways) == 7
    for doorway in result.doorways:
        assert 0.35 <= doorway.width_m <= 0.65
        assert doorway.likely_door


def test_transitive_connection_uses_local_contact_saddle():
    """A transitive merge-tree saddle must not substitute for the actual
    doorway of the current region pair."""
    source, _ = make_room_grid_map(3, 2)
    result = segment(source)
    dist = cv2.distanceTransform(
        result.free_mask.astype('uint8'), cv2.DIST_L2, 5) * source.resolution
    connections = segmentation._connection_values(
        result.labels, dist, result.free_mask)

    # The 3x2 grid produces direct=False transitive connections; at least
    # one real doorway has its merge-tree saddle on another pair's doorway,
    # exactly the case spill clipping must avoid.
    divergent = []
    for doorway in result.doorways:
        pair = doorway.regions
        _clearance, tree_cell, direct = connections[pair]
        contact = (
            segmentation._geodesic_dilate(result.labels == pair[0], result.free_mask)
            & segmentation._geodesic_dilate(result.labels == pair[1], result.free_mask)
        )
        local_cell = segmentation._contact_saddle_cell(contact, dist)
        if not direct and tree_cell != local_cell:
            divergent.append((doorway, local_cell))

    assert divergent
    for doorway, local_cell in divergent:
        assert local_cell == doorway.center


def test_merged_regions_have_no_doorway():
    """Wide opening (saddle-merged into 1 region) and tiny room (area merge)
    both yield no topology edges."""
    assert len(segment(make_open_plan_map(opening_cells=26)).doorways) == 0
    assert len(segment(make_tiny_room_map()).doorways) == 0


def test_doorway_clearance_below_merge_threshold():
    """Every retained doorway's ratio must be below the saddle-merge
    threshold (otherwise it would have been merged)."""
    source, _rooms = make_room_grid_map(3, 2)
    result = segment(source)
    for doorway in result.doorways:
        assert doorway.ratio < result.params.saddle_merge_ratio
