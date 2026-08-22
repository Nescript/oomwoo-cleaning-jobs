"""Multi-room (5/6/7) and furniture-scene segmentation tests."""

import numpy as np
import pytest

from fixtures import (
    make_corridor_apartment_map,
    make_room_grid_map,
)

from oomwoo_cleaning_jobs_core.segmentation import UNCLASSIFIED, segment

ROOM = 30  # fixtures default room_cells


def _room_label(result, rooms, key, margin=3):
    """Assert a room interior (inset by margin) belongs to exactly one
    candidate; return its label."""
    r0, c0 = rooms[key]
    inner = result.labels[
        r0 + margin:r0 + ROOM - margin,
        c0 + margin:c0 + ROOM - margin,
    ]
    values = {int(v) for v in np.unique(inner) if v != UNCLASSIFIED}
    assert len(values) == 1, f'room {key} interior has multiple labels: {values}'
    return values.pop()


def _assert_grid_exact_rooms(result, rooms, expected_count):
    assert len(result.regions) == expected_count
    room_labels = {key: _room_label(result, rooms, key) for key in rooms}
    assert len(set(room_labels.values())) == expected_count


@pytest.mark.parametrize(('n_cols', 'n_rows', 'skip', 'expected'), [
    (3, 2, frozenset({(2, 1)}), 5),
    (3, 2, frozenset(), 6),
    (4, 2, frozenset({(3, 1)}), 7),
], ids=['5_rooms', '6_rooms', '7_rooms'])
def test_room_grid_exact_count(n_cols, n_rows, skip, expected):
    """An N-room grid (0.5 m doorways) yields exactly N candidates, one per
    room with no cross-assignment."""
    source, rooms = make_room_grid_map(n_cols, n_rows, skip=skip)
    assert len(rooms) == expected
    result = segment(source)
    _assert_grid_exact_rooms(result, rooms, expected)
    # Candidates do not overlap and stay within free space
    covered = np.zeros(source.cells.shape, dtype=bool)
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()
        assert not (mask & covered).any()
        covered |= mask


def test_room_grid_with_furniture_keeps_room_count():
    """Furniture placed in 3 of 6 grid rooms (passages stay wide enough):
    room count unchanged, candidates contain no occupied cells."""
    furniture = (
        ((0, 0), 2, 2, 6),     # 0.3 m square, against the corner
        ((1, 0), 2, 10, 8),    # 0.4 m square, against the bottom wall
        ((2, 1), 5, 22, 5),    # 0.25 m square, against the right wall
    )
    source, rooms = make_room_grid_map(3, 2, furniture=furniture)
    result = segment(source)
    _assert_grid_exact_rooms(result, rooms, 6)
    for region in result.regions:
        mask = result.mask_of(region.label)
        assert (mask <= source.free_mask()).all()
        assert not (mask & source.occupied_mask()).any()


def test_corridor_apartment_rooms_plus_corridor():
    """Corridor apartment: 4 rooms + 1 corridor = 5 candidates; the corridor
    is a single label."""
    source, room_cols = make_corridor_apartment_map(n_rooms=4)
    result = segment(source)
    assert len(result.regions) == 5
    # Corridor interior (rows 3-18, avoiding doorway ridges) is one label
    corridor = result.labels[3:18, 3:-3]
    corridor_labels = {int(v) for v in np.unique(corridor) if v != UNCLASSIFIED}
    assert len(corridor_labels) == 1
    corridor_label = corridor_labels.pop()
    # Each room interior is a single label, none equal to the corridor
    room_labels = set()
    for c0 in room_cols:
        inner = result.labels[25:49, c0 + 3:c0 + ROOM - 3]
        values = {int(v) for v in np.unique(inner) if v != UNCLASSIFIED}
        assert len(values) == 1
        room_labels.add(values.pop())
    assert len(room_labels) == 4
    assert corridor_label not in room_labels
