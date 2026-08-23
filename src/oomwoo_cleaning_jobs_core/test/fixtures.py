"""Synthetic map fixtures: deterministic layouts for precise assertions.

Layout (cell row order follows the OccupancyGrid convention, row 0 = bottom):

- 100x80 cells, 0.05 m/cell, origin (-2.5, -2.0, 0).
- Outer walls: one occupied ring at the boundary rows/cols; outside the walls
  (cols 71-99) stays unknown.
- Inner wall at col 30 splits the interior into left/right rooms with a
  doorway at rows 35-44.
- The right room contains an unknown patch at rows 50-59 / cols 50-59; the
  lobes on its two sides connect through wide passages (high saddle/peak
  ratio), so saddle merging should recombine them into one room.
"""

from pathlib import Path

import cv2
import numpy as np
import yaml

from oomwoo_segmentation.models import SegmentationResult
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_segmentation.validation import canonicalize_labels

#: Pixel conventions matching nav2 map_saver
PIXEL_OCCUPIED = 0
PIXEL_FREE = 254
PIXEL_UNKNOWN = 205

#: Thresholds matching nav2 map_saver output (ensures 205 reads back as unknown)
SAVER_OCCUPIED_THRESH = 0.65
SAVER_FREE_THRESH = 0.196

WALL_TOP = 75
WALL_BOTTOM = 4
WALL_LEFT = 4
WALL_RIGHT = 70
INNER_WALL_COL = 30
DOOR_ROWS = (35, 44)
UNKNOWN_PATCH_ROWS = (50, 59)
UNKNOWN_PATCH_COLS = (50, 59)


def fake_segmentation(source: SourceMap, cleanable_mask=None) -> SegmentationResult:
    """Deterministic test adapter; production tests must use a real provider.

    It splits on the strongest interior vertical wall when one exists. This
    keeps RegionSet/UI tests focused on editing semantics rather than ROSE2.
    """
    cleanable = source.free_mask()
    if cleanable_mask is not None:
        cleanable &= np.asarray(cleanable_mask, dtype=bool)
    labels = np.zeros(source.cells.shape, dtype=np.int32)
    rows, cols = np.nonzero(cleanable)
    if not rows.size:
        canonical, regions, cleanable = canonicalize_labels(labels, source, cleanable)
    else:
        c0, c1 = int(cols.min()), int(cols.max())
        r0, r1 = int(rows.min()), int(rows.max())
        scores = source.occupied_mask()[r0:r1 + 1, c0:c1 + 1].sum(axis=0)
        wall_offset = int(np.argmax(scores))
        wall_col = c0 + wall_offset
        if scores[wall_offset] >= max(3, (r1 - r0 + 1) // 3):
            labels[cleanable & (np.indices(labels.shape)[1] < wall_col)] = 1
            labels[cleanable & (np.indices(labels.shape)[1] > wall_col)] = 2
        else:
            labels[cleanable] = 1
        canonical, regions, cleanable = canonicalize_labels(labels, source, cleanable)
    return SegmentationResult(
        canonical, regions, cleanable, 'test-fake', '1')


def make_rooms_map() -> SourceMap:
    width, height = 100, 80
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)

    cells[WALL_BOTTOM + 1:WALL_TOP, WALL_LEFT + 1:WALL_RIGHT] = FREE
    cells[WALL_BOTTOM, WALL_LEFT:WALL_RIGHT + 1] = OCCUPIED
    cells[WALL_TOP, WALL_LEFT:WALL_RIGHT + 1] = OCCUPIED
    cells[WALL_BOTTOM:WALL_TOP + 1, WALL_LEFT] = OCCUPIED
    cells[WALL_BOTTOM:WALL_TOP + 1, WALL_RIGHT] = OCCUPIED

    cells[WALL_BOTTOM + 1:WALL_TOP, INNER_WALL_COL] = OCCUPIED
    cells[DOOR_ROWS[0]:DOOR_ROWS[1] + 1, INNER_WALL_COL] = FREE

    r0, r1 = UNKNOWN_PATCH_ROWS
    c0, c1 = UNKNOWN_PATCH_COLS
    cells[r0:r1 + 1, c0:c1 + 1] = UNKNOWN

    return SourceMap(
        resolution=0.05,
        width=width,
        height=height,
        origin=(-2.5, -2.0, 0.0),
        cells=cells,
    )


def make_two_rooms_map() -> SourceMap:
    """Clean two-room map without the unknown patch (inner wall col 30,
    doorway rows 35-44). One distance peak per room; watershed should
    produce exactly 2 candidates."""
    source = make_rooms_map()
    cells = source.cells.copy()
    r0, r1 = UNKNOWN_PATCH_ROWS
    c0, c1 = UNKNOWN_PATCH_COLS
    cells[r0:r1 + 1, c0:c1 + 1] = FREE
    return SourceMap(
        resolution=source.resolution, width=source.width,
        height=source.height, origin=source.origin, cells=cells)


def make_open_plan_map(opening_cells: int) -> SourceMap:
    """Open-plan two zones: two 30x30-cell (1.5 m square) areas side by side,
    with an opening_cells-wide opening in the inner wall. A wide opening
    gives a high saddle/peak ratio so saddle merging fuses the zones into
    one; a narrow opening (a real doorway) keeps the ratio low and the two
    zones stay separate."""
    width, height = 64, 34
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    cells[1, 1:width - 1] = OCCUPIED
    cells[height - 2, 1:width - 1] = OCCUPIED
    cells[1:height - 1, 1] = OCCUPIED
    cells[1:height - 1, width - 2] = OCCUPIED
    cells[2:height - 2, 2:width - 2] = FREE
    cells[2:height - 2, 32] = OCCUPIED
    mid = (2 + height - 2) // 2
    half = opening_cells // 2
    cells[mid - half:mid - half + opening_cells, 32] = FREE
    return SourceMap(
        resolution=0.05, width=width, height=height,
        origin=(0.0, 0.0, 0.0), cells=cells)


def make_room_grid_map(
    n_cols: int,
    n_rows: int,
    skip: frozenset[tuple[int, int]] = frozenset(),
    room_cells: int = 30,
    door_cells: int = 10,
    furniture: tuple = (),
) -> tuple[SourceMap, dict[tuple[int, int], tuple[int, int]]]:
    """n_cols x n_rows room grid: rooms are room_cells square (default
    1.5 m); adjacent rooms share a wall with a door_cells-wide door at its
    center (default 0.5 m, saddle/peak ratio ~0.33, so saddle merging will
    not fuse them).

    Rooms (ci, ri) in skip are not carved out (stay occupied) and their
    connecting doors are not opened. furniture is a list of
    ((ci, ri), row_off, col_off, size) tuples placing occupied squares
    inside rooms (should not change the room count when the surrounding
    passage stays wide enough).

    Returns (SourceMap, {(ci, ri): (row0, col0)}) where row0/col0 is the
    room's lower-left cell.
    """
    width = 1 + n_cols * (room_cells + 1)
    height = 1 + n_rows * (room_cells + 1)
    cells = np.full((height, width), OCCUPIED, dtype=np.int8)
    rooms: dict[tuple[int, int], tuple[int, int]] = {}

    def room_origin(ci: int, ri: int) -> tuple[int, int]:
        return 1 + ri * (room_cells + 1), 1 + ci * (room_cells + 1)

    for ci in range(n_cols):
        for ri in range(n_rows):
            if (ci, ri) in skip:
                continue
            r0, c0 = room_origin(ci, ri)
            cells[r0:r0 + room_cells, c0:c0 + room_cells] = FREE
            rooms[(ci, ri)] = (r0, c0)

    # Doors between horizontally adjacent rooms (shared vertical wall)
    for ci in range(n_cols - 1):
        for ri in range(n_rows):
            if (ci, ri) in skip or (ci + 1, ri) in skip:
                continue
            r0, c0 = room_origin(ci, ri)
            mid = r0 + room_cells // 2
            half = door_cells // 2
            cells[mid - half:mid - half + door_cells, c0 + room_cells] = FREE
    # Doors between vertically adjacent rooms (shared horizontal wall)
    for ci in range(n_cols):
        for ri in range(n_rows - 1):
            if (ci, ri) in skip or (ci, ri + 1) in skip:
                continue
            r0, c0 = room_origin(ci, ri)
            mid = c0 + room_cells // 2
            half = door_cells // 2
            cells[r0 + room_cells, mid - half:mid - half + door_cells] = FREE

    for (ci, ri), row_off, col_off, size in furniture:
        r0, c0 = rooms[(ci, ri)]
        cells[r0 + row_off:r0 + row_off + size,
              c0 + col_off:c0 + col_off + size] = OCCUPIED

    return SourceMap(
        resolution=0.05, width=width, height=height,
        origin=(0.0, 0.0, 0.0), cells=cells), rooms


def make_corridor_apartment_map(
    n_rooms: int = 4,
    room_cells: int = 30,
    corridor_cells: int = 20,
    door_cells: int = 10,
) -> tuple[SourceMap, list[int]]:
    """Corridor apartment: a 1 m wide corridor runs along the bottom, with
    n_rooms rooms side by side above it, each connected to the corridor by
    a 0.5 m doorway. Corridor peak height ~0.5 m, doorway saddle ~0.25 m
    (ratio 0.5), room peak height ~0.75 m -- should yield n_rooms + 1
    candidates (one per room plus one for the corridor).

    Returns (SourceMap, [col0 per room]).
    """
    width = 2 + n_rooms * room_cells + (n_rooms - 1)
    height = 2 + corridor_cells + 1 + room_cells
    cells = np.full((height, width), OCCUPIED, dtype=np.int8)

    cells[1:1 + corridor_cells, 1:width - 1] = FREE  # corridor
    wall_row = 1 + corridor_cells
    room_row0 = wall_row + 1
    room_cols = []
    for i in range(n_rooms):
        c0 = 1 + i * (room_cells + 1)
        cells[room_row0:room_row0 + room_cells, c0:c0 + room_cells] = FREE
        mid = c0 + room_cells // 2
        half = door_cells // 2
        cells[wall_row, mid - half:mid - half + door_cells] = FREE
        room_cols.append(c0)

    return SourceMap(
        resolution=0.05, width=width, height=height,
        origin=(0.0, 0.0, 0.0), cells=cells), room_cols


def make_open_room_map() -> SourceMap:
    """60x40 single open room (no inner walls), for the degenerate
    low-confidence path."""
    width, height = 60, 40
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    cells[2:height - 2, 2:width - 2] = FREE
    cells[1, 1:width - 1] = OCCUPIED
    cells[height - 2, 1:width - 1] = OCCUPIED
    cells[1:height - 1, 1] = OCCUPIED
    cells[1:height - 1, width - 2] = OCCUPIED
    return SourceMap(
        resolution=0.05, width=width, height=height,
        origin=(0.0, 0.0, 0.0), cells=cells)


def make_tiny_room_map() -> SourceMap:
    """Large room plus one 8x36-cell (0.72 m^2, below the default
    min_region_area of 1 m^2) small room, connected by a doorway; the small
    room's center distance peak ~0.2 m >= min_peak_height produces its own
    candidate, exercising the small-region merge path."""
    width, height = 80, 40
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    cells[2:height - 2, 2:width - 2] = FREE
    for r, c in ((1, slice(1, width - 1)), (height - 2, slice(1, width - 1))):
        cells[r, c] = OCCUPIED
    cells[1:height - 1, 1] = OCCUPIED
    cells[1:height - 1, width - 2] = OCCUPIED
    # Inner wall at col 68 with doorway at rows 15-24; small room on the
    # right spans cols 69-77 (8 columns)
    cells[2:height - 2, 68] = OCCUPIED
    cells[15:25, 68] = FREE
    return SourceMap(
        resolution=0.05, width=width, height=height,
        origin=(0.0, 0.0, 0.0), cells=cells)


def write_map_files(directory, source_map: SourceMap, name: str = 'map') -> Path:
    """Write a SourceMap as name.pgm + name.yaml per nav2 map_saver conventions."""
    directory = Path(directory)
    pixels = np.full(source_map.cells.shape, PIXEL_UNKNOWN, dtype=np.uint8)
    pixels[source_map.cells == FREE] = PIXEL_FREE
    pixels[source_map.cells == OCCUPIED] = PIXEL_OCCUPIED
    # Image row 0 = top row (max y), opposite of the cells row order
    image_path = directory / f'{name}.pgm'
    assert cv2.imwrite(str(image_path), pixels[::-1, :])

    yaml_path = directory / f'{name}.yaml'
    meta = {
        'image': image_path.name,
        'resolution': source_map.resolution,
        'origin': list(source_map.origin),
        'negate': 0,
        'occupied_thresh': SAVER_OCCUPIED_THRESH,
        'free_thresh': SAVER_FREE_THRESH,
        'mode': 'trinary',
    }
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f)
    return yaml_path
