"""合成地图夹具：布局确定、可精确断言。

布局（cells 行序为 OccupancyGrid 约定，row 0 = 最底行）：

- 100x80 cell，0.05 m/cell，origin (-2.5, -2.0, 0)。
- 外墙：rows/cols 边界一圈 occupied；墙外（cols 71-99）保持 unknown。
- 内墙 col 30 把室内分成左右两个房间，rows 35-44 为门洞。
- 右房间内 rows 50-59 / cols 50-59 有一块 unknown，其两侧的凸瓣
  由宽通道相连（鞍部/峰高比高），鞍部合并应并回一个房间。
"""

from pathlib import Path

import cv2
import numpy as np
import yaml

from oomwoo_cleaning_jobs_core import FREE, OCCUPIED, UNKNOWN, SourceMap

#: 与 nav2 map_saver 一致的像素约定
PIXEL_OCCUPIED = 0
PIXEL_FREE = 254
PIXEL_UNKNOWN = 205

#: 与 nav2 map_saver 写出的阈值一致（保证 205 回读为 unknown）
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
    """无未知块的干净双房间（内墙 col 30，rows 35-44 门洞）。
    距离变换每房间各一个峰，watershed 应精确分出 2 个候选。"""
    source = make_rooms_map()
    cells = source.cells.copy()
    r0, r1 = UNKNOWN_PATCH_ROWS
    c0, c1 = UNKNOWN_PATCH_COLS
    cells[r0:r1 + 1, c0:c1 + 1] = FREE
    return SourceMap(
        resolution=source.resolution, width=source.width,
        height=source.height, origin=source.origin, cells=cells)


def make_open_plan_map(opening_cells: int) -> SourceMap:
    """开放式双区：两个 30x30 cell（1.5 m 见方）区域并排，内墙上的开口宽
    opening_cells cell。开口宽 → 鞍部/峰高比高 → 鞍部合并并为一个区；
    开口窄（真门洞）→ 比值低 → 保持两个区。"""
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
    """n_cols × n_rows 房间网格：房间 room_cells 见方（默认 1.5 m），
    相邻房间共享墙中央开 door_cells 宽的门（默认 0.5 m，鞍部/峰高比 ≈0.33，
    不会被鞍部合并误并）。

    skip 中的 (ci, ri) 房间不雕刻（保持 occupied），其相连门也不开。
    furniture 为 ((ci, ri), row_off, col_off, size) 元组列表，在房间内放
    occupied 方块（周围通道足够宽时不应改变房间数）。

    返回 (SourceMap, {(ci, ri): (row0, col0)})，row0/col0 为房间左下角 cell。
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

    # 水平相邻门（共享纵墙）
    for ci in range(n_cols - 1):
        for ri in range(n_rows):
            if (ci, ri) in skip or (ci + 1, ri) in skip:
                continue
            r0, c0 = room_origin(ci, ri)
            mid = r0 + room_cells // 2
            half = door_cells // 2
            cells[mid - half:mid - half + door_cells, c0 + room_cells] = FREE
    # 垂直相邻门（共享横墙）
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
    """走廊户型：底部 1 m 宽走廊贯通，n_rooms 个房间并排在其上方，
    各由 0.5 m 门洞连接走廊。走廊峰高 ≈0.5 m、门洞鞍部 ≈0.25 m（比值 0.5），
    房间峰高 ≈0.75 m——应分出 n_rooms + 1 个候选（每房间 + 走廊各一）。

    返回 (SourceMap, [各房间 col0])。
    """
    width = 2 + n_rooms * room_cells + (n_rooms - 1)
    height = 2 + corridor_cells + 1 + room_cells
    cells = np.full((height, width), OCCUPIED, dtype=np.int8)

    cells[1:1 + corridor_cells, 1:width - 1] = FREE  # 走廊
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
    """60x40 单一大开间（无任何内墙），用于退化低置信路径。"""
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
    """大房间 + 一个 8x36 cell（0.72 m²，小于默认 min_region_area 1 m²）的小房间，
    中间带门洞；小房间中心距离变换峰值 ≈0.2 m ≥ min_peak_height，会产生独立候选，
    用于小区域合并路径。"""
    width, height = 80, 40
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    cells[2:height - 2, 2:width - 2] = FREE
    for r, c in ((1, slice(1, width - 1)), (height - 2, slice(1, width - 1))):
        cells[r, c] = OCCUPIED
    cells[1:height - 1, 1] = OCCUPIED
    cells[1:height - 1, width - 2] = OCCUPIED
    # 内墙 col 68，rows 15-24 为门洞；右侧小房间 cols 69-77（8 列）
    cells[2:height - 2, 68] = OCCUPIED
    cells[15:25, 68] = FREE
    return SourceMap(
        resolution=0.05, width=width, height=height,
        origin=(0.0, 0.0, 0.0), cells=cells)


def write_map_files(directory, source_map: SourceMap, name: str = 'map') -> Path:
    """按 nav2 map_saver 约定把 SourceMap 写成 name.pgm + name.yaml。"""
    directory = Path(directory)
    pixels = np.full(source_map.cells.shape, PIXEL_UNKNOWN, dtype=np.uint8)
    pixels[source_map.cells == FREE] = PIXEL_FREE
    pixels[source_map.cells == OCCUPIED] = PIXEL_OCCUPIED
    # 图像 row 0 = 顶行（最大 y），与 cells 行序相反
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
