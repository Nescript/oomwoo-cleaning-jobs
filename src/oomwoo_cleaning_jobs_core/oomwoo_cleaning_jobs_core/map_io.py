"""nav2 trinary 格式 ``map.yaml + 图像`` 加载器。

与 nav2 map_server ``map_io.cpp``（jazzy）的 trinary 行为对齐，已核实：

- ``occ = 1 - color/255``（``negate: 0``；``negate: 1`` 时为 ``color/255``），
  color 为颜色通道均值（灰度图即像素值本身）。
- ``occ >= occupied_thresh`` → 100；``occ <= free_thresh`` → 0；否则 → -1。
- 含 alpha 通道的图像中 ``alpha < 255`` 的像素一律为 unknown（-1）。
- 图像顶行对应地图最大 y，加载时垂直翻转为 OccupancyGrid 行序（row 0 = 最底行）。
- map_saver 固定写像素 0(occupied)/254(free)/205(unknown)，并配
  ``occupied_thresh: 0.65, free_thresh: 0.196``，保证 205 回读为 unknown。

只支持 ``mode: trinary``（默认）；scale/raw 抛 ValueError。
"""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np
import yaml

from .source_map import FREE, OCCUPIED, UNKNOWN, SourceMap

_SUPPORTED_MODES = ('trinary',)


def load_map_file(yaml_path: str | os.PathLike) -> SourceMap:
    """加载 nav2 trinary ``map.yaml`` 及其引用的图像，返回 SourceMap。"""
    yaml_path = Path(yaml_path)
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    if not isinstance(meta, dict):
        raise ValueError(f'{yaml_path}: 不是合法的 map.yaml（顶层应为 mapping）')

    mode = str(meta.get('mode', 'trinary')).lower()
    if mode not in _SUPPORTED_MODES:
        raise ValueError(
            f'{yaml_path}: 仅支持 mode: trinary，得到 {mode!r}'
        )

    for key in ('image', 'resolution', 'origin'):
        if key not in meta:
            raise ValueError(f'{yaml_path}: 缺少必需字段 {key!r}')

    image_path = Path(meta['image'])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    img = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError(f'{image_path}: 图像读取失败')
    if img.dtype != np.uint8:
        raise ValueError(
            f'{image_path}: 仅支持 8-bit 图像，得到 dtype {img.dtype}'
        )

    resolution = float(meta['resolution'])
    origin = meta['origin']
    if len(origin) != 3:
        raise ValueError(f'{yaml_path}: origin 必须为 [x, y, yaw]')
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))

    if img.ndim == 2:
        color = img.astype(np.float64)
        alpha = None
    elif img.ndim == 3 and img.shape[2] in (3, 4):
        color = img[:, :, :3].mean(axis=2)
        alpha = img[:, :, 3] if img.shape[2] == 4 else None
    else:
        raise ValueError(f'{image_path}: 不支持的通道数 {img.shape}')

    occ = color / 255.0 if negate else (255.0 - color) / 255.0

    cells = np.full(occ.shape, UNKNOWN, dtype=np.int8)
    cells[occ <= free_thresh] = FREE
    cells[occ >= occupied_thresh] = OCCUPIED
    if alpha is not None:
        cells[alpha < 255] = UNKNOWN

    # 图像 row 0 = 顶行（最大 y）→ OccupancyGrid row 0 = 最底行
    cells = np.ascontiguousarray(cells[::-1, :])

    height, width = cells.shape
    return SourceMap(
        resolution=resolution,
        width=width,
        height=height,
        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
        cells=cells,
    )
