"""Loader for nav2 trinary format ``map.yaml + image``.

Aligned with the trinary behavior of nav2 map_server ``map_io.cpp`` (jazzy),
verified against its source:

- ``occ = 1 - color/255`` (``negate: 0``; ``color/255`` when ``negate: 1``),
  where color is the mean of the color channels (the pixel value itself for
  grayscale).
- ``occ >= occupied_thresh`` -> 100; ``occ <= free_thresh`` -> 0; else -1.
- In images with an alpha channel, pixels with ``alpha < 255`` are always
  unknown (-1).
- The image top row corresponds to the map's maximum y; it is flipped
  vertically into OccupancyGrid row order on load (row 0 = bottom row).
- map_saver always writes pixels 0(occupied)/254(free)/205(unknown) with
  ``occupied_thresh: 0.65, free_thresh: 0.196``, so 205 reads back as unknown.

Only ``mode: trinary`` (the default) is supported; scale/raw raise ValueError.
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
    """Load a nav2 trinary ``map.yaml`` and its referenced image; return a SourceMap."""
    yaml_path = Path(yaml_path)
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    if not isinstance(meta, dict):
        raise ValueError(f'{yaml_path}: not a valid map.yaml (top level must be a mapping)')

    mode = str(meta.get('mode', 'trinary')).lower()
    if mode not in _SUPPORTED_MODES:
        raise ValueError(
            f'{yaml_path}: only mode: trinary is supported, got {mode!r}'
        )

    for key in ('image', 'resolution', 'origin'):
        if key not in meta:
            raise ValueError(f'{yaml_path}: missing required field {key!r}')

    image_path = Path(meta['image'])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    img = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError(f'{image_path}: failed to read image')
    if img.dtype != np.uint8:
        raise ValueError(
            f'{image_path}: only 8-bit images are supported, got dtype {img.dtype}'
        )

    resolution = float(meta['resolution'])
    origin = meta['origin']
    if len(origin) != 3:
        raise ValueError(f'{yaml_path}: origin must be [x, y, yaw]')
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
        raise ValueError(f'{image_path}: unsupported channel count {img.shape}')

    occ = color / 255.0 if negate else (255.0 - color) / 255.0

    cells = np.full(occ.shape, UNKNOWN, dtype=np.int8)
    cells[occ <= free_thresh] = FREE
    cells[occ >= occupied_thresh] = OCCUPIED
    if alpha is not None:
        cells[alpha < 255] = UNKNOWN

    # image row 0 = top row (max y) -> OccupancyGrid row 0 = bottom row
    cells = np.ascontiguousarray(cells[::-1, :])

    height, width = cells.shape
    return SourceMap(
        resolution=resolution,
        width=width,
        height=height,
        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
        cells=cells,
    )
