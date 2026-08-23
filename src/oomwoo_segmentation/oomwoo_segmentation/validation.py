"""Validation and canonicalization for implementation-neutral results."""

from __future__ import annotations

import numpy as np

from .models import CandidateRegion, SegmentationError, SegmentationResult
from .source_map import SourceMap


def effective_cleanable_mask(
    source_map: SourceMap,
    cleanable_mask: np.ndarray | None,
) -> np.ndarray:
    """Return source free space intersected with an optional caller mask."""
    source_free = source_map.free_mask()
    if cleanable_mask is None:
        return source_free
    mask = np.asarray(cleanable_mask, dtype=bool)
    if mask.shape != source_free.shape:
        raise SegmentationError(
            f'cleanable mask shape {mask.shape} does not match map shape '
            f'{source_free.shape}'
        )
    return source_free & mask


def canonicalize_labels(
    labels: np.ndarray,
    source_map: SourceMap,
    cleanable_mask: np.ndarray | None,
) -> tuple[np.ndarray, tuple[CandidateRegion, ...], np.ndarray]:
    """Clip, deterministically relabel, and derive room metadata."""
    cleanable = effective_cleanable_mask(source_map, cleanable_mask)
    labels = np.asarray(labels)
    if labels.shape != source_map.cells.shape:
        raise SegmentationError(
            f'label shape {labels.shape} does not match map shape '
            f'{source_map.cells.shape}'
        )
    if not np.issubdtype(labels.dtype, np.integer):
        raise SegmentationError(f'labels must be integers, got {labels.dtype}')
    if np.any(labels < 0):
        raise SegmentationError('labels must be non-negative')

    clipped = np.ascontiguousarray(labels, dtype=np.int32)
    clipped[~cleanable] = 0

    entries: list[tuple[float, float, int, np.ndarray]] = []
    for old_label in (int(v) for v in np.unique(clipped) if v > 0):
        mask = clipped == old_label
        rows, cols = np.nonzero(mask)
        if rows.size:
            entries.append((float(rows.mean()), float(cols.mean()), old_label, mask))
    entries.sort(key=lambda item: (item[0], item[1], item[2]))

    canonical = np.zeros(clipped.shape, dtype=np.int32)
    regions: list[CandidateRegion] = []
    for new_label, (_row, _col, _old_label, mask) in enumerate(entries, start=1):
        canonical[mask] = new_label
        count = int(mask.sum())
        regions.append(CandidateRegion(
            label=new_label,
            cell_count=count,
            area_m2=count * source_map.resolution * source_map.resolution,
        ))
    return canonical, tuple(regions), cleanable


def validate_result(result: SegmentationResult, source_map: SourceMap) -> None:
    """Raise SegmentationError unless result satisfies the shared contract."""
    if result.labels.dtype != np.int32:
        raise SegmentationError(f'labels dtype must be int32, got {result.labels.dtype}')
    if result.labels.shape != source_map.cells.shape:
        raise SegmentationError('labels shape does not match source map')
    if result.cleanable_mask.shape != source_map.cells.shape:
        raise SegmentationError('cleanable mask shape does not match source map')
    if result.cleanable_mask.dtype != np.bool_:
        raise SegmentationError(
            f'cleanable mask dtype must be bool, got {result.cleanable_mask.dtype}')
    if np.any(result.labels < 0):
        raise SegmentationError('labels must be non-negative')
    if np.any(result.cleanable_mask & ~source_map.free_mask()):
        raise SegmentationError('cleanable mask includes cells outside source free space')
    if np.any(result.labels[~result.cleanable_mask] != 0):
        raise SegmentationError('labels include cells outside cleanable space')

    labels = tuple(int(v) for v in np.unique(result.labels) if v > 0)
    expected = tuple(range(1, len(labels) + 1))
    if labels != expected:
        raise SegmentationError(f'positive labels must be contiguous: {labels!r}')
    if tuple(region.label for region in result.regions) != expected:
        raise SegmentationError('region metadata does not match positive labels')
    for region in result.regions:
        count = int((result.labels == region.label).sum())
        if region.cell_count != count:
            raise SegmentationError(f'room {region.label} cell count mismatch')
        expected_area = count * source_map.resolution * source_map.resolution
        if not np.isclose(region.area_m2, expected_area):
            raise SegmentationError(f'room {region.label} area mismatch')
