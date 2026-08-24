"""Algorithm-neutral room-segmentation domain types."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


class SegmentationError(RuntimeError):
    """A segmentation request could not produce a valid canonical result."""


@dataclass(frozen=True)
class CandidateRegion:
    """Metadata derived from one positive label in a canonical label grid."""

    label: int
    cell_count: int
    area_m2: float


@dataclass(frozen=True)
class DiagnosticImage:
    """Optional implementation-specific BGR image for one pipeline stage."""

    stage: str
    image: np.ndarray = field(repr=False, compare=False)


@dataclass(frozen=True)
class WallSegment:
    """One detected wall segment in the map frame (meters).

    Derived, non-authoritative data: the authoritative segmentation result
    remains the label grid. Detected walls are reproducible algorithm output,
    never persisted as-is; converting one into a user constraint (Virtual
    Wall) requires explicit user confirmation.

    ``support`` is the provider-computed evidence fraction in [0, 1];
    ``direction_rad`` is the wall direction in the map frame in [0, pi).
    """

    x1: float
    y1: float
    x2: float
    y2: float
    support: float
    direction_rad: float


@dataclass(frozen=True)
class SegmentationResult:
    """Canonical result shared by every room-segmentation implementation.

    Arrays use OccupancyGrid row order: row 0 is the map's bottom row.
    Label 0 is unassigned and positive labels identify rooms.
    """

    labels: np.ndarray
    regions: tuple[CandidateRegion, ...]
    cleanable_mask: np.ndarray
    implementation_id: str
    implementation_version: str
    walls: tuple[WallSegment, ...] = ()
    diagnostics: tuple[DiagnosticImage, ...] = ()

    @property
    def unassigned_cleanable_mask(self) -> np.ndarray:
        return self.cleanable_mask & (self.labels == 0)

    def mask_of(self, label: int) -> np.ndarray:
        return self.labels == label
