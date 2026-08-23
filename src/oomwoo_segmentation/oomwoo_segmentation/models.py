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
    diagnostics: tuple[DiagnosticImage, ...] = ()

    @property
    def unassigned_cleanable_mask(self) -> np.ndarray:
        return self.cleanable_mask & (self.labels == 0)

    def mask_of(self, label: int) -> np.ndarray:
        return self.labels == label
