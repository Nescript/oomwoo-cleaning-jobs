"""The in-process interface implemented by segmentation providers."""

from __future__ import annotations

from typing import Protocol

import numpy as np

from .models import SegmentationResult
from .source_map import SourceMap


class RoomSegmenter(Protocol):
    """Deep module interface for a deterministic room-segmentation provider."""

    @property
    def implementation_id(self) -> str: ...

    @property
    def implementation_version(self) -> str: ...

    def segment(
        self,
        source_map: SourceMap,
        cleanable_mask: np.ndarray | None = None,
        *,
        include_diagnostics: bool = False,
    ) -> SegmentationResult: ...
