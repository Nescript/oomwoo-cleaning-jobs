"""Algorithm-neutral room-segmentation contract, client, validation, and rendering."""

from .models import CandidateRegion, DiagnosticImage, SegmentationError, SegmentationResult
from .protocol import RoomSegmenter
from .source_map import SourceMap

__all__ = [
    'CandidateRegion',
    'DiagnosticImage',
    'RoomSegmenter',
    'SegmentationError',
    'SegmentationResult',
    'SourceMap',
]
