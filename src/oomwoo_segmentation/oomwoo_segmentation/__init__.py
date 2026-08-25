"""Room-segmentation contract, engine, client, validation, and rendering."""

from .engine import SegmentationConfig, SegmentationEngine
from .models import CandidateRegion, DiagnosticImage, SegmentationError, SegmentationResult, WallSegment
from .protocol import RoomSegmenter
from .source_map import SourceMap

__all__ = [
    'CandidateRegion',
    'DiagnosticImage',
    'RoomSegmenter',
    'SegmentationConfig',
    'SegmentationEngine',
    'SegmentationError',
    'SegmentationResult',
    'SourceMap',
    'WallSegment',
]
