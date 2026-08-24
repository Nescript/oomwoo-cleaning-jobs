"""ROS-independent port of the ROSE + ROSE2 room-segmentation pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
from pathlib import Path
import sys
import tempfile
from typing import Callable

import cv2
import numpy as np

from oomwoo_segmentation.models import (
    DiagnosticImage,
    SegmentationError,
    SegmentationResult,
    WallSegment,
)
from oomwoo_segmentation.render import render_segmentation, render_source_map
from oomwoo_segmentation.source_map import SourceMap
from oomwoo_segmentation.validation import canonicalize_labels, effective_cleanable_mask, validate_result

UPSTREAM_REPOSITORY = 'https://github.com/aislabunimi/ROSE2'
UPSTREAM_COMMIT = '3a010b9e6bb2477de3b5b46208ebfccd71dfafbf'
IMPLEMENTATION_ID = 'rose2'
IMPLEMENTATION_VERSION = f'upstream-{UPSTREAM_COMMIT[:12]}+oomwoo.4'

_LOG = logging.getLogger(__name__)


def _bootstrap_upstream() -> None:
    """Expose vendored modules under the absolute names used upstream."""
    root = str(Path(__file__).with_name('upstream'))
    if root not in sys.path:
        sys.path.insert(0, root)


@dataclass(frozen=True)
class Rose2Config:
    """ROSE.launch-compatible parameters from the pinned upstream commit."""

    filter_level: float = 0.18
    fft_peak_height: float = 0.2
    fft_band_width: int = 50
    spatial_clustering_line_segments_threshold: float = 5.0
    lines_threshold: float = 0.22
    lines_distance_px: float = 20.0
    edges_threshold: float = 0.0
    rooms_voronoi: bool = False
    voronoi_closeness: int = 10
    voronoi_blur: int = 8
    voronoi_iterations: int = 5

    def validate(self) -> None:
        for name in ('filter_level', 'lines_threshold', 'edges_threshold'):
            value = float(getattr(self, name))
            if not 0.0 <= value <= 1.0:
                raise SegmentationError(f'{name} must be between 0 and 1')
        if self.fft_band_width <= 0:
            raise SegmentationError('fft_band_width must be positive')
        if self.lines_distance_px < 0:
            raise SegmentationError('lines_distance_px must be non-negative')
        if self.voronoi_iterations < 0:
            raise SegmentationError('voronoi_iterations must be non-negative')


class Rose2Segmenter:
    """Concrete provider aligned with the upstream two-stage ROSE pipeline."""

    implementation_id = IMPLEMENTATION_ID
    implementation_version = IMPLEMENTATION_VERSION

    def __init__(self, config: Rose2Config | None = None) -> None:
        self.config = config or Rose2Config()
        self.config.validate()

    def segment(
        self,
        source_map: SourceMap,
        cleanable_mask: np.ndarray | None = None,
        *,
        include_diagnostics: bool = False,
        progress: Callable[[str, float], None] | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> SegmentationResult:
        progress = progress or (lambda _stage, _value: None)
        cancelled = cancelled or (lambda: False)
        cleanable = effective_cleanable_mask(source_map, cleanable_mask)
        if not cleanable.any():
            raise SegmentationError('map contains no cleanable cells')

        _bootstrap_upstream()
        try:
            from rose_v1_repo.fft_structure_extraction import FFTStructureExtraction
            from rose_v2_repo import minibatch, parameters
        except ImportError as exc:
            raise SegmentationError(
                f'ROSE2 runtime dependency is unavailable: {exc.name or exc}') from exc

        original_image = self._source_image(source_map, cleanable)
        progress('rose_preprocessing', 0.1)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        rose = FFTStructureExtraction(
            original_image,
            peak_height=self.config.fft_peak_height,
            par=self.config.fft_band_width,
        )
        try:
            rose.process_map()
            if len(rose.main_directions) <= 2:
                raise SegmentationError('ROSE found fewer than two dominant direction pairs')
            rose.simple_filter_map(self.config.filter_level)
            rose.generate_initial_hypothesis_simple()
            rose.find_walls_flood_filing()
        except SegmentationError:
            raise
        except Exception as exc:
            raise SegmentationError(f'ROSE preprocessing failed: {exc}') from exc

        clean_image = np.asarray(rose.analysed_map, dtype=np.uint8)
        clean_image = clean_image[:source_map.height, :source_map.width]
        directions = list(rose.main_directions)
        progress('rose2_layout', 0.45)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        params = parameters.ParameterObj()
        params.comp = directions
        params.filter_level = self.config.filter_level
        params.spatialClusteringLineSegmentsThreshold = (
            self.config.spatial_clustering_line_segments_threshold)
        params.th1 = self.config.lines_threshold
        params.distance_extended_segment = self.config.lines_distance_px
        params.threshold_edges = self.config.edges_threshold
        params.voronoi_closeness = self.config.voronoi_closeness
        params.blur = self.config.voronoi_blur
        params.iterations = self.config.voronoi_iterations

        batch = minibatch.Minibatch()
        try:
            with tempfile.TemporaryDirectory(prefix='oomwoo-rose2-') as work:
                batch.start_main(
                    parameters,
                    params,
                    clean_image,
                    original_image,
                    work + '/',
                    self.config.rooms_voronoi,
                )
        except Exception as exc:
            raise SegmentationError(f'ROSE2 room extraction failed: {exc}') from exc
        if cancelled():
            raise SegmentationError('segmentation cancelled')
        rooms = batch.rooms_th1
        if not rooms:
            raise SegmentationError('ROSE2 found no rooms')

        progress('canonicalizing', 0.85)
        labels = self._rasterize_rooms(rooms, source_map.cells.shape)
        labels, regions, cleanable = canonicalize_labels(
            labels, source_map, cleanable)
        if not regions:
            raise SegmentationError('ROSE2 room polygons contain no cleanable cells')

        walls = self._extract_walls(source_map, batch)

        diagnostics: tuple[DiagnosticImage, ...] = ()
        if include_diagnostics:
            provisional = SegmentationResult(
                labels=labels,
                regions=regions,
                cleanable_mask=cleanable,
                implementation_id=self.implementation_id,
                implementation_version=self.implementation_version,
            )
            diagnostics = self._diagnostics(
                source_map, provisional, clean_image, batch)

        result = SegmentationResult(
            labels=labels,
            regions=regions,
            cleanable_mask=cleanable,
            implementation_id=self.implementation_id,
            implementation_version=self.implementation_version,
            walls=walls,
            diagnostics=diagnostics,
        )
        validate_result(result, source_map)
        if cancelled():
            raise SegmentationError('segmentation cancelled')
        progress('complete', 1.0)
        return result

    @staticmethod
    def _source_image(source_map: SourceMap, cleanable: np.ndarray) -> np.ndarray:
        """Match upstream fromOccupancyGridToImg without changing row order."""
        image = np.full(source_map.cells.shape, 200, dtype=np.uint8)
        image[source_map.occupied_mask()] = 0
        image[source_map.free_mask()] = 255
        image[source_map.free_mask() & ~cleanable] = 0
        return image

    @classmethod
    def _rasterize_rooms(
        cls,
        rooms,
        shape: tuple[int, int],
    ) -> np.ndarray:
        labels = np.zeros(shape, dtype=np.int32)
        ordered = sorted(
            (room for room in rooms if room is not None and not room.is_empty),
            key=lambda room: (float(room.centroid.y), float(room.centroid.x), float(room.area)),
        )
        for label, geometry in enumerate(ordered, start=1):
            for polygon in cls._polygons(geometry):
                exterior = np.rint(np.asarray(polygon.exterior.coords)).astype(np.int32)
                if exterior.size:
                    cv2.fillPoly(labels, [exterior], label)
                for interior in polygon.interiors:
                    hole = np.rint(np.asarray(interior.coords)).astype(np.int32)
                    if hole.size:
                        cv2.fillPoly(labels, [hole], 0)
        return labels

    @staticmethod
    def _extract_walls(source_map: SourceMap, batch) -> tuple[WallSegment, ...]:
        """Convert retained merged extended segments into map-frame walls.

        ``extended_segments_th1_merged`` holds the thresholded, merged wall
        lines in pipeline pixel space (x = col, y = row, row 0 = bottom).
        Upstream extends them past a bounding box padded by ``offset`` px, so
        endpoints are clipped to the map rect before conversion; segments
        lying fully outside the map are dropped. Direction is derived from
        the converted endpoints (pixel space and map-local frame share the
        same x-right/y-up orientation), not from the upstream angular
        cluster, so yaw handling stays explicit.
        """
        walls: list[WallSegment] = []
        rect = (0, 0, source_map.width - 1, source_map.height - 1)
        _, _, yaw = source_map.origin
        for segment in getattr(batch, 'extended_segments_th1_merged', ()):
            p1 = (int(round(segment.x1)), int(round(segment.y1)))
            p2 = (int(round(segment.x2)), int(round(segment.y2)))
            inside, q1, q2 = cv2.clipLine(rect, p1, p2)
            if not inside or q1 == q2:
                continue
            x1, y1 = source_map.map_frame_from_pixel(float(q1[0]), float(q1[1]))
            x2, y2 = source_map.map_frame_from_pixel(float(q2[0]), float(q2[1]))
            local_direction = math.atan2(q2[1] - q1[1], q2[0] - q1[0]) % math.pi
            direction = (local_direction + yaw) % math.pi
            support = float(segment.weight) if segment.weight is not None else 0.0
            walls.append(WallSegment(
                x1=x1, y1=y1, x2=x2, y2=y2,
                support=min(max(support, 0.0), 1.0),
                direction_rad=direction,
            ))
        # Deterministic order: by support (strongest first), then endpoints.
        walls.sort(key=lambda w: (-w.support, w.x1, w.y1, w.x2, w.y2))
        return tuple(walls)

    @staticmethod
    def _polygons(geometry):
        if geometry.geom_type == 'Polygon':
            return (geometry,)
        if geometry.geom_type == 'MultiPolygon':
            return tuple(geometry.geoms)
        if geometry.geom_type == 'GeometryCollection':
            return tuple(
                part for item in geometry.geoms
                for part in Rose2Segmenter._polygons(item)
            )
        return ()

    @staticmethod
    def _diagnostics(
        source_map: SourceMap,
        result: SegmentationResult,
        clean_image: np.ndarray,
        batch,
    ) -> tuple[DiagnosticImage, ...]:
        cleaned = np.where(clean_image > 0, 255, 0).astype(np.uint8)
        cleaned = cv2.cvtColor(cleaned[::-1, :], cv2.COLOR_GRAY2BGR)

        lines = render_source_map(source_map)
        lines_cell_order = lines[::-1, :].copy()
        for segment in getattr(batch, 'extended_segments_th1_merged', ()):
            p1 = (int(round(segment.x1)), int(round(segment.y1)))
            p2 = (int(round(segment.x2)), int(round(segment.y2)))
            cv2.line(lines_cell_order, p1, p2, (0, 0, 255), 1, cv2.LINE_AA)
        lines = np.ascontiguousarray(lines_cell_order[::-1, :])

        overlay = render_segmentation(source_map, result)
        return (
            DiagnosticImage('cleaned_map', cleaned),
            DiagnosticImage('extended_lines', lines),
            DiagnosticImage('labels_overlay', overlay),
        )
