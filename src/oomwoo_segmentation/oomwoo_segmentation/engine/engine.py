"""Native Python 3 room segmentation engine.

Faithful port of the pinned upstream ROSE + ROSE2 two-stage pipeline
(fft structural filtering -> Hough walls -> angular/spatial clustering ->
extended lines -> edges -> planar cells -> affinity DBSCAN -> rooms),
reorganized into pure in-memory modules. ROS 1 wrappers, temporary
directories, and deprecated APIs are removed; the math and data flow are
preserved so results match the benchmark baseline.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

import numpy as np

from ..models import (
    DiagnosticImage,
    SegmentationError,
    SegmentationResult,
)
from ..source_map import SourceMap
from ..validation import canonicalize_labels, effective_cleanable_mask, validate_result
from .clustering import (
    classification_surface,
    create_matrices,
    dbscan_cluster_cells,
    merge_cells,
    remove_frame_fringes,
)
from .config import SegmentationConfig
from .fft import FFTStructureExtraction
from .geometry import (
    assign_orebro_direction,
    cluster_ang,
    create_cells,
    create_edges,
    create_extended_lines,
    create_extended_segments,
    create_short_ex_lines,
    external_contour,
    find_extremes,
    get_representatives,
    get_wall_clusters,
    merge_together,
    new_spatial_cluster,
    remove_less_representatives,
    set_weight_offset,
    set_weights,
    spatial_clustering,
    start_canny_and_hough,
)
from .postprocessing import build_diagnostics, extract_walls, geodesic_coverage, rasterize_rooms

_LOG = logging.getLogger(__name__)
IMPLEMENTATION_ID = 'oomwoo_segmentation'
IMPLEMENTATION_VERSION = 'native-1.0.0'


class SegmentationEngine:
    """Algorithm engine for room segmentation and wall detection."""

    implementation_id = IMPLEMENTATION_ID
    implementation_version = IMPLEMENTATION_VERSION

    def __init__(self, config: Optional[SegmentationConfig] = None) -> None:
        self.config = config or SegmentationConfig()
        self.config.validate()

    def segment(
        self,
        source_map: SourceMap,
        cleanable_mask: Optional[np.ndarray] = None,
        *,
        include_diagnostics: bool = False,
        progress: Optional[Callable[[str, float], None]] = None,
        cancelled: Optional[Callable[[], bool]] = None,
    ) -> SegmentationResult:
        progress = progress or (lambda _stage, _val: None)
        cancelled = cancelled or (lambda: False)
        cfg = self.config

        cleanable = effective_cleanable_mask(source_map, cleanable_mask)
        if not cleanable.any():
            raise SegmentationError('map contains no cleanable cells')

        original_image = self._source_image(source_map, cleanable)
        progress('fft_preprocessing', 0.1)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        # Stage 1: FFT structural filtering and dominant directions.
        rose = FFTStructureExtraction(
            original_image,
            peak_height=cfg.fft_peak_height,
            par=cfg.fft_band_width,
        )
        try:
            rose.process_map()
            if len(rose.main_directions) <= 2:
                raise SegmentationError('ROSE found fewer than two dominant direction pairs')
            rose.simple_filter_map(cfg.filter_level)
        except SegmentationError:
            raise
        except Exception as exc:
            raise SegmentationError(f'FFT structural preprocessing failed: {exc}') from exc

        clean_image = np.asarray(rose.analysed_map, dtype=np.uint8)
        clean_image = clean_image[:source_map.height, :source_map.width]
        directions = list(rose.main_directions)

        progress('layout_extraction', 0.45)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        # Stage 2: Hough walls, angular/spatial clustering, extended lines.
        walls, _ = start_canny_and_hough(
            clean_image,
            rho=cfg.hough_rho,
            theta=cfg.hough_theta,
            threshold=cfg.hough_threshold,
            min_line_length=cfg.hough_min_line_length,
            max_line_gap=cfg.hough_max_line_gap,
        )
        if not walls:
            raise SegmentationError('no structural wall segments detected')

        extremes = find_extremes(walls)
        offset = cfg.offset_px
        xmin = extremes[0] - offset
        xmax = extremes[1] + offset
        ymin = extremes[2] - offset
        ymax = extremes[3] + offset
        if xmin < 0:
            xmin = 0.0
        if ymin < 0:
            ymin = 0.0
        if xmax > source_map.width:
            xmax = float(source_map.width)
        if ymax > source_map.height:
            ymax = float(source_map.height)

        _contour, vertices = external_contour(original_image)
        if vertices is None:
            raise SegmentationError('map has no free-space contour')

        _indexes, walls, _angular_clusters = cluster_ang(
            cfg.mean_shift_bandwidth,
            cfg.mean_shift_min_offset,
            walls,
            diagonals=cfg.diagonals,
        )
        assign_orebro_direction(directions, walls)
        wall_clusters = get_wall_clusters(walls, [w.angular_cluster for w in walls])
        representatives = get_representatives(
            walls, [c for c in wall_clusters if c != -1])
        representatives = spatial_clustering(
            cfg.spatial_clustering_line_segments_threshold, representatives)
        spatial_clusters = new_spatial_cluster(
            walls, representatives, cfg.spatial_clustering_line_segments_threshold)

        ext_lines = create_extended_lines(spatial_clusters, walls, xmin, ymin)
        ext_segs = create_extended_segments(xmin, xmax, ymin, ymax, ext_lines)
        ext_segs = set_weights(ext_segs, walls)
        merged_ext = merge_together(ext_segs, cfg.lines_distance_px, walls)
        merged_ext = set_weights(merged_ext, walls)
        set_weight_offset(merged_ext, xmax, xmin, ymax, ymin)

        retained_ext, ex_li_removed = remove_less_representatives(
            merged_ext, cfg.lines_threshold)

        # Recover wall-supported spans for lines dropped by the threshold.
        short_lines = []
        for line in ex_li_removed:
            short_line = create_short_ex_lines(
                line, walls, (source_map.width, source_map.height), retained_ext)
            if short_line is not None:
                short_lines.append(short_line)
        short_lines = set_weights(short_lines, walls)
        short_lines, _ = remove_less_representatives(short_lines, 0.1)
        for short_line in short_lines:
            retained_ext.append(short_line)
        # Upstream re-filters by lines_threshold with live in-place removal;
        # the resulting skip pattern is preserved for parity.
        for seg in retained_ext:
            if (seg.weight or 0.0) < cfg.lines_threshold:
                retained_ext.remove(seg)

        edges = create_edges(retained_ext)
        edges = set_weights(edges, walls, structural_image=clean_image)
        edges = [e for e in edges if (e.weight or 0.0) >= cfg.edges_threshold]

        cells = create_cells(edges)
        if not cells:
            raise SegmentationError('failed to construct atomic topological cells')

        progress('cell_clustering', 0.7)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        # Stage 3: surface classification, affinity matrix, DBSCAN rooms.
        cells, _cells_out, polygons = classification_surface(
            vertices, cells, cfg.division_threshold)
        if not cells:
            raise SegmentationError('no valid cells found inside layout boundary')

        cells, polygons = remove_frame_fringes(
            cells, polygons, (xmin, ymin, xmax, ymax))

        x_dist = create_matrices(
            cells, sigma=cfg.sigma, hard_wall_threshold=cfg.hard_wall_threshold)
        clusters = dbscan_cluster_cells(cfg.dbscan_eps, cfg.dbscan_min_pts, x_dist)
        rooms = merge_cells(clusters, cells, polygons)
        if not rooms:
            raise SegmentationError('clustering produced no room polygons')

        progress('canonicalizing', 0.85)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        # Stage 4: rasterization, geodesic coverage, canonicalization.
        labels = rasterize_rooms(rooms, source_map.cells.shape)
        if cfg.enable_geodesic_coverage:
            labels, cleanable = geodesic_coverage(
                labels, source_map, cleanable, cfg.min_component_size)

        labels, regions, cleanable = canonicalize_labels(labels, source_map, cleanable)
        if not regions:
            raise SegmentationError('room polygons contain no cleanable cells')

        detected_walls = extract_walls(source_map, retained_ext)

        diagnostics: tuple[DiagnosticImage, ...] = ()
        if include_diagnostics:
            provisional = SegmentationResult(
                labels=labels,
                regions=regions,
                cleanable_mask=cleanable,
                implementation_id=self.implementation_id,
                implementation_version=self.implementation_version,
            )
            diagnostics = build_diagnostics(source_map, provisional, clean_image, retained_ext)

        result = SegmentationResult(
            labels=labels,
            regions=regions,
            cleanable_mask=cleanable,
            implementation_id=self.implementation_id,
            implementation_version=self.implementation_version,
            walls=detected_walls,
            diagnostics=diagnostics,
        )
        validate_result(result, source_map)
        if cancelled():
            raise SegmentationError('segmentation cancelled')

        progress('complete', 1.0)
        return result

    @staticmethod
    def _source_image(source_map: SourceMap, cleanable: np.ndarray) -> np.ndarray:
        """Trinary pipeline image: 255 free, 0 occupied/excluded, 200 unknown."""
        image = np.full(source_map.cells.shape, 200, dtype=np.uint8)
        image[source_map.occupied_mask()] = 0
        image[source_map.free_mask()] = 255
        image[source_map.free_mask() & ~cleanable] = 0
        return image
