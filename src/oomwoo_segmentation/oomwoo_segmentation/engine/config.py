"""Configuration and validation for the native room segmentation engine."""

from __future__ import annotations

from dataclasses import dataclass

from ..models import SegmentationError


@dataclass(frozen=True)
class SegmentationConfig:
    """Parameter set for the native room segmentation pipeline.

    Defaults follow the pinned upstream ROSE.launch / ParameterObj profile
    plus the documented OOMWOO compatibility adjustments.
    """

    # FFT structural filtering
    filter_level: float = 0.18
    fft_peak_height: float = 0.2
    fft_band_width: int = 50

    # Hough wall extraction
    # NOTE: upstream calls
    #   cv2.HoughLinesP(img, rho, theta, 25, 10, 5)
    # positionally, and the OpenCV Python binding maps those extra
    # positional args to (lines, minLineLength) with maxLineGap left at
    # its default. The values that actually took effect upstream are
    # therefore minLineLength=5 and maxLineGap=0, NOT the documented
    # 10/5. These defaults reproduce the effective upstream behavior so
    # the native port matches the benchmark baseline.
    hough_rho: float = 1.0
    hough_theta: float = 0.017453292519943295
    hough_threshold: int = 25
    hough_min_line_length: int = 5
    hough_max_line_gap: int = 0

    # Angular / spatial clustering
    mean_shift_bandwidth: float = 0.023
    mean_shift_min_offset: float = 1e-05
    diagonals: bool = False
    spatial_clustering_line_segments_threshold: float = 5.0

    # Extended lines and edges
    offset_px: float = 20.0
    lines_threshold: float = 0.22
    lines_distance_px: float = 20.0
    edges_threshold: float = 0.0

    # Cell classification and clustering
    division_threshold: float = 5.0
    sigma: float = 0.125
    hard_wall_threshold: float = 0.40
    dbscan_eps: float = 0.85
    dbscan_min_pts: int = 1

    # Coverage completion
    min_component_size: int = 10
    enable_geodesic_coverage: bool = True

    def validate(self) -> None:
        for name in ('filter_level', 'lines_threshold', 'edges_threshold', 'hard_wall_threshold'):
            value = float(getattr(self, name))
            if not 0.0 <= value <= 1.0:
                raise SegmentationError(f'{name} must be between 0 and 1')
        if self.fft_band_width <= 0:
            raise SegmentationError('fft_band_width must be positive')
        if self.lines_distance_px < 0:
            raise SegmentationError('lines_distance_px must be non-negative')
        if self.min_component_size < 0:
            raise SegmentationError('min_component_size must be non-negative')
