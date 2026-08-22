"""render_map CLI: load a map -> print identity/statistics -> write rendered PNGs.

Usage (inside the package directory)::

    python3 -m oomwoo_cleaning_jobs_core.render_map MAP.yaml [--segment]

Also available as ``oomwoo-render-map`` after a colcon install.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from .map_io import load_map_file
from .render import render_segmentation, render_source_map
from .segmentation import SegmentationParams, segment


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='render_map',
        description='Load a nav2 trinary map, print metadata/identity/statistics and render PNGs')
    p.add_argument('map_yaml', help='path to map.yaml')
    p.add_argument('--out', help='base map output PNG (default <map>.render.png)')
    p.add_argument('--segment', action='store_true', help='also run auto segmentation and write an overlay image')
    p.add_argument('--seg-out', help='segmentation overlay output PNG (default <map>.segments.png)')
    p.add_argument('--scale', type=int, default=1, help='nearest-neighbor upscale factor (default 1)')
    p.add_argument('--min-region-area', type=float, default=1.0,
                   help='minimum region area in m^2 (default 1.0)')
    p.add_argument('--marker-neighborhood', type=float, default=0.7,
                   help='local-maximum window diameter in m (default 0.7)')
    p.add_argument('--min-peak-height', type=float, default=0.17,
                   help='minimum peak distance value in m (default 0.17)')
    p.add_argument('--saddle-merge-ratio', type=float, default=0.8,
                   help='saddle merge threshold (default 0.8; >1.0 effectively disables)')
    return p


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    yaml_path = Path(args.map_yaml)

    source_map = load_map_file(yaml_path)
    total = source_map.width * source_map.height
    free = int(source_map.free_mask().sum())
    occupied = int(source_map.occupied_mask().sum())
    unknown = int(source_map.unknown_mask().sum())

    print(f'map file : {yaml_path}')
    print(f'size     : {source_map.width}x{source_map.height} cells '
          f'({source_map.width * source_map.resolution:.2f} x '
          f'{source_map.height * source_map.resolution:.2f} m) '
          f'@ {source_map.resolution} m/cell')
    print(f'origin   : {source_map.origin}')
    print(f'identity : {source_map.identity}')
    print(f'short id : {source_map.short_id}')
    print(f'cells    : free {free} ({free / total:.1%}), '
          f'occupied {occupied} ({occupied / total:.1%}), '
          f'unknown {unknown} ({unknown / total:.1%})')

    out = Path(args.out) if args.out else yaml_path.with_suffix('.render.png')
    assert cv2.imwrite(str(out), render_source_map(source_map, scale=args.scale))
    print(f'base map : {out}')

    if args.segment:
        params = SegmentationParams(
            min_region_area_m2=args.min_region_area,
            marker_neighborhood_m=args.marker_neighborhood,
            min_peak_height_m=args.min_peak_height,
            saddle_merge_ratio=args.saddle_merge_ratio,
        )
        result = segment(source_map, params)
        print(f'candidates : {len(result.regions)}')
        for region in result.regions:
            conf = 'low' if region.low_confidence else 'normal'
            print(f'  #{region.label}: {region.area_m2:.2f} m^2 '
                  f'({region.cell_count} cells), confidence={conf}')
        unclassified = int(result.unclassified_free_mask.sum())
        print(f'unclassified : {unclassified} cells '
              f'({unclassified / max(free, 1):.1%} of free)')
        print(f'doorways : {len(result.doorways)} topology edges')
        for doorway in result.doorways:
            a, b = doorway.regions
            mark = 'door' if doorway.likely_door else 'slit'
            print(f'  #{a} <-> #{b}: width {doorway.width_m:.2f} m, '
                  f'clearance {doorway.clearance_m:.2f} m, '
                  f'ratio {doorway.ratio:.2f} [{mark}]')
        seg_out = (Path(args.seg_out) if args.seg_out
                   else yaml_path.with_suffix('.segments.png'))
        assert cv2.imwrite(
            str(seg_out), render_segmentation(source_map, result, scale=args.scale))
        print(f'segments : {seg_out}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
