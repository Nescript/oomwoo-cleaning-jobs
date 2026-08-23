"""Render a nav2 map and optionally segment it through the shared ROS 2 action."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import cv2

from .client import segment_once
from .map_io import load_map_file
from .render import render_segmentation, render_source_map


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='oomwoo-render-map',
        description='Render a nav2 trinary map and provider-neutral room segmentation',
    )
    parser.add_argument('map_yaml')
    parser.add_argument('--out', help='base-map PNG (default: <map>.render.png)')
    parser.add_argument('--segment', action='store_true', help='request room segmentation')
    parser.add_argument('--seg-out', help='segmentation PNG (default: <map>.segments.png)')
    parser.add_argument('--server', default='/room_segmentation/segment',
                        help='SegmentRooms action name')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--scale', type=int, default=1)
    parser.add_argument('--diagnostics-dir',
                        help='request and save provider diagnostic PNGs')
    return parser


def _safe_stage_name(stage: str, index: int) -> str:
    safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', stage).strip('._')
    return f'{index:02d}_{safe or "stage"}.png'


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
          f'@ {source_map.resolution} m/cell')
    print(f'origin   : {source_map.origin}')
    print(f'identity : {source_map.identity}')
    print(f'cells    : free {free} ({free / max(total, 1):.1%}), '
          f'occupied {occupied}, unknown {unknown}')

    base_path = Path(args.out) if args.out else yaml_path.with_suffix('.render.png')
    if not cv2.imwrite(str(base_path), render_source_map(source_map, scale=args.scale)):
        raise RuntimeError(f'failed to write {base_path}')
    print(f'base map : {base_path}')

    if not args.segment:
        return 0

    result = segment_once(
        source_map,
        action_name=args.server,
        include_diagnostics=bool(args.diagnostics_dir),
        timeout_sec=args.timeout,
    )
    print(f'implementation : {result.implementation_id} '
          f'{result.implementation_version}')
    print(f'rooms : {len(result.regions)}')
    for region in result.regions:
        print(f'  #{region.label}: {region.area_m2:.2f} m^2 '
              f'({region.cell_count} cells)')
    unassigned = int(result.unassigned_cleanable_mask.sum())
    print(f'unassigned : {unassigned} cells '
          f'({unassigned / max(free, 1):.1%} of free)')

    seg_path = (Path(args.seg_out) if args.seg_out
                else yaml_path.with_suffix('.segments.png'))
    overlay = render_segmentation(source_map, result, scale=args.scale)
    if not cv2.imwrite(str(seg_path), overlay):
        raise RuntimeError(f'failed to write {seg_path}')
    print(f'segments : {seg_path}')

    if args.diagnostics_dir:
        diagnostic_dir = Path(args.diagnostics_dir)
        diagnostic_dir.mkdir(parents=True, exist_ok=True)
        for index, diagnostic in enumerate(result.diagnostics, start=1):
            path = diagnostic_dir / _safe_stage_name(diagnostic.stage, index)
            if not cv2.imwrite(str(path), diagnostic.image):
                raise RuntimeError(f'failed to write {path}')
            print(f'diagnostic : {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
