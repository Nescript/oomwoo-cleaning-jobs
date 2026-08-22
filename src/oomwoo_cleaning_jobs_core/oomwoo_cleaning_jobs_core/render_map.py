"""render_map CLI：加载地图 → 打印 identity/统计 → 输出渲染 PNG。

用法（包目录下）::

    python3 -m oomwoo_cleaning_jobs_core.render_map MAP.yaml [--segment]

colcon 安装后也可用 ``oomwoo-render-map``。
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
        description='加载 nav2 trinary 地图，打印元数据/identity/统计并渲染 PNG')
    p.add_argument('map_yaml', help='map.yaml 路径')
    p.add_argument('--out', help='底图输出 PNG（默认 <地图名>.render.png）')
    p.add_argument('--segment', action='store_true', help='同时执行自动分割并输出叠加图')
    p.add_argument('--seg-out', help='分割叠加图输出 PNG（默认 <地图名>.segments.png）')
    p.add_argument('--scale', type=int, default=1, help='最近邻放大倍数（默认 1）')
    p.add_argument('--min-region-area', type=float, default=1.0,
                   help='最小区域面积 m²（默认 1.0）')
    p.add_argument('--marker-neighborhood', type=float, default=0.7,
                   help='局部极大值窗口直径 m（默认 0.7）')
    p.add_argument('--min-peak-height', type=float, default=0.17,
                   help='峰最小距离值 m（默认 0.17）')
    p.add_argument('--saddle-merge-ratio', type=float, default=0.8,
                   help='鞍部合并阈值（默认 0.8；>1.0 近似禁用）')
    return p


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    yaml_path = Path(args.map_yaml)

    source_map = load_map_file(yaml_path)
    total = source_map.width * source_map.height
    free = int(source_map.free_mask().sum())
    occupied = int(source_map.occupied_mask().sum())
    unknown = int(source_map.unknown_mask().sum())

    print(f'地图文件 : {yaml_path}')
    print(f'尺寸     : {source_map.width}x{source_map.height} cell '
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
    print(f'底图     : {out}')

    if args.segment:
        params = SegmentationParams(
            min_region_area_m2=args.min_region_area,
            marker_neighborhood_m=args.marker_neighborhood,
            min_peak_height_m=args.min_peak_height,
            saddle_merge_ratio=args.saddle_merge_ratio,
        )
        result = segment(source_map, params)
        print(f'候选区域 : {len(result.regions)} 个')
        for region in result.regions:
            conf = '低置信' if region.low_confidence else '正常'
            print(f'  #{region.label}: {region.area_m2:.2f} m² '
                  f'({region.cell_count} cells), 置信={conf}')
        unclassified = int(result.unclassified_free_mask.sum())
        print(f'未分类   : {unclassified} cells '
              f'({unclassified / max(free, 1):.1%} of free)')
        seg_out = (Path(args.seg_out) if args.seg_out
                   else yaml_path.with_suffix('.segments.png'))
        assert cv2.imwrite(
            str(seg_out), render_segmentation(source_map, result, scale=args.scale))
        print(f'分割图   : {seg_out}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
