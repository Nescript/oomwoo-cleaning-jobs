"""Batch-run the ROSE2 provider over test map fixtures and write outputs to ``output/``.

Rule (see docs/DEVELOPMENT.md): test inputs live under
``src/oomwoo_rose2/test/maps/`` and every test/verification run writes its
artifacts under the repository-root ``output/`` directory, one subdirectory
per map.

Usage:

    python3 src/oomwoo_rose2/test/run_map_batch.py \
        src/oomwoo_rose2/test/maps/rose2_upstream \
        --output-root output/rose2_upstream

    # demo render images (upscaled for display) are restored by their exact
    # integer block factor, mirroring test_docs_maps.py:
    python3 src/oomwoo_rose2/test/run_map_batch.py \
        src/oomwoo_rose2/test/maps/demo/corridor4.render.png \
        --embedded-scale 3 --output-root output

Inputs may be nav2 ``map.yaml`` files, directories scanned recursively for
``*.yaml``, or plain images (``.png``/``.pgm``). Image inputs are converted
with the demo-fixture trinary convention (>=250 free, <=10 occupied, rest
unknown, vertically flipped) after downsampling by ``--embedded-scale`` at
0.05 m/cell.

Each ``<map>/`` output directory contains ``source.render.png``,
``segments.png``, ``walls.png`` (detected-wall overlay colored by support),
``diagnostics/`` (cleaned map, extended lines, labels
overlay), and ``run.txt`` with sizes, room areas, detected walls, unassigned
cells, and the provider version. Maps that fail to load or segment are recorded in
``run.txt`` and the summary instead of aborting the batch.
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

import cv2
import numpy as np

from oomwoo_rose2.engine import Rose2Segmenter
from oomwoo_segmentation.map_io import load_map_file
from oomwoo_segmentation.render import (
    render_segmentation,
    render_source_map,
    render_walls,
)
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_segmentation.validation import validate_result

_REPOSITORY = Path(__file__).resolve().parents[3]

_IMAGE_SUFFIXES = {'.png', '.pgm'}


def _source_from_render_image(path: Path, embedded_scale: int) -> SourceMap:
    """Restore a demo render image to algorithm input (test_docs_maps rule)."""
    pixels = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise ValueError(f'{path}: failed to read image')
    pixels = pixels[::embedded_scale, ::embedded_scale]
    cells = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    cells[pixels >= 250] = FREE
    cells[pixels <= 10] = OCCUPIED
    cells = np.ascontiguousarray(cells[::-1, :])
    height, width = cells.shape
    return SourceMap(0.05, width, height, (0.0, 0.0, 0.0), cells)

_DIAGNOSTIC_NAMES = {
    'cleaned_map': '01_cleaned_map.png',
    'extended_lines': '02_extended_lines.png',
    'labels_overlay': '03_labels_overlay.png',
}


def _collect_map_inputs(inputs: list[str]) -> list[Path]:
    paths: list[Path] = []
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            # directories yield map.yaml files only; their referenced images
            # must not become separate duplicate inputs
            paths.extend(sorted(path.rglob('*.yaml')))
        else:
            paths.append(path)
    return paths


def _run_one(input_path: Path, output_root: Path, segmenter: Rose2Segmenter,
             embedded_scale: int) -> str:
    relative = input_path.relative_to(_REPOSITORY) if input_path.is_relative_to(
        _REPOSITORY) else input_path
    scene = input_path.stem
    scene = scene[:-len('.render')] if scene.endswith('.render') else scene
    if input_path.parent.name and input_path.parent.name not in ('maps', 'demo'):
        scene = f'{input_path.parent.name}-{scene}'
    out_dir = output_root / scene
    out_dir.mkdir(parents=True, exist_ok=True)
    lines = [f'map file : {relative}']

    try:
        if input_path.suffix == '.yaml':
            source = load_map_file(input_path)
        else:
            source = _source_from_render_image(input_path, embedded_scale)
            lines.append(f'normalization : embedded_render_scale={embedded_scale}')
    except Exception as exc:
        lines.append(f'error     : failed to load map: {exc}')
        (out_dir / 'run.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
        return f'{scene}: LOAD FAILED ({exc})'

    free = int(source.free_mask().sum())
    total = source.width * source.height
    lines.append(f'size     : {source.width}x{source.height} cells @ '
                 f'{source.resolution} m/cell')
    lines.append(f'origin   : {tuple(source.origin)}')
    lines.append(f'identity : {source.identity}')
    lines.append(f'cells    : free {free} ({100.0 * free / total:.1f}%), '
                 f'occupied {int(source.occupied_mask().sum())}, '
                 f'unknown {int(source.unknown_mask().sum())}')

    base_path = out_dir / 'source.render.png'
    cv2.imwrite(str(base_path), render_source_map(source))
    lines.append(f'base map : {base_path.relative_to(_REPOSITORY)}')

    try:
        result = segmenter.segment(source, include_diagnostics=True)
        validate_result(result, source)
    except Exception as exc:
        lines.append(f'error     : segmentation failed: {exc}')
        lines.append(traceback.format_exc(limit=3))
        (out_dir / 'run.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
        return f'{scene}: SEGMENTATION FAILED ({exc})'

    lines.append(f'implementation : {result.implementation_id} '
                 f'{result.implementation_version}')
    lines.append(f'rooms : {len(result.regions)}')
    lines.append(f'walls : {len(result.walls)}')
    for wall in result.walls:
        lines.append(f'  ({wall.x1:.2f},{wall.y1:.2f})-({wall.x2:.2f},{wall.y2:.2f})'
                     f' support={wall.support:.3f} dir={wall.direction_rad:.3f}')
    for region in result.regions:
        cells = int(result.mask_of(region.label).sum())
        lines.append(f'  #{region.label}: '
                     f'{cells * source.resolution ** 2:.2f} m^2 ({cells} cells)')
    unassigned = int(result.unassigned_cleanable_mask.sum())
    lines.append(f'unassigned : {unassigned} cells '
                 f'({100.0 * unassigned / max(free, 1):.1f}% of free)')

    segments_path = out_dir / 'segments.png'
    cv2.imwrite(str(segments_path), render_segmentation(source, result))
    lines.append(f'segments : {segments_path.relative_to(_REPOSITORY)}')

    walls_path = out_dir / 'walls.png'
    cv2.imwrite(str(walls_path), render_walls(source, result.walls))
    lines.append(f'walls overlay : {walls_path.relative_to(_REPOSITORY)}')

    for diagnostic in result.diagnostics:
        name = _DIAGNOSTIC_NAMES.get(diagnostic.stage, f'{diagnostic.stage}.png')
        diag_path = out_dir / 'diagnostics' / name
        diag_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(diag_path), diagnostic.image)
        lines.append(f'diagnostic : {diag_path.relative_to(_REPOSITORY)}')

    (out_dir / 'run.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return (f'{scene}: {len(result.regions)} rooms, '
            f'{len(result.walls)} walls, '
            f'{unassigned} unassigned ({100.0 * unassigned / max(free, 1):.1f}%)')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+',
                        help='map.yaml files, image files (.png/.pgm), or '
                             'directories to scan recursively')
    parser.add_argument('--output-root', default='output',
                        help='output root, relative to the repository root '
                             '(default: output)')
    parser.add_argument('--embedded-scale', type=int, default=1,
                        help='integer block factor used to restore image '
                             'inputs to algorithm resolution (default: 1)')
    args = parser.parse_args(argv)

    if args.embedded_scale < 1:
        print('--embedded-scale must be >= 1', file=sys.stderr)
        return 1

    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = _REPOSITORY / output_root
    output_root.mkdir(parents=True, exist_ok=True)

    paths = _collect_map_inputs(args.inputs)
    if not paths:
        print('no map.yaml or image files found', file=sys.stderr)
        return 1

    segmenter = Rose2Segmenter()
    summary = []
    for path in paths:
        line = _run_one(path.resolve(), output_root, segmenter,
                        args.embedded_scale)
        print(line, flush=True)
        summary.append(line)

    (output_root / 'summary.txt').write_text('\n'.join(summary) + '\n',
                                             encoding='utf-8')
    failed = [line for line in summary if 'FAILED' in line]
    print(f'\n{len(summary) - len(failed)}/{len(summary)} maps segmented; '
          f'summary written to {output_root / "summary.txt"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
