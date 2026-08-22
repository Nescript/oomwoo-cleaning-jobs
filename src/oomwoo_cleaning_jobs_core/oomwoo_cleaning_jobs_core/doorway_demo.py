"""Experimental demo of an alternative approach: skeleton + doorway-cut
room segmentation (for comparison against the maximin flooding approach in
segmentation.py; not finalized, not part of the core pipeline).

Pipeline (matching the structure proposed by the user):
1. Preprocess: close the blocked mask (3x3) to seal broken walls; unknown
   counts as not cleanable.
2. Distance transform: EDT of the free mask, giving each free cell's
   clearance.
3. Skeleton: Zhang-Suen thinning of the free space; skeleton-point
   clearance = dist value.
4. Doorway candidates: cluster low skeleton points with
   clearance <= door_saddle_max; at each cluster's narrowest point, extend
   a cut line along the skeleton normal, ending at non-free cells.
5. Scoring filter: door width must be within [min_door_width, max_door_width].
6. Region labeling: subtract cut lines from free, then label connected
   components.
7. Postprocess: regions smaller than min_region_area merge into the
   neighbor sharing the longest boundary.

Usage::

    python3 -m oomwoo_cleaning_jobs_core.doorway_demo MAP.yaml --out-prefix demo/x
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage

from .map_io import load_map_file
from .render import COLOR_FREE, COLOR_OCCUPIED, COLOR_UNKNOWN, region_color
from .source_map import DEFAULT_FREE_THRESH, SourceMap


@dataclass(frozen=True)
class DoorwayParams:
    free_thresh: float = DEFAULT_FREE_THRESH
    #: Skeleton points with clearance <= this value are narrow-passage
    #: candidates (~ half the maximum door width)
    door_saddle_max_m: float = 0.35
    min_door_width_m: float = 0.30
    max_door_width_m: float = 1.30
    max_cut_len_m: float = 1.50
    min_region_area_m2: float = 1.0
    #: Preprocess: close the blocked mask to seal broken walls. Note: with
    #: 1-cell thin walls this erodes the wall network apart at doorway
    #: junctions; keep it off for synthetic/clean maps and on for real noisy
    #: maps.
    close_gaps: bool = False
    #: Wall support: a non-free connected component must be at least this
    #: large (cells) to count as a wall segment
    wall_support_min_cells: int = 20


@dataclass
class Doorway:
    center: tuple[int, int]  # (row, col)
    line: list[tuple[int, int]]
    width_m: float
    wall_support: bool
    side_areas: tuple[int, int]  # free area on both sides (cells)
    accepted: bool


@dataclass
class DoorwayResult:
    labels: np.ndarray
    regions: list[int]
    free: np.ndarray
    skeleton: np.ndarray
    doorways: list[Doorway]


def _zhang_suen_thinning(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning (vectorized numpy); input and output are 0/1 uint8."""
    img = mask.astype(np.uint8).copy()
    changed = True
    while changed:
        changed = False
        for sub in (0, 1):
            p = np.pad(img, 1)
            p2, p3 = p[:-2, 1:-1], p[:-2, 2:]
            p4, p5 = p[1:-1, 2:], p[2:, 2:]
            p6, p7 = p[2:, 1:-1], p[2:, :-2]
            p8, p9 = p[1:-1, :-2], p[:-2, :-2]
            b = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            seq = (p2, p3, p4, p5, p6, p7, p8, p9, p2)
            a = sum(((seq[i] == 0) & (seq[i + 1] == 1)) for i in range(8))
            if sub == 0:
                kill = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) \
                    & (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                kill = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) \
                    & (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            if kill.any():
                img[kill] = 0
                changed = True
    return img


def _free_run(
    free: np.ndarray,
    center: tuple[int, int],
    direction: tuple[float, float],
    max_cells: int,
) -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """Walk from center in both directions along direction, collecting free cells.

    Returns (line, end_cells): end_cells is the first non-free cell where
    each side's walk terminated (the precise landing points for the wall
    support check).
    """
    h, w = free.shape
    line: list[tuple[int, int]] = []
    ends: list[tuple[int, int]] = []
    for sign in (1.0, -1.0):
        r, c = float(center[0]), float(center[1])
        for _ in range(max_cells):
            r += sign * direction[0]
            c += sign * direction[1]
            ri, ci = int(round(r)), int(round(c))
            if not (0 <= ri < h and 0 <= ci < w):
                break
            if not free[ri, ci]:
                ends.append((ri, ci))
                break
            if (ri, ci) not in line:
                line.append((ri, ci))
    return line, ends


def _side_area(free: np.ndarray, start: tuple[int, int], radius: int) -> int:
    """Bounded flood within free from start (Chebyshev radius); returns the area."""
    h, w = free.shape
    r, c = start
    if not (0 <= r < h and 0 <= c < w) or not free[r, c]:
        return 0
    r0, r1 = max(0, r - radius), min(h, r + radius + 1)
    c0, c1 = max(0, c - radius), min(w, c + radius + 1)
    window = free[r0:r1, c0:c1]
    labeled, _ = ndimage.label(window, structure=np.ones((3, 3)))
    return int((labeled == labeled[r - r0, c - c0]).sum())


def find_doorways(
    free: np.ndarray,
    dist: np.ndarray,
    skeleton: np.ndarray,
    support: np.ndarray,
    params: DoorwayParams,
    res: float,
    min_side_cells: int,
) -> list[Doorway]:
    # Narrow-passage candidates = skeleton points whose clearance is below
    # the door-saddle threshold and that are local minima along the skeleton
    # (a doorway is a 2D saddle, not a 2D minimum: clearance always drops
    # toward the walls, so compare within the skeleton point set only).
    # Take local minima first, then cluster: avoids one long low-clearance
    # skeleton segment fusing into a giant cluster (a door chained to a
    # furniture gap) that would swallow the real door.
    skel_dist = np.where(skeleton, dist, np.inf)
    local_min = dist == ndimage.minimum_filter(skel_dist, size=9)
    candidates = skeleton & (dist <= params.door_saddle_max_m) & local_min
    clusters, n = ndimage.label(candidates, structure=np.ones((3, 3)))
    doorways: list[Doorway] = []
    max_cells = int(params.max_cut_len_m / res)
    # Direction sampling: the shortest free run through the candidate point
    # is its door cross-section direction. More stable than cluster PCA
    # (the PCA major axis of thin/corner clusters is unreliable).
    directions = [(float(np.sin(t)), float(np.cos(t)))
                  for t in np.linspace(0, np.pi, 16, endpoint=False)]
    for cluster_id in range(1, n + 1):
        pts = np.argwhere(clusters == cluster_id)
        clearances = dist[pts[:, 0], pts[:, 1]]
        center = tuple(int(v) for v in pts[np.argmin(clearances)])
        best_line: list[tuple[int, int]] | None = None
        best_ends: list[tuple[int, int]] = []
        best_dir = (1.0, 0.0)
        for direction in directions:
            line, ends = _free_run(free, center, direction, max_cells)
            if best_line is None or len(line) < len(best_line):
                best_line, best_ends, best_dir = line, ends, direction
        line = best_line or []
        width_m = (len(line) + 1) * res
        # Wall support: both end cells of the cut line must land on a
        # sufficiently large non-free component (a wall segment; furniture /
        # isolated unknown blobs are small fragments). 1-cell walls shatter
        # into 10-40-cell segments at doorway junctions, so the threshold is
        # a cell count rather than "the largest component".
        ends_ok = len(best_ends) == 2 and all(support[r, c] for r, c in best_ends)
        # Free space on both sides: both sides of a door must have enough
        # free area (filters out corner slivers)
        side_areas = (0, 0)
        if len(line) >= 1:
            mid = line[len(line) // 2]
            # Lateral = normal of the cut-line direction
            areas = []
            for sign in (1.0, -1.0):
                sr = int(round(mid[0] + sign * (-best_dir[1]) * 3))
                sc = int(round(mid[1] + sign * best_dir[0] * 3))
                areas.append(_side_area(free, (sr, sc), radius=40))
            side_areas = (areas[0], areas[1])
        accepted = (params.min_door_width_m <= width_m <= params.max_door_width_m
                    and ends_ok
                    and min(side_areas) >= min_side_cells)
        doorways.append(Doorway(center=center, line=line, width_m=width_m,
                                wall_support=ends_ok, side_areas=side_areas,
                                accepted=accepted))
    return doorways


def segment_doorway(source_map: SourceMap, params: DoorwayParams | None = None) -> DoorwayResult:
    params = params or DoorwayParams()
    res = source_map.resolution
    free = source_map.free_mask(params.free_thresh)
    # Preprocess (optional): close the blocked mask to seal 1-cell broken walls
    if params.close_gaps:
        blocked = ndimage.binary_closing(~free, structure=np.ones((3, 3)))
        free = ~blocked

    dist = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * res
    skeleton = _zhang_suen_thinning(free).astype(bool)
    # Main wall structure = the largest non-free component (outer walls +
    # outside unknown connect into one; furniture / isolated unknown blobs
    # are small standalone components)
    non_free_components, n_comp = ndimage.label(~free, structure=np.ones((3, 3)))
    # Wall support = non-free components meeting the cell count (wall segments)
    non_free_components, _n_comp = ndimage.label(~free, structure=np.ones((3, 3)))
    comp_sizes = np.bincount(non_free_components.ravel())
    support = comp_sizes[non_free_components] >= params.wall_support_min_cells
    min_side_cells = int(0.5 * params.min_region_area_m2 / (res * res))
    doorways = find_doorways(free, dist, skeleton, support, params, res, min_side_cells)

    cut = free.copy()
    for doorway in doorways:
        if doorway.accepted:
            cells = [doorway.center, *doorway.line]
            for r, c in cells:
                # 3x3 thickening: seals diagonal leakage, paired with
                # 4-connected region labeling
                r0, r1 = max(0, r - 1), min(cut.shape[0], r + 2)
                c0, c1 = max(0, c - 1), min(cut.shape[1], c + 2)
                cut[r0:r1, c0:c1] = False

    cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.int32)
    labels, _n = ndimage.label(cut, structure=cross)
    # Postprocess: small regions merge into the longest-boundary neighbor
    min_cells = params.min_region_area_m2 / (res * res)
    kernel = np.ones((3, 3), dtype=np.uint8)
    changed = True
    while changed:
        changed = False
        for value in list(np.unique(labels)):
            if value == 0:
                continue
            mask = labels == value
            if mask.sum() >= min_cells:
                continue
            dilated = cv2.dilate(mask.astype(np.uint8), kernel) > 0
            border = dilated & ~mask & (labels > 0)
            neighbors, counts = np.unique(labels[border], return_counts=True)
            if len(neighbors) == 0:
                labels[mask] = 0
            else:
                labels[mask] = int(neighbors[np.argmax(counts)])
            changed = True

    regions = sorted(int(v) for v in np.unique(labels) if v != 0)
    return DoorwayResult(
        labels=labels, regions=regions, free=free,
        skeleton=skeleton, doorways=doorways)


def _base_img(result: DoorwayResult) -> np.ndarray:
    img = np.full((*result.free.shape, 3), COLOR_UNKNOWN, dtype=np.uint8)
    img[result.free] = COLOR_FREE
    img[~result.free & (result.labels >= 0)] = img[~result.free]  # keep unknown gray
    return img


def render_stages(source_map: SourceMap, result: DoorwayResult, scale: int = 2) -> np.ndarray:
    """Triptych: skeleton+doorway candidates | accepted cut lines | final regions."""
    occupied = source_map.occupied_mask()

    stage1 = np.full((*result.free.shape, 3), COLOR_UNKNOWN, dtype=np.uint8)
    stage1[result.free] = COLOR_FREE
    stage1[occupied] = COLOR_OCCUPIED
    stage1[result.skeleton] = (0, 0, 255)  # skeleton red
    for d in result.doorways:
        stage1[d.center] = (0, 255, 255)   # doorway candidate center yellow

    stage2 = stage1.copy()
    for d in result.doorways:
        color = (255, 0, 0) if d.accepted else (192, 192, 192)  # blue=accepted gray=rejected
        stage2[d.center] = color
        for r, c in d.line:
            stage2[r, c] = color

    stage3 = np.full((*result.free.shape, 3), COLOR_UNKNOWN, dtype=np.uint8)
    stage3[result.free] = COLOR_FREE
    stage3[occupied] = COLOR_OCCUPIED
    stage3f = stage3.astype(np.float64)
    for label in result.regions:
        mask = result.labels == label
        color = np.array(region_color(label), dtype=np.float64)
        stage3f[mask] = 0.45 * stage3f[mask] + 0.55 * color
    stage3 = stage3f.astype(np.uint8)

    def finish(img: np.ndarray, title: str) -> np.ndarray:
        img = img[::-1, :]  # cells row order -> image row order
        if scale != 1:
            img = cv2.resize(img, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_NEAREST)
        img = np.ascontiguousarray(img)
        cv2.putText(img, title, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 255), 1, cv2.LINE_AA)
        return img

    panels = [finish(stage1, 'skeleton+doorways'),
              finish(stage2, 'cuts'),
              finish(stage3, 'regions')]
    pad = np.full((panels[0].shape[0], 4, 3), 32, dtype=np.uint8)
    sheet = np.hstack([panels[0], pad, panels[1], pad, panels[2]])
    return sheet


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Skeleton + doorway-cut segmentation demo')
    parser.add_argument('map_yaml')
    parser.add_argument('--out-prefix', required=True)
    parser.add_argument('--scale', type=int, default=2)
    args = parser.parse_args(argv)

    source_map = load_map_file(args.map_yaml)
    result = segment_doorway(source_map)
    sheet = render_stages(source_map, result, scale=args.scale)
    out = Path(f'{args.out_prefix}.doorway.png')
    out.parent.mkdir(parents=True, exist_ok=True)
    assert cv2.imwrite(str(out), sheet)

    print(f'{args.map_yaml}: {len(result.regions)} regions, '
          f'{sum(1 for d in result.doorways if d.accepted)}/{len(result.doorways)} doorways accepted')
    for d in result.doorways:
        reason = ''
        if not d.accepted:
            if not d.wall_support:
                reason = 'no wall support'
            elif min(d.side_areas) < 1:
                reason = 'side area too small'
            else:
                reason = 'width out of range'
        mark = 'accepted' if d.accepted else f'rejected({reason})'
        print(f'  doorway@{d.center} width {d.width_m:.2f} m '
              f'side areas {d.side_areas} [{mark}]')
    print(f'image: {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
