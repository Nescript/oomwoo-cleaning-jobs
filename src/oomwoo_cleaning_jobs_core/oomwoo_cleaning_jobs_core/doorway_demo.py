"""实验性新方案 demo：骨架 + 门口切割的房间分割（与 segmentation.py 的
maximin 淹没方案对比用，未定稿，不进核心管线）。

流程（对应用户提出的结构）：
1. 预处理：闭合 blocked mask（3x3）封合断裂墙体；unknown 视为不可清扫。
2. 距离变换：free mask 的 EDT，得每个自由 cell 的 clearance。
3. 骨架：Zhang-Suen 细化提取自由空间骨架，骨架点 clearance = dist 值。
4. 门口候选：骨架上 clearance <= door_saddle_max 的低点聚类；每簇最窄点
   处沿骨架法向延伸分割线，两端止于非 free cell。
5. 评分过滤：门宽须在 [min_door_width, max_door_width] 内。
6. 区域标记：割线从 free 中扣除后做连通域标记。
7. 后处理：小于 min_region_area 的区域并入共享边界最长的邻域。

用法::

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
    #: 骨架上 clearance <= 此值视为窄通道候选（≈ 最大门宽的一半）
    door_saddle_max_m: float = 0.35
    min_door_width_m: float = 0.30
    max_door_width_m: float = 1.30
    max_cut_len_m: float = 1.50
    min_region_area_m2: float = 1.0
    #: 预处理：闭合 blocked 封合断裂墙。注意：对 1-cell 薄墙会在门洞
    #: 交叉处把墙网腐蚀碎裂，合成/干净地图应关闭；真实噪声地图可开启。
    close_gaps: bool = False
    #: 墙体支持：非 free 连通体至少这么大（cells）才算墙体段
    wall_support_min_cells: int = 20


@dataclass
class Doorway:
    center: tuple[int, int]  # (row, col)
    line: list[tuple[int, int]]
    width_m: float
    wall_support: bool
    side_areas: tuple[int, int]  # 两侧自由面积（cells）
    accepted: bool


@dataclass
class DoorwayResult:
    labels: np.ndarray
    regions: list[int]
    free: np.ndarray
    skeleton: np.ndarray
    doorways: list[Doorway]


def _zhang_suen_thinning(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen 细化（numpy 向量化），输入输出为 0/1 uint8。"""
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
    """从 center 沿 direction 正负两方向走，收集 free cell。

    返回 (line, end_cells)：end_cells 为两侧各自行进终止处的
    第一个非 free cell（墙体支持度判定的精确落点）。
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
    """从 start 出发在 free 内做限域 flood（Chebyshev 半径 radius），返回面积。"""
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
    # 窄通道候选 = 骨架上 clearance 不超过门鞍阈值、且沿骨架为局部最低点
    # （门口是 2D 鞍部而非 2D 最低点：clearance 朝墙方向必然下降，
    # 因此只在骨架点集内比较）。先取局部最小再聚簇：避免整条低
    # clearance 骨架段串成巨簇（门与家具窄缝串在一起）而覆盖真门。
    skel_dist = np.where(skeleton, dist, np.inf)
    local_min = dist == ndimage.minimum_filter(skel_dist, size=9)
    candidates = skeleton & (dist <= params.door_saddle_max_m) & local_min
    clusters, n = ndimage.label(candidates, structure=np.ones((3, 3)))
    doorways: list[Doorway] = []
    max_cells = int(params.max_cut_len_m / res)
    # 方向采样：取过候选点的最短自由走向为该点的门截面方向。
    # 比簇 PCA 稳定（细长/拐角簇的 PCA 主轴不可靠）。
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
        # 墙体支持度：割线两侧终止 cell 须落在足够大的非 free 连通体上
        # （墙体段；家具/孤立未知块是碎小连通体）。1-cell 墙在门洞交叉处
        # 会碎成 10-40 cell 的段，因此阈值取 cell 数而非「最大组件」。
        ends_ok = len(best_ends) == 2 and all(support[r, c] for r, c in best_ends)
        # 两侧自由空间：门两侧都须有足够的自由面积（过滤墙角碎缝）
        side_areas = (0, 0)
        if len(line) >= 1:
            mid = line[len(line) // 2]
            # 侧向 = 割线方向的法向
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
    # 预处理（可选）：闭合 blocked 封合 1-cell 断裂墙
    if params.close_gaps:
        blocked = ndimage.binary_closing(~free, structure=np.ones((3, 3)))
        free = ~blocked

    dist = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * res
    skeleton = _zhang_suen_thinning(free).astype(bool)
    # 主墙体结构 = 最大的非 free 连通体（外墙+外部 unknown 连成一体；
    # 家具/孤立未知块是独立小连通体）
    non_free_components, n_comp = ndimage.label(~free, structure=np.ones((3, 3)))
    # 墙体支持 = cell 数达标的非 free 连通体（墙体段）
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
                # 3x3 加厚：封住对角泄漏，配合 4-连通区域标记
                r0, r1 = max(0, r - 1), min(cut.shape[0], r + 2)
                c0, c1 = max(0, c - 1), min(cut.shape[1], c + 2)
                cut[r0:r1, c0:c1] = False

    cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.int32)
    labels, _n = ndimage.label(cut, structure=cross)
    # 后处理：小区域并入共享边界最长邻域
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
    img[~result.free & (result.labels >= 0)] = img[~result.free]  # 保持 unknown 灰
    return img


def render_stages(source_map: SourceMap, result: DoorwayResult, scale: int = 2) -> np.ndarray:
    """三联图：骨架+门口候选 | 有效切割线 | 最终区域。"""
    occupied = source_map.occupied_mask()

    stage1 = np.full((*result.free.shape, 3), COLOR_UNKNOWN, dtype=np.uint8)
    stage1[result.free] = COLOR_FREE
    stage1[occupied] = COLOR_OCCUPIED
    stage1[result.skeleton] = (0, 0, 255)  # 骨架 红
    for d in result.doorways:
        stage1[d.center] = (0, 255, 255)   # 门口候选中心 黄

    stage2 = stage1.copy()
    for d in result.doorways:
        color = (255, 0, 0) if d.accepted else (192, 192, 192)  # 蓝=有效 灰=拒绝
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
        img = img[::-1, :]  # cells 行序 → 图像行序
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
    parser = argparse.ArgumentParser(description='骨架+门口切割分割 demo')
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

    print(f'{args.map_yaml}: {len(result.regions)} 个区域, '
          f'{sum(1 for d in result.doorways if d.accepted)}/{len(result.doorways)} 门口有效')
    for d in result.doorways:
        reason = ''
        if not d.accepted:
            if not d.wall_support:
                reason = '无墙体支持'
            elif min(d.side_areas) < 1:
                reason = '一侧面积不足'
            else:
                reason = '宽度越界'
        mark = '有效' if d.accepted else f'拒绝({reason})'
        print(f'  门口@{d.center} 宽 {d.width_m:.2f} m '
              f'两侧面积 {d.side_areas} [{mark}]')
    print(f'图: {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
