"""自动候选区域分割：距离变换 + 自由空间内的 maximin 淹没分水岭 + 鞍部合并。

对应 docs/DEVELOPMENT.md「第一阶段实施决定 · 自动分割」。流程：

1. free mask（``0 <= v < free_thresh*100``，unknown 视为不可清扫）按连通域处理。
2. 距离变换 + 局部极大值 markers；连通域只有一个峰（大开间）时整体作为
   单一低置信候选。
3. **maximin 淹没**（`_maximin_watershed`）：每个 free cell 归属于
   "到该 cell 的最宽通路"所在的种子。只在自由空间内传播——墙不可穿越，
   因此区域不会溢出到墙另一侧（曾用 `cv2.watershed` 在全图淹没导致越墙）。
4. 小区域合并（`_merge_small_regions`）：小于 ``min_region_area`` 的区域
   并入**连接最宽**（合并树山口最高）的近邻；山口并列时退化为共享边界
   最长者；无近邻则标为未分类。
5. **鞍部合并**（`_merge_by_saddle`）：相邻区域的鞍部（`_connection_values`
   超水平集合并树给出的真实山口高度 = 两区域间最宽通道的半宽，与分界线
   落点无关）不低于 ``saddle_merge_ratio`` × 较小峰高时合并——两区域间
   存在足够宽的通道即同一片开阔空间；真门洞的鞍部显著低于两侧峰高，
   不会被误并。传递闭包用并查集。
6. **门口溢出裁剪**（`_clip_doorway_spills`）：maximin 会把门两侧 dist 低于
   门鞍的 cell 分给对门区域形成溢出带；对每对相邻区域在合并树山口 cell
   处生成切割线，暂时阻断后把落在对方连通体的 cell 重归对方，
   区域边界强制对齐门洞切割线。
7. 脊线标记（`_mark_ridges`）：与不同 label 相邻的 cell 标为未分类
   （watershed 不直接产脊线，接触带两侧各一层 cell 构成未分类边界）。
8. **门口记录**（`_find_doorways`）：对每对相邻区域（测地膨胀接触带判定）
   记录 `Doorway`（山口 center、clearance、width ≈ 2×clearance、ratio、
   likely_door），构成区域拓扑邻接。

未分类 cell（label 0）由 ``unclassified_free_mask`` 显式标出；距离值低于
``min_peak_height_m`` 的局部高地（过窄地带）不会产生种子，也留在未分类。

该算法是「先用它看效果」的初始策略，可替换。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage

from .source_map import DEFAULT_FREE_THRESH, SourceMap

#: labels 数组中"非候选/未分类"的取值
UNCLASSIFIED = 0

_NEIGHBORS_8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))

#: 无环绕的相邻切片对：(a 的切片, b 的切片)，b 是 a 向右/下/右下/左下平移一格
_CONTACT_SLICES = (
    (np.s_[:, :-1], np.s_[:, 1:]),
    (np.s_[:-1, :], np.s_[1:, :]),
    (np.s_[:-1, :-1], np.s_[1:, 1:]),
    (np.s_[:-1, 1:], np.s_[1:, :-1]),
)


@dataclass(frozen=True)
class SegmentationParams:
    free_thresh: float = DEFAULT_FREE_THRESH
    min_region_area_m2: float = 1.0
    #: 局部极大值窗口直径（米），小于此尺度的距离起伏不产生独立房间。
    #: 默认 0.7 m：更大窗口会穿门抑制小房间的峰（导致整个连通域退化为低置信）。
    marker_neighborhood_m: float = 0.7
    #: 峰的最小距离值（米），≈ robot_inscribed_radius；更窄处不成峰
    min_peak_height_m: float = 0.17
    #: 鞍部合并阈值：鞍部 >= ratio × 较小峰高时合并相邻区域。
    #: 1.0 以上等价于禁用；真门洞比值通常 < 0.5，同片开阔地的伪分割 ≥ 1.0。
    saddle_merge_ratio: float = 0.8
    #: 门口记录的 likely_door 判定门宽范围（米）
    min_door_width_m: float = 0.30
    max_door_width_m: float = 1.30
    #: 门口切割线采样时自由走向的最大延伸（米）
    max_door_measure_m: float = 1.50


@dataclass(frozen=True)
class CandidateRegion:
    """自动划分出的候选区域（Candidate Region）。"""

    label: int
    cell_count: int
    area_m2: float
    low_confidence: bool


@dataclass(frozen=True)
class Doorway:
    """两个 Region 间的门口记录（拓扑边）。

    center 是两区域接触带中 dist 最大的 cell（局部山口）；clearance_m 是
    山口高度（瓶颈半宽）；width_m ≈ 2 × clearance（经典近似）；
    ratio = clearance / min(两侧峰高)（越大越像同一片开阔空间）；
    likely_door = width_m 落在门宽范围内。
    """

    regions: tuple[int, int]
    center: tuple[int, int]  # (row, col)，cells 行序
    width_m: float
    clearance_m: float
    ratio: float
    likely_door: bool


@dataclass
class SegmentationResult:
    """分割结果。labels 与 SourceMap.cells 同行序（row 0 = 最底行）。"""

    labels: np.ndarray  # int32，UNCLASSIFIED(0) = 非候选/未分类
    regions: list[CandidateRegion]
    free_mask: np.ndarray
    params: SegmentationParams
    doorways: list[Doorway] = field(default_factory=list)

    @property
    def unclassified_free_mask(self) -> np.ndarray:
        """属于可清扫空间但未划入任何候选区域的 cell。"""
        return self.free_mask & (self.labels == UNCLASSIFIED)

    def mask_of(self, label: int) -> np.ndarray:
        return self.labels == label

    def adjacent_labels(self, label: int) -> set[int]:
        """拓扑邻接：通过门口与该 Region 相连的其他 Region。"""
        out: set[int] = set()
        for doorway in self.doorways:
            a, b = doorway.regions
            if a == label:
                out.add(b)
            elif b == label:
                out.add(a)
        return out


def segment(
    source_map: SourceMap,
    params: SegmentationParams | None = None,
    cleanable_mask: np.ndarray | None = None,
) -> SegmentationResult:
    """自动分割 Source Map 的允许清扫空间。

    ``cleanable_mask`` 供 Keepout 等外部约束收窄候选空间；它不能把
    障碍/未知变成可清扫 cell，且必须与 Source Map 栅格同 shape。
    """
    params = params or SegmentationParams()
    res = source_map.resolution
    source_free = source_map.free_mask(params.free_thresh)
    if cleanable_mask is None:
        free = source_free
    else:
        cleanable_mask = np.asarray(cleanable_mask, dtype=bool)
        if cleanable_mask.shape != source_free.shape:
            raise ValueError('cleanable_mask 与 SourceMap 栅格形状不一致')
        free = source_free & cleanable_mask
    structure = np.ones((3, 3), dtype=np.int32)  # 8-连通

    size_cells = max(3, int(round(params.marker_neighborhood_m / res)) | 1)
    min_cells = params.min_region_area_m2 / (res * res)

    labels = np.zeros(free.shape, dtype=np.int32)
    markers = np.zeros(free.shape, dtype=np.int32)
    low_conf: set[int] = set()
    next_label = 1

    dist = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * res
    components, n_components = ndimage.label(free, structure=structure)
    for comp_id in range(1, n_components + 1):
        comp = components == comp_id
        local_max = ndimage.maximum_filter(dist, size=size_cells)
        peaks = (dist == local_max) & (dist >= params.min_peak_height_m) & comp
        comp_markers, n_markers = ndimage.label(peaks, structure=structure)

        if n_markers <= 1:
            # 退化：整个连通域作为单一低置信候选
            labels[comp] = next_label
            low_conf.add(next_label)
            next_label += 1
            continue

        for k in range(1, n_markers + 1):
            markers[comp_markers == k] = next_label
            next_label += 1

    # maximin 淹没只填充尚未归属的自由 cell（退化连通域已整块标记）
    floodable = free & (labels == UNCLASSIFIED)
    flooded, prio = _maximin_watershed(dist, markers, floodable)
    labels[flooded > UNCLASSIFIED] = flooded[flooded > UNCLASSIFIED]

    labels = _merge_small_regions(labels, dist, free, min_cells, low_conf)
    labels = _merge_by_saddle(labels, dist, free, params.saddle_merge_ratio, low_conf)
    labels = _clip_doorway_spills(labels, dist, free, params, res)
    labels = _mark_ridges(labels)
    doorways = _find_doorways(labels, dist, free, params, res)

    regions = []
    for label in sorted(int(v) for v in np.unique(labels) if v != UNCLASSIFIED):
        cell_count = int((labels == label).sum())
        regions.append(CandidateRegion(
            label=label,
            cell_count=cell_count,
            area_m2=cell_count * res * res,
            low_confidence=label in low_conf,
        ))
    return SegmentationResult(labels=labels, regions=regions, free_mask=free,
                              params=params, doorways=doorways)


def _maximin_watershed(dist: np.ndarray, markers: np.ndarray, free: np.ndarray) -> np.ndarray:
    """自由空间内的 maximin（最宽路径）淹没分水岭。

    每个 free cell 归属于"到该 cell 通路上最小距离值最大"的种子，
    即距离变换面上的集水盆地。洪峰只在 free cell 间传播，墙不可穿越。
    瓶颈值相同时（两盆地相遇 / 平台区）按到种子的测地距离决胜，
    使分界线落在鞍部/门洞处而不是溢出到相邻区域内部；残余的任意
    分界由 `_mark_ridges` 统一标为未分类。
    """
    labels = markers.astype(np.int32).copy()
    prio = np.where(markers > UNCLASSIFIED, dist, -np.inf)
    geo = np.where(markers > UNCLASSIFIED, 0.0, np.inf)
    heap = [(-float(dist[r, c]), 0.0, int(r), int(c)) for r, c in np.argwhere(markers > 0)]
    heapq.heapify(heap)
    h, w = dist.shape
    eps = 1e-9
    while heap:
        neg_p, g, r, c = heapq.heappop(heap)
        p = -neg_p
        if p < prio[r, c] - eps or (p < prio[r, c] + eps and g > geo[r, c] + eps):
            continue  # 已被更优（更宽或同等但更短）的路径更新过
        lab = labels[r, c]
        for dr, dc in _NEIGHBORS_8:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and free[rr, cc]:
                q = min(p, dist[rr, cc])
                gq = g + (1.4142135623730951 if dr and dc else 1.0)
                if q > prio[rr, cc] + eps or (q > prio[rr, cc] - eps and gq < geo[rr, cc] - eps):
                    prio[rr, cc] = q
                    geo[rr, cc] = gq
                    labels[rr, cc] = lab
                    heapq.heappush(heap, (-q, gq, rr, cc))
    return labels, prio


def _merge_small_regions(
    labels: np.ndarray,
    dist: np.ndarray,
    free: np.ndarray,
    min_cells: float,
    low_conf: set[int],
) -> np.ndarray:
    """小于 min_cells 的区域并入邻域；无邻域则标为未分类。

    目标邻域的选择依据**最宽连接**（`_connection_values` 的山口高度）：
    小区域应并入与它连接最宽的区域。若按共享边界长度选择，门洞旁的
    小盆地会穿门漏进相邻房间；按山口选择则留在本房间的主盆地
    （内部连接更宽）。山口并列时退化为边界最长者。
    """
    changed = True
    while changed:
        changed = False
        saddles = _connection_values(labels, dist, free)
        for value in list(np.unique(labels)):
            if value == UNCLASSIFIED:
                continue
            mask = labels == value
            if mask.sum() >= min_cells:
                continue
            candidates = [
                (saddle, b if a == value else a)
                for (a, b), (saddle, _cell, _direct) in saddles.items()
                if a == value or b == value
            ]
            if not candidates:
                labels[mask] = UNCLASSIFIED
            else:
                best_saddle = max(s for s, _ in candidates)
                tied = [n for s, n in candidates if s == best_saddle]
                if len(tied) == 1:
                    target = tied[0]
                else:
                    kernel = np.ones((3, 3), dtype=np.uint8)
                    dilated = cv2.dilate(mask.astype(np.uint8), kernel) > 0
                    border = dilated & ~mask & np.isin(labels, tied)
                    neighbors, counts = np.unique(labels[border], return_counts=True)
                    target = int(neighbors[np.argmax(counts)])
                labels[mask] = target
            low_conf.discard(int(value))
            changed = True
    return labels


def _connection_values(
    labels: np.ndarray,
    dist: np.ndarray,
    free: np.ndarray,
) -> dict[tuple[int, int], tuple[float, tuple[int, int], bool]]:
    """区域对 -> (山口高度 W, 山口 cell, direct)：两区域在 ``dist >= W`` 的
    超水平集内连通的**最大** W，以及首次连通处的 cell（瓶颈点）。

    等价于分水岭合并树（merge tree / persistence）：按 dist 降序扫描
    free cell 并查集合并，首次把两个不同 label 的集合连通时的 dist
    即两区域间最宽通路的瓶颈。与最终分界线落在哪里无关。

    direct=False 表示该连接途经第三方区域（合并集合已含其他 label），
    是传递连接而非直接相邻——合并与门口记录只应使用 direct=True。"""
    h, w = dist.shape
    cells = np.argwhere(free)
    order = np.argsort(-dist[free], kind='stable')
    parent: dict[int, int] = {}
    labelsets: dict[int, set[int]] = {}
    connections: dict[tuple[int, int], tuple[float, tuple[int, int], bool]] = {}

    def find(x: int) -> int:
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    for idx in order:
        r, c = int(cells[idx][0]), int(cells[idx][1])
        x = r * w + c
        parent[x] = x
        labelsets[x] = {int(labels[r, c])} if labels[r, c] > UNCLASSIFIED else set()
        for dr, dc in _NEIGHBORS_8:
            rr, cc = r + dr, c + dc
            if not (0 <= rr < h and 0 <= cc < w and free[rr, cc]):
                continue
            y = rr * w + cc
            if y not in parent:
                continue  # 尚未处理（dist 更低）
            ra, rb = find(x), find(y)
            if ra == rb:
                continue
            direct = len(labelsets[ra]) == 1 and len(labelsets[rb]) == 1
            for la in labelsets[ra]:
                for lb in labelsets[rb]:
                    if la != lb:
                        key = (min(la, lb), max(la, lb))
                        if key not in connections:
                            connections[key] = (float(dist[r, c]), (r, c), direct)
            if len(labelsets[ra]) < len(labelsets[rb]):
                ra, rb = rb, ra
            parent[rb] = ra
            labelsets[ra] |= labelsets[rb]
            del labelsets[rb]
    return connections


def _merge_by_saddle(
    labels: np.ndarray,
    dist: np.ndarray,
    free: np.ndarray,
    ratio: float,
    low_conf: set[int],
) -> np.ndarray:
    """山口高度不低于 ratio × 较小峰高的区域对合并（并查集传递闭包）。

    直觉：两区域间只要存在一条足够宽（相对两者规模）的通道，它们就是
    同一片开阔空间被多个距离峰劈开的伪分割；真门洞的山口显著低于
    两侧峰高，不会被误并。
    """
    values = [int(v) for v in np.unique(labels) if v != UNCLASSIFIED]
    if len(values) < 2:
        return labels
    peaks = {v: float(dist[labels == v].max()) for v in values}

    parent = {v: v for v in values}

    def find(v: int) -> int:
        while parent[v] != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    for (a, b), (saddle, _cell, _direct) in _connection_values(labels, dist, free).items():
        if saddle >= ratio * min(peaks[a], peaks[b]):
            parent[find(a)] = find(b)

    mapping: dict[int, int] = {}
    for v in values:
        root = find(v)
        target = mapping.setdefault(root, min(v, root))
        if v != target:
            low_conf.discard(v)
    if not any(v != find(v) for v in values):
        return labels
    out = labels.copy()
    for v in values:
        target = mapping[find(v)]
        if target != v:
            out[labels == v] = target
    return out


def _geodesic_dilate(mask: np.ndarray, free: np.ndarray, steps: int = 2) -> np.ndarray:
    """在自由空间内膨胀，避免普通膨胀跨过墙体。"""
    kernel3 = np.ones((3, 3), dtype=np.uint8)
    out = mask & free
    for _ in range(steps):
        out = (cv2.dilate(out.astype(np.uint8), kernel3) > 0) & free
    return out


def _contact_saddle_cell(
    contact: np.ndarray,
    dist: np.ndarray,
) -> tuple[int, int]:
    """返回两个 Region 测地接触带内 clearance 最大的 cell。"""
    if contact.shape != dist.shape:
        raise ValueError('contact 与 dist 形状不一致')
    if not contact.any():
        raise ValueError('接触带不能为空')
    contact_dist = np.where(contact, dist, -np.inf)
    return tuple(int(v) for v in np.unravel_index(contact_dist.argmax(), dist.shape))


def _find_doorways(
    labels: np.ndarray,
    dist: np.ndarray,
    free: np.ndarray,
    params: SegmentationParams,
    res: float,
) -> list[Doorway]:
    """为最终 Region 拓扑生成门口记录。

    门口区域对 = 最终标记中空间相邻（接触带 <=2 cell 的脊线/未分类）的
    区域对。山口 center 与 clearance 取两区域公共接触带中 dist 最大的
    cell（局部山口）。门宽 ≈ 2 × clearance（瓶颈半宽经典近似）。

    注意：不用合并树的 direct 标记判定相邻——同一层级先连通的区域会
    把后连通的实际相邻对标记为传递连接（先开的门"吸收"了区域）。
    同一对区域间若有多扇门，只记录山口最高的一扇。
    """
    values = [int(v) for v in np.unique(labels) if v != UNCLASSIFIED]
    dilated: dict[int, np.ndarray] = {
        v: _geodesic_dilate(labels == v, free) for v in values
    }
    peaks = {v: float(dist[labels == v].max()) for v in values}
    doorways: list[Doorway] = []
    for i, a in enumerate(values):
        for b in values[i + 1:]:
            contact = dilated[a] & dilated[b]
            if not contact.any():
                continue
            # 局部山口 = 接触带中 dist 最大的 cell
            center = _contact_saddle_cell(contact, dist)
            clearance = float(dist[center])
            # 墙角对角相邻（十字墙交叉处）的接触带 clearance 极低，滤除
            if clearance < params.min_door_width_m / 2:
                continue
            # 门宽 ≈ 2 × 山口 clearance（瓶颈中点到两侧障碍距离之和的
            # 经典近似；实测方向采样对接触带 1-2 cell 的偏移过于敏感）
            width_m = 2.0 * clearance
            doorways.append(Doorway(
                regions=(a, b),
                center=center,
                width_m=width_m,
                clearance_m=clearance,
                ratio=clearance / max(min(peaks[a], peaks[b]), 1e-6),
                likely_door=params.min_door_width_m <= width_m <= params.max_door_width_m,
            ))
    return doorways


def _free_run(
    free: np.ndarray,
    center: tuple[int, int],
    direction: tuple[float, float],
    max_cells: int,
) -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """从 center 沿 direction 正负两方向走，收集 free cell。

    返回 (line, ends)：ends 为两側行进终止处的第一个非 free cell。"""
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


def _shortest_cut_line(
    free: np.ndarray,
    center: tuple[int, int],
    max_cells: int,
) -> list[tuple[int, int]] | None:
    """过 center 的方向采样最短自由走向（两端都须止于非 free cell）。"""
    directions = [(float(np.sin(t)), float(np.cos(t)))
                  for t in np.linspace(0, np.pi, 16, endpoint=False)]
    best: list[tuple[int, int]] | None = None
    for direction in directions:
        line, ends = _free_run(free, center, direction, max_cells)
        if len(ends) == 2 and (best is None or len(line) < len(best)):
            best = line
    return best


def _clip_doorway_spills(
    labels: np.ndarray,
    dist: np.ndarray,
    free: np.ndarray,
    params: SegmentationParams,
    res: float,
) -> np.ndarray:
    """把门口边界强制对齐到门洞切割线。

    maximin 淹没按"最宽通路"分配 cell：门两侧 dist 低于门鞍的 cell 会被
    分配给对门区域（宽门时形成可见的溢出带）。这里对每个空间相邻的
    区域对：在合并树山口 cell（首次连通 cell，位于门洞通道内）处方向
    采样取两端止于非 free 的最短走向作为切割线，3x3 加厚后暂时阻断，
    把落在对方连通体里的本区域 cell 重归对方；切割带 cell 最后按邻域
    多数表决归边。切割未能把 A∪B 分成两个分含各自峰的连通体时保持原样。
    """
    connections = _connection_values(labels, dist, free)
    values = [int(v) for v in np.unique(labels) if v != UNCLASSIFIED]
    dilated = {v: _geodesic_dilate(labels == v, free) for v in values}
    max_cells = int(params.max_door_measure_m / res)
    out = labels.copy()
    for i, a in enumerate(values):
        for b in values[i + 1:]:
            contact = dilated[a] & dilated[b]
            if not contact.any():
                continue
            # 合并树山口给出可穿过墙体的最窄切割方向；接触带只在
            # _find_doorways 中用于拓扑记录。若没有连接记录则退化到接触带。
            connection = connections.get((a, b))
            cell = (connection[1] if connection is not None
                    else _contact_saddle_cell(contact, dist))
            line = _shortest_cut_line(free, cell, max_cells)
            if line is None:
                continue
            cut = np.zeros(free.shape, dtype=bool)
            for r, c in line:
                r0, r1 = max(0, r - 1), min(free.shape[0], r + 2)
                c0, c1 = max(0, c - 1), min(free.shape[1], c + 2)
                cut[r0:r1, c0:c1] = True
            zone = ((out == a) | (out == b)) & ~cut
            comp, _n = ndimage.label(zone, structure=np.ones((3, 3)))
            peak_a = np.unravel_index(
                np.where(out == a, dist, -np.inf).argmax(), dist.shape)
            peak_b = np.unravel_index(
                np.where(out == b, dist, -np.inf).argmax(), dist.shape)
            comp_a, comp_b = comp[peak_a], comp[peak_b]
            if comp_a == 0 or comp_b == 0 or comp_a == comp_b:
                continue  # 切割未能分离两区域，保持原样
            out[(labels == a) & (comp == comp_b)] = b
            out[(labels == b) & (comp == comp_a)] = a
            # 切割带 cell（被 cut 覆盖、不在任何连通体内）按邻域多数归边
            band = ((out == a) | (out == b)) & cut
            for _ in range(3):
                if not band.any():
                    break
                assigned = False
                for r, c in np.argwhere(band):
                    r0, r1 = max(0, r - 1), min(out.shape[0], r + 2)
                    c0, c1 = max(0, c - 1), min(out.shape[1], c + 2)
                    neighbors = out[r0:r1, c0:c1]
                    votes_a = int((neighbors == a).sum())
                    votes_b = int((neighbors == b).sum())
                    if votes_a != votes_b:
                        out[r, c] = a if votes_a > votes_b else b
                        band[r, c] = False
                        assigned = True
                if not assigned:
                    break
    return out


def _mark_ridges(labels: np.ndarray) -> np.ndarray:
    """与不同 label 相邻（8-连通）的 cell 标为未分类脊线。

    maximin 淹没把每个 cell 严格分给某一侧，不产脊线；这里把接触带
    两侧各一层 cell 标为未分类，作为显式的区域边界。
    """
    ridge = np.zeros(labels.shape, dtype=bool)
    for sa, sb in _CONTACT_SLICES:
        a = labels[sa]
        b = labels[sb]
        touch = (a > UNCLASSIFIED) & (b > UNCLASSIFIED) & (a != b)
        ridge[sa] |= touch
        ridge[sb] |= touch
    out = labels.copy()
    out[ridge] = UNCLASSIFIED
    return out
