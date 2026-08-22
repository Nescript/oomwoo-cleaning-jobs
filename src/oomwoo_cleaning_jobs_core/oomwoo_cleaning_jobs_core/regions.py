"""Region 掩码编辑（draft Region Set）。

对应 docs/DEVELOPMENT.md「第一阶段实施决定 · Region 表示与编辑语义」：

- Region 内部表示为 bitmask（labels 数组），天然支持孔洞与离散组件；
  几何轮廓由 ``cv2.findContours`` 派生，仅用于 GUI 显示与导出。
- **编辑时即时裁剪**：用户画的是意图，系统存的是 ``意图 ∩ Cleanable
  Space``；裁空则该编辑动作无效（返回 False/None，不产生副作用）。
- **后画者抢占**：压到已有 Region 的笔画，重叠 cell 从旧 Region 扣除
  归新 Region（GUI 负责显著提示旧 Region 被改小）。
- 合并为显式操作，不依赖先画出重叠；拆分为画线/画圈切割。
- 只存裁剪后掩码，不存原始笔画。

labels 约定与 segmentation 一致：0 = 未划分（UNASSIGNED）。
Keepout（#6）接入时只需把 cleanable 改为 ``free & ~keepout``。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy import ndimage

from .segmentation import SegmentationResult

#: labels 数组中"未划分"的取值
UNASSIGNED = 0


@dataclass(frozen=True)
class RegionInfo:
    label: int
    name: str
    cell_count: int
    area_m2: float


class RegionSet:
    """一张 Source Map 的 draft Region 集合（掩码权威）。"""

    def __init__(
        self,
        labels: np.ndarray,
        cleanable: np.ndarray,
        resolution: float,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        names: dict[int, str] | None = None,
    ) -> None:
        labels = np.ascontiguousarray(labels, dtype=np.int32)
        cleanable = np.asarray(cleanable, dtype=bool)
        if labels.shape != cleanable.shape:
            raise ValueError('labels 与 cleanable 形状不一致')
        self.labels = labels
        self.cleanable = cleanable
        self.resolution = float(resolution)
        self.origin = origin
        self.names: dict[int, str] = dict(names or {})

    @classmethod
    def from_segmentation(
        cls,
        result: SegmentationResult,
        resolution: float,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        cleanable: np.ndarray | None = None,
    ) -> RegionSet:
        """从自动分割结果初始化 draft（候选区域转为可编辑 Region）。"""
        names = {r.label: f'Region {r.label}' for r in result.regions}
        return cls(
            labels=result.labels.copy(),
            cleanable=result.free_mask if cleanable is None else cleanable,
            resolution=resolution,
            origin=origin,
            names=names,
        )

    # ---- 查询 ----

    def regions(self) -> list[RegionInfo]:
        out = []
        for label in sorted(int(v) for v in np.unique(self.labels) if v != UNASSIGNED):
            cell_count = int((self.labels == label).sum())
            out.append(RegionInfo(
                label=label,
                name=self.names.get(label, f'Region {label}'),
                cell_count=cell_count,
                area_m2=cell_count * self.resolution * self.resolution,
            ))
        return out

    def mask_of(self, label: int) -> np.ndarray:
        return self.labels == label

    @property
    def unassigned_cleanable_mask(self) -> np.ndarray:
        """可清扫但未划分给任何 Region 的 cell（GUI 必须显著呈现）。"""
        return self.cleanable & (self.labels == UNASSIGNED)

    def outline(self, label: int) -> list[np.ndarray]:
        """派生几何轮廓（map frame，米）：[外环, 孔洞1, ...]，每个为 (N, 2) float。

        坐标约定：x = origin.x + (col+0.5)*res，y = origin.y + (row+0.5)*res
        （cells row 0 = 最底行，与 map frame y 轴同向）。
        """
        self._require(label)
        mask = self.mask_of(label).astype(np.uint8)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        # findContours 返回 (x=col, y=row 图像序)；本数组 row 0 = 底行，
        # 与 map frame 同向，无需翻转
        res = self.resolution
        ox, oy = self.origin[0], self.origin[1]
        rings = []
        for contour in contours:
            pts = contour[:, 0, :].astype(np.float64)  # (N, 2): (col, row)
            xs = ox + (pts[:, 0] + 0.5) * res
            ys = oy + (pts[:, 1] + 0.5) * res
            rings.append(np.stack([xs, ys], axis=1))
        return rings

    # ---- 编辑操作 ----

    def paint(self, label: int, stroke: np.ndarray) -> bool:
        """画笔加 cell：裁剪到 Cleanable Space，压到已有 Region 的部分抢占。

        裁空（笔画全在障碍/未知/Keepout 上）则无效，返回 False。
        """
        self._require(label)
        cells = np.asarray(stroke, dtype=bool) & self.cleanable
        if not cells.any():
            return False
        self.labels[cells] = label  # 覆盖即抢占
        return True

    def create(self, stroke: np.ndarray, name: str | None = None) -> int | None:
        """从笔画创建新 Region（裁剪 + 抢占同 paint）。裁空返回 None。"""
        cells = np.asarray(stroke, dtype=bool) & self.cleanable
        if not cells.any():
            return None
        label = self._next_label()
        self.labels[cells] = label
        self.names[label] = name or f'Region {label}'
        return label

    def erase(self, label: int, stroke: np.ndarray) -> bool:
        """画笔减 cell：从 Region 移除笔画覆盖的 cell（变为未划分）。

        减空后 Region 自动删除（不留零 cell Region）。
        """
        self._require(label)
        self.labels[self.mask_of(label) & np.asarray(stroke, dtype=bool)] = UNASSIGNED
        if not self.mask_of(label).any():
            self.names.pop(label, None)
        return True

    def merge(self, target: int, source: int) -> bool:
        """显式合并：source 的全部 cell 并入 target，source 删除。"""
        self._require(target)
        self._require(source)
        if target == source:
            return False
        self.labels[self.labels == source] = target
        self.names.pop(source, None)
        return True

    def split(self, label: int, cut: np.ndarray) -> list[int] | None:
        """画线/画圈拆分：cut 覆盖的 cell 移出 Region（变为未划分），
        剩余部分按 8-连通分成若干片；最大片保留原 label 与名称，
        其余成为新 Region（名称派生）。不足两片返回 None（无效）。
        """
        self._require(label)
        remaining = self.mask_of(label) & ~np.asarray(cut, dtype=bool)
        components, n = ndimage.label(remaining, structure=np.ones((3, 3)))
        if n < 2:
            return None
        counts = np.bincount(components.ravel())
        counts[UNASSIGNED] = 0
        order = np.argsort(-counts)
        name = self.names.get(label, f'Region {label}')
        result_labels = [label]
        # 最大片保留原 label；先清空整个 Region 再逐片重写
        largest = int(order[0])
        self.labels[self.mask_of(label)] = UNASSIGNED
        self.labels[components == largest] = label
        for i, comp_id in enumerate(order[1:], start=2):
            comp_id = int(comp_id)
            if counts[comp_id] == 0:
                continue
            new_label = self._next_label()
            self.labels[components == comp_id] = new_label
            self.names[new_label] = f'{name} ·{i}'
            result_labels.append(new_label)
        return result_labels

    def delete(self, label: int) -> bool:
        """删除 Region：全部 cell 变为未划分。"""
        self._require(label)
        self.labels[self.labels == label] = UNASSIGNED
        self.names.pop(label, None)
        return True

    def rename(self, label: int, name: str) -> bool:
        self._require(label)
        self.names[label] = name
        return True

    # ---- 内部 ----

    def _require(self, label: int) -> None:
        if not self.mask_of(label).any():
            raise ValueError(f'Region {label} 不存在或为空')

    def _next_label(self) -> int:
        current = [int(v) for v in np.unique(self.labels) if v != UNASSIGNED]
        return max(current, default=0) + 1
