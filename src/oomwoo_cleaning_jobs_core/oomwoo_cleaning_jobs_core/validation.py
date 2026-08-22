"""Published 前的校验分级（对应 DEVELOPMENT.md「校验分级」）。

**Error**（阻止发布）：Region 含障碍/未知 cell、Region 掩码经 footprint
半径腐蚀后为空（机器人中心无法停留在 Region 内，永远进不去）、与
Keepout 相交。正常编辑路径下即时裁剪与抢占规则保证这些不会发生——
它们是**系统不变量检查**（防手改文件与 bug）。Region 重叠在内存模型
（单一 labels 数组）下结构性不可能，重叠检查 `check_masks_overlap`
供持久化加载（#5，逐 Region PNG 掩码）时使用。

**Warning**（允许发布，GUI 必须显著呈现）：存在未划分的可清扫自由空间；
Region 的 footprint 可达核心被窄喉断成多片（机器人无法在 Region 内
通行）。注意不用「不可达 cell 比例」指标：任何房间的周界一圈都是
机器人中心不可达的（约 30%），该指标对正常房间必然误报。
Region 中间有去不了的家具（被裁剪或不可达）是**正常行为**，不是错误。

阶段一 footprint 来自参数 `robot_inscribed_radius`（默认 0.17 m）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage

from .regions import RegionSet

LEVEL_ERROR = 'error'
LEVEL_WARNING = 'warning'

#: 默认 footprint 内切圆半径（米），阶段二改从 Nav2 解析
DEFAULT_ROBOT_RADIUS_M = 0.17
#: 可达核心的最小连通片尺寸（cells），小于此的碎片不算「断开的一片」
MIN_CORE_COMPONENT_CELLS = 4


@dataclass(frozen=True)
class ValidationIssue:
    level: str  # LEVEL_ERROR / LEVEL_WARNING
    code: str
    message: str
    region: int | None = None  # 相关 Region label；与具体 Region 无关为 None


@dataclass
class ValidationReport:
    issues: list[ValidationIssue] = field(default_factory=list)

    @property
    def errors(self) -> list[ValidationIssue]:
        return [i for i in self.issues if i.level == LEVEL_ERROR]

    @property
    def warnings(self) -> list[ValidationIssue]:
        return [i for i in self.issues if i.level == LEVEL_WARNING]

    @property
    def ok(self) -> bool:
        """无 error 即可发布（warning 不阻止）。"""
        return not self.errors


def validate_region_set(
    region_set: RegionSet,
    robot_inscribed_radius: float = DEFAULT_ROBOT_RADIUS_M,
    keepout_mask: np.ndarray | None = None,
) -> ValidationReport:
    """发布前校验。keepout_mask 为 None 表示尚无 Keepout（#6 接入）。"""
    report = ValidationReport()
    res = region_set.resolution
    cleanable = region_set.cleanable
    if keepout_mask is not None:
        keepout_mask = np.asarray(keepout_mask, dtype=bool)
        if keepout_mask.shape != region_set.labels.shape:
            raise ValueError('keepout_mask 与 RegionSet 形状不一致')

    regions = region_set.regions()
    if not regions:
        report.issues.append(ValidationIssue(
            level=LEVEL_ERROR, code='empty_region_set',
            message='Region Set 为空，无可发布内容'))

    for info in regions:
        mask = region_set.mask_of(info.label)
        # Error: 含障碍/未知 cell（不变量）
        dirty = int((mask & ~cleanable).sum())
        if dirty:
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='region_outside_cleanable', region=info.label,
                message=f'Region "{info.name}" 含 {dirty} 个障碍/未知 cell'))
        # footprint 可达核心 = Region 掩码经半径腐蚀（机器人中心须能
        # 停留在 Region 内；腐蚀的是 Region 自身，不是整个可清扫空间）
        core = (cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5) * res
                ) >= robot_inscribed_radius
        # Error: 腐蚀后为空（永远进不去）
        if not core.any():
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='region_unreachable', region=info.label,
                message=f'Region "{info.name}" 经 footprint 半径 '
                        f'{robot_inscribed_radius} m 腐蚀后为空，机器人无法进入'))
        else:
            # Warning: 可达核心被窄喉断成多片（机器人无法在 Region 内通行）
            components, n = ndimage.label(core, structure=np.ones((3, 3)))
            counts = np.bincount(components.ravel())
            pieces = int((counts[1:] >= MIN_CORE_COMPONENT_CELLS).sum())
            if pieces > 1:
                report.issues.append(ValidationIssue(
                    level=LEVEL_WARNING, code='region_disconnected_core',
                    region=info.label,
                    message=f'Region "{info.name}" 的 footprint 可达核心断成 '
                            f'{pieces} 片（存在窄于机器人的内部通道）'))
        # Error: 与 Keepout 相交（不变量；#6 接入）
        if keepout_mask is not None:
            overlap = int((mask & keepout_mask).sum())
            if overlap:
                report.issues.append(ValidationIssue(
                    level=LEVEL_ERROR, code='region_in_keepout', region=info.label,
                    message=f'Region "{info.name}" 与 Keepout 相交 {overlap} cell'))

    # Warning: 未划分的可清扫自由空间
    unassigned = region_set.unassigned_cleanable_mask
    if unassigned.any():
        count = int(unassigned.sum())
        report.issues.append(ValidationIssue(
            level=LEVEL_WARNING, code='unassigned_cleanable',
            message=f'存在 {count} 个未划分的可清扫 cell'
                    f'（{count * res * res:.2f} m²）'))

    return report


def check_masks_overlap(masks: dict[int, np.ndarray]) -> list[ValidationIssue]:
    """逐 Region 掩码的重叠检查（不变量）。供 #5 从 PNG 掩码加载后调用。"""
    issues = []
    items = sorted(masks.items())
    for i, (label_a, mask_a) in enumerate(items):
        for label_b, mask_b in items[i + 1:]:
            overlap = int((mask_a & mask_b).sum())
            if overlap:
                issues.append(ValidationIssue(
                    level=LEVEL_ERROR, code='region_overlap',
                    message=f'Region {label_a} 与 Region {label_b} 重叠 {overlap} cell'))
    return issues
