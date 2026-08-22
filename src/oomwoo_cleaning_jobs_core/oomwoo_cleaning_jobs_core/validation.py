"""Pre-publish validation severity grading (see DEVELOPMENT.md "Validation
grading").

**Error** (blocks publishing): a Region contains occupied/unknown cells;
a Region mask eroded by the footprint radius becomes empty (the robot
center can never stay inside the Region, so it can never enter); a Region
intersects a Keepout. In normal editing flows, immediate clipping and
preemption guarantee these never happen — they are **system invariant
checks** (against hand-edited files and bugs). Region overlap is
structurally impossible in the in-memory model (single labels array);
the overlap check `check_masks_overlap` is used at persistence load time
(#5, per-Region PNG masks).

**Warning** (publishing allowed, GUI must display prominently): unassigned
cleanable free space exists; a Region's footprint-reachable core is split
into multiple pieces by a narrow throat (the robot cannot traverse the
Region). Note the deliberate absence of an "unreachable cell ratio"
metric: the perimeter ring of any room is unreachable to the robot center
(~30%), so that metric would always be a false positive for normal rooms.
Unreachable furniture inside a Region (clipped away or unreachable) is
**normal behavior**, not an error.

In phase 1 the footprint comes from the `robot_inscribed_radius` parameter
(default 0.17 m).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage

from .regions import RegionSet

LEVEL_ERROR = 'error'
LEVEL_WARNING = 'warning'

#: Default footprint inscribed-circle radius (m); phase 2 parses it from Nav2
DEFAULT_ROBOT_RADIUS_M = 0.17
#: Minimum size (cells) of a reachable-core connected piece; smaller
#: fragments do not count as a "disconnected piece"
MIN_CORE_COMPONENT_CELLS = 4


@dataclass(frozen=True)
class ValidationIssue:
    level: str  # LEVEL_ERROR / LEVEL_WARNING
    code: str
    message: str
    region: int | None = None  # related Region label; None if not Region-specific


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
        """No errors means publishable (warnings do not block)."""
        return not self.errors


def validate_region_set(
    region_set: RegionSet,
    robot_inscribed_radius: float = DEFAULT_ROBOT_RADIUS_M,
    keepout_mask: np.ndarray | None = None,
) -> ValidationReport:
    """Pre-publish validation. keepout_mask=None means no Keepout yet
    (wired in #6)."""
    report = ValidationReport()
    res = region_set.resolution
    cleanable = region_set.cleanable
    if keepout_mask is not None:
        keepout_mask = np.asarray(keepout_mask, dtype=bool)
        if keepout_mask.shape != region_set.labels.shape:
            raise ValueError('keepout_mask shape must match the RegionSet grid')

    regions = region_set.regions()
    if not regions:
        report.issues.append(ValidationIssue(
            level=LEVEL_ERROR, code='empty_region_set',
            message='Region Set is empty; nothing to publish'))

    for info in regions:
        mask = region_set.mask_of(info.label)
        # Error: contains occupied/unknown cells (invariant)
        dirty = int((mask & ~cleanable).sum())
        if dirty:
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='region_outside_cleanable', region=info.label,
                message=f'Region "{info.name}" contains {dirty} occupied/unknown cells'))
        # Footprint-reachable core = Region mask eroded by the radius (the
        # robot center must be able to stay inside the Region; the Region
        # itself is eroded, not the whole cleanable space)
        core = (cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5) * res
                ) >= robot_inscribed_radius
        # Error: empty after erosion (can never be entered)
        if not core.any():
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='region_unreachable', region=info.label,
                message=f'Region "{info.name}" is empty after erosion by footprint '
                        f'radius {robot_inscribed_radius} m; robot cannot enter'))
        else:
            # Warning: reachable core split into multiple pieces by narrow
            # throats (robot cannot traverse the Region)
            components, n = ndimage.label(core, structure=np.ones((3, 3)))
            counts = np.bincount(components.ravel())
            pieces = int((counts[1:] >= MIN_CORE_COMPONENT_CELLS).sum())
            if pieces > 1:
                report.issues.append(ValidationIssue(
                    level=LEVEL_WARNING, code='region_disconnected_core',
                    region=info.label,
                    message=f'Region "{info.name}" footprint-reachable core is split '
                            f'into {pieces} pieces (internal passage narrower than the robot)'))
        # Error: intersects a Keepout (invariant; wired in #6)
        if keepout_mask is not None:
            overlap = int((mask & keepout_mask).sum())
            if overlap:
                report.issues.append(ValidationIssue(
                    level=LEVEL_ERROR, code='region_in_keepout', region=info.label,
                    message=f'Region "{info.name}" intersects Keepout in {overlap} cells'))

    # Warning: unassigned cleanable free space
    unassigned = region_set.unassigned_cleanable_mask
    if unassigned.any():
        count = int(unassigned.sum())
        report.issues.append(ValidationIssue(
            level=LEVEL_WARNING, code='unassigned_cleanable',
            message=f'{count} unassigned cleanable cells'
                    f' ({count * res * res:.2f} m²)'))

    return report


def check_masks_overlap(masks: dict[int, np.ndarray]) -> list[ValidationIssue]:
    """Overlap check for per-Region masks (invariant). Called after loading
    from PNG masks in #5."""
    issues = []
    items = sorted(masks.items())
    for i, (label_a, mask_a) in enumerate(items):
        for label_b, mask_b in items[i + 1:]:
            overlap = int((mask_a & mask_b).sum())
            if overlap:
                issues.append(ValidationIssue(
                    level=LEVEL_ERROR, code='region_overlap',
                    message=f'Region {label_a} and Region {label_b} overlap in {overlap} cells'))
    return issues
