"""Pre-publish validation severity grading (see DEVELOPMENT.md "Validation
grading").

**Error** (blocks publishing): a Region contains occupied/unknown cells;
a Region cannot be reached by the robot footprint from any navigable position
in cleanable space (e.g. trapped in narrow unreachable cavities); a Region
intersects a Keepout. In normal editing flows, immediate clipping and
preemption guarantee these never happen — they are **system invariant
checks** (against hand-edited files and bugs). Regions smaller than the robot
footprint (such as spot areas) are allowed as long as they can be reached and
covered from adjacent navigable space. Region overlap is structurally impossible
in the in-memory model (single labels array); the overlap check
`check_masks_overlap` is used at persistence load time (#5, per-Region PNG masks).

**Dock-seeded reachability** (optional): when `seed_pose` (the dock staging
pose, map frame) is provided, reachability becomes dock-relative instead of
global. A 4-connected flood fill from the seed cell over cleanable space
yields the main reachable component; cleanable cells outside it form the
*enclosed area* created by Virtual Wall chains (8-connected bands) together
with physical walls. Three additional checks apply:

- **Error `dock_unreachable`**: the seed cell itself is off-grid, occupied,
  unknown, or constrained — the robot could not leave the dock.
- **Error `region_enclosed`**: a Region has cells inside the enclosed area
  (they are preserved, never silently clipped; the user must remove the
  offending wall or redraw the Region).
- **Warning `virtual_wall_seals_nothing`**: removing the wall bands would
  not change the dock-reachable component, i.e. the walls seal nothing
  (possible gaps or unclosed wall chains).

Without a seed pose the global semantics below apply unchanged.

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
import math

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


def _seed_cell(region_set: RegionSet, seed_pose: tuple[float, float]) -> tuple[int, int]:
    """Map-frame seed pose -> (row, col) grid cell, honoring the origin yaw."""
    ox, oy, yaw = region_set.origin
    dx, dy = seed_pose[0] - ox, seed_pose[1] - oy
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return (int(math.floor(local_y / region_set.resolution)),
            int(math.floor(local_x / region_set.resolution)))


def validate_region_set(
    region_set: RegionSet,
    robot_inscribed_radius: float = DEFAULT_ROBOT_RADIUS_M,
    keepout_mask: np.ndarray | None = None,
    seed_pose: tuple[float, float] | None = None,
    wall_band_mask: np.ndarray | None = None,
) -> ValidationReport:
    """Pre-publish validation. keepout_mask=None means no Keepout yet
    (wired in #6). seed_pose=None keeps the global reachability semantics;
    pass the dock staging pose (map frame) for dock-relative checks.
    wall_band_mask marks the keepout cells contributed by Virtual Walls and
    is only used for the virtual_wall_seals_nothing warning."""
    report = ValidationReport()
    res = region_set.resolution
    cleanable = region_set.cleanable
    if keepout_mask is not None:
        keepout_mask = np.asarray(keepout_mask, dtype=bool)
        if keepout_mask.shape != region_set.labels.shape:
            raise ValueError('keepout_mask shape must match the RegionSet grid')
    if wall_band_mask is not None:
        wall_band_mask = np.asarray(wall_band_mask, dtype=bool)
        if wall_band_mask.shape != region_set.labels.shape:
            raise ValueError('wall_band_mask shape must match the RegionSet grid')

    regions = region_set.regions()
    if not regions:
        report.issues.append(ValidationIssue(
            level=LEVEL_ERROR, code='empty_region_set',
            message='Region Set is empty; nothing to publish'))

    # Pre-compute global navigable centers (where the robot center can physically be positioned)
    # and global sweep reachable space (cells within robot_inscribed_radius of any navigable center).
    navigable_centers = (
        cv2.distanceTransform(cleanable.astype(np.uint8), cv2.DIST_L2, 5) * res
    ) >= robot_inscribed_radius

    # Dock-seeded reachability: restrict navigable centers to the 4-connected
    # component reachable from the seed and derive the enclosed area.
    enclosed: np.ndarray | None = None
    if seed_pose is not None:
        row, col = _seed_cell(region_set, seed_pose)
        rows, cols = region_set.labels.shape
        if not (0 <= row < rows and 0 <= col < cols):
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='dock_unreachable',
                message='Dock staging pose lies outside the map grid'))
        elif not cleanable[row, col]:
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='dock_unreachable',
                message='Dock staging cell is occupied, unknown, or constrained; '
                        'the robot could not leave the dock with these constraints'))
        else:
            _, components = cv2.connectedComponents(
                cleanable.astype(np.uint8), connectivity=4)
            main_component = components == components[row, col]
            enclosed = cleanable & ~main_component
            navigable_centers &= main_component
            if wall_band_mask is not None and wall_band_mask.any():
                _, without_walls = cv2.connectedComponents(
                    (cleanable | wall_band_mask).astype(np.uint8), connectivity=4)
                no_wall_component = (without_walls == without_walls[row, col]) & cleanable
                if np.array_equal(no_wall_component, main_component):
                    report.issues.append(ValidationIssue(
                        level=LEVEL_WARNING, code='virtual_wall_seals_nothing',
                        message='Virtual Walls do not change the dock-reachable area '
                                '(possible gaps or unclosed wall chains)'))

    if not navigable_centers.any():
        sweep_reachable = np.zeros_like(cleanable, dtype=bool)
    else:
        dist_to_navigable = (
            cv2.distanceTransform((~navigable_centers).astype(np.uint8), cv2.DIST_L2, 5) * res
        )
        sweep_reachable = (dist_to_navigable <= robot_inscribed_radius) & cleanable

    for info in regions:
        mask = region_set.mask_of(info.label)
        # Error: contains occupied/unknown cells (invariant)
        dirty = int((mask & ~cleanable).sum())
        if dirty:
            report.issues.append(ValidationIssue(
                level=LEVEL_ERROR, code='region_outside_cleanable', region=info.label,
                message=f'Region "{info.name}" contains {dirty} occupied/unknown cells'))

        # Error: Region cells sealed inside a virtual-wall enclosure
        # (dock-seeded mode only; cells are preserved, never clipped).
        sealed = 0
        if enclosed is not None:
            sealed = int((mask & enclosed).sum())
            if sealed:
                report.issues.append(ValidationIssue(
                    level=LEVEL_ERROR, code='region_enclosed', region=info.label,
                    message=f'Region "{info.name}" has {sealed} cells inside virtual-wall '
                            f'enclosures unreachable from the dock'))

        # Footprint-reachability checks (skipped when the enclosure error
        # above already identified the root cause)
        if not sealed:
            # Footprint-reachable core within the Region itself
            self_core = (cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5) * res
                         ) >= robot_inscribed_radius

            if self_core.any():
                # Standard region: check if reachable core is split into multiple pieces by narrow throats
                components, n = ndimage.label(self_core, structure=np.ones((3, 3)))
                counts = np.bincount(components.ravel())
                pieces = int((counts[1:] >= MIN_CORE_COMPONENT_CELLS).sum())
                if pieces > 1:
                    report.issues.append(ValidationIssue(
                        level=LEVEL_WARNING, code='region_disconnected_core',
                        region=info.label,
                        message=f'Region "{info.name}" footprint-reachable core is split '
                                f'into {pieces} pieces (internal passage narrower than the robot)'))
            else:
                # Region is smaller or narrower than the robot footprint (e.g. a small spot area).
                # It is allowed if the robot can reach/cover it from a navigable position in cleanable space.
                # It is an Error only if no navigable position can sweep any part of this region.
                if not (mask & sweep_reachable).any():
                    report.issues.append(ValidationIssue(
                        level=LEVEL_ERROR, code='region_unreachable', region=info.label,
                        message=f'Region "{info.name}" cannot be reached by robot footprint '
                                f'(radius {robot_inscribed_radius} m) from any navigable position'))

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
