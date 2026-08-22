"""Region mask editing (draft Region Set).

See docs/DEVELOPMENT.md "Phase 1 implementation decisions - Region
representation and editing semantics":

- A Region is represented internally as a bitmask (labels array), naturally
  supporting holes and disjoint components; geometric outlines are derived
  via ``cv2.findContours`` and used only for GUI display and export.
- **Immediate clipping at edit time**: the user paints intent; the system
  stores ``intent & Cleanable Space``; a stroke clipped to empty makes the
  edit a no-op (returns False/None with no side effects).
- **Later-painter preemption**: a stroke overlapping an existing Region
  transfers the overlapping cells from the old Region to the new one (the
  GUI must prominently indicate that the old Region shrank).
- Merge is an explicit operation, not triggered by painting overlap; split
  works by drawing a cut line/circle.
- Only the clipped mask is stored, never the raw stroke.

labels convention matches segmentation: 0 = unassigned (UNASSIGNED).
When Keepout (#6) is wired in, cleanable simply becomes ``free & ~keepout``.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy import ndimage

from .segmentation import SegmentationResult

#: labels value for "unassigned"
UNASSIGNED = 0


@dataclass(frozen=True)
class RegionInfo:
    label: int
    name: str
    cell_count: int
    area_m2: float


class RegionSet:
    """Draft Region set of one Source Map (mask is authoritative)."""

    def __init__(
        self,
        labels: np.ndarray,
        cleanable: np.ndarray,
        resolution: float,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        names: dict[int, str] | None = None,
        base_cleanable: np.ndarray | None = None,
        keepout_mask: np.ndarray | None = None,
    ) -> None:
        labels = np.ascontiguousarray(labels, dtype=np.int32)
        cleanable = np.asarray(cleanable, dtype=bool)
        base_cleanable = (cleanable if base_cleanable is None
                          else np.asarray(base_cleanable, dtype=bool))
        if labels.shape != cleanable.shape or labels.shape != base_cleanable.shape:
            raise ValueError('labels, cleanable and base_cleanable shapes must match')
        self.labels = labels
        self.base_cleanable = base_cleanable.copy()
        self.keepout_mask = np.zeros(labels.shape, dtype=bool)
        self.cleanable = cleanable.copy()
        self.names: dict[int, str] = dict(names or {})
        if keepout_mask is not None:
            self.apply_keepout_mask(keepout_mask)
        self.resolution = float(resolution)
        self.origin = origin

    @classmethod
    def from_segmentation(
        cls,
        result: SegmentationResult,
        resolution: float,
        origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
        cleanable: np.ndarray | None = None,
        base_cleanable: np.ndarray | None = None,
        keepout_mask: np.ndarray | None = None,
    ) -> RegionSet:
        """Initialize a draft from an automatic segmentation result (candidate
        regions become editable Regions).

        If segmentation already excluded Keepout, the caller must also pass
        the original free mask as ``base_cleanable`` plus the current
        constraint mask, so cleanable space can be restored correctly when
        constraints are later removed; clipped-away Region cells are not
        resurrected automatically.
        """
        cleanable = result.free_mask if cleanable is None else cleanable
        names = {r.label: f'Region {r.label}' for r in result.regions}
        return cls(
            labels=result.labels.copy(),
            cleanable=cleanable,
            resolution=resolution,
            origin=origin,
            names=names,
            base_cleanable=base_cleanable,
            keepout_mask=keepout_mask,
        )

    # ---- Queries ----

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
        """Cells that are cleanable but assigned to no Region (the GUI must
        display them prominently)."""
        return self.cleanable & (self.labels == UNASSIGNED)

    def outline(self, label: int) -> list[np.ndarray]:
        """Derived geometric outline (map frame, meters):
        [outer ring, hole 1, ...], each an (N, 2) float array.

        Coordinate convention: ``(x, y) = origin.xy + R(origin.yaw) *
        ((col+0.5)*res, (row+0.5)*res)``; cells row 0 is the bottom row of
        the local map.
        """
        self._require(label)
        mask = self.mask_of(label).astype(np.uint8)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        # findContours returns (x=col, y=row in image order); in this array
        # row 0 = bottom row, same orientation as the map frame, no flip
        # needed
        res = self.resolution
        ox, oy, yaw = self.origin
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rings = []
        for contour in contours:
            pts = contour[:, 0, :].astype(np.float64)  # (N, 2): (col, row)
            local_x = (pts[:, 0] + 0.5) * res
            local_y = (pts[:, 1] + 0.5) * res
            xs = ox + cos_yaw * local_x - sin_yaw * local_y
            ys = oy + sin_yaw * local_x + cos_yaw * local_y
            rings.append(np.stack([xs, ys], axis=1))
        return rings

    # ---- Spatial constraints ----

    def apply_keepout_mask(self, keepout_mask: np.ndarray) -> None:
        """Apply the current Keepout and immediately clip restricted Region
        cells.

        Removing a constraint only restores cleanable space; it does not
        restore previously clipped Region cells, so content the user
        explicitly removed or that another operation took over is never
        written back out of thin air.
        """
        keepout_mask = np.asarray(keepout_mask, dtype=bool)
        if keepout_mask.shape != self.labels.shape:
            raise ValueError('keepout_mask shape must match the RegionSet grid')
        self.keepout_mask = keepout_mask.copy()
        self.cleanable = self.base_cleanable & ~self.keepout_mask
        self.labels[~self.cleanable] = UNASSIGNED
        self.names = {
            label: name for label, name in self.names.items()
            if (self.labels == label).any()
        }

    # ---- Editing operations ----

    def paint(self, label: int, stroke: np.ndarray) -> bool:
        """Brush-add cells: clip to Cleanable Space; overlap with an existing
        Region is preempted.

        A stroke clipped to empty (entirely on occupied/unknown/Keepout) is
        a no-op and returns False.
        """
        self._require(label)
        cells = np.asarray(stroke, dtype=bool) & self.cleanable
        if not cells.any():
            return False
        self.labels[cells] = label  # overwrite = preemption
        self._prune_empty_names()
        return True

    def create(self, stroke: np.ndarray, name: str | None = None) -> int | None:
        """Create a new Region from a stroke (same clipping + preemption as
        paint). Returns None when clipped to empty."""
        cells = np.asarray(stroke, dtype=bool) & self.cleanable
        if not cells.any():
            return None
        label = self._next_label()
        self.labels[cells] = label
        self._prune_empty_names()
        self.names[label] = name or f'Region {label}'
        return label

    def erase(self, label: int, stroke: np.ndarray) -> bool:
        """Brush-remove cells: remove stroke-covered cells from the Region
        (they become unassigned).

        A Region emptied this way is deleted automatically (no zero-cell
        Regions are kept).
        """
        self._require(label)
        self.labels[self.mask_of(label) & np.asarray(stroke, dtype=bool)] = UNASSIGNED
        if not self.mask_of(label).any():
            self.names.pop(label, None)
        return True

    def merge(self, target: int, source: int) -> bool:
        """Explicit merge: all cells of source move into target; source is
        deleted."""
        self._require(target)
        self._require(source)
        if target == source:
            return False
        self.labels[self.labels == source] = target
        self.names.pop(source, None)
        return True

    def split(self, label: int, cut: np.ndarray) -> list[int] | None:
        """Split by a drawn line/circle: cells covered by cut leave the
        Region (become unassigned); the remainder is split into 8-connected
        pieces. The largest piece keeps the original label and name; the
        rest become new Regions (derived names). Returns None when fewer
        than two pieces result (invalid).
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
        # The largest piece keeps the original label; clear the whole
        # Region first, then rewrite piece by piece
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
        """Delete a Region: all its cells become unassigned."""
        self._require(label)
        self.labels[self.labels == label] = UNASSIGNED
        self.names.pop(label, None)
        return True

    def rename(self, label: int, name: str) -> bool:
        self._require(label)
        self.names[label] = name
        return True

    # ---- Internals ----

    def _prune_empty_names(self) -> None:
        self.names = {
            label: name for label, name in self.names.items()
            if (self.labels == label).any()
        }

    def _require(self, label: int) -> None:
        if not self.mask_of(label).any():
            raise ValueError(f'Region {label} does not exist or is empty')

    def _next_label(self) -> int:
        current = [int(v) for v in np.unique(self.labels) if v != UNASSIGNED]
        return max(current, default=0) + 1
