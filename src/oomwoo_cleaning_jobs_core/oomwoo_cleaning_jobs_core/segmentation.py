"""Automatic candidate-region segmentation: distance transform + maximin
flooding watershed confined to free space + saddle merge.

See docs/DEVELOPMENT.md "Phase 1 implementation decisions - Automatic
segmentation". Pipeline:

1. free mask (``0 <= v < free_thresh*100``; unknown counts as not cleanable),
   processed per connected component.
2. Distance transform + local-maxima markers; a component with a single peak
   (open-plan room) becomes one low-confidence candidate as a whole.
3. **Maximin flooding** (`_maximin_watershed`): each free cell is assigned to
   the seed reachable via the widest path to that cell. Flooding propagates
   only within free space — walls are not crossable, so regions never spill
   to the other side of a wall (`cv2.watershed` on the full image crossed
   walls; abandoned).
4. Small-region merge (`_merge_small_regions`): regions smaller than
   ``min_region_area`` merge into the neighbor with the **widest connection**
   (highest merge-tree saddle); ties fall back to the longest shared border;
   a region without neighbors becomes unclassified.
5. **Saddle merge** (`_merge_by_saddle`): two adjacent regions merge when
   their saddle (the true saddle height from the `_connection_values`
   superlevel-set merge tree = half-width of the widest passage between the
   regions, independent of where the boundary lands) is at least
   ``saddle_merge_ratio`` x the lower peak height — a wide enough passage
   means one open space; a real doorway's saddle is well below both peak
   heights and is never merged. Transitive closure via union-find.
6. **Doorway spill clipping** (`_clip_doorway_spills`): maximin assigns cells
   near a doorway whose dist is below the door saddle to the opposite region
   (a spill band). For each adjacent pair a cut line is generated at the
   merge-tree saddle cell, temporarily blocked, and cells landing in the
   other component are reassigned; region boundaries are forced onto the
   doorway cut line.
7. Ridge marking (`_mark_ridges`): cells adjacent to a different label are
   marked unclassified (flooding produces no ridges; one cell layer on each
   side of the contact band forms the unclassified boundary).
8. **Doorway records** (`_find_doorways`): for each adjacent region pair
   (decided by geodesic-dilation contact bands) record a `Doorway`
   (saddle center, clearance, width ~= 2x clearance, ratio, likely_door),
   forming the region topology adjacency.

Unclassified cells (label 0) are exposed via ``unclassified_free_mask``;
local high ground below ``min_peak_height_m`` (too-narrow strips) produces
no seed and stays unclassified.

This algorithm is the initial "use it and see" strategy and is replaceable.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage

from .source_map import DEFAULT_FREE_THRESH, SourceMap

#: labels value for "not a candidate / unclassified"
UNCLASSIFIED = 0

_NEIGHBORS_8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))

#: Wrap-free adjacent slice pairs: (slice of a, slice of b) where b is a
#: shifted one cell right/down/down-right/down-left
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
    #: Diameter of the local-maximum window (m); distance ripples smaller
    #: than this produce no separate room. Default 0.7 m: a larger window
    #: suppresses small-room peaks through doorways (the whole component
    #: then degenerates to low confidence).
    marker_neighborhood_m: float = 0.7
    #: Minimum distance value of a peak (m), ~= robot_inscribed_radius;
    #: narrower passages form no peak
    min_peak_height_m: float = 0.17
    #: Saddle-merge threshold: merge adjacent regions when
    #: saddle >= ratio x lower peak height. Values above 1.0 disable merging;
    #: real doorways usually score < 0.5, false splits of one open space >= 1.0.
    saddle_merge_ratio: float = 0.8
    #: Door-width range (m) for the likely_door flag of doorway records
    min_door_width_m: float = 0.30
    max_door_width_m: float = 1.30
    #: Maximum free-run extension (m) when sampling a doorway cut line
    max_door_measure_m: float = 1.50


@dataclass(frozen=True)
class CandidateRegion:
    """Automatically derived Candidate Region."""

    label: int
    cell_count: int
    area_m2: float
    low_confidence: bool


@dataclass(frozen=True)
class Doorway:
    """Doorway record between two Regions (topology edge).

    center is the max-dist cell inside the contact band of the two regions
    (local saddle); clearance_m is the saddle height (bottleneck half-width);
    width_m ~= 2 x clearance (classic approximation);
    ratio = clearance / min(peak heights of both sides) (larger means more
    likely one open space); likely_door = width_m within the door-width range.
    """

    regions: tuple[int, int]
    center: tuple[int, int]  # (row, col), cell row order
    width_m: float
    clearance_m: float
    ratio: float
    likely_door: bool


@dataclass
class SegmentationResult:
    """Segmentation result. labels shares the row order of SourceMap.cells
    (row 0 = bottom row)."""

    labels: np.ndarray  # int32, UNCLASSIFIED(0) = not a candidate / unclassified
    regions: list[CandidateRegion]
    free_mask: np.ndarray
    params: SegmentationParams
    doorways: list[Doorway] = field(default_factory=list)

    @property
    def unclassified_free_mask(self) -> np.ndarray:
        """Cells that are cleanable but not assigned to any candidate region."""
        return self.free_mask & (self.labels == UNCLASSIFIED)

    def mask_of(self, label: int) -> np.ndarray:
        return self.labels == label

    def adjacent_labels(self, label: int) -> set[int]:
        """Topology adjacency: other Regions connected to this one via a doorway."""
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
    """Automatically segment the cleanable space of a Source Map.

    ``cleanable_mask`` lets external constraints such as Keepout narrow the
    candidate space; it cannot turn occupied/unknown cells cleanable and
    must match the Source Map grid shape.
    """
    params = params or SegmentationParams()
    res = source_map.resolution
    source_free = source_map.free_mask(params.free_thresh)
    if cleanable_mask is None:
        free = source_free
    else:
        cleanable_mask = np.asarray(cleanable_mask, dtype=bool)
        if cleanable_mask.shape != source_free.shape:
            raise ValueError('cleanable_mask shape must match the SourceMap grid')
        free = source_free & cleanable_mask
    structure = np.ones((3, 3), dtype=np.int32)  # 8-connectivity

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
            # Degenerate: the whole component is one low-confidence candidate
            labels[comp] = next_label
            low_conf.add(next_label)
            next_label += 1
            continue

        for k in range(1, n_markers + 1):
            markers[comp_markers == k] = next_label
            next_label += 1

    # Maximin flooding only fills free cells not yet assigned (degenerate
    # components are already labeled as a whole)
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
    """Maximin (widest-path) flooding watershed within free space.

    Each free cell is assigned to the seed whose path to that cell has the
    largest minimum distance value, i.e. a catchment basin on the distance-
    transform surface. Flooding propagates only between free cells; walls
    are not crossable. Ties in bottleneck value (two basins meeting / a
    plateau) are broken by geodesic distance to the seed, so boundaries land
    on saddles/doorways instead of spilling into a neighboring region; any
    remaining arbitrary boundary is marked unclassified by `_mark_ridges`.
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
            continue  # already updated by a better (wider, or equal but shorter) path
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
    """Regions smaller than min_cells merge into a neighbor; a region without
    neighbors becomes unclassified.

    The target neighbor is chosen by **widest connection** (saddle height
    from `_connection_values`): a small region should merge into the region
    it connects to most widely. Choosing by shared-border length would leak
    small basins next to a doorway through the door into the adjacent room;
    choosing by saddle keeps them in this room's main basin (wider internal
    connection). Ties fall back to the longest shared border.
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
    """region pair -> (saddle height W, saddle cell, direct): the **maximum**
    W for which the two regions are connected within the ``dist >= W``
    superlevel set, plus the cell where they first connect (the bottleneck).

    Equivalent to the watershed merge tree (persistence): scanning free
    cells in descending dist order with union-find, the dist at which two
    different labels first become connected is the bottleneck of the widest
    path between the regions, independent of where the final boundary lands.

    direct=False means the connection passes through a third region (the
    merged set already contains other labels) — a transitive connection, not
    direct adjacency; merging and doorway records must only use direct=True.
    """
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
                continue  # not processed yet (lower dist)
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
    """Merge region pairs whose saddle height is at least ratio x the lower
    peak height (union-find transitive closure).

    Intuition: if a wide enough passage (relative to both sizes) exists
    between two regions, they are one open space falsely split by multiple
    distance peaks; a real doorway's saddle is well below both peak heights
    and is never merged.
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
    """Dilate within free space so dilation cannot cross walls."""
    kernel3 = np.ones((3, 3), dtype=np.uint8)
    out = mask & free
    for _ in range(steps):
        out = (cv2.dilate(out.astype(np.uint8), kernel3) > 0) & free
    return out


def _contact_saddle_cell(
    contact: np.ndarray,
    dist: np.ndarray,
) -> tuple[int, int]:
    """Return the cell with maximum clearance in the geodesic contact band
    of two Regions."""
    if contact.shape != dist.shape:
        raise ValueError('contact and dist shapes differ')
    if not contact.any():
        raise ValueError('contact band must not be empty')
    contact_dist = np.where(contact, dist, -np.inf)
    return tuple(int(v) for v in np.unravel_index(contact_dist.argmax(), dist.shape))


def _find_doorways(
    labels: np.ndarray,
    dist: np.ndarray,
    free: np.ndarray,
    params: SegmentationParams,
    res: float,
) -> list[Doorway]:
    """Generate doorway records for the final Region topology.

    A doorway region pair is a pair that is spatially adjacent in the final
    labels (contact band with <=2 cells of ridge/unclassified between them).
    Saddle center and clearance are taken from the max-dist cell of the
    shared contact band (local saddle). Door width ~= 2 x clearance (classic
    bottleneck half-width approximation).

    Note: adjacency is NOT decided by the merge tree's direct flag — regions
    connected earlier at the same level mark later actually-adjacent pairs
    as transitive (the first-opened door "absorbs" a region). When two
    regions share multiple doors, only the one with the highest saddle is
    recorded.
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
            # Local saddle = max-dist cell of the contact band
            center = _contact_saddle_cell(contact, dist)
            clearance = float(dist[center])
            # Contact bands of corner-diagonal adjacency (cross-shaped wall
            # junctions) have extremely low clearance; filter them out
            if clearance < params.min_door_width_m / 2:
                continue
            # Door width ~= 2 x saddle clearance (classic approximation of
            # bottleneck midpoint to both obstacle sides; measured directional
            # sampling proved too sensitive to 1-2 cell contact-band offsets)
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
    """Walk from center along direction in both signs, collecting free cells.

    Returns (line, ends): ends are the first non-free cell at each side."""
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
    """Direction-sampled shortest free run through center (both ends must
    terminate on non-free cells)."""
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
    """Force region boundaries onto doorway cut lines.

    Maximin flooding assigns cells by "widest path": cells near a doorway
    whose dist is below the door saddle are assigned to the opposite region
    (a visible spill band on wide doors). Here, for each spatially adjacent
    region pair: direction-sample at the merge-tree saddle cell (the first
    connection cell, inside the doorway passage) for the shortest run
    terminating on non-free cells at both ends, thicken it 3x3, temporarily
    block it, and reassign this region's cells that landed in the other
    component; cut-band cells are finally assigned by neighborhood majority
    vote. If the cut fails to split A|B into two components each containing
    its own peak, labels are kept unchanged.
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
            # The merge-tree saddle gives the narrowest cut direction that
            # can pass through walls; the contact band is only used for
            # topology records in _find_doorways. Fall back to the contact
            # band when there is no connection record.
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
                continue  # cut did not separate the two regions; keep as-is
            out[(labels == a) & (comp == comp_b)] = b
            out[(labels == b) & (comp == comp_a)] = a
            # Cut-band cells (covered by cut, in no component) are assigned
            # by neighborhood majority vote
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
    """Mark cells adjacent (8-connectivity) to a different label as
    unclassified ridges.

    Maximin flooding assigns every cell strictly to one side and produces
    no ridges; here one cell layer on each side of the contact band is
    marked unclassified as an explicit region boundary.
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
