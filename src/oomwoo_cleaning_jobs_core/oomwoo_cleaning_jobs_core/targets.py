"""Cleaning target configuration (whole-map, selected regions, spot cleaning).

Translates cleaning intents into concrete target region labels and an associated
runtime RegionSet view for downstream job orchestration. Spot cleaning builds a
transient RegionSet without mutating persistent room partitions, while retaining
the last-used spot area at the constraint layer.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from oomwoo_segmentation.source_map import SourceMap

from .constraints import ConstraintSet, Point, SpotArea, _rasterize_polygon
from .regions import RegionInfo, RegionSet
from .validation import validate_region_set


@dataclass(frozen=True)
class CleaningTarget:
    """Configured target for cleaning job orchestration.

    Holds the sequence of target region labels to be cleaned and a runtime
    RegionSet view that downstream modules can query for raster masks and
    spatial outlines.
    """

    target_labels: tuple[int, ...]
    region_set: RegionSet

    def __post_init__(self) -> None:
        if not self.target_labels:
            raise ValueError('target_labels must not be empty')
        # Validate that all target labels exist in the region_set
        for label in self.target_labels:
            if not self.region_set.mask_of(label).any():
                raise ValueError(f'Target region {label} does not exist or is empty in region_set')

    @property
    def labels(self) -> tuple[int, ...]:
        """Alias for target_labels."""
        return self.target_labels

    def mask_of(self, label: int) -> np.ndarray:
        """Return the raster mask for the given target region label."""
        if label not in self.target_labels:
            raise ValueError(f'Label {label} is not in target_labels {self.target_labels}')
        return self.region_set.mask_of(label)

    def outline(self, label: int) -> list[np.ndarray]:
        """Return derived map-frame outlines for the given target region label."""
        if label not in self.target_labels:
            raise ValueError(f'Label {label} is not in target_labels {self.target_labels}')
        return self.region_set.outline(label)

    def regions(self) -> list[RegionInfo]:
        """Return RegionInfo descriptors for targets in target_labels order."""
        info_by_label = {r.label: r for r in self.region_set.regions()}
        return [info_by_label[lbl] for lbl in self.target_labels if lbl in info_by_label]


def configure_whole_map(region_set: RegionSet) -> CleaningTarget:
    """Configure a whole-map cleaning target containing all published/active regions."""
    regions = region_set.regions()
    if not regions:
        raise ValueError('RegionSet contains no regions for whole-map cleaning')
    labels = tuple(r.label for r in regions)
    return CleaningTarget(target_labels=labels, region_set=region_set)


def configure_selected_regions(
    region_set: RegionSet,
    labels: Sequence[int],
) -> CleaningTarget:
    """Configure a cleaning target for specific selected room/region labels,
    preserving the provided sequence order."""
    selected = tuple(int(lbl) for lbl in labels)
    if not selected:
        raise ValueError('Selected region labels must not be empty')
    existing_labels = {r.label for r in region_set.regions()}
    for lbl in selected:
        if lbl not in existing_labels:
            raise ValueError(f'Region {lbl} does not exist in RegionSet')
    return CleaningTarget(target_labels=selected, region_set=region_set)


def create_spot_region_set(
    source_map: SourceMap,
    constraints: ConstraintSet,
    spot: SpotArea | tuple[Point, ...] | Sequence[Point],
    name: str = 'Spot Area',
    robot_inscribed_radius: float = 0.17,
    identifier: str = 'spot_area',
) -> tuple[RegionSet, SpotArea]:
    """Create a transient, isolated RegionSet containing a single spot cleaning region.

    The spot area is clipped against known-free space and Keepouts without modifying
    the persistent room partition, and validated against the robot footprint radius.
    """
    if isinstance(spot, SpotArea):
        spot_area = spot
    else:
        spot_area = SpotArea(identifier=identifier, vertices=tuple(spot), name=name)

    raw_spot_mask = _rasterize_polygon(spot_area.vertices, source_map)
    keepout_mask = constraints.mask_for(source_map)
    free_mask = source_map.free_mask()
    cleanable = free_mask & ~keepout_mask
    effective_spot_mask = raw_spot_mask & cleanable

    if not effective_spot_mask.any():
        raise ValueError('Spot area contains no cleanable space after clipping with map and keepouts')

    labels = np.zeros(source_map.cells.shape, dtype=np.int32)
    labels[effective_spot_mask] = 1

    temp_region_set = RegionSet(
        labels=labels,
        cleanable=cleanable,
        resolution=source_map.resolution,
        origin=source_map.origin,
        names={1: spot_area.name},
        base_cleanable=free_mask,
        keepout_mask=keepout_mask,
    )

    if robot_inscribed_radius > 0:
        report = validate_region_set(temp_region_set, robot_inscribed_radius, keepout_mask)
        if not report.ok:
            codes = ', '.join(issue.code for issue in report.errors)
            raise ValueError(f'Spot area failed validation: {codes}')

    return temp_region_set, spot_area


def configure_spot_area(
    source_map: SourceMap,
    constraints: ConstraintSet,
    spot: SpotArea | tuple[Point, ...] | Sequence[Point],
    name: str = 'Spot Area',
    robot_inscribed_radius: float = 0.17,
    identifier: str = 'spot_area',
) -> tuple[CleaningTarget, ConstraintSet]:
    """Configure a spot cleaning target from user geometry, returning the CleaningTarget
    and an updated ConstraintSet that retains this spot area as the last used."""
    temp_region_set, spot_area = create_spot_region_set(
        source_map=source_map,
        constraints=constraints,
        spot=spot,
        name=name,
        robot_inscribed_radius=robot_inscribed_radius,
        identifier=identifier,
    )
    updated_constraints = constraints.with_spot_area(spot_area)
    target = CleaningTarget(target_labels=(1,), region_set=temp_region_set)
    return target, updated_constraints


def configure_last_spot_area(
    source_map: SourceMap,
    constraints: ConstraintSet,
    robot_inscribed_radius: float = 0.17,
) -> CleaningTarget:
    """Configure a spot cleaning target using the last saved spot area in constraints."""
    if constraints.spot_area is None:
        raise ValueError('No previously saved spot area found in constraints')
    target, _ = configure_spot_area(
        source_map=source_map,
        constraints=constraints,
        spot=constraints.spot_area,
        name=constraints.spot_area.name,
        robot_inscribed_radius=robot_inscribed_radius,
        identifier=constraints.spot_area.identifier,
    )
    return target
