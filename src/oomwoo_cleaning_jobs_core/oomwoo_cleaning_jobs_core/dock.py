"""Dock pose discovery: read-only reference to the Nav2 opennav_docking database.

The dock pose is owned and persisted by the dock-cycle module (Nav2
``opennav_docking`` DockDatabase YAML); this package only reads it and never
stores a copy, so the dock database stays the single source of truth. A
missing or unreadable database yields ``None`` so callers can fall back to
seed-free (global) reachability semantics.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

#: Default distance from the dock contact pose to the staging (undock exit)
#: pose, following the opennav_docking staging-offset convention.
DEFAULT_STAGING_OFFSET_M = 0.7


def load_dock_pose(dock_database_path: str | Path) -> tuple[float, float, float] | None:
    """Return the first dock's map-frame pose ``(x, y, theta)`` from an
    opennav_docking dock database YAML, or ``None`` when the file is missing
    or does not contain a valid dock entry."""
    try:
        with Path(dock_database_path).open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(data, dict):
        return None
    docks = data.get('docks')
    if not isinstance(docks, dict) or not docks:
        return None
    first = next(iter(docks.values()))
    if not isinstance(first, dict):
        return None
    pose = first.get('pose')
    if not isinstance(pose, (list, tuple)) or len(pose) != 3:
        return None
    try:
        x, y, theta = (float(value) for value in pose)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, theta)):
        return None
    return (x, y, theta)


def staging_pose(
    dock_pose: tuple[float, float, float],
    offset_m: float = DEFAULT_STAGING_OFFSET_M,
) -> tuple[float, float]:
    """Undock exit pose ``(x, y)``: ``offset_m`` along the dock frame's x axis.

    This mirrors the opennav_docking staging-pose convention: the robot
    undocks to a known start pose in front of the dock at the start of every
    job. Undock success itself proves this pose is reachable and unobstructed,
    which makes it the natural flood-fill seed for dock-relative reachability
    validation.
    """
    x, y, theta = dock_pose
    return (x + math.cos(theta) * offset_m, y + math.sin(theta) * offset_m)
