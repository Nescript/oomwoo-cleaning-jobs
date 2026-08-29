"""Tests for dock pose discovery (opennav_docking dock database reference)."""

import math

import yaml

from oomwoo_cleaning_jobs_core.dock import (
    DEFAULT_STAGING_OFFSET_M,
    load_dock_pose,
    staging_pose,
)


def _write_dock_db(tmp_path, data) -> str:
    path = tmp_path / 'dock_database.yaml'
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return str(path)


def test_load_dock_pose_reads_first_dock(tmp_path):
    path = _write_dock_db(tmp_path, {
        'docks': {
            'dock1': {'type': 'charging_dock', 'frame': 'map', 'pose': [1.0, 2.0, 0.5]},
            'dock2': {'type': 'charging_dock', 'frame': 'map', 'pose': [3.0, 4.0, 1.0]},
        },
    })
    x, y, theta = load_dock_pose(path)
    assert (x, y) == (1.0, 2.0)
    assert math.isclose(theta, 0.5)


def test_load_dock_pose_missing_or_invalid_returns_none(tmp_path):
    assert load_dock_pose(tmp_path / 'does_not_exist.yaml') is None
    assert load_dock_pose(_write_dock_db(tmp_path, 'just a string')) is None
    assert load_dock_pose(_write_dock_db(tmp_path, {'docks': {}})) is None
    assert load_dock_pose(_write_dock_db(tmp_path, {'no_docks_key': True})) is None
    assert load_dock_pose(_write_dock_db(tmp_path, {
        'docks': {'dock1': {'type': 'charging_dock'}},  # no pose
    })) is None
    assert load_dock_pose(_write_dock_db(tmp_path, {
        'docks': {'dock1': {'pose': [1.0, 'bad', 0.0]}},
    })) is None
    assert load_dock_pose(_write_dock_db(tmp_path, {
        'docks': {'dock1': {'pose': [1.0, 2.0]}},  # incomplete pose
    })) is None
    garbage = tmp_path / 'garbage.yaml'
    garbage.write_text('{unclosed: [', encoding='utf-8')
    assert load_dock_pose(garbage) is None


def test_staging_pose_offsets_along_dock_orientation():
    assert staging_pose((1.0, 2.0, 0.0), offset_m=0.5) == (1.5, 2.0)
    x, y = staging_pose((1.0, 2.0, math.pi / 2), offset_m=0.5)
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 2.5, abs_tol=1e-9)
    x, y = staging_pose((0.0, 0.0, math.pi), offset_m=DEFAULT_STAGING_OFFSET_M)
    assert math.isclose(x, -DEFAULT_STAGING_OFFSET_M, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
