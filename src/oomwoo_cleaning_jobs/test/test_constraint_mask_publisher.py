"""Integration tests for the constraint_mask_publisher node.

Runs a real rclpy node against a RegionSetStore-produced published
generation; verifies latched delivery to late subscribers, mask content,
degraded startup, and reload semantics.
"""

import math

import numpy as np
import pytest

import rclpy
from rclpy.parameter import Parameter

from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from fixtures import fake_segmentation, make_two_rooms_map

from oomwoo_cleaning_jobs.constraint_mask_publisher import (
    LATCHED_QOS,
    ConstraintMaskPublisher,
)
from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout
from oomwoo_cleaning_jobs_core.persistence import RegionSetStore
from oomwoo_cleaning_jobs_core.regions import RegionSet


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _published_store(tmp_path):
    """Publish a Region Set with one Keepout into tmp_path; return (source, store)."""
    source = make_two_rooms_map()
    constraints = ConstraintSet(
        keepouts=(Keepout('table', ((-2.0, 0.0), (-1.8, 0.0),
                                    (-1.8, 0.2), (-2.0, 0.2))),))
    keepout = constraints.mask_for(source)
    result = fake_segmentation(source, cleanable_mask=source.free_mask() & ~keepout)
    region_set = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin,
        base_cleanable=source.free_mask(), keepout_mask=keepout)
    store = RegionSetStore(tmp_path)
    store.publish(source, region_set, constraints)
    return source, constraints


def _make_node(tmp_path, map_hash):
    return ConstraintMaskPublisher(parameter_overrides=[
        Parameter('maps_root', value=str(tmp_path)),
        Parameter('map_hash', value=map_hash),
    ])


def _spin_until(node, predicate, timeout_s=5.0):
    import time
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.1)
    return predicate()


def _call_trigger(node, client, timeout_s=5.0):
    """Call a Trigger service without deadlocking the single-threaded executor."""
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    assert future.done(), 'service call timed out'
    return future.result()


def test_latched_info_and_mask_reach_late_subscriber(ros_context, tmp_path):
    source, constraints = _published_store(tmp_path)
    node = _make_node(tmp_path, source.identity)

    received = {}
    listener = rclpy.create_node('test_listener')
    listener.create_subscription(
        CostmapFilterInfo, '/costmap_filter_info',
        lambda msg: received.setdefault('info', msg), LATCHED_QOS)
    listener.create_subscription(
        OccupancyGrid, '/keepout_filter_mask',
        lambda msg: received.setdefault('mask', msg), LATCHED_QOS)

    assert _spin_until(listener, lambda: 'info' in received and 'mask' in received)

    info = received['info']
    assert info.type == 0
    assert info.filter_mask_topic == '/keepout_filter_mask'

    grid = received['mask']
    assert grid.info.width == source.width
    assert grid.info.height == source.height
    assert grid.info.resolution == pytest.approx(source.resolution)
    assert grid.info.origin.position.x == pytest.approx(source.origin[0])
    assert grid.info.origin.position.y == pytest.approx(source.origin[1])
    assert grid.info.origin.orientation.w == pytest.approx(1.0)

    data = np.asarray(grid.data, dtype=np.int8).reshape(source.height, source.width)
    expected = constraints.mask_for(source)
    assert (data[expected] == 100).all()
    assert (data[~expected] == 0).all()

    listener.destroy_node()
    node.destroy_node()


def test_missing_mask_startup_degrades_then_reload_recovers(ros_context, tmp_path):
    # Node starts before anything was published: degraded but alive.
    source, _ = _published_store(tmp_path / 'later')
    node = _make_node(tmp_path / 'later', source.identity)
    client = node.create_client(Trigger, 'reload_keepout_mask')
    assert client.wait_for_service(timeout_sec=5.0)

    response = _call_trigger(node, client)
    assert response.success  # store.publish above already wrote the mask

    received = {}
    listener = rclpy.create_node('test_listener2')
    listener.create_subscription(
        OccupancyGrid, '/keepout_filter_mask',
        lambda msg: received.setdefault('mask', msg), LATCHED_QOS)
    assert _spin_until(listener, lambda: 'mask' in received)
    assert received['mask'].info.width == source.width

    listener.destroy_node()
    node.destroy_node()


def test_reload_fails_cleanly_without_any_published_set(ros_context, tmp_path):
    node = _make_node(tmp_path, 'no-such-map-hash')
    client = node.create_client(Trigger, 'reload_keepout_mask')
    assert client.wait_for_service(timeout_sec=5.0)

    response = _call_trigger(node, client)
    assert not response.success
    assert 'no readable' in response.message

    node.destroy_node()
