"""Planner-level Nav2 integration test for keepout constraint projection.

Brings up a minimal Nav2 planning stack as subprocesses (map_server +
planner_server + lifecycle bringup + static TF) plus the real
constraint_mask_publisher node, then verifies through the
ComputePathToPose action that:

- paths steer around Keepout cells ("no-go zones are never entered" at
  the planner level),
- a goal inside a Keepout fails,
- a goal inside a Virtual-Wall-enclosed room fails (the enclosure itself
  is not in the mask; navigation topology makes it unreachable).

Requires a sourced ROS 2 Jazzy environment with nav2_planner installed.
"""

import os
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')

from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.action.client import GoalStatus

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

from fixtures import fake_segmentation, make_two_rooms_map, write_map_files

from oomwoo_cleaning_jobs.constraint_mask_publisher import ConstraintMaskPublisher
from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout, VirtualWall
from oomwoo_cleaning_jobs_core.persistence import RegionSetStore
from oomwoo_cleaning_jobs_core.regions import RegionSet


def _map_point(source, row, col):
    """Map-frame center point of a cell (identity yaw fixtures only)."""
    x, y, _ = source.origin
    return (x + (col + 0.5) * source.resolution,
            y + (row + 0.5) * source.resolution)


def _cell_of(source, x, y):
    ox, oy, _ = source.origin
    return (int((y - oy) / source.resolution), int((x - ox) / source.resolution))


def _publish_set(store_root, source, constraints):
    keepout = constraints.mask_for(source)
    result = fake_segmentation(source, cleanable_mask=source.free_mask() & ~keepout)
    region_set = RegionSet.from_segmentation(
        result, resolution=source.resolution, origin=source.origin,
        base_cleanable=source.free_mask(), keepout_mask=keepout)
    RegionSetStore(store_root).publish(source, region_set, constraints)


TABLE = Keepout('table', ((-2.0, 0.0), (-1.8, 0.0), (-1.8, 0.2), (-2.0, 0.2)))


@pytest.fixture(scope='module')
def nav2_stack(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('nav2_keepout')
    source = make_two_rooms_map()
    map_yaml = write_map_files(tmp, source, 'map')
    store_root = tmp / 'store'
    _publish_set(store_root, source, ConstraintSet(keepouts=(TABLE,)))

    params_file = tmp / 'nav2_params.yaml'
    params_file.write_text(yaml.safe_dump({
        'planner_server': {'ros__parameters': {
            'use_sim_time': False,
            'expected_planner_frequency': 1.0,
            'planner_plugins': ['GridBased'],
            'GridBased': {
                'plugin': 'nav2_navfn_planner::NavfnPlanner',
                'tolerance': 0.0,
                'use_astar': False,
                'allow_unknown': True,
            },
        }},
        # The costmap is a sub-node named 'global_costmap'; like the official
        # nav2_bringup params, its section lives at the top level.
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'use_sim_time': False,
            'robot_base_frame': 'base_link',
            'global_frame': 'map',
            'rolling_window': False,
            'track_unknown_space': True,
            'update_frequency': 10.0,
            'publish_frequency': 5.0,
            'plugins': ['static_layer', 'inflation_layer', 'keepout_filter'],
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer'},
            'inflation_layer': {
                'plugin': 'nav2_costmap_2d::InflationLayer',
                'inflation_radius': 0.3,
                'cost_scaling_factor': 3.0,
            },
            'keepout_filter': {
                'plugin': 'nav2_costmap_2d::KeepoutFilter',
                'filter_info_topic': '/costmap_filter_info',
                'override_lethal_cost': True,
            },
        }}},
        'lifecycle_manager_navigation': {'ros__parameters': {
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server', 'planner_server'],
        }},
    }), encoding='utf-8')

    procs = [
        subprocess.Popen(
            ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
             '-r', '__node:=map_server',
             '-p', f'yaml_filename:={map_yaml}', '-p', 'use_sim_time:=false'],
            env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(
            ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args',
             '--params-file', str(params_file)],
            env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(
            ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
             '-r', '__node:=lifecycle_manager_navigation',
             '--params-file', str(params_file)],
            env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(
            ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', '1', 'map', 'base_link'],
            env=os.environ, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    ]

    rclpy.init()
    mask_node = ConstraintMaskPublisher(parameter_overrides=[
        Parameter('maps_root', value=str(store_root)),
        Parameter('map_hash', value=source.identity),
    ])
    client = rclpy.create_node('keepout_planner_test_client')
    action = ActionClient(client, ComputePathToPose, 'compute_path_to_pose')
    if not action.wait_for_server(timeout_sec=90.0):
        for proc in procs:
            proc.kill()
        mask_node.destroy_node()
        client.destroy_node()
        rclpy.shutdown()
        pytest.skip('Nav2 planner stack did not come up in time')

    yield SimpleNamespace(source=source, client=client, action=action,
                          mask_node=mask_node, store_root=store_root)

    for proc in procs:
        proc.terminate()
    deadline = time.monotonic() + 10.0
    for proc in procs:
        try:
            proc.wait(timeout=max(0.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            proc.kill()
    mask_node.destroy_node()
    client.destroy_node()
    rclpy.shutdown()


def _pose(frame_time, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = frame_time
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def _compute_path(stack, start_xy, goal_xy, timeout_s=30.0):
    goal = ComputePathToPose.Goal()
    stamp = stack.client.get_clock().now().to_msg()
    goal.start = _pose(stamp, *start_xy)
    goal.goal = _pose(stamp, *goal_xy)
    goal.planner_id = 'GridBased'
    goal.use_start = True
    send = stack.action.send_goal_async(goal)
    rclpy.spin_until_future_complete(stack.client, send, timeout_sec=timeout_s)
    handle = send.result()
    assert handle is not None and handle.accepted, 'planning goal rejected'
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(stack.client, future, timeout_sec=timeout_s)
    wrapped = future.result()
    error_code = getattr(wrapped.result, 'error_code', 0)
    succeeded = (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                 and error_code == 0 and len(wrapped.result.path.poses) > 0)
    return succeeded, wrapped.result.path


def test_planner_path_avoids_keepout_cells(nav2_stack):
    source = nav2_stack.source
    keepout_mask = ConstraintSet(keepouts=(TABLE,)).mask_for(source)
    start = _map_point(source, 40, 7)    # left of the table, left room
    goal = _map_point(source, 40, 20)    # right of the table, left room

    succeeded, path = _compute_path(nav2_stack, start, goal)

    assert succeeded, 'a detour around the table must exist'
    for pose in path.poses:
        row, col = _cell_of(source, pose.pose.position.x, pose.pose.position.y)
        assert not keepout_mask[row, col], f'path entered keepout cell {(row, col)}'


def test_goal_inside_keepout_fails(nav2_stack):
    source = nav2_stack.source
    start = _map_point(source, 40, 7)
    goal = _map_point(source, 42, 12)  # inside the table Keepout polygon

    succeeded, _ = _compute_path(nav2_stack, start, goal)

    assert not succeeded


def test_goal_inside_virtual_wall_enclosure_fails(nav2_stack):
    source = nav2_stack.source
    # Re-publish with a Virtual Wall sealing the doorway and reload the mask.
    constraints = ConstraintSet(virtual_walls=(
        VirtualWall('door-seal', _map_point(source, 34, 30),
                    _map_point(source, 45, 30), source.resolution),))
    _publish_set(nav2_stack.store_root, source, constraints)
    assert nav2_stack.mask_node.reload()

    start = _map_point(source, 40, 15)  # left room
    goal = _map_point(source, 40, 50)   # right room, behind the sealed doorway

    # The reloaded mask reaches the costmap on its next update cycle; poll
    # until the enclosure becomes unreachable (bounded), then assert.
    deadline = time.monotonic() + 15.0
    succeeded = True
    while succeeded and time.monotonic() < deadline:
        succeeded, _ = _compute_path(nav2_stack, start, goal)
        if succeeded:
            time.sleep(0.3)

    assert not succeeded, 'the enclosed room must be unreachable for the planner'
