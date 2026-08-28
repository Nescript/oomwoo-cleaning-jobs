"""ROS 2 action client for room-segmentation providers."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from oomwoo_segmentation_msgs.action import SegmentRooms

from .models import SegmentationError, SegmentationResult
from .ros_conversions import (
    image_from_mask,
    occupancy_grid_from_source_map,
    result_from_ros_messages,
)
from .source_map import SourceMap
from .validation import validate_result


class RoomSegmentationActionClient(Node):
    """Thin ROS adapter; result validation remains provider-neutral."""

    def __init__(
        self,
        *,
        action_name: str = '/room_segmentation/segment',
        node_name: str = 'oomwoo_segmentation_client',
    ) -> None:
        super().__init__(node_name)
        self._client = ActionClient(self, SegmentRooms, action_name)
        self._goal_handle: Optional[ClientGoalHandle] = None

    def segment_async(
        self,
        source_map: SourceMap,
        cleanable_mask: Optional[np.ndarray] = None,
        *,
        include_diagnostics: bool = False,
        server_timeout_sec: float = 10.0,
    ) -> Future:
        completed = Future()
        if not self._client.wait_for_server(timeout_sec=server_timeout_sec):
            completed.set_exception(SegmentationError('segmentation action server unavailable'))
            return completed

        goal = SegmentRooms.Goal()
        goal.map = occupancy_grid_from_source_map(source_map)
        if cleanable_mask is not None:
            goal.cleanable_mask = image_from_mask(cleanable_mask)
        goal.include_diagnostics = include_diagnostics

        sent = self._client.send_goal_async(goal)

        def goal_response(done: Future) -> None:
            try:
                handle: Optional[ClientGoalHandle] = done.result()
                if handle is None or not handle.accepted:
                    raise SegmentationError('segmentation goal was rejected')
                self._goal_handle = handle
                received = handle.get_result_async()
                received.add_done_callback(result_response)
            except Exception as exc:
                completed.set_exception(exc)

        def result_response(done: Future) -> None:
            try:
                res = done.result()
                if res is None:
                    raise SegmentationError('action server returned no result')
                response = res.result
                if response.status != SegmentRooms.Result.STATUS_SUCCESS:
                    raise SegmentationError(response.message or 'segmentation failed')
                result = result_from_ros_messages(
                    response.labels,
                    list(response.rooms),
                    list(response.walls),
                    list(response.diagnostics),
                    source_map,
                    cleanable_mask,
                    'oomwoo_segmentation',
                    response.implementation_version,
                )
                validate_result(result, source_map)
                completed.set_result(result)
            except Exception as exc:
                completed.set_exception(exc)
            finally:
                self._goal_handle = None

        sent.add_done_callback(goal_response)
        return completed

    def cancel_async(self) -> Optional[Future]:
        if self._goal_handle is None:
            return None
        return self._goal_handle.cancel_goal_async()


def segment_once(
    source_map: SourceMap,
    cleanable_mask: Optional[np.ndarray] = None,
    *,
    action_name: str = '/room_segmentation/segment',
    include_diagnostics: bool = False,
    timeout_sec: float = 120.0,
) -> SegmentationResult:
    """Blocking convenience for command-line tools."""
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = RoomSegmentationActionClient(action_name=action_name)
    try:
        future = node.segment_async(
            source_map,
            cleanable_mask,
            include_diagnostics=include_diagnostics,
        )
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise SegmentationError(f'segmentation timed out after {timeout_sec:.1f}s')
        result = future.result()
        if result is None:
            raise SegmentationError('segmentation produced no result')
        return result
    finally:
        node.destroy_node()
        if owns_context:
            rclpy.shutdown()
