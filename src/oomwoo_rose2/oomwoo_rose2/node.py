"""ROS 2 action server exposing the ROSE2 segmentation provider."""

from __future__ import annotations

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from oomwoo_segmentation.models import SegmentationError
from oomwoo_segmentation.ros_conversions import (
    array_from_mask_grid,
    result_to_ros_messages,
    source_map_from_occupancy_grid,
)
from oomwoo_segmentation_interfaces.action import SegmentRooms

from .engine import Rose2Config, Rose2Segmenter


class Rose2SegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__('oomwoo_rose2')
        self.declare_parameter('action_name', '/room_segmentation/segment')
        defaults = Rose2Config()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        config = Rose2Config(**{
            name: self.get_parameter(name).value
            for name in defaults.__dataclass_fields__
        })
        self._segmenter = Rose2Segmenter(config)
        self._action = ActionServer(
            self,
            SegmentRooms,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            cancel_callback=lambda _request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f'ROSE2 room segmenter {self._segmenter.implementation_version} ready')

    def destroy_node(self):
        self._action.destroy()
        return super().destroy_node()

    def _execute(self, goal_handle):
        response = SegmentRooms.Result()
        try:
            source_map = source_map_from_occupancy_grid(goal_handle.request.map)
            cleanable_mask = None
            if goal_handle.request.use_cleanable_mask:
                cleanable_mask = array_from_mask_grid(
                    goal_handle.request.cleanable_mask, source_map)
        except Exception as exc:
            response.status = SegmentRooms.Result.STATUS_INVALID_REQUEST
            response.message = str(exc)
            goal_handle.abort()
            return response

        def progress(stage: str, value: float) -> None:
            feedback = SegmentRooms.Feedback()
            feedback.stage = stage
            feedback.progress = float(value)
            goal_handle.publish_feedback(feedback)

        try:
            result = self._segmenter.segment(
                source_map,
                cleanable_mask,
                include_diagnostics=goal_handle.request.include_diagnostics,
                progress=progress,
                cancelled=lambda: goal_handle.is_cancel_requested,
            )
        except SegmentationError as exc:
            response.message = str(exc)
            if goal_handle.is_cancel_requested:
                response.status = SegmentRooms.Result.STATUS_CANCELLED
                goal_handle.canceled()
            else:
                response.status = SegmentRooms.Result.STATUS_ALGORITHM_FAILED
                goal_handle.abort()
            return response
        except Exception as exc:  # keep action server alive on provider bugs
            self.get_logger().error(f'unexpected ROSE2 failure: {exc}')
            response.status = SegmentRooms.Result.STATUS_ALGORITHM_FAILED
            response.message = f'unexpected ROSE2 failure: {exc}'
            goal_handle.abort()
            return response

        labels, rooms, walls, diagnostics = result_to_ros_messages(result, source_map)
        if goal_handle.is_cancel_requested:
            response.status = SegmentRooms.Result.STATUS_CANCELLED
            response.message = 'segmentation cancelled'
            goal_handle.canceled()
            return response
        response.status = SegmentRooms.Result.STATUS_SUCCESS
        response.message = 'ok'
        response.implementation_id = result.implementation_id
        response.implementation_version = result.implementation_version
        response.labels = labels
        response.rooms = rooms
        response.walls = walls
        response.diagnostics = diagnostics
        goal_handle.succeed()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Rose2SegmentationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
