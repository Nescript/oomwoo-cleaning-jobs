"""Nav2 keepout filter mask publisher for Published Region Set constraints.

Projects the persisted spatial constraints (Keepouts and Virtual Wall bands)
of the single Published Region Set onto the Nav2 costmaps via the standard
KeepoutFilter mechanism:

- Publishes ``nav2_msgs/CostmapFilterInfo`` (type 0 = keepout) latched,
  always *before* the mask, because the filter learns the mask topic name
  from this message;
- Publishes the materialized ``keepout_mask.pgm/.yaml`` of the published
  generation as a latched ``nav_msgs/OccupancyGrid`` (100 = constraint cell);
- Reloads and republishes on the ``reload_keepout_mask`` Trigger service,
  which the publish transaction calls after atomically switching the
  published generation.

Only the published generation is ever served: drafts are preview-only
(see DEVELOPMENT.md "Nav2 keepout filter projection"). A missing mask at
startup is an explicit degraded state (ERROR log, nothing published) that
recovers through the reload service once a Region Set is published.
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from oomwoo_cleaning_jobs_core.persistence import DEFAULT_STORAGE_ROOT
from oomwoo_segmentation.map_io import load_map_file

#: The Nav2 KeepoutFilter subscribes with latched (transient local) QoS, so a
#: matching durability is mandatory for the messages to be delivered at all.
LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

KEEPOUT_FILTER_TYPE = 0  # CostmapFilterInfo: 0 = keepout/lanes filter


class ConstraintMaskPublisher(Node):
    """Serves the published generation's keepout mask to Nav2 costmap filters."""

    def __init__(self, **kwargs) -> None:
        super().__init__('constraint_mask_publisher', **kwargs)
        self.declare_parameter('maps_root', str(DEFAULT_STORAGE_ROOT))
        self.declare_parameter('map_hash', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('filter_info_topic', '/costmap_filter_info')
        self.declare_parameter('filter_mask_topic', '/keepout_filter_mask')

        self._maps_root = Path(self.get_parameter('maps_root').value)
        self._map_hash = str(self.get_parameter('map_hash').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._mask_topic = str(self.get_parameter('filter_mask_topic').value)

        self._info_pub = self.create_publisher(
            CostmapFilterInfo, str(self.get_parameter('filter_info_topic').value),
            LATCHED_QOS)
        self._mask_pub = self.create_publisher(
            OccupancyGrid, self._mask_topic, LATCHED_QOS)
        self._reload_srv = self.create_service(
            Trigger, 'reload_keepout_mask', self._on_reload)

        if not self._map_hash:
            self.get_logger().error(
                'map_hash parameter is empty; nothing to serve until it is set')
        elif not self.reload():
            self.get_logger().error(
                'no published keepout mask found; running without keepout '
                'constraints until a Region Set is published (degraded state)')

    def _mask_yaml(self) -> Path:
        return (self._maps_root / self._map_hash / 'published' / 'keepout_mask.yaml')

    def reload(self) -> bool:
        """Load the published generation's mask and (re)publish info + mask."""
        if not self._map_hash:
            return False
        yaml_path = self._mask_yaml()
        try:
            source_map = load_map_file(yaml_path)
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().warn(f'failed to load keepout mask {yaml_path}: {error}')
            return False

        stamp = self.get_clock().now().to_msg()
        # Info first: the filter learns the mask topic name from this message.
        info = CostmapFilterInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.type = KEEPOUT_FILTER_TYPE
        info.filter_mask_topic = self._mask_topic
        info.base = 0.0
        info.multiplier = 1.0
        self._info_pub.publish(info)

        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self._frame_id
        grid.info.resolution = float(source_map.resolution)
        grid.info.width = int(source_map.width)
        grid.info.height = int(source_map.height)
        ox, oy, yaw = source_map.origin
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.z = math.sin(yaw / 2.0)
        grid.info.origin.orientation.w = math.cos(yaw / 2.0)
        grid.data = source_map.cells.reshape(-1).astype(int).tolist()
        self._mask_pub.publish(grid)

        constrained = int((source_map.cells > 0).sum())
        self.get_logger().info(
            f'published keepout mask {yaml_path} '
            f'({source_map.width}x{source_map.height}, {constrained} constraint cells)')
        return True

    def _on_reload(self, _request, response):
        if self.reload():
            response.success = True
            response.message = 'keepout mask reloaded and republished'
        else:
            response.success = False
            response.message = 'no readable published keepout mask'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConstraintMaskPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
