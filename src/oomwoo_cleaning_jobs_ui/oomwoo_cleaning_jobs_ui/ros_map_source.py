"""Optional live /map input. Qt receives snapshots through a caller callback."""
from __future__ import annotations
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from .map_source import source_map_from_occupancy_grid

class RosMapSource(Node):
    def __init__(self, on_map, topic='/map'):
        super().__init__('oomwoo_cleaning_jobs_ui_map_source')
        qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self._on_map=on_map
        self._subscription=self.create_subscription(OccupancyGrid,topic,self._received,qos)
    def _received(self, message):
        self._on_map(source_map_from_occupancy_grid(message))
