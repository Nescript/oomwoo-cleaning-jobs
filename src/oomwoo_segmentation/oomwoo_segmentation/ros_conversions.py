"""Conversions between canonical Python types and ROS 2 messages."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import cv2
from geometry_msgs.msg import Point, Point32, Polygon
import numpy as np
from nav_msgs.msg import OccupancyGrid
from oomwoo_segmentation_msgs.msg import Room, WallSegment as WallSegmentMsg
from sensor_msgs.msg import Image

from .models import CandidateRegion, DiagnosticImage, SegmentationResult, WallSegment
from .source_map import SourceMap
from .validation import effective_cleanable_mask


def source_map_from_occupancy_grid(message: OccupancyGrid) -> SourceMap:
    """Reconstruct a SourceMap from a ROS 2 OccupancyGrid."""
    info = message.info
    width, height = int(info.width), int(info.height)
    data = np.asarray(message.data, dtype=np.int8)
    if data.size != width * height:
        raise ValueError('OccupancyGrid data length does not match width * height')
    q = info.origin.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return SourceMap(
        float(info.resolution), width, height,
        (float(info.origin.position.x), float(info.origin.position.y), yaw),
        data.reshape((height, width)),
    )


def occupancy_grid_from_source_map(
    source_map: SourceMap,
    *,
    frame_id: str = 'map',
) -> OccupancyGrid:
    """Create a ROS 2 OccupancyGrid from a SourceMap."""
    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.info.resolution = source_map.resolution
    message.info.width = source_map.width
    message.info.height = source_map.height
    x, y, yaw = source_map.origin
    message.info.origin.position.x = x
    message.info.origin.position.y = y
    message.info.origin.orientation.z = math.sin(yaw / 2.0)
    message.info.origin.orientation.w = math.cos(yaw / 2.0)
    message.data = source_map.cells.reshape(-1).astype(np.int8).tolist()
    return message


def image_from_mask(
    mask: np.ndarray,
    *,
    frame_id: str = 'map',
) -> Image:
    """Convert 2D boolean mask to sensor_msgs/Image (mono8)."""
    mask_u8 = (np.asarray(mask, dtype=bool).astype(np.uint8) * 255)
    msg = Image()
    msg.header.frame_id = frame_id
    msg.height, msg.width = mask_u8.shape
    msg.encoding = 'mono8'
    msg.is_bigendian = False
    msg.step = mask_u8.shape[1]
    msg.data = mask_u8.tobytes()
    return msg


def array_from_mask_image(message: Image) -> np.ndarray:
    """Convert sensor_msgs/Image (mono8) to 2D boolean mask."""
    if not message.data:
        return np.zeros((0, 0), dtype=bool)
    arr = np.frombuffer(message.data, dtype=np.uint8).reshape((message.height, message.width))
    return arr > 0


def image_from_labels(
    labels: np.ndarray,
    *,
    frame_id: str = 'map',
) -> Image:
    """Convert 2D int32 label grid to sensor_msgs/Image (32SC1)."""
    labels_i32 = np.asarray(labels, dtype=np.int32)
    msg = Image()
    msg.header.frame_id = frame_id
    msg.height, msg.width = labels_i32.shape
    msg.encoding = '32SC1'
    msg.is_bigendian = False
    msg.step = labels_i32.shape[1] * 4
    msg.data = labels_i32.tobytes()
    return msg


def array_from_label_image(message: Image) -> np.ndarray:
    """Convert sensor_msgs/Image (32SC1) to 2D int32 label array."""
    if not message.data:
        return np.zeros((0, 0), dtype=np.int32)
    return np.frombuffer(message.data, dtype=np.int32).reshape((message.height, message.width))


def walls_to_ros_messages(
    walls: Sequence[WallSegment],
) -> List[WallSegmentMsg]:
    """Convert internal WallSegment objects to ROS WallSegment messages."""
    messages: List[WallSegmentMsg] = []
    for wall in walls:
        msg = WallSegmentMsg()
        msg.start.x, msg.start.y, msg.start.z = wall.x1, wall.y1, 0.0
        msg.end.x, msg.end.y, msg.end.z = wall.x2, wall.y2, 0.0
        msg.support = float(wall.support)
        msg.direction = float(wall.direction_rad)
        messages.append(msg)
    return messages


def walls_from_ros_messages(
    messages: Sequence[WallSegmentMsg],
) -> Tuple[WallSegment, ...]:
    """Convert ROS WallSegment messages to internal WallSegment objects."""
    return tuple(
        WallSegment(
            x1=float(msg.start.x),
            y1=float(msg.start.y),
            x2=float(msg.end.x),
            y2=float(msg.end.y),
            support=float(msg.support),
            direction_rad=float(msg.direction),
        )
        for msg in messages
    )


def result_to_ros_messages(
    result: SegmentationResult,
    source_map: SourceMap,
    *,
    frame_id: str = 'map',
) -> Tuple[Image, List[Room], List[WallSegmentMsg], List[Image]]:
    """Convert SegmentationResult to ROS messages."""
    labels_image = image_from_labels(result.labels, frame_id=frame_id)

    rooms: List[Room] = []
    for region in result.regions:
        room = Room()
        room.id = int(region.label)
        room.area_m2 = float(region.area_m2)

        # Compute centroid from mask
        mask = (result.labels == region.label)
        r_indices, c_indices = np.nonzero(mask)
        if r_indices.size > 0:
            mean_c = float(np.mean(c_indices))
            mean_r = float(np.mean(r_indices))
            mx, my = source_map.map_frame_from_pixel(mean_c, mean_r)
            room.centroid.x = mx
            room.centroid.y = my
            room.centroid.z = 0.0

            # Contour boundary in map frame
            mask_u8 = mask.astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                poly = Polygon()
                for pt in contours[0]:
                    px, py = float(pt[0][0]), float(pt[0][1])
                    wx, wy = source_map.map_frame_from_pixel(px, py)
                    p32 = Point32()
                    p32.x, p32.y, p32.z = wx, wy, 0.0
                    poly.points.append(p32)
                room.boundary = poly

        rooms.append(room)

    walls = walls_to_ros_messages(result.walls)

    diagnostics: List[Image] = []
    for item in result.diagnostics:
        bgr = item.image
        diag_msg = Image()
        diag_msg.header.frame_id = item.stage
        diag_msg.height, diag_msg.width = bgr.shape[:2]
        diag_msg.encoding = 'bgr8'
        diag_msg.is_bigendian = False
        diag_msg.step = bgr.shape[1] * 3
        diag_msg.data = bgr.tobytes()
        diagnostics.append(diag_msg)

    return labels_image, rooms, walls, diagnostics


def result_from_ros_messages(
    labels_image: Image,
    rooms: Sequence[Room],
    walls: Sequence[WallSegmentMsg],
    diagnostics: Sequence[Image],
    source_map: SourceMap,
    cleanable_mask: Optional[np.ndarray],
    implementation_id: str,
    implementation_version: str,
) -> SegmentationResult:
    """Reconstruct a SegmentationResult from ROS messages."""
    labels = array_from_label_image(labels_image)
    cleanable = effective_cleanable_mask(source_map, cleanable_mask)

    regions: List[CandidateRegion] = []
    for room in rooms:
        cell_count = int(np.count_nonzero(labels == room.id))
        regions.append(CandidateRegion(
            label=room.id,
            cell_count=cell_count,
            area_m2=float(room.area_m2),
        ))

    diag_images: List[DiagnosticImage] = []
    for d_msg in diagnostics:
        bgr = np.frombuffer(d_msg.data, dtype=np.uint8).reshape((d_msg.height, d_msg.width, 3))
        diag_images.append(DiagnosticImage(d_msg.header.frame_id, bgr))

    return SegmentationResult(
        labels=labels,
        regions=tuple(regions),
        cleanable_mask=cleanable,
        implementation_id=implementation_id,
        implementation_version=implementation_version,
        walls=walls_from_ros_messages(walls),
        diagnostics=tuple(diag_images),
    )
