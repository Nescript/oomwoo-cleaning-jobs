"""Conversions between canonical Python types and ROS 2 messages."""

from __future__ import annotations

import math

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from oomwoo_segmentation_interfaces.msg import (
    DiagnosticImage as DiagnosticImageMsg,
    LabelGrid,
    MaskGrid,
    Room,
)
from sensor_msgs.msg import CompressedImage

from .models import CandidateRegion, DiagnosticImage, SegmentationResult
from .source_map import SourceMap
from .validation import effective_cleanable_mask


def source_map_from_occupancy_grid(message: OccupancyGrid) -> SourceMap:
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


def mask_grid_from_array(
    mask: np.ndarray,
    source_map: SourceMap,
    *,
    frame_id: str = 'map',
) -> MaskGrid:
    mask = np.asarray(mask, dtype=bool)
    if mask.shape != source_map.cells.shape:
        raise ValueError('mask shape does not match source map')
    message = MaskGrid()
    message.header.frame_id = frame_id
    message.info = occupancy_grid_from_source_map(source_map, frame_id=frame_id).info
    message.data = mask.reshape(-1).astype(np.uint8).tolist()
    return message


def array_from_mask_grid(message: MaskGrid, source_map: SourceMap) -> np.ndarray:
    if (int(message.info.width), int(message.info.height)) != (
        source_map.width, source_map.height
    ):
        raise ValueError('mask dimensions do not match source map')
    data = np.asarray(message.data, dtype=np.uint8)
    if data.size != source_map.width * source_map.height:
        raise ValueError('mask data length does not match width * height')
    return data.reshape(source_map.cells.shape).astype(bool)


def result_to_ros_messages(
    result: SegmentationResult,
    source_map: SourceMap,
    *,
    frame_id: str = 'map',
) -> tuple[LabelGrid, list[Room], list[DiagnosticImageMsg]]:
    grid = LabelGrid()
    grid.header.frame_id = frame_id
    grid.info = occupancy_grid_from_source_map(source_map, frame_id=frame_id).info
    grid.data = result.labels.reshape(-1).astype(np.int32).tolist()

    rooms: list[Room] = []
    for region in result.regions:
        room = Room()
        room.label = region.label
        room.cell_count = region.cell_count
        room.area_m2 = region.area_m2
        rooms.append(room)

    diagnostics: list[DiagnosticImageMsg] = []
    for item in result.diagnostics:
        ok, encoded = cv2.imencode('.png', item.image)
        if not ok:
            raise ValueError(f'failed to encode diagnostic image {item.stage!r}')
        compressed = CompressedImage()
        compressed.header.frame_id = frame_id
        compressed.format = 'png'
        compressed.data = encoded.tobytes()
        message = DiagnosticImageMsg()
        message.stage = item.stage
        message.image = compressed
        diagnostics.append(message)
    return grid, rooms, diagnostics


def result_from_ros_messages(
    labels: LabelGrid,
    rooms: list[Room],
    diagnostics: list[DiagnosticImageMsg],
    source_map: SourceMap,
    cleanable_mask: np.ndarray | None,
    implementation_id: str,
    implementation_version: str,
) -> SegmentationResult:
    data = np.asarray(labels.data, dtype=np.int32)
    if data.size != source_map.width * source_map.height:
        raise ValueError('label data length does not match width * height')
    cleanable = effective_cleanable_mask(source_map, cleanable_mask)
    regions = tuple(CandidateRegion(
        label=int(room.label),
        cell_count=int(room.cell_count),
        area_m2=float(room.area_m2),
    ) for room in rooms)
    decoded: list[DiagnosticImage] = []
    for item in diagnostics:
        image = cv2.imdecode(
            np.frombuffer(bytes(item.image.data), dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if image is None:
            raise ValueError(f'failed to decode diagnostic image {item.stage!r}')
        decoded.append(DiagnosticImage(stage=item.stage, image=image))
    return SegmentationResult(
        labels=np.ascontiguousarray(data.reshape(source_map.cells.shape)),
        regions=regions,
        cleanable_mask=cleanable,
        implementation_id=implementation_id,
        implementation_version=implementation_version,
        diagnostics=tuple(decoded),
    )
