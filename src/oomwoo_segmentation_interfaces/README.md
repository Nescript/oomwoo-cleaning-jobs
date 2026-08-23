# oomwoo_segmentation_interfaces

ROS 2 contract shared by every room-segmentation implementation.

## Canonical result

- `LabelGrid.data` has `width * height` entries in `nav_msgs/OccupancyGrid` row-major order.
- Label `0` means unassigned; positive labels identify rooms.
- Implementations must not assign occupied, unknown, or excluded cells.
- `Room` metadata must be derived from `LabelGrid`.
- Equal map, mask, implementation version, and parameters must produce deterministic labels.

`SegmentRooms` is an action because segmentation can be slow and must support progress and cancellation. Implementation-specific tuning is configured on the action-server node, not added to this contract.
