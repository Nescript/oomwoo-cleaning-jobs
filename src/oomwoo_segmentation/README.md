# oomwoo_segmentation

Algorithm-neutral room-segmentation tooling. This package contains no segmentation algorithm.

It owns:

- canonical Python result types and contract validation;
- Nav2 trinary map loading and `OccupancyGrid` conversion;
- the `SegmentRooms` action client;
- deterministic source-map and room-label rendering;
- the `oomwoo-render-map` CLI.

Concrete providers such as `oomwoo_rose2` run as separate ROS 2 action servers implementing `oomwoo_segmentation_interfaces`.
