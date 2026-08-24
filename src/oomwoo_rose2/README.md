# oomwoo_rose2

GPLv3 ROS 2 port of the pinned `aislabunimi/ROSE2` implementation.

The package preserves the upstream computational stages:

```text
OccupancyGrid -> ROSE structural filtering and dominant directions
              -> ROSE2 walls/extended lines/edges/cells/DBSCAN
              -> room polygons -> canonical LabelGrid
```

It intentionally replaces ROS 1 wrappers and pickled message payloads with the typed `oomwoo_segmentation_interfaces/action/SegmentRooms` action.

## Run

```bash
ros2 launch oomwoo_rose2 rose2.launch.py
```

The action is available at `/room_segmentation/segment`. Parameters are in `config/rose2.yaml` and are based on the pinned upstream `ROSE.launch` profile. The documented OOMWOO profile raises `lines_threshold` to `0.22` to reject low-support furniture-derived full-map lines; deployments can override it as a node parameter. Compatibility safeguards also recover zero-weight retained edges from the ROSE structural raster and reject only strictly bounded frame-side fringe cells behind layout-spanning high-support walls. `rooms_voronoi` enables the optional upstream Voronoi refinement and its additional `networkx`, `skan`, and plotting dependencies.
