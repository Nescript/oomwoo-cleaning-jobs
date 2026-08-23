# Third-party provenance

`oomwoo_rose2/oomwoo_rose2/upstream/` is derived from:

- Project: [aislabunimi/ROSE2](https://github.com/aislabunimi/ROSE2)
- Commit: `3a010b9e6bb2477de3b5b46208ebfccd71dfafbf`
- License: GNU GPL version 3

The original license is installed as `LICENSE`.

## Local changes

- Added Python package markers so the algorithms install with `ament_python`.
- Added a minimal logging shim in place of the ROS 1 `rospy` dependency.
- Added Shapely 2 compatibility by mapping `cascaded_union` to `unary_union`.
- Updated the DBSCAN call to use modern scikit-learn keyword-only arguments without changing parameter values.
- Deferred Voronoi-only imports until `rooms_voronoi` is enabled, so the default upstream path does not load optional `skan`/Voronoi dependencies.
- Fixed two vertical-line branches that read `Y1` before initialization because the `b == 0` guard was evaluated second.
- Replaced the brittle hard-coded second non-free contour with the largest connected free-space contour; this handles cropped maps and avoids selecting furniture as the layout boundary.
- Ignore degenerate cells with fewer than three distinct vertices before constructing Shapely polygons, matching the intended empty/outside-cell behavior on modern Shapely.
- Added `oomwoo_rose2.engine` to adapt `SourceMap`/cleanable masks to the original two-stage ROSE + ROSE2 pipeline and canonical label grids.
- Replaced ROS 1 publishers, services, pickled custom messages, and launch files with the typed `oomwoo_segmentation_interfaces/SegmentRooms` ROS 2 action.
- Added deterministic label canonicalization and in-memory diagnostics.

Algorithm-specific fixes beyond compatibility must be documented here and covered by parity tests.
