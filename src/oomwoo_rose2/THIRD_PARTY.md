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
- Deduplicate detected lines that exactly coincide with the four synthetic frame lines before edge/cell construction; duplicate borders previously produced a triangular lower-left cell with missing opposite edges.
- Raise the default retained-line support floor from upstream `0.0` to `0.22`; this rejects the demonstrated low-support full-map line generated from furniture while preserving the regression fixtures' structural walls. The value remains a provider parameter.
- Supplement zero-weight or partial-support retained edges with local support sampled from the ROSE structural raster. This recovers wall runs omitted by probabilistic Hough extraction.
- Introduce a hard wall barrier in affinity matrix generation (`hard_wall_threshold=0.40`), strictly preventing DBSCAN from merging topological cells across confirmed physical walls.
- Add constrained geodesic wavefront propagation (`_geodesic_coverage`) to ensure 100% of reachable cleanable free cells are assigned to room regions without wall bleeding, while filtering sub-10px noise artifacts.
- Reject a frame-side cell only when a wall has at least 0.9 support, spans at least 60% of the shorter layout dimension, and separates a frame-adjacent cell no larger than 15% of its neighbor. This prevents a noisy free map margin from becoming a room.
- Added `oomwoo_rose2.engine` to adapt `SourceMap`/cleanable masks to the original two-stage ROSE + ROSE2 pipeline and canonical label grids.
- Replaced ROS 1 publishers, services, pickled custom messages, and launch files with the typed `oomwoo_segmentation_interfaces/SegmentRooms` ROS 2 action.
- Added deterministic label canonicalization and in-memory diagnostics.

## Test map fixtures

`oomwoo_rose2/test/maps/rose2_upstream/` is a verbatim copy of the upstream
repository's `src/maps/` test maps at the same pinned commit (carmen,
Freiburg_Building_079, Virtual, maps-nostre). Upstream's
`Virtual/mapirlab.yaml` references a `mapirlab.pgm` that upstream never
committed; the YAML is kept for provenance but is not runnable. These maps
are used only for segmentation verification runs whose outputs go to
`output/rose2_upstream/`.

Algorithm-specific fixes beyond compatibility must be documented here and covered by parity tests.
