# oomwoo_cleaning_jobs development context

This is the single development context and local source of truth for this repository. It records the current scope, architecture, design decisions, domain rules, algorithm details, verification baselines, and open boundaries. Read it in full before making any design, implementation, testing, review, or documentation change.

When implementation, verified source-code facts, or user decisions change, update this document in the same change. Do not create a second design document, RFC, baseline, or separate context file.

## Goal and scope

`oomwoo_cleaning_jobs` owns **user cleaning intent and long-running job orchestration on saved maps**: whole-map, selected-Region, and spot cleaning; Region/virtual wall/keepout editing and persistence; Job lifecycle, pause, resume, retry, and summary.

Boundary definitions:
- It does not own coverage path planning or low-level motion execution. Regular saved-map coverage reuses `oomwoo_coverage`; Nav2 is the motion execution layer.
- `clean-and-map` is an RFC / algorithm reference for initial exploratory cleaning (SLAM, exploration, and coverage), not the backend or coverage-progress provider for saved-map Jobs.
- `floor-care` is a future perimeter/edge pass; it can be combined with `oomwoo_coverage`'s interior sweep but is not a general coverage backend.

External references:
- [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs)
- [clean-and-map RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/clean-and-map)
- [floor-care RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/floor-care)
- [oomwoo-ros2-tools](https://github.com/makerspet/oomwoo-ros2-tools)
- [SOFTWARE_INTERFACES.md](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

## Current phase and package architecture

Phase 1 delivers:

`saved map → automatic candidate regions → manual editing → validation → Published Region Set persistence`

### Package layout

| Package | License | Responsibility |
| --- | --- | --- |
| `src/oomwoo_segmentation_msgs` | Apache-2.0 | Standard-type ROS 2 `SegmentRooms` action (`OccupancyGrid` + optional mask in, 32SC1 labels + `Room[]` + `WallSegment[]` out) and messages |
| `src/oomwoo_segmentation` | GPL-3.0-only | Room-segmentation engine based on the pinned ROSE + ROSE2 pipeline (`engine/`), ROS 2 action server (`oomwoo_segmentation_node`) and client, Source Map model and Nav2 trinary I/O, canonical contract validation, deterministic rendering, and `oomwoo-render-map` CLI |
| `src/oomwoo_cleaning_jobs_core` | Apache-2.0 | Region Set representation and editing (brush paint/erase/merge/split/preemption), spatial constraints (`Keepout`, `VirtualWall`, `SpotArea`), cleaning target configuration, publish validation grading, draft/published persistence (`RegionSetStore`), and dock pose reference (`dock.py`) |
| `src/oomwoo_cleaning_jobs` | Apache-2.0 | ROS 2 nodes for cleaning intent: `constraint_mask_publisher` (Nav2 keepout filter projection) and, later, job orchestration |

### Architecture and runtime boundaries

1. **Headless domain core & action server**: Phase 1 focuses on the headless domain libraries, segmentation engine, action interfaces, and test suites. Verification is conducted via automated tests and rendered artifacts under `output/`; no GUI package ships in this branch.
2. **Algorithm seam**: Automatic partitioning is fully encapsulated behind `oomwoo_segmentation_msgs/action/SegmentRooms`. Callers and downstream consumers depend only on this action interface and standard ROS 2 message types, remaining completely decoupled from algorithm internals.
3. **No legacy fallbacks**: This branch runs only the engine shipped in `oomwoo_segmentation`. All legacy algorithms (maximin, watershed, saddle merge-tree, skeleton doorway clipping) have been eliminated. Failures are explicit and never fall back.

## Domain model

| Term | Meaning |
|---|---|
| Source Map | Immutable saved `OccupancyGrid`; identity is a SHA-256 hash of normalized metadata and raw cell data. Any change in hash denotes a distinct map. |
| Cleanable Space | Known-free, cleaning-allowed space within the Source Map (`free_mask & ~keepout_mask`). |
| Region | Named portion of Cleanable Space, authoritatively represented as a 1-bit raster mask (naturally supporting holes and disjoint components). Geometric outlines are derived on-demand for visualization and export. |
| Candidate Region | A Region produced by automatic segmentation or currently under editing, not yet published. |
| Region Set | Versioned set of Regions and spatial constraints belonging to one Source Map. |
| Published Region Set | A Region Set that has passed validation checks and is frozen for generating Jobs. |
| Keepout / Virtual Wall | Spatial constraints persisted independently that do not alter the Source Map. A Virtual Wall is a line-shaped Keepout with explicit physical width, and acts as a **sealing barrier**: together with physical walls it can enclose an area, making it unreachable from the dock — the enclosed area is the effective keep-out zone. |
| Enclosed Area | Cleanable cells outside the dock-reachable connected component, created when Virtual Wall bands (8-connected rasters) together with physical walls seal an opening. Not an explicit constraint; never written into the Nav2 keepout mask. |
| Detected Wall | A physical wall segment recognized by the segmentation provider (map-frame endpoints, support score, orientation). Reproducible algorithm output; never persisted directly and distinct from a Virtual Wall. |
| Spot Area | A positive transient target polygon in the map frame (e.g. for custom small-area / spot cleaning). Retained at the constraint layer as the last-used spot area without mutating persistent room partitions. |
| Cleaning Target | Configured cleaning task target holding a sequence of target region labels and an associated runtime RegionSet view for query by downstream job orchestration. |
| Segment | Unit of a Job target handled by one cleaning strategy; not necessarily identical to a Region. |
| Coverage artifact | Verifiable spatial record of covered space (e.g. a coverage grid); a bare percentage is not an artifact. |

## Phase 1 implementation details

### 1. Map identity and change detection

Map identity is computed as:
`SHA-256(float32(resolution) + int32(width) + int32(height) + float64(origin.x, origin.y, origin.yaw) + raw int8 cell data)`

Key rules:
- Excludes transient fields (`header.stamp`, `frame_id`, `map_load_time`).
- `resolution` is normalized to `float32` so that `OccupancyGrid.info.resolution` (float32) and `map.yaml` (float64) yield identical hashes for the same physical map.
- The hash is a change detector and persistence storage key, not a multi-map coordinator. A hash change indicates a new map; Region Sets are not silently migrated or reprojected across map hash changes.

### 2. Map loading fidelity convention (Nav2 Jazzy alignment)

The algorithm-neutral `oomwoo_segmentation.map_io.load_map_file` strictly matches Nav2 `map_io.cpp` trinary loading:
- `occ = 1.0 - color / 255.0` (with `negate: 0`).
- `occ >= occupied_thresh` (default 0.65) → `OCCUPIED` (100).
- `occ <= free_thresh` (default 0.196) → `FREE` (0).
- Otherwise → `UNKNOWN` (-1).
- Alpha channel `< 255` is always `UNKNOWN`.
- Image top row corresponds to maximum map Y, so the raw pixel array is vertically flipped on load.

### 3. Automatic segmentation and ROSE2-based engine

The shared ROS 2 action is `oomwoo_segmentation_msgs/action/SegmentRooms`.
- **Request**: Immutable `nav_msgs/OccupancyGrid`, optional `sensor_msgs/Image` mono8 cleanable mask, optional diagnostics flag.
- **Result**: Status code, implementation version, `sensor_msgs/Image` (32SC1) label grid in OccupancyGrid row order, `Room[]` metadata (id, centroid, area, boundary polygon), and `WallSegment[]` of Detected Walls.
- **Contract rules**: Label 0 is unassigned; positive labels (1..N) are contiguous and deterministic. Occupied, unknown, and excluded cells must remain label 0. Invalid inputs and algorithm errors produce explicit error statuses (`STATUS_INVALID_REQUEST`, `STATUS_ALGORITHM_ERROR`, `STATUS_CANCELLED`).

The segmentation engine in `oomwoo_segmentation.engine` is a pure in-memory port derived from `aislabunimi/ROSE2` commit `3a010b9e6bb2477de3b5b46208ebfccd71dfafbf`:
1. **ROSE FFT structural filtering**: Identifies principal structural directions and produces cleaned occupancy images.
2. **ROSE2 Hough wall extraction & clustering**: Extracts line segments, performs angular and spatial clustering, and computes extended lines.
3. **Planar cell & edge topology**: Builds geometric cells and topological edges from line intersections.
4. **Hard wall barrier & DBSCAN clustering**: Builds cell affinity matrix. An explicit hard wall barrier (`hard_wall_threshold=0.40`) prevents merging topological cells across confirmed physical walls.
5. **Constrained geodesic coverage**: After polygon rasterization, `_geodesic_coverage` propagates room labels across unassigned cleanable free cells via wavefront expansion constrained by physical walls, ensuring 100% cleanable free cell coverage with 0 unassigned cells while filtering sub-10px noise artifacts.
6. **Detected Wall extraction**: Map-frame Detected Walls are converted directly from retained merged extended segments (filtered by `lines_threshold=0.22`), clipped to map bounds, and exposed with support score and orientation.

### 4. Region representation and editing semantics

- **Internal representation**: Authoritative Region representation is a 1-bit boolean raster mask; geometric outlines are derived via `cv2.findContours`.
- **Immediate stroke clipping**: User paint strokes are immediately clipped as `intent ∩ Cleanable Space` (`free_mask & ~keepout_mask`). If the intersection is empty, the edit is rejected.
- **Later-painter preemption**: When a stroke overlaps existing Regions, overlapping cells are deducted from earlier Regions and assigned to the new Region. Emptied Regions are automatically pruned.
- **Constraint clipping**: Applying or updating Keepouts/Virtual Walls immediately clips intersecting cells from all existing Regions via `RegionSet.apply_keepout_mask()`. Removing constraints restores cleanable space but does not revive previously clipped Region cells.

### 5. Validation grading (publish-time checks)

Validation grades errors (blocking publication) vs warnings (informative):

| Severity | Condition | Rationale |
| --- | --- | --- |
| **Error** | Regions overlap | Partition invariant violation |
| **Error** | Region contains occupied or unknown cells | Cannot clean non-free cells |
| **Error** | Region cannot be reached by robot footprint from any navigable position | Robot cannot reach or sweep the region from any valid pose |
| **Error** | Region has cells inside the enclosed area (`region_enclosed`) | Virtual Walls sealed part of the Region away from the dock; cells are preserved, never silently clipped |
| **Error** | Dock staging seed is off-grid, occupied, unknown, or constrained (`dock_unreachable`) | Robot could not leave the dock under these constraints |
| **Error** | Region intersects a Keepout or Virtual Wall | Safety constraint violation |
| **Error** | Region Set has 0 Regions | Empty job target |
| **Warning** | Unassigned cleanable free space exists | Informs user of uncovered cleanable space |
| **Warning** | Region core is split into disconnected components by narrow throats | Informs user of potential intra-region traversal bottlenecks |
| **Warning** | Virtual Walls do not change the dock-reachable component (`virtual_wall_seals_nothing`) | Possible gaps or unclosed wall chains |

Reachability semantics:
- **Dock-seeded (when a seed pose is available)**: a 4-connected flood fill from the dock staging cell over cleanable space yields the main dock-reachable component; navigable centers are restricted to it. The seed is the undock exit pose (see `dock.py`), read from the Nav2 `opennav_docking` dock database — undock success itself proves the seed is reachable, so no navigable-center assumption is needed for it.
- **Global fallback (no dock database / no seed)**: any navigable position in cleanable space seeds reachability, matching the pre-enclosure semantics.
- Virtual Wall bands are rasterized 8-connected so a closed wall chain provably blocks the 4-connected flood fill (grid Jordan curve property); free space is flood-filled 4-connected.
- Standard regions with their own footprint-reachable core (`erode(mask, radius) != empty`) are verified for core connectivity (warning if split into multiple pieces).
- Regions smaller or narrower than the robot footprint (e.g. spot cleaning areas or small free patches) are explicitly **allowed** as long as they can be swept by the robot footprint from adjacent navigable space (`distance_to_navigable_centers <= robot_inscribed_radius`).
- Only regions located in completely unreachable cavities/dead-ends (where no navigable robot center position can ever sweep them) are rejected with `region_unreachable`.

Robot footprint radius defaults to `robot_inscribed_radius = 0.17 m`. The deprecated "unreachable cell ratio" metric is not used (as perimeter boundary cells naturally cannot be reached by the robot center).

### 6. Persistence model

Data is stored under `~/.local/share/oomwoo_cleaning_jobs/maps/<map_hash>/`:
- `map_snapshot.yaml`, `map_snapshot.pgm`: Visual provenance snapshot.
- `map_snapshot.cells.npy`: Lossless raw int8 cell array sidecar.
- `draft/`: `regions.yaml` (metadata) + `masks/*.png` (1-bit PNGs) + `constraints.yaml` (Keepout/VirtualWall/SpotArea geometry) + `keepout_mask.pgm`/`.yaml` (Nav2 trinary mask, preview only).
- `published/`: Same structure as draft; atomically switched via symlink pointers. Only the published generation's `keepout_mask` is served to Nav2.
- At most one Published Region Set exists per map at any time.

### 7. Spatial constraints and Spot Area persistence

- **ConstraintSet**: Holds negative spatial constraints (`Keepout` polygons, `VirtualWall` center-lines dilated by physical width) and an optional positive transient target (`SpotArea` polygon).
- **Keepouts / Virtual Walls**: Deducted from cleanable space (`free_mask & ~keepout_mask`). Adding/updating constraints immediately clips existing Region cells.
- **Keepout margin**: `ConstraintSet.mask_for(source_map, keepout_margin_m)` optionally dilates Keepout rasters by a circular metric margin (default 0 = exact cell-center rasterization) for deployments requiring the whole robot footprint — not just its center — to stay out of a Keepout. Virtual Wall bands never take the margin.
- **Spot Area**: Stored at the same spatial constraint layer (`constraints.yaml`), retaining the single last-used spot area across sessions without modifying persistent room partitions.

### 8. Nav2 keepout filter projection

Constraints take effect in navigation only at publish time (drafts are preview-only):

- **Mechanism**: standard Nav2 `nav2_costmap_2d::KeepoutFilter` in the global and local costmaps (see `src/oomwoo_cleaning_jobs/config/nav2_keepout.yaml`). The filter plugin must be listed **after** `inflation_layer` in the `plugins` list: filters apply in plugin-list order, and placed last their lethal mask cells are not expanded by inflation. `override_lethal_cost: true` lets a robot that somehow stands on a keepout cell navigate out instead of deadlocking.
- **Mask content**: only explicit constraints (Keepout polygons, possibly with `keepout_margin_m`, plus Virtual Wall bands). The enclosed area behind sealing walls is deliberately **not** marked: it is unreachable by navigation topology, so goals there fail planning naturally, and a robot inside it never stands on a lethal cell.
- **Materialization**: `RegionSetStore` writes `keepout_mask.pgm`/`.yaml` (Nav2 trinary convention, Source Map geometry) in the same atomic generation as `constraints.yaml`, so geometry source and mask can never diverge.
- **Serving**: the `constraint_mask_publisher` node publishes latched (transient local + reliable) `/costmap_filter_info` (type 0 = keepout, naming the mask topic) **before** `/keepout_filter_mask` (`nav_msgs/OccupancyGrid`, 100 = constraint cell), reloads via `std_srvs/Trigger` `reload_keepout_mask`, and treats a missing published mask at startup as an explicit degraded state (ERROR log, nothing published).
- **Publish transaction**: `RegionSetStore.publish(..., seed_pose=..., post_publish_hook=...)` validates (dock-seeded when a seed is given), atomically switches the published generation, then invokes the hook (reload); a hook failure rolls the published pointer back and raises `PublishError` — disk and Nav2 never diverge silently.
- **Dock pose**: read-only from the Nav2 `opennav_docking` dock database (`dock.load_dock_pose`); the undock exit pose (`dock.staging_pose`) seeds reachability validation. Without a dock database the global fallback applies.

Public interface contract (to be registered in upstream `docs/SOFTWARE_INTERFACES.md` when contributed):

| Interface | Type | Producer | Consumer | QoS | Failure behavior |
| --- | --- | --- | --- | --- | --- |
| `/costmap_filter_info` | `nav2_msgs/msg/CostmapFilterInfo` (`type: 0` = keepout, names the mask topic) | `constraint_mask_publisher` | Nav2 global/local costmap `KeepoutFilter` | Transient local + reliable (latched); always published before the mask | Absent at startup: filters run without constraints (degraded; publisher logs ERROR). Mid-reload: filters may briefly keep the previous mask (acceptable window). |
| `/keepout_filter_mask` | `nav_msgs/msg/OccupancyGrid` (100 = constraint cell, Source Map geometry) | `constraint_mask_publisher` | Nav2 global/local costmap `KeepoutFilter` | Transient local + reliable (latched) | Same as above; mask and info are republished together on every reload. |
| `reload_keepout_mask` | `std_srvs/srv/Trigger` | `constraint_mask_publisher` (server) | Publish transaction (`post_publish_hook`), operators | Service | `success: false` → publish rolls the published pointer back and raises `PublishError`; Nav2 keeps the previous mask, disk and Nav2 stay consistent. |

Known limitations: the upstream dock-cycle module is still RFC-stage, so no dock database exists yet and the fallback path is the only one exercised in current environments; the reload rollback covers the file layer, and a mid-reload Nav2 may briefly serve the previous mask (acceptable window).

### 9. Cleaning target configuration (whole-map, selected regions, spot cleaning)

The cleaning mode configuration layer in `oomwoo_cleaning_jobs_core.targets` bridges user intent to downstream task orchestration:
- **Unified interface**: All modes produce a `CleaningTarget` containing `target_labels: tuple[int, ...]` and a runtime `RegionSet` view. Downstream modules query `target.mask_of(label)` or `target.outline(label)` uniformly without branching on mode type.
- **Whole-map (`configure_whole_map`)**: Targets all published/active regions `[1..N]` in the `RegionSet`.
- **Selected regions (`configure_selected_regions`)**: Targets specific region labels `[lbl_1, lbl_2, ...]`, validating existence and preserving the requested execution order.
- **Spot cleaning (`configure_spot_area` / `configure_last_spot_area`)**:
  - Constructs a transient, isolated `RegionSet` (e.g. single region with `label=1`).
  - The spot polygon is clipped strictly against `free_mask & ~keepout_mask` (free space minus Keepouts).
  - Can cross existing room boundaries as a single continuous/disjoint area.
  - Validated against the robot inscribed radius (`validate_region_set`), ensuring the robot center can enter and traverse it.
  - Updates `ConstraintSet.spot_area` so the last-used spot area is persisted.
  - Leaves persistent published/draft room partitions completely untouched.

## Test strategy, fixtures, and baselines

### Test map fixtures

All input test maps reside in `src/oomwoo_segmentation/test/maps/`:
- `demo/`: 6 standard benchmark scenarios (`corridor4`, `grid6_furniture`, `living_room`, `room3`, `room4`, `two_rooms`). `.render.png` files are integer-upscaled display maps downsampled during load by their exact scale factor.
- `rose2_upstream/`: 20 runnable test maps from `aislabunimi/ROSE2` upstream (`Freiburg_Building_079`, `ViMantic_House`, `carmen`, `movecare`, `simona-house`).
- `ipa/`: 8 test maps from `ipa320/ipa_coverage_planning` (`lab_a`, `lab_ipa`, `office_a`, `office_h`, `NLB`, `office_a_furnitures`, `lab_d_scan_furnitures`, and `Freiburg52_scan`).

### Output artifacts

Test runs, batch executions, and rendering tools write outputs exclusively under the repository root `output/`:
- `output/demo/`: Outputs for demo benchmark maps.
- `output/rose2_upstream/`: Outputs for upstream maps.
- `output/ipa/`: Outputs for IPA benchmark maps.
- `output/README.md` and `output/summary.png`: Demo regression summary report and visual mosaic.

### Demo verification baseline (100% cleanable coverage)

All 6 demo maps achieve 100% cleanable free space coverage (0 unassigned cleanable cells) with hard wall barrier separation:

| Scenario | Rooms | Detected Walls | Unassigned Cells | Description / Expected Structure |
| --- | ---: | ---: | ---: | --- |
| `corridor4` | 5 | 7 | 0 (0.0%) | 4 individual rooms + 1 continuous corridor |
| `grid6_furniture` | 6 | 6 | 0 (0.0%) | 6 rooms separated by physical walls; furniture does not create spurious walls |
| `living_room` | 1 | 5 | 0 (0.0%) | Single open living area fully covered; no furniture over-segmentation |
| `room3` | 6 | 10 | 0 (0.0%) | 6 distinct rooms cleanly partitioned by physical walls |
| `room4` | 5 | 10 | 0 (0.0%) | 5 rooms partitioned along walls; full geodesic coverage without internal holes |
| `two_rooms` | 2 | 8 | 0 (0.0%) | 2 rooms separated by doorway and center wall |

### Upstream & IPA verification baseline

- **ROSE2 upstream suite**: 20/21 maps execute successfully without errors or label boundary violations, exposing 6–19 Detected Walls per map. (`Virtual/mapirlab.yaml` was never committed with its PGM upstream and is skipped).
- **IPA suite**: 7/7 standard maps segment into valid rooms covering 100% of cleanable space. (`Freiburg52_scan` fails in upstream ROSE FFT itself due to lack of walls and serves as a batch runner error-handling test).
- **Test suite**: 91+ unit and regression tests pass cleanly under `pytest` / `colcon test`.

## Subsequent Job execution & Phase 2 direction

`oomwoo_cleaning_jobs` will orchestrate Job lifecycle, tasking, pause/resume, and checkpoint recovery:
1. **Vertical slice validation**: Prioritize `Published Region → target/allowed-clean mask adaptation → oomwoo_coverage → coverage estimator/grid → Job checkpoint`.
2. **Coverage progress tracking**: Saved-map execution relies on an external or localization-based coverage estimator providing `covered_grid` and `coverage_ratio`.
3. **Checkpoints & Recovery**: Checkpoints store current Segment, accumulated coverage artifact, and pending queue. Recovery with a trusted coverage artifact resumes remaining uncovered cells; without an artifact, it restarts the current Segment.

## Open boundaries

1. **Target mask interface**: Format and QoS for passing target Region masks into `oomwoo_coverage`.
2. **Segment completion criteria**: Combination of path execution completion and verified target cell coverage.
3. **Coverage estimator specification**: Grid resolution, confidence bounds, update frequency, and restart recovery.
4. **Checkpoint serialization**: Schema and storage strategy for atomic Job state persistence across robot reboots.
5. **Segment partitioning**: Splitting large Regions into Segments based on area, battery budget, or cleaning strategy.
6. **Floor-care coordination**: Sequencing perimeter edge cleaning with interior coverage sweeping.
7. **Job action interface**: Action definitions for `RunCleaningJob`, pause, resume, cancel, and status feedback.
8. **Map change policy**: Re-validation and user notification workflow when a map hash change is detected.
