# oomwoo_cleaning_jobs development context

This is the single development context for this repository. It records the current scope, decisions made, terminology, and open boundaries; read it in full before changing implementation or design. The user's immediate instructions and verified source-code facts take precedence over this document and must be written back into it afterwards.

## Goal and scope

`oomwoo_cleaning_jobs` owns **user cleaning intent and long-running job orchestration on saved maps**: whole-map, selected-Region, and spot cleaning; Region/virtual wall/keepout persistence; Job state, pause, resume, retry, and summary.

It does not own coverage path planning or base execution algorithms. Regular saved-map coverage reuses `oomwoo_coverage`; Nav2 is the motion execution layer. `clean-and-map` is only an RFC/algorithm reference for first-clean (SLAM, exploration, and coverage), not the backend or coverage-progress provider for saved-map Jobs. `floor-care` is a future perimeter/edge pass; it can be combined with `oomwoo_coverage`'s interior sweep but is not a general coverage backend.

External sources of truth:

- [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs)
- [clean-and-map RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/clean-and-map)
- [floor-care RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/floor-care)
- [oomwoo-ros2-tools](https://github.com/makerspet/oomwoo-ros2-tools)
- [SOFTWARE_INTERFACES.md](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

## Current phase

Phase 1 delivers only:

`saved map → automatic candidate regions → manual editing → validation → Published Region Set persistence`

Input is `nav_msgs/msg/OccupancyGrid`, obtained via two paths: the core library directly parses nav2 trinary `map.yaml + image` (for tests and the CLI); the GUI subscribes to `/map` at runtime (transient_local QoS) or opens a map file. Known-free cells are cleanable; unknown and occupied are not.

Phase 1 does not drive the robot, does not execute full Jobs, and does not freeze executor action or feedback definitions. The verification GUI is `oomwoo_cleaning_jobs_ui` (a standalone PyQt5 application under `src/` in this repository), not the final control app.

## Domain model

| Term | Meaning |
|---|---|
| Source Map | Immutable saved OccupancyGrid; identity is derived from a content hash of metadata and cell data — a hash change means a new map. |
| Cleanable Space | Known-free, cleaning-allowed space within the Source Map. |
| Region | Named part of the Cleanable Space, with a raster mask as the authoritative representation (supports holes and disjoint components); geometric outlines are derived from the mask and used only for GUI and export. |
| Candidate Region | A Region produced by automatic partitioning or under editing, not yet reviewed. |
| Region Set | Versioned set of Regions and spatial constraints belonging to one Source Map. |
| Published Region Set | A Region Set that has passed validation and can be used to generate Jobs. |
| Keepout / Virtual Wall | Independently persisted constraints that do not modify the Source Map; a Virtual Wall is a line-shaped Keepout. |
| Detected Wall | A wall segment recognized by the segmentation provider (map-frame endpoints, support, direction). Pure algorithm output: reproducible from the Source Map, never persisted, and distinct from a Virtual Wall — converting one into a Virtual Wall requires explicit user confirmation. |
| Segment | Part of a Job's target handled by one cleaning strategy; not necessarily equal to a Region. |
| Coverage artifact | Verifiable record of covered space (e.g. a coverage grid); a percentage by itself is not an artifact. |

Automatic partitioning is a replaceable ROS 2 module behind `oomwoo_segmentation_interfaces/SegmentRooms`. This branch runs only the `oomwoo_rose2` provider; failures are explicit and never fall back to a legacy algorithm. The authoritative result is a source-grid `int32` label mask where 0 is unassigned and positive labels are Candidate Regions.

Manual editing supports at least create, move/delete/rename, merge/split of Regions, and creation of Keepout/Virtual Wall. Immediate clipping during editing and validation grading at publish time are described in "Phase 1 implementation decisions". Unassigned or deliberately uncleaned free space is allowed, but the GUI must present it clearly.

The GUI main flow is now: **automatic segmentation → name candidate Regions one by one → save/validate and publish**. Immediately after candidates are generated, naming dialogs pop up in label order; canceling can be resumed later from "Name candidates one by one". Create, paint, erase, merge, split, and constraint editing live in the collapsed-by-default "Advanced editing (use only when candidates are wrong)" section, still serving as the correction mechanism for automatic segmentation errors.

## Phase 1 implementation decisions

The following decisions were confirmed point by point with the user on 2026-08-22 (grilling discussion); re-confirm with the user before changing them.

### Package structure

- `src/oomwoo_segmentation_interfaces`: algorithm-neutral ROS 2 messages and `SegmentRooms` action (result carries room labels plus Detected Walls). The action is the seam implemented by every provider.
- `src/oomwoo_segmentation`: Apache-2.0 algorithm-neutral map model/I/O, canonical result validation, action client, and rendering/CLI. It contains no segmentation implementation.
- `src/oomwoo_rose2`: GPLv3 ROS 2 port of pinned upstream ROSE + ROSE2; it is the only production provider in this branch.
- `src/oomwoo_cleaning_jobs_core`: pure Python Region Set editing, constraints, validation, and persistence.
- `src/oomwoo_cleaning_jobs_ui`: standalone PyQt5 application + rclpy adaptation layer; segmentation runs off the Qt main thread.

### Map identity and change detection

identity = SHA-256(**float32** bytes of `resolution` + `width` + `height` + `origin` position/orientation + raw int8 cell data), excluding `header.stamp`, `frame_id`, `map_load_time`, with no trinarization. The short id is the first 12 hex digits. Resolution is normalized to float32: `OccupancyGrid.info.resolution` is float32 while map.yaml is float64 — without normalization the same map loaded via topic and via file would get different identities (fixed during GUI dual-source verification).

The hash acts as a **change detector and storage key**, not as multi-map management. A saved map is treated as a static artifact; the only source of change is the user re-mapping/re-saving. A hash change means a new map: when no matching Region Set is found, the GUI clearly states "no region set for the current map; N region sets belonging to other maps exist on disk". Phase 1 does not migrate/reproject region sets (origin and resolution may both change, making pixel-level migration unreliable); see open boundary 9.

### Map file loading fidelity convention (verified against nav2 jazzy map_io.cpp)

Trinary loading: `occ = 1 - color/255` (negate=0); `occ >= occupied_thresh` → 100, `occ <= free_thresh` → 0, otherwise -1; `alpha < 255` is always unknown; the image's top row corresponds to the map's maximum y, so the map is flipped vertically after loading. map_saver always writes pixels 0 (occupied)/254 (free)/205 (unknown) with `occupied_thresh: 0.65, free_thresh: 0.196` (205 thereby reads back as unknown). The algorithm-neutral `oomwoo_segmentation.map_io.load_map_file` matches this behavior and supports trinary only.

Fixed the external `oomwoo_sim_support/maps/test_room` asset chain: the PGM correctly represents outside-wall unknown as 205, but the YAML previously used `free_thresh: 0.25`, which misread 205 (occ≈0.196) as free and produced candidate regions outside the walls. The generation script and the sim/deploy YAMLs are now unified at `free_thresh: 0.196`.

### Automatic segmentation

The shared interface is `oomwoo_segmentation_interfaces/action/SegmentRooms`. A request contains the immutable `OccupancyGrid`, an optional same-shape Cleanable Space mask, and an optional diagnostics flag. A successful result contains the provider id/version, an `int32` label grid in OccupancyGrid row order, room metadata derived from that grid, and a `WallSegment[]` of Detected Walls. Label 0 is unassigned; positive labels are deterministic and contiguous. Implementations must never label occupied, unknown, or excluded cells. Invalid requests and algorithm failures are explicit action results; no provider fallback is permitted. Detected Walls are derived, non-authoritative data (the authoritative result remains the label grid); providers that do not detect walls return an empty array. Wall validation requires finite endpoints within one cell of the map bounds, support in [0, 1], and direction in [0, pi).

`oomwoo_segmentation` is the algorithm-neutral deep module used by callers. It owns Source Map identity and map-file loading, canonicalization/validation, ROS message conversion, the action client, stable-color rendering, and `oomwoo-render-map`. Its rendering depends only on the shared result, so every future provider receives identical final visualization.

This branch has one production adapter: `oomwoo_rose2`, derived from `aislabunimi/ROSE2` commit `3a010b9e6bb2477de3b5b46208ebfccd71dfafbf`. It preserves the upstream two-stage computation: occupancy image → ROSE FFT structural filtering and dominant directions → ROSE2 Hough walls, angular/spatial clustering, extended lines, edges, cells, affinity matrix/DBSCAN → Shapely room polygons → canonical source-grid labels. The ROS1 nodes, services, RViz dependencies, and pickled Python-object messages are replaced by the typed ROS 2 action. Source provenance, GPLv3, and compatibility changes are recorded in `src/oomwoo_rose2/THIRD_PARTY.md`.

The default parameter profile is based on the pinned upstream `ROSE.launch`: `filter_level=0.18`, spatial threshold 5, retained-line threshold 0.22, retained-edge threshold 0, line merge distance 20 px, and `rooms_voronoi=false`. The 0.22 line-support floor is an OOMWOO compatibility profile change that rejects the demonstrated furniture-derived full-map line; it remains configurable. Zero-weight retained edges are checked against the ROSE structural raster because probabilistic Hough extraction can omit a local wall run. A strict frame-fringe rule excludes only small frame-adjacent cells behind high-support, layout-spanning walls. The optional Voronoi path remains available through provider parameters. Keepouts are presented to ROSE as occupied cells and the canonical result is clipped again afterwards.

The removed maximin/watershed, merge-tree/saddle, doorway clipping/topology, and skeleton doorway-demo implementations have no source, runtime switch, fallback, parameter, or test path in this branch. The shared result deliberately has no legacy `Doorway` or `low_confidence` fields. Future doorway/topology work must derive from ROSE2 typed edges/lines/polygons as a separate capability, building on the Detected Wall output described below.

**Wall recognition (decided 2026-08-24)**: detected walls are exposed through the existing seam, not a new package or a separate action — the walls are intermediate artifacts of the same pipeline run, so `SegmentRooms` carries them to avoid double computation and inconsistent results. `oomwoo_segmentation_interfaces` defines `WallSegment.msg` (map-frame endpoints, support, direction); `oomwoo_segmentation` owns the algorithm-neutral model, ROS conversion, validation, and support-colored rendering (`render_walls`); `oomwoo_rose2` converts the retained merged extended segments (`extended_segments_th1_merged`, already filtered by `lines_threshold`) into map-frame walls, clipping the upstream `offset`-padded bounding-box endpoints to the map rect. `oomwoo_cleaning_jobs_core` provides `VirtualWall.from_detected_wall`; the GUI display and one-click conversion interaction are deliberately out of scope for now. Pipeline pixel coordinates use cell-center convention `(col + 0.5, row + 0.5) * resolution` via `SourceMap.map_frame_from_pixel`/`pixel_from_map_frame`, honoring origin yaw.

Rendering: `ros2 run oomwoo_segmentation oomwoo-render-map MAP.yaml --segment` writes the base map and a stable labeled overlay through the action server. `--diagnostics-dir` requests provider-specific ROSE2 images for the cleaned map, extended lines, and final overlay.

### Region representation and editing semantics

A Region is internally represented as a bitmask, naturally supporting holes and disjoint components; outlines are derived via `cv2.findContours`. Editing is brush-style: brush adds/removes cells, circle/line drawing splits; merge is an explicit menu operation and does not rely on painting an overlap first.

**Immediate clipping during editing**: the user paints intent; the system stores `intent ∩ Cleanable Space` (known-free and inside no Keepout). Clipping happens on stroke and the true result is displayed (WYSIWYG); if the clip is empty, the edit is rejected with a prompt. A stroke overlapping an existing Region triggers **later-painter preemption**: overlapping cells are deducted from the old Region and given to the new one; the GUI must prominently indicate that the old Region shrank. Only the clipped mask is stored, never the raw stroke. Unreachable furniture inside a user's Region (clipped away or footprint-unreachable) is **normal behavior**, not an error.

### Validation grading (at publish time)

In phase 1 the robot footprint comes from the parameter `robot_inscribed_radius` (default 0.17 m); phase 2 will parse the footprint profile from Nav2.

Errors (block publishing): Regions overlap; a Region contains occupied/unknown cells; a Region mask is empty after erosion by the footprint radius (the robot center cannot stay inside); a Region intersects a Keepout. Under normal editing paths these errors are guaranteed impossible by immediate clipping and preemption rules — in publish validation they are **system invariant checks** (against hand-edited files and bugs) that normal users can never trigger.

Warnings (allow publishing, GUI must present prominently): unassigned cleanable free space exists; a Region's footprint-reachable core is split into multiple components by narrow throats (the robot cannot traverse the Region). **The "unreachable cell ratio" metric is not used** (found during implementation: the perimeter ring of any room is unreachable to the robot center, ~30%, guaranteeing false positives); erosion applies to the Region mask itself, not the whole cleanable space.

### Persistence

Root directory `~/.local/share/oomwoo_cleaning_jobs/maps/<map_hash>/`, containing:

- `map_snapshot.{yaml,pgm}`: map snapshot (for provenance).
- `draft/`: `regions.yaml` (Region metadata) + `masks/*.png` (1-bit masks, inspectable with an image viewer) + `constraints.yaml` (Keepout/Virtual Wall geometry).
- `published/`: same structure. At most one Published Region Set per map at any time; publishing = copy draft after validation passes, recording version number and timestamp.

Implemented: `persistence.RegionSetStore` keys directories by the full Source Map identity, writes a `map_snapshot.{yaml,pgm}` preview plus a lossless `map_snapshot.cells.npy` raw-cell sidecar on first save; draft and published point to immutable generation directories switched via atomic symlink pointers. `publish()` validates against the current Keepouts first, then saves the draft, switches published, and increments the version / records UTC time and footprint radius. Loading validates schema, identity, PNG mask shapes, and mask overlap; Published sets are re-validated against the saved footprint radius, while drafts may retain publish-validation errors for GUI display.

### Keepout / Virtual Wall

Included in phase 1. Keepouts are deducted from the Cleanable Space; a Virtual Wall is a line-shaped Keepout, handled by dilating the line into a polygon. Constraints share the persistence and validation pipeline with Regions.

Core model implemented: `constraints.ConstraintSet` holds `Keepout` (map-frame polygons) and `VirtualWall` (center lines with explicit `width_m`), rasterized according to the Source Map's origin/yaw. The UI sends `source.free_mask() & ~constraints.mask_for(source)` as the shared action's Cleanable Space mask; if a Region Set is initialized from those candidates, the raw `source.free_mask()` and current constraint mask are passed separately to `RegionSet.from_segmentation(..., base_cleanable=..., keepout_mask=...)`. After constraints change, `RegionSet.apply_keepout_mask()` immediately clips existing Region cells. Removing a constraint only restores cleanable space; it does not revive clipped Region cells.

### Test strategy

Headless tests are split by seam: `oomwoo_segmentation` tests contract canonicalization, map fidelity, ROS conversions, and rendering; `oomwoo_rose2` tests parameter/raster adapters plus a slow full upstream pipeline smoke test; cleaning-jobs tests inject a test-only provider so Region editing tests do not depend on ROS or an algorithm. The upstream baseline is pinned for reproducibility; exact stage-by-stage golden parity remains a future strengthening beyond the current contract and real-pipeline smoke coverage. The GUI retains a manual acceptance checklist.

### Test map fixtures and verification outputs

All test input images/maps live in exactly one place: `src/oomwoo_rose2/test/maps/`.

- `demo/`: local demo maps referenced by `test_docs_maps.py`. `*.render.png` files are upscaled-for-display images that must be downsampled by their exact integer block factor before use (see the `embedded_scale` table in `output/README.md`); plain `.png` maps are used at native resolution.
- `rose2_upstream/`: verbatim copy of the pinned upstream `aislabunimi/ROSE2` `src/maps/` test maps (carmen, Freiburg_Building_079 map2–map15, Virtual ViMantic_House20/23/30, maps-nostre movecare/simona-house). Upstream `Virtual/mapirlab.yaml` has no committed image and is not runnable.

Every test, verification, or demo run writes its artifacts only under the repository-root `output/` directory, one subdirectory per map (`output/<map>/source.render.png`, `segments.png`, `walls.png`, `diagnostics/`, `run.txt`); `output/README.md` + `output/summary.png` hold the demo-map regression report, `output/rose2_upstream/` holds upstream-map verification runs. Derived images must never be stored next to the inputs in `test/maps/` — regenerate them into `output/` instead. `src/oomwoo_rose2/test/run_map_batch.py` is the batch runner: point it at a maps directory, individual `map.yaml` files, or demo render images (with `--embedded-scale` to restore the exact integer block factor) and it writes the standard per-map output layout plus a `summary.txt` (failures are recorded per map, never abort the batch).

### Closeout verification baseline

The closeout baseline is `rose2 upstream-3a010b9e6bb2+oomwoo.4`. The six local demo maps are fixed structural regressions; without human label masks these counts and visual boundaries are acceptance evidence, not IoU claims:

| Map | Rooms | Detected Walls | Unassigned cleanable cells | Expected interpretation |
| --- | ---: | ---: | ---: | --- |
| `corridor4` | 5 | 7 | 0 | Four rooms plus the corridor. |
| `grid6_furniture` | 6 | 6 | 0 | Six rooms; furniture does not become a full-map wall. |
| `living_room` | 1 | 5 | 449 (5.8%) | One interior room; free noise beyond the high-support exterior wall remains unassigned. |
| `room3` | 5 | 10 | 0 | Five visually consistent regions. |
| `room4` | 4 | 10 | 180 (2.1%) | Four regions, with one unresolved 15×12 free-space coverage hole at source-grid columns 62–76 and rows 99–110. |
| `two_rooms` | 2 | 8 | 0 | Two rooms and no tiny spurious region. |

Closeout verification passed with 96 full-source pytest cases, including 16 ROSE2 provider cases; the joint five-package `colcon test` reports 81 tests, 0 errors, 0 failures, and 1 dependency-gated skip.

Orange (`COLOR_UNASSIGNED`, BGR `(0, 165, 255)`) in the provider-neutral segmentation rendering means a source cell is cleanable/free but retains canonical label 0. It is neither occupied nor unknown. This is deliberate for the `living_room` exterior fringe and is a publish-time warning under the shared validation policy. In `room4`, the same color exposes a remaining polygon/cell coverage hole; disabling frame-fringe rejection does not remove it, so it must not be described as exterior filtering. Keep this limitation visible in `output/README.md` and the GUI rather than silently merging it into an adjacent room.

Closeout still requires the real-robot GUI checklist in `src/oomwoo_cleaning_jobs_ui/docs/MANUAL_ACCEPTANCE.md`, GPLv3 distribution confirmation for `oomwoo_rose2`, and an explicit decision on whether the `room4` coverage hole is acceptable as a warning or must be fixed before release. Human ground-truth masks are required before reporting IoU/ARI or claiming quantitative segmentation quality.

### Implementation status

Implemented packages: `oomwoo_segmentation_interfaces` provides typed ROS 2 messages and `SegmentRooms`; `oomwoo_segmentation` contains SourceMap identity/masks, Nav2 trinary loading, canonical result validation (labels plus Detected Walls), action client, and rendering/CLI; `oomwoo_rose2` contains the GPLv3 pinned upstream port, wall extraction from retained extended segments, ROS 2 action server, launch/config, and optional diagnostics. `oomwoo_cleaning_jobs_core` contains `RegionSet` editing, constraints, validation, and persistence only. `oomwoo_cleaning_jobs_ui` subscribes to `/map`, requests segmentation through the shared action in a worker thread, converts labels to editable Regions, and retains file loading, naming, Keepout/Virtual Wall editing, draft, and publish flows. The GUI manual acceptance checklist is at `src/oomwoo_cleaning_jobs_ui/docs/MANUAL_ACCEPTANCE.md`.


## Coverage execution facts and integration direction

Existing `oomwoo_coverage`: reads the full `/map` and an optional `keepout_filter_mask`, selects the reachable connected component from the robot's position, performs boustrophedon cell decomposition, sweeping, and gap-fill, and moves via Nav2. It has no public Region, Segment, or target-mask input; it currently covers the entire reachable area and cannot clean only a specified Region.

It subscribes to external `coverage_ratio` and `covered_grid` and does not estimate coverage itself. In simulation both can come from a coverage meter; a real robot needs an estimator based on localization and cleaning width. This estimator is an independent dependency of Job progress, completion judgment, and precise recovery.

Phase 2 prioritizes validating a minimal vertical slice:

`Published Region → target/allowed-clean mask adaptation → oomwoo_coverage → coverage estimator/grid → Job checkpoint`

Do not freeze a generalized coverage-backend contract before this slice is validated. For specified Regions, prefer adding an explicit target/allowed-clean mask; do not fake a temporary map and confuse map semantics unless maintainers confirm this is an established convention.

## Subsequent Job behavior

cleaning-jobs will own RegionSet tasking, Segment splitting/ordering, state persistence, user control, pause/resume, retry, and summary. Only one active Job is allowed at a time.

At Job start, map identity, Published Region Set, cleaning strategy, and the footprint profile parsed from Nav2 are pinned; later edits only affect new Jobs. Pause is a non-terminal state, triggerable by the user or by safety/hardware conditions. The safety layer stops the robot independently; this package only observes and records and cannot take hard safety responsibility for `/cmd_vel`. Resume is a request; the safety layer or executor may reject it and return a stable reason code.

Job checkpoints are saved by cleaning-jobs. Precise recovery relies on a trusted coverage artifact: with an artifact, continue the uncovered remainder; without one, redo the current Segment or ask the user to confirm — the exact policy is TBD.

Long-term goals include battery, dustbin, and mop states triggering dock-cycle recharge/empty/wash before resuming coverage; whole-map, per-room, spot; and perimeter + interior combinations. All are after phase 1.

## Open boundaries

1. `oomwoo_coverage` target input: format, ownership, and version validation of the target/allowed-clean mask.
2. Segment completion condition: path completion, target coverage, no recoverable gaps, or a combination.
3. Real-robot coverage estimator: inputs, error/confidence, grid format, QoS, rebuild after restart.
4. Checkpoint atomicity, storage medium, and recovery rules across backend/robot restarts.
5. Region-to-Segment splitting rules: area, time, battery, resupply, and strategy boundaries.
6. Ordering, overlap tolerance, and shared coverage artifact of the floor-care perimeter pass and the interior sweep.
7. How Keepout/Virtual Wall are simultaneously projected onto the Nav2 costmap and coverage masks.
8. ROS fields, idempotency, QoS, and failure semantics of `RunCleaningJob`, pause/resume/cancel/status, plus battery/bin/mop/dock/localization input interfaces.
9. Migration/rebinding of region sets after map changes (hash change): whether to support it, how to reproject and revalidate. Not done in phase 1; old region sets are only retained and surfaced by the GUI.

## Repository structure and quality goals

All code, tests, interfaces, and development tools live in this repository; internal ROS 2/Python packages are split under `src/`. Cleaning domain logic remains in `oomwoo_cleaning_jobs_core`. The room-segmentation seam is owned by `oomwoo_segmentation_interfaces`, algorithm-neutral tooling by `oomwoo_segmentation`, and concrete provider code by provider packages such as `oomwoo_rose2`. Future cleaning-job actions/messages remain separate from segmentation interfaces.

Future tests should cover in headless CI: automatic/manual Region editing and validation; whole-map, per-room, and spot cleaning only the intended areas; keepouts never entered; and coverage-artifact-driven recovery after forced interruption. Implement phase 1's repeatable map fixtures and verification first, then integrate simulation execution.
