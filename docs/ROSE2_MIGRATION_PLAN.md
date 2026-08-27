# ROSE2 room-segmentation migration

Status: **implemented and verified** on branch `ROSE2`.

> **Update (2026-08-25): in-memory integration.** The `oomwoo_rose2` wrapper package (vendored upstream code + temporary directories) has been replaced by an in-memory engine in `oomwoo_segmentation` (`engine/`), still derived from the same pinned upstream ROSE2 commit. The custom `oomwoo_segmentation_interfaces` package has been replaced by the standard-type `oomwoo_segmentation_msgs`, and the PyQt5 verification GUI (`oomwoo_cleaning_jobs_ui`) has been removed. The sections below remain as the historical record of the original wrapper migration; the package table reflects the current layout.

## Objective

Replace the in-process cleaning-jobs room segmentation with a ROS 2 module seam. ROSE2 is the only production provider on this branch. There is no legacy implementation, fallback, or runtime provider switch.

## Package boundaries

| Package | License | Responsibility |
| --- | --- | --- |
| `oomwoo_segmentation_msgs` | Apache-2.0 | Standard-type `SegmentRooms` action and room/wall messages |
| `oomwoo_segmentation` | GPL-3.0 | Room-segmentation engine based on ROSE + ROSE2, Source-map identity and I/O, canonical models and validation, ROS conversions, action server/client, rendering, CLI |
| `oomwoo_cleaning_jobs_core` | Apache-2.0 | Editable Regions, constraints, persistence, validation; no segmentation algorithm |

The action boundary allows another implementation package to replace ROSE2 without changing cleaning-jobs or the rendering tools. Provider-specific parameters stay on the provider node and do not leak into the common action.

## Canonical contract

- Input is an immutable `nav_msgs/OccupancyGrid` and optional same-shape cleanable mask.
- Output labels use OccupancyGrid row order and `int32` values.
- Label `0` means unassigned; positive labels are deterministic and contiguous.
- Occupied, unknown, and cleanable-mask-excluded cells must remain label `0`.
- Room metadata is derived from the canonical labels.
- Failures are explicit action results. Cancellation and progress feedback are supported.
- Provider diagnostics are optional PNG images and never affect the authoritative labels.

## ROSE2 baseline and adaptation

Upstream: [`aislabunimi/ROSE2`](https://github.com/aislabunimi/ROSE2) commit `3a010b9e6bb2477de3b5b46208ebfccd71dfafbf`, GPLv3.

The adapter preserves the upstream two-stage computation:

1. ROSE FFT structural filtering and dominant directions.
2. ROSE2 Hough walls, line clustering/extension, edge and cell construction, affinity/DBSCAN clustering, and Shapely room polygons.
3. Polygon rasterization and canonicalization back to the original source-grid coordinates.

ROS 1 nodes, services, RViz integration, and pickled Python-object messages are not carried over. Modern-library compatibility changes and provenance are listed in `src/oomwoo_segmentation/THIRD_PARTY.md`.

## Completed migration steps

- [x] Define and build the shared ROS 2 action/messages.
- [x] Extract algorithm-neutral map/model/validation/client/rendering code.
- [x] Port the pinned ROSE2 computation into an isolated GPLv3 package.
- [x] Add ROS 2 action server, parameters, launch file, and diagnostics.
- [x] Migrate cleaning-jobs core/UI to common models and action client.
- [x] Move GUI segmentation off the Qt main thread.
- [x] Add deterministic test-only provider fakes for cleaning domain tests.
- [x] Delete maximin/watershed/saddle/doorway implementations, tests, CLI, parameters, and demo.
- [x] Update package manifests, entry points, README, and development documentation.
- [x] Generate final and intermediate ROSE2 example images under `output/` (test inputs since moved to `src/oomwoo_segmentation/test/maps/`).

## Verification

- Joint `colcon build` of all five affected packages: passed.
- Full `colcon test` and `colcon test-result --verbose`: 81 tests, 0 errors, 0 failures, 1 dependency-gated skip.
- Full source pytest with ROSE2 dependencies: 96 passed, including Qt worker execution, stale-map-result rejection, strict shared-contract validation, post-extraction cancellation, detected-wall contract coverage, and six exact demo-map room/unassigned regressions.
- ROSE2 provider tests with full dependencies: 16 passed, including the real pinned upstream pipeline, cropped/real-map compatibility, frame-line deduplication, furniture-line filtering, structural-raster edge recovery, strict frame-fringe rejection, and `room4` label/wall regressions.
- End-to-end ROS 2 server/client test: cancellation returned Action `CANCELED` plus contract `STATUS_CANCELLED`; a subsequent goal succeeded and produced base, final segmentation, cleaned-map, extended-line, and label-overlay PNGs.
- Legacy algorithm symbol scan: no runtime implementation or fallback remained.
- `git diff --check`: passed.

## Remaining operational checks

- Install dependencies from `src/oomwoo_segmentation/requirements.txt` (or equivalent rosdep packages) in the deployment image.
- Have the project license owner confirm the final GPLv3 distribution arrangement.
- Treat optional `rooms_voronoi=true` as a separate acceptance target; the default and tested profile is `false`.
