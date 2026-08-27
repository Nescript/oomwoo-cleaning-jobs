# oomwoo-cleaning-jobs

User cleaning intent and long-running job orchestration on saved maps (ROS 2 Jazzy).

Room segmentation is based on the ROSE2 method by [AISLab, University of Milano-Bicocca](https://github.com/aislabunimi/ROSE2) (GPLv3): FFT structural filtering and dominant-direction extraction, Hough wall detection and clustering, planar cell construction, and DBSCAN room clustering. This repository integrates the pipeline as an in-memory module behind a ROS 2 action interface.

**Development context and design decisions: [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)** (single source of truth — read before making changes). The completed replacement record is in [docs/ROSE2_MIGRATION_PLAN.md](docs/ROSE2_MIGRATION_PLAN.md).

## Phase 1 deliverable

`saved map → automatic candidate regions → manual editing → validation → Published Region Set persistence`

- `src/oomwoo_segmentation_msgs`: standard-type ROS 2 `SegmentRooms` action and room/wall messages
- `src/oomwoo_segmentation`: room-segmentation engine based on the pinned ROSE + ROSE2 pipeline, action server and client, map model/I/O, contract validation, and deterministic rendering (GPL-3.0-only)
- `src/oomwoo_cleaning_jobs_core`: Region mask editing, Keepout/Virtual Wall, validation grading, and draft/published persistence

## Quick start

```bash
cd /ros_ws/src/oomwoo-cleaning-jobs
python3 -m pip install -r src/oomwoo_segmentation/requirements.txt

cd /ros_ws && colcon build --packages-select \
  oomwoo_segmentation_msgs oomwoo_segmentation oomwoo_cleaning_jobs_core
colcon test --packages-select \
  oomwoo_segmentation_msgs oomwoo_segmentation oomwoo_cleaning_jobs_core
colcon test-result --verbose
source install/setup.bash

# Start the room segmentation action server
ros2 run oomwoo_segmentation oomwoo_segmentation_node

# Render the map and room labels through the shared action
ros2 run oomwoo_segmentation oomwoo-render-map <map.yaml> --segment
```

Segmentation renderings are written by `oomwoo-render-map`; diagnostics can be requested with `--diagnostics-dir`. Test input maps live under `src/oomwoo_segmentation/test/maps/` (local demo maps in `demo/`, upstream ROSE2 maps in `rose2_upstream/`, ipa_coverage_planning review cases in `ipa/`). Every test run writes its outputs under `output/` — see `output/demo/`, `output/ipa/`, and `output/rose2_upstream/`, with the visual assessment in `output/README.md`.

## License

The OOMWOO packages are Apache-2.0 except `oomwoo_segmentation`, which is GPL-3.0-only because its engine derives from [`aislabunimi/ROSE2`](https://github.com/aislabunimi/ROSE2). Upstream attribution and the full list of local modifications are recorded in `src/oomwoo_segmentation/THIRD_PARTY.md`.
