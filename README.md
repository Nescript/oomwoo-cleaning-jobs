# oomwoo-cleaning-jobs

User cleaning intent and long-running job orchestration on saved maps (ROS 2 Jazzy).

**Development context and design decisions: [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)** (single source of truth — read before making changes). The completed replacement record is in [docs/ROSE2_MIGRATION_PLAN.md](docs/ROSE2_MIGRATION_PLAN.md).

## Phase 1 deliverable

`saved map → automatic candidate regions → manual editing → validation → Published Region Set persistence`

- `src/oomwoo_segmentation_interfaces`: algorithm-neutral typed ROS 2 `SegmentRooms` action
- `src/oomwoo_segmentation`: Apache-2.0 map model/I/O, contract validation, action client, and deterministic rendering; contains no segmentation algorithm
- `src/oomwoo_rose2`: GPLv3 ROS 2 port of the pinned ROSE + ROSE2 pipeline and action server
- `src/oomwoo_cleaning_jobs_core`: Region mask editing, Keepout/Virtual Wall, validation grading, and draft/published persistence
- `src/oomwoo_cleaning_jobs_ui`: PyQt5 region editor (file / `/map` dual source)

## Quick start

```bash
cd /ros_ws/src/oomwoo-cleaning-jobs
python3 -m pip install -r src/oomwoo_rose2/requirements.txt

cd /ros_ws && colcon build --packages-select \
  oomwoo_segmentation_interfaces oomwoo_segmentation oomwoo_rose2 \
  oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui
colcon test --packages-select \
  oomwoo_segmentation_interfaces oomwoo_segmentation oomwoo_rose2 \
  oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui
colcon test-result --verbose
source install/setup.bash

# Start the only production segmentation provider
ros2 launch oomwoo_rose2 rose2.launch.py

# Render the map and ROSE2 room labels through the shared action
ros2 run oomwoo_segmentation oomwoo-render-map <map.yaml> --segment

# Region editor GUI
ros2 run oomwoo_cleaning_jobs_ui oomwoo-cleaning-jobs-ui
```

ROSE2 renderings are written by `oomwoo-render-map`; provider diagnostics can be requested with `--diagnostics-dir`. A generated two-room example is in `docs/demo/two_rooms.segments.png`, with ROSE2 stage images under `docs/demo/rose2/`. The five-map regression output and visual assessment are in `docs/output/README.md` and `docs/output/summary.png`.
GUI manual acceptance: `src/oomwoo_cleaning_jobs_ui/docs/MANUAL_ACCEPTANCE.md`.

## License

The OOMWOO packages are Apache-2.0 except `oomwoo_rose2`, which is GPL-3.0-only because it derives from `aislabunimi/ROSE2`. See `src/oomwoo_rose2/THIRD_PARTY.md`.
