# oomwoo-cleaning-jobs

User cleaning intent and long-running job orchestration on saved maps (ROS 2 Jazzy).

**Development context and design decisions: [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)** (single source of truth — read before making changes).

## Phase 1 deliverable

`saved map → automatic candidate regions → manual editing → validation → Published Region Set persistence`

- `src/oomwoo_cleaning_jobs_core`: pure Python core library (zero ROS dependencies) — map loading and identity,
  automatic segmentation (maximin flooding watershed + merge-tree saddle merge + doorway spill clipping + doorway topology),
  Region mask editing, Keepout/Virtual Wall, validation grading, draft/published persistence
- `src/oomwoo_cleaning_jobs_ui`: PyQt5 region editor (file / `/map` dual source)

## Quick start

```bash
cd /ros_ws && colcon build --packages-select oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui
colcon test --packages-select oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui && colcon test-result

# Map render/segmentation CLI
ros2 run oomwoo_cleaning_jobs_core oomwoo-render-map <map.yaml> --segment

# Region editor GUI
ros2 run oomwoo_cleaning_jobs_ui oomwoo-cleaning-jobs-ui
```

Demo images: `docs/demo/` (segmentation + doorway markers), `docs/demo/doorway/` (comparison experiment).
GUI manual acceptance: `src/oomwoo_cleaning_jobs_ui/docs/MANUAL_ACCEPTANCE.md`.

## License

Apache 2.0
