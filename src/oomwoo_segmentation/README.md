# oomwoo_segmentation

Room segmentation for ROS 2 based on the ROSE2 method by [AISLab, University of
Milano-Bicocca](https://github.com/aislabunimi/ROSE2).

It owns:

- the room-segmentation engine (`engine/`), an in-memory port of the pinned
  upstream ROSE + ROSE2 two-stage pipeline (FFT structural
  filtering -> Hough walls -> angular/spatial clustering -> extended lines ->
  planar cells -> affinity DBSCAN -> rooms);
- the `oomwoo_segmentation` ROS 2 action server (`node.py`) implementing
  `oomwoo_segmentation_msgs/action/SegmentRooms` with feedback and
  cancellation;
- canonical Python result types and contract validation;
- Nav2 trinary map loading and `OccupancyGrid` conversion;
- the `SegmentRooms` action client;
- deterministic source-map and room-label rendering;
- the `oomwoo-render-map` CLI.

The shared interface is `oomwoo_segmentation_msgs` (standard ROS 2 message
types only: `OccupancyGrid`, `sensor_msgs/Image`, `geometry_msgs`).

## Test maps

`test/maps/` contains the demo renders (`demo/`), the upstream ROSE2
benchmark maps (`rose2_upstream/`), and additional review cases from
`ipa320/ipa_coverage_planning` (`ipa/`). See `THIRD_PARTY.md` for provenance
and licenses. Batch verification runs write artifacts under the
repository-root `output/` directory:

```bash
python3 test/run_map_batch.py test/maps/ipa/*.png --output-root output/ipa
```

## Upstream and credits

The segmentation engine is derived from
[`aislabunimi/ROSE2`](https://github.com/aislabunimi/ROSE2), pinned at commit
`3a010b9e6bb2477de3b5b46208ebfccd71dfafbf`, licensed GPLv3 by AISLab
(University of Milano-Bicocca). All local modifications are listed in
`THIRD_PARTY.md`.

## License

GPL-3.0-only: the engine derives from `aislabunimi/ROSE2`. See
`THIRD_PARTY.md`.
