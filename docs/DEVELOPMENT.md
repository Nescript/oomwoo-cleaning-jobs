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
| Segment | Part of a Job's target handled by one cleaning strategy; not necessarily equal to a Region. |
| Coverage artifact | Verifiable record of covered space (e.g. a coverage grid); a percentage by itself is not an artifact. |

Automatic partitioning is a replaceable strategy (currently distance transform + maximin flooding watershed, see "Phase 1 implementation decisions · Automatic segmentation"); when confidence is insufficient it degrades to connected free space and explicitly marks uncertain/unclassified areas.

Manual editing supports at least create, move/delete/rename, merge/split of Regions, and creation of Keepout/Virtual Wall. Immediate clipping during editing and validation grading at publish time are described in "Phase 1 implementation decisions". Unassigned or deliberately uncleaned free space is allowed, but the GUI must present it clearly.

The GUI main flow is now: **automatic segmentation → name candidate Regions one by one → save/validate and publish**. Immediately after candidates are generated, naming dialogs pop up in label order; canceling can be resumed later from "Name candidates one by one". Create, paint, erase, merge, split, and constraint editing live in the collapsed-by-default "Advanced editing (use only when candidates are wrong)" section, still serving as the correction mechanism for automatic segmentation errors.

## Phase 1 implementation decisions

The following decisions were confirmed point by point with the user on 2026-08-22 (grilling discussion); re-confirm with the user before changing them.

### Package structure

- `src/oomwoo_cleaning_jobs_core`: pure Python library, zero ROS dependencies. Contains map file loading, automatic segmentation, mask editing, constraints, validation, and persistence; testable with headless pytest.
- `src/oomwoo_cleaning_jobs_ui`: standalone PyQt5 application + rclpy node, thin adaptation layer.
- `oomwoo_cleaning_interfaces` is deferred until the first real cross-process message need (expected in phase 2).

### Map identity and change detection

identity = SHA-256(**float32** bytes of `resolution` + `width` + `height` + `origin` position/orientation + raw int8 cell data), excluding `header.stamp`, `frame_id`, `map_load_time`, with no trinarization. The short id is the first 12 hex digits. Resolution is normalized to float32: `OccupancyGrid.info.resolution` is float32 while map.yaml is float64 — without normalization the same map loaded via topic and via file would get different identities (fixed during GUI dual-source verification).

The hash acts as a **change detector and storage key**, not as multi-map management. A saved map is treated as a static artifact; the only source of change is the user re-mapping/re-saving. A hash change means a new map: when no matching Region Set is found, the GUI clearly states "no region set for the current map; N region sets belonging to other maps exist on disk". Phase 1 does not migrate/reproject region sets (origin and resolution may both change, making pixel-level migration unreliable); see open boundary 9.

### Map file loading fidelity convention (verified against nav2 jazzy map_io.cpp)

Trinary loading: `occ = 1 - color/255` (negate=0); `occ >= occupied_thresh` → 100, `occ <= free_thresh` → 0, otherwise -1; `alpha < 255` is always unknown; the image's top row corresponds to the map's maximum y, so the map is flipped vertically after loading. map_saver always writes pixels 0 (occupied)/254 (free)/205 (unknown) with `occupied_thresh: 0.65, free_thresh: 0.196` (205 thereby reads back as unknown). The core loader `map_io.load_map_file` matches this behavior and supports trinary only.

Fixed the external `oomwoo_sim_support/maps/test_room` asset chain: the PGM correctly represents outside-wall unknown as 205, but the YAML previously used `free_thresh: 0.25`, which misread 205 (occ≈0.196) as free and produced candidate regions outside the walls. The generation script and the sim/deploy YAMLs are now unified at `free_thresh: 0.196`.

### Automatic segmentation

Distance transform + watershed (OpenCV/scipy implementation): free mask → distance transform → local-maxima markers → **maximin (widest-path) flooding** (self-implemented, propagates only within free space — `cv2.watershed` floods the whole image, crosses walls, and spills regions; deprecated; ties on bottleneck priority are broken by geodesic distance to the seed, so boundaries land on doorways/saddles) → area merge (regions below `min_region_area`, default 1 m², merge into the **most widely connected** neighbor — merging by shared boundary length leaks through doors; deprecated) → **saddle merge** (`_connection_values` superlevel-set merge tree gives the true saddle height independent of the boundary line; union-find merge when saddle ≥ `saddle_merge_ratio` (default 0.8) × the smaller peak height; real doorways typically have ratio < 0.5, spurious splits of the same open area > 0.8) → **doorway spill clipping** (`_clip_doorway_spills`: maximin flooding assigns cells on both sides of a door whose dist is below the door saddle to the opposite region, forming spill bands; for each adjacent pair a cut line is generated at the merge-tree saddle cell, temporarily blocked, and cells landing in the other component are reassigned to it, forcing the boundary onto the doorway) → ridge marking (one layer of cells on each side of the contact band is marked unclassified). When distance peaks degenerate (single peak in a large open room), the whole area becomes a single candidate marked low-confidence. All thresholds are parameters. This algorithm is an initial "use it and see" strategy and is replaceable.

Known segmentation behavior: after maximin flooding + merge-tree saddle merge, the real living_room (single room with furniture) converges to exactly 1 candidate and 0 unclassified across the full ratio 0.5–0.8 range; 5/6/7-room grids, the 4-room + corridor apartment, and wall-adjacent furniture scenes all segment into exactly the expected room counts; real doorways (0.5 m) are not wrongly merged, wide openings (1.3 m) merge correctly. Remaining edge cases: a small room with a wide door (ratio 0.6–0.8) may be wrongly merged/split — corrected by manual editing in the GUI; narrow zones with distance values below `min_peak_height_m` produce no seeds and remain unclassified; a 0.5 m furniture gap is geometrically indistinguishable from a 0.5 m doorway (a semantic problem) and relies on user review of candidates.

### Segmentation comparison experiment: skeleton + doorway-cut approach (doorway_demo)

`doorway_demo.py` (experimental, not part of the core pipeline) implements the user-proposed approach "skeleton → doorway candidates → scoring → cutting → connected components". Findings:

- Doorway detection is reliable: all real doors (0.5 m) are detected; corridor apartment yields exactly 5 regions, living_room 1 region, two rooms 2 regions.
- Doorway scoring is the hard part: 1-cell thin walls fragment into small connected bodies at doorway crossings, and "wall support" by component size cannot be distinguished from furniture blocks (size ranges overlap); furniture-gap cut lines slice rooms diagonally (the grid6 demo splits one room diagonally). This is the other face of the same "furniture gap vs doorway" semantic ambiguity as the watershed approach.
- Morphological closing preprocessing harms thin walls (erosion shatters the wall network); it suits only real noisy maps.
- Doorways/topology as first-class citizens are the real value of this approach (needed for phase 2 Segment ordering and navigation); a hybrid is worth considering later: watershed regions + merge-tree saddles as doorway records.

**The hybrid approach is implemented** (segmentation.py): region generation remains maximin flooding + merge tree; `SegmentationResult.doorways` outputs doorway records (`Doorway`: adjacent region pair, saddle center, clearance, width ≈ 2×clearance, ratio, likely_door), and `adjacent_labels()` gives topological adjacency. Implementation notes: adjacency is determined by a **geodesic dilation contact band** (plain dilation crosses walls and jumps across ridges); corner-diagonal adjacency is filtered by a clearance floor; when a region pair has multiple doors, only the one with the highest saddle is recorded. Rendering overlays magenta doorway markers; the CLI prints topology edges. Tests in `test_topology.py` (89 pytest total, core 78 + ui 11).

Known risk (2026-08-22, characterization test added): `_clip_doorway_spills` still uses the merge-tree saddle cell as the cut center; `test_transitive_connection_uses_local_contact_saddle` confirms that for `direct=False` transitive connections in a 3×2 grid, this saddle can differ from the geodesic contact-band saddle of the current region pair. Switching transitive connections to the contact band was tried but caused cross-door mislabeling inside rooms of the furniture grid and corridor apartment, so it was reverted. After constructing a fixture that reproduces an actual clipping error, re-evaluate a local positioning scheme constrained by doorway direction / both-sides peak connectivity.

Demo images: `docs/demo/` (main approach segmentation + doorway markers) and `docs/demo/doorway/` (doorway-cut approach triplets).

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

Core model implemented: `constraints.ConstraintSet` holds `Keepout` (map-frame polygons) and `VirtualWall` (center lines with explicit `width_m`), rasterized according to the Source Map's origin/yaw. Callers pass `source.free_mask() & ~constraints.mask_for(source)` into `segment(..., cleanable_mask=...)`; if a Region Set is initialized from those candidates, the raw `source.free_mask()` and the current constraint mask must be passed separately to `RegionSet.from_segmentation(..., base_cleanable=..., keepout_mask=...)`. After constraints change, use `RegionSet.apply_keepout_mask()` to immediately clip existing Region cells. Removing a constraint only restores cleanable space; it does not revive clipped Region cells.

### Test strategy

Headless pytest: primarily synthetic maps (corridor + multi-room, allowing exact assertions on segment counts, no overlap, validation errors); 1–2 real saved maps committed to the repository as smoke regression (segmentation does not crash, candidate counts within a reasonable range). The GUI is not in CI; a manual acceptance checklist is kept.

### Implementation status

Implemented: `src/oomwoo_cleaning_jobs_core` (ament_python) contains `source_map.SourceMap` (identity/masks), `map_io`, `segmentation` (maximin flooding + merge-tree saddle merge + doorway spill clipping + doorway topology records, with support for an external cleanable mask), `regions.RegionSet` (mask editing: paint/erase/create/delete/rename/merge/split, immediate clipping, later-painter preemption, outline derivation, Keepout application), `constraints` (map-frame polygon Keepout, explicit-width Virtual Wall, origin-yaw rasterization), `validation` (error/warning graded validation), `render`/`render_map` CLI (`oomwoo-render-map`, `--segment` produces overlays); synthetic map fixtures under `test/` (`fixtures`: two rooms / with unknown block / open plan / tiny room / open-plan two-zone / N-room grid / corridor apartment) with 78 core pytest and 11 GUI/adapter tests under `oomwoo_cleaning_jobs_ui/test` (including regression coverage of coordinate mapping, naming robustness, editing gating, and split feedback). `oomwoo_cleaning_jobs_ui` is implemented as a standalone PyQt5 + rclpy package with file loading, candidate review, Region editing, Keepout/Virtual Wall coordinate input, and draft/publish; `RosMapSource` provides transient-local `/map` adaptation via an executor thread, the GUI thread asks for confirmation before replacing the editing session on identity change and keeps current edits when identity is unchanged; when the current map has no region set, the count of region sets belonging to other maps is shown explicitly. The GUI manual acceptance checklist is at `src/oomwoo_cleaning_jobs_ui/docs/MANUAL_ACCEPTANCE.md`. Demo outputs are in `docs/demo/` (segmentation renderings of the real living_room and synthetic two-room maps).


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

All code, tests, interfaces, and development tools live in this repository; internal ROS 2/Python packages may be split under `src/`. The core is Python domain/application logic; ROS 2 is the adaptation layer. Shared ROS actions/messages go into `oomwoo_cleaning_interfaces` in the same repository, so the GUI and executors do not depend on orchestrator internals.

Future tests should cover in headless CI: automatic/manual Region editing and validation; whole-map, per-room, and spot cleaning only the intended areas; keepouts never entered; and coverage-artifact-driven recovery after forced interruption. Implement phase 1's repeatable map fixtures and verification first, then integrate simulation execution.
