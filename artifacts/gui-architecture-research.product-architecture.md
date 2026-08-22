# Code Context

## Files Retrieved

1. `docs/DEVELOPMENT.md` (lines 21-27, 51-67) — Phase-1 scope, two map acquisition routes, and the already approved package decision: standalone PyQt5 + `rclpy` thin adapter.
2. `docs/DEVELOPMENT.md` (lines 90-122) — required edit semantics, publish validation, persistence layout, constraints, and CI/manual-test split.
3. `docs/DEVELOPMENT.md` (lines 124-162) — execution is out of scope; target-mask integration and ROS interfaces remain later decisions.
4. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/source_map.py` (lines 33-90) — ROS-independent immutable `SourceMap`, occupancy masks, coordinate convention, and content identity.
5. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/map_io.py` (lines 30-94) — saved nav2 trinary `map.yaml` + image loader and image-to-grid conversion.
6. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/regions.py` (lines 40-190) — authoritative label-mask model, map-frame outlines, constraint application, and editor operations.
7. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/validation.py` (lines 64-130) — publish-gating errors and non-blocking warnings to display in UI.
8. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/persistence.py` (lines 140-188, 191-248) — identity-bound draft/published loading, validation, constraint encoding, and atomic generation pointers.
9. `src/oomwoo_cleaning_jobs_core/package.xml` (lines 3-23) and `src/oomwoo_cleaning_jobs_core/setup.py` (lines 28-51) — only installed package is the ROS-free core; it has no Qt/rclpy/nav message dependencies.
10. `src/oomwoo_cleaning_jobs_core/test/test_map_identity.py` (lines 1-68), `test/test_regions.py` (lines 1-180), `test/test_persistence.py` (lines 1-112), and `test/test_validation.py` (lines 1-128) — current headless contract coverage for identity, editing, persistence, and validation.

## Decision

**Choose the approved standalone `oomwoo_cleaning_jobs_ui` PyQt5 application with an optional `rclpy` map-input adapter for Phase 1. Do not implement an rqt plugin in Phase 1.**

This is not a new product choice: the repository’s recorded, confirmed decision explicitly names a PyQt5 standalone app plus `rclpy` thin adapter (`docs/DEVELOPMENT.md:51-55`), says it is the Phase-1 GUI rather than the final control app (`:21-27`), and says GUI is still absent (`:67`). Inspection confirms `src/` contains only `oomwoo_cleaning_jobs_core`; no UI package, rqt metadata, `plugin.xml`, Qt code, or ROS client code exists.

### Decision matrix

| Criterion | Standalone PyQt5 + `rclpy` adapter | rqt plugin | Phase-1 result |
|---|---|---|---|
| Matches approved architecture | Exact match | Contradicts recorded package choice | Standalone |
| Direct saved-map workflow | Natural file-open workflow; usable without ROS | Possible, but rqt/ROS runtime is required even for file work | Standalone |
| `/map` transient-local input | A small node adapter owns QoS/lifecycle | Also possible but adds rqt host lifecycle/docking constraints | Standalone |
| Headless core isolation | Preserves current zero-ROS core and pytest strategy | Can preserve it, but creates plugin-host coupling with no Phase-1 benefit | Standalone |
| Manual review/editor ergonomics | Dedicated workflow/window, clear source and publish state | Docked into a broader ROS operator UI; useful only when that host is a requirement | Standalone |
| Packaging/release cost | One ament Python UI package, console entry point | Same UI work plus rqt plugin metadata, discovery/export, host compatibility testing | Standalone |
| Future embedding | Extract a ROS-free editor presentation/controller and embed later | Immediate rqt choice would prematurely commit host API | Defer rqt |
| Execution/job integration | Not required in Phase 1 | Not required in Phase 1 | Tie; out of scope |

## Required inputs and data flow

### Saved map path

`map_io.load_map_file(yaml_path)` is the file-source boundary. It accepts nav2 trinary YAML/image only, reads `resolution` and `[x, y, yaw]` origin, treats alpha as unknown, and vertically flips image rows into OccupancyGrid row order (`map_io.py:30-94`). The UI must pass the resulting `SourceMap` unchanged to core; it must not re-threshold, resize, rotate, or reconstruct it.

### Runtime `/map` path

The documented runtime source is `nav_msgs/msg/OccupancyGrid` on `/map` using transient-local QoS (`docs/DEVELOPMENT.md:25`). The UI adapter must convert:

- `info.resolution`, `info.width`, `info.height`;
- `info.origin.position.{x,y}` and origin quaternion converted to planar yaw;
- `data` reshaped row-major to `(height, width)` `int8`.

`header.stamp`, `header.frame_id`, and map-load time must not enter `SourceMap` identity; the core’s hash uses resolution, dimensions, origin/yaw-derived quaternion, and raw `int8` cells (`source_map.py:63-79`). The adapter should retain source provenance separately for the UI (e.g., “file: …” versus “live `/map`”), not alter the domain object.

**Important state rule:** a newly received live grid whose content hash differs is a different Source Map. Load its matching draft/published set if present; otherwise show the required no-region-set/other-map-set count notice. Do not reproject or silently reuse regions: Phase 1 expressly excludes migration (`docs/DEVELOPMENT.md:59-61, 156`). Because a live map can change during SLAM, display the identity/source and require explicit reload/replace rather than mutate an open edit session under the user.

### Core pipeline owned by UI orchestration

1. Acquire one `SourceMap` from file or `/map`; calculate constraint mask via `ConstraintSet.mask_for(source)`.
2. For first-time candidates call segmentation using `source.free_mask() & ~keepout_mask`, then initialize `RegionSet.from_segmentation` with **both** the original `base_cleanable=source.free_mask()` and `keepout_mask`. This pairing is mandatory for later constraint removal semantics (`docs/DEVELOPMENT.md:116-118`; `regions.py:69-95`).
3. Load existing draft using `RegionSetStore.load_draft(source)`; otherwise show candidates as editable draft. On an existing source, independently expose `load_published(source)` status/version.
4. Apply paint/create/erase/rename/merge/split directly to `RegionSet`; masks are authoritative. Rendering may use `outline()` but must not make contours authoritative (`regions.py:111-145, 168-190`).
5. When keepouts/walls change, construct an immutable `ConstraintSet`, call `apply_keepout_mask(constraints.mask_for(source))`, and visibly explain that removed constrained cells do not revive when a constraint is deleted (`regions.py:149-164`).
6. Save drafts with `RegionSetStore.save_draft`. Before publish, render all `validate_region_set` issues; publish only through `RegionSetStore.publish`, which validates and writes version/timestamp atomically (`persistence.py:146-186`).

## Persistence and UI-visible state

Storage is map-identity-scoped under `~/.local/share/oomwoo_cleaning_jobs/maps/<full-hash>/` and stores map snapshot YAML/PGM plus lossless raw cells, draft/published metadata, PNG masks, and constraints (`docs/DEVELOPMENT.md:104-112`; `persistence.py:191-213`). Draft/published active paths are atomically switched symlinks to immutable generations (`persistence.py:232-248`). Therefore the UI should never edit YAML/PNG directly or infer persistence layout; it should use `RegionSetStore`.

The publish dialog/state must distinguish:

- **Errors (block publish):** empty set, non-cleanable region cells, footprint-eroded empty region, and keepout intersections (`validation.py:78-119`).
- **Warnings (publish allowed but prominent):** unassigned cleanable cells and disconnected reachable core (`validation.py:103-128`).
- **Draft vs Published:** saving a draft may preserve invalid intermediate work; published data is revalidated on load (`persistence.py:174-188`).

## Expected user flows

1. **Open saved map:** choose `map.yaml` → show map provenance and identity → load draft or offer automatic candidate segmentation → visibly show unknown/obstacle, keepout, unassigned, and each labeled region.
2. **Attach to live map:** start the adapter, subscribe with transient-local QoS, wait for one grid → show source/identity → use the same draft/candidate path. A changed identity requires user confirmation to replace the active session; no migration.
3. **Edit review:** select/create a region; paint/erase, rename, merge, or draw a split. A paint/create stroke is clipped to cleanable space and can preempt another region, so the UI must show the actual clipped result and a conspicuous preemption warning (`docs/DEVELOPMENT.md:90-94`; `regions.py:168-190`).
4. **Edit constraints:** create map-frame polygon keepouts or explicit-width virtual walls; immediately clip regions. Removing a constraint only restores availability, not previous region ownership.
5. **Draft/publish:** save draft; show current validation report; block errors, acknowledge warnings, publish; show published version and UTC timestamp. Map change means no publish/edit against prior identity.

## Thin-adapter boundary

### Keep in `oomwoo_cleaning_jobs_core` (unchanged)

`SourceMap`, file IO, segmentation, `RegionSet`, `ConstraintSet`, validation, and `RegionSetStore` remain ROS- and Qt-free. This is already enforced by package intent/dependencies (`package.xml:7-22`) and is covered by headless tests.

### Put in new `oomwoo_cleaning_jobs_ui`

- Qt window/canvas, view-model/presenter, undo/session state, source/provenance display, issue presentation, dialogs, and manual acceptance support.
- A small ROS adapter that creates the node/subscription, owns transient-local QoS and executor/thread handoff, converts `OccupancyGrid` to `SourceMap`, and emits an immutable map snapshot to the UI thread.
- A composition root that calls core APIs; it must not duplicate segmentation, rasterization, identity, validation, or persistence rules.

Use the core object APIs rather than exposing raw labels to Qt widgets. In particular, canvas coordinate conversion must honor `SourceMap` row-0-is-low-y and `RegionSet.outline()` origin/yaw convention (`source_map.py:33-61`; `regions.py:119-145`).

## Package implications

The current package is only `oomwoo_cleaning_jobs_core`; it is ament Python and exports only `oomwoo-render-map` (`setup.py:30-50`). Phase-1 standalone implementation needs a **new** `src/oomwoo_cleaning_jobs_ui` ament-Python package, not changes that make core depend on GUI/ROS libraries.

Expected UI-package dependencies: Python Qt binding chosen by the approved design (PyQt5), `rclpy`, `nav_msgs`, and the core package; likely `geometry_msgs` is unnecessary unless a UI ROS boundary later sends geometry messages. Provide a console entry point for the standalone application. Do not add `oomwoo_cleaning_interfaces`: it is explicitly deferred until a genuine cross-process message need (`docs/DEVELOPMENT.md:51-55`).

An rqt implementation would additionally require rqt plugin discovery/export metadata and host lifecycle tests. It would not remove the need for the canvas/controller/map adapter; it only changes the host. That is avoidable Phase-1 scope.

## Migration / embedding option

Do **not** migrate persistence or core data for embedding. Keep the same `SourceMap`/`RegionSet`/`RegionSetStore` contract. To make later rqt embedding low-risk:

1. Build the Phase-1 window around a host-neutral `EditorController`/presentation layer that accepts core objects and emits commands/results; keep all rclpy subscription/executor code in a `RosMapSource` adapter.
2. Make the standalone app only a composition root: Qt application + controller + file source + optional ROS source.
3. Later create a thin rqt host wrapper that supplies rqt parent/dock lifecycle and instantiates the same editor widget/controller. Do not make the core an rqt dependency and do not change persisted schemas.

This is an embedding option, not a Phase-1 deliverable. It preserves the approved standalone workflow and avoids a second editor implementation.

## Test and manual acceptance strategy

### Automated

Keep current headless tests as the domain acceptance baseline: map identity (`test/test_map_identity.py`), file IO (`test/test_map_io.py`), editing (`test/test_regions.py`), constraints (`test/test_constraints.py`), persistence (`test/test_persistence.py`), and validation (`test/test_validation.py`). Add UI-independent tests for the future OccupancyGrid-to-`SourceMap` converter (dimensions, raw `int8` preservation, quaternion-yaw extraction, identity stability against header changes) using message-shaped fakes if desired; ROS subscription behavior should be integration-tested only when the UI package exists.

The repository specifies GUI outside CI with a manual checklist (`docs/DEVELOPMENT.md:120-122`). Current core verification passed when invoked with its source path on `PYTHONPATH`: `77 passed`.

### Manual Phase-1 checklist

- File-load a known nav2 trinary map and confirm orientation/origin-aligned overlays; unknown and obstacle cells cannot be painted.
- Start the app after a transient-local `/map` publisher is already active; verify the first retained map arrives and matches file-load identity for identical grid metadata/cells.
- Change a live grid cell; confirm a new identity, no silent draft reuse, and explicit other-map-set notice.
- Verify create/paint preemption, empty clipped stroke feedback, erase/delete, merge, split, rename, and unassigned overlay.
- Add/remove keepout and virtual wall; verify immediate clipping and no automatic region-cell resurrection after removal.
- Save/restart/load draft; publish valid data; check version/timestamp. Confirm each blocking error blocks publish and each warning remains visible but permits publish.
- Exercise the GUI without ROS by opening a file, proving the adapter is optional.

## Review findings and residual risks

### Findings

- **High — architecture/package gap:** `docs/DEVELOPMENT.md:53-55` approves `oomwoo_cleaning_jobs_ui`, but no such package exists under `src/`; all currently installed code is core. The Phase-1 GUI cannot be accepted until the new standalone UI package and its manual checklist exist.
- **High — live-map consistency risk:** documentation requires `/map` transient-local input but there is no adapter or policy implemented for a map update while editing. Treat each changed identity as a new map and require explicit replacement; silent reuse would bind region masks to the wrong source.
- **Medium — coordinate conversion risk:** file IO flips image rows and the domain model uses row 0 as lowest y (`map_io.py:84-85`, `source_map.py:36-37`). A Qt canvas or ROS converter that flips again/misses origin yaw will render/edit masks in the wrong physical locations.
- **Medium — UX safety requirement:** core correctly permits paint preemption and constraint clipping, but it does not provide UI notifications. The adapter/UI must surface both effects; otherwise users can unintentionally shrink a region without realizing it.
- **Low — direct test invocation ergonomics:** `pytest -q src/oomwoo_cleaning_jobs_core/test` from repository root fails collection because the package is not on `PYTHONPATH`; running from the package with `PYTHONPATH=$PWD` passes. CI should install the ament package or set the import path explicitly.

### Residual risks

- Segmentation has documented semantic ambiguity between furniture gaps and doorways; Phase-1 relies on manual review (`docs/DEVELOPMENT.md:69, 82`).
- No region migration/reprojection exists for a changed map identity by approved Phase-1 scope.
- `/map` QoS and executor/thread lifecycle have not yet been tested because no UI ROS adapter exists.
- rqt embedding remains intentionally unvalidated; it should be assessed only when an rqt-host requirement is concrete.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Concrete standalone-vs-rqt decision matrix, package implications, adapter boundary, migration option, test strategy, and severity-tagged findings cite exact repository paths and line ranges."
    }
  ],
  "changedFiles": [
    "artifacts/gui-architecture-research.product-architecture.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "cd src/oomwoo_cleaning_jobs_core && PYTHONPATH=\"$PWD\" pytest -q test",
      "result": "passed",
      "summary": "77 passed in 10.39s"
    },
    {
      "command": "pytest -q src/oomwoo_cleaning_jobs_core/test",
      "result": "failed",
      "summary": "Collection failed: oomwoo_cleaning_jobs_core was not importable from repository-root invocation."
    }
  ],
  "validationOutput": [
    "Confirmed only oomwoo_cleaning_jobs_core exists under src; no UI/rqt plugin code or package metadata was found.",
    "Headless core suite passed with the package source on PYTHONPATH."
  ],
  "residualRisks": [
    "Live /map update policy and ROS QoS/executor lifecycle are not implemented or tested.",
    "Coordinate conversion can misplace edits if Qt/ROS adapters violate the row/origin/yaw conventions.",
    "Approved Phase-1 does not migrate region sets across map identity changes."
  ],
  "noStagedFiles": true,
  "diffSummary": "Added the requested architecture research artifact only; no production source or package files were modified.",
  "reviewFindings": [
    "high: docs/DEVELOPMENT.md:53-55 specifies the approved UI package, but src contains no oomwoo_cleaning_jobs_ui implementation.",
    "high: docs/DEVELOPMENT.md:25 specifies transient-local /map input, but no ROS adapter exists to preserve identity/session consistency.",
    "medium: src/oomwoo_cleaning_jobs_core/map_io.py:84-85 and source_map.py:36-37 establish non-default row orientation that UI adapters must preserve."
  ],
  "manualNotes": "This research follows the recorded approved standalone Phase-1 decision; rqt is documented only as a future embedding option."
}
```