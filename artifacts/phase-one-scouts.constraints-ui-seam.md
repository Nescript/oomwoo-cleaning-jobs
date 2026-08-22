# Code Context

## Files Retrieved
1. `docs/DEVELOPMENT.md` (lines 21-27, 51-55, 90-118) — authoritative Phase-1 delivery, core/UI split, constraint semantics, persistence layout, and test boundary.
2. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/source_map.py` (lines 33-90) — immutable map, row-0-is-bottom convention, yaw-bearing origin, and the `free_mask()` starting point for Cleanable Space.
3. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/segmentation.py` (lines 106-190) — segmentation derives its free space exclusively from `SourceMap.free_mask()`.
4. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/regions.py` (lines 40-77, 93-158) — `RegionSet.cleanable` is the existing clipping seam; paint/create intersect it and preempt labels.
5. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/validation.py` (lines 64-140) — accepts a caller-provided keepout mask and checks region intersection, but owns no constraint model/rasterization.
6. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/map_io.py` (lines 30-94) — file-loader path, including vertical flip into the common bottom-row-first grid convention.
7. `src/oomwoo_cleaning_jobs_core/test/test_regions.py` (lines 33-73, 157-177) — existing immediate clipping, preemption, outline, and unassigned-space tests to extend.
8. `src/oomwoo_cleaning_jobs_core/test/test_validation.py` (lines 28-141) — existing invariant/error/warning and injected-mask tests to extend.
9. `src/oomwoo_cleaning_jobs_core/package.xml` (lines 3-22) and `setup.py` (lines 4-27) — only the core ament package exists; no UI package/configuration exists.

## Key Code

- Cleanable Space currently starts as `SourceMap.free_mask()` (`source_map.py:81-83`), while segmentation independently uses that result (`segmentation.py:136-140`). Constraints therefore must reach **both** the candidate-generation input and `RegionSet.cleanable`; merely clipping later edits leaves invalid candidate labels under a newly created constraint.
- `RegionSet.paint()` and `create()` implement the desired interaction seam: `stroke & self.cleanable` then overwrite labels (`regions.py:127-147`). `unassigned_cleanable_mask` is already the correct GUI warning overlay (`regions.py:96-99`).
- Validation has an intended but incomplete seam: `validate_region_set(..., keepout_mask=...)` detects `region_in_keepout` (`validation.py:64-115`); `check_masks_overlap()` is ready for masks loaded individually from persistence (`lines 129-140`). No shape validation, constraint union, geometry, or persistence is implemented.
- `map_io.load_map_file()` flips image rows (`map_io.py:84-85`) and `SourceMap` defines row 0 as map bottom (`source_map.py:36-37`). The UI OccupancyGrid adapter and geometry rasterizer must preserve this convention.

## Architecture

### Proposed sequencing and dependencies
1. **Core constraints first (Milestone 2 prerequisite):** introduce a pure-Python constraint representation/rasterization boundary that produces a boolean keepout mask in `SourceMap.cells` shape. It must support polygon Keepout and line-derived Virtual Wall, and map-frame/grid conversion using resolution and origin.
2. **Make Cleanable Space authoritative:** derive `free & ~keepout`, feed it to segmentation, construct `RegionSet` with it, and on every constraint change atomically remove now-forbidden labels before exposing the set. Existing paint/create then clip correctly without UI-specific logic.
3. **Validate and persist core state:** invoke validation with the same effective keepout mask; implement the documented snapshot/draft/published tree and load-time mask-overlap checks. The persistence work must depend on the constraint representation from step 1, not independently invent it.
4. **Core pytest:** add deterministic geometry/raster, subtract/resegment, edit clipping/preemption after adding/removing constraints, validation, round-trip draft/published, and malformed/overlapping loaded masks tests.
5. **Thin UI package (Milestone 3):** only after core APIs exist, create `src/oomwoo_cleaning_jobs_ui` as a separate `ament_python` package depending on core, PyQt5, rclpy, and ROS map messages. It adapts `/map` transient-local data into `SourceMap`, invokes core operations/persistence, renders masks/outlines/unassigned cells, and prominently shows clipping/preemption and validation reports. GUI remains manual acceptance only per docs.

### Affected files
- Modify: `source_map.py`, `segmentation.py`, `regions.py`, `validation.py`; likely add a dedicated pure-core constraints module and persistence module rather than placing geometry/file I/O in the UI.
- Modify/add tests: `test_regions.py`, `test_validation.py`, plus focused constraint and persistence test modules; extend `test_map_io.py` only for shared coordinate/file-format helpers if used.
- Add: `src/oomwoo_cleaning_jobs_ui/{package.xml,setup.py,setup.cfg,resource/,oomwoo_cleaning_jobs_ui/}` and a manual GUI acceptance checklist.
- Update: core `package.xml`/`setup.py` dependency metadata and `docs/DEVELOPMENT.md`; package documentation currently claims persistence is present although no corresponding module exists.

## Review Findings

1. **High — missing core implementation:** no constraints, persistence, or UI package exists in tracked `src/`; the only current package is core. This blocks both approved milestones. Evidence: complete tracked-file inventory and `package.xml:3-22`.
2. **High — existing regions will not be automatically clipped when a constraint is added:** current clipping only occurs during `paint/create` (`regions.py:127-147`); changing `cleanable` alone cannot remove pre-existing labels. Without one atomic constraint-application operation, validation will report `region_in_keepout` rather than preserving the documented immediate-clipping invariant.
3. **High — candidate generation ignores constraints:** `segment()` calls `source_map.free_mask()` directly (`segmentation.py:136-140`), so initial candidate regions include Keepout cells unless the segmentation input seam is extended.
4. **High — nonzero map yaw is a geometry risk:** `SourceMap.origin` stores yaw (`source_map.py:47, 60`) and identity includes its quaternion (`lines 66-74`), but `RegionSet.outline()` converts grid points with only x/y translation (`regions.py:101-123`). Any Keepout/Virtual Wall map-frame rasterizer/UI conversion must apply yaw; otherwise geometry will be shifted/rotated incorrectly on valid maps.
5. **Medium — runtime dependency metadata is incomplete:** core imports `scipy.ndimage` in regions, segmentation, and validation, but `package.xml:14-18` lacks `python3-scipy`. The new UI package must explicitly declare PyQt5, rclpy, nav_msgs, and a dependency on core; none exists today.
6. **Medium — mask input lacks defensive validation:** `validate_region_set` accepts arbitrary `keepout_mask` and immediately combines it with region masks (`validation.py:109-115`), with no shape/type check. Persistence/load and UI adapter boundaries should reject a mask that does not match the source map before validation.
7. **Documentation debt (confirmed direction):** `docs/DEVELOPMENT.md:53,67` and `package.xml:7-9` state persistence is already implemented, but verified source contains no persistence implementation. Supervisor confirmed source is authoritative and this wording should be corrected with the implementation.

## Test Strategy

- Keep all existing 63 headless tests as regression; current run passed.
- New core tests should use the existing synthetic maps and assert: Keepout subtraction from segmentation and `RegionSet.cleanable`; both polygon and line-wall raster cell expectations; existing labels removed on constraint add; an edit fully in keepout is no-op; preemption still works outside it; constraint removal restores only cleanability (not intentionally removed labels); errors/warnings retain their documented levels; persistence reload rejects mismatched identity, bad dimensions, and overlapping PNG masks.
- UI is outside CI: manually open file and `/map` input, verify transient-local reception, bottom-row/map-frame alignment with a nonzero origin/yaw fixture, create/edit/delete constraints, visible clipped/preempted region changes, warning/error presentation, draft reload, and publish refusal/acceptance.

## Start Here

Open `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/regions.py` first. Its `cleanable` field and paint/create operations are the narrowest existing seam for correct constraints-minus-Cleanable-Space behavior; then update segmentation to consume the same effective mask.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Concrete severity-tagged findings cite docs/DEVELOPMENT.md and source/test/package paths with line ranges."
    }
  ],
  "changedFiles": [
    "artifacts/phase-one-scouts.constraints-ui-seam.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "cd src/oomwoo_cleaning_jobs_core && pytest -q",
      "result": "passed",
      "summary": "63 passed in 7.21s"
    },
    {
      "command": "git status --short && git branch --show-current && git log -1 --oneline && find src -maxdepth 2 -type d -print",
      "result": "passed",
      "summary": "Confirmed main at 8eb3530 and only the core package is present; .pi is pre-existing untracked runtime state."
    }
  ],
  "validationOutput": [
    "63 passed in 7.21s"
  ],
  "residualRisks": [
    "Persistence format/API choices were explicitly out of scope; implementation must honor the documented on-disk layout without treating this scout as an API decision.",
    "Constraint geometry must define and test nonzero-yaw map-frame conversion before UI use.",
    "Nav2 costmap/coverage projection remains an explicitly deferred boundary in docs/DEVELOPMENT.md:150."
  ],
  "noStagedFiles": true,
  "diffSummary": "No product-code changes; wrote the required scouting artifact only.",
  "reviewFindings": [
    "high: regions.py:127-147 - constraint addition has no operation to clip pre-existing region labels.",
    "high: segmentation.py:136-140 - segmentation reads SourceMap.free_mask directly and cannot subtract constraints.",
    "high: src/ contains no constraint, persistence, or UI package implementation.",
    "medium: package.xml:14-18 - scipy is imported but undeclared; new UI dependencies are absent."
  ],
  "manualNotes": "Authoritative documentation/source conflict on persistence was escalated. Supervisor confirmed verified source is authoritative; record the conflicting documentation as debt rather than block implementation."
}
```