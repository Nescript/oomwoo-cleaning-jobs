# Code Context

## Files Retrieved
1. `docs/DEVELOPMENT.md` (lines 60-67, 90-118) — authoritative identity, map-image convention, persistence layout, validation, and test requirements.
2. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/source_map.py` (lines 33-100) — immutable map shape, origin, raw cells, and full SHA-256 storage key.
3. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/map_io.py` (lines 29-84) — Nav2 trinary reader and the only existing map-file serialization convention.
4. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/regions.py` (lines 32-100, 119-213) — `RegionSet` is the editable draft representation; labels are authoritative in memory and names are keyed by label.
5. `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/validation.py` (lines 38-139) — publish gate and loader-specific `check_masks_overlap` hook.
6. `src/oomwoo_cleaning_jobs_core/test/fixtures.py` (lines 1-249) — deterministic `SourceMap` factories and `write_map_files` round-trip helper.
7. `src/oomwoo_cleaning_jobs_core/test/test_regions.py` (lines 1-167) and `test/test_validation.py` (lines 1-128) — behavioral expectations for draft edits and validation.
8. `src/oomwoo_cleaning_jobs_core/setup.py` (lines 1-26) and `package.xml` (lines 10-22) — Python packaging/dependency declarations.

## Key Code

### Smallest correct seam
Add one pure-core module: `oomwoo_cleaning_jobs_core/region_set_store.py` (do **not** put disk I/O in `regions.py`, which is the mutable editing/domain model). It owns only layout, serialization/deserialization, integrity checks, and publication. It depends on `SourceMap`, `RegionSet`, `validate_region_set`, and `check_masks_overlap`; no ROS/UI dependency.

Proposed public boundary:

```python
class RegionSetStore:
    def __init__(self, root: Path | None = None) -> None: ...
    # root defaults to Path.home()/".local/share/oomwoo_cleaning_jobs/maps"

    def save_map_snapshot(self, source_map: SourceMap) -> Path: ...
    def save_draft(self, source_map: SourceMap, region_set: RegionSet) -> None: ...
    def load_draft(self, source_map: SourceMap) -> RegionSet | None: ...
    def load_published(self, source_map: SourceMap) -> PublishedRegionSet | None: ...
    def publish(self, source_map: SourceMap, *, robot_inscribed_radius: float = ...)
        -> PublishedRegionSet: ...

@dataclass(frozen=True)
class PublishedRegionSet:
    region_set: RegionSet
    version: int
    published_at: datetime
```

`publish()` must load/validate the persisted draft, reject a report with errors, then create the next published payload atomically; warnings remain in the returned `ValidationReport` or result object. Do not silently revalidate an unrelated caller-owned mutable `RegionSet`. Add a domain exception such as `RegionSetPersistenceError` for malformed/incompatible disk data and `PublishValidationError(report)` for a blocked publish.

The map directory is deterministically `root / source_map.identity`, not `short_id`: `SourceMap.identity` is full SHA-256 and is explicitly the storage key (`source_map.py:63-75`; docs lines 60-61). Before loading or saving a set, require the set's resolution/origin/shape to equal its `SourceMap`; map hash alone is not enough protection against manually copied files.

### On-disk contract to make explicit
- Snapshot: generate canonical `map_snapshot.pgm` and `map_snapshot.yaml` using the documented Nav2 saver pixel/threshold convention and row inversion (docs lines 63-65; fixture `write_map_files`).
- `draft/regions.yaml`: include a format/schema version, full `map_hash`, width, height, resolution, origin, and a deterministic ordered region list with `label`, `name`, and `mask: masks/<stable filename>.png`. Metadata must retain integer labels; deriving labels from filenames changes the current API semantics.
- `draft/masks/*.png`: one mask per region, exact `(height, width)` cell array, with row 0 serialized in the same bottom-to-top convention used by `SourceMap` (or explicitly invert consistently on both write/read). Use an unambiguous binary convention (e.g. 0=false, 255=true).
- `draft/constraints.yaml`: create an explicitly empty, versioned placeholder now. Keepout geometry is out of this implementation slice, but docs require the file and current validation already accepts an optional `keepout_mask` (`validation.py:64-68`).
- `published/`: same payload plus `version` (strictly increasing per map) and UTC timestamp in `regions.yaml`, as docs require (lines 104-110). A draft has no publish version/time.

On load, rebuild labels from PNGs, reject unknown fields only if a schema policy is adopted, and always verify: YAML map metadata equals `SourceMap`; each PNG is binary and correct shape; listed masks exist with no unlisted files policy defined; labels are positive and unique; `check_masks_overlap(masks)` has no errors; then `validate_region_set(...)` governs domain validity. This directly uses the loader seam called out in `validation.py:129-139`.

### Atomic publish constraint
**Severity: blocker for implementation design.** Replacing an existing non-empty `published/` directory cannot be a single `os.replace` operation on normal POSIX filesystems. A temp directory followed by `published -> backup`, `temp -> published` has a crash window with no published set, so it does not satisfy an unqualified atomic-publish requirement.

Smallest robust representation: store immutable complete generations under an internal sibling (for example `.published-generations/<uuid>/`) and atomically replace a small `published-current.yaml` manifest/pointer containing the selected generation, version, timestamp, and map hash. Expose `published/` only if a symlink-based layout is approved; atomically replace the symlink, not the non-empty directory. This is a compatibility/layout decision because docs literally call for `published/` “同构” (isomorphic). If the literal directory is non-negotiable, define atomicity as crash-safe recovery (journal plus recovery) rather than POSIX single-step atomic replacement. Temporary writes need file `fsync`, directory `fsync`, and atomic manifest replacement; clean old generations only after a successful pointer swap.

### Dependencies
PyYAML and OpenCV are already declared in `package.xml:14-16` and used by `map_io.py`; `setup.py:15` currently declares only `setuptools`, so Python package metadata is already incomplete relative to runtime imports. Standard library `pathlib`, `os`, `tempfile`, `shutil`, and `datetime` suffice for YAML, atomic files, and metadata.

**Severity: major.** The required “1-bit PNG” is not guaranteed by `cv2.imwrite` for a boolean/uint8 binary image; it commonly emits 8-bit grayscale. Add Pillow (`python3-pil` and matching Python install requirement) and write `Image.fromarray(mask).convert("1")`, then test PNG IHDR bit depth is 1. If an 8-bit visually binary PNG is acceptable, amend the requirement instead.

## Architecture

Current flow is `SourceMap` -> `free_mask()` -> segmentation -> `RegionSet.from_segmentation()` -> mutable region edits -> `validate_region_set()`. `RegionSet` deliberately has no map identity and only carries labels, cleanable mask, resolution, origin, and names (`regions.py:40-77`); the store must join it to the supplied `SourceMap` at its public boundary. `SourceMap.cells` is raw int8, so canonical snapshot output can preserve identity-relevant data exactly using the existing fixture's saver convention. No persistence module or disk layout implementation exists: targeted source search found no `regions.yaml`, `constraints.yaml`, publish, atomic, temp-file, or persistence symbols.

There is no source contradiction with docs. The only unresolved design point exposed by the docs is the meaning of “atomic publish” while simultaneously requiring a replaceable, non-empty `published/` directory.

## Test fixture approach

Create `test/test_region_set_store.py`, using `tmp_path` as `RegionSetStore(root=tmp_path / "maps")`, never the real home directory.

1. Build with `make_two_rooms_map()` and `_make_region_set()` patterns from `test_regions.py`; save snapshot and draft; load it; assert `SourceMap` metadata, labels, names, cleanable mask, and each hole/disconnected component round-trip exactly.
2. Use a custom single-region boolean mask with a hole and disconnected island (not only segmentation output) to prove PNG orientation and per-mask reconstruction.
3. Assert expected exact tree: `<root>/<full identity>/map_snapshot.{yaml,pgm}`, draft `regions.yaml`, `constraints.yaml`, and named masks; load snapshot through `load_map_file` and assert equality.
4. Parametrize corrupt artifacts: wrong map hash/shape/origin, absent/extra mask, non-binary pixels, duplicate label, overlapping PNGs. Assert persistence exception and verify `check_masks_overlap` is exercised.
5. Publish clean draft; assert version 1/time; mutate and republish; assert version 2, exactly one selected published generation, and old published remains readable if publish is intentionally failed. Force a validation error using the existing wall-cell mutation from `test_validation.py` and assert no pointer/current published changes.
6. Simulate interrupted staging at every pre-commit failure point (mock writer/rename); assert `load_published` returns the prior complete version, never a partial tree. Test IHDR bit depth if 1-bit is retained as a hard requirement.

## Start Here
Open `src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/regions.py` first: its `RegionSet` constructor and label/name model determine the serializer's lossless contract. Then implement the isolated store module and tests; do not alter editing semantics.

## Supervisor coordination
No escalation sent: source agrees with the authoritative design. The atomic-directory and exact 1-bit-PNG points are implementation constraints requiring an explicit design choice before coding.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Concrete persistence seam, exact file paths/line ranges, severity-tagged blockers, tests, and compatibility risks are documented above."
    }
  ],
  "changedFiles": [
    "artifacts/phase-one-scouts.persistence-seam.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "cd src/oomwoo_cleaning_jobs_core && pytest -q",
      "result": "passed",
      "summary": "63 passed in 7.24s"
    }
  ],
  "validationOutput": [
    "Targeted persistence-symbol search found no existing persistence implementation.",
    "Existing core test suite: 63 passed."
  ],
  "residualRisks": [
    "Atomic replacement of a non-empty published directory is not a single POSIX rename; layout/atomicity semantics must be selected.",
    "OpenCV does not guarantee required 1-bit PNG output; add Pillow or relax the on-disk requirement.",
    "SourceMap lacks original map-file provenance, so snapshot must be canonical regenerated output unless original-byte preservation is specified."
  ],
  "noStagedFiles": true,
  "diffSummary": "No source edits; wrote required scout artifact only.",
  "reviewFindings": [
    "blocker: docs/DEVELOPMENT.md:106-110 - literal non-empty published/ layout conflicts with single-step atomic replacement on POSIX.",
    "major: src/oomwoo_cleaning_jobs_core/setup.py:15 and package.xml:14-16 - setup.py omits existing runtime dependencies and no guaranteed 1-bit PNG writer is declared.",
    "no source/docs contradiction found."
  ],
  "manualNotes": "Repository HEAD was main at 8eb3530; pre-existing untracked .pi/ was present."
}
```