# Final review fixes

Applied accepted UI review fixes only.

- Both file-open and live `/map` now call `Window.replace_source()`: equal identities retain the existing editing session; a different identity requires explicit Yes before calling `EditorController.set_source()`.
- Added `RegionSetStore.other_map_set_count()`. When a current map has no draft, `EditorController.set_source()` reports the exact count of other identity directories with draft/published data.
- Added `python3-numpy` UI runtime dependency.
- Added a focused controller test for the other-map-set message.

Validation:
- Focused UI tests: 4 passed.
- Python UI source compilation: passed.
- `colcon build`: 2 packages passed.
- `colcon test`: core 77 + UI 4 = 81 passed, 0 failures.

```acceptance-report
{
  "criteriaSatisfied": [
    {"id":"criterion-1","status":"satisfied","evidence":"Only source-replacement guard, identity-directory count, dependency declaration, and focused test were changed."},
    {"id":"criterion-2","status":"satisfied","evidence":"Focused pytest, source compilation, colcon build, and colcon test all passed."}
  ],
  "changedFiles": ["src/oomwoo_cleaning_jobs_ui/oomwoo_cleaning_jobs_ui/app.py", "src/oomwoo_cleaning_jobs_ui/oomwoo_cleaning_jobs_ui/controller.py", "src/oomwoo_cleaning_jobs_core/oomwoo_cleaning_jobs_core/persistence.py", "src/oomwoo_cleaning_jobs_ui/test/test_controller.py", "src/oomwoo_cleaning_jobs_ui/package.xml", "docs/DEVELOPMENT.md"],
  "testsAddedOrUpdated": ["src/oomwoo_cleaning_jobs_ui/test/test_controller.py"],
  "commandsRun": [
    {"command":"PYTHONPATH=src/oomwoo_cleaning_jobs_core:src/oomwoo_cleaning_jobs_ui pytest -q src/oomwoo_cleaning_jobs_ui/test","result":"passed","summary":"4 passed"},
    {"command":"python3 -m py_compile src/oomwoo_cleaning_jobs_ui/oomwoo_cleaning_jobs_ui/*.py","result":"passed","summary":"passed"},
    {"command":"colcon build --packages-select oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui --event-handlers console_direct+","result":"passed","summary":"2 packages finished"},
    {"command":"colcon test --packages-select oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui --event-handlers console_direct+ && colcon test-result --verbose","result":"passed","summary":"81 tests, 0 errors, 0 failures"}
  ],
  "validationOutput": ["Focused and colcon test suites passed."],
  "residualRisks": ["Live ROS flow remains subject to the manual transient-local /map checklist."],
  "noStagedFiles": true,
  "diffSummary":"Applied final review fixes for guarded replacement, other-map-set visibility, and numpy packaging.",
  "reviewFindings": ["no blockers"],
  "manualNotes":"Run the GUI manual acceptance checklist before release."
}
```