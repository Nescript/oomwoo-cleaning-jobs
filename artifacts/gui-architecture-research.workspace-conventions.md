# Code Context

## Files Retrieved
1. `docs/DEVELOPMENT.md` (lines 25-27, 47-55) — Phase-1 scope and the explicitly confirmed package/UI architecture.
2. `src/oomwoo_cleaning_jobs_core/package.xml` (lines 4-23) — current repository package is an `ament_python` core library with no ROS, Qt, or rqt dependency.
3. `src/oomwoo_cleaning_jobs_core/setup.py` (lines 10-26) — current installation convention: ament-index resource/package metadata and a console script; no launch/config assets yet.
4. `/ros_ws/src/oomwoo-ros2-tools/src/oomwoo_clean_ui/package.xml` (lines 4-25) — workspace precedent that separates visualization tooling and its RViz dependency from robot runtime.
5. `/ros_ws/src/oomwoo-ros2-tools/src/oomwoo_clean_ui/setup.py` (lines 20-45) — ament-Python UI-tool package installs `launch/*.launch.py` and visualization config into package share.
6. `/ros_ws/src/oomwoo-ros2-tools/src/oomwoo_clean_ui/launch/cleaning_debug.launch.py` (lines 15-49) — debug visualization is launched as a separate `rviz2` process, with a `use_sim_time` launch argument.
7. `/ros_ws/src/oomwoo-ros2-tools/src/oomwoo_bringup/launch/monitor_robot.launch.py` (lines 39-58, 69-84) — general workspace visualization launch convention resolves installed share assets and starts RViz as a `launch_ros.actions.Node`.

## Key Code

- **Decisive recorded Phase-1 decision — blocker to choosing rqt without reconfirmation**: `docs/DEVELOPMENT.md:47-55` says these decisions were confirmed and must be reconfirmed before modification; it specifies:

  ```text
  src/oomwoo_cleaning_jobs_ui: PyQt5 standalone app + rclpy node, thin adapter.
  ```

  The same document calls the validation GUI a “PyQt5 standalone application,” not the final control app (`docs/DEVELOPMENT.md:25-27`). It also requires runtime `/map` subscription with transient-local QoS *or* loading map files (`docs/DEVELOPMENT.md:25`), which fits a thin rclpy adapter without rqt hosting.

- **Current packaging constraint — medium**: only `oomwoo_cleaning_jobs_core/` exists under this repository’s `src/` directory; it is deliberately ROS-free (`package.xml:6-9`) and declares only numerical/image/YAML runtime dependencies (`package.xml:14-17`). Therefore the UI must be a separate package as documented, preserving core headless testability.

- **No local rqt precedent — medium**: exhaustive inspection of all workspace `package.xml`, `setup.py`, `setup.cfg`, `CMakeLists.txt`, and `plugin.xml` files found no `rqt`, `qt_gui`, `python_qt_binding`, or `PyQt5` reference. No `plugin.xml` files or Qt Designer `*.ui` files exist anywhere under `/ros_ws/src`. There is consequently no local rqt plugin manifest/export/install template to reuse.

- **Existing GUI/visualization precedent**: the workspace does have RViz tooling, but it is a separate `ament_python` package. `oomwoo_clean_ui` explicitly keeps UI/RViz dependencies off robot runtime (`package.xml:6-16`), installs launch/config assets (`setup.py:26-34`), and launches standalone `rviz2` (`cleaning_debug.launch.py:36-49`). This supports making the Phase-1 GUI separately deployable rather than embedding it in a robot package.

## Architecture

`oomwoo_cleaning_jobs_core` remains a pure, headless domain package. The planned `oomwoo_cleaning_jobs_ui` package should depend on PyQt5, `rclpy`, ROS message packages, and any needed `ament_index_python`/launch dependencies; it should import the core package and provide the runtime `/map` adapter or file-open path. If a launch entry point is desired, follow the local ament-Python convention: install `launch/*.launch.py` and launch the UI as a separate process, while keeping it out of robot-runtime dependencies.

**Recommendation implication:** choose the documented **standalone PyQt5 + rclpy app**, not an rqt plugin. This is not merely an absence-of-precedent choice: it is the repository’s recorded, confirmed Phase-1 decision. An rqt plugin would add unproven workspace dependencies/packaging (`rqt_gui_py`, plugin export/manifest, `python_qt_binding`) and contradict the documented architecture unless that decision is explicitly reconfirmed.

## Start Here

Open `docs/DEVELOPMENT.md` at lines 25-27 and 47-55 first. It directly fixes the Phase-1 GUI shape and separation from the pure core; package work should then mirror the separate `oomwoo_clean_ui` package’s installation pattern rather than its RViz-specific dependency set.

## Residual Risks

- **Medium:** Local source evidence cannot prove that `rqt` or PyQt5 is installed in the base ROS distribution; it only proves neither is declared nor patterned in this workspace. The implementer must validate target-image/system dependencies before packaging the new UI.
- **Low:** The planned `oomwoo_cleaning_jobs_ui` package does not yet exist (`/ros_ws/src/oomwoo-cleaning-jobs/src` contains only the core package), so its exact executable/launch interface and dependency declarations remain to be implemented.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Concrete evidence and severity are cited from docs/DEVELOPMENT.md, core package metadata, and existing workspace visualization packages."
    }
  ],
  "changedFiles": [
    "artifacts/gui-architecture-research.workspace-conventions.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "find/grep metadata under /ros_ws/src for rqt, qt_gui, python_qt_binding, PyQt5, and plugin.xml",
      "result": "passed",
      "summary": "No rqt/Qt dependency or plugin manifest found in the local workspace."
    },
    {
      "command": "read package metadata and visualization launch files",
      "result": "passed",
      "summary": "Confirmed separate ament-Python RViz tooling and installed launch/config convention."
    }
  ],
  "validationOutput": [
    "Findings artifact written at the required path; git diff --check passed.",
    "Read-only source inspection found no existing rqt plugin pattern.",
    "git status showed existing unrelated modified/untracked repository files; no staged files were reported."
  ],
  "residualRisks": [
    "Target image/base ROS availability of PyQt5/rqt was not verifiable from workspace source alone.",
    "The planned UI package does not yet exist, so its final launch/executable contract is not established."
  ],
  "noStagedFiles": true,
  "diffSummary": "Created the requested workspace-conventions research artifact; unrelated pre-existing worktree changes were not modified.",
  "reviewFindings": [
    "blocker: docs/DEVELOPMENT.md:49-55 - rqt would contradict the recorded confirmed Phase-1 standalone PyQt5+rclpy architecture unless reconfirmed.",
    "medium: /ros_ws/src - no rqt/Qt plugin dependency or plugin.xml precedent exists in inspected workspace metadata.",
    "low: /ros_ws/src/oomwoo-cleaning-jobs/src - planned oomwoo_cleaning_jobs_ui package is absent."
  ],
  "manualNotes": "No implementation files were changed by this task. git status contains unrelated pre-existing modified and untracked files; the index is empty."
}
```
