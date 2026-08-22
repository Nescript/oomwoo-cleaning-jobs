# Research: Official ROS/rqt facts for a map-region editor

## Summary
For an editor intended to be launched alongside other ROS diagnostic/operator tools, an rqt Python plugin is the ROS-supported integrated GUI route: rqt hosts tools as dockable plugins, while `qt_gui` accepts arbitrary Qt widgets from Python or C++ plugins. A standalone PyQt5 + `rclpy` program is the lower-framework route, but the application—not rqt—then owns Qt application startup, ROS context initialization/spinning, node destruction, and shutdown.

**Recommendation for this repository:** choose an rqt plugin if the map-region editor is primarily an operator/developer tool that benefits from rqt's plugin menu, docking, shared layout, and co-location with other ROS tools. Choose standalone only if the product must be a dedicated end-user application with its own window/session and deployment UX; in that case keep ROS/Qt lifecycle integration explicit and tested. This is an architectural implication, not a claim that either option is already implemented here.

## Findings
1. **rqt is a plugin host with both integrated and standalone-window modes.** The ROS 2 rqt overview says rqt implements tools/interfaces as plugins, runs existing GUI tools as dockable windows within rqt, and can also run plugins in standalone windows. Thus “rqt plugin” means integration with an rqt host rather than an inherently non-windowed UI. [ROS 2 rqt overview](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-RQt.html)

2. **Embedding a `QWidget` is the intended `qt_gui` extensibility model.** The official `qt_gui` package documentation describes an integrated Qt GUI extensible by Python- and C++-based plugins that can contribute *arbitrary widgets*. A map canvas/editor `QWidget` therefore fits the framework model; rqt provides the surrounding docking/container behavior. [qt_gui Jazzy documentation](https://docs.ros.org/en/jazzy/p/qt_gui/)

3. **Python-plugin lifecycle has explicit cleanup and settings hooks.** The official `qt_gui_core` source defines the Python `Plugin` interface: a user plugin receives a `PluginContext` at construction; its `shutdown_plugin()` hook is for cleanup before unload; and it exposes `save_settings(plugin_settings, instance_settings)` and `restore_settings(plugin_settings, instance_settings)`. An rqt implementation should put subscriptions/timers/executor-related cleanup in `shutdown_plugin()` and use the settings hooks for editor/UI state where appropriate. [Plugin interface source](https://github.com/ros-visualization/qt_gui_core/blob/1eaff5a7586e78ec4f5bf501d9a3172a5da07b06/qt_gui/src/qt_gui/plugin.py)

4. **A ROS 2 Python rqt plugin uses rqt's ROS-aware Python layer.** The official `rqt_gui_py` documentation states that the package enables GUI plugins to use the Python client library for ROS. The `rqt_gui_py` release history also records ROS 2-specific work to spin an executor in a separate `QThread` and to pass an rqt node through a plugin-context subclass. This supports treating the host-provided rqt context/node as part of plugin integration, rather than blindly duplicating a standalone `rclpy.init()`/spin loop. Exact use of the node/context must be verified against the target ROS distribution/API before coding. [rqt_gui_py Jazzy documentation](https://docs.ros.org/en/jazzy/p/rqt_gui_py/) · [official ROS 2 port changelog](https://github.com/ros-visualization/rqt/blob/jazzy/rqt_gui_py/CHANGELOG.rst)

5. **Plugin discovery/deployment adds metadata and host dependencies.** rqt's Python provider itself is packaged with `plugin.xml`; its official repository package directory contains both `plugin.xml` and `package.xml`, and the ROS 2 changelog notes a required installed location of `${prefix}/plugin.xml`. In practice, an rqt plugin package must install its plugin description and declare/register it in package metadata so rqt can discover it; this is extra packaging work and creates runtime dependence on rqt/qt_gui/rqt_gui_py in addition to ROS/Python/Qt. [rqt_gui_py package, Jazzy](https://github.com/ros-visualization/rqt/tree/jazzy/rqt_gui_py) · [official issue documenting installed `plugin.xml` location](https://github.com/ros-visualization/rqt/issues/179)

6. **Standalone `rclpy` lifecycle is application-owned.** Official rclpy docs require `rclpy.init()` before creating ROS nodes, require spinning (`spin`, `spin_once`, or `spin_until_future_complete`) to execute pending callbacks, and require shutdown after using the initialized context/nodes. A standalone editor must arrange this alongside the Qt event loop (for example, a deliberately selected executor/thread or a safe periodic integration), then destroy/stop it at application exit. [rclpy initialization, shutdown, and spinning](https://docs.ros.org/en/jazzy/p/rclpy/api/init_shutdown.html)

7. **Direct `PyQt5` is a portability/dependency choice, not the documented rqt abstraction.** `qt_gui` requires either PyQt or PySide bindings, and the ROS-maintained `python_qt_binding` package is the Qt-binding abstraction with PySide and PyQt providers. For an rqt plugin, prefer `python_qt_binding` imports unless the repository intentionally pins PyQt5 and accepts coupling to that binding. For standalone, direct PyQt5 is viable but makes that binding an explicit application dependency. [qt_gui documentation](https://docs.ros.org/en/jazzy/p/qt_gui/) · [python_qt_binding documentation](https://docs.ros.org/en/jazzy/p/python_qt_binding/)

## Comparison

| Dimension | rqt Python plugin | Standalone PyQt5 + rclpy application |
|---|---|---|
| Lifecycle owner | rqt/`qt_gui` constructs plugin with a context and calls plugin lifecycle/settings hooks. Plugin must release its own resources at unload. | Application owns Qt event loop and all documented `rclpy` init, callback processing, node/executor teardown, and shutdown. |
| Discovery | Host-discoverable plugin metadata (`plugin.xml` plus package installation/registration) is required. | No rqt discovery; launch via the package's normal executable/launch/deployment mechanism. |
| Runtime dependencies | ROS 2 plus rqt/qt_gui/rqt GUI Python integration and a supported Qt binding. | ROS 2 Python client library plus explicitly selected Qt binding (PyQt5 per option); no rqt host required. |
| User deployment/launch | User installs/sources the package in an ROS environment and launches rqt; the editor appears in rqt's available-plugin UI and can dock with other tools. | User launches one dedicated executable; packaging must provide any icons, config, desktop integration, and application-specific UX desired. |
| Map editor widget | Plugin constructs/contributes the editor widget to rqt. This is supported because `qt_gui` plugins can contribute arbitrary widgets. | Main window/application constructs and owns the widget directly. |
| Best fit | Technician/developer tool, workflow alongside rosgraph/topic/service tools, shared rqt layout. | Dedicated operator product, branded/controlled UX, no requirement to install or use rqt. |

## Repository implications

1. **Keep the map-editor UI separable from the host.** Put geometry/model, map-coordinate transforms, region validation, persistence, and ROS-facing service/topic logic behind a UI-independent boundary. Then the same editor `QWidget` can be inserted into an rqt plugin or a standalone `QMainWindow` without duplicating domain code.
2. **If selecting rqt:** add an rqt plugin class that receives the plugin context, creates the editor `QWidget`, registers it with the context, implements deterministic `shutdown_plugin()` cleanup, and packages/installs `plugin.xml`. Use `python_qt_binding` unless a project-level binding decision justifies PyQt5-only imports.
3. **If selecting standalone:** write one explicit ownership plan for `QApplication`, `rclpy` context, node, executor/callback processing, and exit order. Do not call blocking `rclpy.spin()` on the GUI thread; the official docs establish that spinning executes callbacks but do not prescribe a Qt integration pattern.
4. **Do not infer ROS managed-node lifecycle semantics.** `rclpy.lifecycle` documents lifecycle-node APIs, but neither rqt nor a GUI automatically makes the editor a managed lifecycle node. Adopt that additional model only if the editor itself must participate in managed-node transitions.

## Limitations / uncertainties

- This research deliberately uses only official ROS documentation and the official `ros-visualization` source repository. No third-party tutorials, Q&A, or unverified examples were used.
- The exact `plugin.xml` schema/export stanza and the exact rqt-provided ROS node/context API should be checked against the **target ROS distribution** before implementation. The evidence above spans Rolling (rqt overview) and Jazzy (package/API docs); APIs and package versions can differ by distro.
- Official sources establish that plugins contribute arbitrary widgets and provide lifecycle hooks, but do **not** prescribe a single map-rendering choice, ROS executor/Qt event-loop integration design, or persistence schema.
- No repository source was inspected or changed as part of this documentation-only task; therefore dependency availability, package type, current GUI code, and target ROS distribution remain unverified.

## Sources

### Kept (primary)
- [Overview and usage of RQt — ROS 2 Documentation (Rolling)](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-RQt.html) — official description of rqt's dockable plugin host and standalone plugin windows.
- [qt_gui — Jazzy documentation](https://docs.ros.org/en/jazzy/p/qt_gui/) — official statement that Python/C++ plugins may contribute arbitrary widgets and required Qt bindings.
- [rqt_gui_py — Jazzy documentation](https://docs.ros.org/en/jazzy/p/rqt_gui_py/) — official scope of the ROS-aware Python plugin layer.
- [qt_gui Python Plugin interface — official source](https://github.com/ros-visualization/qt_gui_core/blob/1eaff5a7586e78ec4f5bf501d9a3172a5da07b06/qt_gui/src/qt_gui/plugin.py) — direct lifecycle/settings interface evidence.
- [rqt_gui_py ROS 2 changelog — official source](https://github.com/ros-visualization/rqt/blob/jazzy/rqt_gui_py/CHANGELOG.rst) — direct evidence of ROS 2 executor/thread and node-context integration changes.
- [rqt_gui_py package directory — official source](https://github.com/ros-visualization/rqt/tree/jazzy/rqt_gui_py) — primary packaging evidence for `plugin.xml`/`package.xml`.
- [Official rqt issue #179](https://github.com/ros-visualization/rqt/issues/179) — maintainer-recorded expected installed location for `plugin.xml`.
- [rclpy initialization, shutdown, and spinning — Jazzy documentation](https://docs.ros.org/en/jazzy/p/rclpy/api/init_shutdown.html) — official standalone ROS lifecycle facts.
- [python_qt_binding — Jazzy documentation](https://docs.ros.org/en/jazzy/p/python_qt_binding/) — ROS-maintained Qt binding abstraction facts.

### Dropped
- Third-party rqt plugin tutorials, Stack Overflow answers, and general PyQt/ROS blog posts — excluded by the primary-sources-only constraint.

## Gaps

Before implementation, inspect this repository's `package.xml`, build type, target ROS distribution, launch conventions, and existing Qt/ROS dependencies. Then validate a minimal plugin in the target distro: it must appear in rqt discovery, embed its widget, receive shutdown cleanly, and not block the GUI while ROS callbacks are processed.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Concrete official-source findings, comparison, limitations, repository implications, and source URLs are recorded in artifacts/gui-architecture-research.official-rqt-facts.md; no implementation-file finding was applicable because this was a no-local-edits research task."
    }
  ],
  "changedFiles": [
    "artifacts/gui-architecture-research.official-rqt-facts.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [],
  "validationOutput": [
    "Artifact written to the authoritative requested path.",
    "All cited evidence is from docs.ros.org or official ros-visualization GitHub repositories; third-party sources were excluded."
  ],
  "residualRisks": [
    "Target ROS distribution and its exact plugin metadata/context API were not supplied; validate against that distribution before implementation.",
    "Repository dependencies and existing GUI architecture were intentionally not inspected under the no-local-edits research scope."
  ],
  "noStagedFiles": true,
  "diffSummary": "Added the requested documentation research artifact only; no application source was modified.",
  "reviewFindings": [
    "info: artifacts/gui-architecture-research.official-rqt-facts.md - documentation-only research artifact; no application code was reviewed or changed, so no blocker/major/minor code findings apply."
  ],
  "manualNotes": "Primary sources only. The artifact distinguishes documented facts from implementation implications and calls out cross-distribution uncertainty."
}
```