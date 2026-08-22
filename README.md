# oomwoo-cleaning-jobs

已保存地图上的用户清洁意图与长任务编排（ROS 2 Jazzy）。

**开发上下文与设计决定：[docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)**（唯一事实来源，改动前先读）。

## 阶段一交付

`已保存地图 → 自动候选区域 → 手动编辑 → 校验 → Published Region Set 持久化`

- `src/oomwoo_cleaning_jobs_core`：纯 Python 核心库（零 ROS 依赖）——地图加载与 identity、
  自动分割（maximin 淹没分水岭 + 合并树鞍部合并 + 门口溢出裁剪 + 门口拓扑）、
  Region 掩码编辑、Keepout/Virtual Wall、校验分级、draft/published 持久化
- `src/oomwoo_cleaning_jobs_ui`：PyQt5 区域编辑器（文件 / `/map` 双来源）

## 快速开始

```bash
cd /ros_ws && colcon build --packages-select oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui
colcon test --packages-select oomwoo_cleaning_jobs_core oomwoo_cleaning_jobs_ui && colcon test-result

# 地图渲染/分割 CLI
ros2 run oomwoo_cleaning_jobs_core oomwoo-render-map <map.yaml> --segment

# 区域编辑器 GUI
ros2 run oomwoo_cleaning_jobs_ui oomwoo-cleaning-jobs-ui
```

演示图：`docs/demo/`（分割 + 门口标记）、`docs/demo/doorway/`（对比试验）。
GUI 手动验收：`src/oomwoo_cleaning_jobs_ui/docs/MANUAL_ACCEPTANCE.md`。

## 许可

Apache 2.0
