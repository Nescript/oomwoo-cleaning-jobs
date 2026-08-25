# ROSE2 docs 地图修复后验证

Provider：`rose2 upstream-3a010b9e6bb2+oomwoo.4`（历史 wrapper 记录；当前分支的同名场景已由 `oomwoo_segmentation` 原生引擎重新验证，见 `output/demo/`、`output/ipa/`、`output/rose2_upstream/`）

## 本次修复

- 修正竖直线分支在 `b == 0` 时先读取未初始化 `Y1` 的问题。
- 外轮廓改为最大连通自由空间轮廓，不再固定访问第二个非自由空间 contour；支持无 unknown 外围的裁剪地图，也避免把家具岛误选为房间外边界。
- 在构造 Shapely Polygon 前过滤少于三个不同顶点的退化 cell。
- 去除与四条 synthetic frame lines 精确重合的检测线，避免 `create_cells()` 选择重复边并生成左下角三角形 cell。
- 默认 `lines_threshold=0.22`，过滤由家具 Hough segments 生成的低支持度全图延伸线。
- 仅为零权重 retained edges 从 ROSE structural raster 补算局部墙体支持，恢复被概率 Hough 遗漏的墙段。
- 严格排除位于 synthetic frame 一侧、被高支持度长墙隔开且面积显著更小的外围 cell。
- `test_docs_maps.py` 通过 `Rose2Segmenter.segment()` 公共接口固定五张地图的房间数和未分配 cell 数。

## 输入处理

`*.render.png` 是展示用放大图，运行前按完全相同的像素块无损还原：

| 场景 | 原图 | 还原倍率 | 算法输入尺寸 |
| --- | --- | ---: | ---: |
| corridor4 | `src/oomwoo_segmentation/test/maps/demo/corridor4.render.png` | 3 | 125×53 |
| grid6_furniture | `src/oomwoo_segmentation/test/maps/demo/grid6_furniture.render.png` | 3 | 94×63 |
| living_room | `src/oomwoo_segmentation/test/maps/demo/living_room.render.png` | 2 | 99×98 |
| room3 | `src/oomwoo_segmentation/test/maps/demo/room3.png` | 1 | 200×200 |
| room4 | `src/oomwoo_segmentation/test/maps/demo/room4.png` | 1 | 200×200 |
| two_rooms | `src/oomwoo_segmentation/test/maps/demo/two_rooms.render.png` | 1 | 100×80 |

重新生成命令（每次测试输出到 `output/`，见 docs/DEVELOPMENT.md）：

```bash
python3 src/oomwoo_segmentation/test/run_map_batch.py <图片> --embedded-scale <倍率> --output-root output
```

## 修复后结果

| 场景 | 结果 | walls | 未分配 | 目视判断 |
| --- | ---: | ---: | ---: | --- |
| corridor4 | 5 rooms | 7 | 0 (0.0%) | 4 个房间 + 走廊符合预期，自由区域 100% 覆盖 |
| grid6_furniture | 6 rooms | 6 | 0 (0.0%) | 物理墙体由 structural raster 补全，六个房间被硬墙准确隔开，自由区域 100% 覆盖 |
| living_room | 1 room | 5 | 0 (0.0%) | 家具不切分主体，主体房间测地全覆盖（原 449 空洞已全部消除） |
| room3 | 6 rooms | 10 | 0 (0.0%) | 主要物理墙体隔断 6 个独立房间，自由空间 100% 覆盖 |
| room4 | 5 rooms | 10 | 0 (0.0%) | 5 个房间严格按物理墙体切分，原 180 像素内部空洞已通过测地波前扩散全覆盖 |
| two_rooms | 2 rooms | 8 | 0 (0.0%) | 2 个房间严格以中间实体墙与门洞隔开，自由区域 100% 覆盖 |

## 文件说明

每个场景目录包含：

- `input.original.png`：docs 中的原始展示图片；
- `source.png`：去除展示倍率后的实际算法输入；
- `source.render.png`：通用地图渲染；
- `segments.png`：最终标签结果；
- `walls.png`：Detected Wall 叠加，按 support 黄→红着色；
- `diagnostics/01_cleaned_map.png`：ROSE 清理结果；
- `diagnostics/02_extended_lines.png`：ROSE2 延伸墙线；
- `diagnostics/03_labels_overlay.png`：最终标签叠加；
- `run.txt`：房间面积、cell 数、Detected Wall 端点/support、未分配率和 provider 版本；
- `map.yaml`、`normalization.txt`：可复现实验输入（早期手工流程生成；room4 及之后由 `run_map_batch.py` 生成的场景无此两项，归一化信息记录在 `run.txt`）。

根目录 `summary.png` 汇总早期五张地图的输入和结果；`summary.txt` 为最近一次批量运行的文本汇总。

## 结论

6/6 地图均能通过 Action Server 返回满足公共 labels/rooms 契约的结果，并同时暴露 Detected Walls（5–10 段，外框墙 support=1.0，端点位于 map 坐标系）。`corridor4` 为 4 个房间加走廊，`grid6_furniture` 为 6 个房间，`living_room` 为 1 个主体房间，结构数量均符合这些固定 demo 的预期。`living_room` 的 449 个外围 free cells 按严格 frame-fringe 规则保留为未分配区域，而不是跨越高支持度外墙并入室内。`room4` 的 180 个内部 free cells 是仍待决策的覆盖缺口；关闭 frame-fringe 规则后结果不变，因此不能把它描述为外围过滤。渲染中的橙色统一表示 label 0 的 cleanable/free cells。当前仓库没有人工真值 label mask，因此报告只进行房间数量、边界和未分配率的结构/目视判断，未计算 IoU。
