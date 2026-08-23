# ROSE2 docs 地图修复后验证

Provider：`rose2 upstream-3a010b9e6bb2+oomwoo.3`

## 本次修复

- 修正竖直线分支在 `b == 0` 时先读取未初始化 `Y1` 的问题。
- 外轮廓改为最大连通自由空间轮廓，不再固定访问第二个非自由空间 contour；支持无 unknown 外围的裁剪地图，也避免把家具岛误选为房间外边界。
- 在构造 Shapely Polygon 前过滤少于三个不同顶点的退化 cell。
- 去除与四条 synthetic frame lines 精确重合的检测线，避免 `create_cells()` 选择重复边并生成左下角三角形 cell。
- 默认 `lines_threshold=0.22`，过滤由家具 Hough segments 生成的低支持度全图延伸线。
- `test_docs_maps.py` 通过 `Rose2Segmenter.segment()` 公共接口固定五张地图的房间数和未分配 cell 数。

## 输入处理

`*.render.png` 是展示用放大图，运行前按完全相同的像素块无损还原：

| 场景 | 原图 | 还原倍率 | 算法输入尺寸 |
| --- | --- | ---: | ---: |
| corridor4 | `docs/demo/corridor4.render.png` | 3 | 125×53 |
| grid6_furniture | `docs/demo/grid6_furniture.render.png` | 3 | 94×63 |
| living_room | `docs/demo/living_room.render.png` | 2 | 99×98 |
| room3 | `docs/demo/room3.png` | 1 | 200×200 |
| two_rooms | `docs/demo/two_rooms.render.png` | 1 | 100×80 |

## 修复后结果

| 场景 | 结果 | 未分配 | 目视判断 |
| --- | ---: | ---: | --- |
| corridor4 | 5 rooms | 0 | 左下角三角形及走廊重复分区已消失，4 个房间 + 走廊符合预期 |
| grid6_furniture | 5 rooms | 0 | 左下角三角形及未分配带已消失；右侧上下区域仍被合并，预期约 6 rooms，仍欠分割 |
| living_room | 2 rooms | 0 | 家具造成的主体左右切分已消失；主房间为一个标签，但顶部外围条带仍是独立标签，预期整体 1 room |
| room3 | 5 rooms | 0 | 主要墙体和空间边界目视合理 |
| two_rooms | 2 rooms | 0 | 结果符合预期；修复前的 19-cell 微小伪区域已消失 |

## 文件说明

每个场景目录包含：

- `input.original.png`：docs 中的原始展示图片；
- `source.png`：去除展示倍率后的实际算法输入；
- `source.render.png`：通用地图渲染；
- `segments.png`：最终标签结果；
- `diagnostics/01_cleaned_map.png`：ROSE 清理结果；
- `diagnostics/02_extended_lines.png`：ROSE2 延伸墙线；
- `diagnostics/03_labels_overlay.png`：最终标签叠加；
- `run.txt`：房间面积、cell 数、未分配率和 provider 版本；
- `map.yaml`、`normalization.txt`：可复现实验输入。

根目录 `summary.png` 汇总全部输入和结果。

## 结论

5/5 地图均能通过 Action Server 返回满足公共 labels/rooms 契约的结果。左下角重复边问题已修复，`corridor4` 达到预期；家具低支持度延伸线已过滤，`living_room` 主体不再被家具切分。尚未处理的是 `grid6_furniture` 右侧上下房间欠分割，以及 `living_room` 高支持度外墙与 map frame 之间的顶部外围条带；它们属于下一阶段的 doorway/structural-interior 判定问题。当前仓库没有人工真值 label mask，因此报告只进行房间数量、边界和未分配率的结构/目视判断，未计算 IoU。
