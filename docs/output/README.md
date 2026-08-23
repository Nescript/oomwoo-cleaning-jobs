# ROSE2 docs 地图修复后验证

Provider：`rose2 upstream-3a010b9e6bb2+oomwoo.2`

## 本次修复

- 修正竖直线分支在 `b == 0` 时先读取未初始化 `Y1` 的问题。
- 外轮廓改为最大连通自由空间轮廓，不再固定访问第二个非自由空间 contour；支持无 unknown 外围的裁剪地图，也避免把家具岛误选为房间外边界。
- 在构造 Shapely Polygon 前过滤少于三个不同顶点的退化 cell。
- 新增 `test_docs_maps.py`，通过 `Rose2Segmenter.segment()` 公共接口覆盖原先失败的三张地图。

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
| corridor4 | 6 rooms | 270 cells / 4.4% | 不再崩溃，但走廊被切成两个区域并出现斜向未分配带；预期约 5 rooms，仍过分割 |
| grid6_furniture | 5 rooms | 420 cells / 7.9% | 不再崩溃，但右侧上下区域被合并，并有斜向未分配带；预期约 6 rooms，仍欠分割 |
| living_room | 3 rooms | 0 | 不再崩溃，但单个真实房间被家具/边界线切成 3 个区域；预期 1 room，仍过分割 |
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

三个运行时失败已经修复：5/5 地图均能通过 Action Server 返回满足公共 labels/rooms 契约的结果。修复解决的是崩溃、错误外轮廓和微小伪区域问题；`corridor4`、`grid6_furniture`、`living_room` 仍暴露上游 ROSE2 的过/欠分割质量限制，不能仅凭“成功返回”视为算法效果达标。当前仓库没有人工真值 label mask，因此报告只进行房间数量、边界和未分配率的结构/目视判断，未计算 IoU。
