# ROSE2 上游测试地图验证

Provider：`rose2 upstream-3a010b9e6bb2+oomwoo.4`

## 目的

将上游 [aislabunimi/ROSE2](https://github.com/aislabunimi/ROSE2) `src/maps/` 自带的全部测试地图原样复制到
`src/oomwoo_segmentation/test/maps/rose2_upstream/`（与 pinned commit `3a010b9e6bb2` 一致），
通过本分支的 `SegmentationEngine.segment()` 接口与 Action 批量运行，验证原生移植后的 pipeline
在真实/仿真大地图上不产生崩溃且结果满足公共 labels/rooms 契约
（`validate_result`：label 只落在 cleanable free cell 上，label 连续且确定性）。

## 运行方式

```bash
python3 src/oomwoo_segmentation/test/run_map_batch.py \
    src/oomwoo_segmentation/test/maps/rose2_upstream \
    --output-root output/rose2_upstream
```

每个场景目录包含 `source.render.png`（底图渲染）、`segments.png`（标签叠加）、
`walls.png`（Detected Wall 叠加，按 support 黄→红着色）、
`diagnostics/01..03`（ROSE 清理图、延伸墙线、标签叠加诊断）和 `run.txt`
（尺寸、identity、房间面积、墙体端点/support、未分配率、provider 版本）。

## 结果汇总（20/21 成功）

| 场景 | rooms | walls | 未分配 free cell |
| --- | ---: | ---: | ---: |
| Freiburg_Building_079 map2 | 3 | 6 | 12.5% |
| Freiburg_Building_079 map3 | 4 | 7 | 14.0% |
| Freiburg_Building_079 map4 | 3 | 7 | 2.4% |
| Freiburg_Building_079 map5 | 4 | 9 | 2.8% |
| Freiburg_Building_079 map6 | 6 | 10 | 4.1% |
| Freiburg_Building_079 map7 | 8 | 12 | 2.7% |
| Freiburg_Building_079 map8 | 8 | 15 | 2.5% |
| Freiburg_Building_079 map9 | 10 | 13 | 4.9% |
| Freiburg_Building_079 map10 | 11 | 14 | 3.7% |
| Freiburg_Building_079 map11 | 12 | 17 | 2.6% |
| Freiburg_Building_079 map12 | 12 | 16 | 2.6% |
| Freiburg_Building_079 map13 | 15 | 16 | 1.1% |
| Freiburg_Building_079 map14 | 14 | 17 | 1.7% |
| Freiburg_Building_079 map15 | 18 | 19 | 2.1% |
| Virtual ViMantic_House20 | 5 | 12 | 0.0% |
| Virtual ViMantic_House23 | 7 | 12 | 3.5% |
| Virtual ViMantic_House30 | 4 | 12 | 10.5% |
| carmen ubremen-cartesium | 6 | 13 | 12.2% |
| maps-nostre movecare_map | 6 | 10 | 27.0% |
| maps-nostre simona-house | 3 | 16 | 6.1% |

`Virtual/mapirlab.yaml` 上游从未提交其引用的 `mapirlab.pgm`，记录为
LOAD FAILED 并跳过（YAML 仅为出处完整性保留）。

## 结论

全部 20 张可运行上游地图均通过完整 ROSE + ROSE2 两阶段 pipeline 并满足
`validate_result` 契约，无崩溃、无 label 越界。每张地图同时暴露 Detected Walls
（6–19 段，support∈[0,1]，外框墙 support=1.0），端点经 origin/yaw 转换到 map 坐标系，
墙体校验（有限值、图界内、support/方向范围）全部通过。Freiburg map10–15 的 11–18 rooms
与多层办公楼结构相符；`movecare_map` 未分配率最高（27.0%，大走廊/开放区域保持未分配），
上游仓库没有人工真值 label mask，因此本报告与 demo 地图验证一致，只做结构数量、
未分配率和诊断图的目视检查，不计算 IoU。
