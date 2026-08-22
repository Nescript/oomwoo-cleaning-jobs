# oomwoo_cleaning_jobs 开发上下文

这是本仓库唯一的开发上下文。它记录当前范围、已作决定、术语和开放边界；修改实现或设计前先完整阅读本文档。用户的即时指令和已验证的源码事实优先于本文档，并应随后回写本文档。

## 目标与范围

`oomwoo_cleaning_jobs` 负责**已保存地图上的用户清洁意图和长任务编排**：whole-map、选定 Region 和 spot 清洁；Region/virtual wall/keepout 持久化；Job 的状态、暂停、恢复、重试和汇总。

它不拥有覆盖路径规划或底盘执行算法。常规 saved-map 覆盖复用 `oomwoo_coverage`；Nav2 是运动执行层。`clean-and-map` 仅是 first-clean（SLAM、探索和覆盖）的 RFC/算法参考，不是 saved-map Job 的后端或 coverage-progress 提供者。`floor-care` 是未来的 perimeter/edge pass；它可与 `oomwoo_coverage` 的 interior sweep 组合，但不是通用 coverage backend。

外部事实来源：

- [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs)
- [clean-and-map RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/clean-and-map)
- [floor-care RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/floor-care)
- [oomwoo-ros2-tools](https://github.com/makerspet/oomwoo-ros2-tools)
- [SOFTWARE_INTERFACES.md](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

## 当前阶段

第一阶段只交付：

`已保存地图 → 自动候选区域 → 手动编辑 → 校验 → Published Region Set 持久化`

输入为 `nav_msgs/msg/OccupancyGrid`，两条获取路径：核心库直接解析 nav2 trinary 格式的 `map.yaml + 图像`（供测试与 CLI）；GUI 运行时订阅 `/map`（transient_local QoS）或打开地图文件。已知自由单元可清扫，未知和障碍不可清扫。

第一阶段不驱动机器人、不执行完整 Job，也不冻结执行器 action 或 feedback 定义。验证用 GUI 为 `oomwoo_cleaning_jobs_ui`（PyQt5 独立应用，位于本仓库 `src/` 下），不是最终控制 App。

## 领域模型

| 术语 | 含义 |
|---|---|
| Source Map | 不可变的已保存 OccupancyGrid；identity 由元数据与 cell 数据的内容 hash 派生，hash 变更即视为新地图。 |
| Cleanable Space | Source Map 内已知自由且允许清扫的空间。 |
| Region | Cleanable Space 的命名部分，以栅格掩码为权威表示（支持孔洞与离散组件）；几何轮廓由掩码派生，仅用于 GUI 与导出。 |
| Candidate Region | 自动划分或编辑中、尚未审核的 Region。 |
| Region Set | 属于一张 Source Map 的、版本化 Region 和空间约束集合。 |
| Published Region Set | 已通过校验，可用于生成 Job 的 Region Set。 |
| Keepout / Virtual Wall | 不修改 Source Map 的独立持久化约束；Virtual Wall 是线状 Keepout。 |
| Segment | Job 中由一种清洁策略处理的目标部分；不必等同一个 Region。 |
| Coverage artifact | 可校验的已覆盖空间记录（例如 coverage grid）；百分比本身不是 artifact。 |

自动划分是可替换策略（当前为距离变换 + watershed），置信不足时退化为连通自由空间，并显式标出不确定/未分类区域。

手动编辑至少支持创建、移动/删除/重命名、合并/拆分 Region，以及创建 Keepout/Virtual Wall。编辑时的即时裁剪与发布时的校验分级见「第一阶段实施决定」。允许未划分或故意不清扫的自由空间，但 GUI 必须明确呈现。

## 第一阶段实施决定

以下决定于 2026-08-22 与用户逐条确认（grilling 讨论）；修改前需重新与用户确认。

### 包结构

- `src/oomwoo_cleaning_jobs_core`：纯 Python 库，零 ROS 依赖。含地图文件加载、自动分割、掩码编辑、校验、持久化，可无头 pytest。
- `src/oomwoo_cleaning_jobs_ui`：PyQt5 独立应用 + rclpy 节点，薄适配层。
- `oomwoo_cleaning_interfaces` 推迟到首次出现真实跨进程消息需求（预计阶段二）再建。

### 地图 identity 与变更检测

identity = SHA-256(`resolution` float64 字节 + `width` + `height` + `origin` position/orientation + 原始 int8 cell 数据)，排除 `header.stamp`、`frame_id`、`map_load_time`，不做三值化。短 id 取前 12 位。

hash 的角色是**变更检测器与存储键**，不是多地图管理。saved map 视为静态制品，唯一变更来源是用户重新建图/保存。hash 变化即新地图：找不到对应 Region Set 时 GUI 明确提示「当前地图无区域集；磁盘上存在 N 份属于其他地图的区域集」。阶段一不做区域集迁移/重投影（原点与分辨率可能全变，像素级迁移不可靠），见未决边界 9。

### 地图文件加载保真约定（已核实 nav2 jazzy map_io.cpp）

trinary 加载：`occ = 1 - color/255`（negate=0）；`occ >= occupied_thresh` → 100，`occ <= free_thresh` → 0，否则 -1；`alpha < 255` 一律 unknown；图像顶行对应地图最大 y，加载后垂直翻转。map_saver 固定写像素 0(occupied)/254(free)/205(unknown) 并配 `occupied_thresh: 0.65, free_thresh: 0.196`（205 借此回读为 unknown）。核心库加载器 `map_io.load_map_file` 与上述行为对齐，仅支持 trinary。

已实现：`src/oomwoo_cleaning_jobs_core`（ament_python）含 `source_map.SourceMap`（identity/掩码）、`map_io`、`segmentation`（watershed 分割）、`render`/`render_map` CLI（`oomwoo-render-map`，`--segment` 出叠加图）；`test/` 下合成地图夹具（`fixtures`：双房间/含未知块/开间/极小房间/开放式双区/N 房间网格/走廊户型）与 36 个 pytest，`colcon build/test` 与直接 `pytest` 均已验证通过。演示输出在 `docs/demo/`（真实 living_room 与合成双房间的分割效果图）。

分割已知特性：maximin 淹没 + 合并树鞍部合并后，真实 living_room（单房间多家具）在 ratio 0.5–0.8 全区间精确收敛为 1 个候选、0 未分类；5/6/7 房间网格、4 房间+走廊户型、贴墙家具场景均精确分出预期房间数；真门洞（0.5 m）不被误并，宽开口（1.3 m）正确合并。残留边界情形：小房间配宽门（比值 0.6–0.8 区间）可能误并/误分，由 GUI 手动编辑修正；距离值低于 `min_peak_height_m` 的狭窄地带不产生种子，留在未分类；0.5 m 宽的家具缝隙与 0.5 m 门洞在几何上不可区分（语义问题），依赖用户审核候选。

### 自动分割

距离变换 + 分水岭（OpenCV/scipy 实现）：free mask → distance transform → 局部极大值 markers → **maximin（最宽路径）淹没**（自实现，只在自由空间传播——`cv2.watershed` 在全图淹没会穿墙导致区域溢出，已弃用；瓶颈优先级相同按到种子的测地距离决胜，分界线落在门洞/鞍部处）→ 面积合并（小于 `min_region_area`，默认 1 m²，并入**连接最宽**的邻域——按共享边界长度会穿门漏并，已弃用）→ **鞍部合并**（`_connection_values` 超水平集合并树给出与分界线无关的真实山口高度；山口 ≥ `saddle_merge_ratio`（默认 0.8）× 较小峰高时并查集合并；真门洞比值通常 < 0.5，同片开阔地的伪分割 > 0.8）→ 脊线标记（接触带两侧各一层 cell 标为未分类）。distance peaks 退化（大开间单峰）时整体作为单一候选并标低置信。阈值均为参数。该算法是「先用它看效果」的初始策略，可替换。

### Region 表示与编辑语义

Region 内部表示为 bitmask，天然支持孔洞与离散组件；轮廓由 `cv2.findContours` 派生。编辑为画笔式：brush 增/减 cell、画圈/画线拆分；合并为显式菜单操作，不依赖先画出重叠。

**编辑时即时裁剪**：用户画的是意图，系统存的是 `意图 ∩ Cleanable Space`（已知自由且不在任何 Keepout 内）。落笔即裁剪并显示真实结果（所见即所得）；裁空则该编辑动作无效并提示。压到已有 Region 的笔画**后画者抢占**：重叠 cell 从旧 Region 扣除归新 Region，GUI 必须显著提示旧 Region 被改小。只存裁剪后掩码，不存原始笔画。用户 Region 中间有去不了的家具（被裁剪或 footprint 不可达）是**正常行为**，不是错误。

### 校验分级（发布时）

阶段一 robot footprint 来自参数 `robot_inscribed_radius`（默认 0.17 m）；阶段二才从 Nav2 解析 footprint profile。

Error（阻止发布）：Region 间重叠；Region 含障碍/未知 cell；Region 经 footprint 半径腐蚀后为空；Region 与 Keepout 相交。这些 error 在正常编辑路径下被即时裁剪与抢占规则保证不会发生——发布校验中它们是**系统不变量检查**（防手改文件与 bug），正常用户永远触发不到。

Warning（允许发布，GUI 必须显著呈现）：存在未划分的可清扫自由空间；Region 内 footprint 不可达 cell 比例超过阈值。

### 持久化

根目录 `~/.local/share/oomwoo_cleaning_jobs/maps/<map_hash>/`，含：

- `map_snapshot.{yaml,pgm}`：地图快照（溯源用）。
- `draft/`：`regions.yaml`（Region 元数据）+ `masks/*.png`（1-bit 掩码，可用看图工具检查）+ `constraints.yaml`（Keepout/Virtual Wall 几何）。
- `published/`：同构。一张地图任一时刻至多一个 Published Region Set；发布 = 校验通过后复制 draft 并记录版本号与时间戳。

### Keepout / Virtual Wall

纳入阶段一。Keepout 从 Cleanable Space 扣除；Virtual Wall 是线状 Keepout，以线膨胀为多边形处理。约束与 Region 共享持久化与校验管线。

### 测试策略

pytest 无头：合成地图（走廊 + 多房间，可精确断言分割数量、无重叠、校验判错）为主；仓库内提交 1–2 张真实保存地图做冒烟回归（分割不崩、候选数在合理区间）。GUI 不进 CI，保留手动验收清单。

## 覆盖执行事实与集成方向

现有 `oomwoo_coverage`：读取完整 `/map`、可选 `keepout_filter_mask`，从机器人所在位置选取可达连通域，执行牛耕式单元分解、扫掠和 gap-fill，并通过 Nav2 运动。它没有公开的 Region、Segment 或 target-mask 输入；目前会覆盖整个可达区域，不会只清扫指定 Region。

它订阅外部 `coverage_ratio` 和 `covered_grid`，不自行估计覆盖率。仿真可由 coverage meter 提供这两者；真实机器人需要基于定位与清洁幅宽的 estimator。这一 estimator 是 Job progress、完成判断和精确恢复的独立依赖。

第二阶段优先验证最小垂直切片：

`Published Region → target/allowed-clean mask 适配 → oomwoo_coverage → coverage estimator/grid → Job checkpoint`

不要在该切片被验证前冻结泛化 coverage-backend contract。对指定 Region，优先增加明确的 target/allowed-clean mask；不要通过伪造临时 map 混淆地图语义，除非维护者确认这是既有约定。

## 后续 Job 行为

cleaning-jobs 将负责 RegionSet 任务化、Segment 切分/排序、状态持久化、用户控制、暂停/恢复、重试和汇总。一版仅允许一个 active Job。

Job 启动时固定 map identity、Published Region Set、清洁策略及从 Nav2 解析的 footprint profile；后续编辑仅影响新 Job。暂停是非终止状态，可由用户或安全/硬件条件触发。安全层独立停车；本包只观察和记录，不能承担 `/cmd_vel` 的硬安全职责。resume 是请求，安全层或执行器可拒绝并返回稳定 reason code。

Job checkpoint 由 cleaning-jobs 保存。精确恢复依赖可信 coverage artifact：有 artifact 时继续未覆盖部分；没有时应重做当前 Segment 或要求用户确认，具体策略待定。

长期目标包括：电量、尘盒和拖布状态触发 dock-cycle 的补能/清空/清洗，再恢复覆盖；whole-map、per-room、spot；以及 perimeter + interior 组合。它们均在第一阶段之后。

## 未决边界

1. `oomwoo_coverage` 的指定目标输入：target/allowed-clean mask 的格式、所有者与版本校验。
2. Segment 完成条件：路径完成、目标 coverage、无可恢复 gap，或其组合。
3. 实机 coverage estimator：输入、误差/可信度、grid 格式、QoS、重启后重建。
4. checkpoint 原子性、存储介质及 backend/robot 重启后的恢复规则。
5. Region 到 Segment 的切分规则：面积、时间、电量、补给与策略边界。
6. floor-care perimeter pass 与 interior sweep 的顺序、重叠容忍和共享 coverage artifact。
7. Keepout/Virtual Wall 同时投影到 Nav2 costmap 与 coverage masks 的方式。
8. `RunCleaningJob`、pause/resume/cancel/status 的 ROS 字段、幂等性、QoS、失败语义，以及 battery/bin/mop/dock/localization 输入接口。
9. 区域集在地图变更（hash 变化）后的迁移/重绑定：是否支持、如何重投影与重新校验。阶段一不做，旧区域集仅保留并由 GUI 提示。

## 仓库结构与质量目标

所有代码、测试、接口和开发工具位于本仓库；可在 `src/` 拆分内部 ROS 2/Python 包。核心为 Python 领域/应用逻辑，ROS 2 为适配层。共享 ROS action/message 放在同仓库的 `oomwoo_cleaning_interfaces`，使 GUI 与执行器不依赖 orchestrator 内部代码。

后续测试应可在无头 CI 中覆盖：自动/手动 Region 编辑与校验；whole-map、per-room、spot 仅清扫预期区域；keepout 永不进入；以及强制中断后 coverage artifact 驱动的恢复。先实现第一阶段可重复的地图夹具与验证，再接入仿真执行。
