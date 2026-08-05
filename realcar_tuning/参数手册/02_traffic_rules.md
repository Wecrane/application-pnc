# 实车 TrafficRule 配置文件参数回归分析（2026-08-05）

> 数据源：`realcar_tuning/ORIGINAL/modules/planning/traffic_rules/`（实车 HQEV_BDN 值）
> 回归依据：`modules/planning/traffic_rules/<rule>/proto/*.proto` + `<rule>.cc` + `planning_base/common/util/common.cc`（BuildStopDecision）
> 标注：⭐比赛核心 / ○一般 / △无关
> ⚠️ 本地 `modules/planning` 已恢复官方基线（2b48d5e）；正文"本地竞赛代码"备注指恢复前 add5 版（备份于 backup/），仅为历史对照
> 说明：实车跑的是**官方基线代码 + 限速插件**（无本地竞赛魔改）。本地竞赛代码中若存在硬编码覆盖（注释标出），只影响仿真，不影响实车；参数本身语义以官方逻辑为准。

---

## 1. 文件：traffic_rules/crosswalk/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| ⭐ stop_distance | 0.10 | `crosswalk.cc` 对需停斑马线调 `BuildStopDecision(vo_id, start_s, stop_distance, STOP_REASON_CROSSWALK, ...)`；`common.cc`：停止墙建在 `start_s`，**停车点 = start_s - stop_distance**（车头距停止线 0.10m 停车）。proto 默认 1.0 | 调大→车停得离斑马线停止线更远（距离类评分可能偏低）；调小→更贴线（0.10 几乎贴线）。⚠️本地竞赛代码硬编码覆盖：`signal_overlaps.empty() && stop_sign_overlaps.empty()`（纯人行道场景）时强制用 **1.15**（赛题7），红绿灯/停止标志场景才用 config 0.10；实车官方代码无此覆盖，直接使用 0.10 |
| ○ max_stop_deceleration | 4.0 | `CheckStopForObstacle` 末尾：`if (stop_deceleration >= config_.max_stop_deceleration()) { if (obstacle_l_distance > stop_strict_l_distance) stop = false; }`。`stop_deceleration`=按当前车速在斑马线前急停所需减速度 | 调大→更晚刹车也坚持停车（刹得住范围更大）；调小→更容易"刹不住放行"（安全风险↑）。实车 4.0 = proto 默认，保持默认即可 |
| ○ min_pass_s_distance | 1.0 | `MakeDecisions` 顶部：`adc_front_edge_s - crosswalk_overlap->end_s > min_pass_s_distance` 即视为已通过该斑马线（跳过） | 调大→车头越过斑马线尾端更远才结束处理（处理时间更长）；调小→更快结束。实车 1.0 = 默认，通常不用动 |
| ⭐ expand_s_distance | 0.0 | `CheckStopForObstacle`：`crosswalk_ptr->polygon().ExpandByDistance(expand_s_distance)` 生成**扩展区**，行人/自行车在多边形扩展区内即进入停止判定流程。proto 默认 2.0，实车压到 0.0 → 扩展区=原始斑马线多边形 | 调大→行人离斑马线更远（含人行道两侧人行区域）也触发停止判定，对"行人过斑马线"赛题更安全但可能过度停车；调小→只认斑马线本体的行人。实车 0.0 保守（只认线内）。⚠️本地竞赛代码有 add5：行人在扩展区内**必停**（不依赖速度/横向）；仿真 profile 曾用 4.0~6.0 扩大行人检测（见 speed_decider 配套），实车是否需调大取决于现场行人感知 |
| ⭐ stop_strict_l_distance | 1.0 | `CheckStopForObstacle`：`obstacle_l_distance <= stop_strict_l_distance` 走**强停分支**——on_road 且 s 在车前→停；不在路（斑马线上/路肩）→路径交叉或向车移动→停。proto 默认 4.0，实车压到 1.0 | 调大→行人横向更远也强停（更敏感、更安全、易多停）；调小→只有很贴近车行驶带的行人才强停。实车 1.0 很激进（默认 4.0），仅对非常贴近的行人强停，减少误停 |
| ⭐ stop_loose_l_distance | 1.5 | 同函数：`obstacle_l_distance >= stop_loose_l_distance` 走**宽松分支**——只有路径交叉（ST 边界非空）才停。介于 strict 与 loose 之间用历史决策平滑。proto 默认 5.0，实车压到 1.5 | 调大→横向更远才完全放行（更保守）；调小→1.5m 外基本放行（只拦路径交叉）。实车 strict=1.0 / loose=1.5 间距极窄，中间"历史决策平滑"区间几乎不存在，判定更干脆 |
| ○ start_watch_timer_distance | 10 | `MakeDecisions`：`stop && !is_on_lane && (start_s - adc_front_edge_s <= start_watch_timer_distance)` 才启动"静止行人计时器"（记录时间戳）。proto 默认 40.0，实车压到 10 | 调大→距斑马线更远就开始计时（结合 stop_timeout 更快放行）；调小→临近才计时。实车 10 只在接近斑马线 10m 内计时，避免过早计时误放行 |
| ⭐ stop_timeout | 3.0 | 计时器逻辑：行人静止（速度 ≤ 0.3m/s）且 `stop_time >= stop_timeout` → `stop = false`（放行）。proto 默认 10.0，实车压到 3.0 | 调大→静止行人的等待时间更长（更安全、耗时↑）；调小→静止 3 秒即放行（省时间，配合 30 分钟计时赛关键）。实车 3.0 明显偏激进，配合 start_watch_timer_distance=10 使用 |

---

## 2. 文件：traffic_rules/traffic_light/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| ⭐ enabled | true | `MakeDecisions` 开头：`if (!config_.enabled()) return;` 规则总开关。false 则不处理信号灯（不建 STOP、本地代码中也不下发绿灯限速） | 置 false→彻底忽略信号灯（危险，红绿灯赛题必挂）；置 true→红绿灯赛题生效。实车 true，保持 |
| ⭐ stop_distance | 0.10 | 红灯/黄灯分支 `BuildStopDecision(vo_id, start_s, stop_distance, STOP_REASON_SIGNAL, ...)`；停车点 = 信号灯停止线 start_s - 0.10m。proto 默认 1.0，实车压到 0.10 | 调大→红灯停车离停止线更远；调小→更贴线。实车 0.10 几乎贴线停（红绿灯赛题停车区间若要求 1.0~1.5m 需留意：0.10 会停得很近，需按现场评分区间重标定，参考仿真经验 1.25） |
| ○ max_stop_deceleration | 4.0 | 红/黄灯分支：`if (stop_deceleration > config_.max_stop_deceleration())` 表示"刹不住"→ **SKIP 红灯直接继续走**（忽略红灯）。proto 默认 4.0 | 调大→更晚刹车也会坚持停红灯；调小→更容易闯红灯（危险）。实车 4.0 = 默认。⚠️ 基线自带：`GREEN\|\|BLACK → continue`（绿灯/黑灯直接放行不建 STOP，绿灯题必要）；历史 add5 版曾有绿灯硬编码限速 10.7/4.9（已随恢复官方移除，见 backup） |

---

## 3. 文件：traffic_rules/stop_sign/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| ⭐ enabled | true | `MakeDecisions` 开头：`if (!config_.enabled()) return;` 总开关 | false→停止牌赛题必挂；实车 true |
| ⭐ stop_distance | 4.0 | 对每个未完成停止牌 overlap：`BuildStopDecision(vo_id, start_s, stop_distance, STOP_REASON_STOP_SIGN, ...)`；**停车点 = 停止牌停止线 start_s - 4.0m**。proto 默认 1.0，实车为 4.0（明显加大） | 调大→离停止牌更远停车（4.0 会让车停在停止线前 4m）；调小→更贴线。实车 4.0 偏保守（停止线赛题若评分要求停在线前 1.0~1.5m，4.0 可能停太远，需按现场评分区间重标定） |

---

## 4. 文件：traffic_rules/scene_speed_limit/conf/default_conf.pb.txt（限速插件）

> 源码：`scene_speed_limit.cc` 对 map_path 中四类 overlap 逐段调 `reference_line->AddSpeedLimit(start_s - forward_buffer, end_s + backward_buffer, limit_speed)`。**四区段实车全为 disabled**（插件当前 no-op），需按赛题启用对应区段。

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| ⭐ crosswalk_enabled | false | `if (config_.crosswalk_enabled())` 才遍历 `map_path.crosswalk_overlaps()` 下发斑马线限速区 | 置 true→斑马线区段限速生效（实车赛题"人行道限速/减速带"用）。实车 false = 未启用（待赛前按赛题配置） |
| ○ crosswalk_forward_buffer | 1.0 | 限速区起点 = `start_s - crosswalk_forward_buffer`（斑马线前提前量）。proto 默认 5.0 | 调大→更早进入限速区（减速更从容）；调小→更晚。实车 1.0 提前量很小（限速起点几乎贴斑马线，可能刹不及时，需按减速能力上调） |
| ○ crosswalk_backward_buffer | 1.0 | 限速区终点 = `end_s + crosswalk_backward_buffer`（斑马线后保持量，防出区即加速）。proto 默认 2.0 | 调大→出斑马线后限速保持更久（防评分区超速）；实车 1.0 较小 |
| ⭐ crosswalk_limit_speed | 1.0 | 斑马线限速值（m/s）。proto 默认 3.0 | 实车 1.0。**新版评分上限 1.0 → 推荐 0.7（留 0.3 裕量）**；旧版 4 题制上限 1.5 取 1.2~1.4 |
| ⭐ traffic_light_enabled | false | 同上，遍历 `map_path.signal_overlaps()` 对信号灯区段限速 | 置 true→信号灯区段限速（实车赛题"交通灯路口减速/减速带"用）。实车 false |
| ○ traffic_light_forward_buffer | 1.0 | 信号灯前提前量（proto 默认 8.0） | 同上，1.0 偏小 |
| ○ traffic_light_backward_buffer | 1.0 | 信号灯后保持量（proto 默认 2.0） | 同上 |
| ⭐ traffic_light_limit_speed | 1.0 | 信号灯区段限速值（proto 默认 4.0） | 实车 1.0。**新版评分上限 1.0 → 推荐 0.7（留 0.3 裕量）**；旧版取 1.2~1.4 |
| ○ speed_bump_enabled | false | 遍历 `map_path.speed_bump_overlaps()` | ⚠️苏州正式图 speed_bump=0（减速带评分区=第 1 个信号灯/人行道位置），此区段**大概率无效**，减速带应靠 crosswalk/traffic_light 区段覆盖 |
| ○ speed_bump_forward_buffer | 1.0 | 减速带前提前量（proto 默认 3.0） | 仅当图中有 speed_bump 才相关 |
| ○ speed_bump_backward_buffer | 1.0 | 减速带后保持量（proto 默认 1.0） | 同上 |
| ○ speed_bump_limit_speed | 1.0 | 减速带限速值（proto 默认 2.0） | 苏州图无效，可忽略 |
| △ junction_enabled | false | 遍历 `map_path.junction_overlaps()` 对路口区段限速 | 路口限速与实车赛题无关，保持 false |
| △ junction_forward_buffer | 1.0 | 路口前提前量（proto 默认 5.0） | 无关 |
| △ junction_backward_buffer | 1.0 | 路口后保持量（proto 默认 2.0） | 无关 |
| △ junction_limit_speed | 1.0 | 路口限速值（proto 默认 3.0） | 无关 |

---

## 5. 文件：traffic_rules/destination/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| ○ stop_distance | 0.2 | `MakeDecisions`（is_near_destination 时）：`dest_lane_s = max(0, routing_end->s() - FLAGS_virtual_stop_wall_length - stop_distance)`，再 `BuildStopDecision(vo_id, lane_id, dest_lane_s, stop_distance, STOP_REASON_DESTINATION, ...)`；停车点距终点 ≈ 0.2m。proto 默认 0.5，实车压到 0.2 | 调大→终点前更早停（留更大裕量）；调小→更贴终点停（0.2 几乎到线）。终点距离评分若按"距终点越近越好"则 0.2 有利；但太贴线有压线风险。本地竞赛代码有 `FLAGS_destination_pass_through`（到终点不停稳直接过）与移除 `destination_check_distance` 限制的实验，实车官方代码无 |

---

## 6. 文件：traffic_rules/reference_line_end/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| ○ stop_distance | 0.5 | `ApplyRule`：剩余参考线 ≤ min_reference_line_remain_length 时，在参考线末端建 REF_END 停止墙，`stop_line_s = obstacle_start_s - stop_distance`（stop 点距参考线末端 0.5m）。proto 默认 0.5 | 调大→距参考线末端更远停；调小→更贴末端。实车 0.5 = 默认 |
| ○ min_reference_line_remain_length | 5.0 | `remain_s = Length - adc_front_end_s`；`if (remain_s > min_reference_line_remain_length) return;`（不建墙）。proto 默认 50.0，实车压到 5.0 → **只剩 5m 才建终点停止墙**（默认 50m 会在距末端 50m 就建墙提前减速） | 调大→更早建墙更早减速（提前 50m 减速拖慢节奏）；调小→更晚建墙（可能冲过参考线末端）。实车 5.0 晚建墙，配合 destination 规则避免双墙竞争蠕动。本地竞赛代码有 `FLAGS_disable_reference_line_end_stop` 可完全禁用，实车官方代码无 |

---

## 7. 文件：traffic_rules/keepclear/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| △ enable_keep_clear_zone | true | `ApplyRule`：为 `map_path.clear_area_overlaps()`（黄色网格禁停区）建 KEEP_CLEAR 静态障碍（SetReferenceLineStBoundaryType KEEP_CLEAR） | 路口/网格区"禁停"约束。苏州正式图是否有 clear area 待查，一般赛题不涉及 |
| △ enable_junction | true | 为 `FirstEncounteredOverlaps` 的 PNC_JUNCTION 建 keep clear 障碍（路口内禁停，防堵路口） | 路口保持通行不滞留的辅助约束；赛题无直接评分点 |
| △ min_pass_s_distance | 2.0 | `BuildKeepClearObstacle`：`adc_front_edge_s - keep_clear_start_s > min_pass_s_distance` 视为已通过该区（跳过）。proto 默认 2.0 | 车头越过禁停区起点 2m 即跳过。实车 = 默认 |
| △ align_with_traffic_sign_tolerance | 4.5 | junction 分支：路口起点与最近信号灯/停止牌/斑马线起点距离 ≤ 4.5m 时，把路口起点**对齐**到该交通标志起点（避免 keep-clear 与停止决策错位）。proto 默认 4.5 | 调大→更易对齐；调小→不易对齐。实车 = 默认，一般不动 |

---

## 8. 文件：traffic_rules/rerouting/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| △ cooldown_time | 3.0 | `ChangeLaneFailRerouting`：距上次 rerouting 小于 cooldown_time 秒则跳过（防频繁重路由）。proto 默认 3.0 | 实车 = 默认。赛题单一直线/单一路口路线基本不触发换道失败重路由 |
| △ prepare_rerouting_time | 2.0 | 换道失败末端距离 ≤ `speed × prepare_rerouting_time` 时才重路由（更近才需要）。proto 默认 2.0 | 调大→更早重路由；调小→更晚。实车 = 默认，一般无关 |

---

## 9. 文件：traffic_rules/yield_sign/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| △ enabled | true | `MakeDecisions` 开头：`if (!config_.enabled()) return;` 总开关 | 实车 true。苏州正式图无让行标志（stop_sign=0），基本不触发 |
| △ stop_distance | 1.0 | 对未完成让行标志：`BuildStopDecision(vo_id, start_s, stop_distance, STOP_REASON_YIELD_SIGN, ...)`；停车点 = 让行线前 1.0m。proto 默认 1.0 | 实车 = 默认。地图无让行标志则无效参数 |

---

## 10. 文件：traffic_rules/backside_vehicle/conf/default_conf.pb.txt

| 参数 | 实车值 | 源码含义（回归代码） | 影响/备注 |
|------|--------|----------------------|-----------|
| △ backside_lane_width | 4.0 | `MakeLaneKeepingObstacleDecision`（backside_vehicle.cc:81-93）：对车后/侧障碍（start_s < adc.end_s），若横向投影**与 [-w,w] 有交 → 加 ignore 决策**（忽略后方同车道车，防自车为它刹车）；横向**完全在 ±w 外（start_l>w ∥ end_l<-w）→ continue 不 ignore**（保留约束，防相邻车道车切入）。proto 默认 4.0 | 调大→更多障碍落入带内被忽略（**更激进**）；调小→更多保留约束（更保守）。实车 4.0=默认。直线赛题基本无影响 |
