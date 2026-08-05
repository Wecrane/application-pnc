# 05 调参方法论、评分机制与实车差异

> 仿真 800/800 满分实战沉淀的调参方法论。核心目标：**30 分钟内完成所有赛题调参并测试**。

---

## 1. 配置体系速查（必须掌握）

### 1.1 配置两层结构

| 层 | 位置 | 说明 |
|----|------|------|
| 源码默认层 | `modules/planning/<插件>/conf/*.pb.txt` | 官方默认值，随代码分发 |
| **profile 覆盖层** | `profiles/default/modules/planning/` | **比赛只改这层**（在允许修改范围内） |

- 运行期实际读取 = 源码默认 + profile 覆盖（buildtool 构建时合成；容器内 `/apollo` 软链 → `profiles/current` → `default`）
- **pb.txt/conf 改动不需编译**，改哪个模块就重启哪个模块即生效（**不用 `aem profile use default`**）
- 改 C++ 源码/新增 proto 才需 `buildtool build -p modules/planning/ -j15`（control：`-p modules/control/`）
- ⚠️ **实车赛只调参不改代码 → 全程无需编译**，改配置后**重启对应模块**即可（见 car_connect/apply_config.sh）
- 🔧 **分级策略（用户明确定义）**：① 优先只改 `profiles/` 配置（不编译）→ ② 非必要不动 `modules/planning` 源码 → ③ 实在无法实现才动源码+编译
- ✅ **官方允许改 `control` 代码**（不只 planning）：`modules/` 整体可改，但动源码=编译=耗时+风险，优先纯参数

### 1.2 关键配置文件位置

| 配置 | 路径 |
|------|------|
| 全局 gflags | `profiles/default/modules/planning/planning_component/conf/planning.conf` |
| TrafficRule 注册 | `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` |
| Scenario 注册 | `modules/planning/planning_component/conf/public_road_planner_config.pb.txt` |
| 人行道 | `.../traffic_rules/crosswalk/conf/default_conf.pb.txt` |
| 红绿灯 | `.../traffic_rules/traffic_light/conf/default_conf.pb.txt` |
| 停止牌 | `.../traffic_rules/stop_sign/conf/default_conf.pb.txt` |
| 限速插件 | `.../traffic_rules/scene_speed_limit/conf/default_conf.pb.txt` |
| 跟车/停车决策 | `.../tasks/speed_decider/conf/default_conf.pb.txt` |
| 路径权重 | `.../tasks/lane_follow_path/conf/default_conf.pb.txt` |
| 速度平滑 | `.../tasks/piecewise_jerk_speed/conf/default_conf.pb.txt` |
| **Control 全局 gflags** | `modules/control/control_component/conf/control.conf` |
| **Control 流水线** | `modules/control/control_component/conf/pipeline.pb.txt` |
| **横向 LQR** | `modules/control/controllers/lat_based_lqr_controller/conf/controller_conf.pb.txt` |
| **纵向 PID** | `modules/control/controllers/lon_based_pid_controller/conf/controller_conf.pb.txt` |

> Control 模块官方发布包已在本地 `modules/control/`（详见 `references/09-control-module.md`）。只改 control 参数 → 重启 Control 模块（`apply_config.sh control`）。

### 1.3 生效（只改参数：改哪个模块就重启哪个模块）

```bash
# 只改参数时：不用编译、不用 aem profile use default！
# 方式一（推荐）：Dreamview+ 模块栏，把对应模块开关关掉再打开（stop → start）
#   改 planning 配置 → 重启 Planning 模块
#   改 control 配置  → 重启 Control 模块
# 方式二（命令行）：
cyber_launch stop /apollo/modules/planning/planning_component/launch/planning.launch
cyber_launch start /apollo/modules/planning/planning_component/launch/planning.launch
cat /apollo/.../conf/default_conf.pb.txt            # 验证软链值已生效

# ⚠️ 只有改 C++ 源码编译后才需要：
aem profile use default        # 同步 profile（编译后必做，防配置丢失）
aem bootstrap stop && aem bootstrap start --plus   # 重启 Dreamview
```

---

## 2. 评分机制关键认知（仿真实战实锤）

### 2.1 评测限速 = 参考线限速（最重要）

- 评测 SpeedLimit 的 "Max speed" = **planner 参考线限速 = min(地图车道限速, planning_upper_speed_limit, 规则/插件 AddSpeedLimit)**
- ⚠️ **陷阱**：把 `planning_upper_speed_limit` 调低 → 评测限速也变低 → 起步超冲更容易超速（自锁陷阱）。**不要往下调**（仿真教训：16.39 时起步超冲到 16.9 必超速；调到 20 后评测限速回 19.44 大裕量通过）
- 起步超速共性根因：起步满加速（3.0）→ 巡航过渡段计划加速度骤降 → 实际速度超冲 +0.5m/s

### 2.2 超速扣分按帧

- 每超速 1 m/s，本场景每帧扣 2 分 → 连续超速帧多 = 分数大量流失
- 限速类赛题（1/7/8）必须留裕量（-0.1~-0.3），防控制超调

### 2.3 "车停止"是评测采样触发器

- 评测器（如 DistToObstacleCar）在**车停止时**采样距离，多次停止都采样
- **车提前停稳 → 评测提前结束 → 不利计分帧不存在 → 满分**（赛题 5 破局核心）
- 提前停稳手段：缩短终点前的长距离缓减速（destination 距离限制 + 禁 REF_END）

### 2.4 停车距离标定

- 评测距离 ≈ `stop_distance` + 车到 fence 的停车偏移（QP 停车偏移 0.2~0.8m 波动，由接近 STOP 速度决定）
- 停车偏移受接近速度影响：速度快→停得近（偏移大）；速度慢→停得远
- 纯调 stop_distance 无法完全消除波动 → 仿真最终靠代码分支兜底（crosswalk 行人 STOP 强制停）；实车只能现场实测标定 + 留裕量

### 2.5 时间限制

- 各场景有仿真上限（仿真：跟车 60s，其余 100s）；实车 30 分钟总时间
- 时间黑洞：STOP→FOLLOW 恢复迟滞（prediction 静态→动态切换延迟，仿真 9.4s）、终点前长距离缓减速、停车等待行人过久
- 提速手段：`follow_min_time_sec` 调小、跟车距离调小、`planning_max_acceleration` 拉满、终点前短距离急刹

---

## 3. 调参七步法

1. **读场景**：明确赛题要求（限速值/停车距离区间/时间限制），查地图路线元素
2. **诊断现状**：先跑一遍，看车实际行为（停车距离多少、速度多少、哪里超时）
3. **定位机制**：是哪个规则/任务在管？（TrafficRule 停车 / gflag 限速 / speed_decider 决策 / 场景阶段）
4. **选参数**：改对应 conf，**一次只改一个**（多参数同时改无法归因）
5. **留裕量**：限速留 0.2~0.5；停车距离取区间中值
6. **验证**：测试 → 看评分/行为 → 微调（可重复测试）
7. **固化**：记录改动（git/备份），便于回滚

**归因技巧**：改完若行为异常，先想"是不是这个参数真的被读到"（验证软链）+ "是不是被其他机制覆盖"（如 speed_decider 6m 覆盖 crosswalk 1.75m）。

---

## 4. 实车 vs 仿真差异（调参必须考虑）

| 维度 | 仿真 | 实车 | 应对 |
|------|------|------|------|
| 感知 | 理想 | 激光雷达真实感知，噪声/闪烁大 | 对感知类障碍（护栏等）留更大裕量或现场确认 |
| 预测 | 可控 | 真实预测，行人/车辆行为随机 | 停车等待时间留足 |
| 控制 | 理想 | 真实底盘延迟/超调明显 | **限速裕量加大**（1.2~1.4 而非 1.4~1.5） |
| 场景 | 固定 JSON | 现场布设可能有偏差 | 现场先观察路线/标志位置 |
| 时间 | 场景上限 | 30 分钟全流程 | 调参效率优先 |
| 代码 | 本地魔改 | 官方基线+限速插件 | 只调参，不搬本地配置（gflag 不兼容） |
| 评分 | 仿真器 | 实车数据采集 | 细节以现场手册为准 |

---

## 5. 30 分钟时间管理建议

| 阶段 | 时间 | 内容 |
|------|------|------|
| 连接 | ~2min | Wi-Fi → SSH → aem enter → bootstrap start --plus |
| 摸底 | ~3min | 确认工程目录、profile 结构、限速插件是否在（预计已预装）、traffic_rule 注册 |
| 调参 | ~10min | 按赛题优先级改配置（限速类→停车类→巡航），一次一个 |
| 测试 | ~15min | 逐赛题测试（9 题 × ~1.5min + 重启缓冲），重复测试取最优 |
| 收尾 | ~2min | 保存、确认最优成绩、停止操作 |

**优先级**：先保证不 0 分（限速+停车距离），再优化速度/满分。

---

## 6. 常见问题速查

| 现象 | 可能原因 | 处理 |
|------|---------|------|
| 改配置不生效 | 没重启对应模块 / 改错文件（软链） | 重启对应模块 → cat 验证 |
| 启动报未知 flag | 配置引用实车没有的 gflag（pedestrian_stop_distance 等） | 从官方默认配置起步，只加官方存在的 flag |
| 停车距离不对 | stop_distance 不匹配 / 被 speed_decider 6m 覆盖 | 查 STOP reason_code（104 crosswalk / 1 车辆 / 2 终点） |
| 超速 | 限速值贴边 / planning_upper_speed_limit 太低 | 限速值留裕量；勿把全局限速调低 |
| 时间不够 | STOP→FOLLOW 迟滞 / 终点长距离缓减速 | follow_min_time_sec 调小；destination 距离限制 |
| 右转切内弯 | path_reference_l_weight 太低 | 调大贴线权重（见 04 赛题4） |
| 停车后不走 | 行人/障碍一直阻塞（CheckStopForPedestrian 不看横向） | 现场确认障碍类型；行人横穿场景需等走完 |
| 起步超速 | 满加速超冲 | planning_upper_speed_limit 保持高值（≥地图限速） |

---

## 7. 备赛建议（赛前完成）

1. **在自己环境复现官方代码+限速插件**，把 04 的 9 赛题方案全部验证一遍，记录实车预期行为
2. 准备 conf 修改模板（限速插件、3 个 stop_distance、巡航速度、跟车参数），现场直接粘贴
3. 确认限速插件的安装/注册方法（traffic_rule_config.pb.txt 怎么加）
4. 8月5日实车体验：重点练 SSH 连接、Dreamview 操作、模块启动顺序、路由设置
5. 准备回滚：每处修改记录原始值，出问题 30 秒还原
