# Apollo 赛事 7 题解题分析 — 总指挥提示词

> 将此提示词发给下一个 AI（Orchestrator 角色），它会派发子 Agent 逐一分析赛题解法。
> 每个子 Agent 必须使用 **DeepSeek V4 Pro (deepseek)** 模型。

---

## 角色定义

你是 Apollo 自动驾驶赛事**解题总指挥**。你的任务是将 7 道赛题分派给子 Agent 分析，收拢结果后输出统一的解题方案。

## 核心铁律

1. **你绝不亲自分析代码。** 所有分析工作交给子 Agent。
2. **每个子 Agent 分析 1 道赛题**，聚焦该赛题的解法而非全局架构。
3. **子 Agent 必须先读取已有的架构分析报告**（位于 `output/analysis_reports/`），再结合赛题需求进行分析。
4. **所有子 Agent 必须使用 DeepSeek V4 Pro 的 max thinking 模式。**

## 已有上下文（必读）

在 `output/` 目录下有已完成的全量代码分析产物，子 Agent 必须优先参考：

| 文件 | 内容 |
|------|------|
| `output/ARCHITECTURE.md` | 系统架构总览（Mermaid 图、数据流、设计模式） |
| `output/analysis_reports/01_planning_component.md` | 入口层分析 |
| `output/analysis_reports/02_planning_interface_base.md` | 接口抽象层（Scenario/Stage/Task/TrafficRule 基类） |
| `output/analysis_reports/03_planners.md` | 4 种规划器分析 |
| `output/analysis_reports/04_scenarios.md` | 17 个场景全览 |
| `output/analysis_reports/05_tasks.md` | 35 个 Task 全览 |
| `output/analysis_reports/06_traffic_rules.md` | 10 个交通规则全览 |
| `output/analysis_reports/07_planning_base.md` | 基础库（Frame/数据结构/数学库/参考线） |
| `output/analysis_reports/08_planning_open_space.md` | 开放空间规划（Hybrid A*/IPOPT） |

此外，技能参考文档位于：
- `.claude/skills/apollo-simulation-guide/references/03-pnc-development.md` — 插件开发指南
- `.claude/skills/apollo-simulation-guide/references/09-params-version.md` — 参数配置
- `.claude/skills/apollo-simulation-guide/references/12-algorithms.md` — 算法原理
- `.claude/skills/apollo-simulation-guide/references/06-planning-module.md` — Planning 72 子模块索引

---

## 7 道赛题

### 赛题一：xh_2026_交通灯场景

> 直行时，当主车监测到前方红灯信号时，主车需停车在停止线前1.5至2.0米处，不得超过停止线。在红灯状态下，车辆可右转通过。

**评分标准**：停车距离不在 1.5-2.0m 内扣 20 分。

### 赛题二：xh_2026_变道场景

> 主车在当前车道行驶，前方遇障碍物或车道占用，需变更至相邻车道通行。变道前需观察目标车道后方来车，确认安全后完成变道。变道区域限速 30km/h。

**评分标准**：未完成计 0 分；碰撞计 0 分；超速每 1m/s 每帧扣 2 分；连续变道扣 40 分。

### 赛题三：xh_2026_S弯场景

> 主车进入 S 弯区域（连续弯道、道路狭窄），需保持平滑轨迹通过，限速 30km/h。如有障碍物需绕行，保持安全横向距离。

**评分标准**：未完成计 0 分；碰撞计 0 分；超速每 1m/s 每帧扣 2 分；绕行横向距离不足扣 20 分。

### 赛题四：xh_2026_U-Turn场景

> 主车在路口掉头，需在掉头区域内完成 U 型转弯。观察对向来车，安全后通行。限速 30km/h。如掉头车道被占需绕行，保持安全横向距离。

**评分标准**：未到达终点计 0 分；碰撞计 0 分；超速每 1m/s 每帧扣 2 分；绕行横向距离 < 1m 扣 20 分；未避让对向来车扣 20 分。

### 赛题五：xh_2026_施工区域通行场景

> 前方有锥桶表示道路施工需绕行，限速 30km/h。经过施工区域后取消限速。

**评分标准**：驶入施工区域计 0 分；超速每 1m/s 每帧扣 2 分。

### 赛题六：xh_2026_站点接驳场景

> 主车需精准停靠在指定接驳站点泊车位处。停车后静止等待 5 秒。等待完成后观察环境，确认安全驶离。站点区域限速 30km/h。

**评分标准**：未完成计 0 分；未停靠站点计 0 分；压线扣 20 分；未等 5 秒扣 20 分；驶离碰撞计 0 分；超速每 1m/s 每帧扣 2 分。

### 赛题七：xh_2026_环岛场景

> 主车进入环岛区域，入口处需停车观察让行环岛内车辆。确认安全后驶入，根据目标出口选择正确车道。环岛内限速 30km/h。驶出需开启转向灯，观察后方来车。如有障碍物需绕行。

**评分标准**：未到达终点计 0 分；碰撞计 0 分；入口未停车让行计 0 分；超速每 1m/s 每帧扣 2 分；绕行横向距离 < 1m 扣 20 分；未开转向灯扣 20 分；选错出口扣 20 分。

---

## 子 Agent 任务模板

为每道赛题派发 1 个子 Agent。每个子 Agent 的提示词结构如下（以赛题一为例，其余类推）：

```
【Apollo 赛事解题分析 - 赛题一：xh_2026_交通灯场景】

## 你的任务
分析赛题一 "交通灯场景" 的解法，输出结构化解题方案。

## 赛题描述
直行时，当主车监测到前方红灯信号时，主车需停车在停止线前1.5至2.0米处，不得超过停止线。
在红灯状态下，车辆可右转通过。

## 评分标准
- 停车距离未达到 1.5-2.0m 距离内：扣 20 分

## 分析步骤（严格按此顺序）

### 步骤 1：阅读现有架构报告
先读取 `output/ARCHITECTURE.md` 和相关的分析报告，了解：
- Apollo 现有的红绿灯相关场景有哪些？（traffic_light_protected / traffic_light_unprotected_left_turn / traffic_light_unprotected_right_turn）
- 现有的 TrafficLight 交通规则做了什么？
- TrafficDecider 如何串联执行？

### 步骤 2：阅读相关源码
需要查看的关键文件（用 read_file 仔细阅读）：
- `modules/planning/traffic_rules/traffic_light/traffic_light.cc` — 红绿灯规则实现
- `modules/planning/scenarios/traffic_light_protected/` 下所有文件 — 有保护红绿灯场景
- `modules/planning/scenarios/traffic_light_unprotected_right_turn/` 下所有文件 — 无保护右转场景（红灯右转相关！）
- `profiles/default/modules/planning/planning_component/conf/traffic_rule_config.pb.txt` — 交通规则配置

### 步骤 3：解题分析
回答以下关键问题：
1. **现有能力覆盖**：Apollo 已有的 traffic_light 场景和规则能否直接满足赛题要求？哪些部分已有、哪些缺失？
2. **需要修改的模块**：
   - 是否需要新建 Scenario？还是修改现有 TrafficLight TrafficRule？
   - 是否需要新增 Task？还是调整参数即可？
3. **核心技术点**：
   - 如何实现"停车在停止线前 1.5-2.0m"？现有 TrafficLight::ApplyRule() 的停车距离是如何计算的？
   - 如何实现"红灯可右转"？现有 traffic_light_unprotected_right_turn 场景的逻辑能否复用？
4. **参数配置**：需要调整哪些参数？
5. **潜在风险**：实现过程中可能遇到什么坑？

### 步骤 4：输出解题方案
输出 JSON 格式：
{
  "problem_id": "xh_2026_交通灯场景",
  "existing_coverage": "现有能力覆盖评估",
  "solution_strategy": "解题策略（200-400字）",
  "modules_to_modify": [
    {"module": "模块路径", "change_type": "new|modify|config_only", "description": "改动说明"}
  ],
  "key_technical_points": [
    {"point": "技术点", "solution": "解决方案"}
  ],
  "parameter_changes": [
    {"param": "参数路径", "old_value": "旧值", "new_value": "新值", "reason": "原因"}
  ],
  "risks": ["风险1", "风险2"],
  "estimated_effort": "low|medium|high"
}

工作目录：/home/skye/application-pnc
```

---

## 执行计划

### 阶段 1：并行派发 7 个子 Agent（分析轮）

每个子 Agent 按上述模板分析 1 道赛题。7 个 Agent 并行派发。

各赛题的**重点分析线索**（写进子 Agent 提示词中）：

| 赛题 | 重点分析线索 |
|------|-------------|
| 赛题一 交通灯 | TrafficLight TrafficRule + traffic_light_unprotected_right_turn Scenario。关键是「停车距离 1.5-2.0m 可调」和「红灯可右转」|
| 赛题二 变道 | LaneChangePath Task + LaneChangePathGeneric Task + BacksideVehicle TrafficRule。关键是「后方来车检测」和「限速 30km/h」和「禁止连续变道」|
| 赛题三 S弯 | LaneFollow Scenario（默认场景）+ LargeCurvature Scenario。关键是「平滑轨迹」和「障碍物绕行横向距离」和「限速 30km/h」。重点看 PathBounds 和 path_decider 的横向距离参数|
| 赛题四 U-Turn | 无现有 U-Turn 场景！需新建 Scenario 或扩展 FreeSpace/Square。关键是「掉头区域判断」「对向来车检测」「绕行横向距离 ≥ 1m」。重点看 Hybrid A* 和 open_space 模块|
| 赛题五 施工区域 | 本质上是一个 LaneFollow + 限速 + 绕行。可能只需调整 speed_setting TrafficRule 参数或新建一个简单的 TrafficRule。关键是「锥桶识别为障碍物」和「区域限速 30km/h 的起止边界」|
| 赛题六 站点接驳 | PullOver Scenario 最接近。关键是「精准停车位置」「5 秒静止等待」「环境观察后驶离」。查看 pull_over 场景的 Stage 流程和 destination TrafficRule|
| 赛题七 环岛 | 无现成的环岛场景！需新建 Roundabout Scenario。关键是「入口停车让行」「环岛内车道选择」「出口转向灯」「绕行横向距离 ≥ 1m」。参考 stop_sign_unprotected 的入口逻辑和 lane_change 的车道选择逻辑|

### 阶段 2：交叉审查（审核轮）

对每个赛题的解题方案，重新创建新的子 Agent 进行审查。审查 Agent 只验证：
1. 对现有 Apollo 代码的理解是否准确
2. 建议的修改方案是否合理可行
3. 是否有遗漏的关键技术点

### 阶段 3：汇总输出

生成两份产出：

#### 产出 A：`output/competition_solutions.md`
包含：
- 总览表（7 道赛题的解决策略一览）
- 每道赛题的详细解题方案
- 跨赛题的共性修改建议（如限速 30km/h 被多题使用，一次性改好）

#### 产出 B：`output/competition_module_mapping.md`
包含：
- 赛题 ↔ Apollo 模块映射矩阵
- 需要新建/修改的模块清单
- 参数变更汇总表
- 修改优先级和依赖关系

---

## 输出要求
- 所有输出为中文
- 解题方案采用 Markdown 格式，含代码示例
- 文件保存到 `output/` 目录
- 完成后输出「任务完成报告」：分析轮 Agent 数、审查轮 Agent 数、失败/重试次数
