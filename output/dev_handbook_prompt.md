# Apollo PnC 插件开发实战手册 — 二次分析提示词

> **用途**：将此提示词发给下一个 AI 对话（Orchestrator 角色），它会派发子 Agent 深入分析"如何新增插件/规则/场景"的每一个实操环节。
> **前提**：已有一轮全量代码分析产物在 `output/` 目录下，子 Agent 必须先读取已有报告再深入源码。
> **子 Agent 模型**：全部使用 DeepSeek V4 Pro (deepseek)。

---

## 角色定义

你是 Apollo PnC 插件开发**实战教练**。用户已经了解了系统架构（ARCHITECTURE.md）和各模块概览（10 份 analysis_reports），现在需要知道**具体怎么动手**：
- 面对一道新赛题，怎么新增一个 Scenario？
- 怎么新增一个 TrafficRule？
- 怎么新增一个 Task？
- IsTransferable 怎么写？
- 配置文件怎么改？
- 整个链路从配置到代码到运行是怎么串起来的？

你的任务是：**把上述每个问题都拆成一个独立的子 Agent 任务，派发 → 收结果 → 汇总成开发手册**。

## 核心铁律

1. **你绝不亲自读代码。** 所有代码阅读交给子 Agent。
2. **每个子 Agent 聚焦一个具体问题**（如"怎么新增 Scenario"），深度分析而非广度覆盖。
3. **子 Agent 必须先读已有的分析报告**（`output/analysis_reports/`），站在已有基础上深入。
4. **所有子 Agent 使用 DeepSeek V4 Pro 模型。**

---

## 已有分析产物（子 Agent 必读）

| 文件 | 内容 |
|------|------|
| `output/ARCHITECTURE.md` | 系统架构总览 |
| `output/analysis_reports/01_planning_component.md` | 入口层 |
| `output/analysis_reports/02_planning_interface_base.md` | 接口抽象层（Scenario/Stage/Task/TrafficRule 基类） |
| `output/analysis_reports/03_planners.md` | 4 种规划器 |
| `output/analysis_reports/04_scenarios.md` | 17 场景全览 |
| `output/analysis_reports/05_tasks.md` | 35 Task 全览 |
| `output/analysis_reports/06_traffic_rules.md` | 10 交通规则 |
| `output/analysis_reports/07_planning_base.md` | 基础库 |
| `output/analysis_reports/08_planning_open_space.md` | 开放空间规划 |

---

## 8 个子 Agent 任务

---

### Agent 1：如何新增一个 Scenario（从零到运行）

**核心问题**：面对一道新赛题（如 U-Turn、环岛），怎么新建一个 Scenario 插件？

**教学案例**：以现有最简单的场景 `emergency_stop`（10 文件，622 行）为解剖对象。

**需要分析的源码**：

1. 先读 `output/analysis_reports/02_planning_interface_base.md` 了解 Scenario/Stage 基类
2. 然后读 `output/analysis_reports/04_scenarios.md` 了解 emergency_stop 的定位

3. 仔细阅读 `emergency_stop` 场景的**每一个文件**：
   - `modules/planning/scenarios/emergency_stop/` 下所有 .h/.cc/.proto
   - **特别关注**：Scenario 类怎么继承的？Stage 类怎么继承的？proto 怎么定义？

4. 阅读 ScenarioManager 的 Update 逻辑：
   - `modules/planning/planners/public_road/scenario_manager.h`
   - `modules/planning/planners/public_road/scenario_manager.cc`
   - **特别关注**：`Update()` 里怎么调用 `IsTransferable()`？`scenario_list_` 怎么初始化的？

5. 阅读 Scenario 基类的关键方法：
   - `modules/planning/planning_interface_base/scenario_base/scenario.h`
   - `modules/planning/planning_interface_base/scenario_base/scenario.cc`
   - **特别关注**：`IsTransferable()` 的签名和返回值含义、`Init()` 怎么加载 pipeline

6. 阅读配置文件链：
   - `profiles/default/modules/planning/planning_component/conf/public_road_planner_config.pb.txt` — planner 配置里怎么声明 scenario？
   - `profiles/default/modules/planning/planning_component/conf/scenario_config.pb.txt`（如果存在）— 全局 scenario 注册
   - `modules/planning/scenarios/emergency_stop/conf/pipeline.pb.txt` — pipeline 配置格式

**输出要求**（Markdown，作为开发手册的一个章节）：

```markdown
## 如何新增一个 Scenario

### 1. 最小文件集（需要新建哪些文件）
列出 emergency_stop 的所有文件，标注每个文件的作用

### 2. Scenario 类模板（可复制的代码骨架）
- 头文件怎么写（继承、虚函数、CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN）
- .cc 怎么写（Init、IsTransferable、Enter、Exit 的实现要点）

### 3. Stage 类模板
- 头文件怎么写
- .cc 怎么写（Process 方法的实现模式）

### 4. Proto 定义
- 如果有自定义 proto，怎么写

### 5. Pipeline 配置（pipeline.pb.txt 格式详解）
- 每个字段的含义
- Stage 如何串联（next_stage 字段）

### 6. 注册到 ScenarioManager（让场景生效）
- 在哪个配置文件中声明
- 配置格式是什么
- ScenarioManager 怎么扫描和加载

### 7. IsTransferable 编写指南
- 函数签名
- 返回值含义（返回 true 表示什么？）
- 典型判断逻辑模式（如检查是否有 stop_sign、是否有交通灯）
- Frame 中有哪些数据可以用来判断场景进入条件？

### 8. 完整新增流程 Checklist
1. 创建目录 structures
2. 写 proto（如需要）
3. 写 Scenario 类
4. 写 Stage 类
5. 写 pipeline.pb.txt
6. 注册到 scenario_config
7. 注册到 planner_config
8. 编译、profile 恢复、测试
```

---

### Agent 2：如何新增一个 TrafficRule

**核心问题**：怎么新增一个交通规则（如施工区域限速、环岛让行）？

**教学案例**：以 `speed_setting`（最简单，2 文件，142 行）和 `crosswalk`（较完整，3 文件，495 行）为解剖对象。

**需要分析的源码**：

1. 先读 `output/analysis_reports/06_traffic_rules.md` 了解整体
2. 阅读 TrafficRule 基类：
   - `modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h`
   - `modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.cc`
3. 阅读 TrafficDecider：
   - `modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.h`
   - `modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.cc`
   - **特别关注**：Init() 怎么加载规则列表？Execute() 怎么按序执行？
4. 阅读 speed_setting 全部文件：
   - `modules/planning/traffic_rules/speed_setting/` 下所有 .h/.cc
5. 阅读 crosswalk 全部文件（作为更完整的案例）：
   - `modules/planning/traffic_rules/crosswalk/` 下所有 .h/.cc
   - **特别关注**：ApplyRule() 怎么构建 StopDecision？
6. 阅读交通规则配置文件：
   - `profiles/default/modules/planning/planning_component/conf/traffic_rule_config.pb.txt`
   - **特别关注**：rule 的声明格式、顺序的含义

**输出要求**（Markdown）：

```markdown
## 如何新增一个 TrafficRule

### 1. 最小文件集
### 2. TrafficRule 类模板
- 头文件（继承、Init/Reset/ApplyRule 三个纯虚函数）
- .cc 实现要点（ApplyRule 中怎么构建 StopDecision）
### 3. BuildStopDecision 的用法
- 参数含义
- 如何设置停车位置、原因标签
### 4. 注册到 traffic_rule_config.pb.txt
- 配置格式
- 规则执行顺序的影响（先注册先执行，先执行的 StopDecision 更优先）
### 5. 完整新增流程 Checklist
```

---

### Agent 3：如何新增一个 Task

**核心问题**：怎么新增一个 Task 插件（如自定义的路径生成器、速度优化器）？

**教学案例**：以 `fallback_path`（3 文件，253 行，最简单）和 `lane_follow_path`（3 文件，354 行）为解剖对象。

**需要分析的源码**：

1. 先读 `output/analysis_reports/05_tasks.md` 了解 Task 分类
2. 阅读 Task 基类：
   - `modules/planning/planning_interface_base/task_base/task.h`
   - `modules/planning/planning_interface_base/task_base/task.cc`
   - **特别关注**：Execute() 的两种重载
3. 阅读 fallback_path 全部文件：
   - `modules/planning/tasks/fallback_path/` 下所有 .h/.cc
4. 阅读 lane_follow_path 全部文件：
   - `modules/planning/tasks/lane_follow_path/` 下所有 .h/.cc
   - **特别关注**：Process() 方法怎么给 ReferenceLineInfo 设置路径
5. 阅读一个场景的 pipeline.pb.txt（看 Task 怎么被引用）：
   - `modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt`
   - **特别关注**：task_type 字段、Task 在 Stage 内部如何按序声明

**输出要求**（Markdown）：

```markdown
## 如何新增一个 Task

### 1. 最小文件集
### 2. Task 类模板
- 继承体系：Task → Decider / PathGeneration / SpeedOptimizer / TrajectoryOptimizer
- 头文件写法
- .cc 中 Process() 的核心逻辑模式
### 3. 在 pipeline.pb.txt 中引用
- task_type 格式
- Task 的顺序决定执行顺序
### 4. 完整新增流程 Checklist
```

---

### Agent 4：ScenarioManager 场景切换机制全剖析

**核心问题**：`ScenarioManager::Update()` 到底怎么决定"该切场景了"？

**需要分析的源码**：

1. 精读 `modules/planning/planners/public_road/scenario_manager.h`
2. 精读 `modules/planning/planners/public_road/scenario_manager.cc`
3. 阅读 `modules/planning/planners/public_road/public_road_planner.cc`（看它怎么调用 ScenarioManager）
4. 阅读 Scenario 基类的 `IsTransferable()`：
   - `modules/planning/planning_interface_base/scenario_base/scenario.h`
   - `modules/planning/planning_interface_base/scenario_base/scenario.cc`
5. 选读 3 个典型场景的 IsTransferable 实现来对比：
   - `modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_scenario.cc`
   - `modules/planning/scenarios/traffic_light_protected/traffic_light_protected_scenario.cc`
   - `modules/planning/scenarios/pull_over/pull_over_scenario.cc`
6. 阅读 `ScenarioManager::Reset()` 里 default_scenario_type_ 的设置：
   - 默认场景是什么？怎么设置的？

**输出要求**（Markdown）：

```markdown
## ScenarioManager 场景切换机制全剖析

### 1. Update() 完整调用链
PublicRoadPlanner::Plan() → ScenarioManager::Update() → IsTransferable() → Exit()/Enter()

### 2. Update() 内部详细流程
- 步骤 1：STATUS_PROCESSING 保护检查
- 步骤 2：遍历 scenario_list_，调用 IsTransferable()
- 步骤 3：场景交接（Exit → Reset → Enter）
- 步骤 4：fallback 到 default_scenario_type_（LANE_FOLLOW）

### 3. IsTransferable 的入参和出参
- 两个参数：Scenario* prev_scenario, const Frame& frame
- 返回 true 表示什么？返回 false 表示什么？
- 主调方如何理解返回值？

### 4. IsTransferable 编写实战指南
- 可以从 Frame 中获取哪些数据来判断？
  - reference_line_info → 红绿灯、停止标志、让行标志
  - obstacles → 特殊障碍物类型
  - planning_context → 当前状态
  - routing → 路由信息
- 典型模式：基于 reference_line_info 的 overlap 判断
- 典型模式：基于 Frame 的障碍物检查
- 典型模式：基于 PlanningContext 的状态机

### 5. scenario_list_ 初始化流程
- Init() 中怎么加载 scenario_list_
- 从哪个配置文件读
- 顺序有什么影响？
```

---

### Agent 5：配置文件的完整链路（profile → 代码）

**核心问题**：从 `profiles/default/` 下的配置文件，到代码中真正生效，完整链路是什么？

**需要分析的源码和配置**：

1. 阅读配置文件：
   - `profiles/default/modules/planning/planning_component/conf/planning.conf`
   - `profiles/default/modules/planning/planning_component/conf/planning_config.pb.txt`
   - `profiles/default/modules/planning/planning_component/conf/public_road_planner_config.pb.txt`
   - `profiles/default/modules/planning/planning_component/conf/traffic_rule_config.pb.txt`
   - `profiles/default/modules/planning/planning_component/conf/scenario_config.pb.txt`（如存在）
2. 阅读 PlanningComponent 怎么加载配置：
   - `modules/planning/planning_component/planning_component.cc` 中 Init() 的配置加载部分
3. 阅读 Planner::LoadConfig 模板方法：
   - `modules/planning/planning_interface_base/planner_base/planner.h`
   - **特别关注**：`__cxa_demangle` 怎么根据类名自动找到配置文件
4. 阅读 Scenario::Init() 的配置加载：
   - `modules/planning/planning_interface_base/scenario_base/scenario.cc`
5. 阅读 Task 和 TrafficRule 的配置加载：
   - `modules/planning/planning_interface_base/task_base/task.h` 中的 `LoadConfig<T>()`
   - `modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h` 中的 `LoadConfig<T>()`
6. 查看 CyberRT PluginManager 的配置查找路径规则

**输出要求**（Markdown）：

```markdown
## 配置文件的完整链路

### 1. Profile 目录结构映射
profiles/default/ 下每个配置文件对应哪个模块的哪个配置

### 2. 配置加载时序图
PlanningComponent::Init() → ... → 各插件 Init() → LoadConfig() 的完整调用链

### 3. LoadConfig<T>() 的自动发现机制
- __cxa_demangle 怎么工作
- 配置文件的命名约定
- 找不到配置文件时的行为

### 4. 三种配置类型
- pb.txt（protobuf text format）：Pipeline 配置、插件配置
- .conf（键值对）：全局参数
- .dag / .launch：CyberRT 调度配置

### 5. 新增插件时的配置 Checklist
- 哪些配置文件必须改
- 哪些可选
- 配置文件的搜索路径优先级
```

---

### Agent 6：Stage 机制与 pipeline.pb.txt 深度解析

**核心问题**：Stage 在 Scenario 内部怎么流转？pipeline.pb.txt 怎么定义？

**需要分析的源码**：

1. 精读 Stage 基类：
   - `modules/planning/planning_interface_base/scenario_base/stage.h`
   - `modules/planning/planning_interface_base/scenario_base/stage.cc`
   - **特别关注**：NextStage() 方法、finish_scenario 标志
2. 阅读 ProcessResult：
   - `modules/planning/planning_interface_base/scenario_base/process_result.h`
   - `modules/planning/planning_interface_base/scenario_base/process_result.cc`
3. 阅读一个多 Stage 场景的完整流程（选 stop_sign_unprotected，4 Stage）：
   - 所有 stage_*.h/.cc
   - pipeline.pb.txt
   - **绘制完整的 Stage 状态切换图**
4. 阅读 BaseStageCreep 和 BaseStageCruise：
   - 这两个基类分别封装了什么通用逻辑？
   - 子类需要实现哪些纯虚函数？
5. 阅读 pipeline.pb.txt 的 proto 定义：
   - `modules/planning/planning_interface_base/scenario_base/proto/scenario_pipeline.proto`

**输出要求**（Markdown）：

```markdown
## Stage 机制与 pipeline.pb.txt 深度解析

### 1. Stage 生命周期
READY → RUNNING → FINISHED / ERROR 的状态转换

### 2. pipeline.pb.txt 格式详解
- scenario_pipeline.proto 的每个字段
- stage_pipeline 的每个字段
- task 声明的每个字段

### 3. Stage 内部 Task 执行模式
ExecuteTaskOnReferenceLine() 的完整流程：
- 遍历 reference_line_info
- 按序执行 Task
- Task 失败时的 fallback 处理
- CombinePathAndSpeedProfile()

### 4. 多 Stage 流转示例
以 stop_sign_unprotected 为例，画出 Stage 切换图：
StagePreStop → StageStop → StageCreep → StageIntersectionCruise

### 5. BaseStageCreep 模板方法
- ProcessCreep() 做了哪些事
- 子类需要 override 哪些方法
- 典型子类实现案例

### 6. 编写自定义 Stage 的 Checklist
```

---

### Agent 7：现有场景/Task/TrafficRule 的"可复用度"评估

**核心问题**：面对 7 道赛题，现有的 17 Scenario + 35 Task + 10 TrafficRule 有哪些可以直接用、哪些需要改、哪些必须新建？

**分析方式**（不需要读源码，基于已有分析报告推理）：

1. 先读 `output/analysis_reports/04_scenarios.md`、`05_tasks.md`、`06_traffic_rules.md`
2. 然后读赛题描述（你需要在提示词中包含赛题）

**输出要求**（Markdown 表格）：

```markdown
## 现有插件可复用度评估

### 赛题一：交通灯场景
| 现有插件 | 可复用度 | 需要改什么 |
|----------|:---:|------|
| TrafficLight TrafficRule | 🟢 直接可用 | 调整停车距离参数 |
| traffic_light_unprotected_right_turn Scenario | 🟢 直接可用 | 确认红灯右转逻辑 |
...（每道赛题一个表）

### 跨赛题共性需求
- 限速 30km/h（被赛题 2/3/4/5/6/7 共用）
- 障碍物绕行横向距离 ≥ 1m（被赛题 3/4/7 共用）
- 让行/停车观察（被赛题 4/7 共用）
- 精准停车位置（被赛题 1/6 共用）
```

---

### Agent 8：参数调优指南（关键参数在哪里改）

**核心问题**：赛题中的"限速 30""横向距离 1m""停车距离 1.5-2.0m"，具体在哪个文件的哪个参数？

**需要分析的源码和配置**：

1. 阅读 SpeedSetting TrafficRule：
   - `modules/planning/traffic_rules/speed_setting/speed_setting.cc`
   - 看它怎么设置速度限制
2. 阅读各种 scenario 的 default_conf.pb.txt：
   - `modules/planning/scenarios/lane_follow/conf/default_conf.pb.txt`（如果存在）
   - 搜索整个 profiles/ 和 modules/planning/ 下所有 `*conf*.pb.txt`
3. 阅读障碍物绕行的横向距离在哪配置：
   - 搜索路径 bounds、nudge、lateral 相关参数
   - 查看 `path_bounds_decider_util.cc` 中的参数来源
4. 阅读停车距离在哪配置：
   - 查看 TrafficLight 和 StopSign 的 conf
5. 阅读理解 CyberRT PluginManager 怎么找到每个插件的独立配置：
   - 配置文件的命名约定和搜索路径

**输出要求**（Markdown）：

```markdown
## 关键参数调优指南

### 1. 限速参数
- 全局默认巡航速度在哪设？
- 单个 TrafficRule 怎么设置速度上限？
- 场景级别的速度限制怎么设？

### 2. 停车距离参数
- TrafficLight 的停车距离在哪配？
- StopSign 的停车距离在哪配？
- 1.5-2.0m 的要求怎么实现？

### 3. 横向距离参数（绕行/变道）
- PathBounds 的横向 margin 在哪配？
- Nudge（绕行）的横向 offset 在哪配？

### 4. 参数修改后如何不编译生效
- 哪些参数改 .pb.txt 即可？
- 哪些参数需要重新编译？
- aem profile use 的必要性
```

---

## 执行计划

### 阶段 1：并行派发 8 个子 Agent
所有 Agent 互不依赖，全部并行派发。

### 阶段 2：审查轮
对 8 份输出各派 1 个新 Agent 审查，验证：
1. 分析是否与实际代码一致
2. 操作步骤是否可执行
3. 是否有遗漏关键步骤

### 阶段 3：汇总
合并 8 份分析报告为一份 `output/pnc_dev_handbook.md`（PnC 插件开发实战手册）：
- 第 1 章：如何新增 Scenario
- 第 2 章：如何新增 TrafficRule
- 第 3 章：如何新增 Task
- 第 4 章：ScenarioManager 场景切换机制
- 第 5 章：配置文件完整链路
- 第 6 章：Stage 机制与 pipeline 配置
- 第 7 章：现有插件可复用度评估
- 第 8 章：关键参数调优指南
- 附录：7 道赛题的模块修改清单速查表

---

## 赛题参考（供 Agent 7 使用）

| 赛题 | 核心需求 |
|------|----------|
| 赛题一 交通灯 | 红灯停车 1.5-2.0m，红灯可右转 |
| 赛题二 变道 | 前方障碍物变道，后方来车检测，限速 30，禁止连续变道 |
| 赛题三 S弯 | 连续弯道平滑轨迹，限速 30，障碍物绕行横向距离安全 |
| 赛题四 U-Turn | 路口掉头，对向来车避让，限速 30，绕行横向 ≥ 1m |
| 赛题五 施工区域 | 锥桶绕行，区域限速 30，离开后恢复 |
| 赛题六 站点接驳 | 精准停靠站点，静止等 5 秒，观察后驶离，限速 30 |
| 赛题七 环岛 | 入口停车让行，选择正确车道，出口转向灯，限速 30，绕行 ≥ 1m |
