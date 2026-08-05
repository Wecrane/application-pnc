# Apollo PnC 插件开发实战手册

> 生成时间：2026-06-16 | 基于 8 份深度源码分析 + 4 轮交叉审查
> 配套文档：`output/ARCHITECTURE.md` + `output/analysis_reports/`

---

## 目录

- [第 1 章：如何新增一个 Scenario](#第-1-章如何新增一个-scenario)
- [第 2 章：如何新增一个 TrafficRule](#第-2-章如何新增一个-trafficrule)
- [第 3 章：如何新增一个 Task](#第-3-章如何新增一个-task)
- [第 4 章：ScenarioManager 场景切换机制](#第-4-章-scenariomanager-场景切换机制)
- [第 5 章：配置文件完整链路](#第-5-章配置文件完整链路)
- [第 6 章：Stage 机制与 pipeline 配置](#第-6-章-stage-机制与-pipeline-配置)
- [第 7 章：现有插件可复用度评估](#第-7-章现有插件可复用度评估)
- [第 8 章：关键参数调优指南](#第-8-章关键参数调优指南)
- [附录：7 道赛题模块修改清单速查表](#附录7-道赛题模块修改清单速查表)

---

## 第 1 章：如何新增一个 Scenario

> **教学案例**：`emergency_stop`（10 文件，622 行，3 Stage）

### 1.1 最小文件集

| 文件 | 作用 |
|------|------|
| `<scenario>_scenario.h` | Scenario 类声明（继承 `Scenario`） |
| `<scenario>_scenario.cc` | Scenario 类实现（`Init`、`IsTransferable`、`Enter`、`Exit`） |
| `stage_<name>.h` | 每个 Stage 的类声明（继承 `Stage` 或 `BaseStageCreep`/`BaseStageCruise`） |
| `stage_<name>.cc` | 每个 Stage 的类实现（`Process` 方法） |
| `proto/<scenario>.proto` | 自定义场景配置 proto 消息定义（可选，如需要独有参数） |
| `proto/BUILD` | Proto 编译规则（如需要） |
| `conf/pipeline.pb.txt` | **Stage 编排配置**（最重要）：声明 Stage 列表及每个 Stage 的 Task 链 |
| `conf/scenario_conf.pb.txt` | 场景参数配置（如 `stop_distance`、`max_stop_deceleration`） |
| `BUILD` | Bazel 构建规则（`apollo_plugin` 宏） |
| `plugins.xml` | 插件注册描述文件 |
| `cyberfile.xml` | 包元数据（package name、依赖） |

### 1.2 Scenario 类模板

#### 头文件骨架

```cpp
#pragma once

#include <memory>
#include <string>

// 【1】引入自定义 proto（如需要）
#include "modules/planning/scenarios/<name>/proto/<name>.pb.h"

// 【2】引入基类
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

// 【3】自定义 Context 结构体（继承 ScenarioContext）
//    用于在 Scenario 和 Stage 之间传递场景级共享数据
struct MyScenarioContext : public ScenarioContext {
  ScenarioMyScenarioConfig scenario_config;
};

class MyScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  // 【4】返回自定义 Context，供 Stage 通过 GetContextAs<T>() 访问
  MyScenarioContext* GetContext() override { return &context_; }

  // 【5】场景切换条件（核心方法）
  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;

  // 【6】场景主处理入口
  ScenarioResult Process(const common::TrajectoryPoint& planning_init_point,
                         Frame* frame) override;

  // 【7】场景切入时的初始化逻辑（可选重写）
  bool Enter(Frame* frame) override { return true; }

  // 【8】退出清理
  bool Exit(Frame* frame) override;

 private:
  bool init_ = false;
  MyScenarioContext context_;
};

// 【9】⭐ 关键宏：将 Scenario 类注册到 PluginManager
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyScenario, Scenario)

}  // namespace planning
}  // namespace apollo
```

#### .cc 实现要点

```cpp
bool MyScenario::Init(std::shared_ptr<DependencyInjector> injector,
                      const std::string& name) {
  // 1. 检查是否已初始化
  if (init_) return true;

  // 2. 调用基类 Init（加载 pipeline.pb.txt + scenario_conf.pb.txt）
  if (!Scenario::Init(injector, name)) {
    AERROR << "Failed to init scenario: " << Name();
    return false;
  }

  // 3. 加载场景独有配置（如需要）
  if (!LoadConfig<ScenarioMyScenarioConfig>(
          &context_.scenario_config)) {
    AERROR << "Failed to load config of scenario: " << Name();
    return false;
  }

  init_ = true;
  return true;
}

bool MyScenario::IsTransferable(const Scenario* other_scenario,
                                const Frame& frame) {
  // 典型判断逻辑（详见 §1.7）
  // 返回 true = 可以从 other_scenario 切入本场景
  // 返回 false = 不切入，ScenarioManager 继续遍历下一个场景
  return /* 你的判断条件 */;
}

bool MyScenario::Enter(Frame* frame) {
  // 场景切入时的初始化（例如重置状态变量）
  return true;
}

bool MyScenario::Exit(Frame* frame) {
  // 场景退出时的清理
  return true;
}
```

### 1.3 Stage 类模板

#### 头文件骨架

```cpp
#pragma once

#include <string>

#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace planning {

class MyScenarioStageApproach : public Stage {
 public:
  explicit MyScenarioStageApproach(
      const StagePipeline& stage_pipeline,
      const ScenarioContext* context)
      : Stage(stage_pipeline, context) {}

  // 核心方法：每帧调用，返回 StageResult 驱动状态转换
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  // 可从 ScenarioContext 获取场景级配置
  const MyScenarioContext* GetContext() const {
    return GetContextAs<MyScenarioContext>();
  }
};

}  // namespace planning
}  // namespace apollo
```

#### .cc 实现模式（以 emergency_stop StageApproach 为参考）

```cpp
StageResult MyScenarioStageApproach::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {

  // 1. 执行 pipeline 中配置的 Task 链
  auto stage_result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (stage_result.IsError()) {
    return stage_result;
  }

  // 2. 判断是否应该切换到下一 Stage
  if (/* 完成条件 */) {
    // 方法 A: 切换到 pipeline 中声明的 next_stage
    return FinishStage();
    // 方法 B: 结束整个 Scenario（最后一个 Stage 用）
    // return FinishScenario();
  }

  // 3. 仍在处理中
  return stage_result.SetStageStatus(StageStatusType::RUNNING);
}
```

**关键方法**：
| 方法 | 作用 |
|------|------|
| `FinishStage()` | 标记当前 Stage 完成，切换到 pipeline 中声明的 `next_stage` |
| `FinishScenario()` | 标记整个 Scenario 完成，设置 `next_stage_ = ""`，下一个周期 Scenario 返回 `STATUS_DONE` |
| `ExecuteTaskOnReferenceLine()` | 遍历所有 ReferenceLine，按 pipeline 顺序执行 Task 链，自动调用 `CombinePathAndSpeedProfile()` |

### 1.4 Proto 定义（可选）

如果场景需要独有参数，定义 proto：

```protobuf
syntax = "proto2";
package apollo.planning;

message ScenarioMyScenarioConfig {
  optional double max_stop_deceleration = 1 [default = 6.0];
  optional double stop_distance = 2 [default = 1.0];
}
```

### 1.5 Pipeline 配置（pipeline.pb.txt 格式详解）

```protobuf
stage: {
  name: "MY_SCENARIO_APPROACH"       # Stage 别名（唯一标识）
  type: "MyScenarioStageApproach"    # C++ 类名（不含命名空间）
  enabled: true                       # 是否启用
  task: {
    name: "PATH_DECIDER"              # Task 别名
    type: "PathDecider"               # Task C++ 类名
  }
  task: {
    name: "SPEED_DECIDER"
    type: "SpeedDecider"
  }
  # ...更多 Task
  # fallback_task: { ... }            # 可选，Task 链失败时的兜底 Task
}
stage: {
  name: "MY_SCENARIO_STANDBY"
  type: "MyScenarioStageStandby"
  task: { name: "..."; type: "..." }
  # next_stage 不需要显式声明——Stage 按 pipeline 中声明的顺序自动串联
}
```

**字段说明**：
| 字段 | 说明 |
|------|------|
| `stage.name` | Stage 的唯一标识名，用于内部查找 |
| `stage.type` | Stage 的 C++ 类名，PluginManager 用它创建实例 |
| `stage.enabled` | 默认 true。false 时跳过该 Stage |
| `stage.task` | Task 列表，按声明顺序执行 |
| `stage.fallback_task` | 可选。未设置时默认为 `FastStopTrajectoryFallback` |

**Stage 串联机制**：Pipeline 中声明的 Stage 列表按顺序构成 Stage 链。第一个 Stage 是入口 Stage。每个 Stage 调用 `FinishStage()` 后自动切换到下一个 Stage。最后一个 Stage 调用 `FinishScenario()` 结束整个 Scenario。

### 1.6 注册到 ScenarioManager

#### 在 `public_road_planner_config.pb.txt` 中声明

```protobuf
scenario: {
  name: "MyScenario"
  type: "MyScenario"
}
```

Scenario 列表的顺序**影响优先级**：ScenarioManager 按列表顺序遍历，第一个 `IsTransferable()` 返回 true 的场景会被切入。`LANE_FOLLOW` 通常放在最后作为兜底。

#### ScenarioManager 加载流程

```cpp
// scenario_manager.cc Init()
for (const auto& scenario_config : planner_config.scenario()) {
  auto scenario = PluginManager::CreateInstance<Scenario>(
      ConfigUtil::GetFullPlanningClassName(scenario_config.type()));
  scenario->Init(injector_, scenario_config.name());
  scenario_list_.push_back(scenario);
}
current_scenario_ = default_scenario_type_;  // 初始化为 LANE_FOLLOW
```

### 1.7 IsTransferable 编写指南

#### 函数签名

```cpp
virtual bool IsTransferable(const Scenario* other_scenario,
                            const Frame& frame);
```

- `other_scenario`：当前正在运行的场景（可能为 nullptr，表示初始状态）
- `frame`：当前帧的完整规划数据
- **返回 true**：可以从 `other_scenario` 切入本场景
- **返回 false**：不切入，ScenarioManager 继续遍历下一个场景

#### Frame 中可用的判断数据

| 数据来源 | 获取方式 | 可判断的内容 |
|----------|----------|-------------|
| 参考线信息 | `frame.reference_line_info().front()` | 红绿灯/停止牌/让行标志 overlap |
| 障碍物 | `frame.obstacles()` | 特殊障碍物类型（锥桶等） |
| 规划上下文 | `frame.planning_context()` | 当前规划状态机 |
| 路由信息 | `frame.routing()` | 路由请求、终点距离 |
| ADC 状态 | `frame.PadUp()`, `frame.EgoSpeed()` | 接管状态、车速 |

#### 典型判断模式

**模式一：基于 reference_line_info 的 overlap 判断**（最常用）

```cpp
// 检查参考线上是否有停止标志 overlap
const auto& reference_line_info = frame.reference_line_info().front();
const auto& stop_sign_overlaps =
    reference_line_info.reference_line().map_path().stop_sign_overlaps();
if (!stop_sign_overlaps.empty()) {
  return true;  // 有停止标志，切入 stop_sign 场景
}
```

**模式二：永远可切入（兜底场景用）**

```cpp
// lane_follow 作为默认场景，永远返回 true
bool LaneFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  return true;
}
```

**模式三：基于 PlanningContext 的状态机**

```cpp
// pull_over 场景：检查是否收到靠边停车指令
auto* planning_status = frame.planning_context()->mutable_planning_status();
if (planning_status->has_pull_over() &&
    planning_status->pull_over().is_pull_over()) {
  return true;
}
```

### 1.8 BUILD / plugins.xml / cyberfile.xml 模板

#### BUILD 文件

```python
load("//tools/build_defs:apollo_plugin.bzl", "apollo_plugin")

apollo_plugin(
    name = "libmy_scenario.so",
    srcs = [
        "my_scenario_scenario.cc",
        "stage_approach.cc",
        "stage_standby.cc",
    ],
    hdrs = [
        "my_scenario_scenario.h",
        "stage_approach.h",
        "stage_standby.h",
    ],
    deps = [
        "//modules/planning/planning_interface_base/scenario_base:scenario_base",
        "//modules/planning/planning_base:planning_base",
        # ... 其他依赖
    ],
    description = "plugins.xml",
)
```

#### plugins.xml

```xml
<library path="modules/planning/scenarios/my_scenario/libmy_scenario.so">
    <class type="apollo::planning::MyScenario" base_class="apollo::planning::Scenario"></class>
</library>
```

> ⚠️ **注意**：`path` 必须与 BUILD 中 `apollo_plugin` 规则的 `name` 属性匹配（去掉 `lib` 前缀 + `.so` 后缀），`type` 使用包含命名空间的完整类名，`base_class` 必须是 `apollo::planning::Scenario`。

#### cyberfile.xml

```xml
<package>
  <name>planning-scenarios-my-scenario</name>
  <version>0.0.1</version>
  <description>My custom scenario</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <dependencies>
    <dependency name="planning-scenario-base"/>
  </dependencies>
</package>
```

### 1.9 完整新增流程 Checklist

1. [ ] 创建目录结构 `modules/planning/scenarios/<name>/`
2. [ ] 创建子目录 `proto/`、`conf/`
3. [ ] 写 proto 定义（如需要）+ `proto/BUILD`
4. [ ] 写 Scenario 类（`.h` + `.cc`），实现 `Init`/`IsTransferable`/`Enter`/`Exit`
5. [ ] 写 Stage 类（`.h` + `.cc`），实现 `Process()`，最后一个 Stage 调用 `FinishScenario()`
6. [ ] 写 `conf/pipeline.pb.txt`，声明 Stage 列表和 Task 链
7. [ ] 写 `conf/scenario_conf.pb.txt`（如需要场景参数）
8. [ ] 写 `BUILD` 文件（`apollo_plugin` 宏）
9. [ ] 写 `plugins.xml`（`<library>` 格式）
10. [ ] 写 `cyberfile.xml`
11. [ ] 在 `public_road_planner_config.pb.txt` 中注册场景
12. [ ] 编译 → `aem profile use default` → 测试

---

## 第 2 章：如何新增一个 TrafficRule

> **教学案例**：`speed_setting`（2 文件，142 行，最简单）+ `crosswalk`（3 文件，495 行，完整案例）

### 2.1 最小文件集

**最简方案**（无自定义 proto/config，如 speed_setting）：

| 文件 | 作用 |
|------|------|
| `<rule>.h` | 类声明：继承 `TrafficRule`，声明 `Init`/`ApplyRule`/`Reset` |
| `<rule>.cc` | 类实现（~100 行） |
| `plugins.xml` | 插件注册描述 |
| `BUILD` | Bazel 编译规则 |
| `cyberfile.xml` | 包描述 |

**完整方案**（有自定义 proto/config，如 crosswalk）：

| 文件 | 作用 |
|------|------|
| 上述全部 + `proto/<rule>.proto` + `proto/BUILD` | 规则专用参数配置 |
| `conf/default_conf.pb.txt` | 默认参数值（运行时动态加载，改参无需编译） |

### 2.2 TrafficRule 类模板

#### 头文件骨架

```cpp
#pragma once

#include <memory>
#include <string>

#include "modules/planning/traffic_rules/<name>/proto/<name>.pb.h"  // 如有自定义 proto
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class MyRule : public TrafficRule {
 public:
  // ---- 三个必须重写的纯虚函数 ----
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  void Reset() override;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) override;

 private:
  MyRuleConfig config_;                                    // 规则专属配置（如有）
  std::vector<const hdmap::PathOverlap*> my_overlaps_;    // 规则状态变量
};

// 插件注册宏 — 必须放在类外部
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyRule, TrafficRule)

}  // namespace planning
}  // namespace apollo
```

#### .cc 实现要点

```cpp
bool MyRule::Init(const std::string& name,
                  const std::shared_ptr<DependencyInjector>& injector) {
  // 1. 调用基类 Init（通过 __cxa_demangle 自动加载 conf/default_conf.pb.txt）
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // 2. 加载规则专属配置
  LoadConfig<MyRuleConfig>(&config_);
  return true;
}

void MyRule::Reset() {
  // 每帧开始时重置规则状态
  my_overlaps_.clear();
}

common::Status MyRule::ApplyRule(Frame* const frame,
                                  ReferenceLineInfo* const reference_line_info) {
  // 1. 获取参考线上的相关 overlap
  const auto& my_overlaps = reference_line_info->reference_line()
                                .map_path().xxx_overlaps();

  // 2. 遍历判断
  for (const auto& overlap : my_overlaps) {
    if (/* 满足条件 */) {
      // 3. 构建停车决策
      util::BuildStopDecision(
          virtual_obstacle_id,           // 虚拟障碍物 ID（用于标识）
          overlap->start_s,              // overlap 起始 s 坐标
          config_.stop_distance(),       // 停车距离
          StopReasonCode::STOP_REASON_XXX, // 停车原因标签
          decision_tag,                  // 决策标签（如 "crosswalk"）
          frame,
          reference_line_info);
    }
  }
  return common::Status::OK();
}
```

### 2.3 BuildStopDecision 的用法

```cpp
void BuildStopDecision(
    const std::string& virtual_obstacle_id,   // 虚拟障碍物 ID
    const double stop_s,                        // 停车位置的 s 坐标
    const double stop_distance,                 // 停车距离（stop_s - stop_distance = 实际停车位置）
    const StopReasonCode& stop_reason_code,     // 停车原因（见 decision.proto）
    const std::string& decision_tag,            // 决策标签（用于调试）
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info);
```

**常用 `StopReasonCode`**：

| 枚举值 | 含义 | 使用场景 |
|--------|------|----------|
| `STOP_REASON_STOP_SIGN` | 停止牌 | stop_sign TrafficRule |
| `STOP_REASON_TRAFFIC_LIGHT` | 交通灯 | traffic_light TrafficRule |
| `STOP_REASON_CROSSWALK` | 人行横道 | crosswalk TrafficRule |
| `STOP_REASON_YIELD_SIGN` | 让行标志 | yield_sign TrafficRule |
| `STOP_REASON_DESTINATION` | 终点 | destination TrafficRule |
| `STOP_REASON_EMERGENCY` | 紧急停车 | emergency_stop Scenario |
| `STOP_REASON_CLEAR_ZONE` | 禁停区 | keepclear TrafficRule |
| `STOP_REASON_REFERENCE_LINE_END` | 参考线终点 | reference_line_end TrafficRule |

> 完整列表见 `modules/planning/proto/decision.proto`

### 2.4 TrafficDecider 执行流程

```cpp
// TrafficDecider::Init()
for (const auto& rule_config : rule_pipeline_.rule()) {
  auto rule = PluginManager::CreateInstance<TrafficRule>(
      ConfigUtil::GetFullPlanningClassName(rule_config.type()));
  rule->Init(rule_config.name(), injector);
  rule_list_.push_back(rule);
}

// TrafficDecider::Execute() — 每帧调用
for (auto& rule : rule_list_) {
  rule->Reset();                                       // 重置状态
  rule->ApplyRule(frame, reference_line_info);         // 执行规则（可能构建 StopDecision）
}
BuildPlanningTarget(reference_line_info);               // 汇总：取 s 最小的 StopDecision 作为最终停车目标
```

**规则顺序的影响**：规则按 `traffic_rule_config.pb.txt` 中声明的顺序执行。先执行的规则先构建 StopDecision。但最终停车目标由 `BuildPlanningTarget()` 取所有 StopDecision 中 **s 坐标最小的**（即最近的停车点）。

### 2.5 注册到 traffic_rule_config.pb.txt

```protobuf
rule: {
  name: "MY_RULE"
  type: "MyRule"
}
```

在规则列表中的位置决定其执行顺序。当前 9 条规则的执行顺序为：

```
destination → speed_setting → keepclear → crosswalk
→ traffic_light → stop_sign → yield_sign
→ reference_line_end → rerouting → backside_vehicle
```

### 2.6 完整新增流程 Checklist

1. [ ] 创建目录 `modules/planning/traffic_rules/<name>/`
2. [ ] 写 TrafficRule 类（`.h` + `.cc`）
3. [ ] 写 proto 定义（如需要）+ `conf/default_conf.pb.txt`（如需要）
4. [ ] 写 `BUILD`、`plugins.xml`、`cyberfile.xml`
5. [ ] 在 `traffic_rule_config.pb.txt` 中注册
6. [ ] 编译 → `aem profile use default` → 测试

---

## 第 3 章：如何新增一个 Task

> **教学案例**：`fallback_path`（3 文件，253 行，最简单）+ `lane_follow_path`（3 文件，354 行）

### 3.1 最小文件集

| 文件 | 作用 |
|------|------|
| `<task>.h` | Task 类声明 + `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN` 宏 |
| `<task>.cc` | Task 类实现（`Init`、`Process`） |
| `proto/<task>.proto` | Task 专属配置 proto（如需要） |
| `proto/BUILD` | Proto 编译规则（如需要） |
| `conf/default_conf.pb.txt` | Task 默认配置（protobuf text，运行时加载，改参无需编译） |
| `BUILD` | Bazel 构建规则（`apollo_plugin` 宏） |
| `plugins.xml` | 插件注册描述（`<library>` 格式） |
| `cyberfile.xml` | 包元数据 |

### 3.2 Task 继承体系

```
Task (基类)
├── PathGeneration    — 路径生成（fallback_path, lane_follow_path, lane_change_path...）
├── Decider           — 决策类（path_decider, speed_decider, rule_based_stop_decider...）
├── SpeedOptimizer    — 速度规划（speed_bounds_decider, piecewise_jerk_speed...）
└── TrajectoryOptimizer — 轨迹优化（piecewise_jerk_speed_nonlinear...）
```

**关键设计**：四个中间抽象类各自覆写 `Execute()`，在内部调用虚函数 `Process()`。子类只需实现 `Process()`：

```cpp
// PathGeneration::Execute() 的实际逻辑
apollo::common::Status PathGeneration::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);  // 保存 frame_, reference_line_info_
  return Process(frame, reference_line_info); // 调用子类实现的 Process()
}
```

### 3.3 Task 类模板

#### 头文件骨架

```cpp
#pragma once

#include <memory>
#include <string>

#include "modules/planning/tasks/<name>/proto/<name>.pb.h"  // 专属 proto
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

class MyTask : public PathGeneration {                 // 继承 PathGeneration/Decider/SpeedOptimizer 之一
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  // 核心方法：子类只需实现 Process()
  apollo::common::Status Process(Frame* frame,
                                 ReferenceLineInfo* reference_line_info) override;

  // 路径生成 Task 的三步标准流程（按需实现）
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);
  bool OptimizePath(const std::vector<PathBoundary>& boundaries,
                    std::vector<PathData>* candidate_path_data);
  bool AssessPath(std::vector<PathData>* candidate_path_data,
                  PathData* final_path);

  MyTaskConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyTask, Task)

}  // namespace planning
}  // namespace apollo
```

#### .cc 实现要点

```cpp
apollo::common::Status MyTask::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  // 1. 获取路径数据
  auto* path_data = reference_line_info->mutable_path_data();
  auto* heuristic_path_data = reference_line_info->mutable_fallback_path_data();

  // 2. 三步流程
  std::vector<PathBoundary> path_boundaries;
  if (!DecidePathBounds(&path_boundaries)) {
    // 失败则使用 fallback_path
    return Status(common::PLANNING_ERROR, "Failed to decide path bounds");
  }

  std::vector<PathData> candidate_path_data;
  if (!OptimizePath(path_boundaries, &candidate_path_data)) {
    return Status(common::PLANNING_ERROR, "Failed to optimize path");
  }

  PathData final_path;
  if (!AssessPath(&candidate_path_data, &final_path)) {
    return Status(common::PLANNING_ERROR, "Failed to assess path");
  }

  // 3. 设置最终路径
  reference_line_info->SetPathData(final_path);
  return Status::OK();
}
```

### 3.4 在 pipeline.pb.txt 中引用

```protobuf
stage: {
  name: "LANE_FOLLOW_STAGE"
  type: "LaneFollowStage"
  task: {
    name: "LANE_FOLLOW_PATH"          # Task 别名
    type: "LaneFollowPath"            # Task C++ 类名
  }
  task: {
    name: "PATH_DECIDER"
    type: "PathDecider"
  }
  # ... Task 按声明顺序执行
}
```

- `task_type` 格式：Task 类名（如 `LaneFollowPath`），PluginManager 自动补全命名空间
- Task 按声明顺序依次执行，前一个 Task 的输出是后一个 Task 的输入
- 一个 Stage 通常声明 3-11 个 Task

### 3.5 plugins.xml 格式

```xml
<library path="modules/planning/tasks/my_task/libmy_task.so">
    <class type="apollo::planning::MyTask" base_class="apollo::planning::Task"></class>
</library>
```

> ⚠️ **关键**：`base_class` 必须是 `apollo::planning::Task`（不是具体的子类如 `PathGeneration`），否则 PluginManager 无法正确注册。

### 3.6 完整新增流程 Checklist

1. [ ] 创建目录 `modules/planning/tasks/<name>/`
2. [ ] 创建子目录 `proto/`、`conf/`
3. [ ] 写 proto 定义（如需要）
4. [ ] 写 Task 类（`.h` + `.cc`），实现 `Init()` 和 `Process()`
5. [ ] 写 `conf/default_conf.pb.txt`（如需要可配置参数）
6. [ ] 写 `BUILD`、`plugins.xml`（`<library>` 格式）、`cyberfile.xml`
7. [ ] 在目标场景的 `conf/pipeline.pb.txt` 中引用（声明 Task 顺序）
8. [ ] 编译 → `aem profile use default` → 测试

---

## 第 4 章：ScenarioManager 场景切换机制

### 4.1 完整调用链

```
PublicRoadPlanner::Plan()
  │
  ├── [第 1 次] scenario_manager_.Update(planning_start_point, frame)
  │       └── 检查是否可以从当前场景切换到新场景
  │
  ├── scenario_ = scenario_manager_.mutable_scenario()
  ├── result = scenario_->Process(planning_start_point, frame)
  │       └── Scenario::Process()
  │             ├── current_stage_ == nullptr? → CreateStage(首个)
  │             ├── current_stage_->Process()
  │             │       └── ExecuteTaskOnReferenceLine()
  │             │             ├── for each ReferenceLineInfo:
  │             │             │   └── for each Task: Task::Execute()
  │             │             └── CombinePathAndSpeedProfile()
  │             └── 根据 StageResult 推进 Stage 或返回 STATUS_DONE
  │
  └── [if STATUS_DONE] scenario_manager_.Update(...)  ← 第 2 次调用
```

**关键设计**：`Update()` 被调用两次：
- **第 1 次**（每帧）：检查是否可以从当前场景切换到新场景
- **第 2 次**（仅在当前场景返回 `STATUS_DONE` 时）：场景完成，立即寻找下一个匹配场景

### 4.2 Update() 内部详细流程

```cpp
void ScenarioManager::Update(const TrajectoryPoint& ego_point, Frame* frame) {
  for (auto scenario : scenario_list_) {

    // ═══ 步骤 1：STATUS_PROCESSING 保护 ═══
    // 如果遍历到当前正在运行的场景且它还在处理中 → 不允许任何场景切换
    if (current_scenario_.get() == scenario.get() &&
        current_scenario_->GetStatus() == ScenarioStatusType::STATUS_PROCESSING) {
      return;  // ★ 直接返回，不做任何切换 ★
    }

    // ═══ 步骤 2：检查每个场景的 IsTransferable ═══
    if (scenario->IsTransferable(current_scenario_.get(), *frame)) {

      // ═══ 步骤 3：场景交接三部曲 ═══
      current_scenario_->Exit(frame);              // ① 旧场景退出
      current_scenario_.reset();                    // ② 释放旧场景
      current_scenario_ = scenario;                 // ③ 切换到新场景
      current_scenario_->Enter(frame);              // ④ 新场景初始化
      break;                                        // 找到第一个匹配的就停止
    }
  }

  // ═══ 步骤 4：fallback ═══
  // 如果没有任何场景匹配，使用 default_scenario_type_（通常为 LANE_FOLLOW）
  if (current_scenario_ == nullptr) {
    current_scenario_ = default_scenario_type_;
  }
}
```

### 4.3 IsTransferable 详解

#### 函数签名

```cpp
virtual bool IsTransferable(const Scenario* other_scenario,
                            const Frame& frame);
```

| 参数 | 含义 |
|------|------|
| `other_scenario` | 当前正在运行的场景（可能为 nullptr） |
| `frame` | 当前帧的完整数据 |
| **返回 true** | 本场景可以从 `other_scenario` 切入 |
| **返回 false** | 不切入，ScenarioManager 继续遍历下一个 |

#### ScenarioManager 对返回值的理解

- 遍历 `scenario_list_`，**第一个** `IsTransferable` 返回 true 的场景被切入
- 因此 `scenario_list_` 中**排在前面的场景优先级更高**
- `LANE_FOLLOW` 排在最后作为兜底（它的 `IsTransferable` 永远返回 true）

### 4.4 IsTransferable 编写实战指南

#### 可用数据源

| 数据 | 获取方式 | 典型用途 |
|------|----------|----------|
| 红绿灯 overlap | `reference_line_info.reference_line().map_path().signal_overlaps()` | 判断是否需要 traffic_light 场景 |
| 停止牌 overlap | `reference_line_info.reference_line().map_path().stop_sign_overlaps()` | 判断是否需要 stop_sign 场景 |
| 让行标志 overlap | `reference_line_info.reference_line().map_path().yield_sign_overlaps()` | 判断是否需要 yield_sign 场景 |
| 人行横道 overlap | `reference_line_info.reference_line().map_path().crosswalk_overlaps()` | 判断是否需要 crosswalk 处理 |
| 障碍物列表 | `frame.obstacles()` | 检查特殊障碍物类型 |
| ADC 速度 | `frame.EgoSpeed()` | 判断车辆状态 |
| 规划状态机 | `frame.planning_context()->mutable_planning_status()` | 检查 pull_over 等指令状态 |
| 路由终点距离 | `frame.routing()` | 判断是否接近目的地 |

#### 模式一：overlap 检测型（stop_sign_unprotected）

```cpp
// 参考线上有 stop_sign overlap 时切入
const auto& reference_line_info = frame.reference_line_info().front();
const auto& stop_sign_overlaps =
    reference_line_info.reference_line().map_path().stop_sign_overlaps();
if (!stop_sign_overlaps.empty()) {
  return true;
}
```

#### 模式二：信号灯状态型（traffic_light_protected）

```cpp
// ⚠️ 注意：信号灯为非 GREEN 且非 BLACK 时切入（处理红灯/黄灯场景）
// 绿灯时不切入，车辆走 lane_follow 直行通过
const auto& traffic_light_overlaps =
    reference_line_info.reference_line().map_path().signal_overlaps();
for (const auto& overlap : traffic_light_overlaps) {
  auto signal_color = GetSignalColor(overlap->object_id);
  if (signal_color != GREEN && signal_color != BLACK) {
    return true;  // 红灯或黄灯，需要 traffic_light 场景保护
  }
}
```

#### 模式三：状态机驱动型（pull_over）

```cpp
// 收到 pull_over 指令时切入
auto* planning_status = frame.planning_context()->mutable_planning_status();
if (planning_status->has_pull_over() &&
    planning_status->pull_over().is_pull_over()) {
  return true;
}
```

#### 模式四：永远可切入（lane_follow 兜底）

```cpp
bool LaneFollowScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  return true;  // 永远返回 true，作为兜底场景
}
```

### 4.5 Scenario 生命周期与交接

```
场景 A (当前)                     场景 B (新)
     │                                │
     │  IsTransferable() = true       │
     │◄───────────────────────────────│
     │                                │
     │  Exit(frame)                   │
     │──────► 清理状态/释放资源        │
     │                                │
     │  ~Scenario()                   │
     │──────► 析构                     │
     │                                │
     │                     Enter(frame)
     │                     ──────► 初始化
     │                                │
     │                     Process(frame)
     │                     ──────► 开始规划
```

### 4.6 scenario_list_ 初始化

```cpp
// ScenarioManager::Init()
for (const auto& scenario_config : planner_config.scenario()) {
  auto scenario = PluginManager::CreateInstance<Scenario>(
      ConfigUtil::GetFullPlanningClassName(scenario_config.type()));
  scenario->Init(injector_, scenario_config.name());
  scenario_list_.push_back(scenario);
}
current_scenario_ = default_scenario_type_;  // 启动后首个活跃场景 = LANE_FOLLOW
init_ = true;
```

- 场景列表从 `public_road_planner_config.pb.txt` 的 `scenario` 字段加载
- 列表顺序 = 优先级顺序（先匹配到的先切入）
- 启动后默认场景为第一个场景（通常是 `LANE_FOLLOW`）

---

## 第 5 章：配置文件完整链路

### 5.1 Profile 目录结构映射

| 配置文件 | 对应模块 | 对应类/用途 |
|----------|----------|------------|
| `planning.conf` | planning_component | gflags 参数覆盖（`--default_cruise_speed` 等） |
| `planning_config.pb.txt` | planning_component | topic_config + reference_line_config |
| `public_road_planner_config.pb.txt` | public_road_planner | **Scenario 列表注册**（核心） |
| `traffic_rule_config.pb.txt` | traffic_decider | **TrafficRule 列表注册**（核心） |
| `discrete_points_smoother_config.pb.txt` | planning_component | 参考线平滑器参数 |
| `planning_semantic_map_config.pb.txt` | planning_base | 学习模式配置 |
| `navi_planner_config.pb.txt` | navi_planning | 导航模式配置 |
| `planner_open_space_config.pb.txt` | planning_component | 开放空间规划器参数 |
| `planning_navi.conf` | navi_planning | 导航模式 gflags |

> ⚠️ `scenario_config.pb.txt` 在 EDU 版本中不独立存在 — Scenario 列表已合并到 `public_road_planner_config.pb.txt`。

### 5.2 配置加载时序图

```
PlanningComponent::Init()
  │
  ├─ 1. GetProtoConfig(&config_)              ← planning_config.pb.txt
  │
  ├─ 2. OnLanePlanning::Init(config_)
  │     ├─ TrafficDecider::Init()
  │     │   ├─ LoadConfig → traffic_rule_config.pb.txt → TrafficRulesPipeline
  │     │   └─ for each rule: PluginManager::CreateInstance<TrafficRule>(type)
  │     │       └─ rule->Init() → __cxa_demangle → GetPluginConfPath → default_conf.pb.txt
  │     │
  │     └─ PublicRoadPlanner::Init()
  │         ├─ LoadConfig<PlannerPublicRoadConfig>(config_path, &config_)
  │         │   └─ public_road_planner_config.pb.txt
  │         │
  │         └─ ScenarioManager::Init(config_)
  │             └─ for each scenario in config_.scenario():
  │                 ├─ PluginManager::CreateInstance<Scenario>(type)
  │                 └─ scenario->Init()
  │                       ├─ __cxa_demangle → GetPluginClassHomePath + "/conf"
  │                       ├─ GetPluginConfPath → pipeline.pb.txt → ScenarioPipeline
  │                       └─ GetPluginConfPath → scenario_conf.pb.txt
  │
  └─ 3. 初始化 CyberRT Reader/Writer
```

**Stage 和 Task 的配置加载（Lazy 模式）**：

```
Scenario::Process()
  └─ current_stage_ == nullptr 时：
      └─ Stage::Init()
          ├─ 遍历 pipeline_config_.task()
          │   └─ for each task:
          │       ├─ PluginManager::CreateInstance<Task>(task_type)
          │       └─ task->Init(config_dir, task_name, injector)
          │             ├─ config_path_ = config_dir + "/" + name + ".pb.txt"
          │             ├─ __cxa_demangle → GetPluginConfPath → default_conf.pb.txt
          │             └─ Task::LoadConfig<TaskConfig>(&config_)
          │                   └─ LoadMergedConfig(default_conf, config_path_)
          │                         1. LoadConfig(default_conf)     ← 默认配置
          │                         2. LoadConfig(config_path_)     ← 用户覆盖配置
          │                         3. config->MergeFrom(specific)  ← 合并（用户覆盖优先）
          │
          └─ 创建 fallback_task（默认 FastStopTrajectoryFallback）
```

### 5.3 LoadConfig<T>() 的自动发现机制

核心原理：通过 C++ ABI 的 `__cxa_demangle` 获取运行时类名，再由 PluginManager 查找该类注册时的源码路径，拼接配置文件路径。

```cpp
// Planner::LoadConfig<T>() — 典型实现
template <typename T>
bool Planner::LoadConfig(const std::string& custom_config_path, T* config) {
  std::string config_path = custom_config_path;
  if (config_path.empty()) {
    int status;
    std::string class_name =
        abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    // 类名 → PluginManager 查找 → 插件源码目录
    config_path = PluginManager::Instance()
                      ->GetPluginConfPath<Planner>(class_name,
                                                   "conf/planner_config.pb.txt");
  }
  return apollo::cyber::common::LoadConfig<T>(config_path, config);
}
```

各插件的配置查找路径：

| 插件类型 | 类名示例 | 查找的配置路径 |
|----------|----------|---------------|
| Planner | `apollo::planning::PublicRoadPlanner` | `conf/planner_config.pb.txt` |
| Scenario | `apollo::planning::LaneFollowScenario` | `conf/pipeline.pb.txt` + `conf/scenario_conf.pb.txt` |
| Task | `apollo::planning::LaneFollowPath` | `conf/default_conf.pb.txt` |
| TrafficRule | `apollo::planning::Crosswalk` | `conf/default_conf.pb.txt` |

**配置文件未找到时的行为**：
- `LoadConfig<T>()` 返回 false → `AWARN` 日志警告
- Task 的 `LoadMergedConfig()` 用户配置不存在时仅用默认配置，不报错
- 关键配置（如 pipeline.pb.txt）缺失会导致插件 Init 失败

### 5.4 三种配置类型

| 类型 | 格式 | 示例 | 用途 | 修改后需编译？ |
|------|------|------|------|:---:|
| pb.txt | Protobuf Text | `pipeline.pb.txt`, `default_conf.pb.txt` | 插件参数配置 | 否 |
| .conf | GFlags 键值对 | `planning.conf` | 全局 gflags 覆盖 | 否 |
| .dag / .launch | CyberRT 格式 | `planning.dag` | 调度配置 | 否 |

### 5.5 新增插件时的配置 Checklist

| 插件类型 | 必须创建/修改的配置 | 可选 |
|----------|---------------------|------|
| **Scenario** | `conf/pipeline.pb.txt`（新建）+ `public_road_planner_config.pb.txt`（追加 scenario 声明） | `conf/scenario_conf.pb.txt`（场景参数） |
| **Task** | `conf/default_conf.pb.txt`（新建）+ 目标场景的 `pipeline.pb.txt`（追加 task 声明） | — |
| **TrafficRule** | `traffic_rule_config.pb.txt`（追加 rule 声明） | `conf/default_conf.pb.txt`（规则参数） |

---

## 第 6 章：Stage 机制与 pipeline 配置

### 6.1 Stage 生命周期状态机

```
READY → RUNNING → FINISHED / ERROR
```

| 状态 | 枚举值 | 含义 |
|------|--------|------|
| `READY` | 2 | Stage 刚创建，尚未执行 |
| `RUNNING` | 3 | Stage 正在执行中，下周期继续当前 Stage |
| `FINISHED` | 4 | Stage 任务完成，Scenario 切换到下一 Stage 或结束 |
| `ERROR` | 1 | 发生错误，Scenario 进入 `STATUS_UNKNOWN` |

**上层 Scenario 状态机**：

```
STATUS_UNKNOWN → STATUS_PROCESSING → STATUS_DONE
```

转换规则：
- `Stage RUNNING` → Scenario `STATUS_PROCESSING`
- `Stage FINISHED` + `next_stage_` 非空 → 切换 Stage → Scenario `STATUS_PROCESSING`
- `Stage FINISHED` + `next_stage_` 为空 → Scenario `STATUS_DONE`
- `Stage ERROR` → Scenario `STATUS_UNKNOWN`

### 6.2 pipeline.pb.txt 格式详解

Proto 定义（`scenario_pipeline.proto`）：

```protobuf
message PluginDeclareInfo {
    required string name = 1;  // 插件别名（如 "LANE_FOLLOW_PATH"）
    required string type = 2;  // 插件 C++ 类名（如 "LaneFollowPath"）
}

message StagePipeline {
    required string name = 1;                    // Stage 别名
    required string type = 2;                    // Stage C++ 类名
    optional bool enabled = 3;                   // 是否启用（默认 true）
    repeated PluginDeclareInfo task = 4;         // Task 列表（按序执行）
    optional PluginDeclareInfo fallback_task = 5; // 兜底 Task
}

message ScenarioPipeline {
    repeated StagePipeline stage = 1;  // Stage 列表（按序串联）
}
```

### 6.3 Scenario::Process() 内部调度逻辑

```cpp
ScenarioResult Scenario::Process(const TrajectoryPoint& planning_init_point,
                                  Frame* frame) {
  // 1. 首次进入：创建首个 Stage
  if (current_stage_ == nullptr) {
    current_stage_ = CreateStage(*stage_pipeline_map_[
        scenario_pipeline_config_.stage(0).name()]);
  }

  // 2. 调用 Stage::Process()（纯虚函数，各 Stage 子类实现）
  auto ret = current_stage_->Process(planning_init_point, frame);

  // 3. 根据 StageResult 决定状态转换
  switch (ret.GetStageStatus()) {
    case StageStatusType::RUNNING:
      return STATUS_PROCESSING;           // 保持当前 Stage

    case StageStatusType::FINISHED: {
      auto next_stage = current_stage_->NextStage();
      if (next_stage.empty()) {
        return STATUS_DONE;               // 所有 Stage 完成
      }
      current_stage_ = CreateStage(*stage_pipeline_map_[next_stage]);
      return STATUS_PROCESSING;           // 切换到下一 Stage
    }

    case StageStatusType::ERROR:
      return STATUS_UNKNOWN;              // 错误
  }
}
```

### 6.4 ExecuteTaskOnReferenceLine() 详解

```cpp
StageResult Stage::ExecuteTaskOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame) {

  // 1. 前置检查
  if (frame->reference_line_info().empty()) {
    return StageResult(StageStatusType::ERROR);
  }

  // 2. 遍历所有 ReferenceLineInfo
  for (auto& reference_line_info : frame->reference_line_info()) {
    if (!reference_line_info.IsDrivable()) continue;  // 跳过不可行驶

    // 3. 按序执行 Task 链
    for (auto& task : task_list_) {
      auto ret = task->Execute(frame, &reference_line_info);
      if (!ret.ok()) {
        // Task 失败 → 使用 fallback_task
        fallback_task_->Execute(frame, &reference_line_info);
        break;  // 中断当前 Task 链
      }
    }
  }

  // 4. 合并路径和速度曲线
  CombinePathAndSpeedProfile(planning_start_point, frame);

  return StageResult(StageStatusType::RUNNING);
}
```

### 6.5 多 Stage 流转示例：stop_sign_unprotected

```
StagePreStop ──FinishStage()──▶ StageStop ──FinishStage()──▶
StageCreep ──FinishStage()──▶ StageIntersectionCruise ──FinishScenario()──▶ 场景结束
```

每个 Stage 的核心逻辑：

| Stage | 做什么 | 完成条件 |
|-------|--------|----------|
| **StagePreStop** | 在停止线前预减速 | ADC 速度足够低且距离停止线足够近 |
| **StageStop** | 完全停稳 | ADC 静止 |
| **StageCreep** | 蠕行靠近路口观察 | 到达 creep 终点位置 |
| **StageIntersectionCruise** | 通过路口 | 通过路口后 |

### 6.6 BaseStageCreep 模板方法

`BaseStageCreep` 封装了蠕行靠近的标准流程：

```cpp
// 子类必须实现的纯虚函数
virtual const CreepStageConfig& GetCreepStageConfig() const = 0;
virtual void GetOverlapStopInfo(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    hdmap::PathOverlap* overlap, double* stop_s) const = 0;

// 基类封装的标准流程
StageResult BaseStageCreep::ProcessCreep(Frame* frame,
                                          ReferenceLineInfo* reference_line_info) {
  // 1. 获取 overlap 停车信息（子类提供具体逻辑）
  hdmap::PathOverlap overlap;
  double stop_s;
  GetOverlapStopInfo(frame, reference_line_info, &overlap, &stop_s);

  // 2. 构建虚拟障碍物模拟停车线
  BuildStopDecision(CREEP_VO_ID_PREFIX_ + overlap->object_id,
                    overlap->start_s, config_.stop_distance(), ...);

  // 3. 检查蠕行是否完成（车辆到达 creep 终点位置）
  if (CheckCreepDone(frame, reference_line_info)) {
    return FinishStage();
  }
  return StageResult(StageStatusType::RUNNING);
}
```

**StageCreep 与普通 Stage 的关键区别**：普通 Stage 直接调用 `ExecuteTaskOnReferenceLine()`；而 `BaseStageCreep` 子类先调用 `ProcessCreep()` 构建虚拟障碍物，再调用 `ExecuteTaskOnReferenceLine()`。

### 6.7 BaseStageCruise 模板方法

`BaseStageCruise` 封装了通过路口的巡航流程：

```cpp
// 子类必须实现的纯虚函数
virtual const hdmap::PathOverlap* GetTrafficSignOverlap(
    const ReferenceLineInfo& reference_line_info) const = 0;

// 基类提供的 CheckDone 方法
bool BaseStageCruise::CheckDone(const Frame& frame,
                                 const ReferenceLineInfo& reference_line_info) {
  // 模式一：无 junction_overlap → 距离检查（驶出 20m 即认为完成）
  // 模式二：有 junction_overlap → CheckInsideJunction（检查是否离开路口）
}
```

### 6.8 编写自定义 Stage 的 Checklist

1. [ ] 确定继承哪个基类：`Stage` / `BaseStageCreep` / `BaseStageCruise`
2. [ ] 写头文件（`.h`），声明 `Process()` 或 `ProcessCreep()` / 相关纯虚函数
3. [ ] 实现 `Process()`：调用 `ExecuteTaskOnReferenceLine()` → 判断完成条件 → `FinishStage()` 或 `FinishScenario()`
4. [ ] 对于 `BaseStageCreep` 子类：实现 `GetCreepStageConfig()` 和 `GetOverlapStopInfo()`
5. [ ] 对于 `BaseStageCruise` 子类：实现 `GetTrafficSignOverlap()`
6. [ ] 在场景的 `conf/pipeline.pb.txt` 中声明 Stage
7. [ ] 如果不是最后一个 Stage → 调用 `FinishStage()`；如果是最后一个 Stage → 调用 `FinishScenario()`

---

## 第 7 章：现有插件可复用度评估

### 评估标准

| 标记 | 含义 |
|:---:|------|
| 🟢 | 直接可用，最多调参 |
| 🟡 | 需要修改部分逻辑 |
| 🔴 | 需要新建 |
| ⚪ | 不相关 |

---

### 赛题一：交通灯场景

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `traffic_light_protected` | Scenario | 🟢 | 处理红灯/黄灯停车保护，绿灯时不切入走 lane_follow 直行；直接可用 |
| `traffic_light_unprotected_right_turn` | Scenario | 🟢 | 配置 `enable_right_turn_on_red: true`（现有配置项）即可启用红灯右转 |
| `traffic_light` | TrafficRule | 🟢 | 仅需调参 `stop_distance: 1.0 → 1.75`（见第 8 章参数速查表） |

**分析**：Apollo 已有完整的红绿灯处理体系。**所有改动均为纯配置，零代码修改。**

---

### 赛题二：变道

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `lane_follow` | Scenario | 🟢 | 默认场景，变道由 routing 触发 |
| `lane_change_path` | Task | 🟡 | 需新增变道期间限速；安全距离公式硬编码需 proto 化 |
| `backside_vehicle` | TrafficRule | 🟢 | 后方来车检测逻辑完整 |
| `speed_setting` | TrafficRule | 🟡 | 不感知变道状态，需在 Task 中 `SetCruiseSpeed` 覆盖 |

**分析**：变道核心链路完整。主要缺口：变道限速缺失 + 安全距离硬编码。`change_lane_success_freeze_time` 已有配置项（从 3s 调至 5s 即可防连续变道）。

---

### 赛题三：S弯

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `lane_follow` | Scenario | 🟢 | QP 路径优化器可处理连续弯道 |
| `large_curvature` | Scenario | ⚪ | 不可用：`IsTransferable` 硬编码剩余距离 <10m 才触发，S 弯中段不会切入 |
| `speed_bounds_decider` | Task | 🟢 | 曲率限速公式完整，调参 `max_centric_acceleration_limit: 0.8 → 2.0` |
| `obstacle_nudge_decider` | Task | 🟡 | 调参增大横向安全距离 buffer |
| `speed_setting` | TrafficRule | 🟢 | `planning_upper_speed_limit: 20.0 → 8.33` 实现全局限速 30km/h |

**分析**：**纯配置方案**。QP 路径优化器本身就能处理连续弯道。关键：放开曲率限速 + 增大横向 nudge buffer + 设全局限速 30km/h。

---

### 赛题四：U-Turn

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `large_curvature` | Scenario | 🟡 | 核心扩展点：`IsTransferable` 需增加检测 >150° 航向变化；`Process` 需补全 `NARROW_STREET_U_TURN` case 分支 |
| `open_space_roi_decider_park` | Task | 🟡 | Hybrid A* 支持 180° 翻转；需新增 `NARROW_STREET_U_TURN` case |
| `yield_sign` | TrafficRule | 🟡 | 框架可适配对向来车避让 |
| `speed_setting` | TrafficRule | 🟢 | 全局限速 30km/h |

**分析**：`NARROW_STREET_U_TURN` 枚举已在 proto 中定义，但代码无对应分支。Hybrid A* 能力足够（支持 180° 翻转），需补全调用路径。

---

### 赛题五：施工区域

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `lane_follow` | Scenario | 🟢 | 基础场景直接可用 |
| `lane_borrow_path` | Task | 🟡 | `.oldcode/` 中有完整锥桶处理逻辑（`IsConstructionSmallObstacle` 等），需移植 |
| `speed_setting` | TrafficRule | 🟡 | 不支持区域限速，需用 `ReferenceLine::AddSpeedLimit()` 动态注入施工区限速 |

**分析**：旧版代码中有完整的施工区域锥桶识别 + 借道逻辑可直接移植。区域限速需新增能力。

---

### 赛题六：站点接驳

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `pull_over` | Scenario | 🟡 | APPROACH Stage 可精准泊入；但 `PARK_COMPLETE` 后直接 `FinishScenario()`，需改为跳转 WAITING Stage |
| `park_and_go` | Scenario | 🔴 | 完全不可用：`IsTransferable` 要求 `!is_lane_type_city_driving`，站点道路不符合条件 |
| `pull_over_path` | Task | 🟢 | 精准泊车路径，直接可用 |

**分析**：需扩展 `pull_over` 场景，新增 WAITING Stage（计时 5s）+ DEPARTURE Stage（LaneFollow 驶离）。`park_and_go` 不可用。

---

### 赛题七：环岛

| 现有插件 | 类型 | 可复用度 | 需要改什么 |
|----------|------|:---:|------|
| `stop_sign_unprotected` | Scenario | 🟡 | 4 Stage 框架可参考，但环岛需要 GeoFence 检测入口 |
| `bare_intersection_unprotected` | Scenario | 🟡 | `CheckClear()` S-T 图安全间隙判断可复用 |
| `yield_sign` | TrafficRule | 🟡 | 入口让行框架可适配 |
| `backside_vehicle` | TrafficRule | 🟡 | 需扩展 `roundabout_exit` 模式 |
| 环岛检测 | — | 🔴 | **HDMap 无环岛类型标记**，需新建 GeoFence 外部配置文件 |
| 转向灯 | — | 🟢 | `ReferenceLineInfo::SetTurnSignal()` 现成接口 |

**分析**：最复杂的赛题。HDMap 无法标记环岛，必须新建 GeoFence。需新建 `Roundabout` Scenario（3 Stage：ENTRY → CIRCULATING → EXIT）。约 60% 逻辑可复用现有插件。

---

### 跨赛题共性需求汇总

| 共性需求 | 涉及赛题 | 现有支持 | 建议方案 |
|----------|:---:|------|------|
| 限速 30km/h | 2/3/4/5/6/7 | `speed_setting` + `planning_upper_speed_limit` | 全局设 `planning_upper_speed_limit=8.33` |
| 障碍物绕行 ≥1m | 3/4/7 | `obstacle_nudge_decider` | 调参 `static_obstacle_nudge_l_buffer ≥0.6` |
| 让行/停车观察 | 4/7 | `stop_sign`/`yield_sign` | 复用 `BuildStopDecision` 框架 |
| 精准停车位置 | 1/6 | `traffic_light`/`pull_over` | 复用各自的停车机制 |
| 后方来车检测 | 2/7 | `backside_vehicle` | 赛题二直接可用，赛题七需扩展 |
| 区域限速 | 5 | — | **缺失**：用 `ReferenceLine::AddSpeedLimit()` |
| 静止等待计时 | 6 | — | **缺失**：参考 `EmergencyPullOverStageStandby` |
| GeoFence 区域检测 | 5/7 | — | **缺失**：HDMap 无施工区/环岛标记 |

### 必须新建的插件估算

| 赛题 | 新建 Scenario | 新建 Task | 新建 TrafficRule |
|------|:---:|:---:|:---:|
| 赛题一 | 0 | 0 | 0 |
| 赛题二 | 0 | 0 | 0 |
| 赛题三 | 0 | 0 | 0 |
| 赛题四 | 0 (扩展 large_curvature) | 0 | 0-1 |
| 赛题五 | 0 | 0 (移植旧代码) | 0-1 |
| 赛题六 | 0 (扩展 pull_over) | 0 | 0 |
| 赛题七 | **1** (roundabout) | 0 | 0-1 |

> **关键结论**：7 道赛题中，赛题一/三为**纯配置方案（零代码）**；赛题二/四/五/六为**修改现有插件**；仅有赛题七需要**新建 1 个 Scenario**。

---

## 第 8 章：关键参数调优指南

### 8.1 限速参数

#### 全局巡航速度

| 参数 | 文件 | 当前值 | 目标值（30km/h） |
|------|------|--------|------|
| `--default_cruise_speed` | `profiles/default/.../planning.conf:4` | 11.18 m/s (40km/h) | **8.33** m/s (30km/h) |
| `--planning_upper_speed_limit` | `profiles/default/.../planning.conf:3` | 20.00 m/s | **8.33** m/s |

> gflags 默认值（代码级）：`planning_gflags.cc` — `default_cruise_speed=5.0`, `planning_upper_speed_limit=31.3`。profile 中的值会覆盖 gflags 默认值。

#### 场景/区域级速度限制

`speed_setting` TrafficRule 处理全局巡航速度设置。对于**区域限速**（如施工区），`speed_setting` 不原生支持，需使用 `ReferenceLine::AddSpeedLimit(s_start, s_end, speed_limit)` 动态注入。

### 8.2 停车距离参数

| 参数 | 文件 | 当前值 | 目标值 |
|------|------|--------|--------|
| TrafficLight `stop_distance` | `modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt` | 1.0 m | **1.5~2.0** m |
| StopSign `stop_distance` | `modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt` | 1.0 m | **1.5~2.0** m |
| EmergencyStop `stop_distance` | `modules/planning/scenarios/emergency_stop/conf/scenario_conf.pb.txt` | 1.0 m | 按需 |
| EmergencyPullOver `stop_distance` | `modules/planning/scenarios/emergency_pull_over/conf/scenario_conf.pb.txt` | 1.5 m | 按需 |

### 8.3 横向距离参数（绕行/变道）

| 参数 | 文件 | 当前值 | 说明 |
|------|------|--------|------|
| `max_lateral_left_nudge_dis` | `obstacle_nudge_decider/conf/default_conf.pb.txt` | 3.0 m | 左侧最大绕行距离 |
| `max_lateral_right_nudge_dis` | 同上 | 3.0 m | 右侧最大绕行距离 |
| `full_nudge_buffer` | 同上 | 1.5 m | 完全绕行 buffer |
| `static_obstacle_nudge_l_buffer` | `planning_gflags.cc` | 0.3 m | 静态障碍物绕行最小 l 距离 |
| `nonstatic_obstacle_nudge_l_buffer` | `planning_gflags.cc` | 0.4 m | 非静态障碍物绕行最小 l 距离 |
| `nudge_buffer` | `planning_gflags.cc` | 0.3 m | 横向优化绕行 buffer |
| `obstacle_lat_buffer` | `planning_gflags.cc` | 0.4 m | 障碍物横向 buffer |
| `lateral_buffer` | `discrete_points_smoother_config.pb.txt` | 0.2 m | 路径平滑横向 buffer |

> **绕行 ≥1m**：增大 `static_obstacle_nudge_l_buffer` 到 0.6+ 和 `obstacle_lat_buffer` 到 0.6+ 即可满足。注意过大值可能导致路径规划失败。

### 8.4 参数修改后如何生效

| 参数类型 | 文件 | 需编译？ | 需 `aem profile use`？ |
|----------|------|:---:|:---:|
| pb.txt 配置（插件 conf） | `modules/planning/**/conf/default_conf.pb.txt` | **否** | **是** |
| pb.txt 配置（profile 覆盖） | `profiles/default/.../*.pb.txt` | 否 | 是 |
| .conf 配置（gflag 覆盖） | `planning.conf` | 否 | 是 |
| C++ 硬编码常量 | `*.h` / `*.cc` | **是** | 是 |
| Proto 定义（默认值变更） | `*.proto` | 是 | 是 |
| gflags 默认值变更 | `planning_gflags.cc` | 是 | 是 |

### 8.5 赛题参数速查表

| 赛题需求 | 参数名 | 所在文件 | 当前值 → 目标值 |
|----------|--------|----------|------|
| 限速 30 km/h | `--planning_upper_speed_limit` | `planning.conf` | 20.00 → **8.33** |
| 红灯停车 1.5-2.0m | `stop_distance` | `traffic_light/conf/default_conf.pb.txt` | 1.0 → **1.5~2.0** |
| 停止牌停车 1.5-2.0m | `stop_distance` | `stop_sign/conf/default_conf.pb.txt` | 1.0 → **1.5~2.0** |
| 绕行横向 ≥1m | `static_obstacle_nudge_l_buffer` | `planning_gflags.cc` | 0.3 → **0.6** |
| 绕行横向 ≥1m | `obstacle_lat_buffer` | `planning_gflags.cc` | 0.4 → **0.6** |
| S 弯不减速 | `max_centric_acceleration_limit` | `speed_bounds_decider/conf/` | 0.8 → **2.0** |
| 变道冷却时间 | `change_lane_success_freeze_time` | `lane_change_path/conf/default_conf.pb.txt` | 3.0 → **5.0** |

---

## 附录：7 道赛题模块修改清单速查表

| 赛题 | Scenario | Task | TrafficRule | 参数修改 | 代码修改量 |
|------|----------|------|-------------|----------|:---:|
| 赛题一 交通灯 | 无需改动 | 无需改动 | 无需改动 | `stop_distance` 1.0→1.75, `enable_right_turn_on_red: true` | **零代码** |
| 赛题二 变道 | 无需改动 | 改 `lane_change_path`（加限速 + proto 化安全距离） | 改 `speed_setting` 或配合 Task 限速 | `planning_upper_speed_limit→8.33`, `change_lane_success_freeze_time→5.0` | 小 |
| 赛题三 S弯 | 无需改动 | 无需改动 | 无需改动 | `max_centric_acceleration_limit→2.0`, nudge buffer 增大, `planning_upper_speed_limit→8.33` | **零代码** |
| 赛题四 U-Turn | 扩展 `large_curvature`（补全 U-Turn case） | 补全 `open_space_roi_decider_park` case | 扩展 `yield_sign`（对向来车） | `planning_upper_speed_limit→8.33` | 中 |
| 赛题五 施工区域 | 无需改动 | 移植 `.oldcode/` 施工区逻辑到 `lane_borrow_path` | 扩展 `speed_setting` 或新建区域限速 | `planning_upper_speed_limit→8.33` | 中 |
| 赛题六 站点接驳 | 扩展 `pull_over`（新增 WAITING + DEPARTURE Stage） | 无需改动 | 无需改动 | `planning_upper_speed_limit→8.33` | 中 |
| 赛题七 环岛 | **新建 `roundabout`**（3 Stage） | 无需改动（复用现有 Task 链） | 扩展 `backside_vehicle`（roundabout_exit 模式） | `planning_upper_speed_limit→8.33`, GeoFence 配置文件 | 大 |

---

> **文档版本**：v1.0 | **覆盖范围**：Apollo EDU PnC 插件开发全流程
> **配套资源**：`output/ARCHITECTURE.md` | `output/analysis_reports/` | `.claude/skills/apollo-simulation-guide/`
