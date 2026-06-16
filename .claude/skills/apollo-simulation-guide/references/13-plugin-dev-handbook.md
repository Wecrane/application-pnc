# 13 - PnC 插件开发实战参考

> 来源：`output/pnc_dev_handbook.md`（基于 8 份源码深度分析 + 4 轮交叉审查）
> 定位：通用知识（不限于特定赛题），涵盖 Scenario/Task/TrafficRule 开发全链路

---

## 一、插件体系架构

### 1.1 四层插件类型

| 插件类型 | 基类 | 生命周期 | 配置入口 |
|----------|------|----------|----------|
| **Scenario** | `Scenario` | 全局互斥，同时只有一个生效 | `public_road_planner_config.pb.txt` → `pipeline.pb.txt` |
| **Stage** | `Stage` / `BaseStageCreep` / `BaseStageCruise` | 场景内部，按 pipeline 顺序串联 | `pipeline.pb.txt` 中声明，由 Scenario 管理 |
| **Task** | `Task` → `PathGeneration`/`Decider`/`SpeedOptimizer`/`TrajectoryOptimizer` | 在 Stage 内按序执行 | `pipeline.pb.txt` 中声明 Task 链 |
| **TrafficRule** | `TrafficRule` | 全局并行，所有场景共享 | `traffic_rule_config.pb.txt` |

### 1.2 双层状态机

```
Top Layer: Scenario (STATUS_UNKNOWN → STATUS_PROCESSING → STATUS_DONE)
    └── ScenarioManager::Update() 决定场景切换
        └── Bottom Layer: Stage (READY → RUNNING → FINISHED / ERROR)
            └── Scenario::Process() 管理 Stage 流转
                └── Stage::ExecuteTaskOnReferenceLine() 执行 Task 链
```

### 1.3 调用链路

```
PlanningComponent::Init()
  └── OnLanePlanning::Init()
        ├── TrafficDecider::Init()        → 加载 traffic_rule_config.pb.txt → 创建 TrafficRule 实例
        └── PublicRoadPlanner::Init()
              └── ScenarioManager::Init() → 加载 public_road_planner_config.pb.txt → 创建 Scenario 实例

每帧 Plan():
  PublicRoadPlanner::Plan()
    ├── [第1次] ScenarioManager::Update()  → 检查场景切换
    ├── Scenario::Process()               → 执行当前场景
    │     └── Stage::Process()
    │           └── ExecuteTaskOnReferenceLine()
    │                 └── for each Task: Task::Execute()
    └── [if STATUS_DONE] ScenarioManager::Update() → 第2次，立即找下一个场景
```

---

## 二、ScenarioManager 场景切换机制

### 2.1 Update() 核心逻辑

```
for each scenario in scenario_list_ (按配置顺序):
    ├── 如果遍历到当前场景 且 它还在 PROCESSING → return（不允许切换）
    ├── 调用 scenario->IsTransferable(current_scenario, frame)
    └── 如果返回 true:
          ├── current_scenario->Exit(frame)    // 旧场景退出
          ├── current_scenario = scenario      // 切换
          ├── current_scenario->Enter(frame)   // 新场景初始化
          └── break                            // 找到第一个匹配的就停止

如果没有任何场景匹配 → 使用 default_scenario_type_（通常是 LANE_FOLLOW）
```

**关键规则**：
- `scenario_list_` 的**顺序决定优先级**：排在前面的场景优先匹配
- `STATUS_PROCESSING` 保护：当前场景正在执行时不允许被抢占
- 但列表中排在当前场景**之前**的场景仍可触发 `IsTransferable`，从而**抢占**当前场景
- `LANE_FOLLOW` 永远返回 true，放在列表最后作为兜底

### 2.2 IsTransferable 编写指南

```cpp
virtual bool IsTransferable(const Scenario* other_scenario, const Frame& frame);
```

- `other_scenario`：当前运行的场景（可能为 nullptr）
- 返回 `true`：可以从 other_scenario 切入本场景
- 返回 `false`：不切入，继续遍历

**Frame 中可用的判断数据**：

| 数据 | 获取方式 | 典型用途 |
|------|----------|----------|
| 红绿灯 overlap | `reference_line_info.reference_line().map_path().signal_overlaps()` | 红绿灯场景切入 |
| 停止牌 overlap | `reference_line_info.reference_line().map_path().stop_sign_overlaps()` | 停止牌场景切入 |
| 让行标志 overlap | `reference_line_info.reference_line().map_path().yield_sign_overlaps()` | 让行场景切入 |
| 人行横道 overlap | `reference_line_info.reference_line().map_path().crosswalk_overlaps()` | 人行横道处理 |
| 障碍物列表 | `frame.obstacles()` | 特殊障碍物类型检测 |
| ADC 速度 | `frame.EgoSpeed()` | 停车状态判断 |
| 规划状态机 | `frame.planning_context()->mutable_planning_status()` | pull_over 等指令检测 |
| 路由终点距离 | `frame.routing()` | 接近目的地判断 |

**四种典型 IsTransferable 模式**：

1. **Overlap 检测型**：检查参考线上是否有特定地图元素（stop_sign、traffic_light 等）
2. **信号灯状态型**：检查信号灯颜色（红灯/黄灯切入，绿灯走 lane_follow）
3. **状态机驱动型**：检查 PlanningContext 中的指令状态（如 pull_over 指令）
4. **永远可切入型**：兜底场景（如 LANE_FOLLOW 永远返回 true）

---

## 三、Stage 生命周期与 Pipeline 配置

### 3.1 Stage 状态机

```
READY → RUNNING → FINISHED / ERROR
```

- `READY`：Stage 刚创建
- `RUNNING`：正在执行，下周期继续
- `FINISHED`：完成 → 切换到下一 Stage 或结束 Scenario
- `ERROR`：错误 → Scenario 进入 STATUS_UNKNOWN

### 3.2 Scenario::Process() 调度逻辑

```
1. current_stage_ == nullptr → CreateStage(pipeline 中第一个 Stage)
2. current_stage_->Process() → StageResult
3. 根据 StageResult:
   - RUNNING  → STATUS_PROCESSING（保持当前 Stage）
   - FINISHED → 检查 NextStage():
       - next_stage_ 非空 → 切换 Stage
       - next_stage_ 为空 → STATUS_DONE
   - ERROR    → STATUS_UNKNOWN
```

### 3.3 FinishStage() vs FinishScenario()

| 方法 | 作用 | 何时使用 |
|------|------|----------|
| `FinishStage()` | 标记当前 Stage 完成，切换到 pipeline 中的下一 Stage | 非最后一个 Stage |
| `FinishScenario()` | 设置 `next_stage_ = ""`，标记整个 Scenario 完成 | **最后一个 Stage** |

> ⚠️ **关键**：最后一个 Stage 必须调用 `FinishScenario()` 而非 `FinishStage()`，否则 Scenario 永远不会返回 STATUS_DONE。

### 3.4 ExecuteTaskOnReferenceLine() 流程

```
1. 前置检查：reference_line_info 不能为空
2. 遍历所有 ReferenceLineInfo:
     - 跳过不可行驶的
     - 按序执行 Task 链
     - Task 失败 → 使用 fallback_task_（默认 FastStopTrajectoryFallback）
3. CombinePathAndSpeedProfile() 合并路径和速度曲线
```

### 3.5 Pipeline.pb.txt 格式

```protobuf
stage: {
  name: "STAGE_NAME"       # Stage 别名（唯一标识）
  type: "StageClassName"   # Stage C++ 类名（不含命名空间前缀）
  enabled: true             # 是否启用
  task: {
    name: "TASK_ALIAS"     # Task 别名
    type: "TaskClassName"  # Task C++ 类名
  }
  # ... 更多 Task，按声明顺序执行
  # fallback_task: { ... } # 可选，Task 链失败时的兜底
}
```

**Stage 串联**：pipeline 中声明的 Stage 按顺序构成 Stage 链，通过 `FinishStage()` → `NextStage()` 自动串联。

### 3.6 BaseStageCreep 模板方法

封装了蠕行靠近路口的标准流程：

1. `GetOverlapStopInfo()` — 子类提供：获取 overlap 和停车位置
2. `BuildStopDecision()` — 基类调用：构建虚拟障碍物模拟停止线
3. `CheckCreepDone()` — 基类提供：检查是否到达蠕行终点（min_boundary_t=6.0s, 5帧计数器）
4. `ExecuteTaskOnReferenceLine()` — 基类调用：执行 Task 链

**与普通 Stage 的区别**：`BaseStageCreep` 子类先调用 `ProcessCreep()` 构建虚拟障碍物，再调用 `ExecuteTaskOnReferenceLine()`。

### 3.7 BaseStageCruise 模板方法

封装了通过路口的巡航流程：

1. `GetTrafficSignOverlap()` — 子类提供：获取交通标志 overlap
2. `CheckDone()` — 基类提供：无 junction 时走 20m 距离检查，有 junction 时走 CheckInsideJunction

---

## 四、TrafficRule 开发

### 4.1 三个必须重写的纯虚函数

```cpp
class MyRule : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  void Reset() override;  // 每帧开始时重置规则状态
  common::Status ApplyRule(Frame* frame,
                           ReferenceLineInfo* reference_line_info) override;
};
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyRule, TrafficRule)
```

### 4.2 BuildStopDecision 用法

```cpp
void BuildStopDecision(
    const std::string& virtual_obstacle_id,   // 虚拟障碍物 ID
    const double stop_s,                        // 停车位置 s 坐标
    const double stop_distance,                 // 停车距离（实际停车位置 = stop_s - stop_distance）
    const StopReasonCode& stop_reason_code,     // 停车原因
    const std::string& decision_tag,            // 决策标签（调试用）
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info);
```

**常用 StopReasonCode**：

| 枚举值 | 使用场景 |
|--------|----------|
| `STOP_REASON_STOP_SIGN` | 停止牌 |
| `STOP_REASON_TRAFFIC_LIGHT` | 交通灯 |
| `STOP_REASON_CROSSWALK` | 人行横道 |
| `STOP_REASON_YIELD_SIGN` | 让行标志 |
| `STOP_REASON_DESTINATION` | 终点 |
| `STOP_REASON_EMERGENCY` | 紧急停车 |
| `STOP_REASON_CLEAR_ZONE` | 禁停区 |

> 完整列表见 `modules/planning/proto/decision.proto`

### 4.3 TrafficDecider 执行流程

```
Init():
  for each rule in traffic_rule_config.pb.txt:
    PluginManager::CreateInstance<TrafficRule>(rule.type())
    rule->Init()

每帧 Execute():
  for each rule in rule_list_:
    rule->Reset()
    rule->ApplyRule(frame, reference_line_info)  // 可能构建 StopDecision
  BuildPlanningTarget(reference_line_info)        // 取 s 最小的 StopDecision
```

**规则顺序的影响**：规则按 `traffic_rule_config.pb.txt` 中声明的顺序执行，但最终停车目标由 `BuildPlanningTarget()` 取所有 StopDecision 中 **s 坐标最小的**（最近停车点）。

---

## 五、配置文件完整链路

### 5.1 配置加载调用链

```
PlanningComponent::Init()
  ├── planning_config.pb.txt       → PlanningConfig
  ├── TrafficDecider::Init()
  │     ├── traffic_rule_config.pb.txt → TrafficRulesPipeline
  │     └── for each rule: rule->Init()
  │           └── __cxa_demangle → GetPluginConfPath → conf/default_conf.pb.txt
  └── PublicRoadPlanner::Init()
        ├── public_road_planner_config.pb.txt → PlannerPublicRoadConfig
        └── ScenarioManager::Init()
              └── for each scenario: scenario->Init()
                    ├── __cxa_demangle → GetPluginClassHomePath + "/conf"
                    ├── pipeline.pb.txt → ScenarioPipeline
                    └── scenario_conf.pb.txt（如果存在）

Stage::Init()（Lazy，首次 Process 时调用）:
  └── for each task in pipeline_config_.task():
        ├── PluginManager::CreateInstance<Task>(task_type)
        └── task->Init()
              ├── __cxa_demangle → GetPluginConfPath → conf/default_conf.pb.txt
              └── LoadConfig<TaskConfig>(&config_)
                    └── LoadMergedConfig(default_conf, stage_specific_conf)
                         1. LoadConfig(default_conf)     ← 默认值
                         2. LoadConfig(stage_specific)    ← 用户覆盖
                         3. config->MergeFrom(specific)  ← 合并（用户覆盖优先）
```

### 5.2 LoadConfig<T>() 自动发现机制

核心原理：通过 C++ ABI 的 `abi::__cxa_demangle(typeid(*this).name())` 获取运行时类名，再由 `PluginManager::GetPluginConfPath<BaseClass>(class_name, relative_path)` 查找该类注册时的源码路径，拼接配置文件路径。

**各类插件的配置查找路径**：

| 插件类型 | 查找的配置 |
|----------|-----------|
| Planner | `conf/planner_config.pb.txt` |
| Scenario | `conf/pipeline.pb.txt` + `conf/scenario_conf.pb.txt` |
| Task | `conf/default_conf.pb.txt` |
| TrafficRule | `conf/default_conf.pb.txt` |

### 5.3 三种配置类型

| 类型 | 格式 | 示例 | 修改后需编译？ | 需 aem profile use？ |
|------|------|------|:---:|:---:|
| pb.txt | Protobuf Text | `pipeline.pb.txt`, `default_conf.pb.txt` | 否 | 是 |
| .conf | GFlags 键值对 | `planning.conf` | 否 | 是 |
| C++ 源码 | 硬编码常量/gflags | `planning_gflags.cc` | 是 | 是 |

### 5.4 Profile 目录关键配置文件

| 配置文件 | 作用 |
|----------|------|
| `planning.conf` | gflags 覆盖（`--default_cruise_speed`, `--planning_upper_speed_limit`） |
| `planning_config.pb.txt` | topic_config + reference_line_config |
| `public_road_planner_config.pb.txt` | **Scenario 列表**（决定场景优先级和加载顺序） |
| `traffic_rule_config.pb.txt` | **TrafficRule 列表**（决定规则执行顺序） |

---

## 六、插件注册模板

### 6.1 通用目录结构

```
modules/planning/<scenarios|tasks|traffic_rules>/<plugin_name>/
├── BUILD                  # Bazel 构建（apollo_plugin 宏）
├── plugins.xml            # 插件注册（<library> 格式）
├── cyberfile.xml          # 包元数据
├── <plugin>.h             # 类声明
├── <plugin>.cc            # 类实现
├── conf/
│   ├── pipeline.pb.txt    # [Scenario 专属] Stage 编排
│   ├── scenario_conf.pb.txt  # [Scenario 专属] 场景参数
│   └── default_conf.pb.txt   # [Task/TrafficRule] 默认参数
└── proto/
    ├── <plugin>.proto     # 自定义配置 proto
    └── BUILD              # Proto 编译规则
```

### 6.2 BUILD 文件模板

```python
load("//tools/build_defs:apollo_plugin.bzl", "apollo_plugin")

apollo_plugin(
    name = "lib<plugin_name>.so",
    srcs = ["<plugin>.cc", ...],
    hdrs = ["<plugin>.h", ...],
    deps = [
        "//modules/planning/planning_interface_base/...",
        "//modules/planning/planning_base:planning_base",
        # ...
    ],
    description = "plugins.xml",
)
```

### 6.3 plugins.xml 模板

```xml
<library path="modules/planning/<type>/<name>/lib<name>.so">
    <class type="apollo::planning::<ClassName>" base_class="apollo::planning::<BaseClass>"></class>
</library>
```

> ⚠️ **关键约束**：
> - 必须使用 `<library>` 格式，**不能**使用 `<plugin>` 格式
> - `path` 必须与 BUILD 中 `apollo_plugin` 规则的 `name` 匹配
> - `type` 使用包含命名空间的完整类名
> - `base_class` 必须是对应基类的完整限定名：
>   - Scenario → `apollo::planning::Scenario`
>   - Task → `apollo::planning::Task`（不是 PathGeneration 等子类）
>   - TrafficRule → `apollo::planning::TrafficRule`

### 6.4 cyberfile.xml 模板

```xml
<package>
  <name>planning-<type>-<name></name>
  <version>0.0.1</version>
  <description>Plugin description</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <dependencies>
    <dependency name="planning-<base>"/>
  </dependencies>
</package>
```

---

## 七、新增插件 Checklist

### 7.1 新增 Scenario

1. [ ] 创建目录 `modules/planning/scenarios/<name>/` + 子目录 `proto/`、`conf/`
2. [ ] 写 proto（如需自定义参数）+ `proto/BUILD`
3. [ ] 写 Scenario 类（`.h` + `.cc`）：`Init`/`IsTransferable`/`Enter`/`Exit`
4. [ ] 写 Stage 类（`.h` + `.cc`）：每个 Stage 实现 `Process()`，最后一个 Stage 调用 `FinishScenario()`
5. [ ] 写 `conf/pipeline.pb.txt`：声明 Stage 列表和每个 Stage 的 Task 链
6. [ ] 写 `conf/scenario_conf.pb.txt`（如需场景参数）
7. [ ] 写 `BUILD`、`plugins.xml`（`<library>` 格式）、`cyberfile.xml`
8. [ ] 在 `public_road_planner_config.pb.txt` 中追加 `scenario: { name: "...", type: "..." }`
9. [ ] 编译 → `aem profile use default` → 测试

### 7.2 新增 Task

1. [ ] 创建目录 `modules/planning/tasks/<name>/` + 子目录
2. [ ] 确定继承类型：`PathGeneration` / `Decider` / `SpeedOptimizer` / `TrajectoryOptimizer`
3. [ ] 写 Task 类（`.h` + `.cc`）：实现 `Init()` 和 `Process()`
4. [ ] 写 `conf/default_conf.pb.txt`（如需可配置参数）
5. [ ] 写 `BUILD`、`plugins.xml`、`cyberfile.xml`
6. [ ] 在目标场景的 `conf/pipeline.pb.txt` 中追加 `task: { name: "...", type: "..." }`
7. [ ] 编译 → `aem profile use default` → 测试

### 7.3 新增 TrafficRule

1. [ ] 创建目录 `modules/planning/traffic_rules/<name>/`
2. [ ] 写 TrafficRule 类（`.h` + `.cc`）：实现 `Init`/`Reset`/`ApplyRule`
3. [ ] 写 proto + `conf/default_conf.pb.txt`（如需自定义参数）
4. [ ] 写 `BUILD`、`plugins.xml`、`cyberfile.xml`
5. [ ] 在 `traffic_rule_config.pb.txt` 中追加 `rule: { name: "...", type: "..." }`
6. [ ] 编译 → `aem profile use default` → 测试

---

## 八、关键参数速查

### 8.1 限速参数

| 参数 | 文件 | 说明 |
|------|------|------|
| `--default_cruise_speed` | `planning.conf` | 全局巡航速度 (m/s)，profile 覆盖 gflags 默认值 |
| `--planning_upper_speed_limit` | `planning.conf` | 速度上界 (m/s)，被 SpeedBoundsDecider 等 Task 使用 |
| `FLAGS_speed_upper_bound` | `planning_gflags.cc` | 硬性速度上界 (m/s)，代码级默认 40.0 |
| `FLAGS_default_city_road_speed_limit` | `planning_gflags.cc` | 城市道路默认限速，代码级默认 15.67 (35mph) |

### 8.2 停车距离参数

| 参数 | 文件 | 默认值 |
|------|------|--------|
| `stop_distance` (traffic_light) | `traffic_rules/traffic_light/conf/default_conf.pb.txt` | 1.0 m |
| `stop_distance` (stop_sign) | `traffic_rules/stop_sign/conf/default_conf.pb.txt` | 1.0 m |
| `stop_distance` (emergency_stop) | `scenarios/emergency_stop/conf/scenario_conf.pb.txt` | 1.0 m |
| `stop_distance` (emergency_pull_over) | `scenarios/emergency_pull_over/conf/scenario_conf.pb.txt` | 1.5 m |

### 8.3 横向距离参数

| 参数 | 文件 | 默认值 | 说明 |
|------|------|--------|------|
| `max_lateral_left_nudge_dis` | `tasks/obstacle_nudge_decider/conf/default_conf.pb.txt` | 3.0 m | 左侧最大绕行距离 |
| `static_obstacle_nudge_l_buffer` | `planning_gflags.cc` | 0.3 m | 静态障碍物绕行最小 l 距离 |
| `nonstatic_obstacle_nudge_l_buffer` | `planning_gflags.cc` | 0.4 m | 非静态障碍物绕行最小 l 距离 |
| `obstacle_lat_buffer` | `planning_gflags.cc` | 0.4 m | 障碍物横向 buffer |
| `lateral_buffer` (smoother) | `discrete_points_smoother_config.pb.txt` | 0.2 m | 路径平滑横向 buffer |

### 8.4 曲率限速参数

| 参数 | 说明 |
|------|------|
| `max_centric_acceleration_limit` | 最大向心加速度限制，曲率限速公式 $v = \sqrt{a_{max} / \max(\vert\kappa\vert, \kappa_{min})}$ |

---

## 九、常见陷阱与注意事项

| 陷阱 | 说明 |
|------|------|
| **plugins.xml 格式错误** | 必须使用 `<library>` 格式，不能用 `<plugin>` 格式 |
| **Task base_class 错误** | plugins.xml 中 base_class 必须是 `apollo::planning::Task`，不是子类 |
| **最后一个 Stage 未调用 FinishScenario()** | 改用 `FinishStage()` 会导致 Scenario 永远不返回 STATUS_DONE |
| **编译后忘记 aem profile use** | pb.txt 配置修改后需 `aem profile use default` 才能生效 |
| **Scenario 优先级顺序** | `public_road_planner_config.pb.txt` 中 scenario 列表顺序 = 优先级 |
| **规则执行顺序 ≠ 最终优先级** | TrafficRule 先执行先构建 StopDecision，但最终取 s 最小的 StopDecision |
| **IsTransferable 对 nullptr 的处理** | `other_scenario` 可能为 nullptr（初始状态），需做好空指针保护 |
| **pipeline.pb.txt 中 Task 顺序** | Task 按声明顺序执行，前一个的输出是后一个的输入 |
| **BaseStageCreep ProcessCreep 调用顺序** | 先 `ProcessCreep()` 构建虚拟障碍物，再 `ExecuteTaskOnReferenceLine()` |
