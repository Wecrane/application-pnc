# Apollo 场景新建与切换指南

本文基于本项目隐藏目录中的 Apollo 知识库和当前工程结构整理，重点回答两个问题：

- 如何新建一个 Planning 里的 `Scenario` 插件。
- 如何在 DreamView / Apollo Studio / Profile 中切换仿真场景。

> 重要区分：Apollo 里常说的“场景”至少有两层含义。
>
> 1. **Planning 代码场景**：`modules/planning/scenarios/*` 下的 `Scenario` 插件，是规划模块的状态机，由场景管理器根据 `IsTransferable()` 条件自动切换。
> 2. **仿真交通流场景**：Apollo Studio / DreamView 里选择的场景集、地图、障碍物、红绿灯等仿真资源，用来复现赛事题目或调试路线。

## 资料来源

本次检索重点查看了点开头的隐藏目录，主要有效资料位于：

| 路径 | 用途 |
| --- | --- |
| `.claude/apollo-knowledge/knowledge_index.md` | 本地 Apollo EDU 知识库索引 |
| `.claude/apollo-knowledge/03_规划PnC/07_插件机制—新增Scenario插件.md` | 新增 Planning `Scenario` 插件的主参考 |
| `.claude/apollo-knowledge/03_规划PnC/04_使用SimControl仿真调试.md` | DreamView + SimControl 调试流程 |
| `.claude/apollo-knowledge/02_安装部署/04_赛事场景插件下载指南.md` | Profile 插件安装，用于同步 Apollo Studio 场景 |
| `.claude/apollo-knowledge/05_工程框架与工具/Dreamview功能介绍.md` | DreamView 功能、Route Editing、Default Routing |
| `.claude/apollo-knowledge/07_FAQ故障排查/dreamview地图不加载或场景不跳转.md` | 地图不加载、场景不跳转排查 |
| `modules/planning/scenarios/*` | 当前项目实际 Scenario 插件结构 |
| `modules/planning/planning_component/conf/public_road_planner_config.pb.txt` | 当前 Planning 场景管理流水线配置 |

`.codex`、`.agents` 目录本次没有发现项目内可用的 skill 文件；`.oldcode` 下有一份较完整的 Apollo 旧代码参考，可用于对照官方模块结构。

## 一、Planning 场景机制速览

Planning 2.0 的调度关系可以理解为三层：

| 层级 | 作用 | 典型文件 |
| --- | --- | --- |
| `Scenario` | 规划模块的大状态机，决定当前处于跟车、红绿灯、让行、自主泊车等哪类场景 | `*_scenario.cc/.h` |
| `Stage` | 某个场景内部的小状态机，例如接近停止线、爬行、巡航通过 | `stage_*.cc/.h` |
| `Task` | 真正执行规划计算的任务，例如路径决策、速度决策、fallback 等 | `pipeline.pb.txt` 中配置 |

场景切换不是在 DreamView 里手动选 Planning `Scenario`，而是由 Planning 在运行时自动判断：

1. `public_road_planner_config.pb.txt` 中按顺序列出可用场景。
2. 场景管理器遍历这些场景。
3. 每个场景通过 `IsTransferable(const Scenario* other_scenario, const Frame& frame)` 判断是否可切入。
4. 切入后执行 `Enter()`，场景结束或退出时执行 `Exit()`。
5. 场景内部由 `Stage::Process()` 通过 `FinishStage(next_stage)` 或 `FinishScenario()` 切换阶段或结束场景。

当前项目的场景配置入口是：

```text
modules/planning/planning_component/conf/public_road_planner_config.pb.txt
```

其中已有场景包括：

```protobuf
scenario {
    name: "YIELD_SIGN"
    type: "YieldSignScenario"
}
scenario {
    name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN"
    type: "TrafficLightUnprotectedLeftTurnScenario"
}
scenario {
    name: "LANE_FOLLOW"
    type: "LaneFollowScenario"
}
```

## 二、新建 Planning Scenario 插件

下面以新建 `my_custom_scenario` 为例说明流程。真实开发时，建议先复制或参考一个结构接近的已有场景，例如：

- `modules/planning/scenarios/yield_sign`
- `modules/planning/scenarios/traffic_light_unprotected_left_turn`
- `modules/planning/scenarios/bare_intersection_unprotected`

### 1. 创建插件模板

在 Apollo 包管理环境中，可以用 `buildtool create` 创建插件模板：

```bash
buildtool create \
  --template plugin \
  --namespaces planning \
  --dependencies planning:binary:planning \
  --base_class_name apollo::planning::Scenario \
  --class_name MyCustomScenario \
  modules/planning/scenarios/my_custom_scenario
```

如果模板生成不完整，也可以按已有目录手工补齐。一个完整 Scenario 插件通常包含：

```text
modules/planning/scenarios/my_custom_scenario/
├── BUILD
├── README_cn.md
├── cyberfile.xml
├── plugins.xml
├── my_custom_scenario.cc
├── my_custom_scenario.h
├── context.h
├── stage_approach.cc
├── stage_approach.h
├── conf/
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
└── proto/
    ├── BUILD
    └── my_custom_scenario.proto
```

### 2. 实现 Scenario 类

`Scenario` 类至少需要关注四个函数：

| 函数 | 作用 |
| --- | --- |
| `Init()` | 调用基类初始化，并加载 `scenario_conf.pb.txt` |
| `IsTransferable()` | 判断当前帧是否满足切入条件 |
| `Enter()` | 进入场景时缓存上下文、初始化状态 |
| `Exit()` | 退出场景时清理上下文、恢复全局状态 |

典型头文件结构：

```cpp
#pragma once

#include <memory>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"
#include "modules/planning/scenarios/my_custom_scenario/proto/my_custom_scenario.pb.h"

namespace apollo {
namespace planning {

struct MyCustomScenarioContext : public ScenarioContext {
  MyCustomScenarioConfig scenario_config;
};

class MyCustomScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;
  bool IsTransferable(const Scenario* other_scenario,
                      const Frame& frame) override;
  bool Enter(Frame* frame) override;
  bool Exit(Frame* frame) override;

  MyCustomScenarioContext* GetContext() override { return &context_; }

 private:
  MyCustomScenarioContext context_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyCustomScenario,
                                     apollo::planning::Scenario)

}  // namespace planning
}  // namespace apollo
```

`IsTransferable()` 是场景切换的核心。常见判断来源包括：

- `frame.reference_line_info()` 是否为空。
- `ReferenceLineInfo::FirstEncounteredOverlaps()` 中是否遇到红绿灯、停止标志、让行标志、路口、人行道等 overlap。
- 主车前沿或后沿 `s` 值与 overlap 的距离。
- 车道转向类型，例如 `LEFT_TURN`、`RIGHT_TURN`。
- 感知障碍物、交通灯颜色、routing 终点或泊车命令。

### 3. 实现 Stage 类

每个阶段继承 `Stage`，主要实现 `Process()`：

```cpp
class MyCustomStageApproach : public Stage {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::MyCustomStageApproach,
                                     apollo::planning::Stage)
```

`Process()` 内通常做这些事：

1. 读取 `ScenarioContext` 中的配置和缓存信息。
2. 根据需要设置当前阶段限速：

   ```cpp
   auto& reference_line_info = frame->mutable_reference_line_info()->front();
   reference_line_info.LimitCruiseSpeed(scenario_config.approach_speed());
   ```

3. 执行流水线中的任务：

   ```cpp
   StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
   ```

4. 根据条件决定继续、切下一个 Stage 或结束 Scenario：

   ```cpp
   if (need_next_stage) {
     next_stage_ = "MY_CUSTOM_STAGE_CRUISE";
     return StageResult(StageStatusType::FINISHED);
   }

   if (scenario_done) {
     return FinishScenario();
   }

   return result.SetStageStatus(StageStatusType::RUNNING);
   ```

### 4. 配置 pipeline.pb.txt

`conf/pipeline.pb.txt` 决定这个场景有哪些 Stage，以及每个 Stage 执行哪些 Task：

```protobuf
stage: {
  name: "MY_CUSTOM_STAGE_APPROACH"
  type: "MyCustomStageApproach"
  enabled: true
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  task {
    name: "FALLBACK_PATH"
    type: "FallbackPath"
  }
  task {
    name: "PATH_DECIDER"
    type: "PathDecider"
  }
  task {
    name: "RULE_BASED_STOP_DECIDER"
    type: "RuleBasedStopDecider"
  }
  task {
    name: "SPEED_BOUNDS_PRIORI_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "SPEED_DECIDER"
    type: "SpeedDecider"
  }
  task {
    name: "PIECEWISE_JERK_SPEED"
    type: "PiecewiseJerkSpeedOptimizer"
  }
}
```

任务列表不要凭空发明，优先从相似场景复制。例如红绿灯相关场景可参考 `traffic_light_unprotected_left_turn`，让行相关场景可参考 `yield_sign`。

### 5. 配置 scenario_conf.pb.txt 和 proto

先定义参数 proto：

```protobuf
syntax = "proto2";

package apollo.planning;

message MyCustomScenarioConfig {
  optional double start_scenario_distance = 1 [default = 30.0];
  optional double approach_speed = 2 [default = 5.0];
  optional double cruise_speed = 3 [default = 8.0];
}
```

再在 `conf/scenario_conf.pb.txt` 写实际参数：

```protobuf
start_scenario_distance: 30.0
approach_speed: 5.0
cruise_speed: 8.0
```

### 6. 注册插件

在 `plugins.xml` 中注册 Scenario 和所有 Stage：

```xml
<library path="modules/planning/scenarios/my_custom_scenario/libmy_custom_scenario.so">
    <class type="apollo::planning::MyCustomScenario" base_class="apollo::planning::Scenario"></class>
    <class type="apollo::planning::MyCustomStageApproach" base_class="apollo::planning::Stage"></class>
</library>
```

`BUILD` 中需要使用 `apollo_plugin`，结构可参考当前项目的 `modules/planning/scenarios/yield_sign/BUILD`：

```python
apollo_plugin(
    name = "libmy_custom_scenario.so",
    srcs = [
        "my_custom_scenario.cc",
        "stage_approach.cc",
    ],
    hdrs = [
        "my_custom_scenario.h",
        "stage_approach.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/my_custom_scenario/proto:my_custom_scenario_cc_proto",
    ],
)
```

### 7. 加入 Planning 场景管理流水线

把新场景加入：

```text
modules/planning/planning_component/conf/public_road_planner_config.pb.txt
```

示例：

```protobuf
scenario {
    name: "MY_CUSTOM"
    type: "MyCustomScenario"
}
```

顺序会影响切换优先级。更具体的场景通常放在 `LANE_FOLLOW` 之前，因为 `LANE_FOLLOW` 是兜底场景。

如果使用 profile 配置，实际生效文件可能在：

```text
profiles/current/modules/planning/planning_component/conf/
profiles/default/modules/planning/planning_component/conf/
```

本仓库当前没有这些 Planning profile 文件，但知识库提醒：源码或配置编译后，`profiles/default` 下软链配置可能被覆盖，必要时执行：

```bash
aem profile use default
```

### 8. 编译与验证

编译 Planning：

```bash
buildtool build -p modules/planning/
```

或编译核心包：

```bash
buildtool build -p core
```

启动 DreamView+：

```bash
aem bootstrap start --plus
```

如果 Planning 模块无法启动，单独启动 Planning 看日志：

```bash
mainboard -d /apollo/modules/planning/planning_component/dag/planning.dag
```

验证是否切入新场景，可以看 Planning 日志、DreamView 调试信息，或检索日志中的 scenario/stage 切换信息：

```bash
grep -R "MyCustomScenario\|MY_CUSTOM" data/log/
```

## 三、切换 Planning Scenario 的方法

Planning `Scenario` 的切换本质上是代码逻辑驱动，不是 UI 手动选择。

### 方式 1：修改 IsTransferable 切入条件

这是最常见的方式。比如：

- 距离某个 overlap 小于阈值时切入。
- 检测到某类障碍物时切入。
- 进入某段车道或某个路口时切入。
- 接收到特定外部命令时切入。

调整后重新编译，并在对应仿真场景里触发条件。

### 方式 2：调整 public_road_planner_config.pb.txt 顺序

如果多个场景都可能满足切入条件，配置顺序会影响谁先被选中。新增场景一般放在比 `LANE_FOLLOW` 更靠前的位置。

### 方式 3：通过 Stage 状态切换

场景内部阶段切换由 `Stage::Process()` 决定：

```cpp
next_stage_ = "MY_CUSTOM_STAGE_CRUISE";
return StageResult(StageStatusType::FINISHED);
```

结束整个场景：

```cpp
return FinishScenario();
```

### 方式 4：通过仿真输入触发

有些场景依赖交通灯、障碍物、routing、泊车点等外部输入。可以通过 DreamView / SimControl / Apollo Studio 场景来触发：

- 修改交通灯状态。
- 切换含有特定障碍物的场景。
- 在 Route Editing 中设置经过目标区域的路线。
- 选择包含对应地图元素的赛事场景。

## 四、新建或同步仿真交通流场景

DreamView 本身主要用于运行和调试，不能完整制作带交通流的云端场景。知识库里的建议流程是：

1. 在 Apollo Studio 中创建或选择场景集。
2. 通过 Profile 插件把 Apollo Studio 场景同步到本地 DreamView。
3. 在 DreamView 的 Profile / 资源中心中选择场景集。
4. 在 SimControl 中运行、调试 Planning。

### 1. 安装 Profile 插件

进入 Apollo 环境：

```bash
cd application-pnc
aem enter
```

启动 DreamView+：

```bash
aem bootstrap start --plus
```

在 Apollo Studio 个人信息中生成 Profile 插件密钥，然后回到 Docker 环境，在 `/apollo_workspace` 下执行复制出来的安装命令。

安装后检查：

```bash
ls ~/.apollo/dreamview/
```

重启 DreamView+：

```bash
aem bootstrap stop
aem bootstrap start --plus
```

检查插件节点：

```bash
cyber_node list
```

如果能看到 `studio_connector` 相关节点，说明 Profile 插件已启动。

### 2. 获取赛事场景集和地图

知识库里的入口是：

```text
.claude/apollo-knowledge/09_地图资源/2026星火自动驾驶大赛场景集和地图获取.md
```

该文档指向赛事场景插件下载和 2026 场景集/地图获取。下载完成后，确认本地有地图：

```bash
ls data/map_data/
```

下载或切换地图后重启 DreamView+：

```bash
aem bootstrap restart --plus
```

## 五、在 DreamView 中切换仿真场景

### 1. 启动环境

宿主机：

```bash
cd application-pnc
aem start
aem enter
```

容器内：

```bash
aem bootstrap start --plus
```

浏览器打开：

```text
http://localhost:8888
```

### 2. 选择仿真模式和地图

在 DreamView 顶部配置中选择：

- 车辆：例如 `MkzExample`
- 地图：例如 `Apollo Virtual Map` 或赛事地图
- Tasks：选择 `SimControl`

### 3. 启动相关模块

在左侧 `Module Control` 中启动 PnC 链路常用模块：

- `Planning`
- `Prediction`
- `Routing`
- `Control`，按调试需要启动

如果只调 Planning 行为，至少保证 Planning、Prediction、Routing 链路可用。

### 4. 打开或切换场景

安装 Profile 插件并同步场景后，在 DreamView 的 Profile / 资源中心中选择 Apollo Studio 侧的场景集，再选择具体场景。

如果是 SimControl 文档中的流程：

1. 进入 `SimControl`。
2. 下载或打开赛事场景集。
3. 选择具体赛事场景。
4. 等待地图、障碍物、信号灯等资源加载完成。

### 5. 设置 Routing 路线

在左侧打开 `Route Editing`：

1. 在地图上点击或拖动设置起点、途经点、终点。
2. 点击 `Send Routing Request`。
3. 地图上出现红色 routing 路径后，Planning 会输出浅蓝色局部轨迹。

也可以使用 DreamView 的 `Default Routing`，即预定义 POI 路线。POI 配置通常位于对应地图目录下的：

```text
modules/map/data/<map_name>/default_end_way_point.txt
```

### 6. 切换到另一个仿真场景

推荐流程：

1. 暂停或停止当前仿真。
2. 在 Profile / 场景集列表中选择另一个场景。
3. 确认地图随场景切换成功。
4. 必要时点击 `Reset all` 或重启相关模块。
5. 重新设置 Routing 并发送请求。

如果切换后地图不变、场景不跳转或资源加载异常，按下面排查。

## 六、常见问题排查

### 1. 场景不跳转或地图不加载

知识库给出的修复方式：

```bash
cd ~/.apollo/resources/dynamic_models/models
rm -r *
cd /apollo_workspace
aem bootstrap restart --plus
```

然后：

```bash
ls data/map_data/
```

如果没有对应地图，先下载地图；下载后再次重启：

```bash
aem bootstrap restart --plus
```

再切到 `scenario_sim` / `SimControl`，选择对应地图与场景查看是否加载。

### 2. 安装 Profile 后 DreamView 仍看不到场景

检查点：

- Profile 密钥是否过期，知识库说明密钥有效期较短，过期需重新生成。
- 安装后是否重启了 DreamView。
- `cyber_node list` 是否有 `studio_connector` 节点。
- Apollo Studio 侧是否已经创建或收藏了场景集。

### 3. 新 Scenario 编译通过但没有切入

按顺序检查：

1. `plugins.xml` 是否注册了 Scenario 和 Stage。
2. `BUILD` 的 `apollo_plugin` 是否使用了正确的 `description = ":plugins.xml"`。
3. `public_road_planner_config.pb.txt` 是否加入了新 `scenario`。
4. `type` 是否和 C++ 注册类名一致。
5. `IsTransferable()` 条件是否真的被满足。
6. 场景顺序是否被前面的场景抢先匹配。
7. 仿真地图中是否真的有需要的 overlap / 信号灯 / 路口 / 车道元素。

### 4. 新 Stage 没有执行

检查：

- `pipeline.pb.txt` 中 Stage 的 `type` 是否等于注册类名。
- 上一个 Stage 是否设置了正确的 `next_stage_`。
- Stage 是否写入 `plugins.xml`。
- `Process()` 是否一直返回 `RUNNING`。

### 5. 参数修改后不生效

检查：

- 修改的是 `modules/` 下源码配置，还是 `profiles/default` 下运行配置。
- 编译后是否需要执行：

  ```bash
  aem profile use default
  ```

- DreamView / Planning 是否需要重启。

## 七、推荐开发闭环

新增 Planning 场景时，建议按这个节奏推进：

1. 选一个最相近的已有 Scenario 作为参考。
2. 用 `buildtool create` 或复制结构创建新插件。
3. 先只实现一个简单 `IsTransferable()` 和一个 Stage，确认能编译、能加载、能切入。
4. 再逐步增加复杂 Stage、限速、停止墙、交通灯或障碍物逻辑。
5. 用 Apollo Studio / Profile / SimControl 准备能稳定触发条件的仿真场景。
6. 每次修改后编译 Planning，启动 DreamView+ 验证。
7. 通过日志确认 `Scenario` 和 `Stage` 的切换路径。

常用命令汇总：

```bash
# 进入环境
cd application-pnc
aem start
aem enter

# 编译 Planning
buildtool build -p modules/planning/

# 编译后恢复 profile 配置
aem profile use default

# 启动 DreamView+
aem bootstrap start --plus

# 重启 DreamView+
aem bootstrap restart --plus

# 单独启动 Planning 排查
mainboard -d /apollo/modules/planning/planning_component/dag/planning.dag

# 查看本地地图
ls data/map_data/

# 查看 Profile 插件节点
cyber_node list
```

