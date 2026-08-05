# planning_component 模块分析报告

> 14 文件 / ~3,700 行 | 系统入口层

## 功能概述

`planning_component` 是 Apollo Planning 模块的 **CyberRT 组件入口**，负责：
1. 接收感知（PredictionObstacles）、底盘（Chassis）、定位（LocalizationEstimate）三通道数据
2. 聚合 7 个异步数据源（交通灯、Pad消息、StoryTelling、PerceptionEdge 等）
3. 根据配置选择规划策略，调度执行，发布 ADCTrajectory

## 核心类

| 类 | 文件 | 行数 | 职责 |
|-----|------|------|------|
| `PlanningComponent` | `planning_component.h/.cc` | 470 | CyberRT Component，数据聚合与调度 |
| `PlanningBase` | `planning_base.h/.cc` | 287 | Planner 策略抽象基类 |
| `OnLanePlanning` | `on_lane_planning.h/.cc` | 1,394 | 主规划调度器（常规模式） |
| `NaviPlanning` | `navi_planning.h/.cc` | 736 | 导航模式调度器 |

## 核心逻辑

### 1. 启动流程 (Init)
```
PlanningComponent::Init()
├── 创建 DependencyInjector
├── FLAGS_use_navigation_mode ?
│   ├── true  → new NaviPlanning(injector_)
│   └── false → new OnLanePlanning(injector_)
├── 加载 PlanningConfig protobuf
├── 初始化 MessageProcess（学习模式）
├── planning_base_->Init()
└── 建立 7 个 CyberRT Reader + Writer + Client
```

### 2. 主循环 (Proc)
```
Proc(prediction, chassis, localization)
├── CheckRerouting()          → 检测是否需要重路由
├── 聚合 LocalView            → 9 个数据源合并
├── CheckInput()              → 验证数据完整性
├── [learning_mode 分支]
│   ├── RL_TEST → return (不执行规划)
│   └── 正常   → planning_base_->RunOnce()
├── FillHeader + SetLocation  → 填充轨迹元数据
├── planning_writer_->Write() → 发布 ADCTrajectory
└── command_status_writer_->Write() → 发布执行状态
```

### 3. OnLanePlanning::RunOnce() 内部流程
```
RunOnce()
├── 更新车辆状态 (VehicleStateProvider)
├── 无效状态? → GenerateStopTrajectory() + return
├── 参考线更新/路由变更检测
├── TrajectoryStitcher::Stitch()     → 轨迹拼接
├── InitFrame()                       → 创建 Frame
├── traffic_decider_.Execute(frame)   → 交通规则决策
├── planner_->Plan()                  → 调用 Planner
├── planning_smoother_->Smooth()      → 轨迹平滑
└── FillPlanningPb()                  → 填充输出
```

## 设计模式
- **策略模式**：`PlanningBase` 多态，运行时选择 `OnLanePlanning` / `NaviPlanning`
- **工厂模式**：`LoadPlanner()` 通过 `PluginManager::CreateInstance<Planner>()` 动态创建
- **观察者模式**：7 个 CyberRT Reader 回调异步接收数据

## 潜在问题
1. `SetLocation` 中 `local_view_.localization_estimate` 未加锁保护
2. `RL_TEST` 模式下直接 return true 跳过规划输出
3. 高频消息回调中使用 `std::lock_guard` 拷贝 protobuf，存在性能开销
4. `Proc` 成功时的时间戳 dt 修正可能引入逆序问题
