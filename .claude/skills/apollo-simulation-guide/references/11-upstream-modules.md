# 11 - 上下游模块概述

> 来源：`apollo_docs_md/框架设计/软件核心/核心模块/`

---

## Planning 的上下游关系

虽然比赛**只允许修改 Planning**，但 Planning 不是孤立运行的——它接收上游模块的数据，输出轨迹给下游。理解这些接口有助于写出更合理的规划逻辑。

```
┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│Perception│──→│Prediction│──→│ Planning │──→│ Control  │──→│ CanBus   │
│  (感知)  │   │  (预测)  │   │  (规划)  │   │  (控制)  │   │ (车辆总线)│
└──────────┘   └──────────┘   └──────────┘   └──────────┘   └──────────┘
      ↑              ↑              ↑
   Localization   HD-Map        Routing
     (定位)      (高精地图)     (全局路径)
```

---

## 上游模块 → Planning 的输入

### Perception（感知）→ Prediction → Planning

感知模块识别车辆周围环境，输出：

| 数据 | 说明 | Planning 中如何使用 |
|------|------|-------------------|
| 障碍物位置 (x,y,z) | 静态/动态障碍物坐标 | 绕行、让行决策 |
| 障碍物尺寸 (长宽高) | 障碍物体积 | 计算安全距离 |
| 障碍物类型 | 行人/车辆/自行车等 | 不同障碍物不同策略 |
| 交通灯状态 | 红/黄/绿 | TrafficLight 场景切入 |
| 车道线 | 车道边界 | 保持在车道内 |

### Prediction（预测）→ Planning

预测模块对动态障碍物的未来轨迹进行预测，输出：

| 数据 | 说明 | Planning 中如何使用 |
|------|------|-------------------|
| 障碍物预测轨迹 | 未来 N 秒的轨迹点序列 | 判断是否需要减速/让行 |
| 障碍物意图 | 左转/右转/直行/停车 | 交叉路口决策 |

### Localization（定位）→ Planning

提供自车当前位置和姿态：

| 数据 | 说明 |
|------|------|
| 自车坐标 (x,y,z) | 当前定位 |
| 航向角/速度 | 当前运动状态 |

### Routing（路由）→ Planning

输出全局导航路径 `RoutingResponse`，Planning 据此生成局部轨迹。

### HD-Map（高精地图）→ PnC Map

以库的形式提供道路结构化信息：车道、路口、交通标志位置等。

---

## Planning → 下游模块的输出

### Control（控制）

Planning 输出 `ADCTrajectory`（轨迹）给 Control：
- 轨迹点序列（x, y, z, θ）
- 每个点的时间戳、速度、加速度、jerk
- 轨迹长度通常 8 秒，0.1 秒间隔

**关键约束**：轨迹必须物理可行（曲率、加速度在车辆能力范围内），否则 Control 会报 `trajectory not feasible`。

---

## 比赛注意事项

- ❌ **不能改**：Perception、Prediction、Localization、Control、CanBus
- ✅ **只能改**：Planning（Scenario / Task / TrafficRule / Planner / 参数）
- ⚠️ Planning 拿到的上游数据是**仿真器模拟的**（SimControl 模拟），不是真实传感器数据
- 💡 如果发现"感知不准"或"预测不对"等问题，不能修改上游模块，只能在 Planning 侧通过参数调整来适应

---

## 参考

- 核心模块概述：`apollo_docs_md/框架设计/软件核心/核心模块/概述.md`
- Perception：`apollo_docs_md/框架设计/软件核心/核心模块/perception.md`
- Prediction：`apollo_docs_md/框架设计/软件核心/核心模块/prediction.md`
- Localization：`apollo_docs_md/框架设计/软件核心/核心模块/localization.md`
