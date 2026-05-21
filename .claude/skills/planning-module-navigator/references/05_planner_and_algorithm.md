# 规划器与算法详解

## Planner 规划器

### 参考线平滑算法

**路径**：`modules/planning/planning_component/docs/reference_line_smoother_cn.md`

参考线平滑使用**二次规划（QP）+ 样条插值**方法。

#### 算法原理

将寻路路径划分为 n 段，每段用 2 个 5 次多项式表示：

$$x = f_i(t) = a_{i0} + a_{i1}t + a_{i2}t^2 + a_{i3}t^3 + a_{i4}t^4 + a_{i5}t^5$$

$$y = g_i(t) = b_{i0} + b_{i1}t + b_{i2}t^2 + b_{i3}t^3 + b_{i4}t^4 + b_{i5}t^5$$

#### 目标函数

最小化各段的三阶导数平方积分：

$$cost = \sum_{i=1}^{n} \left( \int_{0}^{t_i} (f_i''')^2(t) dt + \int_{0}^{t_i} (g_i''')^2(t) dt \right)$$

转化为标准 QP 形式：

$$\min \frac{1}{2} x^T H x + f^T x$$

约束条件：
- $LB \leq x \leq UB$
- $A_{eq}x = b_{eq}$（等式约束）
- $Ax \leq b$（不等式约束）

#### 约束条件

1. **平滑节点约束**：相邻段连接处保证连续性
   $$f_k(s_k) = f_{k+1}(s_0), \quad f'_k(s_k) = f'_{k+1}(s_0), \quad f''_k(s_k) = f''_{k+1}(s_0)$$

2. **边界约束**：保证路径经过起始点和目标点

> 来源：`modules/planning/planning_component/docs/reference_line_smoother_cn.md`

---

## 路径优化

### PathBounds（路径边界）

由各 Task 计算生成，定义车辆在 SL 坐标系中横向可行驶的边界范围。

### PathOptimizer

使用优化算法在路径边界内生成平滑路径。常用算法包括：
- **分段 Jerk 路径优化**
- **QP 二次规划**

### 候选路径评估

路径评估标准：
- 路径是否为空
- 路径是否远离参考线
- 路径是否远离道路
- 路径是否与静态障碍物碰撞
- 路径终点是否在逆向车道上
- 路径长度和自车横向位置

选择最优路径的因素：道路长度、是否借道逆向车道、回本车道的速度等。

> 来源：`modules/planning/tasks/lane_borrow_path_generic/README_cn.md`

---

## 速度优化

### SpeedBounds（速度边界）

在 ST 图中计算速度的上下边界，约束来自：
- 障碍物（动态/静态）
- 交通规则（停止线、人行横道）
- 车辆动力学限制

### SpeedOptimizer

使用优化算法生成平滑的速度剖面：
- **PiecewiseJerkSpeed**：分段 jerk 优化
- **PiecewiseJerkSpeedNonlinear**：非线性版本

---

## PnC Map

**路径**：`modules/planning/pnc_map/`

根据 planning 导航命令或地图信息生成参考线数据，作为 planning 局部路径规划的路线参考。

---

## Planner 类型

`modules/planning/planner/` 目录包含多种规划器插件：

| Planner 类型 | 说明 |
|-------------|------|
| `PublicRoadPlanner` | 公共道路规划器（默认） |
| `OpenSpacePlanner` | 开放空间规划器（泊车、掉头等） |
| `LearningPlanner` | 基于学习的规划器 |

---

## SL 坐标系

Apollo 使用 SL（Station-Lateral）坐标系进行路径规划：
- **S 方向**：沿参考线的纵向距离
- **L 方向**：偏离参考线的横向距离

障碍物在 SL 坐标系中投影，规划器在 SL 空间中计算路径，再转换回 XY 坐标。

---

## ST 图

速度规划使用 ST（Station-Time）图：
- **S 轴**：沿路径的纵向位置
- **T 轴**：时间

障碍物在 ST 图中投影为"禁区"，速度规划器在禁区间生成安全的速度剖面。
