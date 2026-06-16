# 10 - Control 控制模块

> 来源：`apollo_docs_md/框架设计/软件核心/核心模块/control.md`

---

## Control 模块概述

Control（控制）模块是 Planning 模块的**直接下游**，接收 Planning 输出的轨迹，通过控制算法生成车辆的油门、刹车、方向盘等执行命令。

### 数据流

```
Planning ──→ ADCTrajectory ──→ Control ──→ ChassisCommand
                (轨迹)              │         (底盘命令)
                                    ↓
                              CAN Bus → 车辆执行
```

### Control 接收的轨迹格式

Planning 输出的 `ADCTrajectory` 包含：
- 轨迹点序列（x, y, z 坐标）
- 每个点的时间戳、速度、加速度
- 轨迹的参考时间

### 比赛关系

- **不允许修改 Control 模块**
- 但需要了解 Control 的输入要求，确保 Planning 输出的轨迹**可被 Control 执行**
- 如果轨迹不合理（如曲率过大、加速度超限），Control 可能无法跟踪

### Planning → Control 的约束

| 约束 | 说明 |
|------|------|
| 轨迹点密度 | 应足够密集（通常 0.1s 间隔） |
| 速度连续性 | 速度不应突变 |
| 曲率限值 | 不能超过车辆最大转向角 |
| 加速度限值 | 不能超过车辆最大加减速能力 |

---

## 调试 Control 反馈

即使不能改 Control，可以查看日志判断问题是否在 Planning 侧：

```bash
# 查看 Control 日志
tail -f data/log/control.INFO

# 如果 Control 报 "trajectory not feasible"
# → Planning 输出的轨迹不合理，需要调整
```

---

## 参考

- Control 模块：`apollo_docs_md/框架设计/软件核心/核心模块/control.md`
