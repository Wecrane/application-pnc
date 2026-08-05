# scenarios 模块分析报告

> 178 文件 / ~12,000 行 | 17 个驾驶场景

## 功能概述

每个 Scenario 继承 `Scenario` 基类，包含多个 `Stage`（按 pipeline.pb.txt 定义顺序执行）。每个 Stage 指定一组 Task 插件链。

---

## 场景清单（按代码量排序）

| # | 场景 | 文件 | 行数 | Stage 数 | 核心功能 |
|---|------|------|------|----------|----------|
| 1 | **stop_sign_unprotected** | 16 | 1,513 | 4 | 无保护停车标志路口：pre_stop→stop→creep→intersection_cruise |
| 2 | **pull_over** | 16 | 1,212 | 5+ | 靠边停车：approach→pull_over→retreat→... |
| 3 | **park_and_go** | 19 | 1,172 | 5+ | 泊出起步：check→cruise→pre_stop→stop→creep |
| 4 | **traffic_light_unprotected_right_turn** | 13 | 1,052 | 3 | 无保护右转：creep→intersection_cruise→... |
| 5 | **lane_follow_park** | 7 | 1,006 | 2 | 泊车模式车道跟随 |
| 6 | **traffic_light_unprotected_left_turn** | 13 | 995 | 3 | 无保护左转：creep→intersection_cruise→... |
| 7 | **yield_sign** | 10 | 849 | 3 | 让行标志：creep→cruise→... |
| 8 | **emergency_pull_over** | 13 | 792 | 4 | 紧急靠边停车（简化流程） |
| 9 | **valet_parking_park** | 9 | 788 | 4 | 代客泊车入库 |
| 10 | **bare_intersection_unprotected** | 9 | 688 | 3 | 裸路口（无交通灯） |
| 11 | **valet_parking** | 10 | 647 | 4 | 代客泊车（停车场驾驶） |
| 12 | **traffic_light_protected** | 10 | 636 | 3 | 有保护红绿灯路口 |
| 13 | **emergency_stop** | 10 | 622 | 3 | 紧急停车 |
| 14 | **square** | 8 | 590 | 2 | 广场/停车场 |
| 15 | **lane_follow** ⭐ | 5 | 474 | 2 | **默认车道跟随（最常用）** |
| 16 | **free_space** | 5 | 385 | 2 | 无结构化道路 |
| 17 | **large_curvature** | 5 | 309 | 2 | 大曲率弯道 |

---

## 场景分类

### 🟢 车道跟随类
| 场景 | 说明 |
|------|------|
| `lane_follow` ⭐ | 默认场景，沿车道行驶 |
| `lane_follow_park` | 泊车模式下车道跟随 |
| `large_curvature` | 大曲率弯道专用处理 |
| `free_space` | 无车道线的自由空间 |

### 🔴 路口通行类
| 场景 | 说明 |
|------|------|
| `stop_sign_unprotected` | 无保护停车标志路口（4 Stage） |
| `traffic_light_protected` | 绿灯保护路口 |
| `traffic_light_unprotected_left_turn` | 无保护左转（蠕行判断） |
| `traffic_light_unprotected_right_turn` | 无保护右转（蠕行判断） |
| `bare_intersection_unprotected` | 裸路口（无线无灯） |

### 🟡 停车让行类
| 场景 | 说明 |
|------|------|
| `yield_sign` | 让行标志 |
| `pull_over` | 靠边停车（多 Stage 完整流程） |
| `emergency_pull_over` | 紧急靠边停车（简化） |
| `emergency_stop` | 紧急停车 |

### 🔵 泊车类
| 场景 | 说明 |
|------|------|
| `park_and_go` | 泊出起步 |
| `valet_parking` | 代客泊车（停车场内行驶） |
| `valet_parking_park` | 代客泊车入库 |
| `square` | 广场/停车场规划 |

---

## Stage 类型模式

| Stage 基类 | 用途 | 典型场景 |
|------------|------|----------|
| `Stage` (直接继承) | 通用 Stage | lane_follow, free_space |
| `BaseStageCreep` | 蠕行靠近停止线 | stop_sign, traffic_light |
| `BaseStageCruise` | 巡航通过 | 路口通过阶段 |
| `BaseStageTrafficLightCreep` | 红绿灯蠕行 | traffic_light_unprotected_* |
| `BaseStageTrafficLightCruise` | 红绿灯巡航 | traffic_light_protected |

## 场景切换机制

```
ScenarioManager::Update()
├── 当前场景 STATUS_PROCESSING? → 不切换（优先级保护）
├── 遍历 scenario_list_:
│   └── scenario->IsTransferable(current_scenario, frame)
├── 找到可切换场景:
│   ├── current_scenario_->Exit(frame)
│   ├── new_scenario->Reset()
│   └── new_scenario->Enter(frame)
└── 无可用场景? → 保持当前场景
```
