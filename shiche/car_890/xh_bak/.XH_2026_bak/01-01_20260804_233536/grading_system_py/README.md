# grading_system_py 功能说明

## 概述

Apollo 自动驾驶仿真评分系统的 Python 实现，从原 C++ 版本移植而来。支持离线 bag 文件评分和实时 Cyber 通道评分两种模式。

## 支持的评分模块 (Metrics)

| # | Metric 名称 | Condition 类型 | 计分方式 | 说明 |
|---|------------|---------------|---------|------|
| 1 | SpeedLimit | `speed_condition` | 扣分制 | 速度上下限检测 + 区域限速 |
| 2 | AccelerationLimit | `acceleration_condition` | 扣分制 | 加速度范围检测 |
| 3 | HardBrake | `acceleration_condition` | 扣分制 | 急刹车检测（排除合理刹车） |
| 4 | OnRoad | `on_road_condition` | 扣分制 | 车辆是否在道路内 |
| 5 | ObstacleBypass | `obstacle_bypass_condition` | 扣分制 | 障碍物绕行横向距离/速度 |
| 6 | TTC | `ttc_condition` | 扣分制 | Time-To-Collision 碰撞风险 |
| 7 | CentripetalAccelerationLimit | `centripetal_acceleration_condition` | 扣分制 | 向心加速度限制 |
| 8 | TimeLimit | `time_limit_condition` | 得分制 | 是否超时 |
| 9 | ReachEnd | `region_overlap_lw_condition` | 得分制 | 是否到达终点 |
| 10 | Collision | `object_overlap_condition` | 扣分制 | 碰撞检测（多边形重叠） |
| 11 | RedLightStop / ContestMetric | `red_light_stop_condition` | 扣分制 | 红灯停车线距离 |
| 12 | CrosswalkStop / ContestMetric | `crosswalk_stop_condition` | 扣分制 | 人行横道停车距离 |

## 各模块详细说明

### 1. SpeedLimit（速度检测）

检测主车速度是否在允许范围内。

- 支持全局限速 `[min_speed, max_speed]`
- 支持 `speed_limit` 模式自动取地图限速
- 支持多边形限速区域 `speed_limit_regions`（车辆包围盒进入区域时生效）
- 超速扣分公式: `ceil((speed - max_speed) / deduction_unit) * single_deduction`

### 2. AccelerationLimit / HardBrake（加速度检测）

检测加速度是否在允许范围内。

- 通用模式: 加速度超出 `[min_acceleration, max_acceleration]` 则扣分
- HardBrake 模式: 检测急刹车并排除以下合理场景:
  - 障碍物 cut-in（射线相交检测）
  - TTC < 阈值（前方碰撞风险高）
  - 信号灯由绿转红/黄

### 3. OnRoad（在路检测）

检测车辆是否保持在道路范围内。

- 检查车辆中心点是否在车道内
- 检查车辆四个角点是否在车道内
- 支持 `use_road_boundary`（使用道路边界 vs 车道边界）
- 转弯车道给予额外容差

### 4. ObstacleBypass（障碍物绕行）

检测主车绕行障碍物的质量。

- 检测是否进入测试范围（多边形区域）
- 通过 X 投影判断是否正在绕行
- 检查绕行时横向距离是否 >= `min_lateral_distance`
- 检查绕行时速度是否 <= `max_speed`
- 最后一帧未绕行直接扣 100 分

### 5. TTC（碰撞时间检测）

基于 Time-To-Collision 算法检测碰撞风险。

- 计算主车与所有障碍物多边形最近点的距离
- 投影相对速度到连线方向
- TTC = 距离 / 相对接近速度
- TTC < 阈值时判定为危险

### 6. CentripetalAccelerationLimit（向心加速度）

检测转弯时向心加速度是否过大。

- 计算公式: `centripetal_acceleration = |speed * spin|`
- spin = 角速度 (rad/s)，来自 localization 的 `angular_velocity.z`
- 超限扣分

### 7. TimeLimit（超时检测）

检测场景是否在规定时间内完成。

- 对比当前时间与第一帧时间的差值
- 未超时: 得 100 分
- 超时: 得 0 分
- 得分制 (`get_deduction_score: false`)

### 8. ReachEnd（到达终点）

检测主车是否到达目标区域。

- 在终点坐标 (x, y) 构建 `length × width` 的矩形区域
- 检测主车多边形是否与目标区域重叠
- 一旦到达，后续帧持续返回通过
- 得分制: 到达得 100 分，未到达得 0 分
- 终点坐标从 `--scenario` 文件的 `autoCarInfo.end` 读取

### 9. Collision（碰撞检测）

检测主车与障碍物之间是否发生碰撞。

- 计算主车多边形与所有障碍物多边形的最短距离
- 距离 <= `distance` 阈值则判定为碰撞
- 支持方向过滤:
  - `INCLUDE_BACK`: 包含后方碰撞
  - `EXCLUDE_BACK`: 排除后方碰撞（追尾不算）
- 支持 `ignore_object_ids` 忽略特定障碍物
- 碰撞扣 100 分

### 10. RedLightStop（红灯停车距离）

检测红灯时主车是否停在停止线合理距离内。

- 只在感知到 `RED` 信号灯时检查
- 主车右转灯或应急灯时跳过
- 车辆移动时只检测是否压过停止线，不检查停车距离
- 车辆停止时检查车头到 stopline 的距离是否在 `[min_distance, max_distance]`
- 压线/越线直接扣 100 分
- 停车距离过近或过远扣 `single_deduction`
- 依赖地图中的 signal stop_line；需通过 `--map_file` 或 `--scenario` 自动加载地图

### 11. CrosswalkStop（人行横道停车距离）

检测有移动行人通过人行横道时，主车是否停在合理距离内。

- 只在检测到移动行人位于 crosswalk 上时检查
- 车辆移动时只检测是否压入人行横道，不检查停车距离
- 车辆停止时检查车头到 crosswalk polygon 的距离是否在 `[min_distance, max_distance]`
- 压入人行横道直接扣 100 分
- 停车距离过近或过远扣 `single_deduction`
- 依赖地图中的 crosswalk polygon；需通过 `--map_file` 或 `--scenario` 自动加载地图

## 数据源支持

| 模式 | 说明 |
|------|------|
| `bag` | 离线读取 Apollo Cyber record/bag 文件 |
| `bag + scene_manifest` | 单次读取一个 bag，默认按底盘 AUTO 周期独立评估多个场景 |
| `channel` | 实时订阅 Cyber 通道 |
| `demo` | 无需依赖，使用模拟数据演示 |

### 订阅的 Cyber 通道

| 通道 | 消息类型 | 提供的数据 |
|------|---------|-----------|
| `/apollo/canbus/chassis` | Chassis | 速度 |
| `/apollo/localization/pose` | LocalizationEstimate | 位置、朝向、角速度 |
| `/apollo/perception/obstacles` | PerceptionObstacles | 障碍物位置、速度、尺寸、行人 |
| `/apollo/perception/traffic_light` | TrafficLightDetection | 红绿灯状态 |

## 多场景评分

使用 `--scene_manifest` 可以让一个数据包按场景顺序评估多个场景。Apollo 默认每个场景生成一个独立 bag；manifest 可用 `bag_dir` 自动匹配 `output_data/<scene_id>/<scene_id>.output.bag`，也可在场景项中直接填写 `bag_file`。每个场景使用独立的 `GradingEngine` 和 handler，所以 A 场景失败、到达终点、累计扣分或障碍物绕行状态都不会影响 B 场景。

如果多个场景确实位于同一个 bag，程序会优先检测 replay engine 在场景切换时产生的 record timestamp 回退；检测到回退就结束当前场景并开始新的时间 epoch。没有 timestamp 回退时，默认 `boundary_mode` 为 `auto`：每次底盘进入 `COMPLETE_AUTO_DRIVE` 开始下一个场景，退出 AUTO 结束当前场景。没有 timestamp 回退、AUTO 切换或 scene marker 时，不能可靠猜测边界；这时将 `boundary_mode` 设置为 `window`，为每个场景提供 `window.start_sec` 和 `window.end_sec`。

## 配置文件格式

支持两种格式:
- **JSON** (`.json`) — 推荐，如 `metric/1.json`
- **Protobuf Text** (`.conf`) — 兼容原 C++ 配置

### JSON 配置示例

```json
{
    "metric": [
        {
            "name": "SpeedLimit",
            "description": "ADC speed limit.",
            "is_critical": true,
            "require_all_time_pass": true,
            "condition": {
                "speed_condition": {
                    "name": "speed",
                    "max_speed": 16.67,
                    "min_speed": -0.5,
                    "use_score": true,
                    "single_deduction": 2,
                    "deduction_unit": 1
                }
            }
        }
    ],
    "use_score": true
}
```

## 项目结构

```
grading_system_py/
├── main.py                              # 主入口
├── common/
│   ├── math_utils.py                    # 几何库 (Vec2d, Polygon2d, Box2d)
│   └── map_interface.py                 # 地图抽象接口 + stub
├── condition/
│   ├── condition_handler_base.py        # 抽象基类
│   ├── util.py                          # 工具类 (TTC, 多边形距离等)
│   ├── speed_handler.py                 # 速度
│   ├── acceleration_handler.py          # 加速度
│   ├── on_road_handler.py               # 在路
│   ├── obstacle_bypass_handler.py       # 障碍物绕行
│   ├── ttc_handler.py                   # TTC碰撞时间
│   ├── centripetal_acceleration_handler.py  # 向心加速度
│   ├── time_limit_handler.py            # 超时
│   ├── region_overlap_lw_handler.py     # 到达终点
│   ├── object_overlap_handler.py        # 碰撞检测
│   ├── red_light_stop_handler.py        # 红灯停车距离
│   └── crosswalk_stop_handler.py        # 人行横道停车距离
└── tests/
    ├── mock_data.py                     # 测试数据模型
    ├── test_acceleration.py
    ├── test_speed.py
    ├── test_on_road.py
    ├── test_obstacle_bypass.py
    └── test_ttc.py
```
