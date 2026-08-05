# 使用方法

## 快速开始

### 1. Demo 模式（无需任何外部依赖）

```bash
PYTHONIOENCODING=utf-8 python3 /apollo_workspace/grading_system_py/main.py \
  --config /apollo_workspace/metric/1.json \
  --mode demo
```

### 2. 读取 Bag 文件（离线评分）

```bash
python3 /apollo_workspace/grading_system_py/main.py \
  --config /apollo_workspace/metric/1.json \
  --scenario /apollo-simulator/input_data/1.json \
  --map_file /path/to/base_map.bin \
  --mode bag \
  --bag_file /apollo-simulator/output_data/1/1.output.bag
```

### 3. 一个 Bag 评估多个场景

A 场景失败不会影响 B 场景。每个场景都会创建独立的评分引擎、metric handler、历史帧和累计分数。默认不需要填写时间戳，只按 manifest 中的场景顺序绑定底盘 AUTO 周期：进入 `COMPLETE_AUTO_DRIVE` 开始一个场景，退出 AUTO 结束这个场景。

```json
{
  "boundary_mode": "auto",
  "scenes": [
    {
      "id": "scene_a",
      "config": "/apollo_workspace/metric/1.json",
      "scenario": "/apollo-simulator/input_data/1.json",
      "map_file": "/path/to/base_map.bin"
    },
    {
      "id": "scene_b",
      "config": "/apollo_workspace/metric/2.json",
      "scenario": "/apollo-simulator/input_data/2.json"
    }
  ]
}
```

运行：

```bash
python3 /apollo_workspace/grading_system_py/main.py \\
  --mode bag \\
  --bag_file /apollo-simulator/output_data/all.output.bag \\
  --scene_manifest /apollo_workspace/metric/scenes.json \\
  --interval 0.1
```

如果是 Apollo 默认的每场景独立 bag，manifest 包含 `bag_dir` 后可以省略 `--bag_file`：

```bash
python3 /apollo_workspace/grading_system_py/main.py \\
  --mode bag \\
  --scene_manifest /apollo_workspace/metric/scenes.json
```

如果 A/B 是仿真器默认生成的独立 bag，推荐直接配置 `bag_dir`，程序会自动匹配 `output_data/<scene_id>/<scene_id>.output.bag`，完全不需要时间戳：

```json
{
  "bag_dir": "/apollo-simulator/output_data",
  "scenes": [
    {"id": "1", "config": "/apollo_workspace/metric/1.json",
     "scenario": "/apollo-simulator/input_data/1.json"},
    {"id": "2", "config": "/apollo_workspace/metric/2.json",
     "scenario": "/apollo-simulator/input_data/2.json"}
  ]
}
```

如果确实把多个场景拼在同一个 bag 中，程序会优先检测 replay engine 在场景切换时产生的 record timestamp 回退；检测到回退就结束当前场景并开始新的时间 epoch。没有 timestamp 回退时，再按底盘 AUTO 周期顺序绑定场景：仿真器需要在 A/B 场景切换之间让底盘退出 AUTO，再在 B 场景开始时重新进入 AUTO。没有这两类边界事件时程序不会猜测 A/B 边界。

如果 bag 没有可用的场景边界事件，可使用显式时间窗口：

```json
{
  "boundary_mode": "window",
  "scenes": [
    {"id": "scene_a", "config": "a.json",
     "window": {"start_sec": 0, "end_sec": 35}},
    {"id": "scene_b", "config": "b.json",
     "window": {"start_sec": 40, "end_sec": 78}}
  ]
}
```

路径可以写成相对于 manifest 文件的路径。`--scene_manifest` 不能与单场景的 `--config`、`--scenario`、`--map_file` 同时使用。

### 4. 订阅 Cyber 通道（实时评分）

默认行为：程序启动后会先监听底盘 `/apollo/canbus/chassis`，只有当 `driving_mode == COMPLETE_AUTO_DRIVE` 时才开始测评；退出 AUTO 后暂停测评。

```bash
python3 /apollo_workspace/grading_system_py/main.py \
  --config /apollo_workspace/metric/1.json \
  --scenario /apollo-simulator/input_data/1.json \
  --map_file /path/to/base_map.bin \
  --mode channel
```

如果想启动后立刻测评（不等 auto）：

```bash
python3 /apollo_workspace/grading_system_py/main.py \
  --config /apollo_workspace/metric/1.json \
  --scenario /apollo-simulator/input_data/1.json \
  --map_file /path/to/base_map.bin \
  --mode channel \
  --no_start_on_auto
```

如果退出 auto 后也继续测评：

```bash
python3 /apollo_workspace/grading_system_py/main.py \
  --config /apollo_workspace/metric/1.json \
  --scenario /apollo-simulator/input_data/1.json \
  --map_file /path/to/base_map.bin \
  --mode channel \
  --no_pause_when_manual
```

## 命令行参数

| 参数 | 必填 | 默认值 | 说明 |
|------|------|--------|------|
| `--config` | 单场景必填 | - | 单场景评分配置文件路径 (.json 或 .conf) |
| `--mode` | 否 | `demo` | 数据源模式: `bag` / `channel` / `demo` |
| `--bag_file` | 单 bag 模式必填 | - | bag/record 文件路径；如果 manifest 为每个场景配置了 `bag_file` 或 `bag_dir`，可省略 |
| `--scene_manifest` | 多场景 bag 模式可选 | - | 按顺序配置场景；默认按 AUTO 周期切分，也支持 `boundary_mode: window` |
| `--scenario` | 否 | - | 单场景 JSON 文件（提供 ReachEnd 终点坐标，并尝试自动查找地图） |
| `--map_file` | 否 | - | Apollo base_map/sim_map 文件，用于 CrosswalkStop/RedLightStop |
| `--interval` | 否 | `0.1` | 评分帧间隔（秒） |
| `--start_on_auto` | 否 | true | channel模式：等底盘进入 `COMPLETE_AUTO_DRIVE` 后开始测评 |
| `--no_start_on_auto` | 否 | - | channel模式：不等auto，启动后立即测评 |
| `--pause_when_manual` | 否 | true | channel模式：退出auto后暂停测评 |
| `--no_pause_when_manual` | 否 | - | channel模式：退出auto后继续测评 |

## 地图文件说明

`CrosswalkStop` 和 `RedLightStop` 依赖地图中的人行横道 polygon 和红绿灯 stop_line。

推荐显式传入地图文件：

```bash
--map_file /opt/apollo/neo/share/modules/map/data/xxx/base_map.bin
```

如果未传 `--map_file`，程序会尝试从 `--scenario` 的 `scenario.roadNetwork.logicFile.filepath` 自动查找：

- `/opt/apollo/neo/share/<filepath>/base_map.bin`
- `/opt/apollo/neo/share/<filepath>/base_map.txt`
- `/opt/apollo/neo/src/<filepath>/base_map.bin`
- `/apollo_workspace/<filepath>/base_map.bin`

如果地图文件找不到，这两个 metric 会像 C++ 地图查询失败时一样跳过地图依赖检查。

## 输出说明

### 实时输出

运行过程中，每帧违规都会实时打印:

```
[FAIL] t=12.70s CentripetalAccelerationLimit: CentripetalAcceleration is 6.877.
[FAIL] t=3.50s SpeedLimit: Max speed is 16.67, ego speed is 18.0.
```

### 汇总输出

运行结束后输出评分汇总表:

```
============================================================
  GRADING SUMMARY
============================================================

  Scenario: FAIL

  Metric                    Result   Score    Fail/Total
  ------------------------- -------- -------- ------------
  CentripetalAccelerationLimit FAIL     88       4/313
  AccelerationLimit         PASS     100      0/313
  TimeLimit                 PASS     100      0/313
  ReachEnd                  FAIL     0        313/313
  Collision                 PASS     100      0/313
  OnRoad                    PASS     100      0/313
  SpeedLimit                PASS     100      0/313
```

- **Result**: PASS/FAIL
- **Score**: 当前得分（满分100，扣分制从100开始扣，得分制从0开始加）
- **Fail/Total**: 失败帧数 / 总帧数

## 使用场景示例

### 批量评分多个场景

```bash
for i in 1 2 3 4 5; do
  echo "=== Scenario $i ==="
  python3 /apollo_workspace/grading_system_py/main.py \
    --config /apollo_workspace/metric/$i.json \
    --scenario /apollo-simulator/input_data/$i.json \
    --mode bag \
    --bag_file /apollo-simulator/output_data/$i/$i.output.bag
done
```

### 在 Python 脚本中调用

```python
import sys
sys.path.insert(0, '/apollo_workspace')

from collections import deque
from grading_system_py.condition.speed_handler import SpeedConditionHandler
from grading_system_py.condition.ttc_handler import TtcConditionHandler
from grading_system_py.main import (
    GradingWorld, Object, DetailedResult, GradingEngine,
    parse_config_file, _fill_endpoint_from_scenario
)

# 方式1: 使用完整引擎
config = parse_config_file('/apollo_workspace/metric/1.json')
engine = GradingEngine(config)

# 喂入每帧数据
world = GradingWorld()
world.timestamp_sec = 1.0
world.auto_driving_car.speed = 15.0
world.auto_driving_car.position_x = 100.0
world.auto_driving_car.position_y = 0.0
# ...

results = engine.feed_frame(world)
summary = engine.get_summary()

# 方式2: 单独使用某个 handler
handler = TtcConditionHandler()
# ... (参见 example_usage.py)
```

### 调整评分帧率

```bash
# 默认 0.1s 一帧 (10 Hz)
python3 main.py --config ... --mode bag --bag_file ... --interval 0.1

# 更高精度: 50ms 一帧 (20 Hz)
python3 main.py --config ... --mode bag --bag_file ... --interval 0.05

# 更快处理: 1s 一帧 (1 Hz)
python3 main.py --config ... --mode bag --bag_file ... --interval 1.0
```

## 环境要求

| 模式 | Python | 依赖 |
|------|--------|------|
| demo | >= 3.6 | 无 |
| bag | >= 3.6 | Apollo Cyber Python (`/opt/apollo/neo/python/`) |
| channel | >= 3.6 | Apollo Cyber Python + Cyber 运行时 |

如果 import 报错，确保:

```bash
# Apollo 环境下通常已自动配置
export PYTHONPATH=/opt/apollo/neo/python:$PYTHONPATH
```
