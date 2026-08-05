# 场景注入 (Scenario Injection)

把仿真场景里的障碍物 / 交通灯注入到**真实感知输出**里，让下游 prediction / planning
把它们当成真实障碍物处理。用于在 `vehicle_test`（纯激光）模式下做场景回灌测试。

## 原理

在 `vehicle_test` 模式下，`/apollo/perception/obstacles` 的最终发布者是 `msg_adapter`
（`multi_sensor_fusion` 不运行）。改造后的 `msg_adapter` 会：

1. 订阅注入话题 `/apollo/perception/scenario_obstacles`，缓存最近一帧；
2. 在发布 `/apollo/perception/obstacles` 前，把缓存的场景障碍物合并进去；
3. 若注入器没在跑（缓存超时），则原样透传真实感知，零影响。

```
lidar_tracking ─▶ msg_adapter ──(合并)──▶ /apollo/perception/obstacles ─▶ prediction ─▶ planning
                       ▲
   scenario_injector.py (sim_obstacle) ─▶ /apollo/perception/scenario_obstacles
```

融合逻辑在 C++ 里（`modules/perception/msg_adapter/`），所以本目录的
`configure_perception_side_channel.py` / `perception_fusion.py` / `run_fused_scenario.py`
（旧的侧通道 + 独立融合进程方案）**已不再需要**。现在只用 `scenario_injector.py`。

## 一次性前提

1. 已编译安装改造后的 `msg_adapter`：
   ```bash
   buildtool build -p modules/perception/msg_adapter
   ```
2. HQEV profile 的 flag 已开启（`profiles/HQEV/modules/perception/data/flag/perception_common.flag`）：
   ```
   --enable_scenario_injection=true
   --scenario_obstacle_topic=/apollo/perception/scenario_obstacles
   --scenario_injection_timeout_sec=0.5
   --scenario_injection_id_offset=100000
   ```
3. 感知（LidarPerception）需**重启一次**以加载新 `.so` 和新 flag。之后每次注入无需再重启。

## 用法

### 方式一：命令行直接启动（推荐调试）

前台运行，能看到 `sim_obstacle` 输出，`Ctrl+C` 停止（会自动清理子进程和临时 flag 文件）：

```bash
python3 /apollo_workspace/scenario_injection/scenario_injector.py \
    /apollo_workspace/scenario_injection/1.json
```

后台运行：

```bash
nohup python3 /apollo_workspace/scenario_injection/scenario_injector.py \
    /apollo_workspace/scenario_injection/1.json \
    > /apollo/data/log/scenario_injection.log 2>&1 &
```

停止后台进程：

```bash
pkill -f scenario_injector.py
```

### 方式二：Dreamview 启停

`vehicle_test` 模式里已注册 `ScenarioInjection` 模块，直接在 Dreamview 界面点启停即可，
效果等同于方式一的后台命令。

## 换场景

把命令里的 `1.json` 换成你自己的场景文件路径即可。场景 JSON 需包含 `scenario.mapDir`
（sim_obstacle 加载地图用）。

## 常用参数

`scenario_injector.py` 常用可选项：

| 参数 | 说明 | 默认 |
|------|------|------|
| `scenario_json` | 场景 JSON 路径（位置参数） | 必填 |
| `--perception-topic` | 注入障碍物话题，需与 flag `scenario_obstacle_topic` 一致 | `/apollo/perception/scenario_obstacles` |
| `--map-dir` | 显式指定地图目录，否则用 `map-dir-prefix + scenario.mapDir` | 无 |
| `--no-trigger-planning-command` | 不自动发 PlanningCommand 触发 | 默认会触发 |
| `--publish-command` | 让 sim_obstacle 用场景终点发 LaneFollowCommand | 关 |
| `--flag KEY=VALUE` | 透传给 sim_obstacle 的额外 gflag，可重复 | 无 |

## 验证是否生效

另开终端看合并后的输出里有没有带 offset（id ≥ 100000）的障碍物：

```bash
cyber_monitor -c /apollo/perception/obstacles
```

或看注入原始话题是否在出数据：

```bash
cyber_monitor -c /apollo/perception/scenario_obstacles
```

## 相关 gflag（在 perception_common.flag 里调）

| flag | 说明 | 默认 |
|------|------|------|
| `enable_scenario_injection` | 总开关；`false` 时 msg_adapter 完全是原版行为 | `false` |
| `scenario_obstacle_topic` | 注入障碍物订阅话题 | `/apollo/perception/scenario_obstacles` |
| `scenario_injection_timeout_sec` | 超过此时间没收到注入帧就不合并（真实感知透传） | `0.5` |
| `scenario_injection_id_offset` | 注入障碍物 id 偏移，避免与真实 id 撞号；`<0` 则保持原 id | `100000` |
