# 09 Control 模块（官方发布包，可改代码/可调参）

> **来源**：`release_control.tar.gz`（官方 control 发布包，9.1.2 amd64）已解压到本地 `modules/control/`。
> 🚨 **铁律：所有情况以实车为准！** 本地 control 是 9.1.2 发布包，实车版本可能不同（字段/默认值/插件）——现场以 `cat /apollo/modules/control/...` 实车实际内容为准。
> **官方确认**：实车赛允许改 `control` 代码（不只 planning）。但 30 分钟很宝贵，**优先调参**，只有纯参数无法实现时才动 control 源码 + 编译。

---

## 1. Control 模块是什么

Control（控制）模块**接收 Planning 的轨迹**，通过横向（LQR）+ 纵向（PID）控制器输出方向盘/油门/刹车指令，驱动底盘。

**与赛题的关系**：
- 停车距离的最终落地精度、超调大小由 control 决定（planning 只规划，control 执行）
- 限速类赛题（1/7/8）的**速度跟踪质量**由纵向 PID 决定
- 起步加速超冲、停车冲过头等实车问题，**既可能来自 planning 也可能来自 control**

⚠️ **调参优先级仍以 planning 为主**（赛题机制全部在 planning）；control 只在遇到"规划对但执行偏差大"时才介入（如停车超调、速度振荡）。

---

## 2. 本地代码位置（已解压）

```
modules/control/
├── control_component/              # 主模块（组件、pipeline、子模块）
│   ├── conf/
│   │   ├── control.conf            # ⭐ 全局 gflags（等价 planning.conf）
│   │   ├── calibration_table.pb.txt # 标定表（油门/刹车查找表，勿乱动）
│   │   └── pipeline.pb.txt         # 控制器流水线（LAT_CONTROLLER + LON_CONTROLLER）
│   ├── common/control_gflags.h/.cc # 全部 gflag 声明/定义
│   ├── control_component.cc/.h     # 组件主入口（Init/OnTimer/OnPlanning）
│   ├── submodules/                 # 可选子模块模式（use_control_submodules=false 时不用）
│   ├── launch/control.launch       # ⭐ 启动文件（重启 control 用这个）
│   ├── dag/control.dag             # DAG 配置
│   ├── controller_task_base/       # 控制器任务基类（PID/LQR/MPC 通用）
│   └── proto/                      # 控制相关 proto
├── controllers/
│   ├── lat_based_lqr_controller/   # 横向 LQR 控制器插件
│   │   └── conf/controller_conf.pb.txt   # ⭐ 横向调参
│   └── lon_based_pid_controller/   # 纵向 PID 控制器插件
│       └── conf/controller_conf.pb.txt   # ⭐ 纵向调参
└── control_task_base_extend/       # 任务扩展（control_checker 等）
```

---

## 3. 关键配置

### 3.1 `control.conf`（全局 gflags，等价 planning.conf）

```bash
--pipeline_file=/apollo/modules/control/control_component/conf/pipeline.pb.txt
--calibration_table_file=/apollo/modules/control/control_component/conf/calibration_table.pb.txt
--control_period=0.01
--enable_gain_scheduler=true
--set_steer_limit=true
--use_control_submodules=false
--max_acceleration_when_stopped=0.01
--soft_estop_brake=15.0
...
```

实车调参重点关注：
| gflag | 作用 | 备注 |
|-------|------|------|
| `max_acceleration_when_stopped` | 判定"已停止"的加速度阈值 | 停车判定相关 |
| `soft_estop_brake` | 软急停制动减速度 | 15 默认 |
| `control_period` | 控制周期 | 0.01s 默认 |
| `use_control_submodules` | 是否用子模块模式 | false=单进程 |

### 3.2 `pipeline.pb.txt`（控制器流水线）

```text
controller {
  name: "LAT_CONTROLLER"
  type: "LatController"
}
controller {
  name: "LON_CONTROLLER"
  type: "LonController"
}
```

### 3.3 横向 LQR `controller_conf.pb.txt`（`controllers/lat_based_lqr_controller/conf/`）

关键参数（**实车调横向用**）：
```text
ts: 0.01                       # 控制周期
preview_window: 0              # 预瞄窗口
matrix_q: 0.05 / 1.0 ...       # Q 矩阵（横向误差权重，越大越"硬"）
lookahead_station: 1.4224      # 前视距离（低速）
lookback_station: 2.8448       # 后视距离
switch_speed: 2.0              # 高低速切换速度
max_lateral_acceleration: 5.0  # 最大横向加速度
lat_err_gain_scheduler { ... } # 横向误差增益随速度调度（低速 1.0 → 高速 0.05）
heading_err_gain_scheduler { ... }
```

### 3.4 纵向 PID `controller_conf.pb.txt`（`controllers/lon_based_pid_controller/conf/`）

关键参数（**实车调纵向/速度用**）：
```text
ts: 0.01
brake_minimum_action: 0.0      # 最小制动指令
throttle_minimum_action: 0.0   # 最小油门指令
speed_controller_input_limit: 1.5
station_error_limit: 2.0
standstill_acceleration: -0.3  # 停车时的目标加速度
full_stop_long_time: 3.0       # 长时间停车判定
stop_gain_acceleration: -1.0   # 停车增益加速度
full_stop_path_remain_gain: 0.3
speed_itfc_full_stop_speed: 0.09
speed_itfc_dcc_emergency: -1.5 # 紧急制动减速度
speed_itfc_speed_cmd: 0.10
speed_itfc_path_remain_max: 0.6
# PID 三档
station_pid_conf { kp: 0.2 ki: 0.0 kd: 0.0 }      # 位置 PID（站距）
low_speed_pid_conf { kp: 2.0 ki: 0.3 kd: 0.0 }    # 低速 PID（<3 m/s）
high_speed_pid_conf { kp: 1.0 ki: 0.3 kd: 0.0 }   # 高速 PID（≥3 m/s）
switch_speed: 3.0              # 高低速 PID 切换速度
```

⚠️ **低速 PID（low_speed_pid_conf）是停车精度核心**：实车低速段停车靠它，kp 太大会振荡、太小会停不准。一般不动，除非停车严重超调/不足。

---

## 4. 生效方式（只改 control 参数）

官方确认：**只改参数 → 改哪个模块就重启哪个模块，不用编译、不用 use default！**

```bash
# 改 control 配置后重启 Control 模块：
# 方式一（Dreamview+ 模块栏）：Control 开关关掉再打开
# 方式二（命令行）：
cyber_launch stop /apollo/modules/control/control_component/launch/control.launch
cyber_launch start /apollo/modules/control/control_component/launch/control.launch
# 方式三（工具包）：
bash shiche/car_connect/apply_config.sh control
```

⚠️ 改 control 源码（.cc/.h）则需要 `buildtool build -p modules/control/ -j15` + `aem profile use default` + 重启。

---

## 5. 什么时候才动 control（决策树）

```
遇到实车行为问题
  ├─ 停车距离/限速/跟车逻辑不对 → 先查 PLANNING（traffic rule / gflag / speed_decider）✅ 主战场
  ├─ 规划明明正确但车"执行"偏差大：
  │    ├─ 停车冲过头/停不到位 → 纵向 PID / standstill / stop_gain
  │    ├─ 速度振荡、跟车顿挫 → 纵向 PID kp/ki、加速度限制
  │    ├─ 跑偏、转向晃动 → 横向 LQR matrix_q / gain_scheduler
  │    └─ 起步超冲 → 纵向 PID + planning_upper_speed_limit 配合
  └─ 以上纯参数调不好 → 才考虑改 control 源码（最后手段）
```

**实车调参铁律**：先诊断（看是 planning 决策错还是 control 执行错）→ 再决定动哪个模块。诊断看日志：
- `data/log/planning.INFO`：planning 决策（STOP/限速）
- `data/log/control.INFO`：control 执行（目标速度 vs 实际速度、指令）

---

## 6. 与实车赛 9 赛题的关联

| 赛题 | 主要机制（planning） | control 可辅助的点 |
|------|---------------------|-------------------|
| 0 定速巡航 | default_cruise_speed | 低速 PID 平稳性 |
| 1 减速带 | speed_bump 限速 | 减速平稳不超调 |
| 2 人行道停车 | crosswalk stop_distance | **停车精度**（stop_gain / low_speed_pid） |
| 3 红绿灯停车 | traffic_light stop_distance | **停车精度** |
| 4 停止牌停车 | stop_sign stop_distance | **停车精度** |
| 5 障碍物避让 | speed_decider | 停车精度 |
| 6 动态跟随 | follow 任务 | 纵向 PID 跟车平稳 |
| 7 路口减速 | 插件 crosswalk_limit | 减速平稳 |
| 8 交通灯减速 | 插件 traffic_light_limit | 减速平稳 |

→ **99% 场景 control 不需要动**；只有实车停车精度/平顺性问题突出时才调纵向 PID。⚠️ 停车距离主要看 `stop_distance`（planning）+ 停车偏移（control），两者配合标定。

---

## 7. 快速命令速查

```bash
# 本地查看 control 配置
cat modules/control/control_component/conf/control.conf
cat modules/control/controllers/lon_based_pid_controller/conf/controller_conf.pb.txt
cat modules/control/controllers/lat_based_lqr_controller/conf/controller_conf.pb.txt

# 实车查看（挂载后）
ssh car "aem exec -- cat /apollo/modules/control/control_component/conf/control.conf"
ssh car "aem exec -- cat /apollo/modules/control/controllers/lon_based_pid_controller/conf/controller_conf.pb.txt"

# 实车重启 control（只改参数）
bash shiche/car_connect/apply_config.sh control

# 实车看 control 日志
ssh car "tail -f ~/apollo-edu-9.1.1/data/log/control.INFO"
```
