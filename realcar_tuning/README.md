# realcar_tuning — 实车 890 配置调参模板（2026-08-05）

> **来源**：2026-08-05 从实车 890（`apollo@192.168.10.6`，工程 `~/apollo-edu_9.1.1`，profile **HQEV_BDN**）拷贝的**生效配置**（`shiche/car_890/.extracted/profiles/HQEV_BDN/modules/`）。
> **用途**：备赛熟悉实车真实参数 + 比赛现场直接改 RECOMMENDED 版上传生效。

## 目录结构

```
realcar_tuning/
├── README.md                          # 本说明
├── 参数改动清单.md                     # 🚨 逐参数改动清单（原值/新值/注释/依据/审查结论）
├── ORIGINAL/                          # ✅ 实车原始配置（未改动，基准/对照）
│   └── modules/
│       ├── planning/                  # planning 全量配置（137 文件）
│       │   ├── planning_component/conf/planning.conf   ← 全局 gflags
│       │   ├── traffic_rules/*/conf/default_conf.pb.txt  ← 交通规则
│       │   ├── tasks/*/conf/default_conf.pb.txt          ← 任务参数
│       │   └── scenarios/*/conf/*.pb.txt                ← 场景参数
│       └── control/                   # control 全量配置（24 文件，Plus 控制器）
│           └── control_component/conf/control.conf + controllers/*/conf/
└── RECOMMENDED/                       # 🚨 推荐参数已应用（比赛直接上传）
    └── modules/planning/              # 与 ORIGINAL 同结构，仅 5 个文件已改
```

> **`ORIGINAL` 与 `RECOMMENDED` 的差异 = 本次推荐的全部改动**（`diff -rq ORIGINAL RECOMMENDED` 只有 5 个文件）：
> ① `planning_component/conf/planning.conf`
> ② `traffic_rules/crosswalk/conf/default_conf.pb.txt`
> ③ `traffic_rules/traffic_light/conf/default_conf.pb.txt`
> ④ `traffic_rules/stop_sign/conf/default_conf.pb.txt`（备选）
> ⑤ `traffic_rules/scene_speed_limit/conf/default_conf.pb.txt`

---

## 关键参数对照表（实车原始 → 推荐）

### ① `planning/planning_component/conf/planning.conf`

| 参数 | ORIGINAL（实车） | RECOMMENDED | 说明 |
|------|-----------------|-------------|------|
| `--planning_upper_speed_limit` | 5.0 | **5.0（别动）** | 评分铁律 [1.5,5.1]，越界整次判 0 |
| `--default_cruise_speed` | 5.0 | **3.0** | 评分 cruise ≤3.0556；5.0 持续超速扣分 |
| `--min_stop_distance_obstacle` | 0.0 | 现场验证 | 障碍停车距离兜底 |

### ② `traffic_rules/crosswalk/conf/default_conf.pb.txt`

| 参数 | ORIGINAL（实车） | RECOMMENDED | 说明 |
|------|-----------------|-------------|------|
| `stop_distance` | 0.10 | **1.25** | 评分停车 [1.0,1.5]m；0.1 停太近扣 20 |
| `expand_s_distance` | 0.0 | **6.0** | 行人提前识别，防急刹（急刹 4.9-6.9 双扣分） |
| `stop_timeout` | 3.0 | **30.0** | 防行人未走完提前放行 |

### ③ `traffic_rules/traffic_light/conf/default_conf.pb.txt`

| 参数 | ORIGINAL（实车） | RECOMMENDED | 说明 |
|------|-----------------|-------------|------|
| `stop_distance` | 0.10 | **1.25** | 红灯停车 [1.0,1.5]m |

### ④ `traffic_rules/stop_sign/conf/default_conf.pb.txt`（备选）

| 参数 | ORIGINAL（实车） | RECOMMENDED | 说明 |
|------|-----------------|-------------|------|
| `stop_distance` | 4.0 | **1.25** | 旧版 4 题制停止牌 [0,2]m；新版正式图无 stop_sign |

### ⑤ `traffic_rules/scene_speed_limit/conf/default_conf.pb.txt`（减速带限速核心）

| 参数 | ORIGINAL（实车） | RECOMMENDED | 说明 |
|------|-----------------|-------------|------|
| `crosswalk_enabled` | false | **true** | 覆盖 7上午/7下午 减速带区（=Crosswalk_1 位置） |
| `crosswalk_limit_speed` | 1.0 | **0.8** | 评分 ≤1.0，留 0.2 裕量 |
| `crosswalk_forward_buffer` | 1.0 | **8.0** | 提前 8m 减速 |
| `crosswalk_backward_buffer` | 1.0 | **2.0** | 区域后 2m 恢复 |
| `traffic_light_enabled` | false | **true** | 覆盖 6上午/6下午 减速带区（=Signal_1/Signal_6 位置） |
| `traffic_light_limit_speed` | 1.0 | **0.8** | 同上 |
| `traffic_light_forward_buffer` | 1.0 | **8.0** | 同上 |
| `traffic_light_backward_buffer` | 1.0 | **2.0** | 同上 |
| `speed_bump_enabled` | false | false | 地图无 speed_bump，开了无效 |
| `junction_enabled` | false | false | 保持关闭 |

---

## 比赛现场怎么用（30 分钟）

```bash
# 1. 连接 + 备份
bash shiche/car_connect/connect_car.sh && bash shiche/car_connect/backup_car.sh

# 2. 上传 RECOMMENDED 的 5 个文件到实车对应路径（对照表）
#    实车生效路径 = ~/apollo-edu_9.1.1/profiles/HQEV_BDN/modules/planning/...
#    例：
scp realcar_tuning/RECOMMENDED/modules/planning/planning_component/conf/planning.conf \
    car:~/apollo-edu_9.1.1/profiles/HQEV_BDN/modules/planning/planning_component/conf/
#    （crosswalk / traffic_light / stop_sign / scene_speed_limit 同理）

# 3. 重启 Planning 模块（Dreamview+ 模块栏关开），不编译
# 4. 验证生效
ssh car "cat /apollo/modules/planning/planning_component/conf/planning.conf | grep default_cruise_speed"
```

---

## ⚠️ 与工作区其他配置的区别（避免混淆！）

| 位置 | 是什么 | 值 |
|------|--------|-----|
| **`realcar_tuning/ORIGINAL/`** | ✅ **实车 890 真实生效配置**（本次基准） | crosswalk 0.10 / traffic_light 0.10 / stop_sign 4.0 / cruise 5.0 |
| `profiles/HQEV_BDN/`（工作区） | 实车配置的本地精简副本（仅 planning/common，缺 control 等） | 与实车 planning 一致 |
| `modules/planning/traffic_rules/*/conf/`（工作区） | ⚠️ **官方源码默认值**（buildtool 拉取，非实车） | crosswalk 1.0 / traffic_light 1.0 / stop_sign 1.0 |

> 🚨 **比赛只认 `realcar_tuning/RECOMMENDED/`（基于实车 ORIGINAL 修改）**，不要参照 `modules/planning/traffic_rules/` 下的源码默认值！

---

## 评分依据（为什么这么配）

- 评分铁律：`default_cruise_speed` / `planning_upper_speed_limit` ∈ [1.5,5.1]，越界整次判 0（presets.json）
- cruise：≤3.0556 m/s（11km/h），加速度 [-3,2]
- 停车题（crosswalk/red_light）：车头距停止线 [1.0,1.5]m；停近扣 20，压线硬失败，停远可补救
- 减速带区域：≤1.0 m/s（区域=第 1 个信号灯/人行道位置，地图无 speed_bump）
- 停车标定 1:1（仿真 stop_distance=1.75 → 车头实际停 1.78m）
- 详细推导见 `.claude/skills/real-car-competition/references/15-final-tuning-guide.md`
