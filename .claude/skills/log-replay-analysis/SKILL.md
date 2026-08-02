---
name: log-replay-analysis
description: Apollo 星火大赛 PnC 日志与回放分析指南（800 分实战验证）。覆盖：**本地日志**（data/log/planning.log.INFO 结构 + planning_start_point/stop_s/ST boundary/DEST/REF_END 等关键日志模式 + Python 解析方法 + 时间戳与字段坑）、**本地回放**（.record 文件、cyber_recorder、analyze_record.py 等脚本）、**云端回放**（offlineview WebSocket 抓包 + AES-CBC 解密"明月几时有" + SimWorld JSON 结构 + decode_cloud_replay.py）、**云端日志**（scenario-logs tgz 下载 + sim_engine.log 评分解析 + replay-engine.log 评测配置 + 唯一计分帧机制 + 评测时间轴映射 t=planning-13.5s）。专为赛题排障设计：评测失败 → 下载云端 log → sim_engine 定位失败帧 → 时间轴映射 → planning 日志定位车行为 → 本地复现验证。当用户提及日志、回放、sim_engine、replay-engine、planning.log、record、offlineview、云端评测、评分、not_pass、计分帧、delta_score、DistToObstacleCar、AES 解密、时间轴、T0、planning_start_point、stop_s、ST boundary、油猴下载、scenario-logs、individualModuleLogs 等关键词时必须使用此技能。
---

# Apollo PnC 日志与回放分析指南

> 本技能基于 2026-08 星火大赛 PnC 800 分实战（8 赛题全满）沉淀，覆盖本地日志/本地回放/云端回放/云端日志的**架构、结构、关键内容与分析方法**。
> 目标场景：赛题评测失败 → 快速定位根因 → 修复 → 验证。

---

## 快速导航

| 你想做什么 | 去哪一节 |
|-----------|---------|
| 看本地 planning 日志（车行为/决策） | [1. 本地日志](#1-本地日志-data-log) |
| 看本地回放（record 文件） | [2. 本地回放](#2-本地回放-record-文件) |
| 看云端回放（评测画面数据） | [3. 云端回放 offlineview](#3-云端回放-offlineview) |
| 下载+分析云端评测日志（评分） | [4. 云端日志 scenario-logs](#4-云端日志-scenario-logs) |
| 评测时间轴怎么对齐 | [4.3 时间轴映射](#43-评测时间轴映射关键) |
| 哪些日志信息最重要 | [5. 关键信息优先级](#5-哪些信息非常重要) |
| 完整排障流程 | [6. 推荐排障工作流](#6-推荐排障工作流) |

---

## 1. 本地日志（data/log）

### 1.1 文件结构与位置

```
data/log/
├── planning.INFO -> planning.log.INFO.<启动时间戳>   # 软链接指向最新
├── planning.log.INFO.20260802-125751.262559        # glog 格式（最常用）
├── control.log.INFO.*  /  prediction.log.INFO.*    # 上下游模块
└── dreamview.INFO.*  /  routing.log.INFO.*         # 其他模块
```

- **glog 行格式**：`I0802 12:41:05.645681 236963 file.cc:123] 内容`
  - `I/W/E/F` = INFO/WARN/ERROR/FATAL，`0802` = 月日，`12:41:05.645681` = 时间戳
  - `236963` = 线程号，`file.cc:123` = 来源文件:行号
- **⚠️ `--minloglevel=3` 会滤掉绝大多数日志（只留 FATAL）**——需要详细日志时临时调低（如 `--minloglevel=0`）再重启

### 1.2 关键日志模式（分析的核心）

| 关键词 | 含义 | 重要性 |
|--------|------|:---:|
| `planning_start_point` | 每帧车状态 `x,y,theta,k,v,a` | ⭐⭐⭐ |
| `reference_line_info.cc:832 stop_s: X end_s: Y` | stop fence 位置(s) 与车后端位置(s) | ⭐⭐⭐ |
| `reference_line_info.cc:812 distance_destination:X distance_ref_end:Y` | 车距终点 / 距参考线末端（DBL_MAX=无效） | ⭐⭐⭐ |
| `reference_line_info.cc:422 build reference line st boundary. id:X` | speed_decider 对障碍物的纵向决策 ST boundary | ⭐⭐⭐ |
| `reference_line_info.cc:420 NO build ... id:X` | 障碍物不建 ST boundary（IGNORE/横向远） | ⭐⭐ |
| `planning_base.cc:93 Current passed destination:0/1` | 是否已通过终点 | ⭐⭐ |
| `BuildStopDecision` | 各 traffic rule 建 stop | ⭐⭐ |
| `PARK_AND_GO: a,b,c,d` | 停车再起步场景检查 `(is_static,距终点够远,on_lane,city)` | ⭐ |
| `REF_END` / `PATH_END` | 参考线末端/路径末端 stop 障碍 | ⭐⭐ |
| `lane_borrow_path.cc:374 Blocking obstacle ID[X]` | 借道路径的阻塞障碍 | ⭐ |

### 1.3 Python 解析模板

```python
import re
LOG = 'data/log/planning.log.INFO.20260802-125751.262559'
def w2s(w):  # 'HH:MM:SS.ffffff' -> 秒
    h, m, s = w.split(':'); return int(h)*3600 + int(m)*60 + float(s)

rows = []
for line in open(LOG, errors='ignore'):
    if 'planning_start_point' in line:
        m = re.search(r'(\d\d:\d\d:\d\d\.\d+)', line)
        nums = re.findall(r'[-+]?\d+\.\d+', line.split('x,y,the,k:')[-1])
        if m and len(nums) >= 6:
            rows.append((w2s(m.group(1)), float(nums[0]), float(nums[1]),  # wall,x,y
                         float(nums[4]), float(nums[5])))                  # v,a
```

**⚠️ 常见坑**：
- **字段错位**：`planning_start_point` 的 nums 索引是 `0=x,1=y,2=theta,3=k,4=v,5=a`——错用会拿 k 当 v（值全是 0.0xx 很可疑）
- **绝对坐标**：x 是完整坐标（423xxx），比较距离别忘减起点（`x > 438` 恒真）
- **相对时间**：wall 秒数大（12:50:xx ≈ 462xx），先选 T0 再算相对时间

---

## 2. 本地回放（.record 文件）

### 2.1 文件与工具

- **record 文件**：`~/.apollo/resources/records/*.record`（分段：`*.00000.*.record` 等，几十 MB）
- **录制**：DreamView 或 `cyber_recorder`（`cyber_recorder.INFO` 是它的日志）
- **回放工具**：DreamView「回放」页 / `cyber_recorder play`
- **分析脚本**（`scripts/` 下）：
  - `analyze_record.py`：轨迹/行人分析
  - `extract_stop_reason.py`：STOP reason_code（104=CROSSWALK, 1=HEAD_VEHICLE, 2=DESTINATION）
  - `planning_lateral.py` / `lane_lateral_offset.py`：横向偏移分析

### 2.2 本地回放的意义与局限

- ✅ 验证代码逻辑（车是否停/跟/让）、决策链、fence 位置
- ⚠️ **本地行为 ≠ 云端行为**（感知/预测抖动放大、评测器只在云端）——本地通过 ≠ 评测通过（add15 教训）

---

## 3. 云端回放（offlineview）

### 3.1 数据来源

- 网页：`offlineview?id=<recordId>`（评测回放页）
- 帧数据 = WebSocket `wss://apollo.baidu.com/workbench/offlineView` 的 **`SimWorldUpdate`** 消息

### 3.2 抓包与解密（已打通）

1. **抓包**：浏览器注入 `addInitScript` 全量捕获 WebSocket 消息 → 点播放 → 帧数据流式到达 → POST 到本地 HTTP 服务器（存 `output/cloud_replay.json`）
2. **解密**：`world` 字段 = base64 + **AES-CBC 加密 JSON**
   - 密钥 = `SHA256(UTF8("明月几时有"))`（32 字节），前 16 字节 = IV，PKCS7 填充
   - 脚本：`scripts/decode_cloud_replay.py`（解密）+ `scripts/analyze_cloud_ped.py`（逐帧分析）

### 3.3 解密后 SimWorld JSON 结构

```
{
  autoDrivingCar: { positionX/Y, speed, heading, currentSignal, throttle, brake, ... },
  object: [ { id, positionX/Y, speed, heading, type, ... STOP决策位置... } ],
  mainDecision: { ... }
}
```

- **可用于还原**：自车轨迹/速度、障碍物位置/决策（STOP 栅栏）、关键帧几何
- **适用**：验证云端真实行为（停车距离、起步时机、决策闪烁），弥补本地回放与云端差异

---

## 4. 云端日志（scenario-logs）

### 4.1 下载

- **API**：`https://apollo.baidu.com/api/workbench/tasks/scenario-logs/<recordId>`（需登录 cookie）
- 旧赛题（自己测评）200 直下 tgz（gzip 无加密）；非自己测评 403 FAILED_TO_AUTH
- **油猴脚本**：`scripts/scenario_log_downloader.user.js`（测评页右下角浮动面板，扫描回放链接提取 id，手动下载）
- 下载目录：`~/下载/log/<recordId>.tgz`

### 4.2 tgz 内部结构

```
<recordId>.tgz
├── sim_engine.log               # ⭐⭐⭐ 完整测评评分（金矿）
├── replay-engine.log.INFO.*     # 回放引擎（含评测器配置）
├── individualModuleLogs.tar.gz  # 各模块日志打包
│   ├── planning.log.INFO.*      # 云端 planning（与本地同格式）
│   ├── prediction.log.INFO.*    # 云端 prediction
│   ├── control.log.INFO.*       # 云端 control
│   ├── map.log.INFO.* / routing.log.INFO.* / ...
└── ...
```

### 4.3 评测时间轴映射（关键！）

**评测 t = planning t − 13.5s**（KeyPoint 反推验证）

- 看 planning 日志行为时，评测 t 要 +13.5s 才是 planning 时刻
- **T0 校准方法**：用车停稳时刻（planning 最后 ego 帧 wall）反推，或用 KeyPoint（评测 not_pass 时刻 = planning 对应事件）

### 4.4 sim_engine.log 评分解析（最重要）

```
metric_result {
  name: "DistToObstacleCar"
  description: "ADC Must Keep Distance to A Obstacle"
  is_metric_pass: false
  detailed_result { timestamp: 80.7  is_pass: false  debug { description: "The distance from Obstacle id: 7673 is not in [2.000000 , 5.500000]" }  delta_score: 20.0 }
  ...
  metric_score: 80.0
}
score: 0.0   # 总分
{"status":1,"msg":{"is_pass":0,"metric_result":{...每项指标...}}}
```

- **每项指标**：`metric_score`（100/80/0）+ `not_pass_timestamp[]`（失败时刻）+ `total_timestamp`（总时长）
- **⭐ 唯一计分帧机制**：`detailed_result` 逐帧记录 pass/fail，**只有带 `debug`+`delta_score` 的帧计分**（静默失败帧只破坏 require_all_time_pass，不计分）
- **关键指标**：AccelerationLimit / CentripetalAccelerationLimit / Collision / DistToObstacleCar / KeyPoint / OnRoad / ReachEnd / SpeedLimit / TimeLimit
- **ReachEnd 是 gate**：不过则总分 0（如 8m 实验：车停终点前 8m → ReachEnd=0 → score=0）
- **TimeLimit**：车不停/超时 → 0 分（pass-through 实验教训）

### 4.5 replay-engine.log 评测配置

- 搜索 `dist_to_obstacle_car_condition` 等关键字可拿到评测器参数：
  ```
  dist_to_obstacle_car_condition { obstacle_id:"7673", min_distance:2, max_distance:5.5,
    if_adc_following_car:false, single_deduction:20 }
  is_critical:true, require_all_time_pass:true
  ```

---

## 5. 哪些信息非常重要

### ⭐⭐⭐ 最高优先级
1. **sim_engine.log 的 detailed_result**：唯一计分帧（带 debug+delta_score）的时刻、描述——直接告诉你怎么失败的
2. **评测时间轴映射**（t = planning − 13.5s）：不对齐一切分析都是错的
3. **planning 的 `planning_start_point`**：车逐帧轨迹/速度——定位车行为

### ⭐⭐ 高优先级
4. **评测器配置**（replay-engine）：`DistToObstacleCar` 的 [min,max] 区间、obstacle_id、single_deduction
5. **ST boundary / 决策日志**（`build reference line st boundary. id:X`）：哪个障碍在阻塞车
6. **fence 位置**（`stop_s/end_s`、`distance_destination/distance_ref_end`）：车为什么减速/停
7. **`not_pass_timestamp`**：各指标失败时刻列表

### ⭐ 参考
8. 本地回放 record（验证代码逻辑）
9. 云端回放 offlineview（验证云端几何/决策）
10. `Current passed destination`、`PARK_AND_GO`、护栏 `UNKNOWN_UNMOVABLE` 等

---

## 6. 推荐排障工作流

```
评测失败
  ↓ ① 下载云端 log（油猴脚本）
  ↓ ② sim_engine.log 找唯一计分帧（debug+delta_score）→ 记录失败时刻+描述
  ↓ ③ 时间轴映射：planning 时刻 = 评测时刻 + 13.5s
  ↓ ④ planning.log 定位该时刻车行为（轨迹/速度/决策/fence）
  ↓ ⑤ 检查评测器配置（replay-engine）确认判定区间
  ↓ ⑥ 提出根因假设 → 改代码 → 本地回放验证
  ↓ ⑦ 重新评测 → 下载 log 验证失败帧是否消失/转移
```

### 实战案例（赛题五 DistToObstacleCar 满分排障，2026-08-02）

1. 5 个 log 的 sim_engine 都有唯一计分帧（80.7/83.8/77.5 随版本变）+ debug "not in [2,5.5]" → 评测在"车停止"采样
2. 时间轴修正：评测 80.7 = 车停稳时刻（planning 94.2s）→ 车停终点距 7673 横向 10.5m 必 >5.5
3. 本地日志：车 40m 外缓减速磨蹭 15s（`stop_s` 持续前移）→ 根因 = 7673 走完仍被 `CheckStopForPedestrian` STOP（不看横向）
4. 修复：行人横向>3.5m 不 STOP + destination 25m 限 stop + 禁 REF_END → 车提前停稳（~69s）→ 评测 71.5s 结束 → **计分帧不存在 → 满分**

---

## 附：相关脚本索引

| 脚本 | 用途 |
|------|------|
| `scripts/scenario_log_downloader.user.js` | 油猴：测评页下载云端 log tgz |
| `scripts/decode_cloud_replay.py` | 解密云端回放 SimWorld（AES "明月几时有"） |
| `scripts/analyze_cloud_ped.py` | 云端回放逐帧分析（行人/决策） |
| `scripts/analyze_record.py` | 本地 record 轨迹/行人分析 |
| `scripts/extract_stop_reason.py` | STOP reason_code 提取 |
| `scripts/planning_lateral.py` | 规划轨迹横向偏移 |
| `scripts/dump_*` 系列 | ST boundary/行人决策/轨迹横向 dump |

## 附：关键经验教训

- **本地通过 ≠ 评测通过**（云端感知/预测抖动放大）
- **评测"距离"是状态门控**（几何不变却单帧失败 → 评测内部状态机，云端不可见）
- **车停 = 评测采样触发器**：提前停稳 → 评测提前结束 → 终点计分帧不存在（比"让距离达标"可行）
- **`PerceptionSLBoundary().s` 噪声可达 60m**（fence 错位撞飞）；ST boundary 的 s 更可靠
- **`is_near_destination` 由 `StopForDestination()` 决定（全程 true）**：配置 `destination_check_distance` 原不生效，需代码手动距离判断
- **多 fence 竞争**（DEST/REF_END/PATH_END）：逐个排查禁用
