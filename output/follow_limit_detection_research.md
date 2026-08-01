# "跟车限制检测"判定线索 — 穷尽检索报告

> 研究日期：2026-08-01 | 场景：障碍物停车避让（id=`6a699aefabf06e2936590839`，赛题六）
> 检索范围：场景 json 全集、apollo_docs_md/赛事总汇、备份知识库、output/ 全部分析文件、回放数据

---

## 0. 结论速览

| 问题 | 结论 |
|------|------|
| 场景 json 是否有检测项定义 | **没有逐场景检测项**。只引用共享外部评分配置 `grading_system/conf/grading_metrics_default.conf`（并移除 Checkpoint）。`speed_limit`/`collision`/`跟车限制` 字段均不在 json 内 |
| 赛事文档是否有"跟车限制"定义 | **无官方定义**。术语仅出现在 3 个解决方案记录文件（全部来源于评测页核对），非官方文档 |
| 最可能的判定逻辑 | **主车不应跟随正在移动/横向离开车道的行人（障碍物），应停车等待其离开**——"跟车限制"是评测系统对"主车跟在障碍物后面蠕动行驶"行为的限制检测（详见 §4 推断） |

---

## 1. 场景 json 完整内容（id=6a699aefabf06e2936590839）

文件：`/home/skye/.apollo/resources/scenario_sets/6a6c99a3abf06e2d1d590840/scenarios/6a699aefabf06e2936590839.json`（7408 字节）

### 1.1 顶层与场景信息

```json
{
  "id": "6a699aefabf06e2936590839",
  "authorName": "apollo_3103917591",
  "type": "worldsim",
  "mapId": "69d51ffd0e41e1217f2fda19",
  "tags": ["Straight Road", "Traveling straight in a through lane", "Static obstacle", "Pedestrian"],
  "descriptionEn": "障碍物停车避让场景",
  "descriptionEnTokens": ["xh_2026_gsxx_障碍物停车避让场景"]
}
```

### 1.2 评分配置（核心！）

```json
"gradingConfigInfo": {
  "baseGradeConfigFile": "grading_system/conf/grading_metrics_default.conf",
  "deselectDefaultMetric": ["Checkpoint"]
}
```

**关键结论**：检测项/评分指标全部来自**共享外部配置文件** `grading_system/conf/grading_metrics_default.conf`（评测系统 simulation 模块，本地/容器内均未找到该文件），场景 json 本身**没有**任何 `speed_limit`、`collision`、`跟车限制`、`follow` 检测字段。

> ⚠️ 已核对：8 个场景（同目录下全部 json）的 `gradingConfigInfo` **完全相同**——都引用同一个 `grading_metrics_default.conf` 并都移除 Checkpoint。即"跟车限制检测"是**所有场景共用**的评测指标，非本场景独有。

### 1.3 地图与路由

```json
"roadNetwork": { "logicFile": { "filepath": "modules/map/data/Xh_2026_contest" } },
"autoCarInfo": {
  "start": { "x": 423278.19, "y": 4437610.95, "heading": -0.0031 },
  "end":   { "x": 423491.11, "y": 4437610.34 },
  "routingRequest": { "waypoint": [起点, 终点] }
}
```
- 路线：`Lane_1725 → Lane_1724`（2 条直道，约 213m）
- 地图限速：两车道均 `speed_limit = 19.444 m/s`（70 km/h，`output/map_analysis_full.json:3249-3261`）
- 路线元素：无（无红绿灯/人行道/停止标志，障碍为感知动态生成）

### 1.4 障碍物定义（entities.scenarioObjects，7 个）

| 对象 | 类型 | 尺寸 | 初始位置 (x, y) |
|------|------|------|------------------|
| 3098 | unknownUnmovableObject | 30×0.3×2 | (423395.25, 4437613.01) |
| 4717 | unknownUnmovableObject | 15×0.3×2 | (423416.12, 4437613.35) |
| 4857 | unknownUnmovableObject | 30×0.3×2 | (423298.30, 4437613.23) |
| 5530 | unknownUnmovableObject | 30×0.3×2 | (423438.43, 4437612.88) |
| 6324 | unknownUnmovableObject | 30×0.3×2 | (423329.67, 4437613.14) |
| 6503 | unknownUnmovableObject | 30×0.3×2 | (423362.17, 4437613.03) |
| **7673** | **pedestrian**（行人） | **5×2×1.8** | **(423446.67, 4437610.95)** |

- 6 个长条障碍 = 路肩护栏/隔离设施（type=2 UNKNOWN_UNMOVABLE，y 偏离车道中心线 ~2m）
- **7673 是行人（PEDESTRIAN），初始静止在车道中央** x=423446.67（主车起点 423278 前方约 168m）

### 1.5 行人触发移动（storyboard）

```json
// 7673 的 maneuver：
"privateAction": {
  "longitudinalAction": {
    "speedAction": {
      "speedActionDynamics": { "dynamicsDimension": "distance", "dynamicsShape": "linear", "value": 20 },
      "speedActionTarget": { "absoluteTargetSpeed": { "value": 3 } }
    }
  }
},
"startTrigger": {
  "conditionGroups": [{
    "conditions": [{
      "conditionEdge": "none",
      "byValueCondition": { "simulationTimeCondition": { "rule": "greaterOrEqual", "value": 50 } }
    }]
  }]
}
```
- **行人轨迹**：(423446.67, 4437610.95) → (423475.75, 4437599.76)（向东北走约 32m，横向离开车道 ~11m）
- **触发条件**：仿真时间 **≥ 50s** 后，20m 距离内线性加速到 **3 m/s**

### 1.6 场景结束条件

```json
"stopTrigger": { "conditions": [{ "simulationTimeCondition": { "rule": "greaterOrEqual", "value": 100 } }] }
```
- 障碍物停车避让 = **100s** 仿真上限（动态跟随场景 6a699d15 是 60s）

---

## 2. "跟车限制"在项目文档中的全部出处（穷尽）

全项目（含 output/、backup/、profiles/、modules/、apollo_docs_md/、备份知识库）grep `跟车限制`，**仅 3 个文件命中，全部是解决方案记录（数据来源=评测页人工核对）**：

### 2.1 `output/solutions/00_赛题总览.md`
- 第 40 行：`> 指标来源：评测页逐题核对（2026-08-01 三次核实，脚本精确提取；修正赛题四/五"时间限制"实为通过）`
- 第 49 行：`| 六 | 障碍物停车避让 | 0 ❌ | 100 | **跟车限制、关键点、到达终点、时间限制** |`
- 第 58 行：`5. **赛题六（障碍物停车避让）**：跟车限制❌+关键点❌+终点❌+时间限制❌（唯一超时）→ 停车后跟车/起步异常`

### 2.2 `output/solutions/05_障碍物停车避让场景.md`
- 第 15 行：`| **跟车限制检测** | ❌ 未通过 |`（其余：加速/向心/碰撞/在路/限速 ✅ 通过）
- 第 102-114 行（7.7 节，最核心推断，见 §4）
- 第 127 行：`- [ ] 评测：跟车限制✅`

### 2.3 `output/solutions/06_障碍物车辆跟随场景.md`
- 第 103 行：`速度震荡（1.86/2.66 m/s 骤降）可能触发"跟车限制/急减速"判定`
- 第 219 行：`跟车速度...前车速度决定主车上限`

**结论**："跟车限制"是**评测页上的指标名**（与"关键点检测/到达终点检测/时间限制/加速/向心/碰撞/在路/限速"并列），无任何官方文档定义其具体算法。知识库（`/home/skye/pnc_backup_full/.claude/apollo-knowledge/`，61 文档）与 `apollo_docs_md/`（353 文档）均无此术语。

---

## 3. 场景行为实测（回放证据）

### 3.1 四次评测前回放 `output/obs7673_1759a.txt`（202608011759 录制）
```
t=  0.46  7673:stop   MAIN_STOP(rc=4) pt=(423438.12,4437610.77)   ← 主车停在行人前
...（持续 stop，主车 v=0，栅栏 423438 固定）
t= 43.52  7673:follow veh_v=0.00                                  ← 行人开始移动 → 主车立即转 follow！
t= 52.18  7673:yield  veh_v=2.35                                  ← 9 秒蠕动跟车后才转 yield
```
**关键行为**：行人 50s 开始移动（感知到约 43.5s），主车在行人刚动 **1.2s** 就转 follow，以 0~2.35 m/s **蠕动跟在行人后面约 9 秒**。

### 3.2 修复后最新回放 `output/2209_ped_vs_ego.txt`（202608012209 录制）
```
t= 58.x  ped=(423462.29,4437604.94, v=2.75~3.00)  veh=(423440.07, v=3.7~5.1)  obs决策: -
```
行人移动时 obs 决策列为 **"-"**（无 stop/follow/yield），主车以 4-5 m/s 正常行驶通过——修复后不再跟车。

---

## 4. "跟车限制检测"最可能判定逻辑（综合推断）

### 4.1 项目内已有推断（`output/solutions/05_障碍物停车避让场景.md` 7.7 节，原文）

> **"跟车限制检测"判定（推断）**：障碍（行人）刚开始移动（仍在近处）时，车**不应立即跟随**（跟车/蠕动跟在行人后面）——应**停车等待**行人横向离开车道后通行。

### 4.2 我的综合分析（证据链）

1. **检测项是全局共享的**：所有 8 场景引用同一 `grading_metrics_default.conf`。"跟车限制"（对应英文名推测为 `FollowLimit` 一类 metric）是评测系统对**车辆跟车行为**的通用合规检测，而非场景特定规则。
2. **本场景唯一动态障碍 = 行人 7673**：50s 前静止（车 STOP 正确）、50s 后以 3m/s 横向离开车道。
3. **失败行为已证实**：回放显示主车在行人刚移动时就 follow 并蠕动跟车 9 秒（t=43.5~52.2）。这正是"跟车"行为。
4. **与场景意图冲突**：场景名叫"障碍物**停车避让**"——正确行为是停车等行人离开（横向位移 ~11m），而非跟在行人后面走。评测器若检测到主车"跟随障碍物行驶"，即判"跟车限制"不通过。
5. **与赛题六（动态跟随）对照**：动态跟随场景（前车为 VEHICLE，同向行驶）主车**应该**跟随——两场景共用同一指标，但行为预期相反。因此"跟车限制"更可能是检测**跟车过程中的限制条件**（如：跟车速度、跟车距离、跟车对象类型），而非"禁止跟车"本身。

### 4.3 判定逻辑候选（按可能性排序）

| 可能性 | 判定逻辑 | 与证据符合度 |
|--------|---------|:---:|
| **A. 不应跟行人（障碍类型限制）** | 动态障碍为**行人**时，主车不应转入 FOLLOW，应停车等待 | ⭐⭐⭐ 最符合（行人横穿/横向离开车道场景的常识要求） |
| **B. 跟车速度限制** | 主车跟车速度低于某阈值（如长时间 <3m/s 蠕动）即判违规 | ⭐⭐⭐ 与回放（0~2.35m/s 蠕动 9s）高度吻合 |
| **C. 跟车距离限制** | 跟车时与障碍纵向距离过近/过远 | ⭐⭐ 本场景行人移动时距离约 8m，不太像主因 |
| **D. 横向距离限制** | 跟随/绕行障碍时横向距离不足 | ⭐ 本场景是纵向跟车，横向非主因 |
| **E. 速度限制（speed limit）** | 跟车段超过某限速 | ⭐ 主车跟车时仅 0~2.35m/s，未超速，排除为主因 |

**最可能**：**A + B 组合**——评测器检测"主车是否跟在行人（障碍物）后面蠕动行驶"（长时间低速跟随动态行人），一旦检测到即判"跟车限制"失败。对应 05 文档已实施的修复（add7：PEDESTRIAN 不 follow、直接 STOP 等待）正是针对此逻辑。

### 4.4 确认"跟车限制检测"最终定义的唯一途径

`grading_metrics_default.conf` 定义所有指标，位于评测系统（Apollo simulation 模块，`GradingConfig` 类，见 `apollo_docs_md/源代码文档/类列表.md:3811`、`类索引.md:1268`：`GradingConfig (apollo::simulation)`）。**本地与备份均无此文件**。建议在容器内查找：

```bash
aem exec -- "find /opt/apollo -name 'grading_metrics*' 2>/dev/null"
aem exec -- "find /opt/apollo -path '*simulation*' -name '*.conf' 2>/dev/null"
```
（找到后可确认 FollowLimit / SpeedLimit / LateralDistance 等 metric 的确切参数与阈值）

---

## 5. 附：已核实的其他关键事实

- **场景 json 无 speed_limit 字段**：本场景限速来自地图（19.44 m/s = 70km/h），无障碍物区域限速。评测"限速检测"✅通过。
- **"关键点检测/到达终点检测"失败** = 主车未到终点（与跟车蠕动耗时直接相关，100s 内未完成 213m）。
- **"时间限制"失败** = 100s 仿真超时（本场景 stopTrigger=100s）。
- **其他场景 json 文件**（同目录）：`67d938f854cb7763ce6083c2`、`67e4bc95cf6d730104b25ec1`、`69d420a3b58e96002b6da084`（减速带/人行道等）、`6a699d15395ab133a4433b3a`（障碍物车辆跟随，60s 上限）、`6a6c3d8babf06e0e5f59083c`、`6a6c3ea5abf06e7b4a59083d`、`6a6c3f21abf06e3f1359083e`。
- **场景集**：`~/.apollo/resources/scenario_sets/6a6c99a3abf06e2d1d590840/scenario_set.json` = "2026星火自动驾驶比赛--全国总决赛仿真赛"（8 个 ticket）。

---

## 6. 检索范围清单（已穷尽）

| 范围 | 结果 |
|------|------|
| `~/.apollo/resources/scenario_sets/`（8 个 json + scenario_set.json） | ✅ 全部 dump，评分配置引用外部文件 |
| `apollo_docs_md/赛事总汇/`（含 01-09 子目录全部 md） | ✅ 无"跟车限制"定义；仅有跟车决策背景（`04_赛事集锦/10_场景—慢速车绕行.md` §五） |
| `apollo_docs_md/` 全库（353 md，含源代码文档/框架设计） | ✅ 仅 `GradingConfig (apollo::simulation)` 类名引用，无指标定义 |
| `.claude/apollo-knowledge/`（实际位置 `/home/skye/pnc_backup_full/.claude/apollo-knowledge/`，61 文档） | ✅ 无"跟车限制"；`赛题.md` 为旧版赛题（变道/S弯/U-Turn/环岛），非当前 8 场景 |
| `output/solutions/*.md`（9 个） | ✅ 评测记录含指标名；05/06 有机制推断 |
| `output/scenarios_analysis.json/.txt`、`map_analysis_full.json` | ✅ 路线/限速数据确认 |
| `output/record_analysis.json` | ✅ 仅位置序列，无评分信息 |
| 回放数据 `obs7673_1759a.txt`、`2209_ped_vs_ego.txt` 等 | ✅ 证实"蠕动跟车 9s"失败行为 |
| `grading_metrics_default.conf`（本地/备份全盘搜索） | ❌ 未找到（在评测云/容器 simulation 模块内） |
