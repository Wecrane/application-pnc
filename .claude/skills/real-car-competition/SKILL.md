---
name: real-car-competition
description: 【最高优先级·实车挑战赛专属】百度 Apollo 星火大赛「智能驾驶实车挑战赛」备赛与调参全指南。覆盖：竞赛规则与红线（30分钟计时/SSH登录/仅可改 profiles 与 modules）、实车连接操作（车载Wi-Fi HQEVxxx/apollo123、SSH apollo@192.168.10.6、aem enter、Dreamview+ vehicle test 模式、9模块启动顺序）、**限速插件 scene_speed_limit（主办方提供，crosswalk/traffic_light/speed_bump 三区段限速纯配置化）**、**9 大赛题（定速巡航/减速带/人行道/红绿灯/停止线/障碍物停车避让/动态跟随/人行道路口减速/交通灯路口减速）的纯参数调参方案**（stop_distance=1.75、speed_bump_speed_limit=2.9、pedestrian_stop_distance=1.75、follow_min_time_sec=1.0、AddSpeedLimit=4.5 等）、调参方法论（留裕量/先诊断后调参/配置生效链路 aem profile use default + 重启 Dreamview）、实车与仿真代码差异分析（官方原始代码+限速插件 vs 本地魔改代码 mayaochang add 的风险）。当用户提及实车赛、实车挑战赛、调参、比赛调参、竞赛、赛题、减速带、人行道、红绿灯、停止标志、停止线、障碍物停车、动态跟随、路口减速、限速插件、scene_speed_limit、定速巡航、HQEV、SSH登录、aem enter、vehicle test、DreamView、秩序册、评分、30分钟、profiles、modules、9.1.1、8月6日、8月7日、实车、调参数、参赛 等关键词时必须使用此技能（优先于其他所有技能触发）。
---

# 智能驾驶实车挑战赛 — 备赛与调参指南（最高优先级技能）

> 本技能是所有 Apollo 技能中**优先级最高**的技能。用户进入实车赛备赛/参赛场景时**必须先触发本技能**。
> 实车赛核心原则：**只改配置文件（`profiles/`），尽量不动源码**。理论上所有赛题均可通过配置参数完赛拿满分。
> - ✅ **只改配置**：`profiles/` 下的 pb.txt / planning.conf，改完 `aem profile use default` 即生效（不编译）
> - ⚠️ **非必要不动 `modules/planning` 源码**（动源码=要编译=耗时+风险，30 分钟很宝贵）
> - 🔧 **实在没办法才动源码 + 编译**：仅当赛题要求纯配置确实无法实现时（如官方代码缺限速插件），才考虑改 C++ 并 `buildtool build`
> 信息来源：`shiche/`（秩序册、附录一、自动驾驶闭环文档、限速插件 hqev_speed_limit）+ `output/solutions/`（8 赛题仿真解题记录，800/800 满分沉淀）。

---

## 一、技能触发与定位

**触发条件**：用户提到实车赛、比赛调参、赛题、竞赛规则、SSH 连接、限速插件、定速巡航、减速带、人行道、红绿灯、停止标志、障碍物停车、动态跟随、路口减速等任何与实车赛相关的话题，**立即启用本技能**，无需用户明确要求。

**与其它技能的关系**：
| 技能 | 关系 |
|------|------|
| `apollo-simulation-guide` | 仿真赛专属（改代码/插件开发）；实车赛场景下本技能优先，仿真知识仅作机制参考 |
| `log-replay-analysis` | 日志/回放排障；实车赛调参后如需诊断车行为可调用其方法论 |
| `real-car-competition`（本技能） | **最高优先级**，实车赛一切问题的第一入口 |

---

## 二、竞赛全景（秩序册要点）

### 2.1 关键时间线

| 事项 | 时间 | 说明 |
|------|------|------|
| 虚拟仿真赛 | 8月1日-4日 | 已结束，800/800 满分 ✅ |
| 签到/实车体验 | 8月5日 9:00-18:00 | 携带安全承诺书、保险打印件；实车体验**不提供技术指导** |
| **实车挑战赛** | **8月6日-7日**（上午+下午场） | 每队 **30 分钟**连续计时 |
| 商业答辩赛 | 8月6日 10:00-18:00 | 附加赛，自愿 |

### 2.2 比赛规则（30 分钟怎么用）

- **30 分钟含全部操作**：SSH 登录、程序/参数调试、程序部署、实车测试、重复测试、结果提交
- **机器自动评分**，无裁判评分；最终成绩取正式测评中的**最优有效成绩**
- 操作流程：进入赛位 → SSH 登录计算单元 → 在允许范围内调参/部署 → 选测试项目启动测试 → 评分系统采集数据 → 可重复测试
- **允许修改**：`~/apollo-edu_9.1.1/profiles/` 和 `~/apollo-edu_9.1.1/modules`；其余一律禁止
- 违规红线：篡改评分/传感器数据/日志、装外部软件、动车辆硬件等 → 取消成绩（详见 references/01）。⚠️ 秩序册书面写"严禁个人热点"，但**官方会议（2026-08-04）明确说明热点可用**，以官方说明为准

### 2.3 比赛内容（9 大赛题）

实车赛围绕**行驶、减速、停车、避让、跟随**能力，共 **9 个赛题**（比仿真 8 题多一个**定速巡航**）：

| # | 赛题 | 核心要求 |
|---|------|---------|
| 0 | **定速巡航** | 最高速度 ≤ 11 km/h（约 3.06 m/s） |
| 1 | **减速带通行** | 通过减速带 ≤ 1.5 m/s |
| 2 | **人行道（有人）** | 人行道前 1～1.5 m 停车，等行人走完 |
| 3 | **红绿灯停车** | 停止线前 1.0～1.5 m 停车，不越线 |
| 4 | **停止标志停车** | 停止线前 1.0～1.5 m 完全停止 |
| 5 | **障碍物停车避让** | 障碍物后安全停车，距离 ≥ 2 m |
| 6 | **动态障碍物跟随** | 平稳跟车，保持安全距离 |
| 7 | **人行道路口减速** | 无人行道通过速度 ≤ 1.5 m/s |
| 8 | **交通灯路口减速** | 绿灯通过路口速度 ≤ 1.5 m/s |

> ⚠️ **注意**：实车赛题扣分标准以**现场竞赛手册**为准（秩序册："赛题题库以现场竞赛手册内容为准"）。仿真赛扣分标准（见 references/04）是重要参考，但实车可能不同——**评分细则现场确认**。

---

## 三、实车 vs 仿真：代码差异分析（关键决策依据）

用户面临的核心疑问：**实车代码是否 = 官方原始代码 + 限速插件？要不要把仿真代码恢复成官方原始代码？**

### 3.1 已确认的事实

1. **限速插件 `scene_speed_limit`**（`shiche/hqev_speed_limit.tar.gz`）是主办方单独提供的 TrafficRule 插件，**仿真赛没用过**——说明实车代码大概率 = 官方基线 + 限速插件
2. **插件作用**：对 crosswalk / traffic_light / speed_bump 三类地图元素按配置加区段限速（`AddSpeedLimit`）——**这正是仿真赛题 7/8 需要改代码才能实现的限速**！
3. 本地仿真代码有大量 `mayaochang add` 魔改（见 3.3），**实车官方代码不会有**

### 3.2 限速插件带来的关键结论

| 赛题 | 仿真实现方式 | **实车（带插件）实现方式** |
|------|------------|--------------------------|
| 1 减速带 | gflag `speed_bump_speed_limit=2.9` | **插件 `speed_bump_*` 配置**（或 gflag，取决于官方代码） |
| 7 人行道路口减速 | 改 crosswalk.cc 加 AddSpeedLimit(4.5) | **插件 `crosswalk_limit_speed` 配置** ✅ |
| 8 交通灯路口减速 | 改 traffic_light.cc 加 AddSpeedLimit(4.5) | **插件 `traffic_light_limit_speed` 配置** ✅ |

→ **只要实车带限速插件，赛题 1/7/8 全部可以纯配置解决**，无需改代码。这是"只调参完赛拿满分"的核心依据。

### 3.3 本地魔改代码清单（实车官方代码没有，慎用当前配置！）

| 魔改 | 文件 | 效果 | 实车风险 |
|------|------|------|---------|
| `pedestrian_stop_distance=1.75` gflag | planning_gflags + speed_decider.cc | 静止行人 STOP 1.75m（赛题2） | **实车无此 gflag**，若照搬 profile 会报未知 flag |
| `CheckStopForPedestrian` 横向>3.5m 返回 false | speed_decider.cc | 走完行人不再阻塞（赛题5） | 实车无此逻辑 |
| 护栏 UNKNOWN_UNMOVABLE 无条件 IGNORE | speed_decider.cc + path_decider.cc | 护栏闪烁不 STOP | 实车无此逻辑 |
| path_decider 对 PEDESTRIAN continue | path_decider.cc | 防感知噪声撞飞 | 实车无此逻辑 |
| 无人人行道 AddSpeedLimit(4.5) | crosswalk.cc | 赛题7 | **实车由限速插件替代** |
| 绿灯 AddSpeedLimit(4.5) | traffic_light.cc | 赛题8 | **实车由限速插件替代** |
| `destination_check_distance=25` + 禁 REF_END | destination.cc + planning.conf | 赛题5 提前停稳躲计分帧 | 实车无此 gflag |
| 绿灯/黑灯跳过 | traffic_light.cc | 绿灯放行 | 官方定制（非我们加的） |
| `planning_upper_speed_limit=20`、`default_cruise_speed=16.39` 等 | planning.conf | 起步超速修复 | 通用 gflag，实车可用但值需复核 |

### 3.4 决策建议（回答用户"要不要恢复官方代码"）

**结论：实车赛应基于"官方原始代码 + 限速插件"从零调参，不要直接搬仿真 profile。**

原因：
1. 实车代码（官方基线）与本地魔改代码**肯定不一致**，直接搬配置会：
   - 引用不存在的 gflag（`pedestrian_stop_distance`、`destination_check_distance`、`disable_reference_line_end_stop`）→ 启动报错
   - 缺少本地代码补丁 → 某些赛题（5/7/8）行为与仿真不同
2. 但**解题知识完全可复用**：每个赛题的"机制 → 该调什么参数 → 留多少裕量"的认知链不变
3. 推荐工作流：
   - **赛前**：根据本技能 references/04 的 9 赛题方案，在**官方原始代码+插件**上逐一验证纯配置方案（在自己的 Apollo EDU 9.1.1 环境复现）
   - **赛中**：SSH 登录后，只改 `profiles/` 下对应 conf，不碰 modules/（除非现场确认官方代码缺限速插件才考虑代码）
   - 若时间紧张：优先用插件配置解决限速类（1/7/8），用 traffic rule stop_distance 解决停车类（2/3/4），其余靠官方默认机制

---

## 四、9 赛题调参速查表（核心！）

> 完整机制 + 配置片段见 `references/04-scenario-tuning.md`。以下为速查。

| # | 赛题 | 关键参数 | 位置 | 建议值 | 备注 |
|---|------|---------|------|--------|------|
| 0 | 定速巡航 | `default_cruise_speed` | planning.conf | ≤3.06 m/s 或按现场标准 | 巡航目标速度 |
| 1 | 减速带 | `speed_bump_speed_limit` | planning.conf | 2.5~2.9 | 赛题 ≤3 m/s；2.5 保守 / 2.9 激进（仿真 017 轮实测 2.9+0.2 超调=3.1 有风险，推荐 2.5~2.7） |
| 1' | 减速带(插件) | `speed_bump_limit_speed` | scene_speed_limit conf | 1.5 以下留裕量 | **实车插件方案**（若赛题 ≤1.5 m/s） |
| 2 | 人行道停车 | `stop_distance` | crosswalk conf | 1.75 | 赛题 1~1.5m；车头到线距离=stop_distance+停车偏移，需标定 |
| 3 | 红绿灯停车 | `stop_distance` | traffic_light conf | 1.75 | 赛题 1~1.5m |
| 4 | 停止标志停车 | `stop_distance` | stop_sign conf | 1.75 | 赛题 1~1.5m |
| 5 | 障碍物停车 | `min_stop_distance_obstacle` | planning.conf | 6.0（默认） | 静态障碍 STOP 6m ≥2m 达标；无需改 |
| 6 | 动态跟随 | `follow_min_time_sec` / `stop_follow_distance` | speed_decider conf | 1.0 / 1.5 | 缩短恢复迟滞 |
| 7 | 人行道路口减速 | `crosswalk_limit_speed`（插件） | scene_speed_limit conf | 1.2~1.4（≤1.5 留裕量） | **实车插件方案**；无插件则需代码 |
| 8 | 交通灯路口减速 | `traffic_light_limit_speed`（插件） | scene_speed_limit conf | 1.2~1.4 | **实车插件方案** |

> ⚠️ **仿真限速值 4.5 是仿真赛题（≤5m/s）的取值**；实车赛题是 **≤1.5 m/s**，插件限速值应取 **1.2~1.4**（留 0.1~0.3 裕量）！

---

## 五、限速插件 `scene_speed_limit`（主办方提供）

### 5.1 插件结构

```
modules/planning/traffic_rules/scene_speed_limit/
├── scene_speed_limit.{h,cc}        # TrafficRule 实现
├── conf/default_conf.pb.txt        # 运行时配置（限速值/缓冲区）
├── proto/scene_speed_limit.proto   # SceneSpeedLimitConfig 消息定义
├── plugin_scene_speed_limit_description.xml
├── cyberfile.xml / BUILD
```

### 5.2 工作原理

```cpp
SceneSpeedLimit::ApplyRule()  // 每个规划周期执行
  ├── crosswalk_enabled → 对每个 crosswalk overlap 加 [start_s-forward_buffer, end_s+backward_buffer] 限速 crosswalk_limit_speed
  ├── traffic_light_enabled → 对每个 signal overlap 同上，限速 traffic_light_limit_speed
  └── speed_bump_enabled → 对每个 speed_bump overlap 同上，限速 speed_bump_limit_speed
// 通过 reference_line->AddSpeedLimit(start_s, end_s, limit_speed) 生效
// 与仿真赛题7/8手写 AddSpeedLimit(4.5) 同机制，但纯配置化！
```

### 5.3 配置字段与默认值（proto 定义）

| 字段 | 默认值 | 含义 |
|------|--------|------|
| `crosswalk_enabled` | true | 人行道限速开关 |
| `crosswalk_forward_buffer` | 5.0 | 人行道前缓冲（提前开始限速的距离） |
| `crosswalk_backward_buffer` | 2.0 | 人行道后缓冲 |
| `crosswalk_limit_speed` | 3.0 | 人行道限速值 m/s |
| `traffic_light_enabled` | true | 信号灯限速开关 |
| `traffic_light_forward_buffer` | 8.0 | 信号灯前缓冲 |
| `traffic_light_backward_buffer` | 2.0 | 信号灯后缓冲 |
| `traffic_light_limit_speed` | 4.0 | 信号灯限速值 m/s |
| `speed_bump_enabled` | true | 减速带限速开关 |
| `speed_bump_forward_buffer` | 3.0 | 减速带前缓冲 |
| `speed_bump_backward_buffer` | 1.0 | 减速带后缓冲 |
| `speed_bump_limit_speed` | 2.0 | 减速带限速值 m/s |

> 主办方压缩包里的 `default_conf.pb.txt` 三组 **enabled 全为 false**（未启用），需自行置 true 并配置限速值。详细分析见 `references/03-speed-limit-plugin.md`。

### 5.4 部署要点

- 插件是独立包 `scene-speed-limit`（bazel 插件，`libscene_speed_limit.so`），需按官方方式安装/注册到 `traffic_rule_config.pb.txt`
- 与内置 crosswalk/traffic_light 规则**共存**：内置规则管"停车/避让"，插件只管"限速"，互不冲突
- 优先级/冲突：`AddSpeedLimit` 取**更小的限速值**（speed_limit_decider 取所有限速最小值）→ 插件限速与其他限速并存时按最小生效

---

## 六、实车操作全流程（连接 → 启动 → 调参 → 测试）

### 6.1 赛前准备（个人电脑）

| 软件 | 版本要求 |
|------|---------|
| SSH Client | 无版本限制，能 `ssh -V` 即可 |
| VS Code | **1.96**（组委会指定） |
| Remote-SSH 扩展 | 与 VS Code 1.96 兼容 |

### 6.2 现场连接步骤（30 分钟内）

```bash
# 1. 连接车载 Wi-Fi（隐藏网络）
#    Wi-Fi 名：HQEV + 车辆编号后三位（如车辆 0802 → HQEV802）
#    密码：apollo123
# 2. SSH 登录计算单元（本赛位车辆）
ssh apollo@192.168.10.6        # 密码：apollo
# 3. 进入 Apollo 工程目录（工程名以现场为准，如 apollo-edu-9.1.1）
cd ~/apollo-edu-9.1.1
# 4. 进入容器
aem enter        # 若失败先 aem start 再 aem enter
# 5. 启动 Dreamview+
aem bootstrap start --plus
```

### 6.3 自动驾驶闭环启动（Dreamview+ 操作）

1. 浏览器打开 `192.168.10.6:8888`（直连屏幕则 `localhost:8888`）
2. **必须勾选 vehicle test 模式**（否则需重启所有模块，浪费时间）
3. 高精地图栏选择相应地图
4. 模块栏依次启动：**Gnss Driver → RtkLocalization → Transform → LidarDriver → LidarPerception → Prediction → Planning → ExternalCmd → Canbus → SafetyManager → TaskManager**
5. 车辆可视化 → 路由编辑 → 设途径点 → 保存 → 发送路由
6. 系安全带、关车门、档位 N 档 → 点击"自动驾驶"按钮（状态变"运行中"）
7. 完成后接管车辆（状态回"自动驾驶"）
8. ⚠️ 全程关注车辆状态，异常立即踩刹车/转方向盘接管

> 详细操作 + 常见 SSH 问题排查见 `references/02-connection-guide.md`。

---

## 七、调参方法论（核心工作流）

### 7.1 配置生效链路（必须掌握）

```
修改 profiles/default/.../*.pb.txt 或 planning.conf
  → aem profile use default        # 恢复/同步 profile（编译后必做）
  → 重启 Dreamview（aem bootstrap stop && aem bootstrap start --plus）
  → 验证：cat /apollo/...（软链指向 profile，确认值已生效）
```

- **pb.txt/conf 改动不需编译**，重启即可；改 C++ 才需 `buildtool build -p modules/planning/ -j15`
- 容器内 `/apollo` 是软链 → `profiles/current` → `default`，改 profile 即改运行配置

### 7.2 调参七步法

1. **读场景**：明确赛题要求（限速值/停车距离区间/时间限制）
2. **诊断现状**：先跑一遍看车实际行为（停车距离、速度、是否超时）
3. **定位机制**：查是哪个规则/任务在管这件事（TrafficRule 停车 / gflag 限速 / speed_decider 决策）
4. **选参数**：改对应 conf 的关键字段，**一次只改一个**（避免多参数耦合无法归因）
5. **留裕量**：限速值留 0.2~0.5 裕量（控制超调）；停车距离取区间中值
6. **验证**：实车测试 → 看评分 → 不达标再微调（30 分钟内可重复测试）
7. **固化**：记录改了什么，方便回滚

### 7.3 评分关键认知（仿真 800 分实战沉淀）

- **"车停止"是评测采样触发器**：多次停止都采样距离；车提前停稳 → 评测提前结束 → 躲过不利计分帧（赛题5 满分解法）
- **评测限速 = 参考线限速**：`min(地图车道限速, planning_upper_speed_limit, 插件/规则 AddSpeedLimit)`——**别把 planning_upper_speed_limit 往下调**（会拉低评测限速，自锁陷阱）
- **超速扣分按帧**：每超速 1 m/s 每帧扣 2 分；起步加速超冲是高频超速点
- **停车距离标定**：评测距离 ≈ stop_distance + 停车偏移（QP 停车偏移 0.2~0.8m 波动，由接近 STOP 的速度决定）

### 7.4 实车 vs 仿真差异注意点

| 维度 | 仿真 | 实车 |
|------|------|------|
| 感知 | 理想（worldsim） | 激光雷达真实感知，噪声/闪烁更大 |
| 预测 | 可控 | 真实预测，行人/车辆行为随机性大 |
| 控制 | 理想 | 真实底盘，控制延迟/超调更明显 → **裕量要更大** |
| 场景元素 | 固定 JSON | 现场布设，位置可能有偏差 |
| 时间 | 100s | 30 分钟含全部操作，**调参速度要快** |
| 评分 | 仿真器 | 实车数据采集，细节以现场手册为准 |

---

## 八、备赛清单（赛前必做）

- [ ] 把 9 赛题方案（references/04）在官方原始代码+限速插件环境逐一验证（自己复现 Apollo EDU 9.1.1）
- [ ] 确认限速插件安装方式、注册到 traffic_rule_config 的方法
- [ ] 准备好所有 conf 修改模板（3 个 stop_distance、限速插件配置、巡航速度），现场直接粘贴
- [ ] 测试 SSH 连接、VS Code Remote-SSH、Dreamview 操作流程（8月5日实车体验时间）
- [ ] 制定 30 分钟时间分配：登录(2min) → 调参(10min) → 每赛题测试(2min×9=18min) 或按现场题量取舍
- [ ] 准备回滚方案：记录每处修改，现场 git/备份，出问题快速还原

---

## 九、参考文档索引

| 文档 | 内容 |
|------|------|
| `references/01-competition-rules.md` | 秩序册全文要点：规则、红线、评分、日程 |
| `references/02-connection-guide.md` | 附录一+闭环文档：SSH 连接、Dreamview 闭环操作、FAQ |
| `references/03-speed-limit-plugin.md` | 限速插件源码/配置/部署全解析 |
| `references/04-scenario-tuning.md` | 9 赛题逐题调参方案（机制+配置+经验） |
| `references/05-tuning-methodology.md` | 调参方法论、评分机制、实车差异、时间管理 |

原始材料：
- `shiche/秩序册.md`（转换自 PDF）
- `shiche/附录一_智能驾驶实车赛_笔记本远程实车连接操作手册.md`
- `shiche/自动驾驶闭环文档.md`
- `shiche/_extracted/modules/planning/traffic_rules/scene_speed_limit/`（限速插件源码）
- `output/solutions/00~08_*.md` + `修改清单_全赛题改动.md`（仿真 800/800 解题记录）
