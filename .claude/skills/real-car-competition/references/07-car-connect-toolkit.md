# 07 连接工具包（car_connect）与官方环境搭建

> 工具包位置：`shiche/car_connect/`（全部脚本已测试：语法/逻辑验证过）
> 官方环境：本地 git `baseline-official` 分支 = 官方仓库最新 planning；`realcar-official-env` 分支 = 官方+插件+官方 profile（已建好）

---

## 1. 工具包脚本总览

| 脚本 | 作用 | 现场使用时机 |
|------|------|------------|
| `connect_car.sh` | **一键连接（主入口）**：输入车辆编号→连隐藏WiFi→免密→探测工程目录→自动挂载 | 现场第一步 |
| `setup_ssh_config.sh` | 仅免密配置（笔记本侧 ControlMaster，实车零改动，合规） | connect 失败时备用 |
| `mount_car.sh` / `umount_car.sh` | SSHFS 挂载/卸载 profiles+modules 到 `~/car/` | 备用 / 收尾 |
| `check_car.sh` | 实车健康检查（系统/磁盘/内存/工程目录/进程） | 连上后摸底 |
| `apply_config.sh` | **配置一键生效**：`aem profile use default`（+`--restart` 重启 Dreamview） | **每次改完配置** |
| `car_cmd.sh` | 在实车容器内执行命令（查看配置等；编译仅改代码时用） | 查看/兜底 |
| `backup_car.sh` | 备份实车配置到本地（回滚保险） | **改配置前必做** |
| `restore_official.sh` | 恢复官方 planning + 装限速插件 + 重置 profile（环境搭建） | 本地/实车环境重置 |

### 用法速查
```bash
# 现场主流程
bash shiche/car_connect/connect_car.sh 802        # ① 一键连接（或交互输入编号）
bash shiche/car_connect/check_car.sh              # ② 健康检查
bash shiche/car_connect/backup_car.sh             # ③ 备份原配置
# ④ 改配置（AI 通过挂载路径 / 新开 VSCode 窗口 ~/car/profiles）
bash shiche/car_connect/apply_config.sh           # ⑤ 生效: aem profile use default
# ⑥ 测试 → 看评分 → 微调 → 重复
```

### 连接原理（为什么要这样）
- 新版 VSCode Remote-SSH 在 Ubuntu 18.04（glibc 2.27）装不上 vscode-server（需 glibc≥2.28）→ 连不上
- **绕过**：不依赖 Remote-SSH，用 **SSHFS 挂载**文件到本地 + **SSH 终端**执行命令
- **免密用 ControlMaster**（笔记本侧 `~/.ssh/config` 的 `Host car`）：首次输一次密码，4 小时内 ssh/scp/sshfs 全免密；**实车 SSH 配置零改动** → 合规

### 关键合规点
- 只改笔记本侧配置；实车只动 `profiles/`（非必要不动 `modules/`）
- sshfs 只装笔记本（已装），车上零安装
- 手机热点：官方会议（2026-08-04）确认**可用**（秩序册书面虽写"严禁"，以官方说明为准）

---

## 2. 官方环境搭建（实车等价环境，本地研究用）

### 2.1 现状（已建好）
| 分支 | 内容 |
|------|------|
| `baseline-official` | **官方仓库最新 planning 代码**（无魔改、无限速插件、官方默认 profile） |
| `realcar-official-env` | = baseline-official + `scene_speed_limit` 插件 + 官方 profile（重装完成） |
| 备份 | `backup/realcar_20260804/`（tar，gitignore）；git 历史各分支完整保留仿真 800 分代码 |

### 2.2 恢复脚本（可复用）
```bash
# 在 application-pnc 根目录执行
bash shiche/car_connect/restore_official.sh                # 备份→恢复官方→装插件→重置profile
bash shiche/car_connect/restore_official.sh --backup-only  # 仅备份
```
> ⚠️ 会覆盖 modules/planning 全部本地修改 + 重置 profiles/default（有自动备份 + git 可回滚）

### 2.3 插件下载机制（官方 skill 赛事汇总）
```bash
buildtool install planning*                     # 下载全部 planning 子模块源码（官方仓库）
buildtool install planning-traffic-rules-crosswalk   # 下载指定模块
```
> **重要概念**：`buildtool install` 下载的是**源码**到 `modules/` 供修改。Apollo 核心包已含所有默认插件**编译好的二进制**，不下载也能运行默认场景。实车若自带插件则无需下载；本地重装直接用 `baseline-official` 分支即可。

### 2.4 插件注册（配置层，装完插件需加）
```text
# modules/planning/planning_component/conf/traffic_rule_config.pb.txt 追加：
rule {
  name: "SCENE_SPEED_LIMIT"
  type: "SceneSpeedLimit"
}
```
> 官方 traffic_rule_config 默认**没有** SCENE_SPEED_LIMIT，需手动加；首次装插件需 `buildtool build -p modules/planning/ -j15` 编译一次生成 `.so`。

---

## 3. 官方默认插件全景（baseline-official 实测）

- **TrafficRule（9 个启用）**：BACKSIDE_VEHICLE / CROSSWALK / DESTINATION / KEEP_CLEAR / REFERENCE_LINE_END / REROUTING / STOP_SIGN / TRAFFIC_LIGHT / YIELD_SIGN
- **Scenario（12 个注册）**：EMERGENCY_PULL_OVER / EMERGENCY_STOP / VALET_PARKING / BARE_INTERSECTION_UNPROTECTED / STOP_SIGN_UNPROTECTED / YIELD_SIGN / TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN / TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN / TRAFFIC_LIGHT_PROTECTED / PULL_OVER / PARK_AND_GO / LANE_FOLLOW
- **LaneFollow 任务链**：LANE_CHANGE_PATH → LANE_FOLLOW_PATH → LANE_BORROW_PATH → FALLBACK_PATH → PATH_DECIDER → RULE_BASED_STOP_DECIDER → SPEED_BOUNDS_PRIORI → PATH_TIME_HEURISTIC → SPEED_DECIDER → SPEED_BOUNDS_FINAL → PIECEWISE_JERK_SPEED
- **关键默认值**：planning_upper_speed_limit=16.667、default_cruise_speed=16.667、speed_bump_speed_limit=3、destination_check_distance=4.0、crosswalk/traffic_light/stop_sign stop_distance=1.0、speed_decider follow_min_time_sec=2.0/stop_follow_distance=2.0

> 9 赛题插件矩阵 + 纯配置方案见 `references/06-official-config-solutions.md`（核心！）
