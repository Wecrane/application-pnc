# 实车 890 代码备份（2026-08-05）

> 从实车（`apollo@192.168.10.6`，工程 `~/apollo-edu_9.1.1`）打包下载的配置与代码备份。
> 用途：本地研究实车配置基线、对比差异、准备调参模板。

## 打包文件（9 个，共约 500M）

| 文件 | 大小 | 内容 |
|------|------|------|
| `car_config_backup.tar.gz` | 31M | `~/apollo-edu_9.1.1` 的 profiles / modules / apollo-edu / vehicle_conf / edu_scripts / WORKSPACE / setup.sh / start.sh / kill_all.sh |
| `car_apollo_planning.tar.gz` | 178K | 容器 `/apollo/modules/planning` + `control` 编译产物（conf/dag/launch/.so/proto） |
| `car_grading_console.tar.gz` | 62K | **🚨 官方评分控制台** `auto_drive_console/`（presets.json + 17 个 metric + 4 场次场景 json） |
| `car_maps_and_tools.tar.gz` | 33M | **赛题地图** `data/map_data`（4 场次 base_map/routing/sim_map）+ grading_system_py + scenario_injection + vehicle_conf + suzhou/bdn 包 |
| `car_replay_logs.tar.gz` | 23M | 8/3 真实驾驶回放（13M record）+ 8/4 日志（19:36/19:44） |
| `car_selected_logs.tar.gz` | 141M | 8/3（1558/1629）+ 8/4（1005/1409/1648/1831/1905）精选 planning+control 日志 |
| `car_scripts_biglogs.tar.gz` | 68M | 官方脚本（apollo_backup_restore.sh/edu_scripts/kill_all/launch_all）+ 8/4 大日志（100527/105641） |
| `car_more_logs.tar.gz` | 106M | 8/3 剩余大日志（1224/1602/1829）+ 8/4 剩余 100M+ 日志 |
| `car_xh_bak.tar.gz` | 95M | **官方备份** `/media/.XH_2026_bak`（评分系统完整版含 tests + 场景注入 + HQEV_BDN + 地图 + run.log） |

## 解压目录

- `extracted/` — 配置代码解压（profiles/apollo_planning/apollo_control/vehicle_conf）
- `grading_console/` — 评分控制台解压
- `maps_tools/` — 地图+工具解压
- `xh_bak/` — 官方备份解压（含 01-01 评分系统完整版 31 py）
- `replay_logs/` / `selected_logs/` / `scripts_biglogs/` — 日志回放解压

> ⚠️ 大解压目录不入 git（见 .gitignore），只保留 tar.gz + 小配置/源码。

## 解压结构（`extracted/`）

```
extracted/
├── profiles/              # ⚠️ 运行配置（current→HQEV_BDN，车厂定制）
├── modules/               # 源码壳（planning 仅含 scene_speed_limit 插件）
├── apollo_planning/       # 容器 /apollo/modules/planning（conf+dag+launch+so+proto）
├── apollo_control/        # 容器 /apollo/modules/control（同上）
├── vehicle_conf/          # 车辆参数配置
├── apollo-edu/            # 工程包元数据（cyberfile.xml/BUILD）
├── edu_scripts/           # 教育脚本
├── WORKSPACE / setup.sh / start.sh / kill_all.sh
```

## ⚠️ 关键结论（详见 `.claude/skills/real-car-competition/references/11-realcar-config-audit.md`）

1. **profile = `HQEV_BDN`**（不是 default），工程名 `apollo-edu_9.1.1`（下划线）
2. **实车 planning.conf 是车厂定制**：`planning_upper_speed_limit=5.0`、`min/max_stop_distance_obstacle=0.0`、crosswalk stop_distance=0.1、stop_sign=4.0 等，与本地默认差异巨大
3. **限速插件已预装**（traffic_rule_config 含 SCENE_SPEED_LIMIT + junction 区段），三组全 false
4. **源码不完整**：`modules/planning` 只有 scene_speed_limit 插件；主体是容器内编译产物（/apollo/modules/planning），无 .cc/.h 源码 → 实车是"预编译+配置覆盖"模式

## 回滚/还原参考

- profiles 回滚：`scp -r ~/car_backup_时间戳/profiles/* car:~/$PROJ/profiles/`
- 完整还原：车上有 `apollo_backup_restore.sh` 和 `apollo-edu_9.1.1.zip`（310M 官方包）
