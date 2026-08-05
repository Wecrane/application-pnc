# 苏州备赛 PnC modules + 配置包（suzhou_pnc_modules_config.tar.gz）

> 2026-08-05 打包，内容为**实车 890 拷回**的 PnC 相关 modules 与配置（车端一致版）。
> 用途：现场对照、调参模板、部署参考。解压后保持 relative 路径即可对应实车目录结构。

## 内容清单

| 包内路径 | 内容 | 大小 |
|---------|------|------|
| `modules/planning/` | 限速插件 `scene_speed_limit` 源码（.cc/.h/proto/BUILD/conf） | 52K |
| `modules/control/` | 官方 control 源码（lon_based_pid 等控制器 + 测试数据） | 5.1M |
| `profiles/HQEV_BDN/modules/planning/` | **实车 planning 配置**：traffic_rules（crosswalk/stop_distance=0.10、traffic_light/0.10、stop_sign/4.0、scene_speed_limit 全 false）+ tasks + scenarios + planning.conf（upper=5.0/cruise=5.0） | 1.1M |
| `profiles/HQEV_BDN/modules/control/` | **实车 control 配置**：control.conf + LQR+(matrix_q 5.5/2.5) + PID+(station kp0.3/ki0.3, speed kp0.5/ki0.05) | 220K |
| `profiles/HQEV_BDN/modules/common/data/` | vehicle_param.pb.txt（DKIT/HQEV，4.49m 车）+ global_flagfile.txt | — |

## ⚠️ 关键调参点（苏州 4 题制/3 题制）

| 配置 | 当前实车值 | 建议 | 原因 |
|------|-----------|------|------|
| crosswalk `stop_distance` | 0.10 | **1.25** | 评分要求停车 [1.0,1.5]m，0.1 太近扣 20 + 急刹 4.9-6.9 扣加速度 |
| traffic_light `stop_distance` | 0.10 | **1.25** | 同上（红灯题） |
| stop_sign `stop_distance` | 4.0 | **1.0~1.5** | 旧版评分要求 [0,2.0]m（若现场用旧版 4 题制） |
| planning.conf `default_cruise_speed` | 5.0 | **≤3.0** | 评分限速 ≤3.056，5.0 超速扣分（road_speed 从不满分的根因） |
| planning.conf `planning_upper_speed_limit` | 5.0 | **保持 [1.5,5.1]** | 全局铁律，越界整次判 0 |
| scene_speed_limit 三组 | 全 false | 按需启用 | 减速带限速（地图无 speed_bump 时注意插件无效） |

> 详细依据：`.claude/skills/real-car-competition/references/11-realcar-config-audit.md`、`12-grading-rules.md`、`13-artifacts-deep-analysis.md`、`14-suzhou-competition.md`

## 实车路径对应

- modules → `~/apollo-edu_9.1.1/modules/`（源码壳）
- profiles/HQEV_BDN/modules → 容器内生效配置（profile 覆盖）
- planning.conf 生效路径：`/apollo/modules/planning/planning_component/conf/planning.conf`（profile 覆盖）
