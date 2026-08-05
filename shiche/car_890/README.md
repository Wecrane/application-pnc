# 实车 890 代码备份（2026-08-05）

> 从实车（`apollo@192.168.10.6`，工程 `~/apollo-edu_9.1.1`）打包下载的配置与代码备份。
> 用途：本地研究实车配置基线、对比差异、准备调参模板。

## 打包文件

| 文件 | 大小 | 内容 |
|------|------|------|
| `car_config_backup.tar.gz` | 31M | `~/apollo-edu_9.1.1` 的 profiles / modules / apollo-edu / vehicle_conf / edu_scripts / WORKSPACE / setup.sh / start.sh / kill_all.sh |
| `car_apollo_planning.tar.gz` | 178K | 容器 `/apollo/modules/planning` + `control` 编译产物（conf/dag/launch/.so/proto） |

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
