# 09 - 参数配置与版本说明

> 来源：`apollo_docs_md/赛事总汇/03_规划PnC/03_Planning2.0参数配置.md` 和 `apollo_docs_md/发版说明/`

---

## Planning2.0 参数配置机制

Apollo 9.0+ 引入了**分级参数配置机制**，将全局参数和局部参数分离管理。

### 参数分级

| 层级 | 作用范围 | 位置 |
|------|---------|------|
| **全局参数** | 所有插件共享 | `profiles/default/modules/planning/planning.conf` |
| **局部参数** | 单个插件独立管理 | 各插件目录下的 `conf/default_conf.pb.txt` |

### 配置文件结构

```
profiles/default/modules/planning/
├── planning.conf                    # 全局配置
├── scenario_config.pb.txt           # 场景注册（哪些场景生效）
├── traffic_rule_config.pb.txt       # 交通规则注册
├── planner/
│   └── public_road_planner/
│       └── default_conf.pb.txt      # 规划器参数
├── scenarios/
│   └── lane_follow/
│       ├── pipeline.pb.txt          # Stage 和 Task 流水线
│       └── conf/
│           └── default_conf.pb.txt  # 场景参数
├── tasks/
│   └── lane_follow_path/
│       └── conf/
│           └── default_conf.pb.txt  # Task 参数
└── traffic_rules/
    └── crosswalk/
        └── conf/
            └── default_conf.pb.txt  # 交通规则参数
```

### 配置生效流程

```bash
# 1. 修改配置文件
vim profiles/default/modules/planning/...

# 2. 编译（如果需要）
buildtool build -p modules/planning/ -j15

# 3. 恢复 profile（编译后必须执行！）
aem profile use default

# 4. 重启 Dreamview 验证
aem bootstrap stop && aem bootstrap start --plus
```

### Profile 切换

```bash
# 创建多份配置
cp -r profiles/default profiles/my_tuning

# 切换
aem profile use my_tuning

# 恢复默认
aem profile use default
```

---

## 关键参数速查

### 全局参数 (planning.conf)

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `enable_pull_over_at_destination` | 到达终点自动靠边停车 | true |
| `default_cruise_speed` | 默认巡航速度 (m/s) | — |

### LaneFollow 场景参数

| 参数 | 说明 |
|------|------|
| 绕行横向距离 | 障碍物绕行时的横向偏移 |
| 绕行限速 | 绕行时的速度上限 |

### 交通规则参数

| 规则 | 关键参数 |
|------|---------|
| Crosswalk | 停车距离、最大减速度 |
| SpeedSetting | 限速值 |
| StopSign | 停止等待时间 |
| TrafficLight | 黄灯处理策略 |

---

## 发版说明

### Apollo 11.0

- 功能型无人车规模化部署支持
- 全面升级感知、定位、规划与开发工具链
- 软硬协同降低研发门槛

### Apollo 10.0

- 包管理 2.0 升级
- 全新 Dreamview+

### Apollo 9.0

- Planning 模块全面重构
- 引入插件扩展机制和分级参数配置
- Scenario/Task/TrafficRule 全部插件化

### 版本选择建议

- **比赛使用 Apollo 11.0 EDU** (基于 11.0 的赛事专版)
- API 接口在 9.0 → 10.0 → 11.0 间变化不大
- 插件开发模式从 9.0 开始稳定

---

## 参考

- 参数配置详解：`apollo_docs_md/赛事总汇/03_规划PnC/03_Planning2.0参数配置.md`
- 发版说明：`apollo_docs_md/发版说明/`
