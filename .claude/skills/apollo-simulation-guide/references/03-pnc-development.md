# 03 - PnC 插件开发指南

> 来源：`apollo_docs_md/赛事总汇/03_规划PnC/` 系列文档和 `planning-module-navigator` skill

---

## 开发模式概述

Apollo 9.0+ 重构了 Planning 模块，引入两大核心机制：

1. **全新插件扩展机制**：Scenario（场景）、Task（任务）、TrafficRule（交通规则）全部插件化
2. **分级参数配置机制**：全局参数与局部参数分离，插件独立管理各自参数

### 工作目录结构

```
/apollo_workspace/
├── .aem/envroot/
│   ├── apollo/        # 配置文件生效目录（挂载到 /apollo）
│   └── opt/           # 软件包安装目录（缓存作用）
├── modules/           # 可修改的源码目录
│   └── planning/      # Planning 模块源码
└── profiles/
    ├── current -> default   # 当前启用的配置
    └── default/             # 默认配置目录
```

---

## 插件体系

### 三种插件类型

| 插件类型 | 接口基类 | 作用范围 | 配置文件 |
|----------|---------|---------|---------|
| **Scenario** | `Scenario` | 全局切换，同时只有一个生效 | `scenario_config.pb.txt` |
| **TrafficRule** | `TrafficRule` | 所有场景通用，并行执行 | `traffic_rule_config.pb.txt` |
| **Task** | `Task` | 在 Stage 中按顺序执行 | 各场景 `pipeline.pb.txt` |

### 双层状态机

```
Top Layer（Scenario）
    └── ScenarioManager 决定当前场景
        └── Bottom Layer（Stage）
            └── 场景内 Stage 按序执行
                └── 每个 Stage 包含多个 Task
```

---

## 开发 Scenario 插件

### 基本流程

1. 创建 Scenario 类，继承 `Scenario` 基类
2. 实现各 Stage 流程
3. 注册到 `scenario_config.pb.txt`
4. 添加场景切入条件（如检测到人行横道则切入 Crosswalk 场景）

### 配置文件结构

```
profiles/default/modules/planning/
├── scenario_config.pb.txt        # 场景注册配置
├── traffic_rule_config.pb.txt    # 交通规则注册配置
└── scenarios/
    └── your_scenario/
        └── pipeline.pb.txt       # Stage 和 Task 流水线
```

### 赛事常见场景

| 场景 | 说明 | 参考 |
|------|------|------|
| 人行道避让 | 检测人行横道，减速让行 | `赛事总汇/03_规划PnC/09_场景—人行道避让（基础版）.md` |
| 借道绕行 | 静态障碍物占据车道时的绕行 | `赛事总汇/03_规划PnC/13_场景—借道绕行解题思路.md` |
| 特殊车辆绕行 | 对特殊车辆（校车等）的绕行 | `赛事总汇/03_规划PnC/14_场景—特殊车辆绕行解题思路.md` |
| 施工区域减速 | 施工区域限速慢行 | `赛事总汇/03_规划PnC/15_场景—施工区域减速慢行.md` |
| 断头路 | 断头路场景处理 | `赛事总汇/03_规划PnC/16_场景—断头路解题思路.md` |
| 自主泊车 | 自动泊车入库 | `赛事总汇/03_规划PnC/17_场景—自主泊车.md` |
| 交通标志停止线 | Stop Sign 停止线 | `赛事总汇/03_规划PnC/12_场景—交通标志停止线.md` |
| 动态避障 | 人行道+交通灯复合场景 | `赛事总汇/03_规划PnC/11_场景—动态避障（人行道+交通灯）.md` |

---

## 开发 TrafficRule 插件

### 基本流程

1. 创建 TrafficRule 类，继承 `TrafficRule` 基类
2. 实现交通规则决策逻辑（如交汇路口减速慢行）
3. 注册到 `traffic_rule_config.pb.txt`

### 参考文档

- **新增 TrafficRule 插件流程**：`apollo_docs_md/赛事总汇/03_规划PnC/05_新增TrafficRule插件流程.md`
- **插件机制详解**：`apollo_docs_md/赛事总汇/03_规划PnC/06_插件机制—新增TrafficRule插件.md`
- **实践案例**：`apollo_docs_md/赛事总汇/03_规划PnC/08_实践—交汇路口减速慢行插件.md`

---

## 开发 Task 插件

### 基本流程

1. 创建 Task 类，继承 `Task` 基类
2. 实现路径规划或速度规划逻辑
3. 在场景的 `pipeline.pb.txt` 中注册

---

## 参数配置

### 配置生效机制

```bash
# 切换配置
aem profile use default

# 初始化和同步配置
buildtool profile config init --package planning --profile=default
```

配置修改后，参数会在 `/apollo` 目录下生效。注意：**编译后需要重新执行 `aem profile use default`** 恢复配置。

### Planning2.0 参数说明

参考：`apollo_docs_md/赛事总汇/03_规划PnC/03_Planning2.0参数配置.md`

---

## 调试方法

### 编译

```bash
buildtool build -p modules/planning/ -j15
aem profile use default   # 恢复配置！
```

### 仿真验证

1. 启动 Dreamview：`aem bootstrap start --plus`
2. 进入 PnC 开发调试模式
3. 选择 SimControl 仿真
4. 加载对应场景测试

### 日志调试

```bash
# 查看 Planning 日志
tail -f data/log/planning.INFO

# 查看 Cyber Monitor
cyber_monitor
```

---

## 竞速场景集锦

更多场景参考：`apollo_docs_md/赛事总汇/04_赛事集锦/`

| 场景 | 文档 |
|------|------|
| 左转待转 | `00_场景索引.md` / `01_场景—左转待转.md` |
| 红绿灯路口减速慢行 | `02_场景—红绿灯路口减速慢行（方法一）.md` |
| 人行道通行 | `04_场景—人行道通行.md` |
| 借道绕行 | `05_场景—借道绕行.md` |
| 障碍物绕行 | `06_场景—障碍物绕行.md` |
| 减速带通行 | `07_场景—减速带通行.md` |
| 交汇路口减速慢行 | `08_场景—交汇路口减速慢行（方法一）.md` |
| 慢速车绕行 | `10_场景—慢速车绕行.md` |

---

---

## 插件开发实战模板

> 来源：`apollo_docs_md/赛事总汇/03_规划PnC/05~08`（I）

### 完整开发流程（以"交汇路口减速慢行"为例）

#### Step 1：创建 TrafficRule 插件

```
modules/planning/traffic_rules/intersection_slowdown/
├── BUILD
├── cyberfile.xml              # 包管理配置
├── plugins.xml                # 插件注册
├── intersection_slowdown.h    # 头文件
├── intersection_slowdown.cc   # 实现文件
├── conf/
│   └── default_conf.pb.txt    # 参数配置
└── proto/
    ├── BUILD
    └── intersection_slowdown.proto
```

#### Step 2：注册到系统

在 `profiles/default/modules/planning/traffic_rule_config.pb.txt` 中添加：
```
rule {
  name: "INTERSECTION_SLOWDOWN"
  type: "IntersectionSlowdown"
}
```

#### Step 3：编译验证

```bash
buildtool build -p modules/planning/ -j15
aem profile use default   # 恢复配置！
```

#### Step 4：SimControl 仿真测试

参考：`apollo_docs_md/赛事总汇/03_规划PnC/04_使用SimControl仿真调试.md`

### Scenario 插件开发同理

新增 Scenario 插件的流程与 TrafficRule 类似，区别在于：
- 继承 `Scenario` 而非 `TrafficRule`
- 注册到 `scenario_config.pb.txt` 而非 `traffic_rule_config.pb.txt`
- 需要定义 Stage 流水线（`pipeline.pb.txt`）

---

## 外部接口使用指南

> 来源：`apollo_docs_md/附录/外部接口使用指南.md`（J）

Planning 模块对外暴露的主要接口：

| 接口 | 类型 | 说明 |
|------|------|------|
| `/planning` Channel | 发布 | 输出轨迹 ADCTrajectory |
| PlanningComponent | Component | 主入口，消息驱动 |
| ScenarioManager | 内部接口 | 场景切换管理 |
| PlannerDispatcher | 内部接口 | 规划器选择 |

参考：`apollo_docs_md/附录/外部接口使用指南.md`

---

## 参考

- PnC 开发模式完整文档：`apollo_docs_md/赛事总汇/03_规划PnC/`
- 插件开发实战（H+I）：`apollo_docs_md/赛事总汇/03_规划PnC/08_实践—交汇路口减速慢行插件.md`
- 赛事集锦场景：`apollo_docs_md/赛事总汇/04_赛事集锦/`
- Planning 模块导航：使用 `planning-module-navigator` skill
- 包管理 2.0：`apollo_docs_md/赛事总汇/03_规划PnC/02_PnC包管理2.0解读.md`
