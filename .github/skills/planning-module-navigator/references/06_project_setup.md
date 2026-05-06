# 项目安装与环境配置

> 来源：`README.md`、`README_cn.md`、`CLAUDE.md`

## 工程概述

本项目为 Apollo 包管理开发调试样例工程，基于 Apollo 10.0 版本。

## 文件目录组织

```shell
.
├── core              # 依赖包配置，包括 apollo 核心包和工具包
├── profiles          # 整车应用配置
├── .env*             # 环境配置文件（容器名、镜像源等）
├── .workspace.json*  # 软件包源配置文件（依赖软件版本号）
├── .buildtool*       # 编译配置文件
├── setup.sh          # 切换 x86_64 和 aarch64 架构配置
├── kill_all.sh       # 停止所有 apollo 相关进程
├── WORKSPACE         # Bazel 配置
├── modules/          # 模块源码
│   └── planning/     # Planning 模块
└── data/             # 数据目录
    ├── log/          # 日志
    ├── bag/          # 数据包
    └── map_data/     # 地图数据
```

## 安装步骤

### 更新 AEM 版本

```shell
sudo apt update
sudo apt install apollo-neo-env-manager-dev --reinstall
```

### 初次安装

```shell
# 切换环境配置
bash setup.sh

# 启动容器
aem start

# 进入容器
aem enter

# 安装软件包（建议执行两次）
buildtool build

# 下载地图
buildtool map get sunnyvale

# 切换车辆配置
aem profile use sample

# 启动 DreamView+
aem bootstrap restart --plus
```

### 从旧版本升级

```shell
# 切换环境配置
bash setup.sh

# 进入容器
aem enter

# 升级工具
buildtool upgrade

# 升级已安装的软件包
buildtool build

# 下载地图
buildtool map get sunnyvale

# 切换车辆配置
aem profile use sample

# 启动 DreamView+
aem bootstrap restart --plus
```

## 常用命令速查

| 操作 | 命令 |
|------|------|
| 启动容器 | `aem start` |
| 进入容器 | `aem enter` |
| 编译全量 | `buildtool build` |
| 编译 planning | `buildtool build -p modules/planning/` |
| 编译核心包 | `buildtool build -p core` |
| 下载 planning 源码 | `buildtool install planning*` |
| 下载指定子模块源码 | `buildtool install planning-traffic-rules-crosswalk` |
| 重装包管理代码 | `buildtool reinstall planning*` |
| 初始化 profile | `buildtool profile config init --package planning --profile=default` |
| 恢复 profile 配置 | `aem profile use default` |
| 启动 DreamView+ | `aem bootstrap start --plus` |
| 启动 Planning（单模块） | `mainboard -d modules/planning/planning_component/dag/planning.dag` |
| 查看日志 | `tail -f data/log/planning.INFO` |

## 日志管理

- 日志位置：`data/log/`
- Planning 日志：`data/log/planning.INFO`（当前）、`data/log/planning.log.INFO.*`（历史）
- 清理历史日志：`find data/log/ -name "*.log.*20[0-9][0-9]*" -type f -delete`

## 编译注意事项

1. `buildtool build` 建议执行两次（第一次下载依赖，第二次确保完整）
2. 编译后务必执行 `aem profile use default` 恢复 profile 配置
3. 修改源码后打包提交：`tar -zcvf 提交包.tar.gz modules/planning/ profiles/default`
4. 仅改配置打包提交：`tar -zcvf 提交包.tar.gz profiles/default`

## PnC 完整链路

```
外部命令（路由）→ Planning（规划）→ Prediction（预测输入）→ Control（控制执行）
```

PnC 链路不包含感知/定位等上游模块，不需要 GPU。

> 来源：`README.md`、`README_cn.md`、`CLAUDE.md`
