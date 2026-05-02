# Apollo EDU 赛事环境安装引导使用手册

> 版本：2.1.0 | 适用平台：Ubuntu Linux | 适用赛事：百度 Apollo 星火自动驾驶大赛（PnC 赛道）

---

## 一、概述

本手册整合了 Apollo EDU 赛事系统的完整安装流程，提供环境检测、交互式安装引导和故障诊断能力。

> **配置说明：** 所有版本要求和 URL 配置统一在 `config.yaml` 中管理（单一事实来源）。
> **PnC 赛道专注：** 本手册专注于 Planning & Control 赛道，不涉及感知模块，不需要 GPU。

### 核心能力

| 能力 | 说明 |
|------|------|
| 环境检测 | 自动检测操作系统、Docker、CPU、内存、磁盘、Swap、网络连通性等 |
| 交互式安装 | 按步骤引导安装，每步需用户确认 |
| 赛事专项 | 编译缓存、场景插件、profile 配置、压缩包制作 |
| 故障诊断 | 匹配常见问题，给出修复方案 |

### 版本配置要求

| 项目 | 最低要求 |
|------|----------|
| 操作系统 | Ubuntu 18.04 / 20.04 / 22.04（不推荐 24.04） |
| Docker Engine | ≥ 19.03 |
| CPU | ≥ 4 核 |
| 内存 | ≥ 16GB |
| 磁盘空间 | ≥ 55GB |
| GPU | 无要求（PnC 赛道不需要 GPU） |
| 安装方式 | aem（Apollo Environment Manager） |

---

## 二、快速开始（老手通道）

如果你已有 Apollo 开发经验，以下是最精简的安装命令序列：

```bash
# 1. 安装 Docker（如已安装跳过）
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh && bash docker_install.sh

# 2. 安装 aem（如已安装跳过）
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg
sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core $(. /etc/os-release && echo "$VERSION_CODENAME") main" | sudo tee /etc/apt/sources.list.d/apolloauto.list
sudo apt-get update && sudo apt install apollo-neo-env-manager-dev --reinstall

# 3. 克隆工程 + 启动 + 编译
git clone https://github.com/ApolloAuto/application-pnc.git
cd application-pnc && bash setup.sh
aem start
buildtool build -p core
buildtool build -p core   # 执行两次确保完整

# 4. 验证
aem bootstrap start --plus
# 访问 http://localhost:8888
```

> 如遇任何问题，使用 `/apollo-env diagnose` 进行诊断。

---

## 三、命令格式

```
/apollo-env          # 主入口：交互式安装引导（检测 → 安装 → 验证）
/apollo-env check    # 仅执行环境检测
/apollo-env install  # 跳过检测，直接安装缺失项
/apollo-env diagnose # 诊断安装问题
```

---

## 四、触发条件

### 自动触发时机

- 用户询问如何安装 Apollo
- 用户需要配置 Apollo 开发环境
- 用户在安装 Apollo 过程中遇到问题
- 用户输入 `/apollo-env` 命令

### 不应触发的情况

- 用户只是询问 Apollo 的使用方法（非安装相关）
- 用户在做 Apollo 模块开发（非环境搭建）— 应走知识库检索流程

---

## 五、环境检测项详解

运行环境检测脚本后，会生成以下检测报告：

### 5.1 操作系统

| 状态 | 条件 |
|------|------|
| ✅ 通过 | Ubuntu 18.04 / 20.04 / 22.04 |
| ⚠️ 警告 | Ubuntu 24.04 或其他 Linux 发行版 |
| ❌ 失败 | macOS 或其他不支持的系统 |

> **macOS 用户注意**：Apollo 不支持在 macOS 上直接安装，需通过 Ubuntu 虚拟机或远程 SSH 方式操作。

### 5.2 CPU

| 检测项 | 要求 |
|--------|------|
| CPU 核心数 | ≥ 4 核 |

### 5.3 Docker Engine

| 检测项 | 要求 |
|--------|------|
| Docker 版本 | ≥ 19.03 |
| 守护进程状态 | 运行中 |
| 用户权限 | 当前用户属于 docker 组（免 sudo） |

### 5.4 内存

| 检测项 | 要求 |
|--------|------|
| 总内存 | ≥ 16GB（推荐） |

> 低于 16GB 可能导致编译缓慢或失败。

### 5.5 磁盘空间

| 要求 | 说明 |
|------|------|
| 根分区 ≥ 55GB | Apollo Docker 镜像和编译产物需要较大空间 |
| Docker 数据目录 ≥ 55GB | Docker 镜像将存储在该分区 |

### 5.6 网络连通性

| 检测项 | 用途 |
|--------|------|
| Apollo CDN | apt 安装、编译缓存下载 |
| GitHub | 克隆 application-pnc 工程 |

### 5.7 aem 环境管理工具

检测 `aem` 命令是否可用，以及是否有运行中的 Apollo 容器。

### 5.8 buildtool 构建工具

检测 `buildtool` 命令是否可用（通常仅在 Apollo 容器内可用）。

### 5.9 Apollo PnC 赛事工程

检测 `application-pnc` 是否已克隆。

---

## 六、交互式安装流程

### Step 1: 环境检测

运行检测脚本，收集系统信息：

```bash
bash {BASE_DIR}/scripts/env_check.sh
```

JSON 格式输出（便于程序解析）：
```bash
bash {BASE_DIR}/scripts/env_check.sh --json
```

根据检测结果判断是否可以继续：
- 所有必须项通过 → 进入 Step 2
- macOS 检测 → 提示用户使用 Ubuntu 或 SSH 方式
- 部分缺失 → 引导安装

### Step 2: 安装缺失依赖

按以下顺序安装，**每步执行前需用户确认**：

#### 2a. 安装基础软件（Ubuntu + Docker）

安装 Ubuntu 操作系统（推荐 18.04、20.04、22.04，不推荐 24.04）后，更新系统：

```bash
sudo apt-get update
sudo apt-get upgrade
```

使用 Apollo 提供的安装脚本安装 Docker：

```bash
wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
bash docker_install.sh
```

> 如遇下载失败或下载过慢，使用阿里云镜像：
> ```bash
> sudo rm -f /etc/apt/sources.list.d/docker.list
> wget http://apollo-pkg-beta.bj.bcebos.com/docker_install.sh
> wget http://apollo-pkg-beta.bj.bcebos.com/get_docker.sh
> bash get_docker.sh --mirror Aliyun
> bash docker_install.sh
> ```

#### 2b. 安装 aem 环境管理工具

```bash
# 添加 GPG key
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg
sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg

# 设置源并更新
echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core"\
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | \
    sudo tee /etc/apt/sources.list.d/apolloauto.list
sudo apt-get update

# 安装 aem
sudo apt install apollo-neo-env-manager-dev --reinstall
```

> **注：** 如果之前已安装过 Apollo 8.0，宿主机 `/etc/apt/sources.list` 中可能有旧配置（形如 `deb https://apollo-pkg-beta.cdn.bcebos.com/neo/beta bionic main`），可以直接删除。

安装成功后验证：
```bash
aem -h
```

### Step 3: 克隆并配置 PnC 赛事工程

```bash
git clone https://github.com/ApolloAuto/application-pnc.git
```

> 如果无法访问 GitHub，使用 gitee 镜像：
> ```bash
> git clone https://gitee.com/ApolloAuto/application-pnc
> ```

进入工程目录并配置环境：

```bash
cd application-pnc
bash setup.sh
```

> `bash setup.sh` 会根据主机系统架构（x86_64/aarch64）自动配置环境。

验证工作目录：
```bash
cat .workspace.json
```

### Step 4: 启动环境并构建

```bash
# 拉取并启动 Docker 容器（在工程目录下执行）
cd application-pnc
aem start

# 检查 buildtool 版本
buildtool -v

# 编译工程（建议执行两次以确保依赖完整）
buildtool build -p core
buildtool build -p core
```

> **编译缓存（可选）**：如因网络问题无法正常下载依赖，可先下载编译缓存：
> ```bash
> wget https://apollo-system.bj.bcebos.com/bazel_deps/cache.tar.gz
> tar -zxvf cache.tar.gz
> ```
> 然后再执行 `buildtool build -p core`。
>
> **Planning 模块专用缓存**：
> ```bash
> aem enter
> cd /apollo_workspace/
> wget https://apollo-pkg-beta.bj.bcebos.com/archive/planning_cache.tar.gz
> sudo rm -rf .cache
> tar -xzvf planning_cache.tar.gz
> buildtool build -p modules/planning/
> ```

### Step 5: 验证安装

```bash
# 启动 DreamView+
aem bootstrap start --plus
```

访问 `http://localhost:8888`，确认 DreamView+ 可正常打开。

常见启动问题：

| 问题 | 原因 | 解决方法 |
|------|------|----------|
| permission denied | 文件夹权限不足 | `sudo chown 用户名:用户名 -R <报错路径>` |
| 部分服务未启动 | 电脑性能问题 | 等待几十秒后重复执行 `aem bootstrap start --plus` |
| 依赖未完全下载 | 网络问题 | 重新执行 `buildtool build -p core` |

---

## 七、EDU 赛事专项步骤

以下步骤在基础安装完成后按需执行。

### 7.1 安装 Profile 插件（场景同步）

Profile 插件用于将 Apollo Studio 云端交通流场景同步到本地 Dreamview。

**步骤一：启动 Apollo 系统**
```bash
cd application-pnc
aem enter
aem bootstrap start --plus
```

**步骤二：生成 Profile 密钥**
1. 登录 Apollo Studio（apollo.baidu.com）
2. 点击【产品 > 仿真工具】或【工作台】
3. 在个人信息 > 服务权益 > 仿真 中点击【生成】
4. 点击【一键复制】复制密钥

> 密钥时效为 1 分钟，超时需重新生成。

**步骤三：安装插件**

在 Apollo Docker 环境的 `apollo_workspace` 目录下执行复制的密钥命令。

安装成功后验证：
```bash
ls ~/.apollo/dreamview/
```

**步骤四：验证插件**

重启 DreamView（插件在启动时加载）：
```bash
aem bootstrap stop
aem bootstrap start --plus
```

验证 studio_connector 节点：
```bash
cyber_node list
```
如果有 `studio_connector` 节点则安装成功。

### 7.2 Profile 配置管理

初始化配置：
```bash
buildtool profile config init --package planning --profile=default
```

常用操作：
```bash
aem profile use default     # 切换到 default 配置
aem profile list             # 查看所有配置
```

详细说明见知识库 `02_安装部署/06_aem_profile配置管理.md`。

### 7.3 赛事压缩包制作

**只修改了配置参数：**
```bash
cd application-pnc
aem enter
tar -zcvf 自己定义名字.tar.gz profiles/default
```

**修改了配置文件和源码：**
```bash
tar -zcvf 自己定义名字.tar.gz modules/planning/ profiles/default
```

> 测评系统仅读取 `profiles/default` 目录下的配置参数。

### 7.4 下载 Planning 代码包

```bash
# 下载所有 planning 代码
buildtool install planning*

# 下载特定包（如 crosswalk）
buildtool install planning-traffic-rules-crosswalk

# 编译 planning 模块
buildtool build -p modules/planning
```

---

## 八、故障诊断

当调用 `/apollo-env diagnose` 或安装过程中遇到错误时：

1. 先运行环境检测脚本获取当前状态
2. 在知识库 `07_FAQ故障排查/` 中检索匹配错误
3. 根据下表给出修复方案

| 问题 | 诊断方法 | 修复方案 |
|------|----------|----------|
| Docker 无法启动 | `systemctl status docker` | 检查守护进程日志 `journalctl -u docker` |
| Docker 权限不足 | `docker run hello-world` | `sudo usermod -aG docker $USER` |
| aem 命令找不到 | `which aem` | 检查 apt 源是否正确添加 |
| apt update 失败 | `apt-get update 2>&1` | 检查网络、GPG key 是否过期 |
| buildtool build 失败 | 查看错误日志 | 检查磁盘空间、网络；尝试编译缓存 |
| DreamView 无法访问 | `curl localhost:8888` | 检查 monitor 模块是否启动 |
| Docker 数据目录磁盘满 | `df -h $(docker info \| grep "Docker Root Dir" \| awk '{print $NF}')` | 清理 Docker 镜像或迁移数据目录 |
| DreamView 地图不加载 | 知识库检索 | 见 `07_FAQ故障排查/dreamview地图不加载或场景不跳转.md` |
| buildtool map get 失败 | 知识库检索 | 见 `07_FAQ故障排查/buildtool map get 地图获取失败.md` |
| **网络不通导致依赖下载失败** | `curl -sSf https://apollo-pkg-beta.cdn.bcebos.com` | 使用编译缓存离线包；GitHub 用 Gitee 镜像 |
| **日志占满磁盘** | `du -sh data/log/` | 清理带日期的历史日志：`find data/log/ -name "*.log.*20[0-9][0-9]*" -type f -delete` |
| **模块打不开** | `mainboard -d <dag路径>` 单独启动排查 | Planning: `mainboard -d /apollo/modules/planning/planning_component/dag/planning.dag`；Control: `mainboard -d /apollo/modules/control/control_component/dag/control.dag` |
| **Planning 代码改坏了** | 模块无法启动或编译报错 | 备份 `tar -zcvf planning_backup.tar.gz modules/planning/` → 删除 `rm -rf modules/planning/` → 重装包管理代码 `buildtool reinstall planning*` →（可选）拉源码 `buildtool install planning*` →（可选）编译 `buildtool build -p modules/planning/` |
| **编译后配置参数丢失** | 修改的 profile 参数编译后失效 | `profiles/default` 是仿真引擎读取的目录；编译时 modules 下的配置会覆盖 profiles 的软链。编译后执行 `aem profile use default` 恢复 |

所有修复命令需用户确认后才执行。

---

## 九、相关配套资源

| 资源 | 链接 |
|------|------|
| PnC 安装视频课程 | https://apollo.baidu.com/community/online-course/814 |
| 安装报错指南 | https://apollo.baidu.com/community/article/1040 |
| Profile 插件 FAQ | https://apollo.baidu.com/community/article/1051 |
| 场景 & 地图获取 | https://apollo.baidu.com/community/article/1242 |
| Planning 文档总览 | https://apollo.baidu.com/docs/apollo/latest/md_collection_2planning_2README__cn.html |

---

## 十、文件结构

```
apollo-edu-assistant/
├── _meta.json               # 元数据
├── config.yaml              # 版本要求和 URL 配置（单一事实来源）
├── SKILL.md                 # 技能定义（行为规范与流程概览）
├── USAGE.md                 # 详细安装手册（本文件）
├── scripts/
│   └── env_check.sh         # 环境检测脚本（支持 --json 输出）
└── references/
    ├── knowledge_index.md   # 知识库索引
    └── knowledge/           # 61 个知识库文档（9 个子目录）
```

### 脚本退出码

| 退出码 | 含义 |
|--------|------|
| 0 | 所有检测项通过 |
| 1 | 存在警告项（可选依赖未满足） |
| 2 | 存在失败项（必须依赖未满足） |
