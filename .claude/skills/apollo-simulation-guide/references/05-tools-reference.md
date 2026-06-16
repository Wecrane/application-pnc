# 05 - 工具参考：aem / buildtool / profile

> 来源：`apollo_docs_md/框架设计/软件核心/包管理工具/` 和 `apollo_docs_md/赛事总汇/`

---

## aem（Apollo Environment Manager）

Apollo 容器环境管理工具。

### 常用命令

```bash
aem start              # 启动 Apollo 容器（在宿主机执行）
aem enter              # 进入容器（在宿主机执行）
aem stop               # 停止容器
aem status             # 查看容器状态
aem bootstrap start    # 启动 Dreamview（基础版，在容器内）
aem bootstrap start --plus  # 启动 Dreamview+（推荐）
aem bootstrap stop     # 关闭 Dreamview
aem profile use default     # 切换 profile
aem profile list            # 列出所有 profile
```

### aem init outside from Apollo container env

如果在容器外执行 aem 命令报此错误，说明不在正确的工程目录下：
```bash
cd ~/application-pnc
aem <command>
```

---

## buildtool

Apollo 构建和包管理工具。

> **重要概念**：`buildtool install` 下载的是**源码**到 `modules/` 目录供你修改。Apollo 核心包已包含所有默认场景/规则/任务的**编译好的二进制**，即使不下载源码也能正常运行默认场景（如 LaneFollow）。只有需要**修改**某个模块时才需要 `buildtool install`。

### 常用命令

```bash
# 编译
buildtool build -p core -j15              # 编译核心包（首次/全量）
buildtool build -p modules/planning/ -j15  # 只编译 planning（改了源码用这个，快很多）

# 编译后必须恢复 profile，否则配置不生效！
aem profile use default

# 安装/下载源码
buildtool install planning*       # 下载全部 planning 子模块源码
buildtool install <package_name>  # 下载指定模块源码

# Profile 管理
buildtool profile config init --package planning --profile=default   # 初始化 profile

# 地图
buildtool map get <map_name>      # 下载地图
buildtool map list                # 列出可用地图
```

### buildtool build 报错

常见错误和解决方案：

| 错误 | 解决 |
|------|------|
| 首次编译失败 | 再执行一次（共两次） |
| 网络超时 | 检查网络，重试 |
| 内存不足 | 增加内存或 swap |
| 磁盘空间不足 | 清理日志 `find data/log/ -name "*.log.*" -delete` |

---

## Profile 配置管理

Profile 是 Apollo 的分级配置管理系统。

### 目录结构

```
profiles/
├── current -> default    # 当前激活的 profile（符号链接）
└── default/              # 默认 profile
    └── modules/
        └── planning/
            ├── scenario_config.pb.txt
            ├── traffic_rule_config.pb.txt
            ├── planning.conf
            └── scenarios/
```

### 常用操作

```bash
# 初始化 profile
buildtool profile config init --package planning --profile=default

# 切换 profile
aem profile use default

# 创建新 profile
cp -r profiles/default profiles/my_profile
aem profile use my_profile

# 检查当前 profile
readlink profiles/current
```

### 注意事项

- **编译后会重置 profile**：每次编译后需重新执行 `aem profile use default`
- **配置修改后需重启模块**：`aem bootstrap stop && aem bootstrap start --plus`

---

## 日志管理

### 日志位置

```
data/log/
├── planning.INFO        # Planning 模块日志
├── control.INFO         # Control 模块日志
├── dreamview.INFO       # Dreamview 日志
├── dreamview_plus.INFO  # Dreamview+ 日志
├── cyber_launch.INFO    # Cyber 启动日志
└── ...
```

### 常用操作

```bash
# 实时查看 Planning 日志
tail -f data/log/planning.INFO

# 清理历史日志（保留当前）
find data/log/ -name "*.log.*20[0-9][0-9]*" -type f -delete

# 查看最新日志
tail -100 data/log/planning.INFO
```

---

## 软件包管理详解

> 来源：`apollo_docs_md/框架设计/软件核心/包管理工具/`（B）

### 包管理核心概念

Apollo 使用自研的包管理系统，基于 `cyberfile.xml` 声明依赖关系。

```
项目结构：
application-pnc/
├── .aem/envroot/
│   ├── apollo/    → 挂载到容器内 /apollo（配置生效）
│   └── opt/       → 挂载到容器内 /opt/（软件包缓存）
├── modules/       → 可修改的源码目录
└── profiles/      → 配置目录
```

### aem 详解

aem (Apollo Environment Manager) 是容器管理工具，核心功能：

| 命令 | 功能 | 使用场景 |
|------|------|---------|
| `aem start` | 拉取镜像并启动容器 | 首次启动 / 电脑重启后 |
| `aem enter` | 进入容器 | 日常开发 |
| `aem stop` | 停止容器 | 需要释放资源 |
| `aem bootstrap start --plus` | 启动 Dreamview+ | 仿真验证 |
| `aem profile use <name>` | 切换配置 | 配置管理 |

### buildtool 详解

| 命令 | 功能 |
|------|------|
| `buildtool build -p core -j15` | 编译核心包（首次/全量） |
| `buildtool build -p modules/planning/ -j15` | **只编译 planning（改代码后首选）** |
| `buildtool install <pkg>` | 下载源码到 modules/ |
| `buildtool profile config init` | 初始化 profile |
| `buildtool map get <name>` | 下载地图 |

编译失败常见原因：
- 首次编译缓存未建立 → 再执行一次
- 内存不足 → 用 `buildtool build -p modules/planning/ -j15` 只编译 planning
- 源码未下载 → 先 `buildtool install planning*`
- 编译后配置不生效 → 忘了执行 `aem profile use default`

---

## Profile 故障排查

> 来源：`apollo_docs_md/赛事总汇/05_工程框架与工具/profile插件问题排查.md` 和 `赛事总汇/02_安装部署/06_aem_profile配置管理.md`（L）

### 常见问题

**Q: 修改了配置参数但不生效？**
1. 确认当前 profile：`readlink profiles/current`
2. 重新加载：`aem profile use default`
3. 重启 Dreamview：`aem bootstrap stop && aem bootstrap start --plus`

**Q: 编译后配置丢失？**
- 编译后会重置 `/apollo` 目录，需重新 `aem profile use default`

**Q: Profile 插件安装失败？**
- 参考：`apollo_docs_md/赛事总汇/05_工程框架与工具/profile插件问题排查.md`

**Q: 如何同时维护多份配置？**
```bash
cp -r profiles/default profiles/tuning_v1
aem profile use tuning_v1
```

---

## 日志管理
