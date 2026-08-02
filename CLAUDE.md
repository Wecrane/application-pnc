# Apollo EDU PnC 赛事工程 — Claude Code 项目指令

本项目为百度 Apollo 星火自动驾驶大赛 PnC（Planning & Control）赛道的赛事工程。

## 技能 (Skills)

Claude Code 专属技能位于 `.claude/skills/`，提供 Planning 模块的深度代码导航、赛题分析和日志回放排障：

| Skill | 路径 | 用途 |
|-------|------|------|
| Apollo 仿真开发指南 | `.claude/skills/apollo-simulation-guide/SKILL.md` | Planning 架构（17 Scenario + 10 TrafficRule + 35 Task）、插件开发、配置链路、参数调优、地图/路由/障碍物解析 |
| 参考文档 | `.claude/skills/apollo-simulation-guide/references/` | 14 份技术参考：架构、场景、交通规则、任务、算法、工具链、CyberRT、PnC Map、日志、地图路由等 |
| **日志与回放分析** | `.claude/skills/log-replay-analysis/SKILL.md` | **本地日志/本地回放/云端回放/云端日志的架构与结构、sim_engine 评分解析（唯一计分帧）、评测时间轴映射、关键日志模式、排障工作流**（800 分实战沉淀） |
| 分析实操手册 | `.claude/skills/log-replay-analysis/references/01-analysis-playbook.md` | sim_engine 解析代码、时间轴 T0 校准、planning 日志提取、云端回放解密实操 |

当用户提出 Planning 代码导航、插件开发、赛题分析等问题时，优先检索 `.claude/skills/apollo-simulation-guide/references/`。
当用户提出日志分析、回放分析、云端评测、评分解析、sim_engine、计分帧、时间轴等问题时，使用 `.claude/skills/log-replay-analysis/`。

## 知识库

Apollo EDU 赛事知识库位于 `.claude/apollo-knowledge/`，共 61 个文档，覆盖 9 个领域。当用户提出 Apollo 相关问题时，优先检索该知识库。

知识库索引：`.claude/apollo-knowledge/knowledge_index.md`

## 环境感知

回答问题前，先判断用户当前环境：

| 信号 | 判断方法 | 结论 |
|------|----------|------|
| 提示符含 `in-dev-docker` 或 pwd 为 `/apollo_workspace` | 观察终端或 `pwd` | Apollo 容器内，跳过 Docker/aem 安装 |
| 存在 `/.dockerenv` | `test -f /.dockerenv` | 容器内 |
| `aem` 和 `docker` 均可执行 | `command -v aem && command -v docker` | 宿主机，已部分安装 |
| 均不可用 | — | 全新环境，需完整安装 |

## 问题分流

| 问题类型 | 处理方式 |
|---------|---------|
| 安装 Apollo、配置环境 | 交互式安装引导（参考 `.claude/scripts/USAGE.md`） |
| `/apollo-env` / `/apollo-env check` | 运行环境检测脚本 |
| `/apollo-env diagnose` | 故障诊断流程 |
| 编译报错、buildtool 失败 | 先查高频快答，未命中则检索知识库 `07_FAQ故障排查/` |
| PnC 开发、场景调试、FAQ | 知识库检索 |

## 高频问题快答

以下问题无需检索知识库，直接回答：

- **电脑重启后进入 Apollo**：`cd application-pnc && aem start && aem enter`，然后 `buildtool build -p core`
- **打包提交**：只改配置 → `tar -zcvf 提交包.tar.gz profiles/default`；改了源码 → `tar -zcvf 提交包.tar.gz modules/planning/ profiles/default`
- **DreamView 打开**：`aem bootstrap start --plus`，浏览器访问 `http://localhost:8888`
- **buildtool 编译次数**：建议执行两次 `buildtool build -p core`
- **下载 Planning 代码**：`buildtool install planning*`（全量）或 `buildtool install planning-traffic-rules-crosswalk`（指定模块）
- **profile 初始化**：`buildtool profile config init --package planning --profile=default`
- **日志清理**：`find data/log/ -name "*.log.*20[0-9][0-9]*" -type f -delete`
- **编译后配置丢失**：编译后执行 `aem profile use default` 恢复

## 知识库检索流程

1. **优先按文件名匹配**：赛事场景有固定名称，先用 Glob 搜索（如用户问"借道绕行"，Glob `*借道*`）
2. **未命中时 Grep 搜索**：根据问题类型定位子目录（见知识库索引），在 `.claude/apollo-knowledge/` 中检索
3. **多关键词重试**：如"安装"→"部署"、"报错"→"error"
4. **未找到**：基于 Apollo EDU 领域知识作答，注明「本地知识库未找到直接记录」

## 回答格式

- FAQ/报错类 → 症状 → 原因 → 命令
- 场景开发类 → 核心代码逻辑 + 配置片段
- 安装流程类 → 按步骤编号列出命令
- 每条回答末尾注明来源文件

## 重要约束

- **🚫 禁止编译！**：**绝对不要执行任何编译命令**（`buildtool build`、`bazel build` 等），用户手动编译。你只负责代码编写和修改，编译由用户自行完成。
- **sudo 操作必须确认**：所有涉及 `sudo` 的命令须先展示给用户
- **不跳过步骤**：安装流程严格按顺序
- **幂等性**：每步执行前先检测是否已完成
- **PnC 赛道专注**：不需要 GPU，不涉及感知/定位模块
- **编译后恢复 profile**：提醒用户编译后自行执行 `aem profile use default`

## 环境检测脚本

```bash
bash .claude/scripts/env_check.sh          # 标准输出
bash .claude/scripts/env_check.sh --json   # JSON 输出
```

退出码：0=全部通过，1=存在警告，2=存在失败项

## 配置参考

版本要求和 URL 配置：`.claude/scripts/config.yaml`
详细安装手册：`.claude/scripts/USAGE.md`
