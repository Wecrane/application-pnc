# 02 实车连接与自动驾驶闭环操作指南

> 来源：`shiche/附录一_智能驾驶实车赛_笔记本远程实车连接操作手册.md` + `shiche/自动驾驶闭环文档.md`

---

## 1. 总体适用场景

- 车辆和车载计算单元由组委会赛前统一准备，计算单元**未连接显示器**
- 选手用自备笔记本连接**车载 Wi-Fi**，通过 **SSH 或 VS Code Remote-SSH** 远程登录车载计算单元，完成 Apollo 工程调试
- 比赛开始后直接连接车载 Wi-Fi 并登录；发现未就绪**报告裁判**，不得自行启动/断电/重启设备

## 2. 赛前软件安装（个人电脑）

| 软件 | 要求 |
|------|------|
| SSH Client | 各系统自带即可；验证 `ssh -V` |
| VS Code | **1.96**（组委会指定版本，https://code.visualstudio.com/updates/v1_96 或网盘下载） |
| Remote-SSH 扩展 | 发布者为 Microsoft，与 1.96 兼容 |

- 赛前完成安装和连接测试；**现场不提供临时安装/升级/降级服务**
- 现场不得临时下载、安装、升级、降级或更换 VS Code 及扩展

### 2.1 Windows 安装 SSH（如缺失）

```powershell
# 管理员 PowerShell 检查
Get-WindowsCapability -Online | Where-Object Name -like 'OpenSSH*'
# 安装
Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
# 验证（重开普通 PowerShell）
ssh -V
```

### 2.2 Ubuntu 安装 SSH

```bash
sudo apt update
sudo apt install openssh-client
ssh -V
```

## 3. 现场连接步骤

### 3.1 连接车载 Wi-Fi

- Wi-Fi 是**隐藏网络**，需手动添加
- **名称**：`HQEV` + 车辆编号后三位（如车辆 0802 → `HQEV802`）
- **密码**：`apollo123`
- 实际车辆编号以本赛位车辆铭牌、现场公布信息或裁判指引为准

### 3.2 SSH 登录

```bash
ssh apollo@192.168.10.6
# 密码：apollo（输入不显示字符）
```

登录后检查：
```bash
whoami      # 应为 apollo
hostname    # 应为本赛位车辆主机
pwd
```

### 3.3 进入 Apollo 工程与容器

```bash
cd ~/apollo-edu-9.1.1      # 工程名以现场为准（有的车是 apollo-edu-9.1.1 等）
pwd                        # 确认目录正确！
aem enter                  # 进入容器；失败则先 aem start 再 aem enter
```

- ⚠️ **严禁进入错误的 Apollo 工程目录修改**——配置不生效、测试异常甚至影响车辆
- ⚠️ 不得在错误目录执行 `aem start`，不得反复启动/强制终止容器；异常时保留错误信息报告技术人员

### 3.4 完整命令示例

```bash
ssh apollo@192.168.10.6
cd ~/apollo-edu-9.1.1
aem enter
aem bootstrap start --plus    # 启动 Dreamview+
```

## 4. 自动驾驶闭环操作（Dreamview+）

> 适用：车辆启动完毕（红旗车：钥匙放车内、踩刹车按启动键；计算单元：按主副驾中间启动按钮）

1. 浏览器打开 `192.168.10.6:8888`（计算单元直连屏幕则 `localhost:8888`）
2. **选择 vehicle test 模式**，勾选 Accept the User Agreement and Privacy Policy，点 Enter this mode
   - ⚠️ **必须勾选 vehicle test 模式**，否则需重启所有模块，影响比赛时间
3. 高精地图栏选择相应地图
4. **模块栏依次启动**：
   `Gnss Driver` → `RtkLocalization` → `Transform` → `LidarDriver` → `LidarPerception` → `Prediction` → `Planing` → `ExternalCmd` → `Canbus` → `SafetyManager` → `TaskManager`
   - 注意：若车不在地图内，先开进地图（感知会屏蔽道路外障碍物）
5. 车辆可视化 → 点"路由编辑" → 设置途径点 → "保存编辑" → 点左下角"发送路由"（生成路线+规划轨迹）
6. 系好主副驾安全带、关好 4 门、档位切 N 档 → 点左下角"自动驾驶"按钮
   - 状态由"自动驾驶"→"运行中"
7. 完成自动驾驶后及时接管（状态回"自动驾驶"）
8. ⚠️ 全程关注车辆状态，异常时安全员立即**踩刹车或转方向盘接管**

## 5. 常见 SSH 问题排查

| 现象 | 处理 |
|------|------|
| `ssh` 命令不存在 | 检查是否安装 SSH Client（现场不得临时安装） |
| `Connection timed out` | 检查是否正确连接车载 Wi-Fi、计算单元是否启动、IP 是否正确 |
| `Connection refused` | 检查计算单元状态和 SSH 服务，报告技术人员 |
| `Permission denied` | 核对用户名密码，不得反复尝试其他账号 |
| 登录主机信息不一致 | 立即退出 SSH，报告裁判 |

Wi-Fi 连接异常自查：车辆编号后三位、名称 HQEV 开头、密码 apollo123、计算单元已启动；仍不行报告裁判。

## 6. 安全与纪律（现场）

- 只连本赛位车载 Wi-Fi 和计算单元
- 不修改操作系统、网络配置、账号权限、SSH 配置、容器基础环境、测评脚本、评分逻辑、地图等未授权文件
- 不重启/断电/刷写/格式化/拆卸/维修车辆和计算单元
- 异常立即停止并报告
- 比赛结束立即退出远程连接并停止一切操作

## 7. 备选连接方案：Ubuntu 笔记本 + 新版 VSCode + AI 助手（不依赖官方旧版 VSCode）

> 适用：选手笔记本是 Ubuntu（新版），想继续用新版 VSCode 和 AI 助手，不切换到 Windows 或旧版 VSCode。
> **核心结论：完全可行。** 官方 Remote-SSH 连不上 18.04 的原因是车上 vscode-server 需要 glibc≥2.28（18.04 只有 2.27）——只要不依赖 Remote-SSH 扩展，改用**文件挂载（SSHFS）+ 终端 SSH**，新版 VSCode 和 AI 都能用，且车上零安装、不违反红线。

### 7.1 为什么新版 VSCode 连不上

- 新版 VSCode 的 Remote-SSH 会在车上自动装 `vscode-server`，其要求 **glibc ≥ 2.28**
- 实车 Ubuntu 18.04 glibc = 2.27 → 服务端装不上 → 连接失败
- 组委会让装旧版 VSCode（1.96）正是因为旧版服务端兼容 18.04
- **绕过思路**：不用 Remote-SSH 扩展，用 sshfs 把文件挂到本地 + 用 ssh 终端执行命令

### 7.2 方案 A：SSHFS 挂载（推荐，首选）

**合规性**：sshfs 只安装在**笔记本**，通过车上已有的 sshd/SFTP 传输，**车上不装任何软件** → 不违反"严禁安装未经允许软件"红线。等价于官方 Remote-SSH 的数据通路。

```bash
# 1. 笔记本安装 sshfs（Ubuntu 24.04 用 fuse3）
sudo apt update && sudo apt install -y sshfs

# 2. 建挂载点，只挂允许修改的两个目录（轻量，避免挂全工程卡顿）
mkdir -p ~/car/profiles ~/car/modules
sshfs apollo@192.168.10.6:/home/apollo/apollo-edu-9.1.1/profiles ~/car/profiles \
  -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
sshfs apollo@192.168.10.6:/home/apollo/apollo-edu-9.1.1/modules ~/car/modules \
  -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
```

> ⚠️ 现场先 `ls ~` 确认工程目录真实名称（附录写 `apollo-edu-9.1.1`，秩序册一处写 `apollo-edu_9.1.1`，现场核实）。
> 挂载其他子目录（如 conf）同理；不建议挂全工程（bazel-* 符号链接/构建产物会拖慢 VSCode 索引）。

**使用**：
- 本地新版 VSCode 打开 `~/car/profiles` → 完整 AI 支持（记忆库/技能都在本地）
- AI/选手编辑挂载路径下的文件 = **实时写回实车**
- 卸载：`fusermount -u ~/car/profiles`

### 7.3 方案 B：AI 全程 SSH 终端操作（兜底）

不依赖挂载，AI 在终端直接 SSH 操作实车：

```bash
# 一次性命令（编译等）
ssh apollo@192.168.10.6 "cd ~/apollo-edu-9.1.1 && aem enter -- buildtool build -p modules/planning/ -j15"
# 交互式容器会话 → 用后台持久终端（async）保持
ssh apollo@192.168.10.6
```

文件编辑：`scp`/`rsync` 双向同步，或终端 heredoc 写入。

### 7.4 混合工作流（推荐）

```
本地新版 VSCode + AI 助手 ──SSHFS 挂载实时读写──▶ 实车 profiles/ + modules/
本地新版 VSCode + AI 助手 ──SSH 终端 aem enter──▶ 实车容器内 build/重启/测试
```

1. 挂载后本地 VSCode 打开挂载目录 → AI 改配置（带完整记忆库/技能）
2. 终端 `ssh` 上去 `aem enter` 编译、`aem profile use default`、重启 Dreamview
3. 车上 Dreamview 浏览器操作（`192.168.10.6:8888`）照旧

### 7.5 注意事项

- SSHFS 走车载 Wi-Fi，性能对配置文件足够；大文件/大目录操作会慢
- 挂载中途断线：`-o reconnect` 会自动重连；卡住可用 `fusermount -u` 强制卸载重挂
- 备好方案 B（scp/rsync）作为挂载失败兜底
- 现场时间紧张（30 分钟）：先确认挂载通、VSCode 能编辑，再开始调参
- 如需 AI 编译/重启：用 `ssh ... "aem enter -- ..."` 一次性命令最稳；交互式用后台持久终端
