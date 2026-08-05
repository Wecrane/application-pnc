# 实车连接工具包（car_connect）

**方案**：Ubuntu 笔记本 + 新版 VSCode + AI 助手，全程通过 SSH 操作实车，SSHFS 挂载到本地文件管理器。**不用 Windows、不用旧版 VSCode、不依赖 Remote-SSH**。

## 为什么不用官方 Remote-SSH

新版 VSCode 的 Remote-SSH 会在实车（Ubuntu 18.04，glibc 2.27）上安装 `vscode-server`，其要求 glibc ≥ 2.28 → 连接失败。**绕过办法：不装 vscode-server，用 sshfs 把文件挂到本地 + 用 ssh 终端执行命令**。

## 文件

| 脚本 | 作用 |
|------|------|
| `connect_car.sh` | **一键连接**：输入车辆编号 → 连隐藏Wi-Fi → 免密 → 自动挂载（现场主入口） |
| `setup_ssh_config.sh` | 仅免密配置（笔记本侧 ControlMaster，实车零改动，合规） |
| `mount_car.sh` | 仅挂载 `profiles/` + `modules/` 到 `~/car/`（`--full` 挂全工程） |
| `umount_car.sh` | 卸载 |
| `check_car.sh` | 实车健康检查（系统/磁盘/内存/工程目录/进程） |
| `apply_config.sh` | **改完配置一键生效**：`aem profile use default`（实车无需编译） |
| `car_cmd.sh` | 在实车容器内执行命令（查看配置等；编译仅改代码时需要） |
| `backup_car.sh` | 备份实车配置到本地（改配置前必做，回滚用） |

## 现场最快路径（30 分钟内）

### 一键连接（推荐）

```bash
bash shiche/car_connect/connect_car.sh 802
# 或交互式（会让你输入编号）：bash shiche/car_connect/connect_car.sh
# 脚本自动完成：连Wi-Fi → 等ping通 → 免密 → 探测工程目录名 → 挂载
```

### ② 连接后标准动作（实车只调参，**全程无需编译**）

```bash
bash shiche/car_connect/check_car.sh           # 1. 健康检查
bash shiche/car_connect/backup_car.sh          # 2. 备份原配置（回滚保险）
# 3. 调参：本工作区 AI 改配置（实时写回实车）或新开 VSCode 窗口改 ~/car/profiles
bash shiche/car_connect/apply_config.sh        # 4. 一键生效: aem profile use default
# 5. 测试 → 看评分 → 不达标微调 → 重复
# ⚠️ 无需编译！只有改 C++ 源码才需要 buildtool build（实车赛基本不会）
```

## 手动分步（脚本失败时的备用）

```bash
# 1. 连隐藏 Wi-Fi（nmcli hidden yes 是关键）
nmcli dev wifi connect "HQEV802" password "apollo123" hidden yes
ping -c 2 192.168.10.6

# 2. 免密配置（只改笔记本，实车零改动）
bash shiche/car_connect/setup_ssh_config.sh
ssh car                      # 首次输一次密码；ls ~ 确认工程目录名

# 3. 挂载（工程目录名不对时用 PROJ_DIR 指定）
bash shiche/car_connect/mount_car.sh
# PROJ_DIR=apollo-edu_9.1.1 bash shiche/car_connect/mount_car.sh

# 4. 卸载
bash shiche/car_connect/umount_car.sh
```

挂载点：`~/car/profiles`、`~/car/modules`、（`--full`）`~/car/root`

### 三种操作方式（按需选择）

**方式 A：AI 助手直接改（推荐）**
- 本工作区目录 + 当前 VSCode，让 AI（我）通过挂载路径编辑配置 = 实时写回实车
- AI 通过终端 `ssh car "..."` 执行编译/重启/查看

**方式 B：文件管理器 / 新开 VSCode 窗口手动看**
- 文件管理器打开 `file://~/car/`，或新开一个 VSCode 窗口打开 `~/car/profiles`
- 新窗口是本地文件夹，**不经过 Remote-SSH** → 绕过版本问题
- 手动改完即写回实车

**方式 C：终端直连**
```bash
ssh car                              # 交互式
ssh car "cd ~/apollo-edu-9.1.1 && aem exec -- cat /apollo/.../planning.conf"
```

### 配置生效 / 重启（AI 或终端）—— 无需编译

> **官方确认（2026-08-05）**：只改参数时，**改哪个模块就重启哪个模块**即可，**不用编译、不用 `aem profile use default`**！官方也允许改 control 代码（不只 planning）。

```bash
# 改完配置一键生效（推荐）：
bash shiche/car_connect/apply_config.sh planning   # 改 planning 参数后 → 重启 Planning
bash shiche/car_connect/apply_config.sh control    # 改 control 参数后  → 重启 Control
# 手动方式（Dreamview+ 模块栏）：把对应模块开关关掉再打开（stop → start）
# 若改了 C++ 源码编译后，才需要：
ssh car "cd ~/apollo-edu-9.1.1 && aem profile use default"
# 全部重启：bash shiche/car_connect/apply_config.sh --restart-all
# 浏览器：192.168.10.6:8888
# ⚠️ 实车只调参 → 不要执行 buildtool build（仅改 C++ 源码才需要，实车赛基本不会）
```

## 合规红线提醒（重要）

- 手机热点：秩序册书面写"严禁个人热点"，但**官方会议（2026-08-04）明确说明可用**，以官方说明为准 → 现场可连热点保持外网/AI 在线
- 免密用 ControlMaster：只改笔记本 `~/.ssh/config`，实车 SSH 配置/安全策略**零改动** → 符合"不得修改…SSH 配置及安全策略"
- sshfs 只装笔记本（已装），车上零安装 → 不违反"严禁安装未经允许软件"
- 只改实车 `profiles/` 配置（允许修改范围）；**非必要不动 `modules/` 源码**，实在无法实现才动源码+编译
- 备赛/在家用手机 USB 共享网络给 AI 完全没问题

## 现场检查清单

- [ ] 连上 HQEV 车载 Wi-Fi（密码 apollo123）
- [ ] `connect_car.sh` 一键完成（或分步验证：ssh car 免密成功、挂载可读）
- [ ] `ls ~/car/profiles` 能看到文件
- [ ] 新开 VSCode 窗口能打开 `~/car/profiles` 并编辑
- [ ] 备份原配置（backup_car.sh）
- [ ] 备好回滚：每处修改记录原值

## 备用（挂载失败时）

- 纯 SSH 终端 + `scp`/`rsync` 同步：
```bash
scp ~/本地文件 car:~/apollo-edu-9.1.1/profiles/... 
rsync -av ~/本地目录/ car:~/apollo-edu-9.1.1/profiles/
```
- 挂载卡住：`bash umount_car.sh` 后重挂
