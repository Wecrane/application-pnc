# 实车连接工具包（car_connect）

**方案**：Ubuntu 笔记本 + 新版 VSCode + AI 助手，全程通过 SSH 操作实车，SSHFS 挂载到本地文件管理器。**不用 Windows、不用旧版 VSCode、不依赖 Remote-SSH**。

## 为什么不用官方 Remote-SSH

新版 VSCode 的 Remote-SSH 会在实车（Ubuntu 18.04，glibc 2.27）上安装 `vscode-server`，其要求 glibc ≥ 2.28 → 连接失败。**绕过办法：不装 vscode-server，用 sshfs 把文件挂到本地 + 用 ssh 终端执行命令**。

## 文件

| 脚本 | 作用 |
|------|------|
| `setup_ssh_config.sh` | 免密配置（笔记本侧 ControlMaster，实车零改动，合规） |
| `mount_car.sh` | 挂载实车 `profiles/` + `modules/` 到 `~/car/`（`--full` 挂全工程） |
| `umount_car.sh` | 卸载 |

## 使用流程（现场，30 分钟内）

### 第 1 步：连 Wi-Fi + 免密（约 2 分钟）

```bash
# 连车载 Wi-Fi（隐藏网络）：HQEV+车辆编号后三位，密码 apollo123
# Ubuntu 最稳方式（hidden yes 是关键）：
nmcli dev wifi connect "HQEV802" password "apollo123" hidden yes
ping -c 2 192.168.10.6    # 确认能通实车
# 详细教程（GUI/持久配置/排查）见技能 references/02-connection-guide.md 3.1.1

# 配置免密（写 ~/.ssh/config，只改笔记本，不碰实车）
bash shiche/car_connect/setup_ssh_config.sh

# 首次连接，输入一次密码（之后 4 小时免密）
ssh car
# 进入后确认工程目录真实名称（下划线还是横杠）
ls ~
# 退出
exit
```

### 第 2 步：挂载（约 1 分钟）

```bash
# 确认工程目录名后（如为 apollo-edu_9.1.1）：
PROJ_DIR=apollo-edu_9.1.1 bash shiche/car_connect/mount_car.sh
# 若默认名 apollo-edu-9.1.1 正确则直接：
bash shiche/car_connect/mount_car.sh
```

挂载点：
- `~/car/profiles` ← 实车 `apollo-edu-9.1.1/profiles`
- `~/car/modules`  ← 实车 `apollo-edu-9.1.1/modules`
- （`--full` 额外挂 `~/car/root` 全工程，浏览代码用）

### 第 3 步：三种操作方式（按需选择）

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
ssh car "cd ~/apollo-edu-9.1.1 && aem enter -- buildtool build -p modules/planning/ -j15"
```

### 第 4 步：编译/重启/测试（AI 或终端）

```bash
ssh car "cd ~/apollo-edu-9.1.1 && aem enter -- buildtool build -p modules/planning/ -j15"
ssh car "cd ~/apollo-edu-9.1.1 && aem profile use default"   # 若需
# Dreamview 浏览器：192.168.10.6:8888
```

## 合规说明

- **免密用 ControlMaster**：只改笔记本 `~/.ssh/config`，实车 SSH 配置/安全策略**零改动** → 符合秩序册"不得修改…SSH 配置及安全策略"
- sshfs 只装笔记本（已装），车上零安装 → 不违反"严禁安装未经允许软件"
- 只改实车 `profiles/` 和 `modules/`（允许修改范围）

## 现场检查清单

- [ ] 连上 HQEV 车载 Wi-Fi（密码 apollo123）
- [ ] `setup_ssh_config.sh` 后 `ssh car` 输一次密码成功
- [ ] `ls ~` 确认工程目录名（下划线/横杠）
- [ ] `mount_car.sh` 挂载成功，`ls ~/car/profiles` 能看到文件
- [ ] 新开 VSCode 窗口能打开 `~/car/profiles` 并编辑
- [ ] 备好回滚：改前记录原值 / 备份 conf

## 备用（挂载失败时）

- 纯 SSH 终端 + `scp`/`rsync` 同步：
```bash
scp ~/本地文件 car:~/apollo-edu-9.1.1/profiles/... 
rsync -av ~/本地目录/ car:~/apollo-edu-9.1.1/profiles/
```
- 挂载卡住：`bash umount_car.sh` 后重挂
