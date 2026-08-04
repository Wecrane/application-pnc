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
