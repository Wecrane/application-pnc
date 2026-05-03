Apollo EDU 故障诊断流程。

执行步骤：
1. 运行 `bash .claude/scripts/env_check.sh` 获取当前环境状态
2. 根据用户描述的错误，在知识库 `07_FAQ故障排查/` 中检索匹配的问题
3. 参考 `.claude/scripts/USAGE.md` 第八章节的故障诊断表
4. 给出诊断结果和修复方案（所有修复命令需用户确认后执行）

常见问题诊断：
- Docker 无法启动：`systemctl status docker`
- Docker 权限不足：`sudo usermod -aG docker $USER`
- buildtool build 失败：检查磁盘空间和网络
- DreamView 无法访问：`curl localhost:8888`
- 模块打不开：`mainboard -d <dag路径>` 单独启动排查
- 编译后配置丢失：`aem profile use default`
