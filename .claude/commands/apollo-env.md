运行 Apollo EDU 环境检测脚本，根据结果引导用户完成安装配置。

执行步骤：
1. 运行 `bash .claude/scripts/env_check.sh`，收集系统信息
2. 对每项标注状态（✅通过 / ⚠️警告 / ❌必须缺失）
3. 如果所有必须项通过，提示可进入下一步
4. 如果有缺失项，按 `.claude/scripts/USAGE.md` 中的安装流程引导（每步需用户确认）
5. 安装完成后再次运行检测验证

参数：$ARGUMENTS（可选：check / install / diagnose）
- 无参数或 check：仅检测并报告
- install：跳过检测，直接安装缺失项
- diagnose：故障诊断模式
