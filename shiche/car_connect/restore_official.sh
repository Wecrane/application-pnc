#!/usr/bin/env bash
# =============================================================================
# restore_official.sh — 恢复官方原始 planning + 安装限速插件（实车等价环境）
# -----------------------------------------------------------------------------
# 用途：把本地被仿真魔改过的 planning 恢复为官方原始代码（baseline-official
#       分支 = 官方仓库最新 planning），再安装主办方限速插件 scene_speed_limit，
#       并重置 profiles 为官方默认。用于搭建"官方代码+限速插件"的实车等价环境，
#       研究纯配置调参方案。
# 用法：在 application-pnc 根目录执行
#   bash shiche/car_connect/restore_official.sh              # 恢复+装插件+重置profile
#   bash shiche/car_connect/restore_official.sh --backup-only # 仅备份
# ⚠️ 会覆盖 modules/planning 全部本地修改 + 重置 profiles/default！
#    自动备份到 backup/realcar_YYYYMMDD_HHMMSS/，且 git 历史完整保留（可回滚）。
# =============================================================================
set -euo pipefail

BRANCH="${1:-baseline-official}"   # 官方基线分支
TS="$(date +%Y%m%d_%H%M%S)"
BK="backup/realcar_${TS}"
PLUGIN_SRC="shiche/_extracted/modules/planning/traffic_rules/scene_speed_limit"

# ---------- 1. 备份 ----------
echo "==> [1/5] 备份当前 planning + profiles → ${BK}"
mkdir -p "${BK}"
tar -zcf "${BK}/planning_current.tar.gz" modules/planning
tar -zcf "${BK}/profiles_default.tar.gz" profiles/default
echo "    已备份：${BK}/planning_current.tar.gz + profiles_default.tar.gz"

if [ "${BRANCH}" = "--backup-only" ]; then
  echo "✅ 仅备份完成，未做任何恢复。"
  exit 0
fi

# ---------- 2. 恢复官方 planning ----------
echo "==> [2/5] 恢复官方原始 planning（分支 ${BRANCH}）"
git rm -rq modules/planning
git checkout "${BRANCH}" -- modules/planning

# ---------- 3. 安装限速插件 ----------
echo "==> [3/5] 安装限速插件 scene_speed_limit"
if [ -d "${PLUGIN_SRC}" ]; then
  cp -r "${PLUGIN_SRC}" modules/planning/traffic_rules/
  echo "    已从 ${PLUGIN_SRC} 复制插件"
else
  echo "    ❌ 未找到插件源码（${PLUGIN_SRC}），跳过安装"
fi

# ---------- 4. 重置 profiles 为官方默认 ----------
echo "==> [4/5] 重置 profiles/default 为官方默认（去掉仿真调参覆盖）"
git rm -rq profiles/default/modules/planning 2>/dev/null || true
git checkout "${BRANCH}" -- profiles/default

# ---------- 5. 验证 ----------
echo "==> [5/5] 验证"
if [ -d "modules/planning/traffic_rules/scene_speed_limit" ]; then
  echo "    ✅ 限速插件已就位：modules/planning/traffic_rules/scene_speed_limit/"
  grep -E "enabled|limit_speed" \
    modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt || true
else
  echo "    ⚠️ 插件未就位"
fi
echo "    官方规划配置确认："
grep -E "speed_bump|default_cruise|upper_speed" \
  profiles/default/modules/planning/planning_component/conf/planning.conf || true

cat <<EOF

✅ 完成！当前 = 官方代码 + 限速插件 + 官方 profile
   下一步：
   - 若要在本地跑起来：aem enter 后 buildtool build -p modules/planning/ -j15（首次装插件需编译）
   - 研究纯配置方案：见技能 references/06-official-config-solutions.md
   - 回滚：git checkout 回原分支 或 解压 backup/realcar_*/ 备份
EOF
