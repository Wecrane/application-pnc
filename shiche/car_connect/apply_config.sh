#!/usr/bin/env bash
# =============================================================================
# apply_config.sh — 改完配置一键生效（实车只调参，无需编译）
# 官方确认：只改参数 → 改哪个模块就重启哪个模块，不用编译、不用 aem profile use default！
#
# 用法：
#   bash apply_config.sh                  # 提示：改哪个模块就重启哪个模块
#   bash apply_config.sh planning         # 重启 Planning 模块（改 planning 参数后）
#   bash apply_config.sh control          # 重启 Control 模块（改 control 参数后）
#   bash apply_config.sh --restart-all    # 重启 Dreamview（全部模块）
# =============================================================================
set -euo pipefail

PROJ="${PROJ_DIR:-apollo-edu_9.1.1}"   # 实车实测为下划线
TARGET="${1:-}"

if ! ssh -O check car >/dev/null 2>&1; then
  echo "⚠️  尚未建立免密 master 连接，请先执行 connect_car.sh 或 ssh car"
  exit 1
fi

restart_module() {
  local mod="$1" dag="$2"
  echo "==> 重启 ${mod} 模块 ..."
  ssh car "cd ~/${PROJ} && aem exec -- cyber_launch stop ${dag} >/dev/null 2>&1 || true; aem exec -- cyber_launch start ${dag}"
  echo "✅ ${mod} 已重启"
}

case "$TARGET" in
  planning)
    restart_module "Planning" "/apollo/modules/planning/planning_component/launch/planning.launch"
    ;;
  control)
    restart_module "Control" "/apollo/modules/control/control_component/launch/control.launch"
    ;;
  --restart-all)
    echo "==> 重启 Dreamview（全部模块）..."
    ssh car "cd ~/${PROJ} && aem bootstrap stop && aem bootstrap start --plus"
    echo "✅ 已重启，浏览器打开 192.168.10.6:8888"
    ;;
  *)
    cat <<EOF
✅ 官方确认：只改参数时，改哪个模块就重启哪个模块，不用编译、不用 aem profile use default！

用法：
  bash apply_config.sh planning    # 改 planning 配置后 → 重启 Planning 模块
  bash apply_config.sh control     # 改 control 配置后  → 重启 Control 模块
  bash apply_config.sh --restart-all   # 或 Dreamview+ 模块栏手动关开对应模块

Dreamview+ 手动方式：左侧模块栏把对应模块开关关掉再打开（stop → start）即可。
验证：ssh car "aem exec -- cat /apollo/modules/planning/.../conf/default_conf.pb.txt"
EOF
    ;;
esac
