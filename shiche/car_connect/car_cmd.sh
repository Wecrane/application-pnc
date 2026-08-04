#!/usr/bin/env bash
# =============================================================================
# car_cmd.sh — 在实车容器内执行命令（非交互）
# 用法：
#   bash car_cmd.sh 'aem profile use default'
#   bash car_cmd.sh 'buildtool build -p modules/planning/ -j15'
#   bash car_cmd.sh 'cat /apollo/modules/planning/planning_component/conf/planning.conf | grep speed_bump'
# 说明：优先用 aem exec；若实车 aem 不支持，改用交互方式：ssh car 后 aem enter
# =============================================================================
set -euo pipefail

PROJ="${PROJ_DIR:-apollo-edu-9.1.1}"
CMD="${1:-}"

if [ -z "${CMD}" ]; then
  echo "用法: bash car_cmd.sh '命令'"
  echo "示例:"
  echo "  bash car_cmd.sh 'aem profile use default'"
  echo "  bash car_cmd.sh 'buildtool build -p modules/planning/ -j15'"
  echo "  bash car_cmd.sh 'bash -lc \"ls /apollo\"'"
  exit 1
fi

if ! ssh -O check car >/dev/null 2>&1; then
  echo "⚠️  尚未建立免密 master 连接，请先执行 connect_car.sh 或 ssh car"
  exit 1
fi

echo "==> 实车容器内执行: ${CMD}"
ssh car "cd ~/${PROJ} && aem exec -- ${CMD}" 2>&1 \
  || { echo "❌ aem exec 失败，可能是实车 aem 不支持非交互执行"; \
       echo "   请改用: ssh car 然后 cd ~/${PROJ} && aem enter"; exit 1; }
