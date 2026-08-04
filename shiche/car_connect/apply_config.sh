#!/usr/bin/env bash
# =============================================================================
# apply_config.sh — 改完配置一键生效（实车只调参，无需编译）
# 说明：实车赛只改 profiles/ 配置 → 无需 buildtool build！
#       只需 aem profile use default 激活配置 + 必要时重启 Dreamview。
# 用法：
#   bash apply_config.sh              # 激活 default profile + 提示重启
#   bash apply_config.sh --restart    # 激活 profile 并重启 Dreamview
# =============================================================================
set -euo pipefail

PROJ="${PROJ_DIR:-apollo-edu-9.1.1}"

if ! ssh -O check car >/dev/null 2>&1; then
  echo "⚠️  尚未建立免密 master 连接，请先执行 connect_car.sh 或 ssh car"
  exit 1
fi

echo "==> 激活 default profile（使配置生效）..."
ssh car "cd ~/${PROJ} && aem profile use default"
echo "✅ profile 已激活"

if [ "${1:-}" = "--restart" ]; then
  echo "==> 重启 Dreamview ..."
  ssh car "cd ~/${PROJ} && aem bootstrap stop && aem bootstrap start --plus"
  echo "✅ 已重启，浏览器打开 192.168.10.6:8888"
else
  cat <<EOF
👉 配置通常已生效；若未生效，重启 Dreamview：
   bash shiche/car_connect/apply_config.sh --restart
   或浏览器 192.168.10.6:8888 中重启相关模块
EOF
fi
