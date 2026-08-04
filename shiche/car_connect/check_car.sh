#!/usr/bin/env bash
# =============================================================================
# check_car.sh — 实车健康检查（连接后执行）
#   系统信息 / 磁盘 / 内存 / 工程目录 / 关键进程
# 用法：bash check_car.sh
# =============================================================================
set -euo pipefail

if ! ssh -O check car >/dev/null 2>&1; then
  echo "⚠️  尚未建立免密 master 连接，请先执行 connect_car.sh 或 ssh car"
  exit 1
fi

ssh car bash -s <<'EOF'
echo "===== 系统 ====="
grep PRETTY /etc/os-release 2>/dev/null || cat /etc/os-release 2>/dev/null | head -2
uname -r
uptime | sed 's/^ *//'
echo "===== 磁盘 ====="
df -h "$HOME" | tail -1
echo "===== 内存 ====="
free -h | sed -n '1,2p'
echo "===== 工程目录 ====="
ls -d "$HOME"/apollo* 2>/dev/null || echo '(未找到 apollo 工程目录)'
echo "===== 关键进程 ====="
ps -eo comm | grep -E 'dreamview|planning|prediction|perception|canbus|task_manager' | sort -u || echo '(无关键进程)'
EOF
