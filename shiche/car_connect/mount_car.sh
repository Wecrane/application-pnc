#!/usr/bin/env bash
# =============================================================================
# 挂载实车目录到本地文件管理器（SSHFS，基于 Host car 复用免密连接）
# -----------------------------------------------------------------------------
# 用法：
#   bash mount_car.sh           # 只挂 profiles/ + modules/（调参用，轻量推荐）
#   bash mount_car.sh --full    # 挂整个工程（浏览代码用，较大）
#   PROJ_DIR=xxx bash mount_car.sh   # 指定实车工程目录名（现场核实）
# 卸载：bash umount_car.sh
# =============================================================================
set -euo pipefail

PROJ="${PROJ_DIR:-apollo-edu_9.1.1}"   # ⚠️ 实车实测为下划线（890车确认）；仍可 PROJ_DIR 覆盖
MNT_ROOT="${HOME}/car"

# 1. 确保 master 连接存在（首次会要求密码）
if ! ssh -O check car >/dev/null 2>&1; then
  echo "⚠️ 尚未建立免密 master 连接"
  echo "   请先在另一个终端执行:  ssh car   并输入一次密码"
  echo "   建立成功后回到这里继续。"
  echo ""
  read -r -p "已建立连接后按回车继续，或 Ctrl+C 取消... " _
fi

# 2. 挂载
if [ "${1:-}" = "--full" ]; then
  mkdir -p "${MNT_ROOT}/root"
  sshfs car:"${PROJ}" "${MNT_ROOT}/root" \
    -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
  echo "✅ 全工程已挂载 → ${MNT_ROOT}/root  （浏览代码用，VSCode 打开前建议排除 bazel-* 等）"
else
  mkdir -p "${MNT_ROOT}/profiles" "${MNT_ROOT}/modules"
  sshfs car:"${PROJ}/profiles" "${MNT_ROOT}/profiles" \
    -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
  sshfs car:"${PROJ}/modules" "${MNT_ROOT}/modules" \
    -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
  echo "✅ profiles 已挂载 → ${MNT_ROOT}/profiles"
  echo "✅ modules  已挂载 → ${MNT_ROOT}/modules"
fi

echo ""
echo "👉 用文件管理器（Nautilus）打开: file://${MNT_ROOT}/ 即可查看/编辑"
echo "👉 或新开一个 VSCode 窗口打开:   ${MNT_ROOT}/profiles  （手动看代码）"
echo "👉 改动即实时写回实车；卸载用:   bash umount_car.sh"
