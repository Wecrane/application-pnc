#!/usr/bin/env bash
# =============================================================================
# 卸载实车挂载目录
# 用法：bash umount_car.sh
# =============================================================================
set -euo pipefail

MNT_ROOT="${HOME}/car"
ok=0
for m in profiles modules root; do
  if mountpoint -q "${MNT_ROOT}/${m}" 2>/dev/null; then
    fusermount -u "${MNT_ROOT}/${m}" && echo "✅ 已卸载 ${MNT_ROOT}/${m}"
    ok=1
  fi
done

if [ "${ok}" = "0" ]; then
  echo "ℹ️  没有已挂载的 car 目录（或已全部卸载）"
fi
