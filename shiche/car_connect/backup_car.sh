#!/usr/bin/env bash
# =============================================================================
# backup_car.sh — 备份实车配置到本地（改配置前必做，回滚用）
# 用法：
#   bash backup_car.sh            # 默认备份 profiles（调参主要改这里，轻量）
#   bash backup_car.sh all        # 备份 profiles + modules（modules 很大，慢）
# 回滚示例：
#   scp -r ~/car_backup_时间戳/profiles/* car:~/apollo-edu-9.1.1/profiles/
# =============================================================================
set -euo pipefail

PROJ="${PROJ_DIR:-apollo-edu_9.1.1}"   # 实车实测为下划线
SCOPE="${1:-profiles}"
TS="$(date +%Y%m%d_%H%M%S)"
DST="${HOME}/car_backup_${TS}"

if ! ssh -O check car >/dev/null 2>&1; then
  echo "⚠️  尚未建立免密 master 连接，请先执行 connect_car.sh 或 ssh car"
  exit 1
fi

echo "==> 备份实车 ~/${PROJ} 到本地 ${DST} ..."
mkdir -p "${DST}"

if [ "${SCOPE}" = "profiles" ] || [ "${SCOPE}" = "all" ]; then
  echo "   - profiles/"
  scp -rq car:"~/${PROJ}/profiles" "${DST}/" || true
fi
if [ "${SCOPE}" = "modules" ] || [ "${SCOPE}" = "all" ]; then
  echo "   - modules/"
  scp -rq car:"~/${PROJ}/modules" "${DST}/" || true
fi

echo "✅ 备份完成: ${DST}"
echo "   回滚示例: scp -r ${DST}/profiles/* car:~/${PROJ}/profiles/"
