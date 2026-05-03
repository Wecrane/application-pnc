#!/bin/bash
#
# Apollo EDU 赛事提交包打包脚本
# 用法: bash scripts/package_submit.sh
#
# 打包内容：
#   - 改源码时:  modules/planning/ + profiles/<profile名>/
#   - 仅改配置时: profiles/<profile名>/
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$WORKSPACE_DIR"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 自动检测 profile 目录（排除 . 和 ..）
PROFILE_DIR=""
for d in profiles/*/; do
    name="$(basename "$d")"
    if [ "$name" != "." ] && [ "$name" != ".." ]; then
        PROFILE_DIR="profiles/$name"
        break
    fi
done

if [ -z "$PROFILE_DIR" ]; then
    error "未找到 profiles/ 下的 profile 目录！"
    exit 1
fi

info "检测到 profile 目录: $PROFILE_DIR"

# 检测 git 改动
info "检测当前代码改动..."

HAS_SRC_CHANGE=false
HAS_PROFILE_CHANGE=false

if git diff --quiet -- modules/planning/ 2>/dev/null; then
    warn "modules/planning/ 无改动"
else
    HAS_SRC_CHANGE=true
    info "检测到 modules/planning/ 有改动"
fi

if git diff --quiet -- "$PROFILE_DIR/" 2>/dev/null; then
    warn "$PROFILE_DIR/ 无改动"
else
    HAS_PROFILE_CHANGE=true
    info "检测到 $PROFILE_DIR/ 有改动"
fi

if [ "$HAS_SRC_CHANGE" = false ] && [ "$HAS_PROFILE_CHANGE" = false ]; then
    warn "未检测到任何改动，将打包当前所有内容"
fi

# 生成时间戳
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE_NAME="submit_${TIMESTAMP}.tar.gz"

info "正在打包..."

if [ "$HAS_SRC_CHANGE" = true ]; then
    # 改了源码 → 打包 modules/planning/ + profiles/
    info "打包方式：源码 + 配置"
    tar -zcvf "$ARCHIVE_NAME" modules/planning/ "$PROFILE_DIR"
elif [ "$HAS_PROFILE_CHANGE" = true ]; then
    # 仅改配置 → 打包 profiles/
    info "打包方式：仅配置"
    tar -zcvf "$ARCHIVE_NAME" "$PROFILE_DIR"
else
    # 无检测到改动 → 打包全部
    info "打包方式：全量（源码 + 配置）"
    tar -zcvf "$ARCHIVE_NAME" modules/planning/ "$PROFILE_DIR"
fi

echo ""
info "✅ 打包完成！"
info "   文件: ${WORKSPACE_DIR}/${ARCHIVE_NAME}"
info "   大小: $(du -h "$ARCHIVE_NAME" | cut -f1)"

# 列出包内容概览
echo ""
info "包内容概览："
tar -tvf "$ARCHIVE_NAME" | head -20
echo "    ...（共 $(tar -tvf "$ARCHIVE_NAME" | wc -l) 个文件）"
