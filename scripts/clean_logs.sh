#!/bin/bash
# ============================================================
# Apollo PnC 日志清理脚本 — clean_logs.sh
# ============================================================
# 用法:
#   bash scripts/clean_logs.sh                      # 清理 7 天前的日志
#   bash scripts/clean_logs.sh --dry-run            # 预览模式
#   bash scripts/clean_logs.sh --all                # 清理所有日志（需确认）
#   bash scripts/clean_logs.sh --days 3             # 清理 3 天前的日志
#   bash scripts/clean_logs.sh --module planning    # 只清理 planning 模块
#   bash scripts/clean_logs.sh --size 50            # 删除大于 50MB 的日志
#   bash scripts/clean_logs.sh --keep-latest        # 每个模块只保留最新文件
#   bash scripts/clean_logs.sh --planning           # Planning 专项清理
#   bash scripts/clean_logs.sh --help               # 显示帮助
#
# 安全设计:
#   - 绝不删除 .INFO 符号链接（glog 写入需要）
#   - 绝不删除正在被写入的文件（fuser/lsof 检测）
#   - --all 操作需要输入 "yes" 确认
#   - 默认只清理 7 天前的文件
#
# Cron 示例（每天凌晨 3 点自动清理 7 天前的日志）:
#   0 3 * * * /bin/bash /apollo_workspace/scripts/clean_logs.sh --days 7 >> /tmp/log_cleanup.log 2>&1
# ============================================================

set -euo pipefail

# ────────────────────────────────────────
# 颜色定义
# ────────────────────────────────────────
if [[ -t 1 ]]; then
    COLOR_GREEN='\033[0;32m'
    COLOR_YELLOW='\033[0;33m'
    COLOR_RED='\033[0;31m'
    COLOR_BLUE='\033[0;34m'
    COLOR_CYAN='\033[0;36m'
    COLOR_BOLD='\033[1m'
    COLOR_RESET='\033[0m'
else
    COLOR_GREEN=''
    COLOR_YELLOW=''
    COLOR_RED=''
    COLOR_BLUE=''
    COLOR_CYAN=''
    COLOR_BOLD=''
    COLOR_RESET=''
fi

# ────────────────────────────────────────
# 路径检测
# ────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="${WORKSPACE_ROOT}/data/log"

# ────────────────────────────────────────
# 全局变量
# ────────────────────────────────────────
DRY_RUN=false
DAYS=7
MODULE=""
SIZE_MB=0
KEEP_LATEST=false
PLANNING_MODE=false
CLEAN_ALL=false
CONFIRMED=false

# 统计变量
FILES_BEFORE=0
FILES_DELETED=0
FILES_SKIPPED=0
FILES_ERROR=0
SIZE_BEFORE=0
SIZE_DELETED=0
SYMLINKS_PROTECTED=0
IN_USE_PROTECTED=0

# ────────────────────────────────────────
# 辅助函数
# ────────────────────────────────────────

print_green()  { echo -e "${COLOR_GREEN}$1${COLOR_RESET}"; }
print_yellow() { echo -e "${COLOR_YELLOW}$1${COLOR_RESET}"; }
print_red()    { echo -e "${COLOR_RED}$1${COLOR_RESET}"; }
print_blue()   { echo -e "${COLOR_BLUE}$1${COLOR_RESET}"; }
print_cyan()   { echo -e "${COLOR_CYAN}$1${COLOR_RESET}"; }
print_bold()   { echo -e "${COLOR_BOLD}$1${COLOR_RESET}"; }

# 检查文件是否正在被使用
is_file_in_use() {
    local file="$1"
    if command -v fuser &>/dev/null; then
        if fuser "$file" >/dev/null 2>&1; then
            return 0
        fi
    elif command -v lsof &>/dev/null; then
        if lsof "$file" >/dev/null 2>&1; then
            return 0
        fi
    fi
    return 1
}

# 获取文件大小（字节）
get_file_size() {
    local file="$1"
    stat -c%s "$file" 2>/dev/null || echo 0
}

# 格式化大小显示
format_size() {
    local bytes=$1
    if (( bytes >= 1073741824 )); then
        printf "%.2f GB" "$(echo "scale=2; $bytes / 1073741824" | bc -l 2>/dev/null || echo "0")"
    elif (( bytes >= 1048576 )); then
        printf "%.2f MB" "$(echo "scale=2; $bytes / 1048576" | bc -l 2>/dev/null || echo "0")"
    elif (( bytes >= 1024 )); then
        printf "%.2f KB" "$(echo "scale=2; $bytes / 1024" | bc -l 2>/dev/null || echo "0")"
    else
        echo "${bytes} B"
    fi
}

# 检查是否是 .INFO 符号链接
is_info_symlink() {
    local file="$1"
    local basename
    basename="$(basename "$file")"
    if [[ "$basename" =~ ^[a-z_]+\.[A-Z]+$ ]] && [[ ! "$basename" =~ \.log\. ]]; then
        return 0
    fi
    return 1
}

# 检查是否是日志文件
is_log_file() {
    local file="$1"
    local basename
    basename="$(basename "$file")"
    [[ "$basename" =~ \.log\. ]]
}

# 删除单个文件
delete_file() {
    local file="$1"

    if [[ -L "$file" ]]; then
        print_yellow "  [SKIP] 符号链接: $file"
        ((FILES_SKIPPED++))
        ((SYMLINKS_PROTECTED++))
        return 1
    fi

    if is_info_symlink "$file"; then
        print_yellow "  [SKIP] .INFO 链接（glog 需要）: $file"
        ((FILES_SKIPPED++))
        ((SYMLINKS_PROTECTED++))
        return 1
    fi

    if is_file_in_use "$file"; then
        print_yellow "  [SKIP] 文件正在使用中: $file"
        ((FILES_SKIPPED++))
        ((IN_USE_PROTECTED++))
        return 1
    fi

    local fsize
    fsize=$(get_file_size "$file")

    if $DRY_RUN; then
        print_blue "  [DRY-RUN] 将删除: $file ($(format_size $fsize))"
        ((FILES_DELETED++))
        ((SIZE_DELETED += fsize))
        return 0
    fi

    if rm -f "$file"; then
        print_green "  [DEL] $file ($(format_size $fsize))"
        ((FILES_DELETED++))
        ((SIZE_DELETED += fsize))
        return 0
    else
        print_red "  [ERR] 删除失败: $file"
        ((FILES_ERROR++))
        return 1
    fi
}

# ────────────────────────────────────────
# 帮助信息
# ────────────────────────────────────────
show_help() {
    cat << 'EOF'
Apollo PnC 日志清理脚本 — clean_logs.sh

用法:
  bash scripts/clean_logs.sh [选项]

选项:
  --days N           清理 N 天前的日志（默认: 7）
  --all              清理所有日志（需要输入 "yes" 确认）
  --dry-run          预览模式，只显示将删除的文件，不实际删除
  --module M         只清理指定模块的日志（如 planning, control）
  --size N           删除大于 N MB 的单个日志文件
  --keep-latest      每个模块只保留最新的一个日志文件
  --planning         Planning 专项清理（重建 data/log/planning/ 目录结构）
  --help             显示此帮助信息

示例:
  # 预览将删除的文件
  bash scripts/clean_logs.sh --dry-run

  # 清理 3 天前的日志
  bash scripts/clean_logs.sh --days 3

  # 只清理 planning 模块的日志
  bash scripts/clean_logs.sh --module planning

  # 删除大于 100MB 的日志文件
  bash scripts/clean_logs.sh --size 100

  # 每个模块只保留最新的一个日志文件
  bash scripts/clean_logs.sh --keep-latest

  # 清理所有日志（需要确认）
  bash scripts/clean_logs.sh --all

  # Planning 专项清理
  bash scripts/clean_logs.sh --planning

  # 组合使用
  bash scripts/clean_logs.sh --days 3 --module control --dry-run

安全说明:
  - .INFO 符号链接（如 planning.INFO）绝不会被删除
  - 正在被进程写入的文件不会被删除
  - --all 操作需要输入 "yes" 确认
EOF
}

# ────────────────────────────────────────
# 参数解析
# ────────────────────────────────────────
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                show_help
                exit 0
                ;;
            --dry-run)
                DRY_RUN=true
                shift
                ;;
            --all)
                CLEAN_ALL=true
                shift
                ;;
            --days)
                if [[ -z "${2:-}" ]] || [[ ! "$2" =~ ^[0-9]+$ ]]; then
                    print_red "错误: --days 需要一个正整数参数"
                    exit 1
                fi
                DAYS="$2"
                shift 2
                ;;
            --module)
                if [[ -z "${2:-}" ]]; then
                    print_red "错误: --module 需要一个模块名参数"
                    exit 1
                fi
                MODULE="$2"
                shift 2
                ;;
            --size)
                if [[ -z "${2:-}" ]] || [[ ! "$2" =~ ^[0-9]+$ ]]; then
                    print_red "错误: --size 需要一个正整数参数 (MB)"
                    exit 1
                fi
                SIZE_MB="$2"
                shift 2
                ;;
            --keep-latest)
                KEEP_LATEST=true
                shift
                ;;
            --planning)
                PLANNING_MODE=true
                shift
                ;;
            *)
                print_red "错误: 未知参数 '$1'"
                echo "使用 --help 查看帮助信息"
                exit 1
                ;;
        esac
    done
}

# ────────────────────────────────────────
# 确认操作
# ────────────────────────────────────────
confirm_action() {
    local prompt="$1"
    if $CONFIRMED; then
        return 0
    fi
    echo ""
    print_yellow "⚠  警告: $prompt"
    echo -n "请输入 'yes' 确认: "
    read -r response
    if [[ "$response" == "yes" ]]; then
        CONFIRMED=true
        return 0
    else
        echo "操作已取消。"
        return 1
    fi
}

# ────────────────────────────────────────
# 收集统计信息
# ────────────────────────────────────────
collect_stats_before() {
    local pattern="${1:-*.log.*}"
    local search_dir="${2:-$LOG_DIR}"

    if [[ ! -d "$search_dir" ]]; then
        return
    fi

    while IFS= read -r -d '' file; do
        if [[ -f "$file" ]] && [[ ! -L "$file" ]]; then
            if is_log_file "$file"; then
                ((FILES_BEFORE++))
                SIZE_BEFORE=$((SIZE_BEFORE + $(get_file_size "$file")))
            fi
        fi
    done < <(find "$search_dir" -maxdepth 1 -type f -name "$pattern" -print0 2>/dev/null || true)
}

# ────────────────────────────────────────
# 核心: 按天数清理
# ────────────────────────────────────────
clean_by_age() {
    local days="$1"
    local log_dir="${LOG_DIR}"

    print_bold ""
    print_bold "═══════════════════════════════════════════════════════════"
    print_bold "  日志清理: 删除 ${days} 天前的日志文件"
    print_bold "  日志目录: ${log_dir}"
    if $DRY_RUN; then
        print_blue  "  模式: DRY-RUN（预览，不实际删除）"
    fi
    print_bold "═══════════════════════════════════════════════════════════"
    echo ""

    local pattern="*.log.*"
    if [[ -n "$MODULE" ]]; then
        pattern="*${MODULE}*.log.*"
        print_cyan "  过滤模块: $MODULE"
    fi

    collect_stats_before "$pattern" "$log_dir"

    local count=0
    while IFS= read -r -d '' file; do
        if [[ -d "$file" ]]; then continue; fi
        if ! is_log_file "$file"; then continue; fi
        delete_file "$file"
        ((count++)) || true
    done < <(find "$log_dir" -maxdepth 1 -type f -name "$pattern" -mtime "+${days}" -print0 2>/dev/null || true)

    if [[ $count -eq 0 ]]; then
        print_cyan "  没有找到 ${days} 天前的日志文件。"
    fi
}

# ────────────────────────────────────────
# 核心: 清理所有日志
# ────────────────────────────────────────
clean_all_logs() {
    local log_dir="${LOG_DIR}"

    print_bold ""
    print_bold "═══════════════════════════════════════════════════════════"
    print_bold "  日志清理: 清理所有日志文件"
    print_bold "  日志目录: ${log_dir}"
    if $DRY_RUN; then
        print_blue  "  模式: DRY-RUN（预览，不实际删除）"
    fi
    print_bold "═══════════════════════════════════════════════════════════"
    echo ""

    local pattern="*.log.*"
    if [[ -n "$MODULE" ]]; then
        pattern="*${MODULE}*.log.*"
        print_cyan "  过滤模块: $MODULE"
    fi

    collect_stats_before "$pattern" "$log_dir"

    local count=0
    while IFS= read -r -d '' file; do
        if [[ -d "$file" ]]; then continue; fi
        if ! is_log_file "$file"; then continue; fi
        delete_file "$file"
        ((count++)) || true
    done < <(find "$log_dir" -maxdepth 1 -type f -name "$pattern" -print0 2>/dev/null || true)

    if [[ $count -eq 0 ]]; then
        print_cyan "  没有找到日志文件。"
    fi

    # 清理 planning 子目录（如果存在）
    local planning_subdir="${log_dir}/planning"
    if [[ -d "$planning_subdir" ]]; then
        print_bold ""
        print_bold "  清理 Planning 子目录..."
        while IFS= read -r -d '' file; do
            delete_file "$file"
        done < <(find "$planning_subdir" -type f -print0 2>/dev/null || true)
    fi
}

# ────────────────────────────────────────
# 核心: 按大小清理
# ────────────────────────────────────────
clean_by_size() {
    local size_mb="$1"
    local log_dir="${LOG_DIR}"

    print_bold ""
    print_bold "═══════════════════════════════════════════════════════════"
    print_bold "  日志清理: 删除大于 ${size_mb}MB 的日志文件"
    print_bold "  日志目录: ${log_dir}"
    if $DRY_RUN; then
        print_blue  "  模式: DRY-RUN（预览，不实际删除）"
    fi
    print_bold "═══════════════════════════════════════════════════════════"
    echo ""

    local pattern="*.log.*"
    if [[ -n "$MODULE" ]]; then
        pattern="*${MODULE}*.log.*"
        print_cyan "  过滤模块: $MODULE"
    fi

    collect_stats_before "$pattern" "$log_dir"

    local count=0
    while IFS= read -r -d '' file; do
        if [[ -d "$file" ]]; then continue; fi
        if ! is_log_file "$file"; then continue; fi
        delete_file "$file"
        ((count++)) || true
    done < <(find "$log_dir" -maxdepth 1 -type f -name "$pattern" -size "+${size_mb}M" -print0 2>/dev/null || true)

    if [[ $count -eq 0 ]]; then
        print_cyan "  没有找到大于 ${size_mb}MB 的日志文件。"
    fi
}

# ────────────────────────────────────────
# 核心: 只保留每个模块最新文件
# ────────────────────────────────────────
clean_keep_latest() {
    local log_dir="${LOG_DIR}"

    print_bold ""
    print_bold "═══════════════════════════════════════════════════════════"
    print_bold "  日志清理: 每个模块只保留最新的日志文件"
    print_bold "  日志目录: ${log_dir}"
    if $DRY_RUN; then
        print_blue  "  模式: DRY-RUN（预览，不实际删除）"
    fi
    print_bold "═══════════════════════════════════════════════════════════"
    echo ""

    collect_stats_before "*.log.*" "$log_dir"

    # 按模块名分组，保留最新的文件
    declare -A module_latest
    declare -A module_latest_time

    # 第一遍：找到每个模块的最新文件
    while IFS= read -r -d '' file; do
        if [[ -d "$file" ]]; then continue; fi
        if ! is_log_file "$file"; then continue; fi

        local basename
        basename="$(basename "$file")"
        # 提取模块名: planning.log.INFO.20260616-112833.622162 → planning
        local mod_name="${basename%%.log.*}"
        if [[ -z "$mod_name" ]]; then continue; fi

        # 如果指定了模块过滤
        if [[ -n "$MODULE" ]] && [[ "$mod_name" != "$MODULE" ]]; then
            continue
        fi

        local mtime
        mtime=$(stat -c%Y "$file" 2>/dev/null || echo 0)

        if [[ -z "${module_latest_time[$mod_name]:-}" ]] || (( mtime > module_latest_time[$mod_name] )); then
            module_latest_time[$mod_name]=$mtime
            module_latest[$mod_name]="$file"
        fi
    done < <(find "$log_dir" -maxdepth 1 -type f -name "*.log.*" -print0 2>/dev/null || true)

    # 第二遍：删除不是最新的文件
    local count=0
    while IFS= read -r -d '' file; do
        if [[ -d "$file" ]]; then continue; fi
        if ! is_log_file "$file"; then continue; fi

        local basename
        basename="$(basename "$file")"
        local mod_name="${basename%%.log.*}"

        if [[ -n "${module_latest[$mod_name]:-}" ]] && [[ "$file" == "${module_latest[$mod_name]}" ]]; then
            print_cyan "  [KEEP] 保留最新文件: $file"
            ((count++)) || true
        else
            delete_file "$file"
            ((count++)) || true
        fi
    done < <(find "$log_dir" -maxdepth 1 -type f -name "*.log.*" -print0 2>/dev/null || true)
}

# ────────────────────────────────────────
# 核心: Planning 专项清理
# ────────────────────────────────────────
clean_planning() {
    local log_dir="${LOG_DIR}"

    print_bold ""
    print_bold "═══════════════════════════════════════════════════════════"
    print_bold "  Planning 专项日志清理"
    print_bold "  日志目录: ${log_dir}"
    if $DRY_RUN; then
        print_blue  "  模式: DRY-RUN（预览，不实际删除）"
    fi
    print_bold "═══════════════════════════════════════════════════════════"
    echo ""

    # 1. 删除 planning.log.* 文件
    print_bold "  [1/3] 清理 planning.log.* 文件..."
    collect_stats_before "planning.log.*" "$log_dir"
    local count=0
    while IFS= read -r -d '' file; do
        if [[ -d "$file" ]]; then continue; fi
        delete_file "$file"
        ((count++)) || true
    done < <(find "$log_dir" -maxdepth 1 -type f -name "planning.log.*" -print0 2>/dev/null || true)

    # 2. 清理 data/log/planning/ 子目录
    local planning_subdir="${log_dir}/planning"
    if [[ -d "$planning_subdir" ]]; then
        print_bold ""
        print_bold "  [2/3] 清理 data/log/planning/ 目录内容..."
        while IFS= read -r -d '' file; do
            delete_file "$file"
            ((count++)) || true
        done < <(find "$planning_subdir" -type f -print0 2>/dev/null || true)
    fi

    # 3. 重建目录结构
    print_bold ""
    print_bold "  [3/3] 重建目录结构..."
    if ! $DRY_RUN; then
        mkdir -p "${planning_subdir}/trace"
        mkdir -p "${planning_subdir}/per_scenario"
        print_green "  [OK] 目录已重建: ${planning_subdir}/"
        print_green "  [OK] 目录已重建: ${planning_subdir}/trace/"
        print_green "  [OK] 目录已重建: ${planning_subdir}/per_scenario/"
    else
        print_blue "  [DRY-RUN] 将重建目录: ${planning_subdir}/trace/"
        print_blue "  [DRY-RUN] 将重建目录: ${planning_subdir}/per_scenario/"
    fi
}

# ────────────────────────────────────────
# 打印统计信息
# ────────────────────────────────────────
print_summary() {
    echo ""
    print_bold "═══════════════════════════════════════════════════════════"
    print_bold "  清理统计"
    print_bold "═══════════════════════════════════════════════════════════"
    if $DRY_RUN; then
        print_blue  "  (预览模式 — 未实际删除任何文件)"
    fi
    echo ""
    echo "  清理前文件数: ${FILES_BEFORE}"
    echo "  清理前总大小: $(format_size $SIZE_BEFORE)"
    echo ""
    print_green "  已删除文件:   ${FILES_DELETED}"
    print_green "  释放空间:     $(format_size $SIZE_DELETED)"
    echo ""
    print_yellow "  跳过文件:     ${FILES_SKIPPED}"
    echo "    - 符号链接:  ${SYMLINKS_PROTECTED}"
    echo "    - 使用中:    ${IN_USE_PROTECTED}"
    if (( FILES_ERROR > 0 )); then
        print_red "  删除失败:     ${FILES_ERROR}"
    fi
    echo ""
    echo "  剩余文件数: $((FILES_BEFORE - FILES_DELETED))"
    local remaining_size=$((SIZE_BEFORE - SIZE_DELETED))
    if (( remaining_size < 0 )); then remaining_size=0; fi
    echo "  剩余总大小: $(format_size $remaining_size)"
    print_bold "═══════════════════════════════════════════════════════════"
}

# ────────────────────────────────────────
# 主函数
# ────────────────────────────────────────
main() {
    parse_args "$@"

    # 检查日志目录是否存在
    if [[ ! -d "$LOG_DIR" ]]; then
        print_red "错误: 日志目录不存在: $LOG_DIR"
        exit 1
    fi

    # 执行清理策略
    if $CLEAN_ALL; then
        if ! $DRY_RUN; then
            confirm_action "即将删除所有日志文件！此操作不可逆。" || exit 0
        fi
        clean_all_logs
    elif $PLANNING_MODE; then
        clean_planning
    elif $KEEP_LATEST; then
        clean_keep_latest
    elif (( SIZE_MB > 0 )); then
        clean_by_size "$SIZE_MB"
    else
        # 默认: 按天数清理
        clean_by_age "$DAYS"
    fi

    print_summary
}

main "$@"
