#!/bin/bash
# ============================================================
# planlog v0.1 — Apollo PnC 日志快速过滤工具
# 用法: bash scripts/planlog.sh <日志文件> [选项]
# ============================================================
# 基于 grep/awk 的轻量级日志过滤工具（Phase 1 临时方案）
#
# 示例:
#   planlog.sh data/log/planning.INFO --scenario lane_follow
#   planlog.sh data/log/planning.INFO --frame 12345
#   planlog.sh data/log/planning.INFO --errors
#   planlog.sh data/log/planning.INFO --timeline
#   planlog.sh data/log/planning.INFO --frames
#   planlog.sh data/log/planning.INFO --level ERROR,WARN
#   planlog.sh data/log/planning.INFO --from "09:00:00" --to "09:05:00"
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
# 全局变量
# ────────────────────────────────────────
LOG_FILE=""
SCENARIO=""
FRAME=""
LEVEL=""
FROM_TIME=""
TO_TIME=""
SHOW_ERRORS=false
SHOW_TIMELINE=false
SHOW_FRAMES=false

# ────────────────────────────────────────
# 帮助信息
# ────────────────────────────────────────
show_help() {
    cat << 'EOF'
planlog v0.1 — Apollo PnC 日志快速过滤工具

用法:
  bash scripts/planlog.sh <日志文件> [选项]
  bash scripts/planlog.sh [选项]                    # 默认使用 data/log/planning.INFO

选项:
  --scenario NAME     按场景过滤日志
  --frame N           提取指定帧的完整日志
  --level LEVELS      按日志级别过滤 (INFO,WARN,ERROR,FATAL, 逗号分隔)
  --from TIME         起始时间过滤 (格式: HH:MM:SS)
  --to TIME           结束时间过滤 (格式: HH:MM:SS)
  --errors            错误统计摘要（按错误模式分组计数）
  --timeline          场景切换 ASCII 时间线
  --frames            帧边界提取（帧开始/结束 + 耗时）
  --help              显示此帮助信息

示例:
  # 按场景过滤
  planlog.sh data/log/planning.INFO --scenario lane_follow

  # 提取第 12345 帧的完整日志
  planlog.sh data/log/planning.INFO --frame 12345

  # 只看 ERROR 和 WARN
  planlog.sh data/log/planning.INFO --level ERROR,WARN

  # 时间范围过滤
  planlog.sh data/log/planning.INFO --from "09:00:00" --to "09:05:00"

  # 错误统计
  planlog.sh data/log/planning.INFO --errors

  # 场景切换时间线
  planlog.sh data/log/planning.INFO --timeline

  # 帧边界提取
  planlog.sh data/log/planning.INFO --frames

  # 组合使用
  planlog.sh data/log/planning.INFO --scenario stop_sign --level ERROR

日志格式:
  I0616 09:52:48.198967 50423 scenario_manager.cc:68] message...
  │  │  │             │     │                      │
  │  │  │             │     │                      └─ 消息体
  │  │  │             │     └─ 文件名:行号
  │  │  │             └─ 线程ID
  │  │  └─ 微秒时间戳
  │  └─ 月日
  └─ 级别: I=INFO W=WARNING E=ERROR F=FATAL
EOF
}

# ────────────────────────────────────────
# 解析日志级别前缀
# ────────────────────────────────────────
level_to_prefix() {
    case "$1" in
        INFO)    echo "I" ;;
        WARN)    echo "W" ;;
        ERROR)   echo "E" ;;
        FATAL)   echo "F" ;;
        *)       echo "" ;;
    esac
}

build_level_pattern() {
    local levels="$1"
    local pattern=""
    IFS=',' read -ra LEVEL_ARRAY <<< "$levels"
    for lvl in "${LEVEL_ARRAY[@]}"; do
        local p
        p=$(level_to_prefix "$lvl")
        if [[ -n "$p" ]]; then
            if [[ -z "$pattern" ]]; then
                pattern="^${p}"
            else
                pattern="${pattern}|^${p}"
            fi
        fi
    done
    echo "$pattern"
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
            --scenario)
                if [[ -z "${2:-}" ]]; then
                    echo -e "${COLOR_RED}错误: --scenario 需要一个场景名参数${COLOR_RESET}"
                    exit 1
                fi
                SCENARIO="$2"
                shift 2
                ;;
            --frame)
                if [[ -z "${2:-}" ]] || [[ ! "$2" =~ ^[0-9]+$ ]]; then
                    echo -e "${COLOR_RED}错误: --frame 需要一个正整数参数${COLOR_RESET}"
                    exit 1
                fi
                FRAME="$2"
                shift 2
                ;;
            --level)
                if [[ -z "${2:-}" ]]; then
                    echo -e "${COLOR_RED}错误: --level 需要日志级别参数${COLOR_RESET}"
                    exit 1
                fi
                LEVEL="$2"
                shift 2
                ;;
            --from)
                if [[ -z "${2:-}" ]]; then
                    echo -e "${COLOR_RED}错误: --from 需要时间参数${COLOR_RESET}"
                    exit 1
                fi
                FROM_TIME="$2"
                shift 2
                ;;
            --to)
                if [[ -z "${2:-}" ]]; then
                    echo -e "${COLOR_RED}错误: --to 需要时间参数${COLOR_RESET}"
                    exit 1
                fi
                TO_TIME="$2"
                shift 2
                ;;
            --errors)
                SHOW_ERRORS=true
                shift
                ;;
            --timeline)
                SHOW_TIMELINE=true
                shift
                ;;
            --frames)
                SHOW_FRAMES=true
                shift
                ;;
            -*)
                echo -e "${COLOR_RED}错误: 未知参数 '$1'${COLOR_RESET}"
                echo "使用 --help 查看帮助信息"
                exit 1
                ;;
            *)
                # 第一个非选项参数作为日志文件路径
                if [[ -z "$LOG_FILE" ]]; then
                    LOG_FILE="$1"
                fi
                shift
                ;;
        esac
    done

    # 默认日志文件
    if [[ -z "$LOG_FILE" ]]; then
        # 尝试多个默认路径
        if [[ -f "data/log/planning.INFO" ]]; then
            LOG_FILE="data/log/planning.INFO"
        elif [[ -f "/apollo_workspace/data/log/planning.INFO" ]]; then
            LOG_FILE="/apollo_workspace/data/log/planning.INFO"
        elif [[ -f "/home/skye/application-pnc/data/log/planning.INFO" ]]; then
            LOG_FILE="/home/skye/application-pnc/data/log/planning.INFO"
        else
            echo -e "${COLOR_RED}错误: 未指定日志文件，也未找到默认的 planning.INFO${COLOR_RESET}"
            echo "用法: bash scripts/planlog.sh <日志文件> [选项]"
            exit 1
        fi
    fi

    # 解析符号链接
    if [[ -L "$LOG_FILE" ]]; then
        LOG_FILE=$(readlink -f "$LOG_FILE")
    fi

    if [[ ! -f "$LOG_FILE" ]]; then
        echo -e "${COLOR_RED}错误: 日志文件不存在: $LOG_FILE${COLOR_RESET}"
        exit 1
    fi
}

# ────────────────────────────────────────
# 按场景过滤
# ────────────────────────────────────────
filter_by_scenario() {
    local scenario="$1"
    echo -e "${COLOR_BOLD}=== 场景过滤: $scenario ===${COLOR_RESET}"
    echo ""

    # 匹配包含场景名的行:
    # - "switch scenario from X to Y" (其中 X 或 Y 匹配)
    # - "scenario" 后跟场景名的各种模式
    # - "[scn:NAME]" 标记
    grep -i -E \
        "scenario.*${scenario}|switch scenario from.*${scenario}|switch scenario.*to ${scenario}|\[scn:${scenario}\]" \
        "$LOG_FILE" 2>/dev/null || echo "(没有匹配的日志行)"
}

# ────────────────────────────────────────
# 按帧号过滤
# ────────────────────────────────────────
filter_by_frame() {
    local frame="$1"
    echo -e "${COLOR_BOLD}=== 帧 [$frame] 完整日志 ===${COLOR_RESET}"
    echo ""

    # 查找帧开始和结束标记
    local start_line
    local end_line

    start_line=$(grep -n "Planning start frame sequence id = \[${frame}\]" "$LOG_FILE" | head -1 | cut -d: -f1)
    if [[ -z "$start_line" ]]; then
        echo -e "${COLOR_YELLOW}未找到帧 [$frame] 的开始标记${COLOR_RESET}"
        return
    fi

    end_line=$(grep -n "Planning end frame sequence id = \[${frame}\]" "$LOG_FILE" | head -1 | cut -d: -f1)
    if [[ -z "$end_line" ]]; then
        # 如果没有结束标记，打印从开始行到文件末尾
        end_line=$(wc -l < "$LOG_FILE")
    fi

    echo -e "${COLOR_CYAN}帧 [$frame]: 行 $start_line → $end_line ($((end_line - start_line + 1)) 行)${COLOR_RESET}"
    echo ""

    sed -n "${start_line},${end_line}p" "$LOG_FILE"
}

# ────────────────────────────────────────
# 按日志级别过滤
# ────────────────────────────────────────
filter_by_level() {
    local pattern
    pattern=$(build_level_pattern "$LEVEL")
    if [[ -z "$pattern" ]]; then
        echo -e "${COLOR_RED}错误: 无效的日志级别: $LEVEL${COLOR_RESET}"
        echo "有效值: INFO, WARN, ERROR, FATAL"
        return 1
    fi

    echo -e "${COLOR_BOLD}=== 日志级别过滤: $LEVEL ===${COLOR_RESET}"
    echo ""

    grep -E "$pattern" "$LOG_FILE" 2>/dev/null || echo "(没有匹配的日志行)"
}

# ────────────────────────────────────────
# 按时间范围过滤
# ────────────────────────────────────────
filter_by_time() {
    local from="$1"
    local to="$2"

    echo -e "${COLOR_BOLD}=== 时间范围: $from → $to ===${COLOR_RESET}"
    echo ""

    # 日志时间格式: MMDD HH:MM:SS.microsec
    # 我们匹配 HH:MM:SS 部分（日志行开头的第 6-13 个字符左右）
    awk -v from="$from" -v to="$to" '
    {
        # 日志行格式: I0616 09:52:48.198967 ...
        # 提取时间部分 (第 6-13 字符)
        time_str = substr($0, 6, 8)
        if (time_str >= from && time_str <= to) {
            print
        }
    }' "$LOG_FILE" 2>/dev/null || echo "(没有匹配的日志行)"
}

# ────────────────────────────────────────
# 错误统计
# ────────────────────────────────────────
show_error_summary() {
    echo -e "${COLOR_BOLD}=== 错误统计摘要 ===${COLOR_RESET}"
    echo ""

    # 提取所有 ERROR 行
    local error_lines
    error_lines=$(grep -c '^E' "$LOG_FILE" 2>/dev/null || echo 0)
    echo -e "总错误行数: ${COLOR_RED}$error_lines${COLOR_RESET}"
    echo ""

    if [[ "$error_lines" -eq 0 ]]; then
        echo -e "${COLOR_GREEN}没有发现错误日志！${COLOR_RESET}"
        return
    fi

    echo -e "${COLOR_BOLD}错误模式统计:${COLOR_RESET}"
    echo "────────────────────────────────────────────────────────────"

    # 提取错误消息（去除时间戳和线程 ID），统计出现次数
    grep '^E' "$LOG_FILE" 2>/dev/null | \
        sed 's/^E[0-9]\{4\} [0-9:.]* *[0-9]* *[^]]*\] //' | \
        sort | uniq -c | sort -rn | head -20 | \
        while read -r count message; do
            if (( count >= 10 )); then
                printf "${COLOR_RED}%6d${COLOR_RESET}  %s\n" "$count" "$message"
            elif (( count >= 3 )); then
                printf "${COLOR_YELLOW}%6d${COLOR_RESET}  %s\n" "$count" "$message"
            else
                printf "%6d  %s\n" "$count" "$message"
            fi
        done

    echo ""
    echo -e "${COLOR_BOLD}首次/末次错误:${COLOR_RESET}"
    echo "────────────────────────────────────────────────────────────"

    # 显示第一个错误的时间戳
    local first_error
    first_error=$(grep '^E' "$LOG_FILE" | head -1)
    if [[ -n "$first_error" ]]; then
        echo -e "首次: ${COLOR_YELLOW}${first_error:0:80}...${COLOR_RESET}"
    fi

    # 显示最后一个错误的时间戳
    local last_error
    last_error=$(grep '^E' "$LOG_FILE" | tail -1)
    if [[ -n "$last_error" ]]; then
        echo -e "末次: ${COLOR_YELLOW}${last_error:0:80}...${COLOR_RESET}"
    fi
}

# ────────────────────────────────────────
# 场景切换时间线
# ────────────────────────────────────────
show_timeline() {
    echo -e "${COLOR_BOLD}=== 场景切换时间线 ===${COLOR_RESET}"
    echo ""

    # 提取所有 "switch scenario" 行
    local switches
    switches=$(grep -n "switch scenario from" "$LOG_FILE" 2>/dev/null || true)

    if [[ -z "$switches" ]]; then
        echo -e "${COLOR_YELLOW}没有发现场景切换记录${COLOR_RESET}"
        return
    fi

    echo "$switches" | while IFS= read -r line; do
        # 提取行号和消息内容
        local lineno="${line%%:*}"
        local rest="${line#*:}"

        # 提取时间戳（月日 时分秒）
        local ts
        ts=$(echo "$rest" | grep -oP '^\w\d{4} \d{2}:\d{2}:\d{2}')

        # 提取场景名
        local from_scene to_scene
        from_scene=$(echo "$rest" | grep -oP 'from\s+\K\S+' || echo "?")
        to_scene=$(echo "$rest" | grep -oP 'to\s+\K\S+' || echo "?")

        # 美化输出
        printf "${COLOR_CYAN}[%s]${COLOR_RESET} " "$ts"
        printf "${COLOR_BOLD}行 %-6s${COLOR_RESET} " "$lineno"
        printf "${COLOR_YELLOW}%-25s${COLOR_RESET}" "$from_scene"
        printf " ${COLOR_GREEN}→${COLOR_RESET} "
        printf "${COLOR_BLUE}%-25s${COLOR_RESET}" "$to_scene"
        echo ""
    done

    echo ""
    local total_switches
    total_switches=$(echo "$switches" | wc -l)
    echo -e "总切换次数: ${COLOR_BOLD}$total_switches${COLOR_RESET}"
}

# ────────────────────────────────────────
# 帧边界提取
# ────────────────────────────────────────
show_frame_boundaries() {
    echo -e "${COLOR_BOLD}=== 帧边界提取 ===${COLOR_RESET}"
    echo ""

    # 提取帧开始和结束
    local frame_start_lines
    frame_start_lines=$(grep -n "Planning start frame sequence id" "$LOG_FILE" 2>/dev/null || true)
    local frame_end_lines
    frame_end_lines=$(grep -n "Planning end frame sequence id" "$LOG_FILE" 2>/dev/null || true)

    if [[ -z "$frame_start_lines" ]]; then
        echo -e "${COLOR_YELLOW}没有发现帧边界记录${COLOR_RESET}"
        return
    fi

    local total_frames
    total_frames=$(echo "$frame_start_lines" | wc -l)
    echo -e "总帧数: ${COLOR_BOLD}$total_frames${COLOR_RESET}"
    echo ""

    # 显示前 20 帧详情
    echo -e "${COLOR_BOLD}帧详情 (前 20 帧):${COLOR_RESET}"
    echo "────────────────────────────────────────────────────────────"

    local count=0
    echo "$frame_start_lines" | head -20 | while IFS= read -r start_line; do
        local lineno="${start_line%%:*}"
        local rest="${start_line#*:}"
        local frame_id
        frame_id=$(echo "$rest" | grep -oP '\[(\d+)\]' | grep -oP '\d+' || echo "?")

        # 查找对应的结束行
        local end_line_info
        end_line_info=$(grep -n "Planning end frame sequence id = \[${frame_id}\]" "$LOG_FILE" | head -1 || echo "")
        local end_lineno
        if [[ -n "$end_line_info" ]]; then
            end_lineno="${end_line_info%%:*}"
        else
            end_lineno="?"
        fi

        # 提取时间戳
        local ts
        ts=$(echo "$rest" | grep -oP '^\w\d{4} \d{2}:\d{2}:\d{2}')

        if [[ "$end_lineno" != "?" ]]; then
            printf "${COLOR_CYAN}[%s]${COLOR_RESET} " "$ts"
            printf "帧 ${COLOR_BOLD}%-6s${COLOR_RESET} " "#$frame_id"
            printf "行 ${COLOR_GREEN}%-6s${COLOR_RESET} → ${COLOR_GREEN}%-6s${COLOR_RESET} " "$lineno" "$end_lineno"
            printf "($((end_lineno - lineno)) 行)"
        else
            printf "${COLOR_CYAN}[%s]${COLOR_RESET} " "$ts"
            printf "帧 ${COLOR_BOLD}%-6s${COLOR_RESET} " "#$frame_id"
            printf "行 ${COLOR_GREEN}%-6s${COLOR_RESET} → ${COLOR_RED}未结束${COLOR_RESET}" "$lineno"
        fi
        echo ""
        ((count++)) || true
    done

    if (( total_frames > 20 )); then
        echo "  ... (还有 $((total_frames - 20)) 帧)"
    fi
}

# ────────────────────────────────────────
# 应用组合过滤
# ────────────────────────────────────────
apply_filters() {
    local filter_cmd="cat"

    # 按级别过滤
    if [[ -n "$LEVEL" ]]; then
        local level_pattern
        level_pattern=$(build_level_pattern "$LEVEL")
        if [[ -n "$level_pattern" ]]; then
            filter_cmd="$filter_cmd | grep -E '$level_pattern'"
        fi
    fi

    # 按场景过滤
    if [[ -n "$SCENARIO" ]]; then
        filter_cmd="$filter_cmd | grep -i -E 'scenario.*${SCENARIO}|switch scenario from.*${SCENARIO}|switch scenario.*to ${SCENARIO}|\[scn:${SCENARIO}\]'"
    fi

    # 按时间过滤
    if [[ -n "$FROM_TIME" ]] || [[ -n "$TO_TIME" ]]; then
        local from="${FROM_TIME:-00:00:00}"
        local to="${TO_TIME:-23:59:59}"
        filter_cmd="$filter_cmd | awk -v from=\"$from\" -v to=\"$to\" '{time_str = substr(\$0, 6, 8); if (time_str >= from && time_str <= to) print}'"
    fi

    echo "$filter_cmd"
}

# ────────────────────────────────────────
# 主函数
# ────────────────────────────────────────
main() {
    parse_args "$@"

    echo -e "${COLOR_BOLD}日志文件: ${LOG_FILE}${COLOR_RESET}"
    echo -e "文件大小: $(du -h "$LOG_FILE" | cut -f1)"
    echo ""

    # 特殊模式（互斥）
    if $SHOW_ERRORS; then
        show_error_summary
        exit 0
    fi

    if $SHOW_TIMELINE; then
        show_timeline
        exit 0
    fi

    if $SHOW_FRAMES; then
        show_frame_boundaries
        exit 0
    fi

    # 按帧过滤（特殊处理：需要提取帧边界内的所有行）
    if [[ -n "$FRAME" ]]; then
        filter_by_frame "$FRAME"
        exit 0
    fi

    # 组合过滤模式
    local filter_cmd
    filter_cmd=$(apply_filters)

    if [[ "$filter_cmd" == "cat" ]]; then
        # 没有任何过滤条件，显示帮助
        echo -e "${COLOR_YELLOW}未指定过滤条件。使用 --help 查看可用选项。${COLOR_RESET}"
        echo ""
        echo "快速示例:"
        echo "  bash scripts/planlog.sh $LOG_FILE --scenario lane_follow"
        echo "  bash scripts/planlog.sh $LOG_FILE --errors"
        echo "  bash scripts/planlog.sh $LOG_FILE --timeline"
        exit 0
    fi

    # 执行过滤
    local result
    result=$(eval "$filter_cmd < '$LOG_FILE'" 2>/dev/null || true)

    if [[ -z "$result" ]]; then
        echo -e "${COLOR_YELLOW}(没有匹配的日志行)${COLOR_RESET}"
    else
        local line_count
        line_count=$(echo "$result" | wc -l)
        echo -e "${COLOR_CYAN}匹配 $line_count 行:${COLOR_RESET}"
        echo "────────────────────────────────────────────────────────────"
        echo "$result"
    fi
}

main "$@"
