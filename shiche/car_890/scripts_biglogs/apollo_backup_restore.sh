#!/usr/bin/env bash
set -Eeuo pipefail

readonly WS="/apollo_workspace"
readonly BAK_ROOT="/media/.XH_2026_bak"
readonly KILL_SCRIPT="${WS}/kill_all.sh"
readonly PORT=8888
readonly PORT_TIMEOUT=10

BAK_NAME=""
ASSUME_YES=0
BAK_DIR=""
LOG_FILE=""
CURRENT_STEP="未开始"
SUMMARY_PRINTED=0

declare -a STEP_RESULT=()
declare -a ITEMS=(
  "auto_drive_console|${WS}/auto_drive_console|${WS}"
  "scenario_injection|${WS}/scenario_injection|${WS}"
  "grading_system_py|${WS}/grading_system_py|${WS}"
  "modules|${WS}/modules|${WS}"
  "HQEV_BDN|${WS}/profiles/HQEV_BDN|${WS}/profiles"
)
declare -a RESTORE_SRC=()
declare -a RESTORE_TYPE=()

usage() {
  cat <<'USAGE'
用法:
  apollo_backup_restore.sh [备份名称] [-y|--yes]

说明:
  脚本必须在目标 Apollo 环境中执行。
  恢复源只检查 /media/.XH_2026_bak 根目录下的对应 .tar.gz 或裸目录。
USAGE
}

log() {
  local message="[$(date '+%F %T')] $*"
  if [[ -n "$LOG_FILE" ]]; then
    printf '%s\n' "$message" | tee -a "$LOG_FILE"
  else
    printf '%s\n' "$message"
  fi
}

ok() {
  STEP_RESULT+=("OK   | $1")
  log "成功: $1"
}

fail() {
  local step="$1"
  local reason="${2:-未知原因}"
  STEP_RESULT+=("FAIL | ${step} (${reason})")
  log "失败: ${step}；原因: ${reason}"
  exit 1
}

print_summary() {
  (( SUMMARY_PRINTED )) && return 0
  SUMMARY_PRINTED=1
  printf '\n================ 执行结果汇总 ================\n'
  if (( ${#STEP_RESULT[@]} )); then
    printf '%s\n' "${STEP_RESULT[@]}"
  fi
  [[ -n "$BAK_DIR" ]] && printf '备份目录: %s\n' "$BAK_DIR"
  [[ -n "$LOG_FILE" ]] && printf '日志文件: %s\n' "$LOG_FILE"
  printf '当前步骤: %s\n' "$CURRENT_STEP"
  printf '==============================================\n'
}

on_exit() {
  local code=$?
  print_summary
  return "$code"
}
trap on_exit EXIT

parse_args() {
  while (($#)); do
    case "$1" in
      -y|--yes) ASSUME_YES=1 ;;
      -h|--help) usage; exit 0 ;;
      -*) usage >&2; fail "参数解析" "未知参数: $1" ;;
      *)
        [[ -z "$BAK_NAME" ]] || fail "参数解析" "备份名称只能指定一次"
        BAK_NAME="$1"
        ;;
    esac
    shift
done

  if [[ -z "$BAK_NAME" ]]; then
    read -r -p '请输入本次备份名称: ' BAK_NAME
  fi
  [[ "$BAK_NAME" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "参数解析" "名称仅允许字母、数字、下划线、短横线和点"

  BAK_DIR="${BAK_ROOT}/${BAK_NAME}_$(date '+%Y%m%d_%H%M%S')"
  LOG_FILE="${BAK_DIR}/run.log"
}

check_disk_space() {
  local required available source size_k
  required=0
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r _ source _ <<< "$item"
    size_k=$(du -sk -- "$source" | cut -f1) || return 1
    required=$((required + size_k))
  done
  available=$(df -Pk -- "$BAK_ROOT" | awk 'NR==2 {print $4}')
  [[ "$available" =~ ^[0-9]+$ ]] || return 1
  (( available >= required * 60 / 100 ))
}

resolve_restore_sources() {
  local missing=0 index=0 name
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r name _ _ <<< "$item"
    if [[ -f "${BAK_ROOT}/${name}.tar.gz" ]]; then
      RESTORE_SRC[$index]="${BAK_ROOT}/${name}.tar.gz"
      RESTORE_TYPE[$index]="tar"
    elif [[ -d "${BAK_ROOT}/${name}" ]]; then
      RESTORE_SRC[$index]="${BAK_ROOT}/${name}"
      RESTORE_TYPE[$index]="dir"
    else
      log "恢复源缺失: ${BAK_ROOT}/${name}.tar.gz 或 ${BAK_ROOT}/${name}/"
      missing=1
      index=$((index + 1))
      continue
    fi
    log "恢复源已发现: ${name} -> ${RESTORE_SRC[$index]}"
    index=$((index + 1))
  done
  (( missing == 0 )) || return 1
}

preflight() {
  CURRENT_STEP="预检"
  local command_name source
  for command_name in tar aem buildtool; do
    command -v "$command_name" >/dev/null 2>&1 || fail "预检-命令依赖" "缺少 ${command_name}"
  done
  [[ -f "$KILL_SCRIPT" ]] || fail "预检-kill_all.sh" "文件不存在: $KILL_SCRIPT"
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r _ source _ <<< "$item"
    [[ -d "$source" ]] || fail "预检-备份源" "目录不存在: $source"
  done
  mkdir -p -- "$BAK_ROOT" || fail "预检-备份根目录" "无法创建: $BAK_ROOT"
  [[ -w "$BAK_ROOT" ]] || fail "预检-备份根目录" "目录不可写: $BAK_ROOT"
  LOG_FILE="${BAK_DIR}/run.log"
  mkdir -p -- "$BAK_DIR" || fail "预检-本次备份目录" "无法创建: $BAK_DIR"
  : > "$LOG_FILE" || fail "预检-日志文件" "无法写入: $LOG_FILE"
  check_disk_space || fail "预检-磁盘空间" "备份根目录可用空间不足"
  resolve_restore_sources || fail "预检-恢复源" "根目录下存在缺失的对应恢复内容"
  ok "预检"
}

step_kill_all() {
  CURRENT_STEP="执行 kill_all.sh"
  bash "$KILL_SCRIPT" || fail "kill_all.sh"
  ok "kill_all.sh"
}

step_backup() {
  CURRENT_STEP="备份"
  local name source archive
  : > "${BAK_DIR}/MANIFEST.txt" || fail "备份清单"
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r name source _ <<< "$item"
    archive="${BAK_DIR}/${name}.tar.gz"
    log "开始打包: ${source} -> ${archive}"
    tar -czf "$archive" -C "$(dirname "$source")" "$(basename "$source")" || fail "备份-${name}"
    tar -tzf "$archive" >/dev/null || fail "备份校验-${name}" "压缩包不可读取"
    printf '%s\t%s\t%s\n' "$name" "$source" "$(du -h "$archive" | cut -f1)" >> "${BAK_DIR}/MANIFEST.txt"
    ok "备份-${name}"
  done
  sync
  ok "备份阶段"
}

step_profile_sample() {
  CURRENT_STEP="后台切换 sample profile"
  aem profile use sample >>"$LOG_FILE" 2>&1 &
  ok "aem profile use sample 已后台启动"
  sleep 3
  ok "等待 3 秒"
}

confirm_delete() {
  (( ASSUME_YES )) && return 0
  printf '即将删除以下目录（不可逆操作）:\n'
  local item name source _
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r name source _ <<< "$item"
    printf '  - %s\n' "$source"
  done
  read -r -p '请输入 yes 确认删除: ' answer
  [[ "$answer" == "yes" ]] || fail "删除确认" "用户未输入 yes"
}

step_delete() {
  CURRENT_STEP="删除目录"
  confirm_delete
  local item name source _
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r name source _ <<< "$item"
    [[ "$source" == "${WS}/"* && ${#source} -gt ${#WS} ]] || fail "删除-${name}" "目标路径校验失败"
    rm -rf -- "$source" || fail "删除-${name}"
    [[ ! -e "$source" ]] || fail "删除-${name}" "删除后目录仍存在"
    ok "删除-${name}"
  done
  sleep 3
  ok "等待 3 秒"
}

step_restore() {
  CURRENT_STEP="恢复目录"
  resolve_restore_sources || fail "恢复前复检" "根目录恢复源已缺失"
  local item index=0 name target_parent restore_source restore_type
  for item in "${ITEMS[@]}"; do
    IFS='|' read -r name _ target_parent <<< "$item"
    restore_source="${RESTORE_SRC[$index]}"
    restore_type="${RESTORE_TYPE[$index]}"
    mkdir -p -- "$target_parent" || fail "恢复-${name}" "无法创建目标父目录"
    case "$restore_type" in
      tar) tar -xzf "$restore_source" -C "$target_parent" || fail "恢复-${name}" ;;
      dir) mkdir -p -- "${target_parent}/${name}" && cp -a "${restore_source}/." "${target_parent}/${name}/" || fail "恢复-${name}" ;;
      *) fail "恢复-${name}" "未知恢复类型: $restore_type" ;;
    esac
    [[ -d "${target_parent}/${name}" ]] || fail "恢复-${name}" "恢复后目标目录不存在"
    ok "恢复-${name} -> ${target_parent}/${name}"
    index=$((index + 1))
  done
  ok "恢复阶段"
}

step_build() {
  CURRENT_STEP="构建"
  (cd "$WS" && buildtool build) || fail "buildtool build"
  ok "buildtool build"
}

step_profile_hqev() {
  CURRENT_STEP="后台切换 HQEV_BDN profile"
  aem profile use HQEV_BDN >>"$LOG_FILE" 2>&1 &
  ok "aem profile use HQEV_BDN 已后台启动"
}

wait_port() {
  local port="$1" deadline=$((SECONDS + "$2"))
  while (( SECONDS < deadline )); do
    if command -v ss >/dev/null 2>&1 && ss -lnt 2>/dev/null | grep -Eq "[:.]${port}[[:space:]]"; then return 0; fi
    if command -v netstat >/dev/null 2>&1 && netstat -lnt 2>/dev/null | grep -Eq "[:.]${port}[[:space:]]"; then return 0; fi
    if command -v curl >/dev/null 2>&1 && curl -ksS -o /dev/null --max-time 2 "http://127.0.0.1:${port}"; then return 0; fi
    sleep 3
  done
  return 1
}

step_start_dv() {
  CURRENT_STEP="启动 DV"
  aem bootstrap start --plus || fail "aem bootstrap start --plus"
  ok "aem bootstrap start --plus"
  wait_port "$PORT" "$PORT_TIMEOUT" || fail "8888 端口检查" "${PORT_TIMEOUT} 秒内未监听"
  ok "8888 端口检查"
}

main() {
  parse_args "$@"
  preflight
  step_kill_all
  step_backup
  step_profile_sample
  step_delete
  step_restore
  step_build
  step_profile_hqev
  step_start_dv
  CURRENT_STEP="全部完成"
  ok "备份、删除、恢复、构建和 DV 启动流程"
}

main "$@"
