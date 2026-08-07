#!/usr/bin/env bash
# ============================================================
# 实车 890 配置一键部署脚本（苏州实车赛 · 2026-08-07）
#
# 功能：
#   1) 连接实车（SSH）
#   2) 先备份实车当前 4 个 planning 配置（车上 + 本地双备份）
#   3) 菜单选择 部署计划 A / B / C
#   4) 打包本地对应计划 -> scp 覆盖实车配置
#   5) 验证生效值
#   6) 支持现场微调（C 计划：--set 参数=值）与回滚
#
# 用法：
#   ./car_deploy.sh                    # 交互菜单
#   ./car_deploy.sh --plan A           # 直接部署 A（自动先备份）
#   ./car_deploy.sh --plan B
#   ./car_deploy.sh --plan C           # C = 先部署 A，随后可 --set 微调
#   ./car_deploy.sh --set cruise=2.2   # 现场微调单个参数
#   ./car_deploy.sh --set min_stop=0.9
#   ./car_deploy.sh --set limit=0.6    # 同时改 crosswalk+traffic_light 限速值
#   ./car_deploy.sh --rollback         # 从最近备份回滚
#   ./car_deploy.sh --backup-only      # 只备份
#   ./car_deploy.sh --list             # 列出本机可用计划与参数
#   ./car_deploy.sh --restore-default  # 一键恢复官方默认配置（ORIGINAL，应急回归基线）
#   ./car_deploy.sh --save-baseline    # 把当前实车配置保存为第一版基线（比赛当天原始状态）
#   ./car_deploy.sh --restore-baseline # 一键恢复到第一版基线（首次部署自动保存，随时可回）
#
# 可选环境变量：HOST / USER / PASS / CAR_PROJ
#   HOST     实车 IP（默认 192.168.10.6）
#   USER     用户名（默认 apollo）
#   PASS     SSH 密码（默认 apollo，⚠️注意不是 WiFi 密码 apollo123）
#   CAR_PROJ 车上工程目录名（默认自动探测 apollo-edu_9.1.1 / apollo-edu-9.1.1）
# 💡 免密：现场第一次手动 `ssh apollo@192.168.10.6` 输一次密码后，4 小时内 ssh/scp 全部免密，脚本不再需要 sshpass。
# ============================================================

set -uo pipefail

# ---------------- 配置 ----------------
HOST="${HOST:-192.168.10.6}"
USER="${USER:-apollo}"
PASS="${PASS:-apollo}"                    # ⚠️ SSH 密码 = apollo（WiFi 才是 apollo123）
CAR_PROJ="${CAR_PROJ:-}"                    # 留空则自动探测
CAR_CFG_ROOT=""                             # 探测后设置
TS=$(date +%Y%m%d_%H%M%S)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_PLANS="$REPO_ROOT/realcar_tuning/PLANS"
LOCAL_BAK="$REPO_ROOT/backup/car_deploy/$TS"

# 需要部署/备份的 4 个配置文件的相对路径（相对 CAR_CFG_ROOT）
CONF_FILES=(
  "modules/planning/planning_component/conf/planning.conf"
  "modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt"
  "modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt"
  "modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt"
)

# 计划 -> 本地目录映射（C = 先同 A）
declare -A PLAN_DIR=(
  [A]="PLAN_A"
  [B]="PLAN_B"
  [C]="PLAN_A"
)

# ---------------- 工具函数 ----------------
# 免密优先：若已建立免密（首次手动 ssh 后），直接用 ssh；否则用 sshpass 带密码
_ssh() {
  ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "$@" 2>/dev/null \
    || sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "$@"
}
# scp 同样免密优先：免密可用直接 scp，否则 sshpass 兜底（与 _ssh 逻辑一致，避免依赖 sshpass）
_scp_up() {
  if ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$USER@$HOST" "echo ok" >/dev/null 2>&1; then
    scp -o StrictHostKeyChecking=no "$1" "$USER@$HOST:$2"
  else
    sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$1" "$USER@$HOST:$2"
  fi
}
_scp_dn() {
  if ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5 "$USER@$HOST" "echo ok" >/dev/null 2>&1; then
    scp -o StrictHostKeyChecking=no "$USER@$HOST:$1" "$2"
  else
    sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$USER@$HOST:$1" "$2"
  fi
}

need_sshpass() {
  # 若免密可用则不需要 sshpass
  if ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$USER@$HOST" "echo ok" >/dev/null 2>&1; then
    return 0
  fi
  if ! command -v sshpass >/dev/null 2>&1; then
    echo "❌ 未安装 sshpass 且未建立免密。"
    echo "   方案1（推荐）：先手动执行一次：ssh $USER@$HOST（输密码 apollo），之后 4 小时免密"
    echo "   方案2：sudo apt-get install sshpass"
    exit 1
  fi
}

# 自动探测工程目录名（apollo-edu_9.1.1 下划线 / apollo-edu-9.1.1 连字符，现场以实际为准）
detect_project() {
  if [[ -n "$CAR_PROJ" ]]; then
    CAR_CFG_ROOT="~/$CAR_PROJ/profiles/HQEV_BDN"
    return 0
  fi
  local found
  found=$(ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "ls -d ~/apollo-edu* 2>/dev/null | head -1 | xargs -n1 basename" 2>/dev/null)
  if [[ -z "$found" ]]; then
    found=$(sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "ls -d ~/apollo-edu* 2>/dev/null | head -1 | xargs -n1 basename" 2>/dev/null)
  fi
  if [[ -z "$found" ]]; then
    echo "❌ 未探测到车上 apollo-edu* 工程目录，请用 CAR_PROJ=实际目录名 ./car_deploy.sh 指定"
    exit 1
  fi
  CAR_PROJ="$found"
  CAR_CFG_ROOT="~/$CAR_PROJ/profiles/HQEV_BDN"
  echo "✅ 自动探测工程目录: $CAR_PROJ"
}


ping_car() {
  echo "==> 连接实车 $USER@$HOST ..."
  need_sshpass
  detect_project
  if _ssh "echo OK && ls -d $CAR_CFG_ROOT 2>/dev/null"; then
    echo "✅ 连接成功，配置目录存在：$CAR_CFG_ROOT"
  else
    echo "❌ 连接失败或配置目录不存在。请检查：WiFi(HQEVxxx)/IP/工程名。"
    echo "   若工程目录探测不对，用 CAR_PROJ=实际目录名 ./car_deploy.sh 覆盖"
    exit 1
  fi
}

# ---------------- 1. 备份 ----------------
backup_car() {
  echo "===== [备份] 车上 + 本地 双备份 ====="
  mkdir -p "$LOCAL_BAK"
  local REMOTE_BAK="~/car_config_backup_$TS"
  # 车上打包
  local files_str=""
  for f in "${CONF_FILES[@]}"; do files_str="$files_str $f"; done
  _ssh "mkdir -p $REMOTE_BAK && cd $CAR_CFG_ROOT && cp --parents $files_str $REMOTE_BAK/ && tar -czf $REMOTE_BAK/config_backup.tar.gz -C $REMOTE_BAK . && echo '车上备份: $REMOTE_BAK' && ls -la $REMOTE_BAK/"
  # 下载到本地
  _ssh "cat $REMOTE_BAK/config_backup.tar.gz" > "$LOCAL_BAK/config_backup.tar.gz"
  echo "✅ 本地备份: $LOCAL_BAK/config_backup.tar.gz"
  echo "   最新备份路径记录: $REMOTE_BAK" > "$LOCAL_BAK/README.txt"
  echo "$REMOTE_BAK" > "$REPO_ROOT/backup/car_deploy/.latest_remote"
}

# ---------------- 2. 部署 ----------------
deploy_plan() {
  local plan="$1"
  local dir="${PLAN_DIR[$plan]:-}"
  if [[ -z "$dir" || ! -d "$LOCAL_PLANS/$dir" ]]; then
    echo "❌ 找不到计划目录: $LOCAL_PLANS/$dir"; exit 1
  fi
  echo "===== [部署] 计划 $plan -> $dir ====="
  # 首次部署自动锁定第一版基线（比赛当天实车原始配置）
  ensure_baseline
  # 先备份
  backup_car
  # 只打包 4 个目标文件（不打包整个 modules 树，避免误覆盖车上其他配置）
  local tarball="/tmp/plan_${plan}_$TS.tar.gz"
  local files_str=""
  for f in "${CONF_FILES[@]}"; do files_str="$files_str $f"; done
  (cd "$LOCAL_PLANS/$dir" && tar -czf "$tarball" $files_str 2>/dev/null) || {
    echo "❌ 本地打包失败"; exit 1; }
  echo "  本地打包: $tarball（仅 4 个目标文件）"
  echo "  内容: $(tar -tzf "$tarball" | tr '\n' ' ')"
  # 上传并覆盖（检查解压是否成功）
  _scp_up "$tarball" "/tmp/"
  if _ssh "cd $CAR_CFG_ROOT && tar -xzf /tmp/$(basename $tarball) && echo DEPLOY_OK"; then
    echo "✅ 已覆盖配置: $CAR_CFG_ROOT"
  else
    echo "❌ 解压覆盖失败！请检查车上空间/权限，用 --rollback 回滚"
    exit 1
  fi
  echo ""
  verify_car
  echo ""
  echo ">>> 下一步：请重启 Planning 模块（DreamView 关闭再开启 Planning，或 cyber_launch restart）"
}

# ---------------- 2.5 一键恢复官方默认配置 ----------------
restore_default() {
  local src="$REPO_ROOT/realcar_tuning/ORIGINAL"
  echo "===== [恢复默认] 一键恢复官方默认配置（ORIGINAL）====="
  # 本地官方默认 4 个目标文件齐全性检查
  for f in "${CONF_FILES[@]}"; do
    if [[ ! -f "$src/$f" ]]; then
      echo "❌ 本地官方默认配置缺失: $src/$f"; exit 1
    fi
  done
  # 首次恢复也自动锁定第一版基线（若尚未保存）
  ensure_baseline
  # 先备份当前配置（恢复前可 --rollback 回到当前状态）
  backup_car
  local tarball="/tmp/restore_default_$TS.tar.gz"
  local files_str=""
  for f in "${CONF_FILES[@]}"; do files_str="$files_str $f"; done
  (cd "$src" && tar -czf "$tarball" $files_str 2>/dev/null) || {
    echo "❌ 本地打包失败"; exit 1; }
  echo "  本地打包: $tarball（官方默认 4 个目标文件）"
  # 上传并覆盖（检查解压是否成功）
  _scp_up "$tarball" "/tmp/"
  if _ssh "cd $CAR_CFG_ROOT && tar -xzf /tmp/$(basename $tarball) && echo RESTORE_OK"; then
    echo "✅ 已恢复官方默认配置: $CAR_CFG_ROOT"
  else
    echo "❌ 恢复失败！请检查车上空间/权限，用 --rollback 回滚"
    exit 1
  fi
  echo ""
  verify_car
  echo ""
  echo "⚠️ 已恢复官方默认（cruise=5.0 会持续超速扣分，仅应急回归基线用）"
  echo ">>> 比赛前必须重新部署 A/B，并重启 Planning 模块"
}

# ---------------- 2.6 第一版基线（比赛当天实车原始配置） ----------------
# 核心：把当前实车 4 个配置打包为"第一版基线"，本地 + 车上双份保存
save_baseline_core() {
  local BASELINE_DIR="$REPO_ROOT/backup/car_deploy/baseline"
  local REMOTE_BASE="~/car_config_baseline"
  mkdir -p "$BASELINE_DIR"
  local files_str=""
  for f in "${CONF_FILES[@]}"; do files_str="$files_str $f"; done
  # 车上也存一份（双保险：换笔记本也能从车上恢复）
  _ssh "mkdir -p $REMOTE_BASE && cd $CAR_CFG_ROOT && tar -czf $REMOTE_BASE/config_backup.tar.gz $files_str && echo '✅ 车上基线: $REMOTE_BASE/config_backup.tar.gz'" \
    || { echo "❌ 车上基线打包失败"; return 1; }
  # 下载到本地（tar 内为 modules/... 相对路径，恢复时 cd CAR_CFG_ROOT 解压即可）
  _ssh "cat $REMOTE_BASE/config_backup.tar.gz" > "$BASELINE_DIR/config_backup.tar.gz"
  echo "第一版基线（实车原始配置）: $(date '+%Y-%m-%d %H:%M:%S')" > "$BASELINE_DIR/README.txt"
  echo "✅ 第一版基线已保存（本地 $BASELINE_DIR/ + 车上 $REMOTE_BASE/）"
}

# 首次自动保存：比赛当天第一次部署/恢复时，把实车原始配置锁为第一版基线（已有则跳过）
ensure_baseline() {
  local BASELINE_DIR="$REPO_ROOT/backup/car_deploy/baseline"
  if [[ -f "$BASELINE_DIR/config_backup.tar.gz" ]]; then
    return 0
  fi
  echo "📌 尚无第一版基线 → 自动保存当前实车配置（=比赛当天原始状态，之后可 --restore-baseline 一键恢复）"
  save_baseline_core || exit 1
}

# 手动保存/覆盖基线
save_baseline() {
  local BASELINE_DIR="$REPO_ROOT/backup/car_deploy/baseline"
  echo "===== [保存基线] 将当前实车配置保存为第一版基线 ====="
  if [[ -f "$BASELINE_DIR/config_backup.tar.gz" ]]; then
    echo "⚠️ 基线已存在：$(head -1 "$BASELINE_DIR/README.txt" 2>/dev/null)"
    read -rp "  确认覆盖为新基线? [y/N]: " yn
    [[ "$yn" != "y" && "$yn" != "Y" ]] && { echo "取消"; return 0; }
  fi
  save_baseline_core
}

# 一键恢复到第一版基线
restore_baseline() {
  local BASELINE_DIR="$REPO_ROOT/backup/car_deploy/baseline"
  echo "===== [恢复基线] 一键恢复第一版基线（比赛当天原始配置）====="
  if [[ ! -f "$BASELINE_DIR/config_backup.tar.gz" ]]; then
    echo "❌ 本地没有第一版基线备份。"
    echo "   比赛当天第一次部署会自动保存第一版；或先执行：./scripts/car_deploy.sh --save-baseline"
    exit 1
  fi
  echo "  基线来源: $(head -1 "$BASELINE_DIR/README.txt" 2>/dev/null)"
  # 先备份当前配置（恢复前可 --rollback 回到当前状态）
  backup_car
  local tarball="/tmp/restore_baseline_$TS.tar.gz"
  cp "$BASELINE_DIR/config_backup.tar.gz" "$tarball"
  _scp_up "$tarball" "/tmp/"
  if _ssh "cd $CAR_CFG_ROOT && tar -xzf /tmp/$(basename $tarball) && echo BASELINE_OK"; then
    echo "✅ 已恢复第一版基线配置: $CAR_CFG_ROOT"
  else
    echo "❌ 恢复失败！请检查车上空间/权限，用 --rollback 回滚"
    exit 1
  fi
  echo ""
  verify_car
  echo ""
  echo ">>> 请重启 Planning 模块使恢复生效"
}

# ---------------- 3. 验证 ----------------
verify_car() {
  echo "===== [验证] 车上生效值 ====="
  _ssh "echo '--- planning.conf ---'; grep -E 'default_cruise_speed|planning_upper_speed_limit|min_stop_distance_obstacle' $CAR_CFG_ROOT/modules/planning/planning_component/conf/planning.conf; echo '--- crosswalk ---'; cat $CAR_CFG_ROOT/modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt; echo ''; echo '--- scene_speed_limit ---'; grep -E 'enabled|limit_speed|forward_buffer|backward_buffer' $CAR_CFG_ROOT/modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt"
}

# ---------------- 4. 回滚 ----------------
rollback_car() {
  local latest=""
  if [[ -f "$REPO_ROOT/backup/car_deploy/.latest_remote" ]]; then
    latest=$(cat "$REPO_ROOT/backup/car_deploy/.latest_remote")
  fi
  echo "===== [回滚] 从最近备份恢复 ====="
  if [[ -z "$latest" ]]; then
    echo "⚠️ 没有本地记录的最新备份，列出车上可用备份："
    _ssh "ls -d ~/car_config_backup_* 2>/dev/null | tail -5"
    read -rp "输入要恢复的备份目录（~/car_config_backup_XXX）: " latest
    [[ -z "$latest" ]] && { echo "取消"; exit 0; }
  fi
  _ssh "cd $CAR_CFG_ROOT && tar -xzf $latest/config_backup.tar.gz && echo '✅ 已从 $latest 恢复'"
  verify_car
  echo ">>> 请重启 Planning 模块使回滚生效"
}

# ---------------- 5. 现场微调 --set ----------------
# 映射：参数名 -> "文件:精确匹配行|替换"
set_param() {
  local kv="$1"; local key="${kv%%=*}"; local val="${kv#*=}"
  [[ -z "$val" ]] && { echo "❌ 用法: --set 参数=值（如 --set cruise=2.2）"; exit 1; }
  echo "===== [微调] $key = $val ====="
  case "$key" in
    cruise)
      file="modules/planning/planning_component/conf/planning.conf"
      match="default_cruise_speed="
      new="--default_cruise_speed=$val";;
    upper)
      file="modules/planning/planning_component/conf/planning.conf"
      match="planning_upper_speed_limit="
      new="--planning_upper_speed_limit=$val";;
    min_stop)
      file="modules/planning/planning_component/conf/planning.conf"
      match="min_stop_distance_obstacle="
      new="--min_stop_distance_obstacle=$val";;
    stop_distance)
      file="modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt"
      match="stop_distance:"
      new="stop_distance: $val";;
    tl_stop)
      # 红灯停车距离微调（traffic_light stop_distance）
      file="modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt"
      match="stop_distance:"
      new="stop_distance: $val";;
    expand)
      file="modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt"
      match="expand_s_distance:"
      new="expand_s_distance: $val";;
    timeout)
      file="modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt"
      match="stop_timeout:"
      new="stop_timeout: $val";;
    limit)
      # 同时改 crosswalk + traffic_light 两个区段限速值
      f1="modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt"
      _ssh "cd $CAR_CFG_ROOT && sed -i 's/crosswalk_limit_speed: .*/crosswalk_limit_speed: $val/' $f1 && sed -i 's/traffic_light_limit_speed: .*/traffic_light_limit_speed: $val/' $f1 && echo '✅ crosswalk+traffic_light limit_speed 已改为 $val'"
      verify_car | grep limit_speed
      echo ">>> 请重启 Planning 模块"; return 0;;
    forward)
      file="modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt"
      match="crosswalk_forward_buffer:"
      new="crosswalk_forward_buffer: $val"
      _ssh "cd $CAR_CFG_ROOT && sed -i 's/traffic_light_forward_buffer: .*/traffic_light_forward_buffer: $val/' $file && sed -i 's/crosswalk_forward_buffer: .*/crosswalk_forward_buffer: $val/' $file && echo '✅ forward_buffer 已改为 $val'"
      verify_car | grep forward_buffer
      echo ">>> 请重启 Planning 模块"; return 0;;
    backward)
      file="modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt"
      _ssh "cd $CAR_CFG_ROOT && sed -i 's/traffic_light_backward_buffer: .*/traffic_light_backward_buffer: $val/' $file && sed -i 's/crosswalk_backward_buffer: .*/crosswalk_backward_buffer: $val/' $file && echo '✅ backward_buffer 已改为 $val'"
      verify_car | grep backward_buffer
      echo ">>> 请重启 Planning 模块"; return 0;;
    *)
      echo "❌ 不支持的参数名: $key"
      echo "   支持: cruise / upper / min_stop / stop_distance(人行道) / tl_stop(红灯) / expand / timeout / limit / forward / backward"
      exit 1;;
  esac
  _ssh "cd $CAR_CFG_ROOT && sed -i 's|.*$match.*|$new|' $file && echo '✅ $file 已改为 $new'"
  verify_car | grep -E "default_cruise_speed|upper|min_stop|stop_distance|expand|timeout" || true
  echo ">>> 请重启 Planning 模块"
}

# ---------------- 6. 菜单 ----------------
menu() {
  echo ""
  echo "=============================================="
  echo "  实车配置部署 · 苏州实车赛 2026-08-07"
  echo "  车: $USER@$HOST  |  配置根: $CAR_CFG_ROOT"
  echo "=============================================="
  echo "  A) 部署计划 A（15项：必改12 + min_stop + expand + upper）"
  echo "  B) 部署计划 B（12项：只必改）"
  echo "  C) 部署计划 C（先=A，比赛时按反馈 --set 微调）"
  echo "  D) 恢复官方默认配置（ORIGINAL，应急回归基线）"
  echo "  E) 恢复第一版基线（比赛当天首次部署自动保存的原始配置）"
  echo "  R) 回滚到最近备份"
  echo "  V) 仅验证当前车上配置"
  echo "  L) 查看本机计划与参数说明"
  echo "  Q) 退出"
  echo "----------------------------------------------"
  read -rp "  请选择 [A/B/C/D/E/R/V/L/Q]: " ch
  case "$ch" in
    A|a) deploy_plan A ;;
    B|b) deploy_plan B ;;
    C|c) deploy_plan C ;;
    D|d) restore_default ;;
    E|e) restore_baseline ;;
    R|r) rollback_car ;;
    V|v) ping_car && verify_car ;;
    L|l) list_info ;;
    Q|q) exit 0 ;;
    *) echo "无效选择"; menu ;;
  esac
}

list_info() {
  echo "===== 本机可用部署计划 ====="
  for key in A B; do
    local d="${PLAN_DIR[$key]}"
    if [[ -d "$LOCAL_PLANS/$d" ]]; then
      echo "  - $key) $d"
      grep -hE "default_cruise_speed|planning_upper|min_stop" "$LOCAL_PLANS/$d/modules/planning/planning_component/conf/planning.conf" 2>/dev/null | sed 's/^/      /'
    fi
  done
  echo "  C) 同 A（先部署 A，比赛时按反馈 --set 微调）"
  echo "  D) 恢复官方默认配置（realcar_tuning/ORIGINAL）"
  echo "  E) 恢复第一版基线（比赛当天原始配置，首次部署自动保存）"
  echo ""
  echo "===== 基线命令 ====="
  echo "  --save-baseline    把当前实车配置保存/覆盖为第一版基线"
  echo "  --restore-baseline 一键恢复到第一版基线"
  echo ""
  echo "===== --set 支持参数 ====="
  echo "  cruise=值(巡航速度)  upper=值  min_stop=值(停车兜底)"
  echo "  stop_distance=值(人行道停车)  tl_stop=值(红灯停车)  expand=值  timeout=值"
  echo "  limit=值(限速带速度,同时改两区段)  forward=值  backward=值"
}

# ---------------- main ----------------

ACTION=""
PLAN=""
SET_KV=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --plan) PLAN="$2"; shift 2;;
    --set)  SET_KV="$2"; shift 2;;
    --rollback) ACTION="rollback"; shift;;
    --backup-only) ACTION="backup"; shift;;
    --list) ACTION="list"; shift;;
    --verify) ACTION="verify"; shift;;
    --restore-default) ACTION="restore_default"; shift;;
    --save-baseline)   ACTION="save_baseline"; shift;;
    --restore-baseline) ACTION="restore_baseline"; shift;;
    *) echo "未知参数: $1"; exit 1;;
  esac
done

case "$ACTION" in
  rollback) ping_car && rollback_car; exit 0;;
  backup)   ping_car && backup_car; exit 0;;
  list)     list_info; exit 0;;
  verify)   ping_car && verify_car; exit 0;;
  restore_default) ping_car && restore_default; exit 0;;
  save_baseline)   ping_car && save_baseline; exit 0;;
  restore_baseline) ping_car && restore_baseline; exit 0;;
esac

if [[ -n "$SET_KV" ]]; then
  ping_car && set_param "$SET_KV"; exit 0
fi

if [[ -n "$PLAN" ]]; then
  ping_car && deploy_plan "$PLAN"; exit 0
fi

# 无参数 -> 交互菜单
ping_car && menu
