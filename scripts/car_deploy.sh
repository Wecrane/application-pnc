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
_scp_up() {
  sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$1" "$USER@$HOST:$2"
}
_scp_dn() {
  sshpass -p "$PASS" scp -o StrictHostKeyChecking=no "$USER@$HOST:$1" "$2"
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
  # 上传并覆盖
  _scp_up "$tarball" "/tmp/"
  _ssh "cd $CAR_CFG_ROOT && tar -xzf /tmp/$(basename $tarball) && echo '✅ 已覆盖配置: $CAR_CFG_ROOT'"
  echo ""
  verify_car
  echo ""
  echo ">>> 下一步：请重启 Planning 模块（DreamView 关闭再开启 Planning，或 cyber_launch restart）"
}

# ---------------- 3. 验证 ----------------
verify_car() {
  echo "===== [验证] 车上生效值 ====="
  _ssh "echo '--- planning.conf ---'; grep -E 'default_cruise_speed|planning_upper_speed_limit|min_stop_distance_obstacle' $CAR_CFG_ROOT/modules/planning/planning_component/conf/planning.conf; echo '--- crosswalk ---'; cat $CAR_CFG_ROOT/modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt; echo '--- scene_speed_limit ---'; grep -E 'enabled|limit_speed|forward_buffer|backward_buffer' $CAR_CFG_ROOT/modules/planning/traffic_rules/scene_speed_limit/conf/default_conf.pb.txt"
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
      echo "   支持: cruise / upper / min_stop / stop_distance / expand / timeout / limit / forward / backward"
      exit 1;;
  esac
  _ssh "cd $CAR_CFG_ROOT && sed -i 's|^$match.*|$new|' $file && echo '✅ $file 已改为 $new'"
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
  echo "  R) 回滚到最近备份"
  echo "  V) 仅验证当前车上配置"
  echo "  L) 查看本机计划与参数说明"
  echo "  Q) 退出"
  echo "----------------------------------------------"
  read -rp "  请选择 [A/B/C/R/V/L/Q]: " ch
  case "$ch" in
    A|a) deploy_plan A ;;
    B|b) deploy_plan B ;;
    C|c) deploy_plan C ;;
    R|r) rollback_car ;;
    V|v) ping_car && verify_car ;;
    L|l) list_info ;;
    Q|q) exit 0 ;;
    *) echo "无效选择"; menu ;;
  esac
}

list_info() {
  echo "===== 本机可用计划 ====="
  for d in PLAN_A PLAN_B PLAN_C_兜底 PLAN_D_极限稳; do
    if [[ -d "$LOCAL_PLANS/$d" ]]; then
      echo "  - $d"
      grep -hE "default_cruise_speed|planning_upper|min_stop" "$LOCAL_PLANS/$d/modules/planning/planning_component/conf/planning.conf" 2>/dev/null | sed 's/^/      /'
    fi
  done
  echo ""
  echo "===== --set 支持参数 ====="
  echo "  cruise=值(巡航速度)  upper=值  min_stop=值(停车兜底)"
  echo "  stop_distance=值(人行道停车)  expand=值  timeout=值"
  echo "  limit=值(限速带速度,同时改两区段)  forward=值  backward=值"
}

# ---------------- main ----------------
need_sshpass

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
    *) echo "未知参数: $1"; exit 1;;
  esac
done

case "$ACTION" in
  rollback) ping_car && rollback_car; exit 0;;
  backup)   ping_car && backup_car; exit 0;;
  list)     list_info; exit 0;;
  verify)   ping_car && verify_car; exit 0;;
esac

if [[ -n "$SET_KV" ]]; then
  ping_car && set_param "$SET_KV"; exit 0
fi

if [[ -n "$PLAN" ]]; then
  ping_car && deploy_plan "$PLAN"; exit 0
fi

# 无参数 -> 交互菜单
ping_car && menu
