#!/usr/bin/env bash
# ============================================================
# 实车 DreamView 一键启动脚本（苏州实车赛 · 2026-08-07）
#
# 功能：
#   1) SSH 连接实车（免密优先 / sshpass 兜底）
#   2) 自动探测工程目录（apollo-edu_9.1.1 / apollo-edu-9.1.1）
#   3) 幂等启动容器（aem start，已启动则跳过）
#   4) 启动 DreamView+（aem bootstrap start --plus）
#   5) 验证 8888 端口
#   6) 提示浏览器访问地址
#
# 用法：
#   ./start_dreamview.sh                # 连接实车并启动（默认 192.168.10.6 / apollo）
#   HOST=127.0.0.1 ./start_dreamview.sh # 本机模拟测试（需本地有 aem）
#   CAR_PROJ=apollo-edu_9.1.1 ./start_dreamview.sh  # 指定工程目录
#   ./start_dreamview.sh --verify-only  # 只检查 DreamView 是否已在运行
#
# 可选环境变量：HOST / USER / PASS / CAR_PROJ
#   注意：SSH 密码默认 apollo（WiFi 才是 apollo123）
# ============================================================

set -uo pipefail

HOST="${HOST:-192.168.10.6}"
USER="${USER:-apollo}"
PASS="${PASS:-apollo}"
CAR_PROJ="${CAR_PROJ:-}"
VERIFY_ONLY="${1:-}"

# ---------------- 连接工具（与 car_deploy.sh 一致） ----------------
_ssh() {
  ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "$@" 2>/dev/null \
    || sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "$@"
}

need_sshpass() {
  if ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$USER@$HOST" "echo ok" >/dev/null 2>&1; then
    return 0
  fi
  if ! command -v sshpass >/dev/null 2>&1; then
    echo "❌ 未安装 sshpass 且未建立免密。先执行一次：ssh $USER@$HOST（密码 apollo）"
    exit 1
  fi
}

# ---------------- 自动探测工程目录 ----------------
detect_project() {
  if [[ -n "$CAR_PROJ" ]]; then return 0; fi
  local found
  found=$(ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "ls -d ~/apollo-edu* 2>/dev/null | head -1 | xargs -n1 basename" 2>/dev/null)
  if [[ -z "$found" ]]; then
    found=$(sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$USER@$HOST" "ls -d ~/apollo-edu* 2>/dev/null | head -1 | xargs -n1 basename" 2>/dev/null)
  fi
  [[ -z "$found" ]] && { echo "❌ 未探测到工程目录，请 CAR_PROJ=目录名 指定"; exit 1; }
  CAR_PROJ="$found"
  echo "✅ 工程目录: $CAR_PROJ"
}

# ---------------- 验证 DreamView 状态 ----------------
check_dv() {
  local out
  out=$(_ssh "ss -tln 2>/dev/null | grep -q ':8888 ' && echo LISTENING || echo NOT_LISTENING")
  case "$out" in
    *LISTENING*) echo "✅ DreamView 8888 端口正在监听"; return 0;;
    *) echo "⚠️  DreamView 8888 端口未监听"; return 1;;
  esac
}

# ---------------- 主流程 ----------------
echo "==> 连接实车 $USER@$HOST ..."
need_sshpass

if [[ "$VERIFY_ONLY" == "--verify-only" ]]; then
  _ssh "echo OK" >/dev/null 2>&1 || { echo "❌ 无法连接实车"; exit 1; }
  echo "✅ 连接成功"
  detect_project
  check_dv
  echo "   浏览器访问: http://$HOST:8888"
  exit 0
fi

# 1. 连接 + 工程目录
if ! _ssh "echo OK" >/dev/null 2>&1; then
  echo "❌ 无法连接实车。请检查：WiFi(HQEVxxx) / IP / 计算单元是否就绪"
  exit 1
fi
echo "✅ 连接成功"
detect_project

# 2. 检查是否已在运行
if check_dv; then
  echo "   DreamView 已在运行，无需重复启动。浏览器访问: http://$HOST:8888"
  exit 0
fi

# 3. 启动容器（幂等：已启动则 aem start 无副作用）+ 启动 DreamView
echo "==> 启动容器 + DreamView+ ..."
echo "   （aem bootstrap start --plus 会拉取/校验环境，首次较慢，请耐心等待）"
ssh -t -o StrictHostKeyChecking=no -o ConnectTimeout=15 "$USER@$HOST" \
  "cd ~/$CAR_PROJ && (docker ps 2>/dev/null | grep -q apollo || aem start) 2>/dev/null; aem bootstrap start --plus"

# 4. 验证
sleep 3
echo ""
if check_dv; then
  echo ""
  echo "=============================================="
  echo "  🎉 DreamView+ 启动完成！"
  echo "  笔记本访问:  http://$HOST:8888"
  echo "  （直连屏幕:  http://localhost:8888）"
  echo "  登录后：选场景 → 开模块 → vehicle test 模式"
  echo "=============================================="
else
  echo "⚠️  8888 端口尚未确认监听，请等待几秒后重试：./start_dreamview.sh --verify-only"
  echo "   或直接在浏览器打开 http://$HOST:8888 确认"
fi
