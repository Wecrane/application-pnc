#!/usr/bin/env bash
# =============================================================================
# 实车 SSH 免密配置脚本（笔记本侧，合规方案）
# -----------------------------------------------------------------------------
# 目标：输入一次密码后，4 小时内 ssh / scp / sshfs 全部免密
# 合规说明：只在笔记本的 ~/.ssh/config 配置 ControlMaster 连接复用，
#           【不修改实车任何配置】（不写 authorized_keys），
#           符合秩序册"不得修改…SSH 配置及安全策略"红线。
# 用法：bash setup_ssh_config.sh [IP] [用户]
# =============================================================================
set -euo pipefail

IP="${1:-192.168.10.6}"
USER="${2:-apollo}"

mkdir -p ~/.ssh
chmod 700 ~/.ssh

# 1. 生成/复用本地 SSH 密钥（可选，有则跳过）
if [ ! -f ~/.ssh/id_ed25519 ]; then
  ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 >/dev/null
  echo "✅ 已生成密钥 ~/.ssh/id_ed25519"
else
  echo "ℹ️  复用已有密钥 ~/.ssh/id_ed25519"
fi

# 2. 写入 ~/.ssh/config（Host 别名 car）
if ! grep -q "^Host car$" ~/.ssh/config 2>/dev/null; then
  cat >> ~/.ssh/config <<EOF

# ---- 实车挑战赛（自动生成，勿改） ----
Host car
    HostName ${IP}
    User ${USER}
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 15
    ServerAliveCountMax 3
    # 连接复用：首次输密码后，4 小时内新连接免密
    ControlMaster auto
    ControlPath ~/.ssh/cm-%r@%h:%p
    ControlPersist 4h
EOF
  chmod 600 ~/.ssh/config
  echo "✅ 已写入 ~/.ssh/config（Host car，ControlMaster 免密复用 4h）"
else
  echo "ℹ️  ~/.ssh/config 已有 Host car，跳过写入"
fi

echo ""
echo "====================================================================="
echo "👉 首次连接（输一次密码，之后 4 小时内 ssh/scp/sshfs 全部免密）："
echo "      ssh car"
echo ""
echo "👉 连接后确认工程目录真实名称（下划线还是横杠）："
echo "      ssh car 'ls ~'"
echo "====================================================================="
