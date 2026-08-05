#!/usr/bin/env bash
# =============================================================================
# connect_car.sh — 一键连接实车
#   输入车辆编号 → 连接隐藏车载Wi-Fi → 免密(master) → 自动挂载SFTP到本地
# 用法：
#   bash connect_car.sh          # 交互输入编号
#   bash connect_car.sh 802      # 直接给后三位
#   bash connect_car.sh 0802     # 给完整编号(自动取后三位)
# =============================================================================
set -euo pipefail

IP="192.168.10.6"
WIFI_PASS="apollo123"
MNT_ROOT="${HOME}/car"

# ---------- 1. 车辆编号 → WiFi 名 ----------
VEH="${1:-}"
if [ -z "${VEH}" ]; then
  read -r -p "请输入车辆编号（后三位即可，如 802）： " VEH
fi
LAST3="${VEH: -3}"
SSID="HQEV${LAST3}"
echo "ℹ️  SSID=${SSID}  密码=${WIFI_PASS}"

# ---------- 2. 连接隐藏 Wi-Fi ----------
echo "==> 连接隐藏 Wi-Fi: ${SSID}"
nmcli radio wifi on || true
if ! nmcli dev wifi connect "${SSID}" password "${WIFI_PASS}" hidden yes; then
  echo "   直接连接失败，改用持久配置方式..."
  nmcli con delete car-wifi >/dev/null 2>&1 || true
  nmcli con add type wifi con-name car-wifi ssid "${SSID}" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "${WIFI_PASS}" wifi.hidden yes
  nmcli con up car-wifi
fi
echo "✅ Wi-Fi 已连接"

# ---------- 3. 等待实车可达（最多 30s） ----------
echo "==> 等待 ${IP} 可达（最多 30s）..."
reachable=0
for i in $(seq 1 30); do
  if ping -c 1 -W 1 "${IP}" >/dev/null 2>&1; then
    echo "✅ ${IP} 可达"
    reachable=1
    break
  fi
  sleep 1
done
if [ "${reachable}" != "1" ]; then
  echo "❌ 30s 内未 ping 通 ${IP}"
  echo "   检查：SSID 是否本赛位车辆 / 计算单元是否已启动 / Wi-Fi 是否连上"
  exit 1
fi

# ---------- 4. 写入 ~/.ssh/config（Host car + ControlMaster 免密复用，笔记本侧合规） ----------
mkdir -p ~/.ssh
if ! grep -q "^Host car$" ~/.ssh/config 2>/dev/null; then
  cat >> ~/.ssh/config <<EOF

Host car
    HostName ${IP}
    User apollo
    IdentityFile ~/.ssh/id_ed25519
    ServerAliveInterval 15
    ServerAliveCountMax 3
    ControlMaster auto
    ControlPath ~/.ssh/cm-%r@%h:%p
    ControlPersist 4h
EOF
  echo "✅ 已写入 ~/.ssh/config (Host car)"
fi

# ---------- 5. 建立免密 master（首次需输一次密码） ----------
if ! ssh -O check car >/dev/null 2>&1; then
  echo "==> 首次连接：请输入实车密码 apollo（输入时不显示，输完回车）"
  ssh -fN car || { echo "❌ 建立 master 连接失败"; exit 1; }
fi
echo "✅ 免密 master 已建立（4 小时内 ssh/scp/sshfs 免密）"

# ---------- 6. 探测工程目录名（现场可能有差异） ----------
PROJ=""
for cand in apollo-edu_9.1.1 apollo-edu-9.1.1 apollo-deu-9.1.1; do   # 实车实测下划线优先
  if ssh car "test -d ~/${cand}" >/dev/null 2>&1; then
    PROJ="${cand}"
    break
  fi
done
PROJ="${PROJ:-apollo-edu_9.1.1}"   # 实车实测为下划线
echo "ℹ️  实车工程目录 = ~/${PROJ}"

# ---------- 7. 挂载 profiles + modules（SFTP 到本地） ----------
mkdir -p "${MNT_ROOT}/profiles" "${MNT_ROOT}/modules"
sshfs car:"${PROJ}/profiles" "${MNT_ROOT}/profiles" \
  -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
sshfs car:"${PROJ}/modules" "${MNT_ROOT}/modules" \
  -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3,dir_cache=yes
echo "✅ 已挂载:"
echo "   ${MNT_ROOT}/profiles"
echo "   ${MNT_ROOT}/modules"

# ---------- 8. 验证 ----------
if ls "${MNT_ROOT}/profiles" >/dev/null 2>&1; then
  echo "✅ 挂载内容可读"
else
  echo "⚠️  挂载内容不可读，可重试: bash shiche/car_connect/umount_car.sh && bash $0 ${LAST3}"
fi

cat <<EOF

==========================================================
✅ 全部就绪！
  A. 本工作区让 AI 改配置（实时写回实车）
  B. 新开 VSCode 窗口打开 ${MNT_ROOT}/profiles 手动看代码
  C. 终端 ssh car 交互
  编译:    bash shiche/car_connect/car_cmd.sh 'buildtool build -p modules/planning/ -j15'
  检查:    bash shiche/car_connect/check_car.sh
  备份:    bash shiche/car_connect/backup_car.sh
  卸载:    bash shiche/car_connect/umount_car.sh
==========================================================
EOF
