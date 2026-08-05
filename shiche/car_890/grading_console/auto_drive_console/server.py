#!/usr/bin/env python3
"""自动驾驶控制台 Web 服务。

只暴露四个功能：自动驾驶、暂停/恢复、退出、读取最新数据包并评测。
仅建议本机使用（默认无鉴权）。

运行: python3 server.py
浏览器: http://localhost:8887
"""

import json
import os
import sys

# 目录/文件名含中文时，若宿主 locale 是 ASCII，Python 的文件系统编码会退化成
# ascii，os.lstat/realpath 对中文路径直接抛 UnicodeEncodeError（表现为 /api
# 全部 500、前端“Failed to fetch”）。这里在启动最早期强制 UTF-8 并重启自身，
# 保证后续所有路径操作都走 UTF-8。
if sys.getfilesystemencoding().lower() not in ("utf-8", "utf8"):
    os.environ["LC_ALL"] = "C.UTF-8"
    os.environ["LANG"] = "C.UTF-8"
    os.environ["PYTHONUTF8"] = "1"
    os.environ["PYTHONIOENCODING"] = "utf-8"
    os.environ["_ADC_UTF8_REEXEC"] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)

from http.server import HTTPServer, BaseHTTPRequestHandler

import socket

from runtime import get_runtime

BASE_DIR = os.path.dirname(os.path.realpath(__file__))
PORT = int(os.environ.get("AUTO_DRIVE_PORT", "8887"))
# 默认绑所有网卡，方便用本机 IP 访问；可用 AUTO_DRIVE_HOST 覆盖回 127.0.0.1。
# 注意：控制台无鉴权且能触发实车动作，绑 0.0.0.0 后同网段任何人都能操作，
# 仅在可信内网使用。
HOST = os.environ.get("AUTO_DRIVE_HOST", "0.0.0.0")
_runtime = get_runtime(BASE_DIR)


def _local_ip():
    """获取本机对外的局域网 IP（不真正发包，仅用于打印访问地址）。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print("[%s] %s" % (self.address_string(), fmt % args))

    def _json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            with open(os.path.join(BASE_DIR, "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/api/status":
            self._json(_runtime.status())
        elif path == "/api/record/latest":
            self._json(_runtime.latest_bag())
        elif path == "/api/evaluate/status":
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            self._json(_runtime.evaluate_status(qs.get("id", [""])[0] or None))
        elif path == "/api/scenario/status":
            self._json(_runtime.scenario_injection_status())
        elif path == "/api/score/history":
            self._json(_runtime.score_history())
        elif path == "/api/presets":
            self._json(_runtime.load_presets())
        elif path == "/api/scenario/endpoint":
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            self._json(_runtime.scenario_end_point(
                qs.get("scenario_file", [""])[0] or None,
                preset_id=qs.get("preset_id", [""])[0] or None))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            data = json.loads(self.rfile.read(length) or b"{}")
        except Exception:
            self._json({"error": "JSON 解析失败"}, 400)
            return
        path = self.path.split("?")[0]
        if path == "/api/auto":
            r = _runtime.auto_drive(data.get("timeout", 25), options=data)
            self._json(r, 200 if r.get("ok") else 503)
        elif path == "/api/pause":
            r = _runtime.pause_auto(data.get("paused", True))
            self._json(r, 200 if r.get("ok") else 409)
        elif path == "/api/stop":
            r = _runtime.stop_auto()
            self._json(r, 200 if r.get("ok") else 409)
        elif path == "/api/evaluate":
            r = _runtime.evaluate_start(data)
            self._json(r, 200 if r.get("ok") else 400)
        elif path == "/api/score/clear":
            self._json(_runtime.clear_history())
        elif path == "/api/presets":
            r = _runtime.save_presets(data)
            self._json(r, 200 if r.get("ok") else 400)
        elif path == "/api/presets/active":
            r = _runtime.set_active_preset(data.get("preset_id"))
            self._json(r, 200 if r.get("ok") else 400)
        elif path == "/api/route":
            r = _runtime.send_route(data)
            self._json(r, 200 if r.get("ok") else 400)
        elif path == "/api/scenario":
            action = str(data.get("action", "")).lower()
            if action == "on":
                r = _runtime.scenario_injection_start(data.get("scenario_file"))
            elif action == "off":
                r = _runtime.scenario_injection_stop()
            else:
                r = {"ok": False, "error": "action 必须是 on 或 off"}
            self._json(r, 200 if r.get("ok") else 400)
        else:
            self._json({"error": "未知路由"}, 404)


if __name__ == "__main__":
    httpd = HTTPServer((HOST, PORT), Handler)
    print("[OK] Auto-drive console started")
    print("[INFO] Bound to %s:%d" % (HOST, PORT))
    if HOST in ("0.0.0.0", "::"):
        print("[INFO] Open (local):   http://localhost:%d" % PORT)
        print("[INFO] Open (this LAN): http://%s:%d" % (_local_ip(), PORT))
    else:
        print("[INFO] Open: http://%s:%d" % (HOST, PORT))
    print("[INFO] Records dir: %s" % _runtime.record_root)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        _runtime.shutdown()
        print("\n[INFO] stopped")
