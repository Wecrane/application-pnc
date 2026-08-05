#!/usr/bin/env python3
"""直接通过 cyber external_command service 发送动作指令。

复用 routing_tool/server.py send_cmd 的做法，不依赖 routing_tool 的 HTTP 服务。
只支持自动驾驶控制台需要的 action 指令：switch_auto / switch_manual / stop / start。
"""

import os
import sys
import threading
import time

_APOLLO_ROOT = os.path.realpath(os.environ.get("APOLLO_ROOT", "/apollo_workspace"))

for _p in (
    os.path.join(_APOLLO_ROOT, ".aem/envroot/opt/apollo/neo/python"),
    os.path.join(_APOLLO_ROOT, "cyber/python"),
):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

CYBER_AVAILABLE = False
_import_error = None
try:
    from cyber.python.cyber_py3 import cyber as _cyber
    from modules.common_msgs.external_command_msgs import (
        action_command_pb2 as _acmd,
        command_status_pb2 as _cstatus,
        lane_follow_command_pb2 as _lfcmd,
    )
    CYBER_AVAILABLE = True
except ImportError as exc:
    _import_error = str(exc)

_ACTION_MAP = {
    "stop": _acmd.STOP if CYBER_AVAILABLE else 5,
    "start": _acmd.START if CYBER_AVAILABLE else 6,
    "switch_manual": _acmd.SWITCH_TO_MANUAL if CYBER_AVAILABLE else 50,
    "switch_auto": _acmd.SWITCH_TO_AUTO if CYBER_AVAILABLE else 51,
}

_node = None
_node_lock = threading.Lock()
_client = None
_lane_follow_client = None
_cid = int(time.time()) % 100000
_cid_lock = threading.Lock()


def _next_id():
    global _cid
    with _cid_lock:
        _cid += 1
        return _cid


def _get_client():
    global _node, _client
    if not CYBER_AVAILABLE:
        return None
    with _node_lock:
        if _client is None:
            try:
                _cyber.init("auto_drive_console")
                _node = _cyber.Node("auto_drive_console_node")
                _client = _node.create_client(
                    "/apollo/external_command/action",
                    _acmd.ActionCommand, _cstatus.CommandStatus)
            except Exception as exc:  # noqa: BLE001 - 保底不让服务崩
                return {"error": "cyber Node/client 初始化失败: %s" % exc}
    return _client


def _get_lane_follow_client():
    global _node, _lane_follow_client
    if not CYBER_AVAILABLE:
        return None
    # 复用同一个 Node（在 _get_client 里创建）。
    if _client is None:
        cl = _get_client()
        if isinstance(cl, dict):
            return cl
    with _node_lock:
        if _lane_follow_client is None:
            try:
                _lane_follow_client = _node.create_client(
                    "/apollo/external_command/lane_follow",
                    _lfcmd.LaneFollowCommand, _cstatus.CommandStatus)
            except Exception as exc:  # noqa: BLE001
                return {"error": "lane_follow client 初始化失败: %s" % exc}
    return _lane_follow_client


def available():
    return CYBER_AVAILABLE


def send_action(command):
    """发送一个 action 指令，返回 {ok, ...}。"""
    if not CYBER_AVAILABLE:
        return {"ok": False, "error": "cyber 不可用: %s" % (_import_error or "未加载")}
    val = _ACTION_MAP.get(str(command).lower())
    if val is None:
        return {"ok": False, "error": "不支持的动作: %s" % command}
    cl = _get_client()
    if cl is None:
        return {"ok": False, "error": "action client 未就绪"}
    if isinstance(cl, dict):
        return {"ok": False, "error": cl["error"]}
    try:
        cmd = _acmd.ActionCommand()
        cmd.header.timestamp_sec = time.time()
        cmd.header.module_name = "auto_drive_console"
        cmd.header.sequence_num = _next_id()
        cmd.command_id = _next_id()
        cmd.command = val
        r = cl.send_request(cmd)
        if r is None:
            return {"ok": False, "error": "无响应 - external_command 是否在运行？"}
        return {"ok": True, "command_id": cmd.command_id, "status": str(r.status)}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": str(exc)}


def send_lane_follow(end_x, end_y, end_heading=None, way_points=None,
                     target_speed=None):
    """发送 LaneFollowCommand 路由（终点 + 可选途经点 / 朝向 / 限速）。"""
    if not CYBER_AVAILABLE:
        return {"ok": False, "error": "cyber 不可用: %s" % (_import_error or "未加载")}
    cl = _get_lane_follow_client()
    if cl is None:
        return {"ok": False, "error": "lane_follow client 未就绪"}
    if isinstance(cl, dict):
        return {"ok": False, "error": cl["error"]}
    try:
        cmd = _lfcmd.LaneFollowCommand()
        cmd.header.timestamp_sec = time.time()
        cmd.header.module_name = "auto_drive_console"
        cmd.header.sequence_num = _next_id()
        cmd.command_id = _next_id()
        cmd.end_pose.x = float(end_x)
        cmd.end_pose.y = float(end_y)
        if end_heading is not None:
            cmd.end_pose.heading = float(end_heading)
        for wp in (way_points or []):
            pose = cmd.way_point.add()
            pose.x = float(wp["x"])
            pose.y = float(wp["y"])
            if wp.get("heading") is not None:
                pose.heading = float(wp["heading"])
        if target_speed:
            cmd.target_speed = float(target_speed)
        r = cl.send_request(cmd)
        if r is None:
            return {"ok": False, "error": "无响应 - external_command 是否在运行？"}
        return {"ok": True, "command_id": cmd.command_id, "status": str(r.status)}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": str(exc)}
