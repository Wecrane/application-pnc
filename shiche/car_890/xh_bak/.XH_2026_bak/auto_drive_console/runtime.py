#!/usr/bin/env python3
"""自动驾驶控制台后端：只做四件事。

1. 自动驾驶：检查/启动 Planning、Control、External Command，开始落盘，切自动。
2. 暂停 / 恢复行驶：routing_tool action=stop / start，落盘保持进行。
3. 退出自动驾驶：切回手动，停止落盘，返回数据包路径。
4. 读取最新数据包并评测出分。

浏览器提交的参数不会变成任意 shell 命令。
"""

import json
import io
import os
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
import concurrent.futures
from urllib import error as urlerror
from urllib import request as urlrequest

import cyber_action

# 评分所需 + PnC 复盘频道。
CHANNELS = (
    "/apollo/planning", "/apollo/planning/pad", "/apollo/routing_response",
    "/apollo/prediction", "/apollo/perception/obstacles",
    "/apollo/perception/traffic_light", "/apollo/localization/pose",
    "/apollo/canbus/chassis", "/apollo/control",
    "/apollo/external_command/lane_follow",
)
MODULES = {
    "planning": "/apollo/modules/planning/planning_component/dag/planning.dag",
    "control": "/apollo/modules/control/control_component/dag/control.dag",
    "external_command": "/apollo/modules/external_command/process_component/dag/external_command_process.dag",
}


class Runtime:
    def __init__(self, base_dir):
        self.base_dir = os.path.realpath(base_dir)
        self.apollo_root = os.path.realpath(os.environ.get("APOLLO_ROOT", "/apollo_workspace"))
        self.record_root = os.path.join(self.base_dir, "records")
        self.dry_run = os.environ.get("APOLLO_RUNTIME_DRY_RUN", "").lower() in ("1", "true", "yes")
        self._lock = threading.RLock()
        self._auto_lock = threading.Lock()
        self._owned = {}
        self._simulated = set()
        self._record = None
        self._auto_state = "inactive"   # inactive/starting/running/paused/error
        self._auto_record_id = None
        self._auto_error = None
        self._updated_at = time.time()
        self._jobs = {}
        self._scenario_proc = None
        self._scenario_file = None
        self._history_file = os.path.join(self.record_root, "score_history.json")
        self._preset_file = os.path.join(self.base_dir, "presets.json")

    # ------------------------------ helpers ---------------------------
    def _send_action(self, command):
        """直接经 cyber external_command service 发送动作指令。"""
        return cyber_action.send_action(command)

    def _process_online(self, module):
        try:
            r = subprocess.run(["pgrep", "-f", "mainboard.*" + module],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True, timeout=2)
            return r.returncode == 0 and bool(r.stdout.strip())
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _module_online(self, name):
        return name in self._simulated or self._process_online(name)

    def _start_module(self, name):
        if self.dry_run:
            self._simulated.add(name)
            return {"ok": True, "name": name, "dry_run": True}
        dag = os.path.realpath(MODULES[name])
        if not os.path.isfile(dag):
            return {"ok": False, "name": name, "error": "DAG 不存在: " + dag}
        try:
            proc = subprocess.Popen(["mainboard", "-d", dag], cwd=self.apollo_root,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                    start_new_session=True)
            self._owned[name] = proc
            return {"ok": True, "name": name, "pid": proc.pid}
        except OSError as exc:
            return {"ok": False, "name": name, "error": str(exc)}

    # ------------------------------ status ----------------------------
    def status(self):
        cyber_ok = cyber_action.available()
        ext_online = self._module_online("external_command")
        modules = {name: {"online": self._module_online(name)} for name in MODULES}
        return {
            "ok": True,
            "timestamp": time.time(),
            "dry_run": self.dry_run,
            "command_bridge": {
                "available": cyber_ok,
                "reachable": cyber_ok and ext_online,
            },
            "modules": modules,
            "auto": {
                "state": self._auto_state,
                "record_id": self._auto_record_id,
                "last_error": self._auto_error,
                "updated_at": self._updated_at,
            },
            "record": self.record_status(),
        }

    def _set_auto(self, state, error=None):
        self._auto_state = state
        self._auto_error = error
        self._updated_at = time.time()

    # ------------------------------ auto drive ------------------------
    def auto_drive(self, timeout=20, options=None):
        options = options or {}
        options, preset_err = self._apply_preset_to_options(options)
        if preset_err:
            return {"ok": False, "error": preset_err, "status": self.status()}
        try:
            timeout = max(3.0, min(float(timeout), 90.0))
        except (TypeError, ValueError):
            timeout = 20.0
        if not self._auto_lock.acquire(False):
            return {"ok": False, "error": "操作进行中，请稍候", "code": "busy"}
        try:
            if self._auto_state == "running":
                return {"ok": True, "stage": "already_running", "status": self.status()}
            if self._auto_state == "paused":
                return {"ok": False, "error": "当前处于暂停状态，请点击恢复行驶", "status": self.status()}
            self._set_auto("starting")
            started = []
            for name in MODULES:
                if not self._module_online(name):
                    res = self._start_module(name)
                    if not res.get("ok"):
                        self._set_auto("error", res.get("error"))
                        return {"ok": False, "stage": "start", "error": res.get("error"), "status": self.status()}
                    started.append(res)
            deadline = time.time() + timeout
            while time.time() < deadline:
                if all(self._module_online(n) for n in ("planning", "control")):
                    break
                time.sleep(0.5)
            if not all(self._module_online(n) for n in ("planning", "control")):
                self._set_auto("error", "Planning 或 Control 未在超时内上线")
                return {"ok": False, "stage": "wait", "error": self._auto_error, "status": self.status()}

            rec = self.record_start("auto-drive")
            if not rec.get("ok"):
                self._set_auto("error", rec.get("error", "落盘启动失败"))
                return {"ok": False, "stage": "record", "error": self._auto_error, "status": self.status()}
            self._auto_record_id = rec["record"].get("id")

            cmd = self._send_action("switch_auto")
            if not cmd.get("ok"):
                self.record_stop()
                self._auto_record_id = None
                self._set_auto("error", cmd.get("error", "自动驾驶指令失败"))
                return {"ok": False, "stage": "switch_auto", "error": self._auto_error, "status": self.status()}
            self._set_auto("running")
            result = {"ok": True, "stage": "ready", "started": started,
                      "record": rec["record"], "status": self.status()}

            # 可选：一键在开启自动驾驶后顺带做「场景注入」和「终点路由下发」，
            # 省去用户再去单独点按钮。任一子步骤失败不回滚自动驾驶，仅在结果里标注。
            inject = options.get("scenario_action")
            if str(inject or "").strip().lower() == "on":
                result["scenario"] = self.scenario_injection_start(
                    options.get("scenario_file"))
            end_x = options.get("end_x")
            end_y = options.get("end_y")
            # 想下发路由但没填终点时，从该赛段的场景文件里读终点，
            # 保证选哪个赛段就走哪个赛段的终点。
            want_route = bool(options.get("want_route")) \
                or str(inject or "").strip().lower() == "on"
            if (end_x in (None, "") or end_y in (None, "")) and want_route:
                ep = self.scenario_end_point(options.get("scenario_file"))
                if ep.get("ok"):
                    end_x, end_y = ep["end_x"], ep["end_y"]
                    if options.get("end_heading") in (None, "") \
                            and ep.get("end_heading") is not None:
                        options["end_heading"] = ep["end_heading"]
                    result["end_from_scenario"] = ep.get("scenario_file")
                else:
                    result["route"] = {"ok": False,
                                       "error": "未填终点且无法从场景文件读取: %s"
                                                % ep.get("error")}
            if end_x is not None and end_y is not None and str(end_x) != "" \
                    and str(end_y) != "":
                try:
                    route = cyber_action.send_lane_follow(
                        float(end_x), float(end_y),
                        end_heading=options.get("end_heading"),
                        way_points=options.get("way_points"),
                        target_speed=options.get("target_speed"))
                except (TypeError, ValueError) as exc:
                    route = {"ok": False, "error": "终点坐标非法: %s" % exc}
                result["route"] = route
            if options.get("_preset"):
                result["preset"] = options["_preset"]
            return result
        finally:
            self._auto_lock.release()

    def pause_auto(self, paused):
        target = bool(paused)
        if not self._auto_lock.acquire(False):
            return {"ok": False, "error": "操作进行中，请稍候", "code": "busy"}
        try:
            if target and self._auto_state == "paused":
                return {"ok": True, "state": "paused", "status": self.status()}
            if not target and self._auto_state == "running":
                return {"ok": True, "state": "running", "status": self.status()}
            expected = "running" if target else "paused"
            if self._auto_state != expected:
                return {"ok": False, "error": "当前不能执行该操作，状态为 " + self._auto_state, "status": self.status()}
            action = "stop" if target else "start"
            cmd = self._send_action(action)
            if not cmd.get("ok"):
                self._auto_error = cmd.get("error", "车辆动作指令失败")
                self._updated_at = time.time()
                return {"ok": False, "error": self._auto_error, "status": self.status()}
            self._set_auto("paused" if target else "running")
            return {"ok": True, "state": self._auto_state, "status": self.status()}
        finally:
            self._auto_lock.release()

    def stop_auto(self):
        if not self._auto_lock.acquire(False):
            return {"ok": False, "error": "操作进行中，请稍候", "code": "busy"}
        try:
            if self._auto_state in ("inactive", "idle"):
                return {"ok": False, "error": "当前没有进行中的自动驾驶", "status": self.status()}
            cmd = self._send_action("switch_manual")
            rec_result = None
            if self._auto_record_id and self._record:
                rec_result = self.record_stop()
            self._auto_record_id = None
            # 退出自动驾驶时一并关闭场景注入，避免注入进程残留继续发障碍物。
            scenario_stop = self.scenario_injection_stop()
            self._set_auto("inactive", None if cmd.get("ok") else cmd.get("error"))
            bag = rec_result["record"].get("bag_file") if rec_result and rec_result.get("ok") else None
            return {"ok": True, "state": self._auto_state, "routing": cmd,
                    "scenario": scenario_stop,
                    "bag_file": bag, "status": self.status()}
        finally:
            self._auto_lock.release()

    # ------------------------------ routing + scenario injection ------
    def send_route(self, options):
        """发送终点路由（LaneFollowCommand），可选同时开/关场景注入。

        options:
          end_x / end_y (必填), end_heading (可选), target_speed (可选),
          way_points (可选 [{x,y,heading?}]),
          scenario_action (可选): 'on' 启动场景注入, 'off' 关闭, 省略则不动;
          scenario_file (scenario_action=='on' 时的场景 JSON 路径);
          preset_id (可选): 用赛段预设补全 scenario_file 与终点。

        end_x/end_y 留空时，自动从 scenario_file（或预设的场景文件）里的
        scenario.end 读终点，保证「选哪个赛段就走哪个赛段的终点」。
        """
        options, preset_err = self._apply_preset_to_options(options or {})
        if preset_err:
            return {"ok": False, "error": preset_err}

        end_x, end_y = options.get("end_x"), options.get("end_y")
        x_blank = end_x in (None, "")
        y_blank = end_y in (None, "")
        if x_blank != y_blank:
            # 只填一个坐标多半是手误，不能静默丢弃已填的那个去读场景终点。
            return {"ok": False,
                    "error": "end_x / end_y 要么都填，要么都留空（都留空则读场景终点）"}
        if x_blank:
            ep = self.scenario_end_point(options.get("scenario_file"))
            if not ep.get("ok"):
                return {"ok": False,
                        "error": "未填终点坐标，且无法从场景文件读取终点: %s"
                                 % ep.get("error")}
            end_x, end_y = ep["end_x"], ep["end_y"]
            if options.get("end_heading") in (None, "") and ep.get("end_heading") is not None:
                options["end_heading"] = ep["end_heading"]
            options["_end_from_scenario"] = ep.get("scenario_file")
        try:
            end_x = float(end_x)
            end_y = float(end_y)
        except (TypeError, ValueError):
            return {"ok": False, "error": "终点坐标 end_x/end_y 需为数字"}

        result = {"ok": True}
        if options.get("_end_from_scenario"):
            result["end_from_scenario"] = options["_end_from_scenario"]
        if options.get("_preset"):
            result["preset"] = options["_preset"]

        # 先按需处理场景注入，再发路由，保证注入的障碍物在路由生效时已经在发。
        action = str(options.get("scenario_action", "")).strip().lower()
        if action == "on":
            inj = self.scenario_injection_start(options.get("scenario_file"))
            result["scenario"] = inj
            if not inj.get("ok"):
                return {"ok": False, "error": "场景注入启动失败: %s" % inj.get("error"),
                        "scenario": inj}
        elif action == "off":
            result["scenario"] = self.scenario_injection_stop()

        route = cyber_action.send_lane_follow(
            end_x, end_y,
            end_heading=options.get("end_heading"),
            way_points=options.get("way_points"),
            target_speed=options.get("target_speed"),
        )
        result["route"] = route
        result["end_x"], result["end_y"] = end_x, end_y
        if not route.get("ok"):
            return {"ok": False, "error": route.get("error", "路由下发失败"),
                    "route": route, "scenario": result.get("scenario"),
                    "end_x": end_x, "end_y": end_y,
                    "end_from_scenario": result.get("end_from_scenario")}
        return result

    def scenario_end_point(self, scenario_file=None, preset_id=None):
        """从场景 JSON 读取终点坐标 scenario.end.{x,y}，供前端自动填充。"""
        if not scenario_file and preset_id:
            preset, err = self.get_preset(preset_id)
            if err:
                return {"ok": False, "error": err}
            scenario_file = preset.get("scenario_file")
        sf = scenario_file or os.path.join(
            self.apollo_root, "scenario_injection", "1.json")
        sf = os.path.realpath(sf)
        if not os.path.isfile(sf):
            return {"ok": False, "error": "场景文件不存在: %s" % sf}
        try:
            with io.open(sf, "r", encoding="utf-8-sig") as f:
                data = json.load(f)
        except (OSError, ValueError) as exc:
            return {"ok": False, "error": "读取场景文件失败: %s" % exc}
        scen = data.get("scenario", data)
        end = scen.get("end") if isinstance(scen, dict) else None
        if not isinstance(end, dict) or "x" not in end or "y" not in end:
            return {"ok": False, "error": "场景文件缺少 scenario.end.x/y"}
        return {"ok": True, "end_x": end["x"], "end_y": end["y"],
                "end_heading": end.get("heading"), "scenario_file": sf}

    def scenario_injection_start(self, scenario_file=None):
        """启动 scenario_injection/scenario_injector.py 注入进程。"""
        with self._lock:
            proc = self._scenario_proc
            if proc is not None and proc.poll() is None:
                return {"ok": True, "state": "already_running", "pid": proc.pid,
                        "scenario_file": self._scenario_file}

            injector = os.path.join(
                self.apollo_root, "scenario_injection", "scenario_injector.py")
            if not os.path.isfile(injector):
                return {"ok": False, "error": "找不到注入脚本: %s" % injector}

            sf = scenario_file or os.path.join(
                self.apollo_root, "scenario_injection", "1.json")
            sf = os.path.realpath(sf)
            if not os.path.isfile(sf):
                return {"ok": False, "error": "场景文件不存在: %s" % sf}

            if self.dry_run:
                self._scenario_file = sf
                return {"ok": True, "state": "dry_run", "scenario_file": sf}

            # 启动前清扫残留 sim_obstacle：cyber 节点名唯一，若上次没杀干净，
            # 新进程会因 "duplicated node[sim_obstacle]" 被立即终止（注入失效）。
            self._sweep_sim_obstacle(sweep_all=True)
            time.sleep(1.0)

            env = dict(os.environ)
            env.setdefault("PYTHONIOENCODING", "utf-8")
            env.setdefault("LC_ALL", "C.UTF-8")
            env.setdefault("LANG", "C.UTF-8")
            env.setdefault("APOLLO_ROOT_DIR", self.apollo_root)
            log_path = os.path.join(self.record_root, "scenario_injection.log")
            try:
                os.makedirs(self.record_root, exist_ok=True)
                logf = open(log_path, "wb")
            except OSError:
                logf = None
            try:
                proc = subprocess.Popen(
                    [sys.executable, "-u", injector, sf],
                    cwd=os.path.dirname(injector), env=env,
                    stdout=(logf or subprocess.DEVNULL),
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid)
            except OSError as exc:
                if logf:
                    logf.close()
                return {"ok": False, "error": "无法启动注入进程: %s" % exc}
            if logf:
                logf.close()
            # 探活：注入脚本若启动即失败（缺 sim_obstacle/依赖/坐标错误），
            # 会很快退出。等 1.5s 后确认存活，否则回读日志报错。
            time.sleep(1.5)
            if proc.poll() is not None:
                tail = ""
                try:
                    with io.open(log_path, "r", encoding="utf-8",
                                 errors="replace") as f:
                        tail = f.read()[-1500:]
                except OSError:
                    pass
                return {"ok": False,
                        "error": "注入进程启动即退出(码 %s): %s"
                                 % (proc.returncode, tail.strip() or "无输出"),
                        "log": log_path}
            self._scenario_proc = proc
            self._scenario_file = sf
            return {"ok": True, "state": "started", "pid": proc.pid,
                    "scenario_file": sf, "log": log_path}

    def scenario_injection_stop(self):
        """停止场景注入进程。

        注入器 scenario_injector.py 把 sim_obstacle 放在自己独立的会话里
        (preexec_fn=os.setsid)，且只在收到 SIGINT 时才优雅清理 sim_obstacle。
        因此先发 SIGINT 让它自己杀掉 sim_obstacle 子进程，再兜底扫残留的
        sim_obstacle，最后 SIGTERM/SIGKILL 收尾，避免注入进程或 sim_obstacle 泄漏。
        """
        with self._lock:
            proc = self._scenario_proc
            scenario_file = self._scenario_file
            if proc is None or proc.poll() is not None:
                self._scenario_proc = None
                self._sweep_sim_obstacle(scenario_file)
                return {"ok": True, "state": "not_running"}
            try:
                pgid = os.getpgid(proc.pid)
                # 1) SIGINT：触发 run_forever 的 KeyboardInterrupt -> injector.stop()
                os.killpg(pgid, signal.SIGINT)
                try:
                    proc.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    # 2) 仍未退出：SIGTERM 再等
                    os.killpg(pgid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        # 3) 最后 SIGKILL
                        os.killpg(pgid, signal.SIGKILL)
                        proc.wait()
            except OSError as exc:
                # 进程可能已退出；继续兜底清理 sim_obstacle。
                pass
            self._scenario_proc = None
            # 兜底：SIGTERM 路径不会触发 injector 的清理，sim_obstacle 可能仍在。
            self._sweep_sim_obstacle(scenario_file)
            return {"ok": True, "state": "stopped"}

    def _sweep_sim_obstacle(self, scenario_file=None, sweep_all=False):
        """兜底清理残留的 sim_obstacle 进程（注入器崩溃/被强杀时可能泄漏）。

        默认只清理命令行里带本次场景文件路径的进程，避免误杀无关 sim_obstacle。
        sweep_all=True 时清理所有 sim_obstacle（启动前用，因 cyber 节点名唯一，
        任何残留都会让新进程因 duplicated node 被终止）。
        """
        if self.dry_run:
            return
        if not sweep_all and not scenario_file:
            return
        try:
            out = subprocess.check_output(
                ["pgrep", "-af", "sim_obstacle"], stderr=subprocess.DEVNULL)
        except (subprocess.CalledProcessError, OSError):
            return
        for line in out.decode("utf-8", "replace").splitlines():
            parts = line.strip().split(None, 1)
            if len(parts) != 2:
                continue
            pid_str, cmdline = parts
            # 只认真正的 sim_obstacle 可执行行，避免误杀 pgrep/grep 自身等。
            if "/bin/sim_obstacle" not in cmdline and not cmdline.startswith("sim_obstacle"):
                if "sim_obstacle" not in os.path.basename(cmdline.split()[0]):
                    continue
            if not sweep_all and scenario_file not in cmdline:
                continue
            try:
                pid = int(pid_str)
            except ValueError:
                continue
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass

    def scenario_injection_status(self):
        with self._lock:
            proc = self._scenario_proc
            running = proc is not None and proc.poll() is None
            return {"ok": True, "running": running,
                    "pid": proc.pid if running else None,
                    "scenario_file": self._scenario_file if running else None}

    def _record_snapshot(self):
        if not self._record:
            return {"state": "idle", "running": False, "id": None, "output": None}
        rec = dict(self._record)
        proc = rec.pop("process", None)
        if proc is not None:
            rec["pid"] = proc.pid
            rec["returncode"] = proc.poll()
            rec["running"] = proc.poll() is None
        else:
            rec["pid"] = None
            rec["returncode"] = None
            rec["running"] = True
        rec["state"] = "recording" if rec.get("running") else "stopped"
        return rec

    def record_status(self):
        with self._lock:
            return self._record_snapshot()

    def record_start(self, name="run"):
        with self._lock:
            if self._record_snapshot().get("running"):
                return {"ok": False, "error": "已有落盘任务运行中", "record": self._record_snapshot()}
            clean = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name or "run"))[:48] or "run"
            stamp = time.strftime("%Y%m%d-%H%M%S") + "-" + clean
            # 每次录包一个独立子目录，cyber_recorder 的分段文件都落在里面。
            run_dir = os.path.join(self.record_root, stamp)
            prefix = os.path.join(run_dir, clean)  # -o 是文件前缀，不是目录
            try:
                if not self.dry_run:
                    os.makedirs(run_dir, exist_ok=True)
                else:
                    os.makedirs(self.record_root, exist_ok=True)
            except OSError as exc:
                return {"ok": False, "error": "无法创建落盘目录: " + str(exc)}
            argv = [os.environ.get("APOLLO_RECORDER_BIN", "cyber_recorder"),
                    "record", "-o", prefix]
            for ch in CHANNELS:
                argv.extend(["-c", ch])
            # 用超长分段间隔，尽量让整段录制落在单个 record 文件里，便于评测。
            argv.extend(["-i", os.environ.get("APOLLO_RECORD_SEGMENT_SEC", "86400")])
            proc = None
            if not self.dry_run:
                try:
                    proc = subprocess.Popen(argv, cwd=self.apollo_root,
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                            universal_newlines=True, start_new_session=True)
                except OSError as exc:
                    return {"ok": False, "error": "无法启动 cyber_recorder: " + str(exc)}
            self._record = {"id": uuid.uuid4().hex[:12], "name": clean,
                            "output": run_dir, "prefix": prefix,
                            "channels": list(CHANNELS), "started_at": time.time(),
                            "process": proc, "dry_run": self.dry_run}
            return {"ok": True, "record": self._record_snapshot()}

    def record_stop(self):
        with self._lock:
            if not self._record:
                return {"ok": False, "error": "当前没有落盘任务"}
            proc = self._record.get("process")
            if proc is not None and proc.poll() is None:
                for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 0)):
                    try:
                        os.killpg(proc.pid, sig)
                        if wait:
                            proc.wait(timeout=wait)
                        break
                    except (OSError, subprocess.TimeoutExpired):
                        continue
            snap = self._record_snapshot()
            snap["stopped_at"] = time.time()
            snap["bag_file"] = self._find_record_file(self._record.get("output"))
            self._record = None
            return {"ok": True, "record": snap}

    @staticmethod
    def _find_record_files(run_dir):
        """列出一次录包目录里所有 record 分段文件，按分段序号排序。

        cyber_recorder 分段录制会生成 <prefix>.00000.<ts>、<prefix>.00001.<ts> …
        它们合起来才是完整录包，必须全部评测，不能只取第一个分段。
        """
        if not run_dir or not os.path.isdir(run_dir):
            return []
        files = [os.path.join(run_dir, n) for n in os.listdir(run_dir)
                 if os.path.isfile(os.path.join(run_dir, n))]
        # 优先按文件名里的分段序号 .NNNNN. 排序，回退到文件名。
        def _seg_key(path):
            m = re.search(r"\.(\d{5})\.", os.path.basename(path))
            return (0, int(m.group(1))) if m else (1, os.path.basename(path))
        files.sort(key=_seg_key)
        return files

    @staticmethod
    def _find_record_file(run_dir):
        """兼容旧调用：返回目录里第一个分段（供 latest_bag 展示用）。"""
        files = Runtime._find_record_files(run_dir)
        return files[0] if files else run_dir

    def _collect_bags(self):
        """按修改时间倒序收集 records/ 下每个录包目录。

        每个目录 = 一次连续录制 = 一个评测单元；目录内的多个分段
        (.00000/.00001…) 属于同一段连续数据，用 os.pathsep 拼接后交给评测器
        连续读取，不拆成多个独立 bag。
        """
        if not os.path.isdir(self.record_root):
            return []
        subdirs = [(os.path.getmtime(os.path.join(self.record_root, n)),
                    os.path.join(self.record_root, n))
                   for n in os.listdir(self.record_root)
                   if os.path.isdir(os.path.join(self.record_root, n))]
        subdirs.sort(reverse=True)
        bags = []
        for mtime, d in subdirs:
            segments = self._find_record_files(d)
            if not segments:
                continue
            bags.append({"bag_file": os.pathsep.join(segments),
                         "name": os.path.basename(d),
                         "segments": len(segments),
                         "modified_at": mtime})
        return bags

    def latest_bag(self):
        try:
            bags = self._collect_bags()
            if not bags:
                return {"ok": False, "error": "落盘目录暂无数据包"}
            # 保留字段兼容旧前端，同时返回全部数据包供整目录评测。
            top = bags[0]
            return {"ok": True, "bag_file": top["bag_file"], "name": top["name"],
                    "modified_at": top["modified_at"], "count": len(bags),
                    "bags": bags}
        except OSError as exc:
            return {"ok": False, "error": "读取落盘目录失败: " + str(exc)}

    # ------------------------------ presets ---------------------------
    _PRESET_PATH_KEYS = ("scenario_file", "scene_manifest", "config")

    def load_presets(self):
        """读取赛段预设（presets.json）。文件缺失/损坏时返回空预设而不报错。"""
        empty = {"active": None, "presets": []}
        with self._lock:
            try:
                with io.open(self._preset_file, "r", encoding="utf-8-sig") as f:
                    data = json.load(f)
            except (OSError, ValueError) as exc:
                return dict(empty, ok=True, file=self._preset_file,
                            warning="读取预设失败，已按空预设处理: %s" % exc)
            if not isinstance(data, dict) or not isinstance(data.get("presets"), list):
                return dict(empty, ok=True, file=self._preset_file,
                            warning="presets.json 格式不正确（需要 {presets: [...]}）")
            presets = []
            for item in data["presets"]:
                if not isinstance(item, dict):
                    continue
                pid = str(item.get("id") or "").strip()
                if not pid:
                    continue
                p = {"id": pid,
                     "label": str(item.get("label") or pid).strip()}
                for key in self._PRESET_PATH_KEYS:
                    val = item.get(key)
                    if val:
                        p[key] = str(val)
                        p[key + "_exists"] = os.path.isfile(os.path.realpath(str(val)))
                presets.append(p)
            active = data.get("active")
            if active not in [p["id"] for p in presets]:
                active = presets[0]["id"] if presets else None
            return {"ok": True, "file": self._preset_file,
                    "active": active, "presets": presets}

    def get_preset(self, preset_id=None):
        """取某个预设（默认取 active），返回 (preset_dict | None, error)。"""
        info = self.load_presets()
        pid = preset_id or info.get("active")
        if not pid:
            return None, "presets.json 里没有可用预设"
        for p in info.get("presets", []):
            if p["id"] == pid:
                return p, None
        return None, "预设不存在: %s" % pid

    @staticmethod
    def _parse_planning_gflags(conf_file):
        """解析 planning.conf 里的 gflag（形如 --key=value）。返回 {key: value_str}。

        文件缺失/读失败返回空 dict（由调用方决定如何处理）。
        """
        flags = {}
        try:
            with io.open(conf_file, "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    s = line.strip()
                    if not s or s.startswith("#") or not s.startswith("--"):
                        continue
                    body = s[2:]
                    if "=" in body:
                        k, v = body.split("=", 1)
                        flags[k.strip()] = v.strip()
                    else:
                        flags[body.strip()] = "true"
        except OSError:
            return {}
        return flags

    def _load_cruise_speed_check(self):
        """读取 presets.json 顶层的 cruise_speed_check 配置（可选）。"""
        with self._lock:
            try:
                with io.open(self._preset_file, "r", encoding="utf-8-sig") as f:
                    data = json.load(f)
            except (OSError, ValueError):
                return None
        cfg = data.get("cruise_speed_check") if isinstance(data, dict) else None
        return cfg if isinstance(cfg, dict) else None

    def check_cruise_speed_gate(self):
        """按 presets.json 的 cruise_speed_check 检查 planning.conf 巡航速度门槛。

        返回 dict：
          {"enabled": bool, "failed": bool, "scope": "cruise"|"all",
           "reason": str, "flags": {...}, "checks": [...]}
        enabled=False 或配置缺失时 failed=False（不拦截）。
        """
        cfg = self._load_cruise_speed_check()
        if not cfg or not cfg.get("enabled", False):
            return {"enabled": False, "failed": False}
        conf_file = cfg.get(
            "conf_file",
            "/apollo/modules/planning/planning_component/conf/planning.conf")
        scope = str(cfg.get("scope", "cruise")).lower()
        if scope not in ("cruise", "all"):
            scope = "cruise"
        flags = self._parse_planning_gflags(conf_file)
        result = {"enabled": True, "failed": False, "scope": scope,
                  "conf_file": conf_file, "flags": {}, "checks": [], "reason": ""}
        # 每个字段可配下限(min_)和/或上限(max_)，两者都可省略。
        gate = [
            ("default_cruise_speed",
             "min_default_cruise_speed", "max_default_cruise_speed"),
            ("planning_upper_speed_limit",
             "min_planning_upper_speed_limit", "max_planning_upper_speed_limit"),
        ]
        failures = []
        if not flags:
            result["failed"] = True
            result["reason"] = "无法读取 planning.conf: %s" % conf_file
            return result

        def _num(key):
            if key not in cfg:
                return None
            try:
                return float(cfg[key])
            except (TypeError, ValueError):
                return None

        for flag_key, min_key, max_key in gate:
            min_val = _num(min_key)
            max_val = _num(max_key)
            if min_val is None and max_val is None:
                continue
            raw = flags.get(flag_key)
            if raw is None:
                failures.append("%s 缺失" % flag_key)
                result["checks"].append(
                    {"flag": flag_key, "value": None,
                     "min": min_val, "max": max_val, "ok": False})
                continue
            try:
                val = float(raw)
            except ValueError:
                failures.append("%s=%s 非数值" % (flag_key, raw))
                result["checks"].append(
                    {"flag": flag_key, "value": raw,
                     "min": min_val, "max": max_val, "ok": False})
                continue
            result["flags"][flag_key] = val
            ok = True
            if min_val is not None and val < min_val:
                ok = False
                failures.append("%s=%g < 下限 %g" % (flag_key, val, min_val))
            if max_val is not None and val > max_val:
                ok = False
                failures.append("%s=%g > 上限 %g" % (flag_key, val, max_val))
            result["checks"].append(
                {"flag": flag_key, "value": val,
                 "min": min_val, "max": max_val, "ok": ok})
        if failures:
            result["failed"] = True
            result["reason"] = "巡航速度配置不达标：" + "；".join(failures)
        return result

    def _write_presets_unlocked(self, data):
        """原子写 presets.json（先写 .tmp 再 rename），失败返回错误字符串。"""
        tmp = self._preset_file + ".tmp"
        try:
            with io.open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=4)
            os.replace(tmp, self._preset_file)
        except OSError as exc:
            return "写入 presets.json 失败: %s" % exc
        return None

    def save_presets(self, payload):
        """整体覆盖写 presets.json（页面编辑用）。写前做结构校验。"""
        if not isinstance(payload, dict) or not isinstance(payload.get("presets"), list):
            return {"ok": False, "error": "请求体需要 {presets: [...]}"}
        cleaned = []
        seen = set()
        for item in payload["presets"]:
            if not isinstance(item, dict):
                return {"ok": False, "error": "presets 每一项必须是对象"}
            pid = str(item.get("id") or "").strip()
            if not pid:
                return {"ok": False, "error": "每个预设都必须有 id"}
            if not re.match(r"^[A-Za-z0-9_.-]{1,40}$", pid):
                return {"ok": False, "error": "预设 id 只允许字母数字 _ . -（≤40字符）: %s" % pid}
            if pid in seen:
                return {"ok": False, "error": "预设 id 重复: %s" % pid}
            seen.add(pid)
            entry = {"id": pid, "label": str(item.get("label") or pid).strip()}
            for key in self._PRESET_PATH_KEYS:
                val = item.get(key)
                if val:
                    entry[key] = str(val).strip()
            if not entry.get("scene_manifest") and not entry.get("config"):
                return {"ok": False,
                        "error": "预设 %s 需要 scene_manifest 或 config 之一" % pid}
            cleaned.append(entry)
        active = payload.get("active")
        if active not in seen:
            active = cleaned[0]["id"] if cleaned else None
        data = {"_comment": "赛段预设，可由控制台页面编辑，也可手改本文件。",
                "active": active, "presets": cleaned}
        with self._lock:
            err = self._write_presets_unlocked(data)
        if err:
            return {"ok": False, "error": err}
        return self.load_presets()

    def set_active_preset(self, preset_id):
        """只切换 active 字段，保留文件里其它内容（含注释键）。"""
        info = self.load_presets()
        if preset_id not in [p["id"] for p in info.get("presets", [])]:
            return {"ok": False, "error": "预设不存在: %s" % preset_id}
        with self._lock:
            try:
                with io.open(self._preset_file, "r", encoding="utf-8-sig") as f:
                    raw = json.load(f)
            except (OSError, ValueError) as exc:
                return {"ok": False, "error": "读取 presets.json 失败: %s" % exc}
            raw["active"] = preset_id
            err = self._write_presets_unlocked(raw)
        if err:
            return {"ok": False, "error": err}
        return self.load_presets()

    def _apply_preset_to_options(self, options):
        """把 options 里的 preset_id 展开成具体的场景/评分路径。

        显式传入的 scenario_file / scene_manifest / config 优先，预设只补空缺。
        """
        pid = options.get("preset_id")
        # 没显式指定就用 presets.json 里的 active 赛段，这样即使调用方
        # （比如脚本或旧页面）不带 preset_id，也不会漏掉赛段路径。
        preset, err = self.get_preset(pid or None)
        if err:
            # 显式指定的预设找不到必须报错；没指定时只是无预设可用，静默跳过。
            return options, (err if pid else None)
        if not preset:
            return options, None
        merged = dict(options)
        for key in self._PRESET_PATH_KEYS:
            if not merged.get(key) and preset.get(key):
                merged[key] = preset[key]
        merged["_preset"] = {"id": preset["id"], "label": preset.get("label")}
        return merged, None

    # ------------------------------ evaluation ------------------------
    @staticmethod
    def _parse_evaluation(stdout, stderr, returncode):
        """按 grading_system_py 的场景（题目）粒度解析成绩。

        规则完全沿用评测器输出：
        - 每个场景有一行 `Scenario: PASS/FAIL/NO_DATA`（见 main.py _print_summary）。
        - 场景 PASS/FAIL 已由评测器按 is_critical 指标决定，不再自行加权。
        - 总成绩 = 通过题目数 / 总题目数（来自 MULTI-SCENE SUMMARY）。
        """
        text = (stdout or "") + "\n" + (stderr or "")
        result = {"passed": None, "score": None, "no_data": False,
                  "scenes": [], "scene_passed": None, "scene_total": None,
                  "raw_output": text[-12000:], "returncode": returncode}

        scene_head = re.compile(r"^\s*Scene:\s*(.+?)\s*\[", re.IGNORECASE)
        scenario_line = re.compile(r"^\s*Scenario:\s*(PASS|FAIL|NO_DATA)\s*$", re.IGNORECASE)
        scene_score_line = re.compile(r"^\s*Scene Score:\s*(-?\d+(?:\.\d+)?)\s*$", re.IGNORECASE)
        metric_row = re.compile(
            r"^\s*(.+?)\s+(PASS|FAIL)\s+(-?\d+(?:\.\d+)?)\s+(\d+)\s*/\s*(\d+)\s*$",
            re.IGNORECASE)
        detail_line = re.compile(
            r"^\s*Detail:\s*first_fail\s+t=(-?\d+(?:\.\d+)?)s\s+code=([A-Za-z0-9_.:-]+)\s*(.*)$",
            re.IGNORECASE)
        reasons_line = re.compile(r"^\s*Reasons:\s*(.*)$", re.IGNORECASE)
        events_line = re.compile(r"^\s*Events:\s*(.*)$", re.IGNORECASE)
        summary_line = re.compile(
            r"Scenes\s*:\s*(\d+)\s*\|\s*Passed\s*:\s*(\d+)\s*\|\s*Failed/NoData\s*:\s*(\d+)",
            re.IGNORECASE)

        scenes = []
        current = None
        last_metric = None
        for line in text.splitlines():
            h = scene_head.match(line)
            if h:
                current = {"name": h.group(1).strip(), "status": None,
                           "scene_score": None, "metrics": []}
                scenes.append(current)
                last_metric = None
                continue
            s = scenario_line.match(line)
            if s and current is not None:
                current["status"] = s.group(1).upper()
                last_metric = None
                continue
            ss = scene_score_line.match(line)
            if ss and current is not None:
                current["scene_score"] = float(ss.group(1))
                continue
            m = metric_row.match(line)
            if m and current is not None and m.group(1).strip().lower() != "metric":
                total = int(m.group(5))
                last_metric = {
                    "name": m.group(1).strip(),
                    "passed": m.group(2).upper() == "PASS",
                    "score": float(m.group(3)),
                    "fail": int(m.group(4)),
                    "total": total,
                    "evaluated": total > 0,
                    "first_failure": None,
                    "failure_reasons": [],
                    "failure_events": [],
                }
                current["metrics"].append(last_metric)
                continue
            d = detail_line.match(line)
            if d and last_metric is not None:
                last_metric["first_failure"] = {
                    "time": float(d.group(1)),
                    "code": d.group(2),
                    "description": d.group(3).strip(),
                }
                continue
            r = reasons_line.match(line)
            if r and last_metric is not None:
                reasons = []
                for item in r.group(1).split(","):
                    item = item.strip()
                    if not item or "=" not in item:
                        continue
                    code, count = item.rsplit("=", 1)
                    try:
                        reasons.append({"code": code.strip(), "count": int(count)})
                    except ValueError:
                        continue
                last_metric["failure_reasons"] = reasons
                continue
            e = events_line.match(line)
            if e and last_metric is not None:
                events = []
                for item in e.group(1).split(";"):
                    item = item.strip()
                    match = re.match(
                        r"^([A-Za-z0-9_.:-]+)\[(-?\d+(?:\.\d+)?)-"
                        r"(-?\d+(?:\.\d+)?)s\]$", item)
                    if not match:
                        continue
                    events.append({
                        "code": match.group(1),
                        "start": float(match.group(2)),
                        "end": float(match.group(3)),
                    })
                last_metric["failure_events"] = events

        result["scenes"] = scenes

        sm = summary_line.search(text)
        if sm:
            result["scene_total"] = int(sm.group(1))
            result["scene_passed"] = int(sm.group(2))
        elif scenes:
            result["scene_total"] = len(scenes)
            result["scene_passed"] = sum(1 for sc in scenes if sc["status"] == "PASS")

        # 单场景（--config 模式，无多场景汇总）时回退成整体判定。
        if not scenes:
            single = scenario_line.search(text) or re.search(
                r"Scenario\s*:\s*(PASS|FAIL|NO_DATA)", text, re.IGNORECASE)
            if single:
                st = single.group(1).upper()
                result["no_data"] = st == "NO_DATA"
                result["passed"] = st == "PASS"
                result["scene_total"] = 1
                result["scene_passed"] = 1 if st == "PASS" else 0
            return result

        # 所有场景都是 NO_DATA => 数据包没被真正评测。
        result["no_data"] = all(sc["status"] == "NO_DATA" for sc in scenes)
        if not result["no_data"]:
            # 扣分制总分：有数据场景 Scene Score 取平均；缺失时回退通过率。
            scored = [sc["scene_score"] for sc in scenes
                      if sc.get("status") != "NO_DATA" and sc.get("scene_score") is not None]
            if scored:
                result["score"] = round(sum(scored) / len(scored), 1)
            elif result["scene_total"]:
                result["score"] = round(100.0 * result["scene_passed"] / result["scene_total"], 1)
            result["passed"] = result["scene_passed"] == result["scene_total"]
        if result["no_data"]:
            result["score"] = None
            result["passed"] = False
        return result

    def evaluate_start(self, options):
        options = options if isinstance(options, dict) else {}
        options, preset_err = self._apply_preset_to_options(options)
        if preset_err:
            return {"ok": False, "error": preset_err}

        # 组装本次要评测的数据包：默认读取 records/ 下全部录包。
        bags = []
        explicit = options.get("bag_file", "")
        if explicit:
            real = os.path.realpath(explicit)
            if not os.path.exists(real):
                return {"ok": False, "error": "数据包不存在: " + str(explicit)}
            bags = [{"bag_file": real, "name": os.path.basename(os.path.dirname(real)) or os.path.basename(real)}]
        else:
            collected = self._collect_bags()
            if not collected:
                return {"ok": False, "error": "落盘目录暂无数据包"}
            bags = [{"bag_file": b["bag_file"], "name": b["name"]} for b in collected]

        # 解析场景清单 / 单配置（对所有数据包统一使用）。显式参数优先，
        # 避免前端传入 config 后又被默认 scenes.json 静默覆盖。
        manifest = options.get("scene_manifest", "")
        config = options.get("config", "")
        if manifest:
            manifest = os.path.realpath(manifest)
            if not os.path.isfile(manifest):
                return {"ok": False, "error": "场景清单不存在: " + str(options.get("scene_manifest"))}
        elif config:
            config = os.path.realpath(config)
            if not os.path.isfile(config):
                return {"ok": False, "error": "评分配置不存在: " + str(options.get("config"))}
            if os.path.basename(config) == "scenes.json":
                manifest, config = config, ""
        else:
            default_manifest = os.path.join(self.base_dir, "metric", "scenes.json")
            if os.path.isfile(default_manifest):
                manifest = default_manifest

        def _argv(bag_file):
            a = [sys.executable, "-m", "grading_system_py.main", "--mode", "bag", "--bag_file", bag_file]
            if manifest:
                a += ["--scene_manifest", manifest]
            elif config:
                a += ["--config", config]
            return a

        job_id = uuid.uuid4().hex[:12]
        job = {"id": job_id, "status": "queued", "created_at": time.time(),
               "manifest": manifest or None, "bag_count": len(bags),
               "preset": options.get("_preset"),
               "score": None, "passed": None}
        with self._lock:
            self._jobs[job_id] = job
        tasks = [(b["name"], b["bag_file"], _argv(b["bag_file"])) for b in bags]
        threading.Thread(target=self._run_evaluation, args=(job_id, tasks), daemon=True).start()
        return {"ok": True, "job": dict(job)}

    def _eval_one(self, pkg_name, bag_file, argv, multi):
        """评测单个数据包，任何异常都被隔离成一条 FAIL 结果，不影响其它包。"""
        timeout = float(os.environ.get("APOLLO_EVAL_TIMEOUT_SEC", "900"))
        prefix = (pkg_name + " / ") if multi else ""
        try:
            env = dict(os.environ)
            env.setdefault("PYTHONIOENCODING", "utf-8")
            env.setdefault("LC_ALL", "C.UTF-8")
            env.setdefault("LANG", "C.UTF-8")
            # 不使用 universal_newlines 的默认 locale 解码。宿主环境可能是
            # LANG=C/ASCII，而 manifest/map 路径和评测器输出包含 UTF-8 中文。
            proc = subprocess.Popen(argv, cwd=self.apollo_root, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, universal_newlines=False,
                                    env=env)
            try:
                stdout_bytes, stderr_bytes = proc.communicate(timeout=timeout)
                rc = proc.returncode
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout_bytes, stderr_bytes = proc.communicate()
                stdout = (stdout_bytes or b"").decode("utf-8", errors="replace")
                stderr = (stderr_bytes or b"").decode("utf-8", errors="replace")
                return {"rc": -1, "stdout": stdout, "stderr": stderr, "scenes": [
                    {"name": prefix + pkg_name, "package": pkg_name, "status": "FAIL",
                     "error": "评测超时(%ds)" % timeout, "metrics": []}]}
        except OSError as exc:
            return {"rc": -1, "stdout": "", "stderr": str(exc), "scenes": [
                {"name": prefix + pkg_name, "package": pkg_name, "status": "FAIL",
                 "error": "无法启动评测器: %s" % exc, "metrics": []}]}
        stdout = (stdout_bytes or b"").decode("utf-8", errors="replace")
        stderr = (stderr_bytes or b"").decode("utf-8", errors="replace")
        parsed = self._parse_evaluation(stdout, stderr, rc)
        scenes = []
        for sc in parsed.get("scenes", []):
            sc = dict(sc)
            sc["package"] = pkg_name
            sc["name"] = prefix + sc["name"]
            scenes.append(sc)
        if not scenes:
            # 无场景：可能是 --config 单场景，或评测器直接报错。
            if parsed.get("passed") is not None:
                status = ("PASS" if parsed["passed"] else
                          ("NO_DATA" if parsed.get("no_data") else "FAIL"))
                # 单场景也要有实际得分：优先用评测器整体分，否则 PASS→100/FAIL→0，
                # NO_DATA 无分数。
                if parsed.get("score") is not None:
                    single_score = parsed["score"]
                elif status == "PASS":
                    single_score = 100.0
                elif status == "FAIL":
                    single_score = 0.0
                else:
                    single_score = None
                scenes.append({"name": prefix + pkg_name, "package": pkg_name,
                               "status": status, "scene_score": single_score,
                               "metrics": []})
            else:
                scenes.append({"name": prefix + pkg_name, "package": pkg_name,
                               "status": "FAIL", "scene_score": 0.0, "metrics": [],
                               "error": "评测器无输出(返回码 %s)" % rc})
        return {"rc": rc, "stdout": stdout, "stderr": stderr, "scenes": scenes}

    def _run_evaluation(self, job_id, tasks):
        with self._lock:
            job = self._jobs.get(job_id)
            if not job:
                return
            job["status"] = "running"
            job["started_at"] = time.time()
            job["progress"] = {"done": 0, "total": len(tasks)}
        multi = len(tasks) > 1
        try:
            workers = max(1, min(int(os.environ.get("APOLLO_EVAL_WORKERS", "4")), len(tasks)))
            results = [None] * len(tasks)
            last_rc = 0
            stdout_tail = stderr_tail = ""
            with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
                fut_map = {pool.submit(self._eval_one, t[0], t[1], t[2], multi): i
                           for i, t in enumerate(tasks)}
                for fut in concurrent.futures.as_completed(fut_map):
                    idx = fut_map[fut]
                    try:
                        results[idx] = fut.result()
                    except Exception as exc:  # noqa: BLE001 - 单包异常兜底
                        results[idx] = {"rc": -1, "stdout": "", "stderr": str(exc),
                                        "scenes": [{"name": tasks[idx][0], "status": "FAIL",
                                                    "metrics": [], "error": str(exc)}]}
                    with self._lock:
                        job["progress"]["done"] += 1
            all_scenes = []
            for r in results:
                if not r:
                    continue
                all_scenes.extend(r["scenes"])
                if r["rc"] != 0:
                    last_rc = r["rc"]
                if r.get("stdout"):
                    stdout_tail = r["stdout"][-12000:]
                if r.get("stderr"):
                    stderr_tail = r["stderr"][-12000:]
            merged = self._merge_scenes(all_scenes, last_rc, stdout_tail, stderr_tail)
            # 巡航速度门槛：读 planning.conf，低于规定值则按 scope 判 0。
            gate = self.check_cruise_speed_gate()
            if gate.get("enabled") and gate.get("failed"):
                self._apply_cruise_gate(merged, gate)
            with self._lock:
                job.update(merged)
                job["cruise_speed_gate"] = gate
                job["stdout"] = stdout_tail
                job["stderr"] = stderr_tail
                # 车辆违规属于正常完成的 FAIL；评测器自身返回非零才是任务失败。
                job["status"] = "succeeded" if last_rc == 0 else "failed"
                if last_rc != 0 and not job.get("error"):
                    real_err = self._filter_benign_stderr(stderr_tail)
                    if not real_err:
                        # stderr 只有无害噪声时，真实原因通常打在 stdout
                        # （评测器的 [ERROR]/argparse usage 等）。
                        real_err = self._extract_error_from_stdout(stdout_tail)
                    job["error"] = real_err or "评测器返回非零状态码: %s" % last_rc
                job["finished_at"] = time.time()
            # 成功完成的评测记入历史，供多次测试统计与取最高分。
            if job.get("status") == "succeeded":
                self._append_history(job)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                job.update({"status": "failed", "error": str(exc), "finished_at": time.time()})

    @staticmethod
    def _apply_cruise_gate(merged, gate):
        """按巡航速度门槛结果改写成绩。

        scope=all  : 整次评测判 0，所有场景 FAIL。
        scope=cruise: 只把定速巡航题（id/名字含 cruise/巡航）判 0 FAIL，
                      并按新的场景分重算总分。
        无论哪种，都把原因挂到对应场景上，便于前端展示。
        """
        reason = gate.get("reason", "巡航速度配置不达标")
        scenes = merged.get("scenes", [])
        if gate.get("scope") == "all":
            for sc in scenes:
                sc["status"] = "FAIL"
                sc["scene_score"] = 0.0
                sc["gate_reason"] = reason
            merged["score"] = 0.0
            merged["passed"] = False
            merged["scene_passed"] = 0
            merged["no_data"] = False
            merged["gate_failed"] = True
            merged["gate_reason"] = reason
            return

        # scope=cruise：只判定速巡航题
        hit = False
        for sc in scenes:
            name = str(sc.get("name", "")).lower()
            if "cruise" in name or "巡航" in name:
                sc["status"] = "FAIL"
                sc["scene_score"] = 0.0
                sc["gate_reason"] = reason
                hit = True
        merged["gate_failed"] = True
        merged["gate_reason"] = reason
        if not hit:
            return
        # 重算总分/通过数（沿用 _merge_scenes 的扣分制口径）
        graded = [sc for sc in scenes if sc.get("status") != "NO_DATA"]
        passed = sum(1 for sc in scenes if sc.get("status") == "PASS")
        merged["scene_passed"] = passed
        merged["passed"] = bool(scenes) and passed == len(scenes)
        scored = [sc["scene_score"] for sc in graded
                  if sc.get("scene_score") is not None]
        if scored:
            merged["score"] = round(sum(scored) / len(scored), 1)
        elif scenes:
            merged["score"] = round(100.0 * passed / len(scenes), 1)

    @staticmethod
    def _merge_scenes(scenes, returncode, stdout_tail, stderr_tail):
        """把多个数据包的题目合并成一次总成绩。

        优先用扣分制：各有数据场景的 Scene Score 取平均（0-100）。
        解析不到 scene_score 时回退到旧口径：通过题目数 / 总题目数。
        """
        result = {"scenes": scenes, "raw_output": (stdout_tail or "")[-12000:],
                  "returncode": returncode, "no_data": False,
                  "score": None, "passed": None,
                  "scene_total": len(scenes), "scene_passed": None}
        if not scenes:
            return result
        result["no_data"] = all(sc.get("status") == "NO_DATA" for sc in scenes)
        passed = sum(1 for sc in scenes if sc.get("status") == "PASS")
        result["scene_passed"] = passed
        if result["no_data"]:
            result["score"] = None
            result["passed"] = False
            return result
        result["passed"] = passed == len(scenes)
        scored = [sc["scene_score"] for sc in scenes
                  if sc.get("status") != "NO_DATA" and sc.get("scene_score") is not None]
        if scored:
            result["score"] = round(sum(scored) / len(scored), 1)
        else:
            result["score"] = round(100.0 * passed / len(scenes), 1)
        return result

    @staticmethod
    def _filter_benign_stderr(stderr):
        """剔除 cyber/glog 启动噪声，只保留真正的错误行。

        cyber_recorder 初始化时会往 stderr 打印无害信息，例如：
          WARNING: Logging before InitGoogleLogging() is written to STDERR
          I0727 16:52:09.373960 19800 py_record.cc:574] []init _cyber_record_wrapper
        这些不是评测失败原因，不应被当作 job["error"] 展示。
        """
        if not stderr:
            return ""
        lines = []
        for ln in stderr.splitlines():
            s = ln.strip()
            if not s:
                continue
            if s.startswith("WARNING: Logging before InitGoogleLogging()"):
                continue
            # glog 的 info/warning 行：I0727.../W0727... 且属于 cyber 初始化噪声
            if (len(s) > 1 and s[0] in "IWEF" and s[1:5].isdigit()
                    and ("_cyber_record_wrapper" in s or "py_record.cc" in s)):
                continue
            lines.append(ln)
        return "\n".join(lines).strip()

    @staticmethod
    def _extract_error_from_stdout(stdout):
        """从评测器 stdout 里抽取真正的失败原因。

        评测器把致命错误打在 stdout（如 [ERROR] Multi-scene evaluation failed: ...
        或 argparse 的 usage/error 行）。返回最后出现的错误行。
        """
        if not stdout:
            return ""
        found = []
        for ln in stdout.splitlines():
            s = ln.strip()
            if not s:
                continue
            if ("[ERROR]" in s or s.lower().startswith("usage:")
                    or "error:" in s.lower() or "failed" in s.lower()):
                found.append(s)
        return found[-1] if found else ""

    def _append_history(self, job):
        """把一次成功评测的成绩追加到历史文件（供多次测试统计与取最高分）。"""
        entry = {
            "id": job.get("id"),
            "score": job.get("score"),
            "passed": job.get("passed"),
            "preset": job.get("preset"),
            "scene_total": job.get("scene_total"),
            "scene_passed": job.get("scene_passed"),
            "finished_at": job.get("finished_at"),
            "scenes": [{"name": sc.get("name"), "status": sc.get("status"),
                        "scene_score": sc.get("scene_score")}
                       for sc in job.get("scenes", [])],
        }
        with self._lock:
            history = self._load_history_unlocked()
            history.append(entry)
            try:
                os.makedirs(self.record_root, exist_ok=True)
                with io.open(self._history_file, "w", encoding="utf-8") as f:
                    json.dump(history, f, ensure_ascii=False, indent=2)
            except OSError:
                pass

    def _load_history_unlocked(self):
        try:
            with io.open(self._history_file, "r", encoding="utf-8") as f:
                data = json.load(f)
                return data if isinstance(data, list) else []
        except (OSError, ValueError):
            return []

    def score_history(self):
        """返回历次成绩列表 + 最高分记录，供前端展示排行。"""
        with self._lock:
            history = self._load_history_unlocked()
        scored = [h for h in history if isinstance(h.get("score"), (int, float))]
        best = max(scored, key=lambda h: h["score"]) if scored else None
        return {"ok": True, "count": len(history),
                "attempts": len(scored),
                "best": best,
                "best_score": best["score"] if best else None,
                "history": history}

    def clear_history(self):
        with self._lock:
            try:
                if os.path.isfile(self._history_file):
                    os.remove(self._history_file)
            except OSError as exc:
                return {"ok": False, "error": str(exc)}
        return {"ok": True}

    def evaluate_status(self, job_id=None):
        with self._lock:
            if job_id:
                job = self._jobs.get(job_id)
                if not job:
                    return {"ok": False, "error": "评测任务不存在"}
                return {"ok": True, "job": dict(job)}
            return {"ok": True, "jobs": [dict(j) for j in list(self._jobs.values())[-20:]]}

    def shutdown(self):
        with self._lock:
            if self._record:
                self.record_stop()
            self.scenario_injection_stop()
            for proc in list(self._owned.values()):
                if proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGTERM)
                    except OSError:
                        pass
            self._owned.clear()


_runtime = None


def get_runtime(base_dir):
    global _runtime
    if _runtime is None:
        _runtime = Runtime(base_dir)
    return _runtime
