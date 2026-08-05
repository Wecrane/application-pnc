#!/usr/bin/env python3
"""Launch Apollo sim_obstacle from a scenario JSON file."""

import argparse
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Dict, Iterable, Optional, Union


VERSION = "2026-07-03-utf8-fusion"
DEFAULT_SIM_OBSTACLE_BIN = "/apollo_workspace/.aem/envroot/opt/apollo/neo/bin/sim_obstacle"
DEFAULT_GFLAG_FILE = "/apollo/modules/common/data/global_flagfile.txt"
DEFAULT_PLANNING_COMMAND_TOPIC = "/apollo/planning/command"
DEFAULT_SCENARIO_TOPIC = "/apollo/perception/scenario_obstacles"
DEFAULT_TRAFFIC_LIGHT_TOPIC = "/apollo/perception/traffic_light"


class ScenarioInjector:
    """Small process wrapper for classic sim_obstacle scenario injection."""

    def __init__(
        self,
        sim_obstacle_bin: str = DEFAULT_SIM_OBSTACLE_BIN,
        perception_topic: str = DEFAULT_SCENARIO_TOPIC,
        traffic_light_topic: str = DEFAULT_TRAFFIC_LIGHT_TOPIC,
        map_dir_prefix: str = "/apollo/",
        map_dir: Optional[str] = None,
        gflag_file: str = DEFAULT_GFLAG_FILE,
        publish_command: bool = False,
        trigger_planning_command: bool = True,
        planning_command_topic: str = DEFAULT_PLANNING_COMMAND_TOPIC,
        trigger_delay_sec: float = 1.0,
        extra_flags: Optional[Dict[str, str]] = None,
        cwd: str = "/apollo_workspace",
    ):
        self.sim_obstacle_bin = self._normalize_path_bytes(sim_obstacle_bin)
        self.perception_topic = perception_topic
        self.traffic_light_topic = traffic_light_topic
        self.map_dir_prefix = map_dir_prefix
        self.map_dir = map_dir
        self.gflag_file = self._normalize_path_bytes(gflag_file)
        self.publish_command = publish_command
        self.trigger_planning_command = trigger_planning_command
        self.planning_command_topic = planning_command_topic
        self.trigger_delay_sec = trigger_delay_sec
        self.extra_flags = extra_flags or {}
        self.cwd = self._normalize_path_bytes(cwd)
        self.process: Optional[subprocess.Popen] = None
        self._runtime_gflag_file: Optional[bytes] = None

    def build_command(self, scenario_json: str) -> Iterable[bytes]:
        scenario_path = self._normalize_path_bytes(scenario_json)
        if not os.path.isfile(scenario_path):
            raise FileNotFoundError(
                "scenario json not found: {}".format(self._display_path(scenario_path))
            )
        if not os.path.isfile(self.sim_obstacle_bin):
            raise FileNotFoundError(
                "sim_obstacle binary not found: {}".format(
                    self._display_path(self.sim_obstacle_bin)
                )
            )

        scenario_data = self._read_scenario_data(scenario_path)
        scenario = scenario_data.get("scenario", scenario_data)
        scenario_map_dir = self._read_scenario_map_dir_from_data(scenario, scenario_path)
        map_dir = self.map_dir or self._join_map_dir(self.map_dir_prefix, scenario_map_dir)
        self._warn_if_scenario_coordinates_look_wrong(scenario)
        flags = {
            "map_dir": map_dir,
            "perception_obstacle_topic": self.perception_topic,
            "traffic_light_detection_topic": self.traffic_light_topic,
        }
        if self.map_dir_prefix:
            flags["scenario_map_dir_prefix"] = self.map_dir_prefix
        flags.update(self.extra_flags)
        runtime_gflag_file = self._write_runtime_gflag_file(flags)

        command = [self.sim_obstacle_bin]
        command.append(b"--gflag_file=" + runtime_gflag_file)
        command.extend([scenario_path, str(self.publish_command).lower().encode("utf-8")])
        return command

    def _write_runtime_gflag_file(self, flags: Dict[str, str]) -> bytes:
        if not os.path.isfile(self.gflag_file):
            raise FileNotFoundError(
                "gflag file not found: {}".format(self._display_path(self.gflag_file))
            )
        fd, path = tempfile.mkstemp(prefix="scenario_injector_", suffix=".flagfile")
        path_bytes = self._normalize_path_bytes(path)
        with os.fdopen(fd, "wb") as runtime_file:
            with io.open(self.gflag_file, "rb") as source_file:
                runtime_file.write(source_file.read())
            runtime_file.write(b"\n# scenario_injector overrides\n")
            for key, value in flags.items():
                runtime_file.write(("--{}={}\n".format(key, value)).encode("utf-8"))
        self._runtime_gflag_file = path_bytes
        return path_bytes

    @staticmethod
    def _normalize_path_bytes(path: Union[str, bytes]) -> bytes:
        if isinstance(path, bytes):
            path_bytes = os.path.expanduser(path)
        else:
            expanded_path = os.path.expanduser(path)
            try:
                path_bytes = expanded_path.encode("utf-8")
            except UnicodeEncodeError:
                path_bytes = os.fsencode(expanded_path)
        if not os.path.isabs(path_bytes):
            path_bytes = os.path.abspath(path_bytes)
        return path_bytes

    @staticmethod
    def _display_path(path: Union[str, bytes]) -> str:
        if isinstance(path, bytes):
            return path.decode("utf-8", "replace")
        return path

    @staticmethod
    def _read_scenario_data(scenario_path: bytes) -> dict:
        display_path = ScenarioInjector._display_path(scenario_path)
        if os.path.getsize(scenario_path) == 0:
            raise ValueError("scenario json is empty: {}".format(display_path))
        try:
            with io.open(scenario_path, "r", encoding="utf-8-sig") as scenario_file:
                return json.load(scenario_file)
        except ValueError as error:
            raise ValueError(
                "invalid scenario json: {}; {}".format(display_path, error)
            )

    @staticmethod
    def _read_scenario_map_dir_from_data(scenario: dict, scenario_path: bytes) -> str:
        map_dir = scenario.get("mapDir") or scenario.get("map_dir")
        if not map_dir:
            raise ValueError(
                "scenario json has no scenario.mapDir: {}".format(
                    ScenarioInjector._display_path(scenario_path)
                )
            )
        return map_dir

    @staticmethod
    def _warn_if_scenario_coordinates_look_wrong(scenario: dict) -> None:
        start = scenario.get("start") or {}
        start_x = start.get("x")
        start_y = start.get("y")
        if start_x is None or start_y is None:
            return

        points = []
        for light in scenario.get("trafficLights") or scenario.get("traffic_lights") or []:
            location = light.get("location") or {}
            if "x" in location and "y" in location:
                points.append(("trafficLight", light.get("id", ""), location["x"], location["y"]))
        for agent in scenario.get("agent") or []:
            position = agent.get("startPosition") or {}
            if "x" in position and "y" in position:
                points.append(("agent", agent.get("id", ""), position["x"], position["y"]))

        for kind, item_id, x, y in points:
            distance = ((float(x) - float(start_x)) ** 2 + (float(y) - float(start_y)) ** 2) ** 0.5
            if distance > 100000:
                sys.stderr.write(
                    "scenario_injector warning: {} {} is {:.0f}m from scenario start; "
                    "check mapDir and coordinates\n".format(kind, item_id, distance)
                )

    @staticmethod
    def _join_map_dir(prefix: str, map_dir: str) -> str:
        if map_dir.startswith("/"):
            return map_dir
        if not prefix:
            return map_dir
        return prefix.rstrip("/") + "/" + map_dir.lstrip("/")

    def start(self, scenario_json: str) -> subprocess.Popen:
        if self.process and self.process.poll() is None:
            raise RuntimeError("scenario injector is already running")

        env = os.environ.copy()
        env.setdefault("APOLLO_ROOT_DIR", "/apollo_workspace")
        self.process = subprocess.Popen(
            list(self.build_command(scenario_json)),
            cwd=self.cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        if self.trigger_planning_command:
            thread = threading.Thread(target=self._trigger_after_delay)
            thread.daemon = True
            thread.start()
        return self.process

    def _trigger_after_delay(self) -> None:
        time.sleep(self.trigger_delay_sec)
        try:
            self.publish_planning_command_trigger()
        except Exception as error:
            sys.stderr.write(
                "scenario_injector warning: failed to publish planning trigger: {}\n".format(
                    error
                )
            )

    def publish_planning_command_trigger(self) -> None:
        self._ensure_apollo_python_path()
        from cyber.python.cyber_py3 import cyber
        from cyber.python.cyber_py3 import cyber_time
        import modules.common_msgs.planning_msgs.planning_command_pb2 as planning_command_pb2

        node_name = "scenario_injector_trigger_{}".format(os.getpid())
        cyber.init(node_name)
        node = cyber.Node(node_name)
        topics = [self.planning_command_topic]
        if "/apollo/planning_command" not in topics:
            topics.append("/apollo/planning_command")
        writers = [
            node.create_writer(topic, planning_command_pb2.PlanningCommand)
            for topic in topics
        ]
        sequence_num = 1
        for _ in range(10):
            for module_name in ("sim_obstacle", "dreamview"):
                planning_command = planning_command_pb2.PlanningCommand()
                planning_command.header.sequence_num = sequence_num
                planning_command.header.timestamp_sec = cyber_time.Time.now().to_sec()
                planning_command.header.module_name = module_name
                for writer in writers:
                    writer.write(planning_command)
                sequence_num += 1
            time.sleep(0.2)
        sys.stderr.write(
            "scenario_injector: published planning trigger on {}\n".format(
                ", ".join(topics)
            )
        )

    @staticmethod
    def _ensure_apollo_python_path() -> None:
        for path in (
            "/apollo_workspace",
            "/apollo_workspace/.aem/envroot/opt/apollo/neo/python",
            "/apollo_workspace/.aem/envroot/opt/apollo/neo/lib",
        ):
            if path not in sys.path and os.path.exists(path):
                sys.path.insert(0, path)

    def stop(self, timeout: float = 5.0) -> None:
        if self.process and self.process.poll() is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait()
        self._cleanup_runtime_gflag_file()

    def _cleanup_runtime_gflag_file(self) -> None:
        if not self._runtime_gflag_file:
            return
        try:
            os.unlink(self._runtime_gflag_file)
        except OSError:
            pass
        self._runtime_gflag_file = None

    def run_forever(self, scenario_json: str) -> int:
        process = self.start(scenario_json)
        try:
            assert process.stdout is not None
            for line in process.stdout:
                sys.stdout.buffer.write(line)
                sys.stdout.buffer.flush()
            return process.wait()
        except KeyboardInterrupt:
            self.stop()
            return 130
        finally:
            self._cleanup_runtime_gflag_file()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()


def parse_extra_flags(flag_values):
    extra_flags = {}
    for item in flag_values:
        if "=" not in item:
            raise ValueError(f"extra flag must be key=value: {item}")
        key, value = item.split("=", 1)
        key = key.lstrip("-")
        extra_flags[key] = value
    return extra_flags


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start sim_obstacle with a scenario JSON file."
    )
    parser.add_argument(
        "scenario_json",
        nargs="?",
        help="path to scenario json; omit with --trigger-only",
    )
    parser.add_argument(
        "--version",
        action="version",
        version="scenario_injector {}".format(VERSION),
    )
    parser.add_argument(
        "--bin",
        default=DEFAULT_SIM_OBSTACLE_BIN,
        help="path to sim_obstacle binary",
    )
    parser.add_argument(
        "--perception-topic",
        default=DEFAULT_SCENARIO_TOPIC,
        help="topic for injected PerceptionObstacles",
    )
    parser.add_argument(
        "--traffic-light-topic",
        default=DEFAULT_TRAFFIC_LIGHT_TOPIC,
        help="topic for injected traffic lights",
    )
    parser.add_argument(
        "--map-dir-prefix",
        default="/apollo/",
        help="prefix prepended by sim_obstacle when checking scenario mapDir",
    )
    parser.add_argument(
        "--map-dir",
        default=None,
        help="explicit Apollo map_dir gflag; defaults to map-dir-prefix + scenario.mapDir",
    )
    parser.add_argument(
        "--gflag-file",
        default=DEFAULT_GFLAG_FILE,
        help="base Apollo global flagfile copied before appending scenario overrides",
    )
    parser.add_argument(
        "--publish-command",
        action="store_true",
        help="let sim_obstacle publish LaneFollowCommand from the scenario end point",
    )
    parser.add_argument(
        "--no-trigger-planning-command",
        action="store_true",
        help="do not publish the PlanningCommand that enables sim_obstacle",
    )
    parser.add_argument(
        "--planning-command-topic",
        default=DEFAULT_PLANNING_COMMAND_TOPIC,
        help="PlanningCommand topic used to enable sim_obstacle",
    )
    parser.add_argument(
        "--trigger-delay-sec",
        default=1.0,
        type=float,
        help="seconds to wait before publishing the PlanningCommand trigger",
    )
    parser.add_argument(
        "--trigger-only",
        action="store_true",
        help="only publish the PlanningCommand trigger and exit",
    )
    parser.add_argument(
        "--flag",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="extra gflag passed to sim_obstacle; can be repeated",
    )
    args = parser.parse_args()
    if not args.scenario_json and not args.trigger_only:
        parser.error("scenario_json is required unless --trigger-only is set")

    injector = ScenarioInjector(
        sim_obstacle_bin=args.bin,
        perception_topic=args.perception_topic,
        traffic_light_topic=args.traffic_light_topic,
        map_dir_prefix=args.map_dir_prefix,
        map_dir=args.map_dir,
        gflag_file=args.gflag_file,
        publish_command=args.publish_command,
        trigger_planning_command=not args.no_trigger_planning_command,
        planning_command_topic=args.planning_command_topic,
        trigger_delay_sec=args.trigger_delay_sec,
        extra_flags=parse_extra_flags(args.flag),
    )
    try:
        if args.trigger_only:
            injector.publish_planning_command_trigger()
            return 0
        return injector.run_forever(args.scenario_json)
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        sys.stderr.write("scenario_injector error: {}\n".format(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
