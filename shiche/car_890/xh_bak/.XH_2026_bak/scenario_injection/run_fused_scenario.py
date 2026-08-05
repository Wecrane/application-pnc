#!/usr/bin/env python3
"""Run scenario injection with temporary perception obstacle fusion."""

import argparse
import os
import signal
import subprocess
import sys
import time

from configure_perception_side_channel import (
    DEFAULT_ORIGINAL_TOPIC,
    DEFAULT_SIDE_TOPIC,
    configure,
    restore,
)

DEFAULT_CONFIG = (
    "/apollo/modules/perception/multi_sensor_fusion/conf/"
    "multi_sensor_fusion_config.pb.txt"
)
DEFAULT_SCENARIO_TOPIC = "/apollo/perception/scenario_obstacles"
DEFAULT_FUSION_SCRIPT = "/apollo_workspace/scenario_injection/perception_fusion.py"
DEFAULT_INJECTOR_SCRIPT = "/apollo_workspace/scenario_injection/scenario_injector.py"


class FusedScenarioRunner:
    def __init__(self, args):
        self.args = args
        self.args.scenario_json = os.path.abspath(args.scenario_json)
        self.args.fusion_script = os.path.abspath(args.fusion_script)
        self.args.injector_script = os.path.abspath(args.injector_script)
        self.processes = []
        self.configured = False
        self.stopping = False

    def run(self):
        self._install_signal_handlers()
        try:
            self._configure_perception_side_channel()
            if self.args.perception_command:
                self._start_process(
                    self.args.perception_command,
                    "perception",
                    shell=True,
                )
                time.sleep(self.args.perception_warmup_sec)
            else:
                sys.stderr.write(
                    "run_fused_scenario: perception output was moved to {}; "
                    "restart perception now if it is already running\n".format(
                        self.args.lidar_topic
                    )
                )
                if self.args.wait_for_perception_restart:
                    input("Press Enter after perception has restarted...")

            self._start_process(self._fusion_command(), "perception_fusion")
            time.sleep(self.args.fusion_warmup_sec)
            injector = self._start_process(self._injector_command(), "scenario_injector")
            return self._wait_for_process(injector)
        finally:
            self.stop()

    def _install_signal_handlers(self):
        def handler(signum, _frame):
            self.stopping = True
            self.stop()
            raise SystemExit(128 + signum)

        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    def _configure_perception_side_channel(self):
        results = configure(
            [self.args.config],
            self.args.original_topic,
            self.args.lidar_topic,
            dry_run=self.args.dry_run,
        )
        self.configured = any(
            result.status in ("changed", "already") for result in results
        )
        if self.args.dry_run:
            raise SystemExit(0)

    def _fusion_command(self):
        command = [
            sys.executable,
            self.args.fusion_script,
            "--lidar-topic",
            self.args.lidar_topic,
            "--scenario-topic",
            self.args.scenario_topic,
            "--output-topic",
            self.args.original_topic,
            "--scenario-timeout-sec",
            str(self.args.scenario_timeout_sec),
            "--module-name",
            self.args.fusion_module_name,
        ]
        if self.args.no_scenario_id_offset:
            command.append("--no-scenario-id-offset")
        else:
            command.extend(["--scenario-id-offset", str(self.args.scenario_id_offset)])
        return command

    def _injector_command(self):
        command = [
            sys.executable,
            self.args.injector_script,
            self.args.scenario_json,
            "--perception-topic",
            self.args.scenario_topic,
        ]
        if self.args.bin:
            command.extend(["--bin", self.args.bin])
        if self.args.map_dir:
            command.extend(["--map-dir", self.args.map_dir])
        if self.args.map_dir_prefix is not None:
            command.extend(["--map-dir-prefix", self.args.map_dir_prefix])
        if self.args.gflag_file:
            command.extend(["--gflag-file", self.args.gflag_file])
        if self.args.publish_command:
            command.append("--publish-command")
        if self.args.no_trigger_planning_command:
            command.append("--no-trigger-planning-command")
        command.extend(["--planning-command-topic", self.args.planning_command_topic])
        command.extend(["--trigger-delay-sec", str(self.args.trigger_delay_sec)])
        for flag in self.args.flag:
            command.extend(["--flag", flag])
        return command

    def _start_process(self, command, name, shell=False):
        sys.stderr.write("run_fused_scenario: starting {}\n".format(name))
        process = subprocess.Popen(
            command,
            cwd=self.args.cwd,
            preexec_fn=os.setsid,
            shell=shell,
        )
        self.processes.append((name, process))
        return process

    def _wait_for_process(self, process):
        while True:
            return_code = process.poll()
            if return_code is not None:
                return return_code
            for name, child in list(self.processes):
                if child is process:
                    continue
                child_return_code = child.poll()
                if child_return_code is not None and not self.stopping:
                    raise RuntimeError(
                        "{} exited early with code {}".format(name, child_return_code)
                    )
            time.sleep(0.5)

    def stop(self):
        if self.stopping:
            pass
        self.stopping = True
        for name, process in reversed(self.processes):
            if process.poll() is not None:
                continue
            sys.stderr.write("run_fused_scenario: stopping {}\n".format(name))
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=self.args.stop_timeout_sec)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait()
            except OSError:
                pass
        if self.configured or self.args.restore_on_exit:
            try:
                restore([self.args.config])
            except Exception as error:
                sys.stderr.write(
                    "run_fused_scenario warning: failed to restore config: {}\n".format(
                        error
                    )
                )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Temporarily fuse real perception with scenario obstacles."
    )
    parser.add_argument("scenario_json", help="path to scenario json")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--original-topic", default=DEFAULT_ORIGINAL_TOPIC)
    parser.add_argument("--lidar-topic", default=DEFAULT_SIDE_TOPIC)
    parser.add_argument("--scenario-topic", default=DEFAULT_SCENARIO_TOPIC)
    parser.add_argument("--fusion-script", default=DEFAULT_FUSION_SCRIPT)
    parser.add_argument("--injector-script", default=DEFAULT_INJECTOR_SCRIPT)
    parser.add_argument("--scenario-timeout-sec", type=float, default=0.5)
    parser.add_argument("--scenario-id-offset", type=int, default=100000)
    parser.add_argument("--no-scenario-id-offset", action="store_true")
    parser.add_argument("--fusion-module-name", default="perception_fusion")
    parser.add_argument(
        "--perception-command",
        default=None,
        help="optional command to start perception after switching its output topic",
    )
    parser.add_argument("--perception-warmup-sec", type=float, default=2.0)
    parser.add_argument(
        "--wait-for-perception-restart",
        action="store_true",
        help="pause until you manually restart perception after the topic switch",
    )
    parser.add_argument("--fusion-warmup-sec", type=float, default=1.0)
    parser.add_argument("--stop-timeout-sec", type=float, default=5.0)
    parser.add_argument("--restore-on-exit", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--cwd", default="/apollo_workspace")

    parser.add_argument("--bin", default=None, help="path to sim_obstacle binary")
    parser.add_argument("--map-dir", default=None)
    parser.add_argument("--map-dir-prefix", default="/apollo/")
    parser.add_argument("--gflag-file", default=None)
    parser.add_argument("--publish-command", action="store_true")
    parser.add_argument("--no-trigger-planning-command", action="store_true")
    parser.add_argument(
        "--planning-command-topic",
        default="/apollo/planning/command",
    )
    parser.add_argument("--trigger-delay-sec", type=float, default=1.0)
    parser.add_argument(
        "--flag",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="extra gflag passed to scenario_injector; can be repeated",
    )
    return parser


def main():
    args = build_parser().parse_args()
    try:
        return FusedScenarioRunner(args).run()
    except SystemExit:
        raise
    except Exception as error:
        sys.stderr.write("run_fused_scenario error: {}\n".format(error))
        return 1


if __name__ == "__main__":
    sys.exit(main())
