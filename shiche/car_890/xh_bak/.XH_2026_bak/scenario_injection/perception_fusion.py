#!/usr/bin/env python3
"""Fuse lidar perception obstacles with scenario-injected obstacles."""

import argparse
import copy
import sys
import threading
import time


def ensure_apollo_python_path():
    import os
    for path in (
        "/apollo_workspace",
        "/apollo_workspace/.aem/envroot/opt/apollo/neo/python",
        "/apollo_workspace/.aem/envroot/opt/apollo/neo/lib",
    ):
        if path not in sys.path and os.path.exists(path):
            sys.path.insert(0, path)


ensure_apollo_python_path()
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles


class PerceptionFusion:
    def __init__(
        self,
        lidar_topic,
        scenario_topic,
        output_topic,
        scenario_timeout_sec,
        module_name,
        scenario_id_offset,
    ):
        self.lidar_topic = lidar_topic
        self.scenario_topic = scenario_topic
        self.output_topic = output_topic
        self.scenario_timeout_sec = scenario_timeout_sec
        self.module_name = module_name
        self.scenario_id_offset = scenario_id_offset
        self.lock = threading.Lock()
        self.latest_scenario = None
        self.latest_scenario_time = 0.0
        self.sequence_num = 1

    def start(self):
        cyber.init(self.module_name)
        self.node = cyber.Node(self.module_name)
        self.writer = self.node.create_writer(self.output_topic, PerceptionObstacles)
        self.node.create_reader(self.scenario_topic, PerceptionObstacles, self.on_scenario)
        self.node.create_reader(self.lidar_topic, PerceptionObstacles, self.on_lidar)
        sys.stderr.write(
            "perception_fusion: {} + {} -> {}\n".format(
                self.lidar_topic, self.scenario_topic, self.output_topic
            )
        )
        self.node.spin()

    def on_scenario(self, message):
        with self.lock:
            self.latest_scenario = copy.deepcopy(message)
            self.latest_scenario_time = time.time()

    def on_lidar(self, lidar_message):
        fused = PerceptionObstacles()
        fused.CopyFrom(lidar_message)
        now = cyber_time.Time.now().to_sec()
        fused.header.timestamp_sec = now
        fused.header.sequence_num = self.sequence_num
        fused.header.module_name = self.module_name
        self.sequence_num += 1

        scenario = None
        with self.lock:
            if (
                self.latest_scenario is not None
                and time.time() - self.latest_scenario_time <= self.scenario_timeout_sec
            ):
                scenario = copy.deepcopy(self.latest_scenario)

        if scenario is not None:
            for obstacle in scenario.perception_obstacle:
                fused_obstacle = fused.perception_obstacle.add()
                fused_obstacle.CopyFrom(obstacle)
                if self.scenario_id_offset is not None:
                    fused_obstacle.id += self.scenario_id_offset
                if fused_obstacle.timestamp == 0:
                    fused_obstacle.timestamp = now

        self.writer.write(fused)


def main():
    parser = argparse.ArgumentParser(
        description="Fuse lidar perception and scenario injected obstacles."
    )
    parser.add_argument(
        "--lidar-topic",
        default="/apollo/perception/lidar_obstacles",
        help="real perception obstacle input topic",
    )
    parser.add_argument(
        "--scenario-topic",
        default="/apollo/perception/scenario_obstacles",
        help="scenario injected obstacle input topic",
    )
    parser.add_argument(
        "--output-topic",
        default="/apollo/perception/obstacles",
        help="fused perception obstacle output topic",
    )
    parser.add_argument(
        "--scenario-timeout-sec",
        type=float,
        default=0.5,
        help="drop scenario obstacles if no scenario frame is received within this time",
    )
    parser.add_argument(
        "--scenario-id-offset",
        type=int,
        default=100000,
        help="offset added to injected obstacle ids to avoid collisions",
    )
    parser.add_argument(
        "--no-scenario-id-offset",
        action="store_true",
        help="preserve injected obstacle ids exactly as published",
    )
    parser.add_argument(
        "--module-name",
        default="perception_fusion",
        help="Cyber node and output header module name",
    )
    args = parser.parse_args()

    fusion = PerceptionFusion(
        lidar_topic=args.lidar_topic,
        scenario_topic=args.scenario_topic,
        output_topic=args.output_topic,
        scenario_timeout_sec=args.scenario_timeout_sec,
        module_name=args.module_name,
        scenario_id_offset=None if args.no_scenario_id_offset else args.scenario_id_offset,
    )
    fusion.start()


if __name__ == "__main__":
    main()
