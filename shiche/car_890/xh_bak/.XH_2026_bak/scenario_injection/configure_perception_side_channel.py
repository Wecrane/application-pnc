#!/usr/bin/env python3
"""Redirect Apollo perception output to a side channel for scenario fusion."""

import argparse
from dataclasses import dataclass
import os
import shutil
import sys

DEFAULT_CONFIGS = [
    "/apollo/modules/perception/data/flag/perception_common.flag",
    "/apollo/modules/perception/multi_sensor_fusion/conf/multi_sensor_fusion_config.pb.txt",
]
DEFAULT_ORIGINAL_TOPIC = "/apollo/perception/obstacles"
DEFAULT_SIDE_TOPIC = "/apollo/perception/lidar_obstacles"


@dataclass
class ConfigureResult:
    config_path: str
    status: str
    backup_path: str = ""
    matched: str = ""


def backup_path_for(config_path):
    return config_path + ".scenario_injection.bak"


def update_content(content, original_topic, side_topic, config_path):
    replacements = [
        (
            'output_obstacles_channel_name: "{}"'.format(original_topic),
            'output_obstacles_channel_name: "{}"'.format(side_topic),
        ),
        (
            "--perception_obstacle_topic={}".format(original_topic),
            "--perception_obstacle_topic={}".format(side_topic),
        ),
    ]
    for old_line, new_line in replacements:
        if new_line in content:
            return content, "already"
        if old_line in content:
            return content.replace(old_line, new_line, 1), old_line
    if config_path.endswith("perception_common.flag"):
        new_line = "--perception_obstacle_topic={}".format(side_topic)
        return content.rstrip() + "\n\n# scenario_injection override\n{}\n".format(new_line), new_line
    return content, None


def configure_one(config_path, original_topic, side_topic, dry_run=False):
    if not os.path.isfile(config_path):
        return ConfigureResult(config_path, "missing")
    with open(config_path, "r") as source_file:
        content = source_file.read()
    updated, matched = update_content(content, original_topic, side_topic, config_path)
    if matched == "already":
        print("already configured: {}".format(config_path))
        return ConfigureResult(config_path, "already")
    if matched is None:
        return ConfigureResult(config_path, "unmatched")

    backup_path = backup_path_for(config_path)
    print("{} -> {}".format(original_topic, side_topic))
    print("config: {}".format(config_path))
    print("matched: {}".format(matched))
    print("backup: {}".format(backup_path))
    if dry_run:
        print("dry run only; no file changed")
        return ConfigureResult(config_path, "would_change", backup_path, matched)
    if not os.path.exists(backup_path):
        shutil.copy2(config_path, backup_path)
    with open(config_path, "w") as target_file:
        target_file.write(updated)
    return ConfigureResult(config_path, "changed", backup_path, matched)


def restore_one(config_path, dry_run=False):
    backup_path = backup_path_for(config_path)
    if not os.path.isfile(backup_path):
        return ConfigureResult(config_path, "missing_backup", backup_path)
    print("restore: {} <- {}".format(config_path, backup_path))
    if dry_run:
        print("dry run only; no file changed")
        return ConfigureResult(config_path, "would_restore", backup_path)
    shutil.copy2(backup_path, config_path)
    return ConfigureResult(config_path, "restored", backup_path)


def configure(config_paths, original_topic, side_topic, dry_run=False):
    results = []
    for config_path in config_paths:
        results.append(configure_one(config_path, original_topic, side_topic, dry_run))
    if not any(result.status in ("already", "would_change", "changed") for result in results):
        raise ValueError(
            "cannot find output topic config in: {}".format(
                ", ".join(config_paths)
            )
        )
    return results


def restore(config_paths, dry_run=False):
    results = [restore_one(config_path, dry_run) for config_path in config_paths]
    if not any(result.status in ("would_restore", "restored") for result in results):
        raise ValueError(
            "cannot find scenario_injection backups for: {}".format(
                ", ".join(config_paths)
            )
        )
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Move perception output to /apollo/perception/lidar_obstacles."
    )
    parser.add_argument(
        "--config",
        action="append",
        default=[],
        help="config file to update; can be repeated",
    )
    parser.add_argument("--original-topic", default=DEFAULT_ORIGINAL_TOPIC)
    parser.add_argument("--side-topic", default=DEFAULT_SIDE_TOPIC)
    parser.add_argument("--restore", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    config_paths = args.config or DEFAULT_CONFIGS
    try:
        if args.restore:
            restore(config_paths, args.dry_run)
        else:
            configure(config_paths, args.original_topic, args.side_topic, args.dry_run)
    except Exception as error:
        sys.stderr.write("configure_perception_side_channel error: {}\n".format(error))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
