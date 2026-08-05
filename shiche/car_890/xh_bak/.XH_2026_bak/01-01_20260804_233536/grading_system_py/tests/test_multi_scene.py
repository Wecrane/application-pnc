"""多场景评分隔离与 manifest 解析测试。"""

import json
import sys

import pytest

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.main import (
    GradingEngine,
    GradingWorld,
    parse_scene_manifest,
    parse_config_file,
)


def _write_metric(path, max_speed=1.0):
    path.write_text(json.dumps({
        'metric': [{
            'name': 'SpeedLimit',
            'is_critical': True,
            'require_all_time_pass': True,
            'condition': {
                'speed_condition': {
                    'min_speed': 0.0,
                    'max_speed': max_speed,
                    'use_score': True,
                    'single_deduction': 10,
                    'deduction_unit': 1,
                }
            },
        }]
    }), encoding='utf-8')


def test_new_engine_does_not_inherit_previous_scene_failure(tmp_path):
    config_path = tmp_path / 'metric.json'
    _write_metric(config_path)
    config_a = parse_config_file(str(config_path))
    config_b = parse_config_file(str(config_path))

    scene_a = GradingEngine(config_a)
    failed_world = GradingWorld()
    failed_world.timestamp_sec = 0.0
    failed_world.auto_driving_car.speed = 5.0
    scene_a.feed_frame(failed_world)
    assert scene_a.get_summary()['is_scenario_pass'] is False

    scene_b = GradingEngine(config_b)
    passed_world = GradingWorld()
    passed_world.timestamp_sec = 0.0
    passed_world.auto_driving_car.speed = 0.5
    scene_b.feed_frame(passed_world)
    summary_b = scene_b.get_summary()

    assert summary_b['is_scenario_pass'] is True
    assert summary_b['metrics']['SpeedLimit']['fail_count'] == 0
    assert summary_b['metrics']['SpeedLimit']['total_frames'] == 1


def test_manifest_resolves_paths_and_sorts_windows(tmp_path):
    metric_a = tmp_path / 'a.json'
    metric_b = tmp_path / 'b.json'
    _write_metric(metric_a)
    _write_metric(metric_b)
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps({
        'scenes': [
            {'id': 'B', 'config': 'b.json',
             'window': {'start_sec': 5, 'end_sec': 10}},
            {'id': 'A', 'config': 'a.json',
             'window': {'start_sec': 0, 'end_sec': 4}},
        ]
    }), encoding='utf-8')

    scenes = parse_scene_manifest(str(manifest))

    assert [scene.name for scene in scenes] == ['A', 'B']
    assert scenes[0].config_path == str(metric_a)
    assert scenes[1].start_sec == 5.0


def test_manifest_rejects_overlapping_windows(tmp_path):
    metric = tmp_path / 'metric.json'
    _write_metric(metric)
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps({
        'scenes': [
            {'name': 'A', 'config': 'metric.json',
             'start_sec': 0, 'end_sec': 5},
            {'name': 'B', 'config': 'metric.json',
             'start_sec': 4, 'end_sec': 8},
        ]
    }), encoding='utf-8')

    with pytest.raises(ValueError, match='non-overlapping'):
        parse_scene_manifest(str(manifest))


def test_manifest_supports_full_mode_and_top_level_map(tmp_path):
    metric = tmp_path / 'metric.json'
    map_file = tmp_path / 'base_map.bin'
    _write_metric(metric)
    map_file.write_bytes(b'placeholder')
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps({
        'boundary_mode': 'full',
        'map_file': 'base_map.bin',
        'scenes': [
            {'name': 'A', 'config': 'metric.json'},
            {'name': 'B', 'config': 'metric.json'},
        ],
    }), encoding='utf-8')

    scenes = parse_scene_manifest(str(manifest))

    assert [scene.boundary_mode for scene in scenes] == ['full', 'full']
    assert [scene.map_file for scene in scenes] == [str(map_file), str(map_file)]
