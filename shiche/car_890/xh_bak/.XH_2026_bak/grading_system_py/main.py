#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grading_system_py 主程序入口

支持两种数据源模式:
  1. bag模式: 读取仿真输出的 record/bag 文件进行离线评分
  2. channel模式: 订阅 Cyber 通道进行实时评分

使用方法:
  # 模式1: 读取bag文件
  python3 main.py --config grading_system/conf/grading_metrics_default.conf \
                  --mode bag \
                  --bag_file /apollo-simulator/output_data/1/1.output.bag

  # 模式2: 订阅Cyber通道 (需要在Apollo环境中运行)
  python3 main.py --config grading_system/conf/grading_metrics_default.conf \
                  --mode channel
"""

import argparse
import copy
import json
import math
import os
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from grading_system_py.condition.acceleration_handler import AccelerationConditionHandler
from grading_system_py.condition.speed_handler import SpeedConditionHandler
from grading_system_py.condition.on_road_handler import OnRoadConditionHandler
from grading_system_py.condition.obstacle_bypass_handler import ObstacleBypassConditionHandler
from grading_system_py.condition.ttc_handler import TtcConditionHandler
from grading_system_py.condition.centripetal_acceleration_handler import CentripetalAccelerationConditionHandler
from grading_system_py.condition.time_limit_handler import TimeLimitConditionHandler
from grading_system_py.condition.region_overlap_lw_handler import RegionOverlapLwConditionHandler
from grading_system_py.condition.object_overlap_handler import ObjectOverlapConditionHandler
from grading_system_py.condition.red_light_stop_handler import RedLightStopConditionHandler
from grading_system_py.condition.crosswalk_stop_handler import CrosswalkStopConditionHandler
from grading_system_py.condition.stop_sign_stop_handler import StopSignStopConditionHandler
from grading_system_py.condition.condition_handler_base import ConditionHandlerBase


# ============================================================
# 数据模型 (与 protobuf 兼容的轻量结构)
# ============================================================

class Object:
    """对应 GradingWorld.Object"""
    def __init__(self):
        self.id = ""
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.speed_acceleration = 0.0
        self.speed_heading = 0.0
        self.spin = 0.0  # 角速度 (rad/s)
        self.length = 4.5
        self.width = 2.0
        self.height = 1.8
        self.type = "UNKNOWN"
        self.current_signal = ""
        self.front_edge_to_center = 2.25
        self.back_edge_to_center = 2.25
        self.left_edge_to_center = 1.0
        self.right_edge_to_center = 1.0
        self.polygon_point = []


class GradingWorld:
    """对应 proto GradingWorld"""
    def __init__(self):
        self.timestamp_sec = 0.0
        self.auto_driving_car = Object()
        self.object: List[Object] = []
        self.speed_limit: Optional[float] = None
        self.last_frame = False
        self.perceived_signal = []


class DetailedResult:
    """评分详细结果"""
    def __init__(self):
        self.timestamp = 0.0
        self.is_pass = True
        self.description = ""
        self.delta_score = 0.0
        self.failure_code = ""

    def add_description(self, text: str):
        self.description += text

    def add_failure(self, code: str, text: str):
        if not self.failure_code:
            self.failure_code = code
        self.add_description(text)


_DEFAULT_VEHICLE_PARAM = {
    'length': 4.5,
    'width': 2.0,
    'height': 1.8,
    'front_edge_to_center': 2.25,
    'back_edge_to_center': 2.25,
    'left_edge_to_center': 1.0,
    'right_edge_to_center': 1.0,
    'max_acceleration': 3.0,
    'max_deceleration': -6.0,
}
_VEHICLE_PARAM_CACHE = {}


def _load_vehicle_param(vehicle_param_file: str = '') -> dict:
    """Load Apollo vehicle geometry for ADC collision and stop-line checks."""
    global _VEHICLE_PARAM_CACHE
    explicit_path = os.path.realpath(vehicle_param_file) if vehicle_param_file else ''
    cache_key = explicit_path or os.path.realpath(
        os.environ.get('APOLLO_VEHICLE_PARAM_FILE', '')) or 'default'
    if cache_key in _VEHICLE_PARAM_CACHE:
        return _VEHICLE_PARAM_CACHE[cache_key]

    params = dict(_DEFAULT_VEHICLE_PARAM)
    candidates = []
    if explicit_path:
        candidates.append(explicit_path)
    env_path = os.environ.get('APOLLO_VEHICLE_PARAM_FILE', '')
    if env_path and env_path != explicit_path:
        candidates.append(env_path)
    candidates.extend([
        '/apollo/modules/common/data/vehicle_param.pb.txt',
        '/apollo_workspace/modules/common/data/vehicle_param.pb.txt',
        '/apollo_workspace/.aem/envroot/apollo/modules/common/data/vehicle_param.pb.txt',
        '/apollo_workspace/.aem/envroot/opt/apollo/neo/share/modules/common/data/vehicle_param.pb.txt',
    ])
    for path in candidates:
        if not path or not os.path.isfile(path):
            continue
        try:
            from google.protobuf import text_format
            from modules.common_msgs.config_msgs import vehicle_config_pb2
            config = vehicle_config_pb2.VehicleConfig()
            with open(path, 'r', encoding='utf-8') as f:
                text_format.Merge(f.read(), config)
            vehicle = config.vehicle_param
            params.update({
                'length': vehicle.length,
                'width': vehicle.width,
                'height': vehicle.height,
                'front_edge_to_center': vehicle.front_edge_to_center,
                'back_edge_to_center': vehicle.back_edge_to_center,
                'left_edge_to_center': vehicle.left_edge_to_center,
                'right_edge_to_center': vehicle.right_edge_to_center,
                'max_acceleration': vehicle.max_acceleration,
                'max_deceleration': vehicle.max_deceleration,
            })
            break
        except Exception as exc:
            print(f"[WARN] Failed to load vehicle param {path}: {exc}")
    _VEHICLE_PARAM_CACHE[cache_key] = params
    print(
        f"[Config] Vehicle param: length={params['length']:.3f}, "
        f"width={params['width']:.3f}, "
        f"front={params['front_edge_to_center']:.3f}, "
        f"back={params['back_edge_to_center']:.3f}")
    return params


def _apply_vehicle_param(adc, vehicle_param_file: str = ''):
    vehicle = _load_vehicle_param(vehicle_param_file)
    adc.length = vehicle['length']
    adc.width = vehicle['width']
    adc.height = vehicle['height']
    adc.front_edge_to_center = vehicle['front_edge_to_center']
    adc.back_edge_to_center = vehicle['back_edge_to_center']
    adc.left_edge_to_center = vehicle['left_edge_to_center']
    adc.right_edge_to_center = vehicle['right_edge_to_center']


# ============================================================
# 配置解析
# ============================================================

class ConditionConfig:
    """条件配置 - 从 .conf 文件解析"""
    def __init__(self):
        self.condition_type = ""  # speed_condition, acceleration_condition, etc.
        # 通用字段
        self.name = ""
        self.use_score = False
        self.single_deduction = 5.0
        self.deduction_unit = 1.0
        # SpeedCondition
        self.min_speed = -0.5
        self.max_speed = 1000.0
        self.speed_limit_regions = []
        # AccelerationCondition
        self.min_acceleration = -1000.0
        self.max_acceleration = 1000.0
        self.duration = 2.0
        self.cut_in_distance = 1.0
        self.ttc = 5.0
        self.dist_stopline = 25.0
        self.light_turn_prev_time = 4.0
        # OnRoadCondition
        self.use_road_boundary = False
        # ObstacleBypassCondition
        self.obstacle_id = ""
        self.min_lateral_distance = 1.0
        self.test_range = None
        # TtcCondition
        self.time = 3.0
        # CentripetalAccelerationCondition
        self.max_centripetal_acceleration = 2.0
        # TimeLimitCondition
        self.timeout = 100.0
        # RegionOverlapLWCondition
        self.x = 0.0
        self.y = 0.0
        self.length = 5.0
        self.width = 12.0
        self.object_ids = ""
        self.require_fully_contain = False
        # ObjectOverlapCondition
        self.source_object_ids = ""
        self.target_object_ids = "*"
        self.distance = 0.05
        self.direction = "INCLUDE_BACK"
        self.ignore_object_ids = []
        self.target_types = []
        # RedLightStopCondition / CrosswalkStopCondition
        self.min_distance = 2.0
        self.max_distance = 2.2
        self.crossing_tolerance = 0.05
        self.target_ids = []
        # StopSignStopCondition
        self.stop_speed_eps = 0.1


class MetricConfig:
    """单个 Metric 配置"""
    def __init__(self):
        self.name = ""
        self.description = ""
        self.is_critical = True
        self.require_all_time_pass = True
        self.once_pass_stay_pass = True
        self.get_deduction_score = True  # True=扣分制, False=得分制
        self.condition = ConditionConfig()


class GradingConfig:
    """完整评分配置"""
    def __init__(self):
        self.metrics: List[MetricConfig] = []
        self.use_score = False


@dataclass
class SceneSpec:
    """一个待评估场景及其可选的独立 bag/时间窗口。"""
    name: str
    config_path: str
    scenario_path: str = ""
    map_file: str = ""
    vehicle_param_file: str = ""
    bag_file: str = ""
    start_sec: float = 0.0
    end_sec: Optional[float] = None
    has_window: bool = False
    boundary_mode: str = "auto"
    scope: str = "full"
    scope_ids: List[str] = None
    scope_forward_distance: float = 80.0
    scope_past_distance: float = 8.0


def parse_config_file(config_path: str) -> GradingConfig:
    """
    解析评分配置文件，支持两种格式:
    - JSON 格式 (.json)
    - Protobuf text format (.conf)
    """
    if config_path.endswith('.json'):
        return _parse_json_config(config_path)
    else:
        return _parse_text_config(config_path)


def _parse_json_config(config_path: str) -> GradingConfig:
    """解析 JSON 格式的配置文件"""
    import json

    config = GradingConfig()

    with open(config_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    config.use_score = data.get('use_score', False)

    for metric_data in data.get('metric', []):
        metric = MetricConfig()
        metric.name = metric_data.get('name', '')
        metric.description = metric_data.get('description', '')
        metric.is_critical = metric_data.get('is_critical', True)
        metric.require_all_time_pass = metric_data.get('require_all_time_pass', True)
        metric.once_pass_stay_pass = metric_data.get('once_pass_stay_pass', True)
        metric.get_deduction_score = metric_data.get('get_deduction_score', True)

        condition_data = metric_data.get('condition', {})
        cond = metric.condition

        # 检测 condition 类型
        supported_types = [
            'speed_condition', 'acceleration_condition', 'on_road_condition',
            'obstacle_bypass_condition', 'ttc_condition',
            'centripetal_acceleration_condition', 'time_limit_condition',
            'region_overlap_lw_condition', 'object_overlap_condition',
            'red_light_stop_condition', 'crosswalk_stop_condition',
            'stop_sign_stop_condition',
        ]
        found = False
        for ctype in supported_types:
            if ctype in condition_data:
                cond.condition_type = ctype
                _fill_condition_from_json(cond, ctype, condition_data[ctype])
                found = True
                break

        if found:
            config.metrics.append(metric)

    return config


def _fill_condition_from_json(cond: ConditionConfig, ctype: str, data: dict):
    """从 JSON dict 填充 condition 字段"""
    cond.name = data.get('name', '')
    cond.use_score = data.get('use_score', False)
    cond.single_deduction = data.get('single_deduction', cond.single_deduction)
    cond.deduction_unit = data.get('deduction_unit', cond.deduction_unit)

    if ctype == 'speed_condition':
        cond.min_speed = data.get('min_speed', cond.min_speed)
        cond.max_speed = data.get('max_speed', cond.max_speed)
        # 限速区域
        for region_data in data.get('speed_limit_regions', []):
            cond.speed_limit_regions.append(region_data)

    elif ctype == 'acceleration_condition':
        cond.min_acceleration = data.get('min_acceleration', cond.min_acceleration)
        cond.max_acceleration = data.get('max_acceleration', cond.max_acceleration)
        cond.duration = data.get('duration', cond.duration)
        cond.cut_in_distance = data.get('cut_in_distance', cond.cut_in_distance)
        cond.ttc = data.get('ttc', cond.ttc)
        cond.dist_stopline = data.get('dist_stopline', cond.dist_stopline)
        cond.light_turn_prev_time = data.get('light_turn_prev_time', cond.light_turn_prev_time)

    elif ctype == 'on_road_condition':
        cond.use_road_boundary = data.get('use_road_boundary', False)

    elif ctype == 'obstacle_bypass_condition':
        cond.obstacle_id = data.get('obstacle_id', '')
        cond.min_lateral_distance = data.get('min_lateral_distance', 1.0)
        cond.max_speed = data.get('max_speed', cond.max_speed)

    elif ctype == 'ttc_condition':
        cond.time = data.get('time', cond.time)

    elif ctype == 'centripetal_acceleration_condition':
        cond.max_centripetal_acceleration = data.get(
            'max_centripetal_acceleration', cond.max_centripetal_acceleration)

    elif ctype == 'time_limit_condition':
        cond.timeout = data.get('timeout', cond.timeout)

    elif ctype == 'region_overlap_lw_condition':
        cond.x = data.get('x', cond.x)
        cond.y = data.get('y', cond.y)
        cond.length = data.get('length', cond.length)
        cond.width = data.get('width', cond.width)
        cond.object_ids = data.get('object_ids', '')
        cond.require_fully_contain = data.get('require_fully_contain', False)

    elif ctype == 'object_overlap_condition':
        cond.source_object_ids = data.get('source_object_ids', '')
        cond.target_object_ids = data.get('target_object_ids', '*')
        cond.distance = data.get('distance', 0.05)
        cond.direction = data.get('direction', 'INCLUDE_BACK')
        cond.ignore_object_ids = data.get('ignore_object_ids', [])
        cond.target_types = data.get('target_types', [])

    elif ctype == 'red_light_stop_condition':
        cond.min_distance = data.get('min_distance', 2.0)
        cond.max_distance = data.get('max_distance', 2.2)
        cond.stop_speed_eps = data.get('stop_speed_eps', cond.stop_speed_eps)
        cond.crossing_tolerance = data.get('crossing_tolerance', cond.crossing_tolerance)
        cond.target_ids = data.get('target_ids', [])

    elif ctype == 'crosswalk_stop_condition':
        cond.min_distance = data.get('min_distance', 2.2)
        cond.max_distance = data.get('max_distance', 2.7)
        cond.stop_speed_eps = data.get('stop_speed_eps', cond.stop_speed_eps)
        cond.crossing_tolerance = data.get('crossing_tolerance', cond.crossing_tolerance)
        cond.target_ids = data.get('target_ids', [])

    elif ctype == 'stop_sign_stop_condition':
        cond.min_distance = data.get('min_distance', 0.0)
        cond.max_distance = data.get('max_distance', 2.0)
        cond.stop_speed_eps = data.get('stop_speed_eps', cond.stop_speed_eps)
        cond.target_ids = data.get('target_ids', [])


def _parse_text_config(config_path: str) -> GradingConfig:
    """解析 protobuf text format 的配置文件 (.conf)"""
    config = GradingConfig()

    with open(config_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 简易解析器 (处理 protobuf text format)
    metrics = _split_metrics(content)
    for metric_text in metrics:
        metric = _parse_metric(metric_text)
        if metric:
            config.metrics.append(metric)

    # 检查顶层 use_score
    if 'use_score: true' in content.split('metric')[0]:
        config.use_score = True

    return config


def _split_metrics(content: str) -> List[str]:
    """拆分出各个 metric 块"""
    metrics = []
    depth = 0
    current = ""
    in_metric = False

    i = 0
    while i < len(content):
        # 跳过注释
        if content[i] == '#':
            while i < len(content) and content[i] != '\n':
                i += 1
            continue

        if content[i:i+6] == 'metric' and depth == 0:
            in_metric = True
            # 找到开始的 {
            while i < len(content) and content[i] != '{':
                i += 1
            depth = 1
            current = ""
            i += 1
            continue

        if in_metric:
            if content[i] == '{':
                depth += 1
            elif content[i] == '}':
                depth -= 1
                if depth == 0:
                    metrics.append(current)
                    in_metric = False
                    i += 1
                    continue
            current += content[i]
        i += 1

    return metrics


def _parse_metric(text: str) -> Optional[MetricConfig]:
    """解析单个 metric 块"""
    metric = MetricConfig()

    metric.name = _extract_string(text, 'name')
    metric.description = _extract_string(text, 'description')
    metric.is_critical = _extract_bool(text, 'is_critical', True)
    metric.require_all_time_pass = _extract_bool(text, 'require_all_time_pass', True)

    # 检测 condition 类型
    condition_types = [
        # 注意顺序：长名字放前面，避免 centripetal_acceleration_condition 被 acceleration_condition 命中
        'centripetal_acceleration_condition',
        'red_light_stop_condition', 'crosswalk_stop_condition',
        'stop_sign_stop_condition',
        'region_overlap_lw_condition', 'object_overlap_condition',
        'time_limit_condition', 'obstacle_bypass_condition', 'ttc_condition',
        'speed_condition', 'acceleration_condition', 'on_road_condition',
    ]
    for ctype in condition_types:
        if ctype in text:
            metric.condition.condition_type = ctype
            _parse_condition_fields(text, ctype, metric.condition)
            return metric

    # 不支持的 condition 类型，跳过
    return None


def _parse_condition_fields(text: str, ctype: str, cond: ConditionConfig):
    """解析 condition 内部字段"""
    # 提取 condition 块内容
    start = text.find(ctype)
    if start < 0:
        return
    brace_start = text.find('{', start)
    if brace_start < 0:
        return

    depth = 1
    i = brace_start + 1
    block = ""
    while i < len(text) and depth > 0:
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                break
        block += text[i]
        i += 1

    cond.name = _extract_string(block, 'name') or cond.name
    cond.use_score = _extract_bool(block, 'use_score', False)
    cond.single_deduction = _extract_float(block, 'single_deduction', cond.single_deduction)
    cond.deduction_unit = _extract_float(block, 'deduction_unit', cond.deduction_unit)

    if ctype == 'speed_condition':
        cond.min_speed = _extract_float(block, 'min_speed', cond.min_speed)
        cond.max_speed = _extract_float(block, 'max_speed', cond.max_speed)
    elif ctype == 'acceleration_condition':
        cond.min_acceleration = _extract_float(block, 'min_acceleration', cond.min_acceleration)
        cond.max_acceleration = _extract_float(block, 'max_acceleration', cond.max_acceleration)
        cond.duration = _extract_float(block, 'duration', cond.duration)
        cond.cut_in_distance = _extract_float(block, 'cut_in_distance', cond.cut_in_distance)
        cond.ttc = _extract_float(block, 'ttc', cond.ttc)
        cond.dist_stopline = _extract_float(block, 'dist_stopline', cond.dist_stopline)
        cond.light_turn_prev_time = _extract_float(block, 'light_turn_prev_time', cond.light_turn_prev_time)
    elif ctype == 'on_road_condition':
        cond.use_road_boundary = _extract_bool(block, 'use_road_boundary', False)
    elif ctype == 'obstacle_bypass_condition':
        cond.obstacle_id = _extract_string(block, 'obstacle_id') or ""
        cond.min_lateral_distance = _extract_float(block, 'min_lateral_distance', 1.0)
        cond.max_speed = _extract_float(block, 'max_speed', cond.max_speed)
    elif ctype == 'ttc_condition':
        cond.time = _extract_float(block, 'time', cond.time)
    elif ctype == 'red_light_stop_condition':
        cond.min_distance = _extract_float(block, 'min_distance', 2.0)
        cond.max_distance = _extract_float(block, 'max_distance', 2.2)
        cond.stop_speed_eps = _extract_float(block, 'stop_speed_eps', cond.stop_speed_eps)
        cond.crossing_tolerance = _extract_float(
            block, 'crossing_tolerance', cond.crossing_tolerance)
        cond.target_ids = _extract_repeated_strings_or_enums(block, 'target_ids')
    elif ctype == 'crosswalk_stop_condition':
        cond.min_distance = _extract_float(block, 'min_distance', 2.2)
        cond.max_distance = _extract_float(block, 'max_distance', 2.7)
        cond.stop_speed_eps = _extract_float(block, 'stop_speed_eps', cond.stop_speed_eps)
        cond.crossing_tolerance = _extract_float(
            block, 'crossing_tolerance', cond.crossing_tolerance)
        cond.target_ids = _extract_repeated_strings_or_enums(block, 'target_ids')
    elif ctype == 'stop_sign_stop_condition':
        cond.min_distance = _extract_float(block, 'min_distance', 0.0)
        cond.max_distance = _extract_float(block, 'max_distance', 2.0)
        cond.stop_speed_eps = _extract_float(block, 'stop_speed_eps', cond.stop_speed_eps)
        cond.target_ids = _extract_repeated_strings_or_enums(block, 'target_ids')
    elif ctype == 'centripetal_acceleration_condition':
        cond.max_centripetal_acceleration = _extract_float(
            block, 'max_centripetal_acceleration', cond.max_centripetal_acceleration)
    elif ctype == 'time_limit_condition':
        cond.timeout = _extract_float(block, 'timeout', cond.timeout)
    elif ctype == 'region_overlap_lw_condition':
        cond.object_ids = _extract_string(block, 'object_ids') or ''
        cond.x = _extract_float(block, 'x', cond.x)
        cond.y = _extract_float(block, 'y', cond.y)
        cond.length = _extract_float(block, 'length', cond.length)
        cond.width = _extract_float(block, 'width', cond.width)
        cond.require_fully_contain = _extract_bool(block, 'require_fully_contain', False)
    elif ctype == 'object_overlap_condition':
        cond.source_object_ids = _extract_string(block, 'source_object_ids') or ''
        cond.target_object_ids = _extract_string(block, 'target_object_ids') or '*'
        cond.distance = _extract_float(block, 'distance', cond.distance)
        direction = _extract_enum(block, 'direction')
        if direction:
            cond.direction = direction
        cond.target_types = _extract_repeated_strings_or_enums(block, 'target_types')


def _extract_repeated_strings_or_enums(text: str, field: str) -> List[str]:
    """提取重复字符串/枚举字段，兼容 `field: "A"` 和 `field: A`。"""
    import re
    values = []
    pattern = rf'{field}\s*:\s*(?:"([^"]*)"|([A-Z_]+))'
    for match in re.finditer(pattern, text):
        values.append(match.group(1) or match.group(2))
    return values


def _extract_string(text: str, field: str) -> str:
    """提取字符串字段值"""
    import re
    pattern = rf'{field}\s*:\s*"([^"]*)"'
    m = re.search(pattern, text)
    return m.group(1) if m else ""


def _extract_float(text: str, field: str, default: float) -> float:
    """提取浮点数字段值"""
    import re
    pattern = rf'{field}\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)'
    m = re.search(pattern, text)
    return float(m.group(1)) if m else default


def _extract_enum(text: str, field: str) -> str:
    """提取枚举字段值，如 direction: INCLUDE_BACK"""
    import re
    pattern = rf'{field}\s*:\s*([A-Z_]+)'
    m = re.search(pattern, text)
    return m.group(1) if m else ""


def _extract_bool(text: str, field: str, default: bool) -> bool:
    """提取布尔字段值"""
    import re
    pattern = rf'{field}\s*:\s*(true|false)'
    m = re.search(pattern, text)
    if m:
        return m.group(1) == 'true'
    return default


# ============================================================
# Condition 适配器 (让 handler 可以直接用 ConditionConfig)
# ============================================================

def parse_scene_manifest(manifest_path: str) -> List[SceneSpec]:
    """解析一个 bag 多场景清单，时间使用 bag 起始时刻后的相对秒数。"""
    with open(manifest_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    raw_scenes = data.get('scenes') if isinstance(data, dict) else None
    if not isinstance(raw_scenes, list) or not raw_scenes:
        raise ValueError("scene manifest must contain a non-empty 'scenes' list")

    raw_boundary_mode = data.get('boundary_mode')
    has_any_window = any(
        isinstance(item, dict) and (
            isinstance(item.get('window'), dict) and bool(item.get('window')) or
            any(key in item for key in ('start_sec', 'end_sec', 'start_time', 'end_time'))
        ) for item in raw_scenes)
    boundary_mode = raw_boundary_mode or ('window' if has_any_window else 'auto')
    if boundary_mode not in ('auto', 'window', 'full'):
        raise ValueError("boundary_mode must be 'auto', 'window', or 'full'")

    manifest_dir = os.path.dirname(os.path.abspath(manifest_path))
    manifest_map_file = data.get('map_file', '')
    manifest_vehicle_param_file = data.get('vehicle_param_file', '')
    raw_bag_dir = data.get('bag_dir', '')
    bag_dir = (os.path.normpath(os.path.join(manifest_dir, raw_bag_dir))
               if raw_bag_dir and not os.path.isabs(raw_bag_dir)
               else raw_bag_dir)
    scenes = []
    for index, item in enumerate(raw_scenes):
        if not isinstance(item, dict):
            raise ValueError(f"scene[{index}] must be an object")
        name = str(item.get('name', item.get('id', f"scene_{index + 1}"))).strip()
        if not name:
            raise ValueError(f"scene[{index}] is missing name/id")
        config_path = str(item.get('config', '')).strip()
        if not config_path:
            raise ValueError(f"scene[{index}] is missing config")
        scenario_path = str(item.get('scenario', '')).strip()
        map_file = str(item.get('map_file', manifest_map_file)).strip()
        vehicle_param_file = str(
            item.get('vehicle_param_file', manifest_vehicle_param_file)).strip()
        bag_file = str(item.get('bag_file', '')).strip()
        scope = str(item.get('scope', 'full')).strip().lower() or 'full'
        if scope not in ('full', 'red_light', 'crosswalk', 'stop_sign'):
            raise ValueError(
                f"scene[{index}] scope must be 'full', 'red_light', 'crosswalk', or 'stop_sign'")
        scope_forward_distance = float(item.get('scope_forward_distance', 80.0))
        scope_past_distance = float(item.get('scope_past_distance', 8.0))
        raw_scope_ids = item.get('scope_ids', [])
        if isinstance(raw_scope_ids, str):
            scope_ids = [raw_scope_ids]
        elif isinstance(raw_scope_ids, list):
            scope_ids = [str(value) for value in raw_scope_ids]
        else:
            raise ValueError(f"scene[{index}] scope_ids must be a string or list")

        def resolve(path):
            if path and not os.path.isabs(path):
                return os.path.normpath(os.path.join(manifest_dir, path))
            return path

        config_path = resolve(config_path)
        scenario_path = resolve(scenario_path)
        map_file = resolve(map_file)
        vehicle_param_file = resolve(vehicle_param_file)
        bag_file = resolve(bag_file)
        if not bag_file and bag_dir:
            bag_file = os.path.join(bag_dir, name, f"{name}.output.bag")
        raw_window = item.get('window')
        has_window = bool(raw_window) or any(
            key in item for key in ('start_sec', 'end_sec', 'start_time', 'end_time'))
        window = raw_window or {}
        if not isinstance(window, dict):
            raise ValueError(f"scene[{index}] window must be an object")
        if has_window and boundary_mode != 'window':
            raise ValueError("explicit scene windows require boundary_mode='window'")
        if not has_window and boundary_mode == 'window':
            raise ValueError("boundary_mode='window' requires a window for every scene")
        start_value = window.get(
            'start_sec', item.get('start_sec', item.get('start_time', 0.0)))
        end_value = window.get(
            'end_sec', item.get('end_sec', item.get('end_time')))
        try:
            start_sec = float(start_value)
            end_sec = None if end_value is None else float(end_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"scene[{index}] has invalid time window") from exc
        if start_sec < 0 or (end_sec is not None and end_sec <= start_sec):
            raise ValueError(f"scene[{index}] must satisfy 0 <= start < end")
        if not os.path.isfile(config_path):
            raise ValueError(f"scene[{index}] config not found: {config_path}")
        if scenario_path and not os.path.isfile(scenario_path):
            raise ValueError(f"scene[{index}] scenario not found: {scenario_path}")
        if map_file and not os.path.isfile(map_file):
            raise ValueError(f"scene[{index}] map_file not found: {map_file}")
        if vehicle_param_file and not os.path.isfile(vehicle_param_file):
            raise ValueError(
                f"scene[{index}] vehicle_param_file not found: {vehicle_param_file}")
        if bag_file and not os.path.isfile(bag_file):
            raise ValueError(f"scene[{index}] bag_file not found: {bag_file}")
        scenes.append(SceneSpec(
            name=name, config_path=config_path, scenario_path=scenario_path,
            map_file=map_file, vehicle_param_file=vehicle_param_file,
            bag_file=bag_file, start_sec=start_sec, end_sec=end_sec,
            has_window=has_window, boundary_mode=boundary_mode, scope=scope,
            scope_ids=scope_ids, scope_forward_distance=scope_forward_distance,
            scope_past_distance=scope_past_distance))

    names = [scene.name for scene in scenes]
    if len(names) != len(set(names)):
        raise ValueError("scene names must be unique")
    if boundary_mode == 'window':
        scenes.sort(key=lambda scene: scene.start_sec)
        for previous, current in zip(scenes, scenes[1:]):
            if previous.end_sec is None or current.start_sec < previous.end_sec:
                raise ValueError("scene time windows must be ordered and non-overlapping")
    return scenes


def build_scene_engine(scene: SceneSpec) -> 'GradingEngine':
    """为一个场景创建全新的配置、地图接口和评分引擎。"""
    config = parse_config_file(scene.config_path)
    if scene.scenario_path:
        _fill_endpoint_from_scenario(config, scene.scenario_path)
    _setup_map_interface(scene.map_file, scene.scenario_path)
    _load_vehicle_param(scene.vehicle_param_file)
    return GradingEngine(config)


class ConditionAdapter:
    """将 ConditionConfig 适配为各 handler 期望的接口"""

    def __init__(self, config: ConditionConfig):
        self._config = config

    @property
    def acceleration_condition(self):
        return self._config

    @property
    def speed_condition(self):
        return self._config

    @property
    def on_road_condition(self):
        return self._config

    @property
    def obstacle_bypass_condition(self):
        return self._config

    @property
    def ttc_condition(self):
        return self._config

    @property
    def centripetal_acceleration_condition(self):
        return self._config

    @property
    def time_limit_condition(self):
        return self._config

    @property
    def region_overlap_lw_condition(self):
        return self._config

    @property
    def object_overlap_condition(self):
        return self._config

    @property
    def red_light_stop_condition(self):
        return self._config

    @property
    def crosswalk_stop_condition(self):
        return self._config

    @property
    def stop_sign_stop_condition(self):
        return self._config


# ============================================================
# 评分引擎
# ============================================================

# handler 映射
HANDLER_MAP = {
    'speed_condition': SpeedConditionHandler,
    'acceleration_condition': AccelerationConditionHandler,
    'on_road_condition': OnRoadConditionHandler,
    'obstacle_bypass_condition': ObstacleBypassConditionHandler,
    'ttc_condition': TtcConditionHandler,
    'centripetal_acceleration_condition': CentripetalAccelerationConditionHandler,
    'time_limit_condition': TimeLimitConditionHandler,
    'region_overlap_lw_condition': RegionOverlapLwConditionHandler,
    'object_overlap_condition': ObjectOverlapConditionHandler,
    'red_light_stop_condition': RedLightStopConditionHandler,
    'crosswalk_stop_condition': CrosswalkStopConditionHandler,
    'stop_sign_stop_condition': StopSignStopConditionHandler,
}


class MetricEvaluator:
    """单个 Metric 的评估器"""

    def __init__(self, metric_config: MetricConfig):
        self.config = metric_config
        self.handler = HANDLER_MAP[metric_config.condition.condition_type]()
        self.condition_adapter = ConditionAdapter(metric_config.condition)
        self.all_passed = True
        self.metric_score = 100.0
        self.frame_results: List[DetailedResult] = []
        # object_overlap 返回 True=碰撞，需要反转为 metric fail
        self._invert = (metric_config.condition.condition_type == 'object_overlap_condition')

    def evaluate_frame(self, world_list: deque, future_list: deque) -> DetailedResult:
        """评估一帧"""
        result = DetailedResult()
        result.timestamp = world_list[-1].timestamp_sec

        handler_result = self.handler.evaluate(
            self.condition_adapter, world_list, future_list, result)

        # object_overlap: True=碰撞=metric不通过, 需要反转
        if self._invert:
            passed = not handler_result
        else:
            passed = handler_result

        result.is_pass = passed
        if not passed:
            self.all_passed = False
            if self.config.get_deduction_score:
                # 扣分制: metric_score -= delta_score
                if self.config.condition.use_score and result.delta_score > 0:
                    self.metric_score = max(0, self.metric_score - result.delta_score)
            else:
                # 得分制: delta_score 就是当前得分，取最新值
                pass

        if not self.config.get_deduction_score:
            # 得分制: metric_score = 最新的 delta_score
            self.metric_score = result.delta_score

        self.frame_results.append(result)
        return result


def _fallback_failure_code(description: str) -> str:
    text = (description or "").strip()
    if not text:
        return "unknown"
    import re
    code = re.sub(r"[^A-Za-z0-9_]+", "_", text.split(".", 1)[0].lower())
    return code.strip("_")[:80] or "unknown"


def _build_failure_events(frame_results: List[DetailedResult],
                          merge_gap_sec: float = 0.25) -> List[dict]:
    """把连续失败帧合并为一次违规事件。"""
    events = []
    for result in frame_results:
        if result.is_pass:
            continue
        code = result.failure_code or _fallback_failure_code(result.description)
        timestamp = float(result.timestamp)
        if (events and events[-1]['code'] == code and
                timestamp - events[-1]['end'] <= merge_gap_sec):
            event = events[-1]
            event['end'] = timestamp
            event['frames'] += 1
            continue
        events.append({
            'code': code,
            'start': timestamp,
            'end': timestamp,
            'frames': 1,
            'description': result.description.strip(),
        })
    return events


class GradingEngine:
    """评分引擎 - 管理所有 Metric 的评估"""

    def __init__(self, config: GradingConfig):
        self.config = config
        self.evaluators: Dict[str, MetricEvaluator] = {}
        self.world_history: deque = deque(maxlen=100)

        for metric in config.metrics:
            if metric.condition.condition_type in HANDLER_MAP:
                self.evaluators[metric.name] = MetricEvaluator(metric)

        print(f"[GradingEngine] Loaded {len(self.evaluators)} metrics: "
              f"{list(self.evaluators.keys())}")

    def feed_frame(self, world: GradingWorld):
        """输入一帧数据进行评估"""
        self.world_history.append(world)
        results = {}
        for name, evaluator in self.evaluators.items():
            result = evaluator.evaluate_frame(self.world_history, deque())
            results[name] = result
        return results

    def get_summary(self) -> dict:
        """获取评分汇总"""
        summary = {
            'is_scenario_pass': True,
            'metrics': {},
        }
        for name, evaluator in self.evaluators.items():
            if evaluator.config.require_all_time_pass:
                # 需要所有帧都通过
                is_pass = evaluator.all_passed
            else:
                # 只要有一帧通过即可 (如 ReachEnd)
                is_pass = any(r.is_pass for r in evaluator.frame_results)

            if evaluator.config.is_critical and not is_pass:
                summary['is_scenario_pass'] = False

            fail_results = [r for r in evaluator.frame_results if not r.is_pass]
            reason_counts = Counter(
                r.failure_code or _fallback_failure_code(r.description)
                for r in fail_results)
            failure_events = _build_failure_events(evaluator.frame_results)
            first_failure = None
            for r in fail_results:
                first_failure = {
                    'time': r.timestamp,
                    'code': r.failure_code or _fallback_failure_code(r.description),
                    'description': r.description.strip(),
                }
                break

            summary['metrics'][name] = {
                'is_pass': is_pass,
                'score': evaluator.metric_score,
                'fail_count': len(fail_results),
                'total_frames': len(evaluator.frame_results),
                'first_failure': first_failure,
                'failure_reasons': [
                    {'code': code, 'count': count}
                    for code, count in reason_counts.most_common()
                ],
                'failure_events': failure_events,
            }
        return summary


# ============================================================
# 模式1: 读取 Bag 文件
# ============================================================

def run_bag_mode(engine: GradingEngine, bag_file: str, interval: float = 0.1,
                 vehicle_param_file: str = ''):
    """
    从 bag/record 文件读取数据并评分

    依赖 Apollo Cyber Python API:
        from cyber.python.cyber_py3 import record
    """
    print(f"[Bag Mode] Reading: {bag_file}")
    print(f"[Bag Mode] Frame interval: {interval}s")

    try:
        from cyber.python.cyber_py3 import record
        from modules.common_msgs.chassis_msgs import chassis_pb2
        from modules.common_msgs.localization_msgs import localization_pb2
        from modules.common_msgs.perception_msgs import perception_obstacle_pb2
        from modules.common_msgs.perception_msgs import traffic_light_detection_pb2
    except ImportError as e:
        print(f"[ERROR] Apollo Cyber Python modules not available: {e}")
        print("  Please run in Apollo environment or set PYTHONPATH to include:")
        print("    /opt/apollo/neo/python/")
        print("")
        print("  Alternatively, use --mode demo for a demonstration.")
        sys.exit(1)

    reader = record.RecordReader(bag_file)
    channels = reader.get_channellist()
    print(f"[Bag Mode] Found {len(channels)} channels in bag")
    for ch in channels:
        count = reader.get_messagenumber(ch)
        print(f"  {ch} ({count} msgs)")

    # 通道名
    chassis_topic = '/apollo/canbus/chassis'
    localization_topic = '/apollo/localization/pose'
    perception_topic = '/apollo/perception/obstacles'
    traffic_light_topic = '/apollo/perception/traffic_light'

    # 状态
    last_chassis = None
    last_localization = None
    last_perception = None
    last_traffic_light = None
    last_speed = 0.0
    last_chassis_ts = 0.0
    begin_time_ns = None
    next_frame_ns = 0
    interval_ns = int(interval * 1e9)
    frame_count = 0

    print(f"\n[Bag Mode] Processing frames...")

    for channel_name, msg_data, datatype, timestamp_ns in reader.read_messages():
        if begin_time_ns is None:
            begin_time_ns = timestamp_ns

        # 更新各通道最新消息
        if channel_name == chassis_topic:
            chassis = chassis_pb2.Chassis()
            chassis.ParseFromString(msg_data)
            last_chassis = chassis

        elif channel_name == localization_topic:
            loc = localization_pb2.LocalizationEstimate()
            loc.ParseFromString(msg_data)
            last_localization = loc

        elif channel_name == perception_topic:
            percep = perception_obstacle_pb2.PerceptionObstacles()
            percep.ParseFromString(msg_data)
            last_perception = percep

        elif channel_name == traffic_light_topic:
            tl = traffic_light_detection_pb2.TrafficLightDetection()
            tl.ParseFromString(msg_data)
            last_traffic_light = tl

        # 按 interval 发射帧
        elapsed_ns = timestamp_ns - begin_time_ns
        if elapsed_ns >= next_frame_ns:
            elapsed_sec = elapsed_ns / 1e9
            world = _build_world_from_messages(
                elapsed_sec, last_chassis, last_localization, last_perception,
                last_traffic_light, last_speed, last_chassis_ts,
                vehicle_param_file
            )
            if last_chassis:
                last_speed = last_chassis.speed_mps
                last_chassis_ts = elapsed_sec

            results = engine.feed_frame(world)
            frame_count += 1
            next_frame_ns += interval_ns

            # 打印不通过的帧
            for name, r in results.items():
                if not r.is_pass:
                    print(f"  [FAIL] t={elapsed_sec:.2f}s {name}: {r.description[:80]}")

    # 标记最后一帧
    if engine.world_history:
        last_world = GradingWorld()
        last_world.timestamp_sec = engine.world_history[-1].timestamp_sec
        last_world.auto_driving_car = engine.world_history[-1].auto_driving_car
        last_world.object = engine.world_history[-1].object
        last_world.speed_limit = engine.world_history[-1].speed_limit
        last_world.last_frame = True
        engine.feed_frame(last_world)

    print(f"\n[Bag Mode] Processed {frame_count} frames")


def _head_position(adc) -> 'Vec2d':
    from grading_system_py.common.math_utils import Vec2d
    heading_vec = Vec2d(math.cos(adc.heading), math.sin(adc.heading))
    return Vec2d(
        adc.position_x + getattr(adc, 'front_edge_to_center', adc.length / 2.0) * heading_vec.x,
        adc.position_y + getattr(adc, 'front_edge_to_center', adc.length / 2.0) * heading_vec.y,
    )


def _signed_distance_to_segment(head_position, heading_vec, points) -> Optional[float]:
    from grading_system_py.common.math_utils import LineSegment2d, Vec2d
    if len(points) < 2:
        return None
    segment = LineSegment2d(Vec2d(points[0][0], points[0][1]),
                            Vec2d(points[-1][0], points[-1][1]))
    _, closest = segment.distance_to_point(head_position)
    return (closest - head_position).inner_prod(heading_vec)


def _within_event_distance(signed_dist: Optional[float],
                           forward_distance: float,
                           past_distance: float) -> bool:
    return signed_dist is not None and -past_distance <= signed_dist <= forward_distance


def _scene_scope_active(scope: str, world: GradingWorld,
                        scope_ids: List[str] = None,
                        forward_distance: float = 80.0,
                        past_distance: float = 8.0) -> bool:
    """判断当前帧是否属于场景事件范围。"""
    scope_id_set = set(scope_ids or [])
    adc = world.auto_driving_car
    heading_vec = None
    head_position = None
    if scope == 'full':
        return True
    if scope == 'red_light':
        red_signal_ids = set()
        for signal in getattr(world, 'perceived_signal', []):
            signal_id = getattr(signal, 'id', '')
            if not signal_id or (scope_id_set and signal_id not in scope_id_set):
                continue
            if getattr(signal, 'current_signal', '') == 'RED':
                red_signal_ids.add(signal_id)
        if not red_signal_ids:
            return False
        from grading_system_py.common.math_utils import Vec2d
        from grading_system_py.common.map_interface import get_map_interface
        heading_vec = Vec2d(math.cos(adc.heading), math.sin(adc.heading))
        head_position = _head_position(adc)
        for signal in get_map_interface().get_forward_nearest_signals(
                adc.position_x, adc.position_y, forward_distance + past_distance):
            signal_id = getattr(signal, 'signal_id', '')
            if signal_id not in red_signal_ids:
                continue
            for stop_line_pts in getattr(signal, 'stop_line_points', []):
                signed_dist = _signed_distance_to_segment(
                    head_position, heading_vec, stop_line_pts)
                if _within_event_distance(signed_dist, forward_distance, past_distance):
                    return True
        return False
    if scope == 'crosswalk':
        from grading_system_py.common.map_interface import get_map_interface
        from grading_system_py.condition.util import ConditionEvaluatorUtil

        for obj in getattr(world, 'object', []):
            obj_type = getattr(obj, 'type', '')
            if obj_type not in ('PEDESTRIAN', 'BICYCLE', 3, 4):
                continue
            polygon = ConditionEvaluatorUtil.get_object_polygon(obj, False)
            crosswalk = get_map_interface().find_crosswalk_for_polygon(
                obj.position_x, obj.position_y, 50, polygon)
            if not crosswalk or (scope_id_set and crosswalk.crosswalk_id not in scope_id_set):
                continue
            if heading_vec is None:
                from grading_system_py.common.math_utils import Vec2d
                heading_vec = Vec2d(math.cos(adc.heading), math.sin(adc.heading))
                head_position = _head_position(adc)
            _, closest = crosswalk.polygon.distance_to_point(head_position)
            signed_dist = (closest - head_position).inner_prod(heading_vec)
            if _within_event_distance(signed_dist, forward_distance, past_distance):
                return True
    if scope == 'stop_sign':
        from grading_system_py.common.math_utils import Vec2d
        from grading_system_py.common.map_interface import get_map_interface
        heading_vec = Vec2d(math.cos(adc.heading), math.sin(adc.heading))
        head_position = _head_position(adc)
        for stop_sign in get_map_interface().get_forward_nearest_stop_signs(
                adc.position_x, adc.position_y, forward_distance + past_distance):
            ss_id = getattr(stop_sign, 'stop_sign_id', '')
            if scope_id_set and ss_id not in scope_id_set:
                continue
            for stop_line_pts in getattr(stop_sign, 'stop_line_points', []):
                signed_dist = _signed_distance_to_segment(
                    head_position, heading_vec, stop_line_pts)
                if _within_event_distance(signed_dist, forward_distance, past_distance):
                    return True
    return False


class SceneSession:
    """单个场景的评分会话，所有可变状态都限制在场景内。"""

    def __init__(self, spec: SceneSpec, interval: float):
        self.spec = spec
        self.interval = interval
        self.engine = build_scene_engine(spec)
        self.last_chassis = None
        self.last_localization = None
        self.last_perception = None
        self.last_traffic_light = None
        self.last_speed = 0.0
        self.last_chassis_ts = 0.0
        self.scene_start_sec = spec.start_sec
        self.scene_end_sec = spec.end_sec
        self.next_frame_sec = spec.start_sec
        self.frame_count = 0
        self.finished = False
        self.active = False
        self.auto_active = False
        self.scope_seen = spec.scope == 'full'
        self.map_loaded = True
        self.map_dependent_scope = spec.scope in ('crosswalk', 'red_light', 'stop_sign')

    def begin_auto(self, elapsed_sec: float):
        """开始一个新的 AUTO 场景，清空上一场景的所有消息和时间基准。"""
        self.last_chassis = None
        self.last_localization = None
        self.last_perception = None
        self.last_traffic_light = None
        self.last_speed = 0.0
        self.last_chassis_ts = 0.0
        self.scene_start_sec = elapsed_sec
        self.scene_end_sec = None
        self.next_frame_sec = elapsed_sec
        self.frame_count = 0
        self.finished = False
        self.active = False
        self.auto_active = True
        self.scope_seen = self.spec.scope == 'full'

    def activate(self):
        if not self.active:
            self.map_loaded = _setup_map_interface(
                self.spec.map_file, self.spec.scenario_path)
            self.active = True

    def accepts(self, elapsed_sec: float) -> bool:
        if self.spec.boundary_mode == 'auto':
            return self.auto_active
        if self.spec.boundary_mode == 'full':
            return True
        return (elapsed_sec >= self.spec.start_sec and
                (self.spec.end_sec is None or elapsed_sec < self.spec.end_sec))

    def consume(self, channel_name: str, message: Any, elapsed_sec: float):
        if self.finished:
            return
        if channel_name == '/apollo/canbus/chassis':
            self.last_chassis = message
        elif channel_name == '/apollo/localization/pose':
            self.last_localization = message
        elif channel_name == '/apollo/perception/obstacles':
            self.last_perception = message
        elif channel_name == '/apollo/perception/traffic_light':
            self.last_traffic_light = message
        else:
            return

        if not self.accepts(elapsed_sec):
            return
        self.activate()
        if elapsed_sec < self.next_frame_sec:
            return

        local_time = elapsed_sec - self.scene_start_sec
        world = _build_world_from_messages(
            local_time, self.last_chassis, self.last_localization,
            self.last_perception, self.last_traffic_light,
            self.last_speed, self.last_chassis_ts,
            self.spec.vehicle_param_file)
        if self.last_chassis:
            self.last_speed = self.last_chassis.speed_mps
            self.last_chassis_ts = local_time

        self.next_frame_sec = elapsed_sec + self.interval
        scope_active = _scene_scope_active(
            self.spec.scope, world, self.spec.scope_ids,
            self.spec.scope_forward_distance,
            self.spec.scope_past_distance)
        if not scope_active:
            return
        self.scope_seen = True
        results = self.engine.feed_frame(world)
        self.frame_count += 1
        for name, result in results.items():
            if not result.is_pass:
                print(f"  [FAIL] [{self.spec.name}] t={local_time:.2f}s "
                      f"{name}: {result.description[:80]}")

    def finish(self):
        if self.finished:
            return
        if self.engine.world_history:
            last_world = copy.deepcopy(self.engine.world_history[-1])
            last_world.last_frame = True
            self.engine.feed_frame(last_world)
            self.scene_end_sec = self.scene_start_sec + last_world.timestamp_sec
        self.auto_active = False
        self.finished = True

    def summary(self) -> dict:
        result = self.engine.get_summary()
        # scoped 场景依赖地图取信号灯/人行道几何；地图没加载会导致 scope 永不激活、
        # frame_count==0。这类"漏评"必须显式判 FAIL，而不是静默 NO_DATA 被跳过。
        if (self.frame_count == 0 and self.map_dependent_scope
                and not self.map_loaded):
            status = 'map_unavailable'
        elif self.frame_count:
            status = 'completed'
        else:
            status = 'no_data'
        result.update({
            'name': self.spec.name,
            'status': status,
            'window': {
                'start_sec': self.scene_start_sec,
                'end_sec': self.scene_end_sec,
            },
            'scope': self.spec.scope,
            'sampled_frames': self.frame_count,
        })
        if status == 'map_unavailable':
            result['is_scenario_pass'] = False
        return result


def _scene_deduction_score(summary: dict) -> float:
    """场景扣分制总分：满分 100 减去本场景所有指标扣掉的分之和，clamp 到 [0,100]。

    每个指标 metric_score 已经是 100 起扣后的当前分，(100 - metric_score) 即该指标扣分。
    """
    metrics = summary.get('metrics', {})
    if not metrics:
        return 100.0
    total_deduction = sum(max(0.0, 100.0 - float(info.get('score', 100.0)))
                          for info in metrics.values())
    return max(0.0, 100.0 - total_deduction)


def _print_summary(summary: dict, title: str = 'GRADING SUMMARY'):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)
    if 'name' in summary:
        window = summary.get('window', {})
        print(f"\n  Scene: {summary['name']} "
              f"[{window.get('start_sec')}, {window.get('end_sec')}]\n")
    status = 'PASS' if summary.get('is_scenario_pass') else 'FAIL'
    if summary.get('status') == 'no_data':
        status = 'NO_DATA'
    elif summary.get('status') == 'map_unavailable':
        status = 'FAIL'
    print(f"  Scenario: {status}")
    if summary.get('status') == 'map_unavailable':
        print("    Detail: map_unavailable — 地图未加载，该场景无法评测，判失败")
    if summary.get('status') not in ('no_data', 'map_unavailable'):
        print(f"  Scene Score: {_scene_deduction_score(summary):.1f}")
    else:
        print("  Scene Score: 0.0")
    print()
    print(f"  {'Metric':<25} {'Result':<8} {'Score':<8} {'Fail/Total'}")
    print(f"  {'-'*25} {'-'*8} {'-'*8} {'-'*12}")
    for name, info in summary.get('metrics', {}).items():
        metric_status = 'PASS' if info['is_pass'] else 'FAIL'
        print(f"  {name:<25} {metric_status:<8} "
              f"{info['score']:.0f}       "
              f"{info['fail_count']}/{info['total_frames']}")
        first_failure = info.get('first_failure')
        if first_failure:
            print(f"    Detail: first_fail t={first_failure['time']:.2f}s "
                  f"code={first_failure['code']} "
                  f"{first_failure['description'][:160]}")
        reasons = info.get('failure_reasons') or []
        if reasons:
            reason_text = ", ".join(
                f"{item['code']}={item['count']}" for item in reasons[:5])
            print(f"    Reasons: {reason_text}")
        events = info.get('failure_events') or []
        if events:
            event_text = "; ".join(
                f"{event['code']}[{event['start']:.2f}-{event['end']:.2f}s]"
                for event in events[:6])
            print(f"    Events: {event_text}")


def _print_multi_total(results: List[dict]):
    """打印多场景扣分制总分：只统计有数据的场景，取其场景分的平均。"""
    scored = [_scene_deduction_score(item) for item in results
              if item.get('status') not in ('no_data', 'map_unavailable')]
    unavailable = sum(1 for item in results
                      if item.get('status') == 'map_unavailable')
    if scored or unavailable:
        # map_unavailable 场景计 0 分纳入平均，避免漏评被当作满分。
        denom = len(scored) + unavailable
        total = round(sum(scored) / denom, 1) if denom else 0.0
        print(f"  Total Score: {total} | Scored Scenes: {denom}")
    else:
        print("  Total Score: NO_DATA")


def _run_independent_scene_bags(scene_specs: List[SceneSpec],
                                 interval: float = 0.1) -> List[dict]:
    """每个场景读取自己的 bag，适配 Apollo 默认输出目录结构。"""
    results = []
    for spec in scene_specs:
        print(f"\n[Multi-Scene] Evaluating {spec.name}: {spec.bag_file}")
        engine = build_scene_engine(spec)
        run_bag_mode(engine, spec.bag_file, interval, spec.vehicle_param_file)
        summary = engine.get_summary()
        summary.update({
            'name': spec.name,
            'status': 'completed' if engine.world_history else 'no_data',
            'window': {'start_sec': 0.0, 'end_sec': None},
        })
        results.append(summary)

    for summary in results:
        _print_summary(summary, 'SCENE GRADING SUMMARY')
    passed = sum(1 for item in results if item['status'] == 'completed' and
                 item['is_scenario_pass'])
    print("\n" + "=" * 60)
    print("  MULTI-SCENE SUMMARY")
    print("=" * 60)
    print(f"  Scenes: {len(results)} | Passed: {passed} | "
          f"Failed/NoData: {len(results) - passed}")
    _print_multi_total(results)
    return results


def run_multi_scene_bag_mode(scene_specs: List[SceneSpec], bag_file: str = '',
                              interval: float = 0.1) -> List[dict]:
    """按独立 bag、AUTO 周期或显式时间窗口评估多个场景。"""
    independent_bags = [scene.bag_file for scene in scene_specs if scene.bag_file]
    if independent_bags:
        if len(independent_bags) != len(scene_specs):
            raise ValueError("all scenes must define bag_file when any scene does")
        return _run_independent_scene_bags(scene_specs, interval)

    print(f"[Multi-Scene Bag Mode] Reading: {bag_file}")
    print(f"[Multi-Scene Bag Mode] Scenes: {len(scene_specs)}")
    try:
        from cyber.python.cyber_py3 import record
        from modules.common_msgs.chassis_msgs import chassis_pb2
        from modules.common_msgs.localization_msgs import localization_pb2
        from modules.common_msgs.perception_msgs import perception_obstacle_pb2
        from modules.common_msgs.perception_msgs import traffic_light_detection_pb2
    except ImportError as exc:
        raise RuntimeError(f"Apollo Cyber Python modules not available: {exc}")

    reader = record.RecordReader(bag_file)
    topics = {
        '/apollo/canbus/chassis': chassis_pb2.Chassis,
        '/apollo/localization/pose': localization_pb2.LocalizationEstimate,
        '/apollo/perception/obstacles': perception_obstacle_pb2.PerceptionObstacles,
        '/apollo/perception/traffic_light': traffic_light_detection_pb2.TrafficLightDetection,
    }
    if scene_specs[0].boundary_mode == 'full':
        return _run_full_scene_bag(scene_specs, reader, topics, interval)
    if scene_specs[0].boundary_mode == 'auto':
        return _run_auto_scene_bag(
            scene_specs, reader, topics,
            chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE, interval)

    sessions = [SceneSession(spec, interval) for spec in scene_specs]
    begin_time_ns = None
    active_index = 0

    for channel_name, msg_data, datatype, timestamp_ns in reader.read_messages():
        if begin_time_ns is None:
            begin_time_ns = timestamp_ns
        elapsed_sec = (timestamp_ns - begin_time_ns) / 1e9

        while active_index < len(sessions):
            end_sec = sessions[active_index].spec.end_sec
            if end_sec is None or elapsed_sec < end_sec:
                break
            sessions[active_index].finish()
            active_index += 1
        if active_index >= len(sessions):
            break

        session = sessions[active_index]
        if not session.accepts(elapsed_sec):
            continue
        message_type = topics.get(channel_name)
        if message_type is None:
            continue
        message = message_type()
        message.ParseFromString(msg_data)
        session.consume(channel_name, message, elapsed_sec)

    for session in sessions:
        session.finish()
    results = [session.summary() for session in sessions]
    for summary in results:
        _print_summary(summary, 'SCENE GRADING SUMMARY')
    passed = sum(1 for item in results if item['status'] == 'completed' and
                 item['is_scenario_pass'])
    failed = len(results) - passed
    print("\n" + "=" * 60)
    print("  MULTI-SCENE SUMMARY")
    print("=" * 60)
    print(f"  Scenes: {len(results)} | Passed: {passed} | Failed/NoData: {failed}")
    _print_multi_total(results)
    return results


def _run_full_scene_bag(scene_specs, reader, topics, interval):
    """对每个场景独立扫描完整 bag，适合连续路测且没有场景边界的录包。"""
    sessions = [SceneSession(spec, interval) for spec in scene_specs]
    begin_time_ns = None

    for channel_name, msg_data, datatype, timestamp_ns in reader.read_messages():
        if begin_time_ns is None:
            begin_time_ns = timestamp_ns
        elapsed_sec = (timestamp_ns - begin_time_ns) / 1e9
        message_type = topics.get(channel_name)
        if message_type is None:
            continue
        message = message_type()
        message.ParseFromString(msg_data)
        for session in sessions:
            session.consume(channel_name, message, elapsed_sec)

    for session in sessions:
        session.finish()
    results = [session.summary() for session in sessions]
    for summary in results:
        _print_summary(summary, 'SCENE GRADING SUMMARY')
    passed = sum(1 for item in results if item['status'] == 'completed' and
                 item['is_scenario_pass'])
    failed = len(results) - passed
    print("\n" + "=" * 60)
    print("  MULTI-SCENE SUMMARY")
    print("=" * 60)
    print(f"  Scenes: {len(results)} | Passed: {passed} | Failed/NoData: {failed}")
    _print_multi_total(results)
    return results


def _run_auto_scene_bag(scene_specs, reader, topics, auto_mode, interval):
    """按 record 时间 epoch 或底盘 AUTO 周期顺序绑定场景。"""
    sessions = [SceneSession(spec, interval) for spec in scene_specs]
    begin_time_ns = None
    previous_timestamp_ns = None
    reset_threshold_ns = int(float(os.environ.get(
        'GRADING_RECORD_RESET_THRESHOLD_SEC', '0.5')) * 1e9)
    last_auto = False
    active_index = None
    next_scene_index = 0

    for channel_name, msg_data, datatype, timestamp_ns in reader.read_messages():
        if (previous_timestamp_ns is not None and
                previous_timestamp_ns - timestamp_ns > reset_threshold_ns):
            if active_index is not None:
                sessions[active_index].finish()
                print(f"[Multi-Scene] Record timestamp reset ended scene "
                      f"{sessions[active_index].spec.name}")
                active_index = None
            begin_time_ns = timestamp_ns
            last_auto = False
            print("[Multi-Scene] Detected record timestamp reset; "
                  "starting a new scene epoch.")
        elif begin_time_ns is None:
            begin_time_ns = timestamp_ns
        previous_timestamp_ns = timestamp_ns
        elapsed_sec = (timestamp_ns - begin_time_ns) / 1e9
        message_type = topics.get(channel_name)
        if message_type is None:
            continue
        message = message_type()
        message.ParseFromString(msg_data)

        if channel_name == '/apollo/canbus/chassis':
            is_auto = message.driving_mode == auto_mode
            if is_auto and not last_auto:
                if next_scene_index >= len(sessions):
                    print("[Multi-Scene] Ignoring AUTO cycle: no scene config remains.")
                    active_index = None
                else:
                    active_index = next_scene_index
                    sessions[active_index].begin_auto(elapsed_sec)
                    next_scene_index += 1
                    print(f"[Multi-Scene] AUTO started scene "
                          f"{sessions[active_index].spec.name} at {elapsed_sec:.2f}s")
            elif not is_auto and last_auto:
                if active_index is not None:
                    sessions[active_index].finish()
                    print(f"[Multi-Scene] AUTO ended scene "
                          f"{sessions[active_index].spec.name} at {elapsed_sec:.2f}s")
                    active_index = None
            last_auto = is_auto

        if active_index is not None:
            sessions[active_index].consume(channel_name, message, elapsed_sec)

    if active_index is not None:
        sessions[active_index].finish()

    if len(scene_specs) > next_scene_index:
        missing = ', '.join(
            scene.spec.name for scene in sessions[next_scene_index:])
        print(f"[WARN] No AUTO cycle was found for scene(s): {missing}")
        print("[WARN] To separate scenes automatically, the simulator must "
              "leave AUTO between scenes.")

    results = [session.summary() for session in sessions]
    for summary in results:
        _print_summary(summary, 'SCENE GRADING SUMMARY')
    passed = sum(1 for item in results if item['status'] == 'completed' and
                 item['is_scenario_pass'])
    failed = len(results) - passed
    print("\n" + "=" * 60)
    print("  MULTI-SCENE SUMMARY")
    print("=" * 60)
    print(f"  Scenes: {len(results)} | Passed: {passed} | Failed/NoData: {failed}")
    _print_multi_total(results)
    return results


def _build_world_from_messages(timestamp, chassis, localization, perception,
                                traffic_light, last_speed, last_chassis_time,
                                vehicle_param_file: str = '') -> GradingWorld:
    """从各通道消息构建 GradingWorld"""
    world = GradingWorld()
    world.timestamp_sec = timestamp

    adc = world.auto_driving_car
    _apply_vehicle_param(adc, vehicle_param_file)

    if chassis:
        speed_mps = chassis.speed_mps
        adc.speed = speed_mps
        # 计算加速度
        if last_chassis_time > 0 and timestamp > last_chassis_time:
            dt = timestamp - last_chassis_time
            if dt > 0:
                adc.speed_acceleration = (speed_mps - last_speed) / dt

    if localization:
        pose = localization.pose
        adc.position_x = pose.position.x
        adc.position_y = pose.position.y
        adc.heading = pose.heading
        if pose.HasField('linear_velocity'):
            adc.speed_heading = math.atan2(
                pose.linear_velocity.y, pose.linear_velocity.x)
        if pose.HasField('angular_velocity'):
            adc.spin = pose.angular_velocity.z

    if perception:
        for obs in perception.perception_obstacle:
            obj = Object()
            obj.id = str(obs.id)
            obj.position_x = obs.position.x
            obj.position_y = obs.position.y
            obj.heading = obs.theta
            obj.length = obs.length
            obj.width = obs.width
            obj.height = obs.height
            obj.speed = math.hypot(obs.velocity.x, obs.velocity.y)
            obj.speed_heading = math.atan2(obs.velocity.y, obs.velocity.x)
            obj.front_edge_to_center = obs.length / 2
            obj.back_edge_to_center = obs.length / 2
            if getattr(obs, 'polygon_point', None):
                obj.polygon_point = list(obs.polygon_point)
            # 兼容 Apollo PerceptionObstacle 类型枚举
            try:
                obj.type = obs.Type.Name(obs.type)
            except Exception:
                obj.type = obs.type
            world.object.append(obj)

    if traffic_light:
        for tl in traffic_light.traffic_light:
            signal = Object()
            signal.id = tl.id
            try:
                signal.current_signal = tl.Color.Name(tl.color)
            except Exception:
                signal.current_signal = str(tl.color)
            world.perceived_signal.append(signal)

    return world


# ============================================================
# 模式2: 订阅 Cyber 通道
# ============================================================

def run_channel_mode(engine: GradingEngine, interval: float = 0.1,
                     start_on_auto: bool = True,
                     pause_when_manual: bool = True):
    """
    订阅 Cyber 通道实时评分

    依赖 Apollo Cyber Python API:
        from cyber.python.cyber_py3 import cyber
    """
    print("[Channel Mode] Starting real-time grading...")

    try:
        from cyber.python.cyber_py3 import cyber
        from modules.common_msgs.chassis_msgs import chassis_pb2
        from modules.common_msgs.localization_msgs import localization_pb2
        from modules.common_msgs.perception_msgs import perception_obstacle_pb2
        from modules.common_msgs.perception_msgs import traffic_light_detection_pb2
    except ImportError as e:
        print(f"[ERROR] Apollo Cyber Python modules not available: {e}")
        print("  Please run in Apollo environment.")
        sys.exit(1)

    cyber.init("grading_system_py")
    node = cyber.Node("grading_evaluator")

    # 共享状态
    state = {
        'chassis': None,
        'localization': None,
        'perception': None,
        'traffic_light': None,
        'last_speed': 0.0,
        'last_time': 0.0,
        'begin_time': None,
        'auto_started': not start_on_auto,
        'last_auto': False,
    }

    def _is_auto(chassis_msg) -> bool:
        if chassis_msg is None:
            return False
        return chassis_msg.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE

    def on_chassis(msg):
        state['chassis'] = msg

    def on_localization(msg):
        state['localization'] = msg

    def on_perception(msg):
        state['perception'] = msg

    def on_traffic_light(msg):
        state['traffic_light'] = msg

    # 订阅通道
    node.create_reader("/apollo/canbus/chassis",
                       chassis_pb2.Chassis, on_chassis)
    node.create_reader("/apollo/localization/pose",
                       localization_pb2.LocalizationEstimate, on_localization)
    node.create_reader("/apollo/perception/obstacles",
                       perception_obstacle_pb2.PerceptionObstacles, on_perception)
    node.create_reader("/apollo/perception/traffic_light",
                       traffic_light_detection_pb2.TrafficLightDetection, on_traffic_light)

    print("[Channel Mode] Subscribed to channels. Waiting for data...")
    print("  Press Ctrl+C to stop and see results.\n")

    frame_count = 0
    try:
        while not cyber.is_shutdown():
            time.sleep(interval)

            is_auto = _is_auto(state['chassis'])
            if is_auto and not state['last_auto']:
                print("[Channel Mode] Chassis entered COMPLETE_AUTO_DRIVE, start grading.")
                state['auto_started'] = True
                state['begin_time'] = time.time()
                state['last_speed'] = state['chassis'].speed_mps if state['chassis'] else 0.0
                state['last_time'] = 0.0
            elif (not is_auto) and state['last_auto']:
                print("[Channel Mode] Chassis left AUTO mode.")
            state['last_auto'] = is_auto

            if not state['auto_started']:
                continue
            if pause_when_manual and not is_auto:
                continue

            if state['begin_time'] is None:
                if state['localization'] is not None:
                    state['begin_time'] = time.time()
                continue

            elapsed = time.time() - state['begin_time']
            world = _build_world_from_messages(
                elapsed, state['chassis'], state['localization'],
                state['perception'], state['traffic_light'],
                state['last_speed'], state['last_time']
            )

            if state['chassis']:
                state['last_speed'] = state['chassis'].speed_mps
                state['last_time'] = elapsed

            results = engine.feed_frame(world)
            frame_count += 1

            # 实时打印不通过的结果
            for name, r in results.items():
                if not r.is_pass:
                    print(f"  [FAIL] t={elapsed:.2f}s {name}: {r.description}")

    except KeyboardInterrupt:
        print(f"\n[Channel Mode] Stopped. Processed {frame_count} frames.")

    cyber.shutdown()


# ============================================================
# Demo 模式 (无需 Apollo 环境)
# ============================================================

def run_demo_mode(engine: GradingEngine):
    """演示模式 - 使用模拟数据"""
    print("[Demo Mode] Running with simulated data...\n")

    # 模拟一段仿真数据: 车辆加速 -> 匀速 -> 急刹
    for i in range(50):
        t = i * 0.1
        if t < 2.0:
            speed = 3.0 * t          # 加速阶段
            accel = 3.0
        elif t < 3.5:
            speed = 6.0 + (t - 2.0) * 8  # 继续加速
            accel = 8.0
        else:
            speed = max(0, 18.0 - (t - 3.5) * 6)  # 急刹
            accel = -6.0

        world = GradingWorld()
        world.timestamp_sec = t
        world.last_frame = (i == 49)

        adc = world.auto_driving_car
        adc.position_x = t * speed / 2
        adc.position_y = 0
        adc.heading = 0
        adc.speed = speed
        adc.speed_acceleration = accel
        adc.speed_heading = 0
        adc.length = 4.5
        adc.width = 2.0
        adc.front_edge_to_center = 2.25
        adc.back_edge_to_center = 2.25

        # 前方有一个静止障碍物
        obs = Object()
        obs.id = "static_obs"
        obs.position_x = 60.0
        obs.position_y = 0.0
        obs.heading = 0
        obs.speed = 0
        obs.speed_heading = 0
        obs.length = 4.5
        obs.width = 2.0
        obs.front_edge_to_center = 2.25
        obs.back_edge_to_center = 2.25
        world.object.append(obs)

        world.speed_limit = 16.67  # 60 km/h

        results = engine.feed_frame(world)

        # 只打印有问题的帧
        fails = [f"{name}" for name, r in results.items() if not r.is_pass]
        if fails:
            print(f"  t={t:.1f}s | speed={speed:.1f} | accel={accel:+.1f} | "
                  f"FAIL: {', '.join(fails)}")


# ============================================================
# 主入口
# ============================================================


def _setup_map_interface(map_file: str, scenario_path: str = ''):
    """初始化地图接口；用于人行横道/红灯停车距离。"""
    import os
    from grading_system_py.common.map_interface import (
        ApolloMapInterface, StubMapInterface, set_map_interface)

    candidates = []
    if map_file:
        candidates.append(map_file)

    if scenario_path:
        try:
            import json
            with open(scenario_path, 'r', encoding='utf-8') as f:
                scenario = json.load(f)
            logic_path = scenario.get('scenario', {}).get('roadNetwork', {}).get(
                'logicFile', {}).get('filepath', '')
            if logic_path:
                candidates.extend([
                    os.path.join('/opt/apollo/neo/share', logic_path, 'base_map.bin'),
                    os.path.join('/opt/apollo/neo/share', logic_path, 'base_map.txt'),
                    os.path.join('/opt/apollo/neo/src', logic_path, 'base_map.bin'),
                    os.path.join('/opt/apollo/neo/src', logic_path, 'base_map.txt'),
                    os.path.join('/apollo_workspace', logic_path, 'base_map.bin'),
                    os.path.join('/apollo_workspace', logic_path, 'base_map.txt'),
                ])
        except Exception:
            pass

    for path in candidates:
        if path and os.path.exists(path):
            try:
                set_map_interface(ApolloMapInterface(path))
                print(f"[Config] Map loaded: {path}")
                return True
            except Exception as e:
                print(f"[WARN] Failed to load map {path}: {e}")

    if candidates:
        print("=" * 60)
        print("[ERROR] Map file NOT loaded — crosswalk/red-light scoped scenes "
              "will be UNEVALUATED (reported as FAIL: map_unavailable).")
        print("[ERROR] Tried candidates:")
        for path in candidates:
            print(f"          - {path} (exists={os.path.exists(path)})")
        print("=" * 60)
    set_map_interface(StubMapInterface())
    return False


def _fill_endpoint_from_scenario(config: GradingConfig, scenario_path: str):
    """从场景文件中读取终点坐标填充到 ReachEnd metric"""
    import json
    try:
        with open(scenario_path, 'r', encoding='utf-8') as f:
            scenario = json.load(f)
        end_info = scenario.get('scenario', {}).get('autoCarInfo', {}).get('end', {})
        end_x = end_info.get('x', 0)
        end_y = end_info.get('y', 0)
        if end_x and end_y:
            for metric in config.metrics:
                if metric.condition.condition_type == 'region_overlap_lw_condition':
                    metric.condition.x = end_x
                    metric.condition.y = end_y
                    print(f"[Config] ReachEnd endpoint set to ({end_x:.2f}, {end_y:.2f})")
    except Exception as e:
        print(f"[WARN] Failed to read scenario file: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Apollo Grading System (Python)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Demo mode (no dependencies needed):
  python3 main.py --config /apollo_workspace/grading_system/conf/grading_metrics_default.conf --mode demo

  # Read from bag file:
  python3 main.py --config /apollo_workspace/grading_system/conf/grading_metrics_default.conf \\
                  --mode bag --bag_file /apollo-simulator/output_data/1/1.output.bag

  # Subscribe to Cyber channels:
  python3 main.py --config /apollo_workspace/grading_system/conf/grading_metrics_default.conf --mode channel
        """)

    parser.add_argument('--config', default='',
                        help='Path to grading config file (.json/.conf)')
    parser.add_argument('--scene_manifest', default='',
                        help='JSON manifest for multiple scenes in one bag')
    parser.add_argument('--mode', choices=['bag', 'channel', 'demo'],
                        default='demo',
                        help='Data source mode: bag/channel/demo')
    parser.add_argument('--bag_file', default='',
                        help='Path to .bag/.record file (bag mode)')
    parser.add_argument('--scenario', default='',
                        help='Path to scenario JSON (provides end position for ReachEnd)')
    parser.add_argument('--map_file', default='',
                        help='Apollo base_map/sim_map file for crosswalk/red-light stop metrics')
    parser.add_argument('--vehicle_param_file', default='',
                        help='Apollo vehicle_param.pb.txt for ADC geometry and vehicle limits')
    parser.add_argument('--interval', type=float, default=0.1,
                        help='Frame interval in seconds (default: 0.1)')
    parser.add_argument('--start_on_auto', action='store_true', default=True,
                        help='Channel mode: start grading only after chassis enters COMPLETE_AUTO_DRIVE (default: true)')
    parser.add_argument('--no_start_on_auto', dest='start_on_auto', action='store_false',
                        help='Channel mode: start grading immediately without waiting for AUTO')
    parser.add_argument('--pause_when_manual', action='store_true', default=True,
                        help='Channel mode: pause grading when chassis leaves AUTO (default: true)')
    parser.add_argument('--no_pause_when_manual', dest='pause_when_manual', action='store_false',
                        help='Channel mode: keep grading after chassis leaves AUTO')

    args = parser.parse_args()

    if args.scene_manifest:
        if args.mode != 'bag':
            parser.error('--scene_manifest is only supported with --mode bag')
        if args.config or args.scenario or args.map_file or args.vehicle_param_file:
            parser.error('--scene_manifest cannot be combined with --config, --scenario, --map_file, or --vehicle_param_file')
        try:
            scene_specs = parse_scene_manifest(args.scene_manifest)
            if not args.bag_file and not all(scene.bag_file for scene in scene_specs):
                parser.error('--bag_file is required unless every scene defines bag_file')
            run_multi_scene_bag_mode(scene_specs, args.bag_file, args.interval)
        except (OSError, ValueError, RuntimeError) as exc:
            print(f"[ERROR] Multi-scene evaluation failed: {exc}")
            sys.exit(2)
        return

    if not args.config:
        parser.error('--config is required unless --scene_manifest is used')

    # 解析配置文件
    print(f"[Config] Loading: {args.config}")
    config = parse_config_file(args.config)
    print(f"[Config] Parsed {len(config.metrics)} supported metrics")

    if args.scenario:
        _fill_endpoint_from_scenario(config, args.scenario)
    _setup_map_interface(args.map_file, args.scenario)
    _load_vehicle_param(args.vehicle_param_file)

    engine = GradingEngine(config)
    print()

    if args.mode == 'bag':
        if not args.bag_file:
            print("[ERROR] --bag_file is required in bag mode")
            sys.exit(1)
        run_bag_mode(engine, args.bag_file, args.interval, args.vehicle_param_file)
    elif args.mode == 'channel':
        run_channel_mode(engine, args.interval,
                         start_on_auto=args.start_on_auto,
                         pause_when_manual=args.pause_when_manual)
    else:
        run_demo_mode(engine)

    _print_summary(engine.get_summary())


if __name__ == '__main__':
    main()
