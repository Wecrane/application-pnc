#!/usr/bin/env python3
"""
grading_system_py 使用示例

展示如何调用各个评分模块来评估自动驾驶车辆行为。
数据可以来自仿真器输出的帧数据（每帧包含车辆状态和障碍物信息）。
"""

import sys
import math
from collections import deque

sys.path.insert(0, '/apollo_workspace')

from grading_system_py.condition.acceleration_handler import AccelerationConditionHandler
from grading_system_py.condition.speed_handler import SpeedConditionHandler
from grading_system_py.condition.on_road_handler import OnRoadConditionHandler
from grading_system_py.condition.obstacle_bypass_handler import ObstacleBypassConditionHandler
from grading_system_py.condition.ttc_handler import TtcConditionHandler
from grading_system_py.tests.mock_data import (
    MockObject, MockGradingWorld, MockDetailedResult,
    MockAccelerationCondition, MockSpeedCondition, MockOnRoadCondition,
    MockObstacleBypassCondition, MockTtcCondition, MockCondition,
    MockPolygon, PolygonPoint
)


# ============================================================
# 1. 构造世界状态数据（模拟仿真器输出的帧数据）
# ============================================================

def build_world_from_frame(timestamp, adc_x, adc_y, adc_heading, adc_speed,
                           adc_acceleration, obstacles=None, speed_limit=None,
                           last_frame=False):
    """
    将每一帧仿真数据转换为 GradingWorld 对象

    在实际使用中，这些数据来自仿真器输出的 ROS bag 或 JSON。
    对应原 C++ 中 InstantGradingWorldService 构建 GradingWorld 的过程。
    """
    adc = MockObject(
        id="adc",
        position_x=adc_x,
        position_y=adc_y,
        heading=adc_heading,
        speed=adc_speed,
        speed_acceleration=adc_acceleration,
        speed_heading=adc_heading,
        length=4.5,
        width=2.0,
        front_edge_to_center=2.25,
        back_edge_to_center=2.25,
    )

    obs_list = []
    if obstacles:
        for obs in obstacles:
            obs_list.append(MockObject(
                id=obs['id'],
                position_x=obs['x'],
                position_y=obs['y'],
                heading=obs.get('heading', 0),
                speed=obs.get('speed', 0),
                speed_heading=obs.get('heading', 0),
                length=obs.get('length', 4.5),
                width=obs.get('width', 2.0),
                front_edge_to_center=obs.get('length', 4.5) / 2,
                back_edge_to_center=obs.get('length', 4.5) / 2,
            ))

    return MockGradingWorld(
        timestamp_sec=timestamp,
        auto_driving_car=adc,
        object=obs_list,
        speed_limit=speed_limit,
        last_frame=last_frame,
    )


# ============================================================
# 2. 使用各个评分模块
# ============================================================

def example_speed_check():
    """示例：速度检测"""
    print("=" * 60)
    print("【速度检测】SpeedConditionHandler")
    print("=" * 60)

    handler = SpeedConditionHandler()

    # 配置：最大速度 16.67 m/s (60 km/h)
    condition = MockCondition(
        speed_condition=MockSpeedCondition(
            name="speed_limit",
            min_speed=0.0,
            max_speed=16.67,
            use_score=True,
            deduction_unit=1.0,
            single_deduction=5.0,
        )
    )

    # 模拟几帧数据
    frames = [
        (0.0, 5.0),    # t=0s, speed=5 m/s
        (0.1, 10.0),   # t=0.1s, speed=10 m/s
        (0.2, 15.0),   # t=0.2s, speed=15 m/s
        (0.3, 18.0),   # t=0.3s, speed=18 m/s  <-- 超速!
        (0.4, 20.0),   # t=0.4s, speed=20 m/s  <-- 超速!
    ]

    for t, speed in frames:
        world = build_world_from_frame(
            timestamp=t, adc_x=t*speed, adc_y=0, adc_heading=0,
            adc_speed=speed, adc_acceleration=0, speed_limit=16.67
        )
        result = MockDetailedResult()
        passed = handler.evaluate(condition, deque([world]), deque(), result)
        status = "PASS" if passed else "FAIL"
        print(f"  t={t:.1f}s | speed={speed:.1f} m/s | {status}"
              + (f" | 扣分={result.delta_score}" if not passed else ""))
    print()


def example_acceleration_check():
    """示例：加速度检测"""
    print("=" * 60)
    print("【加速度检测】AccelerationConditionHandler")
    print("=" * 60)

    handler = AccelerationConditionHandler()

    # 配置：加速度范围 [-4.5, 4.0] m/s²
    condition = MockCondition(
        acceleration_condition=MockAccelerationCondition(
            name="acceleration",
            min_acceleration=-4.5,
            max_acceleration=4.0,
            use_score=True,
            deduction_unit=0.5,
            single_deduction=5.0,
        )
    )

    frames = [
        (0.0, 10.0, 2.0),    # 正常加速
        (0.1, 12.0, 3.5),    # 正常加速
        (0.2, 14.0, 5.0),    # 超过加速度上限!
        (0.3, 12.0, -3.0),   # 正常减速
        (0.4, 8.0, -5.0),    # 急刹车!
    ]

    for t, speed, accel in frames:
        world = build_world_from_frame(
            timestamp=t, adc_x=50+t*speed, adc_y=0, adc_heading=0,
            adc_speed=speed, adc_acceleration=accel
        )
        result = MockDetailedResult()
        passed = handler.evaluate(condition, deque([world]), deque(), result)
        status = "PASS" if passed else "FAIL"
        print(f"  t={t:.1f}s | accel={accel:+.1f} m/s² | {status}"
              + (f" | 扣分={result.delta_score}" if not passed else ""))
    print()


def example_ttc_check():
    """示例：碰撞检测 (TTC)"""
    print("=" * 60)
    print("【碰撞检测】TtcConditionHandler")
    print("=" * 60)

    handler = TtcConditionHandler()

    # 配置：TTC阈值 3 秒
    condition = MockCondition(
        ttc_condition=MockTtcCondition(time=3.0)
    )

    # 主车以 15m/s 行驶，前方有静止障碍物
    scenarios = [
        ("安全距离", 100, 0),    # 障碍物在100m处
        ("需注意",    40, 0),    # 障碍物在40m处
        ("危险!",    10, 0),    # 障碍物在10m处 -> TTC < 3s
        ("同向行驶", 20, 12),   # 障碍物在20m处但也在移动
    ]

    for desc, obs_x, obs_speed in scenarios:
        world = build_world_from_frame(
            timestamp=1.0, adc_x=0, adc_y=0, adc_heading=0,
            adc_speed=15, adc_acceleration=0,
            obstacles=[{'id': 'obs1', 'x': obs_x, 'y': 0,
                       'heading': 0, 'speed': obs_speed}]
        )
        result = MockDetailedResult()
        passed = handler.evaluate(condition, deque([world]), deque(), result)
        status = "PASS" if passed else "FAIL"
        print(f"  {desc:6s} | 障碍物距离={obs_x}m, 速度={obs_speed}m/s | {status}")
    print()


def example_obstacle_bypass():
    """示例：障碍物绕行检测"""
    print("=" * 60)
    print("【障碍物绕行】ObstacleBypassConditionHandler")
    print("=" * 60)

    handler = ObstacleBypassConditionHandler()

    # 障碍物在 x=50, y=0 处，测试区域覆盖 x:[0,100], y:[-20,20]
    test_range = MockPolygon(point=[
        PolygonPoint(x=0, y=-20), PolygonPoint(x=100, y=-20),
        PolygonPoint(x=100, y=20), PolygonPoint(x=0, y=20),
    ])
    condition = MockCondition(
        obstacle_bypass_condition=MockObstacleBypassCondition(
            obstacle_id="obs1",
            min_lateral_distance=1.0,
            max_speed=8.0,
            test_range=test_range,
            use_score=True,
            single_deduction=10.0,
        )
    )

    # 模拟绕行过程：主车从 x=30 开始，沿 y=4 绕过障碍物
    frames = [
        (0.0, 30, 4, 5.0, "接近障碍物"),
        (0.5, 40, 4, 5.0, "接近中"),
        (1.0, 50, 4, 5.0, "正在绕行（投影在障碍物范围内）"),
        (1.5, 60, 4, 5.0, "绕行完成"),
    ]

    obstacle = {'id': 'obs1', 'x': 50, 'y': 0, 'heading': 0,
                'speed': 0, 'length': 4.5, 'width': 2.0}

    for t, x, y, speed, desc in frames:
        world = build_world_from_frame(
            timestamp=t, adc_x=x, adc_y=y, adc_heading=0,
            adc_speed=speed, adc_acceleration=0,
            obstacles=[obstacle]
        )
        result = MockDetailedResult()
        passed = handler.evaluate(condition, deque([world]), deque(), result)
        status = "PASS" if passed else "FAIL"
        print(f"  t={t:.1f}s | pos=({x},{y}) | {desc} | {status}")
    print()


def example_full_pipeline():
    """
    示例：完整评分流程
    模拟一段仿真回放，对每一帧同时运行多个评分模块
    """
    print("=" * 60)
    print("【完整流程】多模块联合评分")
    print("=" * 60)

    # 初始化所有 handler
    handlers = {
        'SpeedLimit': (SpeedConditionHandler(), MockCondition(
            speed_condition=MockSpeedCondition(
                name="speed_limit", min_speed=0, max_speed=16.67))),
        'Acceleration': (AccelerationConditionHandler(), MockCondition(
            acceleration_condition=MockAccelerationCondition(
                name="acceleration", min_acceleration=-4.5, max_acceleration=4.0))),
        'TTC': (TtcConditionHandler(), MockCondition(
            ttc_condition=MockTtcCondition(time=3.0))),
    }

    # 模拟 10 帧数据
    world_history = deque(maxlen=50)
    results_summary = {name: [] for name in handlers}

    obstacle = {'id': 'obs1', 'x': 80, 'y': 0, 'heading': 0, 'speed': 0}

    for i in range(10):
        t = i * 0.1
        speed = 5.0 + i * 1.5         # 逐渐加速
        accel = 1.5                     # 匀加速
        x = 5.0 * t + 0.5 * 1.5 * t * t

        world = build_world_from_frame(
            timestamp=t, adc_x=x, adc_y=0, adc_heading=0,
            adc_speed=speed, adc_acceleration=accel,
            obstacles=[obstacle], speed_limit=16.67
        )
        world_history.append(world)

        frame_results = {}
        for name, (handler, condition) in handlers.items():
            result = MockDetailedResult()
            passed = handler.evaluate(condition, world_history, deque(), result)
            frame_results[name] = passed
            results_summary[name].append(passed)

        all_pass = all(frame_results.values())
        status_str = " | ".join(f"{k}={'OK' if v else 'NG'}" for k, v in frame_results.items())
        print(f"  t={t:.1f}s | speed={speed:.1f} | {status_str}")

    # 汇总
    print("\n  --- 评分汇总 ---")
    for name, results in results_summary.items():
        pass_rate = sum(results) / len(results) * 100
        print(f"  {name:15s}: 通过率 {pass_rate:.0f}% ({sum(results)}/{len(results)})")
    print()


# ============================================================
# 3. 运行示例
# ============================================================

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("  grading_system_py 调用示例")
    print("=" * 60 + "\n")

    example_speed_check()
    example_acceleration_check()
    example_ttc_check()
    example_obstacle_bypass()
    example_full_pipeline()
