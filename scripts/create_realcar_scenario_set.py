#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把实车赛(苏州/河海)赛题转成 DreamView 场景集(scenario_sets)格式，
放入 ~/.apollo/resources/scenario_sets/，使环境资源里可选实车赛场景。

数据源: shiche/car_890/grading_console/auto_drive_console/*号*.json
输出:   ~/.apollo/resources/scenario_sets/<new_set_id>/
"""
import json
import os
import time
import glob

# 颜色数字 -> Apollo 信号灯枚举名
COLOR_MAP = {0: "UNKNOWN", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "BLACK"}

SRC_DIR = "/apollo_workspace/shiche/car_890/grading_console/auto_drive_console"
# 固定输出到容器内 skye 用户的环境资源(不能依赖 ~, docker exec 下是 /root)
OUT_ROOT = "/home/skye/.apollo/resources/scenario_sets"

# 场次 -> 赛题中文名(用于场景命名)
SCENE_NAME_MAP = {
    "red_light": "红灯停车",
    "crosswalk": "人行道让行",
    "cruise": "定速巡航",
    "traffic_road_speed": "路口限速",
    "crosswalk_road_speed": "人行道路口减速",
}


def gen_id():
    """生成 24 位 hex id(与仿真赛场景 id 格式一致)"""
    import hashlib
    h = hashlib.md5(f"{time.time()}{os.urandom(8)}".encode()).hexdigest()
    return h[:24]


def convert_light(light):
    """实车赛 traffic_lights -> DreamView roadNetwork.trafficLights 格式"""
    init = light.get("initial_state", {})
    return {
        "id": light.get("id"),
        "location": light.get("location"),
        "triggerType": "DISTANCE" if light.get("trigger_type") == 1 else str(light.get("trigger_type", "")),
        "triggerValue": light.get("trigger_value"),
        "initialState": {"color": COLOR_MAP.get(init.get("color"), "UNKNOWN")},
        "stateGroup": [
            {"color": COLOR_MAP.get(s.get("color"), "UNKNOWN"), "keepTime": s.get("keep_time")}
            for s in light.get("state_group", [])
        ],
    }


def build_scenario_json(session, sim_time, traffic_lights, scene_ids, map_dir, map_id):
    """构造 DreamView 场景 json"""
    scid = gen_id()
    start = session["scenario"]["start"]
    end = session["scenario"]["end"]
    scene_names = [SCENE_NAME_MAP.get(s, s) for s in scene_ids]
    desc = session.get("name", "实车赛") + "(" + "+".join(scene_names) + ")"

    scenario = {
        "roadNetwork": {
            "logicFile": {"filepath": map_dir},
        },
        "entities": {},
        "storyboard": {
            "init": {"actions": {}},
            "stories": [{"acts": [{}]}],
            "stopTrigger": {
                "conditionGroups": [{
                    "conditions": [{
                        "conditionEdge": "none",
                        "name": "end",
                        "byValueCondition": {
                            "simulationTimeCondition": {"rule": "greaterOrEqual", "value": sim_time}
                        }
                    }]
                }]
            }
        },
        "autoCarInfo": {
            "start": {"x": start["x"], "y": start["y"], "heading": session.get("start_heading", 0.0)},
            "end": {"x": end["x"], "y": end["y"]},
            "routingRequest": {
                "waypoint": [
                    {"pose": {"x": start["x"], "y": start["y"]}},
                    {"pose": {"x": end["x"], "y": end["y"]}},
                ]
            },
        },
        "gradingConfigInfo": {
            "baseGradeConfigFile": "grading_system/conf/grading_metrics_default.conf",
            "deselectDefaultMetric": ["Checkpoint"],
        },
    }
    if traffic_lights:
        scenario["roadNetwork"]["trafficLights"] = [convert_light(t) for t in traffic_lights]

    return {
        "id": scid,
        "authorName": "realcar",
        "scenario": scenario,
        "type": "worldsim",
        "mapId": map_id,
        "tags": session.get("tags", ["Straight Road"]),
        "time": time.strftime("%a %b %d %Y %H:%M:%S GMT+0000 (Coordinated Universal Time)"),
        "createTime": time.strftime("%a %b %d %Y %H:%M:%S GMT+0000 (Coordinated Universal Time)"),
        "descriptionEnTokens": [desc],
    }


def main():
    os.makedirs(OUT_ROOT, exist_ok=True)
    set_id = gen_id()
    set_dir = os.path.join(OUT_ROOT, set_id)
    scen_dir = os.path.join(set_dir, "scenarios")
    os.makedirs(scen_dir, exist_ok=True)

    scenario_ids = []
    created = []

    # 读取 4 个场次场景(按文件名匹配 *号*.json，排除 metric/)
    for f in sorted(glob.glob(os.path.join(SRC_DIR, "*号*.json"))):
        session = json.load(open(f, encoding="utf-8"))
        map_id = session.get("mapId", "")
        map_dir = session["scenario"].get("map_dir", "modules/map/data/" + map_id)
        sim_time = session["scenario"].get("simulator_time", 120)
        lights = session["scenario"].get("traffic_lights", [])

        # 从同场次 scenes.json 读赛题 id 列表(文件名: {场次名}号赛道_scenes.json)
        sess_name = session.get("name", "")
        scene_ids = []
        scenes_file = os.path.join(SRC_DIR, "metric", f"{sess_name}号赛道_scenes.json")
        if os.path.exists(scenes_file):
            sd = json.load(open(scenes_file, encoding="utf-8"))
            scene_ids = [s.get("id") for s in sd.get("scenes", [])]

        sc_json = build_scenario_json(session, sim_time, lights, scene_ids, map_dir, map_id)
        scen_file = os.path.join(scen_dir, sc_json["id"] + ".json")
        json.dump(sc_json, open(scen_file, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        scenario_ids.append(sc_json["id"])
        created.append(sc_json["descriptionEnTokens"][0])

    # 场景集定义
    now = time.time()
    set_json = {
        "id": set_id,
        "name": "2026星火自动驾驶比赛--实车挑战赛(河海)",
        "userid": "627df19dc67edd098a4040b4",
        "ticketids": scenario_ids,
        "visibleToPrivilegedUsers": False,
        "createTime": now,
        "modTime": now,
        "category": "subscriber",
        "isPublic": True,
        "isClassic": False,
    }
    json.dump(set_json, open(os.path.join(set_dir, "scenario_set.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    print(f"场景集已创建: {set_dir}")
    print(f"  id: {set_id}")
    print(f"  场景数: {len(created)}")
    for c in created:
        print(f"    - {c}")


if __name__ == "__main__":
    main()
