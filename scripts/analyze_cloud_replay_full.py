#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云端回放 826 帧深度分析：stGraph / speedPlan / mainDecision / object decision / scenario / stories / signal"""
import json
from collections import Counter, defaultdict

FRAMES = json.load(open('output/cloud_replay_decoded.json'))


def t_of(f):
    return f['timestamp'] / 1000.0


def ego(f):
    return f['autoDrivingCar']


def get_ped(f):
    for o in f['object']:
        if o.get('id') == '7673':
            return o
    return None


def main_decision_types(f):
    return [d.get('type') for d in f.get('mainDecision', {}).get('decision', [])]


# ---------------- 1. 时间线总览 ----------------
print('=' * 80)
print('1. 全程关键时间线 (t, ego_x, ego_v, ego_a, ped_x, ped_y, ped_v, mainDecision types, scenario)')
print('=' * 80)
prev_scen = None
prev_md = None
for f in FRAMES:
    t = t_of(f)
    e = ego(f)
    ped = get_ped(f)
    md = f.get('mainDecision', {})
    mdt = tuple(sorted(set(main_decision_types(f))))
    scen = f.get('planningData', {}).get('scenario', {})
    scen_str = f"{scen.get('scenarioPluginType')}/{scen.get('stagePluginType')}"
    if mdt != prev_md or scen_str != prev_scen:
        pv = ped['speed'] if ped else None
        px = ped['positionX'] if ped else None
        py = ped['positionY'] if ped else None
        print(f"  t={t:6.2f}  ego_x={e['positionX']:.2f} v={e['speed']:6.2f} a={e['speedAcceleration']:6.2f} "
              f"brk={e.get('brakePercentage',0):4.1f} | ped_x={px} py={py} pv={pv} | md={mdt} | {scen_str}")
        prev_md = mdt
        prev_scen = scen_str
