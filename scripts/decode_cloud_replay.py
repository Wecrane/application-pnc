#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解密云端 OfflineView 回放数据（SimWorldUpdate AES 加密 JSON）并分析。

用法:
  python3 decode_cloud_replay.py <cloud_replay.json> [输出前缀]
"""
import json
import base64
import hashlib
import sys
import os

from Crypto.Cipher import AES

KEY = hashlib.sha256('明月几时有'.encode('utf-8')).digest()


def decrypt_world(world_b64):
    raw = base64.b64decode(world_b64)
    iv = raw[:16]
    ct = raw[16:]
    cipher = AES.new(KEY, AES.MODE_CBC, iv)
    pt = cipher.decrypt(ct)
    pad = pt[-1]
    pt = pt[:-pad]
    return pt.decode('utf-8')


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'output/cloud_replay.json'
    prefix = sys.argv[2] if len(sys.argv) > 2 else src.replace('.json', '')
    data = json.load(open(src))
    sims = [f for f in data if f.get('type') == 'SimWorldUpdate']
    print('SimWorldUpdate frames:', len(sims))

    frames = []
    for i, f in enumerate(sims):
        try:
            w = json.loads(decrypt_world(f['world']))
            frames.append(w)
        except Exception as e:
            print('frame', i, 'decrypt fail:', e)
            continue

    # 保存全部解密帧
    out = prefix + '_decoded.json'
    json.dump(frames, open(out, 'w'))
    print('decoded frames saved:', out, 'count:', len(frames))

    # 基础统计
    if not frames:
        return
    print('\n=== 帧时间线 ===')
    ts = [f.get('timestamp') for f in frames]
    print('first/last timestamp:', ts[0], ts[-1], 'frames:', len(ts))

    # 自车轨迹采样
    print('\n=== 自车轨迹采样（每20帧）===')
    for f in frames[::20]:
        adc = f.get('autoDrivingCar', {})
        print('t=%.2f seq=%s x=%.2f y=%.2f h=%.3f' % (
            f.get('timestamp', 0), f.get('sequenceNum'), adc.get('positionX', 0),
            adc.get('positionY', 0), adc.get('heading', 0)))

    # 障碍物类型统计
    print('\n=== 障碍物类型统计 ===')
    types = {}
    ids = {}
    for f in frames:
        for obj in f.get('object', []):
            t = obj.get('type', '?')
            types[t] = types.get(t, 0) + 1
            oid = obj.get('id', 'NOID')
            if oid not in ids:
                ids[oid] = {'type': t, 'count': 0}
            ids[oid]['count'] += 1
    print('types:', types)
    print('ids:', json.dumps(ids, indent=0))

    # STOP 决策统计
    print('\n=== STOP/决策统计 ===')
    stops = 0
    follows = 0
    yields = 0
    for f in frames:
        for obj in f.get('object', []):
            for d in obj.get('decision', []):
                t = d.get('type')
                if t == 'STOP':
                    stops += 1
                elif t == 'FOLLOW':
                    follows += 1
                elif t == 'YIELD':
                    yields += 1
    print('STOP decisions:', stops, 'FOLLOW:', follows, 'YIELD:', yields)

    # main decision
    print('\n=== mainDecision 统计 ===')
    main_stop = {}
    for f in frames:
        md = f.get('mainDecision', {})
        reason = md.get('mainStopReason', '')
        if reason:
            main_stop[reason] = main_stop.get(reason, 0) + 1
    print('mainStopReason:', main_stop)


if __name__ == '__main__':
    main()
