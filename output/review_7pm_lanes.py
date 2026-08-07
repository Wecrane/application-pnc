#!/usr/bin/env python3
"""列出所有车道及其 pred/succ、y 范围，排查连接。"""
import math
import re

MAP = "/home/skye/application-pnc/data/map_data/hehai_contest_7_pm_3/base_map.txt"


def parse_blocks(text, block_name):
    pat = re.compile(r'(?m)^\s*%s\s*\{' % re.escape(block_name))
    blocks = []
    for m in pat.finditer(text):
        i = m.end()
        depth = 1
        while i < len(text) and depth > 0:
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
            i += 1
        blocks.append(text[m.start():i])
    return blocks


def get_field(block, field):
    m = re.search(r'(?m)^\s*%s\s*:\s*"([^"]*)"' % re.escape(field), block)
    if m:
        return m.group(1)
    return None


def get_points(block, struct):
    m = re.search(r'(?m)^\s*%s\s*\{' % re.escape(struct), block)
    if not m:
        return []
    i = m.end()
    depth = 1
    while i < len(block) and depth > 0:
        if block[i] == '{':
            depth += 1
        elif block[i] == '}':
            depth -= 1
        i += 1
    sub = block[m.start():i]
    pts = []
    for pm in re.finditer(r'point\s*\{([^}]*)\}', sub):
        s = pm.group(1)
        x = re.search(r'x\s*:\s*(-?[\d.]+(?:[eE][+-]?\d+)?)', s)
        y = re.search(r'y\s*:\s*(-?[\d.]+(?:[eE][+-]?\d+)?)', s)
        if x and y:
            pts.append((float(x.group(1)), float(y.group(1))))
    return pts


with open(MAP) as f:
    text = f.read()

lanes = []
for blk in parse_blocks(text, 'lane'):
    lid = get_field(blk, 'id')
    if not lid:
        continue
    pts = get_points(blk, 'central_curve')
    # 所有 pred/succ
    preds = re.findall(r'(?m)^\s*predecessor_id\s*\{\s*id\s*:\s*"([^"]*)"', blk)
    succs = re.findall(r'(?m)^\s*successor_id\s*\{\s*id\s*:\s*"([^"]*)"', blk)
    if pts:
        ys = [p[1] for p in pts]
        xs = [p[0] for p in pts]
        ymin, ymax = min(ys), max(ys)
        xmin, xmax = min(xs), max(xs)
    else:
        ymin = ymax = xmin = xmax = float('nan')
    lanes.append({
        'id': lid, 'pts': pts, 'preds': preds, 'succs': succs,
        'y': (ymin, ymax), 'x': (xmin, xmax), 'L': len(pts),
    })

# 按 y 范围排序
lanes.sort(key=lambda l: (l['y'][0], l['x'][0]))
print("%-8s %-16s %-16s  %-16s  %-16s  %s" % ("id", "pred", "succ", "x范围", "y范围", "npts"))
for l in lanes:
    print("%-8s %-14s %-14s  [%.2f,%.2f]  [%.2f,%.2f]  %d" % (
        l['id'], ','.join(l['preds']), ','.join(l['succs']),
        l['x'][0], l['x'][1], l['y'][0], l['y'][1], l['L']))
