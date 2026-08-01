#!/usr/bin/env python3
"""列出 record 的 topics — 容器内运行
用法：python3 scripts/list_topics.py <record>
"""
import sys, os

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/opt/apollo/neo/src/cyber/python",
           "/opt/apollo/neo/src"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record

r = record.RecordReader(sys.argv[1])
topics = set()
for msg in r.read_messages():
    topics.add(msg.topic)
for t in sorted(topics):
    print(t)
