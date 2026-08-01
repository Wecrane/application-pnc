#!/usr/bin/env python3
"""转储 prediction 中行人 7673 的 is_static 标记 + 轨迹数 + 位置/速度 (0.5s采样)
使用 grpc_tools.protoc 现场编译 prediction_obstacle.proto 获取 pb2。
用法: python3 scripts/dump_pred_7673_v2.py <record> <outfile>
"""
import sys, os, tempfile, shutil

PROTO_SRC = "/opt/apollo/neo/src"

# 现场编译 pb2
tmp = tempfile.mkdtemp(prefix="pred_pb2_")
sys.path.insert(0, tmp)
try:
    import grpc_tools.protoc
    ret = grpc_tools.protoc.main([
        "protoc",
        f"-I{PROTO_SRC}",
        f"--python_out={tmp}",
        f"{PROTO_SRC}/modules/common_msgs/prediction_msgs/prediction_obstacle.proto",
    ])
    if ret != 0:
        sys.stderr.write(f"protoc failed rc={ret}\n")
        sys.exit(1)
    from modules.common_msgs.prediction_msgs import prediction_obstacle_pb2 as pb2
except Exception as e:
    sys.stderr.write(f"compile/import failed: {e!r}\n")
    sys.exit(2)

for _p in ["/apollo_workspace/bazel-out/k8-opt/bin/external/apollo_src",
           "/apollo_workspace/bazel-out/k8-opt/bin",
           "/opt/apollo/neo/src"]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from cyber.python.cyber_py3 import record

rf, outfile = sys.argv[1], sys.argv[2]
r = record.RecordReader(rf)
base = None
lines = ["t    pred_id is_static n_traj n_pts  pos(x,y) vel(x,y,spd) traj_first_pt"]
for m in r.read_messages():
    if m.topic != "/apollo/prediction":
        continue
    ts = m.timestamp / 1e9
    if base is None:
        base = ts
    t = ts - base
    if int(t * 2) != int(t * 2 - 0.01):
        continue
    po = pb2.PredictionObstacles()
    po.ParseFromString(m.message)
    for ob in po.prediction_obstacle:
        pid = ob.perception_obstacle.id
        if str(pid) != "7673":
            continue
        pos = ob.perception_obstacle.position
        vel = ob.perception_obstacle.velocity
        spd = (vel.x ** 2 + vel.y ** 2) ** 0.5
        ntr = len(ob.trajectory)
        npts = sum(len(tr.trajectory_point) for tr in ob.trajectory)
        first = ""
        if ntr > 0 and len(ob.trajectory[0].trajectory_point) > 0:
            p0 = ob.trajectory[0].trajectory_point[0]
            # 动态探测轨迹点里的坐标：trajectory_point.path_point.x/y 或 position.x/y
            try:
                tp = p0
                if hasattr(tp, "trajectory_point"):
                    tp = tp.trajectory_point
                if hasattr(tp, "path_point"):
                    pp = tp.path_point
                    first = f"({pp.x:.2f},{pp.y:.2f})"
                elif hasattr(tp, "position"):
                    first = f"({tp.position.x:.2f},{tp.position.y:.2f})"
                else:
                    first = "?" + ",".join(fd.name for fd in tp.DESCRIPTOR.fields)[:40]
            except Exception:
                first = "?"
        lines.append(f"{t:6.2f} {pid:>10} {str(ob.is_static):>7} {ntr:>5} {npts:>5} "
                     f"({pos.x:.2f},{pos.y:.2f}) ({vel.x:.2f},{vel.y:.2f},{spd:.2f}) {first}")

with open(outfile, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"WROTE {len(lines)-1} rows -> {outfile}")
shutil.rmtree(tmp, ignore_errors=True)
