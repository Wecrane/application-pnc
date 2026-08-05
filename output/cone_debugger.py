#!/usr/bin/env python3
"""
施工区域锥桶分类交互调试器
============================
逐帧查看分类详情，手动覆盖分类，即时看效果。

用法:
    python3 cone_debugger.py                  # 完整仿真 + 交互菜单
    python3 cone_debugger.py --frame 5        # 直接跳到第5帧
    python3 cone_debugger.py --cone L_10      # 追踪特定锥桶

交互命令:
    n / Enter   下一帧
    p           上一帧
    j N         跳转到第 N 帧
    i ID        查看锥桶详情 (如: i L_04)
    s           显示本帧所有锥桶摘要
    w           显示墙壁状态
    f ID DIR    强制将锥桶分到 L 或 R (如: f L_04 L)
    u ID        取消强制
    r           从当前帧重新运行到结束
    q           退出
"""

import cmd
import copy
import math
import sys
import os
from typing import List, Dict, Tuple, Optional

# 导入仿真核心
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cone_classification_sim import (
    SLPolygon, ConstructionZoneState, ReferenceLine, NudgeType,
    classify_cones, create_reference_line,
    RIGHT_CONES_XY, LEFT_CONES_XY,
    LEFT_ROAD_BOUNDARY, RIGHT_ROAD_BOUNDARY,
    interpolate_wall_l, nearest_xy_distance,
)


class ConeDebugger:
    """交互式锥桶分类调试器"""

    def __init__(self):
        # 构建数据
        self.ref_line = create_reference_line()
        self.all_cones = self._build_cones()
        self.road_left, self.road_right = self._compute_road_bounds()

        # 仿真参数
        self.look_forward = 60.0
        self.look_backward = 10.0
        self.step_size = 5.0

        # 状态
        self.state = ConstructionZoneState()
        self.state.max_left_bound = self.road_left
        self.state.max_right_bound = self.road_right

        # 帧记录
        self.frames: List[dict] = []  # [{adc_s, visible, classified, ...}]
        self.current_frame = 0

        # 手动覆盖
        self.force_classify: Dict[str, bool] = {}  # cone_id -> to_left

        # 运行完整仿真
        self._run_full_sim()

    def _build_cones(self) -> List[SLPolygon]:
        cones = []
        for i, (x, y) in enumerate(RIGHT_CONES_XY):
            s, l = self.ref_line.xy_to_sl(x, y)
            cones.append(SLPolygon(id=f"R_{i:02d}", s_min=s-0.1, s_max=s+0.1,
                                    l_min=l-0.1, l_max=l+0.1, x=x, y=y))
        for i, (x, y) in enumerate(LEFT_CONES_XY):
            s, l = self.ref_line.xy_to_sl(x, y)
            cones.append(SLPolygon(id=f"L_{i:02d}", s_min=s-0.1, s_max=s+0.1,
                                    l_min=l-0.1, l_max=l+0.1, x=x, y=y))
        cones.sort(key=lambda c: c.s_center)
        return cones

    def _compute_road_bounds(self) -> Tuple[float, float]:
        left_ls = [self.ref_line.xy_to_sl(x, y)[1] for x, y in LEFT_ROAD_BOUNDARY]
        right_ls = [self.ref_line.xy_to_sl(x, y)[1] for x, y in RIGHT_ROAD_BOUNDARY]
        return max(left_ls), min(right_ls)

    def _run_full_sim(self):
        """运行完整仿真，记录每帧状态"""
        state = ConstructionZoneState()
        state.max_left_bound = self.road_left
        state.max_right_bound = self.road_right

        adc_s = min(c.s_center for c in self.all_cones) - self.look_backward - 10
        max_s = max(c.s_center for c in self.all_cones) + 20

        while adc_s < max_s:
            visible = [c for c in self.all_cones
                       if adc_s - self.look_backward <= c.s_center <= adc_s + self.look_forward]
            if visible:
                cones_frame = copy.deepcopy(visible)
                classify_cones(cones_frame, self.road_left, self.road_right,
                             state, adc_back_s=adc_s, verbose=False)
                self.frames.append({
                    'adc_s': adc_s,
                    'visible': visible,
                    'classified': cones_frame,
                    'left_wall': copy.deepcopy(state.left_wall),
                    'right_wall': copy.deepcopy(state.right_wall),
                    'left_xy': copy.deepcopy(state.classified_left_xy),
                    'right_xy': copy.deepcopy(state.classified_right_xy),
                })
            else:
                self.frames.append({
                    'adc_s': adc_s,
                    'visible': [],
                    'classified': [],
                    'left_wall': copy.deepcopy(state.left_wall),
                    'right_wall': copy.deepcopy(state.right_wall),
                    'left_xy': copy.deepcopy(state.classified_left_xy),
                    'right_xy': copy.deepcopy(state.classified_right_xy),
                })
            adc_s += self.step_size

        self.state = state

    # ── 查询方法 ──

    def frame_count(self) -> int:
        return len(self.frames)

    def get_frame(self, idx: int) -> dict:
        if 0 <= idx < len(self.frames):
            return self.frames[idx]
        return None

    def find_cone_frames(self, cone_id: str) -> List[int]:
        """找到锥桶出现的所有帧"""
        result = []
        for i, f in enumerate(self.frames):
            for c in f['visible']:
                if c.id == cone_id:
                    result.append(i)
                    break
        return result

    def get_cone_detail(self, cone_id: str, frame_idx: int) -> Optional[str]:
        """获取锥桶在指定帧的分类详情"""
        f = self.get_frame(frame_idx)
        if not f:
            return None

        cone = None
        for c in f['classified']:
            if c.id == cone_id:
                cone = c
                break
        if not cone:
            # 查找可见锥桶
            for c in f['visible']:
                if c.id == cone_id:
                    cone = c
                    break
        if not cone:
            return f"锥桶 {cone_id} 不在第 {frame_idx} 帧"

        lines = []
        lines.append(f"══════════════════════════════════════")
        lines.append(f"  锥桶: {cone.id}")
        lines.append(f"  XY:   ({cone.x:.3f}, {cone.y:.6f})")
        lines.append(f"  SL:   s={cone.s_center:.2f}  l={cone.l_center:.2f}")
        lines.append(f"  GT:   {'左墙' if cone.id.startswith('L_') else '右墙'}")

        # 分类结果
        side = cone.classified_side()
        gt = 'L' if cone.id.startswith('L_') else 'R'
        correct = '✓' if side == gt else '✗'
        lines.append(f"  分类: {side} ({'正确' if correct == '✓' else '错误'}) {correct}")

        # 墙壁插值
        pl = interpolate_wall_l(f['left_wall'], cone.s_center)
        pr = interpolate_wall_l(f['right_wall'], cone.s_center)
        pl_s = "nan" if math.isnan(pl) else f"{pl:.2f}"
        pr_s = "nan" if math.isnan(pr) else f"{pr:.2f}"
        lines.append(f"  左墙插值 l: {pl_s}  右墙插值 l: {pr_s}")

        if not math.isnan(pl) and not math.isnan(pr):
            dl = abs(cone.l_center - pl)
            dr = abs(cone.l_center - pr)
            lines.append(f"  dL={dl:.2f}  dR={dr:.2f}  |dL-dR|={abs(dl-dr):.2f}")
            if dl < dr:
                lines.append(f"  → SL判断: 左墙 (dL < dR)")
            else:
                lines.append(f"  → SL判断: 右墙 (dL >= dR)")

        # XY 最近邻
        if f['left_xy'] and f['right_xy']:
            nl = nearest_xy_distance(cone.x, cone.y, f['left_xy'])
            nr = nearest_xy_distance(cone.x, cone.y, f['right_xy'])
            lines.append(f"  XY近邻: 左={nl:.2f}m  右={nr:.2f}m")
            if nl < nr:
                lines.append(f"  → XY判断: 左墙")
            else:
                lines.append(f"  → XY判断: 右墙")

        # 墙壁状态
        lines.append(f"  左墙: {len(f['left_wall'])} 点, s=[{f['left_wall'][0][0]:.1f},{f['left_wall'][-1][0]:.1f}]" if f['left_wall'] else "  左墙: 空")
        lines.append(f"  右墙: {len(f['right_wall'])} 点, s=[{f['right_wall'][0][0]:.1f},{f['right_wall'][-1][0]:.1f}]" if f['right_wall'] else "  右墙: 空")
        lines.append(f"  left_xy: {len(f['left_xy'])} 点  right_xy: {len(f['right_xy'])} 点")

        # 覆盖状态
        if cone.id in self.force_classify:
            forced = '左' if self.force_classify[cone.id] else '右'
            lines.append(f"  ⚠ 手动覆盖: {forced}墙")

        lines.append(f"══════════════════════════════════════")
        return '\n'.join(lines)

    # ── 手动覆盖 ──

    def force_cone(self, cone_id: str, side: str) -> str:
        """强制锥桶分类"""
        if side.upper() == 'L':
            self.force_classify[cone_id] = True
            return f"已强制 {cone_id} → 左墙"
        elif side.upper() == 'R':
            self.force_classify[cone_id] = False
            return f"已强制 {cone_id} → 右墙"
        return f"无效方向: {side} (用 L 或 R)"

    def unforce_cone(self, cone_id: str) -> str:
        if cone_id in self.force_classify:
            del self.force_classify[cone_id]
            return f"已取消 {cone_id} 的强制"
        return f"{cone_id} 未被强制"

    def rerun_from(self, frame_idx: int) -> str:
        """从指定帧重新运行仿真（应用手动覆盖）"""
        if frame_idx < 0 or frame_idx >= len(self.frames):
            return f"帧 {frame_idx} 不存在"

        state = ConstructionZoneState()
        state.max_left_bound = self.road_left
        state.max_right_bound = self.road_right

        # 重放到 frame_idx-1
        for i in range(frame_idx):
            f = self.frames[i]
            if f['visible']:
                cones_f = copy.deepcopy(f['visible'])
                classify_cones(cones_f, self.road_left, self.road_right,
                             state, adc_back_s=f['adc_s'], verbose=False)

        # 从 frame_idx 开始，应用手动覆盖，重新分类所有帧
        for i in range(frame_idx, len(self.frames)):
            f = self.frames[i]
            if not f['visible']:
                continue

            cones_f = copy.deepcopy(f['visible'])

            # 应用手动覆盖：直接修改锥桶的 nudge
            # 注意：需要在 classify_cones 调用前后处理
            for cone in cones_f:
                if cone.id in self.force_classify:
                    cone.set_nudge_from_to_left(self.force_classify[cone.id])

            # 重新分类（未强制覆盖的锥桶）
            classify_cones(cones_f, self.road_left, self.road_right,
                         state, adc_back_s=f['adc_s'], verbose=False)

            # 更新帧数据
            f['classified'] = cones_f
            f['left_wall'] = copy.deepcopy(state.left_wall)
            f['right_wall'] = copy.deepcopy(state.right_wall)
            f['left_xy'] = copy.deepcopy(state.classified_left_xy)
            f['right_xy'] = copy.deepcopy(state.classified_right_xy)

        self.state = state
        return f"已从第 {frame_idx} 帧重新运行（应用 {len(self.force_classify)} 个强制覆盖）"

    def stats(self) -> str:
        """统计全部分类正确率"""
        correct = 0
        total = 0
        for f in self.frames:
            for c in f['classified']:
                gt = 'L' if c.id.startswith('L_') else 'R'
                if c.classified_side() == gt:
                    correct += 1
                total += 1

        # 最终全量
        final_state = ConstructionZoneState()
        final_state.max_left_bound = self.road_left
        final_state.max_right_bound = self.road_right
        final_cones = copy.deepcopy(self.all_cones)
        classify_cones(final_cones, self.road_left, self.road_right,
                      final_state, verbose=False)
        final_correct = sum(1 for c in final_cones
                          if c.classified_side() == ('L' if c.id.startswith('L_') else 'R'))

        lines = [
            f"══════════════════════════════════════",
            f"  逐帧累计: {correct}/{total} = {correct/total*100:.1f}%" if total > 0 else "  无数据",
            f"  最终全量: {final_correct}/{len(final_cones)} = {final_correct/len(final_cones)*100:.1f}%",
            f"  手动覆盖: {len(self.force_classify)} 个",
            f"══════════════════════════════════════",
        ]
        return '\n'.join(lines)

    def frame_summary(self, frame_idx: int) -> str:
        """显示帧摘要"""
        f = self.get_frame(frame_idx)
        if not f:
            return f"帧 {frame_idx} 不存在"

        lines = []
        lines.append(f"══════════════════════════════════════")
        lines.append(f"  第 {frame_idx} 帧  adc_s={f['adc_s']:.1f}m")
        lines.append(f"  可见锥桶: {len(f['visible'])}")
        lines.append(f"  左墙: {len(f['left_wall'])}点  右墙: {len(f['right_wall'])}点")
        lines.append(f"  lxy: {len(f['left_xy'])}  rxy: {len(f['right_xy'])}")
        lines.append(f"  ──────────────────────────────────")

        errors = []
        for c in f['classified']:
            gt = 'L' if c.id.startswith('L_') else 'R'
            pred = c.classified_side()
            ok = '✓' if gt == pred else '✗'
            forced = ' [强制]' if c.id in self.force_classify else ''
            line = f"  {c.id:<6s} s={c.s_center:6.1f} l={c.l_center:6.2f}  GT={gt} →{pred} {ok}{forced}"
            lines.append(line)
            if ok == '✗':
                errors.append(c.id)

        if errors:
            lines.append(f"  ──────────────────────────────────")
            lines.append(f"  错误 ({len(errors)}): {', '.join(errors)}")

        lines.append(f"══════════════════════════════════════")
        return '\n'.join(lines)


# ── 交互式命令解释器 ──

class DebuggerShell(cmd.Cmd):
    intro = """
╔══════════════════════════════════════════════════╗
║     施工区域锥桶分类 - 交互调试器               ║
║     n=下一帧  p=上一帧  j N=跳转  s=摘要        ║
║     i ID=详情  w=墙壁  f ID L/R=强制  r=重跑    ║
║     t=统计  q=退出  h=帮助                      ║
╚══════════════════════════════════════════════════╝
"""
    prompt = f"\n[帧 0/{0}]> "

    def __init__(self, debugger: ConeDebugger):
        super().__init__()
        self.dbg = debugger
        self._update_prompt()

    def _update_prompt(self):
        self.prompt = f"\n[帧 {self.dbg.current_frame}/{self.dbg.frame_count()-1}]> "

    # ── 导航命令 ──

    def do_n(self, arg):
        """下一帧"""
        if self.dbg.current_frame < self.dbg.frame_count() - 1:
            self.dbg.current_frame += 1
            self._update_prompt()
            f = self.dbg.get_frame(self.dbg.current_frame)
            if f:
                errors = sum(1 for c in f['classified']
                           if c.classified_side() != ('L' if c.id.startswith('L_') else 'R'))
                print(f"第 {self.dbg.current_frame} 帧  adc_s={f['adc_s']:.1f}  "
                      f"锥桶={len(f['visible'])}  错误={errors}")
        else:
            print("已是最后一帧")

    def do_p(self, arg):
        """上一帧"""
        if self.dbg.current_frame > 0:
            self.dbg.current_frame -= 1
            self._update_prompt()
            f = self.dbg.get_frame(self.dbg.current_frame)
            if f:
                errors = sum(1 for c in f['classified']
                           if c.classified_side() != ('L' if c.id.startswith('L_') else 'R'))
                print(f"第 {self.dbg.current_frame} 帧  adc_s={f['adc_s']:.1f}  "
                      f"锥桶={len(f['visible'])}  错误={errors}")
        else:
            print("已是第一帧")

    def do_j(self, arg):
        """跳转到指定帧"""
        try:
            idx = int(arg.strip())
            if 0 <= idx < self.dbg.frame_count():
                self.dbg.current_frame = idx
                self._update_prompt()
                f = self.dbg.get_frame(idx)
                if f:
                    errors = sum(1 for c in f['classified']
                               if c.classified_side() != ('L' if c.id.startswith('L_') else 'R'))
                    print(f"跳转到第 {idx} 帧  adc_s={f['adc_s']:.1f}  "
                          f"锥桶={len(f['visible'])}  错误={errors}")
            else:
                print(f"帧号需在 0-{self.dbg.frame_count()-1} 之间")
        except ValueError:
            print(f"无效帧号: {arg}")

    # ── 查看命令 ──

    def do_s(self, arg):
        """显示当前帧摘要"""
        print(self.dbg.frame_summary(self.dbg.current_frame))

    def do_i(self, arg):
        """查看锥桶详情"""
        cone_id = arg.strip()
        if not cone_id:
            print("用法: i <锥桶ID>  如: i L_04")
            return
        detail = self.dbg.get_cone_detail(cone_id, self.dbg.current_frame)
        print(detail)

    def do_w(self, arg):
        """显示墙壁状态"""
        f = self.dbg.get_frame(self.dbg.current_frame)
        if not f:
            return
        print(f"左墙 ({len(f['left_wall'])}点):")
        for s, l in f['left_wall'][-10:]:
            print(f"  s={s:.1f}  l={l:.2f}")
        print(f"右墙 ({len(f['right_wall'])}点):")
        for s, l in f['right_wall'][-10:]:
            print(f"  s={s:.1f}  l={l:.2f}")

    def do_t(self, arg):
        """统计"""
        print(self.dbg.stats())

    # ── 修改命令 ──

    def do_f(self, arg):
        """强制锥桶分类: f <锥桶ID> L|R"""
        parts = arg.strip().split()
        if len(parts) != 2:
            print("用法: f <锥桶ID> L|R  如: f L_04 L")
            return
        print(self.dbg.force_cone(parts[0], parts[1]))

    def do_u(self, arg):
        """取消强制"""
        cone_id = arg.strip()
        if not cone_id:
            print("用法: u <锥桶ID>")
            return
        print(self.dbg.unforce_cone(cone_id))

    def do_r(self, arg):
        """从当前帧重新运行"""
        print(self.dbg.rerun_from(self.dbg.current_frame))

    def do_q(self, arg):
        """退出"""
        print("再见!")
        return True

    def do_track(self, arg):
        """追踪锥桶: 显示该锥桶出现过的所有帧及分类"""
        cone_id = arg.strip()
        if not cone_id:
            print("用法: track <锥桶ID>")
            return
        frames = self.dbg.find_cone_frames(cone_id)
        if not frames:
            print(f"锥桶 {cone_id} 未找到")
            return
        print(f"锥桶 {cone_id} 出现在以下帧:")
        for fi in frames:
            f = self.dbg.get_frame(fi)
            for c in f['classified']:
                if c.id == cone_id:
                    gt = 'L' if c.id.startswith('L_') else 'R'
                    print(f"  帧 {fi:3d}: adc_s={f['adc_s']:6.1f}  {c.classified_side()}  "
                          f"(GT={gt})  {'✓' if c.classified_side() == gt else '✗'}")

    def do_export(self, arg):
        """导出错误列表"""
        errors = []
        for fi, f in enumerate(self.dbg.frames):
            for c in f['classified']:
                gt = 'L' if c.id.startswith('L_') else 'R'
                if c.classified_side() != gt:
                    errors.append((fi, c.id, c.s_center, c.l_center, gt, c.classified_side()))
        if errors:
            print(f"错误锥桶 ({len(errors)}):")
            for fi, cid, s, l, gt, pred in errors:
                print(f"  帧{fi:3d} {cid:6s} s={s:6.1f} l={l:6.2f} GT={gt} →{pred}")
        else:
            print("没有错误！全部正确 ✓")

    # 空行 = 下一帧
    def emptyline(self):
        return self.do_n("")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="施工区域锥桶分类交互调试器")
    parser.add_argument("--frame", type=int, default=0, help="起始帧")
    parser.add_argument("--cone", type=str, default="", help="追踪特定锥桶")
    args = parser.parse_args()

    print("正在运行仿真...")
    dbg = ConeDebugger()
    print(f"仿真完成: {dbg.frame_count()} 帧, {len(dbg.all_cones)} 个锥桶")

    if args.cone:
        frames = dbg.find_cone_frames(args.cone)
        if frames:
            print(f"\n锥桶 {args.cone} 出现在帧: {frames}")
            # 跳转到第一次出现
            dbg.current_frame = frames[0] if args.frame == 0 else args.frame
            print(dbg.get_cone_detail(args.cone, dbg.current_frame))

    if args.frame > 0:
        dbg.current_frame = args.frame

    # 启动交互 shell
    shell = DebuggerShell(dbg)
    shell.dbg.current_frame = dbg.current_frame
    shell._update_prompt()
    shell.cmdloop()


if __name__ == "__main__":
    main()
