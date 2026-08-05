#!/usr/bin/env python3
"""Exact simulation of CalculateTotalCost row-index propagation (as in
gridded_path_time_graph.cc) to find where the column breaks.

Key: next_highest/next_lowest row indices propagate; CalculateCostAt computes
feasibility using the 2x acc formula; GetRowRange gives next col reachable rows.
"""
import bisect

unit_t = 1.0
dense_dimension_s = 101
dense_unit_s = 0.1
sparse_unit_s = 1.0
max_acc = 3.0
max_dec = -6.0
acc_coeff = 0.5

# scenario 19:20:28.44
init_v = 10.68
stop_rel = 82.2285 - 56.3531   # STOP relative to adc
path_length = 176.7 - 56.3531
print(f"stop_rel={stop_rel:.2f}")

nodes = []
s = 0.0
while s < 10.0 - 1e-9:
    nodes.append(round(s, 6)); s += dense_unit_s
s = 10.0
while s <= path_length + 1e-9:
    nodes.append(round(s, 6)); s += sparse_unit_s
dim_s = len(nodes)


def row_of(sv):
    return bisect.bisect_left(nodes, sv)


def in_stop(sv):
    return sv >= stop_rel - 1e-9


def get_row_range(s_cur, v0):
    s_upper = s_cur + v0 * unit_t + acc_coeff * max_acc * unit_t * unit_t
    s_lower = max(0.0, s_cur + v0 * unit_t + acc_coeff * max_dec * unit_t * unit_t)
    hi = min(dim_s - 1, row_of(s_upper))
    lo = row_of(s_lower)
    return lo, hi


dim_t = int(15.0 / unit_t) + 1

# cost_table_[c][r] feasible? Track per-column feasible rows and their v.
# Initialize col 0.
feasible = {0: {0: init_v}}
next_lo, next_hi = 0, 0

for c in range(1, dim_t):
    prev = feasible[c - 1]
    # CalculateTotalCost computes [next_lo, next_hi] of PREVIOUS iteration's
    # propagation. Here we recompute from feasible[c-1] union (equivalent).
    cur = {}
    for r_pre, v_pre in prev.items():
        s_pre = nodes[r_pre]
        lo, hi = get_row_range(s_pre, v_pre)
        for r in range(lo, hi + 1):
            s_cur = nodes[r]
            if in_stop(s_cur):
                continue
            v_cur = (s_cur - s_pre) / unit_t
            a = 2 * (v_cur - v_pre) / unit_t   # 2x formula
            if a < max_dec or a > max_acc:
                continue
            if v_cur < -1e-6:
                continue
            if r not in cur or v_cur > cur[r]:
                cur[r] = v_cur
    if not cur:
        print(f"t={c}: *** COLUMN BREAK *** (from rows "
              f"{sorted(prev.keys())[0]}..{sorted(prev.keys())[-1]})")
        break
    feasible[c] = cur
    rows = sorted(cur)
    # row-index range via GetRowRange from feasible points
    los = [get_row_range(nodes[r], cur[r])[0] for r in rows]
    his = [get_row_range(nodes[r], cur[r])[1] for r in rows]
    new_lo, new_hi = min(los), max(his)
    print(f"t={c}: s=[{nodes[rows[0]]:.2f},{nodes[rows[-1]]:.2f}] "
          f"next=[{nodes[new_lo]:.2f},{nodes[new_hi]:.2f}] maxv={max(cur.values()):.1f}")
    next_lo, next_hi = new_lo, new_hi

last = feasible.get(dim_t - 1, {})
print(f"\nt=15 feasible rows: {len(last)}")
if last:
    rs = sorted(last)
    print(f"  s range [{nodes[rs[0]]:.2f},{nodes[rs[-1]]:.2f}]")
