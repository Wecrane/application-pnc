#!/usr/bin/env python3
"""Exact simulation of GriddedPathTimeGraph DP reachability to find where the
column breaks (causing 'Retrieve best speed profile failed').

Models CalculateCostAt feasibility + GetRowRange propagation.
"""
import bisect

# ---- config ----
unit_t = 1.0
dense_dimension_s = 101
dense_unit_s = 0.1
sparse_unit_s = 1.0
max_acc = 3.0
max_dec = -6.0
acc_coeff = 0.5

# ---- scenario 19:20:28.44 ----
init_v = 10.68
init_a = 0.0
init_s_rel = 0.0          # DP works in relative s (adc at s=0)
stop_rel = 82.2285 - 56.3531   # STOP relative to adc
path_length = 176.7 - 56.3531  # relative path length
print(f"stop_rel={stop_rel:.2f}")

# build nodes
nodes = []
s = 0.0
while s < 10.0 - 1e-9:
    nodes.append(round(s, 6))
    s += dense_unit_s
s = 10.0
while s <= path_length + 1e-9:
    nodes.append(round(s, 6))
    s += sparse_unit_s
dim_s = len(nodes)
print(f"nodes={dim_s} s=[{nodes[0]},{nodes[-1]}]")

# speed limit lookup (from crosswalk 24m advance + stop fence)
# s region: limit 4.8 from (stop_rel-24) to (stop_rel+3); beyond: no limit
def speed_limit(sv):
    lo = stop_rel - 24.0
    hi = stop_rel + 3.0
    if lo <= sv <= hi:
        return 4.8
    return 20.0  # planning_upper_speed_limit


def in_stop(sv):
    return sv >= stop_rel - 1e-9


def row_of(sv):
    return bisect.bisect_left(nodes, sv)


def get_row_range(s_cur, v0):
    s_upper = s_cur + v0 * unit_t + acc_coeff * max_acc * unit_t * unit_t
    s_lower = max(0.0, s_cur + v0 * unit_t + acc_coeff * max_dec * unit_t * unit_t)
    hi = min(dim_s - 1, row_of(s_upper))
    lo = row_of(s_lower)
    return lo, hi


# Simulate reachability. For each column, we store the set of (row, v) that are
# feasible. v at a point = v_pre + a*unit_t where a estimated via the 2x formula.
# Track union of reachable rows and min/max via GetRowRange from feasible pts.
dim_t = int(15.0 / unit_t) + 1

# column 0
col0 = {0: init_v}  # row -> v
feasible = {0: {0: init_v}}  # col -> {row: v}

print(f"t=0: feasible rows {list(col0.keys())}")
for c in range(1, dim_t):
    prev = feasible[c - 1]
    cur = {}
    # For every feasible prev row, compute reachable cur rows via GetRowRange
    for r_pre, v_pre in prev.items():
        s_pre = nodes[r_pre]
        lo, hi = get_row_range(s_pre, v_pre)
        for r in range(lo, hi + 1):
            s_cur = nodes[r]
            if in_stop(s_cur):
                continue
            # acc via the EXACT code formula (2x!):
            # curr_a = 2 * ((s_cur - s_pre)/unit_t - v_pre) / unit_t
            v_cur = (s_cur - s_pre) / unit_t
            a = 2 * (v_cur - v_pre) / unit_t
            if a < max_dec or a > max_acc:
                continue
            # backward motion check
            if v_cur < -1e-6:
                continue
            # keep best (largest v) to widen range; but for reachability all ok
            if r not in cur or v_cur > cur[r]:
                cur[r] = v_cur
    if not cur:
        print(f"t={c}: *** COLUMN BREAK - no feasible rows ***")
        break
    feasible[c] = cur
    rows = sorted(cur)
    print(f"t={c}: feasible s=[{nodes[rows[0]]:.2f},{nodes[rows[-1]]:.2f}] "
          f"count={len(rows)} maxv={max(cur.values()):.1f}")

# final column
last = feasible.get(dim_t - 1, {})
print(f"\nt=15 feasible rows: {len(last)}")
if last:
    rs = sorted(last)
    print(f"  s range [{nodes[rs[0]]:.2f},{nodes[rs[-1]]:.2f}]")
