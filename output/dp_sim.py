#!/usr/bin/env python3
"""Simulate GriddedPathTimeGraph DP row-range propagation to find why
'Retrieve best speed profile failed' happens when approaching a STOP fence.
"""
import bisect

# ---- DP config (from dp_st_speed_optimizer default_conf.pb.txt) ----
unit_t = 1.0
total_time = 15.0
dense_unit_s = 0.1
dense_dimension_s = 101
sparse_unit_s = 1.0
max_acc = 3.0      # config max_acceleration
max_dec = -6.0     # config max_deceleration
acc_coeff = 0.5    # GetRowRange scaling

# ---- scenario from 19:20:28.44 log ----
init_v = 10.68
init_a = 0.0
init_s = 56.3531           # adc_end
stop_s = 82.2285           # STOP fence
path_length = 176.7        # ref len ~ DEST

# ---- build spatial_distance_by_index_ ----
nodes = []
s = 0.0
while s < 10.0 - 1e-6:
    nodes.append(round(s, 6))
    s += dense_unit_s
s = 10.0
while s <= path_length + 1e-6:
    nodes.append(round(s, 6))
    s += sparse_unit_s
print(f"total s nodes: {len(nodes)}, s range [{nodes[0]},{nodes[-1]}]")

dim_t = int(total_time / unit_t) + 1  # 16 cols (t=0..15)
dim_s = len(nodes)


def row_index_of_s(sv):
    return bisect.bisect_left(nodes, sv)


def get_row_range(s_cur, v0):
    """GetRowRange: next col reachable row index range [low, high]."""
    t_sq = unit_t * unit_t
    s_upper = s_cur + v0 * unit_t + acc_coeff * max_acc * t_sq
    s_lower = max(0.0, s_cur + v0 * unit_t + acc_coeff * max_dec * t_sq)
    hi = row_index_of_s(s_upper)
    lo = row_index_of_s(s_lower)
    if hi >= dim_s:
        hi = dim_s - 1
    return lo, hi


# ---- propagate row ranges column by column (upper bound of reachability) ----
# We track the *range of reachable rows* assuming acceleration feasible and
# ignoring cost. Also track whether the column has any reachable row that is
# NOT inside STOP boundary (the only hard filter is CheckOverlap: crossing the
# STOP boundary is forbidden).
def in_stop(sv):
    return sv >= stop_s - 1e-9


next_lo, next_hi = 0, 0
print(f"\nt=0: reachable rows s=[{nodes[0]},{nodes[0]}] v={init_v}")
for c in range(1, dim_t):
    count = next_hi - next_lo + 1
    if count <= 0:
        print(f"t={c}: BREAK - no reachable rows from previous col -> all inf")
        break
    # for each reachable row in [next_lo, next_hi], the reachable next range
    # is driven by GetRowRange (using that point's v).
    # Approx: assume v at a row ~ (s - prev_s)/unit_t; worst case propagate union.
    reachable = []
    for r in range(next_lo, next_hi + 1):
        sv = nodes[r]
        if in_stop(sv):
            continue  # s inside STOP -> point inf (crossing filtered)
        # v estimate = avg over the range we had
        # For range propagation, use the max v in previous col to widen, but
        # the actual v is (s_cur - s_prev)/unit_t. Simulate with bounds:
        v_est = (sv - nodes[next_lo]) / unit_t if next_lo > 0 or c > 1 else (sv - init_s + init_s)  # placeholder
        # better: v_est from previous column's avg speed
        lo, hi = get_row_range(sv, v_est)
        reachable.append((lo, hi))
    if not reachable:
        print(f"t={c}: BREAK - no non-STOP reachable rows -> all inf")
        break
    new_lo = min(lo for lo, _ in reachable)
    new_hi = max(hi for _, hi in reachable)
    print(f"t={c}: reachable s range [{nodes[new_lo]:.2f}, {nodes[new_hi]:.2f}] "
          f"(non-stop rows used: {len(reachable)})")
    next_lo, next_hi = new_lo, new_hi
    if next_hi - next_lo < 0:
        break

print("\nDONE")
