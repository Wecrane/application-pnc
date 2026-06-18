#!/usr/bin/env python3
"""Apollo HD Map Parser - 2026 Contest Map Analysis

Parses base_map.bin and routing_map.bin to extract:
- Road/Lane topology
- Junctions, signals, stop signs, crosswalks
- Lane turn types, speed limits
- Routing graph topology
"""

import sys
import os
import json

# Add bazel cache paths for protobuf imports
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin/external/apollo_src')
sys.path.insert(0, '/home/skye/application-pnc/.cache/bazel/679551712d2357b63e6e0ce858ebf90e/execroot/application-pnc/bazel-out/k8-opt/bin')

from modules.common_msgs.map_msgs import map_pb2
from modules.common_msgs.map_msgs import map_lane_pb2
from modules.common_msgs.map_msgs import map_road_pb2
from modules.common_msgs.map_msgs import map_junction_pb2
from modules.common_msgs.map_msgs import map_signal_pb2
from modules.common_msgs.map_msgs import map_stop_sign_pb2
from modules.common_msgs.map_msgs import map_crosswalk_pb2
from modules.common_msgs.map_msgs import map_speed_bump_pb2
from modules.common_msgs.map_msgs import map_yield_sign_pb2
from modules.common_msgs.map_msgs import map_parking_space_pb2
from modules.common_msgs.map_msgs import map_overlap_pb2
from modules.common_msgs.map_msgs import map_clear_area_pb2
from modules.common_msgs.map_msgs import map_pnc_junction_pb2

# LaneTurn enum reverse mapping (from map_lane.proto)
LANE_TURN_MAP = {
    1: 'NO_TURN',
    2: 'LEFT_TURN',
    3: 'RIGHT_TURN',
    4: 'U_TURN',
}

LANE_DIRECTION_MAP = {
    1: 'FORWARD',
    2: 'BACKWARD',
    3: 'BIDIRECTION',
}

JUNCTION_TYPE_MAP = {
    0: 'UNKNOWN',
    1: 'IN_ROAD',
    2: 'CROSS_ROAD',
    3: 'FORK_ROAD',
    4: 'MAIN_SIDE',
    5: 'DEAD_END',
}

ROAD_TYPE_MAP = {
    0: 'UNKNOWN',
    1: 'HIGHWAY',
    2: 'CITY_ROAD',
    3: 'PARK',
}

LANE_TYPE_MAP = {
    1: 'NONE',
    2: 'CITY_DRIVING',
    3: 'BIKING',
    4: 'SIDEWALK',
    5: 'PARKING',
    6: 'SHOULDER',
    7: 'SHARED',
}


def parse_base_map(filepath):
    """Parse base_map.bin and extract all map elements."""
    print(f"[INFO] Parsing base_map: {filepath}")
    
    with open(filepath, 'rb') as f:
        data = f.read()
    
    hdmap = map_pb2.Map()
    hdmap.ParseFromString(data)
    
    result = {
        'header': {},
        'lanes': [],
        'roads': [],
        'junctions': [],
        'signals': [],
        'stop_signs': [],
        'crosswalks': [],
        'speed_bumps': [],
        'yield_signs': [],
        'parking_spaces': [],
        'clear_areas': [],
        'pnc_junctions': [],
        'overlaps': [],
        'summary': {},
    }
    
    # Header
    if hdmap.HasField('header'):
        h = hdmap.header
        result['header'] = {
            'version': h.version.decode('utf-8', errors='replace') if h.version else '',
            'date': h.date.decode('utf-8', errors='replace') if h.date else '',
            'district': h.district.decode('utf-8', errors='replace') if h.district else '',
            'vendor': h.vendor.decode('utf-8', errors='replace') if h.vendor else '',
            'left': h.left if h.HasField('left') else None,
            'top': h.top if h.HasField('top') else None,
            'right': h.right if h.HasField('right') else None,
            'bottom': h.bottom if h.HasField('bottom') else None,
        }
    
    # Lanes
    for lane in hdmap.lane:
        lane_info = {
            'id': lane.id.id if lane.HasField('id') and lane.id.HasField('id') else '',
            'turn': LANE_TURN_MAP.get(lane.turn, f'UNKNOWN_{lane.turn}'),
            'direction': LANE_DIRECTION_MAP.get(lane.direction, f'UNKNOWN_{lane.direction}'),
            'speed_limit': lane.speed_limit if lane.HasField('speed_limit') else None,
            'length': lane.length if lane.HasField('length') else None,
            'type': LANE_TYPE_MAP.get(lane.type, f'UNKNOWN_{lane.type}') if lane.HasField('type') else None,
        }
        # Predecessor lanes
        lane_info['predecessor_ids'] = [p.id for p in lane.predecessor_id if p.HasField('id')]
        # Successor lanes
        lane_info['successor_ids'] = [s.id for s in lane.successor_id if s.HasField('id')]
        # Left/right neighbor lanes
        lane_info['left_neighbor_ids'] = []
        lane_info['right_neighbor_ids'] = []
        for nl in lane.left_neighbor_forward_lane_id:
            if nl.HasField('id'):
                lane_info['left_neighbor_ids'].append(nl.id)
        for nr in lane.right_neighbor_forward_lane_id:
            if nr.HasField('id'):
                lane_info['right_neighbor_ids'].append(nr.id)
        # Central curve points count
        if lane.HasField('central_curve'):
            lane_info['central_curve_segments'] = len(lane.central_curve.segment)
        result['lanes'].append(lane_info)
    
    # Roads
    for road in hdmap.road:
        road_info = {
            'id': road.id.id if road.HasField('id') and road.id.HasField('id') else '',
            'type': ROAD_TYPE_MAP.get(road.type, f'UNKNOWN_{road.type}') if road.HasField('type') else None,
        }
        # Sections and lane IDs
        section_lanes = []
        for section in road.section:
            for lane_id_obj in section.lane_id:
                if lane_id_obj.HasField('id'):
                    section_lanes.append(lane_id_obj.id)
        road_info['lane_ids'] = section_lanes
        # Junction ID
        if road.HasField('junction_id') and road.junction_id.HasField('id'):
            road_info['junction_id'] = road.junction_id.id
        result['roads'].append(road_info)
    
    # Junctions
    for junction in hdmap.junction:
        junc_info = {
            'id': junction.id.id if junction.HasField('id') and junction.id.HasField('id') else '',
            'type': JUNCTION_TYPE_MAP.get(junction.type, f'UNKNOWN_{junction.type}') if junction.HasField('type') else None,
        }
        # Overlap IDs
        junc_info['overlap_ids'] = [o.id for o in junction.overlap_id if o.HasField('id')]
        result['junctions'].append(junc_info)
    for signal in hdmap.signal:
        sig_info = {
            'id': signal.id.id if signal.HasField('id') and signal.id.HasField('id') else '',
            'type': signal.type if signal.HasField('type') else None,
        }
        # Sub-signals
        sig_info['subsignals'] = []
        for sub in signal.subsignal:
            sub_info = {
                'id': sub.id.id if sub.HasField('id') and sub.id.HasField('id') else '',
                'type': sub.type if sub.HasField('type') else None,
            }
            sig_info['subsignals'].append(sub_info)
        # Overlap IDs
        sig_info['overlap_ids'] = [o.id for o in signal.overlap_id if o.HasField('id')]
        # Stop line count
        sig_info['num_stop_lines'] = len(signal.stop_line)
        result['signals'].append(sig_info)
    
    # Stop Signs
    for ss in hdmap.stop_sign:
        ss_info = {
            'id': ss.id.id if ss.HasField('id') and ss.id.HasField('id') else '',
            'type': ss.type if ss.HasField('type') else None,
            'overlap_ids': [o.id for o in ss.overlap_id if o.HasField('id')],
            'num_stop_lines': len(ss.stop_line),
        }
        result['stop_signs'].append(ss_info)
    
    # Crosswalks
    for cw in hdmap.crosswalk:
        cw_info = {
            'id': cw.id.id if cw.HasField('id') and cw.id.HasField('id') else '',
            'overlap_ids': [o.id for o in cw.overlap_id if o.HasField('id')],
        }
        result['crosswalks'].append(cw_info)
    
    # Speed Bumps
    for sb in hdmap.speed_bump:
        sb_info = {
            'id': sb.id.id if sb.HasField('id') and sb.id.HasField('id') else '',
            'overlap_ids': [o.id for o in sb.overlap_id if o.HasField('id')],
        }
        result['speed_bumps'].append(sb_info)
    
    # Yield Signs (field name 'yield' is a Python keyword, use getattr)
    for ys in getattr(hdmap, 'yield'):
        ys_info = {
            'id': ys.id.id if ys.HasField('id') and ys.id.HasField('id') else '',
            'overlap_ids': [o.id for o in ys.overlap_id if o.HasField('id')],
        }
        result['yield_signs'].append(ys_info)
    
    # Parking Spaces
    for ps in hdmap.parking_space:
        ps_info = {
            'id': ps.id.id if ps.HasField('id') and ps.id.HasField('id') else '',
            'overlap_ids': [o.id for o in ps.overlap_id if o.HasField('id')],
        }
        result['parking_spaces'].append(ps_info)
    
    # Clear Areas
    for ca in hdmap.clear_area:
        ca_info = {
            'id': ca.id.id if ca.HasField('id') and ca.id.HasField('id') else '',
            'overlap_ids': [o.id for o in ca.overlap_id if o.HasField('id')],
        }
        result['clear_areas'].append(ca_info)
    
    # PNC Junctions
    for pncj in hdmap.pnc_junction:
        pncj_info = {
            'id': pncj.id.id if pncj.HasField('id') and pncj.id.HasField('id') else '',
            'overlap_ids': [o.id for o in pncj.overlap_id if o.HasField('id')],
        }
        result['pnc_junctions'].append(pncj_info)
    
    # Overlaps
    for ov in hdmap.overlap:
        ov_info = {
            'id': ov.id.id if ov.HasField('id') and ov.id.HasField('id') else '',
        }
        objects = []
        for obj in ov.object:
            obj_info = {
                'id': obj.id.id if obj.HasField('id') and obj.id.HasField('id') else '',
            }
            if obj.HasField('lane_overlap_info'):
                oi = obj.lane_overlap_info
                obj_info['start_s'] = oi.start_s
                obj_info['end_s'] = oi.end_s
            objects.append(obj_info)
        ov_info['objects'] = objects
        result['overlaps'].append(ov_info)
    
    # Summary
    result['summary'] = {
        'num_lanes': len(result['lanes']),
        'num_roads': len(result['roads']),
        'num_junctions': len(result['junctions']),
        'num_signals': len(result['signals']),
        'num_stop_signs': len(result['stop_signs']),
        'num_crosswalks': len(result['crosswalks']),
        'num_speed_bumps': len(result['speed_bumps']),
        'num_yield_signs': len(result['yield_signs']),
        'num_parking_spaces': len(result['parking_spaces']),
        'num_clear_areas': len(result['clear_areas']),
        'num_pnc_junctions': len(result['pnc_junctions']),
        'num_overlaps': len(result['overlaps']),
    }
    
    return result


def analyze_turn_types(lanes):
    """Analyze lane turn type distribution."""
    turn_counts = {}
    for lane in lanes:
        t = lane.get('turn', 'UNKNOWN')
        turn_counts[t] = turn_counts.get(t, 0) + 1
    return turn_counts


def analyze_speed_limits(lanes):
    """Analyze speed limit distribution."""
    speeds = [l.get('speed_limit') for l in lanes if l.get('speed_limit')]
    if not speeds:
        return {}
    return {
        'min': min(speeds),
        'max': max(speeds),
        'avg': sum(speeds) / len(speeds),
        'unique': sorted(set(speeds)),
    }


def build_lane_connectivity(lanes):
    """Build lane connectivity graph (predecessor/successor)."""
    lane_map = {l['id']: l for l in lanes}
    
    # Count lanes by connectivity
    with_pred = sum(1 for l in lanes if l['predecessor_ids'])
    with_succ = sum(1 for l in lanes if l['successor_ids'])
    with_neighbor = sum(1 for l in lanes if l['left_neighbor_ids'] or l['right_neighbor_ids'])
    
    # Find special lanes
    start_lanes = [l['id'] for l in lanes if not l['predecessor_ids'] and l['successor_ids']]
    end_lanes = [l['id'] for l in lanes if l['predecessor_ids'] and not l['successor_ids']]
    isolated_lanes = [l['id'] for l in lanes if not l['predecessor_ids'] and not l['successor_ids']]
    
    # Find U-turn lanes
    uturn_lanes = [l for l in lanes if l.get('turn') == 'U_TURN']
    
    return {
        'total': len(lanes),
        'with_predecessors': with_pred,
        'with_successors': with_succ,
        'with_neighbors': with_neighbor,
        'start_lanes': start_lanes,
        'end_lanes': end_lanes,
        'isolated_lanes': isolated_lanes,
        'uturn_lanes': [l['id'] for l in uturn_lanes],
    }


def find_overlap_mappings(result):
    """Build lane-to-signal, lane-to-crosswalk, lane-to-stop_sign mappings from overlaps."""
    # Build overlap index
    overlap_map = {}
    for ov in result['overlaps']:
        overlap_map[ov['id']] = ov
    
    # Find lane-signal overlaps
    signal_lanes = set()
    for ov in result['overlaps']:
        has_signal = False
        has_lane = False
        lane_id = None
        signal_id = None
        for obj in ov['objects']:
            oid = obj['id']
            if oid.startswith('Lane_'):
                has_lane = True
                lane_id = oid
            elif oid.startswith('Signal_'):
                has_signal = True
                signal_id = oid
        if has_signal and has_lane:
            signal_lanes.add((signal_id, lane_id))
    
    # Find lane-stop_sign overlaps
    stop_lanes = set()
    for ov in result['overlaps']:
        has_stop = False
        has_lane = False
        lane_id = None
        stop_id = None
        for obj in ov['objects']:
            oid = obj['id']
            if oid.startswith('Lane_'):
                has_lane = True
                lane_id = oid
            elif oid.startswith('StopSign_'):
                has_stop = True
                stop_id = oid
        if has_stop and has_lane:
            stop_lanes.add((stop_id, lane_id))
    
    # Find lane-crosswalk overlaps
    crosswalk_lanes = set()
    for ov in result['overlaps']:
        has_cw = False
        has_lane = False
        lane_id = None
        cw_id = None
        for obj in ov['objects']:
            oid = obj['id']
            if oid.startswith('Lane_'):
                has_lane = True
                lane_id = oid
            elif oid.startswith('Crosswalk_'):
                has_cw = True
                cw_id = oid
        if has_cw and has_lane:
            crosswalk_lanes.add((cw_id, lane_id))
    
    return {
        'signal_lane_pairs': sorted(signal_lanes),
        'stop_sign_lane_pairs': sorted(stop_lanes),
        'crosswalk_lane_pairs': sorted(crosswalk_lanes),
    }


def main():
    map_dir = '/home/skye/application-pnc/data/map_data/Xh_2026_contest'
    output_dir = '/home/skye/application-pnc/output'
    
    # Parse base map
    result = parse_base_map(os.path.join(map_dir, 'base_map.bin'))
    
    # Additional analysis
    turn_analysis = analyze_turn_types(result['lanes'])
    speed_analysis = analyze_speed_limits(result['lanes'])
    connectivity = build_lane_connectivity(result['lanes'])
    overlap_mappings = find_overlap_mappings(result)
    
    # Build comprehensive output
    output = {
        'map_info': result['header'],
        'summary': result['summary'],
        'turn_analysis': turn_analysis,
        'speed_analysis': speed_analysis,
        'lane_connectivity': connectivity,
        'overlap_mappings_summary': {
            'num_signal_lane_pairs': len(overlap_mappings['signal_lane_pairs']),
            'num_stop_sign_lane_pairs': len(overlap_mappings['stop_sign_lane_pairs']),
            'num_crosswalk_lane_pairs': len(overlap_mappings['crosswalk_lane_pairs']),
        },
        # Detailed per-element lists (IDs only for compactness, details on request)
        'junction_ids': [j['id'] for j in result['junctions']],
        'signal_ids': [s['id'] for s in result['signals']],
        'stop_sign_ids': [s['id'] for s in result['stop_signs']],
        'crosswalk_ids': sorted([c['id'] for c in result['crosswalks']], key=lambda x: int(x.split('_')[1]) if '_' in x else 0),
        'speed_bump_ids': [s['id'] for s in result['speed_bumps']],
        'yield_sign_ids': [y['id'] for y in result['yield_signs']],
        'parking_space_ids': sorted([p['id'] for p in result['parking_spaces']], key=lambda x: int(x.split('_')[1]) if '_' in x else 0),
        
        # Detailed lane info (selected fields)
        'lane_details': [
            {
                'id': l['id'],
                'turn': l['turn'],
                'speed_limit': l['speed_limit'],
                'num_predecessors': len(l['predecessor_ids']),
                'num_successors': len(l['successor_ids']),
                'has_neighbors': bool(l['left_neighbor_ids'] or l['right_neighbor_ids']),
            }
            for l in result['lanes']
        ],
        
        # Road details
        'road_details': [
            {
                'id': r['id'],
                'num_lanes': len(r['lane_ids']),
                'junction_id': r.get('junction_id'),
            }
            for r in result['roads']
        ],
        
        # Junction details
        'junction_details': [
            {
                'id': j['id'],
                'type': j['type'],
                'num_overlaps': len(j.get('overlap_ids', [])),
            }
            for j in result['junctions']
        ],
        
        # Signal details (with sub-signals)
        'signal_details': [
            {
                'id': s['id'],
                'type': s['type'],
                'num_subsignals': len(s['subsignals']),
            }
            for s in result['signals']
        ],
        
        # Full overlap mappings
        'overlap_mappings': overlap_mappings,
    }
    
    # Save full dump for detailed analysis
    full_output_path = os.path.join(output_dir, 'map_analysis_full.json')
    with open(full_output_path, 'w') as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print(f"[DONE] Full analysis saved to: {full_output_path}")
    
    # Save compact summary
    summary_output_path = os.path.join(output_dir, 'map_analysis_summary.json')
    summary = {
        k: v for k, v in output.items()
        if k not in ['lane_details', 'road_details', 'overlap_mappings', 'junction_details', 'signal_details']
    }
    with open(summary_output_path, 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"[DONE] Summary saved to: {summary_output_path}")
    
    # Print key stats
    print("\n" + "=" * 60)
    print("  2026 Contest Map Analysis Summary")
    print("=" * 60)
    print(f"  Map Version: {output['map_info'].get('version', 'N/A')}")
    print(f"  Map District: {output['map_info'].get('district', 'N/A')}")
    print(f"  Bounds: ({output['map_info'].get('left')}, {output['map_info'].get('top')}) -> ({output['map_info'].get('right')}, {output['map_info'].get('bottom')})")
    print(f"  Total Lanes: {output['summary']['num_lanes']}")
    print(f"  Total Roads: {output['summary']['num_roads']}")
    print(f"  Total Junctions: {output['summary']['num_junctions']}")
    print(f"  Total Signals (Traffic Lights): {output['summary']['num_signals']}")
    print(f"  Total Stop Signs: {output['summary']['num_stop_signs']}")
    print(f"  Total Crosswalks: {output['summary']['num_crosswalks']}")
    print(f"  Total Speed Bumps: {output['summary']['num_speed_bumps']}")
    print(f"  Total Yield Signs: {output['summary']['num_yield_signs']}")
    print(f"  Total Parking Spaces: {output['summary']['num_parking_spaces']}")
    print(f"  Total Clear Areas: {output['summary']['num_clear_areas']}")
    print(f"  Total PNC Junctions: {output['summary']['num_pnc_junctions']}")
    print(f"  Total Overlaps: {output['summary']['num_overlaps']}")
    print(f"")
    print(f"  Lane Turn Distribution: {output['turn_analysis']}")
    print(f"  Speed Limit Range: {output['speed_analysis']}")
    print(f"  U-Turn Lanes: {output['lane_connectivity']['uturn_lanes']}")
    print(f"  Start Lanes (no predecessor): {len(output['lane_connectivity']['start_lanes'])}")
    print(f"  End Lanes (no successor): {len(output['lane_connectivity']['end_lanes'])}")
    print(f"  Lanes with Neighbors: {output['lane_connectivity']['with_neighbors']}")
    print("=" * 60)


if __name__ == '__main__':
    main()
