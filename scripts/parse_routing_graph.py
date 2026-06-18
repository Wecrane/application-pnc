#!/usr/bin/env python3
"""Parse routing_map.bin topology and extract routing information.

The routing_map.bin is a protobuf TopoGraph with:
- Field 1: map version string
- Field 3: lane nodes (ID, geometry, road ID)
- Field 4: lane edges (from_lane, to_lane, weight, edge_type)

Edge types:
  0 = TET_VIRTUAL (virtual connection, weight=0)
  1 = TET_FORWARD (driving forward, weight=distance)
  2 = TET_LEFT/RIGHT (turns, weight may vary)
"""

import re
import json
import struct
import sys
from collections import defaultdict, deque

def parse_routing_raw(raw_file):
    """Parse protoc --decode_raw output of routing_map.bin."""
    with open(raw_file, 'r') as f:
        content = f.read()
    
    result = {
        'map_version': '',
        'nodes': [],
        'edges': [],
    }
    
    # Extract map version
    ver_match = re.search(r'^1: "(.+)"', content, re.MULTILINE)
    if ver_match:
        result['map_version'] = ver_match.group(1)
    
    # Extract lane nodes (field 3) - more robust parsing
    # Each node starts with "\n3 {\n  1: \"Lane_XXX\"" and ends before next "\n3 {" or "\n4 {"
    # Road ID is at "  8: \"Road_XXX\""
    node_blocks = re.split(r'\n(?=3 \{\n|\n4 \{)', content)
    
    for block in node_blocks:
        if not block.strip().startswith('3 {'):
            continue
        
        lane_match = re.search(r'1: "(Lane_\d+)"', block)
        road_match = re.search(r'8: "(Road_\d+)"', block)
        
        if lane_match and road_match:
            result['nodes'].append({
                'lane_id': lane_match.group(1),
                'road_id': road_match.group(1),
            })
    
    # Extract lane edges (field 4)
    edge_pattern = re.compile(
        r'^4 \{\n  1: "(Lane_\d+)"\n  2: "(Lane_\d+)"\n  3: (0x[0-9a-f]+)\n  4: (\d+)',
        re.MULTILINE
    )
    
    for match in edge_pattern.finditer(content):
        from_lane = match.group(1)
        to_lane = match.group(2)
        weight_hex = match.group(3)
        edge_type = int(match.group(4))
        weight = int(weight_hex, 16)
        
        # Convert 64-bit hex to double for non-zero weights
        weight_f = struct.unpack('d', struct.pack('Q', weight))[0] if weight != 0 else 0.0
        
        result['edges'].append({
            'from': from_lane,
            'to': to_lane,
            'weight': round(weight_f, 4),
            'type': edge_type,
        })
    
    return result


def build_routing_graph(routing_data):
    """Build routing graph from nodes and edges."""
    nodes = routing_data['nodes']
    edges = routing_data['edges']
    
    # Build indices
    lane_to_road = {}
    road_to_lanes = defaultdict(list)
    for node in nodes:
        lid = node['lane_id']
        rid = node['road_id']
        lane_to_road[lid] = rid
        road_to_lanes[rid].append(lid)
    
    # Build adjacency lists
    outgoing = defaultdict(list)
    incoming = defaultdict(list)
    
    for edge in edges:
        outgoing[edge['from']].append(edge)
        incoming[edge['to']].append(edge)
    
    # Classify edges by type
    virtual_edges = [e for e in edges if e['type'] == 0]
    forward_edges = [e for e in edges if e['type'] == 1]
    other_edges = [e for e in edges if e['type'] == 2]
    
    # Find start/end lanes (based on forward edges)
    lanes_with_incoming = set(e['to'] for e in forward_edges)
    lanes_with_outgoing = set(e['from'] for e in forward_edges)
    all_lanes = set(n['lane_id'] for n in nodes)
    
    start_lanes = sorted(all_lanes - lanes_with_incoming)
    end_lanes = sorted(all_lanes - lanes_with_outgoing)
    
    # Find dead-end lanes
    dead_end_lanes = sorted(set(all_lanes) - set(outgoing.keys()))
    
    # Build forward chains using ALL edge types
    all_edges_graph = defaultdict(list)
    for edge in edges:
        all_edges_graph[edge['from']].append(edge)
    
    def trace_full_chain(start_lane, visited=None):
        """Trace a chain using ALL edge types."""
        if visited is None:
            visited = set()
        chain = []
        current = start_lane
        while current and current not in visited:
            visited.add(current)
            chain.append(current)
            outs = all_edges_graph.get(current, [])
            # Prefer type 1 > type 2 > type 0
            type1 = [e for e in outs if e['type'] == 1]
            type2 = [e for e in outs if e['type'] == 2]
            type0 = [e for e in outs if e['type'] == 0]
            
            if len(type1) == 1:
                current = type1[0]['to']
            elif len(type2) == 1:
                current = type2[0]['to']
            elif len(type1) == 0 and len(type2) == 0 and len(type0) == 1:
                current = type0[0]['to']
            else:
                break
        return chain, visited
    
    all_visited = set()
    full_chains = []
    for sl in start_lanes:
        chain, visited = trace_full_chain(sl, all_visited)
        all_visited.update(visited)
        if len(chain) > 1:
            full_chains.append(chain)
    
    unvisited = all_lanes - all_visited
    for ul in sorted(unvisited):
        chain, visited = trace_full_chain(ul, all_visited)
        all_visited.update(visited)
        if len(chain) > 1:
            full_chains.append(chain)
    
    return {
        'lane_to_road': lane_to_road,
        'road_to_lanes': road_to_lanes,
        'outgoing': {k: v for k, v in outgoing.items()},
        'incoming': {k: v for k, v in incoming.items()},
        'start_lanes': start_lanes,
        'end_lanes': end_lanes,
        'dead_end_lanes': dead_end_lanes,
        'chains': full_chains,
        'virtual_edges': virtual_edges,
        'forward_edges': forward_edges,
        'other_edges': other_edges,
        'all_lanes': sorted(all_lanes),
    }


def analyze_graph(graph):
    """Comprehensive routing graph analysis."""
    analysis = {}
    
    analysis['num_lanes_routable'] = len(graph['all_lanes'])
    analysis['num_start_lanes'] = len(graph['start_lanes'])
    analysis['num_end_lanes'] = len(graph['end_lanes'])
    analysis['num_dead_end_lanes'] = len(graph['dead_end_lanes'])
    analysis['num_chains'] = len(graph['chains'])
    analysis['num_virtual_edges'] = len(graph['virtual_edges'])
    analysis['num_forward_edges'] = len(graph['forward_edges'])
    analysis['num_other_edges'] = len(graph['other_edges'])
    
    # Chain analysis
    chain_lengths = [(len(c), c[0], c[-1]) for c in graph['chains']]
    chain_lengths.sort(reverse=True)
    analysis['longest_chains'] = [
        {'length': l, 'start': s, 'end': e}
        for l, s, e in chain_lengths[:20]
    ]
    
    # Road analysis
    road_lane_counts = {rid: len(lanes) for rid, lanes in graph['road_to_lanes'].items()}
    top_roads = sorted(road_lane_counts.items(), key=lambda x: x[1], reverse=True)
    analysis['roads_by_lane_count'] = top_roads[:30]
    
    # High-connectivity lanes
    lane_conn = {}
    for lid in graph['all_lanes']:
        out_cnt = len(graph['outgoing'].get(lid, []))
        in_cnt = len(graph['incoming'].get(lid, []))
        lane_conn[lid] = out_cnt + in_cnt
    
    top_connected = sorted(lane_conn.items(), key=lambda x: x[1], reverse=True)
    analysis['most_connected_lanes'] = top_connected[:20]
    
    # Cross-road transitions (all edge types)
    edge_types_data = defaultdict(list)
    all_graph_edges = graph['forward_edges'] + graph['virtual_edges'] + graph['other_edges']
    for e in all_graph_edges:
        fl = e['from']
        tl = e['to']
        w = e['weight']
        from_road = graph['lane_to_road'].get(fl, '?')
        to_road = graph['lane_to_road'].get(tl, '?')
        if from_road != to_road:
            edge_types_data['cross_road'].append({
                'from': fl, 'to': tl, 'weight': w,
                'from_road': from_road, 'to_road': to_road,
                'edge_type': e['type'],
            })
    
    analysis['cross_road_edges'] = edge_types_data.get('cross_road', [])
    
    # U-turn candidates
    uturn_candidates = []
    for lid in graph['all_lanes']:
        outs = graph['outgoing'].get(lid, [])
        if len(outs) >= 2:
            weights = [e['weight'] for e in outs if e['type'] == 1]
            if len(weights) >= 2:
                max_w = max(weights)
                min_w = min(weights)
                if max_w > 0 and max_w / max(min_w, 0.001) > 2:
                    uturn_candidates.append({
                        'lane': lid,
                        'max_weight': max_w,
                        'min_weight': min_w,
                        'ratio': max_w / max(min_w, 0.001),
                    })
    analysis['uturn_candidates'] = sorted(uturn_candidates, key=lambda x: x['ratio'], reverse=True)[:30]
    
    return analysis


def main():
    raw_file = '/home/skye/application-pnc/output/routing_map_raw.txt'
    output_dir = '/home/skye/application-pnc/output'
    
    print("[INFO] Parsing routing map raw protobuf...")
    routing_data = parse_routing_raw(raw_file)
    
    print(f"[INFO] Map version: {routing_data['map_version']}")
    print(f"[INFO] Nodes: {len(routing_data['nodes'])}")
    print(f"[INFO] Edges: {len(routing_data['edges'])}")
    
    print("[INFO] Building routing graph...")
    graph = build_routing_graph(routing_data)
    
    print("[INFO] Analyzing graph...")
    analysis = analyze_graph(graph)
    
    # Save graph
    graph_output = {
        'map_version': routing_data['map_version'],
        'num_nodes': len(routing_data['nodes']),
        'num_edges': len(routing_data['edges']),
        'nodes': routing_data['nodes'],
        'edges': routing_data['edges'],
        'start_lanes': graph['start_lanes'],
        'end_lanes': graph['end_lanes'],
        'chains': graph['chains'],
        'analysis': analysis,
    }
    
    graph_path = f"{output_dir}/routing_graph.json"
    with open(graph_path, 'w') as f:
        json.dump(graph_output, f, indent=2, ensure_ascii=False)
    print(f"[DONE] Routing graph saved to: {graph_path}")
    
    # Print summary
    print("\n" + "=" * 60)
    print("  ROUTING GRAPH ANALYSIS SUMMARY")
    print("=" * 60)
    print(f"  Map version: {routing_data['map_version']}")
    print(f"  Routable lanes: {analysis['num_lanes_routable']}")
    print(f"  Start lanes: {analysis['num_start_lanes']}")
    print(f"  End lanes: {analysis['num_end_lanes']}")
    print(f"  Forward edges: {analysis['num_forward_edges']}")
    print(f"  Virtual edges: {analysis['num_virtual_edges']}")
    print(f"  Other edges: {analysis['num_other_edges']}")
    print(f"  Chains: {analysis['num_chains']}")
    print(f"  Cross-road transitions: {len(analysis['cross_road_edges'])}")
    
    print(f"\n  Top roads by lane count:")
    for rid, cnt in analysis['roads_by_lane_count'][:10]:
        print(f"    {rid}: {cnt} lanes")
    
    print(f"\n  Longest chains:")
    for c in analysis['longest_chains'][:10]:
        print(f"    {c['length']} lanes: {c['start']} -> {c['end']}")
    
    print(f"\n  Top U-Turn candidates:")
    for uc in analysis['uturn_candidates'][:10]:
        print(f"    {uc['lane']}: max_w={uc['max_weight']:.1f}, min_w={uc['min_weight']:.1f}, ratio={uc['ratio']:.1f}")
    
    print("=" * 60)


if __name__ == '__main__':
    main()
