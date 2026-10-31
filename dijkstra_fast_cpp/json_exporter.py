"""
json_exporter.py: Serializes shortest path results, graph topology,
and benchmark timings to standard structured JSON format.
"""

import json
from typing import List, Dict, Any, Optional
from datetime import datetime


def export_path_to_json(
    network_nodes: List[Dict[str, Any]],
    network_edges: List[tuple],
    path_node_ids: List[int],
    total_distance: float,
    waypoints: List[int],
    timing_info: Optional[Dict[str, Any]] = None,
    output_filepath: Optional[str] = None
) -> str:
    """
    Exports the computed shortest path and graph details to a JSON structure.

    Parameters:
        network_nodes: All graph nodes [{'id': int, 'x': float, 'y': float}]
        network_edges: All graph edges [(u, v, weight)]
        path_node_ids: Ordered list of node IDs on the shortest path
        total_distance: Total path length
        waypoints: List of target waypoint node IDs covered
        timing_info: Optional dictionary with benchmark times (python_ms, cython_ms, speedup)
        output_filepath: Optional file path to write the JSON to

    Returns:
        str: Formatted JSON string
    """
    node_coord_map = {n['id']: (n['x'], n['y']) for n in network_nodes}

    # Build sequence of path nodes with coordinates
    path_nodes = []
    for node_id in path_node_ids:
        coords = node_coord_map.get(node_id, (0.0, 0.0))
        path_nodes.append({
            "node_id": node_id,
            "x": coords[0],
            "y": coords[1],
            "is_waypoint": node_id in waypoints
        })

    # Build sequence of path edges with distances
    path_edges = []
    for i in range(len(path_node_ids) - 1):
        u = path_node_ids[i]
        v = path_node_ids[i + 1]
        p1 = node_coord_map.get(u, (0.0, 0.0))
        p2 = node_coord_map.get(v, (0.0, 0.0))
        w = ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)**0.5
        path_edges.append({
            "step": i + 1,
            "from_node": u,
            "to_node": v,
            "distance": round(w, 4)
        })

    payload = {
        "export_metadata": {
            "timestamp": datetime.now().isoformat(),
            "generator": "dijkstra-fast-cpp (Cython C++ & Python)",
            "algorithm": "Dijkstra's Algorithm (Min-Heap Priority Queue)"
        },
        "performance_benchmark": timing_info or {},
        "summary": {
            "total_distance": round(total_distance, 4),
            "num_nodes_in_path": len(path_node_ids),
            "num_edges_in_path": len(path_edges),
            "start_node": path_node_ids[0] if path_node_ids else None,
            "end_node": path_node_ids[-1] if path_node_ids else None,
            "waypoints_covered": waypoints
        },
        "path_nodes": path_nodes,
        "path_edges": path_edges,
        "full_graph_metadata": {
            "total_graph_nodes": len(network_nodes),
            "total_graph_edges": len(network_edges)
        }
    }

    json_str = json.dumps(payload, indent=2)

    if output_filepath:
        with open(output_filepath, "w", encoding="utf-8") as f:
            f.write(json_str)

    return json_str
