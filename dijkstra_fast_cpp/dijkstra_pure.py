"""
Pure Python implementation of Dijkstra's Shortest Path algorithm using heapq.
Used as the baseline to benchmark against the compiled Cython C++ solver.
"""

import heapq
import math


def dijkstra_python(num_nodes: int, edges: list, start_node: int, target_node: int):
    """
    Pure Python Dijkstra shortest path using heapq min-heap priority queue.

    Parameters:
        num_nodes (int): Total number of nodes in graph (0 to num_nodes - 1).
        edges (list of tuple): List of (u, v, weight) edges.
        start_node (int): Source vertex ID.
        target_node (int): Destination vertex ID.

    Returns:
        tuple: (total_distance, path, is_reachable)
    """
    if start_node < 0 or start_node >= num_nodes or target_node < 0 or target_node >= num_nodes:
        return (math.inf, [], False)

    if start_node == target_node:
        return (0.0, [start_node], True)

    # Build adjacency list
    adj = [[] for _ in range(num_nodes)]
    for u, v, w in edges:
        if 0 <= u < num_nodes and 0 <= v < num_nodes and w >= 0:
            adj[u].append((v, float(w)))

    dist = [math.inf] * num_nodes
    parent = [-1] * num_nodes

    dist[start_node] = 0.0
    pq = [(0.0, start_node)]

    while pq:
        d, u = heapq.heappop(pq)

        if d > dist[u]:
            continue

        if u == target_node:
            break

        for v, weight in adj[u]:
            next_dist = d + weight
            if next_dist < dist[v]:
                dist[v] = next_dist
                parent[v] = u
                heapq.heappush(pq, (next_dist, v))

    if math.isinf(dist[target_node]):
        return (math.inf, [], False)

    # Reconstruct path
    path = []
    curr = target_node
    while curr != -1:
        path.append(curr)
        curr = parent[curr]
    path.reverse()

    return (dist[target_node], path, True)


def dijkstra_cover_waypoints_python(num_nodes: int, edges: list, waypoints: list):
    """
    Pure Python shortest path traversing a sequence of waypoints in order.

    Parameters:
        num_nodes (int): Total nodes.
        edges (list of tuple): (u, v, weight).
        waypoints (list of int): Sequence of node IDs to visit in order.

    Returns:
        tuple: (total_distance, path, is_reachable)
    """
    if not waypoints:
        return (0.0, [], True)
    if len(waypoints) == 1:
        return (0.0, [waypoints[0]], True)

    total_dist = 0.0
    full_path = []

    for i in range(len(waypoints) - 1):
        u = waypoints[i]
        v = waypoints[i + 1]
        leg_dist, leg_path, reachable = dijkstra_python(num_nodes, edges, u, v)

        if not reachable:
            return (math.inf, [], False)

        total_dist += leg_dist
        if not full_path:
            full_path = list(leg_path)
        else:
            full_path.extend(leg_path[1:])

    return (total_dist, full_path, True)
