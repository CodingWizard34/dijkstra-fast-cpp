"""
map_generator.py: Generates procedural road maps, finds geometric intersections
of lines, constructs planar road network graphs, and places random waypoints.
"""

import math
import random
from typing import List, Tuple, Dict, Any, Optional


def segment_intersection(p1: Tuple[float, float], p2: Tuple[float, float],
                         p3: Tuple[float, float], p4: Tuple[float, float],
                         eps: float = 1e-6) -> Optional[Tuple[float, float]]:
    """
    Finds the intersection point of two 2D line segments p1-p2 and p3-p4.
    Returns (x, y) if they intersect strictly or at endpoints, otherwise None.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < eps:
        return None  # Parallel or collinear

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    if -eps <= t <= 1.0 + eps and -eps <= u <= 1.0 + eps:
        # Clamp t to [0, 1] for numerical stability
        t_clamped = max(0.0, min(1.0, t))
        ix = x1 + t_clamped * (x2 - x1)
        iy = y1 + t_clamped * (y2 - y1)
        return (round(ix, 4), round(iy, 4))

    return None


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


class RoadNetwork:
    """
    Represents a 2D planar road graph derived from intersecting roads/lines.
    """
    def __init__(self):
        self.nodes: List[Dict[str, Any]] = []          # [{'id': int, 'x': float, 'y': float}]
        self.node_lookup: Dict[Tuple[float, float], int] = {} # (x, y) -> node_id
        self.edges: List[Tuple[int, int, float]] = []  # [(u, v, weight)]
        self.waypoints: List[int] = []                 # list of node_ids to cover
        self.road_segments: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

    def get_or_create_node(self, x: float, y: float, tolerance: float = 1.0) -> int:
        """Find existing node within tolerance or create a new node."""
        for node in self.nodes:
            if math.hypot(node['x'] - x, node['y'] - y) <= tolerance:
                return node['id']

        node_id = len(self.nodes)
        self.nodes.append({'id': node_id, 'x': x, 'y': y})
        self.node_lookup[(x, y)] = node_id
        return node_id

    def add_edge(self, u: int, v: int, bidirectional: bool = True):
        if u == v:
            return
        p1 = (self.nodes[u]['x'], self.nodes[u]['y'])
        p2 = (self.nodes[v]['x'], self.nodes[v]['y'])
        w = round(euclidean_distance(p1, p2), 4)

        # Avoid duplicate parallel edges
        for existing_u, existing_v, _ in self.edges:
            if (existing_u == u and existing_v == v) or (existing_u == v and existing_v == u):
                return

        self.edges.append((u, v, w))
        if bidirectional:
            self.edges.append((v, u, w))

    def build_from_roads(self, roads: List[Tuple[Tuple[float, float], Tuple[float, float]]]):
        """
        Takes raw road lines, computes all pairwise intersections,
        and constructs a fully connected intersection graph.
        """
        self.road_segments = list(roads)
        self.nodes.clear()
        self.node_lookup.clear()
        self.edges.clear()

        # For each road, collect all points: endpoints + intersections with all other roads
        road_points: List[List[Tuple[float, float]]] = [[] for _ in roads]

        for i, (p1, p2) in enumerate(roads):
            road_points[i].append(p1)
            road_points[i].append(p2)

        n_roads = len(roads)
        for i in range(n_roads):
            for j in range(i + 1, n_roads):
                inter = segment_intersection(roads[i][0], roads[i][1], roads[j][0], roads[j][1])
                if inter is not None:
                    road_points[i].append(inter)
                    road_points[j].append(inter)

        # Sort points along each road segment and create edges between consecutive points
        for i, points in enumerate(road_points):
            p_start, p_end = roads[i]
            # Projection factor along vector (p_end - p_start)
            dx = p_end[0] - p_start[0]
            dy = p_end[1] - p_start[1]
            length_sq = dx * dx + dy * dy

            if length_sq < 1e-8:
                continue

            def project_t(pt):
                return ((pt[0] - p_start[0]) * dx + (pt[1] - p_start[1]) * dy) / length_sq

            # Remove duplicates and sort by distance along road
            unique_points = []
            for pt in points:
                if not any(euclidean_distance(pt, u_pt) < 1.0 for u_pt in unique_points):
                    unique_points.append(pt)

            unique_points.sort(key=project_t)

            # Create nodes and connect consecutive points
            node_ids = [self.get_or_create_node(pt[0], pt[1]) for pt in unique_points]
            for k in range(len(node_ids) - 1):
                self.add_edge(node_ids[k], node_ids[k + 1], bidirectional=True)


def generate_random_map(width: float = 800.0, height: float = 600.0,
                        num_roads: int = 14, num_waypoints: int = 4,
                        seed: Optional[int] = 42) -> RoadNetwork:
    """
    Generates a procedural map with random crossing roads, computes all intersections,
    and places random waypoints to be covered by Dijkstra.
    """
    if seed is not None:
        random.seed(seed)

    network = RoadNetwork()
    roads = []

    margin = 40.0
    # Generate horizontal-ish and vertical-ish and diagonal arterial roads
    half = num_roads // 2
    for _ in range(half):
        # East-West wavy/diagonal road
        y1 = random.uniform(margin, height - margin)
        y2 = random.uniform(margin, height - margin)
        roads.append(((margin, y1), (width - margin, y2)))

    for _ in range(num_roads - half):
        # North-South wavy/diagonal road
        x1 = random.uniform(margin, width - margin)
        x2 = random.uniform(margin, width - margin)
        roads.append(((x1, margin), (x2, height - margin)))

    network.build_from_roads(roads)

    # Select random waypoints on the graph to cover
    if len(network.nodes) > 0:
        wp_count = min(num_waypoints, len(network.nodes))
        # Pick well-spaced nodes as waypoints
        sampled_ids = random.sample(range(len(network.nodes)), wp_count)
        network.waypoints = sampled_ids

    return network
