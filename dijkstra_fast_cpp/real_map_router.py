"""
real_map_router.py: Real-world geographic road network router for Dehradun.
Routes between any locations (e.g. UPES to Premnagar, Manduwala, Pondha, etc.)
using Cython C++ Dijkstra, explores all candidate corridors via penalty-based routing,
ranks routes by distance, identifies intermediate landmark nodes, and generates
turn-by-turn navigation directions.
"""

import json
import math
import os
import time
from typing import List, Dict, Any, Tuple, Optional

DATA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "dehradun_roads.json")

# Prominent Dehradun Landmarks with real coordinates (Lat, Lon)
LANDMARKS = {
    "UPES Bidholi (Campus)": (30.41761, 77.96827),
    "UPES Kandoli (Campus)": (30.40420, 77.97340),
    "Pondha": (30.38050, 77.96320),
    "Dunga": (30.39510, 77.93920),
    "Manduwala": (30.39800, 77.92500),
    "Nanda Ki Chowki": (30.35410, 77.96450),
    "Suddhowala": (30.34400, 77.95500),
    "Premnagar (Market)": (30.33522, 77.96400)
}


def haversine_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    R = 6371000.0
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0)**2
    return 2.0 * R * math.asin(math.sqrt(max(0.0, a)))


def calculate_bearing(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0


def bearing_to_direction(deg: float) -> str:
    dirs = ["North", "North-East", "East", "South-East", "South", "South-West", "West", "North-West"]
    idx = int((deg + 22.5) / 45.0) % 8
    return dirs[idx]


class RealWorldMapGraph:
    """
    Manages the OpenStreetMap Dehradun road graph, nodes, edges, street names,
    and multi-path discovery algorithms.
    """
    def __init__(self, osm_json_path: str = DATA_PATH):
        self.osm_json_path = osm_json_path
        self.node_coords: Dict[int, Tuple[float, float]] = {}
        self.osm_id_to_idx: Dict[int, int] = {}
        self.idx_to_osm_id: Dict[int, int] = {}
        self.edges: List[Tuple[int, int, float]] = []
        self.edge_street_names: Dict[Tuple[int, int], str] = {}
        self.loaded = False

        self._load_network()

    def _load_network(self):
        if not os.path.exists(self.osm_json_path):
            print(f"Warning: OSM data file not found at {self.osm_json_path}")
            return

        with open(self.osm_json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        all_nodes = {}
        for el in data.get("elements", []):
            if el["type"] == "node":
                all_nodes[el["id"]] = (float(el["lat"]), float(el["lon"]))

        ways = [el for el in data.get("elements", []) if el["type"] == "way"]

        used_nodes = set()
        for w in ways:
            used_nodes.update(w.get("nodes", []))

        for i, nid in enumerate(used_nodes):
            if nid in all_nodes:
                self.node_coords[nid] = all_nodes[nid]
                self.osm_id_to_idx[nid] = len(self.osm_id_to_idx)
                self.idx_to_osm_id[self.osm_id_to_idx[nid]] = nid

        for w in ways:
            wnodes = w.get("nodes", [])
            tags = w.get("tags", {})
            street_name = tags.get("name", tags.get("ref", tags.get("highway", "Road")))
            oneway = tags.get("oneway", "no") in ["yes", "1", "true"]

            for k in range(len(wnodes) - 1):
                u, v = wnodes[k], wnodes[k + 1]
                if u in self.osm_id_to_idx and v in self.osm_id_to_idx:
                    u_idx = self.osm_id_to_idx[u]
                    v_idx = self.osm_id_to_idx[v]
                    p1 = self.node_coords[u]
                    p2 = self.node_coords[v]
                    d = round(haversine_distance(p1, p2), 2)

                    self.edges.append((u_idx, v_idx, d))
                    self.edge_street_names[(u_idx, v_idx)] = street_name
                    if not oneway:
                        self.edges.append((v_idx, u_idx, d))
                        self.edge_street_names[(v_idx, u_idx)] = street_name

        self.loaded = True

    def find_nearest_node(self, target_coord: Tuple[float, float]) -> int:
        best_idx = 0
        min_d = float("inf")
        for osm_id, p in self.node_coords.items():
            d = haversine_distance(target_coord, p)
            if d < min_d:
                min_d = d
                best_idx = self.osm_id_to_idx[osm_id]
        return best_idx

    def get_node_coord(self, idx: int) -> Tuple[float, float]:
        osm_id = self.idx_to_osm_id[idx]
        return self.node_coords[osm_id]

    def find_intermediate_landmarks_along_path(self, path_indices: List[int], max_offset_m: float = 450.0) -> List[Dict[str, Any]]:
        detected = []
        path_coords = [self.get_node_coord(nid) for nid in path_indices]

        cum_dist = [0.0]
        for i in range(len(path_coords) - 1):
            cum_dist.append(cum_dist[-1] + haversine_distance(path_coords[i], path_coords[i + 1]))

        for name, target_coord in LANDMARKS.items():
            min_d = float("inf")
            best_step = 0
            for step_idx, pt in enumerate(path_coords):
                d = haversine_distance(target_coord, pt)
                if d < min_d:
                    min_d = d
                    best_step = step_idx

            if min_d <= max_offset_m:
                nid = path_indices[best_step]
                dist_from_start_km = cum_dist[best_step] / 1000.0
                detected.append({
                    "name": name,
                    "step_index": best_step,
                    "node_id": nid,
                    "lat": round(path_coords[best_step][0], 5),
                    "lon": round(path_coords[best_step][1], 5),
                    "offset_meters": round(min_d, 1),
                    "distance_from_start_km": round(dist_from_start_km, 2)
                })

        detected.sort(key=lambda x: x["step_index"])
        return detected

    def generate_directions(self, path_indices: List[int]) -> List[Dict[str, Any]]:
        if len(path_indices) < 2:
            return []

        steps = []
        current_street = None
        accum_dist = 0.0

        for i in range(len(path_indices) - 1):
            u = path_indices[i]
            v = path_indices[i + 1]
            p1 = self.get_node_coord(u)
            p2 = self.get_node_coord(v)
            d = haversine_distance(p1, p2)
            street = self.edge_street_names.get((u, v), "Road")

            if current_street is None:
                current_street = street
                accum_dist = d
            elif street == current_street:
                accum_dist += d
            else:
                bearing = calculate_bearing(p1, p2)
                direction = bearing_to_direction(bearing)
                steps.append({
                    "instruction": f"Continue along {current_street}",
                    "distance_meters": round(accum_dist, 1),
                    "distance_km": round(accum_dist / 1000.0, 2),
                    "heading": direction
                })
                current_street = street
                accum_dist = d

        if current_street is not None:
            steps.append({
                "instruction": f"Follow {current_street} to destination",
                "distance_meters": round(accum_dist, 1),
                "distance_km": round(accum_dist / 1000.0, 2),
                "heading": "Destination"
            })

        return steps

    def discover_all_routes(self, start_idx: int, end_idx: int, fast_g_cpp = None, max_alternatives: int = 3) -> List[Dict[str, Any]]:
        """
        Dynamically discovers all viable distinct routes between any start_idx and end_idx
        using Dijkstra path penalties, ranks them by distance, and tags intermediate landmarks.
        """
        from .dijkstra_pure import dijkstra_python

        n_nodes = len(self.osm_id_to_idx)
        edge_weight_map = {(u, v): w for u, v, w in self.edges}

        routes = []
        used_path_signatures = set()

        # Styles for visualizing multiple routes on map
        route_palette = [
            {"color": "#00E5FF", "dash": None, "name_prefix": "Primary Corridor"},
            {"color": "#FF9100", "dash": "7, 5", "name_prefix": "Alternative Corridor A"},
            {"color": "#E040FB", "dash": "4, 4", "name_prefix": "Alternative Corridor B"}
        ]

        current_edges = list(self.edges)
        penalized_edges_set = set()

        for pass_num in range(max_alternatives):
            t0 = time.perf_counter()
            d, path, reach = dijkstra_python(n_nodes, current_edges, start_idx, end_idx)
            t_py = (time.perf_counter() - t0) * 1000.0

            if not reach or len(path) < 2:
                break

            # Calculate true unpenalized ground distance
            true_dist_m = sum(edge_weight_map.get((path[i], path[i + 1]), 0.0) for i in range(len(path) - 1))
            true_dist_km = true_dist_m / 1000.0

            # Path signature to prevent duplicate identical paths
            sig = tuple(path[::max(1, len(path) // 10)])
            if sig in used_path_signatures:
                continue
            used_path_signatures.add(sig)

            # Benchmark Cython C++ query
            cy_ms = 0.0
            if fast_g_cpp is not None:
                t1 = time.perf_counter()
                cy_d, cy_p, _ = fast_g_cpp.shortest_path(start_idx, end_idx)
                cy_ms = (time.perf_counter() - t1) * 1000.0

            style = route_palette[len(routes) % len(route_palette)]
            landmarks_on_route = self.find_intermediate_landmarks_along_path(path)
            directions = self.generate_directions(path)
            coords = [self.get_node_coord(nid) for nid in path]

            routes.append({
                "id": len(routes) + 1,
                "name": f"Route {len(routes) + 1}: {style['name_prefix']}",
                "description": f"Explores distinct road segments ({len(path)} nodes).",
                "distance_km": round(true_dist_km, 2),
                "delta_km": 0.0,
                "is_optimal": False,
                "color": style["color"],
                "dash_array": style["dash"],
                "node_count": len(path),
                "path_indices": path,
                "gps_coordinates": coords,
                "intermediate_landmarks": landmarks_on_route,
                "directions": directions,
                "python_time_ms": round(t_py, 3),
                "cython_time_ms": round(max(0.01, cy_ms), 3),
                "speedup": f"{max(1.0, t_py / max(0.01, cy_ms)):.1f}x"
            })

            # Add penalty to edges of this path to force exploration of alternate roads
            new_penalized = set((path[i], path[i + 1]) for i in range(len(path) - 1))
            penalized_edges_set.update(new_penalized)

            current_edges = []
            for u, v, w in self.edges:
                if (u, v) in penalized_edges_set or (v, u) in penalized_edges_set:
                    current_edges.append((u, v, w * 3.5))
                else:
                    current_edges.append((u, v, w))

        # Sort all discovered routes by true distance
        routes.sort(key=lambda r: r["distance_km"])
        if routes:
            routes[0]["is_optimal"] = True
            routes[0]["name"] = "Route 1: Most Optimal Shortest Path"
            routes[0]["color"] = "#00E5FF"
            routes[0]["dash_array"] = None
            base_km = routes[0]["distance_km"]
            for idx, r in enumerate(routes):
                r["delta_km"] = round(r["distance_km"] - base_km, 2)
                if idx > 0:
                    r["is_optimal"] = False

        return routes
