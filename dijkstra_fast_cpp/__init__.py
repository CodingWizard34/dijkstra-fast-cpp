"""
dijkstra-fast-cpp: High-performance Dijkstra Shortest Path Solver for Map Routing
Comparing Pure Python against Cython-wrapped C++ std::priority_queue with JSON export.
"""

from .dijkstra_pure import dijkstra_python, dijkstra_cover_waypoints_python
from .map_generator import RoadNetwork, generate_random_map, segment_intersection, euclidean_distance
from .json_exporter import export_path_to_json
from .real_map_router import RealWorldMapGraph, LANDMARKS, haversine_distance

try:
    from ._dijkstra_cython import dijkstra_cython, dijkstra_cover_waypoints_cython, FastGraph
    CYTHON_AVAILABLE = True
except ImportError:
    dijkstra_cython = None
    dijkstra_cover_waypoints_cython = None
    FastGraph = None
    CYTHON_AVAILABLE = False

__version__ = "0.1.0"
__all__ = [
    "dijkstra_python",
    "dijkstra_cover_waypoints_python",
    "dijkstra_cython",
    "dijkstra_cover_waypoints_cython",
    "FastGraph",
    "RoadNetwork",
    "generate_random_map",
    "export_path_to_json",
    "CYTHON_AVAILABLE"
]
