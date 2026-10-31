# cython: language_level = 3
from libcpp.vector cimport vector
from libcpp cimport bool

cdef extern from "dijkstra.h":
    struct DijkstraResult:
        double total_distance
        vector[int] path
        bool reachable

    cdef cppclass CppGraph:
        int num_nodes
        CppGraph() except +
        CppGraph(int n) except +
        void reset(int n)
        void add_edge(int u, int v, double weight, bool bidirectional)
        DijkstraResult shortest_path(int start_node, int target_node) const
        DijkstraResult cover_waypoints(const vector[int]& waypoints) const

    DijkstraResult dijkstra_cpp(
        int num_nodes,
        const vector[int]& from_nodes,
        const vector[int]& to_nodes,
        const vector[double]& weights,
        int start_node,
        int target_node
    )

    DijkstraResult dijkstra_cover_waypoints_cpp(
        int num_nodes,
        const vector[int]& from_nodes,
        const vector[int]& to_nodes,
        const vector[double]& weights,
        const vector[int]& waypoints
    )
