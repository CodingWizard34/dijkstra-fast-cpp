# distutils: language = c++
# cython: language_level = 3

from libcpp.vector cimport vector
from libcpp cimport bool
cimport dijkstra_fast_cpp.c_dijkstra as c_dijk

cdef class FastGraph:
    """
    Cython C++ Graph class holding graph adjacency list permanently in C++ memory.
    Enables ultra-fast microsecond Dijkstra queries without paying conversion overhead on every call.
    """
    cdef c_dijk.CppGraph* _cpp_graph

    def __cinit__(self, int num_nodes = 0):
        self._cpp_graph = new c_dijk.CppGraph(num_nodes)

    def __dealloc__(self):
        if self._cpp_graph != NULL:
            del self._cpp_graph

    def reset(self, int num_nodes):
        self._cpp_graph.reset(num_nodes)

    def add_edge(self, int u, int v, double weight, bool bidirectional = False):
        self._cpp_graph.add_edge(u, v, weight, bidirectional)

    def add_edges_from_list(self, list edges, bool bidirectional = False):
        for edge in edges:
            self._cpp_graph.add_edge(<int>edge[0], <int>edge[1], <double>edge[2], bidirectional)

    def shortest_path(self, int start_node, int target_node):
        cdef c_dijk.DijkstraResult res = self._cpp_graph.shortest_path(start_node, target_node)
        cdef list py_path = [node for node in res.path]
        return (res.total_distance, py_path, res.reachable)

    def cover_waypoints(self, list waypoints):
        cdef vector[int] c_wp
        for wp in waypoints:
            c_wp.push_back(<int>wp)
        cdef c_dijk.DijkstraResult res = self._cpp_graph.cover_waypoints(c_wp)
        cdef list py_path = [node for node in res.path]
        return (res.total_distance, py_path, res.reachable)


def dijkstra_cython(int num_nodes, list edges, int start_node, int target_node):
    cdef size_t m = len(edges)
    cdef vector[int] c_from
    cdef vector[int] c_to
    cdef vector[double] c_weights

    c_from.reserve(m)
    c_to.reserve(m)
    c_weights.reserve(m)

    for edge in edges:
        c_from.push_back(<int>edge[0])
        c_to.push_back(<int>edge[1])
        c_weights.push_back(<double>edge[2])

    cdef c_dijk.DijkstraResult res = c_dijk.dijkstra_cpp(
        num_nodes, c_from, c_to, c_weights, start_node, target_node
    )

    cdef list py_path = [node for node in res.path]
    return (res.total_distance, py_path, res.reachable)


def dijkstra_cover_waypoints_cython(int num_nodes, list edges, list waypoints):
    cdef size_t m = len(edges)
    cdef vector[int] c_from
    cdef vector[int] c_to
    cdef vector[double] c_weights
    cdef vector[int] c_waypoints

    c_from.reserve(m)
    c_to.reserve(m)
    c_weights.reserve(m)

    for edge in edges:
        c_from.push_back(<int>edge[0])
        c_to.push_back(<int>edge[1])
        c_weights.push_back(<double>edge[2])

    for wp in waypoints:
        c_waypoints.push_back(<int>wp)

    cdef c_dijk.DijkstraResult res = c_dijk.dijkstra_cover_waypoints_cpp(
        num_nodes, c_from, c_to, c_weights, c_waypoints
    )

    cdef list py_path = [node for node in res.path]
    return (res.total_distance, py_path, res.reachable)
