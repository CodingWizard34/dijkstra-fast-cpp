#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <vector>
#include <limits>
#include <utility>

struct Edge {
    int to;
    double weight;
};

struct DijkstraResult {
    double total_distance;
    std::vector<int> path;
    bool reachable;
};

/**
 * High-performance C++ Graph class holding pre-built adjacency lists in memory.
 * Allows executing instant Dijkstra queries without converting Python edge lists every time.
 */
class CppGraph {
public:
    int num_nodes;
    std::vector<std::vector<Edge>> adj;

    CppGraph();
    CppGraph(int n);
    void reset(int n);
    void add_edge(int u, int v, double weight, bool bidirectional = false);
    
    DijkstraResult shortest_path(int start_node, int target_node) const;
    DijkstraResult cover_waypoints(const std::vector<int>& waypoints) const;
};

// One-shot stateless functions
DijkstraResult dijkstra_cpp(
    int num_nodes,
    const std::vector<int>& from_nodes,
    const std::vector<int>& to_nodes,
    const std::vector<double>& weights,
    int start_node,
    int target_node
);

DijkstraResult dijkstra_cover_waypoints_cpp(
    int num_nodes,
    const std::vector<int>& from_nodes,
    const std::vector<int>& to_nodes,
    const std::vector<double>& weights,
    const std::vector<int>& waypoints
);

#endif // DIJKSTRA_H
