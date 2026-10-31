#include "dijkstra.h"
#include <queue>
#include <algorithm>
#include <limits>

static const double INF = std::numeric_limits<double>::infinity();

CppGraph::CppGraph() : num_nodes(0) {}

CppGraph::CppGraph(int n) : num_nodes(n), adj(n) {}

void CppGraph::reset(int n) {
    num_nodes = n;
    adj.clear();
    adj.resize(n);
}

void CppGraph::add_edge(int u, int v, double weight, bool bidirectional) {
    if (u >= 0 && u < num_nodes && v >= 0 && v < num_nodes && weight >= 0.0) {
        adj[u].push_back({ v, weight });
        if (bidirectional && u != v) {
            adj[v].push_back({ u, weight });
        }
    }
}

DijkstraResult CppGraph::shortest_path(int start_node, int target_node) const {
    DijkstraResult result;
    result.total_distance = INF;
    result.reachable = false;

    if (start_node < 0 || start_node >= num_nodes ||
        target_node < 0 || target_node >= num_nodes) {
        return result;
    }

    if (start_node == target_node) {
        result.total_distance = 0.0;
        result.path = { start_node };
        result.reachable = true;
        return result;
    }

    typedef std::pair<double, int> DistNode;
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;

    std::vector<double> dist(num_nodes, INF);
    std::vector<int> parent(num_nodes, -1);

    dist[start_node] = 0.0;
    pq.push({ 0.0, start_node });

    while (!pq.empty()) {
        DistNode top = pq.top();
        pq.pop();

        double d = top.first;
        int u = top.second;

        if (d > dist[u]) {
            continue;
        }

        if (u == target_node) {
            break;
        }

        for (const auto& edge : adj[u]) {
            int v = edge.to;
            double next_dist = d + edge.weight;

            if (next_dist < dist[v]) {
                dist[v] = next_dist;
                parent[v] = u;
                pq.push({ next_dist, v });
            }
        }
    }

    if (dist[target_node] == INF) {
        return result;
    }

    std::vector<int> path;
    for (int curr = target_node; curr != -1; curr = parent[curr]) {
        path.push_back(curr);
    }
    std::reverse(path.begin(), path.end());

    result.total_distance = dist[target_node];
    result.path = std::move(path);
    result.reachable = true;
    return result;
}

DijkstraResult CppGraph::cover_waypoints(const std::vector<int>& waypoints) const {
    DijkstraResult total_result;
    total_result.total_distance = 0.0;
    total_result.reachable = true;

    if (waypoints.empty()) {
        return total_result;
    }

    if (waypoints.size() == 1) {
        total_result.path = { waypoints[0] };
        return total_result;
    }

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        int u = waypoints[i];
        int v = waypoints[i + 1];

        DijkstraResult leg = shortest_path(u, v);
        if (!leg.reachable) {
            total_result.reachable = false;
            total_result.total_distance = INF;
            total_result.path.clear();
            return total_result;
        }

        total_result.total_distance += leg.total_distance;
        if (total_result.path.empty()) {
            total_result.path = leg.path;
        } else {
            total_result.path.insert(total_result.path.end(), leg.path.begin() + 1, leg.path.end());
        }
    }

    return total_result;
}

// Stateless helpers
DijkstraResult dijkstra_cpp(
    int num_nodes,
    const std::vector<int>& from_nodes,
    const std::vector<int>& to_nodes,
    const std::vector<double>& weights,
    int start_node,
    int target_node
) {
    CppGraph graph(num_nodes);
    size_t m = from_nodes.size();
    for (size_t i = 0; i < m; ++i) {
        graph.add_edge(from_nodes[i], to_nodes[i], weights[i], false);
    }
    return graph.shortest_path(start_node, target_node);
}

DijkstraResult dijkstra_cover_waypoints_cpp(
    int num_nodes,
    const std::vector<int>& from_nodes,
    const std::vector<int>& to_nodes,
    const std::vector<double>& weights,
    const std::vector<int>& waypoints
) {
    CppGraph graph(num_nodes);
    size_t m = from_nodes.size();
    for (size_t i = 0; i < m; ++i) {
        graph.add_edge(from_nodes[i], to_nodes[i], weights[i], false);
    }
    return graph.cover_waypoints(waypoints);
}
