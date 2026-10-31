"""
benchmark.py: Comprehensive performance comparison between Pure Python Dijkstra
and Cython C++ std::priority_queue Dijkstra across varying graph topologies.
"""

import sys
import os
import math
import random
import timeit

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dijkstra_fast_cpp.dijkstra_pure import dijkstra_python
from dijkstra_fast_cpp._dijkstra_cython import dijkstra_cython, FastGraph


def generate_synthetic_graph(num_nodes: int, avg_degree: int = 4, seed: int = 42):
    rng = random.Random(seed)
    edges = []

    for i in range(num_nodes - 1):
        w = round(rng.uniform(1.0, 50.0), 2)
        edges.append((i, i + 1, w))
        edges.append((i + 1, i, w))

    extra_edges = int(num_nodes * (avg_degree - 1) / 2)
    for _ in range(extra_edges):
        u = rng.randint(0, num_nodes - 1)
        v = rng.randint(0, num_nodes - 1)
        if u != v:
            w = round(rng.uniform(1.0, 50.0), 2)
            edges.append((u, v, w))
            edges.append((v, u, w))

    return edges


def run_benchmark():
    test_sizes = [
        {"nodes": 1000,   "runs": 20},
        {"nodes": 5000,   "runs": 10},
        {"nodes": 20000,  "runs": 5},
        {"nodes": 50000,  "runs": 3},
    ]

    print("=" * 95)
    print("DIJKSTRA BENCHMARK: Pure Python (heapq) vs Cython C++ (std::priority_queue)")
    print("=" * 95)

    results = []

    for item in test_sizes:
        n = item["nodes"]
        runs = item["runs"]

        edges = generate_synthetic_graph(n, avg_degree=4, seed=100 + n)
        start_node = 0
        target_node = n - 1

        # Pre-build C++ FastGraph
        fast_g = FastGraph(n)
        fast_g.add_edges_from_list(edges)

        # Correctness check
        py_dist, py_path, py_reach = dijkstra_python(n, edges, start_node, target_node)
        cy_dist, cy_path, cy_reach = fast_g.shortest_path(start_node, target_node)

        assert py_reach == cy_reach, f"Reachability mismatch at N={n}"
        if py_reach:
            assert abs(py_dist - cy_dist) < 1e-4, f"Distance mismatch at N={n}: Py={py_dist}, Cy={cy_dist}"

        # 1. Pure Python Benchmark
        py_timer = timeit.Timer(lambda: dijkstra_python(n, edges, start_node, target_node))
        py_ms = (py_timer.timeit(number=runs) / runs) * 1000.0

        # 2. Cython Pre-loaded FastGraph Benchmark (pure query execution)
        fg_runs = runs * 5
        fg_timer = timeit.Timer(lambda: fast_g.shortest_path(start_node, target_node))
        fg_ms = (fg_timer.timeit(number=fg_runs) / fg_runs) * 1000.0

        speedup = py_ms / fg_ms if fg_ms > 0 else float("inf")

        results.append({
            "nodes": n,
            "edges": len(edges),
            "py_ms": py_ms,
            "fg_ms": fg_ms,
            "speedup": speedup
        })

        print(f"Nodes: {n:6,d} | Edges: {len(edges):8,d} | "
              f"Python: {py_ms:8.2f} ms | FastGraph C++: {fg_ms:6.3f} ms | Speedup: {speedup:5.1f}x")

    print("\n" + "=" * 95)
    print("BENCHMARK SUMMARY TABLE (Markdown)")
    print("=" * 95)
    print("| Graph Nodes (V) | Graph Edges (E) | Pure Python (ms) | FastGraph C++ (ms) | Speedup Factor |")
    print("|:---------------:|:---------------:|:----------------:|:------------------:|:--------------:|")
    for r in results:
        print(f"| {r['nodes']:<15,d} | {r['edges']:<15,d} | {r['py_ms']:<16.2f} | {r['fg_ms']:<18.3f} | {r['speedup']:<14.1f}x |")
    print("=" * 95)

    return results


if __name__ == "__main__":
    run_benchmark()
