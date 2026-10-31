# dijkstra-fast-cpp: High-Performance Dijkstra Map Routing & Intersection Solver

A production-grade implementation of **Dijkstra's Shortest Path Algorithm** comparing pure Python against compiled C++ wrapped with Cython. Includes procedural road network generation, geometric line intersection detection, multi-waypoint path solving, interactive Python frontends (Desktop GUI & Streamlit Web), and JSON path export.

---

## 🚀 Key Features

1. **Dual Solver Engine**:
   - **Pure Python Baseline**: Uses Python's standard `heapq` module and dynamic lists.
   - **Compiled C++ Engine (`FastGraph`)**: Implements `std::priority_queue` with cache-coherent adjacency vectors and early-exit destination targeting.
   - **Speedup**: Up to **~20x–120x faster** on large graph topologies.

2. **Map & Intersection Generation**:
   - Simulates arterial roads and crossing streets across a 2D map coordinate plane.
   - Computes exact segment-segment geometric intersections using vector cross-products.
   - Subdivides roads at each crossing into a connected planar graph of nodes `[x, y]` and weighted edges `(u, v, distance)`.
   - Generates random waypoints and calculates the optimal route covering all designated points.

3. **JSON Path Export**:
   - Exports the traversed nodes, geographic/pixel coordinates, edge distances, timing benchmarks, and complete graph metadata into a standardized JSON format.

4. **Dual Interactive Python Frontends**:
   - **Desktop GUI (`frontend_gui.py`)**: Native Tkinter window with clickable canvas, real-time visual route rendering, performance metrics, and save-file dialog.
   - **Web Dashboard (`frontend_streamlit.py`)**: Modern browser UI with interactive sliders, Matplotlib dark-mode map plotting, real-time speedup badges, and one-click JSON download.

---

## 📊 Benchmark Results (Intel 12 Threads, Windows 11, Python 3.14)

Measured using `timeit` across multiple sampling passes on random connected graphs:

| Graph Nodes (V) | Graph Edges (E) | Pure Python (ms) | Cython FastGraph C++ (ms) | Speedup Factor |
|:---------------:|:---------------:|:----------------:|:-------------------------:|:--------------:|
| **1,000**       | 4,998           | 0.94 ms          | 0.007 ms                  | **127.2x**     |
| **5,000**       | 24,994          | 13.78 ms         | 0.686 ms                  | **20.1x**      |
| **20,000**      | 99,994          | 57.57 ms         | 2.064 ms                  | **27.9x**      |
| **50,000**      | 249,996         | 357.63 ms        | 22.777 ms                 | **15.7x**      |

---

## 📦 Quick Start & Usage

### 1. Installation

#### From PyPI:
```bash
pip install dijkstra-fast-cpp
```

#### From Local Source:
```bash
cd dijkstra_router
python setup.py build_ext --inplace
```

### 2. Python API Usage

```python
from dijkstra_fast_cpp import FastGraph, dijkstra_python, generate_random_map, export_path_to_json

# 1. Generate a procedural road map with intersections
network = generate_random_map(width=800, height=600, num_roads=12, num_waypoints=4, seed=42)

print(f"Generated {len(network.nodes)} intersections and {len(network.edges)} road edges.")

# 2. Fast C++ Dijkstra Query
graph = FastGraph(len(network.nodes))
graph.add_edges_from_list(network.edges)

# Calculate shortest path covering waypoints
total_dist, path, reachable = graph.cover_waypoints(network.waypoints)
print(f"Shortest path distance: {total_dist:.2f} px")
print(f"Path nodes: {path}")

# 3. Export path to JSON
json_output = export_path_to_json(
    network_nodes=network.nodes,
    network_edges=network.edges,
    path_node_ids=path,
    total_distance=total_dist,
    waypoints=network.waypoints,
    output_filepath="shortest_path.json"
)
```

---

## 🖥️ Running the Frontends

### Option A: Desktop GUI (Tkinter)
```bash
python frontend_gui.py
```
*Click anywhere on the map to add custom waypoints, drag sliders, and click "Export Path to JSON".*

### Option B: Streamlit Web Dashboard
```bash
streamlit run frontend_streamlit.py
```

---

## 🧪 Running Tests & Benchmarks

```bash
# Run unit tests
python -m unittest discover -s tests -v

# Run performance benchmark
python benchmark.py
```

---

## 📄 License
MIT License.
