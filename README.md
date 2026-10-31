# IteroPath: High-Performance C++ Dijkstra Navigation Engine (`dijkstra-fast-cpp`)

[![PyPI version](https://img.shields.io/pypi/v/dijkstra-fast-cpp.svg?color=blue)](https://pypi.org/project/dijkstra-fast-cpp/)
[![Python versions](https://img.shields.io/pypi/pyversions/dijkstra-fast-cpp.svg)](https://pypi.org/project/dijkstra-fast-cpp/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A production-grade, high-performance **Dijkstra Navigation Engine** implemented in **native C++** and exposed via **Cython**, providing an empirical performance comparison against pure Python. Includes real OpenStreetMap road routing across Greater Dehradun (19,765 nodes, 37,399 edges), multi-corridor discovery, procedural line-intersection grids, multi-waypoint tours, verifiable numerical parity, and an interactive Streamlit navigation dashboard.

---

## Performance Overview & Verification Dashboard

IteroPath delivers **28x to 77x faster execution** on real-world arterial road networks and up to **120x+ speedup** on synthetic dense topologies, with **0.000000000 m exact distance parity**.

![IteroPath Performance Dashboard](results/overall_performance.png)

---

## Key Features

- **High-Performance C++ Core (`FastGraph`)**:
  - Implements cache-coherent contiguous adjacency lists (`std::vector<Edge>`) and a min-heap priority queue (`std::priority_queue`).
  - Zero-copy graph traversal and instant early exit upon target vertex resolution.
  - Releases Python's Global Interpreter Lock (`nogil`) for true multi-threaded parallel query execution.
- **Pure Python Baseline**:
  - Fully independent pure Python Dijkstra using `heapq` and dynamic dictionaries for transparent, verifiable baseline comparison.
- **Real-World Road Network Navigation (Dehradun)**:
  - 19,765 road nodes and 37,399 directed edges extracted from OpenStreetMap.
  - Multi-corridor route discovery with natural street name synthesis (`Bidholi Connector Road`, `Pondha-Bidholi Road`, `Chakrata Main Road`).
  - Turn-by-turn navigation instructions with cardinal bearings and segment distances.
- **Ordered Multi-Stop Waypoint Tours**:
  - Concatenates shortest path segments across arbitrary ordered waypoints with zero path discontinuity.
- **Exact Floating-Point Verification**:
  - Verifies distance agreement between Python and Cython C++ down to machine precision ($\Delta = 0.000000000\text{ m}$).
- **Interactive Web Interface (Streamlit + Leaflet)**:
  - Live interactive maps, corridor exploration, real-time speedup meters, downloadable JSON route exports, and built-in benchmark visualization charts.

---

## Benchmark Results

All benchmarks were executed on an **Intel(R) Core(TM) i5 (12-thread architecture), Windows 11 64-bit, Python 3.14.0, GCC 14.2.0 (`-O3 -march=native -ffast-math`)**.

### 1. Real-World Dehradun Road Network (19,765 Nodes)

Route latency and speedup across major arterial corridors starting from UPES Bidholi Campus:

| Corridor / Route | Stops | Distance (km) | Graph Nodes | Pure Python (ms) | Cython C++ (ms) | Speedup Multiplier | Distance Agreement ($\Delta$) |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **UPES to Pondha** | 2 | 5.58 km | 295 | 7.66 ms | 0.107 ms | **71.6x** | `0.000000000 m` |
| **UPES to Dunga** | 2 | 10.99 km | 394 | 9.97 ms | 0.148 ms | **67.2x** | `0.000000000 m` |
| **UPES to Manduwala** | 2 | 13.35 km | 458 | 12.46 ms | 0.207 ms | **60.1x** | `0.000000000 m` |
| **UPES to Premnagar** | 2 | 11.84 km | 454 | 9.34 ms | 0.135 ms | **69.2x** | `0.000000000 m` |
| **UPES to Kaulagarh** | 2 | 12.68 km | 590 | 13.14 ms | 0.171 ms | **77.0x** | `0.000000000 m` |
| **UPES to Clock Tower** | 2 | 15.50 km | 682 | 11.93 ms | 0.402 ms | **29.7x** | `0.000000000 m` |
| **UPES to Raipur Road** | 2 | 20.87 km | 850 | 18.04 ms | 0.649 ms | **27.8x** | `0.000000000 m` |
| **Grand Tour (5 Waypoints)** | 5 | 32.71 km | 1,064 | 50.58 ms | 1.082 ms | **46.7x** | `0.000000000 m` |

![Real-World Corridor Comparison](results/route_comparison.png)

---

### 2. Scalability on Increasing Graph Topologies ($V = 1,000$ to $50,000$)

Tested on random intersection graphs with uniform degree $k = 4$:

| Graph Nodes ($V$) | Graph Edges ($E$) | Pure Python (ms) | Cython C++ (ms) | Speedup Multiplier | Cython Throughput (QPS) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **1,000** | 4,996 | 1.10 ms | 0.041 ms | **26.8x** | **24,462 queries/s** |
| **2,500** | 12,494 | 3.70 ms | 0.221 ms | **16.7x** | **4,519 queries/s** |
| **5,000** | 24,998 | 7.13 ms | 0.300 ms | **23.8x** | **3,334 queries/s** |
| **10,000** | 49,996 | 21.15 ms | 0.575 ms | **36.8x** | **1,739 queries/s** |
| **20,000** | 99,996 | 53.94 ms | 2.547 ms | **21.2x** | **393 queries/s** |
| **35,000** | 174,992 | 131.68 ms | 6.196 ms | **21.3x** | **161 queries/s** |
| **50,000** | 249,996 | 202.43 ms | 9.049 ms | **22.4x** | **111 queries/s** |

---

### 3. Detailed Performance Charts

#### Execution Latency (Logarithmic Scale)
![Execution Time](results/execution_time.png)

#### Speedup Multipliers across Topologies & Waypoints
![Speedup Multipliers](results/speedup.png)

#### Algorithmic Scaling vs Theoretical Complexity $O((V+E)\log V)$
![Synthetic Scaling](results/synthetic_scaling.png)

---

## Why is IteroPath C++ Faster than Python?

| Factor | Pure Python Dijkstra (`heapq`) | IteroPath Cython C++ (`FastGraph`) |
|:---|:---|:---|
| **Memory Allocation** | Dynamic `PyObject` wrappers per edge and node | Flat contiguous `std::vector` buffers |
| **Heap Operations** | Allocates new `tuple` on every heap push | Zero-allocation `std::priority_queue` in-place push |
| **Pointer Overhead** | Multiple pointer dereferences via Python heap | Cache-coherent CPU L1/L2 cache hit optimization |
| **GIL Limitation** | Bound to single Python thread by GIL | Releases GIL (`nogil`) for native multi-threaded scalability |
| **Numerical Deviation** | IEEE 754 64-bit float | IEEE 754 64-bit `double` (**$\Delta = 0.000000000\text{ m}$**) |

---

## Installation

### From PyPI
```bash
pip install dijkstra-fast-cpp
```

### From Source
```bash
git clone https://github.com/CodingWizard34/dijkstra-fast-cpp.git
cd dijkstra-fast-cpp
python setup.py build_ext --inplace
```

---

## Quickstart & API Usage

### 1. High-Performance C++ Dijkstra Query
```python
from dijkstra_fast_cpp import FastGraph

# Initialize graph with 4 nodes
graph = FastGraph(4)
graph.add_edge(0, 1, 4.2)
graph.add_edge(1, 2, 2.1)
graph.add_edge(2, 3, 3.5)
graph.add_edge(0, 2, 8.0)

# Single-pair shortest path
dist, path, reachable = graph.shortest_path(0, 3)
print(f"Distance: {dist:.2f} m | Path: {path}")
# Output: Distance: 9.80 m | Path: [0, 1, 2, 3]
```

### 2. Multi-Stop Waypoint Tour
```python
# Traversal covering waypoints [0 -> 2 -> 3]
total_dist, tour_path, ok = graph.cover_waypoints([0, 2, 3])
print(f"Tour Distance: {total_dist:.2f} m | Trajectory: {tour_path}")
```

### 3. OpenStreetMap Real-World Routing
```python
from dijkstra_fast_cpp.real_map_router import RealWorldMapGraph, LANDMARKS

# Load Greater Dehradun road network
road_graph = RealWorldMapGraph()

# Find nearest network nodes
start_node = road_graph.find_nearest_node(LANDMARKS["UPES Bidholi (Campus)"])
end_node = road_graph.find_nearest_node(LANDMARKS["Clock Tower (Ghanta Ghar)"])

# Discover optimal route and alternative corridors
routes = road_graph.discover_all_routes(start_node, end_node, max_alternatives=3)
optimal = routes[0]
print(f"Optimal Corridor: {optimal['name']} ({optimal['distance_km']} km)")
```

---

## Running the Interactive Web App

Launch the Streamlit navigation interface with interactive Leaflet map and benchmark visualizations:

```bash
streamlit run frontend_streamlit.py
```

---

## Technical Documentation

- [Algorithmic Design & Complexity](docs/algorithm.md)
- [System Architecture & Data Flow](docs/architecture.md)
- [Verified Benchmark Results](docs/results.md)

---

## License

MIT License. See [LICENSE](LICENSE) for details.
