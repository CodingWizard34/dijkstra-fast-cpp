"""
frontend_gui.py: Interactive Desktop GUI (Tkinter) for Dijkstra Map Routing.
Supports both real-world Dehradun road routing (UPES to Premnagar)
and synthetic procedural road-intersection graphs.
"""

import sys
import os
import time
import json
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dijkstra_fast_cpp.real_map_router import RealWorldMapGraph, LANDMARKS, haversine_distance
from dijkstra_fast_cpp.map_generator import RoadNetwork, generate_random_map
from dijkstra_fast_cpp.dijkstra_pure import dijkstra_python, dijkstra_cover_waypoints_python
from dijkstra_fast_cpp.json_exporter import export_path_to_json

try:
    from dijkstra_fast_cpp._dijkstra_cython import dijkstra_cython, FastGraph
    CYTHON_AVAILABLE = True
except ImportError:
    CYTHON_AVAILABLE = False


class DijkstraMapApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dijkstra Real Map Router — UPES to Premnagar")
        self.geometry("1220x800")
        self.configure(bg="#1e1e2e")

        self.real_graph = RealWorldMapGraph()
        self.fast_g = None
        if CYTHON_AVAILABLE and self.real_graph.loaded:
            self.fast_g = FastGraph(len(self.real_graph.osm_id_to_idx))
            self.fast_g.add_edges_from_list(self.real_graph.edges)

        self.mode = "real"  # "real" or "synthetic"
        self.procedural_network = RoadNetwork()
        self.path_coords = []
        self.path_indices = []
        self.total_dist_display = 0.0
        self.timing_info = {}

        self._init_ui()
        self.solve_real_route()

    def _init_ui(self):
        header = tk.Frame(self, bg="#181825", height=55)
        header.pack(fill=tk.X, side=tk.TOP)

        title = tk.Label(
            header,
            text="📍 Dijkstra Real-World Navigation & Intersection Routing",
            font=("Segoe UI", 15, "bold"),
            fg="#cdd6f4",
            bg="#181825"
        )
        title.pack(side=tk.LEFT, padx=20, pady=10)

        status_cy = "Active (Compiled C++)" if CYTHON_AVAILABLE else "Disabled"
        cy_badge = tk.Label(
            header,
            text=f"Cython C++: {status_cy}",
            font=("Segoe UI", 10, "bold"),
            fg="#a6e3a1" if CYTHON_AVAILABLE else "#f38ba8",
            bg="#313244",
            padx=10,
            pady=4
        )
        cy_badge.pack(side=tk.RIGHT, padx=20, pady=12)

        main_frame = tk.Frame(self, bg="#1e1e2e")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)

        # Left: Map Canvas
        canvas_card = tk.Frame(main_frame, bg="#181825", bd=1, relief=tk.SOLID)
        canvas_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(canvas_card, bg="#11111b", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Right: Sidebar
        sidebar = tk.Frame(main_frame, bg="#181825", width=360)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y, padx=(15, 0))
        sidebar.pack_propagate(False)

        # Trip Selection
        lbl_trip = tk.Label(sidebar, text="TRIP LOCATIONS", font=("Segoe UI", 11, "bold"), fg="#89b4fa", bg="#181825")
        lbl_trip.pack(anchor=tk.W, padx=15, pady=(15, 4))

        landmarks = list(LANDMARKS.keys())
        tk.Label(sidebar, text="Origin:", fg="#a6adc8", bg="#181825").pack(anchor=tk.W, padx=15)
        self.origin_var = tk.StringVar(value="UPES Bidholi (Campus)")
        self.origin_combo = ttk.Combobox(sidebar, textvariable=self.origin_var, values=landmarks, state="readonly")
        self.origin_combo.pack(fill=tk.X, padx=15, pady=(2, 8))

        tk.Label(sidebar, text="Destination:", fg="#a6adc8", bg="#181825").pack(anchor=tk.W, padx=15)
        self.dest_var = tk.StringVar(value="Premnagar (Market)")
        self.dest_combo = ttk.Combobox(sidebar, textvariable=self.dest_var, values=landmarks, state="readonly")
        self.dest_combo.pack(fill=tk.X, padx=15, pady=(2, 8))

        # Buttons
        btn_route = tk.Button(sidebar, text="🚀 Calculate Shortest Path (C++)", bg="#89b4fa", fg="#11111b",
                              font=("Segoe UI", 10, "bold"), command=self.solve_real_route, relief=tk.FLAT, pady=6)
        btn_route.pack(fill=tk.X, padx=15, pady=(4, 5))

        btn_export = tk.Button(sidebar, text="💾 Export Path to JSON", bg="#a6e3a1", fg="#11111b",
                               font=("Segoe UI", 10, "bold"), command=self.export_json, relief=tk.FLAT, pady=6)
        btn_export.pack(fill=tk.X, padx=15, pady=5)

        sep = tk.Frame(sidebar, bg="#313244", height=1)
        sep.pack(fill=tk.X, padx=15, pady=12)

        # Performance Section
        lbl_perf = tk.Label(sidebar, text="PERFORMANCE BENCHMARK", font=("Segoe UI", 11, "bold"), fg="#fab387", bg="#181825")
        lbl_perf.pack(anchor=tk.W, padx=15, pady=(0, 6))

        self.lbl_py = tk.Label(sidebar, text="Pure Python: -- ms", font=("Consolas", 10), fg="#cdd6f4", bg="#181825")
        self.lbl_py.pack(anchor=tk.W, padx=15, pady=1)

        self.lbl_cy = tk.Label(sidebar, text="Cython C++: -- ms", font=("Consolas", 10), fg="#cdd6f4", bg="#181825")
        self.lbl_cy.pack(anchor=tk.W, padx=15, pady=1)

        self.lbl_speedup = tk.Label(sidebar, text="Speedup: --", font=("Segoe UI", 11, "bold"), fg="#a6e3a1", bg="#181825")
        self.lbl_speedup.pack(anchor=tk.W, padx=15, pady=(4, 8))

        self.lbl_dist = tk.Label(sidebar, text="Driving Distance: -- km", font=("Segoe UI", 11, "bold"), fg="#cdd6f4", bg="#181825")
        self.lbl_dist.pack(anchor=tk.W, padx=15, pady=(0, 8))

        # Directions Box
        lbl_dir = tk.Label(sidebar, text="TURN-BY-TURN DIRECTIONS", font=("Segoe UI", 11, "bold"), fg="#89b4fa", bg="#181825")
        lbl_dir.pack(anchor=tk.W, padx=15, pady=(6, 4))

        dir_frame = tk.Frame(sidebar, bg="#181825")
        dir_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(0, 10))

        self.dir_text = tk.Text(dir_frame, bg="#11111b", fg="#cdd6f4", font=("Consolas", 9), relief=tk.FLAT, wrap=tk.WORD)
        self.dir_text.pack(fill=tk.BOTH, expand=True)

    def solve_real_route(self):
        if not self.real_graph.loaded:
            return

        origin = self.origin_var.get()
        dest = self.dest_var.get()

        start_pt = LANDMARKS[origin]
        end_pt = LANDMARKS[dest]

        u_idx = self.real_graph.find_nearest_node(start_pt)
        v_idx = self.real_graph.find_nearest_node(end_pt)

        # 1. Pure Python Dijkstra
        t0 = time.perf_counter()
        py_dist, py_path, py_reach = dijkstra_python(
            len(self.real_graph.osm_id_to_idx), self.real_graph.edges, u_idx, v_idx
        )
        py_ms = (time.perf_counter() - t0) * 1000.0

        # 2. Cython C++ FastGraph
        if self.fast_g is not None:
            t1 = time.perf_counter()
            cy_dist, cy_path, cy_reach = self.fast_g.shortest_path(u_idx, v_idx)
            cy_ms = (time.perf_counter() - t1) * 1000.0
            speedup = py_ms / cy_ms if cy_ms > 0 else 1.0

            self.path_indices = cy_path
            dist_km = cy_dist / 1000.0
            self.lbl_cy.config(text=f"Cython C++: {cy_ms:.3f} ms")
            self.lbl_speedup.config(text=f"Speedup: {speedup:.1f}x faster 🚀")
            self.timing_info = {
                "python_ms": round(py_ms, 3),
                "cython_ms": round(cy_ms, 3),
                "speedup": f"{speedup:.1f}x"
            }
        else:
            self.path_indices = py_path
            dist_km = py_dist / 1000.0
            self.lbl_cy.config(text="Cython C++: Not available")
            self.timing_info = {"python_ms": round(py_ms, 3)}

        self.lbl_py.config(text=f"Pure Python: {py_ms:.2f} ms")
        self.lbl_dist.config(text=f"Driving Distance: {dist_km:.2f} km")
        self.total_dist_display = dist_km

        # Populate Directions and Landmarks
        routes = self.real_graph.discover_all_routes(u_idx, v_idx, self.fast_g)
        self.dir_text.delete("1.0", tk.END)

        if routes:
            self.dir_text.insert(tk.END, f"=== EXPLORED ROUTES ({len(routes)} Candidates) ===\n")
            for r in routes:
                status = "🏆 OPTIMAL" if r['is_optimal'] else f"+{r['delta_km']} km"
                self.dir_text.insert(tk.END, f"• {r['name']}\n  Distance: {r['distance_km']} km [{status}]\n\n")

            self.dir_text.insert(tk.END, "=== INTERMEDIATE LANDMARK NODES ===\n")
            for lm in routes[0]['intermediate_landmarks']:
                self.dir_text.insert(tk.END, f"📍 {lm['name']}\n   Node ID: #{lm['node_id']} | GPS: ({lm['lat']}, {lm['lon']})\n   At: {lm['distance_from_start_km']} km from start\n\n")

            self.dir_text.insert(tk.END, "=== TURN-BY-TURN DIRECTIONS ===\n")
            for idx, s in enumerate(routes[0]['directions']):
                self.dir_text.insert(tk.END, f"{idx + 1}. {s['instruction']}\n   -> {s['distance_km']} km (towards {s['heading']})\n")

        self.draw_real_map(start_pt, end_pt)

    def draw_real_map(self, start_pt, end_pt):
        self.canvas.delete("all")
        w = max(400, self.canvas.winfo_width()) if self.canvas.winfo_width() > 1 else 780
        h = max(300, self.canvas.winfo_height()) if self.canvas.winfo_height() > 1 else 660

        # Bounding Box of Dehradun network
        min_lat, max_lat = 30.32, 30.43
        min_lon, max_lon = 77.925, 77.995

        def to_canvas(lat, lon):
            x = (lon - min_lon) / (max_lon - min_lon) * (w - 60) + 30
            y = (max_lat - lat) / (max_lat - min_lat) * (h - 60) + 30
            return x, y

        # Draw road network segments
        drawn_edges = set()
        for u_idx, v_idx, _ in self.real_graph.edges:
            pair = (min(u_idx, v_idx), max(u_idx, v_idx))
            if pair not in drawn_edges:
                drawn_edges.add(pair)
                p1 = self.real_graph.get_node_coord(pair[0])
                p2 = self.real_graph.get_node_coord(pair[1])
                x1, y1 = to_canvas(p1[0], p1[1])
                x2, y2 = to_canvas(p2[0], p2[1])
                self.canvas.create_line(x1, y1, x2, y2, fill="#2a2b3d", width=1.2)

        # Draw Shortest Path
        if len(self.path_indices) >= 2:
            pts = []
            for nid in self.path_indices:
                lat, lon = self.real_graph.get_node_coord(nid)
                pts.extend(to_canvas(lat, lon))
            self.canvas.create_line(pts, fill="#00E5FF", width=4, joinstyle=tk.ROUND)

        # Draw Landmarks
        for name, coord in LANDMARKS.items():
            cx, cy = to_canvas(coord[0], coord[1])
            is_start = (name == self.origin_var.get())
            is_end = (name == self.dest_var.get())
            if is_start:
                self.canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#00E676", outline="#ffffff", width=2)
                self.canvas.create_text(cx + 12, cy, text=f"Start: {name}", fill="#00E676", font=("Segoe UI", 9, "bold"), anchor=tk.W)
            elif is_end:
                self.canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#FF1744", outline="#ffffff", width=2)
                self.canvas.create_text(cx + 12, cy, text=f"End: {name}", fill="#FF1744", font=("Segoe UI", 9, "bold"), anchor=tk.W)
            else:
                self.canvas.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#89b4fa", outline="#181825", width=1)
                self.canvas.create_text(cx + 8, cy, text=name, fill="#6c7086", font=("Segoe UI", 8), anchor=tk.W)

    def export_json(self):
        if not self.path_indices:
            messagebox.showwarning("No Path", "No path computed.")
            return

        origin = self.origin_var.get()
        dest = self.dest_var.get()
        directions = self.real_graph.generate_directions(self.path_indices)
        path_coords = [self.real_graph.get_node_coord(nid) for nid in self.path_indices]

        payload = {
            "origin": origin,
            "destination": dest,
            "total_distance_km": round(self.total_dist_display, 3),
            "num_nodes_in_path": len(self.path_indices),
            "performance_benchmark": self.timing_info,
            "turn_by_turn_directions": directions,
            "path_gps_coordinates": [{"lat": lat, "lon": lon} for lat, lon in path_coords]
        }

        default_name = f"route_{origin.split()[0]}_to_{dest.split()[0]}.json"
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json")],
            initialfile=default_name,
            title="Save Route JSON"
        )

        if filepath:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            messagebox.showinfo("Export Successful", f"Route exported to:\n{filepath}")


if __name__ == "__main__":
    app = DijkstraMapApp()
    app.mainloop()
