"""
frontend_streamlit.py: Web-based interactive dashboard for Dijkstra Map Routing.
Explores all possible routes between actual locations (e.g. UPES to Premnagar),
ranks them by distance, identifies the most optimal shortest path, and highlights
all intermediate landmark nodes with coordinates.
"""

import sys
import os
import time
import json
import streamlit as st
import streamlit.components.v1 as components

# Ensure local package path is importable
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


st.set_page_config(
    page_title="Dijkstra Route Exploration — UPES to Premnagar",
    page_icon="🗺️",
    layout="wide"
)

# Custom CSS styling
st.markdown("""
<style>
    .optimal-badge {
        background-color: #a6e3a1;
        color: #11111b;
        padding: 4px 10px;
        border-radius: 12px;
        font-weight: bold;
        font-size: 0.85em;
        display: inline-block;
    }
    .alt-badge {
        background-color: #fab387;
        color: #11111b;
        padding: 4px 10px;
        border-radius: 12px;
        font-weight: bold;
        font-size: 0.85em;
        display: inline-block;
    }
    .landmark-row {
        background-color: #181825;
        border-left: 4px solid #89b4fa;
        padding: 8px 12px;
        margin-bottom: 6px;
        border-radius: 6px;
        font-size: 0.9em;
    }
</style>
""", unsafe_allow_html=True)

st.title("🗺️ Dijkstra Multi-Route Explorer & Optimal Path Solver")
st.caption("Explores all candidate corridors, identifies intermediate landmark nodes, and calculates the most optimal path on Dehradun Road Network")

# Mode selector
mode = st.sidebar.radio(
    "Routing Mode",
    ["📍 Real-World Map (UPES to Premnagar & Dehradun)", "🎲 Procedural Line-Intersection Grid"]
)

# -------------------------------------------------------------
# MODE 1: REAL-WORLD MAP (UPES TO PREMNAGAR)
# -------------------------------------------------------------
if "Real-World" in mode:
    st.sidebar.header("📍 Trip Locations")

    landmarks = list(LANDMARKS.keys())
    origin_name = st.sidebar.selectbox("Start Location", landmarks, index=0) # UPES Bidholi
    dest_name = st.sidebar.selectbox("Destination", landmarks, index=7)     # Premnagar

    if origin_name == dest_name:
        st.warning("⚠️ Origin and Destination are identical. Please select different locations.")
        st.stop()

    # Cache graph
    @st.cache_resource
    def load_real_graph():
        g = RealWorldMapGraph()
        fg = None
        if CYTHON_AVAILABLE and g.loaded:
            fg = FastGraph(len(g.osm_id_to_idx))
            fg.add_edges_from_list(g.edges)
        return g, fg

    real_graph, c_graph = load_real_graph()

    if not real_graph.loaded:
        st.error("OSM road network could not be loaded. Please check data/dehradun_roads.json.")
        st.stop()

    start_idx = real_graph.find_nearest_node(LANDMARKS[origin_name])
    end_idx = real_graph.find_nearest_node(LANDMARKS[dest_name])

    # Discover all candidate routes
    routes = real_graph.discover_all_routes(start_idx, end_idx, c_graph)

    if not routes:
        st.error("No route found between the selected locations.")
        st.stop()

    optimal_route = routes[0]

    # --- TOP METRIC BANNER ---
    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Routes Explored", f"{len(routes)} Candidates")
    m2.metric("Optimal Distance", f"{optimal_route['distance_km']:.2f} km", delta="Shortest Path")
    m3.metric("Cython C++ Query Time", f"{optimal_route['cython_time_ms']:.3f} ms", delta=f"{optimal_route['speedup']} vs Python")
    m4.metric("Road Nodes in Graph", f"{len(real_graph.osm_id_to_idx):,}")

    st.markdown("---")

    # Main columns: Left Map, Right Route comparison & intermediate locations
    col_map, col_details = st.columns([2.2, 1.3])

    with col_details:
        st.subheader("📋 Explored Routes Comparison")
        st.write("Dijkstra explores alternative network corridors to guarantee finding the most optimal path:")

        # Route Selection Radio
        route_options = [
            f"{'🏆 ' if r['is_optimal'] else '🔄 '}{r['name']} — {r['distance_km']} km ({'+0.0 km' if r['is_optimal'] else '+' + str(r['delta_km']) + ' km'})"
            for r in routes
        ]
        selected_route_str = st.radio("Select Route to Inspect:", route_options, index=0)
        selected_idx = route_options.index(selected_route_str)
        active_route = routes[selected_idx]

        st.markdown(f"**Description**: {active_route['description']}")

        # Intermediate Locations Along Selected Route
        st.subheader("📍 Important Intermediate Locations (Nodes)")
        st.caption(f"Landmark checkpoints identified along {active_route['name'].split(':')[0]}:")

        landmarks_passed = active_route["intermediate_landmarks"]
        if landmarks_passed:
            with st.container(height=240):
                for lm in landmarks_passed:
                    st.markdown(
                        f"<div class='landmark-row'>"
                        f"<b>{lm['name']}</b><br>"
                        f"<span style='color: #a6adc8;'>Node ID: <code>#{lm['node_id']}</code> | GPS: <code>{lm['lat']:.4f}, {lm['lon']:.4f}</code></span>"
                        f"<span style='color: #a6e3a1; float: right;'>{lm['distance_from_start_km']} km</span>"
                        f"</div>",
                        unsafe_allow_html=True
                    )
        else:
            st.info("Direct route without intermediate landmarks.")

        # Turn-by-Turn Directions
        with st.expander("🧭 View Turn-by-Turn Directions"):
            for idx, s in enumerate(active_route["directions"]):
                st.write(f"**{idx + 1}.** {s['instruction']} — `{s['distance_km']} km` ({s['heading']})")

        # JSON Export Button
        export_payload = {
            "origin": origin_name,
            "destination": dest_name,
            "explored_routes_count": len(routes),
            "optimal_shortest_route": {
                "route_name": optimal_route["name"],
                "distance_km": optimal_route["distance_km"],
                "node_count": optimal_route["node_count"],
                "cython_time_ms": optimal_route["cython_time_ms"],
                "python_time_ms": optimal_route["python_time_ms"],
                "speedup": optimal_route["speedup"]
            },
            "all_explored_routes": [
                {
                    "route_id": r["id"],
                    "name": r["name"],
                    "distance_km": r["distance_km"],
                    "delta_from_optimal_km": r["delta_km"],
                    "is_optimal": r["is_optimal"],
                    "node_count": r["node_count"],
                    "intermediate_landmarks": r["intermediate_landmarks"],
                    "path_gps_coordinates": [{"lat": lat, "lon": lon} for lat, lon in r["gps_coordinates"]]
                }
                for r in routes
            ]
        }
        json_export_str = json.dumps(export_payload, indent=2)

        st.download_button(
            label="💾 Download All Routes & Landmarks (JSON)",
            data=json_export_str,
            file_name=f"all_routes_{origin_name.split()[0]}_to_{dest_name.split()[0]}.json",
            mime="application/json",
            use_container_width=True
        )

    with col_map:
        st.subheader("🗺️ All Explored Routes on OpenStreetMap")
        st.caption("🏆 **Cyan (Solid)** = Most Optimal Shortest Path | 🟧 **Orange / 🟪 Purple (Dashed)** = Alternative Routes")

        # Construct Leaflet Map showing ALL routes simultaneously
        start_coord = LANDMARKS[origin_name]
        end_coord = LANDMARKS[dest_name]
        center_lat = (start_coord[0] + end_coord[0]) / 2.0
        center_lon = (start_coord[1] + end_coord[1]) / 2.0

        routes_js_data = []
        for r in routes:
            routes_js_data.append({
                "id": r["id"],
                "name": r["name"],
                "distance_km": r["distance_km"],
                "is_optimal": r["is_optimal"],
                "color": r["color"],
                "dash": r["dash_array"],
                "weight": 6 if r["id"] == active_route["id"] else 3.5,
                "opacity": 0.95 if r["id"] == active_route["id"] else 0.65,
                "coords": [[lat, lon] for lat, lon in r["gps_coordinates"]]
            })

        # Landmark checkpoints for pins
        pins_data = []
        for lm in active_route["intermediate_landmarks"]:
            pins_data.append({
                "name": lm["name"],
                "node_id": lm["node_id"],
                "lat": lm["lat"],
                "lon": lm["lon"],
                "km": lm["distance_from_start_km"]
            })

        leaflet_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <style>
                html, body {{ height: 100%; margin: 0; padding: 0; background: #11111b; }}
                #map {{ width: 100%; height: 600px; border-radius: 8px; }}
                .leaflet-popup-content-wrapper {{ background: #181825; color: #cdd6f4; font-family: sans-serif; }}
                .leaflet-popup-tip {{ background: #181825; }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map').setView([{center_lat}, {center_lon}], 12);
                
                L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                    maxZoom: 19,
                    attribution: '© OpenStreetMap contributors'
                }}).addTo(map);

                var allRoutes = {json.dumps(routes_js_data)};
                var boundsGroup = new L.featureGroup();

                // Draw all candidate routes
                allRoutes.forEach(function(r) {{
                    var polyOptions = {{
                        color: r.color,
                        weight: r.weight,
                        opacity: r.opacity,
                        lineJoin: 'round'
                    }};
                    if (r.dash) {{
                        polyOptions.dashArray = r.dash;
                    }}
                    var poly = L.polyline(r.coords, polyOptions).addTo(map);
                    poly.bindPopup("<b>" + r.name + "</b><br>Distance: <b>" + r.distance_km + " km</b><br>" + (r.is_optimal ? "<span style='color:#00E676;'>★ Shortest Optimal Path</span>" : "Alternative Route"));
                    boundsGroup.addLayer(poly);
                }});

                // Draw intermediate landmark checkpoint pins
                var pins = {json.dumps(pins_data)};
                pins.forEach(function(p, i) {{
                    var isStart = (i === 0);
                    var isEnd = (i === pins.length - 1);
                    var pinColor = isStart ? '#00E676' : (isEnd ? '#FF1744' : '#FFD600');
                    var marker = L.circleMarker([p.lat, p.lon], {{
                        radius: (isStart || isEnd) ? 9 : 6,
                        fillColor: pinColor,
                        color: '#ffffff',
                        weight: 2,
                        fillOpacity: 1
                    }}).addTo(map);
                    marker.bindPopup("<b>" + p.name + "</b><br>Node ID: <code>#" + p.node_id + "</code><br>Distance from Start: <b>" + p.km + " km</b><br>GPS: " + p.lat + ", " + p.lon);
                }});

                map.fitBounds(boundsGroup.getBounds(), {{ padding: [30, 30] }});
            </script>
        </body>
        </html>
        """

        components.html(leaflet_html, height=620)

# -------------------------------------------------------------
# MODE 2: PROCEDURAL LINE-INTERSECTION GRID
# -------------------------------------------------------------
else:
    st.sidebar.header("⚙️ Procedural Settings")
    seed = st.sidebar.number_input("Random Seed", min_value=1, max_value=99999, value=42, step=1)
    num_roads = st.sidebar.slider("Number of Arterial Roads", min_value=6, max_value=28, value=14, step=2)
    num_waypoints = st.sidebar.slider("Waypoints to Cover", min_value=2, max_value=8, value=4, step=1)

    network = generate_random_map(width=800, height=600, num_roads=num_roads, num_waypoints=num_waypoints, seed=seed)

    col_map, col_metrics = st.columns([2.5, 1.2])

    with col_metrics:
        st.subheader("⚡ Performance Benchmark")
        n = len(network.nodes)
        edges = network.edges
        wps = network.waypoints

        t0 = time.perf_counter()
        py_dist, py_path, py_reach = dijkstra_cover_waypoints_python(n, edges, wps)
        py_time_ms = (time.perf_counter() - t0) * 1000.0

        if CYTHON_AVAILABLE:
            fast_g = FastGraph(n)
            fast_g.add_edges_from_list(edges)
            t1 = time.perf_counter()
            cy_dist, cy_path, cy_reach = fast_g.cover_waypoints(wps)
            cy_time_ms = (time.perf_counter() - t1) * 1000.0
            speedup = py_time_ms / cy_time_ms if cy_time_ms > 0 else 1.0

            st.metric("Pure Python (heapq)", f"{py_time_ms:.3f} ms")
            st.metric("Cython C++ (FastGraph)", f"{cy_time_ms:.3f} ms", delta=f"{speedup:.1f}x faster")
            st.metric("Total Path Distance", f"{cy_dist:.2f} px")
            active_path = cy_path
            active_dist = cy_dist
            timing_data = {"python_ms": round(py_time_ms, 3), "cython_ms": round(cy_time_ms, 3), "speedup": f"{speedup:.1f}x"}
        else:
            st.metric("Pure Python (heapq)", f"{py_time_ms:.3f} ms")
            st.metric("Total Path Distance", f"{py_dist:.2f} px")
            active_path = py_path
            active_dist = py_dist
            timing_data = {"python_ms": round(py_time_ms, 3)}

        st.caption(f"Generated: **{len(network.nodes)}** intersections, **{len(network.edges)}** road edges")

        json_out = export_path_to_json(
            network_nodes=network.nodes,
            network_edges=network.edges,
            path_node_ids=active_path,
            total_distance=active_dist,
            waypoints=network.waypoints,
            timing_info=timing_data
        )

        st.download_button(
            label="💾 Download Path as JSON",
            data=json_out,
            file_name=f"procedural_dijkstra_path_{seed}.json",
            mime="application/json",
            use_container_width=True
        )

    with col_map:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 7.5), facecolor='#11111b')
        ax.set_facecolor('#181825')

        for p1, p2 in network.road_segments:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#313244', linewidth=2.5, zorder=1)

        node_map = {node['id']: (node['x'], node['y']) for node in network.nodes}
        for u, v, _ in network.edges:
            if u < v:
                p1 = node_map[u]
                p2 = node_map[v]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#45475a', linewidth=1.2, linestyle=':', zorder=2)

        xs = [node['x'] for node in network.nodes]
        ys = [node['y'] for node in network.nodes]
        ax.scatter(xs, ys, color='#89b4fa', s=25, zorder=3, alpha=0.8, edgecolors='#181825')

        if len(active_path) >= 2:
            path_x = [node_map[nid][0] for nid in active_path]
            path_y = [node_map[nid][1] for nid in active_path]
            ax.plot(path_x, path_y, color='#a6e3a1', linewidth=4.5, zorder=4, label='Dijkstra Path')

        for idx, wp_id in enumerate(network.waypoints):
            wx, wy = node_map[wp_id]
            color = '#f9e2af' if idx == 0 else ('#f38ba8' if idx == len(network.waypoints) - 1 else '#fab387')
            ax.scatter([wx], [wy], color=color, s=160, zorder=5, edgecolors='#11111b', linewidth=2)
            ax.text(wx, wy, str(idx + 1), color='#11111b', fontsize=10, fontweight='bold', ha='center', va='center', zorder=6)

        ax.set_xlim(-10, 810)
        ax.set_ylim(-10, 610)
        ax.invert_yaxis()
        ax.axis('off')
        st.pyplot(fig)
