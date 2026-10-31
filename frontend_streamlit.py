"""
frontend_streamlit.py: Interactive web interface for Dijkstra Map Routing and Corridor Analysis.
Compares pure Python against compiled C++ (Cython) across procedural grids and real OpenStreetMap road networks.
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
    page_title="IteroPath — C++ Dijkstra Navigation",
    page_icon="🧭",
    layout="wide"
)

# Refined Professional Styling
st.markdown("""
<style>
    /* Global Typography & Background */
    html, body, [class*="css"] {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }
    
    /* Subtle Metric Cards */
    div[data-testid="stMetric"] {
        background-color: #161922;
        border: 1px solid #232734;
        border-radius: 8px;
        padding: 14px 18px;
    }
    div[data-testid="stMetricLabel"] {
        font-size: 0.82rem;
        font-weight: 500;
        color: #94a3b8;
        text-transform: uppercase;
        letter-spacing: 0.05em;
    }
    div[data-testid="stMetricValue"] {
        font-size: 1.6rem;
        font-weight: 700;
        color: #f1f5f9;
    }
    div[data-testid="stMetricDelta"] {
        font-size: 0.85rem;
    }

    /* Landmark Checkpoint Cards */
    .checkpoint-card {
        background-color: #161922;
        border: 1px solid #232734;
        border-left: 3px solid #38bdf8;
        padding: 10px 14px;
        margin-bottom: 8px;
        border-radius: 6px;
    }
    .checkpoint-title {
        font-weight: 600;
        color: #f8fafc;
        font-size: 0.95rem;
    }
    .checkpoint-meta {
        color: #64748b;
        font-size: 0.82rem;
        font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
        margin-top: 3px;
    }
    .checkpoint-dist {
        color: #38bdf8;
        font-weight: 600;
        float: right;
        font-size: 0.9rem;
    }

    /* Badges */
    .badge-optimal {
        background: #064e3b;
        color: #34d399;
        font-size: 0.75rem;
        font-weight: 600;
        padding: 2px 8px;
        border-radius: 4px;
        border: 1px solid #059669;
        margin-left: 6px;
    }

    /* Expander clean styling */
    div[data-testid="stExpander"] {
        border: 1px solid #232734;
        border-radius: 8px;
        background-color: #12151d;
    }
</style>
""", unsafe_allow_html=True)

# Top Header
st.title("IteroPath")
st.caption("High-Performance C++ Dijkstra Navigation and Corridor Engine (Cython vs Python)")

# Mode selector
mode = st.sidebar.radio(
    "Routing Mode",
    ["Real-World Road Network (Dehradun)", "Procedural Line-Intersection Grid"]
)

# -------------------------------------------------------------
# MODE 1: REAL-WORLD ROAD NETWORK
# -------------------------------------------------------------
if "Real-World" in mode:
    st.sidebar.subheader("Route Configuration")

    landmarks = list(LANDMARKS.keys())
    
    trip_type = st.sidebar.radio(
        "Navigation Type",
        ["Direct Trip (Multi-Corridor)", "Multi-Stop Waypoint Tour"]
    )

    # Cache graph loading
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
        st.error("Road network data could not be loaded. Please verify data/dehradun_roads.json.")
        st.stop()

    if "Multi-Stop" in trip_type:
        st.sidebar.markdown("**Planned Stops:**")
        default_stops = [
            "UPES Bidholi (Campus)",
            "Kaulagarh (ONGC / FRI Area)",
            "Premnagar (Market)",
            "Clock Tower (Ghanta Ghar)",
            "Raipur Road (Ordnance / Raipur)"
        ]
        selected_stops = st.sidebar.multiselect("Stops in visit order", landmarks, default=default_stops)

        if len(selected_stops) < 2:
            st.warning("Select at least 2 stops for a waypoint tour.")
            st.stop()

        stop_nodes = [real_graph.find_nearest_node(LANDMARKS[s]) for s in selected_stops]

        # Solve via Cython C++ FastGraph
        t0 = time.perf_counter()
        if c_graph is not None:
            total_d, full_path, reach = c_graph.cover_waypoints(stop_nodes)
        else:
            total_d, full_path, reach = dijkstra_cover_waypoints_python(len(real_graph.osm_id_to_idx), real_graph.edges, stop_nodes)
        cy_time = (time.perf_counter() - t0) * 1000.0

        # Pure Python benchmark
        t1 = time.perf_counter()
        py_d, py_p, _ = dijkstra_cover_waypoints_python(len(real_graph.osm_id_to_idx), real_graph.edges, stop_nodes)
        py_time = (time.perf_counter() - t1) * 1000.0
        speedup_str = f"{max(1.0, py_time / max(0.01, cy_time)):.1f}x"

        if not reach or len(full_path) < 2:
            st.error("Could not compute continuous road paths connecting the selected stops.")
            st.stop()

        tot_km = total_d / 1000.0
        path_coords = [real_graph.get_node_coord(nid) for nid in full_path]
        directions = real_graph.generate_directions(full_path)
        landmarks_passed = real_graph.find_intermediate_landmarks_along_path(full_path)

        tour_title = " ➔ ".join([s.split()[0] for s in selected_stops])
        routes = [{
            "id": 1,
            "name": f"Waypoint Tour ({tour_title})",
            "description": f"Chained Dijkstra trajectory covering {len(selected_stops)} stops across Dehradun.",
            "distance_km": round(tot_km, 2),
            "delta_km": 0.0,
            "is_optimal": True,
            "color": "#00E5FF",
            "dash_array": None,
            "node_count": len(full_path),
            "path_indices": full_path,
            "gps_coordinates": path_coords,
            "intermediate_landmarks": landmarks_passed,
            "directions": directions,
            "python_time_ms": round(py_time, 3),
            "cython_time_ms": round(cy_time, 3),
            "speedup": speedup_str
        }]
        origin_name = selected_stops[0]
        dest_name = selected_stops[-1]

    else:
        origin_name = st.sidebar.selectbox("Start Location", landmarks, index=0)
        default_dest_idx = landmarks.index("Raipur Road (Ordnance / Raipur)") if "Raipur Road (Ordnance / Raipur)" in landmarks else len(landmarks) - 1
        dest_name = st.sidebar.selectbox("Destination", landmarks, index=default_dest_idx)
        num_routes_to_find = st.sidebar.slider(
            "Corridors to Explore",
            min_value=1,
            max_value=9,
            value=5,
            step=1,
            help="Discovers alternative road corridors, valley connectors, and bypasses."
        )

        if origin_name == dest_name:
            st.warning("Origin and destination must be different.")
            st.stop()

        start_idx = real_graph.find_nearest_node(LANDMARKS[origin_name])
        end_idx = real_graph.find_nearest_node(LANDMARKS[dest_name])

        routes = real_graph.discover_all_routes(start_idx, end_idx, c_graph, max_alternatives=num_routes_to_find)

    if not routes:
        st.error("No route found between the selected locations.")
        st.stop()

    optimal_route = routes[0]

    # Metrics Summary Bar
    m1, m2, m3, m4 = st.columns(4)
    m1.metric("Explored Corridors", f"{len(routes)}")
    m2.metric("Optimal Distance", f"{optimal_route['distance_km']:.2f} km")
    m3.metric("C++ Query Time", f"{optimal_route['cython_time_ms']:.3f} ms", delta=f"{optimal_route['speedup']} vs Python")
    m4.metric("Road Graph Size", f"{len(real_graph.osm_id_to_idx):,} nodes")

    st.markdown("<div style='margin-bottom: 12px;'></div>", unsafe_allow_html=True)

    col_map, col_details = st.columns([2.3, 1.2])

    with col_details:
        st.subheader("Route Selection")

        route_options = [
            f"{'[Optimal] ' if r['is_optimal'] else ''}{r['name']} — {r['distance_km']} km ({'+0.0 km' if r['is_optimal'] else '+' + str(r['delta_km']) + ' km'})"
            for r in routes
        ]
        selected_route_str = st.radio("Select Corridor to Inspect", route_options, index=0, label_visibility="collapsed")
        selected_idx = route_options.index(selected_route_str)
        active_route = routes[selected_idx]

        st.caption(f"**Path profile**: {active_route['description']}")

        # Intermediate Locations Along Selected Route
        st.subheader("Intermediate Checkpoints")
        landmarks_passed = active_route["intermediate_landmarks"]
        if landmarks_passed:
            with st.container(height=250):
                for lm in landmarks_passed:
                    st.markdown(
                        f"<div class='checkpoint-card'>"
                        f"<span class='checkpoint-dist'>{lm['distance_from_start_km']} km</span>"
                        f"<div class='checkpoint-title'>{lm['name']}</div>"
                        f"<div class='checkpoint-meta'>Node #{lm['node_id']} · {lm['lat']:.4f}, {lm['lon']:.4f}</div>"
                        f"</div>",
                        unsafe_allow_html=True
                    )
        else:
            st.info("Direct route segment without intermediate landmark junctions.")

        # Turn-by-Turn Directions
        with st.expander("Turn-by-Turn Directions"):
            for idx, s in enumerate(active_route["directions"]):
                st.write(f"**{idx + 1}.** {s['instruction']} — `{s['distance_km']} km` ({s['heading']})")

        # JSON Export
        export_payload = {
            "origin": origin_name,
            "destination": dest_name,
            "corridors_count": len(routes),
            "optimal_route": {
                "name": optimal_route["name"],
                "distance_km": optimal_route["distance_km"],
                "node_count": optimal_route["node_count"],
                "cython_time_ms": optimal_route["cython_time_ms"],
                "python_time_ms": optimal_route["python_time_ms"],
                "speedup": optimal_route["speedup"]
            },
            "routes": [
                {
                    "id": r["id"],
                    "name": r["name"],
                    "distance_km": r["distance_km"],
                    "delta_km": r["delta_km"],
                    "is_optimal": r["is_optimal"],
                    "node_count": r["node_count"],
                    "intermediate_landmarks": r["intermediate_landmarks"],
                    "coordinates": [{"lat": lat, "lon": lon} for lat, lon in r["gps_coordinates"]]
                }
                for r in routes
            ]
        }
        json_export_str = json.dumps(export_payload, indent=2)

        st.download_button(
            label="Export Route Data (JSON)",
            data=json_export_str,
            file_name=f"routes_{origin_name.split()[0]}_to_{dest_name.split()[0]}.json",
            mime="application/json",
            use_container_width=True
        )

    with col_map:
        st.subheader("OpenStreetMap Route View")
        st.caption("Cyan: Primary shortest path · Dashed Colors: Alternative corridors")

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
                "weight": 5.5 if r["id"] == active_route["id"] else 3.2,
                "opacity": 0.95 if r["id"] == active_route["id"] else 0.55,
                "coords": [[lat, lon] for lat, lon in r["gps_coordinates"]]
            })

        pins_data = [{
            "name": f"Start: {origin_name}",
            "node_id": active_route["path_indices"][0],
            "lat": start_coord[0],
            "lon": start_coord[1],
            "km": 0.0,
            "type": "start"
        }]
        for lm in active_route["intermediate_landmarks"]:
            if lm["name"] != origin_name and lm["name"] != dest_name:
                pins_data.append({
                    "name": lm["name"],
                    "node_id": lm["node_id"],
                    "lat": lm["lat"],
                    "lon": lm["lon"],
                    "km": lm["distance_from_start_km"],
                    "type": "checkpoint"
                })
        pins_data.append({
            "name": f"Destination: {dest_name}",
            "node_id": active_route["path_indices"][-1],
            "lat": end_coord[0],
            "lon": end_coord[1],
            "km": active_route["distance_km"],
            "type": "end"
        })

        leaflet_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <style>
                html, body {{ height: 100%; margin: 0; padding: 0; background: #0f1117; }}
                #map {{ width: 100%; height: 600px; border-radius: 8px; border: 1px solid #232734; }}
                .leaflet-popup-content-wrapper {{ background: #161922; color: #f1f5f9; font-family: sans-serif; border: 1px solid #2d3345; border-radius: 6px; }}
                .leaflet-popup-tip {{ background: #161922; }}
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
                    poly.bindPopup("<b>" + r.name + "</b><br>Distance: <b>" + r.distance_km + " km</b><br>" + (r.is_optimal ? "<span style='color:#34d399; font-weight:600;'>Optimal Shortest Path</span>" : "<span style='color:#94a3b8;'>Alternative Corridor</span>"));
                    boundsGroup.addLayer(poly);
                }});

                var pins = {json.dumps(pins_data)};
                pins.forEach(function(p) {{
                    var pinColor = p.type === 'start' ? '#10b981' : (p.type === 'end' ? '#ef4444' : '#f59e0b');
                    var marker = L.circleMarker([p.lat, p.lon], {{
                        radius: (p.type === 'start' || p.type === 'end') ? 8 : 5,
                        fillColor: pinColor,
                        color: '#ffffff',
                        weight: 2,
                        fillOpacity: 1
                    }}).addTo(map);
                    marker.bindPopup("<b>" + p.name + "</b><br>Node ID: <code>#" + p.node_id + "</code><br>Distance: <b>" + p.km + " km</b><br>Coordinates: " + p.lat + ", " + p.lon);
                    boundsGroup.addLayer(marker);
                }});

                map.fitBounds(boundsGroup.getBounds(), {{ padding: [35, 35] }});
            </script>
        </body>
        </html>
        """

        components.html(leaflet_html, height=620)

# -------------------------------------------------------------
# MODE 2: PROCEDURAL LINE-INTERSECTION GRID
# -------------------------------------------------------------
else:
    st.sidebar.subheader("Grid Configuration")
    seed = st.sidebar.number_input("Random Seed", min_value=1, max_value=99999, value=42, step=1)
    num_roads = st.sidebar.slider("Arterial Roads", min_value=6, max_value=28, value=14, step=2)
    num_waypoints = st.sidebar.slider("Waypoints to Traverse", min_value=2, max_value=8, value=4, step=1)

    network = generate_random_map(width=800, height=600, num_roads=num_roads, num_waypoints=num_waypoints, seed=seed)

    col_map, col_metrics = st.columns([2.5, 1.2])

    with col_metrics:
        st.subheader("Execution Benchmark")
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

        st.caption(f"Graph topology: {len(network.nodes)} nodes, {len(network.edges)} edges")

        json_out = export_path_to_json(
            network_nodes=network.nodes,
            network_edges=network.edges,
            path_node_ids=active_path,
            total_distance=active_dist,
            waypoints=network.waypoints,
            timing_info=timing_data
        )

        st.download_button(
            label="Export Path (JSON)",
            data=json_out,
            file_name=f"procedural_dijkstra_path_{seed}.json",
            mime="application/json",
            use_container_width=True
        )

    with col_map:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 7.5), facecolor='#0f1117')
        ax.set_facecolor('#161922')

        for p1, p2 in network.road_segments:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#2d3345', linewidth=2.2, zorder=1)

        node_map = {node['id']: (node['x'], node['y']) for node in network.nodes}
        for u, v, _ in network.edges:
            if u < v:
                p1 = node_map[u]
                p2 = node_map[v]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#3a4258', linewidth=1.0, linestyle=':', zorder=2)

        xs = [node['x'] for node in network.nodes]
        ys = [node['y'] for node in network.nodes]
        ax.scatter(xs, ys, color='#60a5fa', s=24, zorder=3, alpha=0.85, edgecolors='#161922')

        if len(active_path) >= 2:
            path_x = [node_map[nid][0] for nid in active_path]
            path_y = [node_map[nid][1] for nid in active_path]
            ax.plot(path_x, path_y, color='#34d399', linewidth=4.0, zorder=4, label='Dijkstra Path')

        for idx, wp_id in enumerate(network.waypoints):
            wx, wy = node_map[wp_id]
            color = '#fbbf24' if idx == 0 else ('#f87171' if idx == len(network.waypoints) - 1 else '#fb923c')
            ax.scatter([wx], [wy], color=color, s=150, zorder=5, edgecolors='#0f1117', linewidth=2)
            ax.text(wx, wy, str(idx + 1), color='#0f1117', fontsize=9, fontweight='bold', ha='center', va='center', zorder=6)

        ax.set_xlim(-10, 810)
        ax.set_ylim(-10, 610)
        ax.invert_yaxis()
        ax.axis('off')
        st.pyplot(fig)
